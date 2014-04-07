/*
 * Copyright 2011 Dog Hunter SA
 * Author: Davide Ciminaghi <ciminaghi@gnudd.com>
 *
 * GNU GPLv2 or later
 */

/* mcuio host controller driver */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/regmap.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/kthread.h>
#include <linux/mutex.h>

#include <linux/mcuio.h>
#include <linux/mcuio_ids.h>
#include <linux/mcuio-proto.h>

#include "mcuio-internal.h"
#include "mcuio-hc.h"

/* Max number of read descr timeout before skipping to next device */
#define MAX_ENUM_RETRIES 2

struct mcuio_request;

typedef void (*___request_cb)(struct mcuio_request *);

struct mcuio_deferred_register_element {
	struct mcuio_request r;
	struct list_head list;
};

/* Host controller data */
struct mcuio_hc_data {
	unsigned bus;
	struct mutex lock;
	struct list_head request_queue;
	struct list_head pending_requests;
	/* List of descriptors not yet registered */
	struct list_head to_be_registered;
	atomic_t removing;

	struct kthread_worker tx_kworker;
	struct task_struct *tx_kworker_task;
	struct kthread_work send_messages;

	struct task_struct *rx_thread;
	wait_queue_head_t rd_wq;

	struct mcuio_device *mdev;
	struct kthread_worker enum_kworker;
	struct task_struct *enum_kworker_task;
	struct kthread_work do_enum;

	int *irqs[MCUIO_DEVS_PER_BUS];
};

typedef int (*mcuio_copy)(uint32_t *, const uint32_t *, int, int);

int __mcuio_copyb(uint32_t *dst, const uint32_t *src, int fill, int ntoh)
{
	if (!fill) {
		*(uint8_t *)dst = *(uint8_t *)src;
		return sizeof(uint8_t);
	}
	memcpy(dst, src, sizeof(uint64_t));
	return sizeof(uint64_t);
}

int __mcuio_copyw(uint32_t *__dst, const uint32_t *__src, int fill, int ntoh)
{
	uint16_t *dst = (uint16_t *)__dst;
	uint16_t *src = (uint16_t *)__src;
	int i, n = fill ? sizeof(uint64_t) / sizeof(uint16_t) : 1;
	for (i = 0; i < n; i++)
		*dst++ = ntoh ? mcuio_ntohs(*src++) : mcuio_htons(*src++);
	return n * sizeof(uint16_t);
}

int __mcuio_copydw(uint32_t *dst, const uint32_t *src, int fill, int ntoh)
{
	*dst++ = mcuio_ntohl(*src++);
	if (fill)
		*dst++ = ntoh ? mcuio_ntohl(*src++) : mcuio_htonl(*src++);
	return fill ? sizeof(uint64_t) : sizeof(uint32_t);
}

static const mcuio_copy __copy_table[] = {
	[ mcuio_type_rdb ] = __mcuio_copyb,
	[ mcuio_type_wrb ] = __mcuio_copyb,
	[ mcuio_type_rdw ] = __mcuio_copyw,
	[ mcuio_type_wrw ] = __mcuio_copyw,
	[ mcuio_type_rddw ] = __mcuio_copydw,
	[ mcuio_type_wrdw ] = __mcuio_copydw,
	/* Unsupported */
	[ mcuio_type_rdq ] = NULL,
	[ mcuio_type_wrq ] = NULL,
};

static int __copy_data(uint32_t *addr, struct mcuio_packet *p, int ntoh)
{
	mcuio_copy cp = __copy_table[mcuio_packet_type(p) &
				     mcuio_actual_type_mask];
	uint32_t *__dst = ntoh ? addr : p->data;
	uint32_t *__src = ntoh ? p->data : addr;
	if (!cp)
		return -ENOSYS;
	return cp(__dst, __src, mcuio_packet_is_fill_data(p),
		  ntoh ? mcuio_packet_is_read(p) : !mcuio_packet_is_read(p));
}

static struct mcuio_request *mcuio_alloc_request(struct mcuio_device *mdev)
{
	struct mcuio_request *out = devm_kzalloc(&mdev->dev, sizeof(*out),
						 GFP_KERNEL);
	return out;
}


static void mcuio_free_request(struct mcuio_request *r)
{
	struct mcuio_hc_data *data;
	data = dev_get_drvdata(&r->hc->dev);
	mutex_lock(&data->lock);
	list_del(&r->list);
	mutex_unlock(&data->lock);
	if (!r->dont_free)
		devm_kfree(&r->hc->dev, r);
}


static void __request_to_packet(struct mcuio_request *r, struct mcuio_packet *p)
{
	mcuio_packet_set_addr(p, r->hc->bus, r->dev, r->func, r->offset,
			      r->type, r->fill);
	if (mcuio_packet_is_read(p)) {
		p->data[0] = p->data[1] = 0;
		return;
	}
	/* Copy data to packet (host to network) */
	__copy_data(r->data, p, 0);
}

static struct mcuio_request *__make_request(struct mcuio_device *mdev,
					    unsigned dev, unsigned func,
					    unsigned type,
					    int fill,
					    unsigned offset,
					    unsigned offset_mask,
					    ___request_cb cb)
{
	struct mcuio_request *out = mcuio_alloc_request(mdev);
	if (!out)
		return NULL;
	out->hc = mdev;
	out->dev = dev;
	out->func = func;
	out->type = type;
	out->offset = offset;
	out->offset_mask = offset_mask;
	out->status = -ETIMEDOUT;
	out->cb = cb;
	out->fill = fill;
	return out;
}

static void __request_timeout(struct work_struct *work)
{
	struct mcuio_request *r =
		container_of(work, struct mcuio_request, to_work.work);
	if (r->cb)
		r->cb(r);
	mcuio_free_request(r);
}

static int __write_message(struct regmap *map, const u32 *ptr, int count)
{
	int i, stat;
	for (i = 0; i < count; i++) {
		stat = regmap_write(map,
				    MCUIO_HC_OUTBUF + i * sizeof(u32), ptr[i]);
		if (stat < 0)
			return stat;
	}
	return 0;
}

static int __do_request(struct mcuio_hc_data *data)
{
	struct mcuio_request *r;
	struct mcuio_device *mdev;
	struct regmap *map;
	u32 buf[4];
	struct mcuio_packet *p = (struct mcuio_packet *)buf;

	mutex_lock(&data->lock);
	if (list_empty(&data->request_queue)) {
		mutex_unlock(&data->lock);
		pr_debug("%s %d\n", __func__, __LINE__);
		return 0;
	}
	r = list_entry(data->request_queue.next, struct mcuio_request, list);
	__request_to_packet(r, p);
	mdev = r->hc;
	map = dev_get_regmap(&mdev->dev, NULL);
	if (!map) {
		mutex_unlock(&data->lock);
		WARN_ON(1);
		return -EIO;
	}
	list_move(&r->list, &data->pending_requests);
	mutex_unlock(&data->lock);
	/* Schedule timeout */
	INIT_DELAYED_WORK(&r->to_work, __request_timeout);
	/* FIXME: WHAT IS THE CORRECT DELAY ? */
	schedule_delayed_work(&r->to_work, HZ/10);
	if (__write_message(map, buf, 4) < 0) {
		dev_err(&mdev->dev, "error writing to output fifo");
		goto regmap_error;
	}
	return 1;

regmap_error:
	cancel_delayed_work_sync(&r->to_work);
	mcuio_free_request(r);
	return -EIO;
}

static irqreturn_t hc_irq_handler(int irq, void *__data)
{
	struct mcuio_device *mdev = __data;
	struct regmap *map = dev_get_regmap(&mdev->dev, NULL);
	struct mcuio_hc_data *data = dev_get_drvdata(&mdev->dev);
	int ret;
	u32 status;

	if (!data) {
		dev_err(&mdev->dev, "no drv data in irq handler\n");
		return IRQ_NONE;
	}
	ret = regmap_read(map, MCUIO_IRQ_STAT, &status);
	if (ret < 0)
		return IRQ_NONE;
	if (status & RX_RDY)
		wake_up_interruptible(&data->rd_wq);
	ret = regmap_write(map, MCUIO_IRQ_CLR, status);
	if (ret < 0)
		dev_err(&mdev->dev, "error clearing irq flag\n");
	return IRQ_HANDLED;
}

static inline int __is_irq_controller(struct mcuio_func_descriptor *d)
{
	return d->rev_class == MCUIO_CLASS_IRQ_CONTROLLER_PROTO ||
		d->rev_class == MCUIO_CLASS_IRQ_CONTROLLER_WIRE;
}

static inline u32 __get_available(struct regmap *map)
{
	u32 out;
	int stat = regmap_read(map, MCUIO_RX_CNT, &out);
	if (stat < 0)
		return 0;
	return out;
}

static int __read_message(struct mcuio_hc_data *data,
			  struct regmap *map, u32 *ptr, int count)
{
	int i, stat;

	stat = wait_event_interruptible(data->rd_wq,
					__get_available(map) >= count ||
					kthread_should_stop());
	/* FIXME: handle signals */
	if (stat < 0 || kthread_should_stop()) {
		pr_debug("%s returns %d\n", __func__, stat);
		return stat;
	}
	for (i = 0; i < count; i++, ptr++) {
		stat = regmap_read(map, MCUIO_HC_INBUF + i * sizeof(u32), ptr);
		if (stat < 0)
			return stat;
	}
	return 0;
}

static struct mcuio_request *__find_request(struct mcuio_device *hc,
					    struct mcuio_packet *p)
{
	struct mcuio_request *r;
	struct mcuio_hc_data *data = dev_get_drvdata(&hc->dev);

	mutex_lock(&data->lock);
	list_for_each_entry(r, &data->pending_requests, list) {
		if ((mcuio_packet_type(p) & mcuio_actual_type_mask) ==
		    (r->type & mcuio_actual_type_mask) &&
		    mcuio_packet_bus(p) == hc->bus &&
		    mcuio_packet_dev(p) == r->dev &&
		    mcuio_packet_func(p) == r->func &&
		    (mcuio_packet_offset(p) & r->offset_mask) ==
		    (r->offset & r->offset_mask)) {
			mutex_unlock(&data->lock);
			return r;
		}
	}
	mutex_unlock(&data->lock);
	return NULL;
}

static int __receive_messages(void *__data)
{
	struct mcuio_device *hc = __data;
	struct mcuio_hc_data *data = dev_get_drvdata(&hc->dev);
	struct regmap *map;
	if (!data) {
		dev_err(&hc->dev, "no driver data in %s\n", __func__);
		return -EINVAL;
	}
	map = dev_get_regmap(&hc->dev, NULL);
	while (!kthread_should_stop()) {
		u32 buf[4];
		int stat;
		struct mcuio_packet *p;
		struct mcuio_request *r;
		stat = __read_message(data, map, buf, 4);
		if (stat < 0) {
			schedule();
			continue;
		}
		p = (struct mcuio_packet *)buf;
		if (!mcuio_packet_is_reply(p)) {
			if (mcuio_packet_is_read(p))
				/*
				  Packet is a read request, we do not handle
				  read requests at the moment
				*/
				continue;
		}
		r = __find_request(hc, p);
		if (!r) {
			dev_err(&hc->dev, "unexpected reply");
			continue;
		}
		r->status = mcuio_packet_is_error(p);
		cancel_delayed_work_sync(&r->to_work);
		if (mcuio_packet_is_read(p))
			__copy_data(r->data, p, 1);
		if (r->cb)
			r->cb(r);
		mcuio_free_request(r);
	}
	return 0;
}

static void __send_messages(struct kthread_work *work)
{
	struct mcuio_hc_data *data =
		container_of(work, struct mcuio_hc_data, send_messages);
	while (__do_request(data) > 0);
}

static void __enqueue_request(struct mcuio_device *mdev,
			      struct mcuio_hc_data *data,
			      struct mcuio_request *r,
			      int outgoing)
{
	mutex_lock(&data->lock);
	list_add_tail(&r->list, outgoing ? &data->request_queue :
		      &data->pending_requests);
	mutex_unlock(&data->lock);
	if (outgoing)
		queue_kthread_work(&data->tx_kworker, &data->send_messages);
}

static int mcuio_hc_enqueue_request(struct mcuio_request *r, int outgoing)
{
	struct mcuio_hc_data *data;
	if (!r || !r->hc)
		return -EINVAL;
	data = dev_get_drvdata(&r->hc->dev);
	if (!data)
		return -EINVAL;
	if (atomic_read(&data->removing))
		return -ENODEV;
	__enqueue_request(r->hc, data, r, outgoing);
	return 0;
}

static void __request_cb(struct mcuio_request *r)
{
	struct completion *c = r->cb_data;
	complete(c);
}

int mcuio_submit_request(struct mcuio_request *r)
{
	int ret;
	DECLARE_COMPLETION_ONSTACK(request_complete);
	r->cb = __request_cb;
	r->cb_data = &request_complete;
	r->status = -ETIMEDOUT;
	ret = mcuio_hc_enqueue_request(r, 1);
	if (!ret)
		ret = wait_for_completion_interruptible(&request_complete);
	if (ret)
		return ret;
	return r->status;
}
EXPORT_SYMBOL(mcuio_submit_request);

int mcuio_setup_cb(struct mcuio_request *r)
{
	return mcuio_hc_enqueue_request(r, 0);
}
EXPORT_SYMBOL(mcuio_setup_cb);

int mcuio_hc_set_irqs(struct mcuio_device *hc, unsigned dev, int __irqs[])
{
	struct mcuio_hc_data *data = dev_get_drvdata(&hc->dev);
	int *irqs, size = sizeof(int) * MCUIO_FUNCS_PER_DEV;
	if (!data) {
		WARN_ON(1);
		return -ENODEV;
	}
	irqs = devm_kzalloc(&hc->dev, size, GFP_KERNEL);
	if (!irqs) {
		dev_err(&hc->dev, "No memory for irqs for %u:%u\n", hc->bus,
			dev);
		return -ENOMEM;
	}
	memcpy(irqs, __irqs, size);
	data->irqs[dev] = irqs;
	return 0;
}
EXPORT_SYMBOL(mcuio_hc_set_irqs);

static int __do_one_enum(struct mcuio_device *mdev, unsigned edev,
			 unsigned efunc, struct mcuio_request **out)
{
	struct mcuio_request *r;
	int ret;

	r = __make_request(mdev, edev, efunc,
			   mcuio_type_rddw, 1, 0, 0xffff, NULL);
	if (!r) {
		*out = NULL;
		return -ENOMEM;
	}
	ret = mcuio_submit_request(r);
	*out = r;
	return ret;
}

static void __register_device(struct mcuio_request *r)
{
	struct mcuio_func_descriptor d;
	struct mcuio_device *hc = r->hc;
	struct mcuio_hc_data *data = dev_get_drvdata(&hc->dev);
	struct mcuio_device *new;
	new = kzalloc(sizeof(*new), GFP_KERNEL);
	if (!new) {
		dev_err(&r->hc->dev,
			"error allocating device %u:%u.%u\n",
			hc->bus, r->dev, r->func);
		return;
	}
	memcpy(&d, r->data, sizeof(d));
	new->id.device = mcuio_get_device(&d);
	new->id.vendor = mcuio_get_vendor(&d);
	new->id.class = d.rev_class;
	new->id.class_mask = 0xffffffff;
	new->bus = hc->bus;
	new->device = r->dev;
	new->fn = r->func;
	if (data->irqs[r->dev])
		new->irq = (data->irqs[r->dev])[r->func];
	pr_debug("%s %d, device = 0x%04x, vendor = 0x%04x, "
		 "class = 0x%04x\n", __func__, __LINE__, new->id.device,
		 new->id.vendor, new->id.class);
	if (mcuio_device_register(new, NULL, &hc->dev) < 0) {
		dev_err(&r->hc->dev,
			"error registering device %u:%u.%u\n",
			hc->bus, r->dev, r->func);
		kfree(new);
	}
}

static int __next_enum(unsigned *edev, unsigned *efunc, int *retry)
{
	if ((*retry) > 0) {
		/* Doing retries */
		(*retry)--;
		return 0;
	}
	if (!(*retry)) {
		/* No reply and no more attempts left, skip to next device */
		*retry = -1;
		if ((*edev)++ >= MCUIO_DEVS_PER_BUS - 1)
			return 1;
		*efunc = 0;
		return 0;
	}
	if ((*efunc)++ >= MCUIO_FUNCS_PER_DEV - 1) {
		*efunc = 0;
		if ((*edev)++ >= MCUIO_DEVS_PER_BUS - 1)
			return 1;
	}
	return 0;
}

static void __register_deferred_devices(struct mcuio_hc_data *data)
{
	struct mcuio_deferred_register_element *e, *tmp;
	list_for_each_entry_safe(e, tmp, &data->to_be_registered, list) {
		pr_debug("deferred registering of %u:%u.%u\n",
			 e->r.hc->bus, e->r.dev, e->r.func);
		__register_device(&e->r);
		list_del(&e->list);
		devm_kfree(&e->r.hc->dev, e);
	}
}

static void __do_enum(struct kthread_work *work)
{
	struct mcuio_hc_data *data =
		container_of(work, struct mcuio_hc_data, do_enum);
	struct mcuio_device *mdev = data->mdev;
	struct mcuio_request *r;
	struct mcuio_deferred_register_element *e;
	unsigned edev, efunc;
	struct mcuio_func_descriptor *d;
	int stop_enum, irq_controller_found = 0, stat, retry = -1;

	for (edev = 1, efunc = 0, stop_enum = 0; !stop_enum;
	     stop_enum = __next_enum(&edev, &efunc, &retry)) {
		if (!efunc) {
			/* Register any pending devices for current mcu */
			__register_deferred_devices(data);
			irq_controller_found = 0;
		}
		stat = __do_one_enum(mdev, edev, efunc, &r);
		if (!r) {
			dev_err(&mdev->dev, "no request\n");
			continue;
		}
		if (stat < 0) {
			dev_err(&mdev->dev,
				"error %d on enum of %u.%u\n",
				r->status == -ETIMEDOUT ? r->status :
				r->data[0], edev, efunc);
			if (r->status == -ETIMEDOUT) {
				/* No reply from target */
				retry = retry == -1 ? MAX_ENUM_RETRIES :
					retry - 1;
			}
			continue;
		}
		retry = -1;
		/*
		  Found a new devices, let's add it, but only if an
		  irq controller has already been found
		*/
		d = (struct mcuio_func_descriptor *)&r->data;
		if (__is_irq_controller(d) || irq_controller_found) {
			irq_controller_found = 1;
			__register_device(r);
			continue;
		}
		e = devm_kzalloc(&r->hc->dev, sizeof(*e), GFP_KERNEL);
		if (!e) {
			WARN_ON(1);
			continue;
		}
		memcpy(&e->r, r, sizeof(*r));
		list_add_tail(&e->list, &data->to_be_registered);
	}
	/*
	 * Register any remaining pending devices
	 */
	__register_deferred_devices(data);
}

static int mcuio_host_controller_probe(struct mcuio_device *mdev)
{
	struct mcuio_hc_data *data;
	struct mcuio_hc_platform_data *plat;
	struct regmap *map;
	u32 irq;
	int ret = -ENOMEM;
	/* Only manage local host controllers */
	if (mdev->device)
		return -ENODEV;
	plat = dev_get_platdata(&mdev->dev);
	if (!plat) {
		dev_err(&mdev->dev, "No platform data\n");
		return -EINVAL;
	}
	map = plat->setup_regmap(&mdev->dev, plat);
	if (IS_ERR(map)) {
		dev_err(&mdev->dev, "Error setting up regmap for device\n");
		return PTR_ERR(map);
	}
	data = devm_kzalloc(&mdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return ret;
	dev_set_drvdata(&mdev->dev, data);
	atomic_set(&data->removing, 0);
	mutex_init(&data->lock);
	data->mdev = mdev;
	init_kthread_worker(&data->tx_kworker);
	init_waitqueue_head(&data->rd_wq);
	ret = regmap_read(map, MCUIO_IRQ, &irq);
	if (ret < 0) {
		dev_err(&mdev->dev, "Error %d reading irq number\n", ret);
		return ret;
	}
	ret = devm_request_threaded_irq(&mdev->dev, irq, NULL,
					hc_irq_handler,
					IRQF_ONESHOT,
					dev_name(&mdev->dev), mdev);
	if (ret < 0) {
		dev_err(&mdev->dev, "Error %d requesting irq\n", ret);
		return ret;
	}
	data->tx_kworker_task = kthread_run(kthread_worker_fn,
					    &data->tx_kworker,
					    "%s_%s",
					    dev_name(&mdev->dev), "tx");
	if (IS_ERR(data->tx_kworker_task)) {
		dev_err(&mdev->dev, "failed to create message tx task\n");
		return -ENOMEM;
	}
	init_kthread_work(&data->send_messages, __send_messages);
	INIT_LIST_HEAD(&data->request_queue);
	INIT_LIST_HEAD(&data->pending_requests);
	INIT_LIST_HEAD(&data->to_be_registered);
	data->rx_thread = kthread_run(__receive_messages, mdev, "%s_%s",
				      dev_name(&mdev->dev), "rx");
	if (IS_ERR(data->rx_thread)) {
		dev_err(&mdev->dev, "failed to create message rx task\n");
		kthread_stop(data->tx_kworker_task);
		return PTR_ERR(data->rx_thread);
	}
	init_kthread_worker(&data->enum_kworker);
	data->enum_kworker_task = kthread_run(kthread_worker_fn,
					      &data->enum_kworker,
					      "%s_%s",
					      dev_name(&mdev->dev), "enum");
	if (IS_ERR(data->enum_kworker_task)) {
		dev_err(&mdev->dev, "failed to create enum task\n");
		return -ENOMEM;
	}
	init_kthread_work(&data->do_enum, __do_enum);
	/* Immediately start enum */
	queue_kthread_work(&data->enum_kworker, &data->do_enum);
	return 0;
}

static void __cleanup_outstanding_requests(struct mcuio_hc_data *data)
{
	struct mcuio_request *r, *tmp;
	list_for_each_entry_safe(r, tmp, &data->pending_requests, list) {
		pr_debug("%s %d: freeing request %p\n", __func__,
			 __LINE__, r);
		cancel_delayed_work_sync(&r->to_work);
		if (r->cb)
			r->cb(r);
		mcuio_free_request(r);
	}
}

static int mcuio_host_controller_remove(struct mcuio_device *mdev)
{
	struct mcuio_hc_data *data = dev_get_drvdata(&mdev->dev);
	atomic_set(&data->removing, 1);
	barrier();
	flush_kthread_worker(&data->tx_kworker);
	kthread_stop(data->tx_kworker_task);
	__cleanup_outstanding_requests(data);
	devm_kfree(&mdev->dev, data);
	return 0;
}

static const struct mcuio_device_id hc_drv_ids[] = {
	{
		.class = MCUIO_CLASS_HOST_CONTROLLER,
		.class_mask = 0xffff,
	},
	{
		.class = MCUIO_CLASS_SOFT_HOST_CONTROLLER,
		.class_mask = 0xffff,
	},
	/* Terminator */
	{
		.device = MCUIO_NO_DEVICE,
		.class = MCUIO_CLASS_UNDEFINED,
	},
};

static struct mcuio_driver mcuio_host_controller_driver = {
	.driver = {
		.name = "mcuio-hc",
	},
	.id_table = hc_drv_ids,
	.probe = mcuio_host_controller_probe,
	.remove = mcuio_host_controller_remove,
};

static int __init mcuio_host_controller_init(void)
{
	return mcuio_driver_register(&mcuio_host_controller_driver,
				     THIS_MODULE);
}

static void __exit mcuio_host_controller_exit(void)
{
	return mcuio_driver_unregister(&mcuio_host_controller_driver);
}

subsys_initcall(mcuio_host_controller_init);
module_exit(mcuio_host_controller_exit);

MODULE_VERSION(GIT_VERSION); /* Defined in local Makefile */
MODULE_AUTHOR("Davide Ciminaghi, some code from ZIO by Vaga, Rubini");
MODULE_DESCRIPTION("MCUIO host controller driver");
MODULE_LICENSE("GPL v2");
