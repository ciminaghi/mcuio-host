#include <linux/mcuio.h>
#include <linux/circ_buf.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/regmap.h>
#include <linux/err.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/circ_buf.h>
#include <linux/mcuio_ids.h>

#include <linux/mcuio.h>
#include <linux/mcuio-proto.h>
#include "mcuio-hc.h"
#include "mcuio-soft-hc.h"
#include "mcuio-internal.h"

static bool mcuio_soft_hc_readable(struct device *dev, unsigned int reg)
{
	return true;
}

static bool mcuio_soft_hc_writeable(struct device *dev, unsigned int reg)
{
	return (reg >= MCUIO_HC_OUTBUF && reg < MCUIO_HC_INBUF) ||
		reg == MCUIO_IRQ_CLR;
	return true;
}

/*
 * regmap config for line discipline based mcuio host controller
 */
static struct regmap_config proto = {
	.name = "mcuio-ldisc",
	.reg_bits = 8,
	.reg_stride = 4,
	.val_bits = 32,
	.max_register = MCUIO_HC_MAX_REGISTER,
	.readable_reg = mcuio_soft_hc_readable,
	.writeable_reg = mcuio_soft_hc_writeable,
	.cache_type = REGCACHE_NONE,
};

static int mcuio_soft_hc_read_inbuf(struct mcuio_soft_hc *shc,
			       unsigned int reg,
			       unsigned int *val)
{
	int i, s = sizeof(shc->rx_buf);
	u8 *out = (u8 *)val;
	struct circ_buf *buf = &shc->rx_circ_buf;

	if (CIRC_CNT(buf->head, buf->tail, s) < sizeof(u32))
		return -EAGAIN;
	for (i = 0; i < sizeof(u32); i++) {
		out[i] = buf->buf[buf->tail++];
		buf->tail &= (s - 1);
	}
	return sizeof(unsigned int);
}

static int mcuio_soft_hc_reg_read(void *context, unsigned int reg,
				  unsigned int *val)
{
	struct mcuio_soft_hc *shc = context;
	if (!shc)
		return -EINVAL;
	if (reg >= MCUIO_HC_INBUF && reg < MCUIO_RX_CNT)
		return mcuio_soft_hc_read_inbuf(shc, reg, val);
	switch(reg) {
	case MCUIO_RX_CNT:
	{
		struct circ_buf *buf = &shc->rx_circ_buf;
		*val = CIRC_CNT(buf->head, buf->tail, sizeof(shc->rx_buf));
		return sizeof(*val);
	}
	case MCUIO_IRQ:
		*val = shc->irqno;
		return sizeof(*val);
	case MCUIO_IRQ_STAT:
		*val = shc->irqstat;
		return sizeof(*val);
	default:
		return -EPERM;
	}
	/* NEVER REACHED */
	return -EPERM;
}

static int mcuio_soft_hc_reg_write(void *context,
				   unsigned int reg, unsigned int val)
{
	struct mcuio_soft_hc *shc = context;
	u8 *out = (u8 *)&val;
	if (!shc)
		return -EINVAL;
	if (reg >= MCUIO_HC_OUTBUF && reg < MCUIO_HC_INBUF)
		return shc->ops->write(shc, out, sizeof(val));
	if (reg == MCUIO_IRQ_CLR) {
		shc->irqstat &= ~val;
		return 0;
	}
	return -EPERM;
}

int mcuio_soft_hc_push_chars(struct mcuio_soft_hc *shc, const u8 *in, int len)
{
	int s = sizeof(shc->rx_buf), available, actual;
	struct circ_buf *buf = &shc->rx_circ_buf;
	available = CIRC_SPACE_TO_END(buf->head, buf->tail, s);
	if (available < sizeof(u32)) {
		pr_debug("%s %d\n", __func__, __LINE__);
		return -EAGAIN;
	}
	actual = min(len, available);
	memcpy(&buf->buf[buf->head], in, actual);
	buf->head = (buf->head + actual) & (s - 1);
	/* set irq status register RX_RDY bit */
	shc->irqstat |= RX_RDY;
	if (shc->irq_enabled)
		handle_nested_irq(shc->irqno);
	return actual;
}
EXPORT_SYMBOL(mcuio_soft_hc_push_chars);

static struct regmap_config *mcuio_soft_hc_setup_regmap_config(void)
{
	struct regmap_config *out = kzalloc(sizeof(*out), GFP_KERNEL);
	if (!out)
		return out;
	*out = proto;
	out->reg_read = mcuio_soft_hc_reg_read;
	out->reg_write = mcuio_soft_hc_reg_write;
	return out;
}

static struct regmap *
mcuio_soft_hc_setup_regmap(struct device *dev,
			   void *__plat)
{
	struct mcuio_hc_platform_data *plat = __plat;
	struct regmap_config *map_cfg = mcuio_soft_hc_setup_regmap_config();
	struct mcuio_soft_hc *shc;
	struct regmap *out = ERR_PTR(-ENOMEM);
	if (!map_cfg) {
		dev_err(dev, "%s: cannot setup regmap config\n", __func__);
		return ERR_PTR(-ENOMEM);
	}
	shc = plat->data;
	if (!shc) {
		dev_err(dev, "%s: no platform data\n", __func__);
		return out;
	}
	/*
	  no_bus regmap with reg_read and reg_write, use soft controller
	  structure as regmap context
	*/
	return regmap_init(dev, NULL, shc, map_cfg);
}

static void mcuio_soft_hc_irq_mask(struct irq_data *d)
{
	struct irq_chip *chip = irq_data_get_irq_chip(d);
	struct mcuio_soft_hc *shc =
		container_of(chip, struct mcuio_soft_hc, chip);

	shc->irq_enabled = 0;
}

static void mcuio_soft_hc_irq_unmask(struct irq_data *d)
{
	struct irq_chip *chip = irq_data_get_irq_chip(d);
	struct mcuio_soft_hc *shc =
		container_of(chip, struct mcuio_soft_hc, chip);

	shc->irq_enabled = 1;
}

static struct mcuio_soft_hc *__setup_shc(const struct mcuio_soft_hc_ops *ops,
					 void *priv)
{
	struct mcuio_soft_hc *shc = kzalloc(sizeof(*shc), GFP_KERNEL);
	if (!shc)
		return ERR_PTR(-ENOMEM);
	shc->ops = ops;
	shc->priv = priv;
	shc->rx_circ_buf.head = shc->rx_circ_buf.tail = 0;
	shc->rx_circ_buf.buf = shc->rx_buf;
	shc->chip.name = "MCUIO-SHC";
	shc->chip.irq_mask = mcuio_soft_hc_irq_mask;
	shc->chip.irq_unmask = mcuio_soft_hc_irq_unmask;
	shc->irqno = irq_alloc_desc(0);
	irq_set_chip(shc->irqno, &shc->chip);
	irq_set_handler(shc->irqno, &handle_simple_irq);
	irq_modify_status(shc->irqno,
			  IRQ_NOREQUEST | IRQ_NOAUTOEN,
			  IRQ_NOPROBE);
	return shc;
}

static struct mcuio_device_id default_soft_hc_id = {
	.device = 0,
	.vendor = 0,
	.class = MCUIO_CLASS_SOFT_HOST_CONTROLLER,
};

static void mcuio_soft_hc_release(struct device *device)
{
	struct mcuio_hc_platform_data *plat = dev_get_platdata(device);
	struct mcuio_soft_hc *shc;
	if (!plat) {
		WARN_ON(1);
		return;
	}
	shc = plat->data;
	irq_set_handler(shc->irqno, NULL);
	irq_set_chip(shc->irqno, NULL);
	irq_free_desc(shc->irqno);
	kfree(shc);
	mcuio_hc_dev_default_release(device);
}

struct device *mcuio_add_soft_hc(struct mcuio_device_id *id,
				 const struct mcuio_soft_hc_ops *ops,
				 void *priv)
{
	struct mcuio_hc_platform_data *plat;
	struct mcuio_soft_hc *shc = __setup_shc(ops, priv);
	if (!shc)
		return ERR_PTR(-ENOMEM);
	plat = kzalloc(sizeof(*plat), GFP_KERNEL);
	if (!plat) {
		kfree(shc);
		return ERR_PTR(-ENOMEM);
	}
	plat->setup_regmap = mcuio_soft_hc_setup_regmap;
	plat->data = shc;
	return mcuio_add_hc_device(id ? id : &default_soft_hc_id, plat,
				   mcuio_soft_hc_release);
}
EXPORT_SYMBOL(mcuio_add_soft_hc);

MODULE_VERSION(GIT_VERSION); /* Defined in local Makefile */
MODULE_AUTHOR("Davide Ciminaghi, some code from ZIO by Vaga, Rubini");
MODULE_DESCRIPTION("MCUIO soft host controller code");
MODULE_LICENSE("GPL v2");
