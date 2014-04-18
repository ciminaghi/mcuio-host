/*
 * Copyright 2011 Dog Hunter SA
 * Author: Davide Ciminaghi <ciminaghi@gnudd.com>
 *
 * GNU GPLv2 or later
 */

#define DEBUG

/* mcuio driver for joystick shield */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/regmap.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/hid.h>


#include <linux/mcuio.h>
#include <linux/mcuio_ids.h>
#include <linux/mcuio-proto.h>

#include "mcuio-internal.h"

/* The HID report descriptor (just 4 buttons at present) */
static char mcuio_js_report_descriptor[] = {
	/* USAGE_PAGE (Button) */
	0x05, 0x09,
	/* USAGE_MINIMUM (Button 1) */
	0x19, 0x01,
	/* USAGE_MAXIMUM (Button 4) */
	0x29, 0x04,
	/* LOGICAL_MINIMUM (0) */
	0x15, 0x00,
	/* LOGICAL_MAXIMUM (1) */
	0x25, 0x01,
	/* REPORT_COUNT (4) */
	0x95, 0x04,
	/* REPORT_SIZE (1) */
	0x75, 0x01,
	/* INPUT (Data,Var,Abs) */
	0x81, 0x02,
	/* REPORT_COUNT (4) */
	0x95, 0x04,
	/* REPORT_SIZE (1) */
	0x75, 0x01,
	/* INPUT (Cnst,Var,Abs) */
	0x81, 0x03,
};

struct mcuio_js_data;

struct mcuio_js_gpio {
	const char *name;
	unsigned gpio;
	int irq;
	struct mcuio_js_data *js_data;
};

static const char *js_gpio_names[] = {
	"SCL", "D4", "D5", "D6",
};

struct mcuio_js_data {
	struct hid_device *hid;
	u8 cached_gpios;
	struct mcuio_js_gpio gpios[ARRAY_SIZE(js_gpio_names)];
};


static int __match_gpiochip(struct gpio_chip *chip, void *__gpio_data)
{
	struct mcuio_js_gpio *data = __gpio_data;
	const char *ptr;
	int i;

	pr_debug("%s entered (name = %s)\n", __func__, data->name);

	if (!chip->names) {
		pr_debug("%s: gpiochip has no names\n", __func__);
		return 0;
	}
	for (i = 0; i < chip->ngpio; i++, ptr++) {
		ptr = chip->names[i];
		if (!ptr)
			continue;
		pr_debug("%s: found gpio %s\n", __func__, chip->names[i]);
		if (!strcmp(ptr, data->name)) {
			data->gpio = i + chip->base;
			pr_debug("%s: gpiochip found\n", __func__);
			return 1;
		}
	}
	pr_debug("%s: gpiochip not found\n", __func__);
	return 0;
}

static int __setup_gpio(struct mcuio_device *mdev, struct mcuio_js_gpio *data)
{
	int ret = -ENODEV;
	struct gpio_chip *chip;

	chip = gpiochip_find(data, __match_gpiochip);
	if (!chip) {
		dev_dbg(&mdev->dev, "%s: gpiochip not found\n", __func__);
		return ret;
	}
	ret = devm_gpio_request_one(&mdev->dev, data->gpio, GPIOF_DIR_IN,
				    "js-shield");
	if (ret < 0)
		return ret;
	/* HACK FOR ATMEGA : THIS IS NEEDED TO ENABLE PULLUP */
	ret = gpio_direction_output(data->gpio, 1);
	if (ret) {
		dev_err(&mdev->dev,
			"gpio%u: error setting direction to output\n",
			data->gpio);
		return ret;
	}
	ret = gpio_direction_input(data->gpio);
	if (ret) {
		dev_err(&mdev->dev,
			"gpio%u: error setting direction to input\n",
			data->gpio);
		return ret;
	}
	ret = gpio_to_irq(data->gpio);
	if (ret < 0) {
		dev_err(&mdev->dev,
			"gpio%u: gpio_to_irq returned error\n",
			data->gpio);
		return ret;
	}
	data->irq = ret;
	return 0;
}

static irqreturn_t mcuio_js_irq_handler(int irq, void *__data)
{
	int v;
	struct mcuio_js_gpio *gpio_data = __data;
	struct mcuio_js_data *js_data = gpio_data->js_data;
	int gpio_index = gpio_data - js_data->gpios;
	pr_debug("%s entered (gpio %u)", __func__, gpio_data->gpio);

	v = !!!gpio_get_value_cansleep(gpio_data->gpio);
	js_data->cached_gpios &= ~(1 << gpio_index);
	if (v)
		js_data->cached_gpios |= (1 << gpio_index);

	/* Send out a report now */
	hid_input_report(js_data->hid, HID_INPUT_REPORT, &js_data->cached_gpios,
			 1, 1);

	return IRQ_HANDLED;
}

static int mcuio_js_hid_get_raw_report(struct hid_device *hid,
				       unsigned char report_number,
				       __u8 *buf, size_t count,
				       unsigned char report_type)
{
	struct mcuio_device *mdev = hid->driver_data;
	struct mcuio_js_data *js_data;
	pr_debug("%s invoked, report_number = %u, report_type = %u\n",
		 __func__, report_number, report_type);
	if (!mdev) {
		pr_err("no mcuio device !\n");
		return -ENODEV;
	}
	js_data = dev_get_drvdata(&mdev->dev);
	if (!js_data) {
		dev_err(&mdev->dev, "no drv data !\n");
		return -ENODEV;
	}
	if (report_type == HID_OUTPUT_REPORT)
		return -EINVAL;
	if (report_type == HID_FEATURE_REPORT)
		/* Unsupported at the moment */
		return -EINVAL;
	if (count != 1) {
		pr_err("%s: invalid count %zu\n", __func__, count);
		return -EINVAL;
	}
	/* FIXME !! */
	buf[0] = js_data->cached_gpios;
	return 1;
}


static int mcuio_js_hid_output_raw_report(struct hid_device *hid, __u8 *buf,
					  size_t count,
					  unsigned char report_type)
{
	pr_debug("%s invoked, report_type = %u\n", __func__, report_type);
	return -EINVAL;
}


static int mcuio_js_hid_start(struct hid_device *hid)
{
	struct mcuio_device *mdev = hid->driver_data;
	struct mcuio_js_data *js_data;
	int i, ret;

	hid_dbg(hid, "%s invoked\n", __func__);
	if (!mdev) {
		hid_err(hid, "%s: mdev is NULL\n", __func__);
		return -ENODEV;
	}
	js_data = dev_get_drvdata(&mdev->dev);
	if (!js_data) {
		hid_err(hid, "%s: js_data is NULL\n", __func__);
		return -ENODEV;
	}
	for (i = 0; i < ARRAY_SIZE(js_data->gpios); i++) {
		struct mcuio_js_gpio *data = &js_data->gpios[i];
		unsigned int v;
		ret = devm_request_threaded_irq(&mdev->dev, data->irq,
						NULL,
						mcuio_js_irq_handler,
						IRQF_TRIGGER_RISING |
						IRQF_TRIGGER_FALLING,
						"mcuio-js",
						data);
		if (ret)
			return ret;
		v = !!!gpio_get_value_cansleep(data->gpio);
		if (v)
			js_data->cached_gpios |= (1 << i);
	}
	hid_dbg(hid, "hw start ok\n");
	return 0;
}

static void mcuio_js_hid_stop(struct hid_device *hid)
{
	struct mcuio_device *mdev = hid->driver_data;
	struct mcuio_js_data *js_data;
	int i, ret;

	hid_dbg(hid, "%s invoked\n", __func__);
	if (!mdev) {
		hid_err(hid, "%s: mdev is NULL\n", __func__);
		return;
	}
	js_data = dev_get_drvdata(&mdev->dev);
	if (!js_data) {
		hid_err(hid, "%s: js_data is NULL\n", __func__);
		return;
	}
	for (i = 0; i < ARRAY_SIZE(js_data->gpios); i++) {
		struct mcuio_js_gpio *data = &js_data->gpios[i];
		free_irq(data->irq, data);
	}
	hid_dbg(hid, "hw start ok\n");
	return 0;
}

static int mcuio_js_hid_open(struct hid_device *hid)
{
	pr_debug("%s invoked\n", __func__);
	return 0;
}

static void mcuio_js_hid_close(struct hid_device *hid)
{
	pr_debug("%s invoked\n", __func__);
}

static int mcuio_js_hid_input(struct input_dev *input, unsigned int type,
			      unsigned int code, int value)
{
	pr_debug("%s invoked\n", __func__);
	return 0;
}

static int mcuio_js_hid_parse(struct hid_device *hid)
{
	return hid_parse_report(hid, mcuio_js_report_descriptor,
				sizeof(mcuio_js_report_descriptor));
}


static struct hid_ll_driver mcuio_js_hid_ll_driver = {
	.start = mcuio_js_hid_start,
	.stop = mcuio_js_hid_stop,
	.open = mcuio_js_hid_open,
	.close = mcuio_js_hid_close,
	.hidinput_input_event = mcuio_js_hid_input,
	.parse = mcuio_js_hid_parse,
};

static int mcuio_js_probe(struct mcuio_device *mdev)
{
	int i, ret = 0;
	struct hid_device *hid;
	struct mcuio_js_data *js_data;

	dev_dbg(&mdev->dev, "%s entered\n", __func__);
	js_data = devm_kzalloc(&mdev->dev, sizeof(*js_data), GFP_KERNEL);
	if (!js_data) {
		dev_err(&mdev->dev, "no memory for js data structure\n");
		return -ENOMEM;
	}
	for (i = 0; i < ARRAY_SIZE(js_data->gpios); i++) {
		struct mcuio_js_gpio *data = &js_data->gpios[i];
		data->name = js_gpio_names[i];
		data->js_data = js_data;
		ret = __setup_gpio(mdev, data);
		if (ret < 0)
			return ret;
	}

	hid = hid_allocate_device();
	if (IS_ERR(hid)) {
		dev_err(&mdev->dev, "error allocating hid device\n");
		return PTR_ERR(hid);
	}

	js_data->hid = hid;
	dev_set_drvdata(&mdev->dev, js_data);

	hid->driver_data = mdev;
	hid->ll_driver = &mcuio_js_hid_ll_driver;
	hid->hid_get_raw_report = mcuio_js_hid_get_raw_report;
	hid->hid_output_raw_report = mcuio_js_hid_output_raw_report;
	hid->dev.parent = &mdev->dev;
	hid->bus = BUS_VIRTUAL;
	hid->version = 0;
	hid->vendor = MCUIO_VENDOR_DOGHUNTER;
	hid->product = MCUIO_DEVICE_JOYSTICK_SHIELD;

	snprintf(hid->name, sizeof(hid->name), "mcuio-js-shield %04hX:%04hX",
		 hid->vendor, hid->product);

	ret = hid_add_device(hid);
	if (ret) {
		if (ret != -ENODEV)
			hid_err(mdev, "can't add hid device: %d\n", ret);
		hid_destroy_device(hid);
	}
	dev_dbg(&mdev->dev, "%s returns ok\n", __func__);

	return 0;
}

static int mcuio_js_remove(struct mcuio_device *mdev)
{
	struct mcuio_js_data *js_data = dev_get_drvdata(&mdev->dev);
	if (!js_data) {
		WARN_ON(1);
		return -ENODEV;
	}
	hid_destroy_device(js_data->hid);
	return 0;
}

static const struct mcuio_device_id js_drv_ids[] = {
	{
		.vendor = MCUIO_VENDOR_DOGHUNTER,
		.device = MCUIO_DEVICE_JOYSTICK_SHIELD,
	},
	/* Terminator */
	{
		.device = MCUIO_NO_DEVICE,
		.class = MCUIO_CLASS_UNDEFINED,
	},
};

static struct mcuio_driver mcuio_js_driver = {
	.driver = {
		.name = "mcuio-js-shield",
	},
	.id_table = js_drv_ids,
	.probe = mcuio_js_probe,
	.remove = mcuio_js_remove,
};

static int __init mcuio_js_init(void)
{
	return mcuio_driver_register(&mcuio_js_driver, THIS_MODULE);
}

static void __exit mcuio_js_exit(void)
{
	return mcuio_driver_unregister(&mcuio_js_driver);
}

subsys_initcall(mcuio_js_init);
module_exit(mcuio_js_exit);

MODULE_VERSION(GIT_VERSION); /* Defined in local Makefile */
MODULE_AUTHOR("Davide Ciminaghi");
MODULE_DESCRIPTION("MCUIO driver for joystick shield");
MODULE_LICENSE("GPL v2");
