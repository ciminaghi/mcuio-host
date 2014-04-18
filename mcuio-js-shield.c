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
#include <linux/kthread.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>

#include <linux/mcuio.h>
#include <linux/mcuio_ids.h>
#include <linux/mcuio-proto.h>

#include "mcuio-internal.h"

static int mcuio_js_probe(struct mcuio_device *mdev)
{
	return -ENODEV;
}

static int mcuio_js_remove(struct mcuio_device *mdev)
{
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
