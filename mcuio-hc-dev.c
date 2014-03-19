/*
 * Copyright 2011 Dog Hunter SA
 * Author: Davide Ciminaghi <ciminaghi@gnudd.com>
 *
 * GNU GPLv2 or later
 */

/* mcuio host controller functions */

#include <linux/mcuio.h>
#include <linux/circ_buf.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/regmap.h>
#include <linux/err.h>
#include <linux/workqueue.h>
#include <linux/circ_buf.h>
#include <linux/mcuio_ids.h>

#include "mcuio-hc.h"
#include "mcuio-internal.h"

static struct mcuio_device_id default_hc_id = {
	.device = 0,
	.vendor = 0,
	.class = MCUIO_CLASS_HOST_CONTROLLER,
};

struct device *mcuio_add_hc_device(struct mcuio_device_id *id,
				   struct mcuio_hc_platform_data *plat)
{
	int b, ret = -ENOMEM;
	struct mcuio_device *d = kzalloc(sizeof(*d), GFP_KERNEL);
	if (!d)
		return ERR_PTR(-ENOMEM);
	b = mcuio_get_bus();
	if (b < 0) {
		ret = b;
		goto err0;
	}
	d->bus = b;
	d->device = 0;
	d->fn = 0;
	d->id = id ? *id : default_hc_id;
	d->dev.platform_data = plat;
	ret = mcuio_device_register(d, NULL, NULL);
	if (ret < 0)
		goto err1;
	return &d->dev;

err1:
	mcuio_put_bus(b);
err0:
	kfree(d);
	return ERR_PTR(ret);
}
EXPORT_SYMBOL(mcuio_add_hc_device);

MODULE_VERSION(GIT_VERSION); /* Defined in local Makefile */
MODULE_AUTHOR("Davide Ciminaghi, some code from ZIO by Vaga, Rubini");
MODULE_DESCRIPTION("MCUIO host controller code");
MODULE_LICENSE("GPL v2");
