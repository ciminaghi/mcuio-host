/*
 * Regmap for remote mcuio devices (not living on this machine)
 * Presently, all mcuio devices are remote devices except for the host controller
 * Code comes from regmap-mmio
 */

#include <linux/err.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/regmap.h>

#include <linux/mcuio.h>
#include <linux/mcuio-proto.h>

#include "mcuio-internal.h"

/**
 * mcuio bus context
 * @hc: pointer to host controller
 * @dev: device number of mcuio device
 * @func: function number of mcuio device
 */
struct regmap_mcuio_context {
	struct mcuio_device *hc;
	unsigned dev;
	unsigned func;
	unsigned val_bytes;
};

typedef void (*copyf)(void *, const void *);

static void copyb(void *dst, const void *src)
{
	*(u8 *)dst = *(u8 *)src;
}

static void copyw(void *dst, const void *src)
{
	*(u16 *)dst = *(u16 *)src;
}

static void copydw(void *dst, const void *src)
{
	*(u32 *)dst = *(u32 *)src;
}

static void copyq(void *dst, const void *src)
{
	*(u64 *)dst = *(u64 *)src;
}

static int regmap_mcuio_gather_write(void *context,
				     const void *reg, size_t reg_size,
				     const void *val, size_t val_size)
{
	struct regmap_mcuio_context *ctx = context;
	struct mcuio_request r;
	u32 offset;
	unsigned t;
	int ret;
	copyf f;

	BUG_ON(reg_size != 4);

	offset = *(u32 *)reg;

	r.hc = ctx->hc;
	r.dev = ctx->dev;
	r.func = ctx->func;
	r.offset = offset;
	r.offset_mask = 0xffff;

	switch (ctx->val_bytes) {
	case 1:
		t = mcuio_type_wrb;
		f = copyb;
		break;
	case 2:
		t = mcuio_type_wrw;
		f = copyw;
		break;
	case 4:
		t = mcuio_type_wrdw;
		f = copydw;
		break;
	case 8:
		t = mcuio_type_wrq;
		f = copyq;
		break;
	default:
		BUG();
	}

	while (val_size) {
		r.type = t;
		r.hc = ctx->hc;
		r.dev = ctx->dev;
		r.func = ctx->func;
		r.offset = offset;
		r.fill = 0;
		f(r.data, val);
		ret = mcuio_submit_request(&r);
		if (ret)
			break;
		val_size -= ctx->val_bytes;
		val += ctx->val_bytes;
		offset += ctx->val_bytes;
	}

	return ret;
}

static int regmap_mcuio_write(void *context, const void *data, size_t count)
{
	BUG_ON(count < 4);

	return regmap_mcuio_gather_write(context, data, 4, data + 4, count - 4);
}

static int regmap_mcuio_read(void *context,
			     const void *reg, size_t reg_size,
			     void *val, size_t val_size)
{
	struct regmap_mcuio_context *ctx = context;
	struct mcuio_request r;
	u32 offset = *(u32 *)reg;
	int ret;
	copyf f;
	unsigned t;

	BUG_ON(reg_size != 4);
	
	switch (ctx->val_bytes) {
	case 1:
		t = mcuio_type_rdb;
		f = copyb;
		break;
	case 2:
		t = mcuio_type_rdw;
		f = copyw;
		break;
	case 4:
		t = mcuio_type_rddw;
		f = copydw;
		break;
	case 8:
		t = mcuio_type_rdq;
		f = copyq;
		break;
	default:
		return -EINVAL;
	}
	while (val_size) {
		r.type = t;
		r.hc = ctx->hc;
		r.dev = ctx->dev;
		r.func = ctx->func;
		r.offset = offset;
		r.offset_mask = 0xffff;
		r.status = -ETIMEDOUT;
		r.dont_free = 1;
		r.fill = 0;
		ret = mcuio_submit_request(&r);
		if (ret)
			break;
		f(val, r.data);
		val_size -= ctx->val_bytes;
		val += ctx->val_bytes;
		offset += ctx->val_bytes;
	}
	return ret;
}


static void regmap_mcuio_free_context(void *context)
{
	struct regmap_mcuio_context *ctx = context;
	kfree(ctx);
}

static struct regmap_bus regmap_mcuio = {
	.fast_io = false,
	.write = regmap_mcuio_write,
	.read = regmap_mcuio_read,
	.free_context = regmap_mcuio_free_context,
	.reg_format_endian_default = REGMAP_ENDIAN_NATIVE,
	.val_format_endian_default = REGMAP_ENDIAN_NATIVE,
};

static struct regmap_mcuio_context *
regmap_mcuio_setup_context(struct device *dev,
			   struct mcuio_device *hc,
			   const struct regmap_config *config)
{
	struct mcuio_device *mdev = to_mcuio_dev(dev);
	struct regmap_mcuio_context *ctx;
	int min_stride;

	if (config->reg_bits != 32)
		return ERR_PTR(-EINVAL);

	if (config->pad_bits)
		return ERR_PTR(-EINVAL);

	switch (config->val_bits) {
	case 8:
		/* The core treats 0 as 1 */
		min_stride = 0;
		break;
	case 16:
		min_stride = 2;
		break;
	case 32:
		min_stride = 4;
		break;
#ifdef CONFIG_64BIT
	case 64:
		min_stride = 8;
		break;
#endif
		break;
	default:
		return ERR_PTR(-EINVAL);
	}

	if (config->reg_stride < min_stride)
		return ERR_PTR(-EINVAL);

	switch (config->reg_format_endian) {
	case REGMAP_ENDIAN_DEFAULT:
	case REGMAP_ENDIAN_NATIVE:
		break;
	default:
		return ERR_PTR(-EINVAL);
	}

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return ERR_PTR(-ENOMEM);

	ctx->hc = hc;
	ctx->dev = mdev->device;
	ctx->func = mdev->fn;
	ctx->val_bytes = config->val_bits / 8;
	return ctx;
}


/**
 * regmap_init_mcuio(): Initialise mcuio register map
 *
 * @dev: Device that will be interacted with
 * @hc: mcuio system controller
 * @config: Configuration for register map
 *
 * The return value will be an ERR_PTR() on error or a valid pointer to
 * a struct regmap.
 */
struct regmap *regmap_init_mcuio(struct device *dev,
				 struct mcuio_device *hc,
				 const struct regmap_config *config)
{
	struct regmap_mcuio_context *ctx;
	ctx = regmap_mcuio_setup_context(dev, hc, config);
	if (IS_ERR(ctx))
		return ERR_CAST(ctx);

	return devm_regmap_init(dev, &regmap_mcuio, ctx, config);
}
EXPORT_SYMBOL_GPL(regmap_init_mcuio);


MODULE_VERSION(GIT_VERSION); /* Defined in local Makefile */
MODULE_AUTHOR("Davide Ciminaghi, some code from ZIO by Vaga, Rubini");
MODULE_DESCRIPTION("MCUIO bus regmap implementation");
MODULE_LICENSE("GPL v2");
