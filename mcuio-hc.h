#ifndef __HOST_CONTROLLER_H__
#define __HOST_CONTROLLER_H__

#define MCUIO_HC_OUTBUF 0x8

typedef struct regmap *(*setup_regmap)(struct device *, void *data);

/*
 * Platform data for host controller
 *
 * @setup_regmap: pointer to function setting up a regmap for controller
 * @data: data to be passed on to setup_regmap.
 */
struct mcuio_hc_platform_data {
	setup_regmap setup_regmap;
	void *data;
};

#endif /* __HOST_CONTROLLER_H__ */
