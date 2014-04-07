#ifndef __MCUIO_INTERNAL_H__
#define __MCUIO_INTERNAL_H__

#include <linux/version.h>
#include <linux/regmap.h>

extern struct bus_type mcuio_bus_type;
extern struct device mcuio_bus;
extern struct attribute_group mcuio_default_dev_attr_group;

struct mcuio_request;

typedef void (*request_cb)(struct mcuio_request *);

/*
 * This represents an mcuio request
 * @hc: pointer to host controller mcuio device
 * @dev: destination device
 * @func: destination function
 * @offset: offset within function address space
 * @offset_mask: this mask is applied to incoming packets' offsets when
 *               looking for matching pending requests
 * @type: request type
 * @cb: pointer to callback function
 * @cb_data: callback data.
 * @status: status of request (0 completed OK, -EXXXX errors)
 * @data: request data
 * @list: used for enqueueing requests
 * @to_work: delayed_work struct for request timeout management
 * @priv: private data. FIX THIS
 * @dont_free: this flag is !0 when request shall not be kfree'd
 * @fill: if this is !0 the resulting request packet shall have its fill data
 *        flag set
 */
struct mcuio_request {
	struct mcuio_device *hc;
	unsigned dev;
	unsigned func;
	unsigned offset;
	unsigned offset_mask;
	unsigned type;
	request_cb cb;
	void *cb_data;
	int status;
	uint32_t data[2];
	struct list_head list;
	struct delayed_work to_work;
	void *priv;
	int dont_free;
	int fill;
};

int mcuio_get_bus(void);
void mcuio_put_bus(unsigned bus);

/*
 * Submit a request, block until request done
 *
 * @r: pointer to request
 */
int mcuio_submit_request(struct mcuio_request *r);

/*
 * Setup a callback for an incoming request
 *
 * @r: pointer to corresponding request
 */
int mcuio_setup_cb(struct mcuio_request *r);


/*
 * Set irq numbers for a given bus device (MCUIO_FUNCS_PER_DEV functions)
 */
int mcuio_hc_set_irqs(struct mcuio_device *hc, unsigned dev, int irqs[]);

struct regmap *regmap_init_mcuio(struct device *dev,
				 struct mcuio_device *hc,
				 const struct regmap_config *config);

#endif /* __MCUIO_INTERNAL_H__ */
