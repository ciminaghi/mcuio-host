#ifndef __MCUIO_INTERNAL_H__
#define __MCUIO_INTERNAL_H__

#include <linux/version.h>

extern struct bus_type mcuio_bus_type;
extern struct device mcuio_bus;

struct mcuio_request;

typedef void (*request_cb)(struct mcuio_request *);

/*
 * This represents an mcuio request
 * @hc: pointer to host controller mcuio device
 * @dev: destination device
 * @func: destination function
 * @offset: offset within function address space
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

#endif /* __MCUIO_INTERNAL_H__ */
