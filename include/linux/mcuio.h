#ifndef __MCUIO_H__
#define __MCUIO_H__

#ifdef __KERNEL__

#include <linux/device.h>
#include <linux/module.h>

struct mcuio_packet;

/*
 * Id of an mcuio device.
 */
struct mcuio_device_id {
	unsigned int device;
	unsigned int vendor;
	unsigned int class;
	unsigned int class_mask;
};

/*
 * An mcuio device.
 * @id: device id, as defined above
 * @bus: bus number
 * @device: device number (0 for host controllers)
 * @fn: function number (0 for host controllers)
 * @dev: the relevant device
 */
struct mcuio_device {
	struct mcuio_device_id id;
	unsigned bus, device, fn;
	struct device dev;
};

#define to_mcuio_dev(_dev) container_of(_dev, struct mcuio_device, dev)

/*
 * mcuio_driver -- an mcuio driver struc
 */
struct mcuio_driver {
	const struct mcuio_device_id	*id_table;
	int (*probe)(struct mcuio_device *dev);
	int (*remove)(struct mcuio_device *dev);
	int (*input_ready)(struct mcuio_device *dev);
	struct device_driver		driver;
};

#define to_mcuio_drv(_drv) container_of(_drv, struct mcuio_driver, driver)

/*
 * The parent of all mcuio controllers on this machine
 */
extern struct device mcuio_bus;

int mcuio_driver_register(struct mcuio_driver *drv, struct module *owner);
void mcuio_driver_unregister(struct mcuio_driver *drv);
int mcuio_device_register(struct mcuio_device *dev,
			  struct device_type *type,
			  struct device *parents);
void mcuio_device_unregister(struct mcuio_device *dev);

#endif /* __KERNEL__ */

#endif /* __MCUIO_H__ */
