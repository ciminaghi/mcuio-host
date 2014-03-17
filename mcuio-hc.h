#ifndef __HOST_CONTROLLER_H__
#define __HOST_CONTROLLER_H__

#define MCUIO_HC_OUTBUF 0x8
#define MCUIO_HC_INBUF 0x108
/* How many 32bits words are in rx buffer */
#define MCUIO_RX_CNT	0x208
/* Irq register */
#define MCUIO_IRQ	0x20c
/* Irq status */
#define MCUIO_IRQ_STAT  0x210
#define RX_RDY		0x1
/* Clear register */
#define MCUIO_IRQ_CLR	0x214

#define MCUIO_HC_MAX_REGISTER 0x214

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

/*
 * mcuio_add_hc_device() : add host controller device
 *
 * @id: pointer to mcuio device's id
 * @plat: pointer to platform data structure
 * @release: pointer to custom release function
 */
struct device *mcuio_add_hc_device(struct mcuio_device_id *id,
				   struct mcuio_hc_platform_data *plat,
				   void (*release)(struct device *));

/*
 * mcuio_del_hc_device(): remove host controller device
 *
 * @dev: pointer to relevant struct device
 */
void mcuio_del_hc_device(struct device *);

/*
 * mcuio_hc_dev_default_release(): default release for hc devices
 *
 * @dev: pointer to corresponding device to be deleted
 *
 * If an hc is created with custom release function, such function can
 * invoke mcuio_hc_dev_default_release() to perform some common release
 * operations (freeing memory and releasing the mcuio bus number)
 */
void mcuio_hc_dev_default_release(struct device *dev);


#endif /* __HOST_CONTROLLER_H__ */
