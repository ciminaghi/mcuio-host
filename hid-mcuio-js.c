#include <linux/device.h>
#include <linux/hid.h>
#include <linux/input/mt.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/mcuio_ids.h>

static int mcuio_js_hid_probe(struct hid_device *hdev,
			      const struct hid_device_id *id)
{
	int ret;
	hid_dbg(hdev, "%s entered\n", __func__);
	ret= hid_parse(hdev);
	if (ret) {
		hid_err(hdev, "mcuio_js_hid parse failed\n");
		return ret;
	}
	ret = hid_hw_start(hdev,
			   HID_CONNECT_HIDINPUT|HID_CONNECT_HIDINPUT_FORCE);
	if (ret) {
		hid_err(hdev, "mcuio_js_hid hw start failed\n");
		return ret;
	}
	hid_dbg(hdev, "%s ok\n", __func__);
	return 0;
}

static void mcuio_js_hid_remove(struct hid_device *hdev)
{
	hid_hw_stop(hdev);
}

static const struct hid_device_id mcuio_js_hid[] = {
	{ HID_DEVICE(BUS_VIRTUAL, 0, MCUIO_VENDOR_DOGHUNTER,
		     MCUIO_DEVICE_JOYSTICK_SHIELD), .driver_data = 0, },
	{ }
};
MODULE_DEVICE_TABLE(hid, mcuio_js_hid);

static struct hid_driver mcuio_js_hid_driver = {
	.name = "mcuio-js-shield",
	.id_table = mcuio_js_hid,
	.probe = mcuio_js_hid_probe,
	.remove = mcuio_js_hid_remove,
};

static int __init mcuio_js_hid_init(void)
{
	int ret;

	ret = hid_register_driver(&mcuio_js_hid_driver);
	if (ret)
		pr_err("can't register magicmouse driver\n");

	return ret;
}

static void __exit mcuio_js_hid_exit(void)
{
	hid_unregister_driver(&mcuio_js_hid_driver);
}

module_init(mcuio_js_hid_init);
module_exit(mcuio_js_hid_exit);
MODULE_LICENSE("GPL");
