/*
 * drivers/misc/tegra-baseband/bb-power.c
 *
 * Copyright (C) 2011 NVIDIA Corporation
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/usb.h>
#include <linux/uaccess.h>
#include <linux/platform_data/tegra_usb.h>
#include <mach/usb_phy.h>
#include <mach/tegra-bb-power.h>
#include "bb-power.h"

static struct tegra_bb_callback *callback;
static int attr_load_val;
static struct tegra_bb_power_mdata *mdata;
static bb_get_cblist get_cblist[] = {
	NULL,
	NULL,
	NULL,
	M7400_CB,
};

static int tegra_bb_power_gpio_init(struct tegra_bb_power_gdata *gdata)
{
	int ret;
	int irq;
	unsigned gpio_id;
	const char *gpio_label;
	unsigned long gpio_flags;
	struct tegra_bb_gpio_data *gpiolist;
	struct tegra_bb_gpio_irqdata *gpioirq;

	gpiolist = gdata->gpio;
	for (; gpiolist->data.gpio != GPIO_INVALID; ++gpiolist) {
		gpio_id = (gpiolist->data.gpio);
		gpio_label = (gpiolist->data.label);
		gpio_flags = (gpiolist->data.flags);

		/* Request the gpio */
		ret = gpio_request(gpio_id, gpio_label);
		if (ret) {
			pr_err("%s: Error: gpio_request for gpio %d failed.\n",
							 __func__, gpio_id);
			return ret;
		}

		/* Set gpio direction, as requested */
		if (gpio_flags == GPIOF_IN)
			gpio_direction_input(gpio_id);
		else
			gpio_direction_output(gpio_id, (!gpio_flags ? 0 : 1));

		/* Enable the gpio */
		tegra_gpio_enable(gpio_id);

		/* Create a sysfs node, if requested */
		if (gpiolist->doexport)
			gpio_export(gpio_id, false);
	}

	gpioirq = gdata->gpioirq;
	for (; gpioirq->id != GPIO_INVALID; ++gpioirq) {

		/* Create interrupt handler, if requested */
		if (gpioirq->handler != NULL) {
			irq = gpio_to_irq(gpioirq->id);
			ret = request_threaded_irq(irq, NULL, gpioirq->handler,
				gpioirq->flags, gpioirq->name, gpioirq->cookie);
			if (ret < 0) {
				pr_err("%s: Error: threaded_irq req fail.\n"
								, __func__);
				return ret;
			}

			if (gpioirq->wake_capable) {
				ret = enable_irq_wake(irq);
				if (ret) {
					pr_err("%s: Error: irqwake req fail.\n",
								__func__);
					return ret;
				}
			}
		}
	}
	return 0;
}

static int tegra_bb_power_gpio_deinit(struct tegra_bb_power_gdata *gdata)
{
	struct tegra_bb_gpio_data *gpiolist;
	struct tegra_bb_gpio_irqdata *gpioirq;

	gpiolist = gdata->gpio;
	for (; gpiolist->data.gpio != GPIO_INVALID; ++gpiolist) {

		/* Free the gpio */
		gpio_free(gpiolist->data.gpio);
	}

	gpioirq = gdata->gpioirq;
	for (; gpioirq->id != GPIO_INVALID; ++gpioirq) {

		/* Free the irq */
		free_irq(gpio_to_irq(gpioirq->id), gpioirq->cookie);
	}
	return 0;
}

static ssize_t tegra_bb_attr_write(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	int val;

	if (sscanf(buf, "%d", &val) != 1)
		return -EINVAL;

	if (callback && callback->attrib) {
		if (!callback->attrib(dev, val))
			attr_load_val = val;
	}
	return count;
}

static ssize_t tegra_bb_attr_read(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d", attr_load_val);
}

static DEVICE_ATTR(load, S_IRUSR | S_IWUSR | S_IRGRP,
		tegra_bb_attr_read, tegra_bb_attr_write);

static void tegra_usbdevice_added(struct usb_device *udev)
{
	const struct usb_device_descriptor *desc = &udev->descriptor;

	if (desc->idVendor == mdata->vid &&
	    desc->idProduct == mdata->pid) {
		pr_debug("%s: Device %s added.\n", udev->product, __func__);

		if (mdata->wake_capable)
			device_set_wakeup_enable(&udev->dev, true);
		if (mdata->autosuspend_ready)
			usb_enable_autosuspend(udev);
		if (mdata->reg_cb)
			mdata->reg_cb(udev);
	}
}

static void tegra_usbdevice_removed(struct usb_device *udev)
{
	const struct usb_device_descriptor *desc = &udev->descriptor;

	if (desc->idVendor == mdata->vid &&
	    desc->idProduct == mdata->pid) {
		pr_debug("%s: Device %s removed.\n", udev->product, __func__);
	}
}

static int tegra_usb_notify(struct notifier_block *self, unsigned long action,
		      void *dev)
{
	switch (action) {
	case USB_DEVICE_ADD:
		tegra_usbdevice_added((struct usb_device *)dev);
		break;
	case USB_DEVICE_REMOVE:
		tegra_usbdevice_removed((struct usb_device *)dev);
		break;
	}
	return NOTIFY_OK;
}

static struct notifier_block tegra_usb_nb = {
	.notifier_call = tegra_usb_notify,
};

static int tegra_bb_power_probe(struct platform_device *device)
{
	struct device *dev = &device->dev;
	struct tegra_bb_pdata *pdata;
	struct tegra_bb_power_data *data;
	struct tegra_bb_power_gdata *gdata;
	int err;
	unsigned int bb_id;

	pdata = (struct tegra_bb_pdata *) dev->platform_data;
	if (!pdata) {
		pr_err("%s - Error: platform data is empty.\n", __func__);
		return -ENODEV;
	}

	/* Obtain BB specific callback list */
	bb_id = pdata->bb_id;
	if (get_cblist[bb_id] != NULL) {
		callback = (struct tegra_bb_callback *) get_cblist[bb_id]();
		if (callback && callback->init) {
			data = (struct tegra_bb_power_data *)
			callback->init((void *)pdata);

			gdata = data->gpio_data;
			if (!gdata) {
				pr_err("%s - Error: Gpio data is empty.\n",
								__func__);
				return -ENODEV;
			}

			/* Initialize gpio as required */
			tegra_bb_power_gpio_init(gdata);

			mdata = data->modem_data;
			if (mdata && mdata->vid && mdata->pid)
				/* Register to notifications from usb core */
				usb_register_notify(&tegra_usb_nb);
		} else {
			pr_err("%s - Error: init callback is empty.\n",
								__func__);
			return -ENODEV;
		}
	} else {
		pr_err("%s - Error: callback data is empty.\n", __func__);
		return -ENODEV;
	}

	/* Create the control sysfs node */
	err = device_create_file(dev, &dev_attr_load);
	if (err < 0) {
		pr_err("%s - Error: device_create_file failed.\n", __func__);
		return -ENODEV;
	}
	attr_load_val = 0;

	return 0;
}

static int tegra_bb_power_remove(struct platform_device *device)
{
	struct device *dev = &device->dev;
	struct tegra_bb_power_data *data;
	struct tegra_bb_power_gdata *gdata;

	/* BB specific callback */
	if (callback && callback->deinit) {
		data = (struct tegra_bb_power_data *)
		callback->deinit();

		/* Deinitialize gpios */
		gdata = data->gpio_data;
		if (gdata)
			tegra_bb_power_gpio_deinit(gdata);
		else {
			pr_err("%s - Error: Gpio data is empty.\n", __func__);
			return -ENODEV;
		}

		mdata = data->modem_data;
		if (mdata && mdata->vid && mdata->pid)
			/* Register to notifications from usb core */
			usb_unregister_notify(&tegra_usb_nb);
	}

	/* Remove the control sysfs node */
	device_remove_file(dev, &dev_attr_load);

	return 0;
}

#ifdef CONFIG_PM
static int tegra_bb_power_suspend(struct platform_device *device,
	pm_message_t state)
{
	/* BB specific callback */
	if (callback && callback->power)
		callback->power(PWRSTATE_L2L3);
	return 0;
}

static int tegra_bb_power_resume(struct platform_device *device)
{
	/* BB specific callback */
	if (callback && callback->power)
		callback->power(PWRSTATE_L3L0);
	return 0;
}
#endif

static struct platform_driver tegra_bb_power_driver = {
	.probe = tegra_bb_power_probe,
	.remove = tegra_bb_power_remove,
#ifdef CONFIG_PM
	.suspend = tegra_bb_power_suspend,
	.resume = tegra_bb_power_resume,
#endif
	.driver = {
		.name = "tegra_baseband_power",
	},
};

static int __init tegra_baseband_power_init(void)
{
	pr_debug("%s\n", __func__);
	return platform_driver_register(&tegra_bb_power_driver);
}

static void __exit tegra_baseband_power_exit(void)
{
	pr_debug("%s\n", __func__);
	platform_driver_unregister(&tegra_bb_power_driver);
}

module_init(tegra_baseband_power_init)
module_exit(tegra_baseband_power_exit)
MODULE_AUTHOR("NVIDIA Corporation");
MODULE_DESCRIPTION("Tegra modem power management driver");
MODULE_LICENSE("GPL");
