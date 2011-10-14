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
#include <linux/uaccess.h>
#include <linux/platform_data/tegra_usb.h>
#include <mach/usb_phy.h>
#include <mach/tegra-bb-power.h>
#include "bb-power.h"

static int bb_id;
static bool bb_registered;

static bb_init_cb init_cb_list[] = {
	NULL,
	NULL,
	NULL,
	M7400_INIT_CB,
};

static bb_power_cb power_cb_list[] = {
	NULL,
	NULL,
	NULL,
	M7400_PWR_CB,
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
			pr_err("%s: gpio_request for gpio %d failed.\n",
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
				pr_err("%s: request_threaded_irq error\n",
								 __func__);
				return ret;
			}
			ret = enable_irq_wake(irq);
			if (ret) {
				pr_err("%s: enable_irq_wake error\n", __func__);
				return ret;
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

static int baseband_l2_suspend(void)
{
	/* BB specific callback */
	if (power_cb_list[bb_id] != NULL)
		power_cb_list[bb_id](CB_CODE_L0L2);
	return 0;
}

static int baseband_l2_resume(void)
{
	/* BB specific callback */
	if (power_cb_list[bb_id] != NULL)
		power_cb_list[bb_id](CB_CODE_L2L0);
	return 0;
}

static ssize_t tegra_bb_attr_write(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct tegra_bb_pdata *pdata;
	struct tegra_ehci_platform_data *ehci_data;
	struct tegra_uhsic_config *hsic_config;
	int load;

	if (sscanf(buf, "%d", &load) != 1)
		return -EINVAL;

	if (load == 1 && !bb_registered) {
		pdata = (struct tegra_bb_pdata *) dev->platform_data;
		ehci_data = (struct tegra_ehci_platform_data *)
					pdata->device->dev.platform_data;
		hsic_config = (struct tegra_uhsic_config *)
					ehci_data->phy_config;

		/* Register PHY callbacks */
		hsic_config->postsuspend = baseband_l2_suspend;
		hsic_config->preresume = baseband_l2_resume;

		/* Override required settings */
		ehci_data->power_down_on_bus_suspend = 0;

		/* Register the ehci device. */
		platform_device_register(pdata->device);
		bb_registered = true;
	}

	return count;
}

static ssize_t tegra_bb_attr_read(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int ret = 0;
	return sprintf(buf, "%d", ret);
}

static DEVICE_ATTR(load, S_IRUSR | S_IWUSR | S_IRGRP,
		tegra_bb_attr_read, tegra_bb_attr_write);

static int tegra_bb_power_probe(struct platform_device *device)
{
	struct device *dev = &device->dev;
	struct tegra_bb_pdata *pdata;
	struct tegra_bb_power_gdata *gdata;
	int err;

	pdata = (struct tegra_bb_pdata *) dev->platform_data;
	if (!pdata) {
		pr_err("%s - Error: platform data is empty.\n", __func__);
		return -ENODEV;
	}

	/* BB specific callback */
	bb_id = pdata->bb_id;
	if (init_cb_list[bb_id] != NULL) {
		gdata = (struct tegra_bb_power_gdata *)
		init_cb_list[pdata->bb_id]((void *)pdata, CB_CODE_INIT);

		if (!gdata) {
			pr_err("%s - Error: Gpio data is empty.\n", __func__);
			return -ENODEV;
		}

		/* Initialize gpio as required */
		tegra_bb_power_gpio_init(gdata);
	}

	bb_registered = false;

	/* Create the control sysfs node */
	err = device_create_file(dev, &dev_attr_load);
	if (err < 0) {
		pr_err("%s - device_create_file failed\n", __func__);
		return -ENODEV;
	}

	return 0;
}

static int tegra_bb_power_remove(struct platform_device *device)
{
	struct device *dev = &device->dev;
	struct tegra_bb_pdata *pdata;
	struct tegra_bb_power_gdata *gdata;

	/* BB specific callback */
	if (init_cb_list[bb_id] != NULL) {
		pdata = (struct tegra_bb_pdata *) dev->platform_data;
		gdata = (struct tegra_bb_power_gdata *)
			init_cb_list[bb_id]((void *)pdata, CB_CODE_DEINIT);

		/* Deinitialize gpios */
		if (gdata)
			tegra_bb_power_gpio_deinit(gdata);
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
	if (power_cb_list[bb_id] != NULL)
		power_cb_list[bb_id](CB_CODE_L2L3);

	return 0;
}

static int tegra_bb_power_resume(struct platform_device *device)
{
	/* BB specific callback */
	if (power_cb_list[bb_id] != NULL)
		power_cb_list[bb_id](CB_CODE_L3L0);

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
