/*
 * arch/arm/mach-tegra/board-enterprise-baseband.c
 *
 * Copyright (c) 2011, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/resource.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/err.h>
#include <linux/device.h>
#include <linux/usb.h>
#include <linux/wakelock.h>
#include <linux/platform_data/tegra_usb.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <mach/pinmux.h>
#include <mach/usb_phy.h>
#include "devices.h"
#include "gpio-names.h"

#define ENABLE_AUTO_SUSPEND     0
#define ENABLE_HOST_RECOVERY    0	/* flashless only feature */

/* Tegra3 BB GPIO */
#define MODEM_PWR_ON    TEGRA_GPIO_PE0
#define MODEM_RESET     TEGRA_GPIO_PE1
#define BB_RST_OUT      TEGRA_GPIO_PV1

/* PH450 GPIO */
#define AP2MDM_ACK      TEGRA_GPIO_PE3
#define MDM2AP_ACK      TEGRA_GPIO_PU5
#define AP2MDM_ACK2     TEGRA_GPIO_PE2
#define MDM2AP_ACK2     TEGRA_GPIO_PV0

struct ph450_priv {
	unsigned int wake_gpio;
	unsigned int wake_cnt;
	unsigned int restart_gpio;
	struct mutex lock;
	struct wake_lock wake_lock;
	unsigned int vid;
	unsigned int pid;
	struct usb_device *udev;
	struct usb_interface *intf;
	struct workqueue_struct *wq;
	struct delayed_work reset_work;
};

static struct usb_device_id modem_list[] = {
	{
	 .match_flags = USB_DEVICE_ID_MATCH_VENDOR,
	 .idVendor = 0x1983,
	 },
	{},
};

static int ph450_phy_on(void);
static int ph450_phy_off(void);

static struct ph450_priv ph450_priv;
static struct tegra_ulpi_trimmer e1219_trimmer = { 10, 1, 1, 1 };

static struct tegra_ulpi_config ehci2_null_ulpi_phy_config = {
	.trimmer = &e1219_trimmer,
	.post_phy_on = ph450_phy_on,
	.pre_phy_off = ph450_phy_off,
};

static struct tegra_ehci_platform_data ehci2_null_ulpi_platform_data = {
	.operating_mode = TEGRA_USB_HOST,
	.power_down_on_bus_suspend = 0,
	.phy_config = &ehci2_null_ulpi_phy_config,
	.phy_type = TEGRA_USB_PHY_TYPE_NULL_ULPI,
};

static int __init tegra_null_ulpi_init(void)
{
	tegra_ehci2_device.dev.platform_data = &ehci2_null_ulpi_platform_data;
	platform_device_register(&tegra_ehci2_device);
	return 0;
}

static void device_add_handler(struct usb_device *udev)
{
	const struct usb_device_descriptor *desc = &udev->descriptor;
	struct usb_interface *intf = usb_ifnum_to_if(udev, 0);

	if (usb_match_id(intf, modem_list)) {
		pr_info("Add device %d <%s %s>\n", udev->devnum,
			udev->manufacturer, udev->product);

		mutex_lock(&ph450_priv.lock);
		ph450_priv.udev = udev;
		ph450_priv.intf = intf;
		ph450_priv.vid = desc->idVendor;
		ph450_priv.pid = desc->idProduct;
		ph450_priv.wake_cnt = 0;
		mutex_unlock(&ph450_priv.lock);

		pr_info("persist_enabled: %u\n", udev->persist_enabled);

#if ENABLE_AUTO_SUSPEND
		usb_enable_autosuspend(udev);
		pr_info("enable autosuspend for %s %s\n", udev->manufacturer,
			udev->product);
#endif
	}
}

static void device_remove_handler(struct usb_device *udev)
{
	const struct usb_device_descriptor *desc = &udev->descriptor;

	if (desc->idVendor == ph450_priv.vid
	    && desc->idProduct == ph450_priv.pid) {
		pr_info("Remove device %d <%s %s>\n", udev->devnum,
			udev->manufacturer, udev->product);

		mutex_lock(&ph450_priv.lock);
		ph450_priv.udev = NULL;
		ph450_priv.intf = NULL;
		ph450_priv.vid = 0;
		mutex_unlock(&ph450_priv.lock);

#if ENABLE_HOST_RECOVERY
		queue_delayed_work(ph450_priv.wq, &ph450_priv.reset_work,
				   HZ * 10);
#endif
	}
}

static int usb_notify(struct notifier_block *self, unsigned long action,
		      void *blob)
{
	switch (action) {
	case USB_DEVICE_ADD:
		device_add_handler(blob);
		break;
	case USB_DEVICE_REMOVE:
		device_remove_handler(blob);
		break;
	}

	return NOTIFY_OK;
}

static struct notifier_block usb_nb = {
	.notifier_call = usb_notify,
};

static irqreturn_t mdm_start_thread(int irq, void *data)
{
	struct ph450_priv *priv = (struct ph450_priv *)data;

	if (gpio_get_value(priv->restart_gpio)) {
		pr_info("BB_RST_OUT high\n");
		/* hold wait lock to complete the enumeration */
		wake_lock_timeout(&priv->wake_lock, HZ * 2);
	} else {
		pr_info("BB_RST_OUT low\n");
	}

	return IRQ_HANDLED;
}

static irqreturn_t mdm_wake_thread(int irq, void *data)
{
	struct ph450_priv *priv = (struct ph450_priv *)data;

	wake_lock_timeout(&priv->wake_lock, HZ);
	mutex_lock(&priv->lock);
	if (priv->udev) {
		usb_lock_device(priv->udev);
		pr_info("mdm wake (%u)\n", ++(priv->wake_cnt));
		if (usb_autopm_get_interface(priv->intf) == 0)
			usb_autopm_put_interface_async(priv->intf);
		usb_unlock_device(priv->udev);
	}
	mutex_unlock(&priv->lock);

	return IRQ_HANDLED;
}

static int ph450_reset(void)
{
	pr_info("modem reset\n");

	gpio_set_value(AP2MDM_ACK2, 1);
	gpio_set_value(MODEM_PWR_ON, 0);
	gpio_set_value(MODEM_RESET, 0);
	mdelay(200);
	gpio_set_value(MODEM_RESET, 1);
	mdelay(30);
	gpio_set_value(MODEM_PWR_ON, 1);

	return 0;
}

static int ph450_phy_on(void)
{
	/* set AP2MDM_ACK2 low */
	gpio_set_value(AP2MDM_ACK2, 0);
	pr_info("%s\n", __func__);
	return 0;
}

static int ph450_phy_off(void)
{
	/* set AP2MDM_ACK2 high */
	gpio_set_value(AP2MDM_ACK2, 1);
	pr_info("%s\n", __func__);
	return 0;
}

static void ph450_reset_work_handler(struct work_struct *ws)
{
	struct ph450_priv *priv = container_of(ws, struct ph450_priv,
					       reset_work.work);

	mutex_lock(&priv->lock);
	if (!priv->udev)	/* assume modem crashed */
		ph450_reset();
	mutex_unlock(&priv->lock);
}

static int __init ph450_init(void)
{
	int irq;
	int ret;

	ret = gpio_request(MODEM_PWR_ON, "mdm_power");
	if (ret)
		return ret;

	ret = gpio_request(MODEM_RESET, "mdm_reset");
	if (ret) {
		gpio_free(MODEM_PWR_ON);
		return ret;
	}
	ret = gpio_request(AP2MDM_ACK2, "ap2mdm_ack2");
	if (ret) {
		gpio_free(MODEM_PWR_ON);
		gpio_free(MODEM_RESET);
		return ret;
	}
	ret = gpio_request(MDM2AP_ACK2, "mdm2ap_ack2");
	if (ret) {
		gpio_free(MODEM_PWR_ON);
		gpio_free(MODEM_RESET);
		gpio_free(AP2MDM_ACK2);
		return ret;
	}

	ret = gpio_request(BB_RST_OUT, "bb_rst_out");
	if (ret) {
		gpio_free(MODEM_PWR_ON);
		gpio_free(MODEM_RESET);
		gpio_free(AP2MDM_ACK2);
		gpio_free(MDM2AP_ACK2);
		return ret;
	}

	/* enable pull-up for ULPI STP */
	tegra_pinmux_set_pullupdown(TEGRA_PINGROUP_ULPI_STP,
				    TEGRA_PUPD_PULL_UP);

	/* enable pull-up for MDM2AP_ACK2 and BB_RST_OUT */
	tegra_pinmux_set_pullupdown(TEGRA_PINGROUP_GPIO_PV0,
				    TEGRA_PUPD_PULL_UP);

	tegra_pinmux_set_pullupdown(TEGRA_PINGROUP_GPIO_PV1,
				    TEGRA_PUPD_PULL_UP);

	tegra_gpio_enable(MODEM_PWR_ON);
	tegra_gpio_enable(MODEM_RESET);
	tegra_gpio_enable(AP2MDM_ACK2);
	tegra_gpio_enable(MDM2AP_ACK2);
	tegra_gpio_enable(BB_RST_OUT);

	gpio_direction_output(MODEM_PWR_ON, 0);
	gpio_direction_output(MODEM_RESET, 0);
	gpio_direction_output(AP2MDM_ACK2, 1);
	gpio_direction_input(MDM2AP_ACK2);
	gpio_direction_input(BB_RST_OUT);

	/* phy init */
	tegra_null_ulpi_init();

	ph450_priv.wake_gpio = TEGRA_GPIO_PV0;
	ph450_priv.restart_gpio = TEGRA_GPIO_PV1;

	mutex_init(&(ph450_priv.lock));
	wake_lock_init(&(ph450_priv.wake_lock), WAKE_LOCK_SUSPEND,
		       "mdm_wake_lock");

	/* create work queue */
	ph450_priv.wq = create_workqueue("mdm_queue");
	INIT_DELAYED_WORK(&(ph450_priv.reset_work), ph450_reset_work_handler);

	usb_register_notify(&usb_nb);

	/* enable IRQ for BB_RST_OUT */
	irq = gpio_to_irq(TEGRA_GPIO_PV1);

	ret = request_threaded_irq(irq, NULL, mdm_start_thread,
				   IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
				   "mdm_start", &ph450_priv);
	if (ret < 0) {
		pr_err("%s: request_threaded_irq error\n", __func__);
		return ret;
	}

	ret = enable_irq_wake(irq);
	if (ret) {
		pr_err("%s: enable_irq_wake error\n", __func__);
		free_irq(irq, &ph450_priv);
		return ret;
	}

	/* enable IRQ for MDM2AP_ACK2 */
	irq = gpio_to_irq(TEGRA_GPIO_PV0);

	ret = request_threaded_irq(irq, NULL, mdm_wake_thread,
				   IRQF_TRIGGER_FALLING, "mdm_wake",
				   &ph450_priv);
	if (ret < 0) {
		pr_err("%s: request_threaded_irq error\n", __func__);
		return ret;
	}

	ret = enable_irq_wake(irq);
	if (ret) {
		pr_err("%s: enable_irq_wake error\n", __func__);
		free_irq(irq, &ph450_priv);
		return ret;
	}

	/* reset modem */
	ph450_reset();

	return 0;
}

int __init enterprise_modem_init(void)
{
	int ret;

	ret = ph450_init();
	if (ret) {
		pr_err("modem init failed\n");
		return ret;
	}

	return 0;
}
