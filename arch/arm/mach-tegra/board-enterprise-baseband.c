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
#include <linux/wakelock.h>
#include <linux/platform_data/tegra_usb.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <mach/pinmux.h>
#include <mach/usb_phy.h>
#include <mach/tegra_usb_modem_power.h>
#include "devices.h"
#include "gpio-names.h"

/* Tegra3 BB GPIO */
#define MODEM_PWR_ON    TEGRA_GPIO_PE0
#define MODEM_RESET     TEGRA_GPIO_PE1
#define BB_RST_OUT      TEGRA_GPIO_PV1

/* Icera 450 GPIO */
#define AP2MDM_ACK      TEGRA_GPIO_PE3
#define MDM2AP_ACK      TEGRA_GPIO_PU5
#define AP2MDM_ACK2     TEGRA_GPIO_PE2
#define MDM2AP_ACK2     TEGRA_GPIO_PV0

static struct wake_lock mdm_wake_lock;

static struct gpio modem_gpios[] = {
	{MODEM_PWR_ON, GPIOF_OUT_INIT_LOW, "MODEM PWR ON"},
	{MODEM_RESET, GPIOF_IN, "MODEM RESET"},
	{BB_RST_OUT, GPIOF_IN, "BB RST OUT"},
	{AP2MDM_ACK2, GPIOF_OUT_INIT_HIGH, "AP2MDM ACK2"},
};

static int ph450_phy_on(void);
static int ph450_phy_off(void);

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

static irqreturn_t mdm_start_thread(int irq, void *data)
{
	if (gpio_get_value(BB_RST_OUT)) {
		pr_info("BB_RST_OUT high\n");
	} else {
		pr_info("BB_RST_OUT low\n");
		/* hold wait lock to complete the enumeration */
		wake_lock_timeout(&mdm_wake_lock, HZ * 10);
	}

	return IRQ_HANDLED;
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

static void ph450_reset(void)
{
	pr_info("%s\n", __func__);
	gpio_set_value(MODEM_PWR_ON, 0);
	mdelay(200);
	gpio_set_value(MODEM_PWR_ON, 1);
}

static int ph450_init(void)
{
	int irq;
	int ret;

	ret = gpio_request_array(modem_gpios, ARRAY_SIZE(modem_gpios));
	if (ret)
		return ret;

	/* enable pull-up for ULPI STP */
	tegra_pinmux_set_pullupdown(TEGRA_PINGROUP_ULPI_STP,
				    TEGRA_PUPD_PULL_UP);

	/* enable pull-up for MDM2AP_ACK2 */
	tegra_pinmux_set_pullupdown(TEGRA_PINGROUP_GPIO_PV0,
				    TEGRA_PUPD_PULL_UP);

	tegra_gpio_enable(MODEM_PWR_ON);
	tegra_gpio_enable(MODEM_RESET);
	tegra_gpio_enable(AP2MDM_ACK2);
	tegra_gpio_enable(BB_RST_OUT);

	/* export GPIO for user space access through sysfs */
	gpio_export(MODEM_PWR_ON, false);

	/* phy init */
	tegra_null_ulpi_init();

	wake_lock_init(&mdm_wake_lock, WAKE_LOCK_SUSPEND, "mdm_lock");

	/* enable IRQ for BB_RST_OUT */
	irq = gpio_to_irq(BB_RST_OUT);

	ret = request_threaded_irq(irq, NULL, mdm_start_thread,
				   IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
				   "mdm_start", NULL);
	if (ret < 0) {
		pr_err("%s: request_threaded_irq error\n", __func__);
		return ret;
	}

	ret = enable_irq_wake(irq);
	if (ret) {
		pr_err("%s: enable_irq_wake error\n", __func__);
		free_irq(irq, NULL);
		return ret;
	}

	return 0;
}

static const struct tegra_modem_operations ph450_operations = {
	.init = ph450_init,
	.start = ph450_reset,
	.reset = ph450_reset,
};

static struct tegra_usb_modem_power_platform_data ph450_pdata = {
	.ops = &ph450_operations,
	.wake_gpio = MDM2AP_ACK2,
	.flags = IRQF_TRIGGER_FALLING,
};

static struct platform_device icera_450_device = {
	.name = "tegra_usb_modem_power",
	.id = -1,
	.dev = {
		.platform_data = &ph450_pdata,
	},
};

int __init enterprise_modem_init(void)
{
	platform_device_register(&icera_450_device);
	return 0;
}
