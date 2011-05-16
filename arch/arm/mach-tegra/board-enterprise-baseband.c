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
#include <linux/err.h>
#include <linux/platform_data/tegra_usb.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <mach/pinmux.h>
#include <mach/usb_phy.h>
#include "devices.h"
#include "gpio-names.h"

/* T30 BB GPIO */
#define MODEM_PWR_ON	TEGRA_GPIO_PE0
#define MODEM_RESET	TEGRA_GPIO_PE1

/* PH450 */
#define AP2MDM_ACK	TEGRA_GPIO_PE3
#define MDM2AP_ACK	TEGRA_GPIO_PU5
#define AP2MDM_ACK2	TEGRA_GPIO_PE2
#define MDM2AP_ACK2	TEGRA_GPIO_PV0

static int ph450_reset(void);
static int ph450_handshake(void);

static struct tegra_ulpi_trimmer e1219_trimmer = { 10, 1, 1, 1 };

static struct tegra_ulpi_config ehci2_null_ulpi_phy_config = {
	.trimmer = &e1219_trimmer,
	.preinit = ph450_reset,
	.postinit = ph450_handshake,
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

static int __init ph450_init(void)
{
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

	/* enable pull-up for MDM2AP_ACK2 */
	tegra_pinmux_set_pullupdown(TEGRA_PINGROUP_GPIO_PV0,
				    TEGRA_PUPD_PULL_UP);

	tegra_gpio_enable(MODEM_PWR_ON);
	tegra_gpio_enable(MODEM_RESET);
	tegra_gpio_enable(AP2MDM_ACK2);
	tegra_gpio_enable(MDM2AP_ACK2);

	gpio_direction_output(MODEM_PWR_ON, 0);
	gpio_direction_output(MODEM_RESET, 0);
	gpio_direction_output(AP2MDM_ACK2, 1);
	gpio_direction_input(MDM2AP_ACK2);

	return 0;
}

static int ph450_reset(void)
{
	int retry = 100; /* retry for 10 sec */

	gpio_set_value(AP2MDM_ACK2, 1);
	gpio_set_value(MODEM_PWR_ON, 0);
	gpio_set_value(MODEM_RESET, 0);
	mdelay(200);
	gpio_set_value(MODEM_RESET, 1);
	mdelay(30);
	gpio_set_value(MODEM_PWR_ON, 1);

	while (retry) {
		/* wait for MDM2AP_ACK2 low */
		int val = gpio_get_value(MDM2AP_ACK2);
		if (!val) {
			pr_info("MDM2AP_ACK2 detected\n");
			return 0;
		} else {
			pr_info(".");
			retry--;
			mdelay(100);
		}
	}
	return 1;
}

static int ph450_handshake(void)
{
	/* set AP2MDM_ACK2 low */
	gpio_set_value(AP2MDM_ACK2, 0);

	return 0;
}

int __init enterprise_baseband_init(void)
{
	int ret;

	ret = ph450_init();
	if (ret) {
		pr_err("modem init failed\n");
		return ret;
	}

	tegra_null_ulpi_init();
	return 0;
}
