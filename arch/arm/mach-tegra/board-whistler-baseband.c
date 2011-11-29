/*
 * arch/arm/mach-tegra/board-whistler-baseband.c
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
#include <linux/tegra_caif.h>
#include <mach/tegra_usb_modem_power.h>

#include "board.h"
#include "board-whistler-baseband.h"

static int baseband_phy_on(void);
static int baseband_phy_off(void);
static void baseband_phy_restore_start(void);
static void baseband_phy_restore_end(void);

static struct wake_lock mdm_wake_lock;

static struct gpio modem_gpios[] = {
	{MODEM_PWR_ON, GPIOF_OUT_INIT_LOW, "MODEM PWR ON"},
	{MODEM_RESET, GPIOF_IN, "MODEM RESET"},
	{BB_RST_OUT, GPIOF_IN, "BB RST OUT"},
	{MDM2AP_ACK, GPIOF_IN, "MDM2AP_ACK"},
	{AP2MDM_ACK2, GPIOF_OUT_INIT_HIGH, "AP2MDM ACK2"},
	{AP2MDM_ACK, GPIOF_OUT_INIT_LOW, "AP2MDM ACK"},
	{ULPI_STP, GPIOF_IN, "ULPI_STP"},
	{ULPI_DIR, GPIOF_OUT_INIT_LOW, "ULPI_DIR"},
	{ULPI_D0, GPIOF_OUT_INIT_LOW, "ULPI_D0"},
	{ULPI_D1, GPIOF_OUT_INIT_LOW, "ULPI_D1"},
};

static __initdata struct tegra_pingroup_config whistler_null_ulpi_pinmux[] = {
	{TEGRA_PINGROUP_UAA, TEGRA_MUX_ULPI, TEGRA_PUPD_NORMAL,
	 TEGRA_TRI_NORMAL},
	{TEGRA_PINGROUP_UAB, TEGRA_MUX_ULPI, TEGRA_PUPD_NORMAL,
	 TEGRA_TRI_NORMAL},
	{TEGRA_PINGROUP_UDA, TEGRA_MUX_ULPI, TEGRA_PUPD_NORMAL,
	 TEGRA_TRI_NORMAL},
	{TEGRA_PINGROUP_UAC, TEGRA_MUX_RSVD4, TEGRA_PUPD_NORMAL,
	 TEGRA_TRI_NORMAL},
	{TEGRA_PINGROUP_SDIO1, TEGRA_MUX_UARTA, TEGRA_PUPD_PULL_UP,
	 TEGRA_TRI_NORMAL},
};

static struct tegra_ulpi_trimmer e1219_trimmer = { 10, 1, 1, 1 };

static struct tegra_ulpi_config ehci2_null_ulpi_phy_config = {
	.trimmer = &e1219_trimmer,
	.post_phy_on = baseband_phy_on,
	.pre_phy_off = baseband_phy_off,
	.phy_restore_start = baseband_phy_restore_start,
	.phy_restore_end = baseband_phy_restore_end,
	.phy_restore_gpio = MDM2AP_ACK,
	.ulpi_dir_gpio = ULPI_DIR,
	.ulpi_d0_gpio = ULPI_D0,
	.ulpi_d1_gpio = ULPI_D1,
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

static int baseband_phy_on(void)
{
	static bool phy_init;

	if (!phy_init) {
		/* set AP2MDM_ACK2 low */
		gpio_set_value(AP2MDM_ACK2, 0);
		phy_init = true;
	}
	pr_info("%s\n", __func__);
	return 0;
}

static int baseband_phy_off(void)
{
	pr_info("%s\n", __func__);
	return 0;
}

static void baseband_phy_restore_start(void)
{
	/* set AP2MDM_ACK2 high */
	gpio_set_value(AP2MDM_ACK2, 1);
}

static void baseband_phy_restore_end(void)
{
	/* set AP2MDM_ACK2 low */
	gpio_set_value(AP2MDM_ACK2, 0);
}

static void baseband_start(void)
{
	/*
	 *  Leave baseband powered OFF.
	 *  User-space daemons will take care of powering it up.
	 */
	pr_info("%s\n", __func__);
	gpio_set_value(MODEM_PWR_ON, 0);
}

static void baseband_reset(void)
{
	/* Initiate power cycle on baseband sub system */
	pr_info("%s\n", __func__);
	gpio_set_value(MODEM_PWR_ON, 0);
	mdelay(200);
	gpio_set_value(MODEM_PWR_ON, 1);
}

static int baseband_init(void)
{
	int irq;
	int ret;

	ret = gpio_request_array(modem_gpios, ARRAY_SIZE(modem_gpios));
	if (ret)
		return ret;

	/* enable pull-up for BB_RST_OUT */
	tegra_pinmux_set_pullupdown(TEGRA_PINGROUP_UAC,
				    TEGRA_PUPD_PULL_UP);

	tegra_gpio_enable(MODEM_PWR_ON);
	tegra_gpio_enable(MODEM_RESET);
	tegra_gpio_enable(AP2MDM_ACK2);
	tegra_gpio_enable(BB_RST_OUT);
	tegra_gpio_enable(AP2MDM_ACK);
	tegra_gpio_enable(MDM2AP_ACK);
	tegra_gpio_enable(TEGRA_GPIO_PY3);
	tegra_gpio_enable(TEGRA_GPIO_PY1);
	tegra_gpio_enable(TEGRA_GPIO_PO1);
	tegra_gpio_enable(TEGRA_GPIO_PO2);

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

static const struct tegra_modem_operations baseband_operations = {
	.init = baseband_init,
	.start = baseband_start,
	.reset = baseband_reset,
};

static struct tegra_usb_modem_power_platform_data baseband_pdata = {
	.ops = &baseband_operations,
	.wake_gpio = MDM2AP_ACK2,
	.flags = IRQF_TRIGGER_FALLING,
};

static struct platform_device icera_baseband_device = {
	.name = "tegra_usb_modem_power",
	.id = -1,
	.dev = {
		.platform_data = &baseband_pdata,
	},
};

int  __init whistler_baseband_init(void)
{
	tegra_pinmux_config_table(whistler_null_ulpi_pinmux,
				  ARRAY_SIZE(whistler_null_ulpi_pinmux));
	platform_device_register(&icera_baseband_device);
	return 0;
}
