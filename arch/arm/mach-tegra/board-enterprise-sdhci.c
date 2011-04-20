/*
 * arch/arm/mach-tegra/board-enterprise-sdhci.c
 *
 * Copyright (C) 2010 Google, Inc.
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

#include <linux/resource.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/mmc/host.h>

#include <asm/mach-types.h>
#include <mach/irqs.h>
#include <mach/iomap.h>
#include <mach/sdhci.h>

#include "gpio-names.h"
#include "board.h"


#define ENTERPRISE_SD_CD TEGRA_GPIO_PI5
#define ENTERPRISE_SD_WP TEGRA_GPIO_PT3

static struct resource sdhci_resource0[] = {
	[0] = {
		.start  = INT_SDMMC1,
		.end    = INT_SDMMC1,
		.flags  = IORESOURCE_IRQ,
	},
	[1] = {
		.start	= TEGRA_SDMMC1_BASE,
		.end	= TEGRA_SDMMC1_BASE + TEGRA_SDMMC1_SIZE-1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct resource sdhci_resource3[] = {
	[0] = {
		.start  = INT_SDMMC4,
		.end    = INT_SDMMC4,
		.flags  = IORESOURCE_IRQ,
	},
	[1] = {
		.start	= TEGRA_SDMMC4_BASE,
		.end	= TEGRA_SDMMC4_BASE + TEGRA_SDMMC4_SIZE-1,
		.flags	= IORESOURCE_MEM,
	},
};


static struct tegra_sdhci_platform_data tegra_sdhci_platform_data0 = {
	.clk_id = NULL,
	.force_hs = 1,
	.cd_gpio = -1,
	.wp_gpio = -1,
	.power_gpio = -1,
	.tap_delay = 6,
	.is_voltage_switch_supported = true,
	.vsd_name = "vddio_sdmmc1",
	.vsd_slot_name = "vddio_sd_slot",
	.max_clk = 208000000,
	.is_8bit_supported = false,
};

static struct tegra_sdhci_platform_data tegra_sdhci_platform_data3 = {
	.clk_id = NULL,
	.force_hs = 0,
	.cd_gpio = -1,
	.wp_gpio = -1,
	.power_gpio = -1,
	.tap_delay = 6,
	.is_voltage_switch_supported = false,
	.vsd_name = NULL,
	.vsd_slot_name = NULL,
	.max_clk = 48000000,
	.is_8bit_supported = true,
};

static struct platform_device tegra_sdhci_device0 = {
	.name		= "sdhci-tegra",
	.id		= 0,
	.resource	= sdhci_resource0,
	.num_resources	= ARRAY_SIZE(sdhci_resource0),
	.dev = {
		.platform_data = &tegra_sdhci_platform_data0,
	},
};

static struct platform_device tegra_sdhci_device3 = {
	.name		= "sdhci-tegra",
	.id		= 3,
	.resource	= sdhci_resource3,
	.num_resources	= ARRAY_SIZE(sdhci_resource3),
	.dev = {
		.platform_data = &tegra_sdhci_platform_data3,
	},
};

static int enterprise_sd_cd_gpio_init(void)
{
	unsigned int rc = 0;

	rc = gpio_request(ENTERPRISE_SD_CD, "card_detect");
	if (rc) {
		pr_err("Card detect gpio request failed:%d\n", rc);
		return rc;
	}

	tegra_gpio_enable(ENTERPRISE_SD_CD);

	rc = gpio_direction_input(ENTERPRISE_SD_CD);
	if (rc) {
		pr_err("Unable to configure direction for card detect gpio:%d\n", rc);
		return rc;
	}

	return 0;
}

static int enterprise_sd_wp_gpio_init(void)
{
	unsigned int rc = 0;

	rc = gpio_request(ENTERPRISE_SD_WP, "write_protect");
	if (rc) {
		pr_err("Write protect gpio request failed:%d\n", rc);
		return rc;
	}

	tegra_gpio_enable(ENTERPRISE_SD_WP);

	rc = gpio_direction_input(ENTERPRISE_SD_WP);
	if (rc) {
		pr_err("Unable to configure direction for write protect gpio:%d\n", rc);
		return rc;
	}

	return 0;
}

int __init enterprise_sdhci_init(void)
{
	unsigned int rc = 0;
	platform_device_register(&tegra_sdhci_device3);

	/* Fix ME: The gpios have to enabled for hot plug support */
	rc = enterprise_sd_cd_gpio_init();
	if (!rc) {
		tegra_sdhci_platform_data0.cd_gpio = ENTERPRISE_SD_CD;
		tegra_sdhci_platform_data0.cd_gpio_polarity = 0;
	}
	rc = enterprise_sd_wp_gpio_init();
	if (!rc) {
		tegra_sdhci_platform_data0.wp_gpio = ENTERPRISE_SD_WP;
		tegra_sdhci_platform_data0.wp_gpio_polarity = 1;
	}

	platform_device_register(&tegra_sdhci_device0);

	return 0;
}
