/*
 * arch/arm/mach-tegra/board-harmony-sdhci.c
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
#include "board-cardhu.h"

#define CARDHU_SD_CD TEGRA_GPIO_PI5
#define CARDHU_SD_WP TEGRA_GPIO_PT3
#define PM269_SD_WP TEGRA_GPIO_PZ4

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

static struct resource sdhci_resource2[] = {
	[0] = {
		.start  = INT_SDMMC3,
		.end    = INT_SDMMC3,
		.flags  = IORESOURCE_IRQ,
	},
	[1] = {
		.start	= TEGRA_SDMMC3_BASE,
		.end	= TEGRA_SDMMC3_BASE + TEGRA_SDMMC3_SIZE-1,
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
	.cd_gpio = -1,
	.wp_gpio = -1,
	.power_gpio = -1,
/*	.tap_delay = 6,
	.is_voltage_switch_supported = false,
	.vdd_rail_name = NULL,
	.slot_rail_name = NULL,
	.vdd_max_uv = -1,
	.vdd_min_uv = -1,
	.max_clk = 0,
	.is_8bit_supported = false, */
};

static struct tegra_sdhci_platform_data tegra_sdhci_platform_data2 = {
	.cd_gpio = -1,
	.wp_gpio = -1,
	.power_gpio = -1,
/*	.tap_delay = 6,
	.is_voltage_switch_supported = true,
	.vdd_rail_name = "vddio_sdmmc1",
	.slot_rail_name = "vddio_sd_slot",
	.vdd_max_uv = 3320000,
	.vdd_min_uv = 3280000,
	.max_clk = 208000000,
	.is_8bit_supported = false, */
};

static struct tegra_sdhci_platform_data tegra_sdhci_platform_data3 = {
	.cd_gpio = -1,
	.wp_gpio = -1,
	.power_gpio = -1,
/*	.tap_delay = 6,
	.is_voltage_switch_supported = false,
	.vdd_rail_name = NULL,
	.slot_rail_name = NULL,
	.vdd_max_uv = -1,
	.vdd_min_uv = -1,
	.max_clk = 48000000,
	.is_8bit_supported = true, */
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

static struct platform_device tegra_sdhci_device2 = {
	.name		= "sdhci-tegra",
	.id		= 2,
	.resource	= sdhci_resource2,
	.num_resources	= ARRAY_SIZE(sdhci_resource2),
	.dev = {
		.platform_data = &tegra_sdhci_platform_data2,
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

static int cardhu_sd_cd_gpio_init(void)
{
	unsigned int rc = 0;

	rc = gpio_request(CARDHU_SD_CD, "card_detect");
	if (rc) {
		pr_err("Card detect gpio request failed:%d\n", rc);
		return rc;
	}

	tegra_gpio_enable(CARDHU_SD_CD);

	rc = gpio_direction_input(CARDHU_SD_CD);
	if (rc) {
		pr_err("Unable to configure direction for card detect gpio:%d\n", rc);
		return rc;
	}

	return 0;
}

static int cardhu_sd_wp_gpio_init(void)
{
	unsigned int rc = 0;

	rc = gpio_request(CARDHU_SD_WP, "write_protect");
	if (rc) {
		pr_err("Write protect gpio request failed:%d\n", rc);
		return rc;
	}

	tegra_gpio_enable(CARDHU_SD_WP);

	rc = gpio_direction_input(CARDHU_SD_WP);
	if (rc) {
		pr_err("Unable to configure direction for write protect gpio:%d\n", rc);
		return rc;
	}

	return 0;
}

static int pm269_sd_wp_gpio_init(void)
{
	unsigned int rc = 0;

	rc = gpio_request(PM269_SD_WP, "write_protect");
	if (rc) {
		pr_err("Write protect gpio request failed:%d\n", rc);
		return rc;
	}

	tegra_gpio_enable(PM269_SD_WP);

	rc = gpio_direction_input(PM269_SD_WP);
	if (rc) {
		pr_err("Unable to configure direction for write protect gpio:%d\n", rc);
		return rc;
	}

	return 0;
}

int __init cardhu_sdhci_init(void)
{
	unsigned int rc = 0;
#if 0
	struct board_info board_info;
	tegra_get_board_info(&board_info);
	if (board_info.board_id == BOARD_PM269) {
		tegra_sdhci_platform_data2.max_clk = 12000000;
		rc = pm269_sd_wp_gpio_init();
		if (!rc) {
			tegra_sdhci_platform_data0.wp_gpio = PM269_SD_WP;
			tegra_sdhci_platform_data0.wp_gpio_polarity = 1;
		}
	} else {
		tegra_sdhci_platform_data2.max_clk = 48000000;
		rc = cardhu_sd_wp_gpio_init();
		if (!rc) {
			tegra_sdhci_platform_data0.wp_gpio = CARDHU_SD_WP;
			tegra_sdhci_platform_data0.wp_gpio_polarity = 1;
		}
	}
#endif

	platform_device_register(&tegra_sdhci_device3);
	platform_device_register(&tegra_sdhci_device2);

#if 0
	/* Fix ME: The gpios have to enabled for hot plug support */
	rc = cardhu_sd_cd_gpio_init();
	if (!rc) {
		tegra_sdhci_platform_data0.cd_gpio = CARDHU_SD_CD;
		tegra_sdhci_platform_data0.cd_gpio_polarity = 0;
	}
#endif



	platform_device_register(&tegra_sdhci_device0);

	return 0;
}
