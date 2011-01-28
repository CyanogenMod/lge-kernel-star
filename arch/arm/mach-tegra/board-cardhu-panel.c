/*
 * arch/arm/mach-tegra/board-cardhu-panel.c
 *
 * Copyright (c) 2010, NVIDIA Corporation.
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

#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/resource.h>
#include <asm/mach-types.h>
#include <linux/platform_device.h>
#include <linux/pwm_backlight.h>
#include <mach/nvhost.h>
#include <mach/nvmap.h>
#include <mach/irqs.h>
#include <mach/iomap.h>
#include <mach/dc.h>
#include <mach/fb.h>

#include "board.h"
#include "devices.h"
#include "gpio-names.h"

#define cardhu_lvds_shutdown	TEGRA_GPIO_PL2
#define cardhu_bl_enb		TEGRA_GPIO_PH2

static int cardhu_backlight_init(struct device *dev) {
	int ret;

	ret = gpio_request(cardhu_bl_enb, "backlight_enb");
	if (ret < 0)
		return ret;

	ret = gpio_direction_output(cardhu_bl_enb, 1);
	if (ret < 0)
		gpio_free(cardhu_bl_enb);
	else
		tegra_gpio_enable(cardhu_bl_enb);

	return ret;
};

static void cardhu_backlight_exit(struct device *dev) {
	gpio_set_value(cardhu_bl_enb, 0);
	gpio_free(cardhu_bl_enb);
	tegra_gpio_disable(cardhu_bl_enb);
}

static int cardhu_backlight_notify(struct device *unused, int brightness)
{
	gpio_set_value(cardhu_bl_enb, !!brightness);
	return brightness;
}

static struct platform_pwm_backlight_data cardhu_backlight_data = {
	.pwm_id		= 0,
	.max_brightness	= 255,
	.dft_brightness	= 224,
	.pwm_period_ns	= 5000000,
	.init		= cardhu_backlight_init,
	.exit		= cardhu_backlight_exit,
	.notify		= cardhu_backlight_notify,
};

static struct platform_device cardhu_backlight_device = {
	.name	= "pwm-backlight",
	.id	= -1,
	.dev	= {
		.platform_data = &cardhu_backlight_data,
	},
};

static int cardhu_panel_enable(void)
{
	static struct regulator *reg = NULL;

	if (reg == NULL) {
		reg = regulator_get(NULL, "avdd_lvds");
		if (WARN_ON(IS_ERR(reg)))
			pr_err("%s: couldn't get regulator avdd_lvds: %ld\n",
			       __func__, PTR_ERR(reg));
		else
			regulator_enable(reg);
	}

	gpio_set_value(cardhu_lvds_shutdown, 1);
	return 0;
}

static int cardhu_panel_disable(void)
{
	gpio_set_value(cardhu_lvds_shutdown, 0);
	return 0;
}

static struct resource cardhu_disp1_resources[] = {
	{
		.name	= "irq",
		.start	= INT_DISPLAY_GENERAL,
		.end	= INT_DISPLAY_GENERAL,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name	= "regs",
		.start	= TEGRA_DISPLAY_BASE,
		.end	= TEGRA_DISPLAY_BASE + TEGRA_DISPLAY_SIZE-1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "fbmem",
		.start	= 0,	/* Filled in by cardhu_panel_init() */
		.end	= 0,	/* Filled in by cardhu_panel_init() */
		.flags	= IORESOURCE_MEM,
	},
};

static struct tegra_dc_mode cardhu_panel_modes[] = {
	{
		/* 1366x768@62.3Hz */
		.pclk = 72000000,
		.h_ref_to_sync = 4,
		.v_ref_to_sync = 2,
		.h_sync_width = 32,
		.v_sync_width = 5,
		.h_back_porch = 20,
		.v_back_porch = 12,
		.h_active = 1366,
		.v_active = 768,
		.h_front_porch = 48,
		.v_front_porch = 3,
	},
};

static struct tegra_fb_data cardhu_fb_data = {
	.win		= 0,
	.xres		= 1366,
	.yres		= 768,
	.bits_per_pixel	= 16,
};

static struct tegra_dc_out cardhu_disp1_out = {
	.type		= TEGRA_DC_OUT_RGB,

	.align		= TEGRA_DC_ALIGN_MSB,
	.order		= TEGRA_DC_ORDER_RED_BLUE,

	.modes	 	= cardhu_panel_modes,
	.n_modes 	= ARRAY_SIZE(cardhu_panel_modes),

	.enable		= cardhu_panel_enable,
	.disable	= cardhu_panel_disable,
};

static struct tegra_dc_platform_data cardhu_disp1_pdata = {
	.flags		= TEGRA_DC_FLAG_ENABLED,
	.default_out	= &cardhu_disp1_out,
	.fb		= &cardhu_fb_data,
};

static struct nvhost_device cardhu_disp1_device = {
	.name		= "tegradc",
	.id		= 0,
	.resource	= cardhu_disp1_resources,
	.num_resources	= ARRAY_SIZE(cardhu_disp1_resources),
	.dev = {
		.platform_data = &cardhu_disp1_pdata,
	},
};

static struct nvmap_platform_carveout cardhu_carveouts[] = {
	[0] = {
		.name		= "iram",
		.usage_mask	= NVMAP_HEAP_CARVEOUT_IRAM,
		.base		= TEGRA_IRAM_BASE,
		.size		= TEGRA_IRAM_SIZE,
		.buddy_size	= 0, /* no buddy allocation for IRAM */
	},
	[1] = {
		.name		= "generic-0",
		.usage_mask	= NVMAP_HEAP_CARVEOUT_GENERIC,
		.base		= 0,	/* Filled in by cardhu_panel_init() */
		.size		= 0,	/* Filled in by cardhu_panel_init() */
		.buddy_size	= SZ_32K,
	},
};

static struct nvmap_platform_data cardhu_nvmap_data = {
	.carveouts	= cardhu_carveouts,
	.nr_carveouts	= ARRAY_SIZE(cardhu_carveouts),
};

static struct platform_device cardhu_nvmap_device = {
	.name	= "tegra-nvmap",
	.id	= -1,
	.dev	= {
		.platform_data = &cardhu_nvmap_data,
	},
};

static struct platform_device *cardhu_gfx_devices[] __initdata = {
	&cardhu_nvmap_device,
	&tegra_grhost_device,
	&tegra_pwfm0_device,
	&cardhu_backlight_device,
};


int __init cardhu_panel_init(void)
{
	int err;
	struct resource *res;

	cardhu_carveouts[1].base = tegra_carveout_start;
	cardhu_carveouts[1].size = tegra_carveout_size;

	err = platform_add_devices(cardhu_gfx_devices,
				ARRAY_SIZE(cardhu_gfx_devices));

	res = nvhost_get_resource_byname(&cardhu_disp1_device,
					 IORESOURCE_MEM, "fbmem");
	res->start = tegra_fb_start;
	res->end = tegra_fb_start + tegra_fb_size - 1;

	if (!err)
		err = nvhost_device_register(&cardhu_disp1_device);

	return err;
}
