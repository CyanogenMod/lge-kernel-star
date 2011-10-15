/*
 * arch/arm/mach-tegra/board-harmony-panel.c
 *
 * Copyright (c) 2010-2011, NVIDIA Corporation.
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
#include <asm/mach-types.h>
#include <linux/nvhost.h>
#include <mach/irqs.h>
#include <mach/iomap.h>
#include <mach/nvmap.h>
#include <mach/tegra_fb.h>

#include "devices.h"
#include "board.h"

/* Framebuffer */
static struct resource fb_resource[] = {
	[0] = {
		.start  = INT_DISPLAY_GENERAL,
		.end    = INT_DISPLAY_GENERAL,
		.flags  = IORESOURCE_IRQ,
	},
	[1] = {
		.start	= TEGRA_DISPLAY_BASE,
		.end	= TEGRA_DISPLAY_BASE + TEGRA_DISPLAY_SIZE-1,
		.flags	= IORESOURCE_MEM,
	},
	[2] = {
		.start	= 0x1c012000,
		.end	= 0x1c012000 + 0x500000 - 1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct tegra_fb_lcd_data tegra_fb_lcd_platform_data = {
	.lcd_xres	= 1024,
	.lcd_yres	= 600,
	.fb_xres	= 1024,
	.fb_yres	= 600,
	.bits_per_pixel	= 32,
};

static struct platform_device tegra_fb_device = {
	.name 		= "tegrafb",
	.id		= 0,
	.resource	= fb_resource,
	.num_resources 	= ARRAY_SIZE(fb_resource),
	.dev = {
		.platform_data = &tegra_fb_lcd_platform_data,
	},
};

static struct nvmap_platform_carveout harmony_carveouts[] = {
	[0] = NVMAP_HEAP_CARVEOUT_IRAM_INIT,
	[1] = {
		.name		= "generic-0",
		.usage_mask	= NVMAP_HEAP_CARVEOUT_GENERIC,
		.buddy_size	= SZ_32K,
	},
};

static struct nvmap_platform_data harmony_nvmap_data = {
	.carveouts	= harmony_carveouts,
	.nr_carveouts	= ARRAY_SIZE(harmony_carveouts),
};

static struct platform_device harmony_nvmap_device = {
	.name	= "tegra-nvmap",
	.id	= -1,
	.dev	= {
		.platform_data = &harmony_nvmap_data,
	},
};

static struct platform_device *harmony_gfx_devices[] __initdata = {
	&harmony_nvmap_device,
	&tegra_grhost_device,
};

int __init harmony_panel_init(void) {
	int err;

	harmony_carveouts[1].base = tegra_carveout_start;
	harmony_carveouts[1].size = tegra_carveout_size;

	err = platform_add_devices(harmony_gfx_devices,
				   ARRAY_SIZE(harmony_gfx_devices));
	if (err)
		return err;

	return platform_device_register(&tegra_fb_device);
}

