/*
 * arch/arm/mach-tegra/board-p1852-panel.c
 *
 * Copyright (c) 2012, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/resource.h>
#include <asm/mach-types.h>
#include <linux/platform_device.h>
#include <linux/nvhost.h>
#include <mach/nvmap.h>
#include <mach/irqs.h>
#include <mach/iomap.h>
#include <mach/dc.h>
#include <mach/fb.h>

#include "board.h"
#include "devices.h"

static int p1852_panel_enable(void)
{
	return 0;
}

static int p1852_panel_disable(void)
{
	return 0;
}

static struct tegra_dc_mode p1852_panel_modes[] = {
	{
		/* 800x480@60 */
		.pclk = 32460000,
		.h_ref_to_sync = 1,
		.v_ref_to_sync = 1,
		.h_sync_width = 64,
		.v_sync_width = 3,
		.h_back_porch = 128,
		.v_back_porch = 22,
		.h_front_porch = 64,
		.v_front_porch = 20,
		.h_active = 800,
		.v_active = 480,
	},
};

static struct tegra_fb_data p1852_fb_data = {
	.win		= 0,
	.xres		= 800,
	.yres		= 480,
	.bits_per_pixel	= 32,
};

static struct tegra_dc_out p1852_disp1_out = {
	.align		= TEGRA_DC_ALIGN_MSB,
	.order		= TEGRA_DC_ORDER_RED_BLUE,
	.type		= TEGRA_DC_OUT_RGB,
	.modes		= p1852_panel_modes,
	.n_modes	= ARRAY_SIZE(p1852_panel_modes),
	.enable		= p1852_panel_enable,
	.disable	= p1852_panel_disable,
};

static struct tegra_dc_platform_data p1852_disp1_pdata = {
	.flags		= TEGRA_DC_FLAG_ENABLED,
	.default_out	= &p1852_disp1_out,
	.emc_clk_rate	= 300000000,
	.fb		= &p1852_fb_data,
};

static struct nvmap_platform_carveout p1852_carveouts[] = {
	[0] = {
		.name		= "iram",
		.usage_mask	= NVMAP_HEAP_CARVEOUT_IRAM,
		.base		= TEGRA_IRAM_BASE + TEGRA_RESET_HANDLER_SIZE,
		.size		= TEGRA_IRAM_SIZE - TEGRA_RESET_HANDLER_SIZE,
		.buddy_size	= 0, /* no buddy allocation for IRAM */
	},
	[1] = {
		.name		= "generic-0",
		.usage_mask	= NVMAP_HEAP_CARVEOUT_GENERIC,
		.base		= 0,	/* Filled in by p1852_panel_init() */
		.size		= 0,	/* Filled in by p1852_panel_init() */
		.buddy_size	= SZ_32K,
	},
};

static struct nvmap_platform_data p1852_nvmap_data = {
	.carveouts	= p1852_carveouts,
	.nr_carveouts	= ARRAY_SIZE(p1852_carveouts),
};

static struct platform_device *p1852_gfx_devices[] __initdata = {
	&tegra_nvmap_device,
	&tegra_grhost_device
};

int __init p1852_panel_init(void)
{
	int err;
	struct resource *res;

	p1852_carveouts[1].base = tegra_carveout_start;
	p1852_carveouts[1].size = tegra_carveout_size;
	tegra_nvmap_device.dev.platform_data = &p1852_nvmap_data;
	tegra_disp1_device.dev.platform_data = &p1852_disp1_pdata;

	res = nvhost_get_resource_byname(&tegra_disp1_device,
					 IORESOURCE_MEM, "fbmem");
	if (!res) {
		pr_err("No memory resources\n");
		return -ENODEV;
	}
	res->start = tegra_fb_start;
	res->end = tegra_fb_start + tegra_fb_size - 1;

	err = platform_add_devices(p1852_gfx_devices,
				ARRAY_SIZE(p1852_gfx_devices));
	if (!err)
		err = nvhost_device_register(&tegra_disp1_device);

	return err;
}
