/*
 * arch/arm/mach-tegra/board-aruba-panel.c
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

#include "devices.h"
#include "gpio-names.h"

#define PMC_SCRATCH20	0xa0

#define aruba_lvds_shutdown	TEGRA_GPIO_PB2

static int aruba_panel_enable(void)
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

	gpio_set_value(aruba_lvds_shutdown, 1);
	return 0;
}

static int aruba_panel_disable(void)
{
	gpio_set_value(aruba_lvds_shutdown, 0);
	return 0;
}

static struct resource aruba_disp1_resources[] = {
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
		.start	= 0,	/* Filled in by fbmem_set() */
		.end	= 0,	/* Filled in by fbmem_set() */
		.flags	= IORESOURCE_MEM,
	},
};

static struct tegra_dc_mode aruba_panel_modes[] = {
	{
		.pclk = 18000000,
		.h_ref_to_sync = 8,
		.v_ref_to_sync = 2,
		.h_sync_width = 4,
		.v_sync_width = 1,
		.h_back_porch = 20,
		.v_back_porch = 7,
		.h_active = 480,
		.v_active = 640,
		.h_front_porch = 8,
		.v_front_porch = 8,
	},
};

static struct tegra_fb_data aruba_fb_data = {
	.win		= 0,
	.xres		= 480,
	.yres		= 640,
	.bits_per_pixel	= 16,
};

static struct tegra_dc_out aruba_disp1_out = {
	.type		= TEGRA_DC_OUT_RGB,

	.align		= TEGRA_DC_ALIGN_MSB,
	.order		= TEGRA_DC_ORDER_RED_BLUE,

	.modes	 	= aruba_panel_modes,
	.n_modes 	= ARRAY_SIZE(aruba_panel_modes),

	.enable		= aruba_panel_enable,
	.disable	= aruba_panel_disable,
};

static struct tegra_dc_platform_data aruba_disp1_pdata = {
	.flags		= TEGRA_DC_FLAG_ENABLED,
	.default_out	= &aruba_disp1_out,
	.fb		= &aruba_fb_data,
};

static struct nvhost_device aruba_disp1_device = {
	.name		= "tegradc",
	.id		= 0,
	.resource	= aruba_disp1_resources,
	.num_resources	= ARRAY_SIZE(aruba_disp1_resources),
	.dev = {
		.platform_data = &aruba_disp1_pdata,
	},
};

static struct nvmap_platform_carveout aruba_carveouts[] = {
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
		.base		= 0,	/* Filled in by carveout_set() */
		.size		= 0,	/* Filled in by carveout_set() */
		.buddy_size	= SZ_32K,
	},
};

static struct nvmap_platform_data aruba_nvmap_data = {
	.carveouts	= aruba_carveouts,
	.nr_carveouts	= ARRAY_SIZE(aruba_carveouts),
};

static struct platform_device aruba_nvmap_device = {
	.name	= "tegra-nvmap",
	.id	= -1,
	.dev	= {
		.platform_data = &aruba_nvmap_data,
	},
};

static struct platform_device *aruba_gfx_devices[] __initdata = {
	&aruba_nvmap_device,
	&tegra_grhost_device,
};


static inline u32 pmc_readl(unsigned long offset)
{
	return readl(IO_TO_VIRT(TEGRA_PMC_BASE + offset));
}

static void fbmem_set(struct resource *res, int num_res,
				u32 start, resource_size_t size)
{
	int i;
	for (i = 0; i < num_res ; i++) {
		if (!strcmp(res[i].name, "fbmem")) {
			res[i].start = start;
			res[i].end = start + size - 1;
			return;
		}
	}
	/* Didn't find a framebuffer memory resource */
	BUG();
}

static void carveout_set(struct nvmap_platform_carveout *res, int num_res,
				u32 base, resource_size_t size)
{
	int i;
	for (i = 0; i < num_res ; i++) {
		if (!strcmp(res[i].name, "generic-0")) {
			res[i].base = base;
			res[i].size = size;
			return;
		}
	}
	/* Didn't find a carveout memory resource */
	BUG();
}

int __init aruba_panel_init(void)
{
	int err;
	u32 odm_data = pmc_readl(PMC_SCRATCH20);

	/* !!!FIXME!!!	HAVE TO USE HARD-CODED FRAME BUFFER AND CARVEOUT
			ADDRESSES FOR NOW -- BUG 769986 */
	switch (odm_data & 0x70000000) {
	case 0x10000000:
		/* 256MB LPDDR2 */
		fbmem_set(aruba_disp1_resources,
				ARRAY_SIZE(aruba_disp1_resources),
				0x8E010000,
				0x0012C3C0);
		carveout_set(aruba_carveouts,
				ARRAY_SIZE(aruba_carveouts),
				0x8EC00000, /* 256MB mem - 32MB carveout + 0xC00000 ?*/
				SZ_32M - 0xC00000);
		break;
	case 0x40000000:
		/* 1GB DDR3 -- NOTE: The bootloader cannot map more than 512MB
		   of physical memory. Therefore, the frame buffer and carveout
		   must be completely below the 512MB boundary. */
		fbmem_set(aruba_disp1_resources,
				ARRAY_SIZE(aruba_disp1_resources),
				0x9E010000,
				0x0012C3C0);
		carveout_set(aruba_carveouts,
				ARRAY_SIZE(aruba_carveouts),
				0x9EC00000, /* 512MB mem - 32MB carveout + 0xC00000 ?*/
				SZ_32M - 0xC00000);
		break;
	default:
		BUG();
	}

	err = platform_add_devices(aruba_gfx_devices,
				   ARRAY_SIZE(aruba_gfx_devices));

	if (!err)
		err = nvhost_device_register(&aruba_disp1_device);

	return err;
}
