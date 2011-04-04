/*
 * arch/arm/mach-tegra/board-common.c
 *
 * Standard SoC platform devices for Tegra SoCs
 *
 * Copyright (c) 2009-2010, NVIDIA Corporation.
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/serial_8250.h>
#include <linux/io.h>
#include <linux/ctype.h>
#include <linux/dma-mapping.h>
#include <linux/fsl_devices.h>

#include <mach/iomap.h>
#include <mach/irqs.h>

#include <mach/kbc.h>
#include <mach/nand.h>
#include <mach/sdhci.h>

#if defined(CONFIG_MTD_NAND_TEGRA) || defined(CONFIG_EMBEDDED_MMC_START_OFFSET)
#define MAX_MTD_PARTNR 8
static struct mtd_partition tegra_mtd_partitions[MAX_MTD_PARTNR];
//20100419 bergkamp.cho@lge.com for headset detetion [LGE_START]
#if defined(CONFIG_MACH_STAR)
#include <linux/switch.h>	//20100419 bergkamp.cho@lge.com for Headset Detection [LGE]
#endif /* CONFIG_MACH_STAR */
//20100419 bergkamp.cho@lge.com for headset detection [LGE_END]

struct tegra_nand_platform tegra_nand_plat = {
	.parts = tegra_mtd_partitions,
	.nr_parts = 0,
};

static int __init tegrapart_setup(char *options)
{
	char *str = options;

	if (!options || !*options)
		return 0;

	while (tegra_nand_plat.nr_parts < ARRAY_SIZE(tegra_mtd_partitions)) {
		struct mtd_partition *part;
		unsigned long long start, length, sector_sz;
		char *tmp = str;

		part = &tegra_nand_plat.parts[tegra_nand_plat.nr_parts];

		while (*tmp && !isspace(*tmp) && *tmp!=':')
			tmp++;

		if (tmp==str || *tmp!=':') {
			pr_err("%s: improperly formatted string %s\n",
			       __func__, options);
			break;
		}

		part->name = str;
		*tmp = 0;

		str = tmp+1;
		start = simple_strtoull(str, &tmp, 16);
		if (*tmp!=':')
			break;
		str = tmp+1;
		length = simple_strtoull(str, &tmp, 16);
		if (*tmp!=':')
			break;
		str = tmp+1;
		sector_sz = simple_strtoull(str, &tmp, 16);

		start *= sector_sz;
		length *= sector_sz;
		part->offset = start;
		part->size = length;
		tegra_nand_plat.nr_parts++;
		str = tmp+1;

		if (*tmp!=',')
			break;
	}

	/* clean up if the last partition was parsed incorrectly */
	if (tegra_nand_plat.nr_parts < ARRAY_SIZE(tegra_mtd_partitions) &&
	    tegra_mtd_partitions[tegra_nand_plat.nr_parts].name) {
		kfree(tegra_mtd_partitions[tegra_nand_plat.nr_parts].name);
		tegra_mtd_partitions[tegra_nand_plat.nr_parts].name = NULL;
	}

	return 0;
}
__setup("tegrapart=", tegrapart_setup);
#endif
#ifdef CONFIG_MTD_NAND_TEGRA

static struct platform_device tegra_nand_device = {
	.name = "tegra_nand",
	.id = -1,
	.dev = {
		.platform_data = &tegra_nand_plat,
	},
};
#endif

#ifdef CONFIG_TEGRA_NVRM
static struct platform_device tegra_nvrm_device = {
	.name = "nvrm",
	.id = -1,
};
#endif

#ifdef CONFIG_KEYBOARD_TEGRA
extern struct tegra_kbc_plat tegra_kbc_platform;

static struct resource tegra_kbc_resources[] = {
	[0] = {
		.start = TEGRA_KBC_BASE,
		.end   = TEGRA_KBC_BASE + TEGRA_KBC_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = INT_KBC,
		.end   = INT_KBC,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device tegra_kbc_device = {
	.name = "tegra-kbc",
	.id = -1,
	.dev = {
		.platform_data = &tegra_kbc_platform,
	},
	.resource = tegra_kbc_resources,
	.num_resources = ARRAY_SIZE(tegra_kbc_resources),
	
};
#endif

#ifdef CONFIG_KEYBOARD_GPIO
extern struct gpio_keys_platform_data tegra_button_data;
static struct platform_device tegra_button_device = {
	.name   = "tegra-kbc",
	.id     = 3,
	.dev    = {
		.platform_data  = &tegra_button_data,
	}
};
#endif


#ifdef CONFIG_TEGRA_IOVMM_GART
static struct resource tegra_gart_resources[] = {
	[0] = {
		.start = TEGRA_MC_BASE,
		.end = TEGRA_MC_BASE + TEGRA_MC_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = TEGRA_GART_BASE,
		.end = TEGRA_GART_BASE + TEGRA_GART_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
};
static struct platform_device tegra_gart_device = {
	.name = "tegra_gart",
	.id = -1,
	.resource = tegra_gart_resources,
	.num_resources = ARRAY_SIZE(tegra_gart_resources),
};
#endif

#ifdef CONFIG_FB_TEGRA_GRHOST
static struct resource tegra_grhost_resources[] = {
	[0] = {
		.name = "host1x",
		.start = TEGRA_HOST1X_BASE,
		.end = TEGRA_HOST1X_BASE + TEGRA_HOST1X_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.name = "display",
		.start = TEGRA_DISPLAY_BASE,
		.end = TEGRA_DISPLAY_BASE + TEGRA_DISPLAY_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[2] = {
		.name = "display2",
		.start = TEGRA_DISPLAY2_BASE,
		.end = TEGRA_DISPLAY2_BASE + TEGRA_DISPLAY2_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[3] = {
		.name = "vi",
		.start = TEGRA_VI_BASE,
		.end = TEGRA_VI_BASE + TEGRA_VI_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[4] = {
		.name = "isp",
		.start = TEGRA_ISP_BASE,
		.end = TEGRA_ISP_BASE + TEGRA_ISP_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[5] = {
		.name = "mpe",
		.start = TEGRA_MPE_BASE,
		.end = TEGRA_MPE_BASE + TEGRA_MPE_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[6] = {
		.name = "syncpt_thresh",
		.start = INT_SYNCPT_THRESH_BASE,
		.end = INT_SYNCPT_THRESH_BASE + INT_SYNCPT_THRESH_NR - 1,
		.flags = IORESOURCE_IRQ,
	},
	[7] = {
		.name = "host1x_mpcore_general",
		.start = INT_HOST1X_MPCORE_GENERAL,
		.end = INT_HOST1X_MPCORE_GENERAL,
		.flags = IORESOURCE_IRQ,
	},
};
static struct platform_device tegra_grhost_device = {
	.name = "tegra_grhost",
	.id = -1,
	.resource = tegra_grhost_resources,
	.num_resources = ARRAY_SIZE(tegra_grhost_resources),
};
#endif

#ifdef CONFIG_USB_GADGET_TEGRA
static u64 tegra_udc_dma_mask = DMA_BIT_MASK(32);
static struct fsl_usb2_platform_data tegra_udc_platform = {
	.phy_mode = FSL_USB2_PHY_UTMI,
	.operating_mode = FSL_USB2_DR_DEVICE,
};
static struct resource tegra_udc_resources[] = {
	[0] = {
		.start = TEGRA_USB_BASE,
		.end = TEGRA_USB_BASE + TEGRA_USB_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = INT_USB,
		.end = INT_USB,
		.flags = IORESOURCE_IRQ,
	},
};
static struct platform_device tegra_udc_device = {
	.name = "tegra-udc",
	.id = 0,
	.dev = {
		.platform_data = &tegra_udc_platform,
		.coherent_dma_mask = DMA_BIT_MASK(32),
		.dma_mask = &tegra_udc_dma_mask,
	},
	.resource = tegra_udc_resources,
	.num_resources = ARRAY_SIZE(tegra_udc_resources),
};
#endif

static struct platform_device *tegra_devices[] __initdata = {
#ifdef CONFIG_MTD_NAND_TEGRA
	&tegra_nand_device,
#endif
#ifdef CONFIG_TEGRA_NVRM
	&tegra_nvrm_device,
#endif
#ifdef CONFIG_KEYBOARD_TEGRA
	&tegra_kbc_device,
#endif
#ifdef CONFIG_KEYBOARD_GPIO
	&tegra_button_device,
#endif
#ifdef CONFIG_USB_GADGET_TEGRA
	&tegra_udc_device,
#endif
#ifdef CONFIG_TEGRA_IOVMM_GART
	&tegra_gart_device,
#endif
#ifdef CONFIG_FB_TEGRA_GRHOST
	&tegra_grhost_device,
#endif
};

void __init tegra_register_socdev(void)
{
	platform_add_devices(tegra_devices, ARRAY_SIZE(tegra_devices));
}
