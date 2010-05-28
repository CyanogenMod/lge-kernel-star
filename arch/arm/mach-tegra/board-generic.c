/*
 * arch/arm/mach-tegra/board-generic.c
 *
 * Copyright (C) 2010 Google, Inc.
 * Copyright (C) 2010 NVIDIA Corporation
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
#include <linux/platform_device.h>
#include <linux/serial_8250.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/pda_power.h>
#include <linux/io.h>
#include <linux/usb/android_composite.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>
#include <asm/setup.h>

#include <mach/iomap.h>
#include <mach/irqs.h>

#include "board.h"

#ifdef CONFIG_USB_ANDROID
static char *tegra_android_functions[] = {
#ifdef CONFIG_USB_ANDROID_ADB
	"adb",
#endif
#ifdef CONFIG_USB_ANDROID_MASS_STORAGE
	"usb_mass_storage",
#endif
};

static struct android_usb_product tegra_android_products[] = {
	[0] = {
		.product_id = 0x7100,
		.num_functions = ARRAY_SIZE(tegra_android_functions),
		.functions = tegra_android_functions,
	},
};

static char *harmony_dev = "Harmony ADB";

static struct android_usb_platform_data tegra_android_platform = {
	.vendor_id = 0x955,
	.product_id = 0x7100,
	.manufacturer_name = "NVIDIA",
	.num_products = ARRAY_SIZE(tegra_android_products),
	.products = tegra_android_products,
	.num_functions = ARRAY_SIZE(tegra_android_functions),
	.functions = tegra_android_functions,
};
static struct platform_device tegra_android_device = {
	.name = "android_usb",
	.id = -1,
	.dev = {
		.platform_data = &tegra_android_platform,
	},
};
#ifdef CONFIG_USB_ANDROID_MASS_STORAGE
static struct usb_mass_storage_platform_data tegra_usb_fsg_platform = {
	.vendor = "NVIDIA",
	.product = "Tegra 2",
	.nluns = 1,
	.bulk_size = 16384,
};
static struct platform_device tegra_usb_fsg_device = {
	.name = "usb_mass_storage",
	.id = -1,
	.dev = {
		.platform_data = &tegra_usb_fsg_platform,
	},
};
#endif
#endif

static struct platform_device *platform_devices[] = {
#ifdef CONFIG_USB_ANDROID
#ifdef CONFIG_USB_ANDROID_MASS_STORAGE
	&tegra_usb_fsg_device,
#endif
	&tegra_android_device,
#endif
};

extern void __init tegra_setup_nvodm(bool standard_i2c, bool standard_spi);
extern void __init tegra_register_socdev(void);

static void __init tegra_generic_init(void)
{
	tegra_common_init();
	tegra_setup_nvodm(true, true);
	tegra_register_socdev();
	tegra_android_platform.product_name = harmony_dev;
	platform_add_devices(platform_devices, ARRAY_SIZE(platform_devices));
}

MACHINE_START(TEGRA_GENERIC, "Tegra 2 Development System")
	.boot_params  = 0x00000100,
	.phys_io        = IO_APB_PHYS,
	.io_pg_offst    = ((IO_APB_VIRT) >> 18) & 0xfffc,
	.init_irq       = tegra_init_irq,
	.init_machine   = tegra_generic_init,
	.map_io         = tegra_map_common_io,
	.timer          = &tegra_timer,
MACHINE_END
