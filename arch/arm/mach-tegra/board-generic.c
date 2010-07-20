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
#include <linux/i2c.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>
#include <asm/setup.h>

#include <mach/gpio.h>
#include "gpio-names.h"

#include <mach/iomap.h>
#include <mach/irqs.h>
#include <mach/nvrm_linux.h>
#include <nvrm_module.h>

#include "board.h"

#ifdef CONFIG_USB_ANDROID

static char *tegra_android_functions_ums[] = {
#ifdef CONFIG_USB_ANDROID_MASS_STORAGE
	"usb_mass_storage",
#endif
};

static char *tegra_android_functions_ums_adb[] = {
#ifdef CONFIG_USB_ANDROID_MASS_STORAGE
	"usb_mass_storage",
#endif
#ifdef CONFIG_USB_ANDROID_ADB
	"adb",
#endif
};

static char *tegra_android_functions_rndis[] = {
#ifdef CONFIG_USB_ANDROID_RNDIS
	"rndis",
#endif
};

static char *tegra_android_functions_rndis_adb[] = {
#ifdef CONFIG_USB_ANDROID_RNDIS
	"rndis",
#endif
#ifdef CONFIG_USB_ANDROID_ADB
	"adb",
#endif
};

static char *tegra_android_functions_all[] = {
#ifdef CONFIG_USB_ANDROID_RNDIS
	"rndis",
#endif
#ifdef CONFIG_USB_ANDROID_MASS_STORAGE
	"usb_mass_storage",
#endif
#ifdef CONFIG_USB_ANDROID_ADB
	"adb",
#endif
};

static struct android_usb_product tegra_android_products[] = {
	[0] = {
		.product_id = 0x7100,
		.num_functions = ARRAY_SIZE(tegra_android_functions_ums),
		.functions = tegra_android_functions_ums,
	},
	[1] = {
		.product_id = 0x7100,
		.num_functions = ARRAY_SIZE(tegra_android_functions_ums_adb),
		.functions = tegra_android_functions_ums_adb,
	},
	[2] = {
		.product_id = 0x7102,
		.num_functions = ARRAY_SIZE(tegra_android_functions_rndis),
		.functions = tegra_android_functions_rndis,
	},
	[3] = {
		.product_id = 0x7103,
		.num_functions = ARRAY_SIZE(tegra_android_functions_rndis_adb),
		.functions = tegra_android_functions_rndis_adb,
	},
};

static char *harmony_dev = "NVIDIA Harmony";
static char *ventana_dev = "NVIDIA Ventana";
static char *generic_dev = "NVIDIA Tegra 2";

static struct android_usb_platform_data tegra_android_platform = {
	.vendor_id = 0x955,
	.product_id = 0x7100,
	.manufacturer_name = "NVIDIA",
	.num_products = ARRAY_SIZE(tegra_android_products),
	.products = tegra_android_products,
	.num_functions = ARRAY_SIZE(tegra_android_functions_all),
	.functions = tegra_android_functions_all,
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
#ifdef CONFIG_USB_ANDROID_RNDIS
static struct usb_ether_platform_data tegra_usb_rndis_platform = {
	.ethaddr = {
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00
	},
	.vendorID = 0x7100,
	.vendorDescr = "Tegra 2 RNDIS",
};
static struct platform_device tegra_usb_rndis_device = {
	.name = "rndis",
	.id = -1,
	.dev = {
		.platform_data = &tegra_usb_rndis_platform,
	},
};
#endif
#endif

static struct platform_device *platform_devices[] = {
#ifdef CONFIG_USB_ANDROID
#ifdef CONFIG_USB_ANDROID_RNDIS
	&tegra_usb_rndis_device,
#endif
#ifdef CONFIG_USB_ANDROID_MASS_STORAGE
	&tegra_usb_fsg_device,
#endif
	&tegra_android_device,
#endif
};

static struct i2c_board_info bus0_i2c_devices[] = {
#ifdef CONFIG_SENSORS_ISL29018
	{
		I2C_BOARD_INFO("isl29018", 0x44),
		.irq = (INT_GPIO_BASE + TEGRA_GPIO_PZ2),
	},
#endif
};

static struct i2c_board_info bus3_i2c_devices[] = {
#ifdef CONFIG_SENSORS_AK8975
	{
		I2C_BOARD_INFO("mm_ak8975", 0x0C),
		.irq = (INT_GPIO_BASE + TEGRA_GPIO_PN5),
	},
#endif
};

void __init i2c_device_setup(void)
{
	if (ARRAY_SIZE(bus0_i2c_devices))
		i2c_register_board_info(0, bus0_i2c_devices,
					ARRAY_SIZE(bus0_i2c_devices));

	if (ARRAY_SIZE(bus3_i2c_devices))
		i2c_register_board_info(3, bus3_i2c_devices,
					ARRAY_SIZE(bus3_i2c_devices));
}

extern void __init tegra_setup_nvodm(bool standard_i2c, bool standard_spi);
extern void __init tegra_register_socdev(void);

static void __init do_system_init(bool standard_i2c, bool standard_spi)
{
	unsigned int chip_id[2];
	char serial[17];

	tegra_common_init();
	tegra_setup_nvodm(true, true);
	tegra_register_socdev();

	NvRmQueryChipUniqueId(s_hRmGlobal, sizeof(chip_id), (void*)chip_id);
	snprintf(serial, sizeof(serial), "%08x%08x", chip_id[1], chip_id[0]);
#ifdef CONFIG_USB_ANDROID
	tegra_android_platform.serial_number = kstrdup(serial, GFP_KERNEL);
#endif
	platform_add_devices(platform_devices, ARRAY_SIZE(platform_devices));
}

static void __init tegra_harmony_init(void)
{
#ifdef CONFIG_USB_ANDROID
	tegra_android_platform.product_name = harmony_dev;
#endif
	do_system_init(true, true);
}

static void __init tegra_ventana_init(void)
{
#ifdef CONFIG_USB_ANDROID
	tegra_android_platform.product_name = ventana_dev;
#endif
	do_system_init(false, true);
	i2c_device_setup();
}

static void __init tegra_generic_init(void)
{
#ifdef CONFIG_USB_ANDROID
	tegra_android_platform.product_name = generic_dev;
#endif
	do_system_init(true, true);
}

MACHINE_START(VENTANA, "NVIDIA Ventana Development System")
	.boot_params  = 0x00000100,
	.phys_io        = IO_APB_PHYS,
	.io_pg_offst    = ((IO_APB_VIRT) >> 18) & 0xfffc,
	.init_irq       = tegra_init_irq,
	.init_machine   = tegra_ventana_init,
	.map_io         = tegra_map_common_io,
	.timer          = &tegra_timer,
MACHINE_END

MACHINE_START(HARMONY, "NVIDIA Harmony Development System")
	.boot_params  = 0x00000100,
	.phys_io        = IO_APB_PHYS,
	.io_pg_offst    = ((IO_APB_VIRT) >> 18) & 0xfffc,
	.init_irq       = tegra_init_irq,
	.init_machine   = tegra_harmony_init,
	.map_io         = tegra_map_common_io,
	.timer          = &tegra_timer,
MACHINE_END


MACHINE_START(TEGRA_GENERIC, "Tegra 2 Development System")
	.boot_params  = 0x00000100,
	.phys_io        = IO_APB_PHYS,
	.io_pg_offst    = ((IO_APB_VIRT) >> 18) & 0xfffc,
	.init_irq       = tegra_init_irq,
	.init_machine   = tegra_generic_init,
	.map_io         = tegra_map_common_io,
	.timer          = &tegra_timer,
MACHINE_END
