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
//20100817, jm1.lee@lge.com, for USB mode switching [START]
#if defined(CONFIG_USB_SUPPORT_LGE_ANDROID_GADGET)
#include <linux/usb/android.h>
#define SERIAL_NUMBER_STRING_LEN 16
#else
#include <linux/usb/android_composite.h>
//20100817, jm1.lee@lge.com, for USB mode switching [END]
#endif

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
#include <linux/spi/spi.h>

#include "nvodm_query.h"
#include "nvodm_query_discovery.h"
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

//20100709, jm1.lee@lge.com, for LGE Android USB Driver interface [START]
#if defined(CONFIG_MACH_STAR)
static char *tegra_android_functions_lge[] = {
#ifdef CONFIG_USB_ANDROID_ACM
	"acm",
#endif
#ifdef CONFIG_USB_ANDROID_MASS_STORAGE
	"usb_mass_storage",
#endif
#ifdef CONFIG_USB_ANDROID_ADB
	"adb",
#endif
};
#endif
//20100709, jm1.lee@lge.com, for LGE Android USB Driver interface [END]
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
//20100709, jm1.lee@lge.com, for LGE Android USB Driver interface [START]
#if defined(CONFIG_MACH_STAR)
	[0] = {
		.product_id = 0x618E,
		.num_functions = ARRAY_SIZE(tegra_android_functions_lge),
		.functions = tegra_android_functions_lge,
	},
	[1] = {
		.product_id = 0x6000,
		.num_functions = ARRAY_SIZE(tegra_android_functions_ums),
		.functions = tegra_android_functions_ums,
	},
#else
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
#endif
//20100709, jm1.lee@lge.com, for LGE Android USB Driver interface [END]
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
//20100710, jm1.lee@lge.com, for LGE Android USB Driver interface [START]
#if defined(CONFIG_MACH_STAR)
static char *generic_dev = "LGE Android Phone";
#else
static char *generic_dev = "NVIDIA Tegra 2";
#endif
//20100710, jm1.lee@lge.com, for LGE Android USB Driver interface [END]

static struct android_usb_platform_data tegra_android_platform = {
//20100709, jm1.lee@lge.com, for LGE Android USB Driver interface [START]
#if 0
	.vendor_id = 0x955,
	.product_id = 0x7100,
	.manufacturer_name = "NVIDIA",
	.num_products = ARRAY_SIZE(tegra_android_products),
	.products = tegra_android_products,
	.num_functions = ARRAY_SIZE(tegra_android_functions_all),
	.functions = tegra_android_functions_all,
#else
	.vendor_id = 0x1004,
	.product_id = 0x618E,
	.manufacturer_name = "LG Electronics",
	.num_products = ARRAY_SIZE(tegra_android_products),
	.products = tegra_android_products,
	.num_functions = ARRAY_SIZE(tegra_android_functions_lge),
	.functions = tegra_android_functions_lge,
#endif
//20100709, jm1.lee@lge.com, for LGE Android USB Driver interface [END]
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
//20100710, jm1.lee@lge.com, change mass storage device information [START]
#if defined (CONFIG_MACH_STAR)
	.vendor = "LGE",
	.product = "Android Phone",
#else
	.vendor = "NVIDIA",
	.product = "Tegra 2",
#endif
//20100710, jm1.lee@lge.com, change mass storage device information [END]
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

#ifdef CONFIG_USB_SUPPORT_LGE_ANDROID_GADGET
/* dynamic composition */
/* This depends on each board. QCT original is at device_lge.c */
/* function bit : (in include/linux/usb/android.h)
   ADB				0x0001
   MSC				0x0002
   ACM_MODEM		0x0003
   DIAG				0x0004
   ACM_NMEA			0x0005
   GENERIC_MODEM	0x0006
   GENERIC_NMEA		0x0007
   CDC_ECM			0x0008
   RMNET			0x0009
   RNDIS			0x000A
*/
struct usb_composition usb_func_composition[] = {
    {
        /* Full or Light mode : ADB, UMS, NMEA, DIAG, MODEM */
        /* Light mode : UMS, NMEA, DIAG, MODEM */
        .product_id         = 0x618E,
        .functions	    	= 0x2743,
        .adb_product_id     = 0x618E,
        .adb_functions	    = 0x12743,
    },
    {
        /* Factory mode for WCDMA or GSM : DIAG, MODEM */
        /* We are in factory mode, ignore adb function */
        .product_id         = 0x6000,
        .functions	    	= 0x43,
        .adb_product_id     = 0x6000,
        .adb_functions	    = 0x43,
    },
#ifdef CONFIG_USB_ANDROID_CDC_ECM
    {
        /* LG Rmnet Driver for matching LG Android Net driver */
        .product_id         = 0x61A2,
        .functions          = 0x27384,
        .adb_product_id     = 0x61A1,
        .adb_functions      = 0x127384,
    },
#endif
#ifdef CONFIG_USB_SUPPORT_LGE_ANDROID_AUTORUN
    {
        /* Mass Storage Only for autorun */
        .product_id         = 0x61C6,
        .functions	    	= 0x2,
        .adb_product_id     = 0x618E,
        .adb_functions	    = 0x12743,
    },
    {
        /* For AutoRun, we use UMS function as CD-ROM drive */
        .product_id         = 0x61C8,
        .functions	   		= 0xC,
        .adb_product_id     = 0x61C8,
        .adb_functions	    = 0xC,
    },
#endif
#ifdef CONFIG_USB_ANDROID_RNDIS
    {
        /* RNDIS */
        .product_id         = 0x61DA,
        .functions	    	= 0xA,
        .adb_product_id     = 0x61D9,
        .adb_functions	    = 0x1A,
    },
#endif
};

static char *harmony_dev = "NVIDIA Harmony";
static char *ventana_dev = "NVIDIA Ventana";
static char *generic_dev = "LGE Android Phone";

static struct android_usb_platform_data tegra_android_platform =
{
    .vendor_id = 0x1004,
    .version = 0x0100,
//    .product_id = 0x618E,
//    .adb_product_id = 0x618E,
    .compositions   = usb_func_composition,
    .num_compositions = ARRAY_SIZE(usb_func_composition),
    .product_name = "LG Android USB Device",
    .manufacturer_name = "LG Electronics Inc.",
    .serial_number = "0000000000000000",
    .init_product_id = 0x618E,
    .nluns = 2,
    .bulk_size = 16384,
};

static struct platform_device tegra_android_device =
{
    .name = "android_usb",
    .id = -1,
    .dev =
    {
        .platform_data = &tegra_android_platform,
    },
};

#ifdef CONFIG_USB_ANDROID_MASS_STORAGE
static struct usb_mass_storage_platform_data tegra_usb_fsg_platform = {
//20100710, jm1.lee@lge.com, change mass storage device information [START]
#if defined (CONFIG_MACH_STAR)
	.vendor = "LGE",
	.product = "Android Phone",
#else
	.vendor = "NVIDIA",
	.product = "Tegra 2",
#endif
//20100710, jm1.lee@lge.com, change mass storage device information [END]
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
#endif /* CONFIG_USB_SUPPORT_LGE_ANDROID_GADGET */

static struct platform_device *platform_devices[] = {
#if defined(CONFIG_USB_ANDROID) || defined(CONFIG_USB_SUPPORT_LGE_ANDROID_GADGET)
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
#ifdef CONFIG_ISL29018
	{
		I2C_BOARD_INFO("isl29018", 0x44),
		.irq = (INT_GPIO_BASE + TEGRA_GPIO_PZ2),
	},
#endif
};

static struct i2c_board_info bus4_i2c_devices[] = {
#ifdef CONFIG_SENSORS_AK8975
	{
		I2C_BOARD_INFO("mm_ak8975", 0x0C),
		.irq = (INT_GPIO_BASE + TEGRA_GPIO_PN5),
	},
#endif
#ifdef CONFIG_SENSORS_LM90
	{
		I2C_BOARD_INFO("nct1008", 0x4C),
	},
#endif
};

void __init i2c_device_setup(void)
{
	if (ARRAY_SIZE(bus0_i2c_devices))
		i2c_register_board_info(0, bus0_i2c_devices,
					ARRAY_SIZE(bus0_i2c_devices));

	if (ARRAY_SIZE(bus4_i2c_devices))
		i2c_register_board_info(4, bus4_i2c_devices,
					ARRAY_SIZE(bus4_i2c_devices));
}

// enable 32Khz clock used by bcm4329 wifi, bluetooth and gps
static void __init tegra_setup_32khz_clock(void)
{
	int RequestedPeriod, ReturnedPeriod;
	NvOdmServicesPwmHandle hOdmPwm = NULL;

	hOdmPwm = NvOdmPwmOpen();
	if (!hOdmPwm) {
		pr_err("%s: failed to open NvOdmPwmOpen\n", __func__);
		return;
	}
	RequestedPeriod = 0;
	NvOdmPwmConfig(hOdmPwm, NvOdmPwmOutputId_Blink,
		NvOdmPwmMode_Blink_32KHzClockOutput, 0, &RequestedPeriod, &ReturnedPeriod);
	NvOdmPwmClose(hOdmPwm);
}

extern void __init tegra_setup_nvodm(bool standard_i2c, bool standard_spi);
extern void __init tegra_register_socdev(void);

static struct spi_board_info tegra_spi_ipc_devices[] __initdata = {
	{
		.modalias = "spi_ipc",
		.bus_num = 0,
		.chip_select = 0,
		.mode = SPI_MODE_1,
		.max_speed_hz = 12000000,
		.platform_data = NULL,
		.irq = 0,
	},
};

static void __init register_spi_ipc_devices(void)
{
	unsigned long instance = 0xFFFF;
	unsigned long cs = 0xFFFF;
	const NvOdmPeripheralConnectivity *pConnectivity = NULL;
	int i;

	pConnectivity =
		NvOdmPeripheralGetGuid(NV_ODM_GUID('s','p','i',' ','_','i','p','c'));
	if (!pConnectivity)
		return;

	for (i = 0; i < pConnectivity->NumAddress; i++) {
		switch (pConnectivity->AddressList[i].Interface) {
		case NvOdmIoModule_Spi:
			instance = pConnectivity->AddressList[i].Instance;
			cs = pConnectivity->AddressList[i].Address;
			break;
		default:
			break;
		}
	}

	if (instance == 0xffff || cs == 0xffff) {
		pr_err("%s: SPI IPC Protocol driver: Instance and CS are Invalid\n", __func__);
		return;
	}

	tegra_spi_ipc_devices[0].bus_num = instance;
	tegra_spi_ipc_devices[0].chip_select = cs;
	if (spi_register_board_info(tegra_spi_ipc_devices, ARRAY_SIZE(tegra_spi_ipc_devices)) != 0) {
		pr_err("%s: spi_register_board_info returned error\n", __func__);
	}
}


static void __init do_system_init(bool standard_i2c, bool standard_spi)
{
	unsigned int chip_id[2];
	char serial[17];

	tegra_common_init();
	tegra_setup_nvodm(true, true);
	tegra_register_socdev();

	NvRmQueryChipUniqueId(s_hRmGlobal, sizeof(chip_id), (void*)chip_id);
	snprintf(serial, sizeof(serial), "%08x%08x", chip_id[1], chip_id[0]);
	tegra_android_platform.serial_number = kstrdup(serial, GFP_KERNEL);
	platform_add_devices(platform_devices, ARRAY_SIZE(platform_devices));
}

#ifdef CONFIG_BT_BLUESLEEP
static noinline void __init tegra_setup_bluesleep(void)
{
	struct platform_device *pdev = NULL;
	struct resource *res;

	pdev = platform_device_alloc("bluesleep", 0);
	if (!pdev) {
		pr_err("unable to allocate platform device for bluesleep");
		return;
	}

	res = kzalloc(sizeof(struct resource) * 3, GFP_KERNEL);
	if (!res) {
		pr_err("unable to allocate resource for bluesleep\n");
		goto err_free_dev;
	}

	res[0].name   = "gpio_host_wake";
	res[0].start  = TEGRA_GPIO_PU6;
	res[0].end    = TEGRA_GPIO_PU6;
	res[0].flags  = IORESOURCE_IO;

	res[1].name   = "gpio_ext_wake";
	res[1].start  = TEGRA_GPIO_PU1;
	res[1].end    = TEGRA_GPIO_PU1;
	res[1].flags  = IORESOURCE_IO;

	res[2].name   = "host_wake";
	res[2].start  = gpio_to_irq(TEGRA_GPIO_PU6);
	res[2].end    = gpio_to_irq(TEGRA_GPIO_PU6);
	res[2].flags  = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE ;

	if (platform_device_add_resources(pdev, res, 3)) {
		pr_err("unable to add resources to bluesleep device\n");
		goto err_free_res;
	}

	if (platform_device_add(pdev)) {
		pr_err("unable to add bluesleep device\n");
		goto err_free_res;
	}
	return;

err_free_res:
	kfree(res);
err_free_dev:
	platform_device_put(pdev);
	return;
}
#else
static inline void tegra_setup_bluesleep(void) { }
#endif
#ifdef CONFIG_BT_BLUESLEEP
static noinline void __init tegra_setup_bluesleep_csr(void)
{
	struct platform_device *pdev = NULL;
	struct resource *res;

	pdev = platform_device_alloc("bluesleep", 0);
	if (!pdev) {
		pr_err("unable to allocate platform device for bluesleep");
		return;
	}

	res = kzalloc(sizeof(struct resource) * 2, GFP_KERNEL);
	if (!res) {
		pr_err("unable to allocate resource for bluesleep\n");
		goto err_free_dev;
	}

	res[0].name   = "gpio_host_wake";
	res[0].start  = TEGRA_GPIO_PU6;
	res[0].end    = TEGRA_GPIO_PU6;
	res[0].flags  = IORESOURCE_IO;

	res[1].name   = "host_wake";
	res[1].start  = gpio_to_irq(TEGRA_GPIO_PU6);
	res[1].end    = gpio_to_irq(TEGRA_GPIO_PU6);
	res[1].flags  = IORESOURCE_IRQ | IORESOURCE_IRQ_LOWEDGE;

	if (platform_device_add_resources(pdev, res, 2)) {
		pr_err("unable to add resources to bluesleep device\n");
		goto err_free_res;
	}

	if (platform_device_add(pdev)) {
		pr_err("unable to add bluesleep device\n");
		goto err_free_res;
	}
	return;

err_free_res:
	kfree(res);
err_free_dev:
	platform_device_put(pdev);
	return;
}
#else
static inline void tegra_setup_bluesleep_csr(void) { }
#endif


static void __init tegra_harmony_init(void)
{
#ifdef CONFIG_USB_ANDROID
	tegra_android_platform.product_name = harmony_dev;
#endif
	do_system_init(true, true);
	tegra_setup_bluesleep_csr();
}


static void __init tegra_ventana_init(void)
{
#ifdef CONFIG_USB_ANDROID
	tegra_android_platform.product_name = ventana_dev;
#endif
	do_system_init(false, true);
	i2c_device_setup();
	tegra_setup_32khz_clock();
	tegra_setup_bluesleep();
	ventana_setup_wifi();
}

static void __init tegra_generic_init(void)
{
#if defined(CONFIG_USB_ANDROID) || defined(CONFIG_USB_SUPPORT_LGE_ANDROID_GADGET)
	tegra_android_platform.product_name = generic_dev;
#endif
	do_system_init(true, true);
        register_spi_ipc_devices();
	tegra_setup_bluesleep_csr();
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
