/*
 * arch/arm/mach-tegra/board-cardhu.c
 *
 * Copyright (c) 2011, NVIDIA Corporation.
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
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/serial_8250.h>
#include <linux/i2c.h>
#include <linux/i2c/panjit_ts.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/i2c-tegra.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/platform_data/tegra_usb.h>
#include <linux/usb/android_composite.h>
#include <linux/spi/spi.h>
#include <mach/clk.h>
#include <mach/iomap.h>
#include <mach/irqs.h>
#include <mach/pinmux.h>
#include <mach/iomap.h>
#include <mach/io.h>
#include <mach/i2s.h>
#include <mach/audio.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <mach/usb_phy.h>

#include "board.h"
#include "clock.h"
#include "board-cardhu.h"
#include "devices.h"
#include "gpio-names.h"
#include "fuse.h"


static struct usb_mass_storage_platform_data tegra_usb_fsg_platform = {
	.vendor = "NVIDIA",
	.product = "Tegra 3",
	.nluns = 1,
};

static struct platform_device tegra_usb_fsg_device = {
	.name = "usb_mass_storage",
	.id = -1,
	.dev = {
		.platform_data = &tegra_usb_fsg_platform,
	},
};

/* !!!TODO: Change for cardhu (Taken from Ventana) */
static struct tegra_utmip_config utmi_phy_config[] = {
	[0] = {
			.hssync_start_delay = 0,
			.idle_wait_delay = 17,
			.elastic_limit = 16,
			.term_range_adj = 6,
			.xcvr_setup = 15,
			.xcvr_lsfslew = 2,
			.xcvr_lsrslew = 2,
	},
	[1] = {
			.hssync_start_delay = 0,
			.idle_wait_delay = 17,
			.elastic_limit = 16,
			.term_range_adj = 6,
			.xcvr_setup = 15,
			.xcvr_lsfslew = 2,
			.xcvr_lsrslew = 2,
	},
	[2] = {
			.hssync_start_delay = 0,
			.idle_wait_delay = 17,
			.elastic_limit = 16,
			.term_range_adj = 6,
			.xcvr_setup = 8,
			.xcvr_lsfslew = 2,
			.xcvr_lsrslew = 2,
	},
};

#ifdef CONFIG_BCM4329_RFKILL
static struct resource cardhu_bcm4329_rfkill_resources[] = {
	{
		.name   = "bcm4329_nshutdown_gpio",
		.start  = TEGRA_GPIO_PU0,
		.end    = TEGRA_GPIO_PU0,
		.flags  = IORESOURCE_IO,
	},
};

static struct platform_device cardhu_bcm4329_rfkill_device = {
	.name = "bcm4329_rfkill",
	.id             = -1,
	.num_resources  = ARRAY_SIZE(cardhu_bcm4329_rfkill_resources),
	.resource       = cardhu_bcm4329_rfkill_resources,
};

static noinline void __init cardhu_bt_rfkill(void)
{
	platform_device_register(&cardhu_bcm4329_rfkill_device);

	return;
}
#else
static inline void cardhu_bt_rfkill(void) { }
#endif

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
	tegra_gpio_enable(TEGRA_GPIO_PU6);
	tegra_gpio_enable(TEGRA_GPIO_PU1);

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

static __initdata struct tegra_clk_init_table cardhu_clk_init_table[] = {
	/* name		parent		rate		enabled */
	{ "pll_m",	NULL,		0,		true},
	{ "hda",	"pll_p",	108000000,	false},
	{ "hda2codec_2x","pll_p",	48000000,	false},
	{ "pwm",	"clk_32k",	32768,		false},
	{ "blink",	"clk_32k",	32768,		true},
	{ "pll_a",	NULL,		552960000,	true},
	{ "pll_a_out0",	NULL,		11289600,	true},
	{ "i2s1",	"pll_a_out0",	11289600,	true},
	{ "i2s2",	"pll_a_out0",	11289600,	true},
	{ "audio",	"pll_a_out0",	11289600,	true},
	{ "audio_2x",	"audio",	22579200,	true},
	{ NULL,		NULL,		0,		0},
};

#define USB_MANUFACTURER_NAME	"NVIDIA"
#define USB_PRODUCT_NAME		"Cardhu"
#define USB_PRODUCT_ID_MTP_ADB	0x7100
#define USB_PRODUCT_ID_MTP		0x7102
#define USB_PRODUCT_ID_RNDIS	0x7103
#define USB_VENDOR_ID			0x0955

static char *usb_functions_mtp_ums[] = { "mtp", "usb_mass_storage" };
static char *usb_functions_mtp_adb_ums[] = { "mtp", "adb", "usb_mass_storage" };
#ifdef CONFIG_USB_ANDROID_RNDIS
static char *usb_functions_rndis[] = { "rndis" };
static char *usb_functions_rndis_adb[] = { "rndis", "adb" };
#endif
static char *usb_functions_all[] = {
#ifdef CONFIG_USB_ANDROID_RNDIS
	"rndis",
#endif
	"mtp",
	"adb",
	"usb_mass_storage"
};

static struct android_usb_product usb_products[] = {
	{
		.product_id     = USB_PRODUCT_ID_MTP,
		.num_functions  = ARRAY_SIZE(usb_functions_mtp_ums),
		.functions      = usb_functions_mtp_ums,
	},
	{
		.product_id     = USB_PRODUCT_ID_MTP_ADB,
		.num_functions  = ARRAY_SIZE(usb_functions_mtp_adb_ums),
		.functions      = usb_functions_mtp_adb_ums,
	},
#ifdef CONFIG_USB_ANDROID_RNDIS
	{
		.product_id     = USB_PRODUCT_ID_RNDIS,
		.num_functions  = ARRAY_SIZE(usb_functions_rndis),
		.functions      = usb_functions_rndis,
	},
	{
		.product_id     = USB_PRODUCT_ID_RNDIS,
		.num_functions  = ARRAY_SIZE(usb_functions_rndis_adb),
		.functions      = usb_functions_rndis_adb,
	},
#endif
};

/* standard android USB platform data */
static struct android_usb_platform_data andusb_plat = {
	.vendor_id              = USB_VENDOR_ID,
	.product_id             = USB_PRODUCT_ID_MTP_ADB,
	.manufacturer_name      = USB_MANUFACTURER_NAME,
	.product_name           = USB_PRODUCT_NAME,
	.serial_number          = NULL,
	.num_products = ARRAY_SIZE(usb_products),
	.products = usb_products,
	.num_functions = ARRAY_SIZE(usb_functions_all),
	.functions = usb_functions_all,
};

static struct platform_device androidusb_device = {
	.name   = "android_usb",
	.id     = -1,
	.dev    = {
		.platform_data  = &andusb_plat,
	},
};

#ifdef CONFIG_USB_ANDROID_RNDIS
static struct usb_ether_platform_data rndis_pdata = {
	.ethaddr = {0, 0, 0, 0, 0, 0},
	.vendorID = USB_VENDOR_ID,
	.vendorDescr = USB_MANUFACTURER_NAME,
};

static struct platform_device rndis_device = {
	.name   = "rndis",
	.id     = -1,
	.dev    = {
		.platform_data  = &rndis_pdata,
	},
};
#endif

static struct tegra_i2c_platform_data cardhu_i2c1_platform_data = {
	.adapter_nr	= 0,
	.bus_count	= 1,
	.bus_clk_rate	= { 100000, 0 },
};

static struct tegra_i2c_platform_data cardhu_i2c2_platform_data = {
	.adapter_nr	= 1,
	.bus_count	= 1,
	.bus_clk_rate	= { 100000, 0 },
	.is_clkon_always = true,
};

static struct tegra_i2c_platform_data cardhu_i2c3_platform_data = {
	.adapter_nr	= 2,
	.bus_count	= 1,
	.bus_clk_rate	= { 100000, 0 },
};

static struct tegra_i2c_platform_data cardhu_i2c4_platform_data = {
	.adapter_nr	= 3,
	.bus_count	= 1,
	.bus_clk_rate	= { 100000, 0 },
};

static struct tegra_i2c_platform_data cardhu_i2c5_platform_data = {
	.adapter_nr	= 4,
	.bus_count	= 1,
	.bus_clk_rate	= { 100000, 0 },
};


#if 0
struct tegra_wired_jack_conf audio_wr_jack_conf = {
	.hp_det_n = TEGRA_GPIO_PW2,
	.en_mic_ext = TEGRA_GPIO_PX1,
	.en_mic_int = TEGRA_GPIO_PX0,
};
#endif

static void cardhu_i2c_init(void)
{
	tegra_i2c_device1.dev.platform_data = &cardhu_i2c1_platform_data;
	tegra_i2c_device2.dev.platform_data = &cardhu_i2c2_platform_data;
	tegra_i2c_device3.dev.platform_data = &cardhu_i2c3_platform_data;
	tegra_i2c_device4.dev.platform_data = &cardhu_i2c4_platform_data;
	tegra_i2c_device5.dev.platform_data = &cardhu_i2c5_platform_data;

	platform_device_register(&tegra_i2c_device5);
	platform_device_register(&tegra_i2c_device4);
	platform_device_register(&tegra_i2c_device3);
	platform_device_register(&tegra_i2c_device2);
	platform_device_register(&tegra_i2c_device1);
}

static struct platform_device *cardhu_uart_devices[] __initdata = {
	&tegra_uarta_device,
	&tegra_uartb_device,
	&tegra_uartc_device,
	&tegra_uartd_device,
	&tegra_uarte_device,
};
static struct clk *debug_uart_clk;

static void __init uart_debug_init(void)
{
	struct board_info board_info;

	tegra_get_board_info(&board_info);
	if (board_info.sku & SKU_SLT_ULPI_SUPPORT) {
		if ((board_info.board_id == BOARD_E1186) ||
			(board_info.board_id == BOARD_E1187) ||
			(board_info.board_id == BOARD_PM269)) {
				/* UARTB is the debug port. */
				pr_info("Selecting UARTB as the debug console\n");
				cardhu_uart_devices[1] = &debug_uartb_device;
				debug_uart_clk =
					clk_get_sys("serial8250.0", "uartb");
				return;
		}
		pr_err("%s(): Unhandled SKU information for Board 0x%04x\n",
				__func__, board_info.board_id);
	}
	/* UARTA is the debug port. */
	pr_info("Selecting UARTA as the debug console\n");
	cardhu_uart_devices[0] = &debug_uarta_device;
	debug_uart_clk = clk_get_sys("serial8250.0", "uarta");
}

static void __init cardhu_uart_init(void)
{
	/* Register low speed only if it is selected */
	if (!is_tegra_debug_uartport_hs()) {
		uart_debug_init();
		/* Clock enable for the debug channel */
		if (!IS_ERR_OR_NULL(debug_uart_clk)) {
			pr_info("The debug console clock name is %s\n",
						debug_uart_clk->name);
			clk_enable(debug_uart_clk);
			clk_set_rate(debug_uart_clk, 408000000);
		} else {
			pr_err("Not getting the clock %s for debug console\n",
					debug_uart_clk->name);
		}
	}

	platform_add_devices(cardhu_uart_devices,
				ARRAY_SIZE(cardhu_uart_devices));
}

#if defined(CONFIG_RTC_DRV_TEGRA)
static struct resource tegra_rtc_resources[] = {
	[0] = {
		.start = TEGRA_RTC_BASE,
		.end = TEGRA_RTC_BASE + TEGRA_RTC_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = INT_RTC,
		.end = INT_RTC,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device tegra_rtc_device = {
	.name = "tegra_rtc",
	.id   = -1,
	.resource = tegra_rtc_resources,
	.num_resources = ARRAY_SIZE(tegra_rtc_resources),
};
#endif

static struct platform_device tegra_camera = {
	.name = "tegra_camera",
	.id = -1,
};

static struct platform_device *cardhu_devices[] __initdata = {
	&tegra_pmu_device,
#if defined(CONFIG_RTC_DRV_TEGRA)
	&tegra_rtc_device,
#endif
	&tegra_udc_device,
	&androidusb_device,
	&tegra_usb_fsg_device,
#if defined(CONFIG_SND_HDA_TEGRA)
	&tegra_hda_device,
#endif
#if defined(CONFIG_TEGRA_IOVMM_SMMU)
	&tegra_smmu_device,
#endif
	&tegra_wdt_device,
	&tegra_avp_device,
	&tegra_camera,
	&tegra_spi_device4,
#if defined(CONFIG_CRYPTO_DEV_TEGRA_SE)
	&tegra_se_device,
#endif
};

static int __init cardhu_touch_init(void)
{
	return 0;
}

static struct usb_phy_plat_data tegra_usb_phy_pdata[] = {
	[0] = {
			.instance = 0,
			.vbus_gpio = -1,
			.vbus_reg_supply = "vdd_vbus_micro_usb",
	},
	[1] = {
			.instance = 1,
			.vbus_gpio = -1,
	},
	[2] = {
			.instance = 2,
			.vbus_gpio = -1,
			.vbus_reg_supply = "vdd_vbus_typea_usb",
	},
};

static struct tegra_ulpi_config uhsic_phy_config = {
	.enable_gpio = EN_HSIC_GPIO,
	.reset_gpio = -1,
};

static struct tegra_ehci_platform_data tegra_ehci_uhsic_pdata = {
	.phy_type = TEGRA_USB_PHY_TYPE_HSIC,
	.phy_config = &uhsic_phy_config,
	.operating_mode = TEGRA_USB_HOST,
	.power_down_on_bus_suspend = 1,
};

static struct tegra_ehci_platform_data tegra_ehci_pdata[] = {
	[0] = {
			.phy_config = &utmi_phy_config[0],
			.operating_mode = TEGRA_USB_HOST,
			.power_down_on_bus_suspend = 1,
	},
	[1] = {
			.phy_config = &utmi_phy_config[1],
			.operating_mode = TEGRA_USB_HOST,
			.power_down_on_bus_suspend = 1,
	},
	[2] = {
			.phy_config = &utmi_phy_config[2],
			.operating_mode = TEGRA_USB_HOST,
			.power_down_on_bus_suspend = 1,
	},
};

static struct platform_device *tegra_usb_otg_host_register(void)
{
	struct platform_device *pdev;
	void *platform_data;
	int val;

	pdev = platform_device_alloc(tegra_ehci1_device.name,
		tegra_ehci1_device.id);
	if (!pdev)
		return NULL;

	val = platform_device_add_resources(pdev, tegra_ehci1_device.resource,
		tegra_ehci1_device.num_resources);
	if (val)
		goto error;

	pdev->dev.dma_mask =  tegra_ehci1_device.dev.dma_mask;
	pdev->dev.coherent_dma_mask = tegra_ehci1_device.dev.coherent_dma_mask;

	platform_data = kmalloc(sizeof(struct tegra_ehci_platform_data),
		GFP_KERNEL);
	if (!platform_data)
		goto error;

	memcpy(platform_data, &tegra_ehci_pdata[0],
				sizeof(struct tegra_ehci_platform_data));
	pdev->dev.platform_data = platform_data;

	val = platform_device_add(pdev);
	if (val)
		goto error_add;

	return pdev;

error_add:
	kfree(platform_data);
error:
	pr_err("%s: failed to add the host contoller device\n", __func__);
	platform_device_put(pdev);
	return NULL;
}

static void tegra_usb_otg_host_unregister(struct platform_device *pdev)
{
	platform_device_unregister(pdev);
}

static struct tegra_otg_platform_data tegra_otg_pdata = {
	.host_register = &tegra_usb_otg_host_register,
	.host_unregister = &tegra_usb_otg_host_unregister,
};

#define SERIAL_NUMBER_LENGTH 20
static char usb_serial_num[SERIAL_NUMBER_LENGTH];

static void cardhu_usb_init(void)
{
#if 0
	struct board_info bi;
	char *src = NULL;
	int i;

	tegra_get_board_info(&bi);

	tegra_usb_phy_init(tegra_usb_phy_pdata, ARRAY_SIZE(tegra_usb_phy_pdata));

	tegra_otg_device.dev.platform_data = &tegra_otg_pdata;
	platform_device_register(&tegra_otg_device);

	if (bi.board_id == BOARD_PM267) {
		uhsic_phy_config.reset_gpio =
			PM267_SMSC4640_HSIC_HUB_RESET_GPIO;
		tegra_ehci2_device.dev.platform_data = &tegra_ehci_uhsic_pdata;
	} else if ((bi.board_id == BOARD_PM269) ||
		(bi.board_id == BOARD_E1186)) {
		tegra_ehci2_device.dev.platform_data = &tegra_ehci_uhsic_pdata;
	} else {
		tegra_ehci2_device.dev.platform_data = &tegra_ehci_pdata[1];
	}
	platform_device_register(&tegra_ehci2_device);

	tegra_ehci3_device.dev.platform_data = &tegra_ehci_pdata[2];
	platform_device_register(&tegra_ehci3_device);

#ifdef CONFIG_USB_ANDROID_RNDIS
	src = usb_serial_num;

	/* create a fake MAC address from our serial number.
	 * first byte is 0x02 to signify locally administered.
	 */
	rndis_pdata.ethaddr[0] = 0x02;
	for (i = 0; *src; i++) {
		/* XOR the USB serial across the remaining bytes */
		rndis_pdata.ethaddr[i % (ETH_ALEN - 1) + 1] ^= *src++;
	}
	platform_device_register(&rndis_device);
#endif
#endif
}

static void cardhu_gps_init(void)
{
	tegra_gpio_enable(TEGRA_GPIO_PU2);
	tegra_gpio_enable(TEGRA_GPIO_PU3);
}

static void cardhu_modem_init(void)
{
	tegra_gpio_enable(TEGRA_GPIO_PH5);
}

#ifdef CONFIG_SATA_AHCI_TEGRA
static void cardhu_sata_init(void)
{
	platform_device_register(&tegra_sata_device);
}
#else
static void cardhu_sata_init(void) { }
#endif

static void __init tegra_cardhu_init(void)
{
	tegra_clk_init_from_table(cardhu_clk_init_table);
	cardhu_pinmux_init();
	cardhu_i2c_init();
	cardhu_usb_init();
#ifdef CONFIG_TEGRA_EDP_LIMITS
	cardhu_edp_init();
#endif
	cardhu_uart_init();
	snprintf(usb_serial_num, sizeof(usb_serial_num), "%llx", tegra_chip_uid());
	andusb_plat.serial_number = kstrdup(usb_serial_num, GFP_KERNEL);
	platform_add_devices(cardhu_devices, ARRAY_SIZE(cardhu_devices));
	cardhu_sdhci_init();
	cardhu_regulator_init();
	cardhu_gpio_switch_regulator_init();
	cardhu_suspend_init();
	cardhu_power_off_init();
	cardhu_touch_init();
	cardhu_gps_init();
	cardhu_modem_init();
	//cardhu_kbc_init();
	//cardhu_scroll_init();
	//cardhu_keys_init();
	cardhu_panel_init();
	cardhu_pmon_init();
	cardhu_sensors_init();
	cardhu_bt_rfkill();
	tegra_setup_bluesleep();
	cardhu_sata_init();
	//audio_wired_jack_init();
	cardhu_pins_state_init();
	cardhu_emc_init();
}

static void __init tegra_cardhu_reserve(void)
{
#if defined(CONFIG_NVMAP_CONVERT_CARVEOUT_TO_IOVMM)
	tegra_reserve(0, SZ_8M, SZ_8M);
#else
	tegra_reserve(SZ_128M, SZ_8M, SZ_8M);
#endif
}

MACHINE_START(CARDHU, "cardhu")
	.boot_params    = 0x80000100,
	.map_io         = tegra_map_common_io,
	.reserve        = tegra_cardhu_reserve,
	.init_early	= tegra_init_early,
	.init_irq       = tegra_init_irq,
	.timer          = &tegra_timer,
	.init_machine   = tegra_cardhu_init,
MACHINE_END
