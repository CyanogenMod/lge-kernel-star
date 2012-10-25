/*
 * arch/arm/mach-tegra/board-p1852.c
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
 *
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
#include <linux/platform_data/tegra_nor.h>
#include <linux/spi/spi.h>
#include <linux/mtd/partitions.h>
#include <mach/clk.h>
#include <mach/iomap.h>
#include <mach/irqs.h>
#include <mach/pinmux.h>
#include <mach/iomap.h>
#include <mach/io.h>
#include <mach/pci.h>
#include <mach/audio.h>
#include <asm/mach/flash.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <mach/usb_phy.h>
#include <sound/wm8903.h>
#include <mach/tsensor.h>
#include "board.h"
#include "clock.h"
#include "board-p1852.h"
#include "devices.h"
#include "gpio-names.h"
#include "fuse.h"

static struct tegra_utmip_config utmi_phy_config[] = {
	[0] = {
			.hssync_start_delay = 0,
			.idle_wait_delay = 17,
			.elastic_limit = 16,
			.term_range_adj = 6,
			.xcvr_setup = 15,
			.xcvr_setup_offset = 0,
			.xcvr_use_fuses = 1,
			.xcvr_lsfslew = 2,
			.xcvr_lsrslew = 2,
	},
	[1] = {
			.hssync_start_delay = 0,
			.idle_wait_delay = 17,
			.elastic_limit = 16,
			.term_range_adj = 6,
			.xcvr_setup = 15,
			.xcvr_setup_offset = 0,
			.xcvr_use_fuses = 1,
			.xcvr_lsfslew = 2,
			.xcvr_lsrslew = 2,
	},
	[2] = {
			.hssync_start_delay = 0,
			.idle_wait_delay = 17,
			.elastic_limit = 16,
			.term_range_adj = 6,
			.xcvr_setup = 8,
			.xcvr_setup_offset = 0,
			.xcvr_use_fuses = 1,
			.xcvr_lsfslew = 2,
			.xcvr_lsrslew = 2,
	},
};

static __initdata struct tegra_clk_init_table p1852_clk_init_table[] = {
	/* name		parent		rate		enabled */
	{ "pll_m",		NULL,		0,		true},
	{ "hda",		"pll_p",	108000000,	false},
	{ "hda2codec_2x",	"pll_p",	48000000,	false},
	{ "pwm",		"clk_32k",	32768,		false},
	{ "blink",		"clk_32k",	32768,		true},
	{ "pll_a",		NULL,		552960000,	false},
	{ "pll_a_out0",		NULL,		12288000,	false},
	{ "d_audio",		"pll_a_out0",	12288000,	false},
	{ "nor",		"pll_p",	86500000,	true},
	{ "uarta",		"pll_p",	480000000,	true},
	{ "uarte",		"pll_p",	480000000,	true},
	{ "sdmmc2",		"pll_p",	52000000,	true},
	{ "sbc1",		"pll_m",	100000000,	true},
	{ "sbc2",		"pll_m",	100000000,	true},
	{ "sbc3",		"pll_m",	100000000,	true},
	{ "sbc4",		"pll_m",	100000000,	true},
	{ "sbc5",		"pll_m",	100000000,	true},
	{ "sbc6",		"pll_m",	100000000,	true},
	{ "cpu_g",		"cclk_g",	900000000,	true},
	{ "i2s0",		"clk_m",	12288000,	false},
	{ "i2s1",		"clk_m",	12288000,	false},
	{ "i2s2",		"clk_m",	12288000,	false},
	{ "i2s3",		"clk_m",	12288000,	false},
	{ "i2s4",		"clk_m",	12288000,	false},
	{ "vi",			"pll_p",	200000000,	true},
	{ "vi_sensor",		"pll_p",	150000000,	true},
	{ "vde",		"pll_c",	484000000,	true},
	{ "host1x",		"pll_c",	300000000,	true},
	{ "mpe",		"pll_c",	484000000,	true},
	{ "se",			"pll_m",	650000000,	true},
	{ "i2c1",		"pll_p",	3200000,	true},
	{ "i2c2",		"pll_p",	3200000,	true},
	{ "i2c3",		"pll_p",	3200000,	true},
	{ "i2c4",		"pll_p",	3200000,	true},
	{ "i2c5",		"pll_p",	3200000,	true},
	{ NULL,			NULL,		0,		0},
};

static struct tegra_i2c_platform_data p1852_i2c1_platform_data = {
	.adapter_nr	= 0,
	.bus_count	= 1,
	.bus_clk_rate	= { 100000, 0 },
};

static struct tegra_i2c_platform_data p1852_i2c2_platform_data = {
	.adapter_nr	= 1,
	.bus_count	= 1,
	.bus_clk_rate	= { 100000, 0 },
	.is_clkon_always = true,
};

static struct tegra_i2c_platform_data p1852_i2c4_platform_data = {
	.adapter_nr	= 3,
	.bus_count	= 1,
	.bus_clk_rate	= { 100000, 0 },
};

static struct tegra_i2c_platform_data p1852_i2c5_platform_data = {
	.adapter_nr	= 4,
	.bus_count	= 1,
	.bus_clk_rate	= { 100000, 0 },
};

static struct tegra_pci_platform_data p1852_pci_platform_data = {
	.port_status[0] = 0,
	.port_status[1] = 1,
	.port_status[2] = 1,
	.use_dock_detect = 0,
	.gpio           = 0,
};

static void p1852_pcie_init(void)
{
	tegra_pci_device.dev.platform_data = &p1852_pci_platform_data;
	platform_device_register(&tegra_pci_device);
}

static void p1852_i2c_init(void)
{
	tegra_i2c_device1.dev.platform_data = &p1852_i2c1_platform_data;
	tegra_i2c_device2.dev.platform_data = &p1852_i2c2_platform_data;
	tegra_i2c_device4.dev.platform_data = &p1852_i2c4_platform_data;
	tegra_i2c_device5.dev.platform_data = &p1852_i2c5_platform_data;

	platform_device_register(&tegra_i2c_device5);
	platform_device_register(&tegra_i2c_device4);
	platform_device_register(&tegra_i2c_device2);
	platform_device_register(&tegra_i2c_device1);
}

static struct platform_device *p1852_uart_devices[] __initdata = {
	&tegra_uarta_device,
	&tegra_uartb_device,
	&tegra_uarte_device,
};
static struct clk *debug_uart_clk;

static void __init uart_debug_init(void)
{
	/* Use skuinfo to decide debug uart */
	/* UARTB is the debug port. */
	pr_info("Selecting UARTB as the debug console\n");
	p1852_uart_devices[1] = &debug_uartb_device;
	debug_uart_clk = clk_get_sys("serial8250.0", "uartb");
}

static void __init p1852_uart_init(void)
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

	platform_add_devices(p1852_uart_devices,
				ARRAY_SIZE(p1852_uart_devices));
}

#if defined(CONFIG_SPI_TEGRA) && defined(CONFIG_SPI_SPIDEV)
static struct spi_board_info tegra_spi_devices[] __initdata = {
	{
		.modalias = "spidev",
		.bus_num = 0,
		.chip_select = 0,
		.mode = SPI_MODE_0,
		.max_speed_hz = 18000000,
		.platform_data = NULL,
		.irq = 0,
	},
	{
		.modalias = "spidev",
		.bus_num = 1,
		.chip_select = 1,
		.mode = SPI_MODE_0,
		.max_speed_hz = 18000000,
		.platform_data = NULL,
		.irq = 0,
	},
	{
		.modalias = "spidev",
		.bus_num = 3,
		.chip_select = 1,
		.mode = SPI_MODE_0,
		.max_speed_hz = 18000000,
		.platform_data = NULL,
		.irq = 0,
	},
};

static void __init p852_register_spidev(void)
{
	spi_register_board_info(tegra_spi_devices,
			ARRAY_SIZE(tegra_spi_devices));
}
#else
#define p852_register_spidev() do {} while (0)
#endif


static void p1852_spi_init(void)
{
	tegra_spi_device2.name = "spi_slave_tegra";
	platform_device_register(&tegra_spi_device1);
	platform_device_register(&tegra_spi_device2);
	platform_device_register(&tegra_spi_device4);
	p852_register_spidev();
}

static struct platform_device *p1852_devices[] __initdata = {
#if defined(CONFIG_TEGRA_IOVMM_SMMU)
	&tegra_smmu_device,
#endif
#if defined(CONFIG_TEGRA_AVP)
	&tegra_avp_device,
#endif
};

static struct usb_phy_plat_data tegra_usb_phy_pdata[] = {
	[0] = {
			.instance = 0,
			.vbus_gpio = -1,
			.vbus_reg_supply = NULL,
	},
	[1] = {
			.instance = 1,
			.vbus_gpio = -1,
	},
	[2] = {
			.instance = 2,
			.vbus_gpio = -1,
			.vbus_reg_supply = NULL,
	},
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

static void p1852_usb_init(void)
{
	/* Need to parse sku info to decide host/device mode */
	tegra_usb_phy_init(tegra_usb_phy_pdata,
				ARRAY_SIZE(tegra_usb_phy_pdata));

	tegra_ehci1_device.dev.platform_data = &tegra_ehci_pdata[0];
	platform_device_register(&tegra_ehci1_device);

	tegra_ehci2_device.dev.platform_data = &tegra_ehci_pdata[1];
	platform_device_register(&tegra_ehci2_device);

	tegra_ehci3_device.dev.platform_data = &tegra_ehci_pdata[2];
	platform_device_register(&tegra_ehci3_device);

}

static struct tegra_nor_platform_data p1852_nor_data = {
	.flash = {
		.map_name = "cfi_probe",
		.width = 2,
	},
	.chip_parms = {
		/* FIXME: Need to use characterized value */
		.timing_default = {
			.timing0 = 0xA0400273,
			.timing1 = 0x00030402,
		},
		.timing_read = {
			.timing0 = 0xA0400273,
			.timing1 = 0x00030402,
		},
	},
};

static void p1852_nor_init(void)
{
	tegra_nor_device.resource[2].end = TEGRA_NOR_FLASH_BASE + SZ_64M - 1;
	tegra_nor_device.dev.platform_data = &p1852_nor_data;
	platform_device_register(&tegra_nor_device);
}

static void __init tegra_p1852_init(void)
{
	tegra_clk_init_from_table(p1852_clk_init_table);
	p1852_pinmux_init();
	p1852_i2c_init();
	p1852_gpio_init();
	p1852_uart_init();
	p1852_usb_init();
	p1852_sdhci_init();
	p1852_spi_init();
	platform_add_devices(p1852_devices, ARRAY_SIZE(p1852_devices));
	p1852_panel_init();
	p1852_nor_init();
	p1852_pcie_init();
}

static void __init tegra_p1852_reserve(void)
{
#if defined(CONFIG_NVMAP_CONVERT_CARVEOUT_TO_IOVMM)
	tegra_reserve(0, SZ_8M, 0);
#else
	tegra_reserve(SZ_128M, SZ_8M, 0);
#endif
}

MACHINE_START(P1852, "p1852")
	.boot_params    = 0x80000100,
	.init_irq       = tegra_init_irq,
	.init_early     = tegra_init_early,
	.init_machine   = tegra_p1852_init,
	.map_io         = tegra_map_common_io,
	.reserve        = tegra_p1852_reserve,
	.timer          = &tegra_timer,
MACHINE_END
