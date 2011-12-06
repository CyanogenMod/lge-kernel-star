/*
 * arch/arm/mach-tegra/board-enterprise.c
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
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/i2c-tegra.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/platform_data/tegra_usb.h>
#include <linux/spi/spi.h>
#include <linux/tegra_uart.h>
#include <linux/fsl_devices.h>
#include <linux/i2c/atmel_mxt_ts.h>
#include <linux/memblock.h>

#include <linux/nfc/pn544.h>
#include <sound/max98088.h>

#include <mach/clk.h>
#include <mach/iomap.h>
#include <mach/irqs.h>
#include <mach/pinmux.h>
#include <mach/iomap.h>
#include <mach/io.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <mach/usb_phy.h>
#include <mach/i2s.h>
#include <mach/tegra_max98088_pdata.h>
#include <mach/thermal.h>
#include <mach/tegra-bb-power.h>
#include "board.h"
#include "clock.h"
#include "board-enterprise.h"
#include "baseband-xmm-power.h"
#include "devices.h"
#include "gpio-names.h"
#include "fuse.h"
#include "pm.h"

/* All units are in millicelsius */
static struct tegra_thermal_data thermal_data = {
	.temp_throttle = 85000,
	.temp_shutdown = 90000,
	.temp_offset = TDIODE_OFFSET, /* temps based on tdiode */
#ifdef CONFIG_TEGRA_EDP_LIMITS
	.edp_offset = TDIODE_OFFSET,  /* edp based on tdiode */
	.hysteresis_edp = 3000,
#endif
#ifdef CONFIG_TEGRA_THERMAL_SYSFS
	.tc1 = 0,
	.tc2 = 1,
	.passive_delay = 2000,
#else
	.hysteresis_throttle = 1000,
#endif
};

/* !!!TODO: Change for enterprise (Taken from Cardhu) */
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

static struct resource enterprise_bcm4329_rfkill_resources[] = {
	{
		.name   = "bcm4329_nshutdown_gpio",
		.start  = TEGRA_GPIO_PE6,
		.end    = TEGRA_GPIO_PE6,
		.flags  = IORESOURCE_IO,
	},
};

static struct platform_device enterprise_bcm4329_rfkill_device = {
	.name = "bcm4329_rfkill",
	.id		= -1,
	.num_resources  = ARRAY_SIZE(enterprise_bcm4329_rfkill_resources),
	.resource       = enterprise_bcm4329_rfkill_resources,
};

static struct resource enterprise_bluesleep_resources[] = {
	[0] = {
		.name = "gpio_host_wake",
			.start  = TEGRA_GPIO_PS2,
			.end    = TEGRA_GPIO_PS2,
			.flags  = IORESOURCE_IO,
	},
	[1] = {
		.name = "gpio_ext_wake",
			.start  = TEGRA_GPIO_PE7,
			.end    = TEGRA_GPIO_PE7,
			.flags  = IORESOURCE_IO,
	},
	[2] = {
		.name = "host_wake",
			.start  = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PS2),
			.end    = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PS2),
			.flags  = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE,
	},
};

static struct platform_device enterprise_bluesleep_device = {
	.name           = "bluesleep",
	.id             = -1,
	.num_resources  = ARRAY_SIZE(enterprise_bluesleep_resources),
	.resource       = enterprise_bluesleep_resources,
};

static void __init enterprise_setup_bluesleep(void)
{
	platform_device_register(&enterprise_bluesleep_device);
	tegra_gpio_enable(TEGRA_GPIO_PS2);
	tegra_gpio_enable(TEGRA_GPIO_PE7);
	return;
}

static __initdata struct tegra_clk_init_table enterprise_clk_init_table[] = {
	/* name		parent		rate		enabled */
	{ "pll_m",	NULL,		0,		false},
	{ "hda",	"pll_p",	108000000,	false},
	{ "hda2codec_2x","pll_p",	48000000,	false},
	{ "pwm",	"clk_32k",	32768,		false},
	{ "blink",	"clk_32k",	32768,		true},
	{ "pll_a",	NULL,		564480000,	false},
	{ "pll_a_out0",	NULL,		11289600,	false},
	{ "i2s0",	"pll_a_out0",	0,		false},
	{ "i2s1",	"pll_a_out0",	0,		false},
	{ "i2s2",	"pll_a_out0",	0,		false},
	{ "i2s3",	"pll_a_out0",	0,		false},
	{ "spdif_out",	"pll_a_out0",	0,		false},
	{ "d_audio",	"pll_a_out0",	0,		false},
	{ "dam0",	"pll_a_out0",	0,		false},
	{ "dam1",	"pll_a_out0",	0,		false},
	{ "dam2",	"pll_a_out0",	0,		false},
	{ "audio0",	"i2s0_sync",	0,		false},
	{ "audio1",	"i2s1_sync",	0,		false},
	{ "audio2",	"i2s2_sync",	0,		false},
	{ "audio3",	"i2s3_sync",	0,		false},
	{ NULL,		NULL,		0,		0},
};

static struct tegra_i2c_platform_data enterprise_i2c1_platform_data = {
	.adapter_nr	= 0,
	.bus_count	= 1,
	.bus_clk_rate	= { 100000, 0 },
	.scl_gpio		= {TEGRA_GPIO_PC4, 0},
	.sda_gpio		= {TEGRA_GPIO_PC5, 0},
	.arb_recovery = arb_lost_recovery,
};

static struct tegra_i2c_platform_data enterprise_i2c2_platform_data = {
	.adapter_nr	= 1,
	.bus_count	= 1,
	.bus_clk_rate	= { 100000, 0 },
	.is_clkon_always = true,
	.scl_gpio		= {TEGRA_GPIO_PT5, 0},
	.sda_gpio		= {TEGRA_GPIO_PT6, 0},
	.arb_recovery = arb_lost_recovery,
};

static struct tegra_i2c_platform_data enterprise_i2c3_platform_data = {
	.adapter_nr	= 2,
	.bus_count	= 1,
	.bus_clk_rate	= { 100000, 0 },
	.scl_gpio		= {TEGRA_GPIO_PBB1, 0},
	.sda_gpio		= {TEGRA_GPIO_PBB2, 0},
	.arb_recovery = arb_lost_recovery,
};

static struct tegra_i2c_platform_data enterprise_i2c4_platform_data = {
	.adapter_nr	= 3,
	.bus_count	= 1,
	.bus_clk_rate	= { 100000, 0 },
	.scl_gpio		= {TEGRA_GPIO_PV4, 0},
	.sda_gpio		= {TEGRA_GPIO_PV5, 0},
	.arb_recovery = arb_lost_recovery,
};

static struct tegra_i2c_platform_data enterprise_i2c5_platform_data = {
	.adapter_nr	= 4,
	.bus_count	= 1,
	.bus_clk_rate	= { 100000, 0 },
	.scl_gpio		= {TEGRA_GPIO_PZ6, 0},
	.sda_gpio		= {TEGRA_GPIO_PZ7, 0},
	.arb_recovery = arb_lost_recovery,
};

/* Equalizer filter coefs generated from the MAXIM MAX98088
 * evkit software tool */
static struct max98088_eq_cfg max98088_eq_cfg[] = {
	{
		.name = "FLAT",
		.rate = 44100,
		.band1 = {0x2000, 0xC002, 0x4000, 0x00E9, 0x0000},
		.band2 = {0x2000, 0xC00F, 0x4000, 0x02BC, 0x0000},
		.band3 = {0x2000, 0xC0A7, 0x4000, 0x0916, 0x0000},
		.band4 = {0x2000, 0xC5C2, 0x4000, 0x1A87, 0x0000},
		.band5 = {0x2000, 0xF6B0, 0x4000, 0x3F51, 0x0000},
	},
	{
		.name = "LOWPASS1K",
		.rate = 44100,
		.band1 = {0x205D, 0xC001, 0x3FEF, 0x002E, 0x02E0},
		.band2 = {0x5B9A, 0xC093, 0x3AB2, 0x088B, 0x1981},
		.band3 = {0x0D22, 0xC170, 0x26EA, 0x0D79, 0x32CF},
		.band4 = {0x0894, 0xC612, 0x01B3, 0x1B34, 0x3FFA},
		.band5 = {0x0815, 0x3FFF, 0xCF78, 0x0000, 0x29B7},
	},
	{ /* BASS=-12dB, TREBLE=+9dB, Fc=5KHz */
		.name = "HIBOOST",
		.rate = 44100,
		.band1 = {0x0815, 0xC001, 0x3AA4, 0x0003, 0x19A2},
		.band2 = {0x0815, 0xC103, 0x092F, 0x0B55, 0x3F56},
		.band3 = {0x0E0A, 0xC306, 0x1E5C, 0x136E, 0x3856},
		.band4 = {0x2459, 0xF665, 0x0CAA, 0x3F46, 0x3EBB},
		.band5 = {0x5BBB, 0x3FFF, 0xCEB0, 0x0000, 0x28CA},
	},
	{ /* BASS=12dB, TREBLE=+12dB */
		.name = "LOUD12DB",
		.rate = 44100,
		.band1 = {0x7FC1, 0xC001, 0x3EE8, 0x0020, 0x0BC7},
		.band2 = {0x51E9, 0xC016, 0x3C7C, 0x033F, 0x14E9},
		.band3 = {0x1745, 0xC12C, 0x1680, 0x0C2F, 0x3BE9},
		.band4 = {0x4536, 0xD7E2, 0x0ED4, 0x31DD, 0x3E42},
		.band5 = {0x7FEF, 0x3FFF, 0x0BAB, 0x0000, 0x3EED},
	},
	{
		.name = "FLAT",
		.rate = 16000,
		.band1 = {0x2000, 0xC004, 0x4000, 0x0141, 0x0000},
		.band2 = {0x2000, 0xC033, 0x4000, 0x0505, 0x0000},
		.band3 = {0x2000, 0xC268, 0x4000, 0x115F, 0x0000},
		.band4 = {0x2000, 0xDA62, 0x4000, 0x33C6, 0x0000},
		.band5 = {0x2000, 0x4000, 0x4000, 0x0000, 0x0000},
	},
	{
		.name = "LOWPASS1K",
		.rate = 16000,
		.band1 = {0x2000, 0xC004, 0x4000, 0x0141, 0x0000},
		.band2 = {0x5BE8, 0xC3E0, 0x3307, 0x15ED, 0x26A0},
		.band3 = {0x0F71, 0xD15A, 0x08B3, 0x2BD0, 0x3F67},
		.band4 = {0x0815, 0x3FFF, 0xCF78, 0x0000, 0x29B7},
		.band5 = {0x0815, 0x3FFF, 0xCF78, 0x0000, 0x29B7},
	},
	{ /* BASS=-12dB, TREBLE=+9dB, Fc=2KHz */
		.name = "HIBOOST",
		.rate = 16000,
		.band1 = {0x0815, 0xC001, 0x3BD2, 0x0009, 0x16BF},
		.band2 = {0x080E, 0xC17E, 0xF653, 0x0DBD, 0x3F43},
		.band3 = {0x0F80, 0xDF45, 0xEE33, 0x36FE, 0x3D79},
		.band4 = {0x590B, 0x3FF0, 0xE882, 0x02BD, 0x3B87},
		.band5 = {0x4C87, 0xF3D0, 0x063F, 0x3ED4, 0x3FB1},
	},
	{ /* BASS=12dB, TREBLE=+12dB */
		.name = "LOUD12DB",
		.rate = 16000,
		.band1 = {0x7FC1, 0xC001, 0x3D07, 0x0058, 0x1344},
		.band2 = {0x2DA6, 0xC013, 0x3CF1, 0x02FF, 0x138B},
		.band3 = {0x18F1, 0xC08E, 0x244D, 0x0863, 0x34B5},
		.band4 = {0x2BE0, 0xF385, 0x04FD, 0x3EC5, 0x3FCE},
		.band5 = {0x7FEF, 0x4000, 0x0BAB, 0x0000, 0x3EED},
	},
};


static struct max98088_pdata enterprise_max98088_pdata = {
	/* equalizer configuration */
	.eq_cfg = max98088_eq_cfg,
	.eq_cfgcnt = ARRAY_SIZE(max98088_eq_cfg),

	/* debounce time */
	.debounce_time_ms = 200,

	/* microphone configuration */
	.digmic_left_mode = 1,
	.digmic_right_mode = 1,

	/* receiver output configuration */
	.receiver_mode = 0,	/* 0 = amplifier, 1 = line output */
};

static struct pn544_i2c_platform_data nfc_pdata = {
		.irq_gpio = TEGRA_GPIO_PS4,
		.ven_gpio = TEGRA_GPIO_PM6,
		.firm_gpio = 0,
};


static struct i2c_board_info __initdata max98088_board_info = {
	I2C_BOARD_INFO("max98088", 0x10),
	.platform_data = &enterprise_max98088_pdata,
	.irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_HP_DET),
};

static struct i2c_board_info __initdata nfc_board_info = {
	I2C_BOARD_INFO("pn544", 0x28),
	.platform_data = &nfc_pdata,
	.irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PS4),
};

static void enterprise_i2c_init(void)
{
	tegra_i2c_device1.dev.platform_data = &enterprise_i2c1_platform_data;
	tegra_i2c_device2.dev.platform_data = &enterprise_i2c2_platform_data;
	tegra_i2c_device3.dev.platform_data = &enterprise_i2c3_platform_data;
	tegra_i2c_device4.dev.platform_data = &enterprise_i2c4_platform_data;
	tegra_i2c_device5.dev.platform_data = &enterprise_i2c5_platform_data;

	platform_device_register(&tegra_i2c_device5);
	platform_device_register(&tegra_i2c_device4);
	platform_device_register(&tegra_i2c_device3);
	platform_device_register(&tegra_i2c_device2);
	platform_device_register(&tegra_i2c_device1);

	i2c_register_board_info(0, &max98088_board_info, 1);
	i2c_register_board_info(0, &nfc_board_info, 1);
}

static struct platform_device *enterprise_uart_devices[] __initdata = {
	&tegra_uarta_device,
	&tegra_uartb_device,
	&tegra_uartc_device,
	&tegra_uartd_device,
	&tegra_uarte_device,
};

static struct uart_clk_parent uart_parent_clk[] = {
	[0] = {.name = "clk_m"},
	[1] = {.name = "pll_p"},
#ifndef CONFIG_TEGRA_PLLM_RESTRICTED
	[2] = {.name = "pll_m"},
#endif
};
static struct tegra_uart_platform_data enterprise_uart_pdata;

static void __init uart_debug_init(void)
{
	unsigned long rate;
	struct clk *c;

	/* UARTD is the debug port. */
	pr_info("Selecting UARTD as the debug console\n");
	enterprise_uart_devices[3] = &debug_uartd_device;
	debug_uart_port_base = ((struct plat_serial8250_port *)(
			debug_uartd_device.dev.platform_data))->mapbase;
	debug_uart_clk = clk_get_sys("serial8250.0", "uartd");

	/* Clock enable for the debug channel */
	if (!IS_ERR_OR_NULL(debug_uart_clk)) {
		rate = ((struct plat_serial8250_port *)(
			debug_uartd_device.dev.platform_data))->uartclk;
		pr_info("The debug console clock name is %s\n",
						debug_uart_clk->name);
		c = tegra_get_clock_by_name("pll_p");
		if (IS_ERR_OR_NULL(c))
			pr_err("Not getting the parent clock pll_p\n");
		else
			clk_set_parent(debug_uart_clk, c);

		clk_enable(debug_uart_clk);
		clk_set_rate(debug_uart_clk, rate);
	} else {
		pr_err("Not getting the clock %s for debug console\n",
				debug_uart_clk->name);
	}
}

static void __init enterprise_uart_init(void)
{
	int i;
	struct clk *c;

	for (i = 0; i < ARRAY_SIZE(uart_parent_clk); ++i) {
		c = tegra_get_clock_by_name(uart_parent_clk[i].name);
		if (IS_ERR_OR_NULL(c)) {
			pr_err("Not able to get the clock for %s\n",
						uart_parent_clk[i].name);
			continue;
		}
		uart_parent_clk[i].parent_clk = c;
		uart_parent_clk[i].fixed_clk_rate = clk_get_rate(c);
	}
	enterprise_uart_pdata.parent_clk_list = uart_parent_clk;
	enterprise_uart_pdata.parent_clk_count = ARRAY_SIZE(uart_parent_clk);
	tegra_uarta_device.dev.platform_data = &enterprise_uart_pdata;
	tegra_uartb_device.dev.platform_data = &enterprise_uart_pdata;
	tegra_uartc_device.dev.platform_data = &enterprise_uart_pdata;
	tegra_uartd_device.dev.platform_data = &enterprise_uart_pdata;
	tegra_uarte_device.dev.platform_data = &enterprise_uart_pdata;

	/* Register low speed only if it is selected */
	if (!is_tegra_debug_uartport_hs())
		uart_debug_init();

	platform_add_devices(enterprise_uart_devices,
				ARRAY_SIZE(enterprise_uart_devices));
}



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

static struct platform_device tegra_camera = {
	.name = "tegra_camera",
	.id = -1,
};

static struct tegra_max98088_platform_data enterprise_audio_pdata = {
	.gpio_spkr_en		= -1,
	.gpio_hp_det		= TEGRA_GPIO_HP_DET,
	.gpio_hp_mute		= -1,
	.gpio_int_mic_en	= -1,
	.gpio_ext_mic_en	= -1,
	.audio_port_id		= {
		[HIFI_CODEC] = 0,
		[BASEBAND] = 2,
		[BT_SCO] = 3,
	},
	.baseband_param		= {
		.rate = 8000,
		.channels = 1,
	},
};

static struct platform_device enterprise_audio_device = {
	.name	= "tegra-snd-max98088",
	.id	= 0,
	.dev	= {
		.platform_data  = &enterprise_audio_pdata,
	},
};

static struct resource ram_console_resources[] = {
	{
		.flags = IORESOURCE_MEM,
	},
};

static struct platform_device ram_console_device = {
	.name 		= "ram_console",
	.id 		= -1,
	.num_resources	= ARRAY_SIZE(ram_console_resources),
	.resource	= ram_console_resources,
};

static struct platform_device *enterprise_devices[] __initdata = {
	&tegra_pmu_device,
	&tegra_rtc_device,
	&tegra_udc_device,
#if defined(CONFIG_TEGRA_IOVMM_SMMU)
	&tegra_smmu_device,
#endif
	&tegra_wdt_device,
#if defined(CONFIG_TEGRA_AVP)
	&tegra_avp_device,
#endif
	&tegra_camera,
	&enterprise_bcm4329_rfkill_device,
	&tegra_spi_device4,
	&tegra_hda_device,
#if defined(CONFIG_CRYPTO_DEV_TEGRA_SE)
	&tegra_se_device,
#endif
#if defined(CONFIG_CRYPTO_DEV_TEGRA_AES)
	&tegra_aes_device,
#endif
	&ram_console_device,
};

#define MXT_CONFIG_CRC 0x62F903
/*
 * Config converted from memory-mapped cfg-file with
 * following version information:
 *
 *
 *
 *      FAMILY_ID=128
 *      VARIANT=1
 *      VERSION=32
 *      BUILD=170
 *      VENDOR_ID=255
 *      PRODUCT_ID=TBD
 *      CHECKSUM=0xC189B6
 *
 *
 */

static const u8 config[] = {
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0xFF, 0xFF, 0x32, 0x0A, 0x00, 0x05, 0x01, 0x00,
        0x00, 0x1E, 0x0A, 0x8B, 0x00, 0x00, 0x13, 0x0B,
        0x00, 0x10, 0x32, 0x03, 0x03, 0x00, 0x03, 0x01,
        0x00, 0x0A, 0x0A, 0x0A, 0x0A, 0xBF, 0x03, 0x1B,
        0x02, 0x00, 0x00, 0x37, 0x37, 0x00, 0x00, 0x00,
        0x00, 0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0xA9, 0x7F, 0x9A, 0x0E, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x03, 0x23, 0x00, 0x00, 0x00, 0x0A,
        0x0F, 0x14, 0x19, 0x03, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x01, 0x00, 0x03, 0x08, 0x10,
        0x00
};

static struct mxt_platform_data atmel_mxt_info = {
        .x_line         = 19,
        .y_line         = 11,
        .x_size         = 960,
        .y_size         = 540,
        .blen           = 0x10,
        .threshold      = 0x32,
        .voltage        = 3300000,              /* 3.3V */
        .orient         = 3,
        .config         = config,
        .config_length  = 168,
        .config_crc     = MXT_CONFIG_CRC,
        .irqflags       = IRQF_TRIGGER_FALLING,
/*      .read_chg       = &read_chg, */
        .read_chg       = NULL,
};

static struct i2c_board_info __initdata atmel_i2c_info[] = {
	{
		I2C_BOARD_INFO("atmel_mxt_ts", MXT224_I2C_ADDR1),
		.irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PH6),
		.platform_data = &atmel_mxt_info,
	}
};

static int __init enterprise_touch_init(void)
{
	tegra_gpio_enable(TEGRA_GPIO_PH6);
	tegra_gpio_enable(TEGRA_GPIO_PF5);

	gpio_request(TEGRA_GPIO_PH6, "atmel-irq");
	gpio_direction_input(TEGRA_GPIO_PH6);

	gpio_request(TEGRA_GPIO_PF5, "atmel-reset");
	gpio_direction_output(TEGRA_GPIO_PF5, 0);
	msleep(1);
	gpio_set_value(TEGRA_GPIO_PF5, 1);
	msleep(100);

	i2c_register_board_info(1, atmel_i2c_info, 1);

	return 0;
}

static struct usb_phy_plat_data tegra_usb_phy_pdata[] = {
	[0] = {
			.instance = 0,
			.vbus_gpio = -1,
			.vbus_reg_supply = "usb_vbus",
			.vbus_irq = ENT_TPS80031_IRQ_BASE +
							TPS80031_INT_VBUS_DET,
	},
	[1] = {
			.instance = 1,
			.vbus_gpio = -1,
	},
	[2] = {
			.instance = 2,
			.vbus_gpio = -1,
	},
};

static struct tegra_uhsic_config uhsic_phy_config = {
	.enable_gpio = -1,
	.reset_gpio = -1,
	.sync_start_delay = 9,
	.idle_wait_delay = 17,
	.term_range_adj = 0,
	.elastic_underrun_limit = 16,
	.elastic_overrun_limit = 16,
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

static struct tegra_otg_platform_data tegra_otg_pdata = {
	.ehci_device = &tegra_ehci1_device,
	.ehci_pdata = &tegra_ehci_pdata[0],
};

struct platform_device *tegra_usb_hsic_host_register(void)
{
	struct platform_device *pdev;
	void *platform_data;
	int val;

	pdev = platform_device_alloc(tegra_ehci2_device.name,
		tegra_ehci2_device.id);
	if (!pdev)
		return NULL;

	val = platform_device_add_resources(pdev, tegra_ehci2_device.resource,
		tegra_ehci2_device.num_resources);
	if (val)
		goto error;

	pdev->dev.dma_mask =  tegra_ehci2_device.dev.dma_mask;
	pdev->dev.coherent_dma_mask = tegra_ehci2_device.dev.coherent_dma_mask;

	platform_data = kmalloc(sizeof(struct tegra_ehci_platform_data),
		GFP_KERNEL);
	if (!platform_data)
		goto error;

	memcpy(platform_data, &tegra_ehci_uhsic_pdata,
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

void tegra_usb_hsic_host_unregister(struct platform_device *pdev)
{
	platform_device_unregister(pdev);
}

static int enterprise_usb_hsic_postsupend(void)
{
	pr_debug("%s\n", __func__);
#ifdef CONFIG_TEGRA_BB_XMM_POWER
	baseband_xmm_set_power_status(BBXMM_PS_L2);
#endif
	return 0;
}

static int enterprise_usb_hsic_preresume(void)
{
	pr_debug("%s\n", __func__);
#ifdef CONFIG_TEGRA_BB_XMM_POWER
	baseband_xmm_set_power_status(BBXMM_PS_L2TOL0);
#endif
	return 0;
}

static int enterprise_usb_hsic_phy_ready(void)
{
	pr_debug("%s\n", __func__);
#ifdef CONFIG_TEGRA_BB_XMM_POWER
	baseband_xmm_set_power_status(BBXMM_PS_L0);
#endif
	return 0;
}

static int enterprise_usb_hsic_phy_off(void)
{
	pr_debug("%s\n", __func__);
#ifdef CONFIG_TEGRA_BB_XMM_POWER
	baseband_xmm_set_power_status(BBXMM_PS_L3);
#endif
	return 0;
}

static void enterprise_usb_init(void)
{
	struct	fsl_usb2_platform_data *udc_pdata;

	tegra_usb_phy_init(tegra_usb_phy_pdata, ARRAY_SIZE(tegra_usb_phy_pdata));

	tegra_otg_device.dev.platform_data = &tegra_otg_pdata;
	platform_device_register(&tegra_otg_device);

	udc_pdata = tegra_udc_device.dev.platform_data;
}

static struct platform_device *enterprise_audio_devices[] __initdata = {
	&tegra_ahub_device,
	&tegra_dam_device0,
	&tegra_dam_device1,
	&tegra_dam_device2,
	&tegra_i2s_device2,
	&tegra_i2s_device3,
	&tegra_spdif_device,
	&spdif_dit_device,
	&bluetooth_dit_device,
	&baseband_dit_device,
	&tegra_pcm_device,
	&enterprise_audio_device,
};

static void enterprise_audio_init(void)
{
	struct board_info board_info;

	tegra_get_board_info(&board_info);
	if (board_info.board_id == BOARD_E1197) {
		platform_device_register(&tegra_i2s_device1);
		enterprise_audio_pdata.audio_port_id[HIFI_CODEC] = 1;
	} else
		platform_device_register(&tegra_i2s_device0);

	platform_add_devices(enterprise_audio_devices,
			ARRAY_SIZE(enterprise_audio_devices));
}

static void enterprise_gps_init(void)
{
	tegra_gpio_enable(TEGRA_GPIO_PE4);
	tegra_gpio_enable(TEGRA_GPIO_PE5);
}

static struct baseband_power_platform_data tegra_baseband_power_data = {
	.baseband_type = BASEBAND_XMM,
	.modem = {
		.xmm = {
			.bb_rst = XMM_GPIO_BB_RST,
			.bb_on = XMM_GPIO_BB_ON,
			.ipc_bb_wake = XMM_GPIO_IPC_BB_WAKE,
			.ipc_ap_wake = XMM_GPIO_IPC_AP_WAKE,
			.ipc_hsic_active = XMM_GPIO_IPC_HSIC_ACTIVE,
			.ipc_hsic_sus_req = XMM_GPIO_IPC_HSIC_SUS_REQ,
		},
	},
};

static struct platform_device tegra_baseband_power_device = {
	.name = "baseband_xmm_power",
	.id = -1,
	.dev = {
		.platform_data = &tegra_baseband_power_data,
	},
};

static struct platform_device tegra_baseband_power2_device = {
	.name = "baseband_xmm_power2",
	.id = -1,
	.dev = {
		.platform_data = &tegra_baseband_power_data,
	},
};

#ifdef CONFIG_TEGRA_BB_M7400
static union tegra_bb_gpio_id m7400_gpio_id = {
	.m7400 = {
		.pwr_status = GPIO_BB_RESET,
		.pwr_on = GPIO_BB_PWRON,
		.uart_awr = GPIO_BB_APACK,
		.uart_cwr = GPIO_BB_CPACK,
		.usb_awr = GPIO_BB_APACK2,
		.usb_cwr = GPIO_BB_CPACK2,
		.service = GPIO_BB_RSVD2,
		.resout2 = GPIO_BB_RSVD1,
	},
};

static struct tegra_bb_pdata m7400_pdata = {
	.id = &m7400_gpio_id,
	.device = &tegra_ehci2_device,
	.ehci_register = tegra_usb_hsic_host_register,
	.ehci_unregister = tegra_usb_hsic_host_unregister,
	.bb_id = TEGRA_BB_M7400,
};

static struct platform_device tegra_baseband_m7400_device = {
	.name = "tegra_baseband_power",
	.id = -1,
	.dev = {
		.platform_data = &m7400_pdata,
	},
};
#endif

static void enterprise_baseband_init(void)
{
	int modem_id = tegra_get_modem_id();

	switch (modem_id) {
	case TEGRA_BB_PH450: /* PH450 ULPI */
		enterprise_modem_init();
		break;
	case TEGRA_BB_XMM6260: /* XMM6260 HSIC */
		/* xmm baseband - do not switch off phy during suspend */
		tegra_ehci_uhsic_pdata.power_down_on_bus_suspend = 0;
		uhsic_phy_config.postsuspend = enterprise_usb_hsic_postsupend;
		uhsic_phy_config.preresume = enterprise_usb_hsic_preresume;
		uhsic_phy_config.usb_phy_ready = enterprise_usb_hsic_phy_ready;
		uhsic_phy_config.post_phy_off = enterprise_usb_hsic_phy_off;
		/* enable XMM6260 baseband gpio(s) */
		tegra_gpio_enable(tegra_baseband_power_data.modem.generic
			.mdm_reset);
		tegra_gpio_enable(tegra_baseband_power_data.modem.generic
			.mdm_on);
		tegra_gpio_enable(tegra_baseband_power_data.modem.generic
			.ap2mdm_ack);
		tegra_gpio_enable(tegra_baseband_power_data.modem.generic
			.mdm2ap_ack);
		tegra_gpio_enable(tegra_baseband_power_data.modem.generic
			.ap2mdm_ack2);
		tegra_gpio_enable(tegra_baseband_power_data.modem.generic
			.mdm2ap_ack2);
		tegra_baseband_power_data.hsic_register =
						&tegra_usb_hsic_host_register;
		tegra_baseband_power_data.hsic_unregister =
						&tegra_usb_hsic_host_unregister;
		platform_device_register(&tegra_baseband_power_device);
		platform_device_register(&tegra_baseband_power2_device);
		break;
#ifdef CONFIG_TEGRA_BB_M7400
	case TEGRA_BB_M7400: /* M7400 HSIC */
		tegra_ehci2_device.dev.platform_data
			= &tegra_ehci_uhsic_pdata;
		platform_device_register(&tegra_baseband_m7400_device);
		break;
#endif
	}
}

static void enterprise_nfc_init(void)
{
	tegra_gpio_enable(TEGRA_GPIO_PS4);
	tegra_gpio_enable(TEGRA_GPIO_PM6);
}

static void __init tegra_enterprise_init(void)
{
	tegra_thermal_init(&thermal_data);
	tegra_clk_init_from_table(enterprise_clk_init_table);
	enterprise_pinmux_init();
	enterprise_i2c_init();
	enterprise_uart_init();
	enterprise_usb_init();
	enterprise_tsensor_init();
	platform_add_devices(enterprise_devices, ARRAY_SIZE(enterprise_devices));
	enterprise_regulator_init();
	enterprise_sdhci_init();
#ifdef CONFIG_TEGRA_EDP_LIMITS
	enterprise_edp_init();
#endif
	enterprise_kbc_init();
	enterprise_touch_init();
	enterprise_audio_init();
	enterprise_gps_init();
	enterprise_baseband_init();
	enterprise_panel_init();
	enterprise_setup_bluesleep();
	enterprise_emc_init();
	enterprise_sensors_init();
	enterprise_suspend_init();
	enterprise_bpc_mgmt_init();
	tegra_release_bootloader_fb();
	enterprise_nfc_init();
}

static void __init tegra_enterprise_ramconsole_reserve(unsigned long size)
{
	struct resource *res;
	long ret;

	res = platform_get_resource(&ram_console_device, IORESOURCE_MEM, 0);
	if (!res) {
		pr_err("Failed to find memory resource for ram console\n");
		return;
	}
	res->start = memblock_end_of_DRAM() - size;
	res->end = res->start + size - 1;
	ret = memblock_remove(res->start, size);
	if (ret) {
		ram_console_device.resource = NULL;
		ram_console_device.num_resources = 0;
		pr_err("Failed to reserve memory block for ram console\n");
	}
}

static void __init tegra_enterprise_reserve(void)
{
#if defined(CONFIG_NVMAP_CONVERT_CARVEOUT_TO_IOVMM)
	tegra_reserve(0, SZ_4M, SZ_8M);
#else
	tegra_reserve(SZ_128M, SZ_4M, SZ_8M);
#endif
	tegra_enterprise_ramconsole_reserve(SZ_1M);
}

MACHINE_START(TEGRA_ENTERPRISE, "tegra_enterprise")
	.boot_params    = 0x80000100,
	.map_io         = tegra_map_common_io,
	.reserve        = tegra_enterprise_reserve,
	.init_early	= tegra_init_early,
	.init_irq       = tegra_init_irq,
	.timer          = &tegra_timer,
	.init_machine   = tegra_enterprise_init,
MACHINE_END
