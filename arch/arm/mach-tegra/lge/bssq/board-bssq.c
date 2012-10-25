/*
 * arch/arm/mach-tegra/board-bssq.c
 *
 * Copyright (c) 2010 - 2011, NVIDIA Corporation.
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
#include <linux/i2c-gpio.h>
#include <linux/synaptics_i2c_rmi.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/i2c-tegra.h>
#include <linux/gpio.h>
#include <linux/gpio_keys.h>
#include <linux/input.h>
#include <linux/platform_data/tegra_usb.h>
#include <linux/mfd/max8907c.h>
#include <linux/memblock.h>
#include <linux/tegra_uart.h>
#include <linux/mfd/wm8994/pdata.h>

#if defined(CONFIG_LGE_BROADCAST_TDMB)
#include <linux/spi-tegra.h>
#endif

#include <mach/clk.h>
#include <mach/iomap.h>
#include <mach/irqs.h>
#include <mach/pinmux.h>
#include <mach/iomap.h>
#include <mach/io.h>
#include <mach/i2s.h>
#include <mach/tegra_wm8994_pdata.h>

#if defined(CONFIG_SUBPMIC_LP8720)
#include <lge/lp8720.h>
#endif
//20110525 calvin.hwang@lge.com Camsensor ks1001 merge [S]
//#if defined(CONFIG_CAM_PMIC)
//#include <mach/bssq_cam_pmic.h>
//#endif
//20110525 calvin.hwang@lge.com Camsensor ks1001 merge [E]
#include <media/tegra_camera.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <mach/usb_phy.h>

#include <mach-tegra/board.h>
#include <mach-tegra/clock.h>
#include <lge/board-bssq.h>
#include <mach-tegra/devices.h>
#include <mach-tegra/gpio-names.h>
#include <mach-tegra/fuse.h>
#include <mach-tegra/pm.h>
#include <lge/bssq_devices.h>
//#include <linux/lge_hw_rev.h>
#include <mach-tegra/wakeups-t2.h>
//LGE_CHANGE_S [minwook.huh@lge.com] 2012-06-20 for Bluetooth bring-up
#include <linux/lbee9qmb-rfkill.h>
//LGE_CHANGE_SE

//LGE_CHANGE_S [bae.cheolhwan@lge.com] 2012-04-05, Added For USB Wakeup. (from STAR)
#include <../pm-irq.h>
#include <linux/interrupt.h>
//LGE_CHANGE_E [bae.cheolhwan@lge.com] 2012-04-05, Added For USB Wakeup. (from STAR)
#if defined (CONFIG_BSSQ_REBOOT_MONITOR)
extern void bssq_setup_reboot(void);
#endif
int muic_boot_keeping;
int muic_boot_path;
int half_charging_status;

static struct platform_device *bssq_uart_devices[] __initdata = {
	&tegra_uarta_device,
	&tegra_uartb_device,
	&tegra_uartc_device,
};

struct uart_clk_parent uart_parent_clk[] = {
	[0] = {.name = "pll_p"},
	[1] = {.name = "pll_m"},
	[2] = {.name = "clk_m"},
};

static struct tegra_uart_platform_data bssq_uart_pdata;

static void __init uart_debug_init(void)
{
	unsigned long rate;
	struct clk *debug_uart_clk;
	struct clk *c;

	/* UARTB is the debug port. */
	pr_info("Selecting UARTB as the debug console\n");
	bssq_uart_devices[1] = &debug_uartb_device;
	debug_uart_port_base = ((struct plat_serial8250_port *)(
	debug_uartb_device.dev.platform_data))->mapbase;
	debug_uart_clk = clk_get_sys("serial8250.0", "uartb");

	/* Clock enable for the debug channel */
	if (!IS_ERR_OR_NULL(debug_uart_clk)) {
		rate = ((struct plat_serial8250_port *)(
		debug_uartb_device.dev.platform_data))->uartclk;
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

static void __init bssq_uart_init(void)
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
	bssq_uart_pdata.parent_clk_list = uart_parent_clk;
	bssq_uart_pdata.parent_clk_count = ARRAY_SIZE(uart_parent_clk);

	tegra_uarta_device.dev.platform_data = &bssq_uart_pdata;
	tegra_uartb_device.dev.platform_data = &bssq_uart_pdata;
	tegra_uartc_device.dev.platform_data = &bssq_uart_pdata;

	if (!is_tegra_debug_uartport_hs())
		uart_debug_init();

	platform_add_devices(bssq_uart_devices,
				ARRAY_SIZE(bssq_uart_devices));
}

static struct resource bssq_bcm4329_rfkill_resources[] = {
	{
		.name	= "bcm4329_nshutdown_gpio",
		.start	= TEGRA_GPIO_PU1,
		.end	= TEGRA_GPIO_PU1,
		.flags	= IORESOURCE_IO,
	},
};

static struct platform_device bssq_bcm4329_rfkill_device = {
	.name		= "bcm4329_rfkill",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(bssq_bcm4329_rfkill_resources),
	.resource	= bssq_bcm4329_rfkill_resources,
};

static struct resource bssq_bluesleep_resources[] = {
	[0] = {
		.name = "gpio_host_wake",
			.start  = TEGRA_GPIO_PU6,
			.end    = TEGRA_GPIO_PU6,
			.flags  = IORESOURCE_IO,
	},
	[1] = {
		.name = "gpio_ext_wake",
			.start  = TEGRA_GPIO_PU1,
			.end    = TEGRA_GPIO_PU1,
			.flags  = IORESOURCE_IO,
	},
	[2] = {
		.name = "host_wake",
			.start  = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PU6),
			.end    = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PU6),
			.flags  = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE,
	},
};

static struct platform_device bssq_bluesleep_device = {
	.name           = "bluesleep",
	.id             = -1,
	.num_resources  = ARRAY_SIZE(bssq_bluesleep_resources),
	.resource       = bssq_bluesleep_resources,
};

static void __init bssq_setup_bluesleep(void)
{
	platform_device_register(&bssq_bluesleep_device);
	tegra_gpio_enable(TEGRA_GPIO_PU6);
	tegra_gpio_enable(TEGRA_GPIO_PU1);
	return;
}

static struct tegra_utmip_config utmi_phy_config[] = {
	[0] = {
			.hssync_start_delay = 9,
			.idle_wait_delay = 17,
			.elastic_limit = 16,
			.term_range_adj = 6,
			.xcvr_setup = 15,
			.xcvr_lsfslew = 2,
			.xcvr_lsrslew = 2,
		},
	[1] = {
			.hssync_start_delay = 9,
			.idle_wait_delay = 17,
			.elastic_limit = 16,
			.term_range_adj = 6,
			.xcvr_setup = 8,
			.xcvr_lsfslew = 2,
			.xcvr_lsrslew = 2,
		},
};

static struct tegra_ulpi_config ulpi_phy_config = {
	.reset_gpio = TEGRA_GPIO_PG2,
	.clk = "cdev2",
};

static __initdata struct tegra_clk_init_table bssq_clk_init_table_300MHz[] = {
	/* name			parent			rate		enabled */
	{ "uarta",		"pll_m",		600000000,	false},
	{ "uartb",		"pll_p",		216000000,	true},
	{ "uartd",		"pll_p",		216000000,	false},
	{ "uartc",		"pll_m",		600000000,	true},
	{ "pll_m",		"clk_m",		600000000,	true},
	{ "host1x",		"pll_p",		108000000,	false},
	{ "2d",			"pll_m",		50000000,	true},
	{ "epp",		"pll_m",		50000000,	true},
	{ "emc",		"pll_m",		600000000,	true},
	{ "sdmmc1",     "pll_p", 		52000000,	false},
	{ "sdmmc4",     "pll_p", 		52000000,	true},
	{ "blink",		"clk_32k",		32768,		true},
	{ "pll_a",		NULL,			11289600,	true},
	{ "pll_a_out0",	NULL,			11289600,	true},
	{ "i2s1",		"pll_a_out0",	2822400,	true},
	{ "i2s2",		"pll_a_out0",	2822400,	true},
	{ "audio",		"pll_a_out0",	11289600,	true},
	{ "audio_2x",	"audio",		22579200,	true},
	{ "spdif_out",	"pll_a_out0",		5644800,	false}, // 20110624 chan.jeong@lge.com X2 hdmi audio
	{ /*"clk_dev1"*/"cdev1",	NULL,			26000000,	true},
	{ "pwm", 		"clk_m", 		12000000,	false},
	{ "kbc",		"clk_32k",		32768,		true},
	{ "sbc1",		"pll_p",		96000000,	true},
	{ "sbc2",		"pll_p",		96000000,	true},
	{ "sbc3",		"pll_p",		96000000,	true},
	{ "vi_sensor",	"pll_p",		0,			false},
	{ "vi",			"pll_p",		144000000,	false},
	{ NULL,			NULL,			0,			0},
};

static __initdata struct tegra_clk_init_table bssq_clk_init_table_380MHz[] = {
	/* name			parent			rate		enabled */
	{ "uarta",		"pll_m",		600000000,	false},
	{ "uartb",		"pll_p",		216000000,	true},
	{ "uartd",		"pll_p",		216000000,	false},
//	{ "uartc",		"pll_m",		600000000,	true},
	{ "uartc",		"pll_m",		760000000,	true},
	{ "host1x",		"pll_p",		108000000,	false},
	{ "2d",			"pll_c",		50000000,	false},
	{ "epp",		"pll_c",		50000000,	false},
	{ "sdmmc1",     "pll_p", 		52000000,	false},
	{ "sdmmc4",     "pll_p", 		52000000,	true},
	{ "blink",		"clk_32k",		32768,		true},
	{ "pll_a",		NULL,			11289600,	true},
	{ "pll_a_out0",	NULL,			11289600,	true},
	{ "i2s1",		"pll_a_out0",	2822400,	true},
	{ "i2s2",		"pll_a_out0",	2822400,	true},
	{ "audio",		"pll_a_out0",	11289600,	true},
	{ "audio_2x",	"audio",		22579200,	true},
	{ "spdif_out",	"pll_a_out0",		5644800,	false}, // 20110624 chan.jeong@lge.com X2 hdmi audio
	{ /*"clk_dev1"*/"cdev1",	NULL,			26000000,	true},
	{ "pwm", 		"clk_m", 		12000000,	false},
	{ "kbc",		"clk_32k",		32768,		true},
	{ "sbc1",		"pll_p",		96000000,	true},
	{ "sbc2",		"pll_p",		96000000,	true},
	{ "sbc3",		"pll_p",		96000000,	true},
	{ "vi_sensor",	"pll_p",		0,			false},
	{ "vi",			"pll_p",		144000000,	false},
	{ NULL,			NULL,			0,			0},
};

// GEN1 USE TOUCH
static struct tegra_i2c_platform_data bssq_i2c1_platform_data = {
	.adapter_nr	= 0,
	.bus_count	= 1,
	.bus_clk_rate	= { 400000, 0 },
	.scl_gpio		= {TEGRA_GPIO_PC4, 0},
	.sda_gpio		= {TEGRA_GPIO_PC5, 0},
	.arb_recovery = arb_lost_recovery,
	.slave_addr = 0xFC,
};

// GEN2 - WM8994, COMPASS, MOTION, GYRO
static const struct tegra_pingroup_config i2c2_ddc = {
	.pingroup	= TEGRA_PINGROUP_DDC,
	.func		= TEGRA_MUX_I2C2,
};

static const struct tegra_pingroup_config i2c2_gen2 = {
	.pingroup	= TEGRA_PINGROUP_PTA,
	.func		= TEGRA_MUX_I2C2,
};

static struct tegra_i2c_platform_data bssq_i2c2_platform_data = {
	.adapter_nr	= 1,
	.bus_count	= 2,
	.bus_clk_rate	= { 100000, 100000/*300000*/ },
	.bus_mux	= { &i2c2_ddc, &i2c2_gen2 },
	.bus_mux_len	= { 1, 1 },
	.scl_gpio		= {0, TEGRA_GPIO_PT5},
	.sda_gpio		= {0, TEGRA_GPIO_PT6},
	.arb_recovery = arb_lost_recovery,
	.slave_addr = 0xFC,
};

// GEN3(CAM_I2C) - 5M, VGA,PROX,SUB_PM
#if defined (CONFIG_KS1001) || defined (CONFIG_KS1103)
static struct tegra_i2c_platform_data bssq_i2c3_platform_data = {
	.adapter_nr	= 3,
	.bus_count	= 1,
	.bus_clk_rate	= { 400000/*300000*/, 0 },
	.scl_gpio		= {TEGRA_GPIO_PBB2, 0},
	.sda_gpio		= {TEGRA_GPIO_PBB3, 0},
	.arb_recovery = arb_lost_recovery,
	.slave_addr = 0xFC,
};
#elif defined (CONFIG_LU6500) || defined (CONFIG_SU880) || defined (CONFIG_KU8800) || defined (CONFIG_LU8800)
static struct tegra_i2c_platform_data bssq_i2c3_platform_data = {
	.adapter_nr	= 3,
	.bus_count	= 1,
	.bus_clk_rate	= { 100000, 0 }, 
	.scl_gpio		= {TEGRA_GPIO_PBB2, 0},
	.sda_gpio		= {TEGRA_GPIO_PBB3, 0},
	.arb_recovery = arb_lost_recovery,
	.slave_addr = 0xFC,
};
#endif	
// PWR_I2C - POWER
static struct tegra_i2c_platform_data bssq_dvc_platform_data = {
	.adapter_nr	= 4,
	.bus_count	= 1,
	.bus_clk_rate	= { 400000, 0 },
	.is_dvc		= true,
	.scl_gpio		= {TEGRA_GPIO_PZ6, 0},
	.sda_gpio		= {TEGRA_GPIO_PZ7, 0},
	.arb_recovery = arb_lost_recovery,
};

#define BACKLIGHT_IC_EN		TEGRA_GPIO_PE3
#define BACKLIGHT_IC_SDA	TEGRA_GPIO_PJ4
#define BACKLIGHT_IC_SCL	TEGRA_GPIO_PJ3

// GPIO I2c - BL DCDC
static struct i2c_gpio_platform_data bssq_gpioi2c1_platform_pdata = {
	.sda_pin = BACKLIGHT_IC_SDA,
	.scl_pin = BACKLIGHT_IC_SCL,
	.udelay = 5, /* (500 / udelay) kHz */
	.timeout = 100, /* jiffies */
};

// MUIC - FUEL GAUGE
static struct i2c_gpio_platform_data bssq_gpioi2c2_platform_data = {
	.sda_pin = TEGRA_GPIO_PK4,
	.scl_pin = TEGRA_GPIO_PI7,
	.udelay	= 5, /* (500 / udelay) kHz */
	.timeout = 100, /* jiffies */
};

static void bssq_i2c_init(void)
{
	tegra_i2c_device1.dev.platform_data = &bssq_i2c1_platform_data;
	tegra_i2c_device2.dev.platform_data = &bssq_i2c2_platform_data;
	tegra_i2c_device3.dev.platform_data = &bssq_i2c3_platform_data;
	tegra_i2c_device4.dev.platform_data = &bssq_dvc_platform_data;

	platform_device_register(&tegra_i2c_device4);
	platform_device_register(&tegra_i2c_device3);
	platform_device_register(&tegra_i2c_device2);
	platform_device_register(&tegra_i2c_device1);
}

//20120517 youngmin.kim@lge.com. debounce_interval 10->25
#define GPIO_KEY(_id, _gpio, _iswake, _type)	\
	{								\
		.code = _id,				\
		.gpio = TEGRA_GPIO_##_gpio,	\
		.active_low = 1,			\
		.desc = #_id,				\
		.type = _type,				\
		.wakeup = _iswake,			\
		.debounce_interval = 20,	\
	}

#define PMC_WAKE_STATUS 0x14
#define KEY_NOT_WAKE (-1)

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

static int bssq_wakeup_key(void)
{
	unsigned long status = readl(IO_ADDRESS(TEGRA_PMC_BASE) + PMC_WAKE_STATUS);
	return (status & TEGRA_WAKE_GPIO_PV2) ? KEY_POWER : (status & TEGRA_WAKE_GPIO_PU5) ? SW_LID : KEY_NOT_WAKE;
}

static struct gpio_keys_button bssq_keys[] = {
	[0] = GPIO_KEY(KEY_VOLUMEDOWN, PG1, 0, EV_KEY),
	[1] = GPIO_KEY(KEY_VOLUMEUP, PG0, 0, EV_KEY),
	[2] = GPIO_KEY(KEY_POWER, PV2, 1, EV_KEY),
//	[3] = VIRTUAL_GPIO_KEY(KEY_TESTMODE_UNLOCK, 0, EV_KEY),// 20110724 deukgi.shin@lge.com ADD TESTMODE UNLOCK KEY 
#if defined(CONFIG_KS1001) || defined(CONFIG_LU6500)
	[3] = GPIO_KEY(SW_LID, PU5, 1, EV_SW),
#endif
	
};
// 20110503 bg80.song@lge.com gpio key setup [E]

static struct gpio_keys_platform_data bssq_keys_platform_data = {
   .buttons  = bssq_keys,
   .nbuttons = ARRAY_SIZE(bssq_keys),
   .wakeup_key	= bssq_wakeup_key,
 };

struct platform_device bssq_keys_device = {
   .name = "gpio-keys",
   .id   = -1,
   .dev  = {
       .platform_data = &bssq_keys_platform_data,
   },
};

static struct platform_device *bssq_keys_devices[] __initdata = {
	&bssq_keys_device,
};

#ifdef CONFIG_RTC_DRV_TEGRA
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

//SJ_VIB_S
#ifdef CONFIG_BSSQ_VIBRATOR
/*
static struct resource tegra_pwfm0_resource = {
    .start  = TEGRA_PWFM0_BASE,
    .end    = TEGRA_PWFM0_BASE + TEGRA_PWFM0_SIZE - 1,
    .flags  = IORESOURCE_MEM,
};


struct platform_device tegra_pwfm0_device = {
    .name       = "tegra_pwm",
    .id     = 0,
    .num_resources  = 1,
    .resource   = &tegra_pwfm0_resource,
};
*/
#endif
//SJ_VIB_E

static struct platform_device tegra_camera = {
	.name = "tegra_camera",
	.id = -1,
};

static struct platform_device *bssq_devices[] __initdata = {
	&tegra_pmu_device,
#if defined(CONFIG_ANDROID_RAM_CONSOLE)
        // LGE_UPDATE_S yoolje.cho@lge.com [[
        &ram_console_device,
        // LGE_UPDATE_E yoolje.cho@lge.com ]]
#endif
	&tegra_udc_device,
	&tegra_gart_device,
	&tegra_wdt_device,
	&tegra_avp_device,
#ifdef CONFIG_RTC_DRV_TEGRA
	&tegra_rtc_device,
#endif
//SJ_VIB_S
#if defined (CONFIG_BSSQ_VIBRATOR) || (CONFIG_TSPDRV)
    &tegra_pwfm0_device,
#endif
//SJ_VIB_E
#if defined (CONFIG_KS1001) || defined (CONFIG_KS1103)
	&bssq_cam_pmic,
#endif
	&tegra_camera,
	&tegra_i2s_device1,
	&tegra_i2s_device2,
#if defined(CONFIG_SU880) || defined(CONFIG_KU8800) || defined(CONFIG_LU8800) || defined(CONFIG_KS1103)
	&tegra_spdif_device,
#endif
	&tegra_das_device,
	&spdif_dit_device,
	&bluetooth_dit_device,
#ifdef CONFIG_BCM4329_RFKILL
//	&bssq_bcm4329_rfkill_device,
#endif
	&tegra_pcm_device,
//LGE_UPDATE_S, bae.cheolhwan@lge.com 2012.05.09. Power consumption for audio. (nVidia Patch)
#if !defined(CONFIG_MACH_BSSQ)
	&bssq_audio_device,
#endif
#if defined (CONFIG_BSSQ_CHARGER_RT)
        &bssq_charger_ic_device,
#endif
//LGE_UPDATE_E, bae.cheolhwan@lge.com 2012.05.09.
};

#if defined(CONFIG_LGE_BROADCAST_TDMB)
struct spi_clk_parent spi_dmb_clk[] = {
	[0] = {.name = "pll_p"},
#ifndef CONFIG_TEGRA_PLLM_RESTRICTED
	[1] = {.name = "pll_m"},
	[2] = {.name = "clk_m"},
#else
	[1] = {.name = "clk_m"},
#endif
};

static struct tegra_spi_device_controller_data bssq_spi_bus3_controller_data = {
        .is_hw_based_cs = true,
        .cs_setup_clk_count = 0,
        .cs_hold_clk_count = 0,
};

static struct tegra_spi_platform_data dmb_spi_pdata = {
	.is_dma_based		= true,
	.max_dma_buffer		= (16 * 1024),
	.is_clkon_always	= false,
	.max_rate		= 48000000,
};

static void bssq_dmb_init(void)
{
	int i;
	struct clk* c;
	
	for(i=0; i<ARRAY_SIZE(spi_dmb_clk); ++i) {
		c = tegra_get_clock_by_name(spi_dmb_clk[i].name);
		if(IS_ERR_OR_NULL(c)){
			pr_err("DMB Not able to get the clock for %s\n",
				spi_dmb_clk[i].name);
			continue;
		}
		spi_dmb_clk[i].parent_clk = c;
		spi_dmb_clk[i].fixed_clk_rate = clk_get_rate(c);
	}
	dmb_spi_pdata.parent_clk_list = spi_dmb_clk;
	dmb_spi_pdata.parent_clk_count = ARRAY_SIZE(spi_dmb_clk);
	tegra_spi_device3.dev.platform_data = &dmb_spi_pdata;
	
	//platform_add_devices(dmb_spi_devices, ARRAY_SIZE(dmb_spi_devices));
	platform_device_register(&tegra_spi_device3);
	spi_register_board_info(bssq_spi_bus3_devices_info, ARRAY_SIZE(bssq_spi_bus3_devices_info));
}
#endif /* CONFIG_LGE_BROADCAST */


static int __init bssq_proximity_init(void)
{
//#ifdef CONFIG_BSSQ_PROXIMITY
#ifdef CONFIG_SENSOR_GP2A
	tegra_gpio_enable(TEGRA_GPIO_PX5);
#if defined(CONFIG_LU6500)
	i2c_register_board_info(0, bssq_i2c_proxi_info, ARRAY_SIZE(bssq_i2c_proxi_info));
#else
	i2c_register_board_info(2, bssq_i2c_proxi_info, ARRAY_SIZE(bssq_i2c_proxi_info));
#endif

#endif
	return 0;
}


static int __init bssq_sensor_init(void)
{
    // 20110603 woo.jung@lge.com Movement Sensor Chip [S]
    signed char gyro[MPU_NUM_AXES * MPU_NUM_AXES] = { 0,  1,  0,
                                                      1,  0,  0,
                                                      0,  0, -1 };
    signed char accel[MPU_NUM_AXES * MPU_NUM_AXES] = {-1,  0,  0,
                                                       0,  1,  0,
                                                       0,  0, -1 };
    signed char compass[MPU_NUM_AXES * MPU_NUM_AXES] = { 0, -1,  0,
                                                        -1,  0,  0,
                                                         0,  0, -1};
    #if defined(CONFIG_KS1001)
        if(get_lge_pcb_revision() > REV_A)
        {
            //gyro cal
            memcpy(mpu3050_data.orientation, gyro, MPU_NUM_AXES * MPU_NUM_AXES);
            //accel cal
            memcpy(mpu3050_data.accel.orientation, accel, MPU_NUM_AXES * MPU_NUM_AXES);
            //compass cal
            memcpy(mpu3050_data.compass.orientation , compass, MPU_NUM_AXES * MPU_NUM_AXES);
        }
        if(get_lge_pcb_revision() > REV_B)
        {
            bssq_i2c_sensor_info[0].irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PF6);
        }
    #elif defined (CONFIG_KS1103)
        {
            bssq_i2c_sensor_info[0].irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PF6);
        }
    #endif

	i2c_register_board_info(2, bssq_i2c_sensor_info, ARRAY_SIZE(bssq_i2c_sensor_info));
	return 0;
}

//20110819 woo.jung@lge.com Compass Porting [S]
#if defined(CONFIG_KS1103)
static int __init bssq_sensor_compass_init(void)
{
#if 1
    printk("bssq_sensor_compass_init CALL!!!!!!!!!!!!!!!!!\n");
    i2c_register_board_info(2, bssq_i2c_sensor_compass, ARRAY_SIZE(bssq_i2c_sensor_compass));
#endif
	return 0;
}
#endif
//20110819 woo.jung@lge.com Compass Porting [E]



static void bssq_gpioi2c_init(void)
{
	tegra_gpio_enable(BACKLIGHT_IC_SDA);
	tegra_gpio_enable(BACKLIGHT_IC_SCL);

	tegra_gpioi2c_device1.dev.platform_data = &bssq_gpioi2c1_platform_pdata;
	tegra_gpioi2c_device2.dev.platform_data = &bssq_gpioi2c2_platform_data;
	
	platform_device_register(&tegra_gpioi2c_device1);
	platform_device_register(&tegra_gpioi2c_device2);
}

static void bssq_spi_init(void)
{
//20110614 ws.yang@lge.com add to su880 or ku880 [S]
#if defined (CONFIG_SU880) || defined (CONFIG_KU8800) 
	platform_device_register(&tegra_spi_device1);
#else
	platform_device_register(&tegra_spi_slave_device1);
#if defined(CONFIG_DUAL_SPI) //2011908 ws.yang@lge.com
	platform_device_register(&tegra_spi_slave_device2);
#endif
#endif	
//20110614 ws.yang@lge.com add to su880 or ku880 [E]	
#if !defined (CONFIG_LU6500)
	platform_device_register(&tegra_spi_device3);
#endif
}

#define APBDEV_PMC_NO_IOPOWER_0		0x44

static void __iomem *pmc = IO_ADDRESS(TEGRA_PMC_BASE);

static void bssq_camera_init(void)
{
	u32 reg_data = 0;

//forced clear No IO Power Register of vi
	reg_data = readl(pmc + APBDEV_PMC_NO_IOPOWER_0);
	reg_data &= ~(0x10);
	writel(reg_data, pmc + APBDEV_PMC_NO_IOPOWER_0);

	i2c_register_board_info(3, bssq_i2c_bus3_devices_info, ARRAY_SIZE(bssq_i2c_bus3_devices_info));
}

static int __init bssq_fuelgauge_init(void)
{
	i2c_register_board_info(6, tegra_i2c_fuelgauge_info, ARRAY_SIZE(tegra_i2c_fuelgauge_info));
	return 0;
}

static int __init bssq_muic_init(void)
{
#if defined (CONFIG_MACH_BSSQ)
#if ! defined(CONFIG_LU6500) || ! defined(CONFIG_SU880) || ! defined(CONFIG_KU8800)
	tegra_pm_irq_set_wake_type( TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PU5), IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING);
#endif//CONFIG_LU6500
#endif
#if defined(CONFIG_LU6500) || defined(CONFIG_SU880) || defined(CONFIG_KU8800)
        i2c_register_board_info(6, tegra_i2c_muic_info, ARRAY_SIZE(tegra_i2c_muic_info));
#else 
	i2c_register_board_info(6, bssq_i2c_bus6_devices_info, ARRAY_SIZE(bssq_i2c_bus6_devices_info));
#endif
	return 0;
}

static int __init bssq_charger_init(void)
{
	i2c_register_board_info(6, tegra_i2c_charger_info, ARRAY_SIZE(tegra_i2c_charger_info));
	return 0;
}

// 20110524 bg80.song@lge.com AP Temp Sensor Bring-up [S]
static int __init bssq_ap_temp_init(void)
{

	tegra_gpio_enable(TEGRA_GPIO_PK2);
	gpio_request(TEGRA_GPIO_PK2, "temp_alert");
	gpio_direction_input(TEGRA_GPIO_PK2);
	i2c_register_board_info(4, bssq_ap_temp_info, ARRAY_SIZE(bssq_ap_temp_info));
	return 0;	
}
// 20110524 bg80.song@lge.com AP Temp Sensor Bring-up [E]

static void bssq_keys_init(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(bssq_keys); i++)
		tegra_gpio_enable(bssq_keys[i].gpio);

	printk(KERN_INFO "gpio_keys: key_gpio initializing\n");
}

static int __init bssq_touch_init(void)
{
	tegra_gpio_enable(TEGRA_GPIO_PX6); //irq
	tegra_gpio_enable(TEGRA_GPIO_PK6); //reset pin
	i2c_register_board_info(0, bssq_i2c_touch_info, 1);

	return 0;
}

//SJ_VIB_S
/*
#ifdef CONFIG_BSSQ_VIBRATOR
#include <mach/vibrator.h>
// 20110829 unyou.shim@lge.com Vibrator voltage : requested by HW Hwang Y [S]
#if defined(CONFIG_KS1001) || defined(CONFIG_KS1103)
static struct pwm_vib_platform_data bssq_vib_platform_data = {
    .max_timeout        =   15000,
    .active_low         =   0,
    .initial_vibrate    =   0,
    .pwm_id             =   0,
    .period_ns          =   50000,
    .duty_ns            =   5500, // requested by HW Hwang Y 1250 -> 5500
    .enable             =   TEGRA_GPIO_PU4,
    .power              =   &device_power_control,
};
// 20110829 unyou.shim@lge.com Vibrator voltage : requested by HW Hwang Y [E]

#else
static struct pwm_vib_platform_data bssq_vib_platform_data = {
    .max_timeout        =   15000,
    .active_low         =   0,
    .initial_vibrate    =   0,
    .pwm_id             =   0,
    .period_ns          =   50000,
    .duty_ns            =   1250,//5000, 20110601 seki.park@lge.com motor duty change
    .enable             =   TEGRA_GPIO_PU4,
    .power              =   &device_power_control,
};
#endif
static struct platform_device bssq_vib_device = {
    .name   =   "bssq_vib_name",
    .id     =   -1,
    .dev    =   {
        .platform_data  = &bssq_vib_platform_data,
    },
};
#endif
*/
//SJ_VIB_E

static struct usb_phy_plat_data tegra_usb_phy_pdata[] = {
	[0] = {
			.instance = 0,
			.vbus_irq = MAX8907C_INT_BASE + MAX8907C_IRQ_VCHG_DC_R,
			.vbus_gpio = TEGRA_GPIO_PN6,
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

static struct tegra_ehci_platform_data tegra_ehci_pdata[] = {
	[0] = {
			.phy_config = &utmi_phy_config[0],
//LGE_CHANGE_S, seokjae.yoon@lge.com, 2012-02-28, apply OTG and change flag for powr down on bus suspend
#ifdef CONFIG_USB_TEGRA_OTG
			.operating_mode = TEGRA_USB_OTG,
#else
			.operating_mode = TEGRA_USB_HOST,
#endif
			.power_down_on_bus_suspend = 0,
		},
	[1] = {
			.phy_config = &ulpi_phy_config,
			.operating_mode = TEGRA_USB_HOST,
			.power_down_on_bus_suspend = 1,
		},
	[2] = {
			.phy_config = &utmi_phy_config[1],
			.operating_mode = TEGRA_USB_HOST,
			.power_down_on_bus_suspend = 0,
//LGE_CHANGE_E, seokjae.yoon@lge.com, 2012-02-28, apply OTG and change flag for powr down on bus suspend			
	},
};

static struct tegra_otg_platform_data tegra_otg_pdata = {
	.ehci_device = &tegra_ehci1_device,
	.ehci_pdata = &tegra_ehci_pdata[0],
};

int __init bssq_touch_led_init(void)
{
#ifdef CONFIG_BSSQ_TOUCH_LED
#if defined(CONFIG_LU6500)
	((struct bd2802_led_platform_data*)tegra_i2c_touch_led_info[0].platform_data)->reset_gpio =  	((get_lge_pcb_revision() > REV_D) ? (TEGRA_GPIO_PF5) : (TEGRA_GPIO_PE6) );	
#endif
	i2c_register_board_info(5, tegra_i2c_touch_led_info, ARRAY_SIZE(tegra_i2c_touch_led_info));
#endif
	return	0;
}

int __init bssq_qwerty_led_init(void)
{
#ifdef CONFIG_BSSQ_QWERTY_LED
	platform_device_register(&bssq_qwerty_led_device);
#endif
	return 0;
}

int __init bssq_vibrator_init(void)
{
#ifdef CONFIG_BSSQ_VIBRATOR
	tegra_gpio_enable(TEGRA_GPIO_PU4);
	tegra_gpio_disable(TEGRA_GPIO_PU3);

	platform_device_register(&bssq_vib_device);
#endif
	return	0;
}

static void bssq_power_off(void)
{
	int ret;

	ret = max8907c_power_off();
	if (ret)
		pr_err("whistler: failed to power off\n");

	while (1);
}

static void __init bssq_power_off_init(void)
{
	pm_power_off = bssq_power_off;
}

int __init bssq_modem_init(void)
{
	spi_register_board_info(bssq_spi_bus1_devices_info, ARRAY_SIZE(bssq_spi_bus1_devices_info));
#ifdef CONFIG_DUAL_SPI
	spi_register_board_info(bssq_spi_bus2_devices_info, ARRAY_SIZE(bssq_spi_bus2_devices_info));
#endif
	return 0;
}

static void bssq_usb_init(void)
{
	tegra_usb_phy_init(tegra_usb_phy_pdata, ARRAY_SIZE(tegra_usb_phy_pdata));
#if 0
//taehyun.ahn@lge.com 20110907 Setting the usb serial number
#if defined (CONFIG_KS1001) || defined (CONFIG_KS1103)
	bssq_usb_serial_setup();
#endif
#endif
#ifdef CONFIG_USB_TEGRA_OTG
	tegra_otg_device.dev.platform_data = &tegra_otg_pdata;
	platform_device_register(&tegra_otg_device);
#endif

#if 0
    //LGE_CHANGE_S [bae.cheolhwan@lge.com] 2012-04-05, Added For USB Wakeup. (from STAR)
	tegra_pm_irq_set_wake_type(INT_USB, IRQF_TRIGGER_RISING);
	//LGE_CHANGE_E [bae.cheolhwan@lge.com] 2012-04-05, Added For USB Wakeup. (from STAR)
#endif
#if !defined (CONFIG_MACH_BSSQ)
	tegra_ehci3_device.dev.platform_data = &tegra_ehci_pdata[2];
	platform_device_register(&tegra_ehci3_device);
#endif
}

static int __init bssq_flash_led_init(void)
{
#if defined(CONFIG_LU6500)  // MOBII_CHANGE 20120711 sk.jung@mobii.co.kr : Fixed flash mode
	printk(KERN_INFO "enter %s\n", __func__);
	//tegra_gpio_enable(TEGRA_GPIO_PBB4);
	i2c_register_board_info(5, bssq_flash_led_info, ARRAY_SIZE(bssq_flash_led_info));
#endif
	return 0;
}

static int __init bssq_muic_path_setup(char *line)
{
    if (sscanf(line, "%1d:%1d", &muic_boot_keeping, &muic_boot_path) != 2) {
        muic_boot_keeping = 0;
    muic_boot_path = 0;
    }

    return 1;
}

__setup("muic_path=", bssq_muic_path_setup);

static int __init androidboot_mode(char *boot_mode)
{
	if(strcmp(boot_mode,"charger") == 0)
		half_charging_status = 1;

		return 1;
}

__setup("androidboot.mode=", androidboot_mode);

static void __init tegra_bssq_init(void)
{
	bssq_setup_reboot();
	tegra_clk_init_from_table(bssq_clk_init_table_380MHz);
	bssq_pinmux_init();
	bssq_i2c_init();
	bssq_gpioi2c_init();
	bssq_spi_init();
	bssq_uart_init();
	platform_add_devices(bssq_devices, ARRAY_SIZE(bssq_devices));

	bssq_sdhci_init();
	bssq_regulator_init();
	bssq_audio_init();
	bssq_panel_init();

	platform_add_devices(bssq_keys_devices, ARRAY_SIZE(bssq_keys_devices));

	bssq_proximity_init();
	bssq_touch_init();
	bssq_flash_led_init();
	//bssq_codec_init();
	bssq_sensor_init();
	bssq_fuelgauge_init();
	bssq_charger_init();
	bssq_muic_init();
//	bssq_kbc_init();
//LGE_CHANGE_S [minwook.huh@lge.com] 2012-06-20 for Bluetooth bring-up
	bssq_bt_rfkill(); 
//LGE_CHANGE_S
//MOBII_CHANGES_S 20120626 sgkim@mobii.co.kr - lu6500 qwerty driver porting.
#if defined(CONFIG_LU6500)	
	bssq_kbc_init();
#endif 
//MOBII_CHANGES_E 20120626 sgkim@mobii.co.kr - lu6500 qwerty driver porting.
//	bssq_bt_rfkill();
	bssq_usb_init();
	bssq_power_off_init();
	bssq_emc_init();
//	bssq_baseband_init();
	bssq_camera_init();
	bssq_keys_init();
	bssq_vibrator_init();
	bssq_touch_led_init();
	bssq_qwerty_led_init();
// 20110524 bg80.song@lge.com AP Temp Sensor Bring-up [S]
	bssq_ap_temp_init();
// 20110524 bg80.song@lge.com AP Temp Sensor Bring-up [E]
#ifdef CONFIG_BT_BLUESLEEP
//	bssq_setup_bluesleep();
#endif
	bssq_modem_init();
//20110819 woo.jung@lge.com Compass Porting [S]
#if defined(CONFIG_KS1103)
	bssq_sensor_compass_init();
#endif
//20110819 woo.jung@lge.com Compass Porting [E]

	tegra_release_bootloader_fb();
#if defined(CONFIG_LGE_BROADCAST_TDMB)
	bssq_dmb_init();
#endif /* CONFIG_LGE_BROADCAST */
}

int __init tegra_bssq_protected_aperture_init(void)
{
	tegra_protected_aperture_init(tegra_grhost_aperture);
	return 0;
}

static void __init tegra_bssq_ramconsole_reserve(unsigned long size)
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
#ifdef CONFIG_BSSQ_REBOOT_MONITOR
        res->end = res->start + size/2 -1;
 #endif
}
void __init tegra_bssq_reserve(void)
{
	if (memblock_reserve(0x0, 4096) < 0)
		pr_warn("Cannot reserve first 4K of memory for safety\n");

	if (memblock_end_of_DRAM() == 0x20000000)
	{
        tegra_reserve(SZ_160M, SZ_8M, SZ_16M);	//512MB
    }
    else 
    {
        tegra_reserve(SZ_256M, SZ_8M, SZ_16M);	//1GB
    }
	tegra_bssq_ramconsole_reserve(SZ_1M);
}

MACHINE_START(BSSQ, "bssq")
	.boot_params    = 0x00000100,
	.map_io         = tegra_map_common_io,
	.reserve        = tegra_bssq_reserve,
	.init_early		= tegra_init_early,
	.init_irq       = tegra_init_irq,
	.timer          = &tegra_timer,
	.init_machine   = tegra_bssq_init,
MACHINE_END
