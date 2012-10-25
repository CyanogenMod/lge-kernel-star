/*
 * arch/arm/mach-tegra/board-star.c
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
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/i2c-tegra.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/mfd/max8907c.h>
#include <linux/tegra_uart.h>
#include <linux/switch.h>
#include <linux/memblock.h>
#include <linux/spi-tegra.h>

#include <mach/clk.h>
#include <mach/iomap.h>
#include <mach/irqs.h>
#include <mach/pinmux.h>
#include <mach/iomap.h>
#include <mach/io.h>
#include <mach/i2s.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>

#include <mach-tegra/gpio-names.h>
#include <mach-tegra/clock.h>
#include <mach-tegra/devices.h>
#include <mach-tegra/fuse.h>
#include <mach-tegra/pm.h>
#include <mach-tegra/board.h>
#include <lge/board-star.h>
#include <lge/board-star-gps.h>

//LGE_CHANGE_S [chahee.kim@lge.com] 2012-02-14 
#include <lge/board-star-baseband.h>
//LGE_CHANGE_E [chahee.kim@lge.com] 2012-02-14 
//LGE_CHANGE_S [munho2.lee@lge.com] 2012-02-06 for Bluetooth bring-up
#include <linux/lbee9qmb-rfkill.h>
//LGE_CHANGE_SE

//LGE_CHNAGE_S  euikyeom.kim@lge.com from sunghoon.kim@lge.com
#if defined (CONFIG_STAR_REBOOT_MONITOR)
extern void star_setup_reboot(void);
#else
void star_setup_reboot(void) {}
#endif
//LGE_CHNAGE_E  euikyeom.kim@lge.com from sunghoon.kim@lge.com

static struct platform_device *star_uart_devices[] __initdata = {
	&tegra_uarta_device,
	&tegra_uartb_device,
	&tegra_uartc_device,
//MOBII_CHANGE_S 20120213 dongki.han@lge.com : uart4
	&tegra_uartd_device,
//MOBII_CHANGE_E 20120213 dongki.han@lge.com : uart4
};

struct uart_clk_parent uart_parent_clk[] = {
	[0] = {.name = "pll_p"},
	[1] = {.name = "pll_m"},
	[2] = {.name = "clk_m"},
};

static struct tegra_uart_platform_data star_uart_pdata;

static void __init uart_debug_init(void)
{
	unsigned long rate;
	struct clk *debug_uart_clk;
	struct clk *c;
	int modem_id = tegra_get_modem_id();

	/* UARTB is the debug port. */
	pr_info("Selecting UARTB as the debug console\n");
	star_uart_devices[1] = &debug_uartb_device;
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

static void __init star_uart_init(void)
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
	star_uart_pdata.parent_clk_list = uart_parent_clk;
	star_uart_pdata.parent_clk_count = ARRAY_SIZE(uart_parent_clk);

	tegra_uarta_device.dev.platform_data = &star_uart_pdata;
	tegra_uartb_device.dev.platform_data = &star_uart_pdata;
	tegra_uartc_device.dev.platform_data = &star_uart_pdata;
//MOBII_CHANGE_S 20120213 dongki.han@lge.com : uart4
	tegra_uartd_device.dev.platform_data = &star_uart_pdata;
//MOBII_CHANGE_E 20120213 dongki.han@lge.com : uart4

	/* Register low speed only if it is selected */
	if (!is_tegra_debug_uartport_hs())
		uart_debug_init();

	platform_add_devices(star_uart_devices,
			ARRAY_SIZE(star_uart_devices));
}


	/* name		parent		rate		enabled */
static __initdata struct tegra_clk_init_table star_clk_init_table[] = {
	{ "blink",	"clk_32k",	32768,		true},
//20120525 sgkim@mobii.co.kr Motor PWM change [S]
	{ "pwm",	"clk_m",	12000000,		false},
//20120525 sgkim@mobii.co.kr Motor PWM change [E]
	{ "kbc",	"clk_32k",	32768,		true},
	{ "sdmmc2",	"pll_p",	25000000,	false},
	{ "i2s1",	"pll_a_out0",	0,		false},
	{ "i2s2",	"pll_a_out0",	0,		false},
	{ "spdif_out",	"pll_a_out0",	0,		false},
    //LGE_CHANGE_S jisil.park@lge.com 20120413 RIL SPI
    { "sbc1", "pll_p", 86400000, true}, //21.6 Mhz
    //LGE_CHANGE_E jisil.park@lge.com 20120413 RIL SPI 
	{ NULL,		NULL,		0,		0},
};

//LGE_CHANGE_S [munho2.lee@lge.com] 2012-03-10 BT ADDR Policy.
#if defined(CONFIG_BD_ADDRESS)
static struct platform_device bd_address_device = {
    .name = "bd_address",
    .id = -1,
};
#endif
//LGE_CHANGE_E

static struct tegra_i2c_platform_data star_i2c1_platform_data = {
	.adapter_nr	= 0,
	.bus_count	= 1,
	.bus_clk_rate	= { 400000, 0 },	
	.scl_gpio		= {TEGRA_GPIO_PC4, 0},
	.sda_gpio		= {TEGRA_GPIO_PC5, 0},
	.arb_recovery = arb_lost_recovery,
	.slave_addr = 0xFC,
};

static const struct tegra_pingroup_config i2c2_ddc = {
	.pingroup	= TEGRA_PINGROUP_DDC,
	.func		= TEGRA_MUX_I2C2,
};

static const struct tegra_pingroup_config i2c2_gen2 = {
	.pingroup	= TEGRA_PINGROUP_PTA,
	.func		= TEGRA_MUX_I2C2,
};

static struct tegra_i2c_platform_data star_i2c2_platform_data = {
	.adapter_nr	= 1,
	.bus_count	= 2,
	.bus_clk_rate	= { 100000, 100000 },
	.bus_mux	= { &i2c2_ddc, &i2c2_gen2 },
	.bus_mux_len	= { 1, 1 },	
	.scl_gpio		= {0, TEGRA_GPIO_PT5},
	.sda_gpio		= {0, TEGRA_GPIO_PT6},
	.arb_recovery = arb_lost_recovery,
	.slave_addr = 0xFC,
};

static struct tegra_i2c_platform_data star_i2c3_platform_data = {
	.adapter_nr	= 3,
	.bus_count	= 1,
	.bus_clk_rate	= { 400000, 0 },	
	.scl_gpio		= {TEGRA_GPIO_PBB2, 0},
	.sda_gpio		= {TEGRA_GPIO_PBB3, 0},
	.arb_recovery = arb_lost_recovery,
	.slave_addr = 0xFC,
};

static struct tegra_i2c_platform_data star_dvc_platform_data = {
	.adapter_nr	= 4,
	.bus_count	= 1,
	.bus_clk_rate	= { 400000, 0 },
	.is_dvc		= true,	
	.scl_gpio		= {TEGRA_GPIO_PZ6, 0},
	.sda_gpio		= {TEGRA_GPIO_PZ7, 0},
	.arb_recovery = arb_lost_recovery,
};

static struct i2c_gpio_platform_data star_gpioi2c1_platform_data = {
	.udelay = 2,
	.scl_is_output_only = 1,
	.sda_pin = TEGRA_GPIO_PQ0, 
	.scl_pin = TEGRA_GPIO_PQ1, 
};

static struct i2c_gpio_platform_data star_gpioi2c2_platform_data = {
	.udelay = 2,
	.scl_is_output_only = 1,
	.sda_pin = TEGRA_GPIO_PK4, 
	.scl_pin = TEGRA_GPIO_PI7, 
};

static void star_i2c_init(void)
{
	tegra_i2c_device1.dev.platform_data = &star_i2c1_platform_data;
	tegra_i2c_device2.dev.platform_data = &star_i2c2_platform_data;
	tegra_i2c_device3.dev.platform_data = &star_i2c3_platform_data;
	tegra_i2c_device4.dev.platform_data = &star_dvc_platform_data;

	platform_device_register(&tegra_i2c_device4);
	platform_device_register(&tegra_i2c_device3);
	platform_device_register(&tegra_i2c_device2);
	platform_device_register(&tegra_i2c_device1);
}

static void star_gpioi2c_init(void)
{
	tegra_gpioi2c_device2.dev.platform_data = &star_gpioi2c2_platform_data;
	platform_device_register(&tegra_gpioi2c_device2);
}


static struct platform_device tegra_camera = {
	.name = "tegra_camera",
	.id = -1,
};

static struct platform_device *star_devices[] __initdata = {
	&tegra_pmu_device,
	&tegra_udc_device,
	&tegra_gart_device,
	&tegra_aes_device,
	&tegra_wdt_device,
	&tegra_avp_device,
	&tegra_camera,
	&tegra_i2s_device1,
	&tegra_i2s_device2,
	&tegra_spdif_device,
	&tegra_das_device,
	&spdif_dit_device,
	&bluetooth_dit_device,
	&baseband_dit_device,
	&tegra_pcm_device,
	&star_audio_device,
	&max8922l_charger_ic_device,
	&star_battery_charger_device,
#if defined(CONFIG_STAR_VIBRATOR) || defined(CONFIG_TSPDRV)
	&tegra_pwfm3_device,	//20120525 sgkim@mobii.co.kr Motor PWM change : tegra_pwfm0_device -> tegra_pwfm3_device
#endif
#if defined(CONFIG_BD_ADDRESS)
	&bd_address_device,		//LGE_CHANGE_S [munho2.lee@lge.com] 2012-03-10 BT ADDR Policy.
#endif

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

static struct tegra_spi_device_controller_data star_spi_bus2_controller_data = {
        .is_hw_based_cs = true,
        .cs_setup_clk_count = 0,
        .cs_hold_clk_count = 0,
};

static struct spi_board_info __initdata spi_bus2_devices_info[] = {
	{
		.modalias = "tdmb_lg2102",
		.bus_num = 1,
		.chip_select = 0,
		.mode = SPI_MODE_0,
		.max_speed_hz = (6000*1000),
		.controller_data = &star_spi_bus2_controller_data,
		.irq = 0, // setting in broadcast_lg2102.c
	},
};

static struct tegra_spi_platform_data dmb_spi_pdata = {
	.is_dma_based		= true,
	.max_dma_buffer		= (16 * 1024),
	.is_clkon_always	= false,
	.max_rate		= 48000000,
};

static void star_dmb_init(void)
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
	tegra_spi_device2.dev.platform_data = &dmb_spi_pdata;
	
	//platform_add_devices(dmb_spi_devices, ARRAY_SIZE(dmb_spi_devices));
	platform_device_register(&tegra_spi_device2);
	spi_register_board_info(spi_bus2_devices_info, ARRAY_SIZE(spi_bus2_devices_info));
}
#endif /* CONFIG_LGE_BROADCAST */

#if !defined(CONFIG_BRCM_LPM)
extern void star_setup_bluesleep(void);
#endif

static void __init tegra_star_init(void)
{
	star_setup_reboot();
	tegra_clk_init_from_table(star_clk_init_table);
	star_pinmux_init();
	star_i2c_init();
	star_gpioi2c_init();
	star_uart_init();
	platform_add_devices(star_devices, ARRAY_SIZE(star_devices));
	tegra_ram_console_debug_init();
	star_audio_init();
	star_sdhci_init();
	star_regulator_init();
	star_panel_init();
	star_sensors_init();
	star_touch_init();
	star_misc_init();
	star_kbc_init();
	star_bt_rfkill(); 
	star_gps_init();
	star_usb_init();
#if defined(CONFIG_STAR_TOUCH_LED)
	star_touch_led_init();
#endif
	star_power_off_init();
	star_emc_init();
	star_baseband_init();
#if !defined(CONFIG_BRCM_LPM)
	star_setup_bluesleep();
#endif
	tegra_release_bootloader_fb();
#if defined(CONFIG_LGE_BROADCAST_TDMB)
	star_dmb_init();
#endif /* CONFIG_LGE_BROADCAST */
}

int __init tegra_star_protected_aperture_init(void)
{
	tegra_protected_aperture_init(tegra_grhost_aperture);
	return 0;
}

void __init tegra_star_reserve(void)
{
	if (memblock_reserve(0x0, 4096) < 0)
		pr_warn("Cannot reserve first 4K of memory for safety\n");

	tegra_reserve((SZ_128M | SZ_16M | SZ_8M), SZ_3M, SZ_1M);
	tegra_ram_console_debug_reserve(SZ_1M);
}

MACHINE_START(STAR, "star")
.boot_params    = 0x00000100,
	.map_io         = tegra_map_common_io,
	.reserve        = tegra_star_reserve,
	.init_early	= tegra_init_early,
	.init_irq       = tegra_init_irq,
	.timer          = &tegra_timer,
	.init_machine   = tegra_star_init,
	MACHINE_END
