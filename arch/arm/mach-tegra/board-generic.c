/*
 * arch/arm/mach-tegra/board-harmony.c
 *
 * Copyright (C) 2010 Google, Inc.
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

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>
#include <asm/setup.h>

#include <mach/iomap.h>
#include <mach/irqs.h>

#include <mach/kbc.h>

#include "board.h"

#if defined(CONFIG_TEGRA_DEBUG_UARTA)
#define DEBUG_UART_BASE TEGRA_UARTA_BASE
#define DEBUG_UART_INT INT_UARTA
#define DEBUG_UART_CLK "uart.0"
#elif defined(CONFIG_TEGRA_DEBUG_UARTB)
#define DEBUG_UART_BASE TEGRA_UARTB_BASE
#define DEBUG_UART_INT INT_UARTB
#define DEBUG_UART_CLK "uart.1"
#elif defined(CONFIG_TEGRA_DEBUG_UARTC)
#define DEBUG_UART_BASE TEGRA_UARTC_BASE
#define DEBUG_UART_INT INT_UARTC
#define DEBUG_UART_CLK "uart.2"
#elif defined(CONFIG_TEGRA_DEBUG_UARTD)
#define DEBUG_UART_BASE TEGRA_UARTD_BASE
#define DEBUG_UART_INT INT_UARTD
#define DEBUG_UART_CLK "uart.3"
#elif defined(CONFIG_TEGRA_DEBUG_UARTE)
#define DEBUG_UART_BASE TEGRA_UARTE_BASE
#define DEBUG_UART_INT INT_UARTE
#define DEBUG_UART_CLK "uart.4"
#else
#define DEBUG_UART_BASE NULL
#define DEBUG_UART_INT NO_IRQ
#define DEBUG_UART_CLK ""
#endif

static struct plat_serial8250_port debug_uart_platform_data[] = {
	{
		.membase	= IO_ADDRESS(DEBUG_UART_BASE),
		.mapbase	= DEBUG_UART_BASE,
		.irq		= DEBUG_UART_INT,
		.flags		= UPF_BOOT_AUTOCONF,
		.iotype		= UPIO_MEM,
		.regshift	= 2,
		.uartclk	= 216000000/16 * 16,
	}, {
		.flags		= 0
	}
};

static struct platform_device debug_uart = {
	.name = "serial8250",
	.id = PLAT8250_DEV_PLATFORM,
	.dev = {
		.platform_data = debug_uart_platform_data,
	},
};

#ifdef CONFIG_KEYBOARD_TEGRA
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
#define kbc_row(x)				\
	{					\
		.is_row = true,			\
		.is_col = false,		\
		.num = (x),			\
	}
#define kbc_col(x)				\
	{					\
		.is_row = false,		\
		.is_col = true,			\
		.num = (x),			\
	}


static struct tegra_kbc_plat harmony_kbc = {
	.debounce_cnt = 2,
	.repeat_cnt = 5,
	.wake_cnt = 0,
	.pin_cfg = {
		[0] = kbc_row(0), [1] = kbc_row(1),
		[2] = kbc_row(2), [3] = kbc_row(3),
		[4] = kbc_row(4), [5] = kbc_row(5),
		[6] = kbc_row(6), [7] = kbc_row(7),
		[8] = kbc_row(8), [9] = kbc_row(9),
		[10] = kbc_row(10), [11] = kbc_row(11),
		[12] = kbc_row(12), [13] = kbc_row(13),
		[14] = kbc_row(14), [15] = kbc_row(15),
		[16] = kbc_col(0), [17] = kbc_col(1),
		[18] = kbc_col(2), [19] = kbc_col(3),
		[20] = kbc_col(4), [21] = kbc_col(5),
		[22] = kbc_col(6), [23] = kbc_col(7),
	},
	.keymap = NULL,
	.wake_cfg = NULL,
};
static struct platform_device tegra_kbc = {
	.name = "tegra-kbc",
	.id = -1,
	.dev = {
		.platform_data = &harmony_kbc,
	},
	.resource = tegra_kbc_resources,
	.num_resources = ARRAY_SIZE(tegra_kbc_resources),
};
#endif

static struct platform_device *harmony_devices[] __initdata = {
#if !defined(CONFIG_TEGRA_DEBUG_UART_NONE)
	&debug_uart,
#endif
#ifdef CONFIG_KEYBOARD_TEGRA
	&tegra_kbc,
#endif
};

static void __init tegra_generic_init(void)
{
	struct clk *clk;

	tegra_common_init();

	clk = clk_get_sys(NULL, "pll_p");
	clk_enable(clk);
	clk_set_rate(clk, 216000000);
	clk_put(clk);

#if !defined(CONFIG_TEGRA_DEBUG_UART_NONE)
	clk = clk_get_sys(DEBUG_UART_CLK, NULL);
	clk_set_rate(clk, 216000000);
	clk_enable(clk);
	clk_put(clk);
#endif

	platform_add_devices(harmony_devices, ARRAY_SIZE(harmony_devices));
}

MACHINE_START(TEGRA_GENERIC, "Tegra Generic")
	.boot_params  = 0x00000100,
	.phys_io        = IO_APB_PHYS,
	.io_pg_offst    = ((IO_APB_VIRT) >> 18) & 0xfffc,
	.init_irq       = tegra_init_irq,
	.init_machine   = tegra_generic_init,
	.map_io         = tegra_map_common_io,
	.timer          = &tegra_timer,
MACHINE_END
