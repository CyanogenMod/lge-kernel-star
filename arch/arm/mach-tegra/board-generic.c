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

#include "board.h"

/* NVidia bootloader tags */
#define ATAG_NVIDIA		0x41000801

#define ATAG_NVIDIA_RM			0x1
#define ATAG_NVIDIA_DISPLAY		0x2
#define ATAG_NVIDIA_FRAMEBUFFER		0x3
#define ATAG_NVIDIA_CHIPSHMOO		0x4
#define ATAG_NVIDIA_CHIPSHMOOPHYS	0x5
#define ATAG_NVIDIA_PRESERVED_MEM_0	0x10000
#define ATAG_NVIDIA_PRESERVED_MEM_N	2
#define ATAG_NVIDIA_FORCE_32		0x7fffffff

struct tag_tegra {
	__u32 bootarg_key;
	__u32 bootarg_len;
	char bootarg[1];
};

static int __init parse_tag_nvidia(const struct tag *tag)
{

	return 0;
}
__tagtable(ATAG_NVIDIA, parse_tag_nvidia);


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

static struct platform_device *harmony_devices[] __initdata = {
#if !defined(CONFIG_TEGRA_DEBUG_UART_NONE)
	&debug_uart,
#endif
};

static void __init tegra_generic_init(void)
{
	struct clk *clk;

	tegra_common_init();

	clk = clk_get_sys(NULL, "pll_p");
	clk_enable(clk);
	clk_set_rate(clk, 216000000);

#if !defined(CONFIG_TEGRA_DEBUG_UART_NONE)
	clk = clk_get_sys(DEBUG_UART_CLK, NULL);
	clk_set_rate(clk, 216000000);
	clk_enable(clk);
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
