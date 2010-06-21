/*
 * arch/arm/mach-tegra/board-harmony.c
 *
 * Copyright (C) 2010 Google, Inc.
 *
 * Author:
 *	Colin Cross <ccross@android.com>
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

#include <linux/init.h>
#include <linux/io.h>
#include <linux/cpu.h>

#include <asm/hardware/cache-l2x0.h>
#include <asm/cacheflush.h>
#include <asm/outercache.h>

#include <mach/iomap.h>
#include <mach/dma.h>
#include <mach/nvmem.h>

#include "board.h"

#ifdef CONFIG_DMABOUNCE
int dma_needs_bounce(struct device *dev, dma_addr_t addr, size_t size)
{
	return 0;
}
#endif

static void tegra_machine_restart(char mode, const char *cmd)
{
	disable_nonboot_cpus();
	flush_cache_all();
	outer_shutdown();
	arm_machine_restart(mode, cmd);
}

void __init tegra_init_cache(void)
{
#ifdef CONFIG_CACHE_L2X0
	void __iomem *p = IO_ADDRESS(TEGRA_ARM_PERIF_BASE) + 0x3000;

	writel(0x331, p + L2X0_TAG_LATENCY_CTRL);
	writel(0x441, p + L2X0_DATA_LATENCY_CTRL);
	writel(7, p + L2X0_PREFETCH_OFFSET);

	l2x0_init(p, 0x7C080001, 0x8200c3fe);
#endif
}

void __init tegra_common_init(void)
{
#ifdef CONFIG_CPU_V7
	/* enable dynamic clock gating */
	unsigned int reg;
	asm volatile ("mrc p15, 0, %0, c15, c0, 0" : "=r" (reg) : : "cc");
	reg |= 1;
	asm volatile ("mcr p15, 0, %0, c15, c0, 0" : : "r" (reg) : "cc");
#endif

	nvmap_add_carveout_heap(TEGRA_IRAM_BASE, TEGRA_IRAM_SIZE,
				"iram", NVMEM_HEAP_CARVEOUT_IRAM);
	tegra_init_clock();
	tegra_init_cache();
	tegra_dma_init();
	tegra_mc_init();
	arm_pm_restart = tegra_machine_restart;
}
