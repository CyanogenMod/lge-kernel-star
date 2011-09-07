/*
 * arch/arm/mach-tegra/mc.c
 *
 * Copyright (C) 2010 Google, Inc.
 * Copyright (C) 2011 NVIDIA Corporation
 *
 * Author:
 *	Erik Gilling <konkers@google.com>
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

#include <linux/io.h>
#include <linux/spinlock.h>

#include <mach/iomap.h>
#include <mach/mc.h>

#if defined(CONFIG_ARCH_TEGRA_2x_SOC)
static DEFINE_SPINLOCK(tegra_mc_lock);

void tegra_mc_set_priority(unsigned long client, unsigned long prio)
{
	unsigned long mc_base = IO_TO_VIRT(TEGRA_MC_BASE);
	unsigned long reg = client >> 8;
	int field = client & 0xff;
	unsigned long val;
	unsigned long flags;

	spin_lock_irqsave(&tegra_mc_lock, flags);
	val = readl(mc_base + reg);
	val &= ~(TEGRA_MC_PRIO_MASK << field);
	val |= prio << field;
	writel(val, mc_base + reg);
	spin_unlock_irqrestore(&tegra_mc_lock, flags);

}

int tegra_mc_get_tiled_memory_bandwidth_multiplier(void)
{
	return 1;
}

#else
	/* !!!FIXME!!! IMPLEMENT tegra_mc_set_priority() */

#include "tegra3_emc.h"

/*
 * If using T30/DDR3, the 2nd 16 bytes part of DDR3 atom is 2nd line and is
 * discarded in tiling mode.
 */
int tegra_mc_get_tiled_memory_bandwidth_multiplier(void)
{
	int type;

	type = tegra_emc_get_dram_type();
	WARN_ONCE(type == -1, "unknown type DRAM because DVFS is disabled\n");

	if (type == DRAM_TYPE_DDR3)
		return 2;
	else
		return 1;
}
#endif
