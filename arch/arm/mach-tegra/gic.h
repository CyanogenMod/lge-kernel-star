/*
 * arch/arm/mach-tegra/include/mach/gic.h
 *
 * Copyright (C) 2010-2011 NVIDIA Corporation
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

#ifndef _MACH_TEGRA_GIC_H_
#define _MACH_TEGRA_GIC_H_

#if defined(CONFIG_HOTPLUG_CPU) || defined(CONFIG_PM_SLEEP)

void tegra_gic_cpu_disable(void);
void tegra_gic_cpu_enable(void);

#ifndef CONFIG_ARCH_TEGRA_2x_SOC

void tegra_gic_pass_through_disable(void);

#endif
#endif


#if defined(CONFIG_PM_SLEEP)

int tegra_gic_pending_interrupt(void);

#ifndef CONFIG_ARCH_TEGRA_2x_SOC

void tegra_gic_dist_disable(void);
void tegra_gic_dist_enable(void);

void tegra_gic_disable_affinity(void);
void tegra_gic_restore_affinity(void);
void tegra_gic_affinity_to_cpu0(void);

#endif
#endif

#endif /* _MACH_TEGRA_GIC_H_ */
