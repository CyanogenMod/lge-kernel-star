/*
 * arch/arm/mach-tegra/cpu-tegra.h
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

#ifndef __MACH_TEGRA_CPU_TEGRA_H
#define __MACH_TEGRA_CPU_TEGRA_H

int tegra_cpu_set_speed_cap(unsigned int *speed_cap);
unsigned int tegra_count_slow_cpus(unsigned long speed_limit);
unsigned int tegra_get_slowest_cpu_n(void);
unsigned long tegra_cpu_lowest_speed(void);
unsigned long tegra_cpu_highest_speed(void);

#ifdef CONFIG_TEGRA_THERMAL_THROTTLE
void tegra_throttling_enable(bool enable);
#else
#define tegra_throttling_enable NULL
#endif /* CONFIG_TEGRA_THERMAL_THROTTLE */

#if defined(CONFIG_TEGRA_AUTO_HOTPLUG) && !defined(CONFIG_ARCH_TEGRA_2x_SOC)
int tegra_auto_hotplug_init(struct mutex *cpu_lock);
void tegra_auto_hotplug_exit(void);
void tegra_auto_hotplug_governor(unsigned int cpu_freq, bool suspend);
#else
static inline int tegra_auto_hotplug_init(struct mutex *cpu_lock)
{ return 0; }
static inline void tegra_auto_hotplug_exit(void)
{ }
static inline void tegra_auto_hotplug_governor(unsigned int cpu_freq,
					       bool suspend)
{ }
#endif

#endif /* __MACH_TEGRA_CPU_TEGRA_H */
