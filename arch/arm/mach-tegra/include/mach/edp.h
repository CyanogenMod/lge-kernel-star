/*
 * arch/arm/mach-tegra/include/mach/edp.h
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

#ifndef __MACH_EDP_H
#define __MACH_EDP_H

#include <linux/debugfs.h>

struct tegra_edp_entry {
	char speedo_id;
	char regulator_100mA;
	char temperature;
	char freq_limits[4];
};

struct tegra_edp_limits {
	int temperature;
	unsigned int freq_limits[4];
};

struct system_edp_entry {
	char speedo_id;
	char power_limit_100mW;
	char freq_limits[4];
};

#ifdef CONFIG_TEGRA_EDP_LIMITS


int tegra_edp_update_thermal_zone(int temperature);
void tegra_init_cpu_edp_limits(unsigned int regulator_mA);
void tegra_init_system_edp_limits(unsigned int power_limit_mW);
void tegra_get_cpu_edp_limits(const struct tegra_edp_limits **limits, int *size);
unsigned int tegra_get_edp_limit(void);
void tegra_get_system_edp_limits(const unsigned int **limits);
int tegra_system_edp_alarm(bool alarm);

#else
static inline void tegra_init_cpu_edp_limits(int regulator_mA)
{}
static inline void tegra_init_system_edp_limits(int power_limit_mW)
{}
static inline int tegra_edp_update_thermal_zone(int temperature)
{ return -1; }
static inline void tegra_get_cpu_edp_limits(struct tegra_edp_limits **limits,
					    int *size)
{}
static inline unsigned int tegra_get_edp_limit(void)
{ return -1; }
static inline void tegra_get_system_edp_limits(unsigned int **limits)
{}
static inline int tegra_system_edp_alarm(bool alarm)
{ return -1; }
#endif

#ifdef CONFIG_ARCH_TEGRA_2x_SOC
static inline void tegra_edp_throttle_cpu_now(u8 factor)
{}
#else
void tegra_edp_throttle_cpu_now(u8 factor);
#endif

#endif	/* __MACH_EDP_H */
