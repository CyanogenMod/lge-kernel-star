/*
 * arch/arm/mach-tegra/include/mach/tsensor.h
 *
 * Tegra tsensor header file
 *
 * Copyright (c) 2011, NVIDIA Corporation.
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

#ifndef __MACH_TEGRA_TSENSOR_H
#define __MACH_TEGRA_TSENSOR_H

#include <linux/types.h>

#include <mach/edp.h>

#define MAX_ZONES	16

struct tegra_tsensor_platform_data {
	int sw_intr_temperature;
	int hw_clk_div_temperature;
	int hw_reset_temperature;
	int hysteresis;
	u8 throttling_ext_limit;
	u8 thermal_zones[MAX_ZONES];
	u8 thermal_zones_sz;
	void (*alarm_fn)(bool raised);
};

#endif /* __MACH_TEGRA_TSENSOR_H */

