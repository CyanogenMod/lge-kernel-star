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

struct tegra_tsensor_pmu_data {
	u8 poweroff_reg_data;
	u8 poweroff_reg_addr;
	u8 reset_tegra;
	u8 controller_type;
	u8 i2c_controller_id;
	u8 pinmux;
	u8 pmu_16bit_ops;
	u8 pmu_i2c_addr;
};

struct tegra_tsensor_data;

struct tegra_tsensor_platform_data {
	void (*probe_callback)(struct tegra_tsensor_data *);
};

void __init tegra3_tsensor_init(struct tegra_tsensor_pmu_data *);

int tsensor_thermal_get_temp(struct tegra_tsensor_data *data,
				long *milli_temp);
int tsensor_thermal_get_temp_low(struct tegra_tsensor_data *data,
					long *milli_temp);
int tsensor_thermal_set_limits(struct tegra_tsensor_data *data,
				long lo_limit_milli,
				long hi_limit_milli);
int tsensor_thermal_set_alert(struct tegra_tsensor_data *data,
				void (*alert_func)(void *),
				void *alert_data);
int tsensor_thermal_set_shutdown_temp(struct tegra_tsensor_data *data,
				long shutdown_temp_milli);

#endif /* __MACH_TEGRA_TSENSOR_H */

