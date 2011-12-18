/*
 * arch/arm/mach-tegra/thermal.h
 *
 * Copyright (C) 2010-2011 NVIDIA Corporation.
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

#ifndef __MACH_THERMAL_H
#define __MACH_THERMAL_H

/* All units in millicelsius */
struct tegra_thermal_data {
	long temp_throttle;
	long temp_shutdown;
	long temp_offset;
#ifdef CONFIG_TEGRA_EDP_LIMITS
	long edp_offset;
	long hysteresis_edp;
#endif
#ifdef CONFIG_TEGRA_THERMAL_SYSFS
	int tc1;
	int tc2;
	long passive_delay;
#else
	long hysteresis_throttle;
#endif
};

struct tegra_thermal_device {
	char *name;
	void *data;
	long offset;
	int (*get_temp) (void *, long *);
	int (*get_temp_low)(void *, long *);
	int (*set_limits) (void *, long, long);
	int (*set_alert)(void *, void (*)(void *), void *);
	int (*set_shutdown_temp)(void *, long);
};

#ifndef CONFIG_ARCH_TEGRA_2x_SOC
int tegra_thermal_init(struct tegra_thermal_data *data);
int tegra_thermal_set_device(struct tegra_thermal_device *device);
int tegra_thermal_exit(void);
#else
static inline int tegra_thermal_init(struct tegra_thermal_data *data)
{ return 0; }
static inline int tegra_thermal_set_device(struct tegra_thermal_device *dev)
{ return 0; }
static inline int tegra_thermal_exit(void)
{ return 0; }
#endif

#endif	/* __MACH_THERMAL_H */
