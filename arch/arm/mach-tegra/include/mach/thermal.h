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

struct tegra_thermal_ops {
	int (*get_temp) (void *, long *);
	int (*set_limits) (void *, long, long);
};

struct tegra_thermal {
	void *data;
	struct tegra_thermal_ops *ops;
#ifdef CONFIG_TEGRA_THERMAL_SYSFS
	struct thermal_zone_device *thz;
#endif
};

#ifndef CONFIG_ARCH_TEGRA_2x_SOC
int tegra_thermal_init(void);
int tegra_thermal_exit(void);

struct tegra_thermal
	*tegra_thermal_register(void *data, struct tegra_thermal_ops *ops);
int tegra_thermal_unregister(struct tegra_thermal *thermal);
int tegra_thermal_alert(struct tegra_thermal *thermal);
#else
static inline int tegra_thermal_init(void)
{ return 0; }
static inline int tegra_thermal_exit(void)
{ return 0; }
static inline struct tegra_thermal
	*tegra_thermal_register(void *data, struct tegra_thermal_ops *ops)
{ return NULL; }
static inline int tegra_thermal_unregister(struct tegra_thermal *thermal)
{ return 0; }
static inline int tegra_thermal_alert(struct tegra_thermal *thermal)
{ return 0; }
#endif

#define CELSIUS_TO_MILLICELSIUS(x) ((x)*1000)
#define MILLICELSIUS_TO_CELSIUS(x) ((x)/1000)



#endif	/* __MACH_THERMAL_H */
