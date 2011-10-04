/*
 * arch/arm/mach-tegra/tegra3_thermal.c
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

#include <linux/kernel.h>
#include <linux/cpufreq.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include <linux/thermal.h>
#include <mach/thermal.h>
#include <mach/edp.h>
#include <linux/slab.h>


#include "clock.h"
#include "cpu-tegra.h"
#include "dvfs.h"

#define MAX_ZONES (16)

/* Thermal sysfs handles hysteresis */
#ifndef CONFIG_TEGRA_THERMAL_SYSFS
#define ALERT_HYSTERESIS_THROTTLE	1
#endif

#define ALERT_HYSTERESIS_EDP	3

#define THROTTLING_LIMIT	(85000)
#define MAX_LIMIT		(90000)

u8 thermal_zones[MAX_ZONES];
int thermal_zones_sz;
static int edp_thermal_zone_val = -1;

#ifndef CONFIG_TEGRA_THERMAL_SYSFS
static bool throttle_enb;
struct mutex mutex;
#endif


int __init tegra_thermal_init()
{
	const struct tegra_edp_limits *z;
	int zones_sz;
	int i;

#ifndef CONFIG_TEGRA_THERMAL_SYSFS
	mutex_init(&mutex);
#endif
	tegra_get_cpu_edp_limits(&z, &zones_sz);
	zones_sz = min(zones_sz, MAX_ZONES);

	for (i = 0; i < zones_sz; i++)
		thermal_zones[i] = z[i].temperature;

	thermal_zones_sz = zones_sz;

	return 0;
}


#ifdef CONFIG_TEGRA_THERMAL_SYSFS

static int tegra_thermal_zone_bind(struct thermal_zone_device *thermal,
				struct thermal_cooling_device *cdevice) {
	/* Support only Thermal Throttling (1 trip) for now */
	return thermal_zone_bind_cooling_device(thermal, 0, cdevice);
}

static int tegra_thermal_zone_unbind(struct thermal_zone_device *thermal,
				struct thermal_cooling_device *cdevice) {
	/* Support only Thermal Throttling (1 trip) for now */
	return thermal_zone_unbind_cooling_device(thermal, 0, cdevice);
}

static int tegra_thermal_zone_get_temp(struct thermal_zone_device *thz,
						long *temp)
{
	struct tegra_thermal *thermal = thz->devdata;
	thermal->ops->get_temp(thermal->data, temp);

	return 0;
}

static int tegra_thermal_zone_get_trip_type(
			struct thermal_zone_device *thermal,
			int trip,
			enum thermal_trip_type *type) {

	/* Support only Thermal Throttling (1 trip) for now */
	if (trip != 0)
		return -EINVAL;

	*type = THERMAL_TRIP_PASSIVE;

	return 0;
}

static int tegra_thermal_zone_get_trip_temp(struct thermal_zone_device *thermal,
						int trip,
						long *temp) {
	/* Support only Thermal Throttling (1 trip) for now */
	if (trip != 0)
		return -EINVAL;

	*temp = THROTTLING_LIMIT;

	return 0;
}

static struct thermal_zone_device_ops tegra_thermal_zone_ops = {
	.bind = tegra_thermal_zone_bind,
	.unbind = tegra_thermal_zone_unbind,
	.get_temp = tegra_thermal_zone_get_temp,
	.get_trip_type = tegra_thermal_zone_get_trip_type,
	.get_trip_temp = tegra_thermal_zone_get_trip_temp,
};
#endif



struct tegra_thermal
*tegra_thermal_register(void *data, struct tegra_thermal_ops *thermal_ops)
{
	long temp_milli;
	struct tegra_thermal *thermal;
#ifdef CONFIG_THERMAL_SYSFS
	struct thermal_zone_device *thz;
#endif

	thermal = kzalloc(sizeof(struct tegra_thermal), GFP_KERNEL);
	if (!thermal)
		return ERR_PTR(-ENOMEM);

	thermal->ops = thermal_ops;
	thermal->data = data;

#ifdef CONFIG_TEGRA_THERMAL_SYSFS
	thz = thermal_zone_device_register("nct1008",
					1, /* trips */
					thermal,
					&tegra_thermal_zone_ops,
					2, /* tc1 */
					1, /* tc2 */
					2000, /* passive delay */
					0); /* polling delay */

	if (IS_ERR(thz)) {
		thz = NULL;
		kfree(thermal);
		return ERR_PTR(-ENODEV);
	}

	thermal->thz = thz;
#endif

	thermal->ops->get_temp(thermal->data, &temp_milli);
	tegra_edp_update_thermal_zone(MILLICELSIUS_TO_CELSIUS(temp_milli));

	return thermal;
}

int tegra_thermal_unregister(struct tegra_thermal *thermal)
{
#ifdef CONFIG_TEGRA_THERMAL_SYSFS
	if (thermal->thz)
		thermal_zone_device_unregister(thermal->thz);
#endif

	kfree(thermal);

	return 0;
}

/* The thermal sysfs handles notifying the throttling
 * cooling device */
#ifndef CONFIG_TEGRA_THERMAL_SYSFS
static void tegra_therm_throttle(bool enable)
{
	if (throttle_enb != enable) {
		mutex_lock(&mutex);
		tegra_throttling_enable(enable);
		throttle_enb = enable;
		mutex_unlock(&mutex);
	}
}
#endif

int tegra_thermal_alert(struct tegra_thermal *thermal)
{
	int err;
	int hysteresis;
	long temp, tzone1, tzone2;
	int lo_limit = 0, hi_limit = 0;
	int nentries = thermal_zones_sz;
	int i;

	err = thermal->ops->get_temp(thermal->data, &temp);
	if (err) {
		pr_err("%s: get temp fail(%d)", __func__, err);
		return err;
	}

	hysteresis = ALERT_HYSTERESIS_EDP;

#ifndef CONFIG_TEGRA_THERMAL_SYSFS
	if (temp >= THROTTLING_LIMIT) {
		/* start throttling */
		tegra_therm_throttle(true);
		hysteresis = ALERT_HYSTERESIS_THROTTLE;
	} else if (temp <=
			(THROTTLING_LIMIT -
			ALERT_HYSTERESIS_THROTTLE)) {
		/* switch off throttling */
		tegra_therm_throttle(false);
	}
#endif

	if (temp < CELSIUS_TO_MILLICELSIUS(thermal_zones[0])) {
		lo_limit = 0;
		hi_limit = thermal_zones[0];
	} else if (temp >=
			CELSIUS_TO_MILLICELSIUS(thermal_zones[nentries-1])) {
		lo_limit = thermal_zones[nentries-1] - hysteresis;
		hi_limit = MILLICELSIUS_TO_CELSIUS(MAX_LIMIT);
	} else {
		for (i = 0; (i + 1) < nentries; i++) {
			tzone1 = thermal_zones[i];
			tzone2 = thermal_zones[i + 1];

			if (temp >= CELSIUS_TO_MILLICELSIUS(tzone1) &&
			    temp < CELSIUS_TO_MILLICELSIUS(tzone2)) {
				lo_limit = tzone1 - hysteresis;
				hi_limit = tzone2;
				break;
			}
		}
	}

	err = thermal->ops->set_limits(thermal->data, lo_limit, hi_limit);

	if (err)
		return err;

	/* inform edp governor */
	if (edp_thermal_zone_val != temp)
		/*
		 * FIXME: Move this direct tegra_ function call to be called
		 * via a pointer in 'struct nct1008_data' (like 'alarm_fn')
		 */
		tegra_edp_update_thermal_zone(MILLICELSIUS_TO_CELSIUS(temp));

	edp_thermal_zone_val = temp;

#ifdef CONFIG_TEGRA_THERMAL_SYSFS
	if (thermal->thz) {
		if (!thermal->thz->passive)
			thermal_zone_device_update(thermal->thz);
	}
#endif

	return 0;
}

int tegra_thermal_exit(void)
{
	return 0;
}
