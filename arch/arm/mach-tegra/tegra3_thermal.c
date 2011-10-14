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

struct tegra_thermal {
	struct tegra_thermal_data data;
	struct tegra_thermal_device *device;
#ifdef CONFIG_TEGRA_THERMAL_SYSFS
	struct thermal_zone_device *thz;
#endif
#ifdef CONFIG_TEGRA_EDP_LIMITS
	int edp_thermal_zone_val;
#endif
};

static struct tegra_thermal thermal_state = {
	.device = NULL,
#ifdef CONFIG_TEGRA_EDP_LIMITS
	.edp_thermal_zone_val = -1,
#endif
};

#ifndef CONFIG_TEGRA_THERMAL_SYSFS
static bool throttle_enb;
struct mutex mutex;
#endif

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
	thermal->device->get_temp(thermal->device_client, temp);

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

	*temp = thermal->data.temp_throttle +
		thermal->data.temp_offset -
		thermal->device->offset;

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

void tegra_thermal_alert(void *data)
{
	struct tegra_thermal *thermal = data;
	int err;
	long temp;
	long lo_limit_throttle, hi_limit_throttle;
	long lo_limit_edp = 0, hi_limit_edp = 0;
	long tj_temp, tj_throttle_temp, tj_shutdown_temp;
	int lo_limit_tj = 0, hi_limit_tj = 0;
	int lo_limit = 0, hi_limit = 0;
	const struct tegra_edp_limits *z;
	int zones_sz;
	int i;


	if (thermal != &thermal_state)
		BUG();

#ifdef CONFIG_TEGRA_THERMAL_SYSFS
	if (thermal->thz) {
		if (!thermal->thz->passive)
			thermal_zone_device_update(thermal->thz);
	}
#endif

	err = thermal->device->get_temp(thermal->device->data, &temp);
	if (err) {
		pr_err("%s: get temp fail(%d)", __func__, err);
		return;
	}

	tj_temp = temp + thermal->device->offset;
	tj_throttle_temp = thermal->data.temp_throttle
				+ thermal->data.temp_offset;
	tj_shutdown_temp = thermal->data.temp_shutdown
				+ thermal->data.temp_offset;

	if ((tegra_is_throttling() &&
		(tj_temp >
			(tj_throttle_temp - thermal->data.hysteresis_throttle)))
		|| (tj_temp >= tj_throttle_temp)) {
		lo_limit_throttle = tj_throttle_temp -
					thermal->data.hysteresis_throttle;
		hi_limit_throttle = tj_shutdown_temp;
	} else {
		lo_limit_throttle = 0;
		hi_limit_throttle = tj_throttle_temp;
	}

#ifdef CONFIG_TEGRA_EDP_LIMITS
	tegra_get_cpu_edp_limits(&z, &zones_sz);

/* edp table based off of tdiode measurements */
#define EDP_TEMP(_index)	((z[_index].temperature * 1000)\
				+ thermal->data.edp_offset)
	if (tj_temp < EDP_TEMP(0)) {
		lo_limit_edp = 0;
		hi_limit_edp = EDP_TEMP(0);
	} else if (tj_temp >= EDP_TEMP(zones_sz-1)) {
		lo_limit_edp = EDP_TEMP(zones_sz-1) -
				thermal->data.hysteresis_edp;
		hi_limit_edp = tj_shutdown_temp;
	} else {
		for (i = 0; (i + 1) < zones_sz; i++) {
			if ((tj_temp >= EDP_TEMP(i)) &&
				(tj_temp < EDP_TEMP(i+1))) {
				lo_limit_edp = EDP_TEMP(i) -
						thermal->data.hysteresis_edp;
				hi_limit_edp = EDP_TEMP(i+1);
				break;
			}
		}
	}
#undef EDP_TEMP
#else
	lo_limit_edp = 0;
	hi_limit_edp = tj_shutdown_temp;
#endif

	/* Get smallest window size */
	lo_limit_tj = max(lo_limit_throttle, lo_limit_edp);
	hi_limit_tj = min(hi_limit_throttle, hi_limit_edp);

	/* Get corresponding device temps */
	lo_limit = lo_limit_tj ? (lo_limit_tj - thermal->device->offset) : 0;
	hi_limit = hi_limit_tj ? (hi_limit_tj - thermal->device->offset) : 0;

	thermal->device->set_limits(thermal->device->data, lo_limit, hi_limit);

#ifndef CONFIG_TEGRA_THERMAL_SYSFS
	if (tj_temp >= tj_throttle_temp) {
		/* start throttling */
		if (!tegra_is_throttling())
			tegra_therm_throttle(true);
	} else if (tj_temp <=
			(tj_throttle_temp -
			thermal->data.hysteresis_throttle)) {
		/* switch off throttling */
		if (tegra_is_throttling())
			tegra_therm_throttle(false);
	}
#endif

#ifdef CONFIG_TEGRA_EDP_LIMITS
	/* inform edp governor */
	if (thermal->edp_thermal_zone_val != tj_temp)
		tegra_edp_update_thermal_zone(
			(tj_temp - thermal->data.edp_offset)/1000);

	thermal->edp_thermal_zone_val = tj_temp;
#endif
}

int tegra_thermal_set_device(struct tegra_thermal_device *device)
{
#ifdef CONFIG_THERMAL_SYSFS
	struct thermal_zone_device *thz;
#endif

	/* only support one device */
	if (thermal_state.device)
		return -EINVAL;

	thermal_state.device = device;

#ifdef CONFIG_TEGRA_THERMAL_SYSFS
	thz = thermal_zone_device_register("thermal",
					1, /* trips */
					&thermal_state,
					&tegra_thermal_zone_ops,
					2, /* tc1 */
					1, /* tc2 */
					2000, /* passive delay */
					0); /* polling delay */

	if (IS_ERR(thz)) {
		thz = NULL;
		kfree(thermal);
		return -ENODEV;
	}

	thermal_state.thz = thz;
#endif

	thermal_state.device->set_alert(thermal_state.device->data,
					tegra_thermal_alert,
					&thermal_state);
	thermal_state.device->set_shutdown_temp(thermal_state.device->data,
					thermal_state.data.temp_shutdown +
					thermal_state.data.temp_offset -
					thermal_state.device->offset);
	/* initialize limits */
	tegra_thermal_alert(&thermal_state);

	return 0;
}

int __init tegra_thermal_init(struct tegra_thermal_data *data)
{
#ifndef CONFIG_TEGRA_THERMAL_SYSFS
	mutex_init(&mutex);
#endif

	memcpy(&thermal_state.data, data, sizeof(struct tegra_thermal_data));

	return 0;
}

int tegra_thermal_exit(void)
{
#ifdef CONFIG_TEGRA_THERMAL_SYSFS
	if (thermal->thz)
		thermal_zone_device_unregister(thermal->thz);
#endif

	return 0;
}
