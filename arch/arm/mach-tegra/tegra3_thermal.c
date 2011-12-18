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
	struct tegra_thermal_device *device;
	long temp_throttle_tj;
	long temp_shutdown_tj;
#ifdef CONFIG_TEGRA_THERMAL_SYSFS
	struct thermal_zone_device *thz;
	int tc1;
	int tc2;
	long passive_delay;
#else
	long temp_throttle_low_tj;
#endif
#ifdef CONFIG_TEGRA_EDP_LIMITS
	int edp_thermal_zone_val;
	long edp_offset;
	long hysteresis_edp;
#endif
	struct mutex mutex;
};

static struct tegra_thermal thermal_state = {
	.device = NULL,
#ifdef CONFIG_TEGRA_EDP_LIMITS
	.edp_thermal_zone_val = -1,
#endif
};

#ifndef CONFIG_TEGRA_THERMAL_SYSFS
static bool throttle_enb;
#endif

#ifdef CONFIG_TEGRA_EDP_LIMITS
static inline long edp2tj(struct tegra_thermal *thermal,
				long edp_temp)
{
	return edp_temp + thermal->edp_offset;
}

static inline long tj2edp(struct tegra_thermal *thermal,
				long temp_tj)
{
	return temp_tj - thermal->edp_offset;
}
#endif

static inline long dev2tj(struct tegra_thermal_device *dev,
				long dev_temp)
{
	return dev_temp + dev->offset;
}

static inline long tj2dev(struct tegra_thermal_device *dev,
				long tj_temp)
{
	return tj_temp - dev->offset;
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
						unsigned long *temp)
{
	struct tegra_thermal *thermal = thz->devdata;
	thermal->device->get_temp(thermal->device->data, temp);

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

static int tegra_thermal_zone_get_trip_temp(struct thermal_zone_device *thz,
						int trip,
						unsigned long *temp) {
	struct tegra_thermal *thermal = thz->devdata;

	/* Support only Thermal Throttling (1 trip) for now */
	if (trip != 0)
		return -EINVAL;

	*temp = tj2dev(thermal->device, thermal->temp_throttle_tj);

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
		mutex_lock(&thermal_state.mutex);
		tegra_throttling_enable(enable);
		throttle_enb = enable;
		mutex_unlock(&thermal_state.mutex);
	}
}
#endif

/* Make sure this function remains stateless */
void tegra_thermal_alert(void *data)
{
	struct tegra_thermal *thermal = data;
	int err;
	long temp_dev, temp_tj;
	long lo_limit_throttle_tj, hi_limit_throttle_tj;
	long lo_limit_edp_tj = 0, hi_limit_edp_tj = 0;
	long temp_low_dev, temp_low_tj;
	int lo_limit_tj = 0, hi_limit_tj = 0;
#ifdef CONFIG_TEGRA_EDP_LIMITS
	const struct tegra_edp_limits *z;
	int zones_sz;
	int i;
#endif

	if (thermal != &thermal_state)
		BUG();

	mutex_lock(&thermal_state.mutex);

#ifdef CONFIG_TEGRA_THERMAL_SYSFS
	if (thermal->thz) {
		if (!thermal->thz->passive)
			thermal_zone_device_update(thermal->thz);
	}
#endif

	err = thermal->device->get_temp(thermal->device->data, &temp_dev);
	if (err) {
		pr_err("%s: get temp fail(%d)", __func__, err);
		goto done;
	}

	/* Convert all temps to tj and then do all work/logic in terms of
	   tj in order to avoid confusion */
	temp_tj = dev2tj(thermal->device, temp_dev);
	thermal->device->get_temp_low(thermal->device, &temp_low_dev);
	temp_low_tj = dev2tj(thermal->device, temp_low_dev);

	lo_limit_throttle_tj = temp_low_tj;
	hi_limit_throttle_tj = thermal->temp_throttle_tj;

#ifndef CONFIG_TEGRA_THERMAL_SYSFS
	/* Check to see if we are currently throttling */
	if ((tegra_is_throttling() &&
		(temp_tj > thermal->temp_throttle_low_tj))
		|| (temp_tj >= thermal->temp_throttle_tj)) {
		lo_limit_throttle_tj = thermal->temp_throttle_low_tj;
		hi_limit_throttle_tj = thermal->temp_shutdown_tj;
	}
#else
	if (temp_tj > thermal->temp_throttle_tj) {
		lo_limit_throttle_tj = thermal->temp_throttle_tj;
		hi_limit_throttle_tj = thermal->temp_shutdown_tj;
	}
#endif

#ifdef CONFIG_TEGRA_EDP_LIMITS
	tegra_get_cpu_edp_limits(&z, &zones_sz);

/* edp table based off of tdiode measurements */
#define EDP_TEMP_TJ(_index)	edp2tj(thermal, z[_index].temperature * 1000)

	if (temp_tj < EDP_TEMP_TJ(0)) {
		lo_limit_edp_tj = temp_low_tj;
		hi_limit_edp_tj = EDP_TEMP_TJ(0);
	} else if (temp_tj >= EDP_TEMP_TJ(zones_sz-1)) {
		lo_limit_edp_tj = EDP_TEMP_TJ(zones_sz-1) -
					thermal->hysteresis_edp;
		hi_limit_edp_tj = thermal->temp_shutdown_tj;
	} else {
		for (i = 0; (i + 1) < zones_sz; i++) {
			if ((temp_tj >= EDP_TEMP_TJ(i)) &&
				(temp_tj < EDP_TEMP_TJ(i+1))) {
				lo_limit_edp_tj = EDP_TEMP_TJ(i) -
							thermal->hysteresis_edp;
				hi_limit_edp_tj = EDP_TEMP_TJ(i+1);
				break;
			}
		}
	}
#undef EDP_TEMP_TJ
#else
	lo_limit_edp_tj = temp_low_tj;
	hi_limit_edp_tj = thermal->temp_shutdown_tj;
#endif

	/* Get smallest window size */
	lo_limit_tj = max(lo_limit_throttle_tj, lo_limit_edp_tj);
	hi_limit_tj = min(hi_limit_throttle_tj, hi_limit_edp_tj);

	thermal->device->set_limits(thermal->device->data,
					tj2dev(thermal->device, lo_limit_tj),
					tj2dev(thermal->device, hi_limit_tj));

#ifndef CONFIG_TEGRA_THERMAL_SYSFS
	if (temp_tj >= thermal->temp_throttle_tj) {
		/* start throttling */
		if (!tegra_is_throttling())
			tegra_therm_throttle(true);
	} else if (temp_tj <= thermal->temp_throttle_low_tj) {
		/* switch off throttling */
		if (tegra_is_throttling())
			tegra_therm_throttle(false);
	}
#endif

#ifdef CONFIG_TEGRA_EDP_LIMITS
	/* inform edp governor */
	if (thermal->edp_thermal_zone_val != temp_tj)
		tegra_edp_update_thermal_zone(tj2edp(thermal, temp_tj)/1000);

	thermal->edp_thermal_zone_val = temp_tj;
#endif

done:
	mutex_unlock(&thermal_state.mutex);
}

int tegra_thermal_set_device(struct tegra_thermal_device *device)
{
#ifdef CONFIG_TEGRA_THERMAL_SYSFS
	struct thermal_zone_device *thz;
#endif

	/* only support one device */
	if (thermal_state.device)
		return -EINVAL;

	thermal_state.device = device;

#ifdef CONFIG_TEGRA_THERMAL_SYSFS
	thz = thermal_zone_device_register(thermal_state.device->name,
					1, /* trips */
					&thermal_state,
					&tegra_thermal_zone_ops,
					thermal_state.tc1, /* dT/dt */
					thermal_state.tc2, /* throttle */
					thermal_state.passive_delay,
					0); /* polling delay */

	if (IS_ERR(thz)) {
		thz = NULL;
		return -ENODEV;
	}

	thermal_state.thz = thz;
#endif
	thermal_state.device->set_alert(thermal_state.device->data,
					tegra_thermal_alert,
					&thermal_state);

	thermal_state.device->set_shutdown_temp(thermal_state.device->data,
				tj2dev(device, thermal_state.temp_shutdown_tj));

	/* initialize limits */
	tegra_thermal_alert(&thermal_state);

	return 0;
}

int __init tegra_thermal_init(struct tegra_thermal_data *data)
{
#ifdef CONFIG_TEGRA_THERMAL_SYSFS
	thermal_state.tc1 = data->tc1;
	thermal_state.tc2 = data->tc2;
	thermal_state.passive_delay = data->passive_delay;
#else
	thermal_state.temp_throttle_low_tj = data->temp_throttle +
						data->temp_offset -
						data->hysteresis_throttle;
#endif
	mutex_init(&thermal_state.mutex);
#ifdef CONFIG_TEGRA_EDP_LIMITS
	thermal_state.edp_offset = data->edp_offset;
	thermal_state.hysteresis_edp = data->hysteresis_edp;
#endif
	thermal_state.temp_throttle_tj = data->temp_throttle +
						data->temp_offset;
	thermal_state.temp_shutdown_tj = data->temp_shutdown +
						data->temp_offset;

	return 0;
}

int tegra_thermal_exit(void)
{
#ifdef CONFIG_TEGRA_THERMAL_SYSFS
	if (thermal_state.thz)
		thermal_zone_device_unregister(thermal_state.thz);
#endif

	return 0;
}

#ifdef CONFIG_DEBUG_FS

static int tegra_thermal_throttle_temp_tj_set(void *data, u64 val)
{
#ifndef CONFIG_TEGRA_THERMAL_SYSFS
	long throttle_hysteresis = thermal_state.temp_throttle_tj -
					thermal_state.temp_throttle_low_tj;
#endif

	mutex_lock(&thermal_state.mutex);
	thermal_state.temp_throttle_tj = val;
#ifndef CONFIG_TEGRA_THERMAL_SYSFS
	thermal_state.temp_throttle_low_tj = thermal_state.temp_throttle_tj -
						throttle_hysteresis;
#endif
	mutex_unlock(&thermal_state.mutex);

	tegra_thermal_alert(&thermal_state);

	return 0;
}

static int tegra_thermal_throttle_temp_tj_get(void *data, u64 *val)
{
	*val = (u64)thermal_state.temp_throttle_tj;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(throttle_temp_tj_fops,
			tegra_thermal_throttle_temp_tj_get,
			tegra_thermal_throttle_temp_tj_set,
			"%llu\n");

static int tegra_thermal_shutdown_temp_tj_set(void *data, u64 val)
{
	thermal_state.temp_shutdown_tj = val;

	if (thermal_state.device)
		thermal_state.device->set_shutdown_temp(
				thermal_state.device->data,
				tj2dev(thermal_state.device,
					thermal_state.temp_shutdown_tj));

	tegra_thermal_alert(&thermal_state);

	return 0;
}

static int tegra_thermal_shutdown_temp_tj_get(void *data, u64 *val)
{
	*val = (u64)thermal_state.temp_shutdown_tj;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(shutdown_temp_tj_fops,
			tegra_thermal_shutdown_temp_tj_get,
			tegra_thermal_shutdown_temp_tj_set,
			"%llu\n");


static int tegra_thermal_temp_tj_get(void *data, u64 *val)
{
	long temp_tj, temp_dev;

	if (thermal_state.device) {
		thermal_state.device->get_temp(thermal_state.device->data,
						&temp_dev);

		/* Convert all temps to tj and then do all work/logic in
		   terms of tj in order to avoid confusion */
		temp_tj = dev2tj(thermal_state.device, temp_dev);
	} else {
		temp_tj = -1;
	}

	*val = (u64)temp_tj;

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(temp_tj_fops,
			tegra_thermal_temp_tj_get,
			NULL,
			"%llu\n");

#ifdef CONFIG_TEGRA_THERMAL_SYSFS
static int tegra_thermal_tc1_set(void *data, u64 val)
{
	thermal_state.thz->tc1 = val;
	return 0;
}

static int tegra_thermal_tc1_get(void *data, u64 *val)
{
	*val = (u64)thermal_state.thz->tc1;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(tc1_fops,
			tegra_thermal_tc1_get,
			tegra_thermal_tc1_set,
			"%llu\n");

static int tegra_thermal_tc2_set(void *data, u64 val)
{
	thermal_state.thz->tc2 = val;
	return 0;
}

static int tegra_thermal_tc2_get(void *data, u64 *val)
{
	*val = (u64)thermal_state.thz->tc2;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(tc2_fops,
			tegra_thermal_tc2_get,
			tegra_thermal_tc2_set,
			"%llu\n");

static int tegra_thermal_passive_delay_set(void *data, u64 val)
{
	thermal_state.thz->passive_delay = val;
	return 0;
}

static int tegra_thermal_passive_delay_get(void *data, u64 *val)
{
	*val = (u64)thermal_state.thz->passive_delay;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(passive_delay_fops,
			tegra_thermal_passive_delay_get,
			tegra_thermal_passive_delay_set,
			"%llu\n");
#endif


static struct dentry *thermal_debugfs_root;

static int __init tegra_thermal_debug_init(void)
{
	thermal_debugfs_root = debugfs_create_dir("tegra_thermal", 0);

	if (!debugfs_create_file("throttle_temp_tj", 0644, thermal_debugfs_root,
				 NULL, &throttle_temp_tj_fops))
		goto err_out;

	if (!debugfs_create_file("shutdown_temp_tj", 0644, thermal_debugfs_root,
				 NULL, &shutdown_temp_tj_fops))
		goto err_out;

	if (!debugfs_create_file("temp_tj", 0644, thermal_debugfs_root,
				 NULL, &temp_tj_fops))
		goto err_out;

#ifdef CONFIG_TEGRA_THERMAL_SYSFS
	if (!debugfs_create_file("tc1", 0644, thermal_debugfs_root,
				 NULL, &tc1_fops))
		goto err_out;

	if (!debugfs_create_file("tc2", 0644, thermal_debugfs_root,
				 NULL, &tc2_fops))
		goto err_out;

	if (!debugfs_create_file("passive_delay", 0644, thermal_debugfs_root,
				 NULL, &passive_delay_fops))
		goto err_out;
#endif

	return 0;

err_out:
	debugfs_remove_recursive(thermal_debugfs_root);
	return -ENOMEM;
}

late_initcall(tegra_thermal_debug_init);
#endif
