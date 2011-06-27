/*
 * drivers/misc/max1749.c
 *
 * Driver for MAX1749, vibrator motor driver.
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

#include <linux/regulator/consumer.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/hrtimer.h>
#include <linux/delay.h>

#include "../staging/android/timed_output.h"

static struct regulator *regulator;
static int timeout;

static void vibrator_start(void)
{
	regulator_enable(regulator);
}

static void vibrator_stop(void)
{
	int ret;

	ret = regulator_is_enabled(regulator);
	if (ret > 0)
		regulator_disable(regulator);
}

/*
 * Timeout value can be changed from sysfs entry
 * created by timed_output_dev.
 * echo 100 > /sys/class/timed_output/vibrator/enable
 */
static void vibrator_enable(struct timed_output_dev *dev, int value)
{
	timeout = value;
	if (!regulator)
		return;

	if (value) {
		vibrator_start();
		msleep(value);
		vibrator_stop();
	} else {
		vibrator_stop();
	}
}

/*
 * Timeout value can be read from sysfs entry
 * created by timed_output_dev.
 * cat /sys/class/timed_output/vibrator/enable
 */
static int vibrator_get_time(struct timed_output_dev *dev)
{
	return timeout;
}

static struct timed_output_dev vibrator_dev = {
	.name		= "vibrator",
	.get_time	= vibrator_get_time,
	.enable		= vibrator_enable,
};

static int __init vibrator_init(void)
{
	int status;

	regulator = regulator_get(NULL, "vdd_vbrtr");
	if (IS_ERR_OR_NULL(regulator)) {
		pr_err("vibrator_init:Couldn't get regulator vdd_vbrtr\n");
		regulator = NULL;
		return PTR_ERR(regulator);
	}

	status = timed_output_dev_register(&vibrator_dev);

	if (status) {
		regulator_put(regulator);
		regulator = NULL;
	}
	return status;
}

static void __exit vibrator_exit(void)
{
	if (regulator) {
		timed_output_dev_unregister(&vibrator_dev);
		regulator_put(regulator);
		regulator = NULL;
	}
}

MODULE_DESCRIPTION("timed output vibrator device");
MODULE_AUTHOR("GPL");

module_init(vibrator_init);
module_exit(vibrator_exit);
