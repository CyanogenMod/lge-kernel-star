/*
 * arch/arm/mach-tegra/vibrate.c
 *
 * Miscellaneous timed output driver for vibrators implemented using NVIDIA's
 * Tegra ODM kit
 *
 * Copyright (c) 2009, NVIDIA Corporation.
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

#define NV_DEBUG 0

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/hrtimer.h>
#include <linux/delay.h>

#include <../drivers/staging/android/timed_output.h>

#include "mach/nvrm_linux.h"
#include "nvos.h"
#include "nvassert.h"
#include "nvodm_query.h"
#include "nvodm_vibrate.h"

static NvOdmVibDeviceHandle s_hOdmVibrate = NULL;
static int s_Timeout;

static void vibrator_enable(struct timed_output_dev *dev, int value)
{
	s_Timeout = value;
	if (!s_hOdmVibrate) {
		NvOdmVibOpen(&s_hOdmVibrate);
		if (!s_hOdmVibrate)
			return;
	}
	if (value) {
		NvOdmVibStart(s_hOdmVibrate);
		msleep(value);
		NvOdmVibStop(s_hOdmVibrate);
	} else {
		NvOdmVibStop(s_hOdmVibrate);
	}
}

static int vibrator_get_time(struct timed_output_dev *dev)
{
	return s_Timeout;
}


static struct timed_output_dev tegra_vibrator = {
	.name		= "vibrator",
	.get_time	= vibrator_get_time,
	.enable		= vibrator_enable,
};

static int __init init_tegra_vibrator(void)
{
	int status;
	if(!s_hOdmVibrate)
		NvOdmVibOpen(&s_hOdmVibrate);

	if (!s_hOdmVibrate)
		return -ENODEV;

	s_Timeout = 0;
	status = timed_output_dev_register(&tegra_vibrator);

	if (status)
		NvOdmVibClose(s_hOdmVibrate);
	return status;
}

static void __exit exit_tegra_vibrator(void)
{
	if(s_hOdmVibrate) {
		timed_output_dev_unregister(&tegra_vibrator);
		NvOdmVibClose(s_hOdmVibrate);
		s_hOdmVibrate = NULL;
	}
}

module_init(init_tegra_vibrator);
module_exit(exit_tegra_vibrator);

MODULE_DESCRIPTION("timed output vibrator device");
MODULE_LICENSE("GPL");
