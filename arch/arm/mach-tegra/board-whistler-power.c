/*
 * Copyright (C) 2010 NVIDIA, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */
#include <linux/i2c.h>
#include <linux/pda_power.h>
#include <linux/platform_device.h>
#include <linux/resource.h>
#include <linux/regulator/machine.h>
#include <linux/mfd/tps6586x.h>
#include <linux/gpio.h>
#include <mach/suspend.h>
#include <linux/io.h>

#include <mach/iomap.h>
#include <mach/irqs.h>

#include "gpio-names.h"
#include "power.h"
#include "wakeups-t2.h"
#include "board.h"

static struct tegra_suspend_platform_data whistler_suspend_data = {
	.cpu_timer	= 2000,
	.cpu_off_timer	= 0,
	.suspend_mode	= TEGRA_SUSPEND_NONE,
	.core_timer	= 0x7e7e,
	.core_off_timer = 0,
	.separate_req	= true,
	.corereq_high	= true,
	.sysclkreq_high	= true,
	.wake_enb	= 0,
	.wake_high	= 0,
	.wake_low	= 0,
	.wake_any	= 0,
};

int __init whistler_regulator_init(void)
{
	tegra_init_suspend(&whistler_suspend_data);

	return 0;
}
