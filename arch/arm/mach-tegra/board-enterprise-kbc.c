/*
 * arch/arm/mach-tegra/board-enterprise-kbc.c
 * Keys configuration for Nvidia tegra3 enterprise platform.
 *
 * Copyright (C) 2011 NVIDIA, Inc.
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

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <mach/io.h>
#include <mach/iomap.h>
#include <mach/kbc.h>

#include "board.h"
#include "board-enterprise.h"
#include "devices.h"

#define ENTERPRISE_ROW_COUNT	3
#define ENTERPRISE_COL_COUNT	3
static int plain_kbd_keycode[] = {
	KEY_POWER,	KEY_RESERVED,	KEY_RESERVED,
	KEY_HOME,	KEY_BACK,	KEY_VOLUMEDOWN,
	KEY_MENU,	KEY_SEARCH,	KEY_VOLUMEUP,
};

static struct tegra_kbc_wake_key enterprise_wake_cfg[] = {
	[0] = {
		.row = 0,
		.col = 0,
	},
};

static struct tegra_kbc_platform_data enterprise_kbc_platform_data = {
	.debounce_cnt = 20,
	.repeat_cnt = 50 * 32,
	.scan_timeout_cnt = 3000 * 32,
	.plain_keycode = plain_kbd_keycode,
	.fn_keycode = NULL,
	.is_filter_keys = false,
	.is_wake_on_any_key = false,
	.wake_key_cnt = 1,
	.wake_cfg = &enterprise_wake_cfg[0],
};

int __init enterprise_kbc_init(void)
{
	struct tegra_kbc_platform_data *data = &enterprise_kbc_platform_data;
	int i;
	tegra_kbc_device.dev.platform_data = &enterprise_kbc_platform_data;
	pr_info("Registering tegra-kbc\n");
	 /* Setup the pin configuration information. */
	for (i = 0; i < KBC_MAX_GPIO; i++) {
		data->pin_cfg[i].num = 0;
		data->pin_cfg[i].pin_type = kbc_pin_unused;
	}
	for (i = 0; i < ENTERPRISE_ROW_COUNT; i++) {
		data->pin_cfg[i].num = i;
		data->pin_cfg[i].pin_type = kbc_pin_row;
	}

	for (i = 0; i < ENTERPRISE_COL_COUNT; i++) {
		data->pin_cfg[i + ENTERPRISE_ROW_COUNT].num = i;
		data->pin_cfg[i + ENTERPRISE_ROW_COUNT].pin_type = kbc_pin_col;
	}
	platform_device_register(&tegra_kbc_device);
	pr_info("Registering successful tegra-kbc\n");
	return 0;
}

