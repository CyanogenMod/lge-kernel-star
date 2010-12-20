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


#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/device.h>

#include <mach/clk.h>
#include <mach/iomap.h>
#include <mach/irqs.h>
#include <mach/pinmux.h>
#include <mach/iomap.h>
#include <mach/io.h>
#include <mach/kbc.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>


static int plain_kbd_keycode[] = {
	KEY_POWER, KEY_DOWN, KEY_RIGHT, KEY_SELECT,
		KEY_LEFT, KEY_UP, KEY_RESERVED, KEY_RESERVED,
	KEY_HOME, KEY_BACK, KEY_NUMERIC_6, KEY_NUMERIC_9,
		KEY_NUMERIC_8, KEY_NUMERIC_POUND, KEY_DOT, KEY_RESERVED,
	KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_NUMERIC_3,
		KEY_NUMERIC_5, KEY_NUMERIC_2, KEY_NUMERIC_7, KEY_RESERVED,
	KEY_VOLUMEDOWN, KEY_VOLUMEUP, KEY_RESERVED, KEY_RESERVED,
		KEY_RESERVED, KEY_RESERVED, KEY_NUMERIC_4, KEY_RESERVED,
	KEY_END, KEY_BACK, KEY_RESERVED, KEY_MENU,
		KEY_RESERVED, KEY_RESERVED, KEY_NUMERIC_1, KEY_RESERVED,
	KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED,
		KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED,
	KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED,
		KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED,
	KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED,
		KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED,
	KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED,
		KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED,
	KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED,
		KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED,
	KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED,
		KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED,
	KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED,
		KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED,
	KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED,
		KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED,
	KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED,
		KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED,
	KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED,
		KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED,
	KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED,
		KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED
};

static int fn_kbd_keycode[] = {
	KEY_POWER, KEY_DOWN, KEY_RIGHT, KEY_SELECT,
		KEY_LEFT, KEY_UP, KEY_RESERVED, KEY_RESERVED,
	KEY_HOME, KEY_BACK, KEY_NUMERIC_6, KEY_NUMERIC_9,
		KEY_NUMERIC_8, KEY_NUMERIC_POUND, KEY_DOT, KEY_RESERVED,
	KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_NUMERIC_3,
		KEY_NUMERIC_5, KEY_NUMERIC_2, KEY_NUMERIC_7, KEY_RESERVED,
	KEY_VOLUMEDOWN, KEY_VOLUMEUP, KEY_RESERVED, KEY_RESERVED,
		KEY_RESERVED, KEY_RESERVED, KEY_NUMERIC_4, KEY_RESERVED,
	KEY_END, KEY_BACK, KEY_RESERVED, KEY_MENU,
		KEY_RESERVED, KEY_RESERVED, KEY_NUMERIC_1, KEY_RESERVED,
	KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED,
		KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED,
	KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED,
		KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED,
	KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED,
		KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED,
	KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED,
		KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED,
	KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED,
		KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED,
	KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED,
		KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED,
	KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED,
		KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED,
	KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED,
		KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED,
	KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED,
		KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED,
	KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED,
		KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED,
	KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED,
		KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED
};




static struct tegra_kbc_wake_key whistler_wake_cfg[] = {
	[0] = {
		.row = 0,
		.col = 0,
	},
};


static struct tegra_kbc_platform_data whistler_kbc_platform_data = {
	.debounce_cnt = 20,
	.repeat_cnt = 50 * 32,
	.wake_cnt = 1,
	.wake_cfg = &whistler_wake_cfg[0],
	.filter_keys  = true,
};


static struct resource whistler_kbc_resources[] = {
	[0] = {
		.start = TEGRA_KBC_BASE,
		.end   = TEGRA_KBC_BASE + TEGRA_KBC_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = INT_KBC,
		.end   = INT_KBC,
		.flags = IORESOURCE_IRQ,
	},
};


struct platform_device whistler_kbc_device = {
	.name = "tegra-kbc",
	.id = -1,
	.dev = {
		.platform_data = &whistler_kbc_platform_data,
	},
	.resource = whistler_kbc_resources,
	.num_resources = ARRAY_SIZE(whistler_kbc_resources),
};

int __init whistler_kbc_init(void)
{
	struct tegra_kbc_platform_data *data = &whistler_kbc_platform_data;
	int i, j;

	pr_info("KBC: whistler_kbc_init\n");

	/*
	 * Setup the pin configuration information.
	 */
	for (i = 0; i < KBC_MAX_ROW; i++) {
		data->pin_cfg[i].num = i;
		data->pin_cfg[i].is_row = true;
		data->pin_cfg[i].is_col = false;
	}

	for (j = 0; j < 7/*KBC_MAX_COL*/; j++) {
		data->pin_cfg[i + j].num = j;
		data->pin_cfg[i + j].is_row = false;
		data->pin_cfg[i + j].is_col = true;
	}

	/* tegra-kbc will use default keycodes. */
	data->plain_keycode = plain_kbd_keycode;
	data->fn_keycode = fn_kbd_keycode;
	data->filter_keys = true;
	platform_device_register(&whistler_kbc_device);
	return 0;
}



