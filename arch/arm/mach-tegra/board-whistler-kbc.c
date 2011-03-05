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

/*
* Scrollwheel is connected to KBC pins but has it's own
* driver using those pins as gpio.
* In case of using scrollwheel  Row3 and Col3/4/5
* should NOT be configured as KBC
*/
#ifdef CONFIG_INPUT_ALPS_GPIO_SCROLLWHEEL
#define WHISTLER_ROW_COUNT	3
#define WHISTLER_COL_COUNT	2
#else
#define WHISTLER_ROW_COUNT	4
#define WHISTLER_COL_COUNT	2
#endif

#ifdef CONFIG_INPUT_ALPS_GPIO_SCROLLWHEEL
static int plain_kbd_keycode[] = {
	KEY_POWER,	KEY_RESERVED,
	KEY_HOME,	KEY_BACK,
	KEY_RESERVED,	KEY_MENU,
};
#else
static int plain_kbd_keycode[] = {
	KEY_POWER,      KEY_RESERVED, KEY_RESERVED, KEY_RESERVED,
	KEY_HOME,       KEY_BACK,     KEY_RESERVED, KEY_RESERVED,
	KEY_RESERVED,   KEY_RESERVED, KEY_RESERVED, KEY_RESERVED,
	KEY_VOLUMEDOWN, KEY_VOLUMEUP, KEY_RESERVED, KEY_RESERVED,
};
#endif
static struct tegra_kbc_wake_key whistler_wake_cfg[] = {
	[0] = {
		.row = 0,
		.col = 0,
	},
};

static struct tegra_kbc_platform_data whistler_kbc_platform_data = {
	.debounce_cnt = 20,
	.repeat_cnt = 50 * 32,
	.scan_timeout_cnt = 3000 * 32,
	.plain_keycode = plain_kbd_keycode,
	.fn_keycode = NULL,
	.is_filter_keys = false,
	.is_wake_on_any_key = false,
	.wake_key_cnt = 1,
	.wake_cfg = &whistler_wake_cfg[0],
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
	int i;

	pr_info("KBC: whistler_kbc_init\n");

	/* Setup the pin configuration information. */
	for (i = 0; i < KBC_MAX_GPIO; i++) {
		data->pin_cfg[i].num = 0;
		data->pin_cfg[i].pin_type = kbc_pin_unused;
	}

	for (i = 0; i < WHISTLER_ROW_COUNT; i++) {
		data->pin_cfg[i].num = i;
		data->pin_cfg[i].pin_type = kbc_pin_row;
	}
	for (i = 0; i < WHISTLER_COL_COUNT; i++) {
		data->pin_cfg[i + WHISTLER_ROW_COUNT].num = i;
		data->pin_cfg[i + WHISTLER_ROW_COUNT].pin_type = kbc_pin_col;
	}

	platform_device_register(&whistler_kbc_device);
	return 0;
}



