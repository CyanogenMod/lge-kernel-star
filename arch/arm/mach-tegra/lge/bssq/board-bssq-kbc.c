/*
 * Copyright (C) 2010-2011 NVIDIA, Inc.
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

#define BSSQ_ROW_COUNT	8
#define BSSQ_COL_COUNT	6

//MOBII_CHANGES_S 20120626 sgkim@mobii.co.kr - lu6500 qwerty driver porting.
#if defined(CONFIG_LU6500)
static const u32 bssq_keymap[] = {
	KEY(0, 0, KEY_Q),
	KEY(0, 1, KEY_W),
	KEY(0, 2, KEY_E),
	KEY(0, 3, KEY_R),
	KEY(0, 4, KEY_T),
	KEY(0, 5, KEY_Y),
	
	KEY(1, 0, KEY_U),
	KEY(1, 1, KEY_I),
	KEY(1, 2, KEY_O),
	KEY(1, 3, KEY_P),
	KEY(1, 4, KEY_PROG1),
	KEY(1, 5, KEY_A),

	KEY(2, 0, KEY_S),
	KEY(2, 1, KEY_D),
	KEY(2, 2, KEY_F),
	KEY(2, 3, KEY_G),
	KEY(2, 4, KEY_H),
	KEY(2, 5, KEY_J),

	KEY(3, 0, KEY_K),
	KEY(3, 1, KEY_L),
	KEY(3, 2, KEY_COMMA),
	KEY(3, 3, KEY_BACKSPACE),
	KEY(3, 4, KEY_LEFTSHIFT),
	KEY(3, 5, KEY_Z),

	KEY(4, 0, KEY_X),
	KEY(4, 1, KEY_C),
	KEY(4, 2, KEY_V),
	KEY(4, 3, KEY_B),
	KEY(4, 4, KEY_N),
	KEY(4, 5, KEY_M),

	KEY(5, 0, KEY_DOT),
	KEY(5, 1, KEY_LEFT),
	KEY(5, 2, KEY_ENTER),
	KEY(5, 3, KEY_LEFTALT),
	KEY(5, 4, KEY_BACK),
	KEY(5, 5, KEY_MENU),

	KEY(6, 0, KEY_SEARCH),
	KEY(6, 1, KEY_SPACE),
	KEY(6, 2, KEY_PROG2),
	KEY(6, 3, KEY_PROG3),
	KEY(6, 4, KEY_DOWN),
	KEY(6, 5, KEY_RIGHT),

	KEY(7, 0, KEY_UP),
	KEY(7, 1, KEY_RESERVED),
	KEY(7, 2, KEY_RESERVED),
	KEY(7, 3, KEY_RESERVED),
	KEY(7, 4, KEY_RESERVED),
	KEY(7, 5, KEY_PROG4),
	
};
#else
static const u32 bssq_keymap[] = {
	KEY(0, 0, KEY_Q),
	KEY(0, 1, KEY_W),
	KEY(0, 2, KEY_E),
	KEY(0, 3, KEY_R),
	KEY(0, 4, KEY_T),
	KEY(0, 5, KEY_Y),
	KEY(0, 6, KEY_U),
	KEY(0, 7, KEY_I),
	KEY(0, 8, KEY_O),
	KEY(0, 9, KEY_P),
	KEY(0, 10, KEY_PROG1)/**/,
	
	KEY(1, 0, KEY_A),
	KEY(1, 1, KEY_S),
	KEY(1, 2, KEY_D),
	KEY(1, 3, KEY_F),
	KEY(1, 4, KEY_G),
	KEY(1, 5, KEY_H),
	KEY(1, 6, KEY_J),
	KEY(1, 7, KEY_K),
	KEY(1, 8, KEY_L),
	KEY(1, 9, KEY_COMMA)/**/,
	KEY(1, 10, KEY_BACKSPACE),

	KEY(2, 0, KEY_LEFTSHIFT),
	KEY(2, 1, KEY_Z),
	KEY(2, 2, KEY_X),
	KEY(2, 3, KEY_C),
	KEY(2, 4, KEY_V),
	KEY(2, 5, KEY_B),
	KEY(2, 6, KEY_N),
	KEY(2, 7, KEY_M),
	KEY(2, 8, KEY_DOT),
	KEY(2, 9, KEY_UP),
	KEY(2, 10, KEY_ENTER),

	KEY(3, 0, KEY_LEFTALT),
	KEY(3, 1, KEY_BACK),
	KEY(3, 2, KEY_MENU),
	KEY(3, 3, KEY_SEARCH),
	KEY(3, 4, KEY_SPACE),
	KEY(3, 5, KEY_PROG2/**/),
	KEY(3, 6, KEY_PROG3),
	KEY(3, 7, KEY_LEFT),
	KEY(3, 8, KEY_DOWN),
	KEY(3, 9, KEY_RIGHT),

	KEY(4, 0, KEY_RESERVED),
	KEY(4, 1, KEY_RESERVED),
	KEY(4, 2, KEY_RESERVED),
	KEY(4, 3, KEY_RESERVED),
	KEY(4, 4, KEY_PROG4)/**/	
};
#endif
//MOBII_CHANGES_E 20120626 sgkim@mobii.co.kr - lu6500 qwerty driver porting.

static const struct matrix_keymap_data bssq_keymap_data = {
	.keymap = bssq_keymap,
	.keymap_size = ARRAY_SIZE(bssq_keymap),
};

static struct tegra_kbc_wake_key bssq_wake_cfg[] = {
	[0] = {
		.row = 0,
		.col = 0,
	},
};

static struct tegra_kbc_platform_data bssq_kbc_platform_data = {
	.debounce_cnt = 20,
	.repeat_cnt = 50 * 32,
//	.wake_cnt = 1,
//	.wake_cfg = &bssq_wake_cfg[0],
	.wake_cnt = 0,
	.wake_cfg = NULL,
	.keymap_data = &bssq_keymap_data,
	.use_fn_map = false,
	.wakeup = true,
};

static struct resource bssq_kbc_resources[] = {
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

struct platform_device bssq_kbc_device = {
	.name = "tegra-kbc",
	.id = -1,
	.dev = {
		.platform_data = &bssq_kbc_platform_data,
	},
	.resource = bssq_kbc_resources,
	.num_resources = ARRAY_SIZE(bssq_kbc_resources),
};

int __init bssq_kbc_init(void)
{
	struct tegra_kbc_platform_data *data = &bssq_kbc_platform_data;
	int i;

	pr_info("KBC: bssq_kbc_init\n");
	for (i = 0; i < BSSQ_ROW_COUNT; i++) {
		data->pin_cfg[i].num = i;
		data->pin_cfg[i].is_row = true;
		data->pin_cfg[i].en = true;
	}
	for (i = 0; i < BSSQ_COL_COUNT; i++) {
		data->pin_cfg[i + KBC_MAX_ROW].num = i;
		data->pin_cfg[i + KBC_MAX_ROW].en = true;
	}

	platform_device_register(&bssq_kbc_device);
	return 0;
}
