/*
 * arch/arm/mach-tegra/board-cardhu-kbc.c
 * Keys configuration for Nvidia tegra3 cardhu platform.
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
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/gpio_keys.h>
#include <linux/mfd/tps6591x.h>
#include <linux/interrupt_keys.h>
#include <linux/gpio_scrollwheel.h>

#include <mach/irqs.h>
#include <mach/io.h>
#include <mach/iomap.h>
#include <mach/kbc.h>
#include "board.h"
#include "board-cardhu.h"

#include "gpio-names.h"

#define CARDHU_ROW_COUNT	4
#define CARDHU_COL_COUNT	2
static int plain_kbd_keycode[] = {
	KEY_POWER,	KEY_RESERVED,
	KEY_HOME,	KEY_BACK,
	KEY_CAMERA,	KEY_CAMERA,
	KEY_VOLUMEDOWN,	KEY_VOLUMEUP
};

static struct tegra_kbc_wake_key cardhu_wake_cfg[] = {
	[0] = {
		.row = 0,
		.col = 0,
	},
};

static struct tegra_kbc_platform_data cardhu_kbc_platform_data = {
	.debounce_cnt = 20,
	.repeat_cnt = 50 * 32,
	.scan_timeout_cnt = 3000 * 32,
	.plain_keycode = plain_kbd_keycode,
	.fn_keycode = NULL,
	.is_filter_keys = false,
	.is_wake_on_any_key = false,
	.wake_key_cnt = 1,
	.wake_cfg = &cardhu_wake_cfg[0],
};

static struct resource cardhu_kbc_resources[] = {
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


struct platform_device cardhu_kbc_device = {
	.name = "tegra-kbc",
	.id = -1,
	.dev = {
		.platform_data = &cardhu_kbc_platform_data,
	},
	.resource = cardhu_kbc_resources,
	.num_resources = ARRAY_SIZE(cardhu_kbc_resources),
};

int __init cardhu_kbc_init(void)
{
	struct tegra_kbc_platform_data *data = &cardhu_kbc_platform_data;
	int i;
	struct board_info board_info;

	tegra_get_board_info(&board_info);
	if ((board_info.board_id == BOARD_E1198) ||
			(board_info.board_id == BOARD_E1291))
		return 0;

	pr_info("Registering tegra-kbc\n");
	 /* Setup the pin configuration information. */
	for (i = 0; i < KBC_MAX_GPIO; i++) {
		data->pin_cfg[i].num = 0;
		data->pin_cfg[i].pin_type = kbc_pin_unused;
	}
	for (i = 0; i < CARDHU_ROW_COUNT; i++) {
		data->pin_cfg[i].num = i;
		data->pin_cfg[i].pin_type = kbc_pin_row;
	}

	for (i = 0; i < CARDHU_COL_COUNT; i++) {
		data->pin_cfg[i + CARDHU_ROW_COUNT].num = i;
		data->pin_cfg[i + CARDHU_ROW_COUNT].pin_type = kbc_pin_col;
	}
	platform_device_register(&cardhu_kbc_device);
	return 0;
}

int __init cardhu_scroll_init(void)
{
	return 0;
}

#define GPIO_KEY(_id, _gpio, _iswake)		\
	{					\
		.code = _id,			\
		.gpio = TEGRA_GPIO_##_gpio,	\
		.active_low = 1,		\
		.desc = #_id,			\
		.type = EV_KEY,			\
		.wakeup = _iswake,		\
		.debounce_interval = 10,	\
	}

static struct gpio_keys_button cardhu_keys_pm269[] = {
	[0] = GPIO_KEY(KEY_POWER, PV0, 1),
};

static struct gpio_keys_platform_data cardhu_keys_pm269_platform_data = {
	.buttons	= cardhu_keys_pm269,
	.nbuttons	= ARRAY_SIZE(cardhu_keys_pm269),
};

static struct platform_device cardhu_keys_pm269_device = {
	.name   = "gpio-keys",
	.id     = 0,
	.dev    = {
		.platform_data  = &cardhu_keys_pm269_platform_data,
	},
};

static struct gpio_keys_button cardhu_keys_e1198[] = {
	[0] = GPIO_KEY(KEY_HOME, PQ0, 0),
	[1] = GPIO_KEY(KEY_BACK, PQ1, 0),
	[2] = GPIO_KEY(KEY_MENU, PQ2, 0),
	[3] = GPIO_KEY(KEY_SEARCH, PQ3, 0),
	[4] = GPIO_KEY(KEY_VOLUMEUP, PR0, 0),
	[5] = GPIO_KEY(KEY_VOLUMEDOWN, PR1, 0),
	[6] = GPIO_KEY(KEY_POWER, PV0, 1),
};

static struct gpio_keys_platform_data cardhu_keys_e1198_platform_data = {
	.buttons	= cardhu_keys_e1198,
	.nbuttons	= ARRAY_SIZE(cardhu_keys_e1198),
};

static struct platform_device cardhu_keys_e1198_device = {
	.name   = "gpio-keys",
	.id     = 0,
	.dev    = {
		.platform_data  = &cardhu_keys_e1198_platform_data,
	},
};

static struct gpio_keys_button cardhu_keys_e1291[] = {
	[0] = GPIO_KEY(KEY_POWER, PR0, 0),
	[1] = GPIO_KEY(KEY_BACK, PR1, 0),
	[2] = GPIO_KEY(KEY_HOME, PR2, 0),
	[3] = GPIO_KEY(KEY_SEARCH, PQ3, 0),
	[4] = GPIO_KEY(KEY_VOLUMEUP, PQ0, 0),
	[5] = GPIO_KEY(KEY_VOLUMEDOWN, PQ1, 0),
};

static struct gpio_keys_platform_data cardhu_keys_e1291_platform_data = {
	.buttons	= cardhu_keys_e1291,
	.nbuttons	= ARRAY_SIZE(cardhu_keys_e1291),
};

static struct platform_device cardhu_keys_e1291_device = {
	.name   = "gpio-keys",
	.id     = 0,
	.dev    = {
		.platform_data  = &cardhu_keys_e1291_platform_data,
	},
};

#define INT_KEY(_id, _irq, _iswake, _deb_int)	\
	{					\
		.code = _id,			\
		.irq = _irq,			\
		.active_low = 1,		\
		.desc = #_id,			\
		.type = EV_KEY,			\
		.wakeup = _iswake,		\
		.debounce_interval = _deb_int,	\
	}
static struct interrupt_keys_button cardhu_int_keys_e1291[] = {
	[0] = INT_KEY(KEY_MENU, TPS6591X_IRQ_BASE + TPS6591X_INT_PWRON, 0, 100),
	[1] = INT_KEY(KEY_POWER, TPS6591X_IRQ_BASE + TPS6591X_INT_PWRON_LP, 0, 8000),
};

static struct interrupt_keys_platform_data cardhu_int_keys_e1291_pdata = {
	.int_buttons	= cardhu_int_keys_e1291,
	.nbuttons       = ARRAY_SIZE(cardhu_int_keys_e1291),
};

static struct platform_device cardhu_int_keys_e1291_device = {
	.name   = "interrupt-keys",
	.id     = 0,
	.dev    = {
		.platform_data  = &cardhu_int_keys_e1291_pdata,
	},
};

int __init cardhu_keys_init(void)
{
	int i;
	struct board_info board_info;

	tegra_get_board_info(&board_info);
	if (!((board_info.board_id == BOARD_E1198) ||
		(board_info.board_id == BOARD_E1291) ||
		(board_info.board_id == BOARD_PM269)))
		return 0;

	pr_info("Registering gpio keys\n");

	if (board_info.board_id == BOARD_E1291) {
		/* Enable gpio mode for other pins */
		for (i = 0; i < ARRAY_SIZE(cardhu_keys_e1291); i++)
			tegra_gpio_enable(cardhu_keys_e1291[i].gpio);

		platform_device_register(&cardhu_keys_e1291_device);
		platform_device_register(&cardhu_int_keys_e1291_device);
	} else if (board_info.board_id == BOARD_PM269) {
		for (i = 0; i < ARRAY_SIZE(cardhu_keys_pm269); i++)
			tegra_gpio_enable(cardhu_keys_pm269[i].gpio);

		platform_device_register(&cardhu_keys_pm269_device);
	} else {
		/* For E1198 */
		for (i = 0; i < ARRAY_SIZE(cardhu_keys_e1198); i++)
			tegra_gpio_enable(cardhu_keys_e1198[i].gpio);

		platform_device_register(&cardhu_keys_e1198_device);
	}
	return 0;
}
