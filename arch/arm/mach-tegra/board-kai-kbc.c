/*
 * arch/arm/mach-tegra/board-kai-kbc.c
 * Keys configuration for Nvidia tegra3 kai platform.
 *
 * Copyright (C) 2012 NVIDIA, Inc.
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
#include <linux/interrupt_keys.h>
#include <linux/gpio_scrollwheel.h>

#include <mach/irqs.h>
#include <mach/io.h>
#include <mach/iomap.h>
#include <mach/kbc.h>
#include "board.h"
#include "board-kai.h"

#include "gpio-names.h"
#include "devices.h"

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

static struct gpio_keys_button kai_keys[] = {
	[0] = GPIO_KEY(KEY_MENU, PR2, 0),
	[1] = GPIO_KEY(KEY_BACK, PQ1, 0),
	[2] = GPIO_KEY(KEY_HOME, PQ0, 0),
	[3] = GPIO_KEY(KEY_SEARCH, PQ3, 0),
	[4] = GPIO_KEY(KEY_VOLUMEUP, PR1, 0),
	[5] = GPIO_KEY(KEY_VOLUMEDOWN, PR0, 0),
};

static struct gpio_keys_platform_data kai_keys_platform_data = {
	.buttons	= kai_keys,
	.nbuttons	= ARRAY_SIZE(kai_keys),
};

static struct platform_device kai_keys_device = {
	.name   = "gpio-keys",
	.id     = 0,
	.dev    = {
		.platform_data  = &kai_keys_platform_data,
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
static struct interrupt_keys_button kai_int_keys[] = {
};

static struct interrupt_keys_platform_data kai_int_keys_pdata = {
	.int_buttons	= kai_int_keys,
	.nbuttons       = ARRAY_SIZE(kai_int_keys),
};

static struct platform_device kai_int_keys_device = {
	.name   = "interrupt-keys",
	.id     = 0,
	.dev    = {
		.platform_data  = &kai_int_keys_pdata,
	},
};

int __init kai_keys_init(void)
{
	int i;

	pr_info("Registering gpio keys\n");

	/* Enable gpio mode for other pins */
	for (i = 0; i < kai_keys_platform_data.nbuttons; i++)
		tegra_gpio_enable(kai_keys_platform_data.
					buttons[i].gpio);

	platform_device_register(&kai_keys_device);
	platform_device_register(&kai_int_keys_device);

	return 0;
}
