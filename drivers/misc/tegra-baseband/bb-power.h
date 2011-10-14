/*
 * drivers/misc/tegra-baseband/bb-power.h
 *
 * Copyright (C) 2011 NVIDIA Corporation
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

enum tegra_bb_callback_code {
	CB_CODE_INIT = 1,
	CB_CODE_DEINIT,
	CB_CODE_L0L2,
	CB_CODE_L2L0,
	CB_CODE_L2L3,
	CB_CODE_L3L0,
	CB_CODE_INVALID,
};

struct tegra_bb_gpio_data {
	struct gpio data;
	bool doexport;
};

struct tegra_bb_gpio_irqdata {
	int id;
	const char *name;
	irq_handler_t handler;
	int flags;
	void *cookie;
};

struct tegra_bb_power_gdata {
	struct tegra_bb_gpio_data *gpio;
	struct tegra_bb_gpio_irqdata *gpioirq;
};

typedef void* (*bb_init_cb)(void *pdata, int code);
typedef int (*bb_power_cb)(int code);

#ifdef CONFIG_TEGRA_BB_M7400
extern void *m7400_init(void *pdata, int code);
#define M7400_INIT_CB m7400_init
extern int m7400_power_callback(int code);
#define M7400_PWR_CB m7400_power_callback
#else
#define M7400_INIT_CB NULL
#define M7400_PWR_CB NULL
#endif
