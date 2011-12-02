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

enum tegra_bb_pwrstate {
	PWRSTATE_L2L3,
	PWRSTATE_L3L0,
	PWRSTATE_INVALID,
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

typedef void* (*bb_get_cblist)(void);
typedef void* (*bb_init_cb)(void *pdata);
typedef void* (*bb_deinit_cb)(void);
typedef int (*bb_power_cb)(int code);
typedef int (*bb_attrib_cb)(struct device *dev, int value);

struct tegra_bb_callback {
	bb_init_cb init;
	bb_deinit_cb deinit;
	bb_power_cb power;
	bb_attrib_cb attrib;
	bool valid;
};

#ifdef CONFIG_TEGRA_BB_M7400
extern void *m7400_get_cblist(void);
#define M7400_CB m7400_get_cblist
#else
#define M7400_CB NULL
#endif
