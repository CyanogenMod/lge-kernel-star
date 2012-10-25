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

enum tegra_bb_state {
	BBSTATE_UNKNOWN,
	/* Baseband state L0 - Running */
	BBSTATE_L0,
	/* Baseband state L2 - Suspended */
	BBSTATE_L2,
	/* Baseband state L3 - Suspended and detached */
	BBSTATE_L3,
};

enum tegra_bb_pwrstate {
	/* System power state - Entering suspend */
	PWRSTATE_L2L3,
	/* System power state - Resuming from suspend */
	PWRSTATE_L3L0,
	PWRSTATE_INVALID,
};

struct tegra_bb_gpio_data {
	/* Baseband gpio data */
	struct gpio data;
	/* Baseband gpio - Should it be exported to sysfs ? */
	bool doexport;
};

struct tegra_bb_gpio_irqdata {
	/* Baseband gpio IRQ - Id */
	int id;
	/* Baseband gpio IRQ - Friendly name */
	const char *name;
	/* Baseband gpio IRQ - IRQ handler */
	irq_handler_t handler;
	/* Baseband gpio IRQ - IRQ trigger flags */
	int flags;
	/* Baseband gpio IRQ - Can the gpio wake system from sleep ? */
	bool wake_capable;
	void *cookie;
};

typedef void* (*bb_get_cblist)(void);
typedef void* (*bb_init_cb)(void *pdata);
typedef void* (*bb_deinit_cb)(void);
typedef int (*bb_power_cb)(int code);
typedef int (*bb_attrib_cb)(struct device *dev, int value);
typedef int (*modem_register_cb)(struct usb_device *udev);

struct tegra_bb_power_gdata {
	struct tegra_bb_gpio_data *gpio;
	struct tegra_bb_gpio_irqdata *gpioirq;
};

struct tegra_bb_power_mdata {
	/* Baseband USB vendor ID */
	int vid;
	/* Baseband USB product ID */
	int pid;
	/* Baseband capability - Can it generate a wakeup ? */
	bool wake_capable;
	/* Baseband capability - Can it be auto/runtime suspended ? */
	bool autosuspend_ready;
	/* Baseband callback after a successful registration */
	modem_register_cb reg_cb;
};

struct tegra_bb_power_data {
	struct tegra_bb_power_gdata *gpio_data;
	struct tegra_bb_power_mdata *modem_data;
};

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
