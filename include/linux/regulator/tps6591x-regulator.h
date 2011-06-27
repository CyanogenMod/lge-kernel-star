/*
 * include/linux/regulator/tps6591x-regulator.h
 *
 * Interface for regulator driver for TI TPS6591x PMIC family
 *
 * Copyright (C) 2011 NVIDIA Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 */

#ifndef __REGULATOR_TPS6591X_H
#define	__REGULATOR_TPS6591X_H

#include <linux/regulator/machine.h>

enum tps6591x_ext_control {
	EXT_CTRL_NONE = 0x0,
	EXT_CTRL_EN1,
	EXT_CTRL_EN2,
	EXT_CTRL_SLEEP_OFF,
};

/*
 * struct tps6591x_regulator_platform_data - tps6591x regulator platform data.
 *
 * @regulator: The regulator init data.
 * @init_uV: initial micro volts which need to be set.
 * @init_enable: Enable or do not enable the rails during initialization.
 * @init_apply: Init parameter applied or not.
 */

struct tps6591x_regulator_platform_data {
	struct regulator_init_data regulator;
	int init_uV;
	unsigned init_enable:1;
	unsigned init_apply:1;
	enum tps6591x_ext_control ectrl;
};

#endif	/* __REGULATOR_TPS6591X_H */
