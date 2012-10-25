/*
 * include/linux/regulator/tps65090-regulator.h
 *
 * Interface for regulator driver for TI TPS65090 PMIC family
 *
 * Copyright (C) 2012 NVIDIA Corporation
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
 * with this program; if not, write to  Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 */

#ifndef __REGULATOR_TPS65090_H
#define __REGULATOR_TPS65090_H

#include <linux/regulator/machine.h>

#define tps65090_rails(_name) "tps65090_"#_name

enum {
	TPS65090_ID_DCDC1,
	TPS65090_ID_DCDC2,
	TPS65090_ID_DCDC3,
	TPS65090_ID_FET1,
	TPS65090_ID_FET2,
	TPS65090_ID_FET3,
	TPS65090_ID_FET4,
	TPS65090_ID_FET5,
	TPS65090_ID_FET6,
	TPS65090_ID_FET7,
};

/*
 * struct tps65090_regulator_platform_data
 *
 * @regulator: The regulator init data.
 * @init_uV: initial micro volts which need to be set.
 * @init_enable: Enable or do not enable the rails during initialization.
 * @init_apply: Init parameter applied or not.
 * @slew_rate_uV_per_us: Slew rate microvolt per microsec.
 */

struct tps65090_regulator_platform_data {
	struct regulator_init_data regulator;
	int slew_rate_uV_per_us;
	unsigned int flags;
};

#endif	/* __REGULATOR_TPS65090_H */
