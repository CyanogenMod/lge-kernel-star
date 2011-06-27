/*
 * include/linux/regulator/tps80031-regulator.h
 *
 * Interface for regulator driver for TI TPS80031
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

#ifndef __REGULATOR_TPS80031_H
#define	__REGULATOR_TPS80031_H

#include <linux/regulator/machine.h>

enum {
	/* USBLDO input selection */
	USBLDO_INPUT_VSYS	= 0x00000001,
	USBLDO_INPUT_PMID	= 0x00000002,

	/* LDO3 output mode */
	LDO3_OUTPUT_VIB		= 0x00000004,

	/* VBUS configuration */
	VBUS_SW_ONLY		= 0x00000008,
	VBUS_SW_N_ID		= 0x00000010,
};

/*
 * struct tps80031_regulator_platform_data - tps80031 regulator platform data.
 *
 * @regulator: The regulator init data.
 * @init_uV: initial micro volts which need to be set.
 * @init_enable: Enable or do not enable the rails during initialization.
 * @init_apply: Init parameter applied or not.
 * @flags: Configuration flag to configure the rails. It should be ORed of
 *         above enums.
 */

struct tps80031_regulator_platform_data {
	struct regulator_init_data regulator;
	int init_uV;
	unsigned init_enable:1;
	unsigned init_apply:1;
	unsigned int flags;
};

#endif	/* __REGULATOR_TPS80031_H */
