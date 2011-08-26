/*
 * include/linux/tps80031-charger.h
 *
 * Battery charger driver interface for TI TPS80031 PMIC
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

#ifndef __LINUX_TPS80031_CHARGER_H
#define __LINUX_TPS80031_CHARGER_H

#include <linux/regulator/machine.h>

struct tps80031_charger_platform_data {
	int regulator_id;
	int max_charge_volt_mV;
	int max_charge_current_mA;
	int charging_term_current_mA;
	int refresh_time;
	int irq_base;
	int watch_time_sec;
	struct regulator_consumer_supply *consumer_supplies;
	int num_consumer_supplies;
	int (*board_init)(void *board_data);
	void *board_data;
};

#endif /*__LINUX_TPS80031_CHARGER_H */
