/*
 * Copyright (c) 2011, NVIDIA Corporation.
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
 */

#ifndef _GPIO_SWITCH_REGULATOR_H
#define _GPIO_SWITCH_REGULATOR_H

#include <linux/regulator/machine.h>

/*
 * struct gpio_regulator_switch_platform_data - Gpio based regulato switch
 * platform data.
 *
 * Platform data to register a gpio regulator switch device driver.
 *
 * @regulator_name: The name of regulator.
 * @input_supply: Input supply name.
 * @id: The id of the switch.
 * @gpio_nr: Gpio nr which controls this switch.
 * @active_low: true if making gpio low makes voltage output enable.
 * @init_state: 1 if init_state should be active.
 * @voltages: Possible voltages to set at output.
 * @n_voltages: Number of voltages.
 * @num_consumer_supplies: Number of cosumer supplies.
 * @consumer_supplies: List of consumer spllies.
 */
struct gpio_switch_regulator_platform_data {
	const char	*regulator_name;
	const char	*input_supply;
	int id;
	int gpio_nr;
	int active_low;
	int init_state;
	int *voltages;
	unsigned n_voltages;
	struct regulator_consumer_supply *consumer_supplies;
	int num_consumer_supplies;
	struct regulation_constraints constraints;
};
#endif
