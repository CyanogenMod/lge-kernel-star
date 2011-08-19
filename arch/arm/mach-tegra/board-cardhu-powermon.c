/*
 * arch/arm/mach-tegra/board-cardhu-powermon.c
 *
 * Copyright (c) 2011, NVIDIA, All Rights Reserved.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <linux/i2c.h>
#include <linux/ina219.h>
#include "board-cardhu.h"

static struct ina219_platform_data power_mon_info[] = {
	{
		.calibration_data  = 0xa000,
		.power_lsb = 2,
		.rail_name = "VDD_AC_BAT",
		.divisor = 20,
	},
	{
		.calibration_data  = 0xa000,
		.power_lsb = 2,
		.rail_name = "VDD_DRAM_IN",
		.divisor = 20,
	},
	{
		.calibration_data  = 0x6aaa,
		.power_lsb = 1,
		.rail_name = "VDD_BACKLIGHT_IN",
		.divisor = 20,
	},
	{
		.calibration_data  = 0xa000,
		.power_lsb = 1,
		.rail_name = "VDD_CPU_IN",
		.divisor = 20,
	},
	{
		.calibration_data  = 0x6aaa,
		.power_lsb = 1,
		.rail_name = "VDD_CORE_IN",
		.divisor = 20,
	},
	{
		.calibration_data  = 0x4000,
		.power_lsb = 1,
		.rail_name = "VDD_DISPLAY_IN",
		.divisor = 20,
	},
	{
		.calibration_data  = 0x6aaa,
		.power_lsb = 1,
		.rail_name = "VDD_3V3_TEGRA",
		.divisor = 20,
	},
	{
		.calibration_data  = 0xa000,
		.power_lsb = 1,
		.rail_name = "VDD_OTHER_PMU_IN",
		.divisor = 20,
	},
	{
		.calibration_data  = 0x4000,
		.power_lsb = 1,
		.rail_name = "VDD_1V8_TEGRA",
		.divisor = 20,
	},
	{
		.calibration_data  = 0xa000,
		.power_lsb = 1,
		.rail_name = "VDD_1V8_OTHER",
		.divisor = 20,
	},
	/* All unused INA219 devices use below data*/
	{
		.calibration_data  = 0x4000,
		.power_lsb = 1,
		.rail_name = "unused_rail",
		.divisor = 20,
	},
};

static struct i2c_board_info cardhu_i2c0_ina219_board_info[] = {
	{
		I2C_BOARD_INFO("ina219", 0x40),
		.platform_data = &power_mon_info[0],
		.irq = -1,
	},
	{
		I2C_BOARD_INFO("ina219", 0x41),
		.platform_data = &power_mon_info[1],
		.irq = -1,
	},
	{
		I2C_BOARD_INFO("ina219", 0x42),
		.platform_data = &power_mon_info[2],
		.irq = -1,
	},
	{
		I2C_BOARD_INFO("ina219", 0x43),
		.platform_data = &power_mon_info[3],
		.irq = -1,
	},
	{
		I2C_BOARD_INFO("ina219", 0x44),
		.platform_data = &power_mon_info[4],
		.irq = -1,
	},
	{
		I2C_BOARD_INFO("ina219", 0x45),
		.platform_data = &power_mon_info[5],
		.irq = -1,
	},
	{
		I2C_BOARD_INFO("ina219", 0x46),
		.platform_data = &power_mon_info[6],
		.irq = -1,
	},
	{
		I2C_BOARD_INFO("ina219", 0x47),
		.platform_data = &power_mon_info[7],
		.irq = -1,
	},
	{
		I2C_BOARD_INFO("ina219", 0x48),
		.platform_data = &power_mon_info[8],
		.irq = -1,
	},
	{
		I2C_BOARD_INFO("ina219", 0x49),
		.platform_data = &power_mon_info[9],
		.irq = -1,
	},
	{
		I2C_BOARD_INFO("ina219", 0x4A),
		.platform_data = &power_mon_info[10],
		.irq = -1,
	},
	{
		I2C_BOARD_INFO("ina219", 0x4B),
		.platform_data = &power_mon_info[10],
		.irq = -1,
	},
	{
		I2C_BOARD_INFO("ina219", 0x4C),
		.platform_data = &power_mon_info[10],
		.irq = -1,
	},
	{
		I2C_BOARD_INFO("ina219", 0x4D),
		.platform_data = &power_mon_info[10],
		.irq = -1,
	},
	{
		I2C_BOARD_INFO("ina219", 0x4E),
		.platform_data = &power_mon_info[10],
		.irq = -1,
	},
	{
		I2C_BOARD_INFO("ina219", 0x4F),
		.platform_data = &power_mon_info[10],
		.irq = -1,
	},
};

int __init cardhu_pmon_init(void)
{
	i2c_register_board_info(0, cardhu_i2c0_ina219_board_info,
		ARRAY_SIZE(cardhu_i2c0_ina219_board_info));
	return 0;
}

