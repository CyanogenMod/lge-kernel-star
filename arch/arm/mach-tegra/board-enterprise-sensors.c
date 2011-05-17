/*
 * arch/arm/mach-tegra/board-enterprise-sensors.c
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
#include <linux/err.h>
#include <linux/mpu.h>
#include <mach/gpio.h>
#include "gpio-names.h"
#include "board-enterprise.h"

#define SENSOR_MPU_NAME "mpu3050"
static struct mpu3050_platform_data mpu3050_data = {
	.int_config  = 0x10,
	/* Orientation matrix for MPU on enterprise */
	.orientation = { 0, -1, 0, -1, 0, 0, 0, 0, -1 },
	.level_shifter = 0,

	.accel = {
		.get_slave_descr = get_accel_slave_descr,
		.adapt_num   = 0,
		.bus         = EXT_SLAVE_BUS_SECONDARY,
		.address     = 0x0F,
		/* Orientation matrix for Kionix on enterprise */
		.orientation = { 0, -1, 0, -1, 0, 0, 0, 0, -1 },

	},

	.compass = {
		.get_slave_descr = get_compass_slave_descr,
		.adapt_num   = 0,
		.bus         = EXT_SLAVE_BUS_PRIMARY,
		.address     = 0x0C,
		/* Orientation matrix for AKM on enterprise */
		.orientation = { 1, 0, 0, 0, -1, 0, 0, 0, -1 },
	},
};

static struct i2c_board_info __initdata mpu3050_i2c0_boardinfo[] = {
	{
		I2C_BOARD_INFO(SENSOR_MPU_NAME, 0x68),
		.irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PH4),
		.platform_data = &mpu3050_data,
	},
};

static void enterprise_mpuirq_init(void)
{
	int ret = 0;

	tegra_gpio_enable(TEGRA_GPIO_PH4);
	ret = gpio_request(TEGRA_GPIO_PH4, SENSOR_MPU_NAME);
	if (ret < 0) {
		pr_err("%s: gpio_request failed %d\n", __func__, ret);
		return;
	}

	ret = gpio_direction_input(TEGRA_GPIO_PH4);
	if (ret < 0) {
		pr_err("%s: gpio_direction_input failed %d\n", __func__, ret);
		gpio_free(TEGRA_GPIO_PH4);
		return;
	}

	i2c_register_board_info(0, mpu3050_i2c0_boardinfo,
				ARRAY_SIZE(mpu3050_i2c0_boardinfo));
}

static struct i2c_board_info enterprise_i2c0_isl_board_info[] = {
	{
		I2C_BOARD_INFO("isl29028", 0x44),
	}
};

static void enterprise_isl_init(void)
{
	i2c_register_board_info(0, enterprise_i2c0_isl_board_info,
				ARRAY_SIZE(enterprise_i2c0_isl_board_info));
}

int __init enterprise_sensors_init(void)
{
	enterprise_isl_init();
	enterprise_mpuirq_init();

	return 0;
}
