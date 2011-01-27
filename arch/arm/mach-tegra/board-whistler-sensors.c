/*
 * arch/arm/mach-tegra/board-whistler-sensors.c
 *
 * Copyright (c) 2010, NVIDIA, All Rights Reserved.
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
#include <mach/gpio.h>
#include <media/ov5650.h>

#include "gpio-names.h"

#define ADXL34X_IRQ_GPIO			TEGRA_GPIO_PAA1
#define CAMERA_RESET2_SHUTTER_GPIO	TEGRA_GPIO_PBB1
#define CAMERA_PWNDN1_GPIO			TEGRA_GPIO_PBB4
#define CAMERA_PWNDN2_STROBE_GPIO	TEGRA_GPIO_PBB5
#define CAMERA_RESET1_GPIO			TEGRA_GPIO_PD2
#define CAMERA_FLASH_GPIO			TEGRA_GPIO_PA0
#define ISL29018_IRQ_GPIO			TEGRA_GPIO_PK2

static int whistler_camera_init(void)
{
	tegra_gpio_enable(CAMERA_PWNDN1_GPIO);
	gpio_request(CAMERA_PWNDN1_GPIO, "camera_powerdown");
	gpio_direction_output(CAMERA_PWNDN1_GPIO, 0);
	gpio_export(CAMERA_PWNDN1_GPIO, false);

	tegra_gpio_enable(CAMERA_RESET1_GPIO);
	gpio_request(CAMERA_RESET1_GPIO, "camera_reset1");
	gpio_direction_output(CAMERA_RESET1_GPIO, 1);
	gpio_export(CAMERA_RESET1_GPIO, false);

	tegra_gpio_enable(CAMERA_RESET2_SHUTTER_GPIO);
	gpio_request(CAMERA_RESET2_SHUTTER_GPIO, "camera_reset2_shutter");
	gpio_direction_output(CAMERA_RESET2_SHUTTER_GPIO, 1);
	gpio_export(CAMERA_RESET2_SHUTTER_GPIO, false);

	tegra_gpio_enable(CAMERA_PWNDN2_STROBE_GPIO);
	gpio_request(CAMERA_PWNDN2_STROBE_GPIO, "camera_pwrdwn2_strobe");
	gpio_direction_output(CAMERA_PWNDN2_STROBE_GPIO, 0);
	gpio_export(CAMERA_PWNDN2_STROBE_GPIO, false);

	tegra_gpio_enable(CAMERA_FLASH_GPIO);
	gpio_request(CAMERA_FLASH_GPIO, "camera_flash");
	gpio_direction_output(CAMERA_FLASH_GPIO, 0);
	gpio_export(CAMERA_FLASH_GPIO, false);

	return 0;
}

static int whistler_ov5650_power_on(void)
{
	return 0;
}

static int whistler_ov5650_power_off(void)
{
	return 0;
}

struct ov5650_platform_data whistler_ov5650_data = {
	.power_on = whistler_ov5650_power_on,
	.power_off = whistler_ov5650_power_off,
};

static struct i2c_board_info whistler_i2c3_board_info[] = {
	{
		I2C_BOARD_INFO("ov5650", 0x36),
		.platform_data = &whistler_ov5650_data,
	},
};

static void whistler_adxl34x_init(void)
{
	tegra_gpio_enable(ADXL34X_IRQ_GPIO);
	gpio_request(ADXL34X_IRQ_GPIO, "adxl34x");
	gpio_direction_input(ADXL34X_IRQ_GPIO);
}

static void whistler_isl29018_init(void)
{
	tegra_gpio_enable(ISL29018_IRQ_GPIO);
	gpio_request(ISL29018_IRQ_GPIO, "isl29018");
	gpio_direction_input(ISL29018_IRQ_GPIO);
}

static struct i2c_board_info whistler_i2c1_board_info[] = {
	{
		I2C_BOARD_INFO("adxl34x", 0x1D),
		.irq = TEGRA_GPIO_TO_IRQ(ADXL34X_IRQ_GPIO),
	},
	{
		I2C_BOARD_INFO("isl29018", 0x44),
		.irq = TEGRA_GPIO_TO_IRQ(ISL29018_IRQ_GPIO),
	},
};

int __init whistler_sensors_init(void)
{
	whistler_camera_init();

	whistler_adxl34x_init();

	whistler_isl29018_init();

	i2c_register_board_info(0, whistler_i2c1_board_info,
		ARRAY_SIZE(whistler_i2c1_board_info));

	i2c_register_board_info(3, whistler_i2c3_board_info,
		ARRAY_SIZE(whistler_i2c3_board_info));

	return 0;
}
