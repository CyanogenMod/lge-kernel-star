/*
 * arch/arm/mach-tegra/board-whistler-sensors.c
 *
 * Copyright (c) 2010-2011, NVIDIA, All Rights Reserved.
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

#include <linux/delay.h>
#include <linux/i2c.h>
#include <mach/gpio.h>
#include <media/ov5650.h>
#include <linux/regulator/consumer.h>
#include <linux/err.h>
#include <linux/adt7461.h>
#include <generated/mach-types.h>

#include "gpio-names.h"

#define CAMERA1_PWDN_GPIO		TEGRA_GPIO_PT2
#define CAMERA1_RESET_GPIO		TEGRA_GPIO_PD2
#define CAMERA_AF_PD_GPIO		TEGRA_GPIO_PT3
#define CAMERA_FLASH_EN1_GPIO	TEGRA_GPIO_PBB4
#define CAMERA_FLASH_EN2_GPIO	TEGRA_GPIO_PA0

#define ADXL34X_IRQ_GPIO		TEGRA_GPIO_PAA1
#define ISL29018_IRQ_GPIO		TEGRA_GPIO_PK2
#define ADT7461_IRQ_GPIO		TEGRA_GPIO_PI2

extern void tegra_throttling_enable(bool enable);

static struct regulator *reg_avdd_cam1; /* LDO9 */
static struct regulator *reg_vdd_af;    /* LDO13 */
static struct regulator *reg_vdd_mipi;  /* LDO17 */
static struct regulator *reg_vddio_vi;  /* LDO18 */

static int whistler_camera_init(void)
{
	tegra_gpio_enable(CAMERA1_PWDN_GPIO);
	gpio_request(CAMERA1_PWDN_GPIO, "camera1_powerdown");
	gpio_direction_output(CAMERA1_PWDN_GPIO, 0);

	tegra_gpio_enable(CAMERA1_RESET_GPIO);
	gpio_request(CAMERA1_RESET_GPIO, "camera1_reset");
	gpio_direction_output(CAMERA1_RESET_GPIO, 0);

	tegra_gpio_enable(CAMERA_AF_PD_GPIO);
	gpio_request(CAMERA_AF_PD_GPIO, "camera_autofocus");
	gpio_direction_output(CAMERA_AF_PD_GPIO, 0);
	gpio_export(CAMERA_AF_PD_GPIO, false);

	tegra_gpio_enable(CAMERA_FLASH_EN1_GPIO);
	gpio_request(CAMERA_FLASH_EN1_GPIO, "camera_flash_en1");
	gpio_direction_output(CAMERA_FLASH_EN1_GPIO, 0);
	gpio_export(CAMERA_FLASH_EN1_GPIO, false);

	tegra_gpio_enable(CAMERA_FLASH_EN2_GPIO);
	gpio_request(CAMERA_FLASH_EN2_GPIO, "camera_flash_en2");
	gpio_direction_output(CAMERA_FLASH_EN2_GPIO, 0);
	gpio_export(CAMERA_FLASH_EN2_GPIO, false);

	gpio_set_value(CAMERA1_PWDN_GPIO, 1);
	mdelay(5);

	return 0;
}

static int whistler_ov5650_power_on(void)
{
	gpio_set_value(CAMERA1_PWDN_GPIO, 0);

	if (!reg_avdd_cam1) {
		reg_avdd_cam1 = regulator_get(NULL, "vdd_cam1");
		if (IS_ERR_OR_NULL(reg_avdd_cam1)) {
			pr_err("whistler_ov5650_power_on: vdd_cam1 failed\n");
			reg_avdd_cam1 = NULL;
			return PTR_ERR(reg_avdd_cam1);
		}
		regulator_enable(reg_avdd_cam1);
	}
	mdelay(5);
	if (!reg_vdd_mipi) {
		reg_vdd_mipi = regulator_get(NULL, "vddio_mipi");
		if (IS_ERR_OR_NULL(reg_vdd_mipi)) {
			pr_err("whistler_ov5650_power_on: vddio_mipi failed\n");
			reg_vdd_mipi = NULL;
			return PTR_ERR(reg_vdd_mipi);
		}
		regulator_enable(reg_vdd_mipi);
	}

	gpio_set_value(CAMERA1_RESET_GPIO, 1);
	mdelay(10);
	gpio_set_value(CAMERA1_RESET_GPIO, 0);
	mdelay(5);
	gpio_set_value(CAMERA1_RESET_GPIO, 1);
	mdelay(20);

	return 0;
}

static int whistler_ov5650_power_off(void)
{
	gpio_set_value(CAMERA1_PWDN_GPIO, 1);
	gpio_set_value(CAMERA1_RESET_GPIO, 0);

	if (reg_avdd_cam1) {
		regulator_disable(reg_avdd_cam1);
		regulator_put(reg_avdd_cam1);
		reg_avdd_cam1 = NULL;
	}

	if (reg_vdd_af) {
		regulator_disable(reg_vdd_af);
		regulator_put(reg_vdd_af);
		reg_vdd_af = NULL;
	}

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

static void whistler_adt7461_init(void)
{
	tegra_gpio_enable(ADT7461_IRQ_GPIO);
	gpio_request(ADT7461_IRQ_GPIO, "adt7461");
	gpio_direction_input(ADT7461_IRQ_GPIO);
}

static struct adt7461_platform_data whistler_adt7461_pdata = {
	.supported_hwrev = true,
	.ext_range = false,
	.therm2 = true,
	.conv_rate = 0x05,
	.offset = 0,
	.hysteresis = 0,
	.shutdown_ext_limit = 115,
	.shutdown_local_limit = 120,
	.throttling_ext_limit = 90,
	.alarm_fn = tegra_throttling_enable,
};

static struct i2c_board_info whistler_i2c4_board_info[] = {
	{
		I2C_BOARD_INFO("adt7461", 0x4C),
		.irq = TEGRA_GPIO_TO_IRQ(ADT7461_IRQ_GPIO),
		.platform_data = &whistler_adt7461_pdata,
	},
};

int __init whistler_sensors_init(void)
{
	whistler_camera_init();

	whistler_adxl34x_init();

	whistler_isl29018_init();

	whistler_adt7461_init();

	i2c_register_board_info(0, whistler_i2c1_board_info,
		ARRAY_SIZE(whistler_i2c1_board_info));

	i2c_register_board_info(4, whistler_i2c4_board_info,
		ARRAY_SIZE(whistler_i2c4_board_info));

	i2c_register_board_info(3, whistler_i2c3_board_info,
		ARRAY_SIZE(whistler_i2c3_board_info));

	return 0;
}

int __init whistler_sensor_late_init(void)
{
	int ret;

	if (!machine_is_whistler())
		return 0;

	reg_vddio_vi = regulator_get(NULL, "vddio_vi");
	if (IS_ERR_OR_NULL(reg_vddio_vi)) {
		pr_err("%s: Couldn't get regulator vddio_vi\n", __func__);
		return PTR_ERR(reg_vddio_vi);
	}

	/* set vddio_vi voltage to 1.8v */
	ret = regulator_set_voltage(reg_vddio_vi, 1800*1000, 1800*1000);
	if (ret) {
		pr_err("%s: Failed to set vddio_vi to 1.8v\n", __func__);
		goto fail_put_regulator;
	}

	regulator_put(reg_vddio_vi);
	return 0;

fail_put_regulator:
	regulator_put(reg_vddio_vi);
	return ret;
}

late_initcall(whistler_sensor_late_init);

