/*
 * arch/arm/mach-tegra/board-ventana-sensors.c
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
#include <linux/i2c/nct1008.h>
#include <linux/akm8975.h>

#include "gpio-names.h"
#include <linux/i2c/pca954x.h>
#include <linux/i2c/pca953x.h>
#include <media/ov5650.h>

#define ISL29018_IRQ_GPIO	TEGRA_GPIO_PZ2
#define AKM8975_IRQ_GPIO	TEGRA_GPIO_PN5

#define CAMERA_POWER_GPIO	TEGRA_GPIO_PV4
#define CAMERA_CSI_MUX_SEL_GPIO	TEGRA_GPIO_PBB4

#define AC_PRESENT_GPIO		TEGRA_GPIO_PV3

static int ventana_camera_init(void)
{
	tegra_gpio_enable(CAMERA_POWER_GPIO);
	gpio_request(CAMERA_POWER_GPIO, "camera_power_en");
	gpio_direction_output(CAMERA_POWER_GPIO, 1);
	gpio_export(CAMERA_POWER_GPIO, false);

	tegra_gpio_enable(CAMERA_CSI_MUX_SEL_GPIO);
	gpio_request(CAMERA_CSI_MUX_SEL_GPIO, "camera_csi_sel");
	gpio_direction_output(CAMERA_CSI_MUX_SEL_GPIO, 0);
	gpio_export(CAMERA_CSI_MUX_SEL_GPIO, false);

	return 0;
}

static int ventana_ov5650_power_on(void)
{
	return 0;
}

static int ventana_ov5650_power_off(void)
{
	return 0;
}

struct ov5650_platform_data ventana_ov5650_data = {
	.power_on = ventana_ov5650_power_on,
	.power_off = ventana_ov5650_power_off,
};

static void ventana_isl29018_init(void)
{
	tegra_gpio_enable(ISL29018_IRQ_GPIO);
	gpio_request(ISL29018_IRQ_GPIO, "isl29018");
	gpio_direction_input(ISL29018_IRQ_GPIO);
}

static void ventana_akm8975_init(void)
{
	tegra_gpio_enable(AKM8975_IRQ_GPIO);
	gpio_request(AKM8975_IRQ_GPIO, "akm8975");
	gpio_direction_input(AKM8975_IRQ_GPIO);
}

static void ventana_bq20z75_init(void)
{
	tegra_gpio_enable(AC_PRESENT_GPIO);
	gpio_request(AC_PRESENT_GPIO, "ac_present");
	gpio_direction_input(AC_PRESENT_GPIO);
}

struct nct1008_platform_data ventana_nct1008_pdata = {
	.conv_rate = 5,
	.config = NCT1008_CONFIG_ALERT_DISABLE,
	.thermal_threshold = 110,
};

static const struct i2c_board_info ventana_i2c0_board_info[] = {
	{
		I2C_BOARD_INFO("isl29018", 0x44),
		.irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PZ2),
	},
};

static const struct i2c_board_info ventana_i2c2_board_info[] = {
	{
		I2C_BOARD_INFO("bq20z75-battery", 0x0B),
		.irq = TEGRA_GPIO_TO_IRQ(AC_PRESENT_GPIO),
	},
};

static struct pca953x_platform_data ventana_tca6416_data = {
	.gpio_base      = TEGRA_NR_GPIOS + 4, /* 4 gpios are already requested by tps6586x */
};

static struct pca954x_platform_mode ventana_pca9546_modes[] = {
	{ .adap_id = 6, }, /* REAR CAM1 */
	{ .adap_id = 7, }, /* REAR CAM2 */
	{ .adap_id = 8, }, /* FRONT CAM3 */
};

static struct pca954x_platform_data ventana_pca9546_data = {
	.modes          = ventana_pca9546_modes,
	.num_modes      = ARRAY_SIZE(ventana_pca9546_modes),
};

static const struct i2c_board_info ventana_i2c3_board_info[] = {
	{
		I2C_BOARD_INFO("tca6416", 0x20),
		.platform_data = &ventana_tca6416_data,
	},
	{
		I2C_BOARD_INFO("pca9546", 0x70),
		.platform_data = &ventana_pca9546_data,
	},
};

static struct i2c_board_info ventana_i2c4_board_info[] = {
	{
		I2C_BOARD_INFO("nct1008", 0x4C),
		.platform_data = &ventana_nct1008_pdata,
	},
	{
		I2C_BOARD_INFO("akm8975", 0x0C),
		.irq = TEGRA_GPIO_TO_IRQ(AKM8975_IRQ_GPIO),
	},
};

static struct i2c_board_info ventana_i2c7_board_info[] = {
	{
		I2C_BOARD_INFO("ov5650", 0x36),
		.platform_data = &ventana_ov5650_data,
	},
};

int __init ventana_sensors_init(void)
{
	ventana_isl29018_init();
	ventana_akm8975_init();
	ventana_camera_init();
	ventana_bq20z75_init();

	i2c_register_board_info(0, ventana_i2c0_board_info,
		ARRAY_SIZE(ventana_i2c0_board_info));

	i2c_register_board_info(2, ventana_i2c2_board_info,
		ARRAY_SIZE(ventana_i2c2_board_info));

	i2c_register_board_info(3, ventana_i2c3_board_info,
		ARRAY_SIZE(ventana_i2c3_board_info));

	i2c_register_board_info(4, ventana_i2c4_board_info,
		ARRAY_SIZE(ventana_i2c4_board_info));

	i2c_register_board_info(7, ventana_i2c7_board_info,
		ARRAY_SIZE(ventana_i2c7_board_info));

	return 0;
}
