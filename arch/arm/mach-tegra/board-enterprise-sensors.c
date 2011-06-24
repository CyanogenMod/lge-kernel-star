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
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/mpu.h>
#include <linux/nct1008.h>
#include <linux/regulator/consumer.h>

#include <mach/gpio.h>

#include <media/ar0832_main.h>
#include "cpu-tegra.h"
#include "gpio-names.h"
#include "board-enterprise.h"

static struct nct1008_platform_data enterprise_nct1008_pdata = {
	.supported_hwrev = true,
	.ext_range = true,
	.conv_rate = 0x08,
	.offset = 0,
	.hysteresis = 5,
	.shutdown_ext_limit = 75,
	.shutdown_local_limit = 75,
	.throttling_ext_limit = 90,
	.alarm_fn = tegra_throttling_enable,
};

static struct i2c_board_info enterprise_i2c4_nct1008_board_info[] = {
	{
		I2C_BOARD_INFO("nct1008", 0x4C),
		.irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PH7),
		.platform_data = &enterprise_nct1008_pdata,
	}
};

struct enterprise_power_rail {
	struct regulator *cam_reg;
	struct regulator *csi_reg;
};

static void enterprise_nct1008_init(void)
{
	int ret;

	tegra_gpio_enable(TEGRA_GPIO_PH7);
	ret = gpio_request(TEGRA_GPIO_PH7, "temp_alert");
	if (ret < 0) {
		pr_err("%s: gpio_request failed %d\n", __func__, ret);
		return;
	}

	ret = gpio_direction_input(TEGRA_GPIO_PH7);
	if (ret < 0) {
		pr_err("%s: gpio_direction_input failed %d\n", __func__, ret);
		gpio_free(TEGRA_GPIO_PH7);
		return;
	}

	i2c_register_board_info(4, enterprise_i2c4_nct1008_board_info,
				ARRAY_SIZE(enterprise_i2c4_nct1008_board_info));
}

#define SENSOR_MPU_NAME "mpu3050"
static struct mpu3050_platform_data mpu3050_data = {
	.int_config  = 0x10,
	/* Orientation matrix for MPU on enterprise */
	.orientation = { -1, 0, 0, 0, -1, 0, 0, 0, 1 },
	.level_shifter = 0,

	.accel = {
		.get_slave_descr = get_accel_slave_descr,
		.adapt_num   = 0,
		.bus         = EXT_SLAVE_BUS_SECONDARY,
		.address     = 0x0F,
		/* Orientation matrix for Kionix on enterprise */
		.orientation = { 0, 1, 0, -1, 0, 0, 0, 0, 1 },

	},

	.compass = {
		.get_slave_descr = get_compass_slave_descr,
		.adapt_num   = 0,
		.bus         = EXT_SLAVE_BUS_PRIMARY,
		.address     = 0x0C,
		/* Orientation matrix for AKM on enterprise */
		.orientation = { 0, 1, 0, -1, 0, 0, 0, 0, 1 },
	},
};

static struct i2c_board_info __initdata mpu3050_i2c0_boardinfo[] = {
	{
		I2C_BOARD_INFO(SENSOR_MPU_NAME, 0x68),
		.irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PH4),
		.platform_data = &mpu3050_data,
	},
};

static inline void enterprise_msleep(u32 t)
{
	/*
	If timer value is between ( 10us - 20ms),
	usleep_range() is recommended.
	Please read Documentation/timers/timers-howto.txt.
	*/
	usleep_range(t*1000, t*1000 + 500);
}

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

static int enterprise_ar0832_power_on(struct enterprise_power_rail *prail)
{
	int ret = 0;

	pr_info("%s: ++\n", __func__);

	if (!prail->csi_reg) {
		prail->csi_reg = regulator_get(NULL, "avdd_dsi_csi");
		if (IS_ERR_OR_NULL(prail->csi_reg)) {
			pr_err("%s: failed to get csi pwr\n", __func__);
			return PTR_ERR(prail->csi_reg);
		}
	}
	ret = regulator_enable(prail->csi_reg);
	if (ret) {
		pr_err("%s: failed to enable csi pwr\n", __func__);
		goto fail_regulator_csi_reg;
	}

	if (!prail->cam_reg) {
		prail->cam_reg = regulator_get(NULL, "vddio_cam");
		if (IS_ERR_OR_NULL(prail->cam_reg)) {
			pr_err("%s: failed to get cam pwr\n", __func__);
			ret = PTR_ERR(prail->cam_reg);
			goto fail_regulator_csi_reg;
		}
	}
	ret = regulator_enable(prail->cam_reg);
	if (ret) {
		pr_err("%s: failed to enable cam pwr\n", __func__);
		goto fail_regulator_cam_reg;
	}

	return 0;

fail_regulator_cam_reg:
	regulator_put(prail->cam_reg);
	prail->cam_reg = NULL;
fail_regulator_csi_reg:
	regulator_put(prail->csi_reg);
	prail->csi_reg = NULL;
	return ret;
}

static struct enterprise_power_rail enterprise_ar0832_power_rail;

static int enterprise_ar0832_ri_power_on(int is_stereo)
{
	int ret = 0;

	pr_info("%s: ++\n", __func__);
	ret = enterprise_ar0832_power_on(&enterprise_ar0832_power_rail);

	/* Release Reset */
	if (is_stereo) {
		gpio_set_value(CAM1_RST_L_GPIO, 1);
		gpio_set_value(CAM2_RST_L_GPIO, 1);
	} else
		gpio_set_value(CAM1_RST_L_GPIO, 1);
	/*
	It takes 2400 EXTCLK for ar0832 to be ready for I2c.
	EXTCLK is 10 ~ 24MHz. 1 ms should be enough to cover
	at least 2400 EXTCLK within frequency range.
	*/
	enterprise_msleep(1);

	return ret;
}

static int enterprise_ar0832_le_power_on(int is_stereo)
{
	int ret = 0;

	pr_info("%s: ++\n", __func__);
	ret = enterprise_ar0832_power_on(&enterprise_ar0832_power_rail);

	/* Release Reset */
	gpio_set_value(CAM2_RST_L_GPIO, 1);

	/*
	It takes 2400 EXTCLK for ar0832 to be ready for I2c.
	EXTCLK is 10 ~ 24MHz. 1 ms should be enough to cover
	at least 2400 EXTCLK within frequency range.
	*/
	enterprise_msleep(1);

	/* CSI B is shared between Front camera and Rear Left camera */
	gpio_set_value(CAM_CSI_MUX_SEL_GPIO, 1);

	return ret;
}

static int enterprise_ar0832_power_off(struct enterprise_power_rail *prail)
{
	if (prail->cam_reg)
		regulator_disable(prail->cam_reg);

	if (prail->csi_reg)
		regulator_disable(prail->csi_reg);

	return 0;
}

static int enterprise_ar0832_ri_power_off(int is_stereo)
{
	int ret;

	pr_info("%s: ++\n", __func__);
	ret = enterprise_ar0832_power_off(&enterprise_ar0832_power_rail);

	/* Assert Reset */
	if (is_stereo) {
		gpio_set_value(CAM1_RST_L_GPIO, 0);
		gpio_set_value(CAM2_RST_L_GPIO, 0);
	} else
		gpio_set_value(CAM1_RST_L_GPIO, 0);

	return ret;
}

static int enterprise_ar0832_le_power_off(int is_stereo)
{
	int ret;

	pr_info("%s: ++\n", __func__);
	ret = enterprise_ar0832_power_off(&enterprise_ar0832_power_rail);

	/* Assert Reset */
	gpio_set_value(CAM2_RST_L_GPIO, 0);

	return ret;
}

struct enterprise_cam_gpio {
	int gpio;
	const char *label;
	int value;
};

#define TEGRA_CAMERA_GPIO(_gpio, _label, _value)	\
	{						\
		.gpio = _gpio,				\
		.label = _label,			\
		.value = _value,			\
	}

static struct enterprise_cam_gpio enterprise_cam_gpio_data[] = {
	[0] = TEGRA_CAMERA_GPIO(CAM_CSI_MUX_SEL_GPIO, "cam_csi_sel", 1),
	[1] = TEGRA_CAMERA_GPIO(CAM1_RST_L_GPIO, "cam1_rst_lo", 0),
	[2] = TEGRA_CAMERA_GPIO(CAM2_RST_L_GPIO, "cam2_rst_lo", 0),
	[3] = TEGRA_CAMERA_GPIO(CAM3_RST_L_GPIO, "cam3_rst_lo", 0),
	[4] = TEGRA_CAMERA_GPIO(CAM3_PWDN_GPIO, "cam3_pwdn", 1),
};

struct ar0832_platform_data enterprise_ar0832_ri_data = {
	.power_on = enterprise_ar0832_ri_power_on,
	.power_off = enterprise_ar0832_ri_power_off,
	.id = "right",
};

struct ar0832_platform_data enterprise_ar0832_le_data = {
	.power_on = enterprise_ar0832_le_power_on,
	.power_off = enterprise_ar0832_le_power_off,
	.id = "left",
};

/*
 * Since ar0832 driver should support multiple devices, slave
 * address should be changed after it is open. Default slave
 * address of ar0832 is 0x36. It will be changed to alternate
 * address defined below when device is open.
 */
static struct i2c_board_info ar0832_i2c2_boardinfo[] = {
	{
		/* 0x30: alternative slave address */
		I2C_BOARD_INFO("ar0832", 0x36),
		.platform_data = &enterprise_ar0832_ri_data,
	},
	{
		/* 0x31: alternative slave address */
		I2C_BOARD_INFO("ar0832", 0x32),
		.platform_data = &enterprise_ar0832_le_data,
	},
};

static int enterprise_cam_init(void)
{
	int ret;
	int i;

	pr_info("%s:++\n", __func__);

	for (i = 0; i < ARRAY_SIZE(enterprise_cam_gpio_data); i++) {
		ret = gpio_request(enterprise_cam_gpio_data[i].gpio,
				   enterprise_cam_gpio_data[i].label);
		if (ret < 0) {
			pr_err("%s: gpio_request failed for gpio #%d\n",
				__func__, i);
			goto fail_free_gpio;
		}
		gpio_direction_output(enterprise_cam_gpio_data[i].gpio,
				      enterprise_cam_gpio_data[i].value);
		gpio_export(enterprise_cam_gpio_data[i].gpio, false);
		tegra_gpio_enable(enterprise_cam_gpio_data[i].gpio);
	}

	i2c_register_board_info(2, ar0832_i2c2_boardinfo,
		ARRAY_SIZE(ar0832_i2c2_boardinfo));

	return 0;

fail_free_gpio:
	pr_err("%s enterprise_cam_init failed!\n", __func__);
	while (i--)
		gpio_free(enterprise_cam_gpio_data[i].gpio);
	return ret;
}

int __init enterprise_sensors_init(void)
{
	enterprise_isl_init();
	enterprise_nct1008_init();
	enterprise_mpuirq_init();
	enterprise_cam_init();

	return 0;
}