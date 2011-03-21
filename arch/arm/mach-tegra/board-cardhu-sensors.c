/*
 * arch/arm/mach-tegra/board-cardhu-sensors.c
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
#include <linux/regulator/consumer.h>
#include <linux/i2c/pca954x.h>
#include <linux/i2c/pca953x.h>
#include <linux/nct1008.h>
#include <mach/fb.h>
#include <mach/gpio.h>
#include <media/ov5650.h>
#include <media/ov2710.h>
#include <generated/mach-types.h>
#include "gpio-names.h"
#include "board.h"
#include <linux/mpu.h>

#include <mach/gpio.h>

#include "gpio-names.h"
#include "board-cardhu.h"

static struct regulator *cardhu_1v8_cam1 = NULL;
static struct regulator *cardhu_1v8_cam3 = NULL;
static struct regulator *cardhu_avdd_dsi_csi = NULL;
static struct regulator *cardhu_vdd_2v8_cam1 = NULL;
static struct regulator *cardhu_vdd_cam3 = NULL;

#ifdef CONFIG_I2C_MUX_PCA954x
static struct pca954x_platform_mode cardhu_pca954x_modes[] = {
	{ .adap_id = PCA954x_I2C_BUS0, },
	{ .adap_id = PCA954x_I2C_BUS1, },
	{ .adap_id = PCA954x_I2C_BUS2, },
	{ .adap_id = PCA954x_I2C_BUS3, },
};

static struct pca954x_platform_data cardhu_pca954x_data = {
	.modes    = cardhu_pca954x_modes,
	.num_modes      = ARRAY_SIZE(cardhu_pca954x_modes),
};
#endif

static int cardhu_camera_init(void)
{
	int ret;
	struct board_info board_info;

	tegra_get_board_info(&board_info);
	/* Boards E1198 and E1291 are of Cardhu personality
	 * and donot have TCA6416 exp for camera */
	if ((board_info.board_id == BOARD_E1198) ||
		(board_info.board_id == BOARD_E1291)) {
		tegra_gpio_enable(CAM1_POWER_DWN_GPIO);
		ret = gpio_request(CAM1_POWER_DWN_GPIO, "camera_power_en");
		if (ret < 0)
			pr_err("%s: gpio_request failed for gpio %s\n",
				__func__, "CAM1_POWER_DWN_GPIO");

		tegra_gpio_enable(CAM3_POWER_DWN_GPIO);
		ret = gpio_request(CAM3_POWER_DWN_GPIO, "cam3_power_en");
		if (ret < 0)
			pr_err("%s: gpio_request failed for gpio %s\n",
				__func__, "CAM3_POWER_DWN_GPIO");

		tegra_gpio_enable(OV5650_RESETN_GPIO);
		ret = gpio_request(OV5650_RESETN_GPIO, "camera_reset");
		if (ret < 0)
			pr_err("%s: gpio_request failed for gpio %s\n",
				__func__, "OV5650_RESETN_GPIO");

		gpio_direction_output(CAM3_POWER_DWN_GPIO, 1);
		gpio_direction_output(CAM1_POWER_DWN_GPIO, 1);
		mdelay(10);

		gpio_direction_output(OV5650_RESETN_GPIO, 1);
		mdelay(5);
		gpio_direction_output(OV5650_RESETN_GPIO, 0);
		mdelay(5);
		gpio_direction_output(OV5650_RESETN_GPIO, 1);
		mdelay(5);

	}

	/* To select the CSIB MUX either for cam2 or cam3 */
	tegra_gpio_enable(CAMERA_CSI_MUX_SEL_GPIO);
	ret = gpio_request(CAMERA_CSI_MUX_SEL_GPIO, "camera_csi_sel");
	if (ret < 0)
		pr_err("%s: gpio_request failed for gpio %s\n",
			__func__, "CAMERA_CSI_MUX_SEL_GPIO");
	gpio_direction_output(CAMERA_CSI_MUX_SEL_GPIO, 0);
	gpio_export(CAMERA_CSI_MUX_SEL_GPIO, false);

	return 0;
}

static int cardhu_ov5650_power_on(void)
{
	struct board_info board_info;
	tegra_get_board_info(&board_info);
	/* Boards E1198 and E1291 are of Cardhu personality
	 * and donot have TCA6416 exp for camera */
	if ((board_info.board_id == BOARD_E1198) ||
		(board_info.board_id == BOARD_E1291)) {

		gpio_direction_output(CAM3_POWER_DWN_GPIO, 0);
		gpio_direction_output(CAM1_POWER_DWN_GPIO, 0);
		mdelay(10);

		if (cardhu_vdd_2v8_cam1 == NULL) {
			cardhu_vdd_2v8_cam1 = regulator_get(NULL, "vdd_2v8_cam1");
			if (WARN_ON(IS_ERR(cardhu_vdd_2v8_cam1))) {
				pr_err("%s: couldn't get regulator vdd_2v8_cam1: %ld\n",
					__func__, PTR_ERR(cardhu_vdd_2v8_cam1));
				goto reg_alloc_fail;
			}
		}
		regulator_enable(cardhu_vdd_2v8_cam1);

		if (cardhu_vdd_cam3 == NULL) {
			cardhu_vdd_cam3 = regulator_get(NULL, "vdd_cam3");
			if (WARN_ON(IS_ERR(cardhu_vdd_cam3))) {
				pr_err("%s: couldn't get regulator vdd_cam3: %ld\n",
					__func__, PTR_ERR(cardhu_vdd_cam3));
				goto reg_alloc_fail;
			}
		}
		regulator_enable(cardhu_vdd_cam3);

		/* Enable VDD_1V8_Cam3 */
		if (cardhu_1v8_cam3 == NULL) {
			cardhu_1v8_cam3 = regulator_get(NULL, "vdd_1v8_cam3");
			if (WARN_ON(IS_ERR(cardhu_1v8_cam3))) {
				pr_err("%s: couldn't get regulator vdd_1v8_cam3: %ld\n",
					__func__, PTR_ERR(cardhu_1v8_cam3));
				goto reg_alloc_fail;
			}
		}
		regulator_enable(cardhu_1v8_cam3);

		mdelay(5);
	}

	/* Enable VDD_1V8_Cam1 */
	if (cardhu_1v8_cam1 == NULL) {
		cardhu_1v8_cam1 = regulator_get(NULL, "vdd_1v8_cam1");
		if (WARN_ON(IS_ERR(cardhu_1v8_cam1))) {
			pr_err("%s: couldn't get regulator vdd_1v8_cam1: %ld\n",
				__func__, PTR_ERR(cardhu_1v8_cam1));
			goto reg_alloc_fail;
		}
	}
	regulator_enable(cardhu_1v8_cam1);

	/* Enable AVDD_CSI_DSI */
	if (cardhu_avdd_dsi_csi == NULL) {
		cardhu_avdd_dsi_csi = regulator_get(NULL, "avdd_dsi_csi");
		if (WARN_ON(IS_ERR(cardhu_avdd_dsi_csi))) {
			pr_err("%s: couldn't get regulator avdd_dsi_csi: %ld\n",
				__func__, PTR_ERR(cardhu_avdd_dsi_csi));
			goto reg_alloc_fail;
		}
	}
	regulator_enable(cardhu_avdd_dsi_csi);
	mdelay(5);
	return 0;

reg_alloc_fail:
	if (cardhu_1v8_cam1) {
		regulator_put(cardhu_1v8_cam1);
		cardhu_1v8_cam1 = NULL;
	}
	if (cardhu_1v8_cam3) {
		regulator_put(cardhu_1v8_cam3);
		cardhu_1v8_cam3 = NULL;
	}
	if (cardhu_avdd_dsi_csi) {
		regulator_put(cardhu_avdd_dsi_csi);
		cardhu_avdd_dsi_csi = NULL;
	}
	if (cardhu_vdd_2v8_cam1) {
		regulator_put(cardhu_vdd_2v8_cam1);
		cardhu_vdd_2v8_cam1 = NULL;
	}
	if (cardhu_vdd_cam3) {
		regulator_put(cardhu_vdd_cam3);
		cardhu_vdd_cam3 = NULL;
	}
	return -ENODEV;

}

static int cardhu_ov5650_power_off(void)
{
	struct board_info board_info;
	tegra_get_board_info(&board_info);
	/* Boards E1198 and E1291 are of Cardhu personality
	 * and donot have TCA6416 exp for camera */
	if ((board_info.board_id == BOARD_E1198) ||
		(board_info.board_id == BOARD_E1291)) {
		gpio_direction_output(CAM1_POWER_DWN_GPIO, 1);
	}
	if (cardhu_1v8_cam1)
		regulator_disable(cardhu_1v8_cam1);
	if (cardhu_avdd_dsi_csi)
		regulator_disable(cardhu_avdd_dsi_csi);
	if (cardhu_vdd_2v8_cam1)
		regulator_disable(cardhu_vdd_2v8_cam1);

	return 0;
}

struct ov5650_platform_data cardhu_ov5650_data = {
	.power_on = cardhu_ov5650_power_on,
	.power_off = cardhu_ov5650_power_off,
};

static int cardhu_ov2710_power_on(void)
{
	cardhu_ov5650_power_on();
	gpio_direction_output(CAMERA_CSI_MUX_SEL_GPIO, 1);
	return 0;
}

static int cardhu_ov2710_power_off(void)
{
	cardhu_ov5650_power_off();
	return 0;
}

struct ov2710_platform_data cardhu_ov2710_data = {
	.power_on = cardhu_ov2710_power_on,
	.power_off = cardhu_ov2710_power_off,
};

static const struct i2c_board_info cardhu_i2c3_board_info[] = {
#ifdef CONFIG_I2C_MUX_PCA954x
	{
		I2C_BOARD_INFO("pca9546", 0x70),
		.platform_data = &cardhu_pca954x_data,
	},
#endif
};
static struct i2c_board_info cardhu_i2c6_board_info[] = {
	{
		I2C_BOARD_INFO("ov5650", 0x36),
		.platform_data = &cardhu_ov5650_data,
	},
	{
		I2C_BOARD_INFO("sh532u", 0x72),
	},
};

static struct i2c_board_info cardhu_i2c8_board_info[] = {
	{
		I2C_BOARD_INFO("ov2710", 0x36),
		.platform_data = &cardhu_ov2710_data,
	},
};

static struct nct1008_platform_data cardhu_nct1008_pdata = {
	.supported_hwrev = true,
	.ext_range = false,
	.conv_rate = 0x08,
	.offset = 0,
	.hysteresis = 5,
	.shutdown_ext_limit = 75,
	.shutdown_local_limit = 75,
	.throttling_ext_limit = 60,
	.alarm_fn = NULL,
};

static struct i2c_board_info cardhu_i2c4_board_info[] = {
	{
		I2C_BOARD_INFO("bq27510", 0x55),
	},
	{
		I2C_BOARD_INFO("nct1008", 0x4C),
		.platform_data = &cardhu_nct1008_pdata,
		.irq = -1,
	}
};

static int cardhu_nct1008_init(void)
{
	int nct1008_port = -1;
	struct board_info BoardInfo;
	int ret;

	tegra_get_board_info(&BoardInfo);
	if ((BoardInfo.board_id == BOARD_E1198) ||
		(BoardInfo.board_id == BOARD_E1291)) {
		nct1008_port = TEGRA_GPIO_PCC2;
	} else if ((BoardInfo.board_id == BOARD_E1186) ||
		(BoardInfo.board_id == BOARD_E1187)) {
		/* FIXME: seems to be conflicting with usb3 vbus on E1186 */
		/* nct1008_port = TEGRA_GPIO_PH7; */
	}

	if (nct1008_port >= 0) {
		/* FIXME: enable irq when throttling is supported */
		/* cardhu_i2c4_board_info[0].irq = */
		/* TEGRA_GPIO_TO_IRQ(nct1008_port); */

		ret = gpio_request(nct1008_port, "temp_alert");
		if (ret < 0)
			return ret;

		ret = gpio_direction_input(nct1008_port);
		if (ret < 0)
			gpio_free(nct1008_port);
		else
			tegra_gpio_enable(nct1008_port);

	}
	return ret;
}

#if defined(CONFIG_GPIO_PCA953X)
static struct pca953x_platform_data cardhu_pmu_tca6416_data = {
	.gpio_base      = PMU_TCA6416_GPIO_BASE,
};

static const struct i2c_board_info cardhu_i2c4_board_info_tca6416[] = {
	{
		I2C_BOARD_INFO("tca6416", 0x20),
		.platform_data = &cardhu_pmu_tca6416_data,
	},
};

static struct pca953x_platform_data cardhu_cam_tca6416_data = {
	.gpio_base      = CAM_TCA6416_GPIO_BASE,
};

static const struct i2c_board_info cardhu_i2c2_board_info_tca6416[] = {
	{
		I2C_BOARD_INFO("tca6416", 0x20),
		.platform_data = &cardhu_cam_tca6416_data,
	},
};

static int __init pmu_tca6416_init(void)
{
	struct board_info board_info;
	tegra_get_board_info(&board_info);
	if ((board_info.board_id == BOARD_E1198) ||
			(board_info.board_id == BOARD_E1291))
		return 0;

	pr_info("Registering pmu pca6416\n");
	i2c_register_board_info(4, cardhu_i2c4_board_info_tca6416,
		ARRAY_SIZE(cardhu_i2c4_board_info_tca6416));
	return 0;
}

static int __init cam_tca6416_init(void)
{
	struct board_info board_info;
	tegra_get_board_info(&board_info);
	/* Boards E1198 and E1291 are of Cardhu personality
	 * and donot have TCA6416 exp for camera */
	if ((board_info.board_id == BOARD_E1198) ||
		(board_info.board_id == BOARD_E1291))
		return 0;

	pr_info("Registering cam pca6416\n");
	i2c_register_board_info(2, cardhu_i2c2_board_info_tca6416,
		ARRAY_SIZE(cardhu_i2c2_board_info_tca6416));
	return 0;
}
#else
static int __init pmu_tca6416_init(void)
{
	return 0;
}

static int __init cam_tca6416_init(void)
{
	return 0;
}
#endif

#ifdef CONFIG_SENSORS_MPU3050
#define SENSOR_MPU_NAME "mpu3050"
static struct mpu3050_platform_data mpu3050_data = {
	.int_config  = 0x10,
	.orientation = { 0, -1, 0, -1, 0, 0, 0, 0, -1 },  /* Orientation matrix for MPU on cardhu */
	.level_shifter = 0,

	.accel = {
	.get_slave_descr = get_accel_slave_descr,
	.adapt_num   = 2,
	.bus         = EXT_SLAVE_BUS_SECONDARY,
	.address     = 0x0F,
	.orientation = { 0, -1, 0, -1, 0, 0, 0, 0, -1 },  /* Orientation matrix for Kionix on cardhu */
	},

	.compass = {
	.get_slave_descr = get_compass_slave_descr,
	.adapt_num   = 2,
	.bus         = EXT_SLAVE_BUS_PRIMARY,
	.address     = 0x0C,
	.orientation = { 1, 0, 0, 0, -1, 0, 0, 0, -1 },  /* Orientation matrix for AKM on cardhu */
	},
};

static struct i2c_board_info __initdata mpu3050_i2c0_boardinfo[] = {
	{
		I2C_BOARD_INFO(SENSOR_MPU_NAME, 0x68),
		.irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PX1),
		.platform_data = &mpu3050_data,
	},
};

static void cardhu_mpuirq_init(void)
{
	pr_info("*** MPU START *** cardhu_mpuirq_init...\n");
	tegra_gpio_enable(TEGRA_GPIO_PX1);
	gpio_request(TEGRA_GPIO_PX1, SENSOR_MPU_NAME);
	gpio_direction_input(TEGRA_GPIO_PX1);
	pr_info("*** MPU END *** cardhu_mpuirq_init...\n");
}
#endif


int __init cardhu_sensors_init(void)
{
	int err;

	cardhu_camera_init();
	cam_tca6416_init();

	i2c_register_board_info(2, cardhu_i2c3_board_info,
		ARRAY_SIZE(cardhu_i2c3_board_info));

	/* CAM-A is on BUS0 to set CAM-B need to change
	 * PCA954x_I2C_BUS0 to PCA954x_I2C_BUS1 */
	i2c_register_board_info(PCA954x_I2C_BUS0, cardhu_i2c6_board_info,
		ARRAY_SIZE(cardhu_i2c6_board_info));

	i2c_register_board_info(PCA954x_I2C_BUS2, cardhu_i2c8_board_info,
		ARRAY_SIZE(cardhu_i2c8_board_info));

	pmu_tca6416_init();

	i2c_register_board_info(4, cardhu_i2c4_board_info,
		ARRAY_SIZE(cardhu_i2c4_board_info));

	err = cardhu_nct1008_init();
	if (err)
		return err;

#ifdef CONFIG_SENSORS_MPU3050
	cardhu_mpuirq_init();
#endif

	if (ARRAY_SIZE(cardhu_i2c3_board_info))
		i2c_register_board_info(3, cardhu_i2c3_board_info,
			ARRAY_SIZE(cardhu_i2c3_board_info));

#ifdef CONFIG_SENSORS_MPU3050
	i2c_register_board_info(2, mpu3050_i2c0_boardinfo,
		ARRAY_SIZE(mpu3050_i2c0_boardinfo));
#endif

	return 0;
}

#if defined(CONFIG_GPIO_PCA953X)
struct ov5650_gpios {
	const char *name;
	int gpio;
	int enabled;
};

#define OV5650_GPIO(_name, _gpio, _enabled)		\
	{						\
		.name = _name,				\
		.gpio = _gpio,				\
		.enabled = _enabled,			\
	}

static struct ov5650_gpios ov5650_gpio_keys[] = {
	[0] = OV5650_GPIO("cam1_pwdn", CAM1_PWR_DN_GPIO, 0),
	[1] = OV5650_GPIO("cam1_rst_lo", CAM1_RST_L_GPIO, 1),
	[2] = OV5650_GPIO("cam1_af_pwdn_lo", CAM1_AF_PWR_DN_L_GPIO, 1),
	[3] = OV5650_GPIO("cam1_ldo_shdn_lo", CAM1_LDO_SHUTDN_L_GPIO, 1),
	[4] = OV5650_GPIO("cam2_pwdn", CAM2_PWR_DN_GPIO, 0),
	[5] = OV5650_GPIO("cam2_rst_lo", CAM2_RST_L_GPIO, 1),
	[6] = OV5650_GPIO("cam2_af_pwdn_lo", CAM2_AF_PWR_DN_L_GPIO, 1),
	[7] = OV5650_GPIO("cam2_ldo_shdn_lo", CAM2_LDO_SHUTDN_L_GPIO, 1),
	[8] = OV5650_GPIO("cam3_pwdn", CAM_FRONT_PWR_DN_GPIO, 0),
	[9] = OV5650_GPIO("cam3_rst_lo", CAM_FRONT_RST_L_GPIO, 1),
	[10] = OV5650_GPIO("cam3_af_pwdn_lo", CAM_FRONT_AF_PWR_DN_L_GPIO, 1),
	[11] = OV5650_GPIO("cam3_ldo_shdn_lo", CAM_FRONT_LDO_SHUTDN_L_GPIO, 1),
	[12] = OV5650_GPIO("cam_led_exp", CAM_FRONT_LED_EXP, 1),
	[13] = OV5650_GPIO("cam_led_rear_exp", CAM_SNN_LED_REAR_EXP, 1),
	[14] = OV5650_GPIO("cam_i2c_mux_rst", CAM_I2C_MUX_RST_EXP, 1),
};

int __init cardhu_ov5650_late_init(void)
{
	int ret;
	int i;

	for (i = 0; i < ARRAY_SIZE(ov5650_gpio_keys); i++) {
		ret = gpio_request(ov5650_gpio_keys[i].gpio,
			ov5650_gpio_keys[i].name);
		if (ret < 0) {
			printk("%s: gpio_request failed for gpio #%d\n",
				__func__, i);
			goto fail;
		}
		gpio_direction_output(ov5650_gpio_keys[i].gpio,
			ov5650_gpio_keys[i].enabled);
		gpio_export(ov5650_gpio_keys[i].gpio, false);
	}

	return 0;

fail:
	while (i--)
		gpio_free(ov5650_gpio_keys[i].gpio);
	return ret;
}

late_initcall(cardhu_ov5650_late_init);
#endif
