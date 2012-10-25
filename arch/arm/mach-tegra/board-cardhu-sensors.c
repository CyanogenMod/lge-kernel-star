/*
 * arch/arm/mach-tegra/board-cardhu-sensors.c
 *
 * Copyright (c) 2010-2011, NVIDIA CORPORATION, All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * Neither the name of NVIDIA CORPORATION nor the names of its contributors
 * may be used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
#include <media/ov14810.h>
#include <media/ov2710.h>
#include <media/tps61050.h>
#include <generated/mach-types.h>
#include "gpio-names.h"
#include "board.h"
#include <linux/mpu.h>
#include <media/sh532u.h>
#include <linux/bq27x00.h>
#include <mach/gpio.h>
#include <mach/edp.h>
#include <mach/thermal.h>

#include "gpio-names.h"
#include "board-cardhu.h"
#include "cpu-tegra.h"

static struct regulator *cardhu_1v8_cam1 = NULL;
static struct regulator *cardhu_1v8_cam2 = NULL;
static struct regulator *cardhu_1v8_cam3 = NULL;
static struct regulator *cardhu_vdd_2v8_cam1 = NULL;
static struct regulator *cardhu_vdd_2v8_cam2 = NULL;
static struct regulator *cardhu_vdd_cam3 = NULL;

static struct board_info board_info;

static struct pca954x_platform_mode cardhu_pca954x_modes[] = {
	{ .adap_id = PCA954x_I2C_BUS0, .deselect_on_exit = true, },
	{ .adap_id = PCA954x_I2C_BUS1, .deselect_on_exit = true, },
	{ .adap_id = PCA954x_I2C_BUS2, .deselect_on_exit = true, },
	{ .adap_id = PCA954x_I2C_BUS3, .deselect_on_exit = true, },
};

static struct pca954x_platform_data cardhu_pca954x_data = {
	.modes    = cardhu_pca954x_modes,
	.num_modes      = ARRAY_SIZE(cardhu_pca954x_modes),
};

static int cardhu_camera_init(void)
{
	int ret;

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

		tegra_gpio_enable(CAM2_POWER_DWN_GPIO);
		ret = gpio_request(CAM2_POWER_DWN_GPIO, "camera2_power_en");
		if (ret < 0)
			pr_err("%s: gpio_request failed for gpio %s\n",
				__func__, "CAM2_POWER_DWN_GPIO");

		tegra_gpio_enable(OV5650_RESETN_GPIO);
		ret = gpio_request(OV5650_RESETN_GPIO, "camera_reset");
		if (ret < 0)
			pr_err("%s: gpio_request failed for gpio %s\n",
				__func__, "OV5650_RESETN_GPIO");

		gpio_direction_output(CAM3_POWER_DWN_GPIO, 1);
		gpio_direction_output(CAM1_POWER_DWN_GPIO, 1);
		gpio_direction_output(CAM2_POWER_DWN_GPIO, 1);
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

static int cardhu_left_ov5650_power_on(void)
{
	/* Boards E1198 and E1291 are of Cardhu personality
	 * and donot have TCA6416 exp for camera */
	if ((board_info.board_id == BOARD_E1198) ||
		(board_info.board_id == BOARD_E1291)) {

		if (cardhu_vdd_2v8_cam1 == NULL) {
			cardhu_vdd_2v8_cam1 = regulator_get(NULL, "vdd_2v8_cam1");
			if (WARN_ON(IS_ERR(cardhu_vdd_2v8_cam1))) {
				pr_err("%s: couldn't get regulator vdd_2v8_cam1: %ld\n",
					__func__, PTR_ERR(cardhu_vdd_2v8_cam1));
				goto reg_alloc_fail;
			}
		}
		regulator_enable(cardhu_vdd_2v8_cam1);
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

	mdelay(5);
	if ((board_info.board_id == BOARD_E1198) ||
		(board_info.board_id == BOARD_E1291)) {
		gpio_direction_output(CAM1_POWER_DWN_GPIO, 0);
		mdelay(20);
		gpio_direction_output(OV5650_RESETN_GPIO, 0);
		mdelay(100);
		gpio_direction_output(OV5650_RESETN_GPIO, 1);
	}

	if (board_info.board_id == BOARD_PM269) {
		gpio_direction_output(CAM1_RST_L_GPIO, 0);
		mdelay(100);
		gpio_direction_output(CAM1_RST_L_GPIO, 1);
	}

	return 0;

reg_alloc_fail:
	if (cardhu_1v8_cam1) {
		regulator_put(cardhu_1v8_cam1);
		cardhu_1v8_cam1 = NULL;
	}
	if (cardhu_vdd_2v8_cam1) {
		regulator_put(cardhu_vdd_2v8_cam1);
		cardhu_vdd_2v8_cam1 = NULL;
	}

	return -ENODEV;

}

static int cardhu_left_ov5650_power_off(void)
{
	/* Boards E1198 and E1291 are of Cardhu personality
	 * and donot have TCA6416 exp for camera */
	if ((board_info.board_id == BOARD_E1198) ||
		(board_info.board_id == BOARD_E1291)) {
		gpio_direction_output(CAM1_POWER_DWN_GPIO, 1);
		gpio_direction_output(CAM2_POWER_DWN_GPIO, 1);
		gpio_direction_output(CAM3_POWER_DWN_GPIO, 1);
	}
	if (cardhu_1v8_cam1)
		regulator_disable(cardhu_1v8_cam1);
	if (cardhu_vdd_2v8_cam1)
		regulator_disable(cardhu_vdd_2v8_cam1);

	return 0;
}

struct ov5650_platform_data cardhu_left_ov5650_data = {
	.power_on = cardhu_left_ov5650_power_on,
	.power_off = cardhu_left_ov5650_power_off,
};

#ifdef CONFIG_VIDEO_OV14810
static int cardhu_ov14810_power_on(void)
{
	if (board_info.board_id == BOARD_E1198) {
		gpio_direction_output(CAM1_POWER_DWN_GPIO, 1);
		mdelay(20);
		gpio_direction_output(OV14810_RESETN_GPIO, 0);
		mdelay(100);
		gpio_direction_output(OV14810_RESETN_GPIO, 1);
	}

	return 0;
}

static int cardhu_ov14810_power_off(void)
{
	if (board_info.board_id == BOARD_E1198) {
		gpio_direction_output(CAM1_POWER_DWN_GPIO, 1);
		gpio_direction_output(CAM2_POWER_DWN_GPIO, 1);
		gpio_direction_output(CAM3_POWER_DWN_GPIO, 1);
	}

	return 0;
}

struct ov14810_platform_data cardhu_ov14810_data = {
	.power_on = cardhu_ov14810_power_on,
	.power_off = cardhu_ov14810_power_off,
};

struct ov14810_platform_data cardhu_ov14810uC_data = {
	.power_on = NULL,
	.power_off = NULL,
};

struct ov14810_platform_data cardhu_ov14810SlaveDev_data = {
	.power_on = NULL,
	.power_off = NULL,
};

static struct i2c_board_info cardhu_i2c_board_info_e1214[] = {
	{
		I2C_BOARD_INFO("ov14810", 0x36),
		.platform_data = &cardhu_ov14810_data,
	},
	{
		I2C_BOARD_INFO("ov14810uC", 0x67),
		.platform_data = &cardhu_ov14810uC_data,
	},
	{
		I2C_BOARD_INFO("ov14810SlaveDev", 0x69),
		.platform_data = &cardhu_ov14810SlaveDev_data,
	}
};
#endif

static int cardhu_right_ov5650_power_on(void)
{
	/* CSI-B and front sensor are muxed on cardhu */
	gpio_direction_output(CAMERA_CSI_MUX_SEL_GPIO, 0);

	/* Boards E1198 and E1291 are of Cardhu personality
	 * and donot have TCA6416 exp for camera */
	if ((board_info.board_id == BOARD_E1198) ||
		(board_info.board_id == BOARD_E1291)) {

		gpio_direction_output(CAM1_POWER_DWN_GPIO, 0);
		gpio_direction_output(CAM2_POWER_DWN_GPIO, 0);
		mdelay(10);

		if (cardhu_vdd_2v8_cam2 == NULL) {
			cardhu_vdd_2v8_cam2 = regulator_get(NULL, "vdd_2v8_cam2");
			if (WARN_ON(IS_ERR(cardhu_vdd_2v8_cam2))) {
				pr_err("%s: couldn't get regulator vdd_2v8_cam2: %ld\n",
					__func__, PTR_ERR(cardhu_vdd_2v8_cam2));
				goto reg_alloc_fail;
			}
		}
		regulator_enable(cardhu_vdd_2v8_cam2);
		mdelay(5);
	}

	/* Enable VDD_1V8_Cam2 */
	if (cardhu_1v8_cam2 == NULL) {
		cardhu_1v8_cam2 = regulator_get(NULL, "vdd_1v8_cam2");
		if (WARN_ON(IS_ERR(cardhu_1v8_cam2))) {
			pr_err("%s: couldn't get regulator vdd_1v8_cam2: %ld\n",
				__func__, PTR_ERR(cardhu_1v8_cam2));
			goto reg_alloc_fail;
		}
	}
	regulator_enable(cardhu_1v8_cam2);

	mdelay(5);

	if (board_info.board_id == BOARD_PM269) {
		gpio_direction_output(CAM2_RST_L_GPIO, 0);
		mdelay(100);
		gpio_direction_output(CAM2_RST_L_GPIO, 1);
	}

	return 0;

reg_alloc_fail:
	if (cardhu_1v8_cam2) {
		regulator_put(cardhu_1v8_cam2);
		cardhu_1v8_cam2 = NULL;
	}
	if (cardhu_vdd_2v8_cam2) {
		regulator_put(cardhu_vdd_2v8_cam2);
		cardhu_vdd_2v8_cam2 = NULL;
	}

	return -ENODEV;

}

static int cardhu_right_ov5650_power_off(void)
{
	/* CSI-B and front sensor are muxed on cardhu */
	gpio_direction_output(CAMERA_CSI_MUX_SEL_GPIO, 0);

	/* Boards E1198 and E1291 are of Cardhu personality
	 * and donot have TCA6416 exp for camera */
	if ((board_info.board_id == BOARD_E1198) ||
		(board_info.board_id == BOARD_E1291)) {
		gpio_direction_output(CAM1_POWER_DWN_GPIO, 1);
		gpio_direction_output(CAM2_POWER_DWN_GPIO, 1);
		gpio_direction_output(CAM3_POWER_DWN_GPIO, 1);
	}

	if (cardhu_1v8_cam2)
		regulator_disable(cardhu_1v8_cam2);
	if (cardhu_vdd_2v8_cam2)
		regulator_disable(cardhu_vdd_2v8_cam2);

	return 0;
}

static void cardhu_ov5650_synchronize_sensors(void)
{
	if (board_info.board_id == BOARD_E1198) {
		gpio_direction_output(CAM1_POWER_DWN_GPIO, 1);
		mdelay(50);
		gpio_direction_output(CAM1_POWER_DWN_GPIO, 0);
		mdelay(50);
	}
	else if (board_info.board_id == BOARD_E1291) {
		gpio_direction_output(CAM1_POWER_DWN_GPIO, 1);
		gpio_direction_output(CAM2_POWER_DWN_GPIO, 1);
		mdelay(50);
		gpio_direction_output(CAM1_POWER_DWN_GPIO, 0);
		gpio_direction_output(CAM2_POWER_DWN_GPIO, 0);
		mdelay(50);
	}
	else
		pr_err("%s: UnSupported BoardId\n", __func__);
}

struct ov5650_platform_data cardhu_right_ov5650_data = {
	.power_on = cardhu_right_ov5650_power_on,
	.power_off = cardhu_right_ov5650_power_off,
	.synchronize_sensors = cardhu_ov5650_synchronize_sensors,
};

static int cardhu_ov2710_power_on(void)
{
	/* CSI-B and front sensor are muxed on cardhu */
	gpio_direction_output(CAMERA_CSI_MUX_SEL_GPIO, 1);

	/* Boards E1198 and E1291 are of Cardhu personality
	 * and donot have TCA6416 exp for camera */
	if ((board_info.board_id == BOARD_E1198) ||
		(board_info.board_id == BOARD_E1291)) {

		gpio_direction_output(CAM1_POWER_DWN_GPIO, 0);
		gpio_direction_output(CAM2_POWER_DWN_GPIO, 0);
		gpio_direction_output(CAM3_POWER_DWN_GPIO, 0);
		mdelay(10);

		if (cardhu_vdd_cam3 == NULL) {
			cardhu_vdd_cam3 = regulator_get(NULL, "vdd_cam3");
			if (WARN_ON(IS_ERR(cardhu_vdd_cam3))) {
				pr_err("%s: couldn't get regulator vdd_cam3: %ld\n",
					__func__, PTR_ERR(cardhu_vdd_cam3));
				goto reg_alloc_fail;
			}
		}
		regulator_enable(cardhu_vdd_cam3);
	}

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

	return 0;

reg_alloc_fail:
	if (cardhu_1v8_cam3) {
		regulator_put(cardhu_1v8_cam3);
		cardhu_1v8_cam3 = NULL;
	}
	if (cardhu_vdd_cam3) {
		regulator_put(cardhu_vdd_cam3);
		cardhu_vdd_cam3 = NULL;
	}

	return -ENODEV;
}

static int cardhu_ov2710_power_off(void)
{
	/* CSI-B and front sensor are muxed on cardhu */
	gpio_direction_output(CAMERA_CSI_MUX_SEL_GPIO, 1);

	/* Boards E1198 and E1291 are of Cardhu personality
	 * and donot have TCA6416 exp for camera */
	if ((board_info.board_id == BOARD_E1198) ||
		(board_info.board_id == BOARD_E1291)) {
		gpio_direction_output(CAM1_POWER_DWN_GPIO, 1);
		gpio_direction_output(CAM2_POWER_DWN_GPIO, 1);
		gpio_direction_output(CAM3_POWER_DWN_GPIO, 1);
	}

	if (cardhu_1v8_cam3)
		regulator_disable(cardhu_1v8_cam3);
	if (cardhu_vdd_cam3)
		regulator_disable(cardhu_vdd_cam3);

	return 0;
}

struct ov2710_platform_data cardhu_ov2710_data = {
	.power_on = cardhu_ov2710_power_on,
	.power_off = cardhu_ov2710_power_off,
};

static const struct i2c_board_info cardhu_i2c3_board_info[] = {
	{
		I2C_BOARD_INFO("pca9546", 0x70),
		.platform_data = &cardhu_pca954x_data,
	},
};

static struct sh532u_platform_data sh532u_left_pdata = {
	.num		= 1,
	.sync		= 2,
	.dev_name	= "focuser",
	.gpio_reset	= TEGRA_GPIO_PBB0,
};

static struct sh532u_platform_data sh532u_right_pdata = {
	.num		= 2,
	.sync		= 1,
	.dev_name	= "focuser",
	.gpio_reset	= TEGRA_GPIO_PBB0,
};

static struct sh532u_platform_data pm269_sh532u_left_pdata = {
	.num		= 1,
	.sync		= 2,
	.dev_name	= "focuser",
	.gpio_reset	= CAM1_RST_L_GPIO,
};

static struct sh532u_platform_data pm269_sh532u_right_pdata = {
	.num		= 2,
	.sync		= 1,
	.dev_name	= "focuser",
	.gpio_reset	= CAM2_RST_L_GPIO,
};

static struct nvc_torch_pin_state cardhu_tps61050_pinstate = {
	.mask		= 0x0008, /*VGP3*/
	.values		= 0x0008,
};

static struct tps61050_platform_data cardhu_tps61050_pdata = {
	.dev_name	= "torch",
	.pinstate	= &cardhu_tps61050_pinstate,
};

static const struct i2c_board_info cardhu_i2c_board_info_tps61050[] = {
	{
		I2C_BOARD_INFO("tps61050", 0x33),
		.platform_data = &cardhu_tps61050_pdata,
	},
};

static struct i2c_board_info cardhu_i2c6_board_info[] = {
	{
		I2C_BOARD_INFO("ov5650L", 0x36),
		.platform_data = &cardhu_left_ov5650_data,
	},
	{
		I2C_BOARD_INFO("sh532u", 0x72),
		.platform_data = &sh532u_left_pdata,
	},
};

static struct i2c_board_info cardhu_i2c7_board_info[] = {
	{
		I2C_BOARD_INFO("ov5650R", 0x36),
		.platform_data = &cardhu_right_ov5650_data,
	},
	{
		I2C_BOARD_INFO("sh532u", 0x72),
		.platform_data = &sh532u_right_pdata,
	},
};

static struct i2c_board_info pm269_i2c6_board_info[] = {
	{
		I2C_BOARD_INFO("ov5650L", 0x36),
		.platform_data = &cardhu_left_ov5650_data,
	},
	{
		I2C_BOARD_INFO("sh532u", 0x72),
		.platform_data = &pm269_sh532u_left_pdata,
	},
};

static struct i2c_board_info pm269_i2c7_board_info[] = {
	{
		I2C_BOARD_INFO("ov5650R", 0x36),
		.platform_data = &cardhu_right_ov5650_data,
	},
	{
		I2C_BOARD_INFO("sh532u", 0x72),
		.platform_data = &pm269_sh532u_right_pdata,
	},
};

static struct i2c_board_info cardhu_i2c8_board_info[] = {
	{
		I2C_BOARD_INFO("ov2710", 0x36),
		.platform_data = &cardhu_ov2710_data,
	},
};

#ifndef CONFIG_TEGRA_INTERNAL_TSENSOR_EDP_SUPPORT
static int nct_get_temp(void *_data, long *temp)
{
	struct nct1008_data *data = _data;
	return nct1008_thermal_get_temp(data, temp);
}

static int nct_get_temp_low(void *_data, long *temp)
{
	struct nct1008_data *data = _data;
	return nct1008_thermal_get_temp_low(data, temp);
}

static int nct_set_limits(void *_data,
			long lo_limit_milli,
			long hi_limit_milli)
{
	struct nct1008_data *data = _data;
	return nct1008_thermal_set_limits(data,
					lo_limit_milli,
					hi_limit_milli);
}

static int nct_set_alert(void *_data,
				void (*alert_func)(void *),
				void *alert_data)
{
	struct nct1008_data *data = _data;
	return nct1008_thermal_set_alert(data, alert_func, alert_data);
}

static int nct_set_shutdown_temp(void *_data, long shutdown_temp)
{
	struct nct1008_data *data = _data;
	return nct1008_thermal_set_shutdown_temp(data, shutdown_temp);
}

static void nct1008_probe_callback(struct nct1008_data *data)
{
	struct tegra_thermal_device *thermal_device;

	thermal_device = kzalloc(sizeof(struct tegra_thermal_device),
					GFP_KERNEL);
	if (!thermal_device) {
		pr_err("unable to allocate thermal device\n");
		return;
	}

	thermal_device->name = "nct1008";
	thermal_device->data = data;
	thermal_device->offset = TDIODE_OFFSET;
	thermal_device->get_temp = nct_get_temp;
	thermal_device->get_temp_low = nct_get_temp_low;
	thermal_device->set_limits = nct_set_limits;
	thermal_device->set_alert = nct_set_alert;
	thermal_device->set_shutdown_temp = nct_set_shutdown_temp;

	tegra_thermal_set_device(thermal_device);
}
#endif

static struct nct1008_platform_data cardhu_nct1008_pdata = {
	.supported_hwrev = true,
	.ext_range = true,
	.conv_rate = 0x08,
	.offset = 8, /* 4 * 2C. Bug 844025 - 1C for device accuracies */
#ifndef CONFIG_TEGRA_INTERNAL_TSENSOR_EDP_SUPPORT
	.probe_callback = nct1008_probe_callback,
#endif
};

static struct i2c_board_info cardhu_i2c4_bq27510_board_info[] = {
	{
		I2C_BOARD_INFO("bq27510", 0x55),
	},
};

static struct i2c_board_info cardhu_i2c4_nct1008_board_info[] = {
	{
		I2C_BOARD_INFO("nct1008", 0x4C),
		.platform_data = &cardhu_nct1008_pdata,
		.irq = -1,
	}
};

static int cardhu_nct1008_init(void)
{
	int nct1008_port = -1;
	int ret = 0;

	if ((board_info.board_id == BOARD_E1198) ||
		(board_info.board_id == BOARD_E1291) ||
		(board_info.board_id == BOARD_E1257) ||
		(board_info.board_id == BOARD_PM269) ||
		(board_info.board_id == BOARD_PM305) ||
		(board_info.board_id == BOARD_PM311)) {
		nct1008_port = TEGRA_GPIO_PCC2;
	} else if ((board_info.board_id == BOARD_E1186) ||
		(board_info.board_id == BOARD_E1187) ||
		(board_info.board_id == BOARD_E1256)) {
		/* FIXME: seems to be conflicting with usb3 vbus on E1186 */
		/* nct1008_port = TEGRA_GPIO_PH7; */
	}

	if (nct1008_port >= 0) {
		/* FIXME: enable irq when throttling is supported */
		cardhu_i2c4_nct1008_board_info[0].irq = TEGRA_GPIO_TO_IRQ(nct1008_port);

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

#ifdef CONFIG_MPU_SENSORS_MPU3050
static struct mpu_platform_data mpu3050_data = {
	.int_config	= 0x10,
	.level_shifter	= 0,
	.orientation	= MPU_GYRO_ORIENTATION,	/* Located in board_[platformname].h	*/
};

static struct ext_slave_platform_data mpu3050_accel_data = {
	.address	= MPU_ACCEL_ADDR,
	.irq		= 0,
	.adapt_num	= MPU_ACCEL_BUS_NUM,
	.bus		= EXT_SLAVE_BUS_SECONDARY,
	.orientation	= MPU_ACCEL_ORIENTATION,	/* Located in board_[platformname].h	*/
};

static struct ext_slave_platform_data mpu_compass_data = {
	.address	= MPU_COMPASS_ADDR,
	.irq		= 0,
	.adapt_num	= MPU_COMPASS_BUS_NUM,
	.bus		= EXT_SLAVE_BUS_PRIMARY,
	.orientation	= MPU_COMPASS_ORIENTATION,	/* Located in board_[platformname].h	*/
};

static struct i2c_board_info __initdata inv_mpu_i2c2_board_info[] = {
	{
		I2C_BOARD_INFO(MPU_GYRO_NAME, MPU_GYRO_ADDR),
		.irq = TEGRA_GPIO_TO_IRQ(MPU_GYRO_IRQ_GPIO),
		.platform_data = &mpu3050_data,
	},
	{
		I2C_BOARD_INFO(MPU_ACCEL_NAME, MPU_ACCEL_ADDR),
#if	MPU_ACCEL_IRQ_GPIO
		.irq = TEGRA_GPIO_TO_IRQ(MPU_ACCEL_IRQ_GPIO),
#endif
		.platform_data = &mpu3050_accel_data,
	},
	{
		I2C_BOARD_INFO(MPU_COMPASS_NAME, MPU_COMPASS_ADDR),
#if	MPU_COMPASS_IRQ_GPIO
		.irq = TEGRA_GPIO_TO_IRQ(MPU_COMPASS_IRQ_GPIO),
#endif
		.platform_data = &mpu_compass_data,
	},
};

static void mpuirq_init(void)
{
	int ret = 0;

	pr_info("*** MPU START *** mpuirq_init...\n");

#if	MPU_ACCEL_IRQ_GPIO
	/* ACCEL-IRQ assignment */
	tegra_gpio_enable(MPU_ACCEL_IRQ_GPIO);
	ret = gpio_request(MPU_ACCEL_IRQ_GPIO, MPU_ACCEL_NAME);
	if (ret < 0) {
		pr_err("%s: gpio_request failed %d\n", __func__, ret);
		return;
	}

	ret = gpio_direction_input(MPU_ACCEL_IRQ_GPIO);
	if (ret < 0) {
		pr_err("%s: gpio_direction_input failed %d\n", __func__, ret);
		gpio_free(MPU_ACCEL_IRQ_GPIO);
		return;
	}
#endif

	/* MPU-IRQ assignment */
	tegra_gpio_enable(MPU_GYRO_IRQ_GPIO);
	ret = gpio_request(MPU_GYRO_IRQ_GPIO, MPU_GYRO_NAME);
	if (ret < 0) {
		pr_err("%s: gpio_request failed %d\n", __func__, ret);
		return;
	}

	ret = gpio_direction_input(MPU_GYRO_IRQ_GPIO);
	if (ret < 0) {
		pr_err("%s: gpio_direction_input failed %d\n", __func__, ret);
		gpio_free(MPU_GYRO_IRQ_GPIO);
		return;
	}
	pr_info("*** MPU END *** mpuirq_init...\n");

	i2c_register_board_info(MPU_GYRO_BUS_NUM, inv_mpu_i2c2_board_info,
		ARRAY_SIZE(inv_mpu_i2c2_board_info));
}
#endif


static struct i2c_board_info cardhu_i2c2_isl_board_info[] = {
	{
		I2C_BOARD_INFO("isl29028", 0x44),
	}
};

int __init cardhu_sensors_init(void)
{
	int err;

	tegra_get_board_info(&board_info);

	cardhu_camera_init();
	cam_tca6416_init();

	i2c_register_board_info(2, cardhu_i2c3_board_info,
		ARRAY_SIZE(cardhu_i2c3_board_info));

	i2c_register_board_info(2, cardhu_i2c_board_info_tps61050,
		ARRAY_SIZE(cardhu_i2c_board_info_tps61050));

#ifdef CONFIG_VIDEO_OV14810
	/* This is disabled by default; To enable this change Kconfig;
	 * there should be some way to detect dynamically which board
	 * is connected (E1211/E1214), till that time sensor selection
	 * logic is static;
	 * e1214 corresponds to ov14810 sensor */
	i2c_register_board_info(2, cardhu_i2c_board_info_e1214,
		ARRAY_SIZE(cardhu_i2c_board_info_e1214));
#else
	/* Left  camera is on PCA954x's I2C BUS0, Right camera is on BUS1 &
	 * Front camera is on BUS2 */
	if (board_info.board_id != BOARD_PM269) {
		i2c_register_board_info(PCA954x_I2C_BUS0,
					cardhu_i2c6_board_info,
					ARRAY_SIZE(cardhu_i2c6_board_info));

		i2c_register_board_info(PCA954x_I2C_BUS1,
					cardhu_i2c7_board_info,
					ARRAY_SIZE(cardhu_i2c7_board_info));
	} else {
		i2c_register_board_info(PCA954x_I2C_BUS0,
					pm269_i2c6_board_info,
					ARRAY_SIZE(pm269_i2c6_board_info));

		i2c_register_board_info(PCA954x_I2C_BUS1,
					pm269_i2c7_board_info,
					ARRAY_SIZE(pm269_i2c7_board_info));
	}
	i2c_register_board_info(PCA954x_I2C_BUS2, cardhu_i2c8_board_info,
		ARRAY_SIZE(cardhu_i2c8_board_info));

#endif
	pmu_tca6416_init();

	if (board_info.board_id == BOARD_E1291)
		i2c_register_board_info(4, cardhu_i2c4_bq27510_board_info,
			ARRAY_SIZE(cardhu_i2c4_bq27510_board_info));

	i2c_register_board_info(2, cardhu_i2c2_isl_board_info,
		ARRAY_SIZE(cardhu_i2c2_isl_board_info));

	err = cardhu_nct1008_init();
	if (err)
		return err;

	i2c_register_board_info(4, cardhu_i2c4_nct1008_board_info,
		ARRAY_SIZE(cardhu_i2c4_nct1008_board_info));

#ifdef CONFIG_MPU_SENSORS_MPU3050
	mpuirq_init();
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
	[2] = OV5650_GPIO("cam1_af_pwdn_lo", CAM1_AF_PWR_DN_L_GPIO, 0),
	[3] = OV5650_GPIO("cam1_ldo_shdn_lo", CAM1_LDO_SHUTDN_L_GPIO, 1),
	[4] = OV5650_GPIO("cam2_pwdn", CAM2_PWR_DN_GPIO, 0),
	[5] = OV5650_GPIO("cam2_rst_lo", CAM2_RST_L_GPIO, 1),
	[6] = OV5650_GPIO("cam2_af_pwdn_lo", CAM2_AF_PWR_DN_L_GPIO, 0),
	[7] = OV5650_GPIO("cam2_ldo_shdn_lo", CAM2_LDO_SHUTDN_L_GPIO, 1),
	[8] = OV5650_GPIO("cam3_pwdn", CAM_FRONT_PWR_DN_GPIO, 0),
	[9] = OV5650_GPIO("cam3_rst_lo", CAM_FRONT_RST_L_GPIO, 1),
	[10] = OV5650_GPIO("cam3_af_pwdn_lo", CAM_FRONT_AF_PWR_DN_L_GPIO, 0),
	[11] = OV5650_GPIO("cam3_ldo_shdn_lo", CAM_FRONT_LDO_SHUTDN_L_GPIO, 1),
	[12] = OV5650_GPIO("cam_led_exp", CAM_FRONT_LED_EXP, 1),
	[13] = OV5650_GPIO("cam_led_rear_exp", CAM_SNN_LED_REAR_EXP, 1),
	[14] = OV5650_GPIO("cam_i2c_mux_rst", CAM_I2C_MUX_RST_EXP, 1),
};

int __init cardhu_ov5650_late_init(void)
{
	int ret;
	int i;

	if (!machine_is_cardhu())
		return 0;

	if ((board_info.board_id == BOARD_E1198) ||
		(board_info.board_id == BOARD_E1291))
		return 0;

	printk("%s: \n", __func__);
	for (i = 0; i < ARRAY_SIZE(ov5650_gpio_keys); i++) {
		ret = gpio_request(ov5650_gpio_keys[i].gpio,
			ov5650_gpio_keys[i].name);
		if (ret < 0) {
			printk("%s: gpio_request failed for gpio #%d\n",
				__func__, i);
			goto fail;
		}
		printk("%s: enable - %d\n", __func__, i);
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
