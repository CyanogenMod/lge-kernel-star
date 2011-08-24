/*
 * arch/arm/mach-tegra/board-enterprise-sensors.c
 *
 * Copyright (c) 2011, NVIDIA CORPORATION, All rights reserved.
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
#include <linux/err.h>
#include <linux/mpu.h>
#include <linux/nct1008.h>
#include <linux/regulator/consumer.h>
#include <mach/gpio.h>
#include <media/ar0832_main.h>
#include <media/tps61050.h>
#include <media/ov9726.h>
#include <mach/edp.h>
#include "cpu-tegra.h"
#include "gpio-names.h"
#include "board-enterprise.h"

static struct nct1008_platform_data enterprise_nct1008_pdata = {
	.supported_hwrev = true,
	.ext_range = true,
	.conv_rate = 0x08,
/*
 * BugID 844025 requires 11C guardband (9.7C for hotspot offset + 1.5C
 * for sensor accuracy). FIXME: Move sensor accuracy to sensor driver.
 */
	.offset = 11,
	.hysteresis = 5,
	.shutdown_ext_limit = 90,
	.shutdown_local_limit = 90,
	.throttling_ext_limit = 75,
	.alarm_fn = tegra_throttling_enable,
};

static struct i2c_board_info enterprise_i2c4_nct1008_board_info[] = {
	{
		I2C_BOARD_INFO("nct1008", 0x4C),
		.irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PH7),
		.platform_data = &enterprise_nct1008_pdata,
	}
};

static void enterprise_nct1008_init(void)
{
	int ret;
#ifdef CONFIG_TEGRA_EDP_LIMITS
	const struct tegra_edp_limits *z;
	int zones_sz;
	int i;
	bool throttle_ok = false;
#endif

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
#ifdef CONFIG_TEGRA_EDP_LIMITS
	tegra_get_cpu_edp_limits(&z, &zones_sz);
	zones_sz = min(zones_sz, MAX_ZONES);
	for (i = 0; i < zones_sz; i++) {
		enterprise_nct1008_pdata.thermal_zones[i] = z[i].temperature;
		if (enterprise_nct1008_pdata.thermal_zones[i] ==
		    enterprise_nct1008_pdata.throttling_ext_limit) {
			throttle_ok = true;
		}
	}

	if (throttle_ok != true)
		pr_warn("%s: WARNING! Throttling limit %dC would be inaccurate"
			" as it is NOT one of the EDP points\n",
			__func__, enterprise_nct1008_pdata.throttling_ext_limit);
	else
		pr_info("%s: Throttling limit %dC OK\n",
			__func__, enterprise_nct1008_pdata.throttling_ext_limit);

	enterprise_nct1008_pdata.thermal_zones_sz = zones_sz;
#endif
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

enum CAMERA_INDEX {
	CAM_REAR_LEFT,
	CAM_REAR_RIGHT,
	CAM_FRONT,
	NUM_OF_CAM
};

struct enterprise_power_rail {
	struct regulator *cam_reg;
	struct regulator *csi_reg;
};

static struct enterprise_power_rail ent_vicsi_pwr[NUM_OF_CAM];

static int enterprise_cam_pwr(enum CAMERA_INDEX cam, bool pwr_on)
{
	struct enterprise_power_rail *reg_cam = &ent_vicsi_pwr[cam];
	int ret = 0;

	/*
	* SW must turn on 1.8V first then 2.8V
	* SW must turn off 2.8V first then 1.8V
	*/
	if (pwr_on) {
		if (reg_cam->csi_reg == NULL) {
			reg_cam->csi_reg = regulator_get(NULL,
						"avdd_dsi_csi");
			if (IS_ERR_OR_NULL(reg_cam->csi_reg)) {
				pr_err("%s: csi pwr err\n", __func__);
				ret = PTR_ERR(reg_cam->csi_reg);
				goto enterprise_cam_pwr_fail;
			}
		}

		ret = regulator_enable(reg_cam->csi_reg);
		if (ret) {
			pr_err("%s: enable csi pwr err\n", __func__);
			goto enterprise_cam_pwr_fail;
		}

		if (reg_cam->cam_reg == NULL) {
			reg_cam->cam_reg = regulator_get(NULL,
						"vddio_cam");
			if (IS_ERR_OR_NULL(reg_cam->cam_reg)) {
				pr_err("%s: vddio pwr err\n", __func__);
				ret = PTR_ERR(reg_cam->cam_reg);
				regulator_disable(reg_cam->csi_reg);
				goto enterprise_cam_pwr_fail;
			}
		}

		ret = regulator_enable(reg_cam->cam_reg);
		if (ret) {
			pr_err("%s: enable vddio pwr err\n", __func__);
			regulator_disable(reg_cam->csi_reg);
			goto enterprise_cam_pwr_fail;
		}
	} else {
		if (reg_cam->cam_reg)
			regulator_disable(reg_cam->cam_reg);

		if (reg_cam->csi_reg)
			regulator_disable(reg_cam->csi_reg);
	}
	return 0;

enterprise_cam_pwr_fail:
	if (!IS_ERR_OR_NULL(reg_cam->cam_reg))
		regulator_put(reg_cam->cam_reg);
	reg_cam->cam_reg = NULL;

	if (!IS_ERR_OR_NULL(reg_cam->csi_reg))
		regulator_put(reg_cam->csi_reg);
	reg_cam->csi_reg = NULL;

	return ret;
}

static int enterprise_ar0832_ri_power_on(int is_stereo)
{
	int ret = 0;

	ret = enterprise_cam_pwr(CAM_REAR_RIGHT, true);

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
	ret = enterprise_cam_pwr(CAM_REAR_LEFT, true);

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

static int enterprise_ar0832_ri_power_off(int is_stereo)
{
	int ret;

	pr_info("%s: ++\n", __func__);
	ret = enterprise_cam_pwr(CAM_REAR_RIGHT, false);

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
	ret = enterprise_cam_pwr(CAM_REAR_LEFT, false);

	/* Assert Reset */
	gpio_set_value(CAM2_RST_L_GPIO, 0);

	return ret;
}

static int enterprise_ov9726_power_on(void)
{
	pr_info("ov9726 power on\n");

	/* switch mipi mux to front camera */
	gpio_set_value(CAM_CSI_MUX_SEL_GPIO, CAM_CSI_MUX_SEL_FRONT);
	enterprise_cam_pwr(CAM_FRONT, true);

	return 0;
}

static int enterprise_ov9726_power_off(void)
{
	pr_info("ov9726 power off\n");

	enterprise_cam_pwr(CAM_FRONT, false);

	return 0;
}

struct ov9726_platform_data enterprise_ov9726_data = {
	.power_on = enterprise_ov9726_power_on,
	.power_off = enterprise_ov9726_power_off,
	.gpio_rst = CAM3_RST_L_GPIO,
	.rst_low_active = true,
	.gpio_pwdn = CAM3_PWDN_GPIO,
	.pwdn_low_active = false,
};

static struct tps61050_pin_state enterprise_tps61050_pinstate = {
	.mask		= 0x0008, /*VGP3*/
	.values		= 0x0008,
};

/* I2C bus becomes active when vdd_1v8_cam is enabled */
static int enterprise_tps61050_pm(int pwr)
{
	static struct regulator *enterprise_flash_reg = NULL;
	int ret = 0;

	pr_info("%s: ++%d\n", __func__, pwr);
	switch (pwr) {
	case TPS61050_PWR_OFF:
		if (enterprise_flash_reg) {
			regulator_disable(enterprise_flash_reg);
			regulator_put(enterprise_flash_reg);
			enterprise_flash_reg = NULL;
		}
		break;
	case TPS61050_PWR_STDBY:
	case TPS61050_PWR_COMM:
	case TPS61050_PWR_ON:
		enterprise_flash_reg = regulator_get(NULL, "vdd_1v8_cam");
		if (IS_ERR_OR_NULL(enterprise_flash_reg)) {
			pr_err("%s: failed to get flash pwr\n", __func__);
			return PTR_ERR(enterprise_flash_reg);
		}
		ret = regulator_enable(enterprise_flash_reg);
		if (ret) {
			pr_err("%s: failed to enable flash pwr\n", __func__);
			goto fail_regulator_flash_reg;
		}
		enterprise_msleep(10);
		break;
	default:
		ret = -1;
	}
	return ret;

fail_regulator_flash_reg:
	regulator_put(enterprise_flash_reg);
	enterprise_flash_reg = NULL;
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
	[5] = TEGRA_CAMERA_GPIO(CAM_FLASH_EN_GPIO, "flash_en", 1),
};

static struct ar0832_platform_data enterprise_ar0832_ri_data = {
	.power_on = enterprise_ar0832_ri_power_on,
	.power_off = enterprise_ar0832_ri_power_off,
	.id = "right",
};

static struct ar0832_platform_data enterprise_ar0832_le_data = {
	.power_on = enterprise_ar0832_le_power_on,
	.power_off = enterprise_ar0832_le_power_off,
	.id = "left",
};

static struct tps61050_platform_data enterprise_tps61050_data = {
	.cfg		= 0,
	.num		= 1,
	.max_amp_torch	= CAM_FLASH_MAX_TORCH_AMP,
	.max_amp_flash	= CAM_FLASH_MAX_FLASH_AMP,
	.pinstate	= &enterprise_tps61050_pinstate,
	.init		= NULL,
	.exit		= NULL,
	.pm		= &enterprise_tps61050_pm,
	.gpio_envm	= NULL,
	.gpio_sync	= NULL,
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
	{
		I2C_BOARD_INFO("tps61050", 0x33),
		.platform_data = &enterprise_tps61050_data,
	},
	{
		I2C_BOARD_INFO("ov9726", OV9726_I2C_ADDR >> 1),
		.platform_data = &enterprise_ov9726_data,
	},
};

static int enterprise_cam_init(void)
{
	int ret;
	int i;

	pr_info("%s:++\n", __func__);

	memset(ent_vicsi_pwr, 0, sizeof(ent_vicsi_pwr));
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
	int ret;
	enterprise_isl_init();
	enterprise_nct1008_init();
	enterprise_mpuirq_init();
	ret = enterprise_cam_init();

	return ret;
}

