/*
 * arch/arm/mach-tegra/board-star-sensors.c
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

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <mach/gpio.h>
#include <media/ov5650.h>
#include <media/soc380.h>
#include <linux/regulator/consumer.h>
#include <linux/err.h>
#include <linux/adt7461.h>
#include <generated/mach-types.h>
#include <linux/gpio.h>
#include <linux/i2c/pca953x.h>
#include <linux/input/gp2a.h>
#include <mach/tegra_odm_fuses.h>
#include <linux/io.h>
#include <mach/iomap.h>
#include <mach/io.h>
//MOBII_CHNANGE_S 20120213 mg.chang@mobii.co.kr : Appy Sensor Porting
#if defined(CONFIG_MPU_SENSORS_MPU3050)
#include <linux/mpu.h>
#endif
//MOBII_CHNANGE_E 20120213 mg.chang@mobii.co.kr : Appy Sensor Porting


#include <mach-tegra/gpio-names.h>
#include <mach-tegra/cpu-tegra.h>


//LGE_CHANGE_S chen.yingchun 20111223
#if defined(CONFIG_REGULATOR_LP8720)
#include <linux/regulator/lp8720.h>
#endif

#if defined(CONFIG_VIDEO_IMX073)
#include<media/imx073.h>
#endif

#if defined(CONFIG_TORCH_AAT1270)
#include<media/aat1270.h>
#endif

#if defined(CONFIG_VIDEO_MT9M113)
#include<media/mt9m113.h>
#endif

//LGE_CHANGE_E chen.yingchun 20111223

#define CAMERA1_PWDN_GPIO		TEGRA_GPIO_PT2
#define CAMERA1_RESET_GPIO		TEGRA_GPIO_PD2
#define CAMERA2_PWDN_GPIO		TEGRA_GPIO_PBB5
#define CAMERA2_RESET_GPIO		TEGRA_GPIO_PBB1
#define CAMERA_AF_PD_GPIO		TEGRA_GPIO_PT3
#define CAMERA_FLASH_EN1_GPIO		TEGRA_GPIO_PBB4
#define CAMERA_FLASH_EN2_GPIO		TEGRA_GPIO_PA0

#define TCA6416_GPIO_BASE		(TEGRA_NR_GPIOS)
#define FUSE_POWER_EN_GPIO		(TCA6416_GPIO_BASE + 2)

#define ADXL34X_IRQ_GPIO		TEGRA_GPIO_PAA1
#define ISL29018_IRQ_GPIO		TEGRA_GPIO_PK2
#define ADT7461_IRQ_GPIO		TEGRA_GPIO_PI2

#define PROXI_OUT_GPIO			TEGRA_GPIO_PW2

//MOBII_CHNANGE_S 20120213 mg.chang@mobii.co.kr : Appy Sensor Porting
#if defined(CONFIG_MPU_SENSORS_MPU3050)
/* Invensense MPU Definitions */
#define MPU_GYRO_NAME		"mpu3050"
#define MPU_GYRO_IRQ_GPIO	TEGRA_GPIO_PQ5
#define MPU_GYRO_ADDR		0x68
#define MPU_GYRO_BUS_NUM	2

#if (defined(CONFIG_MACH_STAR_P990) || defined(CONFIG_MACH_STAR_P999))
#define MPU_GYRO_ORIENTATION    { -1, 0, 0,  0, -1, 0,   0, 0, 1 }
#else
#define MPU_GYRO_ORIENTATION	{ 0, -1, 0,  -1, 0, 0,   0, 0, -1 }
#endif

#define MPU_ACCEL_NAME		"kxtf9"
#define MPU_ACCEL_IRQ_GPIO	TEGRA_GPIO_PI0
#define MPU_ACCEL_ADDR		0x0F
#define MPU_ACCEL_BUS_NUM	2

#if (defined(CONFIG_MACH_STAR_P990) || defined(CONFIG_MACH_STAR_P999))
#define MPU_ACCEL_ORIENTATION   { 0, 1, 0,  -1, 0, 0,   0, 0, 1 }
#else
#define MPU_ACCEL_ORIENTATION	{ 0, -1, 0,  -1, 0, 0,   0, 0, -1 }
#endif

#define MPU_COMPASS_NAME	"ami30x"
#define MPU_COMPASS_IRQ_GPIO	TEGRA_GPIO_PR4
#define MPU_COMPASS_ADDR	0x0E
#define MPU_COMPASS_BUS_NUM	2

// P990 Compass Sensor change 2012.11.02
#if 0//(defined(CONFIG_MACH_STAR_P990) || defined(CONFIG_MACH_STAR_P999)) 
#define MPU_COMPASS_ORIENTATION { 0, -1, 0,  1, 0, 0,   0, 0, -1 }
#else
#define MPU_COMPASS_ORIENTATION	{ 0, -1, 0,  -1, 0, 0,   0, 0, -1 }
#endif

#endif
//MOBII_CHNANGE_E 20120213 mg.chang@mobii.co.kr : Appy Sensor Porting

static struct regulator *reg_avdd_cam1; /* LDO9 */
static struct regulator *reg_vdd_af;    /* LDO13 */
static struct regulator *reg_vdd_mipi;  /* LDO17 */
static struct regulator *reg_vddio_vi;  /* LDO18 */

#if 0
static int star_camera_init(void)
{
	tegra_gpio_enable(CAMERA1_PWDN_GPIO);
	gpio_request(CAMERA1_PWDN_GPIO, "camera1_powerdown");
	gpio_direction_output(CAMERA1_PWDN_GPIO, 0);
	gpio_export(CAMERA1_PWDN_GPIO, false);

	tegra_gpio_enable(CAMERA1_RESET_GPIO);
	gpio_request(CAMERA1_RESET_GPIO, "camera1_reset");
	gpio_direction_output(CAMERA1_RESET_GPIO, 0);
	gpio_export(CAMERA1_RESET_GPIO, false);

	tegra_gpio_enable(CAMERA2_PWDN_GPIO);
	gpio_request(CAMERA2_PWDN_GPIO, "camera2_powerdown");
	gpio_direction_output(CAMERA2_PWDN_GPIO, 0);
	gpio_export(CAMERA2_PWDN_GPIO, false);

	tegra_gpio_enable(CAMERA2_RESET_GPIO);
	gpio_request(CAMERA2_RESET_GPIO, "camera2_reset");
	gpio_direction_output(CAMERA2_RESET_GPIO, 0);
	gpio_export(CAMERA2_RESET_GPIO, false);

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

static int star_ov5650_power_on(void)
{
	gpio_set_value(CAMERA1_PWDN_GPIO, 0);

	if (!reg_avdd_cam1) {
		reg_avdd_cam1 = regulator_get(NULL, "vdd_cam1");
		if (IS_ERR_OR_NULL(reg_avdd_cam1)) {
			pr_err("star_ov5650_power_on: vdd_cam1 failed\n");
			reg_avdd_cam1 = NULL;
			return PTR_ERR(reg_avdd_cam1);
		}
		regulator_enable(reg_avdd_cam1);
	}
	mdelay(5);

	if (!reg_vdd_mipi) {
		reg_vdd_mipi = regulator_get(NULL, "vddio_mipi");
		if (IS_ERR_OR_NULL(reg_vdd_mipi)) {
			pr_err("star_ov5650_power_on: vddio_mipi failed\n");
			reg_vdd_mipi = NULL;
			return PTR_ERR(reg_vdd_mipi);
		}
		regulator_enable(reg_vdd_mipi);
	}
	mdelay(5);

	if (!reg_vdd_af) {
		reg_vdd_af = regulator_get(NULL, "vdd_vcore_af");
		if (IS_ERR_OR_NULL(reg_vdd_af)) {
			pr_err("star_ov5650_power_on: vdd_vcore_af failed\n");
			reg_vdd_af = NULL;
			return PTR_ERR(reg_vdd_af);
		}
		regulator_enable(reg_vdd_af);
	}
	mdelay(5);

	gpio_set_value(CAMERA1_RESET_GPIO, 1);
	mdelay(10);
	gpio_set_value(CAMERA1_RESET_GPIO, 0);
	mdelay(5);
	gpio_set_value(CAMERA1_RESET_GPIO, 1);
	mdelay(20);
	gpio_set_value(CAMERA_AF_PD_GPIO, 1);

	return 0;
}

static int star_ov5650_power_off(void)
{
	gpio_set_value(CAMERA_AF_PD_GPIO, 0);
	gpio_set_value(CAMERA1_PWDN_GPIO, 1);
	gpio_set_value(CAMERA1_RESET_GPIO, 0);

	if (reg_avdd_cam1) {
		regulator_disable(reg_avdd_cam1);
		regulator_put(reg_avdd_cam1);
		reg_avdd_cam1 = NULL;
	}

	if (reg_vdd_mipi) {
		regulator_disable(reg_vdd_mipi);
		regulator_put(reg_vdd_mipi);
		reg_vdd_mipi = NULL;
	}

	if (reg_vdd_af) {
		regulator_disable(reg_vdd_af);
		regulator_put(reg_vdd_af);
		reg_vdd_af = NULL;
	}

	return 0;
}

static int star_soc380_power_on(void)
{
	gpio_set_value(CAMERA2_PWDN_GPIO, 0);

	if (!reg_vddio_vi) {
		reg_vddio_vi = regulator_get(NULL, "vddio_vi");
		if (IS_ERR_OR_NULL(reg_vddio_vi)) {
			pr_err("star_soc380_power_on: vddio_vi failed\n");
			reg_vddio_vi = NULL;
			return PTR_ERR(reg_vddio_vi);
		}
		regulator_set_voltage(reg_vddio_vi, 1800*1000, 1800*1000);
		mdelay(5);
		regulator_enable(reg_vddio_vi);
	}

	if (!reg_avdd_cam1) {
		reg_avdd_cam1 = regulator_get(NULL, "vdd_cam1");
		if (IS_ERR_OR_NULL(reg_avdd_cam1)) {
			pr_err("star_soc380_power_on: vdd_cam1 failed\n");
			reg_avdd_cam1 = NULL;
			return PTR_ERR(reg_avdd_cam1);
		}
		regulator_enable(reg_avdd_cam1);
	}
	mdelay(5);

	gpio_set_value(CAMERA2_RESET_GPIO, 1);
	mdelay(10);
	gpio_set_value(CAMERA2_RESET_GPIO, 0);
	mdelay(5);
	gpio_set_value(CAMERA2_RESET_GPIO, 1);
	mdelay(20);

	return 0;

}

static int star_soc380_power_off(void)
{
	gpio_set_value(CAMERA2_PWDN_GPIO, 1);
	gpio_set_value(CAMERA2_RESET_GPIO, 0);

	if (reg_avdd_cam1) {
		regulator_disable(reg_avdd_cam1);
		regulator_put(reg_avdd_cam1);
		reg_avdd_cam1 = NULL;
	}
	if (reg_vddio_vi) {
		regulator_disable(reg_vddio_vi);
		regulator_put(reg_vddio_vi);
		reg_vddio_vi = NULL;
	}

	return 0;
}

struct ov5650_platform_data star_ov5650_data = {
	.power_on = star_ov5650_power_on,
	.power_off = star_ov5650_power_off,
};

struct soc380_platform_data star_soc380_data = {
	.power_on = star_soc380_power_on,
	.power_off = star_soc380_power_off,
};
#endif

//LGE_CHANGE_S chen.yingchun 20111223

#define MAIN_CAM_RESET_GPIO        TEGRA_GPIO_PD2
#define MAIN_CAM_VCM_EN_GPIO       TEGRA_GPIO_PT4
#define SUB_CAM_RESET_GPIO         TEGRA_GPIO_PBB1
#define SUB_CAM_PWDN_GPIO          TEGRA_GPIO_PD5

static int star_imx073_power_on(void)
{

        printk("star_imx073_power_on");

        gpio_set_value(MAIN_CAM_VCM_EN_GPIO, 0);
        mdelay(10);
        gpio_set_value(MAIN_CAM_RESET_GPIO, 0);
        mdelay(10);
#if defined(CONFIG_REGULATOR_LP8720)
	    star_cam_Main_power_on();
#endif
	    mdelay(10);

        gpio_set_value(MAIN_CAM_VCM_EN_GPIO, 1);
        mdelay(10);
        gpio_set_value(MAIN_CAM_RESET_GPIO, 1);
        mdelay(10);

	return 0;
}

static int star_imx073_power_off(void)
{
        gpio_set_value(MAIN_CAM_VCM_EN_GPIO, 0);
        mdelay(10);
        gpio_set_value(MAIN_CAM_RESET_GPIO, 0);
        mdelay(10);
#if defined(CONFIG_REGULATOR_LP8720)
	star_cam_power_off();
#endif
	return 0;
}

static int star_mt9m113_power_on(void)
{

        printk("star_mt9m113_power_on");

        gpio_set_value(SUB_CAM_PWDN_GPIO, 0);
        udelay(5);
        gpio_set_value(SUB_CAM_RESET_GPIO, 0);

#if defined(CONFIG_REGULATOR_LP8720)
        star_cam_VT_power_on();
#endif
        udelay(5);

        gpio_set_value(SUB_CAM_RESET_GPIO, 1);
        udelay(5);

        return 0;
}

static int star_mt9m113_power_off(void)
{
        gpio_set_value(SUB_CAM_PWDN_GPIO, 1);
        udelay(5);
        gpio_set_value(SUB_CAM_RESET_GPIO, 0);
        udelay(5);

#if defined(CONFIG_REGULATOR_LP8720)
        star_cam_power_off();
#endif
        return 0;
}

#if defined(CONFIG_REGULATOR_LP8720)
static struct lp8720_platform_data      lp8720_pdata = {
        .en_gpio_num    =   TEGRA_GPIO_PR6,
};
#endif

#if defined(CONFIG_VIDEO_IMX073)
struct imx073_platform_data star_imx073_data = {
        .power_on = star_imx073_power_on,
        .power_off = star_imx073_power_off,
};
#endif

#if defined(CONFIG_VIDEO_MT9M113)
struct mt9m113_platform_data star_mt9m113_data = {
        .power_on = star_mt9m113_power_on,
        .power_off = star_mt9m113_power_off,
};
#endif

#if defined(CONFIG_TORCH_AAT1270)
struct aat1270_platform_data star_aat1270_data = {
        .gpio_flen = TEGRA_GPIO_PBB4,
        .gpio_enset = TEGRA_GPIO_PT2,
};

struct platform_device star_aat1270_device = {
        .name           = "aat1270",
        .id             = -1,
        .dev = {
                .platform_data = &star_aat1270_data,
        },
};
#endif

static struct i2c_board_info star_i2c3_board_info[] = {
#if defined(CONFIG_REGULATOR_LP8720)
        {
                I2C_BOARD_INFO(LP8720_I2C_NAME,         LP8720_I2C_ADDR),
                .platform_data  =       &lp8720_pdata,
        },
#endif

#if defined(CONFIG_VIDEO_IMX073)
        {
                I2C_BOARD_INFO("imx073", 0x1A),
                .platform_data = &star_imx073_data,
        },
#endif

#if defined(CONFIG_VIDEO_DW9712)
        {
                I2C_BOARD_INFO("dw9712", 0x0C),
        },
#endif

#if defined(CONFIG_VIDEO_MT9M113)
        {
                I2C_BOARD_INFO("mt9m113", 0x3D),
                .platform_data = &star_mt9m113_data,
        },
#endif

};

#define APBDEV_PMC_NO_IOPOWER_0         0x44

static void __iomem *pmc = IO_ADDRESS(TEGRA_PMC_BASE);

static int star_camera_init(void)
{

	//forced clear No IO Power Register of vi
        u32 reg_data = 0;
        reg_data = readl(pmc + APBDEV_PMC_NO_IOPOWER_0);
        reg_data &= ~(0x10);
        writel(reg_data, pmc + APBDEV_PMC_NO_IOPOWER_0);

        tegra_gpio_enable(MAIN_CAM_RESET_GPIO);
        gpio_request(MAIN_CAM_RESET_GPIO, "8M_Cam_Reset");
        gpio_direction_output(MAIN_CAM_RESET_GPIO, 0);

        tegra_gpio_enable(MAIN_CAM_VCM_EN_GPIO);
        gpio_request(MAIN_CAM_VCM_EN_GPIO, "8M_Cam_Vcm_En");
        gpio_direction_output(MAIN_CAM_VCM_EN_GPIO, 0);

        tegra_gpio_enable(SUB_CAM_PWDN_GPIO);
        gpio_request(SUB_CAM_PWDN_GPIO, "VT_Cam_PowerDown");
        gpio_direction_output(SUB_CAM_PWDN_GPIO, 0);

        tegra_gpio_enable(SUB_CAM_RESET_GPIO);
        gpio_request(SUB_CAM_RESET_GPIO, "VT_Cam_Reset");
        gpio_direction_output(SUB_CAM_RESET_GPIO, 0);

	i2c_register_board_info(3, star_i2c3_board_info,
                ARRAY_SIZE(star_i2c3_board_info));

#if defined(CONFIG_TORCH_AAT1270)
	platform_device_register(&star_aat1270_device);
#endif
}

//LGE_CHANGE_E chen.yingchun 20111223

static int star_fuse_power_en(int enb)
{
	int ret;

	ret = gpio_request(FUSE_POWER_EN_GPIO, "fuse_power_en");
	if (ret) {
		pr_err("%s: gpio_request fail (%d)\n", __func__, ret);
		return ret;
	}

	ret = gpio_direction_output(FUSE_POWER_EN_GPIO, enb);
	if (ret) {
		pr_err("%s: gpio_direction_output fail (%d)\n", __func__, ret);
		return ret;
	}

	gpio_free(FUSE_POWER_EN_GPIO);
	return 0;
}

static struct pca953x_platform_data star_tca6416_data = {
	.gpio_base = TCA6416_GPIO_BASE,
};

#if 0
static struct i2c_board_info star_i2c3_board_info[] = {
	{
		I2C_BOARD_INFO("ov5650", 0x36),
		.platform_data = &star_ov5650_data,
	},
	{
		I2C_BOARD_INFO("ad5820", 0x0c),
	},
	{
		I2C_BOARD_INFO("soc380", 0x3C),
		.platform_data = &star_soc380_data,
	},
};
#endif
static void star_adxl34x_init(void)
{
	tegra_gpio_enable(ADXL34X_IRQ_GPIO);
	gpio_request(ADXL34X_IRQ_GPIO, "adxl34x");
	gpio_direction_input(ADXL34X_IRQ_GPIO);
}

static void star_isl29018_init(void)
{
	tegra_gpio_enable(ISL29018_IRQ_GPIO);
	gpio_request(ISL29018_IRQ_GPIO, "isl29018");
	gpio_direction_input(ISL29018_IRQ_GPIO);
}

static struct i2c_board_info star_i2c1_board_info[] = {
	{
		I2C_BOARD_INFO("adxl34x", 0x1D),
		.irq = TEGRA_GPIO_TO_IRQ(ADXL34X_IRQ_GPIO),
	},
	{
		I2C_BOARD_INFO("isl29018", 0x44),
		.irq = TEGRA_GPIO_TO_IRQ(ISL29018_IRQ_GPIO),
	},
};

static void star_adt7461_init(void)
{
	tegra_gpio_enable(ADT7461_IRQ_GPIO);
	gpio_request(ADT7461_IRQ_GPIO, "adt7461");
	gpio_direction_input(ADT7461_IRQ_GPIO);
}

static struct adt7461_platform_data star_adt7461_pdata = {
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

static struct i2c_board_info star_i2c4_board_info[] = {
	{
		I2C_BOARD_INFO("adt7461", 0x4C),
		.irq = TEGRA_GPIO_TO_IRQ(ADT7461_IRQ_GPIO),
		.platform_data = &star_adt7461_pdata,
	},
	{
		I2C_BOARD_INFO("tca6416", 0x20),
		.platform_data = &star_tca6416_data,
	},
};


//proximity sensor related codes
#if defined(CONFIG_SENSOR_GP2A)
static struct i2c_board_info __initdata star_i2c_proxi_info[] = {
{
	I2C_BOARD_INFO(GP2A_NAME, GP2A_ADDR),
	.irq            =   TEGRA_GPIO_TO_IRQ(PROXI_OUT_GPIO),
	   .platform_data  =   0,
	},
};
#endif

static int __init star_proximity_init(void)
{
#if defined(CONFIG_SENSOR_GP2A)
	gpio_request(PROXI_OUT_GPIO, "prox_out");
    tegra_gpio_enable(PROXI_OUT_GPIO);
	gpio_direction_input(PROXI_OUT_GPIO);
    i2c_register_board_info(2, star_i2c_proxi_info, ARRAY_SIZE(star_i2c_proxi_info));
#endif	
    return 0;
}
//end proximity sensor


//MOBII_CHNANGE_S 20120213 mg.chang@mobii.co.kr : Appy Sensor Porting
#if defined(CONFIG_MPU_SENSORS_MPU3050)
static struct mpu3050_platform_data mpu3050_data = {
	.int_config		= 0x10,
	.orientation	= MPU_GYRO_ORIENTATION,
	.level_shifter = 0,
	.accel = {
		.get_slave_descr = kxtf9_get_slave_descr,
		.irq			= TEGRA_GPIO_TO_IRQ(MPU_ACCEL_IRQ_GPIO),
		.adapt_num		= MPU_ACCEL_BUS_NUM, 
		.bus			= EXT_SLAVE_BUS_SECONDARY,
		.address		= MPU_ACCEL_ADDR,
		.orientation	= MPU_ACCEL_ORIENTATION,
		},
	.compass = {
		.get_slave_descr = ami30x_get_slave_descr,
		.irq			= TEGRA_GPIO_TO_IRQ(MPU_COMPASS_IRQ_GPIO),
		.adapt_num		= MPU_COMPASS_BUS_NUM, 
		.bus			= EXT_SLAVE_BUS_PRIMARY,
		.address		= MPU_COMPASS_ADDR,
		.orientation	= MPU_COMPASS_ORIENTATION,
	},
};


static struct i2c_board_info __initdata star_i2c_sensor_info[] = {
	{
		I2C_BOARD_INFO(MPU_GYRO_NAME, MPU_GYRO_ADDR),
		.platform_data = &mpu3050_data,
		.irq = TEGRA_GPIO_TO_IRQ(MPU_GYRO_IRQ_GPIO),
	},
};


static int __init star_mpu3050_init(void)
{
		tegra_gpio_enable(TEGRA_GPIO_PQ5);
		tegra_gpio_enable(TEGRA_GPIO_PR4);
		tegra_gpio_enable(TEGRA_GPIO_PI0);

		i2c_register_board_info(MPU_GYRO_BUS_NUM, star_i2c_sensor_info, ARRAY_SIZE(star_i2c_sensor_info));
		return 0;

}
#endif
//MOBII_CHNANGE_E 20120213 mg.chang@mobii.co.kr : Appy Sensor Porting


int __init star_sensors_init(void)
{
#if defined(CONFIG_SENSOR_GP2A)
	star_proximity_init();
#endif

//MOBII_CHNANGE_S 20120213 mg.chang@mobii.co.kr : Appy Sensor Porting
#if defined(CONFIG_MPU_SENSORS_MPU3050)
	star_mpu3050_init();
#endif
//MOBII_CHNANGE_E 20120213 mg.chang@mobii.co.kr : Appy Sensor Porting

	star_camera_init();
/*
	star_adxl34x_init();

	star_isl29018_init();

	star_adt7461_init();

	i2c_register_board_info(0, star_i2c1_board_info,
		ARRAY_SIZE(star_i2c1_board_info));

	i2c_register_board_info(4, star_i2c4_board_info,
		ARRAY_SIZE(star_i2c4_board_info));

	i2c_register_board_info(3, star_i2c3_board_info,
		ARRAY_SIZE(star_i2c3_board_info));

	tegra_fuse_regulator_en = star_fuse_power_en;
*/

	return 0;
}

int __init star_sensor_late_init(void)
{
	int ret;

	if (!machine_is_star())
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
	reg_vddio_vi = NULL;
	return 0;

fail_put_regulator:
	regulator_put(reg_vddio_vi);
	reg_vddio_vi = NULL;
	return ret;
}

late_initcall(star_sensor_late_init);

