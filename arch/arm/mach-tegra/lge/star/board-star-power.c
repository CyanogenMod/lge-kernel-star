/*
 * Copyright (C) 2010-2011 NVIDIA, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */
#include <linux/i2c.h>
#include <linux/pda_power.h>
#include <linux/platform_device.h>
#include <linux/resource.h>
#include <linux/regulator/machine.h>
#include <linux/mfd/max8907c.h>
#include <linux/regulator/max8907c-regulator.h>
#include <linux/regulator/max8952.h>
#include <linux/gpio.h>
#include <linux/io.h>
#include <mach/iomap.h>
#include <mach/irqs.h>

#include <mach-tegra/gpio-names.h>
#include <mach-tegra/fuse.h>
#include <mach-tegra/pm.h>
#include <mach-tegra/wakeups-t2.h>
#include <mach-tegra/board.h>

#define PMC_CTRL		0x0
#define PMC_CTRL_INTR_LOW	(1 << 17)

// LGE_CHANGE_S [yehan.ahn@lge.com] 2011-01-15, [LGE_AP20] general power control for device driver
#if defined (CONFIG_MACH_STAR)
int device_power_control(char* reg_id, bool on){
	static struct regulator *device_regulator = NULL;

	device_regulator = regulator_get(NULL, reg_id); 

	if (!device_regulator) {
		return -1;
	}

	if(on)
	{     
		if(!regulator_is_enabled(device_regulator))
		{                   
			regulator_enable(device_regulator);
		}
	}
	else
	{          
		if(regulator_is_enabled(device_regulator))
		{                    
			regulator_disable(device_regulator);
		}
	}
	return 0;
}
#endif
// LGE_CHANGE_E [yehan.ahn@lge.com] 2011-01-15, [LGE_AP20]

// LGE_CHANGE [dojip.kim@lge.com] 2011-01-05,
// [LGE_AP20] VCC_IO_1V2
// We use the max8952 for vdd_cpu.
static struct regulator_consumer_supply max8907c_SD1_supply[] = {
#if 0 /* original codes */
	REGULATOR_SUPPLY("vdd_cpu", NULL),
#else
	REGULATOR_SUPPLY("vcc_io_1v2", NULL),
#endif
};

static struct regulator_consumer_supply max8907c_SD2_supply[] = {
	REGULATOR_SUPPLY("vdd_core", NULL),
	REGULATOR_SUPPLY("vdd_aon", NULL),
};

static struct regulator_consumer_supply max8907c_SD3_supply[] = {
	REGULATOR_SUPPLY("vddio_sys", NULL),
};

static struct regulator_consumer_supply max8907c_LDO1_supply[] = {
	REGULATOR_SUPPLY("vddio_rx_ddr", NULL),
};

static struct regulator_consumer_supply max8907c_LDO2_supply[] = {
	REGULATOR_SUPPLY("avdd_plla", NULL),
};

// LGE_CHANGE [dojip.kim@lge.com] 2010-12-31, [LGE_AP20] VCC_LCD_1V8
static struct regulator_consumer_supply max8907c_LDO3_supply[] = {
#if 0 /* original codes */
	REGULATOR_SUPPLY("vdd_vcom_1v8b", NULL),
#else
	REGULATOR_SUPPLY("vcc_lcd_1v8", NULL),
#endif
};

static struct regulator_consumer_supply max8907c_LDO4_supply[] = {
	REGULATOR_SUPPLY("avdd_usb", NULL),
	REGULATOR_SUPPLY("avdd_usb_pll", NULL),
};

static struct regulator_consumer_supply max8907c_LDO5_supply[] = {
	//LGE_CHANGE [kyungsik.lee@lge.com] 2011-11-28, [LGE_AP20] For eMMc
#if 1 /* original codes */
	REGULATOR_SUPPLY("vdd_vcore_mmc", NULL),
#else
	REGULATOR_SUPPLY("vmmc", "sdhci-tegra.3"),
#endif	
};

static struct regulator_consumer_supply max8907c_LDO6_supply[] = {
	REGULATOR_SUPPLY("avdd_hdmi_pll", NULL),
};

// LGE_CHANGE [dojip.kim@lge.com] 2010-12-31, [LGE_AP20] VCC_SENSOR_3V0
static struct regulator_consumer_supply max8907c_LDO7_supply[] = {
#if 0 /* original codes */
	REGULATOR_SUPPLY("avddio_audio", NULL),
#else
	REGULATOR_SUPPLY("vcc_sensor_3v0", NULL),
#endif
};

// LGE_CHANGE [dojip.kim@lge.com] 2010-12-31, [LGE_AP20] VCC_SENSOR_1V8
static struct regulator_consumer_supply max8907c_LDO8_supply[] = {
#if 0 /* original codes */
	REGULATOR_SUPPLY("vdd_vcom_3v0", NULL),
#else
	REGULATOR_SUPPLY("vcc_sensor_1v8", NULL),
#endif
};

// LGE_CHANGE [dojip.kim@lge.com] 2010-12-31, [LGE_AP20] VCC_FUSE_3V3 [DNI]
static struct regulator_consumer_supply max8907c_LDO9_supply[] = {
#if 0 /* original codes */
	REGULATOR_SUPPLY("vdd_cam1", NULL),
#else
	//REGULATOR_SUPPLY("vcc_fuse_3v3", NULL),
#endif
};

// LGE_CHANGE [dojip.kim@lge.com] 2010-12-31, [LGE_AP20] VCC_TOUCH_3V1
static struct regulator_consumer_supply max8907c_LDO10_supply[] = {
#if 0 /* original codes */
	REGULATOR_SUPPLY("avdd_usb_ic", NULL),
#else
	REGULATOR_SUPPLY("vcc_touch_3v1", NULL),
#endif
};

static struct regulator_consumer_supply max8907c_LDO11_supply[] = {
	REGULATOR_SUPPLY("vddio_pex_clk", NULL),
	REGULATOR_SUPPLY("avdd_hdmi", NULL),
};

// LGE_CHANGE [dojip.kim@lge.com] 2010-12-31, [LGE_AP20] VCC_SDIO_2V8 (SD card)
static struct regulator_consumer_supply max8907c_LDO12_supply[] = {
#if 1 /* original codes */
	REGULATOR_SUPPLY("vddio_sdio", NULL),
#else
	REGULATOR_SUPPLY("vmmc", "sdhci-tegra.2"),
#endif
};

// LGE_CHANGE [dojip.kim@lge.com] 2010-12-31, [LGE_AP20] VCC_MOTOR_3V0
static struct regulator_consumer_supply max8907c_LDO13_supply[] = {
#if 0 /* origianl codes */
	REGULATOR_SUPPLY("vdd_vcore_phtn", NULL),
	REGULATOR_SUPPLY("vdd_vcore_af", NULL),
#else
	REGULATOR_SUPPLY("vcc_motor_3v0", NULL),
#endif
};

// LGE_CHANGE [dojip.kim@lge.com] 2010-12-31, [LGE_AP20] VCC_LCD_2V8
static struct regulator_consumer_supply max8907c_LDO14_supply[] = {
#if 0 /* origianl codes */
	REGULATOR_SUPPLY("avdd_vdac", NULL),
#else
	REGULATOR_SUPPLY("vcc_lcd_2v8", NULL),
#endif
};

static struct regulator_consumer_supply max8907c_LDO15_supply[] = {
	REGULATOR_SUPPLY("vdd_vcore_temp", NULL),
	REGULATOR_SUPPLY("vdd_vcore_hdcp", NULL),
};

static struct regulator_consumer_supply max8907c_LDO16_supply[] = {
#ifdef CONFIG_ONETOUCHSCREEN_TEGRA_STAR
	REGULATOR_SUPPLY("vdd_onetouch", NULL),
#endif // MOBII_CHANGE_S [jg.noh@mobii.co.kr] 2012-02-01
};

static struct regulator_consumer_supply max8907c_LDO17_supply[] = {
	REGULATOR_SUPPLY("vddio_mipi", NULL),
};

static struct regulator_consumer_supply max8907c_LDO18_supply[] = {
	REGULATOR_SUPPLY("vddio_vi", NULL),
	REGULATOR_SUPPLY("vcsi", "tegra_camera"),
};

// LGE_CHANGE [dojip.kim@lge.com] 2010-12-31, [LGE_AP20] VCC_TOUCH_1V8
static struct regulator_consumer_supply max8907c_LDO19_supply[] = {
#if 0 /* original codes */
	REGULATOR_SUPPLY("vddio_lx", NULL),
#else
	REGULATOR_SUPPLY("vcc_touch_1v8", NULL),
#endif
};

// LGE_CHANGE [dojip.kim@lge.com] 2010-12-31, [LGE_AP20] GND
static struct regulator_consumer_supply max8907c_LDO20_supply[] = {
#if 0 /* orignal codes */
	REGULATOR_SUPPLY("vddio_ddr_1v2", NULL),
	REGULATOR_SUPPLY("vddio_hsic", NULL),
#else
	// nothing
#endif
};

	static struct regulator_consumer_supply max8907c_TOUCHLED_supply[] = {
#if defined (CONFIG_MACH_STAR)
		REGULATOR_SUPPLY("DBVDD", NULL),
		REGULATOR_SUPPLY("DCVDD", NULL),
		REGULATOR_SUPPLY("AVDD1", NULL),
		REGULATOR_SUPPLY("AVDD2", NULL),
		REGULATOR_SUPPLY("CPVDD", NULL),
		REGULATOR_SUPPLY("SPKVDD1", NULL),
		REGULATOR_SUPPLY("SPKVDD2", NULL),
#endif
	};


static struct regulator_consumer_supply max8907c_WLED_supply[] = {
	// LGE_CHANGE [sungyel.bae@lge.com] 2011-01-21, [LGE_AP20] TOUCH_LED
#if defined (CONFIG_MACH_STAR)
	REGULATOR_SUPPLY("vcc_wled", NULL),

#endif
};

//LGE_CHANGE_S chen.yingchun 20120103 regulator_set_voltage of "vddio_vi" fail 
/*LDO18 has two consumers, "regulator_set_voltage" of "vddio_vi" failed, suspect 
  function "regulator_check_consumers" may have some problem.
  For less change, just set LDO18 voltage to 1.8v when register_regulator.
  If you have better solution, plz revert this change*/
#define MAX8907C_REGULATOR_APPLYUV_DEVICE(_id, _minmv, _maxmv)          \
	static struct regulator_init_data max8907c_##_id##_data = {             \
		.constraints = {                                                \
			.min_uV = (_minmv),                                     \
			.max_uV = (_maxmv),                                     \
			.apply_uV = true,                                       \
			.valid_modes_mask = (REGULATOR_MODE_NORMAL |            \
					REGULATOR_MODE_STANDBY),           \
			.valid_ops_mask = (REGULATOR_CHANGE_MODE |              \
					REGULATOR_CHANGE_STATUS |            \
					REGULATOR_CHANGE_VOLTAGE |           \
					REGULATOR_CHANGE_CURRENT      ),             \
		},                                                              \
		.num_consumer_supplies = ARRAY_SIZE(max8907c_##_id##_supply),   \
		.consumer_supplies = max8907c_##_id##_supply,                   \
	};                                                                      \
static struct platform_device max8907c_##_id##_device = {               \
	.name   = "max8907c-regulator",                                 \
	.id     = MAX8907C_##_id,                                       \
	.dev    = {                                                     \
		.platform_data = &max8907c_##_id##_data,                \
	},                                                              \
}
//LGE_CHANGE_E chen.yingchun 20120103 regulator_set_voltage of "vddio_vi" fail

/* LGE_CHANGE_S [sungyel.bae@lge.com] 2011-01-22, [LGE_AP20] touch led */
#if defined (CONFIG_MACH_STAR)
#define MAX8907C_REGULATOR_DEVICE(_id, _minmv, _maxmv)			\
	static struct regulator_init_data max8907c_##_id##_data = {		\
		.constraints = {						\
			.min_uV = (_minmv),					\
			.max_uV = (_maxmv),					\
			.valid_modes_mask = (REGULATOR_MODE_NORMAL |		\
					REGULATOR_MODE_STANDBY),		\
			.valid_ops_mask = (REGULATOR_CHANGE_MODE |		\
					REGULATOR_CHANGE_STATUS |		\
					REGULATOR_CHANGE_VOLTAGE |		\
					REGULATOR_CHANGE_CURRENT	 ),		\
		},								\
		.num_consumer_supplies = ARRAY_SIZE(max8907c_##_id##_supply),	\
		.consumer_supplies = max8907c_##_id##_supply,			\
	};									\
static struct platform_device max8907c_##_id##_device = {		\
	.name	= "max8907c-regulator",					\
	.id	= MAX8907C_##_id,					\
	.dev	= {							\
		.platform_data = &max8907c_##_id##_data,		\
	},								\
}
#else
#define MAX8907C_REGULATOR_DEVICE(_id, _minmv, _maxmv)			\
	static struct regulator_init_data max8907c_##_id##_data = {		\
		.constraints = {						\
			.min_uV = (_minmv),					\
			.max_uV = (_maxmv),					\
			.valid_modes_mask = (REGULATOR_MODE_NORMAL |		\
					REGULATOR_MODE_STANDBY),		\
			.valid_ops_mask = (REGULATOR_CHANGE_MODE |		\
					REGULATOR_CHANGE_STATUS |		\
					REGULATOR_CHANGE_VOLTAGE),		\
		},								\
		.num_consumer_supplies = ARRAY_SIZE(max8907c_##_id##_supply),	\
		.consumer_supplies = max8907c_##_id##_supply,			\
	};									\
static struct platform_device max8907c_##_id##_device = {		\
	.name	= "max8907c-regulator",					\
	.id	= MAX8907C_##_id,					\
	.dev	= {							\
		.platform_data = &max8907c_##_id##_data,		\
	},								\
}
#endif
/* LGE_CHANGE_E [sungyel.bae@lge.com] 2011-01-22, [LGE_AP20] */
// LGE_CHANGE [dojip.kim@lge.com] 2011-01-05, [LGE_AP20] SD1->1.2V fixed voltage
//MAX8907C_REGULATOR_DEVICE(SD1, 637500, 1425000);
MAX8907C_REGULATOR_DEVICE(SD1, 1200000, 1200000);
MAX8907C_REGULATOR_DEVICE(SD2, 637500, 1425000);
MAX8907C_REGULATOR_DEVICE(SD3, 750000, 3900000);
MAX8907C_REGULATOR_DEVICE(LDO1, 750000, 3900000);
MAX8907C_REGULATOR_DEVICE(LDO2, 650000, 2225000);
// LGE_CHANGE [kyungsik.lee@lge.com] 2011-11-30, [LGE_AP20] LCD 1V8
MAX8907C_REGULATOR_DEVICE(LDO3, 1800000, 1800000);
//MAX8907C_REGULATOR_DEVICE(LDO3, 650000, 2225000);
MAX8907C_REGULATOR_DEVICE(LDO4, 750000, 3900000);
MAX8907C_REGULATOR_DEVICE(LDO5, 750000, 3900000);
MAX8907C_REGULATOR_DEVICE(LDO6, 750000, 3900000);
// LGE_CHANGE_S [dongjin73.kim@lge.com] 2011-01-19, [LGE_AP20] Sensor integration
//MAX8907C_REGULATOR_DEVICE(LDO7, 750000, 3900000);
//MAX8907C_REGULATOR_DEVICE(LDO8, 750000, 3900000);
MAX8907C_REGULATOR_DEVICE(LDO7, 3000000, 3000000);
MAX8907C_REGULATOR_DEVICE(LDO8, 1800000, 1800000);
// LGE_CHANGE_E [dongjin73.kim@lge.com] 2011-01-19, [LGE_AP20] Sensor integration
MAX8907C_REGULATOR_DEVICE(LDO9, 750000, 3900000);
// LGE_CHANGE [yehan.ahn@lge.com] 2011-01-08, [LGE_AP20] touch device VDD
//MAX8907C_REGULATOR_DEVICE(LDO10, 750000, 3900000);
MAX8907C_REGULATOR_DEVICE(LDO10, 3100000, 3100000);
MAX8907C_REGULATOR_DEVICE(LDO11, 750000, 3900000);
MAX8907C_REGULATOR_DEVICE(LDO12, 750000, 3900000);
// LGE_CHANGE [yehan.ahn@lge.com] 2011-01-20, [LGE_AP20] vibrator VDD
//MAX8907C_REGULATOR_DEVICE(LDO13, 750000, 3900000);
MAX8907C_REGULATOR_DEVICE(LDO13, 3000000, 3000000);
// LGE_CHANGE [kyungsik.lee@lge.com] 2011-11-30, [LGE_AP20] LCD 2V8
//MAX8907C_REGULATOR_DEVICE(LDO14, 2800000, 2800000);
MAX8907C_REGULATOR_DEVICE(LDO14, 2800000, 3100000);
//MAX8907C_REGULATOR_DEVICE(LDO14, 750000, 3900000);
MAX8907C_REGULATOR_DEVICE(LDO15, 750000, 3900000);
#ifdef CONFIG_ONETOUCHSCREEN_TEGRA_STAR
MAX8907C_REGULATOR_DEVICE(LDO16, 2800000, 2800000);
#else
MAX8907C_REGULATOR_DEVICE(LDO16, 750000, 3900000);
#endif // MOBII_CHANGE_S [jg.noh@mobii.co.kr] 2012-02-01
MAX8907C_REGULATOR_DEVICE(LDO17, 650000, 2225000);
//LGE_CHANGE_S chen.yingchun 20120103 regulator_set_voltage of "vddio_vi" fail
//MAX8907C_REGULATOR_DEVICE(LDO18, 650000, 2225000);
MAX8907C_REGULATOR_APPLYUV_DEVICE(LDO18, 1800000, 1800000);
//LGE_CHANGE_E chen.yingchun 20120103 regulator_set_voltage of "vddio_vi" fail
// LGE_CHANGE [yehan.ahn@lge.com] 2011-01-08, [LGE_AP20] touch device VDD
//MAX8907C_REGULATOR_DEVICE(LDO19, 750000, 3900000);
MAX8907C_REGULATOR_DEVICE(LDO19, 1800000, 1800000);
MAX8907C_REGULATOR_DEVICE(LDO20, 750000, 3900000);
/* LGE_CHANGE [sungyel.bae@lge.com] 2011-01-22, [LGE_AP20] touch led */
#if defined (CONFIG_MACH_STAR)
MAX8907C_REGULATOR_DEVICE(TOUCHLED, 1250000, 5000000);
MAX8907C_REGULATOR_DEVICE(WLED, 3000000, 3000000);
#endif

static struct platform_device *star_max8907c_power_devices[] = {
	&max8907c_SD1_device,
	&max8907c_SD2_device,
	&max8907c_SD3_device,
	&max8907c_LDO1_device,
	&max8907c_LDO2_device,
	&max8907c_LDO3_device,
	&max8907c_LDO4_device,
	&max8907c_LDO5_device,
	&max8907c_LDO6_device,
	&max8907c_LDO7_device,
	&max8907c_LDO8_device,
	&max8907c_LDO9_device,
	&max8907c_LDO10_device,
	&max8907c_LDO11_device,
	&max8907c_LDO12_device,
	&max8907c_LDO13_device,
	&max8907c_LDO14_device,
	&max8907c_LDO15_device,
	&max8907c_LDO16_device,
	&max8907c_LDO17_device,
	&max8907c_LDO18_device,
	&max8907c_LDO19_device,
	&max8907c_LDO20_device,
	/* LGE_CHANGE_S [sungyel.bae@lge.com] 2011-01-22, [LGE_AP20] touch led */
#if defined (CONFIG_MACH_STAR)
	&max8907c_TOUCHLED_device,
	&max8907c_WLED_device,
#endif
};

static int star_max8907c_setup(void)
{
	int ret;

	/*
	 * Configure PWREN, and attach CPU V1 rail to it.
	 * TODO: h/w events (power cycle, reset, battery low) auto-disables PWREN.
	 * Only soft reset (not supported) requires s/w to disable PWREN explicitly
	 */
	ret = max8907c_pwr_en_config();
	if (ret != 0)
		return ret;
	return max8907c_pwr_en_attach();
}

static struct max8907c_platform_data max8907c_pdata = {
	.num_subdevs = ARRAY_SIZE(star_max8907c_power_devices),
	.subdevs = star_max8907c_power_devices,
	.irq_base = TEGRA_NR_IRQS,
	// 20110602 youngjin.yoo@lge.com deep sleep&resume temporary patch TAG:NVBUG_DEEP_SLEEP [S]	
	//	.max8907c_setup = star_max8907c_setup,
	// 20110602 youngjin.yoo@lge.com deep sleep&resume temporary patch TAG:NVBUG_DEEP_SLEEP [E]	
};

// LGE_CHANGE_S [dojip.kim@lge.com] 2011-01-07, [LGE_AP20] DCDC for cpu 1.0V

static struct regulator_consumer_supply max8952_MODE0_supply[] = {
};

static struct regulator_consumer_supply max8952_MODE1_supply[] = {      
#if defined( CONFIG_MACH_STAR) && defined(CONFIG_MACH_STAR_SU660)
	REGULATOR_SUPPLY("vdd_cpu", NULL),	
#endif	
#if defined( CONFIG_MACH_STAR) && defined(CONFIG_MACH_STAR_P990)
	REGULATOR_SUPPLY("vdd_cpu", NULL),	
#endif
#if defined( CONFIG_MACH_STAR) && defined(CONFIG_MACH_STAR_P999)
	REGULATOR_SUPPLY("vdd_cpu", NULL),	
#endif
};

static struct regulator_consumer_supply max8952_MODE2_supply[] = {
};

static struct regulator_consumer_supply max8952_MODE3_supply[] = {
};

#define MAX8952_REGULATOR_INIT(_id, _minmv, _maxmv)			\
	static struct regulator_init_data max8952_##_id##_data = {		\
		.constraints = {						\
			.min_uV = (_minmv),					\
			.max_uV = (_maxmv),					\
			.valid_modes_mask = (REGULATOR_MODE_NORMAL |		\
					REGULATOR_MODE_STANDBY),		\
			.valid_ops_mask = (REGULATOR_CHANGE_MODE |		\
					REGULATOR_CHANGE_STATUS |		\
					REGULATOR_CHANGE_VOLTAGE),		\
		},								\
		.num_consumer_supplies = ARRAY_SIZE(max8952_##_id##_supply),	\
		.consumer_supplies = max8952_##_id##_supply,			\
	};									\
static struct platform_device max8952_##_id##_device = {		\
	.id	= MAX8952_##_id,					\
	.dev	= {							\
		.platform_data = &max8952_##_id##_data,			\
	},								\
}

MAX8952_REGULATOR_INIT(MODE0, 770000, 1400000);
MAX8952_REGULATOR_INIT(MODE1, 770000, 1400000);
MAX8952_REGULATOR_INIT(MODE2, 770000, 1400000);
MAX8952_REGULATOR_INIT(MODE3, 770000, 1400000);

static struct platform_device *bssq_max8952_power_devices[] = {
	&max8952_MODE0_device,
	&max8952_MODE1_device,
	&max8952_MODE2_device,
	&max8952_MODE3_device,
};

static struct max8952_platform_data max8952_pdata = {
	.num_subdevs = ARRAY_SIZE(bssq_max8952_power_devices),
	.subdevs = bssq_max8952_power_devices,
};
// LGE_CHANGE_E [dojip.kim@lge.com] 2011-01-07, [LGE_AP20] DCDC for cpu 1.0V

static struct i2c_board_info __initdata star_regulators[] = {
	{
		I2C_BOARD_INFO("max8907c", 0x3C),
		.irq = INT_EXTERNAL_PMU,
		.platform_data	= &max8907c_pdata,
	},
	{
		I2C_BOARD_INFO("max8952", 0x60),
		.platform_data = &max8952_pdata,
	},
};

static void star_board_suspend(int lp_state, enum suspend_stage stg)
{
	if ((lp_state == TEGRA_SUSPEND_LP1) && (stg == TEGRA_SUSPEND_BEFORE_CPU))
		tegra_console_uart_suspend();
}

static void star_board_resume(int lp_state, enum resume_stage stg)
{
	if ((lp_state == TEGRA_SUSPEND_LP1) && (stg == TEGRA_RESUME_AFTER_CPU))
		tegra_console_uart_resume();
}

static struct tegra_suspend_platform_data star_suspend_data = {
#if 0
	.cpu_timer	= 2000,
	.cpu_off_timer	= 1000,
	.suspend_mode	= TEGRA_SUSPEND_LP0,
	.core_timer	= 0x7e,
	.core_off_timer = 0xc00,
	.corereq_high	= true,
	.sysclkreq_high	= true,
	.combined_req   = true,	
#else
	.cpu_timer	= 2000,
	.cpu_off_timer	= 1000,
	.suspend_mode	= TEGRA_SUSPEND_LP0,
	.core_timer	= 0x7e7e,
	.core_off_timer = 0xf,
	.corereq_high	= true,
	.sysclkreq_high	= true,
	.combined_req   = false,
	.board_suspend = star_board_suspend,
	.board_resume = star_board_resume,
#endif
};

void star_power_off(void)
{
	int ret;

	ret = max8907c_power_off();
	if (ret)
		pr_err("star: failed to power off\n");

	while (1);
}

void __init star_power_off_init(void)
{
	pm_power_off = star_power_off;
}

int __init star_regulator_init(void)
{
	void __iomem *pmc = IO_ADDRESS(TEGRA_PMC_BASE);
	void __iomem *chip_id = IO_ADDRESS(TEGRA_APB_MISC_BASE) + 0x804;
	u32 pmc_ctrl;
	u32 minor;

	minor = (readl(chip_id) >> 16) & 0xf;
	/* A03 (but not A03p) chips do not support LP0 */
	if (minor == 3 && !(tegra_spare_fuse(18) || tegra_spare_fuse(19)))
		star_suspend_data.suspend_mode = TEGRA_SUSPEND_LP1;

	/* configure the power management controller to trigger PMU
	 * interrupts when low */
	pmc_ctrl = readl(pmc + PMC_CTRL);
	writel(pmc_ctrl | PMC_CTRL_INTR_LOW, pmc + PMC_CTRL);

	// LGE_CHANGE_S [yehan.ahn@lge.com] 2011-01-10, [LGE_AP20] for max8952
#if 0
	i2c_register_board_info(4, star_regulators, 1);
#else
	i2c_register_board_info(4, star_regulators, ARRAY_SIZE(star_regulators));
#endif
	// LGE_CHANGE_E [yehan.ahn@lge.com] 2011-01-10, [LGE_AP20] for max8952

	tegra_deep_sleep = max8907c_deep_sleep;

	tegra_init_suspend(&star_suspend_data);

	printk("star_regulator_init() called : %d\n", star_suspend_data.suspend_mode);
	return 0;
}
