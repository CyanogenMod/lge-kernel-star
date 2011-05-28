/*
 * arch/arm/mach-tegra/board-enterprise-power.c
 *
 * Copyright (C) 2011 NVIDIA, Inc.
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
#include <linux/regulator/gpio-switch-regulator.h>
#include <linux/mfd/tps80031.h>
#include <linux/regulator/tps80031-regulator.h>
#include <linux/gpio.h>
#include <mach/suspend.h>
#include <linux/io.h>

#include <mach/iomap.h>
#include <mach/irqs.h>
#include <mach/pinmux.h>

#include "gpio-names.h"
#include "board-enterprise.h"
#include "power.h"
#include "wakeups-t3.h"

/************************ TPS80031 based regulator ****************/
static struct regulator_consumer_supply tps80031_vio_supply[] = {
	REGULATOR_SUPPLY("vio_1v8", NULL),
	REGULATOR_SUPPLY("avdd_osc", NULL),
	REGULATOR_SUPPLY("vddio_sys", NULL),
	REGULATOR_SUPPLY("vddio_uart", NULL),
	REGULATOR_SUPPLY("vddio_lcd", NULL),
	REGULATOR_SUPPLY("vddio_audio", NULL),
	REGULATOR_SUPPLY("vddio_bb", NULL),
	REGULATOR_SUPPLY("vddio_gmi", NULL),
	REGULATOR_SUPPLY("avdd_usb_pll", NULL),
	REGULATOR_SUPPLY("vddio_cam", NULL),
	REGULATOR_SUPPLY("vddio_sdmmc1", NULL),
	REGULATOR_SUPPLY("vddio_sdmmc4", NULL),
	REGULATOR_SUPPLY("avdd_hdmi_pll", NULL),
	REGULATOR_SUPPLY("vddio_gps", NULL),
	REGULATOR_SUPPLY("vdd_lcd_buffered", NULL),
	REGULATOR_SUPPLY("vddio_nand", NULL),
	REGULATOR_SUPPLY("vddio_sd", NULL),
	REGULATOR_SUPPLY("vdd_bat", NULL),
	REGULATOR_SUPPLY("vdd_io", NULL),
};

static struct regulator_consumer_supply tps80031_smps1_supply[] = {
	REGULATOR_SUPPLY("vdd_cpu", NULL),
};

static struct regulator_consumer_supply tps80031_smps2_supply[] = {
	REGULATOR_SUPPLY("vdd_core", NULL),
};

static struct regulator_consumer_supply tps80031_smps3_supply[] = {
	REGULATOR_SUPPLY("en_vddio_ddr_1v2", NULL),
	REGULATOR_SUPPLY("vddio_ddr", NULL),
	REGULATOR_SUPPLY("vdd_lpddr", NULL),
	REGULATOR_SUPPLY("ddr_comp_pu", NULL),
};

static struct regulator_consumer_supply tps80031_smps4_supply[] = {
	REGULATOR_SUPPLY("vddio_sdmmc_2v85", NULL),
};

static struct regulator_consumer_supply tps80031_vana_supply[] = {
	REGULATOR_SUPPLY("unused_vana", NULL),
};

static struct regulator_consumer_supply tps80031_ldo1_supply[] = {
	REGULATOR_SUPPLY("avdd_dsi_csi", NULL),
};

static struct regulator_consumer_supply tps80031_ldo2_supply[] = {
	REGULATOR_SUPPLY("vdd_rtc", NULL),
};

static struct regulator_consumer_supply tps80031_ldo3_supply[] = {
	REGULATOR_SUPPLY("vdd_vbrtr", NULL),
};

static struct regulator_consumer_supply tps80031_ldo4_supply[] = {
	REGULATOR_SUPPLY("avdd_lcd", NULL),
};

static struct regulator_consumer_supply tps80031_ldo5_supply[] = {
	REGULATOR_SUPPLY("vdd_sensor", NULL),
	REGULATOR_SUPPLY("vdd_compass", NULL),
	REGULATOR_SUPPLY("vdd_als", NULL),
	REGULATOR_SUPPLY("vdd_gyro", NULL),
	REGULATOR_SUPPLY("vdd_touch", NULL),
	REGULATOR_SUPPLY("vdd_proxim_diode", NULL),
};

static struct regulator_consumer_supply tps80031_ldo6_supply[] = {
	REGULATOR_SUPPLY("vdd_ddr_rx", NULL),
	REGULATOR_SUPPLY("vddf_core_emmc", NULL),
};

static struct regulator_consumer_supply tps80031_ldo7_supply[] = {
	REGULATOR_SUPPLY("vdd_plla_p_c_s", NULL),
	REGULATOR_SUPPLY("vdd_pllm", NULL),
	REGULATOR_SUPPLY("vdd_pllu_d", NULL),
	REGULATOR_SUPPLY("vdd_pllx", NULL),
};

static struct regulator_consumer_supply tps80031_ldoln_supply[] = {
	REGULATOR_SUPPLY("vdd_ddr_hs", NULL),
};

static struct regulator_consumer_supply tps80031_ldousb_supply[] = {
	REGULATOR_SUPPLY("unused_ldousb", NULL),
};

#define TPS_PDATA_INIT(_id, _minmv, _maxmv, _supply_reg, _always_on,	\
	_boot_on, _apply_uv, _init_uV, _init_enable, _init_apply)	\
	static struct tps80031_regulator_platform_data pdata_##_id = {	\
		.regulator = {						\
			.constraints = {				\
				.min_uV = (_minmv)*1000,		\
				.max_uV = (_maxmv)*1000,		\
				.valid_modes_mask = (REGULATOR_MODE_NORMAL |  \
						REGULATOR_MODE_STANDBY),      \
				.valid_ops_mask = (REGULATOR_CHANGE_MODE |    \
						REGULATOR_CHANGE_STATUS |     \
						REGULATOR_CHANGE_VOLTAGE),    \
				.always_on = _always_on,		\
				.boot_on = _boot_on,			\
				.apply_uV = _apply_uv,			\
			},						\
			.num_consumer_supplies =			\
				ARRAY_SIZE(tps80031_##_id##_supply),	\
			.consumer_supplies = tps80031_##_id##_supply,	\
			.supply_regulator = _supply_reg,		\
		},							\
		.init_uV =  _init_uV * 1000,				\
		.init_enable = _init_enable,				\
		.init_apply = _init_apply,				\
	}

TPS_PDATA_INIT(vio,   600, 2100, 0, 0, 0, 0, -1, 0, 0);
TPS_PDATA_INIT(smps1, 600, 2100, 0, 0, 0, 0, -1, 0, 0);
TPS_PDATA_INIT(smps2, 600, 2100, 0, 0, 0, 0, -1, 0, 0);
TPS_PDATA_INIT(smps3, 600, 2100, 0, 0, 0, 0, -1, 0, 0);
TPS_PDATA_INIT(smps4, 600, 2100, 0, 0, 0, 0, -1, 0, 0);

TPS_PDATA_INIT(ldo1, 1000, 3300, tps80031_rails(VIO), 0, 0, 0, -1, 0, 0);
TPS_PDATA_INIT(ldo2, 1000, 3300, 0, 0, 0, 0, -1, 0, 0);
TPS_PDATA_INIT(ldo3, 1000, 3300, tps80031_rails(VIO), 0, 0, 0, -1, 0, 0);
TPS_PDATA_INIT(ldo4, 1000, 3300, 0, 0, 0, 0, -1, 0, 0);
TPS_PDATA_INIT(ldo5, 1000, 3300, 0, 0, 0, 0, -1, 0, 0);
TPS_PDATA_INIT(ldo6, 1000, 3300, 0, 0, 0, 0, -1, 0, 0);
TPS_PDATA_INIT(ldo7, 1000, 3300, tps80031_rails(VIO), 0, 0, 0, -1, 0, 0);
TPS_PDATA_INIT(ldoln, 1000, 3300, tps80031_rails(SMPS3), 0, 0, 0, -1, 0, 0);
TPS_PDATA_INIT(ldousb, 1000, 3300, 0, 0, 0, 0, -1, 0, 0);
TPS_PDATA_INIT(vana,  1000, 3300, 0, 0, 0, 0, -1, 0, 0);

#define TPS_REG(_id, _data)				\
	{						\
		.id	 = TPS80031_ID_##_id,		\
		.name   = "tps80031-regulator",		\
		.platform_data  = &pdata_##_data,	\
	}

static struct tps80031_subdev_info tps80031_devs[] = {
	TPS_REG(VIO, vio),
	TPS_REG(SMPS1, smps1),
	TPS_REG(SMPS2, smps2),
	TPS_REG(SMPS3, smps3),
	TPS_REG(SMPS4, smps4),
	TPS_REG(LDO1, ldo1),
	TPS_REG(LDO2, ldo2),
	TPS_REG(LDO3, ldo3),
	TPS_REG(LDO4, ldo4),
	TPS_REG(LDO5, ldo5),
	TPS_REG(LDO6, ldo6),
	TPS_REG(LDO7, ldo7),
	TPS_REG(LDOLN, ldoln),
	TPS_REG(LDOUSB, ldousb),
	TPS_REG(VANA, vana),
};

struct tps80031_32kclock_plat_data clk32k_pdata = {
	.en_clk32kg = 1,
};
static struct tps80031_platform_data tps_platform = {
	.num_subdevs	= ARRAY_SIZE(tps80031_devs),
	.subdevs	= tps80031_devs,
	.irq_base	= TPS80031_IRQ_BASE,
	.gpio_base	= TPS80031_GPIO_BASE,
	.clk32k_pdata	= &clk32k_pdata,
};

static struct i2c_board_info __initdata enterprise_regulators[] = {
	{
		I2C_BOARD_INFO("tps80031", 0x48),
		.irq		= INT_EXTERNAL_PMU,
		.platform_data	= &tps_platform,
	},
};

/************************ GPIO based switch regulator ****************/

/* REGEN1 from PMU*/
static struct regulator_consumer_supply gpio_switch_pmu_5v15_en_supply[] = {
	REGULATOR_SUPPLY("vdd_5v15", NULL),
};
static int gpio_switch_pmu_5v15_en_voltages[] = {5000};

/* REGEN2 from PMU*/
static struct regulator_consumer_supply gpio_switch_pmu_3v3_en_supply[] = {
	REGULATOR_SUPPLY("avdd_usb_hdmi_3v3", NULL),
	REGULATOR_SUPPLY("avdd_usb", NULL),
	REGULATOR_SUPPLY("avdd_hdmi", NULL),
	REGULATOR_SUPPLY("vdd_nct1008", NULL),
};
static int gpio_switch_pmu_3v3_en_voltages[] = {3300};

/* SYSEN from PMU*/
static struct regulator_consumer_supply gpio_switch_pmu_hdmi_5v0_en_supply[] = {
	REGULATOR_SUPPLY("hdmi_5v0", NULL),
};
static int gpio_switch_pmu_hdmi_5v0_en_voltages[] = {5000};

/* LCD-D16 (GPIO M0) from T30*/
static struct regulator_consumer_supply gpio_switch_vdd_fuse_en_supply[] = {
	REGULATOR_SUPPLY("vpp_fuse", NULL),
};
static int gpio_switch_vdd_fuse_en_voltages[] = {3300};

/* LCD-D17 (GPIO M1) from T30*/
static struct regulator_consumer_supply gpio_switch_sdmmc3_vdd_sel_supply[] = {
	REGULATOR_SUPPLY("vddio_sdmmc3_2v85_1v8", NULL),
	REGULATOR_SUPPLY("sdmmc3_compu_pu", NULL),
	REGULATOR_SUPPLY("vddio_sdmmc3", NULL),
};
static int gpio_switch_sdmmc3_vdd_sel_voltages[] = {2850};

/* LCD-D23 (GPIO M7) from T30*/
static struct regulator_consumer_supply gpio_switch_cam_ldo_2v8_en_supply[] = {
	REGULATOR_SUPPLY("vdd_2v8_cam", NULL),
};
static int gpio_switch_cam_ldo_2v8_en_voltages[] = {2800};

/* LCD-D9 (GPIO F1) from T30*/
static struct regulator_consumer_supply gpio_switch_cam_ldo_1v8_en_supply[] = {
	REGULATOR_SUPPLY("vdd_1v8_cam", NULL),
};
static int gpio_switch_cam_ldo_1v8_en_voltages[] = {1800};

/* Macro for defining gpio switch regulator sub device data */
#define GREG_INIT(_id, _name, _input_supply, _gpio_nr, _active_low, \
			_init_state, _pg, _enable, _disable)		\
	static struct gpio_switch_regulator_subdev_data gpio_pdata_##_name =  \
	{								\
		.regulator_name	= "gpio-switch-"#_name,			\
		.input_supply	= _input_supply,			\
		.id		= _id,					\
		.gpio_nr	= _gpio_nr,				\
		.pin_group	= _pg,					\
		.active_low	= _active_low,				\
		.init_state	= _init_state,				\
		.voltages	= gpio_switch_##_name##_voltages,	\
		.n_voltages	= ARRAY_SIZE(gpio_switch_##_name##_voltages), \
		.num_consumer_supplies =				\
				ARRAY_SIZE(gpio_switch_##_name##_supply), \
		.consumer_supplies = gpio_switch_##_name##_supply,	\
		.constraints = {					\
			.valid_modes_mask = (REGULATOR_MODE_NORMAL |	\
					REGULATOR_MODE_STANDBY),	\
			.valid_ops_mask = (REGULATOR_CHANGE_MODE |	\
					REGULATOR_CHANGE_STATUS |	\
					REGULATOR_CHANGE_VOLTAGE),	\
		},							\
		.enable_rail = _enable,					\
		.disable_rail = _disable,				\
	}

GREG_INIT(0, pmu_5v15_en,     NULL,      TPS80031_GPIO_REGEN1, false, 0, 0, 0, 0);
GREG_INIT(1, pmu_3v3_en,      "vdd_5v15", TPS80031_GPIO_REGEN2, false, 0, 0, 0, 0);
GREG_INIT(2, pmu_hdmi_5v0_en, "vdd_5v15", TPS80031_GPIO_SYSEN, false, 0, 0, 0, 0);

GREG_INIT(3, vdd_fuse_en,    "avdd_usb_hdmi_3v3", TEGRA_GPIO_PM0, false, 0, 0, 0, 0);
GREG_INIT(4, sdmmc3_vdd_sel, "vddio_sdmmc_2v85", TEGRA_GPIO_PM1, false, 0, 0, 0, 0);
GREG_INIT(5, cam_ldo_2v8_en,    NULL, TEGRA_GPIO_PM7, false, 0, 0, 0, 0);
GREG_INIT(6, cam_ldo_1v8_en,    NULL, TEGRA_GPIO_PF1, false, 0, 0, 0, 0);

#define ADD_GPIO_REG(_name)	(&gpio_pdata_##_name)
static struct gpio_switch_regulator_subdev_data *gswitch_subdevs[] = {
	ADD_GPIO_REG(pmu_5v15_en),
	ADD_GPIO_REG(pmu_3v3_en),
	ADD_GPIO_REG(pmu_hdmi_5v0_en),
	ADD_GPIO_REG(vdd_fuse_en),
	ADD_GPIO_REG(sdmmc3_vdd_sel),
	ADD_GPIO_REG(cam_ldo_2v8_en),
	ADD_GPIO_REG(cam_ldo_1v8_en),
};

static struct gpio_switch_regulator_platform_data  gswitch_pdata = {
	.num_subdevs = ARRAY_SIZE(gswitch_subdevs),
	.subdevs = gswitch_subdevs,
};

static struct platform_device gswitch_regulator_pdata = {
	.name	= "gpio-switch-regulator",
	.id	= -1,
	.dev	= {
		.platform_data = &gswitch_pdata,
	},
};

static int __init enterprise_gpio_switch_regulator_init(void)
{
	int i;
	for (i = 0; i < gswitch_pdata.num_subdevs; ++i) {
		struct gpio_switch_regulator_subdev_data *gswitch_data =
						gswitch_pdata.subdevs[i];
		if (gswitch_data->gpio_nr <= TEGRA_NR_GPIOS)
			tegra_gpio_enable(gswitch_data->gpio_nr);
	}
	return platform_device_register(&gswitch_regulator_pdata);
}

int __init enterprise_regulator_init(void)
{
	i2c_register_board_info(4, enterprise_regulators, 1);
	enterprise_gpio_switch_regulator_init();
	return 0;
}

static struct tegra_suspend_platform_data enterprise_suspend_data = {
	.cpu_timer	= 2000,
	.cpu_off_timer	= 0,
	.suspend_mode	= TEGRA_SUSPEND_NONE,
	.core_timer	= 0x7e7e,
	.core_off_timer = 0,
	.separate_req	= true,
	.corereq_high	= false,
	.sysclkreq_high	= true,
	.wake_enb	= 0,
	.wake_high	= 0,
	.wake_low	= 0,
	.wake_any	= 0,
};

int __init enterprise_suspend_init(void)
{
	tegra_init_suspend(&enterprise_suspend_data);
	return 0;
}
