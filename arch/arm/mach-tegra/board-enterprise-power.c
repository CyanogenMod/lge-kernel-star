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
#include <linux/tps80031-charger.h>
#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/cpumask.h>
#include <linux/platform_data/tegra_bpc_mgmt.h>

#include <mach/edp.h>
#include <mach/iomap.h>
#include <mach/irqs.h>
#include <mach/pinmux.h>
#include <mach/tsensor.h>

#include "gpio-names.h"
#include "board.h"
#include "board-enterprise.h"
#include "pm.h"
#include "wakeups-t3.h"

#define PMC_CTRL		0x0
#define PMC_CTRL_INTR_LOW	(1 << 17)

#define PMC_DPD_PADS_ORIDE		0x01c
#define PMC_DPD_PADS_ORIDE_BLINK	(1 << 20)

/************************ TPS80031 based regulator ****************/
static struct regulator_consumer_supply tps80031_vio_supply[] = {
	REGULATOR_SUPPLY("vio_1v8", NULL),
	REGULATOR_SUPPLY("avdd_osc", NULL),
	REGULATOR_SUPPLY("vddio_sys", NULL),
	REGULATOR_SUPPLY("vddio_uart", NULL),
	REGULATOR_SUPPLY("pwrdet_uart", NULL),
	REGULATOR_SUPPLY("vddio_lcd", NULL),
	REGULATOR_SUPPLY("pwrdet_lcd", NULL),
	REGULATOR_SUPPLY("vddio_audio", NULL),
	REGULATOR_SUPPLY("pwrdet_audio", NULL),
	REGULATOR_SUPPLY("vddio_bb", NULL),
	REGULATOR_SUPPLY("pwrdet_bb", NULL),
	REGULATOR_SUPPLY("vddio_gmi", NULL),
	REGULATOR_SUPPLY("avdd_usb_pll", NULL),
	REGULATOR_SUPPLY("vddio_cam", NULL),
	REGULATOR_SUPPLY("pwrdet_cam", NULL),
	REGULATOR_SUPPLY("vddio_sdmmc1", NULL),
	REGULATOR_SUPPLY("pwrdet_sdmmc1", NULL),
	REGULATOR_SUPPLY("vddio_sdmmc4", NULL),
	REGULATOR_SUPPLY("pwrdet_sdmmc4", NULL),
	REGULATOR_SUPPLY("avdd_hdmi_pll", NULL),
	REGULATOR_SUPPLY("vddio_gps", NULL),
	REGULATOR_SUPPLY("vdd_lcd_buffered", NULL),
	REGULATOR_SUPPLY("vddio_nand", NULL),
	REGULATOR_SUPPLY("pwrdet_nand", NULL),
	REGULATOR_SUPPLY("vddio_sd", NULL),
	REGULATOR_SUPPLY("vdd_bat", NULL),
	REGULATOR_SUPPLY("vdd_io", NULL),
	REGULATOR_SUPPLY("pwrdet_pex_ctl", NULL),
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
	REGULATOR_SUPPLY("pwrdet_sdmmc3", NULL),
};

static struct regulator_consumer_supply tps80031_vana_supply[] = {
	REGULATOR_SUPPLY("unused_vana", NULL),
};

static struct regulator_consumer_supply tps80031_ldo1_supply[] = {
	REGULATOR_SUPPLY("avdd_dsi_csi", NULL),
	REGULATOR_SUPPLY("pwrdet_mipi", NULL),
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

static struct regulator_consumer_supply tps80031_vbus_supply[] = {
	REGULATOR_SUPPLY("usb_vbus", NULL),
};

static struct regulator_consumer_supply tps80031_battery_charge_supply[] = {
	REGULATOR_SUPPLY("usb_bat_chg", NULL),
};

#define TPS_PDATA_INIT(_id, _minmv, _maxmv, _supply_reg, _always_on,	\
	_boot_on, _apply_uv, _init_uV, _init_enable, _init_apply, 	\
	_flags, _ectrl, _delay)						\
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
		.flags = _flags,					\
		.ext_ctrl_flag = _ectrl,				\
		.delay_us = _delay,					\
	}

TPS_PDATA_INIT(vio,   600, 2100, 0, 1, 0, 0, -1, 0, 0, 0, 0, 0);
TPS_PDATA_INIT(smps1, 600, 2100, 0, 0, 0, 0, -1, 0, 0, 0, PWR_REQ_INPUT_PREQ2 | PWR_OFF_ON_SLEEP, 0);
TPS_PDATA_INIT(smps2, 600, 2100, 0, 0, 0, 0, -1, 0, 0, 0, PWR_REQ_INPUT_PREQ1, 0);
TPS_PDATA_INIT(smps3, 600, 2100, 0, 1, 0, 0, -1, 0, 0, 0, 0, 0);
TPS_PDATA_INIT(smps4, 600, 2100, 0, 0, 0, 0, -1, 0, 0, 0, PWR_REQ_INPUT_PREQ1, 0);
TPS_PDATA_INIT(ldo1, 1000, 3300, tps80031_rails(VIO), 0, 0, 0, -1, 0, 0, 0, 0, 0);
TPS_PDATA_INIT(ldo2, 1000, 3300, 0, 1, 1, 1, 1000, 1, 1, 0, 0, 0);
TPS_PDATA_INIT(ldo3, 1000, 3300, tps80031_rails(VIO), 0, 0, 0, -1, 0, 0, 0, PWR_OFF_ON_SLEEP, 0);
TPS_PDATA_INIT(ldo4, 1000, 3300, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0);
TPS_PDATA_INIT(ldo5, 1000, 3300, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0);
TPS_PDATA_INIT(ldo6, 1000, 3300, 0, 0, 0, 0, -1, 0, 0, 0, PWR_REQ_INPUT_PREQ1, 0);
TPS_PDATA_INIT(ldo7, 1000, 3300, tps80031_rails(VIO), 0, 0, 0, -1, 0, 0, 0, PWR_REQ_INPUT_PREQ1, 0);
TPS_PDATA_INIT(ldoln, 1000, 3300, tps80031_rails(SMPS3), 0, 0, 0, -1, 0, 0, 0, PWR_REQ_INPUT_PREQ1, 0);
TPS_PDATA_INIT(ldousb, 1000, 3300, 0, 0, 0, 0, -1, 0, 0, USBLDO_INPUT_VSYS, PWR_OFF_ON_SLEEP, 0);
TPS_PDATA_INIT(vana,  1000, 3300, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0);
TPS_PDATA_INIT(vbus,  0, 5000, 0, 0, 0, 0, -1, 0, 0, (VBUS_SW_ONLY | VBUS_DISCHRG_EN_PDN), 0, 100000);

static struct tps80031_rtc_platform_data rtc_data = {
	.irq = ENT_TPS80031_IRQ_BASE + TPS80031_INT_RTC_ALARM,
	.time = {
		.tm_year = 2011,
		.tm_mon = 0,
		.tm_mday = 1,
		.tm_hour = 1,
		.tm_min = 2,
		.tm_sec = 3,
	},
};

int battery_charger_init(void *board_data)
{
	int ret;
	ret = gpio_request(TEGRA_GPIO_PF6, "lcd_d14-bat_charge");
	if (ret < 0) {
		pr_err("%s() The gpio_request for battery"
				" charger fails\n", __func__);
	}
	gpio_direction_output(TEGRA_GPIO_PF6, 1);
	tegra_gpio_enable(TEGRA_GPIO_PF6);
	return 0;
}

static struct tps80031_charger_platform_data bcharger_pdata = {
	.max_charge_volt_mV = 4100,
	.max_charge_current_mA = 1000,
	.charging_term_current_mA = 100,
	.watch_time_sec = 100,
	.irq_base = ENT_TPS80031_IRQ_BASE,
	.consumer_supplies = tps80031_battery_charge_supply,
	.num_consumer_supplies = ARRAY_SIZE(tps80031_battery_charge_supply),
	.board_init = battery_charger_init,
	.board_data = NULL,
};

static struct tps80031_bg_platform_data battery_gauge_data = {
	.irq_base = ENT_TPS80031_IRQ_BASE,
	.battery_present = 1,
};

#define TPS_RTC()				\
	{						\
		.id	= 0,		\
		.name	= "rtc_tps80031",	\
		.platform_data = &rtc_data,	\
	}

#define TPS_REG(_id, _data)				\
	{						\
		.id	 = TPS80031_ID_##_id,		\
		.name   = "tps80031-regulator",		\
		.platform_data  = &pdata_##_data,	\
	}
#define TPS_BATTERY()					\
	{						\
		.name   = "tps80031-charger",		\
		.platform_data = &bcharger_pdata,	\
	}
#define TPS_BATTERY_GAUGE()				\
	{						\
		.name   = "tps80031-battery-gauge",	\
		.platform_data = &battery_gauge_data,	\
	}
#define TPS_GPADC()					\
	{						\
		.name	= "tps80031-gpadc",		\
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
	TPS_REG(VBUS, vbus),
	TPS_RTC(),
	TPS_BATTERY(),
	TPS_BATTERY_GAUGE(),
	TPS_GPADC(),
};

struct tps80031_clk32k_init_data clk32k_idata[] = {
	{
		.clk32k_nr = TPS80031_CLOCK32K_G,
		.enable = true,
		.ext_ctrl_flag = PWR_REQ_INPUT_PREQ1,
	},
	{
		.clk32k_nr = TPS80031_CLOCK32K_AUDIO,
		.enable = true,
		.ext_ctrl_flag = PWR_REQ_INPUT_PREQ1,
	},
};

static struct tps80031_platform_data tps_platform = {
	.num_subdevs	= ARRAY_SIZE(tps80031_devs),
	.subdevs	= tps80031_devs,
	.irq_base	= ENT_TPS80031_IRQ_BASE,
	.gpio_base	= ENT_TPS80031_GPIO_BASE,
	.clk32k_init_data	= clk32k_idata,
	.clk32k_init_data_size	= ARRAY_SIZE(clk32k_idata),
};

static struct i2c_board_info __initdata enterprise_regulators[] = {
	{
		I2C_BOARD_INFO("tps80031", 0x4A),
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
	REGULATOR_SUPPLY("vdd", "4-004c"),
};
static int gpio_switch_pmu_3v3_en_voltages[] = {3300};

/* SYSEN from PMU*/
static struct regulator_consumer_supply gpio_switch_pmu_hdmi_5v0_en_supply[] = {
	REGULATOR_SUPPLY("hdmi_5v0", NULL),
};
static int gpio_switch_pmu_hdmi_5v0_en_voltages[] = {5000};

/* LCD-D16 (GPIO M0) from T30*/
static struct regulator_consumer_supply gpio_switch_vdd_fuse_en_supply[] = {
	REGULATOR_SUPPLY("vdd_fuse", NULL),
};
static int gpio_switch_vdd_fuse_en_voltages[] = {3300};

/* LCD-D17 (GPIO M1) from T30*/
static struct regulator_consumer_supply gpio_switch_sdmmc3_vdd_sel_supply[] = {
	REGULATOR_SUPPLY("vddio_sdmmc3_2v85_1v8", NULL),
	REGULATOR_SUPPLY("sdmmc3_compu_pu", NULL),
	REGULATOR_SUPPLY("vddio_sdmmc3", NULL),
	REGULATOR_SUPPLY("vsys_3v7", NULL),
};
static int gpio_switch_sdmmc3_vdd_sel_voltages[] = {2850};

/* LCD-D23 (GPIO M7) from T30*/
/* 2-0036 is dev_name of ar0832 in Enterprise A01*/
/* 2-0032 is alternative dev_name of ar0832 Enterprise A01*/
/* 2-0010 is dev_name of ov9726 */
/* 2-0070 is dev_name of PCA9546 in Enterprise A02*/
/* 6-0036 is dev_name of ar0832 in Enterprise A02 */
/* 7-0036 is dev_name of ar0832 in Enterprise A02 */
static struct regulator_consumer_supply gpio_switch_cam_ldo_2v8_en_supply[] = {
	REGULATOR_SUPPLY("vaa", "2-0036"),
	REGULATOR_SUPPLY("vaa", "2-0032"),
	REGULATOR_SUPPLY("avdd", "2-0010"),
	REGULATOR_SUPPLY("vdd_2v8_cam", NULL),
	REGULATOR_SUPPLY("vcc", "2-0070"),
	REGULATOR_SUPPLY("vaa", "6-0036"),
	REGULATOR_SUPPLY("vaa", "7-0036"),
};
static int gpio_switch_cam_ldo_2v8_en_voltages[] = {2800};

/* LCD-D9 (GPIO F1) from T30*/
/* 2-0036 is dev_name of ar0832 in Enterprise A01*/
/* 2-0032 is alternative dev_name of ar0832 Enterprise A01*/
/* 2-0010 is dev_name of ov9726 */
/* 2-0033 is dev_name of tps61050 */
/* 2-0070 is dev_name of PCA9546 in Enterprise A02*/
/* 6-0036 is dev_name of ar0832 in Enterprise A02 */
/* 7-0036 is dev_name of ar0832 in Enterprise A02 */
static struct regulator_consumer_supply gpio_switch_cam_ldo_1v8_en_supply[] = {
	REGULATOR_SUPPLY("vdd", "2-0036"),
	REGULATOR_SUPPLY("vdd", "2-0032"),
	REGULATOR_SUPPLY("dovdd", "2-0010"),
	REGULATOR_SUPPLY("vdd_1v8_cam", NULL),
	REGULATOR_SUPPLY("vdd_i2c", "2-0033"),
	REGULATOR_SUPPLY("vcc_i2c", "2-0070"),
	REGULATOR_SUPPLY("vdd", "6-0036"),
	REGULATOR_SUPPLY("vdd", "7-0036"),
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

GREG_INIT(0, pmu_5v15_en,     NULL,      ENT_TPS80031_GPIO_REGEN1, false, 0, 0, 0, 0);
GREG_INIT(1, pmu_3v3_en,      "vdd_5v15", ENT_TPS80031_GPIO_REGEN2, false, 0, 0, 0, 0);
GREG_INIT(2, pmu_hdmi_5v0_en, "vdd_5v15", ENT_TPS80031_GPIO_SYSEN, false, 0, 0, 0, 0);

GREG_INIT(3, vdd_fuse_en, "avdd_usb_hdmi_3v3", TEGRA_GPIO_PM0, false, 0, 0, 0, 0);
GREG_INIT(4, sdmmc3_vdd_sel, "vddio_sdmmc_2v85", TEGRA_GPIO_PM1, false, 0, 0, 0, 0);
GREG_INIT(5, cam_ldo_2v8_en, NULL, TEGRA_GPIO_PM7, false, 0, 0, 0, 0);
GREG_INIT(6, cam_ldo_1v8_en, NULL, TEGRA_GPIO_PF1, false, 0, 0, 0, 0);

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

static void enterprise_power_off(void)
{
	int ret;
	pr_info("enterprise: Powering off the device\n");
	ret = tps80031_power_off();
	if (ret)
		pr_err("enterprise: failed to power off\n");
	while(1);
}

void __init enterprise_tsensor_init(void)
{
	tegra3_tsensor_init(NULL);
}

int __init enterprise_regulator_init(void)
{
	void __iomem *pmc = IO_ADDRESS(TEGRA_PMC_BASE);
	u32 pmc_ctrl;
	u32 pmc_dpd_pads;

	/* configure the power management controller to trigger PMU
	 * interrupts when low */

	pmc_ctrl = readl(pmc + PMC_CTRL);
	writel(pmc_ctrl | PMC_CTRL_INTR_LOW, pmc + PMC_CTRL);

	pmc_dpd_pads = readl(pmc + PMC_DPD_PADS_ORIDE);
	writel(pmc_dpd_pads & ~PMC_DPD_PADS_ORIDE_BLINK , pmc + PMC_DPD_PADS_ORIDE);

	/* Disable battery charging if power adapter is connected. */
	if (get_power_supply_type() == POWER_SUPPLY_TYPE_MAINS) {
		bcharger_pdata.num_consumer_supplies = 0;
		bcharger_pdata.consumer_supplies = NULL;
		battery_gauge_data.battery_present = 0;
	}

	i2c_register_board_info(4, enterprise_regulators, 1);
	enterprise_gpio_switch_regulator_init();
	pm_power_off = enterprise_power_off;

	return 0;
}

static void enterprise_board_suspend(int lp_state, enum suspend_stage stg)
{
	if ((lp_state == TEGRA_SUSPEND_LP1) && (stg == TEGRA_SUSPEND_BEFORE_CPU))
		tegra_console_uart_suspend();
}

static void enterprise_board_resume(int lp_state, enum resume_stage stg)
{
	if ((lp_state == TEGRA_SUSPEND_LP1) && (stg == TEGRA_RESUME_AFTER_CPU))
		tegra_console_uart_resume();
}

static struct tegra_suspend_platform_data enterprise_suspend_data = {
	.cpu_timer	= 2000,
	.cpu_off_timer	= 200,
	.suspend_mode	= TEGRA_SUSPEND_LP0,
	.core_timer	= 0x7e7e,
	.core_off_timer = 0,
	.corereq_high	= true,
	.sysclkreq_high	= true,
	.board_suspend = enterprise_board_suspend,
	.board_resume = enterprise_board_resume,
};

static void enterprise_init_deep_sleep_mode(void)
{
	struct board_info bi;
	tegra_get_board_info(&bi);

	if (bi.board_id == BOARD_E1205 && bi.fab == BOARD_FAB_A01)
		enterprise_suspend_data.suspend_mode = TEGRA_SUSPEND_LP1;

	if ((bi.board_id == BOARD_E1205 && (bi.sku & BOARD_SKU_VF_BIT) == 0) ||
	    (bi.board_id == BOARD_E1197 && (bi.sku & BOARD_SKU_VF_BIT)))
		enterprise_suspend_data.cpu_timer = 8000;
}

int __init enterprise_suspend_init(void)
{
	enterprise_init_deep_sleep_mode();
	tegra_init_suspend(&enterprise_suspend_data);
	return 0;
}

#ifdef CONFIG_TEGRA_EDP_LIMITS

int __init enterprise_edp_init(void)
{
	unsigned int regulator_mA;

	regulator_mA = get_maximum_cpu_current_supported();
	if (!regulator_mA) {
		regulator_mA = 2500; /* regular AP30 */
	}
	pr_info("%s: CPU regulator %d mA\n", __func__, regulator_mA);

	tegra_init_cpu_edp_limits(regulator_mA);
	tegra_init_system_edp_limits(TEGRA_BPC_CPU_PWR_LIMIT);
	return 0;
}
#endif

static struct tegra_bpc_mgmt_platform_data bpc_mgmt_platform_data = {
	.gpio_trigger = TEGRA_BPC_TRIGGER,
	.bpc_mgmt_timeout = TEGRA_BPC_TIMEOUT,
};

static struct platform_device enterprise_bpc_mgmt_device = {
	.name		= "tegra-bpc-mgmt",
	.id		= -1,
	.dev		= {
		.platform_data = &bpc_mgmt_platform_data,
	},
};

void __init enterprise_bpc_mgmt_init(void)
{
	int int_gpio;

	tegra_gpio_enable(TEGRA_BPC_TRIGGER);

	int_gpio = tegra_gpio_to_int_pin(TEGRA_BPC_TRIGGER);

#ifdef CONFIG_SMP
	cpumask_setall(&(bpc_mgmt_platform_data.affinity_mask));
	irq_set_affinity_hint(int_gpio,
				&(bpc_mgmt_platform_data.affinity_mask));
	irq_set_affinity(int_gpio, &(bpc_mgmt_platform_data.affinity_mask));
#endif
	platform_device_register(&enterprise_bpc_mgmt_device);

	return;
}
