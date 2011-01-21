/*
 * arch/arm/mach-tegra/board-cardhu-power.c
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
#include <linux/mfd/tps6591x.h>
#include <linux/gpio.h>
#include <mach/suspend.h>
#include <linux/io.h>

#include <mach/iomap.h>
#include <mach/irqs.h>

#include "gpio-names.h"
#include "board-cardhu.h"
#include "power.h"
#include "wakeups-t3.h"

#ifdef CONFIG_MFD_TPS6591X
static struct regulator_consumer_supply tps6591x_vdd1_supply[] = {
	REGULATOR_SUPPLY("vdd_core", NULL),
	REGULATOR_SUPPLY("en_vddio_ddr_1v2", NULL),
};

static struct regulator_consumer_supply tps6591x_vdd2_supply[] = {
	REGULATOR_SUPPLY("vdd_gen1v5", NULL),
	REGULATOR_SUPPLY("vcore_lcd", NULL),
	REGULATOR_SUPPLY("track_ldo1", NULL),
	REGULATOR_SUPPLY("external_ldo_1v2", NULL),
	REGULATOR_SUPPLY("vcore_cam1", NULL),
	REGULATOR_SUPPLY("vcore_cam2", NULL),
};

static struct regulator_consumer_supply tps6591x_vddctrl_supply[] = {
	REGULATOR_SUPPLY("vdd_cpu_pmu", NULL),
};

static struct regulator_consumer_supply tps6591x_vio_supply[] = {
	REGULATOR_SUPPLY("avdd_hdmi_pll", NULL),
	REGULATOR_SUPPLY("avdd_usb_pll", NULL),
	REGULATOR_SUPPLY("avdd_osc", NULL),
	REGULATOR_SUPPLY("vddio_sys", NULL),
	REGULATOR_SUPPLY("vddio_sdmmc4", NULL),
	REGULATOR_SUPPLY("vdd1v8_satelite", NULL),
	REGULATOR_SUPPLY("vddio_uart", NULL),
	REGULATOR_SUPPLY("vddio_audio", NULL),
	REGULATOR_SUPPLY("vddio_bb", NULL),
	REGULATOR_SUPPLY("vddio_lcd_pmu", NULL),
	REGULATOR_SUPPLY("vddio_cam", NULL),
	REGULATOR_SUPPLY("vddio_vi", NULL),
	REGULATOR_SUPPLY("ldo6", NULL),
	REGULATOR_SUPPLY("ldo7", NULL),
	REGULATOR_SUPPLY("ldo8", NULL),
	REGULATOR_SUPPLY("vcore_audio", NULL),
	REGULATOR_SUPPLY("avcore_audio", NULL),
	REGULATOR_SUPPLY("vddio_sdmmc3", NULL),
	REGULATOR_SUPPLY("vcore1_lpddr2", NULL),
	REGULATOR_SUPPLY("vcom_1v8", NULL),
	REGULATOR_SUPPLY("pmuio_1v8", NULL),
	REGULATOR_SUPPLY("avdd_ic_usb", NULL),
};

static struct regulator_consumer_supply tps6591x_ldo1_supply[] = {
	REGULATOR_SUPPLY("avdd_pexb", NULL),
	REGULATOR_SUPPLY("vdd_pexb", NULL),
	REGULATOR_SUPPLY("avdd_pex_pll", NULL),
	REGULATOR_SUPPLY("avdd_pexa", NULL),
	REGULATOR_SUPPLY("vdd_pexa", NULL),
};

static struct regulator_consumer_supply tps6591x_ldo2_supply[] = {
	REGULATOR_SUPPLY("avdd_sata", NULL),
	REGULATOR_SUPPLY("vdd_sata", NULL),
	REGULATOR_SUPPLY("avdd_sata_pll", NULL),
	REGULATOR_SUPPLY("avdd_plle", NULL),
};

static struct regulator_consumer_supply tps6591x_ldo3_supply[] = {
	REGULATOR_SUPPLY("vddio_sdmmc1", NULL),
};

static struct regulator_consumer_supply tps6591x_ldo4_supply[] = {
	REGULATOR_SUPPLY("vdd_rtc", NULL),
};

static struct regulator_consumer_supply tps6591x_ldo5_supply[] = {
	REGULATOR_SUPPLY("avdd_vdac", NULL),
};
static struct regulator_consumer_supply tps6591x_ldo6_supply[] = {
	REGULATOR_SUPPLY("avdd_dsi_csi", NULL),
};
static struct regulator_consumer_supply tps6591x_ldo7_supply[] = {
	REGULATOR_SUPPLY("avdd_plla_p_c_s", NULL),
	REGULATOR_SUPPLY("avdd_pllm", NULL),
	REGULATOR_SUPPLY("avdd_pllu_d", NULL),
	REGULATOR_SUPPLY("avdd_pllu_d2", NULL),
	REGULATOR_SUPPLY("avdd_pllx", NULL),
};

static struct regulator_consumer_supply tps6591x_ldo8_supply[] = {
	REGULATOR_SUPPLY("vdd_ddr_hs", NULL),
};

#define REGULATOR_INIT(_id, _minmv, _maxmv)				\
	{								\
		.constraints = {					\
			.min_uV = (_minmv)*1000,			\
			.max_uV = (_maxmv)*1000,			\
			.valid_modes_mask = (REGULATOR_MODE_NORMAL |	\
					     REGULATOR_MODE_STANDBY),	\
			.valid_ops_mask = (REGULATOR_CHANGE_MODE |	\
					   REGULATOR_CHANGE_STATUS |	\
					   REGULATOR_CHANGE_VOLTAGE),	\
		},							\
		.num_consumer_supplies = ARRAY_SIZE(tps6591x_##_id##_supply),\
		.consumer_supplies = tps6591x_##_id##_supply,		\
	}

#define REGULATOR_INIT_SUPPLY(_id, _minmv, _maxmv, _supply_reg)		\
	{								\
		.constraints = {					\
			.min_uV = (_minmv)*1000,			\
			.max_uV = (_maxmv)*1000,			\
			.valid_modes_mask = (REGULATOR_MODE_NORMAL |	\
					     REGULATOR_MODE_STANDBY),	\
			.valid_ops_mask = (REGULATOR_CHANGE_MODE |	\
					   REGULATOR_CHANGE_STATUS |	\
					   REGULATOR_CHANGE_VOLTAGE),	\
		},							\
		.num_consumer_supplies = ARRAY_SIZE(tps6591x_##_id##_supply),\
		.consumer_supplies = tps6591x_##_id##_supply,		\
		.supply_regulator = tps6591x_rails(_supply_reg),	\
	}

static struct regulator_init_data vdd1_data = REGULATOR_INIT(vdd1, 600, 1500);
static struct regulator_init_data vdd2_data = REGULATOR_INIT(vdd2, 600, 1500);
static struct regulator_init_data vddctrl_data = REGULATOR_INIT(vddctrl, 600,
						1400);
static struct regulator_init_data vio_data = REGULATOR_INIT(vio, 1500, 3300);
static struct regulator_init_data ldo1_data = REGULATOR_INIT_SUPPLY(ldo1,
						1000, 3300, VDD_2);
static struct regulator_init_data ldo2_data = REGULATOR_INIT_SUPPLY(ldo2,
						1000, 3300, VDD_2);
static struct regulator_init_data ldo3_data = REGULATOR_INIT(ldo3, 1000, 3300);
static struct regulator_init_data ldo4_data = REGULATOR_INIT(ldo4, 1000, 3300);
static struct regulator_init_data ldo5_data = REGULATOR_INIT(ldo5, 1000, 3300);
static struct regulator_init_data ldo6_data = REGULATOR_INIT_SUPPLY(ldo6,
						1000, 3300, VIO);
static struct regulator_init_data ldo7_data = REGULATOR_INIT_SUPPLY(ldo7,
						1000, 3300, VIO);
static struct regulator_init_data ldo8_data = REGULATOR_INIT_SUPPLY(ldo8,
						1000, 3300, VIO);

static struct tps6591x_rtc_platform_data rtc_data = {
	.irq = TEGRA_NR_IRQS + TPS6591X_INT_RTC_ALARM,
};

#define TPS_REG(_id, _data)			\
	{					\
		.id	= TPS6591X_ID_##_id,	\
		.name	= "tps6591x-regulator",	\
		.platform_data	= _data,	\
	}

static struct tps6591x_subdev_info tps_devs[] = {
	TPS_REG(VIO, &vio_data),
	TPS_REG(VDD_1, &vdd1_data),
	TPS_REG(VDD_2, &vdd2_data),
	TPS_REG(VDDCTRL, &vddctrl_data),
	TPS_REG(LDO_1, &ldo1_data),
	TPS_REG(LDO_2, &ldo2_data),
	TPS_REG(LDO_3, &ldo3_data),
	TPS_REG(LDO_4, &ldo4_data),
	TPS_REG(LDO_5, &ldo5_data),
	TPS_REG(LDO_6, &ldo6_data),
	TPS_REG(LDO_7, &ldo7_data),
	TPS_REG(LDO_8, &ldo8_data),
	{
		.id	= 0,
		.name	= "tps6591x-rtc",
		.platform_data = &rtc_data,
	},
};

static struct tps6591x_platform_data tps_platform = {
	.irq_base	= TEGRA_NR_IRQS,
	.num_subdevs	= ARRAY_SIZE(tps_devs),
	.subdevs	= tps_devs,
	.gpio_base	= TPS6591X_GPIO_BASE,
};

static struct i2c_board_info __initdata cardhu_regulators[] = {
	{
		I2C_BOARD_INFO("tps6591x", 0x2D),
		.irq		= INT_EXTERNAL_PMU,
		.platform_data	= &tps_platform,
	},
};

int __init cardhu_regulator_init(void)
{
	i2c_register_board_info(4, cardhu_regulators, 1);
	return 0;
}
#else
int __init cardhu_regulator_init(void)
{
	return 0;
}
#endif

static struct tegra_suspend_platform_data cardhu_suspend_data = {
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

int __init cardhu_suspend_init(void)
{
	tegra_init_suspend(&cardhu_suspend_data);
	return 0;
}
