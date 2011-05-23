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
#include <linux/mfd/tps80031x.h>
#include <linux/gpio.h>
#include <mach/suspend.h>
#include <linux/io.h>
#include <linux/i2c/twl.h>

#include <mach/iomap.h>
#include <mach/irqs.h>
#include <mach/pinmux.h>

#include "gpio-names.h"
#include "board-enterprise.h"
#include "power.h"
#include "wakeups-t3.h"

#ifdef CONFIG_TWL4030_CORE
static struct regulator_consumer_supply tps80031x_smps4_supply[] = {
	REGULATOR_SUPPLY("avdd_dsi_csi", NULL),
};

static struct regulator_consumer_supply tps80031x_vio_supply[] = {
	REGULATOR_SUPPLY("avdd_osc", NULL),
	REGULATOR_SUPPLY("vddio_sys", NULL),
	REGULATOR_SUPPLY("vddio_uart", NULL),
	REGULATOR_SUPPLY("vddio_audio", NULL),
	REGULATOR_SUPPLY("vddio_bb", NULL),
	REGULATOR_SUPPLY("vddio_cam", NULL),
	REGULATOR_SUPPLY("vddio_sdmmc1", NULL),
};

static struct regulator_consumer_supply tps80031x_smps3_supply[] = {
	REGULATOR_SUPPLY("en_vddio_ddr_1v2", NULL),
};


static struct regulator_consumer_supply tps80031x_ldo2_supply[] = {
	REGULATOR_SUPPLY("vdd_rtc", NULL),
};

static struct regulator_consumer_supply tps80031x_ldo6_supply[] = {
	REGULATOR_SUPPLY("vddio_sdmmc3", NULL),
};

static struct regulator_consumer_supply tps80031x_ldousb_supply[] = {
	REGULATOR_SUPPLY("avdd_hdmi_pll", NULL),
};

static struct regulator_consumer_supply tps80031x_ldo7_supply[] = {
	REGULATOR_SUPPLY("avdd_plla_p_c_s", NULL),
	REGULATOR_SUPPLY("avdd_pllm", NULL),
	REGULATOR_SUPPLY("avdd_pllu_d", NULL),
	REGULATOR_SUPPLY("avdd_pllx", NULL),
};

#define TPS_PDATA_INIT(_id, _minmv, _maxmv, _supply_reg, _always_on,	\
	_boot_on, _apply_uv) \
	{								\
		.constraints = {				\
		    .min_uV = (_minmv)*1000,		\
			.max_uV = (_maxmv)*1000,		\
			.valid_modes_mask = (REGULATOR_MODE_NORMAL |  \
						     REGULATOR_MODE_STANDBY), \
			.valid_ops_mask = (REGULATOR_CHANGE_MODE |    \
						   REGULATOR_CHANGE_STATUS |  \
						   REGULATOR_CHANGE_VOLTAGE), \
			.always_on = _always_on,		\
			.boot_on = _boot_on,			\
			.apply_uV = _apply_uv,			\
		},						\
		.num_consumer_supplies =			\
			ARRAY_SIZE(tps80031x_##_id##_supply),	\
		.consumer_supplies = tps80031x_##_id##_supply,	\
	}

static struct regulator_init_data smps4_data = TPS_PDATA_INIT(smps4,    600, 2100, 0, 1, 1, 0);
static struct regulator_init_data vio_data = TPS_PDATA_INIT(vio,    0, 1800, 0, 1, 1, 0);
static struct regulator_init_data smps3_data = TPS_PDATA_INIT(smps3, 600, 2100, 0, 1, 1, 0);
static struct regulator_init_data ldo2_data = TPS_PDATA_INIT(ldo2, 1000, 3300, 0, 1, 1, 0);
static struct regulator_init_data ldo6_data = TPS_PDATA_INIT(ldo6, 1000, 3300, 0, 1, 1, 0);
static struct regulator_init_data ldousb_data = TPS_PDATA_INIT(ldousb, 1000, 3300, 0, 1, 1, 0);
static struct regulator_init_data ldo7_data = TPS_PDATA_INIT(ldo7, 1000, 3300, 0, 1, 1, 0);

static struct twl4030_clock_init_data clk_data = {
	.ck32k_lowpwr_enable = 0,
	.clk32_active_state_on = 1,
};

static struct twl4030_platform_data tps_platform = {
	.clock	= &clk_data,
	.smps4	= &smps4_data,
	.vio	= &vio_data,
	.smps3	= &smps3_data,
	.ldo2	= &ldo2_data,
	.ldo6	= &ldo6_data,
	.ldousb	= &ldousb_data,
	.ldo7	= &ldo7_data,
};

static struct i2c_board_info __initdata enterprise_regulators[] = {
	{
		I2C_BOARD_INFO("mpu80031", 0x48),
		.irq		= INT_EXTERNAL_PMU,
		.platform_data	= &tps_platform,
	},
};

int __init enterprise_regulator_init(void)
{
	i2c_register_board_info(4, enterprise_regulators, 1);
	return 0;
}
#else
int __init enterprise_regulator_init(void)
{
	return 0;
}
#endif

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
