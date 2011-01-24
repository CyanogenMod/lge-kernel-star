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
#include <linux/gpio-switch-regulator.h>

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
	REGULATOR_SUPPLY("vdd_cpu", NULL),
	REGULATOR_SUPPLY("vdd_sys", NULL),
};

static struct regulator_consumer_supply tps6591x_vio_supply[] = {
	REGULATOR_SUPPLY("vdd_gen1v8", NULL),
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

#ifdef CONFIG_REGULATOR_GPIO_SWITCH

/* EN_5V_CP from PMU GP0 */
static struct regulator_consumer_supply gpio_switch_en_5v_cp_supply[] = {
	REGULATOR_SUPPLY("vdd_5v0_sby", NULL),
	REGULATOR_SUPPLY("vdd_hall", NULL),
	REGULATOR_SUPPLY("vterm_ddr", NULL),
	REGULATOR_SUPPLY("v2ref_ddr", NULL),
};
static int gpio_switch_en_5v_cp_voltages[] = { 5000};

/* EN_5V0 From PMU GP2 */
static struct regulator_consumer_supply gpio_switch_en_5v0_supply[] = {
	REGULATOR_SUPPLY("vdd_5v0_sys", NULL),
};
static int gpio_switch_en_5v0_voltages[] = { 5000};

/* EN_DDR From PMU GP6 */
static struct regulator_consumer_supply gpio_switch_en_ddr_supply[] = {
	REGULATOR_SUPPLY("mem_vddio_ddr", NULL),
	REGULATOR_SUPPLY("t30_vddio_ddr", NULL),
};
static int gpio_switch_en_ddr_voltages[] = { 1500};

/* EN_3V3_SYS From PMU GP7 */
static struct regulator_consumer_supply gpio_switch_en_3v3_sys_supply[] = {
	REGULATOR_SUPPLY("vdd_lvds", NULL),
	REGULATOR_SUPPLY("vdd_pnl", NULL),
	REGULATOR_SUPPLY("vcom_3v3", NULL),
	REGULATOR_SUPPLY("vdd_3v3", NULL),
	REGULATOR_SUPPLY("vcore_mmc", NULL),
	REGULATOR_SUPPLY("vddo_pex_ctl", NULL),
	REGULATOR_SUPPLY("hvdd_pex", NULL),
	REGULATOR_SUPPLY("avdd_hdmi", NULL),
	REGULATOR_SUPPLY("vpp_fuse", NULL),
	REGULATOR_SUPPLY("avdd_usb", NULL),
	REGULATOR_SUPPLY("vdd_ddr_rx", NULL),
	REGULATOR_SUPPLY("vcore_nand", NULL),
	REGULATOR_SUPPLY("hvdd_sata", NULL),
	REGULATOR_SUPPLY("vddio_gmi_pmu", NULL),
	REGULATOR_SUPPLY("avdd_cam1", NULL),
	REGULATOR_SUPPLY("vdd_af", NULL),
	REGULATOR_SUPPLY("avdd_cam2", NULL),
	REGULATOR_SUPPLY("vdd_acc", NULL),
	REGULATOR_SUPPLY("vdd_phtl", NULL),
	REGULATOR_SUPPLY("vddio_tp", NULL),
	REGULATOR_SUPPLY("vdd_led", NULL),
	REGULATOR_SUPPLY("vddio_cec", NULL),
	REGULATOR_SUPPLY("vdd_cmps", NULL),
	REGULATOR_SUPPLY("vdd_temp", NULL),
	REGULATOR_SUPPLY("vpp_kfuse", NULL),
	REGULATOR_SUPPLY("vddio_ts", NULL),
	REGULATOR_SUPPLY("vdd_ir_led", NULL),
	REGULATOR_SUPPLY("vddio_1wire", NULL),
	REGULATOR_SUPPLY("avddio_audio", NULL),
	REGULATOR_SUPPLY("vdd_ec", NULL),
	REGULATOR_SUPPLY("vcom_pa", NULL),
	REGULATOR_SUPPLY("vdd_3v3_devices", NULL),
	REGULATOR_SUPPLY("vdd_3v3_dock", NULL),
	REGULATOR_SUPPLY("debug_cons", NULL),
};
static int gpio_switch_en_3v3_sys_voltages[] = { 3300};

/* EN_VDD_BL from AP GPIO GMI_CS2  K03 */
static struct regulator_consumer_supply gpio_switch_en_vdd_bl_supply[] = {
	REGULATOR_SUPPLY("vdd_backlight", NULL),
};
static int gpio_switch_en_vdd_bl_voltages[] = { 5000};

/* EN_3V3_MODEM from AP GPIO VI_VSYNCH D06*/
static struct regulator_consumer_supply gpio_switch_en_3v3_modem_supply[] = {
	REGULATOR_SUPPLY("vdd_3v3_mini_card", NULL),
	REGULATOR_SUPPLY("vdd_mini_card", NULL),
};
static int gpio_switch_en_3v3_modem_voltages[] = { 3300};

/* EN_USB1_VBUS_OC from AP GPIO GMI_RST I04*/
static struct regulator_consumer_supply gpio_switch_en_usb1_vbus_oc_supply[] = {
	REGULATOR_SUPPLY("vdd_vbus_micro_usb", NULL),
};
static int gpio_switch_en_usb1_vbus_oc_voltages[] = { 5000};

/*EN_USB3_VBUS_OC from AP GPIO GMI_AD15 H07*/
static struct regulator_consumer_supply gpio_switch_en_usb3_vbus_oc_supply[] = {
	REGULATOR_SUPPLY("vdd_vbus_typea_usb", NULL),
};
static int gpio_switch_en_usb3_vbus_oc_voltages[] = { 5000};

/* EN_VDDIO_VID_OC from AP GPIO VI_PCLK T00*/
static struct regulator_consumer_supply gpio_switch_en_vddio_vid_oc_supply[] = {
	REGULATOR_SUPPLY("vdd_hdmi_con", NULL),
};
static int gpio_switch_en_vddio_vid_oc_voltages[] = { 5000};

/* EN_VDD_PNL1 from AP GPIO VI_D6 L04*/
static struct regulator_consumer_supply gpio_switch_en_vdd_pnl1_supply[] = {
	REGULATOR_SUPPLY("vdd_lcd_panel", NULL),
};
static int gpio_switch_en_vdd_pnl1_voltages[] = { 3300};

/* CAM3_LDO_EN from AP GPIO KB_ROW8 S00*/
static struct regulator_consumer_supply gpio_switch_cam3_ldo_en_supply[] = {
	REGULATOR_SUPPLY("vdd_cam3", NULL),
};
static int gpio_switch_cam3_ldo_en_voltages[] = { 3300};

/* EN_VDD_COM from AP GPIO SDMMC3_DAT5 D00*/
static struct regulator_consumer_supply gpio_switch_en_vdd_com_supply[] = {
	REGULATOR_SUPPLY("vdd_cwcom_bd", NULL),
};
static int gpio_switch_en_vdd_com_voltages[] = { 3300};

/* EN_VDD_SDMMC1 from AP GPIO VI_HSYNC D07*/
static struct regulator_consumer_supply gpio_switch_en_vdd_sdmmc1_supply[] = {
	REGULATOR_SUPPLY("vddio_sd_slot", NULL),
};
static int gpio_switch_en_vdd_sdmmc1_voltages[] = { 3300};

/* EN_3V3_EMMC from AP GPIO SDMMC4_DAT4 D01*/
static struct regulator_consumer_supply gpio_switch_en_3v3_emmc_supply[] = {
	REGULATOR_SUPPLY("vdd_emmc_core", NULL),
};
static int gpio_switch_en_3v3_emmc_voltages[] = { 3300};

/* EN_3V3_PEX_HVDD from AP GPIO VI_D09 L07*/
static struct regulator_consumer_supply gpio_switch_en_3v3_pex_hvdd_supply[] = {
	REGULATOR_SUPPLY("hvdd_pex_3v3", NULL),
};
static int gpio_switch_en_3v3_pex_hvdd_voltages[] = { 3300};

/* EN_3v3_FUSE from AP GPIO VI_D08 L06*/
static struct regulator_consumer_supply gpio_switch_en_3v3_fuse_supply[] = {
	REGULATOR_SUPPLY("vpp_fuse_pg", NULL),
};
static int gpio_switch_en_3v3_fuse_voltages[] = { 3300};

/* EN_1V8_CAM from AP GPIO GPIO_PBB4 PBB04*/
static struct regulator_consumer_supply gpio_switch_en_1v8_cam_supply[] = {
	REGULATOR_SUPPLY("vdd_1v8_cam1", NULL),
	REGULATOR_SUPPLY("vdd_1v8_cam2", NULL),
	REGULATOR_SUPPLY("vdd_1v8_cam3", NULL),
};
static int gpio_switch_en_1v8_cam_voltages[] = { 1800};

/* Macro for defining gpio switch regulator platform data and device */
#define GPIO_REGULATOR_PINIT(_id, _name, _input_supply, _gpio_nr, _active_low) \
	static struct gpio_switch_regulator_platform_data		\
				gpio_switch_regulator_##_name##_pdata = { \
		.regulator_name	= "gpio-switch-"#_name,			\
		.input_supply	= _input_supply,			\
		.id		= _id,					\
		.gpio_nr	= _gpio_nr,				\
		.active_low	= _active_low,				\
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
	};								\
									\
	static struct platform_device gpio_switch_regulator_##_name = { \
		.name = "gpio-switch-regulator",			\
		.id   = _id,						\
		.dev  = {						\
		     .platform_data = &gpio_switch_regulator_##_name##_pdata,\
		},							\
	};

/* Gpio switch regulator platform data */
GPIO_REGULATOR_PINIT(0, en_5v_cp,   NULL, TPS6591X_GPIO_GP0, false)
GPIO_REGULATOR_PINIT(1, en_5v0,     NULL, TPS6591X_GPIO_GP2, false)
GPIO_REGULATOR_PINIT(2, en_ddr,     NULL, TPS6591X_GPIO_GP6, false)
GPIO_REGULATOR_PINIT(3, en_3v3_sys, NULL, TPS6591X_GPIO_GP7, false)

GPIO_REGULATOR_PINIT(4, en_vdd_bl,       NULL,          TEGRA_GPIO_PK3, false)
GPIO_REGULATOR_PINIT(5, en_3v3_modem,    NULL,          TEGRA_GPIO_PD6, false)
GPIO_REGULATOR_PINIT(6, en_usb1_vbus_oc, "vdd_5v0_sys", TEGRA_GPIO_PI4, false)
GPIO_REGULATOR_PINIT(7, en_usb3_vbus_oc, "vdd_5v0_sys", TEGRA_GPIO_PH7, false)
GPIO_REGULATOR_PINIT(8, en_vddio_vid_oc, "vdd_5v0_sys", TEGRA_GPIO_PT0, false)

GPIO_REGULATOR_PINIT(9, en_vdd_pnl1, "vdd_3v3_devices", TEGRA_GPIO_PL4, false)
GPIO_REGULATOR_PINIT(10, cam3_ldo_en, "vdd_3v3_devices", TEGRA_GPIO_PS0, false)
GPIO_REGULATOR_PINIT(11, en_vdd_com,  "vdd_3v3_devices", TEGRA_GPIO_PD0, false)
GPIO_REGULATOR_PINIT(12, en_3v3_fuse, "vdd_3v3_devices", TEGRA_GPIO_PL6, false)
GPIO_REGULATOR_PINIT(13, en_3v3_emmc, "vdd_3v3_devices", TEGRA_GPIO_PD1, false)
GPIO_REGULATOR_PINIT(14, en_vdd_sdmmc1, "vdd_3v3_devices", TEGRA_GPIO_PD7, false)
GPIO_REGULATOR_PINIT(15, en_3v3_pex_hvdd, "vdd_3v3_devices",
							TEGRA_GPIO_PL7, false)

GPIO_REGULATOR_PINIT(16, en_1v8_cam,  "vdd_gen1v8", TEGRA_GPIO_PBB4, false)

static struct platform_device *gpio_switch_regulator_devices[] __initdata = {
	&gpio_switch_regulator_en_5v_cp,
	&gpio_switch_regulator_en_5v0,
	&gpio_switch_regulator_en_ddr,
	&gpio_switch_regulator_en_3v3_sys,
	&gpio_switch_regulator_en_vdd_bl,
	&gpio_switch_regulator_en_3v3_modem,
	&gpio_switch_regulator_en_usb1_vbus_oc,
	&gpio_switch_regulator_en_usb3_vbus_oc,
	&gpio_switch_regulator_en_vddio_vid_oc,
	&gpio_switch_regulator_en_vdd_pnl1,
	&gpio_switch_regulator_cam3_ldo_en,
	&gpio_switch_regulator_en_vdd_com,
	&gpio_switch_regulator_en_3v3_fuse,
	&gpio_switch_regulator_en_3v3_emmc,
	&gpio_switch_regulator_en_vdd_sdmmc1,
	&gpio_switch_regulator_en_3v3_pex_hvdd,
	&gpio_switch_regulator_en_1v8_cam,
};

int __init cardhu_gpio_switch_regulator_init(void)
{
	int i;
	struct gpio_switch_regulator_platform_data *pdata;
	for (i = 0; i < ARRAY_SIZE(gpio_switch_regulator_devices); ++i) {
		pdata = gpio_switch_regulator_devices[i]->dev.platform_data;
		if (pdata->gpio_nr <= TEGRA_NR_GPIOS)
			tegra_gpio_enable(pdata->gpio_nr);
	}

	platform_add_devices(gpio_switch_regulator_devices,
				ARRAY_SIZE(gpio_switch_regulator_devices));
	return 0;
}
#else
int __init cardhu_gpio_switch_regulator_init(void)
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
