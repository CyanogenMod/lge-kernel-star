/*
 * arch/arm/mach-tegra/board-cardhu-pm298-power-rails.c
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
#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/regulator/gpio-switch-regulator.h>
#include <linux/mfd/max77663-core.h>
#include <linux/regulator/max77663-regulator.h>

#include <mach/iomap.h>
#include <mach/irqs.h>
#include <mach/pinmux.h>
#include <mach/edp.h>

#include "gpio-names.h"
#include "board.h"
#include "board-cardhu.h"
#include "pm.h"
#include "wakeups-t3.h"
#include "mach/tsensor.h"

#define PMC_CTRL		0x0
#define PMC_CTRL_INTR_LOW	BIT(17)

static struct regulator_consumer_supply max77663_sd0_supply[] = {
	REGULATOR_SUPPLY("vdd_cpu_pmu", NULL),
	REGULATOR_SUPPLY("vdd_cpu", NULL),
	REGULATOR_SUPPLY("vdd_sys", NULL),
};

static struct regulator_consumer_supply max77663_sd1_supply[] = {
	REGULATOR_SUPPLY("vdd_core", NULL),
	REGULATOR_SUPPLY("en_vddio_ddr_1v2", NULL),
};

static struct regulator_consumer_supply max77663_sd2_supply[] = {
	REGULATOR_SUPPLY("avdd_hdmi_pll", NULL),
	REGULATOR_SUPPLY("avdd_usb_pll", NULL),
	REGULATOR_SUPPLY("avdd_osc", NULL),
	REGULATOR_SUPPLY("vdd1v8_satelite", NULL),
	REGULATOR_SUPPLY("vddio_uart", NULL),
	REGULATOR_SUPPLY("pwrdet_uart", NULL),
	REGULATOR_SUPPLY("vddio_audio", NULL),
	REGULATOR_SUPPLY("pwrdet_audio", NULL),
	REGULATOR_SUPPLY("vddio_bb", NULL),
	REGULATOR_SUPPLY("pwrdet_bb", NULL),
	REGULATOR_SUPPLY("vddio_lcd_pmu", NULL),
	REGULATOR_SUPPLY("pwrdet_lcd", NULL),
	REGULATOR_SUPPLY("vddio_cam", NULL),
	REGULATOR_SUPPLY("pwrdet_cam", NULL),
	REGULATOR_SUPPLY("vddio_vi", NULL),
	REGULATOR_SUPPLY("pwrdet_vi", NULL),
	REGULATOR_SUPPLY("ldo6", NULL),
	REGULATOR_SUPPLY("ldo7", NULL),
	REGULATOR_SUPPLY("ldo8", NULL),
	REGULATOR_SUPPLY("vcore_audio", NULL),
	REGULATOR_SUPPLY("avcore_audio", NULL),
	REGULATOR_SUPPLY("vddio_sdmmc", "sdhci-tegra.2"),
	REGULATOR_SUPPLY("pwrdet_sdmmc3", NULL),
	REGULATOR_SUPPLY("vcore1_lpddr2", NULL),
	REGULATOR_SUPPLY("vcom_1v8", NULL),
	REGULATOR_SUPPLY("pmuio_1v8", NULL),
	REGULATOR_SUPPLY("avdd_ic_usb", NULL),
	REGULATOR_SUPPLY("vdd_gen1v8", NULL),
};

static struct regulator_consumer_supply max77663_sd3_supply[] = {
	REGULATOR_SUPPLY("vdd_gen1v5", NULL),
	REGULATOR_SUPPLY("vcore_lcd", NULL),
	REGULATOR_SUPPLY("track_ldo1", NULL),
	REGULATOR_SUPPLY("external_ldo_1v2", NULL),
	REGULATOR_SUPPLY("vcore_cam1", NULL),
	REGULATOR_SUPPLY("vcore_cam2", NULL),
	REGULATOR_SUPPLY("avdd_pexb", NULL),
	REGULATOR_SUPPLY("vdd_pexb", NULL),
	REGULATOR_SUPPLY("avdd_pex_pll", NULL),
	REGULATOR_SUPPLY("avdd_pexa", NULL),
	REGULATOR_SUPPLY("vdd_pexa", NULL),
	REGULATOR_SUPPLY("vcom_1v2", NULL),
	REGULATOR_SUPPLY("vdio_hsic", NULL),
};

static struct regulator_consumer_supply max77663_ldo0_supply[] = {
	REGULATOR_SUPPLY("vdd_ddr_hs", NULL),
};

static struct regulator_consumer_supply max77663_ldo1_supply[] = {
	REGULATOR_SUPPLY("avdd_plla_p_c_s", NULL),
	REGULATOR_SUPPLY("avdd_pllm", NULL),
	REGULATOR_SUPPLY("avdd_pllu_d", NULL),
	REGULATOR_SUPPLY("avdd_pllu_d2", NULL),
	REGULATOR_SUPPLY("avdd_pllx", NULL),
};

static struct regulator_consumer_supply max77663_ldo2_supply[] = {
	REGULATOR_SUPPLY("avdd_dsi_csi", NULL),
	REGULATOR_SUPPLY("pwrdet_mipi", NULL),
};

static struct regulator_consumer_supply max77663_ldo3_supply[] = {
	REGULATOR_SUPPLY("vddio_sdmmc", "sdhci-tegra.3"),
	REGULATOR_SUPPLY("pwrdet_sdmmc4", NULL),
};

static struct regulator_consumer_supply max77663_ldo4_supply[] = {
	REGULATOR_SUPPLY("vdd_rtc", NULL),
};

static struct regulator_consumer_supply max77663_ldo5_supply[] = {
	REGULATOR_SUPPLY("vddio_sdmmc", "sdhci-tegra.0"),
	REGULATOR_SUPPLY("pwrdet_sdmmc1", NULL),
};

static struct regulator_consumer_supply max77663_ldo6_supply[] = {
	REGULATOR_SUPPLY("vddio_sys", NULL),
};

static struct regulator_consumer_supply max77663_ldo7_supply[] = {
	REGULATOR_SUPPLY("unused_ldo7", NULL),
};

static struct regulator_consumer_supply max77663_ldo8_supply[] = {
	REGULATOR_SUPPLY("vcore_mmc", NULL),
};

static struct max77663_regulator_fps_cfg max77663_fps_cfgs[] = {
	{
		.src = FPS_SRC_0,
		.en_src = FPS_EN_SRC_EN0,
		.time_period = FPS_TIME_PERIOD_DEF,
	},
	{
		.src = FPS_SRC_1,
		.en_src = FPS_EN_SRC_EN1,
		.time_period = FPS_TIME_PERIOD_DEF,
	},
	{
		.src = FPS_SRC_2,
		.en_src = FPS_EN_SRC_EN0,
		.time_period = FPS_TIME_PERIOD_DEF,
	},
};

#define MAX77663_PDATA_INIT(_id, _min_uV, _max_uV, _supply_reg,		\
			    _always_on, _boot_on, _apply_uV,		\
			    _init_apply, _init_enable, _init_uV,	\
			    _fps_src, _fps_pu_period, _fps_pd_period, _flags) \
	static struct max77663_regulator_platform_data max77663_regulator_pdata_##_id = \
	{								\
		.init_data = {						\
			.constraints = {				\
				.min_uV = _min_uV,			\
				.max_uV = _max_uV,			\
				.valid_modes_mask = (REGULATOR_MODE_NORMAL |  \
						     REGULATOR_MODE_STANDBY), \
				.valid_ops_mask = (REGULATOR_CHANGE_MODE |    \
						   REGULATOR_CHANGE_STATUS |  \
						   REGULATOR_CHANGE_VOLTAGE), \
				.always_on = _always_on,		\
				.boot_on = _boot_on,			\
				.apply_uV = _apply_uV,			\
			},						\
			.num_consumer_supplies =			\
				ARRAY_SIZE(max77663_##_id##_supply),	\
			.consumer_supplies = max77663_##_id##_supply,	\
			.supply_regulator = _supply_reg,		\
		},							\
		.init_apply = _init_apply,				\
		.init_enable = _init_enable,				\
		.init_uV = _init_uV,					\
		.fps_src = _fps_src,					\
		.fps_pu_period = _fps_pu_period,			\
		.fps_pd_period = _fps_pd_period,			\
		.fps_cfgs = max77663_fps_cfgs,				\
		.flags = _flags,					\
	}

MAX77663_PDATA_INIT(sd0,  600000, 3387500, NULL, 1, 0, 0,
		    0, 0, -1, FPS_SRC_NONE, -1, -1, EN2_CTRL_SD0 | SD_FSRADE_DISABLE);

MAX77663_PDATA_INIT(sd1,  800000, 1587500, NULL, 1, 0, 0,
		    1, 1, -1, FPS_SRC_1, -1, -1, SD_FSRADE_DISABLE);

MAX77663_PDATA_INIT(sd2,  600000, 3387500, NULL, 1, 0, 0,
		    1, 1, -1, FPS_SRC_NONE, -1, -1, 0);

MAX77663_PDATA_INIT(sd3,  600000, 3387500, NULL, 0, 0, 0,
		    1, 1, -1, FPS_SRC_NONE, -1, -1, 0);

MAX77663_PDATA_INIT(ldo0, 800000, 2350000, max77663_rails(sd2), 0, 0, 0,
		    1, 1, -1, FPS_SRC_NONE, -1, -1, 0);

MAX77663_PDATA_INIT(ldo1, 800000, 2350000, max77663_rails(sd2), 0, 0, 0,
		    1, 1, -1, FPS_SRC_NONE, -1, -1, 0);

MAX77663_PDATA_INIT(ldo2, 800000, 3950000, max77663_rails(sd2), 0, 0, 0,
		    0, 0, -1, FPS_SRC_NONE, -1, -1, 0);

MAX77663_PDATA_INIT(ldo3, 800000, 3950000, NULL, 0, 0, 0,
		    1, 1, -1, FPS_SRC_NONE, -1, -1, 0);

MAX77663_PDATA_INIT(ldo4, 800000, 1587500, NULL, 0, 0, 0,
		    1, 1, -1, FPS_SRC_NONE, -1, -1, 0);

MAX77663_PDATA_INIT(ldo5, 800000, 3950000, NULL, 0, 0, 0,
		    0, 0, -1, FPS_SRC_NONE, -1, -1, 0);

MAX77663_PDATA_INIT(ldo6, 800000, 3950000, NULL, 1, 0, 0,
		    1, 1, -1, FPS_SRC_NONE, -1, -1, 0);

MAX77663_PDATA_INIT(ldo7, 800000, 3950000, NULL, 0, 0, 0,
		    0, 0, -1, FPS_SRC_NONE, -1, -1, 0);

MAX77663_PDATA_INIT(ldo8, 800000, 3950000, NULL, 0, 0, 0,
		    1, 1, -1, FPS_SRC_NONE, -1, -1, 0);

#define MAX77663_REG(_id, _data)					\
	{								\
		.name = "max77663-regulator",				\
		.id = MAX77663_REGULATOR_ID_##_id,			\
		.mfd_data = &max77663_regulator_pdata_##_data,		\
	}

#define MAX77663_RTC()							\
	{								\
		.name = "max77663-rtc",					\
		.id = 0,						\
	}

static struct mfd_cell max77663_subdevs[] = {
	MAX77663_REG(SD0, sd0),
	MAX77663_REG(SD1, sd1),
	MAX77663_REG(SD2, sd2),
	MAX77663_REG(SD3, sd3),
	MAX77663_REG(LDO0, ldo0),
	MAX77663_REG(LDO1, ldo1),
	MAX77663_REG(LDO2, ldo2),
	MAX77663_REG(LDO3, ldo3),
	MAX77663_REG(LDO4, ldo4),
	MAX77663_REG(LDO5, ldo5),
	MAX77663_REG(LDO6, ldo6),
	MAX77663_REG(LDO7, ldo7),
	MAX77663_REG(LDO8, ldo8),
	MAX77663_RTC(),
};

struct max77663_gpio_config max77663_gpio_cfgs[] = {
	{
		.gpio = MAX77663_GPIO0,
		.dir = GPIO_DIR_OUT,
		.dout = GPIO_DOUT_LOW,
		.out_drv = GPIO_OUT_DRV_PUSH_PULL,
		.alternate = GPIO_ALT_DISABLE,
	},
	{
		.gpio = MAX77663_GPIO1,
		.dir = GPIO_DIR_OUT,
		.dout = GPIO_DOUT_HIGH,
		.out_drv = GPIO_OUT_DRV_OPEN_DRAIN,
		.alternate = GPIO_ALT_DISABLE,
	},
	{
		.gpio = MAX77663_GPIO2,
		.dir = GPIO_DIR_OUT,
		.dout = GPIO_DOUT_HIGH,
		.out_drv = GPIO_OUT_DRV_OPEN_DRAIN,
		.alternate = GPIO_ALT_DISABLE,
	},
	{
		.gpio = MAX77663_GPIO3,
		.dir = GPIO_DIR_OUT,
		.dout = GPIO_DOUT_HIGH,
		.out_drv = GPIO_OUT_DRV_OPEN_DRAIN,
		.alternate = GPIO_ALT_DISABLE,
	},
	{
		.gpio = MAX77663_GPIO4,
		.out_drv = GPIO_OUT_DRV_PUSH_PULL,
		.alternate = GPIO_ALT_ENABLE,
	},
	{
		.gpio = MAX77663_GPIO5,
		.dir = GPIO_DIR_OUT,
		.dout = GPIO_DOUT_LOW,
		.out_drv = GPIO_OUT_DRV_PUSH_PULL,
		.alternate = GPIO_ALT_DISABLE,
	},
	{
		.gpio = MAX77663_GPIO6,
		.dir = GPIO_DIR_OUT,
		.dout = GPIO_DOUT_LOW,
		.out_drv = GPIO_OUT_DRV_PUSH_PULL,
		.alternate = GPIO_ALT_DISABLE,
	},
	{
		.gpio = MAX77663_GPIO7,
		.dir = GPIO_DIR_OUT,
		.dout = GPIO_DOUT_LOW,
		.out_drv = GPIO_OUT_DRV_PUSH_PULL,
		.alternate = GPIO_ALT_DISABLE,
	},
};

static struct max77663_platform_data max7763_pdata = {
	.irq_base	= MAX77663_IRQ_BASE,
	.gpio_base	= MAX77663_GPIO_BASE,

	.num_gpio_cfgs = ARRAY_SIZE(max77663_gpio_cfgs),
	.gpio_cfgs = max77663_gpio_cfgs,

	.num_subdevs	= ARRAY_SIZE(max77663_subdevs),
	.sub_devices	= max77663_subdevs,
};

static struct i2c_board_info __initdata max77663_regulators[] = {
	{
		/* The I2C address was determined by OTP factory setting */
		I2C_BOARD_INFO("max77663", 0x1C),
		.irq		= INT_EXTERNAL_PMU,
		.platform_data	= &max7763_pdata,
	},
};

int __init cardhu_pm298_regulator_init(void)
{
	struct board_info board_info;
	struct board_info pmu_board_info;
	void __iomem *pmc = IO_ADDRESS(TEGRA_PMC_BASE);
	u32 pmc_ctrl;

	/* configure the power management controller to trigger PMU
	 * interrupts when low */
	pmc_ctrl = readl(pmc + PMC_CTRL);
	writel(pmc_ctrl | PMC_CTRL_INTR_LOW, pmc + PMC_CTRL);

	/* The regulator details have complete constraints */
	tegra_get_board_info(&board_info);
	tegra_get_pmu_board_info(&pmu_board_info);
	if (pmu_board_info.board_id != BOARD_PMU_PM298) {
		pr_err("%s(): Board ID is not proper\n", __func__);
		return -ENODEV;
	}

	i2c_register_board_info(4, max77663_regulators,
				ARRAY_SIZE(max77663_regulators));

	return 0;
}

static struct regulator_consumer_supply gpio_switch_en_track_ldo2_supply[] = {
	REGULATOR_SUPPLY("avdd_sata", NULL),
	REGULATOR_SUPPLY("vdd_sata", NULL),
	REGULATOR_SUPPLY("avdd_sata_pll", NULL),
	REGULATOR_SUPPLY("avdd_plle", NULL),
};
static int gpio_switch_en_track_ldo2_voltages[] = { 3300};

static struct regulator_consumer_supply gpio_switch_en_5v0_supply[] = {
	REGULATOR_SUPPLY("vdd_5v0_sys", NULL),
	REGULATOR_SUPPLY("vdd_5v0_sby", NULL),
	REGULATOR_SUPPLY("vdd_hall", NULL),
	REGULATOR_SUPPLY("vterm_ddr", NULL),
	REGULATOR_SUPPLY("v2ref_ddr", NULL),
};
static int gpio_switch_en_5v0_voltages[] = { 5000};

static struct regulator_consumer_supply gpio_switch_en_ddr_supply[] = {
	REGULATOR_SUPPLY("mem_vddio_ddr", NULL),
	REGULATOR_SUPPLY("t30_vddio_ddr", NULL),
};
static int gpio_switch_en_ddr_voltages[] = { 1500};

static struct regulator_consumer_supply gpio_switch_en_3v3_sys_supply[] = {
	REGULATOR_SUPPLY("avdd_vdac", NULL),
	REGULATOR_SUPPLY("vdd_lvds", NULL),
	REGULATOR_SUPPLY("vdd_pnl", NULL),
	REGULATOR_SUPPLY("vcom_3v3", NULL),
	REGULATOR_SUPPLY("vdd_3v3", NULL),
	REGULATOR_SUPPLY("vddio_pex_ctl", NULL),
	REGULATOR_SUPPLY("pwrdet_pex_ctl", NULL),
	REGULATOR_SUPPLY("hvdd_pex_pmu", NULL),
	REGULATOR_SUPPLY("avdd_hdmi", NULL),
	REGULATOR_SUPPLY("vpp_fuse", NULL),
	REGULATOR_SUPPLY("avdd_usb", NULL),
	REGULATOR_SUPPLY("vdd_ddr_rx", NULL),
	REGULATOR_SUPPLY("vcore_nand", NULL),
	REGULATOR_SUPPLY("hvdd_sata", NULL),
	REGULATOR_SUPPLY("vddio_gmi_pmu", NULL),
	REGULATOR_SUPPLY("pwrdet_nand", NULL),
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
	REGULATOR_SUPPLY("vdd_3v3_edid", NULL),
	REGULATOR_SUPPLY("vdd_3v3_hdmi_cec", NULL),
	REGULATOR_SUPPLY("vdd_3v3_gmi", NULL),
	REGULATOR_SUPPLY("vdd_3v3_spk_amp", NULL),
	REGULATOR_SUPPLY("vdd_3v3_sensor", NULL),
	REGULATOR_SUPPLY("vdd_3v3_cam", NULL),
	REGULATOR_SUPPLY("vdd_3v3_als", NULL),
	REGULATOR_SUPPLY("debug_cons", NULL),
	REGULATOR_SUPPLY("vdd", "4-004c"),
};
static int gpio_switch_en_3v3_sys_voltages[] = { 3300};

/* DIS_5V_SWITCH from AP SPI2_SCK X02 */
static struct regulator_consumer_supply gpio_switch_dis_5v_switch_supply[] = {
	REGULATOR_SUPPLY("master_5v_switch", NULL),
};
static int gpio_switch_dis_5v_switch_voltages[] = { 5000};

/* EN_VDD_BL */
static struct regulator_consumer_supply gpio_switch_en_vdd_bl_supply[] = {
	REGULATOR_SUPPLY("vdd_backlight", NULL),
	REGULATOR_SUPPLY("vdd_backlight1", NULL),
};
static int gpio_switch_en_vdd_bl_voltages[] = { 5000};

/* EN_3V3_MODEM from AP GPIO VI_VSYNCH D06*/
static struct regulator_consumer_supply gpio_switch_en_3v3_modem_supply[] = {
	REGULATOR_SUPPLY("vdd_3v3_mini_card", NULL),
	REGULATOR_SUPPLY("vdd_mini_card", NULL),
};
static int gpio_switch_en_3v3_modem_voltages[] = { 3300};

/* EN_USB1_VBUS_OC*/
static struct regulator_consumer_supply gpio_switch_en_usb1_vbus_oc_supply[] = {
	REGULATOR_SUPPLY("vdd_vbus_micro_usb", NULL),
};
static int gpio_switch_en_usb1_vbus_oc_voltages[] = { 5000};

/*EN_USB3_VBUS_OC*/
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

/* CAM1_LDO_EN from AP GPIO KB_ROW6 R06*/
static struct regulator_consumer_supply gpio_switch_cam1_ldo_en_supply[] = {
	REGULATOR_SUPPLY("vdd_2v8_cam1", NULL),
	REGULATOR_SUPPLY("vdd_2v8_cam1_af", NULL),
};
static int gpio_switch_cam1_ldo_en_voltages[] = { 2800};

/* CAM2_LDO_EN from AP GPIO KB_ROW7 R07*/
static struct regulator_consumer_supply gpio_switch_cam2_ldo_en_supply[] = {
	REGULATOR_SUPPLY("vdd_2v8_cam2", NULL),
	REGULATOR_SUPPLY("vdd_2v8_cam2_af", NULL),
};
static int gpio_switch_cam2_ldo_en_voltages[] = { 2800};

/* CAM3_LDO_EN from AP GPIO KB_ROW8 S00*/
static struct regulator_consumer_supply gpio_switch_cam3_ldo_en_supply[] = {
	REGULATOR_SUPPLY("vdd_cam3", NULL),
};
static int gpio_switch_cam3_ldo_en_voltages[] = { 3300};

/* EN_VDD_COM from AP GPIO SDMMC3_DAT5 D00*/
static struct regulator_consumer_supply gpio_switch_en_vdd_com_supply[] = {
	REGULATOR_SUPPLY("vdd_com_bd", NULL),
};
static int gpio_switch_en_vdd_com_voltages[] = { 3300};

/* EN_VDD_SDMMC1 from AP GPIO VI_HSYNC D07*/
static struct regulator_consumer_supply gpio_switch_en_vdd_sdmmc1_supply[] = {
	REGULATOR_SUPPLY("vddio_sd_slot", "sdhci-tegra.0"),
};
static int gpio_switch_en_vdd_sdmmc1_voltages[] = { 3300};

/* EN_3V3_EMMC from AP GPIO SDMMC3_DAT4 D01*/
static struct regulator_consumer_supply gpio_switch_en_3v3_emmc_supply[] = {
	REGULATOR_SUPPLY("vdd_emmc_core", NULL),
};
static int gpio_switch_en_3v3_emmc_voltages[] = { 3300};

/* EN_3V3_PEX_HVDD from AP GPIO VI_D09 L07*/
static struct regulator_consumer_supply gpio_switch_en_3v3_pex_hvdd_supply[] = {
	REGULATOR_SUPPLY("hvdd_pex", NULL),
};
static int gpio_switch_en_3v3_pex_hvdd_voltages[] = { 3300};

/* EN_3v3_FUSE from AP GPIO VI_D08 L06*/
static struct regulator_consumer_supply gpio_switch_en_3v3_fuse_supply[] = {
	REGULATOR_SUPPLY("vdd_fuse", NULL),
};
static int gpio_switch_en_3v3_fuse_voltages[] = { 3300};

/* EN_1V8_CAM from AP GPIO GPIO_PBB4 PBB04*/
static struct regulator_consumer_supply gpio_switch_en_1v8_cam_supply[] = {
	REGULATOR_SUPPLY("vdd_1v8_cam1", NULL),
	REGULATOR_SUPPLY("vdd_1v8_cam2", NULL),
	REGULATOR_SUPPLY("vdd_1v8_cam3", NULL),
};
static int gpio_switch_en_1v8_cam_voltages[] = { 1800};

static struct regulator_consumer_supply gpio_switch_en_vbrtr_supply[] = {
	REGULATOR_SUPPLY("vdd_vbrtr", NULL),
};
static int gpio_switch_en_vbrtr_voltages[] = { 3300};

static int enable_load_switch_rail(
		struct gpio_switch_regulator_subdev_data *psubdev_data)
{
	int ret;

	if (psubdev_data->pin_group <= 0)
		return -EINVAL;

	/* Tristate and make pin as input*/
	ret = tegra_pinmux_set_tristate(psubdev_data->pin_group,
						TEGRA_TRI_TRISTATE);
	if (ret < 0)
		return ret;
	return gpio_direction_input(psubdev_data->gpio_nr);
}

static int disable_load_switch_rail(
		struct gpio_switch_regulator_subdev_data *psubdev_data)
{
	int ret;

	if (psubdev_data->pin_group <= 0)
		return -EINVAL;

	/* Un-tristate and driver low */
	ret = tegra_pinmux_set_tristate(psubdev_data->pin_group,
						TEGRA_TRI_NORMAL);
	if (ret < 0)
		return ret;
	return gpio_direction_output(psubdev_data->gpio_nr, 0);
}


/* Macro for defining gpio switch regulator sub device data */
#define GREG_INIT(_id, _var, _name, _input_supply, _always_on, _boot_on, \
	_gpio_nr, _active_low, _init_state, _pg, _enable, _disable)	 \
	static struct gpio_switch_regulator_subdev_data gpio_pdata_##_var =  \
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
			.always_on = _always_on,			\
			.boot_on = _boot_on,				\
		},							\
		.enable_rail = _enable,					\
		.disable_rail = _disable,				\
	}

/* common to most of boards*/
GREG_INIT(0, en_track_ldo2,	en_track_ldo2,	NULL,			0,	0,	MAX77663_GPIO_BASE + MAX77663_GPIO0,	false,	0,	0,	0,	0);
GREG_INIT(1, en_5v0,		en_5v0,		NULL,			1,      0,      MAX77663_GPIO_BASE + MAX77663_GPIO2,	false,	1,	0,	0,	0);
GREG_INIT(2, en_ddr,		en_ddr,		NULL,			1,      0,      MAX77663_GPIO_BASE + MAX77663_GPIO3,	false,	1,	0,	0,	0);
GREG_INIT(3, en_3v3_sys,	en_3v3_sys,	NULL,			1,      0,      MAX77663_GPIO_BASE + MAX77663_GPIO1,	false,	1,	0,	0,	0);
GREG_INIT(4, en_vdd_bl,		en_vdd_bl,	NULL,			0,      0,      TEGRA_GPIO_PK3,		false,	1,	0,	0,	0);
GREG_INIT(5, en_3v3_modem,	en_3v3_modem,	NULL,			1,      0,      TEGRA_GPIO_PD6,		false,	1,	0,	0,	0);
GREG_INIT(6, en_vdd_pnl1,	en_vdd_pnl1,	"vdd_3v3_devices",	0,      0,      TEGRA_GPIO_PL4,		false,	1,	0,	0,	0);
GREG_INIT(7, cam3_ldo_en,	cam3_ldo_en,	"vdd_3v3_devices",	0,      0,      TEGRA_GPIO_PS0,		false,	0,	0,	0,	0);
GREG_INIT(8, en_vdd_com,	en_vdd_com,	"vdd_3v3_devices",	1,      0,      TEGRA_GPIO_PD0,		false,	1,	0,	0,	0);
GREG_INIT(9, en_3v3_fuse,	en_3v3_fuse,	"vdd_3v3_devices",	0,      0,      TEGRA_GPIO_PL6,		false,	0,	0,	0,	0);
GREG_INIT(10, en_3v3_emmc,	en_3v3_emmc,	"vdd_3v3_devices",	1,      0,      TEGRA_GPIO_PD1,		false,	1,	0,	0,	0);
GREG_INIT(11, en_vdd_sdmmc1,	en_vdd_sdmmc1,	"vdd_3v3_devices",	0,      0,      TEGRA_GPIO_PD7,		false,	1,	0,	0,	0);
GREG_INIT(12, en_3v3_pex_hvdd,	en_3v3_pex_hvdd, "hvdd_pex_pmu",	0,      0,      TEGRA_GPIO_PL7,		false,	0,	0,	0,	0);
GREG_INIT(13, en_1v8_cam,	en_1v8_cam,	"vdd_gen1v8",		0,      0,      TEGRA_GPIO_PBB4,	false,	0,	0,	0,	0);

/*Specific to pm269*/
GREG_INIT(4, en_vdd_bl_pm269,		en_vdd_bl,		NULL,
	0,      0,      TEGRA_GPIO_PH3,	false,	1,	0,	0,	0);
GREG_INIT(6, en_vdd_pnl1_pm269,		en_vdd_pnl1,		"vdd_3v3_devices",
	0,      0,      TEGRA_GPIO_PW1,	false,	1,	0,	0,	0);
GREG_INIT(9, en_3v3_fuse_pm269,		en_3v3_fuse,		"vdd_3v3_devices",
	0,      0,      TEGRA_GPIO_PC1,	false,	0,	0,	0,	0);
GREG_INIT(12, en_3v3_pex_hvdd_pm269,	en_3v3_pex_hvdd,	"hvdd_pex_pmu",
	0,      0,      TEGRA_GPIO_PC6,	false,	0,	0,	0,	0);
GREG_INIT(17, en_vddio_vid_oc_pm269,	en_vddio_vid_oc,	"master_5v_switch",
	0,      0,      TEGRA_GPIO_PP2,	false,	0,	TEGRA_PINGROUP_DAP3_DOUT,
	enable_load_switch_rail, disable_load_switch_rail);

/* Specific to E1187/E1186/E1256 */
GREG_INIT(14, dis_5v_switch_e118x,	dis_5v_switch,		"vdd_5v0_sys",
		0,      0,      TEGRA_GPIO_PX2,		true,	0,	0,	0,	0);
GREG_INIT(15, en_usb1_vbus_oc_e118x,	en_usb1_vbus_oc,	"master_5v_switch",
		0,      0,      TEGRA_GPIO_PI4,		false,	0,	TEGRA_PINGROUP_GMI_RST_N,
		enable_load_switch_rail, disable_load_switch_rail);
GREG_INIT(16, en_usb3_vbus_oc_e118x,	en_usb3_vbus_oc,	"master_5v_switch",
		0,      0,      TEGRA_GPIO_PH7,		false,	0,	TEGRA_PINGROUP_GMI_AD15,
		enable_load_switch_rail, disable_load_switch_rail);
GREG_INIT(17, en_vddio_vid_oc_e118x,	en_vddio_vid_oc,	"master_5v_switch",
		0,      0,      TEGRA_GPIO_PT0,		false,	0,	TEGRA_PINGROUP_VI_PCLK,
		enable_load_switch_rail, disable_load_switch_rail);

/* E1198/E1291 specific*/
GREG_INIT(18, cam1_ldo_en,	cam1_ldo_en,	"vdd_3v3_cam",	0,      0,      TEGRA_GPIO_PR6,		false,	0,	0,	0,	0);
GREG_INIT(19, cam2_ldo_en,	cam2_ldo_en,	"vdd_3v3_cam",	0,      0,      TEGRA_GPIO_PR7,		false,	0,	0,	0,	0);

GREG_INIT(22, en_vbrtr,		en_vbrtr,	"vdd_3v3_devices",	0,      0,      PMU_TCA6416_GPIO_PORT12,	false,	0,	0,	0,	0);

#define ADD_GPIO_REG(_name) &gpio_pdata_##_name

#define COMMON_GPIO_REG \
	ADD_GPIO_REG(en_track_ldo2),		\
	ADD_GPIO_REG(en_5v0),			\
	ADD_GPIO_REG(en_ddr),			\
	ADD_GPIO_REG(en_3v3_sys),		\
	ADD_GPIO_REG(en_3v3_modem),		\
	ADD_GPIO_REG(en_vdd_pnl1),		\
	ADD_GPIO_REG(cam1_ldo_en),		\
	ADD_GPIO_REG(cam2_ldo_en),		\
	ADD_GPIO_REG(cam3_ldo_en),		\
	ADD_GPIO_REG(en_vdd_com),		\
	ADD_GPIO_REG(en_3v3_fuse),		\
	ADD_GPIO_REG(en_3v3_emmc),		\
	ADD_GPIO_REG(en_vdd_sdmmc1),		\
	ADD_GPIO_REG(en_3v3_pex_hvdd),		\
	ADD_GPIO_REG(en_1v8_cam),

#define PM269_GPIO_REG \
	ADD_GPIO_REG(en_track_ldo2),		\
	ADD_GPIO_REG(en_5v0),			\
	ADD_GPIO_REG(en_ddr),			\
	ADD_GPIO_REG(en_vdd_bl_pm269),		\
	ADD_GPIO_REG(en_3v3_sys),		\
	ADD_GPIO_REG(en_3v3_modem),		\
	ADD_GPIO_REG(en_vdd_pnl1_pm269),	\
	ADD_GPIO_REG(cam1_ldo_en),		\
	ADD_GPIO_REG(cam2_ldo_en),		\
	ADD_GPIO_REG(cam3_ldo_en),		\
	ADD_GPIO_REG(en_vdd_com),		\
	ADD_GPIO_REG(en_3v3_fuse_pm269),	\
	ADD_GPIO_REG(en_3v3_emmc),		\
	ADD_GPIO_REG(en_3v3_pex_hvdd_pm269),	\
	ADD_GPIO_REG(en_1v8_cam),		\
	ADD_GPIO_REG(dis_5v_switch_e118x),	\
	ADD_GPIO_REG(en_usb1_vbus_oc_e118x),	\
	ADD_GPIO_REG(en_usb3_vbus_oc_e118x),	\
	ADD_GPIO_REG(en_vddio_vid_oc_pm269),

#define E118x_GPIO_REG	\
	ADD_GPIO_REG(en_vdd_bl),		\
	ADD_GPIO_REG(dis_5v_switch_e118x),	\
	ADD_GPIO_REG(en_usb1_vbus_oc_e118x),	\
	ADD_GPIO_REG(en_usb3_vbus_oc_e118x),	\
	ADD_GPIO_REG(en_vddio_vid_oc_e118x),	\
	ADD_GPIO_REG(en_vbrtr),

/* Gpio switch regulator platform data  for E1186/E1187/E1256*/
static struct gpio_switch_regulator_subdev_data *gswitch_subdevs_e118x[] = {
	COMMON_GPIO_REG
	E118x_GPIO_REG
};

/* Gpio switch regulator platform data for PM269*/
static struct gpio_switch_regulator_subdev_data *gswitch_subdevs_pm269[] = {
	PM269_GPIO_REG
};

static struct gpio_switch_regulator_platform_data  gswitch_pdata;
static struct platform_device gswitch_regulator_pdata = {
	.name = "gpio-switch-regulator",
	.id   = -1,
	.dev  = {
	     .platform_data = &gswitch_pdata,
	},
};

int __init cardhu_pm298_gpio_switch_regulator_init(void)
{
	int i;
	struct board_info board_info;
	tegra_get_board_info(&board_info);

	switch (board_info.board_id) {
	case BOARD_PM269:
	case BOARD_PM305:
	case BOARD_PM311:
	case BOARD_E1257:
		gswitch_pdata.num_subdevs = ARRAY_SIZE(gswitch_subdevs_pm269);
		gswitch_pdata.subdevs = gswitch_subdevs_pm269;
		break;

	default:
		gswitch_pdata.num_subdevs = ARRAY_SIZE(gswitch_subdevs_e118x);
		gswitch_pdata.subdevs = gswitch_subdevs_e118x;
		break;
	}

	for (i = 0; i < gswitch_pdata.num_subdevs; ++i) {
		struct gpio_switch_regulator_subdev_data *gswitch_data =
						gswitch_pdata.subdevs[i];
		if (gswitch_data->gpio_nr <= TEGRA_NR_GPIOS)
			tegra_gpio_enable(gswitch_data->gpio_nr);
	}

	return platform_device_register(&gswitch_regulator_pdata);
}
