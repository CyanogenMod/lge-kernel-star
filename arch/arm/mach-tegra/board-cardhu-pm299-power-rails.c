/*
 * arch/arm/mach-tegra/board-cardhu-pm299-power-rails.c
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
#include <linux/mfd/ricoh583.h>
#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/regulator/gpio-switch-regulator.h>
#include <linux/regulator/ricoh583-regulator.h>
#include <linux/regulator/tps6236x-regulator.h>

#include <mach/iomap.h>
#include <mach/irqs.h>
#include <mach/pinmux.h>
#include <mach/edp.h>

#include "gpio-names.h"
#include "board.h"
#include "board-cardhu.h"
#include "wakeups-t3.h"
#include "mach/tsensor.h"

#define PMC_CTRL		0x0
#define PMC_CTRL_INTR_LOW	(1 << 17)

static struct regulator_consumer_supply ricoh583_dc1_supply_skubit0_0[] = {
	REGULATOR_SUPPLY("vdd_core", NULL),
	REGULATOR_SUPPLY("en_vddio_ddr_1v2", NULL),
};

static struct regulator_consumer_supply ricoh583_dc1_supply_skubit0_1[] = {
	REGULATOR_SUPPLY("en_vddio_ddr_1v2", NULL),
};

static struct regulator_consumer_supply ricoh583_dc3_supply_0[] = {
	REGULATOR_SUPPLY("vdd_gen1v5", NULL),
	REGULATOR_SUPPLY("vcore_lcd", NULL),
	REGULATOR_SUPPLY("track_ldo1", NULL),
	REGULATOR_SUPPLY("external_ldo_1v2", NULL),
	REGULATOR_SUPPLY("vcore_cam1", NULL),
	REGULATOR_SUPPLY("vcore_cam2", NULL),
};

static struct regulator_consumer_supply ricoh583_dc0_supply_0[] = {
	REGULATOR_SUPPLY("vdd_cpu_pmu", NULL),
	REGULATOR_SUPPLY("vdd_cpu", NULL),
	REGULATOR_SUPPLY("vdd_sys", NULL),
};

static struct regulator_consumer_supply ricoh583_dc2_supply_0[] = {
	REGULATOR_SUPPLY("vdd_gen1v8", NULL),
	REGULATOR_SUPPLY("avdd_hdmi_pll", NULL),
	REGULATOR_SUPPLY("avdd_usb_pll", NULL),
	REGULATOR_SUPPLY("avdd_osc", NULL),
	REGULATOR_SUPPLY("vddio_sys", NULL),
	REGULATOR_SUPPLY("vddio_sdmmc", "sdhci-tegra.3"),
	REGULATOR_SUPPLY("pwrdet_sdmmc4", NULL),
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
};

static struct regulator_consumer_supply ricoh583_ldo0_supply_0[] = {
	REGULATOR_SUPPLY("unused_ldo0", NULL),
};

static struct regulator_consumer_supply ricoh583_ldo1_supply_0[] = {
	REGULATOR_SUPPLY("avdd_pexb", NULL),
	REGULATOR_SUPPLY("vdd_pexb", NULL),
	REGULATOR_SUPPLY("avdd_pex_pll", NULL),
	REGULATOR_SUPPLY("avdd_pexa", NULL),
	REGULATOR_SUPPLY("vdd_pexa", NULL),
};

static struct regulator_consumer_supply ricoh583_ldo2_supply_0[] = {
	REGULATOR_SUPPLY("avdd_sata", NULL),
	REGULATOR_SUPPLY("vdd_sata", NULL),
	REGULATOR_SUPPLY("avdd_sata_pll", NULL),
	REGULATOR_SUPPLY("avdd_plle", NULL),
};

static struct regulator_consumer_supply ricoh583_ldo3_supply_0[] = {
	REGULATOR_SUPPLY("vddio_sdmmc", "sdhci-tegra.0"),
	REGULATOR_SUPPLY("pwrdet_sdmmc1", NULL),
};

static struct regulator_consumer_supply ricoh583_ldo4_supply_0[] = {
	REGULATOR_SUPPLY("vdd_rtc", NULL),
};

static struct regulator_consumer_supply ricoh583_ldo5_supply_0[] = {
	REGULATOR_SUPPLY("avdd_vdac", NULL),
};

static struct regulator_consumer_supply ricoh583_ldo6_supply_0[] = {
	REGULATOR_SUPPLY("avdd_dsi_csi", NULL),
	REGULATOR_SUPPLY("pwrdet_mipi", NULL),
};
static struct regulator_consumer_supply ricoh583_ldo7_supply_0[] = {
	REGULATOR_SUPPLY("avdd_plla_p_c_s", NULL),
	REGULATOR_SUPPLY("avdd_pllm", NULL),
	REGULATOR_SUPPLY("avdd_pllu_d", NULL),
	REGULATOR_SUPPLY("avdd_pllu_d2", NULL),
	REGULATOR_SUPPLY("avdd_pllx", NULL),
};

static struct regulator_consumer_supply ricoh583_ldo8_supply_0[] = {
	REGULATOR_SUPPLY("vdd_ddr_hs", NULL),
};

#define RICOH_PDATA_INIT(_name, _sname, _minmv, _maxmv, _supply_reg, _always_on, \
	_boot_on, _apply_uv, _init_uV, _init_enable, _init_apply, _flags,      \
	_ext_contol, _ds_slots) \
	static struct ricoh583_regulator_platform_data pdata_##_name##_##_sname = \
	{								\
		.regulator = {						\
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
				ARRAY_SIZE(ricoh583_##_name##_supply_##_sname),	\
			.consumer_supplies = ricoh583_##_name##_supply_##_sname, \
			.supply_regulator = _supply_reg,		\
		},							\
		.init_uV =  _init_uV * 1000,				\
		.init_enable = _init_enable,				\
		.init_apply = _init_apply,				\
		.deepsleep_slots = _ds_slots,				\
		.flags = _flags,					\
		.ext_pwr_req = _ext_contol,				\
	}

RICOH_PDATA_INIT(dc0, 0,         700,  1500, 0, 1, 1, 0, -1, 0, 0, 0,
				RICOH583_EXT_PWRREQ2_CONTROL, 0);
RICOH_PDATA_INIT(dc1, skubit0_0, 700,  1500, 0, 1, 1, 0, -1, 0, 0, 0, 0, 0);
RICOH_PDATA_INIT(dc2, 0,         900,  2400, 0, 1, 1, 0, -1, 0, 0, 0, 0, 0);
RICOH_PDATA_INIT(dc3, 0,         900,  2400, 0, 1, 1, 0, -1, 0, 0, 0, 0, 0);

RICOH_PDATA_INIT(ldo0, 0,         1000, 3300, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0);
RICOH_PDATA_INIT(ldo1, 0,         1000, 3300, ricoh583_rails(DC1), 0, 0, 0, -1, 0, 0, 0, 0, 0);
RICOH_PDATA_INIT(ldo2, 0,         1050, 1050, ricoh583_rails(DC1), 0, 0, 1, -1, 0, 0, 0, 0, 0);

RICOH_PDATA_INIT(ldo3, 0,         1000, 3300, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0);
RICOH_PDATA_INIT(ldo4, 0,         750,  1500, 0, 1, 0, 0, -1, 0, 0, 0, 0, 0);
RICOH_PDATA_INIT(ldo5, 0,         1000, 3300, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0);

RICOH_PDATA_INIT(ldo6, 0,         1200, 1200, ricoh583_rails(DC2), 0, 0, 1, -1, 0, 0, 0, 0, 0);
RICOH_PDATA_INIT(ldo7, 0,         1200, 1200, ricoh583_rails(DC2), 1, 1, 1, -1, 0, 0, 0, 0, 0);
RICOH_PDATA_INIT(ldo8, 0,         900, 3400, ricoh583_rails(DC2), 1, 0, 0, -1, 0, 0, 0, 0, 0);

static struct ricoh583_rtc_platform_data rtc_data = {
	.irq = TEGRA_NR_IRQS + RICOH583_IRQ_YALE,
	.time = {
		.tm_year = 2011,
		.tm_mon = 0,
		.tm_mday = 1,
		.tm_hour = 0,
		.tm_min = 0,
		.tm_sec = 0,
	},
};

#define RICOH_RTC_REG()				\
{						\
	.id	= 0,				\
	.name	= "rtc_ricoh583",		\
	.platform_data = &rtc_data,		\
}

#define RICOH_REG(_id, _name, _sname)			\
{							\
	.id	= RICOH583_ID_##_id,			\
	.name	= "ricoh583-regulator",			\
	.platform_data	= &pdata_##_name##_##_sname,	\
}

#define RICOH583_DEV_COMMON_E118X 		\
	RICOH_REG(DC0, dc0, 0),			\
	RICOH_REG(DC1, dc1, skubit0_0),		\
	RICOH_REG(DC2, dc2, 0),		\
	RICOH_REG(DC3, dc3, 0),		\
	RICOH_REG(LDO0, ldo8, 0),		\
	RICOH_REG(LDO1, ldo7, 0),		\
	RICOH_REG(LDO2, ldo6, 0),		\
	RICOH_REG(LDO3, ldo5, 0),		\
	RICOH_REG(LDO4, ldo4, 0),		\
	RICOH_REG(LDO5, ldo3, 0),		\
	RICOH_REG(LDO6, ldo0, 0),		\
	RICOH_REG(LDO7, ldo1, 0),		\
	RICOH_REG(LDO8, ldo2, 0),		\
	RICOH_RTC_REG()

static struct ricoh583_subdev_info ricoh_devs_e118x_dcdc[] = {
	RICOH583_DEV_COMMON_E118X,
};

#define RICOH_GPIO_INIT(_init_apply, _pulldn, _output_mode, _output_val) \
	{					\
		.pulldn_en = _pulldn,		\
		.output_mode_en = _output_mode,	\
		.output_val = _output_val,	\
		.init_apply = _init_apply,	\
	}
struct ricoh583_gpio_init_data ricoh_gpio_data[] = {
	RICOH_GPIO_INIT(false, false, false, 0),
	RICOH_GPIO_INIT(false, false, false, 0),
	RICOH_GPIO_INIT(false, false, false, 0),
	RICOH_GPIO_INIT(true,  false,  true, 1),
	RICOH_GPIO_INIT(true,  false, true, 1),
	RICOH_GPIO_INIT(false, false, false, 0),
	RICOH_GPIO_INIT(false, false, false, 0),
	RICOH_GPIO_INIT(false, false, false, 0),
};

static struct ricoh583_platform_data ricoh_platform = {
	.irq_base	= RICOH583_IRQ_BASE,
	.gpio_base	= RICOH583_GPIO_BASE,
	.gpio_init_data = ricoh_gpio_data,
	.num_gpioinit_data = ARRAY_SIZE(ricoh_gpio_data),
	.enable_shutdown_pin = true,
};

static struct i2c_board_info __initdata ricoh583_regulators[] = {
	{
		I2C_BOARD_INFO("ricoh583", 0x34),
		.irq		= INT_EXTERNAL_PMU,
		.platform_data	= &ricoh_platform,
	},
};

/* TPS62361B DC-DC converter */
static struct regulator_consumer_supply tps6236x_dcdc_supply[] = {
	REGULATOR_SUPPLY("vdd_core", NULL),
};

static struct tps6236x_regulator_platform_data tps6236x_pdata = {
	.reg_init_data = {					\
		.constraints = {				\
			.min_uV = 500000,			\
			.max_uV = 1770000,			\
			.valid_modes_mask = (REGULATOR_MODE_NORMAL |  \
					     REGULATOR_MODE_STANDBY), \
			.valid_ops_mask = (REGULATOR_CHANGE_MODE |    \
					   REGULATOR_CHANGE_STATUS |  \
					   REGULATOR_CHANGE_VOLTAGE), \
			.always_on = 1,				\
			.boot_on =  1,				\
			.apply_uV = 0,				\
		},						\
		.num_consumer_supplies = ARRAY_SIZE(tps6236x_dcdc_supply), \
		.consumer_supplies = tps6236x_dcdc_supply,		\
		},							\
	.internal_pd_enable = 0,					\
	.vsel = 3,							\
	.init_uV = -1,							\
	.init_apply = 0,						\
};

static struct i2c_board_info __initdata tps6236x_boardinfo[] = {
	{
		I2C_BOARD_INFO("tps62361B", 0x60),
		.platform_data	= &tps6236x_pdata,
	},
};

int __init cardhu_pm299_regulator_init(void)
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
	if (pmu_board_info.board_id != BOARD_PMU_PM299) {
		pr_err("%s(): Board ID is not proper\n", __func__);
		return -ENODEV;
	}

	/* If TPS6236x DCDC is there then consumer for dc1 should
	 * not have vdd_core */
	if ((board_info.sku & SKU_DCDC_TPS62361_SUPPORT) ||
			(pmu_board_info.sku & SKU_DCDC_TPS62361_SUPPORT)) {
		pdata_dc1_skubit0_0.regulator.consumer_supplies =
					ricoh583_dc1_supply_skubit0_1;
		pdata_dc1_skubit0_0.regulator.num_consumer_supplies =
				ARRAY_SIZE(ricoh583_dc1_supply_skubit0_1);
	}

	ricoh_platform.num_subdevs = ARRAY_SIZE(ricoh_devs_e118x_dcdc);
	ricoh_platform.subdevs = ricoh_devs_e118x_dcdc;

	i2c_register_board_info(4, ricoh583_regulators, 1);

	/* Register the TPS6236x for all boards whose sku bit 0 is set. */
	if ((board_info.sku & SKU_DCDC_TPS62361_SUPPORT) ||
			(pmu_board_info.sku & SKU_DCDC_TPS62361_SUPPORT)) {
		pr_info("Registering the device TPS62361B\n");
		i2c_register_board_info(4, tps6236x_boardinfo, 1);
	}
	return 0;
}

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
GREG_INIT(0, en_5v_cp,		en_5v_cp,	NULL,			1,	0,	TPS6591X_GPIO_0,	false,	1,	0,	0,	0);
GREG_INIT(1, en_5v0,		en_5v0,		NULL,			0,      0,      TPS6591X_GPIO_4,	false,	0,	0,	0,	0);
GREG_INIT(2, en_ddr,		en_ddr,		NULL,			0,      0,      TPS6591X_GPIO_3,	false,	1,	0,	0,	0);
GREG_INIT(3, en_3v3_sys,	en_3v3_sys,	NULL,			0,      0,      TPS6591X_GPIO_1,	false,	0,	0,	0,	0);
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
GREG_INIT(11, en_vdd_sdmmc1_pm269,	en_vdd_sdmmc1,		"vdd_3v3_devices",
	0,      0,      TEGRA_GPIO_PP1,	false,	1,	0,	0,	0);
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
	ADD_GPIO_REG(en_5v_cp),			\
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
	ADD_GPIO_REG(en_5v_cp),			\
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
	ADD_GPIO_REG(en_vdd_sdmmc1_pm269),	\
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

int __init cardhu_pm299_gpio_switch_regulator_init(void)
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
