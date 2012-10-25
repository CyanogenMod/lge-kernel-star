/*
 * arch/arm/mach-tegra/board-bssq-panel.c
 *
 * Copyright (c) 2010-2011, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/resource.h>
#include <asm/mach-types.h>
#include <linux/platform_device.h>
#include <linux/earlysuspend.h>
#include <linux/kernel.h>
#include <linux/bd6084_bl.h>
#include <linux/nvhost.h>
#include <mach/nvmap.h>
#include <mach/irqs.h>
#include <mach/iomap.h>
#include <mach/dc.h>
#include <mach/fb.h>
#include <lge/lge_hw_rev.h>

#include <mach-tegra/devices.h>
#include <mach-tegra/gpio-names.h>
#include <mach-tegra/board.h>

#if defined(CONFIG_LU6500)
/****************************/
/*		choose 1 panel		*/
#define HITACH_PANEL	0
#define LGD_PANEL		1
/****************************/
#else
/****************************/
/*		choose 1 panel		*/
#define HITACH_PANEL	1
#define LGD_PANEL		0
/****************************/
#endif

#define bssq_hdmi_reg_en	TEGRA_GPIO_PK5
#define bssq_hdmi_hpd		TEGRA_GPIO_PN7

#ifdef CONFIG_TEGRA_DC
	static struct regulator *bssq_hdmi_reg = NULL;
	static struct regulator *bssq_hdmi_pll = NULL;
#endif

#define DC_CTRL_MODE	TEGRA_DC_OUT_ONE_SHOT_MODE
//#define DC_CTRL_MODE	TEGRA_DC_OUT_CONTINUOUS_MODE

#ifdef CONFIG_TEGRA_DSI
	static struct regulator *bssq_dsi_reg = NULL;

	// When kernel is load, it could be skip panel_enable and poweron, so this 4 flag is needed.
	static int is_dsi_panel_enable = 0;
	static int is_dsi_panel_disable = 0;
	static int is_dsi_panel_poweroff = 0;
	static int is_dsi_panel_poweron = 0;

	#if defined(CONFIG_LU6500)
	#define BSSQ_DSI_PANEL_RESET ((get_lge_pcb_revision() > REV_D) ? (TEGRA_GPIO_PE7) : (TEGRA_GPIO_PV7))
	#define BSSQ_DSI_PANEL_RESET_STR ((get_lge_pcb_revision() > REV_D) ? ("pe7") : ("pv7"))
	#endif

	#if defined(CONFIG_SU880) || defined(CONFIG_KU8800) || defined(CONFIG_LU8800) || defined(CONFIG_KS1103)
	#define BSSQ_DSI_PANEL_RESET TEGRA_GPIO_PE7
	#define BSSQ_DSI_PANEL_RESET_STR "pe7"
	#endif

	#if defined(CONFIG_KS1001)
	#define BSSQ_DSI_PANEL_RESET ((get_lge_pcb_revision() >= REV_C) ? (TEGRA_GPIO_PE7) : (TEGRA_GPIO_PV7))
	#define BSSQ_DSI_PANEL_RESET_STR ((get_lge_pcb_revision() >= REV_C) ? ("pe7") : ("pv7"))
	#endif

	#define bssq_dsi_panel_cs		TEGRA_GPIO_PN4
#endif

#define BACKLIGHT_IC_EN		TEGRA_GPIO_PE3

/* Backlight */
static int backlight_ic_init(struct backlight_device *bd)
{
	struct bd6084_bl_driver_data *drvdata = dev_get_drvdata(&bd->dev);
	int ret;
	
	ret = gpio_request(drvdata->en_pin, "backlight_ic-en");
	if (ret < 0)
		return ret;

	ret = gpio_direction_output(drvdata->en_pin, 1);
	if (ret < 0) {
		gpio_free(drvdata->en_pin);
		return ret;
	}
	else
		tegra_gpio_enable(drvdata->en_pin);

	return 0;
}

static void backlight_ic_uninit(struct backlight_device *bd)
{
	struct bd6084_bl_driver_data *drvdata = dev_get_drvdata(&bd->dev);

	if(drvdata->en_pin > 0)
		gpio_free(drvdata->en_pin);
	tegra_gpio_disable(drvdata->en_pin);
}

static int bssq_disp1_check_fb(struct device *dev, struct fb_info *info);

static struct bd6084_bl_platform_data bd6084_bl_pdata = {
	.en_pin		= BACKLIGHT_IC_EN,
	.avail_ch	= 0xFF, /* use all channels */
	.max_current	= 127,
	.max_brightness	= 255,
	.init		= backlight_ic_init,
	.uninit		= backlight_ic_uninit,
	/* Only toggle backlight on fb blank notifications for disp1 */
	.check_fb   = bssq_disp1_check_fb,
};

static struct i2c_board_info __initdata tegra_i2c_gpio_info[] = {
	 {
		I2C_BOARD_INFO(BD6084_BL_DRV_NAME, 0x76),
		.platform_data = &bd6084_bl_pdata,
	 },
};

#ifdef CONFIG_TEGRA_DSI
static int bssq_dsi_panel_enable(void)
{
	if(is_dsi_panel_enable)	// already panel enabled.
	{
		printk("already panel enabled.\n");
		return 0;
	}

	//vddio_mipi on/off control is here for early suspend/resume.
	printk("++++ bssq_dsi_panel_enable\n");

	if (!bssq_dsi_reg) {
		
		printk("bssq_dsi_reg == NULL\n");
		
		bssq_dsi_reg = regulator_get(NULL, "vddio_mipi");
		if (IS_ERR_OR_NULL(bssq_dsi_reg)) {
			pr_err("dsi: couldn't get regulator vddio_mipi\n");
			bssq_dsi_reg = NULL;
			return PTR_ERR(bssq_dsi_reg);
		}
		regulator_set_voltage(bssq_dsi_reg, 1200000, 1200000);
	}

	if(is_dsi_panel_disable != 1)
	{
		regulator_enable(bssq_dsi_reg);
		mdelay(1);
		regulator_disable(bssq_dsi_reg);
	}

	regulator_enable(bssq_dsi_reg);
	mdelay(1);

	is_dsi_panel_disable = 0;
	is_dsi_panel_enable = 1;

	printk("---- bssq_dsi_panel_enable\n");

	return 0;
}

static int bssq_dsi_panel_disable(void)
{

	// move to bssq_dsi_panel_postsuspend
	printk("bssq_dsi_panel_disable\n");

	return 0;
}
#endif

static int bssq_dsi_panel_poweron(void)
{
	if(is_dsi_panel_poweron) // already power on
	{
		printk("already power on\n");
		return 0;
	}

	printk("++++bssq_dsi_panel_poweron\n");

	gpio_set_value(BSSQ_DSI_PANEL_RESET, 0);

#if (HITACH_PANEL)

	if(is_dsi_panel_poweroff != 1)
	{
		gpio_set_value(bssq_dsi_panel_cs, 0);
		bd6084_bl_set_ldo(BD6084_LDO12, 0);
		mdelay(60);
	}

	gpio_set_value(bssq_dsi_panel_cs, 1);

	bd6084_bl_set_ldo(BD6084_LDO2, 1);
	bd6084_bl_set_ldo(BD6084_LDO1, 1);
	mdelay(25);

	gpio_set_value(BSSQ_DSI_PANEL_RESET, 1);
	mdelay(6);

#else //(LGD_PANEL)
	gpio_set_value(BSSQ_DSI_PANEL_RESET, 0); 

	if(is_dsi_panel_poweroff != 1) 
	{
		gpio_set_value(bssq_dsi_panel_cs, 0);
		bd6084_bl_set_ldo(BD6084_LDO12, 0);
		mdelay(60);
	}

	gpio_set_value(bssq_dsi_panel_cs, 1);

	bd6084_bl_set_ldo(BD6084_LDO2, 0);
	bd6084_bl_set_ldo(BD6084_LDO1, 1);
	mdelay(1);
		
	mdelay(10);
		
	bd6084_bl_set_ldo(BD6084_LDO2, 1);
	mdelay(1);

/*	20110831 deukgishin@lge.com // remove VCC,VCI toggle[S]
	mdelay(30);
		
	bd6084_bl_set_ldo(BD6084_LDO2, 0);
	mdelay(1);
		
	mdelay(30);
		
	bd6084_bl_set_ldo(BD6084_LDO2, 1);
	mdelay(1);
*/	//20110831 deukgishin@lge.com // remove VCC,VCI toggle[S]
		mdelay(20);

	gpio_set_value(BSSQ_DSI_PANEL_RESET, 1);

		mdelay(10);
#endif

	is_dsi_panel_poweroff = 0;
	is_dsi_panel_poweron = 1;

	printk("----bssq_dsi_panel_poweron\n");

	return 0;
}

// +++ kyouonghoon.lim@lge.com nvidia issue  #868446
static int bssq_dsi_panel_postpoweron(void)
{
	static bool called = false;

	printk("bssq_dsi_panel_postpoweron\n");

	if(!called)
	{
		called = true;
		printk("%s:\n", __func__);

		if (!bssq_dsi_reg) {
			bssq_dsi_reg = regulator_get(NULL, "vddio_mipi");
			if (IS_ERR_OR_NULL(bssq_dsi_reg)) {
				pr_err("dsi: couldn't get regulator vddio_mipi\n");
				bssq_dsi_reg = NULL;
				return PTR_ERR(bssq_dsi_reg);
			}
			regulator_set_voltage(bssq_dsi_reg, 1200000, 1200000);
		}

		regulator_enable(bssq_dsi_reg);
		mdelay(1);

		is_dsi_panel_disable = 0;
		is_dsi_panel_enable = 1;
	}

	return 0;
}
// --- kyouonghoon.lim@lge.com

static int bssq_dsi_panel_postsuspend(void)
{

	if(is_dsi_panel_poweroff)	//already power off
	{
		printk("already power off.\n");
		return 0;
	}

	printk("bssq_dsi_panel_postsuspend\n");

	if(is_dsi_panel_disable == 0)
	{
		printk("is_dsi_panel_disable = 0\n");

		if (!is_dsi_panel_enable) {
			bssq_dsi_reg = regulator_get(NULL, "vddio_mipi");
			if (IS_ERR_OR_NULL(bssq_dsi_reg)) {
				pr_err("dsi: couldn't get regulator vddio_mipi\n");
				bssq_dsi_reg = NULL;
				return PTR_ERR(bssq_dsi_reg);
			}
			regulator_set_voltage(bssq_dsi_reg, 1200000, 1200000);
			regulator_enable(bssq_dsi_reg);
			mdelay(1);
		}

		if (bssq_dsi_reg) {
			regulator_disable(bssq_dsi_reg);
		}

		is_dsi_panel_disable = 1;
		is_dsi_panel_enable = 0;
	}

	bd6084_bl_set_ldo(BD6084_LDO2, 0);
	bd6084_bl_set_ldo(BD6084_LDO1, 0);
	mdelay(1);

	gpio_set_value(BSSQ_DSI_PANEL_RESET, 0);
	gpio_set_value(bssq_dsi_panel_cs, 0);
	mdelay(100);
	
	is_dsi_panel_poweroff = 1;
	is_dsi_panel_poweron = 0;
	
	return 0;
}

#ifdef CONFIG_TEGRA_DC
static int bssq_hdmi_enable(void)
{
	if (!bssq_hdmi_reg) {
		bssq_hdmi_reg = regulator_get(NULL, "avdd_hdmi"); /* LDO10 */
		if (IS_ERR_OR_NULL(bssq_hdmi_reg)) {
			pr_err("hdmi: couldn't get regulator avdd_hdmi\n");
			bssq_hdmi_reg = NULL;
			return PTR_ERR(bssq_hdmi_reg);
		}
		// 20110906 hyokmin.kwon@lge.com set voltage 3.3v for hdmi
		regulator_set_voltage(bssq_hdmi_reg, 3300000, 3300000);
	}
	regulator_enable(bssq_hdmi_reg);

	if (!bssq_hdmi_pll) {
		bssq_hdmi_pll = regulator_get(NULL, "avdd_hdmi_pll"); /* LDO6 */
		if (IS_ERR_OR_NULL(bssq_hdmi_pll)) {
			pr_err("hdmi: couldn't get regulator avdd_hdmi_pll\n");
			bssq_hdmi_pll = NULL;
			regulator_disable(bssq_hdmi_reg);
			bssq_hdmi_reg = NULL;
			return PTR_ERR(bssq_hdmi_pll);
		}
	}
	regulator_enable(bssq_hdmi_pll);
	return 0;
}

static int bssq_hdmi_disable(void)
{
	regulator_disable(bssq_hdmi_reg);
	regulator_disable(bssq_hdmi_pll);
	return 0;
}

static struct resource bssq_disp1_resources[] = {
	{
		.name	= "irq",
		.start	= INT_DISPLAY_GENERAL,
		.end	= INT_DISPLAY_GENERAL,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name	= "regs",
		.start	= TEGRA_DISPLAY_BASE,
		.end	= TEGRA_DISPLAY_BASE + TEGRA_DISPLAY_SIZE-1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "fbmem",
		.start	= 0x18012000,
		.end	= 0x18189000 - 1,
		.flags	= IORESOURCE_MEM,
	},
#ifdef CONFIG_TEGRA_DSI
	{
		.name	= "dsi_regs",
		.start	= TEGRA_DSI_BASE,
		.end	= TEGRA_DSI_BASE + TEGRA_DSI_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
#endif
};

static struct resource bssq_disp2_resources[] = {
	{
		.name	= "irq",
		.start	= INT_DISPLAY_B_GENERAL,
		.end	= INT_DISPLAY_B_GENERAL,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name	= "regs",
		.start	= TEGRA_DISPLAY2_BASE,
		.end	= TEGRA_DISPLAY2_BASE + TEGRA_DISPLAY2_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "fbmem",
		.flags	= IORESOURCE_MEM,
		.start	= 0x18414000,
		.end	= 0x18BFD000 - 1,
	},
	{
		.name	= "hdmi_regs",
		.start	= TEGRA_HDMI_BASE,
		.end	= TEGRA_HDMI_BASE + TEGRA_HDMI_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct tegra_dc_mode bssq_panel_modes[] = {
	{
		.pclk = 27000000,
		.h_ref_to_sync = 4,
		.v_ref_to_sync = 2,
		.h_sync_width = 10,
		.v_sync_width = 3,
		.h_back_porch = 20,
		.v_back_porch = 3,
		.h_active = 800,
		.v_active = 480,
		.h_front_porch = 70,
		.v_front_porch = 3,
	},
};

static struct tegra_dc_out_pin bssq_dc_out_pins[] = {
	{
		.name	= TEGRA_DC_OUT_PIN_H_SYNC,
		.pol	= TEGRA_DC_OUT_PIN_POL_LOW,
	},
	{
		.name	= TEGRA_DC_OUT_PIN_V_SYNC,
		.pol	= TEGRA_DC_OUT_PIN_POL_LOW,
	},
	{
		.name	= TEGRA_DC_OUT_PIN_PIXEL_CLOCK,
		.pol	= TEGRA_DC_OUT_PIN_POL_LOW,
	},
};

static u8 bssq_dc_out_pin_sel_config[] = {
	TEGRA_PIN_OUT_CONFIG_SEL_LM1_PM1,
};

#ifdef CONFIG_TEGRA_DSI

#if (HITACH_PANEL)

	static u8 s_hitachi_mipi_setting[] = {0xBC,0x12,0x8A,0x02,0x04,0xFF,0xFF,0xFF,0x10,0xFF,0xFF,0x00,0xA6,0x14,0x0A,0x19,0x00,0x00,0xFF};//0xBC
	static u8 s_hitachi_set_ddvdhp[] = {0xB7,0x1A,0x33,0x03,0x03,0x03,0x00,0x00,0x01,0x02,0x00,0x00,0x00,0x00,0x01,0x01,0x01};//0xB7
	static u8 s_hitachi_set_ddvdhm[] = {0xB8,0x1C,0x53,0x03,0x03,0x00,0x01,0x02,0x00,0x00,0x04,0x00,0x01,0x01};//0xB8
	static u8 s_hitachi_set_vgh[] = {0xB9,0x0A,0x01,0x01,0x00,0x00,0x00,0x02,0x00,0x02,0x01};//0xB9
	static u8 s_hitachi_set_vgl[] = {0xBA,0x0F,0x01,0x01,0x00,0x00,0x00,0x02,0x00,0x02,0x01};//0xBA
	static u8 s_hitachi_source_precharge_timming[] = {0xC6,0xC4,0x04};//0xC6
	static u8 s_hitachi_gateset_3[] = {0xCA,0x04,0x04};//0xCA
	static u8 s_hitachi_ponseqa[] = {0xD8,0x01,0x05,0x06,0x0D,0x18,0x09,0x22,0x23,0x00};//0xD8
	static u8 s_hitachi_ponseqc[] = {0xDE,0x09,0x0F,0x21,0x12,0x04};//0xDE
	static u8 s_hitachi_gamma_r_pos[] = {0xEB,0x01,0x33,0x12,0x19,0xC7,0x56,0x55,0x0F};//0xEB
	static u8 s_hitachi_gamma_r_nes[] = {0xEC,0x01,0x33,0x12,0x19,0xC7,0x56,0x55,0x0F};//0xEC
	static u8 s_hitachi_gamma_g_pos[] = {0xED,0x01,0x33,0x12,0x19,0xC7,0x56,0x55,0x0F};//0xED
	static u8 s_hitachi_gamma_g_nes[] = {0xEE,0x01,0x33,0x12,0x19,0xC7,0x56,0x55,0x0F};//0xEE
	static u8 s_hitachi_gamma_b_pos[] = {0xEF,0x01,0x33,0x12,0x19,0xC7,0x56,0x55,0x0F};//0xEF
	static u8 s_hitachi_gamma_b_nes[] = {0xF0,0x01,0x33,0x12,0x19,0xC7,0x56,0x55,0x0F};//0xF0
	static u8 s_hitachi_set_tear_scanline[] = {0x44,0x00,0x00};//0x44
	
	static struct tegra_dsi_cmd dsi_init_cmd[]= {
		
		DSI_CMD_LONG(0x39, s_hitachi_mipi_setting),
		
		DSI_CMD_SHORT(0x15, 0x36, 0x0A),	// hitachi_set_addr_mode
		DSI_CMD_SHORT(0x23, 0xB4, 0xAA),	// hitachi_set_vgmpm
		
		DSI_CMD_LONG(0x29, s_hitachi_set_ddvdhp),
		DSI_CMD_LONG(0x29, s_hitachi_set_ddvdhm),
		DSI_CMD_LONG(0x29, s_hitachi_set_vgh),
		DSI_CMD_LONG(0x29, s_hitachi_set_vgl),
		
		DSI_CMD_SHORT(0x23, 0xC1, 0x01),	// hitachi_num_of_line
		DSI_CMD_SHORT(0x23, 0xC4, 0x4C),	// hitachi_1h_period
		DSI_CMD_SHORT(0x23, 0xC5, 0x03),	// hitachi_source_precharge
		
		DSI_CMD_LONG(0x29, s_hitachi_source_precharge_timming),
		DSI_CMD_LONG(0x29, s_hitachi_gateset_3),
		
		DSI_CMD_SHORT(0x23, 0xD6, 0x02),	// hitachi_dotinv
		
		DSI_CMD_LONG(0x29, s_hitachi_ponseqa),
		DSI_CMD_LONG(0x29, s_hitachi_ponseqc),
		
		DSI_CMD_SHORT(0x23, 0x53, 0x40),	// hitachi_backlightctl
		DSI_CMD_SHORT(0x23, 0xEA, 0x01),	// hitachi_high_speed_ram
		
		DSI_CMD_LONG(0x29, s_hitachi_gamma_r_pos),
		DSI_CMD_LONG(0x29, s_hitachi_gamma_r_nes),
		DSI_CMD_LONG(0x29, s_hitachi_gamma_g_pos),
		DSI_CMD_LONG(0x29, s_hitachi_gamma_g_nes),
		DSI_CMD_LONG(0x29, s_hitachi_gamma_b_pos),
		DSI_CMD_LONG(0x29, s_hitachi_gamma_b_nes),
		
		DSI_CMD_LONG(0x39, s_hitachi_set_tear_scanline),
		
		DSI_CMD_SHORT(0x05, 0x11, 0x00),	//sleep out
		DSI_DLY_MS(120),
		DSI_CMD_SHORT(0x05, 0x29, 0x00),	//main display on
#if(DC_CTRL_MODE == TEGRA_DC_OUT_ONE_SHOT_MODE)
		DSI_CMD_SHORT(0x15, 0x35, 0x00)		//set tear on
#endif
	};

	static struct tegra_dsi_cmd dsi_sleep_out_cmd[]= {
		DSI_CMD_SHORT(0x05, 0x11, 0x00),	//sleep out
		DSI_DLY_MS(120),
		DSI_CMD_SHORT(0x05, 0x29, 0x00),	//main display on
#if(DC_CTRL_MODE == TEGRA_DC_OUT_ONE_SHOT_MODE)
		DSI_CMD_SHORT(0x15, 0x35, 0x00)		//set tear on
#endif
	};

	static struct tegra_dsi_cmd dsi_sleep_in_cmd[]= {
#if(DC_CTRL_MODE == TEGRA_DC_OUT_ONE_SHOT_MODE)
		DSI_CMD_SHORT(0x05, 0x34, 0x00),	//set tear off
#endif
		// sleep in
		DSI_CMD_SHORT(0x05, 0x28, 0x00),
		DSI_DLY_MS(35),
		DSI_CMD_SHORT(0x05, 0x10, 0x00),
		DSI_DLY_MS(60),
	};

	static struct tegra_dsi_cmd dsi_end_cmd[]= {

#if(DC_CTRL_MODE == TEGRA_DC_OUT_ONE_SHOT_MODE)
		DSI_CMD_SHORT(0x05, 0x34, 0x00),	//set tear off
#endif
		// sleep in
		DSI_CMD_SHORT(0x05, 0x28, 0x00),
		DSI_DLY_MS(35),
		DSI_CMD_SHORT(0x05, 0x10, 0x00),
		DSI_DLY_MS(60),
	};

#else //(LGD_PANEL)

	static u8 lh400wv3_panel_chr_setting[] = {0xB2, 0x00, 0xC8};
	static u8 lh400wv3_display_ctrl1[] = {0xB5, 0x40, 0x18, 0x02, 0x04, 0x20};
	static u8 lh400wv3_display_ctrl2[] = {0xB6, 0x0B, 0x0F, 0x02, 0x40, 0x10, 0xE8};
	
	//default
	//static u8 lh400wv3_display_ctrl3[] = {0xB7, 0x46, 0x06, 0x0C, 0x00, 0x00};
	//static u8 lh400wv3_internal_osc_setting[] = {0xC0, 0x01, 0x15};
	
	//60Hz
	// 20110629 sangki.hyun@lge.com Wave noise [S] {
	// static u8 lh400wv3_display_ctrl3[] = {0xB7, 0x46, 0x06, 0x2F, 0x00, 0x00}; //for inclease back forch
	static u8 lh400wv3_display_ctrl3[] = {0xB7, 0x48, 0x06, 0x2E, 0x00, 0x00}; //for inclease back forch
	// 20110629 sangki.hyun@lge.com Wave noise [E] }
	static u8 lh400wv3_internal_osc_setting[] = {0xC0, 0x01, 0x10};
	
	// 50Hz
	//static u8 lh400wv3_display_ctrl3[] = {0xB7, 0x46, 0x06, 0x0C, 0x00, 0x00}; //for declease back forch
	//static u8 lh400wv3_internal_osc_setting[] = {0xC0, 0x01, 0x0F};

	static u8 lh400wv3_power_ctrl3[] = {0xC3, 0x07, 0x03, 0x04, 0x04, 0x04};
	static u8 lh400wv3_power_ctrl4[] = {0xC4, 0x12, 0x24, 0x18, 0x18, 0x05, 0x49};
	static u8 lh400wv3_power_ctrl6[] = {0xC6, 0x41, 0x63};
	
	// 20110517 julius.moon LCD Gamma Tunning 2011/05/13 [S]
	#undef USING_GAMMA_SET_INITIAL
	#undef USING_GAMMA_SET_ONE
	#undef USING_GAMMA_SET_TWO
	#undef USING_GAMMA_SET_THREE
	#undef USING_GAMMA_SET_FOUR
	#undef USING_GAMMA_SET_0513
	#undef USING_GAMMA_SET_0527
	#define USING_GAMMA_SET_0712
	
	#if defined USING_GAMMA_SET_INITIAL
	//gamma setting - inital value
	static u8 lh400wv3_positive_gamma_red[] = {0xD0, 0x20, 0x07, 0x72, 0x02, 0x00, 0x00, 0x10, 0x00, 0x02};
	static u8 lh400wv3_nagative_gamma_red[] = {0xD1, 0x20, 0x07, 0x72, 0x02, 0x00, 0x00, 0x10, 0x00, 0x02};
	static u8 lh400wv3_positive_gamma_green[] = {0xD2, 0x20, 0x07, 0x72, 0x02, 0x00, 0x00, 0x10, 0x00, 0x02};
	static u8 lh400wv3_nagative_gamma_green[] = {0xD3, 0x20, 0x07, 0x72, 0x02, 0x00, 0x00, 0x10, 0x00, 0x02};
	static u8 lh400wv3_positive_gamma_blue[] = {0xD4, 0x20, 0x07, 0x72, 0x02, 0x00, 0x00, 0x10, 0x00, 0x02};
	static u8 lh400wv3_nagative_gamma_blue[] = {0xD5, 0x20, 0x07, 0x72, 0x02, 0x00, 0x00, 0x10, 0x00, 0x02};
	#elif defined(USING_GAMMA_SET_ONE)
	//gamma setting - S-Curve (1) value
	static u8 lh400wv3_positive_gamma_red[] = {0xD0, 0x10, 0x07, 0x74, 0x02, 0x00, 0x00, 0x10, 0x00, 0x02};
	static u8 lh400wv3_nagative_gamma_red[] = {0xD1, 0x10, 0x07, 0x74, 0x02, 0x00, 0x00, 0x10, 0x00, 0x02};
	static u8 lh400wv3_positive_gamma_green[] = {0xD2, 0x10, 0x07, 0x74, 0x02, 0x00, 0x00, 0x10, 0x00, 0x02};
	static u8 lh400wv3_nagative_gamma_green[] = {0xD3, 0x10, 0x07, 0x74, 0x02, 0x00, 0x00, 0x10, 0x00, 0x02};
	static u8 lh400wv3_positive_gamma_blue[] = {0xD4, 0x10, 0x07, 0x74, 0x02, 0x00, 0x00, 0x10, 0x00, 0x02};
	static u8 lh400wv3_nagative_gamma_blue[] = {0xD5, 0x10, 0x07, 0x74, 0x02, 0x00, 0x00, 0x10, 0x00, 0x02};
	#elif defined(USING_GAMMA_SET_TWO)
	//gamma setting - S-Curve (2) value
	static u8 lh400wv3_positive_gamma_red[] = {0xD0, 0x00, 0x07, 0x75, 0x02, 0x00, 0x00, 0x10, 0x00, 0x02};
	static u8 lh400wv3_nagative_gamma_red[] = {0xD1, 0x00, 0x07, 0x75, 0x02, 0x00, 0x00, 0x10, 0x00, 0x02};
	static u8 lh400wv3_positive_gamma_green[] = {0xD2, 0x00, 0x07, 0x75, 0x02, 0x00, 0x00, 0x10, 0x00, 0x02};
	static u8 lh400wv3_nagative_gamma_green[] = {0xD3, 0x00, 0x07, 0x75, 0x02, 0x00, 0x00, 0x10, 0x00, 0x02};
	static u8 lh400wv3_positive_gamma_blue[] = {0xD4, 0x00, 0x07, 0x75, 0x02, 0x00, 0x00, 0x10, 0x00, 0x02};
	static u8 lh400wv3_nagative_gamma_blue[] = {0xD5, 0x00, 0x07, 0x75, 0x02, 0x00, 0x00, 0x10, 0x00, 0x02};
	#elif defined(USING_GAMMA_SET_THREE)
	//gamma setting - S-Curve (3) value
	static u8 lh400wv3_positive_gamma_red[] = {0xD0, 0x00, 0x07, 0x77, 0x02, 0x00, 0x00, 0x10, 0x00, 0x02};
	static u8 lh400wv3_nagative_gamma_red[] = {0xD1, 0x00, 0x07, 0x77, 0x02, 0x00, 0x00, 0x10, 0x00, 0x02};
	static u8 lh400wv3_positive_gamma_green[] = {0xD2, 0x00, 0x07, 0x77, 0x02, 0x00, 0x00, 0x10, 0x00, 0x02};
	static u8 lh400wv3_nagative_gamma_green[] = {0xD3, 0x00, 0x07, 0x77, 0x02, 0x00, 0x00, 0x10, 0x00, 0x02};
	static u8 lh400wv3_positive_gamma_blue[] = {0xD4, 0x00, 0x07, 0x77, 0x02, 0x00, 0x00, 0x10, 0x00, 0x02};
	static u8 lh400wv3_nagative_gamma_blue[] = {0xD5, 0x00, 0x07, 0x77, 0x02, 0x00, 0x00, 0x10, 0x00, 0x02};
	#elif defined(USING_GAMMA_SET_FOUR)
	//gamma setting - S-Curve (4) value
	static u8 lh400wv3_positive_gamma_red[] = {0xD0, 0x00, 0x05, 0x77, 0x02, 0x00, 0x00, 0x10, 0x00, 0x02};
	static u8 lh400wv3_nagative_gamma_red[] = {0xD1, 0x00, 0x05, 0x77, 0x02, 0x00, 0x00, 0x10, 0x00, 0x02};
	static u8 lh400wv3_positive_gamma_green[] = {0xD2, 0x00, 0x05, 0x77, 0x02, 0x00, 0x00, 0x10, 0x00, 0x02};
	static u8 lh400wv3_nagative_gamma_green[] = {0xD3, 0x00, 0x05, 0x77, 0x02, 0x00, 0x00, 0x10, 0x00, 0x02};
	static u8 lh400wv3_positive_gamma_blue[] = {0xD4, 0x00, 0x05, 0x77, 0x02, 0x00, 0x00, 0x10, 0x00, 0x02};
	static u8 lh400wv3_nagative_gamma_blue[] = {0xD5, 0x00, 0x05, 0x77, 0x02, 0x00, 0x00, 0x10, 0x00, 0x02};
	#elif defined(USING_GAMMA_SET_0513)
	//gamma setting - S-Curve value 2011.5.13. code "BQ_Initial Code_v0.2_110513.pdf"
	static u8 lh400wv3_positive_gamma_red[] = {0xD0, 0x00, 0x07, 0x75, 0x03, 0x00, 0x01, 0x20, 0x00, 0x03};
	static u8 lh400wv3_nagative_gamma_red[] = {0xD1, 0x00, 0x07, 0x75, 0x03, 0x00, 0x01, 0x20, 0x00, 0x03};
	static u8 lh400wv3_positive_gamma_green[] = {0xD2, 0x00, 0x07, 0x75, 0x03, 0x00, 0x01, 0x20, 0x00, 0x03};
	static u8 lh400wv3_nagative_gamma_green[] = {0xD3, 0x00, 0x07, 0x75, 0x03, 0x00, 0x01, 0x20, 0x00, 0x03};
	static u8 lh400wv3_positive_gamma_blue[] = {0xD4, 0x00, 0x07, 0x75, 0x03, 0x00, 0x01, 0x20, 0x00, 0x03};
	static u8 lh400wv3_nagative_gamma_blue[] = {0xD5, 0x00, 0x07, 0x75, 0x03, 0x00, 0x01, 0x20, 0x00, 0x03};
	#elif defined(USING_GAMMA_SET_0527)
	//gamma setting - S-Curve value 2011.5.27. code "LH400WV3-SD02_Initial Code_v0.3_110527.pdf"
	static u8 lh400wv3_positive_gamma_red[] = {0xD0, 0x01, 0x26, 0x71, 0x16, 0x04, 0x03, 0x51, 0x15, 0x04};
	static u8 lh400wv3_nagative_gamma_red[] = {0xD1, 0x01, 0x26, 0x71, 0x16, 0x04, 0x03, 0x51, 0x15, 0x04};
	static u8 lh400wv3_positive_gamma_green[] = {0xD2, 0x01, 0x26, 0x71, 0x16, 0x04, 0x03, 0x51, 0x15, 0x04};
	static u8 lh400wv3_nagative_gamma_green[] = {0xD3, 0x01, 0x26, 0x71, 0x16, 0x04, 0x03, 0x51, 0x15, 0x04};
	static u8 lh400wv3_positive_gamma_blue[] = {0xD4, 0x01, 0x26, 0x71, 0x16, 0x04, 0x03, 0x51, 0x15, 0x04};
	static u8 lh400wv3_nagative_gamma_blue[] = {0xD5, 0x01, 0x26, 0x71, 0x16, 0x04, 0x03, 0x51, 0x15, 0x04};
	#elif defined(USING_GAMMA_SET_0712)
	//gamma setting - S-Curve value 2011.5.27. code "LH400WV3-SD02_Initial Code_v0.3_110527.pdf"
	static u8 lh400wv3_positive_gamma_red[] = {0xD0, 0x04, 0x26, 0x71, 0x16, 0x04, 0x03, 0x51, 0x15, 0x04};
	static u8 lh400wv3_nagative_gamma_red[] = {0xD1, 0x04, 0x26, 0x71, 0x16, 0x04, 0x03, 0x51, 0x15, 0x04};
	static u8 lh400wv3_positive_gamma_green[] = {0xD2, 0x04, 0x26, 0x71, 0x16, 0x04, 0x03, 0x51, 0x15, 0x04};
	static u8 lh400wv3_nagative_gamma_green[] = {0xD3, 0x04, 0x26, 0x71, 0x16, 0x04, 0x03, 0x51, 0x15, 0x04};
	static u8 lh400wv3_positive_gamma_blue[] = {0xD4, 0x04, 0x26, 0x71, 0x16, 0x04, 0x03, 0x51, 0x15, 0x04};
	static u8 lh400wv3_nagative_gamma_blue[] = {0xD5, 0x04, 0x26, 0x71, 0x16, 0x04, 0x03, 0x51, 0x15, 0x04};
	#else
	#error
	#endif
	// 20110517 julius.moon LCD Gamma Tunning 2011/05/13 [E]
	
	static struct tegra_dsi_cmd dsi_init_cmd[]= {
		//display mode setting
		DSI_CMD_SHORT(0x05, 0x20, 0x00),

#if(DC_CTRL_MODE == TEGRA_DC_OUT_ONE_SHOT_MODE)
		DSI_CMD_SHORT(0x15, 0x35, 0x00),
#else
		//tear effect off
		DSI_CMD_SHORT(0x05, 0x34, 0x00),
#endif

		DSI_CMD_SHORT(0x15, 0x36, 0x00),
		DSI_CMD_SHORT(0x15, 0x3A, 0x77),
		DSI_CMD_LONG(0x39, lh400wv3_panel_chr_setting),
		DSI_CMD_SHORT(0x15, 0xB3, 0x00),
		
		//DSI_CMD_SHORT(0x15, 0xB4, 0x00),
		DSI_CMD_SHORT(0x15, 0xB4, 0x04),	// khbin : LGD guide. This makes 18bit -> 24bit.
		
		DSI_CMD_LONG(0x39, lh400wv3_display_ctrl1),
		DSI_CMD_LONG(0x39, lh400wv3_display_ctrl2),
		DSI_CMD_LONG(0x39, lh400wv3_display_ctrl3),
			
		//power setting
		DSI_CMD_LONG(0x39, lh400wv3_internal_osc_setting),
		DSI_CMD_LONG(0x39, lh400wv3_power_ctrl3),
		DSI_CMD_LONG(0x39, lh400wv3_power_ctrl4),
		DSI_CMD_SHORT(0x15, 0xC5, 0x69),
		DSI_CMD_LONG(0x39, lh400wv3_power_ctrl6),
		//gamma setting
		DSI_CMD_LONG(0x39, lh400wv3_positive_gamma_red),
		DSI_CMD_LONG(0x39, lh400wv3_nagative_gamma_red),
		DSI_CMD_LONG(0x39, lh400wv3_positive_gamma_green),
		DSI_CMD_LONG(0x39, lh400wv3_nagative_gamma_green),
		DSI_CMD_LONG(0x39, lh400wv3_positive_gamma_blue),
		DSI_CMD_LONG(0x39, lh400wv3_nagative_gamma_blue),
		//sleep out
		DSI_CMD_SHORT(0x05, 0x11, 0x00),
		DSI_DLY_MS(120),
		//main Display on
		DSI_CMD_SHORT(0x05, 0x29, 0x00),
	};

	static struct tegra_dsi_cmd dsi_sleep_out_cmd[]= {
		//sleep out
		DSI_CMD_SHORT(0x05, 0x11, 0x00),
		DSI_DLY_MS(120),
		//main Display on
		DSI_CMD_SHORT(0x05, 0x29, 0x00),
	};

	static struct tegra_dsi_cmd dsi_sleep_in_cmd[]= {
		// sleep in
		DSI_CMD_SHORT(0x05, 0x28, 0x00),
		DSI_DLY_MS(40),
		DSI_CMD_SHORT(0x05, 0x10, 0x00),
		DSI_DLY_MS(60)
	};

	static struct tegra_dsi_cmd dsi_end_cmd[]= {
		// deep sleep in
		DSI_CMD_SHORT(0x05, 0x10, 0x00),
		DSI_DLY_MS(150),
		DSI_CMD_SHORT(0x15, 0xC1, 0x01),
		DSI_CMD_SHORT(0x15, 0xC1, 0x01),
		DSI_DLY_MS(10),
	};
#endif

struct tegra_dsi_out bssq_dsi = {
	.n_data_lanes = 2,
	.pixel_format = TEGRA_DSI_PIXEL_FORMAT_24BIT_P,
#if defined(CONFIG_LU6500)
	.refresh_rate = 64,
#else
	.refresh_rate = 62,
#endif
	.virtual_channel = TEGRA_DSI_VIRTUAL_CHANNEL_0,

	.panel_has_frame_buffer = true,

	.n_init_cmd = ARRAY_SIZE(dsi_init_cmd),
	.dsi_init_cmd = dsi_init_cmd,
	
	.n_suspend_cmd = ARRAY_SIZE(dsi_end_cmd),
	.dsi_suspend_cmd = dsi_end_cmd,

	.n_sleep_in_cmd = ARRAY_SIZE(dsi_sleep_in_cmd),
	.dsi_sleep_in_cmd = dsi_sleep_in_cmd,

	.n_sleep_out_cmd = ARRAY_SIZE(dsi_sleep_out_cmd),
	.dsi_sleep_out_cmd = dsi_sleep_out_cmd,
	
#if defined(CONFIG_LU6500)
	.video_clock_mode = TEGRA_DSI_VIDEO_CLOCK_CONTINUOUS,
#else
	.video_clock_mode = TEGRA_DSI_VIDEO_CLOCK_TX_ONLY,
#endif

	.video_data_type = TEGRA_DSI_VIDEO_TYPE_COMMAND_MODE,
//	.video_data_type = TEGRA_DSI_VIDEO_TYPE_VIDEO_MODE,
};

static struct tegra_dc_mode bssq_dsi_modes[] = {
	{
#if defined(CONFIG_LU6500)
		.pclk = 27000000,
		.h_ref_to_sync = 4,
		.v_ref_to_sync = 1,
		.h_sync_width = 4,
		.v_sync_width = 1,
		.h_back_porch = 4,
		.v_back_porch = 1,
		.h_active = 480,
		.v_active = 800,
		.h_front_porch = 40,
		.v_front_porch = 2,
#else
		.pclk = 27000000,
		.h_ref_to_sync = 4,
		.v_ref_to_sync = 1,
		.h_sync_width = 16,
		.v_sync_width = 1,
		.h_back_porch = 32,
		.v_back_porch = 1,
		.h_active = 480,
		.v_active = 800,
		.h_front_porch = 64,
		.v_front_porch = 2,
#endif
	},
};
#endif	//#ifdef CONFIG_TEGRA_DSI

static struct tegra_dc_out bssq_disp1_out = {
	.align		= TEGRA_DC_ALIGN_MSB,
	.order		= TEGRA_DC_ORDER_RED_BLUE,

#ifndef CONFIG_TEGRA_DSI
	.type		= TEGRA_DC_OUT_RGB,

	.modes	 	= bssq_panel_modes,
	.n_modes 	= ARRAY_SIZE(bssq_panel_modes),

	.out_pins	= bssq_dc_out_pins,
	.n_out_pins	= ARRAY_SIZE(bssq_dc_out_pins),

	.out_sel_configs   = bssq_dc_out_pin_sel_config,
	.n_out_sel_configs = ARRAY_SIZE(bssq_dc_out_pin_sel_config),

#else
	.type		= TEGRA_DC_OUT_DSI,
	.flags		= DC_CTRL_MODE,
	.modes	 	= bssq_dsi_modes,
	.n_modes 	= ARRAY_SIZE(bssq_dsi_modes),

	.dsi		= &bssq_dsi,

	.enable		= bssq_dsi_panel_enable,
	.disable	= bssq_dsi_panel_disable,
	.poweron	= bssq_dsi_panel_poweron,
	.postsuspend = bssq_dsi_panel_postsuspend,
	.postpoweron = bssq_dsi_panel_postpoweron,
#endif
};

static struct tegra_dc_out bssq_disp2_out = {
	.type		= TEGRA_DC_OUT_HDMI,
	.flags		= TEGRA_DC_OUT_HOTPLUG_HIGH,

	.dcc_bus	= 1,
	.hotplug_gpio	= bssq_hdmi_hpd,

	.max_pixclock	= KHZ2PICOS(148500),

	.align		= TEGRA_DC_ALIGN_MSB,
	.order		= TEGRA_DC_ORDER_RED_BLUE,

	.enable		= bssq_hdmi_enable,
	.disable	= bssq_hdmi_disable,
};

static struct tegra_fb_data bssq_fb_data = {
	.win		= 0,
	.xres		= 800,
	.yres		= 480,
	.bits_per_pixel	= 32,
	.flags		= TEGRA_FB_FLIP_ON_PROBE,
};

static struct tegra_fb_data bssq_fb_dsi_data = {
	.win		= 0,
	.xres		= 480,
	.yres		= 800,
	.bits_per_pixel	= 32,
};

static struct tegra_fb_data bssq_hdmi_fb_data = {
	.win		= 0,
	.xres		= 800,
	.yres		= 480,
	.bits_per_pixel	= 32,
	.flags		= TEGRA_FB_FLIP_ON_PROBE,
};


static struct tegra_dc_platform_data bssq_disp1_pdata = {
	.flags		= TEGRA_DC_FLAG_ENABLED,
	.default_out	= &bssq_disp1_out,
#ifndef CONFIG_TEGRA_DSI
	.fb		= &bssq_fb_data,
#else
	.fb		= &bssq_fb_dsi_data,
#endif
};

static struct nvhost_device bssq_disp1_device = {
	.name		= "tegradc",
	.id		= 0,
	.resource	= bssq_disp1_resources,
	.num_resources	= ARRAY_SIZE(bssq_disp1_resources),
	.dev = {
		.platform_data = &bssq_disp1_pdata,
	},
};

static int bssq_disp1_check_fb(struct device *dev, struct fb_info *info)
{
	return info->device == &bssq_disp1_device.dev;
}

static struct tegra_dc_platform_data bssq_disp2_pdata = {
	.flags		= 0,
	.default_out	= &bssq_disp2_out,
	.fb		= &bssq_hdmi_fb_data,
};

static struct nvhost_device bssq_disp2_device = {
	.name		= "tegradc",
	.id		= 1,
	.resource	= bssq_disp2_resources,
	.num_resources	= ARRAY_SIZE(bssq_disp2_resources),
	.dev = {
		.platform_data = &bssq_disp2_pdata,
	},
};
#else
static int bssq_disp1_check_fb(struct device *dev, struct fb_info *info)
{
	return 0;
}
#endif

static struct nvmap_platform_carveout bssq_carveouts[] = {
	[0] = NVMAP_HEAP_CARVEOUT_IRAM_INIT,
	[1] = {
		.name		= "generic-0",
		.usage_mask	= NVMAP_HEAP_CARVEOUT_GENERIC,
		.base		= 0,	/* Filled in by bssq_panel_init() */
		.size		= 0,	/* Filled in by bssq_panel_init() */
		.buddy_size	= SZ_32K,
	},
};

static struct nvmap_platform_data bssq_nvmap_data = {
	.carveouts	= bssq_carveouts,
	.nr_carveouts	= ARRAY_SIZE(bssq_carveouts),
};

static struct platform_device bssq_nvmap_device = {
	.name	= "tegra-nvmap",
	.id	= -1,
	.dev	= {
		.platform_data = &bssq_nvmap_data,
	},
};

static struct platform_device *bssq_gfx_devices[] __initdata = {
	&bssq_nvmap_device,
#ifdef CONFIG_TEGRA_GRHOST
	&tegra_grhost_device,
#endif
//	&bssq_disp1_backlight_device,
};

#ifdef CONFIG_HAS_EARLYSUSPEND
/* put early_suspend/late_resume handlers here for the display in order
 * to keep the code out of the display driver, keeping it closer to upstream
 */
struct early_suspend bssq_panel_early_suspender;

static void bssq_panel_early_suspend(struct early_suspend *h)
{
        /* power down LCD, add use a black screen for HDMI */
        if (num_registered_fb > 0)
                fb_blank(registered_fb[0], FB_BLANK_POWERDOWN);
        if (num_registered_fb > 1)
                fb_blank(registered_fb[1], FB_BLANK_NORMAL);

#ifdef CONFIG_TEGRA_CONVSERVATIVE_GOV_ON_EARLYSUPSEND
	cpufreq_save_default_governor();
	cpufreq_set_conservative_governor();

   cpufreq_set_conservative_governor_param("up_threshold", SET_CONSERVATIVE_GOVERNOR_UP_THRESHOLD);
   cpufreq_set_conservative_governor_param("down_threshold", SET_CONSERVATIVE_GOVERNOR_DOWN_THRESHOLD);
   cpufreq_set_conservative_governor_param("freq_step", SET_CONSERVATIVE_GOVERNOR_FREQ_STEP);
#endif
}

static void bssq_panel_late_resume(struct early_suspend *h)
{
	unsigned i;
#ifdef CONFIG_TEGRA_CONVSERVATIVE_GOV_ON_EARLYSUPSEND
	cpufreq_restore_default_governor();
#endif
	for (i = 0; i < num_registered_fb; i++)
		fb_blank(registered_fb[i], FB_BLANK_UNBLANK);
}
#endif

int __init bssq_panel_init(void)
{
	int err;
	struct resource __maybe_unused *res;

#if defined (CONFIG_LU8800) || defined (CONFIG_KU8800) || defined (CONFIG_SU880) || defined (CONFIG_KS1103)
	gpio_request(bssq_hdmi_hpd, "hdmi_hpd");
	gpio_direction_input(bssq_hdmi_hpd);
	tegra_gpio_enable(bssq_hdmi_hpd);

	gpio_request(bssq_hdmi_reg_en, "reg_en");
	gpio_direction_output(bssq_hdmi_reg_en, 1);
	tegra_gpio_enable(bssq_hdmi_reg_en);
#endif

#ifdef CONFIG_TEGRA_DSI

	bssq_disp1_out.parent_clk = "pll_d_out0";

	gpio_request(bssq_dsi_panel_cs, "lcd_cs");
	gpio_direction_output(bssq_dsi_panel_cs, 1);
	tegra_gpio_enable(bssq_dsi_panel_cs);
	gpio_request(BSSQ_DSI_PANEL_RESET, BSSQ_DSI_PANEL_RESET_STR);
	gpio_direction_output(BSSQ_DSI_PANEL_RESET, 1);
	tegra_gpio_enable(BSSQ_DSI_PANEL_RESET);
#endif

	err = i2c_register_board_info(5, tegra_i2c_gpio_info, ARRAY_SIZE(tegra_i2c_gpio_info));

#ifdef CONFIG_HAS_EARLYSUSPEND
	bssq_panel_early_suspender.suspend = bssq_panel_early_suspend;
	bssq_panel_early_suspender.resume = bssq_panel_late_resume;
	bssq_panel_early_suspender.level = EARLY_SUSPEND_LEVEL_DISABLE_FB;
	register_early_suspend(&bssq_panel_early_suspender);
#endif
	bssq_carveouts[1].base = tegra_carveout_start;
	bssq_carveouts[1].size = tegra_carveout_size;

	err = platform_add_devices(bssq_gfx_devices,
				   ARRAY_SIZE(bssq_gfx_devices));

#if defined(CONFIG_TEGRA_GRHOST) && defined(CONFIG_TEGRA_DC)
	res = nvhost_get_resource_byname(&bssq_disp1_device,
					 IORESOURCE_MEM, "fbmem");
	res->start = tegra_fb_start;
	res->end = tegra_fb_start + tegra_fb_size - 1;
#endif

	/* Copy the bootloader fb to the fb. */
	tegra_move_framebuffer(tegra_fb_start, tegra_bootloader_fb_start,
		min(tegra_fb_size, tegra_bootloader_fb_size));

	if (!err)
		err = nvhost_device_register(&bssq_disp1_device);

#if defined (CONFIG_LU8800) || defined (CONFIG_KU8800) || defined (CONFIG_SU880) || defined (CONFIG_KS1103)
#if defined(CONFIG_TEGRA_GRHOST) && defined(CONFIG_TEGRA_DC)
	res = nvhost_get_resource_byname(&bssq_disp2_device,
					 IORESOURCE_MEM, "fbmem");
	res->start = tegra_fb2_start;
	res->end = tegra_fb2_start + tegra_fb2_size - 1;

	if (!err)
		err = nvhost_device_register(&bssq_disp2_device);
#endif
#endif
	return err;
}

