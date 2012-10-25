/*
 * arch/arm/mach-tegra/board-star-panel.c
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
#include <linux/i2c.h>
#include <linux/i2c-gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/resource.h>
#include <asm/mach-types.h>
#include <linux/platform_device.h>
#include <linux/earlysuspend.h>
#include <linux/kernel.h>
#include <linux/nvhost.h>
#include <linux/backlight.h>
#include <linux/aat2870.h>
#include <linux/hitachi.h>
#include <mach/nvmap.h>
#include <mach/irqs.h>
#include <mach/iomap.h>
#include <mach/dc.h>
#include <mach/fb.h>

#include "include/lge/board-star.h"
#include <mach-tegra/devices.h>
#include <mach-tegra/gpio-names.h>
#include <mach-tegra/board.h>

#include <mach-tegra/cpu-tegra.h>

#ifdef CONFIG_TEGRA_DC
static struct regulator *star_hdmi_reg = NULL;
static struct regulator *star_hdmi_pll = NULL;
#endif

/* GPIOs for Hitachi LCD */
#define STAR_HITACHI_LCD_RESET	TEGRA_GPIO_PV7
#define STAR_HITACHI_LCD_CS	TEGRA_GPIO_PN4
#define STAR_HITACHI_LCD_DC	TEGRA_GPIO_PN6
#define STAR_HITACHI_LCD_WR	TEGRA_GPIO_PB3

#define STAR_HITACHI_LCD_D0	TEGRA_GPIO_PE0
#define STAR_HITACHI_LCD_D1	TEGRA_GPIO_PE1
#define STAR_HITACHI_LCD_D2	TEGRA_GPIO_PE2
#define STAR_HITACHI_LCD_D3	TEGRA_GPIO_PE3
#define STAR_HITACHI_LCD_D4	TEGRA_GPIO_PE4
#define STAR_HITACHI_LCD_D5	TEGRA_GPIO_PE5
#define STAR_HITACHI_LCD_D6	TEGRA_GPIO_PE6
#define STAR_HITACHI_LCD_D7	TEGRA_GPIO_PE7

static int STAR_HITACHI_DATA[8] = {
		[0] = STAR_HITACHI_LCD_D0,
		[1] = STAR_HITACHI_LCD_D1,
		[2] = STAR_HITACHI_LCD_D2,
		[3] = STAR_HITACHI_LCD_D3,
		[4] = STAR_HITACHI_LCD_D4,
		[5] = STAR_HITACHI_LCD_D5,
		[6] = STAR_HITACHI_LCD_D6,
		[7] = STAR_HITACHI_LCD_D7,
	};

#define STAR_LCD_MARKED_ID	TEGRA_GPIO_PJ5 // MOBII_CHANGE_S [jg.noh@mobii.co.kr] 2012.01.31 Supporting dual lcd driver that including both hitachi and lgd pannels

#define HITACHI_LCD_MAX_DATA_NUM	32
static int MARKED_ID;
static int bAndroidBootMode;
static int is_lcd_panel_poweroff = 0;
static int is_lcd_panel_poweron = 0;

struct hitachi_lcd_data {
	int type; /* -1=end of data, 0=command, 1=data */
	int value;
};

struct hitachi_lcd_data_set {
	struct hitachi_lcd_data data[HITACHI_LCD_MAX_DATA_NUM];
	int delay; /* msec */
};

#define CONFIG_ONE_SHOT_MODE	1

static struct hitachi_lcd_data_set hitachi_lcd_init_data[] = {
	{	//set_partial_area
		.data = {{0, 0x30}, {1, 0x00}, {1, 0x00}, {1, 0x03},
			 {1, 0x1F}, {-1, -1}},
	},
	{	 //set_scroll_area
		.data = {{0, 0x33}, {1, 0x00}, {1, 0x00}, {1, 0x03},
			 {1, 0x20}, {1, 0x00}, {1, 0x00}, {-1, -1}},
	},
	{	//set_address_mode
		.data = {{0, 0x36}, {1, 0x0A}, {-1, -1}},
	},
	{	//set_scroll_start
		.data = {{0, 0x37}, {1, 0x00}, {1, 0x00}, {-1, -1}},
	},
	{	//set_pixel_format
		.data = {{0, 0x3A}, {1, 0x07}, {-1, -1}},
	},
	{	//set_tear_scanline
		.data = {{0, 0x44}, {1, 0x00}, {1, 0x00}, {-1, -1}},
	},
	{	//Ex_Vsync_en
		.data = {{0, 0x71}, {1, 0x00}, {-1, -1}},
	},
	{	//VCSEL
		.data = {{0, 0xB2}, {1, 0x00}, {-1, -1}},
	},
	{	//setvgmpm
		.data = {{0, 0xB4}, {1, 0xAA}, {-1, -1}},
	},
	{	//rbias1
		.data = {{0, 0xB5}, {1, 0x33}, {-1, -1}},
	},
	{	//rbias2
		.data = {{0, 0xB6}, {1, 0x03}, {-1, -1}},
	},
	{	//set_ddvdhp
		.data = {{0, 0xB7}, {1, 0x1A}, {1, 0x33}, {1, 0x03},
			 {1, 0x03}, {1, 0x03}, {1, 0x00}, {1, 0x00},
		  	 {1, 0x01}, {1, 0x02}, {1, 0x00}, {1, 0x00},
			 {1, 0x04}, {1, 0x00}, {1, 0x01}, {1, 0x01},
		 	 {1, 0x01}, {-1, -1}},
	},
	{	//set_ddvdhm
		.data = {{0, 0xB8}, {1, 0x1C}, {1, 0x53}, {1, 0x03},
			 {1, 0x03}, {1, 0x00}, {1, 0x01}, {1, 0x02},
		 	 {1, 0x00}, {1, 0x00}, {1, 0x04}, {1, 0x00},
		 	 {1, 0x01}, {1, 0x01}, {-1, -1}},
	},
	{	//set_vgh
		.data = {{0, 0xB9}, {1, 0x0A}, {1, 0x01}, {1, 0x01},
			 {1, 0x00}, {1, 0x00}, {1, 0x00}, {1, 0x02},
		 	 {1, 0x00}, {1, 0x02}, {1, 0x01}, {-1, -1}},
	},
	{	//set_vgl
		.data = {{0, 0xBA}, {1, 0x0F}, {1, 0x01}, {1, 0x01},
			 {1, 0x00}, {1, 0x00}, {1, 0x00}, {1, 0x02},
			 {1, 0x00}, {1, 0x02}, {1, 0x01}, {-1, -1}},
	},
	{	//set_vcl
		.data = {{0, 0xBB}, {1, 0x00}, {1, 0x00}, {1, 0x00},
			 {1, 0x00}, {1, 0x01}, {1, 0x02}, {1, 0x01},
			 {-1, -1}},
	},
	{	//number of lines
		.data = {{0, 0xC1}, {1, 0x01}, {-1, -1}},
	},
	{	//number of fp lines
		.data = {{0, 0xC2}, {1, 0x08}, {-1, -1}},
	},
	{	//gateset(1)
		.data = {{0, 0xC3}, {1, 0x04}, {-1, -1}},
	},
	{	//1h period
		.data = {{0, 0xC4}, {1, 0x4C}, {-1, -1}},
	},
	{	//source precharge
		.data = {{0, 0xC5}, {1, 0x03}, {-1, -1}},
	},
	{	//source precharge timing
		.data = {{0, 0xC6}, {1, 0xC4}, {1, 0x04}, {-1, -1}},
	},
	{	//source level
		.data = {{0, 0xC7}, {1, 0x00}, {-1, -1}},
	},
	{	//number of bp lines
		.data = {{0, 0xC8}, {1, 0x02}, {-1, -1}},
	},
	{	//gateset(2)
		.data = {{0, 0xC9}, {1, 0x10}, {-1, -1}},
	},
	{	//gateset(3)
		.data = {{0, 0xCA}, {1, 0x04}, {1, 0x04}, {-1, -1}},
	},
	{	//gateset(4)
		.data = {{0, 0xCB}, {1, 0x03}, {-1, -1}},
	},
	{	//gateset(5)
		.data = {{0, 0xCC}, {1, 0x12}, {-1, -1}},
	},
	{	//gateset(6)
		.data = {{0, 0xCD}, {1, 0x12}, {-1, -1}},
	},
	{	//gateset(7)
		.data = {{0, 0xCE}, {1, 0x30}, {-1, -1}},
	},
	{	//gateset(8)
		.data = {{0, 0xCF}, {1, 0x30}, {-1, -1}},
	},
	{	//gateset(9)
		.data = {{0, 0xD0}, {1, 0x40}, {-1, -1}},
	},
	{	//flhw
		.data = {{0, 0xD1}, {1, 0x22}, {-1, -1}},
	},
	{	//vckhw
		.data = {{0, 0xD2}, {1, 0x22}, {-1, -1}},
	},
	{	//flt
		.data = {{0, 0xD3}, {1, 0x04}, {-1, -1}},
	},
	{	//tctrl
		.data = {{0, 0xD4}, {1, 0x14}, {-1, -1}},
	},
	{	//dotinv
		.data = {{0, 0xD6}, {1, 0x02}, {-1, -1}},
	},
	{	//on/off sequence period
		.data = {{0, 0xD7}, {1, 0x00}, {-1, -1}},
	},
	{	//ponseqa
		.data = {{0, 0xD8}, {1, 0x01}, {1, 0x05}, {1, 0x06},
			 {1, 0x0D}, {1, 0x18}, {1, 0x09}, {1, 0x22},
			 {1, 0x23}, {1, 0x00}, {-1, -1}},
	},
	{	//ponseqb
		.data = {{0, 0xD9}, {1, 0x24}, {1, 0x01}, {-1, -1}},
	},
	{	//ponseqc
		.data = {{0, 0xDE}, {1, 0x09}, {1, 0x0F}, {1, 0x21},
			 {1, 0x12}, {1, 0x04}, {-1, -1}},
	},
	{	//pofseqa
		.data = {{0, 0xDF}, {1, 0x02}, {1, 0x06}, {1, 0x06},
			 {1, 0x06}, {1, 0x06}, {1, 0x00}, {-1, -1}},
	},
	{	//pofseqb
		.data = {{0, 0xE0}, {1, 0x01}, {-1, -1}},
	},
	/* {	//pofseqc
		.data = {{0, 0xE1}, {1, 0x00}, {1, 0x00}, {1, 0x00},
			 {1, 0x00}, {1, 0x00}, {-1, -1}},
	}, */
	{	//manual brightness
		.data = {{0, 0x51}, {1, 0xFF}, {-1, -1}},
	},
	{	//minimum brightness
		.data = {{0, 0x52}, {1, 0x00}, {-1, -1}},
	},
	{	//backlight control
		.data = {{0, 0x53}, {1, 0x40}, {-1, -1}},
	},
	{	//cabc pwm
		.data = {{0, 0xE2}, {1, 0x00}, {1, 0x00}, {-1, -1}},
	},
	{	//cabc
		.data = {{0, 0xE3}, {1, 0x03}, {-1, -1}},
	},
	{	 //cabc brightness
		.data = {{0, 0xE4}, {1, 0x66}, {1, 0x7B}, {1, 0x90},
			 {1, 0xA5}, {1, 0xBB}, {1, 0xC7}, {1, 0xE1},
			 {1, 0xE5}, {-1, -1}},
	},
	{	//cabc brightness
		.data = {{0, 0xE5}, {1, 0xC5}, {1, 0xC5}, {1, 0xC9},
			 {1, 0xC9}, {1, 0xD1}, {1, 0xE1}, {1, 0xF1},
			 {1, 0xFE}, {-1, -1}},
	},
	{	//cabc
		.data = {{0, 0xE7}, {1, 0x2A}, {-1, -1}},
	},
	{	//brt_rev
		.data = {{0, 0xE8}, {1, 0x00}, {-1, -1}},
	},
	{	//tefreq
		.data = {{0, 0xE9}, {1, 0x00}, {-1, -1}},
	},
	{	//high speed ram
		.data = {{0, 0xEA}, {1, 0x01}, {-1, -1}},
	},
// MOBII_S jg.noh 2012.06.27 request by HW
	{	//gamma setting r pos
		.data = {{0, 0xEB}, {1, 0x00}, {1, 0x33}, {1, 0x0E},
			 {1, 0x15}, {1, 0xB7}, {1, 0x78}, {1, 0x88},
			 {1, 0x0F}, {-1, -1}},
	},
	{	//gamma setting r neg
		.data = {{0, 0xEC}, {1, 0x00}, {1, 0x33}, {1, 0x0E},
			 {1, 0x15}, {1, 0xB7}, {1, 0x78}, {1, 0x88},
			 {1, 0x0F}, {-1, -1}},
	},
	{	//gamma setting g pos
		.data = {{0, 0xED}, {1, 0x00}, {1, 0x33}, {1, 0x0E},
			 {1, 0x15}, {1, 0xB7}, {1, 0x78}, {1, 0x88},
			 {1, 0x0F}, {-1, -1}},
	},
	{	//gamma setting g neg
		.data = {{0, 0xEE}, {1, 0x00}, {1, 0x33}, {1, 0x0E},
			 {1, 0x15}, {1, 0xB7}, {1, 0x78}, {1, 0x88},
			 {1, 0x0F}, {-1, -1}},
	},
	{	//gamma setting b pos
		.data = {{0, 0xEF}, {1, 0x00}, {1, 0x33}, {1, 0x0E},
			 {1, 0x15}, {1, 0xB7}, {1, 0x78}, {1, 0x88},
			 {1, 0x0F}, {-1, -1}},
	},
	{	//gamma setting b neg
		.data = {{0, 0xF0}, {1, 0x00}, {1, 0x33}, {1, 0x0E},
			 {1, 0x15}, {1, 0xB7}, {1, 0x78}, {1, 0x88},
			 {1, 0x0F}, {-1, -1}},
	},
// MOBII_E
	{	//exit_sleep_mode
		.data = {{0, 0x11}, {-1, -1}},
	},
	{	//Need >100ms delay.
		.data = {{-1, -1}},
		.delay = 110,
	},
	{	//set_display_on
		.data = {{0, 0x29}, {-1, -1}},
	},
#if defined(CONFIG_ONE_SHOT_MODE)
	{	//set_tear_on
		.data = {{0, 0x35}, {1, 0x00}, {-1, -1}},
	},
#endif
};

static struct hitachi_lcd_data_set hitachi_lcd_standby_data[] = {
#if defined(CONFIG_ONE_SHOT_MODE)
	{	//set_tear_off
		.data = {{0, 0x34}, {-1, -1}},
	},
#endif
	{	//disp_off
		.data = {{0, 0x28}, {-1, -1}},
	},
	{	//Need >35ms delay.
		.data = {{-1, -1}},
		.delay = 35,
	},
	{	//enter_sleep
		.data = {{0, 0x10}, {-1, -1}},
	},
	{	//Need >50ms delay.
		.data = {{-1, -1}},
		.delay = 50,
	},
};

static struct hitachi_lcd_data_set lgd_lcd_standby_data[]= {
	{
		.data = {{0, 0x28}, {-1, -1}},
	},
	{
		.data = {{0, 0x10}, {-1, -1}},
		.delay = 150,
	},
	{
		.data = {{0, 0xc1}, {1,0x01}, {-1, -1}},
		.delay = 10,
	},
};

static struct hitachi_lcd_data_set lgd_lcd_init_data[]= {
	{
		.data = {{0, 0x20}, {-1, -1}},
	},
	{
		.data = {{0, 0x35}, {-1, -1}},
	},
	{
		.data = {{0, 0x3a}, {1,0x77}, {-1, -1}},
	},
	{
		.data = {{0, 0xb2}, {1,0x00}, {1, 0xc8}, {-1, -1}},
	},
	{
		.data = {{0, 0xb3}, {1,0x00}, {-1, -1}},
	},
	{
		.data = {{0, 0xb4}, {1,0x04}, {-1, -1}},
	},
	{
		.data = {{0, 0xb5}, {1,0x42}, {1, 0x10}, {1, 0x10},
			{1, 0x00}, {1,0x20}, {-1, -1}},
	},
	{
		.data = {{0, 0xb6}, {1,0x0b}, {1, 0xf}, {1, 0x3c},
			{1, 0x13}, {1,0x13}, {1, 0xe8},{-1, -1}},
	},
	{
		.data = {{0, 0xb7}, {1,0x4c}, {1, 0x06}, {1, 0x0c},
			{1, 0x00}, {1,0x00}, {-1, -1}},
	},
	{
		.data = {{0, 0xc0}, {1,0x01}, {1, 0x11}, {-1, -1}},
	},
	{
		.data = {{0, 0xc3}, {1,0x07}, {1, 0x03}, {1, 0x04},
			{1, 0x04}, {1,0x04}, {-1, -1}},
	},
	{
		.data = {{0, 0xc4}, {1,0x12}, {1, 0x24}, {1, 0x18},
			{1, 0x18}, {1,0x02}, {1, 0x49},{-1, -1}},
	},
	{
		.data = {{0, 0xc5}, {1,0x65}, {-1, -1}},
	},
	{
		.data = {{0, 0xc6}, {1,0x41}, {1, 0x63}, {-1, -1}},
	},
	{
		.data = {{0, 0xd0}, {1,0x00}, {1, 0x46}, {1, 0x74},
			{1, 0x32}, {1,0x1d}, {1, 0x03}, {1, 0x51},
			{1, 0x15}, {1,0x04}, {-1, -1}},
	},
	{
		.data = {{0, 0xd1}, {1,0x00}, {1, 0x46}, {1, 0x74},
			{1, 0x32}, {1,0x1d}, {1, 0x03}, {1, 0x51},
			{1, 0x15}, {1,0x04}, {-1, -1}},
	},
	{
		.data = {{0, 0xd2}, {1,0x00}, {1, 0x46}, {1, 0x74},
			{1, 0x32}, {1,0x1F}, {1, 0x03}, {1, 0x51},
			{1, 0x15}, {1,0x04}, {-1, -1}},
	},
	{
		.data = {{0, 0xd3}, {1,0x00}, {1, 0x46}, {1, 0x74},
			{1, 0x32}, {1,0x1F}, {1, 0x03}, {1, 0x51},
			{1, 0x15}, {1,0x04}, {-1, -1}},
	},
	{
		.data = {{0, 0xd4}, {1,0x01}, {1, 0x46}, {1, 0x74},
			{1, 0x25}, {1,0x00}, {1, 0x03}, {1, 0x51},
			{1, 0x15}, {1,0x04}, {-1, -1}},
	},
	{
		.data = {{0, 0xd5}, {1,0x01}, {1, 0x46}, {1, 0x74},
			{1, 0x25}, {1,0x00}, {1, 0x03}, {1, 0x51},
			{1, 0x15}, {1,0x04}, {-1, -1}},
	},
	{
		.data = {{0, 0x2a}, {1,0x00}, {1, 0x00}, {1, 0x01},
			{1, 0xdf}, {-1, -1}},
	},
	{
		.data = {{0, 0x2b}, {1,0x00}, {1, 0x00}, {1, 0x03},
			{1, 0x1f}, {-1, -1}},
	},
	{
		.data = {{0, 0x11}, {-1, -1}},
		.delay = 120,
	},
	{
		.data = {{0, 0x2c}, {-1, -1}},
	},
	{
		.data = {{0, 0x29}, {-1, -1}},
	},
};


static struct regulator *reg_lcd_1v8 = NULL;
static struct regulator *reg_lcd_vdd = NULL;

/* GPIOs for AAT2870 backlight */
#define STAR_AAT2870_BL_EN	TEGRA_GPIO_PR3
#define STAR_AAT2870_BL_SDA	TEGRA_GPIO_PQ0
#define STAR_AAT2870_BL_SCL	TEGRA_GPIO_PQ1

/* AAT2870 Backlight */
static int aat2870_bl_init(struct backlight_device *bd)
{
	struct aat2870_bl_driver_data *drvdata = dev_get_drvdata(&bd->dev);

	tegra_gpio_enable(drvdata->en_pin);
	return 0;
}

static void aat2870_bl_uninit(struct backlight_device *bd)
{
	struct aat2870_bl_driver_data *drvdata = dev_get_drvdata(&bd->dev);

	tegra_gpio_disable(drvdata->en_pin);
}

static struct aat2870_bl_platform_data aat2870_bl_pdata = {
	.en_pin		= STAR_AAT2870_BL_EN,
	.avail_ch	= 0xFF, /* use all channels */
	.max_current	= AAT2870_BL_CURRENT_27_9,
	.max_brightness	= 255,
	.init		= aat2870_bl_init,
	.uninit		= aat2870_bl_uninit,
};

static struct i2c_board_info __initdata tegra_i2c_gpio_info[] = {
	 {
		I2C_BOARD_INFO(AAT2870_BL_DRV_NAME, 0x60),
		.platform_data = &aat2870_bl_pdata,
	 },
};

/* GPIO I2C */
static struct i2c_gpio_platform_data tegra_i2c_gpio_pdata = {
	.sda_pin = STAR_AAT2870_BL_SDA,
	.scl_pin = STAR_AAT2870_BL_SCL,
	.udelay = 5, /* (500 / udelay) kHz */
	.timeout = 100, /* jiffies */
};

static struct platform_device tegra_i2c_gpio_device = {
	.name	= "i2c-gpio",
	.id	= 5, /* tegra_i2c are bus 0 ~ 4, so start at 5 */
	.dev	= {
		.platform_data = &tegra_i2c_gpio_pdata,
	}
};

static inline int hitachi_lcd_write(struct hitachi_lcd_data_set *data_set,
				    int data_set_num)
{
	struct hitachi_lcd_data *data;
	int dc_en;
	int i, j, k;
	int ret = 0;

	for (i = 0; i < data_set_num; i++) {
		for (j = 0; j < HITACHI_LCD_MAX_DATA_NUM; j++) {
			data = &data_set[i].data[j];
			if (data->type == -1) {
				break;
			} else if (data->type == 0) {
				dc_en = 0;
			} else if (data->type == 1) {
				dc_en = 1;
			} else {
				pr_err("Invalied hitachi data type=%d\n", data->type);
				ret = -EINVAL;
				goto out;
			}

			gpio_set_value(STAR_HITACHI_LCD_DC, dc_en);
			gpio_set_value(STAR_HITACHI_LCD_CS, 0);
			gpio_set_value(STAR_HITACHI_LCD_WR, 0);

			for (k = 0; k < 8; k++)
				gpio_set_value(STAR_HITACHI_DATA[k],
					       (data->value >> k) & 0x1);

			gpio_set_value(STAR_HITACHI_LCD_WR, 1);
			gpio_set_value(STAR_HITACHI_LCD_CS, 1);
			
			udelay(10);
		}

		if (data_set[i].delay > 0)
			mdelay(data_set[i].delay);
	}

out:
	return ret;
}
#if defined(CONFIG_MACH_STAR)
int g_is_suspend=0;
EXPORT_SYMBOL(g_is_suspend);
#endif

static int star_panel_postpoweron(void)
{
	int i;
#if 0 //MBksjung not used

	if(bAndroidBootMode)
	{		
		printk("\n AndroidBootMode Now (return star_panel_postpoweron)...... \n");
		return 0;
	}
#endif
	if(is_lcd_panel_poweron) // already power on
	{
		printk("already power on\n");
		return 0;
	}

	printk("\n star_panel_postpoweron...... \n");
#if defined(CONFIG_MACH_STAR)
				g_is_suspend=1;
#endif

	gpio_set_value(STAR_HITACHI_LCD_RESET, 0);

	if(MARKED_ID /*gpio_get_value(STAR_LCD_MARKED_ID)*/)
	{		
		printk("\n star LGD star_panel_postpoweron...... \n");
// ???
		gpio_set_value(STAR_HITACHI_LCD_CS, 0);
		gpio_set_value(STAR_HITACHI_LCD_CS, 1);
		mdelay(1);
//
		if (!reg_lcd_1v8) {
			reg_lcd_1v8 = regulator_get_exclusive(NULL, "vcc_lcd_1v8");
			if (IS_ERR_OR_NULL(reg_lcd_1v8)) {
				pr_err("lcd: couldn't get regulator vddio_mipi\n");
			}
			regulator_set_voltage(reg_lcd_1v8, 1800000, 1800000);
		}

		if(!regulator_is_enabled(reg_lcd_1v8))
			regulator_enable(reg_lcd_1v8);

		if (!reg_lcd_vdd) {
		    reg_lcd_vdd = regulator_get_exclusive(NULL, "vcc_lcd_2v8");
		    if (IS_ERR(reg_lcd_vdd)) {
		        pr_err("Failed to get vcc_lcd_2v8\n");
	    	}
			regulator_set_voltage(reg_lcd_vdd, 3100000, 3100000);
	    }

		if(!regulator_is_enabled(reg_lcd_vdd))
			regulator_enable(reg_lcd_vdd);

		gpio_set_value(STAR_HITACHI_LCD_RESET, 1);

		mdelay(10);

		gpio_direction_output(STAR_HITACHI_LCD_CS, 1);
		tegra_gpio_enable(STAR_HITACHI_LCD_CS);

		gpio_direction_output(STAR_HITACHI_LCD_WR, 1);
		tegra_gpio_enable(STAR_HITACHI_LCD_WR);

		gpio_direction_output(STAR_HITACHI_LCD_DC, 0);
		tegra_gpio_enable(STAR_HITACHI_LCD_DC);

		hitachi_lcd_write(lgd_lcd_init_data,
					ARRAY_SIZE(lgd_lcd_init_data));
	}
	else
	{	
		printk("\n star Hitach star_panel_postpoweron...... \n");

		if (!reg_lcd_1v8) {
			reg_lcd_1v8 = regulator_get_exclusive(NULL, "vcc_lcd_1v8");
			if (IS_ERR_OR_NULL(reg_lcd_1v8)) {
				pr_err("lcd: couldn't get regulator vddio_mipi\n");
			}
			regulator_set_voltage(reg_lcd_1v8, 1800000, 1800000);
		}

		if(!regulator_is_enabled(reg_lcd_1v8))
			regulator_enable(reg_lcd_1v8);

		if (!reg_lcd_vdd) {
		    reg_lcd_vdd = regulator_get_exclusive(NULL, "vcc_lcd_2v8");
		    if (IS_ERR(reg_lcd_vdd)) {
		        pr_err("Failed to get vcc_lcd_2v8\n");
	    	}
			regulator_set_voltage(reg_lcd_vdd, 2800000, 2800000);
	    }

		if(!regulator_is_enabled(reg_lcd_vdd))
			regulator_enable(reg_lcd_vdd);

		mdelay(25);
		gpio_set_value(STAR_HITACHI_LCD_RESET, 1);
		mdelay(5);

		gpio_direction_output(STAR_HITACHI_LCD_CS, 1);
		tegra_gpio_enable(STAR_HITACHI_LCD_CS);

		gpio_direction_output(STAR_HITACHI_LCD_WR, 1);
		tegra_gpio_enable(STAR_HITACHI_LCD_WR);

		gpio_direction_output(STAR_HITACHI_LCD_DC, 0);
		tegra_gpio_enable(STAR_HITACHI_LCD_DC);

		for (i = 0; i < 8; i++) {
			gpio_direction_output(STAR_HITACHI_DATA[i], 1);
			tegra_gpio_enable(STAR_HITACHI_DATA[i]);
		}

		hitachi_lcd_write(hitachi_lcd_init_data,
					ARRAY_SIZE(hitachi_lcd_init_data));
	}

	tegra_gpio_disable(STAR_HITACHI_LCD_DC);
	tegra_gpio_disable(STAR_HITACHI_LCD_WR);
	
	for (i = 0; i < 8; i++)
		tegra_gpio_disable(STAR_HITACHI_DATA[i]);
	
	gpio_set_value(STAR_HITACHI_LCD_CS, 0);

	is_lcd_panel_poweroff = 0;
	is_lcd_panel_poweron = 1;

	return 0;
}

static int star_panel_postsuspend(void)
{
	int i;
#if 0 //MBksjung not used

	if(bAndroidBootMode)
	{		
		printk("\n AndroidBootMode Now (return star_panel_postsuspend)...... \n");
		return 0;
	}
#endif
	if(is_lcd_panel_poweroff)	//already power off
	{
		printk("already power off.\n");
		return 0;
	}

	printk("\n star_panel_postsuspend...... \n");

	gpio_direction_output(STAR_HITACHI_LCD_CS, 1);
	tegra_gpio_enable(STAR_HITACHI_LCD_CS);
	
	gpio_direction_output(STAR_HITACHI_LCD_WR, 1);
	tegra_gpio_enable(STAR_HITACHI_LCD_WR);
	
	gpio_direction_output(STAR_HITACHI_LCD_DC, 0);
	tegra_gpio_enable(STAR_HITACHI_LCD_DC);
	
	for (i = 0; i < 8; i++) {
		gpio_direction_output(STAR_HITACHI_DATA[i], 1);
		tegra_gpio_enable(STAR_HITACHI_DATA[i]);
	}

	if(MARKED_ID /*gpio_get_value(STAR_LCD_MARKED_ID)*/)
	{
		printk("\n star LGD panel_postsuspend...... \n");
		hitachi_lcd_write( lgd_lcd_standby_data,
				ARRAY_SIZE(lgd_lcd_standby_data));
	}
	else
	{
		printk("\n star Hitachi panel_postsuspend...... \n");		

		hitachi_lcd_write(hitachi_lcd_standby_data,
				ARRAY_SIZE(hitachi_lcd_standby_data));
	}

//	gpio_set_value(STAR_HITACHI_LCD_CS, 1);
//	gpio_set_value(STAR_HITACHI_LCD_WR, 1);
//	gpio_set_value(STAR_HITACHI_LCD_DC, 1);

	// pull reset to low
	gpio_set_value(STAR_HITACHI_LCD_RESET, 0);

	if (!reg_lcd_1v8) {
		reg_lcd_1v8 = regulator_get_exclusive(NULL, "vcc_lcd_1v8");
		if (IS_ERR_OR_NULL(reg_lcd_1v8)) {
			pr_err("lcd: couldn't get regulator vddio_mipi\n");
		}
		regulator_set_voltage(reg_lcd_1v8, 1800000, 1800000);
	}

	if(regulator_is_enabled(reg_lcd_1v8))
		regulator_disable(reg_lcd_1v8);

/*
    if (!reg_lcd_vdd) {
	    reg_lcd_vdd = regulator_get_exclusive(NULL, "vcc_lcd_2v8");
	    if (IS_ERR(reg_lcd_vdd)) {
	        pr_err("Failed to get vcc_lcd_2v8\n");
    	}
		if(MARKED_ID)
			regulator_set_voltage(reg_lcd_vdd, 3100000, 3100000);
    	else
			regulator_set_voltage(reg_lcd_vdd, 2800000, 2800000);
    }

	if(regulator_is_enabled(reg_lcd_vdd))
		regulator_disable(reg_lcd_vdd);
*/
	mdelay(50);

	is_lcd_panel_poweroff = 1;
	is_lcd_panel_poweron = 0;

	return 0;
}

#ifdef CONFIG_TEGRA_DC
static int star_hdmi_enable(void)
{
// LGE_CHANGE_S [dojip.kim@lge.com] 2011-01-13, [LGE_AP20] hdmi
// MOBII_CHANGE [jg.noh@mobii.co.kr] 2012-03-01 move to tegra_dc_enable()
//	gpio_set_value(star_hdmi_reg_en, 1);
// LGE_CHANGE_E [dojip.kim@lge.com] 2011-01-13, [LGE_AP20] hdmi

	if (!star_hdmi_reg) {
		star_hdmi_reg = regulator_get(NULL, "avdd_hdmi"); /* LD011 */
		if (IS_ERR_OR_NULL(star_hdmi_reg)) {
			pr_err("hdmi: couldn't get regulator avdd_hdmi\n");
			star_hdmi_reg = NULL;
			return PTR_ERR(star_hdmi_reg);
		}
	}
	regulator_enable(star_hdmi_reg);

	if (!star_hdmi_pll) {
		star_hdmi_pll = regulator_get(NULL, "avdd_hdmi_pll"); /* LD06 */
		if (IS_ERR_OR_NULL(star_hdmi_pll)) {
			pr_err("hdmi: couldn't get regulator avdd_hdmi_pll\n");
			star_hdmi_pll = NULL;
			regulator_disable(star_hdmi_reg);
			star_hdmi_reg = NULL;
			return PTR_ERR(star_hdmi_pll);
		}
	}
	regulator_enable(star_hdmi_pll);
	return 0;
}

static int star_hdmi_disable(void)
{
// LGE_CHANGE_S [dojip.kim@lge.com] 2011-01-13, [LGE_AP20] hdmi
//MOBII_CHNANGE_S 20120716 hskim@mobii.co.kr : HDMI detect error
        //gpio_set_value(TEGRA_GPIO_HDMI_EN, 0);
// LGE_CHANGE_E [dojip.kim@lge.com] 2011-01-13, [LGE_AP20] hdmi
	regulator_disable(star_hdmi_reg);
	regulator_disable(star_hdmi_pll);
	return 0;
}

/* Framebuffer */
static struct resource star_disp1_resources[] = {
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
		.flags	= IORESOURCE_MEM,
	},
};

static struct resource star_disp2_resources[] = {
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
	},
	{
		.name	= "hdmi_regs",
		.start	= TEGRA_HDMI_BASE,
		.end	= TEGRA_HDMI_BASE + TEGRA_HDMI_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct tegra_dc_mode star_panel_modes[] = {
	{
// LGE_CHANGE_S [youngseok.jeong@lge.com] 2011-02-15 [LGE_AP20] from nvodm
		.pclk = 29816000,
// LGE_CHANGE_E [youngseok.jeong@lge.com] 2011-02-15 [LGE_AP20] from nvodm
		.h_ref_to_sync = 4,
		.v_ref_to_sync = 1,
		.h_sync_width = 10,
		.v_sync_width = 4,
		.h_back_porch = 10,
		.v_back_porch = 4,
		.h_active = 480,
		.v_active = 800,
		.h_front_porch = 10,
		.v_front_porch = 4,
	}
};

static struct tegra_dc_out_pin star_dc_out_pins[] = {
	{
		.name = TEGRA_DC_OUT_PIN_PIXEL_CLOCK,
		.pol = TEGRA_DC_OUT_PIN_POL_LOW,
	},
};

static u8 star_dc_out_pin_sel_config[] = {
	TEGRA_PIN_OUT_CONFIG_SEL_LM1_M1,
};

static struct tegra_dc_out star_disp1_out = {
	.type 		= TEGRA_DC_OUT_CPU,
	.flags		= TEGRA_DC_OUT_ONE_SHOT_MODE,

	.align		= TEGRA_DC_ALIGN_LSB,
	.order		= TEGRA_DC_ORDER_RED_BLUE,

	.depth = 24,	/* BASE_COLOR_SIZE888 */

	.modes = star_panel_modes,
	.n_modes = ARRAY_SIZE(star_panel_modes),

	.out_pins = star_dc_out_pins,
	.n_out_pins = ARRAY_SIZE(star_dc_out_pins),

	.dither = TEGRA_DC_ERRDIFF_DITHER,
	//.out_sel_configs   = star_dc_out_pin_sel_config,
	//.n_out_sel_configs = ARRAY_SIZE(star_dc_out_pin_sel_config),

	.postsuspend = star_panel_postsuspend,
	.postpoweron = star_panel_postpoweron,
};

static struct tegra_dc_out star_disp2_out = {
	.type		= TEGRA_DC_OUT_HDMI,
	.flags		= TEGRA_DC_OUT_HOTPLUG_HIGH,

	.dcc_bus	= 1,
	.hotplug_gpio	= TEGRA_GPIO_HDMI_HPD,

	.max_pixclock	= KHZ2PICOS(148500),

	.align		= TEGRA_DC_ALIGN_MSB,
	.order		= TEGRA_DC_ORDER_RED_BLUE,

	.enable		= star_hdmi_enable,
	.disable	= star_hdmi_disable,
};

static struct tegra_fb_data star_fb_data = {
	.win		= 0,
	.xres		= 480,
	.yres		= 800,
// LGE_CHANGE_S [beobki.chung@lge.com] 2012-01-20 [LGE_AP20]
	.bits_per_pixel	= 32, // modification for flicking issue
// LGE_CHANGE_E [beobki.chung@lge.com] 2012-01-20 [LGE_AP20]
	.flags		= TEGRA_FB_FLIP_ON_PROBE,
};

static struct tegra_fb_data star_hdmi_fb_data = {
#ifdef CONFIG_MACH_STAR
	.win		= 0,
	.xres		= 1366,
	.yres		= 768,
	.bits_per_pixel	= 16,
#else
	.win		= 0,
	.xres		= 800,
	.yres		= 480,
	.bits_per_pixel	= 32,
#endif
	.flags		= TEGRA_FB_FLIP_ON_PROBE,
};

static struct tegra_dc_platform_data star_disp1_pdata = {
	.flags		= TEGRA_DC_FLAG_ENABLED,
	.default_out	= &star_disp1_out,
	.fb		= &star_fb_data,
};

static struct nvhost_device star_disp1_device = {
	.name		= "tegradc",
	.id		= 0,
	.resource	= star_disp1_resources,
	.num_resources	= ARRAY_SIZE(star_disp1_resources),
	.dev = {
		.platform_data = &star_disp1_pdata,
	},
};

static struct tegra_dc_platform_data star_disp2_pdata = {
	.flags		= 0,
	.default_out	= &star_disp2_out,
	.fb		= &star_hdmi_fb_data,
};

static struct nvhost_device star_disp2_device = {
	.name		= "tegradc",
	.id		= 1,
	.resource	= star_disp2_resources,
	.num_resources	= ARRAY_SIZE(star_disp2_resources),
	.dev = {
		.platform_data = &star_disp2_pdata,
	},
};
#endif

static struct nvmap_platform_carveout star_carveouts[] = {
	[0] = NVMAP_HEAP_CARVEOUT_IRAM_INIT,
	[1] = {
		.name		= "generic-0",
		.usage_mask	= NVMAP_HEAP_CARVEOUT_GENERIC,
             .base       = 0,    /* Filled in by star_panel_init() */
             .size       = 0,    /* Filled in by star_panel_init() */
		.buddy_size	= SZ_32K,
	},
};

static struct nvmap_platform_data star_nvmap_data = {
	.carveouts	= star_carveouts,
	.nr_carveouts	= ARRAY_SIZE(star_carveouts),
};

static struct platform_device star_nvmap_device = {
	.name	= "tegra-nvmap",
	.id	= -1,
	.dev	= {
		.platform_data = &star_nvmap_data,
	},
};

static struct platform_device *star_gfx_devices[] __initdata = {
	&star_nvmap_device,
#ifdef CONFIG_TEGRA_GRHOST
	&tegra_grhost_device,
#endif
	&tegra_i2c_gpio_device,
};

#ifdef CONFIG_HAS_EARLYSUSPEND
/* put early_suspend/late_resume handlers here for the display in order
 * to keep the code out of the display driver, keeping it closer to upstream
 */
struct early_suspend star_panel_early_suspender;

static void star_panel_early_suspend(struct early_suspend *h)
{
	/* power down LCD, add use a black screen for HDMI */
	if (num_registered_fb > 0)
			fb_blank(registered_fb[0], FB_BLANK_POWERDOWN);
	if (num_registered_fb > 1)
			fb_blank(registered_fb[1], FB_BLANK_NORMAL);
	
#ifdef CONFIG_TEGRA_CONVSERVATIVE_GOV_ON_EARLYSUPSEND
	cpufreq_save_default_governor();
	cpufreq_set_conservative_governor();
        cpufreq_set_conservative_governor_param("up_threshold",
			SET_CONSERVATIVE_GOVERNOR_UP_THRESHOLD);

	cpufreq_set_conservative_governor_param("down_threshold",
			SET_CONSERVATIVE_GOVERNOR_DOWN_THRESHOLD);

	cpufreq_set_conservative_governor_param("freq_step",
		SET_CONSERVATIVE_GOVERNOR_FREQ_STEP);
#endif
#ifdef CONFIG_TEGRA_AUTO_HOTPLUG
// hyunk        	tegra2_enable_autoplug();
#endif
}

static void star_panel_late_resume(struct early_suspend *h)
{
	unsigned i;

#ifdef CONFIG_TEGRA_AUTO_HOTPLUG
// hyunk        	tegra2_disable_autoplug();
#endif

#ifdef CONFIG_TEGRA_CONVSERVATIVE_GOV_ON_EARLYSUPSEND
	cpufreq_restore_default_governor();
#endif
	for (i = 0; i < num_registered_fb; i++)
		fb_blank(registered_fb[i], FB_BLANK_UNBLANK);
}
#endif

static int __init star_android_boot_mode(char *arg)
{
	if(!strcmp("charger", arg))
		bAndroidBootMode = 1;
	else
		bAndroidBootMode = 0;
}

early_param("androidboot.mode", star_android_boot_mode);

int __init star_panel_init(void)
{
	int err;
	struct resource __maybe_unused *res;
	
//HDMI GPIO INIT
	gpio_request(TEGRA_GPIO_HDMI_HPD, "hdmi_hpd");
	gpio_direction_input(TEGRA_GPIO_HDMI_HPD);
	tegra_gpio_enable(TEGRA_GPIO_HDMI_HPD);

	// LGE_CHANGE_S [dojip.kim@lge.com] 2011-01-13, [LGE_AP20] hdmi
	gpio_request(TEGRA_GPIO_HDMI_EN, "hdmi_reg_en");
//MOBII_CHNANGE_S 20120716 hskim@mobii.co.kr : HDMI detect error
        gpio_direction_output(TEGRA_GPIO_HDMI_EN, 1);
	//gpio_direction_output(TEGRA_GPIO_HDMI_EN, 0);
//MOBII_CHNANGE_E 20120716 hskim@mobii.co.kr : HDMI detect error
	tegra_gpio_enable(TEGRA_GPIO_HDMI_EN);
	// LGE_CHANGE_E [dojip.kim@lge.com] 2011-01-13, [LGE_AP20] hdmi

//LCD GPIO INIT
	gpio_request(STAR_HITACHI_LCD_RESET, "lcd_rst");
	gpio_direction_output(STAR_HITACHI_LCD_RESET, 1);
	tegra_gpio_enable(STAR_HITACHI_LCD_RESET);

	gpio_request(STAR_HITACHI_LCD_CS, "lcd_cs");
	gpio_direction_output(STAR_HITACHI_LCD_CS, 0);
	tegra_gpio_enable(STAR_HITACHI_LCD_CS);

	gpio_request(STAR_HITACHI_LCD_DC, "lcd_dc");
	gpio_request(STAR_HITACHI_LCD_WR, "lcd_wr");

	gpio_request(STAR_HITACHI_LCD_D0, "lcd_data_0");
	gpio_request(STAR_HITACHI_LCD_D1, "lcd_data_1");
	gpio_request(STAR_HITACHI_LCD_D2, "lcd_data_2");
	gpio_request(STAR_HITACHI_LCD_D3, "lcd_data_3");
	gpio_request(STAR_HITACHI_LCD_D4, "lcd_data_4");
	gpio_request(STAR_HITACHI_LCD_D5, "lcd_data_5");
	gpio_request(STAR_HITACHI_LCD_D6, "lcd_data_6");
	gpio_request(STAR_HITACHI_LCD_D7, "lcd_data_7");

	gpio_request(STAR_LCD_MARKED_ID, "lcd_mark_id");
	gpio_direction_input(STAR_LCD_MARKED_ID);
	tegra_gpio_enable(STAR_LCD_MARKED_ID);

	// We assume LCD 1.8V is turned on in bootloader. Because lcd marked pin is ext pull-down.
	MARKED_ID  = gpio_get_value(STAR_LCD_MARKED_ID);

#ifdef CONFIG_HAS_EARLYSUSPEND
	star_panel_early_suspender.suspend = star_panel_early_suspend;
	star_panel_early_suspender.resume = star_panel_late_resume;
	star_panel_early_suspender.level = EARLY_SUSPEND_LEVEL_DISABLE_FB;
	register_early_suspend(&star_panel_early_suspender);
#endif

	star_carveouts[1].base = tegra_carveout_start;
	star_carveouts[1].size = tegra_carveout_size;

	err = platform_add_devices(star_gfx_devices,
				   ARRAY_SIZE(star_gfx_devices));

	/* GPIO I2C pin init */
	tegra_gpio_enable(STAR_AAT2870_BL_SDA);
	tegra_gpio_enable(STAR_AAT2870_BL_SCL);

	err = i2c_register_board_info(5, tegra_i2c_gpio_info,
				      ARRAY_SIZE(tegra_i2c_gpio_info));

#if defined(CONFIG_TEGRA_GRHOST) && defined(CONFIG_TEGRA_DC)
	res = nvhost_get_resource_byname(&star_disp1_device,
					 IORESOURCE_MEM, "fbmem");
	res->start = tegra_fb_start;
	res->end = tegra_fb_start + tegra_fb_size - 1;
#endif

	/* Copy the bootloader fb to the fb. */
	tegra_move_framebuffer(tegra_fb_start, tegra_bootloader_fb_start,
		min(tegra_fb_size, tegra_bootloader_fb_size));

#if defined(CONFIG_TEGRA_GRHOST) && defined(CONFIG_TEGRA_DC)
	res = nvhost_get_resource_byname(&star_disp2_device,
					 IORESOURCE_MEM, "fbmem");
	res->start = tegra_fb2_start;
	res->end = tegra_fb2_start + tegra_fb2_size - 1;

	if (!err)
		err = nvhost_device_register(&star_disp1_device);

	if (!err)
		err = nvhost_device_register(&star_disp2_device);
#endif

	return err;
}

