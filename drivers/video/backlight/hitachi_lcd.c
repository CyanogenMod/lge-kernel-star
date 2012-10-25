/*
 * linux/drivers/video/backlight/hitachi_lcd.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/lcd.h>
#include <linux/fb.h>
#include <linux/gpio.h>
#include <linux/hitachi.h>

// LGE_CHANGE_S [youngseok.jeong@lge.com] 2011-01-07 enable LCD power
#include <linux/regulator/consumer.h>
// LGE_CHANGE_E [youngseok.jeong@lge.com] 2011-01-07 enable LCD power

/*
 * Debug Level
 *	2 : Print all debug messages
 *	1 : Print only dbg() messages
 *	0 : No debug messages
 */
#define HITACHI_LCD_DEBUG_LEVEL		0

#if (HITACHI_LCD_DEBUG_LEVEL == 2)
#define dbg(format, arg...) \
	printk(KERN_ALERT HITACHI_LCD_DRV_NAME \
	       ": Debug: %s(): " format, __func__, ## arg)
#define enter() \
	printk(KERN_ALERT HITACHI_LCD_DRV_NAME ": Enter: %s()\n", __func__)
#define leave() \
	printk(KERN_ALERT HITACHI_LCD_DRV_NAME ": Leave: %s()\n", __func__)
#elif (HITACHI_LCD_DEBUG_LEVEL == 1)
#define dbg(format, arg...) \
	printk(KERN_ALERT HITACHI_LCD_DRV_NAME \
	       ": Debug: %s(): " format, __func__, ## arg)
#define enter()
#define leave()
#else
#define dbg(format, arg...)
#define enter()
#define leave()
#endif

#define CONFIG_ONE_SHOT_MODE	1

enum hitachi_lcd_mode {	
	HITACHI_LCD_DISPLAY_ON,
	HITACHI_LCD_DISPLAY_OFF,
};

struct hitachi_lcd_drvdata {
	struct platform_device *pdev;
	struct lcd_device *ld;

	int reset_pin;
	int cs_pin;
	int dc_pin;
	int wr_pin;
	int data_pin[8];

#if defined (CONFIG_MACH_STAR)
	int maker_id_pin;
#endif // MOBII_CHANG_S [jg.noh@mobii.co.kr] 2012-01-31

	enum hitachi_lcd_mode mode; /* current lcd mode */
	int power; /* current power status */
	int pin_ctrl;
};

#define HITACHI_LCD_MAX_DATA_NUM	32

struct hitachi_lcd_data {
	int type; /* -1=end of data, 0=command, 1=data */
	int value;
};

struct hitachi_lcd_data_set {
	struct hitachi_lcd_data data[HITACHI_LCD_MAX_DATA_NUM];
	int delay; /* msec */
};

static int hitachi_lcd_power_on(struct lcd_device *ld);
static int hitachi_lcd_power_off(struct lcd_device *ld);
static int hitachi_lcd_display_on(struct lcd_device *ld);
static int hitachi_lcd_display_off(struct lcd_device *ld);

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
	{	//gamma setting r pos
		.data = {{0, 0xEB}, {1, 0x10}, {1, 0x33}, {1, 0x08},
			 {1, 0x15}, {1, 0x35}, {1, 0x57}, {1, 0x77},
			 {1, 0x0F}, {-1, -1}},
	},
	{	//gamma setting r neg
		.data = {{0, 0xEC}, {1, 0x10}, {1, 0x33}, {1, 0x08},
			 {1, 0x15}, {1, 0x35}, {1, 0x57}, {1, 0x77},
			 {1, 0x0F}, {-1, -1}},
	},
	{	//gamma setting g pos
		.data = {{0, 0xED}, {1, 0x08}, {1, 0x33}, {1, 0x08},
			 {1, 0x15}, {1, 0x35}, {1, 0x57}, {1, 0x77},
			 {1, 0x0F}, {-1, -1}},
	},
	{	//gamma setting g neg
		.data = {{0, 0xEE}, {1, 0x08}, {1, 0x33}, {1, 0x08},
			 {1, 0x15}, {1, 0x35}, {1, 0x57}, {1, 0x77},
			 {1, 0x0F}, {-1, -1}},
	},
	{	//gamma setting b pos
		.data = {{0, 0xEF}, {1, 0x00}, {1, 0x33}, {1, 0x08},
			 {1, 0x15}, {1, 0x35}, {1, 0x57}, {1, 0x77},
			 {1, 0x0F}, {-1, -1}},
	},
	{	//gamma setting b neg
		.data = {{0, 0xF0}, {1, 0x00}, {1, 0x33}, {1, 0x08},
			 {1, 0x15}, {1, 0x35}, {1, 0x57}, {1, 0x77},
			 {1, 0x0F}, {-1, -1}},
	},
	{	//exit_sleep_mode
		.data = {{0, 0x11}, {-1, -1}},
	},
	{	//Need >100ms delay.
		.data = {{-1, -1}},
		.delay = 100,
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
	{
		.data = {{0, 0x28}, {-1, -1}},
	},
	{
		.data = {{0, 0x70}, {1,0x01}, {-1, -1}},
	},
};

#if defined (CONFIG_MACH_STAR)
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
#endif /* MOBII_CHANG_S [jg.noh@mobii.co.kr] 2012-01-31 Supporting dual lcd driver that including both hitachi & LGD pannels. */

static inline int hitachi_lcd_write(struct lcd_device *ld,
				    struct hitachi_lcd_data_set *data_set,
				    int data_set_num)
{
	struct hitachi_lcd_drvdata *drvdata = dev_get_drvdata(&ld->dev);
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
				dev_err(&ld->dev,
					"Invalied hitachi data type=%d\n",
					data->type);
				ret = -EINVAL;
				goto out;
			}

			gpio_set_value(drvdata->dc_pin, dc_en);
			gpio_set_value(drvdata->cs_pin, 0);
			gpio_set_value(drvdata->wr_pin, 0);

			for (k = 0; k < 8; k++)
				gpio_set_value(drvdata->data_pin[k],
// LGE_CHANGE_S [youngseok.jeong@lge.com] 2011-01-21 [LGE_AP20] bug fix
					       /* original code
					       (data->value >> k) && 0x1);
						*/
					       (data->value >> k) & 0x1);
// LGE_CHANGE_E [youngseok.jeong@lge.com] 2011-01-21 [LGE_AP20] bug fix

			gpio_set_value(drvdata->wr_pin, 1);
			gpio_set_value(drvdata->cs_pin, 1);
			udelay(250);
		}

		if (data_set[i].delay > 0)
			mdelay(data_set[i].delay);
	}

out:
	return ret;
}

static void hitachi_lcd_get_pin_ctrl(struct lcd_device *ld)
{
	struct hitachi_lcd_drvdata *drvdata = dev_get_drvdata(&ld->dev);
	int i;

	dbg("[hitachi_lcd_get_pin_ctrl] pin_ctrl=%d\n", drvdata->pin_ctrl);

	if (drvdata->pin_ctrl)
		return;

	gpio_direction_output(drvdata->cs_pin, 1);
	tegra_gpio_enable(drvdata->cs_pin);

	gpio_direction_output(drvdata->wr_pin, 1);
	tegra_gpio_enable(drvdata->wr_pin);

	gpio_direction_output(drvdata->dc_pin, 0);
	tegra_gpio_enable(drvdata->dc_pin);

	for (i = 0; i < 8; i++) {
		gpio_direction_output(drvdata->data_pin[i], 1);
		tegra_gpio_enable(drvdata->data_pin[i]);
	}

	drvdata->pin_ctrl = 1;
}

static void hitachi_lcd_put_pin_ctrl(struct lcd_device *ld)
{
	struct hitachi_lcd_drvdata *drvdata = dev_get_drvdata(&ld->dev);
	int i;

	dbg("[hitachi_lcd_put_pin_ctrl] pin_ctrl=%d\n", drvdata->pin_ctrl);

	if (!drvdata->pin_ctrl)
		return;

	tegra_gpio_disable(drvdata->dc_pin);
	tegra_gpio_disable(drvdata->wr_pin);

	for (i = 0; i < 8; i++)
		tegra_gpio_disable(drvdata->data_pin[i]);

	gpio_set_value(drvdata->cs_pin, 0);
	drvdata->pin_ctrl = 0;
}

// LGE_CHANGE_S [youngseok.jeong@lge.com] 2011-01-07 enable LCD power
static struct regulator *reg_lcd_1v8 = NULL;
static struct regulator *reg_lcd_2v8 = NULL;
// LGE_CHANGE_E [youngseok.jeong@lge.com] 2011-01-07 enable LCD power

static int hitachi_lcd_power_on(struct lcd_device *ld)
{
	struct hitachi_lcd_drvdata *drvdata = dev_get_drvdata(&ld->dev);
	int ret = 0;

	dbg("hitachi_lcd_power_on\n");

	gpio_set_value(drvdata->reset_pin, 1);
	mdelay(20);

	// LGE_CHANGE_S [youngseok.jeong@lge.com] 2011-01-20 LCD initialize
	gpio_set_value(drvdata->cs_pin, 1);
	gpio_set_value(drvdata->wr_pin, 1);
	gpio_set_value(drvdata->dc_pin, 0);
	// LGE_CHANGE_E [youngseok.jeong@lge.com] 2011-01-20 LCD initialize

	return 0;
}

static int hitachi_lcd_power_off(struct lcd_device *ld)
{
	struct hitachi_lcd_drvdata *drvdata = dev_get_drvdata(&ld->dev);
	int ret = 0;

	dbg("hitachi_lcd_power_off\n");

	hitachi_lcd_get_pin_ctrl(ld);

	if(gpio_get_value(drvdata->maker_id_pin)) 
	{
		ret = hitachi_lcd_write(ld, lgd_lcd_standby_data,
					ARRAY_SIZE(lgd_lcd_standby_data));
	}
	else
	{
		ret = hitachi_lcd_write(ld, hitachi_lcd_standby_data,
					ARRAY_SIZE(hitachi_lcd_standby_data));
	}

	gpio_set_value(drvdata->reset_pin,0);
	return ret;
}

static int hitachi_lcd_display_on(struct lcd_device *ld)
{
	struct hitachi_lcd_drvdata *drvdata = dev_get_drvdata(&ld->dev);
	int ret = 0;

	dbg("mode=%d->%d\n", drvdata->mode, HITACHI_LCD_DISPLAY_ON);

	if (drvdata->mode == HITACHI_LCD_DISPLAY_ON) {
		dev_warn(&ld->dev, "Already display on\n");
		goto out;
	}

	if(gpio_get_value(drvdata->maker_id_pin)) 
	{
		gpio_set_value(drvdata->cs_pin, 0);
		gpio_set_value(drvdata->cs_pin, 1);

		mdelay(1);

		gpio_set_value(drvdata->reset_pin, 1);

		mdelay(10);

		gpio_set_value(drvdata->cs_pin, 1);
		gpio_set_value(drvdata->wr_pin, 1);
		gpio_set_value(drvdata->dc_pin, 0);

		ret = hitachi_lcd_write(ld, lgd_lcd_init_data,
					ARRAY_SIZE(lgd_lcd_init_data));
		
		drvdata->pin_ctrl = 1;
	}
	else
	{
		ret = hitachi_lcd_power_on(ld);
		if (ret < 0) {
			dev_err(&ld->dev, "Failed to panel power on\n");
			goto out;
		}

		hitachi_lcd_get_pin_ctrl(ld);

		ret = hitachi_lcd_write(ld, hitachi_lcd_init_data,
				ARRAY_SIZE(hitachi_lcd_init_data));
	}

	if (ret < 0) {
		dev_err(&ld->dev, "Failed to panel initialize\n");
		goto out_put_pin_ctrl;
	}

	drvdata->mode = HITACHI_LCD_DISPLAY_ON;
	dbg("Panel is HITACHI_LCD_DISPLAY_ON (%d)\n", ret);

out_put_pin_ctrl:
	hitachi_lcd_put_pin_ctrl(ld);
out:
	return ret;
}

static int hitachi_lcd_display_off(struct lcd_device *ld)
{
	struct hitachi_lcd_drvdata *drvdata = dev_get_drvdata(&ld->dev);
	int ret = 0;

	dbg("mode=%d->%d\n", drvdata->mode, HITACHI_LCD_DISPLAY_OFF);

	if (drvdata->mode == HITACHI_LCD_DISPLAY_OFF) {
		dev_warn(&ld->dev, "Already display off\n");
		goto out;
	}

	ret = hitachi_lcd_power_off(ld);
	if (ret < 0) {
		dev_err(&ld->dev, "Failed to panel power off\n");
		goto out;
	}

	drvdata->mode = HITACHI_LCD_DISPLAY_OFF;

out:
	return ret;
}

static int hitachi_lcd_get_power(struct lcd_device *ld)
{
	struct hitachi_lcd_drvdata *drvdata = dev_get_drvdata(&ld->dev);

	dbg("result=%d\n", drvdata->power);

	return drvdata->power;
}

static int hitachi_lcd_set_power(struct lcd_device *ld, int power)
{
	struct hitachi_lcd_drvdata *drvdata = dev_get_drvdata(&ld->dev);
	int ret = 0;

	dbg("power=%d->%d\n", drvdata->power, power);

	if (drvdata->power == power)
		goto out;

	switch (power) {
	case FB_BLANK_UNBLANK:
		ret = hitachi_lcd_display_on(ld);
		if (ret < 0)
			goto out;
		break;

	case FB_BLANK_NORMAL:
	case FB_BLANK_VSYNC_SUSPEND:
	case FB_BLANK_HSYNC_SUSPEND:
	case FB_BLANK_POWERDOWN:
		ret = hitachi_lcd_display_off(ld);
		if (ret < 0)
			goto out;
		break;

	default:
		dev_err(&ld->dev, "Invalied power: %d\n", power);
		ret = -EINVAL;
		goto out;
	}

	drvdata->power = power;

out:
	return ret;
}

static int hitachi_lcd_get_contrast(struct lcd_device *ld)
{
	dbg("get_contrast\n");
	return 0;
}

static int hitachi_lcd_set_contrast(struct lcd_device *ld, int contrast)
{
	dbg("set_contrast\n");
	return 0;
}

static int hitachi_lcd_set_mode(struct lcd_device *ld,
				struct fb_videomode *mode)
{
	dbg("set_mode\n");
	return 0;
}

static int hitachi_lcd_check_fb(struct lcd_device *ld, struct fb_info *fi)
{
	dbg("check_fb\n");
	return 1;
}

static struct lcd_ops hitachi_lcd_ops = {
	.get_power = hitachi_lcd_get_power,
	.set_power = hitachi_lcd_set_power,
	.get_contrast = hitachi_lcd_get_contrast,
	.set_contrast = hitachi_lcd_set_contrast,
	.set_mode = hitachi_lcd_set_mode,
	.check_fb = hitachi_lcd_check_fb,
};

static int hitachi_lcd_init(struct lcd_device *ld)
{
	struct hitachi_lcd_drvdata *drvdata = dev_get_drvdata(&ld->dev);
	int i, j;
	int ret = 0;

	dbg("hitachi_lcd_init\n");


	ret = gpio_request(drvdata->reset_pin, "hitachi-lcd-reset");
	if (ret) {
		dev_err(&ld->dev, "Failed to request GPIO%d for Reset\n",
			drvdata->reset_pin);
		goto out;
	}

	ret = gpio_request(drvdata->cs_pin, "hitachi-lcd-cs");
	if (ret) {
		dev_err(&ld->dev, "Failed to request GPIO%d for CS\n",
			drvdata->cs_pin);
		goto out_reset_gpio_free;
	}

	ret = gpio_request(drvdata->dc_pin, "hitachi-lcd-dc");
	if (ret) {
		dev_err(&ld->dev, "Failed to request GPIO%d for DC\n",
			drvdata->dc_pin);
		goto out_cs_gpio_free;
	}

	ret = gpio_request(drvdata->wr_pin, "hitachi-lcd-wr");
	if (ret) {
		dev_err(&ld->dev, "Failed to request GPIO%d for WR\n",
			drvdata->wr_pin);
		goto out_dc_gpio_free;
	}

	ret = gpio_request(drvdata->maker_id_pin, "lcd_maker_id");
	if (ret) {
		dev_err(&ld->dev, "Failed to request GPIO%d for MAKER ID\n",
			drvdata->maker_id_pin);
		goto out_dc_gpio_free;
	}

	for (i = 0; i < 8; i++) {
		ret = gpio_request(drvdata->data_pin[i], "hitachi-lcd-data");
		if (ret) {
			dev_err(&ld->dev,
				"Failed to request GPIO%d for data%d\n",
				drvdata->data_pin[i], i);
			if (i == 0)
				goto out_wr_gpio_free;
			else
				goto out_data_gpio_free;
		}
	}

	gpio_direction_output(drvdata->reset_pin, 1);
	tegra_gpio_enable(drvdata->reset_pin);

	gpio_direction_output(drvdata->cs_pin, 0);
	tegra_gpio_enable(drvdata->cs_pin);

      	gpio_direction_input(drvdata->maker_id_pin);
      	tegra_gpio_enable(drvdata->maker_id_pin);

        /* VCCIO, VCC, VCI on */
        
        // LGE_CHANGE_S [youngseok.jeong@lge.com] 2011-01-07 enable LCD power
        // VCCIO
        reg_lcd_1v8 = regulator_get(&ld->dev, "vcc_lcd_1v8");
        if (IS_ERR(reg_lcd_1v8)) {
            dev_err(&ld->dev, "Failed to get vcc_lcd_1v8\n");
            return PTR_ERR(reg_lcd_1v8);
        }

        // VCC,VCI
        reg_lcd_2v8 = regulator_get(&ld->dev, "vcc_lcd_2v8");
        if (IS_ERR(reg_lcd_2v8)) {
            dev_err(&ld->dev, "Failed to get vcc_lcd_2v8\n");
            ret = PTR_ERR(reg_lcd_2v8);
            goto err_vcc_get;
        }

        // enable VCCIO
        ret = regulator_enable(reg_lcd_1v8);
        if (ret < 0) {
            dev_err(&ld->dev, "Failed to enable vcc_lcd_1v8\n");
            goto err_vccio_enable;
        }

        // enable VCC,VCI
        ret = regulator_enable(reg_lcd_2v8);
        if (ret < 0) {
            dev_err(&ld->dev, "Failed to enable vcc_lcd_2v8\n");
            goto err_vcc_enable;
        }
          else 
              printk("hitachi_lcd_power_on : Turn on reg_lcd_2v8!!\n");
        // LGE_CHANGE_E [youngseok.jeong@lge.com] 2011-01-07 enable LCD power
    
	goto out;

err_vcc_enable:
	regulator_disable(reg_lcd_1v8);
err_vccio_enable:
	regulator_put(reg_lcd_2v8);
err_vcc_get:
	regulator_put(reg_lcd_1v8);
	reg_lcd_1v8 = NULL;
	reg_lcd_2v8 = NULL;

out_data_gpio_free:
	for (j = 0; j < i; j++)
		gpio_free(drvdata->data_pin[j]);
out_wr_gpio_free:
	gpio_free(drvdata->wr_pin);
out_dc_gpio_free:
	gpio_free(drvdata->dc_pin);
out_cs_gpio_free:
	gpio_free(drvdata->cs_pin);
out_reset_gpio_free:
	gpio_free(drvdata->reset_pin);

    
out:
	return ret;
}

static void hitachi_lcd_uninit(struct lcd_device *ld)
{
	struct hitachi_lcd_drvdata *drvdata = dev_get_drvdata(&ld->dev);
	int i;

	dbg("uninit\n");

	gpio_free(drvdata->reset_pin);
	tegra_gpio_disable(drvdata->reset_pin);

	gpio_free(drvdata->cs_pin);
	tegra_gpio_disable(drvdata->cs_pin);

	gpio_free(drvdata->dc_pin);
	tegra_gpio_disable(drvdata->dc_pin);

	gpio_free(drvdata->wr_pin);
	tegra_gpio_disable(drvdata->wr_pin);

	for (i = 0; i < 8; i++) {
		gpio_free(drvdata->data_pin[i]);
		tegra_gpio_disable(drvdata->data_pin[i]);
	}
}

static int hitachi_lcd_probe(struct platform_device *pdev)
{
	struct hitachi_lcd_platform_data *pdata = pdev->dev.platform_data;
	struct hitachi_lcd_drvdata *drvdata;
	struct lcd_device *ld;
	int ret = 0;

	if (!pdata) {
		dev_err(&pdev->dev, "No platform data\n");
		ret = -ENXIO;
		goto out;
	}

	dev_info(&pdev->dev, "probe\n");

	drvdata = kzalloc(sizeof(struct hitachi_lcd_drvdata), GFP_KERNEL);
	if (!drvdata) {
		dev_err(&pdev->dev, "Can't allocate memory for drvdata\n");
		ret = -ENOMEM;
		goto out;
	}

	ld = lcd_device_register(HITACHI_LCD_DRV_NAME, &pdev->dev, drvdata,
				 &hitachi_lcd_ops);
	if (!ld) {
		dev_err(&pdev->dev, "Can't allocate memory for lcd device\n");
		ret = -ENOMEM;
		goto out_free_drvdata;
	}

	platform_set_drvdata(pdev, ld);
	drvdata->pdev = pdev;
	drvdata->ld = ld;
	drvdata->reset_pin = pdata->reset_pin;
	drvdata->cs_pin = pdata->cs_pin;
	drvdata->dc_pin = pdata->dc_pin;
	drvdata->wr_pin = pdata->wr_pin;
	memcpy(drvdata->data_pin, pdata->data_pin, sizeof(drvdata->data_pin));

#if defined (CONFIG_MACH_STAR)
	drvdata->maker_id_pin = pdata->maker_id_pin;
#endif // MOBII_CHANG_S [jg.noh@mobii.co.kr] 2012-01-31

	ret = hitachi_lcd_init(ld);
	if (ret < 0)
		goto out_lcd_device_unregister;

	drvdata->power = FB_BLANK_POWERDOWN;
	drvdata->mode = HITACHI_LCD_DISPLAY_OFF;
	ret = hitachi_lcd_set_power(ld, FB_BLANK_UNBLANK);
	if (ret < 0)
		goto out_lcd_uninit;

	goto out;

out_lcd_uninit:
	hitachi_lcd_uninit(ld);
out_lcd_device_unregister:
	lcd_device_unregister(ld);
out_free_drvdata:
	kfree(drvdata);
out:
	return ret;
}

static int hitachi_lcd_remove(struct platform_device *pdev)
{
	struct lcd_device *ld = platform_get_drvdata(pdev);
	struct hitachi_lcd_drvdata *drvdata = dev_get_drvdata(&ld->dev);

	dev_info(&pdev->dev, "remove\n");

	hitachi_lcd_set_power(ld, FB_BLANK_POWERDOWN);
	hitachi_lcd_uninit(ld);
	lcd_device_unregister(ld);
	kfree(drvdata);

	return 0;
}

#ifdef CONFIG_PM
static int hitachi_lcd_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct lcd_device *ld = platform_get_drvdata(pdev);
	struct hitachi_lcd_drvdata *drvdata = dev_get_drvdata(&ld->dev);

	dev_info(&pdev->dev, "suspend\n");

	hitachi_lcd_set_power(ld, FB_BLANK_POWERDOWN);
	drvdata->power = FB_BLANK_POWERDOWN;
	drvdata->mode = HITACHI_LCD_DISPLAY_OFF;

	return 0;
}

static int hitachi_lcd_resume(struct platform_device *pdev)
{
	struct lcd_device *ld = platform_get_drvdata(pdev);
	struct hitachi_lcd_drvdata *drvdata = dev_get_drvdata(&ld->dev);
	int ret;

	dev_info(&pdev->dev, "resume\n");

	drvdata->power = FB_BLANK_POWERDOWN;
	drvdata->mode = HITACHI_LCD_DISPLAY_OFF;
	ret = hitachi_lcd_set_power(ld, FB_BLANK_UNBLANK);

	return ret;
}
#else
#define hitachi_lcd_suspend		NULL
#define hitachi_lcd_resume		NULL
#endif /* CONFIG_PM */

static struct platform_driver hitachi_lcd_driver = {
	.probe = hitachi_lcd_probe,
	.remove = hitachi_lcd_remove,
	.suspend = hitachi_lcd_suspend,
	.resume = hitachi_lcd_resume,
	.driver = {
		   .name = HITACHI_LCD_DRV_NAME,
		   .owner = THIS_MODULE,
		   },
};

static int __init hitachi_lcd_module_init(void)
{
	return platform_driver_register(&hitachi_lcd_driver);
}

static void __exit hitachi_lcd_module_exit(void)
{
	platform_driver_unregister(&hitachi_lcd_driver);
}

module_init(hitachi_lcd_module_init);
module_exit(hitachi_lcd_module_exit);

MODULE_DESCRIPTION("Hitachi LCD Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:hitachi-lcd");
