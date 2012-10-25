/*
 * leds-bd2802.c - RGB LED Driver
 *
 * Copyright (C) 2009 Samsung Electronics
 * Kim Kyuwon <q1.kim@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Datasheet: http://www.rohm.com/products/databook/driver/pdf/bd2802gu-e.pdf
 *
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/leds.h>
#include <linux/leds-bd2802.h>
#include <linux/slab.h>
#include <mach/gpio-names.h>
#if defined(CONFIG_LU6500)
#include <linux/lge_hw_rev.h>
#endif
//deukgi.shin@lge.com //20110808 //touch button led off/on early suspend/late resume [S]
#if defined (CONFIG_PM)&&(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#endif
//deukgi.shin@lge.com //20110808 //touch button led off/on early suspend/late resume [E]


#define LED_CTL(rgb2en, rgb1en) ((rgb2en) << 4 | ((rgb1en) << 0))

#define BD2802_LED_OFFSET			0xa
#define BD2802_COLOR_OFFSET			0x3

#define BD2802_REG_CLKSETUP 		0x00
#define BD2802_REG_CONTROL 			0x01
#define BD2802_REG_HOURSETUP		0x02
#define BD2802_REG_CURRENT1SETUP	0x03
#define BD2802_REG_CURRENT2SETUP	0x04
#define BD2802_REG_WAVEPATTERN		0x05

#define BD2802_CURRENT_032			0x10 /* 3.2mA */
#define BD2802_CURRENT_100			0x32 /* 10mA */
#define BD2802_CURRENT_150			0x4B /* 15mA */
#define BD2802_CURRENT_200			0x64 /* 20mA */
#define BD2802_CURRENT_000			0x00 /* 0.0mA */

#define BD2802_PATTERN_FULL			0x07
#define BD2802_PATTERN_HALF			0x03

#if defined(CONFIG_KS1001)
#define TOUCH_LED_RESET TEGRA_GPIO_PE6
#elif defined(CONFIG_LU6500)
#define TOUCH_LED_RESET ((get_lge_pcb_revision() > REV_D) ? (TEGRA_GPIO_PF5) : (TEGRA_GPIO_PE6))
#endif
 
enum led_ids {
	LED1,
	LED2,
	LED_NUM,
};

enum led_colors {
	RED,
	GREEN,
	BLUE,
	COLOR_NUM,
};

enum led_bits {
	BD2802_OFF,
	BD2802_ON,
};

/*
 * State '0' : 'off'
 * State '1' : 'on'.
 */
struct led_state {
	unsigned r:2;
	unsigned g:2;
	unsigned b:2;
};

struct bd2802_led {
	struct bd2802_led_platform_data	*pdata;
	struct i2c_client		*client;
	struct rw_semaphore		rwsem;
	struct work_struct		work;

	struct led_state		led[2];

	/*
	 * Making led_classdev as array is not recommended, because array
	 * members prevent using 'container_of' macro. So repetitive works
	 * are needed.
	 */
	struct led_classdev		cdev_led;
//deukgi.shin@lge.com //20110808 //touch button led off/on early suspend/late resume [S]	
#if defined (CONFIG_PM)&&(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend early_suspend;
#endif
	struct mutex mutex;
//deukgi.shin@lge.com //20110808 //touch button led off/on early suspend/late resume [E]
	/*
	 * Advanced Configuration Function(ADF) mode:
	 * In ADF mode, user can set registers of BD2802GU directly,
	 * therefore BD2802GU doesn't enter reset state.
	 */
	int 				adf_on;

	enum led_ids			led_id;
	enum led_colors			color;
	enum led_bits			state;

	/* General attributes of RGB LEDs */
	int				wave_pattern;
	int				rgb_current;

	int				rgb_current_test;
	bool is_enable;//deukgi.shin@lge.com //20110808 //touch button led off/on early suspend/late resume
};


/*--------------------------------------------------------------*/
/*	BD2802GU helper functions					*/
/*--------------------------------------------------------------*/

static inline int bd2802_is_rgb_off(struct bd2802_led *led, enum led_ids id,
							enum led_colors color)
{
	switch (color) {
	case RED:
		return !led->led[id].r;
	case GREEN:
		return !led->led[id].g;
	case BLUE:
		return !led->led[id].b;
	default:
		dev_err(&led->client->dev, "%s: Invalid color\n", __func__);
		return -EINVAL;
	}
}

static inline int bd2802_is_led_off(struct bd2802_led *led, enum led_ids id)
{
	if (led->led[id].r || led->led[id].g || led->led[id].b)
		return 0;

	return 1;
}

static inline int bd2802_is_all_off(struct bd2802_led *led)
{
	int i;

	for (i = 0; i < LED_NUM; i++)
		if (!bd2802_is_led_off(led, i))
			return 0;

	return 1;
}

static inline u8 bd2802_get_base_offset(enum led_ids id, enum led_colors color)
{
	return id * BD2802_LED_OFFSET + color * BD2802_COLOR_OFFSET;
}

static inline u8 bd2802_get_reg_addr(enum led_ids id, enum led_colors color,
								u8 reg_offset)
{
	return reg_offset + bd2802_get_base_offset(id, color);
}


/*--------------------------------------------------------------*/
/*	BD2802GU core functions					*/
/*--------------------------------------------------------------*/

static int bd2802_write_byte(struct i2c_client *client, u8 reg, u8 val)
{
	int ret = 0;
#if defined(CONFIG_KS1001)||defined(CONFIG_LU6500)
	if((reg > 0x8 && reg < 0xc) || (reg > 0x12 && reg < 0x16)) return 0;
#endif
	ret = i2c_smbus_write_byte_data(client, reg, val);
	if (ret >= 0)
		return 0;

	dev_err(&client->dev, "%s: reg 0x%x, val 0x%x, err %d\n",
						__func__, reg, val, ret);

	return ret;
}

static void bd2802_update_state(struct bd2802_led *led, enum led_ids id,
				enum led_colors color, enum led_bits led_bit)
{
	int i;
	u8 value;

	for (i = 0; i < LED_NUM; i++) {
		if (i == id) {
			switch (color) {
			case RED:
				led->led[i].r = led_bit;
				break;
			case GREEN:
				led->led[i].g = led_bit;
				break;
			case BLUE:
				led->led[i].b = led_bit;
				break;
			default:
				dev_err(&led->client->dev,
					"%s: Invalid color\n", __func__);
				return;
			}
		}
	}

	if (led_bit == BD2802_ON)
		return;

	if (!bd2802_is_led_off(led, id))
		return;

	if (bd2802_is_all_off(led) && !led->adf_on) {
		gpio_set_value(led->pdata->reset_gpio, 0);
		return;
	}

	/*
	 * In this case, other led is turned on, and current led is turned
	 * off. So set RGB LED Control register to stop the current RGB LED
	 */
	value = (id == LED1) ? LED_CTL(1, 0) : LED_CTL(0, 1);
	bd2802_write_byte(led->client, BD2802_REG_CONTROL, value);
}

static void bd2802_configure(struct bd2802_led *led)
{
	struct bd2802_led_platform_data *pdata = led->pdata;
	u8 reg;

	reg = bd2802_get_reg_addr(LED1, RED, BD2802_REG_HOURSETUP);
	bd2802_write_byte(led->client, reg, pdata->rgb_time);

	reg = bd2802_get_reg_addr(LED2, RED, BD2802_REG_HOURSETUP);
	bd2802_write_byte(led->client, reg, pdata->rgb_time);
}

static void bd2802_reset_cancel(struct bd2802_led *led)
{
	gpio_set_value(led->pdata->reset_gpio, 1);
	udelay(100);
	bd2802_configure(led);
}

static void bd2802_enable(struct bd2802_led *led, enum led_ids id)
{
	enum led_ids other_led = (id == LED1) ? LED2 : LED1;
	u8 value, other_led_on;

	other_led_on = !bd2802_is_led_off(led, other_led);
	if (id == LED1)
		value = LED_CTL(other_led_on, 1);
	else
		value = LED_CTL(1 , other_led_on);

	bd2802_write_byte(led->client, BD2802_REG_CONTROL, value);
}

static void bd2802_set_on(struct bd2802_led *led, enum led_ids id,
							enum led_colors color)
{
	u8 reg;

	if (bd2802_is_all_off(led) && !led->adf_on)
		bd2802_reset_cancel(led);

	reg = bd2802_get_reg_addr(id, color, BD2802_REG_CURRENT1SETUP);

	if (led->rgb_current_test >= 0)
		bd2802_write_byte(led->client, reg, led->rgb_current_test);
	else
		bd2802_write_byte(led->client, reg, led->rgb_current);
		
	reg = bd2802_get_reg_addr(id, color, BD2802_REG_CURRENT2SETUP);
	bd2802_write_byte(led->client, reg, BD2802_CURRENT_000);
	reg = bd2802_get_reg_addr(id, color, BD2802_REG_WAVEPATTERN);
	bd2802_write_byte(led->client, reg, BD2802_PATTERN_FULL);

	bd2802_enable(led, id);
	bd2802_update_state(led, id, color, BD2802_ON);
}

static void bd2802_set_brightness(struct led_classdev *led_cdev,
					enum led_brightness value)
{	
	struct bd2802_led *led = container_of(led_cdev, struct bd2802_led, cdev_led);

//deukgi.shin@lge.com //20110808 //touch button led off/on early suspend/late resume [S]
	if(led->is_enable){
		
		if (value == LED_OFF)
			led->state = BD2802_OFF;
		else
			led->state = BD2802_ON;
		schedule_work(&led->work);
	}
//deukgi.shin@lge.com //20110808 //touch button led off/on early suspend/late resume [E]
}

static void bd2802_turn_on(struct bd2802_led *led, enum led_ids id,
				enum led_colors color, enum led_bits led_bit)
{
	if (led_bit == BD2802_OFF) {
		dev_err(&led->client->dev,
					"Only 'on' are allowed\n");
		return;
	}
	else
		bd2802_set_on(led, id, color);
}

static void bd2802_turn_off(struct bd2802_led *led, enum led_ids id,
							enum led_colors color)
{
	u8 reg;

	if (bd2802_is_rgb_off(led, id, color))
		return;

	reg = bd2802_get_reg_addr(id, color, BD2802_REG_CURRENT1SETUP);
	bd2802_write_byte(led->client, reg, BD2802_CURRENT_000);
	reg = bd2802_get_reg_addr(id, color, BD2802_REG_CURRENT2SETUP);
	bd2802_write_byte(led->client, reg, BD2802_CURRENT_000);

	bd2802_update_state(led, id, color, BD2802_OFF);
}

static void bd2802_restore_state(struct bd2802_led *led)
{
	int i;

	for (i = 0; i < LED_NUM; i++) {
		if (led->led[i].r)
			bd2802_turn_on(led, i, RED, led->led[i].r);
		if (led->led[i].g)
			bd2802_turn_on(led, i, GREEN, led->led[i].g);
		if (led->led[i].b)
			bd2802_turn_on(led, i, BLUE, led->led[i].b);
	}
}

static void bd2802_led_work(struct work_struct *work)
{
	struct bd2802_led *led = container_of(work, struct bd2802_led, work);
	int i, j;

	mutex_lock(&led->mutex);//deukgi.shin@lge.com //20110808 //touch button led off/on early suspend/late resume
	if (led->state)
	{
		for (i = 0; i < LED_NUM; i++) {
			led->led_id = i;
			
			for (j = 0; j < COLOR_NUM; j++) {
				led->color = j;
				bd2802_turn_on(led, led->led_id, led->color, led->state);
			}
		}

		if (led->rgb_current_test >= 0)
			led->rgb_current_test = -1;
	}
	else
	{
		for (i = 0; i < LED_NUM; i++) {
			led->led_id = i;
			
			for (j = 0; j < COLOR_NUM; j++) {
				led->color = j;
				bd2802_turn_off(led, led->led_id, led->color);
			}
		}
	}
	mutex_unlock(&led->mutex);//deukgi.shin@lge.com //20110808 //touch button led off/on early suspend/late resume
}

static int bd2802_register_led_classdev(struct bd2802_led *led)
{
	int ret;

	INIT_WORK(&led->work, bd2802_led_work);

	led->cdev_led.name = "button-backlight";
	led->cdev_led.brightness = LED_OFF;
	led->cdev_led.brightness_set = bd2802_set_brightness;

	ret = led_classdev_register(&led->client->dev, &led->cdev_led);
	if (ret < 0) {
		dev_err(&led->client->dev, "couldn't register LED %s\n",
							led->cdev_led.name);
		return ret;
	}

	return 0;
}

static void bd2802_unregister_led_classdev(struct bd2802_led *led)
{
	cancel_work_sync(&led->work);
	led_classdev_unregister(&led->cdev_led);
}

static ssize_t bd2802_current_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned long value = simple_strtoul(buf, NULL, 10);												  
	struct bd2802_led *led = dev_get_drvdata(dev);

	led->rgb_current_test = value;
	led->state = BD2802_ON;
	schedule_work(&led->work);
	
	return size;
}

static DEVICE_ATTR(rgb_current, S_IRUGO|S_IWUGO, NULL, bd2802_current_store);

static struct attribute *bd2802_attributes[] = {
	&dev_attr_rgb_current.attr,
	NULL
};

static const struct attribute_group bd2802_group = {
	.attrs = bd2802_attributes,
};
//deukgi.shin@lge.com //20110808 //touch button led off/on early suspend/late resume [S]
static int bd2802_early_suspend(struct early_suspend *es);
static int bd2802_late_resume(struct early_suspend *es);
//deukgi.shin@lge.com //20110808 //touch button led off/on early suspend/late resume [S]
static int __devinit bd2802_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct bd2802_led *led;
	struct bd2802_led_platform_data *pdata;
	int ret, i;

	led = kzalloc(sizeof(struct bd2802_led), GFP_KERNEL);
	if (!led) {
		dev_err(&client->dev, "failed to allocate driver data\n");
		return -ENOMEM;
	}

	led->client = client;
	pdata = led->pdata = client->dev.platform_data;
	i2c_set_clientdata(client, led);
	mutex_init(&led->mutex);//deukgi.shin@lge.com //20110808 //touch button led off/on early suspend/late resume

	pdata->reset_gpio = TOUCH_LED_RESET ;

	/* Configure RESET GPIO (L: RESET, H: RESET cancel) */
	gpio_request(pdata->reset_gpio, "RGB_RESETB");
	gpio_direction_output(pdata->reset_gpio, 1);

	/* Tacss = min 0.1ms */
	udelay(100);

	/* Detect BD2802GU */
	ret = bd2802_write_byte(client, BD2802_REG_CLKSETUP, 0x00);
	if (ret < 0) {
		dev_err(&client->dev, "failed to detect device\n");
		goto failed_free;
	} else
		dev_info(&client->dev, "return 0x%02x\n", ret);

	/* To save the power, reset BD2802 after detecting */
	gpio_set_value(led->pdata->reset_gpio, 0);

	/* Default attributes */
	led->wave_pattern = BD2802_PATTERN_HALF;
	led->rgb_current = BD2802_CURRENT_100;
	led->rgb_current_test = -1;
	led->is_enable = true;//deukgi.shin@lge.com //20110808 //touch button led off/on early suspend/late resume
	
	init_rwsem(&led->rwsem);

	ret = bd2802_register_led_classdev(led);
	if (ret < 0)
		goto failed_free;

	if ((ret = sysfs_create_group(&client->dev.kobj, &bd2802_group))) {
		ret = -ENOMEM;
		goto failed_unregister_dev_file;
	}
//deukgi.shin@lge.com //20110808 //touch button led off/on early suspend/late resume [S]	
#if defined (CONFIG_PM)&&(CONFIG_HAS_EARLYSUSPEND)
	led->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN - 2;
	led->early_suspend.suspend = bd2802_early_suspend;
	led->early_suspend.resume = bd2802_late_resume;
	register_early_suspend(&led->early_suspend);
#endif	
//deukgi.shin@lge.com //20110808 //touch button led off/on early suspend/late resume [E]

	return 0;

failed_unregister_dev_file:
	bd2802_unregister_led_classdev(led);
failed_free:
	kfree(led);

	return ret;
}

static int __exit bd2802_remove(struct i2c_client *client)
{
	struct bd2802_led *led = i2c_get_clientdata(client);
	int i;

	sysfs_remove_group(&client->dev.kobj, &bd2802_group);
	
	gpio_set_value(led->pdata->reset_gpio, 0);
	bd2802_unregister_led_classdev(led);
	led->is_enable = false;//deukgi.shin@lge.com //20110808 //touch button led off/on early suspend/late resume
	kfree(led);

	return 0;
}
//deukgi.shin@lge.com //20110808 //touch button led off/on early suspend/late resume [S]
#ifdef CONFIG_PM
#if defined (CONFIG_HAS_EARLYSUSPEND)
static int bd2802_early_suspend(struct early_suspend *es)
{
	struct bd2802_led *led = container_of(es, struct bd2802_led, early_suspend);

	printk("ENTER : %s\n",__FUNCTION__);
	gpio_set_value(led->pdata->reset_gpio, 0);
	led->state = BD2802_OFF;
	led->is_enable = false;
	schedule_work(&led->work);

	return 0;
}

static int bd2802_late_resume(struct early_suspend *es)
{
	struct bd2802_led *led = container_of(es, struct bd2802_led, early_suspend);

	printk("ENTER : %s\n",__FUNCTION__);

	if (!bd2802_is_all_off(led) || led->adf_on) {
		bd2802_reset_cancel(led);
		bd2802_restore_state(led);
	}
	led->state = BD2802_ON;
	led->is_enable = true;
	schedule_work(&led->work);

	return 0;
}
#else
static int bd2802_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct bd2802_led *led = i2c_get_clientdata(client);

	printk("ENTER : %s\n",__FUNCTION__);

	gpio_set_value(led->pdata->reset_gpio, 0);
	led->is_enable = false;
	return 0;
}

static int bd2802_resume(struct i2c_client *client)
{
	struct bd2802_led *led = i2c_get_clientdata(client);

	printk("ENTER : %s\n",__FUNCTION__);

	if (!bd2802_is_all_off(led) || led->adf_on) {
		bd2802_reset_cancel(led);
		bd2802_restore_state(led);
	}
	led->is_enable = true;
	return 0;
}
#endif
#endif
//deukgi.shin@lge.com //20110808 //touch button led off/on early suspend/late resume [E]

static const struct i2c_device_id bd2802_id[] = {
	{ "bssq_touch_led", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, bd2802_id);

static struct i2c_driver bd2802_i2c_driver = {
	.driver	= {
		.name	= "bssq_touch_led",
	},
	.probe		= bd2802_probe,
	.remove		= __exit_p(bd2802_remove),
#if defined (CONFIG_PM) && (!CONFIG_HAS_EARLYSUSPEND)//deukgi.shin@lge.com //20110808 //touch button led off/on early suspend/late resume
	.suspend	= bd2802_suspend,
	.resume		= bd2802_resume,
#endif
	.id_table	= bd2802_id,
};

static int __init bd2802_init(void)
{
	return i2c_add_driver(&bd2802_i2c_driver);
}
module_init(bd2802_init);

static void __exit bd2802_exit(void)
{
	i2c_del_driver(&bd2802_i2c_driver);
}
module_exit(bd2802_exit);

MODULE_AUTHOR("youngjin.yoo@lge.com");
MODULE_DESCRIPTION("touch led");
MODULE_LICENSE("GPL");

