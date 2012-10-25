/*
 * star(lgp990) touch LED
 *
 * Copyright (C) 2009 LGE, Inc.
 *
 * Author: Sungyel Bae  <sungyel.bae@lge.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 */

/**
	@brief		 star(lgp999) touch LED
 
	@author		 sungyel.bae@lge.com
	@date		 2011-01-26
 
	@version	 V1.00		 2011.01.26		 Sungyel Bae	 Create
*/
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/leds.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>
#include <linux/mfd/max8907c.h>
#include <linux/err.h>

#define to_max8907c_led(led_cdev) \
    container_of(led_cdev, struct max8907c_led, cdev)

//#define STAR_TOUCH_LED_TEST

#ifdef STAR_TOUCH_LED_TEST
#include <linux/delay.h>
#endif

static void star_led_work(struct work_struct *work)
{
	struct max8907c_led *led = container_of(work, struct max8907c_led, work);
	int ret;
	unsigned long flags;

	mutex_lock(&led->mutex);

	if(led->value)
		ret = regulator_set_current_limit(led->isink, led->value, led->value);
	else
		ret = regulator_set_current_limit(led->isink, 0, 0);
	
	if (ret != 0)
		dev_err(led->cdev.dev, "Failed to set %duA: %d\n",
			led->value, ret);

	mutex_unlock(&led->mutex);
	return;
}

static void star_led_set(struct led_classdev *led_cdev,
				enum led_brightness value)
{
	struct max8907c_led *led = to_max8907c_led(led_cdev);
	unsigned long flags;

// MOBII_S [shhong@mobii.co.kr] 2012-06-25: Add LED Retain Function.
#if defined(CONFIG_MACH_STAR)
	if(led_cdev->br_maintain_trigger == 1) {
		return;
	}
#endif
// MOBII_E [shhong@mobii.co.kr] 2012-06-25: Add LED Retain Function.

	spin_lock_irqsave(&led->value_lock, flags);
	led->value = value;
	schedule_work(&led->work);
	spin_unlock_irqrestore(&led->value_lock, flags);

}

static void star_led_remove(struct platform_device *pdev)
{
	struct max8907c_led *led = platform_get_drvdata(pdev);

	led_classdev_unregister(&led->cdev);
	kfree(led);
}

static void star_led_shutdown(struct platform_device *pdev)
{
	struct max8907c_led *led = platform_get_drvdata(pdev);
	
	mutex_lock(&led->mutex);
	led->value = LED_OFF;
	mutex_unlock(&led->mutex);
}

static int star_led_probe(struct platform_device *pdev)
{
	struct max8907c_led *led;
	struct max8907c_led_platform_data *pdata = pdev->dev.platform_data;
	int ret;

	printk("Touch LED probe\n");

	if (pdev->dev.platform_data == NULL){
		dev_err(&pdev->dev, "no platform data\n");
		return -ENODEV;
	}

	led = kzalloc(sizeof(*led), GFP_KERNEL);
	if (led == NULL){
		ret = -ENOMEM;
		printk("Error: led couldn't get memory\n");
		return ret;
	}
	/* get regulator wled control functions address */
	
	led->isink = regulator_get(NULL, "vcc_wled");
	if (IS_ERR_OR_NULL(led->isink)) {
		printk("touch led: couldn't get rgulator star_led\n");
		kfree(led);
		return -1;
	}

	led->cdev.brightness_set = star_led_set;
	//led->cdev.default_trigger = pdata->default_trigger;	
		/* timer, heartbeat, backlight, default-on */
	led->cdev.name = pdev->name;
	led->cdev.flags |= LED_CORE_SUSPENDRESUME;

	spin_lock_init(&led->value_lock);
	mutex_init(&led->mutex);
	INIT_WORK(&led->work, star_led_work);
	led->value = LED_OFF;

	/* register out new led device */
	platform_set_drvdata(pdev, led);	

	ret = led_classdev_register(&pdev->dev, &led->cdev);
	
	if (ret < 0){
		dev_err(&pdev, "led_classdev_register failed\n");
		kfree(led);
		return ret;
	}
	printk("Touch LED probe success\n");

#ifdef STAR_TOUCH_LED_TEST
	printk("Testing LED\n");
	regulator_set_current_limit(led->isink, 2000, 2000);	/*2mA*/
	mdelay(10);
	regulator_set_current_limit(led->isink, 0, 0);
#endif

	return 0;
}

static struct platform_driver star_led_driver = {
	.probe	= star_led_probe,
	.remove = star_led_remove,
	.shutdown = star_led_shutdown,
	.driver = {
		.name 	= "star_led",
		.owner	= THIS_MODULE,
	},
};

static int __init star_led_init(void)
{
	return platform_driver_register(&star_led_driver);
}

static void __exit star_led_exit(void)
{
	platform_driver_unregister(&star_led_driver);
}

module_init(star_led_init);
module_exit(star_led_exit);

MODULE_AUTHOR("Sungyel Bae <sungyel.bae@lge.com>");
MODULE_DESCRIPTION("star led driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:star_led");

