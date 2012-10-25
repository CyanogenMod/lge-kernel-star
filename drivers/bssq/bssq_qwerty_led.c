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
	@brief		 bssq(lgp999) touch LED
 
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
#include <linux/earlysuspend.h>  // 20110725 sangki.hyun@lge.com touch_led

#define to_max8907c_led(led_cdev) \
    container_of(led_cdev, struct max8907c_led, cdev)

#if defined (CONFIG_SU880) || defined (CONFIG_KU8800) || defined (CONFIG_LU8800) || defined (CONFIG_KS1103)
static int set_current_mA = 15 ; // Qwrty LED Current Value - Unit : mA
#else
static int set_current_mA = 15 ; // Qwrty LED Current Value - Unit : mA  julius.moon 2011.06.16. Requested youngjoon.so for B-Qwerty PJT
#endif

static int lock_key_led_control = 0;

// hyokmin.kwon@lge.com Refer max8907c_regulator_wled_set_current_limit to change this values
#if defined (CONFIG_KS1103)
static int to_be_on = 0;
#define CHARGING_LED_CURRENT 50
#define KEY_LED_CURRENT 50
#define KEY_LED_VALUE 150
#endif


// 20110725 sangki.hyun@lge.com touch_led [S]
//#if defined (CONFIG_HAS_EARLYSUSPEND) && ( defined (CONFIG_SU880) || defined (CONFIG_KU8800) || defined (CONFIG_LU8800) )
static void qwerty_early_suspend(struct early_suspend *es);
static void qwerty_late_resume(struct early_suspend *es);
//#endif
// 20110725 sangki.hyun@lge.com touch_led [E]
static void qwerty_led_work(struct work_struct *work)
{
	struct max8907c_led *led = container_of(work, struct max8907c_led, work);
	int ret;
	unsigned long flags;

	printk("ENTER %s Value is : %d\n", __FUNCTION__, led->value);
	mutex_lock(&led->mutex);
	
#if defined (CONFIG_KS1103)
	if(led->touch_key_enable == true && led->value == KEY_LED_VALUE)
	{
		mutex_unlock(&led->mutex);
		return;
	}
#else
// 20110725 sangki.hyun@lge.com touch_led [S]
//#if defined (CONFIG_HAS_EARLYSUSPEND) && ( defined (CONFIG_SU880) || defined (CONFIG_KU8800) || defined (CONFIG_LU8800) )
	if(led->touch_key_enable == true)
	{
		led->value = 0;
	}
//#endif
// 20110725 sangki.hyun@lge.com touch_led [E]
#endif

#if defined (CONFIG_KS1103)
	if(led->value == KEY_LED_VALUE)
		ret = regulator_set_current_limit(led->isink, KEY_LED_CURRENT, KEY_LED_CURRENT);
	else if(led->value)
		ret = regulator_set_current_limit(led->isink, led->value, led->value);
	else
		ret = regulator_set_current_limit(led->isink, 0, 0);
#else
	if(led->value)
		ret = regulator_set_current_limit(led->isink, led->value, led->value);
	else
		ret = regulator_set_current_limit(led->isink, 0, 0);
#endif
	
	if (ret != 0)
		dev_err(led->cdev.dev, "Failed to set %duA: %d\n",
			led->value, ret);

	mutex_unlock(&led->mutex);
	return;
}

static void qwerty_led_set(struct led_classdev *led_cdev,
				enum led_brightness value)
{
	struct max8907c_led *led = to_max8907c_led(led_cdev);
	unsigned long flags;
	printk("ENTER :  %s led_brightness : %d\n",  __FUNCTION__, value);

#if defined(CONFIG_KS1103)
	if(value == 254)
	{
		lock_key_led_control = 1;
		led->value = KEY_LED_VALUE;
		schedule_work(&led->work);

		return;
	}
	else if(value == 253)
	{
		lock_key_led_control = 0;
		led->value = 0;
		schedule_work(&led->work);

		return;
	}

	if(lock_key_led_control)
		return;
#endif

#if defined (CONFIG_KS1103)
	if((led->touch_key_enable == true) && (value == 255))
	{
		to_be_on = 1;
		return;
	}
	to_be_on = 0;
#else
	if((led->touch_key_enable == true) && (value == 0))
		return;
#endif

//	spin_lock_irqsave(&led->value_lock, flags);

// Qwrty LED Current Value - Unit : mA  julius.moon [START]
#if defined (CONFIG_KS1103)
	if(value == 5)
		led->value = CHARGING_LED_CURRENT;
	else if(value == 0)
		led->value = 0;
	else
		led->value = KEY_LED_VALUE;
#else
	if(value > 0)
		value = set_current_mA*10;
// Qwrty LED Current Value - Unit : mA  julius.moon [END]
	led->value = value;
#endif
	schedule_work(&led->work);
//	spin_unlock_irqrestore(&led->value_lock, flags);
}

static void qwerty_led_remove(struct platform_device *pdev)
{
	struct max8907c_led *led = platform_get_drvdata(pdev);
// 20110725 sangki.hyun@lge.com touch_led [S]
//#if defined (CONFIG_HAS_EARLYSUSPEND) && ( defined (CONFIG_SU880) || defined (CONFIG_KU8800) || defined (CONFIG_LU8800) )
	unregister_early_suspend(&led->early_suspend);
//#endif
// 20110725 sangki.hyun@lge.com touch_led [E]

	led_classdev_unregister(&led->cdev);
	kfree(led);
}

static void qwerty_led_shutdown(struct platform_device *pdev)
{
	struct max8907c_led *led = platform_get_drvdata(pdev);
	
	mutex_lock(&led->mutex);
	led->value = LED_OFF;
// 20110623 sangki.hyun@lge.com keyled shutdown [S] {
#if defined (CONFIG_SU880) || defined (CONFIG_KU8800) || defined (CONFIG_LU8800) || defined (CONFIG_KS1103)    
	schedule_work(&led->work);
#endif
// 20110623 sangki.hyun@lge.com keyled shutdown [E] }
	mutex_unlock(&led->mutex);
}

static int qwerty_led_probe(struct platform_device *pdev)
{
	struct max8907c_led *led;
	struct max8907c_led_platform_data *pdata = pdev->dev.platform_data;
	int ret;

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
		printk("touch led: couldn't get rgulator bssq_led\n");
		kfree(led);
		return -1;
	}

	led->cdev.brightness_set = qwerty_led_set;
	//led->cdev.default_trigger = pdata->default_trigger;	
		/* timer, heartbeat, backlight, default-on */
	// LGE_CHANGE [dojip.kim@lge.com] 2011-02-22, use the generic name
//	led->cdev.name = pdev->name;
// 20110621 sangki.hyun@lge.com key_led [S] {
#if defined (CONFIG_SU880) || defined (CONFIG_KU8800) || defined (CONFIG_LU8800) || defined (CONFIG_KS1103)   
	led->cdev.name = "button-backlight";
#else
	led->cdev.name = "keyboard-backlight";
#endif
// 20110621 sangki.hyun@lge.com key_led [E] }

#if !defined (CONFIG_KS1103)
	led->cdev.flags |= LED_CORE_SUSPENDRESUME;
#endif

	spin_lock_init(&led->value_lock);
	mutex_init(&led->mutex);
	INIT_WORK(&led->work, qwerty_led_work);
	led->value = LED_OFF;

	/* register out new led device */
	platform_set_drvdata(pdev, led);	

	ret = led_classdev_register(&pdev->dev, &led->cdev);
	
	if (ret < 0){
		dev_err(&pdev, "led_classdev_register failed\n");
		kfree(led);
		return ret;
	}
// 20110725 sangki.hyun@lge.com touch_led [S]
//#if defined (CONFIG_HAS_EARLYSUSPEND) && ( defined (CONFIG_SU880) || defined (CONFIG_KU8800) || defined (CONFIG_LU8800) )
		led->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
		led->early_suspend.suspend =  qwerty_early_suspend;
		led->early_suspend.resume =  qwerty_late_resume;
		register_early_suspend(&led->early_suspend);
//#endif
// 20110725 sangki.hyun@lge.com touch_led [E]

	return 0;
}

// 20110725 sangki.hyun@lge.com touch_led [S]
//#if defined (CONFIG_HAS_EARLYSUSPEND) && ( defined (CONFIG_SU880) || defined (CONFIG_KU8800) || defined (CONFIG_LU8800) )
extern int slide_state;
static void qwerty_early_suspend(struct early_suspend *es)
{
	struct max8907c_led *led = container_of(es, struct max8907c_led, early_suspend);

	led->touch_key_enable = true;
#if !defined (CONFIG_KS1103)
	schedule_work(&led->work);
#endif
}

static void qwerty_late_resume(struct early_suspend *es)
{
	struct max8907c_led *led = container_of(es, struct max8907c_led, early_suspend);

	led->touch_key_enable = false;

#if defined (CONFIG_KS1103)
	if(to_be_on)
	{
		to_be_on = 0;
		led->value = KEY_LED_VALUE;
		schedule_work(&led->work);
	}
#else

#if defined(CONFIG_LU6500)	
	if(slide_state == 0)
	{
		led->value = set_current_mA*10;
	}else{
		led->value = 0;
	}
#else
	led->value = set_current_mA*10;
#endif	
	schedule_work(&led->work);

#endif
}
//#endif
// 20110725 sangki.hyun@lge.com touch_led [E]

static struct platform_driver qwerty_led_driver = {
	.probe	= qwerty_led_probe,
	.remove = qwerty_led_remove,
#if !defined (CONFIG_KS1103)
	.shutdown = qwerty_led_shutdown,
#endif
	.driver = {
		.name 	= "bssq_qwerty_led",
		.owner	= THIS_MODULE,
	},
};

static int __init qwerty_led_init(void)
{
	return platform_driver_register(&qwerty_led_driver);
}

static void __exit qwerty_led_exit(void)
{
	platform_driver_unregister(&qwerty_led_driver);
}

module_init(qwerty_led_init);
module_exit(qwerty_led_exit);

MODULE_AUTHOR("Sungyel Bae <sungyel.bae@lge.com>");
MODULE_DESCRIPTION("bssq qwerty led driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:bssq_led");
