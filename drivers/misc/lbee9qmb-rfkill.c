/*
 * Bluetooth+WiFi Murata LBEE19QMBC rfkill power control via GPIO
 *
 * Copyright (C) 2010 NVIDIA Corporation
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/rfkill.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>
#include <linux/platform_device.h>
#include <linux/lbee9qmb-rfkill.h>

#ifdef BRCM_LPM
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/wakelock.h>
#include <linux/spinlock.h>

#define BRCM_WAKELOCKTIMEOUT
#define WAKELOCKTIMEOUT 1147
#ifdef BRCM_WAKELOCKTIMEOUT
#include <linux/hrtimer.h>
#endif
struct bcm_bt_lpm {
	unsigned int gpio_host_wake;

	int wake;
	int host_wake;
	char bt_enable;
	int host_wake_irq;

#ifdef BRCM_WAKELOCKTIMEOUT
	struct hrtimer check_hostwakeup_timer;
	ktime_t check_hostwakeup_delay;
 #endif
 
	spinlock_t bt_lock;
	unsigned long bt_lock_flags;
	struct work_struct host_wake_work;

	struct wake_lock bt_wake_lock;
	struct wake_lock host_wake_lock;
} bt_lpm;
#endif

/* 20100818 for debugging of resetting when getting a dump [START] */
#include "nvodm_services.h"

/* 20101005  for debugging of resetting when getting a dump */
extern NvBool NvOdmBtEnable(NvBool IsEnable);

static int lbee9qmb_rfkill_set_power(void *data, bool blocked)
{
	struct platform_device *pdev = data;
	struct lbee9qmb_platform_data *plat = pdev->dev.platform_data;

/* 20100818 for debugging of resetting when getting a dump [START] */
    NvU32 RequestedPeriod, ReturnedPeriod;
    RequestedPeriod = 0;
/* 20100818 for debugging of resetting when getting a dump [END] */
    
/* 20100721 for BTLA Porting to Froyo [START] */
#if 0
	struct regulator *regulator;

	regulator = regulator_get(&pdev->dev, "Vdd");

	if (IS_ERR(regulator)) {
		dev_err(&pdev->dev, "Unable to get regulator Vdd\n");
		return PTR_ERR(regulator);
	}
#endif
/* 20100721 for BTLA Porting to Froyo [END] */

    /* 20101005  for debugging of resetting when getting a dump [START] */
	if (!blocked) {
                NvOdmBtEnable(NV_TRUE);
//		regulator_enable(regulator); /* 20100721 for BTLA Porting to Froyo */
		gpio_set_value(plat->gpio_reset, 0);
//		if (plat->gpio_pwr!=-1)
//			gpio_set_value(plat->gpio_pwr, 0);
		msleep(plat->delay);
//		if (plat->gpio_pwr!=-1)
//			gpio_set_value(plat->gpio_pwr, 1);
		gpio_set_value(plat->gpio_reset, 1);
		bt_lpm.bt_enable=1;
	} else {
		gpio_set_value(plat->gpio_reset, 0);
//		regulator_disable(regulator); /* 20100721 for BTLA Porting to Froyo */
                NvOdmBtEnable(NV_FALSE);
            bt_lpm.bt_enable=0;
	}
    /* 20101005  for debugging of resetting when getting a dump [END] */

//	regulator_put(regulator); /* 20100721 for BTLA Porting to Froyo */
	return 0;
}

static struct rfkill_ops lbee9qmb_rfkill_ops = {
	.set_block = lbee9qmb_rfkill_set_power,
};

static int lbee9qmb_rfkill_probe(struct platform_device *pdev)
{
	struct lbee9qmb_platform_data *plat = pdev->dev.platform_data;
	struct rfkill *rfkill;

	int rc;

	if (!plat) {
		dev_err(&pdev->dev, "no platform data\n");
		return -ENOSYS;
	}

	rc = gpio_request(plat->gpio_reset, "lbee9qmb_reset");
	if (rc < 0) {
		dev_err(&pdev->dev, "gpio_request failed\n");
		return rc;
	}
	/*if (plat->gpio_pwr!=-1)
	{
		rc = gpio_request(plat->gpio_pwr, "lbee9qmb_pwr");
		gpio_direction_output(plat->gpio_pwr,0);
	}*/

	rfkill = rfkill_alloc("lbee9qmb-rfkill", &pdev->dev,
			RFKILL_TYPE_BLUETOOTH, &lbee9qmb_rfkill_ops, pdev);
	if (!rfkill) {
		rc = -ENOMEM;
		goto fail_gpio;
	}
	platform_set_drvdata(pdev, rfkill);
	gpio_direction_output(plat->gpio_reset, 0);
	
	rc = rfkill_register(rfkill);
	if (rc < 0)
		goto fail_alloc;

	return 0;

fail_alloc:
	rfkill_destroy(rfkill);
fail_gpio:
	gpio_free(plat->gpio_reset);
//	if (plat->gpio_pwr!=-1)
//		gpio_free(plat->gpio_pwr);
	return rc;
		
}

static int lbee9qmb_rfkill_remove(struct platform_device *pdev)
{
	struct lbee9qmb_platform_data *plat = pdev->dev.platform_data;
	struct rfkill *rfkill = platform_get_drvdata(pdev);

	rfkill_unregister(rfkill);
	rfkill_destroy(rfkill);
	gpio_free(plat->gpio_reset);
//	if (plat->gpio_pwr!=-1)
//		gpio_free(plat->gpio_pwr);
	return 0;
	
}

static struct platform_driver lbee9qmb_rfkill_driver = {
	.probe = lbee9qmb_rfkill_probe,
	.remove = lbee9qmb_rfkill_remove,
	.driver = {
		.name = "lbee9qmb-rfkill",
		.owner = THIS_MODULE,
	},
};

#ifdef BRCM_HOST_WAKE
static void update_host_wake_locked(int host_wake)
{
	if (host_wake == bt_lpm.host_wake)
		return;
		
	bt_lpm.host_wake = host_wake;

	printk(KERN_ERR "BRCM_LPM: wake lock HOST_WAKE host=%d, bt=%d\n",host_wake,bt_lpm.wake);
	if (host_wake)
	{
		wake_lock(&bt_lpm.host_wake_lock);
	}
	else
	{
		wake_unlock(&bt_lpm.host_wake_lock);
	}
}

static irqreturn_t host_wake_isr(int irq, void *dev)
{
	int host_wake;

	host_wake = gpio_get_value(bt_lpm.gpio_host_wake);

#ifdef BRCM_WAKELOCKTIMEOUT
	printk(KERN_ERR "BRCM_LPM: host_wake_isr schedule\n");
	schedule_work(&bt_lpm.host_wake_work);
	return IRQ_HANDLED;
#endif

//	spin_lock_irqsave(&bt_lpm.bt_lock, bt_lpm.bt_lock_flags);
//We need disable the irq here

	printk(KERN_ERR "BRCM_LPM: host_wake_isr host wake=%d\n",host_wake);
	set_irq_type(bt_lpm.host_wake_irq, host_wake ? IRQF_TRIGGER_LOW : IRQF_TRIGGER_HIGH);
	schedule_work(&bt_lpm.host_wake_work);

	return IRQ_HANDLED;
}

#ifdef BRCM_WAKELOCKTIMEOUT
static enum hrtimer_restart check_hostwakeup(struct hrtimer *timer) {
	int host_wake;

	host_wake = gpio_get_value(bt_lpm.gpio_host_wake);
	if(host_wake && bt_lpm.bt_enable)
	{
        	printk(KERN_ERR "BRCM_LPM: check_hostwakeup postpone wakelock for host wake=%d\n",host_wake);
           schedule_work(&bt_lpm.host_wake_work);
	}

	return HRTIMER_NORESTART;
}
#endif

static void brcm_host_wake_work_func(struct work_struct *ignored)
{
	int host_wake;

	host_wake = gpio_get_value(bt_lpm.gpio_host_wake);

#ifdef BRCM_WAKELOCKTIMEOUT
    if(host_wake)
    {
	printk(KERN_ERR "BRCM_LPM: BRCM_WAKELOCKTIMEOUT host wake=%d\n",host_wake);

	hrtimer_try_to_cancel(&bt_lpm.check_hostwakeup_timer);
  	hrtimer_start(&bt_lpm.check_hostwakeup_timer, bt_lpm.check_hostwakeup_delay,
			HRTIMER_MODE_REL);

	wake_lock_timeout(&bt_lpm.host_wake_lock, WAKELOCKTIMEOUT);//6*HZ);
    }
    return;
#endif
	update_host_wake_locked(host_wake);
}
#endif

#ifdef BRCM_BT_WAKE
static int lbee9qmb_rfkill_set_btwake(void *data, bool blocked)
{
	struct platform_device *pdev = data;
	struct lbee9qmb_platform_data *plat = pdev->dev.platform_data;

      if(bt_lpm.wake==blocked)
        return 0;
      bt_lpm.wake=blocked;
        
	printk(KERN_ERR "BRCM_LPM: wake gpio = %x blocked=%d \n",plat->gpio_btwake,blocked);

	if (blocked) {
		gpio_set_value(plat->gpio_btwake, 1);
	} else {
		gpio_set_value(plat->gpio_btwake, 0);
	}

	printk(KERN_ERR "BRCM_LPM: wake lock BT_WAKE host=%d, bt=%d\n",bt_lpm.host_wake,bt_lpm.wake);
	if (bt_lpm.wake)
	{
		wake_lock(&bt_lpm.bt_wake_lock);
	}
	else
	{
		wake_unlock(&bt_lpm.bt_wake_lock);
	}
	return 0;
}

static struct rfkill_ops lbee9qmb_rfkill_btwake_ops = {
	.set_block = lbee9qmb_rfkill_set_btwake,
};

static int lbee9qmb_rfkill_btwake_probe(struct platform_device *pdev)
{
	struct lbee9qmb_platform_data *plat = pdev->dev.platform_data;
	struct rfkill *rfkill;

	int rc;
	int irq;
	int ret;
	int host_wake;

	if (!plat) {
		dev_err(&pdev->dev, "no platform data\n");
		return -ENOSYS;
	}
	
	wake_lock_init(&bt_lpm.bt_wake_lock, WAKE_LOCK_SUSPEND,
				"bt_wake");
#ifdef BRCM_HOST_WAKE
	wake_lock_init(&bt_lpm.host_wake_lock, WAKE_LOCK_SUSPEND,
				"host_wake");
	bt_lpm.gpio_host_wake=plat->gpio_hostwake;
	//spin_lock_init(&bt_lpm.bt_lock);
	INIT_WORK(&bt_lpm.host_wake_work, brcm_host_wake_work_func);
#endif

	rc = gpio_request(plat->gpio_btwake, "lbee9qmb_reset_btwake");
	if (rc < 0) {
		dev_err(&pdev->dev, "gpio_request failed\n");
		return rc;
	}

	rfkill = rfkill_alloc("lbee9qmb-rfkill_btwake", &pdev->dev,
			RFKILL_TYPE_BLUETOOTH, &lbee9qmb_rfkill_btwake_ops, pdev);
	if (!rfkill) {
		rc = -ENOMEM;
		goto fail_gpio;
	}
	platform_set_drvdata(pdev, rfkill);
	gpio_direction_output(plat->gpio_btwake, 1);
	
	rc = rfkill_register(rfkill);
	if (rc < 0)
		goto fail_alloc;

#ifdef BRCM_HOST_WAKE
	rc = gpio_request(plat->gpio_hostwake, "lbee9qmb_reset_hostwake");
	gpio_direction_input(plat->gpio_hostwake);
	host_wake=gpio_get_value(bt_lpm.gpio_host_wake);
	irq = gpio_to_irq(plat->gpio_hostwake);
	bt_lpm.host_wake_irq=irq;
#ifdef BRCM_WAKELOCKTIMEOUT
	hrtimer_init(&bt_lpm.check_hostwakeup_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	bt_lpm.check_hostwakeup_delay = ktime_set(5, 0);  /* 5 sec */
	bt_lpm.check_hostwakeup_timer.function = check_hostwakeup;

	set_irq_type(irq, IRQ_TYPE_EDGE_RISING);
	
	ret = request_irq(irq, host_wake_isr, 0,
			"bt host_wake", NULL);
#else
	ret = request_irq(irq, host_wake_isr, IRQF_TRIGGER_HIGH,
			"bt host_wake", NULL);
#endif			
	
	printk(KERN_ERR "BRCM_LPM: irq=%d ret=%d HOST_WAKE=%d\n",irq,ret,host_wake);
#endif

	return 0;

fail_alloc:
	rfkill_destroy(rfkill);
fail_gpio:
	gpio_free(plat->gpio_btwake);

	return rc;
		
}

static int lbee9qmb_rfkill_btwake_remove(struct platform_device *pdev)
{
	struct lbee9qmb_platform_data *plat = pdev->dev.platform_data;
	struct rfkill *rfkill = platform_get_drvdata(pdev);

	rfkill_unregister(rfkill);
	rfkill_destroy(rfkill);
	gpio_free(plat->gpio_btwake);
	wake_lock_destroy(&bt_lpm.bt_wake_lock);
#ifdef BRCM_HOST_WAKE
	wake_lock_destroy(&bt_lpm.host_wake_lock);
#endif
//	if (plat->gpio_pwr!=-1)
//		gpio_free(plat->gpio_pwr);
	return 0;
	
}

static struct platform_driver lbee9qmb_rfkill_btwake_driver = {
	.probe = lbee9qmb_rfkill_btwake_probe,
	.remove = lbee9qmb_rfkill_btwake_remove,
	.driver = {
		.name = "lbee9qmb-rfkill_btwake",
		.owner = THIS_MODULE,
	},
};
#endif

static int __init lbee9qmb_rfkill_init(void)
{
#ifdef BRCM_BT_WAKE
	platform_driver_register(&lbee9qmb_rfkill_driver);
	return platform_driver_register(&lbee9qmb_rfkill_btwake_driver);
#else
	return platform_driver_register(&lbee9qmb_rfkill_driver);
#endif
}

static void __exit lbee9qmb_rfkill_exit(void)
{
#ifdef BRCM_BT_WAKE
	platform_driver_unregister(&lbee9qmb_rfkill_btwake_driver);
#endif
	platform_driver_unregister(&lbee9qmb_rfkill_driver);
}

module_init(lbee9qmb_rfkill_init);
module_exit(lbee9qmb_rfkill_exit);

MODULE_DESCRIPTION("Murata LBEE9QMBC rfkill");
MODULE_AUTHOR("NVIDIA");
MODULE_LICENSE("GPL");
