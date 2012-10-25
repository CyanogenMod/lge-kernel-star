/*
 * drivers/misc/bcm4329_rfkill.c
 *
 * Copyright (c) 2010, NVIDIA Corporation.
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

#include <linux/err.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/rfkill.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/slab.h>
#include <linux/delay.h>

//LGE_CHANGE_S [munho2.lee@lge.com] 2012-02-06 for Bluetooth bring-up
#include <linux/bcm4329-rfkill.h>

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
//LGE_CHANGE_E [munho2.lee@lge.com]

//LGE_CHANGE_S [munho2.lee@lge.com] 2012-02-06 for Bluetooth bring-up
/*
struct bcm4329_rfkill_data {
	int gpio_reset;
	int gpio_shutdown;
	int delay;
	struct clk *bt_32k_clk;
};
*/
//LGE_CHANGE_E

static struct bcm4329_rfkill_data *bcm4329_rfkill;

static int bcm4329_bt_rfkill_set_power(void *data, bool blocked)
{
	if (blocked) {
		//LGE_CHANGE_S [munho2.lee@lge.com] 2012-02-06 for Bluetooth bring-up
		/*
		if (bcm4329_rfkill->gpio_shutdown)
			gpio_direction_output(bcm4329_rfkill->gpio_shutdown, 0);
		*/
		//LGE_CHANGE_E [munho2.lee@lge.com]
		if (bcm4329_rfkill->gpio_reset)
			gpio_direction_output(bcm4329_rfkill->gpio_reset, 0);
//		if (bcm4329_rfkill->bt_32k_clk)
//			clk_disable(bcm4329_rfkill->bt_32k_clk);
		//LGE_CHANGE_S [munho2.lee@lge.com] 2012-02-06 for Bluetooth bring-up
		bt_lpm.bt_enable=0;
		//LGE_CHANGE_E [munho2.lee@lge.com]
	} else {
//		if (bcm4329_rfkill->bt_32k_clk)
//			clk_enable(bcm4329_rfkill->bt_32k_clk);
		//LGE_CHANGE_S [munho2.lee@lge.com] 2012-02-06 for Bluetooth bring-up
		/*
		if (bcm4329_rfkill->gpio_shutdown)
		{
			gpio_direction_output(bcm4329_rfkill->gpio_shutdown, 0);
			msleep(100);
			gpio_direction_output(bcm4329_rfkill->gpio_shutdown, 1);
			msleep(100);
		}
		*/
		//LGE_CHANGE_E [munho2.lee@lge.com]		
		if (bcm4329_rfkill->gpio_reset)
		{
			gpio_direction_output(bcm4329_rfkill->gpio_reset, 0);
			msleep(100);
			gpio_direction_output(bcm4329_rfkill->gpio_reset, 1);
			msleep(100);
			//LGE_CHANGE_S [munho2.lee@lge.com] 2012-02-06 for Bluetooth bring-up
			bt_lpm.bt_enable=1;
			//LGE_CHANGE_E [munho2.lee@lge.com]
		}
	}

	return 0;
}

static const struct rfkill_ops bcm4329_bt_rfkill_ops = {
	.set_block = bcm4329_bt_rfkill_set_power,
};

static int bcm4329_rfkill_probe(struct platform_device *pdev)
{
	struct rfkill *bt_rfkill;
	struct resource *res;
	int ret;
	bool enable = false;  /* off */
	bool default_sw_block_state;

	bcm4329_rfkill = kzalloc(sizeof(*bcm4329_rfkill), GFP_KERNEL);
	if (!bcm4329_rfkill)
		return -ENOMEM;

	bcm4329_rfkill->bt_32k_clk = clk_get(&pdev->dev, "bcm4329_32k_clk");
	if (IS_ERR(bcm4329_rfkill->bt_32k_clk)) {
		pr_warn("%s: can't find bcm4329_32k_clk.\
				assuming 32k clock to chip\n", __func__);
		bcm4329_rfkill->bt_32k_clk = NULL;
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_IO,
						"bcm4329_nreset_gpio");
	if (res) {
		bcm4329_rfkill->gpio_reset = res->start;
		tegra_gpio_enable(bcm4329_rfkill->gpio_reset);
		ret = gpio_request(bcm4329_rfkill->gpio_reset,
						"bcm4329_nreset_gpio");
	} else {
		pr_warn("%s : can't find reset gpio.\n", __func__);
		bcm4329_rfkill->gpio_reset = 0;
	}

	//LGE_CHANGE_S [munho2.lee@lge.com] 2012-02-06 for Bluetooth bring-up
	/*
	res = platform_get_resource_byname(pdev, IORESOURCE_IO,
						"bcm4329_nshutdown_gpio");
	if (res) {
		bcm4329_rfkill->gpio_shutdown = res->start;
		tegra_gpio_enable(bcm4329_rfkill->gpio_shutdown);
		ret = gpio_request(bcm4329_rfkill->gpio_shutdown,
						"bcm4329_nshutdown_gpio");
	} else {
		pr_warn("%s : can't find shutdown gpio.\n", __func__);
		bcm4329_rfkill->gpio_shutdown = 0;
	}
	*/
	//LGE_CHANGE_E [munho2.lee@lge.com]
	
	//LGE_CHANGE_S [munho2.lee@lge.com] 2012-02-06 for Bluetooth bring-up
	/* make sure at-least one of the GPIO is defined */
	if (!bcm4329_rfkill->gpio_reset)
		goto free_bcm_res;
	/*  - origianl -
	if (!bcm4329_rfkill->gpio_reset && !bcm4329_rfkill->gpio_shutdown)
		goto free_bcm_res;
	*/
	//LGE_CHANGE_E [munho2.lee@lge.com]
	if (bcm4329_rfkill->bt_32k_clk && enable)
		clk_enable(bcm4329_rfkill->bt_32k_clk);
	//LGE_CHANGE_S [munho2.lee@lge.com] 2012-02-06 for Bluetooth bring-up
	/*
	if (bcm4329_rfkill->gpio_shutdown)
		gpio_direction_output(bcm4329_rfkill->gpio_shutdown, enable);
	*/
	//LGE_CHANGE_E [munho2.lee@lge.com]		
	if (bcm4329_rfkill->gpio_reset)
		gpio_direction_output(bcm4329_rfkill->gpio_reset, enable);

	bt_rfkill = rfkill_alloc("bcm4329 Bluetooth", &pdev->dev,
				RFKILL_TYPE_BLUETOOTH, &bcm4329_bt_rfkill_ops,
				NULL);

	if (unlikely(!bt_rfkill))
		goto free_bcm_res;

	default_sw_block_state = !enable;
	rfkill_set_states(bt_rfkill, default_sw_block_state, false);

	ret = rfkill_register(bt_rfkill);

	if (unlikely(ret)) {
		rfkill_destroy(bt_rfkill);
		goto free_bcm_res;
	}

	return 0;

free_bcm_res:
	//LGE_CHANGE_S [munho2.lee@lge.com] 2012-02-06 for Bluetooth bring-up
	/*	
	if (bcm4329_rfkill->gpio_shutdown)
		gpio_free(bcm4329_rfkill->gpio_shutdown);
	*/		
	//LGE_CHANGE_E [munho2.lee@lge.com]
	if (bcm4329_rfkill->gpio_reset)
		gpio_free(bcm4329_rfkill->gpio_reset);
	if (bcm4329_rfkill->bt_32k_clk && enable)
		clk_disable(bcm4329_rfkill->bt_32k_clk);
	if (bcm4329_rfkill->bt_32k_clk)
		clk_put(bcm4329_rfkill->bt_32k_clk);
	kfree(bcm4329_rfkill);
	return -ENODEV;
}

static int bcm4329_rfkill_remove(struct platform_device *pdev)
{
	struct rfkill *bt_rfkill = platform_get_drvdata(pdev);

	if (bcm4329_rfkill->bt_32k_clk)
		clk_put(bcm4329_rfkill->bt_32k_clk);
	rfkill_unregister(bt_rfkill);
	rfkill_destroy(bt_rfkill);
	//LGE_CHANGE_S [munho2.lee@lge.com] 2012-02-06 for Bluetooth bring-up
	/*
	if (bcm4329_rfkill->gpio_shutdown)
		gpio_free(bcm4329_rfkill->gpio_shutdown);
	*/
	//LGE_CHANGE_E [munho2.lee@lge.com]
	if (bcm4329_rfkill->gpio_reset)
		gpio_free(bcm4329_rfkill->gpio_reset);
	kfree(bcm4329_rfkill);

	return 0;
}

static struct platform_driver bcm4329_rfkill_driver = {
	.probe = bcm4329_rfkill_probe,
	.remove = bcm4329_rfkill_remove,
	.driver = {
		   .name = "bcm4329_rfkill",
		   .owner = THIS_MODULE,
	},
};

//LGE_CHANGE_S [munho2.lee@lge.com] 2012-02-06 for Bluetooth bring-up
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
	irq_set_irq_type(bt_lpm.host_wake_irq, host_wake ? IRQF_TRIGGER_LOW : IRQF_TRIGGER_HIGH);
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
static int bcm4329_rfkill_set_btwake(void *data, bool blocked)
{
	struct platform_device *pdev = data;
	struct bcm4329_rfkill_data *plat = pdev->dev.platform_data;

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

static struct rfkill_ops bcm4329_rfkill_btwake_ops = {
	.set_block = bcm4329_rfkill_set_btwake,
};


static int bcm4329_rfkill_btwake_probe(struct platform_device *pdev)
{
	struct bcm4329_rfkill_data *plat = pdev->dev.platform_data;
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

	rc = gpio_request(plat->gpio_btwake, "bcm4329_reset_btwake");
	if (rc < 0) {
		dev_err(&pdev->dev, "gpio_request failed\n");
		return rc;
	}

	rfkill = rfkill_alloc("bcm4329-rfkill_btwake", &pdev->dev,
			RFKILL_TYPE_BLUETOOTH, &bcm4329_rfkill_btwake_ops, pdev);
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
	rc = gpio_request(plat->gpio_hostwake, "bcm4329_reset_hostwake");
	gpio_direction_input(plat->gpio_hostwake);
	host_wake=gpio_get_value(bt_lpm.gpio_host_wake);
	irq = gpio_to_irq(plat->gpio_hostwake);
	bt_lpm.host_wake_irq=irq;
#ifdef BRCM_WAKELOCKTIMEOUT
	hrtimer_init(&bt_lpm.check_hostwakeup_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	bt_lpm.check_hostwakeup_delay = ktime_set(5, 0);  /* 5 sec */
	bt_lpm.check_hostwakeup_timer.function = check_hostwakeup;

	irq_set_irq_type(irq, IRQ_TYPE_EDGE_RISING);
	
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

static int bcm4329_rfkill_btwake_remove(struct platform_device *pdev)
{
	struct bcm4329_rfkill_data *plat = pdev->dev.platform_data;
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

static struct platform_driver bcm4329_rfkill_btwake_driver = {
	.probe = bcm4329_rfkill_btwake_probe,
	.remove = bcm4329_rfkill_btwake_remove,
	.driver = {
		.name = "bcm4329-rfkill_btwake",
		.owner = THIS_MODULE,
	},
};
#endif
//LGE_CHANGE_E [munho2.lee@lge.com]


static int __init bcm4329_rfkill_init(void)
{
//LGE_CHANGE_S [munho2.lee@lge.com] 2012-02-06 for Bluetooth bring-up
#ifdef BRCM_BT_WAKE
		platform_driver_register(&bcm4329_rfkill_driver);
		return platform_driver_register(&bcm4329_rfkill_btwake_driver);
#else
	return platform_driver_register(&bcm4329_rfkill_driver);
#endif
//LGE_CHANGE_E [munho2.lee@lge.com]	
}

static void __exit bcm4329_rfkill_exit(void)
{
//LGE_CHANGE_S [munho2.lee@lge.com] 2012-02-06 for Bluetooth bring-up
#ifdef BRCM_BT_WAKE
		platform_driver_unregister(&bcm4329_rfkill_btwake_driver);
#endif
//LGE_CHANGE_E [munho2.lee@lge.com]
	platform_driver_unregister(&bcm4329_rfkill_driver);
}

module_init(bcm4329_rfkill_init);
module_exit(bcm4329_rfkill_exit);

MODULE_DESCRIPTION("BCM4329 rfkill");
MODULE_AUTHOR("NVIDIA");
MODULE_LICENSE("GPL");
