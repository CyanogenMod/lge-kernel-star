/*
 * Hub Charging IC driver (rt9524)
 *
 * Copyright (C) 2011 LGE, Inc.
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

#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <mach/gpio.h>
#include <linux/delay.h>
#include <linux/hrtimer.h>
#include <linux/wakelock.h>
#include <linux/slab.h>
#include "../staging/android//timed_output.h"
#include <lge/bssq_charger_rt.h>
#include <linux/notifier.h>
#if defined(CONFIG_BSSQ_MUIC_TI)
#include "bssq_muic_ti.h"
#endif
#include <linux/lge_hw_rev.h>

struct rt9524_data {
	charger_ic_status status;

	int	gpio_en_set;
	int gpio_pgb;
	int gpio_status;

	struct delayed_work	int_work;
	struct wake_lock 	power_off_charging_lock;
};

static struct rt9524_data* p_rt9524 = NULL;
static bool is_enable = true;

/*
 * mdelay() and udelay() values for when setting charger mode
 */
#define PULSE_DELAY			110	/* 100us < tHigh < 700us */
#define SET_DELAY			2	/* 1.5ms */
#define ENABLE_DELAY		32	/* 32ms */

void charger_ic_disable(void)
{
	if(!p_rt9524)
		return;

	gpio_set_value(p_rt9524->gpio_en_set, 1);
	mdelay(SET_DELAY);

	p_rt9524->status = CHARGER_DISBALE;

	printk(KERN_DEBUG "[CHARGER] %s : rt9524_status = CHARGER_DISBALE\n",__func__);
}
EXPORT_SYMBOL(charger_ic_disable);


void charger_ic_set_mode(charger_ic_status mode)
{
	int i;
	unsigned long flags;

	if(!p_rt9524)
		return;

	if(gpio_get_value(p_rt9524->gpio_en_set) == 1)
	{	// previous disable state
		gpio_set_value(p_rt9524->gpio_en_set, 0);	// enable & set RT9524_USB500
		msleep(ENABLE_DELAY);
	}
	else
	{	// previous enable state
		if(mode != CHARGER_FACTORY) {
			gpio_set_value(p_rt9524->gpio_en_set, 1);
			msleep(SET_DELAY);
		}
		gpio_set_value(p_rt9524->gpio_en_set, 0);	// enable & set RT9524_USB500
		msleep(ENABLE_DELAY);
	}

	local_irq_save(flags);
	for ( i = CHARGER_USB500 ; i < mode ; i++)
	{
		gpio_set_value(p_rt9524->gpio_en_set, 0);
		udelay(PULSE_DELAY);
		gpio_set_value(p_rt9524->gpio_en_set, 1);
		udelay(PULSE_DELAY);
	}
	local_irq_restore(flags);

	gpio_set_value(p_rt9524->gpio_en_set, 0);
	mdelay(SET_DELAY);

	p_rt9524->status = mode;

	printk(KERN_DEBUG "[CHARGER] %s : rt9524_status = %d\n",__func__, mode);
}
EXPORT_SYMBOL(charger_ic_set_mode);

charger_ic_status charger_ic_get_status(void)
{
//	printk(KERN_DEBUG "[CHARGER] %s : rt9524_status = %d\n",__func__, p_rt9524->status);
	if(!p_rt9524)
		return CHARGER_DISBALE;

//	printk(KERN_DEBUG "[CHARGER] CHG_STATUS_N(%d) , CHG_PGB_N(%d) \n", gpio_get_value(p_rt9524->gpio_status), gpio_get_value(p_rt9524->gpio_pgb));
	return p_rt9524->status;
}

EXPORT_SYMBOL(charger_ic_get_status);

void charger_ic_set_irq(bool enable)
{
	if(enable && !is_enable){
		is_enable = true;
		enable_irq(gpio_to_irq(p_rt9524->gpio_status));
		enable_irq_wake(gpio_to_irq(p_rt9524->gpio_status));
	}else if(!enable && is_enable) {
		is_enable = false;
		disable_irq(gpio_to_irq(p_rt9524->gpio_status));
		disable_irq_wake(gpio_to_irq(p_rt9524->gpio_status));
	}
}
EXPORT_SYMBOL(charger_ic_set_irq);

static irqreturn_t rt9524_interrupt_handler(int irq, void *data)
{
    if (p_rt9524->status != CHARGER_DISBALE) {
		if(!gpio_get_value(p_rt9524->gpio_pgb) && gpio_get_value(p_rt9524->gpio_status)) {
			printk(KERN_DEBUG "[CHARGER] %s ",__func__);
	    	schedule_delayed_work(&p_rt9524->int_work, msecs_to_jiffies(500));
    	}
    }
	return IRQ_HANDLED;
}

#if defined (CONFIG_BSSQ_BATTERY)
extern void notification_of_changes_to_battery(void);	// from twl4030_bci_battery.c
extern int set_end_of_charge(int complete);	// from twl4030_bci_battery.c
extern void battery_setEOCVoltage(void);
#endif

static void rt9524_work_func(struct work_struct *work)
{
	if(p_rt9524->status != CHARGER_DISBALE) {
		if(!gpio_get_value(p_rt9524->gpio_pgb) && gpio_get_value(p_rt9524->gpio_status)) {
			printk(KERN_DEBUG "[CHARGER] %s - EOC !!!!! \n",__func__);
			charger_ic_set_irq(0);
#if defined (CONFIG_BSSQ_BATTERY)
			set_end_of_charge(1);
			battery_setEOCVoltage();
#endif
			charger_ic_disable();
#if defined (CONFIG_BSSQ_BATTERY)
			notification_of_changes_to_battery();
#endif
		}
	}
}

ssize_t rt9524_show_status(struct device *dev,
			 struct device_attribute *attr,
			 char *buf)
{
	if(p_rt9524->status == CHARGER_DISBALE)
		return snprintf(buf, PAGE_SIZE, "0\n");
	else
		return snprintf(buf, PAGE_SIZE, "1\n");
}
ssize_t rt9524_store_status(struct device *dev,
			  struct device_attribute *attr,
			  const char *buf,
			  size_t count)
{
	if(buf[0] == '0') {
		charger_ic_disable();
	} else if(buf[0] == '1') {
		charger_ic_set_mode(CHARGER_ISET);
	}
	return count;
}
DEVICE_ATTR(rt9524_state, 0644, rt9524_show_status, rt9524_store_status);

// 20110903 hg.park@lge.com CTS File Permission
ssize_t rt9524_show_status_enable(struct device *dev,
			 struct device_attribute *attr,
			 char *buf)
{
	charger_ic_set_mode(CHARGER_ISET);
	return snprintf(buf, PAGE_SIZE, "1\n");
}


DEVICE_ATTR(rt9524_state_enable, 0644, rt9524_show_status_enable, NULL);

ssize_t rt9524_show_status_disable(struct device *dev,
			 struct device_attribute *attr,
			 char *buf)
{
	charger_ic_disable();
	return snprintf(buf, PAGE_SIZE, "1\n");
}

DEVICE_ATTR(rt9524_state_disable, 0644, rt9524_show_status_disable, NULL);

ssize_t rt9524_show_poc(struct device *dev,
			 struct device_attribute *attr,
			 char *buf)
{
	int res;

	res = wake_lock_active(&p_rt9524->power_off_charging_lock);

	if(res)
		return snprintf(buf, PAGE_SIZE, "1\n");
	return snprintf(buf, PAGE_SIZE, "0\n");

}

//extern int console_enabled;	for factory : to be added

ssize_t rt9524_store_poc(struct device *dev,
			  struct device_attribute *attr,
			  const char *buf,
			  size_t count)
{
	if(buf[0] == '0') {
//		if(console_enabled && wake_lock_active(&power_off_charging_lock))
		if(wake_lock_active(&p_rt9524->power_off_charging_lock))
			wake_unlock(&p_rt9524->power_off_charging_lock);
		printk("[CHARGER] Power Off Charging - End!\n");
	} else if(buf[0] == '1') {
//		if(console_enabled && !wake_lock_active(&power_off_charging_lock))
		if(!wake_lock_active(&p_rt9524->power_off_charging_lock))
			wake_lock(&p_rt9524->power_off_charging_lock);
		printk("[CHARGER] Power Off Charging - Start!\n");
	}
	return count;
}
DEVICE_ATTR(power_off_charging, 0644, rt9524_show_poc, rt9524_store_poc);

static int rt9524_probe(struct platform_device *dev)
{
	struct charger_rt_platform_data *pdata = dev->dev.platform_data;
	struct rt9524_data *rt9524;
	int ret = 0;

	if (!pdata) {
		ret = -EBUSY;
		goto err;
	}

	rt9524 = kzalloc(sizeof(struct rt9524_data), GFP_KERNEL);
	if (!rt9524) {
		ret = -ENOMEM;
		goto err;
	}

	tegra_gpio_enable(pdata->gpio_en_set);
	ret = gpio_request(pdata->gpio_en_set, "rt9524_en");
	if (ret < 0) {
		printk(KERN_ERR "%s: Failed to request GPIO_%d for "
				"rt9524\n", __func__, pdata->gpio_en_set);
		ret = -ENOSYS;
		goto err_gpio_en_set_request;
	}
	gpio_direction_output(pdata->gpio_en_set, 0);

	tegra_gpio_enable(pdata->gpio_pgb);
	ret = gpio_request(pdata->gpio_pgb, "rt9524_pwr_status");
	if (ret < 0) {
		printk(KERN_ERR "%s: Failed to request GPIO_%d for "
				"rt9524_pwr_status\n", __func__,
				pdata->gpio_pgb);
		goto err_gpio_pgb_request;
	}
	gpio_direction_input(pdata->gpio_pgb);

	tegra_gpio_enable(pdata->gpio_status);
	ret = gpio_request(pdata->gpio_status, "rt9524_status");
	if (ret < 0) {
		printk(KERN_ERR "%s: Failed to request GPIO_%d for "
				"rt9524_status\n", __func__,
				pdata->gpio_status);
		goto err_gpio_status_request;
	}
	gpio_direction_input(pdata->gpio_status);

	ret = request_irq(gpio_to_irq(pdata->gpio_status),
			rt9524_interrupt_handler,
			pdata->irqflags,
			"Charging_ic_driver", NULL);
	if (ret < 0) {
		printk(KERN_ERR "%s: Failed to request IRQ for "
				"rt9524_status\n", __func__);
		goto err_gpio_status_irq;
	}
	printk("[CHARGER] interrupt register \n");

	INIT_DELAYED_WORK(&rt9524->int_work,rt9524_work_func);

	wake_lock_init(&rt9524->power_off_charging_lock, WAKE_LOCK_SUSPEND, "Power Off Charging");

	// for AT Command AT%CHARGE
	// sysfs path : /sys/devices/platform/charger_ic_rt9524/charging_state
	ret = device_create_file(&dev->dev, &dev_attr_rt9524_state);
	if (ret < 0) {
		pr_err("%s:File device creation failed: %d\n", __func__, ret);
		ret = -ENODEV;
		goto err_create_state_file;
	}

	ret = device_create_file(&dev->dev, &dev_attr_rt9524_state_enable);
	if (ret < 0) {
		pr_err("%s:File device creation failed: %d\n", __func__, ret);
		ret = -ENODEV;
		goto err_create_state_enable_file;
	}

	ret = device_create_file(&dev->dev, &dev_attr_rt9524_state_disable);
	if (ret < 0) {
		pr_err("%s:File device creation failed: %d\n", __func__, ret);
		ret = -ENODEV;
		goto err_create_state_disable_file;
	}


	// for Power Off Charging
	// sysfs path : /sys/devices/platform/charger_ic_rt9524/power_off_charging
	ret = device_create_file(&dev->dev, &dev_attr_power_off_charging);
	if (ret < 0) {
		pr_err("%s:File device creation failed: %d\n", __func__, ret);
		ret = -ENODEV;
		goto err_create_power_off_charging;
	}

	enable_irq_wake(gpio_to_irq(pdata->gpio_status));
	printk("test : pdata->gpio_status : %d \n", pdata->gpio_status);

	rt9524->gpio_en_set = pdata->gpio_en_set;
	rt9524->gpio_pgb = pdata->gpio_pgb;
	rt9524->gpio_status = pdata->gpio_status;

	p_rt9524 = rt9524;

	printk("rt9524_probe() is OK!\n");
	return 0;

err_create_power_off_charging:
	device_remove_file(&dev->dev, &dev_attr_rt9524_state_disable);	
err_create_state_disable_file:
	device_remove_file(&dev->dev, &dev_attr_rt9524_state_enable);
err_create_state_enable_file:	
	device_remove_file(&dev->dev, &dev_attr_rt9524_state);	
err_create_state_file:
	free_irq(pdata->gpio_status, NULL);
err_gpio_status_irq:
	gpio_free(pdata->gpio_status);
err_gpio_status_request:
	gpio_free(pdata->gpio_pgb);
err_gpio_pgb_request:
	gpio_free(pdata->gpio_en_set);
err_gpio_en_set_request:
	kfree(rt9524);
err:
	return ret;
}

static int rt9524_remove(struct platform_device *dev)
{
	struct charger_rt_platform_data *pdata = dev->dev.platform_data;

	charger_ic_disable();
	device_remove_file(&dev->dev, &dev_attr_rt9524_state);
	device_remove_file(&dev->dev, &dev_attr_rt9524_state_enable);
	device_remove_file(&dev->dev, &dev_attr_rt9524_state_disable);
	device_remove_file(&dev->dev, &dev_attr_power_off_charging);

	free_irq(pdata->gpio_en_set, NULL);

	gpio_free(pdata->gpio_status);
	gpio_free(pdata->gpio_pgb);
	gpio_free(pdata->gpio_en_set);

	return 0;
}

static int rt9524_suspend(struct platform_device *dev, pm_message_t state)
{
	dev->dev.power.power_state = state;
	return 0;
}

static int rt9524_resume(struct platform_device *dev)
{
	dev->dev.power.power_state = PMSG_ON;
	return 0;
}

static struct platform_driver rt9524_driver = {
	.probe = rt9524_probe,
	.remove = rt9524_remove,
	.suspend = rt9524_suspend,
	.resume= rt9524_resume,
	.driver = {
		.name = "charger_ic_rt9524",
	},
};

static int __init rt9524_init(void)
{
	return platform_driver_register(&rt9524_driver);
}

static void __exit rt9524_exit(void)
{
	platform_driver_unregister(&rt9524_driver);
}

module_init(rt9524_init);
module_exit(rt9524_exit);

MODULE_AUTHOR("LG Electronics");
MODULE_DESCRIPTION("rt9524 Driver");
MODULE_LICENSE("GPL");

