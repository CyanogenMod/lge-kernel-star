/*
 * Charging IC driver (MAX8922L)
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

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/debugfs.h>
#include <linux/unistd.h>
#include <linux/slab.h>
#include <linux/time.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/hrtimer.h>
#include <linux/wakelock.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>

#include <linux/max8922l.h>	
#include <linux/su660_battery.h>

/* Debugging Functions */
#define CHG_DEBUG
asmlinkage int cprintk(const char *fmt, ...)
{
        va_list args;
        int r;
        va_start(args, fmt);
        r = vprintk(fmt, args);
        va_end(args);
        return r;
}
#ifdef  CHG_DEBUG 
#define DBG(fmt, arg...) cprintk("[CHARGER] : %s : " fmt "\n", __func__, ## arg)
#else
#define DBG(fmt, arg...) do {} while (0)
#endif

/* Charger String For Debug */
static const char *charger_ic_status_name[] = {
	"CHARGER_USB500",
	"CHARGER_ISET",
	"CHARGER_USB100",
	"CHARGER_FACTORY",
	"CHARGER_DISABLE"
};

/* Data Structure */
struct max8922l_data {
	charger_ic_status status;
	charger_ic_state_machine state;
	
	int gpio_en_set;
	int gpio_status;
	int gpio_pgb;
	
	struct delayed_work	int_work;
};

static struct max8922l_data* p_max8922l = NULL;
static bool is_enable = true;				// Needed?
static int max8922l_tp_boot = 0; 

/* Enable, Disable */
#define PULLUP		1
#define PULLDOWN	0

/* Delays */
#define	PULSE_DELAY	150	/* 100us < tHIGH < 1400us */
#define ENABLE_DELAY	250	/* 250us */
#define SET_DELAY	5	/* 4ms */
#define ON 1
#define OFF 0

/* Declare Extern Functions Here */
int read_gpio_en_set(void)
{
// MOBII_CHANGE_S [shhong@mobii.co.kr] 2012-04-27 : Kernel Panic Issue Solution.
	if(!p_max8922l)
		return 0;
// MOBII_CHANGE_E [shhong@mobii.co.kr] 2012-04-27 : Kernel Panic Issue Solution.

	return gpio_get_value(p_max8922l->gpio_en_set);
}
EXPORT_SYMBOL(read_gpio_en_set);

int read_gpio_status(void)
{
// MOBII_CHANGE_S [shhong@mobii.co.kr] 2012-04-27 : Kernel Panic Issue Solution.
	if(!p_max8922l)
		return 0;
// MOBII_CHANGE_E [shhong@mobii.co.kr] 2012-04-27 : Kernel Panic Issue Solution.

	return gpio_get_value(p_max8922l->gpio_status);
}
EXPORT_SYMBOL(read_gpio_status);

int read_gpio_pgb(void)
{
// MOBII_CHANGE_S [shhong@mobii.co.kr] 2012-04-27 : Kernel Panic Issue Solution.
	if(!p_max8922l)
		return 0;
// MOBII_CHANGE_E [shhong@mobii.co.kr] 2012-04-27 : Kernel Panic Issue Solution.

	return gpio_get_value(p_max8922l->gpio_pgb);
}
EXPORT_SYMBOL(read_gpio_pgb);

void charger_ic_disable(void)
{
	if(!p_max8922l)
		return;
	
	if( max8922l_tp_boot == 0 ) {	
		gpio_set_value(p_max8922l->gpio_en_set, PULLUP);
		mdelay(SET_DELAY);
		DBG("Charger Disabled.");
		p_max8922l->status = CHARGER_DISABLE;	
	}
}
EXPORT_SYMBOL(charger_ic_disable);

void charger_ic_disable_for_recharge(void)
	{
		if(!p_max8922l)
			return;
		
		if( max8922l_tp_boot == 0 ) {	
			gpio_set_value(p_max8922l->gpio_en_set, PULLUP);
			mdelay(SET_DELAY);
			DBG("Charger Disabled for recharge.");
		}
	}
EXPORT_SYMBOL(charger_ic_disable_for_recharge);

void charger_ic_set_mode(charger_ic_status mode)
{
	int i;
	unsigned long flags;
	
	if(!p_max8922l)
		return;
	
	/* Charger Setting Modified */
	if( max8922l_tp_boot == 0 ) {
		gpio_set_value(p_max8922l->gpio_en_set, PULLUP);
		mdelay(SET_DELAY);
		gpio_set_value(p_max8922l->gpio_en_set, PULLDOWN);
		mdelay(SET_DELAY);
	}

	local_irq_save(flags);
	for ( i = CHARGER_USB500; i < mode ; i++)
	{
		gpio_set_value(p_max8922l->gpio_en_set, PULLUP);
		udelay(PULSE_DELAY);
		gpio_set_value(p_max8922l->gpio_en_set, PULLDOWN);
		udelay(PULSE_DELAY);
	}
	local_irq_restore(flags);

	gpio_set_value(p_max8922l->gpio_en_set, PULLDOWN);
	mdelay(SET_DELAY);

	if( ((unsigned int)mode < 5) && ((unsigned int)mode >= 0) )
		DBG("Charger IC Enabled: [%s]", charger_ic_status_name[(unsigned int)mode]);
	p_max8922l->status = mode;	
	p_max8922l->state  = CHARGER_STATE_CHARGE;	
}
EXPORT_SYMBOL(charger_ic_set_mode);

/* State Machine Getter & Setter */
charger_ic_state_machine charger_ic_get_state(void)
{
	if(!p_max8922l)
		return CHARGER_STATE_SHUTDOWN;

	return p_max8922l->state;
}
EXPORT_SYMBOL(charger_ic_get_state);

charger_ic_state_machine charger_ic_set_state(charger_ic_state_machine state)
{
	if(!p_max8922l)
		return CHARGER_STATE_SHUTDOWN;

	p_max8922l->state = state;
}
EXPORT_SYMBOL(charger_ic_set_state);

/* Status Getter */
charger_ic_status charger_ic_get_status(void)
{
	if(!p_max8922l)
		return CHARGER_DISABLE;

	return p_max8922l->status;
}
EXPORT_SYMBOL(charger_ic_get_status);

void charger_ic_set_irq(bool enable)
{
// MOBII_CHANGE_S [shhong@mobii.co.kr] 2012-04-27 : Kernel Panic Issue Solution.
	if(!p_max8922l)
		return;
// MOBII_CHANGE_E [shhong@mobii.co.kr] 2012-04-27 : Kernel Panic Issue Solution.

	if(enable && !is_enable){
		is_enable = true;
		enable_irq(gpio_to_irq(p_max8922l->gpio_status));
		enable_irq_wake(gpio_to_irq(p_max8922l->gpio_status));
	} else if(!enable && is_enable) {
		is_enable = false;
		disable_irq(gpio_to_irq(p_max8922l->gpio_status));
		disable_irq_wake(gpio_to_irq(p_max8922l->gpio_status));

		cancel_delayed_work_sync(&p_max8922l->int_work);
	}
}
EXPORT_SYMBOL(charger_ic_set_irq);

/* Interrupt Procedure */
static irqreturn_t max8922l_interrupt_handler(int irq, void *data)
{
	DBG("Interrupted.");
	schedule_delayed_work(&p_max8922l->int_work, jiffies + msecs_to_jiffies(25));
	return IRQ_HANDLED;
}


/* Work Function */
static void max8922l_work_func(struct work_struct *work)
{
	determine_charger_state_with_charger_ic();
}

/* Determine Initial State */
enum {
	CHGSB_PGB_OFF_OFF = 0,
	CHGSB_PGB_ON_ON,
	CHGSB_PGB_OFF_ON,
	CHGSB_PGB_ON_OFF,
};
static void max8922l_set_initial_state(void)
{
	int status, pgb;
	int charger_state;

	if(p_max8922l == NULL)
		return;
	
	pgb    = gpio_get_value(p_max8922l->gpio_pgb);	
	status = gpio_get_value(p_max8922l->gpio_status);	
	
	if ((status == 1) && (pgb == 1))
		charger_state = CHGSB_PGB_OFF_OFF;
	else if ((status == 0)&& (pgb == 0))
		charger_state = CHGSB_PGB_ON_ON;
	else if ((status == 1)&& (pgb == 0))
		charger_state = CHGSB_PGB_OFF_ON;
	else if ((status == 0)&& (pgb == 1))
		charger_state = CHGSB_PGB_ON_OFF;

	switch (charger_state) {
		case CHGSB_PGB_OFF_OFF:
			p_max8922l->state = CHARGER_STATE_SHUTDOWN;
			p_max8922l->status = CHARGER_DISABLE;
			break;
		case CHGSB_PGB_ON_ON:
			p_max8922l->state = CHARGER_STATE_CHARGE;
			p_max8922l->status = CHARGER_ISET;
			break;
		case CHGSB_PGB_OFF_ON:
			if ( max8922l_tp_boot == 0 ) {
				p_max8922l->state = CHARGER_STATE_CHARGE;
				p_max8922l->status = CHARGER_FACTORY;
			} else if ( max8922l_tp_boot == 1 ) {
				p_max8922l->state = CHARGER_STATE_STANDBY;
				p_max8922l->status = CHARGER_FACTORY;
			}
			break;
		default:
			p_max8922l->state = CHARGER_STATE_SHUTDOWN;
			p_max8922l->status = CHARGER_DISABLE;
			break;
	} // End switch ( charger_state )...
}

/* Basic Driver Function */
static int max8922l_probe(struct platform_device *pdev)
{
	struct charger_ic_platform_data *pdata = pdev->dev.platform_data;
	struct max8922l_data *max8922l;
	int ret = 0;

	DBG();

	if(!pdata) {
		ret = -EBUSY;
		goto err;
	}

	max8922l = kzalloc(sizeof(struct max8922l_data), GFP_KERNEL);
	if(!max8922l) {
		ret = -ENOMEM;
		goto err;
	}	

	/* GPIO Assign Here */
	max8922l->gpio_en_set	= pdata->gpio_en_set;
	max8922l->gpio_status	= pdata->gpio_status;
	max8922l->gpio_pgb	= pdata->gpio_pgb;

	/* Init Interrupt */
	ret = request_irq(gpio_to_irq(pdata->gpio_status),
				max8922l_interrupt_handler,
				pdata->irqflags,
				"Charging_ic_driver", NULL);
	if(ret < 0) {
		DBG("Interrupt Register failure.");
		goto err_gpio_status_irq;
	}
	DBG("Interrupt Registered.");

	/* Initialize Workqueue & Wakelock*/
	INIT_DELAYED_WORK(&max8922l->int_work, max8922l_work_func);

	enable_irq_wake(gpio_to_irq(pdata->gpio_status));
	
	p_max8922l = max8922l;

	/* Set Charger initial State/Status */
	max8922l_set_initial_state();

	DBG("MAX8922L Charger IC Driver probe finished.");
	return 0;

err_gpio_status_irq:
	gpio_free(pdata->gpio_status);
	gpio_free(pdata->gpio_pgb);
	gpio_free(pdata->gpio_en_set);
err:
	return ret;

}
static int max8922l_remove(struct platform_device *pdev)
{
	struct charger_ic_platform_data *pdata = pdev->dev.platform_data;
	DBG();

	/* Disable Charger IC */
	charger_ic_disable();	

	free_irq(gpio_to_irq(pdata->gpio_status), NULL);
	
	gpio_free(pdata->gpio_status);
	gpio_free(pdata->gpio_pgb);
	gpio_free(pdata->gpio_en_set);
	
	return 0;
}
static int max8922l_suspend(struct platform_device *pdev, pm_message_t state)
{
	DBG();
//LP0 sleep during TA Charging	
	pdev->dev.power.power_state = state;
	return 0;
}
static int max8922l_resume(struct platform_device *pdev)
{
	DBG();
//LP0 sleep during TA Charging	
	pdev->dev.power.power_state = PMSG_ON;
	return 0;
}

/* Platform Driver Structure */
static struct platform_driver max8922l_driver = {
	.probe		= max8922l_probe,
	.remove		= max8922l_remove,
	.suspend	= max8922l_suspend,
	.resume		= max8922l_resume,
	.driver		= {
		.name = "charger_ic_max8922l",
	},
};
static int __init max8922l_init(void)
{
	DBG();
	return platform_driver_register(&max8922l_driver);
}
static int __init max8922l_tp_boot_state(char *str)
{
	int tp_boot = (int) simple_strtol(str, NULL, 0);
	if( tp_boot == 1 ) 
		max8922l_tp_boot = 1;

	return 1;
}
__setup("max8922l_tp_boot=", max8922l_tp_boot_state);
static void __exit max8922l_exit(void)
{
	DBG();
	platform_driver_unregister(&max8922l_driver);
}
//module_init(max8922l_init);
subsys_initcall(max8922l_init);
module_exit(max8922l_exit);

MODULE_AUTHOR("sanghyun.hong@lge.com");
MODULE_DESCRIPTION("MAX8922L Charger IC Driver");
MODULE_LICENSE("GPL");
