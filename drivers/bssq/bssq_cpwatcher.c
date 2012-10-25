
/*
 * B-qwerty - CP Watcher Driver
 *
 * Copyright (C) 2010 LGE, Inc.
 *
 *
 * modified: Wonseok Yang <ws.yang@lge.com>
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
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include <linux/syscalls.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <mach/hardware.h>
#include <mach/gpio-names.h>
#include <mach/hardware.h>
#include <linux/workqueue.h>
#include <linux/input.h>
#include <linux/slab.h>

#define GPIO_MDM_RESET_FLAG	TEGRA_GPIO_PV3  // modem reset flag
#define CPW_TEGRA_EDGE_TRIGGER

//#define CPW_DEBUG_MODE  //please don't touch
#ifdef CPW_DEBUG_MODE
#define CPW_DEBUG(format, args...) printk("[CPW] : %s (%d line): " format "\n", __FUNCTION__, __LINE__, ## args)
#else
#define CPW_DEBUG(format, args...) do { } while(0)
#endif

#define CPW_PRINTK(format, args...) printk("[CPW] : %s (%d line): " format "\n", __FUNCTION__, __LINE__, ## args)


/* Set scancode for hidden menu */
//#define EVENT_KEY KEY_RIL_RECOVERY /* 254 *///KEY_PROG3  /*key num is 202 */  
#define EVENT_KEY KEY_F24 /* 194 */

typedef struct  CpwatcherDeviceRec{
	struct input_dev *input;

	unsigned int mdm_reset_irq;
} CpwatcherDevice;

static CpwatcherDevice  s_cpwatcher;

static struct delayed_work work;

static struct platform_device cp_device = { 
		    .name       = "bssq_cpwatcher",
			    .id     = -1, 
};

extern int cp_reset_enable;

void disable_mdm_irq(void)
{

	CPW_PRINTK("MDM is going down..........\n");

	if (s_cpwatcher.mdm_reset_irq  >= 0) 
	{
		free_irq(s_cpwatcher.mdm_reset_irq, (void*)&s_cpwatcher);
		CPW_PRINTK("MDM is going down: Free irq for mdm reset...\n");
	}

}
//EXPORT_SYMBOL(disable_mdm_irq);

static ssize_t mdm_status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int pin_val = 0;

	pin_val = gpio_get_value(GPIO_MDM_RESET_FLAG);

	return sprintf(buf, "%s\n", ((pin_val)?"MDM On":"MDM Off"));
}

static ssize_t mdm_status_store(struct device *dev, struct device_attribute *sttr,
								const char *buf, size_t count)
{
	unsigned long state;
	int err;

	err = strict_strtoul(buf, 0, &state);
	if (err)
	{
		CPW_PRINTK(" err\n");	
		return err;
	}

	if (state == 0) 
	{
		disable_mdm_irq();
	}

	return err ?: count;
}

//20110621 ws.yang@lge.com 
/* /sys/devices/platform/bssq_cpwatcher/mdm_stat 에 대하여 Other 권한으로 writable하지 않아야 함. 
 0664 is read by user ..but 0666 is read or write by user */
static DEVICE_ATTR(mdm_stat, 0664, mdm_status_show, mdm_status_store);

static struct attribute *cpw_attributes[] = {
	&dev_attr_mdm_stat.attr,
	NULL,
};

static const struct attribute_group cpw_group = {
	.attrs = cpw_attributes,
};

static void mdm_reset_delayed_work(struct work_struct *wq)
{

	int nResetStatus = 0;
	int pin_val = 0;

	CPW_DEBUG("start.. \n");

	pin_val = gpio_get_value(GPIO_MDM_RESET_FLAG);
	if (pin_val == 0)
		CPW_PRINTK("pin_val = %d, Auto Reset or Crash by modem!!  \n", pin_val);

	if (pin_val == 0) 
	{
	        if (cp_reset_enable == 1)
	        {
   			 CPW_PRINTK("Do not key push during ril recovery!!! \n");        
	        }
		else
		{
		CPW_PRINTK( ">>>>MDM modem status pin makes interrupt  !!!!!\n");
		input_report_key(s_cpwatcher.input, EVENT_KEY, 1);
		input_report_key(s_cpwatcher.input, EVENT_KEY, 0);
		input_sync(s_cpwatcher.input);		
		CPW_PRINTK( "input_report_key(): %d\n", EVENT_KEY);
		}
		nResetStatus = 1; 
	} 
	else 
	{
		CPW_PRINTK( ">>>>MDM6600 was reset!\n");
	}

	CPW_DEBUG("end.. \n");	
}

static irqreturn_t mdm_reset_interrupt_handler(int irq, void *dev_id)
{
#ifdef CPW_TEGRA_EDGE_TRIGGER
	int pin_val = 0;
	
	CPW_DEBUG(" start \n");
	pin_val = gpio_get_value(GPIO_MDM_RESET_FLAG);
	if (pin_val == 1) //IRQF_TRIGGER_RISING case is ignore !!!
	{
		CPW_DEBUG("irq is ignored\n");
		return IRQ_HANDLED;
	}
#endif	
	schedule_delayed_work(&work, msecs_to_jiffies(5));

	CPW_DEBUG(" irq return is end \n");

	return IRQ_HANDLED;
}

static int __init cpw_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;	
	int err =0;

	CPW_DEBUG(" start !!! \n");

	memset(&s_cpwatcher, 0x00, sizeof(s_cpwatcher));

	/* Input */
	s_cpwatcher.input = input_allocate_device();
	if (!s_cpwatcher.input) 
	{
		CPW_PRINTK(" input_allocate_device  is error !!!\n");			
		goto err_input_device_register_fail;
	}

	s_cpwatcher.input->name = "cpwatcher";
	set_bit(EV_KEY, s_cpwatcher.input->evbit);
	set_bit(EV_SYN, s_cpwatcher.input->evbit);
	set_bit(EVENT_KEY, s_cpwatcher.input->keybit);

	err = input_register_device(s_cpwatcher.input);
	if (err) 
	{
		CPW_PRINTK(" input_register_device is error !!!\n");		
		goto err_input_register_device_fail;
	}
	
	INIT_DELAYED_WORK(&work, mdm_reset_delayed_work);  

	gpio_request(GPIO_MDM_RESET_FLAG, "mdm_reset_int_n");
	tegra_gpio_enable(GPIO_MDM_RESET_FLAG);
	gpio_direction_input(GPIO_MDM_RESET_FLAG);
	s_cpwatcher.mdm_reset_irq = gpio_to_irq(GPIO_MDM_RESET_FLAG);	

// cp에서 trap 발생시 TEGRA_GPIO_PV3를 high로 설정하여 AP에서 falling으로 인식하도록 처리
#ifdef CPW_TEGRA_EDGE_TRIGGER //nvidia dependency
	err = request_irq(s_cpwatcher.mdm_reset_irq, 
				     mdm_reset_interrupt_handler, 
                                     (IRQF_TRIGGER_FALLING|IRQF_TRIGGER_RISING), 
                                     "mdm_reset_int_n", 
                                     (void*)&s_cpwatcher);
#else
	err = request_irq(s_cpwatcher.mdm_reset_irq, 
				     mdm_reset_interrupt_handler, 
                                     IRQF_TRIGGER_FALLING, 
                                     "mdm_reset_int_n", 
                                     (void*)&s_cpwatcher);
#endif
	if(err)
	{
		CPW_PRINTK(" Failed: request_irq for mdm_reset_irq!!! (err:%d)\n", err);
		free_irq(s_cpwatcher.mdm_reset_irq, (void*)&s_cpwatcher);	
		return -ENOSYS;			
	}

	enable_irq_wake(s_cpwatcher.mdm_reset_irq);
	
	if (sysfs_create_group(&dev->kobj, &cpw_group)) 
	{

		CPW_PRINTK(" Failed to create sys filesystem\n");
		goto err_sysfs_create;		
	}

	CPW_DEBUG(" CP Watcher Initialization completed\n");

	return 0;

err_input_device_register_fail:
	input_free_device(s_cpwatcher.input);	
err_input_register_device_fail:
	input_unregister_device(s_cpwatcher.input); 
	s_cpwatcher.input = NULL;	
err_sysfs_create:
	return err;
}


static int cpw_remove(struct platform_device *pdev)
{
	 CPW_DEBUG("start\n");

	if (s_cpwatcher.mdm_reset_irq >= 0)
	{
		free_irq(s_cpwatcher.mdm_reset_irq, (void*)&s_cpwatcher);	 
	}

	if (s_cpwatcher.input) 
	{
	   input_unregister_device(s_cpwatcher.input);
	   input_free_device(s_cpwatcher.input);
	}      

	CPW_DEBUG(" success\n");

	return 0;
}


static struct platform_driver cpw_driver = {
	.probe		= cpw_probe,
	.remove		= __devexit_p(cpw_remove),
	.driver		= {
	.name		= "bssq_cpwatcher",
	.owner		= THIS_MODULE,
	},
};


static int __init cpw_init(void)
{
    CPW_DEBUG(" started\n");

     if (platform_device_register(&cp_device)) 
     {
     
     	CPW_PRINTK(" fail\n");
     }	

     return platform_driver_register(&cpw_driver);
}
module_init(cpw_init);
	
		 
static void __exit cpw_exit(void)
{
	platform_driver_unregister(&cpw_driver);
	CPW_DEBUG(" successed\n");
}
module_exit(cpw_exit);


MODULE_AUTHOR("LG Electronics");
MODULE_DESCRIPTION("B-qwerty CP Watcher Driver");
MODULE_LICENSE("GPL");


