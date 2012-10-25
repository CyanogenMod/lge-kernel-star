
/*
 * X2 - CP Watcher Driver
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


#define IFX_CP_CRASH
#ifdef IFX_CP_CRASH
#define GPIO_IFX_CP_CRASH       TEGRA_GPIO_PR1
#endif

#define GPIO_IFX_CP_RESET       TEGRA_GPIO_PV3

#define CPW_IFX_TEGRA_EDGE_TRIGGER

//#define CPW_DEBUG_MODE
#ifdef CPW_DEBUG_MODE
#define CPW_DEBUG(format, args...) printk("[CPW] : %s (%d line): " format "\n", __FUNCTION__, __LINE__, ## args)
#else
#define CPW_DEBUG(format, args...) do { } while(0)
#endif

#define CPW_PRINTK(format, args...) printk("[CPW] : %s (%d line): " format "\n", __FUNCTION__, __LINE__, ## args)

/* Forward scancode to Framework   */
/* frameworks\base\policy\src\com\android\internal\policy\impl\PhoneWindowManager.java */
#define EVENT_KEY KEY_F24 /* 194 */


#define CP_RESET_TRUE       0
#define CP_CRASH_TRUE       1

typedef struct  CpwatcherDeviceRec{
    struct input_dev *input;

    unsigned int ifx_cp_reset_irq;    
#ifdef IFX_CP_CRASH  
    unsigned int ifx_cp_crash_irq;
#endif  
} CpwatcherDevice;

static CpwatcherDevice  s_cpwatcher;

static struct delayed_work work;

static struct platform_device cp_device = { 
            .name       = "cpwatcher",
            .id         = -1,
};

void disable_ifx_irq(void)
{

    CPW_PRINTK("IFX modem is going down..........\n");

    if (s_cpwatcher.ifx_cp_reset_irq  >= 0) 
    {
        free_irq(s_cpwatcher.ifx_cp_reset_irq, (void*)&s_cpwatcher);
        CPW_PRINTK("IFX modem is going down: Free irq for cp reset...\n");
    }
#ifdef IFX_CP_CRASH
    if (s_cpwatcher.ifx_cp_crash_irq  >= 0) 
    {
        free_irq(s_cpwatcher.ifx_cp_crash_irq, (void*)&s_cpwatcher);
        CPW_PRINTK("IFX modem is going down: Free irq for cp crash or trap...\n");
    }   
#endif
}
//EXPORT_SYMBOL(disable_ifx_irq);

static ssize_t ifx_status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int pin_val = 0;

    pin_val = gpio_get_value(GPIO_IFX_CP_RESET);

    return sprintf(buf, "%s\n", ((pin_val)?"IFX On":"IFX Off"));
}

static ssize_t ifx_status_store(struct device *dev, struct device_attribute *sttr,
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
        disable_ifx_irq();
    }

    return err ?: count;
}

//20110621 ws.yang@lge.com 
/* /sys/devices/platform/cpwatcher/ifx_stat 
 0664 is read by user ..but 0666 is read or write by user */
static DEVICE_ATTR(ifx_stat, 0664, ifx_status_show, ifx_status_store);

static struct attribute *cpw_attributes[] = {
    &dev_attr_ifx_stat.attr,
    NULL,
};

static const struct attribute_group cpw_group = {
    .attrs = cpw_attributes,
};

static void ifx_reset_delayed_work(struct work_struct *wq)
{

    int is_cp_reset = 0;
#ifdef IFX_CP_CRASH  
    int is_cp_crash=0;
#endif
    CPW_DEBUG("start.. \n");

    is_cp_reset = gpio_get_value(GPIO_IFX_CP_RESET);
        if (is_cp_reset == CP_RESET_TRUE)
            CPW_PRINTK("is_cp_reset = %d, Auto Reset!! \n", is_cp_reset);

#ifdef IFX_CP_CRASH
    is_cp_crash = gpio_get_value(GPIO_IFX_CP_CRASH);
    if (is_cp_crash == CP_CRASH_TRUE)
        CPW_PRINTK("is_cp_crash = %d, CP Crash \n", is_cp_crash);
#endif


#ifdef IFX_CP_CRASH      
    if (is_cp_crash == CP_CRASH_TRUE)
#endif      
    {
        CPW_PRINTK( "### CP Crash ### \n");
        input_report_key(s_cpwatcher.input, EVENT_KEY, 1);
        input_report_key(s_cpwatcher.input, EVENT_KEY, 0);
        input_sync(s_cpwatcher.input);     
        CPW_PRINTK( "input_report_key(): %d\n", EVENT_KEY);
    } 
    else 
    {
        CPW_DEBUG( ">>>>IFX modem reset = %d, cp_crash = %d\n", is_cp_reset, is_cp_crash);
        CPW_DEBUG( ">>>>IFX modem was reset!\n");
    }

    CPW_DEBUG("end.. \n");  
}


#ifdef IFX_CP_CRASH
static irqreturn_t ifx_cp_crash_interrupt_handler(int irq, void *dev_id)
{
    int is_cp_crash = 0;

    CPW_DEBUG(" start \n");

    is_cp_crash = gpio_get_value(GPIO_IFX_CP_CRASH);
    if (is_cp_crash == 0)  //IRQF_TRIGGER_FALLING case is ignore !!!
    {
        CPW_DEBUG("IRQF_TRIGGER_FALLING irq is ignored\n");
        return IRQ_HANDLED;
    }

    schedule_delayed_work(&work, msecs_to_jiffies(5));

    CPW_DEBUG(" is end \n");

    return IRQ_HANDLED;
}
#endif

static irqreturn_t ifx_reset_interrupt_handler(int irq, void *dev_id)
{
#ifdef CPW_IFX_TEGRA_EDGE_TRIGGER
    int is_cp_reset = 0;

    CPW_DEBUG(" start \n");

    is_cp_reset = gpio_get_value(GPIO_IFX_CP_RESET);
    if (is_cp_reset == 1)  //IRQF_TRIGGER_RISING case is ignore !!!
    {
        CPW_DEBUG(" IRQF_TRIGGER_RISING irq is ignored\n");
        return IRQ_HANDLED;
    }
#endif  
    schedule_delayed_work(&work, msecs_to_jiffies(5));

    CPW_DEBUG("  is end \n");

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
    
    INIT_DELAYED_WORK(&work, ifx_reset_delayed_work);  

    gpio_request(GPIO_IFX_CP_RESET, "ifx_reset_int_n");
    tegra_gpio_enable(GPIO_IFX_CP_RESET);
    gpio_direction_input(GPIO_IFX_CP_RESET);
    s_cpwatcher.ifx_cp_reset_irq = gpio_to_irq(GPIO_IFX_CP_RESET);   
 
#ifdef CPW_IFX_TEGRA_EDGE_TRIGGER     //nvidia dependency
    err = request_irq(s_cpwatcher.ifx_cp_reset_irq, 
                     ifx_reset_interrupt_handler, 
                                     (IRQF_TRIGGER_FALLING|IRQF_TRIGGER_RISING), 
                                     "ifx_reset_int_n", 
                                     (void*)&s_cpwatcher);
#else
    err = request_irq(s_cpwatcher.ifx_cp_reset_irq, 
                     ifx_reset_interrupt_handler, 
                                     IRQF_TRIGGER_FALLING, 
                                     "ifx_reset_int_n", 
                                     (void*)&s_cpwatcher);

#endif

    if(err)
    {
        CPW_PRINTK(" Failed: [reset_flag] request_irq for ifx_cp_reset_irq!!! (err:%d)\n", err);
        free_irq(s_cpwatcher.ifx_cp_reset_irq, (void*)&s_cpwatcher);
        return -ENOSYS;
    }

    enable_irq_wake(s_cpwatcher.ifx_cp_reset_irq);
 

#ifdef IFX_CP_CRASH
    gpio_request(GPIO_IFX_CP_CRASH, "IFX_CP_CRASH_int_n");
    tegra_gpio_enable(GPIO_IFX_CP_CRASH);
    gpio_direction_input(GPIO_IFX_CP_CRASH);
    s_cpwatcher.ifx_cp_crash_irq = gpio_to_irq(GPIO_IFX_CP_CRASH); 

    err = request_irq(s_cpwatcher.ifx_cp_crash_irq, 
                    ifx_cp_crash_interrupt_handler, 
#ifdef CPW_IFX_TEGRA_EDGE_TRIGGER   
                    (IRQF_TRIGGER_FALLING|IRQF_TRIGGER_RISING), 
#else
                    IRQF_TRIGGER_RISING, 
#endif                  
                    "IFX_CP_CRASH_int_n", 
                    (void*)&s_cpwatcher);

    if(err)
    {
        CPW_PRINTK(" Failed: [cp_state] request_irq for ifx_cp_crash_irq!!! (err:%d)\n", err);
        free_irq(s_cpwatcher.ifx_cp_crash_irq, (void*)&s_cpwatcher);   
        return -ENOSYS;     
    }

//  enable_irq_wake(s_cpwatcher.ifx_cp_crash_irq);
#endif

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

    if (s_cpwatcher.ifx_cp_reset_irq >= 0)
    {
        free_irq(s_cpwatcher.ifx_cp_reset_irq, (void*)&s_cpwatcher);    
    }

    if (!s_cpwatcher.input) 
    {
            input_unregister_device(s_cpwatcher.input);
            input_free_device(s_cpwatcher.input);
    }      

    CPW_DEBUG(" success\n");

    return 0;
}


static struct platform_driver cpw_driver = {
    .probe      = cpw_probe,
    .remove     = __devexit_p(cpw_remove),
    .driver     = {
    .name       = "cpwatcher",
    .owner      = THIS_MODULE,
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
MODULE_DESCRIPTION("X2 CP Watcher Driver");
MODULE_LICENSE("GPL");


