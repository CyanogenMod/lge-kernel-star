/*
 * Copyright (c) 2009 NVIDIA Corporation.  All rights reserved.
 *
 */
 
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/mutex.h>
#include "nvodm_services.h"


#define DRIVER_NAME    "tegra_user_gpio"
#define DRIVER_DESC    "Nvidia Tegra gpio driver for user mode"

#define NV_PANEL_GPIO_ACCESS_DEBUG_PRINT 0

#if NV_PANEL_GPIO_ACCESS_DEBUG_PRINT
#define NV_PANEL_GPIO_ACCESS_DBG(fmt,args...) \
    do { pr_info("("DRIVER_NAME")" fmt, ##args); } while (0)
#else
#define NV_PANEL_GPIO_ACCESS_DBG(fmt,args...) \
    do{} while (0);    
#endif

typedef struct KUserGpioSetValuesRec
{
    NvOdmServicesGpioHandle hOdmGpio;
    NvOdmGpioPinHandle hGpioPin;
    NvU32 PinValue;
} KUserGpioSetValues;


static ssize_t tegra_user_gpio_write(struct file *file, const char __user *data, size_t len, loff_t *ppos)
{
    KUserGpioSetValues* gpio_set_values;
    gpio_set_values=(KUserGpioSetValues*) data;
	NvOdmGpioSetState(gpio_set_values->hOdmGpio, gpio_set_values->hGpioPin, gpio_set_values->PinValue);
     printk(KERN_ERR "tegra_user_gpio_write \n");
    return 0;
}


static ssize_t tegra_user_gpio_read(struct file *file, char __user *buf, size_t len, loff_t *ppos)
{
    return 0;
}

loff_t tegra_user_gpio_lseek(struct file *file, loff_t offset, int origin) 
{ 
    return (loff_t)offset;

} 

static int  
tegra_user_gpio_open(struct inode *inode, struct file *file)
{

    printk("tegra_user_gpio_open is called!!\n");
    return 0;
}


static int  
tegra_user_gpio_release(struct inode *inode, struct file *file)
{
    printk("tegra_user_gpio_release is called!!\n");
    return 0;
}

static struct file_operations tegra_user_gpio_fops = {
    .owner      = THIS_MODULE,
    .read       = tegra_user_gpio_read,
    .write      = tegra_user_gpio_write,
    .open       = tegra_user_gpio_open,
    .release    = tegra_user_gpio_release,
    .llseek     = tegra_user_gpio_lseek, 
};

static struct miscdevice tegra_user_gpio_device =
{
    .name = "tegra_user_gpio",
    .fops = &tegra_user_gpio_fops,
    .minor = 143,
};

static int __init
tegra_user_gpio_init(void)
{
    int retVal = 0;
    retVal = misc_register(&tegra_user_gpio_device);
    
   
    if (retVal <0) printk("tegra_user_gpio_init fails!!\n"); 
    else printk("tegra_user_gpio_init succeeds!!\n");
    
    return retVal;
}

static void __exit
tegra_user_gpio_exit(void)
{
    misc_deregister (&tegra_user_gpio_device);
}

module_init(tegra_user_gpio_init);
module_exit(tegra_user_gpio_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION(DRIVER_DESC);

