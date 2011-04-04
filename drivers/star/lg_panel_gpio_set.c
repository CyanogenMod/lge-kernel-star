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
//20101208 km.lee@lge.com GPIO ioctl [START]
#include <linux/uaccess.h>
#include <linux/ioctl.h>
//20101208 km.lee@lge.com GPIO ioctl [END]

#define DRIVER_NAME    "tegra_panel_gpio_set"
#define DRIVER_DESC    "Nvidia Tegra panel_gpio_set driver"

#define NV_PANEL_GPIO_ACCESS_DEBUG_PRINT 0

#if NV_PANEL_GPIO_ACCESS_DEBUG_PRINT
#define NV_PANEL_GPIO_ACCESS_DBG(fmt,args...) \
    do { pr_info("("DRIVER_NAME")" fmt, ##args); } while (0)
#else
#define NV_PANEL_GPIO_ACCESS_DBG(fmt,args...) \
    do{} while (0);    
#endif

typedef struct KPanelGpioSetValuesRec
{
    NvOdmServicesGpioHandle hOdmGpio;
    NvOdmGpioPinHandle hGpioPin;
    NvU32 PinValue;
} KPanelGpioSetValues;

//20101208 km.lee@lge.com GPIO ioctl
#define HITACHI_CPU_WRITE       _IOW('H', 0x01, struct KPanelGpioSetValuesRec)

static ssize_t tegra_panel_gpio_set_write(struct file *file, const char __user *data,
                          size_t len, loff_t *ppos)
{
    KPanelGpioSetValues* gpio_set_values;
    gpio_set_values=(KPanelGpioSetValues*) data;
	NvOdmGpioSetState(gpio_set_values->hOdmGpio, gpio_set_values->hGpioPin, gpio_set_values->PinValue);

    return 0;
}


static ssize_t tegra_panel_gpio_set_read(struct file *file, char __user *buf, size_t len,
                         loff_t *ppos)
{
    return 0;
}

loff_t tegra_panel_gpio_set_lseek(struct file *file, loff_t offset, int origin) 
{ 
    return (loff_t)offset;

} 

static int  
tegra_panel_gpio_set_open(struct inode *inode, struct file *file)
{

    printk("tegra_panel_gpio_set_open is called!!\n");
    return 0;
}


static int  
tegra_panel_gpio_set_release(struct inode *inode, struct file *file)
{
    printk("tegra_panel_gpio_set_release is called!!\n");
    return 0;
}

//20101208 km.lee@lge.com GPIO ioctl [START]
static int tegra_panel_gpio_set_ioctl(struct inode *inode, struct file *file,
		unsigned int cmd, unsigned long arg)
{
    void __user *argp = (void __user *)arg;
    KPanelGpioSetValues gpio_set_values;

    switch (cmd) {
        case HITACHI_CPU_WRITE:		
            if (copy_from_user(&gpio_set_values, argp, sizeof(gpio_set_values)))
                return -EFAULT;
            NvOdmGpioSetState(gpio_set_values.hOdmGpio, gpio_set_values.hGpioPin, gpio_set_values.PinValue);
            break;
        default :
            printk("tegra_panel_gpio_set_ioctl :: Unknown IOCTL !!\n");	
    }
	
    return 0;
}
//20101208 km.lee@lge.com GPIO ioctl [END]

static struct file_operations tegra_panel_gpio_set_fops = {
    .owner      = THIS_MODULE,
    .read       = tegra_panel_gpio_set_read,
    .write      = tegra_panel_gpio_set_write,
    .open       = tegra_panel_gpio_set_open,
    .release    = tegra_panel_gpio_set_release,
    .llseek     = tegra_panel_gpio_set_lseek, 
//20101208 km.lee@lge.com GPIO ioctl
    .ioctl = tegra_panel_gpio_set_ioctl,
};

static struct miscdevice tegra_panel_gpio_set_device =
{
    .name = "panel_gpio_set",
    .fops = &tegra_panel_gpio_set_fops,
    .minor = 0,
};

static int __init
tegra_panel_gpio_set_init(void)
{
    int retVal = 0;
    retVal = misc_register(&tegra_panel_gpio_set_device);
    
    
    if (retVal <0) printk("tegra_panel_gpio_set_init fails!!\n"); 
    else printk("tegra_panel_gpio_set_init succeeds!!\n");
    
    return retVal;
}

static void __exit
tegra_panel_gpio_set_exit(void)
{
    misc_deregister (&tegra_panel_gpio_set_device);
}

module_init(tegra_panel_gpio_set_init);
module_exit(tegra_panel_gpio_set_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION(DRIVER_DESC);

