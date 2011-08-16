/*
 * star(lgp990) power key
 *
 * Copyright (C) 2009 LGE, Inc.
 *
 * Author: Changsu Ha <>
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
	@brief		 star(lgp990) power key
 
	@author		
	@date		 2010-04-13
 
	@version	 V1.00		 2010.04.13		 Changsu Ha	 Create
*/

//20100610, , sleep status gpio for modem [START]
#define AP_SUSPEND_STATUS
//20100610, , sleep status gpio for modem [START]

#define AP20_A03_POWERKEY_WAR

#define POWERKEY_DELAYED_WORKQUEUE

#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/uaccess.h>

#include <linux/input.h>
#include <linux/wakelock.h>  

#include "mach/nvrm_linux.h"
#include "nvcommon.h"
#include "nvos.h"
#include "nvodm_services.h"
#include "nvodm_query.h"
#include "nvodm_query_discovery.h"

#ifdef AP20_A03_POWERKEY_WAR
#include <mach/iomap.h>
#include <linux/io.h>

#define PMC_WAKE_STATUS 0x14
#define WAKEUP_POWERKEY_MASK    (1 << 24)     // Wake Event 24 - AP_ONKEY
#endif

// 20110209  disable gpio interrupt during power-off  [START] 
#include <linux/gpio.h>

extern void tegra_gpio_disable_all_irq(void);
// 20110209  disable gpio interrupt during power-off  [END] 
#if 1
extern void write_cmd_reserved_buffer(unsigned char *buf, size_t len);
extern void read_cmd_reserved_buffer(unsigned char *buf, size_t len);
extern void emergency_restart(void); 
#endif

int  pwky_shutdown = 0;
typedef struct PowerKeyDeviceRec
{
    NvOdmServicesGpioHandle gpioHandle;
    NvOdmGpioPinHandle  pinHandle;
    NvOdmServicesGpioIntrHandle intHandle;    
    struct input_dev    *inputDev;
    
#ifdef POWERKEY_DELAYED_WORKQUEUE
    struct wake_lock wlock;
    struct delayed_work  work;
#else
    struct work_struct  work;
#endif

    NvU32    kill;
} PowerKeyDevice;

static PowerKeyDevice s_powerkey;

#ifdef AP20_A03_POWERKEY_WAR
static int key_wakeup_ISR = 0;

static void __iomem *pmc_base = IO_ADDRESS(TEGRA_PMC_BASE);
#endif

//20110324, , LP1 powerkey skip issue [START]
extern bool core_lock_on;
static int LP1_key_wake = 0;
//20110324, , LP1 powerkey skip issue [END]

//20101129, , idle current issue [START]
typedef struct TouchMakerRec
{
    NvOdmServicesGpioHandle gpioHandle;
    NvOdmGpioPinHandle  pinHandle;
} TouchMaker;

static TouchMaker s_touchMaker;
//20101129, , idle current issue [END]


//20100610, , sleep status gpio for modem [START]
#ifdef AP_SUSPEND_STATUS
typedef struct ModemCheckRec
{
    NvOdmServicesGpioHandle gpioHandle;
    NvOdmGpioPinHandle  pinHandle;
} ModemCheck;

static ModemCheck s_modemCheck;
#endif
//20100610, , sleep status gpio for modem [END]

#define POWERKEY_DELAY_NSEC     20000000     //20ms
#define KEYCHECK_RETRY_COUNT    5   // it must be odd value.

static void powerkey_interrupt_handler(void* arg);
static char brdrev[5];

static int __init tegra_setup_board_type(char *str)
{
	printk("[hee.seo] %s \n",str); 
	strcpy(brdrev,str); 
	return 0;
}

early_param("brdrev", tegra_setup_board_type);

static void powerkey_handle(struct work_struct *wq)
{
    NvU32   pinValue;

#ifdef AP20_A03_POWERKEY_WAR
    unsigned long reg;

    reg = readl(pmc_base + PMC_WAKE_STATUS);

    // Clear power key wakeup pad bit.
    // Because powerkey interrupt might be called before powerkey_resume() is called.
    // In this case, clear bit not to call power key press at powerkey_resume() function.
    if (key_wakeup_ISR == 0 || LP1_key_wake == 1)
    {
        if( reg & WAKEUP_POWERKEY_MASK){
            printk("[PWR_KEY] wakeup pad clear\n");
            //clear wakeuppad status
            writel(WAKEUP_POWERKEY_MASK, pmc_base + PMC_WAKE_STATUS);
        }
        printk("[PWR_KEY] wakeup by powerkey POWERKEY press\n");
        input_report_key(s_powerkey.inputDev, KEY_POWER, 1);
        input_sync(s_powerkey.inputDev);
        
        NvOdmGpioGetState(s_powerkey.gpioHandle, s_powerkey.pinHandle, &pinValue);
        if(pinValue){
            printk("[PWR_KEY] wakeup by powerkey POWERKEY release\n");
            input_report_key(s_powerkey.inputDev, KEY_POWER, 0);
            input_sync(s_powerkey.inputDev);
        }
        key_wakeup_ISR = 1;
//20110324, , LP1 powerkey skip issue [START]
        LP1_key_wake = 0;
//20110324, , LP1 powerkey skip issue [END]
        return;
    }
#endif

    if(!s_powerkey.gpioHandle || !s_powerkey.pinHandle || !s_powerkey.inputDev)
        printk("POWERKEY handler error\n");
    
    NvOdmGpioGetState(s_powerkey.gpioHandle, s_powerkey.pinHandle, &pinValue);
    
    if(pinValue){
        printk("POWERKEY release\n");
        input_report_key(s_powerkey.inputDev, KEY_POWER, 0);
    }else{
        printk("POWERKEY press\n");
        input_report_key(s_powerkey.inputDev, KEY_POWER, 1);
    }
    input_sync(s_powerkey.inputDev);
}

static void powerkey_interrupt_handler(void* arg)
{
    PowerKeyDevice *powerKeyDevice = (PowerKeyDevice *)arg;
    
    if (pwky_shutdown)
    {
        NvOdmGpioInterruptDone(s_powerkey.intHandle);
        return;
    }
    printk("powerkey_interrupt_handler\n");
//20110324, , LP1 powerkey skip issue [START]
    if(core_lock_on && key_wakeup_ISR==0){
        LP1_key_wake = 1;
    }
//20110324, , LP1 powerkey skip issue [END]
#ifdef POWERKEY_DELAYED_WORKQUEUE
    schedule_delayed_work(&powerKeyDevice->work, msecs_to_jiffies(20));
    wake_lock_timeout(&s_powerkey.wlock, msecs_to_jiffies(50));
#else
    schedule_work(&powerKeyDevice->work);
#endif
    NvOdmGpioInterruptDone(s_powerkey.intHandle);
}

static ssize_t star_pmic_show(struct device *dev, 
            struct device_attribute *attr, char *buf)
{
    sprintf(buf, "PMIC running\n");
    return (ssize_t)(strlen(buf) + 1);
}

static ssize_t star_pmic_store(struct device *dev, 
            struct device_attribute *attr, char *buf, size_t count)
{
    u32 val = 0;

    val = simple_strtoul(buf, NULL, 10);
    if(val){
        NvOdmPeripheralConnectivity const *conn = NULL;
        NvOdmServicesPmuHandle hPmu;

        conn = NvOdmPeripheralGetGuid( NV_ODM_GUID('p','m','_','r','e','s','e','t') );
        hPmu = NvOdmServicesPmuOpen();

        if(!conn){
            printk("ERROR : invalid GUID\n");
            sprintf(buf, "PMIC reset fail\n");
            return (ssize_t)(strlen(buf) + 1); 
        }
        
        if( conn->AddressList[0].Interface == NvOdmIoModule_Vdd )
        {
            NvU32 settle_us;
        
            /* set the rail volatage to the recommended */
            NvOdmServicesPmuSetVoltage( hPmu,
                conn->AddressList[0].Address, NVODM_VOLTAGE_OFF, &settle_us );
        
            /* wait for rail to settle */
            NvOdmOsWaitUS( settle_us );
        }

        NvOdmServicesPmuClose(hPmu);
    }
    sprintf(buf, "PMIC reset\n");
    printk("PMIC reset\n");

    return (ssize_t)(strlen(buf) + 1);
}

static DEVICE_ATTR(pmic, 0666, star_pmic_show, star_pmic_store);


static ssize_t star_hwsku_show(struct device *dev, 
            struct device_attribute *attr, char *buf)
{
    sprintf(buf, "%s",brdrev);
    return (ssize_t)(strlen(buf) + 1);
}

static ssize_t star_hwsku_store(struct device *dev, 
            struct device_attribute *attr, char *buf, size_t count)
{

    return (ssize_t)(strlen(buf) + 1);
}


static DEVICE_ATTR(hwsku, 0666, star_hwsku_show, star_hwsku_store);

static struct attribute *star_pmic_attributes[] = {
    &dev_attr_pmic.attr,
    NULL
};

static struct attribute *star_hwsku_attributes[] = {
    &dev_attr_hwsku.attr,
    NULL
};


static const struct attribute_group star_hwsku_group = {
    .attrs = star_hwsku_attributes,
};


static const struct attribute_group star_pmic_group = {
    .attrs = star_pmic_attributes,
};

#if 1
static ssize_t star_reset_show(struct device *dev, 
            struct device_attribute *attr, char *buf)
{

    unsigned char tmpbuf[2];
    int ret;
    int tag;

    read_cmd_reserved_buffer(tmpbuf,1);
    printk(" power key reserved_buffer = %x\n",tmpbuf[0]);

    if ('w' == tmpbuf[0]||'p'== tmpbuf[0])
        tag = 1;
    else
        tag = 0;

    ret = sprintf(buf,"%d\n",tag);
    return ret;
}

static ssize_t star_reset_store(struct device *dev, 
            struct device_attribute *attr, char *buf, size_t count)
{

    unsigned char tmpbuf[2];
    tmpbuf[0] = 'w'; // index for warm-boot
    write_cmd_reserved_buffer(tmpbuf,1);
    emergency_restart();
    return count;
}

static DEVICE_ATTR(reset, 0666, star_reset_show, star_reset_store);

static struct attribute *star_reset_attributes[] = {
    &dev_attr_reset.attr,
    NULL
};

static const struct attribute_group star_reset_group = {
    .attrs = star_reset_attributes,
};
#endif
// 20110209  disable gpio interrupt during power-off  [START] 
//extern void muic_gpio_interrupt_mask();
extern void keep_touch_led_on();

static ssize_t star_poweroff_store(struct device *dev, struct device_attribute *attr, char *buf, size_t count)
{
    u32 val = 0;
    val = simple_strtoul(buf, NULL, 10);

    pwky_shutdown = 1;

    printk("[PowerKey] input value = %d  @star_powerkey_store \n", val );

    if( val == 1 )
    {
        wake_lock(&s_powerkey.wlock);
        tegra_gpio_disable_all_irq();
        keep_touch_led_on();
    }
    return count;
}

static DEVICE_ATTR(poweroff, 0222, NULL, star_poweroff_store);

static struct attribute *star_poweroff_attributes[] = {
    &dev_attr_poweroff.attr,
    NULL
};

static const struct attribute_group star_poweroff_group = {
    .attrs = star_poweroff_attributes,
};
// 20110209  disable gpio interrupt during power-off  [END] 


#ifndef AP20_A03_POWERKEY_WAR
//20100610, , sleep status gpio for modem [START]
#ifdef AP_SUSPEND_STATUS
static int modem_suspend(struct platform_device *pdev, pm_message_t state)
{
    NvOdmGpioSetState(s_modemCheck.gpioHandle, s_modemCheck.pinHandle, 0);
    return 0;
}

static int modem_resume(struct platform_device *pdev)
{
    NvOdmGpioSetState(s_modemCheck.gpioHandle, s_modemCheck.pinHandle, 1);
    return 0;
}
#endif
//20100610, , sleep status gpio for modem [END]
#endif

static int __init powerkey_probe(struct platform_device *pdev)
{
    int ret;
    NvU32 pin, port;
    const NvOdmPeripheralConnectivity *con = NULL;

//20101129, , idle current issue [START]
    //GPIO configuration
    s_touchMaker.gpioHandle = NvOdmGpioOpen();
    port = 'x'-'a';
    pin = 5;
    s_touchMaker.pinHandle = NvOdmGpioAcquirePinHandle(s_touchMaker.gpioHandle, 
                                                    port, pin);
    NvOdmGpioConfig(s_touchMaker.gpioHandle, s_touchMaker.pinHandle, 
                    NvOdmGpioPinMode_InputData);
//20101129, , idle current issue [END]


//20100610, , sleep status gpio for modem [START]
#ifdef AP_SUSPEND_STATUS
    //GPIO configuration
    s_modemCheck.gpioHandle = NvOdmGpioOpen();
    if (!s_modemCheck.gpioHandle)
    {
        printk(KERN_ERR "[star modem_chk] NvOdmGpioOpen Error \n");
        goto err_open_modem_chk_gpio_fail;
    }
#ifdef CONFIG_MACH_STAR_TMUS
    port = 'h'-'a';
    pin = 2;
#else
    port = 'r'-'a';
    pin = 0;
#endif
    s_modemCheck.pinHandle = NvOdmGpioAcquirePinHandle(s_modemCheck.gpioHandle, 
                                                    port, pin);
    if (!s_modemCheck.pinHandle)
    {
        printk(KERN_ERR "[star modem_chk] NvOdmGpioAcquirePinHandle Error\n");
        goto err_modem_chk_gpio_pin_acquire_fail;
    }
    NvOdmGpioSetState(s_modemCheck.gpioHandle, s_modemCheck.pinHandle, 1);
    NvOdmGpioConfig(s_modemCheck.gpioHandle, s_modemCheck.pinHandle, 
                    NvOdmGpioPinMode_Output);
#endif
//20100610, , sleep status gpio for modem [END]

    memset(&s_powerkey, 0x00, sizeof(s_powerkey));

    //get query
    con = NvOdmPeripheralGetGuid(NV_ODM_GUID('p','o','w','e','r','k','e','y'));
    if(!con){
        printk(KERN_ERR "[star powerkey] ODM GUID Error \n");
        goto err_probe_fail;
    }

    if ( con->AddressList[0].Interface == NvOdmIoModule_Gpio){
        port = con->AddressList[0].Instance;
        pin = con->AddressList[0].Address;
    }else{
        printk(KERN_ERR "[star powerkey] cannot find ODM GUID \n");
        goto err_probe_fail;
    }
    
#ifdef POWERKEY_DELAYED_WORKQUEUE
    INIT_DELAYED_WORK(&s_powerkey.work, powerkey_handle);
    wake_lock_init(&s_powerkey.wlock, WAKE_LOCK_SUSPEND, "powerkey_delay");
#else
    INIT_WORK(&s_powerkey.work, powerkey_handle);
#endif

    //GPIO configuration
    s_powerkey.gpioHandle = NvOdmGpioOpen();
    if (!s_powerkey.gpioHandle)
    {
        printk(KERN_ERR "[star powerkey] NvOdmGpioOpen Error \n");
        goto err_open_gpio_fail;
    }
    s_powerkey.pinHandle = NvOdmGpioAcquirePinHandle(s_powerkey.gpioHandle, 
                                                    port, pin);
    if (!s_powerkey.pinHandle)
    {
        printk(KERN_ERR "[star powerkey] NvOdmGpioAcquirePinHandle Error\n");
        goto err_gpio_pin_acquire_fail;
    }
    //NvOdmGpioSetState(s_powerkey.gpioHandle, s_powerkey.pinHandle, 0);
    NvOdmGpioConfig(s_powerkey.gpioHandle, s_powerkey.pinHandle, 
                    NvOdmGpioPinMode_InputData);

    //GPIO interrupt registration
    if (NvOdmGpioInterruptRegister(s_powerkey.gpioHandle, &s_powerkey.intHandle,
            s_powerkey.pinHandle, NvOdmGpioPinMode_InputInterruptAny, 
            powerkey_interrupt_handler, (void*)&s_powerkey, 0) == NV_FALSE)
    {
        printk(KERN_ERR "[star Powerkey] interrupt registeration fail!\n");
        goto err_interrupt_register_fail;
    }

    // input device
    s_powerkey.inputDev = input_allocate_device();
    if (!s_powerkey.inputDev) {
        printk(KERN_ERR "[star Powerkey] input_allocate_device Error!\n");
        goto err_input_device_allocation_fail;
    }
    s_powerkey.inputDev->name = "powerkey";
    //s_powerkey.inputDev->id.bustype = BUS_HOST;
    s_powerkey.inputDev->evbit[0] = BIT(EV_KEY) | BIT(EV_PWR);
    set_bit(KEY_POWER, s_powerkey.inputDev->keybit);

    ret = input_register_device(s_powerkey.inputDev);
    if (ret) {
        printk(KERN_ERR "[star powerkey] input_register_device Error\n");
        goto err_input_device_register_fail;
    }

//20100703, , PMIC reset [START]
    ret = sysfs_create_group(&pdev->dev.kobj, &star_pmic_group);
    if (ret) {
        printk(KERN_ERR "[star powerkey] sysfs_create_group ERROR\n");
        goto err_pmic_sysfs_fail;
    }
//20100703, , PMIC reset [END]
    
    ret = sysfs_create_group(&pdev->dev.kobj, &star_hwsku_group);
    if (ret) {
        printk(KERN_ERR "[star powerkey] sysfs_create_group ERROR\n");
        goto err_hwsku_sysfs_fail;
    }

    ret = sysfs_create_group(&pdev->dev.kobj, &star_reset_group);
    if (ret) {
        printk(KERN_ERR "[star powerkey] sysfs_create_group ERROR\n");
        goto err_reset_sysfs_fail;
    }

// 20110209  disable gpio interrupt during power-off  [START] 
    ret = sysfs_create_group(&pdev->dev.kobj, &star_poweroff_group);
    if (ret) {
        printk(KERN_ERR "[star powerkey] sysfs_create_group <star_poweroff_group> ERROR\n");
        goto err_input_device_register_fail;
    }
// 20110209  disable gpio interrupt during power-off  [END] 

    return 0;
err_reset_sysfs_fail:    
    sysfs_remove_group(&pdev->dev.kobj, &star_hwsku_group);
err_hwsku_sysfs_fail:
    sysfs_remove_group(&pdev->dev.kobj, &star_pmic_group);
err_pmic_sysfs_fail:
    input_unregister_device(s_powerkey.inputDev);

err_input_device_register_fail: 
    input_free_device(s_powerkey.inputDev);    
err_input_device_allocation_fail: 
    NvOdmGpioInterruptUnregister(s_powerkey.gpioHandle, s_powerkey.pinHandle,
        s_powerkey.intHandle);    
err_interrupt_register_fail:  
    NvOdmGpioReleasePinHandle(s_powerkey.gpioHandle, s_powerkey.pinHandle);
err_gpio_pin_acquire_fail:
    NvOdmGpioClose(s_powerkey.gpioHandle);  

err_open_gpio_fail:
#ifdef POWERKEY_DELAYED_WORKQUEUE
    wake_lock_destroy(&s_powerkey.wlock);
#endif

err_probe_fail:

//20100610, , sleep status gpio for modem [START]
#ifdef AP_SUSPEND_STATUS
    NvOdmGpioReleasePinHandle(s_modemCheck.gpioHandle, s_modemCheck.pinHandle);
err_modem_chk_gpio_pin_acquire_fail:
    NvOdmGpioClose(s_modemCheck.gpioHandle); 
err_open_modem_chk_gpio_fail:
#endif
//20100610, , sleep status gpio for modem [END]
    return -ENOSYS;
}

static int powerkey_remove(struct platform_device *pdev)
{
//20100703, , PMIC reset [START]
    sysfs_remove_group(&pdev->dev.kobj, &star_pmic_group);
    sysfs_remove_group(&pdev->dev.kobj, &star_hwsku_group);
    sysfs_remove_group(&pdev->dev.kobj, &star_reset_group);
//20100703, , PMIC reset [END]
    
//20100610, , sleep status gpio for modem [START]
#ifdef AP_SUSPEND_STATUS
    NvOdmGpioReleasePinHandle(s_modemCheck.gpioHandle, s_modemCheck.pinHandle);
    NvOdmGpioClose(s_modemCheck.gpioHandle); 
#endif
//20100610, , sleep status gpio for modem [END]

    input_unregister_device(s_powerkey.inputDev);
    input_free_device(s_powerkey.inputDev);  

    NvOdmGpioInterruptUnregister(s_powerkey.gpioHandle, s_powerkey.pinHandle,
        s_powerkey.intHandle);
    
    NvOdmGpioReleasePinHandle(s_powerkey.gpioHandle, s_powerkey.pinHandle);
    NvOdmGpioClose(s_powerkey.gpioHandle);

#ifdef POWERKEY_DELAYED_WORKQUEUE
    wake_lock_destroy(&s_powerkey.wlock);
#endif
    return 0;
}

#ifdef AP20_A03_POWERKEY_WAR
int powerkey_suspend(struct platform_device *dev, pm_message_t state)
{
    unsigned long reg;
    
    reg = readl(pmc_base + PMC_WAKE_STATUS);

    // Clear power key wakeup pad bit.
    if (reg & WAKEUP_POWERKEY_MASK)
    {
        writel(WAKEUP_POWERKEY_MASK, pmc_base + PMC_WAKE_STATUS);
    }

#ifdef AP_SUSPEND_STATUS
    NvOdmGpioSetState(s_modemCheck.gpioHandle, s_modemCheck.pinHandle, 0);
#endif
    key_wakeup_ISR = 0;

    return 0;
}

int powerkey_resume(struct platform_device *dev)
{
    NvU32   pinValue;
    unsigned long reg;

#ifdef AP_SUSPEND_STATUS
    NvOdmGpioSetState(s_modemCheck.gpioHandle, s_modemCheck.pinHandle, 1);
#endif

    reg = readl(pmc_base + PMC_WAKE_STATUS);

    if (key_wakeup_ISR == 0)
    {
        if (reg & WAKEUP_POWERKEY_MASK) {
            //clear wakeuppad status
            writel(WAKEUP_POWERKEY_MASK, pmc_base + PMC_WAKE_STATUS);
            printk("[PWR_KEY] resume func POWERKEY press\n");
            input_report_key(s_powerkey.inputDev, KEY_POWER, 1);
            input_sync(s_powerkey.inputDev);

            NvOdmGpioGetState(s_powerkey.gpioHandle, s_powerkey.pinHandle, &pinValue);
            if(pinValue){
                printk("[PWR_KEY] resume func POWERKEY release\n");
                input_report_key(s_powerkey.inputDev, KEY_POWER, 0);
                input_sync(s_powerkey.inputDev);
            }
        }
        key_wakeup_ISR = 1;
    }

    return 0;
}
#endif

static struct platform_driver powerkey_driver = {
    .probe      = powerkey_probe,
    .remove     = powerkey_remove,
//20100610, , sleep status gpio for modem [START]
#ifdef AP20_A03_POWERKEY_WAR
    .suspend    = powerkey_suspend,
    .resume     = powerkey_resume,
#else
#ifdef AP_SUSPEND_STATUS
    .suspend    = modem_suspend,
    .resume     = modem_resume,
#endif
#endif
//20100610, , sleep status gpio for modem [END]
    .driver = {
        .name   = "star_powerkey",
        .owner  = THIS_MODULE,
    },
};

static int __init powerkey_init(void)
{
    return platform_driver_register(&powerkey_driver);
}

static void __exit powerkey_exit(void)
{
    platform_driver_unregister(&powerkey_driver);
}

module_init(powerkey_init);
module_exit(powerkey_exit);

MODULE_AUTHOR("");
MODULE_DESCRIPTION("star power key");
MODULE_LICENSE("GPL");

