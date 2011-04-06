/*
 * star(lgp990) power key
 *
 * Copyright (C) 2009 LGE, Inc.
 *
 * Author: <>
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

//select option
#define POWERKEY_WORKQUEUE

//20100610, sleep status gpio for modem [START]
#define AP_SUSPEND_STATUS
//20100610, sleep status gpio for modem [START]


#define POWERKEY_DELAYED_CHECK

//20100714, AP20 Edge Wakeup Fail [START]
//#define AP20_EDGE_WAKEUP_FAIL
//20100714, AP20 Edge Wakeup Fail [END]


#define AP20_A03_POWERKEY_WAR


#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/uaccess.h>

#include <linux/input.h>

#ifndef POWERKEY_WORKQUEUE
#include <linux/kthread.h>
#include <linux/freezer.h>
#endif

#include "mach/nvrm_linux.h"
#include "nvcommon.h"
#include "nvos.h"
#include "nvodm_services.h"
#include "nvodm_query.h"
#include "nvodm_query_discovery.h"

#ifdef POWERKEY_DELAYED_CHECK
#include <linux/hrtimer.h>
#endif

#ifdef AP20_A03_POWERKEY_WAR
#include <mach/iomap.h>
#include <linux/io.h>

#define PMC_WAKE_STATUS 0x14
#define WAKEUP_POWERKEY_MASK    (1 << 24)     // Wake Event 24 - AP_ONKEY
#endif

//20101110, , Function for Warm-boot [START]
extern void write_cmd_reserved_buffer(unsigned char *buf, size_t len);
extern void read_cmd_reserved_buffer(unsigned char *buf, size_t len);
extern void emergency_restart(void); 
//20101110, , Function for Warm-boot [END]

typedef struct PowerKeyDeviceRec
{
    NvOdmServicesGpioHandle gpioHandle;
    NvOdmGpioPinHandle  pinHandle;
    NvOdmServicesGpioIntrHandle intr;
#ifdef POWERKEY_DELAYED_CHECK
    struct hrtimer  timer;
    NvU32   sleepStatus;
#endif

//20100714, AP20 Edge Wakeup Fail [START]
#ifdef AP20_EDGE_WAKEUP_FAIL
    int             interruptKeyLevel;
#endif
//20100714, AP20 Edge Wakeup Fail [END]

    
    struct input_dev    *inputDev;
    
#ifndef DIRECT_ISR_HANDLE
#ifdef POWERKEY_WORKQUEUE
    struct work_struct  work;
#else
    struct task_struct  *task;
    NvOdmOsSemaphoreHandle  semaphore;
    NvOsMutexHandle hMutex;
#endif
#endif

    NvU32    kill;
} PowerKeyDevice;

static PowerKeyDevice s_powerkey;

#ifdef AP20_A03_POWERKEY_WAR
static int key_wakeup_ISR = 0;

static void __iomem *pmc_base = IO_ADDRESS(TEGRA_PMC_BASE);
#endif

//20101129, idle current issue [START]
typedef struct TouchMakerRec
{
    NvOdmServicesGpioHandle gpioHandle;
    NvOdmGpioPinHandle  pinHandle;
} TouchMaker;

static TouchMaker s_touchMaker;
//20101129, idle current issue [END]


//20100610, sleep status gpio for modem [START]
#ifdef AP_SUSPEND_STATUS
typedef struct ModemCheckRec
{
    NvOdmServicesGpioHandle gpioHandle;
    NvOdmGpioPinHandle  pinHandle;
} ModemCheck;

static ModemCheck s_modemCheck;
#endif
//20100610, sleep status gpio for modem [END]

#define POWERKEY_DELAY_NSEC     20000000     //20ms
#define KEYCHECK_RETRY_COUNT    5   // it must be odd value.

static void powerkey_interrupt_handler(void* arg);
char brdrev[5];

static int __init tegra_setup_board_type(char *str)
{
	printk("[hee.seo] %s \n",str); 
	strcpy(brdrev,str); 
	return 0;
}

early_param("brdrev", tegra_setup_board_type);

#ifdef POWERKEY_DELAYED_CHECK
static enum hrtimer_restart powerkey_timer_func(struct hrtimer *timer)
{
    NvU32   pinValue;

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
    NvOdmGpioInterruptDone(s_powerkey.intr);

    return HRTIMER_NORESTART;
}
#endif


static void powerkey_handle(struct work_struct *wq)
{
    NvU32   pinValue;

#ifdef AP20_A03_POWERKEY_WAR
    unsigned long reg;

    reg = readl(pmc_base + PMC_WAKE_STATUS);

    // Clear power key wakeup pad bit.
    // Because powerkey interrupt might be called before powerkey_resume() is called.
    // In this case, clear bit not to call power key press at powerkey_resume() function.
    if (key_wakeup_ISR == 0 )
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
        NvOdmGpioInterruptDone(s_powerkey.intr);
        return;
    }
#endif

//20100714, AP20 Edge Wakeup Fail [START]
#ifdef AP20_EDGE_WAKEUP_FAIL
    if(s_powerkey.interruptKeyLevel){
        // high level is interrupted, next it will detect low level.
        NvOdmGpioConfig(s_powerkey.gpioHandle, s_powerkey.pinHandle, 
                    NvOdmGpioPinMode_InputInterruptLow);
        s_powerkey.interruptKeyLevel = 0;
        #ifdef POWERKEY_DELAYED_CHECK
        if(s_powerkey.sleepStatus == 0){
            hrtimer_cancel(&s_powerkey.timer);
            hrtimer_start(&s_powerkey.timer, ktime_set(0, POWERKEY_DELAY_NSEC), HRTIMER_MODE_REL);
        }else{
            NvOdmGpioGetState(s_powerkey.gpioHandle, s_powerkey.pinHandle, &pinValue);
            if(!pinValue){
                goto int_done;
            }
            printk("POWERKEY release \n");
            input_report_key(s_powerkey.inputDev, KEY_POWER, 0);
        }
        #else
        printk("POWERKEY release \n");
        input_report_key(s_powerkey.inputDev, KEY_POWER, 0);
        #endif
    }else{
        //low level is interrupted, next it will detect high level.
        NvOdmGpioConfig(s_powerkey.gpioHandle, s_powerkey.pinHandle, 
                    NvOdmGpioPinMode_InputInterruptHigh);
        s_powerkey.interruptKeyLevel = 1;
        #ifdef POWERKEY_DELAYED_CHECK
        if(s_powerkey.sleepStatus == 0){
            hrtimer_cancel(&s_powerkey.timer);
            hrtimer_start(&s_powerkey.timer, ktime_set(0, POWERKEY_DELAY_NSEC), HRTIMER_MODE_REL);
        }else{
            NvOdmGpioGetState(s_powerkey.gpioHandle, s_powerkey.pinHandle, &pinValue);
            if(pinValue){
                goto int_done;
            }
            printk("POWERKEY press \n");
            input_report_key(s_powerkey.inputDev, KEY_POWER, 1);
        }
        #else
        printk("POWERKEY press \n");
        input_report_key(s_powerkey.inputDev, KEY_POWER, 1);
        #endif
    }
    #ifndef POWERKEY_DELAYED_CHECK
    input_sync(s_powerkey.inputDev);
    #endif

int_done:
    NvOdmGpioInterruptDone(s_powerkey.intr);
#else
//20100714, AP20 Edge Wakeup Fail [END]

#ifndef POWERKEY_DELAYED_CHECK
    if(!s_powerkey.gpioHandle || !s_powerkey.pinHandle || !s_powerkey.inputDev)
        printk("POWERKEY handler error\n");
    
    NvOdmGpioGetState(s_powerkey.gpioHandle, s_powerkey.pinHandle, &pinValue);
    
    if(pinValue){
        //printk("POWERKEY release\n");
        input_report_key(s_powerkey.inputDev, KEY_POWER, 0);
        input_sync(s_powerkey.inputDev);
    }else{
        //printk("POWERKEY press\n");
        input_report_key(s_powerkey.inputDev, KEY_POWER, 1);
        input_sync(s_powerkey.inputDev);
    }
    NvOdmGpioInterruptDone(s_powerkey.intr);
#else
    hrtimer_cancel(&s_powerkey.timer);
    hrtimer_start(&s_powerkey.timer, ktime_set(0, POWERKEY_DELAY_NSEC), HRTIMER_MODE_REL);
#endif
#endif
}

#ifdef POWERKEY_WORKQUEUE
static void powerkey_interrupt_handler(void* arg)
{
    PowerKeyDevice *powerKeyDevice = (PowerKeyDevice *)arg;
    
    printk("powerkey_interrupt_handler\n");

    schedule_work(&powerKeyDevice->work);    
}
#else
static void powerkey_interrupt_handler(void* arg)
{
    PowerKeyDevice *powerkeyDevice =(PowerKeyDevice*)arg;
    
    NvOdmOsSemaphoreSignal(powerkeyDevice->semaphore);
}

static int powerkey_thread(void *pdata){
    PowerKeyDevice *powerkeyDevice =(PowerKeyDevice*)pdata;
    
    /* touch event thread should be frozen before suspend */
    set_freezable_with_signal();

    while(powerkeyDevice->kill == 0){
        NvOdmOsSemaphoreWait(powerkeyDevice->semaphore);
        NvOsMutexLock(powerkeyDevice->hMutex);
        powerkey_handle(NULL);
        NvOsMutexUnlock(powerkeyDevice->hMutex);
    }

    return 0;
}
#endif

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

//20101110, , Function for Warm-boot [START]
static ssize_t star_reset_show(struct device *dev, 
            struct device_attribute *attr, char *buf)
{

    unsigned char tmpbuf[2];
    read_cmd_reserved_buffer(tmpbuf,1);
    printk("[doncopy] power key reserved_buffer = %x\n",tmpbuf[0]);
    return 0;
}

static ssize_t star_reset_store(struct device *dev, 
            struct device_attribute *attr, char *buf, size_t count)
{

    unsigned char tmpbuf[2];
    tmpbuf[0] = 'w'; // index for warm-boot
    write_cmd_reserved_buffer(tmpbuf,1);
    emergency_restart();
    return 0;
}

static DEVICE_ATTR(reset, 0666, star_reset_show, star_reset_store);

static struct attribute *star_reset_attributes[] = {
    &dev_attr_reset.attr,
    NULL
};

static const struct attribute_group star_reset_group = {
    .attrs = star_reset_attributes,
};
//20101110, , Function for Warm-boot [END]

#ifndef AP20_A03_POWERKEY_WAR
//20100610, sleep status gpio for modem [START]
#ifdef AP_SUSPEND_STATUS
static int modem_suspend(struct platform_device *pdev, pm_message_t state)
{
    NvOdmGpioSetState(s_modemCheck.gpioHandle, s_modemCheck.pinHandle, 0);
    #ifdef POWERKEY_DELAYED_CHECK 
    s_powerkey.sleepStatus= 1;
    #endif
    return 0;
}

static int modem_resume(struct platform_device *pdev)
{
    NvOdmGpioSetState(s_modemCheck.gpioHandle, s_modemCheck.pinHandle, 1);
    #ifdef POWERKEY_DELAYED_CHECK 
    s_powerkey.sleepStatus= 0;
    #endif
    return 0;
}
#endif
//20100610, sleep status gpio for modem [END]
#endif

static int __init powerkey_probe(struct platform_device *pdev)
{
    int ret;
    NvU32 pin, port;
    const NvOdmPeripheralConnectivity *con = NULL;

//20101129, idle current issue [START]
    //GPIO configuration
    s_touchMaker.gpioHandle = NvOdmGpioOpen();
    port = 'x'-'a';
    pin = 5;
    s_touchMaker.pinHandle = NvOdmGpioAcquirePinHandle(s_touchMaker.gpioHandle, 
                                                    port, pin);
    NvOdmGpioConfig(s_touchMaker.gpioHandle, s_touchMaker.pinHandle, 
                    NvOdmGpioPinMode_InputData);
//20101129, idle current issue [END]


//20100610, sleep status gpio for modem [START]
#ifdef AP_SUSPEND_STATUS
    //GPIO configuration
    s_modemCheck.gpioHandle = NvOdmGpioOpen();
    if (!s_modemCheck.gpioHandle)
    {
        printk(KERN_ERR "[star modem_chk] NvOdmGpioOpen Error \n");
        goto err_open_modem_chk_gpio_fail;
    }
    port = 'r'-'a';
    pin = 0;
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
//20100610, sleep status gpio for modem [END]

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
    
#ifdef POWERKEY_WORKQUEUE
    INIT_WORK(&s_powerkey.work, powerkey_handle);
#else
    NvOsMutexCreate(&s_powerkey.hMutex);
    s_powerkey.semaphore = NvOdmOsSemaphoreCreate(0);
    if (!s_powerkey.semaphore) {
        printk(KERN_ERR "tegra_touch_probe: Semaphore creation failed\n");
        goto err_semaphore_create_failed;
    }
    
    s_powerkey.task =
        kthread_create(powerkey_thread, &s_powerkey, "powerkey_thread");

    if(s_powerkey.task == NULL) {
        goto err_kthread_create_failed;
    }
    wake_up_process( s_powerkey.task );
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

    //20100714, AP20 Edge Wakeup Fail [START]
    #ifdef AP20_EDGE_WAKEUP_FAIL
    s_powerkey.interruptKeyLevel = 0;

    //GPIO interrupt registration
    if (NvOdmGpioInterruptRegister(s_powerkey.gpioHandle, &s_powerkey.intr,
            s_powerkey.pinHandle, NvOdmGpioPinMode_InputInterruptLow, 
            powerkey_interrupt_handler, (void*)&s_powerkey, 0) == NV_FALSE)
    {
        printk(KERN_ERR "[star Powerkey] interrupt registeration fail!\n");
        goto err_interrupt_register_fail;
    }
    #else
    //GPIO interrupt registration
    if (NvOdmGpioInterruptRegister(s_powerkey.gpioHandle, &s_powerkey.intr,
            s_powerkey.pinHandle, NvOdmGpioPinMode_InputInterruptAny, 
            powerkey_interrupt_handler, (void*)&s_powerkey, 0) == NV_FALSE)
    {
        printk(KERN_ERR "[star Powerkey] interrupt registeration fail!\n");
        goto err_interrupt_register_fail;
    }
    #endif
    //20100714, AP20 Edge Wakeup Fail [END]

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

#ifdef POWERKEY_DELAYED_CHECK 
    s_powerkey.sleepStatus = 0;
    hrtimer_init(&s_powerkey.timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    s_powerkey.timer.function = powerkey_timer_func;
#endif

//20100703, PMIC reset [START]
    ret = sysfs_create_group(&pdev->dev.kobj, &star_pmic_group);
    if (ret) {
        printk(KERN_ERR "[star powerkey] sysfs_create_group ERROR\n");
        goto err_input_device_register_fail;
    }
//20100703, PMIC reset [END]
    
    ret = sysfs_create_group(&pdev->dev.kobj, &star_hwsku_group);
    if (ret) {
        printk(KERN_ERR "[star powerkey] sysfs_create_group ERROR\n");
        goto err_input_device_register_fail;
    }

//20101110, , Function for Warm-boot [START]
    ret = sysfs_create_group(&pdev->dev.kobj, &star_reset_group);
    if (ret) {
        printk(KERN_ERR "[star powerkey] sysfs_create_group ERROR\n");
        goto err_input_device_register_fail;
    }
//20101110, , Function for Warm-boot [END]

    return 0;
err_input_device_register_fail: 
    input_free_device(s_powerkey.inputDev);    
err_input_device_allocation_fail: 
    NvOdmGpioInterruptUnregister(s_powerkey.gpioHandle, s_powerkey.pinHandle,
        s_powerkey.intr);    
err_interrupt_register_fail:  
    NvOdmGpioReleasePinHandle(s_powerkey.gpioHandle, s_powerkey.pinHandle);
err_gpio_pin_acquire_fail:
    NvOdmGpioClose(s_powerkey.gpioHandle);  

err_open_gpio_fail:
  
#ifndef POWERKEY_WORKQUEUE
err_kthread_create_failed:
    NvOdmOsSemaphoreDestroy(s_powerkey.semaphore);
err_semaphore_create_failed:
    NvOsMutexDestroy(s_powerkey.hMutex);
#endif

err_probe_fail:

//20100610, sleep status gpio for modem [START]
#ifdef AP_SUSPEND_STATUS
    NvOdmGpioReleasePinHandle(s_modemCheck.gpioHandle, s_modemCheck.pinHandle);
err_modem_chk_gpio_pin_acquire_fail:
    NvOdmGpioClose(s_modemCheck.gpioHandle); 
err_open_modem_chk_gpio_fail:
#endif
//20100610, sleep status gpio for modem [END]
    return -ENOSYS;
}

static int powerkey_remove(struct platform_device *pdev)
{
//20100703, PMIC reset [START]
    sysfs_remove_group(&pdev->dev.kobj, &star_pmic_group);
    sysfs_remove_group(&pdev->dev.kobj, &star_hwsku_group);
    sysfs_remove_group(&pdev->dev.kobj, &star_reset_group); //20101110, , Function for Warm-boot
//20100703, PMIC reset [END]
    
//20100610, sleep status gpio for modem [START]
#ifdef AP_SUSPEND_STATUS
    NvOdmGpioReleasePinHandle(s_modemCheck.gpioHandle, s_modemCheck.pinHandle);
    NvOdmGpioClose(s_modemCheck.gpioHandle); 
#endif
//20100610, sleep status gpio for modem [END]

    input_unregister_device(s_powerkey.inputDev);
    NvOdmGpioInterruptUnregister(s_powerkey.gpioHandle, s_powerkey.pinHandle,
        s_powerkey.intr);
    
    NvOdmGpioReleasePinHandle(s_powerkey.gpioHandle, s_powerkey.pinHandle);
    NvOdmGpioClose(s_powerkey.gpioHandle);

#ifndef POWERKEY_WORKQUEUE
    s_powerkey.kill = NV_TRUE;
    NvOdmOsSemaphoreSignal(s_powerkey.semaphore);

    NvOdmOsSemaphoreDestroy(s_powerkey.semaphore);
    NvOsMutexDestroy(s_powerkey.hMutex);
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
//20100610, sleep status gpio for modem [START]
#ifdef AP20_A03_POWERKEY_WAR
    .suspend    = powerkey_suspend,
    .resume     = powerkey_resume,
#else
#ifdef AP_SUSPEND_STATUS
    .suspend    = modem_suspend,
    .resume     = modem_resume,
#endif
#endif
//20100610, sleep status gpio for modem [END]
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

