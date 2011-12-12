
/*
 * Star - CP Watcher Driver
 *
 * Copyright (C) 2010 LGE, Inc.
 *
 *
 * Author: Kyungsik Lee <>
 *
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


#include <linux/module.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/moduleparam.h>
#include <linux/kthread.h>
#include <linux/workqueue.h>

#include <asm/gpio.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <asm/system.h>

#define NV_DEBUG 0
#if defined(CONFIG_MACH_STAR)
#include "nvodm_services.h"
#include "nvodm_query_discovery.h"
#endif /* CONFIG_MACH_STAR */


#if defined(CONFIG_DEBUG_FS)

#include <linux/debugfs.h>

#define ENABLE_DEBUG_MESSAGE 0x01
#define ENABLE_TEST_MODE 0x02


//#define CPW_SIMPLE_DEBUG

#ifdef CPW_SIMPLE_DEBUG
static int debug_enable_flag = 1;//Debug
#else
static int debug_enable_flag = 0;//Release
#endif

#endif//CONFIG_DEBUG_FS


/*
 * Debug
 */
#define DEBUG_CP
#ifdef DEBUG_CP
#define DBG(x...) if (debug_enable_flag & ENABLE_DEBUG_MESSAGE) { \
						printk(x); \
				  }
#else
#define DBG(x...) do { } while(0)
#endif


#if !defined(TRUE)
#define TRUE 0x01
#endif
#if !defined(FALSE)
#define FALSE 0x00 
#endif


//#define DEBUG_CPWATCHER_REPORT_INPUT
#define DELAY_SECONDS 8
#define REPORT_DELAY (DELAY_SECONDS * 1000)
#define CPWATCHER_DELAY_TIME msecs_to_jiffies(5)


/*
 * ODM Service
 */
#define CPDEVICE_GUID NV_ODM_GUID('c','p','d','e','v','i','c','e')


/*
 * Set scancode for hidden menu
 */
#ifdef CPW_SIMPLE_DEBUG
#define EVENT_KEY KEY_SEARCH //debug
#else
#define EVENT_KEY KEY_F24 //194, Need to be changed
#if defined (CONFIG_MACH_STAR_REV_F)
#define EVENT_HARD_RESET_KEY	195	//this key number is not used in input.h
#endif
#endif

#define	HEADSET_PORT 6
#define	HEADSET_PIN 3
#if defined (CONFIG_MACH_STAR_REV_F)
#define ENABLE_CP_HARD_RESET
#endif
struct cpwatcher_dev {

	struct input_dev *input;
    NvOdmServicesGpioHandle hGpio;
    NvOdmGpioPinHandle hCP_status;
    NvOdmServicesGpioIntrHandle hGpioInterrupt;
	NvU32 port;
	NvU32 pin;
	NvU32 status;
	NvU8 onoff;
	NvU32 delay;
#if defined (CONFIG_MACH_STAR_REV_F)
#ifdef ENABLE_CP_HARD_RESET				
	NvOdmGpioPinHandle hCP_status_HardReset;
	NvOdmServicesGpioIntrHandle hGpioInterrupt_HardReset;
	NvU32 port_HardReset;
	NvU32 pin_HardReset;
	NvU32 status_HardReset;
	NvU8 onoff_HardReset;
	NvU32 delay_HardReset;
#endif
#endif
	struct delayed_work delayed_work_cpwatcher;
	struct mutex lock;
#ifdef DEBUG_CPWATCHER_REPORT_INPUT
	struct task_struct  *task;
#endif

};
static struct cpwatcher_dev *cpwatcher;

#if defined(CONFIG_DEBUG_FS)
static int debug_control_set(void *data, u64 val)
{

	if (val & ENABLE_DEBUG_MESSAGE) {
		
		debug_enable_flag |= ENABLE_DEBUG_MESSAGE;
	}
#if defined (CONFIG_MACH_STAR_TMUS)
	if (val /*& ENABLE_TEST_MODE*/) {
#elif defined (CONFIG_MACH_STAR_REV_F)
	if (val & ENABLE_TEST_MODE) {
#endif
		debug_enable_flag |= ENABLE_TEST_MODE;
		input_report_key(cpwatcher->input, EVENT_KEY, 1);
		input_report_key(cpwatcher->input, EVENT_KEY, 0);
	}

	if (!val) {

		debug_enable_flag = 0;
	}

	return 0;
}


static int debug_control_get(void *data, u64 *val)
{

	*val = debug_enable_flag;
	 
	return 0;
}


DEFINE_SIMPLE_ATTRIBUTE(debug_fops, 
                        debug_control_get, debug_control_set, "%llu\n");


static int __init cpwatcher_debug_init(void)
{
	debugfs_create_file("cpwatcher", (S_IRUGO | S_IWUGO),
					NULL, NULL, &debug_fops);
	return 0;
}
#endif//CONFIG_DEBUG_FS


static struct platform_device cp_device = { 
		    .name       = "cpwatcher",
			    .id     = -1, 
};


#ifdef CPW_SIMPLE_DEBUG
static NvU32 test_flag = 1;//debug with headset detect
#else
static NvU32 test_flag = 0;
#endif
module_param(test_flag, int, 0);


static void cpwatcher_get_status(NvU32 *pin)
{
	NvOdmGpioGetState(cpwatcher->hGpio, cpwatcher->hCP_status, pin);
}
#if defined (CONFIG_MACH_STAR_REV_F)
#ifdef ENABLE_CP_HARD_RESET
static void cpwatcher_HardReset_get_status(NvU32 *pin)
{
	NvOdmGpioGetState(cpwatcher->hGpio, cpwatcher->hCP_status_HardReset, pin);	
}
#endif
#endif
static void cpwatcher_irq_handler(void *dev_id)
{
	struct cpwatcher_dev *dev = cpwatcher;

	if (dev->onoff) {
		DBG("[CPW] %s()\n", __FUNCTION__);
		//NvOdmGpioGetState(cpwatcher_dev.hGpio, cpwatcher_dev.hCP_status, &pinValue);
		schedule_delayed_work(&dev->delayed_work_cpwatcher, dev->delay);
	}

	NvOdmGpioInterruptDone(cpwatcher->hGpioInterrupt);
}


#ifdef DEBUG_CPWATCHER_REPORT_INPUT
static int cpwatcher_thread(void *pd)
{
	NvU32 status = 0;


	for (;;) {
		
		cpwatcher_get_status(&status);

		DBG("[CPW] CP status: %s\n", status? "error" : "available");

		if (status) {//If High, CP error
			input_report_key(cpwatcher->input, EVENT_KEY, 1);
			input_report_key(cpwatcher->input, EVENT_KEY, 0);
			DBG("[CPW] Report Key: %d\n", EVENT_KEY);
		}
		input_sync(cpwatcher->input);

		mdelay(REPORT_DELAY);
	}

	return 0;
}
#endif


static void cpwatcher_work_func(struct work_struct *wq)
{
	struct cpwatcher_dev *dev = cpwatcher;
	NvU32 status = 0;


	if (dev->onoff) {

		cpwatcher_get_status(&status);
		DBG("[CPW] %s(), status: %d\n", __FUNCTION__, status);

		if (status) {//If High, CP error
			input_report_key(dev->input, EVENT_KEY, 1);
			input_report_key(dev->input, EVENT_KEY, 0);
			input_sync(dev->input);
			DBG("[CPW] input_report_key(): %d\n", EVENT_KEY);
		}
#if defined (CONFIG_MACH_STAR_REV_F)
#ifdef ENABLE_CP_HARD_RESET
		cpwatcher_HardReset_get_status(&status);
		printk("[CPW] %s(), Hard reset status: %d\n", __FUNCTION__, status);
		if (status==0) {//If Low, CP hard reset error
			input_report_key(dev->input, EVENT_HARD_RESET_KEY, 1);
			input_report_key(dev->input, EVENT_HARD_RESET_KEY, 0);
			input_sync(dev->input);
			printk("[CPW] input_report_key(): %d\n", EVENT_HARD_RESET_KEY);
		}
#endif
#endif
	}
}


static ssize_t
cpwatcher_show_onoff(struct device *dev, struct device_attribute *attr, char *buf)
{
	int result = 0;


	if (!cpwatcher) return 0;

	result  = snprintf(buf, PAGE_SIZE, "%s\n", (cpwatcher->onoff == TRUE)  ? "1":"0");
	
	return result;
}


static ssize_t
cpwatcher_store_onoff(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int onoff;
	
	
	if (!count)
		return -EINVAL;

	mutex_lock(&cpwatcher->lock);
	sscanf(buf, "%d", &onoff);

	if (onoff) {

		cpwatcher->onoff = TRUE;
		printk("[CPW] On\n");
	} else {

		cpwatcher->onoff = FALSE;
		printk("[CPW] Off\n");
	}
	mutex_unlock(&cpwatcher->lock);

	return count;
}


static DEVICE_ATTR(onoff, 0666, cpwatcher_show_onoff, cpwatcher_store_onoff);


static struct attribute *cpwatcher_attributes[] = {
	&dev_attr_onoff.attr,
	NULL,
};


static const struct attribute_group cpwatcher_group = {
	.attrs = cpwatcher_attributes,
};


	
#ifdef CONFIG_MACH_STAR_REV_F
static void cpwatcher_HardReset_irq_handler(void *dev_id)
{
	struct cpwatcher_dev *dev = cpwatcher;

	if (dev->onoff) {
		DBG("[CPW] %s()\n", __FUNCTION__);
		//NvOdmGpioGetState(cpwatcher_dev.hGpio, cpwatcher_dev.hCP_status, &pinValue);
		schedule_delayed_work(&dev->delayed_work_cpwatcher, dev->delay);
	}

	NvOdmGpioInterruptDone(cpwatcher->hGpioInterrupt_HardReset);
}

static int cpwatcher_probe(struct platform_device *pdev)
{
	struct cpwatcher_dev *dev; 
	struct device *dev_sys = &pdev->dev;
	int i, j;
	NvBool ret_status = 0;
	int ret = 0;
	NvU32 u32HardResetGpio = 'v' - 'a'; 
	NvU32 u32HardResetPin = 3;			//v3 -> IFX_RESET_FLAG
    const NvOdmPeripheralConnectivity *pConnectivity = NULL;

	dev = kzalloc(sizeof(struct cpwatcher_dev), GFP_KERNEL);
	if (!dev) {

		ret = -ENOMEM;
		goto fail_malloc;
	}
	cpwatcher = dev;


	/* Get the GPIO Info for Port and Pin */
    pConnectivity = NvOdmPeripheralGetGuid(CPDEVICE_GUID);

    for (i = 0, j = 0 ; i < pConnectivity->NumAddress; i++) {

        switch (pConnectivity->AddressList[i].Interface)
        {
            case NvOdmIoModule_Gpio:
			dev->port = pConnectivity->AddressList[i].Instance;
			dev->pin = pConnectivity->AddressList[i].Address;
			j++;
                break;
            default:
                break;
        }
    }
	dev->port_HardReset = u32HardResetGpio;
	dev->pin_HardReset = u32HardResetPin;


	/* Input */
	dev->input = input_allocate_device();
	if (!dev->input) {
		//printk(KERN_ERR "button.c: Not enough memory\n");
		ret = -ENOMEM;
		goto fail_input_allocate;
	}

	dev->input->name = "cp_watcher";
	set_bit(EV_KEY, dev->input->evbit);
	set_bit(EV_SYN, dev->input->evbit);
	set_bit(EVENT_KEY, dev->input->keybit);
	set_bit(EVENT_HARD_RESET_KEY, dev->input->keybit);

	ret = input_register_device(dev->input);
	if (ret) {

		goto fail_input_register;
	}

	dev->onoff= TRUE;
	dev->delay = CPWATCHER_DELAY_TIME;	
	dev->onoff_HardReset = TRUE;
	dev->delay_HardReset = CPWATCHER_DELAY_TIME;
	INIT_DELAYED_WORK(&dev->delayed_work_cpwatcher, cpwatcher_work_func);

	/* GPIO */
    dev->hGpio = NvOdmGpioOpen();
    if (!dev->hGpio) {

        printk("[CPW] %s: Error\n", __FUNCTION__);
        goto fail_gpio_open;
    }


	/* GPIO IRQ Test with Earjack 
	 * 
	 * First of all, it needs to remove headset detect feature.
	 * Comment platform_register_device regarding headset detect in board-*.c
	 * before config port and pin number to use(ex. port 6, pin 3). 
	 *
	 */ 
	if (test_flag) {

		dev->port = HEADSET_PORT;
		dev->pin = HEADSET_PIN;
	}

	printk("%s : Soft Reset Port: %d, Soft Reset Pin: %d\n",__FUNCTION__, dev->port, dev->pin);
	dev->hCP_status= NvOdmGpioAcquirePinHandle(dev->hGpio, dev->port, dev->pin);
	if (!dev->hCP_status) 
	{
		printk("[CPW] %s: Fail to get SoftReset pin Error\n", __FUNCTION__);
		goto fail_gpio_pin_handle;
	}

	NvOdmGpioConfig(dev->hGpio, dev->hCP_status, NvOdmGpioPinMode_InputData);

	ret_status = NvOdmGpioInterruptRegister(dev->hGpio, &dev->hGpioInterrupt,
	dev->hCP_status, NvOdmGpioPinMode_InputInterruptAny,
	cpwatcher_irq_handler, (void *) dev, 0);
	if (ret_status == NV_FALSE) 
	{
		printk("[CPW] %s: Fail to register softreset interrupt Error\n", __FUNCTION__);
		goto fail_gpio_int_register;
	}

	printk("%s : Hard Reset Port: %d, Hard Reset Pin: %d\n",__FUNCTION__, dev->port_HardReset, dev->pin_HardReset);
	dev->hCP_status_HardReset = NvOdmGpioAcquirePinHandle(dev->hGpio, dev->port_HardReset, dev->pin_HardReset);
	if (!dev->hCP_status_HardReset) {
		printk("[CPW] %s: Fail to get HardReset pin Error\n", __FUNCTION__);
		goto fail_gpio_pin_handle;
	}

	NvOdmGpioConfig(dev->hGpio, dev->hCP_status_HardReset, NvOdmGpioPinMode_InputData);

	ret_status = NvOdmGpioInterruptRegister(dev->hGpio, &dev->hGpioInterrupt_HardReset,
											dev->hCP_status_HardReset, NvOdmGpioPinMode_InputInterruptFallingEdge,
											cpwatcher_HardReset_irq_handler, (void *) dev, 0);
	if (ret_status == NV_FALSE) 
	{
		printk("[CPW] %s: Fail to register softreset interrupt Error\n", __FUNCTION__);
		goto fail_gpio_int_register;
	}

#ifdef CPW_CHECK_STATUS_AT_BEGINNING
	/* Check the status at the beginning */
	cpwatcher_get_status(&dev->status);
	printk("[CPW] CP status: %s\n", dev->status? "error" : "available");
	if (dev->status) {//If High, CP error
		input_report_key(dev->input, EVENT_KEY, 1);
		input_report_key(dev->input, EVENT_KEY, 0);
		input_sync(dev->input);
	}
#endif
	
	if (sysfs_create_group(&dev_sys->kobj, &cpwatcher_group)) {

		printk("[CPW] Failed to create sys filesystem\n");
	}

	mutex_init(&dev->lock);
	
#ifdef DEBUG_CPWATCHER_REPORT_INPUT
	/* Create kernel thread */
	dev->task = kthread_create(cpwatcher_thread, 0, "cpwatcher_thread");
	if (!dev->task) {

		printk("[CPW] kthread_create(): fail\n");
		goto fail_input_allocate; 
	}
	wake_up_process(dev->task);
#endif 

    printk("[CPW] CP Watcher Initialization completed\n");

	return 0;


fail_gpio_int_register:
    NvOdmGpioReleasePinHandle(dev->hGpio, dev->hCP_status);
    NvOdmGpioReleasePinHandle(dev->hGpio, dev->hCP_status_HardReset);

fail_gpio_pin_handle:
	NvOdmGpioClose(dev->hGpio);

fail_gpio_open:
	input_unregister_device(dev->input);

fail_input_register:
	input_free_device(dev->input);

fail_input_allocate:
	kfree(dev);

fail_malloc:

	return ret;
}

static int cpwatcher_remove(struct platform_device *pdev)
{
	int ret = 0;


    NvOdmGpioInterruptUnregister(cpwatcher->hGpio, cpwatcher->hCP_status, cpwatcher->hGpioInterrupt);//FIXME
    NvOdmGpioInterruptUnregister(cpwatcher->hGpio, cpwatcher->hCP_status_HardReset, cpwatcher->hGpioInterrupt_HardReset);//FIXME

    NvOdmGpioReleasePinHandle(cpwatcher->hGpio, cpwatcher->hCP_status);
    NvOdmGpioReleasePinHandle(cpwatcher->hGpio, cpwatcher->hCP_status_HardReset);

    NvOdmGpioClose(cpwatcher->hGpio);

	input_unregister_device(cpwatcher->input);

	input_free_device(cpwatcher->input);

	kfree(cpwatcher);

	DBG("[CPW] %s(): success\n", __FUNCTION__);

	return ret;
}


#else	//ENABLE_CP_HARD_RESET
static int cpwatcher_probe(struct platform_device *pdev)
{
	struct cpwatcher_dev *dev; 
	struct device *dev_sys = &pdev->dev;
	int i, j;
	NvBool ret_status = 0;
	int ret = 0;
    const NvOdmPeripheralConnectivity *pConnectivity = NULL;


	dev = kzalloc(sizeof(struct cpwatcher_dev), GFP_KERNEL);
	if (!dev) {

		ret = -ENOMEM;
		goto fail_malloc;
	}
	cpwatcher = dev;


	/* Get the GPIO Info for Port and Pin */
    pConnectivity = NvOdmPeripheralGetGuid(CPDEVICE_GUID);

    for (i = 0, j = 0 ; i < pConnectivity->NumAddress; i++) {

        switch (pConnectivity->AddressList[i].Interface)
        {
            case NvOdmIoModule_Gpio:
                dev->port = pConnectivity->AddressList[i].Instance;
                dev->pin = pConnectivity->AddressList[i].Address;
                j++;
                break;
            default:
                break;
        }
    }


	/* Input */
	dev->input = input_allocate_device();
	if (!dev->input) {
		//printk(KERN_ERR "button.c: Not enough memory\n");
		ret = -ENOMEM;
		goto fail_input_allocate;
	}

	dev->input->name = "cp_watcher";
	set_bit(EV_KEY, dev->input->evbit);
	set_bit(EV_SYN, dev->input->evbit);
	set_bit(EVENT_KEY, dev->input->keybit);


	ret = input_register_device(dev->input);
	if (ret) {

		goto fail_input_register;
	}

	dev->onoff= TRUE;
	dev->delay = CPWATCHER_DELAY_TIME;
	INIT_DELAYED_WORK(&dev->delayed_work_cpwatcher, cpwatcher_work_func);

	/* GPIO */
    dev->hGpio = NvOdmGpioOpen();
    if (!dev->hGpio) {

        printk("[CPW] %s: Error\n", __FUNCTION__);
        goto fail_gpio_open;
    }


	/* GPIO IRQ Test with Earjack 
	 * 
	 * First of all, it needs to remove headset detect feature.
	 * Comment platform_register_device regarding headset detect in board-*.c
	 * before config port and pin number to use(ex. port 6, pin 3). 
	 *
	 */ 
	if (test_flag) {

		dev->port = HEADSET_PORT;
		dev->pin = HEADSET_PIN;
	}

	DBG("[CPW] port: %d, pin: %d\n", dev->port, dev->pin);
    dev->hCP_status= NvOdmGpioAcquirePinHandle(dev->hGpio, dev->port, dev->pin);
    if (!dev->hCP_status) {

        printk("[CPW] %s: Error\n", __FUNCTION__);
        goto fail_gpio_pin_handle;
    }

    NvOdmGpioConfig(dev->hGpio, dev->hCP_status, NvOdmGpioPinMode_InputData);

    ret_status = NvOdmGpioInterruptRegister(dev->hGpio, &dev->hGpioInterrupt,
        dev->hCP_status, NvOdmGpioPinMode_InputInterruptAny,
			cpwatcher_irq_handler, (void *) dev, 0);
    if (ret_status == NV_FALSE) {

        printk("[CPW] %s: Error\n", __FUNCTION__);
        goto fail_gpio_int_register;
    }


#ifdef CPW_CHECK_STATUS_AT_BEGINNING
	/* Check the status at the beginning */
	cpwatcher_get_status(&dev->status);
	printk("[CPW] CP status: %s\n", dev->status? "error" : "available");
	if (dev->status) {//If High, CP error
		input_report_key(dev->input, EVENT_KEY, 1);
		input_report_key(dev->input, EVENT_KEY, 0);
		input_sync(dev->input);
	}
#endif
	
	if (sysfs_create_group(&dev_sys->kobj, &cpwatcher_group)) {

		printk("[CPW] Failed to create sys filesystem\n");
	}

	mutex_init(&dev->lock);
	
#ifdef DEBUG_CPWATCHER_REPORT_INPUT
	/* Create kernel thread */
	dev->task = kthread_create(cpwatcher_thread, 0, "cpwatcher_thread");
	if (!dev->task) {

		printk("[CPW] kthread_create(): fail\n");
		goto fail_input_allocate; 
	}
	wake_up_process(dev->task);
#endif 

    printk("[CPW] CP Watcher Initialization completed\n");

	return 0;


fail_gpio_int_register:
    NvOdmGpioReleasePinHandle(dev->hGpio, dev->hCP_status);

fail_gpio_pin_handle:
	NvOdmGpioClose(dev->hGpio);

fail_gpio_open:
	input_unregister_device(dev->input);

fail_input_register:
	input_free_device(dev->input);

fail_input_allocate:
	kfree(dev);

fail_malloc:

	return ret;
}


static int cpwatcher_remove(struct platform_device *pdev)
{
	int ret = 0;


    NvOdmGpioInterruptUnregister(cpwatcher->hGpio, cpwatcher->hCP_status, cpwatcher->hGpioInterrupt);//FIXME

    NvOdmGpioReleasePinHandle(cpwatcher->hGpio, cpwatcher->hCP_status);

    NvOdmGpioClose(cpwatcher->hGpio);

	input_unregister_device(cpwatcher->input);

	input_free_device(cpwatcher->input);

	kfree(cpwatcher);

	DBG("[CPW] %s(): success\n", __FUNCTION__);

	return ret;
}

#endif	//ENABLE_CP_HARD_RESET
//LGSI_BSP_CHANGE Merge from Froyo [][lgp990_gb]18042011 [End]

static struct platform_driver cpwatcher_driver = {
	.probe		= cpwatcher_probe,
	.remove		= __devexit_p(cpwatcher_remove),
	.driver		= {
		.name	= "cpwatcher",
		.owner	= THIS_MODULE,
	},
};


static int __init cpwatcher_init(void)
{
    printk("[CPW] CP Watcher Initialization started\n");

#if defined(CONFIG_DEBUG_FS)
#ifdef DEBUG_CP
	cpwatcher_debug_init();
#endif//DEBUG_CP
#endif//CONFIG_DEBUG_FS


	if (platform_device_register(&cp_device)) {

		printk("[CPW] %s(): fail\n", __FUNCTION__);
	}

	return platform_driver_register(&cpwatcher_driver);
}
module_init(cpwatcher_init);
	
		 
static void __exit cpwatcher_exit(void)
{
	platform_driver_unregister(&cpwatcher_driver);
	DBG("[CPW] %s(): success\n", __FUNCTION__);
}
module_exit(cpwatcher_exit);


MODULE_AUTHOR("LG Electronics");
MODULE_DESCRIPTION("Star CP Watcher Driver");
MODULE_LICENSE("GPL");


