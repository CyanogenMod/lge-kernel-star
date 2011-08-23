
/*
 * Star - CP Watcher Driver
 *
 * Copyright (C) 2010 LGE, Inc.
 *
 *
 * Author: Kyungsik Lee <kyungsik.lee@lge.com>
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

#if defined (CONFIG_STAR_HIDDEN_RESET)
#include <asm/uaccess.h>
#include <linux/fs.h> // for _IOC_XXX define
#include <linux/miscdevice.h>

#define CPWATCHER_MINOR  MISC_DYNAMIC_MINOR  
#define CPWATCHER_NAME   "cpwatcher"
#define RGB888_BUFFER_SIZE 800*480*3
#define CPWATCHER_MAGIC    'c'  
typedef struct 
{     
	unsigned long size;      
	unsigned char *buf;  
} __attribute__ ((packed)) cpwatcher_info;

#define CPWATCHER_REBOOT	 _IO( CPWATCHER_MAGIC, 0 )  
#define CPWATCHER_READ   _IOR( CPWATCHER_MAGIC, 1 , unsigned char* )  
#define CPWATCHER_WRITE  _IOW( CPWATCHER_MAGIC, 2 , unsigned char* )  
#define CPWATCHER_MAXNR 					   3

extern void write_screen_shot_reserved_buffer(unsigned char *buf, size_t len);
extern void read_screen_shot_reserved_buffer(unsigned char *buf, size_t len);
#endif

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
#endif

#define	HEADSET_PORT 6
#define	HEADSET_PIN 3


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
	struct delayed_work delayed_work_cpwatcher;
	struct mutex lock;
#ifdef DEBUG_CPWATCHER_REPORT_INPUT
	struct task_struct  *task;
#endif

};
static struct cpwatcher_dev *cpwatcher;


#if defined (CONFIG_STAR_HIDDEN_RESET)
extern void machine_restart(char *cmd);
#endif

#if defined(CONFIG_DEBUG_FS)
static int debug_control_set(void *data, u64 val)
{

	if (val & ENABLE_DEBUG_MESSAGE) {
		
		debug_enable_flag |= ENABLE_DEBUG_MESSAGE;
	}

	if (val /*& ENABLE_TEST_MODE*/) {

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

		if (status) 
		{//If High, CP error

// LGE_UPDATE_S eungbo.shim@lge.com 20110609 -- For RIL Recovery !! [EBS]
			printk("###### CP RESET ####### Push the key 202 to Framework \n");
// LGE_UPDATE_E eungbo.shim@lge.com 20110609 -- For RIL Recovery !! [EBS]

			input_report_key(dev->input, EVENT_KEY, 1);
			input_report_key(dev->input, EVENT_KEY, 0);
			input_sync(dev->input);
			DBG("[CPW] input_report_key(): %d\n", EVENT_KEY);
		}
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

#if defined (CONFIG_STAR_HIDDEN_RESET)

int cpwatcher_open (struct inode *inode, struct file *filp)  
{      
	 printk("%s\n", __FUNCTION__);

     return 0;  
}    

int cpwatcher_release (struct inode *inode, struct file *filp)  
{
	printk("%s\n", __FUNCTION__);
	
    return 0;  
}  

int cpwatcher_ioctl (struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg)  
{      
	cpwatcher_info	ctrl_info;      
	int  err, size;      
	int loop;      
	NvU8 ret;	    

	void __user *usrbuf;
	usrbuf = (void __user *) arg;

	switch( cmd )      
	{
	  case CPWATCHER_READ:            
	  {                
	  	printk("CPWATCHER_READ\n");
		read_screen_shot_reserved_buffer(usrbuf, 800*480*3);
		break;              
	  }
	  case CPWATCHER_WRITE:            
	  {                
		  printk("CPWATCHER_WRITE\n"); 	
		  write_screen_shot_reserved_buffer(usrbuf, 800*480*3);
		  break;            
	  }	  
	  case CPWATCHER_REBOOT:            
	  {                
		  printk("CPWATCHER_REBOOT\n"); 	
		  machine_restart("hidden");
		  break;            
	  }	  
	}	     
	return 0;  
}
 
struct file_operations cpwatcher_fops =  {
	 .owner    = THIS_MODULE,      
	 .ioctl    = cpwatcher_ioctl,      
	 .open     = cpwatcher_open,           
	 .release  = cpwatcher_release,    
};  

static struct miscdevice cpwatcher_miscdev = {
	.minor = CPWATCHER_MINOR,     
	.name  = CPWATCHER_NAME,     
	.fops       = &cpwatcher_fops,
};
#endif
	


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

// LGE_UPDATE_S eungbo.shim@lge.com 20110609 -- For RIL Recovery !! [EBS]
#if defined(EBS) 
    ret_status = NvOdmGpioInterruptRegister(dev->hGpio, &dev->hGpioInterrupt,
        dev->hCP_status, NvOdmGpioPinMode_InputInterruptAny,
			cpwatcher_irq_handler, (void *) dev, 0);

#else
	ret_status = NvOdmGpioInterruptRegister(dev->hGpio, &dev->hGpioInterrupt,
        dev->hCP_status, NvOdmGpioPinMode_InputInterruptRisingEdge,
			cpwatcher_irq_handler, (void *) dev, 0);

#endif 
// LGE_UPDATE_E eungbo.shim@lge.com 20110609 -- For RIL Recovery !! [EBS]

    if (ret_status == NV_FALSE) {

        printk("[CPW] %s: Error\n", __FUNCTION__);
        goto fail_gpio_int_register;
    }


#ifdef CPW_CHECK_STATUS_AT_BEGINNING
	/* Check the status at the beginning */
	cpwatcher_get_status(&dev->status);
	printk("[CPW] CP status: %s\n", dev->status? "error" : "available");
	if (dev->status) {//If High, CP error
#if defined (CONFIG_STAR_HIDDEN_RESET)
                machine_restart("hidden");
#else
		input_report_key(dev->input, EVENT_KEY, 1);
		input_report_key(dev->input, EVENT_KEY, 0);
		input_sync(dev->input);
#endif
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
#if defined (CONFIG_STAR_HIDDEN_RESET)
    int retval = 0;      
	retval = misc_register(&cpwatcher_miscdev);     
	if (retval < 0) printk("misc_driver_register failed!\n");
#endif
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


