#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/input.h>
#include <linux/sysfs.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/timer.h>

//20100717 sk.hwang@lge.com For HRTIMER @linux 2.6.32[start]
#include <linux/hrtimer.h>
//20100717 sk.hwang@lge.com For HRTIMER @linux 2.6.32[end]

#include "nvcommon.h"
#include "nvodm_services.h"
#include "nvodm_query.h"
#include "nvodm_query_discovery.h"

#define VIB_DEBUG 0 

typedef struct star_vib_device_data
{
	NvOdmServicesGpioHandle h_vib_gpio;
	NvOdmGpioPinHandle  h_vib_gpio_pin;
	NvOdmServicesPmuHandle  h_vib_pmu;
	struct hrtimer timer;
	
	NvU32   vdd_id;
    NvU32   en_pin;
    NvU32   en_port;
	struct delayed_work delayed_work_vib;
}star_vib_device;

static star_vib_device *g_vib;
static NvBool vib_enable = false;
static volatile int vib_stay_time = 0; 
static atomic_t is_enable = ATOMIC_INIT(0);
//static struct timer_list vib_timer;


static NvU32 toggle = 0;
static void star_vib_work_func(struct work_struct *work)
{
	NvU32 gpio_state = 0;	
	#if VIB_DEBUG 
	printk("vibrator test... value = %d\n", toggle);
	#endif

	
	NvOdmGpioConfig( g_vib->h_vib_gpio, g_vib->h_vib_gpio_pin, NvOdmGpioPinMode_Output);
	
	NvOdmGpioSetState( g_vib->h_vib_gpio, g_vib->h_vib_gpio_pin, toggle );
	if( toggle == 0 )
		toggle++;
	else	
		toggle = 0;


	NvOdmGpioGetState(g_vib->h_vib_gpio, g_vib->h_vib_gpio_pin, &gpio_state); 	
	#if VIB_DEBUG
	printk("[skhwang] VIB_EN = %d\n", gpio_state);
	#endif

	NvOdmOsSleepMS(1000);
	schedule_delayed_work(&g_vib->delayed_work_vib,150);
	
}


// 20100903 taewan.kim@lge.com Power control bug fix [START]
static int star_vib_set_power_rail( NvU32 vdd_id, NvBool is_enable )
{
    NvOdmServicesPmuHandle h_pmu = NvOdmServicesPmuOpen();
    NvOdmServicesPmuVddRailCapabilities vddrailcap;
    NvU32 settletime;

    if(h_pmu)
    {
        NvOdmServicesPmuGetCapabilities( h_pmu, vdd_id, &vddrailcap );
        if( is_enable )
        {
#if VIB_DEBUG
            printk("[skhwang] vibrator PMU enable\n");
#endif
            NvOdmServicesPmuSetVoltage(h_pmu, vdd_id, vddrailcap.requestMilliVolts, &settletime);
        }
        else
        {
#if VIB_DEBUG
            printk("[skhwang] vibrator PMU do not enable\n");
#endif
            NvOdmServicesPmuSetVoltage(h_pmu, vdd_id, NVODM_VOLTAGE_OFF, &settletime);
        }

        if(settletime)
            NvOdmOsWaitUS(settletime);

        NvOdmServicesPmuClose(h_pmu);
#if VIB_DEBUG
        printk("[skhwang] vibrator voltage =  %d or %d \n", vddrailcap.requestMilliVolts, vddrailcap.MinMilliVolts);
#endif

        return 0;
    }

    return -1;
}
// 20100903 taewan.kim@lge.com Power control bug fix [END]

static void star_vib_vibrating( NvBool on )
{
//20101112 sh80.choi@lge.com Call Vibrate Volume [START_LGE_LAB1]
#ifdef STAR_COUNTRY_KR
	NvBool vib_onoff;
	
	if(on)
		vib_onoff = true;
	else
		vib_onoff = false;

	NvOdmGpioConfig( g_vib->h_vib_gpio, g_vib->h_vib_gpio_pin, NvOdmGpioPinMode_Output);
	NvOdmGpioSetState( g_vib->h_vib_gpio, g_vib->h_vib_gpio_pin, vib_onoff );
//20101112 sh80.choi@lge.com Call Vibrate Volume [END_LGE_LAB1]
#else

	NvOdmGpioConfig( g_vib->h_vib_gpio, g_vib->h_vib_gpio_pin, NvOdmGpioPinMode_Output);
	NvOdmGpioSetState( g_vib->h_vib_gpio, g_vib->h_vib_gpio_pin, on );
#endif
}

//20101112 sh80.choi@lge.com Call Vibrate Volume [START_LGE_LAB1]
#ifdef STAR_COUNTRY_KR
NvOdmServicesPwmHandle hOdmPwm = NULL;

NvBool star_vibrator_pwn_set(NvBool IsEnable)
{
    NvU32 RequestedPeriod, ReturnedPeriod;
	NvU32 gain;
    RequestedPeriod = 22930; 

	gain = IsEnable << 16;
	
    if (!hOdmPwm) {
        pr_err("%s: failed to open NvOdmPwmOpen\n", __func__);
        return NV_FALSE;
    }
	
	if (IsEnable)
	{		
		printk("[%s] Start Enable...duty[0x%x]\n", __func__, gain);
		NvOdmPwmConfig(hOdmPwm, NvOdmPwmOutputId_PWM3, NvOdmPwmMode_Enable, gain, &RequestedPeriod, &ReturnedPeriod);
	}
	else
	{
		printk("[%s] Start Disable...duty[0x%x]\n", __func__, gain);
		NvOdmPwmConfig(hOdmPwm, NvOdmPwmOutputId_PWM3, NvOdmPwmMode_Disable, 0, &RequestedPeriod, &ReturnedPeriod);
	}
	
	return NV_TRUE;
	   
}
#endif
//20101112 sh80.choi@lge.com Call Vibrate Volume [END_LGE_LAB1]

static void star_vib_enable( NvBool is_enable )
{
	if( is_enable)
	{
        //star_vib_set_power_rail( g_vib->h_vib_pmu, g_vib->vdd_id, is_enable);
		star_vib_vibrating( is_enable );
//20101112 sh80.choi@lge.com Call Vibrate Volume [START_LGE_LAB1]
#ifdef STAR_COUNTRY_KR
		star_vibrator_pwn_set(is_enable);
#endif
//20101112 sh80.choi@lge.com Call Vibrate Volume [END_LGE_LAB1]
	}
	else
	{
        //star_vib_set_power_rail( g_vib->h_vib_pmu, g_vib->vdd_id, is_enable);
		star_vib_vibrating( is_enable );
//20101112 sh80.choi@lge.com Call Vibrate Volume [START_LGE_LAB1]
#ifdef STAR_COUNTRY_KR
		star_vibrator_pwn_set(is_enable);
#endif
//20101112 sh80.choi@lge.com Call Vibrate Volume [END_LGE_LAB1]
	}
}
/*
static void star_vib_timeout(unsigned long arg )
{
	printk("[skhwang][%s] timeout...\n", __func__);
	star_vib_enable(NV_FALSE);
	del_timer(&vib_timer);
}
*/
static enum hrtimer_restart star_vib_timeout(struct hrtimer *timer )
{
	#if VIB_DEBUG
	printk("[%s] enter timer function...\n", __func__);
	printk("vibrator off...\n");
	#endif
//20101112 sh80.choi@lge.com Call Vibrate Volume [START_LGE_LAB1]	
#ifdef STAR_COUNTRY_KR
	star_vib_vibrating(NV_FALSE);
#else
//20101112 sh80.choi@lge.com Call Vibrate Volume [END_LGE_LAB1]
	star_vib_enable(NV_FALSE);
#endif

	return HRTIMER_NORESTART;
}
static ssize_t star_vib_stay_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int time = vib_stay_time;
	return sprintf(buf, "%d\n", time);	
}

static ssize_t star_vib_stay_store(struct device *dev, struct device_attribute *attr, char *buf, size_t count)
{
	int value;
	unsigned long timeout;
	sscanf(buf, "%ld", &value );
	#if VIB_DEBUG
	printk("%s: Timeout value = %ld ms\n", __func__, value );
	#endif
	vib_stay_time = value;
	if( atomic_read(&is_enable) == 1 )
	{
		#if 0 
		timeout = (unsigned long)(value * 1000 * 1000 );
		hrtimer_start(&g_vib->timer, ktime_set(0, timeout), HRTIMER_MODE_ABS);
		#else
		if( value < 1000 )
		{
			timeout = (unsigned long)(value * 1000 * 1000 );
//20101112 sh80.choi@lge.com Call Vibrate Volume [START_LGE_LAB1]
#ifndef STAR_COUNTRY_KR
//20101112 sh80.choi@lge.com Call Vibrate Volume [END_LGE_LAB1]
			star_vib_enable(NV_TRUE);
#endif
			hrtimer_start(&g_vib->timer, ktime_set(0, timeout), HRTIMER_MODE_REL);
		}
		else
		{
			timeout = (long)value / 1000;
//20101112 sh80.choi@lge.com Call Vibrate Volume [START_LGE_LAB1]
#ifndef STAR_COUNTRY_KR
			star_vib_enable(NV_TRUE);
//20101112 sh80.choi@lge.com Call Vibrate Volume [END_LGE_LAB1]
#endif
			hrtimer_start(&g_vib->timer, ktime_set((long)timeout, 0), HRTIMER_MODE_REL);
		}
		#endif
	}

	return count;
}

static ssize_t star_vib_onoff_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	sprintf(buf, "%d\n", vib_enable==true );	
	return (ssize_t)(strlen(buf)+1);
}
//spinlock_t vib_lock;
#define MS_TO_NS(x)	(x * 1E6L)
static ssize_t star_vib_onoff_store(struct device *dev, struct device_attribute *attr, char *buf, size_t count)
{
	u32 val = 0;
	unsigned int timeout = 0;
	val = simple_strtoul(buf, NULL, 10);
	
//	timeout = vib_stay_time;
//	spin_lock(&vib_lock);
	if(val)
	{
		#if VIB_DEBUG
		printk("vibrator on...\n");	
		#endif
//20101112 sh80.choi@lge.com Call Vibrate Volume [START_LGE_LAB1]
#ifdef STAR_COUNTRY_KR
		star_vib_enable(val);
#endif
//20101112 sh80.choi@lge.com Call Vibrate Volume [END_LGE_LAB1]
		
//		star_vib_enable(NV_TRUE);
		#if VIB_DEBUG
//		printk("start timer...\n");
//		printk("app's timeout = %d\n", vib_stay_time);//timeout);
		printk("Vibrator enabled!!!\n");
		#endif
		atomic_set(&is_enable, 1);
	/*
		if( timeout == 1 )
			hrtimer_start(&g_vib->timer, ktime_set(0, timeout*9000000), HRTIMER_MODE_REL);
		else if( timeout == 0)
			hrtimer_start(&g_vib->timer, ktime_set(1,10000000), HRTIMER_MODE_REL);
		else if( timeout >=2 && timeout <=10 )
			hrtimer_start(&g_vib->timer, ktime_set(0, timeout*900000), HRTIMER_MODE_REL);
	*/
	/*
		if( timeout == 1 )
		{
			#if 1//VIB_DEBUG
			printk("timeout == %d \n", timeout );
			#endif
			hrtimer_start(&g_vib->timer, ktime_set(0, timeout*9000000), HRTIMER_MODE_REL);
		}
		else if( timeout == 0)
		{
			#if 1//VIB_DEBUG
			printk("timeout == %d \n", timeout );
			#endif
			hrtimer_start(&g_vib->timer, ktime_set(1,10000000), HRTIMER_MODE_REL);
		}
		else
		{
			#if 1//VIB_DEBUG
			printk("timeout == %d \n", timeout );
			#endif
			hrtimer_start(&g_vib->timer, ktime_set(0, timeout*1000000), HRTIMER_MODE_REL);
		}
	*/		
	}
	else
	{
		#if VIB_DEBUG
		printk("vibrator off...\n");
		#endif
		atomic_set(&is_enable, 0);
		star_vib_enable(NV_FALSE);
	}
//	spin_unlock(&vib_lock);
	return count;
}

static DEVICE_ATTR(onoff, 0666, star_vib_onoff_show, star_vib_onoff_store);
static DEVICE_ATTR(stay, 0666, star_vib_stay_show, star_vib_stay_store);

static struct attribute *star_vib_attributes[] = {
	&dev_attr_onoff.attr,	
	&dev_attr_stay.attr,
	NULL,
};

static const struct attribute_group star_vib_group = {
	.attrs = star_vib_attributes,
};

static int __init star_vib_probe(struct platform_device *pdev )
{
	const NvOdmPeripheralConnectivity *pcon;
	int err;
	NvBool found_gpio=NV_FALSE;
	int loop;
	struct device *dev = &pdev->dev;

	printk("[%s] probing vibrator\n",__func__);

	g_vib = kzalloc(sizeof(*g_vib), GFP_KERNEL );
	if( g_vib == NULL)
	{
		err = -1;
		printk("[%s] fail vib\n", __func__ );
		return err;
	}

        // 20100903 taewan.kim@lge.com Power control bug fix [START]
	/*g_vib->h_vib_pmu = NvOdmServicesPmuOpen();
	if( !g_vib->h_vib_pmu )
	{err=-ENOSYS; return err;}*/
        // 20100903 taewan.kim@lge.com Power control bug fix [END]

	pcon = (NvOdmPeripheralConnectivity*)NvOdmPeripheralGetGuid(NV_ODM_GUID('v','i','b','r','a','t','o','r'));
	for(loop = 0; loop< pcon->NumAddress; loop++)
    {
        switch(pcon->AddressList[loop].Interface)
        {
            case NvOdmIoModule_Gpio:
                g_vib->en_port = pcon->AddressList[loop].Instance;
                g_vib->en_pin = pcon->AddressList[loop].Address;
                found_gpio = NV_TRUE;
                break;
            case NvOdmIoModule_Vdd:
                {
                    g_vib->vdd_id = pcon->AddressList[loop].Address;
				#if VIB_DEBUG
                    printk("VIB POWER %d\n", g_vib->vdd_id );
				#endif
                    if (star_vib_set_power_rail( g_vib->vdd_id, NV_TRUE) != 0)
		        return -ENOSYS;
                }
                break;
            default:
                break;
        }
    }

	#if VIB_DEBUG
	printk("[skhwang][%s] : vibrator Int Port = %c, Int Pin = %d\n", __func__, (g_vib->en_port+'a'), g_vib->en_pin);
	printk("vibrator.....\n");
	#endif


	g_vib->h_vib_gpio = NvOdmGpioOpen();
    if(!g_vib->h_vib_gpio )
    {
        printk("[skhwang][%s] : Failed to open gpio\n",__func__);
        err = - ENOSYS;
        return err;
    }

    g_vib->h_vib_gpio_pin = NvOdmGpioAcquirePinHandle( g_vib->h_vib_gpio, g_vib->en_port, g_vib->en_pin );
    if(!g_vib->h_vib_gpio_pin)
    {
        printk("[skhwang][%s] : Failed to acquire the pin handle\n",__func__);
        err = -ENOSYS;
        return err;
    }

//20101112 sh80.choi@lge.com Call Vibrate Volume [START_LGE_LAB1]
#ifdef STAR_COUNTRY_KR
	hOdmPwm = NvOdmPwmOpen();
#endif
//20101112 sh80.choi@lge.com Call Vibrate Volume [END_LGE_LAB1]
	
//	NvOdmGpioConfig( g_vib->vib_gpio, g_vib->en_port, NvOdmGpioPinMode_Output);
//	INIT_DELAYED_WORK(&g_vib->delayed_work_vib, star_vib_work_func );
//	schedule_delayed_work(&g_vib->delayed_work_vib, 400);	
	hrtimer_init(&g_vib->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	g_vib->timer.function = star_vib_timeout;
	
	#if VIB_DEBUG
	printk("[vib] sysfs_create_group before....\n");
	#endif

	//create sys filesystem.
	if( sysfs_create_group(&dev->kobj, &star_vib_group))
	{
		printk("[star vib] Failed to create sys filesystem\n");
		err = -ENOSYS;
		return err;
	}

	return 0;
}

int star_vib_suspend(struct platform_device *dev, pm_message_t state)
{
	//printk("[SLEEP] %s start \n", __func__);
	star_vib_set_power_rail( g_vib->vdd_id, NV_FALSE);

    return 0;
}

int star_vib_resume(struct platform_device *dev)
{
	//printk("[SLEEP] %s start \n", __func__);
	star_vib_set_power_rail( g_vib->vdd_id, NV_TRUE);

    return 0;
}


static int star_vib_remove( struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver star_vib_driver = {
    .probe = star_vib_probe,
    .remove = star_vib_remove,
    .suspend = star_vib_suspend,
    .resume = star_vib_resume,    
    .driver = {
        .name = "star_vib_name",
    },
};

static int __init star_vib_init(void)
{
    return platform_driver_register(&star_vib_driver);
}

static void __exit star_vib_exit(void)
{
    platform_driver_unregister(&star_vib_driver);
}

module_init(star_vib_init);
module_exit(star_vib_exit);

MODULE_AUTHOR("sk.hwang@lge.com");
MODULE_DESCRIPTION("driver of star viberator");
MODULE_LICENSE("GPL");
