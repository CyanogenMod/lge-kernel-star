/*
 * Star - Headset Detection Driver
 *
 * Copyright (C) 2010 LGE, Inc.
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

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/string.h>
#include <asm/gpio.h>
#include <asm/system.h>
#include <linux/interrupt.h>
#include <linux/switch.h>
#include <linux/workqueue.h>
#include <linux/input.h>

#include "headset_det.h"
#include "nvcommon.h"
#include "nvrm_init.h"
#include "nvrm_interrupt.h"
#include "nvrm_power.h"
#include "nvodm_query.h"
#include "nvodm_services.h"
#include "mach/nvrm_linux.h"
#include "nvodm_query_discovery.h"
#include "mach/lprintk.h"

#if !defined (STAR_COUNTRY_KR) //Global
#if defined(CONFIG_MACH_STAR)//jongik2.kim 20100910 i2c_fix 
#define HOOK_USE_ADC 1
#else
#define HOOK_USE_ADC 0
#endif
#else
#if !defined(CONFIG_MACH_STAR_SKT_REV_A)
#define HOOK_USE_ADC 1
#else
#define HOOK_USE_ADC 0
#endif
#endif
//20101005  STAR_COUNTRY_KR Add [END_LGE_LAB1]

struct headset_switch_data	*headset_sw_data;
static int headset_off = 0;

unsigned int headset_status = 0;
unsigned int headset_gpio_status = 0;
unsigned int hookkey_gpio_status = 0;
unsigned int headset_detecting = 0;

headset_type_enum headset_type = STAR_NONE;
unsigned int hook_status = HOOK_RELEASED;
bool block_hook_int = false;
bool is_hook_test = false;

struct headset_switch_data {
	struct switch_dev sdev;
	unsigned gpio;
	unsigned hook_gpio;
	const char *name_on;
	const char *name_off;
	const char *state_on;
	const char *state_off;
	int irq;
	int hook_irq;
	struct work_struct work;
	struct delayed_work delayed_work;
	struct delayed_work hook_delayed_work;
	struct input_dev *ip_dev;	//20100421  for headset driver [LGE]
	struct input_dev *ip_dev_wake;
};
//jongik2.kim 20100910 i2c_fix [start]
extern NvBool star_get_i2c_busy();
extern void star_set_i2c_busy();
extern void star_unset_i2c_busy();
//jongik2.kim 20100910 i2c_fix [end]
unsigned int get_headset_type(void)
{
    return (unsigned int)headset_type;
}
EXPORT_SYMBOL(get_headset_type);

int type_detection_time = 700; 
int remove_detection_time = 60; 
#if !defined(STAR_COUNTRY_KR) //20101103  heaset detect time[LGE_LAB1]
int hook_press_time = 100; 
int hook_release_time = 20; 
#else
int hook_press_time = 100; 
int hook_release_time = 20; 
#endif

//20101117, , gpio wakeup from LP1 [START]
#include <linux/wakelock.h>
extern bool core_lock_on;
NvU32 headset_vdd_address = 0;
NvU32 headset_vdd_voltage = 0;
NvOdmServicesPmuHandle headset_h_pmu;
int suspend_status = 0;
//static struct wake_lock hook_det_lock;
//20101117, , gpio wakeup from LP1 [END]


extern void star_headsetdet_bias(int bias);	//20100421  for framwork function used in kernel ==> error [LGE]
extern void star_Mic_bias(int bias); // 20110726 detecting headset when resuming
#define HEADSET_GUID                NV_ODM_GUID('h','e','a','d','s','e','t','_')
typedef struct HeadsetDeviceRec
{
    NvOdmServicesI2cHandle hOdmI2c;
    NvOdmServicesGpioHandle hGpio;
    NvOdmGpioPinHandle h_Headset_Detection;
    NvOdmGpioPinHandle h_Hookkey_Detection; 
#if defined(STAR_COUNTRY_KR)&& !defined(CONFIG_MACH_STAR_SKT_REV_A)//20101005  Gpio MicBias[LGE_LAB1]
    NvOdmGpioPinHandle h_Headset_MicBias; 
#endif
    NvOdmServicesGpioIntrHandle hheadsetInterrupt;
	NvOdmServicesGpioIntrHandle hhookInterrupt;
    NvU32 DeviceAddr;    
    NvU32 VddId;     
} HeadsetDevice;
static HeadsetDevice s_hHeadsetHandle;

//20101005  Gpio MicBias[START_LGE_LAB1]
#if defined(STAR_COUNTRY_KR)&& !defined(CONFIG_MACH_STAR_SKT_REV_A)
void headset_Mic_Bias(int on)
{
	NvOdmGpioSetState(s_hHeadsetHandle.hGpio, s_hHeadsetHandle.h_Headset_MicBias, on);
}
#endif
//20101005  Gpio MicBias[END_LGE_LAB1]

NvU32 headset_get_hook_adc_value(void)
{
    NvU32 value =0;
    NvOdmServicesPmuHandle adc_pmu = NvOdmServicesPmuOpen();
        
    value = NvOdmServicesPmuGetHookValue(adc_pmu);		//20100716  blocking for compile error [LGE]
    
    NvOdmServicesPmuClose(adc_pmu);
	return value;
}

NvU32 headset_get_hook_adc_average(int cnt)
{
    NvU32 value =0;
    NvU32 aver =0;
    NvU32 i =0;
    
    NvOdmServicesPmuHandle adc_pmu = NvOdmServicesPmuOpen();
    
    for(i=0;i<cnt;i++){    
        value += NvOdmServicesPmuGetHookValue(adc_pmu);
    }
    
    if(cnt !=0 )
        aver = value/cnt;
    else
        aver = value;
    
    NvOdmServicesPmuClose(adc_pmu);
    lprintk(D_AUDIO, KERN_ERR "##(hook_det_work)## headset_get_hook_adc_average ADC: %d\n", aver);
	return aver;
}

static void headset_det_work(struct work_struct *work)
{
    if (headset_off) return;
	NvOdmGpioGetState(s_hHeadsetHandle.hGpio, s_hHeadsetHandle.h_Headset_Detection, &headset_gpio_status);	//20100419  for nVidia headset driver [LGE]
	headset_status = headset_gpio_status;

	lprintk(D_AUDIO, KERN_ERR "@@(Headset_det.c)@@ headset_det_work(), headset_status = %d\n",headset_status);

   if(headset_status == 0)
   {
        schedule_delayed_work(&headset_sw_data->delayed_work,	msecs_to_jiffies(remove_detection_time));
	#if defined(STAR_COUNTRY_KR) && !defined(CONFIG_MACH_STAR_SKT_REV_A)	  
		 headset_Mic_Bias(0);
	#endif

   }
	else
	{
	     schedule_delayed_work(&headset_sw_data->delayed_work,	msecs_to_jiffies(type_detection_time));
		  
	}
}

//20101005  STAR_COUNTRY_KR Add [START_LGE_LAB1]
#if !defined(STAR_COUNTRY_KR)//Global
static void type_det_work(struct work_struct *work)
{
    NvU32 hook_value =0;
    if (headset_off) return;
	
    if(star_get_i2c_busy())
    {
        schedule_delayed_work(&headset_sw_data->delayed_work,	msecs_to_jiffies(type_detection_time));	
		lprintk(D_AUDIO, KERN_ERR "@@(Headset_det.c)@@ type_det_work(), i2c is busy\n");
		return;
    }
    star_set_i2c_busy();
	
    NvOdmGpioGetState(s_hHeadsetHandle.hGpio, s_hHeadsetHandle.h_Headset_Detection, &headset_gpio_status);	
	headset_status = headset_gpio_status;
    
	if( (headset_status == 1) && (headset_type == STAR_NONE))
	{
	    star_headsetdet_bias(1);	
	    #if HOOK_USE_ADC
		hook_value = headset_get_hook_adc_value();	
		
        if(hook_value > 100)
			headset_type = STAR_HEADSET;	
		else
			headset_type = STAR_HEADPHONE;
       
	    #else
        NvOdmGpioGetState(s_hHeadsetHandle.hGpio, s_hHeadsetHandle.h_Hookkey_Detection, &hookkey_gpio_status);	
		if(hookkey_gpio_status == 0){ 
            headset_type = STAR_HEADPHONE;
			hook_value = 1111;
		}
		else{
            headset_type = STAR_HEADSET;
			hook_value = 2222;
		}
		#endif
	}
	else if(headset_status == 0){
		headset_type = STAR_NONE;
        hook_status = HOOK_RELEASED; 
		input_report_key(headset_sw_data->ip_dev, KEY_HOOK, 0);
		input_sync(headset_sw_data->ip_dev);
    }

	lprintk(D_AUDIO, KERN_ERR "@@(Headset_det.c)@@ type_det_work(), HOOKVALUE %d headset_type = %d\n", hook_value, headset_type);

	if(headset_type != STAR_HEADSET){
		block_hook_int =1;
		star_headsetdet_bias(0);	//20100419   for Headset MIC Bias  ==> framwork function used in kernel ==> error [LGE]
	}
	else{
        block_hook_int =0;
	}
	star_unset_i2c_busy();
	
	headset_detecting = 0;
	
	switch_set_state(&headset_sw_data->sdev, headset_type);
	
}
#else //STAR_COUNTRY_KR
static void type_det_work(struct work_struct *work)
{
    NvU32 hook_value =0;
	if (headset_off) return;

    NvOdmGpioGetState(s_hHeadsetHandle.hGpio, s_hHeadsetHandle.h_Headset_Detection, &headset_gpio_status);	
	headset_status = headset_gpio_status;
    
	if( (headset_status == 1) && (headset_type == STAR_NONE))	
	{
#if defined(CONFIG_MACH_STAR_SKT_REV_A)  
		 if(star_get_i2c_busy())
		 {
			  schedule_delayed_work(&headset_sw_data->delayed_work,	msecs_to_jiffies(type_detection_time));	
			lprintk(D_AUDIO, KERN_ERR "@@(Headset_det.c)@@ type_det_work(), i2c is busy\n");
			return;
		 }
		 star_set_i2c_busy();
		star_headsetdet_bias(1);
#else
		headset_Mic_Bias(1);
#endif

	#if HOOK_USE_ADC
		hook_value = headset_get_hook_adc_value();	

		if(hook_value > 350) //20101127  detect adc 1200==>350[LGE_LAB1]
			headset_type = STAR_HEADSET;	
		else
			headset_type = STAR_HEADPHONE;
	#else
		NvOdmGpioGetState(s_hHeadsetHandle.hGpio, s_hHeadsetHandle.h_Hookkey_Detection, &hookkey_gpio_status);	// 20100419 for nVidia headset driver
		if(hookkey_gpio_status == 0){ 
		   headset_type = STAR_HEADPHONE;
			hook_value = 1111;
		}
		else{
		   headset_type = STAR_HEADSET;
			hook_value = 2222;
		}
	#endif
	}
	else if(headset_status == 0){
		headset_type = STAR_NONE;
        hook_status = HOOK_RELEASED; 
		input_report_key(headset_sw_data->ip_dev, KEY_HOOK, 0);
		input_sync(headset_sw_data->ip_dev);
    }

	lprintk(D_AUDIO, KERN_ERR "@@(Headset_det.c)@@ type_det_work(), HOOKVALUE %d headset_type = %d\n", hook_value, headset_type);
#if defined(CONFIG_MACH_STAR_SKT_REV_A)  
	if(headset_type != STAR_HEADSET){
		block_hook_int =1;
		star_headsetdet_bias(0);	//20100419   for Headset MIC Bias  ==> framwork function used in kernel ==> error [LGE]
	}
	else{
        block_hook_int =0;
	}
	star_unset_i2c_busy();
#else
	if(headset_type != STAR_HEADSET){
        block_hook_int =1;
		headset_Mic_Bias(0);
	}
    else{
        block_hook_int =0;
	}
#endif
	headset_detecting = 0;

	switch_set_state(&headset_sw_data->sdev, headset_type); 
}
#endif
//20101005  STAR_COUNTRY_KR Add [END_LGE_LAB1]
static void hook_det_work(struct work_struct *work)
{
    int hok_adc_value =0;
    if (headset_off) return;
    if(headset_type != STAR_HEADSET)
        return;
	
    if(is_hook_test == false){
        hok_adc_value = 20;
    }
    else{
        lprintk(D_AUDIO, KERN_ERR "##(hook_det)## IS FACTORYMODE\n");
        hok_adc_value = 65;
    }
    
	if(hook_status == HOOK_RELEASED){
		#if HOOK_USE_ADC
        hookkey_gpio_status = headset_get_hook_adc_average(5);
		if( hookkey_gpio_status <= hok_adc_value )
		{
		    hook_status = HOOK_PRESSED; 
		    input_report_key(headset_sw_data->ip_dev, KEY_HOOK, 1);
			input_sync(headset_sw_data->ip_dev);
			lprintk(D_AUDIO, KERN_ERR "##(hook_det_work)## HOOK PRESSED ADC %d\n", hookkey_gpio_status);
		}
		#else
		NvOdmGpioGetState(s_hHeadsetHandle.hGpio, s_hHeadsetHandle.h_Hookkey_Detection, &hookkey_gpio_status);	// 20100419 for nVidia headset driver
		if(hookkey_gpio_status == 0){ 
		    hook_status = HOOK_PRESSED; 
		    input_report_key(headset_sw_data->ip_dev, KEY_HOOK, 1);
			input_sync(headset_sw_data->ip_dev);
			lprintk(D_AUDIO, KERN_ERR "##(hook_det_work)## HOOK PRESSED\n");
		}
		#endif
    }
	else{
		#if HOOK_USE_ADC
        hookkey_gpio_status = headset_get_hook_adc_value();
		if(hookkey_gpio_status > hok_adc_value)
		{ 
		    hook_status = HOOK_RELEASED; 
		    input_report_key(headset_sw_data->ip_dev, KEY_HOOK, 0);
			input_sync(headset_sw_data->ip_dev);
			lprintk(D_AUDIO, KERN_ERR "##(hook_det_work)## HOOK RELEASED ADC %d\n", hookkey_gpio_status);
		}
		#else
		NvOdmGpioGetState(s_hHeadsetHandle.hGpio, s_hHeadsetHandle.h_Hookkey_Detection, &hookkey_gpio_status);	// 20100419 for nVidia headset driver
		if(hookkey_gpio_status == 1){ 
		    hook_status = HOOK_RELEASED; 
		    input_report_key(headset_sw_data->ip_dev, KEY_HOOK, 0);
			input_sync(headset_sw_data->ip_dev);
			lprintk(D_AUDIO, KERN_ERR "##(hook_det_work)## HOOK RELEASED\n");
		}
		#endif
	}
    
}

static void headset_int_handler(void *dev_id)
{
	lprintk(D_AUDIO, KERN_ERR "##(Headset_det.c)## headset_int_handler()!!\n");

	struct headset_switch_data *switch_data =
		(struct headset_switch_data *)dev_id;
	headset_detecting = 1;
	cancel_delayed_work_sync(&switch_data->hook_delayed_work);
	//schedule_work(&switch_data->work);
	NvOdmGpioGetState(s_hHeadsetHandle.hGpio, s_hHeadsetHandle.h_Headset_Detection, &headset_gpio_status);	
	headset_status = headset_gpio_status;

	if(headset_status == 0)
   	{
        schedule_work(&headset_sw_data->delayed_work);
		  #if defined(STAR_COUNTRY_KR) && !defined(CONFIG_MACH_STAR_SKT_REV_A)  
			headset_Mic_Bias(0);
		  #endif
   	}
	else
	{
	    schedule_delayed_work(&headset_sw_data->delayed_work,	msecs_to_jiffies(type_detection_time));	
    }
	NvOdmGpioInterruptDone(s_hHeadsetHandle.hheadsetInterrupt);	//20100420  for next interrupt (nVidia Interrupt Spec.) [LGE]
}

unsigned int hook_gpio =0;

static void headset_hook_int_handler(void *dev_id)
{
    
	//lprintk(D_AUDIO, KERN_ERR "##(Headset_det.c)## headset_hook_int_handler()!! detecting %d headset type %d\n", headset_detecting, headset_type );
    NvOdmGpioGetState(s_hHeadsetHandle.hGpio, s_hHeadsetHandle.h_Hookkey_Detection, &hook_gpio);

    if( block_hook_int ){
		NvOdmGpioInterruptDone(s_hHeadsetHandle.hhookInterrupt);
        return;
	}
	
	struct headset_switch_data	*switch_data =
	    (struct headset_switch_data *)dev_id;

	lprintk(D_AUDIO, KERN_ERR "##(Headset_det.c)## headset_hook_int_handler()!! detecting %d headset type %d\n", headset_detecting, headset_type );
    
	if( (headset_detecting == 1) || (headset_type != STAR_HEADSET)){
		NvOdmGpioInterruptDone(s_hHeadsetHandle.hhookInterrupt);
	}
	else{
		//lprintk(D_AUDIO, KERN_ERR "##(Headset_det.c)## headset_hook_int_handler()!!\n");
		//20101125, , hookkey press is skipped When wakeup from LP1 [START]
        if(suspend_status){
            schedule_work(&switch_data->hook_delayed_work);
        }else{
        //20101125, , hookkey press is skipped When wakeup from LP1 [END]
        
            if(hook_gpio){
                schedule_delayed_work(&switch_data->hook_delayed_work,	msecs_to_jiffies(hook_release_time));
            }
            else{
		        schedule_delayed_work(&switch_data->hook_delayed_work,	msecs_to_jiffies(hook_press_time));
            }
        
        }
        NvOdmGpioInterruptDone(s_hHeadsetHandle.hhookInterrupt);	
	}
}

static ssize_t switch_gpio_print_state(struct switch_dev *sdev, char *buf)
{

	lprintk(D_AUDIO, KERN_ERR "##(Headset_det.c)## switch_gpio_print_state() => sdev->state(%d), headset_sw_data->state_on(%d)/state_off(%d)\n",sdev->state,headset_sw_data->state_on,headset_sw_data->state_off);

	const char *state;
	if (switch_get_state(sdev))
		state = headset_sw_data->state_on;
	else
		state = headset_sw_data->state_off;

	if (state)
		return sprintf(buf, "%s\n", state);
	return -1;
}

static ssize_t headset_detect_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int r =0;

	switch(headset_type)
	{
        case STAR_HEADSET : r += sprintf(buf+r, "1\n");
			 break;
		case STAR_HEADPHONE : r += sprintf(buf+r, "2\n");
			 break;
		case STAR_NONE : r += sprintf(buf+r, "0\n");
			 break;
		default : r += sprintf(buf+r, "headset type : ERROR\n");
			 break;
	}
		
	return r;
}

ssize_t headset_detect_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    if (headset_off) return count;
		
    headset_det_work(&headset_sw_data->work);
	return count;
}

static DEVICE_ATTR(detect, 0666, headset_detect_show, headset_detect_store);

static ssize_t block_hook_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%u\n", block_hook_int);
}

ssize_t block_hook_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    unsigned int n, lock;

	n = sscanf(buf, "%u", &lock);
	if (n != 1)
		return -1;

	block_hook_int = (bool)lock;
	return count;
}

static DEVICE_ATTR(block_hook, 0666, block_hook_show, block_hook_store);


//for factory hook test [start]
static ssize_t factory_test_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%u\n", is_hook_test);
}

ssize_t factory_test_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    unsigned int n, lock;

	n = sscanf(buf, "%u", &lock);
	if (n != 1)
		return -1;

	is_hook_test = (bool)lock;
	return count;
}
//for factory hook test [end]

static DEVICE_ATTR(factory_test, 0666, factory_test_show, factory_test_store);


static int headsetdet_probe(struct platform_device *pdev)
{
	struct gpio_switch_platform_data *pdata = pdev->dev.platform_data;
	struct headset_switch_data *switch_data;
	int ret = 0;
	NvS32 err = 0;

	struct input_dev *ip_dev;  
    struct input_dev *ip_dev_wake; 

    int i, j;
    NvU32 I2cInstance = 0;
    NvU32 pin[4], port[4];
    const NvOdmPeripheralConnectivity *pConnectivity = NULL;

    pConnectivity = NvOdmPeripheralGetGuid(HEADSET_GUID);

    for (i = 0, j = 0 ; i < pConnectivity->NumAddress; i++)
    {
        switch (pConnectivity->AddressList[i].Interface)
        {
            case NvOdmIoModule_Gpio:
                port[j] = pConnectivity->AddressList[i].Instance;
                pin[j] = pConnectivity->AddressList[i].Address;
                j++;
                break;
            //20101117, , gpio wakeup from LP1 [START]
            case NvOdmIoModule_Vdd:
                {
                    NvOdmServicesPmuVddRailCapabilities vddrailcap;

                    headset_h_pmu = NvOdmServicesPmuOpen();
                    headset_vdd_address = pConnectivity->AddressList[i].Address;
                    NvOdmServicesPmuGetCapabilities(headset_h_pmu, pConnectivity->AddressList[i].Address, &vddrailcap);
                    headset_vdd_voltage = vddrailcap.requestMilliVolts;
                }
                break;
            //20101117, , gpio wakeup from LP1 [END]
            default:
                break;
        }
    }

	if (!pdata)
		return -EBUSY;
	switch_data = kzalloc(sizeof(struct headset_switch_data), GFP_KERNEL);
	if (!switch_data)
		return -ENOMEM;

	switch_data->sdev.name = pdata->name;
	switch_data->gpio = pdata->gpio;
	switch_data->name_on = pdata->name_on;
	switch_data->name_off = pdata->name_off;
	switch_data->state_on = pdata->state_on;
	switch_data->state_off = pdata->state_off;
	switch_data->sdev.print_state = switch_gpio_print_state;


	lprintk(D_AUDIO, KERN_ERR "##(Headset_det.c)## headsetdet_probe() => headset detection started..!!\n");	//20100421  [LGE]

    ret = switch_dev_register(&switch_data->sdev);	//20100421  Headset Detection by Headset Observer [LGE]
	if (ret < 0)
		goto err_switch_dev_register;

	lprintk(D_AUDIO, KERN_ERR "##(Headset_det.c)## headset detection - switch device registered..!!\n");	//20100421  [LGE]


/*====================== nVidia GPIO Control(S) =======================*/

    s_hHeadsetHandle.hGpio = NvOdmGpioOpen();
    if (!s_hHeadsetHandle.hGpio)
    {
        lprintk(D_AUDIO, "%s: NvOdmGpioOpen Error \n", __func__);
        goto err_open_gpio_fail;
    }
	lprintk(D_AUDIO, KERN_ERR "##(Headset_det.c)## headset detection - NvOdmGpioOpen() success..!! : s_hHeadsetHandle.hGpio = %d, port[0] = %d, pin[0] = %d\n", s_hHeadsetHandle.hGpio, port[0], pin[0]);	//20100421  [LGE]


    s_hHeadsetHandle.h_Headset_Detection = NvOdmGpioAcquirePinHandle(s_hHeadsetHandle.hGpio, port[0], pin[0]);
    if (!s_hHeadsetHandle.h_Headset_Detection)
    {
        lprintk(D_AUDIO, "%s: Couldn't NvOdmGpioAcquirePinHandle  pin \n", __func__);
        goto err_open_gpio_pin_acquire_fail;
    }
	lprintk(D_AUDIO, KERN_ERR "##(Headset_det.c)## headset detection - NvOdmGpioAcquirePinHandle() success..!!\n");	//20100421  [LGE]


    NvOdmGpioConfig(s_hHeadsetHandle.hGpio, s_hHeadsetHandle.h_Headset_Detection, NvOdmGpioPinMode_InputData/*NvOdmGpioPinMode_InputInterruptHigh*/);
	lprintk(D_AUDIO, KERN_ERR "##(Headset_det.c)## headset detection - NvOdmGpioConfig() success..!! - NvOdmGpioPinMode_InputData");	//20100421  [LGE]



    s_hHeadsetHandle.h_Hookkey_Detection = NvOdmGpioAcquirePinHandle(s_hHeadsetHandle.hGpio, port[1], pin[1]);
    if (!s_hHeadsetHandle.h_Hookkey_Detection)
    {
        lprintk(D_AUDIO, "%s:Couldn't NvOdmGpioAcquirePinHandle  pin \n", __func__);
        goto err_open_gpio_pin_acquire_fail;
    }

    NvOdmGpioConfig(s_hHeadsetHandle.hGpio, s_hHeadsetHandle.h_Hookkey_Detection, NvOdmGpioPinMode_InputData);
	ip_dev = input_allocate_device();
    
	switch_data->ip_dev = ip_dev;
	set_bit(EV_SYN, switch_data->ip_dev->evbit);
	set_bit(EV_KEY, switch_data->ip_dev->evbit);
	set_bit(KEY_HOOK, switch_data->ip_dev->keybit); 
	switch_data->ip_dev->name = "star_headset_hook";

    ip_dev_wake = input_allocate_device();
    switch_data->ip_dev_wake = ip_dev_wake;
	set_bit(EV_SYN, switch_data->ip_dev_wake->evbit);
	set_bit(EV_KEY, switch_data->ip_dev_wake->evbit);
	set_bit(KEY_VOLUMEDOWN, switch_data->ip_dev_wake->keybit); 
	switch_data->ip_dev_wake->name = "star_headset_wake";

		
	ret = input_register_device(switch_data->ip_dev);  
	INIT_DELAYED_WORK(&switch_data->hook_delayed_work, hook_det_work);
    if (NvOdmGpioInterruptRegister(s_hHeadsetHandle.hGpio, &s_hHeadsetHandle.hhookInterrupt,
        s_hHeadsetHandle.h_Hookkey_Detection, NvOdmGpioPinMode_InputInterruptAny, headset_hook_int_handler,
        switch_data, 0) == NV_FALSE)
    {
        lprintk(D_AUDIO, KERN_ERR "%s: cannot register interrupt.\n", __func__);
        goto err_get_interrupt_handler;
    }

	block_hook_int =1;
//20101005  Gpio MicBias[START_LGE_LAB1]
#if defined(STAR_COUNTRY_KR)&& !defined(CONFIG_MACH_STAR_SKT_REV_A)
    s_hHeadsetHandle.h_Headset_MicBias = NvOdmGpioAcquirePinHandle(s_hHeadsetHandle.hGpio, port[2], pin[2]);
    if (!s_hHeadsetHandle.h_Headset_MicBias)
    {
        lprintk(D_AUDIO, "%s: Couldn't NvOdmGpioAcquirePinHandle  pin \n", __func__);
        goto err_open_gpio_pin_acquire_fail;
    }
	lprintk(D_AUDIO, KERN_ERR "##(Headset_det.c)## headset MicBias - NvOdmGpioAcquirePinHandle() success..!!\n");	//20100421  [LGE]

    NvOdmGpioConfig(s_hHeadsetHandle.hGpio, s_hHeadsetHandle.h_Headset_MicBias, NvOdmGpioPinMode_Output);
	lprintk(D_AUDIO, KERN_ERR "##(Headset_det.c)## headset MicBias- NvOdmGpioConfig() success..!! - NvOdmGpioPinMode_Output");	//20100421  [LGE]
#endif
//20101005  Gpio MicBias[END_LGE_LAB1]

	INIT_WORK(&switch_data->work, headset_det_work);
    INIT_DELAYED_WORK(&switch_data->delayed_work, type_det_work);
	lprintk(D_AUDIO, KERN_ERR "##(Headset_det.c)## headset detection - INIT_WORK() & INIT_DELAYED_WORK() success..!!\n");	//20100421  [LGE]

    if (NvOdmGpioInterruptRegister(s_hHeadsetHandle.hGpio, &s_hHeadsetHandle.hheadsetInterrupt,
        s_hHeadsetHandle.h_Headset_Detection, NvOdmGpioPinMode_InputInterruptAny/*NvOdmGpioPinMode_InputInterruptFallingEdge*//*NvOdmGpioPinMode_InputInterruptRisingEdge*/, headset_int_handler,
        switch_data, 0) == NV_FALSE)
    {
        lprintk(D_AUDIO, KERN_ERR "%s: cannot register interrupt.\n", __func__);
        goto err_get_interrupt_handler;
    }
	lprintk(D_AUDIO, KERN_ERR "##(Headset_det.c)## headset detection - NvOdmGpioInterruptRegister() success..!!\n");	//20100421  [LGE]

    //20101125, , hookkey press is skipped When wakeup from LP1 [START]
    //wake_lock_init(&hook_det_lock, WAKE_LOCK_SUSPEND, "hook_det_wake");
    //20101125, , hookkey press is skipped When wakeup from LP1 [END]

/*====================== nVidia GPIO Control(E) =======================*/

	/* Perform initial detection */
	headset_sw_data = switch_data;

	//headset_det_work(&switch_data->work);
	lprintk(D_AUDIO, KERN_ERR "##(Headset_det.c)## headset detection - headset_det_work() first detection - success..!!\n"); //20100421  [LGE]	

	err = device_create_file(&pdev->dev, &dev_attr_detect);
    err = device_create_file(&pdev->dev, &dev_attr_block_hook);
    err = device_create_file(&pdev->dev, &dev_attr_factory_test);
	return 0;

err_open_gpio_fail:	
    switch_dev_unregister(&switch_data->sdev);
err_switch_dev_register:
	kfree(switch_data);
err_open_gpio_pin_acquire_fail:
	NvOdmGpioClose(s_hHeadsetHandle.hGpio);
err_get_interrupt_handler:
    NvOdmGpioReleasePinHandle(s_hHeadsetHandle.hGpio, s_hHeadsetHandle.h_Headset_Detection);
	
    NvOdmGpioReleasePinHandle(s_hHeadsetHandle.hGpio, s_hHeadsetHandle.h_Hookkey_Detection);	//20100421  for Hookkey [LGE]
    

	return ret;
}

static void headset_shutdown(struct platform_device *pdev)
{

	lprintk(D_AUDIO, KERN_ERR "##(Headset_det.c)## headsetdet_shutdown() : headset detection ended..!!\n");	//20100421  [LGE]

	struct headset_switch_data *switch_data = platform_get_drvdata(pdev);

	//20101125, , hookkey press is skipped When wakeup from LP1 [START]
	//wake_lock_destroy(&hook_det_lock);
	//20101125, , hookkey press is skipped When wakeup from LP1 [END]

	headset_off = 1;

	if (headset_h_pmu)
		NvOdmServicesPmuClose(headset_h_pmu);
	lprintk(D_AUDIO, KERN_ERR "##(Headset_det.c)## headsetdet_shutdown() : NvOdmServicesPmuClose\n");
#if 0
	if (&headset_sw_data->work)
	{
		cancel_work_sync(&headset_sw_data->work);
		printk("switch_data->work canceled\n");
	}  

	if (&headset_sw_data->delayed_work)
	{
		cancel_delayed_work_sync(&headset_sw_data->delayed_work);
		printk("witch_data->delayed_work canceled\n");
	}

	if(&headset_sw_data->hook_delayed_work)
	{
		cancel_delayed_work_sync(&headset_sw_data->hook_delayed_work);
		printk("headset_sw_data->hook_delayed_work canceled\n");
	}
#endif
	/*====================== nVidia GPIO Control(S) =======================*/
	NvOdmGpioInterruptUnregister(s_hHeadsetHandle.hGpio, s_hHeadsetHandle.h_Headset_Detection, s_hHeadsetHandle.hheadsetInterrupt);
	NvOdmGpioSetState(s_hHeadsetHandle.hGpio, s_hHeadsetHandle.h_Headset_Detection, 0);
	NvOdmGpioConfig(s_hHeadsetHandle.hGpio, s_hHeadsetHandle.h_Headset_Detection, NvOdmGpioPinMode_Output);

	NvOdmGpioInterruptUnregister(s_hHeadsetHandle.hGpio, s_hHeadsetHandle.h_Hookkey_Detection, s_hHeadsetHandle.hhookInterrupt);	//20100421  for Hookkey [LGE]
	NvOdmGpioSetState(s_hHeadsetHandle.hGpio, s_hHeadsetHandle.h_Hookkey_Detection, 0);
	NvOdmGpioConfig(s_hHeadsetHandle.hGpio, s_hHeadsetHandle.h_Headset_Detection, NvOdmGpioPinMode_Output);


	NvOdmGpioReleasePinHandle(s_hHeadsetHandle.hGpio, s_hHeadsetHandle.h_Headset_Detection);

	NvOdmGpioReleasePinHandle(s_hHeadsetHandle.hGpio, s_hHeadsetHandle.h_Hookkey_Detection);	//20100421  for Hookkey [LGE]

	NvOdmGpioClose(s_hHeadsetHandle.hGpio);
	/*====================== nVidia GPIO Control(E) =======================*/

#if 0
	switch_dev_unregister(&headset_sw_data->sdev);

	input_unregister_device(headset_sw_data->ip_dev);	//20100421  for Hookkey [LGE]

	if(headset_sw_data) kfree(headset_sw_data);
#endif
	lprintk(D_AUDIO, KERN_ERR "##(Headset_det.c)## headsetdet_shutdown() : completed\n");
}

static int headsetdet_remove(struct platform_device *pdev)
{

	lprintk(D_AUDIO, KERN_ERR "##(Headset_det.c)## headsetdet_remove() : headset detection ended..!!\n");	//20100421  [LGE]

    struct headset_switch_data *switch_data = platform_get_drvdata(pdev);

    //20101125, , hookkey press is skipped When wakeup from LP1 [START]
    //wake_lock_destroy(&hook_det_lock);
    //20101125, , hookkey press is skipped When wakeup from LP1 [END]

    if (headset_h_pmu)
        NvOdmServicesPmuClose(headset_h_pmu);

	cancel_work_sync(&switch_data->work);
	cancel_delayed_work_sync(&switch_data->delayed_work);
	
/*====================== nVidia GPIO Control(S) =======================*/
    NvOdmGpioInterruptUnregister(s_hHeadsetHandle.hGpio, s_hHeadsetHandle.h_Headset_Detection, s_hHeadsetHandle.hheadsetInterrupt);
   
    NvOdmGpioInterruptUnregister(s_hHeadsetHandle.hGpio, s_hHeadsetHandle.h_Hookkey_Detection, s_hHeadsetHandle.hhookInterrupt);	//20100421  for Hookkey [LGE]
    
    NvOdmGpioReleasePinHandle(s_hHeadsetHandle.hGpio, s_hHeadsetHandle.h_Headset_Detection);
	
    NvOdmGpioReleasePinHandle(s_hHeadsetHandle.hGpio, s_hHeadsetHandle.h_Hookkey_Detection);	//20100421  for Hookkey [LGE]
    
    NvOdmGpioClose(s_hHeadsetHandle.hGpio);
/*====================== nVidia GPIO Control(E) =======================*/

    switch_dev_unregister(&switch_data->sdev);
    
	input_unregister_device(switch_data->ip_dev);	//20100421  for Hookkey [LGE]
	
	kfree(switch_data);

	return 0;
}

static int headset_suspend(struct platform_device *pdev, pm_message_t state)
{
//20101117, , gpio wakeup from LP1 [START]
    if(core_lock_on && headset_vdd_address){
        NvOdmServicesPmuSetVoltage(headset_h_pmu, headset_vdd_address, headset_vdd_voltage, NULL);
    }
    suspend_status = 1;
//20101117, , gpio wakeup from LP1 [END]
    if(core_lock_on == 0){
    block_hook_int =1;
	headset_type = STAR_NONE;
    lprintk(D_AUDIO, KERN_ERR "##(Headset_det.c)## headset_suspend()!! disable hook int\n");
    }
	cancel_delayed_work_sync(&headset_sw_data->delayed_work);
	cancel_delayed_work_sync(&headset_sw_data->hook_delayed_work);
    
	return 0;
}

static int headset_resume(struct platform_device *pdev)
{
//20101117, , gpio wakeup from LP1 [START]
    if(core_lock_on && headset_vdd_address){
        NvOdmServicesPmuSetVoltage(headset_h_pmu, headset_vdd_address, NVODM_VOLTAGE_OFF, NULL);
    } 
    suspend_status = 0;
//20101117, , gpio wakeup from LP1 [END]


	// 20110726 detecting headset when resuming [START]
     star_Mic_bias(1);
     headset_det_work(&headset_sw_data->work);
    // 20110726 detecting headset when resuming [END]
    
    if(core_lock_on){
        lprintk(D_AUDIO, KERN_ERR "##(Headset_det.c)## headset_resume()!! wakeup form LP1 headset detect\n");
                
        input_report_key(headset_sw_data->ip_dev_wake, KEY_VOLUMEDOWN, 1);
        input_sync(headset_sw_data->ip_dev_wake);
        input_report_key(headset_sw_data->ip_dev_wake, KEY_VOLUMEDOWN, 0);
		input_sync(headset_sw_data->ip_dev_wake);
        
        schedule_delayed_work(&headset_sw_data->delayed_work,	msecs_to_jiffies(300));	
    }
    
	return 0;
}

static struct platform_driver headsetdet_driver = {
	.probe		= headsetdet_probe,
	.remove		= __devexit_p(headsetdet_remove),
	.suspend    = headset_suspend,
	.resume     = headset_resume,
        .shutdown   = headset_shutdown,
	.driver		= {
		.name	= "star_headset",
		.owner	= THIS_MODULE,
	},
};

static int __init headsetdet_init(void)
{
	return platform_driver_register(&headsetdet_driver);
}

static void __exit headsetdet_exit(void)
{
	platform_driver_unregister(&headsetdet_driver);
}

late_initcall(headsetdet_init);
module_exit(headsetdet_exit);

MODULE_AUTHOR("LG Electronics");
MODULE_DESCRIPTION("Star Headset Detection Driver");
MODULE_LICENSE("GPL");


