/*
 * star Proximity Sensor Driver
 *
 * Copyright (C) 2010 LGE, Inc.
 *
 * Author: Taewan.kim <>
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
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/input.h>
#include <linux/sysfs.h>
#include <linux/uaccess.h>
#include <linux/hrtimer.h>
//20100715 apply tp wakelock

#include "nvcommon.h"
#include "nvodm_services.h"
#include "nvodm_query.h"
#include "nvodm_query_discovery.h"

#include <mach/lprintk.h>

#define PROXIMITY_GUID                NV_ODM_GUID('p','r','o','x','i','m','i','t')
#define PROXI_MIN_DELAY_NS            10000000
#define PROXI_DEFAULT_DELAY_NS        200000000

#define STAR_PROX_DEBUG 0 

struct star_proxi_i2c_init_data {
    NvU8 reg;
    NvU8 val;
};

static struct star_proxi_i2c_init_data initial_i2c[] = {
    {0x01, 0x08},
    {0x02, 0x40},
    {0x03, 0x04},
    {0x04, 0x03},
    {0xFF, 0x00}
};

typedef struct ProximityDeviceRec
{
    NvOdmServicesI2cHandle gen2_i2c;
    NvOdmServicesGpioHandle proxi_out_gpio;
    NvOdmGpioPinHandle proxi_out_gpio_pin;
    NvOdmServicesGpioIntrHandle proxi_out_intr;
    NvU32 i2c_address;
    NvU32 vddId;
    struct input_dev *input_dev;
    struct hrtimer timer;
    struct work_struct work;
    unsigned long delay;
    bool use_int_mode;
    bool wakeup_while_sleep;
	unsigned char MVO;
} ProximityDevice;

static atomic_t proxi_status;
static bool proxi_enabled = false;
static ProximityDevice s_proximity;
static int reset_flag = 0;

int star_proxi_get_status(void) {
    return (proxi_enabled ? 1 : 0);
}

static NvBool star_proxi_write_reg(ProximityDevice* proximity, NvU8 reg, NvU8 val)
{
    NvOdmI2cStatus Error;
    NvOdmI2cTransactionInfo TransactionInfo;
    NvU8 arr[2];

        arr[0] = reg;        // register address
        arr[1] = val;        // u16 value (lsb-msb)

        TransactionInfo.Address = proximity->i2c_address;
        TransactionInfo.Buf = arr;
        TransactionInfo.Flags = NVODM_I2C_IS_WRITE;
        TransactionInfo.NumBytes = 2;

    do
    {
        Error = NvOdmI2cTransaction(proximity->gen2_i2c,
                                    &TransactionInfo,
                                    1,
                                    400,
                                    10);
    } while (Error == NvOdmI2cStatus_Timeout);

    if (Error != NvOdmI2cStatus_Success)
    {
        lprintk(D_PROXI,"I2C Write Failure = %d (addr=0x%x, reg=0x%x, val=0x%0x)\n", Error,
                           proximity->i2c_address, reg, val);
        return NV_FALSE;
    }

    return NV_TRUE;
}

static NvBool star_proxi_read_reg(ProximityDevice* proximity, NvU8 reg, NvU8 *val)
{
    NvU8 reg_buff = 0, ReadBuffer[2];
    NvOdmI2cStatus  status = NvOdmI2cStatus_Success;
    NvOdmI2cTransactionInfo TransactionInfo[2];

        reg_buff = reg & 0xFF;

        TransactionInfo[0].Address = (proximity->i2c_address);
        TransactionInfo[0].Buf = &reg_buff;
        TransactionInfo[0].Flags = NVODM_I2C_IS_WRITE;
        TransactionInfo[0].NumBytes = 1;

        TransactionInfo[1].Address = (proximity->i2c_address|0x1);
        TransactionInfo[1].Buf = &ReadBuffer;
        TransactionInfo[1].Flags = 0;
        TransactionInfo[1].NumBytes = 2;


    do
    {
         status = NvOdmI2cTransaction(proximity->gen2_i2c, TransactionInfo, 2, 400, 10);

    } while(status == NvOdmI2cStatus_Timeout);


    if (status != NvOdmI2cStatus_Success)
    {
         lprintk(D_PROXI, "I2C Read Failure = %d (addr=0x%x, reg=0x%x, val=0x%x)\n", status,
                           proximity->i2c_address, reg, *val);
         return NV_FALSE;
    }
    *val = ReadBuffer[1];

    return NV_TRUE;
}


static void star_proxi_sleep_handler(void *arg)
{
    NvOdmGpioInterruptMask(s_proximity.proxi_out_intr, NV_FALSE);

    input_report_key(s_proximity.input_dev, KEY_POWER, 1);
    input_sync(s_proximity.input_dev);

    input_report_key(s_proximity.input_dev, KEY_POWER, 0);
    input_sync(s_proximity.input_dev);
}

static void star_proxi_i2c_init_write(void)
{
    int i;
    for (i = 0; initial_i2c[i].reg != 0xFF; i++) {
        star_proxi_write_reg(&s_proximity, initial_i2c[i].reg, initial_i2c[i].val);
    }
}

static void star_proxi_power_onoff(ProximityDevice *data, bool enable)
{
#if 1
    NvOdmServicesPmuHandle ldo_pmu = NvOdmServicesPmuOpen();
    NvOdmServicesPmuVddRailCapabilities vddrailcap;
    NvU32 settletime = 0;
    
        if (enable)
        {
            NvOdmServicesPmuGetCapabilities(ldo_pmu, data->vddId, &vddrailcap);
            NvOdmServicesPmuSetVoltage(ldo_pmu, data->vddId, vddrailcap.requestMilliVolts, &settletime);
        }
        else
        {
            NvOdmServicesPmuSetVoltage(ldo_pmu, data->vddId, NVODM_VOLTAGE_OFF, &settletime);
        }

    //if (settletime)
        NvOdmOsWaitUS(10000);   

        NvOdmServicesPmuClose(ldo_pmu);
#endif
}

static void star_proxi_vddio_vi_power_onoff( NvU32 vdd_id, NvBool is_enable )
{
	NvOdmServicesPmuHandle h_pmu = NvOdmServicesPmuOpen();
	NvOdmServicesPmuVddRailCapabilities vddrailcap;
	NvU32 settletime;
	if(h_pmu)
	{
		NvOdmServicesPmuGetCapabilities(h_pmu, vdd_id, &vddrailcap);
		if( is_enable )
		{
			#if 1
				printk("VDDID_VI's pmu enable for PROXI_OUT\n");
			#endif
			NvOdmServicesPmuSetVoltage(h_pmu, vdd_id, vddrailcap.requestMilliVolts, &settletime);
		}
		else
		{
			#if 1
				printk("VDDID_VI's pmu do not enable for PROXI_OUT\n");
			#endif
			NvOdmServicesPmuSetVoltage(h_pmu, vdd_id, NVODM_VOLTAGE_OFF, &settletime);
		}
		if(settletime)
			NvOdmOsWaitUS(settletime);
	}

	NvOdmServicesPmuClose(h_pmu);
}

#if	0
static NvU8 star_read_vo_bit(ProximityDevice *data)
{
    NvU8 vo = 0;
	NvU32 pinvalue;
    star_proxi_read_reg(&s_proximity, 0x00, &vo);

//    vo = (vo & 0x01) ? 0 : 1;
	
	if(vo & 0x01)
		vo = 1;
	else
		vo = 0;
		
	
	#if STAR_PROX_DEBUG
    lprintk(D_PROXI, "star proxi out [%d]\n", vo);
 	NvOdmGpioGetState(s_proximity.proxi_out_gpio, s_proximity.proxi_out_gpio_pin, &pinvalue);
	printk("pin value = %d\n", pinvalue);
	#endif
    return vo;
}

static void star_proxi_workqueue_func(struct work_struct *work)
{
    atomic_set(&proxi_status, star_read_vo_bit(&s_proximity));
	//20101126 Fixed the bug that proximity dose not work when target is booted firstly.[start]
    	input_report_abs(s_proximity.input_dev, ABS_DISTANCE, atomic_read(&proxi_status));
    	input_sync(s_proximity.input_dev);
	//20101126 Fixed the bug that proximity dose not work when target is booted firstly.[end]	
	
	if( atomic_read(&proxi_status) == s_proximity.MVO )
	{
		//this code must delete.
		if( atomic_read(&proxi_status) ==  0 )
			atomic_set(&proxi_status, 1 );
		else
			atomic_set(&proxi_status, 0 );
		//end
    	input_report_abs(s_proximity.input_dev, ABS_DISTANCE, atomic_read(&proxi_status));
    	input_sync(s_proximity.input_dev);
		printk("proximity value(1) = %d\n", atomic_read(&proxi_status));
		star_proxi_write_reg(&s_proximity, 0x06, 0x00 );
   // 	NvOdmGpioInterruptMask(s_proximity.proxi_out_intr, NV_FALSE);
	}
	else
	{
    	input_report_abs(s_proximity.input_dev, ABS_DISTANCE, s_proximity.MVO);
    	input_sync(s_proximity.input_dev);
		printk("proximity value(2) = %d\n", s_proximity.MVO);
		star_proxi_write_reg(&s_proximity, 0x04, 0x02 );
		s_proximity.MVO = 0;
		star_proxi_write_reg(&s_proximity, 0x04, 0x03 );
	}
}
#else
// LGE_CHANGE_S [] 2011-05-22, [P999_GB] : status update
static void star_proxi_workqueue_func(struct work_struct *work)
{
	NvU8	status;

    star_proxi_read_reg(&s_proximity, 0x00, &status);
	status	=	(status & 1);

	if (status == s_proximity.MVO) {	// Interrupt mode
		atomic_set(&proxi_status, status ? 0 : 1);

		input_report_abs(s_proximity.input_dev, ABS_DISTANCE, atomic_read(&proxi_status));
		input_sync(s_proximity.input_dev);

		star_proxi_write_reg(&s_proximity, 0x06, 0x00 );

		printk("proximity value(1) = %d\n", atomic_read(&proxi_status));
	}
}
// LGE_CHANGE_E [] 2011-05-22, [P999_GB] : status update
#endif

static void star_proxi_interrupt_handler(void *arg)
{
	//When VOUT terminal changes from H to L, enter the this function --> procedure 3
	s_proximity.MVO++; //Update the MVO value
	if( (s_proximity.MVO & 1) == 0 )
	{
		printk("Even Number INT, MVO = %d\n", s_proximity.MVO);
		s_proximity.MVO = 0;
	}
	else
	{
		printk("Odd Number INT, MVO = %d\n", s_proximity.MVO);
		s_proximity.MVO = 1;
	}

    NvOdmGpioInterruptMask(s_proximity.proxi_out_intr, NV_TRUE); //Procedure 4
	if(s_proximity.MVO == 1)
		star_proxi_write_reg(&s_proximity, 0x02, 0x00 );
	else
		star_proxi_write_reg(&s_proximity, 0x02, 0x20 );
	

	star_proxi_write_reg(&s_proximity, 0x06, 0x18 );

	#if STAR_PROX_DEBUG
	printk("PROX interrupt!!!\n");
	printk("PROX interrupt!!!\n");
	#endif
    schedule_work(&s_proximity.work);
   	NvOdmGpioInterruptMask(s_proximity.proxi_out_intr, NV_FALSE);
    NvOdmGpioInterruptDone(s_proximity.proxi_out_intr);
}
static bool s_shutdown_mode = false;
static void star_proxi_enable(ProximityDevice *data)
{
    if (proxi_enabled == true)
        return;

	
    //star_proxi_power_onoff(data, true);

    star_proxi_i2c_init_write();                 //Initail Operation, Procedure 1, see the 25page GP2AP datasheet

	s_proximity.MVO = 0;


    if (data->use_int_mode)
        NvOdmGpioInterruptMask(data->proxi_out_intr, NV_FALSE);//Interrupt Enable(NV_FALSE) Procedure 2
    else
        hrtimer_start(&data->timer, ktime_set(1, 0), HRTIMER_MODE_REL);

    proxi_enabled = true;

	atomic_set(&proxi_status, 1);

	input_report_abs(s_proximity.input_dev, ABS_DISTANCE, atomic_read(&proxi_status));
	input_sync(s_proximity.input_dev);
}

static void star_proxi_disable(ProximityDevice *data)
{
    if (proxi_enabled == false)
        return;


    if (data->use_int_mode)
        NvOdmGpioInterruptMask(data->proxi_out_intr, NV_TRUE);//Interrupt Disable(NV_TRUE)
    else
        hrtimer_cancel(&data->timer);

    cancel_work_sync(&data->work);
	//go to shutdown
    star_proxi_write_reg(&s_proximity, 0x04, 0x02 );
    s_shutdown_mode = true;
    //star_proxi_power_onoff(data, false);

//	NvOdmGpioInterruptUnregister( s_proximity.proxi_out_gpio, s_proximity.proxi_out_gpio_pin, star_proxi_interrupt_handler);

    proxi_enabled = false;
}

static enum hrtimer_restart star_proxi_timer_func(struct hrtimer *timer)
{
    schedule_work(&s_proximity.work);
    hrtimer_start(&s_proximity.timer, ktime_set(0, s_proximity.delay), HRTIMER_MODE_REL);

    return HRTIMER_NORESTART;
}


static ssize_t star_proxi_status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    sprintf(buf, "%d\n", atomic_read(&proxi_status));
    return (ssize_t)(strlen(buf) + 1);
}

static ssize_t star_proxi_onoff_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    sprintf(buf, "%d\n", proxi_enabled==true);
    return (ssize_t)(strlen(buf) + 1);
}

static ssize_t star_proxi_onoff_store(struct device *dev, struct device_attribute *attr, char *buf, size_t count)
{
    u32 val = 0;
    val = simple_strtoul(buf, NULL, 10);

    if (val) {
        lprintk(D_PROXI, "proximity on\n");
        star_proxi_enable(&s_proximity);
    } else {
        lprintk(D_PROXI, "proximity off\n");
        star_proxi_disable(&s_proximity);
    }
    return count;
}

int lge_sensor_shutdown_proxi(void)
{

    if (proxi_enabled != false)
    {
        star_proxi_disable(&s_proximity);
        reset_flag = 1;
    }
    return 0;
}

int lge_sensor_restart_proximity(void)
{
    if (reset_flag == 1)
    {
        star_proxi_enable(&s_proximity);
        reset_flag = 0;
    }
    return 0;
}

static ssize_t star_proxi_delay_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    sprintf(buf, "%lu\n", s_proximity.delay);
    return (ssize_t)(strlen(buf) + 1);
}

static ssize_t star_proxi_delay_store(struct device *dev, struct device_attribute *attr, char *buf, size_t count)
{
    u32 val = 0;

    val = simple_strtoul(buf, NULL, 10) * 1000000;

    if (val < PROXI_MIN_DELAY_NS) {
        s_proximity.delay = PROXI_MIN_DELAY_NS;
    } else {
        s_proximity.delay = val;
    }
    return count;
}

static ssize_t star_proxi_wake_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    sprintf(buf, "%ul\n", s_proximity.wakeup_while_sleep);
    return (ssize_t)(strlen(buf) + 1);
}

static ssize_t star_proxi_wake_store(struct device *dev, struct device_attribute *attr, char *buf, size_t count)
{
    u32 val = 0;

    val = simple_strtoul(buf, NULL, 10);
    s_proximity.wakeup_while_sleep = (bool)val;

    return count;
}


static DEVICE_ATTR(status, 0444, star_proxi_status_show, NULL);
static DEVICE_ATTR(onoff, 0666, star_proxi_onoff_show, star_proxi_onoff_store);
static DEVICE_ATTR(delay, 0666, star_proxi_delay_show, star_proxi_delay_store);
static DEVICE_ATTR(wake, 0666, star_proxi_wake_show, star_proxi_wake_store);

static struct attribute *star_proxi_attributes[] = {
    &dev_attr_status.attr,
    &dev_attr_onoff.attr,
    &dev_attr_delay.attr,
    &dev_attr_wake.attr,
    NULL
};

static const struct attribute_group star_proxi_group = {
    .attrs = star_proxi_attributes,
};

static int __init proximity_probe(struct platform_device *pdev)
{
    int i, ret = 0;
    NvU32 I2cInstance = 0;
    const NvOdmPeripheralConnectivity *pConnectivity = NULL;
    NvU32 port = 0, pin = 0;
    struct device *dev = &pdev->dev;
	unsigned int pinvalue;

	atomic_set(&proxi_status, 1);
    pConnectivity = NvOdmPeripheralGetGuid(PROXIMITY_GUID);

    for (i = 0; i < pConnectivity->NumAddress; i++)
    {
        switch (pConnectivity->AddressList[i].Interface)
        {
            case NvOdmIoModule_I2c:
                s_proximity.i2c_address = (pConnectivity->AddressList[i].Address << 1);
                I2cInstance = pConnectivity->AddressList[i].Instance;
                break;
            case NvOdmIoModule_Gpio:
                port = pConnectivity->AddressList[i].Instance;
                pin = pConnectivity->AddressList[i].Address;
                break;
            case NvOdmIoModule_Vdd:
                s_proximity.vddId = pConnectivity->AddressList[i].Address;
                break;
            default:
                break;
        }
    }

	s_proximity.MVO = 0;
	#if defined(CONFIG_MACH_STAR_MDM_C)
	port = 'r' - 'a';//'a' - 'a';
	pin = 2;//0;
	#elif defined (CONFIG_MACH_STAR_REV_F) || defined (CONFIG_MACH_STAR_TMUS)
	port = 'w'-'a';
	pin = 2;
	#else
	#error PROXI_OUT PIN not assigned
	#endif

    lprintk(D_PROXI, "[star Proximity] start!!!--------------------------------------------------------------------------\n");

    s_proximity.proxi_out_gpio = NvOdmGpioOpen();
    if (!s_proximity.proxi_out_gpio)
    {
        lprintk(D_PROXI, "[star Proximity] gpio open fail!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
        ret = -ENOSYS;
        goto err_open_gpio_fail;
    }

    s_proximity.proxi_out_gpio_pin = NvOdmGpioAcquirePinHandle(s_proximity.proxi_out_gpio, port, pin);
    if (!s_proximity.proxi_out_gpio_pin)
    {
        lprintk(D_PROXI, "[star Proximity] gpio pin acquire fail!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
        ret = -ENOSYS;
        goto err_open_gpio_pin_acquire_fail;
    }

//    NvOdmGpioSetState(s_proximity.proxi_out_gpio, s_proximity.proxi_out_gpio_pin, 0x1);
//    NvOdmGpioConfig(s_proximity.proxi_out_gpio, s_proximity.proxi_out_gpio_pin, NvOdmGpioPinMode_InputData);
	#if 1

    INIT_WORK(&s_proximity.work, star_proxi_workqueue_func);

    s_proximity.gen2_i2c = NvOdmI2cPinMuxOpen(NvOdmIoModule_I2c, 1, NvOdmI2cPinMap_Config2);
    if (!s_proximity.gen2_i2c)
    {
        lprintk(D_PROXI, "[star Proximity] i2c open fail!\n");
        ret = -ENOSYS;
        goto err_open_i2c_handle_fail;
    }


    s_proximity.use_int_mode = true;
	#if 0
    NvOdmGpioConfig(s_proximity.proxi_out_gpio, s_proximity.proxi_out_gpio_pin, NvOdmGpioPinMode_Output);
    NvOdmGpioSetState(s_proximity.proxi_out_gpio, s_proximity.proxi_out_gpio_pin, 0x1);
		NvOdmOsWaitUS(100000);//100ms
   	NvOdmGpioGetState(s_proximity.proxi_out_gpio, s_proximity.proxi_out_gpio_pin, &pinvalue);
	printk("interrupt pin level = %d\n----------------", pinvalue );
    NvOdmGpioSetState(s_proximity.proxi_out_gpio, s_proximity.proxi_out_gpio_pin, 0x0);
		NvOdmOsWaitUS(100000);//100ms
   	NvOdmGpioGetState(s_proximity.proxi_out_gpio, s_proximity.proxi_out_gpio_pin, &pinvalue);
	printk("interrupt pin level = %d\n----------------", pinvalue );
    NvOdmGpioSetState(s_proximity.proxi_out_gpio, s_proximity.proxi_out_gpio_pin, 0x1);
		NvOdmOsWaitUS(100000);//100ms
   	NvOdmGpioGetState(s_proximity.proxi_out_gpio, s_proximity.proxi_out_gpio_pin, &pinvalue);
	printk("interrupt pin level = %d\n----------------", pinvalue );
    NvOdmGpioSetState(s_proximity.proxi_out_gpio, s_proximity.proxi_out_gpio_pin, 0x0);
		NvOdmOsWaitUS(100000);//100ms
   	NvOdmGpioGetState(s_proximity.proxi_out_gpio, s_proximity.proxi_out_gpio_pin, &pinvalue);
	printk("interrupt pin level = %d\n----------------", pinvalue );
    NvOdmGpioSetState(s_proximity.proxi_out_gpio, s_proximity.proxi_out_gpio_pin, 0x1);
		NvOdmOsWaitUS(100000);//100ms
   	NvOdmGpioGetState(s_proximity.proxi_out_gpio, s_proximity.proxi_out_gpio_pin, &pinvalue);
	printk("interrupt pin level = %d\n----------------", pinvalue );
	#endif	

	#if 0 
	while(1)
	{
    	NvOdmGpioSetState(s_proximity.proxi_out_gpio, s_proximity.proxi_out_gpio_pin, 0x1);
		NvOdmOsWaitUS(100000);//100ms
    	NvOdmGpioSetState(s_proximity.proxi_out_gpio, s_proximity.proxi_out_gpio_pin, 0x0);
		NvOdmOsWaitUS(100000);//100ms
	}
	#endif

    NvOdmGpioConfig(s_proximity.proxi_out_gpio, s_proximity.proxi_out_gpio_pin, NvOdmGpioPinMode_InputData);

    if (s_proximity.use_int_mode == true) {
        if (NvOdmGpioInterruptRegister(s_proximity.proxi_out_gpio, &s_proximity.proxi_out_intr,
            s_proximity.proxi_out_gpio_pin, NvOdmGpioPinMode_InputInterruptLow, star_proxi_interrupt_handler, (void*)&s_proximity, 0) == NV_FALSE)
        {
            lprintk(D_PROXI, "[star Proximity] interrupt register fail!\n");
            ret = -ENOSYS;
            goto err_open_irq_handle_fail;
        }
    } else {
        if (NvOdmGpioInterruptRegister(s_proximity.proxi_out_gpio, &s_proximity.proxi_out_intr,
            s_proximity.proxi_out_gpio_pin, NvOdmGpioPinMode_InputInterruptFallingEdge, star_proxi_sleep_handler, (void*)&s_proximity, 0) == NV_FALSE)
        {
            lprintk(D_PROXI, "[star Proximity] interrupt register fail!\n");
            ret = -ENOSYS;
            goto err_open_irq_handle_fail;
        }

        hrtimer_init(&s_proximity.timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
        s_proximity.timer.function = star_proxi_timer_func;
        s_proximity.delay = PROXI_DEFAULT_DELAY_NS;
    }


    NvOdmGpioInterruptMask(s_proximity.proxi_out_intr, NV_TRUE);

    s_proximity.input_dev = input_allocate_device();
    if (!s_proximity.input_dev) {
        lprintk(D_PROXI, "[star Proximity] input device alloc fail!\n");
        ret = -ENOMEM;
        goto err_alloc_input_device_fail;
    }

    set_bit(EV_KEY, s_proximity.input_dev->evbit);
    set_bit(KEY_POWER, s_proximity.input_dev->keybit);
    set_bit(EV_ABS, s_proximity.input_dev->evbit);
    input_set_abs_params(s_proximity.input_dev, ABS_DISTANCE, 0, 1, 0, 0);
    s_proximity.input_dev->name = "proximity";
    ret = input_register_device(s_proximity.input_dev);
    if (ret) {
        lprintk(D_PROXI, "[star Proximity] input device register fail!\n");
        ret = -ENOMEM;
        goto err_alloc_input_device_fail;
    }

    if ((ret = sysfs_create_group(&dev->kobj, &star_proxi_group))) {
        lprintk(D_PROXI, "[star Proximity] sysfs_create_group fail!\n");
        ret = -ENOMEM;
        goto err_sysfs_group_fail;
    }

//    star_proxi_power_onoff(&s_proximity, true);

    return 0;

err_sysfs_group_fail:
    input_unregister_device(s_proximity.input_dev);
err_alloc_input_device_fail:
    NvOdmGpioInterruptUnregister(s_proximity.proxi_out_gpio, s_proximity.proxi_out_gpio_pin,
        s_proximity.proxi_out_intr);
err_open_irq_handle_fail:
    NvOdmI2cClose(s_proximity.gen2_i2c); 
err_open_i2c_handle_fail:
    NvOdmGpioReleasePinHandle(s_proximity.proxi_out_gpio, s_proximity.proxi_out_gpio_pin);
err_open_gpio_pin_acquire_fail:
    NvOdmGpioClose(s_proximity.proxi_out_gpio);
err_open_gpio_fail:

    return ret;

#endif
}

static int proximity_remove(struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    star_proxi_disable(&s_proximity);
    sysfs_remove_group(&dev->kobj, &star_proxi_group);
    input_unregister_device(s_proximity.input_dev);

    NvOdmGpioInterruptUnregister(s_proximity.proxi_out_gpio, s_proximity.proxi_out_gpio_pin,
        s_proximity.proxi_out_intr);
    NvOdmGpioReleasePinHandle(s_proximity.proxi_out_gpio, s_proximity.proxi_out_gpio_pin);
    NvOdmGpioClose(s_proximity.proxi_out_gpio);
    
    NvOdmI2cClose(s_proximity.gen2_i2c);

    return 0;
}

#if 0
static int proximity_suspend(struct platform_device *pdev, pm_message_t state)
{
    if (!proxi_enabled)
        return 0;

    if (atomic_read(&proxi_status))
        return 0;

    if (hrtimer_try_to_cancel(&s_proximity.timer) > 0)
        hrtimer_cancel(&s_proximity.timer);

    if (s_proximity.wakeup_while_sleep)
        NvOdmGpioInterruptMask(s_proximity.proxi_out_intr, NV_TRUE);
    else
        star_proxi_disable(&s_proximity);

    return 0;
}

static int proximity_resume(struct platform_device *pdev)
{
    if (!proxi_enabled)
        return 0;

    if (s_proximity.wakeup_while_sleep)
        NvOdmGpioInterruptMask(s_proximity.proxi_out_intr, NV_FALSE);

    if (hrtimer_try_to_cancel(&s_proximity.timer) == 0)
        hrtimer_start(&s_proximity.timer, ktime_set(0, s_proximity.delay), HRTIMER_MODE_REL);

    if (!s_proximity.wakeup_while_sleep)
        NvOdmGpioInterruptMask(s_proximity.proxi_out_intr, NV_TRUE);

    return 0;
}
#endif

static struct platform_driver star_proximity_driver = {
        .probe   = proximity_probe,
        .remove  = proximity_remove,
        //.suspend = proximity_suspend,
        //.resume = proximity_resume,
        .driver  = {
                .name = "star_proximity",
                .owner = THIS_MODULE,
        },
};

static int __init proximity_init(void)
{
    return platform_driver_register(&star_proximity_driver);
}

static void __exit proximity_exit(void)
{
    platform_driver_unregister(&star_proximity_driver);
}

module_init(proximity_init);
module_exit(proximity_exit);

MODULE_AUTHOR("");
MODULE_DESCRIPTION("star proximity driver");
MODULE_LICENSE("GPL");
