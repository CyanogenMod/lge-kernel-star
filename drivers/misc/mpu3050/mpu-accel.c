/*
    mpu-accel.c - mpu3050 input device interface

    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/stat.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/signal.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/pm.h>
#include <linux/mutex.h>
#include <linux/suspend.h>
#include <linux/delay.h>



#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/input.h>

#include "mpuirq.h"
#include "slaveirq.h"
#include "mlsl.h"
#include "mpu-i2c.h"
#include "mldl_cfg.h"
#include "mpu.h"
#include "mpu-accel.h"

#define MPUACC_DEBUG 0
#define MPUACC_DEBUG_CFG 0



#define MPUACCEL_INPUT_NAME "mpu-accel"


struct mpuaccel_data {
    struct input_dev *input_data;
    struct delayed_work work;
    struct mutex data_mutex;

    struct mldl_cfg* mldl_cfg;
    void* accel_handle;

    atomic_t enable;
    atomic_t delay;
    int device_is_on;
#ifdef MPUACC_USES_CACHED_DATA
    unsigned char cached_data[6];
#endif //MPUACC_USES_CACHED_DATA
};

static struct mpuaccel_data* pThisData = NULL;


static void
mpu_accel_print_mldl_cfg(struct mldl_cfg *mldl_cfg)
{
    if (MPUACC_DEBUG_CFG) {
        printk("requested_sensors:%ld\n", mldl_cfg->requested_sensors);
//    printk("ignore_system_suspend:%d\n", mldl_cfg->ignore_system_suspend);
        printk("addr:%d\n", mldl_cfg->addr);
        printk("int_config:%d\n", mldl_cfg->int_config);
        printk("ext_sync:%d\n", mldl_cfg->ext_sync);
        printk("full_scale:%d\n", mldl_cfg->full_scale);
        printk("dmp_enable:%d\n", mldl_cfg->dmp_enable);
        printk("fifo_enable:%d\n", mldl_cfg->fifo_enable);
        printk("dmp_cfg1:%d\n", mldl_cfg->dmp_cfg1);
        printk("dmp_cfg2:%d\n", mldl_cfg->dmp_cfg2);
        printk("gyro_power:%d\n", mldl_cfg->gyro_power);
        printk("gyro_is_bypassed:%d\n", mldl_cfg->gyro_is_bypassed);
        printk("dmp_is_running:%d\n", mldl_cfg->dmp_is_running);
        printk("gyro_is_suspended:%d\n", mldl_cfg->gyro_is_suspended);
        printk("accel_is_suspended:%d\n", mldl_cfg->accel_is_suspended);
        printk("compass_is_suspended:%d\n", mldl_cfg->compass_is_suspended);
        printk("pressure_is_suspended:%d\n", mldl_cfg->pressure_is_suspended);
        printk("gyro_needs_reset:%d\n", mldl_cfg->gyro_needs_reset);
    }
}


static int
mpu_accel_mutex_lock(struct mpuaccel_data *data)
{
    mutex_lock(&data->data_mutex);

    return ML_SUCCESS;
}

static int
mpu_accel_mutex_unlock(struct mpuaccel_data *data)
{
    mutex_unlock(&data->data_mutex);

    return ML_SUCCESS;
}


static int
mpu_accel_activate_device(struct mpuaccel_data *data, int enable)
{
    int result = ML_SUCCESS;
    struct mldl_cfg* mldl_cfg = data->mldl_cfg;

    if (enable) {
        /*turn on accel*/
        if (NULL != mldl_cfg->accel && NULL != mldl_cfg->accel->resume) {
            result = mldl_cfg->accel->resume(data->accel_handle,
                                             mldl_cfg->accel,
                                             &mldl_cfg->pdata->accel);
        }
    } else {
        /*turn off accel*/
        if (NULL != mldl_cfg->accel && NULL != mldl_cfg->accel->suspend) {
            result = mldl_cfg->accel->suspend(data->accel_handle,
                                              mldl_cfg->accel,
                                              &mldl_cfg->pdata->accel);
        }
    }

    if (result == ML_SUCCESS) {
        data->device_is_on = enable;
    }

    if (MPUACC_DEBUG) {
        printk("activate device:%d, result=%d\n", enable, result);
    }

    return result;
}


static int
mpu_accel_get_data_from_device(struct mpuaccel_data *data, unsigned char* buffer)
{
    int result = ML_SUCCESS;
    struct mldl_cfg* mldl_cfg = data->mldl_cfg;

    if (NULL != mldl_cfg->accel && NULL != mldl_cfg->accel->read) {
        result = mldl_cfg->accel->read(data->accel_handle,
                                       mldl_cfg->accel,
                                       &mldl_cfg->pdata->accel,
                                       buffer);
    }

    return result;
}


static int
mpu_accel_get_data_from_mpu(struct mpuaccel_data *data, unsigned char* buffer)
{
    int result = ML_SUCCESS;
    struct mldl_cfg* mldl_cfg = data->mldl_cfg;
    result = MLSLSerialRead(data->accel_handle,mldl_cfg->addr,0x23, 6, buffer);
    return result;
}


static int
mpu_accel_get_data(struct mpuaccel_data *data, unsigned char* buffer, int* from_mpu)
{
    int res = ML_SUCCESS;
    struct mldl_cfg* mldl_cfg = data->mldl_cfg;


    if (mldl_cfg->accel_is_suspended == 1 ||
            (mldl_cfg->dmp_is_running==0 && mldl_cfg->accel_is_suspended == 0)) {

        if (from_mpu!=NULL) *from_mpu = 0;

        /*
          Retrieve accel data from accel device driver directly.
        */
        res = mpu_accel_get_data_from_device(data, buffer);
    }
    else if (mldl_cfg->dmp_is_running &&
             mldl_cfg->accel_is_suspended==0) {

        if (from_mpu!=NULL) *from_mpu = 1;

        /*
          Retrieve accel data from MPU registers(0x23 to 0x28).
        */
        res = mpu_accel_get_data_from_mpu(data, buffer);
    }

    return res;
}


static int
mpu_accel_build_data(struct mpuaccel_data *data, const unsigned char* buffer,int* val)
{
    struct mldl_cfg* mldl_cfg = data->mldl_cfg;
    int endian = mldl_cfg->accel->endian;
    int dev_id = mldl_cfg->accel->id;

    if (endian == EXT_SLAVE_LITTLE_ENDIAN) {
        if (dev_id == ACCEL_ID_BMA150)
            *val = (*(s16 *)&buffer[0]) >> 6;
        else if (dev_id == ACCEL_ID_KXTF9)
            *val = (*(s16 *)&buffer[0]) >> 4;
        else
            *val = (buffer[1]<<8)|buffer[0];
    }
    else if (endian == EXT_SLAVE_BIG_ENDIAN) {
        *val = (buffer[0]<<8)|buffer[1];
    }

    return ML_SUCCESS;
}



static void
mpu_accel_input_work_func(struct work_struct *work)
{
    int res = 0;
    int poll_time = 0;
    int enable = 0;
    int i=0;

    struct mpuaccel_data *data = container_of((struct delayed_work *)work,
                                 struct mpuaccel_data, work);

    struct mldl_cfg* mldl_cfg = data->mldl_cfg;

    poll_time = atomic_read(&data->delay);

    if (MPUACC_DEBUG) {
        printk("________________START____________________\n");
    }
    if (MPUACC_DEBUG_CFG) {
        mpu_accel_print_mldl_cfg(mldl_cfg);
    }

    mpu_accel_mutex_lock(data);
    enable = atomic_read(&data->enable);
    mpu_accel_mutex_unlock(data);

    if (enable) {
        unsigned char buffer[6]={0,};
        int raw[3]={0,};
        int data_is_avail = 0;
        int data_is_from_mpu = 0;

        mpu_accel_mutex_lock(data);
        mpu_accel_get_data(data, buffer, &data_is_from_mpu);
        mpu_accel_mutex_unlock(data);


        if (res==ML_SUCCESS) {
            data_is_avail = 1;
        }

        if (data_is_avail) {
            int data_is_valid = 0;

            for (i=0; i<3; i++)  {
                mpu_accel_build_data(data, &buffer[i*2], &raw[i]);
            }

            if (raw[0] != 0 || raw[1] != 0 || raw[2] != 0) {
                data_is_valid = 1;
            }

            if (data_is_valid) {
                int accel[3] = {0,};

                /*apply mounting matrix*/
                for (i=0; i<3; i++) {
#ifdef MPUACC_USES_MOUNTING_MATRIX
                    int j=0;
                    for (j=0; j<3; j++) {
                        accel[i] += mldl_cfg->pdata->accel.orientation[i*3+j]*raw[j];
                    }
#else
                    accel[i] = raw[i];
#endif
                }

                if (MPUACC_DEBUG) {
                    if (data_is_from_mpu==1)
                        printk("MPU_ACCEL:[%d][%d][%d]\n", accel[0],accel[1],accel[2]);
                    else
                        printk("ACCEL:[%d][%d][%d]\n", accel[0],accel[1],accel[2]);
                }
#ifdef MPUACC_USES_CACHED_DATA
                memcpy(data->cached_data, buffer, sizeof(unsigned char)*6);
#endif //#ifdef MPUACC_USES_CACHED_DATA          
                input_report_rel(data->input_data, REL_X, accel[0]);
                input_report_rel(data->input_data, REL_Y, accel[1]);
                input_report_rel(data->input_data, REL_Z, accel[2]);
                input_sync(data->input_data);

                if (MPUACC_DEBUG) {
                    printk("input device is updated\n");
                }
            }
        }


    }


    if (MPUACC_DEBUG) {
        printk("________________END____________________\n");
    }

    mpu_accel_mutex_lock(data);
    enable = atomic_read(&data->enable);
    mpu_accel_mutex_unlock(data);

    if (enable) {
        if (poll_time > 0) {
            schedule_delayed_work(&data->work,
                                  msecs_to_jiffies(poll_time) + 1);
        }
        else {
            schedule_delayed_work(&data->work, 0);
        }
    }

}

static int
mpu_accel_enable(struct mpuaccel_data *data)
{
    int res = ML_SUCCESS;
    struct mldl_cfg* mldl_cfg = data->mldl_cfg;

    if (MPUACC_DEBUG) {
        printk("mpu_accel_enable : %d\n",atomic_read(&data->enable));
    }


    if (atomic_read(&data->enable) != 1) {

        if (MPUACC_DEBUG) {
            printk("mpu_accel_enable : enabled\n");
        }

        if (mldl_cfg->accel_is_suspended == 1) {
            if (MPUACC_DEBUG) {
                printk("mpu_accel_enable : turn on accel\n");
            }
            mpu_accel_activate_device(data,1);
        }

        atomic_set(&data->enable, 1);
        schedule_delayed_work(&data->work, 0);

    }


    return res;
}


static int
mpu_accel_disable(struct mpuaccel_data *data)
{
    int res = ML_SUCCESS;
    struct mldl_cfg* mldl_cfg = data->mldl_cfg;


    if (MPUACC_DEBUG) {
        printk("mpu_accel_disable : %d\n",atomic_read(&data->enable));
    }

    if (atomic_read(&data->enable) != 0) {
        atomic_set(&data->enable, 0);
        cancel_delayed_work(&data->work);

        if (MPUACC_DEBUG) {
            printk("mpu_accel_disable : disabled\n");
        }

        if (mldl_cfg->accel_is_suspended == 1) {
            if (MPUACC_DEBUG) {
                printk("mpu_accel_disable : turn off accel\n");
            }

            /*turn off accel*/
            mpu_accel_activate_device(data,0);
        }
    }

    return res;
}



static ssize_t
mpu_accel_delay_show(struct device *dev,
                     struct device_attribute *attr,
                     char *buf)
{
    struct input_dev *input_data = to_input_dev(dev);
    struct mpuaccel_data *data = input_get_drvdata(input_data);

    return sprintf(buf, "%d\n", atomic_read(&data->delay));
}

static ssize_t
mpu_accel_delay_store(struct device *dev,
                      struct device_attribute *attr,
                      const char *buf,
                      size_t count)
{
    struct input_dev *input_data = to_input_dev(dev);
    struct mpuaccel_data *data = input_get_drvdata(input_data);
    int value = simple_strtoul(buf, NULL, 10);

    atomic_set(&data->delay, value);
    return count;
}

static ssize_t
mpu_accel_enable_show(struct device *dev,
                      struct device_attribute *attr,
                      char *buf)
{
    struct input_dev *input_data = to_input_dev(dev);
    struct mpuaccel_data *data = input_get_drvdata(input_data);

    return sprintf(buf, "%d\n", atomic_read(&data->enable));
}

static ssize_t
mpu_accel_enable_store(struct device *dev,
                       struct device_attribute *attr,
                       const char *buf,
                       size_t count)
{
    struct input_dev *input_data = to_input_dev(dev);
    struct mpuaccel_data *data = input_get_drvdata(input_data);
    int value;

    value = simple_strtoul(buf, NULL, 10);
    if (value != 0 && value != 1) {
        return count;
    }

    mpu_accel_mutex_lock(data);

    if (value) {
        mpu_accel_enable(data);
    }
    else {
        mpu_accel_disable(data);
    }

    mpu_accel_mutex_unlock(data);

    return count;
}


int
mpu_accel_is_active_device(void)
{
    int is_active = 0;

    if (pThisData!=NULL) {
        mpu_accel_mutex_lock(pThisData);
        is_active = pThisData->device_is_on;
        mpu_accel_mutex_unlock(pThisData);
    }

    return is_active;
}

#ifdef MPUACC_USES_CACHED_DATA
int mpu_accel_get_cached_data(unsigned char* cache)
{
    int res = ML_ERROR;

    if (pThisData!=NULL) {
        if (pThisData->device_is_on==1) {
            memcpy(cache, pThisData->cached_data, sizeof(unsigned char)*6);
            if (1) {
                printk("cached data:[%d][%d][%d][%d][%d][%d]\n",
                       cache[0],cache[1],cache[2],cache[3],cache[4],cache[5]);
            }
            res = ML_SUCCESS;
        }

    }

    return res;
}
#endif //MPUACC_USES_CACHED_DATA

// 20110722 deukgi.shin@lge.com // add accel cal [S]
#define GET12BIT(a,b)   (short)(((((unsigned short)(*(b)))<<4) | (unsigned short)((*(a))>>4)) & 0x0FFF)
#define TWOS_CMPLT(a)	(-1 * ((~(a) & 0x0FFF)+1))
#define CHANGE_TO_ABS(a)	((0 < (a)) ? (a) : -(a))

#define SAMPLING_COUNT 100
typedef struct accel_cal_value_type
{
    int value_x;
    int value_y;
    int value_z;
} accel_cal_value_type;

static bool accel_cal_start = false;
#if defined (CONFIG_MACH_BSSQ)
//20111204 bg80.song@lge.com Enable CAL Data Appling Flag [S]
bool accel_cal_complete = true;
//20111204 bg80.song@lge.com Enable CAL Data Appling Flag [E]
#else
bool accel_cal_complete = false;
#endif
static bool accel_cal_result = false;
accel_cal_value_type accel_cal_value;

EXPORT_SYMBOL(accel_cal_value);
EXPORT_SYMBOL(accel_cal_complete);

int mpu_accel_read_accel_raw_data(void *sl_handle, unsigned char slaveAddr, unsigned char registerAddr, unsigned short length, unsigned char *data){

    unsigned short bytesRead = 0;
    int result = 0;
    while (bytesRead < length) {
        unsigned short thisLen = min(SERIAL_MAX_TRANSFER_SIZE, length - bytesRead);
        result =  sensor_i2c_read((struct i2c_adapter *) sl_handle,
				slaveAddr, registerAddr + bytesRead,
				thisLen, &data[bytesRead]);
	if (ML_SUCCESS != result)
		return result;
	bytesRead += thisLen;
    }
    return result;	
}
bool mpu_accel_check_difference_previous_value(accel_cal_value_type current_value, int is_first)
{
    static accel_cal_value_type previous_value;	
    unsigned int accel_data_difference = 0;
    int *ptr_prev = &previous_value;
    int *ptr_curr = &current_value;
    int i;	

    if(!(is_first))
    {
        memcpy(&previous_value, &current_value, sizeof(accel_cal_value_type));
        return true;
    }

    for(i=0; i<3; i++)
    {
        accel_data_difference = CHANGE_TO_ABS((*(ptr_prev+i) - *(ptr_curr+i)));
        printk("deukgi.shin previous_value : %d, current_value : %d\n", *(ptr_prev+i), *(ptr_curr+i));		
        if(accel_data_difference > 10)
        {
            printk("deukgi.shin accel_data_difference : %d\n", accel_data_difference);
            return false;
        }
    }

    memcpy(&previous_value, &current_value, sizeof(accel_cal_value_type));
    return true;	
}
bool mpu_accel_sum_value(struct mpuaccel_data* data, int count)
{
    int i;
    unsigned char buf[6] = {0, 0, 0, 0, 0, 0};
    accel_cal_value_type temp_value = {0, 0, 0};
    accel_cal_value.value_x = 0;
    accel_cal_value.value_y = 0;
    accel_cal_value.value_z = 0;
    int result = 0;
    struct mldl_cfg* mldl_cfg = data->mldl_cfg;

    printk("duekgi.shin ENTER : %s\n", __FUNCTION__);
    printk("duekgi.shin m_data->accel_handle : %d\n", data->accel_handle);	
    for(i=0; i<count; i++)
    {
        result = mpu_accel_read_accel_raw_data(data->accel_handle, 0x0F, 0x06, 6, buf);
        printk("duekgi.shin succes read from accel device %d\n", i);
        msleep(5);
        ERROR_CHECK(result);

        temp_value.value_x = GET12BIT(buf+0, buf+1);
        temp_value.value_y = GET12BIT(buf+2, buf+3);
        temp_value.value_z = GET12BIT(buf+4, buf+5);

        if (temp_value.value_x & 0x800)
            temp_value.value_x = TWOS_CMPLT(temp_value.value_x);
	
        if (temp_value.value_y & 0x800)
            temp_value.value_y = TWOS_CMPLT(temp_value.value_y);

        if (temp_value.value_z & 0x800)
            temp_value.value_z = TWOS_CMPLT(temp_value.value_z);

        accel_cal_value.value_x += temp_value.value_x;
        accel_cal_value.value_y += temp_value.value_y;
        accel_cal_value.value_z += temp_value.value_z;
/*
        printk("deukgi.shin each value x : %d\n", temp_value.value_x);
        printk("deukgi.shin each value y : %d\n", temp_value.value_y);
        printk("deukgi.shin each value z : %d\n", temp_value.value_z);		
*/

        if(!mpu_accel_check_difference_previous_value(temp_value, i))
        	{
        	    return false;
        	}
        	
    }
    printk("deukgi.shin SUM value x : %d\n", accel_cal_value.value_x);
    printk("deukgi.shin SUM value y : %d\n", accel_cal_value.value_y);
    printk("deukgi.shin SUM value z : %d\n", accel_cal_value.value_z);

    return true;	
}

void mpu_accel_get_cal_value(struct mpuaccel_data *data, int count)
{
    printk("duekgi.shin ENTER : %s\n", __FUNCTION__);

    accel_cal_value.value_x = accel_cal_value.value_x / count;
    accel_cal_value.value_y = accel_cal_value.value_y / count;
    accel_cal_value.value_z = accel_cal_value.value_z / count;	

    printk("deukgi.shin AVERAGE value x : %d\n", accel_cal_value.value_x);
    printk("deukgi.shin AVERAGE value y : %d\n", accel_cal_value.value_y);
    printk("deukgi.shin AVERAGE value z : %d\n", accel_cal_value.value_z);

    accel_cal_value.value_x	= (0 - accel_cal_value.value_x);
    accel_cal_value.value_y	= (0 - accel_cal_value.value_y);
    accel_cal_value.value_z	= -(1024 + accel_cal_value.value_z);

    printk("deukgi.shin OFFSET value x : %d\n", accel_cal_value.value_x);
    printk("deukgi.shin OFFSET value y : %d\n", accel_cal_value.value_y);
    printk("deukgi.shin OFFSET value z : %d\n", accel_cal_value.value_z);

}

bool mpu_accel_calibrate(struct mpuaccel_data *data)
{
    bool result = 0;
    printk("duekgi.shin ENTER : %s\n", __FUNCTION__);

    accel_cal_complete = false;
//    mpu_accel_activate_device(data, 0);
    // get & sum value
    result = mpu_accel_sum_value(data, SAMPLING_COUNT);
    if(result == false)
    {
        return false;
    }    	
    // get & avrage value
    mpu_accel_get_cal_value(data, SAMPLING_COUNT);
    // apply cal value  
//    mpu_accel_activate_device(data, 1);    

     return true;		
}

static ssize_t mpu_accel_cal_start_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    printk("duekgi.shin ENTER : %s\n", __FUNCTION__);

    return sprintf(buf, "%d\n", accel_cal_start);
}

static ssize_t mpu_accel_cal_start_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct input_dev *input_data = to_input_dev(dev);
    struct mpuaccel_data *data = input_get_drvdata(input_data);
    bool result = false;

    unsigned long input_value = simple_strtoul(buf, NULL, 10);

    printk("duekgi.shin ENTER : %s\n", __FUNCTION__);
	
    if( (input_value == 1) && (accel_cal_start == false) ){
        accel_cal_start = true;
        accel_cal_result = false;         		
        result = mpu_accel_calibrate(data);
        accel_cal_result = result;
        accel_cal_complete = result;

        accel_cal_start = false;	
    }
    else{
        printk("deukgi.shin Could not start accel cal accel_cal_start : %d\n", accel_cal_start);
    }
    return count;
}

static ssize_t mpu_accel_cal_value_x_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", accel_cal_value.value_x);
}

#if defined (CONFIG_MACH_BSSQ)
static ssize_t mpu_accel_cal_value_x_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct input_dev *input_data = to_input_dev(dev);
    struct mpuaccel_data *data = input_get_drvdata(input_data);
    bool result = false;
//20111204 bg80.song@lge.com Enable CAL Data Appling Flag [S]
    long input_value = simple_strtol(buf, NULL, 10);

    printk("duekgi.shin ENTER : %s\n", __FUNCTION__);
    printk("duekgi.shin mpu_accel_cal_value_x_store ENTER : %s, input_value = %ld \n", __FUNCTION__, input_value);
	
    accel_cal_value.value_x = (int) input_value;

	accel_cal_complete = true;
//20111204 bg80.song@lge.com Enable CAL Data Appling Flag [E]
    return count;
}
#endif

static ssize_t mpu_accel_cal_value_y_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", accel_cal_value.value_y);
}

#if defined (CONFIG_MACH_BSSQ)
static ssize_t mpu_accel_cal_value_y_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct input_dev *input_data = to_input_dev(dev);
    struct mpuaccel_data *data = input_get_drvdata(input_data);
    bool result = false;
//20111204 bg80.song@lge.com Enable CAL Data Appling Flag [S]
    long input_value = simple_strtol(buf, NULL, 10);

    printk("duekgi.shin ENTER : %s\n", __FUNCTION__);
    printk("duekgi.shin mpu_accel_cal_value_y_store ENTER : %s, input_value = %ld \n", __FUNCTION__, input_value);
	
    accel_cal_value.value_y = (int) input_value;

	accel_cal_complete = true;
//20111204 bg80.song@lge.com Enable CAL Data Appling Flag [E]
    return count;
}
#endif

static ssize_t mpu_accel_cal_value_z_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", accel_cal_value.value_z);
}

#if defined (CONFIG_MACH_BSSQ)
static ssize_t mpu_accel_cal_value_z_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct input_dev *input_data = to_input_dev(dev);
    struct mpuaccel_data *data = input_get_drvdata(input_data);
    bool result = false;
//20111204 bg80.song@lge.com Enable CAL Data Appling Flag [S]
    long input_value = simple_strtol(buf, NULL, 10);

    printk("duekgi.shin ENTER : %s\n", __FUNCTION__);
    printk("duekgi.shin mpu_accel_cal_value_z_store ENTER : %s, input_value = %ld \n", __FUNCTION__, input_value);
	
    accel_cal_value.value_z = (int) input_value;

	accel_cal_complete = true;
//20111204 bg80.song@lge.com Enable CAL Data Appling Flag [E]
    return count;
}
#endif

static ssize_t mpu_accel_cal_result_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", accel_cal_result);
}


// 20110722 deukgi.shin@lge.com // add accel cal [E]


static DEVICE_ATTR(delay, S_IRUGO|S_IWUSR|S_IWGRP,
                   mpu_accel_delay_show, mpu_accel_delay_store);
static DEVICE_ATTR(enable, S_IRUGO|S_IWUSR|S_IWGRP,
                   mpu_accel_enable_show, mpu_accel_enable_store);
// 20110722 deukgi.shin@lge.com // add accel cal [S]
static DEVICE_ATTR(cal_start, S_IRUGO|S_IWUSR|S_IWGRP,
                   mpu_accel_cal_start_show, mpu_accel_cal_start_store);
#if defined (CONFIG_MACH_BSSQ)
static DEVICE_ATTR(cal_value_x, S_IRUGO|S_IWUSR|S_IWGRP,
                   mpu_accel_cal_value_x_show, mpu_accel_cal_value_x_store);
static DEVICE_ATTR(cal_value_y, S_IRUGO|S_IWUSR|S_IWGRP,
                   mpu_accel_cal_value_y_show, mpu_accel_cal_value_y_store);
static DEVICE_ATTR(cal_value_z, S_IRUGO|S_IWUSR|S_IWGRP,
                   mpu_accel_cal_value_z_show, mpu_accel_cal_value_z_store);
static DEVICE_ATTR(cal_result, S_IRUGO|S_IWUSR|S_IWGRP,
                   mpu_accel_cal_result_show, NULL);
#else
static DEVICE_ATTR(cal_value_x, S_IRUGO|S_IWUSR|S_IWGRP,
                   mpu_accel_cal_value_x_show, NULL);
static DEVICE_ATTR(cal_value_y, S_IRUGO|S_IWUSR|S_IWGRP,
                   mpu_accel_cal_value_y_show, NULL);
static DEVICE_ATTR(cal_value_z, S_IRUGO|S_IWUSR|S_IWGRP,
                   mpu_accel_cal_value_z_show, NULL);
static DEVICE_ATTR(cal_result, S_IRUGO|S_IWUSR|S_IWGRP,
                   mpu_accel_cal_result_show, NULL);
#endif
// 20110722 deukgi.shin@lge.com // add accel cal [E]




static struct attribute *mpuaccel_attributes[] = {
    &dev_attr_delay.attr,
    &dev_attr_enable.attr,
// 20110722 deukgi.shin@lge.com // add accel cal [S]
    &dev_attr_cal_start.attr,
    &dev_attr_cal_value_x.attr,
    &dev_attr_cal_value_y.attr,
    &dev_attr_cal_value_z.attr,
    &dev_attr_cal_result.attr,
// 20110722 deukgi.shin@lge.com // add accel cal [E]
    NULL
};

static struct attribute_group mpuaccel_attribute_group = {
    .attrs = mpuaccel_attributes
};



int
mpu_accel_init(struct mldl_cfg* mldl_cfg, void* accel_handle)
{
    struct input_dev *input_data = NULL;
    struct mpuaccel_data *data = NULL;
    int res = 0;


    data = kzalloc(sizeof(struct mpuaccel_data), GFP_KERNEL);
    if (data == NULL) {
        res = -ENOMEM;
        goto err;
    }

    data->mldl_cfg = mldl_cfg;
    data->accel_handle = accel_handle;
    atomic_set(&data->enable, 0);
    atomic_set(&data->delay, 20); //set 20ms to polling time

    mutex_init(&data->data_mutex);

    INIT_DELAYED_WORK(&data->work, mpu_accel_input_work_func);

    input_data = input_allocate_device();
    if (input_data == NULL) {
        res = -ENOMEM;
        printk(KERN_ERR
               "mpu_accel_probe: Failed to allocate input_data device\n");
        goto err;
    }


    input_data->name = MPUACCEL_INPUT_NAME;
    input_data->id.bustype = BUS_I2C;

    set_bit(EV_REL, input_data->evbit);
    input_set_capability(input_data, EV_REL, REL_X);
    input_set_capability(input_data, EV_REL, REL_Y);
    input_set_capability(input_data, EV_REL, REL_Z);


    data->input_data = input_data;

    res = input_register_device(input_data);
    if (res) {
        printk(KERN_ERR
               "mpu_accel_init: Unable to register input_data device: %s\n",
               input_data->name);
        goto err;
    }

    input_set_drvdata(input_data, data);
    mldl_cfg->ext.mpuacc_data = (void*)data;

    pThisData = data;

    res = sysfs_create_group(&input_data->dev.kobj,
                             &mpuaccel_attribute_group);
    if (res) {
        printk(KERN_ERR
               "mpu_accel_init: sysfs_create_group failed[%s]\n",
               input_data->name);
        goto err;
    }
#if defined (CONFIG_MACH_BSSQ)
//20111204 bg80.song@lge.com CAL Value Initialize for CAL Data Backup/Restore [S]
	accel_cal_value.value_x = 0;
	accel_cal_value.value_y = 0;
	accel_cal_value.value_z = 0;
//20111204 bg80.song@lge.com CAL Value Initialize for CAL Data Backup/Restore [E]
#endif

    return res;

err:
    sysfs_remove_group(&input_data->dev.kobj,
                       &mpuaccel_attribute_group);
    input_free_device(input_data);
    kfree(data);
    return res;

}



int
mpu_accel_exit(struct mldl_cfg* mldl_cfg)
{
    struct mpuaccel_data *data = NULL;

    if (mldl_cfg==NULL) return ML_ERROR;

    data = (struct mpuaccel_data*)mldl_cfg->ext.mpuacc_data;

    if (data!=NULL) {
        sysfs_remove_group(&(data->input_data->dev.kobj),
                           &mpuaccel_attribute_group);
        input_free_device(data->input_data);

        kfree(data);
        data = NULL;

        mldl_cfg->ext.mpuacc_data = NULL;
    }

    return ML_SUCCESS;
}

int
mpu_accel_suspend(struct mldl_cfg* mldl_cfg)
{
    int result = ML_SUCCESS;
    int enable = 0;
    struct mpuaccel_data *data = NULL;

    if (mldl_cfg==NULL) return ML_ERROR;
    data = (struct mpuaccel_data*)mldl_cfg->ext.mpuacc_data;

    mpu_accel_mutex_lock(data);
    enable = atomic_read(&data->enable);

    if (data->device_is_on==1 && enable==0) {
        result = mpu_accel_activate_device(data,0);
    }
    mpu_accel_mutex_unlock(data);

    return result;
}


int
mpu_accel_resume(struct mldl_cfg* mldl_cfg)
{
    int result = ML_SUCCESS;
    int enable = 0;
    struct mpuaccel_data *data = NULL;

    if (mldl_cfg==NULL) return ML_ERROR;
    data = (struct mpuaccel_data*)mldl_cfg->ext.mpuacc_data;

    mpu_accel_mutex_lock(data);
    enable = atomic_read(&data->enable);

    if (data->device_is_on==0 && enable==0) {
        result = mpu_accel_activate_device(data,1);
    }
    mpu_accel_mutex_unlock(data);

    return result;
}


int
mpu_accel_read(struct mldl_cfg* mldl_cfg, unsigned char* buffer)
{
    int result = ML_SUCCESS;
    int enable = 0;
    struct mpuaccel_data *data = NULL;

    if (mldl_cfg==NULL) return ML_ERROR;
    data = (struct mpuaccel_data*)mldl_cfg->ext.mpuacc_data;

    mpu_accel_mutex_lock(data);
    enable = atomic_read(&data->enable);
#ifdef MPUACC_USES_CACHED_DATA
    if (enable==1)
        memcpy(buffer,data->cached_data, sizeof(unsigned char)*6);
    else
#endif //MPUACC_USES_CACHED_DATA
        result = mpu_accel_get_data_from_device(data, buffer);
    mpu_accel_mutex_unlock(data);

    return result;
}

