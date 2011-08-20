/*
 *  gyro_accel.c
 *  star motion sensor driver  (Accelerometer, Gyroscope Sensor)
 *  Copyright (C) 2010 LGE Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <asm/gpio.h>
#include <asm/system.h>
#include <linux/workqueue.h>
#include <linux/device.h>
#include <linux/string.h>
#include <mach/hardware.h>
#include <asm/uaccess.h>
#include <linux/ioctl.h>
#include <linux/miscdevice.h>
#include <linux/regulator/consumer.h>

#include <nvodm_gyroscope_accel.h>
#include "nvodm_gyro_accel_kxtf9.h"
#include "mach/lprintk.h"

#include "gyro_accel.h"
#include "mpu3050.h"
#include "kxtf9.h"

#include "lge_sensor_verify.h"
#include "star_accel.h"
#include "ami304.h"

#define  DEBUG 0
#define  MISC_DYNAMIC_MINOR		 	255
#define  MAX_MOTION_DATA_LENGTH	 	10
#define  MIN_MOTION_POLLING_TIME    10
//#define  MIN_MOTION_POLLING_TIME    70//  motion daemon update every 70mS
//#define MAX_LGE_ACCEL_SAMPLERATE    7  /* for JSR256 API */

int sensor_sleep_st = 0;
int reboot = 0;
extern int call_once;

#ifdef CONFIG_HAS_EARLYSUSPEND // wkkim : temporary early suspend apply
#include <linux/earlysuspend.h>
static struct early_suspend    early_suspend;
#endif


struct tegra_gyro_accel_device_data
{
	//	struct task_struct	*task;
	struct input_dev	*input_dev;
};

struct tegra_acc_device_data *gyro_accel_dev=NULL;
struct i2c_client    *kxtf9_i2c_client;

static struct regulator 	 		*star_motion_reg;
static struct regulator				*star_gyro_vio_reg;

static atomic_t compass_flag;
static atomic_t accel_flag;
static atomic_t tilt_flag;
static atomic_t tap_flag;
static atomic_t shake_flag;
static atomic_t snap_flag;
static atomic_t flip_flag;
static atomic_t gyro_flag;
static atomic_t composite_flag;
static atomic_t accel_delay;
static atomic_t compass_delay;
static atomic_t tilt_delay;
static atomic_t gyro_delay;
static atomic_t composite_delay;
static atomic_t	jsr256_flag;
static atomic_t suspend_flag;
static atomic_t bypass_flag;
static atomic_t cal_flag;
static atomic_t cal_result;
//LGE_CHANGE_S [dongjin73.kim@lge.com] 2011-03-04, [LGE_AP20] Virtual sensor supports
static atomic_t gravity_flag;
static atomic_t linearaccel_flag;
static atomic_t rotvector_flag;
//LGE_CHANGE_E [dongjin73.kim@lge.com] 2011-03-04, [LGE_AP20] Virtual sensor supports

static NvBool i2c_busy_flag; //jongik2.kim 20100910 i2c_fix

static atomic_t	tilt_roll, tilt_pitch, tilt_yaw;
static atomic_t	accel_x, accel_y, accel_z;
static atomic_t	gyro_x, gyro_y, gyro_z;
static atomic_t	mag_x, mag_y, mag_z;
static atomic_t composite[3];

struct star_motion_device {
	NvOdmGyroAccelDeviceHandle	hOdmGyroAccel;
	struct input_dev  		    *input_dev;
	struct input_dev  		    *input_dev1;	 /* motion daemon process */
	NvU32						freq;
	NvBool						bThreadAlive;
	NvBool						show_log;
	NvOdmGyroAccelIntType		IntType;
	NvOdmGyroAccelAxisType		IntMotionAxis;
	NvOdmGyroAccelAxisType		IntTapAxis;
	struct timeval				tv;
	struct hrtimer       		timer[5];            /* [0] acceleroemter raw data, [1] tilt , [2] Gyro, [3] Compass, [4] composite */
	struct work_struct     		accel_work;
	struct workqueue_struct	 	*accel_wq;
	struct work_struct     		tilt_work;
	struct workqueue_struct	 	*tilt_wq;
	struct work_struct     		gyro_work;
	struct workqueue_struct	 	*gyro_wq;
	struct work_struct     		compass_work;
	struct workqueue_struct	 	*compass_wq;
	struct work_struct     		composite_work;
	struct workqueue_struct	 	*composite_wq;

	int  						irq;
	int  						use_irq;
};

static struct star_motion_device   *star_motion_dev = NULL;
void magnetic_input_report(int *); /* wkkim magnetic repot */

#define write_lock(lock)		_write_lock(lock)
#define read_lock(lock)			_read_lock(lock)
rwlock_t getbuflock;
static unsigned char accelrwbuf[200] = {0,};    /* MPU3050 i2c MAX data length */
static unsigned char rwbuf[200] = {0,};     	 /* MPU3050 i2c MAX data length */

static int  flip_test_count = 0;
static int  gyro_sleep_mode = 0;

/* wkkim add to read compass */
NvBool compassI2CSetRegs(NvU8 offset, NvU8* value, NvU32 len);
NvBool compassI2CGetRegs(NvU8 offset, NvU8* value, NvU32 len);
extern int AMI304_Reset_Init();
extern int AMI304_Init(int mode);
extern int kxtf9_get_acceleration_data_passthrough(int *xyz_data);
extern int tegra_accel_hw_init(void);
extern int lge_sensor_shutdown_proxi(void);
extern int lge_sensor_restart_proximity();
extern int tegra_compass_hw_init(void);

void NvOdmResetI2C(NvOdmGyroAccelHandle );
/** Function to close the ODM device. This function will help in switching
 * between power modes
 */
void close_odm_gyro_accel(void)
{
	NvOdmGyroAccelClose(star_motion_dev->hOdmGyroAccel);
	star_motion_dev->hOdmGyroAccel = 0;
}

/** Function to open the ODM device with a set of default values. The values
 * are hardcoded as of now. Each time the device is closed/open, previous
 * settings will be lost. This function will help in switching
 * between power modes
 */
int open_def_odm_gyro_accel(void)
{
	NvS32 err = -1;
	err = NvOdmGyroAccelOpen(&(star_motion_dev->hOdmGyroAccel));
	//lprintk(" ##  open_def_odm_gyro_accel  ##\n");
	if (!err) {
		err = -ENODEV;
#if DEBUG
		lprintk("open_def_odm_gyro_accel: NvOdmGyroAccelOpen failed\n");
#endif
		return err;
	}

	return err;
}

/*---------------------------------------------------------------------------
  kxtf9_reg_i2c_client
  ---------------------------------------------------------------------------*/
void  kxtf9_reg_i2c_client(struct i2c_client *client)
{
#if DEBUG
	lprintk("kxtf9_reg_i2c_client..........................\n");
#endif
	kxtf9_i2c_client =  client;
}
/*---------------------------------------------------------------------------
  kxtf9_read_reg_in_burst
  ---------------------------------------------------------------------------*/
int kxtf9_read_reg_in_burst(struct i2c_client *client, unsigned char reg,unsigned char *buf,int length)
{
	int err;
	unsigned char reg_val = reg;

	struct i2c_msg msg[2] = {
		{ client->addr, 0, 1,&reg_val },
		{ client->addr, I2C_M_RD, length, buf }
	};

	if ((err = i2c_transfer(client->adapter, msg, 2)) < 0) {
		dev_err(&client->dev, "i2c read error\n");
		return -EIO;
	}

	return 0;
}

/*---------------------------------------------------------------------------
  kxtf9_write_reg_in_burst
  ---------------------------------------------------------------------------*/
int kxtf9_write_reg_in_burst(struct i2c_client *client, unsigned char *value,int length)
{
	unsigned char buf[length];
	int err;
	struct i2c_msg msg ={
		.addr = client->addr,
		.flags = 0,
		.len   = sizeof(buf),
		.buf   = buf};

	memcpy(buf,value,length);

	if ((err = i2c_transfer(client->adapter, &msg, 1)) < 0) {
		dev_err(&client->dev, "i2c write error\n");
		return -EIO;
	}

	return 0;

}
/*---------------------------------------------------------------------------
  kxtf9_write_reg
  ---------------------------------------------------------------------------*/
int kxtf9_write_reg(struct i2c_client *client, unsigned char *buffer)
{
	unsigned char buf[2];
	int err;
	struct i2c_msg msg ={
		.addr = client->addr,
		.flags = 0,
		.len   = 2,
		.buf   = buf};

	buf[0] = buffer[0];
	buf[1] = buffer[1];

	if ((err = i2c_transfer(client->adapter, &msg, 1)) < 0) {
		dev_err(&client->dev, "i2c write error\n");
		return -EIO;
	}

	return 0;

}

/*---------------------------------------------------------------------------
  kxtf9_read_reg
  ---------------------------------------------------------------------------*/
int kxtf9_read_reg(struct i2c_client *client, unsigned char reg, unsigned char *value)
{
	int err;
	unsigned char buf = reg;

	struct i2c_msg msg[2] = {
		{ client->addr, 0, 1, &buf },
		{ client->addr, I2C_M_RD, 1, value }
	};

	if ((err = i2c_transfer(client->adapter, msg, 2)) < 0) {
		dev_err(&client->dev, "i2c read error\n");
		return -EIO;
	}

	return 0;
}



/*---------------------------------------------------------------------------
  kxtf9_i2c_read
  ---------------------------------------------------------------------------*/
int kxtf9_i2c_read(unsigned char reg,unsigned char *buf,int length)
{
	int status = 0;

	//status = kxtf9_read_reg_in_burst(kxtf9_i2c_client,reg,buf,length);
	//lprintk("[Gyro_accel][%s:%d]\n",__FUNCTION__, __LINE__);
	//lprintk("reg: %d(%x) / buf: %d(%x)/ length : %d(%x)\n",reg, reg, buf, buf, length, length);
	//NvAccelerometerI2CGetRegsPassThrough(reg, buf, length); // XOUT_L
	//NvAccelerometerI2CGetRegs(accel_dev->hOdmAcr, 0x06, &xyz[0], 6); // XOUT_L

	return status;
}

/*---------------------------------------------------------------------------
  kxtf9_i2c_write
  ---------------------------------------------------------------------------*/
int  kxtf9_i2c_write(unsigned char *buffer,int length)
{
	int status = 0;

	//status = kxtf9_write_reg_in_burst(kxtf9_i2c_client,buffer,length);
	//lprintk("[Gyro_accel][%s:%d]\n",__FUNCTION__, __LINE__);
	//lprintk("buffer[0]: %d(%x) / buffer[\1]: %d(%x)/ length : %d(%x)\n",buffer[0],buffer[0],buffer[1],buffer[1], length, length);
	//NvAccelerometerI2CSetRegsPassThrough(buffer[0] ,&buffer[1] , length-1 );
	//NvAccelerometerI2CSetRegs(accel_dev->hOdmAcr, 0x1B, &val, 1);


	return status;
}

int is_tap_enabled(void)
{
    return atomic_read(&tap_flag);
}

int is_flip_enabled(void)
{
    return atomic_read(&flip_flag);
}

int lge_sensor_shoutdown_all(void)
{
	printk("[%s] reboot gen2 i2c sensors\n",__func__);

	atomic_set(&bypass_flag, 0);
	
	//call reboot function
	lge_sensor_shutdown_compass();
	lge_sensor_shutdown_kxtf9();
	lge_sensor_shutdown_proxi();
	lge_sensor_shutdown_gyro();

	msleep(10);

	// do power down 
	lge_sensor_restart_gyro();
	lge_sensor_restart_kxtf9();
	lge_sensor_restart_compass();
	lge_sensor_restart_proximity();

	reboot	=	0;
}

int lge_sensor_shutdown_gyro(void)
{
    close_odm_gyro_accel();
    #if 0
    unsigned char value = 0;
    NvOdmResetI2C(star_motion_dev->hOdmGyroAccel);
    printk("[%s] ### Gyro ## Reset  \n",MOD_TAG);

    if(GYRO_WHID != value)
        return SENSOR_ERROR;
#endif
    return SENSOR_OK;
}
int lge_sensor_restart_gyro(void)
{
    int err; 
    
    err = open_def_odm_gyro_accel();
    if (!err) {
        return SENSOR_ERROR;
#if DEBUG
        lprintk("open_def_odm_gyro_accel\n");
#endif
    }    
    return SENSOR_OK;
}

/*---------------------------------------------------------------------------
  motion_sensor_power_on/off
  ---------------------------------------------------------------------------*/
#define ACCEL_CTRL_REG3		0x1D
#define ACCEL_CTRL_REG1		0x1B
#define ACCEL_PC1_ON		0x80
#define ACCEL_PC1_OFF			0x00  //stand-by-mode
#define AMI304_REG_CTRL1 0x1B
#define AMI304_REG_CTRL2 0x1C
#define AMI304_REG_CTRL3 0x1D

void motion_sensor_power_on(void)
{
	static unsigned char   	tempbuf[4]={0,};
	int err;
	u8 databuf[10];
	u8 ctrl1, ctrl2, ctrl3;

	/* Accelerometer power on */
#if DEBUG
	lprintk("[%s:%d] Accelerometer Sensor \n", __FUNCTION__, __LINE__);
#endif
	tempbuf[3]=0;
	tempbuf[1] = 1;
	NvAccelerometerI2CGetRegsPassThrough(ACCEL_CTRL_REG3, &tempbuf[3], tempbuf[1]);

	tempbuf[3] |= 1<<7;
	tempbuf[1] = 2;

	NvAccelerometerI2CSetRegsPassThrough(ACCEL_CTRL_REG3, &tempbuf[3] , tempbuf[1]-1);   // sets SRST bit to reboot
	msleep(1);
	//	mdelay(20);

	tempbuf[3] = ACCEL_PC1_ON;
	tempbuf[1] = 2;
	NvAccelerometerI2CSetRegsPassThrough(ACCEL_CTRL_REG1 ,&tempbuf[3] , tempbuf[1]-1);
	// NvOdmAccelerometerSetParameter(accel_dev->hOdmAcr, CTRL_REG1, PC1_ON);

}

void motion_sensor_power_off(void)
{
	static unsigned char   	tempbuf[4]={0,};
	int err;
	u8 databuf[10];
	u8 ctrl1, ctrl3;

	/* Accelerometer power off */
	tempbuf[3] = ACCEL_PC1_OFF;
	tempbuf[1] = 2;
	NvAccelerometerI2CSetRegsPassThrough(ACCEL_CTRL_REG1 ,&tempbuf[3] , tempbuf[1]-1);
	//NvAccelerometerI2CSetRegsPassThrough(accel_dev->hOdmAcr, CTRL_REG1, PC1_OFF);

#if DEBUG
	lprintk("[%s:%d] Accelerometer Sensor \n", __FUNCTION__, __LINE__);
#endif

}


/*---------------------------------------------------------------------------
  motion_send_event function
  ---------------------------------------------------------------------------*/
//motion_send_accel_detection
void motion_send_accel_detection(int accelx,int accely,int accelz)
{
	//lprintk("[Gyro_accel][%s:%d] %d %d %d\n",__FUNCTION__, __LINE__,accelx , accely, accelz );

	if (atomic_read(&accel_flag)) {
		input_report_abs(star_motion_dev->input_dev,ABS_X, accelx);
		input_report_abs(star_motion_dev->input_dev,ABS_Y, accely);
		input_report_abs(star_motion_dev->input_dev,ABS_Z, accelz);
		input_sync(star_motion_dev->input_dev);
	}
}

void motion_send_tilt_detection(int yaw,int pitch,int roll)
{
	/*DY2*///lprintk("[Gyro_accel][%s:%d]\n",__FUNCTION__, __LINE__);

	if (atomic_read(&tilt_flag)) {
		/*DY*///lprintk("[Gyro_accel][%s:%d]\n",__FUNCTION__, __LINE__);
		input_report_rel(star_motion_dev->input_dev,REL_RX, yaw);
		input_report_rel(star_motion_dev->input_dev,REL_RY, pitch);
		input_report_rel(star_motion_dev->input_dev,REL_RZ, roll);

		input_sync(star_motion_dev->input_dev);
	}
}

void motion_send_composite_detection(int *value)
{
	int buf[3] = {0,};

	memcpy(buf, value, sizeof(int) * 3);

	//	printk("composite %d %d %d \n", buf[0],buf[1],buf[2]);
	if (atomic_read(&composite_flag))
	{
		input_report_abs(star_motion_dev->input_dev, ABS_GAS, buf[0]);
		input_report_abs(star_motion_dev->input_dev, ABS_HAT1X, buf[1]);
		input_report_abs(star_motion_dev->input_dev, ABS_HAT1Y, buf[2]);
		//input_report_abs(star_motion_dev->input_dev, ABS_HAT2X, buf[3]);
		//input_report_abs(star_motion_dev->input_dev, ABS_HAT2Y, buf[4]);
		//input_report_abs(star_motion_dev->input_dev, ABS_HAT3X, buf[5]);
		//input_report_abs(star_motion_dev->input_dev, ABS_HAT3Y, buf[6]);
		//input_report_abs(star_motion_dev->input_dev, ABS_TILT_X, buf[7]);
		//input_report_abs(star_motion_dev->input_dev, ABS_TILT_Y, buf[8]);
		//input_report_abs(star_motion_dev->input_dev, ABS_TOOL_WIDTH, buf[9]);
		//input_report_abs(star_motion_dev->input_dev, ABS_VOLUME, buf[10]);
		//input_report_abs(star_motion_dev->input_dev, ABS_MISC, buf[11]);
		//input_report_rel(star_motion_dev->input_dev,REL_MISC,buf[12]);

		input_sync(star_motion_dev->input_dev);
	}
}

void motion_send_gyro_detection(int gyro_x,int gyro_y,int gyro_z)
{
	/*DY2*///lprintk(D_SENSOR,"[Gyro_accel][%s:%d]\n",__FUNCTION__, __LINE__);

	if (atomic_read(&gyro_flag)) {
		/*DY*///lprintk("[Gyro_accel][%s:%d]\n",__FUNCTION__, __LINE__);
		input_report_rel(star_motion_dev->input_dev,REL_Z, gyro_x);
		input_report_rel(star_motion_dev->input_dev,REL_MISC, gyro_y);
		input_report_rel(star_motion_dev->input_dev,REL_MAX, gyro_z);

		input_sync(star_motion_dev->input_dev);
	}
}

void motion_send_compass_detection(int mag_x,int mag_y,int mag_z)
{
	/*DY2*///lprintk(D_SENSOR,"[Gyro_accel][%s:%d]\n",__FUNCTION__, __LINE__);

	if (atomic_read(&compass_flag)) {
		//printk("[motion_send_compass_detection] %d %d %d \n", mag_x, mag_y, mag_z);
		input_report_abs(star_motion_dev->input_dev,ABS_HAT0X, mag_x);
		input_report_abs(star_motion_dev->input_dev,ABS_HAT0Y, mag_y);
		input_report_abs(star_motion_dev->input_dev,ABS_BRAKE, mag_z);

		input_sync(star_motion_dev->input_dev);
	}
}

void motion_send_tap_detection(int type,int direction)
{
#if DEBUG
	/*DY2*/lprintk(D_SENSOR,"[Gyro_accel][%s:%d]\n", __FUNCTION__, __LINE__);
#endif
	if(atomic_read(&tap_flag)) {
#if DEBUG
		/*DY*/lprintk(D_SENSOR,"[Gyro_accel][%s:%d]\n", __FUNCTION__, __LINE__);
#endif
		/*Test*///type= 111; direction = 222;
		input_report_rel(star_motion_dev->input_dev, REL_X, type);
		input_report_rel(star_motion_dev->input_dev, REL_Y, direction);

		input_sync(star_motion_dev->input_dev);
	}
}

void motion_send_flip_detection(int value)
{
#if DEBUG
	/*DY2*/lprintk(D_SENSOR,"[Gyro_accel][%s:%d]\n", __FUNCTION__, __LINE__);
#endif
	if (atomic_read(&flip_flag)) {
#if DEBUG
		/*DY*/lprintk(D_SENSOR,"[Gyro_accel][%s:%d]\n", __FUNCTION__, __LINE__);
#endif
		input_report_rel(star_motion_dev->input_dev, REL_WHEEL, value);
		input_sync(star_motion_dev->input_dev);
	}
}

void motion_send_shake_detection(int value)
{
#if DEBUG
	/*DY2*/lprintk(D_SENSOR,"[Gyro_accel][%s:%d]\n", __FUNCTION__, __LINE__);
#endif
	if (atomic_read(&shake_flag)) {
#if DEBUG
		/*DY*/lprintk(D_SENSOR,"[Gyro_accel][%s:%d]\n", __FUNCTION__, __LINE__);
#endif
		input_report_rel(star_motion_dev->input_dev, REL_HWHEEL, value);
		input_sync(star_motion_dev->input_dev);
	}
}

void motion_send_snap_detection(int direction)
{
#if DEBUG
	/*DY2*/lprintk(D_SENSOR,"[Gyro_accel][%s:%d]\n", __FUNCTION__, __LINE__);
#endif
	if (atomic_read(&snap_flag)) {
#if DEBUG
		/*DY*/lprintk(D_SENSOR,"[Gyro_accel][%s:%d]\n", __FUNCTION__, __LINE__);
#endif
		input_report_rel(star_motion_dev->input_dev,REL_DIAL,direction);
		input_sync(star_motion_dev->input_dev);
	}
}

//LGE_CHANGE_S [dongjin73.kim@lge.com] 2011-03-04, [LGE_AP20] Virtual sensor supports
#define STAR_GRAVITY_X		0x30
#define STAR_GRAVITY_Y		0x31
#define STAR_GRAVITY_Z		0x32
#define STAR_LINEARACCEL_X	0x33
#define STAR_LINEARACCEL_Y	0x34
#define STAR_LINEARACCEL_Z	0x35
#define STAR_ROTVECTOR_X	0x36
#define STAR_ROTVECTOR_Y	0x37
#define STAR_ROTVECTOR_Z	0x38

void motion_send_gravity_detection(int x, int y, int z)
{
	if (atomic_read(&gravity_flag)) {
		input_report_abs(star_motion_dev->input_dev, STAR_GRAVITY_X, x);
		input_report_abs(star_motion_dev->input_dev, STAR_GRAVITY_Y, y);
		input_report_abs(star_motion_dev->input_dev, STAR_GRAVITY_Z, z);
		input_sync(star_motion_dev->input_dev);
	}
}

void motion_send_linearaccel_detection(int x, int y, int z)
{
	if (atomic_read(&linearaccel_flag)) {
		input_report_abs(star_motion_dev->input_dev, STAR_LINEARACCEL_X, x);
		input_report_abs(star_motion_dev->input_dev, STAR_LINEARACCEL_Y, y);
		input_report_abs(star_motion_dev->input_dev, STAR_LINEARACCEL_Z, z);
		input_sync(star_motion_dev->input_dev);
	}
}

void motion_send_rotvector_detection(int x, int y, int z)
{
	if (atomic_read(&rotvector_flag)) {
		input_report_abs(star_motion_dev->input_dev, STAR_ROTVECTOR_X, x);
		input_report_abs(star_motion_dev->input_dev, STAR_ROTVECTOR_Y, y);
		input_report_abs(star_motion_dev->input_dev, STAR_ROTVECTOR_Z, z);
		input_sync(star_motion_dev->input_dev);
	}
}
//LGE_CHANGE_E [dongjin73.kim@lge.com] 2011-03-04, [LGE_AP20] Virtual sensor supports

/*---------------------------------------------------------------------------
  work function
  ---------------------------------------------------------------------------*/

static void motion_accel_work_func(struct work_struct *work)
{
	int current_x = 0, current_y = 0, current_z = 0;
	u8 raw_data[6]; // xyz data bytes from hardware
	int accel_data[3];

	if(atomic_read(&bypass_flag)){
		kxtf9_get_acceleration_data_passthrough(accel_data);
		current_x = accel_data[0]*10;
		current_y = accel_data[1]*10;
		current_z = accel_data[2]*10;
	}else{
		current_x = atomic_read(&accel_x);
		current_y = atomic_read(&accel_y);
		current_z = atomic_read(&accel_z);
	}
	motion_send_accel_detection(current_x, current_y, current_z);
}

static void motion_tilt_work_func(struct work_struct *work)
{
	int current_yaw = 0, current_pitch = 0, current_roll = 0;
	int i = 0;

	current_yaw   = atomic_read(&tilt_yaw);
	current_pitch = atomic_read(&tilt_pitch);
	current_roll  = atomic_read(&tilt_roll);

	//lprintk("[motion_tilt_work_func] current_roll=[%d], current_pitch=[%d], current_yaw=[%d] \n",current_roll,current_pitch,current_yaw);

	motion_send_tilt_detection(current_yaw, current_pitch, current_roll);

}

static void motion_gyro_work_func(struct work_struct *work)
{
	int current_x = 0, current_y = 0, current_z = 0;

	current_x = atomic_read(&gyro_x);
	current_y = atomic_read(&gyro_y);
	current_z = atomic_read(&gyro_z);

	motion_send_gyro_detection(current_x,current_y,current_z);
}

static void motion_compass_work_func(struct work_struct *work)
{
	int mag_val[3];

	mag_val[0]= atomic_read(&mag_x);
	mag_val[1]= atomic_read(&mag_y);
	mag_val[2]= atomic_read(&mag_z);

	motion_send_compass_detection(mag_val[0],mag_val[1],mag_val[2]);
}

static void motion_composite_work_func(struct work_struct *work)
{
	int data[3] = {0,};
	int i = 0;

	for (i = 0; i < 3; i++) {
		data[i] = atomic_read(&composite[i]);
	}
	motion_send_composite_detection(data);

}

/*---------------------------------------------------------------------------
  motion polling timer
  ---------------------------------------------------------------------------*/
static enum hrtimer_restart motion_accel_timer_func(struct hrtimer *timer)
{
	unsigned long polling_time;

	if (atomic_read(&accel_flag)) {
		queue_work(star_motion_dev->accel_wq, &star_motion_dev->accel_work);

		polling_time = atomic_read(&accel_delay);
		hrtimer_start(&star_motion_dev->timer[0], ktime_set(0, polling_time * 1000000), HRTIMER_MODE_REL);
	}

	return HRTIMER_NORESTART;
}

static enum hrtimer_restart motion_tilt_timer_func(struct hrtimer *timer)
{
	unsigned long  polling_time;

	if (atomic_read(&tilt_flag)) {
		queue_work(star_motion_dev->tilt_wq, &star_motion_dev->tilt_work);

		polling_time = atomic_read(&tilt_delay);
		//lprintk("[Gyro_accel][%s:%d]\n",__FUNCTION__, __LINE__);
		hrtimer_start(&star_motion_dev->timer[1], ktime_set(0, polling_time * 1000000), HRTIMER_MODE_REL);
	}

	return HRTIMER_NORESTART;
}

static enum hrtimer_restart motion_gyro_timer_func(struct hrtimer *timer)
{
	unsigned long polling_time;

	if (atomic_read(&gyro_flag))
	{
		queue_work(star_motion_dev->gyro_wq, &star_motion_dev->gyro_work);

		polling_time = atomic_read(&gyro_delay);
		hrtimer_start(&star_motion_dev->timer[2],ktime_set(0,polling_time*1000000),HRTIMER_MODE_REL);
	}

	return HRTIMER_NORESTART;
}

static enum hrtimer_restart motion_compass_timer_func(struct hrtimer *timer)
{
	unsigned long polling_time;

	if (atomic_read(&compass_flag)) {
		queue_work(star_motion_dev->compass_wq, &star_motion_dev->compass_work);

		polling_time = atomic_read(&compass_delay);
		hrtimer_start(&star_motion_dev->timer[3], ktime_set(0, polling_time * 1000000), HRTIMER_MODE_REL);
	}

	return HRTIMER_NORESTART;
}

static enum hrtimer_restart motion_composite_timer_func(struct hrtimer *timer)
{
	unsigned long polling_time;

	if (atomic_read(&composite_flag)) {
		queue_work(star_motion_dev->composite_wq, &star_motion_dev->composite_work);

		polling_time = atomic_read(&composite_delay);
		hrtimer_start(&star_motion_dev->timer[4], ktime_set(0, polling_time * 1000000), HRTIMER_MODE_REL);
	}

	return HRTIMER_NORESTART;
}

/*---------------------------------------------------------------------------
  sensor enable/disable (Sensor HAL)
  ---------------------------------------------------------------------------*/
static ssize_t motion_accel_onoff_show(struct device *dev, struct device_attribute *attr, char *buf, size_t count)
{
	u32    val;
	val = atomic_read(&accel_flag);
	return sprintf(buf, "%d\n",val);
}

static ssize_t motion_accel_onoff_store(struct device *dev, struct device_attribute *attr, char *buf, size_t count)
{
	u32    val;
	val = simple_strtoul(buf, NULL, 10);
	//	printk("[%s] gyro_flag [%d]\n",__func__,val);

	if (val) {
		atomic_set(&accel_flag, 1);
		//		hrtimer_cancel(&star_motion_dev->timer[0]);
		//		hrtimer_start(&star_motion_dev->timer[0], ktime_set(0,0), HRTIMER_MODE_REL);
	} else {
		atomic_set(&accel_flag, 0);
		//		hrtimer_cancel(&star_motion_dev->timer[0]);
	}

	return count;
}

static ssize_t motion_tilt_onoff_show(struct device *dev, struct device_attribute *attr, char *buf, size_t count)
{
	u32    val;
	val = atomic_read(&tilt_flag);
	return sprintf(buf, "%d\n",val);
}

static ssize_t motion_tilt_onoff_store(struct device *dev, struct device_attribute *attr, char *buf, size_t count)
{
	u32    val;
	val = simple_strtoul(buf, NULL, 10);
	//lprintk("[motion_set_tilt_onoff_store]  flag [%d]\n",val);

	if (val) {
		atomic_set(&tilt_flag, 1);
	} else {
		atomic_set(&tilt_flag, 0);

		atomic_set(&tilt_roll,  0);
		atomic_set(&tilt_pitch, 0);
		atomic_set(&tilt_yaw,   0);
		
	}

	return count;
}

static ssize_t motion_gyro_onoff_show(struct device *dev, struct device_attribute *attr, char *buf, size_t count)
{
	u32    val;
	val = atomic_read(&gyro_flag);
	return sprintf(buf, "%d\n",val);
}

static ssize_t motion_gyro_onoff_store(struct device *dev, struct device_attribute *attr, char *buf, size_t count)
{
	u32    val;
	val = simple_strtoul(buf, NULL, 10);
	//lprintk("[motion_gyro_onoff_store] gyro_flag [%d]\n",val);

	if (val) {
		atomic_set(&gyro_flag, 1);
	} else {
		atomic_set(&gyro_flag, 0);

		atomic_set(&gyro_x,  0);
		atomic_set(&gyro_y, 0);
		atomic_set(&gyro_z,   0);
	}

	return count;
}

static ssize_t motion_compass_onoff_show(struct device *dev,  struct device_attribute *attr,  char *buf, size_t count)
{
	u32    val;
	val = atomic_read(&compass_flag);
	return sprintf(buf, "%d\n",val);
}

static ssize_t motion_compass_onoff_store(struct device *dev,  struct device_attribute *attr,  char *buf, size_t count)
{
	u32    val;
	val = simple_strtoul(buf, NULL, 10);

	if (val) {
		atomic_set(&compass_flag, 1);
	} else {
		atomic_set(&compass_flag, 0);
	}

	return count;
}

static ssize_t motion_composite_onoff_show(struct device *dev, struct device_attribute *attr, char *buf, size_t count)
{
	u32    val;
	val = atomic_read(&composite_flag);
	return sprintf(buf, "%d\n",val);
}

static ssize_t motion_composite_onoff_store(struct device *dev, struct device_attribute *attr, char *buf, size_t count)
{
	u32    val;
	val = simple_strtoul(buf, NULL, 10);
	//lprintk("[motion_set_tilt_onoff_store]  flag [%d]\n",val);

	if (val) {
		atomic_set(&composite_flag, 1);
	} else {
		atomic_set(&composite_flag, 0);

		atomic_set(&composite[0],  0);
		atomic_set(&composite[1],  0);
		atomic_set(&composite[2],  0);		
	}

	return count;
}

static ssize_t motion_tap_onoff_show(struct device *dev, struct device_attribute *attr, char *buf, size_t count)
{
	u32    val;
	val = atomic_read(&tap_flag);
	return sprintf(buf, "%d\n",val);
}

static ssize_t motion_tap_onoff_store(struct device *dev, struct device_attribute *attr, char *buf, size_t count)
{
	u32    val;
	val = simple_strtoul(buf, NULL, 10);
	//lprintk("[motion_set_tap_onoff_store] tap.... flag [%d]\n",val);

	if (val) {
		atomic_set(&tap_flag, 1);
	} else {
		atomic_set(&tap_flag, 0);
	}

	return count;
}

static ssize_t motion_flip_onoff_show(struct device *dev, struct device_attribute *attr, char *buf, size_t count)
{
	u32    val;
	val = atomic_read(&flip_flag);
	return sprintf(buf, "%d\n",val);
}

static ssize_t motion_flip_onoff_store(struct device *dev, struct device_attribute *attr, char *buf, size_t count)
{
	u32    val;
	val = simple_strtoul(buf, NULL, 10);
	//lprintk("[motion_set_flip_onoff_store]  flag [%d]\n",val);

	if (val) {
		atomic_set(&flip_flag, 1);
		call_once = 1;
	} else {
		atomic_set(&flip_flag, 0);
	}

	return count;
}

static ssize_t motion_shake_onoff_show(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u32    val;
	val = atomic_read(&shake_flag);
	return sprintf(buf, "%d\n",val);
}

static ssize_t motion_shake_onoff_store(struct device *dev,  struct device_attribute *attr,const char *buf, size_t count)
{
	u32    val;
	val = simple_strtoul(buf, NULL, 10);
	//lprintk("[motion_set_shake_onoff_store]  flag [%d]\n",val);

	if (val) {
		atomic_set(&shake_flag, 1);
	} else {
		atomic_set(&shake_flag, 0);
	}

	return count;
}

static ssize_t motion_snap_onoff_show(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u32    val;
	val = atomic_read(&snap_flag);
	return sprintf(buf, "%d\n",val);
}

static ssize_t motion_snap_onoff_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u32    val;
	val = simple_strtoul(buf, NULL, 10);
	//lprintk("[motion_set_snap_onoff_store]  flag [%d]\n",val);

	if (val) {
		atomic_set(&snap_flag, 1);
	} else {
		atomic_set(&snap_flag, 0);
	}

	return count;
}

/*---------------------------------------------------------------------------
  control set delay  (Sensor HAL)
  ---------------------------------------------------------------------------*/
static ssize_t motion_accel_delay_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u32    val;
	unsigned long current_delay = 100;

	val = simple_strtoul(buf, NULL, 10);
	current_delay = (short)val;
#if DEBUG
	lprintk(D_SENSOR,"[motion_accel_delay_store]  flag [%d] current_delay[%ld]\n",val, current_delay);
#endif
	if (atomic_read(&accel_flag)) {
		hrtimer_cancel(&star_motion_dev->timer[0]);

		if (current_delay < MIN_MOTION_POLLING_TIME) {
			current_delay = MIN_MOTION_POLLING_TIME;
		}

		atomic_set(&accel_delay, current_delay);
		hrtimer_start(&star_motion_dev->timer[0], ktime_set(0, current_delay * 1000000), HRTIMER_MODE_REL);
	}

	return count;
}

static ssize_t motion_tilt_delay_store(struct device *dev,  struct device_attribute *attr,const char *buf, size_t count)
{
	u32     val;
	unsigned long   current_delay = 100;

	val = simple_strtoul(buf, NULL, 10);
	current_delay = (short)val;

#if DEBUG
	lprintk(D_SENSOR, "[motion_set_tilt_delay_store]  val [%d] current_delay[%ld]\n", val, current_delay);
#endif
	if (atomic_read(&tilt_flag)) {
		hrtimer_cancel(&star_motion_dev->timer[1]);

		if (current_delay < MIN_MOTION_POLLING_TIME) {
			current_delay = MIN_MOTION_POLLING_TIME;
		}

		atomic_set(&tilt_delay, current_delay);
		hrtimer_start(&star_motion_dev->timer[1], ktime_set(0, current_delay * 1000000), HRTIMER_MODE_REL);

	}

	return count;
}

ssize_t motion_gyro_delay_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u32    val;
	unsigned long current_delay = 100;

	val = simple_strtoul(buf, NULL, 10);
	current_delay = (short)val;

#if DEBUG
	lprintk(D_SENSOR, "[motion_gyro_delay_store]  val [%d] current_delay[%ld]\n",val,current_delay);
#endif

	if (atomic_read(&gyro_flag)) {
		hrtimer_cancel(&star_motion_dev->timer[2]);

		if (current_delay < MIN_MOTION_POLLING_TIME) {
			current_delay = MIN_MOTION_POLLING_TIME;
		}

		atomic_set(&gyro_delay, current_delay);
		hrtimer_start(&star_motion_dev->timer[2], ktime_set(0, current_delay * 1000000), HRTIMER_MODE_REL);

	}

	return count;
}

ssize_t motion_compass_delay_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u32    val;
	unsigned long current_delay = 100;

	val = simple_strtoul(buf, NULL, 10);
	current_delay = (short)val;

#if DEBUG
	lprintk(D_SENSOR,"[motion_compass_delay_store]  flag [%d] current_delay[%ld]\n",val, current_delay);
#endif

	if (atomic_read(&compass_flag)) {
		hrtimer_cancel(&star_motion_dev->timer[3]);

		if (current_delay < MIN_MOTION_POLLING_TIME) {
			current_delay = MIN_MOTION_POLLING_TIME;
		}

		atomic_set(&compass_delay, current_delay);
		hrtimer_start(&star_motion_dev->timer[3], ktime_set(0, current_delay * 1000000), HRTIMER_MODE_REL);
	}

	return count;
}

ssize_t motion_composite_delay_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u32    val;
	unsigned long current_delay = 100;

	val = simple_strtoul(buf, NULL, 10);
	current_delay = (short)val;

#if DEBUG
	lprintk(D_SENSOR, "[motion_gyro_delay_store]  val [%d] current_delay[%ld]\n",val,current_delay);
#endif

	if (atomic_read(&composite_flag)) {
		hrtimer_cancel(&star_motion_dev->timer[4]);

		if (current_delay < MIN_MOTION_POLLING_TIME) {
			current_delay = MIN_MOTION_POLLING_TIME;
		}

		atomic_set(&composite_delay, current_delay);
		hrtimer_start(&star_motion_dev->timer[4], ktime_set(0, current_delay * 1000000), HRTIMER_MODE_REL);

	}

	return count;
}

static ssize_t motion_cal_onoff_show(struct device *dev, struct device_attribute *attr, char *buf, size_t count)
{
	u32    val;
	val = atomic_read(&cal_result);
	atomic_set(&cal_result, 2);
	return sprintf(buf, "%d\n",val);
}

static ssize_t motion_cal_onoff_store(struct device *dev, struct device_attribute *attr, char *buf, size_t count)
{
	u32    val;
	val = simple_strtoul(buf, NULL, 10);

	if (val) {
		atomic_set(&cal_flag, 1);
		atomic_set(&suspend_flag, 0);
	} else {
		atomic_set(&cal_flag, 0);
	}

	return count;
}

static ssize_t motion_sensors_reboot_store(struct device *dev,  struct device_attribute *attr,const char *buf, size_t count)
{
	u32    val;
	val = simple_strtoul(buf, NULL, 10);
	//lprintk("[motion_set_shake_onoff_store]  flag [%d]\n",val);

	if (val) {
		reboot = 1; 
	} else {
		reboot = 0; 
	}

	return count;
}

//LGE_CHANGE_S [dongjin73.kim@lge.com] 2011-03-04, [LGE_AP20] Virtual sensor supports
static ssize_t motion_gravity_onoff_show(struct device *dev, struct device_attribute *attr, char *buf, size_t count)
{
	u32    val = atomic_read(&gravity_flag);
	return sprintf(buf, "%d\n",val);
}

static ssize_t motion_gravity_onoff_store(struct device *dev, struct device_attribute *attr, char *buf, size_t count)
{
	u32    val = simple_strtoul(buf, NULL, 10);
	atomic_set(&gravity_flag, val ? 1 : 0);

	return count;
}

static ssize_t motion_linearaccel_onoff_show(struct device *dev, struct device_attribute *attr, char *buf, size_t count)
{
	u32    val = atomic_read(&linearaccel_flag);
	return sprintf(buf, "%d\n",val);
}

static ssize_t motion_linearaccel_onoff_store(struct device *dev, struct device_attribute *attr, char *buf, size_t count)
{
	u32    val = simple_strtoul(buf, NULL, 10);
	atomic_set(&linearaccel_flag, val ? 1 : 0);

	return count;
}

static ssize_t motion_rotvector_onoff_show(struct device *dev, struct device_attribute *attr, char *buf, size_t count)
{
	u32    val = atomic_read(&rotvector_flag);
	return sprintf(buf, "%d\n",val);
}

static ssize_t motion_rotvector_onoff_store(struct device *dev, struct device_attribute *attr, char *buf, size_t count)
{
	u32    val = simple_strtoul(buf, NULL, 10);
	atomic_set(&rotvector_flag, val ? 1 : 0);

	return count;
}
//LGE_CHANGE_E [dongjin73.kim@lge.com] 2011-03-04, [LGE_AP20] Virtual sensor supports

static DEVICE_ATTR(accel_onoff, 0666, motion_accel_onoff_show, motion_accel_onoff_store);
static DEVICE_ATTR(tilt_onoff, 0666, motion_tilt_onoff_show, motion_tilt_onoff_store);
static DEVICE_ATTR(gyro_onoff, 0666, motion_gyro_onoff_show, motion_gyro_onoff_store);
static DEVICE_ATTR(compass_onoff, 0666, motion_compass_onoff_show, motion_compass_onoff_store);
static DEVICE_ATTR(tap_onoff, 0666, motion_tap_onoff_show, motion_tap_onoff_store);
static DEVICE_ATTR(flip_onoff, 0666, motion_flip_onoff_show, motion_flip_onoff_store);
static DEVICE_ATTR(shake_onoff,0666, motion_shake_onoff_show, motion_shake_onoff_store);
static DEVICE_ATTR(snap_onoff, 0666, motion_snap_onoff_show, motion_snap_onoff_store);
static DEVICE_ATTR(composite_onoff, 0666, motion_composite_onoff_show, motion_composite_onoff_store);
//LGE_CHANGE_S [dongjin73.kim@lge.com] 2011-03-04, [LGE_AP20] Virtual sensor supports
static DEVICE_ATTR(gravity_onoff, 0666, motion_gravity_onoff_show, motion_gravity_onoff_store);
static DEVICE_ATTR(linearaccel_onoff, 0666, motion_linearaccel_onoff_show, motion_linearaccel_onoff_store);
static DEVICE_ATTR(rotvector_onoff, 0666, motion_rotvector_onoff_show, motion_rotvector_onoff_store);
//LGE_CHANGE_E [dongjin73.kim@lge.com] 2011-03-04, [LGE_AP20] Virtual sensor supports

static DEVICE_ATTR(accel_delay, 0666, NULL, motion_accel_delay_store);
static DEVICE_ATTR(tilt_delay, 0666, NULL, motion_tilt_delay_store);
static DEVICE_ATTR(gyro_delay, 0666, NULL, motion_gyro_delay_store);
static DEVICE_ATTR(compass_delay, 0666,NULL, motion_compass_delay_store);
static DEVICE_ATTR(composite_delay, 0666,NULL, motion_composite_delay_store);

//Sensor Calibration
static DEVICE_ATTR(cal_onoff, 0666, motion_cal_onoff_show, motion_cal_onoff_store);

//Sensor reboot test
static DEVICE_ATTR(reboot, 0666, NULL, motion_sensors_reboot_store);

static struct attribute *star_motion_attributes[] = {
	&dev_attr_accel_onoff.attr,
	&dev_attr_gyro_onoff.attr,
	&dev_attr_tilt_onoff.attr,
	&dev_attr_compass_onoff.attr,
	&dev_attr_composite_onoff.attr,	
	&dev_attr_tap_onoff.attr,
	&dev_attr_shake_onoff.attr,
	&dev_attr_snap_onoff.attr,
	&dev_attr_flip_onoff.attr,
	&dev_attr_tilt_delay.attr,
	&dev_attr_accel_delay.attr,
	&dev_attr_gyro_delay.attr,
	&dev_attr_compass_delay.attr,
	&dev_attr_cal_onoff.attr,
	&dev_attr_composite_delay.attr,
	&dev_attr_reboot.attr,
//LGE_CHANGE_S [dongjin73.kim@lge.com] 2011-03-04, [LGE_AP20] Virtual sensor supports
	&dev_attr_gravity_onoff.attr,
	&dev_attr_linearaccel_onoff.attr,
	&dev_attr_rotvector_onoff.attr,
//LGE_CHANGE_E [dongjin73.kim@lge.com] 2011-03-04, [LGE_AP20] Virtual sensor supports
	NULL
};

static const struct attribute_group star_motion_group = {
	.attrs = star_motion_attributes,
};

/*---------------------------------------------------------------------------
  ioctl command for heaven motion daemon process
  ---------------------------------------------------------------------------*/
static int star_motion_open(struct inode *inode, struct file *file)
{
	int status = -1;
	status = nonseekable_open(inode,file);

	return status;
}
static int star_motion_release(struct inode *inode, struct file *file)
{
	//lprintk("motion close\n");
	return 0;
}
static int count = 0 ;
static int star_motion_ioctl(struct inode *inode, struct file *file, unsigned int cmd,unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	unsigned char data[MAX_MOTION_DATA_LENGTH]={0,};
	unsigned char tempbuf[200] = {0,};     /* MPU3050 i2c MAX data length */
	unsigned char value;

	int buf[13] = {0,};
	int flag = 0;
	int delay = 0;
	int onoff_flag = 0;
	int ret = 0;
	int i = 0;

	switch (cmd) {
		case MOTION_IOCTL_ENABLE_DISABLE:
			/*
0 : disable sensor
1:  orientation (tilt)
2:  accelerometer
3: tap
4: shake
*/
			//lprintk(".............star_motion_ioctl................\n");
			flag = STAR_SENSOR_NONE;

			if (atomic_read(&accel_flag)) {
				//lprintk(".............if(atomic_read(&snap_flag)){................\n");
				flag |= STAR_ACCELEROMETER;
			}

			if (atomic_read(&tilt_flag)) {
				//lprintk(".............if(atomic_read(&tilt_flag)){................\n");
				flag |= STAR_TILT;
			}

			if (atomic_read(&gyro_flag)) {
				//lprintk(".............if(atomic_read(&snap_flag)){................\n");
				flag |= STAR_GYRO;
			}

			if (atomic_read(&compass_flag)) {
				//lprintk(".............if(atomic_read(&compass_flag)){................\n");
				flag |= STAR_COMPASS;
			}

			if (atomic_read(&composite_flag)) {
				//lprintk(".............if(atomic_read(&compass_flag)){................\n");
				flag |= STAR_COMPOSITE;
			}

			if (atomic_read(&tap_flag)) {
				//lprintk(".............if(atomic_read(&tap_flag)){................\n");
				flag |= STAR_TAP;
			}

			if (atomic_read(&flip_flag)) {
				//lprintk(".............if(atomic_read(&flip_flag)){................\n");
				flag |= STAR_FLIP;
			}

			if (atomic_read(&shake_flag)) {
				//lprintk(".............if(atomic_read(&shake_flag)){................\n");
				flag |= STAR_SHAKE;
			}

			if (atomic_read(&snap_flag)) {
				//lprintk(".............if(atomic_read(&snap_flag)){................\n");
				flag |= STAR_SNAP;
			}

			if (atomic_read(&cal_flag)) {
				//lprintk(".............if(atomic_read(&snap_flag)){................\n");
				flag |= STAR_CALIBRATION;
			}

//LGE_CHANGE_S [dongjin73.kim@lge.com] 2011-03-04, [LGE_AP20] Virtual sensor supports
			if (atomic_read(&gravity_flag)) {
				flag |= STAR_GRAVITY;
			}

			if (atomic_read(&linearaccel_flag)) {
				flag |= STAR_LINEARACCEL;
			}

			if (atomic_read(&rotvector_flag)) {
				flag |= STAR_ROTATIONVECTOR;
			}
//LGE_CHANGE_E [dongjin73.kim@lge.com] 2011-03-04, [LGE_AP20] Virtual sensor supports

			if (copy_to_user(argp,&flag, sizeof(flag))) {
				//lprintk(".............MOTION_IOCTL_SNAP................\n");
				return -EFAULT;
			}
			break;
		case MOTION_IOCTL_ACCEL_RAW:
			if (copy_from_user(&buf, argp, sizeof(buf))) {
				return -EFAULT;
			}
			/* 	buf[0], [1], [2] = accel_x,  accel_y,  accel_z;	 */

			atomic_set(&accel_x, buf[0]);
			atomic_set(&accel_y, buf[1]);
			atomic_set(&accel_z, buf[2]);
#if 0 /*ACCEL_REPORT*/
			motion_send_accel_detection(buf[0],buf[1],buf[2]);
#endif
			//lprintk(".............MOTION_IOCTL_TILT................\n");
			break;
		case MOTION_IOCTL_TILT:
			if (copy_from_user(&buf, argp, sizeof(buf))) {
				return -EFAULT;
			}
			/* 	buf[0], [1], [2] = roll,  pitch,  yaw;	 */

			atomic_set(&tilt_yaw, buf[0]);
			atomic_set(&tilt_pitch, buf[1]);
			atomic_set(&tilt_roll, buf[2]);
			//lprintk(".............MOTION_IOCTL_TILT................\n");
			break;
		case MOTION_IOCTL_COMPOSITE:
			if (copy_from_user(&buf, argp, sizeof(buf))) {
				return -EFAULT;
			}

			for (i = 0; i < 3; i++) {
				atomic_set(&composite[i], buf[i]);
			}

			//motion_send_composite_detection(buf);
			break;
		case MOTION_IOCTL_GYRO_RAW:
			if (copy_from_user(&buf, argp, sizeof(buf))) {
				return -EFAULT;
			}
			/* 	buf[0], [1], [2] = gyro_x,  gyro_y,  gyro_z; */

			atomic_set(&gyro_x, buf[0]);
			atomic_set(&gyro_y, buf[1]);
			atomic_set(&gyro_z, buf[2]);

			//motion_send_tilt_detection(buf[0],buf[1],buf[2]);
			//lprintk(".............MOTION_IOCTL_GYRO_RAW................\n");
			break;
		case MOTION_IOCTL_MAGNETIC_RAW: /* wkkim add to recieve compass raw value */
			if (copy_from_user(&buf, argp, sizeof(buf))) {
				return -EFAULT;
			}
			atomic_set(&mag_x, buf[0]);
			atomic_set(&mag_y, buf[1]);
			atomic_set(&mag_z, buf[2]);
			break;
//LGE_CHANGE_S [dongjin73.kim@lge.com] 2011-03-04, [LGE_AP20] Virtual sensor supports
		case MOTION_IOCTL_GRAVITY:
			if (copy_from_user(&buf, argp, sizeof(buf))) {
				return -EFAULT;
			}
			motion_send_gravity_detection(buf[0], buf[1], buf[2]);
			break;
		case MOTION_IOCTL_LINEARACCEL:
			if (copy_from_user(&buf, argp, sizeof(buf))) {
				return -EFAULT;
			}
			motion_send_linearaccel_detection(buf[0], buf[1], buf[2]);
			break;
		case MOTION_IOCTL_ROTVECTOR:
			if (copy_from_user(&buf, argp, sizeof(buf))) {
				return -EFAULT;
			}
			motion_send_rotvector_detection(buf[0], buf[1], buf[2]);
			break;
//LGE_CHANGE_E [dongjin73.kim@lge.com] 2011-03-04, [LGE_AP20] Virtual sensor supports
		case MOTION_IOCTL_TAP:
			if (copy_from_user(&buf, argp, sizeof(buf))) {
				return -EFAULT;
			}
			/*
			   buf[0] = type;
			   buf[1] = direction;
			   */
#if DEBUG
			lprintk(D_SENSOR,".............MOTION_IOCTL_TAP................\n");
#endif
			motion_send_tap_detection(buf[0], buf[1]);
			break;
		case MOTION_IOCTL_FLIP:
			if (copy_from_user(&buf, argp, sizeof(buf))) {
				return -EFAULT;
			}
#if DEBUG
			lprintk(D_SENSOR,".............MOTION_IOCTL_FLIP................\n");
#endif
			motion_send_flip_detection(buf[0]);
			break;
		case MOTION_IOCTL_SHAKE:
			if (copy_from_user(&buf, argp, sizeof(buf))) {
				return -EFAULT;
			}
			/* 			buf[0] = event;   		  */
#if DEBUG
			lprintk(D_SENSOR,".............MOTION_IOCTL_SHAKE................\n");
#endif
			motion_send_shake_detection(buf[0]);
			break;
		case MOTION_IOCTL_SNAP:
			if (copy_from_user(&buf, argp, sizeof(buf))) {
				return -EFAULT;
			}
			/*
			   buf[0] = direction;
			   */
#if DEBUG
			lprintk(D_SENSOR,".............MOTION_IOCTL_SNAP................\n");
#endif
			motion_send_snap_detection(buf[0]);
			break;
		case MOTION_IOCTL_SENSOR_DELAY:
			delay = atomic_read(&accel_delay);

			//lprintk("MOTION_IOCTL_SENSOR_DELAY[%d]",delay);

			if (copy_to_user(argp, &delay, sizeof(delay))) {
				return -EFAULT;
			}
			break;
		case MOTION_IOCTL_SENSOR_SUSPEND_RESUME:
			onoff_flag = atomic_read(&suspend_flag);

			if (copy_to_user(argp, &onoff_flag, sizeof(onoff_flag))) {
				return -EFAULT;
			}
			break;
		case MOTION_IOCTL_ACCEL_COMPASS_SLEEP_MODE:
			//lprintk(".............MOTION_IOCTL_ACCEL_COMPASS_SLEEP_MODE................\n");
			motion_sensor_power_off();

			//twl4030_i2c_write_u8(0x13, 0x00,0x1b );
			//msleep(100);
			break;
		case MOTION_IOCTL_ACCEL_COMPASS_SLEEP_WAKE_UP:
			//lprintk(".............MOTION_IOCTL_ACCEL_COMPASS_SLEEP_WAKE_UP................\n");
			motion_sensor_power_on();
			break;
		case MOTION_IOCTL_MPU3050_I2C_READ:
			if (copy_from_user(&rwbuf, argp, sizeof(rwbuf))) {
				//lprintk("FAIL!!!!!!copy_from_user.................MOTION_IOCTL_MPU3050_I2C_READ");
				return -EFAULT;
			}
			//lprintk("MOTION_IOCTL_MPU3050_I2C_READ addr : 0x%x\n",rwbuf[0]);

			write_lock(&getbuflock);
			memcpy(&accelrwbuf[0], rwbuf, sizeof(rwbuf));
			write_unlock(&getbuflock);

			if (rwbuf[1] < 1) {
#if DEBUG
				lprintk("EINVAL ERROR......I2C SLAVE MOTION_IOCTL_I2C_READ : rwbuf[1] < 1...\n");
#endif
				return -EINVAL;
			}

			//if(gyro_sleep_mode)
			//{
			//	return -EINVAL;
			//}

			if (rwbuf[0] == GYRO_I2C_SLAVE_ADDR) {

				//lprintk("############ (_0_)############ rwbuf[2]: %d(%x) / rwbuf[3]: %d(%x)/ rwbuf[1] : %d(%x)\n",rwbuf[2],rwbuf[2], rwbuf[3],rwbuf[3], rwbuf[1], rwbuf[1]);

				//jongik2.kim 20100910 i2c_fix [start]
				if (i2c_busy_flag == 0) {
					i2c_busy_flag = 1;
					NvGyroAccelI2CGetRegs(star_motion_dev->hOdmGyroAccel, rwbuf[2] ,&rwbuf[3] , rwbuf[1]);
					i2c_busy_flag = 0;
				}
				//jongik2.kim 20100910 i2c_fix [end]
				if (ret < 0) {
#if DEBUG
					lprintk("MOTION_IOCTL_I2C_READ : GYRO_I2C_SLAVE_ADDR Address ERROR[%d]\n",rwbuf[0]);
#endif
					return -EINVAL;
				}

				if (copy_to_user(argp, &rwbuf, sizeof(rwbuf))) {
#if DEBUG
					lprintk("EINVAL ERROR.### GYRO ### I2C SLAVE MOTION_IOCTL_I2C_READ : rwbuf[1] < 1...\n");
#endif
					return -EFAULT;
				}
			} else if (accelrwbuf[0] == 0x0F) {
				//lprintk("#### (_0_) #### accelrwbuf[2]: %d(%x) / accelrwbuf[3]: %d(%x)/ accelrwbuf[1] : %d(%x)\n",accelrwbuf[2],accelrwbuf[2], accelrwbuf[3],accelrwbuf[3], accelrwbuf[1], accelrwbuf[1]);
				//lprintk("######################## accel get(read) ##########################\n");
				if ((!accelrwbuf)) {
#if DEBUG
					lprintk("### EEROR #### accelrwbuf is NULL pointer \n");
#endif
					return -1;
				} else {
					NvAccelerometerI2CGetRegsPassThrough (accelrwbuf[2] ,&accelrwbuf[3] , accelrwbuf[1]);
				}

				if (ret < 0) {
#if DEBUG
					lprintk("MOTION_IOCTL_I2C_READ : ACCEL_I2C_SLAVE_ADDR Address ERROR[%d]\n",accelrwbuf[0]);
#endif
					return -EINVAL;
				}

				if (copy_to_user(argp, &accelrwbuf, sizeof(accelrwbuf))) {
#if DEBUG
					lprintk("EINVAL ERROR  ### ACCEL ## I2C SLAVE MOTION_IOCTL_I2C_READ : rwbuf[1] < 1...\n");
#endif
					return -EFAULT;
				}
			}
			/* wkkim add to read compass */
			else if (rwbuf[0] == 0x0E) {
#if 1
				compassI2CGetRegs(rwbuf[2], &rwbuf[3], rwbuf[1]);
#if DEBUG
				lprintk("### COMPASS ### I2C SLAVE MOTION_IOCTL_I2C_READ rwbuf[1]:%d, rwbuf[2]:%d, rwbuf[3]:%d\n", rwbuf[1], rwbuf[2], rwbuf[3]);
#endif
				if (copy_to_user(argp, &rwbuf, sizeof(rwbuf))) {
#if DEBUG
					lprintk("EINVAL ERROR.### COMPASS ### I2C SLAVE MOTION_IOCTL_I2C_READ : rwbuf[1] < 1...\n");
#endif
					return -EFAULT;
				}
#endif
			} else {
#if DEBUG
				lprintk("......I2C SLAVE ADDRESS ERROR!!!...[0x%x]...\n",buf[0]);
#endif
				return -EINVAL;
			}
			break;
		case MOTION_IOCTL_MPU3050_I2C_WRITE:
			//lprintk(".............MOTION_IOCTL_MPU3050_I2C_WRITE................\n");
			if (copy_from_user(&rwbuf, argp, sizeof(rwbuf))) {
#if DEBUG
				lprintk("EINVAL ERROR.....copy_from_user.I2C SLAVE MOTION_IOCTL_I2C_WRITE \n");
#endif
				return -EFAULT;
			}
			/*
				rwbuf[0] = slave_addr;  // slave addr - GYRO(0x68-MPU)
				rwbuf[1] = 2;                   // number of bytes to write +1
				rwbuf[2] = reg;               // register address
				rwbuf[3] = value;          // register value
				*/
			if (rwbuf[1] < 2) {
#if DEBUG
				lprintk("MOTION_IOCTL_WRITE ..length ERROR!!![%d].....\n",rwbuf[1]);
#endif
				return -EINVAL;
			}

			//if(gyro_sleep_mode)
			//{
			//	return -EINVAL;
			//}

			if (rwbuf[0] == GYRO_I2C_SLAVE_ADDR) {
				//ret = mpu3050_i2c_write(&rwbuf[2],rwbuf[1]);
				NvGyroAccelI2CSetRegs(star_motion_dev->hOdmGyroAccel, rwbuf[2] ,&rwbuf[3] , rwbuf[1] - 1);
				if (ret < 0) {
#if DEBUG
					lprintk("MOTION_IOCTL_WRITE  : GYRO_I2C_SLAVE_ADDR Address ERROR[%d]\n", rwbuf[0]);
#endif
					return -EINVAL;
				}
			} else if (rwbuf[0] == 0x0F) {
				//ret = kxtf9_i2c_write(&rwbuf[2],rwbuf[1]);
				//lprintk("(_6_)rwbuf[2]: %d(%x) /  rwbuf[1] : %d(%x)\n",rwbuf[2],rwbuf[2], rwbuf[1], rwbuf[1]);
				//			lprintk("######################## accel set(write) ##########################\n");
				NvAccelerometerI2CSetRegsPassThrough(rwbuf[2] ,&rwbuf[3] , rwbuf[1]-1);
				//lprintk("(_7_) rwbuf[3]: %d(%x) \n", rwbuf[3],rwbuf[3]);
				if (ret < 0){
#if DEBUG
					lprintk("[KXTF9] MOTION_IOCTL_WRITE  : ACCEL_I2C_SLAVE_ADDR ERROR[%d]\n",rwbuf[0]);
#endif
					return -EINVAL;
				}
			}
			/* wkkim add to set compass */
			else if (rwbuf[0] == 0x0E) {
				compassI2CSetRegs(rwbuf[2], &rwbuf[3], rwbuf[1] - 1);
#if DEBUG
				lprintk("### COMPASS ### I2C SLAVE MOTION_IOCTL_I2C_WRITE rwbuf[1]:%d, rwbuf[2]:%d, rwbuf[3]:%d\n", rwbuf[1], rwbuf[2], rwbuf[3]);
#endif
				if (ret < 0) {
#if DEBUG
					lprintk("MOTION_IOCTL_WRITE  : GYRO_I2C_SLAVE_ADDR Address ERROR[%d]\n",rwbuf[0]);
#endif
					return -EINVAL;
				}
			} else {
#if DEBUG
				lprintk("......I2C SLAVE ADDRESS ERROR!!!...[0x%x]...\n",buf[0]);
#endif
				return -EINVAL;
			}
			break;
		case MOTION_IOCTL_ACCEL_INT_ENABLE_DISABLE:
			//lprintk(".............MOTION_IOCTL_ACCEL_INT_ENABLE_DISABLE................\n");
			if (copy_from_user(&buf, argp, sizeof(buf))) {
				return -EFAULT;
			}
			atomic_set(&bypass_flag, buf[0]);

			break;		
		case MOTION_IOCTL_ACCEL_INIT:
			{
				tegra_accel_hw_init();

			}
			break;				
		case MOTION_IOCTL_CALIBRATION_FINISHED:
			printk(".............MOTION_IOCTL_CALIBRATION_FINISHED................\n");
			if (copy_from_user(&buf, argp, sizeof(buf))) {
				return -EFAULT;
			}

			atomic_set(&cal_flag, buf[0]);
			atomic_set(&cal_result, buf[1]);
			if(sensor_sleep_st == 1)
				atomic_set(&suspend_flag, 1);
			break;
		case MOTION_IOCTL_COMPASS_INIT:
			{
				tegra_compass_hw_init();
			}
			break;					
		case MOTION_IOCTL_CHECK_SENSOR_I2C_ERROR:
			//printk("MOTION_IOCTL_CHECK_SENSOR_I2C_ERROR[%d] \n",reboot);
			if (copy_to_user(argp, &reboot, sizeof(reboot))) {
				return -EFAULT;
			}
			break;
		case MOTION_IOCTL_REBOOT_SENSORS:
			printk(".............MOTION_IOCTL_REBOOT_SENSORS................\n");
			lge_sensor_shoutdown_all();
			break;

		default:
			break;
	}

	return 0;

}

static struct file_operations  star_motion_fops = {
	.owner    = THIS_MODULE,
	.open     = star_motion_open,
	.release  = star_motion_release,
	.ioctl    = star_motion_ioctl,
};

static struct miscdevice  star_motion_misc_device = {
	.minor 	= MISC_DYNAMIC_MINOR,
	.name 	= STAR_MOTION_IOCTL_NAME,
	.fops	= &star_motion_fops,
};

static int star_accel_open(struct inode *inode, struct file *file)
{
	int err;

	err = nonseekable_open(inode, file);
	if (err < 0)
		return err;
	//file->private_data = kxtf9_misc_data;

	return 0;
}

static int star_accel_ioctl(struct inode *inode, struct file *file,
		unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	/*struct tegra_acc_device_data *accel =
	  (struct tegra_acc_device_data*)accel_dev;*/
#define CTRL_REG1		0x1B
#define CTRL_REG3		0x1D
#define PC1_OFF			0x00
	u8 ctrl[2] = { CTRL_REG1, PC1_OFF };
	int err;
	int tmp;
	int xyz[3] = { 0 };
	NvS32 x = 0, y = 0, z = 0;
	//struct kxtf9_data *tf9 = file->private_data;

	switch (cmd) {
		case KXTF9_IOCTL_GET_DELAY:
			//tmp = tf9->res_interval;
			if (copy_to_user(argp, &tmp, sizeof(tmp)))
				return -EFAULT;
			break;
		case KXTF9_IOCTL_SET_DELAY:
			if (copy_from_user(&tmp, argp, sizeof(tmp)))
				return -EFAULT;
			if (tmp < 0)
				return -EINVAL;
			//tf9->res_interval = max(tmp, tf9->pdata->min_interval);
			//err = kxtf9_update_odr(tf9, tf9->res_interval);
			if (err < 0)
				return err;
			ctrl[0] = CTRL_REG3;
			ctrl[1] = (ctrl[1] >> 1) | (ctrl[1] >> 3);
			break;
		case KXTF9_IOCTL_SET_ENABLE:
			if (copy_from_user(&tmp, argp, sizeof(tmp)))
				return -EFAULT;
			if (tmp < 0 || tmp > 1)
				return -EINVAL;

			if (tmp)
				;//kxtf9_enable(tf9);
			else
				;//kxtf9_disable(tf9);
			break;
		case KXTF9_IOCTL_GET_ENABLE:
			//tmp = atomic_read(&tf9->enabled);
			if (copy_to_user(argp, &tmp, sizeof(tmp)))
				return -EINVAL;
			break;
		case KXTF9_IOCTL_SET_TILT_ENABLE:
			if (copy_from_user(&tmp, argp, sizeof(tmp)))
				return -EFAULT;
			if (tmp < 0 || tmp > 1)
				return -EINVAL;
			break;
		case KXTF9_IOCTL_SET_WAKE_ENABLE:
			if (copy_from_user(&tmp, argp, sizeof(tmp)))
				return -EFAULT;
			if (tmp < 0 || tmp > 1)
				return -EINVAL;
			break;
		case KXTF9_IOCTL_SELF_TEST:
			if (copy_from_user(&tmp, argp, sizeof(tmp)))
				return -EFAULT;
			if (tmp < 0 || tmp > 1)
				return -EINVAL;
			ctrl[0] = 0x3A;
			if (tmp) {
				ctrl[1] = 0xCA;
			} else {
				ctrl[1] = 0x00;
			}
			break;
		case KXTF9_IOCTL_READ_ACCEL_XYZ:
			//atomic_set(&accel_x,buf[0]);
			//atomic_set(&accel_y,buf[1]);
			//atomic_set(&accel_z,buf[2]);
			//xyz[0]=atomic_read(&accel_x);
			//xyz[1]=atomic_read(&accel_y);
			//xyz[2]=atomic_read(&accel_z);
			xyz[0]= atomic_read(&accel_x);
			xyz[1]= atomic_read(&accel_y);
			xyz[2]=atomic_read(&accel_z);

			//lprintk("K: Gyro-accel.c ## ACCEL ## xyz[0]: %d ;	xyz[1]: %d; xyz[2]: %d \n",xyz[0], xyz[1], xyz[2]);
#if 1
			/*#######################
			  This code is modified by (x,x,z)/10
			  x=0,  y= - 2656,  z = 9375
			  Accelerometer range : 0 ~ 4096
#######################*/
			xyz[0] = xyz[0] / 10 + 2048;
			xyz[1] = xyz[1] / 10 + 2048;
			xyz[2] = xyz[2] / 10 + 2048;
#endif
			//err=kxtf9_get_acceleration_data(tf9, xyz);
			/*NvOdmAccelGetAcceleration(
			  accel_dev->hOdmAcr, &x, &y, &z);
			  xyz[0] = x;	xyz[1] = y; xyz[2] = z;*/
			/*DY*///lprintk("K: Gyro-accel.c ## ACCEL ## xyz[ %d: 0x%x ] ;	xyz[ %d: 0x%x ]; xyz[ %d: 0x%x ] \n",xyz[0], xyz[0], xyz[1], xyz[1], xyz[2], xyz[2]);

			if (err < 0)
				return err;

			if (copy_to_user(argp, xyz, sizeof(int)*3))
				return -EINVAL;

			return err;

			break;
		default:
			return -EINVAL;
	}

	return 0;
}


static struct file_operations  star_accel_fops = {
	.owner    = THIS_MODULE,
	.open     = star_accel_open,
	.ioctl    = star_accel_ioctl,
};

static struct miscdevice  star_accel_misc_device = {
	.minor 	= MISC_DYNAMIC_MINOR,
	.name 	= STAR_ACCEL_IOCTL_NAME,
	.fops   = &star_accel_fops,
};

#if 0  /*Star Feature*/

/*---------------------------------------------------------------------------
  accel.  driver
  ---------------------------------------------------------------------------*/
static int __init accel_i2c_probe(struct i2c_client *client)
{
	lprintk("----------accel_i2c_probe\n");
	kxtf9_reg_i2c_client(client);

	return 0;
}

static int  accel_i2c_remove(struct i2c_client *client)
{
	return 0;
}

#define accel_i2c_suspend	NULL
#define accel_i2c_resume		NULL

static const struct i2c_device_id accel_i2c_id[] = {
	{STAR_I2C_ACCEL_NAME, 0 },
	{ /* end of list */ },
};

static struct i2c_driver accel_i2c_driver = {
	.probe     =  accel_i2c_probe,
	.remove  =  accel_i2c_remove,
	.id_table = accel_i2c_id,
#ifdef CONFIG_PM
	.suspend = accel_i2c_suspend,
	.resume	 = accel_i2c_resume,
#endif
	.driver = {
		.name = STAR_I2C_ACCEL_NAME,
	},
};


/*---------------------------------------------------------------------------
  gyro driver
  ---------------------------------------------------------------------------*/
static int __init gyro_i2c_probe(struct i2c_client *client)
{
	lprintk("----------gyro_i2c_probe\n");
	mpu3050_reg_i2c_client(client);

	return 0;

}
static int  gyro_i2c_remove(struct i2c_client *client)
{
	return 0;
}


#define gyro_i2c_suspend		NULL
#define gyro_i2c_resume		NULL

static const struct i2c_device_id gyro_i2c_id[] = {
	{STAR_I2C_GYRO_NAME, 0 },
	{ /* end of list */ },
};

static struct i2c_driver gyro_i2c_driver = {
	.probe = gyro_i2c_probe,
	.remove =  gyro_i2c_remove,
	.id_table = gyro_i2c_id,
#ifdef CONFIG_PM
	.suspend = gyro_i2c_suspend,
	.resume	 = gyro_i2c_resume,
#endif
	.driver = {
		.name = STAR_I2C_GYRO_NAME,
	},
};
#endif

static unsigned short  mpu3050_i2c_through_pass_internal(unsigned char  enable)
{
	unsigned char val_shadow=0, dummy=0;
	unsigned char value = 0;
	unsigned char buf;
	int status = 0;

	status = NvGyroAccelI2CGetRegs(star_motion_dev->hOdmGyroAccel, MPU3050_GYRO_I2C_USER_CTRL ,&val_shadow , 1 );
	
	if (status == 0) {
		#if DEBUG
		lprintk("[MPU3050] MPU3050_GYRO_I2C_USER_CTRL. i2c ERROR: 0x%x................................\n",val_shadow);
		#endif
		return 1; //error
	}

	if ((val_shadow & IME_IF_ENA_POS) != (enable*IME_IF_ENA_POS))
	{
		//printk("Already the bypass mode = %d (On=1, Off=0).", enable);
		return 0; //success
	}

	/* Currently, 2nd I2C is reported to have a bug. 
	 So let me write down the s/w workaround
	 */
	if (enable == MPU3050_BYPASS_MODE_ON)
	{ // bypass ON,
		//disable I2C first
		val_shadow &= ~IME_IF_ENA_POS;
	}
	else
	{ // bypass OFF
		val_shadow |= IME_IF_ENA_POS;
	}

       status = NvGyroAccelI2CSetRegs(star_motion_dev->hOdmGyroAccel, MPU3050_GYRO_I2C_USER_CTRL, &val_shadow, 1);

	if (status == 0) {
		#if DEBUG
		lprintk("[MPU3050] MPU3050_GYRO_I2C_USER_CTRL. i2c ERROR: 0x%x................................\n",val_shadow);
		#endif
		return 1; //error
	}

	if (enable == MPU3050_BYPASS_MODE_ON)
	{ // bypass ON
#if DEBUG
		lprintk(D_SENSOR,"[MPU3050] bypass on\n");
#endif	
		// dummy read
		status = NvAccelerometerI2CGetRegsPassThrough (0x00 ,&dummy , 1);
		if (status == 0) {
			#if DEBUG
			lprintk("[KIONIX] KIONIX_ACCEL_CHIP_ID_REG. i2c ERROR: 0x%x................................\n",dummy);
			#endif
			return 1; //error
		}
	}

	return 0;

}


unsigned short mpu3050_i2c_through_pass(unsigned char enable)
{
	int i;
	unsigned short result;
	unsigned char val_shadow=0;
	int status = 0;

	for (i=0;i<3;i++)
	{
		result = mpu3050_i2c_through_pass_internal(enable);
		if (result == 0) //success
		{
			status = NvGyroAccelI2CGetRegs(star_motion_dev->hOdmGyroAccel, MPU3050_GYRO_I2C_USER_CTRL ,&val_shadow , 1 );
			if (status == 0) {
				#if DEBUG
				lprintk("[KIONIX] KIONIX_ACCEL_CHIP_ID_REG. i2c ERROR: 0x%x................................\n",dummy);
				#endif
				return 1; //error
			}
			if ((val_shadow & IME_IF_ENA_POS) == (enable*IME_IF_ENA_POS)) return 0;
		}
	}

	return result;	
}

void mpu3050_initialize(void)
{
	unsigned char buf[3] = {0,};
	unsigned char value = 0;
	int status = 0;

#if 0
	//  Read WHO AM I
	value = 0;
	//status = mpu3050_read_reg(mpu3050_i2c_client,MPU3050_GYRO_I2C_WHO_AM_I,&value);
	NvGyroAccelI2CGetRegs(star_motion_dev->hOdmGyroAccel, MPU3050_GYRO_I2C_WHO_AM_I ,&value , 1 );
	lprintk("[MPU3050] MPU3050_GYRO_I2C_WHO_AM_I : %x\n",value);

	// Read Product ID
	value = 0;
	//status = mpu3050_read_reg(mpu3050_i2c_client,MPU3050_GYRO_I2C_PRODUCT_ID,&value);
	NvGyroAccelI2CGetRegs(star_motion_dev->hOdmGyroAccel, MPU3050_GYRO_I2C_PRODUCT_ID ,&value , 1 );
	lprintk("[MPU3050] MPU3050_GYRO_I2C_PRODUCT_ID : %x\n",value);
	/*---------------------------------------------------------------------------------------*/
#endif

}

static NvS32 star_motion_resume(struct platform_device *pdev);
static NvS32 star_motion_suspend(struct platform_device *pdev);

#ifdef CONFIG_HAS_EARLYSUSPEND
static void gyro_early_suspend(struct early_suspend *es)
{
#if DEBUG
	lprintk(D_SENSOR, "--->> Gyro early suspend\n");
#endif

	//bypass
	mpu3050_i2c_through_pass(MPU3050_BYPASS_MODE_ON);

	//gyro sleep
	mpu3050_sleep_mode();

	//accel interrupt disable and accel sleep
	motion_sensor_power_off();

	star_motion_suspend(NULL);
	return;

}

static void gyro_late_resume(struct early_suspend *es)
{
#if DEBUG
	lprintk(D_SENSOR, "--->> Gyro early resume\n");
#endif
	star_motion_resume(NULL);
	//gyro_sleep_mode = 0;
	return;
}
#endif



/*---------------------------------------------------------------------------
  platform device
  ---------------------------------------------------------------------------*/
static int __init star_motion_probe(struct platform_device *pdev)
{
	int err = 0;
	unsigned char value = 0;
	struct device *dev = &pdev->dev;
	struct star_motion_device *gyroscope_accel = NULL;
	struct input_dev *input_dev = NULL;

#if DEBUG
	lprintk("[MPU3050] ## [%s:%d]\n",__FUNCTION__, __LINE__);
#endif

	gyroscope_accel = kzalloc(sizeof(*gyroscope_accel), GFP_KERNEL);
	star_motion_dev = gyroscope_accel;

#if DEBUG
	lprintk(KERN_INFO"%s: probe start\n", __func__);
#endif
	/*---------------------------------------------------------------------------
	  register i2c driver
	  ---------------------------------------------------------------------------*/
#if 0  /*Star Feature*/
	err = i2c_add_driver(&gyro_i2c_driver);
	if(err < 0){
		lprintk("************* LGE: gyro_i2c_test_client fail\n");
		goto err_i2c_add_driver;
	}

	err = i2c_add_driver(&accel_i2c_driver);
	if(err < 0){
		lprintk("************* LGE: accel_i2c_test_client fail\n");
		goto err_i2c_add_driver;
	}
#endif


	/*---------------------------------------------------------------------------
	  register misc device
	  ---------------------------------------------------------------------------*/
	err = misc_register(&star_motion_misc_device);
	if (err) {
#if DEBUG
		lprintk(KERN_ERR"star_motion_misc_device register failed\n");
#endif
		goto exit_misc_device_register_failed;
	}

	err = misc_register(&star_accel_misc_device);
	if (err) {
#if DEBUG
		lprintk(KERN_ERR"star_motion_misc_device register failed\n");
#endif
		goto exit_misc_device_register_failed;
	}

	/*---------------------------------------------------------------------------
	  register input device
	  ---------------------------------------------------------------------------*/
	star_motion_dev->input_dev = input_allocate_device();
	if (star_motion_dev->input_dev == NULL) {
#if DEBUG
		lprintk(KERN_ERR"star_motion_sesnor_probe: input_allocate_device (1) failed\n");
#endif
		goto err_input_allocate1;
	}

	star_motion_dev->input_dev->name = STAR_MOTION_INPUT_NAME;

#if 1
	set_bit(EV_SYN,star_motion_dev->input_dev->evbit);
	set_bit(EV_REL,star_motion_dev->input_dev->evbit);


	set_bit(REL_Z,star_motion_dev->input_dev->relbit);  	// gyro_x
	set_bit(REL_MISC,star_motion_dev->input_dev->relbit);  // gyro_y
	set_bit(REL_MAX,star_motion_dev->input_dev->relbit);  	// gyro_z
#endif

	platform_set_drvdata(pdev, gyroscope_accel);
	set_bit(EV_ABS,star_motion_dev->input_dev->evbit);
	input_set_abs_params(star_motion_dev->input_dev, ABS_X, -2000, 2000, 0, 0); /* x-axis acceleration */
	input_set_abs_params(star_motion_dev->input_dev, ABS_Y, -2000, 2000, 0, 0); /* y-axis acceleration */
	input_set_abs_params(star_motion_dev->input_dev, ABS_Z, -2000, 2000, 0, 0); /* z-axis acceleration */
	input_set_abs_params(star_motion_dev->input_dev, ABS_GAS, 0, 0, 0, 0);
	input_set_abs_params(star_motion_dev->input_dev, ABS_HAT1X, 0, 0, 0, 0);
	input_set_abs_params(star_motion_dev->input_dev, ABS_HAT1Y, 0, 0, 0, 0);
	//input_set_abs_params(star_motion_dev->input_dev, ABS_HAT2X, 0, 0, 0, 0);
	//input_set_abs_params(star_motion_dev->input_dev, ABS_HAT2Y, 0, 0, 0, 0);
	//input_set_abs_params(star_motion_dev->input_dev, ABS_HAT3X, 0, 0, 0, 0);
	//input_set_abs_params(star_motion_dev->input_dev, ABS_HAT3Y, 0, 0, 0, 0);
	//input_set_abs_params(star_motion_dev->input_dev, ABS_TILT_X, 0, 0, 0, 0);
	//input_set_abs_params(star_motion_dev->input_dev, ABS_TILT_Y, 0, 0, 0, 0);
	//input_set_abs_params(star_motion_dev->input_dev, ABS_TOOL_WIDTH, 0, 0, 0, 0);
	//input_set_abs_params(star_motion_dev->input_dev, ABS_VOLUME, 0, 0, 0, 0);
	//input_set_abs_params(star_motion_dev->input_dev, ABS_MISC, 0, 0, 0, 0);
	
	/* x-axis of raw magnetic vector */
	input_set_abs_params(star_motion_dev->input_dev, ABS_HAT0X, -3000, 3000, 0, 0);
	/* y-axis of raw magnetic vector */
	input_set_abs_params(star_motion_dev->input_dev, ABS_HAT0Y, -3000, 3000, 0, 0);
	/* z-axis of raw magnetic vector */
	input_set_abs_params(star_motion_dev->input_dev, ABS_BRAKE, -3000, 3000, 0, 0);
	/* status of acceleration sensor */

#if 0

	input_set_abs_params(star_motion_dev->input_dev, ABS_X, -2000, 2000, 0, 0); /* x-axis acceleration */
	input_set_abs_params(star_motion_dev->input_dev, ABS_Y, -2000, 2000, 0, 0); /* y-axis acceleration */
	input_set_abs_params(star_motion_dev->input_dev, ABS_Z, -2000, 2000, 0, 0); /* z-axis acceleration */

	//set_bit(ABS_X,star_motion_dev->input_dev->relbit);  // TAP - Type
	//set_bit(ABS_Y,star_motion_dev->input_dev->relbit);  // TAP - Direction
	//set_bit(ABS_Z,star_motion_dev->input_dev->relbit);  // TILT - Roll

	input_set_abs_params(compass_dev->input_dev, ABS_THROTTE, 0, 360, 0, 0);/* pitch */
	input_set_abs_params(compass_dev->input_dev, ABS_GAS, 	-180, 180, 0, 0); /* roll */
	input_set_abs_params(compass_dev->input_dev, ABS_HAT1X, 	-90, 90, 0, 0);	/* status of magnetic sensor */
	input_set_abs_params(compass_dev->input_dev, ABS_HAT1Y, 	0, 5, 0, 0); 	/* x-axis acceleration */
	input_set_abs_params(compass_dev->input_dev, ABS_HAT2X, 	-2000, 2000, 0, 0); 	/* y-axis acceleration */
	input_set_abs_params(compass_dev->input_dev, ABS_HAT2Y, 	-2000, 2000, 0, 0); 	/* z-axis acceleration */
	input_set_abs_params(compass_dev->input_dev, ABS_HAT3X, 	-2000, 2000, 0, 0); 	/* x-axis of raw magnetic vector */
	input_set_abs_params(compass_dev->input_dev, ABS_HAT3Y, 	-3000, 3000, 0, 0); 	/* y-axis of raw magnetic vector */
#endif
	set_bit(REL_X, star_motion_dev->input_dev->relbit);  // TAP - Type
	set_bit(REL_Y, star_motion_dev->input_dev->relbit);  // TAP - Direction
	set_bit(REL_RX, star_motion_dev->input_dev->relbit);  // TILT - Yaw
	set_bit(REL_RY, star_motion_dev->input_dev->relbit);  // TILT - Pitch
	set_bit(REL_RZ, star_motion_dev->input_dev->relbit);  // TILT - Roll
	//set_bit(REL_RX,star_motion_dev->input_dev->relbit);  // TILT - Roll
	//set_bit(REL_RY,star_motion_dev->input_dev->relbit);  // TILT - PITCH
	//set_bit(REL_RZ,star_motion_dev->input_dev->relbit);  // TILT - Yaw
	set_bit(REL_HWHEEL, star_motion_dev->input_dev->relbit); // SHAKE
	set_bit(REL_DIAL, star_motion_dev->input_dev->relbit);   // SNAP - Direction
	set_bit(REL_WHEEL, star_motion_dev->input_dev->relbit);  // FLIP

//LGE_CHANGE_S [dongjin73.kim@lge.com] 2011-03-04, [LGE_AP20] Virtual sensor supports
	set_bit(STAR_GRAVITY_X, star_motion_dev->input_dev->absbit);		// Gravity
	set_bit(STAR_GRAVITY_Y, star_motion_dev->input_dev->absbit);
	set_bit(STAR_GRAVITY_Z, star_motion_dev->input_dev->absbit);
	set_bit(STAR_LINEARACCEL_X, star_motion_dev->input_dev->absbit);	// Linear Accelerometer
	set_bit(STAR_LINEARACCEL_Y, star_motion_dev->input_dev->absbit);
	set_bit(STAR_LINEARACCEL_Z, star_motion_dev->input_dev->absbit);
	set_bit(STAR_ROTVECTOR_X, star_motion_dev->input_dev->absbit);		// Rotation Vector
	set_bit(STAR_ROTVECTOR_Y, star_motion_dev->input_dev->absbit);
	set_bit(STAR_ROTVECTOR_Z, star_motion_dev->input_dev->absbit);
//LGE_CHANGE_E [dongjin73.kim@lge.com] 2011-03-04, [LGE_AP20] Virtual sensor supports

	err = input_register_device(star_motion_dev->input_dev);
	if (err) {
#if DEBUG
		lprintk(KERN_ERR"star_motion_sesnor_probe: input_allocate_device (1) failed \n");
#endif
		goto err_input_allocate1;
	}

	atomic_set(&cal_result, 2); //0: pass 1:fail 2:initial state

#if 0 /*MUST DO IT*/
	atomic_set(&accel_flag, 1);
	atomic_set(&compass_flag, 1);
	atomic_set(&shake_flag, 1);
	atomic_set(&tilt_flag, 1);
	atomic_set(&tap_flag, 1);
	atomic_set(&snap_flag, 1);
	atomic_set(&flip_flag, 1);
	atomic_set(&gyro_flag, 1);
	atomic_set(&accel_delay, DEFAULT_MOTION_POLLING_TIME);
	atomic_set(&compass_delay, DEFAULT_MOTION_POLLING_TIME);
	atomic_set(&tilt_delay, DEFAULT_MOTION_POLLING_TIME);
#endif

#if 1
	atomic_set(&accel_x,0);
	atomic_set(&accel_y,10240);
	atomic_set(&accel_z,0);
#endif
	/*---------------------------------------------------------------------------
	  init. sysfs
	  ---------------------------------------------------------------------------*/
	if ((err = sysfs_create_group(&dev->kobj, &star_motion_group))) {
#if DEBUG
		lprintk("[motion_sensor] sysfs_create_group FAIL \n");
#endif
		goto err_sysfs_create;
	}

	/*---------------------------------------------------------------------------
	  INIT_WORK
	  ---------------------------------------------------------------------------*/
#if 1
	INIT_WORK(&star_motion_dev->accel_work, motion_accel_work_func);
	INIT_WORK(&star_motion_dev->tilt_work, motion_tilt_work_func);
	INIT_WORK(&star_motion_dev->gyro_work, motion_gyro_work_func);
	INIT_WORK(&star_motion_dev->compass_work, motion_compass_work_func);
	INIT_WORK(&star_motion_dev->composite_work, motion_composite_work_func);

	/*---------------------------------------------------------------------------
	  init. workqueue
	  ---------------------------------------------------------------------------*/
	star_motion_dev->tilt_wq = create_singlethread_workqueue("motion_tilt_wq");
	if (!star_motion_dev->tilt_wq) {
#if DEBUG
		lprintk("[motion_sensor] couldn't create tilt work queue\n");
#endif
		goto err_motion_tilt_wq;
	}

	star_motion_dev->accel_wq = create_singlethread_workqueue("motion_accel_wq");
	if (!star_motion_dev->accel_wq) {
#if DEBUG
		lprintk("[motion_sensor] couldn't create accel work queue\n");
#endif
		goto err_motion_accel_wq;
	}

	star_motion_dev->gyro_wq = create_singlethread_workqueue("motion_gyro_wq");
	if (!star_motion_dev->gyro_wq) {
#if DEBUG
		lprintk("[motion_sensor] couldn't create gyro work queue\n");
#endif
		goto err_motion_gyro_wq;
	}

	star_motion_dev->compass_wq = create_singlethread_workqueue("motion_compass_wq");
	if (!star_motion_dev->compass_wq) {
#if DEBUG
		lprintk("[motion_sensor] couldn't create accel work queue\n");
#endif
		goto err_motion_compass_wq;
	}

	star_motion_dev->composite_wq = create_singlethread_workqueue("motion_composite_wq");
	if (!star_motion_dev->composite_wq) {
#if DEBUG
		lprintk("[motion_sensor] couldn't create composite work queue\n");
#endif
		goto err_motion_composite_wq;
	}

	/*---------------------------------------------------------------------------
	  init. timer
	  ---------------------------------------------------------------------------*/
	// TILT POLLING TIMER
	hrtimer_init(&star_motion_dev->timer[0], CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	star_motion_dev->timer[0].function = motion_accel_timer_func;

	hrtimer_init(&star_motion_dev->timer[1], CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	star_motion_dev->timer[1].function = motion_tilt_timer_func;

	hrtimer_init(&star_motion_dev->timer[2], CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	star_motion_dev->timer[2].function = motion_gyro_timer_func;

	hrtimer_init(&star_motion_dev->timer[3], CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	star_motion_dev->timer[3].function = motion_compass_timer_func;

	hrtimer_init(&star_motion_dev->timer[4], CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	star_motion_dev->timer[4].function = motion_composite_timer_func;	
#endif
	/*---------------------------------------------------------------------------
	  power
	  ---------------------------------------------------------------------------*/
#if 0
#if defined(CONFIG_MACH_LGE_STAR_REV_C)
	star_gyro_vio_reg = regulator_get(&pdev->dev, "vaux2");
	if (star_gyro_vio_reg == NULL) {
		lprintk(KERN_ERR": Failed to get motion power resources !! \n");
		return -ENODEV;
	}
#endif

	star_motion_reg = regulator_get(&pdev->dev, "vmmc2");
	if (star_motion_reg == NULL) {
		lprintk(KERN_ERR": Failed to get motion power resources !! \n");
		return -ENODEV;
	}
#endif

#if DEBUG
	lprintk("[MPU3050] ## [%s:%d]\n",__FUNCTION__, __LINE__);
#endif

	err = open_def_odm_gyro_accel();
	if (!err) {
#if DEBUG
		lprintk("open_def_odm_gyro_accel\n");
#endif
		goto allocate_dev_fail;
	}
#if DEBUG
	lprintk("[MPU3050] ## [%s:%d]\n",__FUNCTION__, __LINE__);
#endif

	msleep(1);
	//	mdelay(50);
	//  Read WHO AM I
	//value = 0;
	//status = mpu3050_read_reg(mpu3050_i2c_client,MPU3050_GYRO_I2C_WHO_AM_I,&value);
	//NvGyroAccelI2CGetRegs(star_motion_dev->hOdmGyroAccel, MPU3050_GYRO_I2C_WHO_AM_I ,&value , 1 );
#if DEBUG
	lprintk("[MPU3050] MPU3050_GYRO_I2C_WHO_AM_I : %x\n",value);
#endif
	// Read Product ID
	//value = 0;
	//status = mpu3050_read_reg(mpu3050_i2c_client,MPU3050_GYRO_I2C_PRODUCT_ID,&value);
	//NvGyroAccelI2CGetRegs(star_motion_dev->hOdmGyroAccel, MPU3050_GYRO_I2C_PRODUCT_ID ,&value , 1 );
#if DEBUG
	lprintk("[MPU3050] MPU3050_GYRO_I2C_PRODUCT_ID : %x\n",value);
#endif
#ifdef CONFIG_HAS_EARLYSUSPEND // wkkim : temporary early suspend apply
	early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 2;
	early_suspend.suspend = gyro_early_suspend;
	early_suspend.resume = gyro_late_resume;
	register_early_suspend(&early_suspend);
#endif

#if 0
	err = open_def_odm_accl();
	if (!err) {
		lprintk("open_def_odm_gyro_accel\n");
		goto allocate_dev_fail;
	}
	lprintk("[MPU3050] ## [%s:%d]\n",__FUNCTION__, __LINE__);
#endif
	// mpu3050_initialize();

	//motion_sensor_power_on();
	//twl4030_i2c_write_u8(0x13, 0x00,0x1b );
	//msleep(100);

	return 0;
#if 0  /*Star Feature*/
err_i2c_add_driver:
	i2c_del_driver(&gyro_i2c_driver);
	i2c_del_driver(&accel_i2c_driver);
#endif
allocate_dev_fail:
	printk("##  sensor: allocated_device_failed\n");
	close_odm_gyro_accel();
err_input_allocate1:
	printk("##  sensor: input_device_failed\n");
	input_unregister_device(star_motion_dev->input_dev);
exit_misc_device_register_failed:
err_sysfs_create:
	printk("##  sensor: heaven motion misc_device_register_failed\n");
err_motion_accel_wq:
	printk("##  sensor: accel timer_failed\n");
	destroy_workqueue(star_motion_dev->accel_wq);
err_motion_tilt_wq:
	printk("##  sensor: tilt timer_failed\n");
	destroy_workqueue(star_motion_dev->tilt_wq);
err_motion_gyro_wq:
	printk("##  sensor: gyro timer_failed\n");
	destroy_workqueue(star_motion_dev->gyro_wq);
err_motion_compass_wq:
	printk("##  sensor: compass timer_failed\n");
	destroy_workqueue(star_motion_dev->compass_wq);
err_motion_composite_wq:
	printk("##  sensor: composite timer_failed\n");
	destroy_workqueue(star_motion_dev->composite_wq);	
	return err;

}

int lge_sensor_verify_gyro(void)
{
	unsigned char value = 0;
	NvGyroAccelI2CGetRegs(star_motion_dev->hOdmGyroAccel, MPU3050_GYRO_I2C_WHO_AM_I ,&value , 1 );
	printk("[%s] ### Gyro ## WHO_AM_I = 0x%x \n",MOD_TAG,value);

	if(GYRO_WHID != value)
		return SENSOR_ERROR;

	return SENSOR_OK;
}

int lge_sensor_reset_gyro(void)
{
	unsigned char value = 0;
	NvOdmResetI2C(star_motion_dev->hOdmGyroAccel);
	printk("[%s] ### Gyro ## Reset  \n",MOD_TAG);

	if(GYRO_WHID != value)
		return SENSOR_ERROR;

	return SENSOR_OK;
}

static int star_motion_remove(struct platform_device *pdev)
{
	//struct device *dev = &pdev->dev;
	input_unregister_device(star_motion_dev->input_dev);
#if 0  /*Star Feature*/
	i2c_del_driver(&gyro_i2c_driver);
	i2c_del_driver(&accel_i2c_driver);
#endif
	if (star_motion_dev->tilt_wq)
		destroy_workqueue(star_motion_dev->tilt_wq);

	return 0;
}

#if 0 /*Dont need the function about suspend and resume */
#define star_motion_suspend  NULL
#define star_motion_resume   NULL
#else
void mpu3050_sleep_mode(void)
{
	unsigned char value = 0;
	unsigned char buf[5] = {0,};

	//mpu3050_read_reg(mpu3050_i2c_client,MPU3050_GYRO_I2C_PWR_MGM,&value);
	NvGyroAccelI2CGetRegs(star_motion_dev->hOdmGyroAccel, MPU3050_GYRO_I2C_PWR_MGM ,&value , 1 );
#if DEBUG
	lprintk("--->> %s:%d PWR Reg : 0x%x\n",__func__,__LINE__,value);
#endif
	/*
	   Bit 6   SLEEP
	   Bit 5   STBY_XG
	   Bit 4   STBY_YG
	   Bit 3   STBY_ZG
	   */

	if (!(value & 0x40)) {
		value |= 0x40;

		buf[0] = MPU3050_GYRO_I2C_PWR_MGM;
		buf[1] = value;

		//mpu3050_write_reg(mpu3050_i2c_client,buf);
		NvGyroAccelI2CSetRegs(star_motion_dev->hOdmGyroAccel, buf[0], &buf[1], 1);
		msleep(5);

#if DEBUG
		lprintk("%s:%d PWR Reg : 0x%x\n", __func__, __LINE__, value);
#endif
	}

}

void mpu3050_sleep_wake_up(void)
{
	unsigned char value = 0;
	unsigned char buf[5] = {0,};

	//mpu3050_read_reg(mpu3050_i2c_client,MPU3050_GYRO_I2C_PWR_MGM,&value);
#if DEBUG
	lprintk("--->> %s:%d PWR Reg : 0x%x, jiffies : %u\n", __func__, __LINE__, value, jiffies);
#endif
	NvGyroAccelI2CGetRegs(star_motion_dev->hOdmGyroAccel, MPU3050_GYRO_I2C_PWR_MGM, &value, 1);

	if (value & 0x40) {
		value &= ~0x40;

		buf[0] = MPU3050_GYRO_I2C_PWR_MGM;
		buf[1] = value;

		//mpu3050_write_reg(mpu3050_i2c_client,buf);
		NvGyroAccelI2CSetRegs(star_motion_dev->hOdmGyroAccel, buf[0], &buf[1], 1);
	}
}

//jongik2.kim 20100910 i2c_fix [start]
NvBool star_get_i2c_busy()
{
	return i2c_busy_flag;
}

void star_set_i2c_busy()
{
	i2c_busy_flag = 1;
	return 0;
}

void star_unset_i2c_busy()
{
	i2c_busy_flag = 0;
	return 0;
}
//jongik2.kim 20100910 i2c_fix [end]

static NvS32 star_motion_resume(struct platform_device *pdev)
{
	atomic_set(&suspend_flag, 0);
	sensor_sleep_st = 0;
	return 0;
}

static NvS32 star_motion_suspend(struct platform_device *pdev)
{
	atomic_set(&suspend_flag, 1);
	sensor_sleep_st = 1;
	return 0;
}

#endif
static struct platform_driver star_motion_driver = {
	.probe    	= star_motion_probe,
	.remove   	= star_motion_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend  	= star_motion_suspend,
	.resume   	= star_motion_resume,
#endif
	.driver =  {
		.name = "tegra_gyro_accel",
		.owner = THIS_MODULE,
	},
};
static int __init star_motion_init(void)
{
	int err;

#if DEBUG
	lprintk("[MPU3050] ## [%s:%d]\n", __FUNCTION__, __LINE__);
#endif
	rwlock_init(&getbuflock);
	memset(&accelrwbuf[0], 0, sizeof(unsigned char) * 200);
	memset(&rwbuf[0], 0, sizeof(unsigned char) * 200);


	err = platform_driver_register(&star_motion_driver);

	return 0;
}
static void __exit star_motion_exit(void)
{
#if DEBUG
	lprintk(KERN_INFO "[MPU3050] lge star_motion_exit was unloaded!\nHave a nice day!\n");
#endif
	platform_driver_unregister(&star_motion_driver);
	return;
}

module_init(star_motion_init);
module_exit(star_motion_exit);

MODULE_AUTHOR("LG Electronics");
MODULE_DESCRIPTION("Gyro-accel Driver for Star");
MODULE_LICENSE("GPL");

