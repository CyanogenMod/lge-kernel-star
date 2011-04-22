/*
 * Copyright (C) 2009 Kionix, Inc.
 * Written by Chris Hudson <chudson@kionix.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */

#include <linux/err.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include "kxtf9.h"

//******************//
#include <mach/nvrm_linux.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/kthread.h>
#include <linux/device.h>
#include <linux/kernel.h>
//#include <linux/tegra_devices.h>
#include <nvodm_services.h>
#include <nvodm_gyro_accel_kxtf9.h>
#include <mach/lprintk.h>
#include "star_sensors.h"
//#include "star_motion.h"
//*****************//

#define DEBUG_ACCEL	0
#define DEBUG_ACCEL_DATA 0

#define NAME			"kxtf9"
#define G_MAX			8000
/* OUTPUT REGISTERS */
#define XOUT_L			0x06
#define XOUT_H			0x07
#define YOUT_L			0x08
#define YOUT_H			0x09
#define ZOUT_L			0x0A
#define ZOUT_H			0x0B

#define INT_SRC_REG1		0x15
#define INT_STATUS_REG		0x16
#define TILT_POS_CUR		0x10
#define INT_REL			0x1A
#define WHO_AM_I		0x0F
/* CONTROL REGISTERS */
#define DATA_CTRL		0x21
#define CTRL_REG1		0x1B
#define INT_CTRL1		0x1E
#define CTRL_REG3		0x1D
#define TILT_TIMER		0x28
#define WUF_TIMER		0x29
#define WUF_THRESH		0x5A
#define TDT_TIMER		0x2B
/* CONTROL REGISTER 1 BITS */
#define PC1_OFF			0x00
#define PC1_ON			0x80
/* INTERRUPT SOURCE 2 BITS */
#define TPS			0x01
#define TDTS0			0x04
#define TDTS1			0x08
/* INPUT_ABS CONSTANTS */
#define FUZZ			32
#define FLAT			32
/* RESUME STATE INDICES */
#define RES_DATA_CTRL		0
#define RES_CTRL_REG1		1
#define RES_INT_CTRL1		2
#define RES_TILT_TIMER		3
#define RES_CTRL_REG3		4
#define RES_WUF_TIMER		5
#define RES_WUF_THRESH		6
#define RES_TDT_TIMER		7
#define RES_TDT_H_THRESH	8
#define RES_TDT_L_THRESH	9
#define RES_TAP_TIMER		10
#define RES_TOTAL_TIMER		11
#define RES_LAT_TIMER		12
#define RES_WIN_TIMER		13
#define RESUME_ENTRIES		14

//******************//
//#define DRIVER_NAME 		"nvodm_accelerometer"
//#define READ_BUFFER_LENGTH	20

//typedef unsigned char	u8;

#define write_lock(lock)	_write_lock(lock)
#define read_lock(lock)		_read_lock(lock)
rwlock_t accelpassthroughlock;

extern atomic_t accel_init;
atomic_t accel_flag;
EXPORT_SYMBOL(accel_flag);
extern void sensors_wake_up_now(void);

atomic_t accel_delay;
EXPORT_SYMBOL(accel_delay);
atomic_t   accel_x, accel_y, accel_z;
EXPORT_SYMBOL(accel_x);
EXPORT_SYMBOL(accel_y);
EXPORT_SYMBOL(accel_z);

struct accelerometer_data {
	int x;
	int y;
	int z;
};

static struct accelerometer_data defaultaccel;
static struct accelerometer_data accelpassthrough ;

struct tegra_acc_device_data
{
	NvOdmAcrDeviceHandle	hOdmAcr;
	struct task_struct	*task;
	struct input_dev	*input_dev;
	NvU32			freq;
	NvBool			show_log;
	NvOdmAccelIntType	IntType;
	NvOdmAccelAxisType	IntMotionAxis;
	NvOdmAccelAxisType	IntTapAxis;

	struct accelerometer_data prev_data;
	struct accelerometer_data min_data;
	struct accelerometer_data max_data;
};

struct tegra_acc_device_data *accel_dev = NULL;

#if 0
static const char* parameter[] = {
	"log=",
	"frequency=",
	"forcemotion=",
	"forcetap=",
	"timetap=",
	"openclose=",
};

static enum {
	COMMAND_LOG = 0,
	COMMAND_FREQUENCY,
	COMMAND_FORCEMOTION,
	COMMAND_FORCETAP,
	COMMAND_TIMETAP,
	COMMAND_OPENCLOSE,
}accel_enum;
#endif

//******************//


/*
 * The following table lists the maximum appropriate poll interval for each
 * available output data rate.
 */
struct {
	unsigned int cutoff;
	u8 mask;
}

kxtf9_odr_table[] = {
	{ 3,	ODR800F},
	{ 5,	ODR400F},
	{ 10,	ODR200F},
	{ 20,	ODR100F},
	{ 40,	ODR50F},
	{ 80,	ODR25F},
	{ 0,	ODR12_5F},
};

struct kxtf9_data {
	struct i2c_client *client;
	struct input_dev *accel_input_dev;
	struct kxtf9_platform_data *pdata;
	struct mutex lock;
	struct delayed_work input_work;
	struct input_dev *input_dev;
	struct work_struct irq_work;

	int hw_initialized;
	atomic_t enabled;
	u8 resume[RESUME_ENTRIES];
	int res_interval;
	int irq;

#if 1 // from YJ  2010-06-14, Need change to last kionix driver version, currently temp. modification.
	u8 axis_map_x;
	u8 axis_map_y;
	u8 axis_map_z;

	u8 negate_x;
	u8 negate_y;
	u8 negate_z;
#endif

};

static struct kxtf9_data *kxtf9_misc_data;

int get_accel_flag(void)
{
	return atomic_read(&accel_flag);
}
EXPORT_SYMBOL(get_accel_flag);

static unsigned char accelrwbuf[200] = {0,};    /* i2c MAX data length */

/** Function to close the ODM device. This function will help in switching
 * between power modes
 */
void close_odm_accl(void)
{
	NvOdmAccelClose(accel_dev->hOdmAcr);
	accel_dev->hOdmAcr = 0;
}

/** Function to open the ODM device with a set of default values. The values
 * are hardcoded as of now. Each time the device is closed/open, previous
 * settings will be lost. This function will help in switching
 * between power modes
 */
int open_def_odm_accl(void)
{
	NvS32 err = -1;
	unsigned char buf;

	err = NvOdmAccelOpen(&(accel_dev->hOdmAcr));
	if (!err) {
		err = -ENODEV;
		pr_err("open_def_odm_accl: NvOdmAccelOpen failed\n");

		return err;
	}
#if 1
	err = NvOdmAccelSetIntEnable(accel_dev->hOdmAcr,
			NvOdmAccelInt_MotionThreshold, NvOdmAccelAxis_All, 0, NV_TRUE);
#endif

	if (!err) {
		pr_err("open_def_odm_accl: NvOdmAccelSetIntEnable failed\n");
		return err;
	}

	//
#if defined(CONFIG_MACH_STAR)
	NvOdmAccelSetSampleRate(accel_dev->hOdmAcr, 800);
#endif

	NvAccelerometerI2CGetRegs(accel_dev->hOdmAcr, 0x0F, &buf, 1);
	printk("### Accelerometer ## KXTF9 ## ======> WHO_AM_I = 0x%x \n", buf);

	return err;
}

/* Accel : jay.sim */
/*---------------------------------------------------------------------------
    motion_sensor_power_on/off
 ---------------------------------------------------------------------------*/
/* TODO : rearrange define statements */
#define ACCEL_CTRL_REG3     0x1D
#define ACCEL_CTRL_REG1     0x1B
#define ACCEL_PC1_ON        0x80
#define ACCEL_PC1_OFF           0x00  //stand-by-mode
#define AMI304_REG_CTRL1 0x1B
#define AMI304_REG_CTRL2 0x1C
#define AMI304_REG_CTRL3 0x1D

void motion_sensor_power_on(void)
{
	u8 accel_reg;

	/* Accelerometer power on */
	NvAccelerometerI2CGetRegsPassThrough(ACCEL_CTRL_REG3, &accel_reg, 1);

	/* TODO : change '1 << 7' to define statement */
	accel_reg |= 1 << 7;	/* SRST set */
	NvAccelerometerI2CSetRegsPassThrough(ACCEL_CTRL_REG3, &accel_reg, 1);   // sets SRST bit to reboot
	msleep(1);
//  mdelay(20);

	accel_reg = ACCEL_PC1_ON;
	NvAccelerometerI2CSetRegsPassThrough(ACCEL_CTRL_REG1, &accel_reg, 1);

#if DEBUG_ACCEL
	printk("[%s:%d] Accelerometer Sensor Power On\n", __FUNCTION__, __LINE__);
#endif
}
EXPORT_SYMBOL(motion_sensor_power_on);

void motion_sensor_power_off(void)
{
	u8 accel_reg;

	/* Accelerometer power off */
	accel_reg = ACCEL_PC1_OFF;
	NvAccelerometerI2CSetRegsPassThrough(ACCEL_CTRL_REG1, &accel_reg, 1);

#if DEBUG_ACCEL
	printk("[%s:%d] Accelerometer Sensor Power OFf\n", __FUNCTION__, __LINE__);
#endif

}
EXPORT_SYMBOL(motion_sensor_power_off);

// return value
// int *ret_xyz : count value, range : 0~4095

NvBool NvAccelerometerI2CSetRegsPassThrough(NvU8 offset, NvU8* value, NvU32 len)
{

	return NvAccelerometerI2CSetRegs(accel_dev->hOdmAcr, offset, value, len);
}

NvBool NvAccelerometerI2CGetRegsPassThrough(NvU8 offset, NvU8* value, NvU32 len)
{

	return NvAccelerometerI2CGetRegs(accel_dev->hOdmAcr, offset, value, len);
}

#if 1 // YJ, 2010-06-14
static int kxtf9_get_acceleration_data(struct kxtf9_data *tf9, int *xyz_cnt)
{

#ifndef KXTF9_I2C_XOUT_L
#define KXTF9_I2C_XOUT_L 0x06
#endif

//	if (accelpassthrough.x != NULL) {
	if (accelpassthrough.x != 0) {
		xyz_cnt[0] = accelpassthrough.x;
		xyz_cnt[1] = accelpassthrough.y;
		xyz_cnt[2] = accelpassthrough.z;
	} else {
		xyz_cnt[0] = defaultaccel.x;
		xyz_cnt[1] = defaultaccel.y;
		xyz_cnt[2] = defaultaccel.z;
	}

	return 0;
}

#else // YJ, 2010-06-14
static int kxtf9_get_acceleration_data(struct kxtf9_data *tf9, int *ret_xyz)
{
#ifndef KXTF9_I2C_XOUT_L
#define KXTF9_I2C_XOUT_L 0x06
#endif

	int err;

	int status, x_sign, y_sign, z_sign, sensitivity;
	char x_char;
	char ret[3] = {0, 0, 0};
	char xyz[6] = {0, 0, 0, 0, 0, 0};
	int x = 0, gx;
	int y = 0, gy;
	int z = 0, gz;
	char Res = 0;
	char G_range = 0;
	int range = 0;

	NvAccelerometerI2CGetRegs(accel_dev->hOdmAcr, 0x1B, &Res, 1);
	NvAccelerometerI2CGetRegs(accel_dev->hOdmAcr, 0x1B, &G_range, 1);
	G_range = G_range & 0x18;
	G_range = G_range >> 3;
	switch (G_range) {
		case 0:
			range = 2;
			break;
		case 1:
			range = 4;
			break;
		case 2:
			range = 8;
			break;
		default:
			break;
	}
	Res = Res & 0x40;
	switch(Res) {
		case 0x00:	//low-resolution state
			NvAccelerometerI2CGetRegs(accel_dev->hOdmAcr, KXTF9_I2C_XOUT_L, &xyz[0], 6);
			ret_xyz[0] = x = ((int)xyz[1]);
			x_sign = x >> 7;	//1 = negative; 0 = positive
			if (x_sign == 1){
				x = ((~(x) + 0x01) & 0x0FF);
				x = -(x);
			}
			ret_xyz[1] = y = ((int)xyz[3]);
			y_sign = y >> 7;	//1 = negative; 0 = positive
			if (y_sign == 1){
				y = ((~(y) + 0x01) & 0x0FF);	//2's complement
				y = -(y);
			}
			ret_xyz[2] = z = ((int)xyz[5]);
			z_sign = z >> 7;	//1 = negative; 0 = positive
			if (z_sign == 1){
				z = ((~(z) + 0x01) & 0x0FF);	//2's complement
				z = -(z);
			}
			sensitivity = (256)/(2*range);
			/* calculate milli-G's */
			gx = 1000 * (x) / sensitivity;
			gy = 1000 * (y) / sensitivity;
			gz = 1000 * (z) / sensitivity;
			break;
		case 0x40:	//high-resolution state
			NvAccelerometerI2CGetRegs(accel_dev->hOdmAcr, KXTF9_I2C_XOUT_L, &xyz[0], 6);
			x = ((int)xyz[0]) >> 4;
			ret_xyz[0] = x = x + (((int)xyz[1]) << 4);
			if(ret_xyz[0] < 2048) ret_xyz[0] += 2048;
			else ret_xyz[0] -= 2048;
			x_sign = x >> 11; 	//1 = negative; 0 = positive
			if (x_sign == 1){
				x = ((~(x) + 0x01) & 0x0FFF);	//2's complement
				x = -(x);
			}
			y = ((int)xyz[2]) >> 4;
			ret_xyz[1] = y = y + (((int)xyz[3]) << 4);
			if(ret_xyz[1] < 2048) ret_xyz[1] += 2048;
			else ret_xyz[1] -= 2048;
			y_sign = y >> 11; 	//1 = negative; 0 = positive
			if (y_sign == 1){
				y = ((~(y) + 0x01) & 0x0FFF);	//2's complement
				y = -(y);
			}
			z = ((int)xyz[4]) >> 4;
			ret_xyz[2] = z = z + (((int)xyz[5]) << 4);
			if(ret_xyz[2] < 2048) ret_xyz[2] += 2048;
			else ret_xyz[2] -= 2048;
			z_sign = z >> 11; 	//1 = negative; 0 = positive
			if (z_sign == 1){
				z = ((~(z) + 0x01) & 0x0FFF);	//2's complement
				z = -(z);
			}
			sensitivity = (4096)/(2*range);
			/* calculate milli-G's */
			gx = 1000 * (x) / sensitivity;
			gy = 1000 * (y) / sensitivity;
			gz = 1000 * (z) / sensitivity;
			break;
	}
#if DEBUG_ACCEL_DATA
	printk("[%s:%d] XYZ = (%d, %d mg), (%d, %d mg), (%d, %d mg) \n",
			__FUNCTION__,__LINE__, ret_xyz[0], gx, ret_xyz[1], gy, ret_xyz[2], gz);
#endif
	return 0;
}
#endif // end of YJ, 2010-06-14

int kxtf9_update_odr(struct kxtf9_data *tf9, int poll_interval)
{
	int err = -1;
	int i;
	u8 config;

	/* Convert the poll interval into an output data rate configuration
	 *  that is as low as possible.  The ordering of these checks must be
	 *  maintained due to the cascading cut off values - poll intervals are
	 *  checked from shortest to longest.  At each check, if the next slower
	 *  ODR cannot support the current poll interval, we stop searching */
	for (i = 0; i < ARRAY_SIZE(kxtf9_odr_table); i++) {
		config = kxtf9_odr_table[i].mask;
		if (poll_interval < kxtf9_odr_table[i].cutoff)
			break;
	}

	if (atomic_read(&tf9->enabled)) {
		//err = kxtf9_i2c_write(tf9, DATA_CTRL, &config, 1);
		err = NvOdmAccelerometerSetParameter(accel_dev->hOdmAcr, DATA_CTRL, (int)config);
		if (err < 0)
			return err;
		/*
		 *  Latch on input_dev - indicates that kxtf9_input_init passed
		 *  and this workqueue is available
		 */
		if (tf9->input_dev) {
			cancel_delayed_work_sync(&tf9->input_work);
			schedule_delayed_work(&tf9->input_work,
					msecs_to_jiffies(poll_interval));
		}
	}
	tf9->resume[RES_DATA_CTRL] = config;

	return 0;
}

static int kxtf9_hw_init(struct kxtf9_data *tf9)
{
	int err = -1;
	u8 buf[7];

	//buf[0] = PC1_OFF;
	NvOdmAccelerometerSetParameter(accel_dev->hOdmAcr, CTRL_REG1, (int)PC1_OFF);

	if (err < 0)
		return err;
	NvOdmAccelerometerSetParameter(accel_dev->hOdmAcr, DATA_CTRL, (int)tf9->resume[RES_DATA_CTRL]);

	if (err < 0)
		return err;
	NvOdmAccelerometerSetParameter(accel_dev->hOdmAcr, CTRL_REG3, (int)tf9->resume[RES_CTRL_REG3]);

	if (err < 0)
		return err;
	NvOdmAccelerometerSetParameter(accel_dev->hOdmAcr, TILT_TIMER, (int)tf9->resume[RES_TILT_TIMER]);

	if (err < 0)
		return err;
	NvOdmAccelerometerSetParameter(accel_dev->hOdmAcr, WUF_TIMER, (int)tf9->resume[RES_WUF_TIMER]);

	if (err < 0)
		return err;
	NvOdmAccelerometerSetParameter(accel_dev->hOdmAcr, WUF_THRESH, (int)tf9->resume[RES_WUF_THRESH]);

	if (err < 0)
		return err;
	buf[0] = tf9->resume[RES_TDT_TIMER];
	buf[1] = tf9->resume[RES_TDT_H_THRESH];
	buf[2] = tf9->resume[RES_TDT_L_THRESH];
	buf[3] = tf9->resume[RES_TAP_TIMER];
	buf[4] = tf9->resume[RES_TOTAL_TIMER];
	buf[5] = tf9->resume[RES_LAT_TIMER];
	buf[6] = tf9->resume[RES_WIN_TIMER];
//	NvOdmAccelerometerSetParameter(accel_dev->hOdmAcr, TDT_TIMER, buf);
	NvOdmAccelerometerSetParameter(accel_dev->hOdmAcr, TDT_TIMER, (int)buf[0]);
	if (err < 0)
		return err;

	NvOdmAccelerometerSetParameter(accel_dev->hOdmAcr, INT_CTRL1, tf9->resume[RES_INT_CTRL1]);
	if (err < 0)
		return err;

	buf[0] = (tf9->resume[RES_CTRL_REG1] | PC1_ON);

//	NvOdmAccelerometerSetParameter(accel_dev->hOdmAcr, CTRL_REG1, buf);
	NvOdmAccelerometerSetParameter(accel_dev->hOdmAcr, CTRL_REG1, buf[0]);
	if (err < 0)
		return err;
	tf9->resume[RES_CTRL_REG1] = buf[0];
	tf9->hw_initialized = 1;

	return 0;
}

static u8 kxtf9_resolve_dir(struct kxtf9_data *tf9, u8 dir)
{
	switch (dir) {
		case 0x20:	/* -X */
			if (tf9->pdata->negate_x)
				dir = 0x10;
			if (tf9->pdata->axis_map_y == 0)
				dir >>= 2;
			if (tf9->pdata->axis_map_z == 0)
				dir >>= 4;
			break;
		case 0x10:	/* +X */
			if (tf9->pdata->negate_x)
				dir = 0x20;
			if (tf9->pdata->axis_map_y == 0)
				dir >>= 2;
			if (tf9->pdata->axis_map_z == 0)
				dir >>= 4;
			break;
		case 0x08:	/* -Y */
			if (tf9->pdata->negate_y)
				dir = 0x04;
			if (tf9->pdata->axis_map_x == 1)
				dir <<= 2;
			if (tf9->pdata->axis_map_z == 1)
				dir >>= 2;
			break;
		case 0x04:	/* +Y */
			if (tf9->pdata->negate_y)
				dir = 0x08;
			if (tf9->pdata->axis_map_x == 1)
				dir <<= 2;
			if (tf9->pdata->axis_map_z == 1)
				dir >>= 2;
			break;
		case 0x02:	/* -Z */
			if (tf9->pdata->negate_z)
				dir = 0x01;
			if (tf9->pdata->axis_map_x == 2)
				dir <<= 4;
			if (tf9->pdata->axis_map_y == 2)
				dir <<= 2;
			break;
		case 0x01:	/* +Z */
			if (tf9->pdata->negate_z)
				dir = 0x02;
			if (tf9->pdata->axis_map_x == 2)
				dir <<= 4;
			if (tf9->pdata->axis_map_y == 2)
				dir <<= 2;
			break;
		default:
			return -EINVAL;
	}

	return dir;
}

static void kxtf9_device_power_off(struct kxtf9_data *tf9)
{
	int err;

	err = NvOdmAccelerometerSetParameter(accel_dev->hOdmAcr, CTRL_REG1, PC1_OFF);
	if (err < 0)
		printk("soft power off failed\n");
	disable_irq(tf9->irq);
	if (tf9->pdata->power_off)
		tf9->pdata->power_off();
	tf9->hw_initialized = 0;
}

static int kxtf9_device_power_on(struct kxtf9_data *tf9)
{
	int err;

	if (tf9->pdata->power_on) {
		err = tf9->pdata->power_on();
		if (err < 0)
			return err;
	}
	enable_irq(tf9->irq);
	if (!tf9->hw_initialized) {
		msleep(1);
		//		mdelay(100);
		err = kxtf9_hw_init(tf9);
		if (err < 0) {
			kxtf9_device_power_off(tf9);
			return err;
		}
	}

	return 0;
}

static int kxtf9_enable(struct kxtf9_data *tf9)
{
	int err;
	int int_status = 0;
	u8 buf;

	if (!atomic_cmpxchg(&tf9->enabled, 0, 1)) {
		err = kxtf9_device_power_on(tf9);
		//err = kxtf9_i2c_read(tf9, INT_REL, &buf, 1);
		err = NvOdmAccelerometerGetParameter(accel_dev->hOdmAcr, INT_REL , (NvU32 *)&buf);
		if (err < 0) {
			printk("[%s:%d] error clearing interrupt: %d\n", __FUNCTION__,  __LINE__, err);
			/*dev_err(&tf9->client->dev,
			  "error clearing interrupt: %d\n", err);*/
			atomic_set(&tf9->enabled, 0);
			return err;
		}
		if ((tf9->resume[RES_CTRL_REG1] & TPE) > 0) {
			//err = kxtf9_i2c_read(tf9, TILT_POS_CUR, &buf, 1);
			err = NvOdmAccelerometerGetParameter(accel_dev->hOdmAcr, TILT_POS_CUR , (NvU32 *)&buf );
			if (err < 0) {
				printk("[%s:%d] read err current tilt\n", __FUNCTION__, __LINE__);
				/*dev_err(&tf9->client->dev,
				  "read err current tilt\n");*/
				int_status |= kxtf9_resolve_dir(tf9, buf);
				input_report_abs(tf9->input_dev, ABS_MISC, int_status);
				input_sync(tf9->input_dev);
			}
		}
		schedule_delayed_work(&tf9->input_work,
				msecs_to_jiffies(tf9->res_interval));
	}

	return 0;
}

static int kxtf9_disable(struct kxtf9_data *tf9)
{
	if (atomic_cmpxchg(&tf9->enabled, 1, 0)) {
		cancel_delayed_work_sync(&tf9->input_work);
		kxtf9_device_power_off(tf9);
	}

	return 0;
}

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
			 *       (struct tegra_acc_device_data*)accel_dev;*/
#define CTRL_REG1       0x1B
#define CTRL_REG3       0x1D
#define PC1_OFF         0x00
		u8 ctrl[2] = { CTRL_REG1, PC1_OFF };
		int err = 0;
		int tmp;
		int xyz[3] = { 0 };
		//struct kxtf9_data *tf9 = file->private_data;

#if DEBUG_ACCEL
	printk("%s is called\n", __func__);
#endif

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
				//if (err < 0)
				//	return err;
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
				xyz[0] = atomic_read(&accel_x);
				xyz[1] = atomic_read(&accel_y);
				xyz[2] = atomic_read(&accel_z);

				//printk("K: Gyro-accel.c ## ACCEL ## xyz[0]: %d ; xyz[1]: %d; xyz[2]: %d \n",xyz[0], xyz[1], xyz[2]);
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
				  xyz[0] = x;   xyz[1] = y; xyz[2] = z;*/
				/*DY*///printk("K: Gyro-accel.c ## ACCEL ## xyz[ %d: 0x%x ] ;  xyz[ %d: 0x%x ]; xyz[ %d: 0x%x ] \n",xyz[0], xyz[0], xyz[1], xyz[1], xyz[2], xyz[2]);

				//if (err < 0)
				//	return err;

				if (copy_to_user(argp, xyz, sizeof(int)*3))
					return -EINVAL;

				return err;

				break;

			/* jay.sim : TODO */

//			case MOTION_IOCTL_MPU3050_I2C_READ:
			case KXTF9_IOCTL_I2C_READ:

				printk("%s KXTF9_IOCTL_I2C_READ:accelrwbuf %d %d %d %d\n", __func__, accelrwbuf[0], accelrwbuf[1], accelrwbuf[2], accelrwbuf[3]);

				if (copy_from_user(&accelrwbuf, argp, sizeof(accelrwbuf))) {
					printk("[%s:%d] copy_from_user - Fail(KXTF9_IOCTL_I2C_READ)\n", __func__, __LINE__);
					return -EFAULT;
				}

				if (accelrwbuf[1] < 1) {
#if DEBUG_ACCEL
					printk("[%s:%d] accelrwbut[1]:%d length Fail(KXTF9_IOCTL_I2C_READ)\n", __func__, __LINE__, accelrwbuf[1]);
#endif
					return -EINVAL;
				}
				/* Accel : jay.sim */
#if 1
				if (accelrwbuf[0] == 0x0F) {
					NvAccelerometerI2CGetRegsPassThrough(accelrwbuf[2], &accelrwbuf[3], accelrwbuf[1]);

					if (copy_to_user(argp, &accelrwbuf, sizeof(accelrwbuf))) {
#if DEBUG_ACCEL
						printk("[%s:%d] copy_to_user - Fail(KXTF9_IOCTL_I2C_READ) : rwbuf[1] < 1...\n", __func__, __LINE__);
#endif
						return -EFAULT;
					}
#endif
				}

				break;

			case KXTF9_IOCTL_I2C_WRITE:
				printk("%s KXTF9_IOCTL_I2C_WRITE:accelrwbuf %d %d %d %d\n", __func__, accelrwbuf[0], accelrwbuf[1], accelrwbuf[2], accelrwbuf[3]);
				if (copy_from_user(&accelrwbuf, argp, sizeof(accelrwbuf))) {
#if DEBUG_ACCEL
					printk("[%s:%d] copy_from_user - Fail(KXTF9_IOCTL_I2C_WRITE)\n", __func__, __LINE__);
#endif
					return -EFAULT;
				}
				/*
				   accelrwbuf[0] = slave_addr;  // slave addr - GYRO(0x68-MPU)
				   accelrwbuf[1] = 2;                   // number of bytes to write +1
				   accelrwbuf[2] = reg;               // register address
				   accelrwbuf[3] = value;          // register value
				   */
				if (accelrwbuf[1] < 2) {
#if DEBUG_ACCEL
					printk("[%s:%d] accelrwbut[1]:%d length Fail(KXTF9_IOCTL_I2C_WRITE)\n", __func__, __LINE__, accelrwbuf[1]);
#endif
					return -EINVAL;
				}

				/* Accel : jay.sim */
#if 1
				if (accelrwbuf[0] == 0x0F) {
					NvAccelerometerI2CSetRegsPassThrough(accelrwbuf[2] ,&accelrwbuf[3] , accelrwbuf[1]-1);
#endif
				}
				break;
			case KXTF9_IOCTL_ACCEL_RAW:
				if (copy_from_user(&xyz, argp, sizeof(xyz))) {
					return -EFAULT;
				}
				/*  xyz[0], [1], [2] = accel_x,  accel_y,  accel_z;  */

				atomic_set(&accel_x, xyz[0]);
				atomic_set(&accel_y, xyz[1]);
				atomic_set(&accel_z, xyz[2]);
				//printk(".............KXTF9_IOCTL_TILT................\n");
				break;

				/**/

			default:
				return -EINVAL;
		}

		return 0;
}

static int kxtf9_accel_gyro_misc_open(struct inode *inode, struct file *file)
{
	int err;

	err = nonseekable_open(inode, file);
	if (err < 0)
		return err;
	file->private_data = kxtf9_misc_data;

	return 0;
}

static int kxtf9_accel_gyro_misc_ioctl(struct inode *inode, struct file *file,
		unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	/*struct tegra_acc_device_data *accel =
	  (struct tegra_acc_device_data*)accel_dev;*/
	u8 ctrl[2] = { CTRL_REG1, PC1_OFF };
	int err;
	int tmp;
	int xyz[3] = { 0 };
	struct kxtf9_data *tf9 = file->private_data;

#if DEBUG_ACCEL
	printk("%s is called\n", __func__);
#endif

	switch (cmd) {
		case KXTF9_IOCTL_GET_DELAY:
			tmp = tf9->res_interval;
			if (copy_to_user(argp, &tmp, sizeof(tmp)))
				return -EFAULT;
			break;
		case KXTF9_IOCTL_SET_DELAY:
			if (copy_from_user(&tmp, argp, sizeof(tmp)))
				return -EFAULT;
			if (tmp < 0)
				return -EINVAL;
			tf9->res_interval = max(tmp, tf9->pdata->min_interval);
			err = kxtf9_update_odr(tf9, tf9->res_interval);
			if (err < 0)
				return err;
			ctrl[0] = CTRL_REG3;
			//please check it		ctrl[1] = tf9->resume_state[RES_CTRL_REG1] & 0x18;
			//please check it		tf9->resume_state[RES_CURRENT_ODR] = ctrl[1];
			ctrl[1] = (ctrl[1] >> 1) | (ctrl[1] >> 3);
			//please check it		err = kxtf9_i2c_write(tf9, ctrl, 1);
			//please check it		if (err < 0)
			//please check it			return err;
			//please check it		tf9->resume_state[RES_CTRL_REG3] = ctrl[1];
			break;
		case KXTF9_IOCTL_SET_ENABLE:
			if (copy_from_user(&tmp, argp, sizeof(tmp)))
				return -EFAULT;
			if (tmp < 0 || tmp > 1)
				return -EINVAL;

			if (tmp)
				kxtf9_enable(tf9);
			else
				kxtf9_disable(tf9);
			break;
		case KXTF9_IOCTL_GET_ENABLE:
			tmp = atomic_read(&tf9->enabled);
			if (copy_to_user(argp, &tmp, sizeof(tmp)))
				return -EINVAL;
			break;
		case KXTF9_IOCTL_SET_TILT_ENABLE:
			if (copy_from_user(&tmp, argp, sizeof(tmp)))
				return -EFAULT;
			if (tmp < 0 || tmp > 1)
				return -EINVAL;
			//please check it		if (tmp)
			//please check it			tf9->resume_state[RES_CTRL_REG1] |= TPE;
			//please check it
			//please check it		else
			//please check it			tf9->resume_state[RES_CTRL_REG1] &= (~TPE);
			//please check it		ctrl[1] = tf9->resume_state[RES_CTRL_REG1];
			//please check it		err = kxtf9_i2c_write(tf9, ctrl, 1);
			//please check it		if (err < 0)
			//please check it			return err;
			break;
		case KXTF9_IOCTL_SET_WAKE_ENABLE:
			if (copy_from_user(&tmp, argp, sizeof(tmp)))
				return -EFAULT;
			if (tmp < 0 || tmp > 1)
				return -EINVAL;
			//please check it		if (tmp) {
			//please check it			tf9->resume_state[RES_CTRL_REG1] |= (WUFE | B2SE);
			//please check it			ctrl[1] = tf9->resume_state[RES_CTRL_REG1];
			//please check it			err = kxtf9_i2c_write(tf9, ctrl, 1);
			//please check it			if (err < 0)
			//please check it				return err;
			//please check it		} else {
			//please check it			tf9->resume_state[RES_CTRL_REG1] &= (~WUFE & ~B2SE);
			//please check it			ctrl[1] = tf9->resume_state[RES_CTRL_REG1];
			//please check it			err = kxtf9_i2c_write(tf9, ctrl, 1);
			//please check it			if (err < 0)
			//please check it				return err;
			//please check it		}
			break;
		case KXTF9_IOCTL_SELF_TEST:
			if (copy_from_user(&tmp, argp, sizeof(tmp)))
				return -EFAULT;
			if (tmp < 0 || tmp > 1)
				return -EINVAL;
			ctrl[0] = 0x3A;
			if (tmp) {
				ctrl[1] = 0xCA;
				//please check it			err = kxtf9_i2c_write(tf9, ctrl, 1);
				//please check it			if (err < 0)
				//please check it				return err;
			} else {
				ctrl[1] = 0x00;
				//please check it			err = kxtf9_i2c_write(tf9, ctrl, 1);
				//please check it			if (err < 0)
				//please check it				return err;
			}
			break;
		case KXTF9_IOCTL_READ_ACCEL_XYZ:
			err = kxtf9_get_acceleration_data(tf9, xyz);
			/*NvOdmAccelGetAcceleration(
			  accel_dev->hOdmAcr, &x, &y, &z);
			  xyz[0] = x;	xyz[1] = y; xyz[2] = z;*/

			if (err < 0)
				return err;

			if (copy_to_user(argp, xyz, sizeof(int) * 3))
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
	.minor  = MISC_DYNAMIC_MINOR,
	.name   = STAR_ACCEL_IOCTL_NAME,
	.fops   = &star_accel_fops,
};

static const struct file_operations kxtf9_accel_gyro_misc_fops = {
	.owner = THIS_MODULE,
	.open = kxtf9_accel_gyro_misc_open,
	.ioctl = kxtf9_accel_gyro_misc_ioctl,
};

static struct miscdevice kxtf9_accel_gyro_misc_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "accel_passthrough_daemon", //STAR_ACCEL_IOCTL_NAME
	.fops = &kxtf9_accel_gyro_misc_fops,
};

static ssize_t kxtf9_selftest_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int val = simple_strtoul(buf, NULL, 10);
	u8 ctrl = 0x00;
	if (val)
		ctrl = 0xCA;
	NvOdmAccelerometerSetParameter(accel_dev->hOdmAcr, 0x3A, ctrl);
	return count;
}

#if 1 // YJ, 2010-06-14

/* e.g.
 * #cat sys/bus/i2c/devices/0-000f/pcbmount
 * EVB   -> axis_map = [0, 1, 2], negate = [0, 0, 0]
 * Rev.A -> axis_map = [1, 0, 2], negate = [0, 0, 0] */
static ssize_t kxtf9_pcbmount_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kxtf9_data *tf9 = i2c_get_clientdata(client);
	return sprintf(buf, "axis_map = [%d, %d, %d], negate = [%d, %d, %d] \n",
			tf9->axis_map_x, tf9->axis_map_y, tf9->axis_map_z,
			tf9->negate_x, tf9->negate_y, tf9->negate_z );
}
#endif

#if 0
/*---------------------------------------------------------------------------
    motion_send_event function
 ---------------------------------------------------------------------------*/
/* Common? jay.sim */
void motion_send_accel_detection(int accelx,int accely,int accelz)
{
	//printk("[Gyro_accel][%s:%d] %d %d %d\n",__FUNCTION__, __LINE__,accelx , accely, accelz );

	if (atomic_read(&accel_flag)) {
		input_report_abs(accel_dev->input_dev,ABS_X, accelx);
		input_report_abs(accel_dev->input_dev,ABS_Y, accely);
		input_report_abs(accel_dev->input_dev,ABS_Z, accelz);
		input_sync(accel_dev->input_dev);
	}
}
/**/

static void motion_accel_work_func(struct work_struct *work)
{
	int current_x = 0, current_y = 0, current_z = 0;

	current_x = atomic_read(&accel_x);
	current_y = atomic_read(&accel_y);
	current_z = atomic_read(&accel_z);

	motion_send_accel_detection(current_x, current_y, current_z);
}

static enum hrtimer_restart motion_accel_timer_func(struct hrtimer *timer)
{
	unsigned long polling_time;

	if (atomic_read(&accel_flag)) {
		queue_work(accel_dev->accel_wq, &accel_dev->accel_work);

		polling_time = atomic_read(&accel_delay);
		hrtimer_start(&accel_dev->timer, ktime_set(0, polling_time * 1000000), HRTIMER_MODE_REL);
	}

	return HRTIMER_NORESTART;
}

static ssize_t show_motion_accel_status(struct device *dev, struct device_attribute *attr, char *buf, size_t count)
{
	u32    val;
	val = atomic_read(&accel_flag);
	return sprintf(buf, "%d\n",val);
}

/* Common? : jay.sim */
static ssize_t store_motion_accel_status(struct device *dev, struct device_attribute *attr, char *buf, size_t count)
{
	u32    val;
	val = simple_strtoul(buf, NULL, 10);
	//  printk("[%s] gyro_flag [%d]\n",__func__,val);

	if (val) {
		atomic_set(&accel_flag, 1);
		//      hrtimer_cancel(&tar_motion_dev->timer[0]);
		//      hrtimer_start(&tar_motion_dev->timer[0], ktime_set(0,0), HRTIMER_MODE_REL);
	} else {
		atomic_set(&accel_flag, 0);
		//      hrtimer_cancel(&tar_motion_dev->timer[0]);
	}

	return count;
}

static ssize_t store_motion_accel_delay(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u32    val;
	unsigned long current_delay = 100;

	val = simple_strtoul(buf, NULL, 10);
	current_delay = (short)val;
#if DEBUG_ACCEL
	printk("[motion_accel_delay_store]  flag [%d] current_delay[%ld]\n", val, current_delay);
#endif
	if (atomic_read(&accel_flag)) {
		hrtimer_cancel(&accel_dev->timer);

		if (current_delay < MIN_MOTION_POLLING_TIME) {
			current_delay = MIN_MOTION_POLLING_TIME;
		}

		atomic_set(&accel_delay, current_delay);
		hrtimer_start(&accel_dev->timer, ktime_set(0, current_delay * 1000000), HRTIMER_MODE_REL);
	}

	return count;
}
#endif


static DEVICE_ATTR(selftest, S_IWUSR, NULL, kxtf9_selftest_store);
#if 1 // YJ, 2010-06-14
static DEVICE_ATTR(pcbmount, S_IRUGO, kxtf9_pcbmount_show, NULL);
#endif
//static DEVICE_ATTR(accel_onoff, 0666, show_motion_accel_status, store_motion_accel_status);
//static DEVICE_ATTR(accel_delay, 0666, NULL, store_motion_accel_delay);

static struct attribute *kxtf9_attributes[] = {
	&dev_attr_selftest.attr,
#if 1 // YJ, 2010-06-14
	&dev_attr_pcbmount.attr,
#endif
//	&dev_attr_accel_onoff.attr,
//	&dev_attr_accel_delay.attr,
	NULL
};

static struct attribute_group kxtf9_attribute_group = {
	.attrs = kxtf9_attributes
};
/* /sysfs */

static NvS32 __init tegra_acc_probe(struct platform_device *pdev)
{
//	struct tegra_acc_device_data *accelerometer = NULL;

	struct input_dev *input_dev = NULL;
	struct kxtf9_data *accel_data = NULL;
	NvS32 err;

#if DEBUG_ACCEL
	printk("%s Started\n", __func__);
#endif

	accel_data = kzalloc(sizeof(*accel_data), GFP_KERNEL);
	if (accel_data == NULL) {
		printk("failed to allocate memory for module data\n");
		err = -ENOMEM;
		goto alloc_mem_fail;
	}

	accel_data->pdata = kmalloc(sizeof(*accel_data->pdata), GFP_KERNEL);
	if (accel_data->pdata == NULL)
		goto alloc_pdata_fail;

#if 1 // YJ, 2010-06-14
	/* EVB : 
	accel_data->axis_map_x = 0;
	accel_data->axis_map_y = 1;
	accel_data->axis_map_z = 2;

	accel_data->negate_x = 0;
	accel_data->negate_y = 0;
	accel_data->negate_z = 0;
	*/

	// Rev. A.
	accel_data->axis_map_x = 1;
	accel_data->axis_map_y = 0;
	accel_data->axis_map_z = 2;

	accel_data->negate_x = 0;
	accel_data->negate_y = 0;
	accel_data->negate_z = 0;
#endif

	mutex_init(&accel_data->lock);
	mutex_lock(&accel_data->lock);

	memset(accel_data->resume, 0, ARRAY_SIZE(accel_data->resume));
	accel_data->resume[RES_DATA_CTRL] = accel_data->pdata->data_odr_init;
	accel_data->resume[RES_CTRL_REG1] = accel_data->pdata->ctrl_reg1_init;
	accel_data->resume[RES_INT_CTRL1] = accel_data->pdata->int_ctrl_init;
	accel_data->resume[RES_TILT_TIMER] = accel_data->pdata->tilt_timer_init;
	accel_data->resume[RES_CTRL_REG3] = accel_data->pdata->engine_odr_init;
	accel_data->resume[RES_WUF_TIMER] = accel_data->pdata->wuf_timer_init;
	accel_data->resume[RES_WUF_THRESH] = accel_data->pdata->wuf_thresh_init;
	accel_data->resume[RES_TDT_TIMER] = accel_data->pdata->tdt_timer_init;
	accel_data->resume[RES_TDT_H_THRESH] = accel_data->pdata->tdt_h_thresh_init;
	accel_data->resume[RES_TDT_L_THRESH] = accel_data->pdata->tdt_l_thresh_init;
	accel_data->resume[RES_TAP_TIMER] = accel_data->pdata->tdt_tap_timer_init;
	accel_data->resume[RES_TOTAL_TIMER] = accel_data->pdata->tdt_total_timer_init;
	accel_data->resume[RES_LAT_TIMER] = accel_data->pdata->tdt_latency_timer_init;
	accel_data->resume[RES_WIN_TIMER]    = accel_data->pdata->tdt_window_timer_init;
	accel_data->res_interval = accel_data->pdata->poll_interval;

	kxtf9_misc_data = accel_data;

	mutex_unlock(&accel_data->lock);

	//**************************//
	accel_dev = kzalloc(sizeof(*accel_dev), GFP_KERNEL);
	if (accel_dev == NULL) {
		err = -ENOMEM;
		pr_err("tegra_acc_probe: Failed to memory\n");
		goto alloc_dev_fail;
	}
//	accel_dev = accelerometer;

#if DEBUG_ACCEL
	printk("Allocate Memory for Accelerometer Device \n");
#endif

	err = open_def_odm_accl();
	if (!err) {
		pr_err("open_def_odm_accl: NvOdmAccelSetIntForceThreshold\n");
		goto open_odm_accel_fail;
	}

#if DEBUG_ACCEL
	printk("Open NVOdm for Accelerometer Device \n");
#endif

	input_dev = input_allocate_device();
	if (input_dev == NULL) {
		err = -ENOMEM;
		pr_err("tegra_acc_probe: Failed to allocate input device\n");
		goto alloc_input_dev_fail;
	}
	accel_dev->input_dev = input_dev;

	set_bit(EV_SYN, accel_dev->input_dev->evbit);
	set_bit(EV_KEY, accel_dev->input_dev->evbit);
	set_bit(EV_ABS, accel_dev->input_dev->evbit);

	input_set_abs_params(accel_dev->input_dev, ABS_X,
			accel_dev->min_data.x,
			accel_dev->max_data.x, 0, 0);
	input_set_abs_params(accel_dev->input_dev, ABS_Y,
			accel_dev->min_data.y,
			accel_dev->max_data.y, 0, 0);
	input_set_abs_params(accel_dev->input_dev, ABS_Z,
			accel_dev->min_data.z,
			accel_dev->max_data.z, 0, 0);

	platform_set_drvdata(pdev, accel_dev);

	input_dev->name = "accelerometer_tegra";
	err = input_register_device(input_dev);
	if (err) {
		pr_err("tegra_acc_probe: Unable to register %s\
				input device\n", input_dev->name);
		goto register_input_dev_fail;
	}
#if DEBUG_ACCEL
	printk("Register Input Device for Accelerometer Device \n");
#endif

#if 0	/* TODO : which function should be called? */
//	err = device_create_file(&accel_dev->input_dev->dev, &dev_attr_selftest);
	err = device_create_file(&pdev->dev, &dev_attr_selftest);
#else
//	err = sysfs_create_group(&accel_dev->input_dev->dev, &kxtf9_attribute_group);
	err = sysfs_create_group(&pdev->dev.kobj, &kxtf9_attribute_group);
#endif
	if (err)
		goto create_sysfs_fail;
#if DEBUG_ACCEL
	printk("Create sysfs attributes for Accelerometer Device \n");
#endif

	err = misc_register(&star_accel_misc_device);
	if (err) {
#if DEBUG_ACCEL
		printk(KERN_ERR"star_motion_misc_device register failed\n");
#endif
		goto register_misc_fail1;
	}
#if DEBUG_ACCEL
	printk("Register Misc. Device for 'accel_daemon' device \n");
#endif

	defaultaccel.x = 1024;
	defaultaccel.y = 0;
	defaultaccel.z = 0;

	accelpassthrough.x = 0;
	accelpassthrough.y = 0;
	accelpassthrough.z = 0;
	//accelpassthrough.x = NULL;
	//accelpassthrough.y = NULL;
	//accelpassthrough.z = NULL;

	err = misc_register(&kxtf9_accel_gyro_misc_device);
	if (err < 0) {
		printk("tf9_device register failed\n");
		goto register_misc_fail2;
	}
#if DEBUG_ACCEL
	printk("Register Misc. Device for 'accel_passthrough_daemon' device(%lu) \n");
#endif

	atomic_set(&accel_init, 1);
	sensors_wake_up_now();
	printk("%s accel init done\n", __func__);

	return 0;

register_misc_fail2:
	misc_deregister(&star_accel_misc_device);

register_misc_fail1:
#if 1	/* TODO: */
	device_remove_file(&pdev->dev, &dev_attr_selftest);
#else
	sysfs_remove_group(&pdev->dev.kobj, &kxtf9_attribute_group);
#endif
create_sysfs_fail:
	input_unregister_device(input_dev);

register_input_dev_fail:

	input_free_device(input_dev);
alloc_input_dev_fail:
	close_odm_accl();

open_odm_accel_fail:
	kfree(accel_dev);

alloc_dev_fail:
	kfree(accel_data->pdata);

alloc_pdata_fail:
	kfree(accel_data);

alloc_mem_fail:
	printk("Error Accelerometer Device Initialization\n");
	return -ENOMEM;
}

static NvS32 tegra_acc_remove(struct platform_device *pdev)
{

//	kxtf9_device_power_off(kxtf9_misc_data);
	misc_deregister(&kxtf9_accel_gyro_misc_device);
	misc_deregister(&star_accel_misc_device);

#if 1	/* TODO: */
	device_remove_file(&pdev->dev, &dev_attr_selftest);
#else
	sysfs_remove_group(&pdev->dev.kobj, &kxtf9_attribute_group);
#endif

	input_unregister_device(accel_dev->input_dev);
	input_free_device(accel_dev->input_dev);
	close_odm_accl();
	kfree(accel_dev);
	kfree(kxtf9_misc_data->pdata);
	kfree(kxtf9_misc_data);

	return 0;
}

static NvS32 tegra_acc_resume(struct platform_device *pdev)
{
	#if 0
	struct kxtf9_data *tf9 = (void *)kxtf9_misc_data;
	printk("=============>  [%s,%d]\n",__FUNCTION__, __LINE__);
	kxtf9_device_power_on(tf9);

	accelpassthrough.x = 0;
	accelpassthrough.y = 0;
	accelpassthrough.z = 0;
	//accelpassthrough.x = NULL;
	//accelpassthrough.y = NULL;
	//accelpassthrough.z = NULL;
	#endif
	
	return 0;
}

static NvS32 tegra_acc_suspend(struct platform_device *pdev, pm_message_t state)
{
	#if 0	
	struct kxtf9_data *tf9 = (void *)kxtf9_misc_data;
	printk("=============>  [%s,%d]\n",__FUNCTION__, __LINE__);
	kxtf9_device_power_off(tf9);
	#endif
	
	return 0;
}

static struct platform_driver tegra_acc_driver = {
	.probe	= tegra_acc_probe,
	.remove	= tegra_acc_remove,
	.resume = tegra_acc_resume,
	.suspend = tegra_acc_suspend,
	.driver	= {
		.name = "tegra_accelerometer",
	},
};

static NvS32 __devinit tegra_acc_init(void)
{
	int ret = -1;
	printk("%s is called \n", __FUNCTION__);
	ret = platform_driver_register(&tegra_acc_driver);

	return ret;

}

static void __exit tegra_acc_exit(void)
{
	printk("%s is called \n", __FUNCTION__);
	platform_driver_unregister(&tegra_acc_driver);
}

module_init(tegra_acc_init);
module_exit(tegra_acc_exit);

MODULE_DESCRIPTION("Tegra ODM Accelerometer driver");
MODULE_LICENSE("GPL");

#if 0
/*---------------------------------------------------------------------------
  kxtf9_reg_i2c_client
  ---------------------------------------------------------------------------*/
void  kxtf9_reg_i2c_client(struct i2c_client *client)
{
#if DEBUG_ACCEL
	printk("kxtf9_reg_i2c_client..........................\n");
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
	//printk("[Gyro_accel][%s:%d]\n",__FUNCTION__, __LINE__);
	//printk("reg: %d(%x) / buf: %d(%x)/ length : %d(%x)\n",reg, reg, buf, buf, length, length);
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
	//printk("[Gyro_accel][%s:%d]\n",__FUNCTION__, __LINE__);
	//printk("buffer[0]: %d(%x) / buffer[\1]: %d(%x)/ length : %d(%x)\n",buffer[0],buffer[0],buffer[1],buffer[1], length, length);
	//NvAccelerometerI2CSetRegsPassThrough(buffer[0] ,&buffer[1] , length-1 );
	//NvAccelerometerI2CSetRegs(accel_dev->hOdmAcr, 0x1B, &val, 1);


	return status;
}

static int kxtf9_i2c_read(struct kxtf9_data *tf9, u8 addr, u8 *data, int len)
{
	int err;

	struct i2c_msg msgs[] = {
		{
			.addr = tf9->client->addr,
			.flags = tf9->client->flags & I2C_M_TEN,
			.len = 1,
			.buf = &addr,
		},
		{
			.addr = tf9->client->addr,
			.flags = (tf9->client->flags & I2C_M_TEN) | I2C_M_RD,
			.len = len,
			.buf = data,
		},
	};
	err = i2c_transfer(tf9->client->adapter, msgs, 2);

	if (err != 2)
		dev_err(&tf9->client->dev, "read transfer error\n");
	else
		err = 0;

	return err;
}

static int kxtf9_i2c_write(struct kxtf9_data *tf9, u8 addr, u8 *data, int len)
{
	int err;
	int i;
	u8 buf[len + 1];

	struct i2c_msg msgs[] = {
		{
			.addr = tf9->client->addr,
			.flags = tf9->client->flags & I2C_M_TEN,
			.len = len + 1,
			.buf = buf,
		},
	};

	buf[0] = addr;
	for (i = 0; i < len; i++)
		buf[i + 1] = data[i];

	err = i2c_transfer(tf9->client->adapter, msgs, 1);

	if (err != 1)
		dev_err(&tf9->client->dev, "write transfer error\n");
	else
		err = 0;

	return err;
}

static int kxtf9_verify(struct kxtf9_data *tf9)
{
	int err;
	u8 buf;

	err = NvOdmAccelerometerGetParameter(accel_dev->hOdmAcr, WHO_AM_I, &buf);


#if DEBUG_ACCEL
	printk("WHO_AM_I = 0x%02x\n", buf);  // 	dev_info(&tf9->client->dev, "WHO_AM_I = 0x%02x\n", buf);
#endif

	if (err < 0)
		printk("read err int source\n");  // 		dev_err(&tf9->client->dev, "read err int source\n");
	if (buf != 1)
		err = -1;
	return err;
}

static irqreturn_t kxtf9_isr(int irq, void *dev)
{
	struct kxtf9_data *tf9 = dev;

	disable_irq_nosync(irq);
	schedule_work(&tf9->irq_work);

	return IRQ_HANDLED;
}

static void kxtf9_irq_work_func(struct work_struct *work)
{
	/*
	 *	int_status output:
	 *	[INT_SRC_REG2][INT_SRC_REG1][TILT_POS_PRE][TILT_POS_CUR]
	 *	INT_SRC_REG1, TILT_POS_PRE, and TILT_POS_CUR directions are translated
	 *	based on platform data variables.
	 */

	int err;
	int int_status = 0;
	u8 status;
	u8 buf[2];

	struct kxtf9_data *tf9
		= container_of(work, struct kxtf9_data, irq_work);

	//err = kxtf9_i2c_read(tf9, INT_STATUS_REG, &status, 1);
	err = NvOdmAccelerometerGetParameter(accel_dev->hOdmAcr, INT_STATUS_REG, &status);
	if (err < 0)
		printk("read err int source\n"); //		dev_err(&tf9->client->dev, "read err int source\n");
	int_status = status << 24;
	if ((status & TPS) > 0) {
		//err = kxtf9_i2c_read(tf9, TILT_POS_CUR, buf, 2);
		err = NvOdmAccelerometerGetParameter(accel_dev->hOdmAcr, TILT_POS_CUR, &buf);

		if (err < 0)
			printk("read err tilt dir\n");  //			dev_err(&tf9->client->dev, "read err tilt dir\n");
		int_status |= kxtf9_resolve_dir(tf9, buf[0]);
		int_status |= kxtf9_resolve_dir(tf9, buf[1]) << 8;
		/*** DEBUG OUTPUT - REMOVE ***/
#if DEBUG_ACCEL
		printk("IRQ TILT [%x]\n",
				kxtf9_resolve_dir(tf9, buf[0]));
#endif

		/*dev_info(&tf9->client->dev, "IRQ TILT [%x]\n",
		  kxtf9_resolve_dir(tf9, buf[0]));*/
		/*** <end> DEBUG OUTPUT - REMOVE ***/
	}

	if (((status & TDTS0) | (status & TDTS1)) > 0) {
		//err = kxtf9_i2c_read(tf9, INT_SRC_REG1, buf, 1);
		err = NvOdmAccelerometerGetParameter(accel_dev->hOdmAcr, INT_SRC_REG1, &buf);
		if (err < 0)
			printk("read err tap dir\n");  // 			dev_err(&tf9->client->dev, "read err tap dir\n");
		int_status |= (kxtf9_resolve_dir(tf9, buf[0])) << 16;
		/*** DEBUG OUTPUT - REMOVE ***/
#if DEBUG_ACCEL
		printk("IRQ TAP%d [%x]\n",
				((status & TDTS1) ? (2) : (1)), kxtf9_resolve_dir(tf9, buf[0]));
#endif
		/*dev_info(&tf9->client->dev, "IRQ TAP%d [%x]\n",
		  ((status & TDTS1) ? (2) : (1)), kxtf9_resolve_dir(tf9, buf[0]));*/
		/*** <end> DEBUG OUTPUT - REMOVE ***/
	}
#if DEBUG_ACCEL
	/*** DEBUG OUTPUT - REMOVE ***/
	if ((status & 0x02) > 0) {
		if (((status & TDTS0) | (status & TDTS1)) > 0)
			printk("IRQ WUF + TAP\n"); 			//dev_info(&tf9->client->dev, "IRQ WUF + TAP\n");
		else
			printk("IRQ WUF\n"); 			//dev_info(&tf9->client->dev, "IRQ WUF\n");
	}
	/*** <end> DEBUG OUTPUT - REMOVE ***/
#endif

	if (int_status & 0x2FFF) {
		input_report_abs(tf9->input_dev, ABS_MISC, int_status);
		input_sync(tf9->input_dev);
	}
	//err = kxtf9_i2c_read(tf9, INT_REL, buf, 1);
	err = NvOdmAccelerometerGetParameter(accel_dev->hOdmAcr, INT_REL, &buf);
	if (err < 0)
		printk("error clearing interrupt status: %d\n", err);
	/*dev_err(&tf9->client->dev,
	  "error clearing interrupt status: %d\n", err);*/

	enable_irq(tf9->irq);
}

int kxtf9_update_g_range(struct kxtf9_data *tf9, u8 new_g_range)
{
	int err;
	u8 shift;
	u8 buf;

	switch (new_g_range) {
		case KXTF9_G_2G:
			shift = KXTF9_SHIFT_ADJ_2G;
			break;
		case KXTF9_G_4G:
			shift = KXTF9_SHIFT_ADJ_4G;
			break;
		case KXTF9_G_8G:
			shift = KXTF9_SHIFT_ADJ_8G;
			break;
		default:
			printk("invalid g range request\n");
			//dev_err(&tf9->client->dev, "invalid g range request\n");
			return -EINVAL;
	}
	if (shift != tf9->pdata->shift_adj) {
		if (tf9->pdata->shift_adj > shift)
			tf9->resume[RES_WUF_THRESH] >>=
				(tf9->pdata->shift_adj - shift);
		if (tf9->pdata->shift_adj < shift)
			tf9->resume[RES_WUF_THRESH] <<=
				(shift - tf9->pdata->shift_adj);

		if (atomic_read(&tf9->enabled)) {
			NvOdmAccelerometerSetParameter(accel_dev->hOdmAcr, CTRL_REG1, PC1_OFF);

			if (err < 0)
				return err;
			NvOdmAccelerometerSetParameter(accel_dev->hOdmAcr, WUF_THRESH, tf9->resume[RES_WUF_THRESH]);

			if (err < 0)
				return err;
			NvOdmAccelerometerSetParameter(accel_dev->hOdmAcr, CTRL_REG1, (tf9->resume[RES_CTRL_REG1] & 0xE7) | new_g_range);
			if (err < 0)
				return err;
		}
	}
	tf9->resume[RES_CTRL_REG1] = buf;
	tf9->pdata->shift_adj = shift;

	return 0;
}

static void kxtf9_report_values(struct kxtf9_data *tf9, int *xyz)
{
	input_report_abs(tf9->input_dev, ABS_X, xyz[0]);
	input_report_abs(tf9->input_dev, ABS_Y, xyz[1]);
	input_report_abs(tf9->input_dev, ABS_Z, xyz[2]);
	input_sync(tf9->input_dev);
}

static void kxtf9_input_work_func(struct work_struct *work)
{
	struct kxtf9_data *tf9 = container_of((struct delayed_work *)work,
			struct kxtf9_data, input_work);
	int xyz[3] = { 0 };

	mutex_lock(&tf9->lock);

	if (kxtf9_get_acceleration_data(tf9, xyz) == 0)
		kxtf9_report_values(tf9, xyz);

	schedule_delayed_work(&tf9->input_work,
			msecs_to_jiffies(tf9->res_interval));
	mutex_unlock(&tf9->lock);
}

static void kxtf9_input_cleanup(struct kxtf9_data *tf9)
{
	input_unregister_device(tf9->input_dev);
}

static int kxtf9_misc_open(struct inode *inode, struct file *file)
{
	int err;

	err = nonseekable_open(inode, file);
	if (err < 0)
		return err;
	file->private_data = kxtf9_misc_data;

	return 0;
}

static int kxtf9_misc_ioctl(struct inode *inode, struct file *file,
		unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	/*struct tegra_acc_device_data *accel =
	  (struct tegra_acc_device_data*)accel_dev;*/
	u8 ctrl[2] = { CTRL_REG1, PC1_OFF };
	int err;
	int tmp;
	int xyz[3] = { 0 };
	struct kxtf9_data *tf9 = file->private_data;

#if DEBUG_ACCEL
	printk("%s is called\n", __func__);
#endif

	switch (cmd) {
		case KXTF9_IOCTL_GET_DELAY:
			tmp = tf9->res_interval;
			if (copy_to_user(argp, &tmp, sizeof(tmp)))
				return -EFAULT;
			break;
		case KXTF9_IOCTL_SET_DELAY:
			if (copy_from_user(&tmp, argp, sizeof(tmp)))
				return -EFAULT;
			if (tmp < 0)
				return -EINVAL;
			tf9->res_interval = max(tmp, tf9->pdata->min_interval);
			err = kxtf9_update_odr(tf9, tf9->res_interval);
			if (err < 0)
				return err;
			ctrl[0] = CTRL_REG3;
			//please check it		ctrl[1] = tf9->resume_state[RES_CTRL_REG1] & 0x18;
			//please check it		tf9->resume_state[RES_CURRENT_ODR] = ctrl[1];
			ctrl[1] = (ctrl[1] >> 1) | (ctrl[1] >> 3);
			//please check it		err = kxtf9_i2c_write(tf9, ctrl, 1);
			//please check it		if (err < 0)
			//please check it			return err;
			//please check it		tf9->resume_state[RES_CTRL_REG3] = ctrl[1];
			break;
		case KXTF9_IOCTL_SET_ENABLE:
			if (copy_from_user(&tmp, argp, sizeof(tmp)))
				return -EFAULT;
			if (tmp < 0 || tmp > 1)
				return -EINVAL;

			if (tmp)
				kxtf9_enable(tf9);
			else
				kxtf9_disable(tf9);
			break;
		case KXTF9_IOCTL_GET_ENABLE:
			tmp = atomic_read(&tf9->enabled);
			if (copy_to_user(argp, &tmp, sizeof(tmp)))
				return -EINVAL;
			break;
		case KXTF9_IOCTL_SET_TILT_ENABLE:
			if (copy_from_user(&tmp, argp, sizeof(tmp)))
				return -EFAULT;
			if (tmp < 0 || tmp > 1)
				return -EINVAL;
			//please check it		if (tmp)
			//please check it			tf9->resume_state[RES_CTRL_REG1] |= TPE;
			//please check it
			//please check it		else
			//please check it			tf9->resume_state[RES_CTRL_REG1] &= (~TPE);
			//please check it		ctrl[1] = tf9->resume_state[RES_CTRL_REG1];
			//please check it		err = kxtf9_i2c_write(tf9, ctrl, 1);
			//please check it		if (err < 0)
			//please check it			return err;
			break;
		case KXTF9_IOCTL_SET_WAKE_ENABLE:
			if (copy_from_user(&tmp, argp, sizeof(tmp)))
				return -EFAULT;
			if (tmp < 0 || tmp > 1)
				return -EINVAL;
			//please check it		if (tmp) {
			//please check it			tf9->resume_state[RES_CTRL_REG1] |= (WUFE | B2SE);
			//please check it			ctrl[1] = tf9->resume_state[RES_CTRL_REG1];
			//please check it			err = kxtf9_i2c_write(tf9, ctrl, 1);
			//please check it			if (err < 0)
			//please check it				return err;
			//please check it		} else {
			//please check it			tf9->resume_state[RES_CTRL_REG1] &= (~WUFE & ~B2SE);
			//please check it			ctrl[1] = tf9->resume_state[RES_CTRL_REG1];
			//please check it			err = kxtf9_i2c_write(tf9, ctrl, 1);
			//please check it			if (err < 0)
			//please check it				return err;
			//please check it		}
			break;
		case KXTF9_IOCTL_SELF_TEST:
			if (copy_from_user(&tmp, argp, sizeof(tmp)))
				return -EFAULT;
			if (tmp < 0 || tmp > 1)
				return -EINVAL;
			ctrl[0] = 0x3A;
			if (tmp) {
				ctrl[1] = 0xCA;
				//please check it			err = kxtf9_i2c_write(tf9, ctrl, 1);
				//please check it			if (err < 0)
				//please check it				return err;
			} else {
				ctrl[1] = 0x00;
				//please check it			err = kxtf9_i2c_write(tf9, ctrl, 1);
				//please check it			if (err < 0)
				//please check it				return err;
			}
			break;
		case KXTF9_IOCTL_READ_ACCEL_XYZ:
			err=kxtf9_get_acceleration_data(tf9, xyz);
			/*NvOdmAccelGetAcceleration(
			  accel_dev->hOdmAcr, &x, &y, &z);
			  xyz[0] = x;	xyz[1] = y; xyz[2] = z;*/

			if (err < 0)
				return err;

			if (copy_to_user(argp, xyz, sizeof(int) * 3))
				return -EINVAL;

			return err;

			break;

		default:
			return -EINVAL;
	}

	return 0;
}

static const struct file_operations kxtf9_misc_fops = {
	.owner	= THIS_MODULE,
	.open	= kxtf9_misc_open,
	.ioctl	= kxtf9_misc_ioctl,
};

static struct miscdevice kxtf9_misc_device = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= "kxtf9",
	.fops	= &kxtf9_misc_fops,
};

#endif
