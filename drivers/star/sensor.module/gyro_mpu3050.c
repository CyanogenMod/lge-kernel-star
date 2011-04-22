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
#include <linux/wait.h>
#include <mach/nvrm_linux.h>

#include <nvodm_gyroscope_accel.h>
#include <nvodm_gyro_accel_kxtf9.h>
#include "mach/lprintk.h"

#include "mpu3050.h"
#include "kxtf9.h"
#include "star_sensors.h"

#define  DEBUG_GYRO 0

#ifdef CONFIG_HAS_EARLYSUSPEND // wkkim : temporary early suspend apply
#include <linux/earlysuspend.h>
static struct early_suspend    early_suspend;
#endif

extern atomic_t gyro_init;
extern atomic_t boot_flag;
extern void sensors_wake_up_now(void);

extern void motion_sensor_power_on(void);
extern void motion_sensor_power_off(void);
void motion_send_tilt_detection(int yaw,int pitch,int roll);
void motion_send_composite_detection(int *value);
void motion_send_gyro_detection(int gyro_x,int gyro_y,int gyro_z);
void motion_send_tap_detection(int type,int direction);
void motion_send_flip_detection(int value);
void motion_send_shake_detection(int value);
void motion_send_snap_detection(int direction);

struct tegra_gyro_accel_device_data
{
//	struct task_struct	*task;
	struct input_dev	*input_dev;
};

struct tegra_acc_device_data *gyro_accel_dev = NULL;
struct i2c_client    *kxtf9_i2c_client;

static atomic_t tilt_flag;
static atomic_t tap_flag;
static atomic_t shake_flag;
static atomic_t snap_flag;
static atomic_t flip_flag;
static atomic_t gyro_flag;
static atomic_t tilt_delay;
static atomic_t gyro_delay;
static atomic_t suspend_flag;
static atomic_t composite[12];

//static NvBool i2c_busy_flag; //jongik2.kim 20100910 i2c_fix
extern int star_get_i2c_busy(void);
extern void star_set_i2c_busy(void);
extern void star_unset_i2c_busy(void);
//

static atomic_t	tilt_roll, tilt_pitch, tilt_yaw;
static atomic_t	gyro_x, gyro_y, gyro_z;

/* jay.sim */
extern atomic_t	mag_x, mag_y, mag_z;
extern atomic_t	accel_x, accel_y, accel_z;

extern atomic_t accel_flag;
extern atomic_t accel_delay;
extern atomic_t compass_flag;
extern atomic_t compass_delay;
/**/

struct star_motion_device {
	NvOdmGyroAccelDeviceHandle	hOdmGyroAccel;
	struct input_dev  		    *input_dev;
//	struct input_dev  		    *input_dev1;	 /* motion daemon process */
	NvU32						freq;
	NvBool						bThreadAlive;
	NvBool						show_log;
	NvOdmGyroAccelIntType		IntType;
	NvOdmGyroAccelAxisType		IntMotionAxis;
	NvOdmGyroAccelAxisType		IntTapAxis;
	struct timeval				tv;
	struct hrtimer       		timer[4];            /* [0] acceleroemter raw data, [1] tilt , [2] Gyro, [3] Compass */

	struct work_struct     		accel_work;
	struct workqueue_struct	 	*accel_wq;
	struct work_struct     		tilt_work;
	struct workqueue_struct	 	*tilt_wq;
	struct work_struct     		gyro_work;
	struct workqueue_struct	 	*gyro_wq;
	struct work_struct     		compass_work;
	struct workqueue_struct	 	*compass_wq;

	int  						irq;
	int  						use_irq;
};

static struct star_motion_device *star_motion_dev = NULL;
/* jay.sim */
void magnetic_input_report(int *); /* wkkim magnetic repot */

#define write_lock(lock)		_write_lock(lock)
#define read_lock(lock)			_read_lock(lock)
rwlock_t getbuflock;
//static unsigned char accelrwbuf[200] = {0,};    /* MPU3050 i2c MAX data length */
static unsigned char rwbuf[200] = {0,};     	 /* MPU3050 i2c MAX data length */

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
	//printk("motion close\n");
	return 0;
}

static int star_motion_ioctl(struct inode *inode, struct file *file, unsigned int cmd,unsigned long arg)
{
	void __user *argp = (void __user *)arg;

	int buf[13] = {0,};
	int flag = 0;
	int delay = 0;
	int onoff_flag = 0;
	int ret = 0;
	int i = 0;

	switch (cmd) {
		case MOTION_IOCTL_ENABLE_DISABLE:
#if 0
			printk("%s:ioctl cmd(MOTION_IOCTL_ENABLE_DISABLE:0x%x)\n", __FUNCTION__, cmd);
#endif
			/*
				0: disable sensor
				1: orientation (tilt)
				2: accelerometer
				3: tap
				4: shake
			*/

			//printk(".............star_motion_ioctl................\n");
			flag = STAR_SENSOR_NONE;

			if (atomic_read(&accel_flag)) {
				//printk(".............if(atomic_read(&snap_flag)){................\n");
				flag |= STAR_ACCELEROMETER;
			}

			if (atomic_read(&compass_flag)) {
				//printk(".............if(atomic_read(&gyro_flag)){................\n");
				flag |= STAR_COMPASS;
			}

			if (atomic_read(&tilt_flag)) {
				//printk(".............if(atomic_read(&tilt_flag)){................\n");
				flag |= STAR_TILT;
			}

			if (atomic_read(&gyro_flag)) {
				//printk(".............if(atomic_read(&snap_flag)){................\n");
				flag |= STAR_GYRO;
			}

			if (atomic_read(&tap_flag)) {
				//printk(".............if(atomic_read(&tap_flag)){................\n");
				flag |= STAR_TAP;
			}

			if (atomic_read(&flip_flag)) {
				//printk(".............if(atomic_read(&flip_flag)){................\n");
				flag |= STAR_FLIP;
			}

			if (atomic_read(&shake_flag)) {
				//printk(".............if(atomic_read(&shake_flag)){................\n");
				flag |= STAR_SHAKE;
			}

			if (atomic_read(&snap_flag)) {
				//printk(".............if(atomic_read(&snap_flag)){................\n");
				flag |= STAR_SNAP;
			}

			if (copy_to_user(argp,&flag, sizeof(flag))) {
				//printk(".............MOTION_IOCTL_SNAP................\n");
				return -EFAULT;
			}
			break;
#if 0
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
			//printk(".............MOTION_IOCTL_TILT................\n");
			break;
#endif
		case MOTION_IOCTL_TILT:
#if DEBUG_GYRO
			printk("%s:ioctl cmd(MOTION_IOCTL_TILT:0x%x)\n", __FUNCTION__, cmd);
#endif
			if (copy_from_user(&buf, argp, sizeof(buf))) {
				return -EFAULT;
			}
			/* 	buf[0], [1], [2] = roll,  pitch,  yaw;	 */

			atomic_set(&tilt_yaw, buf[0]);
			atomic_set(&tilt_pitch, buf[1]);
			atomic_set(&tilt_roll, buf[2]);
			//printk(".............MOTION_IOCTL_TILT................\n");
			break;
		case MOTION_IOCTL_COMPOSITE:
#if DEBUG_GYRO
			printk("%s:ioctl cmd(MOTION_IOCTL_COMPOSITE:0x%x)\n", __FUNCTION__, cmd);
#endif
			if (copy_from_user(&buf, argp, sizeof(buf))) {
				return -EFAULT;
			}

			for (i = 0; i < 12; i++) {
				atomic_set(&composite[i], buf[i]);
			}

			//motion_send_composite_detection(buf);
			break;
		case MOTION_IOCTL_GYRO_RAW:
#if DEBUG_GYRO
			printk("%s:ioctl cmd(MOTION_IOCTL_GYRO_RAW:0x%x)\n", __FUNCTION__, cmd);
#endif
			if (copy_from_user(&buf, argp, sizeof(buf))) {
				return -EFAULT;
			}
			/* 	buf[0], [1], [2] = gyro_x,  gyro_y,  gyro_z; */

			atomic_set(&gyro_x, buf[0]);
			atomic_set(&gyro_y, buf[1]);
			atomic_set(&gyro_z, buf[2]);

			//motion_send_tilt_detection(buf[0],buf[1],buf[2]);
			//printk(".............MOTION_IOCTL_GYRO_RAW................\n");
			break;
#if 0
		case MOTION_IOCTL_MAGNETIC_RAW: /* wkkim add to recieve compass raw value */
			if (copy_from_user(buf, argp, sizeof(int) * 3)) {
				return -EFAULT;
			}
			//magnetic_input_report(buf);
			atomic_set(&mag_x, buf[0]);
			atomic_set(&mag_y, buf[1]);
			atomic_set(&mag_z, buf[2]);
			break;
#endif
		case MOTION_IOCTL_TAP:
#if DEBUG_GYRO
			printk("%s:ioctl cmd(MOTION_IOCTL_TAP:0x%x)\n", __FUNCTION__, cmd);
#endif
			if (copy_from_user(&buf, argp, sizeof(buf))) {
				return -EFAULT;
			}
			/*
			   buf[0] = type;
			   buf[1] = direction;
			   */
#if DEBUG_GYRO
			printk(D_SENSOR,".............MOTION_IOCTL_TAP................\n");
#endif
			motion_send_tap_detection(buf[0], buf[1]);
			break;
		case MOTION_IOCTL_FLIP:
#if DEBUG_GYRO
			printk("%s:ioctl cmd(MOTION_IOCTL_FLIP:0x%x)\n", __FUNCTION__, cmd);
#endif
			if (copy_from_user(&buf, argp, sizeof(buf))) {
				return -EFAULT;
			}
#if DEBUG_GYRO
			printk(D_SENSOR,".............MOTION_IOCTL_FLIP................\n");
#endif
			motion_send_flip_detection(buf[0]);
			break;
		case MOTION_IOCTL_SHAKE:
#if DEBUG_GYRO
			printk("%s:ioctl cmd(MOTION_IOCTL_SHAKE:0x%x)\n", __FUNCTION__, cmd);
#endif
			if (copy_from_user(&buf, argp, sizeof(buf))) {
				return -EFAULT;
			}
			/* 			buf[0] = event;   		  */
#if DEBUG_GYRO
			printk(D_SENSOR,".............MOTION_IOCTL_SHAKE................\n");
#endif
			motion_send_shake_detection(buf[0]);
			break;
		case MOTION_IOCTL_SNAP:
#if DEBUG_GYRO
			printk("%s:ioctl cmd(MOTION_IOCTL_SNAP:0x%x)\n", __FUNCTION__, cmd);
#endif
			if (copy_from_user(&buf, argp, sizeof(buf))) {
				return -EFAULT;
			}
			/*
			   buf[0] = direction;
			   */
#if DEBUG_GYRO
			printk(D_SENSOR,".............MOTION_IOCTL_SNAP................\n");
#endif
			motion_send_snap_detection(buf[0]);
			break;
		case MOTION_IOCTL_SENSOR_DELAY:
#if DEBUG_GYRO
			printk("%s:ioctl cmd(MOTION_IOCTL_SENSOR_DELAY:0x%x)\n", __FUNCTION__, cmd);
#endif
			delay = atomic_read(&tilt_delay);

			//printk("MOTION_IOCTL_SENSOR_DELAY[%d]",delay);

			if (copy_to_user(argp, &delay, sizeof(delay))) {
				return -EFAULT;
			}
			break;
		case MOTION_IOCTL_SENSOR_SUSPEND_RESUME:
			onoff_flag = atomic_read(&suspend_flag);

#if DEBUG_GYRO
			printk("%s:ioctl cmd, (MOTION_IOCTL_SENSOR_SUSPEND_RESUME:0x%x) value : %d\n", __FUNCTION__, cmd, onoff_flag);
#endif

			if (copy_to_user(argp, &onoff_flag, sizeof(onoff_flag))) {
				return -EFAULT;
			}
			break;
#if 1
		case MOTION_IOCTL_ACCEL_COMPASS_SLEEP_MODE:
			//printk(".............MOTION_IOCTL_ACCEL_COMPASS_SLEEP_MODE................\n");
			motion_sensor_power_off();

			break;
		case MOTION_IOCTL_ACCEL_COMPASS_SLEEP_WAKE_UP:
			//printk(".............MOTION_IOCTL_ACCEL_COMPASS_SLEEP_WAKE_UP................\n");
			motion_sensor_power_on();
			break;
#endif
		case MOTION_IOCTL_MPU3050_I2C_READ:
#if 0
			printk("%s:ioctl cmd(MOTION_IOCTL_MPU3050_I2C_READ:0x%x)\n", __FUNCTION__, cmd);
#endif
			if (copy_from_user(&rwbuf, argp, sizeof(rwbuf))) {
				printk("[%s:%d copy_from_user - Fail(MOTION_IOCTL_MPU3050_I2C_READ)\n", __func__, __LINE__);
				return -EFAULT;
			}
			//printk("MOTION_IOCTL_MPU3050_I2C_READ addr : 0x%x\n",rwbuf[0]);

#if 0
			write_lock(&getbuflock);
			memcpy(&accelrwbuf[0], rwbuf, sizeof(rwbuf));
			write_unlock(&getbuflock);
#endif

			if (rwbuf[1] < 1) {
#if 0
				printk("EINVAL ERROR......I2C SLAVE MOTION_IOCTL_I2C_READ : rwbuf[1] < 1...\n");
#endif
				return -EINVAL;
			}

			if (rwbuf[0] == GYRO_I2C_SLAVE_ADDR) {

				//printk("############ (_0_)############ rwbuf[2]: %d(%x) / rwbuf[3]: %d(%x)/ rwbuf[1] : %d(%x)\n",rwbuf[2],rwbuf[2], rwbuf[3],rwbuf[3], rwbuf[1], rwbuf[1]);

				//jongik2.kim 20100910 i2c_fix [start]
				if (star_get_i2c_busy() == 0) {
					star_set_i2c_busy();
					NvGyroAccelI2CGetRegs(star_motion_dev->hOdmGyroAccel, rwbuf[2] ,&rwbuf[3] , rwbuf[1]);
					star_unset_i2c_busy();
				}
				//jongik2.kim 20100910 i2c_fix [end]
				if (ret < 0) {
#if DEBUG_GYRO
					printk("MOTION_IOCTL_I2C_READ : GYRO_I2C_SLAVE_ADDR Address ERROR[%d]\n",rwbuf[0]);
#endif
					return -EINVAL;
				}

				if (copy_to_user(argp, &rwbuf, sizeof(rwbuf))) {
#if DEBUG_GYRO
					printk("EINVAL ERROR.### GYRO ### I2C SLAVE MOTION_IOCTL_I2C_READ : rwbuf[1] < 1...\n");
#endif
					return -EFAULT;
				}
			/* Accel : jay.sim */
#if 0
			} else if (accelrwbuf[0] == 0x0F) {
				//printk("#### (_0_) #### accelrwbuf[2]: %d(%x) / accelrwbuf[3]: %d(%x)/ accelrwbuf[1] : %d(%x)\n",accelrwbuf[2],accelrwbuf[2], accelrwbuf[3],accelrwbuf[3], accelrwbuf[1], accelrwbuf[1]);
				//printk("######################## accel get(read) ##########################\n");
				if ((!accelrwbuf)) {
	#if DEBUG_GYRO
					printk("### EEROR #### accelrwbuf is NULL pointer \n");
	#endif
					return -1;
				} else {
					NvAccelerometerI2CGetRegsPassThrough (accelrwbuf[2] ,&accelrwbuf[3] , accelrwbuf[1]);
				}

				if (ret < 0) {
	#if DEBUG_GYRO
					printk("MOTION_IOCTL_I2C_READ : ACCEL_I2C_SLAVE_ADDR Address ERROR[%d]\n",accelrwbuf[0]);
	#endif
					return -EINVAL;
				}

				if (copy_to_user(argp, &accelrwbuf, sizeof(accelrwbuf))) {
	#if DEBUG_GYRO
					printk("EINVAL ERROR  ### ACCEL ## I2C SLAVE MOTION_IOCTL_I2C_READ : rwbuf[1] < 1...\n");
	#endif
					return -EFAULT;
				}
#endif
#if 0
			/* wkkim add to read compass */
			} else if (rwbuf[0] == 0x0E) {
				compassI2CGetRegs(rwbuf[2], &rwbuf[3], rwbuf[1]);
	#if DEBUG_GYRO
				printk("### COMPASS ### I2C SLAVE MOTION_IOCTL_I2C_READ rwbuf[1]:%d, rwbuf[2]:%d, rwbuf[3]:%d\n", rwbuf[1], rwbuf[2], rwbuf[3]);
	#endif
				if (copy_to_user(argp, &rwbuf, sizeof(rwbuf))) {
	#if DEBUG_GYRO
					printk("EINVAL ERROR.### COMPASS ### I2C SLAVE MOTION_IOCTL_I2C_READ : rwbuf[1] < 1...\n");
	#endif
					return -EFAULT;
				}
#endif	/* #if 0 */
			} else {
#if DEBUG_GYRO
				printk("......I2C SLAVE ADDRESS ERROR!!!...[0x%x]...\n",buf[0]);
#endif
				return -EINVAL;
			}
			break;
		case MOTION_IOCTL_MPU3050_I2C_WRITE:
#if 0
			printk("%s:ioctl cmd(MOTION_IOCTL_MPU3050_I2C_WRITE:0x%x)\n", __FUNCTION__, cmd);
#endif
			//printk(".............MOTION_IOCTL_MPU3050_I2C_WRITE................\n");
			if (copy_from_user(&rwbuf, argp, sizeof(rwbuf))) {
#if DEBUG_GYRO
				printk("EINVAL ERROR.....copy_from_user.I2C SLAVE MOTION_IOCTL_I2C_WRITE \n");
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
#if DEBUG_GYRO
				printk("MOTION_IOCTL_WRITE ..length ERROR!!![%d].....\n",rwbuf[1]);
#endif
				return -EINVAL;
			}

			if (rwbuf[0] == GYRO_I2C_SLAVE_ADDR) {
				//ret = mpu3050_i2c_write(&rwbuf[2],rwbuf[1]);
				NvGyroAccelI2CSetRegs(star_motion_dev->hOdmGyroAccel, rwbuf[2] ,&rwbuf[3] , rwbuf[1] - 1);
				if (ret < 0) {
#if DEBUG_GYRO
					printk("MOTION_IOCTL_WRITE  : GYRO_I2C_SLAVE_ADDR Address ERROR[%d]\n", rwbuf[0]);
#endif
					return -EINVAL;
				}
#if 0	/* Accel : jay.sim */
			} else if (rwbuf[0] == 0x0F) {
				//ret = kxtf9_i2c_write(&rwbuf[2],rwbuf[1]);
				//printk("(_6_)rwbuf[2]: %d(%x) /  rwbuf[1] : %d(%x)\n",rwbuf[2],rwbuf[2], rwbuf[1], rwbuf[1]);
				//			printk("######################## accel set(write) ##########################\n");
				NvAccelerometerI2CSetRegsPassThrough(rwbuf[2] ,&rwbuf[3] , rwbuf[1]-1);
				//printk("(_7_) rwbuf[3]: %d(%x) \n", rwbuf[3],rwbuf[3]);
				if (ret < 0){
#if DEBUG_GYRO
					printk("[KXTF9] MOTION_IOCTL_WRITE  : ACCEL_I2C_SLAVE_ADDR ERROR[%d]\n",rwbuf[0]);
#endif
					return -EINVAL;
				}
#endif
#if 0
			/* wkkim add to set compass */
			} else if (rwbuf[0] == 0x0E) {
				compassI2CSetRegs(rwbuf[2], &rwbuf[3], rwbuf[1] - 1);
	#if DEBUG_GYRO
				printk("### COMPASS ### I2C SLAVE MOTION_IOCTL_I2C_WRITE rwbuf[1]:%d, rwbuf[2]:%d, rwbuf[3]:%d\n", rwbuf[1], rwbuf[2], rwbuf[3]);
	#endif
				if (ret < 0) {
	#if DEBUG_GYRO
					printk("MOTION_IOCTL_WRITE  : GYRO_I2C_SLAVE_ADDR Address ERROR[%d]\n",rwbuf[0]);
	#endif
					return -EINVAL;
				}
#endif /* #if 0 */
			} else {
#if DEBUG_GYRO
				printk("......I2C SLAVE ADDRESS ERROR!!!...[0x%x]...\n",buf[0]);
#endif
				return -EINVAL;
			}
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

/* Accel : jay.sim */
#if 0
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
			xyz[0] = atomic_read(&accel_x);
			xyz[1] = atomic_read(&accel_y);
			xyz[2] = atomic_read(&accel_z);

			//printk("K: Gyro-accel.c ## ACCEL ## xyz[0]: %d ;	xyz[1]: %d; xyz[2]: %d \n",xyz[0], xyz[1], xyz[2]);
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
			/*DY*///printk("K: Gyro-accel.c ## ACCEL ## xyz[ %d: 0x%x ] ;	xyz[ %d: 0x%x ]; xyz[ %d: 0x%x ] \n",xyz[0], xyz[0], xyz[1], xyz[1], xyz[2], xyz[2]);

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

#if 0
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
#endif
#endif

static NvS32 star_motion_resume(struct platform_device *pdev);
static NvS32 star_motion_suspend(struct platform_device *pdev);

#ifdef CONFIG_HAS_EARLYSUSPEND
static void gyro_early_suspend(struct early_suspend *es)
{
#if DEBUG_GYRO
	printk(D_SENSOR, "--->> Gyro early suspend\n");
#endif
	//mdelay(200);
	star_motion_suspend(NULL);
	return;

}

static void gyro_late_resume(struct early_suspend *es)
{
#if DEBUG_GYRO
	printk(D_SENSOR, "--->> Gyro early resume\n");
#endif
	star_motion_resume(NULL);
	return;
}
#endif

/* jay.sim */
/* wkkim add to read compass */
//NvBool compassI2CSetRegs(NvU8 offset, NvU8* value, NvU32 len);
//NvBool compassI2CGetRegs(NvU8 offset, NvU8* value, NvU32 len);

//extern int AMI304_Reset_Init();
//extern int AMI304_Init(int mode);

/** Function to close the ODM device. This function will help in switching
 * between power modes
 */
void close_odm_gyro_accel(void)
{
#if DEBUG_GYRO
	printk("## %s is called\n", __func__);
#endif
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

#if DEBUG_GYRO
	printk("## %s is called\n", __func__);
#endif
	err = NvOdmGyroAccelOpen(&(star_motion_dev->hOdmGyroAccel));
	if (!err) {
		err = -ENODEV;
		printk("%s: Open failed\n", __func__);
		return err;
	}

	return err;
}

/* Accel : jay.sim */
#if 0
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
#if DEBUG_GYRO
	printk("[%s:%d] Accelerometer Sensor \n", __FUNCTION__, __LINE__);
#endif
	tempbuf[3] = 0;
	tempbuf[1] = 1;
	NvAccelerometerI2CGetRegsPassThrough(ACCEL_CTRL_REG3, &tempbuf[3], tempbuf[1]);

	tempbuf[3] |= 1 << 7;
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

#if DEBUG_GYRO
	printk("[%s:%d] Accelerometer Sensor \n", __FUNCTION__, __LINE__);
#endif

}
#endif

/*---------------------------------------------------------------------------
  motion_send_event function
  ---------------------------------------------------------------------------*/
void motion_send_accel_detection(int accelx,int accely,int accelz)
{
	//printk("[Gyro_accel][%s:%d] %d %d %d\n",__FUNCTION__, __LINE__,accelx , accely, accelz );

	if (atomic_read(&accel_flag)) {
		input_report_abs(star_motion_dev->input_dev,ABS_X, accelx);
		input_report_abs(star_motion_dev->input_dev,ABS_Y, accely);
		input_report_abs(star_motion_dev->input_dev,ABS_Z, accelz);
		input_sync(star_motion_dev->input_dev);
	}
}

void motion_send_tilt_detection(int yaw,int pitch,int roll)
{
	/*DY2*///printk("[Gyro_accel][%s:%d]\n",__FUNCTION__, __LINE__);

	if (atomic_read(&tilt_flag)) {
		/*DY*///printk("[Gyro_accel][%s:%d]\n",__FUNCTION__, __LINE__);
		input_report_rel(star_motion_dev->input_dev,REL_RX, yaw);
		input_report_rel(star_motion_dev->input_dev,REL_RY, pitch);
		input_report_rel(star_motion_dev->input_dev,REL_RZ, roll);

		input_sync(star_motion_dev->input_dev);
	}
}

void motion_send_composite_detection(int *value)
{
	int buf[12] = {0,};

	memcpy(buf, value, sizeof(int) * 12);

	//	printk("composite %d %d %d ... %d %d %d\n", buf[0],buf[1],buf[2],buf[9],buf[10],buf[11]);
	if (atomic_read(&tilt_flag))
	{
		input_report_abs(star_motion_dev->input_dev, ABS_GAS, buf[0]);
		input_report_abs(star_motion_dev->input_dev, ABS_HAT1X, buf[1]);
		input_report_abs(star_motion_dev->input_dev, ABS_HAT1Y, buf[2]);
		input_report_abs(star_motion_dev->input_dev, ABS_HAT2X, buf[3]);
		input_report_abs(star_motion_dev->input_dev, ABS_HAT2Y, buf[4]);
		input_report_abs(star_motion_dev->input_dev, ABS_HAT3X, buf[5]);
		input_report_abs(star_motion_dev->input_dev, ABS_HAT3Y, buf[6]);
		input_report_abs(star_motion_dev->input_dev, ABS_TILT_X, buf[7]);
		input_report_abs(star_motion_dev->input_dev, ABS_TILT_Y, buf[8]);
		input_report_abs(star_motion_dev->input_dev, ABS_TOOL_WIDTH, buf[9]);
		input_report_abs(star_motion_dev->input_dev, ABS_VOLUME, buf[10]);
		input_report_abs(star_motion_dev->input_dev, ABS_MISC, buf[11]);
		//input_report_rel(star_motion_dev->input_dev,REL_MISC,buf[12]);

		input_sync(star_motion_dev->input_dev);
	}
}

void motion_send_gyro_detection(int gyro_x,int gyro_y,int gyro_z)
{
	/*DY2*///printk(D_SENSOR,"[Gyro_accel][%s:%d]\n",__FUNCTION__, __LINE__);

	if (atomic_read(&gyro_flag)) {
		/*DY*///printk("[Gyro_accel][%s:%d]\n",__FUNCTION__, __LINE__);
		input_report_rel(star_motion_dev->input_dev,REL_Z, gyro_x);
		input_report_rel(star_motion_dev->input_dev,REL_MISC, gyro_y);
		input_report_rel(star_motion_dev->input_dev,REL_MAX, gyro_z);

		input_sync(star_motion_dev->input_dev);
	}
}

void motion_send_tap_detection(int type,int direction)
{
#if DEBUG_GYRO
	/*DY2*/printk(D_SENSOR,"[Gyro_accel][%s:%d]\n", __FUNCTION__, __LINE__);
#endif
	if(atomic_read(&tap_flag)) {
#if DEBUG_GYRO
		/*DY*/printk(D_SENSOR,"[Gyro_accel][%s:%d]\n", __FUNCTION__, __LINE__);
#endif
		/*Test*///type= 111; direction = 222;
		input_report_rel(star_motion_dev->input_dev, REL_X, type);
		input_report_rel(star_motion_dev->input_dev, REL_Y, direction);

		input_sync(star_motion_dev->input_dev);
	}
}

void motion_send_flip_detection(int value)
{
#if DEBUG_GYRO
	/*DY2*/printk(D_SENSOR,"[Gyro_accel][%s:%d]\n", __FUNCTION__, __LINE__);
#endif
	if (atomic_read(&flip_flag)) {
#if DEBUG_GYRO
		/*DY*/printk(D_SENSOR,"[Gyro_accel][%s:%d]\n", __FUNCTION__, __LINE__);
#endif
		input_report_rel(star_motion_dev->input_dev, REL_WHEEL, value);
		input_sync(star_motion_dev->input_dev);
	}
}

void motion_send_shake_detection(int value)
{
#if DEBUG_GYRO
	/*DY2*/printk(D_SENSOR,"[Gyro_accel][%s:%d]\n", __FUNCTION__, __LINE__);
#endif
	if (atomic_read(&shake_flag)) {
#if DEBUG_GYRO
		/*DY*/printk(D_SENSOR,"[Gyro_accel][%s:%d]\n", __FUNCTION__, __LINE__);
#endif
		input_report_rel(star_motion_dev->input_dev, REL_HWHEEL, value);
		input_sync(star_motion_dev->input_dev);
	}
}

void motion_send_snap_detection(int direction)
{
#if DEBUG_GYRO
	/*DY2*/printk(D_SENSOR,"[Gyro_accel][%s:%d]\n", __FUNCTION__, __LINE__);
#endif
	if (atomic_read(&snap_flag)) {
#if DEBUG_GYRO
		/*DY*/printk(D_SENSOR,"[Gyro_accel][%s:%d]\n", __FUNCTION__, __LINE__);
#endif
		input_report_rel(star_motion_dev->input_dev,REL_DIAL,direction);
		input_sync(star_motion_dev->input_dev);
	}
}

/*---------------------------------------------------------------------------
  work function
  ---------------------------------------------------------------------------*/
/* Accel : jay.sim */
#if 1
static void motion_accel_work_func(struct work_struct *work)
{
	int current_x = 0, current_y = 0, current_z = 0;

	current_x = atomic_read(&accel_x);
	current_y = atomic_read(&accel_y);
	current_z = atomic_read(&accel_z);

	motion_send_accel_detection(current_x, current_y, current_z);
}
#endif

static void motion_tilt_work_func(struct work_struct *work)
{
	int current_yaw = 0, current_pitch = 0, current_roll = 0;
	int data[12] = {0,};
	int i = 0;

	current_yaw = atomic_read(&tilt_yaw);
	current_pitch = atomic_read(&tilt_pitch);
	current_roll = atomic_read(&tilt_roll);

	//printk("[motion_tilt_work_func] current_roll=[%d], current_pitch=[%d], current_yaw=[%d] \n",current_roll,current_pitch,current_yaw);

	motion_send_tilt_detection(current_yaw, current_pitch, current_roll);

	for (i = 0; i < 12; i++) {
		data[i] = atomic_read(&composite[i]);
	}
	motion_send_composite_detection(data);

}

static void motion_gyro_work_func(struct work_struct *work)
{
	int current_x = 0, current_y = 0, current_z = 0;

	current_x = atomic_read(&gyro_x);
	current_y = atomic_read(&gyro_y);
	current_z = atomic_read(&gyro_z);

	motion_send_gyro_detection(current_x,current_y,current_z);

}

#if 1	/* jay.sim */
static void motion_compass_work_func(struct work_struct *work)
{
	int mag_val[3];

	mag_val[0]= atomic_read(&mag_x);
	mag_val[1]= atomic_read(&mag_y);
	mag_val[2]= atomic_read(&mag_z);
	/* TODO : compass enable check */
	if (atomic_read(&compass_flag))
		magnetic_input_report(mag_val);
}
#endif

/*---------------------------------------------------------------------------
  motion polling timer
  ---------------------------------------------------------------------------*/
/* Accel : jay.sim */
#if 1
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
#endif

static enum hrtimer_restart motion_tilt_timer_func(struct hrtimer *timer)
{
	unsigned long  polling_time;

	if (atomic_read(&tilt_flag)) {
		queue_work(star_motion_dev->tilt_wq, &star_motion_dev->tilt_work);

		polling_time = atomic_read(&tilt_delay);
		//printk("[Gyro_accel][%s:%d]\n",__FUNCTION__, __LINE__);
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

#if 1	/* jay.sim */
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
#endif


/*---------------------------------------------------------------------------
  sensor enable/disable (Sensor HAL)
  ---------------------------------------------------------------------------*/
/* Accel : jay.sim */
#if 1
static ssize_t motion_accel_onoff_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	u32    val;
	val = atomic_read(&accel_flag);
	return sprintf(buf, "%d\n",val);
}

static ssize_t motion_accel_onoff_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
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
#endif

static ssize_t motion_tilt_onoff_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	u32    val;
	val = atomic_read(&tilt_flag);
	return sprintf(buf, "%d\n",val);
}

static ssize_t motion_tilt_onoff_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u32    val;
	val = simple_strtoul(buf, NULL, 10);
	//printk("[motion_set_tilt_onoff_store]  flag [%d]\n",val);

	if (val) {
		atomic_set(&tilt_flag, 1);
	} else {
		atomic_set(&tilt_flag, 0);
	}

	return count;
}

static ssize_t motion_gyro_onoff_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	u32    val;
	val = atomic_read(&gyro_flag);
	return sprintf(buf, "%d\n",val);
}

static ssize_t motion_gyro_onoff_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u32    val;
	val = simple_strtoul(buf, NULL, 10);
	//printk("[motion_gyro_onoff_store] gyro_flag [%d]\n",val);

	if (val) {
		atomic_set(&gyro_flag, 1);
	} else {
		atomic_set(&gyro_flag, 0);
	}

	return count;
}

static ssize_t motion_tap_onoff_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	u32    val;
	val = atomic_read(&tap_flag);
	return sprintf(buf, "%d\n",val);
}

static ssize_t motion_tap_onoff_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u32    val;
	val = simple_strtoul(buf, NULL, 10);
	//printk("[motion_set_tap_onoff_store] tap.... flag [%d]\n",val);

	if (val) {
		atomic_set(&tap_flag, 1);
	} else {
		atomic_set(&tap_flag, 0);
	}

	return count;
}

static ssize_t motion_flip_onoff_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	u32    val;
	val = atomic_read(&flip_flag);
	return sprintf(buf, "%d\n",val);
}

static ssize_t motion_flip_onoff_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u32    val;
	val = simple_strtoul(buf, NULL, 10);
	//printk("[motion_set_flip_onoff_store]  flag [%d]\n",val);

	if (val) {
		atomic_set(&flip_flag, 1);
	} else {
		atomic_set(&flip_flag, 0);
	}

	return count;
}

static ssize_t motion_shake_onoff_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	u32    val;
	val = atomic_read(&shake_flag);
	return sprintf(buf, "%d\n",val);
}

static ssize_t motion_shake_onoff_store(struct device *dev,  struct device_attribute *attr,const char *buf, size_t count)
{
	u32    val;
	val = simple_strtoul(buf, NULL, 10);
	//printk("[motion_set_shake_onoff_store]  flag [%d]\n",val);

	if (val) {
		atomic_set(&shake_flag, 1);
	} else {
		atomic_set(&shake_flag, 0);
	}

	return count;
}

static ssize_t motion_snap_onoff_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	u32    val;
	val = atomic_read(&snap_flag);
	return sprintf(buf, "%d\n", val);
}

static ssize_t motion_snap_onoff_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u32    val;
	val = simple_strtoul(buf, NULL, 10);
	//printk("[motion_set_snap_onoff_store]  flag [%d]\n",val);

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
/* Accel : jay.sim */
#if 1
static ssize_t motion_accel_delay_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	u32    val;
	val = atomic_read(&accel_delay);
	return sprintf(buf, "%d\n",val);
}

static ssize_t motion_accel_delay_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u32    val;
	unsigned long current_delay = 100;

	val = simple_strtoul(buf, NULL, 10);
	current_delay = (short)val;
#if DEBUG_GYRO
	printk(D_SENSOR,"[motion_accel_delay_store]  flag [%d] current_delay[%ld]\n",val, current_delay);
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
#endif

static ssize_t motion_tilt_delay_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	u32    val;
	val = atomic_read(&tilt_delay);
	return sprintf(buf, "%d\n",val);
}

static ssize_t motion_tilt_delay_store(struct device *dev,  struct device_attribute *attr,const char *buf, size_t count)
{
	u32     val;
	unsigned long   current_delay = 100;

	val = simple_strtoul(buf, NULL, 10);
	current_delay = (short)val;

#if DEBUG_GYRO
	printk(D_SENSOR, "[motion_set_tilt_delay_store]  val [%d] current_delay[%ld]\n", val, current_delay);
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

static ssize_t motion_gyro_delay_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	u32    val;
	val = atomic_read(&gyro_delay);
	return sprintf(buf, "%d\n",val);
}

ssize_t motion_gyro_delay_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u32    val;
	unsigned long current_delay = 100;

	val = simple_strtoul(buf, NULL, 10);
	current_delay = (short)val;

#if DEBUG_GYRO
	printk(D_SENSOR, "[motion_gyro_delay_store]  val [%d] current_delay[%ld]\n",val,current_delay);
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

static ssize_t show_bootcomplete_flag(struct device *dev, struct device_attribute *attr, char *buf)
{
	u32    val;
	val = atomic_read(&boot_flag);
	return sprintf(buf, "%d\n",val);
}

ssize_t store_bootcomplete_flag(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u32 val;
	val = simple_strtoul(buf, NULL, 10);

	if (val) {
		atomic_set(&boot_flag, 1);
		sensors_wake_up_now();
		printk("%s is called, bootcomplete %d\n", __func__, val);
	} else {
		atomic_set(&boot_flag, 0);
	}
	return count;
}

static DEVICE_ATTR(accel_onoff, 0666, motion_accel_onoff_show, motion_accel_onoff_store);
static DEVICE_ATTR(tilt_onoff, 0666, motion_tilt_onoff_show, motion_tilt_onoff_store);
static DEVICE_ATTR(gyro_onoff, 0666, motion_gyro_onoff_show, motion_gyro_onoff_store);
static DEVICE_ATTR(tap_onoff, 0666, motion_tap_onoff_show, motion_tap_onoff_store);
static DEVICE_ATTR(flip_onoff, 0666, motion_flip_onoff_show, motion_flip_onoff_store);
static DEVICE_ATTR(shake_onoff,0666, motion_shake_onoff_show, motion_shake_onoff_store);
static DEVICE_ATTR(snap_onoff, 0666, motion_snap_onoff_show, motion_snap_onoff_store);

static DEVICE_ATTR(accel_delay, 0666, motion_accel_delay_show, motion_accel_delay_store);
static DEVICE_ATTR(tilt_delay, 0666, motion_tilt_delay_show, motion_tilt_delay_store);
static DEVICE_ATTR(gyro_delay, 0666, motion_gyro_delay_show, motion_gyro_delay_store);

static DEVICE_ATTR(bootcomplete, 0666, show_bootcomplete_flag, store_bootcomplete_flag);

static struct attribute *star_motion_attributes[] = {
	&dev_attr_accel_onoff.attr,
	&dev_attr_gyro_onoff.attr,
	&dev_attr_tilt_onoff.attr,
	&dev_attr_tap_onoff.attr,
	&dev_attr_shake_onoff.attr,
	&dev_attr_snap_onoff.attr,
	&dev_attr_flip_onoff.attr,
	&dev_attr_tilt_delay.attr,
	&dev_attr_accel_delay.attr,
	&dev_attr_gyro_delay.attr,
	&dev_attr_bootcomplete.attr,
	NULL
};

static const struct attribute_group star_motion_group = {
	.attrs = star_motion_attributes,
};


/*---------------------------------------------------------------------------
  platform device
  ---------------------------------------------------------------------------*/
static int __init star_motion_probe(struct platform_device *pdev)
{
	int err = 0;
	unsigned char value = 0;
	struct device *dev = &pdev->dev;

#if DEBUG_GYRO
	printk("[%s:%d] Started \n", __FUNCTION__, __LINE__);
#endif

	star_motion_dev = kzalloc(sizeof(*star_motion_dev), GFP_KERNEL);
	if (!star_motion_dev)
		goto alloc_mem_fail;

#if DEBUG_GYRO
	printk("%s: Allocate Memory for Gyro Device \n", __func__);
#endif

	platform_set_drvdata(pdev, star_motion_dev);

	err = open_def_odm_gyro_accel();
	if (!err) {
#if DEBUG_GYRO
		printk("%s :Error: open_def_odm_gyro_accel\n");
#endif
		goto open_odm_fail;
	}
#if DEBUG_GYRO
	printk("%s: Open NVOdm for Gyro Device \n", __func__);
#endif

	//  Read WHO AM I
	value = 0;
	NvGyroAccelI2CGetRegs(star_motion_dev->hOdmGyroAccel, MPU3050_GYRO_I2C_WHO_AM_I, &value, 1);
#if DEBUG_GYRO
	printk("%s: MPU3050_GYRO_I2C_WHO_AM_I : %x \n", value);
#endif
	// Read Product ID
	value = 0;
	NvGyroAccelI2CGetRegs(star_motion_dev->hOdmGyroAccel, MPU3050_GYRO_I2C_PRODUCT_ID, &value, 1);
#if DEBUG_GYRO
	printk("%s: MPU3050_GYRO_I2C_PRODUCT_ID : %x\n", value);
#endif

	/* Common : jay.sim */
	/*---------------------------------------------------------------------------
	  register input device
	  ---------------------------------------------------------------------------*/
	star_motion_dev->input_dev = input_allocate_device();
	if (star_motion_dev->input_dev == NULL) {
#if DEBUG_GYRO
		printk(KERN_ERR"star_motion_sesnor_probe: input_allocate_device (1) failed\n");
#endif
		goto alloc_input_dev_fail;
	}

	star_motion_dev->input_dev->name = STAR_MOTION_INPUT_NAME;

#if 1
	set_bit(EV_SYN,star_motion_dev->input_dev->evbit);
	set_bit(EV_REL,star_motion_dev->input_dev->evbit);

	set_bit(REL_Z,star_motion_dev->input_dev->relbit);  	// gyro_x
	set_bit(REL_MISC,star_motion_dev->input_dev->relbit);  // gyro_y
	set_bit(REL_MAX,star_motion_dev->input_dev->relbit);  	// gyro_z
#endif

	set_bit(EV_ABS,star_motion_dev->input_dev->evbit);
	input_set_abs_params(star_motion_dev->input_dev, ABS_X, -2000, 2000, 0, 0); /* x-axis acceleration */
	input_set_abs_params(star_motion_dev->input_dev, ABS_Y, -2000, 2000, 0, 0); /* y-axis acceleration */
	input_set_abs_params(star_motion_dev->input_dev, ABS_Z, -2000, 2000, 0, 0); /* z-axis acceleration */
	input_set_abs_params(star_motion_dev->input_dev, ABS_GAS, 0, 0, 0, 0);
	input_set_abs_params(star_motion_dev->input_dev, ABS_HAT1X, 0, 0, 0, 0);
	input_set_abs_params(star_motion_dev->input_dev, ABS_HAT1Y, 0, 0, 0, 0);
	input_set_abs_params(star_motion_dev->input_dev, ABS_HAT2X, 0, 0, 0, 0);
	input_set_abs_params(star_motion_dev->input_dev, ABS_HAT2Y, 0, 0, 0, 0);
	input_set_abs_params(star_motion_dev->input_dev, ABS_HAT3X, 0, 0, 0, 0);
	input_set_abs_params(star_motion_dev->input_dev, ABS_HAT3Y, 0, 0, 0, 0);
	input_set_abs_params(star_motion_dev->input_dev, ABS_TILT_X, 0, 0, 0, 0);
	input_set_abs_params(star_motion_dev->input_dev, ABS_TILT_Y, 0, 0, 0, 0);
	input_set_abs_params(star_motion_dev->input_dev, ABS_TOOL_WIDTH, 0, 0, 0, 0);
	input_set_abs_params(star_motion_dev->input_dev, ABS_VOLUME, 0, 0, 0, 0);
	input_set_abs_params(star_motion_dev->input_dev, ABS_MISC, 0, 0, 0, 0);

	set_bit(REL_X, star_motion_dev->input_dev->relbit);  // TAP - Type
	set_bit(REL_Y, star_motion_dev->input_dev->relbit);  // TAP - Direction
	set_bit(REL_RX, star_motion_dev->input_dev->relbit);  // TILT - Yaw
	set_bit(REL_RY, star_motion_dev->input_dev->relbit);  // TILT - Pitch
	set_bit(REL_RZ, star_motion_dev->input_dev->relbit);  // TILT - Roll
	set_bit(REL_HWHEEL, star_motion_dev->input_dev->relbit); // SHAKE
	set_bit(REL_DIAL, star_motion_dev->input_dev->relbit);   // SNAP - Direction
	set_bit(REL_WHEEL, star_motion_dev->input_dev->relbit);  // FLIP

	err = input_register_device(star_motion_dev->input_dev);
	if (err) {
#if DEBUG_GYRO
		printk(KERN_ERR"star_motion_sesnor_probe: input_allocate_device (1) failed \n");
#endif
		goto register_input_dev_fail;
	}
#if DEBUG_GYRO
	printk("%s: Register Gyro Input Device \n", __func__);
#endif

	/*---------------------------------------------------------------------------
	  init. sysfs
	  ---------------------------------------------------------------------------*/
	if ((err = sysfs_create_group(&dev->kobj, &star_motion_group))) {
#if DEBUG_GYRO
		printk("[motion_sensor] sysfs_create_group FAIL \n");
#endif
		goto create_sysfs_fail;
	}
#if DEBUG_GYRO
	printk("%s: Create sysfs attribute for Gyro Input Device \n", __func__);
#endif

	/*---------------------------------------------------------------------------
	  register misc device
	  ---------------------------------------------------------------------------*/
	err = misc_register(&star_motion_misc_device);
	if (err) {
#if DEBUG_GYRO
		printk(KERN_ERR"star_motion_misc_device register failed\n");
#endif
		goto register_misc_fail;
	}
#if DEBUG_GYRO
	printk("%s: Register Misc. Device for 'motion_daemon' device file(%lu) \n", __func__);
#endif

	/*---------------------------------------------------------------------------
	  INIT_WORK
	  ---------------------------------------------------------------------------*/
#if 1
	INIT_WORK(&star_motion_dev->accel_work, motion_accel_work_func);
	INIT_WORK(&star_motion_dev->tilt_work, motion_tilt_work_func);
	INIT_WORK(&star_motion_dev->gyro_work, motion_gyro_work_func);
	INIT_WORK(&star_motion_dev->compass_work, motion_compass_work_func);

	/*---------------------------------------------------------------------------
	  init. workqueue
	  ---------------------------------------------------------------------------*/
	star_motion_dev->tilt_wq = create_singlethread_workqueue("motion_tilt_wq");
	if (!star_motion_dev->tilt_wq) {
#if DEBUG_GYRO
		printk("[motion_sensor] couldn't create tilt work queue\n");
#endif
		goto create_tilt_wq_fail;
	}

#if DEBUG_GYRO
	printk("%s: Create workqueue for motion_tilt (%lu) \n", __func__);
#endif

	star_motion_dev->gyro_wq = create_singlethread_workqueue("motion_gyro_wq");
	if (!star_motion_dev->gyro_wq) {
#if DEBUG_GYRO
		printk("[motion_sensor] couldn't create gyro work queue\n");
#endif
		goto create_gyro_wq_fail;
	}
#if DEBUG_GYRO
	printk("%s: Create workqueue for motion_gyro (%lu) \n", __func__);
#endif

	star_motion_dev->accel_wq = create_singlethread_workqueue("motion_accel_wq");
	if (!star_motion_dev->accel_wq) {
#if DEBUG_GYRO
		lprintk("[motion_sensor] couldn't create accel work queue\n");
#endif
		goto create_accel_wq_fail;
	}

	star_motion_dev->compass_wq = create_singlethread_workqueue("motion_compass_wq");
	if (!star_motion_dev->compass_wq) {
#if DEBUG_GYRO
		lprintk("[motion_sensor] couldn't create accel work queue\n");
#endif
		goto create_compass_wq_fail;
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

#endif

#if DEBUG_GYRO
	printk("%s: Register hrtimer for tilt / gyro (%lu) \n", __func__);
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND // wkkim : temporary early suspend apply
	early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	early_suspend.suspend = gyro_early_suspend;
	early_suspend.resume = gyro_late_resume;
	register_early_suspend(&early_suspend);
#endif

	atomic_set(&gyro_init, 1);
	sensors_wake_up_now();
	printk("%s gyro init done\n", __func__);

	return 0;

create_compass_wq_fail:
	destroy_workqueue(star_motion_dev->accel_wq);
create_accel_wq_fail:
	destroy_workqueue(star_motion_dev->gyro_wq);
create_gyro_wq_fail:
	destroy_workqueue(star_motion_dev->tilt_wq);
create_tilt_wq_fail:
	misc_deregister(&star_motion_misc_device);
register_misc_fail:
	sysfs_remove_group(&dev->kobj, &star_motion_group);
create_sysfs_fail:
	input_unregister_device(star_motion_dev->input_dev);
register_input_dev_fail:
	input_free_device(star_motion_dev->input_dev);
alloc_input_dev_fail:
	close_odm_gyro_accel();
open_odm_fail:
	kfree(star_motion_dev);
alloc_mem_fail:

	return err;

}
static int star_motion_remove(struct platform_device *pdev)
{
	//struct device *dev = &pdev->dev;
	input_unregister_device(star_motion_dev->input_dev);
	if (star_motion_dev->tilt_wq)
		destroy_workqueue(star_motion_dev->tilt_wq);

	return 0;
}

void mpu3050_sleep_mode(void)
{
	unsigned char value = 0;
	unsigned char buf[5] = {0,};

	//mpu3050_read_reg(mpu3050_i2c_client,MPU3050_GYRO_I2C_PWR_MGM,&value);
	NvGyroAccelI2CGetRegs(star_motion_dev->hOdmGyroAccel, MPU3050_GYRO_I2C_PWR_MGM ,&value , 1 );
#if DEBUG_GYRO
	printk("--->> %s:%d PWR Reg : 0x%x\n",__func__,__LINE__,value);
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
#if DEBUG_GYRO
		printk("%s:%d PWR Reg : 0x%x\n", __func__, __LINE__, value);
#endif
	}
}

void mpu3050_sleep_wake_up(void)
{
	unsigned char value = 0;
	unsigned char buf[5] = {0,};

	//mpu3050_read_reg(mpu3050_i2c_client,MPU3050_GYRO_I2C_PWR_MGM,&value);
#if DEBUG_GYRO
	printk("--->> %s:%d PWR Reg : 0x%x, jiffies : %u\n", __func__, __LINE__, value, jiffies);
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

static NvS32 star_motion_resume(struct platform_device *pdev)
{
	atomic_set(&suspend_flag, 0);
	return 0;
}

static NvS32 star_motion_suspend(struct platform_device *pdev)
{
	atomic_set(&suspend_flag, 1);
	return 0;
}

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

#if DEBUG_GYRO
	printk("[MPU3050] ## [%s:%d]\n", __FUNCTION__, __LINE__);
#endif
	rwlock_init(&getbuflock);
//	memset(&accelrwbuf[0], 0, sizeof(unsigned char) * 200);
	memset(&rwbuf[0], 0, sizeof(unsigned char) * 200);

	err = platform_driver_register(&star_motion_driver);

	return 0;
}
static void __exit star_motion_exit(void)
{
#if DEBUG_GYRO
	printk(KERN_INFO "[MPU3050] lge star_motion_exit was unloaded!\nHave a nice day!\n");
#endif
	platform_driver_unregister(&star_motion_driver);
	return;
}

module_init(star_motion_init);
module_exit(star_motion_exit);

MODULE_AUTHOR("LG Electronics");
MODULE_DESCRIPTION("Gyro-accel Driver for Star");
MODULE_LICENSE("GPL");

#if 0
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
	printk("[MPU3050] MPU3050_GYRO_I2C_WHO_AM_I : %x\n",value);

	// Read Product ID
	value = 0;
	//status = mpu3050_read_reg(mpu3050_i2c_client,MPU3050_GYRO_I2C_PRODUCT_ID,&value);
	NvGyroAccelI2CGetRegs(star_motion_dev->hOdmGyroAccel, MPU3050_GYRO_I2C_PRODUCT_ID ,&value , 1 );
	printk("[MPU3050] MPU3050_GYRO_I2C_PRODUCT_ID : %x\n",value);
	/*---------------------------------------------------------------------------------------*/
#endif

}

void mpu3050_i2c_through_pass(int benable)
{
	unsigned char value = 0;
	unsigned char buf;
	int status = 0;

	NvGyroAccelI2CGetRegs(star_motion_dev->hOdmGyroAccel, MPU3050_GYRO_I2C_USER_CTRL ,&value , 1 );
	if (status < 0) {
#if DEBUG_GYRO
		printk("[MPU3050] MPU3050_GYRO_I2C_USER_CTRL. i2c ERROR: 0x%x................................\n",value);
#endif
		return ;
	}

	if (benable == MPU3050_BYPASS_MODE_ON) {
#if DEBUG_GYRO
		printk(D_SENSOR,"[MPU3050] bypass on\n");
#endif
		//if(value & 0x20)
		value &= ~(0x20);

		NvGyroAccelI2CSetRegs(star_motion_dev->hOdmGyroAccel, MPU3050_GYRO_I2C_USER_CTRL, &value, 1);

		/* Accel WHO_AM_I bucause of gyro chip problem */
		NvAccelerometerI2CSetRegsPassThrough(0x0F, &buf, 0);

	} else { // bypass off
#if DEBUG_GYRO
		printk(D_SENSOR,"[MPU3050] bypass off\n");
#endif
		//if(!(value & 0x20))
		value |= 0x20;

		NvGyroAccelI2CSetRegs(star_motion_dev->hOdmGyroAccel, MPU3050_GYRO_I2C_USER_CTRL, &value, 1);
	}

#if 0  /* Delete MPU reset. It is not necessary */
	/* MPU reset */
	value |= 0x08;
	NvGyroAccelI2CSetRegs(star_motion_dev->hOdmGyroAccel, MPU3050_GYRO_I2C_USER_CTRL ,&value , 1 );
#endif

}


#endif
