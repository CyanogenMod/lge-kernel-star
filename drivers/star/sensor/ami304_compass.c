/* drivers/i2c/chips/ami304.c - AMI304 compass driver
 *
 * Copyright (C) 2009 AMIT Technology Inc.
 * Author: Kyle Chen <sw-support@amit-inc.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/kthread.h>
#include <linux/device.h>
#include <linux/kernel.h>
//#include <linux/tegra_devices.h>

#include <nvodm_services.h>

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include "ami304.h"
#include <linux/kobject.h>
#include <nvodm_compass.h>
#include "mach/lprintk.h"

#include "lge_sensor_verify.h"

#define DEBUG_AMI304 0
#define DEBUG_IOCTL 0
#define DEBUG_HAL_IOCTL 0
#define DEBUG_DAEMON_IOCTL 0
#define DEBUG_DATA 0

#define AMI_ORIENTATION_SENSOR		0
#define AMI_MAGNETIC_FIELD_SENSOR	1
#define AMI_ACCELEROMETER_SENSOR		2

static struct i2c_client *ami304_i2c_client = NULL;

/* Addresses to scan */
//static unsigned short normal_i2c[] = { AMI304_I2C_ADDRESS, I2C_CLIENT_END };
/* Insmod parameters */
//I2C_CLIENT_INSMOD_1(ami304);
//static int ami304_i2c_attach_adapter(struct i2c_adapter *adapter);
//static int ami304_i2c_detect(struct i2c_adapter *adapter, int address, int kind);
//static int ami304_i2c_detach_client(struct i2c_client *client);

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
static struct early_suspend    early_suspend; //wkkim : temporary early suspend apply

static NvS32 ami304_resume(struct platform_device *pdev);
static NvS32 ami304_suspend(struct platform_device *pdev);
static void compass_early_suspend(struct early_suspend *es)
{
#if DEBUG_AMI304
	printk("[Sensor] --->> compass early suspend\n");
#endif
	ami304_suspend(NULL);
	return;
}

static void compass_late_resume(struct early_suspend *es)
{
#if DEBUG_AMI304
	printk("[Sensor] --->> compass early resume\n");
#endif
	ami304_resume(NULL);
	return;
}
#endif

static struct input_dev *compass_input_dev;

struct _ami302_data {
	rwlock_t lock;
	int mode;
	int rate;
	volatile int updated;
} ami304_data;

struct _ami304mid_data {
	rwlock_t datalock;
	rwlock_t ctrllock;
	int controldata[10];
	int yaw;
	int roll;
	int pitch;
	int nmx;
	int nmy;
	int nmz;
	int nax;
	int nay;
	int naz;
	int mag_status;
} ami304mid_data;

struct compass_data {
	NvU32 x;
	NvU32 y;
	NvU32 z;
};

struct tegra_compass_device_data
{
	NvOdmCompassDeviceHandle	hOdmComp;
	struct task_struct	*task;
	struct input_dev	*input_dev;
	NvU32			freq;
	NvBool			bThreadAlive;
	NvBool			show_log;
	NvOdmCompassIntType	IntType;
	NvOdmCompassAxisType	IntMotionAxis;
	NvOdmCompassAxisType	IntTapAxis;
	struct timeval		tv;
	struct compass_data prev_data;
	struct compass_data min_data;
	struct compass_data max_data;
};

struct tegra_compass_device_data *compass_dev;

/** Function to close the ODM device. This function will help in switching
 * between power modes
 */
void close_odm_compass(void)
{
	NvOdmCompassClose(compass_dev->hOdmComp);
	compass_dev->hOdmComp = 0;
}

/** Function to open the ODM device with a set of default values. The values
 * are hardcoded as of now. Each time the device is closed/open, previous
 * settings will be lost. This function will help in switching
 * between power modes
 */
int open_def_odm_compass(void)
{
	NvS32 err = -1;
	err = NvOdmCompassOpen(&(compass_dev->hOdmComp));
	if (!err) {
		err = -ENODEV;
		pr_err("open_def_odm_compass: NvOdmCompassOpen failed\n");
		return err;
	}
	return err;
}

int AMI304_Reset_Init()
{
	u8 databuf[10];
	NvU8 regaddr;
	u8 ctrl1, ctrl2, ctrl3;


#if DEBUG_AMI304
	printk("[%s:%d] SRST Setting \n", __FUNCTION__, __LINE__);
#endif

	databuf[1] = 0x80;
#if DEBUG_AMI304
	printk("[%s:%d] AMI304_REG_CTRL3 setting = 0x%x \n", __FUNCTION__, __LINE__, databuf[1]);
#endif
	NvCompassI2CSetRegs(compass_dev->hOdmComp, AMI304_REG_CTRL3, &databuf[1], 1);

#if DEBUG_AMI304
	printk("## [%s:%d] ##\n", __FUNCTION__, __LINE__);
#endif

	msleep(1);
	//	mdelay(20);
#if DEBUG_AMI304
	regaddr = AMI304_REG_CTRL3;
	NvCompassI2CGetRegs(compass_dev->hOdmComp, AMI304_REG_CTRL3, &ctrl3, 1);
	printk("[%s:%d] AMI304_REG_CTRL3(read) = 0x%x \n", __FUNCTION__, __LINE__, ctrl3);
#endif
	return 0;
}

static atomic_t dev_open_count;
static atomic_t hal_open_count;
static atomic_t daemon_open_count;

int AMI304_Init(int mode)
{
	u8 databuf[10];
	//	u8 regaddr;
	NvU8 regaddr;
	u8 ctrl1, ctrl2, ctrl3;

	NvCompassI2CGetRegs(compass_dev->hOdmComp, AMI304_REG_CTRL1, &ctrl1, 1);

#if DEBUG_AMI304
	printk("[%s:%d] Compass Sensor \n", __FUNCTION__, __LINE__);
#endif

#if DEBUG_AMI304
	printk("[%s:%d] AMI304_REG_CTRL1 = 0x%x \n", __FUNCTION__, __LINE__, ctrl1);
#endif

	regaddr = AMI304_REG_CTRL2;
	NvCompassI2CGetRegs(compass_dev->hOdmComp, AMI304_REG_CTRL2, &ctrl2, 1);
#if DEBUG_AMI304
	printk("[%s:%d] AMI304_REG_CTRL2 = 0x%x \n", __FUNCTION__, __LINE__, ctrl2);
#endif

	regaddr = AMI304_REG_CTRL3;
	NvCompassI2CGetRegs(compass_dev->hOdmComp, AMI304_REG_CTRL3, &ctrl3, 1);
#if DEBUG_AMI304
	printk("[%s:%d] AMI304_REG_CTRL3 = 0x%x \n", __FUNCTION__, __LINE__, ctrl3);
	printk("## [%s:%d] ##\n", __FUNCTION__, __LINE__);
#endif
	databuf[0] = AMI304_REG_CTRL1;
	if (mode==AMI304_FORCE_MODE) {
#if DEBUG_AMI304
		printk("## [%s:%d] ##\n", __FUNCTION__, __LINE__);
#endif
		databuf[1] = ctrl1 | AMI304_CTRL1_PC1 | AMI304_CTRL1_FS1_FORCE;
		write_lock(&ami304_data.lock);
		ami304_data.mode = AMI304_FORCE_MODE;
		write_unlock(&ami304_data.lock);
	} else	{
#if DEBUG_AMI304
		printk("## [%s:%d] ##\n", __FUNCTION__, __LINE__);
#endif
		databuf[1] = ctrl1 | AMI304_CTRL1_PC1 | AMI304_CTRL1_FS1_NORMAL | AMI304_CTRL1_ODR1;
		write_lock(&ami304_data.lock);
		ami304_data.mode = AMI304_NORMAL_MODE;
		write_unlock(&ami304_data.lock);
	}
#if DEBUG_AMI304
	printk("## [%s:%d] ##\n", __FUNCTION__, __LINE__);
#endif

	NvCompassI2CSetRegs(compass_dev->hOdmComp, AMI304_REG_CTRL1, &databuf[1], 1);
#if DEBUG_AMI304
	printk("## [%s:%d] ##\n", __FUNCTION__, __LINE__);
#endif

	databuf[0] = AMI304_REG_CTRL2;
	databuf[1] = ctrl2 | AMI304_CTRL2_DREN;
	NvCompassI2CSetRegs(compass_dev->hOdmComp, AMI304_REG_CTRL2, &databuf[1], 1);
#if DEBUG_AMI304
	printk("## [%s:%d] ##\n", __FUNCTION__, __LINE__);
#endif

	databuf[0] = AMI304_REG_CTRL3;
	databuf[1] = ctrl3 | AMI304_CTRL3_B0_LO_CLR;
	NvCompassI2CSetRegs(compass_dev->hOdmComp, AMI304_REG_CTRL3, &databuf[1], 1);
#if DEBUG_AMI304
	printk("## [%s:%d] ##\n", __FUNCTION__, __LINE__);
#endif

	return 0;
}

int tegra_compass_hw_init(void)
{
	msleep(1);
	//	mdelay(50); //to waiting time(50ms) from PowerOFF to Stand-by
	//On soft reset
	AMI304_Reset_Init();

	AMI304_Init(AMI304_FORCE_MODE); // default is Force State
	
}

static int AMI304_SetMode(int newmode)
{
	int mode = 0;

	read_lock(&ami304_data.lock);
	mode = ami304_data.mode;
	read_unlock(&ami304_data.lock);

	if (mode == newmode)
		return 0;

	return AMI304_Init(newmode);
}

static int AMI304_ReadChipInfo(char *buf, int bufsize)
{
	if ((!buf) || (bufsize <= 30))
		return -1;
	/*
	   if (!ami304_i2c_client)
	   {
	 *buf = 0;
	 return -2;
	 }*/

	sprintf(buf, "AMI304 Chip");
	return 0;
}

static int AMI304_ReadSensorData(char *buf, int bufsize)
{
	char cmd;
	int mode = 0;
	unsigned char ctrl3;
	unsigned char databuf[10];

	if ((!buf)||(bufsize<=80))
		return -1;
	/*
	   if (!ami304_i2c_client)
	   {
	 *buf = 0;
	 return -2;
	 }*/

	read_lock(&ami304_data.lock);
	mode = ami304_data.mode;
	read_unlock(&ami304_data.lock);

	databuf[0] = AMI304_REG_CTRL3;
	databuf[1] = AMI304_CTRL3_FORCE_BIT;
	NvCompassI2CSetRegs(compass_dev->hOdmComp, AMI304_REG_CTRL3, &databuf[1], 1);
	NvCompassI2CGetRegs(compass_dev->hOdmComp, AMI304_REG_CTRL3, &ctrl3, 1);

#if DEBUG_AMI304
	//printk("[%s:%d] AMI304_REG_CTRL3 = 0x%x \n", __FUNCTION__, __LINE__, ctrl3);
#endif

	// We can read all measured data in once
	//cmd = AMI304_REG_DATAXH;
	NvCompassI2CGetRegs(compass_dev->hOdmComp, AMI304_REG_DATAXH, &databuf[0], 6);

#if DEBUG_AMI304
	printk("%02x %02x %02x %02x %02x %02x \n", databuf[0], databuf[1], databuf[2], databuf[3], databuf[4], databuf[5]);
#endif

	sprintf(buf, "%02x %02x %02x %02x %02x %02x", databuf[0], databuf[1], databuf[2], databuf[3], databuf[4], databuf[5]);
#if DEBUG_DATA
	//printk( "============================> COMPASS-DATA %02x %02x %02x %02x %02x %02x\n", databuf[0], databuf[1], databuf[2], databuf[3], databuf[4], databuf[5]);
#endif
#if DEBUG_DATA
	{
		int mx=0;
		int my=0;
		int mz=0;
		int tmp_x=databuf[1];
		int tmp_y=databuf[3];
		int tmp_z=databuf[5];

		mx = tmp_x << 8 | databuf[0];
		my = tmp_y << 8 | databuf[2];
		mz = tmp_z << 8 | databuf[4];
		if (mx>32768)  mx = mx-65536;
		if (my>32768)  my = my-65536;
		if (mz>32768)  mz = mz-65536;
#if 0 /*Dont need, Because of confusion*/
		//mx += 2048;
		//my += 2048;
		//mz += 2048;
#endif
		printk("%s Magnetic : mx=%d my=%d mz=%d\n",__FUNCTION__, mx, my, mz);
	}
#endif

	return 0;
}

static int AMI304_ReadPostureData(char *buf, int bufsize)
{
	if ((!buf) || (bufsize <= 80))
		return -1;

	read_lock(&ami304mid_data.datalock);
	sprintf(buf, "%d %d %d %d", ami304mid_data.yaw, ami304mid_data.pitch, ami304mid_data.roll, ami304mid_data.mag_status);
	read_unlock(&ami304mid_data.datalock);
	return 0;
}

static int AMI304_ReadCaliData(char *buf, int bufsize)
{
	if ((!buf) || (bufsize <= 80))
		return -1;

	read_lock(&ami304mid_data.datalock);
	sprintf(buf, "%d %d %d %d %d %d %d", ami304mid_data.nmx, ami304mid_data.nmy, ami304mid_data.nmz,ami304mid_data.nax,ami304mid_data.nay,ami304mid_data.naz,ami304mid_data.mag_status);
	read_unlock(&ami304mid_data.datalock);
	return 0;
}

static int AMI304_ReadMiddleControl(char *buf, int bufsize)
{
	if ((!buf) || (bufsize <= 80))
		return -1;

	read_lock(&ami304mid_data.ctrllock);
	sprintf(buf, "%d %d %d %d %d %d %d %d %d %d",
			ami304mid_data.controldata[0], ami304mid_data.controldata[1], ami304mid_data.controldata[2],ami304mid_data.controldata[3],ami304mid_data.controldata[4],
			ami304mid_data.controldata[5], ami304mid_data.controldata[6], ami304mid_data.controldata[7], ami304mid_data.controldata[8], ami304mid_data.controldata[9]);
	read_unlock(&ami304mid_data.ctrllock);
	return 0;
}

static int AMI304_Report_Value(int en_dis)
{
	//	struct ami304_i2c_data *data = i2c_get_clientdata(ami304_i2c_client);

	if (!en_dis)
		return 0;
#if  DEBUG_DATA
	printk("\n## Kernel   Accel   ## %s, %d ## %d ,%d ,%d ,%d 	\n",__FUNCTION__, __LINE__, ami304mid_data.yaw ,ami304mid_data.pitch,
			ami304mid_data.roll, ami304mid_data.mag_status);
	printk("\n## Kernel Raw-Accel ## %s, %d ## %d , %d , %d 	\n",__FUNCTION__, __LINE__, ami304mid_data.nax ,ami304mid_data.nay,
			ami304mid_data.naz );
	printk("## Kernel Magn    ## %s, %d ## %d ,%d ,%d ,%d \n",__FUNCTION__, __LINE__, ami304mid_data.nmx, ami304mid_data.nmy ,
			ami304mid_data.nmz, ami304mid_data.mag_status );
#endif

	input_report_abs(compass_dev->input_dev, ABS_RX, ami304mid_data.yaw);	/* yaw */
	input_report_abs(compass_dev->input_dev, ABS_RY, ami304mid_data.pitch);/* pitch */
	input_report_abs(compass_dev->input_dev, ABS_RZ, ami304mid_data.roll);/* roll */
	input_report_abs(compass_dev->input_dev, ABS_RUDDER, ami304mid_data.mag_status);/* status of orientation sensor */

#if 1
	input_report_abs(compass_dev->input_dev, ABS_X, ami304mid_data.nax);/* x-axis raw acceleration */
	input_report_abs(compass_dev->input_dev, ABS_Y, ami304mid_data.nay);/* y-axis raw acceleration */
	input_report_abs(compass_dev->input_dev, ABS_Z, ami304mid_data.naz);/* z-axis raw acceleration */
#endif

	input_report_abs(compass_dev->input_dev, ABS_HAT0X, ami304mid_data.nmx); /* x-axis of raw magnetic vector */
	input_report_abs(compass_dev->input_dev, ABS_HAT0Y, ami304mid_data.nmy); /* y-axis of raw magnetic vector */
	input_report_abs(compass_dev->input_dev, ABS_BRAKE, ami304mid_data.nmz); /* z-axis of raw magnetic vector */
	input_report_abs(compass_dev->input_dev, ABS_WHEEL, ami304mid_data.mag_status);/* status of magnetic sensor */

	input_sync(compass_dev->input_dev);

	return 0;
}

int open_ami304()
{
	AMI304_Reset_Init();

	AMI304_Init(AMI304_FORCE_MODE); // default is Force State
#if DEBUG_AMI304
	printk("[%s:%d] Compass Sensor: AMI304 registered driver! \n", __FUNCTION__, __LINE__);
#endif
	return 0;  
}

static ssize_t show_chipinfo_value(struct device *dev, struct device_attribute *attr, char *buf)
{
	char strbuf[AMI304_BUFSIZE];
	AMI304_ReadChipInfo(strbuf, AMI304_BUFSIZE);
	return sprintf(buf, "%s\n", strbuf);
}

static ssize_t show_sensordata_value(struct device *dev, struct device_attribute *attr, char *buf)
{
	char strbuf[AMI304_BUFSIZE];
	AMI304_ReadSensorData(strbuf, AMI304_BUFSIZE);
	return sprintf(buf, "%s\n", strbuf);
}

static ssize_t show_posturedata_value(struct device *dev, struct device_attribute *attr, char *buf)
{
	char strbuf[AMI304_BUFSIZE];
	AMI304_ReadPostureData(strbuf, AMI304_BUFSIZE);
	return sprintf(buf, "%s\n", strbuf);
}

static ssize_t show_calidata_value(struct device *dev, struct device_attribute *attr, char *buf)
{
	char strbuf[AMI304_BUFSIZE];
	AMI304_ReadCaliData(strbuf, AMI304_BUFSIZE);
	return sprintf(buf, "%s\n", strbuf);
}

static ssize_t show_midcontrol_value(struct device *dev, struct device_attribute *attr, char *buf)
{
	char strbuf[AMI304_BUFSIZE];
	AMI304_ReadMiddleControl(strbuf, AMI304_BUFSIZE);
	return sprintf(buf, "%s\n", strbuf);
}

static ssize_t store_midcontrol_value(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	write_lock(&ami304mid_data.ctrllock);
	memcpy(&ami304mid_data.controldata[0], buf, sizeof(int) * 10);
	write_unlock(&ami304mid_data.ctrllock);
	return count;
}

static ssize_t show_mode_value(struct device *dev, struct device_attribute *attr, char *buf)
{
	int mode=0;
	read_lock(&ami304_data.lock);
	mode = ami304_data.mode;
	read_unlock(&ami304_data.lock);
	return sprintf(buf, "%d\n", mode);
}

static ssize_t store_mode_value(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int mode = 0;
	sscanf(buf, "%d", &mode);
	AMI304_SetMode(mode);
	return count;
}


static DEVICE_ATTR(chipinfo, S_IRUGO, show_chipinfo_value, NULL);
static DEVICE_ATTR(sensordata, S_IRUGO, show_sensordata_value, NULL);
static DEVICE_ATTR(posturedata, S_IRUGO, show_posturedata_value, NULL);
static DEVICE_ATTR(calidata, S_IRUGO, show_calidata_value, NULL);
static DEVICE_ATTR(midcontrol, S_IRUGO | S_IWUSR, show_midcontrol_value, store_midcontrol_value);
static DEVICE_ATTR(mode, S_IRUGO | S_IWUSR, show_mode_value, store_mode_value );


static struct attribute *ami304_attributes[] = {
	&dev_attr_chipinfo.attr,
	&dev_attr_sensordata.attr,
	&dev_attr_posturedata.attr,
	&dev_attr_calidata.attr,
	&dev_attr_midcontrol.attr,
	&dev_attr_mode.attr,
	NULL
};

static struct attribute_group ami304_attribute_group = {
	.attrs = ami304_attributes
};

static int ami304_open(struct inode *inode, struct file *file)
{
	int ret = -1;
#if DEBUG_AMI304
	printk(KERN_ERR  "ami304 - %s \n",__FUNCTION__);
#endif
	if (atomic_cmpxchg(&dev_open_count, 0, 1)==0) {
		printk(KERN_INFO "Open device node:ami304\n");
		ret = nonseekable_open(inode, file);
	}
	return ret;
}

static int ami304_release(struct inode *inode, struct file *file)
{
	atomic_set(&dev_open_count, 0);
#if DEBUG_AMI304
	printk(KERN_ERR  "ami304 - %s \n",__FUNCTION__);
#endif

	return 0;
}

static int ami304_ioctl(struct inode *inode, struct file *file, unsigned int cmd,unsigned long arg)
{
	char strbuf[AMI304_BUFSIZE];
	int controlbuf[10];
	void __user *data;
	int retval=0;
	int mode=0;
#if DEBUG_AMI304
	printk(KERN_ERR  "ami304 - %s \n",__FUNCTION__);
#endif

	//check the authority is root or not
	if (!capable(CAP_SYS_ADMIN)) {
		retval = -EPERM;
		goto err_out;
	}

	switch (cmd) {
		case AMI304_IOCTL_INIT:
			read_lock(&ami304_data.lock);
			mode = ami304_data.mode;
			read_unlock(&ami304_data.lock);
			AMI304_Init(mode);
			break;

		case AMI304_IOCTL_READ_CHIPINFO:
			data = (void __user *) arg;
			if (data == NULL)
				break;
			AMI304_ReadChipInfo(strbuf, AMI304_BUFSIZE);
			if (copy_to_user(data, strbuf, strlen(strbuf) + 1)) {
				retval = -EFAULT;
				goto err_out;
			}
			break;

		case AMI304_IOCTL_READ_SENSORDATA:
			data = (void __user *) arg;
			if (data == NULL)
				break;
			AMI304_ReadSensorData(strbuf, AMI304_BUFSIZE);
			if (copy_to_user(data, strbuf, strlen(strbuf) + 1)) {
				retval = -EFAULT;
				goto err_out;
			}
			break;

		case AMI304_IOCTL_READ_POSTUREDATA:
			data = (void __user *) arg;
			if (data == NULL)
				break;
			AMI304_ReadPostureData(strbuf, AMI304_BUFSIZE);
			if (copy_to_user(data, strbuf, strlen(strbuf) + 1)) {
				retval = -EFAULT;
				goto err_out;
			}
			break;

		case AMI304_IOCTL_READ_CALIDATA:
			data = (void __user *) arg;
			if (data == NULL)
				break;
			AMI304_ReadCaliData(strbuf, AMI304_BUFSIZE);
			if (copy_to_user(data, strbuf, strlen(strbuf) + 1)) {
				retval = -EFAULT;
				goto err_out;
			}
			break;

		case AMI304_IOCTL_READ_CONTROL:
			read_lock(&ami304mid_data.ctrllock);
			memcpy(controlbuf, &ami304mid_data.controldata[0], sizeof(controlbuf));
			read_unlock(&ami304mid_data.ctrllock);
			data = (void __user *) arg;
			if (data == NULL)
				break;
			if (copy_to_user(data, controlbuf, sizeof(controlbuf))) {
				retval = -EFAULT;
				goto err_out;
			}
			break;

		case AMI304_IOCTL_SET_CONTROL:
			data = (void __user *) arg;
			if (data == NULL)
				break;
			if (copy_from_user(controlbuf, data, sizeof(controlbuf))) {
				retval = -EFAULT;
				goto err_out;
			}
			write_lock(&ami304mid_data.ctrllock);
			memcpy(&ami304mid_data.controldata[0], controlbuf, sizeof(controlbuf));
			write_unlock(&ami304mid_data.ctrllock);
			break;

		case AMI304_IOCTL_SET_MODE:
			data = (void __user *) arg;
			if (data == NULL)
				break;
			if (copy_from_user(&mode, data, sizeof(mode))) {
				retval = -EFAULT;
				goto err_out;
			}
			AMI304_SetMode(mode);
			break;

		default:
			printk(KERN_ERR "%s not supported = 0x%04x", __FUNCTION__, cmd);
			retval = -ENOIOCTLCMD;
			break;
	}

err_out:
	return retval;
}

static int ami304daemon_open(struct inode *inode, struct file *file)
{
	//return nonseekable_open(inode, file);
	int ret = -1;
	if (atomic_cmpxchg(&daemon_open_count, 0, 1)==0) {
		printk(KERN_INFO "Open device node:ami304daemon\n");
		ret = 0;
	}
	return ret;
}

static int ami304daemon_release(struct inode *inode, struct file *file)
{
	atomic_set(&daemon_open_count, 0);
#if DEBUG_AMI304
	printk(KERN_INFO "[%s:%d]Compass sensor \n",__FUNCTION__, __LINE__);
#endif
	return 0;
}

static int ami304daemon_ioctl(struct inode *inode, struct file *file, unsigned int cmd,
		unsigned long arg)
{
	int valuebuf[4];
	int calidata[7];
	int controlbuf[10];
	char strbuf[AMI304_BUFSIZE];
	void __user *data;
	int retval=0;
	int mode;
	int en_dis_Report;

	//check the authority is root or not
	if (!capable(CAP_SYS_ADMIN)) {
		retval = -EPERM;
		goto err_out;
	}

	switch (cmd) {

		case AMI304MID_IOCTL_GET_SENSORDATA:
#if DEBUG_DAEMON_IOCTL
			printk("Release ami304daemon_ioctl, :AMI304MID_IOCTL_GET_SENSORDATA\n");
#endif
			data = (void __user *) arg;
			if (data == NULL)
				break;
			AMI304_ReadSensorData(strbuf, AMI304_BUFSIZE);
			if (copy_to_user(data, strbuf, strlen(strbuf) + 1)) {
				retval = -EFAULT;
				goto err_out;
			}
			break;

		case AMI304MID_IOCTL_SET_POSTURE:
#if DEBUG_DAEMON_IOCTL
			printk("Release ami304daemon_ioctl, :AMI304MID_IOCTL_SET_POSTURE\n");
#endif
			data = (void __user *) arg;
			if (data == NULL)
				break;
			if (copy_from_user(&valuebuf, data, sizeof(valuebuf))) {
				retval = -EFAULT;
				goto err_out;
			}
			write_lock(&ami304mid_data.datalock);
			ami304mid_data.yaw   = valuebuf[0];
			ami304mid_data.pitch = valuebuf[1];
			ami304mid_data.roll  = valuebuf[2];
			ami304mid_data.mag_status = valuebuf[3];
			write_unlock(&ami304mid_data.datalock);
			break;

		case AMI304MID_IOCTL_SET_CALIDATA:
#if DEBUG_DAEMON_IOCTL
			printk("Release ami304daemon_ioctl, :AMI304MID_IOCTL_SET_CALIDATA\n");
#endif
			data = (void __user *) arg;
			if (data == NULL)
				break;
			if (copy_from_user(&calidata, data, sizeof(calidata))) {
				retval = -EFAULT;
				goto err_out;
			}
			write_lock(&ami304mid_data.datalock);
			ami304mid_data.nmx = calidata[0];
			ami304mid_data.nmy = calidata[1];
			ami304mid_data.nmz = calidata[2];
			ami304mid_data.nax = calidata[3];
			ami304mid_data.nay = calidata[4];
			ami304mid_data.naz = calidata[5];
			ami304mid_data.mag_status = calidata[6];
			write_unlock(&ami304mid_data.datalock);
			//			AMI304_Report_Value(en_dis_Report);
			break;

		case AMI304MID_IOCTL_GET_CONTROL:
#if DEBUG_DAEMON_IOCTL
			printk("Release ami304daemon_ioctl, :AMI304MID_IOCTL_GET_CONTROL\n");
#endif
			read_lock(&ami304mid_data.ctrllock);
			memcpy(controlbuf, &ami304mid_data.controldata[0], sizeof(controlbuf));
			read_unlock(&ami304mid_data.ctrllock);
			data = (void __user *) arg;
			if (data == NULL)
				break;
			if (copy_to_user(data, controlbuf, sizeof(controlbuf))) {
				retval = -EFAULT;
				goto err_out;
			}
			break;

		case AMI304MID_IOCTL_SET_CONTROL:
#if DEBUG_DAEMON_IOCTL
			printk("Release ami304daemon_ioctl, :AMI304MID_IOCTL_SET_CONTROL\n");
#endif

			data = (void __user *) arg;
			if (data == NULL)
				break;
			if (copy_from_user(controlbuf, data, sizeof(controlbuf))) {
				retval = -EFAULT;
				goto err_out;
			}
			write_lock(&ami304mid_data.ctrllock);
			memcpy(&ami304mid_data.controldata[0], controlbuf, sizeof(controlbuf));
			write_unlock(&ami304mid_data.ctrllock);
			break;

		case AMI304MID_IOCTL_SET_MODE:
#if DEBUG_DAEMON_IOCTL
			printk("Release ami304daemon_ioctl, :AMI304MID_IOCTL_SET_MODE\n");
#endif

			data = (void __user *) arg;
			if (data == NULL)
				break;
			if (copy_from_user(&mode, data, sizeof(mode))) {
				retval = -EFAULT;
				goto err_out;
			}
			AMI304_SetMode(mode);
			break;

			//Add for input_device sync
		case AMI304MID_IOCTL_SET_REPORT:
			data = (void __user *) arg;
			if (data == NULL)
				break;
			if (copy_from_user(&en_dis_Report, data, sizeof(mode))) {
				retval = -EFAULT;
				goto err_out;
			}
			read_lock(&ami304mid_data.datalock);
			AMI304_Report_Value(en_dis_Report);
			read_unlock(&ami304mid_data.datalock);
			break;

		default:
			printk(KERN_ERR "%s not supported = 0x%04x", __FUNCTION__, cmd);
			retval = -ENOIOCTLCMD;
			break;
	}

err_out:
	return retval;
}

static int ami304hal_open(struct inode *inode, struct file *file)
{
	//return nonseekable_open(inode, file);
	atomic_inc_and_test(&hal_open_count);
#if DEBUG_AMI304
	printk(KERN_INFO "Open device node:ami304hal %d times.\n", atomic_read(&hal_open_count));
#endif

	return 0;
}

static int ami304hal_release(struct inode *inode, struct file *file)
{
	atomic_dec_and_test(&hal_open_count);
#if DEBUG_AMI304
	printk(KERN_INFO "Release ami304hal, remainder is %d times.\n", atomic_read(&hal_open_count));
#endif
	return 0;
}

static int ami304hal_ioctl(struct inode *inode, struct file *file, unsigned int cmd,unsigned long arg)
{
	char strbuf[AMI304_BUFSIZE];
	void __user *data;
	int retval=0;
#if DEBUG_AMI304
	printk("Release ami304hal_ioctl\n");
#endif

	switch (cmd) {

		case AMI304HAL_IOCTL_GET_SENSORDATA:
#if DEBUG_HAL_IOCTL
			printk("Release ami304hal_ioctl, AMI304HAL_IOCTL_GET_SENSORDATA:\n");
#endif
			data = (void __user *) arg;
			if (data == NULL)
				break;
			AMI304_ReadSensorData(strbuf, AMI304_BUFSIZE);
			if (copy_to_user(data, strbuf, strlen(strbuf) + 1)) {
				retval = -EFAULT;
				goto err_out;
			}
			break;

		case AMI304HAL_IOCTL_GET_POSTURE:
#if DEBUG_HAL_IOCTL
			printk("Release ami304hal_ioctl, :AMI304HAL_IOCTL_GET_POSTURE\n");
#endif

			data = (void __user *) arg;
			if (data == NULL)
				break;
			AMI304_ReadPostureData(strbuf, AMI304_BUFSIZE);
			if (copy_to_user(data, strbuf, strlen(strbuf) + 1)) {
				retval = -EFAULT;
				goto err_out;
			}
			break;

		case AMI304HAL_IOCTL_GET_CALIDATA:
#if DEBUG_HAL_IOCTL
			printk("Release ami304hal_ioctl, :AMI304HAL_IOCTL_GET_CALIDATA\n");
#endif

			data = (void __user *) arg;
			if (data == NULL)
				break;
			AMI304_ReadCaliData(strbuf, AMI304_BUFSIZE);
			if (copy_to_user(data, strbuf, strlen(strbuf) + 1)) {
				retval = -EFAULT;
				goto err_out;
			}
			break;

		default:
#if DEBUG_HAL_IOCTL
			printk("Release ami304hal_ioctl, :\n");
#endif
			printk("[error] ami304hal_ioctl %s not supported = 0x%04x", __FUNCTION__, cmd);
			retval = -ENOIOCTLCMD;
			break;
	}

err_out:
	return retval;
}

static struct file_operations ami304_fops = {
	.owner = THIS_MODULE,
	.open = ami304_open,
	.release = ami304_release,
	.ioctl = ami304_ioctl,
};

static struct miscdevice ami304_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "ami304",
	.fops = &ami304_fops,
};


static struct file_operations ami304daemon_fops = {
	.owner = THIS_MODULE,
	.open = ami304daemon_open,
	.release = ami304daemon_release,
	.ioctl = ami304daemon_ioctl,
};

static struct miscdevice ami304daemon_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "ami304daemon",
	.fops = &ami304daemon_fops,
};

static struct file_operations ami304hal_fops = {
	.owner = THIS_MODULE,
	.open = ami304hal_open,
	.release = ami304hal_release,
	.ioctl = ami304hal_ioctl,
};

static struct miscdevice ami304hal_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "ami304hal",
	.fops = &ami304hal_fops,
};

/*
   static int ami304_i2c_attach_adapter(struct i2c_adapter *adapter)
   {
   int res;
   printk(KERN_INFO "\n\nEnter ami304_i2c_attach_adapter!!\n");
   res = i2c_probe(adapter, &addr_data, ami304_i2c_detect);
   printk(KERN_INFO "      res of ami304_i2c_attach_adapter= %d\n", res);
   return res;
   }
   */

/**
 * All the device spefic initializations happen here.
 */
static NvS32 __init ami304_probe(struct platform_device *pdev)
{

	struct i2c_client *new_client = ami304_i2c_client;
	struct tegra_compass_device_data *compass = NULL;
	struct input_dev *input_dev = NULL;
	struct device *dev = &pdev->dev;
	NvS32 err;

#if DEBUG_AMI304
	printk("[%s:%d] Compass Sensor \n", __FUNCTION__, __LINE__);
#endif
	compass = kzalloc(sizeof(*compass), GFP_KERNEL);
	if (compass == NULL) {
		err = -ENOMEM;
		pr_err("ami304_probe: Failed to memory\n");
		goto allocate_dev_fail;
	}
	compass_dev = compass;


	input_dev = input_allocate_device();
	if (input_dev == NULL) {
		err = -ENOMEM;
		pr_err("tegra_com_ami304_probe: Failed to allocate input device\n");
		goto allocate_dev_fail;
	}
	compass_dev->input_dev 	= input_dev;


	err = open_def_odm_compass();
	if (!err) {
		pr_err("open_def_odm_comp: Failed \n");
		goto exit_alloc_data_failed;
	}

#if DEBUG_AMI304
	printk("[%s:%d] Compass Sensor: AMI304 registered driver! \n", __FUNCTION__, __LINE__);
#endif

	err = misc_register(&ami304_device);
	if (err) {
		printk(KERN_ERR
				"ami304_device register failed\n");
		goto exit_misc_device_register_failed;
	}
#if 1
	platform_set_drvdata(pdev, compass);
	set_bit(EV_ABS, compass_dev->input_dev->evbit);
	/* yaw */
	input_set_abs_params(compass_dev->input_dev, ABS_RX, 0, 360, 0, 0);
	/* pitch */
	input_set_abs_params(compass_dev->input_dev, ABS_RY, -180, 180, 0, 0);
	/* roll */
	input_set_abs_params(compass_dev->input_dev, ABS_RZ, -90, 90, 0, 0);
	/* status of magnetic sensor */
	input_set_abs_params(compass_dev->input_dev, ABS_RUDDER, 0, 5, 0, 0);

	/* x-axis acceleration */
	input_set_abs_params(compass_dev->input_dev, ABS_X, -2000, 2000, 0, 0);
	/* y-axis acceleration */
	input_set_abs_params(compass_dev->input_dev, ABS_Y, -2000, 2000, 0, 0);
	/* z-axis acceleration */
	input_set_abs_params(compass_dev->input_dev, ABS_Z, -2000, 2000, 0, 0);

	/* x-axis of raw magnetic vector */
	//input_set_abs_params(compass_dev->input_dev, ABS_HAT0X, -3000, 3000, 0, 0);
	/* y-axis of raw magnetic vector */
	//input_set_abs_params(compass_dev->input_dev, ABS_HAT0Y, -3000, 3000, 0, 0);
	/* z-axis of raw magnetic vector */
	//input_set_abs_params(compass_dev->input_dev, ABS_BRAKE, -3000, 3000, 0, 0);
	/* status of acceleration sensor */
	input_set_abs_params(compass_dev->input_dev, ABS_WHEEL, 0, 5, 0, 0);

#if DEBUG_AMI304
	printk("[%s:%d] Compass Sensor \n", __FUNCTION__, __LINE__);
#endif
	compass_dev->input_dev->name = "Acompass";
	err = input_register_device(input_dev);
	if (err) {
		pr_err("tegra_compass_probe: Unable to register %s\
				input device\n", input_dev->name);
		goto exit_input_register_device_failed;
	}

	err = sysfs_create_group(&dev->kobj, &ami304_attribute_group);

#else
	platform_set_drvdata(pdev, compass);

	input_dev->name = "accelerometer_tegra";
	err = input_register_device(input_dev);
	if (err) {
		pr_err("tegra_compass_probe: Unable to register %s\
				input device\n", input_dev->name);
		goto input_register_device_failed;
	}

	err = add_sysfs_entry();
	err = sysfs_create_group(&compass_dev->input_dev, &ami304_attribute_group);
	if (err)
		printk("goto err1;\n");

#endif
	err = misc_register(&ami304daemon_device);
	if (err) {
		printk(KERN_ERR
				"ami304daemon_device register failed\n");
		goto exit_misc_device_register_failed;
	}

	err = misc_register(&ami304hal_device);
	if (err) {
		printk(KERN_ERR
				"ami304hal_device register failed\n");
		goto exit_misc_device_register_failed;
	}
#ifdef CONFIG_HAS_EARLYSUSPEND // wkkim : temporary early suspend apply
	early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	early_suspend.suspend = compass_early_suspend;
	early_suspend.resume = compass_late_resume;
	register_early_suspend(&early_suspend);
#endif

	return 0;
exit_input_register_device_failed:
	input_free_device(compass_dev->input_dev);
exit_misc_device_register_failed:
	//exit_kfree:
	kfree(compass_dev);
allocate_dev_fail:
exit_input_dev_alloc_failed:
exit_alloc_data_failed:
	close_odm_compass();
	input_free_device(input_dev);
	kfree(compass);
	compass = 0;
	err = -ENOMEM;
exit:
	return err;
}

int lge_sensor_verify_compass(void)
{
    unsigned char value = 0;
    NvCompassI2CGetRegs(compass_dev->hOdmComp, 0x1d, &value, 1);
    printk("[%s] ### Compass ## WHO_AM_I = 0x%x \n", MOD_TAG,value);
    
    if(COMPASS_WHID != value)
        return SENSOR_ERROR;

    return SENSOR_OK;
}

int lge_sensor_shutdown_compass(void)
{
	// reset device 
	printk("[%s:%d] \n", __FUNCTION__, __LINE__);
	close_odm_compass();
	return SENSOR_OK;
}

int lge_sensor_restart_compass(void)
{
	NvS32 err = -1;

	printk("[%s:%d] \n", __FUNCTION__, __LINE__);
	err = open_def_odm_compass();
	if (!err) {
		pr_err("open_def_odm_comp: Failed \n");
		return (-1);
	}
	msleep(1);
	//	mdelay(50); //to waiting time(50ms) from PowerOFF to Stand-by

	err = open_ami304();
	if (err) {
		printk(KERN_ERR
				"ami304 device Init failed\n");
		return (-1);
	}

#if DEBUG_AMI304
	printk("[%s:%d] Compass Sensor: AMI304 registered driver! \n", __FUNCTION__, __LINE__);
#endif

	return SENSOR_OK;
}
static NvS32 ami304_remove(struct platform_device *pdev)
{
	int err;
	//struct ami304_i2c_data *data = i2c_get_clientdata(client);
	input_unregister_device(compass_dev->input_dev);

	close_odm_compass();

	kfree(compass_dev);
	misc_deregister(&ami304hal_device);
	misc_deregister(&ami304daemon_device);
	misc_deregister(&ami304_device);
	return 0;
}

static NvS32 ami304_resume(struct platform_device *pdev)
{
#if 0
	int err;
	u8 databuf[10], ctrl1;
	//struct ami304_i2c_data *data = i2c_get_clientdata(client);
#if DEBUG_AMI304
	printk("[%s:%d] Compass Sensor \n", __FUNCTION__, __LINE__);
#endif

	//one step
	NvCompassI2CGetRegs(compass_dev->hOdmComp, AMI304_REG_CTRL1, &ctrl1, 1);
	databuf[0] = (ctrl1 | 0x80);
	NvCompassI2CSetRegs(compass_dev->hOdmComp, AMI304_REG_CTRL1, &databuf[0], 1);

	//second step
	NvCompassI2CGetRegs(compass_dev->hOdmComp, AMI304_REG_CTRL2, &ctrl1, 1);
	databuf[0] = (ctrl1 | 0x0C);
	NvCompassI2CSetRegs(compass_dev->hOdmComp, AMI304_REG_CTRL2, &databuf[0], 1);

#endif

	return 0;
}

static NvS32 ami304_suspend(struct platform_device *pdev)
{
#if 0
	int err;
	u8 databuf[10], ctrl1;

#if DEBUG_AMI304
	printk("[%s:%d] Compass Sensor \n", __FUNCTION__, __LINE__);
	//struct ami304_i2c_data *data = i2c_get_clientdata(client);
#endif

	NvCompassI2CGetRegs(compass_dev->hOdmComp, AMI304_REG_CTRL1, &ctrl1, 1);

	databuf[0] = AMI304_REG_CTRL1;
	databuf[1] = (ctrl1 & ~(0x80));
	NvCompassI2CSetRegs(compass_dev->hOdmComp, AMI304_REG_CTRL1, &databuf[1], 1);
#endif
	return 0;
}


static struct platform_driver tegra_compass_driver = {
	.probe	= ami304_probe,
	.remove = ami304_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND  /* wkkim change early suspend */
	.suspend = ami304_suspend,
	.resume  = ami304_resume,
#endif
	.driver = {
		.name = "tegra_compass",
	},
};

static NvS32 __devinit  ami304_init(void)
{
	printk(KERN_INFO "AMI304 MI sensor driver: init\n");

	rwlock_init(&ami304mid_data.ctrllock);
	rwlock_init(&ami304mid_data.datalock);
	rwlock_init(&ami304_data.lock);
	memset(&ami304mid_data.controldata[0], 0, sizeof(int) * 10);
	ami304mid_data.controldata[0] = 100000;/*100 ms 1sec 10 times*/ //200000; //Loop Delay (sleep time) , 200ms
	ami304mid_data.controldata[1] = 0; // Run
	ami304mid_data.controldata[2] = 0; // Disable Start-AccCali
	ami304mid_data.controldata[3] = 1; // Enable Start-Cali
	ami304mid_data.controldata[4] = 250; // MW-Timout ( calibration )
	ami304mid_data.controldata[5] = 10; // MW-IIRStrength_M
	ami304mid_data.controldata[6] = 10; // MW-IIRStrength_G
	atomic_set(&dev_open_count, 0);
	atomic_set(&hal_open_count, 0);
	atomic_set(&daemon_open_count, 0);
	return platform_driver_register(&tegra_compass_driver);
}

static void __exit ami304_exit(void)
{
	atomic_set(&dev_open_count, 0);
	atomic_set(&hal_open_count, 0);
	atomic_set(&daemon_open_count, 0);
	platform_driver_unregister(&tegra_compass_driver);
}

/* wkkim wrapper function to read compass info */
NvBool compassI2CSetRegs(NvU8 offset, NvU8* value, NvU32 len)
{
	return NvCompassI2CSetRegs(compass_dev->hOdmComp, offset, value, len);
}


NvBool compassI2CGetRegs(NvU8 offset, NvU8* value, NvU32 len)
{
	return NvCompassI2CGetRegs(compass_dev->hOdmComp, offset, value, len);
}

void magnetic_input_report(int *mag_raw)
{
	//printk("MAG report %d:%d:%d \n",mag_raw[0], mag_raw[1], mag_raw[2]);
	input_report_abs(compass_dev->input_dev, ABS_HAT0X, mag_raw[0]); /* x-axis of raw magnetic vector */
	input_report_abs(compass_dev->input_dev, ABS_HAT0Y, mag_raw[1]); /* y-axis of raw magnetic vector */
	input_report_abs(compass_dev->input_dev, ABS_BRAKE, mag_raw[2]); /* z-axis of raw magnetic vector */
	input_sync(compass_dev->input_dev);
}

module_init(ami304_init);
module_exit(ami304_exit);

MODULE_AUTHOR("Kyle K.Y. Chen");
MODULE_DESCRIPTION("AMI304 MI sensor input_dev driver v1.0.5.10");
MODULE_LICENSE("GPL");
