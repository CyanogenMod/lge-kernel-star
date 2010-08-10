/*
 * drivers/input/misc/tegra_odm_accel.c
 *
 * Accelerometer input device using NVIDIA Tegra ODM kit
 *
 * Copyright (c) 2009, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#define NV_DEBUG 0

#include <linux/module.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <asm/uaccess.h>
#include <linux/kernel.h>

#include <nvodm_services.h>
#include <nvodm_accelerometer.h>

#define DRIVER_NAME "nvodm_accelerometer"
#define READ_BUFFER_LENGTH 20

static struct input_dev *g_input_dev;

struct accelerometer_data {
	NvU32 x;
	NvU32 y;
	NvU32 z;
};

struct tegra_acc_device_data
{
	NvOdmAcrDeviceHandle	hOdmAcr;
	struct task_struct	*task;
	struct input_dev	*input_dev;
	NvU32			freq;
	NvBool			bThreadAlive;
	NvBool			show_log;
	NvOdmAccelIntType	IntType;
	NvOdmAccelAxisType	IntMotionAxis;
	NvOdmAccelAxisType	IntTapAxis;
	struct timeval		tv;
	struct accelerometer_data prev_data;
	struct accelerometer_data min_data;
	struct accelerometer_data max_data;
};

struct tegra_acc_device_data *accel_dev;

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
NvBool open_def_odm_accl(void)
{
	NvBool err;

	err = NvOdmAccelOpen(&(accel_dev->hOdmAcr));
	if (!err) {
		pr_err("open_def_odm_accl: NvOdmAccelOpen failed\n");
	}

	return err;
}

/**Function to parse the values sent through sysfs and sets them accordingly
 */
void change_nvodm_accelerometer_settings(NvU32 command, NvS32 value)
{
	switch (command) {
	case COMMAND_LOG:
		accel_dev->show_log = (value == 0 ? NV_FALSE : NV_TRUE);
		break;
	case COMMAND_FREQUENCY:
		if (value <  3) {
			NvOdmAccelSetSampleRate(accel_dev->hOdmAcr,
				NvOdmAccelPower_Low);
			if (NvOdmAccelSetSampleRate(accel_dev->hOdmAcr, value)) {
				accel_dev->freq = value;
			}
		} else {
			NvOdmAccelSetSampleRate(accel_dev->hOdmAcr,
				NvOdmAccelPower_Fullrun);
			if (NvOdmAccelSetSampleRate(accel_dev->hOdmAcr, value)) {
				accel_dev->freq = value;
			}
		}
		break;
	case COMMAND_TIMETAP:
		if( value > 0) {
			NvOdmAccelSetIntEnable(accel_dev->hOdmAcr,
				NvOdmAccelInt_TapThreshold,
				NvOdmAccelAxis_All, 0, NV_TRUE);

			NvOdmAccelSetIntTimeThreshold(accel_dev->hOdmAcr,
				NvOdmAccelInt_TapThreshold, 0, value);
		} else {
			NvOdmAccelSetIntEnable(accel_dev->hOdmAcr,
				NvOdmAccelInt_TapThreshold,
				NvOdmAccelAxis_All, 0, NV_FALSE);
		}
		break;
	case COMMAND_FORCETAP:
		if( value > 0) {
			NvOdmAccelSetIntEnable(accel_dev->hOdmAcr,
				NvOdmAccelInt_TapThreshold,
				NvOdmAccelAxis_All, 0, NV_TRUE);

			NvOdmAccelSetIntForceThreshold(accel_dev->hOdmAcr,
				NvOdmAccelInt_TapThreshold, 0, value);
		} else {
			NvOdmAccelSetIntEnable(accel_dev->hOdmAcr,
				NvOdmAccelInt_TapThreshold,
				NvOdmAccelAxis_All, 0, NV_FALSE);
		}
		break;
	case COMMAND_FORCEMOTION:
		if (value > 0) {
			NvOdmAccelSetIntEnable(accel_dev->hOdmAcr,
				NvOdmAccelInt_MotionThreshold,
				NvOdmAccelAxis_All, 0, NV_TRUE);
			NvOdmAccelSetIntForceThreshold(accel_dev->hOdmAcr,
				NvOdmAccelInt_MotionThreshold, 0, value);
		} else {
			NvOdmAccelSetIntEnable(accel_dev->hOdmAcr,
				NvOdmAccelInt_MotionThreshold,
				NvOdmAccelAxis_All, 0, NV_FALSE);
		}
		break;
	case COMMAND_OPENCLOSE:
		if (value) {
			if (!accel_dev->hOdmAcr)
				open_def_odm_accl();
		} else {
			if (accel_dev->hOdmAcr)
				close_odm_accl();
		}
		break;
	default:
		break;
	}
}


/** Function to give out settings values to the caller modules/application
 * though the use of sysfs. This function is stub as of now, but can be used
 * to return the device specific caps/attributes to the caller modules/app
 *  This function maps to show of sysfs
 */
ssize_t read_sysfs_accel(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	return len;
}

/** Function to take settings values to the caller modules/application
 * though the use of sysfs. This function can be used to set the
 * device specific caps/attributes as specidied by the caller modules/app
 *  This function maps to store of sysfs
 */
ssize_t write_sysfs_accel(struct device *dev,
	struct device_attribute *attr, const char *buffer, size_t count)
{
	NvS32  i = 0;
	NvS8 *input;
	NvS32 value;

	count = count > READ_BUFFER_LENGTH ? READ_BUFFER_LENGTH : count;
	input = kzalloc(((int)count+1), GFP_KERNEL);
	if (!input) {
		goto err_0;
	}

	memcpy(input, buffer, count);

	input[count] = '\0';
	for (i=0; i<NV_ARRAY_SIZE(parameter); i++)  {
		if (count > strlen(parameter[i]) &&
			!strncmp(parameter[i], input, strlen(parameter[i]))) {
			value = simple_strtol(&input[strlen(parameter[i])],
				'\0', 10);
			accel_enum = i;
			goto proceed;
		}
	}
	goto err_0;

proceed:
	if (accel_dev->hOdmAcr)
		change_nvodm_accelerometer_settings(accel_enum, value);
	else
		count = 0;

	kfree(input);
	return count;
err_0:
	if(input)
		kfree(input);
	count = 0;
	return count;

}

/*
 * Function to register sysfs "device" and register the corresponding read
 * write functions
 */

DEVICE_ATTR(tegra_accelerometer, 0777, read_sysfs_accel, write_sysfs_accel);
NvS32 add_sysfs_entry(void)
{
	return device_create_file(&accel_dev->input_dev->dev,
		&dev_attr_tegra_accelerometer);
}

/*
 * Function to register sysfs "device" and register the corresponding read
 * write functions
 */
NvS32 remove_sysfs_entry(void)
{
	device_remove_file(&accel_dev->input_dev->dev,
		&dev_attr_tegra_accelerometer);
	return 0;
}

/**
 * Thread that waits for the interrupt from ODM and then sends out the
 * corresponding events.
 */
static int tegra_acc_thread(void *pdata)
{
	struct tegra_acc_device_data *accelerometer =
		(struct tegra_acc_device_data*)pdata;
	NvS32 x=0, y=0, z=0;
	accelerometer->bThreadAlive = 1;

	while (accelerometer->bThreadAlive) {
		NvOdmAccelWaitInt(accelerometer->hOdmAcr,
			&(accelerometer->IntType),
			&(accelerometer->IntMotionAxis),
			&(accelerometer->IntTapAxis));
		NvOdmAccelGetAcceleration(
			accelerometer->hOdmAcr, &x, &y, &z);
		if (accelerometer->show_log) {
			printk("Accelerometer: x=%d, y=%d, z=%d\n", x, y, z);
		}

		input_report_abs(accelerometer->input_dev, ABS_X, x);
		input_report_abs(accelerometer->input_dev, ABS_Y, y);
		input_report_abs(accelerometer->input_dev, ABS_Z, z);
		input_sync(accelerometer->input_dev);

		accelerometer->prev_data.x = x;
		accelerometer->prev_data.y = y;
		accelerometer->prev_data.z = z;
	}

	return 0;
}

/**
 * All the device spefic initializations happen here.
 */
static NvS32 __init tegra_acc_probe(struct platform_device *pdev)
{
	struct tegra_acc_device_data *accelerometer = NULL;
	struct input_dev *input_dev = NULL;
	NvS32 err;
	NvBool ret;

	accelerometer = kzalloc(sizeof(*accelerometer), GFP_KERNEL);
	if (accelerometer == NULL) {
		err = -ENOMEM;
		pr_err("tegra_acc_probe: Failed to memory\n");
		goto allocate_dev_fail;
	}
	accel_dev = accelerometer;

	input_dev = input_allocate_device();
	if (input_dev == NULL) {
		err = -ENOMEM;
		pr_err("tegra_acc_probe: Failed to allocate input device\n");
		goto allocate_dev_fail;
	}
	g_input_dev = input_dev;

	ret = open_def_odm_accl();
	if (!ret) {
		pr_err("open_def_odm_accl failed\n");
		goto allocate_dev_fail;
	}

	//start the Int thread.
	accelerometer->task = kthread_create(tegra_acc_thread,
		accelerometer, "tegra_acc_thread");
	if (accelerometer->task == NULL) {
		err = -1;
		goto thread_create_failed;
	}
	wake_up_process(accelerometer->task);

	accelerometer->input_dev = input_dev;
	set_bit(EV_SYN, accelerometer->input_dev->evbit);
	set_bit(EV_KEY, accelerometer->input_dev->evbit);
	set_bit(EV_ABS, accelerometer->input_dev->evbit);

	input_set_abs_params(accelerometer->input_dev, ABS_X,
		accelerometer->min_data.x,
		accelerometer->max_data.x, 0, 0);
	input_set_abs_params(accelerometer->input_dev, ABS_Y,
		accelerometer->min_data.y,
		accelerometer->max_data.y, 0, 0);
	input_set_abs_params(accelerometer->input_dev, ABS_Z,
		accelerometer->min_data.z,
		accelerometer->max_data.z, 0, 0);

	platform_set_drvdata(pdev, accelerometer);

	input_dev->name = "accelerometer_tegra";
	err = input_register_device(input_dev);
	if (err) {
		pr_err("tegra_acc_probe: Unable to register %s\
				input device\n", input_dev->name);
		goto input_register_device_failed;
	}

	err = add_sysfs_entry();
	if (err) {
		pr_err("tegra_acc_probe: add_sysfs_entry failed\n");
		goto sysfs_failed;
	}

	printk(KERN_INFO DRIVER_NAME "successfully registered\n");
	return err;

sysfs_failed:
	input_unregister_device(input_dev);
input_register_device_failed:
	accelerometer->bThreadAlive = 0;
thread_create_failed:
	//KillThreadHere!
allocate_dev_fail:
	close_odm_accl();
	input_free_device(input_dev);
	kfree(accelerometer);
	accelerometer = 0;
	err = -ENOMEM;

	return err;
}

static NvS32 tegra_acc_remove(struct platform_device *pdev)
{
	remove_sysfs_entry();
	input_unregister_device(g_input_dev);
	return 0;
}

static int tegra_acc_suspend(struct platform_device *pAccel, pm_message_t state)
{
	if (NvOdmAccelSetPowerState(accel_dev->hOdmAcr, NvOdmAccelPower_Standby))
		return 0;
	else
		return -EIO;
}

static int tegra_acc_resume(struct platform_device *pAccel)
{
	if (NvOdmAccelSetPowerState(accel_dev->hOdmAcr, NvOdmAccelPower_Fullrun))
		return 0;
	else
		return -EIO;
}

static struct platform_driver tegra_acc_driver = {
	.probe	= tegra_acc_probe,
	.remove	= tegra_acc_remove,
	.suspend	= tegra_acc_suspend,
	.resume	= tegra_acc_resume,
	.driver	= {
		.name = "tegra_accelerometer",
	},
};

static NvS32 __devinit tegra_acc_init(void)
{
	return platform_driver_register(&tegra_acc_driver);
}

static void __exit tegra_acc_exit(void)
{
	platform_driver_unregister(&tegra_acc_driver);
}

module_init(tegra_acc_init);
module_exit(tegra_acc_exit);

MODULE_DESCRIPTION("Tegra ODM Accelerometer driver");

