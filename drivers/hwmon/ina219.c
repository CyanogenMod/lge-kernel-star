/*
 * ina219.c - driver for TI INA219 current / power monitor sensor
 *
 * Copyright (c) 2011, NVIDIA Corporation.
 *
 * The INA219 is a sensor chip made by Texas Instruments. It measures
 * power, voltage and current on a power rail.
 * Complete datasheet can be obtained from website:
 *   http://focus.ti.com/lit/ds/symlink/ina219.pdf
 *
 * This program is free software. you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation.
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <linux/sysfs.h>
#include <linux/kobject.h>
#include <linux/hrtimer.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/device.h>
#include <linux/sysdev.h>
#include "linux/ina219.h"
#include <linux/init.h>
#include <linux/hwmon-sysfs.h>
#include <linux/hwmon.h>

#define DRIVER_NAME "ina219"

/* INA219 register offsets */
#define INA219_CONFIG	0
#define INA219_SHUNT	1
#define INA219_VOLTAGE	2
#define INA219_POWER	3
#define INA219_CURRENT	4
#define INA219_CAL	5

/*
	INA219 Sensor defines
	Config info for ina219s
	D15   D14  D13  D12 D11 D10 D9 D8 D7 D6 D5 D4 D3 D2 D1 D0
	rst  BRNG  PG1 PG0  BADC4 .-.BADC1 SADC4 - SADC1 MODE3 - MODE1
	reset D15
	bus_range=0 (d13)
	pga_gain=0  (D12:D11)
	bus_adc_setting=0x3     (d10:D7) 12-bit w/o oversampling (532uS)
	shunt_adc_setting=0xb    (D6:D3) 8x oversampling (4.26ms)
	mode=0x7                 (D2:D0) continuous shunt & bus
*/
#define INA219_CONFIG_DATA 0x1df
#define INA219_RESET 0x8000

struct power_mon_data {
	s32 voltage;
	s32 currentInMillis;
	s32 power;
};

struct ina219_data {
	struct device *hwmon_dev;
	struct i2c_client *client;
	struct ina219_platform_data *pInfo;
	struct power_mon_data pm_data;
	struct mutex mutex;
};

/* Set non-zero to enable debug prints */
#define INA219_DEBUG_PRINTS 0

#if INA219_DEBUG_PRINTS
#define DEBUG_INA219(x) printk x
#else
#define DEBUG_INA219(x)
#endif

static s16 reorder_bytes(s16 a)
{
	s16 ret = ((a >> 8) & 0xff) | ((a & 0xff) << 8);
	return ret;
}

/* set ina219 to power down mode */
static s32 power_down_INA219(struct i2c_client *client)
{
	s32 retval;
	retval = i2c_smbus_write_word_data(client, INA219_CONFIG, 0);
	if (retval < 0)
		dev_err(&client->dev, "power down failure sts: 0x%x\n", retval);
	return retval;
}

static s32 show_rail_name(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ina219_data *data = i2c_get_clientdata(client);
	return sprintf(buf, "%s\n", data->pInfo->rail_name);
}

static s32 show_voltage(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ina219_data *data = i2c_get_clientdata(client);
	s32 retval;
	s32 voltage_mV;

	/* fill config data */
	retval = i2c_smbus_write_word_data(client, INA219_CONFIG,
		reorder_bytes(INA219_CONFIG_DATA));
	if (retval < 0) {
		dev_err(dev, "config data write failed sts: 0x%x\n", retval);
		goto error;
	}

	/* fill calibration data */
	retval = i2c_smbus_write_word_data(client, INA219_CAL,
		reorder_bytes(data->pInfo->calibration_data));
	if (retval < 0) {
		dev_err(dev, "calib data write failed sts: 0x%x\n", retval);
		goto error;
	}

	/* getting voltage readings in milli volts*/
	voltage_mV =
		reorder_bytes(i2c_smbus_read_word_data(client,
			INA219_VOLTAGE));
	DEBUG_INA219(("Ina219 voltage reg Value: 0x%x\n", voltage_mV));
	if (voltage_mV < 0)
		goto error;
	voltage_mV = voltage_mV >> 1;
	DEBUG_INA219(("Ina219 voltage in mv: %d\n", voltage_mV));

	/* set ina219 to power down mode */
	retval = power_down_INA219(client);
	if (retval < 0)
		goto error;

	DEBUG_INA219(("%s volt = %d\n", __func__, voltage_mV));
	return sprintf(buf, "%d mV\n", voltage_mV);
error:
	dev_err(dev, "%s: failed\n", __func__);
	return retval;
}


static s32 show_power(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ina219_data *data = i2c_get_clientdata(client);
	s32 retval;
	s32 power_mW;
	s32 voltage_mV;
	s32 overflow, conversion;

	/* fill config data */
	retval = i2c_smbus_write_word_data(client, INA219_CONFIG,
		reorder_bytes(INA219_CONFIG_DATA));
	if (retval < 0) {
		dev_err(dev, "config data write failed sts: 0x%x\n", retval);
		goto error;
	}

	/* fill calib data */
	retval = i2c_smbus_write_word_data(client, INA219_CAL,
		reorder_bytes(data->pInfo->calibration_data));
	if (retval < 0) {
		dev_err(dev, "calibration data write failed sts: 0x%x\n",
			retval);
		goto error;
	}

	/* check if the readings are valid */
	do {
		/* read power register to clear conversion bit */
		retval = reorder_bytes(i2c_smbus_read_word_data(client,
			INA219_POWER));
		if (retval < 0) {
			dev_err(dev, "CNVR bit clearing failure sts: 0x%x\n",
				retval);
			goto error;
		}

		voltage_mV =
			reorder_bytes(i2c_smbus_read_word_data(client,
				INA219_VOLTAGE));
		DEBUG_INA219(("Ina219 voltage reg Value: 0x%x\n", voltage_mV));
		overflow = voltage_mV & 1;
		if (overflow) {
			dev_err(dev, "overflow error\n");
			return 0;
		}
		conversion = (voltage_mV >> 1) & 1;
		DEBUG_INA219(("\n ina219 CNVR value:%d", conversion));
	} while (!conversion);

	/* getting power readings in milli watts*/
	power_mW = reorder_bytes(i2c_smbus_read_word_data(client,
		INA219_POWER));
	DEBUG_INA219(("Ina219 power Reg: 0x%x\n", power_mW));
	power_mW *= data->pInfo->power_lsb;
	DEBUG_INA219(("Ina219 power Val: %d\n", power_mW));
	if (power_mW < 0)
		goto error;

	/* set ina219 to power down mode */
	retval = power_down_INA219(client);
	if (retval < 0)
		goto error;

	DEBUG_INA219(("%s pow = %d\n", __func__, power_mW));
	return sprintf(buf, "%d mW\n", power_mW);
error:
	dev_err(dev, "%s: failed\n", __func__);
	return retval;
}

static s32 show_current(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ina219_data *data = i2c_get_clientdata(client);
	s32 retval;
	s32 current_mA;
	s32 voltage_mV;
	s32 overflow, conversion;

	/* fill config data */
	retval = i2c_smbus_write_word_data(client, INA219_CONFIG,
		reorder_bytes(INA219_CONFIG_DATA));
	if (retval < 0) {
		dev_err(dev, "config data write failed sts: 0x%x\n", retval);
		goto error;
	}

	/* fill calib data */
	retval = i2c_smbus_write_word_data(client, INA219_CAL,
		reorder_bytes(data->pInfo->calibration_data));
	if (retval < 0) {
		dev_err(dev, "calibration data write failed sts: 0x%x\n",
			retval);
		goto error;
	}

	/* check if the readings are valid */
	do {
		/* read power register to clear conversion bit */
		retval = reorder_bytes(i2c_smbus_read_word_data(client,
			INA219_POWER));
		if (retval < 0) {
			dev_err(dev, "CNVR bit clearing failure sts: 0x%x\n",
				retval);
			goto error;
		}

		voltage_mV =
			reorder_bytes(i2c_smbus_read_word_data(client,
				INA219_VOLTAGE));
		DEBUG_INA219(("Ina219 voltage reg Value: 0x%x\n", voltage_mV));
		overflow = voltage_mV & 1;
		if (overflow) {
			dev_err(dev, "overflow error\n");
			return 0;
		}
		conversion = (voltage_mV >> 1) & 1;
		DEBUG_INA219(("\n ina219 CNVR value:%d", conversion));
	} while (!conversion);

	/* getting current readings in milli amps*/
	current_mA = reorder_bytes(i2c_smbus_read_word_data(client,
		INA219_CURRENT));
	DEBUG_INA219(("Ina219 current Reg: 0x%x\n", current_mA));
	if (current_mA < 0)
		goto error;
	current_mA =
		(current_mA * data->pInfo->power_lsb) / data->pInfo->divisor;
	DEBUG_INA219(("Ina219 current Value: %d\n", current_mA));

	/* set ina219 to power down mode */
	retval = power_down_INA219(client);
	if (retval < 0)
		goto error;


	DEBUG_INA219(("%s current = %d\n", __func__, current_mA));
	return sprintf(buf, "%d mA\n", current_mA);
error:
	dev_err(dev, "%s: failed\n", __func__);
	return retval;
}

static struct sensor_device_attribute ina219[] = {
	SENSOR_ATTR(rail_name, S_IRUGO, show_rail_name, NULL, 0),
	SENSOR_ATTR(in1_input, S_IRUGO, show_voltage, NULL, 0),
	SENSOR_ATTR(curr1_input, S_IRUGO, show_current, NULL, 0),
	SENSOR_ATTR(power1_input, S_IRUGO, show_power, NULL, 0),
};

static int __devinit ina219_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct ina219_data *data;
	int err;
	u8 i;
	data = kzalloc(sizeof(struct ina219_data), GFP_KERNEL);
	if (!data) {
		err = -ENOMEM;
		goto exit;
	}

	i2c_set_clientdata(client, data);
	data->pInfo = client->dev.platform_data;
	mutex_init(&data->mutex);
	/* reset ina219 */
	err = i2c_smbus_write_word_data(client, INA219_CONFIG,
		reorder_bytes(INA219_RESET));
	if (err < 0) {
		dev_err(&client->dev, "ina219 reset failure status: 0x%x\n",
			err);
		goto exit_free;
	}

	for (i = 0; i < ARRAY_SIZE(ina219); i++) {
		err = device_create_file(&client->dev, &ina219[i].dev_attr);
		if (err) {
			dev_err(&client->dev, "device_create_file failed.\n");
			goto exit_free;
		}
	}

	data->hwmon_dev = hwmon_device_register(&client->dev);
	if (IS_ERR(data->hwmon_dev)) {
		err = PTR_ERR(data->hwmon_dev);
		goto exit_remove;
	}

	/* set ina219 to power down mode */
	err = power_down_INA219(client);
	if (err < 0)
		goto exit_remove;

	return 0;

exit_remove:
	for (i = 0; i < ARRAY_SIZE(ina219); i++)
		device_remove_file(&client->dev, &ina219[i].dev_attr);
exit_free:
	kfree(data);
exit:
	return err;
}

static int __devexit ina219_remove(struct i2c_client *client)
{
	u8 i;
	struct ina219_data *data = i2c_get_clientdata(client);
	hwmon_device_unregister(data->hwmon_dev);
	for (i = 0; i < ARRAY_SIZE(ina219); i++)
		device_remove_file(&client->dev, &ina219[i].dev_attr);
	kfree(data);
	return 0;
}

static const struct i2c_device_id ina219_id[] = {
	{DRIVER_NAME, 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, ina219_id);

static struct i2c_driver ina219_driver = {
	.class		= I2C_CLASS_HWMON,
	.driver = {
		.name	= DRIVER_NAME,
	},
	.probe		= ina219_probe,
	.remove		= __devexit_p(ina219_remove),
	.id_table	= ina219_id,
};

static int __init ina219_init(void)
{
	return i2c_add_driver(&ina219_driver);
}

static void __exit ina219_exit(void)
{
	i2c_del_driver(&ina219_driver);
}

module_init(ina219_init);
module_exit(ina219_exit);
MODULE_LICENSE("GPL");
