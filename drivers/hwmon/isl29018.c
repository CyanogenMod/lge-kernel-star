/*
 * A hwmon driver for the Ambient light sensor ISL 29018.
 *
 * Ambient light sensor driver for monitoring ambient light intensity in lux.
 *
 * Copyright (c) 2010, NVIDIA Corporation.
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
 * 51 Franklin Street, Fifth Floor, Boston, MA	02110-1301, USA.
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/delay.h>

#include <asm/gpio.h>

/*#define DEBUG           1*/
/*#define VERBOSE_DEBUG   1*/

#define MAX_CONVERSION_TRIAL            5
#define MAX_CONVERION_TIMEOUT           500
#define CONVERSION_TIME                 100
#define RESOLUTION_ADC_BIT              16

#define ISL_29018_REG_ADD_COMMAND1      0x00

#define COMMMAND1_OPMODE_SHIFT          5
#define COMMMAND1_OPMODE_MASK           (7 << COMMMAND1_OPMODE_SHIFT)
#define COMMMAND1_OPMODE_POWER_DOWN     0
#define COMMMAND1_OPMODE_ALS_ONCE       1
#define COMMMAND1_OPMODE_ALS_CONT       5

#define COMMAND1_INT_FLAG_SHIFT         2
#define COMMAND1_INT_FLAG_MASK          (1 << COMMAND1_INT_FLAG_SHIFT)
#define COMMAND1_INT_FLAG_ON            1
#define COMMAND1_INT_FLAG_CLEAR         0

#define COMMAND1_PERSISTANCE_SHIFT      0
#define COMMAND1_PERSISTANCE_MASK      (3 << COMMAND1_PERSISTANCE_SHIFT)
#define COMMAND1_PERSISTANCE_1          0
#define COMMAND1_PERSISTANCE_4          1
#define COMMAND1_PERSISTANCE_8          2
#define COMMAND1_PERSISTANCE_16         3

#define ISL_29018_REG_ADD_COMMANDII     0x01
#define COMMANDII_RESOLUTION_SHIFT      2
#define COMMANDII_RESOLUTION_MASK      (0x3 << COMMANDII_RESOLUTION_SHIFT)
#define COMMANDII_RESOLUTION_ADC_16BIT  0
#define COMMANDII_RESOLUTION_ADC_12BIT  1
#define COMMANDII_RESOLUTION_ADC_8BIT   2
#define COMMANDII_RESOLUTION_ADC_4BIT   3

#define COMMANDII_RANGE_SHIFT             0
#define COMMANDII_RANGE_MASK             (0x3 << COMMANDII_RANGE_SHIFT)
#define COMMANDII_RANGE_ALS_SENSING_1000  0
#define COMMANDII_RANGE_ALS_SENSING_4000  1
#define COMMANDII_RANGE_ALS_SENSING_16000 2
#define COMMANDII_RANGE_ALS_SENSING_64000 3

#define ISL_29018_REG_ADD_DATA_LSB              0x02
#define ISL_29018_REG_ADD_DATA_MSB              0x03
#define ISL_29018_REG_ADD_INT_LOW_THRES_LSB     0x04
#define ISL_29018_REG_ADD_INT_LOW_THRES_MSB     0x05
#define ISL_29018_REG_ADD_INT_HIGH_THRES_LSB    0x06
#define ISL_29018_REG_ADD_INT_HIGH_THRES_MSB    0x07
#define ISL_29018_REG_ADD_TEST                  0x08
#define ISL_29018_MAX_REGS                      ISL_29018_REG_ADD_TEST

struct isl29018_data {
	struct device          *hwmon_dev;
	struct attribute_group attrs;
	struct mutex           lock;
	unsigned int           min_range;
	unsigned int           max_range;
	int                    mode;
	int                    eoc_gpio;
	int                    eoc_irq;
	u32                    adc_bit;
	u8                     reg_cache[ISL_29018_MAX_REGS];
	u16                    bits_per_unit;
};

static int isl29018_probe(struct i2c_client *client,
			      const struct i2c_device_id *id);
static int isl29018_remove(struct i2c_client *client);

static const struct i2c_device_id isl29018_id[] = {
	{"isl29018", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, isl29018_id);

static struct i2c_driver isl29018_driver = {
	.class    = I2C_CLASS_HWMON,
	.driver   = {
		   .name = "isl29018",
		   },
	.probe    = isl29018_probe,
	.remove   = isl29018_remove,
	.id_table = isl29018_id,
};

static bool isl29018_write_data(struct i2c_client *client, u8 reg,
			u8 val, u8 mask, u8 shift)
{
	u8 regval;
	struct i2c_msg msg;
	u8 w_data[2];
	int ret = 0;

	struct isl29018_data *data = i2c_get_clientdata(client);

	regval = data->reg_cache[reg];
	regval &= ~mask;
	regval |= val << shift;
	w_data[0] = reg;
	w_data[1] = regval;

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = 2;
	msg.buf = w_data;
	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0) {
		dev_err(&client->dev, "Write to device fails status %x\n", ret);
		return false;
	}
	data->reg_cache[reg] = regval;
	return true;
}

static bool isl29018_read_data(struct i2c_client *client, u8 reg, u8 length,
				u8 * buffer)
{
	struct i2c_msg msg[2];
	u8 w_data[2];
	int ret = 0;

	w_data[0] = reg;

	msg[0].addr = client->addr;
	msg[0].flags = I2C_M_NOSTART;	/* set repeated start and write */
	msg[0].len = 1;
	msg[0].buf = w_data;

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = length;
	msg[1].buf = buffer;

	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret < 0) {
		dev_err(&client->dev, "Read from device fails.\n");
		return false;
	}
	return true;
}

static bool isl29018_set_range(struct i2c_client *client, u32 min, u32 max)
{
	u32 max_ranges[] = { 1000, 4000, 16000, 64000 };
	int i;
	for (i = 0; i < (ARRAY_SIZE(max_ranges) -1); ++i) {
		if (max < max_ranges[i])
			break;
	}

	return isl29018_write_data(client, ISL_29018_REG_ADD_COMMANDII,
				i, COMMANDII_RANGE_MASK,
				COMMANDII_RANGE_SHIFT);
}

static bool isl29018_read_lux(struct i2c_client *client, int *lux_val)
{
	bool status;
	u8 data_lsb;
	u8 data_msb;

	dev_vdbg(&client->dev, "%s()\n", __func__);
	/* Write once ALS */
	status = isl29018_write_data(client, ISL_29018_REG_ADD_COMMAND1,
				COMMMAND1_OPMODE_ALS_ONCE,
				COMMMAND1_OPMODE_MASK,
				COMMMAND1_OPMODE_SHIFT);
	if (!status) {
		dev_err(&client->dev, "Error in setting operating mode\n");
		return -EINVAL;
	}
	msleep(CONVERSION_TIME);
	status = isl29018_read_data(client, ISL_29018_REG_ADD_DATA_LSB, 1,
				&data_lsb);
	if (!status) {
		dev_err(&client->dev, "Error in reading LSB DATA\n");
		return false;
	}

	status = isl29018_read_data(client, ISL_29018_REG_ADD_DATA_MSB, 1,
				&data_msb);
	if (!status) {
		dev_err(&client->dev, "Error in reading MSB DATA\n");
		return false;
	}
	dev_vdbg(&client->dev, "MSB 0x%x and LSB 0x%x\n", data_msb, data_lsb);
	*lux_val = (int) (((data_msb << 8) & 0xFF00) | (data_lsb & 0xFF));
	return true;
}

static ssize_t show_mode(struct device *dev,
			struct device_attribute *devattr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct isl29018_data *data = i2c_get_clientdata(client);
	dev_vdbg(dev, "%s()\n", __func__);
	return sprintf(buf, "%d\n", data->mode);
}

static ssize_t store_mode(struct device *dev,
			struct device_attribute *devattr,
			const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct isl29018_data *data = i2c_get_clientdata(client);
	long oval;
	int mode;
	bool status;

	dev_vdbg(dev, "%s()\n", __func__);
	if (strict_strtol(buf, 10, &oval))
		return -EINVAL;

	mutex_lock(&data->lock);
	mode = (int) oval;
	if ((mode < 0) || (mode > 1)) {
		dev_err(&client->dev,
			"Operating mode value is not supported\n");
		mutex_unlock(&data->lock);
		return -EINVAL;
	}

	if (data->mode != mode) {
		status = isl29018_write_data(client, ISL_29018_REG_ADD_COMMAND1,
					mode, COMMMAND1_OPMODE_MASK,
					COMMMAND1_OPMODE_SHIFT);
		if (status && !mode) {
			status = isl29018_write_data(client,
					ISL_29018_REG_ADD_COMMAND1,
					COMMAND1_PERSISTANCE_8,
					COMMAND1_PERSISTANCE_MASK,
					COMMAND1_PERSISTANCE_SHIFT);
			if (status)
				status = isl29018_set_range(client,
						data->min_range,
						data->max_range);
		}

		if (!status) {
			dev_err(&client->dev,
				"Error in setting operating mode\n");
			mutex_unlock(&data->lock);
			return -EINVAL;
		}
		data->mode = mode;
	}
	mutex_unlock(&data->lock);
	return count;
}

static ssize_t show_min_range(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct isl29018_data *data = i2c_get_clientdata(client);

	dev_vdbg(dev, "%s()\n", __func__);
	return sprintf(buf, "%u\n", data->min_range);
}

static ssize_t store_min_range(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct isl29018_data *data = i2c_get_clientdata(client);
	unsigned long min_val;

	dev_vdbg(dev, "%s()\n", __func__);

	if (strict_strtoul(buf, 10, &min_val))
		return -EINVAL;

	mutex_lock(&data->lock);
	/* There is no need to set min range */
	data->min_range = (unsigned int) min_val;
	mutex_unlock(&data->lock);
	return count;
}

static ssize_t show_max_range(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct isl29018_data *data = i2c_get_clientdata(client);

	dev_vdbg(dev, "%s()\n", __func__);
	return sprintf(buf, "%u\n", data->max_range);
}

static ssize_t store_max_range(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct isl29018_data *data = i2c_get_clientdata(client);
	bool status;
	unsigned long max_val;

	dev_vdbg(dev, "%s()\n", __func__);

	if (strict_strtoul(buf, 10, &max_val))
		return -EINVAL;

	mutex_lock(&data->lock);
	status = isl29018_set_range(client, data->min_range, max_val);
	if (!status) {
		mutex_unlock(&data->lock);
		dev_err(dev, "Error in setting max range\n");
		return -EINVAL;
	}
	data->max_range = (unsigned int) max_val;
	data->bits_per_unit = ((1 << data->adc_bit) + data->max_range - 1) /
					data->max_range;
	mutex_unlock(&data->lock);
	return count;
}

static ssize_t show_show_lux(struct device *dev,
		struct device_attribute *devattr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct isl29018_data *data = i2c_get_clientdata(client);
	int luxval = 0;
	bool status;
	int trial_count = MAX_CONVERSION_TRIAL;

	dev_vdbg(dev, "%s()\n", __func__);
	if (!data) {
		dev_err(&client->dev, "No device found\n");
		return -ENODEV;
	}

	mutex_lock(&data->lock);
	if (data->mode == 0) {
		dev_err(&client->dev, "Operating mode is in power down mode\n");
		return -EBUSY;
	}
	while (trial_count--) {
		status = isl29018_read_lux(client, &luxval);
		if (!status) {
			dev_err(&client->dev, "Error Reading flux trial %d\n",
				trial_count);
			continue;
		}
		break;
	}

	if (!status) {
		dev_err(&client->dev, "Tried maximum trials %d\n",
			MAX_CONVERSION_TRIAL);
		mutex_unlock(&data->lock);
		return -EBUSY;
	}

	luxval = (luxval / data->bits_per_unit);
	mutex_unlock(&data->lock);
	return sprintf(buf, "%d\n", luxval);
}

static SENSOR_DEVICE_ATTR(min_range, S_IRUGO | S_IWUSR,
				show_min_range, store_min_range, 1);
static SENSOR_DEVICE_ATTR(max_range, S_IRUGO | S_IWUSR,
				show_max_range, store_max_range, 2);
static SENSOR_DEVICE_ATTR(mode, S_IRUGO | S_IWUSR,
				show_mode, store_mode, 3);
static SENSOR_DEVICE_ATTR(show_lux, S_IRUGO, show_show_lux, NULL, 4);

static struct attribute *isl29018_attr[] = {
	&sensor_dev_attr_min_range.dev_attr.attr,
	&sensor_dev_attr_max_range.dev_attr.attr,
	&sensor_dev_attr_mode.dev_attr.attr,
	&sensor_dev_attr_show_lux.dev_attr.attr,
	NULL
};

static void init_adc_resolution(struct isl29018_data *data)
{
	u8 adc_resol[] = {16, 12, 8, 4};
	int i;
	for (i =0; i < ARRAY_SIZE(adc_resol); ++i) {
		if (adc_resol[i] != data->adc_bit)
			continue;
		data->reg_cache[ISL_29018_REG_ADD_COMMANDII] &=
					~COMMANDII_RESOLUTION_MASK;
		data->reg_cache[ISL_29018_REG_ADD_COMMANDII] |=
					(i << COMMANDII_RESOLUTION_SHIFT);
	}
}
static int isl29018_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{
	struct isl29018_data *data;
	int err;

	data = kzalloc(sizeof (struct isl29018_data), GFP_KERNEL);
	if (!data) {
		dev_err(&client->dev, "Memory allocation fails\n");
		err = -ENOMEM;
		goto exit;
	}

	i2c_set_clientdata(client, data);
	mutex_init(&data->lock);

	data->eoc_irq = client->irq;
	if (!data->eoc_irq) {
		data->eoc_gpio = irq_to_gpio(client->irq);

		err = gpio_request(data->eoc_gpio, "isl29018");
		if (err < 0) {
			dev_err(&client->dev, "failed to request GPIO %d,"
				" error %d\n", data->eoc_gpio, err);
			goto exit_free;
		}

		err = gpio_direction_input(data->eoc_gpio);
		if (err < 0) {
			dev_err(&client->dev, "Failed to configure input"
				"direction for GPIO %d, error %d\n",
				data->eoc_gpio, err);
			gpio_free(data->eoc_gpio);
			goto exit_gpio;
		}
	}
	data->min_range = 1;
	data->max_range = 1000;
	data->mode = 0;
	data->adc_bit = RESOLUTION_ADC_BIT;
	data->bits_per_unit = ((1 << data->adc_bit) + data->max_range - 1) /
					data->max_range;
	init_adc_resolution(data);

	/* Register sysfs hooks */
	data->attrs.attrs = isl29018_attr;
	err = sysfs_create_group(&client->dev.kobj, &data->attrs);
	if (err) {
		dev_err(&client->dev, "Not able to create the sysfs\n");
		goto exit_free;
	}

	data->hwmon_dev = hwmon_device_register(&client->dev);
	if (IS_ERR(data->hwmon_dev)) {
		dev_err(&client->dev, "hwmon registration fails\n");
		err = PTR_ERR(data->hwmon_dev);
		goto exit_remove;
	}

	return 0;

exit_remove:
	sysfs_remove_group(&client->dev.kobj, &data->attrs);
exit_gpio:
	if (!data->eoc_irq)
		gpio_free(data->eoc_gpio);
exit_free:
	kfree(data);
exit:
	return err;
}

static int isl29018_remove(struct i2c_client *client)
{
	struct isl29018_data *data = i2c_get_clientdata(client);

	dev_dbg(&client->dev, "%s()\n", __func__);
	hwmon_device_unregister(data->hwmon_dev);
	sysfs_remove_group(&client->dev.kobj, &data->attrs);
	if (!data->eoc_irq)
		gpio_free(data->eoc_gpio);
	kfree(data);
	return 0;
}

static int __init isl29018_init(void)
{
	pr_info("%s()\n", __func__);
	return i2c_add_driver(&isl29018_driver);
}

static void __exit isl29018_exit(void)
{
	pr_info("%s()\n", __func__);
	i2c_del_driver(&isl29018_driver);
}

module_init(isl29018_init);
module_exit(isl29018_exit);
