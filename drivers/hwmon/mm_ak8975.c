/*
 * A hwmon driver for the magnetometer AK8975.
 *
 * Magnetic compass sensor driver for monitoring magnetic flux information.
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
#include <linux/kernel.h>
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
#define CONVERSION_DONE_POLL_TIME       10

#define AK_8975_REG_ADD_WIA             0x00
#define MM_AK_8975_DEVICE_ID            0x48
#define AK_8975_REG_ADD_INFO            0x01

#define AK_8975_REG_ADD_ST1             0x02
#define REG_ST1_DRDY_SHIFT              0
#define REG_ST1_DRDY_MASK               (1 << REG_ST1_DRDY_SHIFT)

#define AK_8975_REG_ADD_HXL             0x03
#define AK_8975_REG_ADD_HXH             0x04
#define AK_8975_REG_ADD_HYL             0x05
#define AK_8975_REG_ADD_HYH             0x06
#define AK_8975_REG_ADD_HZL             0x07
#define AK_8975_REG_ADD_HZH             0x08
#define AK_8975_REG_ADD_ST2             0x09
#define REG_ST2_DERR_SHIFT              2
#define REG_ST2_DERR_MASK               (1 << REG_ST2_DERR_SHIFT)

#define REG_ST2_HOFL_SHIFT              3
#define REG_ST2_HOFL_MASK               (1 << REG_ST2_HOFL_SHIFT)

#define AK_8975_REG_ADD_CNTL            0x0A
#define REG_CNTL_MODE_SHIFT             0
#define REG_CNTL_MODE_MASK              (0xF << REG_CNTL_MODE_SHIFT)
#define REG_CNTL_MODE_POWER_DOWN        0
#define REG_CNTL_MODE_ONCE              1
#define REG_CNTL_MODE_SELF_TEST         8
#define REG_CNTL_MODE_FUSE_ROM          0xF

#define AK_8975_REG_ADD_RSVC            0x0B
#define AK_8975_REG_ADD_ASTC            0x0C
#define AK_8975_REG_ADD_TS1             0x0D
#define AK_8975_REG_ADD_TS2             0x0E
#define AK_8975_REG_ADD_I2CDIS          0x0F
#define AK_8975_REG_ADD_ASAX            0x10
#define AK_8975_REG_ADD_ASAY            0x11
#define AK_8975_REG_ADD_ASAZ            0x12

#define AK_8975_MAX_REGS AK_8975_REG_ADD_ASAZ

struct mm_ak8975_data {
	struct device           *hwmon_dev;
	struct attribute_group  attrs;
	struct mutex            lock;
	u8                      asa[3];
	unsigned long           mode;
	u8                      reg_cache[AK_8975_MAX_REGS];
	int                     eoc_gpio;
	int                     eoc_irq;
};

static int mm_ak8975_probe(struct i2c_client *client,
			   const struct i2c_device_id *id);
static int mm_ak8975_remove(struct i2c_client *client);
static int mm_ak8975_suspend(struct i2c_client *client, pm_message_t mesg);
static int mm_ak8975_resume(struct i2c_client *client);

static const struct i2c_device_id mm_ak8975_id[] = {
	{"mm_ak8975", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, mm_ak8975_id);

static struct i2c_driver mm_ak8975_driver = {
	.class		= I2C_CLASS_HWMON,
	.driver = {
		.name	= "mm_ak8975",
	},
	.probe		= mm_ak8975_probe,
	.remove		= mm_ak8975_remove,
	.resume         = mm_ak8975_resume,
	.suspend        = mm_ak8975_suspend,
	.id_table	= mm_ak8975_id,
};

static bool ak8975_write_data(struct i2c_client *client,
		u8 reg, u8 val, u8 mask, u8 shift)
{
	u8 regval;
	struct i2c_msg msg;
	u8 w_data[2];
	int ret = 0;

	struct mm_ak8975_data *data = i2c_get_clientdata(client);

	regval = data->reg_cache[reg];
	regval &= ~mask;
	regval |= val << shift;
	w_data[0] = reg;
	w_data[1] = regval;

	dev_vdbg(&client->dev,"%s(): Writing Reg 0x%x to value 0x%x\n",
			__func__,w_data[0], w_data[1]);
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

static bool ak8975_read_data(struct i2c_client *client,
		u8 reg, u8 length, u8 * buffer)
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

static bool ak8975_read_flux(struct i2c_client *client,
		 int *flux_x, int *flux_y, int *flux_z)
{
	struct mm_ak8975_data *data = i2c_get_clientdata(client);
	u32 timeout_ms = MAX_CONVERION_TIMEOUT;
	u16 meas_reg[3];
	s16 meas_val[3];
	u8 read_status;
	bool status;
	int state;

	dev_vdbg(&client->dev, "%s()\n", __func__);
	status = ak8975_write_data(client, AK_8975_REG_ADD_CNTL,
				   REG_CNTL_MODE_ONCE, REG_CNTL_MODE_MASK,
				   REG_CNTL_MODE_SHIFT);
	if (!status) {
		dev_err(&client->dev, "Error in setting operating mode\n");
		return false;
	}

	while (timeout_ms) {
		msleep(CONVERSION_DONE_POLL_TIME);
		state = (gpio_get_value(data->eoc_gpio) ? 1 : 0);
		if (state)
			break;
		timeout_ms -= CONVERSION_DONE_POLL_TIME;
	}
	if (!timeout_ms) {
		dev_err(&client->dev, "Conversion timeout happend\n");
		return false;
	}
	status = ak8975_read_data(client, AK_8975_REG_ADD_ST1, 1, &read_status);
	if (!status) {
		dev_err(&client->dev, "Error in reading ST1\n");
		return false;
	}

	if (read_status & REG_ST1_DRDY_MASK) {
		status = ak8975_read_data(client, AK_8975_REG_ADD_ST2,
					  1, &read_status);
		if (!status) {
			dev_err(&client->dev, "Error in reading ST2\n");
			return false;
		}
		if (read_status & (REG_ST2_DERR_MASK | REG_ST2_HOFL_MASK)) {
			dev_err(&client->dev, "ST2 status error 0x%x\n",
				read_status);
			return false;
		}
	}

	status = ak8975_read_data(client, AK_8975_REG_ADD_HXL, 6,
				  (u8 *) meas_reg);
	if (!status) {
		dev_err(&client->dev, "Read axis data fails\n");
		return false;
	}
	meas_val[0] = (s16) (le16_to_cpu(meas_reg[0]));
	meas_val[1] = (s16) (le16_to_cpu(meas_reg[1]));
	meas_val[2] = (s16) (le16_to_cpu(meas_reg[2]));

	/* The value can be range of -4096 to 4096 */
	meas_val[0] = clamp_t(s16, meas_val[0], -4096, 4096);
	meas_val[1] = clamp_t(s16, meas_val[1], -4096, 4096);
	meas_val[2] = clamp_t(s16, meas_val[2], -4096, 4096);

	/* Hadj = (H*(Asa+128))/256 */
	meas_val[0] = (meas_val[0] * (data->asa[0] + 128)) >> 8;
	meas_val[1] = (meas_val[1] * (data->asa[1] + 128)) >> 8;
	meas_val[2] = (meas_val[2] * (data->asa[2] + 128)) >> 8;

	/* Return  in uT */
	*flux_x = (int) ((meas_val[0] * 3)/10);
	*flux_y = (int) ((meas_val[1] * 3)/10);
	*flux_z = (int) ((meas_val[2] * 3)/10);
	return true;
}

static int ak8975_init(struct i2c_client *client)
{
	struct mm_ak8975_data *data = i2c_get_clientdata(client);
	u8 device_id;
	u8 buffer[3];
	bool status;

	dev_vdbg(&client->dev, "%s()\n", __func__);

	status = ak8975_read_data(client, AK_8975_REG_ADD_WIA, 1, &device_id);
	if ((!status) || (device_id != MM_AK_8975_DEVICE_ID)) {
		dev_err(&client->dev, "Device mm_ak8975 not found\n");
		return -ENODEV;
	}

	/* Write the fused rom access mode */
	status = ak8975_write_data(client, AK_8975_REG_ADD_CNTL,
				REG_CNTL_MODE_FUSE_ROM, REG_CNTL_MODE_MASK,
				REG_CNTL_MODE_SHIFT);
	if (!status) {
		dev_err(&client->dev, "Error in setting fuse access mode\n");
		return false;
	}
	/* Get asa data and store in the device data */
	status = ak8975_read_data(client, AK_8975_REG_ADD_ASAX, 3, buffer);
	if (!status) {
		dev_err(&client->dev, "Not able to read asa data\n");
		return -ENODEV;
	}
	data->asa[0] = buffer[0] & 0xFF;
	data->asa[1] = buffer[1] & 0xFF;
	data->asa[2] = buffer[2] & 0xFF;
	return 0;
}

static ssize_t show_mode(struct device *dev, struct device_attribute *devattr,
			 char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mm_ak8975_data *data = i2c_get_clientdata(client);
	dev_vdbg(dev, "%s()\n", __func__);
	return sprintf(buf, "%lu\n", data->mode);
}

static ssize_t store_mode(struct device *dev, struct device_attribute *devattr,
			const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mm_ak8975_data *data = i2c_get_clientdata(client);
	unsigned long oval;
	bool status;

	dev_vdbg(dev, "%s()\n", __func__);
	if (strict_strtol(buf, 10, &oval))
		return -EINVAL;

	mutex_lock(&data->lock);
	if ((oval < 0) || (oval > 1)) {
		dev_err(&client->dev, "mode value is not supported\n");
		mutex_unlock(&data->lock);
		return -EINVAL;
	}

	if (data->mode != oval) {
		status = ak8975_write_data(client, AK_8975_REG_ADD_CNTL,
					   (u8) oval, REG_CNTL_MODE_MASK,
					   REG_CNTL_MODE_SHIFT);

		if (!status) {
			dev_err(&client->dev, "Error in setting mode\n");
			mutex_unlock(&data->lock);
			return -EINVAL;
		}
		data->mode = oval;
	}
	mutex_unlock(&data->lock);
	return count;
}

static ssize_t show_show_flux(struct device *dev,
			struct device_attribute *devattr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mm_ak8975_data *data = i2c_get_clientdata(client);
	int x_luxval = 0;
	int y_luxval = 0;
	int z_luxval = 0;
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
		mutex_unlock(&data->lock);
		return -EBUSY;
	}
	while (trial_count--) {
		status = ak8975_read_flux(client, &x_luxval, &y_luxval,
					  &z_luxval);
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

	mutex_unlock(&data->lock);
	return sprintf(buf, "%d %d %d\n", x_luxval, y_luxval, z_luxval);
}

static SENSOR_DEVICE_ATTR(mode, S_IRUGO | S_IWUSR, show_mode, store_mode, 1);
static SENSOR_DEVICE_ATTR(show_flux, S_IRUGO, show_show_flux, NULL, 2);

static struct attribute *mm_ak8975_attr[] = {
	&sensor_dev_attr_mode.dev_attr.attr,
	&sensor_dev_attr_show_flux.dev_attr.attr,
	NULL
};

static int mm_ak8975_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct mm_ak8975_data *data;
	int err;

	dev_dbg(&client->dev, "%s()\n", __func__);

	data = kzalloc(sizeof (struct mm_ak8975_data), GFP_KERNEL);
	if (!data) {
		dev_err(&client->dev, "Memory allocation fails\n");
		err = -ENOMEM;
		goto exit;
	}

	i2c_set_clientdata(client, data);
	mutex_init(&data->lock);

	data->eoc_irq = client->irq;
	data->eoc_gpio = irq_to_gpio(client->irq);

	err = gpio_request(data->eoc_gpio, "ak_8975");
	if (err < 0) {
		dev_err(&client->dev, "failed to request GPIO %d, error %d\n",
			data->eoc_gpio, err);
		goto exit_free;
	}

	err = gpio_direction_input(data->eoc_gpio);
	if (err < 0) {
		dev_err(&client->dev, "Failed to configure input direction for"
			" GPIO %d, error %d\n", data->eoc_gpio, err);
		gpio_free(data->eoc_gpio);
		goto exit_gpio;
	}

	err = ak8975_init(client);
	if (err < 0) {
		dev_err(&client->dev, "MM AK8975 initialization fails\n");
		goto exit_gpio;
	}

	dev_info(&client->dev, "%s chip found Gpio %d\n", client->name,
		 data->eoc_gpio);

	/* Register sysfs hooks */
	data->attrs.attrs = mm_ak8975_attr;
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
	gpio_free(data->eoc_gpio);
exit_free:
	kfree(data);
exit:
	return err;
}

static int mm_ak8975_remove(struct i2c_client *client)
{
	struct mm_ak8975_data *data = i2c_get_clientdata(client);

	dev_dbg(&client->dev, "%s()\n", __func__);
	hwmon_device_unregister(data->hwmon_dev);
	sysfs_remove_group(&client->dev.kobj, &data->attrs);
	gpio_free(data->eoc_gpio);
	kfree(data);
	return 0;
}

static int mm_ak8975_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct mm_ak8975_data *data = i2c_get_clientdata(client);
	bool status;

	dev_dbg(&client->dev, "%s()\n", __func__);

	mutex_lock(&data->lock);
	status = ak8975_write_data(client, AK_8975_REG_ADD_CNTL,
					   0x0, REG_CNTL_MODE_MASK,
					   REG_CNTL_MODE_SHIFT);
	if (!status) {
		dev_err(&client->dev, "Error in setting fuse access mode\n");
		mutex_unlock(&data->lock);
		return -EBUSY;
	}
	mutex_unlock(&data->lock);
	return 0;
}
static int mm_ak8975_resume(struct i2c_client *client)
{
	struct mm_ak8975_data *data = i2c_get_clientdata(client);
	bool status;

	dev_dbg(&client->dev, "%s()\n", __func__);

	mutex_lock(&data->lock);
	status = ak8975_write_data(client, AK_8975_REG_ADD_CNTL,
					   data->mode, REG_CNTL_MODE_MASK,
					   REG_CNTL_MODE_SHIFT);
	if (!status) {
		dev_err(&client->dev, "Error in setting fuse access mode\n");
		mutex_unlock(&data->lock);
		return -EBUSY;
	}
	mutex_unlock(&data->lock);
	return 0;
}

static int __init mm_ak8975_init(void)
{
	pr_info("%s()\n", __func__);
	return i2c_add_driver(&mm_ak8975_driver);
}

static void __exit mm_ak8975_exit(void)
{
	pr_info("%s()\n", __func__);
	i2c_del_driver(&mm_ak8975_driver);
}

module_init(mm_ak8975_init);
module_exit(mm_ak8975_exit);
