/*
 * ov9726.c - ov9726 sensor driver
 *
 * Copyright (c) 2011, NVIDIA, All Rights Reserved.
 *
 * Contributors:
 *	  Charlie Huang <chahuang@nvidia.com>
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <mach/iomap.h>
#include <asm/atomic.h>
#include <mach/gpio.h>
#include <linux/regulator/consumer.h>

#include <media/ov9726.h>

struct ov9726_power_rail {
	struct regulator *sen_1v8_reg;
	struct regulator *sen_2v8_reg;
};

struct ov9726_devinfo {
	struct miscdevice		miscdev_info;
	struct i2c_client		*i2c_client;
	struct ov9726_platform_data	*pdata;
	struct ov9726_power_rail	power_rail;
	atomic_t			in_use;
	__u32				mode;
};

static inline void
msleep_range(unsigned int delay_base)
{
	usleep_range(delay_base*1000, delay_base*1000 + 500);
}

static inline int
ov9726_power_init(struct ov9726_devinfo *dev)
{
	struct i2c_client *i2c_client = dev->i2c_client;
	int err = 0;

	dev->power_rail.sen_1v8_reg = regulator_get(&i2c_client->dev, "dovdd");
	if (IS_ERR_OR_NULL(dev->power_rail.sen_1v8_reg)) {
		dev_err(&i2c_client->dev, "%s: failed to get vdd\n",
			__func__);
		err = PTR_ERR(dev->power_rail.sen_1v8_reg);
		goto ov9726_power_init_end;
	}

	dev->power_rail.sen_2v8_reg = regulator_get(&i2c_client->dev, "avdd");
	if (IS_ERR_OR_NULL(dev->power_rail.sen_2v8_reg)) {
		dev_err(&i2c_client->dev, "%s: failed to get vaa\n",
			__func__);
		err = PTR_ERR(dev->power_rail.sen_2v8_reg);

		regulator_put(dev->power_rail.sen_1v8_reg);
		dev->power_rail.sen_1v8_reg = NULL;
	}

ov9726_power_init_end:
	return err;
}

static inline void
ov9726_power_release(struct ov9726_devinfo *dev)
{
	regulator_put(dev->power_rail.sen_1v8_reg);
	regulator_put(dev->power_rail.sen_2v8_reg);
}

static int
ov9726_power(struct ov9726_devinfo *dev, bool pwr_on)
{
	struct i2c_client *i2c_client = dev->i2c_client;
	int rst_active_state = dev->pdata->rst_low_active ? 0 : 1;
	int pwdn_active_state = dev->pdata->pwdn_low_active ? 0 : 1;
	int ret = 0;

	dev_info(&i2c_client->dev, "%s %s\n", __func__, pwr_on ? "on" : "off");

	if (pwr_on) {
		/* pull low the RST pin of ov9726 first */
		gpio_set_value(dev->pdata->gpio_rst, rst_active_state);
		msleep_range(1);
		/* Plug 1.8V and 2.8V power to sensor */
		ret = regulator_enable(dev->power_rail.sen_1v8_reg);
		if (ret) {
			dev_err(&i2c_client->dev, "%s: failed to enable vdd\n",
				__func__);
			goto fail_regulator_1v8_reg;
		}

		msleep_range(20);

		ret = regulator_enable(dev->power_rail.sen_2v8_reg);
		if (ret) {
			dev_err(&i2c_client->dev, "%s: failed to enable vaa\n",
				__func__);
			goto fail_regulator_2v8_reg;
		}
		msleep_range(1);
		/* turn on ov9726 */
		gpio_set_value(dev->pdata->gpio_pwdn, !pwdn_active_state);

		msleep_range(5);
		/* release RST pin */
		gpio_set_value(dev->pdata->gpio_rst, !rst_active_state);
		msleep_range(20);

		/* Board specific power-on sequence */
		dev->pdata->power_on();
	} else {
		/* pull low the RST pin of ov9726 */
		gpio_set_value(dev->pdata->gpio_rst, rst_active_state);
		msleep_range(1);
		/* turn off ov9726 */
		gpio_set_value(dev->pdata->gpio_pwdn, pwdn_active_state);
		msleep_range(1);

		/* Unplug 1.8V and 2.8V power from sensor */
		regulator_disable(dev->power_rail.sen_2v8_reg);
		regulator_disable(dev->power_rail.sen_1v8_reg);

		/* Board specific power-down sequence */
		dev->pdata->power_off();
	}

	return 0;

fail_regulator_2v8_reg:
	regulator_put(dev->power_rail.sen_2v8_reg);
	dev->power_rail.sen_2v8_reg = NULL;
	regulator_disable(dev->power_rail.sen_1v8_reg);
fail_regulator_1v8_reg:
	regulator_put(dev->power_rail.sen_1v8_reg);
	dev->power_rail.sen_1v8_reg = NULL;
	return ret;
}

/* 2 regs to program frame length */
static inline void
ov9726_get_frame_length_regs(struct ov9726_reg *regs, u32 frame_length)
{
	regs->addr = OV9726_REG_FRAME_LENGTH_HI;
	regs->val = (frame_length >> 8) & 0xff;
	regs++;
	regs->addr = OV9726_REG_FRAME_LENGTH_LO;
	regs->val = frame_length & 0xff;
}

/* 3 regs to program coarse time */
static inline void
ov9726_get_coarse_time_regs(struct ov9726_reg *regs, u32 coarse_time)
{
	regs->addr = OV9726_REG_COARSE_TIME_HI;
	regs->val = (coarse_time >> 8) & 0xff;
	regs++;
	regs->addr = OV9726_REG_COARSE_TIME_LO;
	regs->val = coarse_time & 0xff;
}

/* 1 reg to program gain */
static inline void
ov9726_get_gain_reg(struct ov9726_reg *regs, u16 gain)
{
	regs->addr = OV9726_REG_GAIN_HI;
	regs->val = (gain >> 8) & 0xff;
	regs++;
	regs->addr = OV9726_REG_GAIN_LO;
	regs->val = gain & 0xff;
}

static int
ov9726_read_reg8(struct i2c_client *client, u16 addr, u8 *val)
{
	int	err;
	struct i2c_msg  msg[2];
	unsigned char   data[3];

	if (!client->adapter)
		return -ENODEV;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf = data;

	/* high byte goes out first */
	data[0] = (u8)(addr >> 8);
	data[1] = (u8)(addr & 0xff);

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 1;
	msg[1].buf = data + 2;

	err = i2c_transfer(client->adapter, msg, 2);

	if (err != 2)
		err = -EINVAL;
	else {
		*val = data[2];
		err = 0;
	}

	return err;
}

static int
ov9726_write_reg8(struct i2c_client *client, u16 addr, u8 val)
{
	int		err;
	struct i2c_msg	msg;
	unsigned char	data[3];
	int		retry = 0;

	if (!client->adapter)
		return -ENODEV;

	data[0] = (u8)(addr >> 8);
	data[1] = (u8)(addr & 0xff);
	data[2] = (u8)(val & 0xff);

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = 3;
	msg.buf = data;

	do {
		err = i2c_transfer(client->adapter, &msg, 1);
		if (err == 1)
			break;

		retry++;
		dev_err(&client->dev,
			"ov9726: i2c transfer failed, retrying %x %x\n",
			addr, val);
		msleep_range(3);
	} while (retry <= OV9726_MAX_RETRIES);

	return (err != 1);
}

static int
ov9726_write_reg16(struct i2c_client *client, u16 addr, u16 val)
{
	int		count;
	struct i2c_msg  msg;
	unsigned char   data[4];
	int		retry = 0;

	if (!client->adapter)
		return -ENODEV;

	data[0] = (u8)(addr >> 8);
	data[1] = (u8)(addr & 0xff);
	data[2] = (u8)(val >> 8);
	data[3] = (u8)(val & 0xff);

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = 4;
	msg.buf = data;

	do {
		count = i2c_transfer(client->adapter, &msg, 1);
		if (count == 1)
			return 0;

		retry++;
		dev_err(&client->dev,
			"ov9726: i2c transfer failed, retrying %x %x %x\n",
			addr, val, count);
		msleep_range(3);
	} while (retry <= OV9726_MAX_RETRIES);

	return -EIO;
}

static int
ov9726_write_table(
	struct i2c_client *client,
	struct ov9726_reg table[],
	struct ov9726_reg override_list[],
	int num_override_regs)
{
	const struct ov9726_reg	*next;
	int			err = 0;
	int			i;
	u16			val;

	dev_info(&client->dev, "ov9726_write_table\n");

	for (next = table; next->addr != OV9726_TABLE_END; next++) {

		if (next->addr == OV9726_TABLE_WAIT_MS) {
			msleep_range(next->val);
			continue;
		}

		val = next->val;

		/* When an override list is passed in, replace the reg */
		/* value to write if the reg is in the list */
		if (override_list) {
			for (i = 0; i < num_override_regs; i++) {
				if (next->addr == override_list[i].addr) {
					val = override_list[i].val;
					break;
				}
			}
		}

		err = ov9726_write_reg8(client, next->addr, val);
		if (err)
			break;
	}

	return err;
}

static int
ov9726_set_frame_length(struct i2c_client *i2c_client, u32 frame_length)
{
	int ret;

	dev_info(&i2c_client->dev, "[%s] (0x%08x)\n", __func__,  frame_length);
	/* hold register value */
	ret = ov9726_write_reg8(i2c_client, 0x104, 0x01);
	if (ret)
		return ret;

	ret = ov9726_write_reg16(i2c_client,
				  OV9726_REG_FRAME_LENGTH_HI,
				  frame_length);

	/* release hold, update register value */
	ret |= ov9726_write_reg8(i2c_client, 0x104, 0x00);

	return ret;
}

static int
ov9726_set_coarse_time(struct i2c_client *i2c_client, u32 coarse_time)
{
	int ret;

	dev_info(&i2c_client->dev, "[%s] (0x%08x)\n", __func__,  coarse_time);
	/* hold register value */
	ret = ov9726_write_reg8(i2c_client, 0x104, 0x01);
	if (ret)
		return ret;

	ret = ov9726_write_reg16(i2c_client,
			  OV9726_REG_COARSE_TIME_HI,
			  coarse_time);

	/* release hold, update register value */
	ret |= ov9726_write_reg8(i2c_client, 0x104, 0x00);

	return ret;
}

static int ov9726_set_gain(struct i2c_client *i2c_client, u16 gain)
{
	int ret;

	/* hold register value */
	ret = ov9726_write_reg8(i2c_client, 0x104, 0x01);
	if (ret)
		return ret;

	ret = ov9726_write_reg16(i2c_client, OV9726_REG_GAIN_HI, gain);

	/* release hold, update register value */
	ret |= ov9726_write_reg8(i2c_client, 0x104, 0x00);

	return ret;
}

static int ov9726_get_status(struct i2c_client *i2c_client, u8 *status)
{
	int err;

	err = ov9726_read_reg8(i2c_client, 0x003, status);
	*status = 0;
	return err;
}

static int
ov9726_set_mode(
	struct ov9726_devinfo *dev,
	struct ov9726_mode *mode,
	struct ov9726_reg *reg_list)
{
	struct i2c_client	*i2c_client = dev->i2c_client;
	int			err = 0;

	dev_info(&i2c_client->dev, "%s.\n", __func__);

	if (!reg_list) {
		dev_err(&i2c_client->dev, "%s: empty reg list\n", __func__);
		return -EINVAL;
	}

	if (dev->mode != mode->mode_id) {
		dev_info(&i2c_client->dev,
			"%s: xres %u yres %u framelen %u coarse %u gain %u\n",
			__func__, mode->xres, mode->yres, mode->frame_length,
			mode->coarse_time, mode->gain);

		err = ov9726_write_table(i2c_client, reg_list, NULL, 0);
		if (err)
			goto ov9726_set_mode_exit;

		err = ov9726_set_frame_length(i2c_client, mode->frame_length);
		if (err)
			goto ov9726_set_mode_exit;

		err = ov9726_set_coarse_time(i2c_client, mode->coarse_time);
		if (err)
			goto ov9726_set_mode_exit;

		dev->mode = mode->mode_id;
	}

	err = ov9726_set_gain(i2c_client, mode->gain);

ov9726_set_mode_exit:
	return err;
}

static long
ov9726_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct ov9726_devinfo	*dev = file->private_data;
	struct i2c_client	*i2c_client = dev->i2c_client;
	int			 err = 0;

	switch (cmd) {
	case OV9726_IOCTL_SET_MODE:
	{
		struct ov9726_cust_mode	cust_mode;
		struct ov9726_reg	*reg_list = NULL;

		if (copy_from_user(&cust_mode,
				   (const void __user *)arg,
				   sizeof(struct ov9726_cust_mode))) {
			err = -EFAULT;
			break;
		}

		reg_list = kzalloc(
			sizeof(struct ov9726_reg) * cust_mode.reg_num,
			GFP_KERNEL);

		if (!reg_list) {
			dev_err(&i2c_client->dev,
				"ov9726: Unable to allocate memory!\n");
			err = -ENOMEM;
			break;
		}

		if (copy_from_user(reg_list,
			   (const void __user *)cust_mode.reg_seq,
			   sizeof(struct ov9726_reg) * cust_mode.reg_num))
			err = -EFAULT;

		if (!err)
			err = ov9726_set_mode(dev, &(cust_mode.mode), reg_list);

		kfree(reg_list);
		break;
	}

	case OV9726_IOCTL_SET_FRAME_LENGTH:
		err = ov9726_set_frame_length(i2c_client, (u32)arg);
		break;

	case OV9726_IOCTL_SET_COARSE_TIME:
		err = ov9726_set_coarse_time(i2c_client, (u32)arg);
		break;

	case OV9726_IOCTL_SET_GAIN:
		err = ov9726_set_gain(i2c_client, (u16)arg);
		break;

	case OV9726_IOCTL_GET_STATUS:
	{
		u8 status;

		err = ov9726_get_status(i2c_client, &status);
		if (!err) {
			if (copy_to_user((void __user *)arg,
					&status, sizeof(status)))
				err = -EFAULT;
		}
		break;
	}

	default:
		err = -EINVAL;
		break;
	}

	return err;
}

static int ov9726_open(struct inode *inode, struct file *file)
{
	struct miscdevice	*miscdev = file->private_data;
	struct ov9726_devinfo	*dev;

	dev = container_of(miscdev, struct ov9726_devinfo, miscdev_info);
	/* check if device is in use */
	if (atomic_xchg(&dev->in_use, 1))
		return -EBUSY;
	dev->mode = (__u32)-1;
	file->private_data = dev;

	ov9726_power(dev, true);

	return 0;
}

int ov9726_release(struct inode *inode, struct file *file)
{
	struct ov9726_devinfo *dev;

	dev = file->private_data;
	file->private_data = NULL;

	ov9726_power(dev, false);

	/* warn if device already released */
	WARN_ON(!atomic_xchg(&dev->in_use, 0));
	return 0;
}

static const struct file_operations ov9726_fileops = {
	.owner		= THIS_MODULE,
	.open		= ov9726_open,
	.unlocked_ioctl	= ov9726_ioctl,
	.release	= ov9726_release,
};

static struct miscdevice ov9726_device = {
	.name		= "ov9726",
	.minor		= MISC_DYNAMIC_MINOR,
	.fops		= &ov9726_fileops,
};

static int
ov9726_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct ov9726_devinfo	*dev;
	int			err = 0;

	dev_info(&client->dev, "ov9726: probing sensor.\n");

	dev = kzalloc(sizeof(struct ov9726_devinfo), GFP_KERNEL);
	if (!dev) {
		dev_err(&client->dev, "ov9726: Unable to allocate memory!\n");
		err = -ENOMEM;
		goto probe_end;
	}

	memcpy(&(dev->miscdev_info),
		&ov9726_device,
		sizeof(struct miscdevice));

	err = misc_register(&(dev->miscdev_info));
	if (err) {
		dev_err(&client->dev, "ov9726: Unable to register misc device!\n");
		goto probe_end;
	}

	dev->pdata = client->dev.platform_data;
	dev->i2c_client = client;
	atomic_set(&dev->in_use, 0);
	i2c_set_clientdata(client, dev);

	err = ov9726_power_init(dev);

probe_end:
	if (err) {
		kfree(dev);
		dev_err(&client->dev, "failed.\n");
	}

	return err;
}

static int ov9726_remove(struct i2c_client *client)
{
	struct ov9726_devinfo	*dev;

	dev = i2c_get_clientdata(client);
	i2c_set_clientdata(client, NULL);
	misc_deregister(&ov9726_device);
	ov9726_power_release(dev);
	kfree(dev);

	return 0;
}

static const struct i2c_device_id ov9726_id[] = {
	{"ov9726", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, ov9726_id);

static struct i2c_driver ov9726_i2c_driver = {
	.driver	= {
		.name   = "ov9726",
		.owner  = THIS_MODULE,
		},
	.probe  = ov9726_probe,
	.remove = ov9726_remove,
	.id_table   = ov9726_id,
};

static int __init ov9726_init(void)
{
	pr_info("ov9726 sensor driver loading\n");
	return i2c_add_driver(&ov9726_i2c_driver);
}

static void __exit ov9726_exit(void)
{
	i2c_del_driver(&ov9726_i2c_driver);
}

module_init(ov9726_init);
module_exit(ov9726_exit);
