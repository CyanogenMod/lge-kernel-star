/*
* dw9712.c focuser driver.
*
* Copyright (C) 2010 LGE Inc.
*
* This file is licensed under the terms of the GNU General Public License
* version 2. This program is licensed "as is" without any warranty of any
* kind, whether express or implied.
*/

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <media/dw9712_lu6500.h>
#include <mach/gpio-names.h>
#include <linux/gpio.h>

#define POS_LOW 50
#define POS_HIGH 1000
#define SETTLETIME_MS 100
#define FOCAL_LENGTH (3.5f)
#define FNUMBER (2.6f)
#define FPOS_COUNT 1024
DEFINE_MUTEX(star_focuser_lock);
#define DW9712_MAX_RETRIES (3)
#define DW9712_ENABLE        TEGRA_GPIO_PT4

struct dw9712_info {
	struct i2c_client *i2c_client;
	struct i2c_client *i2c_client_right;
	struct regulator *regulator;
	struct dw9712_config config;
};

enum StereoCameraMode{
	Main = 0,
	/// Sets the stereo camera to stereo mode.
	Stereo = 1,
	/// Only the sensor on the left is on.
	LeftOnly,
	/// Only the sensor on the right is on.
	RightOnly,
	/// Ignore -- Forces compilers to make 32-bit enums.
	StereoCameraMode_Force32 = 0x7FFFFFFF
};
static enum StereoCameraMode camera_mode;

static int dw9712_write(struct i2c_client *client, u16 value)
{
	int count;
	struct i2c_msg msg[1];
	unsigned char data[2];
	int retry = 0;

	if (!client->adapter)
		return -ENODEV;

	data[0] = (u8) ((value >> 4) & 0x3F);
	data[1] = (u8) ((value & 0xF) << 4);
	// Slew rate control (8 steps, 50us)
	data[1] = (data[1] &0xF0) |0x05;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = ARRAY_SIZE(data);
	msg[0].buf = data;
	//pr_info("%s: focuser set position = %d, 0x%x(0x%x)\n",
		//__func__, value, *(u16*)data, client->addr);
	do {
		count = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
		if (count == ARRAY_SIZE(msg))
			return 0;
		retry++;
		pr_err("dw9712: i2c transfer failed, retrying %x\n",
				value);
		msleep(3);
	}while (retry <= DW9712_MAX_RETRIES);
	return -EIO;
}
static int dw9712_write_helper(struct dw9712_info *info, u16 value)
{
	int ret;
	switch(camera_mode){
		case Main:
		case LeftOnly:
			ret = dw9712_write(info->i2c_client,  value);
			break;
		case Stereo:
			ret = dw9712_write(info->i2c_client,  value);
			ret = dw9712_write(info->i2c_client_right,  value);
			break;
		case RightOnly:
			ret = dw9712_write(info->i2c_client_right,  value);
			break;
		default :
			return -1;
	}
	return ret;
}
static int dw9712_set_position(struct dw9712_info *info, u32 position)
{
	if (position < info->config.pos_low ||
		position > info->config.pos_high)
	{
		pr_err("dw9712: set position error: pos_low(%d), pos_hig(%d),\
			position(%d)\n",
			info->config.pos_low, info->config.pos_high, position);
		return -EINVAL;
	}

	return dw9712_write(info->i2c_client, position);
}

static long dw9712_ioctl(struct file *file,
			unsigned int cmd, unsigned long arg)
{
	struct dw9712_info *info = file->private_data;
	int ret;
	switch (cmd) {
	case DW9712_IOCTL_GET_CONFIG:
	{
		pr_err("%s: DW9712_IOCTL_GET_CONFIG\n", __func__);
		if (copy_to_user((void __user *) arg,
				 &info->config,
				 sizeof(info->config))) {
			pr_err("%s: 0x%x\n", __func__, __LINE__);
			return -EFAULT;
		}

		break;
	}
	case DW9712_IOCTL_SET_POSITION:
		//pr_err("%s: DW9712_IOCTL_SET_POSITION(%d)\n", (u32)arg);
		mutex_lock(&star_focuser_lock);
		ret = dw9712_set_position(info, (u32) arg);
		mutex_unlock(&star_focuser_lock);
		return ret;
	case DW9712_IOCTL_SET_MODE:
		pr_err("%s: DW9712_IOCTL_SET_MODE: %d\n", 
				__func__, camera_mode);
		camera_mode =(enum StereoCameraMode)arg;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static struct dw9712_info *info = NULL;

static int dw9712_open(struct inode *inode, struct file *file)
{
	pr_info("dw9712: open!\n");
	gpio_set_value(DW9712_ENABLE,1);
	file->private_data = info;
	if (info->regulator)
		regulator_enable(info->regulator);
	return 0;
}

int dw9712_release(struct inode *inode, struct file *file)
{
	pr_info("dw9712: release!\n");
        gpio_set_value(DW9712_ENABLE,0);    
	if (info->regulator)
		regulator_disable(info->regulator);
	file->private_data = NULL;
	return 0;
}


static const struct file_operations dw9712_fileops = {
	.owner = THIS_MODULE,
	.open = dw9712_open,
	.unlocked_ioctl = dw9712_ioctl,
	.release = dw9712_release,
};

static struct miscdevice dw9712_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "dw9712",
	.fops = &dw9712_fileops,
};

static int dw9712_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int err;

	pr_info("dw9712: probing sensor.\n");

	info = kzalloc(sizeof(struct dw9712_info), GFP_KERNEL);
	if (!info) {
		pr_err("dw9712: Unable to allocate memory!\n");
		return -ENOMEM;
	}

	err = misc_register(&dw9712_device);
	if (err) {
		pr_err("dw9712: Unable to register misc device!\n");
		kfree(info);
		return err;
	}

    gpio_request(DW9712_ENABLE, "vcm_dw9712_enable");
    tegra_gpio_enable(DW9712_ENABLE);
    gpio_direction_output(DW9712_ENABLE,1);
    mdelay(5);
    gpio_set_value(DW9712_ENABLE,0);

    
//WBT#196353 : don'use the regulator. the power turn on when camera turn on.
/*
	info->regulator = regulator_get(&client->dev, "p_cam_avdd");
	if (IS_ERR_OR_NULL(info->regulator)) {
		dev_err(&client->dev, "unable to get regulator %s\n",
			dev_name(&client->dev));
		info->regulator = NULL;
	} else {
		regulator_enable(info->regulator);
	}
*/
	info->regulator = 0;
	info->i2c_client = client;
	info->config.settle_time = SETTLETIME_MS;
	info->config.focal_length = FOCAL_LENGTH;
	info->config.fnumber = FNUMBER;
	info->config.pos_low = POS_LOW;
	info->config.pos_high = POS_HIGH;
	i2c_set_clientdata(client, info);

	return 0;
}

static int dw9712_remove(struct i2c_client *client)
{
	struct dw9712_info *info;
	info = i2c_get_clientdata(client);
	misc_deregister(&dw9712_device);
	kfree(info);
	return 0;
}

static const struct i2c_device_id dw9712_id[] = {
	{ "dw9712", 0 },
	{ },
};

MODULE_DEVICE_TABLE(i2c, dw9712_id);

static struct i2c_driver dw9712_i2c_driver = {
	.driver = {
		.name = "dw9712",
		.owner = THIS_MODULE,
	},
	.probe = dw9712_probe,
	.remove = dw9712_remove,
	.id_table = dw9712_id,
};

static int __init dw9712_init(void)
{
	pr_info("dw9712 sensor driver loading\n");
	i2c_add_driver(&dw9712_i2c_driver);

	return 0;
}

static void __exit dw9712_exit(void)
{
	i2c_del_driver(&dw9712_i2c_driver);
}

module_init(dw9712_init);
module_exit(dw9712_exit);
