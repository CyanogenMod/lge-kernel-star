/*
* dw9712.c - focuser driver
*
* Copyright (c) 2011, NVIDIA, All Rights Reserved.
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
#include <media/dw9712.h>
#include <linux/gpio.h>

#if defined(CONFIG_REGULATOR_CAM_SUBPMIC_LP8720)
#include <linux/regulator/lp8720.h>
extern void subpm_set_output(subpm_output_enum outnum, int onoff);
extern void subpm_output_enable(void);
#endif

#define POS_LOW (100)
#define POS_HIGH (900)
#define SETTLETIME_MS 100
#define FOCAL_LENGTH (4.49f)
#define FNUMBER (2.8f)

//DEFINE_MUTEX(dw9712_lock);
#define DW9712_MAX_RETRIES (3)

#define IMX111_VCM_PWD_GPIO     83 //TEGRA_GPIO_PK03


struct dw9712_info {
	struct i2c_client *i2c_client;
	struct dw9712_config config;
};
static struct dw9712_info *info;

static int dw9712_write(struct i2c_client *client, u16 value)
{
	int count;
	struct i2c_msg msg[1];
	unsigned char data[2];
	int retry = 0;

	if (!client->adapter)
		return -ENODEV;

	data[0] = value >> 8;
	data[1] = value & 0xff;
  
	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = ARRAY_SIZE(data);
	msg[0].buf = data;

	do {
		count = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
		if (count == ARRAY_SIZE(msg))
			return 0;
		retry++;
		pr_err("dw9712: i2c transfer failed, retrying %x\n",
				value);
		//hyojin.an usleep_range(3000, 3500);
		msleep(1); //hyojin.an
	} while (retry <= DW9712_MAX_RETRIES);
	return -EIO;  
}

static int dw9712_set_position(struct dw9712_info *info, u32 position)
{
	pr_info("%s %d\n", __func__, position);

	int err;
	u16 value;
	u8 *ptr = (u8 *)&value;

	if (position < info->config.pos_low || position > info->config.pos_high){
		pr_err(" !! RETURN !! position = %d, info->config.pos_low = %d, info->config.pos_high = %d\n",
				position, info->config.pos_low, info->config.pos_high);
		return -EINVAL;
	}

	value = 0x0;
	ptr[1] = (u8)((position>>4) & 0x3F);
	ptr[0] = (u8)(((position & 0xf) << 4) | 0xe); // Bit 0-1: set delay time. Bit 2-3: set incre
	err = dw9712_write(info->i2c_client, value);
	if (err) {
		pr_err("[CAM] DW9712: %s: set position failed\n", __func__);
	}
   
	return err;
}

static long dw9712_ioctl(struct file *file,
			unsigned int cmd, unsigned long arg)
{
	struct dw9712_info *info = file->private_data;
	int ret;

	pr_info("%s\n", __func__);

	switch (cmd) {
	case DW9712_IOCTL_GET_CONFIG:
	{
		if (copy_to_user((void __user *) arg, &info->config, sizeof(info->config))) 
		{
			pr_err("%s: 0x%x\n", __func__, __LINE__);
			return -EFAULT;
		}
		break;
	}
	case DW9712_IOCTL_SET_POSITION:
//		mutex_lock(&dw9712_lock);
		ret = dw9712_set_position(info, (u32) arg);
//		mutex_unlock(&dw9712_lock);
		return ret;
    
	default:
		return -EINVAL;
	}

	return 0;
}

static int dw9712_power_on(void)
{

#if defined(CONFIG_REGULATOR_CAM_SUBPMIC_LP8720)
	pr_info("LP8720 CamPMIC Focuser Power ON ++\n");

	subpm_set_output(LDO5,1);
	subpm_output_enable();
	//udelay(500);
	mdelay(20);

#endif

#if 0  
	gpio_direction_output(IMX111_VCM_PWD_GPIO, 1);
	gpio_set_value(IMX111_VCM_PWD_GPIO, 1);  
	mdelay(10);
#endif

	return 0;
}

static int dw9712_power_off(void)
{
#if 0
	gpio_set_value(IMX111_VCM_PWD_GPIO, 0);
	udelay(300);
#endif

#if defined(CONFIG_REGULATOR_CAM_SUBPMIC_LP8720)
	pr_info("LP8720 CamPMIC Focuser Power OFF ++\n");

	subpm_set_output(LDO5,0);
	subpm_output_enable();
	udelay(10);
#endif

	return 0;
}

static int dw9712_open(struct inode *inode, struct file *file)
{
	pr_info("%s\n", __func__);

	dw9712_power_on();
	
	file->private_data = info;
  
	return 0;
}

int dw9712_release(struct inode *inode, struct file *file)
{
	pr_info("%s\n", __func__);

	file->private_data = NULL;

	dw9712_power_off();
  
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

	pr_info("%s\n", __func__);

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

#if 0
	tegra_gpio_enable(IMX111_VCM_PWD_GPIO);
	err = gpio_request(IMX111_VCM_PWD_GPIO, "8m_cam_vcm_pwd");
	if (err < 0)
		pr_err("%s: gpio_request failed for gpio %s\n",
			__func__, "8m_cam_vcm_pwd");

	gpio_direction_output(IMX111_VCM_PWD_GPIO, 1);
	gpio_set_value(IMX111_VCM_PWD_GPIO, 0);
#endif
  
	//info->regulator = 0;
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
  
	pr_info("%s\n", __func__);
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
