/*
* BU9889GUL-W E2PROM driver.
*
* Copyright (C) 2011 NVIDIA Inc.
*
* This file is licensed under the terms of the GNU General Public License
* version 2. This program is licensed "as is" without any warranty of any
* kind, whether express or implied.
*/

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <media/bu9889gulw.h>
#include <mach/gpio-names.h>
#include <linux/gpio.h>
#include <lge/bssq_cam_pmic.h>

DEFINE_MUTEX(e2prom_lock);

#define DEBUG_EEPROM_DATA   (0)
#define EEPROM_WP_ENABLE   (0)

#define PAGE_SIZE (256)
#if EEPROM_WP_ENABLE
#define EEPROM_WP			TEGRA_GPIO_PD2
#endif
#define BU9889_MAX_RETRIES 3
#define AR0832_RESET		TEGRA_GPIO_PT3		//Before Rev.D  TEGRA_GPIO_PD2 
#define AR0832_PWRDN		TEGRA_GPIO_PD5

static struct bu9889gulw_info *info = NULL;

static void ar0832_cam_pin_reset(void)
{
#if defined(CONFIG_LU6500) ||defined(CONFIG_KS1001)
	 if (REV_E > get_lge_pcb_revision()){
	    gpio_set_value(TEGRA_GPIO_PD2,1);
	    gpio_set_value(TEGRA_GPIO_PD2,0);
	}else
#endif
	{
	    gpio_set_value(AR0832_RESET,1);
	    mdelay(1);
		gpio_set_value(AR0832_RESET,0);
	}
	mdelay(5);
	//gpio_set_value(Imx072_PWRDN,1);
	//mdelay(5);
}

/*
This EEPROM has 1K capacity.
The range of reg is from 0 ~ 1023(10bits resolution).

 reg: represents the register address from 0 ~ 1023
 length: 
*/
static int bu9889gulw_read(struct i2c_client *client, u16 reg, u16 length, u8 *data)
{
	u16 reg_addr = reg;
	u16 reg_cnt;
	u8 page_num;
	u8 remainder_reg;
	
	struct i2c_msg msg[2] = {
		{ .flags = 0, .len = 1 },
		{ .flags = I2C_M_RD, .len = 1 },
	};
	if (!client->adapter)
		return -ENODEV;

	for(reg_cnt = 0; reg_cnt < length; reg_cnt ++)
	{
		page_num = ((reg_addr+reg_cnt) / PAGE_SIZE) & 0x03;
		remainder_reg = ((reg_addr+reg_cnt) % PAGE_SIZE) & 0xff;

		msg[0].addr = (client->addr | page_num);
		msg[0].buf = 	&remainder_reg;
		
		msg[1].addr = (client->addr | page_num);
		msg[1].buf = &data[reg_cnt];
		if(i2c_transfer(client->adapter, msg, 2) != 2)
		{
			pr_err("bu9889gulw(R): i2c transfer failed\n");
			return -EIO;
		}
#if DEBUG_EEPROM_DATA		
		printk("%s page_num:%x, remainder:%x, addr[0x%x]=>[0x%x]\n", \
			__func__, page_num, remainder_reg, msg[0].addr, data[reg_cnt]);		
#endif // DEBUG_EEPROM_DATA
	}

	return 0;
}

static int bu9889gulw_write(struct i2c_client *client, u16 reg, u16 length, u8 *data)
{
	u16 reg_addr = reg;
	u16 reg_cnt;
	u8 page_num = (reg_addr / PAGE_SIZE) & 0x03;
	u8 msg_data[2];
	int retry = 0;
	
	struct i2c_msg msg[1] = {
		{ .flags = 0, .buf = msg_data, .len = 2 },
	};
	if (!client->adapter)
		return -ENODEV;

	for(reg_cnt = 0; reg_cnt < length; reg_cnt ++)
	{
		page_num = ((reg_addr+reg_cnt) / PAGE_SIZE) & 0x03;

		msg[0].addr = (client->addr | page_num);
		msg_data[0] = ((reg_addr+reg_cnt) % PAGE_SIZE) & 0xff;
		msg_data[1] = data[reg_cnt];

		do {
			if(i2c_transfer(client->adapter, msg, 1) != 1)
			{
				pr_err("bu9889gulw(W): i2c transfer failed\n");
				return -EIO;
			}
			msleep(3);
			retry++;
			pr_err("bu9889: i2c transfer failed, retrying %x %x\n",
			       msg[0].addr, msg_data[1]);
			//msleep(3);
		} while (retry <= BU9889_MAX_RETRIES);			
#if 0
		u8 read_data;
		if(bu9889gulw_read(client, reg_addr+reg_cnt, 1, &read_data) == 0)
		{
			if(read_data != data[reg_cnt])
			{
				pr_err("Error: (%d)data doesn't match!!\n", reg_addr+reg_cnt);			
			}
		}
#endif
	}

	return 0;
}
static long bu9889gulw_ioctl(struct file *file,
			unsigned int cmd, unsigned long arg)
{
	struct bu9889gulw_info *info = (struct bu9889gulw_info *)file->private_data;

	switch (cmd) {
	case BU9889GULW_IOCTL_GET_E2PROM_DATA:
	{
		pr_err("%s: BU9889GULW_IOCTL_GET_E2PROM_DATA: i2c client(name:%s, addr:0x%08x, flag:0x%08x)\n", __func__, info->i2c_client->name, info->i2c_client->addr, info->i2c_client->flags);
		if (copy_from_user(&info->reg_info,
			(const void __user *)arg,
			sizeof(struct bu9889gulw_register_info)))
		{
			pr_err("%s %d\n", __func__, __LINE__);
			return -EFAULT;
		}
		
		if(info->reg_info.reg_addr < 0 || info->reg_info.reg_addr > 1023)
		{
			pr_err("[Error] Requested reg_addr is not valid(r1) %s %d %x\n", __func__, __LINE__,info->reg_info.reg_addr);
			return -EFAULT;
		}
		if((info->reg_info.reg_addr +  info->reg_info.length) < 1 || (info->reg_info.reg_addr +  info->reg_info.length - 1) > 1023)
		{
			pr_err("[Error] Requested reg_addr is not valid(r2) %s %d %x\n", __func__, __LINE__,info->reg_info.reg_addr);
			return -EFAULT;
		}
		
		printk("[0] %s: Reading data from user level successful\n", __func__);

		if(bu9889gulw_read(info->i2c_client,
			info->reg_info.reg_addr,
			info->reg_info.length,
			info->reg_info.e2prom_data) == 0)
		{
			printk("[1] %s: Reading success: 0x%0x,\
				transfer data to user level\n", __func__, info->reg_info.e2prom_data[0]);
			if (copy_to_user((void __user *) arg,
				 			&info->reg_info,
							sizeof(struct bu9889gulw_register_info)))
			{
				pr_err("%s: 0x%x\n", __func__, __LINE__);
				return -EFAULT;
			}
			printk("[2] %s: Transfer success!!\n", __func__);			
		}
		else
		{
			pr_err("%s BU9889GULW_IOCTL_PUT_E2PROM_DATA Failure\n", __func__);
			return -EFAULT;
		}
		break;
	}
	case BU9889GULW_IOCTL_PUT_E2PROM_DATA:
	{
		pr_err("%s: BU9889GULW_IOCTL_PUT_E2PROM_DATA\n", __func__);
		if (copy_from_user(&info->reg_info,
			(const void __user *)arg,
			sizeof(struct bu9889gulw_register_info)))
		{
			pr_err("%s %d\n", __func__, __LINE__);
			return -EFAULT;
		}
		if(info->reg_info.reg_addr < 0 || info->reg_info.reg_addr > 1023)
		{
			pr_err("[Error] Requested reg_addr is not valid(w1) %s %d %x\n", __func__, __LINE__,info->reg_info.reg_addr);
			return -EFAULT;
		}
		if((info->reg_info.reg_addr +  info->reg_info.length) < 1 || (info->reg_info.reg_addr +  info->reg_info.length - 1) > 1023)
		{
			pr_err("[Error] Requested reg_addr is not valid(w2) %s %d %x\n", __func__, __LINE__,info->reg_info.reg_addr);
			return -EFAULT;
		}
		
		printk("[0] %s: (W)Writing data from user level successful\n", __func__);
		if(bu9889gulw_write(info->i2c_client,
			info->reg_info.reg_addr,
			info->reg_info.length,
			info->reg_info.e2prom_data) == 0)
		{
			printk("[1] %s: (W)Writing is successful\n", __func__);
		}
		else
		{
			return -EFAULT;
		}
		break;
	}	
	default:
		printk("%s: 0x%x DEFAULT IOCTL\n", __func__, __LINE__);		
		return -EINVAL;
	}
	return 0;
}

static int read_header_marker(void)
{
	int ret;
	u8 rdata = 0;

	ret = star_cam_Main_power_on();
	if(ret < 0)
	{
		pr_info("%s: ldo or sensor power-on for eeprom failure \n", __func__);
		return -EINVAL;
	}
	ar0832_cam_pin_reset();
	
	ret = bu9889gulw_read(info->i2c_client, 0x00d4, sizeof(u8), &rdata);
	if(ret < 0)
	{
		// error
		return 0; // '0 means false
	}

	if(rdata == 0xca)
	{
		ret = 1;
	}
	else
	{
		ret = 0;
	}

	star_cam_power_off();	

	return ret;
}

ssize_t show_header(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = read_header_marker();
	
    if(ret){
		printk("%s: SENSOR EEPROM OK\n", __func__);
        return snprintf(buf, PAGE_SIZE, "1\n");
    }else{
		printk("%s: SENSOR EEPROM NOK\n",__func__);
        return snprintf(buf, PAGE_SIZE, "0\n");
	}
}

static DEVICE_ATTR(check_data, 0644, show_header, NULL);

static int bu9889gulw_open(struct inode *inode, struct file *file)
{
	pr_info("bu9889gulw: open!\n");
#if EEPROM_WP_ENABLE	
	gpio_set_value(EEPROM_WP,0); 	
#endif
//	gpio_set_value(BU9889GULW_ENABLE,1);
	file->private_data = (struct file *)info;

	int ret = 0;

	pr_info("%s: called\n", __func__);
	ret = star_cam_Main_power_on();
	if(ret < 0)
	{
		pr_info("%s: ldo or sensor power-on for eeprom failure \n", __func__);
		return -EINVAL;
	}
	ar0832_cam_pin_reset();

	return 0;
}

int bu9889gulw_release(struct inode *inode, struct file *file)
{
	pr_info("bu9889gulw: release!\n");
	star_cam_power_off();	
//        gpio_set_value(BU9889GULW_ENABLE,0);  
#if EEPROM_WP_ENABLE
	gpio_set_value(EEPROM_WP,1);
#endif
	file->private_data = NULL;

	return 0;
}


static const struct file_operations bu9889gulw_fileops = {
	.owner = THIS_MODULE,
	.open = bu9889gulw_open,
	.unlocked_ioctl = bu9889gulw_ioctl,
	.release = bu9889gulw_release,
};

static struct miscdevice bu9889gulw_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "bu9889gulw",
	.fops = &bu9889gulw_fileops,
};

static int bu9889gulw_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int err;

	pr_info("bu9889gulw: probing sensor.\n");

	info = kzalloc(sizeof(struct bu9889gulw_info), GFP_KERNEL);
	if (!info) {
		pr_err("bu9889gulw: Unable to allocate memory!\n");
		return -ENOMEM;
	}

	err = misc_register(&bu9889gulw_device);
	if (err) {
		pr_err("bu9889gulw: Unable to register misc device!\n");
		kfree(info);
		return err;
	}

//	gpio_request(BU9889GULW_ENABLE, "vcm_bu9889gulw_enable");
//	tegra_gpio_enable(BU9889GULW_ENABLE);
//	gpio_direction_output(BU9889GULW_ENABLE,1);
//	mdelay(5);
//	gpio_set_value(BU9889GULW_ENABLE,0);
#if EEPROM_WP_ENABLE
	gpio_request(EEPROM_WP, "eeprom_wp");
	tegra_gpio_enable(EEPROM_WP);
	gpio_direction_output(EEPROM_WP,1);
	mdelay(5);
	gpio_set_value(EEPROM_WP,0); 
#endif	
	info->i2c_client = client;
	i2c_set_clientdata(client, info);

	err = device_create_file(&client->dev, &dev_attr_check_data);                                                      
	if(err){                                                                                                      
		printk("[sensor eeprom] device create file check_data fail!\n");   
		misc_deregister(&bu9889gulw_device);
		if(info)
			kfree(info);
		return err;
	}                                                                                                             

	return 0;
}

static int bu9889gulw_remove(struct i2c_client *client)
{
	struct bu9889gulw_info *info;
	info = i2c_get_clientdata(client);

	device_remove_file(&client->dev, &dev_attr_check_data);
	
	misc_deregister(&bu9889gulw_device);

	if(info)
		kfree(info);
	return 0;
}

static const struct i2c_device_id bu9889gulw_id[] = {
	{ "bu9889gulw", 0 },
	{ },
};

MODULE_DEVICE_TABLE(i2c, bu9889gulw_id);

static struct i2c_driver bu9889gulw_i2c_driver = {
	.driver = {
		.name = "bu9889gulw",
		.owner = THIS_MODULE,
	},
	.probe = bu9889gulw_probe,
	.remove = bu9889gulw_remove,
	.id_table = bu9889gulw_id,
};

static int __init bu9889gulw_init(void)
{
	pr_info("bu9889gulw sensor driver loading\n");
	i2c_add_driver(&bu9889gulw_i2c_driver);

	return 0;
}

static void __exit bu9889gulw_exit(void)
{
	i2c_del_driver(&bu9889gulw_i2c_driver);
}

module_init(bu9889gulw_init);
module_exit(bu9889gulw_exit);
