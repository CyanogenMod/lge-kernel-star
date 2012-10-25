#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <mach/gpio.h>
#include <bssq_flash_led.h>

#define setbits(data, masks)		data |=  masks
#define clrbits(data, masks)		data &= ~masks

static struct i2c_client *this_client	=	0;
static struct bssq_flash_led_platform_data*	pdata	=	0;
static unsigned char	flash_brightness	=	7;//13;   //20110608 jinkwan.kim@lge.com Camsensor flash

static int bssq_flash_led_enable(int status)
{
//	printk(KERN_INFO "enter %s input value %d \n", __func__, status );

	gpio_set_value(pdata->gpio_hwen, (status != 0));

	return	0;
}

static int bssq_flash_led_write(unsigned char index, unsigned char data)
{
//	printk(KERN_INFO "[bssq_flash_led] %s, index: 0x%x, data: 0x%x \n", __func__,index, data);
	return i2c_smbus_write_byte_data(this_client,index,data);
}

static int bssq_flash_led_read(unsigned char index)
{
#if 0
	int ret = i2c_smbus_read_byte_data(this_client,index);
	printk(KERN_INFO "[bssq_flash_led] %s, index: 0x%x, data: 0x%2x \n", __func__,index, ret&0x000000ff);
	return ret;
#endif
	return i2c_smbus_read_byte_data(this_client,index);
}

static int bssq_flash_led_flash(unsigned char timeout)
{

	unsigned char value;

//	printk(KERN_INFO "enter %s\n", __func__);  	

	if (timeout > 0x1f)
		return	-1;

	// Set flash brightness
	bssq_flash_led_write(FLASH_BRIGHTNESS_REG_INDEX, (flash_brightness << 4) | flash_brightness);

	value	=	bssq_flash_led_read(FLASH_DURATION_REG_INDEX);

	clrbits(value, 0x1F);
	setbits(value, timeout);
	bssq_flash_led_write(FLASH_DURATION_REG_INDEX, value); 
	
	value	=	bssq_flash_led_read(ENABLE_REG_INDEX);
	setbits(value, 0x03);
	bssq_flash_led_write(ENABLE_REG_INDEX, value);
#if 0
	 bssq_flash_led_write(0xb0, 0x77);
	 bssq_flash_led_write(0xc0, 0x6f);
	 bssq_flash_led_write(0x80, 0xc9);
	 bssq_flash_led_write(0xa0, 0x3f);
	 bssq_flash_led_write(0x10, 0x1b);
	 bssq_flash_led_read(0xd0);
#endif 
	return 0;
}        

static void camera_flash(unsigned long timeout)
{
//	printk(KERN_INFO "enter %s\n", __func__);

	if (timeout > 1024)
		return	;

	bssq_flash_led_flash(timeout/32);
}

static int bssq_flash_led_torch(unsigned char brightness)
{
	unsigned char value;

//	printk(KERN_INFO "enter %s\n", __func__);

	if (brightness > 0x3F)
		brightness = 0x3F;

	if(brightness == 0)
	{
		// Off torch
		value = bssq_flash_led_read(ENABLE_REG_INDEX);
		clrbits(value, 0x03);
		bssq_flash_led_write(ENABLE_REG_INDEX,value);
	}
	else
	{
		bssq_flash_led_write(TORCH_BRIGHTNESS_REG_INDEX, brightness);//set torch brightness

		value = bssq_flash_led_read(ENABLE_REG_INDEX);
		clrbits(value, 0x01);
		setbits(value, 0x02);
		bssq_flash_led_write(ENABLE_REG_INDEX,value);
	}
	
#if 0
	bssq_flash_led_write(0xa0, 0x52);//set torch brightness
	bssq_flash_led_write(0x10, 0x1a);//set torch brightness
	bssq_flash_led_read(0xa0);//set torch brightness
	bssq_flash_led_read(0x10);//set torch brightness
#endif
	return	0;
}


static ssize_t flash_store(struct device* dev,
							struct device_attribute* attr, const char* buf, size_t count)
{
//	printk(KERN_INFO "enter %s\n", __func__);
	camera_flash(simple_strtoul(buf, NULL, 10));

	return count;
}
static DEVICE_ATTR(flash, 0644, NULL, flash_store);

static ssize_t flash_brightness_store(struct device* dev,
							struct device_attribute* attr, const char* buf, size_t count)
{
	unsigned long	value	=	simple_strtoul(buf, NULL, 10);

//	printk(KERN_INFO "enter %s\n", __func__);

	if ((0 <= value) && (value < 16))
		flash_brightness	=	(unsigned char)value;

	return	count;
}

static ssize_t flash_brightness_show(struct device* dev,
							struct device_attribute* attr, const char* buf, size_t count)
{
//	printk(KERN_INFO "enter %s\n", __func__);

	return	snprintf(buf, PAGE_SIZE, "%d\n", flash_brightness);
}
static DEVICE_ATTR(flash_brightness, 0644, flash_brightness_show, flash_brightness_store);

static ssize_t torch_store(struct device* dev,
							struct device_attribute* attr, const char* buf, size_t count)
{
//	printk(KERN_INFO "enter %s\n", __func__);

	bssq_flash_led_torch(simple_strtoul(buf, NULL, 10));

	return	count;
}
static DEVICE_ATTR(torch, 0644, NULL, torch_store);

static ssize_t enable_store(struct device* dev,
							 struct device_attribute* attr, const char* buf, size_t count)
{
//	printk(KERN_INFO "enter %s\n", __func__);

	bssq_flash_led_enable(simple_strtoul(buf, NULL, 10));

	return	count;
}

static ssize_t enable_show(struct device* dev,
							 struct device_attribute* attr, const char* buf, size_t count)
{
//	printk(KERN_INFO "enter %s\n", __func__);

	return	snprintf(buf, PAGE_SIZE, "%d\n", gpio_get_value(pdata->gpio_hwen));
}
static DEVICE_ATTR(enable, 0644, enable_show, enable_store);

static int bssq_flash_led_remove(struct i2c_client *client)
{
//	printk(KERN_INFO "enter %s\n", __func__);

	bssq_flash_led_enable(0);

	gpio_free(pdata->gpio_hwen);

	if (client != 0) {
		device_remove_file(&client->dev, &dev_attr_enable);
		device_remove_file(&client->dev, &dev_attr_flash);
		device_remove_file(&client->dev, &dev_attr_flash_brightness);
		device_remove_file(&client->dev, &dev_attr_torch);
	}

	client	=	0;

	return	0;
}

static int bssq_flash_led_probe(struct i2c_client *client, const struct i2c_device_id *devid)
{
	int res = 0;

	printk(KERN_INFO "enter %s\n", __func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return	-ENODEV;

	this_client	=	client;

	pdata	=	client->dev.platform_data;

	tegra_gpio_enable(pdata->gpio_hwen);
	gpio_request(pdata->gpio_hwen, "flash_en");

	res = device_create_file(&client->dev, &dev_attr_flash);
	if(res){
		printk("[bssq_flash_led:] device create file flash fail!\n");
		goto exit;
	}

	res = device_create_file(&client->dev, &dev_attr_flash_brightness);
	if(res){
		printk("[bssq_flash_led:] device create file flash_brightness fail!\n");
		goto exit;
	}

	res = device_create_file(&client->dev, &dev_attr_torch);
	if(res){
		printk("[bssq_flash_led:] device create file torch fail!\n");
		goto exit;
	}

	res = device_create_file(&client->dev, &dev_attr_enable);
	if(res){
		printk("[bssq_flash_led:] device create file enable fail!\n");
		goto exit;
	}

	return res;

exit:
	bssq_flash_led_remove(this_client);
	this_client	=	0;
	pdata		=	0;

	return	res;

}

static int bssq_flash_led_suspend(struct i2c_client *client, pm_message_t message)
{
//	printk(KERN_INFO "enter %s\n", __func__);

	bssq_flash_led_enable(0);

	return	0;
}

static int bssq_flash_led_resume(struct i2c_client *client)
{
//	printk(KERN_INFO "enter %s\n", __func__);

	bssq_flash_led_enable(1);

	return 0;
}

static const struct i2c_device_id id_table_bssq_flash_led[] =
{
	{	bssq_flash_led_I2C_NAME,	0	},
	{}
};

static struct i2c_driver i2c_driver_bssq_flash_led =
{
	.driver = {
		.owner	=	THIS_MODULE,
		.name	=	bssq_flash_led_I2C_NAME,
	},
	.probe		=	bssq_flash_led_probe,
	.remove		=	__devexit_p(bssq_flash_led_remove),
	.suspend	=	bssq_flash_led_suspend,
	.resume		=	bssq_flash_led_resume,
	.id_table	=	id_table_bssq_flash_led,
};

static int __init bssq_flash_led_init(void)
{
	printk(KERN_INFO "enter %s\n", __func__);
	return i2c_add_driver(&i2c_driver_bssq_flash_led);
}

static void __exit bssq_flash_led_exit(void)
{
	printk(KERN_INFO "enter %s\n", __func__);
	i2c_del_driver(&i2c_driver_bssq_flash_led);
}


module_init(bssq_flash_led_init);
module_exit(bssq_flash_led_exit);     

MODULE_AUTHOR("chen.xuming@lge.com");
MODULE_DESCRIPTION("bssq_flash_led synchronous boost flash driver");
MODULE_LICENSE("GPL");
