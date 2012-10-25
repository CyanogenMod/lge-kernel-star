#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/backlight.h>
#include <linux/fb.h>
#include <linux/i2c.h>
#include <linux/string.h>
#include <linux/regulator/consumer.h>
#include <lge/lp8720.h>
#include <../gpio-names.h>
#include <asm/gpio.h>
#include <asm/system.h>

static struct i2c_client *lp8720_client = NULL;

//20110901 hyongbink@nvidia.com
static struct regulator *regulator_vddio_mipi = NULL;
static struct regulator *regulator_vddio_vi = NULL;

static u8	cam	=	0;
static u8	af	=	0;
static u8	vt	=	0;
static int	enabled	=	0;

static void	lp8720_enable(struct i2c_client* client, int status)
{
 	struct lp8720_platform_data*	pdata	=	client->dev.platform_data;

printk("lp8720_enable: %d", status);

	if (status == 0) {	// Standby mode
		gpio_set_value(pdata->en_gpio_num, 0);
		udelay(25*7); //ts*7
		udelay(200); //tbon
	}
	else {				// Normal mode
		gpio_set_value(pdata->en_gpio_num, 1);
		udelay(200); //tbon
		udelay(25*7); //ts*7
		
		// Recover LDE settings
		if (enabled == 0) {
			i2c_smbus_write_byte_data(client, LP8720_LDO1_SETTING, 0x19);	// 2.8V VT_CAM_DRV_2.8V
			i2c_smbus_write_byte_data(client, LP8720_LDO2_SETTING, 0x0c);	// 1.8V VT_CAM_IO_1.8V
			i2c_smbus_write_byte_data(client, LP8720_LDO3_SETTING, 0x17);	// 2.7V 2.7V_5M_VANA
			i2c_smbus_write_byte_data(client, LP8720_LDO4_SETTING, 0x11);	// 1.8V 1.8V_5M_VIO
			i2c_smbus_write_byte_data(client, LP8720_LDO5_SETTING, 0x19);	// 2.8V 2.8V_5M_VCM
			i2c_smbus_write_byte_data(client, LP8720_BUCK_SETTING1, 0x09);	// Buck = 1.2V 1.2V_5M_VDIG
			i2c_smbus_write_byte_data(client, LP8720_BUCK_SETTING2, 0x09);	// Buck = 1.2V

			printk("[lp8720_enable] LP8720_LDO1_SETTING: %d\n", i2c_smbus_read_byte_data(client, LP8720_LDO1_SETTING));
			printk("[lp8720_enable] LP8720_LDO2_SETTING: %d\n", i2c_smbus_read_byte_data(client, LP8720_LDO2_SETTING));
			printk("[lp8720_enable] LP8720_LDO3_SETTING: %d\n", i2c_smbus_read_byte_data(client, LP8720_LDO3_SETTING));
			printk("[lp8720_enable] LP8720_LDO4_SETTING: %d\n", i2c_smbus_read_byte_data(client, LP8720_LDO4_SETTING));
			printk("[lp8720_enable] LP8720_LDO5_SETTING: %d\n", i2c_smbus_read_byte_data(client, LP8720_LDO5_SETTING));			
		}
	}

	enabled	=	status;
}

static void	lp8720_ldo_update(struct i2c_client* client)
{
	u8	output	=	0;

	printk(KERN_DEBUG "%s: cam(%d) af(%d) vt(%d)\n", __FUNCTION__, cam, af, vt);

	if (cam)	output	|=	(BUCK_EN | LDO3_EN | LDO4_EN);
	if (af)		output	|=	LDO5_EN;
	if (vt)		output	|=	(LDO1_EN | LDO2_EN);

	if (output)
		output	|=	DVS_V1;

	lp8720_enable(client, output);
	if(output)
		i2c_smbus_write_byte_data(client, LP8720_OUTPUT_ENABLE, output);
}

static ssize_t lp8720_show_cam(struct device *dev,
                struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", cam);
}

static ssize_t lp8720_store_cam(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client	=	to_i2c_client(dev);
	int	num	=	simple_strtoul(buf, NULL, 10);

	if ((num == 0) || (num == 1)) {
		cam	=	num;
		lp8720_ldo_update(client);
	}

	return	count;
}

static ssize_t lp8720_show_af(struct device *dev,
                struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", af);
}

static ssize_t lp8720_store_af(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client	=	to_i2c_client(dev);
	int		num	=	simple_strtoul(buf, NULL, 10);

	if ((num == 0) || (num == 1)) {
		af	=	num;
		lp8720_ldo_update(client);
	}

	return	count;
}

static ssize_t lp8720_show_vt(struct device *dev,
                struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", vt);
}

static ssize_t lp8720_store_vt(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client	=	to_i2c_client(dev);
	int		num	=	simple_strtoul(buf, NULL, 10);

	if ((num == 0) || (num == 1)) {
		vt	=	num;
		lp8720_ldo_update(client);
	}

	return	count;
}

static DEVICE_ATTR(cam, 0644, lp8720_show_cam, lp8720_store_cam);
static DEVICE_ATTR(af, 0644, lp8720_show_af, lp8720_store_af);
static DEVICE_ATTR(vt, 0644, lp8720_show_vt, lp8720_store_vt);

// LGE_CHANGE_S [dongjin73.kim@lge.com] 2011-01-19, [LGE_AP20] Power interface for camera sensor
int star_cam_power_off(void)
{
	printk(KERN_DEBUG "%s: cam(%d) af(%d) vt(%d)\n", __FUNCTION__, cam, af, vt);
	if (lp8720_client == NULL)
		return	-EINVAL;

	cam	=	0;
	af	=	0;
	vt	=	0;
	
	lp8720_ldo_update(lp8720_client);

	//20110901 hyongbink@nvidia.com
	if(regulator_vddio_mipi != NULL)
		regulator_disable(regulator_vddio_mipi);

	return	0;
}

int star_cam_Main_power_on(void)
{
	printk(KERN_DEBUG "%s: cam(%d) af(%d) vt(%d)\n", __FUNCTION__, cam, af, vt);
	if (lp8720_client == NULL)
		return	-EINVAL;

	//20110901 hyongbink@nvidia.com
	if(regulator_vddio_mipi != NULL)
	{
		regulator_enable(regulator_vddio_mipi);
		mdelay(1);
	}

	cam	=	1;
	af	=	1;
	vt	=	0;

	lp8720_ldo_update(lp8720_client);

	return	0;
}

int star_cam_VT_power_on(void)
{
	printk(KERN_DEBUG "%s: cam(%d) af(%d) vt(%d)\n", __FUNCTION__, cam, af, vt);
	if (lp8720_client == NULL)
		return	-EINVAL;

	//20110901 hyongbink@nvidia.com
	if(regulator_vddio_mipi != NULL)
	{
		regulator_enable(regulator_vddio_mipi);
		mdelay(1);
	}

	cam	=	0;
	af	=	0;
	vt	=	1;

	lp8720_ldo_update(lp8720_client);

	return	0;
}

EXPORT_SYMBOL(star_cam_power_off);
EXPORT_SYMBOL(star_cam_Main_power_on);
EXPORT_SYMBOL(star_cam_VT_power_on);
// LGE_CHANGE_E [dongjin73.kim@lge.com] 2011-01-19, [LGE_AP20] Power interface for camera sensor

#ifdef CONFIG_PM
static int lp8720_suspend(struct i2c_client *client, pm_message_t mesg)
{
	printk(KERN_DEBUG "%s: cam(%d) af(%d) vt(%d)\n", __FUNCTION__, cam, af, vt);

	lp8720_enable(client, 0);

	return	0;
}

static int lp8720_resume(struct i2c_client *client)
{
	printk(KERN_DEBUG "%s: cam(%d) af(%d) vt(%d)\n", __FUNCTION__, cam, af, vt);

	lp8720_ldo_update(client);

	return	0;
}
#else
#define lp8720_suspend        NULL
#define lp8720_resume         NULL
#endif /* CONFIG_PM */

static int __init lp8720_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct lp8720_platform_data *pdata = NULL;
	int err;

    printk("%s: ++++\n", __FUNCTION__);

	if (i2c_get_clientdata(client))
		return	-EBUSY;

	regulator_vddio_vi = regulator_get(NULL, "vddio_vi");
	if (IS_ERR_OR_NULL(regulator_vddio_vi)) {
		pr_err("%s: Couldn't get regulator vddio_vi\n", __func__);
	}

	/* set vddio_vi voltage to 1.8v */
	err = regulator_set_voltage(regulator_vddio_vi, 1800*1000, 1800*1000);
	if (err) {
		pr_err("%s: Failed to set vddio_vi to 1.8v\n", __func__);
	}

	regulator_enable(regulator_vddio_vi);

	//20110901 hyongbink@nvidia.com
	regulator_vddio_mipi = regulator_get(NULL, "vddio_mipi");
	if (IS_ERR_OR_NULL(regulator_vddio_mipi)) {
		pr_err("dsi: couldn't get regulator vddio_mipi\n");
		regulator_vddio_mipi = NULL;
	}

	// device data
	pdata	=	client->dev.platform_data;
	lp8720_client	=	client;
    printk("    : lp8720 enable gpio num: %d)\n", pdata->en_gpio_num);

	// gpio setup
	tegra_gpio_enable(pdata->en_gpio_num);
	gpio_request(pdata->en_gpio_num, "hwen_lp8720");
	gpio_direction_output(pdata->en_gpio_num, 0);

	lp8720_enable(client, 0);	// turn off for initial status
#if 0	
	mdelay(1);
	lp8720_enable(client, 1);	// turn on for test
#endif 

	// sysfs
	err = device_create_file(&client->dev, &dev_attr_cam);
	if (err)
		goto exit_sysfs;
	err = device_create_file(&client->dev, &dev_attr_af);
	if (err)
		goto exit_sysfs;
	err = device_create_file(&client->dev, &dev_attr_vt);
	if (err)
		goto exit_sysfs;

	return	0;

exit_sysfs:
	return	err;
}

static int lp8720_remove(struct i2c_client *client)
{
	lp8720_enable(client, 0);

	return	0;
}	

static const struct i2c_device_id lp8720_ids[] = {
	{	LP8720_I2C_NAME,	0	},
	{},
};

static struct i2c_driver subpm_lp8720_driver = {
	.probe		= lp8720_probe,
	.remove		= lp8720_remove,
	.suspend	= lp8720_suspend,
	.resume		= lp8720_resume,
	.id_table	= lp8720_ids,
	.driver = {
		.name	= LP8720_I2C_NAME,
		.owner	= THIS_MODULE,
    },
};

static int __init subpm_lp8720_init(void)
{
	return i2c_add_driver(&subpm_lp8720_driver);
}

static void __exit subpm_lp8720_exit(void)
{
	i2c_del_driver(&subpm_lp8720_driver);
}

module_init(subpm_lp8720_init);
module_exit(subpm_lp8720_exit);

MODULE_AUTHOR("LG Electronics");
MODULE_DESCRIPTION("LP8720 sub pmic Driver");
MODULE_LICENSE("GPL");
