#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/backlight.h>
#include <linux/fb.h>
#include <linux/i2c.h>
#include <linux/string.h>
#include <linux/regulator/consumer.h>
#include <mach/lp8720.h>
#include <mach/gpio-names.h>
#include <asm/gpio.h>
#include <asm/system.h>

//20110718 calvin.hwang@lge.com early suspend
#ifdef CONFIG_HAS_EARLYSUSPEND
#undef CONFIG_HAS_EARLYSUSPEND
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

// vi enable pin issue #860932
#define LGE_CONTROL_VI_ENABLE (0)

//20110629 jinkwan.kim@lge.com vi-power save[S]
static struct i2c_client *lp8720_client = NULL;
#if LGE_CONTROL_VI_ENABLE
static struct regulator *regulator_vddio_vi = NULL;
static int	regulator_enabled = 0;
#endif
//20110629 jinkwan.kim@lge.com vi-power save[E]

static u8	cam	=	0;
static u8	af	=	0;
static u8	vt	=	0;
static int	enabled	=	0;

#ifdef CONFIG_HAS_EARLYSUSPEND
static struct early_suspend lp8720_early_suspend;
#endif

#define RETURN_IF_FAIL(a) {ret = a; if(ret < 0) return ret;}

static int	lp8720_enable(struct i2c_client* client, int status)
{
	int ret = 0;
 	struct lp8720_platform_data*	pdata	=	client->dev.platform_data;

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
			RETURN_IF_FAIL(i2c_smbus_write_byte_data(client, LP8720_LDO1_SETTING, 0x19));	// 2.8V VT_CAM_DRV_2.8V
			RETURN_IF_FAIL(i2c_smbus_write_byte_data(client, LP8720_LDO2_SETTING, 0x0c));	// 1.8V VT_CAM_IO_1.8V
			RETURN_IF_FAIL(i2c_smbus_write_byte_data(client, LP8720_LDO3_SETTING, 0x17));	// 2.7V 2.7V_5M_VANA
			RETURN_IF_FAIL(i2c_smbus_write_byte_data(client, LP8720_LDO4_SETTING, 0x11));	// 1.8V 1.8V_5M_VIO
			RETURN_IF_FAIL(i2c_smbus_write_byte_data(client, LP8720_LDO5_SETTING, 0x19));	// 2.8V 2.8V_5M_VCM
			RETURN_IF_FAIL(i2c_smbus_write_byte_data(client, LP8720_BUCK_SETTING1, 0x09));	// Buck = 1.2V 1.2V_5M_VDIG
			RETURN_IF_FAIL(i2c_smbus_write_byte_data(client, LP8720_BUCK_SETTING2, 0x09));	// Buck = 1.2V
		}
	}
	enabled	=	status;

	return ret;
}

static int	lp8720_ldo_update(struct i2c_client* client)
{
	int ret = 0;
	u8	output	=	0;

	printk(KERN_DEBUG "%s: cam(%d) af(%d) vt(%d)\n", __FUNCTION__, cam, af, vt);

	if (cam)	output	|=	(BUCK_EN | LDO3_EN | LDO4_EN);
	if (af)		output	|=	LDO5_EN;
	if (vt)		output	|=	(LDO1_EN | LDO2_EN);

	if (output)
		output	|=	DVS_V1;

	RETURN_IF_FAIL(lp8720_enable(client, output));
	
	if(output)
	{
		RETURN_IF_FAIL(i2c_smbus_write_byte_data(client, LP8720_OUTPUT_ENABLE, output));
	}

	return ret;
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

#if LGE_CONTROL_VI_ENABLE
	if(regulator_vddio_vi != NULL)
	{
		if (regulator_disable(regulator_vddio_vi))
		{
			printk(KERN_ERR "disable vddio_vi failed\n");
			return -EINVAL;
		}

		regulator_enabled = 0;
	}
#endif // LGE_CONTROL_VI_ENABLE

	return	0;
}

int star_cam_Main_power_on(void)
{
	printk(KERN_DEBUG "%s: cam(%d) af(%d) vt(%d)\n", __FUNCTION__, cam, af, vt);
	if (lp8720_client == NULL)
		return	-EINVAL;

#if LGE_CONTROL_VI_ENABLE
	if(regulator_vddio_vi != NULL)
	{
		if (regulator_enable(regulator_vddio_vi))
			return -EINVAL;
		
		mdelay(5);
		regulator_enabled = 1;
	}
#endif // LGE_CONTROL_VI_ENABLE

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

#if LGE_CONTROL_VI_ENABLE
	if(regulator_vddio_vi != NULL)
	{
		if (regulator_enable(regulator_vddio_vi))
			return -EINVAL;
			
		mdelay(5);
		regulator_enabled = 1;
	}
#endif // LGE_CONTROL_VI_ENABLE

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

    // WAR for exceptional case that suspended with sub-pmic enabled. In normal case variables, cam, af and vt, should be FALSE,
    // but if camera applicatoin closed with any error without disabling camra sensor,
    // they remain enabled. In this case resume sequence try to enable sub-pmic so reset them to FALSE here.
	if(cam || af || vt)
	{
		printk(KERN_DEBUG "WAR %s: abnormal case => reset variables\n", __FUNCTION__, cam, af, vt);
		cam = af = vt = 0;
	}

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

#ifdef CONFIG_HAS_EARLYSUSPEND
void	lp8720_early_suspend_func(struct early_suspend *h)
{
	pm_message_t pt;
	//printk(KERN_DEBUG "%s: regulator(%d)\n", __FUNCTION__, regulator_enabled);

	lp8720_suspend(lp8720_client, pt);
}

void	lp8720_early_resume_func(struct early_suspend *h)
{
	//printk(KERN_DEBUG "%s: regulator(%d)\n", __FUNCTION__, regulator_enabled);

	lp8720_resume(lp8720_client);
}
#endif

static int __init lp8720_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct lp8720_platform_data *pdata = NULL;
	int err;

    printk(KERN_DEBUG "%s: ++++\n", __FUNCTION__);

	if (i2c_get_clientdata(client))
		return	-EBUSY;

#if LGE_CONTROL_VI_ENABLE
	// setup regulator_vddio_vi
	// TODO : need to move or changed for power management
	regulator_vddio_vi = regulator_get(NULL, "vddio_vi");
	if (regulator_vddio_vi == NULL) {
		printk(KERN_DEBUG "lp8720: vddio_vi failed\n");
		goto	exit_sysfs;
	}
	
	if(regulator_vddio_vi != NULL)
	{	
		printk(KERN_DEBUG "Enable and disable VI\n");	
		regulator_set_voltage(regulator_vddio_vi, 1800000, 1800000);
		regulator_enable(regulator_vddio_vi);
		mdelay(5);
		err = regulator_disable(regulator_vddio_vi);
		if (err)
		{
			printk(KERN_ERR "disable vddio_vi failed\n");	
			return err;
		}
		
		regulator_enabled = 0;
	}
	else
	{
		printk(KERN_INFO "lp8720: vddio_vi failed2\n");
	}
#endif // LGE_CONTROL_VI_ENABLE

	// device data
	pdata	=	client->dev.platform_data;
	lp8720_client	=	client;
    printk("    : lp8720 enable gpio num: %d)\n", pdata->en_gpio_num);

	// gpio setup
	tegra_gpio_enable(pdata->en_gpio_num);
	gpio_request(pdata->en_gpio_num, "hwen_lp8720");
	gpio_direction_output(pdata->en_gpio_num, 0);

	lp8720_enable(client, 0);	// turn off for initial status

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

#ifdef CONFIG_HAS_EARLYSUSPEND
	lp8720_early_suspend.level		=	EARLY_SUSPEND_LEVEL_BLANK_SCREEN -1;
	lp8720_early_suspend.suspend	=	lp8720_early_suspend_func;
	lp8720_early_suspend.resume		=	lp8720_early_resume_func;
	register_early_suspend(&lp8720_early_suspend);
#endif

	return	0;

exit_sysfs:
	return	err;
}

static int lp8720_remove(struct i2c_client *client)
{
	lp8720_enable(client, 0);

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&lp8720_early_suspend);
#endif
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
