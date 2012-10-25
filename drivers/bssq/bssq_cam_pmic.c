/*
* drivers/startablet/star_cam_pmic.c
*
* Star Camera PMIC i2c client driver
*
* Copyright (c) 2010, LG Electronics Corporation.
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
* 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
*/
//TODO[LGE_AP20] It is a star tablet code. need to fix tablet dependency code
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/miscdevice.h> //20110525 calvin.hwang@lge.com Camsensor ks1001 merge
#include <linux/regulator/consumer.h> //20110525 calvin.hwang@lge.com Camsensor ks1001 merge
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#include <linux/wakelock.h>
#include <linux/freezer.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <../gpio-names.h>
#include <mach/hardware.h>
#include <lge/bssq_cam_pmic.h>  //20110525 calvin.hwang@lge.com Camsensor ks1001 merge

// LDO1 : 2.8V (25uS*4), LDO2 : 2.8V (25uS*4), LDO3 : 1.9V (25uS*2), LDO4 : 1.25V (0us),  (LDO1, LDO3 On)

//TODO[LGE_AP20] change Power set [PowerSetForSony5M, PowerSetForSS2M]
// LGE_CHANGE_S [sungmo.yang@lge.com] 2011-01-13 [LGE_AP20] subpmic setting
// static u16 PowerSetForSony5M[] =	{0x0199, 0x0299, 0x034e, 0x0406, 0x0000, 0x0000, 0x080f};
// static u16 PowerSetForSS2M[]   =	{0x011A, 0x02F7, 0x036D, 0x04E5, 0x0000, 0x0000, 0x0805};

static u16 PowerSetMainCAM[] 	=	{0x0319, 0x0419, 0x0519, 0x0609, 0x0709, 0x08A0, 0x08A8, 0x08BC};
static u16 PowerSetSubCAM[] 	=	{0x010C, 0x0219, 0x0000, 0x0000, 0x0000, 0x0880, 0x0883};

#define PowerSetMainCAM_MAX ((sizeof(PowerSetMainCAM)) /(sizeof(PowerSetMainCAM[0])))
#define PowerSetSubCAM_MAX 	((sizeof(PowerSetSubCAM)) /(sizeof(PowerSetSubCAM[0])))

// LGE_CHANGE_E [sungmo.yang@lge.com] 2011-01-13 [LGE_AP20] subpmic setting

// LGE_CHANGE_S [sungmo.yang@lge.com] 2011-01-14, [LGE_AP20] change [Cam_Power_EN -> CAMERA_SUBPMIC_EN]
//static int Cam_Power_EN;
// LGE_CHANGE_E [sungmo.yang@lge.com] 2011-01-14, [LGE_AP20] change [Cam_Power_EN -> CAMERA_SUBPMIC_EN]

static struct i2c_client* star_cam_power_client;
//20110525 calvin.hwang@lge.com Camsensor ks1001 merge [S]
#if defined (CONFIG_KS1001) || defined (CONFIG_KS1103)

//20111108 hyongbink@nvidia.com
static struct regulator *regulator_vddio_mipi = NULL;

static struct regulator *cam_regulator_avdd_2v8 = NULL;//cam_avdd_2v8
static struct regulator *cam_regulator_lvdd_2v8 = NULL; //cam_lvdd_2v8
static struct regulator *cam_regulator_iovdd_1v8 = NULL; //cam_iovdd_1v8

static struct regulator *cam_regulator_vt_avdd_2v8 = NULL; //cam_vt_2v8
static struct regulator *cam_regulator_vt_iovdd_1v8 = NULL; //cam_vt_1v8

static u8	main_cam	=	0;
static u8	vt_cam	=	0;

#elif defined (CONFIG_LU6500) || defined (CONFIG_SU880) || defined (CONFIG_KU8800) || defined (CONFIG_LU8800)

#define CAMERA_FLASH_EN      	TEGRA_GPIO_PBB4
#define CAMERA_FLASH_FINH      	TEGRA_GPIO_PT3
#define CAMERA_FLASH_ENSET      TEGRA_GPIO_PT2
#define CAMERA_SUBPMIC_EN 		TEGRA_GPIO_PF2
#endif
//20110525 calvin.hwang@lge.com Camsensor ks1001 merge [E]

//LGE_CHANGE_E [sungmo.yang@lge.com] 2011-01-14, Fix Camera Flash GPIO address

static int star_cam_flash_gpio_init(void)
{
	printk("############star_cam_flash_gpio_init###############\n");
#if 0	 //20110525 calvin.hwang@lge.com Camsensor ks1001 merge
	tegra_gpio_enable(CAMERA_FLASH_EN);
	gpio_request(CAMERA_FLASH_EN, "camera_flash_en");
	gpio_direction_output(CAMERA_FLASH_EN, 0);
	gpio_export(CAMERA_FLASH_EN, 0);

	tegra_gpio_enable(CAMERA_FLASH_FINH);
	gpio_request(CAMERA_FLASH_FINH, "camera_flash_finh");
	gpio_direction_output(CAMERA_FLASH_FINH, 0);
	gpio_export(CAMERA_FLASH_FINH, 0);

	tegra_gpio_enable(CAMERA_FLASH_ENSET);
	gpio_request(CAMERA_FLASH_ENSET, "camera_flash_enset");
	gpio_direction_output(CAMERA_FLASH_ENSET, 0);
	gpio_export(CAMERA_FLASH_ENSET, 0);
#endif	 //20110525 calvin.hwang@lge.com Camsensor ks1001 merge
	return 0;

}


int star_cam_Main_power_on(void)
{
//20110525 calvin.hwang@lge.com Camsensor ks1001 merge [S]
#if defined (CONFIG_KS1001) || defined (CONFIG_KS1103)
	int ret;

	printk("############CAMERA PMIC ###############\n");
	printk("[%s]\n", __FUNCTION__);

	//20111108 hyongbink@nvidia.com
	if(regulator_vddio_mipi != NULL)
	{
		regulator_enable(regulator_vddio_mipi);
		mdelay(1);
	}

	if(cam_regulator_iovdd_1v8>0)
	{
		ret = regulator_enable(cam_regulator_iovdd_1v8);
		if (ret)
			return ret;
	}
	mdelay(10);
	if(cam_regulator_avdd_2v8>0)
	{
		ret = regulator_enable(cam_regulator_avdd_2v8);
		if (ret)
			return ret;
	}
	mdelay(10);
	if(cam_regulator_lvdd_2v8>0)
	{
		ret = regulator_enable(cam_regulator_lvdd_2v8);
		if (ret)
			return ret;
	}

    main_cam = 1;
#elif defined (CONFIG_LU6500)	|| defined (CONFIG_SU880) || defined (CONFIG_KU8800) || defined (CONFIG_LU8800)

	int i =0;
	int ret = 0;
	u8 reg;
	u8 data;

	tegra_gpio_enable(CAMERA_SUBPMIC_EN);
	gpio_direction_output(CAMERA_SUBPMIC_EN,1);
	mdelay(10);
	gpio_set_value(CAMERA_SUBPMIC_EN,1);
	mdelay(10);
	printk("############CAMERA PMIC ###############\n");
	printk("[%s]\n", __FUNCTION__);

	for(i=0;i<= PowerSetMainCAM_MAX ;i++) {
		reg = (u8)((PowerSetMainCAM[i]>>8)&0xFF);
		data = (u8)(PowerSetMainCAM[i] & 0x00FF);

		ret = i2c_smbus_write_byte_data(star_cam_power_client, reg, data);
		if(ret)
		{
			printk("[%s]\n I2C error err = %d",__FUNCTION__, ret);
			return ret;
		}
	}
#endif
//20110525 calvin.hwang@lge.com Camsensor ks1001 merge [E]

	return 0;
}


//TODO[LGE_AP20] check i2c_smbus_write_byte_data() function
int star_cam_power_off(void)
{
//20110525 calvin.hwang@lge.com Camsensor ks1001 merge [S]
#if defined (CONFIG_KS1001) || defined (CONFIG_KS1103)
	int ret;

	printk("############CAMERA PMIC ###############\n");
	printk("[%s]:main_cam:%d, vt_cam:%d \n", __FUNCTION__,main_cam,vt_cam);

	//Main 8M
	if (main_cam != 0)
	{
		if(cam_regulator_iovdd_1v8>0)
		{
			ret = regulator_disable(cam_regulator_iovdd_1v8);
			if (ret)
				return ret;
		}

		if(cam_regulator_avdd_2v8>0)
		{
			ret = regulator_disable(cam_regulator_avdd_2v8);
			if (ret)
				return ret;
		}

		if(cam_regulator_lvdd_2v8>0)
		{
			ret = regulator_disable(cam_regulator_lvdd_2v8);
			if (ret)
				return ret;
		}

		//20111108 hyongbink@nvidia.com
		if(regulator_vddio_mipi != NULL)
			regulator_disable(regulator_vddio_mipi);

		main_cam = 0;
	}

	//VGA
	if (vt_cam != 0)
	{
		if(cam_regulator_vt_avdd_2v8>0)
		{
			ret = regulator_disable(cam_regulator_vt_avdd_2v8);
			if (ret)
				return ret;
		}

		if(cam_regulator_vt_iovdd_1v8>0)
		{
			ret = regulator_disable(cam_regulator_vt_iovdd_1v8);
			if (ret)
				return ret;
		}
		vt_cam = 0;
	}
#elif defined (CONFIG_LU6500)	|| defined (CONFIG_SU880) || defined (CONFIG_KU8800) || defined (CONFIG_LU8800)

	int i =0;
	int ret = 0;
	u8 reg;
	u8 data;

	ret = i2c_smbus_write_byte_data(star_cam_power_client, 0x08, 0x80);
	if(ret)
	{
		printk("[%s]\n I2C error err = %d",__FUNCTION__, ret);
		return ret;
	}
	gpio_set_value(CAMERA_SUBPMIC_EN,0);
	tegra_gpio_disable(CAMERA_SUBPMIC_EN);
#endif
//20110525 calvin.hwang@lge.com Camsensor ks1001 merge [E]

	return 0;
}


int star_cam_VT_power_on(void)
{
	int ret;

//20110525 calvin.hwang@lge.com Camsensor ks1001 merge [S]
#if defined (CONFIG_KS1001) || defined (CONFIG_KS1103)
	printk("############CAMERA PMIC ###############\n");
	printk("[%s]\n", __FUNCTION__);

	if(cam_regulator_vt_avdd_2v8>0)
	{
		ret = regulator_enable(cam_regulator_vt_avdd_2v8);
		if (ret)
			return ret;
	}

	if(cam_regulator_vt_iovdd_1v8>0)
	{
		ret = regulator_enable(cam_regulator_vt_iovdd_1v8);
		if (ret)
			return ret;
	}

    vt_cam = 1;
#elif defined (CONFIG_LU6500) || defined (CONFIG_SU880) || defined (CONFIG_KU8800) || defined (CONFIG_LU8800)

	int i =0;
	int ret = 0;
	u8 reg;
	u8 data;

	tegra_gpio_enable(CAMERA_SUBPMIC_EN);
	gpio_direction_output(CAMERA_SUBPMIC_EN,1);
	mdelay(10);
	gpio_set_value(CAMERA_SUBPMIC_EN,1);
	printk("############CAMERA PMIC ###############\n");
	printk("[%s]\n", __FUNCTION__);

	for(i=0;i<= PowerSetSubCAM_MAX ;i++) {
		reg = (u8)((PowerSetSubCAM[i]>>8)&0xFF);
		data = (u8)(PowerSetSubCAM[i] & 0x00FF);

		ret = i2c_smbus_write_byte_data(star_cam_power_client, reg, data);
		if(!ret)
		{
			printk("[%s]\n I2C error err = %d",__FUNCTION__, ret);
			return ret;
		}
	}
#endif
//20110525 calvin.hwang@lge.com Camsensor ks1001 merge [E]

	return 0;
}
//20110525 calvin.hwang@lge.com Camsensor ks1001 merge [S]
static const struct file_operations star_cam_pmic_fops = {
	.owner = THIS_MODULE,
//	.unlocked_ioctl = star_cam_pmic_ioctl,
//	.release = star_cam_pmic_release,
};

static struct miscdevice star_cam_pmic_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = BSSQ_CAM_PMIC,
	.fops = &star_cam_pmic_fops,
};
//20110525 calvin.hwang@lge.com Camsensor ks1001 merge [E]

static int star_cam_pmic_probe(struct platform_device *pdev)
{
//20110525 calvin.hwang@lge.com Camsensor ks1001 merge [S]
#if defined (CONFIG_KS1001) || defined (CONFIG_KS1103)
	int err;

	printk("############CAMERA PMIC ###############\n");
	printk("[%s]\n", __FUNCTION__);
	printk("###############################\n");

	//Main 8M

	//20111108 hyongbink@nvidia.com
	regulator_vddio_mipi = regulator_get(NULL, "vddio_mipi");
	if (IS_ERR_OR_NULL(regulator_vddio_mipi)) {
		pr_err("dsi: couldn't get regulator vddio_mipi\n");
		regulator_vddio_mipi = NULL;
	}

	cam_regulator_avdd_2v8 = regulator_get(&pdev->dev, "cam_avdd_2v8");
	if (IS_ERR_OR_NULL(cam_regulator_avdd_2v8))
	{
		pr_err("%s: Couldn't get regulator cam_regulator_avdd_2v8!\n",
				__FUNCTION__);
		cam_regulator_avdd_2v8 = NULL;
		return PTR_ERR(cam_regulator_avdd_2v8);
	}

	cam_regulator_lvdd_2v8 = regulator_get(&pdev->dev, "cam_lvdd_2v8");
	if (IS_ERR_OR_NULL(cam_regulator_lvdd_2v8))
	{
		pr_err("%s: Couldn't get regulator cam_regulator_lvdd_2v8!\n",
				__FUNCTION__);
		cam_regulator_lvdd_2v8 = NULL;
		return PTR_ERR(cam_regulator_lvdd_2v8);
	}

	cam_regulator_iovdd_1v8 = regulator_get(&pdev->dev, "cam_iovdd_1v8");
	if (IS_ERR_OR_NULL(cam_regulator_iovdd_1v8))
	{
		pr_err("%s: Couldn't get regulator cam_regulator_iovdd_1v8!\n",
				__FUNCTION__);
		cam_regulator_iovdd_1v8 = NULL;
		return PTR_ERR(cam_regulator_iovdd_1v8);
	}

	//VGA
	cam_regulator_vt_avdd_2v8 = regulator_get(&pdev->dev, "cam_vt_2v8");
	if (IS_ERR_OR_NULL(cam_regulator_vt_avdd_2v8))
	{
		pr_err("%s: Couldn't get regulator cam_regulator_vt_avdd_2v8!\n",
				__FUNCTION__);
		cam_regulator_vt_avdd_2v8 = NULL;
		return PTR_ERR(cam_regulator_vt_avdd_2v8);
	}

	cam_regulator_vt_iovdd_1v8 = regulator_get(&pdev->dev, "cam_vt_1v8");
	if (IS_ERR_OR_NULL(cam_regulator_vt_iovdd_1v8))
	{
		pr_err("%s: Couldn't get regulator cam_regulator_vt_iovdd_1v8!\n",
				__FUNCTION__);
		cam_regulator_vt_iovdd_1v8 = NULL;
		return PTR_ERR(cam_regulator_vt_iovdd_1v8);
	}

    //Main 8M
	err = regulator_set_voltage(cam_regulator_avdd_2v8, 2800000, 2800000);
	pr_err("%s: regulator_set_voltage(cam_regulator_avdd_2v8): %d\n",
				__FUNCTION__, err);
	err = regulator_set_voltage(cam_regulator_lvdd_2v8, 2800000, 2800000);
	pr_err("%s: regulator_set_voltage(cam_regulator_lvdd_2v8): %d\n",
				__FUNCTION__, err);
	err = regulator_set_voltage(cam_regulator_iovdd_1v8, 1800000, 1800000);
	pr_err("%s: regulator_set_voltage(cam_regulator_iovdd_1v8): %d\n",
				__FUNCTION__, err);

	//VGA
	printk("cam_regulator_vt_avdd_2v8 set voltage\n");
	err = regulator_set_voltage(cam_regulator_vt_avdd_2v8, 2800000, 2800000);
	pr_err("%s: regulator_set_voltage(cam_regulator_vt_avdd_2v8): %d\n",
				__FUNCTION__, err);
	err = regulator_set_voltage(cam_regulator_vt_iovdd_1v8, 1800000, 1800000);
	pr_err("%s: regulator_set_voltage(cam_regulator_vt_iovdd_1v8): %d\n",
				__FUNCTION__, err);

	pr_info("%s: probe2\n", __FUNCTION__);

	err = misc_register(&star_cam_pmic_device);
	if (err) {
		pr_err("%s: Unable to register misc device!\n",
				__FUNCTION__);
		goto misc_register_err;
	}

	return 0;

misc_register_err:

	//20111108 hyongbink@nvidia.com
	if(regulator_vddio_mipi>0)
		regulator_put(regulator_vddio_mipi);

	if(cam_regulator_avdd_2v8>0)
		regulator_put(cam_regulator_avdd_2v8);
	if(cam_regulator_lvdd_2v8>0)
		regulator_put(cam_regulator_lvdd_2v8);
	if(cam_regulator_iovdd_1v8>0)
		regulator_put(cam_regulator_iovdd_1v8);
	if(cam_regulator_vt_avdd_2v8>0)
		regulator_put(cam_regulator_vt_avdd_2v8);
	if(cam_regulator_vt_iovdd_1v8>0)
		regulator_put(cam_regulator_vt_iovdd_1v8);
	return err;

#elif defined (CONFIG_LU6500) || defined (CONFIG_SU880) || defined (CONFIG_KU8800) || defined (CONFIG_LU8800)

	struct i2c_client *pclient = client;
	unsigned short addr = pclient->addr;
	//hw_rev board_rev=0;
	star_cam_power_client = client;
	star_cam_flash_gpio_init();
	//board_rev = get_hw_rev();

// LGE_CHANGE_S [sungmo.yang@lge.com] 2011-01-14, [LGE_AP20] Fix Camera SubPMIC GPIO address
//	if (board_rev == REV_E || board_rev == REV_F)
//		Cam_Power_EN = TEGRA_GPIO_PS5;
//	else
//		Cam_Power_EN = TEGRA_GPIO_PQ1;
//
//  gpio_request(CAMERA_SUBPMIC_EN, "Cam_Power_En");
// LGE_CHANGE_E [sungmo.yang@lge.com] 2011-01-14, [LGE_AP20] Fix Camera SubPMIC GPIO address



	printk("############CAMERA PMIC ###############\n");
	printk("[%s]\n", __FUNCTION__);
	printk("[%s] - client address : %x\n", __FUNCTION__, addr);
	printk("###############################\n");


	return 0;
#endif
//20110525 calvin.hwang@lge.com Camsensor ks1001 merge [E]
}

static int star_cam_pmic_remove(struct platform_device *pdev)
{
//20110525 calvin.hwang@lge.com Camsensor ks1001 merge [S]
#if defined (CONFIG_KS1001) || defined (CONFIG_KS1103)
    //Main 8M

	//20111108 hyongbink@nvidia.com
	if(regulator_vddio_mipi>0)
		regulator_put(regulator_vddio_mipi);

	if(cam_regulator_avdd_2v8>0)
		regulator_put(cam_regulator_avdd_2v8);
	if(cam_regulator_lvdd_2v8>0)
		regulator_put(cam_regulator_lvdd_2v8);
	if(cam_regulator_iovdd_1v8>0)
		regulator_put(cam_regulator_iovdd_1v8);

	//VGA
	if(cam_regulator_vt_avdd_2v8>0)
		regulator_put(cam_regulator_vt_avdd_2v8);
	if(cam_regulator_vt_iovdd_1v8>0)
		regulator_put(cam_regulator_vt_iovdd_1v8);
#elif defined (CONFIG_LU6500)	|| defined (CONFIG_SU880) || defined (CONFIG_KU8800) || defined (CONFIG_LU8800)

	gpio_set_value(CAMERA_SUBPMIC_EN,0);
	tegra_gpio_disable(CAMERA_SUBPMIC_EN);
// LGE_CHANGE_S [sungmo.yang@lge.com] 2011-01-14, change [Cam_Power_EN -> CAMERA_SUBPMIC_EN]
	gpio_free(CAMERA_SUBPMIC_EN);
// LGE_CHANGE_E [sungmo.yang@lge.com] 2011-01-14, change [Cam_Power_EN -> CAMERA_SUBPMIC_EN]
#endif
//20110525 calvin.hwang@lge.com Camsensor ks1001 merge [E]

	return 0;
}


static struct platform_driver star_cam_pmic_driver = {
	.probe = star_cam_pmic_probe,
	.remove = star_cam_pmic_remove,
	.driver = {
		.name = BSSQ_CAM_PMIC,
	},
};

static int __init star_cam_pmic_init(void)
{
        pr_info("star_cam_pmic_init sensor driver loading\n");
	return platform_driver_register(&star_cam_pmic_driver);
}

static void __exit star_cam_pmic_exit(void)
{
	platform_driver_unregister(&star_cam_pmic_driver);
}

module_init(star_cam_pmic_init);
module_exit(star_cam_pmic_exit);

