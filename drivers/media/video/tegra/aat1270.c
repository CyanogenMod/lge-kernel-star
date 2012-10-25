/*
 * camera flash led aat1270 
 *
 * Copyright (C) 2012 LGE, Inc.
 *
 * Author: chen yingchun  <chen.yingchun@lge.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <mach/gpio.h>
#include <media/aat1270.h>
#include <linux/err.h>

static struct aat1270_platform_data*   pdata   =       0;
static unsigned char    flash_brightness        =       7;

static ssize_t flash_store(struct device* dev,
				struct device_attribute* attr, const char* buf, size_t count)
{
	unsigned long time = simple_strtoul(buf, NULL, 10);

	gpio_set_value(pdata->gpio_flen, 0);
	gpio_set_value(pdata->gpio_enset, 0);
	if(time > 0) {
	        gpio_set_value(pdata->gpio_flen, 1);
	        gpio_set_value(pdata->gpio_enset, 1);
	}

	return count;
}
static DEVICE_ATTR(flash, 0666, NULL, flash_store);

static ssize_t flash_brightness_store(struct device* dev,
				struct device_attribute* attr, const char* buf, size_t count)
{
	unsigned long value = simple_strtoul(buf, NULL, 10);

	flash_brightness = (unsigned char)value;

	return  count;
}

static ssize_t flash_brightness_show(struct device* dev,
				struct device_attribute* attr, const char* buf, size_t count)
{
	return  snprintf(buf, PAGE_SIZE, "%d\n", flash_brightness);	
}
static DEVICE_ATTR(flash_brightness, 0666, flash_brightness_show, flash_brightness_store);

static ssize_t torch_store(struct device* dev,
				 struct device_attribute* attr, const char* buf, size_t count)
{
	unsigned long brightness = simple_strtoul(buf, NULL, 10);
	
	gpio_set_value(pdata->gpio_enset, 0);
	if(brightness > 0) {
		gpio_set_value(pdata->gpio_enset, 1);
	}	

	return  count;
}
static DEVICE_ATTR(torch, 0666, NULL, torch_store);

static ssize_t enable_store(struct device* dev,
				struct device_attribute* attr, const char* buf, size_t count)
{
	int status = simple_strtoul(buf, NULL, 10);

	gpio_set_value(pdata->gpio_flen, 0);
	gpio_set_value(pdata->gpio_enset, 0);

	return  count;
}

static ssize_t enable_show(struct device* dev,
				struct device_attribute* attr, const char* buf, size_t count)
{
	return  snprintf(buf, PAGE_SIZE, "%d\n", gpio_get_value(pdata->gpio_flen));
}
static DEVICE_ATTR(enable, 0666, enable_show, enable_store);


static void aat1270_remove(struct platform_device *pdev)
{
	printk("%s\n", __func__);
	gpio_free(pdata->gpio_flen);
        gpio_free(pdata->gpio_enset);

	device_remove_file(&pdev->dev, &dev_attr_enable);
	device_remove_file(&pdev->dev, &dev_attr_flash);
	device_remove_file(&pdev->dev, &dev_attr_flash_brightness);
	device_remove_file(&pdev->dev, &dev_attr_torch);
}

static void aat1270_shutdown(struct platform_device *pdev)
{
	printk("%s\n", __func__);
	gpio_free(pdata->gpio_flen);
	gpio_free(pdata->gpio_enset);

        device_remove_file(&pdev->dev, &dev_attr_enable);
        device_remove_file(&pdev->dev, &dev_attr_flash);
        device_remove_file(&pdev->dev, &dev_attr_flash_brightness);
        device_remove_file(&pdev->dev, &dev_attr_torch);
}

static int aat1270_probe(struct platform_device *pdev)
{
	int res;

	printk("%s\n", __func__);

	if (pdev->dev.platform_data == NULL){
		dev_err(&pdev->dev, "no platform data\n");
		return -ENODEV;
	}

	pdata = pdev->dev.platform_data;

	tegra_gpio_enable(pdata->gpio_flen);
	res = gpio_request(pdata->gpio_flen, "flash_en");
	if (res < 0) {
		printk("[aat1270] gpio_request : gpio_flen=%d, ret=%x \n",pdata->gpio_flen,res);
	}
	res = gpio_direction_output(pdata->gpio_flen, 0);
	if (res < 0) {
		printk("[aat1270] gpio_direction_output : flen_gpio_num=%d, ret=%x \n",pdata->gpio_flen,res);
	}
	
        tegra_gpio_enable(pdata->gpio_enset);
        res = gpio_request(pdata->gpio_enset, "flash_enset");
        if (res < 0) {
                printk("[aat1270] gpio_request : gpio_enset=%d, ret=%x \n",pdata->gpio_enset,res);
        }
        res = gpio_direction_output(pdata->gpio_enset, 0);
        if (res < 0) {
                printk("[aat1270] gpio_direction_output : enset_gpio_num=%d, ret=%x \n",pdata->gpio_enset,res);
        }


	res = device_create_file(&pdev->dev, &dev_attr_flash);
	if(res){
		printk("[aat1270] device create file flash fail!\n");
		goto exit;
	}

        res = device_create_file(&pdev->dev, &dev_attr_flash_brightness);
        if(res){
                printk("[aat1270] device create file flash_brightness fail!\n");
                goto exit;
        }

	res = device_create_file(&pdev->dev, &dev_attr_torch);
        if(res){
                printk("[aat1270] device create file torch fail!\n");
                goto exit;
        }

        res = device_create_file(&pdev->dev, &dev_attr_enable);
        if(res){
                printk("[aat1270] device create file enable fail!\n");
                goto exit;
        }

	return 0;
	
exit:

        gpio_free(pdata->gpio_flen);
        gpio_free(pdata->gpio_enset);

        device_remove_file(&pdev->dev, &dev_attr_enable);
        device_remove_file(&pdev->dev, &dev_attr_flash);
        device_remove_file(&pdev->dev, &dev_attr_flash_brightness);
        device_remove_file(&pdev->dev, &dev_attr_torch);

	return res;
}

static struct platform_driver aat1270_driver = {
	.probe	= aat1270_probe,
	.remove = aat1270_remove,
	.shutdown = aat1270_shutdown,
	.driver = {
		.name 	= "aat1270",
		.owner	= THIS_MODULE,
	},
};

static int __init aat1270_init(void)
{
	return platform_driver_register(&aat1270_driver);
}

static void __exit aat1270_exit(void)
{
	platform_driver_unregister(&aat1270_driver);
}

module_init(aat1270_init);
module_exit(aat1270_exit);

MODULE_AUTHOR("Chen Yingchun <chen.yingchun@lge.com>");
MODULE_DESCRIPTION("aat1270 driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:aat1270");

