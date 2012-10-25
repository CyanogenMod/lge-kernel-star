/*
 * Cosmopolitan GPS GPIO Control driver
 *
 * linux/arch/arm/mach-omap2/cosmo-gps.c
 *
 * Copyright (C) 2010 LGE, Inc.
 * Author: Miok Park <miok.park@lge.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * taesook.yoon@lge.com 20110426
 * android\kernel\drivers\p940\misc/gps_gpio.c
 * < GPS on Android >
 * GPS Android SW Architecture.pdf
 * Broadcom GPS driver is a daemon process
 * So, Custom GPIO kernel driver to toggle reset and stndby pins is needed
 * The GPIO toggling can be exposed as a sysfs path.
 */

#include <linux/kernel.h>
#include <linux/device.h>

#include <linux/platform_device.h>
#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <mach/gpio.h>

#include <linux/input.h>
#include <linux/fsl_devices.h>

#include <mach-tegra/board.h>
#include <mach-tegra/clock.h>
#include <lge/board-star.h>
#include <lge/board-star-gps.h>
#include <mach-tegra/devices.h>
#include <mach-tegra/gpio-names.h>

/* LGE_CHANGE_S youngseok@lge.com
모델 회로도에서 GPS가 사용하는 RESET/ POWERON(STANDBY) 를 확인, LNA는 추후 확인. 
B2(P940)'   ( kernel manager :엄주관 J)
	GPS_PWR_ON  :	KB_ROW6 / GPIO_PJ2  
	GPS_RESET_N :	KB_ROW7 / GPIO_PJ0
	

	GPS_UART_TXD :	UART4
	GPS_UART_RXD :	UART4
	GPS_UART_RTS_N :	UART4
	GPS_UART_CTS_N :	UART4

LGE_CHANGE_E youngseok@lge.com */

/* LGE_CHANGE_S  20111104  gsd5xp dongseon.kim@lge.com
26MHz_GPS_REF_EN : GMI_AD8 / GPIO_H00

 LGE_CHANGE_S  20111104  gsd5xp dongseon.kim@lge.com */

struct gps_gpio_platform_data gps_pdata = {
	.pwron  = TEGRA_GPIO_PJ2,
	.reset_n = TEGRA_GPIO_PJ0,
	.lna_sd  = TEGRA_GPIO_PD0,
};

struct platform_device gps_gpio =
{
	.name	= "gps_gpio",
	.id	= -1,
	.dev	= {
		.platform_data = &gps_pdata,
	}
};

void star_gps_init(void)
{
	platform_device_register(&gps_gpio);
}

//LGE_CHANGE_S dongseon.kim@lge.com add GPS-GPIO.c 
static ssize_t gps_gpio_reset_show(struct device *dev, 
		struct device_attribute *attr, char *buf)
{
	int value;
	struct gps_gpio_platform_data *pdata = dev->platform_data;
	
	pr_info("%s\n", __func__);

	value = gpio_get_value(pdata->reset_n);

	pr_info("%s(gpio_get_value) %d %d\n", __func__,pdata->reset_n,value);

	return sprintf(buf, "%d\n", value);
}

static ssize_t gps_gpio_reset_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int value;
	struct gps_gpio_platform_data *pdata = dev->platform_data;

	pr_info("%s\n", __func__);

	sscanf(buf, "%d", &value);

	gpio_set_value(pdata->reset_n, value);

	pr_info("%s(gpio_set_value) %d %d\n", __func__,pdata->reset_n,value);

	return size;
}

static ssize_t gps_gpio_poweron_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int value;
	struct gps_gpio_platform_data *pdata = dev->platform_data;

	pr_info("%s\n", __func__);

	value = gpio_get_value(pdata->pwron);

	pr_info("%s(gpio_get_value) %d %d\n", __func__,pdata->pwron,value);

	return sprintf(buf, "%d\n", value);
}

static ssize_t gps_gpio_poweron_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int value;
	struct gps_gpio_platform_data *pdata = dev->platform_data;

	pr_info("%s\n", __func__);

	sscanf(buf, "%d", &value);

	gpio_set_value(pdata->pwron, value);

	pr_info("%s(gpio_set_value) %d %d\n", __func__,pdata->pwron,value);

	gpio_set_value(pdata->lna_sd, value);
	
	pr_info("%s(gpio_set_lna_sd_value) %d %d\n", __func__,pdata->lna_sd,value);

	return size;
}

static DEVICE_ATTR(reset, S_IRUGO | S_IWUSR, gps_gpio_reset_show, gps_gpio_reset_store);
static DEVICE_ATTR(poweron, S_IRUGO | S_IWUSR, gps_gpio_poweron_show, gps_gpio_poweron_store);

static int __devinit gps_gpio_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct gps_gpio_platform_data *pdata = pdev->dev.platform_data;
	unsigned pwron, reset_n;

	pr_info("%s\n", __func__);

	if (!pdata)
		return -EINVAL;

	ret = gpio_request(pdata->pwron, "GPS power on GPIO");
	if (ret) {
		pr_err("%s: failed to request GPIO_%d\n", __func__, pdata->pwron);
		goto err_gpio_pwron_req;
	}
	gpio_direction_output(pdata->pwron, 0);

	tegra_gpio_enable(pdata->pwron);
	
	ret = gpio_request(pdata->reset_n, "GPS reset GPIO");
	if (ret) {
		pr_err("%s: failed to request GPIO_%d\n", __func__, pdata->reset_n);
		goto err_gpio_reset_req;
	}
	gpio_direction_output(pdata->reset_n, 0);
	
	tegra_gpio_enable(pdata->reset_n);

//#if defined(CONFIG_P940_GPS_LNA_SD_USE)
	ret = gpio_request(pdata->lna_sd, "GPS extend LNA GPIO");
	if (ret) {
		pr_err("%s: failed to request GPIO_%d\n", __func__, pdata->lna_sd);
		goto err_gpio_lna_sd_req;
	}
	gpio_direction_output(pdata->lna_sd, 0);

	tegra_gpio_enable(pdata->lna_sd);	
//#endif

	ret = device_create_file(&pdev->dev, &dev_attr_reset);
	if (ret) {
		pr_err("%s: failed to create \"dev_attr_reset\" attribute!\n",
				__func__);
		goto err_reset_attr_create;
	}

	ret = device_create_file(&pdev->dev, &dev_attr_poweron);
	if (ret) {
		pr_err("%s: failed to create \"dev_attr_poweron\" attribute!\n",
				__func__);
		goto err_pwron_attr_create;
	}

	return 0;

err_pwron_attr_create:
	device_remove_file(&pdev->dev, &dev_attr_reset);
err_reset_attr_create:
	gpio_free(pdata->lna_sd);
err_gpio_lna_sd_req:
	gpio_free(pdata->reset_n);
err_gpio_reset_req:
	gpio_free(pdata->pwron);
err_gpio_pwron_req:
	return ret;
}


static int __devexit gps_gpio_remove(struct platform_device *pdev)
{
	pr_info("%s\n", __func__);
	device_remove_file(&pdev->dev, &dev_attr_reset);
	device_remove_file(&pdev->dev, &dev_attr_poweron);
	return 0;
}


static struct platform_driver gps_gpio_driver = {
	.probe	= gps_gpio_probe,
	.remove	= __devexit_p(gps_gpio_remove),
	.driver	= {
		.name   = "gps_gpio",
		.owner  = THIS_MODULE
	},
};


static int __devinit gps_gpio_init(void)
{
	pr_info("%s\n", __func__);
	return platform_driver_register(&gps_gpio_driver);
}

static void __exit gps_gpio_exit(void)
{
	pr_info("%s\n", __func__);
	platform_driver_unregister(&gps_gpio_driver);
}

module_init(gps_gpio_init);
module_exit(gps_gpio_exit);

MODULE_AUTHOR("LG Electronics");
MODULE_DESCRIPTION("GPS GPIO Controller");
MODULE_LICENSE("GPL v2");
