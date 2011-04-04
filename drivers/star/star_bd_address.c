/*
 * linux/drivers/star/star_bd_address.c
 *
 * Copyright (C) 2010 LG Electronics, Inc.
 * Author: LG Electronics, Inc.
 * 
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>

// 20101108 BT: Added for the BT Address Read Factory command dohyung10.lee [Start]
// dohyung10.lee@lge.com
/** @brief  To store Bluetooth Address value that is sent via AT command from CP
    @author dohyung10.lee@lge.com
    @date   2010.11.08
*/
char mBDAddr[13];

static ssize_t bd_address_store(struct device *dev,struct device_attribute *attr,const char *buf, size_t count)
{
    
	sscanf(buf, "%s", &mBDAddr);

    return count;
}

/** @brief  To read Bluetooth Address value that is sent via AT command from CP
    @author dohyung10.lee@lge.com
    @date   2010.11.08
*/

static ssize_t bd_address_show(struct device *dev,struct device_attribute *attr,const char *buf, size_t count)
{
   
   return sprintf(buf, "%s\n", mBDAddr);
}
static DEVICE_ATTR(bdaddr_if, 0666, bd_address_show, bd_address_store);


/** @brief  It doesn't do anything. Just a stub.
    @author dohyung10.lee@lge.com
    @date   2010.11.11
*/

static int __exit bd_address_remove(struct platform_device *pdev)
{
	return 0;
}

/** @brief  It doesn't do anything. Just a stub.
    @author dohyung10.lee@lge.com
    @date   2010.11.11
*/

static int __init bd_address_probe(struct platform_device *pdev)
{
	int ret;

	ret = device_create_file(&pdev->dev, &dev_attr_bdaddr_if);
    if (ret) {
        printk( "BD Addr sysfs register failed: Fail\n");
    }

	return ret;
}

/** @brief  It doesn't do anything. Just a stub.
    @author dohyung10.lee@lge.com
    @date   2010.11.11
*/

static int bd_address_suspend(struct platform_device *pdev,
	pm_message_t state)
{
	return 0;
}

/** @brief  It doesn't do anything. Just a stub.
    @author dohyung10.lee@lge.com
    @date   2010.11.11
*/

static int bd_address_resume(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver bd_address_driver = {
	.probe		= bd_address_probe,
	.remove		= __exit_p(bd_address_remove),
	.suspend	= bd_address_suspend,
	.resume		= bd_address_resume,
	.driver		= {
		.name	= "star_bd_address",
	},
};

MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:star_bd_address");
MODULE_AUTHOR("LG Electronics Inc");

static int __init bd_address_init(void)
{
	return platform_driver_register(&bd_address_driver);
}
module_init(bd_address_init);

static void __exit bd_address_exit(void)
{
	platform_driver_unregister(&bd_address_driver);
}
module_exit(bd_address_exit);
//20101108 BT: Added for the BT Address Read Factory command dohyung10.lee [End]
