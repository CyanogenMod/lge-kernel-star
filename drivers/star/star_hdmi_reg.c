/*
 * star(lgp990) HDMI regulator
 *
 * Copyright (C) 2009 LGE, Inc.
 *
 * Author: Changsu Ha <cs77.ha@lge.com>
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

/**
	@brief		 star(lgp990) HDMI regulator
 
	@author		 cs77.ha@lge.com
	@date		 2010-07-02
 
	@version	 V1.00		 2010.07.02		 Changsu Ha	 Create
*/

#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/uaccess.h>

#include "mach/nvrm_linux.h"
#include "nvcommon.h"
#include "nvos.h"
#include "nvodm_services.h"

typedef struct HdmiRegRec
{
    NvOdmServicesGpioHandle gpioHandle;
    NvOdmGpioPinHandle  pinHandle;
} HdmiReg;

static HdmiReg s_hdmi_reg;

static int hdmi_reg_suspend(struct platform_device *pdev, pm_message_t state)
{
    NvOdmGpioSetState(s_hdmi_reg.gpioHandle, s_hdmi_reg.pinHandle, 0);
    return 0;
}

static int hdmi_reg_resume(struct platform_device *pdev)
{
    NvOdmGpioSetState(s_hdmi_reg.gpioHandle, s_hdmi_reg.pinHandle, 1);
    return 0;
}

static int __init hdmi_reg_probe(struct platform_device *pdev)
{
    int ret;
    NvU32 pin, port;

    //GPIO configuration
    s_hdmi_reg.gpioHandle = NvOdmGpioOpen();
    if (!s_hdmi_reg.gpioHandle)
    {
        printk(KERN_ERR "[star modem_chk] NvOdmGpioOpen Error \n");
        goto err_probe_fail;
    }
    port = 'k'-'a';
    pin = 5;
    s_hdmi_reg.pinHandle = NvOdmGpioAcquirePinHandle(s_hdmi_reg.gpioHandle, 
                                                    port, pin);
    if (!s_hdmi_reg.pinHandle)
    {
        printk(KERN_ERR "[star modem_chk] NvOdmGpioAcquirePinHandle Error\n");
        goto err_gpio_pin_acquire_fail;
    }
    NvOdmGpioSetState(s_hdmi_reg.gpioHandle, s_hdmi_reg.pinHandle, 1);
    NvOdmGpioConfig(s_hdmi_reg.gpioHandle, s_hdmi_reg.pinHandle, 
                    NvOdmGpioPinMode_Output);
    
    return 0;

    NvOdmGpioReleasePinHandle(s_hdmi_reg.gpioHandle, s_hdmi_reg.pinHandle);
err_gpio_pin_acquire_fail:
    NvOdmGpioClose(s_hdmi_reg.gpioHandle); 
err_probe_fail:
    return -ENOSYS;
}

static int hdmi_reg_remove(struct platform_device *pdev)
{
    NvOdmGpioReleasePinHandle(s_hdmi_reg.gpioHandle, s_hdmi_reg.pinHandle);
    NvOdmGpioClose(s_hdmi_reg.gpioHandle); 

    return 0;
}

static struct platform_driver hdmi_reg = {
    .probe      = hdmi_reg_probe,
    .remove     = hdmi_reg_remove,
    .suspend    = hdmi_reg_suspend,
    .resume     = hdmi_reg_resume,
    .driver = {
        .name   = "star_hdmi_reg",
        .owner  = THIS_MODULE,
    },
};

static int __init hdmi_reg_init(void)
{
    return platform_driver_register(&hdmi_reg);
}

static void __exit hdmi_reg_exit(void)
{
    platform_driver_unregister(&hdmi_reg);
}

module_init(hdmi_reg_init);
module_exit(hdmi_reg_exit);

MODULE_AUTHOR("cs77.ha@lge.com");
MODULE_DESCRIPTION("star hdmi regulator");
MODULE_LICENSE("GPL");

