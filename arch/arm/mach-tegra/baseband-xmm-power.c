/*
 * arch/arm/mach-tegra/baseband-xmm-power.c
 *
 * Copyright (C) 2011 NVIDIA Corporation
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/wakelock.h>
#include <linux/usb.h>
#include <mach/usb_phy.h>
#include "board.h"
#include "devices.h"
#include "gpio-names.h"
#include "baseband-xmm-power.h"

MODULE_LICENSE("GPL");

unsigned long modem_ver = XMM_MODEM_VER_1121;
EXPORT_SYMBOL(modem_ver);

unsigned long modem_flash;
EXPORT_SYMBOL(modem_flash);

unsigned long modem_pm = 1;
EXPORT_SYMBOL(modem_pm);

unsigned long enum_delay_ms = 1000; /* ignored if !modem_flash */

module_param(modem_ver, ulong, 0644);
MODULE_PARM_DESC(modem_ver,
	"baseband xmm power - modem software version");
module_param(modem_flash, ulong, 0644);
MODULE_PARM_DESC(modem_flash,
	"baseband xmm power - modem flash (1 = flash, 0 = flashless)");
module_param(modem_pm, ulong, 0644);
MODULE_PARM_DESC(modem_pm,
	"baseband xmm power - modem power management (1 = pm, 0 = no pm)");
module_param(enum_delay_ms, ulong, 0644);
MODULE_PARM_DESC(enum_delay_ms,
	"baseband xmm power - delay in ms between modem on and enumeration");

static struct usb_device_id xmm_pm_ids[] = {
	{ USB_DEVICE(VENDOR_ID, PRODUCT_ID),
	.driver_info = 0 },
	{}
};

static struct gpio tegra_baseband_gpios[] = {
	{ -1, GPIOF_OUT_INIT_LOW,  "BB_RSTn" },
	{ -1, GPIOF_OUT_INIT_LOW,  "BB_ON"   },
	{ -1, GPIOF_OUT_INIT_LOW,  "IPC_BB_WAKE" },
	{ -1, GPIOF_IN,            "IPC_AP_WAKE" },
	{ -1, GPIOF_OUT_INIT_HIGH, "IPC_HSIC_ACTIVE" },
	{ -1, GPIOF_IN,            "IPC_HSIC_SUS_REQ" },
};

static enum {
	IPC_AP_WAKE_UNINIT,
	IPC_AP_WAKE_IRQ_READY,
	IPC_AP_WAKE_INIT1,
	IPC_AP_WAKE_INIT2,
	IPC_AP_WAKE_L,
	IPC_AP_WAKE_H,
} ipc_ap_wake_state;

enum baseband_xmm_powerstate_t baseband_xmm_powerstate;
static struct workqueue_struct *workqueue;
static struct work_struct init1_work;
static struct work_struct init2_work;
static struct work_struct L2_resume_work;
static struct baseband_power_platform_data *baseband_power_driver_data;
static bool register_hsic_device;
static struct wake_lock wakelock;
static struct usb_device *usbdev;
static bool CP_initiated_L2toL0;
static bool modem_power_on;
static int power_onoff;
static void baseband_xmm_power_L2_resume(void);

static int baseband_modem_power_on(struct baseband_power_platform_data *data)
{
	/* set IPC_HSIC_ACTIVE active */
	gpio_set_value(baseband_power_driver_data->
		modem.xmm.ipc_hsic_active, 1);

	/* wait 20 ms */
	mdelay(20);

	/* reset / power on sequence */
	mdelay(40);
	gpio_set_value(data->modem.xmm.bb_rst, 1);
	mdelay(1);
	gpio_set_value(data->modem.xmm.bb_on, 1);
	udelay(40);
	gpio_set_value(data->modem.xmm.bb_on, 0);

	return 0;
}

static int baseband_xmm_power_on(struct platform_device *device)
{
	struct baseband_power_platform_data *data
		= (struct baseband_power_platform_data *)
			device->dev.platform_data;
	pr_debug("%s {\n", __func__);

	/* check for platform data */
	if (!data) {
		pr_err("%s: !data\n", __func__);
		return -EINVAL;
	}

	/* reset the state machine */
	baseband_xmm_powerstate = BBXMM_PS_INIT;
	if (modem_ver < XMM_MODEM_VER_1130)
		ipc_ap_wake_state = IPC_AP_WAKE_INIT1;
	else
		ipc_ap_wake_state = IPC_AP_WAKE_INIT2;

	pr_debug("%s - %d\n", __func__, __LINE__);

	/* register usb host controller */
	if (!modem_flash) {
		pr_debug("%s - %d\n", __func__, __LINE__);
		/* register usb host controller only once */
		if (register_hsic_device) {
			pr_debug("%s: register usb host controller\n",
				__func__);
			modem_power_on = true;
			if (data->hsic_register)
				data->modem.xmm.hsic_device =
						data->hsic_register();
			else
				pr_err("%s: hsic_register is missing\n",
					__func__);
			register_hsic_device = false;
		} else {
			/* register usb host controller */
			if (data->hsic_register)
				data->modem.xmm.hsic_device =
							data->hsic_register();
			/* turn on modem */
			pr_debug("%s call baseband_modem_power_on\n", __func__);
			baseband_modem_power_on(data);
		}
	}

	pr_debug("%s }\n", __func__);

	return 0;
}

static int baseband_xmm_power_off(struct platform_device *device)
{
	struct baseband_power_platform_data *data;

	pr_debug("%s {\n", __func__);

	/* check for device / platform data */
	if (!device) {
		pr_err("%s: !device\n", __func__);
		return -EINVAL;
	}
	data = (struct baseband_power_platform_data *)
		device->dev.platform_data;
	if (!data) {
		pr_err("%s: !data\n", __func__);
		return -EINVAL;
	}

	/* unregister usb host controller */
	pr_info("%s: hsic device: %p\n", __func__, data->modem.xmm.hsic_device);
	if (data->hsic_unregister)
		data->hsic_unregister(data->modem.xmm.hsic_device);
	else
		pr_err("%s: hsic_unregister is missing\n", __func__);

	/* set IPC_HSIC_ACTIVE low */
	gpio_set_value(baseband_power_driver_data->
		modem.xmm.ipc_hsic_active, 0);

	/* wait 20 ms */
	mdelay(20);

	/* drive bb_rst low */
	gpio_set_value(data->modem.xmm.bb_rst, 0);
	mdelay(1);

	pr_debug("%s }\n", __func__);

	return 0;
}

static ssize_t baseband_xmm_onoff(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	int size;
	struct platform_device *device = to_platform_device(dev);

	pr_debug("%s\n", __func__);

	/* check input */
	if (buf == NULL) {
		pr_err("%s: buf NULL\n", __func__);
		return -EINVAL;
	}
	pr_debug("%s: count=%d\n", __func__, count);

	/* parse input */
	size = sscanf(buf, "%d", &power_onoff);
	if (size != 1) {
		pr_err("%s: size=%d -EINVAL\n", __func__, size);
		return -EINVAL;
	}
	pr_debug("%s power_onoff=%d\n", __func__, power_onoff);

	if (power_onoff == 0)
		baseband_xmm_power_off(device);
	else if (power_onoff == 1)
		baseband_xmm_power_on(device);
	return count;
}

static DEVICE_ATTR(xmm_onoff, S_IRUSR | S_IWUSR | S_IRGRP,
		NULL, baseband_xmm_onoff);

void baseband_xmm_set_power_status(unsigned int status)
{
	struct baseband_power_platform_data *data = baseband_power_driver_data;
	int value = 0;

	pr_debug("%s\n", __func__);

	if (baseband_xmm_powerstate == status)
		return;

	switch (status) {
	case BBXMM_PS_L0:
		pr_info("L0\n");
		value = gpio_get_value(data->modem.xmm.ipc_hsic_active);
		pr_debug("before L0 ipc_hsic_active=%d\n", value);
		if (!value) {
			pr_debug("before L0 gpio set ipc_hsic_active=1 ->\n");
			gpio_set_value(data->modem.xmm.ipc_hsic_active, 1);
		}
		if (modem_power_on) {
			modem_power_on = false;
			baseband_modem_power_on(data);
		}
		wake_lock(&wakelock);
		pr_debug("gpio host active high->\n");
		break;
	case BBXMM_PS_L2:
		pr_info("L2\n");
		wake_unlock(&wakelock);
		break;
	case BBXMM_PS_L3:
		pr_info("L3\n");
		if (wake_lock_active(&wakelock)) {
			pr_info("%s: releasing wakelock before L3\n",
				__func__);
			wake_unlock(&wakelock);
		}
		gpio_set_value(data->modem.xmm.ipc_hsic_active, 0);
		pr_debug("gpio host active low->\n");
		break;
	case BBXMM_PS_L2TOL0:
		/* do this only from L2 state */
		if (baseband_xmm_powerstate == BBXMM_PS_L2) {
			baseband_xmm_powerstate = status;
			pr_debug("BB XMM POWER STATE = %d\n", status);
			baseband_xmm_power_L2_resume();
		}
	default:
		break;
	}
	baseband_xmm_powerstate = status;
	pr_debug("BB XMM POWER STATE = %d\n", status);
}
EXPORT_SYMBOL_GPL(baseband_xmm_set_power_status);

irqreturn_t baseband_xmm_power_ipc_ap_wake_irq(int irq, void *dev_id)
{
	int value;

	pr_debug("%s\n", __func__);

	if (ipc_ap_wake_state < IPC_AP_WAKE_IRQ_READY) {
		pr_err("%s - spurious irq\n", __func__);
	} else if (ipc_ap_wake_state == IPC_AP_WAKE_IRQ_READY) {
		value = gpio_get_value(baseband_power_driver_data->
			modem.xmm.ipc_ap_wake);
		if (!value) {
			pr_debug("%s - IPC_AP_WAKE_INIT1"
				" - got falling edge\n",
				__func__);
			/* go to IPC_AP_WAKE_INIT1 state */
			ipc_ap_wake_state = IPC_AP_WAKE_INIT1;
			/* queue work */
			queue_work(workqueue, &init1_work);
		} else {
			pr_debug("%s - IPC_AP_WAKE_INIT1"
				" - wait for falling edge\n",
				__func__);
		}
	} else if (ipc_ap_wake_state == IPC_AP_WAKE_INIT1) {
		value = gpio_get_value(baseband_power_driver_data->
			modem.xmm.ipc_ap_wake);
		if (!value) {
			pr_debug("%s - IPC_AP_WAKE_INIT2"
				" - wait for rising edge\n",
				__func__);
		} else {
			pr_debug("%s - IPC_AP_WAKE_INIT2"
				" - got rising edge\n",
				__func__);
			/* go to IPC_AP_WAKE_INIT2 state */
			ipc_ap_wake_state = IPC_AP_WAKE_INIT2;
			/* queue work */
			queue_work(workqueue, &init2_work);
		}
	} else {
		value = gpio_get_value(baseband_power_driver_data->
			modem.xmm.ipc_ap_wake);
		if (!value) {
			pr_debug("%s - falling\n", __func__);
			/* [ver < 1130] gpio protocol falling edge */
			if (modem_ver < XMM_MODEM_VER_1130) {
				pr_debug("gpio host wakeup done <-\n");
				value = gpio_get_value
					(baseband_power_driver_data->
					modem.xmm.ipc_bb_wake);
				if (value) {
					/* Clear the slave wakeup request */
					gpio_set_value
						(baseband_power_driver_data->
						modem.xmm.ipc_bb_wake, 0);
					pr_debug("gpio slave wakeup done ->\n");
				}
			}
			/* [ver >= 1130] gpio protocol falling edge */
			if (modem_ver >= XMM_MODEM_VER_1130) {
				if (baseband_xmm_powerstate == BBXMM_PS_L2) {
					CP_initiated_L2toL0 = true;
					baseband_xmm_set_power_status
						(BBXMM_PS_L2TOL0);
				}
			}
			/* save gpio state */
			ipc_ap_wake_state = IPC_AP_WAKE_L;
		} else {
			pr_debug("%s - rising\n", __func__);
			/* [ver >= 1130] gpio protocol rising edge */
			if (modem_ver >= XMM_MODEM_VER_1130) {
				pr_debug("gpio host wakeup done <-\n");
				value = gpio_get_value
					(baseband_power_driver_data->
					modem.xmm.ipc_bb_wake);
				if (value) {
					/* Clear the slave wakeup request */
					gpio_set_value
						(baseband_power_driver_data->
						modem.xmm.ipc_bb_wake, 0);
					pr_debug("gpio slave wakeup done ->\n");
				}
				baseband_xmm_set_power_status(BBXMM_PS_L0);
			}
			/* save gpio state */
			ipc_ap_wake_state = IPC_AP_WAKE_H;
		}
	}

	return IRQ_HANDLED;
}
EXPORT_SYMBOL(baseband_xmm_power_ipc_ap_wake_irq);

static void baseband_xmm_power_init1_work(struct work_struct *work)
{
	int value;

	pr_debug("%s {\n", __func__);

	/* check if IPC_HSIC_ACTIVE high */
	value = gpio_get_value(baseband_power_driver_data->
		modem.xmm.ipc_hsic_active);
	if (value != 1) {
		pr_err("%s - expected IPC_HSIC_ACTIVE high!\n", __func__);
		return;
	}

	/* wait 100 ms */
	mdelay(100);

	/* set IPC_HSIC_ACTIVE low */
	gpio_set_value(baseband_power_driver_data->
		modem.xmm.ipc_hsic_active, 0);

	/* wait 10 ms */
	mdelay(10);

	/* set IPC_HSIC_ACTIVE high */
	gpio_set_value(baseband_power_driver_data->
		modem.xmm.ipc_hsic_active, 1);

	/* wait 20 ms */
	mdelay(20);

	pr_debug("%s }\n", __func__);
}

static void baseband_xmm_power_init2_work(struct work_struct *work)
{
	struct baseband_power_platform_data *data = baseband_power_driver_data;

	pr_debug("%s\n", __func__);

	/* check input */
	if (!data)
		return;

	/* register usb host controller only once */
	if (register_hsic_device) {
		if (data->hsic_register)
			data->modem.xmm.hsic_device = data->hsic_register();
		else
			pr_err("%s: hsic_register is missing\n", __func__);
		register_hsic_device = false;
	}

}

/* Do the work for AP/CP initiated L2->L0 */
static void baseband_xmm_power_L2_resume(void)
{
	struct baseband_power_platform_data *data = baseband_power_driver_data;
	int value;
	int delay = 10000; /* maxmum delay in msec */

	pr_debug("%s\n", __func__);

	if (!baseband_power_driver_data)
		return;
	if (CP_initiated_L2toL0)  {
		pr_info("CP L2->L0\n");
		CP_initiated_L2toL0 = false;
		queue_work(workqueue, &L2_resume_work);
	} else {
		/* set the slave wakeup request */
		pr_info("AP L2->L0\n");
		gpio_set_value(data->modem.xmm.ipc_bb_wake, 1);
		pr_debug("waiting for host wakeup from CP...\n");
		do {
			mdelay(1);
			value = gpio_get_value(data->modem.xmm.ipc_ap_wake);
			delay--;
		} while ((value) && (delay));
		if (delay)
			pr_debug("gpio host wakeup low <-\n");
		else
			pr_info("!!AP L2->L0 Failed\n");
	}
}

/* Do the work for CP initiated L2->L0 */
static void baseband_xmm_power_L2_resume_work(struct work_struct *work)
{
	struct usb_interface *intf;

	pr_debug("%s {\n", __func__);

	if (!usbdev)
		return;
	usb_lock_device(usbdev);
	intf = usb_ifnum_to_if(usbdev, 0);
	if (usb_autopm_get_interface(intf) == 0)
		usb_autopm_put_interface(intf);
	usb_unlock_device(usbdev);

	pr_debug("} %s\n", __func__);
}

static void baseband_xmm_power_reset_on(void)
{
	/* reset / power on sequence */
	mdelay(40);
	gpio_set_value(baseband_power_driver_data->modem.xmm.bb_rst, 1);
	mdelay(1);
	gpio_set_value(baseband_power_driver_data->modem.xmm.bb_on, 1);
	udelay(40);
	gpio_set_value(baseband_power_driver_data->modem.xmm.bb_on, 0);
}

static struct baseband_xmm_power_work_t *baseband_xmm_power_work;

static void baseband_xmm_power_work_func(struct work_struct *work)
{
	struct baseband_xmm_power_work_t *bbxmm_work
		= (struct baseband_xmm_power_work_t *) work;

	pr_debug("%s\n", __func__);

	switch (bbxmm_work->state) {
	case BBXMM_WORK_UNINIT:
		pr_debug("BBXMM_WORK_UNINIT\n");
		break;
	case BBXMM_WORK_INIT:
		pr_debug("BBXMM_WORK_INIT\n");
		/* go to next state */
		bbxmm_work->state = (modem_flash && !modem_pm)
			? BBXMM_WORK_INIT_FLASH_STEP1
			: (modem_flash && modem_pm)
			? BBXMM_WORK_INIT_FLASH_PM_STEP1
			: (!modem_flash && modem_pm)
			? BBXMM_WORK_INIT_FLASHLESS_PM_STEP1
			: BBXMM_WORK_UNINIT;
		pr_debug("Go to next state %d\n", bbxmm_work->state);
		queue_work(workqueue, work);
		break;
	case BBXMM_WORK_INIT_FLASH_STEP1:
		pr_debug("BBXMM_WORK_INIT_FLASH_STEP1\n");
		/* register usb host controller */
		pr_debug("%s: register usb host controller\n", __func__);
		if (baseband_power_driver_data->hsic_register)
			baseband_power_driver_data->modem.xmm.hsic_device =
				baseband_power_driver_data->hsic_register();
		else
			pr_err("%s: hsic_register is missing\n", __func__);
		break;
	case BBXMM_WORK_INIT_FLASH_PM_STEP1:
		pr_debug("BBXMM_WORK_INIT_FLASH_PM_STEP1\n");
		/* [modem ver >= 1130] start with IPC_HSIC_ACTIVE low */
		if (modem_ver >= XMM_MODEM_VER_1130) {
			pr_debug("%s: ver > 1130:"
				" ipc_hsic_active -> 0\n", __func__);
			gpio_set_value(baseband_power_driver_data->
				modem.xmm.ipc_hsic_active, 0);
		}
		/* reset / power on sequence */
		baseband_xmm_power_reset_on();
		/* optional delay
		 * 0 = flashless
		 *   ==> causes next step to enumerate modem boot rom
		 *       (058b / 0041)
		 * some delay > boot rom timeout
		 *   ==> causes next step to enumerate modem software
		 *       (1519 / 0020)
		 *       (requires modem to be flash version, not flashless
		 *       version)
		 */
		if (enum_delay_ms)
			mdelay(enum_delay_ms);
		/* register usb host controller */
		pr_debug("%s: register usb host controller\n", __func__);
		if (baseband_power_driver_data->hsic_register)
			baseband_power_driver_data->modem.xmm.hsic_device =
				baseband_power_driver_data->hsic_register();
		else
			pr_err("%s: hsic_register is missing\n", __func__);
		/* go to next state */
		bbxmm_work->state = (modem_ver < XMM_MODEM_VER_1130)
			? BBXMM_WORK_INIT_FLASH_PM_VER_LT_1130_STEP1
			: BBXMM_WORK_INIT_FLASH_PM_VER_GE_1130_STEP1;
		queue_work(workqueue, work);
		pr_debug("Go to next state %d\n", bbxmm_work->state);
		break;
	case BBXMM_WORK_INIT_FLASH_PM_VER_LT_1130_STEP1:
		pr_debug("BBXMM_WORK_INIT_FLASH_PM_VER_LT_1130_STEP1\n");
		break;
	case BBXMM_WORK_INIT_FLASH_PM_VER_GE_1130_STEP1:
		pr_debug("BBXMM_WORK_INIT_FLASH_PM_VER_GE_1130_STEP1\n");
		break;
	case BBXMM_WORK_INIT_FLASHLESS_PM_STEP1:
		pr_debug("BBXMM_WORK_INIT_FLASHLESS_PM_STEP1\n");
		/* go to next state */
		bbxmm_work->state = (modem_ver < XMM_MODEM_VER_1130)
			? BBXMM_WORK_INIT_FLASHLESS_PM_VER_LT_1130_WAIT_IRQ
			: BBXMM_WORK_INIT_FLASHLESS_PM_VER_GE_1130_STEP1;
		queue_work(workqueue, work);
		break;
	case BBXMM_WORK_INIT_FLASHLESS_PM_VER_LT_1130_STEP1:
		pr_debug("BBXMM_WORK_INIT_FLASHLESS_PM_VER_LT_1130_STEP1\n");
		break;
	case BBXMM_WORK_INIT_FLASHLESS_PM_VER_GE_1130_STEP1:
		pr_debug("BBXMM_WORK_INIT_FLASHLESS_PM_VER_GE_1130_STEP1\n");
		break;
	default:
		break;
	}

}

static void baseband_xmm_device_add_handler(struct usb_device *udev)
{
	struct usb_interface *intf = usb_ifnum_to_if(udev, 0);
	const struct usb_device_id *id = usb_match_id(intf, xmm_pm_ids);

	if (id) {
		pr_debug("persist_enabled: %u\n", udev->persist_enabled);
		pr_info("Add device %d <%s %s>\n", udev->devnum,
			udev->manufacturer, udev->product);
		usbdev = udev;
		usb_enable_autosuspend(udev);
		pr_info("enable autosuspend\n");
	}
}

static void baseband_xmm_device_remove_handler(struct usb_device *udev)
{
	if (usbdev == udev) {
		pr_info("Remove device %d <%s %s>\n", udev->devnum,
			udev->manufacturer, udev->product);
		usbdev = 0;
	}

}

static int usb_xmm_notify(struct notifier_block *self, unsigned long action,
			void *blob)
{
	switch (action) {
	case USB_DEVICE_ADD:
		baseband_xmm_device_add_handler(blob);
		break;
	case USB_DEVICE_REMOVE:
		baseband_xmm_device_remove_handler(blob);
		break;
	}

	return NOTIFY_OK;
}


static struct notifier_block usb_xmm_nb = {
	.notifier_call = usb_xmm_notify,
};

static int baseband_xmm_power_driver_probe(struct platform_device *device)
{
	struct baseband_power_platform_data *data
		= (struct baseband_power_platform_data *)
			device->dev.platform_data;
	struct device *dev = &device->dev;
	int err;

	pr_debug("%s\n", __func__);
	pr_debug("[XMM] enum_delay_ms=%ld\n", enum_delay_ms);

	/* check for platform data */
	if (!data)
		return -ENODEV;

	/* check if supported modem */
	if (data->baseband_type != BASEBAND_XMM) {
		pr_err("unsuppported modem\n");
		return -ENODEV;
	}

	/* save platform data */
	baseband_power_driver_data = data;

	/* create device file */
	err = device_create_file(dev, &dev_attr_xmm_onoff);
	if (err < 0) {
		pr_err("%s - device_create_file failed\n", __func__);
		return -ENODEV;
	}

	/* init wake lock */
	wake_lock_init(&wakelock, WAKE_LOCK_SUSPEND, "baseband_xmm_power");

	/* request baseband gpio(s) */
	tegra_baseband_gpios[0].gpio = baseband_power_driver_data
		->modem.xmm.bb_rst;
	tegra_baseband_gpios[1].gpio = baseband_power_driver_data
		->modem.xmm.bb_on;
	tegra_baseband_gpios[2].gpio = baseband_power_driver_data
		->modem.xmm.ipc_bb_wake;
	tegra_baseband_gpios[3].gpio = baseband_power_driver_data
		->modem.xmm.ipc_ap_wake;
	tegra_baseband_gpios[4].gpio = baseband_power_driver_data
		->modem.xmm.ipc_hsic_active;
	tegra_baseband_gpios[5].gpio = baseband_power_driver_data
		->modem.xmm.ipc_hsic_sus_req;
	err = gpio_request_array(tegra_baseband_gpios,
		ARRAY_SIZE(tegra_baseband_gpios));
	if (err < 0) {
		pr_err("%s - request gpio(s) failed\n", __func__);
		return -ENODEV;
	}

	/* request baseband irq(s) */
	if (modem_flash && modem_pm) {
		pr_debug("%s: request_irq IPC_AP_WAKE_IRQ\n", __func__);
		ipc_ap_wake_state = IPC_AP_WAKE_UNINIT;
		err = request_irq(gpio_to_irq(data->modem.xmm.ipc_ap_wake),
			baseband_xmm_power_ipc_ap_wake_irq,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			"IPC_AP_WAKE_IRQ",
			NULL);
		if (err < 0) {
			pr_err("%s - request irq IPC_AP_WAKE_IRQ failed\n",
				__func__);
			return err;
		}
		ipc_ap_wake_state = IPC_AP_WAKE_IRQ_READY;
		if (modem_ver >= XMM_MODEM_VER_1130) {
			pr_debug("%s: ver > 1130: AP_WAKE_INIT1\n", __func__);
			/* ver 1130 or later starts in INIT1 state */
			ipc_ap_wake_state = IPC_AP_WAKE_INIT1;
		}
	}

	/* init work queue */
	workqueue = create_singlethread_workqueue
		("baseband_xmm_power_workqueue");
	if (!workqueue) {
		pr_err("cannot create workqueue\n");
		return -1;
	}
	baseband_xmm_power_work = (struct baseband_xmm_power_work_t *)
		kmalloc(sizeof(struct baseband_xmm_power_work_t), GFP_KERNEL);
	if (!baseband_xmm_power_work) {
		pr_err("cannot allocate baseband_xmm_power_work\n");
		return -1;
	}
	INIT_WORK((struct work_struct *) baseband_xmm_power_work,
		baseband_xmm_power_work_func);
	baseband_xmm_power_work->state = BBXMM_WORK_INIT;
	queue_work(workqueue,
		(struct work_struct *) baseband_xmm_power_work);

	/* init work objects */
	INIT_WORK(&init1_work, baseband_xmm_power_init1_work);
	INIT_WORK(&init2_work, baseband_xmm_power_init2_work);
	INIT_WORK(&L2_resume_work, baseband_xmm_power_L2_resume_work);

	/* init state variables */
	register_hsic_device = true;
	baseband_xmm_powerstate = BBXMM_PS_UNINIT;
	CP_initiated_L2toL0 = false;

	usb_register_notify(&usb_xmm_nb);

	pr_debug("%s }\n", __func__);
	return 0;
}

static int baseband_xmm_power_driver_remove(struct platform_device *device)
{
	struct baseband_power_platform_data *data
		= (struct baseband_power_platform_data *)
			device->dev.platform_data;
	struct device *dev = &device->dev;

	pr_debug("%s\n", __func__);

	/* check for platform data */
	if (!data)
		return 0;

	usb_unregister_notify(&usb_xmm_nb);

	/* free work structure */
	kfree(baseband_xmm_power_work);
	baseband_xmm_power_work = (struct baseband_xmm_power_work_t *) 0;

	/* free baseband irq(s) */
	if (modem_flash && modem_pm) {
		free_irq(gpio_to_irq(baseband_power_driver_data
			->modem.xmm.ipc_ap_wake), NULL);
	}

	/* free baseband gpio(s) */
	gpio_free_array(tegra_baseband_gpios,
		ARRAY_SIZE(tegra_baseband_gpios));

	/* destroy wake lock */
	wake_lock_destroy(&wakelock);

	/* delete device file */
	device_remove_file(dev, &dev_attr_xmm_onoff);

	/* unregister usb host controller */
	if (data->hsic_unregister)
		data->hsic_unregister(data->modem.xmm.hsic_device);
	else
		pr_err("%s: hsic_unregister is missing\n", __func__);

	return 0;
}

#ifdef CONFIG_PM
static int baseband_xmm_power_driver_suspend(struct platform_device *device,
	pm_message_t state)
{
	pr_debug("%s\n", __func__);
	return 0;
}

static int baseband_xmm_power_driver_resume(struct platform_device *device)
{
	struct baseband_power_platform_data *data
		= (struct baseband_power_platform_data *)
			device->dev.platform_data;
	int value;
	int delay = 10000; /* maxmum delay in msec */

	pr_debug("%s\n", __func__);

	/* check for platform data */
	if (!baseband_power_driver_data)
		return 0;

	/* check if modem is on */
	if (power_onoff == 0) {
		pr_debug("%s - flight mode - nop\n", __func__);
		baseband_xmm_set_power_status(BBXMM_PS_L3TOL0);
		return 0;
	}

	/* L3->L0 */
	baseband_xmm_set_power_status(BBXMM_PS_L3TOL0);
	value = gpio_get_value(data->modem.xmm.ipc_ap_wake);
	if (value) {
		pr_info("AP L3 -> L0\n");
		/* wake bb */
		gpio_set_value(data->modem.xmm.ipc_bb_wake, 1);

		pr_debug("waiting for host wakeup...\n");
		do {
			mdelay(1);
			value = gpio_get_value(data->modem.xmm.ipc_ap_wake);
			delay--;
		} while ((value) && (delay));
		if (delay)
			pr_debug("gpio host wakeup low <-\n");
	} else {
		pr_info("CP L3 -> L0\n");
	}

	return 0;
}
#endif

static struct platform_driver baseband_power_driver = {
	.probe = baseband_xmm_power_driver_probe,
	.remove = baseband_xmm_power_driver_remove,
#ifdef CONFIG_PM
	.suspend = baseband_xmm_power_driver_suspend,
	.resume = baseband_xmm_power_driver_resume,
#endif
	.driver = {
		.name = "baseband_xmm_power",
	},
};

static int __init baseband_xmm_power_init(void)
{
	return platform_driver_register(&baseband_power_driver);
}

static void __exit baseband_xmm_power_exit(void)
{
	pr_debug("%s\n", __func__);
	platform_driver_unregister(&baseband_power_driver);
}

module_init(baseband_xmm_power_init)
module_exit(baseband_xmm_power_exit)
