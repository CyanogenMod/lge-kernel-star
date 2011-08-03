/*
 * arch/arm/mach-tegra/baseband-xmm-power2.c
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
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <mach/usb_phy.h>
#include "baseband-xmm-power.h"
#include "board.h"
#include "devices.h"

MODULE_LICENSE("GPL");

static struct baseband_power_platform_data *baseband_power2_driver_data;

static enum {
	IPC_HSIC_SUS_REQ_UNINIT,
	IPC_HSIC_SUS_REQ_IRQ_READY,
	IPC_HSIC_SUS_REQ_INIT,
	IPC_HSIC_SUS_REQ_L,
	IPC_HSIC_SUS_REQ_H,
} ipc_hsic_sus_req_state;

static enum {
	IPC_AP_WAKE_UNINIT,
	IPC_AP_WAKE_IRQ_READY,
	IPC_AP_WAKE_INIT1,
	IPC_AP_WAKE_INIT2,
	IPC_AP_WAKE_L,
	IPC_AP_WAKE_H,
} ipc_ap_wake_state;

static struct workqueue_struct *workqueue;
static struct work_struct init1_work;
static struct work_struct init2_work;

static irqreturn_t ipc_hsic_sus_req_irq(int irq, void *dev_id)
{
	int value;

	pr_debug("%s\n", __func__);

	/* check for platform data */
	if (!baseband_power2_driver_data)
		return IRQ_HANDLED;

	/* IPC_HSIC_SUS_REQ state machine */
	if (ipc_hsic_sus_req_state < IPC_HSIC_SUS_REQ_IRQ_READY) {
		pr_err("%s - spurious irq\n", __func__);
	} else {
		value = gpio_get_value(baseband_power2_driver_data->
			modem.xmm.ipc_hsic_sus_req);
		if (!value) {
			pr_debug("%s - falling\n", __func__);
			ipc_hsic_sus_req_state = IPC_HSIC_SUS_REQ_L;
		} else {
			pr_debug("%s - rising\n", __func__);
			ipc_hsic_sus_req_state = IPC_HSIC_SUS_REQ_H;
		}
	}

	return IRQ_HANDLED;
}

static irqreturn_t ipc_ap_wake_irq(int irq, void *dev_id)
{
	int value;

	pr_debug("%s\n", __func__);

	/* check for platform data */
	if (!baseband_power2_driver_data)
		return IRQ_HANDLED;

	/* IPC_AP_WAKE state machine */
	if (ipc_ap_wake_state < IPC_AP_WAKE_IRQ_READY) {
		pr_err("%s - spurious irq\n", __func__);
	} else if (ipc_ap_wake_state == IPC_AP_WAKE_IRQ_READY) {
		value = gpio_get_value(baseband_power2_driver_data->
			modem.xmm.ipc_ap_wake);
		if (!value) {
			pr_debug("%s - IPC_AP_WAKE_INIT1 - got falling edge\n",
				__func__);
			/* go to IPC_AP_WAKE_INIT1 state */
			ipc_ap_wake_state = IPC_AP_WAKE_INIT1;
			/* queue work */
			queue_work(workqueue, &init1_work);
		} else {
			pr_debug("%s - IPC_AP_WAKE_INIT1 - wait for falling edge\n",
				__func__);
		}
	} else if (ipc_ap_wake_state == IPC_AP_WAKE_INIT1) {
		value = gpio_get_value(baseband_power2_driver_data->
			modem.xmm.ipc_ap_wake);
		if (!value) {
			pr_debug("%s - IPC_AP_WAKE_INIT2 - wait for rising edge\n",
				__func__);
		} else {
			pr_debug("%s - IPC_AP_WAKE_INIT2 - got rising edge\n",
				__func__);
			/* go to IPC_AP_WAKE_INIT2 state */
			ipc_ap_wake_state = IPC_AP_WAKE_INIT2;
			/* queue work */
			queue_work(workqueue, &init2_work);
		}
	} else {
		value = gpio_get_value(baseband_power2_driver_data->
			modem.xmm.ipc_ap_wake);
		if (!value) {
			pr_debug("%s - falling\n", __func__);
			ipc_ap_wake_state = IPC_AP_WAKE_L;
		} else {
			pr_debug("%s - rising\n", __func__);
			ipc_ap_wake_state = IPC_AP_WAKE_H;
		}
	}

	return IRQ_HANDLED;
}

static void baseband_xmm_power2_init1_work(struct work_struct *work)
{
	int value;

	pr_debug("%s {\n", __func__);

	/* check for platform data */
	if (!baseband_power2_driver_data)
		return;

	/* check if IPC_HSIC_ACTIVE high */
	value = gpio_get_value(baseband_power2_driver_data->
		modem.xmm.ipc_hsic_active);
	if (value != 1) {
		pr_err("%s - expected IPC_HSIC_ACTIVE high!\n", __func__);
		return;
	}

	/* wait 30 ms */
	mdelay(30);

	/* set IPC_HSIC_ACTIVE low */
	gpio_set_value(baseband_power2_driver_data->
		modem.xmm.ipc_hsic_active, 0);

	pr_debug("%s }\n", __func__);
}

static void baseband_xmm_power2_init2_work(struct work_struct *work)
{
	int value;

	pr_debug("%s {\n", __func__);

	/* check for platform data */
	if (!baseband_power2_driver_data)
		return;

	/* check if IPC_HSIC_ACTIVE low */
	value = gpio_get_value(baseband_power2_driver_data->
		modem.xmm.ipc_hsic_active);
	if (value != 0) {
		pr_err("%s - expected IPC_HSIC_ACTIVE low!\n", __func__);
		return;
	}

	/* wait 1 ms */
	mdelay(1);

	/* turn on usb host controller */
	{
		mm_segment_t oldfs;
		struct file *filp;
		oldfs = get_fs();
		set_fs(KERNEL_DS);
		filp = filp_open("/sys/devices/platform/tegra-ehci.1/ehci_power", O_RDWR, 0);
		if (!filp) {
			pr_err("open ehci_power failed\n");
		} else {
			filp->f_op->write(filp, "1", 1, &filp->f_pos);
			filp_close(filp, NULL);
		}
		set_fs(oldfs);
	}

	/* set IPC_HSIC_ACTIVE high */
	gpio_set_value(baseband_power2_driver_data->
		modem.xmm.ipc_hsic_active, 1);

	/* wait 20 ms */
	mdelay(20);

	/* set IPC_HSIC_ACTIVE low */
	gpio_set_value(baseband_power2_driver_data->
		modem.xmm.ipc_hsic_active, 0);

	/* wait 20 ms */
	mdelay(20);

	/* set IPC_HSIC_ACTIVE high */
	gpio_set_value(baseband_power2_driver_data->
		modem.xmm.ipc_hsic_active, 1);

	pr_debug("%s }\n", __func__);
}

static int baseband_xmm_power2_driver_probe(struct platform_device *device)
{
	struct baseband_power_platform_data *data
		= (struct baseband_power_platform_data *) device->dev.platform_data;
	int err;

	pr_debug("%s\n", __func__);

	/* save platform data */
	baseband_power2_driver_data = data;

	/* request baseband irq(s) */
	ipc_hsic_sus_req_state = IPC_HSIC_SUS_REQ_UNINIT;
	err = request_irq(gpio_to_irq(data->modem.xmm.ipc_hsic_sus_req),
		ipc_hsic_sus_req_irq,
		IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
		"IPC_HSIC_SUS_REQ_IRQ",
		NULL);
	if (err < 0) {
		pr_err("%s - request irq IPC_HSIC_SUS_REQ_IRQ failed\n",
			__func__);
		return err;
	}
	ipc_hsic_sus_req_state = IPC_HSIC_SUS_REQ_IRQ_READY;
	ipc_ap_wake_state = IPC_AP_WAKE_UNINIT;
	err = request_irq(gpio_to_irq(data->modem.xmm.ipc_ap_wake),
		ipc_ap_wake_irq,
		IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
		"IPC_AP_WAKE_IRQ",
		NULL);
	if (err < 0) {
		pr_err("%s - request irq IPC_AP_WAKE_IRQ failed\n",
			__func__);
		return err;
	}
	ipc_ap_wake_state = IPC_AP_WAKE_IRQ_READY;

	/* init work queue */
	workqueue = create_singlethread_workqueue
		("baseband_power_workqueue");
	if (!workqueue) {
		pr_err("cannot create workqueue\n");
		return -1;
	}
	INIT_WORK(&init1_work, baseband_xmm_power2_init1_work);
	INIT_WORK(&init2_work, baseband_xmm_power2_init2_work);

	return 0;
}

static int baseband_xmm_power2_driver_remove(struct platform_device *device)
{
	pr_debug("%s\n", __func__);

	/* check for platform data */
	if (!baseband_power2_driver_data)
		return 0;

	/* free baseband irq(s) */
	free_irq(gpio_to_irq(baseband_power2_driver_data
		->modem.xmm.ipc_ap_wake), NULL);
	free_irq(gpio_to_irq(baseband_power2_driver_data
		->modem.xmm.ipc_hsic_sus_req), NULL);

	return 0;
}

#ifdef CONFIG_PM
static int baseband_xmm_power2_driver_suspend(struct platform_device *device,
	pm_message_t state)
{
	pr_debug("%s\n", __func__);

	/* check for platform data */
	if (!baseband_power2_driver_data)
		return 0;

	/* signal bb to suspend hsic */
	gpio_set_value(baseband_power2_driver_data
		->modem.xmm.ipc_hsic_active, 0);

	return 0;
}

static int baseband_xmm_power2_driver_resume(struct platform_device *device)
{
	pr_debug("%s\n", __func__);

	/* check for platform data */
	if (!baseband_power2_driver_data)
		return 0;

	/* wake bb */
	gpio_set_value(baseband_power2_driver_data
		->modem.xmm.ipc_bb_wake, 1);

	/* signal bb to resume hsic */
	gpio_set_value(baseband_power2_driver_data
		->modem.xmm.ipc_hsic_active, 1);

	return 0;
}
#endif

static struct platform_driver baseband_power2_driver = {
	.probe = baseband_xmm_power2_driver_probe,
	.remove = baseband_xmm_power2_driver_remove,
#ifdef CONFIG_PM
	.suspend = baseband_xmm_power2_driver_suspend,
	.resume = baseband_xmm_power2_driver_resume,
#endif
	.driver = {
		.name = "baseband_xmm_power2",
	},
};

static int __init baseband_xmm_power2_init(void)
{
	pr_debug("%s\n", __func__);
	return platform_driver_register(&baseband_power2_driver);
}

static int __exit baseband_xmm_power2_exit(void)
{
	pr_debug("%s\n", __func__);
	platform_driver_unregister(&baseband_power2_driver);
	return 0;
}

module_init(baseband_xmm_power2_init)
module_exit(baseband_xmm_power2_exit)

