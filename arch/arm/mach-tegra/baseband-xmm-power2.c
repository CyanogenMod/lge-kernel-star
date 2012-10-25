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
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/wakelock.h>
#include <mach/usb_phy.h>
#include "baseband-xmm-power.h"
#include "board.h"
#include "devices.h"

MODULE_LICENSE("GPL");

static unsigned long XYZ = 1000 * 1000000 + 800 * 1000 + 500;

module_param(modem_ver, ulong, 0644);
MODULE_PARM_DESC(modem_ver,
	"baseband xmm power2 - modem software version");
module_param(modem_flash, ulong, 0644);
MODULE_PARM_DESC(modem_flash,
	"baseband xmm power2 - modem flash (1 = flash, 0 = flashless)");
module_param(modem_pm, ulong, 0644);
MODULE_PARM_DESC(modem_pm,
	"baseband xmm power2 - modem power management (1 = pm, 0 = no pm)");
module_param(XYZ, ulong, 0644);
MODULE_PARM_DESC(XYZ,
	"baseband xmm power2 - timing parameters X/Y/Z delay in ms");

static struct baseband_power_platform_data *baseband_power2_driver_data;
static struct workqueue_struct *workqueue;
static struct baseband_xmm_power_work_t *baseband_xmm_power2_work;

static enum {
	IPC_AP_WAKE_UNINIT,
	IPC_AP_WAKE_IRQ_READY,
	IPC_AP_WAKE_INIT1,
	IPC_AP_WAKE_INIT2,
	IPC_AP_WAKE_L,
	IPC_AP_WAKE_H,
} ipc_ap_wake_state;

static irqreturn_t baseband_xmm_power2_ver_lt_1130_ipc_ap_wake_irq2
	(int irq, void *dev_id)
{
	int value;

	pr_debug("%s\n", __func__);

	/* check for platform data */
	if (!baseband_power2_driver_data)
		return IRQ_HANDLED;

	value = gpio_get_value(baseband_power2_driver_data->
		    modem.xmm.ipc_ap_wake);

	/* IPC_AP_WAKE state machine */
	if (ipc_ap_wake_state < IPC_AP_WAKE_IRQ_READY) {
		pr_err("%s - spurious irq\n", __func__);
	} else if (ipc_ap_wake_state == IPC_AP_WAKE_IRQ_READY) {
		if (!value) {
			pr_debug("%s - IPC_AP_WAKE_INIT1"
				" - got falling edge\n",
				__func__);
			/* go to IPC_AP_WAKE_INIT1 state */
			ipc_ap_wake_state = IPC_AP_WAKE_INIT1;
			/* queue work */
			baseband_xmm_power2_work->state =
				BBXMM_WORK_INIT_FLASHLESS_PM_VER_LT_1130_STEP1;
			queue_work(workqueue, (struct work_struct *)
				baseband_xmm_power2_work);
		} else {
			pr_debug("%s - IPC_AP_WAKE_INIT1"
				" - wait for falling edge\n",
				__func__);
		}
	} else if (ipc_ap_wake_state == IPC_AP_WAKE_INIT1) {
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
			baseband_xmm_power2_work->state =
				BBXMM_WORK_INIT_FLASHLESS_PM_VER_LT_1130_STEP2;
			queue_work(workqueue, (struct work_struct *)
				baseband_xmm_power2_work);
		}
	} else {
		if (!value) {
			pr_debug("%s - falling\n", __func__);
			ipc_ap_wake_state = IPC_AP_WAKE_L;
		} else {
			pr_debug("%s - rising\n", __func__);
			ipc_ap_wake_state = IPC_AP_WAKE_H;
		}
		return baseband_xmm_power_ipc_ap_wake_irq(irq, dev_id);
	}

	return IRQ_HANDLED;
}

static irqreturn_t baseband_xmm_power2_ver_ge_1130_ipc_ap_wake_irq2
	(int irq, void *dev_id)
{
	int value;

	pr_debug("%s\n", __func__);

	/* check for platform data */
	if (!baseband_power2_driver_data)
		return IRQ_HANDLED;

	value = gpio_get_value(baseband_power2_driver_data->
		    modem.xmm.ipc_ap_wake);

	/* IPC_AP_WAKE state machine */
	if (ipc_ap_wake_state < IPC_AP_WAKE_IRQ_READY) {
		pr_err("%s - spurious irq\n", __func__);
	} else if (ipc_ap_wake_state == IPC_AP_WAKE_IRQ_READY) {
		if (!value) {
			pr_debug("%s - IPC_AP_WAKE_INIT1"
				" - got falling edge\n",
				__func__);
			/* go to IPC_AP_WAKE_INIT2 state */
			ipc_ap_wake_state = IPC_AP_WAKE_INIT2;
			/* queue work */
			baseband_xmm_power2_work->state =
				BBXMM_WORK_INIT_FLASHLESS_PM_VER_GE_1130_STEP2;
			queue_work(workqueue, (struct work_struct *)
				baseband_xmm_power2_work);
		} else {
			pr_debug("%s - IPC_AP_WAKE_INIT1"
				" - wait for falling edge\n",
				__func__);
		}
	} else {
		if (!value) {
			pr_debug("%s - falling\n", __func__);
			ipc_ap_wake_state = IPC_AP_WAKE_L;
		} else {
			pr_debug("%s - rising\n", __func__);
			ipc_ap_wake_state = IPC_AP_WAKE_H;
		}
		return baseband_xmm_power_ipc_ap_wake_irq(irq, dev_id);
	}

	return IRQ_HANDLED;
}

static void baseband_xmm_power2_flashless_pm_ver_lt_1130_step1
	(struct work_struct *work)
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

static void baseband_xmm_power2_flashless_pm_ver_lt_1130_step2
	(struct work_struct *work)
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

	/* unregister usb host controller */
	if (baseband_power2_driver_data->hsic_unregister)
		baseband_power2_driver_data->hsic_unregister(
			baseband_power2_driver_data->modem.xmm.hsic_device);
	else
		pr_err("%s: hsic_unregister is missing\n", __func__);

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

static void baseband_xmm_power2_flashless_pm_ver_ge_1130_step1
	(struct work_struct *work)
{
	int X = XYZ / 1000000;
	int Y = XYZ / 1000 - X * 1000;
	int Z = XYZ % 1000;

	pr_info("%s {\n", __func__);

	pr_info("XYZ=%ld X=%d Y=%d Z=%d\n", XYZ, X, Y, Z);

	/* check for platform data */
	if (!baseband_power2_driver_data)
		return;

	/* unregister usb host controller */
	if (baseband_power2_driver_data->hsic_unregister)
		baseband_power2_driver_data->hsic_unregister(
			baseband_power2_driver_data->modem.xmm.hsic_device);
	else
		pr_err("%s: hsic_unregister is missing\n", __func__);

	/* wait X ms */
	mdelay(X);

	/* set IPC_HSIC_ACTIVE low */
	gpio_set_value(baseband_power2_driver_data->
		modem.xmm.ipc_hsic_active, 0);

	pr_info("%s }\n", __func__);
}

static void baseband_xmm_power2_flashless_pm_ver_ge_1130_step2
	(struct work_struct *work)
{
	int X = XYZ / 1000000;
	int Y = XYZ / 1000 - X * 1000;
	int Z = XYZ % 1000;

	pr_info("%s {\n", __func__);

	pr_info("XYZ=%ld X=%d Y=%d Z=%d\n", XYZ, X, Y, Z);

	/* check for platform data */
	if (!baseband_power2_driver_data)
		return;

	/* wait Y ms */
	mdelay(Y);

	/* register usb host controller */
	if (baseband_power2_driver_data->hsic_register)
		baseband_power2_driver_data->modem.xmm.hsic_device =
			baseband_power2_driver_data->hsic_register();
	else
		pr_err("%s: hsic_register is missing\n", __func__);

	/* wait Z ms */
	mdelay(Z);

	/* set IPC_HSIC_ACTIVE high */
	gpio_set_value(baseband_power2_driver_data->
		modem.xmm.ipc_hsic_active, 1);

	/* queue work function to check if enumeration succeeded */
	baseband_xmm_power2_work->state =
		BBXMM_WORK_INIT_FLASHLESS_PM_VER_GE_1130_STEP3;
	queue_work(workqueue, (struct work_struct *)
		baseband_xmm_power2_work);

	pr_info("%s }\n", __func__);
}

static void baseband_xmm_power2_flashless_pm_ver_ge_1130_step3
	(struct work_struct *work)
{
	int X = XYZ / 1000000;
	int Y = XYZ / 1000 - X * 1000;
	int Z = XYZ % 1000;
	int enum_success = 0;

	pr_info("%s {\n", __func__);

	pr_info("XYZ=%ld X=%d Y=%d Z=%d\n", XYZ, X, Y, Z);

	/* check for platform data */
	if (!baseband_power2_driver_data)
		return;

	/* wait 500 ms */
	mdelay(500);

	/* check if enumeration succeeded */
	{
		mm_segment_t oldfs;
		struct file *filp;
		oldfs = get_fs();
		set_fs(KERNEL_DS);
		filp = filp_open("/dev/ttyACM0",
			O_RDONLY, 0);
		if (IS_ERR(filp) || (filp == NULL)) {
			pr_err("/dev/ttyACM0 %ld\n",
				PTR_ERR(filp));
		} else {
			filp_close(filp, NULL);
			enum_success = 1;
		}
		set_fs(oldfs);
	}

	/* if enumeration failed, attempt recovery pulse */
	if (!enum_success) {
		pr_info("attempting recovery pulse...\n");
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
		/* check if recovery pulse worked */
		baseband_xmm_power2_work->state =
			BBXMM_WORK_INIT_FLASHLESS_PM_VER_GE_1130_STEP4;
		queue_work(workqueue, (struct work_struct *)
			baseband_xmm_power2_work);
	}

	pr_info("%s }\n", __func__);
}

static void baseband_xmm_power2_flashless_pm_ver_ge_1130_step4
	(struct work_struct *work)
{
	int X = XYZ / 1000000;
	int Y = XYZ / 1000 - X * 1000;
	int Z = XYZ % 1000;
	int enum_success = 0;

	pr_info("%s {\n", __func__);

	pr_info("XYZ=%ld X=%d Y=%d Z=%d\n", XYZ, X, Y, Z);

	/* check for platform data */
	if (!baseband_power2_driver_data)
		return;

	/* wait 500 ms */
	mdelay(500);

	/* check if enumeration succeeded */
	{
		mm_segment_t oldfs;
		struct file *filp;
		oldfs = get_fs();
		set_fs(KERNEL_DS);
		filp = filp_open("/dev/ttyACM0",
			O_RDONLY, 0);
		if (IS_ERR(filp) || (filp == NULL)) {
			pr_err("open /dev/ttyACM0 failed %ld\n",
				PTR_ERR(filp));
		} else {
			filp_close(filp, NULL);
			enum_success = 1;
		}
		set_fs(oldfs);
	}

	/* if recovery pulse did not fix enumeration, retry from beginning */
	if (!enum_success) {
		static int retry = 3;
		if (!retry) {
			pr_info("failed to enumerate modem software"
				" - too many retry attempts\n");
		} else {
			pr_info("recovery pulse failed to fix modem"
				" enumeration..."
				" restarting from beginning"
				" - attempt #%d\n",
				retry);
			--retry;
			ipc_ap_wake_state = IPC_AP_WAKE_IRQ_READY;
			baseband_xmm_power2_work->state =
				BBXMM_WORK_INIT_FLASHLESS_PM_VER_GE_1130_STEP1;
			queue_work(workqueue, (struct work_struct *)
				baseband_xmm_power2_work);
		}
	}

	pr_info("%s }\n", __func__);
}

static int free_ipc_ap_wake_irq;

static void baseband_xmm_power2_work_func(struct work_struct *work)
{
	struct baseband_xmm_power_work_t *bbxmm_work
		= (struct baseband_xmm_power_work_t *) work;
	int err;

	pr_debug("%s bbxmm_work->state=%d\n", __func__, bbxmm_work->state);

	switch (bbxmm_work->state) {
	case BBXMM_WORK_UNINIT:
		pr_debug("BBXMM_WORK_UNINIT\n");
		/* free baseband irq(s) */
		if (free_ipc_ap_wake_irq) {
			free_irq(gpio_to_irq(baseband_power2_driver_data
				->modem.xmm.ipc_ap_wake), NULL);
			free_ipc_ap_wake_irq = 0;
		}
		break;
	case BBXMM_WORK_INIT:
		pr_debug("BBXMM_WORK_INIT\n");
		/* request baseband irq(s) */
		ipc_ap_wake_state = IPC_AP_WAKE_UNINIT;
		err = request_threaded_irq(
			gpio_to_irq(baseband_power2_driver_data->
			    modem.xmm.ipc_ap_wake),
			NULL,
			(modem_ver < XMM_MODEM_VER_1130)
			? baseband_xmm_power2_ver_lt_1130_ipc_ap_wake_irq2
			: baseband_xmm_power2_ver_ge_1130_ipc_ap_wake_irq2,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			"BBXMM_POWER2_IPC_AP_WAKE_IRQ",
			NULL);
		if (err < 0) {
			pr_err("%s - request irq IPC_AP_WAKE_IRQ failed\n",
				__func__);
			return;
		}
		free_ipc_ap_wake_irq = 1;
		ipc_ap_wake_state = IPC_AP_WAKE_IRQ_READY;
		/* go to next state */
		bbxmm_work->state = (modem_flash && !modem_pm)
			? BBXMM_WORK_INIT_FLASH_STEP1
			: (modem_flash && modem_pm)
			? BBXMM_WORK_INIT_FLASH_PM_STEP1
			: (!modem_flash && modem_pm)
			? BBXMM_WORK_INIT_FLASHLESS_PM_STEP1
			: BBXMM_WORK_UNINIT;
		queue_work(workqueue, work);
		break;
	case BBXMM_WORK_INIT_FLASH_STEP1:
		pr_debug("BBXMM_WORK_INIT_FLASH_STEP1\n");
		break;
	case BBXMM_WORK_INIT_FLASH_PM_STEP1:
		pr_debug("BBXMM_WORK_INIT_FLASH_PM_STEP1\n");
		/* go to next state */
		bbxmm_work->state = (modem_ver < XMM_MODEM_VER_1130)
			? BBXMM_WORK_INIT_FLASH_PM_VER_LT_1130_STEP1
			: BBXMM_WORK_INIT_FLASH_PM_VER_GE_1130_STEP1;
		queue_work(workqueue, work);
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
	case BBXMM_WORK_INIT_FLASHLESS_PM_VER_LT_1130_WAIT_IRQ:
		pr_debug("BBXMM_WORK_INIT_FLASHLESS_PM_VER_LT_1130_WAIT_IRQ"
			" - waiting for IPC_AP_WAKE_IRQ to trigger step1\n");
		break;
	case BBXMM_WORK_INIT_FLASHLESS_PM_VER_LT_1130_STEP1:
		pr_debug("BBXMM_WORK_INIT_FLASHLESS_PM_VER_LT_1130_STEP1\n");
		baseband_xmm_power2_flashless_pm_ver_lt_1130_step1(work);
		break;
	case BBXMM_WORK_INIT_FLASHLESS_PM_VER_LT_1130_STEP2:
		pr_debug("BBXMM_WORK_INIT_FLASHLESS_PM_VER_LT_1130_STEP2\n");
		baseband_xmm_power2_flashless_pm_ver_lt_1130_step2(work);
		break;
	case BBXMM_WORK_INIT_FLASHLESS_PM_VER_GE_1130_STEP1:
		pr_debug("BBXMM_WORK_INIT_FLASHLESS_PM_VER_GE_1130_STEP1\n");
		baseband_xmm_power2_flashless_pm_ver_ge_1130_step1(work);
		break;
	case BBXMM_WORK_INIT_FLASHLESS_PM_VER_GE_1130_STEP2:
		pr_debug("BBXMM_WORK_INIT_FLASHLESS_PM_VER_GE_1130_STEP2\n");
		baseband_xmm_power2_flashless_pm_ver_ge_1130_step2(work);
		break;
	case BBXMM_WORK_INIT_FLASHLESS_PM_VER_GE_1130_STEP3:
		pr_debug("BBXMM_WORK_INIT_FLASHLESS_PM_VER_GE_1130_STEP3\n");
		baseband_xmm_power2_flashless_pm_ver_ge_1130_step3(work);
		break;
	case BBXMM_WORK_INIT_FLASHLESS_PM_VER_GE_1130_STEP4:
		pr_debug("BBXMM_WORK_INIT_FLASHLESS_PM_VER_GE_1130_STEP4\n");
		baseband_xmm_power2_flashless_pm_ver_ge_1130_step4(work);
		break;
	}

}

static int baseband_xmm_power2_driver_probe(struct platform_device *device)
{
	struct baseband_power_platform_data *data
		= (struct baseband_power_platform_data *)
			device->dev.platform_data;

	pr_debug("%s\n", __func__);

	/* save platform data */
	baseband_power2_driver_data = data;

	/* init work queue */
	pr_debug("%s: init work queue\n", __func__);
	workqueue = create_singlethread_workqueue
		("baseband_xmm_power2_workqueue");
	if (!workqueue) {
		pr_err("cannot create workqueue\n");
		return -1;
	}
	baseband_xmm_power2_work = (struct baseband_xmm_power_work_t *)
		kmalloc(sizeof(struct baseband_xmm_power_work_t), GFP_KERNEL);
	if (!baseband_xmm_power2_work) {
		pr_err("cannot allocate baseband_xmm_power2_work\n");
		return -1;
	}
	pr_debug("%s: BBXMM_WORK_INIT\n", __func__);
	INIT_WORK((struct work_struct *) baseband_xmm_power2_work,
		baseband_xmm_power2_work_func);
	baseband_xmm_power2_work->state = BBXMM_WORK_INIT;
	queue_work(workqueue,
		(struct work_struct *) baseband_xmm_power2_work);
	return 0;
}

static int baseband_xmm_power2_driver_remove(struct platform_device *device)
{
	struct baseband_power_platform_data *data
		= (struct baseband_power_platform_data *)
			device->dev.platform_data;

	pr_debug("%s\n", __func__);

	/* check for platform data */
	if (!data)
		return 0;

	/* free irq */
	if (free_ipc_ap_wake_irq) {
		free_irq(gpio_to_irq(data->modem.xmm.ipc_ap_wake), NULL);
		free_ipc_ap_wake_irq = 0;
	}

	/* free work structure */
	if (workqueue) {
		cancel_work_sync(baseband_xmm_power2_work);
		destroy_workqueue(workqueue);
	}
	kfree(baseband_xmm_power2_work);
	baseband_xmm_power2_work = (struct baseband_xmm_power_work_t *) 0;

	return 0;
}

#ifdef CONFIG_PM
static int baseband_xmm_power2_driver_suspend(struct platform_device *device,
	pm_message_t state)
{
	struct baseband_power_platform_data *data
		= (struct baseband_power_platform_data *)
			device->dev.platform_data;

	pr_debug("%s - nop\n", __func__);

	/* check for platform data */
	if (!data)
		return 0;

	return 0;
}

static int baseband_xmm_power2_driver_resume(struct platform_device *device)
{
	struct baseband_power_platform_data *data
		= (struct baseband_power_platform_data *)
			device->dev.platform_data;

	pr_debug("%s - nop\n", __func__);

	/* check for platform data */
	if (!data)
		return 0;

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

static void __exit baseband_xmm_power2_exit(void)
{
	pr_debug("%s\n", __func__);
	platform_driver_unregister(&baseband_power2_driver);
}

module_init(baseband_xmm_power2_init)
module_exit(baseband_xmm_power2_exit)
