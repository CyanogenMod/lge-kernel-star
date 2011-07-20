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
#include <linux/delay.h>
#include <mach/usb_phy.h>
#include "board.h"
#include "devices.h"
#include "gpio-names.h"
#include "baseband-xmm-power.h"

MODULE_LICENSE("GPL");

unsigned long enum_delay_ms = 1000;

module_param(enum_delay_ms, ulong, 0644);
MODULE_PARM_DESC(enum_delay_ms, "baseband xmm power"
			" - delay in ms between modem on and enumeration");

/* Currently no baseband initiated suspend */
#define BB_INITIATED_L2_SUSPEND 0

static struct gpio tegra_baseband_gpios[] = {
	{ -1, GPIOF_OUT_INIT_LOW,  "BB_RSTn" },
	{ -1, GPIOF_OUT_INIT_LOW,  "BB_ON"   },
	{ -1, GPIOF_OUT_INIT_LOW,  "IPC_BB_WAKE" },
	{ -1, GPIOF_IN,            "IPC_AP_WAKE" },
	{ -1, GPIOF_OUT_INIT_HIGH, "IPC_HSIC_ACTIVE" },
#if BB_INITIATED_L2_SUSPEND
	{ -1, GPIOF_IN,            "IPC_HSIC_SUS_REQ" },
#endif
};

#if BB_INITIATED_L2_SUSPEND
static enum {
	IPC_HSIC_SUS_REQ_UNINIT,
	IPC_HSIC_SUS_REQ_IRQ_READY,
	IPC_HSIC_SUS_REQ_INIT,
	IPC_HSIC_SUS_REQ_L,
	IPC_HSIC_SUS_REQ_H,
} ipc_hsic_sus_req_state;
#endif

static enum {
	IPC_AP_WAKE_UNINIT,
	IPC_AP_WAKE_IRQ_READY,
	IPC_AP_WAKE_INIT1,
	IPC_AP_WAKE_INIT2,
	IPC_AP_WAKE_L,
	IPC_AP_WAKE_H,
} ipc_ap_wake_state;

static enum {
	BBXMM_PS_UNINIT   = 0,
	BBXMM_PS_INIT     = 1,
	BBXMM_PS_L0       = 2,
	BBXMM_PS_L0TOL2   = 3,
	BBXMM_PS_L2       = 4,
	BBXMM_PS_L2TOL0   = 5,
	BBXMM_PS_L2TOL3   = 6,
	BBXMM_PS_L3       = 7,
	BBXMM_PS_L3TOL0   = 8,
	BBXMM_PS_LAST     = -1,
} baseband_xmm_powerstate;

static struct workqueue_struct *workqueue;
static struct work_struct init1_work;
static struct work_struct init2_work;
static struct baseband_power_platform_data *baseband_power_driver_data;

static void baseband_xmm_set_power_status(unsigned int status)
{
	baseband_xmm_powerstate = status;
	pr_debug("BB XMM POWER STATE = %d\n", status);
}

#if BB_INITIATED_L2_SUSPEND
static irqreturn_t ipc_hsic_sus_req_irq(int irq, void *dev_id)
{
	int value;

	pr_debug("%s\n", __func__);

	if (ipc_hsic_sus_req_state < IPC_HSIC_SUS_REQ_IRQ_READY) {
		pr_err("%s - spurious irq\n", __func__);
	} else {
		value = gpio_get_value(baseband_power_driver_data->
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

#endif

static irqreturn_t ipc_ap_wake_irq(int irq, void *dev_id)
{
	int value;

	pr_debug("%s\n", __func__);

	if (ipc_ap_wake_state < IPC_AP_WAKE_IRQ_READY) {
		pr_err("%s - spurious irq\n", __func__);
	} else if (ipc_ap_wake_state == IPC_AP_WAKE_IRQ_READY) {
		value = gpio_get_value(baseband_power_driver_data->
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
		value = gpio_get_value(baseband_power_driver_data->
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
		value = gpio_get_value(baseband_power_driver_data->
			modem.xmm.ipc_ap_wake);
		if (!value) {
			pr_debug("%s - falling\n", __func__);
			pr_debug("gpio host wakeup done <-\n");
			/* Set the slave wakeup request */
			gpio_set_value(baseband_power_driver_data->
				modem.xmm.ipc_bb_wake, 0);
			pr_debug("gpio slave wakeup done ->\n");
			ipc_ap_wake_state = IPC_AP_WAKE_L;
			baseband_xmm_set_power_status(BBXMM_PS_L0);
		} else {
			pr_debug("%s - rising\n", __func__);
			ipc_ap_wake_state = IPC_AP_WAKE_H;
		}
	}

	return IRQ_HANDLED;
}

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
	pr_debug("%s\n", __func__);

	/* register usb host controller */
	platform_device_register(baseband_power_driver_data->
		modem.xmm.hsic_device);

	baseband_xmm_set_power_status(BBXMM_PS_L0);

}

static int baseband_xmm_power_driver_probe(struct platform_device *device)
{
	struct baseband_power_platform_data *data
	    = (struct baseband_power_platform_data *) device->dev.platform_data;
	int err;

	pr_debug("%s\n", __func__);

	baseband_xmm_powerstate = BBXMM_PS_UNINIT;
	/* check if supported modem */
	if (data->baseband_type != BASEBAND_XMM) {
		pr_err("unsuppported modem\n");
		return -ENODEV;
	}

	/* save platform data */
	baseband_power_driver_data = data;

	/* request baseband gpio(s) */
	tegra_baseband_gpios[0].gpio = data->modem.xmm.bb_rst;
	tegra_baseband_gpios[1].gpio = data->modem.xmm.bb_on;
	tegra_baseband_gpios[2].gpio = data->modem.xmm.ipc_bb_wake;
	tegra_baseband_gpios[3].gpio = data->modem.xmm.ipc_ap_wake;
	tegra_baseband_gpios[4].gpio = data->modem.xmm.ipc_hsic_active;
#if BB_INITIATED_L2_SUSPEND
	tegra_baseband_gpios[5].gpio = data->modem.xmm.ipc_hsic_sus_req;
#endif
	err = gpio_request_array(tegra_baseband_gpios,
		ARRAY_SIZE(tegra_baseband_gpios));
	if (err < 0) {
		pr_err("%s - request gpio(s) failed\n", __func__);
		return err;
	}

	/* request baseband irq(s) */
#if BB_INITIATED_L2_SUSPEND
	if (enum_delay_ms) {
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
	}
#endif
	if (enum_delay_ms) {
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
	}

	/* init work queue */
	workqueue = create_singlethread_workqueue
		("baseband_xmm_power_workqueue");
	if (!workqueue) {
		pr_err("cannot create workqueue\n");
		return -1;
	}

	baseband_xmm_powerstate = BBXMM_PS_INIT;
	INIT_WORK(&init1_work, baseband_xmm_power_init1_work);
	INIT_WORK(&init2_work, baseband_xmm_power_init2_work);

	/* reset / power on sequence */
	mdelay(40);
	gpio_set_value(data->modem.xmm.bb_rst, 1);
	mdelay(1);
	gpio_set_value(data->modem.xmm.bb_on, 1);
	udelay(40);
	gpio_set_value(data->modem.xmm.bb_on, 0);

	/* optional delay
	 * 0 = flashless
	 *   ==> causes next step to enumerate modem boot rom (058b / 0041)
	 * some delay > boot rom timeout
	 *   ==> causes next step to enumerate modem software (1519 / 0020)
	 *       (requires modem to be flash version, not flashless version)
	*/
	if (enum_delay_ms)
		mdelay(enum_delay_ms);

	/* register usb host controller */
	if (!enum_delay_ms)
		platform_device_register(data->modem.xmm.hsic_device);

	return 0;
}

static int baseband_xmm_power_driver_remove(struct platform_device *device)
{
	struct baseband_power_platform_data *data
	 = (struct baseband_power_platform_data *) device->dev.platform_data;

	pr_debug("%s\n", __func__);

	/* free baseband irq(s) */
	if (enum_delay_ms) {
		free_irq(gpio_to_irq(data->modem.xmm.ipc_ap_wake), NULL);
#if BB_INITIATED_L2_SUSPEND
		free_irq(gpio_to_irq(data->modem.xmm.ipc_hsic_sus_req), NULL);
#endif
	}

	/* free baseband gpio(s) */
	gpio_free_array(tegra_baseband_gpios,
		ARRAY_SIZE(tegra_baseband_gpios));

	/* unregister usb host controller */
	platform_device_unregister(&tegra_ehci2_device);

	return 0;
}

static int baseband_xmm_power_driver_suspend(struct platform_device *device,
	pm_message_t state)
{
	struct baseband_power_platform_data *data
	 = (struct baseband_power_platform_data *) device->dev.platform_data;

	pr_debug("%s\n", __func__);

	/* Indiate host active low to CP*/
	gpio_set_value(data->modem.xmm.ipc_hsic_active, 0);
	pr_debug("gpio host active low->\n");

	baseband_xmm_set_power_status(BBXMM_PS_L3);

	return 0;
}

static int baseband_xmm_power_driver_resume(struct platform_device *device)
{
	struct baseband_power_platform_data *data
	 = (struct baseband_power_platform_data *) device->dev.platform_data;
	int value;

	pr_debug("%s\n", __func__);

	/* wake bb */
	gpio_set_value(data->modem.xmm.ipc_bb_wake, 1);

	pr_debug("waiting for host wakeup\n");
	do {
		mdelay(1);
		value = gpio_get_value(baseband_power_driver_data
			->modem.xmm.ipc_ap_wake);
	} while (!value);
	pr_debug("gpio host wakeup high <-\n");

	/* signal bb to resume hsic */
	gpio_set_value(data->modem.xmm.ipc_hsic_active, 1);

	baseband_xmm_set_power_status(BBXMM_PS_L3TOL0);

	return 0;
}

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
	pr_debug("%s\n", __func__);
	return platform_driver_register(&baseband_power_driver);
}

static void __exit baseband_xmm_power_exit(void)
{
	pr_debug("%s\n", __func__);
	platform_driver_unregister(&baseband_power_driver);
}

module_init(baseband_xmm_power_init)
module_exit(baseband_xmm_power_exit)
