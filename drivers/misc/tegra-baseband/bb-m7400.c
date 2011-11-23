/*
 * drivers/misc/tegra-baseband/bb-m7400.c
 *
 * Copyright (c) 2011, NVIDIA Corporation.
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

#include <linux/resource.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/err.h>
#include <linux/device.h>
#include <linux/usb.h>
#include <linux/wakelock.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <mach/tegra-bb-power.h>
#include "bb-power.h"

static struct tegra_bb_gpio_data m7400_gpios[] = {
	{ { GPIO_INVALID, GPIOF_OUT_INIT_LOW, "MDM_PWR_ON" }, true },
	{ { GPIO_INVALID, GPIOF_IN, "MDM_PWRSTATUS" }, true },
	{ { GPIO_INVALID, GPIOF_OUT_INIT_HIGH, "MDM_SERVICE" }, true },
	{ { GPIO_INVALID, GPIOF_OUT_INIT_HIGH, "MDM_USB_AWR" }, false },
	{ { GPIO_INVALID, GPIOF_IN, "MDM_USB_CWR" }, false },
	{ { GPIO_INVALID, GPIOF_IN, "MDM_RESOUT2" }, true },
	{ { GPIO_INVALID, 0, NULL }, false },	/* End of table */
};

static int gpio_wait_timeout(int gpio, int value, int timeout_msec)
{
	int count;
	for (count = 0; count < timeout_msec; ++count) {
		if (gpio_get_value(gpio) == value)
			return 0;
		mdelay(1);
	}
	return -1;
}

static int baseband_l3_suspend(void)
{
	int gpio_awr = m7400_gpios[3].data.gpio;

	/* Signal L3 to modem - Drive USB_AWR low. */
	gpio_set_value(gpio_awr, 0);

	return 0;
}

static int baseband_l3_resume(void)
{
	int gpio_awr = m7400_gpios[3].data.gpio;
	int gpio_cwr = m7400_gpios[4].data.gpio;

	/* Signal L0 to modem - Drive USB_AWR high. */
	gpio_set_value(gpio_awr, 1);

	/* If this is a AP driven wakeup, wait for CP
	   to ack by driving USB_CWR high.
	   If this is a CP driven wakeup, USB_CWR will
	   be high already. AP has already driven it's
	   response as above.
	*/
	if (gpio_wait_timeout(gpio_cwr, 1, 10) != 0)
		pr_info("%s: timeout waiting for modem ack.\n", __func__);

	return 0;
}

static irqreturn_t baseband_wake_irq(int irq, void *dev_id)
{
	pr_info("%s {\n", __func__);

	/* Resume usb host activity. */
	/* TBD */

	return IRQ_HANDLED;
}

int m7400_power_callback(int code)
{
	switch (code) {
	case CB_CODE_L0L2:
	break;
	case CB_CODE_L2L0:
	break;
	case CB_CODE_L2L3:
	baseband_l3_suspend();
	break;
	case CB_CODE_L3L0:
	baseband_l3_resume();
	break;
	default:
	break;
	}
	return 0;
}


static struct tegra_bb_gpio_irqdata m7400_gpioirqs[] = {
	{ GPIO_INVALID, "tegra_bb_wake", baseband_wake_irq,
					IRQF_TRIGGER_RISING, NULL },
	{ GPIO_INVALID, NULL, NULL, 0, NULL },	/* End of table */
};

static struct tegra_bb_power_gdata m7400_gdata = {
	.gpio = m7400_gpios,
	.gpioirq = m7400_gpioirqs,
};

void *m7400_init(void *pdata, int code)
{
	struct tegra_bb_pdata *platdata = (struct tegra_bb_pdata *) pdata;
	union tegra_bb_gpio_id *id = platdata->id;

	if (code == CB_CODE_INIT) {
		/* Fill the gpio ids allocated by hardware */
		m7400_gpios[0].data.gpio = id->m7400.pwr_on;
		m7400_gpios[1].data.gpio = id->m7400.pwr_status;
		m7400_gpios[2].data.gpio = id->m7400.service;
		m7400_gpios[3].data.gpio = id->m7400.usb_awr;
		m7400_gpios[4].data.gpio = id->m7400.usb_cwr;
		m7400_gpios[5].data.gpio = id->m7400.resout2;
		m7400_gpioirqs[0].id = id->m7400.usb_cwr;
	}

	return (void *) &m7400_gdata;
}
