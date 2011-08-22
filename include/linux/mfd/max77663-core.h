/*
 * include/linux/mfd/max77663-core.h
 *
 * Copyright 2011 Maxim Integrated Products, Inc.
 * Copyright (C) 2011 NVIDIA Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version.
 *
 */

#ifndef __LINUX_MFD_MAX77663_CORE_H__
#define __LINUX_MFD_MAX77663_CORE_H__

#include <linux/irq.h>
#include <linux/mfd/core.h>

/*
 * Interrupts
 */
enum {
	MAX77663_IRQ_LBT_LB,		/* Low-Battery */
	MAX77663_IRQ_LBT_THERM_ALRM1,	/* Thermal alarm status, > 120C */
	MAX77663_IRQ_LBT_THERM_ALRM2,	/* Thermal alarm status, > 140C */

	MAX77663_IRQ_GPIO0,		/* GPIO0 edge detection */
	MAX77663_IRQ_GPIO1,		/* GPIO1 edge detection */
	MAX77663_IRQ_GPIO2,		/* GPIO2 edge detection */
	MAX77663_IRQ_GPIO3,		/* GPIO3 edge detection */
	MAX77663_IRQ_GPIO4,		/* GPIO4 edge detection */
	MAX77663_IRQ_GPIO5,		/* GPIO5 edge detection */
	MAX77663_IRQ_GPIO6,		/* GPIO6 edge detection */
	MAX77663_IRQ_GPIO7,		/* GPIO7 edge detection */

	MAX77663_IRQ_RTC_1SEC,		/* 1s timer expired */
	MAX77663_IRQ_RTC_60SEC,		/* 60s timer expired */
	MAX77663_IRQ_RTC_ALRM1,		/* Alarm 1 */
	MAX77663_IRQ_RTC_ALRM2,		/* Alarm 2 */
	MAX77663_IRQ_RTC_SMPL,		/* SMPL(Sudden Momentary Power Loss) */

	MAX77663_IRQ_ONOFF_HRDPOWRN,	/* Hard power off warnning */
	MAX77663_IRQ_ONOFF_EN0_1SEC,	/* EN0 active for 1s */
	MAX77663_IRQ_ONOFF_EN0_FALLING,	/* EN0 falling */
	MAX77663_IRQ_ONOFF_EN0_RISING,	/* EN0 rising */
	MAX77663_IRQ_ONOFF_LID_FALLING,	/* LID falling */
	MAX77663_IRQ_ONOFF_LID_RISING,	/* LID rising */
	MAX77663_IRQ_ONOFF_ACOK_FALLING,/* ACOK falling */
	MAX77663_IRQ_ONOFF_ACOK_RISING,	/* ACOK rising */

	MAX77663_IRQ_SD_PF,		/* SD power fail */
	MAX77663_IRQ_LDO_PF,		/* LDO power fail */
	MAX77663_IRQ_32K,		/* 32kHz oscillator */
	MAX77663_IRQ_NVER,		/* Non-Volatile Event Recorder */

	MAX77663_IRQ_NR,
};

/*
 *GPIOs
 */
enum {
	MAX77663_GPIO0,
	MAX77663_GPIO1,
	MAX77663_GPIO2,
	MAX77663_GPIO3,
	MAX77663_GPIO4,
	MAX77663_GPIO5,
	MAX77663_GPIO6,
	MAX77663_GPIO7,

	MAX77663_GPIO_NR,
};

enum max77663_gpio_alternate {
	GPIO_ALT_DISABLE,
	GPIO_ALT_ENABLE,
};

struct max77663_gpio_config {
	int gpio;	/* gpio number */
	bool alternate;	/* alternate mode */
};

struct max77663_platform_data {
	int irq_base;
	int gpio_base;
	int num_gpio_cfg;
	struct max77663_gpio_config *gpio_cfg;

	int num_subdevs;
	struct mfd_cell *sub_devices;
};

int max77663_read(struct device *dev, u8 addr, void *values, u32 len,
		  bool is_rtc);
int max77663_write(struct device *dev, u8 addr, void *values, u32 len,
		   bool is_rtc);
int max77663_set_bits(struct device *dev, u8 addr, u8 mask, u8 value,
		      bool is_rtc);
int max77663_gpio_set_alternate(int gpio, int alternate);

#endif /* __LINUX_MFD_MAX77663_CORE_H__ */
