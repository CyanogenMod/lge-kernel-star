/*
 * include/linux/mfd/tps80031.c
 *
 * Core driver interface for TI TPS80031 PMIC
 *
 * Copyright (C) 2011 NVIDIA Corporation
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
 *
 */

#ifndef __LINUX_MFD_TPS80031_H
#define __LINUX_MFD_TPS80031_H

#include <linux/rtc.h>

/* Supported chips */
enum chips {
	TPS80031 = 0x00000001,
	TPS80032 = 0x00000002,
};

enum {
	TPS80031_INT_PWRON,
	TPS80031_INT_RPWRON,
	TPS80031_INT_SYS_VLOW,
	TPS80031_INT_RTC_ALARM,
	TPS80031_INT_RTC_PERIOD,
	TPS80031_INT_HOT_DIE,
	TPS80031_INT_VXX_SHORT,
	TPS80031_INT_SPDURATION,
	TPS80031_INT_WATCHDOG,
	TPS80031_INT_BAT,
	TPS80031_INT_SIM,
	TPS80031_INT_MMC,
	TPS80031_INT_RES,
	TPS80031_INT_GPADC_RT,
	TPS80031_INT_GPADC_SW2_EOC,
	TPS80031_INT_CC_AUTOCAL,
	TPS80031_INT_ID_WKUP,
	TPS80031_INT_VBUSS_WKUP,
	TPS80031_INT_ID,
	TPS80031_INT_VBUS,
	TPS80031_INT_CHRG_CTRL,
	TPS80031_INT_EXT_CHRG,
	TPS80031_INT_INT_CHRG,
	TPS80031_INT_RES2,
	TPS80031_INT_BAT_TEMP_OVRANGE,
	TPS80031_INT_BAT_REMOVED,
	TPS80031_INT_VBUS_DET,
	TPS80031_INT_VAC_DET,
	TPS80031_INT_FAULT_WDG,
	TPS80031_INT_LINCH_GATED,

	/* Last interrupt id to get the end number */
	TPS80031_INT_NR,
};

enum TPS80031_GPIO {
	TPS80031_GPIO_REGEN1,
	TPS80031_GPIO_REGEN2,
	TPS80031_GPIO_SYSEN,

	/* Last entry */
	TPS80031_GPIO_NR,
};

enum {
	SLAVE_ID0 = 0,
	SLAVE_ID1 = 1,
	SLAVE_ID2 = 2,
	SLAVE_ID3 = 3,
};

enum {
	I2C_ID0_ADDR = 0x12,
	I2C_ID1_ADDR = 0x48,
	I2C_ID2_ADDR = 0x49,
	I2C_ID3_ADDR = 0x4A,
};

struct tps80031_subdev_info {
	int		id;
	const char	*name;
	void		*platform_data;
};

struct tps80031_rtc_platform_data {
	int irq;
	struct rtc_time time;
};

struct tps80031_32kclock_plat_data {
	unsigned en_clk32kao:1;
	unsigned en_clk32kg:1;
	unsigned en_clk32kaudio:1;
};

struct tps80031_platform_data {
	int num_subdevs;
	struct tps80031_subdev_info *subdevs;
	int gpio_base;
	int irq_base;
	struct tps80031_32kclock_plat_data *clk32k_pdata;
};

struct tps80031_bg_platform_data {
	int irq_base;
};

/*
 * NOTE: the functions below are not intended for use outside
 * of the TPS80031 sub-device drivers
 */
extern int tps80031_write(struct device *dev, int sid, int reg, uint8_t val);
extern int tps80031_writes(struct device *dev, int sid, int reg, int len,
				uint8_t *val);
extern int tps80031_read(struct device *dev, int sid, int reg, uint8_t *val);
extern int tps80031_reads(struct device *dev, int sid, int reg, int len,
				uint8_t *val);
extern int tps80031_set_bits(struct device *dev, int sid, int reg,
				uint8_t bit_mask);
extern int tps80031_clr_bits(struct device *dev, int sid, int reg,
				uint8_t bit_mask);
extern int tps80031_update(struct device *dev, int sid, int reg, uint8_t val,
			   uint8_t mask);
extern int tps80031_force_update(struct device *dev, int sid, int reg,
				 uint8_t val, uint8_t mask);
extern int tps80031_power_off(void);

extern unsigned long tps80031_get_chip_info(struct device *dev);

#endif /*__LINUX_MFD_TPS80031_H */
