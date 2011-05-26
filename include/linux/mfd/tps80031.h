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

#define tps80031_rails(_name) "tps80031_"#_name

enum {
	TPS80031_ID_VIO,
	TPS80031_ID_SMPS1,
	TPS80031_ID_SMPS2,
	TPS80031_ID_SMPS3,
	TPS80031_ID_SMPS4,
	TPS80031_ID_VANA,
	TPS80031_ID_LDO1,
	TPS80031_ID_LDO2,
	TPS80031_ID_LDO3,
	TPS80031_ID_LDO4,
	TPS80031_ID_LDO5,
	TPS80031_ID_LDO6,
	TPS80031_ID_LDO7,
	TPS80031_ID_LDOLN,
	TPS80031_ID_LDOUSB,
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
};

struct tps80031_subdev_info {
	int		id;
	const char	*name;
	void		*platform_data;
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

/*
 * NOTE: the functions below are not intended for use outside
 * of the TPS80031 sub-device drivers
 */
extern int tps80031_write(struct device *dev, int reg, uint8_t val);
extern int tps80031_writes(struct device *dev, int reg, int len, uint8_t *val);
extern int tps80031_read(struct device *dev, int reg, uint8_t *val);
extern int tps80031_reads(struct device *dev, int reg, int len, uint8_t *val);
extern int tps80031_set_bits(struct device *dev, int reg, uint8_t bit_mask);
extern int tps80031_clr_bits(struct device *dev, int reg, uint8_t bit_mask);
extern int tps80031_update(struct device *dev, int reg, uint8_t val,
			   uint8_t mask);
extern int tps80031_power_off(void);

#endif /*__LINUX_MFD_TPS80031_H */
