/*
 * include/linux/mfd/tps80031x.c
 * Core driver interface for TI TPS80031x PMIC family
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

#ifndef __LINUX_MFD_TPS80031X_H
#define __LINUX_MFD_TPS80031X_H

#define tps80031x_rails(_name) "tps80031x_"#_name

enum {
	TPS80031X_ID_SMPS4,
	TPS80031X_ID_VIO,
	TPS80031X_ID_SMPS1,
	TPS80031X_ID_SMPS2,
	TPS80031X_ID_SMPS3,
	TPS80031X_ID_VANA,
	TPS80031X_ID_LDO_2,
	TPS80031X_ID_LDO_4,
	TPS80031X_ID_LDO_3,
	TPS80031X_ID_LDO_6,
	TPS80031X_ID_LDO_LN,
	TPS80031X_ID_LDO_5,
	TPS80031X_ID_LDO_1,
	TPS80031X_ID_LDO_USB,
	TPS80031X_ID_LDO_7,
	TPS80031X_ID_LDO_VRTC,
};

enum {
	TPS80031X_INT_PWRHOLD_F,
	TPS80031X_INT_VMBHI,
	TPS80031X_INT_PWRON,
	TPS80031X_INT_PWRON_LP,
	TPS80031X_INT_PWRHOLD_R,
	TPS80031X_INT_HOTDIE,
	TPS80031X_INT_RTC_ALARM,
	TPS80031X_INT_RTC_PERIOD,
	TPS80031X_INT_GPIO0_R,
	TPS80031X_INT_GPIO0_F,
	TPS80031X_INT_GPIO1_R,
	TPS80031X_INT_GPIO1_F,
	TPS80031X_INT_GPIO2_R,
	TPS80031X_INT_GPIO2_F,
	TPS80031X_INT_GPIO3_R,
	TPS80031X_INT_GPIO3_F,
	TPS80031X_INT_GPIO4_R,
	TPS80031X_INT_GPIO4_F,
	TPS80031X_INT_GPIO5_R,
	TPS80031X_INT_GPIO5_F,
	TPS80031X_INT_WTCHDG,
	TPS80031X_INT_VMBCH2_H,
	TPS80031X_INT_VMBCH2_L,
	TPS80031X_INT_PWRDN,
};

struct tps80031x_subdev_info {
	int		id;
	const char	*name;
	void		*platform_data;
};

struct tps80031x_rtc_platform_data {
	int irq;
};

struct tps80031x_platform_data {
	int num_subdevs;
	struct tps80031x_subdev_info *subdevs;

	int gpio_base;
	int irq_base;
};

/*
 * NOTE: the functions below are not intended for use outside
 * of the TPS80031X sub-device drivers
 */
extern int tps80031x_write(struct device *dev, int reg, uint8_t val);
extern int tps80031x_writes(struct device *dev, int reg, int len, uint8_t *val);
extern int tps80031x_read(struct device *dev, int reg, uint8_t *val);
extern int tps80031x_reads(struct device *dev, int reg, int len, uint8_t *val);
extern int tps80031x_set_bits(struct device *dev, int reg, uint8_t bit_mask);
extern int tps80031x_clr_bits(struct device *dev, int reg, uint8_t bit_mask);
extern int tps80031x_update(struct device *dev, int reg, uint8_t val,
			   uint8_t mask);
extern int tps80031x_power_off(void);

#endif /*__LINUX_MFD_TPS80031X_H */
