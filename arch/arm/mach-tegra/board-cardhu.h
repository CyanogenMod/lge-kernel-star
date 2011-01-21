/*
 * arch/arm/mach-tegra/board-cardhu.h
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

#ifndef _MACH_TEGRA_BOARD_CARDHU_H
#define _MACH_TEGRA_BOARD_CARDHU_H

#include <mach/gpio.h>

#define TPS6591X_GPIO_BASE	TEGRA_NR_GPIOS

/* PCA954x I2C bus expander bus addresses */
#define PCA954x_I2C_BUS_BASE	6
#define PCA954x_I2C_BUS0	(PCA954x_I2C_BUS_BASE + 0)
#define PCA954x_I2C_BUS1	(PCA954x_I2C_BUS_BASE + 1)
#define PCA954x_I2C_BUS2	(PCA954x_I2C_BUS_BASE + 2)
#define PCA954x_I2C_BUS3	(PCA954x_I2C_BUS_BASE + 3)

int cardhu_charge_init(void);
int cardhu_regulator_init(void);
int cardhu_suspend_init(void);
int cardhu_sdhci_init(void);
int cardhu_pinmux_init(void);
int cardhu_panel_init(void);
int cardhu_sensors_init(void);
int cardhu_kbc_init(void);
int cardhu_scroll_init(void);
int cardhu_keys_init(void);

#endif
