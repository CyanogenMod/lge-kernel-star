/*
 * arch/arm/mach-tegra/board-enterprise.h
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

#ifndef _MACH_TEGRA_BOARD_ENTERPRISE_H
#define _MACH_TEGRA_BOARD_ENTERPRISE_H

#include <mach/gpio.h>
#include <mach/irqs.h>

int enterprise_charge_init(void);
int enterprise_sdhci_init(void);
int enterprise_pinmux_init(void);
int enterprise_panel_init(void);
int touch_init(void);
int enterprise_emc_init(void);
int enterprise_regulator_init(void);

/* Touchscreen GPIO addresses   */
#ifdef CONFIG_TOUCHSCREEN_ATMEL_MT_T9
#define TOUCH_GPIO_IRQ_ATMEL_T9	TEGRA_GPIO_PH4
#define TOUCH_GPIO_RST_ATMEL_T9	TEGRA_GPIO_PH6
#endif

#endif
