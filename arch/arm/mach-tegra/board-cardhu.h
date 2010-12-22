/*
 * arch/arm/mach-tegra/board-cardhu.h
 *
 * Copyright (C) 2010 Google, Inc.
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

#ifndef _MACH_TEGRA_BOARD_CARDHU_H
#define _MACH_TEGRA_BOARD_CARDHU_H

int cardhu_charge_init(void);
int cardhu_regulator_init(void);
int cardhu_sdhci_init(void);
int cardhu_pinmux_init(void);
int cardhu_panel_init(void);
int cardhu_sensors_init(void);

#endif
