/*
 * arch/arm/mach-tegra/wakeups-t3.h
 *
 * Declarations of Tegra 3 LP0 wakeup sources
 *
 * Copyright (c) 2010, NVIDIA Corporation.
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

#ifndef __MACH_TEGRA_WAKEUPS_T3_H
#define __MACH_TEGRA_WAKEUPS_T3_H

#ifndef CONFIG_ARCH_TEGRA_3x_SOC
#error "Tegra 3 wakeup sources valid only for CONFIG_ARCH_TEGRA_3x_SOC"
#endif

#define TEGRA_WAKE_GPIO_PO5	(1ull << 0)
#define TEGRA_WAKE_GPIO_PV1	(1ull << 1)
#define TEGRA_WAKE_GPIO_PL1	(1ull << 2)
#define TEGRA_WAKE_GPIO_PB6	(1ull << 3)
#define TEGRA_WAKE_GPIO_PN7	(1ull << 4)
#define TEGRA_WAKE_GPIO_PBB6	(1ull << 5)
#define TEGRA_WAKE_GPIO_PU5	(1ull << 6)
#define TEGRA_WAKE_GPIO_PU6	(1ull << 7)
#define TEGRA_WAKE_GPIO_PC7	(1ull << 8)
#define TEGRA_WAKE_GPIO_PS2	(1ull << 9)
#define TEGRA_WAKE_GPIO_PAA1	(1ull << 10)
#define TEGRA_WAKE_GPIO_PW3	(1ull << 11)
#define TEGRA_WAKE_GPIO_PW2	(1ull << 12)
#define TEGRA_WAKE_GPIO_PY6	(1ull << 13)
#define TEGRA_WAKE_GPIO_PDD3	(1ull << 14)
#define TEGRA_WAKE_GPIO_PJ2	(1ull << 15)
#define TEGRA_WAKE_RTC_ALARM	(1ull << 16)
#define TEGRA_WAKE_KBC_EVENT	(1ull << 17)
#define TEGRA_WAKE_PWR_INT	(1ull << 18)
#define TEGRA_WAKE_USB1_VBUS	(1ull << 19)
#define TEGRA_WAKE_USB2_VBUS	(1ull << 20)
#define TEGRA_WAKE_USB1_ID	(1ull << 21)
#define TEGRA_WAKE_USB2_ID	(1ull << 22)
#define TEGRA_WAKE_GPIO_PI5	(1ull << 23)
#define TEGRA_WAKE_GPIO_PV0	(1ull << 24)
#define TEGRA_WAKE_GPIO_PS4	(1ull << 25)
#define TEGRA_WAKE_GPIO_PS5	(1ull << 26)
#define TEGRA_WAKE_GPIO_PS0	(1ull << 27)
#define TEGRA_WAKE_GPIO_PS6	(1ull << 28)
#define TEGRA_WAKE_GPIO_PS7	(1ull << 29)
#define TEGRA_WAKE_GPIO_PN2	(1ull << 30)
/* bit 31 is unused */

#define TEGRA_WAKE_GPIO_PO4	(1ull << 32)
#define TEGRA_WAKE_GPIO_PJ0	(1ull << 33)
#define TEGRA_WAKE_GPIO_PK2	(1ull << 34)
#define TEGRA_WAKE_GPIO_PI6	(1ull << 35)
#define TEGRA_WAKE_GPIO_PBB1	(1ull << 36)
#define TEGRA_WAKE_USB3_ID	(1ull << 37)
#define TEGRA_WAKE_USB3_VBUS	(1ull << 38)

#endif
