/*
 * arch/arm/mach-tegra/board-bssq.h
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

#ifndef _MACH_TEGRA_BOARD_BSSQ_H
#define _MACH_TEGRA_BOARD_BSSQ_H

int bssq_regulator_init(void);
int bssq_sdhci_init(void);
int bssq_pinmux_init(void);
int bssq_panel_init(void);
int bssq_kbc_init(void);
int bssq_emc_init(void);
extern void bssq_audio_init(void);
//LGE_CHANGE_S [minwook.huh@lge.com] 2012-06-20 for Bluetooth bring-up
extern void bssq_bt_rfkill(void); 
//LGE_CHANGE_E [minwook.huh@lge.com]

#define BSSQ_I2C_DEVICE_ADDR_WM8994		0x1A//0X34

/* Interrupt numbers from external peripherals */
#define MAX8907C_INT_BASE       TEGRA_NR_IRQS
#define MAX8907C_INT_END        (MAX8907C_INT_BASE + 31)

/* Audio-related GPIOs */
#define BSSQ_GPIO_WM8753(_x_)	(MAX8907C_INT_END + 1 + (_x_))
#define TEGRA_GPIO_SPKR_EN		BSSQ_GPIO_WM8753(2)
#if defined (CONFIG_LU6500) || defined (CONFIG_SU880) || defined (CONFIG_KU8800)
#define TEGRA_GPIO_HP_DET		TEGRA_GPIO_PE0
#else
#define TEGRA_GPIO_HP_DET		TEGRA_GPIO_PB3  // GPIO_HEADSET_DET(11)
#endif

#if defined (CONFIG_SU880) || defined (CONFIG_KU8800) 
#define TEGRA_GPIO_HP_HOOK    TEGRA_GPIO_PW2
#else
#define TEGRA_GPIO_HP_HOOK	TEGRA_GPIO_PJ7  // GPIO_HOOK_DET (79)
#endif
#define TEGRA_GPIO_EAR_MIC       TEGRA_GPIO_PX7  // GPIO_MIC_MODE (191)
extern struct wm8994_pdata wm8994_data;
extern struct platform_device bssq_audio_device;
//extern struct platform_device bssq_headset_detect_device;

#endif
