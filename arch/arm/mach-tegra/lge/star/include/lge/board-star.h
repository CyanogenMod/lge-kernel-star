/*
 * arch/arm/mach-tegra/board-star.h
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

#ifndef _MACH_TEGRA_BOARD_STAR_H
#define _MACH_TEGRA_BOARD_STAR_H

typedef enum
{
	REV_A = 0,
	REV_C,
	REV_D,
	REV_E,
	REV_F,
	REV_G,
	REV_H,
	REV_I,
	REV_J,
	REV_1_0,
	REV_1_1,
	REV_1_2,
	REV_1_3,
} hw_rev;

extern int star_regulator_init(void);
extern int star_sdhci_init(void);
extern int star_pinmux_init(void);
extern int star_panel_init(void);
extern int star_kbc_init(void);
extern int star_sensors_init(void);
extern int star_emc_init(void);
extern void star_audio_init(void);
extern hw_rev get_hw_rev(void);
extern void star_misc_init(void);
extern void star_usb_init(void);
extern int __init star_touch_init(void);
extern int __init star_touch_led_init(void);
extern void __init star_power_off_init(void);
//LGE_CHANGE_S [munho2.lee@lge.com] 2012-02-06 for Bluetooth bring-up
extern void star_bt_rfkill(void); 
//LGE_CHANGE_E [munho2.lee@lge.com]
extern noinline void __init tegra_setup_bluesleep(void);

#define STAR_I2C_DEVICE_ADDR_MUIC		0x44
#define STAR_I2C_DEVICE_ADDR_WM8994		0x1A//0X34
#define STAR_I2C_DEVICE_ADDR_CAM_PMIC		0X7D
#define STAR_I2C_DEVICE_ADDR_CAM_IMX073		0x34
#define STAR_I2C_DEVICE_ADDR_FOCUSER_DW9712	0x18
#define STAR_I2C_DEVICE_ADDR_CAM_MT9M113       	0x7A

/* Interrupt numbers from external peripherals */
#define MAX8907C_INT_BASE       TEGRA_NR_IRQS
#define MAX8907C_INT_END        (MAX8907C_INT_BASE + 31)

/* Audio-related GPIOs */
#define WHISTLER_GPIO_WM8753(_x_)	(MAX8907C_INT_END + 1 + (_x_))
#define TEGRA_GPIO_SPKR_EN		WHISTLER_GPIO_WM8753(2)
#define TEGRA_GPIO_HP_DET		TEGRA_GPIO_PG3
//LGE_CHANGE_S [chahee.kim@lge.com] 2012-02-01 
#if (defined(CONFIG_MACH_STAR_P990) || defined(CONFIG_MACH_STAR_P999))
#define TEGRA_GPIO_HP_HOOK	TEGRA_GPIO_PD3
#define TEGRA_GPIO_EAR_MIC	TEGRA_GPIO_PH1		//LGE_CHANGE_S [chahee.kim@lge.com] 2011-11-14 
#elif defined (CONFIG_MACH_STAR_SU660)
#define TEGRA_GPIO_HP_HOOK	TEGRA_GPIO_PN5		//LGE_CHANGE_S [chahee.kim@lge.com] 2011-11-14 
#define TEGRA_GPIO_EAR_MIC	TEGRA_GPIO_PH1		//LGE_CHANGE_S [chahee.kim@lge.com] 2011-11-14 
#endif

#define TEGRA_GPIO_HDMI_EN	TEGRA_GPIO_PK5		//MOBII_CHANGE [jg.noh@.mobii.co.kr] 2012.03.01
#define TEGRA_GPIO_HDMI_HPD	TEGRA_GPIO_PN7

extern struct wm8994_pdata wm8994_data;
extern struct platform_device star_audio_device;
extern struct platform_device star_headset_detect_device;
extern struct platform_device max8922l_charger_ic_device;
extern struct platform_device star_battery_charger_device;

#endif
