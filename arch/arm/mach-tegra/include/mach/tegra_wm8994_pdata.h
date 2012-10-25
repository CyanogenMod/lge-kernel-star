/*
 * arch/arm/mach-tegra/include/mach/tegra_wm8903_pdata.h
 *
 * Copyright 2011 NVIDIA, Inc.
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

struct tegra_wm8994_platform_data {
	const char *name;	//LGE_CHANGE_S [chahee.kim@lge.com] 2011-11-14 
	int gpio_hook;		//LGE_CHANGE_S [chahee.kim@lge.com] 2011-11-14 
	int gpio_ear_mic;	//LGE_CHANGE_S [chahee.kim@lge.com] 2011-11-14 

	int gpio_spkr_en;
	int gpio_hp_det;
	int gpio_hp_mute;
	int gpio_int_mic_en;
	int gpio_ext_mic_en;
	unsigned int debounce_time_hp;
};
