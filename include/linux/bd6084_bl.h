/*
 * linux/include/linux/aat2870_bl.h
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __LINUX_BD6084_BL_H
#define __LINUX_BD6084_BL_H

// LGE_CHANGE_S [chan.jeong@address] 2011-01-22, [LGE_AP20] earlysuspend
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
// LGE_CHANGE_E [chan.jeong@address] 2011-01-22, [LGE_AP20] earlysuspend

#define BD6084_BL_DRV_NAME	"lcd-backlight"

/* LDOA ~ LDOD enable offsets */
enum bd6084_ldo {
	BD6084_LDO1,
	BD6084_LDO2,
	BD6084_LDO3,
	BD6084_LDO4,
	BD6084_LDO12,
	BD6084_LDO34
};

struct bd6084_bl_driver_data {
	struct i2c_client *client;
	struct backlight_device *bd;

	int en_pin;
	int avail_ch;
	int max_current;

	int brightness; /* current brightness */
	int reg_cache[0x16]; /* register caches */

	int (*init)(struct backlight_device *bd);
	void (*uninit)(struct backlight_device *bd);

	int (*check_fb)(struct device *dev, struct fb_info *info);

// LGE_CHANGE_S [chan.jeong@address] 2011-01-22, [LGE_AP20] earlysuspend
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
// LGE_CHANGE_E [chan.jeong@address] 2011-01-22, [LGE_AP20] earlysuspend
};

struct bd6084_bl_platform_data {
	int en_pin; /* enable GPIO (if < 0, ignore this value) */
	int avail_ch; /* available backlight channels, < 0xFF */
	int max_current; /* backlight current magnitude */
	int max_brightness; /* if < 0, set this to 100 */

	int (*init)(struct backlight_device *bd);
	void (*uninit)(struct backlight_device *bd);

	int (*check_fb)(struct device *dev, struct fb_info *info);
};

int bd6084_bl_set_ldo(enum bd6084_ldo ldo, int en);
int bd6084_forced_off(void);
int bd6084_forced_resume(void);
bool bd6084_bl_is_LP1(void);

#endif /* __LINUX_AAT2870_BL_H */
