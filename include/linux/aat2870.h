/*
 * linux/include/linux/aat2870_bl.h
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __LINUX_AAT2870_BL_H
#define __LINUX_AAT2870_BL_H

// LGE_CHANGE_S [chan.jeong@address] 2011-01-22, [LGE_AP20] earlysuspend
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
// LGE_CHANGE_E [chan.jeong@address] 2011-01-22, [LGE_AP20] earlysuspend

/* True/False Definition */
#if !defined(TRUE)
#define TRUE	0x01
#endif
#if !defined(FALSE)
#define FALSE	0x00 
#endif

#define AAT2870_BL_DRV_NAME	"aat2870-backlight"

/* Register offsets */
#define AAT2870_BL_EN		0x00
#define AAT2870_BL_BLM		0x01
#define AAT2870_BL_BLS		0x02
#define AAT2870_BL_BL1		0x03
#define AAT2870_BL_BL2		0x04
#define AAT2870_BL_BL3		0x05
#define AAT2870_BL_BL4		0x06
#define AAT2870_BL_BL5		0x07
#define AAT2870_BL_BL6		0x08
#define AAT2870_BL_BL7		0x09
#define AAT2870_BL_BL8		0x0A
#define AAT2870_BL_FLR		0x0B
#define AAT2870_BL_FM		0x0C
#define AAT2870_BL_FS		0x0D
#define AAT2870_BL_ALSF_	0x0E
#define AAT2870_BL_SBIAS	0x0F
#define AAT2870_BL_ALS_G	0x10
#define AAT2870_BL_AMB		0x11
#define AAT2870_BL_ALS0		0x12
#define AAT2870_BL_ALS1		0x13
#define AAT2870_BL_ALS2		0x14
#define AAT2870_BL_ALS3		0x15
#define AAT2870_BL_ALS4		0x16
#define AAT2870_BL_ALS5		0x17
#define AAT2870_BL_ALS6		0x18
#define AAT2870_BL_ALS7		0x19
#define AAT2870_BL_ALS8		0x1A
#define AAT2870_BL_ALS9		0x1B
#define AAT2870_BL_ALSA		0x1C
#define AAT2870_BL_ALSB		0x1D
#define AAT2870_BL_ALSC		0x1E
#define AAT2870_BL_ALSD		0x1F
#define AAT2870_BL_ALSE		0x20
#define AAT2870_BL_ALSF		0x21
#define AAT2870_BL_SUB_SET	0x22
#define AAT2870_BL_SUB_CTRL	0x23
#define AAT2870_BL_LDO_AB	0x24
#define AAT2870_BL_LDO_CD	0x25
#define AAT2870_BL_EN_LDO	0x26

/* Parameters */
#define AAT2870_HW_RESET_DELAY  100	// Unit:us
#define AAT2870_INTENSITY_MAX   0x16
#define AAT2870_DEFAULT_LSENSOR_POLL_TIME msecs_to_jiffies(1000)
#define AAT2870_FADE_IN_DELAY   400
#define AAT2870_FADE_OUT_DELAY  400
#define AAT2870_POWER_STATE_ON  0x01
#define AAT2870_POWER_STATE_OFF 0x00

/* Other Parameters */
#define LCD_LED_DIM     1
#define BRIGHTNESS_MIN  30
#define NUMERATOR1      6
#define NUMERATOR2      14
#define TURNING_POINT   104

#define BACKLIGHT_DEFAULT 0x0B   //101017, sk.jang@lge.com set the value to the current consumption '9.9mA'

/* LDOA ~ LDOD enable offsets */
enum aat2870_bl_ldo {
	AAT2870_BL_LDOA,
	AAT2870_BL_LDOB,
	AAT2870_BL_LDOC,
	AAT2870_BL_LDOD
};

/* LDOA ~ LDOD voltages */
enum aat2870_bl_ldo_voltage {
	AAT2870_BL_LDO_1_2V,
	AAT2870_BL_LDO_1_3V,
	AAT2870_BL_LDO_1_5V,
	AAT2870_BL_LDO_1_6V,
	AAT2870_BL_LDO_1_8V,
	AAT2870_BL_LDO_2_0V,
	AAT2870_BL_LDO_2_2V,
	AAT2870_BL_LDO_2_5V,
	AAT2870_BL_LDO_2_6V,
	AAT2870_BL_LDO_2_7V,
	AAT2870_BL_LDO_2_8V,
	AAT2870_BL_LDO_2_9V,
	AAT2870_BL_LDO_3_0V,
	AAT2870_BL_LDO_3_1V,
	AAT2870_BL_LDO_3_2V,
	AAT2870_BL_LDO_3_3V,
	AAT2870_BL_LDO_MASK = AAT2870_BL_LDO_3_3V
};

/* Backlight current magnitude (mA) */
enum aat2870_bl_current {
	AAT2870_BL_CURRENT_0_45,
	AAT2870_BL_CURRENT_0_90,
	AAT2870_BL_CURRENT_1_80,
	AAT2870_BL_CURRENT_2_70,
	AAT2870_BL_CURRENT_3_60,
	AAT2870_BL_CURRENT_4_50,
	AAT2870_BL_CURRENT_5_40,
	AAT2870_BL_CURRENT_6_30,
	AAT2870_BL_CURRENT_7_20,
	AAT2870_BL_CURRENT_8_10,
	AAT2870_BL_CURRENT_9_00,
	AAT2870_BL_CURRENT_9_90,
	AAT2870_BL_CURRENT_10_8,
	AAT2870_BL_CURRENT_11_7,
	AAT2870_BL_CURRENT_12_6,
	AAT2870_BL_CURRENT_13_5,
	AAT2870_BL_CURRENT_14_4,
	AAT2870_BL_CURRENT_15_3,
	AAT2870_BL_CURRENT_16_2,
	AAT2870_BL_CURRENT_17_1,
	AAT2870_BL_CURRENT_18_0,
	AAT2870_BL_CURRENT_18_9,
	AAT2870_BL_CURRENT_19_8,
	AAT2870_BL_CURRENT_20_7,
	AAT2870_BL_CURRENT_21_6,
	AAT2870_BL_CURRENT_22_5,
	AAT2870_BL_CURRENT_23_4,
	AAT2870_BL_CURRENT_24_3,
	AAT2870_BL_CURRENT_25_2,
	AAT2870_BL_CURRENT_26_1,
	AAT2870_BL_CURRENT_27_0,
	AAT2870_BL_CURRENT_27_9
};

/* Command Interface */
struct aat2870_ctl_tbl_t {
        unsigned char reg;
        unsigned char val;
};

struct aat2870_lux_tbl_t {
        unsigned int lev;
        unsigned int lux;
};

struct aat2870_cmds_t {
        struct aat2870_ctl_tbl_t *normal;
        struct aat2870_ctl_tbl_t *alc;
        struct aat2870_ctl_tbl_t *sleep;
};

struct AAT2870_initial_ALC_cur {
        unsigned char reg;
        unsigned char val;
};

/* Register Format */
#define AAT2870_FM_REG_FMT 2
#define AAT2870_FM_REG_EN_FM 1
#define AAT2870_FM_REG_INIT_FM 0
#define AAT2870_FM_REG_FADEIN_EN_VAL (((AAT2870_FADE_IN_DELAY == 1000? 0: \
                                                        AAT2870_FADE_IN_DELAY == 800?    1: \
                                                        AAT2870_FADE_IN_DELAY == 600?    2: \
                                                        AAT2870_FADE_IN_DELAY == 400?    3: 3) << AAT2870_FM_REG_FMT) | \
                                                        (1 << AAT2870_FM_REG_INIT_FM))
#define AAT2870_FM_REG_FADEOUT_EN_VAL (((AAT2870_FADE_OUT_DELAY == 1000? 0: \
                                                        AAT2870_FADE_IN_DELAY == 800?    1: \
                                                        AAT2870_FADE_IN_DELAY == 600?    2: \
                                                        AAT2870_FADE_IN_DELAY == 400?    3: 3) << AAT2870_FM_REG_FMT) | \
                                                        (0 << AAT2870_FM_REG_INIT_FM))

struct aat2870_bl_driver_data {
	struct i2c_client *client;
	struct backlight_device *bd;

	int en_pin;
	int avail_ch;
	enum aat2870_bl_current max_current;

	int brightness; /* current brightness */
	int reg_cache[AAT2870_BL_EN_LDO + 1]; /* register caches */

	/* Other Driver Information Block */
        int intensity;
	int op_mode;
        int intensity_max;
        int status;
        int dim_status;		//For dimming in ALC Mode
        int hw_dimming_enable;
        int lsensor_enable;
        int lsensor_poll_time;
        int version;
        struct mutex lock;
        struct mutex cmd_lock;
        struct aat2870_cmds_t cmds;
        struct delayed_work delayed_work_bl;
        struct input_dev *input_dev;
        unsigned char power_onoff_ref;
	/* End Driver Information Block */

	int (*init)(struct backlight_device *bd);
	void (*uninit)(struct backlight_device *bd);
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
};

struct aat2870_bl_platform_data {
	int en_pin; /* enable GPIO (if < 0, ignore this value) */
	int avail_ch; /* available backlight channels, < 0xFF */
	enum aat2870_bl_current max_current; /* backlight current magnitude */
	int max_brightness; /* if < 0, set this to 100 */

	int (*init)(struct backlight_device *bd);
	void (*uninit)(struct backlight_device *bd);
};

int aat2870_bl_set_ldo(enum aat2870_bl_ldo ldo,
		       enum aat2870_bl_ldo_voltage volt, int en);

#endif /* __LINUX_AAT2870_BL_H */
