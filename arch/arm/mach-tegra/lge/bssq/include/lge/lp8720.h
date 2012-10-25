#ifndef _LINUX_LP8720_H
#define _LINUX_LP8720_H

#define LP8720_I2C_NAME		"lp8720"
#define LP8720_I2C_ADDR		0x7d

//LGE_CHANGE_S [sungmo.yang@lge.com] 2011-01-17 [LGE_AP20], TEST!
extern int star_cam_Main_power_on(void); 	// cam,af on
extern int star_cam_VT_power_on(void);
extern int star_cam_power_off(void);		// cam, af off
//LGE_CHANGE_E [sungmo.yang@lge.com] 2011-01-17 [LGE_AP20], TEST!

struct lp8720_platform_data {
	int en_gpio_num;
};

#define LDO1_EN		(1 << 0)
#define LDO2_EN		(1 << 1)
#define LDO3_EN		(1 << 2)
#define LDO4_EN		(1 << 3)
#define LDO5_EN		(1 << 4)
#define BUCK_EN		(1 << 5)
#define SLEEP_EN	(1 << 6)
#define DVS_V1		(1 << 7)

#define TIMESTEP_25US		1 //default
#define TIMESTEP_50US		0

#define LP8720_GENERAL_SETTING		0x00
#define LP8720_LDO1_SETTING			0x01
#define LP8720_LDO2_SETTING			0x02
#define LP8720_LDO3_SETTING			0x03
#define LP8720_LDO4_SETTING			0x04
#define LP8720_LDO5_SETTING			0x05
#define LP8720_BUCK_SETTING1		0x06
#define LP8720_BUCK_SETTING2		0x07
#define LP8720_OUTPUT_ENABLE		0x08
#define LP8720_PULLDOWN_BITS		0x09

#define LP8720_STARTUP_DELAY_0		0x00
#define LP8720_STARTUP_DELAY_1TS	0x20
#define LP8720_STARTUP_DELAY_2TS	0x40
#define LP8720_STARTUP_DELAY_3TS	0x60
#define LP8720_STARTUP_DELAY_4TS	0x80
#define LP8720_STARTUP_DELAY_5TS	0xa0
#define LP8720_STARTUP_DELAY_6TS	0xc0
#define LP8720_NO_STARTUP			0xe0

#endif
