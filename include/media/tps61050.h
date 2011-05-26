/*
 * tps61050.h - tps61050 flash/torch kernel driver
 *
 * Copyright (C) 2011 NVIDIA Corp.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#ifndef __TPS61050_H__
#define __TPS61050_H__

#include <linux/ioctl.h> /* For IOCTL macros */

#define NUM_TORCH_LEVELS	8
#define NUM_FLASH_LEVELS	9

/* Expected higher level power calls are:
 * 1 = OFF
 * 2 = STANDBY
 * 3 = ON
 * These will be multiplied by 2 before given to the driver PM code
 * that uses the _PWR_ defines. This allows us to insert defines
 * (most notably _PWR_COMM) allowing more versatile PM.
 * The PM actions:
 * _PWR_OFF     = board support for regulators, clocks, etc so no
 *                power to device.
 * _PWR_STDBY   = board support to put device in reset. Device is
 *                useless but powered.  No communication possible.
 * _PWR_COMM    = Device is powered but in SW standby. Communication
 *                is possible and device retains programming.
 * _PWR_ON      = Full power with active output.
 */

#define TPS61050_PWR_ERR		0
#define TPS61050_PWR_OFF		2
#define TPS61050_PWR_STDBY		4
#define TPS61050_PWR_COMM		5
#define TPS61050_PWR_ON			6

#define TPS61050_CFG_NOSENSOR		(1 << 0)

#define TPS61050_IOCTL_CAP		_IOWR('o', 1, struct tps61050_param)
#define TPS61050_IOCTL_PWR		_IOW('o', 2, __u8)
#define TPS61050_IOCTL_PARAM_RD		_IOWR('o', 3, struct tps61050_param)
#define TPS61050_IOCTL_PARAM_WR		_IOW('o', 4, struct tps61050_param)

#define TPS61050_FLASH_CAPS		6
#define TPS61050_FLASH_LEVEL		7
#define TPS61050_FLASH_PIN_STATE	8
#define TPS61050_TORCH_CAPS		9
#define TPS61050_TORCH_LEVEL		10

struct tps61050_param {
	int param;
	__s32 sizeofvalue;
	void *p_value;
};

struct tps61050_level_info {
	__s32 guidenum;
	__u32 sustaintime;
	__s32 rechargefactor;
};

struct tps61050_pin_state {
	__u16 mask;
	__u16 values;
};

struct tps61050_flash_capabilities {
	__u32 numberoflevels;
	struct tps61050_level_info levels[NUM_FLASH_LEVELS];
};

struct tps61050_torch_capabilities {
	__u32 numberoflevels;
	__s32 guidenum[NUM_TORCH_LEVELS];
};

#ifdef __KERNEL__
struct tps61050_platform_data {
	int cfg;
	int num;
	int max_amp_torch;
	int max_amp_flash;
	void (*pinstate);
	int (*init)(void);
	void (*exit)(void);
	int (*pm)(int);
	int (*gpio_envm)(int);
	int (*gpio_sync)(int);
};
#endif /* __KERNEL__ */

#endif /* __TPS61050_H__ */
