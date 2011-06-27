/**
 * Copyright (c) 2011 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#ifndef __SSL3250A_H__
#define __SSL3250A_H__

#include <linux/ioctl.h> /* For IOCTL macros */

#define SSL3250A_IOCTL_MODE_SHUTDOWN	_IOW('o', 1, __u8)
#define SSL3250A_IOCTL_MODE_STANDBY	_IOW('o', 2, __u8)
#define SSL3250A_IOTCL_MODE_TORCH	_IOW('o', 3, __u8)
#define SSL3250A_IOCTL_MODE_FLASH	_IOW('o', 4, __u8)
#define SSL3250A_IOCTL_MODE_LED		_IOW('o', 5, __u8)
#define SSL3250A_IOCTL_STRB		_IOW('o', 6, __u8)
#define SSL3250A_IOCTL_TIMER		_IOW('o', 7, __u8)

#ifdef __KERNEL__
struct ssl3250a_platform_data {
	int config;
	int max_amp_indic;
	int max_amp_torch;
	int max_amp_flash;
	int (*init)(void);
	void (*exit)(void);
	int (*gpio_act)(int);
	int (*gpio_en1)(int);
	int (*gpio_en2)(int);
	int (*gpio_strb)(int);
};
#endif /* __KERNEL__ */

#endif /* __SSL3250A_H__ */

