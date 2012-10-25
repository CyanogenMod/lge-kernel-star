/**
 * Copyright (c) 2008 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#ifndef __IMX073_H__
#define __IMX073_H__

#include <linux/ioctl.h>  /* For IOCTL macros */

#define IMX073_IOCTL_SET_MODE		_IOW('o', 1, struct imx073_mode)
#define IMX073_IOCTL_SET_FRAME_LENGTH	_IOW('o', 2, __u32)
#define IMX073_IOCTL_SET_COARSE_TIME	_IOW('o', 3, __u32)
#define IMX073_IOCTL_SET_GAIN		_IOW('o', 4, __u16)
#define IMX073_IOCTL_GET_STATUS		_IOR('o', 5, __u8)
//#define IMX073_IOCTL_TEST_PATTERN	_IOW('o', 7, enum ov5650_test_pattern)

//enum ov5650_test_pattern {
//	TEST_PATTERN_NONE,
//	TEST_PATTERN_COLORBARS,
//	TEST_PATTERN_CHECKERBOARD
//};

struct imx073_mode {
	int xres;
	int yres;
	__u32 frame_length;
	__u32 coarse_time;
	__u16 gain;
};
#ifdef __KERNEL__
struct imx073_platform_data {
	int (*power_on)(void);
	int (*power_off)(void);

};
#endif /* __KERNEL__ */

#endif  /* __IMX073_H__ */

