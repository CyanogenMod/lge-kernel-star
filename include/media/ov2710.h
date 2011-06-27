/**
 * Copyright (c) 2011 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#ifndef __OV2710_H__
#define __OV2710_H__

#include <linux/ioctl.h>  /* For IOCTL macros */

#define OV2710_IOCTL_SET_MODE		_IOW('o', 1, struct ov2710_mode)
#define OV2710_IOCTL_SET_FRAME_LENGTH	_IOW('o', 2, __u32)
#define OV2710_IOCTL_SET_COARSE_TIME	_IOW('o', 3, __u32)
#define OV2710_IOCTL_SET_GAIN		_IOW('o', 4, __u16)
#define OV2710_IOCTL_GET_STATUS		_IOR('o', 5, __u8)

struct ov2710_mode {
	int xres;
	int yres;
	__u32 frame_length;
	__u32 coarse_time;
	__u16 gain;
};
#ifdef __KERNEL__
struct ov2710_platform_data {
	int (*power_on)(void);
	int (*power_off)(void);

};
#endif /* __KERNEL__ */

#endif  /* __OV2710_H__ */

