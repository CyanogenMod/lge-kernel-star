/*
 * Copyright (C) 2010 Motorola, Inc.
 * Copyright (C) 2011 NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */

#ifndef __OV5650_H__
#define __OV5650_H__

#include <linux/ioctl.h>  /* For IOCTL macros */

#define OV5650_IOCTL_SET_MODE		_IOW('o', 1, struct ov5650_mode)
#define OV5650_IOCTL_SET_FRAME_LENGTH	_IOW('o', 2, __u32)
#define OV5650_IOCTL_SET_COARSE_TIME	_IOW('o', 3, __u32)
#define OV5650_IOCTL_SET_GAIN		_IOW('o', 4, __u16)
#define OV5650_IOCTL_GET_STATUS		_IOR('o', 5, __u8)
#define OV5650_IOCTL_TEST_PATTERN	_IOW('o', 7, enum ov5650_test_pattern)
#define OV5650_IOCTL_SET_CAMERA_MODE	_IOW('o', 10, __u32)

enum ov5650_test_pattern {
	TEST_PATTERN_NONE,
	TEST_PATTERN_COLORBARS,
	TEST_PATTERN_CHECKERBOARD
};

struct ov5650_mode {
	int xres;
	int yres;
	__u32 frame_length;
	__u32 coarse_time;
	__u16 gain;
};
#ifdef __KERNEL__
struct ov5650_platform_data {
	int (*power_on)(void);
	int (*power_off)(void);

};
#endif /* __KERNEL__ */

#endif  /* __OV5650_H__ */

