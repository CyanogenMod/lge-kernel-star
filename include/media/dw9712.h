/*
* dw9712.h
*
* Copyright (c) 2011, NVIDIA, All Rights Reserved.
*
* This file is licensed under the terms of the GNU General Public License
* version 2. This program is licensed "as is" without any warranty of any
* kind, whether express or implied.
*/

#ifndef __DW9712_H__
#define __DW9712_H__

#include <linux/ioctl.h>  /* For IOCTL macros */
#include <linux/i2c.h>

#define DW9712_IOCTL_GET_CONFIG		_IOR('o', 12, struct dw9712_config)
#define DW9712_IOCTL_SET_POSITION	_IOW('o', 13, __u32)
#define DW9712_IOCTL_SET_MODE		_IOW('o', 14, __u32)

struct dw9712_config {
	__u32 settle_time;
	__u32 actuator_range;
	__u32 pos_low;
	__u32 pos_high;
	float focal_length;
	float fnumber;
	float max_aperture;
};

#endif /* __DW9712_H__ */
