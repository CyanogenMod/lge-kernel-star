/*
* Copyright (C) 2010 LGE, Inc.
*
* Contributors:
*      Hyunsu Choi<hyunsu.choi@nvidia.com>
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

#ifndef __DW9712_H__
#define __DW9712_H__

#include <linux/ioctl.h>  /* For IOCTL macros */

#define DW9712_IOCTL_GET_CONFIG _IOR('o', 1, struct dw9712_config)
#define DW9712_IOCTL_SET_POSITION _IOW('o', 2, __u32)
#define DW9712_IOCTL_SET_MODE _IOW('o', 3, __u32)

struct dw9712_config
{
	__u32 settle_time;
	__u32 actuator_range;
	__u32 pos_low;
	__u32 pos_high;
	float focal_length;
	float fnumber;
	float max_aperture;
};

#endif //__DW9712_H__
