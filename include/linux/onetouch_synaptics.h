/*
 * include/linux/onetouch_synaptics.h 
 *
 * Copyright (C) 2012 LGE, Inc.
 *
 * writer: jingyeong.noh@lge.com 
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef _LINUX_ONETOUCH_SYNAPTICS_H
#define _LINUX_ONETOUCH_SYNAPTICS_H

#define ONETOUCH_SYNAPTICS_NAME "star_onetouch"
#define ONETOUCH_SYNAPTICS_ADDR 0x2C

struct star_onetouch_synaptics_platform_data {
	u16 gpio;	
	int (*power)(char* reg_id, int on);	
	unsigned long irqflags;
};

#endif /* _LINUX_ONETOUCH_SYNAPTICS_H*/

