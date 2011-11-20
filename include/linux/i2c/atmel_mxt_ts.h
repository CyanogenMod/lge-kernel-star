/*
 * Atmel maXTouch Touchscreen driver
 *
 * Copyright (C) 2010 Samsung Electronics Co.Ltd
 * Copyright (C) 2011 Atmel Corporation
 * Copyright (C) 2011 NVIDIA Corporation
 * Author: Joonyoung Shim <jy0922.shim@samsung.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef __LINUX_ATMEL_MXT_TS_H
#define __LINUX_ATMEL_MXT_TS_H

#include <linux/types.h>

/*
 * Atmel I2C addresses
 */
#define	MXT224_I2C_ADDR1	0x4A
#define	MXT224_I2C_ADDR2	0x4B
#define	MXT1386_I2C_ADDR1	0x4C
#define	MXT1386_I2C_ADDR2	0x4D
#define	MXT1386_I2C_ADDR3	0x5A
#define	MXT1386_I2C_ADDR4	0x5B

/* Orient */
#define MXT_NORMAL		0x0
#define MXT_DIAGONAL		0x1
#define MXT_HORIZONTAL_FLIP	0x2
#define MXT_ROTATED_90_COUNTER	0x3
#define MXT_VERTICAL_FLIP	0x4
#define MXT_ROTATED_90		0x5
#define MXT_ROTATED_180		0x6
#define MXT_DIAGONAL_COUNTER	0x7

/* The platform data for the Atmel maXTouch touchscreen driver */
struct mxt_platform_data {
	const u8 *config;
	size_t config_length;

	unsigned int x_line;
	unsigned int y_line;
	unsigned int x_size;
	unsigned int y_size;
	unsigned int blen;
	unsigned int threshold;
	unsigned int voltage;
	unsigned char orient;
	unsigned long irqflags;
	u8(*read_chg) (void);
	unsigned long config_crc;
	unsigned int actv_cycle_time;
	unsigned int idle_cycle_time;
};

#endif /* __LINUX_ATMEL_MXT_TS_H */
