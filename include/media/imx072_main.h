/*
* Copyright (C) 2010 LGE, Inc.
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

#ifndef __IMX072_MAIN_H__
#define __IMX072_MAIN_H__

#include <linux/ioctl.h> /* For IOCTL macros */

#define IMX072_IOCTL_SET_MODE				_IOW('o', 1, struct imx072_mode)
#define IMX072_IOCTL_SET_FRAME_LENGTH	_IOW('o', 2, __u32)
#define IMX072_IOCTL_SET_COARSE_TIME		_IOW('o', 3, __u32)
#define IMX072_IOCTL_SET_GAIN				_IOW('o', 4, struct imx072_gain)
#define IMX072_IOCTL_GET_STATUS			_IOR('o', 5, __u8)
#define IMX072_IOCTL_GET_OTP				_IOR('o', 6, struct imx072_otp_data)
#define IMX072_IOCTL_TEST_PATTERN			_IOW('o', 7, enum imx072_test_pattern)
#define IMX072_IOCTL_SET_POWER_ON		_IOW('o', 10, __u32)
#define IMX072_IOCTL_SET_SENSOR_REGION	_IOW('o', 11, struct imx072_stereo_region)
#define IMX072_IOCTL_SENSOR_RESET		_IOW('o', 12, int)
enum imx072_test_pattern {
	TEST_PATTERN_NONE,
	TEST_PATTERN_COLORBARS,
	TEST_PATTERN_CHECKERBOARD
};

struct imx072_otp_data {
	/* Only the first 5 bytes are actually used. */
	__u8 sensor_serial_num[6];
	__u8 part_num[8];
	__u8 lens_id[1];
	__u8 manufacture_id[2];
	__u8 factory_id[2];
	__u8 manufacture_date[9];
	__u8 manufacture_line[2];

	__u32 module_serial_num;
	__u8 focuser_liftoff[2];
	__u8 focuser_macro[2];
	__u8 reserved1[12];
	__u8 shutter_cal[16];
	__u8 reserved2[183];

	/* Big-endian. CRC16 over 0x00-0x41 (inclusive) */
	__u16 crc;
	__u8 reserved3[3];
	__u8 auto_load[2];
} __attribute__ ((packed));

struct imx072_gain{
	__u16 AnalogGain;
	__u16 DigitalGain_Upper;
	__u16 DigitalGain_Lower;
};

struct imx072_mode {
	int xres;
	int yres;
	__u32 frame_length;
	__u32 coarse_time;
	struct imx072_gain gain;
};
struct imx072_point{
	int x;
	int y;
};
struct imx072_reg {
	u16 addr;
	u16 val;
};
struct imx072_stereo_region{
	int	camer_index;
	struct imx072_point image_start;
	struct imx072_point image_end;
};

#ifdef __KERNEL__
struct imx072_platform_data {
	int (*power_on)(void);
	int (*power_off)(void);

};
#endif /* __KERNEL__ */

#endif
