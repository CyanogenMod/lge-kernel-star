/*
*
* Contributors:
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

#ifndef __AR0832_FOCUSER_H__
#define __AR0832_FOCUSER_H__

#include <linux/ioctl.h>  /* For IOCTL macros */
#include <linux/i2c.h>

// LGE_CHANGE_S X2_ICS [byun.youngki@lge.com], 2012-05-03, < recover to GB version >
// recover GB version 20120503
/*
struct ar0832_focuser_config
{
	__u32 settle_time;
	__u32 actuator_range;
	__u32 pos_low;
	__u32 pos_high;
	float focal_length;
	float fnumber;
	float max_aperture;
};
*/
// LGE_CHANGE_E X2_ICS [byun.youngki@lge.com], 2012-05-03, < recover to GB version >

enum StereoCameraMode{
	Main = 0,
	/// Sets the stereo camera to stereo mode.
	Stereo = 1,
	/// Only the sensor on the left is on.
	LeftOnly,
	/// Only the sensor on the right is on.
	RightOnly,
	/// Ignore -- Forces compilers to make 32-bit enums.
	StereoCameraMode_Force32 = 0x7FFFFFFF
};

struct ar0832_focuser_info {
	struct i2c_client *i2c_client;
	struct i2c_client *i2c_client_right;
	struct regulator *regulator;
	struct ar0832_focuser_config config;
	__u8 focuser_init_flag;
	
	enum StereoCameraMode camera_mode;
};

//int ar0832_focuser_write_helper(struct ar0832_focuser_info *info, u16 value);
//int ar0832_focuser_write(struct i2c_client *client, u16 value);


#endif //__AR0832_FOCUSER_H__
