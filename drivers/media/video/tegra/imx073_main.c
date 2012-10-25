/*
* imx073_main.c - sony imx073 8M Bayer type sensor driver
*
* Copyright (C) 2010 LG Inc.
*
* Contributors:
*	 
*
*  
*
* This file is licensed under the terms of the GNU General Public License
* version 2. This program is licensed "as is" without any warranty of any
* kind, whether express or implied.
*/

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <media/imx073_main.h>
#include <mach/gpio-names.h>
#include <mach/hardware.h>
#include <linux/gpio.h>
//LGE_CHANGE_S [sungmo.yang@lge.com] 2011-01-17 [LGE_AP20], TEST!
#include <mach/lp8720.h>
// LGE_CHANGE_E [sungmo.yang@lge.com] 2011-01-17 [LGE_AP20], TEST! 

// LGE_CHANGE_S [sungmo.yang@lge.com] 2011-01-17 [LGE_AP20], for IMX073 Unit test
#define imx073_UnitTest
// LGE_CHANGE_E [sungmo.yang@lge.com] 2011-01-17 [LGE_AP20], for IMX073 Unit test

#define imx073_TABLE_WAIT_MS 		0
#define imx073_TABLE_END 			1
#define imx073_MAX_RETRIES 			3

#define imx073_MODE_3264x2448_xres	(3264)
#define imx073_MODE_3264x2448_yres	(2448)
#define imx073_MODE_3264x1224_xres	(3264)
#define imx073_MODE_3264x1224_yres	(1224)
#define imx073_MODE_2720x1530_xres	(2720)
#define imx073_MODE_2720x1530_yres	(1530)
#define imx073_MODE_1920x1080_xres	(1920)
#define imx073_MODE_1920x1080_yres	(1080)


#define REG_ADDR_1_FRAME_LENGTH_LINES		0x0340
#define REG_ADDR_2_FRAME_LENGTH_LINES		0x0341
#define REG_ADDR_1_COARSE_TIME_REGS			0x0202
#define REG_ADDR_2_COARSE_TIME_REGS			0x0203
#define REG_ADDR_GET_GAIN_REG				0x0205
#define REG_ADDR_GROUPED_PARAMETER_HOLD	0x0104




enum CameraMode{
	Main = 0
};

//TODO[LGE_AP20] add resolution mode register enum
// 3264X2448
// 3264X1224
// 2720X1530
// 1920X1080
enum {
	imx073_MODE_3264x2448,
	imx073_MODE_3264x1224,
	imx073_MODE_2720x1530,
	imx073_MODE_1920x1080
};

static enum CameraMode camera_mode;
static struct imx073_info *info=NULL;

// LGE_CHANGE_S [sungmo.yang@lge.com] 2011-01-13, [LGE_AP20] fix GPIO PIN 
static int Imx073_RESET			= TEGRA_GPIO_PD2;  
static int Imx073VT_PWRDN 		= TEGRA_GPIO_PD5;  
static int Imx073Focuser_ENABLE	= TEGRA_GPIO_PT4; // 8M_CAM_VCM_EN
// LGE_CHANGE_S [sungmo.yang@lge.com] 2011-01-13, [LGE_AP20] fix GPIO PIN 

// Device Driver setting related Functions -----------------------------------------




static const struct i2c_device_id imx073_id[] = {
	{ "imx073", 0 },
	{ },
};




//TODO[LGE_AP20] check info structure is ok?!
struct imx073_info {
	int mode;
	struct i2c_client *i2c_client;
	struct imx073_platform_data *pdata;
};


static struct imx073_reg mode_start[] = {
	{0x0103, 0x01},//reset 0
	{imx073_TABLE_WAIT_MS, 0x0001},

//	Stand-by OFF Sequence
//		Power ON
//		Input INCK
//		XCLR OFF
//		PLL SettingINCK=24MHz, 27times
//		Address	value	

//	Global Setting
	{0x0307, 0x24},	// PLL multiplier
	{0x302B, 0x38},	// PLL oscillation stable wait timer setting
	{0x30E5, 0x04},	// Output Method Switching register [CSI2 Enable]
	{0x3300, 0x00},	// Select subLVDS and Mipi exclusively [Mipi operation]
	{0x0101, 0x03},	
	{0x0202, 0x03},
	{0x0203, 0xe8},
	
	{0x300A, 0x80},
	{0x3014, 0x08},
	{0x3015, 0x37},
	{0x3017, 0x60},
	{0x301C, 0x01},
	{0x3031, 0x28},
	{0x3040, 0x00},
	{0x3041, 0x60},
	{0x3047, 0x10},
	{0x3051, 0x24},
	{0x3053, 0x34},
	{0x3055, 0x3B},
	{0x3057, 0xC0},
	{0x3060, 0x30},
	{0x3065, 0x00},
	{0x30A1, 0x03},
	{0x30A3, 0x01},
	{0x30AA, 0x88},
	{0x30AB, 0x1C},
	{0x30B0, 0x32},
	{0x30B2, 0x83},
	{0x30D3, 0x04},
	{0x310C, 0xE9},
	{0x310D, 0x00},	
	{0x316B, 0x14},
	{0x316D, 0x3B},
	{0x31A4, 0xD8},
	{0x31A6, 0x17},
	{0x31AC, 0xCF},
	{0x31AE, 0xF1},
	{0x31B4, 0xD8},
	{0x31B6, 0x17},
	{0x3302, 0x0A},
	{0x3303, 0x09},
	{0x3304, 0x05},
	{0x3305, 0x04},
	{0x3306, 0x15},
	{0x3307, 0x03},
	{0x3308, 0x13},
	{0x3309, 0x05},
	{0x330A, 0x0B},
	{0x330B, 0x04},
	{0x330C, 0x0B},
	{0x330D, 0x06},
	{0x330E, 0x01},

// LGE_CHANGE_S [sungmo.yang@lge.com] 2011-01-14, [LGE_AP20] Resolution dependent setting? remove? Is it default resolution?
//TODO[LGE_AP20] check resolution dependent register setting!
//	{0x0340, 0x04},
//	{0x0341, 0xE6},
//	{0x034C, 0x0C},
//	{0x034D, 0xD0},
//	{0x034E, 0x04},
//	{0x034F, 0xD0},
//	{0x0381, 0x01},
//	{0x0383, 0x01},
//	{0x0385, 0x01},
//	{0x0387, 0x03},
//	{0x3016, 0x06},
//	{0x0100, 0x01},
// LGE_CHANGE_E [sungmo.yang@lge.com] 2011-01-14, [LGE_AP20] Resolution dependent setting? remove?

	{imx073_TABLE_END, 0x0000}
};



static struct imx073_reg mode_3264x2448[] = {
	{0x0340, 0x09},
	{0x0341, 0xCE},
	{0x034C, 0x0C},
	{0x034D, 0xD0},
	{0x034E, 0x09},
	{0x034F, 0xA0},
	{0x0381, 0x01},
	{0x0383, 0x01},
	{0x0385, 0x01},
	{0x0387, 0x01},
	{0x3001, 0x00},
	{0x3016, 0x06},
	{0x30E8, 0x06},
	{0x3301, 0x00},
	{imx073_TABLE_END, 0x0000}
};


static struct imx073_reg mode_3264x1224[] = {
	{0x0340, 0x04},
	{0x0341, 0xE6},
	{0x034C, 0x0C},
	{0x034D, 0xD0},
	{0x034E, 0x04},
	{0x034F, 0xD0},
	{0x0381, 0x01},
	{0x0383, 0x01},
	{0x0385, 0x01},
	{0x0387, 0x03},
	{0x3001, 0x00},
	{0x3016, 0x46},
	{0x30E8, 0x06},
	{0x3301, 0x00},
	{imx073_TABLE_END, 0x0000}
};


static struct imx073_reg mode_2720x1530[] = {
	{0x0340, 0x06},
	{0x0341, 0x1f},
	{0x0344, 0x01},
	{0x0345, 0x18},
	{0x0346, 0x01},
	{0x0347, 0xd5},
	{0x0348, 0x0b},
	{0x0349, 0xb7},
	{0x034A, 0x07},
	{0x034B, 0xce},
	{0x034C, 0x0a},	// x_out size, [0x0aa0:2720]
	{0x034D, 0xa0},
	{0x034E, 0x05},
	{0x034F, 0xfa},
	{0x0381, 0x01},
	{0x0383, 0x01},
	{0x0385, 0x01},
	{0x0387, 0x01},
	{0x3001, 0x00},
	{0x3016, 0x06},
	{0x30E8, 0x06},
	{0x3301, 0x00},
	{imx073_TABLE_END, 0x0000}
};



static struct imx073_reg mode_1920x1080[] = {
	{0x0340, 0x04},
	{0x0341, 0xE6},
	{0x0344, 0x02},
	{0x0345, 0xA8},
	{0x0346, 0x02},
	{0x0347, 0xB5},
	{0x0348, 0x0A},
	{0x0349, 0x27},
	{0x034A, 0x06},
	{0x034B, 0xEC},
	{0x034C, 0x07},	// x_out size [0x0780:1920]
	{0x034D, 0x80},
	{0x034E, 0x04},
	{0x034F, 0x38},
	{0x0381, 0x01},
	{0x0383, 0x01},
	{0x0385, 0x01},
	{0x0387, 0x01},
	{0x3001, 0x00},
	{0x3016, 0x06},
	{0x30E8, 0x06},
	{0x3301, 0x00},
	{imx073_TABLE_END, 0x0000}
};


static struct imx073_reg mode_end[] = {

	{0x0100, 0x01}, /* Data Stream On*/
	/*	{FAST_SETMODE_END, 0}, */
	{imx073_TABLE_END, 0x0000}
};


static struct imx073_reg *mode_table[] = {
	[imx073_MODE_3264x2448] = mode_3264x2448,
	[imx073_MODE_3264x1224] = mode_3264x1224,
	[imx073_MODE_2720x1530] = mode_2720x1530,
	[imx073_MODE_1920x1080] = mode_1920x1080
};



// imx073_write_reg_helper Package
static int imx073_write_reg(struct i2c_client *client, u16 addr, u8 val)
{
	int err;
	struct i2c_msg msg;
	unsigned char data[3];
	int retry = 0;

	if (!client->adapter)
		return -ENODEV;

	data[0] = (u8) (addr >> 8);;
	data[1] = (u8) (addr & 0xff);
	data[2] = (u8) (val & 0xff);

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = 3;
	msg.buf = data;
//s32 i2c_write_block_data(client, u8 addr_flags, 1, addr, &val)
	do {
		err = i2c_transfer(client->adapter, &msg, 1);
		if (err >0 )
			return 0;
		retry++;
		pr_err("imx073: i2c transfer failed, retrying %x %x\n",
			addr, val);
		msleep(3);
	} while(retry <imx073_MAX_RETRIES);

	return err;
}

static int imx073_write_reg_helper(struct imx073_info *info, u16 addr, u8 val)
{
	int ret;
	switch(camera_mode){
		case Main:
			ret = imx073_write_reg(info->i2c_client, addr, val);
			break;
		default :
			return -1;
	}
	return ret;
}


static int imx073_write_table(struct imx073_info *info,
			const struct imx073_reg table[],
			const struct imx073_reg override_list[],
			int num_override_regs)
{
	int err;
	const struct imx073_reg *next;
	int i;
	u16 val;

	for (next = table; next->addr != imx073_TABLE_END; next++) {
		if (next->addr == imx073_TABLE_WAIT_MS) {
			msleep(next->val);
			continue;
		}

		val = next->val;

		/* When an override list is passed in, replace the reg */
		/* value to write if the reg is in the list            */
		if (override_list) {
			for (i = 0; i < num_override_regs; i++) {
				if (next->addr == override_list[i].addr) {
					val = override_list[i].val;
					break;
				}
			}
		}

		err = imx073_write_reg_helper(info, next->addr, val);
		if (err)
			return err;
	}
	return 0;
}





/* 2 regs to program frame length */
static inline void imx073_get_frame_length_regs(struct imx073_reg *regs,
						u32 frame_length)
{
//LGE_CHANGE_S [sungmo.yang@lge.com] 2011-01-14 [LGE_AP20] frame_length_lines [0x0320 -> REG_ADDR_FRAME_LENGTH_LINES]
//	regs->addr = 0x0320;
//	regs->val = (frame_length >> 8) & 0xff;
//	(regs + 1)->addr = 0x0321;
//	(regs + 1)->val = (frame_length) & 0xff;	
	regs->addr = REG_ADDR_1_FRAME_LENGTH_LINES;
	regs->val = (frame_length >> 8) & 0xff;
	(regs + 1)->addr = REG_ADDR_2_FRAME_LENGTH_LINES;
	(regs + 1)->val = (frame_length) & 0xff;
	//LGE_CHANGE_E [sungmo.yang@lge.com] 2011-01-14 [LGE_AP20] frame_length_lines [0x0320 -> REG_ADDR_FRAME_LENGTH_LINES]
}



/* 2 regs to program coarse time */
static inline void imx073_get_coarse_time_regs(struct imx073_reg *regs,	
						u32 coarse_time)
{

// LGE_CHANGE_S [sungmo.yang@lge.com] 2011-01-14, [LGE_AP20] coarse_integration_time [0x0202 -> REG_ADDR_COARSE_TIME_REGS]
//	regs->addr = 0x0202;
//	regs->val = (coarse_time >> 8) & 0xff;
//	(regs + 1)->addr = 0x0203;
//	(regs + 1)->val = (coarse_time) & 0xff;
	regs->addr = REG_ADDR_1_COARSE_TIME_REGS;
	regs->val = (coarse_time >> 8) & 0xff;
	(regs + 1)->addr = REG_ADDR_2_COARSE_TIME_REGS;
	(regs + 1)->val = (coarse_time) & 0xff;
// LGE_CHANGE_E [sungmo.yang@lge.com] 2011-01-14, [LGE_AP20] coarse_integration_time [0x0202 -> REG_ADDR_COARSE_TIME_REGS]



}




/* 1 reg to program gain */
static inline void imx073_get_gain_reg(struct imx073_reg *regs, u16 gain)
{
// LGE_CHANGE_S [sungmo.yang@lge.com] 2011-01-14, [LGE_AP20] analogue_gain_code_global [0x0205 -> REG_ADDR_GET_GAIN_REG]
//	regs->addr = 0x0205;
//	regs->val = gain;
	regs->addr = REG_ADDR_GET_GAIN_REG;
	regs->val = gain;
// LGE_CHANGE_E [sungmo.yang@lge.com] 2011-01-14, [LGE_AP20] analogue_gain_code_global 

}



static int imx073_set_frame_length(struct imx073_info *info, u32 frame_length)
{
	struct imx073_reg reg_list[2];
	int i = 0;
	int ret;

	imx073_get_frame_length_regs(reg_list, frame_length);
// LGE_CHANGE_S [sungmo.yang@lge.com] 2011-01-14, [LGE_AP20] REG_ADDR_GROUPED_PARAMETER_HOLD [0x0104 -> REG_ADDR_GROUPED_PARAMETER_HOLD]
	//ret = imx073_write_reg_helper(info, 0x0104, 0x01);
	ret = imx073_write_reg_helper(info, REG_ADDR_GROUPED_PARAMETER_HOLD, 0x01);
// LGE_CHANGE_E [sungmo.yang@lge.com] 2011-01-14, [LGE_AP20] REG_ADDR_GROUPED_PARAMETER_HOLD [0x0104 -> REG_ADDR_GROUPED_PARAMETER_HOLD]

	if (ret)
		return ret;

	for (i = 0; i < 2; i++)	{
		ret = imx073_write_reg_helper(info, reg_list[i].addr,
			reg_list[i].val);
		if (ret)
			return ret;
	}
// LGE_CHANGE_S [sungmo.yang@lge.com] 2011-01-14, [LGE_AP20] REG_ADDR_GROUPED_PARAMETER_HOLD [0x0104 -> REG_ADDR_GROUPED_PARAMETER_HOLD]
//	ret = imx073_write_reg_helper(info, 0x0104, 0x00);
	ret = imx073_write_reg_helper(info, REG_ADDR_GROUPED_PARAMETER_HOLD, 0x00);
// LGE_CHANGE_E [sungmo.yang@lge.com] 2011-01-14, [LGE_AP20] REG_ADDR_GROUPED_PARAMETER_HOLD [0x0104 -> REG_ADDR_GROUPED_PARAMETER_HOLD]	
	if (ret)
		return ret;

	return 0;
}




static int imx073_set_coarse_time(struct imx073_info *info, u32 coarse_time) // 2
{
	int ret;

	struct imx073_reg reg_list[2];
	int i = 0;

	imx073_get_coarse_time_regs(reg_list, coarse_time);
// LGE_CHANGE_S [sungmo.yang@lge.com] 2011-01-14, [LGE_AP20] REG_ADDR_GROUPED_PARAMETER_HOLD [0x0104 -> REG_ADDR_GROUPED_PARAMETER_HOLD]
	//	ret = imx073_write_reg_helper(info, 0x0104, 0x01);
	ret = imx073_write_reg_helper(info, REG_ADDR_GROUPED_PARAMETER_HOLD, 0x01);
// LGE_CHANGE_E [sungmo.yang@lge.com] 2011-01-14, [LGE_AP20] REG_ADDR_GROUPED_PARAMETER_HOLD [0x0104 -> REG_ADDR_GROUPED_PARAMETER_HOLD]

	if (ret)
		return ret;

	for (i = 0; i < 2; i++)	{
		ret = imx073_write_reg_helper(info, reg_list[i].addr,
			reg_list[i].val);
		if (ret)
			return ret;
	}

// LGE_CHANGE_S [sungmo.yang@lge.com] 2011-01-14, [LGE_AP20] REG_ADDR_GROUPED_PARAMETER_HOLD [0x0104 -> REG_ADDR_GROUPED_PARAMETER_HOLD]
	//	ret = imx073_write_reg_helper(info, 0x0104, 0x00);
	ret = imx073_write_reg_helper(info, REG_ADDR_GROUPED_PARAMETER_HOLD, 0x00);
// LGE_CHANGE_E [sungmo.yang@lge.com] 2011-01-14, [LGE_AP20] REG_ADDR_GROUPED_PARAMETER_HOLD [0x0104 -> REG_ADDR_GROUPED_PARAMETER_HOLD]

	if (ret)
		return ret;

	return 0;
}


//TODO[LGE_AP20] check imx073_get_status(), I think it doesn't return device driver's status.
static int imx073_get_status(struct imx073_info *info, u8 *status)
{
	int err=0;

	*status = 0;

//	err = imx073_read_reg(info->i2c_client, 0x002, status);  // which status do you want?
	pr_info("%s: %u %d\n", __func__, *status, err);
	return err;
}


static int imx073_read_reg(struct i2c_client *client, u16 addr, u8 *val)
{
	int err;
	struct i2c_msg msg[2];
	unsigned char data[3];

	if (!client->adapter)
		return -ENODEV;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf = data;

	/* high byte goes out first */
	data[0] = (u8) (addr >> 8);
	data[1] = (u8) (addr & 0xff);

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 1;
	msg[1].buf = data + 2;
	err = i2c_transfer(client->adapter, msg, 2);

	if (err != 2)
	{
		pr_info("imx073_read_reg, err : %d ", err);
		return -EINVAL;
	}

	*val = data[2];

	return err;
}




static int imx073_set_gain(struct imx073_info *info, u16 gain)
{
	int ret;
	struct imx073_reg reg_list;

	imx073_get_gain_reg(&reg_list, gain);

// LGE_CHANGE_S [sungmo.yang@lge.com] 2011-01-14, [LGE_AP20] REG_ADDR_GROUPED_PARAMETER_HOLD [0x0104 -> REG_ADDR_GROUPED_PARAMETER_HOLD]
	//	ret = imx073_write_reg_helper(info, 0x0104, 0x01);
	ret = imx073_write_reg_helper(info, REG_ADDR_GROUPED_PARAMETER_HOLD, 0x01);
// LGE_CHANGE_E [sungmo.yang@lge.com] 2011-01-14, [LGE_AP20] REG_ADDR_GROUPED_PARAMETER_HOLD [0x0104 -> REG_ADDR_GROUPED_PARAMETER_HOLD]

	if (ret)
		return ret;

	ret = imx073_write_reg_helper(info, reg_list.addr, reg_list.val);
	if (ret)
		return ret;

// LGE_CHANGE_S [sungmo.yang@lge.com] 2011-01-14, [LGE_AP20] REG_ADDR_GROUPED_PARAMETER_HOLD [0x0104 -> REG_ADDR_GROUPED_PARAMETER_HOLD]
	//	ret = imx073_write_reg_helper(info, 0x0104, 0x00);
	ret = imx073_write_reg_helper(info, REG_ADDR_GROUPED_PARAMETER_HOLD, 0x00);
// LGE_CHANGE_E [sungmo.yang@lge.com] 2011-01-14, [LGE_AP20] REG_ADDR_GROUPED_PARAMETER_HOLD [0x0104 -> REG_ADDR_GROUPED_PARAMETER_HOLD]

	if (ret)
		return ret;

	return 0;
}



static int imx073_set_mode(struct imx073_info *info, struct imx073_mode *mode)
{
	int sensor_mode;
	int err;
	int i;
	int ret;
	struct imx073_reg reg_list[5];

// pr_info

	if(camera_mode != Main){
		//TODO[LGE_AP20] check default resolution setting
		sensor_mode = imx073_MODE_2720x1530;
	}
	else{
		if (mode->xres 	== imx073_MODE_3264x2448_xres && mode->yres == imx073_MODE_3264x2448_yres) 
			sensor_mode = imx073_MODE_3264x2448;
		else if (mode->xres == imx073_MODE_3264x1224_xres && mode->yres == imx073_MODE_3264x1224_yres)
			sensor_mode = imx073_MODE_3264x1224;
		else if (mode->xres == imx073_MODE_2720x1530_xres && mode->yres == imx073_MODE_2720x1530_yres)
			sensor_mode = imx073_MODE_2720x1530;
		else if (mode->xres == imx073_MODE_1920x1080_xres && mode->yres == imx073_MODE_1920x1080_yres)
			sensor_mode = imx073_MODE_1920x1080;
		else {
			pr_err("%s: invalid resolution supplied to set mode %d %d\n",
					__func__, mode->xres, mode->yres);
			return -EINVAL;
		}
	}
	/* get a list of override regs for the asking frame length,	*/
	/* coarse integration time, and gain.				*/
//	imx072_get_frame_length_regs(reg_list, mode->frame_length);
//	imx072_get_coarse_time_regs(reg_list + 2, mode->coarse_time);
//	imx072_get_gain_reg(reg_list + 4, mode->gain);

	err = imx073_write_table(info, mode_start, NULL, 0);

	if (err)
		return err;
	err = imx073_write_table(info, mode_table[sensor_mode],
		reg_list, 5);
	if (err)
		return err;
	imx073_get_frame_length_regs(reg_list, mode->frame_length);
	for (i = 0; i < 2; i++)	{
		ret = imx073_write_reg_helper(info, reg_list[i].addr,
			reg_list[i].val);
		if (ret)
			return ret;
	}
	imx073_get_coarse_time_regs(reg_list + 2, mode->coarse_time);
	for (i = 0; i < 2; i++)	{
		ret = imx073_write_reg_helper(info, reg_list[i+2].addr,
			reg_list[i].val);
		if (ret)
			return ret;
	}
	imx073_get_gain_reg(reg_list + 4, mode->gain);
	for (i = 0; i < 2; i++)	{
		ret = imx073_write_reg_helper(info, reg_list[i+4].addr,
			reg_list[i].val);
		if (ret)
			return ret;
	}

	err = imx073_write_table(info, mode_end, NULL, 0);
	if (err)
		return err;
	info->mode = sensor_mode;
	return 0;
}




//-----------------------------------------------------------------------------------------
static int imx073_open(struct inode *inode, struct file *file)
{
	pr_info("%s:++++\n", __func__);
	file->private_data = info;
	pr_info("%s:---\n", __func__);

//	imx073_get_status(info, &status);
	return 0;
}


static void imx073_cam_pin_set(void)
{
	gpio_request(Imx073_RESET, "Cam_Reset");
	gpio_request(Imx073Focuser_ENABLE, "Cam_Power_Down");
	tegra_gpio_enable(Imx073_RESET);
	tegra_gpio_enable(Imx073Focuser_ENABLE);
	gpio_direction_output(Imx073_RESET,1);
	gpio_direction_output(Imx073Focuser_ENABLE,1);
	gpio_set_value(Imx073_RESET,0);
	gpio_set_value(Imx073Focuser_ENABLE,0);
	mdelay(10);
	gpio_set_value(Imx073_RESET,1);
	mdelay(10);
	gpio_set_value(Imx073Focuser_ENABLE,1);
	mdelay(10);
}





static int imx073_power_on(void)
{

	pr_info("%s: start\n", __func__);
// LGE_CHANGE_S [sungmo.yang@lge.com] 	2011-01-18,[LGE_AP20] comment out until it makes
	star_cam_Main_power_on();
// LGE_CHANGE_E [sungmo.yang@lge.com]	2011-01-18,[LGE_AP20] comment out until it makes

	switch(camera_mode){
		case Main:
// LGE_CHANGE_S [sungmo.yang@lge.com]	2011-01-14,[LGE_AP20] imx073_cam_pin_set() fix
			imx073_cam_pin_set(); 
// LGE_CHANGE_E [sungmo.yang@lge.com]	2011-01-14,[LGE_AP20] imx073_cam_pin_set() fix
			break;
		default :
			return -1;
	}

	pr_info("%s: end\n", __func__);
	return 0;
}


static int imx073_release(struct inode *inode, struct file *file)
{
	pr_info("%s\n", __func__);

	file->private_data = NULL;

	switch(camera_mode){
		case Main:
//LGE_CHANGE_S [sungmo.yang@lge.com] 2011-01-18 [LGE_AP20], fix GPIO pin
			gpio_free(Imx073_RESET);
			gpio_free(Imx073Focuser_ENABLE);
//LGE_CHANGE_E [sungmo.yang@lge.com] 2011-01-18 [LGE_AP20], fix GPIO pin		
			break;
		default :
			return -1;
	}
// LGE_CHANGE_S [sungmo.yang] 2011-01-17 [LGE_AP20], Camera unit test functions
	star_cam_power_off();
// LGE_CHANGE_E [sungmo.yang] 2011-01-17 [LGE_AP20], Camera unit test functions

	camera_mode = Main;
	return 0;
}


//TODO[LGE_AP20] add Camera_sensor_ioctl.h
//static int imx072_ioctl(struct inode *inode, struct file *file,
//			unsigned int cmd, unsigned long arg)
static int imx073_ioctl(struct file *file,
			unsigned int cmd, unsigned long arg)
{
	int err;
	struct imx073_info *info = file->private_data;

	pr_info("\nimx073_ioctl : cmd = %d\n", cmd);

	switch (cmd) {
	case IMX073_IOCTL_SET_POWER_ON:
		return imx073_power_on();
	case IMX073_IOCTL_SET_MODE:
	{
		struct imx073_mode mode;
		if (copy_from_user(&mode,
					(const void __user *)arg,
					sizeof(struct imx073_mode))) {
			pr_info("%s %d\n", __func__, __LINE__);
			return -EFAULT;
		}

		return imx073_set_mode(info, &mode);
	}
	case IMX073_IOCTL_SET_FRAME_LENGTH:
		return imx073_set_frame_length(info, (u32)arg);
	case IMX073_IOCTL_SET_COARSE_TIME:
		return imx073_set_coarse_time(info, (u32)arg);
	case IMX073_IOCTL_SET_GAIN:
		return imx073_set_gain(info, (u16)arg);
	case IMX073_IOCTL_GET_STATUS:
	{
		u8 status;

		err = imx073_get_status(info, &status);
		if (err)
			return err;
		if (copy_to_user((void __user *)arg, &status,
				 2)) {
			pr_info("%s %d\n", __func__, __LINE__);
			return -EFAULT;
		}
		return 0;
	}		
	default:
		return -EINVAL;
	}
	return 0;
}



static const struct file_operations imx073_fileops = {
	.owner = THIS_MODULE,
	.open = imx073_open,
///	.ioctl = imx073_ioctl,
	.unlocked_ioctl = imx073_ioctl,
	.release = imx073_release
};


static struct miscdevice imx073_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "imx073",
	.fops = &imx073_fileops,
	.mode = S_IRWXUGO
};

// LGE_CHANGE_S [sungmo.yang@lge.com] 2011-01-18, [LGE_AP20] UnitTest 
#if defined (imx073_UnitTest) 
#include "imx073_main_UnitTest.c" // 815
#endif // end imx073_UnitTest
// LGE_CHANGE_E [sungmo.yang@lge.com] 2011-01-18, [LGE_AP20] UnitTest 

static int imx073_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int err;
	pr_info("imx073: probing sensor.\n");

	info = kzalloc(sizeof(struct imx073_info), GFP_KERNEL);
	if (!info) {
		pr_err("imx073: Unable to allocate memory!\n");
		return -ENOMEM;
	}

	err = misc_register(&imx073_device);
	if (err) {
		pr_err("imx073: Unable to register misc device!\n");
		kfree(info);
		return err;
	}

	info->pdata = client->dev.platform_data;
	info->i2c_client = client;

	i2c_set_clientdata(client, info);
	
// LGE_CHANGE_S [sungmo.yang] 2011-01-17 [LGE_AP20], Camera unit test functions
#if defined (imx073_UnitTest)
	pr_info("imx073 i2c_client address : %x \n", info->i2c_client->addr);
	pr_info("imx073 i2c_client address : %x \n", client->addr);

	// sysfs
	err = device_create_file(&client->dev, &dev_attr_Imx073);
	if (err) {
		pr_info("imx073: Unable to allocate to make sysfs!\n");
		return err;		
	}
#endif 
	return 0;
// LGE_CHANGE_E [sungmo.yang] 2011-01-17 [LGE_AP20], Camera unit test functions
}


static int imx073_remove(struct i2c_client *client)
{
	struct imx073_info *info;
	info = i2c_get_clientdata(client);
	misc_deregister(&imx073_device);
	kfree(info);
	return 0;
}



static struct i2c_driver imx073_i2c_driver = {
	.probe = imx073_probe,
	.remove = imx073_remove,
	.id_table = imx073_id,
	.driver = {
		.name = "imx073Main",
		.owner = THIS_MODULE,
	},
};

static int __init imx073_init(void)
{
	pr_info("imx073 sensor driver loading\n");
	 i2c_add_driver(&imx073_i2c_driver);
	 return 0;
}

static void __exit imx073_exit(void)
{
	i2c_del_driver(&imx073_i2c_driver);
}


module_init(imx073_init);
module_exit(imx073_exit);

MODULE_AUTHOR("LG Electronics");
MODULE_DESCRIPTION("Imx073 camera sensor driver");
MODULE_LICENSE("GPL");

