/*
* imx072_main.c - sony imx072 5M Bayer type sensor driver
*
* Copyright (C) 2010 LG Inc.
*
* Contributors:
*	Hyunsu Choi <hyunsu.choi@lge.com>
*
* Leverage OV9640.c
*
* This file is licensed under the terms of the GNU General Public License
* version 2. This program is licensed "as is" without any warranty of any
* kind, whether express or implied.
*/

#define DEBUG 1

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <media/imx072_main.h>
#include <mach/gpio-names.h>
#include <mach/hardware.h>
#include <linux/gpio.h>
#include "../../../bssq/bssq_cam_pmic.h"
#include <linux/lge_hw_rev.h> 

DEFINE_MUTEX(star_camera_lock);

#define IMX072_RESET			TEGRA_GPIO_PT3		//Before Rev.D  TEGRA_GPIO_PD2 
#define IMX072_PWRDN        TEGRA_GPIO_PD5

#define IMX072_PROBE_ADDR    0x0101
#define IMX072_PROBE_DATA    0x03

struct imx072_info {
	int mode;
	struct i2c_client *i2c_client;
	struct i2c_client *i2c_client_right;
	struct imx072_platform_data *pdata;
};

static struct imx072_info *info=NULL;;


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
static enum StereoCameraMode camera_mode = Main;
////////////////////////for stereo //////////////////
static u16 DefaultImageWidth =  1200;
static u16 DefaultImageHeight =  680;
#define UpperByte16to8(x) ((u8)((x&0xFF00)>>8))
#define LowerByte16to8(x) ((u8)(x&0x00FF))
//////////////////////////////////////////////////
#define imx072_TABLE_WAIT_MS 0
#define imx072_TABLE_END 1
#define imx072_MAX_RETRIES 3
#define USE_CLOCK_MAXIMUM 1
#define USE_DIGITAL_GAIN 1
#define USE_BINNING_MODE 1

static struct imx072_reg mode_start[] = {
	{0x0103,	0x01},//reset 0
	{imx072_TABLE_WAIT_MS, 0x0001},

//	Stand-by OFF Sequence
//		Power ON
//		Input INCK
//		XCLR OFF
//		PLL SettingINCK=24MHz,27times
//		Address	value
#if USE_CLOCK_MAXIMUM
	{0x0307,	0x1B},	//648M
#else
	{0x0307,	0x12},	//432M
#endif
	{0x0111,	0x00},
	{0x0101,	0x03},//{0x0101,	0x00},	//20010526 jinkwan.kim@lge.com camsensor 5M Mirror-flip
	{0x300A,	0x80},
	{0x300f,	0x03}, // mipi signal current 0x03--> 2mA, 0X02--> 1.5mA
	{0x3014,	0x08},
	{0x3015,	0x37},
	{0x3017,	0x40},
//	{0x3017,	0x01},
	{0x301C,	0x01},
	{0x3031,	0x28},
	{0x3040,	0x00},
	{0x3041,	0x60},
	{0x3051,	0x24},
	{0x3053,	0x34},
	{0x3055,	0x3B},
	{0x3057,	0xC0},
	{0x3060,	0x30},
	{0x3065,	0x00},
	{0x30AA,	0x88},
	{0x30AB,	0x1C},
	{0x30B0,	0x32},
	{0x30B2,	0x83},
	{0x30D3,	0x04},
	{0x310E,	0xDD},
	{0x31A4,	0xD8},
	{0x31A6,	0x17},
	{0x31AC,	0xCF},
	{0x31AE,	0xF1},
	{0x31B4,	0xD8},
	{0x31B6,	0x17},
/*	{0x3304,	0x03},
	{0x3305,	0x03},
	{0x3306,	0x0A},
	{0x3307,	0x02},
	{0x3308,	0x11},
	{0x3309,	0x04},
	{0x330A,	0x05},
//	{0x330B,	0x04},
//	{0x330C,	0x05},
	{0x330B,	0x04},
	{0x330C,	0x0a},
	{0x330D,	0x04},
	{0x330E,	0x01},
*/
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
	{0x330E, 0x00},

	{0x0100,	0x00},
	{imx072_TABLE_END, 0x0000}
};

static struct imx072_reg mode_2592x1944[] = {
	{0x0100, 0x00},
	{0x0101,	0x03}, //20010526 jinkwan.kim@lge.com camsensor 5M Mirror-flip
//		FULL
//		Mipi 2Lane	Address	value
//	{0x0340,	0x07},
//	{0x0341,	0xEE},
	{0x0104,	0x01},
	{0x034C,	0x0A},
	{0x034D,	0x30},
	{0x034E,	0x07},
	{0x034F,	0xA8},
	{0x0381,	0x01},
	{0x0383,	0x01},
	{0x0385,	0x01},
	{0x0387,	0x01},
	{0x3001, 0x00},
	{0x3016,	0x06},
	{0x30E8,	0x06},
	{0x3301,	0x00},  // discontinous mode 0x00
	{0x0104,	0x00},
	//{0x0100 , 0x01,  1},

	{imx072_TABLE_END, 0x0000}

};
static struct imx072_reg mode_2592x1464[] = {
	{0x0100, 0x00},
	{0x0101,	0x03},//{0x0101,	0x00},	//20010526 jinkwan.kim@lge.com camsensor 5M Mirror-flip
//		FULL
//		Mipi 2Lane	Address	value
	{0x0104,	0x01},
	{0x034C,	0x0A},
	{0x034D,	0x30},
	{0x034E,	0x05}, // Y size 1488
	{0x034F,	0xD0},

	{0x0346,	0x00},//		y_addr_start
	{0x0347,	0x81},//		y_addr_start
	{0x034A,	0x06},//		y_addr_end
	{0x034B,	0x50},//		y_addr_end

	{0x0381,	0x01},
	{0x0383,	0x01},
	{0x0385,	0x01},
	{0x0387,	0x01},
	{0x3001, 0x00},
	{0x3016,	0x06},
	{0x30E8,	0x06},
	{0x3301,	0x00},  //discontinous mode 0x00
	{0x0104,	0x00},
	//{0x0100 , 0x01,  1},

	{imx072_TABLE_END, 0x0000}

};

static struct imx072_reg mode_1296x972[] = {

	{0x0100, 0x00},
	{0x0101,	0x03},//{0x0101,	0x00},	//20010526 jinkwan.kim@lge.com camsensor 5M Mirror-flip
	{0x034C,	0x05},//X SIZE
	{0x034D,	0x18},

	{0x034E,	0x03},// Y SIZE
	{0x034F,	0xD4},

	{0x0381,	0x01},
	{0x0383,	0x03},
	{0x0385,	0x01},
	{0x0387,	0x03},
#if USE_BINNING_MODE
	{0x3016,	0x46},
	{0x30E8,	0x86},
#else
	{0x3016,	0x06},
	{0x30E8,	0x06},

#endif
	{0x3301,	0x40}, // discontinuous mode
	{imx072_TABLE_END, 0x0000}
};

static struct imx072_reg mode_1920x1088[] = {
	{0x0100,	0x00},
	{0x0101,	0x03},//{0x0101,	0x00},	//20010526 jinkwan.kim@lge.com camsensor 5M Mirror-flip
//		1920x1080 (30fps)
//		Mipi 2Lane	Address	value
//	{0x0340,	0x06},//	1550	frame_length
//	{0x0341,	0x0E},//		frame_length
	{0x0344,	0x01},//	336	x_addr_start
	{0x0345,	0x50},//		x_addr_start
//	{0x0346,	0x01},//	432	y_addr_start
//	{0x0347,	0xB0},//		y_addr_start
	{0x0348,	0x08},//	2271	x_addr_end
	{0x0349,	0xDF},//		x_addr_end
//	{0x034A,	0x05},//	1527	y_addr_end
//	{0x034B,	0xF7},//		y_addr_end
	{0x034C,	0x07},//	1936	x_out_size
	{0x034D,	0x90},//		x_out_size
	{0x034E,	0x04},//	1096	y_out_size
	{0x034F,	0x48},//		y_out_size

	{0x0346,	0x01},
	{0x0347,	0xB3},// FOR RGB ORDER
	{0x034A,	0x05},
	{0x034B,	0xFa},

	{0x0381,	0x01},//		x_even_inc
	{0x0383,	0x01},//		x_odd_inc
	{0x0385,	0x01},//		y_even_inc
	{0x0387,	0x01},//		y_odd_inc
	{0x3016,	0x06},//		VMODEADD
	{0x30E8,	0x06},//		HADDAVE
	{0x3301,	0x00},//		DisCONTINUOUS MODE
	{imx072_TABLE_END, 0x0000}
};

static struct imx072_reg mode_1296X736[]=
{
	{0x0100, 0x00},
	{0x0101,	0x03},//{0x0101,	0x00},	//20010526 jinkwan.kim@lge.com camsensor 5M Mirror-flip
	{0x0340,	0x03},
	{0x0341,	0xF7},
	{0x034C,	0x05},// X SIZE
	{0x034D,	0x18},
	{0x034E,	0x02},// Y SIZE
	{0x034F,	0xE8},

	{0x0346,	0x00},//		y_addr_start
	{0x0347,	0x81},//		y_addr_start
	{0x034A,	0x06},//		y_addr_end
	{0x034B,	0x50},//		y_addr_end

	{0x0381,	0x01},
	{0x0383,	0x03},
	{0x0385,	0x01},
	{0x0387,	0x03},
#if USE_BINNING_MODE
	{0x3016,	0x46},
	{0x30E8,	0x86},
#else
	{0x3016,	0x06},
	{0x30E8,	0x06},

#endif
	{0x3301,	0x40},  //discontinuous mode
	{imx072_TABLE_END, 0x0000}

};

static struct imx072_reg mode_1200x680[] = {
	//////////////////////////
	{0x0100,		0x00},
	{0x0101,	0x03},//{0x0101,	0x00},	//20010526 jinkwan.kim@lge.com camsensor 5M Mirror-flip
	{0x0340,     	0x03},
	{0x0341,       	0x03},
	{0x034C,       	0x04},   // x output size 4b0 -> 1200
	{0x034D,       	0xb0},
	{0x034E,       	0x02}, // y output size 2A8 -> 680
	{0x034F,       	0xA8},
	{0x0344, 	0x00},  //	x_addr_start
	{0x0345, 	0x69},  //	x_addr_start
	{0x0348, 	0x09},  //	x_addr_end
	{0x0349, 	0xc8},  //	x_addr_end
	{0x0346,		0x01},	//	y_addr_start
	{0x0347,		0x2d},	//	y_addr_start
	{0x034A,		0x06},	//	y_addr_end
	{0x034B,		0x7c},	//	y_addr_end
	{0x0381,        0x01},
	{0x0383,        0x03},
	{0x0385,        0x01},
	{0x0387,        0x03},
#if USE_BINNING_MODE
	{0x3016,	0x46},
	{0x30E8,	0x86},
#else
	{0x3016,	0x06},
	{0x30E8,	0x06},
#endif
	{0x3017,        0x40},
	{0x3301,        0x01},  // discontinous mode
	//    {0x0202,		0x0a},
	{imx072_TABLE_END, 0x0000}
};

static struct imx072_reg mode_end[] = {
	{0x30ee, 0x00},
	{0x30ee, 0x01},
	{0x0100, 0x01}, /* Data Stream On*/
	{imx072_TABLE_WAIT_MS, 0x00010},
	{0x30ee, 0x00},
	{imx072_TABLE_END, 0x0000}
};

enum {
	imx072_MODE_2592x1944,
	imx072_MODE_2592x1464,
	imx072_MODE_1296x972,
	imx072_MODE_1920x1088,
	imx072_MODE_1296x736,
	imx072_MODE_1200x680
};

static struct imx072_reg *mode_table[] = {
	[imx072_MODE_2592x1944] = mode_2592x1944,
	[imx072_MODE_2592x1464] = mode_2592x1464,
	[imx072_MODE_1296x972] = mode_1296x972,
	[imx072_MODE_1920x1088] = mode_1920x1088,
	[imx072_MODE_1296x736] = mode_1296X736,
	[imx072_MODE_1200x680] = mode_1200x680
};


/* 2 regs to program frame length */
static inline void imx072_get_frame_length_regs(struct imx072_reg *regs,
						u32 frame_length)
{
	regs->addr = 0x0340;
	regs->val = (frame_length >> 8) & 0xff;
	(regs + 1)->addr = 0x0341;
	(regs + 1)->val = (frame_length) & 0xff;
}

/* 2 regs to program coarse time */
static inline void imx072_get_coarse_time_regs(struct imx072_reg *regs,
						u32 coarse_time)
{
	regs->addr = 0x0202;
	regs->val = (coarse_time >> 8) & 0xff;
	(regs + 1)->addr = 0x0203;
	(regs + 1)->val = (coarse_time) & 0xff;
}

/* 1 reg to program gain */
static inline void imx072_get_gain_reg(struct imx072_reg *regs, u16 gain)
{
	regs->addr = 0x0205;
	regs->val = gain;
}
#if 0 //WBT#196355 : READ FUNCTION DON'T BE NEEDED.
static int imx072_read_reg(struct i2c_client *client, u16 addr, u8 *val)
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
	data[0] = (u8) (addr >> 8);;
	data[1] = (u8) (addr & 0xff);

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 1;
	msg[1].buf = data + 2;
	err = i2c_transfer(client->adapter, msg, 2);

	if (err != 2)
		return -EINVAL;

	*val = data[2];

	return 0;
}
#endif
static int imx072_write_reg(struct i2c_client *client, u16 addr, u8 val)
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
		//pr_info("imx072: I2C Transfer start Addr = %x\n", addr);
		err = i2c_transfer(client->adapter, &msg, 1);
		//pr_info("imx072: I2C Transfer end Addr = %x\n", addr);
		if (err >0 )
			return 0;
		retry++;
		pr_err("imx072: i2c transfer failed, retrying %x %x\n",
			addr, val);
		msleep(3);
	} while(retry <imx072_MAX_RETRIES);

	return err;
}
#if 0
/********************************************************************/
/*	the max of num is 50																*/
static int imx072_write_reg_burst(struct i2c_client *client, struct imx072_reg *reg_list, int num)
{
	int err;
	struct i2c_msg msg[50];
	unsigned char data[3];
	int retry = 0;
	int i=0;
	//msg = (struct i2c_msg *) kzalloc(sizeof(struct i2c_msg)*num, GFP_KERNEL);
	if (!client->adapter)
		return -ENODEV;
	for(i=0; i<num; i++){
		data[0] = (u8) (reg_list[i].addr >> 8);;
		data[1] = (u8) (reg_list[i].addr & 0xff);
		data[2] = (u8) (reg_list[i].val & 0xff);
		msg[i].addr = client->addr;
		msg[i].flags = 0;
		msg[i].len = 3;
		msg[i].buf = data;
//		pr_info("[%s] transferd msg addr 0x%x, value 0x%x", __func__, reg_list[i].addr, reg_list[i].val );
	}
//s32 i2c_write_block_data(client, u8 addr_flags, 1, addr, &val)
	do {
		err = i2c_transfer(client->adapter, &msg, (int)num);
		if (err == num ){
			pr_info("[%s] transferd msg number %d", __func__, err);
//			kfree(msg);
			return 0;
		}
		pr_err("imx072: i2c burst transfer failed, transferd msg number %d, requested number =%d\n",
			err, num);
		retry++;
		msleep(1);
	} while(retry <imx072_MAX_RETRIES);
//	kfree(msg);
	return err;
}
#endif
static int imx072_write_reg_helper(struct imx072_info *info, u16 addr, u8 val)
{
	int ret;
	switch(camera_mode){
		case Main:
		case LeftOnly:
			//pr_info("[%s] Left Camera\n",__func__);
			ret = imx072_write_reg(info->i2c_client, addr, val );
			break;
		case Stereo:
			//pr_info("[%s] Stereo Camera\n",__func__);
			ret = imx072_write_reg(info->i2c_client, addr, val);
			ret = imx072_write_reg(info->i2c_client_right, addr, val);
			break;
		case RightOnly:
			//pr_info("[%s] Right Camera\n",__func__);
			ret = imx072_write_reg(info->i2c_client_right, addr, val);
			break;
		default :
			return -1;
	}
	return ret;
}
#if 0
static int imx072_write_reg_burst_helper(struct imx072_info *info, struct imx072_reg *reg_list, int num)
{
	int ret;
	switch(camera_mode){
		case Main:
		case LeftOnly:
			//pr_info("[%s] Left Camera\n",__func__);
			ret = imx072_write_reg_burst(info->i2c_client, reg_list, num );
			break;
		case Stereo:
			//pr_info("[%s] Stereo Camera\n",__func__);
			ret = imx072_write_reg_burst(info->i2c_client, reg_list, num);
			ret = imx072_write_reg_burst(info->i2c_client_right, reg_list, num);
			break;
		case RightOnly:
			//pr_info("[%s] Right Camera\n",__func__);
			ret = imx072_write_reg_burst(info->i2c_client_right, reg_list, num);
			break;
		default :
			return -1;
	}
	return ret;
}
#endif
static int imx072_write_table(struct imx072_info *info,
			const struct imx072_reg table[],
			const struct imx072_reg override_list[],
			int num_override_regs)
{
	int err;
	const struct imx072_reg *next;

	for (next = table; next->addr != imx072_TABLE_END; next++) {
		if (next->addr ==  imx072_TABLE_WAIT_MS) {
			msleep(next->val);
			continue;
		}
		err = imx072_write_reg_helper(info, next->addr, next->val);
		if (err)
			return err;
	}
	return 0;
}

static int imx072_set_frame_length(struct imx072_info *info, u32 frame_length)
{
	struct imx072_reg reg_list[2];
	int i = 0;
	int ret;

	imx072_get_frame_length_regs(reg_list, frame_length);
	ret = imx072_write_reg_helper(info, 0x0104, 0x01);
	if (ret)
		return ret;

	for (i = 0; i < 2; i++)	{
		ret = imx072_write_reg_helper(info, reg_list[i].addr,
			reg_list[i].val);
		if (ret)
			return ret;
	}
	ret = imx072_write_reg_helper(info, 0x0104, 0x00);
	if (ret)
		return ret;

	return 0;
}

static int imx072_set_coarse_time(struct imx072_info *info, u32 coarse_time)
{
	int ret;

	struct imx072_reg reg_list[2];
	int i = 0;

	imx072_get_coarse_time_regs(reg_list, coarse_time);
	ret = imx072_write_reg_helper(info, 0x0104, 0x01);
	if (ret)
		return ret;

	for (i = 0; i < 2; i++)	{
		ret = imx072_write_reg_helper(info, reg_list[i].addr,
			reg_list[i].val);
		if (ret)
			return ret;
	}
	ret = imx072_write_reg_helper(info, 0x0104, 0x00);
	if (ret)
		return ret;

	return 0;
}

static int imx072_set_gain(struct imx072_info *info, struct imx072_gain *gain)
{
	int ret;
	struct imx072_reg reg_list_analog;
#if USE_DIGITAL_GAIN
	struct imx072_reg reg_list_digtal[8];
	u16 i;
#endif
	imx072_get_gain_reg(&reg_list_analog, gain->AnalogGain);
	ret = imx072_write_reg_helper(info, 0x0104, 0x01);
	if (ret)
		return ret;
	ret = imx072_write_reg_helper(info, reg_list_analog.addr, reg_list_analog.val);
	if (ret)
		return ret;
#if USE_DIGITAL_GAIN
	for(i=0; i<4; i++)
	{
		reg_list_digtal[i*2].addr = 0x020E + i*2;
		reg_list_digtal[i*2].val = gain->DigitalGain_Upper;
		reg_list_digtal[i*2+1].addr = 0x020F + i*2;
		reg_list_digtal[i*2+1].val = gain->DigitalGain_Lower;
		//pr_info("%s: addr 0x%x,  val 0x%x\n", __func__, reg_list_digtal[i*2].addr, reg_list_digtal[i*2].val);
		//pr_info("%s: addr 0x%x,  val 0x%x\n", __func__, reg_list_digtal[i*2+1].addr, reg_list_digtal[i*2+1].val);
		ret = imx072_write_reg_helper(info, reg_list_digtal[i*2].addr, reg_list_digtal[i*2].val);
		//ret = imx072_write_reg_helper(info, reg_list_digtal[i*2+1].addr, reg_list_digtal[i*2+1].val);
	}
	//ret = imx072_write_reg_burst_helper(info,reg_list_digtal, 8);
	if (ret)
		return ret;
#endif
	ret = imx072_write_reg_helper(info, 0x0104, 0x00);
	if (ret)
		return ret;

	return 0;
}

static int imx072_set_mode(struct imx072_info *info, struct imx072_mode *mode)
{
	int sensor_mode;
	int err;
	int i;
	int ret;
	struct imx072_reg reg_list[5];

	if(camera_mode != Main)
		sensor_mode = imx072_MODE_1200x680;
		//sensor_mode = imx072_MODE_1296x736;
	else{
		if (mode->xres == 2592 && mode->yres == 1944)
			sensor_mode = imx072_MODE_2592x1944;
		else if (mode->xres == 1296 && mode->yres == 972)
			sensor_mode = imx072_MODE_1296x972;
		else if (mode->xres == 1920 && mode->yres == 1088)
			sensor_mode = imx072_MODE_1920x1088;
		else if (mode->xres == 1296 && mode->yres == 736)
			sensor_mode = imx072_MODE_1296x736;
		else if (mode->xres == 2592 && mode->yres == 1464)
			sensor_mode = imx072_MODE_2592x1464;
		else {
			pr_err("%s: invalid resolution supplied to set mode %d %d\n",
					__func__, mode->xres, mode->yres);
			return -EINVAL;
		}
	}
	/* get a list of override regs for the asking frame length,	*/
	/* coarse integration time, and gain.*/
	err = imx072_write_table(info, mode_start, NULL, 0);
	if (err)
		return err;

	err = imx072_write_table(info, mode_table[sensor_mode], NULL, 0);
	if (err)
		return err;

	imx072_get_frame_length_regs(reg_list, mode->frame_length);
	for (i = 0; i < 2; i++)	{
		ret = imx072_write_reg_helper(info, reg_list[i].addr,
			reg_list[i].val);
		if (ret)
			return ret;
	}
	imx072_get_coarse_time_regs(reg_list + 2, mode->coarse_time);
	for (i = 0; i < 2; i++)	{
		ret = imx072_write_reg_helper(info, reg_list[i+2].addr,
			reg_list[i+2].val);
		if (ret)
			return ret;
	}
	ret = imx072_set_gain(info, &(mode->gain));
	if (ret)
		return ret;
	err = imx072_write_table(info, mode_end, NULL, 0);
	if (err)
		return err;

	info->mode = sensor_mode;
	//pr_info("%s: end\n",__func__);
	return 0;
}

static int imx072_get_status(struct imx072_info *info, u8 *status)
{
	int err=0;

	*status = 0;
	//err = imx072_read_reg(info->i2c_client, 0x001, status);
	//pr_info("%s: %u %d\n", __func__, *status, err);
	return err;
}
static int imx072_set_region(struct imx072_info *info, struct imx072_stereo_region *region)
{
	u16 image_width = region->image_end.x - region->image_start.x+1;
	u16 image_height = region->image_end.y - region->image_start.y+1;
	struct i2c_client *i2c_client;
	pr_info("%s: %d\n", __func__, region->camer_index);
	if(region->camer_index == 0)
		i2c_client = info ->i2c_client;
	else if(region->camer_index == 1)
		i2c_client = info ->i2c_client_right;
	else
		return -1;
	pr_info("%s: width = %d  height = %d\n", __func__, image_width, image_height );
	imx072_write_reg(i2c_client, 0x0104, 1);
	imx072_write_reg(i2c_client, 0x0346, UpperByte16to8(region->image_start.y));
	imx072_write_reg(i2c_client, 0x0347, LowerByte16to8(region->image_start.y));   // Y_ADDR START  LOWER BYTE
	imx072_write_reg(i2c_client, 0x034E, UpperByte16to8(DefaultImageHeight));  // Y_OUT SIZE UPPER BYTE
	imx072_write_reg(i2c_client, 0x034F, LowerByte16to8(DefaultImageHeight));   // Y_OUT SIZE  LOWER BYTE
	imx072_write_reg(i2c_client, 0x034A, UpperByte16to8(region->image_end.y)); // Y_ADDR_END      UPPER BYTE
	imx072_write_reg(i2c_client, 0x034B, LowerByte16to8(region->image_end.y)); // Y_ADDR_END    LOWER BYTE

	imx072_write_reg(i2c_client, 0x0344, UpperByte16to8(region->image_start.x));  // X_ADDR_START  UPPER BYTE
	imx072_write_reg(i2c_client, 0x0345, LowerByte16to8(region->image_start.x));   // X_ADDR START  LOWER BYTE
	imx072_write_reg(i2c_client, 0x034C, UpperByte16to8(DefaultImageWidth));  // X_SIZE  UPPER BYTE
	imx072_write_reg(i2c_client, 0x034D, LowerByte16to8(DefaultImageWidth));   //X_SIZE  LOWER BYTE
	imx072_write_reg(i2c_client, 0x0348, UpperByte16to8(region->image_end.x)); // X_ADDR_END      UPPER BYTE
	imx072_write_reg(i2c_client, 0x0349, LowerByte16to8(region->image_end.x)); // X_ADDR_END    LOWER BYTE
	imx072_write_reg(i2c_client, 0x0104, 0);

	return 0;

}

static void imx072_cam_pin_set(void)
{
	 mdelay(5); 	 //20110608 jinkwan.kim@lge.com Camsensor delay 5ms before Reset 
#if defined(CONFIG_LU6500)
	 if (REV_E > get_lge_pcb_revision()){
	    gpio_set_value(TEGRA_GPIO_PD2,1);
	}else
#endif
	{
	    gpio_set_value(IMX072_RESET,1);
	}
    mdelay(5);
    //gpio_set_value(Imx072_PWRDN,1);
    //mdelay(5);
}

static void imx072_cam_pin_unset(void)
{
#if defined(CONFIG_LU6500)
	 if (REV_E > get_lge_pcb_revision()){
	    gpio_set_value(TEGRA_GPIO_PD2,0);
	}else
#endif
	{
	    gpio_set_value(IMX072_RESET,0);
	} 
    mdelay(5);    
    //gpio_set_value(Imx072_PWRDN,0);
	//mdelay(5);
}

static int imx072_power_on(struct file *file)
{
	int ret = 0;
	struct imx072_info *info = file->private_data;
	//pr_info("%s: start - camera mode %d \n", __func__, camera_mode);
	ret = star_cam_Main_power_on();
	if(ret < 0)
	{
		pr_info("%s: ldo or sensor power on failure \n", __func__);
		return -EINVAL;
	}

	switch(camera_mode){
		case Main:
			imx072_cam_pin_set();
			break;
		case Stereo:
			imx072_cam_pin_set();
			break;
		case LeftOnly:
			imx072_cam_pin_set();
			break;
		case RightOnly:
			imx072_cam_pin_set();
			break;
		default :
			return -1;
	}

	ret = imx072_write_reg(info->i2c_client, IMX072_PROBE_ADDR, IMX072_PROBE_DATA);
	if(ret < 0)
	{
		pr_info("%s: sensor probe failure \n", __func__);
	}
	else
	{
		pr_info("%s: sensor probe succeeded \n", __func__);
	}

	//pr_info("%s: end\n", __func__);
	return ret;
}

static void imx072_power_off(enum StereoCameraMode mode)
{
	switch(mode){
		case Main:
            imx072_cam_pin_unset();          
			break;
		case Stereo:
            imx072_cam_pin_unset();          
			break;
		case LeftOnly:
            imx072_cam_pin_unset();          
			break;
		case RightOnly:
            imx072_cam_pin_unset();          
			break;
		default :
			return -1;
	}
	mdelay(3);
	star_cam_power_off();
}

static long imx072_ioctl(struct file *file,
			unsigned int cmd, unsigned long arg)
{
	int err;
	struct imx072_info *info = file->private_data;

	//pr_info("\nimx072_ioctl : cmd = %d\n", cmd);

	switch (cmd) {
	case IMX072_IOCTL_SET_POWER_ON:
		pr_info("IMX072_IOCTL_SET_POWER_ON\n");
		camera_mode = arg;
		return imx072_power_on(file);
	case IMX072_IOCTL_SET_MODE:
	{
		struct imx072_mode mode;
		pr_info("IMX072_IOCTL_SET_MODE\n");
		if (copy_from_user(&mode,
					(const void __user *)arg,
					sizeof(struct imx072_mode))) {
			pr_info("%s %d\n", __func__, __LINE__);
			return -EFAULT;
		}
		mutex_lock(&star_camera_lock);
		err = imx072_set_mode(info, &mode);
		mutex_unlock(&star_camera_lock);
		return err;
	}
	case IMX072_IOCTL_SET_FRAME_LENGTH:
		//pr_info("IMX072_IOCTL_SET_FRAME_LENGTH\n");
		mutex_lock(&star_camera_lock);
		err = imx072_set_frame_length(info, (u32)arg);
		mutex_unlock(&star_camera_lock);
		return err;
	case IMX072_IOCTL_SET_COARSE_TIME:
		//pr_info("IMX072_IOCTL_SET_COARSE_TIME\n");
		mutex_lock(&star_camera_lock);
		err = imx072_set_coarse_time(info, (u32)arg);
		mutex_unlock(&star_camera_lock);
		return err;
	case IMX072_IOCTL_SET_GAIN:
	{
		struct imx072_gain gain;
		//pr_info("IMX072_IOCTL_SET_GAIN\n");
		if (copy_from_user(&gain,
					(const void __user *)arg,
					sizeof(struct imx072_gain))) {
			pr_info("%s %d\n", __func__, __LINE__);
			return -EFAULT;
		}
		mutex_lock(&star_camera_lock);
		err = imx072_set_gain(info, &gain);
		mutex_unlock(&star_camera_lock);
		return err;
	}
	case IMX072_IOCTL_GET_STATUS:
	{
		u8 status;
		pr_info("IMX072_IOCTL_GET_STATUS\n");
		err = imx072_get_status(info, &status);
		if (err)
			return err;
		if (copy_to_user((void __user *)arg, &status,
				 2)) {
			pr_info("%s %d\n", __func__, __LINE__);
			return -EFAULT;
		}
		return 0;
	}
	case IMX072_IOCTL_SET_SENSOR_REGION:
	{
		struct imx072_stereo_region region;
		pr_info("IMX072_IOCTL_SET_SENSOR_REGION\n");
		if (copy_from_user(&region,
					(const void __user *)arg,
					sizeof(struct imx072_stereo_region))) {
			pr_info("%s %d\n", __func__, __LINE__);
			return -EFAULT;
		}
		mutex_lock(&star_camera_lock);
		err = imx072_set_region(info, &region);
		mutex_unlock(&star_camera_lock);
		return err;
	}
	case IMX072_IOCTL_SENSOR_RESET:
	{
		u8 status;
		pr_info("IMX072_IOCTL_SENSOR_RESET(%d)\n", (int)arg);
		imx072_power_off(camera_mode);
		return 0;
	}
	default:
		pr_info("%s: Invalid command(%d)\n", __FUNCTION__, cmd);
		return -EINVAL;
	}
	return 0;
}

static int imx072_open(struct inode *inode, struct file *file)
{
    //pr_info("%s:++++\n", __func__);
	//int err;
	file->private_data = info;
	return 0;
}

static int imx072_release(struct inode *inode, struct file *file)
{
	//pr_info("%s\n", __func__);

	file->private_data = NULL;
	imx072_power_off(camera_mode);
	camera_mode = 0;
	return 0;
}


static const struct file_operations imx072_fileops = {
	.owner = THIS_MODULE,
	.open = imx072_open,
	.unlocked_ioctl = imx072_ioctl,
	.release = imx072_release,
};

static struct miscdevice imx072_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "imx072",
	.fops = &imx072_fileops,
	.mode = S_IRWXUGO
};

static int imx072_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int err;
	//pr_info("imx072: (GENE)probing sensor.(id:%s)\n", id->name);

	info = kzalloc(sizeof(struct imx072_info), GFP_KERNEL);
	if (!info) {
		pr_err("imx072: Unable to allocate memory!\n");
		return -ENOMEM;
	}

	err = misc_register(&imx072_device);
	if (err) {
		pr_err("imx072: Unable to register misc device!\n");
		kfree(info);
		return err;
	}

	info->pdata = client->dev.platform_data;
	info->i2c_client = client;

	i2c_set_clientdata(client, info);

	#if defined(CONFIG_LU6500)
	if (REV_E > get_lge_pcb_revision()){
		gpio_request(TEGRA_GPIO_PD2, "main_cam_reset");
    	tegra_gpio_enable(TEGRA_GPIO_PD2);
    	gpio_direction_output(TEGRA_GPIO_PD2,1);
    	mdelay(5);
    	gpio_set_value(TEGRA_GPIO_PD2,0);
	}else
	#endif
	{   
		gpio_request(IMX072_RESET, "main_cam_reset");
    	tegra_gpio_enable(IMX072_RESET);
    	gpio_direction_output(IMX072_RESET,1);
    	mdelay(5);
    	gpio_set_value(IMX072_RESET,0);
	}
	//pr_info("@@@@@@@@@@@@@@@ imx072: +++++++ SET ++++++\n");
	
	return 0;
}



static int imx072_remove(struct i2c_client *client)
{
	struct imx072_info *info;

    //pr_info("@@@@@@@@@@@@@@@ imx072_remove\n");
	info = i2c_get_clientdata(client);
	misc_deregister(&imx072_device);
	kfree(info);

	#if defined(CONFIG_LU6500)
	if (REV_E > get_lge_pcb_revision()){
		gpio_free(TEGRA_GPIO_PD2);
	}else
	#endif
	{
	    gpio_free(IMX072_RESET);
	}
	return 0;
}

static const struct i2c_device_id imx072_id[] = {
    { "imx072", 0 },
};

//MODULE_DEVICE_TABLE(i2c, imx072_id);

static struct i2c_driver imx072_i2c_driver = {
    .probe = imx072_probe,
    .remove = imx072_remove,
    .id_table = imx072_id,
    .driver = {
        .name = "imx072",
        .owner = THIS_MODULE,
    },
};


static int __init imx072_init(void)
{
    //pr_info("imx072 sensor driver loading\n");
	return i2c_add_driver(&imx072_i2c_driver);
}

static void __exit imx072_exit(void)
{
	i2c_del_driver(&imx072_i2c_driver);
}

module_init(imx072_init);
module_exit(imx072_exit);
