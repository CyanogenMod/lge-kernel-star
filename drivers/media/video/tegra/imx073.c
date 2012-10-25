/*
 * imx073.c - imx073 sensor driver
 *
 * Copyright (C) 2011 Google Inc.
 *
 * Contributors:
 *      Rebecca Schultz Zavin <rebecca@android.com>
 *
 * Leverage OV9640.c
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
#include <media/imx073.h>


struct imx073_reg {
	u16 addr;
	u16 val;
};

struct imx073_info {
	int mode;
	struct i2c_client *i2c_client;
	struct imx073_platform_data *pdata;
};

#define IMX073_TABLE_WAIT_MS 0
#define IMX073_TABLE_END 1
#define IMX073_MAX_RETRIES 3

static struct imx073_reg tp_none_seq[] = {
	{0x5046, 0x00}, /* isp_off */
	{IMX073_TABLE_END, 0x0000}
};

static struct imx073_reg tp_cbars_seq[] = {
	{0x503D, 0xC0},
	{0x503E, 0x00},
	{0x5046, 0x01}, /* isp_on */
	{IMX073_TABLE_END, 0x0000}
};

static struct imx073_reg tp_checker_seq[] = {
	{0x503D, 0xC0},
	{0x503E, 0x0A},
	{0x5046, 0x01}, /* isp_on */
	{IMX073_TABLE_END, 0x0000}
};

static struct imx073_reg *test_pattern_modes[] = {
	tp_none_seq,
	tp_cbars_seq,
	tp_checker_seq,
};

static struct imx073_reg reset_seq[] = {
	{0x3008, 0x82}, /* reset registers pg 72 */
	{IMX073_TABLE_WAIT_MS, 5},
	{0x3008, 0x42}, /* register power down pg 72 */
	{IMX073_TABLE_WAIT_MS, 5},
	{IMX073_TABLE_END, 0x0000},
};

static struct imx073_reg mode_start[] = {
	{0x3103, 0x93}, /* power up system clock from PLL page 77 */
	{0x3017, 0xff}, /* PAD output enable page 100 */
	{0x3018, 0xfc}, /* PAD output enable page 100 */

	{0x3600, 0x50}, /* analog pg 108 */
	{0x3601, 0x0d}, /* analog pg 108 */
	{0x3604, 0x50}, /* analog pg 108 */
	{0x3605, 0x04}, /* analog pg 108 */
	{0x3606, 0x3f}, /* analog pg 108 */
	{0x3612, 0x1a}, /* analog pg 108 */
	{0x3630, 0x22}, /* analog pg 108 */
	{0x3631, 0x22}, /* analog pg 108 */
	{0x3702, 0x3a}, /* analog pg 108 */
	{0x3704, 0x18}, /* analog pg 108 */
	{0x3705, 0xda}, /* analog pg 108 */
	{0x3706, 0x41}, /* analog pg 108 */
	{0x370a, 0x80}, /* analog pg 108 */
	{0x370b, 0x40}, /* analog pg 108 */
	{0x370e, 0x00}, /* analog pg 108 */
	{0x3710, 0x28}, /* analog pg 108 */
	{0x3712, 0x13}, /* analog pg 108 */
	{0x3830, 0x50}, /* manual exposure gain bit [0] */
	{0x3a18, 0x00}, /* AEC gain ceiling bit 8 pg 114 */
	{0x3a19, 0xf8}, /* AEC gain ceiling pg 114 */
	{0x3a00, 0x38}, /* AEC control 0 debug mode band low
			   limit mode band func pg 112 */

	{0x3603, 0xa7}, /* analog pg 108 */
	{0x3615, 0x50}, /* analog pg 108 */
	{0x3620, 0x56}, /* analog pg 108 */
	{0x3810, 0x00}, /* TIMING HVOFFS both are zero pg 80 */
	{0x3836, 0x00}, /* TIMING HVPAD both are zero pg 82 */
	{0x3a1a, 0x06}, /* DIFF MAX an AEC register??? pg 114 */
	{0x4000, 0x01}, /* BLC enabled pg 120 */
	{0x401c, 0x48}, /* reserved pg 120 */
	{0x401d, 0x28}, /* BLC control pg 120 */
	{0x5000, 0x00}, /* ISP control00 features are disabled. pg 132 */
	{0x5001, 0x00}, /* ISP control01 awb disabled. pg 132 */
	{0x5002, 0x00}, /* ISP control02 debug mode disabled pg 132 */
	{0x503d, 0x00}, /* ISP control3D features disabled pg 133 */
	{0x5046, 0x00}, /* ISP control isp disable awbg disable pg 133 */

	{0x300f, 0x8f}, /* PLL control00 R_SELD5 [7:6] div by 4 R_DIVL [2]
			   two lane div 1 SELD2P5 [1:0] div 2.5 pg 99 */
	{0x3010, 0x10}, /* PLL control01 DIVM [3:0] DIVS [7:4] div 1 pg 99 */
	{0x3011, 0x14}, /* PLL control02 R_DIVP [5:0] div 20 pg 99 */
	{0x3012, 0x02}, /* PLL CTR 03, default */
	{0x3815, 0x82}, /* PCLK to SCLK ratio bit[4:0] is set to 2 pg 81 */
	{0x3503, 0x33}, /* AEC auto AGC auto gain has no latch delay. pg 38 */
	/*	{FAST_SETMODE_START, 0}, */
	{0x3613, 0x44}, /* analog pg 108 */
	{IMX073_TABLE_END, 0x0},
};

static struct imx073_reg mode_3264x2448[] = {

    {0x0100, 0x00},
// sungmin.woo for test start //
	{0x0307,	0x24},
	{0x302B,	0x38},
	{0x30E5,	0x04},
	{0x3300,	0x00},
	{0x0101,	0x03},// imager_orientation
//    {0x0202, 0x03},// default
//    {0x0203, 0xe8},
	{0x300A,	0x80},
	{0x3014,	0x08},
	{0x3015,	0x37},
	{0x3017,	0x60},
	{0x301C,	0x01},
	{0x3031,	0x28},
	{0x3040,	0x00},
	{0x3041,	0x60},
	{0x3047,	0x10},
	{0x3051,	0x24},
	{0x3053,	0x34},
	{0x3055,	0x3B},
	{0x3057,	0xC0},
	{0x3060,	0x30},
	{0x3065,	0x00},
	{0x30A1,	0x03},
	{0x30A3,	0x01},
	{0x30AA,	0x88},
	{0x30AB,	0x1C},
	{0x30B0,	0x32},
	{0x30B2,	0x83},
	{0x30D3,	0x04},
	{0x310C,	0xE9},
	{0x310D,	0x00},
	{0x316B,	0x14},
	{0x316D,	0x3B},
	{0x31A4,	0xD8},
	{0x31A6,	0x17},
	{0x31AC,	0xCF},
	{0x31AE,	0xF1},
	{0x31B4,	0xD8},
	{0x31B6,	0x17},
	{0x3302,	0x0A},
	{0x3303,	0x09},
	{0x3304,	0x05},
	{0x3305,	0x04},
	{0x3306,	0x15},
	{0x3307,	0x03},
	{0x3308,	0x13},
	{0x3309,	0x05},
	{0x330A,	0x0B},
	{0x330B,	0x04},
	{0x330C,	0x0B},
	{0x330D,	0x06},
	{0x0340,	0x09},
	{0x0341,	0xCE},
    {0x0342,  0x0d},
    {0x0343,  0x70},
// MOBII_CHANGE  dkhan : for fullhd video [
	{0x0344,	0x00},	// x_addr_start
	{0x0345,	0x00},
	{0x0346,	0x00},	// y_addr_start
	{0x0347,	0x00},
	{0x0348,	0x0c},	// x_addr_end
	{0x0349,	0xcf},
	{0x034A,	0x09},	// y_addr_end
	{0x034B,	0x9f},
// MOBII_CHANGE  dkhan : for fullhd video ]

	{0x034C,	0x0C},	// 3280 
	{0x034D,	0xD0},
	{0x034E,	0x09},	// 2464
	{0x034F,	0xA0},
	{0x0381,	0x01},
	{0x0383,	0x01},
	{0x0385,	0x01},
	{0x0387,	0x01},
	{0x3001,	0x00},
	{0x3016,	0x06},
	{0x30E8,	0x06},
	{0x3301,	0x00},
	{0x0100,	0x01},
	{IMX073_TABLE_END, 0x0000}
	// sungmin.woo for test start //
};

//MOBII_CHANGE_S dongki.han@lge.com 20120308 : merge camera sensor su660 gb
static struct imx073_reg mode_3264x1224[] = {
    {0x0100, 0x00},
// sungmin.woo for test start //
	{0x0307,	0x24},
	{0x302B,	0x38},
	{0x30E5,	0x04},
	{0x3300,	0x00},
	{0x0101,	0x03},// imager_orientation
//	{0x0202,	0x03},// default
//	{0x0203,	0xe8},
	{0x300A,	0x80},
	{0x3014,	0x08},
	{0x3015,	0x37},
	{0x3017,	0x60},
	{0x301C,	0x01},
	{0x3031,	0x28},
	{0x3040,	0x00},
	{0x3041,	0x60},
	{0x3047,	0x10},
	{0x3051,	0x24},
	{0x3053,	0x34},
	{0x3055,	0x3B},
	{0x3057,	0xC0},
	{0x3060,	0x30},
	{0x3065,	0x00},
	{0x30A1,	0x03},
	{0x30A3,	0x01},
	{0x30AA,	0x88},
	{0x30AB,	0x1C},
	{0x30B0,	0x32},
	{0x30B2,	0x83},
	{0x30D3,	0x04},
	{0x310C,	0xE9},
	{0x310D,	0x00},
	{0x316B,	0x14},
	{0x316D,	0x3B},
	{0x31A4,	0xD8},
	{0x31A6,	0x17},
	{0x31AC,	0xCF},
	{0x31AE,	0xF1},
	{0x31B4,	0xD8},
	{0x31B6,	0x17},
	{0x3302,	0x0A},
	{0x3303,	0x09},
	{0x3304,	0x05},
	{0x3305,	0x04},
	{0x3306,	0x15},
	{0x3307,	0x03},
	{0x3308,	0x13},
	{0x3309,	0x05},
	{0x330A,	0x0B},
	{0x330B,	0x04},
	{0x330C,	0x0B},
	{0x330D,	0x06},
	{0x0340,	0x04}, // pg 66, frame length lines 
	{0x0341,	0xE6}, //
    {0x0342,  0x0d},
    {0x0343,  0x70},

// MOBII_CHANGE  dkhan : for fullhd video [
	{0x0344,	0x00},	// x_addr_start
	{0x0345,	0x00},
	{0x0346,	0x00},	// y_addr_start
	{0x0347,	0x00},
	{0x0348,	0x0c},	// x_addr_end
	{0x0349,	0xcf},
	{0x034A,	0x09},	// y_addr_end
	{0x034B,	0x9f},
// MOBII_CHANGE  dkhan : for fullhd video ]

	{0x034C,	0x0C}, // x output size
	{0x034D,	0xD0}, // x output size (3280)
	{0x034E,	0x04}, // pg 66, y output size (1232 lines, which is 22 more lines than 1210)
	{0x034F,	0xD0}, //
	{0x0381,	0x01}, // x_even_inc
	{0x0383,	0x01}, // x_odd_inc
	{0x0385,	0x01}, // y_even_inc
	{0x0387,	0x03}, // y_odd_inc
	{0x3001,	0x00},
	{0x3016,	0x46},
	{0x30E8,	0x06},
	{0x3301,	0x00},
	{0x0100,	0x01},

	{IMX073_TABLE_END, 0x0000}
	// sungmin.woo for test start //
};
//MOBII_CHANGE_E dongki.han@lge.com 20120308 : merge camera sensor su660 gb

static struct imx073_reg mode_1296x972[] = {
    {0x0100, 0x00},
	{0x3621, 0xaf}, /* analog horizontal binning/sampling not enabled.
			   pg 108 */
	{0x3632, 0x5a}, /* analog pg 108 */
	{0x3703, 0xb0}, /* analog pg 108 */
	{0x370c, 0xc5}, /* analog pg 108 */
	{0x370d, 0x42}, /* analog pg 108 */
	{0x3713, 0x2f}, /* analog pg 108 */
	{0x3800, 0x03}, /* HREF start point higher 4 bits [3:0] pg 108 */
	{0x3801, 0x3c}, /* HREF start point lower  8 bits [7:0] pg 108 */
	{0x3802, 0x00}, /* VREF start point higher 4 bits [3:0] pg 108 */
	{0x3803, 0x06}, /* VREF start point [7:0] pg 108 */
	{0x3804, 0x05}, /* HREF width  higher 4 bits [3:0] pg 108 */
	{0x3805, 0x10}, /* HREF width  lower  8 bits [7:0] pg 108 */
	{0x3806, 0x03}, /* VREF height higher 4 bits [3:0] pg 109 */
	{0x3807, 0xd0}, /* VREF height lower  8 bits [7:0] pg 109 */
	{0x3808, 0x05}, /* DVP horizontal output size higher 4 bits [3:0]
			   pg 109 */
	{0x3809, 0x10}, /* DVP horizontal output size lower  8 bits [7:0]
			   pg 109 */
	{0x380a, 0x03}, /* DVP vertical   output size higher 4 bits [3:0]
			   pg 109 */
	{0x380b, 0xd0}, /* DVP vertical   output size lower  8 bits [7:0]
			   pg 109 */
	{0x380c, 0x08}, /* total horizontal size higher 5 bits [4:0]
			   pg 109, line length */
	{0x380d, 0xa8}, /* total horizontal size lower  8 bits [7:0] pg 109,
			   line length */
	{0x380e, 0x05}, /* total vertical   size higher 5 bits [4:0] pg 109,
			   frame length */
	{0x380f, 0xa4}, /* total horizontal size lower  8 bits [7:0] pg 109,
			   frame length */
	{0x3818, 0xc1}, /* timing control reg18 mirror & dkhf pg 110 */
	{0x381a, 0x00}, /* HS mirror adjustment pg 110 */
	{0x3a0d, 0x08}, /* b60 max pg 113 */
	{0x3c01, 0x00}, /* 5060HZ_CTRL01 pg 116 */
	{0x3007, 0x3b}, /* clock enable03 pg 98 */
	{0x5059, 0x80}, /* => NOT found. added */
	{0x3003, 0x03}, /* reset MIPI and DVP pg 97 */
	{0x3500, 0x00}, /* long exp 1/3 in unit of 1/16 line, pg 38,
			   note frame length is from 0x5a4,
			   and SENSOR_BAYER_DEFAULT_MAX_COARSE_DIFF=3 */
	{0x3501, 0x5a}, /* long exp 2/3 in unit of 1/16 line, pg 38 */
	{0x3502, 0x10}, /* long exp 3/3 in unit of 1/16 line, pg 38 */
	{0x350a, 0x00}, /* gain output to sensor, pg 38 */
	{0x350b, 0x10}, /* gain output to sensor, pg 38 */
	{0x4801, 0x0f}, /* MIPI control01 pg 125 */
	{0x300e, 0x0c}, /* SC_MIPI_SC_CTRL0 pg 73 */
	{0x4803, 0x50}, /* MIPI CTRL3 pg 91 */
	{0x4800, 0x34}, /* MIPI CTRl0 idle and short line pg 89 */
	{IMX073_TABLE_END, 0x0000}
};

static struct imx073_reg mode_2080x1164[] = {
	{0x3103, 0x93}, // power up system clock from PLL page 77
	{0x3007, 0x3b}, // clock enable03 pg 98
	{0x3017, 0xff}, // PAD output enable page 100
	{0x3018, 0xfc}, // PAD output enable page 100

	{0x3600, 0x54}, // analog pg 108
	{0x3601, 0x05}, // analog pg 108
	{0x3603, 0xa7}, // analog pg 108
	{0x3604, 0x40}, // analog pg 108
	{0x3605, 0x04}, // analog pg 108
	{0x3606, 0x3f}, // analog pg 108
	{0x3612, 0x1a}, // analog pg 108
	{0x3613, 0x44}, // analog pg 108
	{0x3615, 0x52}, // analog pg 108
	{0x3620, 0x56}, // analog pg 108
	{0x3623, 0x01}, // analog pg 108
	{0x3630, 0x22}, // analog pg 108
	{0x3631, 0x36}, // analog pg 108
	{0x3632, 0x5f}, // analog pg 108
	{0x3633, 0x24}, // analog pg 108

	{0x3702, 0x3a}, // analog pg 108
	{0x3704, 0x18}, // analog pg 108
	{0x3706, 0x41}, // analog pg 108
	{0x370b, 0x40}, // analog pg 108
	{0x370e, 0x00}, // analog pg 108
	{0x3710, 0x28}, // analog pg 108
	{0x3711, 0x24},
	{0x3712, 0x13}, // analog pg 108

	{0x3810, 0x00}, // TIMING HVOFFS both are zero pg 80
	{0x3815, 0x82}, // PCLK to SCLK ratio bit[4:0] is set to 2 pg 81
	{0x3830, 0x50}, // manual exposure gain bit [0]
	{0x3836, 0x00}, // TIMING HVPAD both are zero pg 82

	{0x3a1a, 0x06}, // DIFF MAX an AEC register??? pg 114
	{0x3a18, 0x00}, // AEC gain ceiling bit 8 pg 114
	{0x3a19, 0xf8}, // AEC gain ceiling pg 114
	{0x3a00, 0x38}, // AEC control 0 debug mode band low limit mode band func pg 112
	{0x3a0d, 0x06}, // b60 max pg 113
	{0x3c01, 0x34}, // 5060HZ_CTRL01 pg 116

	{0x401f, 0x03}, // BLC enabled pg 120
	{0x4000, 0x05}, // BLC enabled pg 120
	{0x401d, 0x08}, // reserved pg 120
	{0x4001, 0x02}, // BLC control pg 120

	{0x5000, 0x00}, // ISP control00 features are disabled. pg 132
	{0x5001, 0x00}, // ISP control01 awb disabled. pg 132
	{0x5002, 0x00}, // ISP control02 debug mode disabled pg 132
	{0x503d, 0x00}, // ISP control3D features disabled pg 133
	{0x5046, 0x00}, // ISP control isp disable awbg disable pg 133

	{0x300f, 0x8f}, // PLL control00 R_SELD5 [7:6] div by 4 R_DIVL [2] two lane div 1 SELD2P5 [1:0] div 2.5 pg 99
	{0x3010, 0x10}, // PLL control01 DIVM [3:0] DIVS [7:4] div 1 pg 99
	{0x3011, 0x14}, // PLL control02 R_DIVP [5:0] div 20 pg 99
	{0x3012, 0x02}, // PLL CTR 03, default
	{0x3503, 0x33}, // AEC auto AGC auto gain has delay of 2 frames. pg 38

	{0x3621, 0x2f}, // analog horizontal binning/sampling not enabled. pg 108
	{0x3703, 0xe6}, // analog pg 108
	{0x370c, 0x00}, // analog pg 108
	{0x370d, 0x04}, // analog pg 108
	{0x3713, 0x22}, // analog pg 108
	{0x3714, 0x27},
	{0x3705, 0xda},
	{0x370a, 0x80},

	{0x3800, 0x02}, // HREF start point higher 4 bits [3:0] pg 108
	{0x3801, 0x12}, // HREF start point lower  8 bits [7:0] pg 108
	{0x3802, 0x00}, // VREF start point higher 4 bits [3:0] pg 108
	{0x3803, 0x0a}, // VREF start point [7:0] pg 108
	{0x3804, 0x08}, // HREF width  higher 4 bits [3:0] pg 108
	{0x3805, 0x20}, // HREF width  lower  8 bits [7:0] pg 108
	{0x3806, 0x04}, // VREF height higher 4 bits [3:0] pg 109
	{0x3807, 0x92}, // VREF height lower  8 bits [7:0] pg 109
	{0x3808, 0x08}, // DVP horizontal output size higher 4 bits [3:0] pg 109
	{0x3809, 0x20}, // DVP horizontal output size lower  8 bits [7:0] pg 109
	{0x380a, 0x04}, // DVP vertical   output size higher 4 bits [3:0] pg 109
	{0x380b, 0x92}, // DVP vertical   output size lower  8 bits [7:0] pg 109
	{0x380c, 0x0a}, // total horizontal size higher 5 bits [4:0] pg 109, line length
	{0x380d, 0x96}, // total horizontal size lower  8 bits [7:0] pg 109, line length
	{0x380e, 0x04}, // total vertical   size higher 5 bits [4:0] pg 109, frame length
	{0x380f, 0x9e}, // total vertical   size lower  8 bits [7:0] pg 109, frame length
	{0x3818, 0xc0}, // timing control reg18 mirror & dkhf pg 110
	{0x381a, 0x3c}, // HS mirror adjustment pg 110
	{0x381c, 0x31},
	{0x381d, 0x8e},
	{0x381e, 0x04},
	{0x381f, 0x92},
	{0x3820, 0x04},
	{0x3821, 0x19},
	{0x3824, 0x01},
	{0x3827, 0x0a},
	{0x401c, 0x46},

	{0x3003, 0x03}, // reset MIPI and DVP pg 97
	{0x3500, 0x00}, // long exp 1/3 in unit of 1/16 line, pg 38
	{0x3501, 0x49}, // long exp 2/3 in unit of 1/16 line, pg 38
	{0x3502, 0xa0}, // long exp 3/3 in unit of 1/16 line, pg 38
	{0x350a, 0x00}, // gain output to sensor, pg 38
	{0x350b, 0x00}, // gain output to sensor, pg 38
	{0x4801, 0x0f}, // MIPI control01 pg 125
	{0x300e, 0x0c}, // SC_MIPI_SC_CTRL0 pg 73
	{0x4803, 0x50}, // MIPI CTRL3 pg 91
	{0x4800, 0x34}, // MIPI CTRl0 idle and short line pg 89

	{IMX073_TABLE_END, 0x0000}
};

static struct imx073_reg mode_1920x1088[] = {
	{0x3621, 0x2f}, /* analog horizontal binning/sampling not enabled.
			   pg 108 */
	{0x3632, 0x55}, /* analog pg 108 */
	{0x3703, 0xe6}, /* analog pg 108 */
	{0x370c, 0xa0}, /* analog pg 108 */
	{0x370d, 0x04}, /* analog pg 108 */
	{0x3713, 0x2f}, /* analog pg 108 */
	{0x3800, 0x02}, /* HREF start point higher 4 bits [3:0] pg 108 */
	{0x3801, 0x58}, /* HREF start point lower  8 bits [7:0] pg 108 */
	{0x3802, 0x00}, /* VREF start point higher 4 bits [3:0] pg 108 */
	{0x3803, 0x0c}, /* VREF start point [7:0] pg 108 */
	{0x3804, 0x0a}, /* HREF width  higher 4 bits [3:0] pg 108 */
	{0x3805, 0x20}, /* HREF width  lower  8 bits [7:0] pg 108 */
	{0x3806, 0x07}, /* VREF height higher 4 bits [3:0] pg 109 */
	{0x3807, 0xa0}, /* VREF height lower  8 bits [7:0] pg 109 */
	{0x3808, 0x0a}, /* DVP horizontal output size higher 4 bits [3:0]
			   pg 109 */
	{0x3809, 0x20}, /* DVP horizontal output size lower  8 bits [7:0]
			   pg 109 */
	{0x380a, 0x07}, /* DVP vertical   output size higher 4 bits [3:0]
			   pg 109 */
	{0x380b, 0xa0}, /* DVP vertical   output size lower  8 bits [7:0]
			   pg 109 */
	{0x380c, 0x0c}, /* total horizontal size higher 5 bits [4:0] pg 109,
			   line length */
	{0x380d, 0xb4}, /* total horizontal size lower  8 bits [7:0] pg 109,
			   line length */
	{0x380e, 0x07}, /* total vertical size higher 5 bits [4:0] pg 109,
			   frame length */
	{0x380f, 0xb0}, /* total vertical size lower  8 bits [7:0] pg 109,
			   frame length */
	{0x3818, 0xc0}, /* timing control reg18 mirror & dkhf pg 110 */
	{0x381a, 0x3c}, /* HS mirror adjustment pg 110 */
	{0x3a0d, 0x06}, /* b60 max pg 113 */
	{0x3c01, 0x00}, /* 5060HZ_CTRL01 pg 116 */
	{0x3007, 0x3f}, /* clock enable03 pg 98 */
	{0x5059, 0x80}, /* => NOT found */
	{0x3003, 0x03}, /* reset MIPI and DVP pg 97 */
	{0x3500, 0x00}, /* long exp 1/3 in unit of 1/16 line, pg 38 */
	{0x3501, 0x7a}, /* long exp 2/3 in unit of 1/16 line, pg 38,
			   note frame length start with 0x7b0,
			   and SENSOR_BAYER_DEFAULT_MAX_COARSE_DIFF=3 */
	{0x3502, 0xd0}, /* long exp 3/3 in unit of 1/16 line, pg 38.
			   Two lines of integration time. */
	{0x350a, 0x00}, /* gain output to sensor, pg 38 */
	{0x350b, 0x00}, /* gain output to sensor, pg 38 */
	{0x4801, 0x0f}, /* MIPI control01 pg 125 */
	{0x300e, 0x0c}, /* SC_MIPI_SC_CTRL0 pg 73 */
	{0x4803, 0x50}, /* MIPI CTRL3 pg 91 */
	{0x4800, 0x34}, /* MIPI CTRl0 idle and short line pg 89 */
	{IMX073_TABLE_END, 0x0000}
};

static struct imx073_reg mode_1264x704[] = {
	{0x3600, 0x54}, /* analog pg 108 */
	{0x3601, 0x05}, /* analog pg 108 */
	{0x3604, 0x40}, /* analog pg 108 */
	{0x3705, 0xdb}, /* analog pg 108 */
	{0x370a, 0x81}, /* analog pg 108 */
	{0x3615, 0x52}, /* analog pg 108 */
	{0x3810, 0x40}, /* TIMING HVOFFS both are zero pg 80 */
	{0x3836, 0x41}, /* TIMING HVPAD both are zero pg 82 */
	{0x4000, 0x05}, /* BLC enabled pg 120 */
	{0x401c, 0x42}, /* reserved pg 120 */
	{0x5046, 0x09}, /* ISP control isp disable awbg disable pg 133 */
	{0x3010, 0x00}, /* PLL control01 DIVM [3:0] DIVS [7:4] div 1 pg 99 */
	{0x3503, 0x00}, /* AEC auto AGC auto gain has no latch delay. pg 38 */
	{0x3613, 0xc4}, /* analog pg 108 */

	{0x3621, 0xaf}, /* analog horizontal binning/sampling not enabled.
			   pg 108 */
	{0x3632, 0x55}, /* analog pg 108 */
	{0x3703, 0x9a}, /* analog pg 108 */
	{0x370c, 0x00}, /* analog pg 108 */
	{0x370d, 0x42}, /* analog pg 108 */
	{0x3713, 0x22}, /* analog pg 108 */
	{0x3800, 0x02}, /* HREF start point higher 4 bits [3:0] pg 108 */
	{0x3801, 0x54}, /* HREF start point lower  8 bits [7:0] pg 108 */
	{0x3802, 0x00}, /* VREF start point higher 4 bits [3:0] pg 108 */
	{0x3803, 0x0c}, /* VREF start point [7:0] pg 108 */
	{0x3804, 0x05}, /* HREF width  higher 4 bits [3:0] pg 108 */
	{0x3805, 0x00}, /* HREF width  lower  8 bits [7:0] pg 108 */
	{0x3806, 0x02}, /* VREF height higher 4 bits [3:0] pg 109 */
	{0x3807, 0xd0}, /* VREF height lower  8 bits [7:0] pg 109 */
	{0x3808, 0x05}, /* DVP horizontal output size higher 4 bits [3:0]
			   pg 109 */
	{0x3809, 0x00}, /* DVP horizontal output size lower  8 bits [7:0]
			   pg 109 */
	{0x380a, 0x02}, /* DVP vertical   output size higher 4 bits [3:0]
			   pg 109 */
	{0x380b, 0xd0}, /* DVP vertical   output size lower  8 bits [7:0]
			   pg 109 */
	{0x380c, 0x08}, /* total horizontal size higher 5 bits [4:0] pg 109,
			   line length */
	{0x380d, 0x72}, /* total horizontal size lower  8 bits [7:0] pg 109,
			   line length */
	{0x380e, 0x02}, /* total vertical size higher 5 bits [4:0] pg 109,
			   frame length */
	{0x380f, 0xe4}, /* total vertical size lower  8 bits [7:0] pg 109,
			   frame length */
	{0x3818, 0xc1}, /* timing control reg18 mirror & dkhf pg 110 */
	{0x381a, 0x3c}, /* HS mirror adjustment pg 110 */
	{0x3a0d, 0x06}, /* b60 max pg 113 */
	{0x3c01, 0x34}, /* 5060HZ_CTRL01 pg 116 */
	{0x3007, 0x3b}, /* clock enable03 pg 98 */
	{0x5059, 0x80}, /* => NOT found */
	{0x3003, 0x03}, /* reset MIPI and DVP pg 97 */
	{0x3500, 0x04}, /* long exp 1/3 in unit of 1/16 line, pg 38 */
	{0x3501, 0xa5}, /* long exp 2/3 in unit of 1/16 line, pg 38,
			   note frame length start with 0x7b0,
			   and SENSOR_BAYER_DEFAULT_MAX_COARSE_DIFF=3 */
	{0x3502, 0x10}, /* long exp 3/3 in unit of 1/16 line, pg 38.
			   Two lines of integration time. */
	{0x350a, 0x00}, /* gain output to sensor, pg 38 */
	{0x350b, 0x00}, /* gain output to sensor, pg 38 */
	{0x4801, 0x0f}, /* MIPI control01 pg 125 */
	{0x300e, 0x0c}, /* SC_MIPI_SC_CTRL0 pg 73 */
	{0x4803, 0x50}, /* MIPI CTRL3 pg 91 */
	{0x4800, 0x24}, /* MIPI CTRl0 idle and short line pg 89 */
	{0x300f, 0x8b}, /* PLL control00 R_SELD5 [7:6] div by 4 R_DIVL [2]
			   two lane div 1 SELD2P5 [1:0] div 2.5 pg 99 */

	{0x3711, 0x24},
	{0x3713, 0x92},
	{0x3714, 0x17},
	{0x381c, 0x10},
	{0x381d, 0x82},
	{0x381e, 0x05},
	{0x381f, 0xc0},
	{0x3821, 0x20},
	{0x3824, 0x23},
	{0x3825, 0x2c},
	{0x3826, 0x00},
	{0x3827, 0x0c},
	{0x3623, 0x01},
	{0x3633, 0x24},
	{0x3632, 0x5f},
	{0x401f, 0x03},

	{IMX073_TABLE_END, 0x0000}
};

static struct imx073_reg mode_end[] = {
	{0x3212, 0x00}, /* SRM_GROUP_ACCESS (group hold begin) */
	{0x3003, 0x01}, /* reset DVP pg 97 */
	{0x3212, 0x10}, /* SRM_GROUP_ACCESS (group hold end) */
	{0x3212, 0xa0}, /* SRM_GROUP_ACCESS (group hold launch) */
	{0x3008, 0x02}, /* SYSTEM_CTRL0 mipi suspend mask pg 98 */

	/*	{FAST_SETMODE_END, 0}, */
	{IMX073_TABLE_END, 0x0000}
};

// MOBII_CHANGE  dkhan : for fullhd video [
static struct imx073_reg mode_2720x1530[] =
{ 
    {0x0100, 0x00},
	{0x0307,	0x24},
	{0x302B,	0x38},	
	{0x30E5,	0x04},
	{0x3300,	0x00},
	{0x0101,	0x03},
//	{0x0202, 	0x03},
//	{0x0203, 	0xe8},
	
	{0x300A,	0x80},
	{0x3014,	0x08},
	{0x3015,	0x37},
	{0x3017,	0x60},
	{0x301C,	0x01},
	{0x3031,	0x28},
	{0x3040,	0x00},
	{0x3041,	0x60},
	{0x3047,	0x10},
	{0x3051,	0x24},
	{0x3053,	0x34},
	{0x3055,	0x3B},
	{0x3057,	0xC0},
	{0x3060,	0x30},
	{0x3065,	0x00},
	{0x30A1,	0x03},
	{0x30A3,	0x01},
	{0x30AA,	0x88},
	{0x30AB,	0x1C},
	{0x30B0,	0x32},
	{0x30B2,	0x83},
	{0x30D3,	0x04},
	{0x310C,	0xE9},
	{0x310D,	0x00},
	{0x316B,	0x14},
	{0x316D,	0x3B},
	{0x31A4,	0xD8},
	{0x31A6,	0x17},
	{0x31AC,	0xCF},
	{0x31AE,	0xF1},
	{0x31B4,	0xD8},
	{0x31B6,	0x17},
	{0x3302,	0x0A},
	{0x3303,	0x09},
	{0x3304,	0x05},
	{0x3305,	0x04},
	{0x3306,	0x15},
	{0x3307,	0x03},
	{0x3308,	0x13},
	{0x3309,	0x05},
	{0x330A,	0x0B},
	{0x330B,	0x04},
	{0x330C,	0x0B},
	{0x330D,	0x06},

	{0x0340,	0x06},	// frame_length
	{0x0341,	0x1f},

	{0x0344,	0x01},	// x_addr_start
	{0x0345,	0x18},
	{0x0346,	0x01},	// y_addr_start
	{0x0347,	0xd5},
	{0x0348,	0x0b},	// x_addr_end
	{0x0349,	0xb7},
	{0x034A,	0x07},	// y_addr_end
	{0x034B,	0xce},

	{0x034C,	0x0a},	// x_out size
	{0x034D,	0xa0},
	{0x034E,	0x05},	// y_out size
	{0x034F,	0xfa},
	{0x0381,	0x01},	// x_even_inc
	{0x0383,	0x01},	// x_odd_inc
	{0x0385,	0x01},	// y_even_inc
	{0x0387,	0x01},	// y_odd_inc
	{0x3001,	0x00},	// HMODE ADD
	{0x3016,	0x06},	// VMODE ADD
	{0x30E8,	0x06},	// HADDAVE
	{0x3301,	0x00},	// RGLANESEL

	{0x0100,	0x01},

	{IMX073_TABLE_END, 0x0000}
};
// MOBII_CHANGE  dkhan : for fullhd video ]


enum {
	IMX073_MODE_3264x2448,
//MOBII_CHANGE_S dongki.han@lge.com 20120308 : merge camera sensor su660 gb
	IMX073_MODE_3264x1224,
//MOBII_CHANGE_E dongki.han@lge.com 20120308 : merge camera sensor su660 gb
// MOBII_CHANGE  dkhan : for fullhd video [
	IMX073_MODE_2720x1530,
// MOBII_CHANGE  dkhan : for fullhd video ]
	//IMX073_MODE_1296x972,
	//IMX073_MODE_2080x1164,
	//IMX073_MODE_1264x704
};

static struct imx073_reg *mode_table[] = {
	[IMX073_MODE_3264x2448] = mode_3264x2448,
//MOBII_CHANGE_S dongki.han@lge.com 20120308 : merge camera sensor su660 gb
	[IMX073_MODE_3264x1224] = mode_3264x1224,
//MOBII_CHANGE_E dongki.han@lge.com 20120308 : merge camera sensor su660 gb
// MOBII_CHANGE  dkhan : for fullhd video [
	[IMX073_MODE_2720x1530] = mode_2720x1530,
// MOBII_CHANGE  dkhan : for fullhd video ]	
	//[IMX073_MODE_1296x972] = mode_1296x972,
	//[IMX073_MODE_2080x1164] = mode_2080x1164,
	//[IMX073_MODE_1264x704] = mode_1264x704
};

/* 2 regs to program frame length */
static inline void imx073_get_frame_length_regs(struct imx073_reg *regs,
						u32 frame_length)
{
	regs->addr = 0x0340;
	regs->val = (frame_length >> 8) & 0xff;
	(regs + 1)->addr = 0x0341;
	(regs + 1)->val = (frame_length) & 0xff;
}

/* 3 regs to program coarse time */
static inline void imx073_get_coarse_time_regs(struct imx073_reg *regs,
                                               u32 coarse_time)
{
	regs->addr = 0x202;
	regs->val = (coarse_time >> 8) & 0xff;
	(regs + 1)->addr = 0x203;
	(regs + 1)->val = (coarse_time) & 0xff;
}

/* 1 reg to program gain */
static inline void imx073_get_gain_reg(struct imx073_reg *regs, u16 gain)
{
	regs->addr = 0x205;
	regs->val = gain;
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

	do {
		err = i2c_transfer(client->adapter, &msg, 1);
		if (err == 1)
			return 0;
		retry++;
		pr_err("imx073: i2c transfer failed, retrying %x %x\n",
		       addr, val);
		msleep(3);
	} while (retry <= IMX073_MAX_RETRIES);

	return err;
}

static int imx073_write_table(struct i2c_client *client,
			      const struct imx073_reg table[],
			      const struct imx073_reg override_list[],
			      int num_override_regs)
{
	int err;
	const struct imx073_reg *next;
	int i;
	u16 val;
    u16 addr;

	for (next = table; next->addr != IMX073_TABLE_END; next++) {
		if (next->addr == IMX073_TABLE_WAIT_MS) {
			msleep(next->val);
			continue;
		}

		val = next->val;

		err = imx073_write_reg(client, next->addr, val);
		if (err)
			return err;
	}

		/* When an override list is passed in, replace the reg */
		/* value to write if the reg is in the list            */
    err = imx073_write_reg(client, 0x104, 0x01);
    if (err)
        return err;
    
		if (override_list) {
			for (i = 0; i < num_override_regs; i++) {
	        addr = override_list[i].addr;
					val = override_list[i].val;
//		}

			err = imx073_write_reg(client, addr, val);
			if (err)
				return err;
			}
		}

    err = imx073_write_reg(client, 0x104, 0x00);
		if (err)
			return err;

	return 0;
}

static int imx073_set_mode(struct imx073_info *info, struct imx073_mode *mode)
{
	int sensor_mode;
	int err;
	struct imx073_reg reg_list[6] = {NULL};

	pr_info("%s: xres %u yres %u framelength %u coarsetime %u gain %u\n",
		__func__, mode->xres, mode->yres, mode->frame_length,
		mode->coarse_time, mode->gain);
	if (mode->xres == 3264 && mode->yres == 2448)
		sensor_mode = IMX073_MODE_3264x2448;
//MOBII_CHANGE_S dongki.han@lge.com 20120308 : merge camera sensor su660 gb
	else if (mode->xres == 3264 && mode->yres == 1224)
		sensor_mode = IMX073_MODE_3264x1224;
//MOBII_CHANGE_E dongki.han@lge.com 20120308 : merge camera sensor su660 gb
// MOBII_CHANGE  dkhan : for fullhd video [
	else if (mode->xres == 2720 && mode->yres == 1530)
		sensor_mode = IMX073_MODE_2720x1530;
// MOBII_CHANGE  dkhan : for fullhd video ]
	else {
		pr_err("%s: invalid resolution supplied to set mode %d %d\n",
		       __func__, mode->xres, mode->yres);
		return -EINVAL;
	}

	imx073_get_frame_length_regs(&reg_list[0], mode->frame_length);
	imx073_get_coarse_time_regs(&reg_list[2], mode->coarse_time);
	imx073_get_gain_reg(&reg_list[4], mode->gain);

	err = imx073_write_table(info->i2c_client, mode_table[sensor_mode],
		reg_list, 6);
	if (err)
		return err;

	info->mode = sensor_mode;
	return 0;
}

static int imx073_set_frame_length(struct imx073_info *info, u32 frame_length)
{
	struct imx073_reg reg_list[2];
	int i = 0;
	int ret;

    //pr_err("[Karl-imx073] FrameLength = %u\n", frame_length);
    
	imx073_get_frame_length_regs(reg_list, frame_length);
	ret = imx073_write_reg(info->i2c_client, 0x0104, 0x01);
	if (ret)
		return ret;

	for (i = 0; i < 2; i++)	{
		ret = imx073_write_reg(info->i2c_client, reg_list[i].addr,
			reg_list[i].val);
		if (ret)
			return ret;
	}

	ret = imx073_write_reg(info->i2c_client, 0x0104, 0x0);
	if (ret)
		return ret;

	return 0;
}

static int imx073_set_coarse_time(struct imx073_info *info, u32 coarse_time)
{
	int ret;

	struct imx073_reg reg_list[3];
	int i = 0;

    //pr_err("[Karl-imx073] coarse_time = %u\n", coarse_time);
    
	imx073_get_coarse_time_regs(reg_list, coarse_time);

	ret = imx073_write_reg(info->i2c_client, 0x104, 0x01);
	if (ret)
		return ret;

	for (i = 0; i < 3; i++)	{
		ret = imx073_write_reg(info->i2c_client, reg_list[i].addr,
			reg_list[i].val);
		if (ret)
			return ret;
	}

	ret = imx073_write_reg(info->i2c_client, 0x104, 0x0);
	if (ret)
		return ret;

	return 0;
}

static int imx073_set_gain(struct imx073_info *info, u16 gain)
{
	int ret;
	struct imx073_reg reg_list;

    //pr_err("[Karl-imx073] Gain = %u\n", gain);

	imx073_get_gain_reg(&reg_list, gain);

	ret = imx073_write_reg(info->i2c_client, 0x104, 0x1);
	if (ret)
		return ret;

	ret = imx073_write_reg(info->i2c_client, reg_list.addr, reg_list.val);

	ret = imx073_write_reg(info->i2c_client, 0x104, 0x0);
	if (ret)
		return ret;

	return ret;
}

static int imx073_get_status(struct imx073_info *info, u8 *status)
{
	int err;

	*status = 0;
	err = imx073_read_reg(info->i2c_client, 0x205, status);
	pr_info("%s: status=%u err=%d\n", __func__, *status, err);
	return err;
}

//static int imx073_test_pattern(struct imx073_info *info,
//			       enum imx073_test_pattern pattern)
//{
//	if (pattern >= ARRAY_SIZE(test_pattern_modes))
//		return -EINVAL;

//	return imx073_write_table(info->i2c_client,
//				  test_pattern_modes[pattern],
//				  NULL, 0);
//      return 0;
//}

static long imx073_ioctl(struct file *file,
			 unsigned int cmd, unsigned long arg)
{
	int err;
	struct imx073_info *info = file->private_data;

	switch (cmd) {
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
	//case IMX073_IOCTL_TEST_PATTERN:
	//{
	//	err = imx073_test_pattern(info, (enum imx073_test_pattern) arg);
	//	if (err)
	//		pr_err("%s %d %d\n", __func__, __LINE__, err);
	//	return err;
	//}
	default:
		return -EINVAL;
	}
	return 0;
}

static struct imx073_info *info;

static int imx073_open(struct inode *inode, struct file *file)
{
	u8 status;

	pr_info("%s\n", __func__);
	file->private_data = info;
	if (info->pdata && info->pdata->power_on)
		info->pdata->power_on();
	imx073_get_status(info, &status);
	return 0;
}

int imx073_release(struct inode *inode, struct file *file)
{
	if (info->pdata && info->pdata->power_off)
		info->pdata->power_off();
	file->private_data = NULL;
	return 0;
}


static const struct file_operations imx073_fileops = {
	.owner = THIS_MODULE,
	.open = imx073_open,
	.unlocked_ioctl = imx073_ioctl,
	.release = imx073_release,
};

static struct miscdevice imx073_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "imx073",
	.fops = &imx073_fileops,
};

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
	return 0;
}

static int imx073_remove(struct i2c_client *client)
{
	struct imx073_info *info;
	info = i2c_get_clientdata(client);
	misc_deregister(&imx073_device);
	kfree(info);
	return 0;
}

static const struct i2c_device_id imx073_id[] = {
	{ "imx073", 0 },
	{ },
};

MODULE_DEVICE_TABLE(i2c, imx073_id);

static struct i2c_driver imx073_i2c_driver = {
	.driver = {
		.name = "imx073",
		.owner = THIS_MODULE,
	},
	.probe = imx073_probe,
	.remove = imx073_remove,
	.id_table = imx073_id,
};

static int __init imx073_init(void)
{
	pr_info("imx073 sensor driver loading\n");
	return i2c_add_driver(&imx073_i2c_driver);
}

static void __exit imx073_exit(void)
{
	i2c_del_driver(&imx073_i2c_driver);
}

module_init(imx073_init);
module_exit(imx073_exit);

