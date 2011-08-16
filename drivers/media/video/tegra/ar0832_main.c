/*
* ar0832_main.c - Aptina AR0832 8M Bayer type sensor driver
*
* Copyright (c) 2011, NVIDIA, All Rights Reserved.
*
* This file is licensed under the terms of the GNU General Public License
* version 2. This program is licensed "as is" without any warranty of any
* kind, whether express or implied.
*/

#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <mach/hardware.h>
#include <linux/gpio.h>
#include <asm/atomic.h>
#include <linux/regulator/consumer.h>
#include <media/ar0832_main.h>

#define POS_LOW 50
#define POS_HIGH 1000
#define SETTLETIME_MS 100

struct ar0832_sensor_info {
	int mode;
	struct ar0832_stereo_region region;
};

struct ar0832_focuser_info {
	struct ar0832_focuser_config config;
	int focuser_init_flag;
};

struct ar0832_power_rail {
	struct regulator *sen_1v8_reg;
	struct regulator *sen_2v8_reg;
};

struct ar0832_dev {
	struct ar0832_sensor_info *sensor_info;
	struct ar0832_focuser_info *focuser_info;
	struct ar0832_platform_data *pdata;
	struct i2c_client *i2c_client;
	struct mutex ar0832_camera_lock;
	struct miscdevice misc_dev;
	struct ar0832_power_rail power_rail;
	atomic_t in_use;
	char dname[20];
	int is_stereo;
};

/* stereo */
static u16 DefaultImageWidth =  1200;
static u16 DefaultImageHeight =  680;
#define UpperByte16to8(x) ((u8)((x & 0xFF00) >> 8))
#define LowerByte16to8(x) ((u8)(x & 0x00FF))

#define ar0832_TABLE_WAIT_MS 0
#define ar0832_TABLE_END 1
#define ar0832_MAX_RETRIES 3

#define AR0832_RESET_REG	0x301A
#define AR0832_ID_REG		0x31FC

/* AR0832_RESET_REG */
#define AR0832_RESET_REG_GROUPED_PARAMETER_HOLD		(1 << 15)
#define AR0832_RESET_REG_GAIN_INSERT			(1 << 14)
#define AR0832_RESET_REG_SMIA_SERIALIZER_DIS		(1 << 12)
#define AR0832_RESET_REG_RESTART_BAD			(1 << 10)
#define AR0832_RESET_REG_MASK_BAD			(1 << 9)
#define AR0832_RESET_REG_GPI_EN				(1 << 8)
#define AR0832_RESET_REG_PARALLEL_EN			(1 << 7)
#define AR0832_RESET_REG_DRIVE_PINS			(1 << 6)
#define AR0832_RESET_REG_STDBY_EOF			(1 << 4)
#define AR0832_RESET_REG_LOCK_REG			(1 << 3)
#define AR0832_RESET_REG_STREAM				(1 << 2)
#define AR0832_RESET_REG_RESTART			(1 << 1)
#define AR0832_RESET_REG_RESET				(1 << 0)

static struct ar0832_reg mode_start[] = {
	{ar0832_TABLE_END, 0x0000}
};

static struct ar0832_reg mode_3264X2448[] = {
	{0x301A, 0x0058},	/* RESET_REGISTER */
	{0x301A, 0x0050},	/* RESET_REGISTER */
	{0x0104, 0x0100},	/* GROUPED_PARAMETER_HOLD */
	{0x3064, 0x7800},	/* RESERVED_MFR_3064 */
	{0x31AE, 0x0202},	/* SERIAL_FORMAT */
	{0x31B0, 0x0083},	/* FRAME_PREAMBLE */
	{0x31B2, 0x004D},	/* LINE_PREAMBLE */
	{0x31B4, 0x0E77},	/* MIPI_TIMING_0 */
	{0x31B6, 0x0D20},	/* MIPI_TIMING_1 */
	{0x31B8, 0x020E},	/* MIPI_TIMING_2 */
	{0x31BA, 0x0710},	/* MIPI_TIMING_3 */
	{0x31BC, 0x2A0D},	/* MIPI_TIMING_4 */
	{ar0832_TABLE_WAIT_MS, 0x0005},
	{0x0112, 0x0A0A},	/* CCP_DATA_FORMAT */

	{0x3044, 0x0590},	/* RESERVED_MFR_3044 */
	{0x306E, 0xFC80},	/* DATAPATH_SELECT */
	{0x30B2, 0xC000},	/* RESERVED_MFR_30B2 */
	{0x30D6, 0x0800},	/* RESERVED_MFR_30D6 */
	{0x316C, 0xB42F},	/* RESERVED_MFR_316C */
	{0x316E, 0x869A},	/* RESERVED_MFR_316E */
	{0x3170, 0x210E},	/* RESERVED_MFR_3170 */
	{0x317A, 0x010E},	/* RESERVED_MFR_317A */
	{0x31E0, 0x1FB9},	/* RESERVED_MFR_31E0 */
	{0x31E6, 0x07FC},	/* RESERVED_MFR_31E6 */
	{0x37C0, 0x0000},	/* P_GR_Q5 */
	{0x37C2, 0x0000},	/* P_RD_Q5 */
	{0x37C4, 0x0000},	/* P_BL_Q5 */
	{0x37C6, 0x0000},	/* P_GB_Q5 */
	{0x3E00, 0x0011},	/* RESERVED_MFR_3E00 */
	{0x3E02, 0x8801},	/* RESERVED_MFR_3E02 */
	{0x3E04, 0x2801},	/* RESERVED_MFR_3E04 */
	{0x3E06, 0x8449},	/* RESERVED_MFR_3E06 */
	{0x3E08, 0x6841},	/* RESERVED_MFR_3E08 */
	{0x3E0A, 0x400C},	/* RESERVED_MFR_3E0A */
	{0x3E0C, 0x1001},	/* RESERVED_MFR_3E0C */
	{0x3E0E, 0x2603},	/* RESERVED_MFR_3E0E */
	{0x3E10, 0x4B41},	/* RESERVED_MFR_3E10 */
	{0x3E12, 0x4B24},	/* RESERVED_MFR_3E12 */
	{0x3E14, 0xA3CF},	/* RESERVED_MFR_3E14 */
	{0x3E16, 0x8802},	/* RESERVED_MFR_3E16 */
	{0x3E18, 0x84FF},	/* RESERVED_MFR_3E18 */
	{0x3E1A, 0x8601},	/* RESERVED_MFR_3E1A */
	{0x3E1C, 0x8401},	/* RESERVED_MFR_3E1C */
	{0x3E1E, 0x840A},	/* RESERVED_MFR_3E1E */
	{0x3E20, 0xFF00},	/* RESERVED_MFR_3E20 */
	{0x3E22, 0x8401},	/* RESERVED_MFR_3E22 */
	{0x3E24, 0x00FF},	/* RESERVED_MFR_3E24 */
	{0x3E26, 0x0088},	/* RESERVED_MFR_3E26 */
	{0x3E28, 0x2E8A},	/* RESERVED_MFR_3E28 */
	{0x3E30, 0x0000},	/* RESERVED_MFR_3E30 */
	{0x3E32, 0x8801},	/* RESERVED_MFR_3E32 */
	{0x3E34, 0x4029},	/* RESERVED_MFR_3E34 */
	{0x3E36, 0x00FF},	/* RESERVED_MFR_3E36 */
	{0x3E38, 0x8469},	/* RESERVED_MFR_3E38 */
	{0x3E3A, 0x00FF},	/* RESERVED_MFR_3E3A */
	{0x3E3C, 0x2801},	/* RESERVED_MFR_3E3C */
	{0x3E3E, 0x3E2A},	/* RESERVED_MFR_3E3E */
	{0x3E40, 0x1C01},	/* RESERVED_MFR_3E40 */
	{0x3E42, 0xFF84},	/* RESERVED_MFR_3E42 */
	{0x3E44, 0x8401},	/* RESERVED_MFR_3E44 */
	{0x3E46, 0x0C01},	/* RESERVED_MFR_3E46 */
	{0x3E48, 0x8401},	/* RESERVED_MFR_3E48 */
	{0x3E4A, 0x00FF},	/* RESERVED_MFR_3E4A */
	{0x3E4C, 0x8402},	/* RESERVED_MFR_3E4C */
	{0x3E4E, 0x8984},	/* RESERVED_MFR_3E4E */
	{0x3E50, 0x6628},	/* RESERVED_MFR_3E50 */
	{0x3E52, 0x8340},	/* RESERVED_MFR_3E52 */
	{0x3E54, 0x00FF},	/* RESERVED_MFR_3E54 */
	{0x3E56, 0x4A42},	/* RESERVED_MFR_3E56 */
	{0x3E58, 0x2703},	/* RESERVED_MFR_3E58 */
	{0x3E5A, 0x6752},	/* RESERVED_MFR_3E5A */
	{0x3E5C, 0x3F2A},	/* RESERVED_MFR_3E5C */
	{0x3E5E, 0x846A},	/* RESERVED_MFR_3E5E */
	{0x3E60, 0x4C01},	/* RESERVED_MFR_3E60 */
	{0x3E62, 0x8401},	/* RESERVED_MFR_3E62 */
	{0x3E66, 0x3901},	/* RESERVED_MFR_3E66 */
	{0x3E90, 0x2C01},	/* RESERVED_MFR_3E90 */
	{0x3E98, 0x2B02},	/* RESERVED_MFR_3E98 */
	{0x3E92, 0x2A04},	/* RESERVED_MFR_3E92 */
	{0x3E94, 0x2509},	/* RESERVED_MFR_3E94 */
	{0x3E96, 0x0000},	/* RESERVED_MFR_3E96 */
	{0x3E9A, 0x2905},	/* RESERVED_MFR_3E9A */
	{0x3E9C, 0x00FF},	/* RESERVED_MFR_3E9C */
	{0x3ECC, 0x00EB},	/* RESERVED_MFR_3ECC */
	{0x3ED0, 0x1E24},	/* RESERVED_MFR_3ED0 */
	{0x3ED4, 0xAFC4},	/* RESERVED_MFR_3ED4 */
	{0x3ED6, 0x909B},	/* RESERVED_MFR_3ED6 */
	{0x3EE0, 0x2424},	/* RESERVED_MFR_3EE0 */
	{0x3EE2, 0x9797},	/* RESERVED_MFR_3EE2 */
	{0x3EE4, 0xC100},	/* RESERVED_MFR_3EE4 */
	{0x3EE6, 0x0540},	/* RESERVED_MFR_3EE6 */
	{0x3174, 0x8000},	/* RESERVED_MFR_3174 */
	{0x0300, 0x0004},	/* VT_PIX_CLK_DIV */
	{0x0302, 0x0001},	/* VT_SYS_CLK_DIV */
	{0x0304, 0x0002},	/* PRE_PLL_CLK_DIV */
	{0x0306, 0x0040},	/* PLL_MULTIPLIER */
	{0x0308, 0x000A},	/* OP_PIX_CLK_DIV */
	{0x030A, 0x0001},	/* OP_SYS_CLK_DIV */
	{ar0832_TABLE_WAIT_MS, 0x0001},
	{0x3064, 0x7400},	/* RESERVED_MFR_3064 */
	{0x0104, 0x0100},	/* GROUPED_PARAMETER_HOLD */
	{0x0344, 0x0004},	/* X_ADDR_START */
	{0x0348, 0x0CCB},	/* X_ADDR_END */
	{0x0346, 0x0004},	/* Y_ADDR_START */
	{0x034A, 0x099B},	/* Y_ADDR_END */
	{0x034C, 0x0CC8},	/* X_OUTPUT_SIZE */
	{0x034E, 0x0998},	/* Y_OUTPUT_SIZE */
	{0x3040, 0xC041},	/* READ_MODE */
	{0x306E, 0xFC80},	/* DATAPATH_SELECT */
	{0x0400, 0x0000},	/* SCALING_MODE */
	{0x0404, 0x0010},	/* SCALE_M */
	{0x3178, 0x0000},	/* RESERVED_MFR_3178 */
	{0x3ED0, 0x1E24},	/* RESERVED_MFR_3ED0 */
	{0x0400, 0x0000},	/* SCALING_MODE */
	{0x0404, 0x0010},	/* SCALE_M */
	{0x0342, 0x133C},	/* LINE_LENGTH_PCK */
	{0x0340, 0x0A27},	/* FRAME_LENGTH_LINES */
	{0x0202, 0x0A27},	/* COARSE_INTEGRATION_TIME */
	{0x3014, 0x09DC},	/* FINE_INTEGRATION_TIME */
	{0x3010, 0x0078},	/* FINE_CORRECTION */
	{0x301A, 0x8250},	/* RESET_REGISTER */
	{0x301A, 0x8650},	/* RESET_REGISTER */
	{0x301A, 0x8658},	/* RESET_REGISTER */
	/* gain */
	{0x3056, 0x10AA},	/* gain */
	{0x3058, 0x10AA},	/* gain */
	{0x305a, 0x10AA},	/* gain */
	{0x305c, 0x10AA},	/* gain */
	{0x0104, 0x0000},	/* GROUPED_PARAMETER_HOLD */
	{0x301A, 0x065C},	/* RESET_REGISTER */
	{ar0832_TABLE_END, 0x0000}
};

static struct ar0832_reg mode_2880X1620[] = {
	{0x301A, 0x0058},	/* RESET_REGISTER */
	{0x301A, 0x0050},	/* RESET_REGISTER */
	{0x0104, 0x0100},	/* GROUPED_PARAMETER_HOLD */
	{0x3064, 0x7800},	/* RESERVED_MFR_3064 */
	{0x31AE, 0x0202},	/* SERIAL_FORMAT */
	{0x31B0, 0x0083},	/* FRAME_PREAMBLE */
	{0x31B2, 0x004D},	/* LINE_PREAMBLE */
	{0x31B4, 0x0E77},	/* MIPI_TIMING_0 */
	{0x31B6, 0x0D20},	/* MIPI_TIMING_1 */
	{0x31B8, 0x020E},	/* MIPI_TIMING_2 */
	{0x31BA, 0x0710},	/* MIPI_TIMING_3 */
	{0x31BC, 0x2A0D},	/* MIPI_TIMING_4 */
	{ar0832_TABLE_WAIT_MS, 0x0005},
	{0x0112, 0x0A0A},	/* CCP_DATA_FORMAT */

	{0x3044, 0x0590},	/* RESERVED_MFR_3044 */
	{0x306E, 0xFC80},	/* DATAPATH_SELECT */
	{0x30B2, 0xC000},	/* RESERVED_MFR_30B2 */
	{0x30D6, 0x0800},	/* RESERVED_MFR_30D6 */
	{0x316C, 0xB42F},	/* RESERVED_MFR_316C */
	{0x316E, 0x869A},	/* RESERVED_MFR_316E */
	{0x3170, 0x210E},	/* RESERVED_MFR_3170 */
	{0x317A, 0x010E},	/* RESERVED_MFR_317A */
	{0x31E0, 0x1FB9},	/* RESERVED_MFR_31E0 */
	{0x31E6, 0x07FC},	/* RESERVED_MFR_31E6 */
	{0x37C0, 0x0000},	/* P_GR_Q5 */
	{0x37C2, 0x0000},	/* P_RD_Q5 */
	{0x37C4, 0x0000},	/* P_BL_Q5 */
	{0x37C6, 0x0000},	/* P_GB_Q5 */
	{0x3E00, 0x0011},	/* RESERVED_MFR_3E00 */
	{0x3E02, 0x8801},	/* RESERVED_MFR_3E02 */
	{0x3E04, 0x2801},	/* RESERVED_MFR_3E04 */
	{0x3E06, 0x8449},	/* RESERVED_MFR_3E06 */
	{0x3E08, 0x6841},	/* RESERVED_MFR_3E08 */
	{0x3E0A, 0x400C},	/* RESERVED_MFR_3E0A */
	{0x3E0C, 0x1001},	/* RESERVED_MFR_3E0C */
	{0x3E0E, 0x2603},	/* RESERVED_MFR_3E0E */
	{0x3E10, 0x4B41},	/* RESERVED_MFR_3E10 */
	{0x3E12, 0x4B24},	/* RESERVED_MFR_3E12 */
	{0x3E14, 0xA3CF},	/* RESERVED_MFR_3E14 */
	{0x3E16, 0x8802},	/* RESERVED_MFR_3E16 */
	{0x3E18, 0x84FF},	/* RESERVED_MFR_3E18 */
	{0x3E1A, 0x8601},	/* RESERVED_MFR_3E1A */
	{0x3E1C, 0x8401},	/* RESERVED_MFR_3E1C */
	{0x3E1E, 0x840A},	/* RESERVED_MFR_3E1E */
	{0x3E20, 0xFF00},	/* RESERVED_MFR_3E20 */
	{0x3E22, 0x8401},	/* RESERVED_MFR_3E22 */
	{0x3E24, 0x00FF},	/* RESERVED_MFR_3E24 */
	{0x3E26, 0x0088},	/* RESERVED_MFR_3E26 */
	{0x3E28, 0x2E8A},	/* RESERVED_MFR_3E28 */
	{0x3E30, 0x0000},	/* RESERVED_MFR_3E30 */
	{0x3E32, 0x8801},	/* RESERVED_MFR_3E32 */
	{0x3E34, 0x4029},	/* RESERVED_MFR_3E34 */
	{0x3E36, 0x00FF},	/* RESERVED_MFR_3E36 */
	{0x3E38, 0x8469},	/* RESERVED_MFR_3E38 */
	{0x3E3A, 0x00FF},	/* RESERVED_MFR_3E3A */
	{0x3E3C, 0x2801},	/* RESERVED_MFR_3E3C */
	{0x3E3E, 0x3E2A},	/* RESERVED_MFR_3E3E */
	{0x3E40, 0x1C01},	/* RESERVED_MFR_3E40 */
	{0x3E42, 0xFF84},	/* RESERVED_MFR_3E42 */
	{0x3E44, 0x8401},	/* RESERVED_MFR_3E44 */
	{0x3E46, 0x0C01},	/* RESERVED_MFR_3E46 */
	{0x3E48, 0x8401},	/* RESERVED_MFR_3E48 */
	{0x3E4A, 0x00FF},	/* RESERVED_MFR_3E4A */
	{0x3E4C, 0x8402},	/* RESERVED_MFR_3E4C */
	{0x3E4E, 0x8984},	/* RESERVED_MFR_3E4E */
	{0x3E50, 0x6628},	/* RESERVED_MFR_3E50 */
	{0x3E52, 0x8340},	/* RESERVED_MFR_3E52 */
	{0x3E54, 0x00FF},	/* RESERVED_MFR_3E54 */
	{0x3E56, 0x4A42},	/* RESERVED_MFR_3E56 */
	{0x3E58, 0x2703},	/* RESERVED_MFR_3E58 */
	{0x3E5A, 0x6752},	/* RESERVED_MFR_3E5A */
	{0x3E5C, 0x3F2A},	/* RESERVED_MFR_3E5C */
	{0x3E5E, 0x846A},	/* RESERVED_MFR_3E5E */
	{0x3E60, 0x4C01},	/* RESERVED_MFR_3E60 */
	{0x3E62, 0x8401},	/* RESERVED_MFR_3E62 */
	{0x3E66, 0x3901},	/* RESERVED_MFR_3E66 */
	{0x3E90, 0x2C01},	/* RESERVED_MFR_3E90 */
	{0x3E98, 0x2B02},	/* RESERVED_MFR_3E98 */
	{0x3E92, 0x2A04},	/* RESERVED_MFR_3E92 */
	{0x3E94, 0x2509},	/* RESERVED_MFR_3E94 */
	{0x3E96, 0x0000},	/* RESERVED_MFR_3E96 */
	{0x3E9A, 0x2905},	/* RESERVED_MFR_3E9A */
	{0x3E9C, 0x00FF},	/* RESERVED_MFR_3E9C */
	{0x3ECC, 0x00EB},	/* RESERVED_MFR_3ECC */
	{0x3ED0, 0x1E24},	/* RESERVED_MFR_3ED0 */
	{0x3ED4, 0xAFC4},	/* RESERVED_MFR_3ED4 */
	{0x3ED6, 0x909B},	/* RESERVED_MFR_3ED6 */
	{0x3EE0, 0x2424},	/* RESERVED_MFR_3EE0 */
	{0x3EE2, 0x9797},	/* RESERVED_MFR_3EE2 */
	{0x3EE4, 0xC100},	/* RESERVED_MFR_3EE4 */
	{0x3EE6, 0x0540},	/* RESERVED_MFR_3EE6 */
	{0x3174, 0x8000},	/* RESERVED_MFR_3174 */
	{0x0300, 0x0004},	/* VT_PIX_CLK_DIV */
	{0x0302, 0x0001},	/* VT_SYS_CLK_DIV */
	{0x0304, 0x0002},	/* PRE_PLL_CLK_DIV */
	{0x0306, 0x0040},	/* PLL_MULTIPLIER */
	{0x0308, 0x000A},	/* OP_PIX_CLK_DIV */
	{0x030A, 0x0001},	/* OP_SYS_CLK_DIV */
	{ar0832_TABLE_WAIT_MS, 0x0001},
	{0x3064, 0x7400},	/* RESERVED_MFR_3064 */
	{0x0104, 0x0100},	/* GROUPED_PARAMETER_HOLD */
	{0x0344, 0x00C8},	/* X_ADDR_START */
	{0x0348, 0x0C07},	/* X_ADDR_END */
	{0x0346, 0x01A6},	/* Y_ADDR_START */
	{0x034A, 0x07F9},	/* Y_ADDR_END */
	{0x034C, 0x0B40},	/* X_OUTPUT_SIZE */
	{0x034E, 0x0654},	/* Y_OUTPUT_SIZE */
	{0x3040, 0xC041},	/* READ_MODE */
	{0x306E, 0xFC80},	/* DATAPATH_SELECT */
	{0x0400, 0x0000},	/* SCALING_MODE */
	{0x0404, 0x0010},	/* SCALE_M */
	{0x3178, 0x0000},	/* RESERVED_MFR_3178 */
	{0x3ED0, 0x1E24},	/* RESERVED_MFR_3ED0 */

	{0x0342, 0x11B8},	/* LINE_LENGTH_PCK */
	{0x0340, 0x06E3},	/* FRAME_LENGTH_LINES */
	{0x0202, 0x06E3},	/* COARSE_INTEGRATION_TIME */
	{0x3014, 0x0BD8},	/* FINE_INTEGRATION_TIME */
	{0x3010, 0x0078},	/* FINE_CORRECTION */
	{0x301A, 0x8250},	/* RESET_REGISTER */
	{0x301A, 0x8650},	/* RESET_REGISTER */
	{0x301A, 0x8658},	/* RESET_REGISTER */
	/* gain */
	{0x3056, 0x10AA},	/* gain */
	{0x3058, 0x10AA},	/* gain */
	{0x305a, 0x10AA},	/* gain */
	{0x305c, 0x10AA},	/* gain */
	{0x0104, 0x0000},	/* GROUPED_PARAMETER_HOLD */
	{0x301A, 0x065C},	/* RESET_REGISTER */
	{ar0832_TABLE_END, 0x0000}
};

static struct ar0832_reg mode_1920X1080[] = {
	{0x301A, 0x0058},	/* RESET_REGISTER */
	{0x301A, 0x0050},	/* RESET_REGISTER */
	{0x0104, 0x0100},	/* GROUPED_PARAMETER_HOLD */
	{0x3064, 0x7800},	/* RESERVED_MFR_3064 */
	{0x31AE, 0x0202},	/* SERIAL_FORMAT */
	{0x31B0, 0x0083},	/* FRAME_PREAMBLE */
	{0x31B2, 0x004D},	/* LINE_PREAMBLE */
	{0x31B4, 0x0E77},	/* MIPI_TIMING_0 */
	{0x31B6, 0x0D20},	/* MIPI_TIMING_1 */
	{0x31B8, 0x020E},	/* MIPI_TIMING_2 */
	{0x31BA, 0x0710},	/* MIPI_TIMING_3 */
	{0x31BC, 0x2A0D},	/* MIPI_TIMING_4 */
	{ar0832_TABLE_WAIT_MS, 0x0005},
	{0x0112, 0x0A0A},	/* CCP_DATA_FORMAT */
	{0x3044, 0x0590},	/* RESERVED_MFR_3044 */
	{0x306E, 0xFC80},	/* DATAPATH_SELECT */
	{0x30B2, 0xC000},	/* RESERVED_MFR_30B2 */
	{0x30D6, 0x0800},	/* RESERVED_MFR_30D6 */
	{0x316C, 0xB42F},	/* RESERVED_MFR_316C */
	{0x316E, 0x869A},	/* RESERVED_MFR_316E */
	{0x3170, 0x210E},	/* RESERVED_MFR_3170 */
	{0x317A, 0x010E},	/* RESERVED_MFR_317A */
	{0x31E0, 0x1FB9},	/* RESERVED_MFR_31E0 */
	{0x31E6, 0x07FC},	/* RESERVED_MFR_31E6 */
	{0x37C0, 0x0000},	/* P_GR_Q5 */
	{0x37C2, 0x0000},	/* P_RD_Q5 */
	{0x37C4, 0x0000},	/* P_BL_Q5 */
	{0x37C6, 0x0000},	/* P_GB_Q5 */
	{0x3E00, 0x0011},	/* RESERVED_MFR_3E00 */
	{0x3E02, 0x8801},	/* RESERVED_MFR_3E02 */
	{0x3E04, 0x2801},	/* RESERVED_MFR_3E04 */
	{0x3E06, 0x8449},	/* RESERVED_MFR_3E06 */
	{0x3E08, 0x6841},	/* RESERVED_MFR_3E08 */
	{0x3E0A, 0x400C},	/* RESERVED_MFR_3E0A */
	{0x3E0C, 0x1001},	/* RESERVED_MFR_3E0C */
	{0x3E0E, 0x2603},	/* RESERVED_MFR_3E0E */
	{0x3E10, 0x4B41},	/* RESERVED_MFR_3E10 */
	{0x3E12, 0x4B24},	/* RESERVED_MFR_3E12 */
	{0x3E14, 0xA3CF},	/* RESERVED_MFR_3E14 */
	{0x3E16, 0x8802},	/* RESERVED_MFR_3E16 */
	{0x3E18, 0x84FF},	/* RESERVED_MFR_3E18 */
	{0x3E1A, 0x8601},	/* RESERVED_MFR_3E1A */
	{0x3E1C, 0x8401},	/* RESERVED_MFR_3E1C */
	{0x3E1E, 0x840A},	/* RESERVED_MFR_3E1E */
	{0x3E20, 0xFF00},	/* RESERVED_MFR_3E20 */
	{0x3E22, 0x8401},	/* RESERVED_MFR_3E22 */
	{0x3E24, 0x00FF},	/* RESERVED_MFR_3E24 */
	{0x3E26, 0x0088},	/* RESERVED_MFR_3E26 */
	{0x3E28, 0x2E8A},	/* RESERVED_MFR_3E28 */
	{0x3E30, 0x0000},	/* RESERVED_MFR_3E30 */
	{0x3E32, 0x8801},	/* RESERVED_MFR_3E32 */
	{0x3E34, 0x4029},	/* RESERVED_MFR_3E34 */
	{0x3E36, 0x00FF},	/* RESERVED_MFR_3E36 */
	{0x3E38, 0x8469},	/* RESERVED_MFR_3E38 */
	{0x3E3A, 0x00FF},	/* RESERVED_MFR_3E3A */
	{0x3E3C, 0x2801},	/* RESERVED_MFR_3E3C */
	{0x3E3E, 0x3E2A},	/* RESERVED_MFR_3E3E */
	{0x3E40, 0x1C01},	/* RESERVED_MFR_3E40 */
	{0x3E42, 0xFF84},	/* RESERVED_MFR_3E42 */
	{0x3E44, 0x8401},	/* RESERVED_MFR_3E44 */
	{0x3E46, 0x0C01},	/* RESERVED_MFR_3E46 */
	{0x3E48, 0x8401},	/* RESERVED_MFR_3E48 */
	{0x3E4A, 0x00FF},	/* RESERVED_MFR_3E4A */
	{0x3E4C, 0x8402},	/* RESERVED_MFR_3E4C */
	{0x3E4E, 0x8984},	/* RESERVED_MFR_3E4E */
	{0x3E50, 0x6628},	/* RESERVED_MFR_3E50 */
	{0x3E52, 0x8340},	/* RESERVED_MFR_3E52 */
	{0x3E54, 0x00FF},	/* RESERVED_MFR_3E54 */
	{0x3E56, 0x4A42},	/* RESERVED_MFR_3E56 */
	{0x3E58, 0x2703},	/* RESERVED_MFR_3E58 */
	{0x3E5A, 0x6752},	/* RESERVED_MFR_3E5A */
	{0x3E5C, 0x3F2A},	/* RESERVED_MFR_3E5C */
	{0x3E5E, 0x846A},	/* RESERVED_MFR_3E5E */
	{0x3E60, 0x4C01},	/* RESERVED_MFR_3E60 */
	{0x3E62, 0x8401},	/* RESERVED_MFR_3E62 */
	{0x3E66, 0x3901},	/* RESERVED_MFR_3E66 */
	{0x3E90, 0x2C01},	/* RESERVED_MFR_3E90 */
	{0x3E98, 0x2B02},	/* RESERVED_MFR_3E98 */
	{0x3E92, 0x2A04},	/* RESERVED_MFR_3E92 */
	{0x3E94, 0x2509},	/* RESERVED_MFR_3E94 */
	{0x3E96, 0x0000},	/* RESERVED_MFR_3E96 */
	{0x3E9A, 0x2905},	/* RESERVED_MFR_3E9A */
	{0x3E9C, 0x00FF},	/* RESERVED_MFR_3E9C */
	{0x3ECC, 0x00EB},	/* RESERVED_MFR_3ECC */
	{0x3ED0, 0x1E24},	/* RESERVED_MFR_3ED0 */
	{0x3ED4, 0xAFC4},	/* RESERVED_MFR_3ED4 */
	{0x3ED6, 0x909B},	/* RESERVED_MFR_3ED6 */
	{0x3EE0, 0x2424},	/* RESERVED_MFR_3EE0 */
	{0x3EE2, 0x9797},	/* RESERVED_MFR_3EE2 */
	{0x3EE4, 0xC100},	/* RESERVED_MFR_3EE4 */
	{0x3EE6, 0x0540},	/* RESERVED_MFR_3EE6 */
	{0x3174, 0x8000},	/* RESERVED_MFR_3174 */
	{0x0300, 0x0004},	/* VT_PIX_CLK_DIV */
	{0x0302, 0x0001},	/* VT_SYS_CLK_DIV */
	{0x0304, 0x0002},	/* PRE_PLL_CLK_DIV */
	{0x0306, 0x0040},	/* PLL_MULTIPLIER */
	{0x0308, 0x000A},	/* OP_PIX_CLK_DIV */
	{0x030A, 0x0001},	/* OP_SYS_CLK_DIV */
	{ar0832_TABLE_WAIT_MS, 0x0001},
	{0x3064, 0x7400},	/* RESERVED_MFR_3064 */
	{0x0104, 0x0100},	/* GROUPED_PARAMETER_HOLD */
	{0x0344, 0x028C},	/* X_ADDR_START */
	{0x0348, 0x0A0B},	/* X_ADDR_END */
	{0x0346, 0x006E},	/* Y_ADDR_START */
	{0x034A, 0x04A5},	/* Y_ADDR_END */
	{0x034C, 0x0780},	/* X_OUTPUT_SIZE */
	{0x034E, 0x0438},	/* Y_OUTPUT_SIZE */
	{0x3040, 0xC041},	/* READ_MODE */
	{0x306E, 0xFC80},	/* DATAPATH_SELECT */
	{0x0400, 0x0000},	/* SCALING_MODE */
	{0x0404, 0x0010},	/* SCALE_M */
	{0x3178, 0x0000},	/* RESERVED_MFR_3178 */
	{0x3ED0, 0x1E24},	/* RESERVED_MFR_3ED0 */

	{0x0342, 0x103B},	/* LINE_LENGTH_PCK */
	{0x0340, 0x05C4},	/* FRAME_LENGTH_LINES */
	{0x0202, 0x05C4},	/* COARSE_INTEGRATION_TIME */
	{0x3014, 0x0702},	/* FINE_INTEGRATION_TIME */
	{0x3010, 0x0078},	/* FINE_CORRECTION */
	{0x301A, 0x8250},	/* RESET_REGISTER */
	{0x301A, 0x8650},	/* RESET_REGISTER */
	{0x301A, 0x8658},	/* RESET_REGISTER */
	/* gain */
	{0x3056, 0x10AA},	/* gain */
	{0x3058, 0x10AA},	/* gain */
	{0x305a, 0x10AA},	/* gain */
	{0x305c, 0x10AA},	/* gain */
	{0x0104, 0x0000},	/* GROUPED_PARAMETER_HOLD */
	{0x301A, 0x065C},	/* RESET_REGISTER */
	{ar0832_TABLE_END, 0x0000}
};

static struct ar0832_reg mode_1632X1224[] = {
	{0x301A, 0x0058},	/* RESET_REGISTER */
	{0x301A, 0x0050},	/* RESET_REGISTER */

	/* SC-CHANGE: to-do 8 bit write */
	{0x0104, 0x0100},	/* GROUPED_PARAMETER_HOLD */

	{0x3064, 0x7800},	/* RESERVED_MFR_3064 */
	{0x31AE, 0x0202},	/* SERIAL_FORMAT */
	{0x31B0, 0x0083},	/* FRAME_PREAMBLE */
	{0x31B2, 0x004D},	/* LINE_PREAMBLE */
	{0x31B4, 0x0E77},	/* MIPI_TIMING_0 */
	{0x31B6, 0x0D20},	/* MIPI_TIMING_1 */
	{0x31B8, 0x020E},	/* MIPI_TIMING_2 */
	{0x31BA, 0x0710},	/* MIPI_TIMING_3 */
	{0x31BC, 0x2A0D},	/* MIPI_TIMING_4 */
	{ar0832_TABLE_WAIT_MS, 0x0005},
	{0x0112, 0x0A0A},	/* CCP_DATA_FORMAT */
	{0x3044, 0x0590},	/* RESERVED_MFR_3044 */
	{0x306E, 0xFC80},	/* DATAPATH_SELECT */
	{0x30B2, 0xC000},	/* RESERVED_MFR_30B2 */
	{0x30D6, 0x0800},	/* RESERVED_MFR_30D6 */
	{0x316C, 0xB42F},	/* RESERVED_MFR_316C */
	{0x316E, 0x869A},	/* RESERVED_MFR_316E */
	{0x3170, 0x210E},	/* RESERVED_MFR_3170 */
	{0x317A, 0x010E},	/* RESERVED_MFR_317A */
	{0x31E0, 0x1FB9},	/* RESERVED_MFR_31E0 */
	{0x31E6, 0x07FC},	/* RESERVED_MFR_31E6 */
	{0x37C0, 0x0000},	/* P_GR_Q5 */
	{0x37C2, 0x0000},	/* P_RD_Q5 */
	{0x37C4, 0x0000},	/* P_BL_Q5 */
	{0x37C6, 0x0000},	/* P_GB_Q5 */
	{0x3E00, 0x0011},	/* RESERVED_MFR_3E00 */
	{0x3E02, 0x8801},	/* RESERVED_MFR_3E02 */
	{0x3E04, 0x2801},	/* RESERVED_MFR_3E04 */
	{0x3E06, 0x8449},	/* RESERVED_MFR_3E06 */
	{0x3E08, 0x6841},	/* RESERVED_MFR_3E08 */
	{0x3E0A, 0x400C},	/* RESERVED_MFR_3E0A */
	{0x3E0C, 0x1001},	/* RESERVED_MFR_3E0C */
	{0x3E0E, 0x2603},	/* RESERVED_MFR_3E0E */
	{0x3E10, 0x4B41},	/* RESERVED_MFR_3E10 */
	{0x3E12, 0x4B24},	/* RESERVED_MFR_3E12 */
	{0x3E14, 0xA3CF},	/* RESERVED_MFR_3E14 */
	{0x3E16, 0x8802},	/* RESERVED_MFR_3E16 */
	{0x3E18, 0x84FF},	/* RESERVED_MFR_3E18 */
	{0x3E1A, 0x8601},	/* RESERVED_MFR_3E1A */
	{0x3E1C, 0x8401},	/* RESERVED_MFR_3E1C */
	{0x3E1E, 0x840A},	/* RESERVED_MFR_3E1E */
	{0x3E20, 0xFF00},	/* RESERVED_MFR_3E20 */
	{0x3E22, 0x8401},	/* RESERVED_MFR_3E22 */
	{0x3E24, 0x00FF},	/* RESERVED_MFR_3E24 */
	{0x3E26, 0x0088},	/* RESERVED_MFR_3E26 */
	{0x3E28, 0x2E8A},	/* RESERVED_MFR_3E28 */
	{0x3E30, 0x0000},	/* RESERVED_MFR_3E30 */
	{0x3E32, 0x8801},	/* RESERVED_MFR_3E32 */
	{0x3E34, 0x4029},	/* RESERVED_MFR_3E34 */
	{0x3E36, 0x00FF},	/* RESERVED_MFR_3E36 */
	{0x3E38, 0x8469},	/* RESERVED_MFR_3E38 */
	{0x3E3A, 0x00FF},	/* RESERVED_MFR_3E3A */
	{0x3E3C, 0x2801},	/* RESERVED_MFR_3E3C */
	{0x3E3E, 0x3E2A},	/* RESERVED_MFR_3E3E */
	{0x3E40, 0x1C01},	/* RESERVED_MFR_3E40 */
	{0x3E42, 0xFF84},	/* RESERVED_MFR_3E42 */
	{0x3E44, 0x8401},	/* RESERVED_MFR_3E44 */
	{0x3E46, 0x0C01},	/* RESERVED_MFR_3E46 */
	{0x3E48, 0x8401},	/* RESERVED_MFR_3E48 */
	{0x3E4A, 0x00FF},	/* RESERVED_MFR_3E4A */
	{0x3E4C, 0x8402},	/* RESERVED_MFR_3E4C */
	{0x3E4E, 0x8984},	/* RESERVED_MFR_3E4E */
	{0x3E50, 0x6628},	/* RESERVED_MFR_3E50 */
	{0x3E52, 0x8340},	/* RESERVED_MFR_3E52 */
	{0x3E54, 0x00FF},	/* RESERVED_MFR_3E54 */
	{0x3E56, 0x4A42},	/* RESERVED_MFR_3E56 */
	{0x3E58, 0x2703},	/* RESERVED_MFR_3E58 */
	{0x3E5A, 0x6752},	/* RESERVED_MFR_3E5A */
	{0x3E5C, 0x3F2A},	/* RESERVED_MFR_3E5C */
	{0x3E5E, 0x846A},	/* RESERVED_MFR_3E5E */
	{0x3E60, 0x4C01},	/* RESERVED_MFR_3E60 */
	{0x3E62, 0x8401},	/* RESERVED_MFR_3E62 */
	{0x3E66, 0x3901},	/* RESERVED_MFR_3E66 */
	{0x3E90, 0x2C01},	/* RESERVED_MFR_3E90 */
	{0x3E98, 0x2B02},	/* RESERVED_MFR_3E98 */
	{0x3E92, 0x2A04},	/* RESERVED_MFR_3E92 */
	{0x3E94, 0x2509},	/* RESERVED_MFR_3E94 */
	{0x3E96, 0x0000},	/* RESERVED_MFR_3E96 */
	{0x3E9A, 0x2905},	/* RESERVED_MFR_3E9A */
	{0x3E9C, 0x00FF},	/* RESERVED_MFR_3E9C */
	{0x3ECC, 0x00EB},	/* RESERVED_MFR_3ECC */
	{0x3ED0, 0x1E24},	/* RESERVED_MFR_3ED0 */
	{0x3ED4, 0xAFC4},	/* RESERVED_MFR_3ED4 */
	{0x3ED6, 0x909B},	/* RESERVED_MFR_3ED6 */
	{0x3EE0, 0x2424},	/* RESERVED_MFR_3EE0 */
	{0x3EE2, 0x9797},	/* RESERVED_MFR_3EE2 */
	{0x3EE4, 0xC100},	/* RESERVED_MFR_3EE4 */
	{0x3EE6, 0x0540},	/* RESERVED_MFR_3EE6 */
	{0x3174, 0x8000},	/* RESERVED_MFR_3174 */
	{0x0300, 0x0004},	/* VT_PIX_CLK_DIV */
	{0x0302, 0x0001},	/* VT_SYS_CLK_DIV */
	{0x0304, 0x0002},	/* PRE_PLL_CLK_DIV */

	{0x0306, 0x0040},	/* PLL_MULTIPLIER */

	{0x0308, 0x000A},	/* OP_PIX_CLK_DIV */
	{0x030A, 0x0001},	/* OP_SYS_CLK_DIV */
	{ar0832_TABLE_WAIT_MS, 0x0001}, /* waitmsec 1 */

	{0x3064, 0x7400},	/* RESERVED_MFR_3064 */

	{0x0104, 0x0100},	/* GROUPED_PARAMETER_HOLD */

	{0x0344, 0x0008},	/* X_ADDR_START */
	{0x0348, 0x0CC9},	/* X_ADDR_END */
	{0x0346, 0x0008},	/* Y_ADDR_START */
	{0x034A, 0x0999},	/* Y_ADDR_END */
	{0x034C, 0x0660},	/* X_OUTPUT_SIZE */
	{0x034E, 0x04C8},	/* Y_OUTPUT_SIZE */
	{0x3040, 0xC4C3},	/* READ_MODE */
	{0x306E, 0xFC80},	/* DATAPATH_SELECT */
	{0x3178, 0x0000},	/* RESERVED_MFR_3178 */
	{0x3ED0, 0x1E24},	/* RESERVED_MFR_3ED0 */
	{0x0400, 0x0002},	/* SCALING_MODE */
	{0x0404, 0x0010},	/* SCALE_M */
	{0x0342, 0x101A},	/* LINE_LENGTH_PCK */
	{0x0340, 0x0610},	/* FRAME_LENGTH_LINES */
	{0x0202, 0x0557},	/* COARSE_INTEGRATION_TIME */
	{0x3014, 0x0988},	/* FINE_INTEGRATION_TIME */
	{0x3010, 0x0130},	/* FINE_CORRECTION */
	{0x301A, 0x8250},	/* RESET_REGISTER */
	{0x301A, 0x8650},	/* RESET_REGISTER */
	{0x301A, 0x8658},	/* RESET_REGISTER */

	/* gain */
	{0x3056, 0x10AA},	/* gain */
	{0x3058, 0x10AA},	/* gain */
	{0x305a, 0x10AA},	/* gain */
	{0x305c, 0x10AA},	/* gain */

	/* todo 8-bit write */
	{0x0104, 0x0000},	/* GROUPED_PARAMETER_HOLD */

	{0x301A, 0x065C},	/* RESET_REGISTER */
	{ar0832_TABLE_END, 0x0000}
};

static struct ar0832_reg mode_end[] = {
	{ar0832_TABLE_END, 0x0000}
};

enum {
	ar0832_MODE_3264X2448,
	ar0832_MODE_2880X1620,
	ar0832_MODE_1920X1080,
	ar0832_MODE_1632X1224,
};

static struct ar0832_reg *mode_table[] = {
	[ar0832_MODE_3264X2448] = mode_3264X2448,
	[ar0832_MODE_2880X1620] = mode_2880X1620,
	[ar0832_MODE_1920X1080] = mode_1920X1080,
	[ar0832_MODE_1632X1224] = mode_1632X1224,
};

static inline void ar0832_msleep(u32 t)
{
	/*
	why usleep_range() instead of msleep() ?
	Read Documentation/timers/timers-howto.txt
	*/
	usleep_range(t*1000, t*1000 + 500);
}

/* 16 bit reg to program frame length */
static inline void ar0832_get_frame_length_regs(struct ar0832_reg *regs,
						u32 frame_length)
{
	regs->addr = 0x0340;
	regs->val = (frame_length) & 0xFFFF;
}

static inline void ar0832_get_coarse_time_regs(struct ar0832_reg *regs,
						u32 coarse_time)
{
	regs->addr = 0x0202;
	regs->val = (coarse_time) & 0xFFFF;
}

static inline void ar0832_get_focuser_vcm_control_regs(struct ar0832_reg *regs,
							u16 value)
{
	regs->addr = 0x30F0;
	regs->val = (value) & 0xFFFF;
}

static inline void ar0832_get_focuser_vcm_step_time_regs
	(struct ar0832_reg *regs, u16 value)
{
	regs->addr = 0x30F4;
	regs->val = (value) & 0xFFFF;
}

static inline void ar0832_get_focuser_data_regs(struct ar0832_reg *regs,
						u16 value)
{
	regs->addr = 0x30F2;
	regs->val = (value) & 0xFFFF;
}

static inline void ar0832_get_gain_reg(struct ar0832_reg *regs, u16 gain)
{
	regs->addr = 0x3056;
	regs->val = gain;
	(regs + 1)->addr = 0x3058;
	(regs + 1)->val = gain;
	(regs + 2)->addr = 0x305A;
	(regs + 2)->val = gain;
	(regs + 3)->addr = 0x305C;
	(regs + 3)->val = gain;
}

static int ar0832_write_reg8(struct i2c_client *client, u16 addr, u8 val)
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

	dev_dbg(&client->dev, "0x%x = 0x%x\n", addr, val);

	do {
		err = i2c_transfer(client->adapter, &msg, 1);
		if (err > 0)
			return 0;
		retry++;
		dev_err(&client->dev,
			"%s: i2c transfer failed, retrying %x %x\n",
			__func__, addr, val);
		ar0832_msleep(3);
	} while (retry < ar0832_MAX_RETRIES);

	return err;
}

static int ar0832_write_reg16(struct i2c_client *client, u16 addr, u16 val)
{
	int count;
	struct i2c_msg msg;
	unsigned char data[4];
	int retry = 0;

	if (!client->adapter)
		return -ENODEV;

	data[0] = (u8) (addr >> 8);
	data[1] = (u8) (addr & 0xff);
	data[2] = (u8) (val >> 8);
	data[3] = (u8) (val & 0xff);

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = 4;
	msg.buf = data;

	dev_dbg(&client->dev, "0x%x = 0x%x\n", addr, val);

	do {
		count = i2c_transfer(client->adapter, &msg, 1);
		if (count == 1)
			return 0;
		retry++;
		dev_err(&client->dev,
			"%s: i2c transfer failed, retrying %x %x\n",
		       __func__, addr, val);
		ar0832_msleep(3);
	} while (retry <= ar0832_MAX_RETRIES);

	return -EIO;
}

static int ar0832_read_reg16(struct i2c_client *client, u16 addr, u16 *val)
{
	struct i2c_msg msg[2];
	u8 data[4];

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf = data;
	data[0] = (addr >> 8);
	data[1] = (addr & 0xff);
	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 2;
	msg[1].buf = data + 2;

	if (i2c_transfer(client->adapter, msg, 2) == 2) {
		*val = ((data[2] << 8) | data[3]);
		dev_dbg(&client->dev, "0x%x = 0x%x\n", addr, *val);
		return 0;
	} else {
		*val = 0;
		dev_err(&client->dev,
			"%s: i2c read failed.\n", __func__);
		return -1;
	}
}

static int ar0832_write_reg_helper(struct ar0832_dev *dev,
					u16 addr,
					u16 val)
{
	int ret;

	if (addr == 0x104)
		ret = ar0832_write_reg8(dev->i2c_client, addr,
					(val >> 8 & 0xff));
	else
		ret = ar0832_write_reg16(dev->i2c_client, addr, val);

	return ret;
}

static int ar0832_write_table(struct ar0832_dev *dev,
				const struct ar0832_reg table[],
				const struct ar0832_reg override_list[],
				int num_override_regs)
{
	int err;
	const struct ar0832_reg *next;

	for (next = table; next->addr != ar0832_TABLE_END; next++) {
		if (next->addr ==  ar0832_TABLE_WAIT_MS) {
			ar0832_msleep(next->val);
			continue;
		}
		err = ar0832_write_reg_helper(dev, next->addr, next->val);
		if (err)
			return err;
	}
	return 0;
}

static int ar0832_set_frame_length(struct ar0832_dev *dev,
					u32 frame_length)
{
	struct ar0832_reg reg_list;
	struct i2c_client *i2c_client = dev->i2c_client;
	int ret;

	dev_dbg(&i2c_client->dev, "[%s] (0x%08x)\n", __func__,  frame_length);

	ar0832_get_frame_length_regs(&reg_list, frame_length);
	ret = ar0832_write_reg8(i2c_client, 0x0104, 0x1);
	if (ret)
		return ret;

	ret = ar0832_write_reg16(i2c_client, reg_list.addr,
			reg_list.val);
	if (ret)
		return ret;

	ret = ar0832_write_reg8(i2c_client, 0x0104, 0x0);
	if (ret)
		return ret;

	return 0;
}

static int ar0832_set_coarse_time(struct ar0832_dev *dev,
				  u32 coarse_time)
{
	int ret;
	struct ar0832_reg reg_list;
	struct i2c_client *i2c_client = dev->i2c_client;

	dev_dbg(&i2c_client->dev, "[%s] (0x%08x)\n", __func__,  coarse_time);
	ar0832_get_coarse_time_regs(&reg_list, coarse_time);

	ret = ar0832_write_reg8(i2c_client, 0x0104, 0x1);
	if (ret)
		return ret;

	ret = ar0832_write_reg16(i2c_client, reg_list.addr,
			reg_list.val);
	if (ret)
		return ret;

	ret = ar0832_write_reg8(i2c_client, 0x0104, 0x0);
	if (ret)
		return ret;

	return 0;
}

static int ar0832_set_gain(struct ar0832_dev *dev, __u16 gain)
{
	int i;
	int ret = 0;
	struct ar0832_reg reg_list_gain[4];

	ret |= ar0832_write_reg8(dev->i2c_client, 0x0104, 0x1);
	/* Gain Registers Start */
	ar0832_get_gain_reg(reg_list_gain, gain);
	for (i = 0; i < 4; i++)	{
		ret = ar0832_write_reg16(dev->i2c_client,
					reg_list_gain[i].addr,
					reg_list_gain[i].val);
		if (ret)
			return ret;
	}
	/* Gain register End */
	ret |= ar0832_write_reg8(dev->i2c_client, 0x0104, 0x0);

	return ret;
}

static int ar0832_set_mode(struct ar0832_dev *dev,
				struct ar0832_mode *mode)
{
	int sensor_mode;
	int err;
	int ret;
	struct i2c_client *i2c_client = dev->i2c_client;
	struct ar0832_reg reg_frame_length, reg_coarse_time;

	dev_info(&i2c_client->dev, "%s: ++\n", __func__);

	if (mode->xres == 3264 && mode->yres == 2448)
		sensor_mode = ar0832_MODE_3264X2448;
	else if (mode->xres == 2880 && mode->yres == 1620)
		sensor_mode = ar0832_MODE_2880X1620;
	else if (mode->xres == 1920 && mode->yres == 1080)
		sensor_mode = ar0832_MODE_1920X1080;
	else if (mode->xres == 1632 && mode->yres == 1224)
		sensor_mode = ar0832_MODE_1632X1224;
	else {
		dev_err(&i2c_client->dev,
			"%s: invalid resolution supplied to set mode %d %d\n",
			__func__ , mode->xres, mode->yres);
		return -EINVAL;
	}

	/* get a list of override regs for the asking frame length,    */
	/* coarse integration time, and gain.*/
	err = ar0832_write_table(dev, mode_start, NULL, 0);
	if (err)
		return err;

	err = ar0832_write_table(dev, mode_table[sensor_mode], NULL, 0);
	if (err)
		return err;

	/* When we change the resolution */
	ar0832_get_frame_length_regs(&reg_frame_length, mode->frame_length);
	ret = ar0832_write_reg16(i2c_client, reg_frame_length.addr,
		reg_frame_length.val);
	if (ret)
		return ret;

	ar0832_get_coarse_time_regs(&reg_coarse_time, mode->coarse_time);
	ret = ar0832_write_reg16(i2c_client, reg_coarse_time.addr,
			reg_coarse_time.val);
	if (ret)
		return ret;

	ret = ar0832_set_gain(dev, mode->gain);
	if (ret)
		return ret;

	err = ar0832_write_table(dev, mode_end, NULL, 0);
	if (err)
		return err;

	dev->sensor_info->mode = sensor_mode;
	dev_dbg(&i2c_client->dev, "%s: --\n", __func__);

	return 0;
}

static int ar0832_get_status(struct ar0832_dev *dev, u8 *status)
{
	int err = 0;
	struct i2c_client *i2c_client = dev->i2c_client;

	*status = 0;
	/* FixMe */
	/*
	err = ar0832_read_reg(dev->i2c_client, 0x001, status);
	*/
	dev_dbg(&i2c_client->dev, "%s: %u %d\n", __func__, *status, err);
	return err;
}

static int ar0832_set_region(struct ar0832_dev *dev,
				struct ar0832_stereo_region *region)
{
	u16 image_width = region->image_end.x - region->image_start.x+1;
	u16 image_height = region->image_end.y - region->image_start.y+1;
	struct i2c_client *i2c_client = dev->i2c_client;

	dev_dbg(&i2c_client->dev, "%s: %d\n", __func__, region->camera_index);
	if (region->camera_index == 0)
		i2c_client = dev->i2c_client;
#if 0
	else if (region->camera_index == 1)
		i2c_client = dev->i2c_client_right;
#endif
	else
		return -1;
	dev_dbg(&i2c_client->dev, "%s: width = %d  height = %d\n",
		 __func__, image_width, image_height);
#if 0
	ar0832_write_reg(i2c_client, 0x0104, 1);
	ar0832_write_reg(i2c_client, 0x0346,
			 UpperByte16to8(region->image_start.y));
	/* Y_ADDR START  LOWER BYTE */
	ar0832_write_reg(i2c_client, 0x0347,
			 LowerByte16to8(region->image_start.y));
	/* Y_OUT SIZE UPPER BYTE */
	ar0832_write_reg(i2c_client, 0x034E,
			 UpperByte16to8(DefaultImageHeight));
	/* Y_OUT SIZE LOWER BYTE */
	ar0832_write_reg(i2c_client, 0x034F,
			 LowerByte16to8(DefaultImageHeight));
	/* Y_ADDR_END UPPER BYTE */
	ar0832_write_reg(i2c_client, 0x034A,
			 UpperByte16to8(region->image_end.y));
	/* Y_ADDR_END LOWER BYTE */
	ar0832_write_reg(i2c_client, 0x034B,
			 LowerByte16to8(region->image_end.y));
	/* X_ADDR_START UPPER BYTE */
	ar0832_write_reg(i2c_client, 0x0344,
			 UpperByte16to8(region->image_start.x));
	/* X_ADDR START LOWER BYTE */
	ar0832_write_reg(i2c_client, 0x0345,
			 LowerByte16to8(region->image_start.x));
	/* X_SIZE UPPER BYTE */
	ar0832_write_reg(i2c_client, 0x034C,
			 UpperByte16to8(DefaultImageWidth));
	/* X_SIZE LOWER BYTE */
	ar0832_write_reg(i2c_client, 0x034D,
			 LowerByte16to8(DefaultImageWidth));
	/* X_ADDR_END UPPER BYTE */
	ar0832_write_reg(i2c_client, 0x0348,
			 UpperByte16to8(region->image_end.x));
	/* X_ADDR_END LOWER BYTE */
	ar0832_write_reg(i2c_client, 0x0349,
			 LowerByte16to8(region->image_end.x));
	ar0832_write_reg(i2c_client, 0x0104, 0);
#endif
	return 0;
}

static int ar0832_set_alternate_addr(struct i2c_client *client)
{
	int ret = 0;
	u8 new_addr = client->addr;
	u16 val;

	/* Default slave address of ar0832 is 0x36 */
	client->addr = 0x36;
	ret = ar0832_read_reg16(client, AR0832_RESET_REG, &val);
	val &= ~AR0832_RESET_REG_LOCK_REG;
	ret |= ar0832_write_reg16(client, AR0832_RESET_REG, val);
	ret |= ar0832_write_reg16(client, AR0832_ID_REG, new_addr << 1);

	if (!ret) {
		client->addr = new_addr;
		dev_info(&client->dev,
			"new slave address is set to 0x%x\n", new_addr);
	}

	ret |= ar0832_read_reg16(client, AR0832_RESET_REG, &val);
	val |= AR0832_RESET_REG_LOCK_REG;
	ret |= ar0832_write_reg16(client, AR0832_RESET_REG, val);

	return ret;
}

static int ar0832_power_on(struct ar0832_dev *dev)
{
	struct i2c_client *i2c_client = dev->i2c_client;
	int ret = 0;

	dev_info(&i2c_client->dev, "%s: ++ %d\n", __func__, dev->is_stereo);

	/* Plug 1.8V and 2.8V power to sensor */
	if (dev->power_rail.sen_1v8_reg) {
		ret = regulator_enable(dev->power_rail.sen_1v8_reg);
		if (ret) {
			dev_err(&i2c_client->dev, "%s: failed to enable vdd\n",
				__func__);
			goto fail_regulator_1v8_reg;
		}
	}

	ar0832_msleep(20);

	if (dev->power_rail.sen_2v8_reg) {
		ret = regulator_enable(dev->power_rail.sen_2v8_reg);
		if (ret) {
			dev_err(&i2c_client->dev, "%s: failed to enable vaa\n",
				__func__);
			goto fail_regulator_2v8_reg;
		}
	}

	/* Board specific power-on sequence */
	dev->pdata->power_on(dev->is_stereo);

	/* Change slave address */
	if (i2c_client->addr)
		ret = ar0832_set_alternate_addr(i2c_client);

	return 0;

fail_regulator_2v8_reg:
	regulator_put(dev->power_rail.sen_2v8_reg);
	dev->power_rail.sen_2v8_reg = NULL;
	regulator_disable(dev->power_rail.sen_1v8_reg);
fail_regulator_1v8_reg:
	regulator_put(dev->power_rail.sen_1v8_reg);
	dev->power_rail.sen_1v8_reg = NULL;
	return ret;
}

static int ar0832_focuser_set_config(struct ar0832_dev *dev)
{
	struct ar0832_reg reg_vcm_ctrl, reg_vcm_step_time;
	int ret = 0;
	u8 vcm_slew = 1;

	/* bit15(0x80) means that VCM driver enable bit. */
	/* bit3(0x08) means that keep VCM(AF position) */
	/* while sensor is in soft standby mode during mode transitions. */
	u16 vcm_control_data = (0x80 << 8 | (0x08 | (vcm_slew & 0x07)));
	u16 vcm_step_time = 2048;

	ar0832_get_focuser_vcm_control_regs(&reg_vcm_ctrl, vcm_control_data);
	ret = ar0832_write_reg16(dev->i2c_client, reg_vcm_ctrl.addr,
					reg_vcm_ctrl.val);
	if (ret)
		return ret;

	ar0832_get_focuser_vcm_step_time_regs(&reg_vcm_step_time,
						vcm_step_time);
	ret = ar0832_write_reg16(dev->i2c_client, reg_vcm_step_time.addr,
					reg_vcm_step_time.val);
	return ret;
}

static int ar0832_focuser_set_position(struct ar0832_dev *dev,
					u32 position)
{
	int ret = 0;
	struct ar0832_reg reg_data;

	if (position < dev->focuser_info->config.pos_low ||
		position > dev->focuser_info->config.pos_high)
		return -EINVAL;

	ar0832_get_focuser_data_regs(&reg_data, position);
	ret = ar0832_write_reg16(dev->i2c_client, reg_data.addr,
				     reg_data.val);
	return ret;
}

static long ar0832_ioctl(struct file *file,
				unsigned int cmd, unsigned long arg)
{
	int err;
	struct ar0832_dev *dev = file->private_data;
	struct i2c_client *i2c_client = dev->i2c_client;
	struct ar0832_mode mode;

	switch (cmd) {
	case AR0832_IOCTL_SET_POWER_ON:
		dev_dbg(&i2c_client->dev, "AR0832_IOCTL_SET_POWER_ON\n");
		if (copy_from_user(&mode,
			(const void __user *)arg,
			sizeof(struct ar0832_mode))) {
			dev_err(&i2c_client->dev,
				"%s: AR0832_IOCTL_SET_POWER_ON failed\n",
				__func__);
			return -EFAULT;
		}
		dev->is_stereo = mode.stereo;
		return ar0832_power_on(dev);
	case AR0832_IOCTL_SET_MODE:
	{
		dev_dbg(&i2c_client->dev, "AR0832_IOCTL_SET_MODE\n");
		if (copy_from_user(&mode,
			(const void __user *)arg,
			sizeof(struct ar0832_mode))) {
			dev_err(&i2c_client->dev,
				"%s: AR0832_IOCTL_SET_MODE failed\n",
				__func__);
			return -EFAULT;
		}
		mutex_lock(&dev->ar0832_camera_lock);
		err = ar0832_set_mode(dev, &mode);
		if (dev->focuser_info->focuser_init_flag == false) {
			ar0832_focuser_set_config(dev);
			dev->focuser_info->focuser_init_flag = true;
		}
		mutex_unlock(&dev->ar0832_camera_lock);
		return err;
	}
	case AR0832_IOCTL_SET_FRAME_LENGTH:
		mutex_lock(&dev->ar0832_camera_lock);
		err = ar0832_set_frame_length(dev, (u32)arg);
		mutex_unlock(&dev->ar0832_camera_lock);
		return err;
	case AR0832_IOCTL_SET_COARSE_TIME:
		mutex_lock(&dev->ar0832_camera_lock);
		err = ar0832_set_coarse_time(dev, (u32)arg);
		mutex_unlock(&dev->ar0832_camera_lock);
		return err;
	case AR0832_IOCTL_SET_GAIN:
		mutex_lock(&dev->ar0832_camera_lock);
		err = ar0832_set_gain(dev, (u16)arg);
		mutex_unlock(&dev->ar0832_camera_lock);
		return err;
	case AR0832_IOCTL_GET_STATUS:
	{
		u8 status;
		dev_dbg(&i2c_client->dev, "AR0832_IOCTL_GET_STATUS\n");
		err = ar0832_get_status(dev, &status);
		if (err)
			return err;
		if (copy_to_user((void __user *)arg, &status,
				 2)) {
			dev_err(&i2c_client->dev,
				"%s: AR0832_IOCTL_GET_STATUS failed\n",
				__func__);
			return -EFAULT;
		}
		return 0;
	}
	case AR0832_IOCTL_SET_SENSOR_REGION:
	{
		struct ar0832_stereo_region region;
		dev_dbg(&i2c_client->dev, "AR0832_IOCTL_SET_SENSOR_REGION\n");
		if (copy_from_user(&region,
			(const void __user *)arg,
			sizeof(struct ar0832_stereo_region))) {
			dev_err(&i2c_client->dev,
				"%s: AR0832_IOCTL_SET_SENSOR_REGION failed\n",
				__func__);
			return -EFAULT;
		}
		err = ar0832_set_region(dev, &region);
		return err;
	}

	case AR0832_FOCUSER_IOCTL_GET_CONFIG:
		dev_dbg(&i2c_client->dev,
			"%s AR0832_FOCUSER_IOCTL_GET_CONFIG\n", __func__);
		if (copy_to_user((void __user *) arg,
				 &dev->focuser_info->config,
				 sizeof(dev->focuser_info->config))) {
			dev_err(&i2c_client->dev,
				"%s: AR0832_FOCUSER_IOCTL_GET_CONFIG failed\n",
				__func__);
			return -EFAULT;
		}
		return 0;

	case AR0832_FOCUSER_IOCTL_SET_POSITION:
		dev_dbg(&i2c_client->dev,
			"%s AR0832_FOCUSER_IOCTL_SET_POSITION\n", __func__);
		mutex_lock(&dev->ar0832_camera_lock);
		err = ar0832_focuser_set_position(dev, (u32)arg);
		mutex_unlock(&dev->ar0832_camera_lock);
		return err;

	default:
		dev_err(&i2c_client->dev, "(error) %s NONE IOCTL\n",
			__func__);
		return -EINVAL;
	}
	return 0;
}

static int ar0832_open(struct inode *inode, struct file *file)
{
	struct miscdevice *miscdev = file->private_data;
	struct ar0832_dev *dev = dev_get_drvdata(miscdev->parent);
	struct i2c_client *i2c_client = dev->i2c_client;

	dev_info(&i2c_client->dev, "%s: ++\n", __func__);
	if (atomic_xchg(&dev->in_use, 1))
		return -EBUSY;

	dev->focuser_info->focuser_init_flag = false;
	file->private_data = dev;
	return 0;
}

static int ar0832_release(struct inode *inode, struct file *file)
{
	struct ar0832_dev *dev = file->private_data;
	struct i2c_client *i2c_client = dev->i2c_client;

	dev_info(&i2c_client->dev, "%s: ++\n", __func__);

	/* Unplug 1.8V and 2.8V power from sensor */
	if (dev->power_rail.sen_2v8_reg)
		regulator_disable(dev->power_rail.sen_2v8_reg);
	if (dev->power_rail.sen_1v8_reg)
		regulator_disable(dev->power_rail.sen_1v8_reg);

	/* Board specific power-down sequence */
	dev->pdata->power_off(dev->is_stereo);

	file->private_data = NULL;

	WARN_ON(!atomic_xchg(&dev->in_use, 0));
	return 0;
}

static const struct file_operations ar0832_fileops = {
	.owner = THIS_MODULE,
	.open = ar0832_open,
	.unlocked_ioctl = ar0832_ioctl,
	.release = ar0832_release,
};

static int ar0832_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int err;
	struct ar0832_dev *dev = NULL;
	int ret;

	dev_info(&client->dev, "ar0832: probing sensor.(id:%s)\n",
		id->name);

	dev = kzalloc(sizeof(struct ar0832_dev), GFP_KERNEL);
	if (!dev)
		goto probe_fail_release;

	dev->sensor_info = kzalloc(sizeof(struct ar0832_sensor_info),
					GFP_KERNEL);
	if (!dev->sensor_info)
		goto probe_fail_release;

	dev->focuser_info = kzalloc(sizeof(struct ar0832_focuser_info),
					GFP_KERNEL);
	if (!dev->focuser_info)
		goto probe_fail_release;

	/* sensor */
	dev->pdata = client->dev.platform_data;
	dev->i2c_client = client;

	/* focuser */
	dev->focuser_info->config.settle_time = SETTLETIME_MS;
	dev->focuser_info->config.pos_low = POS_LOW;
	dev->focuser_info->config.pos_high = POS_HIGH;

	snprintf(dev->dname, sizeof(dev->dname), "%s-%s",
		id->name, dev->pdata->id);
	dev->misc_dev.minor = MISC_DYNAMIC_MINOR;
	dev->misc_dev.name = dev->dname;
	dev->misc_dev.fops = &ar0832_fileops;
	dev->misc_dev.mode = S_IRWXUGO;
	dev->misc_dev.parent = &client->dev;
	err = misc_register(&dev->misc_dev);
	if (err) {
		dev_err(&client->dev, "Unable to register misc device!\n");
		ret = -ENOMEM;
		goto probe_fail_free;
	}

	i2c_set_clientdata(client, dev);
	mutex_init(&dev->ar0832_camera_lock);

	dev->power_rail.sen_1v8_reg = regulator_get(&client->dev, "vdd");
	if (IS_ERR_OR_NULL(dev->power_rail.sen_1v8_reg)) {
		dev_err(&client->dev, "%s: failed to get vdd\n",
			__func__);
		ret = PTR_ERR(dev->power_rail.sen_1v8_reg);
		goto probe_fail_free;
	}

	dev->power_rail.sen_2v8_reg = regulator_get(&client->dev, "vaa");
	if (IS_ERR_OR_NULL(dev->power_rail.sen_2v8_reg)) {
		dev_err(&client->dev, "%s: failed to get vaa\n",
			__func__);
		ret = PTR_ERR(dev->power_rail.sen_2v8_reg);
		regulator_put(dev->power_rail.sen_1v8_reg);
		dev->power_rail.sen_1v8_reg = NULL;
		goto probe_fail_free;
	}

	return 0;

probe_fail_release:
	dev_err(&client->dev, "%s: unable to allocate memory!\n", __func__);
	ret = -ENOMEM;
probe_fail_free:
	if (dev) {
		kfree(dev->focuser_info);
		kfree(dev->sensor_info);
	}
	kfree(dev);
	return ret;
}

static int ar0832_remove(struct i2c_client *client)
{
	struct ar0832_dev *dev = i2c_get_clientdata(client);

	if (dev->power_rail.sen_1v8_reg)
		regulator_put(dev->power_rail.sen_1v8_reg);
	if (dev->power_rail.sen_2v8_reg)
		regulator_put(dev->power_rail.sen_2v8_reg);

	misc_deregister(&dev->misc_dev);
	if (dev) {
		kfree(dev->sensor_info);
		kfree(dev->focuser_info);
	}
	kfree(dev);
	return 0;
}

static const struct i2c_device_id ar0832_id[] = {
	{ "ar0832", 0 },
	{ }
};

static struct i2c_driver ar0832_i2c_driver = {
	.probe = ar0832_probe,
	.remove = ar0832_remove,
	.id_table = ar0832_id,
	.driver = {
		.name = "ar0832",
		.owner = THIS_MODULE,
	},
};

static int __init ar0832_init(void)
{
	pr_info("%s: ++\n", __func__);
	return i2c_add_driver(&ar0832_i2c_driver);
}

static void __exit ar0832_exit(void)
{
	i2c_del_driver(&ar0832_i2c_driver);
}

module_init(ar0832_init);
module_exit(ar0832_exit);
