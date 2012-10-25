/*
 * mt9m113.c - mt9m113 sensor driver
 *
 * Copyright (c) 2011, NVIDIA, All Rights Reserved.
 *
 * Contributors:
 *      hyeongjin.kim<hyeongjin.kim@lge.com>
 *
 * Leverage mt9m113.c
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

/**
 * SetMode Sequence for 640x480. Phase 0. Sensor Dependent.
 * This sequence should put sensor in streaming mode for 640x480
 * This is usually given by the FAE or the sensor vendor.
 */

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <media/mt9m113.h>
#include <linux/gpio.h>

//MOBII_CHANGE_S dongki.han : customsetting
DEFINE_MUTEX(mt9m113_camera_lock);

#define MT9M113_WAIT_TIMES 100

typedef struct ExposureValueTypeRec{
    int index;
    int range;
}ExposureValueType;

typedef struct FpsRangeTypeRec{
    int low;
    int high;
}FpsRangeType;

//MOBII_CHANGE_E dongki.han : customsetting

struct mt9m113_reg {
	u16 addr;
	u16 val;
};

struct mt9m113_info {
	int mode;
	struct i2c_client *i2c_client;
	struct mt9m113_platform_data *pdata;
};
#define MT9M113_DEBUG

#ifdef MT9M113_DEBUG
	#define mt_info pr_info
#else
	#define mt_info(arg...) do {} while (0)
#endif

#define  mt_err pr_err

#define MT9M113_TABLE_WAIT_MS 0
#define MT9M113_TABLE_END 1
#define MT9M113_MAX_RETRIES 3

//MOBII_CHANGE_S dongki.han : customsetting
static struct mt9m113_info *info;

static int mt9m113_get_status(struct mt9m113_info *info,
		struct mt9m113_status *dev_status);

//MOBII_CHANGE_E dongki.han : customsetting

static struct mt9m113_reg init_table[] = {
// potential candidate for star lgp990 1.3M camera init code. it was no error record in around 20000 times
{0x001C, 0x0001},
{0x001C, 0x0000},
{MT9M113_TABLE_WAIT_MS, 10},
{0x0014,0x304A},
{0xFFFF, 0x121C},
{0x0016, 0x40FF},
{0x0018, 0x0028},
{0x0014, 0x2147},
{0x0014, 0x2143},
{0x0014, 0x2145},
{0x0010, 0x0114},
{0x0012, 0x1FF1},
{0x0014, 0x2545},
{0x0014, 0x2547},
{0x0014, 0x3447},
{MT9M113_TABLE_WAIT_MS, 20},
{0x0014, 0x3047},		 //PLL control: TEST_BYP  SS off = 12358
{0x0014, 0x3046},		 //PLL control: PLL_BYPA  llow PLL to lock 
// sungmin.woo modification start	
{MT9M113_TABLE_WAIT_MS, 5},
{0x0014, 0x3047},
{MT9M113_TABLE_WAIT_MS, 5},
{0x0014, 0x3046},
{0x001E, 0x0771}, // 20110222 sungmin.woo@lge.com for minor module noise issue fix
{0x332E, 0x0200}, // new 2010 0705	
{0x098C, 0x2703},
{0x0990, 0x0280},//640 W
{0x098C, 0x2705},
{0x0990, 0x01E0},// 480 W
{0x098C, 0x2707},
{0x0990, 0x0500},//1280
{0x098C, 0x2709},
{0x0990, 0x03C0},//960
{0x098C, 0x270D},
{0x0990, 0x0000},
{0x098C, 0x270F},
{0x0990, 0x0000},
{0x098C, 0x2711},
{0x0990, 0x03CD},
{0x098C, 0x2713},
{0x0990, 0x050D},
{0x098C, 0x2715},
{0x0990, 0x2111},
{0x098C, 0x2717},
{0x0990, 0x046C}, // Jongkyung.kim 20110613 MIRRORED / Flip IMAGES for CAPTURE
{0x098C, 0x2719},
{0x0990, 0x00AC},
{0x098C, 0x271B},
{0x0990, 0x01F1},
{0x098C, 0x271D},
{0x0990, 0x013F},
{0x098C, 0x271F},// sensor frame length (A)
{0x0990, 0x0239},
{0x098C, 0x2721},
{0x0990, 0x0726},  //6DD
{0x098C, 0x2723},
{0x0990, 0x0004},
{0x098C, 0x2725},
{0x0990, 0x0004},
{0x098C, 0x2727},
{0x0990, 0x03CB},  // FROM APTINA CHOI'S FIX
{0x098C, 0x2729},
{0x0990, 0x050B},
{0x098C, 0x272B},
{0x0990, 0x2111},
{0x098C, 0x272D},
{0x0990, 0x0024}, // Jongkyung.kim 20110613 MIRRORED / Flip IMAGES for CAPTURE
{0x098C, 0x272F},
{0x0990, 0x004C},
{0x098C, 0x2731},
{0x0990, 0x00F9},
{0x098C, 0x2733},
{0x0990, 0x00A7},
{0x098C, 0x2735},
{0x0990, 0x045B},
{0x098C, 0x2737},
{0x0990, 0x0726},  //72D
{0x098C, 0x2739},
{0x0990, 0x0000},
{0x098C, 0x273B},
{0x0990, 0x027F},
{0x098C, 0x273D},
{0x0990, 0x0000},
{0x098C, 0x273F},
{0x0990, 0x01DF},
{0x098C, 0x2747},
{0x0990, 0x0000},
{0x098C, 0x2749},
{0x0990, 0x04FF},
{0x098C, 0x274B},
{0x0990, 0x0000},
{0x098C, 0x274D},
{0x0990, 0x03BF},  // FROM APTINA CHOI'S FIX
{0x098C, 0x222D},
{0x0990, 0x0089},  //8E
{0x098C, 0xA404},
{0x0990, 0x0010},
{0x098C, 0xA408},
{0x0990, 0x0021},  //22
{0x098C, 0xA409},
{0x0990, 0x0023},  //24
{0x098C, 0xA40A},
{0x0990, 0x0028},  //29
{0x098C, 0xA40B},
{0x0990, 0x002A},  //2B
{0x098C, 0x2411},
{0x0990, 0x0089},  //8E
{0x098C, 0x2413},
{0x0990, 0x00A4},  //AB
{0x098C, 0x2415},
{0x0990, 0x0089},  //88
{0x098C, 0x2417},
{0x0990, 0x00A4},  //A3
{0x098C, 0xA40D},
{0x0990, 0x0002},
{0x098C, 0xA40E},
{0x0990, 0x0003},
{0x098C, 0xA410},
{0x0990, 0x000A},
//[HIGH POWER PREVIEW MODE]
{0x098C, 0x275F},
{0x0990, 0x0596},
{0x098C, 0x2761},
{0x0990, 0x0094},
//[PEVUEW 0 SEQ]
{0x098C, 0xA117},
{0x0990, 0x0002},
{0x098C, 0xA118},
{0x0990, 0x0001},
{0x098C, 0xA119},
{0x0990, 0x0001},
{0x098C, 0xA11A},
{0x0990, 0x0001},
//[PEVUEW 1 SEQ]
{0x098C, 0xA11D},
{0x0990, 0x0002},
{0x098C, 0xA11E},
{0x0990, 0x0001},
{0x098C, 0xA11F},
{0x0990, 0x0001},
{0x098C, 0xA120},
{0x0990, 0x0001},
//[LENS CORRECTION 95%]
{0x364E, 0x0730},
{0x3650, 0x1E4E},
{0x3652, 0x14F1},
{0x3654, 0x32CF},
{0x3656, 0x734F},
{0x3658, 0x0330},
{0x365A, 0xF1AD},
{0x365C, 0x2731},
{0x365E, 0x550E},
{0x3660, 0x6F8F},
{0x3662, 0x0250},
{0x3664, 0x01AE},
{0x3666, 0x7FD0},
{0x3668, 0x03EF},
{0x366A, 0x442F},
{0x366C, 0x0250},
{0x366E, 0x82EE},
{0x3670, 0x0591},
{0x3672, 0x128E},
{0x3674, 0x540F},
{0x3676, 0x650D},
{0x3678, 0x714C},
{0x367A, 0x272E},
{0x367C, 0x09CA},
{0x367E, 0xD12E},
{0x3680, 0x652D},
{0x3682, 0x9C0D},
{0x3684, 0x172F},
{0x3686, 0xB64D},
{0x3688, 0xB3EF},
{0x368A, 0x1F6D},
{0x368C, 0xAA2D},
{0x368E, 0x68CA},
{0x3690, 0x300D},
{0x3692, 0x826E},
{0x3694, 0x13CD},
{0x3696, 0x37AB},
{0x3698, 0x2B2D},
{0x369A, 0xB52A},
{0x369C, 0x874F},
{0x369E, 0x5390},
{0x36A0, 0x0370},
{0x36A2, 0x3812},
{0x36A4, 0xFAB0},
{0x36A6, 0xBE13},
{0x36A8, 0x7F90},
{0x36AA, 0xA5AC},
{0x36AC, 0x0393},
{0x36AE, 0x1DAD},
{0x36B0, 0xF693},
{0x36B2, 0x3C90},
{0x36B4, 0x5CCF},
{0x36B6, 0x3E72},
{0x36B8, 0x8F70},
{0x36BA, 0xABB3},
{0x36BC, 0x5390},
{0x36BE, 0x06AE},
{0x36C0, 0x23F2},
{0x36C2, 0x9FB0},
{0x36C4, 0x93F3},
{0x36C6, 0xA0AA},
{0x36C8, 0x43CD},
{0x36CA, 0x4E0A},
{0x36CC, 0xE750},
{0x36CE, 0xDAF1},
{0x36D0, 0x50EB},
{0x36D2, 0x88CB},
{0x36D4, 0xAE0C},
{0x36D6, 0x256E},
{0x36D8, 0xA36F},
{0x36DA, 0xC06E},
{0x36DC, 0x844D},
{0x36DE, 0x8CED},
{0x36E0, 0x9110},
{0x36E2, 0xADD1},
{0x36E4, 0xEA2D},
{0x36E6, 0x1BAE},
{0x36E8, 0xD86E},
{0x36EA, 0xBF0D},
{0x36EC, 0xED30},
{0x36EE, 0x224F},
{0x36F0, 0xCC10},
{0x36F2, 0xAAB2},
{0x36F4, 0x510E},
{0x36F6, 0xDC54},
{0x36F8, 0x29B0},
{0x36FA, 0xE5ED},
{0x36FC, 0x90F3},
{0x36FE, 0xA811},
{0x3700, 0xE814},
{0x3702, 0x3810},
{0x3704, 0xFE2F},
{0x3706, 0xA1B2},
{0x3708, 0x328D},
{0x370A, 0xBC34},
{0x370C, 0x2D4F},
{0x370E, 0xEA70},
{0x3710, 0xBCF1},
{0x3712, 0x7BD1},
{0x3714, 0xED54},
{0x3644, 0x0280},
{0x3642, 0x01C0},
{0x3210, 0x01A8},
{0x098C, 0x2306},
{0x0990, 0x00A4},
{0x098C, 0x2308},
{0x0990, 0x00D4},
{0x098C, 0x230A},
{0x0990, 0xFFBD},
{0x098C, 0x230C},
{0x0990, 0xFF68},
{0x098C, 0x230E},
{0x0990, 0x0284},
{0x098C, 0x2310},
{0x0990, 0xFF70},
{0x098C, 0x2312},
{0x0990, 0xFFE5},
{0x098C, 0x2314},
{0x0990, 0xFF1B},
{0x098C, 0x2316},
{0x0990, 0x0259},
{0x098C, 0x2318}, 
{0x0990, 0x002B},			  
{0x098C, 0x231A},
{0x0990, 0x0042},	  
{0x098C, 0x231C},
{0x0990, 0x002D},			 
{0x098C, 0x231E},
{0x0990, 0xFF32},		  
{0x098C, 0x2320},
{0x0990, 0x0045},   
{0x098C, 0x2322},
{0x0990, 0x0034},
{0x098C, 0x2324},
{0x0990, 0xFF52},			
{0x098C, 0x2326},
{0x0990, 0x002A},   
{0x098C, 0x2328},
{0x0990, 0xFFF8},
{0x098C, 0x232A},
{0x0990, 0x0052},   
{0x098C, 0x232C},
{0x0990, 0xFF61},  
{0x098C, 0x232E},
{0x0990, 0x0008},
{0x098C, 0x2330},
{0x0990, 0xFFE7}, 
{0x098C, 0xA348},
{0x0990, 0x0008},
{0x098C, 0xA349},
{0x0990, 0x0002},
{0x098C, 0xA34A},
{0x0990, 0x0059},
{0x098C, 0xA34B},
{0x0990, 0x00A6},
{0x098C, 0xA34C},
{0x0990, 0x0059},
{0x098C, 0xA34D},
{0x0990, 0x00A6},
{0x098C, 0xA351},
{0x0990, 0x0000},
{0x098C, 0xA352},
{0x0990, 0x007F},
{0x098C, 0xA354},
{0x0990, 0x0043},
{0x098C, 0xA355},
{0x0990, 0x0002},
{0x098C, 0xA35D},
{0x0990, 0x0078},
{0x098C, 0xA35E},
{0x0990, 0x0086},
{0x098C, 0xA35F},
{0x0990, 0x007E},
{0x098C, 0xA360},
{0x0990, 0x0082},
{0x098C, 0xA365},
{0x0990, 0x0010},
//[TRUE GRAY] 
{0x098C, 0xA363},
{0x0990, 0x00C7},
{0x098C, 0xA364},
{0x0990, 0x00F0},
//[K FACTOR]
{0x098C, 0xA366},
{0x0990, 0x007D},
{0x098C, 0xA367},
{0x0990, 0x0080},
{0x098C, 0xA368},
{0x0990, 0x0080},
{0x098C, 0xA369},
{0x0990, 0x0080},
{0x098C, 0xA36A},
{0x0990, 0x0077},
{0x098C, 0xA36B},
{0x0990, 0x0078},
//LOWLIGHT BLUISH
{0x35A2, 0x00A4},
{0x3240, 0xC802},
//[AFD]
////AUTO FLICKER DETECTION
{0x098C, 0xA11E},
{0x0990, 0x0001},
{0x098C, 0xA404},
{0x0990, 0x0000},
//[VIRTGAIN]
{0x098C, 0xA20C},
{0x0990, 0x0011},
{0x098C, 0x2212},
{0x0990, 0x01A0},
{0x098C, 0xA20D},
{0x0990, 0x0017},
{0x098C, 0xA20E},
{0x0990, 0x0080},
{0x098C, 0xA216},
{0x0990, 0x0060},
{0x098C, 0xA11D},
{0x0990, 0x0002},
//[NO GAIN UP3] 	   
{0x098C, 0xAB2C},
{0x0990, 0x0006},
{0x098C, 0xAB2D},
{0x0990, 0x000E},
{0x098C, 0xAB2E},
{0x0990, 0x0006},
{0x098C, 0xAB2F},
{0x0990, 0x0006},
{0x098C, 0xAB30},
{0x0990, 0x001E},
{0x098C, 0xAB31},
{0x0990, 0x000E},
{0x098C, 0xAB32},
{0x0990, 0x001E},
{0x098C, 0xAB33},
{0x0990, 0x001E},
{0x098C, 0xAB34},
{0x0990, 0x0008},
{0x098C, 0xAB35},
{0x0990, 0x0080},
//[AE SETTING]			 
{0x098C, 0xA207},	
{0x0990, 0x0004},
{0x098C, 0xA24F},	
{0x0990, 0x004D},
{0x098C, 0x2257},	
{0x0990, 0x2710},
{0x098C, 0x2250},	
{0x0990, 0x1B58},
{0x098C, 0x2252},	
{0x0990, 0x32C8},
//[BLACK GAMMA CONTRAST]
{0x098C, 0x2B1B},	// MCU_ADDRESS [HG_BRIGHTNESSMETRIC]
{0x0990, 0x0643},	// MCU_DATA_0
{0x098C, 0xAB37},	// MCU_ADDRESS [HG_GAMMA_MORPH_CTRL]
{0x0990, 0x0003},	// MCU_DATA_0
{0x098C, 0x2B38},	// MCU_ADDRESS [HG_GAMMASTARTMORPH]
{0x0990, 0x2800},			// MCU_DATA_0
{0x098C, 0x2B3A},	// MCU_ADDRESS [HG_GAMMASTOPMORPH]
{0x0990, 0x46FE},	// MCU_DATA_0
//BLACK 06
{0x098C, 0xAB3C},	// MCU_ADDRESS [HG_GAMMA_TABLE_A_0]
{0x0990, 0x0000},	// MCU_DATA_0
{0x098C, 0xAB3D},	// MCU_ADDRESS [HG_GAMMA_TABLE_A_1]
{0x0990, 0x0008},	// MCU_DATA_0
{0x098C, 0xAB3E},	// MCU_ADDRESS [HG_GAMMA_TABLE_A_2]
{0x0990, 0x0019},	// MCU_DATA_0
{0x098C, 0xAB3F},	// MCU_ADDRESS [HG_GAMMA_TABLE_A_3]
{0x0990, 0x0035},	// MCU_DATA_0
{0x098C, 0xAB40},	// MCU_ADDRESS [HG_GAMMA_TABLE_A_4]
{0x0990, 0x0056},	// MCU_DATA_0
{0x098C, 0xAB41},	// MCU_ADDRESS [HG_GAMMA_TABLE_A_5]
{0x0990, 0x006F},	// MCU_DATA_0
{0x098C, 0xAB42},	// MCU_ADDRESS [HG_GAMMA_TABLE_A_6]
{0x0990, 0x0085},	// MCU_DATA_0
{0x098C, 0xAB43},	// MCU_ADDRESS [HG_GAMMA_TABLE_A_7]
{0x0990, 0x0098},	// MCU_DATA_0
{0x098C, 0xAB44},	// MCU_ADDRESS [HG_GAMMA_TABLE_A_8]
{0x0990, 0x00A7},	// MCU_DATA_0
{0x098C, 0xAB45},	// MCU_ADDRESS [HG_GAMMA_TABLE_A_9]
{0x0990, 0x00B4},	// MCU_DATA_0
{0x098C, 0xAB46},	// MCU_ADDRESS [HG_GAMMA_TABLE_A_10]
{0x0990, 0x00C0},	// MCU_DATA_0
{0x098C, 0xAB47},	// MCU_ADDRESS [HG_GAMMA_TABLE_A_11]
{0x0990, 0x00CA},	// MCU_DATA_0
{0x098C, 0xAB48},	// MCU_ADDRESS [HG_GAMMA_TABLE_A_12]
{0x0990, 0x00D4},	// MCU_DATA_0
{0x098C, 0xAB49},	// MCU_ADDRESS [HG_GAMMA_TABLE_A_13]
{0x0990, 0x00DC},	// MCU_DATA_0
{0x098C, 0xAB4A},	// MCU_ADDRESS [HG_GAMMA_TABLE_A_14]
{0x0990, 0x00E4},	// MCU_DATA_0
{0x098C, 0xAB4B},	// MCU_ADDRESS [HG_GAMMA_TABLE_A_15]
{0x0990, 0x00EC},	// MCU_DATA_0
{0x098C, 0xAB4C},	// MCU_ADDRESS [HG_GAMMA_TABLE_A_16]
{0x0990, 0x00F3},	// MCU_DATA_0
{0x098C, 0xAB4D},	// MCU_ADDRESS [HG_GAMMA_TABLE_A_17]
{0x0990, 0x00F9},	// MCU_DATA_0
{0x098C, 0xAB4E},	// MCU_ADDRESS [HG_GAMMA_TABLE_A_18]
{0x0990, 0x00FF},	// MCU_DATA_0
//BLACK 05 CONTRAST 1.35
{0x098C, 0xAB4F},	// MCU_ADDRESS [HG_GAMMA_TABLE_B_0]
{0x0990, 0x0000},	// MCU_DATA_0
{0x098C, 0xAB50},	// MCU_ADDRESS [HG_GAMMA_TABLE_B_1]
{0x0990, 0x0006},	// MCU_DATA_0
{0x098C, 0xAB51},	// MCU_ADDRESS [HG_GAMMA_TABLE_B_2]
{0x0990, 0x0012},	// MCU_DATA_0
{0x098C, 0xAB52},	// MCU_ADDRESS [HG_GAMMA_TABLE_B_3]
{0x0990, 0x002F},	// MCU_DATA_0
{0x098C, 0xAB53},	// MCU_ADDRESS [HG_GAMMA_TABLE_B_4]
{0x0990, 0x0053},	// MCU_DATA_0
{0x098C, 0xAB54},	// MCU_ADDRESS [HG_GAMMA_TABLE_B_5]
{0x0990, 0x006D},	// MCU_DATA_0
{0x098C, 0xAB55},	// MCU_ADDRESS [HG_GAMMA_TABLE_B_6]
{0x0990, 0x0083},	// MCU_DATA_0
{0x098C, 0xAB56},	// MCU_ADDRESS [HG_GAMMA_TABLE_B_7]
{0x0990, 0x0096},	// MCU_DATA_0
{0x098C, 0xAB57},	// MCU_ADDRESS [HG_GAMMA_TABLE_B_8]
{0x0990, 0x00A6},	// MCU_DATA_0
{0x098C, 0xAB58},	// MCU_ADDRESS [HG_GAMMA_TABLE_B_9]
{0x0990, 0x00B3},	// MCU_DATA_0
{0x098C, 0xAB59},	// MCU_ADDRESS [HG_GAMMA_TABLE_B_10]
{0x0990, 0x00BF},	// MCU_DATA_0
{0x098C, 0xAB5A},	// MCU_ADDRESS [HG_GAMMA_TABLE_B_11]
{0x0990, 0x00CA},	// MCU_DATA_0
{0x098C, 0xAB5B},	// MCU_ADDRESS [HG_GAMMA_TABLE_B_12]
{0x0990, 0x00D3},	// MCU_DATA_0
{0x098C, 0xAB5C},	// MCU_ADDRESS [HG_GAMMA_TABLE_B_13]
{0x0990, 0x00DC},	// MCU_DATA_0
{0x098C, 0xAB5D},	// MCU_ADDRESS [HG_GAMMA_TABLE_B_14]
{0x0990, 0x00E4},	// MCU_DATA_0
{0x098C, 0xAB5E},	// MCU_ADDRESS [HG_GAMMA_TABLE_B_15]
{0x0990, 0x00EB},	// MCU_DATA_0
{0x098C, 0xAB5F},	// MCU_ADDRESS [HG_GAMMA_TABLE_B_16]
{0x0990, 0x00F2},	// MCU_DATA_0
{0x098C, 0xAB60},	// MCU_ADDRESS [HG_GAMMA_TABLE_B_17]
{0x0990, 0x00F9},	// MCU_DATA_0
{0x098C, 0xAB61},	// MCU_ADDRESS [HG_GAMMA_TABLE_B_18]
{0x0990, 0x00FF},	// MCU_DATA_0
//[LL-MODE-MODifIED]
{0x098C, 0xAB04},	// MCU_ADDRESS [HG_MAX_DLEVEL]
{0x0990, 0x0040},	// MCU_DATA_0
{0x098C, 0xAB06},	// MCU_ADDRESS [HG_PERCENT]
{0x0990, 0x0005},		// 3, //0x000A	// MCU_DATA_0
{0x098C, 0xAB08},	// MCU_ADDRESS [HG_DLEVEL]
{0x0990, 0x0010},	// MCU_DATA_0
{0x098C, 0xAB20},	// MCU_ADDRESS [HG_LL_SAT1]
{0x0990, 0x0050},   //63,	// MCU_DATA_0
{0x098C, 0xAB21},	// MCU_ADDRESS [RESERVED_HG_21]
{0x0990, 0x001D},	// MCU_DATA_0
{0x098C, 0xAB22},	// MCU_ADDRESS [RESERVED_HG_22]
{0x0990, 0x0007},	// MCU_DATA_0
{0x098C, 0xAB23},	// MCU_ADDRESS [RESERVED_HG_23]
{0x0990, 0x0004}, //0x000A	// MCU_DATA_0
{0x098C, 0xAB24},	// MCU_ADDRESS [HG_LL_SAT2]
{0x0990, 0x0000},	// MCU_DATA_0
{0x098C, 0xAB25},	// MCU_ADDRESS [RESERVED_HG_25]
{0x0990, 0x00A0},	 //0x0014	// MCU_DATA_0
{0x098C, 0xAB26},	// MCU_ADDRESS [RESERVED_HG_26]
{0x0990, 0x0005},	// MCU_DATA_0
{0x098C, 0xAB27},	// MCU_ADDRESS [RESERVED_HG_27]
{0x0990, 0x0010},	// MCU_DATA_0
{0x098C, 0x2B28},	// MCU_ADDRESS [HG_LL_BRIGHTNESSSTART]
{0x0990, 0x157C}, //0x0A00 //0x157C  //0x0A00  // MCU_DATA_0
{0x098C, 0x2B2A},  // MCU_ADDRESS [HG_LL_BRIGHTNESSSTOP]
{0x0990, 0x37EF},   //0x7000 // MCU_DATA_0
////////////////////////////////////// NOISE REDUCTION /////////////////////////////////////
{0x098C, 0x2717},
{0x0990, 0x046C},// Jongkyung.kim 20110613 MIRRORED / Flip IMAGES for CAPTURE
//{0x0990, 0x046D},  // 1116 ??????
//ENTER REFRESH MODE
{0x098C,0xA103 },
{0x0990,0x0006 },
{MT9M113_TABLE_WAIT_MS, 10},
{0x098C,0xA103 },
{0x0990,0x0005 },
{MT9M113_TABLE_WAIT_MS, 10},

{MT9M113_TABLE_END, 0x0000}
};

static struct mt9m113_reg mode_1280x960[] = {
{0x098C, 0xA115},
{0x0990, 0x0002},
{0x098C, 0xA103},
{0x0990, 0x0002},
{MT9M113_TABLE_WAIT_MS, 50},

{MT9M113_TABLE_END, 0x0000}
};

static struct mt9m113_reg mode_640x480[] = {
{0x098C, 0xA115},
{0x0990, 0x0000},
{0x098C, 0xA103},
{0x0990, 0x0001},
{MT9M113_TABLE_WAIT_MS, 50},

{MT9M113_TABLE_END, 0x0000}
};

enum {
	MT9M113_MODE_1280x960,
	MT9M113_MODE_640x480,
};

static struct mt9m113_reg *mode_table[] = {
	[MT9M113_MODE_1280x960] = mode_1280x960,
	[MT9M113_MODE_640x480] = mode_640x480,
};

//MOBII_CHANGE_S dongki.han : customsetting
static struct mt9m113_reg mt9m113_Color_Effect_Normal[] =
{
	{0x098C, 0x2759},
	{0x0990, 0x6440},
	{0x098C, 0x275B},
	{0x0990, 0x6440},
	{0x098C, 0xA103},
	{0x0990, 0x0005},
    {MT9M113_TABLE_WAIT_MS, 0x0000},
    {MT9M113_TABLE_END, 0x0000}
};

static struct mt9m113_reg mt9m113_Color_Effect_MONO[] =
{
	{0x098C,0x2759},
	{0x0990,0x6441},
	{0x098C,0x275B},
	{0x0990,0x6441},
	{0x098C,0xA103},
	{0x0990,0x0005},
	{MT9M113_TABLE_WAIT_MS, 0x0000},
	{MT9M113_TABLE_END, 0x0000}
};

static struct mt9m113_reg mt9m113_Color_Effect_NEGATIVE[] =
{
	{0x098C,0x2759},
	{0x0990,0x6443},
	{0x098C,0x275B},
	{0x0990,0x6443},
	{0x098C,0xA103},
	{0x0990,0x0005},
	{MT9M113_TABLE_WAIT_MS, 0x0000},
	{MT9M113_TABLE_END, 0x0000}
};

static struct mt9m113_reg mt9m113_Color_Effect_SEPIA[] =
{
	{0x098C,0x2759},
	{0x0990,0x6442},
	{0x098C,0x275B},
	{0x0990,0x6442},
	{0x098C,0xA103},
	{0x0990,0x0005},
	{MT9M113_TABLE_WAIT_MS, 0x0000},
	{MT9M113_TABLE_END, 0x0000}
};

static struct mt9m113_reg mt9m113_Color_Effect_SOLARIZE[] =
{
	{0x098C,0x2759},
	{0x0990,0x4845},
	{0x098C,0x275B},
	{0x0990,0x4845},
	{0x098C,0xA103},
	{0x0990,0x0005},
	{MT9M113_TABLE_WAIT_MS, 0x0000},
	{MT9M113_TABLE_END, 0x0000}
};

static struct mt9m113_reg mt9m113_MWB_Auto[] =
{
    {0x098C,0xA11F},
    {0x0990,0x0001},
    {0x098C,0xA351},
    {0x0990,0x0000},
    {0x098C,0xA352},
    {0x0990,0x007F},
    {0x098C,0xA34A},
    {0x0990,0x0059},
    {0x098C,0xA34B},
    {0x0990,0x00A6},
    {0x098C,0xA34C},
    {0x0990,0x0059},
    {0x098C,0xA34D},
    {0x0990,0x00A6},
    {0x098C,0xA103},
    {0x0990,0x0006},
    {MT9M113_TABLE_WAIT_MS, 0x0000},
    {0x098C,0xA103},
    {0x0990,0x0005},
    {MT9M113_TABLE_WAIT_MS, 0x0000},
    {MT9M113_TABLE_END, 0x0000}
};


static struct mt9m113_reg mt9m113_MWB_Incandescent[] =
{
    {0x098C,0xA11F},
    {0x0990,0x0000},
    {0x098C,0xA351},
    {0x0990,0x0010},
    {0x098C,0xA352},
    {0x0990,0x0010},
    {0x098C,0xA353},
    {0x0990,0x0010},
    {0x098C,0xA34A},
    {0x0990,0x006c},
    {0x098C,0xA34B},
    {0x0990,0x006c},
    {0x098C,0xA34C},
    {0x0990,0x0090},
    {0x098C,0xA34D},
    {0x0990,0x0090},
    {0x098C,0xA103},
    {0x0990,0x0006},
    {MT9M113_TABLE_WAIT_MS, 0x0000},
    {0x098C,0xA103},
    {0x0990,0x0005},
    {MT9M113_TABLE_WAIT_MS, 0x0000},
    {MT9M113_TABLE_END, 0x00}
};

static struct mt9m113_reg mt9m113_MWB_Fluorescent[] =
{
    {0x098C,0xA11F},
    {0x0990,0x0000},
    {0x098C,0xA351},
    {0x0990,0x005F},
    {0x098C,0xA352},
    {0x0990,0x005F},
    {0x098C,0xA353},
    {0x0990,0x005F},
    {0x098C,0xA34A},
    {0x0990,0x007a},
    {0x098C,0xA34B},
    {0x0990,0x007a},
    {0x098C,0xA34C},
    {0x0990,0x0094},
    {0x098C,0xA34D},
    {0x0990,0x0094},
    {0x098C,0xA103},
    {0x0990,0x0006},
    {MT9M113_TABLE_WAIT_MS, 0x0000},
    {0x098C,0xA103},
    {0x0990,0x0005},
    {MT9M113_TABLE_WAIT_MS, 0x0000},
    {MT9M113_TABLE_END, 0x00}
};

static struct mt9m113_reg mt9m113_MWB_Daylight[] =
{
    {0x098C,0xA11F},
    {0x0990,0x0000},
    {0x098C,0xA351},
    {0x0990,0x007F},
    {0x098C,0xA352},
    {0x0990,0x007F},
    {0x098C,0xA353},
    {0x0990,0x007F},
    {0x098C,0xA34A},
    {0x0990,0x008f},
    {0x098C,0xA34B},
    {0x0990,0x008f},
    {0x098C,0xA34C},
    {0x0990,0x0092},
    {0x098C,0xA34D},
    {0x0990,0x0092},
    {0x098C,0xA103},
    {0x0990,0x0006},
    {MT9M113_TABLE_WAIT_MS, 0x0000},
    {0x098C,0xA103},
    {0x0990,0x0005},
    {MT9M113_TABLE_WAIT_MS, 0x0000},
    {MT9M113_TABLE_END, 0x00}
    };

static struct mt9m113_reg mt9m113_MWB_CloudyDaylight[] =
{
    {0x098C,0xA11F},
    {0x0990,0x0000},
    {0x098C,0xA351},
    {0x0990,0x0074},
    {0x098C,0xA352},
    {0x0990,0x0074},
    {0x098C,0xA353},
    {0x0990,0x0074},
    {0x098C,0xA34A},
    {0x0990,0x0095},
    {0x098C,0xA34B},
    {0x0990,0x0095},
    {0x098C,0xA34C},
    {0x0990,0x0090},
    {0x098C,0xA34D},
    {0x0990,0x0090},
    {0x098C,0xA103},
    {0x0990,0x0006},
    {MT9M113_TABLE_WAIT_MS, 0x0000},
    {0x098C,0xA103},
    {0x0990,0x0005},
    {MT9M113_TABLE_WAIT_MS, 0x0000},
    {MT9M113_TABLE_END, 0x00}
};

static struct mt9m113_reg mt9m113_Framerate_5Fps[] =
{
    {0x098C,0x271F},
    {0x0990,0x0CCE},
    {0x098C,0xA20C},
    {0x0990,0x0005},
    {0x098C,0xA103},
    {0x0990,0x0006},
    {MT9M113_TABLE_END, 0x00}
};

static struct mt9m113_reg mt9m113_Framerate_7Fps[] =
{
    {0x098C,0x271F},
    {0x0990,0x0925},
    {0x098C,0xA20C},
    {0x0990,0x0005},
    {0x098C,0xA103},
    {0x0990,0x0006},
    {MT9M113_TABLE_END, 0x00}
};

static struct mt9m113_reg mt9m113_Framerate_10Fps[] =
{
    {0x098C,0x271F},
    {0x0990,0x0667},
    {0x098C,0xA20C},
    {0x0990,0x0005},
    {0x098C,0xA103},
    {0x0990,0x0006},
    {MT9M113_TABLE_END, 0x00}
};

static struct mt9m113_reg mt9m113_Framerate_12Fps[] =
{
    {0x098C,0x271F},
    {0x0990,0x0444},
    {0x098C,0xA20C},
    {0x0990,0x0005},
    {0x098C,0xA103},
    {0x0990,0x0006},
    {MT9M113_TABLE_END, 0x00}
};

static struct mt9m113_reg mt9m113_Framerate_15Fps[] =
{
    {0x098C,0x271F},
    {0x0990,0x0444},
    {0x098C,0xA20C},
    {0x0990,0x0005},
    {0x098C,0xA103},
    {0x0990,0x0006},
    {MT9M113_TABLE_END, 0x00}
};

static struct mt9m113_reg mt9m113_Framerate_20Fps[] =
{
    {0x098C,0x271F},
    {0x0990,0x0333},
    {0x098C,0xA20C},
    {0x0990,0x0005},
    {0x098C,0xA103},
    {0x0990,0x0006},
    {MT9M113_TABLE_END, 0x00}
};

static struct mt9m113_reg mt9m113_Framerate_24Fps[] =
{
    {0x098C,0x271F},
    {0x0990,0x02AB},
    {0x098C,0xA20C},
    {0x0990,0x0005},
    {0x098C,0xA103},
    {0x0990,0x0006},
    {MT9M113_TABLE_END, 0x00}
};

static struct mt9m113_reg mt9m113_Framerate_25Fps[] =
{
    {0x098C,0x271F},
    {0x0990,0x028F},
    {0x098C,0xA20C},
    {0x0990,0x0005},
    {0x098C,0xA103},
    {0x0990,0x0006},
    {MT9M113_TABLE_END, 0x00}
};

static struct mt9m113_reg mt9m113_Framerate_30Fps[] =
{
    {0x098C,0x271F},
    {0x0990,0x0239},
    {0x098C,0xA20C},
    {0x0990,0x0005},
    {0x098C,0xA103},
    {0x0990,0x0006},
    {MT9M113_TABLE_END, 0x00}
};

#if 1 // MOBII_CHANGE dk.han 20120813 
static struct mt9m113_reg mt9m113_Framerate_AutoFps[] =
{
    {0x098C,0x271F},
    {0x0990,0x0239},
    {0x098C,0x2721},
    {0x0990,0x0726},  //6DD
    {0x098C,0xA20C},
    {0x0990,0x0005},
    {0x098C, 0xA103},   // MCU_ADDRESS [SEQ_CMD]
    {0x0990, 0x0006},   // MCU_DATA_0
    {MT9M113_TABLE_WAIT_MS, 0},
    {0x098C, 0xA103},   // MCU_ADDRESS [SEQ_CMD]
    {0x0990, 0x0005},   // MCU_DATA_0
    {MT9M113_TABLE_WAIT_MS, 0},
    {MT9M113_TABLE_END, 0x00}
};
#else
static struct mt9m113_reg mt9m113_Framerate_AutoFps[] =
{
    {0x098C,0x271F},
    {0x0990,0x0239},
    {0x098C,0x2721},
    {0x0990,0x0726},  //6DD
    {0x098C,0xA20C},
    {0x0990,0x0005},
    {0x098C,0xA103},
    {0x0990,0x0006},
    {MT9M113_TABLE_END, 0x00}
};
#endif

static struct mt9m113_reg *mt9m113_Set_FramerateList[] =
{
	mt9m113_Framerate_5Fps,
	mt9m113_Framerate_7Fps,	
	mt9m113_Framerate_10Fps,
	mt9m113_Framerate_12Fps,
	mt9m113_Framerate_15Fps,
	mt9m113_Framerate_20Fps,
	mt9m113_Framerate_24Fps,
	mt9m113_Framerate_25Fps,
	mt9m113_Framerate_30Fps,
	mt9m113_Framerate_AutoFps
};

static int brightnessValues[13] = {
              17, // 0
              27, // 1
              37, // 2
              47, // 3
              57, // 4
              67, // 5
              77, // 6
              87, // 7
              97, // 8
              107, // 9
              117, //10
              127, //11
              145 // 12
              };

#if 1 // MOBII_CHANGE dk.han 20120813 
static struct mt9m113_reg mt9m113_ScenMode_Night[] =
{
	{0x098C, 0x2212},
	{0x0990, 0x0250},
	{0x098C, 0xA103},   // MCU_ADDRESS [SEQ_CMD]
	{0x0990, 0x0006},   // MCU_DATA_0
	{MT9M113_TABLE_WAIT_MS, 0},
	{0x098C, 0xA103},   // MCU_ADDRESS [SEQ_CMD]
	{0x0990, 0x0005},   // MCU_DATA_0
	{MT9M113_TABLE_WAIT_MS, 0},
    {MT9M113_TABLE_END, 0x0000}
};
#else
static struct mt9m113_reg mt9m113_ScenMode_Night[] =
{
	{0x098C, 0xA20C},
	{0x0990, 0x0018},
	{0x098C, 0xA103},
	{0x0990, 0x0005},
    {MT9M113_TABLE_END, 0x0000}
};
#endif

#if 1 // MOBII_CHANGE dk.han 20120813
static struct mt9m113_reg mt9m113_ScenMode_Auto[] =
{
        {0x098C, 0x2212},
        {0x0990, 0x01A0},
        {0x098C, 0xA103},   // MCU_ADDRESS [SEQ_CMD]
        {0x0990, 0x0006},   // MCU_DATA_0
        {MT9M113_TABLE_WAIT_MS, 0},
        {0x098C, 0xA103},   // MCU_ADDRESS [SEQ_CMD]
        {0x0990, 0x0005},   // MCU_DATA_0
        {MT9M113_TABLE_WAIT_MS, 0},
    {MT9M113_TABLE_END, 0x0000}
};
#else
static struct mt9m113_reg mt9m113_ScenMode_Auto[] =
{
	{0x098C,0xA20C},
	{0x0990,0x0010},
	{0x098C,0xA103},
	{0x0990,0x0005},    
	{MT9M113_TABLE_END, 0x0000}
};
#endif

//MOBII_CHANGE_E dongki.han : customsetting


static int mt9m113_read_reg(struct i2c_client *client, u16 addr, u16 *val)
{
	int err;
	struct i2c_msg msg[2];
	unsigned char data[4];

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
	msg[1].len = 2;
	msg[1].buf = data + 2;

	err = i2c_transfer(client->adapter, msg, 2);

	if (err != 2)
		return -EINVAL;

	*val = data[2] << 8 | data[3];

	return 0;
}

static int mt9m113_write_reg(struct i2c_client *client, u16 addr, u16 val)
{
	int err;
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

	do {
		err = i2c_transfer(client->adapter, &msg, 1);
		//mt_info("mt9m113: i2c transfer under transferring %x %x\n", addr, val);
		if (err == 1)
			return 0;
		retry++;
		mt_err("mt9m113: i2c transfer failed, retrying %x %x\n",
		       addr, val);
		msleep(3);
	} while (retry <= MT9M113_MAX_RETRIES);

	return err;
}

static int mt9m113_write_table(struct i2c_client *client,
			      const struct mt9m113_reg table[],
			      const struct mt9m113_reg override_list[],
			      int num_override_regs)
{
	int err;
	const struct mt9m113_reg *next;
	int i;
	u16 val;
//MOBII_CHANGE_S dongki.han : customsetting
    struct mt9m113_status dev_status;
//MOBII_CHANGE_E dongki.han : customsetting

	mt_info("mt9m113: mt9m113_write_table entered\n");
	for (next = table; next->addr != MT9M113_TABLE_END; next++) {
		//mt_info("mt9m113: mt9m113_write_table 1\n");
		if (next->addr == MT9M113_TABLE_WAIT_MS) {
//MOBII_CHANGE_S dongki.han : customsetting
            if (next->val == 0){
                for(i = 0; i < MT9M113_WAIT_TIMES; i++) {
					//mt_info("mt9m113: MT9M113_WAIT_TIME\n");
                    dev_status.data = 0xA103;
                    dev_status.status = 0;
                    err = mt9m113_get_status(info, &dev_status);
                    if (err) {
                        return err;
                    }
                    if (dev_status.status == 0) {
                        break;
                    }
                }
                continue;
            }
//MOBII_CHANGE_E dongki.han : customsetting
			mt_info("mt9m113: mt9m113_write_table : MT9M113_TABLE_WAIT_MS \n");
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

		//mt_info("mt9m113: mt9m113_write_table 2\n");
		err = mt9m113_write_reg(client, next->addr, val);
		if (err) {
			mt_err("mt9m113: mt9m113_write_table : err\n");
			return err;
		}
	}
	return 0;
}

static int mt9m113_init_sensor(struct mt9m113_info *info)
{
	int err;

        mt_info("mt9m113: init sensor\n");
        err = mt9m113_write_table(info->i2c_client, init_table, NULL, 0);
        if (err)
                return err;

	return 0;
}

static int mt9m113_set_mode(struct mt9m113_info *info, struct mt9m113_mode *mode)
{
	int sensor_mode;
	int err;

	mt_info("%s: xres %u yres %u\n", __func__, mode->xres, mode->yres);
	if (mode->xres == 640 && mode->yres == 480) {
		sensor_mode = MT9M113_MODE_640x480;
	}
	else if (mode->xres == 1280 && mode->yres == 960) {
		sensor_mode = MT9M113_MODE_1280x960;
	}
	else {
		mt_err("%s: invalid resolution supplied to set mode %d %d\n",
		       __func__, mode->xres, mode->yres);
		return -EINVAL;
	}


	mt_info("mt9m113: mt9m113_set_mode before write table\n");
	err = mt9m113_write_table(info->i2c_client, mode_table[sensor_mode],
		NULL, 0);
	if (err)
		return err;

	info->mode = sensor_mode;
	return 0;
}

static int mt9m113_get_status(struct mt9m113_info *info,
		struct mt9m113_status *dev_status)
{
	int err;

	err = mt9m113_write_reg(info->i2c_client, 0x98C, dev_status->data);
	if (err)
		return err;

	err = mt9m113_read_reg(info->i2c_client, 0x0990,
		(u16 *) &dev_status->status);
	if (err)
		return err;

	return err;
}

//MOBII_CHANGE_S dongki.han : customsetting
static int mt9m113_set_color_effect(struct mt9m113_info *info, unsigned int color_effect)
{
	int err = 0;
	pr_info("%s : %d\n", __func__, color_effect);

	switch(color_effect)
	{
		case YUVCamColorEffect_None :
			err = mt9m113_write_table(info->i2c_client, mt9m113_Color_Effect_Normal,NULL,0);
			break;

		case YUVCamColorEffect_Negative :
			err = mt9m113_write_table(info->i2c_client, mt9m113_Color_Effect_NEGATIVE,NULL,0);
			break;

		case YUVCamColorEffect_Sepia :
			err = mt9m113_write_table(info->i2c_client, mt9m113_Color_Effect_SEPIA,NULL,0);
			break;

		case YUVCamColorEffect_Mono :
			err = mt9m113_write_table(info->i2c_client, mt9m113_Color_Effect_MONO,NULL,0);
            break;

		case YUVCamColorEffect_Solarize:
			err = mt9m113_write_table(info->i2c_client, mt9m113_Color_Effect_SOLARIZE,NULL,0);
			break;

		default :
			//err = mt9m113_write_table(info->i2c_client, mt9m113_Color_Effect_Normal,NULL,0);

			break;
	}
	if (err !=0)
		pr_info("%s : Color Effect : %d,  error : %d\n", __func__, color_effect, err);

	return err;
}

static int mt9m113_set_white_balance(struct mt9m113_info *info, unsigned int wb_mode)
{
	int err = 0;
	pr_info("%s : %d\n", __func__, wb_mode);
	switch(wb_mode)
	{
		case YUVCamWhitebalance_Auto :
			err = mt9m113_write_table(info->i2c_client, mt9m113_MWB_Auto,NULL,0);
			break;

		case YUVCamWhitebalance_Incandescent :
			err = mt9m113_write_table(info->i2c_client, mt9m113_MWB_Incandescent,NULL,0);

			break;

		case YUVCamWhitebalance_SunLight :
			err = mt9m113_write_table(info->i2c_client, mt9m113_MWB_Daylight,NULL,0);

			break;
		case YUVCamWhitebalance_CloudyDayLight:
			err = mt9m113_write_table(info->i2c_client, mt9m113_MWB_CloudyDaylight,NULL,0);
			
			break;

		case YUVCamWhitebalance_Fluorescent :
			err = mt9m113_write_table(info->i2c_client, mt9m113_MWB_Fluorescent,NULL,0);

			break;

		default :
			//err = mt9m113_write_table(info->i2c_client, mt9m113_MWB_Auto,NULL,0);

			break;
	}
	if (err !=0)
		pr_info("%s : White Balance : %d,  error : %d\n", __func__, wb_mode, err);
	return err;
}

#define EXP_COMPENSATION_MIN  -20
#define EXP_COMPENSATION_MAX  20

static int mt9m113_set_exposure(struct mt9m113_info *info, ExposureValueType *exposure)
{
	int err = 0;
	s8 value;

	if(exposure == NULL || exposure->range == 0)
	{
		pr_info("%s : invalid pointer or range value\n", __func__);
	    return -1;
	}
    
    if (exposure->index >= EXP_COMPENSATION_MAX)
        value = 12;
    else if (exposure->index >= 16)
        value = 11;
    else if (exposure->index >= 12)
        value = 10;
    else if (exposure->index >= 10)
        value = 9;
    else if (exposure->index >= 6)
        value = 8;
    else if (exposure->index >= 2)
        value = 7;
    else if (exposure->index >= 0)
        value = 6;
    else if (exposure->index >= -2)
        value = 5;
    else if (exposure->index >= -6)
        value = 4;
    else if (exposure->index >= -10)
        value = 3;
    else if (exposure->index >= -12)
        value = 2;
    else if (exposure->index >= -16)
        value = 1;
    else if (exposure->index <= EXP_COMPENSATION_MIN)
        value = 0;

    pr_info(" %s : exp(%d) value(%d) range(%d)\n", __func__, exposure->index, value, exposure->range);

    err = mt9m113_write_reg(info->i2c_client, 0x98C, 0xA24F);
	if (err)
	    return err;
    err = mt9m113_write_reg(info->i2c_client, 0x0990, brightnessValues[value]);

	return err;
}

static int mt9m113_set_fpsrange(struct mt9m113_info *info, FpsRangeType *fpsRange)
{
	int err = 0;

	int lo = fpsRange->low;
	int hi = fpsRange->high;
	pr_info("%s:lo=%d, hi=%d\n", __func__, lo, hi);		
	if (lo == hi){//fixed framerate
		if (lo==5){
			err =  mt9m113_write_table(info->i2c_client,mt9m113_Set_FramerateList[0],NULL,0);		
		}
		else if (lo == 7){
			err =  mt9m113_write_table(info->i2c_client,mt9m113_Set_FramerateList[1],NULL,0);	
		}
		else if (lo == 10){
			err =  mt9m113_write_table(info->i2c_client,mt9m113_Set_FramerateList[2],NULL,0);	
		}
        else if (lo == 12){
			err =  mt9m113_write_table(info->i2c_client,mt9m113_Set_FramerateList[3],NULL,0);	
		}
		else if (lo == 15){
			err =  mt9m113_write_table(info->i2c_client,mt9m113_Set_FramerateList[4],NULL,0);	
		}
        else if (lo == 20){
			err =  mt9m113_write_table(info->i2c_client,mt9m113_Set_FramerateList[5],NULL,0);	
		}
        else if (lo == 24){
			err =  mt9m113_write_table(info->i2c_client,mt9m113_Set_FramerateList[6],NULL,0);	
		}
        else if (lo == 25){
			err =  mt9m113_write_table(info->i2c_client,mt9m113_Set_FramerateList[7],NULL,0);	
		}
        else if (lo == 30){
			err =  mt9m113_write_table(info->i2c_client,mt9m113_Set_FramerateList[8],NULL,0);	
		}
	}
	else{//variable framerate
		err = mt9m113_write_table(info->i2c_client,mt9m113_Set_FramerateList[9],NULL,0);
	}

	return err;
}

static int mt9m113_set_scene_mode(struct mt9m113_info *info, unsigned int scene_mode)
{
	int err = 0;
	pr_info("%s : %d\n", __func__, scene_mode);

	switch(scene_mode)
	{
		case YUVCamSceneMode_Auto :
			err = mt9m113_write_table(info->i2c_client, mt9m113_ScenMode_Auto,NULL,0);
			break;

		case YUVCamSceneMode_Night :
			err = mt9m113_write_table(info->i2c_client, mt9m113_ScenMode_Night,NULL,0);
			break;

		default :
			//err = mt9m113_write_table(info->i2c_client, mt9m113_ScenMode_Auto,NULL,0);

			break;
	}
	if (err !=0)
		pr_info("%s : Scene Mode : %d,  error : %d\n", __func__, scene_mode, err);

	return err;
}
//MOBII_CHANGE_E dongki.han : customsetting

static long mt9m113_ioctl(struct file *file,
			 unsigned int cmd, unsigned long arg)
{
	int err;
	struct mt9m113_info *info = file->private_data;
	
	mt_info("mt9m113: mt9m113_ioctl \n");

	switch (cmd) {
	case MT9M113_IOCTL_SET_MODE:
	{
		struct mt9m113_mode mode;
		if (copy_from_user(&mode,
				   (const void __user *)arg,
				   sizeof(struct mt9m113_mode))) {
			return -EFAULT;
		}

		mt_info("mt9m113: mt9m113_ioctl : MT9M113_IOCTL_SET_MODE\n");
		return mt9m113_set_mode(info, &mode);
	}

//MOBII_CHANGE_S dongki.han : customsetting
	case MT9M113_IOCTL_SET_COLOR_EFFECT :
	{
		unsigned int color_effect;

		if (copy_from_user(&color_effect,
					(const void __user *)arg,
					sizeof(unsigned int))) {
			return -EFAULT;
		}
		mutex_lock(&mt9m113_camera_lock);
		err = mt9m113_set_color_effect(info, color_effect);
		mutex_unlock(&mt9m113_camera_lock);
		pr_info("    :MT9M113_IOCTL_SET_COLOR_EFFECT(%d)\n", color_effect);
		return err; 
	}
  
	case MT9M113_IOCTL_SET_WHITE_BALANCE :
	{
		unsigned int white_balance;

		if (copy_from_user(&white_balance,
					(const void __user *)arg,
					sizeof(unsigned int))) {
			return -EFAULT;
		}
		mutex_lock(&mt9m113_camera_lock);	
        err = mt9m113_set_white_balance(info, white_balance);		
		mutex_unlock(&mt9m113_camera_lock);
		pr_info("    :MT9M113_IOCTL_SET_WHITE_BALANCE(%d)\n", white_balance);
		return err; 
	}
    
	case MT9M113_IOCTL_SET_EXPOSURE:
	{
		ExposureValueType exposure;

		if (copy_from_user(&exposure,
					(const void __user *)arg,
					sizeof(ExposureValueType))) {
			return -EFAULT;
		}
		mutex_lock(&mt9m113_camera_lock);
        err = mt9m113_set_exposure(info, &exposure);		
		mutex_unlock(&mt9m113_camera_lock);
		pr_info("    :MT9M113_IOCTL_SET_EXPOSURE(%d/%d)\n", exposure.index, exposure.range);
		return err; 
	}

    case MT9M113_IOCTL_SET_FPSRANGE:
    {
        FpsRangeType FpsRange;

        if (copy_from_user(&FpsRange,
                    (const void __user *)arg,
                    sizeof(FpsRangeType))) {
            return -EFAULT;
        }
        mutex_lock(&mt9m113_camera_lock);
        err = mt9m113_set_fpsrange(info, &FpsRange);      
        mutex_unlock(&mt9m113_camera_lock);
        pr_info("    :MT9M113_IOCTL_SET_FPSRANGE(%d:%d)\n", FpsRange.low, FpsRange.high);
        return err;         
    }

	case MT9M113_IOCTL_SET_SCENE_MODE:
	{
		unsigned int scene_mode;

		if (copy_from_user(&scene_mode,
					(const void __user *)arg,
					sizeof(unsigned int))) {
			return -EFAULT;
		}
		mutex_lock(&mt9m113_camera_lock);	
        err = mt9m113_set_scene_mode(info, scene_mode);		
		mutex_unlock(&mt9m113_camera_lock);
		pr_info("    :MT9M113_IOCTL_SET_SCENE_MODE(%d)\n", scene_mode);
		return err; 
	}   
//MOBII_CHANGE_E dongki.han : customsetting

	default:
		mt_info("mt9m113: mt9m113_ioctl : default\n");
		return -EINVAL;
	}
	return 0;
}

//MOBII_CHANGE_S dongki.han : customsetting
//static struct mt9m113_info *info;
//MOBII_CHANGE_E dongki.han : customsetting

static int mt9m113_open(struct inode *inode, struct file *file)
{
	struct mt9m113_status dev_status;
	int err;

	mt_info("mt9m113: mt9m113_open\n");

	file->private_data = info;
	if (info->pdata && info->pdata->power_on)
		info->pdata->power_on();

	dev_status.data = 0;
	dev_status.status = 0;
	err = mt9m113_get_status(info, &dev_status);
	if (err) {
		mt_err("mt9m113: mt9m113_get_status failed\n");
		return err;
	}

	err = mt9m113_init_sensor(info);
	if (err) {
		mt_err("mt9m113: mt9m113_init_sensor failed\n");
	}

	return 0;
}

int mt9m113_release(struct inode *inode, struct file *file)
{

	mt_info("mt9m113: mt9m113_release\n");
	if (info->pdata && info->pdata->power_off)
		info->pdata->power_off();

	file->private_data = NULL;
	return 0;
}

static const struct file_operations mt9m113_fileops = {
	.owner = THIS_MODULE,
	.open = mt9m113_open,
	.unlocked_ioctl = mt9m113_ioctl,
	.release = mt9m113_release,
};

static struct miscdevice mt9m113_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "mt9m113",
	.fops = &mt9m113_fileops,
};

static int mt9m113_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int err;

	mt_info("mt9m113: probing sensor.\n");

	info = kzalloc(sizeof(struct mt9m113_info), GFP_KERNEL);
	if (!info) {
		mt_err("mt9m113: Unable to allocate memory!\n");
		return -ENOMEM;
	}

	err = misc_register(&mt9m113_device);
	if (err) {
		mt_err("mt9m113: Unable to register misc device!\n");
		kfree(info);
		return err;
	}

	info->mode = -1;
	info->pdata = client->dev.platform_data;
	info->i2c_client = client;

	i2c_set_clientdata(client, info);

	return 0;
}

static int mt9m113_remove(struct i2c_client *client)
{
	struct mt9m113_info *info;
	info = i2c_get_clientdata(client);
	misc_deregister(&mt9m113_device);
	kfree(info);
	return 0;
}

static const struct i2c_device_id mt9m113_id[] = {
	{ "mt9m113", 0 },
	{ },
};

MODULE_DEVICE_TABLE(i2c, mt9m113_id);

static struct i2c_driver mt9m113_i2c_driver = {
	.driver = {
		.name = "mt9m113",
		.owner = THIS_MODULE,
	},
	.probe = mt9m113_probe,
	.remove = mt9m113_remove,
	.id_table = mt9m113_id,
};

static int __init mt9m113_init(void)
{
	mt_info("mt9m113 sensor driver loading\n");
	return i2c_add_driver(&mt9m113_i2c_driver);
}

static void __exit mt9m113_exit(void)
{
	i2c_del_driver(&mt9m113_i2c_driver);
}

module_init(mt9m113_init);
module_exit(mt9m113_exit);
