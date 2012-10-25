/*
 * linux/sound/soc/codecs/tlv320aic3262.h
 *
 *
 * Copyright (C) 2012 Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * History:
 *  Rev 0.1   ASoC driver support          20-01-2011
 *
 * The AIC3262 ASoC driver is ported for the codec AIC3262.
 *
 */

#ifndef _TLV320AIC3262_H
#define _TLV320AIC3262_H
#include <linux/input.h>
#define AUDIO_NAME "aic3262"
#define AIC3262_VERSION "1.1"

//#define AIC3262_ASI2_MASTER 1

/* Enable this macro allow for different ASI formats */
/*#define ASI_MULTI_FMT*/
#undef ASI_MULTI_FMT

#define  INT_FLAG2_BUTTN_PRESSBIT 0x20

/* Enable register caching on write */
#define EN_REG_CACHE 1

#define MULTIBYTE_CONFIG_SUPPORT

/*Setting all codec reg/write locally*/
/* This definition is added as the snd_ direct call are
result some issue with cache. Common code doesnot support
page, so fix that before commenting this line*/
#define LOCAL_REG_ACCESS 1

/* Macro to enable the inclusion of tiload kernel driver */
#define AIC3262_TiLoad


/* Macro enables or disables support for miniDSP in the driver */
/* Enable the AIC3262_TiLoad macro first before enabling these macros */
#define CONFIG_MINI_DSP
/*#undef CONFIG_MINI_DSP*/

/* Enable or disable controls to have Input routing*/
/*#define FULL_IN_CNTL */
#undef FULL_IN_CNTL
/* AIC3262 supported sample rate are 8k to 192k */
#define AIC3262_RATES	SNDRV_PCM_RATE_8000_192000

/* AIC3262 supports the word formats 16bits, 20bits, 24bits and 32 bits */
#define AIC3262_FORMATS	(SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE \
			 | SNDRV_PCM_FMTBIT_S24_3LE | SNDRV_PCM_FMTBIT_S32_LE)

#define AIC3262_FREQ_12000000 12000000
#define AIC3262_FREQ_12288000 12288000
#define AIC3262_FREQ_24000000 24000000

/* Macro for enabling the Multi_I2S Support in Driver */
#define AIC3262_MULTI_I2S	1

/* Driver Debug Messages Enabled */
//#define DEBUG

#ifdef DEBUG
	#define DBG(x...)	printk(x)
#else
	#define DBG(x...)
#endif

/*Select the below macro to decide on the DAC master volume controls.
 *2 independent or one combined
 */
/*#define DAC_INDEPENDENT_VOL*/
#undef DAC_INDEPENDENT_VOL

/* Audio data word length = 16-bits (default setting) */
#define AIC3262_WORD_LEN_16BITS		0x00
#define AIC3262_WORD_LEN_20BITS		0x01
#define AIC3262_WORD_LEN_24BITS		0x02
#define AIC3262_WORD_LEN_32BITS		0x03

/* sink: name of target widget */
#define AIC3262_WIDGET_NAME	0
/* control: mixer control name */
#define AIC3262_CONTROL_NAME
/* source: name of source name */
#define AIC3262_SOURCE_NAME	2

/* D15..D8 aic3262 register offset */
#define AIC3262_REG_OFFSET_INDEX    0
/* D7...D0 register data */
#define AIC3262_REG_DATA_INDEX      1

/* Serial data bus uses I2S mode (Default mode) */
#define AIC3262_I2S_MODE		0x00
#define AIC3262_DSP_MODE		0x01
#define AIC3262_RIGHT_JUSTIFIED_MODE	0x02
#define AIC3262_LEFT_JUSTIFIED_MODE	0x03

/* 8 bit mask value */
#define AIC3262_8BITS_MASK           0xFF

/* shift value for CLK_REG_3 register */
#define CLK_REG_3_SHIFT			6
/* shift value for DAC_OSR_MSB register */
#define DAC_OSR_MSB_SHIFT		4

/* number of codec specific register for configuration */
#define NO_FEATURE_REGS			2

/* Total number of ASI Ports */
#define MAX_ASI_COUNT			3


/* AIC3262 register space */
/* Updated from 256 to support Page 3 registers */
#define	AIC3262_CACHEREGNUM		1024
#define BIT7             (0x01 << 7)
#define BIT6             (0x01 << 6)
#define BIT5             (0x01 << 5)
#define BIT4             (0x01 << 4)
#define BIT3             (0x01 << 3)
#define BIT2             (0x01 << 2)
#define BIT1             (0x01 << 1)
#define BIT0             (0x01 << 0)

#define DAC_FLAG_MIC_MASKBITS	0x30
#define DAC_FLAG_HS_MASKBITS	0x03
#define DAC_FLAG_R1_NOJACK	0
#define DAC_FLAG_R1_NOMIC	(0x1 << 4)
#define DAC_FLAG_R1_MIC		(0x3 << 4)
#define DAC_FLAG_R1_NOHS	0
#define DAC_FLAG_R1_MONOHS	1
#define DAC_FLAG_R1_STEREOHS	2

/*mask patterns for DAC and ADC polling logic*/
#define LDAC_POW_FLAG_MASK	0x80
#define RDAC_POW_FLAG_MASK	0x08
#define LADC_POW_FLAG_MASK	0x40
#define RADC_POW_FLAG_MASK	0x04

/* ****************** Book 0 Registers **************************************/

/* ****************** Page 0 Registers **************************************/

#define PAGE_SEL_REG		0
#define RESET_REG		1
#define DAC_ADC_CLKIN_REG	4
#define PLL_CLKIN_REG		5
#define PLL_CLK_RANGE_REG	5
#define PLL_PR_POW_REG		6
#define PLL_J_REG		7
#define PLL_D_MSB		8
#define PLL_D_LSB		9
#define PLL_CKIN_DIV		10

#define NDAC_DIV_POW_REG	11
#define MDAC_DIV_POW_REG	12
#define DOSR_MSB_REG		13
#define DOSR_LSB_REG		14

#define NADC_DIV_POW_REG	18
#define MADC_DIV_POW_REG	19
#define AOSR_REG		20
#define CLKOUT_MUX		21
#define CLKOUT_MDIV_VAL		22
#define TIMER_REG		23

#define LF_CLK_CNTL		24
#define HF_CLK_CNTL_R1		25
#define HF_CLK_CNTL_R2		26
#define HF_CLK_CNTL_R3		27
#define HF_CLK_CNTL_R4		28
#define HF_CLK_TRIM_R1		29
#define HF_CLK_TRIM_R2		30
#define HF_CLK_TRIM_R3		31
#define HF_CLK_TRIM_R4		32
#define ADC_FLAG_R1		36
#define DAC_FLAG_R1		37
#define DAC_FLAG_R2		38

#define STICKY_FLAG1		42
#define INT_FLAG1		43
#define STICKY_FLAG2		44
#define STICKY_FLAG3		45
#define INT_FLAG2		46
#define INT1_CNTL		48
#define INT2_CNTL		49
#define INT_FMT			51

#define DAC_PRB			60
#define ADC_PRB			61
#define PASI_DAC_DP_SETUP	63
#define DAC_MVOL_CONF		64
#define DAC_LVOL		65
#define DAC_RVOL		66
#define HP_DETECT		67
#define DRC_CNTL_R1		68
#define DRC_CNTL_R2		69
#define DRC_CNTL_R3		70
#define BEEP_CNTL_R1		71
#define BEEP_CNTL_R2		72

#define ADC_CHANNEL_POW		81
#define ADC_FINE_GAIN		82
#define LADC_VOL		83
#define RADC_VOL		84
#define ADC_PHASE		85

#define LAGC_CNTL		86
#define LAGC_CNTL_R2		87
#define LAGC_CNTL_R3		88
#define LAGC_CNTL_R4		89
#define LAGC_CNTL_R5		90
#define LAGC_CNTL_R6		91
#define LAGC_CNTL_R7		92
#define LAGC_CNTL_R8		93

#define RAGC_CNTL		94
#define RAGC_CNTL_R2		95
#define RAGC_CNTL_R3		96
#define RAGC_CNTL_R4		97
#define RAGC_CNTL_R5		98
#define RAGC_CNTL_R6		99
#define RAGC_CNTL_R7		100
#define RAGC_CNTL_R8		101
#define MINIDSP_ACCESS_CTRL	121
/* ****************** Page 1 Registers **************************************/
#define PAGE_1			128

#define POWER_CONF		(PAGE_1 + 1)
#define LDAC_PTM		(PAGE_1 + 3)
#define RDAC_PTM		(PAGE_1 + 4)
#define CM_REG			(PAGE_1 + 8)
#define HP_CTL			(PAGE_1 + 9)
#define HP_DEPOP		(PAGE_1 + 11)
#define RECV_DEPOP		(PAGE_1 + 12)
#define MA_CNTL			(PAGE_1 + 17)
#define LADC_PGA_MAL_VOL	(PAGE_1 + 18)
#define RADC_PGA_MAR_VOL	(PAGE_1 + 19)


#define LINE_AMP_CNTL_R1	(PAGE_1 + 22)
#define LINE_AMP_CNTL_R2	(PAGE_1 + 23)

#define HP_AMP_CNTL_R1		(PAGE_1 + 27)
#define HP_AMP_CNTL_R2		(PAGE_1 + 28)
#define HP_AMP_CNTL_R3		(PAGE_1 + 29)

#define HPL_VOL			(PAGE_1 + 31)
#define HPR_VOL			(PAGE_1 + 32)
#define INT1_SEL_L		(PAGE_1 + 34)
#define RAMP_CNTL_R1		(PAGE_1 + 36)
#define RAMP_CNTL_R2		(PAGE_1 + 37)
//#define INT1_SEL_RM		(PAGE_1 + 39)
#define IN1L_SEL_RM		(PAGE_1 + 39)
#define IN1R_SEL_RM		(PAGE_1 + 39)

#define REC_AMP_CNTL_R5		(PAGE_1 + 40)
#define RAMPR_VOL		(PAGE_1 + 41)
#define RAMP_TIME_CNTL		(PAGE_1 + 42)
#define SPK_AMP_CNTL_R1		(PAGE_1 + 45)
#define SPK_AMP_CNTL_R2		(PAGE_1 + 46)
#define SPK_AMP_CNTL_R3		(PAGE_1 + 47)
#define SPK_AMP_CNTL_R4		(PAGE_1 + 48)
#define MIC_BIAS_CNTL		(PAGE_1 + 51)

#define LMIC_PGA_PIN		(PAGE_1 + 52)
#define LMIC_PGA_PM_IN4		(PAGE_1 + 53)
#define LMIC_PGA_MIN		(PAGE_1 + 54)
#define RMIC_PGA_PIN		(PAGE_1 + 55)
#define RMIC_PGA_PM_IN4		(PAGE_1 + 56)
#define RMIC_PGA_MIN		(PAGE_1 + 57)
/* MIC PGA Gain Registers */
#define MICL_PGA		(PAGE_1 + 59)
#define MICR_PGA		(PAGE_1 + 60)
#define HEADSET_TUNING1_REG	(PAGE_1 + 119)
#define HEADSET_TUNING2_REG	(PAGE_1 + 120)
#define MIC_PWR_DLY		(PAGE_1 + 121)
#define REF_PWR_DLY		(PAGE_1 + 122)

/* ****************** Page 4 Registers **************************************/
#define PAGE_4			512
#define ASI1_BUS_FMT		(PAGE_4 + 1)
#define ASI1_LCH_OFFSET		(PAGE_4 + 2)
#define ASI1_RCH_OFFSET		(PAGE_4 + 3)
#define ASI1_CHNL_SETUP		(PAGE_4 + 4)
#define ASI1_MULTI_CH_SETUP_R1	(PAGE_4 + 5)
#define ASI1_MULTI_CH_SETUP_R2	(PAGE_4 + 6)
#define ASI1_ADC_INPUT_CNTL	(PAGE_4 + 7)
#define ASI1_DAC_OUT_CNTL	(PAGE_4 + 8)
#define ASI1_ADC_OUT_TRISTATE	(PAGE_4 + 9)
#define ASI1_BWCLK_CNTL_REG	(PAGE_4 + 10)
#define ASI1_BCLK_N_CNTL	(PAGE_4 + 11)
#define ASI1_BCLK_N		(PAGE_4 + 12)
#define ASI1_WCLK_N		(PAGE_4 + 13)
#define ASI1_BWCLK_OUT_CNTL	(PAGE_4 + 14)
#define ASI1_DATA_OUT           (PAGE_4 + 15)
#define ASI2_BUS_FMT	        (PAGE_4 + 17)
#define ASI2_LCH_OFFSET		(PAGE_4 + 18)
#define ASI2_RCH_OFFSET		(PAGE_4 + 19)
#define ASI2_ADC_INPUT_CNTL	(PAGE_4 + 23)
#define ASI2_DAC_OUT_CNTL       (PAGE_4 + 24)
#define ASI2_BWCLK_CNTL_REG	(PAGE_4 + 26)
#define ASI2_BCLK_N_CNTL	(PAGE_4 + 27)
#define ASI2_BCLK_N		(PAGE_4 + 28)
#define ASI2_WCLK_N		(PAGE_4 + 29)
#define ASI2_BWCLK_OUT_CNTL	(PAGE_4 + 30)
#define ASI2_DATA_OUT		(PAGE_4 + 31)
#define ASI3_BUS_FMT            (PAGE_4 + 33)
#define ASI3_LCH_OFFSET		(PAGE_4 + 34)
#define ASI3_RCH_OFFSET		(PAGE_4 + 35)
#define ASI3_ADC_INPUT_CNTL	(PAGE_4 + 39)
#define ASI3_DAC_OUT_CNTL	(PAGE_4 + 40)
#define ASI3_BWCLK_CNTL_REG	(PAGE_4 + 42)
#define ASI3_BCLK_N_CNTL	(PAGE_4 + 43)
#define ASI3_BCLK_N             (PAGE_4 + 44)
#define ASI3_WCLK_N             (PAGE_4 + 45)
#define ASI3_BWCLK_OUT_CNTL	(PAGE_4 + 46)
#define ASI3_DATA_OUT		(PAGE_4 + 47)
#define WCLK1_PIN_CNTL_REG	(PAGE_4 + 65)
#define DOUT1_PIN_CNTL_REG	(PAGE_4 + 67)
#define DIN1_PIN_CNTL_REG	(PAGE_4 + 68)
#define WCLK2_PIN_CNTL_REG	(PAGE_4 + 69)
#define BCLK2_PIN_CNTL_REG	(PAGE_4 + 70)
#define DOUT2_PIN_CNTL_REG	(PAGE_4 + 71)
#define DIN2_PIN_CNTL_REG	(PAGE_4 + 72)
#define WCLK3_PIN_CNTL_REG	(PAGE_4 + 73)
#define BCLK3_PIN_CNTL_REG	(PAGE_4 + 74)
#define DOUT3_PIN_CNTL_REG	(PAGE_4 + 75)
#define DIN3_PIN_CNTL_REG	(PAGE_4 + 76)
#define MCLK2_PIN_CNTL_REG	(PAGE_4 + 82)
#define GPIO1_IO_CNTL		(PAGE_4 + 86)
#define GPIO2_IO_CNTL		(PAGE_4 + 87)
#define GPI1_EN			(PAGE_4 + 91)
#define GPO2_EN			(PAGE_4 + 92)
#define GPO1_PIN_CNTL		(PAGE_4 + 96)
#define MINIDSP_PORT_CNTL_REG	(PAGE_4 + 118)

/****************************************************************************
* Mixer control  related #defines
***************************************************************************
*/
#define WCLK1_ENUM            0
#define DOUT1_ENUM            1
#define DIN1_ENUM             2
#define WCLK2_ENUM            3
#define BCLK2_ENUM            4
#define DOUT2_ENUM            5
#define DIN2_ENUM             6
#define WCLK3_ENUM            7
#define BCLK3_ENUM            8
#define DOUT3_ENUM            9
#define DIN3_ENUM             10
#define CLKIN_ENUM            11
/*
*****************************************************************************
* Enumeration Definitions
*****************************************************************************
*/
/* The below enumeration lists down all the possible inputs to the
* the PLL of the AIC3262. The Private structure will hold a member
* of this Enumeration Type.
*/
enum AIC3262_PLL_OPTION {
	PLL_CLKIN_MCLK1 = 0,	/* 0000: (Device Pin) */
	PLL_CLKIN_BLKC1,	/* 0001: (Device Pin) */
	PLL_CLKIN_GPIO1,	/* 0010: (Device Pin)*/
	PLL_CLKIN_DIN1,		/* 0011: (Device Pin)*/
	PLL_CLKIN_BCLK2,	/* 0100: (Device Pin)*/
	PLL_CLKIN_GPI1,		/* 0101: (Device Pin)*/
	PLL_CLKIN_HF_REF_CLK,	/* 0110: (Device Pin)*/
	PLL_CLKIN_GPIO2,	/* 0111: (Device Pin)*/
	PLL_CLKIN_GPI2,		/* 1000: (Device Pin)*/
	PLL_CLKIN_MCLK2		/* 1001: (Device Pin)*/
};

/* ASI Specific Bit Clock Divider Input Options.
* Please refer to Page 4 Reg 11, Reg 27 and Reg 43
*/
enum ASI_BDIV_CLKIN_OPTION {
	BDIV_CLKIN_DAC_CLK = 0,		/* 00 DAC_CLK     */
	BDIV_CLKIN_DAC_MOD_CLK,		/* 01 DAC_MOD_CLK */
	BDIV_CLKIN_ADC_CLK,		/* 02 ADC_CLK	  */
	BDIV_CLKIN_ADC_MOD_CLK		/* 03 ADC_MOD_CLK */
};

/* ASI Specific Bit Clock Output Mux Options.
* Please refer to Page 4 Reg 14, Reg 30 and Reg 46
* Please note that we are not handling the Reserved
* cases here.
*/
enum ASI_BCLK_OPTION {
	ASI1_BCLK_DIVIDER_OUTPUT = 0,	/* 00 ASI1 Bit Clock Divider Output */
	ASI1_BCLK_INPUT,		/* 01 ASI1 Bit Clock Input */
	ASI2_BCLK_DIVIDER_OUTPUT,	/* 02 ASI2 Bit Clock Divider Output  */
	ASI2_BCLK_INPUT,		/* 03 ASI2 Bit Clock Input */
	ASI3_BCLK_DIVIDER_OUTPUT,	/* 04 ASI3 Bit Clock Divider Output  */
	ASI3_BBCLK_INPUT		/* 05 ASi3 Bit Clock Input */
};

/* Above bits are to be configured after Shifting 4 bits */
#define AIC3262_ASI_BCLK_MUX_SHIFT	4
#define AIC3262_ASI_BCLK_MUX_MASK	(BIT6 | BIT5 | BIT4)
#define AIC3262_ASI_WCLK_MUX_MASK	(BIT2 | BIT1 | BIT0)

/* ASI Specific Word Clock Output Mux Options */
enum ASI_WCLK_OPTION {
	GENERATED_DAC_FS = 0,		/* 00 WCLK = DAC_FS */
	GENERATED_ADC_FS = 1,		/* 01 WCLK = ADC_FS */
	ASI1_WCLK_DIV_OUTPUT = 2,	/* 02 WCLK = ASI1 WCLK_DIV_OUT */
	ASI1_WCLK_INPUT      = 3,	/* 03 WCLK = ASI1 WCLK Input   */
	ASI2_WCLK_DIV_OUTPUT = 4,	/* 04 WCLK = ASI2 WCLK_DIV_OUT */
	ASI2_WCLK_INPUT      = 5,	/* 05 WCLK = ASI2 WCLK Input   */
	ASI3_WCLK_DIV_OUTPUT = 6,	/* 06 WCLK = ASI3 WCLK_DIV_OUT */
	ASI3_WCLK_INPUT      = 7	/* 07 WCLK = ASI3 WCLK Input   */
};

/* ASI DAC Output Control Options */
enum ASI_DAC_OUTPUT_OPTION {
	DAC_PATH_OFF = 0,	/* 00 DAC Datapath Off */
	DAC_PATH_LEFT,		/* 01 DAC Datapath left Data */
	DAC_PATH_RIGHT,		/* 02 DAC Datapath Right Data */
};

#define AIC3262_READ_COMMAND_WORD(addr)   ((1 << 15) | (addr << 5))
#define AIC3262_WRITE_COMMAND_WORD(addr)  ((0 << 15) | (addr << 5))

/* Shift the above options by so many bits */
#define AIC3262_ASI_LDAC_PATH_SHIFT	6
#define AIC3262_ASI_LDAC_PATH_MASK	(BIT5 | BIT4)
#define AIC3262_ASI_RDAC_PATH_SHIFT	4
#define AIC3262_ASI_RDAC_PATH_MASK	(BIT7 | BIT6)


#define DAC_LR_MUTE_MASK	0xc
#define DAC_LR_MUTE		0xc
#define ENABLE_CLK_MASK		0x80
#define ENABLE_CLK		0x80

/* ASI specific ADC Input Control Options */
enum ASI_ADC_INPUT_OPTION {
	ADC_PATH_OFF = 0,	/* 00 ASI Digital Output Disabled */
	ADC_PATH_MINIDSP_1,	/* 01 ASI Digital O/P from miniDSP_A(L1,R1) */
	ADC_PATH_ASI1,		/* 02 ASI Digital Output from ASI1 */
	ADC_PATH_ASI2,		/* 03 ASI Digital Output from ASI2 */
	ADC_PATH_ASI3,		/* 04 ASI Digital Output from ASI3 */
	ADC_PATH_MINIDSP_2,	/* 05 ASI Digital O/P from miniDSP_A(L2,R2) */
	ADC_PATH_MINIDSP_3	/* 05 ASI Digital O/P from miniDSP_A(L3,R3) */
};

/* ASI Specific DOUT Pin Options */
enum ASI_DOUT_OPTION {
	ASI_OUTPUT = 0,		/* 00 Default ASI Output */
	ASI1_INPUT,		/* 01 ASI1 Data Input    */
	ASI2_INPUT,		/* 02 ASI2 Data Input    */
	ASI3_INPUT		/* 03 ASI3 Data Input    */
};

#define AIC3262_ASI_DOUT_MASK	(BIT1 | BIT0)

/*
 *****************************************************************************
 * Structures Definitions
 *****************************************************************************
 */
#define AIC3262_MULTI_ASI_ACTIVE(x) (((x)->asiCtxt[0].asi_active) || \
			((x)->asiCtxt[1].asi_active) || \
			((x)->asiCtxt[2].asi_active))

/*
*----------------------------------------------------------------------------
* @struct  aic3262_setup_data |
*          i2c specific data setup for AIC3262.
* @field   unsigned short |i2c_address |
*          Unsigned short for i2c address.
*----------------------------------------------------------------------------
*/
	struct aic3262_setup_data {
	unsigned short i2c_address;
};

/*
*----------------------------------------------------------------------------
* @struct aic3262_asi_data
*	ASI specific data stored for each ASI Interface
*
*
*---------------------------------------------------------------------------
*/
struct aic3262_asi_data {
	u8 asi_active;					/* ASI Active Flag */
	u8 master;					/* Frame Master */
	u32 sampling_rate;				/* Sampling Rate */
	enum ASI_BDIV_CLKIN_OPTION bclk_div_option;	/* BCLK DIV Mux Option*/
	enum ASI_BCLK_OPTION	bclk_output;		/* BCLK Output Option*/
	enum ASI_WCLK_OPTION	wclk_output;		/* WCLK Output Option*/
	u8			bclk_div;		/* BCLK Divider */
	u8			wclk_div;		/* WCLK Divider */
	enum ASI_DAC_OUTPUT_OPTION left_dac_output;	/* LDAC Path */
	enum ASI_DAC_OUTPUT_OPTION right_dac_output;	/* RDAC Path */
	enum ASI_ADC_INPUT_OPTION  adc_input;		/* ADC Input Control */
	enum ASI_DOUT_OPTION	dout_option;		/* DOUT Option */
	u8			playback_mode;		/* Playback Selected */
	u8			capture_mode;		/* Record Selected */
	u8			port_muted;		/* ASI Muted */
	u8			pcm_format;		/* PCM Format */
	u8			word_len;		/* Word Length */
	u8			offset1;		/* Left Ch offset */
	u8			offset2;		/* Right Ch Offset */
};

/*
*----------------------------------------------------------------------------
* @struct  aic3262_priv |
*          AIC3262 priviate data structure to set the system clock, mode and
*          page number.
* @field   u32 | sysclk |
*          system clock
* @field   s32 | master |
*          master/slave mode setting for AIC3262
* @field   u8 | book_no |
*          book number.
* @field   u8 | page_no |
*          page number. Here, page 0 and page 1 are used.
*----------------------------------------------------------------------------
*/
struct aic3262_priv {
	enum snd_soc_control_type control_type;
	struct aic326x_pdata *pdata;
	struct snd_soc_codec codec;
	u32 sysclk;
	s32 master;
	u8 book_no;
	u8 page_no;
	u8 process_flow;
	u8 mute_codec;
	u8 stream_status;
	u32 active_count;
	int current_dac_config[MAX_ASI_COUNT];
	int current_adc_config[MAX_ASI_COUNT];
	int current_config;
	struct aic3262_asi_data asiCtxt[MAX_ASI_COUNT];
	enum AIC3262_PLL_OPTION aic3262_pllclkin_option;
	u8 dac_clkin_option;
	u8 adc_clkin_option;
	int irq;
	u8 dac_reg;
	u8 adc_gain;
	u8 hpl;
	u8 hpr;
	u8 rec_amp;
	u8 rampr;
	u8 spk_amp;
	struct spi_device *spi;
	struct snd_soc_jack *headset_jack;
	struct input_dev *button_dev;
	int codec_audio_mode;
#if defined(LOCAL_REG_ACCESS)
	void *control_data;
#endif
};

/*
 *----------------------------------------------------------------------------
 * @struct  aic3262_configs |
 *          AIC3262 initialization data which has register offset and register
 *          value.
 * @field   u8 | book_no |
 *          AIC3262 Book Number Offsets required for initialization..
 * @field   u16 | reg_offset |
 *          AIC3262 Register offsets required for initialization..
 * @field   u8 | reg_val |
 *          value to set the AIC3262 register to initialize the AIC3262.
 *----------------------------------------------------------------------------
 */
struct aic3262_configs {
	u8 book_no;
	u16 reg_offset;
	u8  reg_val;
};

/*
 *----------------------------------------------------------------------------
 * @struct  aic3262_rate_divs |
 *          Setting up the values to get different freqencies
 * @field   u32 | mclk |
 *          Master clock
 * @field   u32 | rate |
 *          sample rate
 * @field   u8 | p_val |
 *          value of p in PLL
 * @field   u32 | pll_j |
 *          value for pll_j
 * @field   u32 | pll_d |
 *          value for pll_d
 * @field   u32 | dosr |
 *          value to store dosr
 * @field   u32 | ndac |
 *          value for ndac
 * @field   u32 | mdac |
 *          value for mdac
 * @field   u32 | aosr |
 *          value for aosr
 * @field   u32 | nadc |
 *          value for nadc
 * @field   u32 | madc |
 *          value for madc
 * @field   u32 | blck_N |
 *          value for block N
 * @field   u32 | aic3262_configs |
 *          configurations for aic3262 register value
 *----------------------------------------------------------------------------
 */
struct aic3262_rate_divs {
	u32 mclk;
	u32 rate;
	u8 p_val;
	u8 pll_j;
	u16 pll_d;
	u16 dosr;
	u8 ndac;
	u8 mdac;
	u8 aosr;
	u8 nadc;
	u8 madc;
	u8 blck_N;
	struct aic3262_configs codec_specific_regs[NO_FEATURE_REGS];
};

/*
*****************************************************************************
* EXTERN DECLARATIONS
*****************************************************************************
*/
/*
 *----------------------------------------------------------------------------
 * @func  aic326x_headset_detect
 *      This function help to setup the needed registers to
 *      enable the headset detection
 *
 */
extern int aic326x_headset_detect(struct snd_soc_codec *codec,
	struct snd_soc_jack *jack, int jack_type);
extern int aic326x_headset_button_init(struct snd_soc_codec *codec,
	struct snd_soc_jack *jack, int jack_type);

extern u8 aic3262_read(struct snd_soc_codec *codec, u16 reg);
extern u16 aic3262_read_2byte(struct snd_soc_codec *codec, u16 reg);
extern int aic3262_reset_cache(struct snd_soc_codec *codec);
extern int aic3262_change_page(struct snd_soc_codec *codec, u8 new_page);
extern int aic3262_write(struct snd_soc_codec *codec, u16 reg, u8 value);
extern void aic3262_write_reg_cache(struct snd_soc_codec *codec,
				    u16 reg, u8 value);
extern int aic3262_change_book(struct snd_soc_codec *codec, u8 new_book);
extern int reg_def_conf(struct snd_soc_codec *codec);
extern int i2c_verify_book0(struct snd_soc_codec *codec);
extern int poll_dac(struct snd_soc_codec *codec, int left_right, int on_off);
extern int poll_adc(struct snd_soc_codec *codec, int left_right, int on_off);

#ifdef CONFIG_MINI_DSP
extern int aic3262_minidsp_program(struct snd_soc_codec *codec);
extern int aic3262_add_minidsp_controls(struct snd_soc_codec *codec);
#endif


#ifdef MULTIBYTE_CONFIG_SUPPORT
extern int aic3262_add_multiconfig_controls(struct snd_soc_codec *codec);
#endif

#endif				/* _TLV320AIC3262_H */

