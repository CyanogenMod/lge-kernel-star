/*
* linux/sound/soc/codecs/tlv320aic3262.c
*
* Copyright (C) 2012 Texas Instruments, Inc.
*
* Based on sound/soc/codecs/tlv320aic3262.c
*
* This package is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
* IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
* WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
*
* The TLV320AIC3262 is a flexible, low-power, low-voltage stereo audio
* codec with digital microphone inputs and programmable outputs.
*
* History:
*
* Rev 0.1   ASoC driver support       20-01-2011
*
*		The AIC325x ASoC driver is ported for the codec AIC3262.
* Rev 0.2   ASoC driver support      21-03-2011
*		The AIC326x ASoC driver is updated abe changes.
*
* Rev 0.3   ASoC driver support      12.09.2011
*		fixed the compilation issues for Whistler support
*
* Rev 0.4   ASoC driver support     27.09.2011
*              The AIC326x driver ported for Nvidia cardhu.
*
* Rev 0.5   Modified to support Multiple ASI Ports  08-Nov-2011
*		Driver updated to support ASI Ports of AIC3262
*
* Modified by Nvidia 23-Nov-2011 for K39 ASoC changes.
*/

/*
 *****************************************************************************
 * INCLUDES
 *****************************************************************************
 */
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#include <asm/div64.h>
#include <sound/tlv320aic326x.h>
#include <sound/jack.h>
#include <linux/spi/spi.h>

#include "tlv320aic326x.h"
#include <linux/gpio.h>
/*
 *****************************************************************************
 * Global Variable
 *****************************************************************************
 */
static u8 aic3262_reg_ctl;

#ifdef AIC3262_TiLoad
	extern int aic3262_driver_init(struct snd_soc_codec *codec);
#endif



/* whenever aplay/arecord is run, aic3262_hw_params() function gets called.
 * This function reprograms the clock dividers etc. this flag can be used to
 * disable this when the clock dividers are programmed by pps config file
 */
static int soc_static_freq_config = 1;
static struct aic3262_priv *aic3262_priv_data;
static struct i2c_client *i2c_pdev;
static struct snd_soc_codec *aic3262_codec;

/*
 *****************************************************************************
 * Macros
 *****************************************************************************
 */

/* ASoC Widget Control definition for a single Register based Control */
#define SOC_SINGLE_AIC3262(xname) \
{\
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname, \
	.info = __new_control_info, .get = __new_control_get,\
	.put = __new_control_put, \
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE, \
}
#define SOC_SINGLE_N(xname, xreg, xshift, xmax, xinvert) \
{\
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname, \
	.info = n_control_info, .get = n_control_get,\
	.put = n_control_put, \
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE, \
	.private_value =  ((unsigned long)&(struct soc_mixer_control)) \
	{.reg = xreg, .shift = xshift, .rshift = xshift, .max = xmax, \
	.invert = xinvert} }

/* ASoC Widget Control definition for a Double Register based Control */

#define SOC_DOUBLE_R_N(xname, reg_left, reg_right, xshift, xmax, xinvert) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname), \
	.info = snd_soc_info_volsw_2r_n, \
	.get = snd_soc_get_volsw_2r_n, .put = snd_soc_put_volsw_2r_n, \
	.private_value = (unsigned long)&(struct soc_mixer_control) \
		{.reg = reg_left, .rreg = reg_right, .shift = xshift, \
		.max = xmax, .invert = xinvert} }

#define SND_SOC_DAPM_SWITCH_N(wname, wreg, wshift, winvert) \
{	.id = snd_soc_dapm_switch, .name = wname, .reg = wreg, .shift = wshift,\
	.invert = winvert, .kcontrols = NULL, .num_kcontrols = 0}
/*
 *****************************************************************************
 * Function Prototype
 *****************************************************************************
 */
static int aic3262_set_bias_level(struct snd_soc_codec *codec,
						enum snd_soc_bias_level level);

static int __new_control_info(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_info *uinfo);

static int __new_control_get(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol);

static int __new_control_put(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol);

static inline int aic3262_get_divs(int mclk, int rate);

static int aic3262_multi_i2s_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *dai);

static int aic3262_multi_i2s_set_dai_sysclk(struct snd_soc_dai *codec_dai,
		int clk_id, unsigned int freq, int dir);
static int aic3262_multi_i2s_set_dai_pll(struct snd_soc_dai *codec_dai,
		int pll_id, int source, unsigned int freq_in,
		unsigned int freq_out);

static int aic3262_multi_i2s_asi1_set_dai_fmt(struct snd_soc_dai *codec_dai,
		unsigned int fmt);

static int aic3262_multi_i2s_asi2_set_dai_fmt(struct snd_soc_dai *codec_dai,
		unsigned int fmt);

static int aic3262_multi_i2s_asi3_set_dai_fmt(struct snd_soc_dai *codec_dai,
		unsigned int fmt);

static int aic3262_multi_i2s_asi1_mute(struct snd_soc_dai *dai, int mute);

static int aic3262_multi_i2s_asi2_mute(struct snd_soc_dai *dai, int mute);

static int aic3262_multi_i2s_asi3_mute(struct snd_soc_dai *dai, int mute);

#if 0
static const char *wclk1_pincontrol[] = {
	"ASI1 Word Clock Input/Output", "CLKOUT output"};
static const char *dout1_pincontrol[] = {
	"disabled", "ASI1 data output", "gpio", "clock out",
	"INT1", "INT2", "SAR ADC interrupt"};

static const char *din1_pincontrol[] = {"disabled", "enabled"};

static const char *wclk2_pincontrol[] = {
	"diabled", "ASI1 secondary wclk", "general purpose input",
	"general purpose output", "clkout", "INT1 interrupt",
	"IN2 interrupt", "output digital microphone",
	"SAR ADC interrupt", "data output for ASI1"};

static const char *bclk2_pincontrol[] = {
	"diabled", "ASI1 secondary wclk", "general purpose input",
	"general purpose output", "clkout", "INT1 interrupt",
	"IN2 interrupt", "output digital microphone",
	"SAR ADC interrupt", "data output for ASI1"};

static const char *dout2_pincontrol[] = {
	"disabled", "ASI2 Data Output", "General Purpose Output",
	"INT1 Interrupt", "INT2 Interrupt", "SAR ADC interrupt",
	"Output for digital microphone", "Data Output for ASI1"};

static const char *din2_pincontrol[] = {"disabled", "enabled"};

static const char *wclk3_pincontrol[] = {
	"Disabled", "ASI3 WCLK", "General Purpose Input",
	"General Purpose output", "Data Output for ASI1"};

static const char *bclk3_pincontrol[] = {
	"Disabled", "ASI3 BCLK", "General Purpose Input",
	"General Purpose output", "Data Output for ASI1"};

static const char *dout3_pincontrol[] = {
	"disabled", "ASI3 data ooutput", "General Purpose Output",
	"ASI1 Word Clock Output", "Data Output for ASI1"};

static const char *din3_pincontrol[] = {"disabled", "enabled"};

static const char *clkin[] = {
	"mclk1", "bclk1", "gpio1", "pll_clk", "bclk2", "gpi1",
	"hf_ref_clk", "hf_osc_clk", "mclk2", "gpio2", "gpi2"};

#endif
#ifdef DAC_INDEPENDENT_VOL
/*
 *----------------------------------------------------------------------------
 * Function : n_control_info
 * Purpose  : This function is to initialize data for new control required to
 *            program the AIC3262 registers.
 *
 *----------------------------------------------------------------------------
 */
static int n_control_info(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_info *uinfo)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	int max = mc->max;
	unsigned int shift = mc->shift;
	unsigned int rshift = mc->rshift;

	if (max == 1)
		uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
	else
		uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;

	uinfo->count = shift == rshift ? 1 : 2;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = max;
	return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : n_control_get
 * Purpose  : This function is to read data of new control for
 *            program the AIC3262 registers.
 *
 *----------------------------------------------------------------------------
 */
static int n_control_get(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	u32 val;
	unsigned short mask, shift;
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	if (!strcmp(kcontrol->id.name, "Left DAC Volume")) {
		mask = AIC3262_8BITS_MASK;
		shift = 0;
		val = snd_soc_read(codec, mc->reg);
		ucontrol->value.integer.value[0] =
		    (val <= 48) ? (val + 127) : (val - 129);
	}
	if (!strcmp(kcontrol->id.name, "Right DAC Volume")) {
		mask = AIC3262_8BITS_MASK;
		shift = 0;
		val = snd_soc_read(codec, mc->reg);
		ucontrol->value.integer.value[0] =
		    (val <= 48) ? (val + 127) : (val - 129);
	}

	return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : __new_control_put
 * Purpose  : new_control_put is called to pass data from user/application to
 *            the driver.
 *
 *----------------------------------------------------------------------------
 */
static int n_control_put(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	u8 val, val_mask;
	int reg, err;
	unsigned int invert = mc->invert;
	int max = mc->max;
	DBG("n_control_put\n");
	reg = mc->reg;
	val = ucontrol->value.integer.value[0];
	if (invert)
		val = max - val;
	if (!strcmp(kcontrol->id.name, "Left DAC Volume")) {
		DBG("LDAC\n");
		val = (val >= 127) ? (val - 127) : (val + 129);
		val_mask = AIC3262_8BITS_MASK;
	}
	if (!strcmp(kcontrol->id.name, "Right DAC Volume")) {
		DBG("RDAC\n");
		val = (val >= 127) ? (val - 127) : (val + 129);
		val_mask = AIC3262_8BITS_MASK;
	}

	err = snd_soc_update_bits_locked(codec, reg, val_mask, val);
	if (err < 0) {
		printk(KERN_ERR "Error while updating bits\n");
		return err;
	}

	return 0;
}
#endif /*#ifdef DAC_INDEPENDENT_VOL*/
/*
 *------------------------------------------------------------------------------
 * snd_soc_info_volsw_2r_n - double mixer info callback
 * @kcontrol: mixer control
 * @uinfo: control element information
 *
 * Callback to provide information about a double mixer control that
 * spans 2 codec registers.
 *
 * Returns 0 for success.
 *------------------------------------------------------------------------------
 */
int snd_soc_info_volsw_2r_n(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_info *uinfo)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	int max = mc->max;

	if (max == 1)
		uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
	else
		uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;

	uinfo->count = 2;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = max;
	return 0;
}

/*
 *------------------------------------------------------------------------------
 * snd_soc_get_volsw_2r_n - double mixer get callback
 * @kcontrol: mixer control
 * @ucontrol: control element information
 *
 * Callback to get the value of a double mixer control that spans 2 registers.
 *
 * Returns 0 for success.
 *------------------------------------------------------------------------------
 */
int snd_soc_get_volsw_2r_n(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	unsigned int reg = mc->reg;
	unsigned int reg2 = mc->rreg;
	unsigned int shift = mc->shift;
	int max = mc->max;
	unsigned int mask;
	unsigned int invert = mc->invert;
	unsigned short val, val2;

	if (!strcmp(kcontrol->id.name, "PCM Playback Volume")) {
		mask = AIC3262_8BITS_MASK;
		shift = 0;
	} else if (!strcmp(kcontrol->id.name, "HP Driver Gain")) {
		mask = 0x3F;
		shift = 0;
	} else if (!strcmp(kcontrol->id.name, "PGA Capture Volume")) {
		mask = 0x7F;
		shift = 0;
	} else if (!strcmp(kcontrol->id.name, "REC Driver Volume")) {
		mask = 0x3F;
		shift = 0;
	} else if (!strcmp(kcontrol->id.name, "LO to HP Volume")) {
		mask = 0x7F;
		shift = 0;
	} else if (!strcmp(kcontrol->id.name, "MA Volume")) {
		mask = 0x7F;
		shift = 0;
	} else {
		printk(KERN_ERR "Invalid kcontrol name\n");
		return -1;
	}

	/* Read, update the corresponding Registers */
	val = (snd_soc_read(codec, reg) >> shift) & mask;
	val2 = (snd_soc_read(codec, reg2) >> shift) & mask;

	if (!strcmp(kcontrol->id.name, "PCM Playback Volume")) {
		ucontrol->value.integer.value[0] =
		    (val <= 48) ? (val + 127) : (val - 129);
		ucontrol->value.integer.value[1] =
		    (val2 <= 48) ? (val2 + 127) : (val2 - 129);
	} else if (!strcmp(kcontrol->id.name, "HP Driver Gain")) {
		ucontrol->value.integer.value[0] =
		    (val >= 57) ? (val - 57) : (val + 7);
		ucontrol->value.integer.value[1] =
		    (val2 >= 57) ? (val2 - 57) : (val2 + 7);
	} else if (!strcmp(kcontrol->id.name, "PGA Capture Volume")) {
		ucontrol->value.integer.value[0] =
		    (val <= 40) ? (val + 24) : (val - 104);
		ucontrol->value.integer.value[1] =
		    (val2 <= 40) ? (val2 + 24) : (val2 - 104);
	} else if (!strcmp(kcontrol->id.name, "REC Driver Volume")) {
		ucontrol->value.integer.value[0] = ((val >= 0) & (val <= 29)) ?
						 (val + 7) : (val - 57);
		ucontrol->value.integer.value[1] = ((val2 >= 0) &
		    (val2 <= 29)) ? (val2 + 7) : (val2 - 57);

	} else if (!strcmp(kcontrol->id.name, "LO to HP Volume")) {
		ucontrol->value.integer.value[0] = ((val >= 0) & (val <= 116)) ?
				 (val + 1) : ((val == 127) ? (0) : (117));
		ucontrol->value.integer.value[1] = ((val2 >= 0) & (val2 <= 116))
				 ? (val2 + 1) : ((val2 == 127) ? (0) : (117));

	} else if (!strcmp(kcontrol->id.name, "MA Volume")) {
		ucontrol->value.integer.value[0] = (val <= 40) ?
					 (41 - val) : (val = 0);
		ucontrol->value.integer.value[1] = (val2 <= 40) ?
					 (41 - val2) : (val2 = 0);
	}

	if (invert) {
		ucontrol->value.integer.value[0] =
			max - ucontrol->value.integer.value[0];
		ucontrol->value.integer.value[1] =
			max - ucontrol->value.integer.value[1];
	}

	return 0;
}
/*
*-------------------------------------------------------------------------------
* snd_soc_put_volsw_2r_n - double mixer set callback
* @kcontrol: mixer control
* @ucontrol: control element information
*
* Callback to set the value of a double mixer control that spans 2 registers.
*
* Returns 0 for success.
*-------------------------------------------------------------------------------
*/
int snd_soc_put_volsw_2r_n(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	unsigned int reg = mc->reg;
	unsigned int reg2 = mc->rreg;
	unsigned int shift = mc->shift;
	int max = mc->max;
	unsigned int mask;
	unsigned int invert = mc->invert;
	int err;
	unsigned short val, val2, val_mask;

	mask = 0x00FF;

	val = (ucontrol->value.integer.value[0] & mask);
	val2 = (ucontrol->value.integer.value[1] & mask);
	if (invert) {
		val = max - val;
		val2 = max - val2;
	}

	/* Check for the string name of the kcontrol */
	if (!strcmp(kcontrol->id.name, "PCM Playback Volume")) {
		val = (val >= 127) ? (val - 127) : (val + 129);
		val2 = (val2 >= 127) ? (val2 - 127) : (val2 + 129);
		val_mask = AIC3262_8BITS_MASK;	/* 8 bits */
	} else if ((!strcmp(kcontrol->id.name, "HP Driver Gain")) ||
		   (!strcmp(kcontrol->id.name, "LO Driver Gain"))) {
		val = (val <= 6) ? (val + 57) : (val - 7);
		val2 = (val2 <= 6) ? (val2 + 57) : (val2 - 7);
		val_mask = 0x3F;	/* 6 bits */
		DBG("val=%d, val2=%d", val, val2);
	} else if (!strcmp(kcontrol->id.name, "PGA Capture Volume")) {
		val = (val >= 24) ? ((val <= 64) ?
			(val-24) : (40)) : (val + 104);
		val2 = (val2 >= 24) ?
			((val2 <= 64) ? (val2 - 24) : (40)) : (val2 + 104);
		val_mask = 0x7F;	/* 7 bits */
	} else if (!strcmp(kcontrol->id.name, "LO to REC Volume")) {

		val = (val <= 116) ?
			 (val % 116) : ((val == 117) ? (127) : (117));
		val2 = (val2 <= 116) ?
			(val2 % 116) : ((val2 == 117) ? (127) : (117));
		val_mask = 0x7F;
	} else if (!strcmp(kcontrol->id.name, "REC Driver Volume")) {

		val = (val <= 7) ? (val + 57) : ((val < 36) ? (val - 7) : (29));
		val2 = (val2 <= 7) ?
				(val2 + 57) : ((val2 < 36) ? (val2 - 7) : (29));
		val_mask = 0x3F;
	} else if (!strcmp(kcontrol->id.name, "LO to HP Volume")) {

		val = ((val > 0) & (val <= 117)) ?
			 (val - 1) : ((val == 0) ? (127) : (116));
		val2 = ((val2 > 0) & (val2 <= 117)) ?
			 (val2 - 1) : ((val2 == 0) ? (127) : (116));
		val_mask = 0x7F;
	} else if (!strcmp(kcontrol->id.name, "MA Volume")) {

		val = ((val <= 41) & (val > 0)) ?
			 (41 - val) : ((val > 41) ? (val = 41) : (63));
		val2 = ((val2 <= 41) & (val2 > 0)) ?
			 (41 - val2) : ((val2 > 41) ? (val2 = 41) : (63));
		val_mask = 0x7F;
	} else {
		printk(KERN_ERR "Invalid control name\n");
		return -1;
	}

	val = val << shift;
	val2 = val2 << shift;

	err = snd_soc_update_bits_locked(codec, reg, val_mask, val);
	if (err < 0)
		return err;

	err = snd_soc_update_bits_locked(codec, reg2, val_mask, val2);
	return err;
}

static int __new_control_info(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 65535;

	return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : __new_control_get
 * Purpose  : This function is to read data of new control for
 *            program the AIC3262 registers.
 *
 *----------------------------------------------------------------------------
 */
static int __new_control_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	u32 val;
	val = snd_soc_read(codec, aic3262_reg_ctl);
	ucontrol->value.integer.value[0] = val;
	return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : __new_control_put
 * Purpose  : new_control_put is called to pass data from user/application to
 *            the driver.
 *
 *----------------------------------------------------------------------------
 */
static int __new_control_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct aic3262_priv *aic3262 = snd_soc_codec_get_drvdata(codec);
	u8 data[2];
	int ret = 0;

	u32 data_from_user = ucontrol->value.integer.value[0];

	aic3262_change_book(codec, 0);
	aic3262_reg_ctl = data[0] = (u8) ((data_from_user & 0xFF00) >> 8);
	data[1] = (u8) ((data_from_user & 0x00FF));

	if (!data[0])
		aic3262->page_no = data[1];

	DBG("reg = %d val = %x\n", data[0], data[1]);
#if defined(LOCAL_REG_ACCESS)
	if (codec->hw_write(codec->control_data, data, 2) != 2)
		ret = -EIO;
#else
	ret = snd_soc_write(codec, data[0], data[1]);
#endif
	if (ret)
		printk(KERN_ERR "Error in i2c write\n");

	return ret;
}


/*
 *****************************************************************************
 * Structure Initialization
 *****************************************************************************
 */
static const DECLARE_TLV_DB_SCALE(dac_vol_tlv, -6350, 50, 0);
static const DECLARE_TLV_DB_SCALE(adc_vol_tlv, -1200, 50, 0);
static const DECLARE_TLV_DB_SCALE(spk_gain_tlv, 600, 600, 0);
static const DECLARE_TLV_DB_SCALE(output_gain_tlv, -600, 100, 0);
static const DECLARE_TLV_DB_SCALE(micpga_gain_tlv, 0, 50, 0);
static const DECLARE_TLV_DB_SCALE(adc_fine_gain_tlv, -40, 10, 0);
static const DECLARE_TLV_DB_SCALE(beep_gen_volume_tlv, -6300, 100, 0);

/* Chip-level Input and Output CM Mode Controls */
static const char *input_common_mode_text[] = {
	"0.9v", "0.75v" };

static const char *output_common_mode_text[] = {
	"Input CM", "1.25v", "1.5v", "1.65v" };

static const struct soc_enum input_cm_mode =
	SOC_ENUM_SINGLE(CM_REG, 2, 2, input_common_mode_text);

static const struct soc_enum output_cm_mode =
	SOC_ENUM_SINGLE(CM_REG, 0, 4, output_common_mode_text);

/*
 *****************************************************************************
 * Structure Initialization
 *****************************************************************************
 */
static const struct snd_kcontrol_new aic3262_snd_controls[] = {
	/* Output */
#ifndef DAC_INDEPENDENT_VOL
	/* sound new kcontrol for PCM Playback volume control */

	SOC_DOUBLE_R_SX_TLV("PCM Playback Volume",
			DAC_LVOL, DAC_RVOL, 8,0xffffff81, 0x30, dac_vol_tlv),
#endif
	/*HP Driver Gain Control*/
	SOC_DOUBLE_R_SX_TLV("HeadPhone Driver Amplifier Volume",
			HPL_VOL, HPR_VOL, 6, 0xfffffffa, 0xe, output_gain_tlv),

	/*LO Driver Gain Control*/
	SOC_DOUBLE_TLV("Speaker Amplifier Volume",
				SPK_AMP_CNTL_R4, 4, 0, 5, 0, spk_gain_tlv),

	SOC_DOUBLE_R_SX_TLV("Receiver Amplifier Volume",
	REC_AMP_CNTL_R5, RAMPR_VOL, 6, 0xfffffffa, 0x1d, output_gain_tlv),

	SOC_DOUBLE_R_SX_TLV("PCM Capture Volume",
		LADC_VOL, RADC_VOL, 7,0xffffff68, 0x24, adc_vol_tlv),


	SOC_DOUBLE_R_TLV ("MicPGA Volume Control",
		MICL_PGA, MICR_PGA, 0, 0x5F, 0, micpga_gain_tlv),
	SOC_DOUBLE_TLV("PCM Capture Fine Gain Volume",
		ADC_FINE_GAIN, 4, 0, 5, 1, adc_fine_gain_tlv),

	SOC_DOUBLE("ADC channel mute", ADC_FINE_GAIN, 7, 3, 1, 0),

	SOC_DOUBLE("DAC MUTE", DAC_MVOL_CONF, 2, 3, 1, 1),

	/* sound new kcontrol for Programming the registers from user space */
	SOC_SINGLE_AIC3262("Program Registers"),

	SOC_SINGLE("RESET", RESET_REG, 0,1,0),

	SOC_SINGLE("DAC VOL SOFT STEPPING", DAC_MVOL_CONF, 0, 2, 0),

#ifdef DAC_INDEPENDENT_VOL
	/*SOC_SINGLE_N("Left DAC Volume", DAC_LVOL, 0, 0xAF, 0),
	SOC_SINGLE_N("Right DAC Volume", DAC_RVOL, 0, 0xAF, 0),*/
#endif

	SOC_SINGLE("DAC AUTO MUTE CONTROL", DAC_MVOL_CONF, 4, 7, 0),
	SOC_SINGLE("RIGHT MODULATOR SETUP", DAC_MVOL_CONF, 7, 1, 0),

	SOC_SINGLE("ADC Volume soft stepping", ADC_CHANNEL_POW, 0, 3, 0),

	SOC_DOUBLE_R("MICPGA enable/disable",MICL_PGA,MICR_PGA,7, 1, 0),

	SOC_SINGLE("Mic Bias ext independent enable", MIC_BIAS_CNTL, 7, 1, 0),
	SOC_SINGLE("MICBIAS_EXT ON", MIC_BIAS_CNTL, 6, 1, 0),
	SOC_SINGLE("MICBIAS EXT Power Level", MIC_BIAS_CNTL, 4, 3, 0),

	SOC_SINGLE("MICBIAS_INT ON", MIC_BIAS_CNTL, 2, 1, 0),
	SOC_SINGLE("MICBIAS INT Power Level", MIC_BIAS_CNTL, 0, 3, 0),

	SOC_DOUBLE("DRC_EN_CTL", DRC_CNTL_R1, 6, 5, 1, 0),
	SOC_SINGLE("DRC_THRESHOLD_LEVEL", DRC_CNTL_R1, 2, 7, 1),
	SOC_SINGLE("DRC_HYSTERISIS_LEVEL", DRC_CNTL_R1, 0, 7, 0),

	SOC_SINGLE("DRC_HOLD_LEVEL", DRC_CNTL_R2, 3, 0x0F, 0),
	SOC_SINGLE("DRC_GAIN_RATE", DRC_CNTL_R2, 0, 4, 0),
	SOC_SINGLE("DRC_ATTACK_RATE", DRC_CNTL_R3, 4, 0x0F, 1),
	SOC_SINGLE("DRC_DECAY_RATE", DRC_CNTL_R3, 0, 0x0F, 1),

	SOC_SINGLE("BEEP_GEN_EN", BEEP_CNTL_R1, 7, 1, 0),
	SOC_DOUBLE_R("BEEP_VOL_CNTL", BEEP_CNTL_R1, BEEP_CNTL_R2, 0, 0x0F, 1),
	SOC_SINGLE("BEEP_MAS_VOL", BEEP_CNTL_R2, 6, 3, 0),

	SOC_DOUBLE_R("AGC_EN", LAGC_CNTL, RAGC_CNTL, 7, 1, 0),
	SOC_DOUBLE_R("AGC_TARGET_LEVEL", LAGC_CNTL, RAGC_CNTL, 4, 7, 1),

	SOC_DOUBLE_R("AGC_GAIN_HYSTERESIS", LAGC_CNTL, RAGC_CNTL, 0, 3, 0),
	SOC_DOUBLE_R("AGC_HYSTERESIS", LAGC_CNTL_R2, RAGC_CNTL_R2, 6, 3, 0),
	SOC_DOUBLE_R("AGC_NOISE_THRESHOLD", LAGC_CNTL_R2,
						RAGC_CNTL_R2, 1, 31, 1),

	SOC_DOUBLE_R("AGC_MAX_GAIN", LAGC_CNTL_R3, RAGC_CNTL_R3, 0, 116, 0),
	SOC_DOUBLE_R("AGC_ATCK_TIME", LAGC_CNTL_R4, RAGC_CNTL_R4, 3, 31, 0),
	SOC_DOUBLE_R("AGC_ATCK_SCALE_FACTOR",
				LAGC_CNTL_R4, RAGC_CNTL_R4, 0, 7, 0),

	SOC_DOUBLE_R("AGC_DECAY_TIME", LAGC_CNTL_R5, RAGC_CNTL_R5, 3, 31, 0),
	SOC_DOUBLE_R("AGC_DECAY_SCALE_FACTOR",
				LAGC_CNTL_R5, RAGC_CNTL_R5, 0, 7, 0),
	SOC_DOUBLE_R("AGC_NOISE_DEB_TIME", LAGC_CNTL_R6,
						RAGC_CNTL_R6, 0, 31, 0),

	SOC_DOUBLE_R("AGC_SGL_DEB_TIME", LAGC_CNTL_R7,
					RAGC_CNTL_R7, 0, 0x0F, 0),

	SOC_SINGLE("DAC PRB Selection",DAC_PRB, 0, 25, 0),
	SOC_SINGLE("HP_DEPOP", HP_DEPOP, 0, 255,0),
	SOC_DOUBLE("IN1 LO BYPASS VOLUME" , LINE_AMP_CNTL_R2, 3, 0, 3, 1),
	SOC_ENUM("Input CM mode", input_cm_mode),
	SOC_ENUM("Output CM mode", output_cm_mode),
};


/* the sturcture contains the different values for mclk */
static const struct aic3262_rate_divs aic3262_divs[] = {
/*
 * mclk, rate, p_val, pll_j, pll_d, dosr, ndac, mdac, aosr, nadc, madc, blck_N,
 * codec_speficic_initializations
 */
	/* 8k rate */
#ifdef CONFIG_MINI_DSP
	{12000000, 8000, 1, 8, 1920, 768, 8, 2, 128, 8, 12, 4,
		{{0, 60, 0}, {0, 61, 0} } },
#else
	{12000000, 8000, 1, 8, 1920, 128, 12, 8, 128, 8, 6, 4,
		{{0, 60, 1}, {0, 61, 1} } },
	{12288000, 8000, 1, 1, 3333, 128, 12, 8, 128, 8, 6, 4,
		{{0, 60, 1}, {0, 61, 1} } },
	{24000000, 8000, 1, 4, 96, 128, 12, 8, 128, 12, 8, 4,
		{{0, 60, 1}, {0, 61, 1} } },
#endif
	/* 11.025k rate */
	{12000000, 11025, 1, 1, 8816, 1024, 8, 2, 128, 8, 2, 48,
		{{0, 60, 1}, {0, 61, 1} } },
	{12288000, 11025, 1, 1, 8375, 1024, 8, 2, 128, 8, 2, 48,
		{{0, 60, 1}, {0, 61, 1} } },
	{24000000, 11025, 1, 3, 7632, 128, 8, 8, 128, 8, 8, 4,
		{{0, 60, 1}, {0, 61, 1} } },

	/* 16k rate */
#ifdef CONFIG_MINI_DSP
	{12000000, 16000, 1, 8, 1920, 384, 4, 4, 128, 4, 12, 12,
		{{0, 60, 0}, {0, 61, 0} } },
	{12288000, 16000, 1, 9, 0, 216, 2, 16, 72, 2, 48, 27,
		{{0, 60, 0}, {0, 61, 0} } },
#else
	{12000000, 16000, 1, 8, 1920, 128, 8, 6, 128, 8, 6, 4,
		{{0, 60, 1}, {0, 61, 1} } },
	{12288000, 16000, 1, 2, 6667, 128, 8, 6, 128, 8, 6, 4,
		{{0, 60, 1}, {0, 61, 1} } },
	{24000000, 16000, 1, 4, 96, 128, 8, 6, 128, 8, 6, 4,
		{{0, 60, 1}, {0, 61, 1} } },
#endif
	/* 22.05k rate */
	{12000000, 22050, 1, 3, 7632, 128, 8, 2, 128, 8, 2, 4,
		{{0, 60, 1}, {0, 61, 1} } },
	{12288000, 22050, 1, 3, 675, 128, 8, 2, 128, 8, 2, 4,
		{{0, 60, 1}, {0, 61, 1} } },
	{24000000, 22050, 1, 3, 7632, 128, 8, 3, 128, 8, 3, 4,
		{{0, 60, 1}, {0, 61, 1} } },
	/* 32k rate */
	{12000000, 32000, 1, 5, 4613, 128, 8, 2, 128, 8, 2, 4,
		{{0, 60, 1}, {0, 61, 1} } },
	{12288000, 32000, 1, 5, 3333, 128, 8, 2, 128, 8, 2, 4,
		{{0, 60, 1}, {0, 61, 1} } },
	{24000000, 32000, 1, 4, 96, 128, 6, 4, 128, 6, 4, 4,
		{{0, 60, 1}, {0, 61, 1} } },

#ifdef CONFIG_MINI_DSP
	{12000000, 44100, 1, 7, 5264, 128, 2, 8, 128, 2, 8, 4,
		{{0, 60, 0}, {0, 61, 0} } },
	{12288000, 44100, 1, 7, 3548, 128, 2, 8, 128, 8, 2, 4,
		{{0, 60, 0}, {0, 61, 0} } },
#else
	/* 44.1k rate */
	{12000000, 44100, 1, 7, 5264, 128, 8, 2, 128, 8, 2, 4,
		{{0, 60, 1}, {0, 61, 1} } },
	{12288000, 44100, 1, 7, 3548, 128, 8, 2, 128, 8, 2, 4,
		{{0, 60, 1}, {0, 61, 1} } },
	{24000000, 44100, 1, 3, 7632, 128, 4, 4, 64, 4, 4, 4,
		{{0, 60, 1}, {0, 61, 1} } },
#endif

#ifdef CONFIG_MINI_DSP
	{12288000, 48000, 1, 8, 52, 128, 2, 8, 128, 2, 8, 4,
		{{0, 60, 0}, {0, 61, 0} } },
	{12000000, 48000, 1, 8, 1920, 128, 2, 8, 128, 2, 8, 4,
         {{0, 60, 0}, {0, 61, 0}}},
#else
	/* 48k rate */
	{12000000, 48000, 1, 8, 1920, 128, 8, 2, 128, 8, 2, 4,
		{{0, 60, 1}, {0, 61, 1} } },
	{12288000, 48000, 1, 8, 52, 128, 8, 2, 128, 8, 2, 4,
		{{0, 60, 1}, {0, 61, 1} } },
	{24000000, 48000, 1, 4, 960, 128, 4, 4, 128, 4, 4, 4,
		{{0, 60, 1}, {0, 61, 1} } },
#endif

	/*96k rate */
	{12000000, 96000, 1, 16, 3840, 128, 8, 2, 128, 8, 2 , 4,
		{{0, 60, 7}, {0, 61, 7} } },
	{24000000, 96000, 1, 4, 960, 128, 4, 2, 128, 4, 2, 2,
		{{0, 60, 7}, {0, 61, 7} } },
	/*192k */
	{12000000, 192000, 1, 32, 7680, 128, 8, 2, 128, 8, 2, 4,
		{{0, 60, 17}, {0, 61, 13} } },
	{24000000, 192000, 1, 4, 960, 128, 2, 2, 128, 2, 2, 4,
		{{0, 60, 17}, {0, 61, 13} } },
};



/*
*----------------------------------------------------------------------------
* Function : aic3262_multi_i2s_dump_regs
* Purpose  : This function is to mute or unmute the left and right DAC
*
*----------------------------------------------------------------------------
*/
static void aic3262_multi_i2s_dump_regs(struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct aic3262_priv *aic3262 = snd_soc_codec_get_drvdata(codec);
	unsigned int counter;

	DBG(KERN_INFO "#%s: Dai Active %d ASI%d REGS DUMP\n",
		__func__, aic3262->active_count, dai->id);

	aic3262_change_page(codec, 0);
	aic3262_change_book(codec, 0);

	DBG(KERN_INFO "#Page0 REGS..\n");
	for (counter = 0; counter < 85; counter++) {
		DBG(KERN_INFO "#%2d -> 0x%x\n", counter,
			snd_soc_read(codec, counter));
	}

	DBG(KERN_INFO "#Page1 REGS..\n");
	for (counter = 128; counter < 176; counter++) {
		DBG(KERN_INFO "#%2d -> 0x%x\n", (counter % 128),
			snd_soc_read(codec, counter));
	}

	DBG(KERN_INFO "#Page4 REGS..\n");
	for (counter = 512; counter < 631; counter++) {
		DBG(KERN_INFO "#%2d -> 0x%x\n",
			(counter % 128), snd_soc_read(codec, counter));
	}

	for (counter = 0; counter < MAX_ASI_COUNT; counter++) {
		DBG(KERN_INFO "#ASI%d Frame %s @ %dHz Playback %d Record %d\n",
		(counter + 1),
		(aic3262->asiCtxt[counter].master == 1) ? "Master" : "Slave",
		aic3262->asiCtxt[counter].sampling_rate,
		aic3262->asiCtxt[counter].playback_mode,
		aic3262->asiCtxt[counter].capture_mode);
		DBG(KERN_INFO "#DAC Option [%d,%d] ADC Option %d WLEN %d\n\n",
		aic3262->asiCtxt[counter].left_dac_output,
		aic3262->asiCtxt[counter].right_dac_output,
		aic3262->asiCtxt[counter].adc_input,
		aic3262->asiCtxt[counter].word_len);
	}
	return;
}

/*
 *----------------------------------------------------------------------------
 * Function : aic3262_multi_i2s_mute
 * Purpose  : This function is to mute or unmute the left and right DAC
 *
 *----------------------------------------------------------------------------
 */
static int aic3262_multi_i2s_mute(struct snd_soc_dai *dai, int mute)
{
	struct snd_soc_codec *codec = dai->codec;
	struct aic3262_priv *aic3262 = snd_soc_codec_get_drvdata(codec);

	DBG(KERN_INFO "#%s : mute entered with %d\n", __func__, mute);

	/* If we are doing both Recording and Playback on this DAI interface,
	* do not MUTE the Codec.
	*/
	if (mute && (aic3262->asiCtxt[dai->id - 1].asi_active > 1)) {
		DBG("#%s Cannot Mute the ASI%d Now..\n",
			__func__, dai->id);
	} else {
		switch (dai->id) {
		case 1:
			aic3262_multi_i2s_asi1_mute(dai, mute);
		break;
		case 2:
			aic3262_multi_i2s_asi2_mute(dai, mute);
		break;
		case 3:
			aic3262_multi_i2s_asi3_mute(dai, mute);
		break;
		default:
			printk(KERN_ERR "#%s: Invalid DAI id\n", __func__);
			return -EINVAL;
		}
	}
	DBG(KERN_INFO "#%s : mute ended\n", __func__);
	return 0;
}


/*
*----------------------------------------------------------------------------
* Function : aic3262_multi_i2s_asi1_mute
* Purpose  : This function is to mute or unmute the left and right DAC
*
*----------------------------------------------------------------------------
*/
static int aic3262_multi_i2s_asi1_mute(struct snd_soc_dai *dai, int mute)
{
	struct snd_soc_codec *codec = dai->codec;
	struct aic3262_priv *aic3262 = snd_soc_codec_get_drvdata(codec);

	DBG(KERN_INFO "#%s : mute %d started\n", __func__, mute);

	if (mute && !aic3262->asiCtxt[0].port_muted ) {
		DBG(KERN_INFO "Mute if part\n");


	snd_soc_update_bits(codec, DAC_MVOL_CONF, DAC_LR_MUTE_MASK,DAC_LR_MUTE);

		/* First check if both Playback and Recording is going on
		* this interface.
		*/
		if (aic3262->asiCtxt[0].asi_active > 1) {
			DBG("#%s Cannot Mute the ASI Now..\n", __func__);
		} else if (!(aic3262->asiCtxt[1].playback_mode) &&
			!(aic3262->asiCtxt[2].playback_mode)) {
			/* Before Muting, please check if any other
			* ASI is active. if so, we cannot simply mute the
			* DAC and ADC Registers.
			*/
		DBG("#%s None of the ASI's are active now..\n", __func__);
			snd_soc_write(codec, DAC_MVOL_CONF,
				((aic3262->dac_reg & 0xF3) | 0x0C));
			snd_soc_write(codec, ADC_FINE_GAIN,
				((aic3262->adc_gain & 0x77) | 0x88));
			snd_soc_write(codec, HPL_VOL, 0xB9);
			snd_soc_write(codec, HPR_VOL, 0xB9);
			snd_soc_write(codec, REC_AMP_CNTL_R5, 0x39);
			snd_soc_write(codec, RAMPR_VOL, 0x39);
			snd_soc_write(codec, SPK_AMP_CNTL_R4, 0x00);
			aic3262->asiCtxt[0].port_muted = 1;
		}
	} else {
		DBG(KERN_INFO "Mute else part\n");
		snd_soc_update_bits(codec, DAC_MVOL_CONF,
						DAC_LR_MUTE_MASK, 0x0);
		snd_soc_write(codec, ADC_FINE_GAIN,(0X00 & 0x77) | 0x0);
		aic3262_multi_i2s_dump_regs(dai);
	}

	DBG(KERN_INFO "#%s : mute %d ended\n", __func__, mute);

	return 0;
}

/*
*----------------------------------------------------------------------------
* Function : aic3262_multi_i2s_asi2_maic3262_asi3_clk_configute
* Purpose : This function is to mute or unmute the left and right DAC
*
*----------------------------------------------------------------------------
*/
static int aic3262_multi_i2s_asi2_mute(struct snd_soc_dai *dai, int mute)
{
	struct snd_soc_codec *codec = dai->codec;
	struct aic3262_priv *aic3262 = snd_soc_codec_get_drvdata(codec);

	DBG(KERN_INFO "#%s : mute %d started\n", __func__, mute);

	if (mute && !aic3262->asiCtxt[1].port_muted ) {
		DBG(KERN_INFO "Mute if part\n");
	snd_soc_update_bits(codec, DAC_MVOL_CONF, DAC_LR_MUTE_MASK,DAC_LR_MUTE);

		/* First check if both Playback and Recording is going on
		* this interface.
		*/
		if (aic3262->asiCtxt[1].asi_active > 1) {
			DBG("#%s Cannot Mute the ASI Now..\n", __func__);
		} else if (!(aic3262->asiCtxt[0].playback_mode) &&
			!(aic3262->asiCtxt[2].playback_mode)) {
			/* Before Muting, please check if any other
			* ASI is active. if so, we cannot simply mute the
			* DAC and ADC Registers.
			*/
			snd_soc_write(codec, DAC_MVOL_CONF,
				((aic3262->dac_reg & 0xF3) | 0x0C));
			snd_soc_write(codec, ADC_FINE_GAIN,
				((aic3262->adc_gain & 0x77) | 0x88));
			snd_soc_write(codec, HPL_VOL, 0xB9);
			snd_soc_write(codec, HPR_VOL, 0xB9);
			snd_soc_write(codec, REC_AMP_CNTL_R5, 0x39);
			snd_soc_write(codec, RAMPR_VOL, 0x39);
			snd_soc_write(codec, SPK_AMP_CNTL_R4, 0x00);
			aic3262->asiCtxt[1].port_muted = 1;
		}
	} else {
		DBG(KERN_INFO "Mute else part\n");
		snd_soc_update_bits(codec, DAC_MVOL_CONF,
						DAC_LR_MUTE_MASK, 0x0);

		/*aic3262_multi_i2s_dump_regs(dai);*/
	}

	DBG(KERN_INFO "#%s : mute %d ended\n", __func__, mute);

	return 0;
}

/*
*----------------------------------------------------------------------------
* Function : aic3262_multi_i2s_asi3_mute
* Purpose : This function is to mute or unmute the left and right DAC
*
*----------------------------------------------------------------------------
*/
static int aic3262_multi_i2s_asi3_mute(struct snd_soc_dai *dai, int mute)
{
	struct snd_soc_codec *codec = dai->codec;
	struct aic3262_priv *aic3262 = snd_soc_codec_get_drvdata(codec);

	DBG(KERN_INFO "#%s : mute %d started\n", __func__, mute);

	if (mute && !aic3262->asiCtxt[2].port_muted) {
		DBG("Mute if part\n");
	snd_soc_update_bits(codec, DAC_MVOL_CONF, DAC_LR_MUTE_MASK,DAC_LR_MUTE);

		/* First check if both Playback and Recording is going on
		* this interface.
		*/
		if (aic3262->asiCtxt[2].asi_active > 1) {
			DBG("#%s Cannot Mute the ASI Now..\n", __func__);
		} else if (!(aic3262->asiCtxt[0].playback_mode) &&
			!(aic3262->asiCtxt[1].playback_mode)) {
			/* Before Muting, please check if any other
			* ASI is active. if so, we cannot simply mute the
			* DAC and ADC Registers.
			*/
			snd_soc_write(codec, DAC_MVOL_CONF,
				((aic3262->dac_reg & 0xF3) | 0x0C));
			snd_soc_write(codec, ADC_FINE_GAIN,
				((aic3262->adc_gain & 0x77) | 0x88));
			snd_soc_write(codec, HPL_VOL, 0xB9);
			snd_soc_write(codec, HPR_VOL, 0xB9);
			snd_soc_write(codec, REC_AMP_CNTL_R5, 0x39);
			snd_soc_write(codec, RAMPR_VOL, 0x39);
			snd_soc_write(codec, SPK_AMP_CNTL_R4, 0x00);
			aic3262->asiCtxt[2].port_muted = 1;
		}
	} else {
		DBG("Mute else part\n");
		snd_soc_update_bits(codec, DAC_MVOL_CONF,
						DAC_LR_MUTE_MASK, 0x0);

		/*aic3262_multi_i2s_dump_regs(dai);*/

	}

	DBG(KERN_INFO "#%s : mute %d ended\n", __func__, mute);

	return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : aic3262_multi_i2s_set_dai_fmt
 * Purpose  : This function is to set the DAI format
 *
 *----------------------------------------------------------------------------
 */
static int aic3262_multi_i2s_set_dai_fmt(struct snd_soc_dai *codec_dai,
		unsigned int fmt)
{
	/* Check the DAI Id and based on that switch the configuration for
	* the Individual ASI Port.
	*/
	switch (codec_dai->id) {
	case 1:
		aic3262_multi_i2s_asi1_set_dai_fmt(codec_dai, fmt);
	break;
	case 2:
		aic3262_multi_i2s_asi2_set_dai_fmt(codec_dai, fmt);
	break;
	case 3:
		aic3262_multi_i2s_asi3_set_dai_fmt(codec_dai, fmt);
	break;
	default:
		printk(KERN_ERR
			"#%s: Invalid DAI interface format\n", __func__);
		return -EINVAL;
	}
	return 0;
}



/*
*----------------------------------------------------------------------------
* Function : aic3262_multi_i2s_asi1_set_dai_fmt
* Purpose  : This function is to set the DAI format for ASI1 Port
*
*----------------------------------------------------------------------------
*/
static int aic3262_multi_i2s_asi1_set_dai_fmt(struct snd_soc_dai *codec_dai,
		unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct aic3262_priv *aic3262 = snd_soc_codec_get_drvdata(codec);
	u8 iface_reg, clk_reg;
	u8 regvalue;

	DBG(KERN_INFO "%s: DAI_ID %d fmt %d\n",
		__func__, codec_dai->id, fmt);

	/* Read the B0_P4_R4 and B0_P4_R10 Registers to configure the
	* ASI1 Bus and Clock Formats depending on the PCM Format.
	*/
	iface_reg = snd_soc_read(codec, ASI1_BUS_FMT);
	clk_reg   = snd_soc_read(codec, ASI1_BWCLK_CNTL_REG);

	/* set master/slave audio interface */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		DBG(KERN_INFO "#%s: Configuring ASI%d as Frame Master..\n",
			__func__, codec_dai->id);
		aic3262->asiCtxt[0].master = 1;
		clk_reg |= (BIT5 | BIT2);	/* Codec Interface as Master */
	break;
	case SND_SOC_DAIFMT_CBS_CFS:
		DBG(KERN_INFO "#%s: Configuring ASI%d as Frame Slave..\n",
			__func__, codec_dai->id);
		clk_reg &= ~0xFC; /* Reset bits D[7:5] and D[4:2] to zero */
		aic3262->asiCtxt[0].master = 0;
	break;
	case SND_SOC_DAIFMT_CBS_CFM:
		/* new case..just for debugging */
		DBG(KERN_INFO "%s: SND_SOC_DAIFMT_CBS_CFM\n", __func__);
		aic3262->asiCtxt[0].master = 0;
		clk_reg |= BIT5;	/* Only WCLK1 Output from Codec */
		clk_reg &= ~0x1C;	/* BCLK1 Input to Codec */
	break;
	default:
		printk(KERN_ERR "#%s: Invalid DAI master/slave interface\n",
			__func__);
		return -EINVAL;
	}
	aic3262->asiCtxt[0].pcm_format = (fmt & SND_SOC_DAIFMT_FORMAT_MASK);
	/* interface format */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		DBG(KERN_INFO "#%s: Configuring ASI%d for I2s Mode..\n",
			__func__, codec_dai->id);
		iface_reg = (iface_reg & 0x1f);
	break;
	case SND_SOC_DAIFMT_DSP_A:
		DBG(KERN_INFO "#%s: Configuring ASI%d for DSP_A Mode..\n",
			__func__, codec_dai->id);
		iface_reg = (iface_reg & 0x1f) | 0x20;
	break;
	case SND_SOC_DAIFMT_RIGHT_J:
		iface_reg = (iface_reg & 0x1f) | 0x40;
	break;
	case SND_SOC_DAIFMT_LEFT_J:
		iface_reg = (iface_reg & 0x1f) | 0x60;
	break;
	case SND_SOC_DAIFMT_DSP_B:
		DBG(KERN_INFO "#%s: Configuring ASI%d for DSP_B Mode..\n",
			__func__, codec_dai->id);
		iface_reg = (iface_reg & 0x1f) | 0x80;
		/* voice call need data offset in 1 bitclock */
		snd_soc_write(codec, ASI1_LCH_OFFSET, 1);
	break;
	default:
		printk(KERN_ERR
			"#%s: Invalid DAI interface format\n", __func__);
		return -EINVAL;
	}
	/* Also Configure the Pin Control Registers before writing into
	* the ASI specific Clock Control and Format Registers
	*/

	/* Configure B0_P4_R65_D[5:2] to 001 This configures the
	* WCLK1 Pin to ASI1
	*/
	regvalue = snd_soc_read(codec, WCLK1_PIN_CNTL_REG);
	snd_soc_write(codec, WCLK1_PIN_CNTL_REG, (regvalue | BIT2));

	/* Configure B0_P4_R68_d[6:5] = 01 and B0_P4_R67_D[4:1] to 0001
	* to ensure that the DIN1 and DOUT1 Pins are configured
	* correctly
	*/
	regvalue = snd_soc_read(codec, DIN1_PIN_CNTL_REG);
	snd_soc_write(codec, DIN1_PIN_CNTL_REG, (regvalue | BIT5));
	regvalue = snd_soc_read(codec, DOUT1_PIN_CNTL_REG);
	snd_soc_write(codec, DOUT1_PIN_CNTL_REG, (regvalue | BIT1));

	snd_soc_write(codec, ASI1_BWCLK_CNTL_REG, clk_reg);

	snd_soc_write(codec, ASI1_BUS_FMT, iface_reg);

	return 0;
}


/*
*----------------------------------------------------------------------------
* Function : aic3262_multi_i2s_asi2_set_dai_fmt
* Purpose  : This function is to set the DAI format for ASI2 Port
*
*----------------------------------------------------------------------------
*/
static int aic3262_multi_i2s_asi2_set_dai_fmt(struct snd_soc_dai *codec_dai,
			unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct aic3262_priv *aic3262 = snd_soc_codec_get_drvdata(codec);
	u8 iface_reg, clk_reg;
	u8 regvalue;

	DBG(KERN_INFO "%s: DAI_ID %d fmt %d\n",
		__func__, codec_dai->id, fmt);

	/* Read the B0_P4_R17 and B0_P4_R26 Registers to configure the
	* ASI1 Bus and Clock Formats depending on the PCM Format.
	*/
	iface_reg = snd_soc_read(codec, ASI2_BUS_FMT);
	clk_reg   = snd_soc_read(codec, ASI2_BWCLK_CNTL_REG);

	/* set master/slave audio interface */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		DBG(KERN_INFO "#%s: Configuring ASI%d as Frame Master..\n",
			__func__, codec_dai->id);
		aic3262->asiCtxt[1].master = 1;
		clk_reg |= (BIT5 | BIT2);
	break;
	case SND_SOC_DAIFMT_CBS_CFS:
		DBG(KERN_INFO "#%s: Configuring ASI%d as Frame Slave..\n",
			__func__, codec_dai->id);

		clk_reg &= ~0xFC;
		aic3262->asiCtxt[1].master = 0;
	break;
	case SND_SOC_DAIFMT_CBS_CFM:
		/*new case..just for debugging */
		DBG(KERN_INFO "%s: SND_SOC_DAIFMT_CBS_CFM\n", __func__);
		aic3262->asiCtxt[1].master = 0;
		clk_reg |= BIT5;
		clk_reg &= ~0x1C;
	break;
	default:
		printk(KERN_ERR "#%s:Invalid DAI master/slave interface\n",
			__func__);
		return -EINVAL;
	}
	aic3262->asiCtxt[1].pcm_format = (fmt & SND_SOC_DAIFMT_FORMAT_MASK);
	/* interface format */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		DBG(KERN_INFO "#%s: Configuring ASI%d for I2S Mode..\n",
				__func__, codec_dai->id);
		iface_reg = (iface_reg & 0x1f);
	break;
	case SND_SOC_DAIFMT_DSP_A:
		DBG(KERN_INFO "#%s: Configuring ASI%d for DSP_A Mode..\n",
			__func__, codec_dai->id);
		iface_reg = (iface_reg & 0x1f) | 0x20;
	break;
	case SND_SOC_DAIFMT_RIGHT_J:
		iface_reg = (iface_reg & 0x1f) | 0x40;
	break;
	case SND_SOC_DAIFMT_LEFT_J:
		iface_reg = (iface_reg & 0x1f) | 0x60;
	break;
	case SND_SOC_DAIFMT_DSP_B:
		DBG(KERN_INFO "#%s: Configuring ASI%d for DSP Mode..\n",
			__func__, codec_dai->id);
		iface_reg = (iface_reg & 0x1f) | 0x80;
		/* voice call need data offset in 1 bitclock */
		snd_soc_write(codec, ASI2_LCH_OFFSET, 1);
	break;
	default:
		printk(KERN_ERR "#%s:Invalid DAI interface format\n", __func__);
		return -EINVAL;
	}

	/* Also Configure the Pin Control Registers before writing into
	* the ASI2 specific Clock Control and Format Registers
	*/

	/* Configure B0_P4_R69_D[5:2] to 001 This configures the
	* WCLK2 Pin to ASI2
	*/

	regvalue = snd_soc_read(codec, WCLK2_PIN_CNTL_REG);
	snd_soc_write(codec, WCLK2_PIN_CNTL_REG, (regvalue | BIT2));

	regvalue = snd_soc_read(codec, BCLK2_PIN_CNTL_REG);
	snd_soc_write(codec, BCLK2_PIN_CNTL_REG, (regvalue | BIT2));

	/* Configure B0_P4_R72_d[6:5] = 01 and B0_P4_R71_D[4:1] to 0001
	 * to ensure that the DIN2 and DOUT2 Pins are configured
	 * correctly
	 */
	regvalue = snd_soc_read(codec, DIN2_PIN_CNTL_REG);
	snd_soc_write(codec, DIN2_PIN_CNTL_REG, (regvalue | BIT5));

	regvalue = snd_soc_read(codec, DOUT2_PIN_CNTL_REG);
	snd_soc_write(codec, DOUT2_PIN_CNTL_REG, (regvalue | BIT5 | BIT1));

	snd_soc_write(codec, ASI2_BWCLK_CNTL_REG, clk_reg);

	snd_soc_write(codec, ASI2_BUS_FMT, iface_reg);

	return 0;
}

/*
*----------------------------------------------------------------------------
* Function : aic3262_multi_i2s_asi3_set_dai_fmt
* Purpose  : This function is to set the DAI format for ASI3 Port
*
*----------------------------------------------------------------------------
*/
static int aic3262_multi_i2s_asi3_set_dai_fmt(struct snd_soc_dai *codec_dai,
			unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct aic3262_priv *aic3262 = snd_soc_codec_get_drvdata(codec);
	u8 iface_reg, clk_reg;
	u8 regvalue;

	DBG(KERN_INFO "%s: DAI_ID %d fmt %d\n",
		__func__, codec_dai->id, fmt);

	/* Read the B0_P4_R33 and B0_P4_R42 Registers to configure the
	* ASI1 Bus and Clock Formats depending on the PCM Format.
	*/
	iface_reg = snd_soc_read(codec, ASI3_BUS_FMT);
	clk_reg   = snd_soc_read(codec, ASI3_BWCLK_CNTL_REG);

	/* set master/slave audio interface */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		DBG(KERN_INFO "#%s: Configuring ASI%d as Frame Master..\n",
			__func__, codec_dai->id);
		aic3262->asiCtxt[2].master = 1;
		clk_reg |= (BIT5 | BIT2);
	break;
	case SND_SOC_DAIFMT_CBS_CFS:
		DBG(KERN_INFO "#%s: Configuring ASI%d as Frame Slave..\n",
			__func__, codec_dai->id);
		clk_reg &= ~0xFC;
		aic3262->asiCtxt[2].master = 0;
	break;
	case SND_SOC_DAIFMT_CBS_CFM:
		/* new case..just for debugging */
		DBG(KERN_INFO "%s: SND_SOC_DAIFMT_CBS_CFM\n", __func__);
		aic3262->asiCtxt[2].master = 0;
		clk_reg |= BIT5;
		clk_reg &= ~0x1C;
	break;
	default:
		printk(KERN_ERR "Invalid DAI master/slave interface\n");
		return -EINVAL;
	}
	aic3262->asiCtxt[2].pcm_format = (fmt & SND_SOC_DAIFMT_FORMAT_MASK);
	/* interface format */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		DBG(KERN_INFO "#%s: Configuring ASI%d for I2S Mode..\n",
			__func__, codec_dai->id);
		iface_reg = (iface_reg & 0x1f);
	break;
	case SND_SOC_DAIFMT_DSP_A:
		DBG(KERN_INFO "#%s: Configuring ASI%d for DSP_A Mode..\n",
			__func__, codec_dai->id);
		iface_reg = (iface_reg & 0x1f) | 0x20;
	break;
	case SND_SOC_DAIFMT_RIGHT_J:
		iface_reg = (iface_reg & 0x1f) | 0x40;
	break;
	case SND_SOC_DAIFMT_LEFT_J:
		iface_reg = (iface_reg & 0x1f) | 0x60;
	break;
	case SND_SOC_DAIFMT_DSP_B:
		DBG(KERN_INFO "#%s: Configuring ASI%d for DSP Mode..\n",
			__func__, codec_dai->id);
		iface_reg = (iface_reg & 0x1f) | 0x80;
		/* voice call need data offset in 1 bitclock */
		snd_soc_write(codec, ASI3_LCH_OFFSET, 1);
	break;
	default:
		printk(KERN_ERR
			"#%s: Invalid DAI interface format\n", __func__);
		return -EINVAL;
	}

	/* Also Configure the Pin Control Registers before writing into
	* the ASI specific Clock Control and Format Registers
	*/
	/* Configure B0_P4_R73_D[5:2] to 0001 This configures the
	* WCLK1 Pin to ASI1
	*/
	regvalue = snd_soc_read(codec, WCLK3_PIN_CNTL_REG);
	snd_soc_write(codec, WCLK3_PIN_CNTL_REG, (regvalue | BIT2));

	regvalue = snd_soc_read(codec, BCLK3_PIN_CNTL_REG);
	snd_soc_write(codec, BCLK3_PIN_CNTL_REG, (regvalue | BIT2));

	/* Configure B0_P4_R76_d[6:5] = 01 and B0_P4_R75_D[4:1] to 0001
	* to ensure that the DIN1 and DOUT1 Pins are configured
	* correctly
	*/
	regvalue = snd_soc_read(codec, DIN3_PIN_CNTL_REG);
	snd_soc_write(codec, DIN3_PIN_CNTL_REG, (regvalue | BIT5));
	regvalue = snd_soc_read(codec, DOUT3_PIN_CNTL_REG);
	snd_soc_write(codec, DOUT3_PIN_CNTL_REG, (regvalue | BIT1));

	snd_soc_write(codec, ASI3_BWCLK_CNTL_REG, clk_reg);

	snd_soc_write(codec, ASI3_BUS_FMT, iface_reg);

	return 0;
}

/*
 * Clock after PLL and dividers
 */
static int aic3262_multi_i2s_set_dai_sysclk(struct snd_soc_dai *codec_dai,
		int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct aic3262_priv *aic3262 = snd_soc_codec_get_drvdata(codec);

	DBG(KERN_INFO "#%s: DAI ID %d Freq %d Direction %d\n",
			__func__, codec_dai->id, freq, dir);
	switch (freq) {
	case AIC3262_FREQ_12000000:
	case AIC3262_FREQ_12288000:
	case AIC3262_FREQ_24000000:
		aic3262->sysclk = freq;
		return 0;
		break;
	}
	printk(KERN_ERR "Invalid frequency to set DAI system clock\n");
	return -EINVAL;
}

/*
* aic3262_multi_i2s_set_pll
*
* This function is invoked as part of the PLL call-back
* handler from the ALSA layer.
*/
static int aic3262_multi_i2s_set_dai_pll(struct snd_soc_dai *codec_dai,
		int pll_id, int source, unsigned int freq_in,
		unsigned int freq_out)
{

	printk(KERN_INFO "%s: DAI ID %d PLL_ID %d InFreq %d OutFreq %d\n",
		__func__, pll_id, codec_dai->id, freq_in, freq_out);

	return 0;
}

/*
* aic3262_asi1_clk_config
*
* This function is used to configure the BCLK1, WCLK1 pins which
* are specific to ASI1 Interface. This function just enables the
* BCLk and WCLK  along with the miniDSP Port Control Registers.
* However, depending on the user requirement, this function can also be
* extended to configure the sourc for the BCLK and WCLK on a ASI basis.
*/
static int aic3262_asi1_clk_config(struct snd_soc_codec *codec,
		struct snd_pcm_hw_params *params)
{
	u8 bclk_N_value, wclk_N_value;
	u8 minidspD_data, minidspA_data;
	u8 regval;

	struct aic3262_priv *aic3262 = snd_soc_codec_get_drvdata(codec);

	DBG(KERN_INFO "%s: Invoked\n",  __func__);

	/* Configure the BCLK and WCLK Output Mux Options */
	regval = snd_soc_read(codec, ASI1_BWCLK_OUT_CNTL);
	regval &= ~(AIC3262_ASI_BCLK_MUX_MASK | AIC3262_ASI_WCLK_MUX_MASK);

	regval |= (aic3262->asiCtxt[0].bclk_output <<
			AIC3262_ASI_BCLK_MUX_SHIFT);
	regval |= aic3262->asiCtxt[0].wclk_output;
	snd_soc_write(codec, ASI1_BWCLK_OUT_CNTL, regval);

	/* Configure the corresponding miniDSP Data Ports */
	minidspD_data = snd_soc_read(codec, MINIDSP_PORT_CNTL_REG);
	minidspD_data &= ~(BIT5 | BIT4);
	snd_soc_write(codec, MINIDSP_PORT_CNTL_REG, minidspD_data);

	minidspA_data = snd_soc_read(codec, ASI1_ADC_INPUT_CNTL);
	minidspA_data &= ~(BIT2 | BIT1 | BIT0);
	minidspA_data |= aic3262->asiCtxt[0].adc_input;
	snd_soc_write(codec, ASI1_ADC_INPUT_CNTL, minidspA_data);


	if (aic3262->asiCtxt[0].master == 1) {
		DBG(KERN_INFO
		"#%s: Codec Master on ASI1 Port. Enabling BCLK WCLK Divider.\n",
			__func__);
		bclk_N_value = aic3262->asiCtxt[0].bclk_div;
		snd_soc_write(codec, ASI1_BCLK_N, (bclk_N_value | 0x80));

		wclk_N_value = snd_soc_read(codec, ASI1_WCLK_N);
		snd_soc_write(codec, ASI1_WCLK_N, (wclk_N_value | 0xA0));
	}
	return 0;

}

/*
* aic3262_asi2_clk_config
*
* This function is used to configure the BCLK2, WCLK2 pins which
* are specific to ASI2 Interface. This function just enables the
* BCLk and WCLK  along with the miniDSP Port Control Registers.
* However, depending on the user requirement, this function can also be
* extended to configure the sourc for the BCLK and WCLK on a ASI basis.
*/
static int aic3262_asi2_clk_config(struct snd_soc_codec *codec,
		struct snd_pcm_hw_params *params)
{
	u8 bclk_N_value, wclk_N_value, minidspD_data, minidspA_data;
	u8 regval;

	struct aic3262_priv *aic3262 = snd_soc_codec_get_drvdata(codec);

	DBG(KERN_INFO "%s: Invoked\n",  __func__);


	/* Configure the BCLK and WCLK Output Mux Options */
	regval = snd_soc_read(codec, ASI2_BWCLK_OUT_CNTL);
	regval &= ~(AIC3262_ASI_BCLK_MUX_MASK | AIC3262_ASI_WCLK_MUX_MASK);
	regval |= (aic3262->asiCtxt[1].bclk_output <<
			AIC3262_ASI_BCLK_MUX_SHIFT);
	regval |= aic3262->asiCtxt[1].wclk_output;

	snd_soc_write(codec, ASI2_BWCLK_OUT_CNTL, regval);
	/* Configure the corresponding miniDSP Data Ports */
	minidspD_data = snd_soc_read(codec, MINIDSP_PORT_CNTL_REG);
	minidspD_data |= (BIT2);
	snd_soc_write(codec, MINIDSP_PORT_CNTL_REG, minidspD_data);

	minidspA_data = snd_soc_read(codec, ASI2_ADC_INPUT_CNTL);
	minidspA_data &= ~(BIT2 | BIT1 | BIT0);
	minidspA_data |= aic3262->asiCtxt[1].adc_input;
	snd_soc_write(codec, ASI2_ADC_INPUT_CNTL, minidspA_data);

	/* NO Manual configuration of WCLK and BCLK for Master Mode.
	* DAPM Handles all the required modifications.
	*/
	if (aic3262->asiCtxt[1].master == 1) {
		DBG(KERN_INFO
		"#%s: Codec Master on ASI2 Port. Enabling BCLK WCLK Divider.\n",
			__func__);
		bclk_N_value = aic3262->asiCtxt[1].bclk_div;
		snd_soc_write(codec, ASI2_BCLK_N, (bclk_N_value | 0x80));

		wclk_N_value = snd_soc_read(codec, ASI2_WCLK_N);
		snd_soc_write(codec, ASI2_WCLK_N, (wclk_N_value | 0xA0));
	}

	return 0;

}

/*
* aic3262_asi3_clk_config
*
* This function is used to configure the BCLK3, WCLK3 pins which
* are specific to ASI3 Interface. This function just enables the
* BCLk and WCLK  along with the miniDSP Port Control Registers.
* However, depending on the user requirement, this function can also be
* extended to configure the sourc for the BCLK and WCLK on a ASI basis.
*/
static int aic3262_asi3_clk_config(struct snd_soc_codec *codec,
		struct snd_pcm_hw_params *params)
{
	u8 bclk_N_value, wclk_N_value, minidspD_data, minidspA_data;
	u8 regval;

	struct aic3262_priv *aic3262 = snd_soc_codec_get_drvdata(codec);

	DBG(KERN_INFO "%s:\n",  __func__);


	/* Configure the BCLK and WCLK Output Mux Options */
	regval = snd_soc_read(codec, ASI3_BWCLK_OUT_CNTL);
	regval &= ~(AIC3262_ASI_BCLK_MUX_MASK | AIC3262_ASI_WCLK_MUX_MASK);
	regval |= (aic3262->asiCtxt[2].bclk_output <<
			AIC3262_ASI_BCLK_MUX_SHIFT);
	regval |= aic3262->asiCtxt[2].wclk_output;
	snd_soc_write(codec, ASI3_BWCLK_OUT_CNTL, regval);

	minidspD_data = snd_soc_read(codec, MINIDSP_PORT_CNTL_REG);
	minidspD_data |= (BIT1);
	snd_soc_write(codec, MINIDSP_PORT_CNTL_REG, minidspD_data);

	minidspA_data = snd_soc_read(codec, ASI3_ADC_INPUT_CNTL);
	minidspA_data &= ~(BIT2 | BIT1 | BIT0);
	minidspA_data |= aic3262->asiCtxt[2].adc_input;
	snd_soc_write(codec, ASI3_ADC_INPUT_CNTL, minidspA_data);

		if (aic3262->asiCtxt[2].master == 1) {
		DBG(KERN_INFO
		"#%s: Codec Master on ASI3 Port. Enabling BCLK WCLK Divider.\n",
			__func__);
		bclk_N_value = aic3262->asiCtxt[2].bclk_div;
		snd_soc_write(codec, ASI2_BCLK_N, (bclk_N_value | 0x80));

		wclk_N_value = snd_soc_read(codec, ASI3_WCLK_N);
		snd_soc_write(codec, ASI3_WCLK_N, (wclk_N_value | 0xA0));
	}
	return 0;

}

/*
* aic3262_multi_i2s_hw_params
*
* This function is used to configure the individual ASI port registers
* depending on the configuration passed on by the snd_pcm_hw_params
* structure.
* This function internally configures the ASI specific pins and clock
* Control Registers.
*/
static int aic3262_multi_i2s_hw_params(struct snd_pcm_substream *substream,
			struct snd_pcm_hw_params *params,
			struct snd_soc_dai *dai)
{
	int i, j;
	u8 data;
	u16 regoffset = 0;
	u8 dacpath = 0;
	u8 adcpath = 0;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;
	struct aic3262_priv *aic3262 = snd_soc_codec_get_drvdata(codec);

	DBG(KERN_INFO "#%s: Invoked for ASI%d Port for %s Mode\n",
		__func__, dai->id,
		(substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		? "Playback" : "Record");

	i = aic3262_get_divs(aic3262->sysclk, params_rate(params));

	i2c_verify_book0(codec);

	if (i < 0) {
		printk(KERN_ERR "#%s: Sampling rate %d not supported\n",
			__func__, params_rate(params));
		return i;
	}

	aic3262_set_bias_level(codec, SND_SOC_BIAS_STANDBY);

	/* Configure the PLL J, R D values only if none of the ASI
	* Interfaces are Active.
	*/

	if (1) {
		DBG(KERN_INFO "#%s: None of the ASIs active yet...\n",
			__func__);
		/*We will fix R value to 1 and make P & J=K.D as variable */
		/* Setting P & R values are set to 1 and 1 at init*/

		/* J value */
		snd_soc_write(codec, PLL_J_REG, aic3262_divs[i].pll_j);

		/* MSB & LSB for D value */

		snd_soc_write(codec, PLL_D_MSB, (aic3262_divs[i].pll_d >> 8));
		snd_soc_write(codec, PLL_D_LSB,
			      (aic3262_divs[i].pll_d & AIC3262_8BITS_MASK));

		/* NDAC divider value */
		data = snd_soc_read(codec, NDAC_DIV_POW_REG);
		DBG(KERN_INFO "# reading NDAC = %d , NDAC_DIV_POW_REG = %x\n",
			aic3262_divs[i].ndac, data);
		snd_soc_write(codec, NDAC_DIV_POW_REG,
			 ((data & 0x80)|(aic3262_divs[i].ndac)));
		DBG(KERN_INFO "# writing NDAC = %d , NDAC_DIV_POW_REG = %x\n",
			aic3262_divs[i].ndac,
			((data & 0x80)|(aic3262_divs[i].ndac)));

		/* MDAC divider value */
		data = snd_soc_read(codec, MDAC_DIV_POW_REG);
		DBG(KERN_INFO "# reading MDAC = %d , MDAC_DIV_POW_REG = %x\n",
			aic3262_divs[i].mdac, data);
		snd_soc_write(codec, MDAC_DIV_POW_REG,
			((data & 0x80)|(aic3262_divs[i].mdac)));
		DBG(KERN_INFO "# writing MDAC = %d , MDAC_DIV_POW_REG = %x\n",
		aic3262_divs[i].mdac, ((data & 0x80)|(aic3262_divs[i].mdac)));

		/* DOSR MSB & LSB values */
		snd_soc_write(codec, DOSR_MSB_REG, aic3262_divs[i].dosr >> 8);
		DBG(KERN_INFO "# writing DOSR_MSB_REG = %d\n",
			(aic3262_divs[i].dosr >> 8));
		snd_soc_write(codec, DOSR_LSB_REG,
			aic3262_divs[i].dosr & AIC3262_8BITS_MASK);
		DBG(KERN_INFO "# writing DOSR_LSB_REG = %d\n",
			(aic3262_divs[i].dosr & AIC3262_8BITS_MASK));

		/* NADC divider value */
		data = snd_soc_read(codec, NADC_DIV_POW_REG);
		snd_soc_write(codec, NADC_DIV_POW_REG,
			((data & 0x80)|(aic3262_divs[i].nadc)));
		DBG(KERN_INFO "# writing NADC_DIV_POW_REG = %d\n",
			aic3262_divs[i].nadc);

		/* MADC divider value */
		data = snd_soc_read(codec, MADC_DIV_POW_REG);
		snd_soc_write(codec, MADC_DIV_POW_REG,
			((data & 0x80)|(aic3262_divs[i].madc)));
		DBG(KERN_INFO "# writing MADC_DIV_POW_REG = %d\n",
			aic3262_divs[i].madc);

		/* AOSR value */
		snd_soc_write(codec, AOSR_REG, aic3262_divs[i].aosr);
		DBG(KERN_INFO "# writing AOSR = %d\n", aic3262_divs[i].aosr);
	} else {
		DBG(KERN_INFO "#Atleast 1 ASI Active. Cannot Program PLL..\n");
	}
	/* Check for the DAI ID to know which ASI needs
	* Configuration.
	*/
	switch (dai->id) {
	case 1:
		regoffset = ASI1_BUS_FMT;

		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			DBG(KERN_INFO "#%s: ASI1 DAC Inputs enabled..\n",
					__func__);
			/* Read the DAC Control Register and configure it
			* as per the ASIContext Structure Settings.
			*/
			dacpath = snd_soc_read(codec, ASI1_DAC_OUT_CNTL);
			dacpath &= ~(AIC3262_ASI_LDAC_PATH_MASK |
				AIC3262_ASI_RDAC_PATH_MASK);
			dacpath |= (aic3262->asiCtxt[0].left_dac_output
				<< AIC3262_ASI_LDAC_PATH_SHIFT);

			dacpath |= (aic3262->asiCtxt[0].right_dac_output
				<< AIC3262_ASI_RDAC_PATH_SHIFT);
			snd_soc_write(codec, ASI1_DAC_OUT_CNTL, dacpath);

			aic3262->asiCtxt[0].playback_mode = 1;
			aic3262->asiCtxt[0].bclk_div =
				aic3262_divs[i].blck_N;
		} else {
			/* For Recording, Configure the DOUT Pin as per
			 * ASIContext Structure Settings.
			 */
			adcpath = snd_soc_read(codec, ASI1_DATA_OUT);
			adcpath &= ~(AIC3262_ASI_DOUT_MASK);

			adcpath |= aic3262->asiCtxt[0].dout_option;
			snd_soc_write(codec, ASI1_DATA_OUT, adcpath);

			aic3262->asiCtxt[0].capture_mode = 1;
		}
	break;
	case 2:
		regoffset = ASI2_BUS_FMT;

		/* Since we are configuring ASI2, please check if Playback
		 * is expected. If so, enable ASI2 Inputs to Left and
		 * Right DACs
		 */
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			DBG(KERN_INFO "#%s: ASI2 DAC Inputs enabled..\n",
					__func__);
			/* Read the DAC Control Register and configure it
			 * as per theASIContext Structure Settings.
			 */
			dacpath = snd_soc_read(codec, ASI2_DAC_OUT_CNTL);
			dacpath &= ~(AIC3262_ASI_LDAC_PATH_MASK |
				AIC3262_ASI_RDAC_PATH_MASK);
			dacpath |= (aic3262->asiCtxt[1].left_dac_output
				<< AIC3262_ASI_LDAC_PATH_SHIFT);

			dacpath |= (aic3262->asiCtxt[1].right_dac_output
				<< AIC3262_ASI_RDAC_PATH_SHIFT);
			snd_soc_write(codec, ASI2_DAC_OUT_CNTL, dacpath);
			aic3262->asiCtxt[1].playback_mode = 1;

			aic3262->asiCtxt[1].bclk_div =
				aic3262_divs[i].blck_N;
		} else {
			/* For Recording, Configure the DOUT Pin as per
			 * ASIContext Structure Settings.
			 */
			adcpath = snd_soc_read(codec, ASI2_DATA_OUT);
			adcpath &= ~(AIC3262_ASI_DOUT_MASK);
			adcpath |= aic3262->asiCtxt[1].dout_option;
			snd_soc_write(codec, ASI2_DATA_OUT, adcpath);

			aic3262->asiCtxt[1].capture_mode = 1;
		}
	break;
	case 3:
		regoffset = ASI3_BUS_FMT;
		/* Since we are configuring ASI3, please check if Playback
		* is expected. If so, enable ASI3 Inputs to Left and
		* Right DACs
		*/
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			DBG(KERN_INFO "#%s:ASI3 DAC Inputs enabled.\n",
				__func__);
			/* Read the DAC Control Register and configure
			* it as per the ASIContext Structure Settings.
			*/
			dacpath = snd_soc_read(codec, ASI3_DAC_OUT_CNTL);
			dacpath &= ~(AIC3262_ASI_LDAC_PATH_MASK |
					AIC3262_ASI_RDAC_PATH_MASK);
			dacpath |= (aic3262->asiCtxt[2].left_dac_output
					<< AIC3262_ASI_LDAC_PATH_SHIFT);
			dacpath |= (aic3262->asiCtxt[2].right_dac_output
				<< AIC3262_ASI_RDAC_PATH_SHIFT);
			snd_soc_write(codec,
				ASI3_DAC_OUT_CNTL, dacpath);

			aic3262->asiCtxt[2].playback_mode = 1;

			aic3262->asiCtxt[2].bclk_div =
				aic3262_divs[i].blck_N;
		} else {
			/* For Recording, Configure the DOUT Pin as per
			 * ASIContext Structure Settings.
			 */
			adcpath &= ~(AIC3262_ASI_DOUT_MASK);
			adcpath |= aic3262->asiCtxt[2].dout_option;
			snd_soc_write(codec, ASI3_DATA_OUT, adcpath);

			aic3262->asiCtxt[2].capture_mode = 1;
		}
	break;
	default:
		printk(KERN_ERR "Invalid Dai ID %d in %s",
			dai->id, __func__);
	break;
	}
	DBG(KERN_INFO "#%s: Reading Pg %d Reg %d for Bus Format Control.\n",
		__func__, (regoffset/128),  (regoffset % 128));

	/* Read the correspondig ASI DAI Interface Register */
	data = snd_soc_read(codec, regoffset);

	data = data & 0xe7;

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		DBG(KERN_INFO "#%s: Configuring ASI%d S16_LE Fmt..\n",
			__func__, dai->id);
		data = data | 0x00;
		aic3262->asiCtxt[dai->id - 1].word_len = 16;
	break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		data |= (0x08);
		aic3262->asiCtxt[dai->id - 1].word_len = 20;
	break;
	case SNDRV_PCM_FORMAT_S24_LE:
		DBG(KERN_INFO "#%s: Configuring ASI%d S24_LE Fmt..\n",
			__func__, dai->id);
		data |= (0x10);
		aic3262->asiCtxt[dai->id - 1].word_len = 24;
	break;
	case SNDRV_PCM_FORMAT_S32_LE:
		DBG(KERN_INFO "#%s: Configuring ASI%d S32_LE Fmt..\n",
			__func__, dai->id);
		data |= (0x18);
		aic3262->asiCtxt[dai->id - 1].word_len = 32;
	break;
	}

	/* configure the respective Registers for the above configuration */
	snd_soc_write(codec, regoffset, data);

	for (j = 0; j < NO_FEATURE_REGS; j++) {
		snd_soc_write(codec,
			aic3262_divs[i].codec_specific_regs[j].reg_offset,
			aic3262_divs[i].codec_specific_regs[j].reg_val);
	}

	/* Enable the PLL, MDAC, NDAC, NADC, MADC and BCLK Dividers */
	aic3262_set_bias_level(codec, SND_SOC_BIAS_ON);

	/* Based on the DAI ID we enable the corresponding pins related to the
	* ASI Port.
	*/
	switch (dai->id) {
	case 1:
		aic3262_asi1_clk_config(codec, params);
	break;
	case 2:
		aic3262_asi2_clk_config(codec, params);
	break;
	case 3:
		aic3262_asi3_clk_config(codec, params);
	break;
	default:
		printk(KERN_ERR "Invalid Dai ID %d in %s",
			dai->id, __func__);
	break;
	}
	/* Depending on the DAI->ID update the local Flags */
	aic3262->asiCtxt[dai->id - 1].asi_active++;
	aic3262->asiCtxt[dai->id - 1].sampling_rate = params_rate(params);
	/* Update the active_count flag */
	aic3262->active_count++;

	return 0;
}

/*
*
* aic3262_multi_i2s_hw_free
*
* This function is used to configure the Codec after the usage is completed.
* We can use this function to disable the DAC and ADC specific inputs from the
* individual ASI Ports of the Audio Codec.
*/
static int aic3262_multi_i2s_shutdown(struct snd_pcm_substream *substream,
			struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;
	struct aic3262_priv *aic3262 = snd_soc_codec_get_drvdata(codec);

	u8 value;
	u8 dacpath;
	u8 adcpath;
	u16 dacregoffset = 0;
	u16 adcregoffset = 0;

	DBG(KERN_INFO "#%s: ASI%d Port for %s Mode\n",
		__func__, dai->id,
		(substream->stream == SNDRV_PCM_STREAM_PLAYBACK) ?
		"Playback" : "Record");

	/* Check if this function was already executed earlier for the same
	* ASI Port
	*/
	if ((substream->stream == SNDRV_PCM_STREAM_PLAYBACK) &&
		(aic3262->asiCtxt[dai->id - 1].playback_mode == 0)) {
		DBG(KERN_INFO "#%s: Function Already Executed. Exiting..\n",
			__func__);
		goto err;
	} else if ((substream->stream != SNDRV_PCM_STREAM_PLAYBACK) &&
		(aic3262->asiCtxt[dai->id - 1].capture_mode == 0)) {
		DBG(KERN_INFO "#%s: Function Already Executed. Exiting..\n",
			__func__);
		goto err;
	}

	switch (dai->id) {
	case 1:
		/* In case we are Frame Master on this Interface, Switch off
		* the Bit Clock Divider and Word Clock Dividers
		*/
		if (aic3262->asiCtxt[0].master == 1) {
			/* Also check if either Playback or Recording is still
			* going on this ASI Interface
			*/

			value = snd_soc_read(codec, ASI1_BCLK_N);
			snd_soc_write(codec, ASI1_BCLK_N, (value & 0x7f));

			value = snd_soc_read(codec, ASI1_WCLK_N);
			snd_soc_write(codec, ASI1_WCLK_N, (value & 0x7f));
		}

		dacregoffset = ASI1_DAC_OUT_CNTL;
		adcregoffset = ASI1_ADC_INPUT_CNTL;
	break;
	case 2:
		/* In case we are Frame Master on this Interface, Switch off
		* the Bit Clock Divider and Word Clock Dividers
		*/
		if (aic3262->asiCtxt[1].master == 1) {
			value = snd_soc_read(codec, ASI2_BCLK_N);
			snd_soc_write(codec, ASI2_BCLK_N, (value & 0x7f));

			value = snd_soc_read(codec, ASI2_WCLK_N);
			snd_soc_write(codec, ASI2_WCLK_N, (value & 0x7f));
		}
		dacregoffset = ASI2_DAC_OUT_CNTL;
		adcregoffset = ASI2_ADC_INPUT_CNTL;
	break;
	case 3:
		/* In case we are Frame Master on this Interface, Switch off
		* the Bit Clock Divider and Word Clock Dividers
		*/
		if (aic3262->asiCtxt[2].master == 1) {
			value = snd_soc_read(codec, ASI3_BCLK_N);
			snd_soc_write(codec, ASI3_BCLK_N, (value & 0x7f));

			value = snd_soc_read(codec, ASI3_WCLK_N);
			snd_soc_write(codec, ASI3_WCLK_N, (value & 0x7f));
		}
		dacregoffset = ASI3_DAC_OUT_CNTL;
		adcregoffset = ASI3_ADC_INPUT_CNTL;
	break;
	default:
		printk(KERN_ERR "#%s: Invalid dai id\n", __func__);
	}
	/* If this was a Playback Stream Stop, then only
	* switch off the DAC Inputs
	*/
	if ((substream->stream == SNDRV_PCM_STREAM_PLAYBACK) &&
		(dacregoffset != 0)) {
		DBG(KERN_INFO "#%s: Disabling Pg %d Reg %d DAC Inputs ..\n",
			__func__, (dacregoffset/128), (dacregoffset % 128));

		dacpath = snd_soc_read(codec, dacregoffset);
		snd_soc_write(codec, dacregoffset, (dacpath & ~(BIT6 | BIT4)));

		aic3262->asiCtxt[dai->id - 1].playback_mode = 0;
	} else {
		/* Switch off the ADC Input Control Registers here */
		DBG(KERN_INFO "#%s: Disabling Pg %d Reg %d for ADC Inputs..\n",
			__func__, (adcregoffset/128), (adcregoffset % 128));

		adcpath =  snd_soc_read(codec, adcregoffset);
		snd_soc_write(codec, adcregoffset,
			(adcpath & ~(BIT2 | BIT1 | BIT0)));

		aic3262->asiCtxt[dai->id - 1].capture_mode = 0;
	}

	/* If we were configured in mono PCM Mode earlier, then reset the
	* Left Channel and Right Channel offset Registers here.
	*/
	switch (dai->id) {
	case 1:
		if (aic3262->asiCtxt[0].pcm_format == SND_SOC_DAIFMT_DSP_B) {
			snd_soc_write(codec, ASI1_LCH_OFFSET, 0x00);
			snd_soc_write(codec, ASI1_RCH_OFFSET, 0x00);
		}
	break;
	case 2:
		if (aic3262->asiCtxt[1].pcm_format == SND_SOC_DAIFMT_DSP_B) {
			snd_soc_write(codec, ASI2_LCH_OFFSET, 0x00);
			snd_soc_write(codec, ASI2_RCH_OFFSET, 0x00);
		}

	break;
	case 3:
		if (aic3262->asiCtxt[2].pcm_format == SND_SOC_DAIFMT_DSP_B) {
			snd_soc_write(codec, ASI3_LCH_OFFSET, 0x00);
			snd_soc_write(codec, ASI3_RCH_OFFSET, 0x00);
		}
	break;
	}
	/* Depending on the DAI->ID update the asi_active Flags */
	if (aic3262->asiCtxt[dai->id - 1].asi_active) {
		aic3262->asiCtxt[dai->id - 1].asi_active--;

		/* Update the active_count flag */
		if (aic3262->active_count)
			aic3262->active_count--;
	}
err:
	return 0;
}


/*
*
* aic3262_multi_i2s_set_clkdiv
*
*/
static int aic3262_multi_i2s_set_clkdiv(struct snd_soc_dai *codec_dai, int div_id, int div)
{
	int value;
	struct snd_soc_codec *codec = codec_dai->codec;
	struct aic3262_priv *aic3262 = snd_soc_codec_get_drvdata(codec);


	value = snd_soc_read(codec, div_id);
	snd_soc_write(codec, div_id, (value | div));

	printk(KERN_INFO "#%s: DAI ID %d Page %d Register %d Divider_Val %d Final_Value 0x%x\n",
			__func__, codec_dai->id, (div_id /128), (div_id%128), div,
			(value | div));

	/* Store the Clock Divider inside the Private Structure */
	switch(codec_dai->id) {
	case 1:
		if (div_id == ASI1_BCLK_N)
		aic3262->asiCtxt[0].bclk_div = div;
		if (div_id == ASI1_WCLK_N)
		aic3262->asiCtxt[0].wclk_div = div;
	break;
	case 2:
		if (div_id == ASI2_BCLK_N)
		aic3262->asiCtxt[1].bclk_div = div;
		if (div_id == ASI2_WCLK_N)
		aic3262->asiCtxt[1].wclk_div = div;
	break;
	case 3:
		if (div_id == ASI3_BCLK_N)
		aic3262->asiCtxt[2].bclk_div = div;
		if (div_id == ASI3_WCLK_N)
		aic3262->asiCtxt[2].wclk_div = div;
	break;
	}
	return 0;
}


/*
 *----------------------------------------------------------------------------
 * @struct  snd_soc_codec_dai |
 *          It is SoC Codec DAI structure which has DAI capabilities viz.,
 *          playback and capture, DAI runtime information viz. state of DAI
 *			and pop wait state, and DAI private data.
 *          The AIC3262 rates ranges from 8k to 192k
 *          The PCM bit format supported are 16, 20, 24 and 32 bits
 *----------------------------------------------------------------------------
 */
struct snd_soc_dai_ops aic3262_multi_i2s_dai_ops = {
	.hw_params      = aic3262_multi_i2s_hw_params,
	.digital_mute   = aic3262_multi_i2s_mute,
	.set_fmt        = aic3262_multi_i2s_set_dai_fmt,
	.set_pll        = aic3262_multi_i2s_set_dai_pll,
	.set_sysclk     = aic3262_multi_i2s_set_dai_sysclk,
	.shutdown        = aic3262_multi_i2s_shutdown,
	.set_clkdiv      = aic3262_multi_i2s_set_clkdiv,
};


static struct snd_soc_dai_driver tlv320aic3262_dai[] = {
/* AIC3262 ASI1 DAI */
{
	.name = "aic3262-asi1",
	.id = 1,
	.playback = {
		.stream_name = "ASI1 Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = AIC3262_RATES,
		.formats = AIC3262_FORMATS},
	.capture = { /* dummy for fast DAI switching */
		.stream_name = "ASI1 Capture",
		.channels_min = 1,
		.channels_max = 2,
		.rates = AIC3262_RATES,
		.formats = AIC3262_FORMATS},
	.ops = &aic3262_multi_i2s_dai_ops,
},
/* AIC3262 ASI2 DAI */
{
	.name = "aic3262-asi2",
	.id = 2,
	.playback = {
		.stream_name = "ASI2 Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = AIC3262_RATES,
		.formats = AIC3262_FORMATS,},
	.capture = {
		.stream_name = "ASI2 Capture",
		.channels_min = 1,
		.channels_max = 2,
		.rates = AIC3262_RATES,
		.formats = AIC3262_FORMATS,},
	.ops = &aic3262_multi_i2s_dai_ops,

},
/* AIC3262 ASI3 DAI  */
{
	.name = "aic3262-asi3",
	.id = 3,
	.playback = {
		.stream_name = "ASI3 Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = AIC3262_RATES,
		.formats = AIC3262_FORMATS, },
	.capture = {
		.stream_name = "ASI3 Capture",
		.channels_min = 1,
		.channels_max = 2,
		.rates = AIC3262_RATES,
		.formats = AIC3262_FORMATS, },
	.ops = &aic3262_multi_i2s_dai_ops,

},
};

/*
 *****************************************************************************
 * Initializations
 *****************************************************************************
 */
/*
 * AIC3262 register cache
 * We are caching the registers here.
 * There is no point in caching the reset register.
 *
 * NOTE: In AIC3262, there are 127 registers supported in both page0 and page1
 *       The following table contains the page0 and page 1 and page 3
 *       registers values.
 */
static const u8 aic3262_reg[AIC3262_CACHEREGNUM] = {
	0x00, 0x00, 0x10, 0x00,	/* 0 */
	0x03, 0x40, 0x11, 0x08,	/* 4 */
	0x00, 0x00, 0x00, 0x82,	/* 8 */
	0x88, 0x00, 0x80, 0x02,	/* 12 */
	0x00, 0x08, 0x01, 0x01,	/* 16 */
	0x80, 0x01, 0x00, 0x04,	/* 20 */
	0x00, 0x00, 0x01, 0x00,	/* 24 */
	0x00, 0x00, 0x01, 0x00,	/* 28 */
	0x00, 0x00, 0x00, 0x00,	/* 32 */
	0x00, 0x00, 0x00, 0x00,	/* 36 */
	0x00, 0x00, 0x00, 0x00,	/* 40 */
	0x00, 0x00, 0x00, 0x00,	/* 44 */
	0x00, 0x00, 0x00, 0x00,	/* 48 */
	0x00, 0x42, 0x02, 0x02,	/* 52 */
	0x42, 0x02, 0x02, 0x02,	/* 56 */
	0x00, 0x00, 0x00, 0x01,	/* 60 */
	0x01, 0x00, 0x14, 0x00,	/* 64 */
	0x0C, 0x00, 0x00, 0x00,	/* 68 */
	0x00, 0x00, 0x00, 0xEE,	/* 72 */
	0x10, 0xD8, 0x10, 0xD8,	/* 76 */
	0x00, 0x00, 0x88, 0x00,	/* 80 */
	0x00, 0x00, 0x00, 0x00,	/* 84 */
	0x7F, 0x00, 0x00, 0x00,	/* 88 */
	0x00, 0x00, 0x00, 0x00,	/* 92 */
	0x7F, 0x00, 0x00, 0x00,	/* 96 */
	0x00, 0x00, 0x00, 0x00,	/* 100 */
	0x00, 0x00, 0x00, 0x00,	/* 104 */
	0x00, 0x00, 0x00, 0x00,	/* 108 */
	0x00, 0x00, 0x00, 0x00,	/* 112 */
	0x00, 0x00, 0x00, 0x00,	/* 116 */
	0x00, 0x00, 0x00, 0x00,	/* 120 */
	0x00, 0x00, 0x00, 0x00,	/* 124 - PAGE0 Registers(127) ends here */
	0x01, 0x00, 0x08, 0x00,	/* 128, PAGE1-0 */
	0x00, 0x00, 0x00, 0x00,	/* 132, PAGE1-4 */
	0x00, 0x00, 0x00, 0x10,	/* 136, PAGE1-8 */
	0x00, 0x00, 0x00, 0x00,	/* 140, PAGE1-12 */
	0x40, 0x40, 0x40, 0x40,	/* 144, PAGE1-16 */
	0x00, 0x00, 0x00, 0x00,	/* 148, PAGE1-20 */
	0x00, 0x00, 0x00, 0x00,	/* 152, PAGE1-24 */
	0x00, 0x00, 0x00, 0x00,	/* 156, PAGE1-28 */
	0x00, 0x00, 0x00, 0x00,	/* 160, PAGE1-32 */
	0x00, 0x00, 0x00, 0x00,	/* 164, PAGE1-36 */
	0x00, 0x00, 0x00, 0x00,	/* 168, PAGE1-40 */
	0x00, 0x00, 0x00, 0x00,	/* 172, PAGE1-44 */
	0x00, 0x00, 0x00, 0x00,	/* 176, PAGE1-48 */
	0x00, 0x00, 0x00, 0x00,	/* 180, PAGE1-52 */
	0x00, 0x00, 0x00, 0x80,	/* 184, PAGE1-56 */
	0x80, 0x00, 0x00, 0x00,	/* 188, PAGE1-60 */
	0x00, 0x00, 0x00, 0x00,	/* 192, PAGE1-64 */
	0x00, 0x00, 0x00, 0x00,	/* 196, PAGE1-68 */
	0x00, 0x00, 0x00, 0x00,	/* 200, PAGE1-72 */
	0x00, 0x00, 0x00, 0x00,	/* 204, PAGE1-76 */
	0x00, 0x00, 0x00, 0x00,	/* 208, PAGE1-80 */
	0x00, 0x00, 0x00, 0x00,	/* 212, PAGE1-84 */
	0x00, 0x00, 0x00, 0x00,	/* 216, PAGE1-88 */
	0x00, 0x00, 0x00, 0x00,	/* 220, PAGE1-92 */
	0x00, 0x00, 0x00, 0x00,	/* 224, PAGE1-96 */
	0x00, 0x00, 0x00, 0x00,	/* 228, PAGE1-100 */
	0x00, 0x00, 0x00, 0x00,	/* 232, PAGE1-104 */
	0x00, 0x00, 0x00, 0x00,	/* 236, PAGE1-108 */
	0x00, 0x00, 0x00, 0x00,	/* 240, PAGE1-112 */
	0x00, 0x00, 0x00, 0x00,	/* 244, PAGE1-116 */
	0x00, 0x00, 0x00, 0x00,	/* 248, PAGE1-120 */
	0x00, 0x00, 0x00, 0x00,	/* 252, PAGE1-124 Page 1 Registers Ends Here */
	0x00, 0x00, 0x00, 0x00,	/* 256, PAGE2-0  */
	0x00, 0x00, 0x00, 0x00,	/* 260, PAGE2-4  */
	0x00, 0x00, 0x00, 0x00,	/* 264, PAGE2-8  */
	0x00, 0x00, 0x00, 0x00,	/* 268, PAGE2-12 */
	0x00, 0x00, 0x00, 0x00,	/* 272, PAGE2-16 */
	0x00, 0x00, 0x00, 0x00,	/* 276, PAGE2-20 */
	0x00, 0x00, 0x00, 0x00,	/* 280, PAGE2-24 */
	0x00, 0x00, 0x00, 0x00,	/* 284, PAGE2-28 */
	0x00, 0x00, 0x00, 0x00,	/* 288, PAGE2-32 */
	0x00, 0x00, 0x00, 0x00,	/* 292, PAGE2-36 */
	0x00, 0x00, 0x00, 0x00,	/* 296, PAGE2-40 */
	0x00, 0x00, 0x00, 0x00,	/* 300, PAGE2-44 */
	0x00, 0x00, 0x00, 0x00,	/* 304, PAGE2-48 */
	0x00, 0x00, 0x00, 0x00,	/* 308, PAGE2-52 */
	0x00, 0x00, 0x00, 0x00,	/* 312, PAGE2-56 */
	0x00, 0x00, 0x00, 0x00,	/* 316, PAGE2-60 */
	0x00, 0x00, 0x00, 0x00,	/* 320, PAGE2-64 */
	0x00, 0x00, 0x00, 0x00,	/* 324, PAGE2-68 */
	0x00, 0x00, 0x00, 0x00,	/* 328, PAGE2-72 */
	0x00, 0x00, 0x00, 0x00,	/* 332, PAGE2-76 */
	0x00, 0x00, 0x00, 0x00,	/* 336, PAGE2-80 */
	0x00, 0x00, 0x00, 0x00,	/* 340, PAGE2-84 */
	0x00, 0x00, 0x00, 0x00,	/* 344, PAGE2-88 */
	0x00, 0x00, 0x00, 0x00,	/* 348, PAGE2-92 */
	0x00, 0x00, 0x00, 0x00,	/* 352, PAGE2-96 */
	0x00, 0x00, 0x00, 0x00,	/* 356, PAGE2-100 */
	0x00, 0x00, 0x00, 0x00,	/* 360, PAGE2-104 */
	0x00, 0x00, 0x00, 0x00,	/* 364, PAGE2-108 */
	0x00, 0x00, 0x00, 0x00,	/* 368, PAGE2-112*/
	0x00, 0x00, 0x00, 0x00,	/* 372, PAGE2-116*/
	0x00, 0x00, 0x00, 0x00,	/* 376, PAGE2-120*/
	0x00, 0x00, 0x00, 0x00,	/* 380, PAGE2-124 Page 2 Registers Ends Here */
	0x00, 0x00, 0x00, 0x00,	/* 384, PAGE3-0  */
	0x00, 0x00, 0x00, 0x00,	/* 388, PAGE3-4  */
	0x00, 0x00, 0x00, 0x00,	/* 392, PAGE3-8  */
	0x00, 0x00, 0x00, 0x00,	/* 392, PAGE3-12  */
	0x00, 0x00, 0x00, 0x00,	/* 392, PAGE3-16  */
	0x00, 0x00, 0x00, 0x00,	/* 392, PAGE3-20  */
	0x00, 0x00, 0x00, 0x00,	/* 392, PAGE3-24  */
	0x00, 0x00, 0x00, 0x00,	/* 392, PAGE3-28  */
	0x00, 0x00, 0x00, 0x00,	/* 392, PAGE3-32  */
	0x00, 0x00, 0x00, 0x00,	/* 392, PAGE3-36  */
	0x00, 0x00, 0x00, 0x00,	/* 392, PAGE3-40  */
	0x00, 0x00, 0x00, 0x00,	/* 392, PAGE3-44  */
	0x00, 0x00, 0x00, 0x00,	/* 392, PAGE3-48  */
	0x00, 0x00, 0x00, 0x00,	/* 392, PAGE3-52  */
	0x00, 0x00, 0x00, 0x00,	/* 392, PAGE3-56  */
	0x00, 0x00, 0x00, 0x00,	/* 392, PAGE3-60  */
	0x00, 0x00, 0x00, 0x00,	/* 392, PAGE3-64  */
	0x00, 0x00, 0x00, 0x00,	/* 392, PAGE3-68  */
	0x00, 0x00, 0x00, 0x00,	/* 328, PAGE3-72 */
	0x00, 0x00, 0x00, 0x00,	/* 332, PAGE3-76 */
	0x00, 0x00, 0x00, 0x00,	/* 336, PAGE3-80 */
	0x00, 0x00, 0x00, 0x00,	/* 340, PAGE3-84 */
	0x00, 0x00, 0x00, 0x00,	/* 344, PAGE3-88 */
	0x00, 0x00, 0x00, 0x00,	/* 348, PAGE3-92 */
	0x00, 0x00, 0x00, 0x00,	/* 352, PAGE3-96 */
	0x00, 0x00, 0x00, 0x00,	/* 356, PAGE3-100 */
	0x00, 0x00, 0x00, 0x00,	/* 360, PAGE3-104 */
	0x00, 0x00, 0x00, 0x00,	/* 364, PAGE3-108 */
	0x00, 0x00, 0x00, 0x00,	/* 368, PAGE3-112*/
	0x00, 0x00, 0x00, 0x00,	/* 372, PAGE3-116*/
	0x00, 0x00, 0x00, 0x00,	/* 376, PAGE3-120*/
	0x00, 0x00, 0x00, 0x00,	/* 380, PAGE3-124 Page 3 Registers Ends Here */
	0x00, 0x00, 0x00, 0x00,	/* 384, PAGE4-0  */
	0x00, 0x00, 0x00, 0x00,	/* 388, PAGE4-4  */
	0x00, 0x00, 0x00, 0x00,	/* 392, PAGE4-8  */
	0x00, 0x00, 0x00, 0x00,	/* 392, PAGE4-12  */
	0x00, 0x00, 0x00, 0x00,	/* 392, PAGE4-16  */
	0x00, 0x00, 0x00, 0x00,	/* 392, PAGE4-20  */
	0x00, 0x00, 0x00, 0x00,	/* 392, PAGE4-24  */
	0x00, 0x00, 0x00, 0x00,	/* 392, PAGE4-28  */
	0x00, 0x00, 0x00, 0x00,	/* 392, PAGE4-32  */
	0x00, 0x00, 0x00, 0x00,	/* 392, PAGE4-36  */
	0x00, 0x00, 0x00, 0x00,	/* 392, PAGE4-40  */
	0x00, 0x00, 0x00, 0x00,	/* 392, PAGE4-44  */
	0x00, 0x00, 0x00, 0x00,	/* 392, PAGE4-48  */
	0x00, 0x00, 0x00, 0x00,	/* 392, PAGE4-52  */
	0x00, 0x00, 0x00, 0x00,	/* 392, PAGE4-56  */
	0x00, 0x00, 0x00, 0x00,	/* 392, PAGE4-60  */
	0x00, 0x00, 0x00, 0x00,	/* 392, PAGE4-64  */
	0x00, 0x00, 0x00, 0x00,	/* 392, PAGE4-68  */
	0x00, 0x00, 0x00, 0x00,	/* 328, PAGE4-72 */
	0x00, 0x00, 0x00, 0x00,	/* 332, PAGE4-76 */
	0x00, 0x00, 0x00, 0x00,	/* 336, PAGE4-80 */
	0x00, 0x00, 0x00, 0x00,	/* 340, PAGE4-84 */
	0x00, 0x00, 0x00, 0x00,	/* 344, PAGE4-88 */
	0x00, 0x00, 0x00, 0x00,	/* 348, PAGE4-92 */
	0x00, 0x00, 0x00, 0x00,	/* 352, PAGE4-96 */
	0x00, 0x00, 0x00, 0x00,	/* 356, PAGE4-100 */
	0x00, 0x00, 0x00, 0x00,	/* 360, PAGE4-104 */
	0x00, 0x00, 0x00, 0x00,	/* 364, PAGE4-108 */
	0x00, 0x00, 0x00, 0x00,	/* 368, PAGE4-112*/
	0x00, 0x00, 0x00, 0x00,	/* 372, PAGE4-116*/
	0x00, 0x00, 0x00, 0x00,	/* 376, PAGE4-120*/
	0x00, 0x00, 0x00, 0x00,	/* 380, PAGE4-124 Page 2 Registers Ends Here */

};

/*
 *------------------------------------------------------------------------------
 * aic3262 initialization data
 * This structure initialization contains the initialization required for
 * AIC326x.
 * These registers values (reg_val) are written into the respective AIC3262
 * register offset (reg_offset) to  initialize AIC326x.
 * These values are used in aic3262_init() function only.
 *------------------------------------------------------------------------------
 */
static const struct aic3262_configs aic3262_reg_init[] = {
	/* CLOCKING */

	{0, RESET_REG, 1},
	{0, RESET_REG, 0},

	{0, PASI_DAC_DP_SETUP,  0xc0},	/*DAC */
	{0, DAC_MVOL_CONF,  0x00},	/*DAC un-muted*/
	/* set default volumes */
	{0, DAC_LVOL, 0x01},
	{0, DAC_RVOL, 0x01},
	{0, HPL_VOL,  0x80},
	{0, HPR_VOL,  0x80},
	{0, SPK_AMP_CNTL_R2, 0x14},
	{0, SPK_AMP_CNTL_R3, 0x14},
	{0, SPK_AMP_CNTL_R4, 0x33},
	{0, REC_AMP_CNTL_R5, 0x82},
	{0, RAMPR_VOL, 20},
	{0, RAMP_CNTL_R1, 70},
	{0, RAMP_CNTL_R2, 70},

	/* DRC Defaults */
	{0, DRC_CNTL_R1, 0x6c},
	{0, DRC_CNTL_R2, 16},

	/* DEPOP SETTINGS */
	{0, HP_DEPOP, 0x14},
	{0, RECV_DEPOP, 0x14},

	{0, POWER_CONF, 0x00},	 /* Disconnecting AVDD-DVD weak link*/
	{0, REF_PWR_DLY, 0x01},
	{0, CM_REG, 0x00},	/*CM - default*/
	{0, LDAC_PTM, 0},	/*LDAC_PTM - default*/
	{0, RDAC_PTM, 0},	/*RDAC_PTM - default*/
	{0, HP_CTL, 0x30},	/*HP output percentage - at 75%*/
	{0, LADC_VOL, 0x01},	/*LADC volume*/
	{0, RADC_VOL, 0x01},	/*RADC volume*/

	{0, DAC_ADC_CLKIN_REG, 0x33},	/*DAC ADC CLKIN*/
	{0, PLL_CLKIN_REG, 0x00},	/*PLL CLKIN*/
	{0, PLL_PR_POW_REG, 0x11},	/*PLL Power=0-down, P=1, R=1 vals*/
	{0, 0x3d, 1},

	{0, LMIC_PGA_PIN, 0x0},	/*IN1_L select - - 10k -LMICPGA_P*/
	{0, LMIC_PGA_MIN, 0x40},	/*CM to LMICPGA-M*/
	{0, RMIC_PGA_PIN, 0x0},	/*IN1_R select - - 10k -RMIC_PGA_P*/
	{0, RMIC_PGA_MIN, 0x0},	/*CM to RMICPGA_M*/
	{0, MIC_PWR_DLY , 33},	/*LMIC-PGA-POWERUP-DELAY - default*/
	{0, REF_PWR_DLY, 1},	/*FIXMELATER*/


	{0, ADC_CHANNEL_POW, 0x0}, /*ladc, radc ON , SOFT STEP disabled*/
	{0, ADC_FINE_GAIN, 0x00},   /*ladc - unmute, radc - unmute*/
	{0, MICL_PGA, 0x3f},
	{0, MICR_PGA, 0x3f},
	/*controls MicBias ext power based on B0_P1_R51_D6*/
	{0, MIC_BIAS_CNTL, 0x80},
	/*   ASI1 Configuration */
	{0, ASI1_BUS_FMT, 0},
	{0, ASI1_BWCLK_CNTL_REG, 0x00},		/* originaly 0x24*/
	{0, ASI1_BCLK_N_CNTL, 1},
	{0, ASI1_BCLK_N, 0x04},

	{0, MA_CNTL, 0},			/* Mixer Amp disabled */
	{0, LINE_AMP_CNTL_R2, 0x00},		/* Line Amp Cntl disabled */

	/* ASI2 Configuration */
	{0, ASI2_BUS_FMT, 0},
	{0, ASI2_BCLK_N_CNTL, 0x01},
	{0, ASI2_BCLK_N, 0x04},
	{0, ASI2_BWCLK_OUT_CNTL, 0x20},

	{0, BEEP_CNTL_R1, 0x05},
	{0, BEEP_CNTL_R2, 0x04},

	/* Interrupt config for headset detection */
	{0,HEADSET_TUNING1_REG,0x7f},
	{0, INT1_CNTL, 0x40},
	/*{0, TIMER_REG, 0x8c},*/
	{0, INT_FMT, 0x40},
	{0, GPIO1_IO_CNTL, 0x14},
	{0, HP_DETECT, 0x96},

#if defined(CONFIG_MINI_DSP)
	{0, 60, 0},
	{0, 61, 0},
	/* Added the below set of values after consulting the miniDSP
	 * Program Section Array
	 */
	{0, MINIDSP_ACCESS_CTRL, 0x00},
#endif


};

static int reg_init_size =
	sizeof(aic3262_reg_init) / sizeof(struct aic3262_configs);

static const unsigned int adc_ma_tlv[] = {
TLV_DB_RANGE_HEAD(4),
	0, 29, TLV_DB_SCALE_ITEM(-1450, 500, 0),
	30, 35, TLV_DB_SCALE_ITEM(-2060, 1000, 0),
	36, 38, TLV_DB_SCALE_ITEM(-2660, 2000, 0),
	39, 40, TLV_DB_SCALE_ITEM(-3610, 5000, 0),
};
static const DECLARE_TLV_DB_SCALE(lo_hp_tlv, -7830, 50, 0);

static const struct snd_kcontrol_new mal_pga_mixer_controls[] = {
	SOC_DAPM_SINGLE("IN1L Switch", MA_CNTL, 5, 1, 0),
	SOC_DAPM_SINGLE_TLV("Left MicPGA Volume", LADC_PGA_MAL_VOL, 0,
							0x3f, 1, adc_ma_tlv),

};

static const struct snd_kcontrol_new mar_pga_mixer_controls[] = {
	SOC_DAPM_SINGLE("IN1R Switch", MA_CNTL, 4, 1, 0),
	SOC_DAPM_SINGLE_TLV("Right MicPGA Volume", RADC_PGA_MAR_VOL, 0,
							0x3f, 1, adc_ma_tlv),
};

/* Left HPL Mixer */
static const struct snd_kcontrol_new hpl_output_mixer_controls[] = {
	SOC_DAPM_SINGLE("MAL Switch", HP_AMP_CNTL_R1, 7, 1, 0),
	SOC_DAPM_SINGLE("LDAC Switch", HP_AMP_CNTL_R1, 5, 1, 0),
	SOC_DAPM_SINGLE_TLV("LOL-B1 Volume", HP_AMP_CNTL_R2, 0,
							0x7f, 1, lo_hp_tlv),
};

/* Right HPR Mixer */
static const struct snd_kcontrol_new hpr_output_mixer_controls[] = {
	SOC_DAPM_SINGLE_TLV("LOR-B1 Volume", HP_AMP_CNTL_R3, 0,
							0x7f, 1, lo_hp_tlv),
	SOC_DAPM_SINGLE("LDAC Switch", HP_AMP_CNTL_R1,	 2, 1, 0),
	SOC_DAPM_SINGLE("RDAC Switch", HP_AMP_CNTL_R1, 4, 1, 0),
	SOC_DAPM_SINGLE("MAR Switch", HP_AMP_CNTL_R1, 6, 1, 0),
};

/* Left LOL Mixer */
static const struct snd_kcontrol_new lol_output_mixer_controls[] = {
	SOC_DAPM_SINGLE("MAL Switch", LINE_AMP_CNTL_R2, 7, 1, 0),
	SOC_DAPM_SINGLE("IN1L-B Switch", LINE_AMP_CNTL_R2, 3, 1,0),
	SOC_DAPM_SINGLE("LDAC Switch", LINE_AMP_CNTL_R1, 7, 1, 0),
	SOC_DAPM_SINGLE("RDAC Switch", LINE_AMP_CNTL_R1, 5, 1, 0),
};

/* Right LOR Mixer */
static const struct snd_kcontrol_new lor_output_mixer_controls[] = {
	SOC_DAPM_SINGLE("LOL Switch", LINE_AMP_CNTL_R1, 2, 1, 0),
	SOC_DAPM_SINGLE("RDAC Switch", LINE_AMP_CNTL_R1, 6, 1, 0),
	SOC_DAPM_SINGLE("MAR Switch", LINE_AMP_CNTL_R2, 6, 1, 0),
	SOC_DAPM_SINGLE("IN1R-B Switch", LINE_AMP_CNTL_R2, 0, 1,0),
};

/* Left SPKL Mixer */
static const struct snd_kcontrol_new spkl_output_mixer_controls[] = {
	SOC_DAPM_SINGLE("MAL Switch", SPK_AMP_CNTL_R1, 7, 1, 0),
	SOC_DAPM_SINGLE_TLV("LOL Volume", SPK_AMP_CNTL_R2, 0, 0x7f,0,
								lo_hp_tlv),
	SOC_DAPM_SINGLE("SPR_IN Switch", SPK_AMP_CNTL_R1, 2, 1, 0),
};

/* Right SPKR Mixer */
static const struct snd_kcontrol_new spkr_output_mixer_controls[] = {
	SOC_DAPM_SINGLE_TLV("LOR Volume", SPK_AMP_CNTL_R3, 0, 0x7f, 0,
								lo_hp_tlv),
	SOC_DAPM_SINGLE("MAR Switch", SPK_AMP_CNTL_R1, 6, 1, 0),
};

/* REC Mixer */
static const struct snd_kcontrol_new rec_output_mixer_controls[] = {
	SOC_DAPM_SINGLE_TLV("LOL-B2 Volume", RAMP_CNTL_R1, 0, 0x7f,0,
							lo_hp_tlv),
	SOC_DAPM_SINGLE_TLV("IN1L Volume", IN1L_SEL_RM, 0, 0x7f, 1, lo_hp_tlv),
	SOC_DAPM_SINGLE_TLV("IN1R Volume", IN1R_SEL_RM, 0, 0x7f, 1, lo_hp_tlv),
	SOC_DAPM_SINGLE_TLV("LOR-B2 Volume", RAMP_CNTL_R2, 0,0x7f, 0,lo_hp_tlv),
};

/* Left Input Mixer */
static const struct snd_kcontrol_new left_input_mixer_controls[] = {
	SOC_DAPM_SINGLE("IN1L Switch", LMIC_PGA_PIN, 6, 1, 0),
	SOC_DAPM_SINGLE("IN2L Switch", LMIC_PGA_PIN, 4, 1, 0),
	SOC_DAPM_SINGLE("IN3L Switch", LMIC_PGA_PIN, 2, 1, 0),
	SOC_DAPM_SINGLE("IN4L Switch", LMIC_PGA_PM_IN4, 5, 1, 0),
	SOC_DAPM_SINGLE("IN1R Switch", LMIC_PGA_PIN, 0, 1, 0),
	SOC_DAPM_SINGLE("IN2R Switch", LMIC_PGA_MIN, 4, 1, 0),
	SOC_DAPM_SINGLE("IN3R Switch", LMIC_PGA_MIN, 2, 1, 0),
	SOC_DAPM_SINGLE("IN4R Switch", LMIC_PGA_PM_IN4, 4, 1, 0),
	SOC_DAPM_SINGLE("CM2L Switch", LMIC_PGA_MIN, 0, 1, 0),
	SOC_DAPM_SINGLE("CM1L Switch", LMIC_PGA_MIN, 6, 1, 0),
};

/* Right Input Mixer */
static const struct snd_kcontrol_new right_input_mixer_controls[] = {
	SOC_DAPM_SINGLE("IN1R Switch", RMIC_PGA_PIN, 6, 1, 0),
	SOC_DAPM_SINGLE("IN2R Switch", RMIC_PGA_PIN, 4, 1, 0),
	SOC_DAPM_SINGLE("IN3R Switch", RMIC_PGA_PIN, 2, 1, 0),
	SOC_DAPM_SINGLE("IN4R Switch", RMIC_PGA_PM_IN4, 5, 1, 0),
	SOC_DAPM_SINGLE("IN2L Switch", RMIC_PGA_PIN, 0, 1, 0),
	SOC_DAPM_SINGLE("IN1L Switch", RMIC_PGA_MIN, 4, 1, 0),
	SOC_DAPM_SINGLE("IN3L Switch", RMIC_PGA_MIN, 2, 1, 0),
	SOC_DAPM_SINGLE("IN4L Switch", RMIC_PGA_PM_IN4, 4, 1, 0),
	SOC_DAPM_SINGLE("CM1R Switch", RMIC_PGA_MIN, 6, 1, 0),
	SOC_DAPM_SINGLE("CM2R Switch", RMIC_PGA_MIN, 0, 1, 0),
};


static const char *asi1lin_text[] = {
	"Off", "ASI1 Left In","ASI1 Right In","ASI1 MonoMix In"
};

SOC_ENUM_SINGLE_DECL(asi1lin_enum, ASI1_DAC_OUT_CNTL, 6, asi1lin_text);

static const struct snd_kcontrol_new asi1lin_control =
	SOC_DAPM_ENUM("ASI1LIN Route", asi1lin_enum);


static const char *asi1rin_text[] = {
	"Off", "ASI1 Right In","ASI1 Left In","ASI1 MonoMix In"
};

SOC_ENUM_SINGLE_DECL(asi1rin_enum, ASI1_DAC_OUT_CNTL, 4, asi1rin_text);

static const struct snd_kcontrol_new asi1rin_control =
	SOC_DAPM_ENUM("ASI1RIN Route", asi1rin_enum);

static const char *asi2lin_text[] = {
	"Off", "ASI2 Left In","ASI2 Right In","ASI2 MonoMix In"
};

SOC_ENUM_SINGLE_DECL(asi2lin_enum, ASI2_DAC_OUT_CNTL, 6, asi2lin_text);
static const struct snd_kcontrol_new asi2lin_control =
	SOC_DAPM_ENUM("ASI2LIN Route", asi2lin_enum);

static const char *asi2rin_text[] = {
	"Off", "ASI2 Right In","ASI2 Left In","ASI2 MonoMix In"
};


SOC_ENUM_SINGLE_DECL(asi2rin_enum, ASI2_DAC_OUT_CNTL, 4, asi2rin_text);

static const struct snd_kcontrol_new asi2rin_control =
	SOC_DAPM_ENUM("ASI2RIN Route", asi2rin_enum);

static const char *asi3lin_text[] = {
	"Off", "ASI3 Left In","ASI3 Right In","ASI3 MonoMix In"
};


SOC_ENUM_SINGLE_DECL(asi3lin_enum, ASI3_DAC_OUT_CNTL, 6, asi3lin_text);
static const struct snd_kcontrol_new asi3lin_control =
			SOC_DAPM_ENUM("ASI3LIN Route", asi3lin_enum);


static const char *asi3rin_text[] = {
	"Off", "ASI3 Right In","ASI3 Left In","ASI3 MonoMix In"
};


SOC_ENUM_SINGLE_DECL(asi3rin_enum, ASI3_DAC_OUT_CNTL, 4, asi3rin_text);
static const struct snd_kcontrol_new asi3rin_control =
	SOC_DAPM_ENUM("ASI3RIN Route", asi3rin_enum);


static const char *dacminidspin1_text[] = {
	"ASI1 In", "ASI2 In","ASI3 In","ADC MiniDSP Out"
};

SOC_ENUM_SINGLE_DECL(dacminidspin1_enum, MINIDSP_PORT_CNTL_REG, 4, dacminidspin1_text);
static const struct snd_kcontrol_new dacminidspin1_control =
	SOC_DAPM_ENUM("DAC MiniDSP IN1 Route", dacminidspin1_enum);

static const char *dacminidspin2_text[] = {
	"ASI1 In", "ASI2 In","ASI3 In"
};

//static const struct soc_enum dacminidspin1_enum =
//       SOC_ENUM_SINGLE(MINIDSP_DATA_PORT_CNTL, 5, 2, dacminidspin1_text);
SOC_ENUM_SINGLE_DECL(dacminidspin2_enum, MINIDSP_PORT_CNTL_REG, 2, dacminidspin2_text);

static const struct snd_kcontrol_new dacminidspin2_control =
	SOC_DAPM_ENUM("DAC MiniDSP IN2 Route", dacminidspin2_enum);

static const char *dacminidspin3_text[] = {
	"ASI1 In", "ASI2 In","ASI3 In"
};

//static const struct soc_enum dacminidspin1_enum =
//       SOC_ENUM_SINGLE(MINIDSP_DATA_PORT_CNTL, 5, 2, dacminidspin1_text);
SOC_ENUM_SINGLE_DECL(dacminidspin3_enum, MINIDSP_PORT_CNTL_REG, 0, dacminidspin3_text);

static const struct snd_kcontrol_new dacminidspin3_control =
SOC_DAPM_ENUM("DAC MiniDSP IN3 Route", dacminidspin3_enum);

static const char *asi1out_text[] = {
	"Off",
	"ASI1 Out",
	"ASI1In Bypass",
	"ASI2In Bypass",
	"ASI3In Bypass",
};
SOC_ENUM_SINGLE_DECL(asi1out_enum, ASI1_ADC_INPUT_CNTL, 0, asi1out_text);
static const struct snd_kcontrol_new asi1out_control =
	SOC_DAPM_ENUM("ASI1OUT Route", asi1out_enum);

static const char *asi2out_text[] = {
	"Off",
	"ASI1 Out",
	"ASI1In Bypass",
	"ASI2In Bypass",
	"ASI3In Bypass",
	"ASI2 Out",
};
SOC_ENUM_SINGLE_DECL(asi2out_enum, ASI2_ADC_INPUT_CNTL, 0, asi2out_text);
static const struct snd_kcontrol_new asi2out_control =
	SOC_DAPM_ENUM("ASI2OUT Route", asi2out_enum);
static const char *asi3out_text[] = {
	"Off",
	"ASI1 Out",
	"ASI1In Bypass",
	"ASI2In Bypass",
	"ASI3In Bypass",
	"ASI3 Out",
};
SOC_ENUM_SINGLE_DECL(asi3out_enum, ASI3_ADC_INPUT_CNTL, 0, asi3out_text);
static const struct snd_kcontrol_new asi3out_control =
	SOC_DAPM_ENUM("ASI3OUT Route", asi3out_enum);

static const char *asi1bclk_text[] = {
	"DAC_CLK",
	"DAC_MOD_CLK",
	"ADC_CLK",
	"ADC_MOD_CLK",
};

SOC_ENUM_SINGLE_DECL(asi1bclk_enum, ASI1_BCLK_N_CNTL, 0, asi1bclk_text);
static const struct snd_kcontrol_new asi1bclk_control =
	SOC_DAPM_ENUM("ASI1_BCLK Route", asi1bclk_enum);

static const char *asi2bclk_text[] = {
	"DAC_CLK",
	"DAC_MOD_CLK",
	"ADC_CLK",
	"ADC_MOD_CLK",
};
SOC_ENUM_SINGLE_DECL(asi2bclk_enum, ASI2_BCLK_N_CNTL, 0, asi2bclk_text);
static const struct snd_kcontrol_new asi2bclk_control =
	SOC_DAPM_ENUM("ASI2_BCLK Route", asi2bclk_enum);
static const char *asi3bclk_text[] = {
	"DAC_CLK",
	"DAC_MOD_CLK",
	"ADC_CLK",
	"ADC_MOD_CLK",
};
SOC_ENUM_SINGLE_DECL(asi3bclk_enum, ASI3_BCLK_N_CNTL, 0, asi3bclk_text);
static const struct snd_kcontrol_new asi3bclk_control =
	SOC_DAPM_ENUM("ASI3_BCLK Route", asi3bclk_enum);

static int aic326x_hp_event(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	return 0;
}
static int pll_power_on_event(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	if (event == SND_SOC_DAPM_POST_PMU)
	{
		mdelay(10);
	}
	return 0;
}

static int polling_loop(struct snd_soc_codec *codec, unsigned int reg,
			int mask, int on_off)
{
	unsigned int counter, status;

	counter = 0;
	switch(on_off) {
		case 0: /*off*/
			do {
				status = snd_soc_read(codec, reg);
				counter++;
			} while ((counter < 500) && ((status & mask) == mask));
		break;
		case 1: /*on*/
			do {
				status = snd_soc_read(codec, reg);
				counter++;
			} while ((counter < 500) && ((status & mask) != mask));
		break;
		default:
			printk("%s: unknown arguement\n", __func__);
			break;
	}

	printk("%s: exiting with count value %d \n", __func__, counter);
	if(counter >= 500)
		return -1;
	return 0;
}

int poll_dac(struct snd_soc_codec *codec, int left_right, int on_off)
{
	int ret = 0;

	aic3262_change_page(codec, 0);
	aic3262_change_book(codec, 0);

	switch(on_off) {

		case 0:/*power off polling*/
			/*DAC power polling logic*/
			switch(left_right) {
				case 0: /*left dac polling*/
					ret = polling_loop(codec, DAC_FLAG_R1, LDAC_POW_FLAG_MASK, 0);
					break;
				case 1:/*right dac polling*/
					ret = polling_loop(codec, DAC_FLAG_R1, RDAC_POW_FLAG_MASK, 0);
					break;
			}
			break;
		case 1:/*power on polling*/
			/*DAC power polling logic*/
			switch(left_right) {
				case 0: /*left dac polling*/
					ret = polling_loop(codec, DAC_FLAG_R1, LDAC_POW_FLAG_MASK, 1);
					break;
				case 1:/*right dac polling*/
					ret = polling_loop(codec, DAC_FLAG_R1, RDAC_POW_FLAG_MASK, 1);
					break;
			}
		break;
		default:
			printk("%s:unknown arguement\n", __func__);
			break;
		}
	if(ret)
		printk("%s: power %s %s failure", __func__, left_right?"right":"left", on_off?"on":"off");
	return ret;
}

int poll_adc(struct snd_soc_codec *codec, int left_right, int on_off)
{
	int ret = 0;

	aic3262_change_page(codec, 0);
	aic3262_change_book(codec, 0);

	switch(on_off) {

		case 0:/*power off polling*/
			/*DAC power polling logic*/
			switch(left_right) {
				case 0: /*left dac polling*/
					ret = polling_loop(codec, ADC_FLAG_R1, LADC_POW_FLAG_MASK, 0);
					break;
				case 1:/*right dac polling*/
					ret = polling_loop(codec, ADC_FLAG_R1, RADC_POW_FLAG_MASK, 0);
					break;
	}
			break;
		case 1:/*power on polling*/
			/*DAC power polling logic*/
			switch(left_right) {
				case 0: /*left dac polling*/
					ret = polling_loop(codec, ADC_FLAG_R1, LADC_POW_FLAG_MASK, 1);
					break;
				case 1:/*right dac polling*/
					ret = polling_loop(codec, ADC_FLAG_R1, RADC_POW_FLAG_MASK, 1);
					break;
		}
			break;
		default:
			printk("%s:unknown arguement\n", __func__);
			break;
	}

	if(ret)
		printk("%s: power %s %s failure", __func__, left_right?"right":"left", on_off?"on":"off");
	return ret;
}

static int  slave_dac_event(struct snd_soc_dapm_widget *w,
			       struct snd_kcontrol *kcontrol, int event)

{
	struct snd_soc_codec *codec = w->codec;

	if (event & SND_SOC_DAPM_POST_PMU) {
		/* Poll for DAC Power-up first */
		poll_dac(codec, 0, 1);
		poll_dac(codec, 1, 1);
	}

	if (event & SND_SOC_DAPM_POST_PMD) {
		poll_dac(codec, 0, 0);
		poll_dac(codec, 1, 0);
	}
	return 0;
}


static int  slave_adc_event(struct snd_soc_dapm_widget *w,
			       struct snd_kcontrol *kcontrol, int event)

{
	struct snd_soc_codec *codec = w->codec;

	if (event & SND_SOC_DAPM_POST_PMU) {

		/* Poll for ADC Power-up first */
		poll_adc(codec, 0, 1);
		poll_adc(codec, 1, 1);
	}


	if (event & SND_SOC_DAPM_POST_PMD) {
		poll_adc(codec, 0, 0);
		poll_adc(codec, 1, 0);
	}

	return 0;
}

static const struct snd_soc_dapm_widget aic3262_dapm_widgets[] = {
	/* TODO: Can we switch these off ? */
	SND_SOC_DAPM_AIF_IN("ASI1IN", "ASI1 Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("ASI2IN", "ASI2 Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("ASI3IN", "ASI3 Playback", 0, SND_SOC_NOPM, 0, 0),


	SND_SOC_DAPM_DAC_E("Left DAC", NULL, PASI_DAC_DP_SETUP, 7, 0,
			slave_dac_event, SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD |
			SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_DAC_E("Right DAC", NULL, PASI_DAC_DP_SETUP, 6, 0,
			slave_dac_event, SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD |
			SND_SOC_DAPM_PRE_PMD),

	/* dapm widget (path domain) for HPL Output Mixer */
	SND_SOC_DAPM_MIXER("HPL Output Mixer", SND_SOC_NOPM, 0, 0,
		 &hpl_output_mixer_controls[0],
		ARRAY_SIZE(hpl_output_mixer_controls)),

	/* dapm widget (path domain) for HPR Output Mixer */
	SND_SOC_DAPM_MIXER("HPR Output Mixer", SND_SOC_NOPM, 0, 0,
			&hpr_output_mixer_controls[0],
			ARRAY_SIZE(hpr_output_mixer_controls)),


	SND_SOC_DAPM_PGA_E("HPL Driver", HP_AMP_CNTL_R1, 1, 0, NULL, 0,
				aic326x_hp_event, SND_SOC_DAPM_POST_PMU),
	SND_SOC_DAPM_PGA_E("HPR Driver", HP_AMP_CNTL_R1, 0, 0, NULL, 0,
				aic326x_hp_event, SND_SOC_DAPM_POST_PMU),


	/* dapm widget (path domain) for LOL Output Mixer */
	SND_SOC_DAPM_MIXER("LOL Output Mixer", SND_SOC_NOPM, 0, 0,
			&lol_output_mixer_controls[0],
			ARRAY_SIZE(lol_output_mixer_controls)),

	/* dapm widget (path domain) for LOR Output Mixer mixer */
	SND_SOC_DAPM_MIXER("LOR Output Mixer", SND_SOC_NOPM, 0, 0,
			&lor_output_mixer_controls[0],
			ARRAY_SIZE(lor_output_mixer_controls)),

	SND_SOC_DAPM_PGA("LOL Driver", LINE_AMP_CNTL_R1, 1, 0, NULL, 0),
	SND_SOC_DAPM_PGA("LOR Driver", LINE_AMP_CNTL_R1, 0, 0, NULL, 0),


	/* dapm widget (path domain) for SPKL Output Mixer */
	SND_SOC_DAPM_MIXER("SPKL Output Mixer", SND_SOC_NOPM, 0, 0,
			&spkl_output_mixer_controls[0],
			ARRAY_SIZE(spkl_output_mixer_controls)),

	/* dapm widget (path domain) for SPKR Output Mixer */
	SND_SOC_DAPM_MIXER("SPKR Output Mixer", SND_SOC_NOPM, 0, 0,
			&spkr_output_mixer_controls[0],
			ARRAY_SIZE(spkr_output_mixer_controls)),

	SND_SOC_DAPM_PGA("SPKL Driver", SPK_AMP_CNTL_R1, 1, 0, NULL, 0),
	SND_SOC_DAPM_PGA("SPKR Driver", SPK_AMP_CNTL_R1, 0, 0, NULL, 0),


	/* dapm widget (path domain) for SPKR Output Mixer */
	SND_SOC_DAPM_MIXER("REC Output Mixer", SND_SOC_NOPM, 0, 0,
			&rec_output_mixer_controls[0],
			ARRAY_SIZE(rec_output_mixer_controls)),

	SND_SOC_DAPM_PGA("RECP Driver", REC_AMP_CNTL_R5, 7, 0, NULL, 0),
	SND_SOC_DAPM_PGA("RECM Driver", REC_AMP_CNTL_R5, 6, 0, NULL, 0),


	SND_SOC_DAPM_MUX("ASI1LIN Route",
		SND_SOC_NOPM, 0, 0, &asi1lin_control),
	SND_SOC_DAPM_MUX("ASI1RIN Route",
		SND_SOC_NOPM, 0, 0, &asi1rin_control),
	SND_SOC_DAPM_MUX("ASI2LIN Route",
		SND_SOC_NOPM, 0, 0, &asi2lin_control),
	SND_SOC_DAPM_MUX("ASI2RIN Route",
		SND_SOC_NOPM, 0, 0, &asi2rin_control),
	SND_SOC_DAPM_MUX("ASI3LIN Route",
		SND_SOC_NOPM, 0, 0, &asi3lin_control),
	SND_SOC_DAPM_MUX("ASI3RIN Route",
		SND_SOC_NOPM, 0, 0, &asi3rin_control),

	SND_SOC_DAPM_PGA("ASI1LIN", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_PGA("ASI1RIN", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_PGA("ASI2LIN", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_PGA("ASI2RIN", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_PGA("ASI3LIN", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_PGA("ASI3RIN", SND_SOC_NOPM, 0, 0, NULL, 0),

	SND_SOC_DAPM_PGA("ASI1LOUT", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_PGA("ASI1ROUT", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_PGA("ASI2LOUT", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_PGA("ASI2ROUT", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_PGA("ASI3LOUT", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_PGA("ASI3ROUT", SND_SOC_NOPM, 0, 0, NULL, 0),

	SND_SOC_DAPM_PGA("ASI1MonoMixIN", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_PGA("ASI2MonoMixIN", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_PGA("ASI3MonoMixIN", SND_SOC_NOPM, 0, 0, NULL, 0),
	/* TODO: Can we switch the ASIxIN off? */
	SND_SOC_DAPM_PGA("ASI1IN Port", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_PGA("ASI2IN Port", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_PGA("ASI3IN Port", SND_SOC_NOPM, 0, 0, NULL, 0),


	SND_SOC_DAPM_MUX("DAC MiniDSP IN1 Route",
			SND_SOC_NOPM, 0, 0, &dacminidspin1_control),
SND_SOC_DAPM_MUX("DAC MiniDSP IN2 Route",
			SND_SOC_NOPM, 0, 0, &dacminidspin2_control),
	SND_SOC_DAPM_MUX("DAC MiniDSP IN3 Route",
			SND_SOC_NOPM, 0, 0, &dacminidspin3_control),

	SND_SOC_DAPM_PGA("CM", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_PGA("CM1L", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_PGA("CM2L", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_PGA("CM1R", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_PGA("CM2R", SND_SOC_NOPM, 0, 0, NULL, 0),

	/* TODO: Can we switch these off ? */
	SND_SOC_DAPM_AIF_OUT("ASI1OUT","ASI1 Capture", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_OUT("ASI2OUT", "ASI2 Capture",0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_OUT("ASI3OUT", "ASI3 Capture",0, SND_SOC_NOPM, 0, 0),

	SND_SOC_DAPM_MUX("ASI1OUT Route",
		SND_SOC_NOPM, 0, 0, &asi1out_control),
	SND_SOC_DAPM_MUX("ASI2OUT Route",
		SND_SOC_NOPM, 0, 0, &asi2out_control),
	SND_SOC_DAPM_MUX("ASI3OUT Route",
		SND_SOC_NOPM, 0, 0, &asi3out_control),

	/* TODO: Will be used during MINIDSP programming */
	/* TODO: Can we switch them off? */
	SND_SOC_DAPM_PGA("ADC MiniDSP OUT1", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_PGA("ADC MiniDSP OUT2", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_PGA("ADC MiniDSP OUT3", SND_SOC_NOPM, 0, 0, NULL, 0),



	SND_SOC_DAPM_ADC_E("Left ADC", NULL, ADC_CHANNEL_POW, 7, 0,
			slave_adc_event, SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD |
			SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_ADC_E("Right ADC", NULL, ADC_CHANNEL_POW, 6, 0,
			slave_adc_event, SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD |
			SND_SOC_DAPM_PRE_PMD),

	SND_SOC_DAPM_PGA("Left MicPGA",MICL_PGA, 7, 1, NULL, 0),
	SND_SOC_DAPM_PGA("Right MicPGA",MICR_PGA, 7, 1, NULL, 0),

	SND_SOC_DAPM_PGA("MAL PGA", MA_CNTL, 3, 0, NULL, 0),
	SND_SOC_DAPM_PGA("MAR PGA", MA_CNTL, 2, 0, NULL, 0),


	/* dapm widget for MAL PGA Mixer*/
	SND_SOC_DAPM_MIXER("MAL PGA Mixer", SND_SOC_NOPM, 0, 0,
			 &mal_pga_mixer_controls[0],
			 ARRAY_SIZE(mal_pga_mixer_controls)),

	/* dapm widget for MAR PGA Mixer*/
	SND_SOC_DAPM_MIXER("MAR PGA Mixer", SND_SOC_NOPM, 0, 0,
			&mar_pga_mixer_controls[0],
			ARRAY_SIZE(mar_pga_mixer_controls)),

	/* dapm widget for Left Input Mixer*/
	SND_SOC_DAPM_MIXER("Left Input Mixer", SND_SOC_NOPM, 0, 0,
			&left_input_mixer_controls[0],
			ARRAY_SIZE(left_input_mixer_controls)),

	/* dapm widget for Right Input Mixer*/
	SND_SOC_DAPM_MIXER("Right Input Mixer", SND_SOC_NOPM, 0, 0,
			&right_input_mixer_controls[0],
			ARRAY_SIZE(right_input_mixer_controls)),


	SND_SOC_DAPM_OUTPUT("HPL"),
	SND_SOC_DAPM_OUTPUT("HPR"),
	SND_SOC_DAPM_OUTPUT("LOL"),
	SND_SOC_DAPM_OUTPUT("LOR"),
	SND_SOC_DAPM_OUTPUT("SPKL"),
	SND_SOC_DAPM_OUTPUT("SPKR"),
	SND_SOC_DAPM_OUTPUT("RECP"),
	SND_SOC_DAPM_OUTPUT("RECM"),

	SND_SOC_DAPM_INPUT("IN1L"),
	SND_SOC_DAPM_INPUT("IN2L"),
	SND_SOC_DAPM_INPUT("IN3L"),
	SND_SOC_DAPM_INPUT("IN4L"),
	SND_SOC_DAPM_INPUT("IN1R"),
	SND_SOC_DAPM_INPUT("IN2R"),
	SND_SOC_DAPM_INPUT("IN3R"),
	SND_SOC_DAPM_INPUT("IN4R"),


	SND_SOC_DAPM_MICBIAS("Mic Bias Ext", MIC_BIAS_CNTL, 6, 0),
	SND_SOC_DAPM_MICBIAS("Mic Bias Int", MIC_BIAS_CNTL, 2, 0),

	SND_SOC_DAPM_SUPPLY("PLLCLK",PLL_PR_POW_REG,7,0,pll_power_on_event,
						SND_SOC_DAPM_POST_PMU),
	SND_SOC_DAPM_SUPPLY("DACCLK",NDAC_DIV_POW_REG,7,0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("CODEC_CLK_IN",SND_SOC_NOPM,0,0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("DAC_MOD_CLK",MDAC_DIV_POW_REG,7,0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("ADCCLK",NADC_DIV_POW_REG,7,0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("ADC_MOD_CLK",MADC_DIV_POW_REG,7,0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("ASI1_BCLK",ASI1_BCLK_N,7,0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("ASI1_WCLK",ASI1_WCLK_N,7,0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("ASI2_BCLK",ASI2_BCLK_N,7,0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("ASI2_WCLK",ASI2_WCLK_N,7,0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("ASI3_BCLK",ASI3_BCLK_N,7,0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("ASI3_WCLK",ASI3_WCLK_N,7,0, NULL, 0),
	SND_SOC_DAPM_MUX("ASI1_BCLK Route",
		SND_SOC_NOPM, 0, 0, &asi1bclk_control),
	SND_SOC_DAPM_MUX("ASI2_BCLK Route", SND_SOC_NOPM, 0, 0, &asi2bclk_control),
	SND_SOC_DAPM_MUX("ASI3_BCLK Route", SND_SOC_NOPM, 0, 0, &asi3bclk_control),
};

static const struct snd_soc_dapm_route aic3262_dapm_routes[] ={
	/* TODO: Do we need only DACCLK for ASIIN's and ADCCLK for ASIOUT??? */
	/* Clock portion */
	{"CODEC_CLK_IN", NULL, "PLLCLK"},
	{"DACCLK", NULL, "CODEC_CLK_IN"},
	{"ADCCLK", NULL, "CODEC_CLK_IN"},
	{"DAC_MOD_CLK", NULL, "DACCLK"},
#ifdef AIC3262_SYNC_MODE
	{"ADC_MOD_CLK", NULL,"DACCLK"},
#else
	{"ADC_MOD_CLK", NULL, "ADCCLK"},
#endif

	{"ASI1_BCLK Route","DAC_CLK","DACCLK"},
	{"ASI1_BCLK Route","DAC_MOD_CLK","DAC_MOD_CLK"},
	{"ASI1_BCLK Route","ADC_CLK","ADCCLK"},
	{"ASI1_BCLK Route","ADC_MOD_CLK","ADC_MOD_CLK"},

	{"ASI2_BCLK Route","DAC_CLK","DACCLK"},
	{"ASI2_BCLK Route","DAC_MOD_CLK","DAC_MOD_CLK"},
	{"ASI2_BCLK Route","ADC_CLK","ADCCLK"},
	{"ASI2_BCLK Route","ADC_MOD_CLK","ADC_MOD_CLK"},

	{"ASI3_BCLK Route","DAC_CLK","DACCLK"},
	{"ASI3_BCLK Route","DAC_MOD_CLK","DAC_MOD_CLK"},
	{"ASI3_BCLK Route","ADC_CLK","ADCCLK"},
	{"ASI3_BCLK Route","ADC_MOD_CLK","ADC_MOD_CLK"},

	{"ASI1_BCLK", NULL, "ASI1_BCLK Route"},
	{"ASI2_BCLK", NULL, "ASI2_BCLK Route"},
	{"ASI3_BCLK", NULL, "ASI3_BCLK Route"},


	{"ASI1IN", NULL , "PLLCLK"},
	{"ASI1IN", NULL , "DACCLK"},
	{"ASI1IN", NULL , "ADCCLK"},
	{"ASI1IN", NULL , "DAC_MOD_CLK"},
	{"ASI1IN", NULL , "ADC_MOD_CLK"},

	{"ASI1OUT", NULL , "PLLCLK"},
	{"ASI1OUT", NULL , "DACCLK"},
	{"ASI1OUT", NULL , "ADCCLK"},
	{"ASI1OUT", NULL , "DAC_MOD_CLK"},
	{"ASI1OUT", NULL , "ADC_MOD_CLK"},
#ifdef AIC3262_ASI1_MASTER
	{"ASI1IN", NULL , "ASI1_BCLK"},
	{"ASI1OUT", NULL , "ASI1_BCLK"},
	{"ASI1IN", NULL , "ASI1_WCLK"},
	{"ASI1OUT", NULL , "ASI1_WCLK"},
#else

#endif

	{"ASI2IN", NULL , "PLLCLK"},
	{"ASI2IN", NULL , "DACCLK"},
	{"ASI2IN", NULL , "ADCCLK"},
	{"ASI2IN", NULL , "DAC_MOD_CLK"},
	{"ASI2IN", NULL , "ADC_MOD_CLK"},

	{"ASI2OUT", NULL , "PLLCLK"},
	{"ASI2OUT", NULL , "DACCLK"},
	{"ASI2OUT", NULL , "ADCCLK"},
	{"ASI2OUT", NULL , "DAC_MOD_CLK"},
	{"ASI2OUT", NULL , "ADC_MOD_CLK"},

#ifdef AIC3262_ASI2_MASTER
	{"ASI2IN", NULL , "ASI2_BCLK"},
	{"ASI2OUT", NULL , "ASI2_BCLK"},
	{"ASI2IN", NULL , "ASI2_WCLK"},
	{"ASI2OUT", NULL , "ASI2_WCLK"},
#else

#endif
	{"ASI3IN", NULL , "PLLCLK"},
	{"ASI3IN", NULL , "DACCLK"},
	{"ASI3IN", NULL , "ADCCLK"},
	{"ASI3IN", NULL , "DAC_MOD_CLK"},
	{"ASI3IN", NULL , "ADC_MOD_CLK"},


	{"ASI3OUT", NULL , "PLLCLK"},
	{"ASI3OUT", NULL , "DACCLK"},
	{"ASI3OUT", NULL , "ADCCLK"},
	{"ASI3OUT", NULL , "DAC_MOD_CLK"},
	{"ASI3OUT", NULL , "ADC_MOD_CLK"},

#ifdef AIC3262_ASI3_MASTER
	{"ASI3IN", NULL , "ASI3_BCLK"},
	{"ASI3OUT", NULL , "ASI3_BCLK"},
	{"ASI3IN", NULL , "ASI3_WCLK"},
	{"ASI3OUT", NULL , "ASI3_WCLK"},
#else
#endif

/* Playback (DAC) Portion */
	{"HPL Output Mixer","LDAC Switch","Left DAC"},
	{"HPL Output Mixer","MAL Switch","MAL PGA"},
	{"HPL Output Mixer","LOL-B1 Volume","LOL"},

	{"HPR Output Mixer","LOR-B1 Volume","LOR"},
	{"HPR Output Mixer","LDAC Switch","Left DAC"},
	{"HPR Output Mixer","RDAC Switch","Right DAC"},
	{"HPR Output Mixer","MAR Switch","MAR PGA"},

	{"HPL Driver",NULL,"HPL Output Mixer"},
	{"HPR Driver",NULL,"HPR Output Mixer"},

	{"HPL",NULL,"HPL Driver"},
	{"HPR",NULL,"HPR Driver"},

	{"LOL Output Mixer","MAL Switch","MAL PGA"},
	{"LOL Output Mixer","IN1L-B Switch","IN1L"},
	{"LOL Output Mixer","LDAC Switch","Left DAC"},
	{"LOL Output Mixer","RDAC Switch","Right DAC"},

	{"LOR Output Mixer","LOL Switch","LOL"},
	{"LOR Output Mixer","RDAC Switch","Right DAC"},
	{"LOR Output Mixer","MAR Switch","MAR PGA"},
	{"LOR Output Mixer","IN1R-B Switch","IN1R"},

	{"LOL Driver",NULL,"LOL Output Mixer"},
	{"LOR Driver",NULL,"LOR Output Mixer"},

	{"LOL",NULL,"LOL Driver"},
	{"LOR",NULL,"LOR Driver"},

	{"REC Output Mixer","LOL-B2 Volume","LOL"},
	{"REC Output Mixer","IN1L Volume","IN1L"},
	{"REC Output Mixer","IN1R Volume","IN1R"},
	{"REC Output Mixer","LOR-B2 Volume","LOR"},

	{"RECP Driver",NULL,"REC Output Mixer"},
	{"RECM Driver",NULL,"REC Output Mixer"},

	{"RECP",NULL,"RECP Driver"},
	{"RECM",NULL,"RECM Driver"},

	{"SPKL Output Mixer","MAL Switch","MAL PGA"},
	{"SPKL Output Mixer","LOL Volume","LOL"},
	{"SPKL Output Mixer","SPR_IN Switch","SPKR Output Mixer"},

	{"SPKR Output Mixer", "LOR Volume","LOR"},
	{"SPKR Output Mixer", "MAR Switch","MAR PGA"},


	{"SPKL Driver",NULL,"SPKL Output Mixer"},
	{"SPKR Driver",NULL,"SPKR Output Mixer"},

	{"SPKL",NULL,"SPKL Driver"},
	{"SPKR",NULL,"SPKR Driver"},
/* ASI Input routing */
	{"ASI1LIN", NULL, "ASI1IN"},
	{"ASI1RIN", NULL, "ASI1IN"},
	{"ASI2LIN", NULL, "ASI2IN"},
	{"ASI2RIN", NULL, "ASI2IN"},
	{"ASI3LIN", NULL, "ASI3IN"},
	{"ASI3RIN", NULL, "ASI3IN"},

	{"ASI1MonoMixIN", NULL, "ASI1IN"},
	{"ASI2MonoMixIN", NULL, "ASI2IN"},
	{"ASI3MonoMixIN", NULL, "ASI3IN"},

	{"ASI1LIN Route","ASI1 Left In","ASI1LIN"},
	{"ASI1LIN Route","ASI1 Right In","ASI1RIN"},
	{"ASI1LIN Route","ASI1 MonoMix In","ASI1MonoMixIN"},

	{"ASI1RIN Route", "ASI1 Right In","ASI1RIN"},
	{"ASI1RIN Route","ASI1 Left In","ASI1LIN"},
	{"ASI1RIN Route","ASI1 MonoMix In","ASI1MonoMixIN"},


	{"ASI2LIN Route","ASI2 Left In","ASI2LIN"},
	{"ASI2LIN Route","ASI2 Right In","ASI2RIN"},
	{"ASI2LIN Route","ASI2 MonoMix In","ASI2MonoMixIN"},

	{"ASI2RIN Route","ASI2 Right In","ASI2RIN"},
	{"ASI2RIN Route","ASI2 Left In","ASI2LIN"},
	{"ASI2RIN Route","ASI2 MonoMix In","ASI2MonoMixIN"},


	{"ASI3LIN Route","ASI3 Left In","ASI3LIN"},
	{"ASI3LIN Route","ASI3 Right In","ASI3RIN"},
	{"ASI3LIN Route","ASI3 MonoMix In","ASI3MonoMixIN"},

	{"ASI3RIN Route","ASI3 Right In","ASI3RIN"},
	{"ASI3RIN Route","ASI3 Left In","ASI3LIN"},
	{"ASI3RIN Route","ASI3 MonoMix In","ASI3MonoMixIN"},

	{"ASI1IN Port", NULL, "ASI1LIN Route"},
	{"ASI1IN Port", NULL, "ASI1RIN Route"},
	{"ASI2IN Port", NULL, "ASI2LIN Route"},
	{"ASI2IN Port", NULL, "ASI2RIN Route"},
	{"ASI3IN Port", NULL, "ASI3LIN Route"},
	{"ASI3IN Port", NULL, "ASI3RIN Route"},

	{"DAC MiniDSP IN1 Route", "ASI1 In","ASI1IN Port"},
	{"DAC MiniDSP IN1 Route","ASI2 In","ASI2IN Port"},
	{"DAC MiniDSP IN1 Route","ASI3 In","ASI3IN Port"},
	{"DAC MiniDSP IN1 Route","ADC MiniDSP Out","ADC MiniDSP OUT1"},

	{"DAC MiniDSP IN2 Route","ASI1 In","ASI1IN Port"},
	{"DAC MiniDSP IN2 Route","ASI2 In","ASI2IN Port"},
	{"DAC MiniDSP IN2 Route","ASI3 In","ASI3IN Port"},

	{"DAC MiniDSP IN3 Route","ASI1 In","ASI1IN Port"},
	{"DAC MiniDSP IN3 Route","ASI2 In","ASI2IN Port"},
	{"DAC MiniDSP IN3 Route","ASI3 In","ASI3IN Port"},

	{"Left DAC", "NULL", "DAC MiniDSP IN1 Route"},
	{"Right DAC", "NULL", "DAC MiniDSP IN1 Route"},

	{"Left DAC", "NULL","DAC MiniDSP IN2 Route"},
	{"Right DAC", "NULL","DAC MiniDSP IN2 Route"},

	{"Left DAC", "NULL","DAC MiniDSP IN3 Route"},
	{"Right DAC", "NULL","DAC MiniDSP IN3 Route"},


/* Mixer Amplifier */

	{"MAL PGA Mixer", "IN1L Switch","IN1L"},
	{"MAL PGA Mixer", "Left MicPGA Volume","Left MicPGA"},

	{"MAL PGA", NULL, "MAL PGA Mixer"},


	{"MAR PGA Mixer", "IN1R Switch","IN1R"},
	{"MAR PGA Mixer", "Right MicPGA Volume","Right MicPGA"},

	{"MAR PGA", NULL, "MAR PGA Mixer"},


/* Capture (ADC) portions */
	/* Left Positive PGA input */
	{"Left Input Mixer","IN1L Switch","IN1L"},
	{"Left Input Mixer","IN2L Switch","IN2L"},
	{"Left Input Mixer","IN3L Switch","IN3L"},
	{"Left Input Mixer","IN4L Switch","IN4L"},
	{"Left Input Mixer","IN1R Switch","IN1R"},
	/* Left Negative PGA input */
	{"Left Input Mixer","IN2R Switch","IN2R"},
	{"Left Input Mixer","IN3R Switch","IN3R"},
	{"Left Input Mixer","IN4R Switch","IN4R"},
	{"Left Input Mixer","CM2L Switch","CM2L"},
	{"Left Input Mixer","CM1L Switch","CM1L"},

	/* Right Positive PGA Input */
	{"Right Input Mixer","IN1R Switch","IN1R"},
	{"Right Input Mixer","IN2R Switch","IN2R"},
	{"Right Input Mixer","IN3R Switch","IN3R"},
	{"Right Input Mixer","IN4R Switch","IN4R"},
	{"Right Input Mixer","IN2L Switch","IN2L"},

	/* Right Negative PGA Input */
	{"Right Input Mixer","IN1L Switch","IN1L"},
	{"Right Input Mixer","IN3L Switch","IN3L"},
	{"Right Input Mixer","IN4L Switch","IN4L"},
	{"Right Input Mixer","CM1R Switch","CM1R"},
	{"Right Input Mixer","CM2R Switch","CM2R"},

	{"CM1L", NULL, "CM"},
	{"CM2L", NULL, "CM"},
	{"CM1R", NULL, "CM"},
	{"CM2R", NULL, "CM"},

	{"Left MicPGA",NULL,"Left Input Mixer"},
	{"Right MicPGA",NULL,"Right Input Mixer"},

	{"Left ADC", NULL, "Left MicPGA"},
	{"Right ADC", NULL, "Right MicPGA"},

/* ASI Output Routing */
	{"ADC MiniDSP OUT1", NULL, "Left ADC"},
	{"ADC MiniDSP OUT1", NULL, "Right ADC"},
	{"ADC MiniDSP OUT2", NULL, "Left ADC"},
	{"ADC MiniDSP OUT2", NULL, "Right ADC"},
	{"ADC MiniDSP OUT3", NULL, "Left ADC"},
	{"ADC MiniDSP OUT3", NULL, "Right ADC"},

	{"ASI1OUT Route", "ASI1 Out","ADC MiniDSP OUT1"},// Port 1
	{"ASI1OUT Route", "ASI1In Bypass","ASI1IN Port"},
	{"ASI1OUT Route", "ASI2In Bypass","ASI2IN Port"},
	{"ASI1OUT Route", "ASI3In Bypass","ASI3IN Port"},

	{"ASI2OUT Route", "ASI1 Out","ADC MiniDSP OUT1"},// Port 1
	{"ASI2OUT Route", "ASI1In Bypass","ASI1IN Port"},
	{"ASI2OUT Route", "ASI2In Bypass","ASI2IN Port"},
	{"ASI2OUT Route", "ASI3In Bypass","ASI3IN Port"},
	{"ASI2OUT Route", "ASI2 Out","ADC MiniDSP OUT2"},// Port 2

	{"ASI3OUT Route", "ASI1 Out","ADC MiniDSP OUT1"},// Port 1
	{"ASI3OUT Route", "ASI1In Bypass","ASI1IN Port"},
	{"ASI3OUT Route", "ASI2In Bypass","ASI2IN Port"},
	{"ASI3OUT Route", "ASI3In Bypass","ASI3IN Port"},
	{"ASI3OUT Route", "ASI3 Out","ADC MiniDSP OUT3"},// Port 3

	{"ASI1OUT",NULL,"ASI1OUT Route"},
	{"ASI2OUT",NULL,"ASI2OUT Route"},
	{"ASI3OUT",NULL,"ASI3OUT Route"},

};


#define AIC3262_DAPM_ROUTE_NUM (sizeof(aic3262_dapm_routes)/sizeof(struct snd_soc_dapm_route))

/*
 *****************************************************************************
 * Function Definitions
 *****************************************************************************
 */


/*
 *----------------------------------------------------------------------------
 * Function : aic3262_change_page
 * Purpose  : This function is to switch between page 0 and page 1.
 *
 *----------------------------------------------------------------------------
 */
int aic3262_change_page(struct snd_soc_codec *codec, u8 new_page)
{
	struct aic3262_priv *aic3262 = snd_soc_codec_get_drvdata(codec);
	u8 data[2];
	int ret = 0;

	data[0] = 0;
	data[1] = new_page;
	aic3262->page_no = new_page;

#if defined(LOCAL_REG_ACCESS)
	if (codec->hw_write(codec->control_data, data, 2) != 2)
		ret = -EIO;
#else
	ret = snd_soc_write(codec, data[0], data[1]);
#endif
	if (ret)
		printk(KERN_ERR "Error in changing page to %d\n", new_page);

	/*DBG("# Changing page to %d\r\n", new_page);*/

	return ret;
}
/*
 *----------------------------------------------------------------------------
 * Function : aic3262_change_book
 * Purpose  : This function is to switch between books
 *
 *----------------------------------------------------------------------------
 */
int aic3262_change_book(struct snd_soc_codec *codec, u8 new_book)
{
	struct aic3262_priv *aic3262 = snd_soc_codec_get_drvdata(codec);
	u8 data[2];
	int ret = 0;

	data[0] = 0x7F;
	data[1] = new_book;
	aic3262->book_no = new_book;

	ret = aic3262_change_page(codec, 0);
	if (ret)
		return ret;

#if defined(LOCAL_REG_ACCESS)
	if (codec->hw_write(codec->control_data, data, 2) != 2)
		ret = -EIO;
#else
	ret = snd_soc_write(codec, data[0], data[1]);
#endif
	if (ret)
		printk(KERN_ERR "Error in changing Book\n");

	/*DBG("# Changing book to %d\r\n", new_book);*/

	return ret;
}
/*
 *----------------------------------------------------------------------------
 * Function : aic3262_write_reg_cache
 * Purpose  : This function is to write aic3262 register cache
 *
 *----------------------------------------------------------------------------
 */
void aic3262_write_reg_cache(struct snd_soc_codec *codec,
					   u16 reg, u8 value)
{
#if defined(EN_REG_CACHE)
	u8 *cache = codec->reg_cache;

	if (reg >= AIC3262_CACHEREGNUM)
		return;

	if (cache)
		cache[reg] = value;
#endif
}

/*
 *----------------------------------------------------------------------------
 * Function : aic3262_read
 * Purpose  : This function is to read the aic3262 register space.
 *
 *----------------------------------------------------------------------------
 */

u8 aic3262_read(struct snd_soc_codec *codec, u16 reg)
{
	struct aic3262_priv *aic3262 = snd_soc_codec_get_drvdata(codec);
	u8 value;
	u8 page = reg / 128;
	u16 *cache = codec->reg_cache;
	u16 cmd;
	u8 buffer[2];
	int rc;
	reg = reg % 128;

	if (reg >= AIC3262_CACHEREGNUM) {
		return 0;
	}

	if (aic3262->control_type == SND_SOC_I2C) {
		if (aic3262->page_no != page) {
		aic3262_change_page(codec, page);
		}
		i2c_master_send(codec->control_data, (char *)&reg, 1);
		i2c_master_recv(codec->control_data, &value, 1);
		/*DBG("r %2x %02x\r\n", reg, value); */
	} else if (aic3262->control_type == SND_SOC_SPI) {
		u16 value;

		/* Do SPI transfer; first 16bits are command; remaining is
		 * register contents */
		cmd = AIC3262_READ_COMMAND_WORD(reg);
		buffer[0] = (cmd >> 8) & 0xff;
		buffer[1] = cmd & 0xff;
		//rc = spi_write_then_read(aic3262->spi, buffer, 2, buffer, 2);

		if (rc) {
			dev_err(&aic3262->spi->dev, "AIC26 reg read error\n");
			return -EIO;
		}
		value = (buffer[0] << 8) | buffer[1];
	} else {
		printk(KERN_ERR "Unknown Interface Type in aic3262_read\n");
	}

	/* Update the cache before returning with the value */
	cache[reg] = value;
	return value;

}

/*
 *----------------------------------------------------------------------------
 * Function : aic3262_write
 * Purpose  : This function is to write to the aic3262 register space.
 *
 *----------------------------------------------------------------------------
 */
int aic3262_write(struct snd_soc_codec *codec, u16 reg, u8 value)
{
	struct aic3262_priv *aic3262 = snd_soc_codec_get_drvdata(codec);
	u8 data[2];
	u8 page;
	int ret = 0;

	page = reg / 128;
	data[AIC3262_REG_OFFSET_INDEX] = reg % 128;
	if (aic3262->page_no != page)
		aic3262_change_page(codec, page);

	/* data is
	 *   D15..D8 aic3262 register offset
	 *   D7...D0 register data
	 */
	data[AIC3262_REG_DATA_INDEX] = value & AIC3262_8BITS_MASK;
#if defined(EN_REG_CACHE)
	if ((page >= 0) & (page <= 4))
		aic3262_write_reg_cache(codec, reg, value);

#endif
	if (!data[AIC3262_REG_OFFSET_INDEX]) {
		/* if the write is to reg0 update aic3262->page_no */
		aic3262->page_no = value;
	}

	/*DBG("w %2x %02x\r\n",
		data[AIC3262_REG_OFFSET_INDEX], data[AIC3262_REG_DATA_INDEX]);*/

#if defined(LOCAL_REG_ACCESS)
	if (codec->hw_write(codec->control_data, data, 2) != 2)
		ret = -EIO;
#else
	ret = snd_soc_write(codec, data[AIC3262_REG_OFFSET_INDEX],
			data[AIC3262_REG_DATA_INDEX]);
#endif
	if (ret)
		printk(KERN_ERR "Error in i2c write\n");

	return ret;
}

/*
 *------------------------------------------------------------------------------
 * Function : aic3262_write__
 * Purpose  : This function is to write to the aic3262 register space.
 *            (low level).
 *------------------------------------------------------------------------------
 */

int aic3262_write__(struct i2c_client *client, const char *buf, int count)
{
	u8 data[3];
	int ret;
	data[0] = *buf;
	data[1] = *(buf+1);
	data[2] = *(buf+2);
	/*DBG("w %2x %02x\r\n",
		data[AIC3262_REG_OFFSET_INDEX], data[AIC3262_REG_DATA_INDEX]);*/
	ret = i2c_master_send(client, data, 2);
	if (ret < 2) {
		printk(
		KERN_ERR "I2C write Error : bytes written = %d\n\n", ret);
		return -EIO;
	}

	return ret;
}
/*
 *----------------------------------------------------------------------------
 * Function : aic3262_reset_cache
 * Purpose  : This function is to reset the cache.
 *----------------------------------------------------------------------------
 */
int aic3262_reset_cache(struct snd_soc_codec *codec)
{
#if defined(EN_REG_CACHE)
	if (codec->reg_cache) {
		memcpy(codec->reg_cache, aic3262_reg, sizeof(aic3262_reg));
		return 0;
	}

	codec->reg_cache = kmemdup(aic3262_reg,
			sizeof(aic3262_reg), GFP_KERNEL);
	if (!codec->reg_cache) {
		printk(KERN_ERR "aic32x4: kmemdup failed\n");
		return -ENOMEM;
	}
#endif
	return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : aic3262_get_divs
 * Purpose  : This function is to get required divisor from the "aic3262_divs"
 *            table.
 *
 *----------------------------------------------------------------------------
 */
static inline int aic3262_get_divs(int mclk, int rate)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(aic3262_divs); i++) {
		if ((aic3262_divs[i].rate == rate)
		    && (aic3262_divs[i].mclk == mclk)) {
			DBG(KERN_INFO "#%s: Found Entry %d in Clock_Array\n",
				__func__, i);
			return i;
		}
	}
	printk(KERN_ERR "Master clock and sample rate is not supported\n");
	return -EINVAL;
}

/*
 *----------------------------------------------------------------------------
 * Function : aic3262_add_widgets
 * Purpose  : This function is to add the dapm widgets
 *            The following are the main widgets supported
 *                # Left DAC to Left Outputs
 *                # Right DAC to Right Outputs
 *		  # Left Inputs to Left ADC
 *		  # Right Inputs to Right ADC
 *
 *----------------------------------------------------------------------------
 */
static int aic3262_add_widgets(struct snd_soc_codec *codec)
{
	int ret;
	struct snd_soc_dapm_context *dapm = &codec->dapm;
#ifndef AIC3262_MULTI_I2S
	int i;
	for (i = 0; i < ARRAY_SIZE(aic3262_dapm_widgets); i++)
		ret = snd_soc_dapm_new_control(dapm, &aic3262_dapm_widgets[i]);
#else
	ret = snd_soc_dapm_new_controls(dapm, aic3262_dapm_widgets,
			ARRAY_SIZE(aic3262_dapm_widgets));
	if (ret != 0) {
		printk(KERN_ERR "#%s: Unable to add DAPM Controls. Err %d\n",
				 __func__, ret);
	}
#endif
	/* set up audio path interconnects */
	DBG("#Completed adding new dapm widget controls size=%d\n",
		ARRAY_SIZE(aic3262_dapm_widgets));
	snd_soc_dapm_add_routes(dapm, aic3262_dapm_routes,
				ARRAY_SIZE(aic3262_dapm_routes));
	DBG("#Completed adding DAPM routes\n");
	snd_soc_dapm_new_widgets(dapm);
	DBG("#Completed updating dapm\n");
	return 0;
}
/*
 *----------------------------------------------------------------------------
 * Function : reg_def_conf
 * Purpose  : This function is to reset the codec book 0 registers
 *
 *----------------------------------------------------------------------------
 */
int reg_def_conf(struct snd_soc_codec *codec)
{
	int i = 0, ret;
	DBG(KERN_INFO "#%s: Invoked..\n", __func__);

	ret = aic3262_change_page(codec, 0);
	if (ret != 0)
		return ret;

	ret = aic3262_change_book(codec, 0);
	if (ret != 0)
		return ret;

	/* Configure the Codec with the default Initialization Values */
	for (i = 0; i < reg_init_size; i++) {
		ret = snd_soc_write(codec, aic3262_reg_init[i].reg_offset,
			aic3262_reg_init[i].reg_val);
		if (ret)
			break;
	}
	DBG(KERN_INFO "#%s: Done..\n", __func__);
	return ret;
}

/*
 * i2c_verify_book0
 *
 * This function is used to dump the values of the Book 0 Pages.
 */
int i2c_verify_book0(struct snd_soc_codec *codec)
{
	int i, j, k = 0;
	u8 val1;

	DBG("starting i2c_verify\n");
	DBG("Resetting page to 0\n");
	aic3262_change_book(codec, 0);
	for (j = 0; j < 3; j++) {
		if (j == 0) {
			aic3262_change_page(codec, 0);
			k = 0;
		}
		if (j == 1) {
			aic3262_change_page(codec, 1);
			k = 1;
		}
		/*
		if (j == 2) {
			aic3262_change_page(codec, 4);
			k = 4;
		}*/
		for (i = 0; i <= 127; i++) {
#if defined(LOCAL_REG_ACCESS)
			val1 = i2c_smbus_read_byte_data(codec->control_data, i);
#else
			val1 = snd_soc_read(codec, i);
#endif
			/* printk("[%d][%d]=[0x%2x]\n",k,i,val1); */
		}
	}
	return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : aic3262_set_bias_level
 * Purpose  : This function is to get triggered when dapm events occurs.
 *
 *----------------------------------------------------------------------------
 */

static int aic3262_set_bias_level(struct snd_soc_codec *codec,
			enum snd_soc_bias_level level)
{
	struct aic3262_priv *aic3262 = snd_soc_codec_get_drvdata(codec);
	u8 value;
	switch (level) {
		/* full On */
	case SND_SOC_BIAS_ON:

		/* all power is driven by DAPM system */
		dev_dbg(codec->dev, "set_bias_on\n");
		break;

		/* partial On */
	case SND_SOC_BIAS_PREPARE:

		dev_dbg(codec->dev, "set_bias_prepare\n");

		break;

	/* Off, with power */
	case SND_SOC_BIAS_STANDBY:
		/*
		 * all power is driven by DAPM system,
		 * so output power is safe if bypass was set
		 */
		 dev_dbg(codec->dev, "set_bias_stby\n");

		break;
	/* Off, without power */
	case SND_SOC_BIAS_OFF:
		 dev_dbg(codec->dev, "set_bias_off\n");

		break;
	}
	codec->dapm.bias_level=level;

	return 0;
}


/*
 *----------------------------------------------------------------------------
 * Function : aic3262_suspend
 * Purpose  : This function is to suspend the AIC3262 driver.
 *
 *----------------------------------------------------------------------------
 */
static int aic3262_suspend(struct snd_soc_codec *codec, pm_message_t state)
{
	struct aic3262_priv *aic3262 = snd_soc_codec_get_drvdata(codec);
	DBG(KERN_INFO "#%s: Invoked..\n", __func__);
	if (aic3262)
		disable_irq(aic3262->irq);

	aic3262_set_bias_level(codec, SND_SOC_BIAS_STANDBY);

	return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : aic3262_resume
 * Purpose  : This function is to resume the AIC3262 driver
 *
 *----------------------------------------------------------------------------
 */
static int aic3262_resume(struct snd_soc_codec *codec)
{
	int i;
	u8 data[2];
	int ret = 0;
	struct aic3262_priv *aic3262 = snd_soc_codec_get_drvdata(codec);
	u8 *cache = codec->reg_cache;
	DBG(KERN_INFO "#%s: Invoked..\n", __func__);

	ret = aic3262_change_page(codec, 0);
	if (ret)
		return ret;
#if defined(EN_REG_CACHE)
	/* Sync reg_cache with the hardware */
	for (i = 0; i < ARRAY_SIZE(aic3262_reg); i++) {
		data[0] = i % 128;
		data[1] = cache[i];
#if defined(LOCAL_REG_ACCESS)
		codec->hw_write(codec->control_data, data, 2);
#else
		ret = snd_soc_write(codec, data[0], data[1]);
		if (ret)
			break;
#endif
	}
#endif
	if (!ret) {
		aic3262_change_page(codec, 0);
		aic3262_set_bias_level(codec, SND_SOC_BIAS_ON);

		if (aic3262)
			enable_irq(aic3262->irq);
	}
	return ret;
}
/*
 *----------------------------------------------------------------------------
 * Function : aic3262_hw_read
 * Purpose  : This is a low level harware read function.
 *
 *----------------------------------------------------------------------------
 */
unsigned int aic3262_hw_read(struct snd_soc_codec *codec, unsigned int count)
{
	struct i2c_client *client = codec->control_data;
	unsigned int buf;

	if (count > (sizeof(unsigned int)))
		return 0;

	i2c_master_recv(client, (char *)&buf, count);
	return buf;
}

/*
* aic3262_jack_handler
*
* This function is called from the Interrupt Handler
* to check the status of the AIC3262 Registers related to Headset Detection
*/
static irqreturn_t aic3262_jack_handler(int irq, void *data)
{
	struct snd_soc_codec *codec = data;
	struct aic3262_priv *aic3262 = snd_soc_codec_get_drvdata(codec);
	unsigned int value;
	unsigned int micbits, hsbits = 0;
	DBG(KERN_INFO "%s++\n", __func__);

	aic3262_change_page(codec, 0);

	/* Read the Jack Status Register*/
	value = snd_soc_read(codec, STICKY_FLAG2);
	DBG(KERN_INFO "reg44 0x%x\n", value);


	value = snd_soc_read(codec, INT_FLAG2);
	DBG(KERN_INFO "reg46 0x%x\n", value);

	value = snd_soc_read(codec, DAC_FLAG_R1);
	DBG(KERN_INFO "reg37 0x%x\n", value);

	micbits = value & DAC_FLAG_MIC_MASKBITS;
	DBG(KERN_INFO "micbits 0x%x\n", micbits);

	hsbits = value & DAC_FLAG_HS_MASKBITS;
	DBG(KERN_INFO "hsbits 0x%x\n", hsbits);


	/* No Headphone or Headset*/
	if (!micbits && !hsbits) {
		DBG(KERN_INFO "no headset/headphone\n");
		snd_soc_jack_report(aic3262->headset_jack,
				0, SND_JACK_HEADSET);
	}

	/* Headphone Detected */
	if ((micbits == DAC_FLAG_R1_NOMIC) || (hsbits)) {
		DBG(KERN_INFO "headphone\n");
		snd_soc_jack_report(aic3262->headset_jack,
				SND_JACK_HEADPHONE, SND_JACK_HEADSET);
	}

	/* Headset Detected - only with capless */
	if (micbits == DAC_FLAG_R1_MIC) {
		DBG(KERN_INFO "headset\n");
		snd_soc_jack_report(aic3262->headset_jack,
				SND_JACK_HEADSET, SND_JACK_HEADSET);
	}
	DBG(KERN_INFO "%s--\n", __func__);
	return IRQ_HANDLED;
}

/*
* aic326x_headset_detect
*
* Call-back function called to check the status of Headset Pin.
*/
int aic326x_headset_detect(struct snd_soc_codec *codec,
	struct snd_soc_jack *jack, int jack_type)
{
	struct aic3262_priv *aic3262 = snd_soc_codec_get_drvdata(codec);

	aic3262->headset_jack = jack;
	/*Enable the Headset Interrupts*/
	snd_soc_write(codec, INT1_CNTL, 0x80);

	return 0;
}
EXPORT_SYMBOL_GPL(aic326x_headset_detect);

int aic326x_headset_button_init(struct snd_soc_codec *codec,
	struct snd_soc_jack *jack, int jack_type)
{
	struct aic3262_priv *aic3262 = snd_soc_codec_get_drvdata(codec);

	aic3262->button_dev = input_allocate_device();
	aic3262->button_dev->name = "aic326x_headset_button";
       aic3262->button_dev->phys = "codec/input0";
	aic3262->button_dev->dev.parent = snd_card_get_device_link(codec->card->snd_card);
	input_set_capability(aic3262->button_dev, EV_KEY, KEY_MEDIA);

	if (input_register_device(aic3262->button_dev))
	{
	   printk( "Unable to register input device headset button");
	}

	aic3262_jack_handler(aic3262->irq, codec);
	return 0;
}
#ifdef AIC3262_MULTI_I2S
/*
* aic3262_asi_default_config
*
* This function is used to perform the default pin configurations for
* the functionalities which are specific to each ASI Port of the AIC3262
* Audio Codec Chipset. The user is encouraged to change these values
* if required on their platforms.
*/
static void aic3262_asi_default_config(struct snd_soc_codec *codec)
{
	struct aic3262_priv *aic3262 = snd_soc_codec_get_drvdata(codec);
	u16 counter;

	DBG(KERN_INFO
		"#%s: Invoked. Will Config ASI Registers to Defaults..\n",
			__func__);
	for (counter = 0; counter < MAX_ASI_COUNT; counter++) {
		aic3262->asiCtxt[counter].asi_active = 0;
		aic3262->asiCtxt[counter].bclk_div = 1;
		aic3262->asiCtxt[counter].wclk_div = 1;
		aic3262->asiCtxt[counter].port_muted = 1;
		aic3262->asiCtxt[counter].bclk_div_option =
			BDIV_CLKIN_DAC_MOD_CLK;
		aic3262->asiCtxt[counter].offset1 = 0;
		aic3262->asiCtxt[counter].offset2 = 0;
	}
	/* ASI1 Defaults */
	aic3262->asiCtxt[0].bclk_output = ASI1_BCLK_DIVIDER_OUTPUT;
	aic3262->asiCtxt[0].wclk_output = GENERATED_DAC_FS;
	aic3262->asiCtxt[0].left_dac_output  = DAC_PATH_LEFT;
	aic3262->asiCtxt[0].right_dac_output = DAC_PATH_LEFT;
	aic3262->asiCtxt[0].adc_input        = ADC_PATH_MINIDSP_1;
	aic3262->asiCtxt[0].dout_option      = ASI_OUTPUT;

	/* ASI2 Defaults */
	aic3262->asiCtxt[1].bclk_output = ASI2_BCLK_DIVIDER_OUTPUT;
	aic3262->asiCtxt[1].wclk_output = GENERATED_DAC_FS;
	aic3262->asiCtxt[1].left_dac_output  = DAC_PATH_LEFT;
	aic3262->asiCtxt[1].right_dac_output = DAC_PATH_LEFT;
	aic3262->asiCtxt[1].adc_input        = ADC_PATH_MINIDSP_2;
	aic3262->asiCtxt[1].dout_option      = ASI_OUTPUT;

	/* ASI3 Defaults */
	aic3262->asiCtxt[2].bclk_output = ASI3_BCLK_DIVIDER_OUTPUT;
	aic3262->asiCtxt[2].wclk_output = GENERATED_DAC_FS;
	aic3262->asiCtxt[2].left_dac_output  = DAC_PATH_LEFT;
	aic3262->asiCtxt[2].right_dac_output = DAC_PATH_LEFT;
	aic3262->asiCtxt[2].adc_input        = ADC_PATH_MINIDSP_3;
	aic3262->asiCtxt[2].dout_option      = ASI2_INPUT;
	return;
}

#endif /* #ifdef AIC3262_MULTI_I2S */

/*
 *----------------------------------------------------------------------------
 * Function : aic3262_probe
 * Purpose  : This is first driver function called by the SoC core driver.
 *
 *----------------------------------------------------------------------------
 */

static int aic3262_probe(struct snd_soc_codec *codec)
{
	struct aic3262_priv *aic3262 = snd_soc_codec_get_drvdata(codec);
	int ret = 0;

	DBG(KERN_INFO "#%s: Invoked..\n", __func__);

#if defined(EN_REG_CACHE)
	codec->reg_cache =
		kmemdup(aic3262_reg, sizeof(aic3262_reg), GFP_KERNEL);

	if (!codec->reg_cache) {
		printk(KERN_ERR "aic3262: kmemdup failed\n");
		return -ENOMEM;
	}
#else
	/* Setting cache bypass - not to overwrite the cache registers,
	Codec registers have 4 pages which is not handled in the common
	cache code properly - bypass it in write value and save it
	using separate call*/
	codec->cache_bypass = 1;
#endif

#if defined(LOCAL_REG_ACCESS)
	codec->control_data = aic3262->control_data;
	codec->hw_write = (hw_write_t) aic3262_write__;
	codec->hw_read = aic3262_hw_read;
#else
	ret = snd_soc_codec_set_cache_io(codec, 8, 8, SND_SOC_I2C);
	if (ret != 0) {
		dev_err(codec->dev, "Failed to set cache I/O: %d\n", ret);
		return ret;
	}
#endif
	ret = reg_def_conf(codec);
	if (ret != 0) {
		printk(KERN_ERR "Failed to init TI codec: %d\n", ret);
		return ret;
	}

	if (aic3262->irq) {
		/* audio interrupt */
		ret = request_threaded_irq(aic3262->irq, NULL,
				aic3262_jack_handler,
				IRQF_TRIGGER_FALLING,
				"tlv320aic3262", codec);
		if (ret) {
			printk(KERN_INFO "#%s: IRQ Registration failed..[%d]",
					__func__, ret);
			dev_err(codec->dev, "Failed to request IRQ: %d\n", ret);
			return ret;
		} else
			DBG(KERN_INFO
				"#%s: irq Registration for IRQ %d done..\n",
					__func__, aic3262->irq);
	} else {
		DBG(KERN_INFO "#%s: I2C IRQ Configuration is Wrong. \
			Please check it..\n", __func__);
	}

	aic3262_asi_default_config(codec);

	/* off, with power on */
	aic3262_set_bias_level(codec, SND_SOC_BIAS_STANDBY);

	ret = snd_soc_add_controls(codec, aic3262_snd_controls,
					ARRAY_SIZE(aic3262_snd_controls));
	if(ret)
	{
		printk(KERN_INFO "%s failed\n", __func__);
	}

	aic3262_add_widgets(codec);
	/*TODO*/
	snd_soc_write(codec, MIC_BIAS_CNTL, 0x66);

#ifdef AIC3262_TiLoad
	ret = aic3262_driver_init(codec);
	if (ret < 0)
		printk(KERN_ERR
	"\nAIC3262 CODEC: aic3262_probe :TiLoad Initialization failed\n");
#endif


#ifdef CONFIG_MINI_DSP
	/* Program MINI DSP for ADC and DAC */
	aic3262_minidsp_program(codec);
	aic3262_add_minidsp_controls(codec);
	aic3262_change_book(codec, 0x0);
#endif

#ifdef MULTIBYTE_CONFIG_SUPPORT
	aic3262_add_multiconfig_controls(codec);
#endif

	DBG(KERN_INFO "#%s: done..\n", __func__);
	return ret;
}



/*
 *----------------------------------------------------------------------------
 * Function : aic3262_remove
 * Purpose  : to remove aic3262 soc device
 *
 *----------------------------------------------------------------------------
 */
static int aic3262_remove(struct snd_soc_codec *codec)
{

	/* power down chip */
	aic3262_set_bias_level(codec, SND_SOC_BIAS_OFF);

	return 0;
}


/*
 *----------------------------------------------------------------------------
 * @struct  snd_soc_codec_device |
 *          This structure is soc audio codec device sturecute which pointer
 *          to basic functions aic3262_probe(), aic3262_remove(),
 *          aic3262_suspend() and aic3262_resume()
 *----------------------------------------------------------------------------
 */
static struct snd_soc_codec_driver soc_codec_dev_aic3262 = {
	.probe = aic3262_probe,
	.remove = aic3262_remove,
	.suspend = aic3262_suspend,
	.resume = aic3262_resume,
	.set_bias_level = aic3262_set_bias_level,
#if defined(LOCAL_REG_ACCESS)
	.read = aic3262_read,
	.write = aic3262_write,
#endif
#if !defined(EN_REG_CACHE)
	.reg_cache_size = ARRAY_SIZE(aic3262_reg),
	.reg_word_size = sizeof(u8),
	.reg_cache_default = aic3262_reg,
#endif
};


#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)
/*
 *----------------------------------------------------------------------------
 * Function : aic3262_codec_probe
 * Purpose  : This function attaches the i2c client and initializes
 *				AIC3262 CODEC.
 *            NOTE:
 *            This function is called from i2c core when the I2C address is
 *            valid.
 *            If the i2c layer weren't so broken, we could pass this kind of
 *            data around
 *
 *----------------------------------------------------------------------------
 */
static __devinit int aic3262_codec_probe(struct i2c_client *i2c,
			const struct i2c_device_id *id)
{
	int ret;

	struct aic3262_priv *aic3262;

	DBG(KERN_INFO "#%s: Entered\n", __func__);

	aic3262 = kzalloc(sizeof(struct aic3262_priv), GFP_KERNEL);

	if (!aic3262) {
		printk(KERN_ERR "#%s: Unable to Allocate Priv struct..\n",
			__func__);
		return -ENOMEM;
	}

	i2c_set_clientdata(i2c, aic3262);
#if defined(LOCAL_REG_ACCESS)
	aic3262->control_data = i2c;
#endif
	aic3262->control_type = SND_SOC_I2C;
	aic3262->irq = i2c->irq;
	aic3262->pdata = i2c->dev.platform_data;

	/* The Configuration Support will be by default to 3 which
	* holds the MAIN Patch Configuration.
	*/
	aic3262->current_dac_config[0] = -1;
	aic3262->current_dac_config[1] = -1;
	aic3262->current_adc_config[0] = -1;
	aic3262->current_adc_config[1] = -1;

	aic3262->mute_codec = 1;

	aic3262->page_no = 0;
	aic3262->book_no = 0;
	aic3262->active_count = 0;
	aic3262->dac_clkin_option = 3;
	aic3262->adc_clkin_option = 3;

	ret = snd_soc_register_codec(&i2c->dev,
		&soc_codec_dev_aic3262,
		tlv320aic3262_dai, ARRAY_SIZE(tlv320aic3262_dai));

	if (ret < 0)
		kfree(aic3262);
	DBG(KERN_INFO "#%s: Done ret %d\n", __func__, ret);
	return ret;
}

/*
 *----------------------------------------------------------------------------
 * Function : aic3262_i2c_remove
 * Purpose  : This function removes the i2c client and uninitializes
 *                              AIC3262 CODEC.
 *            NOTE:
 *            This function is called from i2c core
 *            If the i2c layer weren't so broken, we could pass this kind of
 *            data around
 *
 *----------------------------------------------------------------------------
 */
static __devexit int aic3262_i2c_remove(struct i2c_client *i2c)
{
	snd_soc_unregister_codec(&i2c->dev);
	kfree(i2c_get_clientdata(i2c));
	return 0;
}

static const struct i2c_device_id tlv320aic3262_id[] = {
	{"aic3262-codec", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, tlv320aic3262_id);

static struct i2c_driver tlv320aic3262_i2c_driver = {
	.driver = {
		.name = "aic3262-codec",
		.owner = THIS_MODULE,
	},
	.probe = aic3262_codec_probe,
	.remove = __devexit_p(aic3262_i2c_remove),
	.id_table = tlv320aic3262_id,
};
#endif /*#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)*/

#if defined(CONFIG_SPI_MASTER)
static int aic3262_spi_write(struct spi_device *spi, const char *data, int len)
{
	struct spi_transfer t;
	struct spi_message m;
	u8 msg[2];

	if (len <= 0)
		return 0;

	msg[0] = data[0];
	msg[1] = data[1];

	spi_message_init(&m);
	memset(&t, 0, (sizeof t));
	t.tx_buf = &msg[0];
	t.len = len;

	spi_message_add_tail(&t, &m);
	spi_sync(spi, &m);

	return len;
}

/*
 * This function forces any delayed work to be queued and run.
 */
static int run_delayed_work(struct delayed_work *dwork)
{
	int ret;

	/* cancel any work waiting to be queued. */
	ret = cancel_delayed_work(dwork);

	/* if there was any work waiting then we run it now and
	 * wait for it's completion */
	if (ret) {
		schedule_delayed_work(dwork, 0);
		flush_scheduled_work();
	}
	return ret;
}
static int __devinit aic3262_spi_probe(struct spi_device *spi)
{
	int ret;
	struct snd_soc_codec *codec;
	struct aic3262_priv *aic3262;
	printk(KERN_INFO "%s entering\n",__func__);
	aic3262 = kzalloc(sizeof(struct aic3262_priv), GFP_KERNEL);

	if (!aic3262) {
		printk(KERN_ERR "#%s: Unable to Allocate Priv struct..\n",
			__func__);
		return -ENOMEM;
	}
	codec = &aic3262->codec;
	codec->control_data = spi;
	aic3262->control_type = SND_SOC_SPI;
	codec->hw_write = (hw_write_t)aic3262_spi_write;
	codec->dev = &spi->dev;

	aic3262->pdata = spi->dev.platform_data;

	/* The Configuration Support will be by default to 3 which
	* holds the MAIN Patch Configuration.
	*/
	aic3262->current_dac_config[0] = -1;
	aic3262->current_dac_config[1] = -1;
	aic3262->current_adc_config[0] = -1;
	aic3262->current_adc_config[1] = -1;

	aic3262->mute_codec = 1;

	aic3262->page_no = 0;
	aic3262->book_no = 0;
	aic3262->active_count = 0;
	aic3262->dac_clkin_option = 3;
	aic3262->adc_clkin_option = 3;
	dev_set_drvdata(&spi->dev, aic3262);
	spi_set_drvdata(spi, aic3262);
	ret = snd_soc_register_codec(&spi->dev,
		&soc_codec_dev_aic3262,
		tlv320aic3262_dai, ARRAY_SIZE(tlv320aic3262_dai));

	if (ret < 0) {
		printk(KERN_INFO "%s codec registeration failed\n",__func__);
		kfree(aic3262);
	}
	else {
		printk(KERN_INFO "%s registered\n",__func__);
	}
	printk(KERN_INFO "#%s: Done ret %d\n", __func__, ret);
	return ret;
}

static int __devexit aic3262_spi_remove(struct spi_device *spi)
{
	struct aic3262_priv *aic3262 = dev_get_drvdata(&spi->dev);
	aic3262_set_bias_level(&aic3262->codec, SND_SOC_BIAS_OFF);
	snd_soc_unregister_codec(&spi->dev);
	kfree(aic3262);
	aic3262_codec = NULL;
	return 0;

}

static struct spi_driver aic3262_spi_driver = {
	.driver = {
		.name	= "aic3262-codec",
		.bus	= &spi_bus_type,
		.owner	= THIS_MODULE,
	},
	.probe		= aic3262_spi_probe,
	.remove		= __devexit_p(aic3262_spi_remove),
};
#endif
static int __init tlv320aic3262_modinit(void)
{
	int ret = 0;
	printk(KERN_INFO "In %s\n",__func__);
#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)
	ret = i2c_add_driver(&tlv320aic3262_i2c_driver);
	if (ret != 0)
		printk(KERN_ERR "Failed to register aic326x i2c driver %d\n",
			ret);
#endif
#if defined(CONFIG_SPI_MASTER)
	printk(KERN_INFO "Inside config_spi_master\n");
	ret = spi_register_driver(&aic3262_spi_driver);
	if (ret != 0)
		printk(KERN_ERR "Failed to register aic3262 SPI driver: %d\n", ret);
#endif
	return ret;

}

module_init(tlv320aic3262_modinit);

static void __exit tlv320aic3262_exit(void)
{
#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)
	i2c_del_driver(&tlv320aic3262_i2c_driver);
#endif
}
module_exit(tlv320aic3262_exit);

MODULE_DESCRIPTION("ASoC TLV320AIC3262 codec driver");
MODULE_AUTHOR("Barani Prashanth<gvbarani@mistralsolutions.com>");
MODULE_AUTHOR("Ravindra<ravindra@mistralsolutions.com>");
MODULE_LICENSE("GPL");
