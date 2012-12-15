/*
 * tegra_wm8994.c - Tegra machine ASoC driver for boards using WM8994 codec.
 *
 * Author: Sumit Bhattacharya <sumitb@nvidia.com>
 * Copyright (C) 2010-2011 - NVIDIA, Inc.
 *
 * Based on code copyright/by:
 *
 * (c) 2009, 2010 Nvidia Graphics Pvt. Ltd.
 *
 * Copyright 2007 Wolfson Microelectronics PLC.
 * Author: Graeme Gregory
 *         graeme.gregory@wolfsonmicro.com or linux@wolfsonmicro.com
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */
#define DEBUG

#include <asm/mach-types.h>

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#ifdef CONFIG_SWITCH
#include <linux/switch.h>
#endif

#if defined (CONFIG_MACH_STAR) || defined (CONFIG_MACH_BSSQ)
#include "../../../arch/arm/mach-tegra/clock.h"
#endif

#include <mach/tegra_wm8994_pdata.h>

#include <sound/core.h>
#include <sound/jack.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include "../codecs/wm8994.h"

#include "tegra_pcm.h"
#include "tegra_asoc_utils.h"

#include <linux/wakelock.h>
#ifdef CONFIG_ARCH_TEGRA_2x_SOC
#include "tegra20_das.h"
#endif

#if defined (CONFIG_MACH_STAR) || defined (CONFIG_MACH_BSSQ)
extern unsigned long clk_get_rate(struct clk *c);
#endif

#define DRV_NAME "tegra-snd-wm8994"

#define GPIO_SPKR_EN    BIT(0)
#define GPIO_HP_MUTE    BIT(1)
#define GPIO_INT_MIC_EN BIT(2)
#define GPIO_EXT_MIC_EN BIT(3)

struct headset_switch_data	*headset_sw_data;	//LGE_CHANGE_S [chahee.kim@lge.com] 2011-11-14

extern struct wake_lock headset_wake_lock;  //

#if defined(CONFIG_MACH_STAR) || defined(CONFIG_MACH_BSSQ)
static bool is_call_mode;
bool in_call_state();
#endif // MOBII LP1 sleep

//
static int is_fmradio_mode;
//

struct tegra_wm8994 {
	struct tegra_asoc_utils_data util_data;
	struct tegra_wm8994_platform_data *pdata;
	struct regulator *audio_reg;
	int gpio_requested;

    bool init_done;
	int is_call_mode;
	int is_device_bt;
#ifndef CONFIG_ARCH_TEGRA_2x_SOC
	struct codec_config codec_info[NUM_I2S_DEVICES];
#endif
	enum snd_soc_bias_level bias_level;
	struct snd_soc_card *pcard; // nVidia patch
};

static int tegra_wm8994_hw_params(struct snd_pcm_substream *substream,
					struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_card *card = codec->card;
	struct tegra_wm8994 *machine = snd_soc_card_get_drvdata(card);
	int srate, mclk, i2s_daifmt;
	int err;
    unsigned long cdev_srate;  //heejeong.seo@lge.com 20111128 ICS ap20 wm8994


	srate = params_rate(params);
	switch (srate) {
	case 8000:
	case 16000:
	case 24000:
	case 32000:
	case 48000:
	case 64000:
	case 96000:
		mclk = 12288000;
		break;
	case 11025:
	case 22050:
	case 44100:
	case 88200:
		mclk = 11289600;
		break;
	default:
		mclk = 12000000;
		break;
	}

	err = tegra_asoc_utils_set_rate(&machine->util_data, srate, mclk);
	if (err < 0) {
		if (!(machine->util_data.set_mclk % mclk))
			mclk = machine->util_data.set_mclk;
		else {
			dev_err(card->dev, "Can't configure clocks\n");
			return err;
		}
	}

	tegra_asoc_utils_lock_clk_rate(&machine->util_data, 1);

	i2s_daifmt = SND_SOC_DAIFMT_NB_NF |
		     SND_SOC_DAIFMT_CBS_CFS;

	/* Use DSP mode for mono on Tegra20 */
	if ((params_channels(params) != 2) && machine_is_whistler())
		i2s_daifmt |= SND_SOC_DAIFMT_DSP_A;
	else
		i2s_daifmt |= SND_SOC_DAIFMT_I2S;

	err = snd_soc_dai_set_fmt(codec_dai, i2s_daifmt);
	if (err < 0) {
		dev_err(card->dev, "codec_dai fmt not set\n");
		return err;
	}

	err = snd_soc_dai_set_fmt(cpu_dai, i2s_daifmt);
	if (err < 0) {
		dev_err(card->dev, "cpu_dai fmt not set\n");
		return err;
	}
#if 1 //heejeong.seo@lge.com 20111128 for ICS ap20 wm8994
//function in tegra_i2s.c(ap25+k36), but no function in tegra20_i2s.c(k39)-- Need check
/*
      err = snd_soc_dai_set_sysclk(cpu_dai, 1, params_rate(params), SND_SOC_CLOCK_IN);
	if (err < 0) {
		pr_err("cpu_dai sysclk not set\n");
		return err;
	}
*/
	cdev_srate = clk_get_rate(machine->util_data.clk_cdev1);
	printk(KERN_ERR "tegra_soc_wm8994.c tegra_wm8994_hw_params sys_clk=%d\n",cdev_srate);
#if 0
	err = snd_soc_dai_set_pll(codec_dai, WM8994_FLL1, WM8994_FLL_SRC_MCLK1, cdev_srate, 11289600/* I2S1_CLK*/);
	if (err < 0) {
		pr_err("codec_dai pll not set\n");
		return err;
	}
#endif

#if 1
	err = snd_soc_dai_set_sysclk(codec_dai, WM8994_SYSCLK_MCLK1, /*I2S1_CLK*/cdev_srate, SND_SOC_CLOCK_IN);
#else
	err = snd_soc_dai_set_sysclk(codec_dai, WM8994_SYSCLK_FLL1, 0,SND_SOC_CLOCK_IN);
#endif
	if (err < 0) {
		pr_err("codec_dai sysclk not set\n");
		return err;
	}

#else
	err = snd_soc_dai_set_sysclk(codec_dai, 0, mclk,
					SND_SOC_CLOCK_IN);
	if (err < 0) {
		dev_err(card->dev, "codec_dai clock not set\n");
		return err;
	}
#endif

#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	err = tegra20_das_connect_dac_to_dap(TEGRA20_DAS_DAP_SEL_DAC1,
					TEGRA20_DAS_DAP_ID_1);
	if (err < 0) {
		dev_err(card->dev, "failed to set dap-dac path\n");
		return err;
	}

	err = tegra20_das_connect_dap_to_dac(TEGRA20_DAS_DAP_ID_1,
					TEGRA20_DAS_DAP_SEL_DAC1);
	if (err < 0) {
		dev_err(card->dev, "failed to set dac-dap path\n");
		return err;
	}
#endif
	return 0;
}

//LGE_CHANGE_S [chahee.kim@lge.com] 2012-01-30
static int tegra_voice_call_hw_params(struct snd_pcm_substream *substream,
					struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_card *card = codec->card;
	struct tegra_wm8994 *machine = snd_soc_card_get_drvdata(card);
	int srate, mclk, i2s_daifmt;
	int err;
    unsigned long cdev_srate;  //heejeong.seo@lge.com 20111128 ICS ap20 wm8994


	srate = params_rate(params);
	switch (srate) {
	case 8000:
	case 16000:
	case 24000:
	case 32000:
	case 48000:
	case 64000:
	case 96000:
		mclk = 12288000;
		break;
	case 11025:
	case 22050:
	case 44100:
	case 88200:
		mclk = 11289600;
		break;
	default:
		mclk = 12000000;
		break;
	}

	err = tegra_asoc_utils_set_rate(&machine->util_data, srate, mclk);
	if (err < 0) {
		if (!(machine->util_data.set_mclk % mclk))
			mclk = machine->util_data.set_mclk;
		else {
			dev_err(card->dev, "Can't configure clocks\n");
			return err;
		}
	}

	tegra_asoc_utils_lock_clk_rate(&machine->util_data, 1);

#if defined(CONFIG_MACH_STAR)
	i2s_daifmt = SND_SOC_DAIFMT_IB_NF |
		     SND_SOC_DAIFMT_CBM_CFM;
#elif defined(CONFIG_MACH_BSSQ)
  //LGE_CHANGE_S, bae.cheolhwan@lge.com. 2012-02-27. Modify for call sound.
	i2s_daifmt = SND_SOC_DAIFMT_IB_NF |
		     SND_SOC_DAIFMT_CBS_CFS;
  //LGE_CHANGE_E, bae.cheolhwan@lge.com. 2012-02-27. Modify for call sound.
#endif

	/* Use DSP mode for mono on Tegra20 */
	if ((params_channels(params) != 2))
	{
		i2s_daifmt |= SND_SOC_DAIFMT_DSP_A;
		printk ("[chahee.kim] set DSP(PCM) mode\n");
	}
	else
		i2s_daifmt |= SND_SOC_DAIFMT_I2S;

	err = snd_soc_dai_set_fmt(codec_dai, i2s_daifmt);
	if (err < 0) {
		dev_err(card->dev, "codec_dai fmt not set\n");
		return err;
	}

/*	err = snd_soc_dai_set_fmt(cpu_dai, i2s_daifmt);
	if (err < 0) {
		dev_err(card->dev, "cpu_dai fmt not set\n");
		return err;
	}*/
#if 1 //heejeong.seo@lge.com 20111128 for ICS ap20 wm8994
//function in tegra_i2s.c(ap25+k36), but no function in tegra20_i2s.c(k39)-- Need check
/*
      err = snd_soc_dai_set_sysclk(cpu_dai, 1, params_rate(params), SND_SOC_CLOCK_IN);
	if (err < 0) {
		pr_err("cpu_dai sysclk not set\n");
		return err;
	}
*/
	cdev_srate = clk_get_rate(machine->util_data.clk_cdev1);
	printk(KERN_ERR "tegra_soc_wm8994.c tegra_voice_call_hw_params sys_clk=%d\n",cdev_srate);
#if 0
	err = snd_soc_dai_set_pll(codec_dai, WM8994_FLL1, WM8994_FLL_SRC_MCLK1, cdev_srate, 11289600/* I2S1_CLK*/);
	if (err < 0) {
		pr_err("codec_dai pll not set\n");
		return err;
	}
#endif

	err = snd_soc_dai_set_fmt(cpu_dai,
					SND_SOC_DAIFMT_DSP_A |
					SND_SOC_DAIFMT_NB_NF |
					SND_SOC_DAIFMT_CBM_CFM);
	if (err < 0) {
		dev_err(card->dev, "cpu_dai fmt not set\n");
		return err;
	}

#if 1
	err = snd_soc_dai_set_sysclk(codec_dai, WM8994_SYSCLK_MCLK1, /*I2S1_CLK*/cdev_srate, SND_SOC_CLOCK_IN);
#else
	err = snd_soc_dai_set_pll(codec_dai, WM8994_FLL2, WM8994_FLL_SRC_MCLK1 ,cdev_srate, cdev_srate);
	err = snd_soc_dai_set_sysclk(codec_dai, WM8994_SYSCLK_FLL2, cdev_srate, SND_SOC_CLOCK_IN);

	if (err < 0) {
		pr_err("codec_dai sysclk not set\n");
		return err;
	}
#endif

#else
	err = snd_soc_dai_set_sysclk(codec_dai, 0, mclk,
					SND_SOC_CLOCK_IN);
	if (err < 0) {
		dev_err(card->dev, "codec_dai clock not set\n");
		return err;
	}
#endif

//LGE_CHANGE_S [chahee.kim@lge.com] 2012-02-16
	machine->is_device_bt = 0;
//LGE_CHANGE_E [chahee.kim@lge.com] 2012-02-16
	return 0;
}
//LGE_CHANGE_E [chahee.kim@lge.com] 2012-01-30

static int tegra_bt_sco_hw_params(struct snd_pcm_substream *substream,
					struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_card *card = codec->card;
	struct tegra_wm8994 *machine = snd_soc_card_get_drvdata(card);
	int srate, mclk, min_mclk;
	int err;

	srate = params_rate(params);
	switch (srate) {
	case 11025:
	case 22050:
	case 44100:
	case 88200:
		mclk = 11289600;
		break;
	case 8000:
	case 16000:
	case 32000:
	case 48000:
	case 64000:
	case 96000:
		mclk = 12288000;
		break;
	default:
		return -EINVAL;
	}
	min_mclk = 64 * srate;

	err = tegra_asoc_utils_set_rate(&machine->util_data, srate, mclk);
	if (err < 0) {
		if (!(machine->util_data.set_mclk % min_mclk))
			mclk = machine->util_data.set_mclk;
		else {
			dev_err(card->dev, "Can't configure clocks\n");
			return err;
		}
	}

	tegra_asoc_utils_lock_clk_rate(&machine->util_data, 1);

	err = snd_soc_dai_set_fmt(cpu_dai,
					SND_SOC_DAIFMT_DSP_A |
					SND_SOC_DAIFMT_NB_NF |
					SND_SOC_DAIFMT_CBM_CFM);
	if (err < 0) {
		dev_err(card->dev, "cpu_dai fmt not set\n");
		return err;
	}

#ifdef CONFIG_ARCH_TEGRA_2x_SOC
		err = tegra20_das_connect_dac_to_dap(TEGRA20_DAS_DAP_SEL_DAC2,
						TEGRA20_DAS_DAP_ID_4);
		if (err < 0) {
			dev_err(card->dev, "failed to set dac-dap path\n");
			return err;
		}

		err = tegra20_das_connect_dap_to_dac(TEGRA20_DAS_DAP_ID_4,
						TEGRA20_DAS_DAP_SEL_DAC2);
		if (err < 0) {
			dev_err(card->dev, "failed to set dac-dap path\n");
			return err;
		}
#endif

	machine->is_device_bt = 0;
	return 0;
}

static int tegra_bt_call_hw_params(struct snd_pcm_substream *substream,
					struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_card *card = codec->card;
	struct tegra_wm8994 *machine = snd_soc_card_get_drvdata(card);
	int srate, mclk;
	int err;

	srate = params_rate(params);
	switch (srate) {
	case 8000:
	case 16000:
	case 24000:
	case 32000:
	case 48000:
	case 64000:
	case 96000:
		mclk = 12288000;
		break;
	case 11025:
	case 22050:
	case 44100:
	case 88200:
		mclk = 11289600;
		break;
	default:
		mclk = 12000000;
		break;
	}

	err = tegra_asoc_utils_set_rate(&machine->util_data, srate, mclk);
	if (err < 0) {
		if (!(machine->util_data.set_mclk % mclk))
			mclk = machine->util_data.set_mclk;
		else {
			dev_err(card->dev, "Can't configure clocks\n");
			return err;
		}
	}

	tegra_asoc_utils_lock_clk_rate(&machine->util_data, 1);

	machine->is_device_bt = 1;


		err = snd_soc_dai_set_fmt(cpu_dai,
						SND_SOC_DAIFMT_DSP_A |
						SND_SOC_DAIFMT_NB_NF |
						SND_SOC_DAIFMT_CBM_CFM);
		if (err < 0) {
			dev_err(card->dev, "cpu_dai fmt not set\n");
			return err;
		}

#ifdef CONFIG_ARCH_TEGRA_2x_SOC
			err = tegra20_das_connect_dac_to_dap(TEGRA20_DAS_DAP_SEL_DAC2,
							TEGRA20_DAS_DAP_ID_4);
			if (err < 0) {
				dev_err(card->dev, "failed to set dac-dap path\n");
				return err;
			}

			err = tegra20_das_connect_dap_to_dac(TEGRA20_DAS_DAP_ID_4,
							TEGRA20_DAS_DAP_SEL_DAC2);
			if (err < 0) {
				dev_err(card->dev, "failed to set dac-dap path\n");
				return err;
			}
#endif
	return 0;
}

static int tegra_spdif_hw_params(struct snd_pcm_substream *substream,
					struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_card *card = rtd->card;
	struct tegra_wm8994 *machine = snd_soc_card_get_drvdata(card);
	int srate, mclk, min_mclk;
	int err;

	srate = params_rate(params);
	switch (srate) {
	case 11025:
	case 22050:
	case 44100:
	case 88200:
		mclk = 11289600;
		break;
	case 8000:
	case 16000:
	case 32000:
	case 48000:
	case 64000:
	case 96000:
		mclk = 12288000;
		break;
	default:
		return -EINVAL;
	}
	min_mclk = 128 * srate;

	err = tegra_asoc_utils_set_rate(&machine->util_data, srate, mclk);
	if (err < 0) {
		if (!(machine->util_data.set_mclk % min_mclk))
			mclk = machine->util_data.set_mclk;
		else {
			dev_err(card->dev, "Can't configure clocks\n");
			return err;
		}
	}

	tegra_asoc_utils_lock_clk_rate(&machine->util_data, 1);

	return 0;
}

static int tegra_hw_free(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct tegra_wm8994 *machine = snd_soc_card_get_drvdata(rtd->card);

	tegra_asoc_utils_lock_clk_rate(&machine->util_data, 0);

//LGE_CHANGE_S [chahee.kim@lge.com] 2012-02-16
	machine->is_device_bt = 0;
//LGE_CHANGE_E [chahee.kim@lge.com] 2012-02-16
	return 0;
}

//LGE_CHANGE_S [chahee.kim@lge.com] 2012-02-16
static int tegra_call_mode_info(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 1;
	return 0;
}

static int tegra_call_mode_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct tegra_wm8994 *machine = snd_kcontrol_chip(kcontrol);

	ucontrol->value.integer.value[0] = machine->is_call_mode;

	return 0;
}

static int tegra_call_mode_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct tegra_wm8994 *machine = snd_kcontrol_chip(kcontrol);
	int is_call_mode_new = ucontrol->value.integer.value[0];
	int codec_dap_id, codec_dap_sel, bb_dap_id, bb_dap_sel;
	unsigned int i; // nVidia patch

	if (machine->is_call_mode == is_call_mode_new)
		return 0;

	bb_dap_id = TEGRA20_DAS_DAP_ID_3;
	bb_dap_sel = TEGRA20_DAS_DAP_SEL_DAP3;

	if (machine->is_device_bt) {
		codec_dap_id = TEGRA20_DAS_DAP_ID_4;
		codec_dap_sel = TEGRA20_DAS_DAP_SEL_DAP4;
	}
	else {
		codec_dap_id = TEGRA20_DAS_DAP_ID_2;
		codec_dap_sel = TEGRA20_DAS_DAP_SEL_DAP2;
	}

	if (is_call_mode_new) {
#ifdef CONFIG_ARCH_TEGRA_2x_SOC
		tegra20_das_set_tristate(codec_dap_id, 1);
		tegra20_das_set_tristate(bb_dap_id, 1);
#if defined(CONFIG_MACH_STAR)
		tegra20_das_connect_dap_to_dap(codec_dap_id,
			bb_dap_sel, 1, 0, 0);
		tegra20_das_connect_dap_to_dap(bb_dap_id,
			codec_dap_sel, 0, 0, 0);
#elif defined(CONFIG_MACH_BSSQ)
//LGE_CHANGE_S, bae.cheolhwan@lge.com. 2012-02-27. Modify for call sound.
		tegra20_das_connect_dap_to_dap(codec_dap_id,
			bb_dap_sel, 0, 0, 0);
		tegra20_das_connect_dap_to_dap(bb_dap_id,
			codec_dap_sel, 1, 0, 0);
//LGE_CHANGE_E, bae.cheolhwan@lge.com. 2012-02-27. Modify for call sound.
#endif
		tegra20_das_set_tristate(codec_dap_id, 0);
		tegra20_das_set_tristate(bb_dap_id, 0);
		for (i = 0; i < machine->pcard->num_links; i++) // nVidia patch
			machine->pcard->dai_link[i].ignore_suspend = 1;
		printk ("[chahee.kim] tegra_call_mode_put() is_call_mode_new !!\n");
#endif
	} else {
#ifdef CONFIG_ARCH_TEGRA_2x_SOC
		tegra20_das_set_tristate(codec_dap_id, 1);
		tegra20_das_set_tristate(bb_dap_id, 1);
		tegra20_das_connect_dap_to_dap(bb_dap_id,
			bb_dap_sel, 0, 0, 0);
		tegra20_das_connect_dap_to_dap(codec_dap_id,
			codec_dap_sel, 0, 0, 0);
		tegra20_das_set_tristate(codec_dap_id, 0);
		tegra20_das_set_tristate(bb_dap_id, 0);
		for (i = 0; i < machine->pcard->num_links; i++) // nVidia patch
			machine->pcard->dai_link[i].ignore_suspend = 0;
		printk ("[chahee.kim] tegra_call_mode_put() don't is_call_mode_new !!\n");
#endif
	}

#if defined(CONFIG_MACH_STAR) || defined(CONFIG_MACH_BSSQ)
	is_call_mode = machine->is_call_mode = is_call_mode_new;
#else
	machine->is_call_mode = is_call_mode_new;
#endif // MOBII_E

	return 1;
}

struct snd_kcontrol_new tegra_call_mode_control = {
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.name = "Call Mode Switch",
	.private_value = 0xffff,
	.info = tegra_call_mode_info,
	.get = tegra_call_mode_get,
	.put = tegra_call_mode_put
};

//LGE_CHANGE_E [chahee.kim@lge.com] 2012-02-16
static int tegra_fmradio_mode_info(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
		uinfo->count = 1;
		uinfo->value.integer.min = 0;
		uinfo->value.integer.max = 1;
		return 0;
}

static int tegra_fmradio_mode_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct tegra_wm8994 *machine = snd_kcontrol_chip(kcontrol);

		printk(KERN_ERR "tegra_fmradio_mode_get() is_fmradio_mode=%d\n", is_fmradio_mode);
		ucontrol->value.integer.value[0] = is_fmradio_mode;

		return 0;
}

static int tegra_fmradio_mode_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct tegra_wm8994 *machine = snd_kcontrol_chip(kcontrol);
		int is_fmradio_mode_new = ucontrol->value.integer.value[0];

		printk(KERN_ERR "tegra_fmradio_mode_put() is_fmradio_mode=%d\n", is_fmradio_mode);
		printk(KERN_ERR "tegra_fmradio_mode_put() is_fmradio_mode_new=%d\n", is_fmradio_mode_new);
		if (is_fmradio_mode == is_fmradio_mode_new)
			return 0;

				is_fmradio_mode = is_fmradio_mode_new;
				printk(KERN_ERR "tegra_fmradio_mode_put() is_fmradio_mode=%d\n", is_fmradio_mode);

				return 1;
}

struct snd_kcontrol_new tegra_fmradio_mode_control = {
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "FMRadio Mode Switch",
		.private_value = 0xffff,
		.info = tegra_fmradio_mode_info,
		.get = tegra_fmradio_mode_get,
		.put = tegra_fmradio_mode_put
};

bool is_fmradio_state(void)
{
	if (!is_fmradio_mode)
		return false;
	else
		return true;
}
//
#if 0
//LGE_CHANGE_S [heejeong.seo@lge.com] 2011-12-20 [LGE_AP20] set dynamic pull up
int tegra_codec_startup(struct snd_pcm_substream *substream)
{
	tegra20_das_power_mode(true);

	return 0;
}

void tegra_codec_shutdown(struct snd_pcm_substream *substream)
{
	tegra20_das_power_mode(false);
}
//LGE_CHANGE_E [heejeong.seo@lge.com] 2011-12-20 [LGE_AP20] set dynamic pull up
#endif
//LGE_CHANGE_E [chahee.kim@lge.com] 2012-01-21

static struct snd_soc_ops tegra_wm8994_ops = {
	.hw_params = tegra_wm8994_hw_params,
	.hw_free = tegra_hw_free,
//LGE_CHANGE_S [heejeong.seo@lge.com] 2011-12-20 [LGE_AP20] set dynamic pull up
//	.startup = tegra_codec_startup,
//	.shutdown = tegra_codec_shutdown,
//LGE_CHANGE_E [heejeong.seo@lge.com] 2011-12-20 [LGE_AP20] set dynamic pull up

};

//LGE_CHANGE_S [chahee.kim@lge.com] 2012-01-30
static struct snd_soc_ops tegra_voice_call_ops = {
	.hw_params = tegra_voice_call_hw_params,
	.hw_free = tegra_hw_free,
};
//LGE_CHANGE_E [chahee.kim@lge.com] 2012-01-30

static struct snd_soc_ops tegra_bt_sco_ops = {
	.hw_params = tegra_bt_sco_hw_params,
	.hw_free = tegra_hw_free,
};

static struct snd_soc_ops tegra_bt_call_ops = {
	.hw_params = tegra_bt_call_hw_params,
	.hw_free = tegra_hw_free,
};

static struct snd_soc_ops tegra_spdif_ops = {
	.hw_params = tegra_spdif_hw_params,
	.hw_free = tegra_hw_free,
};

static struct snd_soc_jack tegra_wm8994_hp_jack;

// LGE_CHANGE_S [heejeong.seo@lge.com] 2011-12-20 [LGE_AP20]
#if defined (CONFIG_MACH_STAR) || defined (CONFIG_MACH_BSSQ)
static struct snd_soc_jack_pin tegra_wm8994_hp_jack_pins[] = {
	{
		.pin = "Headphone Jack",
		.mask = SND_JACK_HEADPHONE,
	},
};

static struct snd_soc_jack_gpio tegra_wm8994_hp_jack_gpio = {
	.name = "headphone detect",
	.report = SND_JACK_HEADPHONE,
	.debounce_time =700,
	.invert = 0,
};
#endif

#if defined(CONFIG_MACH_STAR) || defined(CONFIG_MACH_BSSQ)
bool in_call_state(void)
{
    return is_call_mode;
}
#endif // MOBII LP1 sleep

// LGE_CHANGE_E [heejeong.seo@lge.com] 2011-12-20 [LGE_AP20]

#if 0
#ifdef CONFIG_SWITCH
static struct switch_dev wired_switch_dev = {
	.name = "h2w",
};

/* These values are copied from WiredAccessoryObserver */
enum headset_state {
	BIT_NO_HEADSET = 0,
	BIT_HEADSET = (1 << 0),
	BIT_HEADSET_NO_MIC = (1 << 1),
};

static int headset_switch_notify(struct notifier_block *self,
	unsigned long action, void *dev)
{
	switch (action) {
	case SND_JACK_HEADPHONE:
		switch_set_state(&wired_switch_dev, BIT_HEADSET_NO_MIC);
		break;
	case SND_JACK_HEADSET:
		switch_set_state(&wired_switch_dev, BIT_HEADSET);
		break;
	default:
		switch_set_state(&wired_switch_dev, BIT_NO_HEADSET);
	}

	return NOTIFY_OK;
}

static struct notifier_block headset_switch_nb = {
	.notifier_call = headset_switch_notify,
};
#else
static struct snd_soc_jack_pin tegra_wm8994_hp_jack_pins[] = {
	{
		.pin = "Headphone Jack",
		.mask = SND_JACK_HEADPHONE,
	},
};

#endif
#endif

static int tegra_wm8994_event_int_spk(struct snd_soc_dapm_widget *w,
					struct snd_kcontrol *k, int event)
{
	struct snd_soc_dapm_context *dapm = w->dapm;
	struct snd_soc_card *card = dapm->card;
	struct tegra_wm8994 *machine = snd_soc_card_get_drvdata(card);
	struct tegra_wm8994_platform_data *pdata = machine->pdata;

	if (!(machine->gpio_requested & GPIO_SPKR_EN))
		return 0;

	gpio_set_value_cansleep(pdata->gpio_spkr_en,
				SND_SOC_DAPM_EVENT_ON(event));

	return 0;
}

static int tegra_wm8994_event_hp(struct snd_soc_dapm_widget *w,
					struct snd_kcontrol *k, int event)
{
	struct snd_soc_dapm_context *dapm = w->dapm;
	struct snd_soc_card *card = dapm->card;
	struct tegra_wm8994 *machine = snd_soc_card_get_drvdata(card);
	struct tegra_wm8994_platform_data *pdata = machine->pdata;

	if (!(machine->gpio_requested & GPIO_HP_MUTE))
		return 0;

	gpio_set_value_cansleep(pdata->gpio_hp_mute,
				!SND_SOC_DAPM_EVENT_ON(event));

	return 0;
}

static const struct snd_soc_dapm_widget tegra_wm8994_dapm_widgets[] = {
	SND_SOC_DAPM_SPK("Int Spk", tegra_wm8994_event_int_spk),
//	SND_SOC_DAPM_HP("Earpiece", NULL),
//	SND_SOC_DAPM_OUTPUT("Mono Out"),
	SND_SOC_DAPM_HP("Headphone Jack", tegra_wm8994_event_hp),
	SND_SOC_DAPM_MIC("Mic Jack", NULL),
	SND_SOC_DAPM_INPUT("Int Mic"),
	SND_SOC_DAPM_LINE("LineIn Jack", NULL),
	SND_SOC_DAPM_HP("Headset Jack", NULL),
};

static const struct snd_soc_dapm_route tegra_wm8994_audio_map[] = {

	/* speaker conntected to SPKOUTLN, SPKOUTLP, SPKOUTRN and SPKOUTRP */
	{"Int Spk", NULL, "SPKOUTLN"},
	{"Int Spk", NULL, "SPKOUTLP"},
	{"Int Spk", NULL, "SPKOUTRN"},
	{"Int Spk", NULL, "SPKOUTRP"},

	{"Headphone Jack", NULL, "HPOUT1L"},
	{"Headphone Jack", NULL, "HPOUT1R"},

	 /* headset connected to input */
       {"Headset Jack", NULL, "IN2RN"},

	/* main mic is connected to INLN */
	{ "MICBIAS1", NULL, "Int Mic" },
	{ "IN1LN", NULL, "MICBIAS1" },

	/* sub mic is connected to IN1RN */
	{"MICBIAS2", NULL, "Line Jack"},
	{"IN1RN", NULL, "MICBIAS2" },

};

static const struct snd_kcontrol_new tegra_wm8994_controls[] = {
	SOC_DAPM_PIN_SWITCH("Int Spk"),
//	SOC_DAPM_PIN_SWITCH("Earpiece"),
//	SOC_DAPM_PIN_SWITCH("Mono Out"),
	SOC_DAPM_PIN_SWITCH("Headphone Jack"),
	SOC_DAPM_PIN_SWITCH("Mic Jack"),
	SOC_DAPM_PIN_SWITCH("Int Mic"),
	SOC_DAPM_PIN_SWITCH("LineIn Jack"),
};

//LGE_CHANGE_S [chahee.kim@lge.com] 2011-11-14
static ssize_t switch_gpio_print_state(struct switch_dev *sdev, char *buf)
{

	printk(KERN_ERR "##(Headset_det.c)## switch_gpio_print_state() => sdev->state(%d), headset_sw_data->state_on(%s)/state_off(%s)\n",sdev->state,headset_sw_data->state_on,headset_sw_data->state_off);

	const char *state;
	if (switch_get_state(sdev))
		state = headset_sw_data->state_on;
	else
		state = headset_sw_data->state_off;

	if (state)
		return sprintf(buf, "%s\n", state);
	return -1;
}
//LGE_CHANGE_E [chahee.kim@lge.com] 2011-11-14

static int tegra_wm8994_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_dapm_context *dapm = &codec->dapm;
	struct snd_soc_card *card = codec->card;
	struct tegra_wm8994 *machine = snd_soc_card_get_drvdata(card);
	struct tegra_wm8994_platform_data *pdata = machine->pdata;
	int ret;
    //LGE_CHANGE_S [chahee.kim@lge.com] 2011-11-14
    struct headset_switch_data *switch_data;
    struct input_dev *ip_dev;

    printk(KERN_ERR "tegra_wm8994_init - start \n");

    switch_data = kzalloc(sizeof(struct headset_switch_data), GFP_KERNEL);
    switch_data->sdev.name = pdata->name;
    switch_data->gpio = pdata->gpio_hp_det;
    switch_data->name_on = NULL;
    switch_data->name_off = NULL;
    switch_data->state_on = NULL;
    switch_data->state_off = NULL;
    switch_data->sdev.print_state = switch_gpio_print_state;
    switch_data->hook_gpio = pdata->gpio_hook;
    switch_data->ear_mic = pdata->gpio_ear_mic;

    ret = switch_dev_register(&switch_data->sdev);

    ip_dev = input_allocate_device();
    switch_data->ip_dev = ip_dev;
    set_bit(EV_SYN, switch_data->ip_dev->evbit);
    set_bit(EV_KEY, switch_data->ip_dev->evbit);
    set_bit(KEY_HOOK, switch_data->ip_dev->keybit);

    switch_data->ip_dev->name = "tegra-snd-wm8994";
    input_register_device(switch_data->ip_dev);
    //switch_data->jack_gpio = &tegra_wm8994_hp_jack_gpio;
    headset_sw_data = switch_data;

    wake_lock_init(&headset_wake_lock, WAKE_LOCK_SUSPEND, "headset_wlock"); //20111017 heejeong.seo@lge.com Problem that no wake up when disconn headset in calling
//LGE_CHANGE_E [chahee.kim@lge.com] 2011-11-14

	if (machine_is_whistler()) {
		machine->audio_reg = regulator_get(NULL, "avddio_audio");
		if (IS_ERR(machine->audio_reg)) {
			dev_err(card->dev, "cannot get avddio_audio reg\n");
			ret = PTR_ERR(machine->audio_reg);
			return ret;
		}

		ret = regulator_enable(machine->audio_reg);
		if (ret) {
			dev_err(card->dev, "cannot enable avddio_audio reg\n");
			regulator_put(machine->audio_reg);
			machine->audio_reg = NULL;
			return ret;
		}
	}

	machine->pcard = card; // nVidia patch
	machine->bias_level = SND_SOC_BIAS_STANDBY;

	if (gpio_is_valid(pdata->gpio_spkr_en)) {
		ret = gpio_request(pdata->gpio_spkr_en, "spkr_en");
		if (ret) {
			dev_err(card->dev, "cannot get spkr_en gpio\n");
			return ret;
		}
		machine->gpio_requested |= GPIO_SPKR_EN;

		gpio_direction_output(pdata->gpio_spkr_en, 0);
	}

	if (gpio_is_valid(pdata->gpio_hp_mute)) {
		ret = gpio_request(pdata->gpio_hp_mute, "hp_mute");
		if (ret) {
			dev_err(card->dev, "cannot get hp_mute gpio\n");
			return ret;
		}
		machine->gpio_requested |= GPIO_HP_MUTE;

		gpio_direction_output(pdata->gpio_hp_mute, 0);
	}

	if (gpio_is_valid(pdata->gpio_int_mic_en)) {
		ret = gpio_request(pdata->gpio_int_mic_en, "int_mic_en");
		if (ret) {
			dev_err(card->dev, "cannot get int_mic_en gpio\n");
			return ret;
		}
		machine->gpio_requested |= GPIO_INT_MIC_EN;

		/* Disable int mic; enable signal is active-high */
		gpio_direction_output(pdata->gpio_int_mic_en, 0);
	}

	if (gpio_is_valid(pdata->gpio_ext_mic_en)) {
		ret = gpio_request(pdata->gpio_ext_mic_en, "ext_mic_en");
		if (ret) {
			dev_err(card->dev, "cannot get ext_mic_en gpio\n");
			return ret;
		}
		machine->gpio_requested |= GPIO_EXT_MIC_EN;

		/* Enable ext mic; enable signal is active-low */
		gpio_direction_output(pdata->gpio_ext_mic_en, 0);
	}

	ret = snd_soc_add_controls(codec, tegra_wm8994_controls,
				   ARRAY_SIZE(tegra_wm8994_controls));
	if (ret < 0)
		return ret;

	snd_soc_dapm_new_controls(dapm, tegra_wm8994_dapm_widgets,
					ARRAY_SIZE(tegra_wm8994_dapm_widgets));

	snd_soc_dapm_add_routes(dapm, tegra_wm8994_audio_map,
					ARRAY_SIZE(tegra_wm8994_audio_map));

//LGE_CHANGE_S [chahee.kim@lge.com] 2012-02-16
	   /* Add call mode switch control */
	ret = snd_ctl_add(codec->card->snd_card,
			snd_ctl_new1(&tegra_call_mode_control, machine));
	if (ret < 0)
		return ret;
//LGE_CHANGE_E [chahee.kim@lge.com] 2012-02-16

  ret = snd_ctl_add(codec->card->snd_card, snd_ctl_new1(&tegra_fmradio_mode_control, machine));
  if (ret < 0)
    return ret;
//
// LGE_CHANGE_S [heejeong.seo@lge.com] 2011-12-20 [LGE_AP20]
#if defined (CONFIG_MACH_STAR) || defined (CONFIG_MACH_BSSQ)
    if (gpio_is_valid(pdata->gpio_hp_det))
    {
        printk(KERN_ERR "(0x%x) \n", pdata->gpio_hp_det);

        tegra_wm8994_hp_jack_gpio.gpio = pdata->gpio_hp_det;
        snd_soc_jack_new(codec, "Headphone Jack", SND_JACK_HEADPHONE,
                &tegra_wm8994_hp_jack);
        snd_soc_jack_add_pins(&tegra_wm8994_hp_jack,
            ARRAY_SIZE(tegra_wm8994_hp_jack_pins),
            tegra_wm8994_hp_jack_pins);

        snd_soc_jack_add_gpios(&tegra_wm8994_hp_jack,
                    1,
                    &tegra_wm8994_hp_jack_gpio);

        //wm8994_mic_detect(codec, &tegra_wm8994_hp_jack,
        //    SND_JACK_HEADPHONE, 0, 0);
    }
#endif
#if 0
    snd_soc_jack_new(codec, "Headphone Jack", SND_JACK_HEADPHONE,
        &tegra_wm8994_hp_jack);
/*
wm8994_headphone_detect(codec, &tegra_wm8994_hp_jack,
    SND_JACK_HEADPHONE, pdata->debounce_time_hp);
    */

#ifdef CONFIG_SWITCH
	snd_soc_jack_notifier_register(&tegra_wm8994_hp_jack,
		&headset_switch_nb);
#else
	snd_soc_jack_add_pins(&tegra_wm8994_hp_jack,
		ARRAY_SIZE(tegra_wm8994_hp_jack_pins),
		tegra_wm8994_hp_jack_pins);
#endif
#endif
// LGE_CHANGE_E [heejeong.seo@lge.com] 2011-12-20 [LGE_AP20]

#if 0
	snd_soc_dapm_nc_pin(dapm, "ACIN");
	snd_soc_dapm_nc_pin(dapm, "ACOP");
	snd_soc_dapm_nc_pin(dapm, "OUT3");
	snd_soc_dapm_nc_pin(dapm, "OUT4");
#endif

	snd_soc_dapm_sync(dapm);

	return 0;
}

static struct snd_soc_dai_link tegra_wm8994_dai[] = {
	{
		.name = "WM8994",
		.stream_name = "WM8994 PCM HIFI",
		.codec_name = "wm8994-codec",
		.platform_name = "tegra-pcm-audio",
		.cpu_dai_name = "tegra20-i2s.0",
		.codec_dai_name = "wm8994-aif1",  //"wm8753-hifi", //heejeong.seo@lge.com   ref.wm8994.c
		.init = tegra_wm8994_init,
		.ops = &tegra_wm8994_ops,
	},
#ifndef CONFIG_LU6500
	{
		.name = "SPDIF",
		.stream_name = "SPDIF PCM",
		.codec_name = "spdif-dit.0",
		.platform_name = "tegra-pcm-audio",
		.cpu_dai_name = "tegra20-spdif",
		.codec_dai_name = "dit-hifi",
		.ops = &tegra_spdif_ops,
	},
#endif
#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	{
		.name = "BT-SCO",
		.stream_name = "BT SCO PCM",
		.codec_name = "spdif-dit.1",
		.platform_name = "tegra-pcm-audio",
		.cpu_dai_name = "tegra20-i2s.1",
		.codec_dai_name = "dit-hifi",
		.ops = &tegra_bt_sco_ops,
	},
#endif
//LGE_CHANGE_S [chahee.kim@lge.com] 2012-01-30
	{
		.name = "VOICE CALL",
		.stream_name = "VOICE CALL PCM",
		.codec_name = "wm8994-codec",
		.platform_name = "tegra-pcm-audio",
		.cpu_dai_name =  "tegra20-i2s.1",
		.codec_dai_name = "wm8994-aif2",
		.ops = &tegra_voice_call_ops,
	},
//LGE_CHANGE_E [chahee.kim@lge.com] 2012-01-30
	{
		.name = "BT VOICE CALL",
		.stream_name = "BT VOICE CALL PCM",
		.codec_name = "spdif-dit.1",
		.platform_name = "tegra-pcm-audio",
		.cpu_dai_name = "tegra20-i2s.1",
		.codec_dai_name = "dit-hifi",
		.ops = &tegra_bt_call_ops,
	},
};

static int tegra20_soc_set_bias_level(struct snd_soc_card *card,
					enum snd_soc_bias_level level)
{
	struct tegra_wm8994 *machine = snd_soc_card_get_drvdata(card);

	if (machine->bias_level == SND_SOC_BIAS_OFF && level != SND_SOC_BIAS_OFF)
		tegra_asoc_utils_clk_enable(&machine->util_data);

	return 0;
}

static int tegra20_soc_set_bias_level_post(struct snd_soc_card *card,
					enum snd_soc_bias_level level)
{
	struct tegra_wm8994 *machine = snd_soc_card_get_drvdata(card);

	if (machine->bias_level != SND_SOC_BIAS_OFF && level == SND_SOC_BIAS_OFF)
		tegra_asoc_utils_clk_disable(&machine->util_data);

	machine->bias_level = level;

	return 0 ;
}

static struct snd_soc_card snd_soc_tegra_wm8994 = {
	.name = "tegra-wm8994",
	.dai_link = tegra_wm8994_dai,
	.num_links = ARRAY_SIZE(tegra_wm8994_dai),
	.set_bias_level = tegra20_soc_set_bias_level,
	.set_bias_level_post = tegra20_soc_set_bias_level_post,
};

static __devinit int tegra_wm8994_driver_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &snd_soc_tegra_wm8994;
	struct tegra_wm8994 *machine;
	struct tegra_wm8994_platform_data *pdata;
	int ret;

	pdata = pdev->dev.platform_data;
	if (!pdata) {
		dev_err(&pdev->dev, "No platform data supplied\n");
		return -EINVAL;
	}

	machine = kzalloc(sizeof(struct tegra_wm8994), GFP_KERNEL);
	if (!machine) {
		dev_err(&pdev->dev, "Can't allocate tegra_wm8994 struct\n");
		return -ENOMEM;
	}

	machine->pdata = pdata;

	ret = tegra_asoc_utils_init(&machine->util_data, &pdev->dev);
	if (ret)
		goto err_free_machine;

	card->dev = &pdev->dev;
	platform_set_drvdata(pdev, card);
	snd_soc_card_set_drvdata(card, machine);

	ret = snd_soc_register_card(card);
	if (ret) {
		dev_err(&pdev->dev, "snd_soc_register_card failed (%d)\n",
			ret);
		goto err_fini_utils;
	}
#if 0
#ifdef CONFIG_SWITCH
	/* Add h2w swith class support */
	ret = switch_dev_register(&wired_switch_dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "not able to register switch device(%d)\n",
			ret);
		goto err_unregister_card;
	}
#endif
#endif

	return 0;

err_unregister_card:
	snd_soc_unregister_card(card);
err_fini_utils:
	tegra_asoc_utils_fini(&machine->util_data);
err_free_machine:
	kfree(machine);
	return ret;
}

static int __devexit tegra_wm8994_driver_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);
	struct tegra_wm8994 *machine = snd_soc_card_get_drvdata(card);
	struct tegra_wm8994_platform_data *pdata = machine->pdata;

	snd_soc_unregister_card(card);
#if 0
#ifdef CONFIG_SWITCH
	switch_dev_unregister(&wired_switch_dev);
#endif
#else
    switch_dev_unregister(&headset_sw_data->sdev);

    cancel_work_sync(&headset_sw_data->work);
    cancel_delayed_work_sync(headset_sw_data->pdelayed_work);
    wake_lock_destroy(&headset_wake_lock);        //20111017 heejeong.seo@lge.com Problem that no wake up when disconn headset in calling


#endif
	tegra_asoc_utils_fini(&machine->util_data);

	if (machine->gpio_requested & GPIO_EXT_MIC_EN)
		gpio_free(pdata->gpio_ext_mic_en);
	if (machine->gpio_requested & GPIO_INT_MIC_EN)
		gpio_free(pdata->gpio_int_mic_en);
	if (machine->gpio_requested & GPIO_HP_MUTE)
		gpio_free(pdata->gpio_hp_mute);
	if (machine->gpio_requested & GPIO_SPKR_EN)
		gpio_free(pdata->gpio_spkr_en);
	if (machine->audio_reg) {
		regulator_disable(machine->audio_reg);
		regulator_put(machine->audio_reg);
	}

	kfree(machine);

	return 0;
}

static struct platform_driver tegra_wm8994_driver = {
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.pm = &snd_soc_pm_ops,
	},
	.probe = tegra_wm8994_driver_probe,
	.remove = __devexit_p(tegra_wm8994_driver_remove),
};

static int __init tegra_wm8994_modinit(void)
{
	return platform_driver_register(&tegra_wm8994_driver);
}
module_init(tegra_wm8994_modinit);

static void __exit tegra_wm8994_modexit(void)
{
	platform_driver_unregister(&tegra_wm8994_driver);
}
module_exit(tegra_wm8994_modexit);


MODULE_DESCRIPTION("Tegra+WM8994 machine ASoC driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRV_NAME);

