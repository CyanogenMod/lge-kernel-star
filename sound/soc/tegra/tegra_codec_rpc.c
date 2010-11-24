/*
 * sound/soc/tegra/tegra_codec_rpc.c
 *
 * ALSA SOC driver for NVIDIA Tegra SoCs
 *
 * Copyright (C) 2010 NVIDIA Corporation
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <sound/soc.h>
#include <sound/pcm.h>
#include <sound/initval.h>

#include "tegra_transport.h"

extern struct tegra_audio_data* tegra_snd_cx[];

static int tegra_master_volume_info(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 2;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = NvAudioFxVolumeMax;
	return 0;
}

static int tegra_master_volume_get(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	int rval = NvAudioFxVolumeDefault;
	int lval = NvAudioFxVolumeDefault;
	struct tegra_audio_data *ptscx = tegra_snd_cx[I2S1];

	if (ptscx) {
		if (!tegra_audiofx_init(ptscx)) {
			rval = ptscx->i2s1volume;
			lval = ptscx->i2s1volume;
		}
	}
	ucontrol->value.integer.value[0] = rval;
	ucontrol->value.integer.value[1] = lval;
	return 0;
}

static int tegra_master_volume_put(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	int change = 0, val;
	NvAudioFxVolumeDescriptor vd;
	struct tegra_audio_data *ptscx = tegra_snd_cx[I2S1];

	val = ucontrol->value.integer.value[0] & 0xffff;
	vd.LeftVolume = val;
	vd.RightVolume = val;
	if(val) {
		vd.Mute = 0;
	}
	else {
		vd.Mute = 1;
	}

	if (ptscx) {
		if (!tegra_audiofx_init(ptscx)) {
			if(ptscx->i2s1volume != val) {
				ptscx->i2s1volume = val;
				ptscx->xrt_fxn.SetProperty(
					ptscx->mvolume,
					NvAudioFxVolumeProperty_Volume,
					sizeof(NvAudioFxVolumeDescriptor),
					&vd);
				change = 1;
			}
		}
	}

	return change;
}

static struct snd_kcontrol_new tegra_codec_ctrl_volume =
{
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.name = "Master Playback Volume",
	.private_value = 0xffff,
	.info = tegra_master_volume_info,
	.get = tegra_master_volume_get,
	.put = tegra_master_volume_put
};

static int tegra_master_route_info(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 1;
	return 0;
}

static int tegra_master_route_get(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	struct tegra_audio_data *ptscx = tegra_snd_cx[I2S1];

	ucontrol->value.integer.value[0] = 0;
	if (ptscx) {
		if (!tegra_audiofx_init(ptscx)) {
			ucontrol->value.integer.value[0] =
						ptscx->spdif_plugin;
		}
	}
	return 0;
}

static int tegra_master_route_put(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	int change = 0, val;
	struct tegra_audio_data *ptscx = tegra_snd_cx[I2S1];

	val = ucontrol->value.integer.value[0] & 0xffff;

	if (ptscx) {
		if (!tegra_audiofx_init(ptscx)) {
			ptscx->spdif_plugin = val;
			tegra_audiofx_route(ptscx);
			change = 1;
		}
	}
	return change;
}

static struct snd_kcontrol_new tegra_codec_ctrl_route =
{
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.name = "SPDIF Playback Switch",
	.private_value = 0xffff,
	.info = tegra_master_route_info,
	.get = tegra_master_route_get,
	.put = tegra_master_route_put
};

static int tegra_generic_codec_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *dai)
{
	return 0;
}

static int tegra_generic_codec_mute(struct snd_soc_dai *dai, int mute)
{
	return 0;
}

static int tegra_generic_codec_set_dai_fmt(struct snd_soc_dai *codec_dai,
		unsigned int fmt)
{
	return 0;
}

static int tegra_generic_codec_set_dai_clkdiv(struct snd_soc_dai *codec_dai,
		int div_id, int div)
{
	return 0;
}

static int tegra_generic_codec_set_dai_pll(struct snd_soc_dai *codec_dai,
		int pll_id, unsigned int freq_in, unsigned int freq_out)
{
	return 0;
}

static int tegra_generic_codec_set_dai_sysclk(struct snd_soc_dai *codec_dai,
		int clk_id, unsigned int freq, int dir)
{
	return 0;
}

static struct snd_soc_dai_ops dit_stub_ops = {
	.hw_params = tegra_generic_codec_hw_params,
	.digital_mute = tegra_generic_codec_mute,
	.set_fmt = tegra_generic_codec_set_dai_fmt,
	.set_clkdiv = tegra_generic_codec_set_dai_clkdiv,
	.set_pll = tegra_generic_codec_set_dai_pll,
	.set_sysclk = tegra_generic_codec_set_dai_sysclk,
};

struct snd_soc_dai tegra_generic_codec_dai[] = {
	{
		.name = "tegra-codec-rpc",
		.id = 0,
		.playback = {
			.stream_name    = "Playback",
			.channels_min   = 1,
			.channels_max   = 2,
			.rates          = TEGRA_SAMPLE_RATES,
			.formats        = TEGRA_SAMPLE_FORMATS,
		},
		.capture = {
			.stream_name    = "Capture",
			.channels_min   = 1,
			.channels_max   = 2,
			.rates          = TEGRA_SAMPLE_RATES,
			.formats        = TEGRA_SAMPLE_FORMATS,
		},
		.ops = &dit_stub_ops,
	},
	{
		.name = "tegra-codec-bluetooth",
		.id = 1,
		.playback = {
			.stream_name    = "Playback",
			.channels_min   = 1,
			.channels_max   = 2,
			.rates          = TEGRA_SAMPLE_RATES,
			.formats        = TEGRA_SAMPLE_FORMATS,
		},
		.capture = {
			.stream_name    = "Capture",
			.channels_min   = 1,
			.channels_max   = 2,
			.rates          = TEGRA_SAMPLE_RATES,
			.formats        = TEGRA_SAMPLE_FORMATS,
		},
		.ops = &dit_stub_ops,
	}
};
EXPORT_SYMBOL_GPL(tegra_generic_codec_dai);

static int __init dit_modinit(void)
{
	return snd_soc_register_dais(tegra_generic_codec_dai, ARRAY_SIZE(tegra_generic_codec_dai));
}

static void __exit dit_exit(void)
{
	snd_soc_unregister_dais(tegra_generic_codec_dai, ARRAY_SIZE(tegra_generic_codec_dai));
}

module_init(dit_modinit);
module_exit(dit_exit);

static int codec_soc_probe(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec;
	int ret = 0;

	socdev->card->codec = kzalloc(sizeof(struct snd_soc_codec), GFP_KERNEL);
	if (!socdev->card->codec)
		return -ENOMEM;

	codec = socdev->card->codec;
	mutex_init(&codec->mutex);

	codec->name = "tegra-generic-codec";
	codec->owner = THIS_MODULE;
	codec->dai = tegra_generic_codec_dai;
	codec->num_dai = ARRAY_SIZE(tegra_generic_codec_dai);
	codec->write = NULL;
	codec->read = NULL;
	INIT_LIST_HEAD(&codec->dapm_widgets);
	INIT_LIST_HEAD(&codec->dapm_paths);

	/* Register PCMs. */
	ret = snd_soc_new_pcms(socdev, SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
	if (ret < 0) {
		printk(KERN_ERR "codec: failed to create pcms\n");
		goto pcm_err;
	}
	/* Register Card. */
	ret = snd_soc_init_card(socdev);
	if (ret < 0) {
		printk(KERN_ERR "codec: failed to register card\n");
		goto card_err;
	}
	/* Add volume control */
	ret = snd_ctl_add(codec->card,
			   snd_ctl_new1(&tegra_codec_ctrl_volume, codec));
	if (ret < 0) {
		printk(KERN_ERR "codec: failed to add control\n");
		goto card_err;
	}
	/* Add route control */
	return snd_ctl_add(codec->card,
			   snd_ctl_new1(&tegra_codec_ctrl_route, codec));

card_err:
	snd_soc_free_pcms(socdev);
pcm_err:
	kfree(socdev->card->codec);

	return ret;
}

static int codec_soc_remove(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->card->codec;

	if (!codec)
		return 0;

	snd_soc_free_pcms(socdev);
	kfree(socdev->card->codec);

	return 0;
}

#define codec_soc_suspend NULL
#define codec_soc_resume NULL

struct snd_soc_codec_device soc_codec_dev_tegra_generic_codec = {
	.probe = 	codec_soc_probe,
	.remove = 	codec_soc_remove,
	.suspend =	codec_soc_suspend,
	.resume =	codec_soc_resume,
};
EXPORT_SYMBOL_GPL(soc_codec_dev_tegra_generic_codec);

/* Module information */
MODULE_DESCRIPTION("Tegra Codec RPC Interface");
MODULE_LICENSE("GPL");
