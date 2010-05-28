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

struct snd_soc_dai dit_stub_dai = {
	.name = "tegra-codec-rpc",
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
};
EXPORT_SYMBOL_GPL(dit_stub_dai);

static int __init dit_modinit(void)
{
	return snd_soc_register_dai(&dit_stub_dai);
}

static void __exit dit_exit(void)
{
	snd_soc_unregister_dai(&dit_stub_dai);
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
	codec->dai = &dit_stub_dai;
	codec->num_dai = 1;
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

	return ret;

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
