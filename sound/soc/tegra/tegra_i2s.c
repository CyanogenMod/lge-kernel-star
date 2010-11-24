/*
 * sound/soc/tegra/tegra_i2s.c
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


#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/jiffies.h>
#include <linux/io.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/initval.h>
#include <sound/soc.h>

#include "mach/nvrm_linux.h"
#include "nvrm_memmgr.h"
#include "nvassert.h"
#include "tegra_transport.h"

static int tegra_i2s_rpc_hw_params(struct snd_pcm_substream *substream,
				   struct snd_pcm_hw_params *params,
				   struct snd_soc_dai *dai)
{
	switch (params_rate(params)) {
	case 8000:
	case 11025:
	case 16000:
	case 22050:
	case 24000:
	case 32000:
	case 44100:
	case 48000:
	case 88200:
	case 96000:
		return 0;
	default:
		return -EINVAL;
	}
}

static int tegra_i2s_rpc_probe(struct platform_device *pdev,
			       struct snd_soc_dai *dai)
{
	return 0;
}

static struct snd_soc_dai_ops tegra_dai_ops = {
	.hw_params = tegra_i2s_rpc_hw_params,
};

struct snd_soc_dai tegra_i2s_rpc_dai[] = {
	{
		.name = "tegra-i2s-rpc",
		.id = 0,
		.probe = tegra_i2s_rpc_probe,
		.playback = {
			.channels_min = 1,
			.channels_max = 2,
			.rates = TEGRA_SAMPLE_RATES,
			.formats = TEGRA_SAMPLE_FORMATS,
		},
		.capture = {
			.channels_min = 1,
			.channels_max = 2,
			.rates = TEGRA_SAMPLE_RATES,
			.formats = TEGRA_SAMPLE_FORMATS,
		},
		.ops = &tegra_dai_ops,
	},
	{
		.name = "tegra-i2s-rpc",
		.id = 1,
		.probe = tegra_i2s_rpc_probe,
		.playback = {
			.channels_min = 1,
			.channels_max = 2,
			.rates = TEGRA_SAMPLE_RATES,
			.formats = TEGRA_SAMPLE_FORMATS,
		},
		.capture = {
			.channels_min = 1,
			.channels_max = 2,
			.rates = TEGRA_SAMPLE_RATES,
			.formats = TEGRA_SAMPLE_FORMATS,
		},
		.ops = &tegra_dai_ops,
	},
};
EXPORT_SYMBOL_GPL(tegra_i2s_rpc_dai);

static int __init tegra_i2s_rpc_init(void)
{
	return snd_soc_register_dais(tegra_i2s_rpc_dai, ARRAY_SIZE(tegra_i2s_rpc_dai));
}
module_init(tegra_i2s_rpc_init);

static void __exit tegra_i2s_rpc_exit(void)
{
	snd_soc_unregister_dais(tegra_i2s_rpc_dai, ARRAY_SIZE(tegra_i2s_rpc_dai));
}
module_exit(tegra_i2s_rpc_exit);

/* Module information */
MODULE_DESCRIPTION("Tegra I2S RPC Interface");
MODULE_LICENSE("GPL");
