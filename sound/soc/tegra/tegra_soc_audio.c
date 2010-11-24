/*
 * sound/soc/tegra/tegra_soc_audio.c
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

#define NV_DEBUG 0

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/tlv.h>
#include <linux/clk.h>
#include <asm/mach-types.h>
#include <asm/hardware/scoop.h>
#include <linux/io.h>
#include "nvodm_query_discovery.h"
#include "tegra_transport.h"

struct codec_setup_data {
	unsigned dem0_pin;
	unsigned dem1_pin;
	unsigned pdad_pin;
	unsigned pdda_pin;
};

extern struct snd_soc_codec_device soc_codec_dev_tegra_generic_codec;
extern struct snd_soc_dai tegra_generic_codec_dai[];

static struct platform_device *tegra_snd_device;
NvU64 codec_guid;

#define NVODM_CODEC_MAX_CLOCKS 3

static unsigned int clock_frequencies[NVODM_CODEC_MAX_CLOCKS];

static int set_clock_source_on_codec(NvU64 codec_guid,int IsEnable)
{
	const NvOdmPeripheralConnectivity *p_connectivity = NULL;
	unsigned int clock_instances[NVODM_CODEC_MAX_CLOCKS];
	unsigned int num_clocks;
	p_connectivity = NvOdmPeripheralGetGuid(codec_guid);
	if (p_connectivity == NULL)
		return NV_FALSE;

	if (IsEnable) {
		if (!NvOdmExternalClockConfig(codec_guid, NV_FALSE,
					      clock_instances,
					      clock_frequencies, &num_clocks))
			return NV_FALSE;
	} else {
		if (!NvOdmExternalClockConfig(codec_guid,
					      NV_TRUE,
					      clock_instances,
					      clock_frequencies,
					      &num_clocks));
		return NV_FALSE;
	}
	return NV_TRUE;
}

static int tegra_hifi_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params)
{
	/* Set codec DAI configuration */
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
	int ret;

	ret = snd_soc_dai_set_fmt(codec_dai,SND_SOC_DAIFMT_I2S);

	if (ret < 0)
		return ret;

	return 0;
}

static struct snd_soc_ops tegra_hifi_ops = {
	.hw_params = tegra_hifi_hw_params,
};

static int tegra_codec_init(struct snd_soc_codec *codec)
{
	return 0;
}

extern struct snd_soc_dai tegra_i2s_rpc_dai;
extern struct snd_soc_platform tegra_soc_platform;

static struct snd_soc_dai_link tegra_board_dai[] = {
	{
		.name = "tegra-generic-codec",
		.stream_name = "tegra-codec-rpc",
		.cpu_dai = &tegra_i2s_rpc_dai,
		.codec_dai = &tegra_generic_codec_dai[I2S1],
		.init = tegra_codec_init,
		.ops = &tegra_hifi_ops,
	},
	{
		.name = "tegra-bluetooth",
		.stream_name = "tegra-codec-bluetooth",
		.cpu_dai = &tegra_i2s_rpc_dai,
		.codec_dai = &tegra_generic_codec_dai[I2S2],
		.init = tegra_codec_init,
		.ops = &tegra_hifi_ops,
	}
};

static struct snd_soc_card tegra_board = {
	.name = "tegra",
	.platform = &tegra_soc_platform,
	.dai_link = tegra_board_dai,
	.num_links = ARRAY_SIZE(tegra_board_dai),
};

static struct snd_soc_device tegra_board_snd_devdata = {
	.card = &tegra_board,
	.codec_dev = &soc_codec_dev_tegra_generic_codec,
};

static int __init tegra_soc_init(void)
{
	int ret;
	tegra_snd_device = platform_device_alloc("soc-audio", -1);
	if (!tegra_snd_device)
		return -ENOMEM;

	platform_set_drvdata(tegra_snd_device, &tegra_board_snd_devdata);
	tegra_board_snd_devdata.dev = &tegra_snd_device->dev;

	ret = platform_device_add(tegra_snd_device);
	if (ret) {
		snd_printk(KERN_ERR "tegra audio device could not be added \n");
		platform_device_put(tegra_snd_device);
		return ret;
	}

	codec_guid = NV_ODM_GUID('w','o','l','f','8','7','5','3');

	set_clock_source_on_codec(codec_guid,NV_TRUE);
	if (ret != 0)
		platform_device_unregister(tegra_snd_device);

	return ret;
}

static void __exit tegra_soc_exit(void)
{
	set_clock_source_on_codec(codec_guid,0);
	platform_device_unregister(tegra_snd_device);
}

module_init(tegra_soc_init);
module_exit(tegra_soc_exit);

/* Module information */
MODULE_DESCRIPTION("Tegra SoC Sound");
MODULE_LICENSE("GPL");
