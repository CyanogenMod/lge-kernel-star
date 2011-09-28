/*
 * tegra30_i2s.c - Tegra 30 I2S driver
 *
 * Author: Stephen Warren <swarren@nvidia.com>
 * Copyright (c) 2010-2011, NVIDIA Corporation.
 *
 * Based on code copyright/by:
 *
 * Copyright (c) 2009-2010, NVIDIA Corporation.
 * Scott Peterson <speterson@nvidia.com>
 *
 * Copyright (C) 2010 Google, Inc.
 * Iliyan Malchev <malchev@google.com>
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

#include <linux/clk.h>
#include <linux/module.h>
#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <mach/iomap.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include "tegra30_ahub.h"
#include "tegra30_i2s.h"

#define DRV_NAME "tegra30-i2s"

static inline void tegra30_i2s_write(struct tegra30_i2s *i2s, u32 reg, u32 val)
{
	__raw_writel(val, i2s->regs + reg);
}

static inline u32 tegra30_i2s_read(struct tegra30_i2s *i2s, u32 reg)
{
	return __raw_readl(i2s->regs + reg);
}

static void tegra30_i2s_enable_clocks(struct tegra30_i2s *i2s)
{
	tegra30_ahub_enable_clocks();
	clk_enable(i2s->clk_i2s);
}

static void tegra30_i2s_disable_clocks(struct tegra30_i2s *i2s)
{
	clk_disable(i2s->clk_i2s);
	tegra30_ahub_disable_clocks();
}

#ifdef CONFIG_DEBUG_FS
static int tegra30_i2s_show(struct seq_file *s, void *unused)
{
#define REG(r) { r, #r }
	static const struct {
		int offset;
		const char *name;
	} regs[] = {
		REG(TEGRA30_I2S_CTRL),
		REG(TEGRA30_I2S_TIMING),
		REG(TEGRA30_I2S_OFFSET),
		REG(TEGRA30_I2S_CH_CTRL),
		REG(TEGRA30_I2S_SLOT_CTRL),
		REG(TEGRA30_I2S_CIF_TX_CTRL),
		REG(TEGRA30_I2S_CIF_RX_CTRL),
		REG(TEGRA30_I2S_FLOWCTL),
		REG(TEGRA30_I2S_TX_STEP),
		REG(TEGRA30_I2S_FLOW_STATUS),
		REG(TEGRA30_I2S_FLOW_TOTAL),
		REG(TEGRA30_I2S_FLOW_OVER),
		REG(TEGRA30_I2S_FLOW_UNDER),
		REG(TEGRA30_I2S_LCOEF_1_4_0),
		REG(TEGRA30_I2S_LCOEF_1_4_1),
		REG(TEGRA30_I2S_LCOEF_1_4_2),
		REG(TEGRA30_I2S_LCOEF_1_4_3),
		REG(TEGRA30_I2S_LCOEF_1_4_4),
		REG(TEGRA30_I2S_LCOEF_1_4_5),
		REG(TEGRA30_I2S_LCOEF_2_4_0),
		REG(TEGRA30_I2S_LCOEF_2_4_1),
		REG(TEGRA30_I2S_LCOEF_2_4_2),
	};
#undef REG

	struct tegra30_i2s *i2s = s->private;
	int i;

	tegra30_i2s_enable_clocks(i2s);

	for (i = 0; i < ARRAY_SIZE(regs); i++) {
		u32 val = tegra30_i2s_read(i2s, regs[i].offset);
		seq_printf(s, "%s = %08x\n", regs[i].name, val);
	}

	tegra30_i2s_disable_clocks(i2s);

	return 0;
}

static int tegra30_i2s_debug_open(struct inode *inode, struct file *file)
{
	return single_open(file, tegra30_i2s_show, inode->i_private);
}

static const struct file_operations tegra30_i2s_debug_fops = {
	.open    = tegra30_i2s_debug_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = single_release,
};

static void tegra30_i2s_debug_add(struct tegra30_i2s *i2s, int id)
{
	char name[] = DRV_NAME ".0";

	snprintf(name, sizeof(name), DRV_NAME".%1d", id);
	i2s->debug = debugfs_create_file(name, S_IRUGO, snd_soc_debugfs_root,
						i2s, &tegra30_i2s_debug_fops);
}

static void tegra30_i2s_debug_remove(struct tegra30_i2s *i2s)
{
	if (i2s->debug)
		debugfs_remove(i2s->debug);
}
#else
static inline void tegra30_i2s_debug_add(struct tegra30_i2s *i2s, int id)
{
}

static inline void tegra30_i2s_debug_remove(struct tegra30_i2s *i2s)
{
}
#endif

int tegra30_i2s_startup(struct snd_pcm_substream *substream,
			struct snd_soc_dai *dai)
{
	struct tegra30_i2s *i2s = snd_soc_dai_get_drvdata(dai);
	int ret;

	tegra30_i2s_enable_clocks(i2s);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		ret = tegra30_ahub_allocate_tx_fifo(&i2s->txcif,
					&i2s->playback_dma_data.addr,
					&i2s->playback_dma_data.req_sel);
		i2s->playback_dma_data.wrap = 4;
		i2s->playback_dma_data.width = 32;
		tegra30_ahub_set_rx_cif_source(TEGRA30_AHUB_RXCIF_I2S0_RX0 +
					       i2s->id,
					       i2s->txcif);
	} else {
		ret = tegra30_ahub_allocate_rx_fifo(&i2s->rxcif,
					&i2s->capture_dma_data.addr,
					&i2s->capture_dma_data.req_sel);
		i2s->capture_dma_data.wrap = 4;
		i2s->capture_dma_data.width = 32;
		tegra30_ahub_set_rx_cif_source(i2s->rxcif,
					TEGRA30_AHUB_TXCIF_I2S0_TX0 + i2s->id);
	}

	tegra30_i2s_disable_clocks(i2s);

	return ret;
}

void tegra30_i2s_shutdown(struct snd_pcm_substream *substream,
			struct snd_soc_dai *dai)
{
	struct tegra30_i2s *i2s = snd_soc_dai_get_drvdata(dai);

	tegra30_i2s_enable_clocks(i2s);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		tegra30_ahub_unset_rx_cif_source(
					TEGRA30_AHUB_RXCIF_I2S0_RX0 + i2s->id);
		tegra30_ahub_free_tx_fifo(i2s->txcif);
	} else {
		tegra30_ahub_unset_rx_cif_source(i2s->rxcif);
		tegra30_ahub_free_rx_fifo(i2s->rxcif);
	}

	tegra30_i2s_disable_clocks(i2s);
}

static int tegra30_i2s_set_fmt(struct snd_soc_dai *dai,
				unsigned int fmt)
{
	struct tegra30_i2s *i2s = snd_soc_dai_get_drvdata(dai);

	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		break;
	default:
		return -EINVAL;
	}

	i2s->reg_ctrl &= ~TEGRA30_I2S_CTRL_MASTER_ENABLE;
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBS_CFS:
		i2s->reg_ctrl |= TEGRA30_I2S_CTRL_MASTER_ENABLE;
		break;
	case SND_SOC_DAIFMT_CBM_CFM:
		break;
	default:
		return -EINVAL;
	}

	i2s->reg_ctrl &= ~(TEGRA30_I2S_CTRL_FRAME_FORMAT_MASK |
				TEGRA30_I2S_CTRL_LRCK_MASK);
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_DSP_A:
		i2s->reg_ctrl |= TEGRA30_I2S_CTRL_FRAME_FORMAT_FSYNC;
		i2s->reg_ctrl |= TEGRA30_I2S_CTRL_LRCK_L_LOW;
		break;
	case SND_SOC_DAIFMT_DSP_B:
		i2s->reg_ctrl |= TEGRA30_I2S_CTRL_FRAME_FORMAT_FSYNC;
		i2s->reg_ctrl |= TEGRA30_I2S_CTRL_LRCK_R_LOW;
		break;
	case SND_SOC_DAIFMT_I2S:
		i2s->reg_ctrl |= TEGRA30_I2S_CTRL_FRAME_FORMAT_LRCK;
		i2s->reg_ctrl |= TEGRA30_I2S_CTRL_LRCK_L_LOW;
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		i2s->reg_ctrl |= TEGRA30_I2S_CTRL_FRAME_FORMAT_LRCK;
		i2s->reg_ctrl |= TEGRA30_I2S_CTRL_LRCK_L_LOW;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		i2s->reg_ctrl |= TEGRA30_I2S_CTRL_FRAME_FORMAT_LRCK;
		i2s->reg_ctrl |= TEGRA30_I2S_CTRL_LRCK_L_LOW;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int tegra30_i2s_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *dai)
{
	struct device *dev = substream->pcm->card->dev;
	struct tegra30_i2s *i2s = snd_soc_dai_get_drvdata(dai);
	u32 val;
	int ret, sample_size, srate, i2sclock, bitcnt;

	if (params_channels(params) != 2)
		return -EINVAL;

	i2s->reg_ctrl &= ~TEGRA30_I2S_CTRL_BIT_SIZE_MASK;
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		i2s->reg_ctrl |= TEGRA30_I2S_CTRL_BIT_SIZE_16;
		sample_size = 16;
		break;
	default:
		return -EINVAL;
	}

	srate = params_rate(params);

	/* Final "* 2" required by Tegra hardware */
	i2sclock = srate * params_channels(params) * sample_size * 2;

	bitcnt = (i2sclock / (2 * srate)) - 1;
	if (bitcnt < 0 || bitcnt > TEGRA30_I2S_TIMING_CHANNEL_BIT_COUNT_MASK_US)
		return -EINVAL;

	ret = clk_set_rate(i2s->clk_i2s, i2sclock);
	if (ret) {
		dev_err(dev, "Can't set I2S clock rate: %d\n", ret);
		return ret;
	}

	tegra30_i2s_enable_clocks(i2s);

	val = bitcnt << TEGRA30_I2S_TIMING_CHANNEL_BIT_COUNT_SHIFT;

	if (i2sclock % (2 * srate))
		val |= TEGRA30_I2S_TIMING_NON_SYM_ENABLE;

	tegra30_i2s_write(i2s, TEGRA30_I2S_TIMING, val);

	val = (0 << TEGRA30_AUDIOCIF_CTRL_FIFO_THRESHOLD_SHIFT) |
	      (1 << TEGRA30_AUDIOCIF_CTRL_AUDIO_CHANNELS_SHIFT) |
	      (1 << TEGRA30_AUDIOCIF_CTRL_CLIENT_CHANNELS_SHIFT) |
	      TEGRA30_AUDIOCIF_CTRL_AUDIO_BITS_16 |
	      TEGRA30_AUDIOCIF_CTRL_CLIENT_BITS_16;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		val |= TEGRA30_AUDIOCIF_CTRL_DIRECTION_RX;
		tegra30_i2s_write(i2s, TEGRA30_I2S_CIF_RX_CTRL, val);
	} else {
		val |= TEGRA30_AUDIOCIF_CTRL_DIRECTION_TX;
		tegra30_i2s_write(i2s, TEGRA30_I2S_CIF_TX_CTRL, val);
	}

	val = (1 << TEGRA30_I2S_OFFSET_RX_DATA_OFFSET_SHIFT) |
	      (1 << TEGRA30_I2S_OFFSET_TX_DATA_OFFSET_SHIFT);
	tegra30_i2s_write(i2s, TEGRA30_I2S_OFFSET, val);

	tegra30_i2s_disable_clocks(i2s);

	return 0;
}

static void tegra30_i2s_start_playback(struct tegra30_i2s *i2s)
{
	tegra30_ahub_enable_tx_fifo(i2s->txcif);
	i2s->reg_ctrl |= TEGRA30_I2S_CTRL_XFER_EN_TX;
	tegra30_i2s_write(i2s, TEGRA30_I2S_CTRL, i2s->reg_ctrl);
}

static void tegra30_i2s_stop_playback(struct tegra30_i2s *i2s)
{
	tegra30_ahub_disable_tx_fifo(i2s->txcif);
	i2s->reg_ctrl &= ~TEGRA30_I2S_CTRL_XFER_EN_TX;
	tegra30_i2s_write(i2s, TEGRA30_I2S_CTRL, i2s->reg_ctrl);
}

static void tegra30_i2s_start_capture(struct tegra30_i2s *i2s)
{
	tegra30_ahub_enable_rx_fifo(i2s->rxcif);
	i2s->reg_ctrl |= TEGRA30_I2S_CTRL_XFER_EN_RX;
	tegra30_i2s_write(i2s, TEGRA30_I2S_CTRL, i2s->reg_ctrl);
}

static void tegra30_i2s_stop_capture(struct tegra30_i2s *i2s)
{
	tegra30_ahub_disable_rx_fifo(i2s->rxcif);
	i2s->reg_ctrl &= ~TEGRA30_I2S_CTRL_XFER_EN_RX;
	tegra30_i2s_write(i2s, TEGRA30_I2S_CTRL, i2s->reg_ctrl);
}

static int tegra30_i2s_trigger(struct snd_pcm_substream *substream, int cmd,
				struct snd_soc_dai *dai)
{
	struct tegra30_i2s *i2s = snd_soc_dai_get_drvdata(dai);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
	case SNDRV_PCM_TRIGGER_RESUME:
		tegra30_i2s_enable_clocks(i2s);
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			tegra30_i2s_start_playback(i2s);
		else
			tegra30_i2s_start_capture(i2s);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
	case SNDRV_PCM_TRIGGER_SUSPEND:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			tegra30_i2s_stop_playback(i2s);
		else
			tegra30_i2s_stop_capture(i2s);
		tegra30_i2s_disable_clocks(i2s);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int tegra30_i2s_probe(struct snd_soc_dai *dai)
{
	struct tegra30_i2s *i2s = snd_soc_dai_get_drvdata(dai);

	dai->capture_dma_data = &i2s->capture_dma_data;
	dai->playback_dma_data = &i2s->playback_dma_data;

	return 0;
}

static struct snd_soc_dai_ops tegra30_i2s_dai_ops = {
	.startup	= tegra30_i2s_startup,
	.shutdown	= tegra30_i2s_shutdown,
	.set_fmt	= tegra30_i2s_set_fmt,
	.hw_params	= tegra30_i2s_hw_params,
	.trigger	= tegra30_i2s_trigger,
};

#define TEGRA30_I2S_DAI(id) \
	{ \
		.name = DRV_NAME "." #id, \
		.probe = tegra30_i2s_probe, \
		.playback = { \
			.channels_min = 2, \
			.channels_max = 2, \
			.rates = SNDRV_PCM_RATE_8000_96000, \
			.formats = SNDRV_PCM_FMTBIT_S16_LE, \
		}, \
		.capture = { \
			.channels_min = 2, \
			.channels_max = 2, \
			.rates = SNDRV_PCM_RATE_8000_96000, \
			.formats = SNDRV_PCM_FMTBIT_S16_LE, \
		}, \
		.ops = &tegra30_i2s_dai_ops, \
		.symmetric_rates = 1, \
	}

struct snd_soc_dai_driver tegra30_i2s_dai[] = {
	TEGRA30_I2S_DAI(0),
	TEGRA30_I2S_DAI(1),
	TEGRA30_I2S_DAI(2),
	TEGRA30_I2S_DAI(3),
	TEGRA30_I2S_DAI(4),
};

static __devinit int tegra30_i2s_platform_probe(struct platform_device *pdev)
{
	struct tegra30_i2s *i2s;
	struct resource *mem, *memregion;
	int ret;

	if ((pdev->id < 0) ||
		(pdev->id >= ARRAY_SIZE(tegra30_i2s_dai))) {
		dev_err(&pdev->dev, "ID %d out of range\n", pdev->id);
		return -EINVAL;
	}

	i2s = kzalloc(sizeof(struct tegra30_i2s), GFP_KERNEL);
	if (!i2s) {
		dev_err(&pdev->dev, "Can't allocate tegra30_i2s\n");
		ret = -ENOMEM;
		goto exit;
	}
	dev_set_drvdata(&pdev->dev, i2s);
	i2s->id = pdev->id;

	i2s->clk_i2s = clk_get(&pdev->dev, NULL);
	if (IS_ERR(i2s->clk_i2s)) {
		dev_err(&pdev->dev, "Can't retrieve i2s clock\n");
		ret = PTR_ERR(i2s->clk_i2s);
		goto err_free;
	}

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem) {
		dev_err(&pdev->dev, "No memory resource\n");
		ret = -ENODEV;
		goto err_clk_put;
	}

	memregion = request_mem_region(mem->start, resource_size(mem),
					DRV_NAME);
	if (!memregion) {
		dev_err(&pdev->dev, "Memory region already claimed\n");
		ret = -EBUSY;
		goto err_clk_put;
	}

	i2s->regs = ioremap(mem->start, resource_size(mem));
	if (!i2s->regs) {
		dev_err(&pdev->dev, "ioremap failed\n");
		ret = -ENOMEM;
		goto err_release;
	}

	ret = snd_soc_register_dai(&pdev->dev, &tegra30_i2s_dai[pdev->id]);
	if (ret) {
		dev_err(&pdev->dev, "Could not register DAI: %d\n", ret);
		ret = -ENOMEM;
		goto err_unmap;
	}

	tegra30_i2s_debug_add(i2s, pdev->id);

	return 0;

err_unmap:
	iounmap(i2s->regs);
err_release:
	release_mem_region(mem->start, resource_size(mem));
err_clk_put:
	clk_put(i2s->clk_i2s);
err_free:
	kfree(i2s);
exit:
	return ret;
}

static int __devexit tegra30_i2s_platform_remove(struct platform_device *pdev)
{
	struct tegra30_i2s *i2s = dev_get_drvdata(&pdev->dev);
	struct resource *res;

	snd_soc_unregister_dai(&pdev->dev);

	tegra30_i2s_debug_remove(i2s);

	iounmap(i2s->regs);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(res->start, resource_size(res));

	clk_put(i2s->clk_i2s);

	kfree(i2s);

	return 0;
}

static struct platform_driver tegra30_i2s_driver = {
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
	},
	.probe = tegra30_i2s_platform_probe,
	.remove = __devexit_p(tegra30_i2s_platform_remove),
};

static int __init snd_tegra30_i2s_init(void)
{
	return platform_driver_register(&tegra30_i2s_driver);
}
module_init(snd_tegra30_i2s_init);

static void __exit snd_tegra30_i2s_exit(void)
{
	platform_driver_unregister(&tegra30_i2s_driver);
}
module_exit(snd_tegra30_i2s_exit);

MODULE_AUTHOR("Stephen Warren <swarren@nvidia.com>");
MODULE_DESCRIPTION("Tegra 30 I2S ASoC driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRV_NAME);
