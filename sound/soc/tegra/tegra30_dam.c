/*
 * tegra30_dam.c - Tegra 30 DAM driver
 *
 * Author: Nikesh Oswal <noswal@nvidia.com>
 * Copyright (C) 2011 - NVIDIA, Inc.
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
#include <sound/soc.h>
#include "tegra30_dam.h"
#include "tegra30_ahub.h"

#define DRV_NAME "tegra30-dam"

static struct tegra30_dam_context	*dams_cont_info[TEGRA30_NR_DAM_IFC];

enum {
	dam_ch_in0 = 0x0,
	dam_ch_in1,
	dam_ch_out,
	dam_ch_maxnum
} tegra30_dam_chtype;

struct tegra30_dam_src_step_table  step_table[] = {
	{ 8000, 44100, 80 },
	{ 8000, 48000, 1 },
	{ 16000, 44100, 160 },
	{ 16000, 48000, 1 },
	{ 44100, 8000, 441 },
	{ 48000, 8000, 0 },
	{ 44100, 16000, 441 },
	{ 48000, 16000, 0 },
};

static void tegra30_dam_set_output_samplerate(struct tegra30_dam_context *dam,
		int fsout);
static void tegra30_dam_set_input_samplerate(struct tegra30_dam_context *dam,
		int fsin);
static int tegra30_dam_set_step_reset(struct tegra30_dam_context *dam,
		int insample, int outsample);
static void tegra30_dam_ch0_set_step(struct tegra30_dam_context *dam, int step);

static inline void tegra30_dam_writel(struct tegra30_dam_context *dam,
			u32 val, u32 reg)
{
#ifdef CONFIG_PM
	dam->reg_cache[reg >> 2] = val;
#endif
	__raw_writel(val, dam->damregs + reg);
}

static inline u32 tegra30_dam_readl(struct tegra30_dam_context *dam, u32 reg)
{
	u32 val = __raw_readl(dam->damregs + reg);

	return val;
}

#ifdef CONFIG_PM
int tegra30_dam_resume(int ifc)
{
	int i = 0;
	struct tegra30_dam_context *dam;

	if (ifc >= TEGRA30_NR_DAM_IFC)
		return -EINVAL;

	dam = dams_cont_info[ifc];

	if (dam->in_use) {
		tegra30_dam_enable_clock(ifc);

		for (i = 0; i <= TEGRA30_DAM_CTRL_REGINDEX; i++) {
			if ((i == TEGRA30_DAM_CTRL_RSVD_6) ||
				(i == TEGRA30_DAM_CTRL_RSVD_10))
				continue;

			tegra30_dam_writel(dam, dam->reg_cache[i],
						(i << 2));
		}

		tegra30_dam_disable_clock(ifc);
	}

	return 0;
}
#endif

int tegra30_dam_disable_clock(int ifc)
{
	struct tegra30_dam_context *dam;

	if (ifc >= TEGRA30_NR_DAM_IFC)
		return -EINVAL;

	dam =  dams_cont_info[ifc];
	clk_disable(dam->dam_clk);
	tegra30_ahub_disable_clocks();
	return 0;
}

int tegra30_dam_enable_clock(int ifc)
{
	struct tegra30_dam_context *dam;

	if (ifc >= TEGRA30_NR_DAM_IFC)
		return -EINVAL;

	dam =  dams_cont_info[ifc];
	tegra30_ahub_enable_clocks();
	clk_enable(dam->dam_clk);

	return 0;
}

#ifdef CONFIG_DEBUG_FS
static int tegra30_dam_show(struct seq_file *s, void *unused)
{
#define REG(r) { r, #r }
	static const struct {
		int offset;
		const char *name;
	} regs[] = {
		REG(TEGRA30_DAM_CTRL),
		REG(TEGRA30_DAM_CLIP),
		REG(TEGRA30_DAM_CLIP_THRESHOLD),
		REG(TEGRA30_DAM_AUDIOCIF_OUT_CTRL),
		REG(TEGRA30_DAM_CH0_CTRL),
		REG(TEGRA30_DAM_CH0_CONV),
		REG(TEGRA30_DAM_AUDIOCIF_CH0_CTRL),
		REG(TEGRA30_DAM_CH1_CTRL),
		REG(TEGRA30_DAM_CH1_CONV),
		REG(TEGRA30_DAM_AUDIOCIF_CH1_CTRL),
	};
#undef REG

	struct tegra30_dam_context *dam = s->private;
	int i;

	clk_enable(dam->dam_clk);

	for (i = 0; i < ARRAY_SIZE(regs); i++) {
		u32 val = tegra30_dam_readl(dam, regs[i].offset);
		seq_printf(s, "%s = %08x\n", regs[i].name, val);
	}

	clk_disable(dam->dam_clk);

	return 0;
}

static int tegra30_dam_debug_open(struct inode *inode, struct file *file)
{
	return single_open(file, tegra30_dam_show, inode->i_private);
}

static const struct file_operations tegra30_dam_debug_fops = {
	.open    = tegra30_dam_debug_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = single_release,
};

static void tegra30_dam_debug_add(struct tegra30_dam_context *dam, int id)
{
	char name[] = DRV_NAME ".0";

	snprintf(name, sizeof(name), DRV_NAME".%1d", id);
	dam->debug = debugfs_create_file(name, S_IRUGO, snd_soc_debugfs_root,
			dam, &tegra30_dam_debug_fops);
}

static void tegra30_dam_debug_remove(struct tegra30_dam_context *dam)
{
	if (dam->debug)
		debugfs_remove(dam->debug);
}
#else
static inline void tegra30_dam_debug_add(struct tegra30_dam_context *dam,
						int id)
{
}

static inline void tegra30_dam_debug_remove(struct tegra30_dam_context *dam)
{
}
#endif

int tegra30_dam_allocate_controller()
{
	int i = 0;
	struct tegra30_dam_context *dam = NULL;

	for (i = 0; i < TEGRA30_NR_DAM_IFC; i++) {

		dam =  dams_cont_info[i];

		if (!dam->in_use) {
			dam->in_use = true;
			return i;
		}
	}

	return -ENOENT;
}

int tegra30_dam_allocate_channel(int ifc, int chid)
{
	struct tegra30_dam_context *dam = NULL;

	if (ifc >= TEGRA30_NR_DAM_IFC)
		return -EINVAL;

	dam =  dams_cont_info[ifc];

	if (!dam->ch_alloc[chid]) {
		dam->ch_alloc[chid] = true;
		return 0;
	}

	return -ENOENT;
}

int tegra30_dam_free_channel(int ifc, int chid)
{
	struct tegra30_dam_context *dam = NULL;

	if (ifc >= TEGRA30_NR_DAM_IFC)
		return -EINVAL;

	dam =  dams_cont_info[ifc];

	if (dam->ch_alloc[chid]) {
		dam->ch_alloc[chid] = false;
		return 0;
	}

	return -EINVAL;
}

int tegra30_dam_free_controller(int ifc)
{
	struct tegra30_dam_context *dam = NULL;

	if (ifc >= TEGRA30_NR_DAM_IFC)
		return -EINVAL;

	dam =  dams_cont_info[ifc];

	if (!dam->ch_alloc[dam_ch_in0] &&
		!dam->ch_alloc[dam_ch_in1]) {
		dam->in_use = false;
		return 0;
	}

	return -EINVAL;
}

int tegra30_dam_set_samplerate(int ifc, int chid, int samplerate)
{
	struct tegra30_dam_context *dam = dams_cont_info[ifc];

	if (ifc >= TEGRA30_NR_DAM_IFC)
		return -EINVAL;

	switch (chid) {
	case dam_ch_in0:
		tegra30_dam_set_input_samplerate(dam, samplerate);
		dam->ch_insamplerate[dam_ch_in0] = samplerate;
		tegra30_dam_set_step_reset(dam, samplerate, dam->outsamplerate);
		break;
	case dam_ch_in1:
		if (samplerate != dam->outsamplerate)
			return -EINVAL;
		dam->ch_insamplerate[dam_ch_in1] = samplerate;
		break;
	case dam_ch_out:
		tegra30_dam_set_output_samplerate(dam, samplerate);
		dam->outsamplerate = samplerate;
		break;
	default:
		break;
	}
	return 0;
}

void tegra30_dam_set_output_samplerate(struct tegra30_dam_context *dam,
					int fsout)
{
	u32 val;

	val = tegra30_dam_readl(dam, TEGRA30_DAM_CTRL);
	val &= ~TEGRA30_DAM_CTRL_FSOUT_MASK;

	switch (fsout) {
	case TEGRA30_AUDIO_SAMPLERATE_8000:
		val |= TEGRA30_DAM_CTRL_FSOUT_FS8;
		break;
	case TEGRA30_AUDIO_SAMPLERATE_16000:
		val |= TEGRA30_DAM_CTRL_FSOUT_FS16;
		break;
	case TEGRA30_AUDIO_SAMPLERATE_44100:
		val |= TEGRA30_DAM_CTRL_FSOUT_FS44;
		break;
	case TEGRA30_AUDIO_SAMPLERATE_48000:
		val |= TEGRA30_DAM_CTRL_FSOUT_FS48;
		break;
	default:
		break;
	}

	tegra30_dam_writel(dam, val, TEGRA30_DAM_CTRL);
}

void tegra30_dam_set_input_samplerate(struct tegra30_dam_context *dam, int fsin)
{
	u32 val;

	val = tegra30_dam_readl(dam, TEGRA30_DAM_CH0_CTRL);
	val &= ~TEGRA30_DAM_CH0_CTRL_FSIN_MASK;

	switch (fsin) {
	case TEGRA30_AUDIO_SAMPLERATE_8000:
		val |= TEGRA30_DAM_CH0_CTRL_FSIN_FS8;
		break;
	case TEGRA30_AUDIO_SAMPLERATE_16000:
		val |= TEGRA30_DAM_CH0_CTRL_FSIN_FS16;
		break;
	case TEGRA30_AUDIO_SAMPLERATE_44100:
		val |= TEGRA30_DAM_CH0_CTRL_FSIN_FS44;
		break;
	case TEGRA30_AUDIO_SAMPLERATE_48000:
		val |= TEGRA30_DAM_CH0_CTRL_FSIN_FS48;
		break;
	default:
		break;
	}

	tegra30_dam_writel(dam, val, TEGRA30_DAM_CH0_CTRL);
}

int tegra30_dam_set_step_reset(struct tegra30_dam_context *dam,
		int insample, int outsample)
{
	int step_reset = 0;
	int i = 0;

	for (i = 0; i < ARRAY_SIZE(step_table); i++) {
		if ((insample == step_table[i].insample) &&
			(outsample == step_table[i].outsample))
			step_reset = step_table[i].stepreset;
	}

	tegra30_dam_ch0_set_step(dam, step_reset);

	return 0;
}

void tegra30_dam_ch0_set_step(struct tegra30_dam_context *dam, int step)
{
	u32 val;

	val = tegra30_dam_readl(dam, TEGRA30_DAM_CH0_CTRL);
	val &= ~TEGRA30_DAM_CH0_CTRL_STEP_MASK;
	val |= step << TEGRA30_DAM_CH0_CTRL_STEP_SHIFT;
	tegra30_dam_writel(dam, val, TEGRA30_DAM_CH0_CTRL);
}

int tegra30_dam_set_gain(int ifc, int chid, int gain)
{

	if (ifc >= TEGRA30_NR_DAM_IFC)
		return -EINVAL;

	switch (chid) {
	case dam_ch_in0:
		tegra30_dam_writel(dams_cont_info[ifc], gain,
			TEGRA30_DAM_CH0_CONV);
		break;
	case dam_ch_in1:
		tegra30_dam_writel(dams_cont_info[ifc], gain,
			TEGRA30_DAM_CH1_CONV);
		break;
	default:
		break;
	}

	return 0;
}

int tegra30_dam_set_acif(int ifc, int chid, unsigned int audio_channels,
	unsigned int audio_bits, unsigned int client_channels,
	unsigned int client_bits)
{
	unsigned int reg;
	unsigned int value = 0;

	if (ifc >= TEGRA30_NR_DAM_IFC)
		return -EINVAL;

	/*ch0 takes input as mono/16bit always*/
	if ((chid == dam_ch_in0) &&
		((client_channels != 1) || (client_bits != 16)))
		return -EINVAL;

	value |= TEGRA30_CIF_MONOCONV_COPY;
	value |= TEGRA30_CIF_STEREOCONV_CH0;
	value |= (audio_channels-1)  << TEGRA30_AUDIO_CHANNELS_SHIFT;
	value |= (((audio_bits>>2)-1)<<TEGRA30_AUDIO_BITS_SHIFT);
	value |= (client_channels-1)  << TEGRA30_CLIENT_CHANNELS_SHIFT;
	value |= (((client_bits>>2)-1)<<TEGRA30_CLIENT_BITS_SHIFT);

	switch (chid) {
	case dam_ch_out:
		value |= TEGRA30_CIF_DIRECTION_TX;
		reg = TEGRA30_DAM_AUDIOCIF_OUT_CTRL;
		break;
	case dam_ch_in0:
		value |= TEGRA30_CIF_DIRECTION_RX;
		reg = TEGRA30_DAM_AUDIOCIF_CH0_CTRL;
		break;
	case dam_ch_in1:
		value |= TEGRA30_CIF_DIRECTION_RX;
		reg = TEGRA30_DAM_AUDIOCIF_CH1_CTRL;
		break;
	default:
		return -EINVAL;
	}

	tegra30_dam_writel(dams_cont_info[ifc], value, reg);

	return 0;
}

int tegra30_dam_enable(int ifc, int on, int chid)
{
	u32 old_val, val, enreg;
	struct tegra30_dam_context *dam = dams_cont_info[ifc];

	if (ifc >= TEGRA30_NR_DAM_IFC)
		return -EINVAL;

	if (chid == dam_ch_in0)
		enreg = TEGRA30_DAM_CH0_CTRL;
	else
		enreg = TEGRA30_DAM_CH1_CTRL;

	old_val = val = tegra30_dam_readl(dam, enreg);

	if (on) {
		if (!dam->ch_enable_refcnt[chid]++)
			val |= TEGRA30_DAM_CH0_CTRL_EN;
	} else if (dam->ch_enable_refcnt[chid]) {
		dam->ch_enable_refcnt[chid]--;
		if (!dam->ch_enable_refcnt[chid])
			val &= ~TEGRA30_DAM_CH0_CTRL_EN;
	}

	if (val != old_val)
		tegra30_dam_writel(dam, val, enreg);

	old_val = val = tegra30_dam_readl(dam, TEGRA30_DAM_CTRL);

	if (dam->ch_enable_refcnt[dam_ch_in0] ||
		dam->ch_enable_refcnt[dam_ch_in1])
		val |= TEGRA30_DAM_CTRL_DAM_EN;
	else
		val &= ~TEGRA30_DAM_CTRL_DAM_EN;

	if (old_val != val)
		tegra30_dam_writel(dam, val, TEGRA30_DAM_CTRL);
	return 0;
}

void tegra30_dam_ch0_set_datasync(struct tegra30_dam_context *dam, int datasync)
{
	u32 val;

	val = tegra30_dam_readl(dam, TEGRA30_DAM_CH0_CTRL);
	val &= ~TEGRA30_DAM_CH0_CTRL_DATA_SYNC_MASK;
	val |= datasync << TEGRA30_DAM_DATA_SYNC_SHIFT;
	tegra30_dam_writel(dam, val, TEGRA30_DAM_CH0_CTRL);
}

void tegra30_dam_ch1_set_datasync(struct tegra30_dam_context *dam, int datasync)
{
	u32 val;

	val = tegra30_dam_readl(dam, TEGRA30_DAM_CH1_CTRL);
	val &= ~TEGRA30_DAM_CH1_CTRL_DATA_SYNC_MASK;
	val |= datasync << TEGRA30_DAM_DATA_SYNC_SHIFT;
	tegra30_dam_writel(dam, val, TEGRA30_DAM_CH1_CTRL);
}

void tegra30_dam_enable_clip_counter(struct tegra30_dam_context *dam, int on)
{
	u32 val;

	val = tegra30_dam_readl(dam, TEGRA30_DAM_CLIP);
	val &= ~TEGRA30_DAM_CLIP_COUNTER_ENABLE;
	val |= on ?  TEGRA30_DAM_CLIP_COUNTER_ENABLE : 0;
	tegra30_dam_writel(dam, val, TEGRA30_DAM_CLIP);
}

static int __devinit tegra30_dam_probe(struct platform_device *pdev)
{
	struct resource *res,  *region;
	struct tegra30_dam_context *dam;
	int ret = 0;
#ifdef CONFIG_PM
	int i;
#endif
	int clkm_rate;

	if ((pdev->id < 0) ||
		(pdev->id >= TEGRA30_NR_DAM_IFC)) {
		dev_err(&pdev->dev, "ID %d out of range\n", pdev->id);
		return -EINVAL;
	}

	dams_cont_info[pdev->id] = devm_kzalloc(&pdev->dev,
					sizeof(struct tegra30_dam_context),
					GFP_KERNEL);
	if (!dams_cont_info[pdev->id]) {
		dev_err(&pdev->dev, "Can't allocate dam context\n");
		ret = -ENOMEM;
		goto exit;
	}
	dam = dams_cont_info[pdev->id];

	dam->dam_clk = clk_get(&pdev->dev, NULL);
	if (IS_ERR(dam->dam_clk)) {
		dev_err(&pdev->dev, "Can't retrieve dam clock\n");
		ret = PTR_ERR(dam->dam_clk);
		goto err_free;
	}
	clkm_rate = clk_get_rate(clk_get_parent(dam->dam_clk));
	while (clkm_rate > 12000000)
		clkm_rate >>= 1;

	clk_set_rate(dam->dam_clk,clkm_rate);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "No memory 0 resource\n");
		ret = -ENODEV;
		goto err_clk_put_dam;
	}

	region = devm_request_mem_region(&pdev->dev, res->start,
			resource_size(res), pdev->name);
	if (!region) {
		dev_err(&pdev->dev, "Memory region 0 already claimed\n");
		ret = -EBUSY;
		goto err_clk_put_dam;
	}

	dam->damregs = devm_ioremap(&pdev->dev, res->start, resource_size(res));
	if (!dam->damregs) {
		dev_err(&pdev->dev, "ioremap 0 failed\n");
		ret = -ENOMEM;
		goto err_clk_put_dam;
	}

#ifdef CONFIG_PM
	/* cache the POR values of DAM regs*/
	tegra30_dam_enable_clock(pdev->id);

	for (i = 0; i <= TEGRA30_DAM_CTRL_REGINDEX; i++) {
		if ((i == TEGRA30_DAM_CTRL_RSVD_6) ||
			(i == TEGRA30_DAM_CTRL_RSVD_10))
			continue;

			dam->reg_cache[i] =
				tegra30_dam_readl(dam, i << 2);
	}

	tegra30_dam_disable_clock(pdev->id);
#endif

	platform_set_drvdata(pdev, dam);

	tegra30_dam_debug_add(dam, pdev->id);

	return 0;

err_clk_put_dam:
	clk_put(dam->dam_clk);
err_free:
	dams_cont_info[pdev->id] = NULL;
exit:
	return ret;
}

static int __devexit tegra30_dam_remove(struct platform_device *pdev)
{
	struct tegra30_dam_context *dam;

	dam = platform_get_drvdata(pdev);
	clk_put(dam->dam_clk);
	tegra30_dam_debug_remove(dam);
	dams_cont_info[pdev->id] = NULL;

	return 0;
}

static struct platform_driver tegra30_dam_driver = {
	.probe = tegra30_dam_probe,
	.remove = __devexit_p(tegra30_dam_remove),
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
	},
};

static int __init tegra30_dam_modinit(void)
{
	return platform_driver_register(&tegra30_dam_driver);
}
module_init(tegra30_dam_modinit);

static void __exit tegra30_dam_modexit(void)
{
	platform_driver_unregister(&tegra30_dam_driver);
}
module_exit(tegra30_dam_modexit);

MODULE_AUTHOR("Nikesh Oswal <noswal@nvidia.com>");
MODULE_DESCRIPTION("Tegra 30 DAM driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRV_NAME);
