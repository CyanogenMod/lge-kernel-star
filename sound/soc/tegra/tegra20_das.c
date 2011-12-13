/*
 * tegra20_das.c - Tegra20 DAS driver
 *
 * Author: Stephen Warren <swarren@nvidia.com>
 * Copyright (C) 2010 - NVIDIA, Inc.
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

#include <linux/module.h>
#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <mach/iomap.h>
#include <sound/soc.h>
#include "tegra20_das.h"

#define DRV_NAME "tegra20-das"

static struct tegra20_das *das;

static inline void tegra20_das_write(u32 reg, u32 val)
{
#ifdef CONFIG_PM
	das->reg_cache[reg >> 2] = val;
#endif
	__raw_writel(val, das->regs + reg);
}

static inline u32 tegra20_das_read(u32 reg)
{
	return __raw_readl(das->regs + reg);
}

#ifdef CONFIG_PM
int tegra20_das_resume()
{
	int i, reg;

	for (i = 0; i <= TEGRA20_DAS_DAP_ID_5; i++)
		tegra20_das_write(i << 2, das->reg_cache[i]);

	for (i = 0; i <= TEGRA20_DAS_DAC_ID_3; i++) {
		reg = TEGRA20_DAS_DAC_INPUT_DATA_CLK_SEL +
			(i * TEGRA20_DAS_DAC_INPUT_DATA_CLK_SEL_STRIDE);
		tegra20_das_write(reg, das->reg_cache[reg >> 2]);
	}

	return 0;
}
#endif

int tegra20_das_connect_dap_to_dac(int dap, int dac)
{
	u32 addr;
	u32 reg;

	if (!das)
		return -ENODEV;

	addr = TEGRA20_DAS_DAP_CTRL_SEL +
		(dap * TEGRA20_DAS_DAP_CTRL_SEL_STRIDE);
	reg = dac << TEGRA20_DAS_DAP_CTRL_SEL_DAP_CTRL_SEL_P;

	tegra20_das_write(addr, reg);

	return 0;
}
EXPORT_SYMBOL_GPL(tegra20_das_connect_dap_to_dac);

int tegra20_das_connect_dap_to_dap(int dap, int otherdap, int master,
					int sdata1rx, int sdata2rx)
{
	u32 addr;
	u32 reg;

	if (!das)
		return -ENODEV;

	addr = TEGRA20_DAS_DAP_CTRL_SEL +
		(dap * TEGRA20_DAS_DAP_CTRL_SEL_STRIDE);
	reg = otherdap << TEGRA20_DAS_DAP_CTRL_SEL_DAP_CTRL_SEL_P |
		!!sdata2rx << TEGRA20_DAS_DAP_CTRL_SEL_DAP_SDATA2_TX_RX_P |
		!!sdata1rx << TEGRA20_DAS_DAP_CTRL_SEL_DAP_SDATA1_TX_RX_P |
		!!master << TEGRA20_DAS_DAP_CTRL_SEL_DAP_MS_SEL_P;

	tegra20_das_write(addr, reg);

	return 0;
}
EXPORT_SYMBOL_GPL(tegra20_das_connect_dap_to_dap);

int tegra20_das_connect_dac_to_dap(int dac, int dap)
{
	u32 addr;
	u32 reg;

	if (!das)
		return -ENODEV;

	addr = TEGRA20_DAS_DAC_INPUT_DATA_CLK_SEL +
		(dac * TEGRA20_DAS_DAC_INPUT_DATA_CLK_SEL_STRIDE);
	reg = dap << TEGRA20_DAS_DAC_INPUT_DATA_CLK_SEL_DAC_CLK_SEL_P |
		dap << TEGRA20_DAS_DAC_INPUT_DATA_CLK_SEL_DAC_SDATA1_SEL_P |
		dap << TEGRA20_DAS_DAC_INPUT_DATA_CLK_SEL_DAC_SDATA2_SEL_P;

	tegra20_das_write(addr, reg);

	return 0;
}
EXPORT_SYMBOL_GPL(tegra20_das_connect_dac_to_dap);

#ifdef CONFIG_DEBUG_FS
static int tegra20_das_show(struct seq_file *s, void *unused)
{
	int i;
	u32 addr;
	u32 reg;

	for (i = 0; i < TEGRA20_DAS_DAP_CTRL_SEL_COUNT; i++) {
		addr = TEGRA20_DAS_DAP_CTRL_SEL +
			(i * TEGRA20_DAS_DAP_CTRL_SEL_STRIDE);
		reg = tegra20_das_read(addr);
		seq_printf(s, "TEGRA20_DAS_DAP_CTRL_SEL[%d] = %08x\n", i, reg);
	}

	for (i = 0; i < TEGRA20_DAS_DAC_INPUT_DATA_CLK_SEL_COUNT; i++) {
		addr = TEGRA20_DAS_DAC_INPUT_DATA_CLK_SEL +
			(i * TEGRA20_DAS_DAC_INPUT_DATA_CLK_SEL_STRIDE);
		reg = tegra20_das_read(addr);
		seq_printf(s, "TEGRA20_DAS_DAC_INPUT_DATA_CLK_SEL[%d] = %08x\n",
				 i, reg);
	}

	return 0;
}

static int tegra20_das_debug_open(struct inode *inode, struct file *file)
{
	return single_open(file, tegra20_das_show, inode->i_private);
}

static const struct file_operations tegra20_das_debug_fops = {
	.open    = tegra20_das_debug_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = single_release,
};

static void tegra20_das_debug_add(struct tegra20_das *das)
{
	das->debug = debugfs_create_file(DRV_NAME, S_IRUGO,
					 snd_soc_debugfs_root, das,
					 &tegra20_das_debug_fops);
}

static void tegra20_das_debug_remove(struct tegra20_das *das)
{
	if (das->debug)
		debugfs_remove(das->debug);
}
#else
static inline void tegra20_das_debug_add(struct tegra20_das *das)
{
}

static inline void tegra20_das_debug_remove(struct tegra20_das *das)
{
}
#endif

static int __devinit tegra20_das_probe(struct platform_device *pdev)
{
	struct resource *res, *region;
	int ret = 0;
#ifdef CONFIG_PM
	int i, reg;
#endif

	if (das)
		return -ENODEV;

	das = kzalloc(sizeof(struct tegra20_das), GFP_KERNEL);
	if (!das) {
		dev_err(&pdev->dev, "Can't allocate tegra20_das\n");
		ret = -ENOMEM;
		goto exit;
	}
	das->dev = &pdev->dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "No memory resource\n");
		ret = -ENODEV;
		goto err_free;
	}

	region = request_mem_region(res->start, resource_size(res),
					pdev->name);
	if (!region) {
		dev_err(&pdev->dev, "Memory region already claimed\n");
		ret = -EBUSY;
		goto err_free;
	}

	das->regs = ioremap(res->start, resource_size(res));
	if (!das->regs) {
		dev_err(&pdev->dev, "ioremap failed\n");
		ret = -ENOMEM;
		goto err_release;
	}

#ifdef CONFIG_PM
	/* populate the das reg cache with POR values*/
	for (i = 0; i <= TEGRA20_DAS_DAP_ID_5; i++)
		das->reg_cache[i] = tegra20_das_read(i << 2);

	for (i = 0; i <= TEGRA20_DAS_DAC_ID_3; i++) {
		reg = TEGRA20_DAS_DAC_INPUT_DATA_CLK_SEL +
			(i * TEGRA20_DAS_DAC_INPUT_DATA_CLK_SEL_STRIDE);
		das->reg_cache[reg >> 2] = tegra20_das_read(reg);
	}
#endif

	tegra20_das_debug_add(das);

	platform_set_drvdata(pdev, das);

	return 0;

err_release:
	release_mem_region(res->start, resource_size(res));
err_free:
	kfree(das);
	das = 0;
exit:
	return ret;
}

static int __devexit tegra20_das_remove(struct platform_device *pdev)
{
	struct resource *res;

	if (!das)
		return -ENODEV;

	platform_set_drvdata(pdev, NULL);

	tegra20_das_debug_remove(das);

	iounmap(das->regs);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(res->start, resource_size(res));

	kfree(das);
	das = 0;

	return 0;
}

static struct platform_driver tegra20_das_driver = {
	.probe = tegra20_das_probe,
	.remove = __devexit_p(tegra20_das_remove),
	.driver = {
		.name = DRV_NAME,
	},
};

static int __init tegra20_das_modinit(void)
{
	return platform_driver_register(&tegra20_das_driver);
}
module_init(tegra20_das_modinit);

static void __exit tegra20_das_modexit(void)
{
	platform_driver_unregister(&tegra20_das_driver);
}
module_exit(tegra20_das_modexit);

MODULE_AUTHOR("Stephen Warren <swarren@nvidia.com>");
MODULE_DESCRIPTION("Tegra DAS driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRV_NAME);
