/*
 * drivers/powergate/tegra-powergate.c
 *
 * Copyright (c) 2010 Google, Inc
 *
 * Author:
 *	Colin Cross <ccross@google.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/seq_file.h>
#include <linux/spinlock.h>

#include <mach/clk.h>
#include <mach/iomap.h>
#include <mach/powergate.h>

#define PWRGATE_TOGGLE		0x30
#define  PWRGATE_TOGGLE_START	(1 << 8)

#define REMOVE_CLAMPING		0x34

#define PWRGATE_STATUS		0x38

#define MC_CLIENT_HOTRESET_CTRL	0x200
#define MC_CLIENT_HOTRESET_STAT	0x204

typedef enum {
	MC_CLIENT_AFI	= 0,
	MC_CLIENT_AVPC	= 1,
	MC_CLIENT_DC	= 2,
	MC_CLIENT_DCB	= 3,
	MC_CLIENT_EPP	= 4,
	MC_CLIENT_G2	= 5,
	MC_CLIENT_HC	= 6,
	MC_CLIENT_HDA	= 7,
	MC_CLIENT_ISP	= 8,
	MC_CLIENT_MPCORE	= 9,
	MC_CLIENT_MPCORELP	= 10,
	MC_CLIENT_MPE	= 11,
	MC_CLIENT_NV	= 12,
	MC_CLIENT_NV2	= 13,
	MC_CLIENT_PPCS	= 14,
	MC_CLIENT_SATA	= 15,
	MC_CLIENT_VDE	= 16,
	MC_CLIENT_VI	= 17,
	MC_CLIENT_LAST	= -1,
} MC_CLIENT;

static DEFINE_SPINLOCK(tegra_powergate_lock);

#define MAX_HOTRESET_CLIENT_NUM		3

typedef struct {
	const char * name;
	MC_CLIENT hot_reset_clients[MAX_HOTRESET_CLIENT_NUM];
	/* add clocks for each partition*/
} powergate_partition;

static powergate_partition powergate_partition_info[TEGRA_NUM_POWERGATE] = {
	[TEGRA_POWERGATE_CPU]	= { "cpu0",	{MC_CLIENT_LAST} },
	[TEGRA_POWERGATE_L2]	= { "l2",	{MC_CLIENT_LAST} },
	[TEGRA_POWERGATE_3D]	= { "3d0",
						{MC_CLIENT_NV, MC_CLIENT_LAST} },
	[TEGRA_POWERGATE_PCIE]	= { "pcie",
						{MC_CLIENT_AFI, MC_CLIENT_LAST} },
	[TEGRA_POWERGATE_VDEC]	= { "vde",
						{MC_CLIENT_VDE, MC_CLIENT_LAST} },
	[TEGRA_POWERGATE_MPE]	= { "mpe",
						{MC_CLIENT_MPE, MC_CLIENT_LAST} },
	[TEGRA_POWERGATE_VENC]	= { "ve",
						{MC_CLIENT_ISP, MC_CLIENT_VI, MC_CLIENT_LAST} },
#if !defined(CONFIG_ARCH_TEGRA_2x_SOC)
	[TEGRA_POWERGATE_CPU1]	= { "cpu1",	{MC_CLIENT_LAST}},
	[TEGRA_POWERGATE_CPU2]	= { "cpu2",	{MC_CLIENT_LAST}},
	[TEGRA_POWERGATE_CPU3]	= { "cpu3",	{MC_CLIENT_LAST}},
	[TEGRA_POWERGATE_A9LP]	= { "a9lp",	{MC_CLIENT_LAST}},
	[TEGRA_POWERGATE_SATA]	= { "sata",
						{MC_CLIENT_SATA, MC_CLIENT_LAST} },
	[TEGRA_POWERGATE_3D1]	= { "3d1",
						{MC_CLIENT_NV2, MC_CLIENT_LAST} },
	[TEGRA_POWERGATE_HEG]	= { "heg",
						{MC_CLIENT_G2, MC_CLIENT_EPP, MC_CLIENT_HC} },
#endif
};

static void __iomem *pmc = IO_ADDRESS(TEGRA_PMC_BASE);

static u32 pmc_read(unsigned long reg)
{
	return readl(pmc + reg);
}

static void pmc_write(u32 val, unsigned long reg)
{
	writel(val, pmc + reg);
}

#if !defined(CONFIG_ARCH_TEGRA_2x_SOC)
static void __iomem *mc = IO_ADDRESS(TEGRA_MC_BASE);

static u32 mc_read(unsigned long reg)
{
	return readl(mc + reg);
}

static void mc_write(u32 val, unsigned long reg)
{
	writel(val, mc + reg);
}

static void mc_flush(int id)
{
	u32 idx, rst_ctrl, rst_stat;
	MC_CLIENT mcClientBit;
	unsigned long flags;

	BUG_ON(id < 0 || id >= TEGRA_NUM_POWERGATE);

	for (idx = 0; idx < MAX_HOTRESET_CLIENT_NUM; idx++) {
		mcClientBit = powergate_partition_info[id].hot_reset_clients[idx];
		if (mcClientBit == MC_CLIENT_LAST)
			break;

		spin_lock_irqsave(&tegra_powergate_lock, flags);

		rst_ctrl = mc_read(MC_CLIENT_HOTRESET_CTRL);
		rst_ctrl |= (1 << mcClientBit);
		mc_write(rst_ctrl, MC_CLIENT_HOTRESET_CTRL);

		spin_unlock_irqrestore(&tegra_powergate_lock, flags);

		do {
			udelay(10);
			rst_stat = mc_read(MC_CLIENT_HOTRESET_STAT);
		} while (!(rst_stat & (1 << mcClientBit)));
	}
}

static void mc_flush_done(int id)
{
	u32 idx, rst_ctrl;
	MC_CLIENT mcClientBit;
	unsigned long flags;

	BUG_ON(id < 0 || id >= TEGRA_NUM_POWERGATE);

	for (idx = 0; idx < MAX_HOTRESET_CLIENT_NUM; idx++) {
		mcClientBit = powergate_partition_info[id].hot_reset_clients[idx];
		if (mcClientBit == MC_CLIENT_LAST)
			break;

		spin_lock_irqsave(&tegra_powergate_lock, flags);

		rst_ctrl = mc_read(MC_CLIENT_HOTRESET_CTRL);
		rst_ctrl &= ~(1 << mcClientBit);
		mc_write(rst_ctrl, MC_CLIENT_HOTRESET_CTRL);

		spin_unlock_irqrestore(&tegra_powergate_lock, flags);
	}

	wmb();
}
#else
static void mc_flush(int id) {}
static void mc_flush_done(int id) {}
#endif

static int tegra_powergate_set(int id, bool new_state)
{
	bool status;
	unsigned long flags;

	spin_lock_irqsave(&tegra_powergate_lock, flags);

	status = !!(pmc_read(PWRGATE_STATUS) & (1 << id));

	if (status == new_state) {
		spin_unlock_irqrestore(&tegra_powergate_lock, flags);
		return -EINVAL;
	}

	pmc_write(PWRGATE_TOGGLE_START | id, PWRGATE_TOGGLE);

	spin_unlock_irqrestore(&tegra_powergate_lock, flags);

	return 0;
}

int tegra_powergate_power_on(int id)
{
	if (id < 0 || id >= TEGRA_NUM_POWERGATE)
		return -EINVAL;

	return tegra_powergate_set(id, true);
}

int tegra_powergate_power_off(int id)
{
	if (id < 0 || id >= TEGRA_NUM_POWERGATE)
		return -EINVAL;

	mc_flush(id);

	return tegra_powergate_set(id, false);
}

bool tegra_powergate_is_powered(int id)
{
	u32 status;

	if (id < 0 || id >= TEGRA_NUM_POWERGATE)
		return false;

	status = pmc_read(PWRGATE_STATUS) & (1 << id);
	return !!status;
}

int tegra_powergate_remove_clamping(int id)
{
	u32 mask;

	if (id < 0 || id >= TEGRA_NUM_POWERGATE)
		return -EINVAL;

	/*
	 * PCIE and VDE clamping masks are swapped with respect to their
	 * partition ids
	 */
	if (id ==  TEGRA_POWERGATE_VDEC)
		mask = (1 << TEGRA_POWERGATE_PCIE);
	else if (id == TEGRA_POWERGATE_PCIE)
		mask = (1 << TEGRA_POWERGATE_VDEC);
	else
		mask = (1 << id);

	pmc_write(mask, REMOVE_CLAMPING);

	return 0;
}

/* Must be called with clk disabled, and returns with clk enabled */
static int tegra_powergate_reset_module(struct clk *clk)
{
	int ret;

	tegra_periph_reset_assert(clk);

	udelay(10);

	ret = clk_enable(clk);
	if (ret)
		return ret;

	udelay(10);

	tegra_periph_reset_deassert(clk);

	return 0;
}

/* Must be called with clk disabled, and returns with clk enabled */
int tegra_powergate_sequence_power_up(int id, struct clk *clk)
{
	int ret;

	if (tegra_powergate_is_powered(id))
		return tegra_powergate_reset_module(clk);

	tegra_periph_reset_assert(clk);

	ret = tegra_powergate_power_on(id);
	if (ret)
		goto err_power;

	ret = clk_enable(clk);
	if (ret)
		goto err_clk;

	udelay(10);

	ret = tegra_powergate_remove_clamping(id);
	if (ret)
		goto err_clamp;

	udelay(10);
	tegra_periph_reset_deassert(clk);

	mc_flush_done(id);

	return 0;

err_clamp:
	clk_disable(clk);
err_clk:
	tegra_powergate_power_off(id);
err_power:
	return ret;
}

const char* tegra_powergate_get_name(int id)
{
	if (id < 0 || id >= TEGRA_NUM_POWERGATE)
		return "invalid";

	return powergate_partition_info[id].name;
}

#ifdef CONFIG_DEBUG_FS

static int powergate_show(struct seq_file *s, void *data)
{
	int i;

	seq_printf(s, " powergate powered\n");
	seq_printf(s, "------------------\n");

	for (i = 0; i < TEGRA_NUM_POWERGATE; i++)
		seq_printf(s, " %9s %7s\n", powergate_partition_info[i].name,
			tegra_powergate_is_powered(i) ? "yes" : "no");
	return 0;
}

static int powergate_open(struct inode *inode, struct file *file)
{
	return single_open(file, powergate_show, inode->i_private);
}

static const struct file_operations powergate_fops = {
	.open		= powergate_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int __init powergate_debugfs_init(void)
{
	struct dentry *d;

	d = debugfs_create_file("powergate", S_IRUGO, NULL, NULL,
		&powergate_fops);
	if (!d)
		return -ENOMEM;

	return 0;
}

late_initcall(powergate_debugfs_init);

#endif
