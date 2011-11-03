/*
 * drivers/powergate/tegra-powergate.c
 *
 * Copyright (c) 2010 Google, Inc
 * Copyright (C) 2011 NVIDIA Corporation.
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

#include "clock.h"

#define PWRGATE_TOGGLE		0x30
#define PWRGATE_TOGGLE_START	(1 << 8)

#define REMOVE_CLAMPING		0x34

#define PWRGATE_STATUS		0x38

#if !defined(CONFIG_ARCH_TEGRA_2x_SOC)
enum mc_client {
	MC_CLIENT_AFI		= 0,
	MC_CLIENT_AVPC		= 1,
	MC_CLIENT_DC		= 2,
	MC_CLIENT_DCB		= 3,
	MC_CLIENT_EPP		= 4,
	MC_CLIENT_G2		= 5,
	MC_CLIENT_HC		= 6,
	MC_CLIENT_HDA		= 7,
	MC_CLIENT_ISP		= 8,
	MC_CLIENT_MPCORE	= 9,
	MC_CLIENT_MPCORELP	= 10,
	MC_CLIENT_MPE		= 11,
	MC_CLIENT_NV		= 12,
	MC_CLIENT_NV2		= 13,
	MC_CLIENT_PPCS		= 14,
	MC_CLIENT_SATA		= 15,
	MC_CLIENT_VDE		= 16,
	MC_CLIENT_VI		= 17,
	MC_CLIENT_LAST		= -1,
};
#else
enum mc_client {
	MC_CLIENT_AVPC		= 0,
	MC_CLIENT_DC		= 1,
	MC_CLIENT_DCB		= 2,
	MC_CLIENT_EPP		= 3,
	MC_CLIENT_G2		= 4,
	MC_CLIENT_HC		= 5,
	MC_CLIENT_ISP		= 6,
	MC_CLIENT_MPCORE	= 7,
	MC_CLIENT_MPEA		= 8,
	MC_CLIENT_MPEB		= 9,
	MC_CLIENT_MPEC		= 10,
	MC_CLIENT_NV		= 11,
	MC_CLIENT_PPCS		= 12,
	MC_CLIENT_VDE		= 13,
	MC_CLIENT_VI		= 14,
	MC_CLIENT_LAST		= -1,
	MC_CLIENT_AFI		= MC_CLIENT_LAST,
};
#endif

#define MAX_CLK_EN_NUM			4

static DEFINE_SPINLOCK(tegra_powergate_lock);

#define MAX_HOTRESET_CLIENT_NUM		4

enum clk_type {
	CLK_AND_RST,
	RST_ONLY,
	CLK_ONLY,
};

struct partition_clk_info {
	const char *clk_name;
	enum clk_type clk_type;
	/* true if clk is only used in assert/deassert reset and not while enable-den*/
	struct clk *clk_ptr;
};

struct powergate_partition {
	const char *name;
	enum mc_client hot_reset_clients[MAX_HOTRESET_CLIENT_NUM];
	struct partition_clk_info clk_info[MAX_CLK_EN_NUM];
};

static struct powergate_partition powergate_partition_info[TEGRA_NUM_POWERGATE] = {
	[TEGRA_POWERGATE_CPU]	= { "cpu0",	{MC_CLIENT_LAST}, },
	[TEGRA_POWERGATE_L2]	= { "l2",	{MC_CLIENT_LAST}, },
	[TEGRA_POWERGATE_3D]	= { "3d0",
						{MC_CLIENT_NV, MC_CLIENT_LAST},
						{{"3d", CLK_AND_RST} }, },
	[TEGRA_POWERGATE_PCIE]	= { "pcie",
						{MC_CLIENT_AFI, MC_CLIENT_LAST},
						{{"afi", CLK_AND_RST},
						{"pcie", CLK_AND_RST},
						{"pciex", RST_ONLY} }, },
	[TEGRA_POWERGATE_VDEC]	= { "vde",
						{MC_CLIENT_VDE, MC_CLIENT_LAST},
						{{"vde", CLK_AND_RST} }, },
	[TEGRA_POWERGATE_MPE]	= { "mpe",
#ifndef CONFIG_ARCH_TEGRA_2x_SOC
					{MC_CLIENT_MPE, MC_CLIENT_LAST},
#else
					{MC_CLIENT_MPEA, MC_CLIENT_MPEB,
					 MC_CLIENT_MPEC, MC_CLIENT_LAST},
#endif
						{{"mpe", CLK_AND_RST} }, },
	[TEGRA_POWERGATE_VENC]	= { "ve",
						{MC_CLIENT_ISP, MC_CLIENT_VI, MC_CLIENT_LAST},
						{{"isp", CLK_AND_RST},
						{"vi", CLK_AND_RST},
						{"csi", CLK_AND_RST} }, },
#if !defined(CONFIG_ARCH_TEGRA_2x_SOC)
	[TEGRA_POWERGATE_CPU1]	= { "cpu1",	{MC_CLIENT_LAST}, },
	[TEGRA_POWERGATE_CPU2]	= { "cpu2",	{MC_CLIENT_LAST}, },
	[TEGRA_POWERGATE_CPU3]	= { "cpu3",	{MC_CLIENT_LAST}, },
	[TEGRA_POWERGATE_CELP]	= { "celp",	{MC_CLIENT_LAST}, },
	[TEGRA_POWERGATE_SATA]	= { "sata",     {MC_CLIENT_SATA, MC_CLIENT_LAST},
						{{"sata", CLK_AND_RST},
						{"sata_oob", CLK_AND_RST},
						{"cml1", CLK_ONLY},
						{"sata_cold", RST_ONLY} }, },
	[TEGRA_POWERGATE_3D1]	= { "3d1",
						{MC_CLIENT_NV2, MC_CLIENT_LAST},
						{{"3d2", CLK_AND_RST} }, },
	[TEGRA_POWERGATE_HEG]	= { "heg",
						{MC_CLIENT_G2, MC_CLIENT_EPP,
							MC_CLIENT_HC,
							MC_CLIENT_LAST},
						{{"2d", CLK_AND_RST},
						{"epp", CLK_AND_RST},
						{"host1x", CLK_AND_RST},
						{"3d", RST_ONLY} }, },
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

static void __iomem *mc = IO_ADDRESS(TEGRA_MC_BASE);

static u32 mc_read(unsigned long reg)
{
	return readl(mc + reg);
}

static void mc_write(u32 val, unsigned long reg)
{
	writel(val, mc + reg);
}

#if !defined(CONFIG_ARCH_TEGRA_2x_SOC)

#define MC_CLIENT_HOTRESET_CTRL	0x200
#define MC_CLIENT_HOTRESET_STAT	0x204

static void mc_flush(int id)
{
	u32 idx, rst_ctrl, rst_stat;
	enum mc_client mcClientBit;
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
	enum mc_client mcClientBit;
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

int tegra_powergate_mc_flush(int id)
{
	if (id < 0 || id >= TEGRA_NUM_POWERGATE)
		return -EINVAL;
	mc_flush(id);
	return 0;
}

int tegra_powergate_mc_flush_done(int id)
{
	if (id < 0 || id >= TEGRA_NUM_POWERGATE)
		return -EINVAL;
	mc_flush_done(id);
	return 0;
}

int tegra_powergate_mc_disable(int id)
{
	return 0;
}

int tegra_powergate_mc_enable(int id)
{
	return 0;
}

#else

#define MC_CLIENT_CTRL		0x100
#define MC_CLIENT_HOTRESETN	0x104
#define MC_CLIENT_ORRC_BASE	0x140

int tegra_powergate_mc_disable(int id)
{
	u32 idx, clt_ctrl, orrc_reg;
	enum mc_client mcClientBit;
	unsigned long flags;

	if (id < 0 || id >= TEGRA_NUM_POWERGATE) {
		WARN_ON(1);
		return -EINVAL;
	}

	for (idx = 0; idx < MAX_HOTRESET_CLIENT_NUM; idx++) {
		mcClientBit =
			powergate_partition_info[id].hot_reset_clients[idx];
		if (mcClientBit == MC_CLIENT_LAST)
			break;

		spin_lock_irqsave(&tegra_powergate_lock, flags);

		/* clear client enable bit */
		clt_ctrl = mc_read(MC_CLIENT_CTRL);
		clt_ctrl &= ~(1 << mcClientBit);
		mc_write(clt_ctrl, MC_CLIENT_CTRL);

		/* read back to flush write */
		clt_ctrl = mc_read(MC_CLIENT_CTRL);

		spin_unlock_irqrestore(&tegra_powergate_lock, flags);

		/* wait for outstanding requests to reach 0 */
		orrc_reg = MC_CLIENT_ORRC_BASE + (mcClientBit * 4);
		while (mc_read(orrc_reg) != 0)
			udelay(10);
	}
	return 0;
}

int tegra_powergate_mc_flush(int id)
{
	u32 idx, hot_rstn;
	enum mc_client mcClientBit;
	unsigned long flags;

	if (id < 0 || id >= TEGRA_NUM_POWERGATE) {
		WARN_ON(1);
		return -EINVAL;
	}

	for (idx = 0; idx < MAX_HOTRESET_CLIENT_NUM; idx++) {
		mcClientBit =
			powergate_partition_info[id].hot_reset_clients[idx];
		if (mcClientBit == MC_CLIENT_LAST)
			break;

		spin_lock_irqsave(&tegra_powergate_lock, flags);

		/* assert hotreset (client module is currently in reset) */
		hot_rstn = mc_read(MC_CLIENT_HOTRESETN);
		hot_rstn &= ~(1 << mcClientBit);
		mc_write(hot_rstn, MC_CLIENT_HOTRESETN);

		/* read back to flush write */
		hot_rstn = mc_read(MC_CLIENT_HOTRESETN);

		spin_unlock_irqrestore(&tegra_powergate_lock, flags);
	}
	return 0;
}

int tegra_powergate_mc_flush_done(int id)
{
	u32 idx, hot_rstn;
	enum mc_client mcClientBit;
	unsigned long flags;

	if (id < 0 || id >= TEGRA_NUM_POWERGATE) {
		WARN_ON(1);
		return -EINVAL;
	}

	for (idx = 0; idx < MAX_HOTRESET_CLIENT_NUM; idx++) {
		mcClientBit =
			powergate_partition_info[id].hot_reset_clients[idx];
		if (mcClientBit == MC_CLIENT_LAST)
			break;

		spin_lock_irqsave(&tegra_powergate_lock, flags);

		/* deassert hotreset */
		hot_rstn = mc_read(MC_CLIENT_HOTRESETN);
		hot_rstn |= (1 << mcClientBit);
		mc_write(hot_rstn, MC_CLIENT_HOTRESETN);

		/* read back to flush write */
		hot_rstn = mc_read(MC_CLIENT_HOTRESETN);

		spin_unlock_irqrestore(&tegra_powergate_lock, flags);
	}
	return 0;
}

int tegra_powergate_mc_enable(int id)
{
	u32 idx, clt_ctrl;
	enum mc_client mcClientBit;
	unsigned long flags;

	if (id < 0 || id >= TEGRA_NUM_POWERGATE) {
		WARN_ON(1);
		return -EINVAL;
	}

	for (idx = 0; idx < MAX_HOTRESET_CLIENT_NUM; idx++) {
		mcClientBit =
			powergate_partition_info[id].hot_reset_clients[idx];
		if (mcClientBit == MC_CLIENT_LAST)
			break;

		spin_lock_irqsave(&tegra_powergate_lock, flags);

		/* enable client */
		clt_ctrl = mc_read(MC_CLIENT_CTRL);
		clt_ctrl |= (1 << mcClientBit);
		mc_write(clt_ctrl, MC_CLIENT_CTRL);

		/* read back to flush write */
		clt_ctrl = mc_read(MC_CLIENT_CTRL);

		spin_unlock_irqrestore(&tegra_powergate_lock, flags);
	}
	return 0;
}

static void mc_flush(int id) {}
static void mc_flush_done(int id) {}
#endif

static int tegra_powergate_set(int id, bool new_state)
{
	bool status;
	unsigned long flags;
	/* 10us timeout for toggle operation if it takes affect*/
	int toggle_timeout = 10;
	/* 100 * 10 = 1000us timeout for toggle command to take affect in case
	   of contention with h/w initiated CPU power gating */
	int contention_timeout = 100;

	spin_lock_irqsave(&tegra_powergate_lock, flags);

	status = !!(pmc_read(PWRGATE_STATUS) & (1 << id));

	if (status == new_state) {
		spin_unlock_irqrestore(&tegra_powergate_lock, flags);
		return 0;
	}

	if (TEGRA_IS_CPU_POWERGATE_ID(id)) {
		/* CPU ungated in s/w only during boot/resume with outer
		   waiting loop and no contention from other CPUs */
		pmc_write(PWRGATE_TOGGLE_START | id, PWRGATE_TOGGLE);
		spin_unlock_irqrestore(&tegra_powergate_lock, flags);
		return 0;
	}

	do {
		pmc_write(PWRGATE_TOGGLE_START | id, PWRGATE_TOGGLE);
		do {
			udelay(1);
			status = !!(pmc_read(PWRGATE_STATUS) & (1 << id));

			toggle_timeout--;
		} while ((status != new_state) && (toggle_timeout > 0));

		contention_timeout--;
	} while ((status != new_state) && (contention_timeout > 0));

	spin_unlock_irqrestore(&tegra_powergate_lock, flags);

	if (status != new_state) {
		WARN(1, "Could not set powergate %d to %d", id, new_state);
		return -EBUSY;
	}

	return 0;
}

static int unpowergate_module(int id)
{
	if (id < 0 || id >= TEGRA_NUM_POWERGATE)
		return -EINVAL;
	return tegra_powergate_set(id, true);
}

static int powergate_module(int id)
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

static void get_clk_info(int id)
{
	int idx;

	for (idx = 0; idx < MAX_CLK_EN_NUM; idx++) {
		if (!powergate_partition_info[id].clk_info[idx].clk_name)
			break;
		powergate_partition_info[id].
				clk_info[idx].clk_ptr =
					tegra_get_clock_by_name(
			powergate_partition_info[id].clk_info[idx].clk_name);
	}
}

static int partition_clk_enable(int id)
{
	int ret;
	u32 idx;
	struct clk *clk;
	struct partition_clk_info *clk_info;

	BUG_ON(id < 0 || id >= TEGRA_NUM_POWERGATE);

	for (idx = 0; idx < MAX_CLK_EN_NUM; idx++) {
		clk_info = &powergate_partition_info[id].clk_info[idx];
		clk = clk_info->clk_ptr;
		if (!clk)
			break;

		if (clk_info->clk_type != RST_ONLY) {
			ret = clk_enable(clk);
			if (ret)
				goto err_clk_en;
		}
	}

	return 0;

err_clk_en:
	WARN(1, "Could not enable clk %s", clk->name);
	while (idx--) {
		clk_info = &powergate_partition_info[id].clk_info[idx];
		if (clk_info->clk_type != RST_ONLY)
			clk_disable(clk_info->clk_ptr);
	}

	return ret;
}

static int is_partition_clk_disabled(int id)
{
	u32 idx;
	struct clk *clk;
	struct partition_clk_info *clk_info;
	int ret = 0;

	BUG_ON(id < 0 || id >= TEGRA_NUM_POWERGATE);

	for (idx = 0; idx < MAX_CLK_EN_NUM; idx++) {
		clk_info = &powergate_partition_info[id].clk_info[idx];
		clk = clk_info->clk_ptr;
		if (!clk)
			break;

		if (clk_info->clk_type != RST_ONLY) {
			if (tegra_is_clk_enabled(clk)) {
				ret = -1;
				break;
			}
		}
	}

	return ret;
}

static void partition_clk_disable(int id)
{
	u32 idx;
	struct clk *clk;
	struct partition_clk_info *clk_info;

	BUG_ON(id < 0 || id >= TEGRA_NUM_POWERGATE);

	for (idx = 0; idx < MAX_CLK_EN_NUM; idx++) {
		clk_info = &powergate_partition_info[id].clk_info[idx];
		clk = clk_info->clk_ptr;
		if (!clk)
			break;

		if (clk_info->clk_type != RST_ONLY)
			clk_disable(clk);
	}
}

static void powergate_partition_assert_reset(int id)
{
	u32 idx;
	struct clk *clk_ptr;
	struct partition_clk_info *clk_info;

	BUG_ON(id < 0 || id >= TEGRA_NUM_POWERGATE);

	for (idx = 0; idx < MAX_CLK_EN_NUM; idx++) {
		clk_info = &powergate_partition_info[id].clk_info[idx];
		clk_ptr = clk_info->clk_ptr;
		if (!clk_ptr)
			break;
		if (clk_info->clk_type != CLK_ONLY)
			tegra_periph_reset_assert(clk_ptr);
	}
}

static void powergate_partition_deassert_reset(int id)
{
	u32 idx;
	struct clk *clk_ptr;
	struct partition_clk_info *clk_info;

	BUG_ON(id < 0 || id >= TEGRA_NUM_POWERGATE);

	for (idx = 0; idx < MAX_CLK_EN_NUM; idx++) {
		clk_info = &powergate_partition_info[id].clk_info[idx];
		clk_ptr = clk_info->clk_ptr;
		if (!clk_ptr)
			break;
		if (clk_info->clk_type != CLK_ONLY)
			tegra_periph_reset_deassert(clk_ptr);
	}
}

/* Must be called with clk disabled, and returns with clk disabled */
static int tegra_powergate_reset_module(int id)
{
	int ret;

	powergate_partition_assert_reset(id);

	udelay(10);

	ret = partition_clk_enable(id);
	if (ret)
		return ret;

	udelay(10);

	powergate_partition_deassert_reset(id);

	partition_clk_disable(id);

	return 0;
}

/*
 * Must be called with clk disabled, and returns with clk disabled
 * Drivers should enable clks for partition. Unpowergates only the
 * partition.
 */
int tegra_unpowergate_partition(int id)
{
	int ret;

	/* If first clk_ptr is null, fill clk info for the partition */
	if (!powergate_partition_info[id].clk_info[0].clk_ptr)
		get_clk_info(id);

	if (tegra_powergate_is_powered(id))
		return tegra_powergate_reset_module(id);

	ret = unpowergate_module(id);
	if (ret)
		goto err_power;

	powergate_partition_assert_reset(id);

	/* Un-Powergating fails if all clks are not enabled */
	ret = partition_clk_enable(id);
	if (ret)
		goto err_clk_on;

	udelay(10);

	ret = tegra_powergate_remove_clamping(id);
	if (ret)
		goto err_clamp;

	udelay(10);
	powergate_partition_deassert_reset(id);

	mc_flush_done(id);

	/* Disable all clks enabled earlier. Drivers should enable clks */
	partition_clk_disable(id);

	return 0;

err_clamp:
	partition_clk_disable(id);
err_clk_on:
	powergate_module(id);
err_power:
	WARN(1, "Could not Un-Powergate %d", id);
	return ret;
}

/*
 * Must be called with clk disabled, and returns with clk enabled
 * Unpowergates the partition and enables all required clks.
 */
int tegra_unpowergate_partition_with_clk_on(int id)
{
	int ret = 0;

#ifndef CONFIG_ARCH_TEGRA_2x_SOC
	/* Restrict this functions use to few partitions */
	BUG_ON(id != TEGRA_POWERGATE_SATA && id != TEGRA_POWERGATE_PCIE);
#else
	/* Restrict this functions use to few partitions */
	BUG_ON(id != TEGRA_POWERGATE_PCIE);
#endif

	ret = tegra_unpowergate_partition(id);
	if (ret)
		goto err_unpowergating;

	/* Enable clks for the partition */
	ret = partition_clk_enable(id);
	if (ret)
		goto err_unpowergate_clk;

	return ret;

err_unpowergate_clk:
	tegra_powergate_partition(id);
	WARN(1, "Could not Un-Powergate %d, err in enabling clk", id);
err_unpowergating:
	WARN(1, "Could not Un-Powergate %d", id);
	return ret;
}

/*
 * Must be called with clk disabled. Powergates the partition only
 */
int tegra_powergate_partition(int id)
{
	int ret;

	/* If first clk_ptr is null, fill clk info for the partition */
	if (powergate_partition_info[id].clk_info[0].clk_ptr)
		get_clk_info(id);
	powergate_partition_assert_reset(id);

	/* Powergating is done only if refcnt of all clks is 0 */
	ret = is_partition_clk_disabled(id);
	if (ret)
		goto err_clk_off;

	ret = powergate_module(id);
	if (ret)
		goto err_power_off;

	return 0;

err_power_off:
	WARN(1, "Could not Powergate Partition %d", id);
err_clk_off:
	WARN(1, "Could not Powergate Partition %d, all clks not disabled", id);
	return ret;
}

int tegra_powergate_partition_with_clk_off(int id)
{
	int ret = 0;

#ifndef CONFIG_ARCH_TEGRA_2x_SOC
	/* Restrict functions use to selected partitions */
	BUG_ON(id != TEGRA_POWERGATE_PCIE && id != TEGRA_POWERGATE_SATA);
#else
	/* Restrict functions use to selected partitions */
	BUG_ON(id != TEGRA_POWERGATE_PCIE);
#endif
	/* Disable clks for the partition */
	partition_clk_disable(id);

	ret = is_partition_clk_disabled(id);
	if (ret)
		goto err_powergate_clk;

	ret = tegra_powergate_partition(id);
	if (ret)
		goto err_powergating;

	return ret;

err_powergate_clk:
	WARN(1, "Could not Powergate Partition %d, all clks not disabled", id);
err_powergating:
	partition_clk_enable(id);
	WARN(1, "Could not Powergate Partition %d", id);
	return ret;
}

const char *tegra_powergate_get_name(int id)
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
