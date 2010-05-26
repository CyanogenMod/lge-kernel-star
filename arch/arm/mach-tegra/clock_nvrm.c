/*
 * arch/arm/mach-tegra/tegra2_rm_clocks.c
 *
 * Clock controls layered on NvRm implementation
 *
 * Copyright (c) 2009-2010, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
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
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/err.h>
#include <linux/clk.h>

#include <asm/clkdev.h>
#include <asm/smp_twd.h>

#include <mach/iomap.h>
#include <mach/timex.h>

#include <mach/nvrm_linux.h>
#include "nvrm/core/common/nvrm_clocks.h"
#include "clock.h"
#include "nvrm_module.h"
#include "nvrm_power.h"

static LIST_HEAD(clocks);
static DEFINE_SPINLOCK(clock_lock);
static NvU32 clk_pwr_client;

struct clk *get_tegra_clock_by_name(const char *name)
{
	struct clk *c;
	struct clk *ret = NULL;
	unsigned long flags;
	spin_lock_irqsave(&clock_lock, flags);
	list_for_each_entry(c, &clocks, node) {
		if (strcmp(c->name, name) == 0) {
			ret = c;
			break;
		}
	}
	spin_unlock_irqrestore(&clock_lock, flags);
	return ret;
}

void tegra_periph_reset_deassert(struct clk *c)
{
	NvRmModuleResetWithHold(s_hRmGlobal, c->module, NV_FALSE);
}
EXPORT_SYMBOL(tegra_periph_reset_deassert);

void tegra_periph_reset_assert(struct clk *c)
{
	NvRmModuleResetWithHold(s_hRmGlobal, c->module, NV_TRUE);
}
EXPORT_SYMBOL(tegra_periph_reset_assert);

static void tegra_periph_clk_init(struct clk *c)
{
	NvRmModuleReset(s_hRmGlobal, c->module);
}

static int tegra_periph_clk_enable(struct clk *c)
{
	NvError e;

	if (c->power) {
		e = NvRmPowerVoltageControl(s_hRmGlobal, c->module,
			clk_pwr_client, NvRmVoltsUnspecified,
			NvRmVoltsUnspecified, NULL, 0, NULL);
		if (e!=NvSuccess) {
			pr_err("%s: failed to voltage control %s\n",
			       __func__, c->name);
			return -ENXIO;
		}
	}

	e = NvRmPowerModuleClockControl(s_hRmGlobal, c->module,
		clk_pwr_client, NV_TRUE);

	if (e!=NvSuccess) {
		pr_err("%s: failed to clock control %s\n", __func__, c->name);
		return -ENXIO;
	}

	return 0;
}

static void tegra_periph_clk_disable(struct clk *c)
{
	NvError e;

	e = NvRmPowerModuleClockControl(s_hRmGlobal, c->module,
		clk_pwr_client, NV_FALSE);

	if (e!=NvSuccess)
		pr_err("%s: failed to disable %s\n", __func__, c->name);

	if (c->power) {
		e = NvRmPowerVoltageControl(s_hRmGlobal, c->module,
		clk_pwr_client,	NvRmVoltsOff, NvRmVoltsOff, NULL, 0, NULL);

		if (e!=NvSuccess)
			pr_err("%s: failed to disable %s\n", __func__, c->name);
	}
}

static int tegra_periph_clk_set_rate(struct clk *c, unsigned long rate)
{
	NvError e;
	NvRmFreqKHz freq = rate / 1000;
	NvRmFreqKHz min, max;

	if (c->rate_tolerance) {
		NvRmFreqKHz temp = freq * c->rate_tolerance / 100;
		min = freq - temp;
		max = freq + temp;
	} else if (c->rate_min) {
		max = NvRmFreqMaximum;
		min = c->rate_min;
		freq = max_t(NvRmFreqKHz, c->rate_min, freq);
	} else {
		max = min = freq;
	}

	e = NvRmPowerModuleClockConfig(s_hRmGlobal, c->module, clk_pwr_client,
		min, max, &freq, 1, &freq, 0);

	if (e!=NvSuccess) {
		pr_err("%s: failed to configure %s to %luHz\n",
			 __func__, c->name, rate);
		return -EIO;
	}

	pr_debug("%s: requested %luKHz, got %uKHz\n", c->name, rate/1000, freq);

	return 0;
}

static unsigned long tegra_periph_clk_get_rate(struct clk *c)
{
	NvError e;
	NvRmFreqKHz freq;

	e = NvRmPowerModuleClockConfig(s_hRmGlobal, c->module,
		clk_pwr_client, 0, 0, NULL, 0, &freq, 0);

	if (e != NvSuccess) {
		pr_debug("%s: failed to read %s\n", __func__, c->name);
		return 0;
	}

	return (unsigned long)freq * 1000;
}


static struct clk_ops tegra_periph_clk_ops = {
	.init = tegra_periph_clk_init,
	.enable = tegra_periph_clk_enable,
	.disable = tegra_periph_clk_disable,
	.set_rate = tegra_periph_clk_set_rate,
	.get_rate = tegra_periph_clk_get_rate,
};

static unsigned long tegra_clksrc_clk_get_rate(struct clk *c)
{
	NvRmFreqKHz freq;

	freq = NvRmPrivGetClockSourceFreq(c->module);
	return freq * 1000;
}

static struct clk_ops clksrc_clk_ops = {
	.get_rate = tegra_clksrc_clk_get_rate,
};

static unsigned long tegra_dfs_clk_get_rate(struct clk *c)
{
	NvError e;
	NvRmDfsClockUsage usage;

	e = NvRmDfsGetClockUtilization(s_hRmGlobal, c->module, &usage);
	if (e != NvSuccess) {
		pr_debug("%s: failed to read %s\n", __func__, c->name);
		return 0;
	}
	return (unsigned long)usage.CurrentKHz * 1000;
}

static struct clk_ops dfs_clk_ops = {
	.get_rate = tegra_dfs_clk_get_rate,
};

#define NvRmModuleID_Pcie NvRmPrivModuleID_Pcie
#define NvRmModuleID_Afi NvRmPrivModuleID_Afi
#define NvRmModuleID_PcieXclk NvRmPrivModuleID_PcieXclk

#define PERIPH_CLK(_name, _dev, _modname, _instance, _tol, _min, _pow)	\
	{								\
		.name = _name,						\
		.lookup = {						\
			.dev_id = _dev,					\
		},							\
		.module = NVRM_MODULE_ID(NvRmModuleID_##_modname, _instance), \
		.ops = &tegra_periph_clk_ops,				\
		.rate_min = _min,					\
		.rate_tolerance = _tol,					\
		.power = _pow,						\
	}

static struct clk tegra_periph_clk[] = {
	PERIPH_CLK("rtc", "rtc-tegra", Rtc, 0, 0, 0, false),
	PERIPH_CLK("kbc", "tegra-kbc", Kbc, 0, 0, 0, false),
	PERIPH_CLK("uarta", "uart.0", Uart, 0, 5, 0, true),
	PERIPH_CLK("uartb", "uart.1", Uart, 1, 5, 0, true),
	PERIPH_CLK("uartc", "uart.2", Uart, 2, 5, 0, true),
	PERIPH_CLK("uartd", "uart.3", Uart, 3, 5, 0, true),
	PERIPH_CLK("uarte", "uart.4", Uart, 4, 5, 0, true),
	PERIPH_CLK("sdmmc1", "tegra-sdhci.0", Sdio, 0, 0, 400, false),
	PERIPH_CLK("sdmmc2", "tegra-sdhci.1", Sdio, 1, 0, 400, false),
	PERIPH_CLK("sdmmc3", "tegra-sdhci.2", Sdio, 2, 0, 400, false),
	PERIPH_CLK("sdmmc4", "tegra-sdhci.3", Sdio, 3, 0, 400, false),
	PERIPH_CLK("pcie", "tegra_pcie", Pcie, 0, 0, 0, true),
	PERIPH_CLK("pcie_xclk", "tegra_pcie_xclk", PcieXclk, 0, 0, 0, false),
};

static struct clk tegra_clk_cpu = {
	.name = "cpu",
	.module = NvRmDfsClockId_Cpu,
	.ops = &dfs_clk_ops,
};

static struct clk tegra_clk_pclk = {
	.name = "pclk",
	.module = NvRmDfsClockId_Apb,
	.ops = &dfs_clk_ops,
};

static struct clk tegra_clk_hclk = {
	.name = "hclk",
	.module = NvRmDfsClockId_Ahb,
	.ops = &dfs_clk_ops,
};

static struct clk tegra_clk_sys = {
	.name = "sys",
	.module = NvRmDfsClockId_System,
	.ops = &dfs_clk_ops,
};

static struct clk tegra_clk_pllp = {
	.name = "pll_p",
	.module = NvRmClockSource_PllP0,
	.ops = &clksrc_clk_ops,
};

#define DFS_CLK(dev, con, ck)			\
	{					\
		.dev_id = dev,			\
		.con_id = con,			\
		.clk = ck,			\
	}

static struct clk_lookup tegra_clk_lookups[] = {
	DFS_CLK(NULL, "cpu", &tegra_clk_cpu),
	DFS_CLK(NULL, "pclk", &tegra_clk_pclk),
	DFS_CLK(NULL, "hclk", &tegra_clk_hclk),
	DFS_CLK(NULL, "sys", &tegra_clk_sys),
	DFS_CLK(NULL, "pll_p", &tegra_clk_pllp),
};

#define CLK_DUPLICATE(_name, _dev, _con) \
	{						\
		.name      = _name,			\
		.lookup    = {				\
			.dev_id    = _dev,		\
			.con_id	   = _con,		\
		},					\
	}

/* Some clocks may be used by different drivers depending on the board
 * configuration.  List those here to register them twice in the clock lookup
 * table under two names.
 */
struct clk_duplicate tegra_clk_duplicates[] = {
	CLK_DUPLICATE("uarta", "tegra_uart.0", NULL),
	CLK_DUPLICATE("uartb", "tegra_uart.1", NULL),
	CLK_DUPLICATE("uartc", "tegra_uart.2", NULL),
	CLK_DUPLICATE("uartd", "tegra_uart.3", NULL),
	CLK_DUPLICATE("uarte", "tegra_uart.4", NULL),
};

void __init tegra2_init_clocks(void)
{
	int i;
	struct clk_lookup *cl;
	struct clk *c;
	struct clk_duplicate *cd;

	for (i=0; i<ARRAY_SIZE(tegra_clk_lookups); i++) {
		cl = &tegra_clk_lookups[i];
		clk_init(cl->clk);
		clkdev_add(cl);
	}

	for (i=0; i<ARRAY_SIZE(tegra_periph_clk); i++) {
		c = &tegra_periph_clk[i];
		cl = &c->lookup;
		cl->clk = c;
		clk_init(c);
		clkdev_add(cl);
	}

	for (i = 0; i < ARRAY_SIZE(tegra_clk_duplicates); i++) {
		cd = &tegra_clk_duplicates[i];
		c = get_tegra_clock_by_name(cd->name);
		if (c) {
			cl = &cd->lookup;
			cl->clk = c;
			clkdev_add(cl);
		} else {
			pr_err("%s: Unknown duplicate clock %s\n", __func__,
				cd->name);
		}
	}
}

void clk_init(struct clk *c)
{
	unsigned long flags;

	if (c->ops && c->ops->init)
		c->ops->init(c);

	spin_lock_irqsave(&clock_lock, flags);
	list_add(&c->node, &clocks);
	spin_unlock_irqrestore(&clock_lock, flags);
}

int clk_enable(struct clk *c)
{
	if (!c)
		return -ENODEV;

	if (c->ops && c->ops->enable)
		return c->ops->enable(c);

	return -ENOSYS;
}
EXPORT_SYMBOL(clk_enable);

void clk_disable(struct clk *c)
{
	if (c->ops && c->ops->disable)
		c->ops->disable(c);
}
EXPORT_SYMBOL(clk_disable);

int clk_set_rate(struct clk *c, unsigned long rate)
{
	if (c->ops && c->ops->set_rate)
		return c->ops->set_rate(c, rate);

	return -ENOSYS;
}
EXPORT_SYMBOL(clk_set_rate);

unsigned long clk_get_rate(struct clk *c)
{
	BUG_ON(!(c->ops && c->ops->get_rate));

	return c->ops->get_rate(c);
}
EXPORT_SYMBOL(clk_get_rate);

void __init tegra_init_clock(void)
{
	NvError e;
	struct clk *cpu_clk = NULL;
	unsigned long rate = 0;

	e = NvRmOpenNew(&s_hRmGlobal);
	BUG_ON(e!=NvSuccess);
	NvRmPowerRegister(s_hRmGlobal, 0, &clk_pwr_client);
	tegra2_init_clocks();

#ifdef CONFIG_USE_ARM_TWD_PRESCALER
	cpu_clk = clk_get_sys(NULL, "cpu");
	BUG_ON(IS_ERR(cpu_clk));

	rate = clk_get_rate(cpu_clk);
	local_timer_rescale(rate / 1000);
	clk_put(cpu_clk);
	on_each_cpu(twd_set_prescaler, NULL, true);
#endif
}
