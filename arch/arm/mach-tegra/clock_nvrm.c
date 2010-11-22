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
static NvU32 busy_pwr_client_2d;
static NvU32 busy_pwr_client_3d;

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

	/* max out emc when 2d or 3d is on */
	if (NVRM_MODULE_ID_MODULE(c->module) == NvRmModuleID_3D) {
		NvRmDfsBusyHint hint =
			{NvRmDfsClockId_Emc, 0xffffffff, NvRmFreqMaximum, true};
		NvRmPowerBusyHintMulti(s_hRmGlobal, busy_pwr_client_3d, &hint, 1,
			NvRmDfsBusyHintSyncMode_Async);
	} else if (NVRM_MODULE_ID_MODULE(c->module) == NvRmModuleID_2D) {
		NvRmDfsBusyHint hint =
			{NvRmDfsClockId_Emc, 0xffffffff, NvRmFreqMaximum, true};
		hint.BoostKHz = NvRmPrivDfsGetMaxKHz(NvRmDfsClockId_Emc) / 2;
		NvRmPowerBusyHintMulti(s_hRmGlobal, busy_pwr_client_2d, &hint, 1,
			NvRmDfsBusyHintSyncMode_Async);
	}

	return 0;
}

static void tegra_periph_clk_disable(struct clk *c)
{
	NvError e;

	if (NVRM_MODULE_ID_MODULE(c->module) == NvRmModuleID_3D) {
		NvRmDfsBusyHint hint = {NvRmDfsClockId_Emc, 0, 0, true};
		NvRmPowerBusyHintMulti(s_hRmGlobal, busy_pwr_client_3d, &hint, 1,
			NvRmDfsBusyHintSyncMode_Async);
	} else if (NVRM_MODULE_ID_MODULE(c->module) == NvRmModuleID_2D) {
		NvRmDfsBusyHint hint = {NvRmDfsClockId_Emc, 0, 0, true};
		NvRmPowerBusyHintMulti(s_hRmGlobal, busy_pwr_client_2d, &hint, 1,
			NvRmDfsBusyHintSyncMode_Async);
	}

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
		/* If no tolerance, and no low limit - let RM find the
		   best approximation to the target */
		max = min = NvRmFreqUnspecified;
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

static long tegra_periph_clk_round_rate(struct clk *c, unsigned long rate)
{
	NvRmFreqKHz max;
	/* Keep Host on low power PLLP */
	if (c->module == NvRmModuleID_GraphicsHost)
		max = NVRM_PLLP_FIXED_FREQ_KHZ / 2;
	else
		max = NvRmPowerModuleGetMaxFrequency(s_hRmGlobal, c->module);
	return min(((unsigned long)max) * 1000, rate);
}

static struct clk_ops tegra_periph_clk_ops = {
	.init = tegra_periph_clk_init,
	.enable = tegra_periph_clk_enable,
	.disable = tegra_periph_clk_disable,
	.set_rate = tegra_periph_clk_set_rate,
	.get_rate = tegra_periph_clk_get_rate,
	.round_rate = tegra_periph_clk_round_rate,
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

#define PERIPH_CLK(_name, _dev, _con, _modname, _instance, _tol, _min, _pow) \
	{								\
		.name = _name,						\
		.lookup = {						\
			.dev_id = _dev,					\
			.con_id = _con,					\
		},							\
		.module = NVRM_MODULE_ID(NvRmModuleID_##_modname, _instance), \
		.ops = &tegra_periph_clk_ops,				\
		.rate_min = _min,					\
		.rate_tolerance = _tol,					\
		.power = _pow,						\
	}

static struct clk tegra_periph_clk[] = {
	PERIPH_CLK("rtc", "rtc-tegra", NULL, Rtc, 0, 0, 0, false),
	PERIPH_CLK("kbc", "tegra-kbc", NULL, Kbc, 0, 0, 0, false),
	PERIPH_CLK("uarta", "uart.0", NULL, Uart, 0, 5, 0, true),
	PERIPH_CLK("uartb", "uart.1", NULL, Uart, 1, 5, 0, true),
	PERIPH_CLK("uartc", "uart.2", NULL, Uart, 2, 5, 0, true),
	PERIPH_CLK("uartd", "uart.3", NULL, Uart, 3, 5, 0, true),
	PERIPH_CLK("uarte", "uart.4", NULL, Uart, 4, 5, 0, true),
	PERIPH_CLK("sdmmc1", "tegra-sdhci.0", NULL, Sdio, 0, 0, 400, false),
	PERIPH_CLK("sdmmc2", "tegra-sdhci.1", NULL, Sdio, 1, 0, 400, false),
	PERIPH_CLK("sdmmc3", "tegra-sdhci.2", NULL, Sdio, 2, 0, 400, false),
	PERIPH_CLK("sdmmc4", "tegra-sdhci.3", NULL, Sdio, 3, 0, 400, false),
	PERIPH_CLK("pcie", "tegra_pcie", NULL, Pcie, 0, 0, 0, true),
	PERIPH_CLK("pcie_xclk", "tegra_pcie_xclk", NULL, PcieXclk, 0, 0, 0, false),
	PERIPH_CLK("gr3d", "tegra_grhost", "gr3d", 3D, 0, 0, 0, true),
	PERIPH_CLK("gr2d", "tegra_grhost", "gr2d", 2D, 0, 0, 0, true),
	PERIPH_CLK("host1x", "tegra_grhost", "host1x", GraphicsHost, 0, 0, 0, true),
	PERIPH_CLK("epp", "tegra_grhost", "epp", Epp, 0, 0, 0, true),
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

long clk_round_rate(struct clk *c, unsigned long rate)
{
	if (c->ops && c->ops->round_rate)
		return c->ops->round_rate(c, rate);

	return -ENOSYS;
}
EXPORT_SYMBOL(clk_round_rate);


void __init tegra_init_clock(void)
{
	NvError e = NvSuccess;
	struct clk *cpu_clk = NULL;
	unsigned long rate = 0;

	if (!s_hRmGlobal)
		e = NvRmOpenNew(&s_hRmGlobal);
	BUG_ON(e!=NvSuccess);

	NvRmPrivPostRegulatorInit(s_hRmGlobal);
	NvRmPowerRegister(s_hRmGlobal, 0, &clk_pwr_client);
	NvRmPowerRegister(s_hRmGlobal, 0, &busy_pwr_client_2d);
	NvRmPowerRegister(s_hRmGlobal, 0, &busy_pwr_client_3d);
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

#ifdef CONFIG_PM
#define CLK_RESET_RST_DEVICES_L		0x04
#define CLK_RESET_RST_DEVICES_NUM	3
#define CLK_RESET_PROPAGATION_US	10

#define CLK_RESET_CLK_OUT_ENB_L		0x10
#define CLK_RESET_CLK_OUT_ENB_H		0x14
#define CLK_RESET_CLK_OUT_ENB_U		0x18
#define CLK_RESET_CLK_OUT_ENB_L_ALL	0xbffffff9ul
#define CLK_RESET_CLK_OUT_ENB_H_ALL	0xfefffff7ul
#define CLK_RESET_CLK_OUT_ENB_U_ALL	0x77f01bfful
#define CLK_RESET_CLK_OUT_ENB_NUM	3

#define CLK_RESET_CLK_MASK_ARM		0x44
#define CLK_RESET_MISC_CLK_ENB		0x48
#define CLK_RESET_OSC_CTRL		0x50
#define CLK_RESET_OSC_CTRL_MASK		0x3f2	/* drive strength & bypass */

#define CLK_RESET_PLLC_BASE		0x80
#define CLK_RESET_PLLC_MISC		0x8C
#define CLK_RESET_PLLA_BASE		0xB0
#define CLK_RESET_PLLA_MISC		0xBC
#define CLK_RESET_PLLD_BASE		0xD0
#define CLK_RESET_PLLD_MISC		0xDC
#define CLK_RESET_NON_BOOT_PLLS_NUM	3
#define CLK_RESET_PLL_ENABLE_MASK	(0x1 << 30)
#define CLK_RESET_PLL_STAB_US		300
#define CLK_RESET_PLL_STAB_LONG_US	1000

#define CLK_RESET_CLK_SOURCE_I2S1	0x100
#define CLK_RESET_CLK_SOURCE_EMC	0x19c
#define CLK_RESET_CLK_SOURCE_OSC	0x1fc
#define CLK_RESET_CLK_SOURCE_NUM \
	(((CLK_RESET_CLK_SOURCE_OSC - CLK_RESET_CLK_SOURCE_I2S1) / 4) + 1 - 1)

static u32 clk_rst[CLK_RESET_RST_DEVICES_NUM + CLK_RESET_CLK_OUT_ENB_NUM +
		   (CLK_RESET_NON_BOOT_PLLS_NUM * 2) +
		   CLK_RESET_CLK_SOURCE_NUM + 3];

void tegra_clk_suspend(void)
{
	void __iomem *car = IO_ADDRESS(TEGRA_CLK_RESET_BASE);
	unsigned long offs, i;
	u32 *ctx = clk_rst;

	*ctx++ = readl(car + CLK_RESET_OSC_CTRL) & CLK_RESET_OSC_CTRL_MASK;

	*ctx++ = readl(car + CLK_RESET_PLLC_MISC);
	*ctx++ = readl(car + CLK_RESET_PLLC_BASE);
	*ctx++ = readl(car + CLK_RESET_PLLA_MISC);
	*ctx++ = readl(car + CLK_RESET_PLLA_BASE);
	*ctx++ = readl(car + CLK_RESET_PLLD_MISC);
	*ctx++ = readl(car + CLK_RESET_PLLD_BASE);

	for (offs=CLK_RESET_CLK_SOURCE_I2S1;
	     offs<=CLK_RESET_CLK_SOURCE_OSC; offs+=4) {

		if (offs==CLK_RESET_CLK_SOURCE_EMC)
			continue;

		*ctx++ = readl(car + offs);
	}

	offs = CLK_RESET_RST_DEVICES_L;
	for (i=0; i<CLK_RESET_RST_DEVICES_NUM; i++)
		*ctx++ = readl(car + offs + i*4);

	offs = CLK_RESET_CLK_OUT_ENB_L;
	for (i=0; i<CLK_RESET_CLK_OUT_ENB_NUM; i++)
		*ctx++ = readl(car + offs + i*4);

	*ctx++ = readl(car + CLK_RESET_MISC_CLK_ENB);
	*ctx++ = readl(car + CLK_RESET_CLK_MASK_ARM);

	BUG_ON(ctx-clk_rst != ARRAY_SIZE(clk_rst));
}

void tegra_clk_resume(void)
{
	void __iomem *car = IO_ADDRESS(TEGRA_CLK_RESET_BASE);
	unsigned long offs, i;
	u32 *ctx = clk_rst;
	u32 temp;

	temp = readl(car + CLK_RESET_OSC_CTRL) & ~CLK_RESET_OSC_CTRL_MASK;
	temp |= *ctx++;
	writel(temp, car + CLK_RESET_OSC_CTRL);
	wmb();

	writel(*ctx++, car + CLK_RESET_PLLC_MISC);
	temp = *ctx & (~CLK_RESET_PLL_ENABLE_MASK);
	writel(temp, car + CLK_RESET_PLLC_BASE);
	wmb();
	writel(*ctx++, car + CLK_RESET_PLLC_BASE);

	writel(*ctx++, car + CLK_RESET_PLLA_MISC);
	temp = *ctx & (~CLK_RESET_PLL_ENABLE_MASK);
	writel(temp, car + CLK_RESET_PLLA_BASE);
	wmb();
	writel(*ctx++, car + CLK_RESET_PLLA_BASE);

	writel(*ctx++, car + CLK_RESET_PLLD_MISC);
	temp = *ctx & (~CLK_RESET_PLL_ENABLE_MASK);
	writel(temp, car + CLK_RESET_PLLD_BASE);
	wmb();
	temp = *ctx++;
	if (temp & CLK_RESET_PLL_ENABLE_MASK) {
		writel(temp, car + CLK_RESET_PLLD_BASE);
		udelay(CLK_RESET_PLL_STAB_LONG_US);
	} else
		udelay(CLK_RESET_PLL_STAB_US);

	writel(CLK_RESET_CLK_OUT_ENB_L_ALL, car + CLK_RESET_CLK_OUT_ENB_L);
	writel(CLK_RESET_CLK_OUT_ENB_H_ALL, car + CLK_RESET_CLK_OUT_ENB_H);
	writel(CLK_RESET_CLK_OUT_ENB_U_ALL, car + CLK_RESET_CLK_OUT_ENB_U);
	wmb();

	for (offs=CLK_RESET_CLK_SOURCE_I2S1;
	     offs<=CLK_RESET_CLK_SOURCE_OSC; offs+=4) {
		if (offs==CLK_RESET_CLK_SOURCE_EMC)
			continue;
		writel(*ctx++, car + offs);
	}
	wmb();
	udelay(CLK_RESET_PROPAGATION_US);

	offs = CLK_RESET_RST_DEVICES_L;
	for (i=0; i<CLK_RESET_RST_DEVICES_NUM; i++)
		writel(*ctx++, car + offs + i*4);
	wmb();

	offs = CLK_RESET_CLK_OUT_ENB_L;
	for (i=0; i<CLK_RESET_CLK_OUT_ENB_NUM; i++)
		writel(*ctx++, car + offs + i*4);
	wmb();

	writel(*ctx++, car + CLK_RESET_MISC_CLK_ENB);
	writel(*ctx++, car + CLK_RESET_CLK_MASK_ARM);
	BUG_ON(ctx-clk_rst != ARRAY_SIZE(clk_rst));
}

#endif
