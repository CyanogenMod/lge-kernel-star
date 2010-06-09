/*
 * arch/arm/mach-tegra/suspend.c
 *
 * CPU complex suspend & resume functions for Tegra SoCs
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/sched.h>
#include <linux/smp.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/suspend.h>

#include <asm/cacheflush.h>
#include <asm/hardware/gic.h>
#include <asm/localtimer.h>
#include <asm/tlbflush.h>
#include <asm/cacheflush.h>

#include <mach/iomap.h>
#include <mach/iovmm.h>
#include <mach/irqs.h>
#include <mach/nvrm_linux.h>

#include <nvrm_memmgr.h>
#include <nvrm_power_private.h>
#include "nvrm/core/common/nvrm_message.h"

#include "power.h"

/* NOTE: only add elements to the end of this structure, since the assembly
 * code uses hard-coded offsets */
struct suspend_context
{
	u32 cpu_burst;
	u32 clk_csite_src;
	u32 pllx_base;
	u32 pllx_misc;
	u32 pllx_timeout;
	u32 twd_ctrl;
	u32 twd_load;
};

volatile struct suspend_context tegra_sctx;

static void __iomem *pmc = IO_ADDRESS(TEGRA_PMC_BASE);
static void __iomem *clk_rst = IO_ADDRESS(TEGRA_CLK_RESET_BASE);
static void __iomem *flow_ctrl = IO_ADDRESS(TEGRA_FLOW_CTRL_BASE);
static void __iomem *evp_reset = IO_ADDRESS(TEGRA_EXCEPTION_VECTORS_BASE)+0x100;
static void __iomem *tmrus = IO_ADDRESS(TEGRA_TMRUS_BASE);

#define PMC_CTRL		0x0
#define PMC_COREPWRGOOD_TIMER	0x3c
#define PMC_SCRATCH1		0x54
#define PMC_CPUPWRGOOD_TIMER	0xc8
#define PMC_SCRATCH38		0x134
#define PMC_SCRATCH39		0x138

#define CLK_RESET_CCLK_BURST	0x20
#define CLK_RESET_PLLX_BASE	0xe0
#define CLK_RESET_PLLX_MISC	0xe4
#define CLK_RESET_SOURCE_CSITE	0x1d4


#define CLK_RESET_CCLK_BURST_POLICY_SHIFT 28
#define CLK_RESET_CCLK_BURST_POLICY_PLLM   3
#define CLK_RESET_CCLK_BURST_POLICY_PLLX   8

#define FLOW_CTRL_CPU_CSR	0x8
#define FLOW_CTRL_CPU1_CSR	0x18

static struct clk *tegra_pclk = NULL;
static const struct tegra_suspend_platform_data *pdata = NULL;

bool tegra_nvrm_lp2_allowed(void)
{
	bool ret_value = true;

#ifdef CONFIG_TEGRA_NVRM
	if (s_hRmGlobal) {
		if (g_Lp2Policy == NvRmLp2Policy_Disabled)
			ret_value = false;
		else if(g_Lp2Policy == NvRmLp2Policy_EnterInLowCorner)
			ret_value = NvRmPrivGetDfsFlags(s_hRmGlobal) &
					NvRmDfsStatusFlags_Pause;
	}
#endif
	return ret_value;
}

static bool tegra_nvrm_lp2_persist(void)
{
	bool ret_value = true;

#ifdef CONFIG_TEGRA_NVRM
	if (s_hRmGlobal && (g_Lp2Policy == NvRmLp2Policy_MaskInLowCorner))
		ret_value = NvRmPrivGetDfsFlags(s_hRmGlobal) &
				NvRmDfsStatusFlags_Pause;
#endif
	return ret_value;
}

static void set_powergood_time(unsigned int us)
{
	static int last_pclk = 0;
	unsigned long long ticks;
	unsigned long long pclk;

	pclk = clk_get_rate(tegra_pclk);
	if (pclk != last_pclk) {
		ticks = (us * pclk) + 999999ull;
		do_div(ticks, 1000000);
		writel((unsigned int)ticks, pmc + PMC_CPUPWRGOOD_TIMER);
		wmb();
	}
	last_pclk = pclk;
}

/*
 * suspend_cpu_complex
 *
 *   disable periodic IRQs used for DVFS to prevent suspend wakeups
 *   disable coresight debug interface
 *
 *
 */
static noinline void restore_cpu_complex(void)
{
	unsigned int reg;

	/* restore original burst policy setting; PLLX state restored
	 * by CPU boot-up code - wait for PLL stabilization if PLLX
	 * was enabled */

	BUG_ON(readl(clk_rst + CLK_RESET_PLLX_BASE) != tegra_sctx.pllx_base);

	if (tegra_sctx.pllx_base & (1<<30)) {
		while (readl(tmrus)-tegra_sctx.pllx_timeout >= 0x80000000UL)
			cpu_relax();
	}
	writel(tegra_sctx.cpu_burst, clk_rst + CLK_RESET_CCLK_BURST);
	writel(tegra_sctx.clk_csite_src, clk_rst + CLK_RESET_SOURCE_CSITE);

	/* do not power-gate the CPU when flow controlled */
	reg = readl(flow_ctrl + FLOW_CTRL_CPU_CSR);
	reg &= ~((1<<14) | (1<<5) | (1<<4) | 1); /* clear WFE bitmask */
	writel(reg, flow_ctrl + FLOW_CTRL_CPU_CSR);
	wmb();

	writel(tegra_sctx.twd_ctrl, twd_base + 0x8);
	writel(tegra_sctx.twd_load, twd_base + 0);

	gic_dist_restore(0);
	get_irq_chip(IRQ_LOCALTIMER)->unmask(IRQ_LOCALTIMER);

	if(tegra_nvrm_lp2_persist())
		enable_irq(INT_SYS_STATS_MON);
}

extern unsigned long tegra_pgd_phys;

static noinline void suspend_cpu_complex(void)
{
	unsigned int reg;
	int i;

	if(tegra_nvrm_lp2_persist())
		disable_irq(INT_SYS_STATS_MON);

	/* switch coresite to clk_m, save off original source */
	tegra_sctx.clk_csite_src = readl(clk_rst + CLK_RESET_SOURCE_CSITE);
	writel(3<<30, clk_rst + CLK_RESET_SOURCE_CSITE);

	tegra_sctx.cpu_burst = readl(clk_rst + CLK_RESET_CCLK_BURST);
	tegra_sctx.pllx_base = readl(clk_rst + CLK_RESET_PLLX_BASE);
	tegra_sctx.pllx_misc = readl(clk_rst + CLK_RESET_PLLX_MISC);

	tegra_sctx.twd_ctrl = readl(twd_base + 0x8);
	tegra_sctx.twd_load = readl(twd_base + 0);
	local_timer_stop();

	reg = readl(flow_ctrl + FLOW_CTRL_CPU_CSR);
	/* clear any pending events, set the WFE bitmap to specify just
	 * CPU0, and clear any pending events for this CPU */
	reg &= ~(1<<5); /* clear CPU1 WFE */
	reg |= (1<<14) | (1<<4) | 1; /* enable CPU0 WFE */
	writel(reg, flow_ctrl + FLOW_CTRL_CPU_CSR);
	wmb();

	for (i=1; i<num_present_cpus(); i++) {
		unsigned int offs = FLOW_CTRL_CPU1_CSR + (i-1)*8;
		reg = readl(flow_ctrl + offs);
		writel(reg | (1<<14), flow_ctrl + offs);
		wmb();
	}

	gic_cpu_exit(0);
	gic_dist_exit(0);
}

unsigned int tegra_suspend_lp2(unsigned int us)
{
	unsigned int mode, entry, exit;
	unsigned long orig;

	mode = TEGRA_POWER_CPU_PWRREQ_OE;
	orig = readl(evp_reset);
	writel(virt_to_phys(tegra_lp2_startup), evp_reset);

	/* FIXME: power good time (in us) should come from the board file,
	 * not hard-coded here. */
	set_powergood_time(pdata->cpu_timer);

	if (us)
		tegra_lp2_set_trigger(us);
	suspend_cpu_complex();
	flush_cache_all();
	/* structure is written by reset code, so the L2 lines
	 * must be invalidated */
	outer_flush_range(__pa(&tegra_sctx),__pa(&tegra_sctx+1));
	barrier();

	__cortex_a9_save(mode);
	/* return from __cortex_a9_restore */
	restore_cpu_complex();

	writel(orig, evp_reset);

	entry = readl(pmc + PMC_SCRATCH38);
	exit = readl(pmc + PMC_SCRATCH39);
	return exit - entry;
}

#ifdef CONFIG_PM
static int tegra_suspend_prepare_late(void)
{
#ifdef CONFIG_TEGRA_NVRM
	static NvRmTransportHandle port = NULL;
	static NvRmMemHandle iram_area = NULL;
	static unsigned long iram_area_pa = 0;
	static void __iomem *barrier = NULL;

	NvRmMessage_InitiateLP0 msg;
	unsigned long timeout;
	NvError e;

	if (!s_hRmGlobal)
		return 0;

	if (!port) {
		e = NvRmTransportOpen(s_hRmGlobal, "RPC_AVP_PORT", NULL, &port);
		if (e != NvSuccess) {
			pr_err("%s: aborting suspend due to TransportOpen "
			       "returning 0x%08x\n", __func__, e);
			return -ENOSYS;
		}
	}

	if (!iram_area) {
		NvRmHeap h = NvRmHeap_ExternalCarveOut;

		e = NvRmMemHandleCreate(s_hRmGlobal, &iram_area,
				TEGRA_IRAM_SIZE + PAGE_SIZE);
		if (e != NvSuccess) {
			pr_err("%s: MemHandleCreate returned 0x%08x\n",
			       __func__, e);
			return -ENOMEM;
		}
		e = NvRmMemAlloc(iram_area, &h, 1, PAGE_SIZE,
				 NvOsMemAttribute_Uncached);
		if (e != NvSuccess) {
			pr_err("%s: NvRmMemAlloc returned 0x%08x\n",
			       __func__, e);
			NvRmMemHandleFree(iram_area);
			iram_area = NULL;
			return -ENOMEM;
		}

		iram_area_pa = NvRmMemPin(iram_area);

	}

	BUG_ON(iram_area_pa == ~0ul);

	if (!barrier) {
		barrier = ioremap(iram_area_pa + TEGRA_IRAM_SIZE, PAGE_SIZE);
		if (IS_ERR_OR_NULL(barrier)) {
			pr_err("%s: failed to map barrier\n", __func__);
			barrier = NULL;
			return -ENOMEM;
		}
	}

	/* the AVP will write a non-zero value to PMC_SCRATCH38 once it has
	 * suspended itself */
	msg.msg = NvRmMsg_InitiateLP0;
	msg.sourceAddr = TEGRA_IRAM_BASE;
	msg.bufferAddr = iram_area_pa;
	msg.bufferSize = TEGRA_IRAM_SIZE;

	writel(0, barrier);
	wmb();

	e = NvRmTransportSendMsgInLP0(port, &msg, sizeof(msg));
	if (e != NvSuccess) {
		pr_err("%s: aborting suspend due to SendMsgInLP0 returning "
		       "0x%08x\n", __func__, e);
		return -EIO;
	}
	timeout = jiffies + msecs_to_jiffies(1000);

	while (time_before(jiffies, timeout)) {
		if (readl(barrier))
			break;
		udelay(10);
		rmb();
	}

	/* FIXME: reset the AVP, don't abort suspend */
	if (!readl(barrier)) {
		pr_err("%s: aborting suspend due to AVP timeout\n", __func__);
		return -EIO;
	}
	e = NvRmKernelPowerSuspend(s_hRmGlobal);
	if (e != NvSuccess) {
		pr_err("%s: aborting suspend due to RM failure 0x%08x\n",
		       __func__, e);
		return -EIO;
	}
#endif
	disable_irq(INT_SYS_STATS_MON);
	return tegra_iovmm_suspend();
}

static void tegra_suspend_wake(void)
{
	tegra_iovmm_resume();
	enable_irq(INT_SYS_STATS_MON);

#ifdef CONFIG_TEGRA_NVRM
	{
		NvError e;

		e = NvRmKernelPowerResume(s_hRmGlobal);
		if (e != NvSuccess)
			panic("%s: RM resume failed 0x%08x!\n", __func__, e);
	}
#endif
}

extern void tegra_pinmux_suspend(void);
extern void tegra_irq_suspend(void);
extern void tegra_gpio_suspend(void);
extern void tegra_clk_suspend(void);
extern void tegra_dma_suspend(void);

extern void tegra_pinmux_resume(void);
extern void tegra_irq_resume(void);
extern void tegra_gpio_resume(void);
extern void tegra_clk_resume(void);
extern void tegra_dma_resume(void);

#define MC_SECURITY_START	0x6c
#define MC_SECURITY_SIZE	0x70

static int tegra_suspend_enter(suspend_state_t state)
{
	struct irq_desc *desc;
	void __iomem *mc = IO_ADDRESS(TEGRA_MC_BASE);
	unsigned long flags;
	u32 mc_data[2];
	int irq;

	local_irq_save(flags);
	tegra_irq_suspend();
	tegra_dma_suspend();
	tegra_pinmux_suspend();
	tegra_gpio_suspend();
	tegra_clk_suspend();

	mc_data[0] = readl(mc + MC_SECURITY_START);
	mc_data[1] = readl(mc + MC_SECURITY_SIZE);

	for_each_irq_desc(irq, desc) {
		if ((desc->status & IRQ_WAKEUP) &&
		    (desc->status & IRQ_SUSPENDED)) {
			get_irq_chip(irq)->unmask(irq);
		}
	}

	/* lie about the power state so that the RM restarts DVFS */
	NvRmPrivPowerSetState(s_hRmGlobal, NvRmPowerState_LP1);
	tegra_suspend_lp2(0);

	for_each_irq_desc(irq, desc) {
		if ((desc->status & IRQ_WAKEUP) &&
		    (desc->status & IRQ_SUSPENDED)) {
			get_irq_chip(irq)->mask(irq);
		}
	}

	writel(mc_data[0], mc + MC_SECURITY_START);
	writel(mc_data[1], mc + MC_SECURITY_SIZE);

	tegra_clk_resume();
	tegra_gpio_resume();
	tegra_pinmux_resume();
	tegra_dma_resume();
	tegra_irq_resume();

	local_irq_restore(flags);

	return 0;
}

static struct platform_suspend_ops tegra_suspend_ops = {
	.valid		= suspend_valid_only_mem,
	.prepare_late	= tegra_suspend_prepare_late,
	.wake		= tegra_suspend_wake,
	.enter		= tegra_suspend_enter,
};
#endif

void __init tegra_init_suspend(struct tegra_suspend_platform_data *plat)
{
	tegra_pclk = clk_get_sys(NULL, "pclk");
	BUG_ON(!tegra_pclk);
	pdata = plat;

	if (pdata->core_off) {
		u32 reg = 0, mode;

		writel(pdata->core_timer, pmc + PMC_COREPWRGOOD_TIMER);
		reg = readl(pmc + PMC_CTRL);
		mode = (reg >> TEGRA_POWER_PMC_SHIFT) & TEGRA_POWER_PMC_MASK;

		mode &= ~TEGRA_POWER_SYSCLK_POLARITY;
		mode &= ~TEGRA_POWER_PWRREQ_POLARITY;

		if (!pdata->sysclkreq_high)
			mode |= TEGRA_POWER_SYSCLK_POLARITY;
		if (!pdata->corereq_high)
			mode |= TEGRA_POWER_PWRREQ_POLARITY;

		/* configure output inverters while the request is tristated */
		reg |= (mode << TEGRA_POWER_PMC_SHIFT);
		writel(reg, pmc + PMC_CTRL);
		wmb();
		udelay(2000); /* 32KHz domain delay */
		reg |= (TEGRA_POWER_SYSCLK_OE << TEGRA_POWER_PMC_SHIFT);
		writel(reg, pmc + PMC_CTRL);
	}
#ifdef CONFIG_PM
	suspend_set_ops(&tegra_suspend_ops);
#endif
}
