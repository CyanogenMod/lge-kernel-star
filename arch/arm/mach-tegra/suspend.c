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
#include <mach/pmc.h>

#include <nvrm_memmgr.h>
#include <nvrm_power_private.h>
#include "nvrm/core/common/nvrm_message.h"

#include "power.h"
#include "board.h"

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
	u32 cclk_divider;
};

volatile struct suspend_context tegra_sctx;
bool core_lock_on = false;

#ifdef CONFIG_HOTPLUG_CPU
extern void tegra_board_nvodm_suspend(void);
extern void tegra_board_nvodm_resume(void);

static void __iomem *pmc = IO_ADDRESS(TEGRA_PMC_BASE);
static void __iomem *clk_rst = IO_ADDRESS(TEGRA_CLK_RESET_BASE);
static void __iomem *flow_ctrl = IO_ADDRESS(TEGRA_FLOW_CTRL_BASE);
static void __iomem *evp_reset = IO_ADDRESS(TEGRA_EXCEPTION_VECTORS_BASE)+0x100;
static void __iomem *tmrus = IO_ADDRESS(TEGRA_TMRUS_BASE);
#endif

#define PMC_CTRL		0x0
#define PMC_CTRL_LATCH_WAKEUPS	(1 << 5)
#define PMC_WAKE_MASK		0xc
#define PMC_WAKE_LEVEL		0x10
#define PMC_DPAD_ORIDE		0x1C
#define PMC_WAKE_DELAY		0xe0
#define PMC_DPD_SAMPLE  	0x20

#define PMC_SW_WAKE_STATUS	0x18
#define PMC_COREPWRGOOD_TIMER	0x3c
#define PMC_SCRATCH0		0x50
#define PMC_SCRATCH1		0x54
#define PMC_CPUPWRGOOD_TIMER	0xc8
#define PMC_CPUPWROFF_TIMER	0xcc
#define PMC_COREPWROFF_TIMER	PMC_WAKE_DELAY
#define PMC_SCRATCH38		0x134
#define PMC_SCRATCH39		0x138
#define PMC_SCRATCH41		0x140

#define CLK_RESET_CCLK_BURST	0x20
#define CLK_RESET_CCLK_DIVIDER  0x24
#define CLK_RESET_PLLC_BASE	0x80
#define CLK_RESET_PLLM_BASE	0x90
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

#ifdef CONFIG_HOTPLUG_CPU
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

static inline void disable_pll(unsigned long base)
{
	unsigned long reg = readl(clk_rst + base);
	reg &= ~(1<<30);
	writel(reg, clk_rst + base);
}

static bool tegra_nvrm_suspend_plls(void)
{
	bool ret_value = false;

#ifdef CONFIG_TEGRA_NVRM
	if (s_hRmGlobal) {
		unsigned long dfs_flags = NvRmPrivGetDfsFlags(s_hRmGlobal);

		if (dfs_flags & NvRmDfsStatusFlags_StopPllC0) {
			disable_pll(CLK_RESET_PLLC_BASE);
			ret_value = true;
		}
		if (dfs_flags & NvRmDfsStatusFlags_StopPllM0) {
			disable_pll(CLK_RESET_PLLM_BASE);
			ret_value = true;
		}
	}
#endif
	return ret_value;
}

static void set_power_timers(unsigned long us_on, unsigned long us_off)
{
	static int last_pclk = 0;
	unsigned long long ticks;
	unsigned long long pclk;

	pclk = clk_get_rate(tegra_pclk);
	if (pclk != last_pclk) {
		ticks = (us_on * pclk) + 999999ull;
		do_div(ticks, 1000000);
		writel((unsigned long)ticks, pmc + PMC_CPUPWRGOOD_TIMER);

		ticks = (us_off * pclk) + 999999ull;
		do_div(ticks, 1000000);
		writel((unsigned long)ticks, pmc + PMC_CPUPWROFF_TIMER);
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
static noinline void restore_cpu_complex(bool wait_plls)
{
	unsigned int reg;

	/* restore original burst policy setting; PLLX state restored
	 * by CPU boot-up code - wait for PLL stabilization if PLLX
	 * was enabled, or if explicitly requested by caller */

	BUG_ON(readl(clk_rst + CLK_RESET_PLLX_BASE) != tegra_sctx.pllx_base);

	if ((tegra_sctx.pllx_base & (1<<30)) || wait_plls) {
		while (readl(tmrus)-tegra_sctx.pllx_timeout >= 0x80000000UL)
			cpu_relax();
	}
	writel(tegra_sctx.cclk_divider, clk_rst + CLK_RESET_CCLK_DIVIDER);
	writel(tegra_sctx.cpu_burst, clk_rst + CLK_RESET_CCLK_BURST);
	writel(tegra_sctx.clk_csite_src, clk_rst + CLK_RESET_SOURCE_CSITE);

	/* do not power-gate the CPU when flow controlled */
	reg = readl(flow_ctrl + FLOW_CTRL_CPU_CSR);
	reg &= ~((1<<5) | (1<<4) | 1); /* clear WFE bitmask */
	reg |= (1<<14); /* write-1-clear event flag */
	writel(reg, flow_ctrl + FLOW_CTRL_CPU_CSR);
	wmb();

	writel(tegra_sctx.twd_ctrl, twd_base + 0x8);
	writel(tegra_sctx.twd_load, twd_base + 0);

	gic_dist_restore(0);
	get_irq_chip(IRQ_LOCALTIMER)->unmask(IRQ_LOCALTIMER);

	if(tegra_nvrm_lp2_persist())
		enable_irq(INT_SYS_STATS_MON);
}

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
	tegra_sctx.cclk_divider = readl(clk_rst + CLK_RESET_CCLK_DIVIDER);

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
	unsigned long orig, reg, lp2time, lp2timelast;
	bool wait_plls = false;

	reg = readl(pmc + PMC_CTRL);
	mode = (reg >> TEGRA_POWER_PMC_SHIFT) & TEGRA_POWER_PMC_MASK;
	mode |= TEGRA_POWER_CPU_PWRREQ_OE;
	if (pdata->separate_req)
		mode |= TEGRA_POWER_PWRREQ_OE;
	else
		mode &= ~TEGRA_POWER_PWRREQ_OE;
	mode &= ~TEGRA_POWER_EFFECT_LP0;

	orig = readl(evp_reset);
	writel(virt_to_phys(tegra_lp2_startup), evp_reset);

	set_power_timers(pdata->cpu_timer, pdata->cpu_off_timer);

	if (us) {
		wait_plls = tegra_nvrm_suspend_plls();
		tegra_lp2_set_trigger(us);
	}
	suspend_cpu_complex();
	flush_cache_all();
	/* structure is written by reset code, so the L2 lines
	 * must be invalidated */
	outer_flush_range(__pa(&tegra_sctx),__pa(&tegra_sctx+1));
	barrier();

	__cortex_a9_save(mode);
	/* return from __cortex_a9_restore */
	restore_cpu_complex(wait_plls);
	if (us)
		tegra_lp2_set_trigger(0);

	writel(orig, evp_reset);

	entry = readl(pmc + PMC_SCRATCH38);
	exit = readl(pmc + PMC_SCRATCH39);
	lp2time = (exit - entry);

#ifdef CONFIG_TEGRA_NVRM
	if (us)	{
		lp2timelast = NvRmPrivGetLp2TimeUS(s_hRmGlobal);
		NvRmPrivSetLp2TimeUS(s_hRmGlobal,(lp2timelast + lp2time));
	}
#endif
	return lp2time;
}
#endif

#ifdef	CONFIG_PM

extern unsigned int s_AvpWarmbootEntry;

/* ensures that sufficient time is passed for a register write to
 * serialize into the 32KHz domain */
static void pmc_32kwritel(u32 val, unsigned long offs)
{
	writel(val, pmc + offs);
	wmb();
	udelay(130);
}

static void tegra_setup_warmboot(bool lp0_ok)
{
	if (lp0_ok) {
		u32 scratch0;

		scratch0 = readl(pmc + PMC_SCRATCH0);
		/* lp0 restore is broken in the ap20 a03 boot rom, so fake the
		 * bootrom into performing a regular boot, but pass a flag to the
		 * bootloader to bypass the kernel reload and jump to the lp0
		 * restore sequence */
		if (tegra_is_ap20_a03() && (!tegra_is_ap20_a03p()))
			scratch0 |= (1<<5);
		else
			scratch0 |= 1;

		pmc_32kwritel(scratch0, PMC_SCRATCH0);

		/* Write the AVP warmboot entry address in SCRATCH1 */
		pmc_32kwritel(s_AvpWarmbootEntry, PMC_SCRATCH1);

		/* Write the CPU LP0 reset vector address in SCRATCH41 */
		writel(virt_to_phys(tegra_lp2_startup), pmc + PMC_SCRATCH41);
	} else {
		/* Setup LP1 start addresses */
		writel(TEGRA_IRAM_CODE_AREA, evp_reset);
		writel(virt_to_phys(tegra_lp2_startup), pmc + PMC_SCRATCH1);
	}
}

static void tegra_setup_wakepads(bool lp0_ok)
{
	u32 temp, status, lvl;

	/* wakeup by interrupt, nothing to do here */
	if (!lp0_ok)
		return;

	pmc_32kwritel(0, PMC_SW_WAKE_STATUS);
	temp = readl(pmc + PMC_CTRL);
	temp |= PMC_CTRL_LATCH_WAKEUPS;
	pmc_32kwritel(temp, PMC_CTRL);
	temp &= ~PMC_CTRL_LATCH_WAKEUPS;
	pmc_32kwritel(temp, PMC_CTRL);
	status = readl(pmc + PMC_SW_WAKE_STATUS);
	lvl = readl(pmc + PMC_WAKE_LEVEL);

	/* flip the wakeup trigger for any-edge triggered pads
	 * which are currently asserting as wakeups */
	status &= pdata->wake_any;
	lvl &= ~pdata->wake_low;
	lvl |= pdata->wake_high;
	lvl ^= status;

	writel(lvl, pmc + PMC_WAKE_LEVEL);
	/* Enable DPD sample to trigger sampling pads data and direction
	 * in which pad will be driven during lp0 mode*/
	writel(0x1, pmc + PMC_DPD_SAMPLE);

	writel(pdata->wake_enb, pmc + PMC_WAKE_MASK);

}

extern void __tegra_lp1_reset(void);
extern void __tegra_iram_end(void);

static u8 *iram_save = NULL;
static unsigned int iram_save_size = 0;
static void __iomem *iram_code = IO_ADDRESS(TEGRA_IRAM_CODE_AREA);
static void __iomem *iram_avp_resume = IO_ADDRESS(TEGRA_IRAM_BASE);

static void tegra_suspend_dram(bool lp0_ok)
{
	static unsigned long cpu_timer_32k = 0;
	static unsigned long cpu_off_timer_32k = 0;

	unsigned int on_timer, off_timer;
	unsigned int mode = TEGRA_POWER_SDRAM_SELFREFRESH;
	unsigned long orig, reg;

	orig = readl(evp_reset);
	/* copy the reset vector and SDRAM shutdown code into IRAM */
	memcpy(iram_save, iram_code, iram_save_size);
	memcpy(iram_code, (void *)__tegra_lp1_reset, iram_save_size);

	if (!cpu_timer_32k) {
		unsigned long long temp = 32768ull*pdata->cpu_timer + 999999;
		do_div(temp, 1000000ul);
		cpu_timer_32k = temp;

		temp = 32768ull*pdata->cpu_off_timer + 999999;
		do_div(temp, 1000000ul);
		cpu_off_timer_32k = temp;
	}
	on_timer = readl(pmc + PMC_CPUPWRGOOD_TIMER);
	writel(cpu_timer_32k, pmc + PMC_CPUPWRGOOD_TIMER);
	off_timer = readl(pmc + PMC_CPUPWROFF_TIMER);
	writel(cpu_off_timer_32k, pmc + PMC_CPUPWROFF_TIMER);

	reg = readl(pmc + PMC_CTRL);
	mode |= ((reg >> TEGRA_POWER_PMC_SHIFT) & TEGRA_POWER_PMC_MASK);

	if (!lp0_ok) {
		NvRmPrivPowerSetState(s_hRmGlobal, NvRmPowerState_LP1);

		mode |= TEGRA_POWER_CPU_PWRREQ_OE;
		if (pdata->separate_req)
			mode |= TEGRA_POWER_PWRREQ_OE;
		else
			mode &= ~TEGRA_POWER_PWRREQ_OE;
		mode &= ~TEGRA_POWER_EFFECT_LP0;
	} else {
		NvRmPrivPowerSetState(s_hRmGlobal, NvRmPowerState_LP0);

		mode |= TEGRA_POWER_CPU_PWRREQ_OE;
		mode |= TEGRA_POWER_PWRREQ_OE;
		mode |= TEGRA_POWER_EFFECT_LP0;

		/* for platforms where the core & CPU power requests are
		 * combined as a single request to the PMU, transition to
		 * LP0 state by temporarily enabling both requests
		 */
		if (!pdata->separate_req) {
			reg |= ((mode & TEGRA_POWER_PMC_MASK) <<
				TEGRA_POWER_PMC_SHIFT);
			pmc_32kwritel(reg, PMC_CTRL);
			mode &= ~TEGRA_POWER_CPU_PWRREQ_OE;
		}
	}

	tegra_setup_warmboot(lp0_ok);
	tegra_setup_wakepads(lp0_ok);
	suspend_cpu_complex();
	flush_cache_all();
	outer_shutdown();

	__cortex_a9_save(mode);
	restore_cpu_complex(false);

	writel(orig, evp_reset);
	outer_restart();
	writel(on_timer, pmc + PMC_CPUPWRGOOD_TIMER);
	writel(off_timer, pmc + PMC_CPUPWROFF_TIMER);

	if (!lp0_ok) {
		memcpy(iram_code, iram_save, iram_save_size);
	} else {
		/* for platforms where the core & CPU power requests are
		 * combined as a single request to the PMU, transition out
		 * of LP0 state by temporarily enabling both requests
		 */
		if (!pdata->separate_req) {
			reg = readl(pmc + PMC_CTRL);
			reg |= (TEGRA_POWER_CPU_PWRREQ_OE << TEGRA_POWER_PMC_SHIFT);
			pmc_32kwritel(reg, PMC_CTRL);
			reg &= ~(TEGRA_POWER_PWRREQ_OE << TEGRA_POWER_PMC_SHIFT);
			writel(reg, pmc + PMC_CTRL);
		}
	}

	wmb();
}

static int tegra_suspend_prepare(void)
{
#ifdef CONFIG_TEGRA_NVRM
	NvOdmSocPowerState state = NvRmPowerLowestStateGet();

	NvRmPrivDfsSuspend(state);
	NvRmPrivPmuLPxStateConfig(s_hRmGlobal, state, NV_TRUE);
#endif
	return 0;
}

static void tegra_suspend_finish(void)
{
#ifdef CONFIG_TEGRA_NVRM
	NvOdmSocPowerState state = NvRmPowerLowestStateGet();

	NvRmPrivPmuLPxStateConfig(s_hRmGlobal, state, NV_FALSE);
	NvRmPrivDfsResume();
#endif
}

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

	tegra_board_nvodm_suspend();

	e = NvRmKernelPowerSuspend(s_hRmGlobal);
	if (e != NvSuccess) {
		pr_err("%s: aborting suspend due to RM failure 0x%08x\n",
		       __func__, e);
		return -EIO;
	}

	/* The AVP stores its resume address in the first word of IRAM
	 * Write this resume address to SCRATCH39, where the warmboot
	 * code can later find it */
	writel(*(volatile unsigned int *)iram_avp_resume, pmc + PMC_SCRATCH39);
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
	tegra_board_nvodm_resume();
#endif
}

extern void __init lp0_suspend_init(void);

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
	bool lp0_ok = (pdata->core_off && (!core_lock_on));

	local_irq_save(flags);

	if (lp0_ok) {
		tegra_irq_suspend();
		tegra_dma_suspend();
		tegra_pinmux_suspend();
		tegra_gpio_suspend();
		tegra_clk_suspend();

		mc_data[0] = readl(mc + MC_SECURITY_START);
		mc_data[1] = readl(mc + MC_SECURITY_SIZE);
	}

	for_each_irq_desc(irq, desc) {
		if ((desc->status & IRQ_WAKEUP) &&
		    (desc->status & IRQ_SUSPENDED)) {
			get_irq_chip(irq)->unmask(irq);
		}
	}

	if (!pdata->dram_suspend || !iram_save) {
		/* lie about the power state so that the RM restarts DVFS */
		NvRmPrivPowerSetState(s_hRmGlobal, NvRmPowerState_LP1);
		tegra_suspend_lp2(0);
	} else
		tegra_suspend_dram(lp0_ok);

	for_each_irq_desc(irq, desc) {
		if ((desc->status & IRQ_WAKEUP) &&
		    (desc->status & IRQ_SUSPENDED)) {
			get_irq_chip(irq)->mask(irq);
		}
	}

	/* Clear DPD sample */
	writel(0x0, pmc + PMC_DPD_SAMPLE);

	if (lp0_ok) {
		writel(mc_data[0], mc + MC_SECURITY_START);
		writel(mc_data[1], mc + MC_SECURITY_SIZE);

		tegra_clk_resume();
		tegra_gpio_resume();
		tegra_pinmux_resume();
		tegra_dma_resume();
		tegra_irq_resume();
	}

	local_irq_restore(flags);

	return 0;
}

static struct platform_suspend_ops tegra_suspend_ops = {
	.valid		= suspend_valid_only_mem,
	.prepare	= tegra_suspend_prepare,
	.finish 	= tegra_suspend_finish,
	.prepare_late	= tegra_suspend_prepare_late,
	.wake		= tegra_suspend_wake,
	.enter		= tegra_suspend_enter,
};
#endif

void __init tegra_init_suspend(struct tegra_suspend_platform_data *plat)
{
	u32 reg, mode;

	tegra_pclk = clk_get_sys(NULL, "pclk");
	BUG_ON(!tegra_pclk);
	pdata = plat;
	(void)reg;
	(void)mode;

#ifdef CONFIG_PM
	iram_save_size = (unsigned long)__tegra_iram_end;
	iram_save_size -= (unsigned long)__tegra_lp1_reset;

	iram_save = kmalloc(iram_save_size, GFP_KERNEL);
	if (!iram_save) {
		pr_err("%s: unable to allocate memory for SDRAM self-refresh "
		       "LP0/LP1 unavailable\n", __func__);
	}
	writel(virt_to_phys(tegra_lp2_startup), pmc + PMC_SCRATCH1);

	/* Always enable CPU power request; just normal polarity is supported */
	reg = readl(pmc + PMC_CTRL);
	BUG_ON(reg & (TEGRA_POWER_CPU_PWRREQ_POLARITY << TEGRA_POWER_PMC_SHIFT));
	reg |= (TEGRA_POWER_CPU_PWRREQ_OE << TEGRA_POWER_PMC_SHIFT);
	pmc_32kwritel(reg, PMC_CTRL);

	/* Configure core power request and system clock control if LP0
	   is supported */
	writel(pdata->core_timer, pmc + PMC_COREPWRGOOD_TIMER);
	writel(pdata->core_off_timer, pmc + PMC_COREPWROFF_TIMER);
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
	pmc_32kwritel(reg, PMC_CTRL);

	/* now enable requests */
	reg |= (TEGRA_POWER_SYSCLK_OE << TEGRA_POWER_PMC_SHIFT);
	if (pdata->separate_req)
		reg |= (TEGRA_POWER_PWRREQ_OE << TEGRA_POWER_PMC_SHIFT);
	writel(reg, pmc + PMC_CTRL);

	if (pdata->core_off)
		lp0_suspend_init();

	suspend_set_ops(&tegra_suspend_ops);
#endif
}

#ifdef CONFIG_PM
void tegra_configure_dpd_kbc(unsigned int kbc_rows, unsigned int kbc_cols)
{
        unsigned long dpd_oride;

	/* Only need to configure the enabled rows */
	dpd_oride = readl(pmc + PMC_DPAD_ORIDE);
	dpd_oride &= 0x00300000;
	dpd_oride |= (kbc_rows & 0xFFFF);
	writel(dpd_oride, pmc + PMC_DPAD_ORIDE);
}
#endif
