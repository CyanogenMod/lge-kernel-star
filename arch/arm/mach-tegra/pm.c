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
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/suspend.h>
#include <linux/slab.h>
#include <linux/serial_reg.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include <linux/syscore_ops.h>

#include <asm/cacheflush.h>
#include <asm/cpu_pm.h>
#include <asm/hardware/cache-l2x0.h>
#include <asm/hardware/gic.h>
#include <asm/localtimer.h>
#include <asm/pgalloc.h>
#include <asm/tlbflush.h>

#include <mach/clk.h>
#include <mach/iomap.h>
#include <mach/irqs.h>

#include "board.h"
#include "clock.h"
#include "pm.h"
#include "pm-irq.h"
#include "sleep.h"

struct suspend_context {
	/*
	 * The next 7 values are referenced by offset in __restart_plls
	 * in headsmp-t2.S, and should not be moved
	 */
	u32 pllx_misc;
	u32 pllx_base;
	u32 pllp_misc;
	u32 pllp_base;
	u32 pllp_outa;
	u32 pllp_outb;
	u32 pll_timeout;

	u32 cpu_burst;
	u32 clk_csite_src;
	u32 cclk_divider;

	u32 mc[3];
	u8 uart[5];
};

static u8 *iram_save;
static unsigned long iram_save_size;

struct suspend_context tegra_sctx;

static void __iomem *iram_code = IO_ADDRESS(TEGRA_IRAM_CODE_AREA);
static void __iomem *pmc = IO_ADDRESS(TEGRA_PMC_BASE);
#ifdef CONFIG_PM
static void __iomem *clk_rst = IO_ADDRESS(TEGRA_CLK_RESET_BASE);
static void __iomem *flow_ctrl = IO_ADDRESS(TEGRA_FLOW_CTRL_BASE);
static void __iomem *evp_reset =
	IO_ADDRESS(TEGRA_EXCEPTION_VECTORS_BASE) + 0x100;
#endif

#define TEGRA_POWER_PWRREQ_POLARITY	(1 << 8)   /* core power request polarity */
#define TEGRA_POWER_PWRREQ_OE		(1 << 9)   /* core power request enable */
#define TEGRA_POWER_SYSCLK_POLARITY	(1 << 10)  /* sys clk polarity */
#define TEGRA_POWER_SYSCLK_OE		(1 << 11)  /* system clock enable */
#define TEGRA_POWER_PWRGATE_DIS		(1 << 12)  /* power gate disabled */
#define TEGRA_POWER_EFFECT_LP0		(1 << 14)  /* enter LP0 when CPU pwr gated */
#define TEGRA_POWER_CPU_PWRREQ_POLARITY (1 << 15)  /* CPU power request polarity */
#define TEGRA_POWER_CPU_PWRREQ_OE	(1 << 16)  /* CPU power request enable */

#define PMC_CTRL		0x0
#define PMC_CTRL_LATCH_WAKEUPS	(1 << 5)
#define PMC_WAKE_MASK		0xc
#define PMC_WAKE_LEVEL		0x10
#define PMC_DPAD_ORIDE		0x1C
#define PMC_WAKE_DELAY		0xe0
#define PMC_DPD_SAMPLE		0x20

#define PMC_WAKE_STATUS		0x14
#define PMC_SW_WAKE_STATUS	0x18
#define PMC_COREPWRGOOD_TIMER	0x3c
#define PMC_SCRATCH0		0x50
#define PMC_SCRATCH1		0x54
#define PMC_CPUPWRGOOD_TIMER	0xc8
#define PMC_CPUPWROFF_TIMER	0xcc
#define PMC_COREPWROFF_TIMER	PMC_WAKE_DELAY

#define CLK_RESET_CCLK_BURST	0x20
#define CLK_RESET_CCLK_DIVIDER  0x24
#define CLK_RESET_PLLC_BASE	0x80
#define CLK_RESET_PLLM_BASE	0x90
#define CLK_RESET_PLLX_BASE	0xe0
#define CLK_RESET_PLLX_MISC	0xe4
#define CLK_RESET_PLLP_BASE	0xa0
#define CLK_RESET_PLLP_OUTA	0xa4
#define CLK_RESET_PLLP_OUTB	0xa8
#define CLK_RESET_PLLP_MISC	0xac

#define CLK_RESET_SOURCE_CSITE	0x1d4

#define CLK_RESET_CCLK_BURST_POLICY_SHIFT 28
#define CLK_RESET_CCLK_BURST_POLICY_PLLM   3
#define CLK_RESET_CCLK_BURST_POLICY_PLLX   8

#define FLOW_CTRL_CPU_CSR(cpu)	((cpu) == 0 ? 0x8 : (0x18 + 8 * ((cpu) - 1)))
#define FLOW_CTRL_HALT_CPU(cpu) ((cpu) == 0 ? 0x0 : (0x4 + cpu * 0x10))

#ifdef CONFIG_ARCH_TEGRA_2x_SOC
#define FLOW_CTRL_CSR_WFE_CPU0		(1 << 4)
#define FLOW_CTRL_CSR_WFE_BITMAP	(3 << 4)
#define FLOW_CTRL_CSR_WFI_BITMAP	0
#else
#define FLOW_CTRL_CSR_WFE_BITMAP	(0xF << 4)
#define FLOW_CTRL_CSR_WFI_CPU0		(1 << 8)
#define FLOW_CTRL_CSR_WFI_BITMAP	(0xF << 8)
#endif

#define FLOW_CTRL_CSR_CLEAR_INTR	(1 << 15)
#define FLOW_CTRL_CSR_CLEAR_EVENT	(1 << 14)
#define FLOW_CTRL_CSR_ENABLE		(1 << 0)

#define EMC_MRW_0		0x0e8
#define EMC_MRW_DEV_SELECTN     30
#define EMC_MRW_DEV_NONE	(3 << EMC_MRW_DEV_SELECTN)

#define MC_SECURITY_START	0x6c
#define MC_SECURITY_SIZE	0x70
#define MC_SECURITY_CFG2	0x7c

unsigned long tegra_pgd_phys;  /* pgd used by hotplug & LP2 bootup */
static pgd_t *tegra_pgd;

static int tegra_last_pclk;
static struct clk *tegra_pclk;
static const struct tegra_suspend_platform_data *pdata;
static enum tegra_suspend_mode current_suspend_mode = TEGRA_SUSPEND_NONE;

static DEFINE_SPINLOCK(tegra_lp2_lock);
static cpumask_t tegra_in_lp2;

unsigned long tegra_cpu_power_good_time(void)
{
	if (WARN_ON_ONCE(!pdata))
		return 5000;

	return pdata->cpu_timer;
}

unsigned long tegra_cpu_power_off_time(void)
{
	if (WARN_ON_ONCE(!pdata))
		return 5000;

	return pdata->cpu_off_timer;
}

/* ensures that sufficient time is passed for a register write to
 * serialize into the 32KHz domain */
static void pmc_32kwritel(u32 val, unsigned long offs)
{
	writel(val, pmc + offs);
	udelay(130);
}

static void set_power_timers(unsigned long us_on, unsigned long us_off,
			     long rate)
{
	unsigned long long ticks;
	unsigned long long pclk;

	if (WARN_ON_ONCE(rate <= 0))
		pclk = 100000000;
	else
		pclk = rate;

	if (rate != tegra_last_pclk) {
		ticks = (us_on * pclk) + 999999ull;
		do_div(ticks, 1000000);
		writel((unsigned long)ticks, pmc + PMC_CPUPWRGOOD_TIMER);

		ticks = (us_off * pclk) + 999999ull;
		do_div(ticks, 1000000);
		writel((unsigned long)ticks, pmc + PMC_CPUPWROFF_TIMER);
		wmb();
	}
	tegra_last_pclk = pclk;
}

/*
 * create_suspend_pgtable
 *
 * Creates a page table with identity mappings of physical memory and IRAM
 * for use when the MMU is off, in addition to all the regular kernel mappings.
 */
static int create_suspend_pgtable(void)
{
	tegra_pgd = pgd_alloc(&init_mm);
	if (!tegra_pgd)
		return -ENOMEM;

	identity_mapping_add(tegra_pgd, PLAT_PHYS_OFFSET, IO_IRAM_PHYS);
	identity_mapping_add(tegra_pgd, IO_IRAM_PHYS,
		IO_IRAM_PHYS + SECTION_SIZE);

	tegra_pgd_phys = virt_to_phys(tegra_pgd);

	return 0;
}

static int tegra_reset_sleeping_cpu(int cpu)
{
	int ret = 0;

	BUG_ON(cpu == smp_processor_id());
	tegra_pen_lock();

	if (readl(pmc + PMC_SCRATCH41) == CPU_RESETTABLE)
		tegra_cpu_reset(cpu);
	else
		ret = -EINVAL;

	tegra_pen_unlock();

	return ret;
}

static void tegra_wake_reset_cpu(int cpu)
{
	u32 reg;

	writel(virt_to_phys(tegra_secondary_resume), evp_reset);

	/* enable cpu clock on cpu */
	reg = readl(clk_rst + 0x4c);
	writel(reg & ~(1 << (8 + cpu)), clk_rst + 0x4c);

	reg = 0x1111 << cpu;
	writel(reg, clk_rst + 0x344);

	/* unhalt the cpu */
	writel(0, flow_ctrl + 0x14);
}

#ifdef CONFIG_PM
/*
 * restore_cpu_complex
 *
 * restores cpu clock setting, clears flow controller
 *
 * always called on cpu 0, even when suspend_cpu_complex was called on cpu 1
 * in idle
 */
static void restore_cpu_complex(void)
{
	unsigned int reg;
	int i;

	/* restore original burst policy setting */

	writel(tegra_sctx.pllx_misc, clk_rst + CLK_RESET_PLLX_MISC);
	writel(tegra_sctx.pllx_base, clk_rst + CLK_RESET_PLLX_BASE);
	writel(tegra_sctx.pllp_misc, clk_rst + CLK_RESET_PLLP_MISC);
	writel(tegra_sctx.pllp_base, clk_rst + CLK_RESET_PLLP_BASE);
	writel(tegra_sctx.pllp_outa, clk_rst + CLK_RESET_PLLP_OUTA);
	writel(tegra_sctx.pllp_outb, clk_rst + CLK_RESET_PLLP_OUTB);

	/* Is CPU complex already running on PLLX? */
	reg = readl(clk_rst + CLK_RESET_CCLK_BURST);
	reg &= 0xF;
	if (reg != 0x8) {
		/* restore original burst policy setting; PLLX state restored
		 * by CPU boot-up code - wait for PLL stabilization if PLLX
		 * was enabled */

		BUG_ON(readl(clk_rst + CLK_RESET_PLLX_BASE) !=
		       tegra_sctx.pllx_base);

		if (tegra_sctx.pllx_base & (1<<30)) {
#if USE_PLL_LOCK_BITS
			/* Enable lock detector */
			reg = readl(clk_rst + CLK_RESET_PLLX_MISC);
			reg |= 1<<18;
			writel(reg, clk_rst + CLK_RESET_PLLX_MISC);
			while (!(readl(clk_rst + CLK_RESET_PLLX_BASE) &&
				 (1<<27)))
				cpu_relax();
#else
			udelay(300);
#endif
		}
		writel(tegra_sctx.cclk_divider, clk_rst +
		       CLK_RESET_CCLK_DIVIDER);
		writel(tegra_sctx.cpu_burst, clk_rst +
		       CLK_RESET_CCLK_BURST);
	}

	writel(tegra_sctx.clk_csite_src, clk_rst + CLK_RESET_SOURCE_CSITE);

	/* do not power-gate the CPU when flow controlled */
	for (i = 0; i < num_possible_cpus(); i++) {
		reg = readl(flow_ctrl + FLOW_CTRL_CPU_CSR(i));
		reg &= ~FLOW_CTRL_CSR_WFE_BITMAP;	/* clear wfe bitmap */
		reg &= ~FLOW_CTRL_CSR_WFI_BITMAP;	/* clear wfi bitmap */
		reg &= ~FLOW_CTRL_CSR_ENABLE;		/* clear enable */
		reg |= FLOW_CTRL_CSR_CLEAR_INTR;	/* clear intr */
		reg |= FLOW_CTRL_CSR_CLEAR_EVENT;	/* clear event */
		writel(reg, flow_ctrl + FLOW_CTRL_CPU_CSR(i));
		wmb();
	}
}

/*
 * suspend_cpu_complex
 *
 * saves pll state for use by restart_plls, prepares flow controller for
 * transition to suspend state
 *
 * in suspend, always called on cpu 0
 * in idle, called on the last cpu to request lp2
 */
static void suspend_cpu_complex(void)
{
	unsigned int reg;
	int i;
	int cpu = smp_processor_id();

	/* switch coresite to clk_m, save off original source */
	tegra_sctx.clk_csite_src = readl(clk_rst + CLK_RESET_SOURCE_CSITE);
	writel(3<<30, clk_rst + CLK_RESET_SOURCE_CSITE);

	tegra_sctx.cpu_burst = readl(clk_rst + CLK_RESET_CCLK_BURST);
	tegra_sctx.pllx_base = readl(clk_rst + CLK_RESET_PLLX_BASE);
	tegra_sctx.pllx_misc = readl(clk_rst + CLK_RESET_PLLX_MISC);
	tegra_sctx.pllp_base = readl(clk_rst + CLK_RESET_PLLP_BASE);
	tegra_sctx.pllp_outa = readl(clk_rst + CLK_RESET_PLLP_OUTA);
	tegra_sctx.pllp_outb = readl(clk_rst + CLK_RESET_PLLP_OUTB);
	tegra_sctx.pllp_misc = readl(clk_rst + CLK_RESET_PLLP_MISC);
	tegra_sctx.cclk_divider = readl(clk_rst + CLK_RESET_CCLK_DIVIDER);

	reg = readl(flow_ctrl + FLOW_CTRL_CPU_CSR(cpu));
	reg &= ~FLOW_CTRL_CSR_WFE_BITMAP;	/* clear wfe bitmap */
	reg &= ~FLOW_CTRL_CSR_WFI_BITMAP;	/* clear wfi bitmap */
	reg |= FLOW_CTRL_CSR_CLEAR_EVENT;	/* clear event flag */
#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	reg |= FLOW_CTRL_CSR_WFE_CPU0 << cpu;	/* enable power gating on wfe */
#else
	reg |= FLOW_CTRL_CSR_WFI_CPU0 << cpu;	/* enable power gating on wfi */
#endif
	reg |= FLOW_CTRL_CSR_ENABLE;		/* enable power gating */
	writel(reg, flow_ctrl + FLOW_CTRL_CPU_CSR(cpu));
	wmb();

	for (i = 0; i < num_possible_cpus(); i++) {
		if (i == cpu)
			continue;
		reg = readl(flow_ctrl + FLOW_CTRL_CPU_CSR(i));
		reg |= FLOW_CTRL_CSR_CLEAR_EVENT;
		reg |= FLOW_CTRL_CSR_CLEAR_INTR;
		writel(reg, flow_ctrl + FLOW_CTRL_CPU_CSR(i));
		writel(0, flow_ctrl + FLOW_CTRL_HALT_CPU(i));
		wmb();
	}
}

int tegra_reset_other_cpus(int cpu)
{
	int i;
	int abort = -1;

	for_each_online_cpu(i) {
		if (i != cpu) {
			if (tegra_reset_sleeping_cpu(i)) {
				abort = i;
				break;
			}
		}
	}

	if (abort >= 0) {
		for_each_online_cpu(i) {
			if (i != cpu && i < abort)
				tegra_wake_reset_cpu(i);
		}
		return -EINVAL;
	}

	return 0;
}

void tegra_idle_lp2_last(unsigned int flags)
{
	u32 reg;
	int i;
	int cpu = smp_processor_id();

	while (tegra_cpu_is_resettable_soon())
		cpu_relax();

	if (tegra_reset_other_cpus(cpu))
		return;

	/* Only the last cpu down does the final suspend steps */
	reg = readl(pmc + PMC_CTRL);
	reg |= TEGRA_POWER_CPU_PWRREQ_OE;
	reg |= TEGRA_POWER_PWRREQ_OE;
	reg |= flags;
	reg &= ~TEGRA_POWER_EFFECT_LP0;
	pmc_32kwritel(reg, PMC_CTRL);

	writel(virt_to_phys(tegra_resume), evp_reset);

	/*
	 * we can use the locked call here, because all other cpus are in reset
	 * and irqs are disabled
	 */
	set_power_timers(pdata->cpu_timer, pdata->cpu_off_timer,
		clk_get_rate_all_locked(tegra_pclk));

	if (flags & TEGRA_POWER_CLUSTER_MASK)
		tegra_cluster_switch_prolog(mode);

	cpu_complex_pm_enter();

	suspend_cpu_complex();
	flush_cache_all();
	outer_flush_all();
	outer_disable();

	tegra_sleep_cpu(PLAT_PHYS_OFFSET - PAGE_OFFSET);

	l2x0_enable();
	restore_cpu_complex();
	cpu_complex_pm_exit();

	if (flags & TEGRA_POWER_CLUSTER_MASK)
		tegra_cluster_switch_epilog(mode);

	spin_lock(&tegra_lp2_lock);

	cpumask_clear_cpu(cpu, &tegra_in_lp2);

	for_each_online_cpu(i) {
		if (i != cpu) {
			tegra_wake_reset_cpu(i);
			cpumask_clear_cpu(i, &tegra_in_lp2);
		}
	}

	spin_unlock(&tegra_lp2_lock);
}

void tegra_idle_lp2(void)
{
	bool last_cpu = false;
	int cpu = smp_processor_id();

	spin_lock(&tegra_lp2_lock);

	cpumask_set_cpu(cpu, &tegra_in_lp2);
	if (cpumask_equal(&tegra_in_lp2, cpu_online_mask))
		last_cpu = true;
	else
		tegra_cpu_set_resettable_soon();

	spin_unlock(&tegra_lp2_lock);

	cpu_pm_enter();

	if (last_cpu)
		tegra_idle_lp2_last(0);
	else
		tegra_sleep_wfi(PLAT_PHYS_OFFSET - PAGE_OFFSET);

	cpu_pm_exit();

	spin_lock(&tegra_lp2_lock);
	cpumask_clear_cpu(cpu, &tegra_in_lp2);
	spin_unlock(&tegra_lp2_lock);
}

static int tegra_common_suspend(void)
{
	void __iomem *mc = IO_ADDRESS(TEGRA_MC_BASE);

	tegra_sctx.mc[0] = readl(mc + MC_SECURITY_START);
	tegra_sctx.mc[1] = readl(mc + MC_SECURITY_SIZE);
	tegra_sctx.mc[2] = readl(mc + MC_SECURITY_CFG2);

	/* copy the reset vector and SDRAM shutdown code into IRAM */
	memcpy(iram_save, iram_code, iram_save_size);
	memcpy(iram_code, &tegra_iram_start, iram_save_size);

	return 0;
}

static void tegra_common_resume(void)
{
	void __iomem *mc = IO_ADDRESS(TEGRA_MC_BASE);
	void __iomem *emc = IO_ADDRESS(TEGRA_EMC_BASE);

	/* Clear DPD sample */
	writel(0x0, pmc + PMC_DPD_SAMPLE);

	writel(tegra_sctx.mc[0], mc + MC_SECURITY_START);
	writel(tegra_sctx.mc[1], mc + MC_SECURITY_SIZE);
	writel(tegra_sctx.mc[2], mc + MC_SECURITY_CFG2);

	/* trigger emc mode write */
	writel(EMC_MRW_DEV_NONE, emc + EMC_MRW_0);

	/* clear scratch registers shared by suspend and the reset pen */
	writel(0x0, pmc + PMC_SCRATCH39);
	writel(0x0, pmc + PMC_SCRATCH41);

	/* restore IRAM */
	memcpy(iram_code, iram_save, iram_save_size);
}

static int tegra_suspend_prepare_late(void)
{
	disable_irq(INT_SYS_STATS_MON);
	return 0;
}

static void tegra_suspend_wake(void)
{
	enable_irq(INT_SYS_STATS_MON);
}

static void tegra_pm_set(enum tegra_suspend_mode mode)
{
	u32 reg;
	unsigned long rate = 32768;

	reg = readl(pmc + PMC_CTRL);
	reg |= TEGRA_POWER_CPU_PWRREQ_OE;
	reg |= TEGRA_POWER_PWRREQ_OE;
	reg &= ~TEGRA_POWER_EFFECT_LP0;

	switch (mode) {
	case TEGRA_SUSPEND_LP0:
		/*
		 * lp0 boots through the AVP, which then resumes the AVP to
		 * the address in scratch 39, and the cpu to the address in
		 * scratch 41 to tegra_resume
		 */
		writel(0x0, pmc + PMC_SCRATCH39);
		writel(virt_to_phys(tegra_resume), pmc + PMC_SCRATCH41);
		reg |= TEGRA_POWER_EFFECT_LP0;

		/* Enable DPD sample to trigger sampling pads data and direction
		 * in which pad will be driven during lp0 mode*/
		writel(0x1, pmc + PMC_DPD_SAMPLE);
		break;
	case TEGRA_SUSPEND_LP1:
		/*
		 * lp1 boots through the normal cpu reset vector pointing to
		 * tegra_lp1_reset in IRAM, which resumes the CPU to
		 * the address in scratch 41 to tegra_resume
		 */
		writel(&tegra_lp1_reset - &tegra_iram_start +
			TEGRA_IRAM_CODE_AREA, evp_reset);
		writel(virt_to_phys(tegra_resume), pmc + PMC_SCRATCH41);
		break;
	case TEGRA_SUSPEND_LP2:
		/*
		 * lp2 boots through the normal cpu reset vector directly to
		 * tegra_resume
		 */
		writel(virt_to_phys(tegra_resume), evp_reset);
		rate = clk_get_rate(tegra_pclk);
		break;
	default:
		BUG();
	}

	set_power_timers(pdata->cpu_timer, pdata->cpu_off_timer, rate);

	pmc_32kwritel(reg, PMC_CTRL);

	/* Set warmboot flag */
	reg = readl(pmc + PMC_SCRATCH0);
	pmc_32kwritel(reg | 1, PMC_SCRATCH0);

	pmc_32kwritel(tegra_lp0_vec_start, PMC_SCRATCH1);
}

static const char *lp_state[TEGRA_MAX_SUSPEND_MODE] = {
	[TEGRA_SUSPEND_NONE] = "none",
	[TEGRA_SUSPEND_LP2] = "LP2",
	[TEGRA_SUSPEND_LP1] = "LP1",
	[TEGRA_SUSPEND_LP0] = "LP0",
};

static int tegra_suspend_enter(suspend_state_t state)
{
	enum tegra_suspend_mode mode = current_suspend_mode;

	BUG_ON(mode < 0 || mode >= TEGRA_MAX_SUSPEND_MODE);

	if ((mode == TEGRA_SUSPEND_LP0) && !tegra_pm_irq_lp0_allowed()) {
		pr_info("LP0 not used due to unsupported wakeup events\n");
		mode = TEGRA_SUSPEND_LP1;
	}

	tegra_common_suspend();

	pr_info("Entering suspend state %s\n", lp_state[mode]);

	tegra_pm_set(mode);

	local_fiq_disable();

	cpu_pm_enter();
	cpu_complex_pm_enter();

	suspend_cpu_complex();
	flush_cache_all();
	outer_flush_all();
	outer_disable();

	if (mode == TEGRA_SUSPEND_LP2)
		tegra_sleep_cpu(PLAT_PHYS_OFFSET - PAGE_OFFSET);
	else
		tegra_sleep_core(PLAT_PHYS_OFFSET - PAGE_OFFSET);

	tegra_init_cache();
	restore_cpu_complex();

	cpu_complex_pm_exit();
	cpu_pm_exit();

	local_fiq_enable();

	tegra_common_resume();

	return 0;
}

/*
 * Function pointers to optional board specific function
 */
void (*tegra_deep_sleep)(int);
EXPORT_SYMBOL(tegra_deep_sleep);

static int tegra_suspend_prepare(void)
{
	if ((current_suspend_mode == TEGRA_SUSPEND_LP0) && tegra_deep_sleep)
		tegra_deep_sleep(1);
	return 0;
}

static void tegra_suspend_finish(void)
{
	if ((current_suspend_mode == TEGRA_SUSPEND_LP0) && tegra_deep_sleep)
		tegra_deep_sleep(0);
}

static const struct platform_suspend_ops tegra_suspend_ops = {
	.valid		= suspend_valid_only_mem,
	.prepare	= tegra_suspend_prepare,
	.finish		= tegra_suspend_finish,
	.prepare_late	= tegra_suspend_prepare_late,
	.wake		= tegra_suspend_wake,
	.enter		= tegra_suspend_enter,
};
#endif

void __init tegra_init_suspend(struct tegra_suspend_platform_data *plat)
{
	u32 reg;
	u32 mode;

	tegra_pclk = clk_get_sys(NULL, "pclk");
	BUG_ON(IS_ERR(tegra_pclk));
	pdata = plat;
	(void)reg;
	(void)mode;

	create_suspend_pgtable();

#ifdef CONFIG_PM
	if (plat->suspend_mode == TEGRA_SUSPEND_LP0 && !tegra_lp0_vec_size) {
		pr_warning("Suspend mode LP0 requested, no lp0_vec\n");
		pr_warning("Disabling LP0\n");
		plat->suspend_mode = TEGRA_SUSPEND_LP1;
	}

	iram_save_size = &tegra_iram_end - &tegra_iram_start;

	iram_save = kmalloc(iram_save_size, GFP_KERNEL);
	if (!iram_save) {
		pr_err("%s: unable to allocate memory for SDRAM self-refresh "
		       "LP0/LP1 unavailable\n", __func__);
		plat->suspend_mode = TEGRA_SUSPEND_LP2;
	}

	/* Initialize scratch registers used for CPU LP2 synchronization */
	writel(0, pmc + PMC_SCRATCH37);
	writel(0, pmc + PMC_SCRATCH38);
	writel(0, pmc + PMC_SCRATCH39);
	writel(0, pmc + PMC_SCRATCH41);

	/* Always enable CPU power request; just normal polarity is supported */
	reg = readl(pmc + PMC_CTRL);
	BUG_ON(reg & TEGRA_POWER_CPU_PWRREQ_POLARITY);
	reg |= TEGRA_POWER_CPU_PWRREQ_OE;
	pmc_32kwritel(reg, PMC_CTRL);

	/* Configure core power request and system clock control if LP0
	   is supported */
	writel(pdata->core_timer, pmc + PMC_COREPWRGOOD_TIMER);
	writel(pdata->core_off_timer, pmc + PMC_COREPWROFF_TIMER);

	reg = readl(pmc + PMC_CTRL);

	if (!pdata->sysclkreq_high)
		reg |= TEGRA_POWER_SYSCLK_POLARITY;
	else
		reg &= ~TEGRA_POWER_SYSCLK_POLARITY;

	if (!pdata->corereq_high)
		reg |= TEGRA_POWER_PWRREQ_POLARITY;
	else
		reg &= ~TEGRA_POWER_PWRREQ_POLARITY;

	/* configure output inverters while the request is tristated */
	pmc_32kwritel(reg, PMC_CTRL);

	/* now enable requests */
	reg |= TEGRA_POWER_SYSCLK_OE;
	reg |= TEGRA_POWER_PWRREQ_OE;
	pmc_32kwritel(reg, PMC_CTRL);

	if (pdata->suspend_mode == TEGRA_SUSPEND_LP0)
		tegra2_lp0_suspend_init();

	suspend_set_ops(&tegra_suspend_ops);
#endif

	current_suspend_mode = plat->suspend_mode;
}

static int tegra_debug_uart_suspend(void)
{
	void __iomem *uart;
	u32 lcr;

	if (TEGRA_DEBUG_UART_BASE == 0)
		return 0;

	uart = IO_ADDRESS(TEGRA_DEBUG_UART_BASE);

	lcr = readb(uart + UART_LCR * 4);

	tegra_sctx.uart[0] = lcr;
	tegra_sctx.uart[1] = readb(uart + UART_MCR * 4);

	/* DLAB = 0 */
	writeb(lcr & ~UART_LCR_DLAB, uart + UART_LCR * 4);

	tegra_sctx.uart[2] = readb(uart + UART_IER * 4);

	/* DLAB = 1 */
	writeb(lcr | UART_LCR_DLAB, uart + UART_LCR * 4);

	tegra_sctx.uart[3] = readb(uart + UART_DLL * 4);
	tegra_sctx.uart[4] = readb(uart + UART_DLM * 4);

	writeb(lcr, uart + UART_LCR * 4);

	return 0;
}

static void tegra_debug_uart_resume(void)
{
	void __iomem *uart;
	u32 lcr;

	if (TEGRA_DEBUG_UART_BASE == 0)
		return;

	uart = IO_ADDRESS(TEGRA_DEBUG_UART_BASE);

	lcr = tegra_sctx.uart[0];

	writeb(tegra_sctx.uart[1], uart + UART_MCR * 4);

	/* DLAB = 0 */
	writeb(lcr & ~UART_LCR_DLAB, uart + UART_LCR * 4);

	writeb(tegra_sctx.uart[2], uart + UART_IER * 4);

	/* DLAB = 1 */
	writeb(lcr | UART_LCR_DLAB, uart + UART_LCR * 4);

	writeb(tegra_sctx.uart[3], uart + UART_DLL * 4);
	writeb(tegra_sctx.uart[4], uart + UART_DLM * 4);

	writeb(lcr, uart + UART_LCR * 4);
}

static struct syscore_ops tegra_debug_uart_syscore_ops = {
	.suspend = tegra_debug_uart_suspend,
	.resume = tegra_debug_uart_resume,
};

static int tegra_debug_uart_syscore_init(void)
{
	register_syscore_ops(&tegra_debug_uart_syscore_ops);
	return 0;
}
arch_initcall(tegra_debug_uart_syscore_init);

#ifdef CONFIG_DEBUG_FS
static const char *tegra_suspend_name[TEGRA_MAX_SUSPEND_MODE] = {
	[TEGRA_SUSPEND_NONE]	= "none",
	[TEGRA_SUSPEND_LP2]	= "lp2",
	[TEGRA_SUSPEND_LP1]	= "lp1",
	[TEGRA_SUSPEND_LP0]	= "lp0",
};

static int tegra_suspend_debug_show(struct seq_file *s, void *data)
{
	seq_printf(s, "%s\n", tegra_suspend_name[*(int *)s->private]);
	return 0;
}

static int tegra_suspend_debug_open(struct inode *inode, struct file *file)
{
	return single_open(file, tegra_suspend_debug_show, inode->i_private);
}

static int tegra_suspend_debug_write(struct file *file,
	const char __user *user_buf, size_t count, loff_t *ppos)
{
	char buf[32];
	int buf_size;
	int i;
	struct seq_file *s = file->private_data;
	enum tegra_suspend_mode *val = s->private;

	memset(buf, 0x00, sizeof(buf));
	buf_size = min(count, (sizeof(buf)-1));
	if (copy_from_user(buf, user_buf, buf_size))
		return -EFAULT;

	for (i = 0; i < TEGRA_MAX_SUSPEND_MODE; i++) {
		if (!strnicmp(buf, tegra_suspend_name[i],
		    strlen(tegra_suspend_name[i]))) {
			if (i > pdata->suspend_mode)
				return -EINVAL;
			*val = i;
			return count;
		}
	}

	return -EINVAL;
}

static const struct file_operations tegra_suspend_debug_fops = {
	.open		= tegra_suspend_debug_open,
	.write		= tegra_suspend_debug_write,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int __init tegra_suspend_debug_init(void)
{
	struct dentry *d;

	d = debugfs_create_file("suspend_mode", 0755, NULL,
		(void *)&current_suspend_mode, &tegra_suspend_debug_fops);
	if (!d) {
		pr_info("Failed to create suspend_mode debug file\n");
		return -ENOMEM;
	}

	return 0;
}

late_initcall(tegra_suspend_debug_init);
#endif
