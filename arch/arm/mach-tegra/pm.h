/*
 * arch/arm/mach-tegra/include/mach/suspend.h
 *
 * Copyright (C) 2010 Google, Inc.
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


#ifndef _MACH_TEGRA_SUSPEND_H_
#define _MACH_TEGRA_SUSPEND_H_

enum tegra_suspend_mode {
	TEGRA_SUSPEND_NONE = 0,
	TEGRA_SUSPEND_LP2,	/* CPU voltage off */
	TEGRA_SUSPEND_LP1,	/* CPU voltage off, DRAM self-refresh */
	TEGRA_SUSPEND_LP0,	/* CPU + core voltage off, DRAM self-refresh */
	TEGRA_MAX_SUSPEND_MODE,
};

struct tegra_suspend_platform_data {
	unsigned long cpu_timer;   /* CPU power good time in us,  LP2/LP1 */
	unsigned long cpu_off_timer;	/* CPU power off time us, LP2/LP1 */
	unsigned long core_timer;  /* core power good time in ticks,  LP0 */
	unsigned long core_off_timer;	/* core power off time ticks, LP0 */
	bool corereq_high;         /* Core power request active-high */
	bool sysclkreq_high;       /* System clock request is active-high */
	enum tegra_suspend_mode suspend_mode;
};

unsigned long tegra_cpu_power_good_time(void);
unsigned long tegra_cpu_power_off_time(void);

#define TEGRA_POWER_SDRAM_SELFREFRESH	0x400	/* SDRAM is in self-refresh */

#define TEGRA_POWER_CLUSTER_G		0x1000	/* G CPU */
#define TEGRA_POWER_CLUSTER_LP		0x2000	/* LP CPU */
#define TEGRA_POWER_CLUSTER_MASK	0x3000
#define TEGRA_POWER_CLUSTER_IMMEDIATE	0x4000	/* Immediate wake */
#define TEGRA_POWER_CLUSTER_FORCE	0x8000	/* Force switch */

#ifdef CONFIG_ARCH_TEGRA_2x_SOC
void tegra2_lp0_suspend_init(void);
#else
static inline void tegra2_lp0_suspend_init(void)
{
}
#endif
void __init tegra_init_suspend(struct tegra_suspend_platform_data *plat);

void tegra_idle_lp2(void);

u64 tegra_rtc_read_ms(void);

/*
 * Callbacks for platform drivers to implement.
 */
extern void (*tegra_deep_sleep)(int);

void tegra_idle_lp2_last(unsigned int flags);
#ifdef CONFIG_ARCH_TEGRA_2x_SOC
static inline int tegra_cluster_control(unsigned int us, unsigned int flags)
{ return -EPERM; }
#define tegra_cluster_switch_prolog(flags) do {} while(0)
#define tegra_cluster_switch_epilog(flags) do {} while(0)
static inline unsigned int is_lp_cluster(void)
{ return 0; }
static inline unsigned long tegra_get_lpcpu_max_rate(void)
{ return 0; }
#else
int tegra_cluster_control(unsigned int us, unsigned int flags);
void tegra_cluster_switch_prolog(unsigned int flags);
void tegra_cluster_switch_epilog(unsigned int flags);
unsigned int is_lp_cluster(void);
unsigned long tegra_get_lpcpu_max_rate(void);
#endif

#define FLOW_CTRL_HALT_CPU(cpu)	(IO_ADDRESS(TEGRA_FLOW_CTRL_BASE) + \
	((cpu) == 0 ? 0x8 : (0x18 + 8 * ((cpu) - 1))))
#define FLOW_CTRL_CPU_CSR(cpu)	(IO_ADDRESS(TEGRA_FLOW_CTRL_BASE) + \
	((cpu) == 0 ? 0x0 : (0x4 + cpu * 0x10)))

static inline void flowctrl_writel(unsigned long val, void __iomem *addr)
{
	writel(val, addr);
#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	wmb();
#endif
	(void)__raw_readl(addr);
}

#endif /* _MACH_TEGRA_SUSPEND_H_ */
