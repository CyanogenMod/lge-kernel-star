/*
 * arch/arm/mach-tegra/include/mach/pm.h
 *
 * Copyright (C) 2010 Google, Inc.
 * Copyright (C) 2010-2011 NVIDIA Corporation
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


#ifndef _MACH_TEGRA_PM_H_
#define _MACH_TEGRA_PM_H_

#include <linux/mutex.h>

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
	unsigned long cpu_lp2_min_residency; /* Min LP2 state residency in us */
};

unsigned long tegra_cpu_power_good_time(void);
unsigned long tegra_cpu_power_off_time(void);
unsigned long tegra_cpu_lp2_min_residency(void);
void tegra_clear_cpu_in_lp2(int cpu);
bool tegra_set_cpu_in_lp2(int cpu);

int tegra_suspend_dram(enum tegra_suspend_mode mode);

#define TEGRA_POWER_SDRAM_SELFREFRESH	0x400	/* SDRAM is in self-refresh */

#define TEGRA_POWER_CLUSTER_G		0x1000	/* G CPU */
#define TEGRA_POWER_CLUSTER_LP		0x2000	/* LP CPU */
#define TEGRA_POWER_CLUSTER_MASK	0x3000
#define TEGRA_POWER_CLUSTER_IMMEDIATE	0x4000	/* Immediate wake */
#define TEGRA_POWER_CLUSTER_FORCE	0x8000	/* Force switch */

#define FLOW_CTRL_HALT_CPU(cpu)	(IO_ADDRESS(TEGRA_FLOW_CTRL_BASE) + \
	((cpu) == 0 ? 0x0 : 0x14 + ((cpu) - 1) * 0x8))
#define FLOW_CTRL_CPU_CSR(cpu)	(IO_ADDRESS(TEGRA_FLOW_CTRL_BASE) + \
	((cpu) == 0 ? 0x8 : 0x18 + ((cpu) - 1) * 0x8))

#define FLOW_CTRL_CLUSTER_CONTROL \
	(IO_ADDRESS(TEGRA_FLOW_CTRL_BASE) + 0x2c)
#define FLOW_CTRL_CPU_CSR_IMMEDIATE_WAKE	(1<<3)
#define FLOW_CTRL_CPU_CSR_SWITCH_CLUSTER	(1<<2)

#define FUSE_SKU_DIRECT_CONFIG \
	(IO_ADDRESS(TEGRA_FUSE_BASE) + 0x1F4)
#define FUSE_SKU_DISABLE_ALL_CPUS	(1<<5)
#define FUSE_SKU_NUM_DISABLED_CPUS(x)	(((x) >> 3) & 3)

void __init tegra_init_suspend(struct tegra_suspend_platform_data *plat);

unsigned int tegra_count_slow_cpus(unsigned long speed_limit);
unsigned int tegra_get_slowest_cpu_n(void);
unsigned long tegra_cpu_lowest_speed(void);
unsigned long tegra_cpu_highest_speed(void);
int tegra_cpu_cap_highest_speed(unsigned int *speed_cap);

#if defined(CONFIG_TEGRA_AUTO_HOTPLUG) && !defined(CONFIG_ARCH_TEGRA_2x_SOC)
int tegra_auto_hotplug_init(struct mutex *cpu_lock);
void tegra_auto_hotplug_exit(void);
void tegra_auto_hotplug_governor(unsigned int cpu_freq);
#else
static inline int tegra_auto_hotplug_init(struct mutex *cpu_lock)
{ return 0; }
static inline void tegra_auto_hotplug_exit(void)
{ }
static inline void tegra_auto_hotplug_governor(unsigned int cpu_freq)
{ }
#endif

u64 tegra_rtc_read_ms(void);

/*
 * Callbacks for platform drivers to implement.
 */
extern void (*tegra_deep_sleep)(int);

void tegra_idle_lp2_last(unsigned int flags);
#ifdef CONFIG_ARCH_TEGRA_2x_SOC
#define INSTRUMENT_CLUSTER_SWITCH 0	/* Must be zero for ARCH_TEGRA_2x_SOC */
#define DEBUG_CLUSTER_SWITCH 0		/* Must be zero for ARCH_TEGRA_2x_SOC */
#define PARAMETERIZE_CLUSTER_SWITCH 0	/* Must be zero for ARCH_TEGRA_2x_SOC */
static inline int tegra_cluster_control(unsigned int us, unsigned int flags)
{ return -EPERM; }
#define tegra_cluster_switch_prolog(flags) do {} while(0)
#define tegra_cluster_switch_epilog(flags) do {} while(0)
static inline bool is_g_cluster_present(void)
{ return true; }
static inline unsigned int is_lp_cluster(void)
{ return 0; }
#define tegra_lp0_suspend_mc() do {} while(0)
#define tegra_lp0_resume_mc() do {} while(0)
void tegra2_lp0_suspend_init(void);

#else

#define INSTRUMENT_CLUSTER_SWITCH 1	/* Should be zero for shipping code */
#define DEBUG_CLUSTER_SWITCH 1		/* Should be zero for shipping code */
#define PARAMETERIZE_CLUSTER_SWITCH 1	/* Should be zero for shipping code */
int tegra_cluster_control(unsigned int us, unsigned int flags);
void tegra_cluster_switch_prolog(unsigned int flags);
void tegra_cluster_switch_epilog(unsigned int flags);
static inline bool is_g_cluster_present(void)
{
	u32 fuse_sku = readl(FUSE_SKU_DIRECT_CONFIG);
	if (fuse_sku & FUSE_SKU_DISABLE_ALL_CPUS)
		return false;
	return true;
}
static inline unsigned int is_lp_cluster(void)
{
	unsigned int reg;
	reg = readl(FLOW_CTRL_CLUSTER_CONTROL);
	return (reg & 1); /* 0 == G, 1 == LP*/
}
void tegra_lp0_suspend_mc(void);
void tegra_lp0_resume_mc(void);
#endif

static inline void tegra_lp0_suspend_init(void)
{
#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	tegra2_lp0_suspend_init();
#endif
}

#ifdef CONFIG_ARCH_TEGRA_2x_SOC
void tegra2_lp2_set_trigger(unsigned long cycles);
unsigned long tegra2_lp2_timer_remain(void);
#endif

static inline void tegra_lp2_set_trigger(unsigned long cycles)
{
#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	tegra2_lp2_set_trigger(cycles);
#endif
}

static inline unsigned long tegra_lp2_timer_remain(void)
{
#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	return tegra2_lp2_timer_remain();
#endif
}

#if DEBUG_CLUSTER_SWITCH
extern unsigned int tegra_cluster_debug;
#define DEBUG_CLUSTER(x) do { if (tegra_cluster_debug) printk x; } while (0)
#else
#define DEBUG_CLUSTER(x) do { } while (0)
#endif
#if PARAMETERIZE_CLUSTER_SWITCH
void tegra_cluster_switch_set_parameters(unsigned int us, unsigned int flags);
#else
static inline void tegra_cluster_switch_set_parameters(
	unsigned int us, unsigned int flags)
{ }
#endif

static inline void flowctrl_writel(unsigned long val, void __iomem *addr)
{
	writel(val, addr);
#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	wmb();
#endif
	(void)__raw_readl(addr);
}

#endif /* _MACH_TEGRA_PM_H_ */
