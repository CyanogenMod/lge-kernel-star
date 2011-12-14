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
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/clkdev.h>

#include <mach/iomap.h>

enum tegra_suspend_mode {
	TEGRA_SUSPEND_NONE = 0,
	TEGRA_SUSPEND_LP2,	/* CPU voltage off */
	TEGRA_SUSPEND_LP1,	/* CPU voltage off, DRAM self-refresh */
	TEGRA_SUSPEND_LP0,	/* CPU + core voltage off, DRAM self-refresh */
	TEGRA_MAX_SUSPEND_MODE,
};

enum suspend_stage {
	TEGRA_SUSPEND_BEFORE_PERIPHERAL,
	TEGRA_SUSPEND_BEFORE_CPU,
};

enum resume_stage {
	TEGRA_RESUME_AFTER_PERIPHERAL,
	TEGRA_RESUME_AFTER_CPU,
};

struct tegra_suspend_platform_data {
	unsigned long cpu_timer;   /* CPU power good time in us,  LP2/LP1 */
	unsigned long cpu_off_timer;	/* CPU power off time us, LP2/LP1 */
	unsigned long core_timer;  /* core power good time in ticks,  LP0 */
	unsigned long core_off_timer;	/* core power off time ticks, LP0 */
	bool corereq_high;         /* Core power request active-high */
	bool sysclkreq_high;       /* System clock request is active-high */
	bool combined_req;         /* if core & CPU power requests are combined */
	enum tegra_suspend_mode suspend_mode;
	unsigned long cpu_lp2_min_residency; /* Min LP2 state residency in us */
	void (*board_suspend)(int lp_state, enum suspend_stage stg);
	/* lp_state = 0 for LP0 state, 1 for LP1 state, 2 for LP2 state */
	void (*board_resume)(int lp_state, enum resume_stage stg);
};

unsigned long tegra_cpu_power_good_time(void);
unsigned long tegra_cpu_power_off_time(void);
unsigned long tegra_cpu_lp2_min_residency(void);
void tegra_clear_cpu_in_lp2(int cpu);
bool tegra_set_cpu_in_lp2(int cpu);

int tegra_suspend_dram(enum tegra_suspend_mode mode, unsigned int flags);

#define FLOW_CTRL_CLUSTER_CONTROL \
	(IO_ADDRESS(TEGRA_FLOW_CTRL_BASE) + 0x2c)
#define FLOW_CTRL_CPU_CSR_IMMEDIATE_WAKE	(1<<3)
#define FLOW_CTRL_CPU_CSR_SWITCH_CLUSTER	(1<<2)

#define FUSE_SKU_DIRECT_CONFIG \
	(IO_ADDRESS(TEGRA_FUSE_BASE) + 0x1F4)
#define FUSE_SKU_DISABLE_ALL_CPUS	(1<<5)
#define FUSE_SKU_NUM_DISABLED_CPUS(x)	(((x) >> 3) & 3)

void __init tegra_init_suspend(struct tegra_suspend_platform_data *plat);

u64 tegra_rtc_read_ms(void);

/*
 * Callbacks for platform drivers to implement.
 */
extern void (*tegra_deep_sleep)(int);

unsigned int tegra_idle_lp2_last(unsigned int us, unsigned int flags);

#if defined(CONFIG_PM_SLEEP) && !defined(CONFIG_ARCH_TEGRA_2x_SOC)
void tegra_lp0_suspend_mc(void);
void tegra_lp0_resume_mc(void);
void tegra_lp0_cpu_mode(bool enter);
#else
static inline void tegra_lp0_suspend_mc(void) {}
static inline void tegra_lp0_resume_mc(void) {}
static inline void tegra_lp0_cpu_mode(bool enter) {}
#endif

#ifdef CONFIG_TEGRA_CLUSTER_CONTROL
#define INSTRUMENT_CLUSTER_SWITCH 0	/* Should be zero for shipping code */
#define DEBUG_CLUSTER_SWITCH 0		/* Should be zero for shipping code */
#define PARAMETERIZE_CLUSTER_SWITCH 1	/* Should be zero for shipping code */

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
int tegra_cluster_control(unsigned int us, unsigned int flags);
void tegra_cluster_switch_prolog(unsigned int flags);
void tegra_cluster_switch_epilog(unsigned int flags);
#else
#define INSTRUMENT_CLUSTER_SWITCH 0	/* Must be zero for ARCH_TEGRA_2x_SOC */
#define DEBUG_CLUSTER_SWITCH 0		/* Must be zero for ARCH_TEGRA_2x_SOC */
#define PARAMETERIZE_CLUSTER_SWITCH 0	/* Must be zero for ARCH_TEGRA_2x_SOC */

static inline bool is_g_cluster_present(void)   { return true; }
static inline unsigned int is_lp_cluster(void)  { return 0; }
static inline int tegra_cluster_control(unsigned int us, unsigned int flags)
{
	return -EPERM;
}
static inline void tegra_cluster_switch_prolog(unsigned int flags) {}
static inline void tegra_cluster_switch_epilog(unsigned int flags) {}
#endif

#ifdef CONFIG_ARCH_TEGRA_2x_SOC
void tegra2_lp0_suspend_init(void);
void tegra2_lp2_set_trigger(unsigned long cycles);
unsigned long tegra2_lp2_timer_remain(void);
#endif
#ifdef CONFIG_ARCH_TEGRA_3x_SOC
void tegra3_lp2_set_trigger(unsigned long cycles);
unsigned long tegra3_lp2_timer_remain(void);
#endif

static inline void tegra_lp0_suspend_init(void)
{
#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	tegra2_lp0_suspend_init();
#endif
}

static inline void tegra_lp2_set_trigger(unsigned long cycles)
{
#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	tegra2_lp2_set_trigger(cycles);
#endif
#ifdef CONFIG_ARCH_TEGRA_3x_SOC
	tegra3_lp2_set_trigger(cycles);
#endif
}

static inline unsigned long tegra_lp2_timer_remain(void)
{
#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	return tegra2_lp2_timer_remain();
#endif
#ifdef CONFIG_ARCH_TEGRA_3x_SOC
	return tegra3_lp2_timer_remain();
#endif
}

#if DEBUG_CLUSTER_SWITCH && 0 /* !!!FIXME!!! THIS IS BROKEN */
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

#ifdef CONFIG_SMP
extern bool tegra_all_cpus_booted __read_mostly;
#else
#define tegra_all_cpus_booted (true)
#endif

#ifdef CONFIG_TRUSTED_FOUNDATIONS
void tegra_generic_smc(u32 type, u32 subtype, u32 arg);
#endif

/* The debug channel uart base physical address */
extern unsigned long  debug_uart_port_base;

extern struct clk *debug_uart_clk;
void tegra_console_uart_suspend(void);
void tegra_console_uart_resume(void);

#endif /* _MACH_TEGRA_PM_H_ */
