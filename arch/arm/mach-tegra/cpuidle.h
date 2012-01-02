/*
 * arch/arm/mach-tegra/cpuidle.h
 *
 * Declarations for power state transition code
 *
 * Copyright (c) 2011, NVIDIA Corporation.
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

#ifndef __MACH_TEGRA_CPUIDLE_H
#define __MACH_TEGRA_CPUIDLE_H

#include <linux/cpuidle.h>

#ifdef CONFIG_PM_SLEEP

extern int tegra_lp2_exit_latency;

#ifdef CONFIG_ARCH_TEGRA_2x_SOC
void tegra2_idle_lp2(struct cpuidle_device *dev, struct cpuidle_state *state);
void tegra2_cpu_idle_stats_lp2_ready(unsigned int cpu);
void tegra2_cpu_idle_stats_lp2_time(unsigned int cpu, s64 us);
bool tegra2_lp2_is_allowed(struct cpuidle_device *dev,
			struct cpuidle_state *state);
#ifdef CONFIG_DEBUG_FS
int tegra2_lp2_debug_show(struct seq_file *s, void *data);
#endif
#endif
#ifdef CONFIG_ARCH_TEGRA_3x_SOC
void tegra3_idle_lp2(struct cpuidle_device *dev, struct cpuidle_state *state);
void tegra3_cpu_idle_stats_lp2_ready(unsigned int cpu);
void tegra3_cpu_idle_stats_lp2_time(unsigned int cpu, s64 us);
bool tegra3_lp2_is_allowed(struct cpuidle_device *dev,
			   struct cpuidle_state *state);
int tegra3_cpudile_init_soc(void);
#ifdef CONFIG_DEBUG_FS
int tegra3_lp2_debug_show(struct seq_file *s, void *data);
#endif
#endif

static inline int tegra_cpudile_init_soc(void)
{
#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	return 0;
#else
	return tegra3_cpudile_init_soc();
#endif
}

static inline void tegra_cpu_idle_stats_lp2_ready(unsigned int cpu)
{
#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	tegra2_cpu_idle_stats_lp2_ready(cpu);
#endif
#ifdef CONFIG_ARCH_TEGRA_3x_SOC
	tegra3_cpu_idle_stats_lp2_ready(cpu);
#endif
}

static inline void tegra_cpu_idle_stats_lp2_time(unsigned int cpu, s64 us)
{
#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	tegra2_cpu_idle_stats_lp2_time(cpu, us);
#endif
#ifdef CONFIG_ARCH_TEGRA_3x_SOC
	tegra3_cpu_idle_stats_lp2_time(cpu, us);
#endif
}

static inline void tegra_idle_lp2(struct cpuidle_device *dev,
			struct cpuidle_state *state)
{
#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	tegra2_idle_lp2(dev, state);
#endif
#ifdef CONFIG_ARCH_TEGRA_3x_SOC
	tegra3_idle_lp2(dev, state);
#endif
}

static inline bool tegra_lp2_is_allowed(struct cpuidle_device *dev,
			struct cpuidle_state *state)
{
#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	return tegra2_lp2_is_allowed(dev, state);
#endif
#ifdef CONFIG_ARCH_TEGRA_3x_SOC
	return tegra3_lp2_is_allowed(dev, state);
#endif
}

#ifdef CONFIG_DEBUG_FS
static inline int tegra_lp2_debug_show(struct seq_file *s, void *data)
{
#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	return tegra2_lp2_debug_show(s, data);
#endif
#ifdef CONFIG_ARCH_TEGRA_3x_SOC
	return tegra3_lp2_debug_show(s, data);
#endif
}
#endif
#endif /* CONFIG_PM_SLEEP */

#if defined(CONFIG_CPU_IDLE) && defined(CONFIG_PM_SLEEP)
void tegra_lp2_in_idle(bool enable);
#else
static inline void tegra_lp2_in_idle(bool enable) {}
#endif

#endif
