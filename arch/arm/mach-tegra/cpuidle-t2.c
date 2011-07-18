/*
 * arch/arm/mach-tegra/cpuidle-t2.c
 *
 * CPU idle driver for Tegra2 CPUs
 *
 * Copyright (c) 2010-2011, NVIDIA Corporation.
 * Copyright (c) 2011 Google, Inc.
 * Author: Colin Cross <ccross@android.com>
 *         Gary King <gking@nvidia.com>
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
#include <linux/cpu.h>
#include <linux/cpuidle.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/sched.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/smp.h>
#include <linux/suspend.h>
#include <linux/tick.h>

#include <asm/cpu_pm.h>

#include <mach/iomap.h>
#include <mach/irqs.h>

#include "cpuidle.h"
#include "pm.h"
#include "sleep.h"

static struct {
	unsigned int cpu_ready_count[2];
	unsigned long long cpu_wants_lp2_time[2];
	unsigned long long in_lp2_time;
	unsigned int both_idle_count;
	unsigned int tear_down_count;
	unsigned int lp2_count;
	unsigned int lp2_count_bin[32];
	unsigned int lp2_int_count[NR_IRQS];
	unsigned int last_lp2_int_count[NR_IRQS];
} idle_stats;

void tegra2_cpu_idle_stats_lp2_ready(unsigned int cpu)
{
	idle_stats.cpu_ready_count[cpu]++;
}

void tegra2_cpu_idle_stats_lp2_time(unsigned int cpu, s64 us)
{
	idle_stats.cpu_wants_lp2_time[cpu] += us;
}

#ifdef CONFIG_DEBUG_FS
int tegra2_lp2_debug_show(struct seq_file *s, void *data)
{
	int bin;
	int i;
	seq_printf(s, "                                    cpu0     cpu1\n");
	seq_printf(s, "-------------------------------------------------\n");
	seq_printf(s, "cpu ready:                      %8u %8u\n",
		idle_stats.cpu_ready_count[0],
		idle_stats.cpu_ready_count[1]);
	seq_printf(s, "both idle:      %8u        %7u%% %7u%%\n",
		idle_stats.both_idle_count,
		idle_stats.both_idle_count * 100 /
			(idle_stats.cpu_ready_count[0] ?: 1),
		idle_stats.both_idle_count * 100 /
			(idle_stats.cpu_ready_count[1] ?: 1));
	seq_printf(s, "tear down:      %8u %7u%%\n", idle_stats.tear_down_count,
		idle_stats.tear_down_count * 100 /
			(idle_stats.both_idle_count ?: 1));
	seq_printf(s, "lp2:            %8u %7u%%\n", idle_stats.lp2_count,
		idle_stats.lp2_count * 100 /
			(idle_stats.both_idle_count ?: 1));

	seq_printf(s, "\n");
	seq_printf(s, "cpu ready time:                 %8llu %8llu ms\n",
		div64_u64(idle_stats.cpu_wants_lp2_time[0], 1000),
		div64_u64(idle_stats.cpu_wants_lp2_time[1], 1000));
	seq_printf(s, "lp2 time:       %8llu ms     %7d%% %7d%%\n",
		div64_u64(idle_stats.in_lp2_time, 1000),
		(int)div64_u64(idle_stats.in_lp2_time * 100,
			idle_stats.cpu_wants_lp2_time[0] ?: 1),
		(int)div64_u64(idle_stats.in_lp2_time * 100,
			idle_stats.cpu_wants_lp2_time[1] ?: 1));

	seq_printf(s, "\n");
	seq_printf(s, "%19s %8s\n", "", "lp2");
	seq_printf(s, "-------------------------------------------------\n");
	for (bin = 0; bin < 32; bin++) {
		if (idle_stats.lp2_count_bin[bin] == 0)
			continue;
		seq_printf(s, "%6u - %6u ms: %8u\n",
			1 << (bin - 1), 1 << bin,
			idle_stats.lp2_count_bin[bin]);
	}

	seq_printf(s, "\n");
	seq_printf(s, "%3s %20s %6s %10s\n",
		"int", "name", "count", "last count");
	seq_printf(s, "--------------------------------------------\n");
	for (i = 0; i < NR_IRQS; i++) {
		if (idle_stats.lp2_int_count[i] == 0)
			continue;
		seq_printf(s, "%3d %20s %6d %10d\n",
			i, irq_to_desc(i)->action ?
				irq_to_desc(i)->action->name ?: "???" : "???",
			idle_stats.lp2_int_count[i],
			idle_stats.lp2_int_count[i] -
				idle_stats.last_lp2_int_count[i]);
		idle_stats.last_lp2_int_count[i] = idle_stats.lp2_int_count[i];
	};
	return 0;
}
#endif
