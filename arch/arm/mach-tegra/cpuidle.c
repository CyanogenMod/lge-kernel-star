/*
 * arch/arm/mach-tegra/cpuidle.c
 *
 * CPU idle driver for Tegra CPUs
 *
 * Copyright (c) 2010, NVIDIA Corporation.
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

#include "pm.h"
#include "sleep.h"

static bool lp2_in_idle __read_mostly = true;
module_param(lp2_in_idle, bool, 0644);

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

struct cpuidle_driver tegra_idle = {
	.name = "tegra_idle",
	.owner = THIS_MODULE,
};

static DEFINE_PER_CPU(struct cpuidle_device *, idle_devices);

static inline unsigned int time_to_bin(unsigned int time)
{
	return fls(time);
}

static int tegra_idle_enter_lp3(struct cpuidle_device *dev,
	struct cpuidle_state *state)
{
	ktime_t enter, exit;
	s64 us;

	local_irq_disable();
	local_fiq_disable();

	enter = ktime_get();

	tegra_cpu_wfi();

	exit = ktime_sub(ktime_get(), enter);
	us = ktime_to_us(exit);

	local_fiq_enable();
	local_irq_enable();
	return (int)us;
}

static int tegra_idle_enter_lp2(struct cpuidle_device *dev,
	struct cpuidle_state *state)
{
	ktime_t enter, exit;
	s64 us;

	local_irq_disable();
	enter = ktime_get();

	idle_stats.cpu_ready_count[dev->cpu]++;

	clockevents_notify(CLOCK_EVT_NOTIFY_BROADCAST_ENTER, &dev->cpu);
	tegra_idle_lp2();
	clockevents_notify(CLOCK_EVT_NOTIFY_BROADCAST_EXIT, &dev->cpu);

	exit = ktime_sub(ktime_get(), enter);
	us = ktime_to_us(exit);

	local_irq_enable();

	/* cpu clockevents may have been reset by powerdown */
	hrtimer_peek_ahead_timers();

	smp_rmb();

	idle_stats.cpu_wants_lp2_time[dev->cpu] += us;

	return (int)us;
}

static int tegra_idle_prepare(struct cpuidle_device *dev)
{
	if (lp2_in_idle)
		dev->states[1].flags &= ~CPUIDLE_FLAG_IGNORE;
	else
		dev->states[1].flags |= CPUIDLE_FLAG_IGNORE;

	return 0;
}

static int tegra_idle_enter(unsigned int cpu)
{
	struct cpuidle_device *dev;
	struct cpuidle_state *state;

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	dev->state_count = 0;
	dev->cpu = cpu;

	state = &dev->states[0];
	snprintf(state->name, CPUIDLE_NAME_LEN, "LP3");
	snprintf(state->desc, CPUIDLE_DESC_LEN, "CPU flow-controlled");
	state->exit_latency = 10;
	state->target_residency = 10;
	state->power_usage = 600;
	state->flags = CPUIDLE_FLAG_TIME_VALID;
	state->enter = tegra_idle_enter_lp3;
	dev->safe_state = state;
	dev->state_count++;

	state = &dev->states[1];
	snprintf(state->name, CPUIDLE_NAME_LEN, "LP2");
	snprintf(state->desc, CPUIDLE_DESC_LEN, "CPU power-gate");
	state->exit_latency = tegra_cpu_power_good_time();

	state->target_residency = tegra_cpu_power_off_time() +
		tegra_cpu_power_good_time();
	state->power_usage = 0;
	state->flags = CPUIDLE_FLAG_TIME_VALID;
	state->enter = tegra_idle_enter_lp2;

	dev->power_specified = 1;
	dev->safe_state = state;
	dev->state_count++;
	dev->prepare = tegra_idle_prepare;

	if (cpuidle_register_device(dev)) {
		pr_err("CPU%u: failed to register idle device\n", cpu);
		kfree(dev);
		return -EIO;
	}
	per_cpu(idle_devices, cpu) = dev;
	return 0;
}

static int __init tegra_cpuidle_init(void)
{
	unsigned int cpu;
	int ret;

	ret = cpuidle_register_driver(&tegra_idle);

	if (ret)
		return ret;

	for_each_possible_cpu(cpu) {
		if (tegra_idle_enter(cpu))
			pr_err("CPU%u: error initializing idle loop\n", cpu);
	}

	return 0;
}

static void __exit tegra_cpuidle_exit(void)
{
	cpuidle_unregister_driver(&tegra_idle);
}

module_init(tegra_cpuidle_init);
module_exit(tegra_cpuidle_exit);

#ifdef CONFIG_DEBUG_FS
static int tegra_lp2_debug_show(struct seq_file *s, void *data)
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

static int tegra_lp2_debug_open(struct inode *inode, struct file *file)
{
	return single_open(file, tegra_lp2_debug_show, inode->i_private);
}

static const struct file_operations tegra_lp2_debug_ops = {
	.open		= tegra_lp2_debug_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int __init tegra_cpuidle_debug_init(void)
{
	struct dentry *dir;
	struct dentry *d;

	dir = debugfs_create_dir("cpuidle", NULL);
	if (!dir)
		return -ENOMEM;

	d = debugfs_create_file("lp2", S_IRUGO, dir, NULL,
		&tegra_lp2_debug_ops);
	if (!d)
		return -ENOMEM;

	return 0;
}

late_initcall(tegra_cpuidle_debug_init);
#endif
