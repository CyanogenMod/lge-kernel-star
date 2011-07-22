/*
 * arch/arm/mach-tegra/cpuidle.c
 *
 * CPU idle driver for Tegra CPUs
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

#ifdef CONFIG_PM_SLEEP
static bool lp2_in_idle __read_mostly = true;
module_param(lp2_in_idle, bool, 0644);
static bool lp2_disabled_by_suspend;

int tegra_lp2_exit_latency;
static unsigned int tegra_lp2_min_residency;
#endif

struct cpuidle_driver tegra_idle = {
	.name = "tegra_idle",
	.owner = THIS_MODULE,
};

static DEFINE_PER_CPU(struct cpuidle_device *, idle_devices);

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

void tegra_lp2_in_idle(bool enable)
{
#ifdef CONFIG_PM_SLEEP
	lp2_in_idle = enable;
#endif
}

#ifdef CONFIG_PM_SLEEP
static int tegra_idle_enter_lp2(struct cpuidle_device *dev,
	struct cpuidle_state *state)
{
	ktime_t enter, exit;
	s64 us;

	if (!lp2_in_idle || lp2_disabled_by_suspend ||
	    !tegra_lp2_is_allowed(dev, state))
		return tegra_idle_enter_lp3(dev, state);

	local_irq_disable();
	enter = ktime_get();

	tegra_cpu_idle_stats_lp2_ready(dev->cpu);

	clockevents_notify(CLOCK_EVT_NOTIFY_BROADCAST_ENTER, &dev->cpu);
	tegra_idle_lp2(dev, state);
	clockevents_notify(CLOCK_EVT_NOTIFY_BROADCAST_EXIT, &dev->cpu);

	exit = ktime_sub(ktime_get(), enter);
	us = ktime_to_us(exit);

	local_irq_enable();

	/* cpu clockevents may have been reset by powerdown */
	hrtimer_peek_ahead_timers();

	smp_rmb();
	if (state->target_residency < tegra_lp2_min_residency)
		state->target_residency = tegra_lp2_min_residency;

	tegra_cpu_idle_stats_lp2_time(dev->cpu, us);

	return (int)us;
}
#endif

static int tegra_idle_prepare(struct cpuidle_device *dev)
{
#ifdef CONFIG_PM_SLEEP
	if (lp2_in_idle)
		dev->states[1].flags &= ~CPUIDLE_FLAG_IGNORE;
	else
		dev->states[1].flags |= CPUIDLE_FLAG_IGNORE;
#endif

	return 0;
}

static int tegra_cpuidle_register_device(unsigned int cpu)
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

#ifdef CONFIG_PM_SLEEP
	state = &dev->states[1];
	snprintf(state->name, CPUIDLE_NAME_LEN, "LP2");
	snprintf(state->desc, CPUIDLE_DESC_LEN, "CPU power-gate");
	state->exit_latency = tegra_cpu_power_good_time();

	state->target_residency = tegra_cpu_power_off_time() +
		tegra_cpu_power_good_time();
	if (state->target_residency < tegra_lp2_min_residency)
		state->target_residency = tegra_lp2_min_residency;
	state->power_usage = 0;
	state->flags = CPUIDLE_FLAG_TIME_VALID;
	state->enter = tegra_idle_enter_lp2;

	dev->power_specified = 1;
	dev->safe_state = state;
	dev->state_count++;
#endif

	dev->prepare = tegra_idle_prepare;

	if (cpuidle_register_device(dev)) {
		pr_err("CPU%u: failed to register idle device\n", cpu);
		kfree(dev);
		return -EIO;
	}
	per_cpu(idle_devices, cpu) = dev;
	return 0;
}

static int tegra_cpuidle_pm_notify(struct notifier_block *nb,
	unsigned long event, void *dummy)
{
#ifdef CONFIG_PM_SLEEP
	if (event == PM_SUSPEND_PREPARE)
		lp2_disabled_by_suspend = true;
	else if (event == PM_POST_SUSPEND)
		lp2_disabled_by_suspend = false;
#endif

	return NOTIFY_OK;
}

static struct notifier_block tegra_cpuidle_pm_notifier = {
	.notifier_call = tegra_cpuidle_pm_notify,
};

static int __init tegra_cpuidle_init(void)
{
	unsigned int cpu;
	int ret;

	ret = cpuidle_register_driver(&tegra_idle);
	if (ret)
		return ret;

#ifdef CONFIG_PM_SLEEP
	/* !!!FIXME!!! Add tegra_lp2_power_off_time */
	tegra_lp2_min_residency = tegra_cpu_lp2_min_residency();
#endif

	for_each_possible_cpu(cpu) {
		if (tegra_cpuidle_register_device(cpu))
			pr_err("CPU%u: error initializing idle loop\n", cpu);
	}

	register_pm_notifier(&tegra_cpuidle_pm_notifier);
	return 0;
}

static void __exit tegra_cpuidle_exit(void)
{
	unregister_pm_notifier(&tegra_cpuidle_pm_notifier);
	cpuidle_unregister_driver(&tegra_idle);
}

module_init(tegra_cpuidle_init);
module_exit(tegra_cpuidle_exit);

#if defined(CONFIG_DEBUG_FS) && defined(CONFIG_PM_SLEEP)
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
