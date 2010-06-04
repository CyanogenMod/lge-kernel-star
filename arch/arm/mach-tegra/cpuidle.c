/*
 * arch/arm/mach-tegra/cpuidle.c
 *
 * CPU idle driver for Tegra CPUs
 *
 * Copyright (c) 2010, NVIDIA Corporation.
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
#include <linux/smp.h>
#include <linux/sched.h>
#include <linux/cpuidle.h>
#include <linux/hrtimer.h>
#include <linux/cpu.h>
#include <linux/io.h>
#include <linux/tick.h>
#include <linux/interrupt.h>
#include <mach/iomap.h>
#include <linux/suspend.h>

static unsigned int latency_factor __read_mostly = 2;
static unsigned int system_is_suspending = 0;
module_param(latency_factor, uint, 0644);

struct cpuidle_driver tegra_idle = {
	.name = "tegra_idle",
	.owner = THIS_MODULE,
};

static DEFINE_PER_CPU(struct cpuidle_device *, idle_devices);

static int lp2_supported = 0;

#define FLOW_CTRL_WAITEVENT   (2<<29)
#define FLOW_CTRL_JTAG_RESUME (1<<28)
#define FLOW_CTRL_HALT_CPUx_EVENTS(cpu) ((cpu)?((cpu-1)*0x8 + 0x14) : 0x0)

#define PMC_SCRATCH_38 0x134
#define PMC_SCRATCH_39 0x138

#define CLK_RESET_CLK_MASK_ARM 0x44

static int tegra_idle_enter_lp3(struct cpuidle_device *dev,
	struct cpuidle_state *state)
{
	void __iomem *flow_ctrl = IO_ADDRESS(TEGRA_FLOW_CTRL_BASE);
	ktime_t enter, exit;
	s64 us;
	u32 reg = FLOW_CTRL_WAITEVENT | FLOW_CTRL_JTAG_RESUME;

	flow_ctrl = flow_ctrl + FLOW_CTRL_HALT_CPUx_EVENTS(dev->cpu);
	local_irq_disable();

	smp_rmb();
	if (lp2_supported)
		hrtimer_peek_ahead_timers();

	enter = ktime_get();
	if (!need_resched()) {
		dsb();
		__raw_writel(reg, flow_ctrl);
		reg = __raw_readl(flow_ctrl);
		__asm__ volatile ("wfi");
		__raw_writel(0, flow_ctrl);
		reg = __raw_readl(flow_ctrl);
	}
	exit = ktime_get();
	enter = ktime_sub(exit, enter);
	us = ktime_to_us(enter);
	local_irq_enable();
	return (int)us;
}

extern unsigned int tegra_suspend_lp2(unsigned int);

static int tegra_idle_enter_lp2(struct cpuidle_device *dev,
	struct cpuidle_state *state)
{
	ktime_t enter, exit;
	s64 request, us, latency;
	unsigned int last_sample = (unsigned int)cpuidle_get_statedata(state);

	/* LP2 not possible when running in SMP mode */
	smp_rmb();
	request = ktime_to_us(tick_nohz_get_sleep_length());
	if (!lp2_supported || request <= state->exit_latency ||
			system_is_suspending) {
		dev->last_state = &dev->states[0];
		return tegra_idle_enter_lp3(dev, &dev->states[0]);
	}

	local_irq_disable();
	enter = ktime_get();
	request -= state->exit_latency;
	us = tegra_suspend_lp2((unsigned int)max_t(s64, 200, request));
	exit = ktime_sub(ktime_get(), enter);
	latency = ktime_to_us(exit) - us;
	/* FIXME: un-hardcode the powergood timer latency */
	latency += 2000;
	cpuidle_set_statedata(state, (void*)(unsigned int)(latency));
	state->exit_latency = (12*latency + 4*last_sample) >> 4;

	state->target_residency = latency_factor*state->exit_latency;
	local_irq_enable();
	return (int)us;
}

static int tegra_idle_lp2_allowed(struct notifier_block *nfb,
	unsigned long action, void *hcpu)
{
	switch (action) {
	case CPU_UP_PREPARE:
		lp2_supported = 0;
		smp_wmb();
		break;
	case CPU_UP_CANCELED:
	case CPU_POST_DEAD:
		lp2_supported = (num_online_cpus()==1);
		smp_wmb();
		break;
	}

	return NOTIFY_OK;
}

static int tegra_cpuidle_notifier(struct notifier_block *nfb,
	unsigned long event, void *data)
{
	int notification = NOTIFY_OK;

	switch (event) {
	case PM_SUSPEND_PREPARE:
		system_is_suspending = 1;
		smp_wmb();
		break;
	case PM_POST_SUSPEND:
		system_is_suspending = 0;
		smp_wmb();
		break;
	default:
		printk(KERN_ERR "%s: unknown event %ld\n", __func__, event);
		notification = NOTIFY_DONE;
	}

	return notification;
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
	state->flags = CPUIDLE_FLAG_SHALLOW | CPUIDLE_FLAG_TIME_VALID;
	state->enter = tegra_idle_enter_lp3;
	dev->safe_state = state;
	dev->state_count++;

	if (cpu == 0) {
		state = &dev->states[1];
		snprintf(state->name, CPUIDLE_NAME_LEN, "LP2");
		snprintf(state->desc, CPUIDLE_DESC_LEN, "CPU power-gate");
		state->exit_latency = 2500;

		state->target_residency = state->exit_latency * latency_factor;
		state->power_usage = 0;
		state->flags = CPUIDLE_FLAG_BALANCED | CPUIDLE_FLAG_TIME_VALID;
		state->enter = tegra_idle_enter_lp2;
		cpuidle_set_statedata(state, (void*)state->exit_latency);

		dev->safe_state = state;
		dev->state_count++;
	}

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
	unsigned int cpu = smp_processor_id();
	void __iomem *mask_arm;
	unsigned int reg;
	int ret;

	mask_arm = IO_ADDRESS(TEGRA_CLK_RESET_BASE) + CLK_RESET_CLK_MASK_ARM;
	lp2_supported = (num_online_cpus()==1);
	hotcpu_notifier(tegra_idle_lp2_allowed, 0);
	pm_notifier(tegra_cpuidle_notifier, 0);

	reg = readl(mask_arm);
	writel(reg | (1<<31), mask_arm);

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
