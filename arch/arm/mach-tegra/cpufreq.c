/*
 * arch/arm/mach-tegra/cpufreq.c
 *
 * cpufreq driver for NVIDIA Tegra SoCs
 *
 * Copyright (c) 2008-2010, NVIDIA Corporation.
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
#include <linux/types.h>
#include <linux/init.h>
#include <linux/cpufreq.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/smp.h>
#include <linux/cpu.h>
#include <linux/freezer.h>
#include <linux/kthread.h>
#include <linux/workqueue.h>
#include <linux/smp_lock.h>
#include <linux/suspend.h>
// 20100728 cs77.ha@lge.com related deepsleep wakeup delay, (NVIDIA john moser) [START]
#include <linux/delay.h>
// 20100728 cs77.ha@lge.com related deepsleep wakeup delay, (NVIDIA john moser) [END]
#include <linux/reboot.h>
#include <linux/delay.h>

#include <asm/system.h>
#include <asm/smp_twd.h>

#include <mach/hardware.h>
#include <mach/nvrm_linux.h>
#include <mach/timex.h>

#include <nvrm_power.h>
#include <nvrm_power_private.h>

#define KTHREAD_IRQ_PRIO (MAX_RT_PRIO>>1)

static NvRmDeviceHandle rm_cpufreq = NULL;
static struct task_struct *cpufreq_dfsd = NULL;
static struct clk *clk_cpu = NULL;

static DEFINE_MUTEX(init_mutex);

#ifdef CONFIG_HOTPLUG_CPU
static int disable_hotplug = 0;
#endif

static void tegra_cpufreq_hotplug(NvRmPmRequest req)
{
	int rc = 0;
#ifdef CONFIG_HOTPLUG_CPU
	unsigned int cpu;

	smp_rmb();
	if (disable_hotplug)
		return;
        #if 0
		// 20101002 sunghoon.kim@lge.com disable dual core and dvfs enable for single core [START]
	        req = NvRmPmRequest_CpuOffFlag;
        	disable_hotplug = 1;
		// 20101002 sunghoon.kim@lge.com disable dual core and dvfs enable for single core [STOP]
        #endif
	if (req & NvRmPmRequest_CpuOnFlag) {
		struct cpumask m;

		cpumask_andnot(&m, cpu_present_mask, cpu_online_mask);
		cpu = cpumask_any(&m);

		if (cpu_present(cpu) && !cpu_online(cpu))
			rc = cpu_up(cpu);

	} else if (req & NvRmPmRequest_CpuOffFlag) {
		cpu = cpumask_any_but(cpu_online_mask, 0);

		if (cpu_present(cpu) && cpu_online(cpu))
			rc = cpu_down(cpu);
	}
#endif
	if (rc)
		pr_err("%s: error %d servicing hot plug request\n",
		       __func__, rc);
}

#ifdef CONFIG_HOTPLUG_CPU
static int tegra_cpufreq_pm_notifier(struct notifier_block *nfb,
				     unsigned long event, void *data)
{
	switch (event) {
	case PM_SUSPEND_PREPARE:
		disable_hotplug = 1;
		smp_wmb();
		break;
	case PM_POST_SUSPEND:
		disable_hotplug = 0;
		smp_wmb();
		break;
	default:
		pr_err("%s: unknown event %lu\n", __func__, event);
		return NOTIFY_DONE;
	}

	return NOTIFY_OK;
}
#endif

static int dfs_reboot_notify(struct notifier_block *nb,
			     unsigned long event, void *data)
{
	switch (event) {
	case SYS_RESTART:
	case SYS_HALT:
	case SYS_POWER_OFF:
		/* Warm boot setting at max voltages works for any reboot */
		NvRmPrivDfsSuspend(NvOdmSocPowerState_DeepSleep);
		return NOTIFY_OK;
	}
	return NOTIFY_DONE;
}

static struct notifier_block dfs_reboot_nb = {
	.notifier_call = dfs_reboot_notify,
	.next = NULL,
	.priority = 0
};

static int tegra_cpufreq_dfsd(void *arg)
{
	unsigned long rate, last_rate;
	NvRmPmRequest req = 0;

	BUG_ON(!clk_cpu);

	preset_lpj = loops_per_jiffy;
	rate = clk_get_rate(clk_cpu);
	last_rate = rate;

	NvRmDfsSetState(rm_cpufreq, NvRmDfsRunState_ClosedLoop);
	set_freezable_with_signal();

	while (!kthread_should_stop() && !(req & NvRmPmRequest_ExitFlag)) {

		req = NvRmPrivPmThread();

		if (try_to_freeze())
			continue;

		tegra_cpufreq_hotplug(req);

#ifdef CONFIG_USE_ARM_TWD_PRESCALER
		rate = clk_get_rate(clk_cpu);
		if (rate != last_rate) {
			local_timer_rescale(rate / 1000);
			smp_wmb();
			on_each_cpu(twd_set_prescaler, NULL, true);
			last_rate = rate;
		}
#endif
	}
	pr_info("dvfs thead shutdown\n");

	return 0;
}

static int tegra_verify_speed(struct cpufreq_policy *policy)
{
	cpufreq_verify_within_limits(policy, policy->cpuinfo.min_freq,
		policy->cpuinfo.max_freq);
	return 0;
}

static unsigned int tegra_get_speed(unsigned int cpu)
{
	unsigned long rate;

	rate = clk_get_rate(clk_cpu);
	return rate / 1000;
}

static int tegra_set_policy(struct cpufreq_policy *pol)
{
	NvError e = NvRmDfsSetCpuEnvelope(rm_cpufreq, pol->min, pol->max);

	if (e) {
		pr_err("%s: error 0x%08x \n", __func__, e);
		return -EINVAL;
	}
	return 0;
}

int tegra_start_dvfsd(void) {
	int rc = 0;
	static bool started = false;

	mutex_lock(&init_mutex);
	if (cpufreq_dfsd && !started) {
		wake_up_process(cpufreq_dfsd);
		started = true;
	} else
		rc = -ENOSYS;
	mutex_unlock(&init_mutex);

	return rc;
}

static int tegra_cpufreq_init_once(void)
{
	struct sched_param sp;
	int rc = 0;

	mutex_lock(&init_mutex);

	if (rm_cpufreq)
		goto clean;

	if (NvRmOpenNew(&rm_cpufreq)!=NvSuccess) {
		pr_err("%s: unable to open NvRm\n", __func__);
		rc = -ENOSYS;
		goto clean;
	}

	clk_cpu = clk_get_sys(NULL, "cpu");
	if (IS_ERR(clk_cpu)) {
		rc = PTR_ERR(clk_cpu);
		clk_cpu = NULL;
		goto clean;
	}

	rc = register_reboot_notifier(&dfs_reboot_nb);
	if (rc) {
		pr_err("%s: unable to regsiter DVFS reboot notifier\n", __func__);
		goto clean;
	}

	cpufreq_dfsd = kthread_create(tegra_cpufreq_dfsd, NULL, "cpufreq-dvfsd");
	if (IS_ERR(cpufreq_dfsd)) {
		pr_err("%s: unable to start DVFS daemon\n", __func__);
		rc = PTR_ERR(cpufreq_dfsd);
		cpufreq_dfsd = NULL;
		goto clean;
	}

	sp.sched_priority = KTHREAD_IRQ_PRIO + 1;
	if (sched_setscheduler_nocheck(cpufreq_dfsd, SCHED_FIFO, &sp) < 0)
		pr_err("%s: unable to elevate DVFS daemon priority\n",__func__);

clean:
	if (rc) {
		if (rm_cpufreq)
			NvRmClose(rm_cpufreq);
		if (clk_cpu)
			clk_put(clk_cpu);
		clk_cpu = NULL;
		rm_cpufreq = NULL;
		unregister_reboot_notifier(&dfs_reboot_nb);
	}

	mutex_unlock(&init_mutex);
	return rc;
}

static int tegra_cpufreq_driver_init(struct cpufreq_policy *pol)
{
	NvRmDfsClockUsage usage;
	NvError e;
	int rc;

	rc = tegra_cpufreq_init_once();
	if (rc)
		return rc;

	e = NvRmDfsGetClockUtilization(rm_cpufreq, NvRmDfsClockId_Cpu, &usage);

	if (e != NvSuccess) {
		WARN_ON(1);
		return -ENXIO;
	}

	pr_debug("%s: min: %u max: %u current: %u\n",
		 __func__, usage.MinKHz, usage.MaxKHz, usage.CurrentKHz);

	pol->min = usage.LowCornerKHz;
	pol->max = usage.HighCornerKHz;
	pol->cur = usage.CurrentKHz;

	pol->cpuinfo.min_freq = usage.MinKHz;
	pol->cpuinfo.max_freq = usage.MaxKHz;
	pol->cpuinfo.transition_latency = 0;

	return 0;
}

static struct cpufreq_driver s_tegra_cpufreq_driver = {
	.flags		= CPUFREQ_CONST_LOOPS,
	.verify		= tegra_verify_speed,
	.setpolicy	= tegra_set_policy,
	.get		= tegra_get_speed,
	.init		= tegra_cpufreq_driver_init,
	.name		= "tegra_cpufreq",
	.owner		= THIS_MODULE,

};

static int __init tegra_cpufreq_init(void)
{
#ifdef CONFIG_HOTPLUG_CPU
	pm_notifier(tegra_cpufreq_pm_notifier, 0);
#endif
	return cpufreq_register_driver(&s_tegra_cpufreq_driver);
}

static void __exit tegra_cpufreq_exit(void)
{
	kthread_stop(cpufreq_dfsd);
	clk_put(clk_cpu);
	unregister_reboot_notifier(&dfs_reboot_nb);

	cpufreq_unregister_driver(&s_tegra_cpufreq_driver);
}

MODULE_DESCRIPTION("CPU frequency driver for the Tegra SOC");
MODULE_LICENSE("GPL");
module_init(tegra_cpufreq_init);
module_exit(tegra_cpufreq_exit);
