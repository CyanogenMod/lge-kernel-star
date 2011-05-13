/*
 * arch/arm/mach-tegra/cpu-tegra.c
 *
 * Copyright (C) 2010 Google, Inc.
 *
 * Author:
 *	Colin Cross <ccross@google.com>
 *	Based on arch/arm/plat-omap/cpu-omap.c, (C) 2005 Nokia Corporation
 *
 * Copyright (C) 2010-2011 NVIDIA Corporation
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/sched.h>
#include <linux/cpufreq.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/suspend.h>
#include <linux/debugfs.h>
#include <linux/cpu.h>

#include <asm/system.h>

#include <mach/hardware.h>
#include <mach/clk.h>

#include "clock.h"
#include "pm.h"

/* tegra throttling and edp governors require frequencies in the table
   to be in ascending order */
static struct cpufreq_frequency_table *freq_table;


static struct clk *cpu_clk;
static struct clk *emc_clk;

static unsigned long target_cpu_speed[CONFIG_NR_CPUS];
static DEFINE_MUTEX(tegra_cpu_lock);
static bool is_suspended;
static int suspend_index;

unsigned int tegra_getspeed(unsigned int cpu);
static int tegra_update_cpu_speed(unsigned long rate);

#ifdef CONFIG_TEGRA_THERMAL_THROTTLE
/* CPU frequency is gradually lowered when throttling is enabled */
#define THROTTLE_DELAY		msecs_to_jiffies(2000)

static bool is_throttling;
static int throttle_lowest_index;
static int throttle_highest_index;
static int throttle_index;
static int throttle_next_index;
static struct delayed_work throttle_work;
static struct workqueue_struct *workqueue;

#define tegra_cpu_is_throttling() (is_throttling)

static void tegra_throttle_work_func(struct work_struct *work)
{
	unsigned int current_freq;

	mutex_lock(&tegra_cpu_lock);
	current_freq = tegra_getspeed(0);
	throttle_index = throttle_next_index;

	if (freq_table[throttle_index].frequency < current_freq)
		tegra_update_cpu_speed(freq_table[throttle_index].frequency);

	if (throttle_index > throttle_lowest_index) {
		throttle_next_index = throttle_index - 1;
		queue_delayed_work(workqueue, &throttle_work, THROTTLE_DELAY);
	}

	mutex_unlock(&tegra_cpu_lock);
}

/*
 * tegra_throttling_enable
 * This function may sleep
 */
void tegra_throttling_enable(bool enable)
{
	mutex_lock(&tegra_cpu_lock);

	if (enable && !is_throttling) {
		unsigned int current_freq = tegra_getspeed(0);

		is_throttling = true;

		for (throttle_index = throttle_highest_index;
		     throttle_index >= throttle_lowest_index;
		     throttle_index--)
			if (freq_table[throttle_index].frequency
			    < current_freq)
				break;

		throttle_index = max(throttle_index, throttle_lowest_index);
		throttle_next_index = throttle_index;
		queue_delayed_work(workqueue, &throttle_work, 0);
	} else if (!enable && is_throttling) {
		cancel_delayed_work_sync(&throttle_work);
		is_throttling = false;
		/* restore speed requested by governor */
		tegra_cpu_cap_highest_speed(NULL);
	}

	mutex_unlock(&tegra_cpu_lock);
}
EXPORT_SYMBOL_GPL(tegra_throttling_enable);

static unsigned int throttle_governor_speed(unsigned int requested_speed)
{
	return tegra_cpu_is_throttling() ?
		min(requested_speed, freq_table[throttle_index].frequency) :
		requested_speed;
}

static ssize_t show_throttle(struct cpufreq_policy *policy, char *buf)
{
	return sprintf(buf, "%u\n", is_throttling);
}

cpufreq_freq_attr_ro(throttle);

#ifdef CONFIG_DEBUG_FS
static int throttle_debug_set(void *data, u64 val)
{
	tegra_throttling_enable(val);
	return 0;
}
static int throttle_debug_get(void *data, u64 *val)
{
	*val = (u64) is_throttling;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(throttle_fops, throttle_debug_get, throttle_debug_set, "%llu\n");

static struct dentry *cpu_tegra_debugfs_root;

static int __init tegra_cpu_debug_init(void)
{
	cpu_tegra_debugfs_root = debugfs_create_dir("cpu-tegra", 0);

	if (!cpu_tegra_debugfs_root)
		return -ENOMEM;

	if (!debugfs_create_file("throttle", 0644, cpu_tegra_debugfs_root, NULL, &throttle_fops))
		goto err_out;

	return 0;

err_out:
	debugfs_remove_recursive(cpu_tegra_debugfs_root);
	return -ENOMEM;

}

static void __exit tegra_cpu_debug_exit(void)
{
	debugfs_remove_recursive(cpu_tegra_debugfs_root);
}

late_initcall(tegra_cpu_debug_init);
module_exit(tegra_cpu_debug_exit);
#endif /* CONFIG_DEBUG_FS */

#else /* CONFIG_TEGRA_THERMAL_THROTTLE */
#define tegra_cpu_is_throttling() (0)
#define throttle_governor_speed(requested_speed) (requested_speed)

void tegra_throttling_enable(bool enable)
{
}
#endif /* CONFIG_TEGRA_THERMAL_THROTTLE */

#ifdef CONFIG_TEGRA_EDP_LIMITS

static const struct tegra_edp_limits *cpu_edp_limits;
static int cpu_edp_limits_size;
static int edp_thermal_index;
static cpumask_t edp_cpumask;
static unsigned int edp_limit;

static void edp_update_limit(void)
{
	int i;
	unsigned int limit;

	if (!cpu_edp_limits)
		return;

	limit = cpu_edp_limits[edp_thermal_index].freq_limits[
			cpumask_weight(&edp_cpumask) - 1];

	for (i = 0; freq_table[i].frequency != CPUFREQ_TABLE_END; i++) {
		if (freq_table[i].frequency > limit) {
			break;
		}
	}
	BUG_ON(i == 0);	/* min freq above the limit or table empty */
	edp_limit = freq_table[i-1].frequency;
}

static unsigned int edp_governor_speed(unsigned int requested_speed)
{
	if ((!cpu_edp_limits) || (requested_speed <= edp_limit))
		return requested_speed;
	else
		return edp_limit;
}

static int tegra_cpu_edp_notify(
	struct notifier_block *nb, unsigned long event, void *hcpu)
{
	int ret = 0;
	unsigned int cpu_speed, new_speed;
	int cpu = (long)hcpu;

	switch (event) {
	case CPU_UP_PREPARE:
		mutex_lock(&tegra_cpu_lock);
		cpu_set(cpu, edp_cpumask);
		edp_update_limit();

		cpu_speed = tegra_getspeed(0);
		new_speed = edp_governor_speed(cpu_speed);
		if (new_speed < cpu_speed) {
			ret = tegra_update_cpu_speed(new_speed);
			if (ret) {
				cpu_clear(cpu, edp_cpumask);
				edp_update_limit();
			}
			printk(KERN_DEBUG "tegra CPU:%sforce EDP limit %u kHz"
				"\n", ret ? " failed to " : " ", new_speed);
		}
		mutex_unlock(&tegra_cpu_lock);
		break;
	case CPU_DEAD:
		mutex_lock(&tegra_cpu_lock);
		cpu_clear(cpu, edp_cpumask);
		edp_update_limit();
		mutex_unlock(&tegra_cpu_lock);
		break;
	}
	return notifier_from_errno(ret);
}

static struct notifier_block tegra_cpu_edp_notifier = {
	.notifier_call = tegra_cpu_edp_notify,
};

static void tegra_cpu_edp_init(bool resume)
{
	if (!cpu_edp_limits) {
		if (!resume)
			pr_info("tegra CPU: no EDP table is provided\n");
		return;
	}

	edp_thermal_index = 0;
	edp_cpumask = *cpu_online_mask;
	edp_update_limit();

	if (!resume)
		register_hotcpu_notifier(&tegra_cpu_edp_notifier);

	pr_info("tegra CPU: set EDP limit %u MHz\n", edp_limit / 1000);
}

static void tegra_cpu_edp_exit(void)
{
	if (!cpu_edp_limits)
		return;

	unregister_hotcpu_notifier(&tegra_cpu_edp_notifier);
}

void tegra_init_cpu_edp_limits(const struct tegra_edp_limits *limits, int size)
{
	cpu_edp_limits = limits;
	cpu_edp_limits_size = cpu_edp_limits_size;
}

#else	/* CONFIG_TEGRA_EDP_LIMITS */

#define edp_governor_speed(requested_speed) (requested_speed)
#define tegra_cpu_edp_init(resume)
#define tegra_cpu_edp_exit()
#endif	/* CONFIG_TEGRA_EDP_LIMITS */

int tegra_verify_speed(struct cpufreq_policy *policy)
{
	return cpufreq_frequency_table_verify(policy, freq_table);
}

unsigned int tegra_getspeed(unsigned int cpu)
{
	unsigned long rate;

	if (cpu >= CONFIG_NR_CPUS)
		return 0;

	rate = clk_get_rate(cpu_clk) / 1000;
	return rate;
}

static int tegra_update_cpu_speed(unsigned long rate)
{
	int ret = 0;
	struct cpufreq_freqs freqs;

	freqs.old = tegra_getspeed(0);
	freqs.new = rate;

	rate = clk_round_rate(cpu_clk, rate * 1000);
	if (!IS_ERR_VALUE(rate))
		freqs.new = rate / 1000;

	if (freqs.old == freqs.new)
		return ret;

	/*
	 * Vote on memory bus frequency based on cpu frequency
	 * This sets the minimum frequency, display or avp may request higher
	 */
	clk_set_rate(emc_clk, tegra_emc_to_cpu_ratio(freqs.new));

	for_each_online_cpu(freqs.cpu)
		cpufreq_notify_transition(&freqs, CPUFREQ_PRECHANGE);

#ifdef CONFIG_CPU_FREQ_DEBUG
	printk(KERN_DEBUG "cpufreq-tegra: transition: %u --> %u\n",
	       freqs.old, freqs.new);
#endif

	ret = clk_set_rate(cpu_clk, freqs.new * 1000);
	if (ret) {
		pr_err("cpu-tegra: Failed to set cpu frequency to %d kHz\n",
			freqs.new);
		return ret;
	}

	for_each_online_cpu(freqs.cpu)
		cpufreq_notify_transition(&freqs, CPUFREQ_POSTCHANGE);

	return 0;
}

unsigned int tegra_count_slow_cpus(unsigned long speed_limit)
{
	unsigned int cnt = 0;
	int i;

	for_each_online_cpu(i)
		if (target_cpu_speed[i] <= speed_limit)
			cnt++;
	return cnt;
}

unsigned int tegra_get_slowest_cpu_n(void) {
	unsigned int cpu = nr_cpu_ids;
	unsigned long rate = ULONG_MAX;
	int i;

	for_each_online_cpu(i)
		if ((i > 0) && (rate > target_cpu_speed[i])) {
			cpu = i;
			rate = target_cpu_speed[i];
		}
	return cpu;
}

unsigned long tegra_cpu_lowest_speed(void) {
	unsigned long rate = ULONG_MAX;
	int i;

	for_each_online_cpu(i)
		rate = min(rate, target_cpu_speed[i]);
	return rate;
}

unsigned long tegra_cpu_highest_speed(void) {
	unsigned long rate = 0;
	int i;

	for_each_online_cpu(i)
		rate = max(rate, target_cpu_speed[i]);
	return rate;
}

int tegra_cpu_cap_highest_speed(unsigned int *speed_cap)
{
	unsigned int new_speed = tegra_cpu_highest_speed();

	new_speed = throttle_governor_speed(new_speed);
	new_speed = edp_governor_speed(new_speed);
	if (speed_cap)
		*speed_cap = new_speed;
	return tegra_update_cpu_speed(new_speed);
}

static int tegra_target(struct cpufreq_policy *policy,
		       unsigned int target_freq,
		       unsigned int relation)
{
	int idx;
	unsigned int freq;
	unsigned int new_speed;
	int ret = 0;

	mutex_lock(&tegra_cpu_lock);

	if (is_suspended) {
		ret = -EBUSY;
		goto out;
	}

	cpufreq_frequency_table_target(policy, freq_table, target_freq,
		relation, &idx);

	freq = freq_table[idx].frequency;

	target_cpu_speed[policy->cpu] = freq;
	ret = tegra_cpu_cap_highest_speed(&new_speed);
	if (ret == 0)
		tegra_auto_hotplug_governor(new_speed);
out:
	mutex_unlock(&tegra_cpu_lock);

	return ret;
}


static int tegra_pm_notify(struct notifier_block *nb, unsigned long event,
	void *dummy)
{
	mutex_lock(&tegra_cpu_lock);
	if (event == PM_SUSPEND_PREPARE) {
		is_suspended = true;
		pr_info("Tegra cpufreq suspend: setting frequency to %d kHz\n",
			freq_table[suspend_index].frequency);
		tegra_update_cpu_speed(freq_table[suspend_index].frequency);
	} else if (event == PM_POST_SUSPEND) {
		is_suspended = false;
		tegra_cpu_edp_init(true);
	}
	mutex_unlock(&tegra_cpu_lock);

	return NOTIFY_OK;
}

static struct notifier_block tegra_cpu_pm_notifier = {
	.notifier_call = tegra_pm_notify,
};

static int tegra_cpu_init(struct cpufreq_policy *policy)
{
	if (policy->cpu >= CONFIG_NR_CPUS)
		return -EINVAL;

	cpu_clk = clk_get_sys(NULL, "cpu");
	if (IS_ERR(cpu_clk))
		return PTR_ERR(cpu_clk);

	emc_clk = clk_get_sys("cpu", "emc");
	if (IS_ERR(emc_clk)) {
		clk_put(cpu_clk);
		return PTR_ERR(emc_clk);
	}

	clk_enable(emc_clk);
	clk_enable(cpu_clk);

	cpufreq_frequency_table_cpuinfo(policy, freq_table);
	cpufreq_frequency_table_get_attr(freq_table, policy->cpu);
	policy->cur = tegra_getspeed(policy->cpu);
	target_cpu_speed[policy->cpu] = policy->cur;

	/* FIXME: what's the actual transition time? */
	policy->cpuinfo.transition_latency = 300 * 1000;

	policy->shared_type = CPUFREQ_SHARED_TYPE_ALL;
	cpumask_copy(policy->related_cpus, cpu_possible_mask);

	if (policy->cpu == 0) {
		register_pm_notifier(&tegra_cpu_pm_notifier);
	}

	return 0;
}

static int tegra_cpu_exit(struct cpufreq_policy *policy)
{
	cpufreq_frequency_table_cpuinfo(policy, freq_table);
	clk_disable(emc_clk);
	clk_put(emc_clk);
	clk_put(cpu_clk);
	return 0;
}

static struct freq_attr *tegra_cpufreq_attr[] = {
	&cpufreq_freq_attr_scaling_available_freqs,
#ifdef CONFIG_TEGRA_THERMAL_THROTTLE
	&throttle,
#endif
	NULL,
};

static struct cpufreq_driver tegra_cpufreq_driver = {
	.verify		= tegra_verify_speed,
	.target		= tegra_target,
	.get		= tegra_getspeed,
	.init		= tegra_cpu_init,
	.exit		= tegra_cpu_exit,
	.name		= "tegra",
	.attr		= tegra_cpufreq_attr,
};

static int __init tegra_cpufreq_init(void)
{
	int ret = 0;

	struct tegra_cpufreq_table_data *table_data =
		tegra_cpufreq_table_get();
	BUG_ON(!table_data);
	suspend_index = table_data->suspend_index;

#ifdef CONFIG_TEGRA_THERMAL_THROTTLE
	/*
	 * High-priority, others flags default: not bound to a specific
	 * CPU, has rescue worker task (in case of allocation deadlock,
	 * etc.).  Single-threaded.
	 */
	workqueue = alloc_workqueue("cpu-tegra",
				    WQ_HIGHPRI | WQ_UNBOUND | WQ_RESCUER, 1);
	if (!workqueue)
		return -ENOMEM;
	INIT_DELAYED_WORK(&throttle_work, tegra_throttle_work_func);

	throttle_lowest_index = table_data->throttle_lowest_index;
	throttle_highest_index = table_data->throttle_highest_index;
#endif
	ret = tegra_auto_hotplug_init(&tegra_cpu_lock);
	if (ret)
		return ret;

	freq_table = table_data->freq_table;
	tegra_cpu_edp_init(false);
	return cpufreq_register_driver(&tegra_cpufreq_driver);
}

static void __exit tegra_cpufreq_exit(void)
{
#ifdef CONFIG_TEGRA_THERMAL_THROTTLE
	destroy_workqueue(workqueue);
#endif
	tegra_cpu_edp_exit();
	tegra_auto_hotplug_exit();
        cpufreq_unregister_driver(&tegra_cpufreq_driver);
}


MODULE_AUTHOR("Colin Cross <ccross@android.com>");
MODULE_DESCRIPTION("cpufreq driver for Nvidia Tegra2");
MODULE_LICENSE("GPL");
module_init(tegra_cpufreq_init);
module_exit(tegra_cpufreq_exit);
