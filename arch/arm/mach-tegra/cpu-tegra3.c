/*
 * arch/arm/mach-tegra/cpu-tegra3.c
 *
 * CPU auto-hotplug for Tegra3 CPUs
 *
 * Copyright (c) 2011, NVIDIA Corporation.
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
#include <linux/module.h>
#include <linux/types.h>
#include <linux/sched.h>
#include <linux/cpufreq.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/cpu.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>

#include "pm.h"

#define INITIAL_STATE		TEGRA_HP_DISABLED
#define IDLE_HYSTERESIS		100000
#define UP2G0_DELAY_MS		100
#define UP2Gn_DELAY_MS		1000
#define DOWN_DELAY_MS		2000

static DEFINE_MUTEX(tegra_hp_lock);

static struct workqueue_struct *hotplug_wq;
static struct delayed_work hotplug_work;

static unsigned long up2gn_delay;
static unsigned long up2g0_delay;
static unsigned long down_delay;
module_param(up2gn_delay, ulong, 0644);
module_param(up2g0_delay, ulong, 0644);
module_param(down_delay, ulong, 0644);

static unsigned int idle_top_freq;
static unsigned int idle_bottom_freq;
module_param(idle_top_freq, uint, 0644);
module_param(idle_bottom_freq, uint, 0644);

static unsigned int lpcpu_max_freq;

static struct {
	cputime64_t time_up_total;
	u64 last_update;
	unsigned int up_down_count;
} hp_stats[CONFIG_NR_CPUS + 1];	/* Append LP CPU entry at the end */

enum {
	TEGRA_HP_DISABLED = 0,
	TEGRA_HP_IDLE,
	TEGRA_HP_DOWN,
	TEGRA_HP_UP,
};
static int hp_state;

static int hp_state_set(const char *arg, const struct kernel_param *kp)
{
	int ret = 0;
	int old_state;

	mutex_lock(&tegra_hp_lock);

	old_state = hp_state;
	ret = param_set_int(arg, kp);

	if (ret == 0) {
		switch (hp_state) {
		case TEGRA_HP_DISABLED:
			if (old_state != TEGRA_HP_DISABLED)
				pr_info("Tegra auto-hotplug disabled\n");
			break;
		case TEGRA_HP_IDLE:
		case TEGRA_HP_DOWN:
		case TEGRA_HP_UP:
			if (old_state == TEGRA_HP_DISABLED) {
				queue_delayed_work(
					hotplug_wq, &hotplug_work, down_delay);
				pr_info("Tegra auto-hotplug enabled\n");
			}
			break;
		default:
			pr_warn("%s: unable to set tegra hotplug state %d\n",
				__func__, hp_state);
			hp_state = old_state;
		}
	}
	mutex_unlock(&tegra_hp_lock);
	return ret;
}

static int hp_state_get(char *buffer, const struct kernel_param *kp)
{
	return param_get_int(buffer, kp);
}

static struct kernel_param_ops tegra_hp_state_ops = {
	.set = hp_state_set,
	.get = hp_state_get,
};
module_param_cb(auto_hotplug, &tegra_hp_state_ops, &hp_state, 0644);


static void hp_stats_update(unsigned int cpu, bool up)
{
	u64 cur_jiffies = get_jiffies_64();
	bool was_up = hp_stats[cpu].up_down_count & 0x1;

	if (was_up)
		hp_stats[cpu].time_up_total = cputime64_add(
			hp_stats[cpu].time_up_total, cputime64_sub(
				cur_jiffies, hp_stats[cpu].last_update));

	if (was_up != up) {
		hp_stats[cpu].up_down_count++;
		if ((hp_stats[cpu].up_down_count & 0x1) != up) {
			/* FIXME: sysfs user space CPU control breaks stats */
			pr_err("tegra hotplug stats out of sync with %s CPU%d",
			       (cpu < CONFIG_NR_CPUS) ? "G" : "LP",
			       (cpu < CONFIG_NR_CPUS) ?  cpu : 0);
			hp_stats[cpu].up_down_count ^=  0x1;
		}
	}
	hp_stats[cpu].last_update = cur_jiffies;
}

static void tegra_auto_hotplug_work_func(struct work_struct *work)
{
	bool up = false;
	unsigned int cpu = nr_cpu_ids;

	mutex_lock(&tegra_hp_lock);

	switch (hp_state) {
	case TEGRA_HP_DISABLED:
	case TEGRA_HP_IDLE:
		break;
	case TEGRA_HP_DOWN:
		cpu = cpumask_next(0, cpu_online_mask);
		if (cpu < nr_cpu_ids) {
			up = false;
			queue_delayed_work(
				hotplug_wq, &hotplug_work, down_delay);
			hp_stats_update(cpu, false);
		} else if (!is_lp_cluster()) {
			tegra_cluster_control(0, TEGRA_POWER_CLUSTER_LP |
						 TEGRA_POWER_CLUSTER_IMMEDIATE);
			hp_stats_update(CONFIG_NR_CPUS, true);
			hp_stats_update(0, false);
		}
		break;
	case TEGRA_HP_UP:
		if (is_lp_cluster()) {
			tegra_cluster_control(0, TEGRA_POWER_CLUSTER_G |
						 TEGRA_POWER_CLUSTER_IMMEDIATE);
			hp_stats_update(CONFIG_NR_CPUS, false);
			hp_stats_update(0, true);
			queue_delayed_work(
				hotplug_wq, &hotplug_work, up2gn_delay);
		} else {
			cpu = cpumask_next_zero(0, cpu_online_mask);
			if (cpu < nr_cpu_ids) {
				up = true;
				queue_delayed_work(
					hotplug_wq, &hotplug_work, up2gn_delay);
				hp_stats_update(cpu, true);
			}
		}
		break;
	default:
		pr_err("%s: invalid tegra hotplug state %d\n",
		       __func__, hp_state);
	}
	mutex_unlock(&tegra_hp_lock);

	if (cpu < nr_cpu_ids) {
		if (up)
			cpu_up(cpu);
		else
			cpu_down(cpu);
	}
}

void tegra_auto_hotplug_governor(unsigned int cpu_freq)
{
	unsigned long up_delay;

	if (!is_g_cluster_present())
		return;

	mutex_lock(&tegra_hp_lock);
	up_delay = is_lp_cluster() ? up2g0_delay : up2gn_delay;

	switch (hp_state) {
	case TEGRA_HP_DISABLED:
		break;
	case TEGRA_HP_IDLE:
		if (cpu_freq > idle_top_freq) {
			hp_state = TEGRA_HP_UP;
			queue_delayed_work(
				hotplug_wq, &hotplug_work, up_delay);
		} else if (cpu_freq <= idle_bottom_freq) {
			hp_state = TEGRA_HP_DOWN;
			queue_delayed_work(
				hotplug_wq, &hotplug_work, down_delay);
		}
		break;
	case TEGRA_HP_DOWN:
		if (cpu_freq > idle_top_freq) {
			hp_state = TEGRA_HP_UP;
			queue_delayed_work(
				hotplug_wq, &hotplug_work, up_delay);
		} else if (cpu_freq > idle_bottom_freq) {
			hp_state = TEGRA_HP_IDLE;
		}
		break;
	case TEGRA_HP_UP:
		if (cpu_freq <= idle_bottom_freq) {
			hp_state = TEGRA_HP_DOWN;
			queue_delayed_work(
				hotplug_wq, &hotplug_work, down_delay);
		} else if (cpu_freq <= idle_top_freq) {
			hp_state = TEGRA_HP_IDLE;
		}
		break;
	default:
		pr_err("%s: invalid tegra hotplug state %d\n",
		       __func__, hp_state);
		BUG();
	}
	mutex_unlock(&tegra_hp_lock);
}

int tegra_auto_hotplug_init(void)
{
	int i;
	u64 cur_jiffies = get_jiffies_64();

	/*
	 * Not bound to the issuer CPU (=> high-priority), has rescue worker
	 * task, single-threaded, frrezeable.
	 */
	hotplug_wq = alloc_workqueue(
		"cpu-tegra3", WQ_UNBOUND | WQ_RESCUER | WQ_FREEZEABLE, 1);
	if (!hotplug_wq)
		return -ENOMEM;
	INIT_DELAYED_WORK(&hotplug_work, tegra_auto_hotplug_work_func);

	lpcpu_max_freq = tegra_get_lpcpu_max_rate() / 1000;
	idle_top_freq = lpcpu_max_freq;
	idle_bottom_freq = idle_top_freq - IDLE_HYSTERESIS;

	up2g0_delay = msecs_to_jiffies(UP2G0_DELAY_MS);
	up2gn_delay = msecs_to_jiffies(UP2Gn_DELAY_MS);
	down_delay = msecs_to_jiffies(DOWN_DELAY_MS);

	hp_state = INITIAL_STATE;
	pr_info("Tegra auto-hotplug initialized: %s\n",
		(hp_state == TEGRA_HP_DISABLED) ? "disabled" : "enabled");

	for (i = 0; i <= CONFIG_NR_CPUS; i++) {
		hp_stats[i].time_up_total = 0;
		hp_stats[i].last_update = cur_jiffies;

		hp_stats[i].up_down_count = 0;
		if (is_lp_cluster()) {
			if (i == CONFIG_NR_CPUS)
				hp_stats[i].up_down_count = 1;
		} else {
			if ((i < nr_cpu_ids) && cpu_online(i))
				hp_stats[i].up_down_count = 1;
		}
	}

	return 0;
}

#ifdef CONFIG_DEBUG_FS

static struct dentry *hp_debugfs_root;

static int hp_stats_show(struct seq_file *s, void *data)
{
	int i;
	u64 cur_jiffies = get_jiffies_64();

	mutex_lock(&tegra_hp_lock);
	for (i = 0; i <= CONFIG_NR_CPUS; i++) {
		bool was_up = (hp_stats[i].up_down_count & 0x1);
		hp_stats_update(i, was_up);
	}
	mutex_unlock(&tegra_hp_lock);

	seq_printf(s, "%-15s ", "cpu:");
	for (i = 0; i < CONFIG_NR_CPUS; i++) {
		seq_printf(s, "G%-9d ", i);
	}
	seq_printf(s, "LP\n");

	seq_printf(s, "%-15s ", "transitions:");
	for (i = 0; i <= CONFIG_NR_CPUS; i++) {
		seq_printf(s, "%-10u ", hp_stats[i].up_down_count);
	}
	seq_printf(s, "\n");

	seq_printf(s, "%-15s ", "time plugged:");
	for (i = 0; i <= CONFIG_NR_CPUS; i++) {
		seq_printf(s, "%-10llu ",
			   cputime64_to_clock_t(hp_stats[i].time_up_total));
	}
	seq_printf(s, "\n");

	seq_printf(s, "%-15s %llu\n", "time-stamp:",
		   cputime64_to_clock_t(cur_jiffies));

	return 0;
}

static int hp_stats_open(struct inode *inode, struct file *file)
{
	return single_open(file, hp_stats_show, inode->i_private);
}

static const struct file_operations hp_stats_fops = {
	.open		= hp_stats_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int __init tegra_auto_hotplug_debug_init(void)
{
	hp_debugfs_root = debugfs_create_dir("tegra_hotplug", NULL);
	if (!hp_debugfs_root)
		return -ENOMEM;

	if (!debugfs_create_file(
		"stats", S_IRUGO, hp_debugfs_root, NULL, &hp_stats_fops))
		goto err_out;

	return 0;

err_out:
	debugfs_remove_recursive(hp_debugfs_root);
	return -ENOMEM;
}

late_initcall(tegra_auto_hotplug_debug_init);
#endif

void tegra_auto_hotplug_exit(void)
{
	destroy_workqueue(hotplug_wq);
#ifdef CONFIG_DEBUG_FS
	debugfs_remove_recursive(hp_debugfs_root);
#endif
}
