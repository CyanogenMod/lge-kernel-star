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

#include "pm.h"

#define INITIAL_STATE		TEGRA_HP_DISABLED
#define IDLE_HYSTERESIS		100000
#define UP_DELAY_MS		1000
#define DOWN_DELAY_MS		2000

static DEFINE_MUTEX(tegra_hp_lock);

static struct workqueue_struct *hotplug_wq;
static struct delayed_work hotplug_work;

static unsigned long up_delay;
static unsigned long down_delay;
module_param(up_delay, ulong, 0644);
module_param(down_delay, ulong, 0644);

static unsigned int idle_top_freq;
static unsigned int idle_bottom_freq;
module_param(idle_top_freq, uint, 0644);
module_param(idle_bottom_freq, uint, 0644);

static unsigned int lpcpu_max_freq;

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
					hotplug_wq, &hotplug_work, up_delay);
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


static void tegra_auto_hotplug_work_func(struct work_struct *work)
{
	bool up = false;
	unsigned int cpu = nr_cpu_ids;

	mutex_lock(&tegra_hp_lock);

	if (is_lp_cluster()) {
		mutex_unlock(&tegra_hp_lock);
		return;
	}

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
		}
		else
			tegra_cluster_control(0, TEGRA_POWER_CLUSTER_LP |
					         TEGRA_POWER_CLUSTER_IMMEDIATE);
		break;
	case TEGRA_HP_UP:
		cpu = cpumask_next_zero(0, cpu_online_mask);
		if (cpu < nr_cpu_ids) {
			up = true;
			queue_delayed_work(
				hotplug_wq, &hotplug_work, up_delay);
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
	mutex_lock(&tegra_hp_lock);

	if (is_lp_cluster() && (cpu_freq > lpcpu_max_freq)) {
		tegra_cluster_control(0, TEGRA_POWER_CLUSTER_G |
					 TEGRA_POWER_CLUSTER_IMMEDIATE);
	}

	switch (hp_state) {
	case TEGRA_HP_DISABLED:
		break;
	case TEGRA_HP_IDLE:
		if (cpu_freq > idle_top_freq) {
			hp_state = TEGRA_HP_UP;
			queue_delayed_work(
				hotplug_wq, &hotplug_work, up_delay);
		}
		else if (cpu_freq <= idle_bottom_freq) {
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
		}
		else if (cpu_freq > idle_bottom_freq) {
			hp_state = TEGRA_HP_IDLE;
		}
		break;
	case TEGRA_HP_UP:
		if (cpu_freq <= idle_bottom_freq) {
			hp_state = TEGRA_HP_DOWN;
			queue_delayed_work(
				hotplug_wq, &hotplug_work, down_delay);
		}
		else if (cpu_freq <= idle_top_freq) {
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

	up_delay = msecs_to_jiffies(UP_DELAY_MS);
	down_delay = msecs_to_jiffies(DOWN_DELAY_MS);

	hp_state = INITIAL_STATE;
	pr_info("Tegra auto-hotplug initialized: %s\n",
		(hp_state == TEGRA_HP_DISABLED) ? "disabled" : "enabled");

	return 0;
}

void tegra_auto_hotplug_exit(void)
{
	destroy_workqueue(hotplug_wq);
}
