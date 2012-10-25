/*
 * arch/arm/mach-tegra/cpu-tegra2.c
 *
 * CPU auto-hotplug for Tegra2 CPUs
 *
 * Copyright (c) 2012, NVIDIA Corporation.
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
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/pm_qos_params.h>

#include "pm.h"
#include "cpu-tegra.h"
#include "clock.h"

#define INITIAL_STATE		TEGRA_HP_DISABLED
#define DELAY_MS		1

static struct mutex *tegra2_cpu_lock;

static struct workqueue_struct *hotplug_wq;
static struct delayed_work hotplug_work;

static unsigned long delay;

static unsigned long top_freq;
static unsigned long bottom_freq;

static struct clk *cpu_clk;

enum {
	TEGRA_HP_DISABLED = 0,
	TEGRA_HP_IDLE,
	TEGRA_HP_DOWN,
	TEGRA_HP_UP,
};
static int hp_state;

void tegra2_enable_autoplug(void)
{
	mutex_lock(tegra2_cpu_lock);
	hp_state = TEGRA_HP_IDLE;
	mutex_unlock(tegra2_cpu_lock);
}

void tegra2_disable_autoplug(void)
{
	mutex_lock(tegra2_cpu_lock);
	hp_state = TEGRA_HP_DISABLED;

	/* check if CPU-1 is offline before leaving from here.
	 * If it was, bring it online as it was before enabling hot-plug
	 * for Tegra2 */
	if (!cpu_online(1))
		cpu_up(1);

	mutex_unlock(tegra2_cpu_lock);
}

static void tegra2_auto_hotplug_work_func(struct work_struct *work)
{
	bool up = false;

	mutex_lock(tegra2_cpu_lock);

	switch (hp_state) {

	case TEGRA_HP_DISABLED:
		mutex_unlock(tegra2_cpu_lock);
		return;

	case TEGRA_HP_IDLE:
	case TEGRA_HP_DOWN:
		break;

	case TEGRA_HP_UP:
		up = true;
		break;

	default:
		pr_err("%s: invalid tegra hotplug state %d\n",
		       __func__, hp_state);
	}
	mutex_unlock(tegra2_cpu_lock);

	if (up)
		cpu_up(1);
	else
		cpu_down(1);
}

void tegra2_auto_hotplug_governor(unsigned int cpu_freq, bool suspend)
{
	switch (hp_state) {

	case TEGRA_HP_DISABLED:
		break;

	case TEGRA_HP_IDLE:
		if (cpu_freq > top_freq) {
			hp_state = TEGRA_HP_UP;
			queue_delayed_work(hotplug_wq, &hotplug_work, delay);
		} else if (cpu_freq <= bottom_freq) {
			hp_state = TEGRA_HP_DOWN;
			queue_delayed_work(hotplug_wq, &hotplug_work, delay);
		}
		break;

	case TEGRA_HP_DOWN:
		if (cpu_freq > top_freq) {
			hp_state = TEGRA_HP_UP;
			queue_delayed_work(hotplug_wq, &hotplug_work, delay);
		} else if (cpu_freq > bottom_freq) {
			hp_state = TEGRA_HP_IDLE;
		}
		break;

	case TEGRA_HP_UP:
		if (cpu_freq <= bottom_freq) {
			hp_state = TEGRA_HP_DOWN;
			queue_delayed_work(hotplug_wq, &hotplug_work, delay);
		} else if (cpu_freq <= top_freq) {
			hp_state = TEGRA_HP_IDLE;
		}
		break;

	default:
		pr_err("%s: invalid tegra hotplug state %d\n",
		       __func__, hp_state);
		BUG();
	}
}

int tegra2_auto_hotplug_init(struct mutex *cpu_lock)
{
	/*
	 * Not bound to the issuer CPU (=> high-priority), has rescue worker
	 * task, single-threaded, freezable.
	 */
	hotplug_wq = alloc_workqueue("cpu-tegra2",
		WQ_UNBOUND | WQ_RESCUER | WQ_FREEZABLE, 1);
	if (!hotplug_wq)
		return -ENOMEM;

	INIT_DELAYED_WORK(&hotplug_work, tegra2_auto_hotplug_work_func);

	cpu_clk = clk_get_sys(NULL, "cpu");
	if (IS_ERR(cpu_clk))
		return -ENOENT;

	/* top frequency = 80% of max CPU frequency */
	top_freq = clk_get_max_rate(cpu_clk)/1000;
	top_freq = (top_freq * 8)/10;

	/* bottom frequency = 250 MHz */
	bottom_freq = 250000;

	delay = msecs_to_jiffies(DELAY_MS);

	tegra2_cpu_lock = cpu_lock;
	hp_state = INITIAL_STATE;
	pr_info("Tegra auto-hotplug initialized: %s\n",
		(hp_state == TEGRA_HP_DISABLED) ? "disabled" : "enabled");

	return 0;
}

void tegra2_auto_hotplug_exit(void)
{
	destroy_workqueue(hotplug_wq);
}
