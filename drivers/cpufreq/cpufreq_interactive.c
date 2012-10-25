/*
 * drivers/cpufreq/cpufreq_interactive.c
 *
 * Copyright (C) 2010 Google, Inc.
 * Copyright (C) 2012 NVIDIA Corporation.
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
 * Author: Mike Chan (mike@android.com)
 *
 */

#include <linux/cpu.h>
#include <linux/cpumask.h>
#include <linux/cpufreq.h>
#include <linux/mutex.h>
#include <linux/sched.h>
#ifdef CONFIG_DYNAMIC_FREQ_MODE
#include <linux/slab.h>
#endif
#include <linux/tick.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/kthread.h>

#include <asm/cputime.h>

static void (*pm_idle_old)(void);
static atomic_t active_count = ATOMIC_INIT(0);

struct cpufreq_interactive_cpuinfo {
	struct timer_list cpu_timer;
	int timer_idlecancel;
	u64 time_in_idle;
	u64 time_in_iowait;
	u64 idle_exit_time;
	u64 timer_run_time;
	int idling;
	u64 freq_change_time;
	u64 freq_change_time_in_idle;
	u64 freq_change_time_in_iowait;
	struct cpufreq_policy *policy;
	struct cpufreq_frequency_table *freq_table;
	unsigned int target_freq;
	int governor_enabled;
#ifdef CONFIG_DYNAMIC_FREQ_MODE
	unsigned long boosted;
#endif
};

static DEFINE_PER_CPU(struct cpufreq_interactive_cpuinfo, cpuinfo);

/* Workqueues handle frequency scaling */
static struct task_struct *up_task;
static struct workqueue_struct *down_wq;
static struct work_struct freq_scale_down_work;
static cpumask_t up_cpumask;
static spinlock_t up_cpumask_lock;
static cpumask_t down_cpumask;
static spinlock_t down_cpumask_lock;

/* Go to max speed when CPU load at or above this value. */
#define DEFAULT_GO_MAXSPEED_LOAD 85
static unsigned long go_maxspeed_load;

/* Base of exponential raise to max speed; if 0 - jump to maximum */
static unsigned long boost_factor;

/* Max frequency boost in Hz; if 0 - no max is enforced */
static unsigned long max_boost;

/* Consider IO as busy */
static unsigned long io_is_busy;

/*
 * Targeted sustainable load relatively to current frequency.
 * If 0, target is set realtively to the max speed
 */
static unsigned long sustain_load;

/*
 * The minimum amount of time to spend at a frequency before we can ramp down.
 */
#define DEFAULT_MIN_SAMPLE_TIME 30000;
static unsigned long min_sample_time;

#ifdef CONFIG_DYNAMIC_FREQ_MODE
/*
 * Parameters for dynamic frequency mode
 */
struct dynamic_freq_param {
	unsigned long downsustain_load;
};

struct cpufreq_interactive_dynamic_freq {
	unsigned long go_maxspeed_load;
	unsigned long go_upspeed_load;
	unsigned long sustain_load;
	unsigned long go_downspeed_load;
	unsigned long boost_high_boundary;
	unsigned long boost_low_boundary;
	unsigned long boost_step;
	unsigned long buck_step;

	struct dynamic_freq_param *params;
	struct cpufreq_frequency_table *freq_table;
	unsigned int freq_num;

	unsigned long enabled;
};

#define DYNAMIC_FREQ_GO_MAXSPEED_LOAD		95
#define DYNAMIC_FREQ_GO_UPSPEED_LOAD		85
#define DYNAMIC_FREQ_SUSTAIN_LOAD		80
#define DYNAMIC_FREQ_GO_DOWNSPEED_LOAD		70
#define DYNAMIC_FREQ_BOOST_HIGH_BOUNDARY	700000
#define DYNAMIC_FREQ_BOOST_LOW_BOUNDARY		300000
#define DYNAMIC_FREQ_BOOST_STEP			2
#define DYNAMIC_FREQ_BUCK_STEP			5

static struct cpufreq_interactive_dynamic_freq dynamic_freq = {
	.go_maxspeed_load = DYNAMIC_FREQ_GO_MAXSPEED_LOAD,
	.go_upspeed_load = DYNAMIC_FREQ_GO_UPSPEED_LOAD,
	.sustain_load = DYNAMIC_FREQ_SUSTAIN_LOAD,
	.go_downspeed_load = DYNAMIC_FREQ_GO_DOWNSPEED_LOAD,
	.boost_high_boundary = DYNAMIC_FREQ_BOOST_HIGH_BOUNDARY,
	.boost_low_boundary = DYNAMIC_FREQ_BOOST_LOW_BOUNDARY,
	.boost_step = DYNAMIC_FREQ_BOOST_STEP,
	.buck_step = DYNAMIC_FREQ_BUCK_STEP,
	.enabled = 0,
};
#endif /* CONFIG_DYNAMIC_FREQ_MODE */

#define DEBUG 0
#define BUFSZ 128

#if DEBUG
#include <linux/proc_fs.h>

struct dbgln {
	int cpu;
	unsigned long jiffy;
	unsigned long run;
	char buf[BUFSZ];
};

#define NDBGLNS 256

static struct dbgln dbgbuf[NDBGLNS];
static int dbgbufs;
static int dbgbufe;
static struct proc_dir_entry	*dbg_proc;
static spinlock_t dbgpr_lock;

static u64 up_request_time;
static unsigned int up_max_latency;

static void dbgpr(char *fmt, ...)
{
	va_list args;
	int n;
	unsigned long flags;

	spin_lock_irqsave(&dbgpr_lock, flags);
	n = dbgbufe;
        va_start(args, fmt);
        vsnprintf(dbgbuf[n].buf, BUFSZ, fmt, args);
        va_end(args);
	dbgbuf[n].cpu = smp_processor_id();
	dbgbuf[n].run = nr_running();
	dbgbuf[n].jiffy = jiffies;

	if (++dbgbufe >= NDBGLNS)
		dbgbufe = 0;

	if (dbgbufe == dbgbufs)
		if (++dbgbufs >= NDBGLNS)
			dbgbufs = 0;

	spin_unlock_irqrestore(&dbgpr_lock, flags);
}

static void dbgdump(void)
{
	int i, j;
	unsigned long flags;
	static struct dbgln prbuf[NDBGLNS];

	spin_lock_irqsave(&dbgpr_lock, flags);
	i = dbgbufs;
	j = dbgbufe;
	memcpy(prbuf, dbgbuf, sizeof(dbgbuf));
	dbgbufs = 0;
	dbgbufe = 0;
	spin_unlock_irqrestore(&dbgpr_lock, flags);

	while (i != j)
	{
		printk("%lu %d %lu %s",
		       prbuf[i].jiffy, prbuf[i].cpu, prbuf[i].run,
		       prbuf[i].buf);
		if (++i == NDBGLNS)
			i = 0;
	}
}

static int dbg_proc_read(char *buffer, char **start, off_t offset,
			       int count, int *peof, void *dat)
{
	printk("max up_task latency=%uus\n", up_max_latency);
	dbgdump();
	*peof = 1;
	return 0;
}


#else
#define dbgpr(...) do {} while (0)
#endif

static int cpufreq_governor_interactive(struct cpufreq_policy *policy,
		unsigned int event);

#ifndef CONFIG_CPU_FREQ_DEFAULT_GOV_INTERACTIVE
static
#endif
struct cpufreq_governor cpufreq_gov_interactive = {
	.name = "interactive",
	.governor = cpufreq_governor_interactive,
	.max_transition_latency = 10000000,
	.owner = THIS_MODULE,
};

#ifdef CONFIG_DYNAMIC_FREQ_MODE
#define GET_UP_FREQ(_freq_table, _freq_num, _cur_idx, _up_step)	\
		(((int)_cur_idx + (int)_up_step) < _freq_num ?	\
		_freq_table[_cur_idx + _up_step].frequency :	\
		_freq_table[_freq_num - 1].frequency)

#define GET_DOWN_FREQ(_freq_table, _cur_idx, _down_step)	\
		(((int)_cur_idx - (int)_down_step) > 0 ?	\
		_freq_table[_cur_idx - _down_step].frequency :	\
		_freq_table[0].frequency)

static int _set_param_go_downspeed_load(int idx)
{
	unsigned int freq = dynamic_freq.freq_table[idx].frequency;
	unsigned int freq_1dn = GET_DOWN_FREQ(dynamic_freq.freq_table, idx, 1);
	int downspeed_load = dynamic_freq.go_downspeed_load;
	unsigned long dn_load;

calc_downspeed_load:
	for (dn_load = 1; dn_load <= 100; dn_load++) {
		if ((freq_1dn * dn_load / freq) >= downspeed_load)
			break;
	}

	if (dn_load >= 100) {
		if (--downspeed_load > 0)
			goto calc_downspeed_load;
		else
			downspeed_load = 100;
	}

	dynamic_freq.params[idx].downsustain_load = dn_load;
	return 0;
}

static int cpufreq_interactive_dynamic_freq_alloc(void)
{
	int i = 0;

	dynamic_freq.freq_table = cpufreq_frequency_get_table(0);
	while (dynamic_freq.freq_table[i].frequency != CPUFREQ_TABLE_END)
		i++;
	dynamic_freq.freq_num = i;

	dynamic_freq.params = kzalloc(sizeof(struct dynamic_freq_param) * i,
				      GFP_KERNEL);
	if (!dynamic_freq.params) {
		pr_err("%s: Failed to alloc memory for params\n", __func__);
		return -ENOMEM;
	}
	return 0;
}

static void cpufreq_interactive_dynamic_freq_free(void)
{
	kfree(dynamic_freq.params);
}

static int cpufreq_interactive_dynamic_freq_init(unsigned long enable)
{
	int i;

	if (!enable)
		goto out;

	for (i = 0; i < dynamic_freq.freq_num; i++) {
		if (dynamic_freq.freq_table[i].frequency
				== CPUFREQ_ENTRY_INVALID)
			continue;
		_set_param_go_downspeed_load(i);
	}

out:
	dynamic_freq.enabled = enable;
	return 0;
}

static unsigned int cpufreq_interactive_dynamic_freq_get_target(
	int cpu_load, int load_since_change, struct cpufreq_policy *policy)
{
	struct cpufreq_interactive_cpuinfo *pcpu =
			&per_cpu(cpuinfo, policy->cpu);
	struct cpufreq_frequency_table *freq_table = dynamic_freq.freq_table;
	unsigned int freq_num = dynamic_freq.freq_num;
	unsigned long low_boundary = dynamic_freq.boost_low_boundary;
	unsigned long high_boundary = dynamic_freq.boost_high_boundary;
	unsigned long downsustain_load = 1;
	unsigned int target_freq;
	int i, freq_idx = 0;

	/*
	 * Choose greater of short-term load (since last idle timer
	 * started or timer function re-armed itself) or long-term load
	 * (since last frequency change).
	 */
	if (load_since_change > cpu_load)
		cpu_load = load_since_change;

	for (i = 0; i < freq_num; i++) {
		if (policy->cur == freq_table[i].frequency) {
			downsustain_load =
				dynamic_freq.params[i].downsustain_load;
			freq_idx = i;
			break;
		} else if (policy->cur > freq_table[i].frequency) {
			downsustain_load =
				dynamic_freq.params[i].downsustain_load;
			freq_idx = i;
		}
	}

	if (cpu_load >= dynamic_freq.go_maxspeed_load) {
		if ((!pcpu->boosted && policy->cur <= high_boundary) ||
				(policy->cur <= low_boundary))
			target_freq = GET_UP_FREQ(freq_table, freq_num,
					freq_idx, dynamic_freq.boost_step);
		else
			target_freq = GET_UP_FREQ(freq_table, freq_num,
					freq_idx, 1);

		if (policy->cur > low_boundary)
			pcpu->boosted++;
	} else {
		if (cpu_load >= dynamic_freq.go_upspeed_load) {
			target_freq = GET_UP_FREQ(freq_table, freq_num,
					freq_idx, 1);
		} else if (cpu_load >= dynamic_freq.sustain_load) {
			target_freq = policy->cur;
		} else if (cpu_load >= dynamic_freq.go_downspeed_load) {
			target_freq = GET_DOWN_FREQ(freq_table, freq_idx, 1);
		} else {
			unsigned int buck_freq = GET_DOWN_FREQ(freq_table,
					freq_idx, dynamic_freq.buck_step);
			target_freq = policy->cur * cpu_load / downsustain_load;
			if (target_freq < buck_freq)
				target_freq = buck_freq;
		}
		pcpu->boosted = 0;
	}

	target_freq = min(target_freq, policy->max);
	return target_freq;
}
#endif /* CONFIG_DYNAMIC_FREQ_MODE */

static unsigned int cpufreq_interactive_get_target(
	int cpu_load, int load_since_change, struct cpufreq_policy *policy)
{
	unsigned int target_freq;

	/*
	 * Choose greater of short-term load (since last idle timer
	 * started or timer function re-armed itself) or long-term load
	 * (since last frequency change).
	 */
	if (load_since_change > cpu_load)
		cpu_load = load_since_change;

	if (cpu_load >= go_maxspeed_load) {
		if (!boost_factor)
			return policy->max;

		target_freq = policy->cur * boost_factor;

		if (max_boost && target_freq > policy->cur + max_boost)
			target_freq = policy->cur + max_boost;
	}
	else {
		if (!sustain_load)
			return policy->max * cpu_load / 100;

		target_freq = policy->cur * cpu_load / sustain_load;
	}

	target_freq = min(target_freq, policy->max);
	return target_freq;
}

static inline cputime64_t get_cpu_iowait_time(
	unsigned int cpu, cputime64_t *wall)
{
	u64 iowait_time = get_cpu_iowait_time_us(cpu, wall);

	if (iowait_time == -1ULL)
		return 0;

	return iowait_time;
}

static void cpufreq_interactive_timer(unsigned long data)
{
	unsigned int delta_idle;
	unsigned int delta_iowait;
	unsigned int delta_time;
	int cpu_load;
	int load_since_change;
	u64 time_in_idle;
	u64 time_in_iowait;
	u64 idle_exit_time;
	struct cpufreq_interactive_cpuinfo *pcpu =
		&per_cpu(cpuinfo, data);
	u64 now_idle;
	u64 now_iowait;
	unsigned int new_freq;
	unsigned int index;
	unsigned long flags;

	smp_rmb();

	if (!pcpu->governor_enabled)
		goto exit;

	/*
	 * Once pcpu->timer_run_time is updated to >= pcpu->idle_exit_time,
	 * this lets idle exit know the current idle time sample has
	 * been processed, and idle exit can generate a new sample and
	 * re-arm the timer.  This prevents a concurrent idle
	 * exit on that CPU from writing a new set of info at the same time
	 * the timer function runs (the timer function can't use that info
	 * until more time passes).
	 */
	time_in_idle = pcpu->time_in_idle;
	time_in_iowait = pcpu->time_in_iowait;
	idle_exit_time = pcpu->idle_exit_time;
	now_idle = get_cpu_idle_time_us(data, &pcpu->timer_run_time);
	now_iowait = get_cpu_iowait_time(data, NULL);
	smp_wmb();

	/* If we raced with cancelling a timer, skip. */
	if (!idle_exit_time) {
		dbgpr("timer %d: no valid idle exit sample\n", (int) data);
		goto exit;
	}

#if DEBUG
	if ((int) jiffies - (int) pcpu->cpu_timer.expires >= 10)
		dbgpr("timer %d: late by %d ticks\n",
		      (int) data, jiffies - pcpu->cpu_timer.expires);
#endif

	delta_idle = (unsigned int) cputime64_sub(now_idle, time_in_idle);
	delta_iowait = (unsigned int) cputime64_sub(now_iowait, time_in_iowait);
	delta_time = (unsigned int) cputime64_sub(pcpu->timer_run_time,
						  idle_exit_time);

	/*
	 * If timer ran less than 1ms after short-term sample started, retry.
	 */
	if (delta_time < 1000) {
		dbgpr("timer %d: time delta %u too short exit=%llu now=%llu\n", (int) data,
		      delta_time, idle_exit_time, pcpu->timer_run_time);
		goto rearm;
	}

	if (delta_idle > delta_time)
		cpu_load = 0;
	else {
		if (io_is_busy && delta_idle >= delta_iowait)
			delta_idle -= delta_iowait;

		cpu_load = 100 * (delta_time - delta_idle) / delta_time;
	}

	delta_idle = (unsigned int) cputime64_sub(now_idle,
						 pcpu->freq_change_time_in_idle);
	delta_iowait = (unsigned int) cputime64_sub(now_iowait,
					pcpu->freq_change_time_in_iowait);
	delta_time = (unsigned int) cputime64_sub(pcpu->timer_run_time,
						  pcpu->freq_change_time);

	if ((delta_time == 0) || (delta_idle > delta_time))
		load_since_change = 0;
	else {
		if (io_is_busy && delta_idle >= delta_iowait)
			delta_idle -= delta_iowait;

		load_since_change =
			100 * (delta_time - delta_idle) / delta_time;
	}

	/*
	 * Combine short-term load (since last idle timer started or timer
	 * function re-armed itself) and long-term load (since last frequency
	 * change) to determine new target frequency
	 */
#ifdef CONFIG_DYNAMIC_FREQ_MODE
	if (dynamic_freq.enabled)
		new_freq = cpufreq_interactive_dynamic_freq_get_target(cpu_load,
					load_since_change, pcpu->policy);
	else
#endif
	new_freq = cpufreq_interactive_get_target(cpu_load, load_since_change,
						  pcpu->policy);

	if (cpufreq_frequency_table_target(pcpu->policy, pcpu->freq_table,
					   new_freq, CPUFREQ_RELATION_H,
					   &index)) {
		dbgpr("timer %d: cpufreq_frequency_table_target error\n", (int) data);
		goto rearm;
	}

	new_freq = pcpu->freq_table[index].frequency;

	if (pcpu->target_freq == new_freq)
	{
		dbgpr("timer %d: load=%d, already at %d\n", (int) data, cpu_load, new_freq);
		goto rearm_if_notmax;
	}

	/*
	 * Do not scale down unless we have been at this frequency for the
	 * minimum sample time.
	 */
	if (new_freq < pcpu->target_freq) {
		if (cputime64_sub(pcpu->timer_run_time, pcpu->freq_change_time) <
		    min_sample_time) {
			dbgpr("timer %d: load=%d cur=%d tgt=%d not yet\n", (int) data, cpu_load, pcpu->target_freq, new_freq);
			goto rearm;
		}
	}

	dbgpr("timer %d: load=%d cur=%d tgt=%d queue\n", (int) data, cpu_load, pcpu->target_freq, new_freq);

	if (new_freq < pcpu->target_freq) {
		pcpu->target_freq = new_freq;
		spin_lock_irqsave(&down_cpumask_lock, flags);
		cpumask_set_cpu(data, &down_cpumask);
		spin_unlock_irqrestore(&down_cpumask_lock, flags);
		queue_work(down_wq, &freq_scale_down_work);
	} else {
		pcpu->target_freq = new_freq;
#if DEBUG
		up_request_time = ktime_to_us(ktime_get());
#endif
		spin_lock_irqsave(&up_cpumask_lock, flags);
		cpumask_set_cpu(data, &up_cpumask);
		spin_unlock_irqrestore(&up_cpumask_lock, flags);
		wake_up_process(up_task);
	}

rearm_if_notmax:
	/*
	 * Already set max speed and don't see a need to change that,
	 * wait until next idle to re-evaluate, don't need timer.
	 */
	if (pcpu->target_freq == pcpu->policy->max)
		goto exit;

rearm:
	if (!timer_pending(&pcpu->cpu_timer)) {
		/*
		 * If already at min: if that CPU is idle, don't set timer.
		 * Else cancel the timer if that CPU goes idle.  We don't
		 * need to re-evaluate speed until the next idle exit.
		 */
		if (pcpu->target_freq == pcpu->policy->min) {
			smp_rmb();

			if (pcpu->idling) {
				dbgpr("timer %d: cpu idle, don't re-arm\n", (int) data);
				goto exit;
			}

			pcpu->timer_idlecancel = 1;
		}

		pcpu->time_in_idle = get_cpu_idle_time_us(
			data, &pcpu->idle_exit_time);
		pcpu->time_in_iowait = get_cpu_iowait_time(
			data, NULL);
		mod_timer(&pcpu->cpu_timer, jiffies + 2);
		dbgpr("timer %d: set timer for %lu exit=%llu\n", (int) data, pcpu->cpu_timer.expires, pcpu->idle_exit_time);
	}

exit:
	return;
}

static void cpufreq_interactive_idle(void)
{
	struct cpufreq_interactive_cpuinfo *pcpu =
		&per_cpu(cpuinfo, smp_processor_id());
	int pending;

	if (!pcpu->governor_enabled) {
		pm_idle_old();
		return;
	}

	pcpu->idling = 1;
	smp_wmb();
	pending = timer_pending(&pcpu->cpu_timer);

	if (pcpu->target_freq != pcpu->policy->min) {
#ifdef CONFIG_SMP
		/*
		 * Entering idle while not at lowest speed.  On some
		 * platforms this can hold the other CPU(s) at that speed
		 * even though the CPU is idle. Set a timer to re-evaluate
		 * speed so this idle CPU doesn't hold the other CPUs above
		 * min indefinitely.  This should probably be a quirk of
		 * the CPUFreq driver.
		 */
		if (!pending) {
			pcpu->time_in_idle = get_cpu_idle_time_us(
				smp_processor_id(), &pcpu->idle_exit_time);
			pcpu->time_in_iowait = get_cpu_iowait_time(
				smp_processor_id(), NULL);
			pcpu->timer_idlecancel = 0;
			mod_timer(&pcpu->cpu_timer, jiffies + 2);
			dbgpr("idle: enter at %d, set timer for %lu exit=%llu\n",
			      pcpu->target_freq, pcpu->cpu_timer.expires,
			      pcpu->idle_exit_time);
		}
#endif
	} else {
		/*
		 * If at min speed and entering idle after load has
		 * already been evaluated, and a timer has been set just in
		 * case the CPU suddenly goes busy, cancel that timer.  The
		 * CPU didn't go busy; we'll recheck things upon idle exit.
		 */
		if (pending && pcpu->timer_idlecancel) {
			dbgpr("idle: cancel timer for %lu\n", pcpu->cpu_timer.expires);
			del_timer(&pcpu->cpu_timer);
			/*
			 * Ensure last timer run time is after current idle
			 * sample start time, so next idle exit will always
			 * start a new idle sampling period.
			 */
			pcpu->idle_exit_time = 0;
			pcpu->timer_idlecancel = 0;
		}
	}

	pm_idle_old();
	pcpu->idling = 0;
	smp_wmb();

	/*
	 * Arm the timer for 1-2 ticks later if not already, and if the timer
	 * function has already processed the previous load sampling
	 * interval.  (If the timer is not pending but has not processed
	 * the previous interval, it is probably racing with us on another
	 * CPU.  Let it compute load based on the previous sample and then
	 * re-arm the timer for another interval when it's done, rather
	 * than updating the interval start time to be "now", which doesn't
	 * give the timer function enough time to make a decision on this
	 * run.)
	 */
	if (timer_pending(&pcpu->cpu_timer) == 0 &&
	    pcpu->timer_run_time >= pcpu->idle_exit_time &&
	    pcpu->governor_enabled) {
		pcpu->time_in_idle =
			get_cpu_idle_time_us(smp_processor_id(),
					     &pcpu->idle_exit_time);
		pcpu->time_in_iowait =
			get_cpu_iowait_time(smp_processor_id(),
						NULL);
		pcpu->timer_idlecancel = 0;
		mod_timer(&pcpu->cpu_timer, jiffies + 2);
		dbgpr("idle: exit, set timer for %lu exit=%llu\n", pcpu->cpu_timer.expires, pcpu->idle_exit_time);
#if DEBUG
	} else if (timer_pending(&pcpu->cpu_timer) == 0 &&
		   pcpu->timer_run_time < pcpu->idle_exit_time) {
		dbgpr("idle: timer not run yet: exit=%llu tmrrun=%llu\n",
		      pcpu->idle_exit_time, pcpu->timer_run_time);
#endif
	}

}

static int cpufreq_interactive_up_task(void *data)
{
	unsigned int cpu;
	cpumask_t tmp_mask;
	unsigned long flags;
	struct cpufreq_interactive_cpuinfo *pcpu;

#if DEBUG
	u64 now;
	u64 then;
	unsigned int lat;
#endif

	while (1) {
		set_current_state(TASK_INTERRUPTIBLE);
		spin_lock_irqsave(&up_cpumask_lock, flags);

		if (cpumask_empty(&up_cpumask)) {
			spin_unlock_irqrestore(&up_cpumask_lock, flags);
			schedule();

			if (kthread_should_stop())
				break;

			spin_lock_irqsave(&up_cpumask_lock, flags);
		}

		set_current_state(TASK_RUNNING);

#if DEBUG
		then = up_request_time;
		now = ktime_to_us(ktime_get());

		if (now > then) {
			lat = ktime_to_us(ktime_get()) - then;

			if (lat > up_max_latency)
				up_max_latency = lat;
		}
#endif

		tmp_mask = up_cpumask;
		cpumask_clear(&up_cpumask);
		spin_unlock_irqrestore(&up_cpumask_lock, flags);

		for_each_cpu(cpu, &tmp_mask) {
			pcpu = &per_cpu(cpuinfo, cpu);

			if (nr_running() == 1) {
				dbgpr("up %d: tgt=%d nothing else running\n", cpu,
				      pcpu->target_freq);
			}

			smp_rmb();

			if (!pcpu->governor_enabled)
				continue;

			__cpufreq_driver_target(pcpu->policy,
						pcpu->target_freq,
						CPUFREQ_RELATION_H);
			pcpu->freq_change_time_in_idle =
				get_cpu_idle_time_us(cpu,
						     &pcpu->freq_change_time);
			pcpu->freq_change_time_in_iowait =
				get_cpu_iowait_time(cpu, NULL);
			dbgpr("up %d: set tgt=%d (actual=%d)\n", cpu, pcpu->target_freq, pcpu->policy->cur);
		}
	}

	return 0;
}

static void cpufreq_interactive_freq_down(struct work_struct *work)
{
	unsigned int cpu;
	cpumask_t tmp_mask;
	unsigned long flags;
	struct cpufreq_interactive_cpuinfo *pcpu;

	spin_lock_irqsave(&down_cpumask_lock, flags);
	tmp_mask = down_cpumask;
	cpumask_clear(&down_cpumask);
	spin_unlock_irqrestore(&down_cpumask_lock, flags);

	for_each_cpu(cpu, &tmp_mask) {
		pcpu = &per_cpu(cpuinfo, cpu);

		smp_rmb();

		if (!pcpu->governor_enabled)
			continue;

		__cpufreq_driver_target(pcpu->policy,
					pcpu->target_freq,
					CPUFREQ_RELATION_H);
		pcpu->freq_change_time_in_idle =
			get_cpu_idle_time_us(cpu,
					     &pcpu->freq_change_time);
		pcpu->freq_change_time_in_iowait =
			get_cpu_iowait_time(cpu, NULL);
		dbgpr("down %d: set tgt=%d (actual=%d)\n", cpu, pcpu->target_freq, pcpu->policy->cur);
	}
}

static ssize_t show_go_maxspeed_load(struct kobject *kobj,
				     struct attribute *attr, char *buf)
{
	return sprintf(buf, "%lu\n", go_maxspeed_load);
}

static ssize_t store_go_maxspeed_load(struct kobject *kobj,
			struct attribute *attr, const char *buf, size_t count)
{
	if (!strict_strtoul(buf, 0, &go_maxspeed_load))
		return count;
	return -EINVAL;
}

static struct global_attr go_maxspeed_load_attr = __ATTR(go_maxspeed_load, 0644,
		show_go_maxspeed_load, store_go_maxspeed_load);

static ssize_t show_boost_factor(struct kobject *kobj,
				     struct attribute *attr, char *buf)
{
	return sprintf(buf, "%lu\n", boost_factor);
}

static ssize_t store_boost_factor(struct kobject *kobj,
			struct attribute *attr, const char *buf, size_t count)
{
	if (!strict_strtoul(buf, 0, &boost_factor))
		return count;
	return -EINVAL;
}

static struct global_attr boost_factor_attr = __ATTR(boost_factor, 0644,
		show_boost_factor, store_boost_factor);

static ssize_t show_io_is_busy(struct kobject *kobj,
				     struct attribute *attr, char *buf)
{
	return sprintf(buf, "%lu\n", io_is_busy);
}

static ssize_t store_io_is_busy(struct kobject *kobj,
			struct attribute *attr, const char *buf, size_t count)
{
	if (!strict_strtoul(buf, 0, &io_is_busy))
		return count;
	return -EINVAL;
}

static struct global_attr io_is_busy_attr = __ATTR(io_is_busy, 0644,
		show_io_is_busy, store_io_is_busy);

static ssize_t show_max_boost(struct kobject *kobj,
				     struct attribute *attr, char *buf)
{
	return sprintf(buf, "%lu\n", max_boost);
}

static ssize_t store_max_boost(struct kobject *kobj,
			struct attribute *attr, const char *buf, size_t count)
{
	if (!strict_strtoul(buf, 0, &max_boost))
		return count;
	return -EINVAL;
}

static struct global_attr max_boost_attr = __ATTR(max_boost, 0644,
		show_max_boost, store_max_boost);


static ssize_t show_sustain_load(struct kobject *kobj,
				     struct attribute *attr, char *buf)
{
	return sprintf(buf, "%lu\n", sustain_load);
}

static ssize_t store_sustain_load(struct kobject *kobj,
			struct attribute *attr, const char *buf, size_t count)
{
	if (!strict_strtoul(buf, 0, &sustain_load))
		return count;
	return -EINVAL;
}

static struct global_attr sustain_load_attr = __ATTR(sustain_load, 0644,
		show_sustain_load, store_sustain_load);

static ssize_t show_min_sample_time(struct kobject *kobj,
				struct attribute *attr, char *buf)
{
	return sprintf(buf, "%lu\n", min_sample_time);
}

static ssize_t store_min_sample_time(struct kobject *kobj,
			struct attribute *attr, const char *buf, size_t count)
{
	if (!strict_strtoul(buf, 0, &min_sample_time))
		return count;
	return -EINVAL;
}

static struct global_attr min_sample_time_attr = __ATTR(min_sample_time, 0644,
		show_min_sample_time, store_min_sample_time);

#ifdef CONFIG_DYNAMIC_FREQ_MODE
static ssize_t show_dynamic_freq_mode(struct kobject *kobj,
			struct attribute *attr, char *buf)
{
	return sprintf(buf, "%lu\n", dynamic_freq.enabled);
}

static ssize_t store_dynamic_freq_mode(struct kobject *kobj,
			struct attribute *attr, const char *buf, size_t count)
{
	unsigned long val;
	int ret;

	ret = strict_strtoul(buf, 0, &val);
	if (ret < 0)
		return ret;

	if (val == dynamic_freq.enabled)
		return count;

	cpufreq_interactive_dynamic_freq_init(val);
	return count;
}

static struct global_attr dynamic_freq_mode_attr = __ATTR(dynamic_freq_mode,
		0644, show_dynamic_freq_mode, store_dynamic_freq_mode);
#endif /* CONFIG_DYNAMIC_FREQ_MODE */

static struct attribute *interactive_attributes[] = {
	&go_maxspeed_load_attr.attr,
	&boost_factor_attr.attr,
	&max_boost_attr.attr,
	&io_is_busy_attr.attr,
	&sustain_load_attr.attr,
	&min_sample_time_attr.attr,
#ifdef CONFIG_DYNAMIC_FREQ_MODE
	&dynamic_freq_mode_attr.attr,
#endif
	NULL,
};

static struct attribute_group interactive_attr_group = {
	.attrs = interactive_attributes,
	.name = "interactive",
};

static int cpufreq_governor_interactive(struct cpufreq_policy *new_policy,
		unsigned int event)
{
	int rc;
	struct cpufreq_interactive_cpuinfo *pcpu =
		&per_cpu(cpuinfo, new_policy->cpu);

	switch (event) {
	case CPUFREQ_GOV_START:
		if (!cpu_online(new_policy->cpu))
			return -EINVAL;

		pcpu->policy = new_policy;
		pcpu->freq_table = cpufreq_frequency_get_table(new_policy->cpu);
		pcpu->target_freq = new_policy->cur;
		pcpu->freq_change_time_in_idle =
			get_cpu_idle_time_us(new_policy->cpu,
					     &pcpu->freq_change_time);
		pcpu->time_in_idle = pcpu->freq_change_time_in_idle;
		pcpu->freq_change_time_in_iowait =
			get_cpu_iowait_time(new_policy->cpu, NULL);
		pcpu->time_in_iowait = pcpu->freq_change_time_in_iowait;
		pcpu->idle_exit_time = pcpu->freq_change_time;
		pcpu->timer_idlecancel = 1;
		pcpu->governor_enabled = 1;
		smp_wmb();

		if (!timer_pending(&pcpu->cpu_timer))
			mod_timer(&pcpu->cpu_timer, jiffies + 2);

		/*
		 * Do not register the idle hook and create sysfs
		 * entries if we have already done so.
		 */
		if (atomic_inc_return(&active_count) > 1)
			return 0;

		rc = sysfs_create_group(cpufreq_global_kobject,
				&interactive_attr_group);
		if (rc)
			return rc;

		pm_idle_old = pm_idle;
		pm_idle = cpufreq_interactive_idle;
#ifdef CONFIG_DYNAMIC_FREQ_MODE
		cpufreq_interactive_dynamic_freq_init(dynamic_freq.enabled);
#endif
		break;

	case CPUFREQ_GOV_STOP:
		pcpu->governor_enabled = 0;
		smp_wmb();
		del_timer_sync(&pcpu->cpu_timer);
		flush_work(&freq_scale_down_work);
		/*
		 * Reset idle exit time since we may cancel the timer
		 * before it can run after the last idle exit time,
		 * to avoid tripping the check in idle exit for a timer
		 * that is trying to run.
		 */
		pcpu->idle_exit_time = 0;

		if (atomic_dec_return(&active_count) > 0)
			return 0;

		sysfs_remove_group(cpufreq_global_kobject,
				&interactive_attr_group);

		pm_idle = pm_idle_old;
		break;

	case CPUFREQ_GOV_LIMITS:
		if (new_policy->max < new_policy->cur)
			__cpufreq_driver_target(new_policy,
					new_policy->max, CPUFREQ_RELATION_H);
		else if (new_policy->min > new_policy->cur)
			__cpufreq_driver_target(new_policy,
					new_policy->min, CPUFREQ_RELATION_L);
		break;
	}
	return 0;
}

static int __init cpufreq_interactive_init(void)
{
	unsigned int i;
	struct cpufreq_interactive_cpuinfo *pcpu;
	struct sched_param param = { .sched_priority = MAX_RT_PRIO-1 };
#ifdef CONFIG_DYNAMIC_FREQ_MODE
	int ret;
#endif

	go_maxspeed_load = DEFAULT_GO_MAXSPEED_LOAD;
	min_sample_time = DEFAULT_MIN_SAMPLE_TIME;

	/* Initalize per-cpu timers */
	for_each_possible_cpu(i) {
		pcpu = &per_cpu(cpuinfo, i);
		init_timer(&pcpu->cpu_timer);
		pcpu->cpu_timer.function = cpufreq_interactive_timer;
		pcpu->cpu_timer.data = i;
	}

#ifdef CONFIG_DYNAMIC_FREQ_MODE
	ret = cpufreq_interactive_dynamic_freq_alloc();
	if (ret < 0)
		return ret;
#endif

	up_task = kthread_create(cpufreq_interactive_up_task, NULL,
				 "kinteractiveup");
	if (IS_ERR(up_task)) {
#ifdef CONFIG_DYNAMIC_FREQ_MODE
		cpufreq_interactive_dynamic_freq_free();
#endif
		return PTR_ERR(up_task);
	}

	sched_setscheduler_nocheck(up_task, SCHED_FIFO, &param);
	get_task_struct(up_task);

	/* No rescuer thread, bind to CPU queuing the work for possibly
	   warm cache (probably doesn't matter much). */
	down_wq = alloc_workqueue("knteractive_down", 0, 1);

	if (! down_wq)
		goto err_freeuptask;

	INIT_WORK(&freq_scale_down_work,
		  cpufreq_interactive_freq_down);

	spin_lock_init(&up_cpumask_lock);
	spin_lock_init(&down_cpumask_lock);

#if DEBUG
	spin_lock_init(&dbgpr_lock);
	dbg_proc = create_proc_entry("igov", S_IWUSR | S_IRUGO, NULL);
	dbg_proc->read_proc = dbg_proc_read;
#endif

	return cpufreq_register_governor(&cpufreq_gov_interactive);

err_freeuptask:
	put_task_struct(up_task);
#ifdef CONFIG_DYNAMIC_FREQ_MODE
	cpufreq_interactive_dynamic_freq_free();
#endif
	return -ENOMEM;
}

#ifdef CONFIG_CPU_FREQ_DEFAULT_GOV_INTERACTIVE
fs_initcall(cpufreq_interactive_init);
#else
module_init(cpufreq_interactive_init);
#endif

static void __exit cpufreq_interactive_exit(void)
{
	cpufreq_unregister_governor(&cpufreq_gov_interactive);
	kthread_stop(up_task);
	put_task_struct(up_task);
	destroy_workqueue(down_wq);
#ifdef CONFIG_DYNAMIC_FREQ_MODE
	cpufreq_interactive_dynamic_freq_free();
#endif
}

module_exit(cpufreq_interactive_exit);

MODULE_AUTHOR("Mike Chan <mike@android.com>");
MODULE_DESCRIPTION("'cpufreq_interactive' - A cpufreq governor for "
	"Latency sensitive workloads");
MODULE_LICENSE("GPL");
