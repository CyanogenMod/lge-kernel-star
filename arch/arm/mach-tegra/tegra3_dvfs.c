/*
 * arch/arm/mach-tegra/tegra3_dvfs.c
 *
 * Copyright (C) 2010-2011 NVIDIA Corporation.
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
#include <linux/init.h>
#include <linux/string.h>
#include <linux/module.h>

#include "clock.h"
#include "dvfs.h"
#include "fuse.h"

static bool tegra_dvfs_cpu_disabled = false;

static const int cpu_millivolts[MAX_DVFS_FREQS] =
	{750, 775, 800, 825, 850, 875, 900, 925, 950, 975, 1000, 1025, 1050, 1075, 1100, 1125};

static const int core_millivolts[MAX_DVFS_FREQS] =
	{950, 1000, 1100, 1200};

#define KHZ 1000
#define MHZ 1000000

static struct dvfs_rail tegra3_dvfs_rail_vdd_cpu = {
	.reg_id = "vdd_cpu",
	.max_millivolts = 1125,
	.min_millivolts = 800,
	.nominal_millivolts = 1000,
};

static struct dvfs_rail tegra3_dvfs_rail_vdd_core = {
	.reg_id = "vdd_core",
	.max_millivolts = 1300,
	.min_millivolts = 950,
	.nominal_millivolts = 1200,
	.step = 100, /* FIXME: step vdd_core by 100 mV - maybe not needed */
	.disabled = true, /* FIXME: replace with sysfs control */
};

static struct dvfs_rail *tegra3_dvfs_rails[] = {
	&tegra3_dvfs_rail_vdd_cpu,
	&tegra3_dvfs_rail_vdd_core,
};

#define CPU_DVFS(_clk_name, _speedo_id, _process_id, _mult, _freqs...)	\
	{								\
		.clk_name	= _clk_name,				\
		.speedo_id	= _speedo_id,				\
		.process_id	= _process_id,				\
		.freqs		= {_freqs},				\
		.freqs_mult	= _mult,				\
		.millivolts	= cpu_millivolts,			\
		.auto_dvfs	= true,					\
		.dvfs_rail	= &tegra3_dvfs_rail_vdd_cpu,		\
	}

static struct dvfs cpu_dvfs_table[] = {
	/* Cpu voltages (mV):	     750, 775, 800, 825, 850, 875, 900, 925, 950, 975, 1000, 1025, 1050, 1075, 1100, 1125 */
	CPU_DVFS("cpu_g", 0, 0, MHZ,   0,   0, 614, 614, 714, 714, 815, 815, 915, 915, 1000),
};

#define CORE_DVFS(_clk_name, _process_id, _auto, _mult, _freqs...)	\
	{							\
		.clk_name	= _clk_name,			\
		.speedo_id	= -1,				\
		.process_id	= _process_id,				\
		.freqs		= {_freqs},			\
		.freqs_mult	= _mult,			\
		.millivolts	= core_millivolts,		\
		.auto_dvfs	= _auto,			\
		.dvfs_rail	= &tegra3_dvfs_rail_vdd_core,	\
	}

static struct dvfs core_dvfs_table[] = {
	/* Cpu voltages (mV):	            950,   1000,   1100,   1200 */
	CORE_DVFS("cpu_lp", 0, 1, KHZ,   313500, 361000, 456000, 456000)
};

int tegra_dvfs_disable_cpu_set(const char *arg, const struct kernel_param *kp)
{
	int ret;

	ret = param_set_bool(arg, kp);
	if (ret)
		return ret;

	if (tegra_dvfs_cpu_disabled)
		tegra_dvfs_rail_disable(&tegra3_dvfs_rail_vdd_cpu);
	else
		tegra_dvfs_rail_enable(&tegra3_dvfs_rail_vdd_cpu);

	return 0;
}

int tegra_dvfs_disable_get(char *buffer, const struct kernel_param *kp)
{
	return param_get_bool(buffer, kp);
}

static struct kernel_param_ops tegra_dvfs_disable_cpu_ops = {
	.set = tegra_dvfs_disable_cpu_set,
	.get = tegra_dvfs_disable_get,
};

module_param_cb(disable_cpu, &tegra_dvfs_disable_cpu_ops, &tegra_dvfs_cpu_disabled, 0644);

static void init_dvfs_from_table(struct dvfs *dvfs_table, int table_size,
				 int speedo_id, int process_id)
{
	int i, ret;
	struct clk *c;
	struct dvfs *d;

	for (i = 0; i < table_size; i++) {
		d = &dvfs_table[i];

		if ((d->process_id != -1 && d->process_id != process_id) ||
			(d->speedo_id != -1 && d->speedo_id != speedo_id)) {
			pr_debug("tegra3_dvfs: rejected %s speedo %d,"
				" process %d\n", d->clk_name, d->speedo_id,
				d->process_id);
			continue;
		}

		c = tegra_get_clock_by_name(d->clk_name);

		if (!c) {
			pr_debug("tegra3_dvfs: no clock found for %s\n",
				d->clk_name);
			continue;
		}

		ret = tegra_enable_dvfs_on_clk(c, d);
		if (ret)
			pr_err("tegra3_dvfs: failed to enable dvfs on %s\n",
				c->name);
	}
}

void __init tegra_soc_init_dvfs(void)
{
	int speedo_id = 0; 		/* FIXME: get real speedo ID */
	int cpu_process_id = 0;		/* FIXME: get real CPU process ID */
	int core_process_id = 0;	/* FIXME: get real core process ID */

#ifndef CONFIG_TEGRA_CPU_DVFS
	tegra_dvfs_cpu_disabled = true;
#endif

	tegra_dvfs_init_rails(tegra3_dvfs_rails, ARRAY_SIZE(tegra3_dvfs_rails));

	/* FIXME: add [CPU/CORE/AON] relationships here */

	init_dvfs_from_table(cpu_dvfs_table, ARRAY_SIZE(cpu_dvfs_table),
			     speedo_id, cpu_process_id);
	init_dvfs_from_table(core_dvfs_table, ARRAY_SIZE(core_dvfs_table),
			     speedo_id, core_process_id);

	if (tegra_dvfs_cpu_disabled)
		tegra_dvfs_rail_disable(&tegra3_dvfs_rail_vdd_cpu);
}
