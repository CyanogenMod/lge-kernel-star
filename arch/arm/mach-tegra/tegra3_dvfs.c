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
#include "board.h"

static bool tegra_dvfs_cpu_disabled;
static bool tegra_dvfs_core_disabled;

static const int cpu_millivolts[MAX_DVFS_FREQS] =
	{750, 775, 800, 825, 850, 875, 900, 925, 950, 975, 1000, 1025, 1050, 1075, 1100, 1125};

static const int core_millivolts[MAX_DVFS_FREQS] =
	{1000, 1050, 1100, 1150, 1200, 1250, 1300};

static const int core_speedo_nominal_millivolts[] =
/* speedo_id 0,    1,    2 */
	{ 1200, 1200, 1300 };

static const int cpu_speedo_nominal_millivolts[] =
/* speedo_id 0,    1,    2 */
	{ 1125, 1125, 1125 };

#define KHZ 1000
#define MHZ 1000000

#define VDD_CPU_BELOW_VDD_CORE_MAX	300

static struct dvfs_rail tegra3_dvfs_rail_vdd_cpu = {
	.reg_id = "vdd_cpu",
	.max_millivolts = 1125,
	.min_millivolts = 800,
	.step = VDD_CPU_BELOW_VDD_CORE_MAX,
};

static struct dvfs_rail tegra3_dvfs_rail_vdd_core = {
	.reg_id = "vdd_core",
	.max_millivolts = 1300,
	.min_millivolts = 950,
	.step = VDD_CPU_BELOW_VDD_CORE_MAX,
};

static struct dvfs_rail *tegra3_dvfs_rails[] = {
	&tegra3_dvfs_rail_vdd_cpu,
	&tegra3_dvfs_rail_vdd_core,
};


/* vdd_core must not be lower than vdd_cpu */
static int tegra3_dvfs_rel_vdd_cpu_vdd_core(struct dvfs_rail *vdd_cpu,
	struct dvfs_rail *vdd_core)
{
	int core_floor = max(vdd_cpu->new_millivolts, vdd_cpu->millivolts);
	return max(vdd_core->new_millivolts, core_floor);
}

/* vdd_cpu must be within VDD_CPU_BELOW_VDD_CORE_MAX below vdd_core */
static int tegra3_dvfs_rel_vdd_core_vdd_cpu(struct dvfs_rail *vdd_core,
	struct dvfs_rail *vdd_cpu)
{
	int cpu_floor = max(vdd_core->new_millivolts, vdd_core->millivolts) -
		VDD_CPU_BELOW_VDD_CORE_MAX;
	return max(vdd_cpu->new_millivolts, cpu_floor);
}

static struct dvfs_relationship tegra3_dvfs_relationships[] = {
	{
		.from = &tegra3_dvfs_rail_vdd_cpu,
		.to = &tegra3_dvfs_rail_vdd_core,
		.solve = tegra3_dvfs_rel_vdd_cpu_vdd_core,
		.solved_at_nominal = true,
	},
	{
		.from = &tegra3_dvfs_rail_vdd_core,
		.to = &tegra3_dvfs_rail_vdd_cpu,
		.solve = tegra3_dvfs_rel_vdd_core_vdd_cpu,
	},
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
	/* Cpu voltages (mV):	     750, 775, 800, 825, 850, 875,  900,  925,  950,  975, 1000, 1025, 1050, 1075, 1100, 1125 */
	CPU_DVFS("cpu_g", 0, 0, MHZ, 399, 399, 541, 541, 684, 684,  817,  817,  817, 1026, 1102, 1149, 1187, 1225, 1282, 1300),
	CPU_DVFS("cpu_g", 0, 1, MHZ, 481, 481, 652, 652, 807, 807,  948,  948,  948, 1117, 1171, 1206, 1300),
	CPU_DVFS("cpu_g", 0, 2, MHZ, 540, 540, 711, 711, 883, 883, 1039, 1039, 1039, 1178, 1206, 1300),
	CPU_DVFS("cpu_g", 0, 3, MHZ, 570, 570, 777, 777, 931, 931, 1102, 1102, 1102, 1216, 1300),

	/*
	 * "Safe entry" to be used when no match for chip speedo, process
	 *  corner is found (just to boot at low rate); must be the last one
	 */
	CPU_DVFS("cpu_g",-1,-1, MHZ,   1,   1, 216, 216, 300),
};

#define CORE_DVFS(_clk_name, _speedo_id, _auto, _mult, _freqs...)	\
	{							\
		.clk_name	= _clk_name,			\
		.speedo_id	= _speedo_id,			\
		.process_id	= -1,				\
		.freqs		= {_freqs},			\
		.freqs_mult	= _mult,			\
		.millivolts	= core_millivolts,		\
		.auto_dvfs	= _auto,			\
		.dvfs_rail	= &tegra3_dvfs_rail_vdd_core,	\
	}

static struct dvfs core_dvfs_table[] = {
	/* Core voltages (mV):	           1000,   1050,   1100,   1150,    1200,    1250,    1300 */
	CORE_DVFS("cpu_lp", 0, 1, KHZ,   294500, 342000, 427000, 484000,  500000,  500000,  500000),

	CORE_DVFS("emc",    0, 1, KHZ,   266500, 266500, 266500, 266500,  533000,  533000,  533000),

	CORE_DVFS("sbus",   0, 1, KHZ,   136000, 163000, 191000, 216000,  216000,  216000,  216000),

	CORE_DVFS("vde",    0, 1, KHZ,   216000, 270000, 332000, 380000,  416000,  416000,  416000),
	CORE_DVFS("mpe",    0, 1, KHZ,   234000, 285000, 332000, 380000,  416000,  416000,  416000),
	CORE_DVFS("2d",     0, 1, KHZ,   267000, 285000, 332000, 380000,  416000,  416000,  416000),
	CORE_DVFS("epp",    0, 1, KHZ,   267000, 285000, 332000, 380000,  416000,  416000,  416000),
	CORE_DVFS("3d",     0, 1, KHZ,   234000, 285000, 332000, 380000,  416000,  416000,  416000),
	CORE_DVFS("3d2",    0, 1, KHZ,   234000, 285000, 332000, 380000,  416000,  416000,  416000),
	CORE_DVFS("host1x", 0, 1, KHZ,   152000, 188000, 222000, 254000,  267000,  267000,  267000),
	CORE_DVFS("vi",     0, 1, KHZ,   216000, 285000, 300000, 300000,  300000,  300000,  300000),
	CORE_DVFS("se",     0, 1, KHZ,   267000, 285000, 332000, 380000,  416000,  416000,  416000),

	CORE_DVFS("pll_c",  0, 1, KHZ,   466000, 564000, 664000, 760000,  832000, 1066000, 1066000),
	CORE_DVFS("pll_m",  0, 1, KHZ,   600000, 600000, 800000, 800000, 1066000, 1066000, 1066000),

	CORE_DVFS("mipi",   0, 1, KHZ,        1,      1,      1,      1,      1,       1,       1),
	CORE_DVFS("sdmmc1",-1, 1, KHZ,   104000, 104000, 104000, 104000, 208000,  208000,  208000),
	CORE_DVFS("sdmmc3",-1, 1, KHZ,   104000, 104000, 104000, 104000, 208000,  208000,  208000),
	CORE_DVFS("ndflash",-1,1, KHZ,   110000, 166000, 166000, 166000, 166000,  166000,  166000),
	CORE_DVFS("nor",   -1, 1, KHZ,   100000, 126000, 126000, 133000, 133000,  133000,  133000),
	CORE_DVFS("sbc1",  -1, 1, KHZ,    40000,  60000,  60000,  60000, 100000,  100000,  100000),
	CORE_DVFS("sbc2",  -1, 1, KHZ,    40000,  60000,  60000,  60000, 100000,  100000,  100000),
	CORE_DVFS("sbc3",  -1, 1, KHZ,    40000,  60000,  60000,  60000, 100000,  100000,  100000),
	CORE_DVFS("sbc4",  -1, 1, KHZ,    40000,  60000,  60000,  60000, 100000,  100000,  100000),
	CORE_DVFS("sbc5",  -1, 1, KHZ,    40000,  60000,  60000,  60000, 100000,  100000,  100000),
	CORE_DVFS("sbc6",  -1, 1, KHZ,    40000,  60000,  60000,  60000, 100000,  100000,  100000),
	CORE_DVFS("tvo",   -1, 1, KHZ,        1, 297000, 297000, 297000, 297000,  297000,  297000),
	CORE_DVFS("dsi",   -1, 1, KHZ,   430000, 430000, 430000, 430000, 500000,  500000,  500000),

	/*
	 * The clock rate for the display controllers that determines the
	 * necessary core voltage depends on a divider that is internal
	 * to the display block.  Disable auto-dvfs on the display clocks,
	 * and let the display driver call tegra_dvfs_set_rate manually
	 */
	CORE_DVFS("disp1", -1, 0, KHZ,   120000, 120000, 120000, 120000, 190000,  190000,  190000),
	CORE_DVFS("disp2", -1, 0, KHZ,   120000, 120000, 120000, 120000, 190000,  190000,  190000),
};


int tegra_dvfs_disable_core_set(const char *arg, const struct kernel_param *kp)
{
	int ret;

	ret = param_set_bool(arg, kp);
	if (ret)
		return ret;

	if (tegra_dvfs_core_disabled)
		tegra_dvfs_rail_disable(&tegra3_dvfs_rail_vdd_core);
	else
		tegra_dvfs_rail_enable(&tegra3_dvfs_rail_vdd_core);

	return 0;
}

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

static struct kernel_param_ops tegra_dvfs_disable_core_ops = {
	.set = tegra_dvfs_disable_core_set,
	.get = tegra_dvfs_disable_get,
};

static struct kernel_param_ops tegra_dvfs_disable_cpu_ops = {
	.set = tegra_dvfs_disable_cpu_set,
	.get = tegra_dvfs_disable_get,
};

module_param_cb(disable_core, &tegra_dvfs_disable_core_ops,
	&tegra_dvfs_core_disabled, 0644);
module_param_cb(disable_cpu, &tegra_dvfs_disable_cpu_ops,
	&tegra_dvfs_cpu_disabled, 0644);

static void __init init_dvfs_one(struct dvfs *d, int nominal_mv_index)
{
	int ret;
	struct clk *c = tegra_get_clock_by_name(d->clk_name);

	if (!c) {
		pr_debug("tegra3_dvfs: no clock found for %s\n",
			d->clk_name);
		return;
	}

	if (d->auto_dvfs) {
		/* Update max rate for auto-dvfs clocks */
		BUG_ON(!d->freqs[nominal_mv_index]);
		tegra_init_max_rate(
			c, d->freqs[nominal_mv_index] * d->freqs_mult);
	}
	d->max_millivolts = d->dvfs_rail->nominal_millivolts;

	ret = tegra_enable_dvfs_on_clk(c, d);
	if (ret)
		pr_err("tegra3_dvfs: failed to enable dvfs on %s\n",
			c->name);
}

static bool __init match_dvfs_one(struct dvfs *d, int speedo_id, int process_id)
{
	if ((d->process_id != -1 && d->process_id != process_id) ||
		(d->speedo_id != -1 && d->speedo_id != speedo_id)) {
		pr_debug("tegra3_dvfs: rejected %s speedo %d,"
			" process %d\n", d->clk_name, d->speedo_id,
			d->process_id);
		return false;
	}
	return true;
}

static int __init get_cpu_nominal_mv_index(
	int speedo_id, int process_id, struct dvfs **cpu_dvfs)
{
	int i, j, mv;
	struct dvfs *d;
	struct clk *c;

	/*
	 * Start with nominal level for the chips with this speedo_id. Then,
	 * make sure cpu nominal voltage is below core ("solve from cpu to
	 * core at nominal").
	 */
	BUG_ON(speedo_id >= ARRAY_SIZE(cpu_speedo_nominal_millivolts));
	mv = cpu_speedo_nominal_millivolts[speedo_id];
	if (tegra3_dvfs_rail_vdd_core.nominal_millivolts)
		mv = min(mv, tegra3_dvfs_rail_vdd_core.nominal_millivolts);

	/*
	 * Find matching cpu dvfs entry, and use it to determine index to the
	 * final nominal voltage, that satisfies the following requirements:
	 * - allows CPU to run at minimum of the maximum rates specified in
	 *   the dvfs entry and clock tree
	 * - does not violate cpu_to_core dependency as determined above
	 */
	for (i = 0, j = 0; j <  ARRAY_SIZE(cpu_dvfs_table); j++) {
		d = &cpu_dvfs_table[j];
		if (match_dvfs_one(d, speedo_id, process_id)) {
			c = tegra_get_clock_by_name(d->clk_name);
			BUG_ON(!c);

			for (; i < MAX_DVFS_FREQS; i++) {
				if ((d->freqs[i] == 0) ||
				    (cpu_millivolts[i] == 0) ||
				    (mv < cpu_millivolts[i]))
					break;

				if (c->max_rate <= d->freqs[i]*d->freqs_mult) {
					i++;
					break;
				}
			}
			break;
		}
	}

	BUG_ON(i == 0);
	if (j == (ARRAY_SIZE(cpu_dvfs_table) - 1))
		pr_err("tegra3_dvfs: WARNING!!!\n"
		       "tegra3_dvfs: no cpu dvfs table found for chip speedo_id"
		       " %d and process_id %d: set CPU rate limit at %lu\n"
		       "tegra3_dvfs: WARNING!!!\n",
		       speedo_id, process_id, d->freqs[i-1] * d->freqs_mult);

	*cpu_dvfs = d;
	return (i - 1);
}

static int __init get_core_nominal_mv_index(int speedo_id)
{
	int i, mv;
	int core_edp_limit = get_core_edp();

	/*
	 * Start with nominal level for the chips with this speedo_id. Then,
	 * make sure core nominal voltage is below edp limit for the board
	 * (if edp limit is set).
	 */
	BUG_ON(speedo_id >= ARRAY_SIZE(core_speedo_nominal_millivolts));
	mv = core_speedo_nominal_millivolts[speedo_id];

	if (core_edp_limit)
		mv = min(mv, core_edp_limit);

	/* Round nominal level down to the nearest core scaling step */
	for (i = 0; i < MAX_DVFS_FREQS; i++) {
		if ((core_millivolts[i] == 0) || (mv < core_millivolts[i]))
			break;
	}

	if (i == 0) {
		pr_err("tegra3_dvfs: unable to adjust core dvfs table to"
		       " nominal voltage %d\n", mv);
		return -ENOSYS;
	}
	return (i - 1);
}

void __init tegra_soc_init_dvfs(void)
{
	int speedo_id = 0; 		/* FIXME: get real speedo ID */
	int cpu_process_id = 0;		/* FIXME: get real CPU process ID */
	int core_process_id = 0;	/* FIXME: get real core process ID */

	int i;
	int core_nominal_mv_index;
	int cpu_nominal_mv_index;
	struct dvfs *cpu_dvfs = NULL;

#ifndef CONFIG_TEGRA_CORE_DVFS
	tegra_dvfs_core_disabled = true;
#endif
#ifndef CONFIG_TEGRA_CPU_DVFS
	tegra_dvfs_cpu_disabled = true;
#endif
	/*
	 * Find nominal voltages for core (1st) and cpu rails before rail
	 * init. Nominal voltage index in the scaling ladder will also be
	 * used to determine max dvfs frequency for the respective domains.
	 */
	core_nominal_mv_index = get_core_nominal_mv_index(speedo_id);
	if (core_nominal_mv_index < 0) {
		tegra3_dvfs_rail_vdd_core.disabled = true;
		tegra_dvfs_core_disabled = true;
		core_nominal_mv_index = 0;
	}
	tegra3_dvfs_rail_vdd_core.nominal_millivolts =
		core_millivolts[core_nominal_mv_index];

	cpu_nominal_mv_index = get_cpu_nominal_mv_index(
		speedo_id, cpu_process_id, &cpu_dvfs);
	BUG_ON((cpu_nominal_mv_index < 0) || (!cpu_dvfs));
	tegra3_dvfs_rail_vdd_cpu.nominal_millivolts =
		cpu_millivolts[cpu_nominal_mv_index];

	/* Init rail structures and dependencies */
	tegra_dvfs_init_rails(tegra3_dvfs_rails, ARRAY_SIZE(tegra3_dvfs_rails));
	tegra_dvfs_add_relationships(tegra3_dvfs_relationships,
		ARRAY_SIZE(tegra3_dvfs_relationships));

	/* Search core dvfs table for speedo/process matching entries and
	   initialize dvfs-ed clocks */
	for (i = 0; i <  ARRAY_SIZE(core_dvfs_table); i++) {
		struct dvfs *d = &core_dvfs_table[i];
		if (!match_dvfs_one(d, speedo_id, core_process_id))
			continue;
		init_dvfs_one(d, core_nominal_mv_index);
	}

	/* Initialize matching cpu dvfs entry already found when nominal
	   voltage was determined */
	init_dvfs_one(cpu_dvfs, cpu_nominal_mv_index);

	/* Finally disable dvfs on rails if necessary */
	if (tegra_dvfs_core_disabled)
		tegra_dvfs_rail_disable(&tegra3_dvfs_rail_vdd_core);
	if (tegra_dvfs_cpu_disabled)
		tegra_dvfs_rail_disable(&tegra3_dvfs_rail_vdd_cpu);

	pr_info("tegra dvfs: VDD_CPU nominal %dmV, scaling %s\n",
		tegra3_dvfs_rail_vdd_cpu.nominal_millivolts,
		tegra_dvfs_cpu_disabled ? "disabled" : "enabled");
	pr_info("tegra dvfs: VDD_CORE nominal %dmV, scaling %s\n",
		tegra3_dvfs_rail_vdd_core.nominal_millivolts,
		tegra_dvfs_core_disabled ? "disabled" : "enabled");
}
