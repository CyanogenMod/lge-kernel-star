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
#include <linux/clk.h>
#include <linux/kobject.h>
#include <linux/err.h>

#include "clock.h"
#include "dvfs.h"
#include "fuse.h"
#include "board.h"
#include "tegra3_emc.h"

static bool tegra_dvfs_cpu_disabled;
static bool tegra_dvfs_core_disabled;

static const int cpu_millivolts[MAX_DVFS_FREQS] =
	{800, 825, 850, 875, 900, 912, 925, 950, 975, 1000, 1025, 1050, 1075, 1100, 1125, 1150, 1200, 1237};

static const int core_millivolts[MAX_DVFS_FREQS] =
	{1000, 1050, 1100, 1150, 1200, 1250, 1300};

#define KHZ 1000
#define MHZ 1000000

/* VDD_CPU >= (VDD_CORE - cpu_below_core) */
/* VDD_CORE >= min_level(VDD_CPU), see tegra3_get_core_floor_mv() below */
#define VDD_CPU_BELOW_VDD_CORE		300
static int cpu_below_core = VDD_CPU_BELOW_VDD_CORE;

#define VDD_SAFE_STEP			100

static struct dvfs_rail tegra3_dvfs_rail_vdd_cpu = {
	.reg_id = "vdd_cpu",
	.max_millivolts = 1250,
	.min_millivolts = 850,
	.step = VDD_SAFE_STEP,
	.jmp_to_zero = true,
};

static struct dvfs_rail tegra3_dvfs_rail_vdd_core = {
	.reg_id = "vdd_core",
	.max_millivolts = 1300,
	.min_millivolts = 1000,
	.step = VDD_SAFE_STEP,
};

static struct dvfs_rail *tegra3_dvfs_rails[] = {
	&tegra3_dvfs_rail_vdd_cpu,
	&tegra3_dvfs_rail_vdd_core,
};

static int tegra3_get_core_floor_mv(int cpu_mv)
{
	if (cpu_mv <= 825)
		return 1000;
	if (cpu_mv <=  975)
		return 1100;
	if ((tegra_cpu_speedo_id() < 2) ||
	    (tegra_cpu_speedo_id() == 4))
		return 1200;
	if (cpu_mv <= 1075)
		return 1200;
	if (cpu_mv <= 1250)
		return 1300;
	BUG();
}

/* vdd_core must be >= min_level as a function of vdd_cpu */
static int tegra3_dvfs_rel_vdd_cpu_vdd_core(struct dvfs_rail *vdd_cpu,
	struct dvfs_rail *vdd_core)
{
	int core_floor = max(vdd_cpu->new_millivolts, vdd_cpu->millivolts);
	core_floor = tegra3_get_core_floor_mv(core_floor);
	return max(vdd_core->new_millivolts, core_floor);
}

/* vdd_cpu must be >= (vdd_core - cpu_below_core) */
static int tegra3_dvfs_rel_vdd_core_vdd_cpu(struct dvfs_rail *vdd_core,
	struct dvfs_rail *vdd_cpu)
{
	int cpu_floor;

	if (vdd_cpu->new_millivolts == 0)
		return 0; /* If G CPU is off, core relations can be ignored */

	cpu_floor = max(vdd_core->new_millivolts, vdd_core->millivolts) -
		cpu_below_core;
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
	/* Cpu voltages (mV):	     800, 825, 850, 875,  900,  912,  925,  950,  975, 1000, 1025, 1050, 1075, 1100, 1125, 1150, 1200, 1237 */
	CPU_DVFS("cpu_g",  0, 0, MHZ,   1,   1, 684, 684,  817,  817,  817,  817, 1026, 1102, 1149, 1187, 1225, 1282, 1300),
	CPU_DVFS("cpu_g",  0, 1, MHZ,   1,   1, 807, 807,  948,  948,  948,  948, 1117, 1171, 1206, 1300),
	CPU_DVFS("cpu_g",  0, 2, MHZ,   1,   1, 883, 883, 1039, 1039, 1039, 1039, 1178, 1206, 1300),
	CPU_DVFS("cpu_g",  0, 3, MHZ,   1,   1, 931, 931, 1102, 1102, 1102, 1102, 1216, 1300),

	CPU_DVFS("cpu_g",  1, 0, MHZ,   1,   1, 550, 550,  680,  680,  680,  680,  820,  970, 1040, 1080, 1150, 1200, 1280, 1300),
	CPU_DVFS("cpu_g",  1, 1, MHZ,   1,   1, 650, 650,  820,  820,  820,  820, 1000, 1060, 1100, 1200, 1300),
	CPU_DVFS("cpu_g",  1, 2, MHZ,   1,   1, 720, 720,  880,  880,  880,  880, 1090, 1180, 1200, 1300),
	CPU_DVFS("cpu_g",  1, 3, MHZ,   1,   1, 800, 800, 1000, 1000, 1000, 1000, 1180, 1230, 1300),

	CPU_DVFS("cpu_g",  2, 1, MHZ,   1,   1, 650, 650,  820,  820,  820,  820, 1000, 1060, 1100, 1200, 1250, 1300, 1330, 1400),
	CPU_DVFS("cpu_g",  2, 2, MHZ,   1,   1, 720, 720,  880,  880,  880,  880, 1090, 1180, 1200, 1300, 1310, 1350, 1400),
	CPU_DVFS("cpu_g",  2, 3, MHZ,   1,   1, 800, 800, 1000, 1000, 1000, 1000, 1180, 1230, 1300, 1320, 1350, 1400),

	CPU_DVFS("cpu_g",  3, 1, MHZ,   1,   1, 650, 650,  820,  820,  820,  820, 1000, 1060, 1100, 1200, 1250, 1300, 1330, 1400),
	CPU_DVFS("cpu_g",  3, 2, MHZ,   1,   1, 720, 720,  880,  880,  880,  880, 1090, 1180, 1200, 1300, 1310, 1350, 1400),
	CPU_DVFS("cpu_g",  3, 3, MHZ,   1,   1, 800, 800, 1000, 1000, 1000, 1000, 1180, 1230, 1300, 1320, 1350, 1400),

	CPU_DVFS("cpu_g",  4, 0, MHZ,   1,   1, 550, 550,  680,  680,  680,  680,  820,  970, 1040, 1080, 1150, 1200, 1280, 1350, 1400, 1500),
	CPU_DVFS("cpu_g",  4, 1, MHZ,   1,   1, 650, 650,  820,  820,  820,  820, 1000, 1060, 1100, 1200, 1250, 1300, 1360, 1400, 1500),
	CPU_DVFS("cpu_g",  4, 2, MHZ,   1,   1, 720, 720,  880,  880,  880,  880, 1090, 1180, 1200, 1300, 1310, 1380, 1400, 1500),
	CPU_DVFS("cpu_g",  4, 3, MHZ,   1,   1, 800, 800, 1000, 1000, 1000, 1000, 1180, 1230, 1300, 1330, 1380, 1400, 1500),

	CPU_DVFS("cpu_g",  5, 3, MHZ,   1,   1, 800, 800, 1000, 1000, 1000, 1000, 1180, 1230, 1300, 1330, 1380, 1400, 1470, 1500, 1540, 1700),
	CPU_DVFS("cpu_g",  5, 4, MHZ,   1,   1, 840, 840, 1000, 1000, 1000, 1000, 1200, 1280, 1330, 1380, 1400, 1480, 1500, 1520, 1700),

	CPU_DVFS("cpu_g",  6, 3, MHZ,   1,   1, 800, 800, 1000, 1000, 1000, 1000, 1180, 1230, 1300, 1330, 1380, 1400, 1470, 1500, 1540, 1700),
	CPU_DVFS("cpu_g",  6, 4, MHZ,   1,   1, 840, 840, 1000, 1000, 1000, 1000, 1200, 1280, 1330, 1380, 1400, 1480, 1500, 1520, 1700),

	CPU_DVFS("cpu_g",  7, 0, MHZ,   1,   1, 550, 550,  680,  680,  680,  680,  820,  970, 1040, 1080, 1150, 1200, 1280, 1300),
	CPU_DVFS("cpu_g",  7, 1, MHZ,   1,   1, 650, 650,  820,  820,  820,  820, 1000, 1060, 1100, 1200, 1300),
	CPU_DVFS("cpu_g",  7, 2, MHZ,   1,   1, 720, 720,  880,  880,  880,  880, 1090, 1180, 1200, 1300),
	CPU_DVFS("cpu_g",  7, 3, MHZ,   1,   1, 800, 800, 1000, 1000, 1000, 1000, 1180, 1200, 1300),
	CPU_DVFS("cpu_g",  7, 4, MHZ,   1,   1, 840, 840, 1000, 1000, 1000, 1000, 1200, 1300),

	CPU_DVFS("cpu_g",  8, 0, MHZ,   1,   1, 550, 550,  680,  680,  680,  680,  820,  970, 1040, 1080, 1150, 1200, 1280, 1300),
	CPU_DVFS("cpu_g",  8, 1, MHZ,   1,   1, 650, 650,  820,  820,  820,  820, 1000, 1060, 1100, 1200, 1300),
	CPU_DVFS("cpu_g",  8, 2, MHZ,   1,   1, 720, 720,  880,  880,  880,  880, 1090, 1180, 1200, 1300),
	CPU_DVFS("cpu_g",  8, 3, MHZ,   1,   1, 800, 800, 1000, 1000, 1000, 1000, 1180, 1200, 1300),
	CPU_DVFS("cpu_g",  8, 4, MHZ,   1,   1, 840, 840, 1000, 1000, 1000, 1000, 1200, 1300),

	CPU_DVFS("cpu_g",  9, -1, MHZ,  1,   1,   1,   1,    1,  900,  900,  900,  900,  900,  900,  900,  900,  900,  900,  900),
	CPU_DVFS("cpu_g", 10, -1, MHZ,  1,   1, 900, 900,  900,  900,  900,  900,  900,  900,  900,  900,  900,  900,  900,  900),
	CPU_DVFS("cpu_g", 11, -1, MHZ,  1,   1, 600, 600,  600,  600,  600,  600,  600,  600,  600,  600,  600,  600,  600,  600),

	/*
	 * "Safe entry" to be used when no match for chip speedo, process
	 *  corner is found (just to boot at low rate); must be the last one
	 */
	CPU_DVFS("cpu_g", -1, -1, MHZ, 1,   1, 216, 216, 300),
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
	/* Core voltages (mV):		   1000,   1050,   1100,   1150,    1200,    1250,    1300 */
	/* Clock limits for internal blocks, PLLs */
	CORE_DVFS("cpu_lp", 0, 1, KHZ,   294000, 342000, 427000, 475000,  500000,  500000,  500000),
	CORE_DVFS("cpu_lp", 1, 1, KHZ,   294000, 342000, 427000, 475000,  500000,  500000,  500000),
	CORE_DVFS("cpu_lp", 2, 1, KHZ,   295000, 370000, 428000, 475000,  513000,  579000,  620000),
	CORE_DVFS("cpu_lp", 3, 1, KHZ,        1,      1,      1,      1,       1,  450000,  450000),

	CORE_DVFS("emc",    0, 1, KHZ,   266500, 266500, 266500, 266500,  533000,  533000,  533000),
	CORE_DVFS("emc",    1, 1, KHZ,   408000, 408000, 408000, 408000,  667000,  667000,  667000),
	CORE_DVFS("emc",    2, 1, KHZ,   408000, 408000, 408000, 408000,  667000,  667000,  800000),
	CORE_DVFS("emc",    3, 1, KHZ,        1,      1,      1,      1,       1,  625000,  625000),

	CORE_DVFS("sbus",   0, 1, KHZ,   136000, 164000, 191000, 216000,  216000,  216000,  216000),
	CORE_DVFS("sbus",   1, 1, KHZ,   205000, 205000, 227000, 227000,  267000,  267000,  267000),
	CORE_DVFS("sbus",   2, 1, KHZ,   205000, 205000, 227000, 227000,  267000,  334000,  334000),
	CORE_DVFS("sbus",   3, 1, KHZ,        1,      1,      1,      1,       1,  216000,  216000),

	CORE_DVFS("vi",     0, 1, KHZ,   216000, 285000, 300000, 300000,  300000,  300000,  300000),
	CORE_DVFS("vi",     1, 1, KHZ,   216000, 267000, 300000, 371000,  409000,  409000,  409000),
	CORE_DVFS("vi",     2, 1, KHZ,   219000, 267000, 300000, 371000,  409000,  425000,  425000),
	CORE_DVFS("vi",     3, 1, KHZ,        1,      1,      1,      1,       1,  300000,  300000),

	CORE_DVFS("vde",    0, 1, KHZ,   228000, 275000, 332000, 380000,  416000,  416000,  416000),
	CORE_DVFS("mpe",    0, 1, KHZ,   234000, 285000, 332000, 380000,  416000,  416000,  416000),
	CORE_DVFS("2d",     0, 1, KHZ,   267000, 285000, 332000, 380000,  416000,  416000,  416000),
	CORE_DVFS("epp",    0, 1, KHZ,   267000, 285000, 332000, 380000,  416000,  416000,  416000),
	CORE_DVFS("3d",     0, 1, KHZ,   234000, 285000, 332000, 380000,  416000,  416000,  416000),
	CORE_DVFS("3d2",    0, 1, KHZ,   234000, 285000, 332000, 380000,  416000,  416000,  416000),
	CORE_DVFS("se",     0, 1, KHZ,   267000, 285000, 332000, 380000,  416000,  416000,  416000),

	CORE_DVFS("vde",    1, 1, KHZ,   228000, 275000, 332000, 380000,  416000,  416000,  416000),
	CORE_DVFS("mpe",    1, 1, KHZ,   234000, 285000, 332000, 380000,  416000,  416000,  416000),
	CORE_DVFS("2d",     1, 1, KHZ,   267000, 285000, 332000, 380000,  416000,  416000,  416000),
	CORE_DVFS("epp",    1, 1, KHZ,   267000, 285000, 332000, 380000,  416000,  416000,  416000),
	CORE_DVFS("3d",     1, 1, KHZ,   234000, 285000, 332000, 380000,  416000,  416000,  416000),
	CORE_DVFS("3d2",    1, 1, KHZ,   234000, 285000, 332000, 380000,  416000,  416000,  416000),
	CORE_DVFS("se",     1, 1, KHZ,   267000, 285000, 332000, 380000,  416000,  416000,  416000),

	CORE_DVFS("vde",    2, 1, KHZ,   247000, 304000, 352000, 400000,  437000,  484000,  520000),
	CORE_DVFS("mpe",    2, 1, KHZ,   247000, 304000, 361000, 408000,  446000,  484000,  520000),
	CORE_DVFS("2d",     2, 1, KHZ,   267000, 304000, 361000, 408000,  446000,  484000,  520000),
	CORE_DVFS("epp",    2, 1, KHZ,   267000, 304000, 361000, 408000,  446000,  484000,  520000),
	CORE_DVFS("3d",     2, 1, KHZ,   247000, 304000, 361000, 408000,  446000,  484000,  520000),
	CORE_DVFS("3d2",    2, 1, KHZ,   247000, 304000, 361000, 408000,  446000,  484000,  520000),
	CORE_DVFS("se",     2, 1, KHZ,   267000, 304000, 361000, 408000,  446000,  484000,  520000),

	CORE_DVFS("vde",    3, 1, KHZ,        1,      1,      1,      1,       1,  484000,  484000),
	CORE_DVFS("mpe",    3, 1, KHZ,        1,      1,      1,      1,       1,  484000,  484000),
	CORE_DVFS("2d",     3, 1, KHZ,        1,      1,      1,      1,       1,  484000,  484000),
	CORE_DVFS("epp",    3, 1, KHZ,        1,      1,      1,      1,       1,  484000,  484000),
	CORE_DVFS("3d",     3, 1, KHZ,        1,      1,      1,      1,       1,  484000,  484000),
	CORE_DVFS("3d2",    3, 1, KHZ,        1,      1,      1,      1,       1,  484000,  484000),
	CORE_DVFS("se",     3, 1, KHZ,        1,      1,      1,      1,       1,  650000,  650000),

	CORE_DVFS("host1x", 0, 1, KHZ,   152000, 188000, 222000, 254000,  267000,  267000,  267000),
	CORE_DVFS("host1x", 1, 1, KHZ,   152000, 188000, 222000, 254000,  267000,  267000,  267000),
	CORE_DVFS("host1x", 2, 1, KHZ,   152000, 188000, 222000, 254000,  267000,  267000,  267000),
	CORE_DVFS("host1x", 3, 1, KHZ,        1,      1,      1,      1,       1,  300000,  300000),

	CORE_DVFS("cbus",   0, 1, KHZ,   228000, 275000, 332000, 380000,  416000,  416000,  416000),
	CORE_DVFS("cbus",   1, 1, KHZ,   228000, 275000, 332000, 380000,  416000,  416000,  416000),
	CORE_DVFS("cbus",   2, 1, KHZ,   247000, 304000, 352000, 400000,  437000,  484000,  520000),
	CORE_DVFS("cbus",   3, 1, KHZ,   484000, 484000, 484000, 484000,  484000,  484000,  484000),

	CORE_DVFS("pll_c",  -1, 1, KHZ,  667000, 667000, 800000, 800000, 1066000, 1066000, 1066000),
	CORE_DVFS("pll_m",  -1, 1, KHZ,  667000, 667000, 800000, 800000, 1066000, 1066000, 1066000),

	/* Core voltages (mV):		   1000,   1050,   1100,   1150,    1200,   1250,    1300 */
	/* Clock limits for I/O peripherals */
	CORE_DVFS("mipi",   0, 1, KHZ,        1,      1,      1,      1,      1,       1,        1),
	CORE_DVFS("mipi",   1, 1, KHZ,        1,      1,      1,      1,  60000,   60000,    60000),
	CORE_DVFS("mipi",   2, 1, KHZ,        1,      1,      1,      1,  60000,   60000,    60000),
	CORE_DVFS("mipi",   3, 1, KHZ,        1,      1,      1,      1,      1,       1,        1),

	CORE_DVFS("fuse_burn", -1, 1, KHZ,    1,      1,      1,  26000,  26000,   26000,    26000),
	CORE_DVFS("sdmmc1",-1, 1, KHZ,   104000, 104000, 104000, 104000, 208000,  208000,   208000),
	CORE_DVFS("sdmmc3",-1, 1, KHZ,   104000, 104000, 104000, 104000, 208000,  208000,   208000),
	CORE_DVFS("ndflash", -1, 1, KHZ, 120000, 120000, 120000, 200000, 200000,  200000,   200000),

	CORE_DVFS("nor",    0, 1, KHZ,   115000, 130000, 130000, 133000, 133000,  133000,   133000),
	CORE_DVFS("nor",    1, 1, KHZ,   115000, 130000, 130000, 133000, 133000,  133000,   133000),
	CORE_DVFS("nor",    2, 1, KHZ,   115000, 130000, 130000, 133000, 133000,  133000,   133000),
	CORE_DVFS("nor",    3, 1, KHZ,        1,      1,      1,      1,      1,  108000,   108000),

	CORE_DVFS("sbc1",  -1, 1, KHZ,    40000,  60000,  60000,  60000, 100000,  100000,   100000),
	CORE_DVFS("sbc2",  -1, 1, KHZ,    40000,  60000,  60000,  60000, 100000,  100000,   100000),
	CORE_DVFS("sbc3",  -1, 1, KHZ,    40000,  60000,  60000,  60000, 100000,  100000,   100000),
	CORE_DVFS("sbc4",  -1, 1, KHZ,    40000,  60000,  60000,  60000, 100000,  100000,   100000),
	CORE_DVFS("sbc5",  -1, 1, KHZ,    40000,  60000,  60000,  60000, 100000,  100000,   100000),
	CORE_DVFS("sbc6",  -1, 1, KHZ,    40000,  60000,  60000,  60000, 100000,  100000,   100000),

	CORE_DVFS("tvo",   -1, 1, KHZ,        1, 297000, 297000, 297000, 297000,  297000,   297000),
	CORE_DVFS("cve",   -1, 1, KHZ,        1, 297000, 297000, 297000, 297000,  297000,   297000),
	CORE_DVFS("dsia",  -1, 1, KHZ,   275000, 275000, 275000, 275000, 275000,  275000,   275000),
	CORE_DVFS("dsib",  -1, 1, KHZ,   275000, 275000, 275000, 275000, 275000,  275000,   275000),

	/*
	 * The clock rate for the display controllers that determines the
	 * necessary core voltage depends on a divider that is internal
	 * to the display block.  Disable auto-dvfs on the display clocks,
	 * and let the display driver call tegra_dvfs_set_rate manually
	 */
	CORE_DVFS("disp1",  0, 0, KHZ,   120000, 120000, 120000, 120000, 190000,  190000,   190000),
	CORE_DVFS("disp1",  1, 0, KHZ,   151000, 268000, 268000, 268000, 268000,  268000,   268000),
	CORE_DVFS("disp1",  2, 0, KHZ,   151000, 268000, 268000, 268000, 268000,  268000,   268000),
	CORE_DVFS("disp1",  3, 0, KHZ,   120000, 120000, 120000, 120000, 190000,  190000,   190000),

	CORE_DVFS("disp2",  0, 0, KHZ,   120000, 120000, 120000, 120000, 190000,  190000,   190000),
	CORE_DVFS("disp2",  1, 0, KHZ,   151000, 268000, 268000, 268000, 268000,  268000,   268000),
	CORE_DVFS("disp2",  2, 0, KHZ,   151000, 268000, 268000, 268000, 268000,  268000,   268000),
	CORE_DVFS("disp2",  3, 0, KHZ,   120000, 120000, 120000, 120000, 190000,  190000,   190000),
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

static bool __init is_pllm_dvfs(struct clk *c, struct dvfs *d)
{
#ifdef CONFIG_TEGRA_PLLM_RESTRICTED
	/* Restricting PLLM usage on T30 and T33, rev A02+, allows to apply
	   maximum PLLM frequency to clock tree at minimum core voltage;
	   no need to enable dvfs on PLLM in this case */
	if ((tegra_cpu_speedo_id() == 2) || (tegra_cpu_speedo_id() == 5))
	   return false;
#endif
	/* Check if PLLM boot frequency can be applied to clock tree at
	   minimum voltage. If yes, no need to enable dvfs on PLLM */
	if (clk_get_rate_all_locked(c) <= d->freqs[0] * d->freqs_mult)
		return false;

	return true;
}

static void __init init_dvfs_one(struct dvfs *d, int nominal_mv_index)
{
	int ret;
	struct clk *c = tegra_get_clock_by_name(d->clk_name);

	if (!c) {
		pr_debug("tegra3_dvfs: no clock found for %s\n",
			d->clk_name);
		return;
	}

	/*
	 * Update max rate for auto-dvfs clocks, except EMC.
	 * EMC is a special case, since EMC dvfs is board dependent: max rate
	 * and EMC scaling frequencies are determined by tegra BCT (flashed
	 * together with the image) and board specific EMC DFS table; we will
	 * check the scaling ladder against nominal core voltage when the table
	 * is loaded (and if on particular board the table is not loaded, EMC
	 * scaling is disabled).
	 */
	if (!(c->flags & PERIPH_EMC_ENB) && d->auto_dvfs) {
		BUG_ON(!d->freqs[nominal_mv_index]);
		tegra_init_max_rate(
			c, d->freqs[nominal_mv_index] * d->freqs_mult);
	}
	d->max_millivolts = d->dvfs_rail->nominal_millivolts;

	/*
	 * Check if we may skip enabling dvfs on PLLM. PLLM is a special case,
	 * since its frequency never exceeds boot rate, and configuration with
	 * restricted PLLM usage is possible.
	 */
	if (!(c->flags & PLLM) || is_pllm_dvfs(c, d)) {
		ret = tegra_enable_dvfs_on_clk(c, d);
		if (ret)
			pr_err("tegra3_dvfs: failed to enable dvfs on %s\n",
				c->name);
	}
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
	 * Find maximum cpu voltage that satisfies cpu_to_core dependency for
	 * nominal core voltage ("solve from cpu to core at nominal"). Clip
	 * result to the nominal cpu level for the chips with this speedo_id.
	 */
	mv = tegra3_dvfs_rail_vdd_core.nominal_millivolts;
	for (i = 0; i < MAX_DVFS_FREQS; i++) {
		if ((cpu_millivolts[i] == 0) ||
		    tegra3_get_core_floor_mv(cpu_millivolts[i]) > mv)
			break;
	}
	BUG_ON(i == 0);
	mv = cpu_millivolts[i - 1];
	BUG_ON(mv < tegra3_dvfs_rail_vdd_cpu.min_millivolts);
	mv = min(mv, tegra_cpu_speedo_mv());

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
	int i;
	int mv = tegra_core_speedo_mv();
	int core_edp_limit = get_core_edp();

	/*
	 * Start with nominal level for the chips with this speedo_id. Then,
	 * make sure core nominal voltage is below edp limit for the board
	 * (if edp limit is set).
	 */
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
	int cpu_speedo_id = tegra_cpu_speedo_id();
	int soc_speedo_id = tegra_soc_speedo_id();
	int cpu_process_id = tegra_cpu_process_id();
	int core_process_id = tegra_core_process_id();

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
	core_nominal_mv_index = get_core_nominal_mv_index(soc_speedo_id);
	if (core_nominal_mv_index < 0) {
		tegra3_dvfs_rail_vdd_core.disabled = true;
		tegra_dvfs_core_disabled = true;
		core_nominal_mv_index = 0;
	}
	tegra3_dvfs_rail_vdd_core.nominal_millivolts =
		core_millivolts[core_nominal_mv_index];

	cpu_nominal_mv_index = get_cpu_nominal_mv_index(
		cpu_speedo_id, cpu_process_id, &cpu_dvfs);
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
		if (!match_dvfs_one(d, soc_speedo_id, core_process_id))
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

int tegra_dvfs_rail_disable_prepare(struct dvfs_rail *rail)
{
	int ret = 0;

	if (tegra_emc_get_dram_type() != DRAM_TYPE_DDR3)
		return ret;

	if (((&tegra3_dvfs_rail_vdd_core == rail) &&
	     (rail->nominal_millivolts > TEGRA_EMC_BRIDGE_MVOLTS_MIN)) ||
	    ((&tegra3_dvfs_rail_vdd_cpu == rail) &&
	     (tegra3_get_core_floor_mv(rail->nominal_millivolts) >
	      TEGRA_EMC_BRIDGE_MVOLTS_MIN))) {
		struct clk *bridge = tegra_get_clock_by_name("bridge.emc");
		BUG_ON(!bridge);

		ret = clk_enable(bridge);
		pr_info("%s: %s: %s bridge.emc\n", __func__,
			rail->reg_id, ret ? "failed to enable" : "enabled");
	}
	return ret;
}

int tegra_dvfs_rail_post_enable(struct dvfs_rail *rail)
{
	if (tegra_emc_get_dram_type() != DRAM_TYPE_DDR3)
		return 0;

	if (((&tegra3_dvfs_rail_vdd_core == rail) &&
	     (rail->nominal_millivolts > TEGRA_EMC_BRIDGE_MVOLTS_MIN)) ||
	    ((&tegra3_dvfs_rail_vdd_cpu == rail) &&
	     (tegra3_get_core_floor_mv(rail->nominal_millivolts) >
	      TEGRA_EMC_BRIDGE_MVOLTS_MIN))) {
		struct clk *bridge = tegra_get_clock_by_name("bridge.emc");
		BUG_ON(!bridge);

		clk_disable(bridge);
		pr_info("%s: %s: disabled bridge.emc\n",
			__func__, rail->reg_id);
	}
	return 0;
}

/*
 * sysfs and dvfs interfaces to cap tegra core domains frequencies
 */
static DEFINE_MUTEX(core_cap_lock);

struct core_cap {
	int refcnt;
	int level;
};
static struct core_cap tegra3_core_cap;
static struct core_cap kdvfs_core_cap;
static struct core_cap user_core_cap;

static struct kobject *cap_kobj;

/* Arranged in order required for enabling/lowering the cap */
static struct {
	const char *cap_name;
	struct clk *cap_clk;
	unsigned long freqs[MAX_DVFS_FREQS];
} core_cap_table[] = {
	{ .cap_name = "cap.cbus" },
	{ .cap_name = "cap.sclk" },
	{ .cap_name = "cap.emc" },
};


static void core_cap_level_set(int level)
{
	int i, j;

	for (j = 0; j < ARRAY_SIZE(core_millivolts); j++) {
		int v = core_millivolts[j];
		if ((v == 0) || (level < v))
			break;
	}
	j = (j == 0) ? 0 : j - 1;
	level = core_millivolts[j];

	if (level < tegra3_core_cap.level) {
		for (i = 0; i < ARRAY_SIZE(core_cap_table); i++)
			if (core_cap_table[i].cap_clk)
				clk_set_rate(core_cap_table[i].cap_clk,
					     core_cap_table[i].freqs[j]);
	} else if (level > tegra3_core_cap.level) {
		for (i = ARRAY_SIZE(core_cap_table) - 1; i >= 0; i--)
			if (core_cap_table[i].cap_clk)
				clk_set_rate(core_cap_table[i].cap_clk,
					     core_cap_table[i].freqs[j]);
	}
	tegra3_core_cap.level = level;
}

static void core_cap_update(void)
{
	int new_level = tegra3_dvfs_rail_vdd_core.max_millivolts;

	if (kdvfs_core_cap.refcnt)
		new_level = min(new_level, kdvfs_core_cap.level);
	if (user_core_cap.refcnt)
		new_level = min(new_level, user_core_cap.level);

	if (tegra3_core_cap.level != new_level)
		core_cap_level_set(new_level);
}

static void core_cap_enable(bool enable)
{
	int i;

	if (enable) {
		tegra3_core_cap.refcnt++;
		if (tegra3_core_cap.refcnt == 1)
			for (i = 0; i < ARRAY_SIZE(core_cap_table); i++)
				if (core_cap_table[i].cap_clk)
					clk_enable(core_cap_table[i].cap_clk);
	} else if (tegra3_core_cap.refcnt) {
		tegra3_core_cap.refcnt--;
		if (tegra3_core_cap.refcnt == 0)
			for (i = ARRAY_SIZE(core_cap_table) - 1; i >= 0; i--)
				if (core_cap_table[i].cap_clk)
					clk_disable(core_cap_table[i].cap_clk);
	}
	core_cap_update();
}

static ssize_t
core_cap_state_show(struct kobject *kobj, struct kobj_attribute *attr,
		    char *buf)
{
	return sprintf(buf, "%d (%d)\n", tegra3_core_cap.refcnt ? 1 : 0,
			user_core_cap.refcnt ? 1 : 0);
}
static ssize_t
core_cap_state_store(struct kobject *kobj, struct kobj_attribute *attr,
		     const char *buf, size_t count)
{
	int state;

	if (sscanf(buf, "%d", &state) != 1)
		return -1;

	mutex_lock(&core_cap_lock);

	if (state) {
		user_core_cap.refcnt++;
		if (user_core_cap.refcnt == 1)
			core_cap_enable(true);
	} else if (user_core_cap.refcnt) {
		user_core_cap.refcnt--;
		if (user_core_cap.refcnt == 0)
			core_cap_enable(false);
	}

	mutex_unlock(&core_cap_lock);
	return count;
}

static ssize_t
core_cap_level_show(struct kobject *kobj, struct kobj_attribute *attr,
		    char *buf)
{
	return sprintf(buf, "%d (%d)\n", tegra3_core_cap.level,
			user_core_cap.level);
}
static ssize_t
core_cap_level_store(struct kobject *kobj, struct kobj_attribute *attr,
		     const char *buf, size_t count)
{
	int level;

	if (sscanf(buf, "%d", &level) != 1)
		return -1;

	mutex_lock(&core_cap_lock);
	user_core_cap.level = level;
	core_cap_update();
	mutex_unlock(&core_cap_lock);
	return count;
}

static struct kobj_attribute cap_state_attribute =
	__ATTR(core_cap_state, 0644, core_cap_state_show, core_cap_state_store);
static struct kobj_attribute cap_level_attribute =
	__ATTR(core_cap_level, 0644, core_cap_level_show, core_cap_level_store);

const struct attribute *cap_attributes[] = {
	&cap_state_attribute.attr,
	&cap_level_attribute.attr,
	NULL,
};

void tegra_dvfs_core_cap_enable(bool enable)
{
	mutex_lock(&core_cap_lock);

	if (enable) {
		kdvfs_core_cap.refcnt++;
		if (kdvfs_core_cap.refcnt == 1)
			core_cap_enable(true);
	} else if (kdvfs_core_cap.refcnt) {
		kdvfs_core_cap.refcnt--;
		if (kdvfs_core_cap.refcnt == 0)
			core_cap_enable(false);
	}
	mutex_unlock(&core_cap_lock);
}

void tegra_dvfs_core_cap_level_set(int level)
{
	mutex_lock(&core_cap_lock);
	kdvfs_core_cap.level = level;
	core_cap_update();
	mutex_unlock(&core_cap_lock);
}

static int __init init_core_cap_one(struct clk *c, unsigned long *freqs)
{
	int i, v, next_v;
	unsigned long rate, next_rate = 0;

	for (i = 0; i < ARRAY_SIZE(core_millivolts); i++) {
		v = core_millivolts[i];
		if (v == 0)
			break;

		for (;;) {
			rate = next_rate;
			next_rate = clk_round_rate(c, rate + 1000);
			if (IS_ERR_VALUE(next_rate)) {
				pr_debug("tegra3_dvfs: failed to round %s"
					   " rate %lu", c->name, rate);
				return -EINVAL;
			}
			if (rate == next_rate)
				break;

			next_v = tegra_dvfs_predict_millivolts(
				c->parent, next_rate);
			if (IS_ERR_VALUE(next_rate)) {
				pr_debug("tegra3_dvfs: failed to predict %s mV"
					 " for rate %lu", c->name, next_rate);
				return -EINVAL;
			}
			if (next_v > v)
				break;
		}

		if (rate == 0) {
			rate = next_rate;
			pr_warn("tegra3_dvfs: minimum %s cap %lu requires"
				" %d mV", c->name, rate, next_v);
		}
		freqs[i] = rate;
		next_rate = rate;
	}
	return 0;
}

static int __init tegra_dvfs_init_core_cap(void)
{
	int i;
	struct clk *c = NULL;

	tegra3_core_cap.level = kdvfs_core_cap.level = user_core_cap.level =
		tegra3_dvfs_rail_vdd_core.max_millivolts;

	for (i = 0; i < ARRAY_SIZE(core_cap_table); i++) {
		c = tegra_get_clock_by_name(core_cap_table[i].cap_name);
		if (!c || !c->parent ||
		    init_core_cap_one(c, core_cap_table[i].freqs)) {
			pr_err("tegra3_dvfs: failed to initialize %s frequency"
			       " table", core_cap_table[i].cap_name);
			continue;
		}
		core_cap_table[i].cap_clk = c;
	}

	cap_kobj = kobject_create_and_add("tegra_cap", kernel_kobj);
	if (!cap_kobj) {
		pr_err("tegra3_dvfs: failed to create sysfs cap object");
		return 0;
	}

	if (sysfs_create_files(cap_kobj, cap_attributes)) {
		pr_err("tegra3_dvfs: failed to create sysfs cap interface");
		return 0;
	}
	pr_info("tegra dvfs: tegra sysfs cap interface is initialized\n");

	return 0;
}
late_initcall(tegra_dvfs_init_core_cap);
