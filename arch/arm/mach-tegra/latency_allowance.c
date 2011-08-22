/*
 * arch/arm/mach-tegra/latency_allowance.c
 *
 * Copyright (C) 2011 NVIDIA Corporation
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

#include <linux/types.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/err.h>
#include <linux/spinlock_types.h>
#include <linux/spinlock.h>
#include <linux/stringify.h>
#include <asm/bug.h>
#include <asm/io.h>
#include <asm/string.h>
#include <mach/iomap.h>
#include <mach/io.h>
#include <mach/latency_allowance.h>

#define MC_ARB_OVERRIDE		0xe8
#define GLOBAL_LATENCY_SCALING_ENABLE_BIT 7

#define MC_LA_AFI_0		0x2e0
#define MC_LA_AVPC_ARM7_0	0x2e4
#define MC_LA_DC_0		0x2e8
#define MC_LA_DC_1		0x2ec
#define MC_LA_DC_2		0x2f0
#define MC_LA_DCB_0		0x2f4
#define MC_LA_DCB_1		0x2f8
#define MC_LA_DCB_2		0x2fc
#define MC_LA_EPP_0		0x300
#define MC_LA_EPP_1		0x304
#define MC_LA_G2_0		0x308
#define MC_LA_G2_1		0x304
#define MC_LA_HC_0		0x310
#define MC_LA_HC_1		0x314
#define MC_LA_HDA_0		0x318
#define MC_LA_ISP_0		0x31C
#define MC_LA_MPCORE_0		0x320
#define MC_LA_MPCORELP_0	0x324
#define MC_LA_MPE_0		0x328
#define MC_LA_MPE_1		0x32c
#define MC_LA_MPE_2		0x330
#define MC_LA_NV_0		0x334
#define MC_LA_NV_1		0x338
#define MC_LA_NV2_0		0x33c
#define MC_LA_NV2_1		0x340
#define MC_LA_PPCS_0		0x344
#define MC_LA_PPCS_1		0x348
#define MC_LA_PTC_0		0x34c
#define MC_LA_SATA_0		0x350
#define MC_LA_VDE_0		0x354
#define MC_LA_VDE_1		0x358
#define MC_LA_VDE_2		0x35c
#define MC_LA_VDE_3		0x360
#define MC_LA_VI_0		0x364
#define MC_LA_VI_1		0x368
#define MC_LA_VI_2		0x36c

#define DS_DISP_MCCIF_DISPLAY0A_HYST (0x481 * 4)
#define DS_DISP_MCCIF_DISPLAY0B_HYST (0x482 * 4)
#define DS_DISP_MCCIF_DISPLAY0C_HYST (0x483 * 4)
#define DS_DISP_MCCIF_DISPLAY1B_HYST (0x484 * 4)

#define DS_DISP_MCCIF_DISPLAY0AB_HYST (0x481 * 4)
#define DS_DISP_MCCIF_DISPLAY0BB_HYST (0x482 * 4)
#define DS_DISP_MCCIF_DISPLAY0CB_HYST (0x483 * 4)
#define DS_DISP_MCCIF_DISPLAY1BB_HYST (0x484 * 4)

#define VI_MCCIF_VIWSB_HYST	(0x9a * 4)
#define VI_MCCIF_VIWU_HYST	(0x9b * 4)
#define VI_MCCIF_VIWV_HYST	(0x9c * 4)
#define VI_MCCIF_VIWY_HYST	(0x9d * 4)

#define VI_TIMEOUT_WOCAL_VI	(0x70 * 4)
#define VI_RESERVE_3		(0x97 * 4)
#define VI_RESERVE_4		(0x98 * 4)

/* maximum valid value for latency allowance */
#define MC_LA_MAX_VALUE		255

#define ENABLE_LA_DEBUG		0
#define TEST_LA_CODE		0

#define la_debug(fmt, ...) \
	if (ENABLE_LA_DEBUG) { \
		printk(KERN_INFO pr_fmt(fmt), ##__VA_ARGS__); \
	}

static struct dentry *latency_debug_dir;

struct la_client_info {
	unsigned int fifo_size_in_atoms;
	unsigned int expiration_in_ns;	/* worst case expiration value */
	unsigned long reg_addr;
	unsigned long mask;
	unsigned long shift;
	enum tegra_la_id id;
	char *name;
	bool scaling_supported;
};

static DEFINE_SPINLOCK(safety_lock);

static const int ns_per_tick = 30;
/* fifo atom size in bytes for non-fdc clients*/
static const int normal_atom_size = 16;
/* fifo atom size in bytes for fdc clients*/
static const int fdc_atom_size = 32;

#define MC_RA(r) \
	((u32)IO_ADDRESS(TEGRA_MC_BASE) + (MC_##r))
#define RA(r) \
	((u32)IO_ADDRESS(TEGRA_MC_BASE) + (MC_LA_##r))

#define MASK(x) \
	((0xFFFFFFFFUL >> (31 - (1 ? x) + (0 ? x))) << (0 ? x))
#define SHIFT(x) \
	(0 ? x)
#define ID(id) \
	TEGRA_LA_##id

#define LA_INFO(f, e, a, r, id, ss) \
{f, e, RA(a), MASK(r), SHIFT(r), ID(id), __stringify(id), ss}

/*
 * The rule for getting the fifo_size_in_atoms is:
 * 1.If REORDER_DEPTH exists, use it(default is overridden).
 * 2.Else if (write_client) use RFIFO_DEPTH.
 * 3.Else (read client) use RDFIFO_DEPTH.
 * Refer to project.h file.
 */
struct la_client_info la_info[] = {
	LA_INFO(32,	150,	AFI_0,	7 : 0,		AFIR,		false),
	LA_INFO(32,	150,	AFI_0,	23 : 16,	AFIW,		false),
	LA_INFO(2,	150,	AVPC_ARM7_0, 7 : 0,	AVPC_ARM7R,	false),
	LA_INFO(2,	150,	AVPC_ARM7_0, 23 : 16,	AVPC_ARM7W,	false),
	LA_INFO(128,	1050,	DC_0,	7 : 0,		DISPLAY_0A,	true),
	LA_INFO(64,	1050,	DC_0,	23 : 16,	DISPLAY_0B,	true),
	LA_INFO(128,	1050,	DC_1,	7 : 0,		DISPLAY_0C,	true),
	LA_INFO(64,	1050,	DC_1,	23 : 16,	DISPLAY_1B,	true),
	LA_INFO(2,	1050,	DC_2,	7 : 0,		DISPLAY_HC,	false),
	LA_INFO(128,	1050,	DCB_0,	7 : 0,		DISPLAY_0AB,	true),
	LA_INFO(64,	1050,	DCB_0,	23 : 16,	DISPLAY_0BB,	true),
	LA_INFO(128,	1050,	DCB_1,	7 : 0,		DISPLAY_0CB,	true),
	LA_INFO(64,	1050,	DCB_1,	23 : 16,	DISPLAY_1BB,	true),
	LA_INFO(2,	1050,	DCB_2,	7 : 0,		DISPLAY_HCB,	false),
	LA_INFO(8,	150,	EPP_0,	7 : 0,		EPPUP,		false),
	LA_INFO(64,	150,	EPP_0,	23 : 16,	EPPU,		false),
	LA_INFO(64,	150,	EPP_1,	7 : 0,		EPPV,		false),
	LA_INFO(64,	150,	EPP_1,	23 : 16,	EPPY,		false),
	LA_INFO(64,	150,	G2_0,	7 : 0,		G2PR,		false),
	LA_INFO(64,	150,	G2_0,	23 : 16,	G2SR,		false),
	LA_INFO(48,	150,	G2_1,	7 : 0,		G2DR,		false),
	LA_INFO(128,	150,	G2_1,	23 : 16,	G2DW,		false),
	LA_INFO(16,	150,	HC_0,	7 : 0,		HOST1X_DMAR,	false),
	LA_INFO(8,	150,	HC_0,	23 : 16,	HOST1XR,	false),
	LA_INFO(32,	150,	HC_1,	7 : 0,		HOST1XW,	false),
	LA_INFO(16,	150,	HDA_0,	7 : 0,		HDAR,		false),
	LA_INFO(16,	150,	HDA_0,	23 : 16,	HDAW,		false),
	LA_INFO(64,	150,	ISP_0,	7 : 0,		ISPW,		false),
	LA_INFO(14,	150,	MPCORE_0, 7 : 0,	MPCORER,	false),
	LA_INFO(24,	150,	MPCORE_0, 23 : 16,	MPCOREW,	false),
	LA_INFO(14,	150,	MPCORELP_0, 7 : 0,	MPCORE_LPR,	false),
	LA_INFO(24,	150,	MPCORELP_0, 23 : 16,	MPCORE_LPW,	false),
	LA_INFO(8,	150,	MPE_0,	7 : 0,		MPE_UNIFBR,	false),
	LA_INFO(2,	150,	MPE_0,	23 : 16,	MPE_IPRED,	false),
	LA_INFO(64,	150,	MPE_1,	7 : 0,		MPE_AMEMRD,	false),
	LA_INFO(8,	150,	MPE_1,	23 : 16,	MPE_CSRD,	false),
	LA_INFO(8,	150,	MPE_2,	7 : 0,		MPE_UNIFBW,	false),
	LA_INFO(8,	150,	MPE_2,	23 : 16,	MPE_CSWR,	false),
	LA_INFO(48,	150,	NV_0,	7 : 0,		FDCDRD,		false),
	LA_INFO(64,	150,	NV_0,	23 : 16,	IDXSRD,		false),
	LA_INFO(64,	150,	NV_1,	7 : 0,		TEXSRD,		false),
	LA_INFO(48,	150,	NV_1,	23 : 16,	FDCDWR,		false),
	LA_INFO(48,	150,	NV2_0,	7 : 0,		FDCDRD2,	false),
	LA_INFO(64,	150,	NV2_0,	23 : 16,	IDXSRD2,	false),
	LA_INFO(64,	150,	NV2_1,	7 : 0,		TEXSRD2,	false),
	LA_INFO(48,	150,	NV2_1,	23 : 16,	FDCDWR2,	false),
	LA_INFO(2,	150,	PPCS_0,	7 : 0,		PPCS_AHBDMAR,	false),
	LA_INFO(8,	150,	PPCS_0,	23 : 16,	PPCS_AHBSLVR,	false),
	LA_INFO(2,	150,	PPCS_1,	7 : 0,		PPCS_AHBDMAW,	false),
	LA_INFO(4,	150,	PPCS_1,	23 : 16,	PPCS_AHBSLVW,	false),
	LA_INFO(2,	150,	PTC_0,	7 : 0,		PTCR,		false),
	LA_INFO(32,	150,	SATA_0,	7 : 0,		SATAR,		false),
	LA_INFO(32,	150,	SATA_0,	23 : 16,	SATAW,		false),
	LA_INFO(8,	150,	VDE_0,	7 : 0,		VDE_BSEVR,	false),
	LA_INFO(4,	150,	VDE_0,	23 : 16,	VDE_MBER,	false),
	LA_INFO(16,	150,	VDE_1,	7 : 0,		VDE_MCER,	false),
	LA_INFO(16,	150,	VDE_1,	23 : 16,	VDE_TPER,	false),
	LA_INFO(4,	150,	VDE_2,	7 : 0,		VDE_BSEVW,	false),
	LA_INFO(16,	150,	VDE_2,	23 : 16,	VDE_DBGW,	false),
	LA_INFO(2,	150,	VDE_3,	7 : 0,		VDE_MBEW,	false),
	LA_INFO(16,	150,	VDE_3,	23 : 16,	VDE_TPMW,	false),
	LA_INFO(8,	1050,	VI_0,	7 : 0,		VI_RUV,		false),
	LA_INFO(64,	1050,	VI_0,	23 : 16,	VI_WSB,		true),
	LA_INFO(64,	1050,	VI_1,	7 : 0,		VI_WU,		true),
	LA_INFO(64,	1050,	VI_1,	23 : 16,	VI_WV,		true),
	LA_INFO(64,	1050,	VI_2,	7 : 0,		VI_WY,		true),

/* end of list. */
	LA_INFO(0,	0,	AFI_0,	0 : 0,		MAX_ID,		false)
};

struct la_scaling_info {
	unsigned int threshold_low;
	unsigned int threshold_mid;
	unsigned int threshold_high;
	int scaling_ref_count;
	int actual_la_to_set;
	int la_set;
};

struct la_scaling_reg_info {
	enum tegra_la_id id;
	unsigned int tl_reg_addr;
	unsigned int tl_mask;
	unsigned int tl_shift;
	unsigned int tm_reg_addr;
	unsigned int tm_mask;
	unsigned int tm_shift;
	unsigned int th_reg_addr;
	unsigned int th_mask;
	unsigned int th_shift;
};

#define DISP1_RA(r) \
	((u32)IO_ADDRESS(TEGRA_DISPLAY_BASE) + DS_DISP_MCCIF_##r##_HYST)
#define DISP2_RA(r) \
	((u32)IO_ADDRESS(TEGRA_DISPLAY2_BASE) + DS_DISP_MCCIF_##r##_HYST)

#define DISP_SCALING_REG_INFO(id, r, ra) \
	{ \
		ID(id), \
		ra(r), MASK(15 : 8), SHIFT(15 : 8), \
		ra(r), MASK(23 : 16), SHIFT(15 : 8), \
		ra(r), MASK(7 : 0), SHIFT(15 : 8) \
	}

struct la_scaling_reg_info disp_info[] = {
	DISP_SCALING_REG_INFO(DISPLAY_0A, DISPLAY0A, DISP1_RA),
	DISP_SCALING_REG_INFO(DISPLAY_0B, DISPLAY0B, DISP1_RA),
	DISP_SCALING_REG_INFO(DISPLAY_0C, DISPLAY0C, DISP1_RA),
	DISP_SCALING_REG_INFO(DISPLAY_1B, DISPLAY1B, DISP1_RA),
	DISP_SCALING_REG_INFO(MAX_ID,     DISPLAY1B, DISP1_RA), /*dummy entry*/
	DISP_SCALING_REG_INFO(DISPLAY_0AB, DISPLAY0AB, DISP2_RA),
	DISP_SCALING_REG_INFO(DISPLAY_0BB, DISPLAY0BB, DISP2_RA),
	DISP_SCALING_REG_INFO(DISPLAY_0CB, DISPLAY0CB, DISP2_RA),
	DISP_SCALING_REG_INFO(DISPLAY_1BB, DISPLAY1BB, DISP2_RA),
};

#define VI_TH_RA(r) \
	((u32)IO_ADDRESS(TEGRA_VI_BASE) + VI_MCCIF_##r##_HYST)
#define VI_TM_RA(r) \
	((u32)IO_ADDRESS(TEGRA_VI_BASE) + VI_TIMEOUT_WOCAL_VI)
#define VI_TL_RA(r) \
	((u32)IO_ADDRESS(TEGRA_VI_BASE) + VI_RESERVE_##r)

struct la_scaling_reg_info vi_info[] = {
	{
		ID(VI_WSB),
		VI_TL_RA(4), MASK(7 : 0), SHIFT(7 : 0),
		VI_TM_RA(0), MASK(7 : 0), SHIFT(7 : 0),
		VI_TH_RA(VIWSB), MASK(7 : 0), SHIFT(7 : 0)
	},
	{
		ID(VI_WU),
		VI_TL_RA(3), MASK(15 : 8), SHIFT(15 : 8),
		VI_TM_RA(0), MASK(15 : 8), SHIFT(15 : 8),
		VI_TH_RA(VIWU), MASK(7 : 0), SHIFT(7 : 0)
	},
	{
		ID(VI_WV),
		VI_TL_RA(3), MASK(7 : 0), SHIFT(7 : 0),
		VI_TM_RA(0), MASK(23 : 16), SHIFT(23 : 16),
		VI_TH_RA(VIWV), MASK(7 : 0), SHIFT(7 : 0)
	},
	{
		ID(VI_WY),
		VI_TL_RA(4), MASK(15 : 8), SHIFT(15 : 8),
		VI_TM_RA(0), MASK(31 : 24), SHIFT(31 : 24),
		VI_TH_RA(VIWY), MASK(7 : 0), SHIFT(7 : 0)
	}
};

static struct la_scaling_info scaling_info[TEGRA_LA_MAX_ID];
static int la_scaling_enable_count;

#define VALIDATE_ID(id) \
	do { \
		if (id >= TEGRA_LA_MAX_ID) \
			return -EINVAL; \
		BUG_ON(la_info[id].id != id); \
	} while (0)

#define VALIDATE_BW(bw_in_mbps) \
	do { \
		if (bw_in_mbps >= 4096) \
			return -EINVAL; \
	} while (0)

#define VALIDATE_THRESHOLDS(tl, tm, th) \
	do { \
		if (tl > 100 || tm > 100 || th > 100) \
			return -EINVAL; \
	} while (0)

static void set_thresholds(struct la_scaling_reg_info *info,
			    enum tegra_la_id id)
{
	unsigned long reg_read;
	unsigned long reg_write;
	unsigned int thresh_low;
	unsigned int thresh_mid;
	unsigned int thresh_high;
	int la_set;

	reg_read = readl(la_info[id].reg_addr);
	la_set = (reg_read & la_info[id].mask) >> la_info[id].shift;
	/* la should be set before enabling scaling. */
	BUG_ON(la_set != scaling_info[id].la_set);

	thresh_low = (scaling_info[id].threshold_low * la_set) / 100;
	thresh_mid = (scaling_info[id].threshold_mid * la_set) / 100;
	thresh_high = (scaling_info[id].threshold_high * la_set) / 100;
	la_debug("%s: la_set=%d, thresh_low=%d(%d%%), thresh_mid=%d(%d%%),"
		" thresh_high=%d(%d%%) ", __func__, la_set,
		thresh_low, scaling_info[id].threshold_low,
		thresh_mid, scaling_info[id].threshold_mid,
		thresh_high, scaling_info[id].threshold_high);

	reg_read = readl(info->tl_reg_addr);
	reg_write = (reg_read & ~info->tl_mask) |
		(thresh_low << info->tl_shift);
	writel(reg_write, info->tl_reg_addr);
	la_debug("reg_addr=0x%x, read=0x%x, write=0x%x",
		(u32)info->tl_reg_addr, (u32)reg_read, (u32)reg_write);

	reg_read = readl(info->tm_reg_addr);
	reg_write = (reg_read & ~info->tm_mask) |
		(thresh_mid << info->tm_shift);
	writel(reg_write, info->tm_reg_addr);
	la_debug("reg_addr=0x%x, read=0x%x, write=0x%x",
		(u32)info->tm_reg_addr, (u32)reg_read, (u32)reg_write);

	reg_read = readl(info->th_reg_addr);
	reg_write = (reg_read & ~info->th_mask) |
		(thresh_high << info->th_shift);
	writel(reg_write, info->th_reg_addr);
	la_debug("reg_addr=0x%x, read=0x%x, write=0x%x",
		(u32)info->th_reg_addr, (u32)reg_read, (u32)reg_write);
}

static void set_disp_latency_thresholds(enum tegra_la_id id)
{
	set_thresholds(&disp_info[id - ID(DISPLAY_0A)], id);
}

static void set_vi_latency_thresholds(enum tegra_la_id id)
{
	set_thresholds(&vi_info[id - ID(VI_WSB)], id);
}

/* Sets latency allowance based on clients memory bandwitdh requirement.
 * Bandwidth passed is in mega bytes per second.
 */
int tegra_set_latency_allowance(enum tegra_la_id id,
				unsigned int bandwidth_in_mbps)
{
	int ideal_la;
	int la_to_set;
	unsigned long reg_read;
	unsigned long reg_write;
	int bytes_per_atom = normal_atom_size;
	struct la_client_info *ci;

	VALIDATE_ID(id);
	VALIDATE_BW(bandwidth_in_mbps);
	if (id == ID(FDCDRD) || id == ID(FDCDWR) ||
		id == ID(FDCDRD2) || id == ID(FDCDWR2))
		bytes_per_atom = fdc_atom_size;

	ci = &la_info[id];

	if (bandwidth_in_mbps == 0) {
		la_to_set = MC_LA_MAX_VALUE;
	} else {
		ideal_la = (ci->fifo_size_in_atoms * bytes_per_atom * 1000) /
			   (bandwidth_in_mbps * ns_per_tick);
		la_to_set = ideal_la - (ci->expiration_in_ns/ns_per_tick) - 1;
	}

	la_debug("\n%s:id=%d,bw=%dmbps, la_to_set=%d",
		__func__, id, bandwidth_in_mbps, la_to_set);
	la_to_set = (la_to_set < 0) ? 0 : la_to_set;
	la_to_set = (la_to_set > MC_LA_MAX_VALUE) ? MC_LA_MAX_VALUE : la_to_set;
	scaling_info[id].actual_la_to_set = la_to_set;

	/* until display can use latency allowance scaling, use a more
	 * aggressive LA setting. Bug 862709 */
	if (id >= ID(DISPLAY_0A) && id <= ID(DISPLAY_HCB))
		la_to_set /= 3;

	spin_lock(&safety_lock);
	reg_read = readl(ci->reg_addr);
	reg_write = (reg_read & ~ci->mask) |
			(la_to_set << ci->shift);
	writel(reg_write, ci->reg_addr);
	scaling_info[id].la_set = la_to_set;
	la_debug("reg_addr=0x%x, read=0x%x, write=0x%x",
		(u32)ci->reg_addr, (u32)reg_read, (u32)reg_write);
	spin_unlock(&safety_lock);
	return 0;
}

/* Thresholds for scaling are specified in % of fifo freeness.
 * If threshold_low is specified as 20%, it means when the fifo free
 * between 0 to 20%, use la as programmed_la.
 * If threshold_mid is specified as 50%, it means when the fifo free
 * between 20 to 50%, use la as programmed_la/2 .
 * If threshold_high is specified as 80%, it means when the fifo free
 * between 50 to 80%, use la as programmed_la/4.
 * When the fifo is free between 80 to 100%, use la as 0(highest priority).
 */
int tegra_enable_latency_scaling(enum tegra_la_id id,
				    unsigned int threshold_low,
				    unsigned int threshold_mid,
				    unsigned int threshold_high)
{
	unsigned long reg;
	unsigned long scaling_enable_reg = MC_RA(ARB_OVERRIDE);

	VALIDATE_ID(id);
	VALIDATE_THRESHOLDS(threshold_low, threshold_mid, threshold_high);

	if (la_info[id].scaling_supported == false)
		goto exit;

	spin_lock(&safety_lock);

	la_debug("\n%s: id=%d, tl=%d, tm=%d, th=%d", __func__,
		id, threshold_low, threshold_mid, threshold_high);
	scaling_info[id].threshold_low = threshold_low;
	scaling_info[id].threshold_mid = threshold_mid;
	scaling_info[id].threshold_high = threshold_high;
	scaling_info[id].scaling_ref_count++;

	if (id >= ID(DISPLAY_0A) && id <= ID(DISPLAY_1BB))
		set_disp_latency_thresholds(id);
	else if (id >= ID(VI_WSB) && id <= ID(VI_WY))
		set_vi_latency_thresholds(id);
	if (!la_scaling_enable_count++) {
		reg = readl(scaling_enable_reg);
		reg |= (1 << GLOBAL_LATENCY_SCALING_ENABLE_BIT);
		writel(reg,  scaling_enable_reg);
		la_debug("enabled scaling.");
	}
	spin_unlock(&safety_lock);
exit:
	return 0;
}

void tegra_disable_latency_scaling(enum tegra_la_id id)
{
	unsigned long reg;
	unsigned long scaling_enable_reg = MC_RA(ARB_OVERRIDE);

	if (id >= TEGRA_LA_MAX_ID)
		return;
	BUG_ON(la_info[id].id != id);

	if (la_info[id].scaling_supported == false)
		return;
	spin_lock(&safety_lock);
	la_debug("\n%s: id=%d", __func__, id);
	scaling_info[id].scaling_ref_count--;
	BUG_ON(scaling_info[id].scaling_ref_count < 0);

	if (!--la_scaling_enable_count) {
		reg = readl(scaling_enable_reg);
		reg = reg & ~(1 << GLOBAL_LATENCY_SCALING_ENABLE_BIT);
		writel(reg, scaling_enable_reg);
		la_debug("disabled scaling.");
	}
	spin_unlock(&safety_lock);
}

static int la_regs_show(struct seq_file *s, void *unused)
{
	unsigned i;
	unsigned long la;

	/* iterate the list, but don't print MAX_ID */
	for (i = 0; i < ARRAY_SIZE(la_info) - 1; i++) {
		la = (readl(la_info[i].reg_addr) & la_info[i].mask)
			>> la_info[i].shift;
		seq_printf(s, "%-16s: %4lu\n", la_info[i].name, la);
	}

        return 0;
}

static int dbg_la_regs_open(struct inode *inode, struct file *file)
{
	return single_open(file, la_regs_show, inode->i_private);
}

static const struct file_operations regs_fops = {
	.open           = dbg_la_regs_open,
	.read           = seq_read,
	.llseek         = seq_lseek,
	.release        = single_release,
};

static int __init tegra_latency_allowance_debugfs_init(void)
{
	if (latency_debug_dir)
		return 0;

	latency_debug_dir = debugfs_create_dir("tegra_latency", NULL);

	debugfs_create_file("la_info", S_IRUGO, latency_debug_dir, NULL,
		&regs_fops);

	return 0;
}

late_initcall(tegra_latency_allowance_debugfs_init);

static int __init tegra_latency_allowance_init(void)
{
	la_scaling_enable_count = 0;
	return 0;
}

core_initcall(tegra_latency_allowance_init);

#if TEST_LA_CODE
static int __init test_la(void)
{
	int err;
	enum tegra_la_id id = 0;
	int repeat_count = 5;

	do {
		for (id = 0; id < TEGRA_LA_MAX_ID; id++) {
			err = tegra_set_latency_allowance(id, 200);
			if (err)
				la_debug("\n***tegra_set_latency_allowance,"
					" err=%d", err);
		}

		for (id = 0; id < TEGRA_LA_MAX_ID; id++) {
			if (id >= ID(DISPLAY_0AB) && id <= ID(DISPLAY_HCB))
				continue;
			if (id >= ID(VI_WSB) && id <= ID(VI_WY))
				continue;
			err = tegra_enable_latency_scaling(id, 20, 50, 80);
			if (err)
				la_debug("\n***tegra_enable_latency_scaling,"
					" err=%d", err);
		}

		la_debug("la_scaling_enable_count =%d",
			la_scaling_enable_count);
		for (id = 0; id < TEGRA_LA_MAX_ID; id++) {
			if (id >= ID(DISPLAY_0AB) && id <= ID(DISPLAY_HCB))
				continue;
			if (id >= ID(VI_WSB) && id <= ID(VI_WY))
				continue;
			tegra_disable_latency_scaling(id);
		}
		la_debug("la_scaling_enable_count=%d",
			la_scaling_enable_count);
	} while (--repeat_count);
	return 0;
}

late_initcall(test_la);
#endif
