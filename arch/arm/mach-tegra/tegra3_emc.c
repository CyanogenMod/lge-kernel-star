/*
 * arch/arm/mach-tegra/tegra3_emc.c
 *
 * Copyright (C) 2011 NVIDIA Corporation
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
 *
 */

#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>

#include <asm/cputime.h>

#include <mach/iomap.h>

#include "clock.h"
#include "dvfs.h"
#include "tegra3_emc.h"

#ifdef CONFIG_TEGRA_EMC_SCALING_ENABLE
static bool emc_enable = true;
#else
static bool emc_enable;
#endif
module_param(emc_enable, bool, 0644);

#define EMC_STATUS_UPDATE_TIMEOUT	100
#define TEGRA_EMC_TABLE_MAX_SIZE 	16

enum {
	DLL_CHANGE_NONE = 0,
	DLL_CHANGE_ON,
	DLL_CHANGE_OFF,
};

#define EMC_CLK_DIV_SHIFT		0
#define EMC_CLK_DIV_MASK		(0xFF << EMC_CLK_DIV_SHIFT)
#define EMC_CLK_SOURCE_SHIFT		30
#define EMC_CLK_SOURCE_MASK		(0x3 << EMC_CLK_SOURCE_SHIFT)
#define EMC_CLK_LOW_JITTER_ENABLE	(0x1 << 29)
#define	EMC_CLK_MC_SAME_FREQ		(0x1 << 16)

#define BURST_REG_LIST \
	DEFINE_REG(TEGRA_EMC_BASE, EMC_RC)			\
	DEFINE_REG(TEGRA_EMC_BASE, EMC_RFC)			\
	DEFINE_REG(TEGRA_EMC_BASE, EMC_RAS)			\
	DEFINE_REG(TEGRA_EMC_BASE, EMC_RP)			\
	DEFINE_REG(TEGRA_EMC_BASE, EMC_R2W)			\
	DEFINE_REG(TEGRA_EMC_BASE, EMC_W2R)			\
	DEFINE_REG(TEGRA_EMC_BASE, EMC_R2P)			\
	DEFINE_REG(TEGRA_EMC_BASE, EMC_W2P)			\
	DEFINE_REG(TEGRA_EMC_BASE, EMC_RD_RCD)			\
	DEFINE_REG(TEGRA_EMC_BASE, EMC_WR_RCD)			\
	DEFINE_REG(TEGRA_EMC_BASE, EMC_RRD)			\
	DEFINE_REG(TEGRA_EMC_BASE, EMC_REXT)			\
	DEFINE_REG(TEGRA_EMC_BASE, EMC_WEXT)			\
	DEFINE_REG(TEGRA_EMC_BASE, EMC_WDV)			\
	DEFINE_REG(TEGRA_EMC_BASE, EMC_QUSE)			\
	DEFINE_REG(TEGRA_EMC_BASE, EMC_QRST)			\
	DEFINE_REG(TEGRA_EMC_BASE, EMC_QSAFE)			\
	DEFINE_REG(TEGRA_EMC_BASE, EMC_RDV)			\
	DEFINE_REG(TEGRA_EMC_BASE, EMC_REFRESH)			\
	DEFINE_REG(TEGRA_EMC_BASE, EMC_BURST_REFRESH_NUM)	\
	DEFINE_REG(TEGRA_EMC_BASE, EMC_PRE_REFRESH_REQ_CNT)	\
	DEFINE_REG(TEGRA_EMC_BASE, EMC_PDEX2WR)			\
	DEFINE_REG(TEGRA_EMC_BASE, EMC_PDEX2RD)			\
	DEFINE_REG(TEGRA_EMC_BASE, EMC_PCHG2PDEN)		\
	DEFINE_REG(TEGRA_EMC_BASE, EMC_ACT2PDEN)		\
	DEFINE_REG(TEGRA_EMC_BASE, EMC_AR2PDEN)			\
	DEFINE_REG(TEGRA_EMC_BASE, EMC_RW2PDEN)			\
	DEFINE_REG(TEGRA_EMC_BASE, EMC_TXSR)			\
	DEFINE_REG(TEGRA_EMC_BASE, EMC_TXSRDLL)			\
	DEFINE_REG(TEGRA_EMC_BASE, EMC_TCKE)			\
	DEFINE_REG(TEGRA_EMC_BASE, EMC_TFAW)			\
	DEFINE_REG(TEGRA_EMC_BASE, EMC_TRPAB)			\
	DEFINE_REG(TEGRA_EMC_BASE, EMC_TCLKSTABLE)		\
	DEFINE_REG(TEGRA_EMC_BASE, EMC_TCLKSTOP)		\
	DEFINE_REG(TEGRA_EMC_BASE, EMC_TREFBW)			\
	DEFINE_REG(TEGRA_EMC_BASE, EMC_QUSE_EXTRA)		\
	DEFINE_REG(TEGRA_EMC_BASE, EMC_FBIO_CFG6)		\
	DEFINE_REG(TEGRA_EMC_BASE, EMC_ODT_WRITE)		\
	DEFINE_REG(TEGRA_EMC_BASE, EMC_ODT_READ)		\
	DEFINE_REG(TEGRA_EMC_BASE, EMC_FBIO_CFG5)		\
	DEFINE_REG(TEGRA_EMC_BASE, EMC_CFG_DIG_DLL)		\
	DEFINE_REG(TEGRA_EMC_BASE, EMC_CFG_DIG_DLL_PERIOD)	\
	DEFINE_REG(TEGRA_EMC_BASE, EMC_DLL_XFORM_DQS0)		\
	DEFINE_REG(TEGRA_EMC_BASE, EMC_DLL_XFORM_DQS1)		\
	DEFINE_REG(TEGRA_EMC_BASE, EMC_DLL_XFORM_DQS2)          \
	DEFINE_REG(TEGRA_EMC_BASE, EMC_DLL_XFORM_DQS3)          \
	DEFINE_REG(TEGRA_EMC_BASE, EMC_DLL_XFORM_DQS4)          \
	DEFINE_REG(TEGRA_EMC_BASE, EMC_DLL_XFORM_DQS5)          \
	DEFINE_REG(TEGRA_EMC_BASE, EMC_DLL_XFORM_DQS6)          \
	DEFINE_REG(TEGRA_EMC_BASE, EMC_DLL_XFORM_DQS7)          \
	DEFINE_REG(TEGRA_EMC_BASE, EMC_DLL_XFORM_QUSE0)		\
	DEFINE_REG(TEGRA_EMC_BASE, EMC_DLL_XFORM_QUSE1)		\
	DEFINE_REG(TEGRA_EMC_BASE, EMC_DLL_XFORM_QUSE2)		\
	DEFINE_REG(TEGRA_EMC_BASE, EMC_DLL_XFORM_QUSE3)         \
	DEFINE_REG(TEGRA_EMC_BASE, EMC_DLL_XFORM_QUSE4)         \
	DEFINE_REG(TEGRA_EMC_BASE, EMC_DLL_XFORM_QUSE5)         \
	DEFINE_REG(TEGRA_EMC_BASE, EMC_DLL_XFORM_QUSE6)         \
	DEFINE_REG(TEGRA_EMC_BASE, EMC_DLL_XFORM_QUSE7)         \
	DEFINE_REG(TEGRA_EMC_BASE, EMC_DLI_TRIM_TXDQS0)         \
	DEFINE_REG(TEGRA_EMC_BASE, EMC_DLI_TRIM_TXDQS1)         \
	DEFINE_REG(TEGRA_EMC_BASE, EMC_DLI_TRIM_TXDQS2)         \
	DEFINE_REG(TEGRA_EMC_BASE, EMC_DLI_TRIM_TXDQS3)         \
	DEFINE_REG(TEGRA_EMC_BASE, EMC_DLI_TRIM_TXDQS4)         \
	DEFINE_REG(TEGRA_EMC_BASE, EMC_DLI_TRIM_TXDQS5)         \
	DEFINE_REG(TEGRA_EMC_BASE, EMC_DLI_TRIM_TXDQS6)         \
	DEFINE_REG(TEGRA_EMC_BASE, EMC_DLI_TRIM_TXDQS7)         \
	DEFINE_REG(TEGRA_EMC_BASE, EMC_DLL_XFORM_DQ0)		\
	DEFINE_REG(TEGRA_EMC_BASE, EMC_DLL_XFORM_DQ1)		\
	DEFINE_REG(TEGRA_EMC_BASE, EMC_DLL_XFORM_DQ2)		\
	DEFINE_REG(TEGRA_EMC_BASE, EMC_DLL_XFORM_DQ3)		\
	DEFINE_REG(TEGRA_EMC_BASE, EMC_XM2CMDPADCTRL)		\
	DEFINE_REG(TEGRA_EMC_BASE, EMC_XM2DQSPADCTRL2)		\
	DEFINE_REG(TEGRA_EMC_BASE, EMC_XM2DQPADCTRL2)		\
	DEFINE_REG(TEGRA_EMC_BASE, EMC_XM2CLKPADCTRL)		\
	DEFINE_REG(TEGRA_EMC_BASE, EMC_XM2COMPPADCTRL)		\
	DEFINE_REG(TEGRA_EMC_BASE, EMC_XM2VTTGENPADCTRL)	\
	DEFINE_REG(TEGRA_EMC_BASE, EMC_XM2VTTGENPADCTRL2)	\
	DEFINE_REG(TEGRA_EMC_BASE, EMC_XM2QUSEPADCTRL)		\
	DEFINE_REG(TEGRA_EMC_BASE, EMC_XM2DQSPADCTRL3)		\
	DEFINE_REG(TEGRA_EMC_BASE, EMC_CTT_TERM_CTRL)		\
	DEFINE_REG(TEGRA_EMC_BASE, EMC_ZCAL_INTERVAL)		\
	DEFINE_REG(TEGRA_EMC_BASE, EMC_ZCAL_WAIT_CNT)		\
	DEFINE_REG(TEGRA_EMC_BASE, EMC_MRS_WAIT_CNT)		\
	DEFINE_REG(TEGRA_EMC_BASE, EMC_AUTO_CAL_CONFIG)		\
	DEFINE_REG(TEGRA_EMC_BASE, EMC_CTT)			\
	DEFINE_REG(TEGRA_EMC_BASE, EMC_CTT_DURATION)		\
	DEFINE_REG(TEGRA_EMC_BASE, EMC_DYN_SELF_REF_CONTROL)	\
								\
	DEFINE_REG(TEGRA_MC_BASE, MC_EMEM_ARB_CFG)		\
	DEFINE_REG(TEGRA_MC_BASE, MC_EMEM_ARB_OUTSTANDING_REQ)	\
	DEFINE_REG(TEGRA_MC_BASE, MC_EMEM_ARB_TIMING_RCD)	\
	DEFINE_REG(TEGRA_MC_BASE, MC_EMEM_ARB_TIMING_RP)	\
	DEFINE_REG(TEGRA_MC_BASE, MC_EMEM_ARB_TIMING_RC)	\
	DEFINE_REG(TEGRA_MC_BASE, MC_EMEM_ARB_TIMING_RAS)	\
	DEFINE_REG(TEGRA_MC_BASE, MC_EMEM_ARB_TIMING_FAW)	\
	DEFINE_REG(TEGRA_MC_BASE, MC_EMEM_ARB_TIMING_RRD)	\
	DEFINE_REG(TEGRA_MC_BASE, MC_EMEM_ARB_TIMING_RAP2PRE)	\
	DEFINE_REG(TEGRA_MC_BASE, MC_EMEM_ARB_TIMING_WAP2PRE)	\
	DEFINE_REG(TEGRA_MC_BASE, MC_EMEM_ARB_TIMING_R2R)	\
	DEFINE_REG(TEGRA_MC_BASE, MC_EMEM_ARB_TIMING_W2W)	\
	DEFINE_REG(TEGRA_MC_BASE, MC_EMEM_ARB_TIMING_R2W)	\
	DEFINE_REG(TEGRA_MC_BASE, MC_EMEM_ARB_TIMING_W2R)	\
	DEFINE_REG(TEGRA_MC_BASE, MC_EMEM_ARB_DA_TURNS)		\
	DEFINE_REG(TEGRA_MC_BASE, MC_EMEM_ARB_DA_COVERS)	\
	DEFINE_REG(TEGRA_MC_BASE, MC_EMEM_ARB_MISC0)		\
	DEFINE_REG(TEGRA_MC_BASE, MC_EMEM_ARB_RING1_THROTTLE)

#define DEFINE_REG(base, reg)	((u32)IO_ADDRESS((base)) + (reg)),
static const u32 burst_reg_addr[TEGRA_EMC_NUM_REGS] = {
	BURST_REG_LIST
};
#undef DEFINE_REG

#define DEFINE_REG(base, reg)	reg##_INDEX,
enum {
	BURST_REG_LIST
};
#undef DEFINE_REG

static struct clk_mux_sel tegra_emc_clk_sel[TEGRA_EMC_TABLE_MAX_SIZE];
static int emc_last_sel;
static struct tegra_emc_table start_timing;
static bool emc_timing_in_sync;

static const struct tegra_emc_table *tegra_emc_table;
static int tegra_emc_table_size;

static u32 dram_type;
static u32 dram_dev_num;
static u32 emc_cfg_saved;

static struct clk *emc;

static struct {
	cputime64_t time_at_clock[TEGRA_EMC_TABLE_MAX_SIZE];
	u64 last_update;
	u64 clkchange_count;
	spinlock_t spinlock;
} emc_stats;

static void __iomem *emc_base = IO_ADDRESS(TEGRA_EMC_BASE);
static void __iomem *mc_base = IO_ADDRESS(TEGRA_MC_BASE);
static void __iomem *clk_base = IO_ADDRESS(TEGRA_CLK_RESET_BASE);

static inline void emc_writel(u32 val, unsigned long addr)
{
	writel(val, (u32)emc_base + addr);
	barrier();
}
static inline u32 emc_readl(unsigned long addr)
{
	return readl((u32)emc_base + addr);
}
static inline void mc_writel(u32 val, unsigned long addr)
{
	writel(val, (u32)mc_base + addr);
	barrier();
}
static inline u32 mc_readl(unsigned long addr)
{
	return readl((u32)mc_base + addr);
}

static void emc_last_stats_update(int last_sel)
{
	unsigned long flags;
	u64 cur_jiffies = get_jiffies_64();

	spin_lock_irqsave(&emc_stats.spinlock, flags);

	emc_stats.time_at_clock[emc_last_sel] = cputime64_add(
		emc_stats.time_at_clock[emc_last_sel], cputime64_sub(
			cur_jiffies, emc_stats.last_update));

	emc_stats.last_update = cur_jiffies;

	if (last_sel < TEGRA_EMC_TABLE_MAX_SIZE) {
		emc_stats.clkchange_count++;
		emc_last_sel = last_sel;
	}
	spin_unlock_irqrestore(&emc_stats.spinlock, flags);
}

static int wait_for_update(u32 status_reg, u32 bit_mask, bool updated_state)
{
	int i;
	for (i = 0; i < EMC_STATUS_UPDATE_TIMEOUT; i++) {
		if (!!(emc_readl(status_reg) & bit_mask) == updated_state)
			return 0;
		udelay(1);
	}
	return -ETIMEDOUT;
}

static inline void emc_timing_update(void)
{
	int err;

	emc_writel(0x1, EMC_TIMING_CONTROL);
	err = wait_for_update(EMC_STATUS,
			      EMC_STATUS_TIMING_UPDATE_STALLED, false);
	if (err) {
		pr_err("%s: timing update error: %d", __func__, err);
		BUG();
	}
}

static inline void auto_cal_disable(void)
{
	int err;

	emc_writel(0, EMC_AUTO_CAL_INTERVAL);
	err = wait_for_update(EMC_AUTO_CAL_STATUS,
			      EMC_AUTO_CAL_STATUS_ACTIVE, false);
	if (err) {
		pr_err("%s: disable auto-cal error: %d", __func__, err);
		BUG();
	}
}

static inline bool dqs_preset(const struct tegra_emc_table *next_timing,
			      const struct tegra_emc_table *last_timing)
{
	bool ret = false;

#define DQS_SET(reg, bit)						      \
	do {								      \
		if ((next_timing->burst_regs[EMC_##reg##_INDEX] &	      \
		     EMC_##reg##_##bit##_ENABLE) &&			      \
		    (!(last_timing->burst_regs[EMC_##reg##_INDEX] &	      \
		       EMC_##reg##_##bit##_ENABLE)))   {		      \
			emc_writel(last_timing->burst_regs[EMC_##reg##_INDEX] \
				   | EMC_##reg##_##bit##_ENABLE, EMC_##reg);  \
			ret = true;					      \
		}							      \
	} while (0)

	DQS_SET(XM2DQSPADCTRL2, VREF);
	DQS_SET(XM2DQSPADCTRL3, VREF);
	DQS_SET(XM2QUSEPADCTRL, IVREF);

	return ret;
}

static inline void overwrite_mrs_wait_cnt(
	const struct tegra_emc_table *next_timing,
	bool zcal_long)
{
	u32 reg;
	u32 cnt = 512;

	/* For ddr3 when DLL is re-started: overwrite EMC DFS table settings
	   for MRS_WAIT_LONG with maximum of MRS_WAIT_SHORT settings and
	   expected operation length. Reduce the latter by the overlapping
	   zq-calibration, if any */
	if (zcal_long)
		cnt -= dram_dev_num * 256;

	reg = (next_timing->burst_regs[EMC_MRS_WAIT_CNT_INDEX] &
		EMC_MRS_WAIT_CNT_SHORT_WAIT_MASK) >>
		EMC_MRS_WAIT_CNT_SHORT_WAIT_SHIFT;
	if (cnt < reg)
		cnt = reg;

	reg = (next_timing->burst_regs[EMC_MRS_WAIT_CNT_INDEX] &
		(~EMC_MRS_WAIT_CNT_LONG_WAIT_MASK));
	reg |= (cnt << EMC_MRS_WAIT_CNT_LONG_WAIT_SHIFT) &
		EMC_MRS_WAIT_CNT_LONG_WAIT_MASK;

	emc_writel(reg, EMC_MRS_WAIT_CNT);
}

static inline bool need_qrst(const struct tegra_emc_table *next_timing,
			     const struct tegra_emc_table *last_timing,
			     u32 emc_dpd_reg)
{
	u32 last_mode = (last_timing->burst_regs[EMC_FBIO_CFG5_INDEX] &
		EMC_CFG5_QUSE_MODE_MASK) >> EMC_CFG5_QUSE_MODE_SHIFT;
	u32 next_mode = (next_timing->burst_regs[EMC_FBIO_CFG5_INDEX] &
		EMC_CFG5_QUSE_MODE_MASK) >> EMC_CFG5_QUSE_MODE_SHIFT;

	/* QUSE DPD is disabled */
	bool ret = !(emc_dpd_reg & EMC_SEL_DPD_CTRL_QUSE_DPD_ENABLE) &&

	/* QUSE uses external mode before or after clock change */
		(((last_mode != EMC_CFG5_QUSE_MODE_PULSE_INTERN) &&
		  (last_mode != EMC_CFG5_QUSE_MODE_INTERNAL_LPBK)) ||
		 ((next_mode != EMC_CFG5_QUSE_MODE_PULSE_INTERN) &&
		  (next_mode != EMC_CFG5_QUSE_MODE_INTERNAL_LPBK)))  &&

	/* QUSE pad switches from schmitt to vref mode */
		(((last_timing->burst_regs[EMC_XM2QUSEPADCTRL_INDEX] &
		   EMC_XM2QUSEPADCTRL_IVREF_ENABLE) == 0) &&
		 ((next_timing->burst_regs[EMC_XM2QUSEPADCTRL_INDEX] &
		   EMC_XM2QUSEPADCTRL_IVREF_ENABLE) != 0));

	return ret;
}

static inline void periodic_qrst_enable(u32 emc_cfg_reg, u32 emc_dbg_reg)
{
	/* enable write mux => enable periodic QRST => restore mux */
	emc_writel(emc_dbg_reg | EMC_DBG_WRITE_MUX_ACTIVE, EMC_DBG);
	emc_writel(emc_cfg_reg | EMC_CFG_PERIODIC_QRST, EMC_CFG);
	emc_writel(emc_dbg_reg, EMC_DBG);
}

static inline void periodic_qrst_restore(u32 emc_cfg_reg, u32 emc_dbg_reg)
{
	/* enable write mux => restore periodic QRST => restore mux */
	emc_writel(emc_dbg_reg | EMC_DBG_WRITE_MUX_ACTIVE, EMC_DBG);
	emc_writel(emc_cfg_reg, EMC_CFG);
	emc_writel(emc_dbg_reg, EMC_DBG);
}

static inline int get_dll_change(const struct tegra_emc_table *next_timing,
				 const struct tegra_emc_table *last_timing)
{
	bool next_dll_enabled = !(next_timing->emc_mode_1 & 0x1);
	bool last_dll_enabled = !(last_timing->emc_mode_1 & 0x1);

	if (next_dll_enabled == last_dll_enabled)
		return DLL_CHANGE_NONE;
	else if (next_dll_enabled)
		return DLL_CHANGE_ON;
	else
		return DLL_CHANGE_OFF;
}

static inline void set_dram_mode(const struct tegra_emc_table *next_timing,
				 const struct tegra_emc_table *last_timing,
				 int dll_change)
{
	if (dram_type == DRAM_TYPE_DDR3) {
		/* first mode_1, then mode_2, then mode_reset*/
		if (next_timing->emc_mode_1 != last_timing->emc_mode_1)
			emc_writel(next_timing->emc_mode_1, EMC_EMRS);
		if (next_timing->emc_mode_2 != last_timing->emc_mode_2)
			emc_writel(next_timing->emc_mode_2, EMC_EMRS);

		if ((next_timing->emc_mode_reset !=
		     last_timing->emc_mode_reset) ||
		    (dll_change == DLL_CHANGE_ON))
		{
			u32 reg = next_timing->emc_mode_reset &
				(~EMC_MODE_SET_DLL_RESET);
			if (dll_change == DLL_CHANGE_ON) {
				reg |= EMC_MODE_SET_DLL_RESET;
				reg |= EMC_MODE_SET_LONG_CNT;
			}
			emc_writel(reg, EMC_MRS);
		}
	} else {
		/* first mode_2, then mode_1; mode_reset is not applicable */
		if (next_timing->emc_mode_2 != last_timing->emc_mode_2)
			emc_writel(next_timing->emc_mode_2, EMC_MRW);
		if (next_timing->emc_mode_1 != last_timing->emc_mode_1)
			emc_writel(next_timing->emc_mode_1, EMC_MRW);
	}
}

static inline void do_clock_change(u32 clk_setting)
{
	int err;

	mc_readl(MC_EMEM_ADR_CFG);	/* completes prev writes */
	writel(clk_setting, (u32)clk_base + emc->reg);

	err = wait_for_update(EMC_INTSTATUS,
			      EMC_INTSTATUS_CLKCHANGE_COMPLETE, true);
	if (err) {
		pr_err("%s: clock change completion error: %d", __func__, err);
		BUG();
	}
}

static noinline void emc_set_clock(const struct tegra_emc_table *next_timing,
				   const struct tegra_emc_table *last_timing,
				   u32 clk_setting)
{
	int i, dll_change, pre_wait;
	bool dyn_sref_enabled, vref_cal_toggle, qrst_used, zcal_long;

	u32 emc_cfg_reg = emc_readl(EMC_CFG);
	u32 emc_dbg_reg = emc_readl(EMC_DBG);

	dyn_sref_enabled = emc_cfg_reg & EMC_CFG_DYN_SREF_ENABLE;
	dll_change = get_dll_change(next_timing, last_timing);
	zcal_long = (next_timing->burst_regs[EMC_ZCAL_INTERVAL_INDEX] != 0) &&
		(last_timing->burst_regs[EMC_ZCAL_INTERVAL_INDEX] == 0);

	/* FIXME: remove steps enumeration below? */

	/* 1. clear clkchange_complete interrupts */
	emc_writel(EMC_INTSTATUS_CLKCHANGE_COMPLETE, EMC_INTSTATUS);

	/* 2. disable dynamic self-refresh and preset dqs vref, then wait for
	   possible self-refresh entry/exit and/or dqs vref settled - waiting
	   before the clock change decreases worst case change stall time */
	pre_wait = 0;
	if (dyn_sref_enabled) {
		emc_cfg_reg &= ~EMC_CFG_DYN_SREF_ENABLE;
		emc_writel(emc_cfg_reg, EMC_CFG);
		pre_wait = 5;		/* 5us+ for self-refresh entry/exit */
	}
	if (dqs_preset(next_timing, last_timing)) {
		if (pre_wait < 3)
			pre_wait = 3;	/* 3us+ for dqs vref settled */
	}
	if (pre_wait) {
		emc_timing_update();
		udelay(pre_wait);
	}

	/* 3. disable auto-cal if vref mode is switching */
	vref_cal_toggle = (next_timing->emc_acal_interval != 0) &&
		((next_timing->burst_regs[EMC_XM2COMPPADCTRL_INDEX] ^
		  last_timing->burst_regs[EMC_XM2COMPPADCTRL_INDEX]) &
		 EMC_XM2COMPPADCTRL_VREF_CAL_ENABLE);
	if (vref_cal_toggle)
		auto_cal_disable();

	/* 4. program burst shadow registers */
	for (i = 0; i < TEGRA_EMC_NUM_REGS; i++)
		__raw_writel(next_timing->burst_regs[i], burst_reg_addr[i]);
	wmb();
	barrier();

	/* On ddr3 when DLL is re-started predict MRS long wait count and
	   overwrite DFS table setting */
	if ((dram_type == DRAM_TYPE_DDR3) && (dll_change == DLL_CHANGE_ON))
		overwrite_mrs_wait_cnt(next_timing, zcal_long);

	/* the last read below makes sure prev writes are completed */
	qrst_used = need_qrst(next_timing, last_timing,
			      emc_readl(EMC_SEL_DPD_CTRL));

	/* 5. flow control marker 1 (no EMC read access after this) */
	emc_writel(1, EMC_STALL_BEFORE_CLKCHANGE);

	/* 6. enable periodic QRST */
	if (qrst_used)
		periodic_qrst_enable(emc_cfg_reg, emc_dbg_reg);

	/* 6.1 disable auto-refresh to save time after clock change */
	emc_writel(EMC_REFCTRL_DISABLE_ALL(dram_dev_num), EMC_REFCTRL);

	/* 7. turn Off dll and enter self-refresh on DDR3 */
	if (dram_type == DRAM_TYPE_DDR3) {
		if (dll_change == DLL_CHANGE_OFF)
			emc_writel(next_timing->emc_mode_1, EMC_EMRS);
		emc_writel(DRAM_BROADCAST(dram_dev_num) |
			   EMC_SELF_REF_CMD_ENABLED, EMC_SELF_REF);
	}

	/* 8. flow control marker 2 */
	emc_writel(1, EMC_STALL_AFTER_CLKCHANGE);

	/* 9. exit self-refresh on DDR3 */
	if (dram_type == DRAM_TYPE_DDR3)
		emc_writel(DRAM_BROADCAST(dram_dev_num), EMC_SELF_REF);

	/* 10. restore periodic QRST */
	if ((qrst_used) || (next_timing->emc_periodic_qrst !=
			    last_timing->emc_periodic_qrst)) {
		emc_cfg_reg = next_timing->emc_periodic_qrst ?
			emc_cfg_reg | EMC_CFG_PERIODIC_QRST :
			emc_cfg_reg & (~EMC_CFG_PERIODIC_QRST);
		periodic_qrst_restore(emc_cfg_reg, emc_dbg_reg);
	}

	/* 11. set dram mode registers */
	set_dram_mode(next_timing, last_timing, dll_change);

	/* 12. issue zcal command if turning zcal On */
	if (zcal_long) {
		emc_writel(EMC_ZQ_CAL_LONG_CMD_DEV0, EMC_ZQ_CAL);
		if (dram_dev_num > 1)
			emc_writel(EMC_ZQ_CAL_LONG_CMD_DEV1, EMC_ZQ_CAL);
	}

	/* 13. flow control marker 3 */
	emc_writel(1, EMC_UNSTALL_RW_AFTER_CLKCHANGE);

	/* 14. read any MC register to ensure the programming is done
	       change EMC clock source register (EMC read access restored)
	       wait for clk change completion */
	do_clock_change(clk_setting);

	/* 14.1 re-enable auto-refresh */
	emc_writel(EMC_REFCTRL_ENABLE_ALL(dram_dev_num), EMC_REFCTRL);

	/* 15. restore auto-cal */
	if (vref_cal_toggle)
		emc_writel(next_timing->emc_acal_interval,
			   EMC_AUTO_CAL_INTERVAL);

	/* 16. restore dynamic self-refresh */
	if (dyn_sref_enabled) {
		emc_cfg_reg |= EMC_CFG_DYN_SREF_ENABLE;
		emc_writel(emc_cfg_reg, EMC_CFG);
	}

	/* 17. set zcal wait count */
	if (zcal_long)
		emc_writel(next_timing->emc_zcal_cnt_long, EMC_ZCAL_WAIT_CNT);

	/* 18. update restored timing */
	udelay(2);
	emc_timing_update();
}

static inline void emc_get_timing(struct tegra_emc_table *timing)
{
	int i;

	for (i = 0; i < TEGRA_EMC_NUM_REGS; i++)
		timing->burst_regs[i] = __raw_readl(burst_reg_addr[i]);
	timing->emc_acal_interval = 0;
	timing->emc_zcal_cnt_long = 0;
	timing->emc_mode_reset = 0;
	timing->emc_mode_1 = 0;
	timing->emc_mode_2 = 0;
	timing->emc_periodic_qrst = (emc_readl(EMC_CFG) &
				     EMC_CFG_PERIODIC_QRST) ? 1 : 0;
}

/* After deep sleep EMC power features are not restored.
 * Do it at run-time after the 1st clock change.
 */
static inline void emc_cfg_power_restore(void)
{
	u32 reg = emc_readl(EMC_CFG);
	if ((reg ^ emc_cfg_saved) & EMC_CFG_PWR_MASK) {
		reg = (reg & (~EMC_CFG_PWR_MASK)) |
			(emc_cfg_saved & EMC_CFG_PWR_MASK);
		emc_writel(reg, EMC_CFG);
		emc_timing_update();
	}
}

/* The EMC registers have shadow registers. When the EMC clock is updated
 * in the clock controller, the shadow registers are copied to the active
 * registers, allowing glitchless memory bus frequency changes.
 * This function updates the shadow registers for a new clock frequency,
 * and relies on the clock lock on the emc clock to avoid races between
 * multiple frequency changes */
int tegra_emc_set_rate(unsigned long rate)
{
	int i;
	u32 clk_setting;
	const struct tegra_emc_table *last_timing;

	if (!tegra_emc_table)
		return -EINVAL;

	/* Table entries specify rate in kHz */
	rate = rate / 1000;

	for (i = 0; i < tegra_emc_table_size; i++) {
		if (tegra_emc_clk_sel[i].input == NULL)
			continue;	/* invalid entry */

		if (tegra_emc_table[i].rate == rate)
			break;
	}

	if (i >= tegra_emc_table_size)
		return -EINVAL;

	if (!emc_timing_in_sync) {
		/* can not assume that boot timing matches dfs table even
		   if boot frequency matches one of the table nodes */
		emc_get_timing(&start_timing);
		last_timing = &start_timing;
	}
	else
		last_timing = &tegra_emc_table[emc_last_sel];

	clk_setting = tegra_emc_clk_sel[i].value;
	emc_set_clock(&tegra_emc_table[i], last_timing, clk_setting);
	if (!emc_timing_in_sync)
		emc_cfg_power_restore();
	emc_timing_in_sync = true;
	emc_last_stats_update(i);

	pr_debug("%s: rate %lu setting 0x%x\n", __func__, rate, clk_setting);

	return 0;
}

/* Select the closest EMC rate that is higher than the requested rate */
long tegra_emc_round_rate(unsigned long rate)
{
	int i;
	int best = -1;
	unsigned long distance = ULONG_MAX;

	if (!tegra_emc_table)
		return -EINVAL;

	if (!emc_enable)
		return -EINVAL;

	pr_debug("%s: %lu\n", __func__, rate);

	/* Table entries specify rate in kHz */
	rate = rate / 1000;

	for (i = 0; i < tegra_emc_table_size; i++) {
		if (tegra_emc_clk_sel[i].input == NULL)
			continue;	/* invalid entry */

		if (tegra_emc_table[i].rate >= rate &&
		    (tegra_emc_table[i].rate - rate) < distance) {
			distance = tegra_emc_table[i].rate - rate;
			best = i;
		}
	}

	if (best < 0)
		return -EINVAL;

	pr_debug("%s: using %lu\n", __func__, tegra_emc_table[best].rate);

	return tegra_emc_table[best].rate * 1000;
}

struct clk *tegra_emc_predict_parent(unsigned long rate, u32 *div_value)
{
	int i;

	if (!tegra_emc_table)
		return NULL;

	pr_debug("%s: %lu\n", __func__, rate);

	/* Table entries specify rate in kHz */
	rate = rate / 1000;

	for (i = 0; i < tegra_emc_table_size; i++) {
		if (tegra_emc_table[i].rate == rate) {
			*div_value = (tegra_emc_clk_sel[i].value &
				EMC_CLK_DIV_MASK) >> EMC_CLK_DIV_SHIFT;
			return tegra_emc_clk_sel[i].input;
		}
	}

	return NULL;
}

static const struct clk_mux_sel *find_matching_input(
	unsigned long table_rate,
	u32 *div_value)
{
	unsigned long inp_rate;
	const struct clk_mux_sel *sel;

	for (sel = emc->inputs; sel->input != NULL; sel++) {
		/* Table entries specify rate in kHz */
		inp_rate = clk_get_rate(sel->input) / 1000;

		if ((inp_rate >= table_rate) &&
		     (inp_rate % table_rate == 0)) {
			*div_value = 2 * inp_rate / table_rate - 2;
			return sel;
		}
	}
	return NULL;
}

static bool is_emc_bridge(void)
{
	int mv;
	unsigned long rate;
	struct clk *bridge = tegra_get_clock_by_name("bridge.emc");
	BUG_ON(!bridge);

	/* LPDDR2 does not need a bridge entry in DFS table: just lock bridge
	   rate at minimum so it won't interfere with emc bus operations */
	if (dram_type == DRAM_TYPE_LPDDR2) {
		clk_set_rate(bridge, 0);
		return true;
	}

	/* DDR3 requires EMC DFS table to include a bridge entry with frequency
	   above minimum bridge threshold, and voltage below bridge threshold */
	rate = clk_round_rate(bridge, TEGRA_EMC_BRIDGE_RATE_MIN);
	if (IS_ERR_VALUE(rate))
		return false;

	mv = tegra_dvfs_predict_millivolts(emc, rate);
	if (IS_ERR_VALUE(mv) || (mv > TEGRA_EMC_BRIDGE_MVOLTS_MIN))
		return false;

	if (clk_set_rate(bridge, rate))
		return false;

	return true;
}

void tegra_init_emc(const struct tegra_emc_table *table, int table_size)
{
	int i;
	u32 reg, div_value;
	bool max_entry = false;
	unsigned long boot_rate, max_rate;
	const struct clk_mux_sel *sel;

	emc_cfg_saved = emc_readl(EMC_CFG);

	emc_stats.clkchange_count = 0;
	spin_lock_init(&emc_stats.spinlock);
	emc_stats.last_update = get_jiffies_64();

	emc = tegra_get_clock_by_name("emc");
	BUG_ON(!emc);
	boot_rate = clk_get_rate(emc) / 1000;
	max_rate = clk_get_max_rate(emc) / 1000;

	if (emc->parent != tegra_get_clock_by_name("pll_m")) {
		pr_warn("tegra: boot parent %s is not supported by EMC DFS\n",
			emc->parent->name);
		return;
	}

	tegra_emc_table_size = min(table_size, TEGRA_EMC_TABLE_MAX_SIZE);
	for (i = 0; i < tegra_emc_table_size; i++) {
		unsigned long table_rate = table[i].rate;
		if (!table_rate)
			continue;

		sel = find_matching_input(table_rate, &div_value);
		if (!sel)
			continue;

		if (table_rate == boot_rate)
			emc_last_sel = i;

		if (table_rate == max_rate)
			max_entry = true;

		tegra_emc_clk_sel[i] = *sel;
		BUG_ON(div_value >
		       (EMC_CLK_DIV_MASK >> EMC_CLK_DIV_SHIFT));
		tegra_emc_clk_sel[i].value <<= EMC_CLK_SOURCE_SHIFT;
		tegra_emc_clk_sel[i].value |= (div_value << EMC_CLK_DIV_SHIFT);

		if ((div_value == 0) &&
		    (tegra_emc_clk_sel[i].input == emc->parent)) {
			tegra_emc_clk_sel[i].value |= EMC_CLK_LOW_JITTER_ENABLE;
		}

		if (table[i].burst_regs[MC_EMEM_ARB_MISC0_INDEX] &
		    MC_EMEM_ARB_MISC0_EMC_SAME_FREQ)
			tegra_emc_clk_sel[i].value |= EMC_CLK_MC_SAME_FREQ;
	}

	dram_type = (emc_readl(EMC_FBIO_CFG5) &
		     EMC_CFG5_TYPE_MASK) >> EMC_CFG5_TYPE_SHIFT;
	if ((dram_type != DRAM_TYPE_DDR3) && (dram_type != DRAM_TYPE_LPDDR2)) {
		pr_err("Not supported DRAM type %u\n", dram_type);
		return;
	}
	reg = emc_readl(EMC_CFG_2) & (~EMC_CFG_2_MODE_MASK);
	reg |= ((dram_type == DRAM_TYPE_LPDDR2) ? EMC_CFG_2_PD_MODE :
		EMC_CFG_2_SREF_MODE) << EMC_CFG_2_MODE_SHIFT;
	emc_writel(reg, EMC_CFG_2);

	dram_dev_num = (mc_readl(MC_EMEM_ADR_CFG) & 0x1) + 1; /* 2 dev max */

	if (!max_entry) {
		pr_err("tegra: invalid EMC DFS table: entry for max rate"
		       " %lu kHz is not found\n", max_rate);
		return;
	}

	tegra_emc_table = table;

	if (!is_emc_bridge()) {
		tegra_emc_table = NULL;
		pr_err("tegra: invalid EMC DFS table: emc bridge not found");
	}
	pr_info("tegra: validated EMC DFS table\n");
}

void tegra_emc_timing_invalidate(void)
{
	emc_timing_in_sync = false;
}

#ifdef CONFIG_DEBUG_FS

static struct dentry *emc_debugfs_root;

static int emc_stats_show(struct seq_file *s, void *data)
{
	int i;

	emc_last_stats_update(TEGRA_EMC_TABLE_MAX_SIZE);

	seq_printf(s, "%-10s %-10s \n", "rate kHz", "time");
	for (i = 0; i < tegra_emc_table_size; i++) {
		if (tegra_emc_clk_sel[i].input == NULL)
			continue;	/* invalid entry */

		seq_printf(s, "%-10lu %-10llu \n", tegra_emc_table[i].rate,
			   cputime64_to_clock_t(emc_stats.time_at_clock[i]));
	}
	seq_printf(s, "%-15s %llu\n", "transitions:",
		   emc_stats.clkchange_count);
	seq_printf(s, "%-15s %llu\n", "time-stamp:",
		   cputime64_to_clock_t(emc_stats.last_update));

	return 0;
}

static int emc_stats_open(struct inode *inode, struct file *file)
{
	return single_open(file, emc_stats_show, inode->i_private);
}

static const struct file_operations emc_stats_fops = {
	.open		= emc_stats_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int __init tegra_emc_debug_init(void)
{
	if (!tegra_emc_table)
		return 0;

	emc_debugfs_root = debugfs_create_dir("tegra_emc", NULL);
	if (!emc_debugfs_root)
		return -ENOMEM;

	if (!debugfs_create_file(
		"stats", S_IRUGO, emc_debugfs_root, NULL, &emc_stats_fops))
		goto err_out;

	return 0;

err_out:
	debugfs_remove_recursive(emc_debugfs_root);
	return -ENOMEM;
}

late_initcall(tegra_emc_debug_init);
#endif
