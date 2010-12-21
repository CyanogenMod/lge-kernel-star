/*
 * arch/arm/mach-tegra/tegra3_clocks.c
 *
 * Copyright (C) 2010 NVIDIA Corporation
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
#include <linux/module.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/clk.h>

#include <asm/clkdev.h>

#include <mach/iomap.h>

#include "clock.h"
#include "fuse.h"
#include "pm.h"

#define RST_DEVICES_L			0x004
#define RST_DEVICES_H			0x008
#define RST_DEVICES_U			0x00C
#define RST_DEVICES_V			0x358
#define RST_DEVICES_W			0x35C
#define RST_DEVICES_SET_L		0x300
#define RST_DEVICES_CLR_L		0x304
#define RST_DEVICES_SET_V		0x430
#define RST_DEVICES_CLR_V		0x434
#define RST_DEVICES_NUM			5

#define CLK_OUT_ENB_L			0x010
#define CLK_OUT_ENB_H			0x014
#define CLK_OUT_ENB_U			0x018
#define CLK_OUT_ENB_V			0x360
#define CLK_OUT_ENB_W			0x364
#define CLK_OUT_ENB_SET_L		0x320
#define CLK_OUT_ENB_CLR_L		0x324
#define CLK_OUT_ENB_SET_V		0x440
#define CLK_OUT_ENB_CLR_V		0x444
#define CLK_OUT_ENB_NUM			5

#define PERIPH_CLK_TO_BIT(c)		(1 << (c->u.periph.clk_num % 32))
#define PERIPH_CLK_TO_RST_REG(c)	\
	periph_clk_to_reg((c), RST_DEVICES_L, RST_DEVICES_V, 4)
#define PERIPH_CLK_TO_RST_SET_REG(c)	\
	periph_clk_to_reg((c), RST_DEVICES_SET_L, RST_DEVICES_SET_V, 8)
#define PERIPH_CLK_TO_RST_CLR_REG(c)	\
	periph_clk_to_reg((c), RST_DEVICES_CLR_L, RST_DEVICES_CLR_V, 8)

#define PERIPH_CLK_TO_ENB_REG(c)	\
	periph_clk_to_reg((c), CLK_OUT_ENB_L, CLK_OUT_ENB_V, 4)
#define PERIPH_CLK_TO_ENB_SET_REG(c)	\
	periph_clk_to_reg((c), CLK_OUT_ENB_SET_L, CLK_OUT_ENB_SET_V, 8)
#define PERIPH_CLK_TO_ENB_CLR_REG(c)	\
	periph_clk_to_reg((c), CLK_OUT_ENB_CLR_L, CLK_OUT_ENB_CLR_V, 8)

#define CLK_MASK_ARM			0x44
#define MISC_CLK_ENB			0x48

#define OSC_CTRL			0x50
#define OSC_CTRL_OSC_FREQ_MASK		(0xF<<28)
#define OSC_CTRL_OSC_FREQ_13MHZ		(0x0<<28)
#define OSC_CTRL_OSC_FREQ_19_2MHZ	(0x4<<28)
#define OSC_CTRL_OSC_FREQ_12MHZ		(0x8<<28)
#define OSC_CTRL_OSC_FREQ_26MHZ		(0xC<<28)
#define OSC_CTRL_OSC_FREQ_16_8MHZ	(0x1<<28)
#define OSC_CTRL_OSC_FREQ_38_4MHZ	(0x5<<28)
#define OSC_CTRL_OSC_FREQ_48MHZ		(0x9<<28)
#define OSC_CTRL_MASK			(0x3f2 | OSC_CTRL_OSC_FREQ_MASK)

#define OSC_CTRL_PLL_REF_DIV_MASK	(3<<26)
#define OSC_CTRL_PLL_REF_DIV_1		(0<<26)
#define OSC_CTRL_PLL_REF_DIV_2		(1<<26)
#define OSC_CTRL_PLL_REF_DIV_4		(2<<26)

#define OSC_FREQ_DET			0x58
#define OSC_FREQ_DET_TRIG		(1<<31)

#define OSC_FREQ_DET_STATUS		0x5C
#define OSC_FREQ_DET_BUSY		(1<<31)
#define OSC_FREQ_DET_CNT_MASK		0xFFFF

#define PERIPH_CLK_SOURCE_I2S1		0x100
#define PERIPH_CLK_SOURCE_EMC		0x19c
#define PERIPH_CLK_SOURCE_OSC		0x1fc
#define PERIPH_CLK_SOURCE_NUM1 \
	((PERIPH_CLK_SOURCE_OSC - PERIPH_CLK_SOURCE_I2S1) / 4)

#define PERIPH_CLK_SOURCE_G3D2		0x3b0
#define PERIPH_CLK_SOURCE_SE		0x42c
#define PERIPH_CLK_SOURCE_NUM2 \
	((PERIPH_CLK_SOURCE_G3D2 - PERIPH_CLK_SOURCE_SE) / 4)

#define PERIPH_CLK_SOURCE_NUM		(PERIPH_CLK_SOURCE_NUM1 + \
					 PERIPH_CLK_SOURCE_NUM2)

#define PERIPH_CLK_SOURCE_MASK(c)	\
	(((c)->flags & MUX8) ? (7<<29) :\
		(((c)->flags & MUX_PWM) ? (3 << 28) : (3<<30)))
#define PERIPH_CLK_SOURCE_SHIFT(c)	\
	(((c)->flags & MUX8) ? 29 : (((c)->flags & MUX_PWM) ? 28 : 30))
#define PERIPH_CLK_SOURCE_DIVU71_MASK	0xFF
#define PERIPH_CLK_SOURCE_DIVU16_MASK	0xFFFF
#define PERIPH_CLK_SOURCE_DIV_SHIFT	0

#define AUDIO_SYNC_SOURCE_MASK		0x0F
#define AUDIO_SYNC_DISABLE_BIT		0x10
#define AUDIO_SYNC_TAP_NIBBLE_SHIFT(c)	((c->reg_shift - 24) * 4)

#define PLL_BASE			0x0
#define PLL_BASE_BYPASS			(1<<31)
#define PLL_BASE_ENABLE			(1<<30)
#define PLL_BASE_REF_ENABLE		(1<<29)
#define PLL_BASE_OVERRIDE		(1<<28)
#define PLL_BASE_LOCK			(1<<27)
#define PLL_BASE_DIVP_MASK		(0x7<<20)
#define PLL_BASE_DIVP_SHIFT		20
#define PLL_BASE_DIVN_MASK		(0x3FF<<8)
#define PLL_BASE_DIVN_SHIFT		8
#define PLL_BASE_DIVM_MASK		(0x1F)
#define PLL_BASE_DIVM_SHIFT		0

#define PLL_OUT_RATIO_MASK		(0xFF<<8)
#define PLL_OUT_RATIO_SHIFT		8
#define PLL_OUT_OVERRIDE		(1<<2)
#define PLL_OUT_CLKEN			(1<<1)
#define PLL_OUT_RESET_DISABLE		(1<<0)

#define PLL_MISC(c)			(((c)->flags & PLL_ALT_MISC_REG) ? 0x4 : 0xc)
#define PLL_MISC_LOCK_ENABLE(c)		\
	(((c)->flags & (PLLU | PLLD)) ? (1<<22) : (1<<18))

#define PLL_MISC_DCCON_SHIFT		20
#define PLL_MISC_CPCON_SHIFT		8
#define PLL_MISC_CPCON_MASK		(0xF<<PLL_MISC_CPCON_SHIFT)
#define PLL_MISC_LFCON_SHIFT		4
#define PLL_MISC_LFCON_MASK		(0xF<<PLL_MISC_LFCON_SHIFT)
#define PLL_MISC_VCOCON_SHIFT		0
#define PLL_MISC_VCOCON_MASK		(0xF<<PLL_MISC_VCOCON_SHIFT)

#define PLLU_BASE_POST_DIV		(1<<20)

#define PLLD_MISC_CLKENABLE		(1<<30)
#define PLLD_MISC_DIV_RST		(1<<23)
#define PLLD_MISC_DCCON_SHIFT		12

#define PLLDU_LFCON_SET_DIVN		600

 /* FIXME: OUT_OF_TABLE_CPCON per pll */
 #define OUT_OF_TABLE_CPCON		0x8

#define SUPER_CLK_MUX			0x00
#define SUPER_STATE_SHIFT		28
#define SUPER_STATE_MASK		(0xF << SUPER_STATE_SHIFT)
#define SUPER_STATE_STANDBY		(0x0 << SUPER_STATE_SHIFT)
#define SUPER_STATE_IDLE		(0x1 << SUPER_STATE_SHIFT)
#define SUPER_STATE_RUN			(0x2 << SUPER_STATE_SHIFT)
#define SUPER_STATE_IRQ			(0x3 << SUPER_STATE_SHIFT)
#define SUPER_STATE_FIQ			(0x4 << SUPER_STATE_SHIFT)
#define SUPER_SOURCE_MASK		0xF
#define	SUPER_FIQ_SOURCE_SHIFT		12
#define	SUPER_IRQ_SOURCE_SHIFT		8
#define	SUPER_RUN_SOURCE_SHIFT		4
#define	SUPER_IDLE_SOURCE_SHIFT		0

#define SUPER_CLK_DIVIDER		0x04
#define IS_PLLX_DIV2_BYPASS		(clk_readl(0x370) & (1<<16))
/* FIXME: replace with global is_lp_cluster() ? */
#define IS_LP_CLUSTER			(flow_readl(0x2c) & 1)

#define BUS_CLK_DISABLE			(1<<3)
#define BUS_CLK_DIV_MASK		0x3

#define PMC_CTRL			0x0
 #define PMC_CTRL_BLINK_ENB		(1 << 7)

#define PMC_DPD_PADS_ORIDE		0x1c
 #define PMC_DPD_PADS_ORIDE_BLINK_ENB	(1 << 20)

#define PMC_BLINK_TIMER_DATA_ON_SHIFT	0
#define PMC_BLINK_TIMER_DATA_ON_MASK	0x7fff
#define PMC_BLINK_TIMER_ENB		(1 << 15)
#define PMC_BLINK_TIMER_DATA_OFF_SHIFT	16
#define PMC_BLINK_TIMER_DATA_OFF_MASK	0xffff

static void __iomem *reg_clk_base = IO_ADDRESS(TEGRA_CLK_RESET_BASE);
static void __iomem *reg_pmc_base = IO_ADDRESS(TEGRA_PMC_BASE);
static void __iomem *reg_flow_base = IO_ADDRESS(TEGRA_FLOW_CTRL_BASE);

/*
 * Some peripheral clocks share an enable bit, so refcount the enable bits
 * in registers CLK_ENABLE_L, ... CLK_ENABLE_W
 */
static int tegra_periph_clk_enable_refcount[CLK_OUT_ENB_NUM * 32];

#define clk_writel(value, reg) \
	__raw_writel(value, (u32)reg_clk_base + (reg))
#define clk_readl(reg) \
	__raw_readl((u32)reg_clk_base + (reg))
#define pmc_writel(value, reg) \
	__raw_writel(value, (u32)reg_pmc_base + (reg))
#define pmc_readl(reg) \
	__raw_readl((u32)reg_pmc_base + (reg))
#define flow_readl(reg) \
	__raw_readl((u32)reg_flow_base + (reg))

static inline u32 periph_clk_to_reg(
	struct clk *c, u32 reg_L, u32 reg_V, int offs)
{
	u32 reg = c->u.periph.clk_num / 32;
	BUG_ON(reg >= RST_DEVICES_NUM);
	if (reg < 3) {
		reg = reg_L + (reg * offs);
	} else {
		reg = reg_V + ((reg - 3) * offs);
	}
	return reg;
}

unsigned long clk_measure_input_freq(void)
{
	u32 clock_autodetect;
	clk_writel(OSC_FREQ_DET_TRIG | 1, OSC_FREQ_DET);
	do {} while (clk_readl(OSC_FREQ_DET_STATUS) & OSC_FREQ_DET_BUSY);
	clock_autodetect = clk_readl(OSC_FREQ_DET_STATUS);
	if (clock_autodetect >= 732 - 3 && clock_autodetect <= 732 + 3) {
		return 12000000;
	} else if (clock_autodetect >= 794 - 3 && clock_autodetect <= 794 + 3) {
		return 13000000;
	} else if (clock_autodetect >= 1172 - 3 && clock_autodetect <= 1172 + 3) {
		return 19200000;
	} else if (clock_autodetect >= 1587 - 3 && clock_autodetect <= 1587 + 3) {
		return 26000000;
	} else if (clock_autodetect >= 1025 - 3 && clock_autodetect <= 1025 + 3) {
		return 16800000;
	} else if (clock_autodetect >= 2344 - 3 && clock_autodetect <= 2344 + 3) {
		return 38400000;
	} else if (clock_autodetect >= 2928 - 3 && clock_autodetect <= 2928 + 3) {
		return 48000000;
	} else {
		pr_err("%s: Unexpected clock autodetect value %d", __func__, clock_autodetect);
		BUG();
		return 0;
	}
}

static int clk_div71_get_divider(unsigned long parent_rate, unsigned long rate)
{
	s64 divider_u71 = parent_rate * 2;
	divider_u71 += rate - 1;
	do_div(divider_u71, rate);

	if (divider_u71 - 2 < 0)
		return 0;

	if (divider_u71 - 2 > 255)
		return -EINVAL;

	return divider_u71 - 2;
}

static int clk_div16_get_divider(unsigned long parent_rate, unsigned long rate)
{
	s64 divider_u16;

	divider_u16 = parent_rate;
	divider_u16 += rate - 1;
	do_div(divider_u16, rate);

	if (divider_u16 - 1 < 0)
		return 0;

	if (divider_u16 - 1 > 255)
		return -EINVAL;

	return divider_u16 - 1;
}

/* clk_m functions */
static unsigned long tegra3_clk_m_autodetect_rate(struct clk *c)
{
	u32 osc_ctrl = clk_readl(OSC_CTRL);
	u32 auto_clock_control = osc_ctrl & ~OSC_CTRL_OSC_FREQ_MASK;
	u32 pll_ref_div = osc_ctrl & OSC_CTRL_PLL_REF_DIV_MASK;

	c->rate = clk_measure_input_freq();
	switch (c->rate) {
	case 12000000:
		auto_clock_control |= OSC_CTRL_OSC_FREQ_12MHZ;
		BUG_ON(pll_ref_div != OSC_CTRL_PLL_REF_DIV_1);
		break;
	case 13000000:
		auto_clock_control |= OSC_CTRL_OSC_FREQ_13MHZ;
		BUG_ON(pll_ref_div != OSC_CTRL_PLL_REF_DIV_1);
		break;
	case 19200000:
		auto_clock_control |= OSC_CTRL_OSC_FREQ_19_2MHZ;
		BUG_ON(pll_ref_div != OSC_CTRL_PLL_REF_DIV_1);
		break;
	case 26000000:
		auto_clock_control |= OSC_CTRL_OSC_FREQ_26MHZ;
		BUG_ON(pll_ref_div != OSC_CTRL_PLL_REF_DIV_1);
		break;
	case 16800000:
		auto_clock_control |= OSC_CTRL_OSC_FREQ_16_8MHZ;
		BUG_ON(pll_ref_div != OSC_CTRL_PLL_REF_DIV_1);
		break;
	case 38400000:
		auto_clock_control |= OSC_CTRL_OSC_FREQ_38_4MHZ;
		BUG_ON(pll_ref_div != OSC_CTRL_PLL_REF_DIV_2);
		break;
	case 48000000:
		auto_clock_control |= OSC_CTRL_OSC_FREQ_48MHZ;
		BUG_ON(pll_ref_div != OSC_CTRL_PLL_REF_DIV_4);
		break;
	default:
		pr_err("%s: Unexpected clock rate %ld", __func__, c->rate);
		BUG();
	}
	clk_writel(auto_clock_control, OSC_CTRL);
	return c->rate;
}

static void tegra3_clk_m_init(struct clk *c)
{
	pr_debug("%s on clock %s\n", __func__, c->name);
	tegra3_clk_m_autodetect_rate(c);
}

static int tegra3_clk_m_enable(struct clk *c)
{
	pr_debug("%s on clock %s\n", __func__, c->name);
	return 0;
}

static void tegra3_clk_m_disable(struct clk *c)
{
	pr_debug("%s on clock %s\n", __func__, c->name);
	BUG();
}

static struct clk_ops tegra_clk_m_ops = {
	.init		= tegra3_clk_m_init,
	.enable		= tegra3_clk_m_enable,
	.disable	= tegra3_clk_m_disable,
};

/* PLL reference divider functions */
static void tegra3_pll_ref_init(struct clk *c)
{
	u32 pll_ref_div = clk_readl(OSC_CTRL) & OSC_CTRL_PLL_REF_DIV_MASK;
	pr_debug("%s on clock %s\n", __func__, c->name);

	switch (pll_ref_div) {
	case OSC_CTRL_PLL_REF_DIV_1:
		c->div = 1;
		break;
	case OSC_CTRL_PLL_REF_DIV_2:
		c->div = 2;
		break;
	case OSC_CTRL_PLL_REF_DIV_4:
		c->div = 4;
		break;
	default:
		pr_err("%s: Invalid pll ref divider %d", __func__, pll_ref_div);
		BUG();
	}
	c->mul = 1;
	c->state = ON;
}

static struct clk_ops tegra_pll_ref_ops = {
	.init		= tegra3_pll_ref_init,
	.enable		= tegra3_clk_m_enable,
	.disable	= tegra3_clk_m_disable,
};

/* super clock functions */
/* "super clocks" on tegra have two-stage muxes and a clock skipping
 * super divider.  We will ignore the clock skipping divider, since we
 * can't lower the voltage when using the clock skip, but we can if we
 * lower the PLL frequency.
 */
static void tegra3_super_clk_init(struct clk *c)
{
	u32 val;
	int source;
	int shift;
	const struct clk_mux_sel *sel;
	val = clk_readl(c->reg + SUPER_CLK_MUX);
	c->state = ON;
	BUG_ON(((val & SUPER_STATE_MASK) != SUPER_STATE_RUN) &&
		((val & SUPER_STATE_MASK) != SUPER_STATE_IDLE));
	shift = ((val & SUPER_STATE_MASK) == SUPER_STATE_IDLE) ?
		SUPER_IDLE_SOURCE_SHIFT : SUPER_RUN_SOURCE_SHIFT;
	source = (val >> shift) & SUPER_SOURCE_MASK;
	for (sel = c->inputs; sel->input != NULL; sel++) {
		if (sel->value == source)
			break;
	}
	BUG_ON(sel->input == NULL);
	c->parent = sel->input;

	INIT_LIST_HEAD(&c->u.shared_bus.list);
}

static int tegra3_super_clk_enable(struct clk *c)
{
	clk_writel(0, c->reg + SUPER_CLK_DIVIDER);
	return 0;
}

static void tegra3_super_clk_disable(struct clk *c)
{
	pr_debug("%s on clock %s\n", __func__, c->name);

	/* oops - don't disable the CPU clock! */
	BUG();
}

static int tegra3_super_clk_set_parent(struct clk *c, struct clk *p)
{
	u32 val;
	const struct clk_mux_sel *sel;
	int shift;

	val = clk_readl(c->reg + SUPER_CLK_MUX);;
	BUG_ON(((val & SUPER_STATE_MASK) != SUPER_STATE_RUN) &&
		((val & SUPER_STATE_MASK) != SUPER_STATE_IDLE));
	shift = ((val & SUPER_STATE_MASK) == SUPER_STATE_IDLE) ?
		SUPER_IDLE_SOURCE_SHIFT : SUPER_RUN_SOURCE_SHIFT;
	for (sel = c->inputs; sel->input != NULL; sel++) {
		if (sel->input == p) {
			val &= ~(SUPER_SOURCE_MASK << shift);
			val |= sel->value << shift;

			if (c->refcnt)
				clk_enable(p);

			clk_writel(val, c->reg);

			if (c->refcnt && c->parent)
				clk_disable(c->parent);

			clk_reparent(c, p);
			return 0;
		}
	}
	return -EINVAL;
}

/*
 * Super clocks have "clock skippers" instead of dividers.  Dividing using
 * a clock skipper does not allow the voltage to be scaled down, so instead
 * adjust the rate of the parent clock.  This requires that the parent of a
 * super clock have no other children, otherwise the rate will change
 * underneath the other children.
 */
static int tegra3_super_clk_set_rate(struct clk *c, unsigned long rate)
{
	return clk_set_rate(c->parent, rate);
}

static struct clk_ops tegra_super_ops = {
	.init			= tegra3_super_clk_init,
	.enable			= tegra3_super_clk_enable,
	.disable		= tegra3_super_clk_disable,
	.set_parent		= tegra3_super_clk_set_parent,
	.set_rate		= tegra3_super_clk_set_rate,
};

/* virtual cpu clock functions */
/* some clocks can not be stopped (cpu, memory bus) while the SoC is running.
   To change the frequency of these clocks, the parent pll may need to be
   reprogrammed, so the clock must be moved off the pll, the pll reprogrammed,
   and then the clock moved back to the pll.  To hide this sequence, a virtual
   clock handles it.
 */
static void tegra3_cpu_clk_init(struct clk *c)
{
	/* FIXME: max limits for different SKUs */
}

static int tegra3_cpu_clk_enable(struct clk *c)
{
	return 0;
}

static void tegra3_cpu_clk_disable(struct clk *c)
{
	pr_debug("%s on clock %s\n", __func__, c->name);

	/* oops - don't disable the CPU clock! */
	BUG();
}

static int tegra3_cpu_clk_set_rate(struct clk *c, unsigned long rate)
{
	int ret;
	/*
	 * Take an extra reference to the main pll so it doesn't turn
	 * off when we move the cpu off of it
	 */
	clk_enable(c->u.cpu.main);

	ret = clk_set_parent(c->parent, c->u.cpu.backup);
	if (ret) {
		pr_err("Failed to switch cpu to clock %s\n", c->u.cpu.backup->name);
		goto out;
	}

	if (rate == clk_get_rate(c->u.cpu.backup))
		goto out;

	ret = clk_set_rate(c->u.cpu.main, rate);
	if (ret) {
		pr_err("Failed to change cpu pll to %lu\n", rate);
		goto out;
	}

	ret = clk_set_parent(c->parent, c->u.cpu.main);
	if (ret) {
		pr_err("Failed to switch cpu to clock %s\n", c->u.cpu.main->name);
		goto out;
	}

out:
	clk_disable(c->u.cpu.main);
	return ret;
}

static unsigned long tegra3_cpu_get_max_rate(struct clk *c)
{
	if (IS_LP_CLUSTER)
		return c->u.cpu.lp_max_rate;
	else
		return c->max_rate;
}

static struct clk_ops tegra_cpu_ops = {
	.init     = tegra3_cpu_clk_init,
	.enable   = tegra3_cpu_clk_enable,
	.disable  = tegra3_cpu_clk_disable,
	.set_rate = tegra3_cpu_clk_set_rate,
	.get_max_rate = tegra3_cpu_get_max_rate,
};

/* virtual cop clock functions. Used to acquire the fake 'cop' clock to
 * reset the COP block (i.e. AVP) */
static void tegra3_cop_clk_reset(struct clk *c, bool assert)
{
	unsigned long reg = assert ? RST_DEVICES_SET_L : RST_DEVICES_CLR_L;

	pr_debug("%s %s\n", __func__, assert ? "assert" : "deassert");
	clk_writel(1 << 1, reg);
}

static struct clk_ops tegra_cop_ops = {
	.reset    = tegra3_cop_clk_reset,
};

/* bus clock functions */
static void tegra3_bus_clk_init(struct clk *c)
{
	u32 val = clk_readl(c->reg);
	c->state = ((val >> c->reg_shift) & BUS_CLK_DISABLE) ? OFF : ON;
	c->div = ((val >> c->reg_shift) & BUS_CLK_DIV_MASK) + 1;
	c->mul = 1;
}

static int tegra3_bus_clk_enable(struct clk *c)
{
	u32 val = clk_readl(c->reg);
	val &= ~(BUS_CLK_DISABLE << c->reg_shift);
	clk_writel(val, c->reg);
	return 0;
}

static void tegra3_bus_clk_disable(struct clk *c)
{
	u32 val = clk_readl(c->reg);
	val |= BUS_CLK_DISABLE << c->reg_shift;
	clk_writel(val, c->reg);
}

static int tegra3_bus_clk_set_rate(struct clk *c, unsigned long rate)
{
	u32 val = clk_readl(c->reg);
	unsigned long parent_rate = clk_get_rate(c->parent);
	int i;
	for (i = 1; i <= 4; i++) {
		if (rate >= parent_rate / i) {
			val &= ~(BUS_CLK_DIV_MASK << c->reg_shift);
			val |= (i - 1) << c->reg_shift;
			clk_writel(val, c->reg);
			c->div = i;
			c->mul = 1;
			return 0;
		}
	}
	return -EINVAL;
}

static struct clk_ops tegra_bus_ops = {
	.init			= tegra3_bus_clk_init,
	.enable			= tegra3_bus_clk_enable,
	.disable		= tegra3_bus_clk_disable,
	.set_rate		= tegra3_bus_clk_set_rate,
};

/* Blink output functions */

static void tegra3_blink_clk_init(struct clk *c)
{
	u32 val;

	val = pmc_readl(PMC_CTRL);
	c->state = (val & PMC_CTRL_BLINK_ENB) ? ON : OFF;
	c->mul = 1;
	val = pmc_readl(c->reg);

	if (val & PMC_BLINK_TIMER_ENB) {
		unsigned int on_off;

		on_off = (val >> PMC_BLINK_TIMER_DATA_ON_SHIFT) &
			PMC_BLINK_TIMER_DATA_ON_MASK;
		val >>= PMC_BLINK_TIMER_DATA_OFF_SHIFT;
		val &= PMC_BLINK_TIMER_DATA_OFF_MASK;
		on_off += val;
		/* each tick in the blink timer is 4 32KHz clocks */
		c->div = on_off * 4;
	} else {
		c->div = 1;
	}
}

static int tegra3_blink_clk_enable(struct clk *c)
{
	u32 val;

	val = pmc_readl(PMC_DPD_PADS_ORIDE);
	pmc_writel(val | PMC_DPD_PADS_ORIDE_BLINK_ENB, PMC_DPD_PADS_ORIDE);

	val = pmc_readl(PMC_CTRL);
	pmc_writel(val | PMC_CTRL_BLINK_ENB, PMC_CTRL);

	return 0;
}

static void tegra3_blink_clk_disable(struct clk *c)
{
	u32 val;

	val = pmc_readl(PMC_CTRL);
	pmc_writel(val & ~PMC_CTRL_BLINK_ENB, PMC_CTRL);

	val = pmc_readl(PMC_DPD_PADS_ORIDE);
	pmc_writel(val & ~PMC_DPD_PADS_ORIDE_BLINK_ENB, PMC_DPD_PADS_ORIDE);
}

static int tegra3_blink_clk_set_rate(struct clk *c, unsigned long rate)
{
	unsigned long parent_rate = clk_get_rate(c->parent);
	if (rate >= parent_rate) {
		c->div = 1;
		pmc_writel(0, c->reg);
	} else {
		unsigned int on_off;
		u32 val;

		on_off = DIV_ROUND_UP(parent_rate / 8, rate);
		c->div = on_off * 8;

		val = (on_off & PMC_BLINK_TIMER_DATA_ON_MASK) <<
			PMC_BLINK_TIMER_DATA_ON_SHIFT;
		on_off &= PMC_BLINK_TIMER_DATA_OFF_MASK;
		on_off <<= PMC_BLINK_TIMER_DATA_OFF_SHIFT;
		val |= on_off;
		val |= PMC_BLINK_TIMER_ENB;
		pmc_writel(val, c->reg);
	}

	return 0;
}

static struct clk_ops tegra_blink_clk_ops = {
	.init			= &tegra3_blink_clk_init,
	.enable			= &tegra3_blink_clk_enable,
	.disable		= &tegra3_blink_clk_disable,
	.set_rate		= &tegra3_blink_clk_set_rate,
};

/* PLL Functions */
static int tegra3_pll_clk_wait_for_lock(struct clk *c)
{
#if USE_PLL_LOCK_BITS
	int i;
	for (i = 0; i < c->u.pll.lock_delay; i++) {
		if (clk_readl(c->reg + PLL_BASE) & PLL_BASE_LOCK)
			return 0;
		udelay(2);		/* timeout = 2 * lock time */
	}
	pr_err("Timed out waiting for lock bit on pll %s", c->name);
	return -1;
#endif
	udelay(c->u.pll.lock_delay);

	return 0;
}

static void tegra3_pll_clk_init(struct clk *c)
{
	u32 val = clk_readl(c->reg + PLL_BASE);

	c->state = (val & PLL_BASE_ENABLE) ? ON : OFF;

	if (c->flags & PLL_FIXED && !(val & PLL_BASE_OVERRIDE)) {
		const struct clk_pll_freq_table *sel;
		unsigned long input_rate = clk_get_rate(c->parent);
		for (sel = c->u.pll.freq_table; sel->input_rate != 0; sel++) {
			if (sel->input_rate == input_rate &&
				sel->output_rate == c->u.pll.fixed_rate) {
				c->mul = sel->n;
				c->div = sel->m * sel->p;
				return;
			}
		}
		pr_warning("Clock %s has unknown fixed frequency\n", c->name);
		c->mul = 1;
		c->div = 1;
	} else if (val & PLL_BASE_BYPASS) {
		c->mul = 1;
		c->div = 1;
	} else {
		c->mul = (val & PLL_BASE_DIVN_MASK) >> PLL_BASE_DIVN_SHIFT;
		c->div = (val & PLL_BASE_DIVM_MASK) >> PLL_BASE_DIVM_SHIFT;
		if (c->flags & PLLU)
			c->div *= (val & PLLU_BASE_POST_DIV) ? 1 : 2;
		else
			c->div *= (0x1 << ((val & PLL_BASE_DIVP_MASK) >>
					PLL_BASE_DIVP_SHIFT));
	}
}

static int tegra3_pll_clk_enable(struct clk *c)
{
	u32 val;
	pr_debug("%s on clock %s\n", __func__, c->name);

#if USE_PLL_LOCK_BITS
	val = clk_readl(c->reg + PLL_MISC(c));
	val |= PLL_MISC_LOCK_ENABLE(c);
	clk_writel(val, c->reg + PLL_MISC(c));
#endif
	val = clk_readl(c->reg + PLL_BASE);
	val &= ~PLL_BASE_BYPASS;
	val |= PLL_BASE_ENABLE;
	clk_writel(val, c->reg + PLL_BASE);

	tegra3_pll_clk_wait_for_lock(c);

	return 0;
}

static void tegra3_pll_clk_disable(struct clk *c)
{
	u32 val;
	pr_debug("%s on clock %s\n", __func__, c->name);

	val = clk_readl(c->reg);
	val &= ~(PLL_BASE_BYPASS | PLL_BASE_ENABLE);
	clk_writel(val, c->reg);
}

static int tegra3_pll_clk_set_rate(struct clk *c, unsigned long rate)
{
	u32 val, p_div, old_base;
	unsigned long input_rate;
	const struct clk_pll_freq_table *sel;
	struct clk_pll_freq_table cfg;

	pr_debug("%s: %s %lu\n", __func__, c->name, rate);

	if (c->flags & PLL_FIXED) {
		val = clk_readl(c->reg + PLL_BASE);
		if (!(val & PLL_BASE_OVERRIDE) && (rate == c->u.pll.fixed_rate))
			return 0;
	}

	p_div = 0;
	input_rate = clk_get_rate(c->parent);

	/* Check if the target rate is tabulated */
	for (sel = c->u.pll.freq_table; sel->input_rate != 0; sel++) {
		if (sel->input_rate == input_rate && sel->output_rate == rate) {
			if (c->flags & PLLU) {
				BUG_ON(sel->p < 1 || sel->p > 2);
				if (sel->p == 1)
					p_div = PLLU_BASE_POST_DIV;
			} else {
				BUG_ON(sel->p < 1);
				for (val = sel->p; val > 1; val >>= 1, p_div++);
				p_div <<= PLL_BASE_DIVP_SHIFT;
			}
			break;
		}
	}

	/* Configure out-of-table rate */
	if (sel->input_rate == 0) {
		unsigned long cfreq;
		BUG_ON(c->flags & PLLU);
		sel = &cfg;

		switch (input_rate) {
		case 12000000:
		case 13000000:
		case 26000000:
			cfreq = 1000000;
			break;
		case 16800000:
		case 19200000:
			cfreq = 1200000;
			break;
		default:
			pr_err("%s: Unexpected reference rate %lu\n", __func__, input_rate);
			BUG();
		}

		/* Raise VCO to guarantee 0.5% accuracy */
		for (cfg.output_rate = rate; cfg.output_rate < 200 * cfreq;
		      cfg.output_rate <<= 1, p_div++);

		cfg.p = 0x1 << p_div;
		cfg.m = input_rate / cfreq;
		cfg.n = cfg.output_rate / cfreq;
		cfg.cpcon = OUT_OF_TABLE_CPCON;

		if ((cfg.m > (PLL_BASE_DIVM_MASK >> PLL_BASE_DIVM_SHIFT)) ||
		    (cfg.n > (PLL_BASE_DIVN_MASK >> PLL_BASE_DIVN_SHIFT)) ||
		    (p_div > (PLL_BASE_DIVP_MASK >> PLL_BASE_DIVP_SHIFT)) ||
		    (cfg.output_rate > c->u.pll.vco_max)) {
			pr_err("%s: Failed to set %s out-of-table rate %lu\n",
			       __func__, c->name, rate);
			return -EINVAL;
		}
		p_div <<= PLL_BASE_DIVP_SHIFT;
	}

	c->mul = sel->n;
	c->div = sel->m * sel->p;

	old_base = val = clk_readl(c->reg + PLL_BASE);
	if (c->flags & PLL_FIXED) {
		BUG();
		val |= PLL_BASE_OVERRIDE;
	}
	val &= ~(PLL_BASE_DIVM_MASK | PLL_BASE_DIVN_MASK |
		 ((c->flags & PLLU) ? PLLU_BASE_POST_DIV : PLL_BASE_DIVP_MASK));
	val |= (sel->m << PLL_BASE_DIVM_SHIFT) |
		(sel->n << PLL_BASE_DIVN_SHIFT) | p_div;
	if (val == old_base)
		return 0;

	if (c->state == ON) {
		tegra3_pll_clk_disable(c);
		val &= ~(PLL_BASE_BYPASS | PLL_BASE_ENABLE);
	}
	clk_writel(val, c->reg + PLL_BASE);

	if (c->flags & PLL_HAS_CPCON) {
		val = clk_readl(c->reg + PLL_MISC(c));
		val &= ~PLL_MISC_CPCON_MASK;
		val |= sel->cpcon << PLL_MISC_CPCON_SHIFT;
		if (c->flags & (PLLU | PLLD)) {
			val &= ~PLL_MISC_LFCON_MASK;
			if (sel->n >= PLLDU_LFCON_SET_DIVN)
				val |= 0x1 << PLL_MISC_LFCON_SHIFT;
		} else if (c->flags & PLLX) {
			val &= ~(0x1 << PLL_MISC_DCCON_SHIFT);
			if (rate >= (c->u.pll.vco_max >> 1))
				val |= 0x1 << PLL_MISC_DCCON_SHIFT;
		}
		clk_writel(val, c->reg + PLL_MISC(c));
	}

	if (c->state == ON)
		tegra3_pll_clk_enable(c);

	return 0;
}

static struct clk_ops tegra_pll_ops = {
	.init			= tegra3_pll_clk_init,
	.enable			= tegra3_pll_clk_enable,
	.disable		= tegra3_pll_clk_disable,
	.set_rate		= tegra3_pll_clk_set_rate,
};

/* Clock divider ops */
static void tegra3_pll_div_clk_init(struct clk *c)
{
	u32 val = clk_readl(c->reg);
	u32 divu71;
	val >>= c->reg_shift;
	c->state = (val & PLL_OUT_CLKEN) ? ON : OFF;
	if (!(val & PLL_OUT_RESET_DISABLE))
		c->state = OFF;

	if (c->flags & DIV_U71) {
		divu71 = (val & PLL_OUT_RATIO_MASK) >> PLL_OUT_RATIO_SHIFT;
		c->div = (divu71 + 2);
		c->mul = 2;
	} else if (c->flags & DIV_2) {
		if (c->flags & PLLD) {
			c->div = 2;
			c->mul = 1;
		}
		else if (c->flags & PLLX) {
			c->div = (IS_LP_CLUSTER &&
					(!IS_PLLX_DIV2_BYPASS)) ? 2 : 1;
			c->mul = 1;
		}
		else
			BUG();
	} else {
		c->div = 1;
		c->mul = 1;
	}
}

static int tegra3_pll_div_clk_enable(struct clk *c)
{
	u32 val;
	u32 new_val;

	pr_debug("%s: %s\n", __func__, c->name);
	if (c->flags & DIV_U71) {
		val = clk_readl(c->reg);
		new_val = val >> c->reg_shift;
		new_val &= 0xFFFF;

		new_val |= PLL_OUT_CLKEN | PLL_OUT_RESET_DISABLE;

		val &= ~(0xFFFF << c->reg_shift);
		val |= new_val << c->reg_shift;
		clk_writel(val, c->reg);
		return 0;
	} else if (c->flags & DIV_2) {
		return 0;
	}
	return -EINVAL;
}

static void tegra3_pll_div_clk_disable(struct clk *c)
{
	u32 val;
	u32 new_val;

	pr_debug("%s: %s\n", __func__, c->name);
	if (c->flags & DIV_U71) {
		val = clk_readl(c->reg);
		new_val = val >> c->reg_shift;
		new_val &= 0xFFFF;

		new_val &= ~(PLL_OUT_CLKEN | PLL_OUT_RESET_DISABLE);

		val &= ~(0xFFFF << c->reg_shift);
		val |= new_val << c->reg_shift;
		clk_writel(val, c->reg);
	}
}

static int tegra3_pll_div_clk_set_rate(struct clk *c, unsigned long rate)
{
	u32 val;
	u32 new_val;
	int divider_u71;
	unsigned long parent_rate = clk_get_rate(c->parent);

	pr_debug("%s: %s %lu\n", __func__, c->name, rate);
	if (c->flags & DIV_U71) {
		divider_u71 = clk_div71_get_divider(parent_rate, rate);
		if (divider_u71 >= 0) {
			val = clk_readl(c->reg);
			new_val = val >> c->reg_shift;
			new_val &= 0xFFFF;
			if (c->flags & DIV_U71_FIXED)
				new_val |= PLL_OUT_OVERRIDE;
			new_val &= ~PLL_OUT_RATIO_MASK;
			new_val |= divider_u71 << PLL_OUT_RATIO_SHIFT;

			val &= ~(0xFFFF << c->reg_shift);
			val |= new_val << c->reg_shift;
			clk_writel(val, c->reg);
			c->div = divider_u71 + 2;
			c->mul = 2;
			return 0;
		}
	} else if (c->flags & DIV_2) {
		if (c->flags & PLLD) {
			return clk_set_rate(c->parent, rate * 2);
		}
		else if (c->flags & PLLX) {
			if (IS_LP_CLUSTER && (!IS_PLLX_DIV2_BYPASS))
				rate *= 2;
			return clk_set_rate(c->parent, rate);
		}
	}
	return -EINVAL;
}

static long tegra3_pll_div_clk_round_rate(struct clk *c, unsigned long rate)
{
	int divider;
	unsigned long parent_rate = clk_get_rate(c->parent);
	pr_debug("%s: %s %lu\n", __func__, c->name, rate);

	if (c->flags & DIV_U71) {
		divider = clk_div71_get_divider(parent_rate, rate);
		if (divider < 0)
			return divider;
		return parent_rate * 2 / (divider + 2);
	}
	return -EINVAL;
}

static void tegra3_pllx_div_clk_recalculate_rate(struct clk *c)
{
	c->div = (IS_LP_CLUSTER && (!IS_PLLX_DIV2_BYPASS)) ? 2 : 1;
}

static struct clk_ops tegra_pll_div_ops = {
	.init			= tegra3_pll_div_clk_init,
	.enable			= tegra3_pll_div_clk_enable,
	.disable		= tegra3_pll_div_clk_disable,
	.set_rate		= tegra3_pll_div_clk_set_rate,
	.round_rate		= tegra3_pll_div_clk_round_rate,
};

static struct clk_ops tegra_plld_div_ops = {
	.init			= tegra3_pll_div_clk_init,
	.enable			= tegra3_pll_div_clk_enable,
	.disable		= tegra3_pll_div_clk_disable,
	.set_rate		= tegra3_pll_div_clk_set_rate,
};

static struct clk_ops tegra_pllx_div_ops = {
	.init			= tegra3_pll_div_clk_init,
	.enable			= tegra3_pll_div_clk_enable,
	.disable		= tegra3_pll_div_clk_disable,
	.set_rate		= tegra3_pll_div_clk_set_rate,
	.recalculate_rate = tegra3_pllx_div_clk_recalculate_rate,
};

/* Periph clk ops */

static void tegra3_periph_clk_init(struct clk *c)
{
	u32 val = clk_readl(c->reg);
	const struct clk_mux_sel *mux = 0;
	const struct clk_mux_sel *sel;
	if (c->flags & (MUX | MUX_PWM | MUX8)) {
		for (sel = c->inputs; sel->input != NULL; sel++) {
			if (val >> PERIPH_CLK_SOURCE_SHIFT(c) == sel->value)
				mux = sel;
		}
		BUG_ON(!mux);

		c->parent = mux->input;
	} else {
		c->parent = c->inputs[0].input;
	}

	if (c->flags & DIV_U71) {
		u32 divu71 = val & PERIPH_CLK_SOURCE_DIVU71_MASK;
		c->div = divu71 + 2;
		c->mul = 2;
	} else if (c->flags & DIV_U16) {
		u32 divu16 = val & PERIPH_CLK_SOURCE_DIVU16_MASK;
		c->div = divu16 + 1;
		c->mul = 1;
	} else {
		c->div = 1;
		c->mul = 1;
	}

	c->state = ON;
	if (!(clk_readl(PERIPH_CLK_TO_ENB_REG(c)) & PERIPH_CLK_TO_BIT(c)))
		c->state = OFF;
	if (!(c->flags & PERIPH_NO_RESET))
		if (clk_readl(PERIPH_CLK_TO_RST_REG(c)) & PERIPH_CLK_TO_BIT(c))
			c->state = OFF;
}

static int tegra3_periph_clk_enable(struct clk *c)
{
	pr_debug("%s on clock %s\n", __func__, c->name);

	tegra_periph_clk_enable_refcount[c->u.periph.clk_num]++;
	if (tegra_periph_clk_enable_refcount[c->u.periph.clk_num] > 1)
		return 0;

	clk_writel(PERIPH_CLK_TO_BIT(c), PERIPH_CLK_TO_ENB_SET_REG(c));
	if (!(c->flags & PERIPH_NO_RESET) && !(c->flags & PERIPH_MANUAL_RESET)) {
		if (clk_readl(PERIPH_CLK_TO_RST_REG(c)) & PERIPH_CLK_TO_BIT(c)) {
			udelay(5);	/* reset propagation delay */
			clk_writel(PERIPH_CLK_TO_BIT(c), PERIPH_CLK_TO_RST_CLR_REG(c));
		}
	}
	return 0;
}

static void tegra3_periph_clk_disable(struct clk *c)
{
	pr_debug("%s on clock %s\n", __func__, c->name);

	if (c->refcnt)
		tegra_periph_clk_enable_refcount[c->u.periph.clk_num]--;

	if (tegra_periph_clk_enable_refcount[c->u.periph.clk_num] == 0)
		clk_writel(PERIPH_CLK_TO_BIT(c), PERIPH_CLK_TO_ENB_CLR_REG(c));
}

static void tegra3_periph_clk_reset(struct clk *c, bool assert)
{
	pr_debug("%s %s on clock %s\n", __func__,
		 assert ? "assert" : "deassert", c->name);

	if (!(c->flags & PERIPH_NO_RESET)) {
		if (assert)
			clk_writel(PERIPH_CLK_TO_BIT(c),
				   PERIPH_CLK_TO_RST_SET_REG(c));
		else
			clk_writel(PERIPH_CLK_TO_BIT(c),
				   PERIPH_CLK_TO_RST_CLR_REG(c));
	}
}

static int tegra3_periph_clk_set_parent(struct clk *c, struct clk *p)
{
	u32 val;
	const struct clk_mux_sel *sel;
	pr_debug("%s: %s %s\n", __func__, c->name, p->name);
	for (sel = c->inputs; sel->input != NULL; sel++) {
		if (sel->input == p) {
			val = clk_readl(c->reg);
			val &= ~PERIPH_CLK_SOURCE_MASK(c);
			val |= (sel->value) << PERIPH_CLK_SOURCE_SHIFT(c);

			if (c->refcnt)
				clk_enable(p);

			clk_writel(val, c->reg);

			if (c->refcnt && c->parent)
				clk_disable(c->parent);

			clk_reparent(c, p);
			return 0;
		}
	}

	return -EINVAL;
}

static int tegra3_periph_clk_set_rate(struct clk *c, unsigned long rate)
{
	u32 val;
	int divider;
	unsigned long parent_rate = clk_get_rate(c->parent);

	if (c->flags & DIV_U71) {
		divider = clk_div71_get_divider(parent_rate, rate);
		if (divider >= 0) {
			val = clk_readl(c->reg);
			val &= ~PERIPH_CLK_SOURCE_DIVU71_MASK;
			val |= divider;
			clk_writel(val, c->reg);
			c->div = divider + 2;
			c->mul = 2;
			return 0;
		}
	} else if (c->flags & DIV_U16) {
		divider = clk_div16_get_divider(parent_rate, rate);
		if (divider >= 0) {
			val = clk_readl(c->reg);
			val &= ~PERIPH_CLK_SOURCE_DIVU16_MASK;
			val |= divider;
			clk_writel(val, c->reg);
			c->div = divider + 1;
			c->mul = 1;
			return 0;
		}
	} else if (parent_rate <= rate) {
		c->div = 1;
		c->mul = 1;
		return 0;
	}
	return -EINVAL;
}

static long tegra3_periph_clk_round_rate(struct clk *c,
	unsigned long rate)
{
	int divider;
	unsigned long parent_rate = clk_get_rate(c->parent);
	pr_debug("%s: %s %lu\n", __func__, c->name, rate);

	if (c->flags & DIV_U71) {
		divider = clk_div71_get_divider(parent_rate, rate);
		if (divider < 0)
			return divider;

		return parent_rate * 2 / (divider + 2);
	} else if (c->flags & DIV_U16) {
		divider = clk_div16_get_divider(parent_rate, rate);
		if (divider < 0)
			return divider;
		return parent_rate / (divider + 1);
	}
	return -EINVAL;
}

static struct clk_ops tegra_periph_clk_ops = {
	.init			= &tegra3_periph_clk_init,
	.enable			= &tegra3_periph_clk_enable,
	.disable		= &tegra3_periph_clk_disable,
	.set_parent		= &tegra3_periph_clk_set_parent,
	.set_rate		= &tegra3_periph_clk_set_rate,
	.round_rate		= &tegra3_periph_clk_round_rate,
	.reset			= &tegra3_periph_clk_reset,
};

/* Clock doubler ops */
static void tegra3_clk_double_init(struct clk *c)
{
	u32 val = clk_readl(c->reg);
	c->mul = val & (0x1 << c->reg_shift) ? 1 : 2;
	c->div = 1;
	c->state = ON;
	if (!(clk_readl(PERIPH_CLK_TO_ENB_REG(c)) & PERIPH_CLK_TO_BIT(c)))
		c->state = OFF;
};

static int tegra3_clk_double_set_rate(struct clk *c, unsigned long rate)
{
	u32 val;
	unsigned long parent_rate = clk_get_rate(c->parent);
	if (rate == parent_rate) {
		val = clk_readl(c->reg) | (0x1 << c->reg_shift);
		clk_writel(val, c->reg);
		c->mul = 1;
		c->div = 1;
		return 0;
	} else if (rate == 2 * parent_rate) {
		val = clk_readl(c->reg) & (~(0x1 << c->reg_shift));
		clk_writel(val, c->reg);
		c->mul = 2;
		c->div = 1;
		return 0;
	}
	return -EINVAL;
}

static struct clk_ops tegra_clk_double_ops = {
	.init			= &tegra3_clk_double_init,
	.enable			= &tegra3_periph_clk_enable,
	.disable		= &tegra3_periph_clk_disable,
	.set_rate		= &tegra3_clk_double_set_rate,
};

/* Audio sync clock ops */
static void tegra3_audio_sync_clk_init(struct clk *c)
{
	int source;
	const struct clk_mux_sel *sel;
	u32 val = clk_readl(c->reg);
	c->state = (val & AUDIO_SYNC_DISABLE_BIT) ? OFF : ON;
	source = val & AUDIO_SYNC_SOURCE_MASK;
	for (sel = c->inputs; sel->input != NULL; sel++)
		if (sel->value == source)
			break;
	BUG_ON(sel->input == NULL);
	c->parent = sel->input;
}

static int tegra3_audio_sync_clk_enable(struct clk *c)
{
	u32 val = clk_readl(c->reg);
	clk_writel((val & (~AUDIO_SYNC_DISABLE_BIT)), c->reg);
	return 0;
}

static void tegra3_audio_sync_clk_disable(struct clk *c)
{
	u32 val = clk_readl(c->reg);
	clk_writel((val | AUDIO_SYNC_DISABLE_BIT), c->reg);
}

static int tegra3_audio_sync_clk_set_parent(struct clk *c, struct clk *p)
{
	u32 val;
	const struct clk_mux_sel *sel;
	for (sel = c->inputs; sel->input != NULL; sel++) {
		if (sel->input == p) {
			val = clk_readl(c->reg);
			val &= ~AUDIO_SYNC_SOURCE_MASK;
			val |= sel->value;

			if (c->refcnt)
				clk_enable(p);

			clk_writel(val, c->reg);

			if (c->refcnt && c->parent)
				clk_disable(c->parent);

			clk_reparent(c, p);
			return 0;
		}
	}

	return -EINVAL;
}

static struct clk_ops tegra_audio_sync_clk_ops = {
	.init       = tegra3_audio_sync_clk_init,
	.enable     = tegra3_audio_sync_clk_enable,
	.disable    = tegra3_audio_sync_clk_disable,
	.set_parent = tegra3_audio_sync_clk_set_parent,
};

/* cdev1 and cdev2 (dap_mclk1 and dap_mclk2) ops */

static void tegra3_cdev_clk_init(struct clk *c)
{
	/* We could un-tristate the cdev1 or cdev2 pingroup here; this is
	 * currently done in the pinmux code. */
	c->state = ON;
	if (!(clk_readl(PERIPH_CLK_TO_ENB_REG(c)) & PERIPH_CLK_TO_BIT(c)))
		c->state = OFF;
}

static int tegra3_cdev_clk_enable(struct clk *c)
{
	clk_writel(PERIPH_CLK_TO_BIT(c), PERIPH_CLK_TO_ENB_SET_REG(c));
	return 0;
}

static void tegra3_cdev_clk_disable(struct clk *c)
{
	clk_writel(PERIPH_CLK_TO_BIT(c), PERIPH_CLK_TO_ENB_CLR_REG(c));
}

static struct clk_ops tegra_cdev_clk_ops = {
	.init			= &tegra3_cdev_clk_init,
	.enable			= &tegra3_cdev_clk_enable,
	.disable		= &tegra3_cdev_clk_disable,
};

/* shared bus ops */
/*
 * Some clocks may have multiple downstream users that need to request a
 * higher clock rate.  Shared bus clocks provide a unique shared_bus_user
 * clock to each user.  The frequency of the bus is set to the highest
 * enabled shared_bus_user clock, with a minimum value set by the
 * shared bus.
 */
static void tegra_clk_shared_bus_update(struct clk *bus)
{
	struct clk *c;
	unsigned long rate = bus->u.shared_bus.min_rate;

	list_for_each_entry(c, &bus->u.shared_bus.list,
			u.shared_bus_user.node) {
		if (c->u.shared_bus_user.enabled)
			rate = max(c->u.shared_bus_user.rate, rate);
	}

	clk_set_rate(bus, rate);
};

static void tegra_clk_shared_bus_init(struct clk *c)
{
	c->max_rate = c->parent->max_rate;
	c->u.shared_bus_user.rate = c->parent->max_rate;
	c->state = OFF;
	c->set = true;

	list_add_tail(&c->u.shared_bus_user.node,
		&c->parent->u.shared_bus.list);
}

static int tegra_clk_shared_bus_set_rate(struct clk *c, unsigned long rate)
{
	c->u.shared_bus_user.rate = rate;
	tegra_clk_shared_bus_update(c->parent);
	return 0;
}

static int tegra_clk_shared_bus_enable(struct clk *c)
{
	c->u.shared_bus_user.enabled = true;
	tegra_clk_shared_bus_update(c->parent);
	return 0;
}

static void tegra_clk_shared_bus_disable(struct clk *c)
{
	c->u.shared_bus_user.enabled = false;
	tegra_clk_shared_bus_update(c->parent);
}

static struct clk_ops tegra_clk_shared_bus_ops = {
	.init = tegra_clk_shared_bus_init,
	.enable = tegra_clk_shared_bus_enable,
	.disable = tegra_clk_shared_bus_disable,
	.set_rate = tegra_clk_shared_bus_set_rate,
};


/* Clock definitions */
static struct clk tegra_clk_32k = {
	.name = "clk_32k",
	.rate = 32768,
	.ops  = NULL,
	.max_rate = 32768,
};

static struct clk tegra_clk_m = {
	.name      = "clk_m",
	.flags     = ENABLE_ON_INIT,
	.ops       = &tegra_clk_m_ops,
	.reg       = 0x1fc,
	.reg_shift = 28,
	.max_rate  = 48000000,
};

static struct clk tegra_pll_ref = {
	.name      = "pll_ref",
	.flags     = ENABLE_ON_INIT,
	.ops       = &tegra_pll_ref_ops,
	.parent    = &tegra_clk_m,
	.max_rate  = 26000000,
};

static struct clk_pll_freq_table tegra_pll_c_freq_table[] = {
	{ 12000000, 600000000, 600, 12, 1, 8},
	{ 13000000, 600000000, 600, 13, 1, 8},
	{ 16800000, 600000000, 500, 14, 1, 8},
	{ 19200000, 600000000, 375, 12, 1, 6},
	{ 26000000, 600000000, 600, 26, 1, 8},
	{ 0, 0, 0, 0, 0, 0 },
};

static struct clk tegra_pll_c = {
	.name      = "pll_c",
	.flags	   = PLL_HAS_CPCON,
	.ops       = &tegra_pll_ops,
	.reg       = 0x80,
	.parent    = &tegra_pll_ref,
	.max_rate  = 600000000,
	.u.pll = {
		.input_min = 2000000,
		.input_max = 31000000,
		.cf_min    = 1000000,
		.cf_max    = 6000000,
		.vco_min   = 20000000,
		.vco_max   = 1400000000,
		.freq_table = tegra_pll_c_freq_table,
		.lock_delay = 300,
	},
};

static struct clk tegra_pll_c_out1 = {
	.name      = "pll_c_out1",
	.ops       = &tegra_pll_div_ops,
	.flags     = DIV_U71,
	.parent    = &tegra_pll_c,
	.reg       = 0x84,
	.reg_shift = 0,
	.max_rate  = 600000000,
};

static struct clk_pll_freq_table tegra_pll_m_freq_table[] = {
	{ 12000000, 666000000, 666, 12, 1, 8},
	{ 13000000, 666000000, 666, 13, 1, 8},
	{ 16800000, 666000000, 555, 14, 1, 8},
	{ 19200000, 666000000, 555, 16, 1, 8},
	{ 26000000, 666000000, 666, 26, 1, 8},
	{ 12000000, 600000000, 600, 12, 1, 8},
	{ 13000000, 600000000, 600, 13, 1, 8},
	{ 16800000, 600000000, 500, 14, 1, 8},
	{ 19200000, 600000000, 375, 12, 1, 6},
	{ 26000000, 600000000, 600, 26, 1, 8},
	{ 0, 0, 0, 0, 0, 0 },
};

static struct clk tegra_pll_m = {
	.name      = "pll_m",
	.flags     = PLL_HAS_CPCON,
	.ops       = &tegra_pll_ops,
	.reg       = 0x90,
	.parent    = &tegra_pll_ref,
	.max_rate  = 800000000,
	.u.pll = {
		.input_min = 2000000,
		.input_max = 31000000,
		.cf_min    = 1000000,
		.cf_max    = 6000000,
		.vco_min   = 20000000,
		.vco_max   = 1200000000,
		.freq_table = tegra_pll_m_freq_table,
		.lock_delay = 300,
	},
};

static struct clk tegra_pll_m_out1 = {
	.name      = "pll_m_out1",
	.ops       = &tegra_pll_div_ops,
	.flags     = DIV_U71,
	.parent    = &tegra_pll_m,
	.reg       = 0x94,
	.reg_shift = 0,
	.max_rate  = 600000000,
};

static struct clk_pll_freq_table tegra_pll_p_freq_table[] = {
	{ 12000000, 216000000, 432, 12, 2, 8},
	{ 13000000, 216000000, 432, 13, 2, 8},
	{ 16800000, 216000000, 360, 14, 2, 8},
	{ 19200000, 216000000, 360, 16, 2, 8},
	{ 26000000, 216000000, 432, 26, 2, 8},
	{ 0, 0, 0, 0, 0, 0 },
};

static struct clk tegra_pll_p = {
	.name      = "pll_p",
	.flags     = ENABLE_ON_INIT | PLL_FIXED | PLL_HAS_CPCON,
	.ops       = &tegra_pll_ops,
	.reg       = 0xa0,
	.parent    = &tegra_pll_ref,
	.max_rate  = 432000000,
	.u.pll = {
		.input_min = 2000000,
		.input_max = 31000000,
		.cf_min    = 1000000,
		.cf_max    = 6000000,
		.vco_min   = 20000000,
		.vco_max   = 1400000000,
		.freq_table = tegra_pll_p_freq_table,
		.lock_delay = 300,
		.fixed_rate = 216000000,
	},
};

static struct clk tegra_pll_p_out1 = {
	.name      = "pll_p_out1",
	.ops       = &tegra_pll_div_ops,
	.flags     = ENABLE_ON_INIT | DIV_U71 | DIV_U71_FIXED,
	.parent    = &tegra_pll_p,
	.reg       = 0xa4,
	.reg_shift = 0,
	.max_rate  = 432000000,
};

static struct clk tegra_pll_p_out2 = {
	.name      = "pll_p_out2",
	.ops       = &tegra_pll_div_ops,
	.flags     = ENABLE_ON_INIT | DIV_U71 | DIV_U71_FIXED,
	.parent    = &tegra_pll_p,
	.reg       = 0xa4,
	.reg_shift = 16,
	.max_rate  = 432000000,
};

static struct clk tegra_pll_p_out3 = {
	.name      = "pll_p_out3",
	.ops       = &tegra_pll_div_ops,
	.flags     = ENABLE_ON_INIT | DIV_U71 | DIV_U71_FIXED,
	.parent    = &tegra_pll_p,
	.reg       = 0xa8,
	.reg_shift = 0,
	.max_rate  = 432000000,
};

static struct clk tegra_pll_p_out4 = {
	.name      = "pll_p_out4",
	.ops       = &tegra_pll_div_ops,
	.flags     = ENABLE_ON_INIT | DIV_U71 | DIV_U71_FIXED,
	.parent    = &tegra_pll_p,
	.reg       = 0xa8,
	.reg_shift = 16,
	.max_rate  = 432000000,
};

static struct clk_pll_freq_table tegra_pll_a_freq_table[] = {
	{ 28800000, 56448000, 49, 25, 1, 1},
	{ 28800000, 73728000, 64, 25, 1, 1},
	{ 28800000, 11289600, 49, 25, 1, 1},	/* actual: 56.448MHz */
	{ 28800000, 12288000, 64, 25, 1, 1},	/* actual: 73.728MHz */
	{ 28800000, 24000000,  5,  6, 1, 1},
	{ 0, 0, 0, 0, 0, 0 },
};

static struct clk tegra_pll_a = {
	.name      = "pll_a",
	.flags     = PLL_HAS_CPCON,
	.ops       = &tegra_pll_ops,
	.reg       = 0xb0,
	.parent    = &tegra_pll_p_out1,
	.max_rate  = 56448000,
	.u.pll = {
		.input_min = 2000000,
		.input_max = 31000000,
		.cf_min    = 1000000,
		.cf_max    = 6000000,
		.vco_min   = 20000000,
		.vco_max   = 1400000000,
		.freq_table = tegra_pll_a_freq_table,
		.lock_delay = 300,
	},
};

static struct clk tegra_pll_a_out0 = {
	.name      = "pll_a_out0",
	.ops       = &tegra_pll_div_ops,
	.flags     = DIV_U71,
	.parent    = &tegra_pll_a,
	.reg       = 0xb4,
	.reg_shift = 0,
	.max_rate  = 56448000,
};

static struct clk_pll_freq_table tegra_pll_d_freq_table[] = {
	{ 12000000, 216000000, 216, 12, 1, 4},
	{ 13000000, 216000000, 216, 13, 1, 4},
	{ 16800000, 216000000, 180, 14, 1, 4},
	{ 19200000, 216000000, 180, 16, 1, 4},
	{ 26000000, 216000000, 216, 26, 1, 4},

	{ 12000000, 594000000, 594, 12, 1, 8},
	{ 13000000, 594000000, 594, 13, 1, 8},
	{ 16800000, 594000000, 495, 14, 1, 8},
	{ 19200000, 594000000, 495, 16, 1, 8},
	{ 26000000, 594000000, 594, 26, 1, 8},

	{ 12000000, 1000000000, 1000, 12, 1, 12},
	{ 13000000, 1000000000, 1000, 13, 1, 12},
	{ 19200000, 1000000000, 625,  12, 1, 8},
	{ 26000000, 1000000000, 1000, 26, 1, 12},

	{ 0, 0, 0, 0, 0, 0 },
};

static struct clk tegra_pll_d = {
	.name      = "pll_d",
	.flags     = PLL_HAS_CPCON | PLLD,
	.ops       = &tegra_pll_ops,
	.reg       = 0xd0,
	.parent    = &tegra_pll_ref,
	.max_rate  = 1000000000,
	.u.pll = {
		.input_min = 2000000,
		.input_max = 40000000,
		.cf_min    = 1000000,
		.cf_max    = 6000000,
		.vco_min   = 40000000,
		.vco_max   = 1000000000,
		.freq_table = tegra_pll_d_freq_table,
		.lock_delay = 1000,
	},
};

static struct clk tegra_pll_d_out0 = {
	.name      = "pll_d_out0",
	.ops       = &tegra_plld_div_ops,
	.flags     = DIV_2 | PLLD,
	.parent    = &tegra_pll_d,
	.max_rate  = 500000000,
};

static struct clk tegra_pll_d2 = {
	.name      = "pll_d2",
	.flags     = PLL_HAS_CPCON | PLL_ALT_MISC_REG | PLLD,
	.ops       = &tegra_pll_ops,
	.reg       = 0x4b8,
	.parent    = &tegra_pll_ref,
	.max_rate  = 1000000000,
	.u.pll = {
		.input_min = 2000000,
		.input_max = 40000000,
		.cf_min    = 1000000,
		.cf_max    = 6000000,
		.vco_min   = 40000000,
		.vco_max   = 1000000000,
		.freq_table = tegra_pll_d_freq_table,
		.lock_delay = 1000,
	},
};

static struct clk tegra_pll_d2_out0 = {
	.name      = "pll_d2_out0",
	.ops       = &tegra_plld_div_ops,
	.flags     = DIV_2 | PLLD,
	.parent    = &tegra_pll_d2,
	.max_rate  = 500000000,
};

static struct clk_pll_freq_table tegra_pll_u_freq_table[] = {
	{ 12000000, 480000000, 960, 12, 2, 12},
	{ 13000000, 480000000, 960, 13, 2, 12},
	{ 16800000, 480000000, 400, 7,  2, 5},
	{ 19200000, 480000000, 200, 4,  2, 3},
	{ 26000000, 480000000, 960, 26, 2, 12},
	{ 0, 0, 0, 0, 0, 0 },
};

static struct clk tegra_pll_u = {
	.name      = "pll_u",
	.flags     = PLL_HAS_CPCON | PLLU,
	.ops       = &tegra_pll_ops,
	.reg       = 0xc0,
	.parent    = &tegra_pll_ref,
	.max_rate  = 480000000,
	.u.pll = {
		.input_min = 2000000,
		.input_max = 40000000,
		.cf_min    = 1000000,
		.cf_max    = 6000000,
		.vco_min   = 480000000,
		.vco_max   = 960000000,
		.freq_table = tegra_pll_u_freq_table,
		.lock_delay = 1000,
	},
};

static struct clk_pll_freq_table tegra_pll_x_freq_table[] = {
	/* 1 GHz */
	{ 12000000, 1000000000, 1000, 12, 1, 8},
	{ 13000000, 1000000000, 1000, 13, 1, 8},
	{ 16800000, 1000000000, 833,  14, 1, 8},	/* actual: 999.6 MHz */
	{ 19200000, 1000000000, 625,  12, 1, 8},
	{ 26000000, 1000000000, 1000, 26, 1, 8},

	/* 912 MHz */
	{ 12000000, 912000000,  912,  12, 1, 8},
	{ 13000000, 912000000,  912,  13, 1, 8},
	{ 16800000, 912000000,  760,  14, 1, 8},
	{ 19200000, 912000000,  760,  16, 1, 8},
	{ 26000000, 912000000,  912,  26, 1, 8},

	/* 816 MHz */
	{ 12000000, 816000000,  816,  12, 1, 8},
	{ 13000000, 816000000,  816,  13, 1, 8},
	{ 16800000, 816000000,  680,  14, 1, 8},
	{ 19200000, 816000000,  680,  16, 1, 8},
	{ 26000000, 816000000,  816,  26, 1, 8},

	/* 760 MHz */
	{ 12000000, 760000000,  760,  12, 1, 8},
	{ 13000000, 760000000,  760,  13, 1, 8},
	{ 16800000, 760000000,  633,  14, 1, 8},	/* actual: 759.6 MHz */
	{ 19200000, 760000000,  475,  12, 1, 8},
	{ 26000000, 760000000,  760,  26, 1, 8},

	/* 624 MHz */
	{ 12000000, 624000000,  624,  12, 1, 8},
	{ 13000000, 624000000,  624,  13, 1, 8},
	{ 16800000, 624000000,  520,  14, 1, 8},
	{ 19200000, 624000000,  520,  16, 1, 8},
	{ 26000000, 624000000,  624,  26, 1, 8},

	/* 608 MHz */
	{ 12000000, 608000000,  608,  12, 1, 8},
	{ 13000000, 608000000,  608,  13, 1, 8},
	{ 16800000, 608000000,  579,  16, 1, 8},	/* actual: 607.95 MHz */
	{ 19200000, 608000000,  380,  12, 1, 8},
	{ 26000000, 608000000,  608,  26, 1, 8},

	/* 456 MHz */
	{ 12000000, 456000000,  456,  12, 1, 8},
	{ 13000000, 456000000,  456,  13, 1, 8},
	{ 16800000, 456000000,  380,  14, 1, 8},
	{ 19200000, 456000000,  380,  16, 1, 8},
	{ 26000000, 456000000,  456,  26, 1, 8},

	/* 312 MHz */
	{ 12000000, 312000000,  312,  12, 1, 8},
	{ 13000000, 312000000,  312,  13, 1, 8},
	{ 16800000, 312000000,  260,  14, 1, 8},
	{ 19200000, 312000000,  260,  16, 1, 8},
	{ 26000000, 312000000,  312,  26, 1, 8},

	{ 0, 0, 0, 0, 0, 0 },
};

static struct clk tegra_pll_x = {
	.name      = "pll_x",
	.flags     = PLL_HAS_CPCON | PLL_ALT_MISC_REG | PLLX,
	.ops       = &tegra_pll_ops,
	.reg       = 0xe0,
	.parent    = &tegra_pll_ref,
	.max_rate  = 1000000000,
	.u.pll = {
		.input_min = 2000000,
		.input_max = 31000000,
		.cf_min    = 1000000,
		.cf_max    = 6000000,
		.vco_min   = 20000000,
		.vco_max   = 1200000000,
		.freq_table = tegra_pll_x_freq_table,
		.lock_delay = 300,
	},
};

static struct clk tegra_pll_x_out0 = {
	.name      = "pll_x_out0",
	.ops       = &tegra_pllx_div_ops,
	.flags     = DIV_2 | PLLX,
	.parent    = &tegra_pll_x,
	.max_rate  = 1000000000,
};

/* dap_mclk1, belongs to the cdev1 pingroup. */
static struct clk tegra_dev1_clk = {
	.name      = "clk_dev1",
	.ops       = &tegra_cdev_clk_ops,
	.rate      = 26000000,
	.max_rate  = 26000000,
	.u.periph  = {
		.clk_num = 94,
	},
};

/* dap_mclk2, belongs to the cdev2 pingroup. */
static struct clk tegra_dev2_clk = {
	.name      = "clk_dev2",
	.ops       = &tegra_cdev_clk_ops,
	.rate      = 26000000,
	.max_rate  = 26000000,
	.u.periph  = {
		.clk_num   = 93,
	},
};

/* initialized before peripheral clocks */
static struct clk_mux_sel mux_audio_sync_clk[AUDIO_SYNC_SOURCE_MASK+1+1];
static const struct audio_sources {
	const char *name;
	int value;
} mux_audio_sync_clk_sources[] = {
	{ .name = "spdif_in", .value = 0 },
	{ .name = "i2s0", .value = 1 },
	{ .name = "i2s1", .value = 2 },
	{ .name = "i2s2", .value = 3 },
	{ .name = "i2s3", .value = 4 },
	{ .name = "i2s4", .value = 5 },
	{ .name = "pll_a_out0", .value = 6 },
#if 0 /* FIXME: not implemented */
	{ .name = "ext_vimclk", .value = 7 },
#endif
#if 1 /* FIXME: for debugging on tegra2 - to be removed */
	{ .name = "pll_a_out0", .value = 7 },
	{ .name = "pll_a_out0", .value = 8 },
	{ .name = "pll_a_out0", .value = 9 },
	{ .name = "pll_a_out0", .value =10 },
	{ .name = "pll_a_out0", .value =11 },
	{ .name = "pll_a_out0", .value =12 },
	{ .name = "pll_a_out0", .value =13 },
	{ .name = "pll_a_out0", .value =14 },
	{ .name = "pll_a_out0", .value =15 },
#endif
	{ 0, 0 }
};

#define AUDIO_SYNC_CLK(_id, _index)			\
	{						\
		.name      = #_id,			\
		.inputs    = mux_audio_sync_clk,	\
		.reg       = 0x4A0 + (_index) * 4,	\
		.max_rate  = 24000000,			\
		.ops       = &tegra_audio_sync_clk_ops	\
	}
static struct clk tegra_clk_audio_list[] = {
	AUDIO_SYNC_CLK(audio0, 0),
	AUDIO_SYNC_CLK(audio1, 1),
	AUDIO_SYNC_CLK(audio2, 2),
	AUDIO_SYNC_CLK(audio3, 3),
	AUDIO_SYNC_CLK(audio4, 4),
	AUDIO_SYNC_CLK(audio, 5),	/* SPDIF */
};

#define AUDIO_SYNC_2X_CLK(_id, _index)				\
	{							\
		.name      = #_id "_2x",			\
		.flags     = PERIPH_NO_RESET,			\
		.max_rate  = 48000000,				\
		.ops       = &tegra_clk_double_ops,		\
		.reg       = 0x49C,				\
		.reg_shift = 24 + (_index),			\
		.parent    = &tegra_clk_audio_list[(_index)],	\
		.u.periph = {					\
			.clk_num = 113 + (_index),		\
		},						\
	}
static struct clk tegra_clk_audio_2x_list[] = {
	AUDIO_SYNC_2X_CLK(audio0, 0),
	AUDIO_SYNC_2X_CLK(audio1, 1),
	AUDIO_SYNC_2X_CLK(audio2, 2),
	AUDIO_SYNC_2X_CLK(audio3, 3),
	AUDIO_SYNC_2X_CLK(audio4, 4),
	AUDIO_SYNC_2X_CLK(audio, 5),	/* SPDIF */
};

#define MUX_I2S_SPDIF(_id, _index)					\
static struct clk_mux_sel mux_pllaout0_##_id##_2x_pllp_clkm[] = {	\
	{.input = &tegra_pll_a_out0, .value = 0},			\
	{.input = &tegra_clk_audio_2x_list[(_index)], .value = 1},	\
	{.input = &tegra_pll_p, .value = 2},				\
	{.input = &tegra_clk_m, .value = 3},				\
	{ 0, 0},							\
}
MUX_I2S_SPDIF(audio0, 0);
MUX_I2S_SPDIF(audio1, 1);
MUX_I2S_SPDIF(audio2, 2);
MUX_I2S_SPDIF(audio3, 3);
MUX_I2S_SPDIF(audio4, 4);
MUX_I2S_SPDIF(audio, 5);		/* SPDIF */

/* This is called after peripheral clocks are initialized, as the
 * audio_sync clock depends on some of the peripheral clocks.
 */

static void init_audio_sync_clock_mux(void)
{
	int i;
	struct clk_mux_sel *sel = mux_audio_sync_clk;
	const struct audio_sources *src = mux_audio_sync_clk_sources;

	for (i = 0; src->name; i++, sel++, src++) {
		sel->input = tegra_get_clock_by_name(src->name);
		if (!sel->input)
			pr_err("%s: could not find clk %s\n", __func__,
				src->name);
		sel->value = src->value;
	}
}

static struct clk_mux_sel mux_cclk[] = {
	{ .input = &tegra_clk_m,	.value = 0},
	{ .input = &tegra_pll_c,	.value = 1},
	{ .input = &tegra_clk_32k,	.value = 2},
	{ .input = &tegra_pll_m,	.value = 3},
	{ .input = &tegra_pll_p,	.value = 4},
	{ .input = &tegra_pll_p_out4,	.value = 5},
	{ .input = &tegra_pll_p_out3,	.value = 6},
	/* { .input = &tegra_clk_d,	.value = 7}, - no use on tegra3 */
	{ .input = &tegra_pll_x_out0,	.value = 8},
	{ 0, 0},
};

static struct clk_mux_sel mux_sclk[] = {
	{ .input = &tegra_clk_m,	.value = 0},
	{ .input = &tegra_pll_c_out1,	.value = 1},
	{ .input = &tegra_pll_p_out4,	.value = 2},
	{ .input = &tegra_pll_p_out3,	.value = 3},
	{ .input = &tegra_pll_p_out2,	.value = 4},
	/* { .input = &tegra_clk_d,	.value = 5}, - no use on tegra3 */
	{ .input = &tegra_clk_32k,	.value = 6},
	{ .input = &tegra_pll_m_out1,	.value = 7},
	{ 0, 0},
};

static struct clk tegra_clk_cclk = {
	.name	= "cclk",
	.inputs	= mux_cclk,
	.reg	= 0x20,
	.ops	= &tegra_super_ops,
	.max_rate = 1000000000,
};

static struct clk tegra_clk_sclk = {
	.name	= "sclk",
	.inputs	= mux_sclk,
	.reg	= 0x28,
	.ops	= &tegra_super_ops,
	.max_rate = 240000000,
	.u.shared_bus = {
		.min_rate = 120000000,
	},
};

static struct clk tegra_clk_virtual_cpu = {
	.name      = "cpu",
	.parent    = &tegra_clk_cclk,
	.ops       = &tegra_cpu_ops,
	.max_rate  = 1000000000,
	.u.cpu = {
		.main      = &tegra_pll_x_out0,
		.backup    = &tegra_pll_p,
		.lp_max_rate = 456000000,
	},
};

static struct clk tegra_clk_cop = {
	.name      = "cop",
	.parent    = &tegra_clk_sclk,
	.ops       = &tegra_cop_ops,
	.max_rate  = 240000000,
};

static struct clk tegra_clk_hclk = {
	.name		= "hclk",
	.flags		= DIV_BUS,
	.parent		= &tegra_clk_sclk,
	.reg		= 0x30,
	.reg_shift	= 4,
	.ops		= &tegra_bus_ops,
	.max_rate       = 240000000,
};

static struct clk tegra_clk_pclk = {
	.name		= "pclk",
	.flags		= DIV_BUS,
	.parent		= &tegra_clk_hclk,
	.reg		= 0x30,
	.reg_shift	= 0,
	.ops		= &tegra_bus_ops,
	.max_rate       = 120000000,
};

static struct clk tegra_clk_blink = {
	.name		= "blink",
	.parent		= &tegra_clk_32k,
	.reg		= 0x40,
	.ops		= &tegra_blink_clk_ops,
	.max_rate	= 32768,
};

static struct clk_mux_sel mux_pllm_pllc_pllp_plla[] = {
	{ .input = &tegra_pll_m, .value = 0},
	{ .input = &tegra_pll_c, .value = 1},
	{ .input = &tegra_pll_p, .value = 2},
	{ .input = &tegra_pll_a_out0, .value = 3},
	{ 0, 0},
};

static struct clk_mux_sel mux_pllm_pllc_pllp_clkm[] = {
	{ .input = &tegra_pll_m, .value = 0},
	{ .input = &tegra_pll_c, .value = 1},
	{ .input = &tegra_pll_p, .value = 2},
	{ .input = &tegra_clk_m, .value = 3},
	{ 0, 0},
};

static struct clk_mux_sel mux_pllp_pllc_pllm_clkm[] = {
	{ .input = &tegra_pll_p, .value = 0},
	{ .input = &tegra_pll_c, .value = 1},
	{ .input = &tegra_pll_m, .value = 2},
	{ .input = &tegra_clk_m, .value = 3},
	{ 0, 0},
};

static struct clk_mux_sel mux_pllp_plld_pllc_clkm[] = {
	{.input = &tegra_pll_p, .value = 0},
	{.input = &tegra_pll_d_out0, .value = 1},
	{.input = &tegra_pll_c, .value = 2},
	{.input = &tegra_clk_m, .value = 3},
	{ 0, 0},
};

static struct clk_mux_sel mux_pllp_pllm_plld_plla_pllc_plld2_clkm[] = {
	{.input = &tegra_pll_p, .value = 0},
	{.input = &tegra_pll_m, .value = 1},
	{.input = &tegra_pll_d_out0, .value = 2},
	{.input = &tegra_pll_a_out0, .value = 3},
	{.input = &tegra_pll_c, .value = 4},
	{.input = &tegra_pll_d2_out0, .value = 5},
	{.input = &tegra_clk_m, .value = 6},
	{ 0, 0},
};

static struct clk_mux_sel mux_plla_pllc_pllp_clkm[] = {
	{ .input = &tegra_pll_a_out0, .value = 0},
	{ .input = &tegra_pll_c, .value = 1},
	{ .input = &tegra_pll_p, .value = 2},
	{ .input = &tegra_clk_m, .value = 3},
	{ 0, 0},
};

static struct clk_mux_sel mux_pllp_pllc_clk32_clkm[] = {
	{.input = &tegra_pll_p,     .value = 0},
	{.input = &tegra_pll_c,     .value = 1},
	{.input = &tegra_clk_32k,   .value = 2},
	{.input = &tegra_clk_m,     .value = 3},
	{ 0, 0},
};

static struct clk_mux_sel mux_pllp_pllc_pllm[] = {
	{.input = &tegra_pll_p,     .value = 0},
	{.input = &tegra_pll_c,     .value = 1},
	{.input = &tegra_pll_m,     .value = 2},
	{ 0, 0},
};

static struct clk_mux_sel mux_clk_m[] = {
	{ .input = &tegra_clk_m, .value = 0},
	{ 0, 0},
};

static struct clk_mux_sel mux_pllp_out3[] = {
	{ .input = &tegra_pll_p_out3, .value = 0},
	{ 0, 0},
};

static struct clk_mux_sel mux_plld_out0[] = {
	{ .input = &tegra_pll_d_out0, .value = 0},
	{ 0, 0},
};

static struct clk_mux_sel mux_clk_32k[] = {
	{ .input = &tegra_clk_32k, .value = 0},
	{ 0, 0},
};

#define PERIPH_CLK(_name, _dev, _con, _clk_num, _reg, _max, _inputs, _flags) \
	{						\
		.name      = _name,			\
		.lookup    = {				\
			.dev_id    = _dev,		\
			.con_id	   = _con,		\
		},					\
		.ops       = &tegra_periph_clk_ops,	\
		.reg       = _reg,			\
		.inputs    = _inputs,			\
		.flags     = _flags,			\
		.max_rate  = _max,			\
		.u.periph = {				\
			.clk_num   = _clk_num,		\
		},					\
	}

#define SHARED_CLK(_name, _dev, _con, _parent)		\
	{						\
		.name      = _name,			\
		.lookup    = {				\
			.dev_id    = _dev,		\
			.con_id    = _con,		\
		},					\
		.ops       = &tegra_clk_shared_bus_ops,	\
		.parent = _parent,			\
	}

struct clk tegra_list_clks[] = {
	PERIPH_CLK("rtc",	"rtc-tegra",		NULL,	4,	0,	32768,     mux_clk_32k,			PERIPH_NO_RESET),
	PERIPH_CLK("timer",	"timer",		NULL,	5,	0,	26000000,  mux_clk_m,			0),
	PERIPH_CLK("i2s0",      "i2s.4",		NULL,   30,	0x1d8,	26000000,  mux_pllaout0_audio0_2x_pllp_clkm,	MUX | DIV_U71),
	PERIPH_CLK("i2s1",	"i2s.0",		NULL,	11,	0x100,	26000000,  mux_pllaout0_audio1_2x_pllp_clkm,	MUX | DIV_U71),
	PERIPH_CLK("i2s2",	"i2s.1",		NULL,	18,	0x104,	26000000,  mux_pllaout0_audio2_2x_pllp_clkm,	MUX | DIV_U71),
	PERIPH_CLK("i2s3",      "i2s.2",		NULL,   101,	0x3bc,	26000000,  mux_pllaout0_audio3_2x_pllp_clkm,	MUX | DIV_U71),
	PERIPH_CLK("i2s4",      "i2s.3",		NULL,   102,	0x3c0,	26000000,  mux_pllaout0_audio4_2x_pllp_clkm,	MUX | DIV_U71),
	PERIPH_CLK("spdif_out",	"spdif_out",		NULL,	10,	0x108,	100000000, mux_pllaout0_audio_2x_pllp_clkm,	MUX | DIV_U71),
	PERIPH_CLK("spdif_in",	"spdif_in",		NULL,	10,	0x10c,	100000000, mux_pllp_pllc_pllm,		MUX | DIV_U71),
	PERIPH_CLK("pwm",	"pwm",			NULL,	17,	0x110,	432000000, mux_pllp_pllc_clk32_clkm,	MUX_PWM | DIV_U71),
	PERIPH_CLK("d_audio",	"d_audio",		NULL,   106,	0x3d0,	48000000,  mux_plla_pllc_pllp_clkm,	MUX | DIV_U71),
	PERIPH_CLK("dam0",	"dam.0",		NULL,   108,	0x3d8,	48000000,  mux_plla_pllc_pllp_clkm,	MUX | DIV_U71),
	PERIPH_CLK("dam1",	"dam.1",		NULL,   109,	0x3dc,	48000000,  mux_plla_pllc_pllp_clkm,	MUX | DIV_U71),
	PERIPH_CLK("dam2",	"dam.2",		NULL,   110,	0x3e0,	48000000,  mux_plla_pllc_pllp_clkm,	MUX | DIV_U71),
	PERIPH_CLK("hda",	"hda",			NULL,   125,	0x428,	48000000,  mux_pllp_pllc_pllm_clkm,	MUX | DIV_U71),
	PERIPH_CLK("xio",	"xio",			NULL,	45,	0x120,	150000000, mux_pllp_pllc_pllm_clkm,	MUX | DIV_U71),
	PERIPH_CLK("twc",	"twc",			NULL,	16,	0x12c,	150000000, mux_pllp_pllc_pllm_clkm,	MUX | DIV_U71),
	PERIPH_CLK("sbc1",	"spi_tegra.0",		NULL,	41,	0x134,	160000000, mux_pllp_pllc_pllm_clkm,	MUX | DIV_U71),
	PERIPH_CLK("sbc2",	"spi_tegra.1",		NULL,	44,	0x118,	160000000, mux_pllp_pllc_pllm_clkm,	MUX | DIV_U71),
	PERIPH_CLK("sbc3",	"spi_tegra.2",		NULL,	46,	0x11c,	160000000, mux_pllp_pllc_pllm_clkm,	MUX | DIV_U71),
	PERIPH_CLK("sbc4",	"spi_tegra.3",		NULL,	68,	0x1b4,	160000000, mux_pllp_pllc_pllm_clkm,	MUX | DIV_U71),
	PERIPH_CLK("sbc5",	"spi_tegra.4",		NULL,	104,	0x3c8,	160000000, mux_pllp_pllc_pllm_clkm,	MUX | DIV_U71),
	PERIPH_CLK("sbc6",	"spi_tegra.5",		NULL,	105,	0x3cc,	160000000, mux_pllp_pllc_pllm_clkm,	MUX | DIV_U71),
	PERIPH_CLK("sata_oob",	"sata",			NULL,	123,	0x420,	100000000, mux_pllp_pllc_pllm_clkm,	MUX | DIV_U71),
	PERIPH_CLK("sata",	"sata",			NULL,	124,	0x424,	100000000, mux_pllp_pllc_pllm_clkm,	MUX | DIV_U71),
	PERIPH_CLK("ndflash",	"tegra_nand",		NULL,	13,	0x160,	164000000, mux_pllp_pllc_pllm_clkm,	MUX | DIV_U71), /* scales with voltage */
	PERIPH_CLK("vfir",	"vfir",			NULL,	7,	0x168,	72000000,  mux_pllp_pllc_pllm_clkm,	MUX | DIV_U71),
	PERIPH_CLK("sdmmc1",	"sdhci-tegra.0",	NULL,	14,	0x150,	52000000,  mux_pllp_pllc_pllm_clkm,	MUX | DIV_U71), /* scales with voltage */
	PERIPH_CLK("sdmmc2",	"sdhci-tegra.1",	NULL,	9,	0x154,	52000000,  mux_pllp_pllc_pllm_clkm,	MUX | DIV_U71), /* scales with voltage */
	PERIPH_CLK("sdmmc3",	"sdhci-tegra.2",	NULL,	69,	0x1bc,	52000000,  mux_pllp_pllc_pllm_clkm,	MUX | DIV_U71), /* scales with voltage */
	PERIPH_CLK("sdmmc4",	"sdhci-tegra.3",	NULL,	15,	0x164,	52000000,  mux_pllp_pllc_pllm_clkm,	MUX | DIV_U71), /* scales with voltage */
	PERIPH_CLK("vcp",	"tegra-avp",		"vcp",	29,	0,	250000000, mux_clk_m, 			0),
	PERIPH_CLK("bsea",	"tegra-avp",		"bsea",	62,	0,	250000000, mux_clk_m, 			0),
	PERIPH_CLK("vde",	"tegra-avp",		"vde",	61,	0x1c8,	250000000, mux_pllp_pllc_pllm_clkm,	MUX | DIV_U71), /* scales with voltage and process_id */
	PERIPH_CLK("csite",	"csite",		NULL,	73,	0x1d4,	144000000, mux_pllp_pllc_pllm_clkm,	MUX | DIV_U71), /* max rate ??? */
	/* FIXME: what is la? */
	PERIPH_CLK("la",	"la",			NULL,	76,	0x1f8,	26000000,  mux_pllp_pllc_pllm_clkm,	MUX | DIV_U71),
	PERIPH_CLK("owr",	"tegra_w1",		NULL,	71,	0x1cc,	26000000,  mux_pllp_pllc_pllm_clkm,	MUX | DIV_U71),
	PERIPH_CLK("nor",	"nor",			NULL,	42,	0x1d0,	92000000,  mux_pllp_pllc_pllm_clkm,	MUX | DIV_U71), /* requires min voltage */
	PERIPH_CLK("mipi",	"mipi",			NULL,	50,	0x174,	60000000,  mux_pllp_pllc_pllm_clkm,	MUX | DIV_U71), /* scales with voltage */
	PERIPH_CLK("i2c1",	"tegra-i2c.0",		NULL,	12,	0x124,	26000000,  mux_pllp_pllc_pllm_clkm,	MUX | DIV_U16),
	PERIPH_CLK("i2c2",	"tegra-i2c.1",		NULL,	54,	0x198,	26000000,  mux_pllp_pllc_pllm_clkm,	MUX | DIV_U16),
	PERIPH_CLK("i2c3",	"tegra-i2c.2",		NULL,	67,	0x1b8,	26000000,  mux_pllp_pllc_pllm_clkm,	MUX | DIV_U16),
	PERIPH_CLK("i2c4",	"tegra-i2c.3",		NULL,	47,	0x128,	26000000,  mux_pllp_pllc_pllm_clkm,	MUX | DIV_U16),
	PERIPH_CLK("i2c5",	"tegra-i2c.4",		NULL,	103,	0x3c4,	26000000,  mux_pllp_pllc_pllm_clkm,	MUX | DIV_U16),
	PERIPH_CLK("i2c1_i2c",	"tegra-i2c.0",		"i2c",	0,	0,	72000000,  mux_pllp_out3,		0),
	PERIPH_CLK("i2c2_i2c",	"tegra-i2c.1",		"i2c",	0,	0,	72000000,  mux_pllp_out3,		0),
	PERIPH_CLK("i2c3_i2c",	"tegra-i2c.2",		"i2c",	0,	0,	72000000,  mux_pllp_out3,		0),
	PERIPH_CLK("i2c4_i2c",	"tegra-i2c.3",		"i2c",	0,	0,	72000000,  mux_pllp_out3,		0),
	PERIPH_CLK("i2c5_i2c",	"tegra-i2c.4",		"i2c",	0,	0,	72000000,  mux_pllp_out3,		0),
	PERIPH_CLK("uarta",	"uart.0",		NULL,	6,	0x178,	600000000, mux_pllp_pllc_pllm_clkm,	MUX),
	PERIPH_CLK("uartb",	"uart.1",		NULL,	7,	0x17c,	600000000, mux_pllp_pllc_pllm_clkm,	MUX),
	PERIPH_CLK("uartc",	"uart.2",		NULL,	55,	0x1a0,	600000000, mux_pllp_pllc_pllm_clkm,	MUX),
	PERIPH_CLK("uartd",	"uart.3",		NULL,	65,	0x1c0,	600000000, mux_pllp_pllc_pllm_clkm,	MUX),
	PERIPH_CLK("uarte",	"uart.4",		NULL,	66,	0x1c4,	600000000, mux_pllp_pllc_pllm_clkm,	MUX),
	PERIPH_CLK("3d",	"3d",			NULL,	24,	0x158,	300000000, mux_pllm_pllc_pllp_plla,	MUX | DIV_U71 | PERIPH_MANUAL_RESET), /* scales with voltage and process_id */
	PERIPH_CLK("3d2",       "3d2",			NULL,	98,	0x3b0,	300000000, mux_pllm_pllc_pllp_plla,	MUX | DIV_U71 | PERIPH_MANUAL_RESET), /* scales with voltage and process_id */
	PERIPH_CLK("2d",	"2d",			NULL,	21,	0x15c,	300000000, mux_pllm_pllc_pllp_plla,	MUX | DIV_U71), /* scales with voltage and process_id */
	PERIPH_CLK("vi",	"tegra_camera",		"vi",	20,	0x148,	150000000, mux_pllm_pllc_pllp_plla,	MUX | DIV_U71), /* scales with voltage and process_id */
	PERIPH_CLK("vi_sensor",	"tegra_camera",		"vi_sensor",	20,	0x1a8,	150000000, mux_pllm_pllc_pllp_plla,	MUX | DIV_U71 | PERIPH_NO_RESET), /* scales with voltage and process_id */
	PERIPH_CLK("epp",	"epp",			NULL,	19,	0x16c,	300000000, mux_pllm_pllc_pllp_plla,	MUX | DIV_U71), /* scales with voltage and process_id */
	PERIPH_CLK("mpe",	"mpe",			NULL,	60,	0x170,	250000000, mux_pllm_pllc_pllp_plla,	MUX | DIV_U71), /* scales with voltage and process_id */
	PERIPH_CLK("host1x",	"host1x",		NULL,	28,	0x180,	166000000, mux_pllm_pllc_pllp_plla,	MUX | DIV_U71), /* scales with voltage and process_id */
	PERIPH_CLK("cve",	"cve",			NULL,	49,	0x140,	250000000, mux_pllp_plld_pllc_clkm,	MUX | DIV_U71), /* requires min voltage */
	PERIPH_CLK("tvo",	"tvo",			NULL,	49,	0x188,	250000000, mux_pllp_plld_pllc_clkm,	MUX | DIV_U71), /* requires min voltage */
	PERIPH_CLK("hdmi",	"hdmi",			NULL,	51,	0x18c,	600000000, mux_pllp_pllm_plld_plla_pllc_plld2_clkm,	MUX8 | DIV_U71), /* requires min voltage */
	PERIPH_CLK("tvdac",	"tvdac",		NULL,	53,	0x194,	250000000, mux_pllp_plld_pllc_clkm,	MUX | DIV_U71), /* requires min voltage */
	PERIPH_CLK("disp1",	"tegradc.0",		NULL,	27,	0x138,	600000000, mux_pllp_pllm_plld_plla_pllc_plld2_clkm,	MUX8), /* scales with voltage and process_id */
	PERIPH_CLK("disp2",	"tegradc.1",		NULL,	26,	0x13c,	600000000, mux_pllp_pllm_plld_plla_pllc_plld2_clkm,	MUX8), /* scales with voltage and process_id */
	PERIPH_CLK("usbd",	"fsl-tegra-udc",	NULL,	22,	0,	480000000, mux_clk_m,			0), /* requires min voltage */
	PERIPH_CLK("usb2",	"tegra-ehci.1",		NULL,	58,	0,	480000000, mux_clk_m,			0), /* requires min voltage */
	PERIPH_CLK("usb3",	"tegra-ehci.2",		NULL,	59,	0,	480000000, mux_clk_m,			0), /* requires min voltage */
	PERIPH_CLK("emc",	"emc",			NULL,	57,	0x19c,	800000000, mux_pllm_pllc_pllp_clkm,	MUX | DIV_U71 | PERIPH_EMC_ENB),
	PERIPH_CLK("dsi",	"dsi",			NULL,	48,	0,	500000000, mux_plld_out0,		0), /* scales with voltage */
	PERIPH_CLK("csi",	"tegra_camera",		"csi",	52,	0,	72000000,  mux_pllp_out3,		0),
	PERIPH_CLK("isp",	"tegra_camera",		"isp",	23,	0,	150000000, mux_clk_m,			0), /* same frequency as VI */
	PERIPH_CLK("csus",	"tegra_camera",		"csus",	92,	0,	150000000, mux_clk_m,			PERIPH_NO_RESET),

	SHARED_CLK("avp.sclk",	"tegra-avp",		"sclk",	&tegra_clk_sclk),
};

#define CLK_DUPLICATE(_name, _dev, _con)		\
	{						\
		.name	= _name,			\
		.lookup	= {				\
			.dev_id	= _dev,			\
			.con_id		= _con,		\
		},					\
	}

/* Some clocks may be used by different drivers depending on the board
 * configuration.  List those here to register them twice in the clock lookup
 * table under two names.
 */
struct clk_duplicate tegra_clk_duplicates[] = {
	CLK_DUPLICATE("uarta",	"tegra_uart.0",	NULL),
	CLK_DUPLICATE("uartb",	"tegra_uart.1",	NULL),
	CLK_DUPLICATE("uartc",	"tegra_uart.2",	NULL),
	CLK_DUPLICATE("uartd",	"tegra_uart.3",	NULL),
	CLK_DUPLICATE("uarte",	"tegra_uart.4",	NULL),
	CLK_DUPLICATE("usbd", "utmip-pad", NULL),
	CLK_DUPLICATE("usbd", "tegra-ehci.0", NULL),
	CLK_DUPLICATE("usbd", "tegra-otg", NULL),
	CLK_DUPLICATE("hdmi", "tegradc.0", "hdmi"),
	CLK_DUPLICATE("hdmi", "tegradc.1", "hdmi"),
	CLK_DUPLICATE("pwm", "tegra_pwm.0", NULL),
	CLK_DUPLICATE("pwm", "tegra_pwm.1", NULL),
	CLK_DUPLICATE("pwm", "tegra_pwm.2", NULL),
	CLK_DUPLICATE("pwm", "tegra_pwm.3", NULL),
	CLK_DUPLICATE("host1x", "tegra_grhost", "host1x"),
	CLK_DUPLICATE("2d", "tegra_grhost", "gr2d"),
	CLK_DUPLICATE("3d", "tegra_grhost", "gr3d"),
	CLK_DUPLICATE("epp", "tegra_grhost", "epp"),
	CLK_DUPLICATE("mpe", "tegra_grhost", "mpe"),
	CLK_DUPLICATE("cop", "tegra-avp", "cop"),
};

struct clk *tegra_ptr_clks[] = {
	&tegra_clk_32k,
	&tegra_clk_m,
	&tegra_pll_ref,
	&tegra_pll_m,
	&tegra_pll_m_out1,
	&tegra_pll_c,
	&tegra_pll_c_out1,
	&tegra_pll_p,
	&tegra_pll_p_out1,
	&tegra_pll_p_out2,
	&tegra_pll_p_out3,
	&tegra_pll_p_out4,
	&tegra_pll_a,
	&tegra_pll_a_out0,
	&tegra_pll_d,
	&tegra_pll_d_out0,
	&tegra_pll_d2,
	&tegra_pll_d2_out0,
	&tegra_pll_u,
	&tegra_pll_x,
	&tegra_pll_x_out0,
	&tegra_clk_cclk,
	&tegra_clk_sclk,
	&tegra_clk_hclk,
	&tegra_clk_pclk,
	&tegra_dev1_clk,
	&tegra_dev2_clk,
	&tegra_clk_virtual_cpu,
	&tegra_clk_blink,
	&tegra_clk_cop,
};

static void tegra3_init_one_clock(struct clk *c)
{
	clk_init(c);
	if (!c->lookup.dev_id && !c->lookup.con_id)
		c->lookup.con_id = c->name;
	c->lookup.clk = c;
	clkdev_add(&c->lookup);
}

//FIXME: void __init tegra3_init_clocks(void)
void __init tegra_soc_init_clocks(void)
{
	int i;
	struct clk *c;

	for (i = 0; i < ARRAY_SIZE(tegra_ptr_clks); i++)
		tegra3_init_one_clock(tegra_ptr_clks[i]);

	for (i = 0; i < ARRAY_SIZE(tegra_list_clks); i++)
		tegra3_init_one_clock(&tegra_list_clks[i]);

	for (i = 0; i < ARRAY_SIZE(tegra_clk_duplicates); i++) {
		c = tegra_get_clock_by_name(tegra_clk_duplicates[i].name);
		if (!c) {
			pr_err("%s: Unknown duplicate clock %s\n", __func__,
				tegra_clk_duplicates[i].name);
			continue;
		}

		tegra_clk_duplicates[i].lookup.clk = c;
		clkdev_add(&tegra_clk_duplicates[i].lookup);
	}

	init_audio_sync_clock_mux();
	for (i = 0; i < ARRAY_SIZE(tegra_clk_audio_list); i++)
		tegra3_init_one_clock(&tegra_clk_audio_list[i]);
	for (i = 0; i < ARRAY_SIZE(tegra_clk_audio_2x_list); i++)
		tegra3_init_one_clock(&tegra_clk_audio_2x_list[i]);
}

#ifdef CONFIG_PM
static u32 clk_rst_suspend[RST_DEVICES_NUM + CLK_OUT_ENB_NUM +
			   PERIPH_CLK_SOURCE_NUM + 15];

void tegra_clk_suspend(void)
{
	unsigned long off;
	u32 *ctx = clk_rst_suspend;

	*ctx++ = clk_readl(OSC_CTRL) & OSC_CTRL_MASK;
	*ctx++ = clk_readl(tegra_pll_c.reg + PLL_BASE);
	*ctx++ = clk_readl(tegra_pll_c.reg + PLL_MISC(&tegra_pll_c));
	*ctx++ = clk_readl(tegra_pll_a.reg + PLL_BASE);
	*ctx++ = clk_readl(tegra_pll_a.reg + PLL_MISC(&tegra_pll_a));

	*ctx++ = clk_readl(tegra_pll_m_out1.reg);
	*ctx++ = clk_readl(tegra_pll_a_out0.reg);
	*ctx++ = clk_readl(tegra_pll_c_out1.reg);

	*ctx++ = clk_readl(tegra_clk_cclk.reg);
	*ctx++ = clk_readl(tegra_clk_cclk.reg + SUPER_CLK_DIVIDER);

	*ctx++ = clk_readl(tegra_clk_sclk.reg);
	*ctx++ = clk_readl(tegra_clk_sclk.reg + SUPER_CLK_DIVIDER);
	*ctx++ = clk_readl(tegra_clk_pclk.reg);

	for (off = PERIPH_CLK_SOURCE_I2S1; off <= PERIPH_CLK_SOURCE_OSC;
			off += 4) {
		if (off == PERIPH_CLK_SOURCE_EMC)
			continue;
		*ctx++ = clk_readl(off);
	}
	for (off = PERIPH_CLK_SOURCE_G3D2; off <= PERIPH_CLK_SOURCE_SE;
			off+=4) {
		*ctx++ = clk_readl(off);
	}

	*ctx++ = clk_readl(RST_DEVICES_L);
	*ctx++ = clk_readl(RST_DEVICES_H);
	*ctx++ = clk_readl(RST_DEVICES_U);
	*ctx++ = clk_readl(RST_DEVICES_V);
	*ctx++ = clk_readl(RST_DEVICES_W);

	*ctx++ = clk_readl(CLK_OUT_ENB_L);
	*ctx++ = clk_readl(CLK_OUT_ENB_H);
	*ctx++ = clk_readl(CLK_OUT_ENB_U);
	*ctx++ = clk_readl(CLK_OUT_ENB_V);
	*ctx++ = clk_readl(CLK_OUT_ENB_W);

	*ctx++ = clk_readl(MISC_CLK_ENB);
	*ctx++ = clk_readl(CLK_MASK_ARM);
}

void tegra_clk_resume(void)
{
	unsigned long off;
	const u32 *ctx = clk_rst_suspend;
	u32 val;

	val = clk_readl(OSC_CTRL) & ~OSC_CTRL_MASK;
	val |= *ctx++;
	clk_writel(val, OSC_CTRL);

	/* FIXME: add plld, and wait for lock */
	clk_writel(*ctx++, tegra_pll_c.reg + PLL_BASE);
	clk_writel(*ctx++, tegra_pll_c.reg + PLL_MISC(&tegra_pll_c));
	clk_writel(*ctx++, tegra_pll_a.reg + PLL_BASE);
	clk_writel(*ctx++, tegra_pll_a.reg + PLL_MISC(&tegra_pll_a));
	udelay(300);

	clk_writel(*ctx++, tegra_pll_m_out1.reg);
	clk_writel(*ctx++, tegra_pll_a_out0.reg);
	clk_writel(*ctx++, tegra_pll_c_out1.reg);

	clk_writel(*ctx++, tegra_clk_cclk.reg);
	clk_writel(*ctx++, tegra_clk_cclk.reg + SUPER_CLK_DIVIDER);

	clk_writel(*ctx++, tegra_clk_sclk.reg);
	clk_writel(*ctx++, tegra_clk_sclk.reg + SUPER_CLK_DIVIDER);
	clk_writel(*ctx++, tegra_clk_pclk.reg);

	/* enable all clocks before configuring clock sources */
	clk_writel(0xfdfffff1ul, CLK_OUT_ENB_L);
	clk_writel(0xfefff7f7ul, CLK_OUT_ENB_H);
	clk_writel(0x75f79bfful, CLK_OUT_ENB_U);
	clk_writel(0xfffffffful, CLK_OUT_ENB_V);
	clk_writel(0x00003ffful, CLK_OUT_ENB_W);
	wmb();

	for (off = PERIPH_CLK_SOURCE_I2S1; off <= PERIPH_CLK_SOURCE_OSC;
			off += 4) {
		if (off == PERIPH_CLK_SOURCE_EMC)
			continue;
		clk_writel(*ctx++, off);
	}
	for (off = PERIPH_CLK_SOURCE_G3D2; off <= PERIPH_CLK_SOURCE_SE;
			off += 4) {
		clk_writel(*ctx++, off);
	}
	wmb();

	clk_writel(*ctx++, RST_DEVICES_L);
	clk_writel(*ctx++, RST_DEVICES_H);
	clk_writel(*ctx++, RST_DEVICES_U);
	clk_writel(*ctx++, RST_DEVICES_V);
	clk_writel(*ctx++, RST_DEVICES_W);
	wmb();

	clk_writel(*ctx++, CLK_OUT_ENB_L);
	clk_writel(*ctx++, CLK_OUT_ENB_H);
	clk_writel(*ctx++, CLK_OUT_ENB_U);
	clk_writel(*ctx++, CLK_OUT_ENB_V);
	clk_writel(*ctx++, CLK_OUT_ENB_W);
	wmb();

	clk_writel(*ctx++, MISC_CLK_ENB);
	clk_writel(*ctx++, CLK_MASK_ARM);
}
#endif
