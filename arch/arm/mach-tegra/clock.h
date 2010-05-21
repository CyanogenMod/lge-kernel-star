/*
 * arch/arm/mach-tegra/include/mach/clock.h
 *
 * Copyright (C) 2010 Google, Inc.
 *
 * Author:
 *	Colin Cross <ccross@google.com>
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

#ifndef __MACH_TEGRA_CLOCK_H
#define __MACH_TEGRA_CLOCK_H

#include <asm/clkdev.h>

#define DIV_BUS		0x00000001
#define DIV_U71		0x00000002
#define DIV_U71_FIXED	0x00000004
#define DIV_2		0x00000008
#define PLL_FIXED	0x00000010
#define PLL_HAS_CPCON	0x00000020
#define MUX		0x00000040
#define PLLD		0x00000080
#define PERIPH_NO_RESET	0x00000100
#define PERIPH_NO_ENB	0x00000200
#define PERIPH_EMC_ENB	0x00000400
#define PERIPH_PMC_RESET 0x00000800
#define ENABLE_ON_INIT	0x10000000
struct clk;

struct clk_mux_sel {
	struct clk	*input;
	u32		value;
};

struct clk_pll_table {
	unsigned long	input_rate;
	unsigned long	output_rate;
	u16		n;
	u16		m;
	u8		p;
	u8		cpcon;
};

struct clk_ops {
	void		(*init)(struct clk *);
	int		(*enable)(struct clk *);
	void		(*disable)(struct clk *);
	void		(*recalc)(struct clk *);
	int		(*set_parent)(struct clk *, struct clk *);
	int		(*set_rate)(struct clk *, unsigned long);
	unsigned long	(*get_rate)(struct clk *);
	long		(*round_rate)(struct clk *, unsigned long);
	unsigned long	(*recalculate_rate)(struct clk *);
};

enum clk_state {
	UNINITIALIZED = 0,
	ON,
	OFF,
};

#ifdef CONFIG_TEGRA_NVRM
struct clk {
	struct list_head	node;
	struct clk_ops		*ops;
	struct clk_lookup	lookup;
	const char		*name;
	u32			module;
	u32			flags;
	unsigned long		rate_min;
	u8			rate_tolerance;
	bool			power;
};	
#else
struct clk {
	/* node for master clocks list */
	struct list_head		node;
	struct list_head		children;	/* list of children */
	struct list_head		sibling;	/* node for children */
#ifdef CONFIG_DEBUG_FS
	struct dentry 			*dent;
	struct dentry 			*parent_dent;
#endif
	struct clk_ops			*ops;
	struct clk			*parent;
	struct clk_lookup		lookup;
	unsigned long			rate;
	u32				flags;
	u32				refcnt;
	const char			*name;
	u32				reg;
	u32				reg_shift;
	unsigned int			clk_num;
	enum clk_state			state;
#ifdef CONFIG_DEBUG_FS
	bool				set;
#endif

	/* PLL */
	unsigned long			input_min;
	unsigned long			input_max;
	unsigned long			cf_min;
	unsigned long			cf_max;
	unsigned long			vco_min;
	unsigned long			vco_max;
	u32				m;
	u32				n;
	u32				p;
	u32				cpcon;
	const struct clk_pll_table	*pll_table;

	/* DIV */
	u32				div;
	u32				mul;

	/* MUX */
	const struct clk_mux_sel	*inputs;
	u32				sel;
	u32				reg_mask;
};
#endif


struct clk_duplicate {
	const char *name;
	struct clk_lookup lookup;
};

void tegra2_init_clocks(void);
void clk_init(struct clk *clk);
struct clk *get_tegra_clock_by_name(const char *name);
unsigned long clk_measure_input_freq(void);
void clk_disable_locked(struct clk *c);
int clk_enable_locked(struct clk *c);
int clk_set_parent_locked(struct clk *c, struct clk *parent);
int clk_reparent(struct clk *c, struct clk *parent);

#endif
