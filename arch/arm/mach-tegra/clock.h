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

#define ENABLE_ON_INIT	0x00000001
#define DIV_U71		0x00000002
#define DIV_U71_FIXED	0x00000004
#define DIV_2		0x00000008
#define PLL_FIXED	0x00000010
#define PLL_HAS_CPCON	0x00000020
#define MUX		0x00000040

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
};

struct clk {
	struct list_head		node;
	struct clk_ops			*ops;
	struct clk			*parent;
	unsigned long			rate;
	u32				flags;
	u32				refcnt;
	const char			*name;
	const char			*dev_id;
	const char			*con_id;
	u32				reg;
	u32				reg_shift;
	unsigned int			clk_num;
	spinlock_t			lock;

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

	/* MUX */
	const struct clk_mux_sel	*inputs;
	u32				sel;
	u32				reg_mask;
};

extern struct clk_lookup tegra_clk_lookups[];
extern struct clk tegra_periph_clks[];
extern struct clk_lookup tegra_periph_clk_lookups[];
extern const int tegra_num_periph_clks;

unsigned long clk_measure_input_freq(void);

#endif
