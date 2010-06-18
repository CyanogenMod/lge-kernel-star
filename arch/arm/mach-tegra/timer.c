/*
 * arch/arch/mach-tegra/timer.c
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

#include <linux/init.h>
#include <linux/time.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/clockchips.h>
#include <linux/clocksource.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/cnt32_to_63.h>

#include <asm/mach/time.h>
#include <asm/mach/time.h>
#include <asm/localtimer.h>

#include <mach/iomap.h>
#include <mach/irqs.h>

#include "board.h"
#include "clock.h"

#define CLK_RST_OSC_CTRL 0x50

#define TIMERUS_CNTR_1US 0x10
#define TIMERUS_USEC_CFG 0x14
#define TIMERUS_CNTR_FREEZE 0x4c

#define TIMER1_BASE 0x0
#define TIMER2_BASE 0x8
#define TIMER3_BASE 0x50
#define TIMER4_BASE 0x58

#define TIMER_PTV 0x0
#define TIMER_PCR 0x4

struct tegra_timer;

static void __iomem *timer_reg_base = IO_ADDRESS(TEGRA_TMR1_BASE);

#define timer_writel(value, reg) \
	__raw_writel(value, (u32)timer_reg_base + (reg))
#define timer_readl(reg) \
	__raw_readl((u32)timer_reg_base + (reg))

static int tegra_timer_set_next_event(unsigned long cycles,
					 struct clock_event_device *evt)
{
	u32 reg;

	reg = 0x80000000 | ((cycles > 1) ? (cycles-1) : 0);
	timer_writel(reg, TIMER3_BASE + TIMER_PTV);

	return 0;
}

static void tegra_timer_set_mode(enum clock_event_mode mode,
				    struct clock_event_device *evt)
{
	u32 reg;

	timer_writel(0, TIMER3_BASE + TIMER_PTV);

	switch (mode) {
	case CLOCK_EVT_MODE_PERIODIC:
		reg = 0xC0000000 | ((1000000/HZ)-1);
		timer_writel(reg, TIMER3_BASE + TIMER_PTV);
		break;
	case CLOCK_EVT_MODE_ONESHOT:
		break;
	case CLOCK_EVT_MODE_UNUSED:
	case CLOCK_EVT_MODE_SHUTDOWN:
	case CLOCK_EVT_MODE_RESUME:
		break;
	}
}

static cycle_t tegra_clocksource_read(struct clocksource *cs)
{
	return cnt32_to_63(timer_readl(TIMERUS_CNTR_1US));
}

static struct clock_event_device tegra_clockevent = {
	.name		= "timer0",
	.rating         = 300,
	.features       = CLOCK_EVT_FEAT_ONESHOT | CLOCK_EVT_FEAT_PERIODIC,
	.mult           = 16777,
	.shift		= 24,
	.set_next_event	= tegra_timer_set_next_event,
	.set_mode	= tegra_timer_set_mode,
};

static struct clocksource tegra_clocksource = {
	.name	= "timer_us",
	.rating	= 300,
	.read	= tegra_clocksource_read,
	.mask	= 0x7FFFFFFFFFFFFFFFULL,
	.mult	= 1000,
	.shift	= 0,
	.flags	= CLOCK_SOURCE_IS_CONTINUOUS,
};

unsigned long long sched_clock(void)
{
        return clocksource_cyc2ns(tegra_clocksource.read(&tegra_clocksource),
		tegra_clocksource.mult, tegra_clocksource.shift);
}

static irqreturn_t tegra_timer_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *evt = (struct clock_event_device *)dev_id;
	timer_writel(1<<30, TIMER3_BASE + TIMER_PCR);
	evt->event_handler(evt);
	return IRQ_HANDLED;
}

static struct irqaction tegra_timer_irq = {
	.name		= "timer0",
	.flags		= IRQF_DISABLED | IRQF_TIMER | IRQF_TRIGGER_HIGH,
	.handler	= tegra_timer_interrupt,
	.dev_id		= &tegra_clockevent,
	.irq            = INT_TMR3,
};

static irqreturn_t tegra_lp2wake_interrupt(int irq, void *dev_id)
{
	timer_writel(1<<30, TIMER4_BASE + TIMER_PCR);
	return IRQ_HANDLED;
}

static struct irqaction tegra_lp2wake_irq = {
	.name		= "timer_lp2wake",
	.flags		= IRQF_DISABLED,
	.handler	= tegra_lp2wake_interrupt,
	.dev_id		= NULL,
	.irq		= INT_TMR4,
};

static unsigned long measure_input_freq(unsigned int *m, unsigned int *n)
{
	void __iomem *clk_rst = IO_ADDRESS(TEGRA_CLK_RESET_BASE);
	unsigned long osc = readl(clk_rst + CLK_RST_OSC_CTRL);
	osc >>= 30;

	switch (osc) {
	case 0: if (m && n) { *m=1; *n=13; } return 13000;
	case 1: if (m && n) { *m=5; *n=96; } return 19200;
	case 2: if (m && n) { *m=1; *n=12; } return 12000;
	case 3: if (m && n) { *m=1; *n=26; } return 26000;
	}
	return 0;
}

static void __init tegra_init_timer(void)
{
	unsigned long rate;
	unsigned int m, n;
	int ret;

#ifdef CONFIG_HAVE_ARM_TWD
	twd_base = IO_ADDRESS(TEGRA_ARM_PERIF_BASE + 0x600);
#endif

	rate = measure_input_freq(&m, &n);
	timer_writel(((m-1)<<8 | (n-1)), TIMERUS_USEC_CFG);

	if (clocksource_register(&tegra_clocksource)) {
		printk(KERN_ERR "Failed to register clocksource\n");
		BUG();
	}

	ret = setup_irq(tegra_timer_irq.irq, &tegra_timer_irq);
	if (ret) {
		printk(KERN_ERR "Failed to register timer IRQ: %d\n", ret);
		BUG();
	}

	ret = setup_irq(tegra_lp2wake_irq.irq, &tegra_lp2wake_irq);
	if (ret) {
		printk(KERN_ERR "Failed to register LP2 timer IRQ: %d\n", ret);
		BUG();
	}

	tegra_clockevent.max_delta_ns =
		clockevent_delta2ns(0x1fffffff, &tegra_clockevent);
	tegra_clockevent.min_delta_ns =
		clockevent_delta2ns(0x1, &tegra_clockevent);
	tegra_clockevent.cpumask = cpu_all_mask;
	tegra_clockevent.irq = tegra_timer_irq.irq;
	clockevents_register_device(&tegra_clockevent);

	return;
}

struct sys_timer tegra_timer = {
	.init = tegra_init_timer,
};

void tegra_lp2_set_trigger(unsigned long cycles)
{
	timer_writel(0, TIMER4_BASE + TIMER_PTV);
	if (cycles) {
		u32 reg = 0x80000000ul | min(0x1ffffffful, cycles);
		timer_writel(reg, TIMER4_BASE + TIMER_PTV);
	}
}
EXPORT_SYMBOL(tegra_lp2_set_trigger);
