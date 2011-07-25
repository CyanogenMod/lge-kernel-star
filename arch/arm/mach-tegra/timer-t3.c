/*
 * arch/arch/mach-tegra/timer-t3.c
 *
 * Copyright (c) 2011, NVIDIA Corporation.
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
 */

#include <linux/init.h>
#include <linux/err.h>
#include <linux/sched.h>
#include <linux/time.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/clockchips.h>
#include <linux/clocksource.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/smp.h>
#include <linux/syscore_ops.h>

#include <asm/mach/time.h>
#include <asm/localtimer.h>
#include <asm/sched_clock.h>

#include <mach/iomap.h>
#include <mach/irqs.h>

#include "board.h"
#include "clock.h"
#include "timer.h"

/*
 * Timers usage:
 * TMR1 - used as general cpu timer.
 * TMR2 - used by AVP.
 * TMR3 - used by CPU0 for Lp2 wakeup.
 * TMR4 - used by CPU1 for Lp2 wakeup.
 * TMR5 - used by CPU2 for Lp2 wakeup.
 * TMR6 - used by CPU3 for Lp2 wakeup.
 * TMR7 - Free.
 * TMR8 - Free.
 * TMR9 - Free.
 * TMR10 - used as src for watchdog controller 0.
*/

#define TIMER1_OFFSET (TEGRA_TMR1_BASE-TEGRA_TMR1_BASE)
#define TIMER2_OFFSET (TEGRA_TMR2_BASE-TEGRA_TMR1_BASE)
#define TIMER3_OFFSET (TEGRA_TMR3_BASE-TEGRA_TMR1_BASE)
#define TIMER4_OFFSET (TEGRA_TMR4_BASE-TEGRA_TMR1_BASE)
#define TIMER5_OFFSET (TEGRA_TMR5_BASE-TEGRA_TMR1_BASE)
#define TIMER6_OFFSET (TEGRA_TMR6_BASE-TEGRA_TMR1_BASE)

static void __iomem *timer_reg_base = IO_ADDRESS(TEGRA_TMR1_BASE);

#define timer_writel(value, reg) \
	__raw_writel(value, (u32)timer_reg_base + (reg))
#define timer_readl(reg) \
	__raw_readl((u32)timer_reg_base + (reg))


#if 0
static int lp2_wake_timers[] = {
	TIMER3_OFFSET,
	TIMER4_OFFSET,
	TIMER5_OFFSET,
	TIMER6_OFFSET,
};

static irqreturn_t tegra_lp2wake_interrupt(int irq, void *dev_id)
{
	int cpu = (int)dev_id;
	int timer_base;

	timer_base = lp2_wake_timers[cpu];
	timer_writel(1<<30, timer_base + TIMER_PCR);
	return IRQ_HANDLED;
}

#define LP2_TIMER_IRQ_ACTION(n, i) \
static struct irqaction tegra_lp2wake_irq_cpu##n = { \
	.name		= "tmr_lp2wake_cpu" __stringify(n), \
	.flags		= IRQF_DISABLED, \
	.handler	= tegra_lp2wake_interrupt, \
	.dev_id		= (void*)n, \
	.irq		= i, \
};

#define LP2_TIMER_IRQ_ACTIONS() \
	LP2_TIMER_IRQ_ACTION(0, INT_TMR3); \
	LP2_TIMER_IRQ_ACTION(1, INT_TMR4); \
	LP2_TIMER_IRQ_ACTION(2, INT_TMR5); \
	LP2_TIMER_IRQ_ACTION(3, INT_TMR6);

LP2_TIMER_IRQ_ACTIONS();

#define REGISTER_LP2_WAKE_IRQ(n) \
	ret = setup_irq(tegra_lp2wake_irq_cpu##n.irq, &tegra_lp2wake_irq_cpu##n); \
	if (ret) { \
		printk(KERN_ERR "Failed to register LP2 timer IRQ: " \
			"irq=%d, ret=%d\n", tegra_lp2wake_irq_cpu##n.irq, ret); \
		BUG(); \
	} \
	ret = irq_set_affinity(tegra_lp2wake_irq_cpu##n.irq, cpumask_of(n)); \
	if (ret) { \
		printk(KERN_ERR "Failed to set affinity for LP2 timer IRQ: " \
			"irq=%d, ret=%d\n", tegra_lp2wake_irq_cpu##n.irq, ret); \
		BUG(); \
	}

#define REGISTER_LP2_WAKE_IRQS() \
do { \
	REGISTER_LP2_WAKE_IRQ(0); \
	REGISTER_LP2_WAKE_IRQ(1); \
	REGISTER_LP2_WAKE_IRQ(2); \
	REGISTER_LP2_WAKE_IRQ(3); \
} while (0)

/*
 * To sanity test timer interrupts for cpu 0-3, enable this flag and check
 * /proc/interrupts for timer interrupts. Cpu's 0-3 would have one interrupt
 * counted against them for tmr_lp2wake_cpu0,1,2,3.
 */
#define TEST_LP2_WAKE_TIMERS 0
#if TEST_LP2_WAKE_TIMERS
static void test_lp2_wake_timers(void)
{
	unsigned int cpu;
	unsigned int timer_base;
	unsigned long cycles = 50000;

	for_each_possible_cpu(cpu) {
		timer_base = lp2_wake_timers[cpu];
		timer_writel(0, timer_base + TIMER_PTV);
		if (cycles) {
			u32 reg = 0x80000000ul | min(0x1ffffffful, cycles);
			timer_writel(reg, timer_base + TIMER_PTV);
		}
	}
}
#else
static void test_lp2_wake_timers(void){}
#endif
#endif

void __init tegra3_init_timer(u32 *offset, int *irq)
{
	unsigned long rate = clk_measure_input_freq();
	void __iomem *chip_id = IO_ADDRESS(TEGRA_APB_MISC_BASE) + 0x804;
	unsigned long id;

	switch (rate) {
	case 12000000:
		timer_writel(0x000b, TIMERUS_USEC_CFG);
		break;
	case 13000000:
		timer_writel(0x000c, TIMERUS_USEC_CFG);
		break;
	case 19200000:
		timer_writel(0x045f, TIMERUS_USEC_CFG);
		break;
	case 26000000:
		timer_writel(0x0019, TIMERUS_USEC_CFG);
		break;
	case 16800000:
		timer_writel(0x0453, TIMERUS_USEC_CFG);
		break;
	case 38400000:
		timer_writel(0x04BF, TIMERUS_USEC_CFG);
		break;
	case 48000000:
		timer_writel(0x002F, TIMERUS_USEC_CFG);
		break;
	default:
		WARN(1, "Unknown clock rate");
	}

#if 0
	/* For T30.A01 use INT_TMR_SHARED instead of INT_TMR6. */
	id = readl(chip_id);
	if (((id & 0xFF00) >> 8) == 0x30) {
#ifndef CONFIG_TEGRA_FPGA_PLATFORM
		if (((id >> 16) & 0xf) == 1) {
			tegra_lp2wake_irq_cpu3.irq = INT_TMR_SHARED;
		}
#else
		void __iomem *emu_rev = IO_ADDRESS(TEGRA_APB_MISC_BASE) + 0x860;
		unsigned long reg = readl(emu_rev);
		unsigned long netlist = reg & 0xFFFF;
		unsigned long patch = (reg >> 16) & 0xFF;
		if ((netlist == 12) && (patch < 14)) {
			tegra_lp2wake_irq_cpu3.irq = INT_TMR_SHARED;
		}
#endif
	}

	REGISTER_LP2_WAKE_IRQS();
#endif

#if 0
	test_lp2_wake_timers();
#endif

	*offset = TIMER1_OFFSET;
	*irq = INT_TMR1;
}

#if 0
#ifdef CONFIG_SMP
#define hard_smp_processor_id()						\
	({								\
		unsigned int cpunum;					\
		__asm__("\n"						\
			"1:	mrc p15, 0, %0, c0, c0, 5\n"		\
			"	.pushsection \".alt.smp.init\", \"a\"\n"\
			"	.long	1b\n"				\
			"	mov	%0, #0\n"			\
			"	.popsection"				\
			: "=r" (cpunum));				\
		cpunum &= 0x0F;						\
	})
#define cpu_number()	hard_smp_processor_id()
#else
#define cpu_number()	0
#endif

void tegra_lp2_set_trigger(unsigned long cycles)
{
	int cpu = cpu_number();
	int timer_base;

	timer_base = lp2_wake_timers[cpu];
	timer_writel(0, timer_base + TIMER_PTV);
	if (cycles) {
		u32 reg = 0x80000000ul | min(0x1ffffffful, cycles);
		timer_writel(reg, timer_base + TIMER_PTV);
	}
}
EXPORT_SYMBOL(tegra_lp2_set_trigger);

unsigned long tegra_lp2_timer_remain(void)
{
	int cpu = cpu_number();
	int timer_base;

	timer_base = lp2_wake_timers[cpu];
	return timer_readl(timer_base + TIMER_PCR) & 0x1ffffffful;
}
#endif
