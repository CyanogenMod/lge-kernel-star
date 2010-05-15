/*
 *  arch/arm/mach-tegra/localtimer.c
 *
 *  Copyright (C) 2002 ARM Ltd.
 *  All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/init.h>
#include <linux/smp.h>
#include <linux/clockchips.h>
#include <asm/irq.h>
#include <asm/smp_twd.h>
#include <asm/localtimer.h>

#ifdef CONFIG_USE_ARM_TWD_PRESCALER
#define CPU_FREQ_SCALE_SHIFT   24
#define CPU_FREQ_SCALE_DIVIDER (0x1 << CPU_FREQ_SCALE_SHIFT)
#define CPU_FREQ_SCALE_INIT    125
extern unsigned long twd_prescaler;
#endif

/*
 * Setup the local clock events for a CPU.
 */
void __cpuinit local_timer_setup(struct clock_event_device *evt)
{
#ifdef CONFIG_USE_ARM_TWD_PRESCALER
	if (!twd_prescaler) {
		twd_prescaler = CPU_FREQ_SCALE_INIT - 1;
		smp_wmb();
	}
#endif
	evt->irq = IRQ_LOCALTIMER;
	twd_timer_setup(evt);
}

#ifdef CONFIG_USE_ARM_TWD_PRESCALER
/*
 * Find new prescaler value for cpu frequency changes, so that local timer
 * input frequency is kept at calibration level. Save new value in shadow
 * variable - do not update h/w.
 */
void local_timer_rescale(unsigned long cpu_freq_khz)
{
	static unsigned long cpu_freq_scale_mult = 0;
	unsigned long scale;

	/* 1st call at boot/calibration frequency */
	if (cpu_freq_scale_mult == 0) {
		cpu_freq_scale_mult = ((twd_prescaler + 1) <<
			CPU_FREQ_SCALE_SHIFT) / cpu_freq_khz;
		printk("Local timer scaling factor %lu, shift %d\n",
                       cpu_freq_scale_mult, CPU_FREQ_SCALE_SHIFT);
		return;
	}

	scale = (unsigned long)((((uint64_t)cpu_freq_khz * cpu_freq_scale_mult)
			+ CPU_FREQ_SCALE_DIVIDER -1) >> CPU_FREQ_SCALE_SHIFT);
	twd_prescaler = scale - 1;
}
#endif
