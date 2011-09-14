/*
 *  linux/arch/arm/mach-tegra/platsmp.c
 *
 *  Copyright (C) 2002 ARM Ltd.
 *  All Rights Reserved
 *
 *  Copyright (C) 2009 Palm
 *  All Rights Reserved
 *
 *  Copyright (C) 2010-2011 NVIDIA Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/smp.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/cpumask.h>

#include <asm/hardware/gic.h>
#include <asm/smp_scu.h>

#include <mach/iomap.h>
#include <mach/powergate.h>

#include "pm.h"
#include "clock.h"
#include "reset.h"
#include "sleep.h"

bool tegra_all_cpus_booted;

static DECLARE_BITMAP(tegra_cpu_init_bits, CONFIG_NR_CPUS) __read_mostly;
const struct cpumask *const tegra_cpu_init_mask = to_cpumask(tegra_cpu_init_bits);
#define tegra_cpu_init_map	(*(cpumask_t *)tegra_cpu_init_mask)

#define CLK_RST_CONTROLLER_CLK_CPU_CMPLX \
	(IO_ADDRESS(TEGRA_CLK_RESET_BASE) + 0x4c)
#define CLK_RST_CONTROLLER_RST_CPU_CMPLX_SET \
	(IO_ADDRESS(TEGRA_CLK_RESET_BASE) + 0x340)
#define CLK_RST_CONTROLLER_RST_CPU_CMPLX_CLR \
	(IO_ADDRESS(TEGRA_CLK_RESET_BASE) + 0x344)

#define CPU_CLOCK(cpu)	(0x1<<(8+cpu))
#define CPU_RESET(cpu)	(0x1111ul<<(cpu))

#ifndef CONFIG_ARCH_TEGRA_2x_SOC
#define CLK_RST_CONTROLLER_CLK_CPU_CMPLX_CLR \
	(IO_ADDRESS(TEGRA_CLK_RESET_BASE) + 0x34c)
#define CAR_BOND_OUT_V \
	(IO_ADDRESS(TEGRA_CLK_RESET_BASE) + 0x390)
#define CAR_BOND_OUT_V_CPU_G	(1<<0)
#endif

static void __iomem *scu_base = IO_ADDRESS(TEGRA_ARM_PERIF_BASE);

static unsigned int available_cpus(void)
{
	static unsigned int ncores;

	if (ncores == 0) {
		ncores = scu_get_core_count(scu_base);
#ifndef CONFIG_ARCH_TEGRA_2x_SOC
		if (ncores > 1) {
			u32 fuse_sku = readl(FUSE_SKU_DIRECT_CONFIG);
			ncores -= FUSE_SKU_NUM_DISABLED_CPUS(fuse_sku);
			BUG_ON((int)ncores <= 0);
		}
#endif
	}
	return ncores;
}

static int is_g_cluster_available(unsigned int cpu)
{
#ifdef CONFIG_TEGRA_CLUSTER_CONTROL
	u32 fuse_sku = readl(FUSE_SKU_DIRECT_CONFIG);
	u32 bond_out = readl(CAR_BOND_OUT_V);

	/* Does the G CPU complex exist at all? */
	if ((fuse_sku & FUSE_SKU_DISABLE_ALL_CPUS) ||
	    (bond_out & CAR_BOND_OUT_V_CPU_G))
		return -EPERM;

	if (cpu >= available_cpus())
		return -EPERM;

	/* FIXME: The G CPU can be unavailable for a number of reasons
	 *	  (e.g., low battery, over temperature, etc.). Add checks for
	 *	  these conditions. */
	return 0;
#else
	return -EPERM;
#endif
}

#ifndef CONFIG_ARCH_TEGRA_2x_SOC
static bool is_cpu_powered(unsigned int cpu)
{
	if (is_lp_cluster())
		return true;
	else
		return tegra_powergate_is_powered(TEGRA_CPU_POWERGATE_ID(cpu));
}
#endif

static int power_up_cpu(unsigned int cpu)
{
	u32 reg;
	int ret = 0;
#ifndef CONFIG_ARCH_TEGRA_2x_SOC
	unsigned long timeout;

	BUG_ON(cpu == smp_processor_id());
	BUG_ON(is_lp_cluster());

	/* If this cpu has booted this function is entered after
	 * CPU has been already un-gated by flow controller. Wait
	 * for confirmation that cpu is powered and remove clamps.
	 * On first boot entry do not wait - go to direct ungate.
	 */
	if (cpu_isset(cpu, tegra_cpu_init_map)) {
		timeout = jiffies + 5;
		do {
			if (is_cpu_powered(cpu))
				goto remove_clamps;
			udelay(10);
		} while (time_before(jiffies, timeout));
	}

	/* First boot or Flow controller did not work as expected. Try to
	   directly toggle power gates. Error if direct power on also fails. */
	if (!is_cpu_powered(cpu)) {
		ret = tegra_unpowergate_partition(TEGRA_CPU_POWERGATE_ID(cpu));
		if (ret)
			goto fail;

		/* Wait for the power to come up. */
		timeout = jiffies + 10*HZ;

		do {
			if (is_cpu_powered(cpu))
				goto remove_clamps;
			udelay(10);
		} while (time_before(jiffies, timeout));
		ret = -ETIMEDOUT;
		goto fail;
	}

remove_clamps:
	/* CPU partition is powered. Enable the CPU clock. */
	writel(CPU_CLOCK(cpu), CLK_RST_CONTROLLER_CLK_CPU_CMPLX_CLR);
	reg = readl(CLK_RST_CONTROLLER_CLK_CPU_CMPLX_CLR);
	udelay(10);

	/* Remove I/O clamps. */
	ret = tegra_powergate_remove_clamping(TEGRA_CPU_POWERGATE_ID(cpu));
	udelay(10);
fail:
#else
	/* Enable the CPU clock. */
	reg = readl(CLK_RST_CONTROLLER_CLK_CPU_CMPLX);
	writel(reg & ~CPU_CLOCK(cpu), CLK_RST_CONTROLLER_CLK_CPU_CMPLX);
	barrier();
	reg = readl(CLK_RST_CONTROLLER_CLK_CPU_CMPLX);
#endif
	/* Clear flow controller CSR. */
	flowctrl_writel(0, FLOW_CTRL_CPU_CSR(cpu));
	return ret;
}

void __cpuinit platform_secondary_init(unsigned int cpu)
{
	gic_secondary_init(0);

	cpumask_set_cpu(cpu, to_cpumask(tegra_cpu_init_bits));
	if (!tegra_all_cpus_booted)
		if (cpumask_equal(tegra_cpu_init_mask, cpu_present_mask))
			tegra_all_cpus_booted = true;
}

int boot_secondary(unsigned int cpu, struct task_struct *idle)
{
	int status;

	/* Avoid timer calibration on slave cpus. Use the value calibrated
	 * on master cpu. This reduces the bringup time for each slave cpu
	 * by around 260ms.
	 */
	preset_lpj = loops_per_jiffy;
	if (is_lp_cluster()) {
		struct clk *cpu_clk, *cpu_g_clk;

		/* The G CPU may not be available for a variety of reasons. */
		status = is_g_cluster_available(cpu);
		if (status)
			goto done;

		cpu_clk = tegra_get_clock_by_name("cpu");
		cpu_g_clk = tegra_get_clock_by_name("cpu_g");

		/* Switch to G CPU before continuing. */
		if (!cpu_clk || !cpu_g_clk) {
			/* Early boot, clock infrastructure is not initialized
			   - CPU mode switch is not allowed */
			status = -EINVAL;
		} else
			status = clk_set_parent(cpu_clk, cpu_g_clk);

		if (status)
			goto done;
	}

	smp_wmb();

	/* Force the CPU into reset. The CPU must remain in reset when the
	   flow controller state is cleared (which will cause the flow
	   controller to stop driving reset if the CPU has been power-gated
	   via the flow controller). This will have no effect on first boot
	   of the CPU since it should already be in reset. */
	writel(CPU_RESET(cpu), CLK_RST_CONTROLLER_RST_CPU_CMPLX_SET);
	dmb();

	/* Unhalt the CPU. If the flow controller was used to power-gate the
	   CPU this will cause the flow controller to stop driving reset.
	   The CPU will remain in reset because the clock and reset block
	   is now driving reset. */
	flowctrl_writel(0, FLOW_CTRL_HALT_CPU(cpu));

	status = power_up_cpu(cpu);
	if (status)
		goto done;

	/* Take the CPU out of reset. */
	writel(CPU_RESET(cpu), CLK_RST_CONTROLLER_RST_CPU_CMPLX_CLR);
	wmb();
done:
	return status;
}

/*
 * Initialise the CPU possible map early - this describes the CPUs
 * which may be present or become present in the system.
 */
void __init smp_init_cpus(void)
{
	unsigned int ncores = available_cpus();
	unsigned int i;

	if (ncores > NR_CPUS) {
		printk(KERN_ERR "Tegra: no. of cores (%u) greater than configured (%u), clipping\n",
			ncores, NR_CPUS);
		ncores = NR_CPUS;
	}

	for (i = 0; i < ncores; i++)
		cpu_set(i, cpu_possible_map);

	/* If only one CPU is possible, platform_smp_prepare_cpus() will
	   never get called. We must therefore initialize the reset handler
	   here. If there is more than one CPU, we must wait until after
	   the cpu_present_mask has been updated with all present CPUs in
	   platform_smp_prepare_cpus() before initializing the reset handler. */
	if (ncores == 1) {
		tegra_cpu_reset_handler_init();
		tegra_all_cpus_booted = true;
	}
}

void __init platform_smp_prepare_cpus(unsigned int max_cpus)
{
	int i;

	/*
	 * Initialise the present map, which describes the set of CPUs
	 * actually populated at the present time.
	 */
	for (i = 0; i < max_cpus; i++)
		set_cpu_present(i, true);

	/* Always mark the boot CPU as initialized. */
	cpumask_set_cpu(0, to_cpumask(tegra_cpu_init_bits));

	if (max_cpus == 1)
		tegra_all_cpus_booted = true;

	/* If we're here, it means that more than one CPU was found by
	   smp_init_cpus() which also means that it did not initialize the
	   reset handler. Do it now before the secondary CPUs are started. */
	tegra_cpu_reset_handler_init();
	scu_enable(scu_base);
}
