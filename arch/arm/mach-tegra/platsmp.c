/*
 *  linux/arch/arm/mach-tegra/platsmp.c
 *
 *  Copyright (C) 2002 ARM Ltd.
 *  All Rights Reserved
 *
 *  Copyright (C) 2009 Palm
 *  All Rights Reserved
 *
 *  Copyright (C) 2010 NVIDIA Corporation
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

#include <asm/hardware/gic.h>
#include <asm/smp_scu.h>

#include <mach/iomap.h>
#include <mach/powergate.h>

#include "pm.h"

#define EVP_CPU_RESET_VECTOR \
	(IO_ADDRESS(TEGRA_EXCEPTION_VECTORS_BASE) + 0x100)
#define CLK_RST_CONTROLLER_CLK_CPU_CMPLX \
	(IO_ADDRESS(TEGRA_CLK_RESET_BASE) + 0x4c)
#define CLK_RST_CONTROLLER_RST_CPU_CMPLX_SET \
	(IO_ADDRESS(TEGRA_CLK_RESET_BASE) + 0x340)
#define CLK_RST_CONTROLLER_RST_CPU_CMPLX_CLR \
	(IO_ADDRESS(TEGRA_CLK_RESET_BASE) + 0x344)
#define FLOW_CTRL_HALT_CPUx_EVENTS(cpu)	\
	(IO_ADDRESS(TEGRA_FLOW_CTRL_BASE + ((cpu)?(((cpu)-1)*8 + 0x14) : 0x0)))

#define CPU_CLOCK(cpu)	(0x1<<(8+cpu))
#define CPU_RESET(cpu)	(0x1111ul<<(cpu))

static unsigned int available_cpus(void);
#if defined(CONFIG_ARCH_TEGRA_2x_SOC)
static inline int is_g_cluster_available(unsigned int cpu)
{ return -EPERM; }
static inline bool is_cpu_powered(unsigned int cpu)
{ return true; }
static inline int power_up_cpu(unsigned int cpu)
{ return 0; }

/* For Tegra2 use the software-written value of the reset regsiter for status.*/
#define CLK_RST_CONTROLLER_CPU_CMPLX_STATUS CLK_RST_CONTROLLER_RST_CPU_CMPLX_SET

#else
static int is_g_cluster_available(unsigned int cpu);
static bool is_cpu_powered(unsigned int cpu);
static int power_up_cpu(unsigned int cpu);

#define CAR_BOND_OUT_V \
	(IO_ADDRESS(TEGRA_CLK_RESET_BASE) + 0x390)
#define CAR_BOND_OUT_V_CPU_G	(1<<0)
#define CLK_RST_CONTROLLER_CPU_CMPLX_STATUS \
	(IO_ADDRESS(TEGRA_CLK_RESET_BASE) + 0x470)

#define FUSE_SKU_DIRECT_CONFIG \
	(IO_ADDRESS(TEGRA_FUSE_BASE) + 0x1F4)
#define FUSE_SKU_DISABLE_ALL_CPUS	(1<<5)
#define FUSE_SKU_NUM_DISABLED_CPUS(x)	(((x) >> 3) & 3)
#endif

extern void tegra_secondary_startup(void);

static void __iomem *scu_base = IO_ADDRESS(TEGRA_ARM_PERIF_BASE);

void __cpuinit platform_secondary_init(unsigned int cpu)
{
	gic_secondary_init(0);
}

int __cpuinit boot_secondary(unsigned int cpu, struct task_struct *idle)
{
	u32 reg;
	int status;

	if (is_lp_cluster()) {
		/* The G CPU may not be available for a
		   variety of reasons. */
		status = is_g_cluster_available(cpu);
		if (status)
			return status;

		/* Switch to the G CPU before continuing. */
		status = tegra_cluster_control(0,
					       TEGRA_POWER_CLUSTER_G |
					       TEGRA_POWER_CLUSTER_IMMEDIATE);
		if (status)
			return status;
	}

	smp_wmb();

	/* set the reset vector to point to the secondary_startup routine */
	writel(virt_to_phys(tegra_secondary_startup), EVP_CPU_RESET_VECTOR);

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
	writel(0, FLOW_CTRL_HALT_CPUx_EVENTS(cpu));
	dmb();

	/* enable cpu clock on cpu */
	reg = readl(CLK_RST_CONTROLLER_CLK_CPU_CMPLX);
	writel(reg & ~CPU_CLOCK(cpu), CLK_RST_CONTROLLER_CLK_CPU_CMPLX);
	dmb();

	status = power_up_cpu(cpu);
	if (status)
		goto done;

	dmb();
	writel(CPU_RESET(cpu), CLK_RST_CONTROLLER_RST_CPU_CMPLX_CLR);

done:
	return status;
}

/*
 * Initialise the CPU possible map early - this describes the CPUs
 * which may be present or become present in the system.
 */
void __init smp_init_cpus(void)
{
	unsigned int i, ncores = available_cpus();

	if (ncores > NR_CPUS) {
		printk(KERN_ERR "Tegra: no. of cores (%u) greater than configured (%u), clipping\n",
			ncores, NR_CPUS);
		ncores = NR_CPUS;
	}

	for (i = 0; i < ncores; i++)
		cpu_set(i, cpu_possible_map);
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

	scu_enable(scu_base);
}

#if defined(CONFIG_ARCH_TEGRA_3x_SOC)

static bool is_cpu_powered(unsigned int cpu)
{
	if (is_lp_cluster())
		return true;
	else
		return tegra_powergate_is_powered(TEGRA_CPU_POWERGATE_ID(cpu));
}

static int power_up_cpu(unsigned int cpu)
{
	int ret;
	unsigned long timeout;

	BUG_ON(cpu == smp_processor_id());
	BUG_ON(is_lp_cluster());

	if (!is_cpu_powered(cpu))
	{
		ret = tegra_powergate_power_on(TEGRA_CPU_POWERGATE_ID(cpu));
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
	ret = tegra_powergate_remove_clamping(TEGRA_CPU_POWERGATE_ID(cpu));
fail:
	return ret;
}

static int is_g_cluster_available(unsigned int cpu)
{
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
}
#endif

static unsigned int available_cpus(void)
{
	static unsigned int ncores = 0;

	if (ncores == 0) {
		ncores = scu_get_core_count(scu_base);
#ifdef CONFIG_ARCH_TEGRA_3x_SOC
		if (ncores > 1) {
			u32 fuse_sku = readl(FUSE_SKU_DIRECT_CONFIG);
			ncores -= FUSE_SKU_NUM_DISABLED_CPUS(fuse_sku);
			BUG_ON((int)ncores <= 0);
		}
#endif
	}
	return ncores;
}
