/*
 * Copyright (C) 2010 Google, Inc.
 *
 * Author:
 *	Colin Cross <ccross@google.com>
 *
 * Copyright (C) 2010, NVIDIA Corporation
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/io.h>

#include <asm/hardware/gic.h>

#include <mach/iomap.h>

#include "board.h"

#define INT_SYS_NR	(INT_GPIO_BASE - INT_PRI_BASE)
#define INT_SYS_SZ	(INT_SEC_BASE - INT_PRI_BASE)
#define PPI_NR		((INT_SYS_NR+INT_SYS_SZ-1)/INT_SYS_SZ)

#define APBDMA_IRQ_STA_CPU  0x14
#define APBDMA_IRQ_MASK_SET 0x20
#define APBDMA_IRQ_MASK_CLR 0x24

#define ICTLR_CPU_IER		0x20
#define ICTLR_CPU_IER_SET	0x24
#define ICTLR_CPU_IER_CLR	0x28
#define ICTLR_CPU_IEP_CLASS	0x2c

static void (*gic_mask_irq)(unsigned int irq) = NULL;
static void (*gic_unmask_irq)(unsigned int irq) = NULL;

#define irq_to_ictlr(irq) (((irq)-32) >> 5)
static void __iomem *tegra_ictlr_base = IO_ADDRESS(TEGRA_PRIMARY_ICTLR_BASE);
static void __iomem *tegra_apbdma_base = IO_ADDRESS(TEGRA_APBDMA_BASE);
#define ictlr_to_virt(ictlr) (tegra_ictlr_base + (ictlr)*0x100)

static void tegra_mask(unsigned int irq)
{
	void __iomem *addr = ictlr_to_virt(irq_to_ictlr(irq));
	gic_mask_irq(irq);
	writel(1<<(irq&31), addr+ICTLR_CPU_IER_CLR);
}

static void tegra_unmask(unsigned int irq)
{
	void __iomem *addr = ictlr_to_virt(irq_to_ictlr(irq));
	gic_unmask_irq(irq);
	writel(1<<(irq&31), addr+ICTLR_CPU_IER_SET);
}

#ifdef CONFIG_PM

static int tegra_set_wake(unsigned int irq, unsigned int on)
{
	return 0;
}
#endif

static struct irq_chip tegra_irq = {
	.name		= "PPI",
	.mask		= tegra_mask,
	.unmask		= tegra_unmask,
#ifdef CONFIG_PM
	.set_wake	= tegra_set_wake,
#endif
};

static DEFINE_SPINLOCK(apbdma_lock);

static void apbdma_ack(unsigned int irq) { }

static void apbdma_mask(unsigned int irq)
{
	irq -= INT_APBDMA_BASE;
	writel(1<<irq, tegra_apbdma_base + APBDMA_IRQ_MASK_CLR);
}

static void apbdma_unmask(unsigned int irq)
{
	irq -= INT_APBDMA_BASE;
	writel(1<<irq, tegra_apbdma_base + APBDMA_IRQ_MASK_SET);
}

static void apbdma_cascade(unsigned int irq, struct irq_desc *desc)
{
	struct irq_chip *pri = get_irq_chip(irq);
	u32 reg, ch=0;

	pri->ack(irq);
	spin_lock(&apbdma_lock);
	reg = readl(tegra_apbdma_base + APBDMA_IRQ_STA_CPU);
	if (reg) {
		reg = __fls(reg);
		writel(1<<reg, tegra_apbdma_base + APBDMA_IRQ_STA_CPU);
		ch = INT_APBDMA_BASE + reg;
	}
	spin_unlock(&apbdma_lock);
	if (ch)	generic_handle_irq(ch);
	pri->unmask(irq);
}

static struct irq_chip apbdma_irq = {
	.name	= "APBDMA",
	.ack	= apbdma_ack,
	.mask	= apbdma_mask,
	.unmask	= apbdma_unmask,
};

void __init tegra_init_irq(void)
{
	struct irq_chip *gic;
	unsigned int i;

	for (i=0; i<PPI_NR; i++) {
		writel(~0, ictlr_to_virt(i) + ICTLR_CPU_IER_CLR);
		writel(0, ictlr_to_virt(i) + ICTLR_CPU_IEP_CLASS);
	}

	gic_dist_init(0, IO_ADDRESS(TEGRA_ARM_INT_DIST_BASE), 29);
	gic_cpu_init(0, IO_ADDRESS(TEGRA_ARM_PERIF_BASE + 0x100));

	gic = get_irq_chip(29);
	gic_unmask_irq = gic->unmask;
	gic_mask_irq = gic->mask;
	tegra_irq.ack = gic->ack;
#ifdef CONFIG_SMP
	tegra_irq.set_affinity = gic->set_affinity;
#endif

	for (i=INT_PRI_BASE; i<INT_GPIO_BASE; i++) {
		set_irq_chip(i, &tegra_irq);
		set_irq_handler(i, handle_level_irq);
		set_irq_flags(i, IRQF_VALID);
	}

	for (i=INT_APBDMA_BASE; i<INT_APBDMA_NR+INT_APBDMA_BASE; i++) {
		set_irq_chip(i, &apbdma_irq);
		set_irq_handler(i, handle_level_irq);
		set_irq_flags(i, IRQF_VALID);
	}
	set_irq_chained_handler(INT_APB_DMA, apbdma_cascade);
}
