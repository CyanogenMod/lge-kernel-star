/*
 * Copyright (C) 2011 Google, Inc.
 *
 * Author:
 *	Colin Cross <ccross@android.com>
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
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/moduleparam.h>
#include <linux/seq_file.h>
#include <linux/syscore_ops.h>

#include <mach/iomap.h>

#include "pm-irq.h"

#define PMC_CTRL		0x0
#define PMC_CTRL_LATCH_WAKEUPS	(1 << 5)
#define PMC_WAKE_MASK		0xc
#define PMC_WAKE_LEVEL		0x10
#define PMC_WAKE_STATUS		0x14
#define PMC_SW_WAKE_STATUS	0x18

static void __iomem *pmc = IO_ADDRESS(TEGRA_PMC_BASE);

static u32 tegra_lp0_wake_enb;
static u32 tegra_lp0_wake_level;
static u32 tegra_lp0_wake_level_any;
static int tegra_prevent_lp0;

static unsigned int tegra_wake_irq_count[32];

static bool debug_lp0;
module_param(debug_lp0, bool, S_IRUGO | S_IWUSR);

static bool warn_prevent_lp0;
module_param(warn_prevent_lp0, bool, S_IRUGO | S_IWUSR);

bool tegra_pm_irq_lp0_allowed(void)
{
	return (tegra_prevent_lp0 == 0);
}

/* ensures that sufficient time is passed for a register write to
 * serialize into the 32KHz domain */
static void pmc_32kwritel(u32 val, unsigned long offs)
{
	writel(val, pmc + offs);
	udelay(130);
}

int tegra_pm_irq_set_wake(int irq, int enable)
{
	int wake = tegra_irq_to_wake(irq);

	if (wake == -EALREADY) {
		/* EALREADY means wakeup event already accounted for */
		return 0;
	} else if (wake == -ENOTSUPP) {
		/* ENOTSUPP means LP0 not supported with this wake source */
		WARN(enable && warn_prevent_lp0, "irq %d prevents lp0\n", irq);
		if (enable)
			tegra_prevent_lp0++;
		else if (!WARN_ON(tegra_prevent_lp0 == 0))
			tegra_prevent_lp0--;
		return 0;
	} else if (wake < 0) {
		return -EINVAL;
	}

	if (enable)
		tegra_lp0_wake_enb |= 1 << wake;
	else
		tegra_lp0_wake_enb &= ~(1 << wake);

	return 0;
}

int tegra_pm_irq_set_wake_type(int irq, int flow_type)
{
	int wake = tegra_irq_to_wake(irq);

	if (wake < 0)
		return 0;

	switch (flow_type) {
	case IRQF_TRIGGER_FALLING:
	case IRQF_TRIGGER_LOW:
		tegra_lp0_wake_level &= ~(1 << wake);
		tegra_lp0_wake_level_any &= ~(1 << wake);
		break;
	case IRQF_TRIGGER_HIGH:
	case IRQF_TRIGGER_RISING:
		tegra_lp0_wake_level |= 1 << wake;
		tegra_lp0_wake_level_any &= ~(1 << wake);
		break;

	case IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING:
		tegra_lp0_wake_level_any |= 1 << wake;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

/* translate lp0 wake sources back into irqs to catch edge triggered wakeups */
static void tegra_pm_irq_syscore_resume(void)
{
	int wake;
	int irq;
	struct irq_desc *desc;
	unsigned long wake_status = readl(pmc + PMC_WAKE_STATUS);

	for_each_set_bit(wake, &wake_status, sizeof(wake_status) * 8) {
		irq = tegra_wake_to_irq(wake);
		if (!irq) {
			pr_info("Resume caused by WAKE%d\n", wake);
			continue;
		}

		desc = irq_to_desc(irq);
		if (!desc || !desc->action || !desc->action->name) {
			pr_info("Resume caused by WAKE%d, irq %d\n", wake, irq);
			continue;
		}

		pr_info("Resume caused by WAKE%d, %s\n", wake,
			desc->action->name);

		tegra_wake_irq_count[wake]++;

		generic_handle_irq(irq);
	}
}

/* set up lp0 wake sources */
static int tegra_pm_irq_syscore_suspend(void)
{
	u32 temp;
	u32 status;
	u32 lvl;
	u32 wake_level;
	u32 wake_enb;

	pmc_32kwritel(0, PMC_SW_WAKE_STATUS);

	temp = readl(pmc + PMC_CTRL);
	temp |= PMC_CTRL_LATCH_WAKEUPS;
	pmc_32kwritel(temp, PMC_CTRL);

	temp &= ~PMC_CTRL_LATCH_WAKEUPS;
	pmc_32kwritel(temp, PMC_CTRL);

	status = readl(pmc + PMC_SW_WAKE_STATUS);

	lvl = readl(pmc + PMC_WAKE_LEVEL);

	/* flip the wakeup trigger for any-edge triggered pads
	 * which are currently asserting as wakeups */
	lvl ^= status;
	lvl &= tegra_lp0_wake_level_any;

	wake_level = lvl | tegra_lp0_wake_level;
	wake_enb = tegra_lp0_wake_enb;

	if (debug_lp0) {
		wake_level = lvl ^ status;
		wake_enb = 0xffffffff;
	}

	writel(wake_level, pmc + PMC_WAKE_LEVEL);

	writel(wake_enb, pmc + PMC_WAKE_MASK);

	return 0;
}

static struct syscore_ops tegra_pm_irq_syscore_ops = {
	.suspend = tegra_pm_irq_syscore_suspend,
	.resume = tegra_pm_irq_syscore_resume,
};

static int tegra_pm_irq_syscore_init(void)
{
	register_syscore_ops(&tegra_pm_irq_syscore_ops);

	return 0;
}
subsys_initcall(tegra_pm_irq_syscore_init);

#ifdef CONFIG_DEBUG_FS
static int tegra_pm_irq_debug_show(struct seq_file *s, void *data)
{
	int wake;
	int irq;
	struct irq_desc *desc;
	const char *irq_name;

	seq_printf(s, "wake  irq  count  name\n");
	seq_printf(s, "----------------------\n");
	for (wake = 0; wake < 32; wake++) {
		irq = tegra_wake_to_irq(wake);
		if (irq < 0)
			continue;

		desc = irq_to_desc(irq);
		if (tegra_wake_irq_count[wake] == 0 && desc->action == NULL)
			continue;

		irq_name = (desc->action && desc->action->name) ?
			desc->action->name : "???";

		seq_printf(s, "%4d  %3d  %5d  %s\n",
			wake, irq, tegra_wake_irq_count[wake], irq_name);
	}
	return 0;
}

static int tegra_pm_irq_debug_open(struct inode *inode, struct file *file)
{
	return single_open(file, tegra_pm_irq_debug_show, NULL);
}

static const struct file_operations tegra_pm_irq_debug_fops = {
	.open		= tegra_pm_irq_debug_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int __init tegra_pm_irq_debug_init(void)
{
	struct dentry *d;

	d = debugfs_create_file("wake_irq", S_IRUGO, NULL, NULL,
		&tegra_pm_irq_debug_fops);
	if (!d) {
		pr_err("Failed to create suspend_mode debug file\n");
		return -ENOMEM;
	}

	return 0;
}

late_initcall(tegra_pm_irq_debug_init);
#endif
