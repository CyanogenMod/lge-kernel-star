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

#ifndef _MACH_TERA_PM_IRQ_H_
#define _MACH_TERA_PM_IRQ_H_

#ifdef CONFIG_PM_SLEEP
int tegra_pm_irq_set_wake(int irq, int enable);
int tegra_pm_irq_set_wake_type(int irq, int flow_type);
bool tegra_pm_irq_lp0_allowed(void);
int tegra_irq_to_wake(int irq);
int tegra_wake_to_irq(int wake);
#else
static inline int tegra_pm_irq_set_wake_type(int irq, int flow_type)
{
	return 0;
}
#endif
#endif
