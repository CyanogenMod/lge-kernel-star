/*
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
 */

#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/io.h>

#include <mach/iomap.h>
#include <mach/irqs.h>
#include <mach/gpio.h>

#include "gpio-names.h"

static int tegra_wake_event_irq[] = {
	TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PO5),	/* wake0 */
	TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PV1),	/* wake1 */
	TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PL1),	/* wake2 */
	TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PB6),	/* wake3 */
	TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PN7),	/* wake4 */
	TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PBB6),	/* wake5 */
	TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PU5),	/* wake6 */
	TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PU6),	/* wake7 */
	TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PC7),	/* wake8 */
	TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PS2),	/* wake9 */
	TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PAA1),	/* wake10 */
	TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PW3),	/* wake11 */
	TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PW2),	/* wake12 */
	TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PY6),	/* wake13 */
	TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PDD3),	/* wake14 */
	TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PJ2),	/* wake15 */
	INT_RTC,				/* wake16 */
	INT_KBC,				/* wake17 */
	INT_EXTERNAL_PMU,			/* wake18 */
	-EINVAL, /* TEGRA_USB1_VBUS, */		/* wake19 */
	-EINVAL, /* TEGRA_USB2_VBUS, */		/* wake20 */
	-EINVAL, /* TEGRA_USB1_ID, */		/* wake21 */
	-EINVAL, /* TEGRA_USB2_ID, */		/* wake22 */
	TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PI5),	/* wake23 */
	TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PV0),	/* wake24 */
	TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PS4),	/* wake25 */
	TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PS5),	/* wake26 */
	TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PS0),	/* wake27 */
	TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PS6),	/* wake28 */
	TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PS7),	/* wake29 */
	TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PN2),	/* wake30 */
	-EINVAL, /* not used */			/* wake31 */
	TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PO4),	/* wake32 */
	TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PJ0),	/* wake33 */
	TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PK2),	/* wake34 */
	TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PI6),	/* wake35 */
	TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PBB1),	/* wake36 */
	-EINVAL, /* TEGRA_USB3_VBUS, */		/* wake37 */
	-EINVAL, /* TEGRA_USB3_ID, */		/* wake38 */
	INT_USB, /* TEGRA_USB1_UTMIP, */	/* wake39 */
	INT_USB2, /* TEGRA_USB2_UTMIP, */	/* wake40 */
	INT_USB3, /* TEGRA_USB3_UTMIP, */	/* wake41 */
};

int tegra_irq_to_wake(int irq)
{
	int i;
	int wake_irq;
	int search_gpio;
	static int last_wake = -1;

	/* Two level wake irq search for gpio based wakeups -
	 * 1. check for GPIO irq(based on tegra_wake_event_irq table)
	 * e.g. for a board, wake7 based on GPIO PU6 and irq==390 done first
	 * 2. check for gpio bank irq assuming search for GPIO irq
	 *    preceded this search.
	 * e.g. in this step check for gpio bank irq GPIO6 irq==119
	 */
	for (i = 0; i < ARRAY_SIZE(tegra_wake_event_irq); i++) {
		/* return if step 1 matches */
		if (tegra_wake_event_irq[i] == irq) {
			pr_info("Wake%d for irq=%d\n", i, irq);
			last_wake = i;
			return i;
		}

		/* step 2 below uses saved last_wake from step 1
		 * in previous call */
		search_gpio = irq_to_gpio(
			tegra_wake_event_irq[i]);
		if (search_gpio < 0)
			continue;
		wake_irq = tegra_gpio_get_bank_int_nr(search_gpio);
		if (wake_irq < 0)
			continue;
		if ((last_wake == i) &&
			(wake_irq == irq)) {
			pr_info("gpio bank wake found: wake%d for irq=%d\n",
				i, irq);
			return i;
		}
	}

	return -EINVAL;
}

int tegra_wake_to_irq(int wake)
{
	if (wake < 0)
		return -EINVAL;

	if (wake >= ARRAY_SIZE(tegra_wake_event_irq))
		return -EINVAL;

	return tegra_wake_event_irq[wake];
}
