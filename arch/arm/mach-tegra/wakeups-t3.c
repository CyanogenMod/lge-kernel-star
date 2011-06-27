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

#include "gpio-names.h"

#define NUM_WAKE_EVENTS 39

static int tegra_wake_event_irq[NUM_WAKE_EVENTS] = {
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
};

int tegra_irq_to_wake(int irq)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(tegra_wake_event_irq); i++)
		if (tegra_wake_event_irq[i] == irq)
			return i;

	return -EINVAL;
}

int tegra_wake_to_irq(int wake)
{
	if (wake < 0)
		return -EINVAL;

	if (wake >= NUM_WAKE_EVENTS)
		return -EINVAL;

	return tegra_wake_event_irq[wake];
}
