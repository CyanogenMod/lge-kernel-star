/*
 * arch/arm/mach-tegra/include/mach/gpio.h
 *
 * Copyright (C) 2010 Google, Inc.
 *
 * Author:
 *	Erik Gilling <konkers@google.com>
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

#ifndef __MACH_TEGRA_GPIO_H
#define __MACH_TEGRA_GPIO_H

#include <linux/init.h>
#include <mach/irqs.h>

#define TEGRA_NR_GPIOS		INT_GPIO_NR
#define ARCH_NR_GPIOS		(TEGRA_NR_GPIOS + 128)

#include <asm-generic/gpio.h>
#include "pinmux.h"

struct gpio_init_pin_info {
	char name[16];
	int gpio_nr;
	bool is_gpio;
	bool is_input;
	int value; /* Value if it is output*/
};

#define gpio_get_value		__gpio_get_value
#define gpio_set_value		__gpio_set_value
#define gpio_cansleep		__gpio_cansleep

#define TEGRA_GPIO_TO_IRQ(gpio) (INT_GPIO_BASE + (gpio))
#define TEGRA_IRQ_TO_GPIO(irq) ((irq) - INT_GPIO_BASE)

static inline int gpio_to_irq(unsigned int gpio)
{
	/* SOC gpio */
	if (gpio < TEGRA_NR_GPIOS)
		return INT_GPIO_BASE + gpio;

	/* For non soc gpio, the external peripheral driver need to
	 * provide the implementation */
	return __gpio_to_irq(gpio);
}

static inline int irq_to_gpio(unsigned int irq)
{
	/* SOC gpio */
	if ((irq >= INT_GPIO_BASE) && (irq < INT_GPIO_BASE + INT_GPIO_NR))
		return irq - INT_GPIO_BASE;

	/* we don't supply reverse mappings for non-SOC gpios */
	return -EIO;
}

struct tegra_gpio_table {
	int	gpio;	/* GPIO number */
	bool	enable;	/* Enable for GPIO at init? */
};

void tegra_gpio_config(struct tegra_gpio_table *table, int num);
void tegra_gpio_enable(int gpio);
void tegra_gpio_disable(int gpio);
int tegra_gpio_resume_init(void);
void tegra_gpio_init_configure(unsigned gpio, bool is_input, int value);
void tegra_gpio_set_tristate(int gpio, enum tegra_tristate ts);
int tegra_gpio_get_bank_int_nr(int gpio);
#endif
