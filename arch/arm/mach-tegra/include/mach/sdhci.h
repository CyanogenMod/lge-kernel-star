/*
 * arch/arm/mach-tegra/include/mach/sdhci.h
 *
 * SDHCI driver platform data definitions
 *
 * Copyright (C) 2009 NVIDIA Corporation
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
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

#ifndef __MACH_TEGRA_SDHCI_H
#define __MACH_TEGRA_SDHCI_H

#include <mach/pinmux.h>
#include "nvcommon.h"
#include "nvodm_query.h"

struct tegra_sdhci_platform_data {
	const struct tegra_pingroup_config *pinmux;
	int nr_pins;
	int gpio_nr_cd;		/* card detect gpio, -1 if unused */
	int gpio_polarity_cd;	/* active high card detect */
	int gpio_nr_wp;		/* write protect gpio, -1 if unused */
	int gpio_polarity_wp;	/* active high write protect */
	int bus_width;		/* bus width in bits */
	int is_removable;	/* card can be removed */
	unsigned int debounce;	/* debounce time in milliseconds */
	unsigned long max_clk;	/* maximum card clock */
	int is_always_on;	/* card is not powered down in suspend */
#ifdef CONFIG_EMBEDDED_MMC_START_OFFSET
	unsigned long offset;	/* offset in blocks to MBR */
#endif
	/* card detect callback registration function*/
	int (*register_status_notify)(void (*callback)(int card_present,
		void *dev_id), void *dev_id);
};


#endif
