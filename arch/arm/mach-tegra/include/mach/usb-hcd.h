/*
 * arch/arm/mach-tegra/include/mach/usb-hcd.h
 *
 * HCD platform data definitions
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

#ifndef __MACH_TEGRA_USB_HCD_H
#define __MACH_TEGRA_USB_HCD_H

#include "nvcommon.h"
#include "nvrm_gpio.h"
#include "nvodm_query.h"
#include "nvddk_usbphy.h"

enum {
	ID_PIN_CABLE_ID = 1,
	ID_PIN_GPIO,
};

struct tegra_hcd_platform_data {
	NvU32			instance;
	unsigned int		id_detect;
	int			gpio_nr;
	bool			otg_mode;
	NvU32			powerClientId;
	NvU32			vBusPowerRail;
	/* USB PHY power rail. Tegra has integrated UTMI (USB transciver
	 * macrocell interface) PHY on USB controllers 0 and 2. These 2 PHYs
	 * have its own rails.
	 */
	NvU32			phyPowerRail;
	NvDdkUsbPhyHandle	hUsbPhy;
        struct work_struct      work;
};

#endif
