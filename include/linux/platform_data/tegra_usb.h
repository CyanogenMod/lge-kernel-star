/*
 * Copyright (C) 2010 Google, Inc.
 * Copyright (C) 2010-2011 NVIDIA Corporation
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

#ifndef _TEGRA_USB_H_
#define _TEGRA_USB_H_

enum tegra_usb_operating_modes {
	TEGRA_USB_DEVICE,
	TEGRA_USB_HOST,
	TEGRA_USB_OTG,
};

enum tegra_usb_phy_type {
	TEGRA_USB_PHY_TYPE_UTMIP = 0,
	TEGRA_USB_PHY_TYPE_LINK_ULPI = 1,
	TEGRA_USB_PHY_TYPE_NULL_ULPI = 2,
	TEGRA_USB_PHY_TYPE_HSIC = 3,
	TEGRA_USB_PHY_TYPE_ICUSB = 4,
};

struct tegra_ehci_platform_data {
	enum tegra_usb_operating_modes operating_mode;
	/* power down the phy on bus suspend */
	int power_down_on_bus_suspend;
	int hotplug;
	void *phy_config;
	enum tegra_usb_phy_type phy_type;
};

struct tegra_otg_platform_data {
	struct platform_device *ehci_device;
	struct tegra_ehci_platform_data *ehci_pdata;
};

#endif /* _TEGRA_USB_H_ */
