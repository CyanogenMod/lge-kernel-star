/*
 * drivers/usb/gadget/tegra_udc.c
 *
 * USB device controller clock and PHY programming for Tegra SoCs
 *
 * Copyright (c) 2009, NVIDIA Corporation.
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
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#define NV_DEBUG 0

#include <linux/kernel.h>
#include <linux/platform_device.h>

#include "nvddk_usbphy.h"
#include "mach/nvrm_linux.h"
#include "nvassert.h"

NvDdkUsbPhyHandle s_hUsbPhy;

int tegra_udc_clk_init(struct platform_device *pdev)
{
	NvError nverr;

	nverr = NvDdkUsbPhyOpen(s_hRmGlobal, pdev->id, &s_hUsbPhy);
	if (nverr != NvSuccess)
		return -ENODEV;

	/* Power up the USB phy */
	NV_ASSERT_SUCCESS(NvDdkUsbPhyPowerUp(s_hUsbPhy, NV_FALSE, 0));

	return 0;
}

void tegra_udc_clk_finalize(struct platform_device *pdev)
{
}

void tegra_udc_clk_release(void)
{
}

void tegra_udc_clk_suspend(void)
{
	NV_ASSERT_SUCCESS(NvDdkUsbPhyPowerDown(s_hUsbPhy, NV_FALSE, 0));
}

void tegra_udc_clk_resume(void)
{
	NV_ASSERT_SUCCESS(NvDdkUsbPhyPowerUp(s_hUsbPhy, NV_FALSE, 0));
}

bool tegra_udc_charger_detection(void)
{
	NvDdkUsbPhyIoctl_DedicatedChargerDetectionInputArgs Charger;
	NvDdkUsbPhyIoctl_DedicatedChargerStatusOutputArgs Status;

	/* clear the input args */
	NvOsMemset(&Charger, 0, sizeof(Charger));
	/* enable the charger detection logic */
	Charger.EnableChargerDetection = NV_TRUE;
	NV_ASSERT_SUCCESS(NvDdkUsbPhyIoctl(
		s_hUsbPhy,
		NvDdkUsbPhyIoctlType_DedicatedChargerDetection,
		&Charger,
		NULL));
	/* get the charger detection status */
	NV_ASSERT_SUCCESS(NvDdkUsbPhyIoctl(
		s_hUsbPhy,
		NvDdkUsbPhyIoctlType_DedicatedChargerStatus,
		NULL,
		&Status));
	/* disable the charger detection */
	Charger.EnableChargerDetection = NV_FALSE;
	NV_ASSERT_SUCCESS(NvDdkUsbPhyIoctl(
		s_hUsbPhy,
		NvDdkUsbPhyIoctlType_DedicatedChargerDetection,
		&Charger,
		NULL));

	return Status.ChargerDetected;
}

void tegra_udc_dtd_prepare(void)
{
	/* When we are programming two DTDs very close to each other,
	 * the second DTD is being prefetched before it is actually written
	 * to DDR. To prevent this, we disable prefetcher before programming
	 * any new DTD and re-enable it before priming endpoint.
	 */
	NvDdkUsbPhyMemoryPrefetch(s_hUsbPhy, NV_FALSE);
}

void tegra_udc_ep_barrier(void)
{
	NvDdkUsbPhyMemoryPrefetch(s_hUsbPhy, NV_TRUE);
}
