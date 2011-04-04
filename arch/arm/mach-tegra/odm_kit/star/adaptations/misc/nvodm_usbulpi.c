/*
 * arch/arm/mach-tegra/odm_kit/query/whistler/subboards/nvodm_query_discovery_e1129_peripherals.h
 *
 * Specifies the peripheral connectivity database peripheral entries for the E1129 keypad module
 *
 * Copyright (c) 2009 NVIDIA Corporation.
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
#include "nvodm_usbulpi.h"
#include "nvodm_services.h"
#include "nvodm_query_discovery.h"
#include "nvodm_services.h"
#include "nvos.h"

typedef struct NvOdmUsbUlpiRec
{
    NvU64    CurrentGUID;
} NvOdmUsbUlpi;

NvOdmUsbUlpiHandle NvOdmUsbUlpiOpen(NvU32 Instance)
{
    // return dummay handle
    NvOdmUsbUlpi*pDevice = NULL;
    pDevice = NvOdmOsAlloc(sizeof(NvOdmUsbUlpi));
    if(pDevice == NULL)
        goto ExitUlpiOdm;
    return pDevice;
    
ExitUlpiOdm:
	if (pDevice)
		NvOdmOsFree(pDevice);
    return NULL;
}

void NvOdmUsbUlpiClose(NvOdmUsbUlpiHandle hOdmUlpi)
{
    if (hOdmUlpi)
    {
        NvOdmOsFree(hOdmUlpi);
        hOdmUlpi = NULL;
    }
}


