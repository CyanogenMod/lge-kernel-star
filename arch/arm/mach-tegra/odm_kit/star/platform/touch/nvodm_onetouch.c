/*
 * Copyright (c) 2006-2009 NVIDIA Corporation.
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

#include "nvodm_onetouch.h"
#include "nvodm_onetouch_int.h"

#include "nvodm_onetouch_synaptics.h"

#include <star_hw_definition.h>
#include <star_pinmux_definition.h>

#include "../../adaptations/pmu/max8907/max8907_supply_info_table.h"


/** Implementation for the NvOdm TouchPad */

NvBool NvOdmOneTouchDeviceOpen( NvOdmOneTouchDeviceHandle *hDevice, NvOdmOsSemaphoreHandle* hIntSema )
{
    NvBool ret = NV_TRUE;

    ret = Synaptics_OneTouch_Open(hDevice, hIntSema);
    return ret;
}

NvBool NvOdmOneTouchReadButton( NvOdmOneTouchDeviceHandle hDevice, NvOdmOneTouchButtonInfo *button)
{
    return hDevice->ReadButton(hDevice, button);
}

void NvOdmOneTouchDeviceClose(NvOdmOneTouchDeviceHandle hDevice)
{
    hDevice->Close(hDevice);    
}

NvBool NvOdmOneTouchEnableInterrupt(NvOdmOneTouchDeviceHandle hDevice, NvOdmOsSemaphoreHandle hInterruptSemaphore)
{
    return hDevice->EnableInterrupt(hDevice, hInterruptSemaphore);
}

NvBool NvOdmOneTouchHandleInterrupt(NvOdmOneTouchDeviceHandle hDevice)
{
    return hDevice->HandleInterrupt(hDevice);
}

NvBool
NvOdmOneTouchSleepMode(NvOdmOneTouchDeviceHandle hDevice, NvBool OnOff)
{
    hDevice->SleepMode(hDevice, OnOff);
}

void
NvOdmOneTouchPowerOnOff(NvOdmOneTouchDeviceHandle hDevice, NvBool OnOff)
{
    hDevice->PowerOnOff(hDevice, OnOff);
}

NvBool
NvOdmOneTouchOutputDebugMessage(NvOdmOneTouchDeviceHandle hDevice)
{
    return hDevice->OutputDebugMessage;
}

void
NvOdmOneTouchInterruptMask(NvOdmOneTouchDeviceHandle hDevice, NvBool mask)
{
	hDevice->InterruptMask(hDevice, mask);
}

