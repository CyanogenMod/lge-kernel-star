/*
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

#include "nvcommon.h"
#include "nvodm_query_discovery.h"
#include "nvodm_gpio_ext.h"
#include "nvodm_services.h"
#include "gpio_ext_hal.h"
#include "gpio_ext_null.h"
#include "gpio_pcf50626.h"

void
NvOdmExternalGpioWritePins(
    NvU32 Port,
    NvU32 Pin,
    NvU32 PinValue)
{
    static NvBool IsInit = NV_FALSE;
    static NvOdmGpioExtDevice GpioExtDevice;

    if (!IsInit)
    {
        NvOdmOsMemset(&GpioExtDevice, 0, sizeof(GpioExtDevice));
        if (NvOdmPeripheralGetGuid(NV_ODM_GUID('p','c','f','_','p','m','u','0')))
        {
            //  fill in HAL function here.
            GpioExtDevice.pfnWritePins = GPIO_PCF50626_NvOdmExternalGpioWritePins;
        }
        else
        {
            // NULL implementation
            GpioExtDevice.pfnWritePins = null_NvOdmExternalGpioWritePins;
        }
        IsInit = NV_TRUE;
    }
    GpioExtDevice.pfnWritePins(Port, Pin, PinValue);
}

NvU32
NvOdmExternalGpioReadPins(
    NvU32 Port,
    NvU32 Pin)
{
    static NvBool IsInit = NV_FALSE;
    static NvOdmGpioExtDevice GpioExtDevice;

    if (!IsInit)
    {
        NvOdmOsMemset(&GpioExtDevice, 0, sizeof(GpioExtDevice));
        if (NvOdmPeripheralGetGuid(NV_ODM_GUID('p','c','f','_','p','m','u','0')))
        {
            //  fill in HAL function here.
            GpioExtDevice.pfnReadPins = GPIO_PCF50626_NvOdmExternalGpioReadPins;
        }
        else
        {
            // NULL implementation
            GpioExtDevice.pfnReadPins = null_NvOdmExternalGpioReadPins;
        }
        IsInit = NV_TRUE;
    }
    return GpioExtDevice.pfnReadPins(Port, Pin);
}
