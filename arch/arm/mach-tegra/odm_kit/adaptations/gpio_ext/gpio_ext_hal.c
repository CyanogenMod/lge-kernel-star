/*
 * Copyright (c) 2009 NVIDIA Corporation.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * Neither the name of the NVIDIA Corporation nor the names of its contributors
 * may be used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
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
