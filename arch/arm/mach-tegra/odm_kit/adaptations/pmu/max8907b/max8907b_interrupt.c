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

#include "max8907b.h"
#include "max8907b_interrupt.h"
#include "max8907b_i2c.h"
#include "max8907b_reg.h"
#include "max8907b_batterycharger.h"
#include "nvodm_services.h"

NvBool 
Max8907bSetupInterrupt(
    NvOdmPmuDeviceHandle hDevice,
    Max8907bStatus *pmuStatus)
{
    NvBool status = NV_FALSE;
    NvU8   data   = 0;    

    NV_ASSERT(hDevice);
    NV_ASSERT(pmuStatus);
    
    /* Init Pmu Status */
    pmuStatus->lowBatt       = NV_FALSE;
    pmuStatus->highTemp      = NV_FALSE;

    if (!Max8907bBatteryChargerMainBatt(hDevice, &status))
        return NV_FALSE;
    pmuStatus->mChgPresent = status;

    /* Set up Interrupt Mask */

    // CHG_IRQ1
    data = 0;
    if (!Max8907bI2cWrite8(hDevice, MAX8907B_CHG_IRQ1, data))
        return NV_FALSE;

    // CHG_IRQ2
    data = MAX8907B_CHG_IRQ2_CHG_DONE_MASK      |
           MAX8907B_CHG_IRQ2_MBATTLOW_F_SHIFT   |
           MAX8907B_CHG_IRQ2_THM_OK_F_MASK      |
           MAX8907B_CHG_IRQ2_THM_OK_R_MASK      ;
    if (!Max8907bI2cWrite8(hDevice, MAX8907B_CHG_IRQ2, data))
        return NV_FALSE;

    // ON_OFF_IRQ1
    data = 0;
    if (!Max8907bI2cWrite8(hDevice, MAX8907B_ON_OFF_IRQ1, data))
        return NV_FALSE;

    // ON_OFF_IRQ2
    data = 0;
    if (!Max8907bI2cWrite8(hDevice, MAX8907B_ON_OFF_IRQ2, data))
        return NV_FALSE;

    // RTC_IRQ
    data = 0;
    if (!Max8907bI2cWrite8(hDevice, MAX8907B_RTC_IRQ, data))
        return NV_FALSE;

    return NV_TRUE;
}

void 
Max8907bInterruptHandler_int(
    NvOdmPmuDeviceHandle hDevice,
    Max8907bStatus *pmuStatus)
{
    NvU8 data = 0;

    NV_ASSERT(hDevice);
    NV_ASSERT(pmuStatus);

    /* Check which interrupt... */

    // CHG_IRQ1
    if (!Max8907bI2cRead8(hDevice, MAX8907B_CHG_IRQ1, &data))
    {
        return;
    }

    if (data)
    {
        // VBUS connect interrupt
        if (data &
           (MAX8907B_CHG_IRQ1_VCHG_R_MASK << MAX8907B_CHG_IRQ1_VCHG_R_SHIFT))
        {
            NvOdmEnableOtgCircuitry(NV_TRUE);
        }
        // VBUS dis-connect interrupt
        else if (data &
                (MAX8907B_CHG_IRQ1_VCHG_F_MASK << MAX8907B_CHG_IRQ1_VCHG_F_SHIFT))
        {
            NvOdmEnableOtgCircuitry(NV_FALSE);
        }
    }

    // CHG_IRQ2
    if (!Max8907bI2cRead8(hDevice, MAX8907B_CHG_IRQ2, &data))
    {
        return;
    }
    if (data)
    {
        if (data & MAX8907B_CHG_IRQ2_CHG_DONE_MASK)
        {
            pmuStatus->mChgPresent = NV_TRUE;
            pmuStatus->batFull = NV_TRUE;
        }
        if (data & MAX8907B_CHG_IRQ2_MBATTLOW_F_SHIFT)
        {
            pmuStatus->batFull = NV_FALSE;
        }
        if (data & MAX8907B_CHG_IRQ2_THM_OK_F_MASK)
        {
            pmuStatus->highTemp = NV_TRUE;
        }
        if (data & MAX8907B_CHG_IRQ2_THM_OK_R_MASK)
        {
            pmuStatus->highTemp = NV_FALSE;
        }
    }

    // ON_OFF_IRQ1
    if (!Max8907bI2cRead8(hDevice, MAX8907B_ON_OFF_IRQ1, &data))
    {
        return;
    }

    // ON_OFF_IRQ2
    if (!Max8907bI2cRead8(hDevice, MAX8907B_ON_OFF_IRQ2, &data))
    {
        return;
    }

    // RTC_IRQ
    if (!Max8907bI2cRead8(hDevice, MAX8907B_RTC_IRQ, &data))
    {
        return;
    }

    return;
}

