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

#ifndef INCLUDED_TPS6586X_INTERRUPT_HEADER
#define INCLUDED_TPS6586X_INTERRUPT_HEADER

#include "nvodm_pmu_tps6586x.h"

#if defined(__cplusplus)
extern "C"
{
#endif

typedef struct TPS6586xStatusRef
{
    /* Low Battery voltage detected by BVM */
    NvBool lowBatt;

    /* PMU high temperature */
    NvBool highTemp;

    /* Main charger Presents */
    NvBool mChgPresent;

    /* battery Full */
    NvBool batFull;

    /* Porwer In type*/
    NvU32   powerType;      /* Bit meanings:
                                0: AC_DET, 
                                1: USB_DET,
                                2: BAT_DET */                           
    NvU32   powerGood;      /* Bit meanings:
                                0-7: LDO0 to LDO7 
                                10-11: LDO8 and LDO9,
                                12-15: SMO0 to SM11 */
}TPS6586xStatus;

#if 0
typedef struct {
    NvBool pmuInterruptSupported;
    NvBool pmuPresented; 
    NvBool battPresence;
    TPS6586xStatus pmuStatus;
} TPS6586xDevice, *TPS6586xHandle;
#endif

NvBool 
Tps6586xSetupInterrupt(
    NvOdmPmuDeviceHandle hDevice,
    TPS6586xStatus *pmuStatus);



void 
Tps6586xInterruptHandler_int(
    NvOdmPmuDeviceHandle hDevice,
    TPS6586xStatus *pmuStatus);

#if defined(__cplusplus)
}
#endif

#endif // INCLUDED_TPS6586X_INTERRUPT_HEADER
