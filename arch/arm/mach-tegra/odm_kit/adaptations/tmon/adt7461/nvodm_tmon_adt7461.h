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
 
#ifndef INCLUDED_NVODM_TMON_ADT7461_H
#define INCLUDED_NVODM_TMON_ADT7461_H

#include "nvodm_tmon.h"
#include "nvodm_tmon_adt7461_reg.h"
#include "nvodm_tmon_adt7461_channel.h"

#if defined(__cplusplus)
extern "C"
{
#endif

typedef struct ADT7461RegisterInfoRec
{
    NvU8 RdAddr;            // Invalid if WO
    NvU8 WrAddr;            // Invalid if RO
} ADT7461RegisterInfo;

typedef struct ADT7461ChannelOdmPolicyRec
{
    NvBool RateOdmProtected;
    NvBool IntrLimitsOdmProtected;
    NvBool HwLimitCritOdmProtected;
} ADT7461ChannelOdmPolicy;

typedef struct ADT7461ChannelInfoRec
{
    // TMON device conversion channel ID
    ADT7461ChannelID  ChannelId;

    // ODM channel policy
    ADT7461ChannelOdmPolicy ChannelPolicy;

    // Alert Interrupt limits registers
    ADT7461RegisterInfo IntrLimitHigh;
    ADT7461RegisterInfo IntrLimitLow;

    // Thermal comparator limit register
    ADT7461RegisterInfo ComparatorLimit;

    // Temperature measurement offset
    ADT7461RegisterInfo Toffset;

    // Temperature Data register
    ADT7461RegisterInfo Tdata;
} ADT7461ChannelInfo;

typedef struct ADT7461InfoRec
{
    // TMON device conversion channels
    ADT7461ChannelInfo  Channels[ADT7461ChannelID_Num];

    // Chip status register
    ADT7461RegisterInfo Status;

    // Common configration controls
    ADT7461RegisterInfo Config;

    // Common conversion rate
    ADT7461RegisterInfo Rate;

    // One-shot trigger register
    ADT7461RegisterInfo OneShot;

    // Common comparator hysteresis 
    ADT7461RegisterInfo ComparatorHysteresis;

    // Number of consecutive limit violation before
    // interrupt is generated
    ADT7461RegisterInfo IntrCntDelay;
} ADT7461Info;

typedef struct ADT7461PrivDataRec
{
    // ADT7461 device registers descriptors
    const ADT7461Info* pDeviceInfo;

    // ADT7461 I2C device Address
    NvU32 DeviceI2cAddr;

    // The handle to the I2C controller
    NvOdmServicesI2cHandle hOdmI2C;

    // The odm pmu service handle
    NvOdmServicesPmuHandle hOdmPmuSevice;

    // Zone => Channel map
    ADT7461ChannelID ConnectivityMap[NvOdmTmonZoneID_Num];

    // ADR7461 run mode reference count 
    NvU32 RunRefCount;

    // Shadow of ADT7461 internal configuration register
    NvU8 ShadowConfig;

    // Shadow of ADT7461 internal rate settings
    NvU8 ShadowRate;

    // Shadow of ADT7461 internal address pointer
    NvU8 ShadowRegPtr;

    // The odm GPIO service handle
    NvOdmServicesGpioHandle hGpio;

    // SoC GPIO dedicated for ADT7461 out of limit interrupt
    NvOdmGpioPinHandle hGpioPin;

    // The ADT7461 interrupt handle
    NvOdmServicesGpioIntrHandle hGpioIntr;

    // The ADT7461 interrupt callback
    NvOdmInterruptHandler Callback;

    // The ADT7461 interrupt callback context
    void* CallbackArg;
} ADT7461PrivData;

NvBool Adt7461Init(NvOdmTmonDeviceHandle hTmon);
void Adt7461Deinit(NvOdmTmonDeviceHandle hTmon);
NvBool Adt7461Run(NvOdmTmonDeviceHandle hTmon, NvOdmTmonZoneID ZoneId);
NvBool Adt7461Stop(NvOdmTmonDeviceHandle hTmon, NvOdmTmonZoneID ZoneId);

NvBool 
Adt7461TemperatureGet(
    NvOdmTmonDeviceHandle hTmon,
    NvOdmTmonZoneID ZoneId,
    NvS32* pDegreesC);

void 
Adt7461CapabilitiesGet(
    NvOdmTmonDeviceHandle hTmon,
    NvOdmTmonZoneID ZoneId,
    NvOdmTmonCapabilities* pCaps);

void
Adt7461ParameterCapsGet(
    NvOdmTmonDeviceHandle hTmon,
    NvOdmTmonZoneID ZoneId,
    NvOdmTmonConfigParam ParamId,
    NvOdmTmonParameterCaps* pCaps);

NvBool
Adt7461ParameterConfig(
    NvOdmTmonDeviceHandle hTmon,
    NvOdmTmonZoneID ZoneId,
    NvOdmTmonConfigParam ParamId,
    NvS32* pSetting);

NvOdmTmonIntrHandle
Adt7461IntrRegister(
    NvOdmTmonDeviceHandle hTmon,
    NvOdmTmonZoneID ZoneId,
    NvOdmInterruptHandler Callback,
    void* arg);

void
Adt7461IntrUnregister(
    NvOdmTmonDeviceHandle hTmon,
    NvOdmTmonZoneID ZoneId,
    NvOdmTmonIntrHandle hIntr);

#if defined(__cplusplus)
}
#endif

#endif //INCLUDED_NVODM_TMON_ADT7461_H

