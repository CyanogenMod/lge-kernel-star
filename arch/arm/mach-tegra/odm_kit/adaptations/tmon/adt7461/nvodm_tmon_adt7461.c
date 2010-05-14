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


#include "nvodm_tmon_adt7461.h"
#include "tmon_hal.h"

// TODO: Always Disable before check-in
// Always debug module: 0=disable, 1=enable
#define NV_ADT7461_DEBUG (0)

#if (NV_DEBUG || NV_ADT7461_DEBUG)
#define NVODM_ADT7461_PRINTF(x)   NvOdmOsDebugPrintf x
#else
#define NVODM_ADT7461_PRINTF(x)
#endif

#define ADT7461_ALERT_DEBOUNCE (1)

// ADT7461 Descrriptor
static const ADT7461Info s_Adt7461Info = 
{
    // TMON device conversion channels
    {
        // Invalid channel
        {0},

        // Local channel
        {
            ADT7461ChannelID_Local,
            {
                ADT7461_ODM_LOCAL_RATE_PROTECTED,
                ADT7461_ODM_LOCAL_INTR_LIMITS_PROTECTED,
                ADT7461_ODM_LOCAL_COMPARATOR_LIMIT_PROTECTED
            },
            {
                ADT7461_LOCAL_INTR_LIMIT_HIGH_RD_ADDR,
                ADT7461_LOCAL_INTR_LIMIT_HIGH_WR_ADDR,
            },
            {
                ADT7461_LOCAL_INTR_LIMIT_LOW_RD_ADDR,
                ADT7461_LOCAL_INTR_LIMIT_LOW_WR_ADDR,
            },
            {
                ADT7461_LOCAL_COMPARATOR_LIMIT_ADDR,
                ADT7461_LOCAL_COMPARATOR_LIMIT_ADDR,
            },
            {
                ADT7461_INVALID_ADDR,   // Local offset does not exist
                ADT7461_INVALID_ADDR,
            },
            {
                ADT7461_LOCAL_TDATA_RD_ADDR,
                ADT7461_INVALID_ADDR,
            },
        },

        // Remote channel
        {
            ADT7461ChannelID_Remote,
            {
                ADT7461_ODM_REMOTE_RATE_PROTECTED,
                ADT7461_ODM_REMOTE_INTR_LIMITS_PROTECTED,
                ADT7461_ODM_REMOTE_COMPARATOR_LIMIT_PROTECTED
            },
            {
                ADT7461_REMOTE_INTR_LIMIT_HIGH_RD_ADDR,
                ADT7461_REMOTE_INTR_LIMIT_HIGH_WR_ADDR,
            },
            {
                ADT7461_REMOTE_INTR_LIMIT_LOW_RD_ADDR,
                ADT7461_REMOTE_INTR_LIMIT_LOW_WR_ADDR,
            },
            {
                ADT7461_REMOTE_COMPARATOR_LIMIT_ADDR,
                ADT7461_REMOTE_COMPARATOR_LIMIT_ADDR,
            },
            {
                ADT7461_REMOTE_TOFFSET_ADDR,
                ADT7461_REMOTE_TOFFSET_ADDR,
            },
            {
                ADT7461_REMOTE_TDATA_RD_ADDR,
                ADT7461_INVALID_ADDR,
            },
        }
    },
    
    // TMON device common status/control registers
    {
        ADT7461_STATUS_RD_ADDR,
        ADT7461_INVALID_ADDR,
    },
    {
        ADT7461_CONFIG_RD_ADDR,
        ADT7461_CONFIG_WR_ADDR,
    },
    {
        ADT7461_RATE_RD_ADDR,
        ADT7461_RATE_WR_ADDR,
    },
    {
        ADT7461_INVALID_ADDR,
        ADT7461_ONE_SHOT_WR_ADDR,
    },
    {
        ADT7461_COMPARATOR_HYSTERESIS_ADDR,
        ADT7461_COMPARATOR_HYSTERESIS_ADDR,
    },
    {
        ADT7461_INTR_CNT_DELAY_ADDR,
        ADT7461_INTR_CNT_DELAY_ADDR,
    },
};

// ADT7461 sample intervals
static const NvS32 s_Adt7461SampleIntervalsMS[] =
{
    ADT7461_SAMPLE_INTERVALS_MS
};

// ADT7461 converison times
static const NvS32 s_Adt7461ConversionTimesMS[] =
{
    ADT7461_CONVERSION_TIME_MS
};

NV_CT_ASSERT(NV_ARRAY_SIZE(s_Adt7461SampleIntervalsMS) ==
             NV_ARRAY_SIZE(s_Adt7461ConversionTimesMS));

/*****************************************************************************/

#define ADT7461_T_DATA_TO_VALUE(ExtRange, data) \
    ( (ExtRange) ? \
        ((NvS32)((NvU32)(data) - ADT7461_RANGE_EXTENDED_DATA_OFFSET)) : \
        ((NvS32)((NvS8)data)) \
    )

#define ADT7461_T_VALUE_TO_DATA(ExtRange, val) \
    ( (ExtRange) ? \
        ((NvU8)((NvU32)(val) + ADT7461_RANGE_EXTENDED_DATA_OFFSET)) : \
        ((NvU8)(val)) \
    )

#define ADT7461_T_RANGE_LIMIT_HIGH(ExtRange) \
    ( (ExtRange) ? \
        ADT7461_RANGE_EXTENDED_LIMIT_HIGH : \
        ADT7461_RANGE_STANDARD_LIMIT_HIGH \
    )

#define ADT7461_T_RANGE_LIMIT_LOW(ExtRange) \
    ( (ExtRange) ? \
        ADT7461_RANGE_EXTENDED_LIMIT_LOW : \
        ADT7461_RANGE_STANDARD_LIMIT_LOW \
    )

/*****************************************************************************/

static NvBool
Adt7461WriteReg(
    ADT7461PrivData* pPrivData,
    const ADT7461RegisterInfo* pReg,
    NvU8 Data)
{  
    NvU32 i;
    NvU8 WriteBuffer[2];
    NvOdmI2cStatus status;
    NvOdmI2cTransactionInfo TransactionInfo;

    NV_ASSERT(pPrivData && pReg);
    NV_ASSERT(pReg->WrAddr != ADT7461_INVALID_ADDR);

    for (i = 0; i < ADT7461_I2C_RETRY_CNT; i++)
    {
        WriteBuffer[0] = pReg->WrAddr;
        WriteBuffer[1] = Data;

        TransactionInfo.Address = pPrivData->DeviceI2cAddr;
        TransactionInfo.Buf = &WriteBuffer[0];
        TransactionInfo.Flags = NVODM_I2C_IS_WRITE;
        TransactionInfo.NumBytes = 2;

        status = NvOdmI2cTransaction(pPrivData->hOdmI2C, &TransactionInfo, 1,
                    ADT7461_I2C_SPEED_KHZ, ADT7461_I2C_TIMEOUT_MS);
        if (status == NvOdmI2cStatus_Success)
            break;
    }

    switch (status)
    {
        case NvOdmI2cStatus_Success:
            pPrivData->ShadowRegPtr = pReg->WrAddr;
            return NV_TRUE;

        case NvOdmI2cStatus_Timeout:
            NVODM_ADT7461_PRINTF(("ADT7461: WriteReg Timeout\n")); 
            return NV_FALSE;

         case NvOdmI2cStatus_SlaveNotFound:
         default:
            NVODM_ADT7461_PRINTF(("ADT7461: WriteReg SlaveNotFound\n"));
            return NV_FALSE;
    }
}

static NvBool
Adt7461ReadReg(
    ADT7461PrivData* pPrivData,
    const ADT7461RegisterInfo* pReg,
    NvU8* pData)
{
    NvU32 i;
    NvU8 Buffer = 0;
    NvOdmI2cStatus status;    
    NvOdmI2cTransactionInfo TransactionInfo[2];

    NV_ASSERT(pPrivData && pReg && pData);
    NV_ASSERT(pReg->RdAddr != ADT7461_INVALID_ADDR);

    // TODO: possible optimization - is shadow pointer matches register
    // address, just send one read transaction (can be done only if Read/Wr
    // Reg routines are serialized).

    for (i = 0; i < ADT7461_I2C_RETRY_CNT; i++)
    {
        Buffer = pReg->RdAddr;

        TransactionInfo[0].Address = pPrivData->DeviceI2cAddr;
        TransactionInfo[0].Buf = &Buffer;
        TransactionInfo[0].Flags = NVODM_I2C_IS_WRITE;
        TransactionInfo[0].NumBytes = 1;

        TransactionInfo[1].Address = (pPrivData->DeviceI2cAddr | 0x1);
        TransactionInfo[1].Buf = &Buffer;
        TransactionInfo[1].Flags = 0;
        TransactionInfo[1].NumBytes = 1;

        status = NvOdmI2cTransaction(pPrivData->hOdmI2C, &TransactionInfo[0], 2,
                    ADT7461_I2C_SPEED_KHZ, ADT7461_I2C_TIMEOUT_MS);
        if (status == NvOdmI2cStatus_Success)
            break;
    }

    switch (status)
    {
        case NvOdmI2cStatus_Success:
            pPrivData->ShadowRegPtr = pReg->RdAddr;
            *pData = Buffer;
            return NV_TRUE;

        case NvOdmI2cStatus_Timeout:
            NVODM_ADT7461_PRINTF(("ADT7461: ReadReg Timeout\n")); 
            return NV_FALSE;

         case NvOdmI2cStatus_SlaveNotFound:
         default:
            NVODM_ADT7461_PRINTF(("ADT7461: ReadReg SlaveNotFound\n"));
            return NV_FALSE;
    }
}

static void Adt7461ReadAra(ADT7461PrivData* pPrivData)
{
    NvU32 i;
    NvU8 Buffer = 0;
    NvOdmI2cStatus status;    
    NvOdmI2cTransactionInfo TransactionInfo;

    NV_ASSERT(pPrivData);

    for (i = 0; i < ADT7461_ARA_RETRY_CNT; i++)
    {
        TransactionInfo.Address = (ADT7461_ARA | 0x1);
        TransactionInfo.Buf = &Buffer;
        TransactionInfo.Flags = 0;
        TransactionInfo.NumBytes = 1;

        status = NvOdmI2cTransaction(pPrivData->hOdmI2C, &TransactionInfo, 1,
                    ADT7461_I2C_SPEED_KHZ, ADT7461_I2C_TIMEOUT_MS);
        if ((status == NvOdmI2cStatus_SlaveNotFound) ||     // False alarm
            ((status == NvOdmI2cStatus_Success) &&
            ((Buffer & 0xFE) == (NvU8)pPrivData->DeviceI2cAddr))  // Cleared ARA
            )
            break;
    }
}

static NvBool
Adt7461ConfigureSampleInterval(
    ADT7461PrivData* pPrivData,
    NvBool OdmProtected,
    NvS32* pTargetMs)
{
    NvU8 i;
    NvS32 Delta;
    const ADT7461RegisterInfo* pReg = &pPrivData->pDeviceInfo->Rate;

    if (OdmProtected ||
        ((*pTargetMs) == ODM_TMON_PARAMETER_UNSPECIFIED))
    {
        // Read ADT7461 rate register (fail the call if returned data
        // does not make sense)
        if(!Adt7461ReadReg(pPrivData, pReg, &i))
            return NV_FALSE;
        if (i >= NV_ARRAY_SIZE(s_Adt7461SampleIntervalsMS))
            return NV_FALSE;
    }
    else
    {
        // Find and set the best floor approximation of the target sample
        // interval. Note the descending order of sample intervals array.
        for (i = 0; i < NV_ARRAY_SIZE(s_Adt7461SampleIntervalsMS); i++)
        {
            Delta = (*pTargetMs) - s_Adt7461SampleIntervalsMS[i];
            if(Delta >= 0)
                break;
        }
        if (i == NV_ARRAY_SIZE(s_Adt7461SampleIntervalsMS))
            i--;    // min interval is the best we can do

        if(!Adt7461WriteReg(pPrivData, pReg, i))
            return NV_FALSE;
        pPrivData->ShadowRate = i;
    }
    *pTargetMs = s_Adt7461SampleIntervalsMS[i];
    return NV_TRUE;
}

/*****************************************************************************/

static void Adt7461Isr(void* arg)
{
    NvU8 Data;
    ADT7461PrivData* pPrivData = (ADT7461PrivData*)arg;
    NvOdmInterruptHandler volatile Callback = pPrivData->Callback;
    void* volatile CallbackArg = pPrivData->CallbackArg;
    const ADT7461RegisterInfo* pReg = NULL;

    if (Callback && CallbackArg)
    {
        Callback(CallbackArg);
    }
#if ADT7461_ALERT_DEBOUNCE
    // New range limits set by callback are not guaranteed to take effect
    // before the next temperature conversion is completed, and interrupt
    // can not be cleared until then. Hence, the debounce delay below.
    NvOdmOsSleepMS(s_Adt7461SampleIntervalsMS[pPrivData->ShadowRate] +
                   s_Adt7461ConversionTimesMS[pPrivData->ShadowRate] + 1);
#endif
    // Read status and ARA to finish clearing interrupt after callback
    pReg = &pPrivData->pDeviceInfo->Status;
    (void)Adt7461ReadReg(pPrivData, pReg, &Data);
    Adt7461ReadAra(pPrivData);

    // Re-enable interrupt
    if (pPrivData->hGpioIntr)
        NvOdmGpioInterruptDone(pPrivData->hGpioIntr);
}

static void Adt7461FreePrivData(ADT7461PrivData* pPrivData)
{
    if (pPrivData)
    {
        if (pPrivData->hGpioIntr)
        {
            NvOdmGpioInterruptUnregister(
                pPrivData->hGpio, pPrivData->hGpioPin, pPrivData->hGpioIntr);
        }
        NvOdmI2cClose(pPrivData->hOdmI2C);
        NvOdmGpioReleasePinHandle(pPrivData->hGpio, pPrivData->hGpioPin);
        NvOdmGpioClose(pPrivData->hGpio);
        NvOdmServicesPmuClose(pPrivData->hOdmPmuSevice);
        NvOdmOsFree(pPrivData);
    }
}

/*****************************************************************************/

NvBool Adt7461Init(NvOdmTmonDeviceHandle hTmon)
{
    NvU8 Data;
    NvBool ExtRange;
    NvU32 i = 0;
    NvU32 I2cInstance = 0;
    NvOdmIoModule I2cModule = NvOdmIoModule_Num;    // Inavlid module
    const ADT7461RegisterInfo* pReg = NULL;
    ADT7461PrivData* pPrivData = NULL;
    
    NV_ASSERT(hTmon && hTmon->pConn && hTmon->pConn->AddressList);
    
    // Allocate and clear priavte data
    pPrivData = (ADT7461PrivData*) NvOdmOsAlloc(sizeof(ADT7461PrivData));
    if (pPrivData == NULL)
    {
        NVODM_ADT7461_PRINTF(("ADT7461: Error Allocating PrivData. \n"));
        return NV_FALSE;
    }
    NvOdmOsMemset(pPrivData, 0, sizeof(ADT7461PrivData));
    hTmon->pPrivate = pPrivData;

    // Register for PMU services
    pPrivData->hOdmPmuSevice = NvOdmServicesPmuOpen();
    if (pPrivData->hOdmPmuSevice == NULL)
    {
        NVODM_ADT7461_PRINTF(("ADT7461: Error Open PMU service. \n"));
        goto fail;
    }

    // Register for GPIO services
    pPrivData->hGpio = NvOdmGpioOpen();
    if (pPrivData->hOdmPmuSevice == NULL)
    {
        NVODM_ADT7461_PRINTF(("ADT7461: Error Open GPIO service. \n"));
        goto fail;
    }

    /*
     * Parse connectivity data: turn On power to the device, acquire I2C
     * interface and GPIO interrupt (optional); map device channels to
     * thermal zones
     */
    for (i = 0; i < hTmon->pConn->NumAddress; i ++)
    {
        const NvOdmIoAddress* pIoAddress = &hTmon->pConn->AddressList[i];
        if (pIoAddress->Interface == NvOdmIoModule_I2c_Pmu)
        {
            I2cModule   = NvOdmIoModule_I2c_Pmu;
            I2cInstance = pIoAddress->Instance;
            NV_ASSERT(pIoAddress->Address != 0);
            pPrivData->DeviceI2cAddr = pIoAddress->Address;
        }
        else if (pIoAddress->Interface == NvOdmIoModule_Tsense)
        {
            NV_ASSERT(pIoAddress->Instance < NvOdmTmonZoneID_Num);
            NV_ASSERT(pIoAddress->Address < ADT7461ChannelID_Num);
            pPrivData->ConnectivityMap[pIoAddress->Instance] =
                pIoAddress->Address;
        }
        else if (pIoAddress->Interface == NvOdmIoModule_Vdd)
        {
            NvU32 usec = 0;
            NvU32 RailAddress = pIoAddress->Address;
            NvOdmServicesPmuVddRailCapabilities RailCapabilities = {0};
            NvOdmServicesPmuGetCapabilities(
                pPrivData->hOdmPmuSevice, RailAddress, &RailCapabilities);
            NvOdmServicesPmuSetVoltage(pPrivData->hOdmPmuSevice, RailAddress,
                              RailCapabilities.requestMilliVolts, &usec);
            NvOdmOsWaitUS(usec + (ADT7461_POWERUP_DELAY_MS * 1000));
        }
        else if (pIoAddress->Interface == NvOdmIoModule_Gpio)
        {
            NvU32 port = pIoAddress->Instance;
            NvU32 pin = pIoAddress->Address;
            pPrivData->hGpioPin = NvOdmGpioAcquirePinHandle(
                pPrivData->hGpio, port, pin);
        }

    }
    NV_ASSERT(I2cModule == NvOdmIoModule_I2c_Pmu);
    pPrivData->hOdmI2C = NvOdmI2cOpen(I2cModule, I2cInstance);
    if (pPrivData->hOdmI2C == NULL)
    {
        NVODM_ADT7461_PRINTF(("ADT7461: Error Open I2C device. \n"));     
        goto fail;
    }

    /*
     * Initialize device info and configuration. Force standby mode to avoid
     * glitch on shutdown comparator output when temperature range and/or
     * comparator limit is changing during initialization. The Adt7461Run()
     * call from the hal that follows initialization will switch device to
     * run mode and re-start temperature monitoring (note that out of limit
     * interrupt is always masked during and after initialization)
     */
    pPrivData->pDeviceInfo = &s_Adt7461Info;
    pPrivData->ShadowRegPtr = ADT7461_INVALID_ADDR;

    pReg = &pPrivData->pDeviceInfo->Config;
    if (!Adt7461ReadReg(pPrivData, pReg, &Data))
        goto fail;
    if ((Data & ADT7461ConfigBits_ExtendedRange) !=
        (ADT7461_INITIAL_CONFIG & ADT7461ConfigBits_ExtendedRange))
    {
        // Only switch from standard to extended range is supported
        NV_ASSERT((Data & ADT7461ConfigBits_ExtendedRange) == 0);
        Data |= ADT7461ConfigBits_Standby;
        if(!Adt7461WriteReg(pPrivData, pReg, Data))
            goto fail;
    }
    Data = ADT7461_INITIAL_CONFIG | ADT7461ConfigBits_Standby;
    if(!Adt7461WriteReg(pPrivData, pReg, Data))
        goto fail;
    pPrivData->ShadowConfig = Data;
    ExtRange = ((Data & ADT7461ConfigBits_ExtendedRange) != 0);

    // Program shutdown comparators settings
    Data = ADT7461_T_VALUE_TO_DATA(
        ExtRange, ADT7461_ODM_LOCAL_COMPARATOR_LIMIT_VALUE);
    pReg = &pPrivData->pDeviceInfo->Channels[
        ADT7461ChannelID_Local].ComparatorLimit;
    if(!Adt7461WriteReg(pPrivData, pReg, Data))
        goto fail;

    Data = ADT7461_T_VALUE_TO_DATA(
        ExtRange, ADT7461_ODM_REMOTE_COMPARATOR_LIMIT_VALUE);
    pReg = &pPrivData->pDeviceInfo->Channels[
        ADT7461ChannelID_Remote].ComparatorLimit;
    if(!Adt7461WriteReg(pPrivData, pReg, Data))
        goto fail;

    // Set interrupt limits to the range boundaries to prevent out of limit
    // interrupt
    Data = ADT7461_T_VALUE_TO_DATA(
        ExtRange, ADT7461_T_RANGE_LIMIT_HIGH(ExtRange)); 
    pReg = &pPrivData->pDeviceInfo->Channels[
        ADT7461ChannelID_Local].IntrLimitHigh;
    if(!Adt7461WriteReg(pPrivData, pReg, Data))
        goto fail;
    pReg = &pPrivData->pDeviceInfo->Channels[
        ADT7461ChannelID_Remote].IntrLimitHigh;
    if(!Adt7461WriteReg(pPrivData, pReg, Data))
        goto fail;

    Data = ADT7461_T_VALUE_TO_DATA(
            ExtRange, ADT7461_T_RANGE_LIMIT_LOW(ExtRange));
    pReg = &pPrivData->pDeviceInfo->Channels[
        ADT7461ChannelID_Local].IntrLimitLow;
    if(!Adt7461WriteReg(pPrivData, pReg, Data))
        goto fail;
    pReg = &pPrivData->pDeviceInfo->Channels[
        ADT7461ChannelID_Remote].IntrLimitLow;
    if(!Adt7461WriteReg(pPrivData, pReg, Data))
        goto fail;

    // Set initial rate
    Data = ADT7461_INITIAL_RATE_SETTING;  
    pReg = &pPrivData->pDeviceInfo->Rate;
    if(!Adt7461WriteReg(pPrivData, pReg, Data))
        goto fail;
    pPrivData->ShadowRate = Data;

    // Set remote channel offset (8-bit 2's complement value for any range)
    Data = ((NvU8)ADT7461_ODM_REMOTE_OFFSET_VALUE);
    pReg = &pPrivData->pDeviceInfo->Channels[
        ADT7461ChannelID_Remote].Toffset;
    if(!Adt7461WriteReg(pPrivData, pReg, Data))
        goto fail;

    // Read ADT7461 status and ARA (clear pending Alert interrupt, if any)
    pReg = &pPrivData->pDeviceInfo->Status;
    if (!Adt7461ReadReg(pPrivData, pReg, &Data))
        goto fail;
    // TODO: check open remote circuit error

    Adt7461ReadAra(pPrivData);
    return NV_TRUE;

fail:
    Adt7461FreePrivData(pPrivData);
    hTmon->pPrivate = NULL;
    return NV_FALSE;
}

void Adt7461Deinit(NvOdmTmonDeviceHandle hTmon)
{
    if (hTmon && hTmon->pPrivate)
    {
        ADT7461PrivData* pPrivData = hTmon->pPrivate;
        (void)Adt7461WriteReg(pPrivData, &pPrivData->pDeviceInfo->Config,
            ADT7461_INITIAL_CONFIG);    //leave device in default configuration
                                        // with power rail ON (forever)
        Adt7461FreePrivData(pPrivData);
        hTmon->pPrivate = NULL;
    }
}

/*****************************************************************************/

NvBool Adt7461Run(NvOdmTmonDeviceHandle hTmon, NvOdmTmonZoneID ZoneId)
{
    NvU8 Data; 
    NvBool IsRunning;
    ADT7461PrivData* pPrivData;

    NV_ASSERT(hTmon && hTmon->pPrivate);
    pPrivData = hTmon->pPrivate;
    IsRunning = (pPrivData->ShadowConfig & ADT7461ConfigBits_Standby) == 0;

    if (!IsRunning)
    {
        Data = pPrivData->ShadowConfig & (~ADT7461ConfigBits_Standby);
        if(!Adt7461WriteReg(pPrivData, &pPrivData->pDeviceInfo->Config, Data))
            return NV_FALSE;
        pPrivData->ShadowConfig = Data;
    }
    pPrivData->RunRefCount++;
    return NV_TRUE;
}

NvBool Adt7461Stop(NvOdmTmonDeviceHandle hTmon, NvOdmTmonZoneID ZoneId)
{
    NvU8 Data; 
    NvBool IsRunning;
    ADT7461PrivData* pPrivData;

    NV_ASSERT(hTmon && hTmon->pPrivate);
    pPrivData = hTmon->pPrivate;
    IsRunning = (pPrivData->ShadowConfig & ADT7461ConfigBits_Standby) == 0;

    if (ADT7461_ODM_STANDBY_ENABLED &&
        IsRunning && (pPrivData->RunRefCount == 1))
    {
        Data = pPrivData->ShadowConfig | ADT7461ConfigBits_Standby;
        if(!Adt7461WriteReg(pPrivData, &pPrivData->pDeviceInfo->Config, Data))
            return NV_FALSE;
        pPrivData->ShadowConfig = Data;
    }
    if (pPrivData->RunRefCount != 0)
    {
        pPrivData->RunRefCount--;
        return NV_TRUE;
    }
    NV_ASSERT(!"RunRefCount balance failed");
    NVODM_ADT7461_PRINTF(("ADT7461: RunRefCount balance failed. \n"));
    return NV_FALSE;
}

/*****************************************************************************/
// ADT7461 aborts and restarts conversion cycle when temperature is read
// (actually on any I2C access for that matter, but other accesses are rare).
// TODO: add time stamps and implement refresh policy to make sure that
// frequent temperature reads would not stall the conversion forever.

NvBool 
Adt7461TemperatureGet(
    NvOdmTmonDeviceHandle hTmon,
    NvOdmTmonZoneID ZoneId,
    NvS32* pDegreesC)
{
    NvU8 Data;
    NvBool ExtRange;
    ADT7461ChannelID ChannelId;
    ADT7461PrivData* pPrivData;
    const ADT7461RegisterInfo* pReg;

    NV_ASSERT(hTmon && hTmon->pPrivate && pDegreesC);
    pPrivData = hTmon->pPrivate;
    ExtRange = ((pPrivData->ShadowConfig &
                 ADT7461ConfigBits_ExtendedRange) != 0);
    ChannelId = pPrivData->ConnectivityMap[ZoneId];
    pReg = &pPrivData->pDeviceInfo->Channels[ChannelId].Tdata;

    if(!Adt7461ReadReg(pPrivData, pReg, &Data))
        return NV_FALSE;

    *pDegreesC = ADT7461_T_DATA_TO_VALUE(ExtRange, Data);
    return NV_TRUE;
}

/*****************************************************************************/

void 
Adt7461CapabilitiesGet(
    NvOdmTmonDeviceHandle hTmon,
    NvOdmTmonZoneID ZoneId,
    NvOdmTmonCapabilities* pCaps)
{
    NvBool ExtRange;
    ADT7461PrivData* pPrivData;

    NV_ASSERT(hTmon && hTmon->pPrivate && pCaps);
    pPrivData = hTmon->pPrivate;
    ExtRange = ((pPrivData->ShadowConfig &
                 ADT7461ConfigBits_ExtendedRange) != 0);
    
    pCaps->Tmax = ADT7461_T_RANGE_LIMIT_HIGH(ExtRange);
    pCaps->Tmin = ADT7461_T_RANGE_LIMIT_LOW(ExtRange);
    pCaps->IntrSupported = NV_TRUE;
    pCaps->HwCriticalSupported = NV_TRUE;
    pCaps->HwCoolingSupported = NV_FALSE;
}

void
Adt7461ParameterCapsGet(
    NvOdmTmonDeviceHandle hTmon,
    NvOdmTmonZoneID ZoneId,
    NvOdmTmonConfigParam ParamId,
    NvOdmTmonParameterCaps* pCaps)
{
    NvBool ExtRange;
    ADT7461PrivData* pPrivData;
    const ADT7461ChannelInfo* pChannel;

    NV_ASSERT(hTmon && hTmon->pPrivate && pCaps);
    pPrivData = hTmon->pPrivate;
    ExtRange = ((pPrivData->ShadowConfig &
                 ADT7461ConfigBits_ExtendedRange) != 0);
    pChannel = &pPrivData->pDeviceInfo->Channels[(
        pPrivData->ConnectivityMap[ZoneId])];

    switch (ParamId)
    {
        case NvOdmTmonConfigParam_IntrLimitHigh:
        case NvOdmTmonConfigParam_IntrLimitLow:
            pCaps->OdmProtected =
                pChannel->ChannelPolicy.IntrLimitsOdmProtected;
            break;

        case NvOdmTmonConfigParam_HwLimitCrit:
            pCaps->OdmProtected =
                pChannel->ChannelPolicy.HwLimitCritOdmProtected;
            break;

        case NvOdmTmonConfigParam_SampleMs:
            // smaple intervals in descending order
            pCaps->MaxValue = s_Adt7461SampleIntervalsMS[0];
            pCaps->MinValue = s_Adt7461SampleIntervalsMS[(
                NV_ARRAY_SIZE(s_Adt7461SampleIntervalsMS) - 1)];
            pCaps->OdmProtected = pChannel->ChannelPolicy.RateOdmProtected;
            return;

        default:        // unsupported parameter
            pCaps->MaxValue = ODM_TMON_PARAMETER_UNSPECIFIED;
            pCaps->MinValue = ODM_TMON_PARAMETER_UNSPECIFIED;
            pCaps->OdmProtected = NV_TRUE;
            return;
    }

    // Common range for limits
    pCaps->MaxValue = ADT7461_T_RANGE_LIMIT_HIGH(ExtRange);
    pCaps->MinValue = ADT7461_T_RANGE_LIMIT_LOW(ExtRange);
}

NvBool
Adt7461ParameterConfig(
    NvOdmTmonDeviceHandle hTmon,
    NvOdmTmonZoneID ZoneId,
    NvOdmTmonConfigParam ParamId,
    NvS32* pSetting)
{
    NvU8 Data;
    NvBool ExtRange, OdmProtected;
    ADT7461PrivData* pPrivData;
    const ADT7461RegisterInfo* pReg;
    const ADT7461ChannelInfo* pChannel;

    NV_ASSERT(hTmon && hTmon->pPrivate && pSetting);
    pPrivData = hTmon->pPrivate;
    ExtRange = ((pPrivData->ShadowConfig &
                 ADT7461ConfigBits_ExtendedRange) != 0);
    pChannel = &pPrivData->pDeviceInfo->Channels[(
        pPrivData->ConnectivityMap[ZoneId])];

    switch (ParamId)
    {
        case NvOdmTmonConfigParam_IntrLimitHigh:
            pReg = &pChannel->IntrLimitHigh;
            OdmProtected = pChannel->ChannelPolicy.IntrLimitsOdmProtected;
            break;

        case NvOdmTmonConfigParam_IntrLimitLow:
            pReg = &pChannel->IntrLimitLow;
            OdmProtected = pChannel->ChannelPolicy.IntrLimitsOdmProtected;
            break;

        case NvOdmTmonConfigParam_HwLimitCrit:
            pReg = &pChannel->ComparatorLimit;
            OdmProtected = pChannel->ChannelPolicy.HwLimitCritOdmProtected;
            break;

        case NvOdmTmonConfigParam_SampleMs:
            OdmProtected = pChannel->ChannelPolicy.RateOdmProtected;
            return Adt7461ConfigureSampleInterval(
                pPrivData, OdmProtected, pSetting);

        default:        // unsupported parameter
            *pSetting = ODM_TMON_PARAMETER_UNSPECIFIED;
            return NV_TRUE;
    }

    // Common processing for temperature limits configuration
    if ((OdmProtected) ||
        ((*pSetting) == ODM_TMON_PARAMETER_UNSPECIFIED))
    {
        // Read ADT7461 register and convert data to current parameter value
        if(!Adt7461ReadReg(pPrivData, pReg, &Data))
            return NV_FALSE;

        *pSetting = ADT7461_T_DATA_TO_VALUE(ExtRange, Data);
    }
    else
    {
        // Clip target setting to temperature range 
        if ((*pSetting) > ADT7461_T_RANGE_LIMIT_HIGH(ExtRange))
            *pSetting = ADT7461_T_RANGE_LIMIT_HIGH(ExtRange);
        else if ((*pSetting) < ADT7461_T_RANGE_LIMIT_LOW(ExtRange))
            *pSetting = ADT7461_T_RANGE_LIMIT_LOW(ExtRange);

        // Convert new configuration setting and write to ADT7461 register
        Data = ADT7461_T_VALUE_TO_DATA(ExtRange, *pSetting);
        if(!Adt7461WriteReg(pPrivData, pReg, Data))
            return NV_FALSE;
    }
    return NV_TRUE;
}

/*****************************************************************************/

NvOdmTmonIntrHandle
Adt7461IntrRegister(
    NvOdmTmonDeviceHandle hTmon,
    NvOdmTmonZoneID ZoneId,
    NvOdmInterruptHandler Callback,
    void* CallbackArg)
{
    NvU8 Data;
    ADT7461PrivData* pPrivData;
    const ADT7461ChannelInfo* pChannel;
    NvOdmServicesGpioIntrHandle hGpioIntr = NULL;

    NV_ASSERT(hTmon && hTmon->pPrivate && Callback && CallbackArg);
    pPrivData = hTmon->pPrivate;

    // No registration, if no GPIO pin available or interrupt already registred
    if (!pPrivData->hGpioPin || pPrivData->hGpioIntr)
        return NULL;

    // No registration for other than remote channel
    pChannel = &pPrivData->pDeviceInfo->Channels[(
        pPrivData->ConnectivityMap[ZoneId])];
    if (pChannel->ChannelId != ADT7461ChannelID_Remote)
        return NULL;

    // Register GPIO interrupt (will be enabled at SoC IC, but still disabled
    // at ADT7461 device)
    pPrivData->Callback = Callback;
    pPrivData->CallbackArg = CallbackArg;
    if (!NvOdmGpioInterruptRegister(
        pPrivData->hGpio, &hGpioIntr, pPrivData->hGpioPin,
        ADT7461_ODM_INTR_POLARITY, Adt7461Isr, (void *)pPrivData, 0))
    {
        pPrivData->Callback = NULL;
        pPrivData->CallbackArg = NULL;
        return NULL;
    }
    NV_ASSERT(hGpioIntr);
    pPrivData->hGpioIntr = hGpioIntr;
    
    // Finally enable ADT7461 device interrupt output (interrupt may or may
    // not be generated depending on temperature and limt settings).
    Data = pPrivData->ShadowConfig & (~ADT7461ConfigBits_IntrDisabled);
    if(!Adt7461WriteReg(pPrivData, &pPrivData->pDeviceInfo->Config, Data))
    {
        NvOdmGpioInterruptUnregister(
            pPrivData->hGpio, pPrivData->hGpioPin, hGpioIntr);
        pPrivData->Callback = NULL;
        pPrivData->CallbackArg = NULL;
        pPrivData->hGpioIntr = NULL;
        return NULL;
    }
    pPrivData->ShadowConfig = Data;

    return (NvOdmTmonIntrHandle)hGpioIntr;
}

void
Adt7461IntrUnregister(
    NvOdmTmonDeviceHandle hTmon,
    NvOdmTmonZoneID ZoneId,
    NvOdmTmonIntrHandle hIntr)
{
    NvU8 Data;
    ADT7461PrivData* pPrivData;
    const ADT7461ChannelInfo* pChannel;

    // Ignore invalid handles
    if(!hIntr || !hTmon || !hTmon->pPrivate) 
        return;

    pPrivData = hTmon->pPrivate;
    if (hIntr != ((NvOdmTmonIntrHandle)pPrivData->hGpioIntr))
        return;

    // Ignore any channel other than remote
    pChannel = &pPrivData->pDeviceInfo->Channels[(
        pPrivData->ConnectivityMap[ZoneId])];
    if (pChannel->ChannelId != ADT7461ChannelID_Remote)
        return;

    // Disable ADT7461 interrupt output
    Data = pPrivData->ShadowConfig | ADT7461ConfigBits_IntrDisabled;
    if(Adt7461WriteReg(pPrivData, &pPrivData->pDeviceInfo->Config, Data))
        pPrivData->ShadowConfig = Data;

    // Unregister GPIO interrupt, clear callbacks and handle
    NvOdmGpioInterruptUnregister(
        pPrivData->hGpio, pPrivData->hGpioPin, pPrivData->hGpioIntr);

    pPrivData->Callback = NULL;
    pPrivData->CallbackArg = NULL;
    pPrivData->hGpioIntr = NULL;
}

/*****************************************************************************/

