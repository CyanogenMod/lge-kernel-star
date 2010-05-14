/*
 * Copyright (c) 2010 NVIDIA Corporation.
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

/*  NVIDIA Tegra ODM Kit Sample Accelerometer Adaptation of the
 *  WinCE Accelerometer Driver
 */

#include "nvodm_accelerometer_bma150.h"
#include "nvodm_services.h"
#include "nvodm_query_discovery.h"
#include "nvos.h"

#define NVODMACCELEROMETER_ENABLE_PRINTF 0

#if NVODMACCELEROMETER_ENABLE_PRINTF
    #define NVODMACCELEROMETER_PRINTF(x) \
    do { \
        NvOdmOsPrintf x; \
    } while (0)
#else
    #define NVODMACCELEROMETER_PRINTF(x)
#endif

#define NV_BMA150_MAX_FORCE_IN_REG 512 // It indicates force register length.
#define NV_DEBOUNCE_TIME_MS 0

#define ENABLE_XYZ_POLLING 0

//FIXME: protect this variable using spinlock.
static volatile int g_WaitCounter = 0;
static void BMA150_ResetInterrupt(NvOdmAccelHandle hDevice);

static void
SetPowerRail(
    NvOdmServicesPmuHandle hPMUDevice,
    NvU32 Id,
    NvBool IsEnable)
{
    NvOdmServicesPmuVddRailCapabilities vddrailcap;
    NvU32 settletime;

    if (hPMUDevice && Id)
    {
        NvOdmServicesPmuGetCapabilities(hPMUDevice, Id, &vddrailcap);
        if (IsEnable)
        {
            NvOdmServicesPmuSetVoltage(hPMUDevice, Id,
                vddrailcap.requestMilliVolts, &settletime);
        }
        else
        {
            NvOdmServicesPmuSetVoltage(hPMUDevice, Id,
                vddrailcap.MinMilliVolts, &settletime);
        }
        NvOdmOsWaitUS(settletime);
    }
}

static void GpioInterruptHandler(void *arg)
{
    NvU32 pinValue;
    NvOdmAccelHandle hDevice =  (NvOdmAccelHandle)arg;

    NvOdmGpioGetState(hDevice->hGpioINT, hDevice->hPinINT, &pinValue);
    if (pinValue == 1)
    {
        NVODMACCELEROMETER_PRINTF(("\r\nBMA150 Interrupt"));
        g_WaitCounter = 10;
        BMA150_ResetInterrupt(hDevice);
    } else
        NVODMACCELEROMETER_PRINTF(("\r\nBMA150 non-Interrupt"));

    if (pinValue == 1)
    {
        NvOdmOsSemaphoreSignal(hDevice->SemaphoreForINT);
    }
    NvOdmGpioInterruptDone(hDevice->hGpioInterrupt);
    return;
}

static NvBool ConnectSemaphore(NvOdmAccelHandle hDevice)
{
    NvOdmGpioPinMode mode;
    NvOdmInterruptHandler callback =
        (NvOdmInterruptHandler)GpioInterruptHandler;

    hDevice->hGpioINT = (NvOdmServicesGpioHandle)NvOdmGpioOpen();
    if (!(hDevice->hGpioINT))
    {
        NVODMACCELEROMETER_PRINTF((
            "NvOdm Accelerometer : NvOdmGpioOpen Error \n"));
        return NV_FALSE;
    }

    hDevice->hPinINT = NvOdmGpioAcquirePinHandle(hDevice->hGpioINT,
                           hDevice->GPIOPortINT,
                           hDevice->GPIOPinINT);
    hDevice->SemaphoreForINT = NvOdmOsSemaphoreCreate(0);

    if (!(hDevice->SemaphoreForINT))
    {
        NVODMACCELEROMETER_PRINTF((
            "NvOdm Accelerometer : NvOdmOsSemaphoreCreate Error \n"));
        NvOdmGpioClose(hDevice->hGpioINT);
        return NV_FALSE;
    }

    mode = NvOdmGpioPinMode_InputInterruptHigh;
    if (NvOdmGpioInterruptRegister(hDevice->hGpioINT,
        &hDevice->hGpioInterrupt, hDevice->hPinINT, mode, callback,
        hDevice, NV_DEBOUNCE_TIME_MS) == NV_FALSE)
    {
        return NV_FALSE;
    }

    if (!(hDevice->hGpioInterrupt))
    {
        NVODMACCELEROMETER_PRINTF((
            "NvOdm Accelerometer : NvOdmGpioInterruptRegister Error \n"));
        NvOdmGpioClose(hDevice->hGpioINT);
        NvOdmOsSemaphoreDestroy(hDevice->SemaphoreForINT);
        return NV_FALSE;
    }
    return NV_TRUE;
}

static NvBool
WriteReg(
    NvOdmAccelHandle hDevice,
    NvU8 RegAddr,
    NvU8* value,
    NvU32 len)
{
    NvOdmI2cTransactionInfo TransactionInfo;

    if ( (NULL == hDevice) || (NULL == value) ||
         (len > I2C_ACCELRATOR_PACKET_SIZE-1 ) )
    {
        NVODMACCELEROMETER_PRINTF((
            "NvOdmI2c Set Regs Failed, max size is %d bytes\n",
            I2C_ACCELRATOR_PACKET_SIZE-1));
        return NV_FALSE;
    }

    s_WriteBuffer[0] = RegAddr;
    NvOdmOsMemcpy(&s_WriteBuffer[1], value, len);

    TransactionInfo.Address = hDevice->nDevAddr;
    TransactionInfo.Buf = s_WriteBuffer;
    TransactionInfo.Flags = NVODM_I2C_IS_WRITE;
    TransactionInfo.NumBytes = len+1;

    // Write the accelerator RegAddr (from where data is to be read).
    if (NvOdmI2cTransaction(hDevice->hOdmI2C, &TransactionInfo, 1, 400,
            I2C_ACCELRATOR_TRANSACTION_TIMEOUT) != NvOdmI2cStatus_Success)
        return NV_FALSE;

    return NV_TRUE;
}

static NvBool
ReadReg(
    NvOdmAccelHandle hDevice,
    NvU8 RegAddr,
    NvU8* value,
    NvU32 len)
{
    NvOdmI2cTransactionInfo TransactionInfo;

    if ( (NULL == hDevice) || (NULL == value) ||
         (len > I2C_ACCELRATOR_PACKET_SIZE-1 ) )
    {
        NVODMACCELEROMETER_PRINTF((
            "NvOdmI2c Get Regs Failed, max size is %d bytes\n",
            I2C_ACCELRATOR_PACKET_SIZE-1));
        return NV_FALSE;
    }

    s_WriteBuffer[0] = RegAddr;
    TransactionInfo.Address = hDevice->nDevAddr;
    TransactionInfo.Buf = s_WriteBuffer;
    TransactionInfo.Flags = NVODM_I2C_IS_WRITE;
    TransactionInfo.NumBytes = 1;

    // Write the accelerometor RegAddr (from where data is to be read).
    if (NvOdmI2cTransaction(hDevice->hOdmI2C, &TransactionInfo, 1, 400,
            I2C_ACCELRATOR_TRANSACTION_TIMEOUT) != NvOdmI2cStatus_Success)
        return NV_FALSE;

    s_ReadBuffer[0] = 0;
    TransactionInfo.Address = (hDevice->nDevAddr| 0x1);
    TransactionInfo.Buf = s_ReadBuffer;
    TransactionInfo.Flags = 0;
    TransactionInfo.NumBytes = len;

    //Read the data from the eeprom at the specified RegAddr
    if (NvOdmI2cTransaction(hDevice->hOdmI2C, &TransactionInfo, 1, 400,
            I2C_ACCELRATOR_TRANSACTION_TIMEOUT) != NvOdmI2cStatus_Success)
        return NV_FALSE;

    NvOdmOsMemcpy(value, &s_ReadBuffer[0], len);
    return NV_TRUE;
}

static NvBool BMA150_Init(NvOdmAccelHandle hAccel)
{
    NvU8 TestVal;

    ReadReg(hAccel, CHIP_ID_REG, &TestVal, 1);
    if (TestVal != BMA150_CHIP_ID)
    {
        NVODMACCELEROMETER_PRINTF(("Unknown BMA150 ID = 0x%x\n", TestVal));
        goto error;
    }
    NVODMACCELEROMETER_PRINTF(("BMA150 ID is 0x%x\n", TestVal));

    // Init Hw
    if (!ReadReg(hAccel, RANGE_BWIDTH_REG, &TestVal, 1))
        goto error;
    TestVal &= 0xE0;
    TestVal |= 0x04; //Set bandwidth to 375hz
    if (!WriteReg(hAccel, RANGE_BWIDTH_REG, &TestVal, 1))
        goto error;

    if (!ReadReg(hAccel, SMB150_CONF2_REG, &TestVal, 1))
        goto error;
    // Enable Advanced interrupt(6), latch int(4)
    TestVal |= (0 << 3) | (1 << 6) | (1 << 4);
    if (!WriteReg(hAccel, SMB150_CONF2_REG, &TestVal, 1))
        goto error;
    // Init Hw end
    // Set mode
    if (!ReadReg(hAccel, SMB150_CTRL_REG, &TestVal, 1))
        goto error;
    TestVal &= 0xFE;
    if (!WriteReg(hAccel, SMB150_CTRL_REG, &TestVal, 1))
        goto error;
    // Set mode end

    // Set motion thres
    if (!ReadReg(hAccel, MOTION_THRS_REG, &TestVal, 1))
        goto error;
    TestVal = 0x0A;
    if (!WriteReg(hAccel, MOTION_THRS_REG, &TestVal, 1))
        goto error;
    // Set motion thres end

    // Set any motion int
    if (!ReadReg(hAccel, SMB150_CONF1_REG, &TestVal, 1))
        goto error;
    TestVal &= 0xFC;
    TestVal |= (1 << 6) | (1 << 1) | (1 << 0);
    if (!WriteReg(hAccel, SMB150_CONF1_REG, &TestVal, 1))
        goto error;
    // Set any motion int end
    NVODMACCELEROMETER_PRINTF(("\n BMA150_Init passed"));
    return NV_TRUE;
error:
    NVODMACCELEROMETER_PRINTF(("\n BMA150_Init failed"));
    return NV_FALSE;
}

static NvBool
BMA150_ReadXYZ(
    NvOdmAccelHandle hDevice,
    NvS32* X,
    NvS32* Y,
    NvS32* Z)
{
    NvU8 Data[6];
    NvBool NewData = 0;

    if (!ReadReg(hDevice, X_AXIS_LSB_REG, &Data[0], 6))
        return NV_FALSE;
    NewData = ( (Data[0] & 0x1) || (Data[2] & 0x1) || (Data[4] & 0x1) ) ? 1 : 0;

    *X = ((Data[1] << 2) | (Data[0] >> 6));
    *Y = ((Data[3] << 2) | (Data[2] >> 6));
    *Z = ((Data[5] << 2) | (Data[4] >> 6));

    // Preserve sign bits.
    *X = *X << ((sizeof(*X)*8) - 10);
    *X = *X >> ((sizeof(*X)*8) - 10);
    *Y = *Y << ((sizeof(*Y)*8) - 10);
    *Y = *Y >> ((sizeof(*Y)*8) - 10);
    *Z = *Z << ((sizeof(*Z)*8) - 10);
    *Z = *Z >> ((sizeof(*Z)*8) - 10);
    return NewData;
}

static void BMA150_ResetInterrupt(NvOdmAccelHandle hDevice)
{
    NvU8 Data = (1 << 6);

    WriteReg(hDevice, SMB150_CTRL_REG, &Data, 1);
}

NvBool NvOdmAccelOpen(NvOdmAccelHandle* hDevice)
{
    NvU32 i;
    NvOdmAccelHandle  hAccel;
    NvOdmIoModule IoModule = NvOdmIoModule_I2c;
    const NvOdmPeripheralConnectivity *pConnectivity;
    NvBool FoundGpio = NV_FALSE, FoundI2cModule = NV_FALSE;

    hAccel = NvOdmOsAlloc(sizeof(NvOdmAccel));
    if (hAccel == NULL)
    {
        NVODMACCELEROMETER_PRINTF(("Error Allocating NvOdmAccel. \n"));
        return NV_FALSE;
    }
    NvOdmOsMemset(hAccel, 0, sizeof(NvOdmAccel));
    hAccel->nBusType = NV_ACCELEROMETER_BUS_I2C;

    // Info of accelerometer with current setting.
    hAccel->Caption.MaxForceInGs = 2000;
    hAccel->Caption.MaxTapTimeDeltaInUs = 255;
    hAccel->Caption.NumMotionThresholds = 1;
    hAccel->Caption.SupportsFreefallInt = 0;
    hAccel->Caption.MaxSampleRate = 100;
    hAccel->Caption.MinSampleRate = 3;
    hAccel->PowerState = NvOdmAccelPower_Fullrun;
    hAccel->AxisXMapping = NvOdmAccelAxis_Y;
    hAccel->AxisXDirection = -1;
    hAccel->AxisYMapping = NvOdmAccelAxis_X;
    hAccel->AxisYDirection = 1;
    hAccel->AxisZMapping = NvOdmAccelAxis_Z;
    hAccel->AxisZDirection = -1;

    hAccel->hPmu = NvOdmServicesPmuOpen();
    if (!hAccel->hPmu)
    {
        NVODMACCELEROMETER_PRINTF(("NvOdmServicesPmuOpen Error \n"));
        goto error;
    }

    pConnectivity = (NvOdmPeripheralConnectivity*)NvOdmPeripheralGetGuid(
                        NV_ODM_GUID('b','m','a','1','5','0','a','c'));
    if (!pConnectivity)
    {
        NvOdmOsDebugPrintf(("NvOdmPeripheralGetGuid doesn't detect\
            BMA150 accelerometer device\n"));
        goto error;
    }

    if (pConnectivity->Class != NvOdmPeripheralClass_Other)
        goto error;

    for( i = 0; i < pConnectivity->NumAddress; i++)
    {
        switch(pConnectivity->AddressList[i].Interface)
        {
            case NvOdmIoModule_I2c:
            case NvOdmIoModule_I2c_Pmu:
                hAccel->I2CChannelId = pConnectivity->AddressList[i].Instance;
                hAccel->nDevAddr = (NvU8)pConnectivity->AddressList[i].Address;
                FoundI2cModule = NV_TRUE;
                IoModule = pConnectivity->AddressList[i].Interface;
                break;
            case NvOdmIoModule_Gpio:
                hAccel->GPIOPortINT = pConnectivity->AddressList[i].Instance;
                hAccel->GPIOPinINT = pConnectivity->AddressList[i].Address;
                FoundGpio = NV_TRUE;
                break;
            case NvOdmIoModule_Vdd:
                hAccel->VddId = pConnectivity->AddressList[i].Address;
                // Power on accelerometer according to Vddid
                SetPowerRail(hAccel->hPmu, hAccel->VddId, NV_TRUE);
                break;
            default:
                break;
        }
    }

    if (!FoundGpio || !FoundI2cModule)
    {
        NVODMACCELEROMETER_PRINTF(("Accelerometer : didn't find any periperal\
            in discovery query for touch device Error \n"));
        goto error;
    }

    // Open I2C handle.
    hAccel->hOdmI2C = NvOdmI2cOpen(IoModule, hAccel->I2CChannelId);
    if (!hAccel->hOdmI2C)
        goto error;

    hAccel->RegsRead  = ReadReg;
    hAccel->RegsWrite = WriteReg;

    if (!BMA150_Init(hAccel))
        goto error;
    if (!ConnectSemaphore(hAccel))
        goto error;

    *hDevice = hAccel;
    return NV_TRUE;
    error:
        NVODMACCELEROMETER_PRINTF(("Error during BMA150 NvOdmAccelOpen\n"));
        // Release all of resources requested.
        if (hAccel)
        {
            SetPowerRail(hAccel->hPmu, hAccel->VddId, NV_FALSE);
            NvOdmServicesPmuClose(hAccel->hPmu);
            hAccel->hPmu = NULL;
            NvOdmI2cClose(hAccel->hOdmI2C);
            hAccel->hOdmI2C = NULL;
            NvOdmOsFree(hAccel);
            *hDevice = NULL;
        }
        return NV_FALSE;
}

void NvOdmAccelClose(NvOdmAccelHandle hDevice)
{
    if (hDevice)
    {
        if (hDevice->SemaphoreForINT && hDevice->hGpioINT &&
            hDevice->hPinINT && hDevice->hGpioInterrupt)
        {
            NvOdmGpioInterruptUnregister(hDevice->hGpioINT,
                hDevice->hPinINT, hDevice->hGpioInterrupt);
            NvOdmOsSemaphoreDestroy(hDevice->SemaphoreForINT);
            NvOdmGpioReleasePinHandle(hDevice->hGpioINT, hDevice->hPinINT);
            NvOdmGpioClose(hDevice->hGpioINT);
        }
        NvOdmI2cClose(hDevice->hOdmI2C);

        // Power off accelermeter
        SetPowerRail(hDevice->hPmu, hDevice->VddId, NV_FALSE);
        if (hDevice->hPmu)
        {
            //NvAccelerometerSetPowerOn(0);
            NvOdmServicesPmuClose(hDevice->hPmu);
        }
    }
}

NvBool
NvOdmAccelSetIntForceThreshold(
    NvOdmAccelHandle  hDevice,
    NvOdmAccelIntType IntType,
    NvU32             IntNum,
    NvU32             Threshold)
{
    return NV_TRUE;
}

NvBool
NvOdmAccelSetIntTimeThreshold(
    NvOdmAccelHandle  hDevice,
    NvOdmAccelIntType IntType,
    NvU32             IntNum,
    NvU32             Threshold)
{
    return NV_TRUE;
}

NvBool
NvOdmAccelSetIntEnable(
    NvOdmAccelHandle  hDevice,
    NvOdmAccelIntType  IntType,
    NvOdmAccelAxisType IntAxis,
    NvU32              IntNum,
    NvBool             Toggle)
{
    return NV_TRUE;
}

void
NvOdmAccelWaitInt(
    NvOdmAccelHandle    hDevice,
    NvOdmAccelIntType  *IntType,
    NvOdmAccelAxisType *IntMotionAxis,
    NvOdmAccelAxisType *IntTapAxis)
{
    NV_ASSERT(hDevice);
    NV_ASSERT(IntType);
    NV_ASSERT(IntMotionAxis);
    NV_ASSERT(IntTapAxis);

    if ((g_WaitCounter > 0) || ENABLE_XYZ_POLLING)
    {
        NvOdmOsSemaphoreWaitTimeout( hDevice->SemaphoreForINT, 300);
        g_WaitCounter--;
    }
    else
        NvOdmOsSemaphoreWait( hDevice->SemaphoreForINT);
}

void NvOdmAccelSignal(NvOdmAccelHandle hDevice)
{
    NvOdmOsSemaphoreSignal(hDevice->SemaphoreForINT);
}

NvBool
NvOdmAccelGetAcceleration(
    NvOdmAccelHandle hDevice,
    NvS32           *AccelX,
    NvS32           *AccelY,
    NvS32           *AccelZ)
{
    NvS32 data;
    NvBool NewData = 0;
    NvS32 TempAccelX = 0;
    NvS32 TempAccelY = 0;
    NvS32 TempAccelZ = 0;

    NV_ASSERT(NULL != hDevice);
    NV_ASSERT(NULL != AccelX);
    NV_ASSERT(NULL != AccelY);
    NV_ASSERT(NULL != AccelZ);
    NewData = BMA150_ReadXYZ(hDevice, &TempAccelX, &TempAccelY, &TempAccelZ);

    data = (((NvS32)(hDevice->Caption.MaxForceInGs))*TempAccelX+(NvS32)(NV_BMA150_MAX_FORCE_IN_REG/2))/
             (NvS32)NV_BMA150_MAX_FORCE_IN_REG;
    switch(hDevice->AxisXMapping)
    {
        case NvOdmAccelAxis_X:
            *AccelX = data*hDevice->AxisXDirection;
            break;
        case NvOdmAccelAxis_Y:
            *AccelY = data*hDevice->AxisYDirection;
            break;
        case NvOdmAccelAxis_Z:
            *AccelZ = data*hDevice->AxisZDirection;
            break;
        default:
            return NV_FALSE;
    }

    data = (((NvS32)(hDevice->Caption.MaxForceInGs))*TempAccelY+(NvS32)(NV_BMA150_MAX_FORCE_IN_REG/2))/
             (NvS32)NV_BMA150_MAX_FORCE_IN_REG;
    switch(hDevice->AxisYMapping)
    {
        case NvOdmAccelAxis_X:
            *AccelX = data*hDevice->AxisXDirection;
            break;
        case NvOdmAccelAxis_Y:
            *AccelY = data*hDevice->AxisYDirection;
            break;
        case NvOdmAccelAxis_Z:
            *AccelZ = data*hDevice->AxisZDirection;
            break;
        default:
            return NV_FALSE;
    }

    data = (((NvS32)(hDevice->Caption.MaxForceInGs))*TempAccelZ+(NvS32)(NV_BMA150_MAX_FORCE_IN_REG/2))/
             (NvS32)NV_BMA150_MAX_FORCE_IN_REG;
    switch(hDevice->AxisZMapping)
    {
        case NvOdmAccelAxis_X:
            *AccelX = data*hDevice->AxisXDirection;
            break;
        case NvOdmAccelAxis_Y:
            *AccelY = data*hDevice->AxisYDirection;
            break;
        case NvOdmAccelAxis_Z:
            *AccelZ = data*hDevice->AxisZDirection;
            break;
        default:
            return NV_FALSE;
    }

    NVODMACCELEROMETER_PRINTF(("\naccel output, x=%d,y=%d,z=%d, NewData=%d",
        *AccelX, *AccelY, *AccelZ, NewData));
    return NewData;
}

NvOdmAccelerometerCaps NvOdmAccelGetCaps(NvOdmAccelHandle hDevice)
{
    NV_ASSERT(NULL != hDevice);
    return hDevice->Caption;
}

NvBool NvOdmAccelSetSampleRate(NvOdmAccelHandle hDevice, NvU32 SampleRate)
{
    return NV_TRUE;
}

NvBool NvOdmAccelGetSampleRate(NvOdmAccelHandle hDevice, NvU32 *pSampleRate)
{
    return NV_TRUE;
}

NvBool
NvOdmAccelSetPowerState(
    NvOdmAccelHandle hDevice,
    NvOdmAccelPowerType PowerState)
{
    return NV_TRUE;
}

