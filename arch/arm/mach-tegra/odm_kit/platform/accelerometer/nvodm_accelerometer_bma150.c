/*
 * Copyright (c) 2010 NVIDIA Corporation.
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

#include "nvodm_priv_accelerometer.h"

/* BMA150 register address */
#define CHIP_ID_REG             0x00
#define VERSION_REG             0x01
#define X_AXIS_LSB_REG          0x02
#define X_AXIS_MSB_REG          0x03
#define Y_AXIS_LSB_REG          0x04
#define Y_AXIS_MSB_REG          0x05
#define Z_AXIS_LSB_REG          0x06
#define Z_AXIS_MSB_REG          0x07
#define TEMP_RD_REG             0x08
#define SMB150_STATUS_REG       0x09
#define SMB150_CTRL_REG         0x0a
#define SMB150_CONF1_REG        0x0b
#define LG_THRESHOLD_REG        0x0c
#define LG_DURATION_REG         0x0d
#define HG_THRESHOLD_REG        0x0e
#define HG_DURATION_REG         0x0f
#define MOTION_THRS_REG         0x10
#define HYSTERESIS_REG          0x11
#define CUSTOMER1_REG           0x12
#define CUSTOMER2_REG           0x13
#define RANGE_BWIDTH_REG        0x14
#define SMB150_CONF2_REG        0x15

#define OFFS_GAIN_X_REG         0x16
#define OFFS_GAIN_Y_REG         0x17
#define OFFS_GAIN_Z_REG         0x18
#define OFFS_GAIN_T_REG         0x19
#define OFFSET_X_REG            0x1a
#define OFFSET_Y_REG            0x1b
#define OFFSET_Z_REG            0x1c
#define OFFSET_T_REG            0x1d
/* BMA150 register address ends here*/

/* range and bandwidth */
#define BMA_RANGE_2G            0
#define BMA_RANGE_4G            1
#define BMA_RANGE_8G            2

#define BMA_BW_25HZ             0
#define BMA_BW_50HZ             1
#define BMA_BW_100HZ            2
#define BMA_BW_190HZ            3
#define BMA_BW_375HZ            4
#define BMA_BW_750HZ            5
#define BMA_BW_1500HZ           6

/* mode settings */
#define BMA_MODE_NORMAL         0
#define BMA_MODE_SLEEP          1

#define BMA150_CHIP_ID      0x02        // RO - device identification

#define INT_EVENT_TIMEOUT 100
#define NV_ACCELEROMETER_BUS_I2C 0
#define NV_ACCELEROMETER_BUS_SPI_3 1
#define NV_ACCELEROMETER_BUS_SPI_4 2

#define EEPROM_ID_E1206 0x0C06

#define NV_BMA150_MAX_FORCE_IN_REG 512 // It indicates force register length.
#define NV_DEBOUNCE_TIME_MS 0

#define ENABLE_XYZ_POLLING 1
#define NV_BMA150_MAX_SAMPLE_RATE   12000 //Hz
#define NV_BMA150_MIN_SAMPLE_RATE   50 //Hz
// sw polling time is slower than hw sample rate 225 time
#define NV_BMA150_POLLING_FACTOR    225
static NvU32 CurrSampleRate = 2*375; // Current Sample Rate
static NvU32 PollingTime = 300; // (1000 * 225)/2*375(ms)

typedef struct BmaSampleRateRec
{
    // Range register value
    NvU8 RangeReg;
    // Bandwidth register value
    NvU8 BandWidthReg;
    // SampleRate(Hz) = Full scale acceleration range * BandWidth(in Hz)
    NvU32 SampleRate;
} BmaSampleRate;

BmaSampleRate OutputRate[] = {
    {0, 0, 2*25},
    {0, 1, 2*50},
    {0, 2, 2*100},
    {0, 3, 2*190},
    {1, 2, 4*100},
    {0, 4, 2*375},
    {1, 3, 4*190},
    {2, 2, 8*100},
    {0, 5, 2*750},
    {2, 3, 8*190},
    {0, 6, 2*1500},
    {1, 6, 4*1500},
    {2, 6, 8*1500}
};

//FIXME: protect this variable using spinlock.
static volatile int g_WaitCounter = 0;

static NvU8 s_ReadBuffer[I2C_ACCELRATOR_PACKET_SIZE];
static NvU8 s_WriteBuffer[I2C_ACCELRATOR_PACKET_SIZE];

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

static void BMA150_ResetInterrupt(NvOdmAccelHandle hDevice)
{
    NvU8 Data = (1 << 6);

    WriteReg(hDevice, SMB150_CTRL_REG, &Data, 1);
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

#if !(ENABLE_XYZ_POLLING)
    if (!ReadReg(hAccel, SMB150_CONF2_REG, &TestVal, 1))
        goto error;
    // Enable Advanced interrupt(6), latch int(4)
    TestVal |= (0 << 3) | (1 << 6) | (1 << 4);
    if (!WriteReg(hAccel, SMB150_CONF2_REG, &TestVal, 1))
        goto error;
#endif

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

void BMA150_deInit(NvOdmAccelHandle hDevice)
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
BMA150_SetIntForceThreshold(
    NvOdmAccelHandle  hDevice,
    NvOdmAccelIntType IntType,
    NvU32             IntNum,
    NvU32             Threshold)
{
    return NV_TRUE;
}

NvBool
BMA150_SetIntTimeThreshold(
    NvOdmAccelHandle  hDevice,
    NvOdmAccelIntType IntType,
    NvU32             IntNum,
    NvU32             Threshold)
{
    return NV_TRUE;
}

NvBool
BMA150_SetIntEnable(
    NvOdmAccelHandle  hDevice,
    NvOdmAccelIntType  IntType,
    NvOdmAccelAxisType IntAxis,
    NvU32              IntNum,
    NvBool             Toggle)
{
    return NV_TRUE;
}

void
BMA150_WaitInt(
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
        NvOdmOsSemaphoreWaitTimeout( hDevice->SemaphoreForINT, PollingTime);
        g_WaitCounter--;
    }
    else
        NvOdmOsSemaphoreWait( hDevice->SemaphoreForINT);
}

void BMA150_Signal(NvOdmAccelHandle hDevice)
{
    NvOdmOsSemaphoreSignal(hDevice->SemaphoreForINT);
}

NvBool
BMA150_GetAcceleration(
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

    data = (((NvS32)(hDevice->Caption.MaxForceInGs))*TempAccelX+
            (NvS32)(NV_BMA150_MAX_FORCE_IN_REG/2))/
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

    data = (((NvS32)(hDevice->Caption.MaxForceInGs))*TempAccelY+
            (NvS32)(NV_BMA150_MAX_FORCE_IN_REG/2))/
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

    data = (((NvS32)(hDevice->Caption.MaxForceInGs))*TempAccelZ+
            (NvS32)(NV_BMA150_MAX_FORCE_IN_REG/2))/
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

NvOdmAccelerometerCaps BMA150_GetCaps(NvOdmAccelHandle hDevice)
{
    NV_ASSERT(NULL != hDevice);
    return hDevice->Caption;
}

NvBool BMA150_SetSampleRate(NvOdmAccelHandle hDevice, NvU32 SampleRate)
{
    NvU8 BandWidthReg, RangeReg, Val;
    NvU32 i;
    NvS32 index;

    if (!ReadReg(hDevice, RANGE_BWIDTH_REG, &Val, 1))
        goto error;

    index = -1;
    if (SampleRate <= NV_BMA150_MIN_SAMPLE_RATE)
    {
        index = 0;
    }
    else if (SampleRate >= NV_BMA150_MAX_SAMPLE_RATE)
    {
        index = NV_ARRAY_SIZE(OutputRate)-1;
    }
    else
    {
        for (i = 0; i < NV_ARRAY_SIZE(OutputRate); i++)
        {
            if ((SampleRate >= OutputRate[i].SampleRate) &&
                (SampleRate < OutputRate[i+1].SampleRate))
            {
                index = i;
                break;
            }
        }
    }

    if (index != -1)
    {
        BandWidthReg = OutputRate[index].BandWidthReg;
        RangeReg = OutputRate[index].RangeReg;
        Val = Val & 0xE0;
        Val = Val | BandWidthReg | (RangeReg << 3);
        if (!WriteReg(hDevice, RANGE_BWIDTH_REG, &Val, 1))
            goto error;
        CurrSampleRate = OutputRate[index].SampleRate;
        PollingTime = (1000 * NV_BMA150_POLLING_FACTOR)/CurrSampleRate; //ms
        return NV_TRUE;
    }
error:
    return NV_FALSE;
}

NvBool BMA150_GetSampleRate(NvOdmAccelHandle hDevice, NvU32 *pSampleRate)
{
    *pSampleRate = CurrSampleRate;
    return NV_TRUE;
}

NvBool
BMA150_SetPowerState(
    NvOdmAccelHandle hDevice,
    NvOdmAccelPowerType PowerState)
{
    return NV_TRUE;
}

NvBool bma150_init(NvOdmAccelHandle* hDevice)
{
    NvU32 i;
    NvOdmAccelHandle  hAccel;
    NvOdmIoModule IoModule = NvOdmIoModule_I2c;
    const NvOdmPeripheralConnectivity *pConnectivity;
    NvBool FoundGpio = NV_FALSE, FoundI2cModule = NV_FALSE;
    NvOdmBoardInfo BoardInfo;
    // Accelerometer is supported only on E1206.
    if (!NvOdmPeripheralGetBoardInfo(EEPROM_ID_E1206, &BoardInfo))
    {
        NVODMACCELEROMETER_PRINTF(("\n Accelerometer is not supported \n"));
        return NV_FALSE;
    }

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
#if AXES_MAPPING_FOR_PROPER_DISPLAY_ALIGNMENT
    hAccel->AxisXMapping = NvOdmAccelAxis_Y;
    hAccel->AxisXDirection = -1;
    hAccel->AxisYMapping = NvOdmAccelAxis_X;
#else
    hAccel->AxisXMapping = NvOdmAccelAxis_X;
    hAccel->AxisXDirection = 1;
    hAccel->AxisYMapping = NvOdmAccelAxis_Y;
#endif
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

    hAccel->AccelClose = BMA150_deInit;
    hAccel->AccelSetIntForceThreshold = BMA150_SetIntForceThreshold;
    hAccel->AccelSetIntTimeThreshold = BMA150_SetIntTimeThreshold;
    hAccel->AccelSetIntEnable = BMA150_SetIntEnable;
    hAccel->AccelWaitInt = BMA150_WaitInt;
    hAccel->AccelSignal = BMA150_Signal;
    hAccel->AccelGetAcceleration = BMA150_GetAcceleration;
    hAccel->AccelGetCaps = BMA150_GetCaps;
    hAccel->AccelSetPowerState = BMA150_SetPowerState;
    hAccel->AccelSetSampleRate = BMA150_SetSampleRate;
    hAccel->AccelGetSampleRate = BMA150_GetSampleRate;

    *hDevice = hAccel;
    NVODMACCELEROMETER_PRINTF(("BMA150 NvOdmAccelOpen success\n"));
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

