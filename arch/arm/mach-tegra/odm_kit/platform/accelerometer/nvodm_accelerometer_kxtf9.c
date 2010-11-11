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

/* kxtf9 register address */
#define kxtf9_XOUT_HPF_LSB           0x00
#define kxtf9_XOUT_HPF_MSB           0x01
#define kxtf9_YOUT_HPF_LSB           0x02
#define kxtf9_YOUT_HPF_MSB           0x03
#define kxtf9_ZOUT_HPF_LSB           0x04
#define kxtf9_ZOUT_HPF_MSB           0x05

#define kxtf9_X_AXIS_LSB           0x06
#define kxtf9_X_AXIS_MSB           0x07
#define kxtf9_Y_AXIS_LSB           0x08
#define kxtf9_Y_AXIS_MSB           0x09
#define kxtf9_Z_AXIS_LSB           0x0A
#define kxtf9_Z_AXIS_MSB           0x0B

#define kxtf9_DCSTRESP              0x0C
// WHO_AM_I reg
#define kxtf9_CHIP_ID              0x0F

#define kxtf9_TILT_CUR             0x10
#define kxtf9_TILT_PRE             0x11

#define kxtf9_INT_SRC_REG1            0x15
#define kxtf9_INT_SRC_REG2            0x16

#define kxtf9_STATUS         0x18
#define kxtf9_INT_REL            0x1A
#define kxtf9_CTRL_REG1            0x1B
#define kxtf9_CTRL_REG2            0x1C
#define kxtf9_CTRL_REG3            0x1D

#define kxtf9_INT_CTRL_REG1            0x1E
#define kxtf9_INT_CTRL_REG2            0x1F
#define kxtf9_INT_CTRL_REG3            0x20

#define kxtf9_DATA_CTRL             0x21

#define kxtf9_TILT_TIMER             0x28
#define kxtf9_WUF_TIMER             0x29
#define kxtf9_TDT_TIMER             0x2B
#define kxtf9_TDT_H_THRESH         0x2C
#define kxtf9_TDT_L_THRESH         0x2D

#define kxtf9_TDT_TAP_TIMER            0x2E
#define kxtf9_TDT_TOTAL_TIMER        0x2F
#define kxtf9_TDT_LATENCY_TIMER    0x30
#define kxtf9_TDT_WINDOW_TIMER    0x31
#define kxtf9_SELF_TEST                   0x3A

#define kxtf9_WUF_THRESH         0x5A
#define kxtf9_TILT_ANGLE         0x5C
#define kxtf9_HYST_SET             0x5F
/* kxtf9 register address info ends here*/

#define kxtf9_CHIP_ID_VAL      0x01        // POR value- device identification

// set following macros based on requirement
#define TILT_ONLY_ENABLE 1
#define TAP_ONLY_ENABLE 1
#define MOTION_ONLY_ENABLE 1

// I2C clock speed.
#define I2C_CLK_SPEED 400

#define ACCEL_VER0_GUID NV_ODM_GUID('k','x','t','9','-','0','0','0')
#define ACCEL_VER1_GUID NV_ODM_GUID('k','x','t','9','-','0','9','0')

#define NV_DEBOUNCE_TIME_MS 0
#define ENABLE_XYZ_POLLING 0

static NvU8 s_ReadBuffer[I2C_ACCELRATOR_PACKET_SIZE];
static NvU8 s_WriteBuffer[I2C_ACCELRATOR_PACKET_SIZE];
static NvU8 s_IntSrcReg1;
static NvU8 s_IntSrcReg2;

typedef struct discovery_entry_rec
{
   NvU64 guid;
   NvU32 axesmapping;
} discovery_entry;

static discovery_entry disc_entry[] = {
    {ACCEL_VER0_GUID, 1},
    {ACCEL_VER1_GUID, 2},
};


static void
ConfigPowerRail(
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
    if (settletime)
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
    if (NvOdmI2cTransaction(hDevice->hOdmI2C, &TransactionInfo, 1, I2C_CLK_SPEED,
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
    if (NvOdmI2cTransaction(hDevice->hOdmI2C, &TransactionInfo, 1, I2C_CLK_SPEED,
            I2C_ACCELRATOR_TRANSACTION_TIMEOUT) != NvOdmI2cStatus_Success)
        return NV_FALSE;

    s_ReadBuffer[0] = 0;
    TransactionInfo.Address = (hDevice->nDevAddr| 0x1);
    TransactionInfo.Buf = s_ReadBuffer;
    TransactionInfo.Flags = 0;
    TransactionInfo.NumBytes = len;

    //Read the data from the eeprom at the specified RegAddr
    if (NvOdmI2cTransaction(hDevice->hOdmI2C, &TransactionInfo, 1, I2C_CLK_SPEED,
            I2C_ACCELRATOR_TRANSACTION_TIMEOUT) != NvOdmI2cStatus_Success)
        return NV_FALSE;

    NvOdmOsMemcpy(value, &s_ReadBuffer[0], len);
    return NV_TRUE;
}

static NvBool WRITE_VERIFY(NvOdmAccelHandle hDevice, NvU8 Reg,NvU8 Val)
{
    NvU8 TestVal = 0;
    NvBool ret = NV_TRUE;
    if (!WriteReg(hDevice, Reg, &Val, 1))
    {
        NVODMACCELEROMETER_PRINTF(("\n write to Reg[0x%x] val [0x%x]failed",
            Reg, Val));
        ret = NV_FALSE;
    }
    if (!ReadReg(hDevice, Reg, &Val, 1))
    {
        NVODMACCELEROMETER_PRINTF(("\n read after write to Reg[0x%x] val \
         [0x%x]failed", Reg, Val));
        ret = NV_FALSE;
    }
    if (TestVal != Val)
    {
        NVODMACCELEROMETER_PRINTF(("\n read validation fail: Reg[0x%x] val \
         [0x%x]failed", Reg, Val));
    }
    return ret;
}

static void kxtf9_ResetInterrupt(NvOdmAccelHandle hDevice)
{
    NvU8 Data[2];
    if (!ReadReg(hDevice, kxtf9_INT_SRC_REG1, Data, 2))
        return; // NV_FALSE;
    s_IntSrcReg1  = Data[0];
    s_IntSrcReg2  = Data[1];
    // To clear the interrupts.
    ReadReg(hDevice, kxtf9_INT_REL, Data, 1);
}

#define SET_INT_RAISING_EDGE 0
static void GpioInterruptHandler(void *arg)
{
    NvU32 pinValue;
    NvOdmAccelHandle hDevice =  (NvOdmAccelHandle)arg;

    NvOdmGpioGetState(hDevice->hGpioINT, hDevice->hPinINT, &pinValue);
#if SET_INT_RAISING_EDGE
    if (pinValue == 1)
#else
    if (pinValue == 0)
#endif
    {
        kxtf9_ResetInterrupt(hDevice);

        NvOdmOsSemaphoreSignal(hDevice->SemaphoreForINT);
    }
    NvOdmGpioInterruptDone(hDevice->hGpioInterrupt);
    return;
}

static NvBool ConfigInterrupt(NvOdmAccelHandle hDevice)
{
    NvOdmGpioPinMode mode;
    NvOdmInterruptHandler callback =
        (NvOdmInterruptHandler)GpioInterruptHandler;

    hDevice->hGpioINT = (NvOdmServicesGpioHandle)NvOdmGpioOpen();
    if (!(hDevice->hGpioINT))
    {
        NVODMACCELEROMETER_PRINTF(("kxtf9: GpioOpen Error \n"));
        return NV_FALSE;
    }
    hDevice->hPinINT = NULL;
    hDevice->hPinINT = NvOdmGpioAcquirePinHandle(hDevice->hGpioINT,
                           hDevice->GPIOPortINT,
                           hDevice->GPIOPinINT);
    if (!hDevice->hPinINT)
    {
        NVODMACCELEROMETER_PRINTF(("kxtf9: GpioAcquirePinHandle Error\n"));
        goto CloseGpioHandle;
    }
    hDevice->SemaphoreForINT = NvOdmOsSemaphoreCreate(0);

    if (!(hDevice->SemaphoreForINT))
    {
        NVODMACCELEROMETER_PRINTF(("kxtf9: OsSemaphoreCreate Error \n"));
        goto ReleasePinHandle;
    }

    NvOdmGpioConfig(hDevice->hGpioINT, hDevice->hPinINT, NvOdmGpioPinMode_InputData);

#if SET_INT_RAISING_EDGE
    mode = NvOdmGpioPinMode_InputInterruptHigh;
#else
    mode = NvOdmGpioPinMode_InputInterruptLow;
#endif
    if (NvOdmGpioInterruptRegister(hDevice->hGpioINT,
        &hDevice->hGpioInterrupt, hDevice->hPinINT, mode, callback,
        hDevice, NV_DEBOUNCE_TIME_MS) == NV_FALSE)
    {
        NVODMACCELEROMETER_PRINTF(("kxtf9: GpioInterruptRegister failure\n"));
        goto SemDestroy;
    }

    if (!(hDevice->hGpioInterrupt))
    {
        NVODMACCELEROMETER_PRINTF(("kxtf9: Invalid interrupt handle\n"));
        goto SemDestroy;
    }
    return NV_TRUE;
SemDestroy:
    NvOdmOsSemaphoreDestroy(hDevice->SemaphoreForINT);
ReleasePinHandle:
    NvOdmGpioReleasePinHandle(hDevice->hGpioINT, hDevice->hPinINT);
CloseGpioHandle:
    NvOdmGpioClose(hDevice->hGpioINT);
    return NV_FALSE;
}

// For 12 bit resolution, Max reading possible with msb left for sign bit.
static NvS32 s_MaxAccelReading = 1 << 11;
// Considering g variation to be +/-2g, Max g value in milli g's.
static NvS32 s_MaxGValInMilliGs = 2000;
static NvBool kxtf9_Init(NvOdmAccelHandle hDevice)
{
    NvU8 TestVal;
    NvU8 RegVal = 0;

    // wait for 110 msec.
    NvOdmOsSleepMS(110);
    NVODMACCELEROMETER_PRINTF(("\n kxtf9 I2C addr: 0x%x",hDevice->nDevAddr));

    ReadReg(hDevice, kxtf9_CHIP_ID, &TestVal, 1);
    if (TestVal != kxtf9_CHIP_ID_VAL)
    {
        NVODMACCELEROMETER_PRINTF(("\n Unknown kxtf9 ID = 0x%x\n", TestVal));
        goto error;
    }
    else
    {
        NVODMACCELEROMETER_PRINTF(("\n kxtf9 ID is 0x%x\n", TestVal));
    }

#if SET_INT_RAISING_EDGE
    // set interrupt bits - IEN, IEA & IEL only active
    RegVal = 0x30;
#else
    RegVal = 0x20;
#endif
    if (!WriteReg(hDevice, kxtf9_INT_CTRL_REG1, &RegVal, 1))
    {
        NVODMACCELEROMETER_PRINTF(("\n Int Set failed\n"));
    }

    // set ODR to 25 Hz
    RegVal = 0x0;
    if (!WriteReg(hDevice, kxtf9_CTRL_REG3, &RegVal, 1))
    {
        NVODMACCELEROMETER_PRINTF(("\n LPF freq set failed\n"));
    }
    // Set WUF_TIMER to 2 ODR clock
    RegVal = 2;
    if (!WriteReg(hDevice, kxtf9_WUF_TIMER, &RegVal, 1))
    {
        NVODMACCELEROMETER_PRINTF(("\n LPF freq set failed\n"));
    }

    RegVal = 0;
#if TILT_ONLY_ENABLE
    //Activate Tilt Position Function
    WRITE_VERIFY(hDevice, kxtf9_TILT_TIMER, 0x01);
    RegVal = 0x81;
#endif

#if TAP_ONLY_ENABLE
    // Activate Tap/Double Tap Function
    RegVal |= 0x84;
#endif

#if MOTION_ONLY_ENABLE
    // For getting continuous data from accelerometer, set DRDY = 1.
    //set PC1, RES & WUFE to 1.
    RegVal |= 0XC2;
#endif
    if(!WRITE_VERIFY(hDevice, kxtf9_CTRL_REG1, RegVal))
        goto error;

    NVODMACCELEROMETER_PRINTF(("\n kxtf9_Init passed"));
    return NV_TRUE;
error:
    NVODMACCELEROMETER_PRINTF(("\n kxtf9_Init failed"));
    return NV_FALSE;
}

static void
kxtf9_ReadXYZ(
    NvOdmAccelHandle hDevice,
    NvS32* X,
    NvS32* Y,
    NvS32* Z)
{
    NvU8 Data[6];
    if (!ReadReg(hDevice, kxtf9_X_AXIS_LSB, Data, 6))
        return;

    *X = ((Data[1] << 4) | (Data[0] >> 4));
    *Y = ((Data[3] << 4) | (Data[2] >> 4));
    *Z = ((Data[5] << 4) | (Data[4] >> 4));

    if (*X & 0x800)
        *X |= 0xFFFFF000;
    if (*Y & 0x800)
        *Y |= 0xFFFFF000;
    if (*Z & 0x800)
        *Z |= 0xFFFFF000;
}

void kxtf9_deInit(NvOdmAccelHandle hDevice)
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
        NVODMACCELEROMETER_PRINTF(("\n I2C handle: 0x%x slave addr1: 0x%x",
            hDevice->hOdmI2C, hDevice->nDevAddr));
        // Power off accelermeter
        ConfigPowerRail(hDevice->hPmu, hDevice->VddId, NV_FALSE);
        if (hDevice->hPmu)
        {
            //NvAccelerometerSetPowerOn(0);
            NvOdmServicesPmuClose(hDevice->hPmu);
        }
    }
}

static NvBool SetAccelerometerActive(NvOdmAccelHandle hDevice, NvBool State)
{
    NvU8 RegVal;

    // to clear PC1 of CTRL_REG1
    if (!ReadReg(hDevice, kxtf9_CTRL_REG1, &RegVal, 1))
    {
        NVODMACCELEROMETER_PRINTF(("\n %s-> %d failed\n", __func__,__LINE__));
        return NV_FALSE;
    }

    if (State)
        RegVal |= 0x80;
    else
        RegVal &= 0x7F;

    if(!WRITE_VERIFY(hDevice, kxtf9_CTRL_REG1, RegVal))
    {
        NVODMACCELEROMETER_PRINTF(("\n %s-> %d failed\n", __func__,__LINE__));
        return NV_FALSE;
    }
    return NV_TRUE;
}
NvBool
kxtf9_SetIntForceThreshold(
    NvOdmAccelHandle  hDevice,
    NvOdmAccelIntType IntType,
    NvU32             IntNum,
    NvU32             Threshold)
{
    NvU8 RegVal;
    NvU32 Reading;

    // If not motion Threshold, return from here
    if (IntType != NvOdmAccelInt_MotionThreshold)
        return NV_TRUE;

    if(!SetAccelerometerActive(hDevice, NV_FALSE))
        return NV_FALSE;

    Reading = (Threshold * s_MaxAccelReading)/ (s_MaxGValInMilliGs);
    RegVal = (NvU8) Reading;
    // To set requested Threshold Value.
    if(!WRITE_VERIFY(hDevice, kxtf9_WUF_THRESH, RegVal))
    {
        NvOdmOsPrintf("\nkxtf9:Set WUF_Threshold failed\n");
        return NV_FALSE;
    }

    if(!SetAccelerometerActive(hDevice, NV_TRUE))
        return NV_FALSE;
    NvOdmOsPrintf("\nkxtf9: set WUF_Threshold val:%d",Threshold);
    return NV_TRUE;
}

NvBool
kxtf9_SetIntTimeThreshold(
    NvOdmAccelHandle  hDevice,
    NvOdmAccelIntType IntType,
    NvU32             IntNum,
    NvU32             Threshold)
{
    return NV_TRUE;
}

NvBool
kxtf9_SetIntEnable(
    NvOdmAccelHandle  hDevice,
    NvOdmAccelIntType  IntType,
    NvOdmAccelAxisType IntAxis,
    NvU32              IntNum,
    NvBool             Toggle)
{
    return NV_TRUE;
}

void
kxtf9_WaitInt(
    NvOdmAccelHandle    hDevice,
    NvOdmAccelIntType  *IntType,
    NvOdmAccelAxisType *IntMotionAxis,
    NvOdmAccelAxisType *IntTapAxis)
{
    NV_ASSERT(hDevice);
    NV_ASSERT(IntType);
    NV_ASSERT(IntMotionAxis);
    NV_ASSERT(IntTapAxis);
    NvOdmOsSemaphoreWait( hDevice->SemaphoreForINT);
}

void kxtf9_Signal(NvOdmAccelHandle hDevice)
{
    NvOdmOsSemaphoreSignal(hDevice->SemaphoreForINT);
}

#if NVODMACCELEROMETER_ENABLE_PRINTF
static void PrintAccelEventInfo(NvOdmAccelHandle hDevice)
{
    NvU8 RegVal = 0;
    if (s_IntSrcReg2 & 1)
    {
        NVODMACCELEROMETER_PRINTF(("\nkxtf9 Tilt Int set"));
        ReadReg(hDevice, kxtf9_TILT_CUR, &RegVal, 1);

        if (s_IntSrcReg1 & 0x1)
        {
            NVODMACCELEROMETER_PRINTF(("\nkxtf9 Tilt: Face Up state"));
        }
        if (s_IntSrcReg1 & 0x2)
        {
            NVODMACCELEROMETER_PRINTF(("\nkxtf9 Tilt: Face Down state"));
        }
        if (s_IntSrcReg1 & 0x4)
        {
            NVODMACCELEROMETER_PRINTF(("\nkxtf9 Tilt: Up state"));
        }
        if (s_IntSrcReg1 & 0x8)
        {
            NVODMACCELEROMETER_PRINTF(("\nkxtf9 Tilt: Down state"));
        }
        if (s_IntSrcReg1 & 0x10)
        {
            NVODMACCELEROMETER_PRINTF(("\nkxtf9 Tilt: Right state"));
        }
        if (s_IntSrcReg1 & 0x20)
        {
            NVODMACCELEROMETER_PRINTF(("\nkxtf9 Tilt: Left state"));
        }
    }

    if (((s_IntSrcReg2 >> 2) & 3) == 1)
    {
        NVODMACCELEROMETER_PRINTF(("\nkxtf9 single Tap Int set"));
    }
    if (((s_IntSrcReg2 >> 2) & 3) == 2)
    {
        NVODMACCELEROMETER_PRINTF(("\nkxtf9 double Tap Int set"));
    }
    if ((s_IntSrcReg2 >> 1) & 1)
    {
        NVODMACCELEROMETER_PRINTF(("\n kxtf9 Motion event (WUFS) Int set"));
    }
    if ((s_IntSrcReg2 >> 4) & 1)
    {
        NVODMACCELEROMETER_PRINTF(("\n kxtf9 DRDY Int set"));
    }

    if (s_IntSrcReg1 & 0x1)
    {
        NVODMACCELEROMETER_PRINTF(("\nkxtf9 Tap In Z+ direction"));
    }
    if (s_IntSrcReg1 & 0x2)
    {
        NVODMACCELEROMETER_PRINTF(("\nkxtf9 Tap In Z- direction"));
    }
    if (s_IntSrcReg1 & 0x4)
    {
        NVODMACCELEROMETER_PRINTF(("\nkxtf9 Tap In Y+ direction"));
    }
    if (s_IntSrcReg1 & 0x8)
    {
        NVODMACCELEROMETER_PRINTF(("\nkxtf9 Tap In Y- direction"));
    }
    if (s_IntSrcReg1 & 0x10)
    {
        NVODMACCELEROMETER_PRINTF(("\nkxtf9 Tap In X+ direction"));
    }
    if (s_IntSrcReg1 & 0x20)
    {
        NVODMACCELEROMETER_PRINTF(("\nkxtf9 Tap In X- direction"));
    }
}
#endif

NvBool
kxtf9_GetAcceleration(
    NvOdmAccelHandle hDevice,
    NvS32           *AccelX,
    NvS32           *AccelY,
    NvS32           *AccelZ)
{
    NvS32 data;
    NvS32 TempAccelX = 0;
    NvS32 TempAccelY = 0;
    NvS32 TempAccelZ = 0;

#if NVODMACCELEROMETER_ENABLE_PRINTF
    static NvU32 PrintCount;
    // Print once only in every PRINT_FREQUENCY iterations.
    #define PRINT_FREQUENCY 50
#endif

    NV_ASSERT(NULL != hDevice);
    NV_ASSERT(NULL != AccelX);
    NV_ASSERT(NULL != AccelY);
    NV_ASSERT(NULL != AccelZ);

#if NVODMACCELEROMETER_ENABLE_PRINTF
    PrintAccelEventInfo(hDevice);
#endif

    kxtf9_ReadXYZ(hDevice, &TempAccelX, &TempAccelY, &TempAccelZ);

    data = (TempAccelX * s_MaxGValInMilliGs) / s_MaxAccelReading;
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

    data = (TempAccelY * s_MaxGValInMilliGs) / s_MaxAccelReading;
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

    data = (TempAccelZ * s_MaxGValInMilliGs) / s_MaxAccelReading;
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

    if (s_IntSrcReg2)
    {
#if NVODMACCELEROMETER_ENABLE_PRINTF
        if (!(PrintCount++ % PRINT_FREQUENCY))
        {
            NVODMACCELEROMETER_PRINTF(("\nkxtf9 milli g-Vals, x=%d,y=%d, z=%d",
            *AccelX, *AccelY, *AccelZ));
        }
#endif
        return NV_TRUE;
    }
    else
        return NV_FALSE;
}

NvOdmAccelerometerCaps kxtf9_GetCaps(NvOdmAccelHandle hDevice)
{
    NV_ASSERT(NULL != hDevice);
    return hDevice->Caption;
}

// FixMe: kxtf9 accelerometer supports only 4 sample rates - 25, 50, 100 & 200.
// So setting the param to the nearest max value below the requested one.
NvBool kxtf9_SetSampleRate(NvOdmAccelHandle hDevice, NvU32 SampleRate)
{
    NvU8 BitVal = 0;
    NvU8 RegVal = 0;

    if (SampleRate < 50)
        BitVal = 0; // set to 25
    else if (SampleRate < 100)
        BitVal = 1; // set to 50
    else if (SampleRate < 200)
        BitVal = 2; // set to 100
    else
        BitVal = 3; // set to 200

    if(!SetAccelerometerActive(hDevice, NV_FALSE))
        return NV_FALSE;

    // To set requested freq.
    ReadReg(hDevice, kxtf9_CTRL_REG3, &RegVal, 1);
    RegVal &= 0xFC;
    RegVal |= BitVal;
    WriteReg(hDevice, kxtf9_CTRL_REG3, &RegVal, 1);

    // to set PC1 of CTRL_REG1
    if(!SetAccelerometerActive(hDevice, NV_TRUE))
        return NV_FALSE;
    NVODMACCELEROMETER_PRINTF(("\naccel set freq: %d", SampleRate));
    return NV_TRUE;
}

NvBool kxtf9_GetSampleRate(NvOdmAccelHandle hDevice, NvU32 *pSampleRate)
{
    return NV_TRUE;
}

NvBool
kxtf9_SetPowerState(
    NvOdmAccelHandle hDevice,
    NvOdmAccelPowerType PowerState)
{
    if (PowerState == NvOdmAccelPower_Fullrun)
    {
        NVODMACCELEROMETER_PRINTF(("\nkxtf9: Resume"));
        return SetAccelerometerActive(hDevice, NV_TRUE);

    }
    else  // any other case, set accelerometer to standby.
    {
        NVODMACCELEROMETER_PRINTF(("\nkxtf9: Suspend"));
        return SetAccelerometerActive(hDevice, NV_FALSE);
    }
}

NvBool kxtf9_init(NvOdmAccelHandle* hDevice)
{
    NvU32 i;
    NvOdmAccelHandle hAccel;
    NvOdmIoModule IoModule = NvOdmIoModule_I2c;
    const NvOdmPeripheralConnectivity *pConnectivity;
    NvBool FoundGpio = NV_FALSE, FoundI2cModule = NV_FALSE;
    NvU32 AxesMapping = 0;
    hAccel = NvOdmOsAlloc(sizeof(NvOdmAccel));
    if (hAccel == NULL)
    {
        NVODMACCELEROMETER_PRINTF(("\nkxtf9: Error Allocating NvOdmAccel"));
        return NV_FALSE;
    }
    NvOdmOsMemset(hAccel, 0, sizeof(NvOdmAccel));

    // Info of accelerometer with current setting.
    hAccel->Caption.MaxForceInGs = 2000;
    hAccel->Caption.MaxTapTimeDeltaInUs = 255;
    hAccel->Caption.NumMotionThresholds = 1;
    hAccel->Caption.SupportsFreefallInt = 0;
    hAccel->Caption.MaxSampleRate = 100;
    hAccel->Caption.MinSampleRate = 3;
    hAccel->PowerState = NvOdmAccelPower_Fullrun;
    hAccel->hPmu = NvOdmServicesPmuOpen();
    if (!hAccel->hPmu)
    {
        NVODMACCELEROMETER_PRINTF(("kxtf9 NvOdmServicesPmuOpen Error \n"));
        goto error;
    }
    for ( i = 0 ; i < NV_ARRAY_SIZE(disc_entry); ++i )
    {
        pConnectivity = NvOdmPeripheralGetGuid(disc_entry[i].guid);
        if (pConnectivity)
        {
            AxesMapping = disc_entry[i].axesmapping;
            break;
        }
    }
    if (!pConnectivity)
        goto error;

    if (AxesMapping == 1)
    {
    // Axes mapping for display orientation aligning correctly in 3 orientations
    // (0, 90 & 270 degrees) on Tango with (froyo + K32).
        hAccel->AxisXMapping = NvOdmAccelAxis_Y;
        hAccel->AxisXDirection = 1;
        hAccel->AxisYMapping = NvOdmAccelAxis_X;
        hAccel->AxisYDirection = -1;
        hAccel->AxisZMapping = NvOdmAccelAxis_Z;
        hAccel->AxisZDirection = -1;
    }
    else
    {
        hAccel->AxisXMapping = NvOdmAccelAxis_X;
        hAccel->AxisXDirection = 1;
        hAccel->AxisYMapping = NvOdmAccelAxis_Y;
        hAccel->AxisYDirection = -1;
        hAccel->AxisZMapping = NvOdmAccelAxis_Z;
        hAccel->AxisZDirection = 1;
    }

    if (pConnectivity->Class != NvOdmPeripheralClass_Other)
    {
        NVODMACCELEROMETER_PRINTF(("\nkxtf9: NvOdmPeripheral class mismatch"));
        goto error;
    }

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
                ConfigPowerRail(hAccel->hPmu, hAccel->VddId, NV_TRUE);
                break;
            default:
                break;
        }
    }

    if (!FoundGpio || !FoundI2cModule)
    {
        NVODMACCELEROMETER_PRINTF(("kxtf9: didn't find any periperal\
            in discovery query for touch device Error \n"));
        goto error;
    }

    // Open I2C handle.
    hAccel->hOdmI2C = NvOdmI2cOpen(IoModule, hAccel->I2CChannelId);
    if (!hAccel->hOdmI2C)
    {
        NVODMACCELEROMETER_PRINTF(("kxtf9: I2c Open Fail\n"));
        goto error;
    }

    hAccel->RegsRead  = ReadReg;
    hAccel->RegsWrite = WriteReg;

    if (!kxtf9_Init(hAccel))
    {
        NVODMACCELEROMETER_PRINTF(("kxtf9: Init Failed\n"));
        goto error;
    }

    if (!ConfigInterrupt(hAccel))
    {
        NVODMACCELEROMETER_PRINTF(("kxtf9: ConfigInterrupt Failed \n"));
        goto error;
    }

    hAccel->AccelClose = kxtf9_deInit;
    hAccel->AccelSetIntForceThreshold = kxtf9_SetIntForceThreshold;
    hAccel->AccelSetIntTimeThreshold = kxtf9_SetIntTimeThreshold;
    hAccel->AccelSetIntEnable = kxtf9_SetIntEnable;
    hAccel->AccelWaitInt = kxtf9_WaitInt;
    hAccel->AccelSignal = kxtf9_Signal;
    hAccel->AccelGetAcceleration = kxtf9_GetAcceleration;
    hAccel->AccelGetCaps = kxtf9_GetCaps;
    hAccel->AccelSetPowerState = kxtf9_SetPowerState;
    hAccel->AccelSetSampleRate = kxtf9_SetSampleRate;
    hAccel->AccelGetSampleRate = kxtf9_GetSampleRate;

    *hDevice = hAccel;
    NVODMACCELEROMETER_PRINTF(("kxtf9 Accel Open success\n"));
    return NV_TRUE;
error:
    NVODMACCELEROMETER_PRINTF(("kxtf9: Error during NvOdmAccelOpen\n"));

    // Release all of resources requested.
    if (hAccel)
    {
        ConfigPowerRail(hAccel->hPmu, hAccel->VddId, NV_FALSE);
        NvOdmServicesPmuClose(hAccel->hPmu);
        hAccel->hPmu = NULL;
        NvOdmI2cClose(hAccel->hOdmI2C);
        hAccel->hOdmI2C = NULL;
        NvOdmOsFree(hAccel);
        *hDevice = NULL;
    }
    return NV_FALSE;
}


