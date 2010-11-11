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

#include "nvodm_touch_panjit.h"
#include "nvodm_query_discovery.h"
#include "nvos.h"

#define PANJIT_BENCHMARK_SAMPLE                     0
#define PANJIT_TOUCH_DEVICE_GUID NV_ODM_GUID('p','a','n','j','i','t','_','0')
#define PANJIT_READ(dev, reg, buffer, len) \
    PANJIT_ReadRegister(dev, reg, buffer, len)
#define PANJIT_I2C_SPEED_KHZ                        400
#define SAMPLES_PER_SECOND                          80

#define SLEEP_MODE_NORMAL                           0x00
#define SLEEP_MODE_SENSOR_SLEEP                     0x01

#define X_MIN                                       0x00
#define Y_MIN                                       0x00
#define X_MAX                                       4095
#define Y_MAX                                       4095
#define PANJIT_I2C_TIMEOUT                          1000
#define PANJIT_DEBOUNCE_TIME_MS                     1
#define TP_DATA_LENGTH                              12
#define TP_TOUCH_STATE_BYTE                         0
#define TP_FINGER_ONE_MASK                          0x01
#define TP_FINGER_TWO_MASK                          0x02
#define TP_SPECIAL_FUNCTION_BYTE                    (TP_DATA_LENGTH-1)
#define TP_FINGER_POSITION                          (TP_SPECIAL_FUNCTION_BYTE-1)

static NvBool ResumeInProgress = NV_FALSE;
static NvBool IsResetSupported = NV_FALSE;

static const
NvOdmTouchCapabilities PANJIT_Capabilities =
{
    1,      // IsMultiTouchSupported
    2,      // MaxNumberOfFingerCoordReported;
    0,      // IsRelativeDataSupported
    0,      // MaxNumberOfRelativeCoordReported
    0,      // MaxNumberOfWidthReported
    0,      // MaxNumberOfPressureReported
    (NvU32)NvOdmTouchGesture_Not_Supported, // Gesture
    0,      // IsWidthSupported
    0,      // IsPressureSupported
    1,      // IsFingersSupported
    X_MIN,   // XMinPosition
    Y_MIN,   // YMinPosition
    X_MAX,   // XMaxPosition
    Y_MAX,   // YMaxPosition
    0
};

#define INT_PIN_ACTIVE_STATE 0

static NvBool
PANJIT_ReadRegister(
    PANJIT_TouchDevice* hTouch,
    NvU8 reg,
    NvU8* buffer,
    NvU32 len)
{
    NvOdmI2cStatus Error;
    NvOdmI2cTransactionInfo TransactionInfo;

    TransactionInfo.Address = (hTouch->DeviceAddr | 0x1);
    TransactionInfo.Buf = buffer;
    TransactionInfo.Flags = 0;
    TransactionInfo.NumBytes = len;

    Error = NvOdmI2cTransaction(hTouch->hOdmI2c,
                                &TransactionInfo,
                                1,
                                hTouch->I2cClockSpeedKHz,
                                PANJIT_I2C_TIMEOUT);

    if (Error != NvOdmI2cStatus_Success)
    {
        NVODMTOUCH_PRINTF(("I2C Read Failure = %d (addr=0x%x, reg=0x%x)\r\n", Error,
                           hTouch->DeviceAddr, reg));
        return NV_FALSE;
    }

    return NV_TRUE;
}

static NvBool
PANJIT_WriteRegister(
    PANJIT_TouchDevice* hTouch,
    NvU8 reg,
    NvU8* buffer,
    NvU32 len)
{
    NvOdmI2cStatus Error;
    NvOdmI2cTransactionInfo TransactionInfo;

    TransactionInfo.Address = hTouch->DeviceAddr;
    TransactionInfo.Buf = buffer;
    TransactionInfo.Flags = NVODM_I2C_IS_WRITE;
    TransactionInfo.NumBytes = len;

    Error = NvOdmI2cTransaction(hTouch->hOdmI2c,
                                &TransactionInfo,
                                1,
                                hTouch->I2cClockSpeedKHz,
                                PANJIT_I2C_TIMEOUT);

    if (Error != NvOdmI2cStatus_Success)
    {
        NVODMTOUCH_PRINTF(("I2C Write Failure = %d (addr=0x%x, reg=0x%x)\r\n",
            Error, hTouch->DeviceAddr, reg));
        return NV_FALSE;
    }
    return NV_TRUE;
}

static void PANJIT_EnableScanMode(PANJIT_TouchDevice* hTouch)
{
    NvU8 buff[4];

    // Enable Scan Mode.
    buff[0] = 0;
    buff[1] = 8;
    PANJIT_WriteRegister(hTouch, 0, buff, 2);
}

static void PANJIT_ClearInterrupt(PANJIT_TouchDevice* hTouch)
{
    NvU8 buff[4];

    // Clear Interrupt.
    buff[0] = 1;
    buff[1] = 0;
    PANJIT_WriteRegister(hTouch, 0, buff, 2);
}

static void PANJIT_ToggleSleepMode(PANJIT_TouchDevice* hTouch, NvBool IsSleep)
{
    NvU8 buff[4];

    buff[0] = 0;
    if (IsSleep)
        buff[1] = 0x0;
    else
        buff[1] = 0;
    PANJIT_WriteRegister(hTouch, 0, buff, 2);

    buff[0] = 0;
    if (IsSleep) {
        buff[1] = 0x80;
        PANJIT_WriteRegister(hTouch, 0, buff, 2);
    }
}
static NvBool PANJIT_Configure(PANJIT_TouchDevice* hTouch)
{

    NVODMTOUCH_PRINTF(("PANJIT_Configure\r\n"));

    hTouch->SleepMode = SLEEP_MODE_NORMAL;
    hTouch->SampleRate = SAMPLES_PER_SECOND;

    // Enable Scan Mode.
    PANJIT_EnableScanMode(hTouch);
    // Clear Interrupt.
    PANJIT_ClearInterrupt(hTouch);
    return NV_TRUE;
}

static NvBool
PANJIT_GetSample(
    PANJIT_TouchDevice* hTouch,
    NvOdmTouchCoordinateInfo* coord)
{
    NvU8 TouchData[TP_DATA_LENGTH] = {0};

    NVODMTOUCH_PRINTF(("PANJIT_GetSample+\r\n"));

    if (!hTouch || !coord)
        return NV_FALSE;

    if (!PANJIT_READ(hTouch, 0, &TouchData[0], TP_DATA_LENGTH))
        return NV_FALSE;

    NVODMTOUCH_PRINTF(("TouchData=0x%x %x %x %x %x %x %x %x %x %x %x %x\r\n",
        TouchData[0], TouchData[1], TouchData[2], TouchData[3], TouchData[4],
        TouchData[5], TouchData[6], TouchData[7],TouchData[8], TouchData[9],
        TouchData[10], TouchData[11]));

    /* Ignore No finger */
    coord->fingerstate = (TouchData[TP_FINGER_POSITION] & (TP_FINGER_ONE_MASK | TP_FINGER_TWO_MASK)) ?
        NvOdmTouchSampleValidFlag : NvOdmTouchSampleIgnore;

    if (coord->fingerstate == NvOdmTouchSampleIgnore)
    {
        if (hTouch->PrevFingers == 0)
        {
            NVODMTOUCH_PRINTF(("NvOdmTouchSampleIgnore\r\n"));
            return NV_TRUE;
        }
        coord->fingerstate = NvOdmTouchSampleValidFlag;
    }

    // get the finger count
    coord->additionalInfo.Fingers = TouchData[TP_FINGER_POSITION];
    if (coord->additionalInfo.Fingers > hTouch->Caps.MaxNumberOfFingerCoordReported)
        coord->additionalInfo.Fingers = hTouch->Caps.MaxNumberOfFingerCoordReported;

    NVODMTOUCH_PRINTF(("coord->additionalInfo.Fingers = %d\r\n",
        coord->additionalInfo.Fingers));

    if (coord->additionalInfo.Fingers)
    {
        coord->fingerstate |= NvOdmTouchSampleDownFlag;
        coord->xcoord = TouchData[2];
        coord->xcoord = ((coord->xcoord<<8)|(TouchData[3]));
        coord->ycoord = TouchData[4];
        coord->ycoord = ((coord->ycoord<<8)|(TouchData[5]));
        coord->additionalInfo.multi_XYCoords[0][0] = coord->xcoord;
        coord->additionalInfo.multi_XYCoords[0][1] = coord->ycoord;
        coord->additionalInfo.multi_XYCoords[1][0] = (TouchData[6]<<8) | (TouchData[7]);
        coord->additionalInfo.multi_XYCoords[1][1] = (TouchData[8]<<8) | (TouchData[9]);
    }
    hTouch->PrevFingers = coord->additionalInfo.Fingers;
    NVODMTOUCH_PRINTF(("(%d,%d)\r\n", coord->xcoord, coord->ycoord));
    return NV_TRUE;
}

static void PANJIT_GpioIsr(void *arg)
{
    PANJIT_TouchDevice *hTouch = (PANJIT_TouchDevice *)arg;
    if (ResumeInProgress == NV_TRUE)
    {
        //Skip this interrupt as this is a spurious interrupt
        NvOdmGpioInterruptDone(hTouch->hGpioIntr);
        return;
    }

    /* Signal the touch thread to read the sample. After it is done reading the
     * sample it should re-enable the interrupt. */
    NvOdmOsSemaphoreSignal(hTouch->hIntSema);
}

NvBool
PANJIT_ReadCoordinate(
    NvOdmTouchDeviceHandle hDevice,
    NvOdmTouchCoordinateInfo* coord)
{
    PANJIT_TouchDevice *hTouch = (PANJIT_TouchDevice *)hDevice;
    NvBool status = NV_FALSE;
    static NvU32 prevSamleTime;
    NvU32 CurrentSampleTime;
#if PANJIT_BENCHMARK_SAMPLE
    NvU32 time;
#endif
    NvU32 pinValue;

    NvOdmGpioGetState(hTouch->hGpio, hTouch->hPin, &pinValue);
    if (pinValue != INT_PIN_ACTIVE_STATE)
        goto fail;

    CurrentSampleTime = NvOdmOsGetTimeMS();
    if (prevSamleTime)
    {
        if ((1000/hTouch->SampleRate) > (CurrentSampleTime - prevSamleTime))
        {
            NvOsSleepMS((1000/hTouch->SampleRate) - (CurrentSampleTime - prevSamleTime));
        }
    }
    prevSamleTime = CurrentSampleTime;

#if PANJIT_BENCHMARK_SAMPLE
    time = CurrentSampleTime;
#endif

    status = PANJIT_GetSample(hTouch, coord);

#if PANJIT_BENCHMARK_SAMPLE
    NvOdmOsDebugPrintf("Touch sample time %d\r\n", NvOdmOsGetTimeMS() - time);
#endif
fail:
    if (status == NV_FALSE)
        NvOdmGpioInterruptDone(hTouch->hGpioIntr);
    return status;
}

void
PANJIT_GetCapabilities(
    NvOdmTouchDeviceHandle hDevice,
    NvOdmTouchCapabilities* pCapabilities)
{
    PANJIT_TouchDevice *hTouch = (PANJIT_TouchDevice *)hDevice;

    if (hTouch && pCapabilities)
        *pCapabilities = hTouch->Caps;
}

void PANJIT_Close (NvOdmTouchDeviceHandle hDevice)
{
    PANJIT_TouchDevice *hTouch = (PANJIT_TouchDevice *)hDevice;

    if (hTouch)
    {
        if (hTouch->hGpio)
        {
            if (hTouch->hGpioIntr)
            {
                NvOdmGpioInterruptUnregister(hTouch->hGpio,
                    hTouch->hPin, hTouch->hGpioIntr);
                hTouch->hGpioIntr = NULL;
            }

            if (hTouch->hPin)
            {
                NvOdmGpioReleasePinHandle(hTouch->hGpio, hTouch->hPin);
                hTouch->hPin = NULL;
            }
            if (hTouch->hResetPin)
            {
                NvOdmGpioReleasePinHandle(hTouch->hGpio, hTouch->hResetPin);
                hTouch->hResetPin = NULL;
            }

            NvOdmGpioClose(hTouch->hGpio);
            hTouch->hGpio = NULL;
        }

        if (hTouch->hOdmI2c)
        {
            NvOdmI2cClose(hTouch->hOdmI2c);
            hTouch->hOdmI2c = NULL;
        }

        NvOdmOsFree(hTouch);
        hTouch = NULL;
    }
}

static void InitOdmTouch (NvOdmTouchDevice *pDev)
{
    if (pDev)
    {
        pDev->Close              = PANJIT_Close;
        pDev->GetCapabilities    = PANJIT_GetCapabilities;
        pDev->ReadCoordinate     = PANJIT_ReadCoordinate;
        pDev->EnableInterrupt    = PANJIT_EnableInterrupt;
        pDev->HandleInterrupt    = PANJIT_HandleInterrupt;
        pDev->GetSampleRate      = PANJIT_GetSampleRate;
        pDev->SetSampleRate      = PANJIT_SetSampleRate;
        pDev->PowerControl       = PANJIT_PowerControl;
        pDev->PowerOnOff         = PANJIT_PowerOnOff;
        pDev->GetCalibrationData = PANJIT_GetCalibrationData;
        pDev->OutputDebugMessage = NV_TRUE;
    }
}

NvBool PANJIT_Open(NvOdmTouchDeviceHandle *hDevice)
{
    PANJIT_TouchDevice *hTouch = NULL;
    NvU32 i = 0;
    NvU32 found = 0, gpiofound = 0;
    NvU32 GpioPort = 0, ResetPort = 0;
    NvU32 GpioPin = 0, ResetPin = 0;
    NvU32 I2cInstance = 0;
    NvOdmIoModule IoModule = NvOdmIoModule_I2c;
    const NvOdmPeripheralConnectivity *pConnectivity = NULL;

    NVODMTOUCH_PRINTF(("NvOdm Touch : PANJIT_Open() \r\n"));

    // allocate memory to be used for the handle
    hTouch = NvOdmOsAlloc(sizeof(PANJIT_TouchDevice));
    if (!hTouch)
        return NV_FALSE;

    NvOdmOsMemset(hTouch, 0, sizeof(PANJIT_TouchDevice));

    /* set function pointers */
    InitOdmTouch(&hTouch->OdmTouch);

    // read the query database
    pConnectivity = NvOdmPeripheralGetGuid(PANJIT_TOUCH_DEVICE_GUID);
    if (!pConnectivity)
    {
        NVODMTOUCH_PRINTF(("NvOdm Touch : pConnectivity is NULL Error \r\n"));
        goto fail;
    }

    if (pConnectivity->Class != NvOdmPeripheralClass_HCI)
    {
        NVODMTOUCH_PRINTF(("NvOdm Touch : didn't find any periperal\
            in discovery query for touch device Error \r\n"));
        goto fail;
    }

    for (i = 0; i < pConnectivity->NumAddress; i++)
    {
        switch (pConnectivity->AddressList[i].Interface)
        {
            case NvOdmIoModule_I2c:
            case NvOdmIoModule_I2c_Pmu:
                hTouch->DeviceAddr = pConnectivity->AddressList[i].Address;
                I2cInstance = pConnectivity->AddressList[i].Instance;
                found++;
                IoModule = pConnectivity->AddressList[i].Interface;
                break;

            case NvOdmIoModule_Gpio:
                if (gpiofound == 0)
                {
                    GpioPort = pConnectivity->AddressList[i].Instance;
                    GpioPin = pConnectivity->AddressList[i].Address;
                }
                else
                {
                    ResetPort = pConnectivity->AddressList[i].Instance;
                    ResetPin = pConnectivity->AddressList[i].Address;
                }
                gpiofound++;
                break;

            default:
                break;
        }
    }

    if ((found == 0) || (gpiofound == 0))
    {
        NVODMTOUCH_PRINTF(("NvOdm Touch:peripheral connectivity problem\n"));
        goto fail;
    }
    // see if we found the bus, GPIO and reset GPIO used by the hardware
    if (gpiofound != 2)
    {
        // No information about reset pin available, so don't reset
        // Suspend and resume will not work in this configuration
        IsResetSupported = NV_FALSE;
        NVODMTOUCH_PRINTF(("NvOdm Touch: Suspend/resume are not supported\n"));
    }
    else
    {
        // Reset pin available
        IsResetSupported = NV_TRUE;
    }

    // allocate I2C instance
    hTouch->hOdmI2c = NvOdmI2cOpen(IoModule, I2cInstance);
    if (!hTouch->hOdmI2c)
    {
        NVODMTOUCH_PRINTF(("NvOdm Touch : NvOdmI2cOpen Error \r\n"));
        goto fail;
    }

    // get the handle to the pin used as pen down interrupt
    hTouch->hGpio = (NvOdmServicesGpioHandle)NvOdmGpioOpen();
    if (!hTouch->hGpio)
    {
        NVODMTOUCH_PRINTF(("NvOdm Touch : NvOdmGpioOpen Error \r\n"));
        goto fail;
    }

    hTouch->hPin = NvOdmGpioAcquirePinHandle(hTouch->hGpio, GpioPort, GpioPin);
    if (!hTouch->hPin)
    {
        NVODMTOUCH_PRINTF(("NvOdm Touch : Couldn't get GPIO pin \r\n"));
        goto fail;
    }

    NvOdmGpioConfig(hTouch->hGpio, hTouch->hPin, NvOdmGpioPinMode_InputData);

    if (IsResetSupported)
    {
        /* Configure the reset GPIO */
        hTouch->hResetPin = NvOdmGpioAcquirePinHandle(hTouch->hGpio, ResetPort, ResetPin);
        if (!hTouch->hResetPin)
        {
            NVODMTOUCH_PRINTF(("NvOdm Touch : Couldn't get Reset GPIO pin \r\n"));
            goto fail;
        }
        NvOdmGpioConfig(hTouch->hGpio, hTouch->hResetPin, NvOdmGpioPinMode_Output);
        /* Send reset pulse to touch HW */
        NvOdmGpioSetState(hTouch->hGpio, hTouch->hResetPin, 1);
        NvOsWaitUS(50);
        NvOdmGpioSetState(hTouch->hGpio, hTouch->hResetPin, 0);
        NvOsSleepMS(50);
    }

    /* set default capabilities */
    NvOdmOsMemcpy(&hTouch->Caps, &PANJIT_Capabilities,
        sizeof(NvOdmTouchCapabilities));

    /* set default I2C speed */
    hTouch->I2cClockSpeedKHz = PANJIT_I2C_SPEED_KHZ;

    /* set max positions */
    hTouch->Caps.XMaxPosition = X_MAX;
    hTouch->Caps.YMaxPosition = Y_MAX;

    PANJIT_Configure(hTouch);

    *hDevice = (NvOdmTouchDeviceHandle)hTouch;
    return NV_TRUE;

 fail:
    PANJIT_Close((NvOdmTouchDeviceHandle)hTouch);
    hTouch = NULL;
    return NV_FALSE;
}

NvBool
PANJIT_EnableInterrupt(
    NvOdmTouchDeviceHandle hDevice,
    NvOdmOsSemaphoreHandle hIntSema)
{
    PANJIT_TouchDevice *hTouch = (PANJIT_TouchDevice *)hDevice;

    NV_ASSERT(hIntSema);

    /* can only be initialized once */
    if (hTouch->hGpioIntr || hTouch->hIntSema)
        return NV_FALSE;

    hTouch->hIntSema = hIntSema;

    if (NvOdmGpioInterruptRegister(
                hTouch->hGpio,
                &hTouch->hGpioIntr,
                hTouch->hPin,
                NvOdmGpioPinMode_InputInterruptLow,
                PANJIT_GpioIsr,
                (void*)hTouch,
                PANJIT_DEBOUNCE_TIME_MS) == NV_FALSE)
    {
        return NV_FALSE;
    }

    if (!hTouch->hGpioIntr)
        return NV_FALSE;
    return NV_TRUE;
}

NvBool PANJIT_HandleInterrupt(NvOdmTouchDeviceHandle hDevice)
{
    PANJIT_TouchDevice *hTouch = (PANJIT_TouchDevice *)hDevice;
    NvU32 pinValue;

    if (hTouch)
    {
        NvOdmGpioGetState(hTouch->hGpio, hTouch->hPin, &pinValue);
        if (pinValue == INT_PIN_ACTIVE_STATE)
        {
            // Clear Interrupt.
            PANJIT_ClearInterrupt(hTouch);
            NvOdmGpioInterruptDone(hTouch->hGpioIntr);
            return NV_TRUE;
        }
        NvOdmGpioInterruptDone(hTouch->hGpioIntr);
    }
    return NV_FALSE;
}

NvBool
PANJIT_GetSampleRate(
    NvOdmTouchDeviceHandle hDevice,
    NvOdmTouchSampleRate* pTouchSampleRate)
{
    if (pTouchSampleRate)
    {
        pTouchSampleRate->NvOdmTouchSampleRateHigh = SAMPLES_PER_SECOND;
        pTouchSampleRate->NvOdmTouchSampleRateLow = SAMPLES_PER_SECOND;
        pTouchSampleRate->NvOdmTouchCurrentSampleRate = 0; // 0 = low , 1 = high
    }
    return NV_TRUE;
}

NvBool PANJIT_SetSampleRate(NvOdmTouchDeviceHandle hDevice, NvU32 rate)
{
    PANJIT_TouchDevice *hTouch = (PANJIT_TouchDevice *)hDevice;

    if (hTouch)
    {
        if (rate > SAMPLES_PER_SECOND)
            hTouch->SampleRate = SAMPLES_PER_SECOND;
        else
            hTouch->SampleRate = rate;
    }
    return NV_TRUE;
}

static NvBool
PANJIT_SetSleepMode(
    NvOdmTouchDeviceHandle hDevice,
        NvU8 mode)
{
    PANJIT_TouchDevice *hTouch = (PANJIT_TouchDevice *)hDevice;
    if (IsResetSupported == NV_FALSE)
        return NV_TRUE;
    if (mode == SLEEP_MODE_SENSOR_SLEEP)
    {
        PANJIT_ToggleSleepMode(hTouch, NV_TRUE);
    }
    else if (mode == SLEEP_MODE_NORMAL)
    {
        ResumeInProgress = NV_TRUE;
        NvOdmGpioInterruptMask(hTouch->hGpioIntr, NV_TRUE);
        NvOdmGpioSetState(hTouch->hGpio, hTouch->hResetPin, 1);
        NvOsSleepMS(50);
        NvOdmGpioSetState(hTouch->hGpio, hTouch->hResetPin, 0);
        NvOsSleepMS(50);
        PANJIT_EnableScanMode(hTouch);
        PANJIT_ClearInterrupt(hTouch);
        NvOdmGpioInterruptMask(hTouch->hGpioIntr, NV_FALSE);
        ResumeInProgress = NV_FALSE;
    }
    else
        return NV_FALSE;
    return NV_TRUE;
}

NvBool
PANJIT_PowerControl(
    NvOdmTouchDeviceHandle hDevice,
    NvOdmTouchPowerModeType mode)
{
    PANJIT_TouchDevice *hTouch = (PANJIT_TouchDevice *)hDevice;
    NvU8 SleepMode;

    switch(mode)
    {
        case NvOdmTouch_PowerMode_0:
            SleepMode = SLEEP_MODE_NORMAL;
            break;
        case NvOdmTouch_PowerMode_1:
        case NvOdmTouch_PowerMode_2:
        case NvOdmTouch_PowerMode_3:
            SleepMode = SLEEP_MODE_SENSOR_SLEEP;
            break;
        default:
            return NV_FALSE;
    }
    PANJIT_SetSleepMode(hDevice, SleepMode);
    if (hTouch->SleepMode == SleepMode)
        return NV_TRUE;
    hTouch->SleepMode = SleepMode;
    return NV_TRUE;
}

NvBool
PANJIT_GetCalibrationData(
    NvOdmTouchDeviceHandle hDevice,
    NvU32 NumOfCalibrationData,
    NvS32* pRawCoordBuffer)
{
    static NvS32 RawCoordBuffer[] = {2048,2048,840,840,840,3330,3280,3330,3280,840};

    if (!pRawCoordBuffer)
        return NV_FALSE;

    if (NumOfCalibrationData * 2 != (sizeof(RawCoordBuffer) / sizeof(NvS32)))
    {
        NVODMTOUCH_PRINTF(("WARNING: number of calibration data isn't matched\r\n"));
        return NV_FALSE;
    }

    NvOdmOsMemcpy(pRawCoordBuffer, RawCoordBuffer, sizeof(RawCoordBuffer));
    return NV_TRUE;
}

NvBool PANJIT_PowerOnOff(NvOdmTouchDeviceHandle hDevice, NvBool OnOff)
{
    if (!hDevice)
        return NV_FALSE;

    if (OnOff)
        return PANJIT_PowerControl(hDevice, NvOdmTouch_PowerMode_0); // power ON
    else
        return PANJIT_PowerControl(hDevice, NvOdmTouch_PowerMode_3); // power OFF
}

