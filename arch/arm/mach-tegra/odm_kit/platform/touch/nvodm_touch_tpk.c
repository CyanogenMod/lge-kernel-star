/*
 * Copyright (c) 2007-2009 NVIDIA Corporation.
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

#include "nvodm_touch_int.h"
#include "nvodm_services.h"
#include "nvodm_touch_tpk.h"
#include "nvodm_query_discovery.h"
#include "tpk_reg.h"

#define TPK_I2C_SPEED_KHZ                          40
#define TPK_I2C_TIMEOUT                            500
#define TPK_LOW_SAMPLE_RATE                        0    //40 reports per-second
#define TPK_HIGH_SAMPLE_RATE                       1    //80 reports per-second
#define TPK_MAX_READ_BYTES                         16
#define TPK_MAX_PACKET_SIZE                        8
#define TPK_CHECK_ERRORS                           0
#define TPK_BENCHMARK_SAMPLE                       0
#define TPK_REPORT_WAR_SUCCESS                     0
#define TPK_REPORT_2ND_FINGER_DATA                 0
#define TPK_DUMP_REGISTER                           0
#define TPK_SCREEN_ANGLE                            0   //0=Landscape, 1=Portrait
#define TPK_QUERY_SENSOR_RESOLUTION                0    //units per millimeter
#define TPK_SET_MAX_POSITION                       0
#define TPK_PAGE_CHANGE_DELAY                      0 //This should be unnecessary
#define TPK_POR_DELAY                              100 //Dealy after Power-On Reset
                                                       //500ms(Max) is suggested 
                                                       //by RMI Interface Guide
                                                       //(511-000099-01 Rev.B)

#define TPK_ADL340_WAR                              0
/* WAR for spurious zero reports from panel: verify zero
   fingers sample data by waiting one refresh interval and
   retrying reading data */
#define TPK_SPURIOUS_ZERO_WAR                      1

#define TPK_TOUCH_DEVICE_GUID NV_ODM_GUID('t','p','k','t','o','u','c','h')

#define TPK_WRITE(dev, reg, byte) TPK_WriteRegister(dev, reg, byte)
#define TPK_READ(dev, reg, buffer, len) TPK_ReadRegisterSafe(dev, reg, buffer, len)
#define TPK_DEBOUNCE_TIME_MS 0

typedef struct TPK_TouchDeviceRec
{
    NvOdmTouchDevice OdmTouch;
    NvOdmTouchCapabilities Caps;
    NvOdmServicesI2cHandle hOdmI2c;
    NvOdmServicesGpioHandle hGpio;
    NvOdmServicesPmuHandle hPmu;
    NvOdmGpioPinHandle hPin;
    NvOdmServicesGpioIntrHandle hGpioIntr;
    NvOdmOsSemaphoreHandle hIntSema;
    NvBool PrevFingers;
    NvU32 DeviceAddr;
    NvU32 SampleRate;
    NvU32 SleepMode;
    NvBool PowerOn;
    NvU32 VddId;    
    NvU32 ChipRevisionId; //Id=0x01:TPK chip on Concorde1
                          //id=0x02:TPK chip with updated firmware on Concorde2
    NvU32 I2cClockSpeedKHz;
} TPK_TouchDevice;


static const NvOdmTouchCapabilities TPK_Capabilities =
{
    1, //IsMultiTouchSupported
    2, //MaxNumberOfFingerCoordReported;
    0, //IsRelativeDataSupported
    1, //MaxNumberOfRelativeCoordReported
    15, //MaxNumberOfWidthReported
    255, //MaxNumberOfPressureReported
    (NvU32)NvOdmTouchGesture_Not_Supported, //Gesture
    1, //IsWidthSupported
    1, //IsPressureSupported
    1, //IsFingersSupported
    0, //XMinPosition
    0, //YMinPosition
    0, //XMaxPosition
    0, //YMaxPosition
#if TPK_SCREEN_ANGLE
    (NvU32)NvOdmTouchOrientation_H_FLIP // Orientation 4 inch tpk panel
#else
    (NvU32)(NvOdmTouchOrientation_XY_SWAP | NvOdmTouchOrientation_H_FLIP | NvOdmTouchOrientation_V_FLIP)
#endif    
};

#if TPK_ADL340_WAR
// Dummy write accelerometer in order to workaround a HW bug of ADL340
// Will Remove it once we use new accelerometer
static NvBool NvAccDummyI2CSetRegs(TPK_TouchDevice* hTouch)
{
    NvOdmI2cTransactionInfo TransactionInfo;
    NvU8 arr[2];
    NvOdmI2cStatus Error;    

    arr[0] = 0x0;
    arr[1] = 0x0;

    TransactionInfo.Address = 0x3A;
    TransactionInfo.Buf = arr;
    TransactionInfo.Flags = NVODM_I2C_IS_WRITE;
    TransactionInfo.NumBytes = 2;

    // Write dummy data to accelerometer

    do
    {
        Error = NvOdmI2cTransaction(hTouch->hOdmI2c,
                                    &TransactionInfo,
                                    1,
                                    hTouch->I2cClockSpeedKHz,
                                    TPK_I2C_TIMEOUT);
    } while (Error == NvOdmI2cStatus_Timeout);

    if (Error != NvOdmI2cStatus_Success)
    {
        //NvOdmOsDebugPrintf("error!\r\n");
        return NV_FALSE;
    }                        
    //NvOdmOsDebugPrintf("dummy!\r\n");
    return NV_TRUE;
}
#endif

static NvBool TPK_WriteRegister (TPK_TouchDevice* hTouch, NvU8 reg, NvU8 val)
{
    NvOdmI2cStatus Error;
    NvOdmI2cTransactionInfo TransactionInfo;
    NvU8 arr[2];
#if TPK_ADL340_WAR    
    // dummy write
    Error = NvAccDummyI2CSetRegs(hTouch);
#endif
    arr[0] = reg;
    arr[1] = val;
    
    TransactionInfo.Address = hTouch->DeviceAddr;
    TransactionInfo.Buf = arr;
    TransactionInfo.Flags = NVODM_I2C_IS_WRITE;
    TransactionInfo.NumBytes = 2;
    
    do
    {
        Error = NvOdmI2cTransaction(hTouch->hOdmI2c,
                                    &TransactionInfo,
                                    1,
                                    hTouch->I2cClockSpeedKHz,
                                    TPK_I2C_TIMEOUT);
    } while (Error == NvOdmI2cStatus_Timeout); 

    if (Error != NvOdmI2cStatus_Success)
    {
        NVODMTOUCH_PRINTF(("I2C Write Failure = %d (addr=0x%x, reg=0x%x, val=0x%0x)\n", Error, 
                           hTouch->DeviceAddr, reg, val));
        return NV_FALSE;
    }
    return NV_TRUE;
}

#define TPK_MAX_READS (((TPK_MAX_READ_BYTES+(TPK_MAX_PACKET_SIZE-1))/TPK_MAX_PACKET_SIZE))

static NvBool TPK_ReadRegisterOnce (TPK_TouchDevice* hTouch, NvU8 reg, NvU8* buffer, NvU32 len)
{
    NvOdmI2cStatus Error;
    NvOdmI2cTransactionInfo TransactionInfo[2 * TPK_MAX_READS];
    int reads = (len+(TPK_MAX_PACKET_SIZE-1))/TPK_MAX_PACKET_SIZE;
    int left = len;
    int i;
    
    NV_ASSERT(len <= TPK_MAX_READ_BYTES);
#if TPK_ADL340_WAR    
    // dummy write
    Error = NvAccDummyI2CSetRegs(hTouch);
#endif
    ////////////////////////////////////////////////////////////////////////////
    // For multi-byte reads, the TPK panel supports just sending the first
    // address and then keep reading registers (non-standard SMBus operation).
    // The limit for I2C packets is 8 bytes, so we read up to 8 bytes per
    // multi-byte read transaction.
    ////////////////////////////////////////////////////////////////////////////

    for (i = 0; i < reads; i++)
    {
        int ind = i*2;

        TransactionInfo[ind].Address = hTouch->DeviceAddr;
        TransactionInfo[ind].Buf = &reg;
        TransactionInfo[ind].Flags = NVODM_I2C_IS_WRITE;
        TransactionInfo[ind].NumBytes = 1;

        ind++;

        TransactionInfo[ind].Address = hTouch->DeviceAddr | 0x1;
        TransactionInfo[ind].Buf = buffer + i*TPK_MAX_PACKET_SIZE;
        TransactionInfo[ind].Flags = 0;
        TransactionInfo[ind].NumBytes =
            left > TPK_MAX_PACKET_SIZE ? TPK_MAX_PACKET_SIZE : left;
        
        left -= TPK_MAX_PACKET_SIZE;
    }
    
    do
    {
        Error = NvOdmI2cTransaction(hTouch->hOdmI2c,
                                    TransactionInfo,
                                    reads * 2,
                                    hTouch->I2cClockSpeedKHz,
                                    TPK_I2C_TIMEOUT);
    } while (Error == NvOdmI2cStatus_Timeout);

    if (Error != NvOdmI2cStatus_Success)
    {
        NVODMTOUCH_PRINTF(("I2C Read Failure = %d (addr=0x%x, reg=0x%x)\n", Error,
                           hTouch->DeviceAddr, reg));
        return NV_FALSE;
    }

    return NV_TRUE;
}

static NvBool TPK_ReadRegisterSafe (TPK_TouchDevice* hTouch, NvU8 reg, NvU8* buffer, NvU32 len)
{
    
    if (!TPK_ReadRegisterOnce(hTouch, reg, buffer, len))
        return NV_FALSE;


    return NV_TRUE;
}

static NvBool TPK_SetPage (TPK_TouchDevice* hTouch, NvU8 page)
{
#if TPK_PAGE_CHANGE_DELAY
    NvU32 SetPageDelayMs = 100; //Wait for 100 millisecond after change page
#endif

    if (!TPK_WRITE(hTouch, TPK_PAGE_SELECT, page)) return NV_FALSE;

#if TPK_PAGE_CHANGE_DELAY
    NvOdmOsSleepMS(SetPageDelayMs);
#endif
    
    return NV_TRUE;
}

#if TPK_CHECK_ERRORS
static void TPK_CheckError (TPK_TouchDevice* hTouch)
{
    NvU8 status;
    TPK_READ(hTouch, TPK_DEVICE_STATUS, &status, 1);
    if (status & 0x80)
    {
        NvU8 error;
        TPK_READ(hTouch, TPK_ERROR_STATUE, &error, 1);
        NvOdmOsDebugPrintf("Panel error %x %x\n", status, error);
    }
#if TPK_DUMP_REGISTER
    TPK_READ(hTouch, TPK_DEVICE_STATUS, &status, 1);
    NvOdmOsDebugPrintf("DeviceStatus(0x%x)=0x%x\n", TPK_DEVICE_STATUS, status);
    
    TPK_READ(hTouch, TPK_DEVICE_CONTROL, &status, 1);
    NvOdmOsDebugPrintf("DeviceControl(0x%x)=0x%x\n", TPK_DEVICE_CONTROL, status);
        
    TPK_READ(hTouch, TPK_INTR_ENABLE, &status, 1);
    NvOdmOsDebugPrintf("InterruptEnable(0x%x)=0x%x\n", TPK_INTR_ENABLE, status);
        
    TPK_READ(hTouch, TPK_ERROR_STATUE, &status, 1);
    NvOdmOsDebugPrintf("TPK_ERROR_STATUE(0x%x)=0x%x\n", TPK_ERROR_STATUE, status);
       
    TPK_READ(hTouch, TPK_INTR_STATUS, &status, 1);
    NvOdmOsDebugPrintf("InterruptStatus(0x%x)=0x%x\n", TPK_INTR_STATUS, status);
        
    TPK_READ(hTouch, TPK_DEVICE_COMMAND, &status, 1);
    NvOdmOsDebugPrintf("DeviceCommand(0x%x)=0x%x\n", TPK_DEVICE_COMMAND, status);
#endif      
}
#endif

static NvBool TPK_Configure (TPK_TouchDevice* hTouch)
{
    hTouch->SleepMode = 0x0;
    hTouch->SampleRate = 0; /* this forces register write */
    return TPK_SetSampleRate(&hTouch->OdmTouch, TPK_HIGH_SAMPLE_RATE);
}

static NvBool TPK_GetSample (TPK_TouchDevice* hTouch, NvOdmTouchCoordinateInfo* coord)
{
    NvU8 Finger0[6] = {0};
    NvU8 Finger1[6] = {0};
    NvU8 Relative[2] = {0};
    int status = 0;

    NVODMTOUCH_PRINTF(("TPK_GetSample+\n"));
    coord->fingerstate = NvOdmTouchSampleIgnore;

    if (!TPK_ReadRegisterOnce(hTouch, TPK_DATA_0, Finger0, 6))
        return NV_FALSE;
    else
        status = (Finger0[0] & 0x7);

    if (!TPK_ReadRegisterOnce(hTouch, TPK_DATA_0+6, Finger1, 6))
        return NV_FALSE;

    if (!TPK_ReadRegisterOnce(hTouch, TPK_DATA_0+12, Relative, 2))
        return NV_FALSE; 

    /* tell windows to ignore transitional finger count samples */
    coord->fingerstate = (status > 2) ? NvOdmTouchSampleIgnore : NvOdmTouchSampleValidFlag;
    coord->additionalInfo.Fingers = status;

    if (Finger0[0] & 0x8)
        // Bit 3 of status indicates a tap. Driver still doesn't expose
        // gesture capabilities. This is added more for testing of the support
        // in the hardware for gesture support.
    {
        coord->additionalInfo.Gesture = NvOdmTouchGesture_Tap;
        // NvOdmOsDebugPrintf("Detected the Tap gesture\n");
    }

    if (status)
    {
        /* always read first finger data, even if transitional */
        coord->fingerstate |= NvOdmTouchSampleDownFlag;

        coord->xcoord =
        coord->additionalInfo.multi_XYCoords[0][0] =
            (((NvU16)Finger0[2] & 0x1f) << 8) | (NvU16)Finger0[3];

        coord->ycoord =
        coord->additionalInfo.multi_XYCoords[0][1] =
            (((NvU16)Finger0[4] & 0x1f) << 8) | (NvU16)Finger0[5];

        coord->additionalInfo.width[0] = Finger0[0] >> 4;
        coord->additionalInfo.Pressure[0] = Finger0[1];

        /* only read second finger data if reported */
        if (status == 2)
        {
            coord->additionalInfo.multi_XYCoords[1][0] =
                (((NvU16)Finger1[2] & 0x1f) << 8) | (NvU16)Finger1[3];

            coord->additionalInfo.multi_XYCoords[1][1] =
            (((NvU16)Finger1[4] & 0x1f) << 8) | (NvU16)Finger1[5];

            /* these are not supported, zero out just in case */
            coord->additionalInfo.width[1] = 0;
            coord->additionalInfo.Pressure[1] = 0;
            if ( coord->additionalInfo.multi_XYCoords[1][0] <= 0 ||
                coord->additionalInfo.multi_XYCoords[1][0] >= hTouch->Caps.XMaxPosition ||
                coord->additionalInfo.multi_XYCoords[1][1] <= 0 || 
                coord->additionalInfo.multi_XYCoords[1][1] >= hTouch->Caps.YMaxPosition)
                coord->fingerstate = NvOdmTouchSampleIgnore;
#if TPK_REPORT_2ND_FINGER_DATA
            else
                NvOdmOsDebugPrintf("catch 2 fingers width=0x%x, X=%d, Y=%d, DeltaX=%d, DeltaY=%d\n",
                                    coord->additionalInfo.width[0],
                                    coord->additionalInfo.multi_XYCoords[1][0],
                                    coord->additionalInfo.multi_XYCoords[1][1],
                                    Relative[0], Relative[1]);
#endif
        }
    }
    else if (!hTouch->PrevFingers)
    {
        /* two successive 0 finger samples */
        coord->fingerstate = NvOdmTouchSampleIgnore;
    }

    hTouch->PrevFingers = status;
        
    NVODMTOUCH_PRINTF(("TPK_GetSample-\n"));
    return NV_TRUE;
}

static void InitOdmTouch (NvOdmTouchDevice* Dev)
{
    Dev->Close              = TPK_Close;
    Dev->GetCapabilities    = TPK_GetCapabilities;
    Dev->ReadCoordinate     = TPK_ReadCoordinate;
    Dev->EnableInterrupt    = TPK_EnableInterrupt;
    Dev->HandleInterrupt    = TPK_HandleInterrupt;
    Dev->GetSampleRate      = TPK_GetSampleRate;
    Dev->SetSampleRate      = TPK_SetSampleRate;
    Dev->PowerControl       = TPK_PowerControl;
    Dev->PowerOnOff         = TPK_PowerOnOff;
    Dev->GetCalibrationData = TPK_GetCalibrationData;
    Dev->OutputDebugMessage = NV_FALSE;
}

static void TPK_GpioIsr(void *arg)
{
    TPK_TouchDevice* hTouch = (TPK_TouchDevice*)arg;

    /* Signal the touch thread to read the sample. After it is done reading the
     * sample it should re-enable the interrupt. */
    NvOdmOsSemaphoreSignal(hTouch->hIntSema);            
}

NvBool TPK_ReadCoordinate (NvOdmTouchDeviceHandle hDevice, NvOdmTouchCoordinateInfo* coord)
{
    TPK_TouchDevice* hTouch = (TPK_TouchDevice*)hDevice;

#if TPK_BENCHMARK_SAMPLE    
    NvU32 time = NvOdmOsGetTimeMS();
#endif
    NVODMTOUCH_PRINTF(("GpioIst+\n"));

#if TPK_CHECK_ERRORS
    TPK_CheckError(hTouch);
#endif

    for (;;)
    {
        if (TPK_GetSample(hTouch, coord))
        {
            break;
        }

        /* If reading the data failed, the panel may be resetting itself.
           Poll device status register to find when panel is back up
           and reconfigure */
        for (;;)
        {
            NvU8 status;
            
            while (!TPK_READ(hTouch, TPK_DEVICE_STATUS, &status, 1))
            {
                NvOdmOsSleepMS(10);
            }

            /* if we have a panel error, force reset and wait for status again */
            if (status & 0x80)
            {
                TPK_WRITE(hTouch, TPK_DEVICE_COMMAND, 0x1);
                continue;
            }

            /* reconfigure panel, if failed start again */
            if (!TPK_Configure(hTouch))
                continue;

            /* re-enable interrupts for absolute data only */
            if (!TPK_WRITE(hTouch, TPK_INTR_ENABLE, 0x3))
                continue;

            break;
        }
    }

#if TPK_BENCHMARK_SAMPLE    
    NvOdmOsDebugPrintf("Touch sample time %d\n", NvOdmOsGetTimeMS() - time);
#endif
    
    return NV_TRUE;
}

void TPK_GetCapabilities (NvOdmTouchDeviceHandle hDevice, NvOdmTouchCapabilities* pCapabilities)
{
    TPK_TouchDevice* hTouch = (TPK_TouchDevice*)hDevice;
    *pCapabilities = hTouch->Caps;
}

NvBool TPK_PowerOnOff (NvOdmTouchDeviceHandle hDevice, NvBool OnOff)
{
    TPK_TouchDevice* hTouch = (TPK_TouchDevice*)hDevice;

    hTouch->hPmu = NvOdmServicesPmuOpen();

    if (!hTouch->hPmu)
    {
        NVODMTOUCH_PRINTF(("NvOdm Touch : NvOdmServicesPmuOpen Error \n"));
        return NV_FALSE;
    }
    
    if (OnOff != hTouch->PowerOn)
    {
        NvOdmServicesPmuVddRailCapabilities vddrailcap;
        NvU32 settletime;

        NvOdmServicesPmuGetCapabilities( hTouch->hPmu, hTouch->VddId, &vddrailcap);

        if(OnOff)
            NvOdmServicesPmuSetVoltage( hTouch->hPmu, hTouch->VddId, vddrailcap.requestMilliVolts, &settletime);
        else
            NvOdmServicesPmuSetVoltage( hTouch->hPmu, hTouch->VddId, NVODM_VOLTAGE_OFF, &settletime);

        if (settletime)
            NvOdmOsWaitUS(settletime); // wait to settle power

        hTouch->PowerOn = OnOff;

        if(OnOff)
            NvOdmOsSleepMS(TPK_POR_DELAY);
    }

    NvOdmServicesPmuClose(hTouch->hPmu);

    return NV_TRUE;
}

NvBool TPK_Open (NvOdmTouchDeviceHandle* hDevice)
{
    TPK_TouchDevice* hTouch;
    NvU32 i;
    NvU32 found = 0;
    NvU32 GpioPort = 0;
    NvU32 GpioPin = 0;
    NvU32 I2cInstance = 0;
    NvU8 Buf[4];
#if TPK_QUERY_SENSOR_RESOLUTION
    NvU8  sensorresolution = 0; //units per millimeter
#endif
#if TPK_SET_MAX_POSITION
    NvU32 SET_MAX_POSITION = 8191; //Max Position range from 0x0002 to 0x1fff
    NvU32 SENSOR_MAX_POSITION = 0;
#endif
    NvU8 TPKChipRevID;

    const NvOdmPeripheralConnectivity *pConnectivity = NULL;

    hTouch = NvOdmOsAlloc(sizeof(TPK_TouchDevice));
    if (!hTouch) return NV_FALSE;

    NvOdmOsMemset(hTouch, 0, sizeof(TPK_TouchDevice));

    /* set function pointers */
    InitOdmTouch(&hTouch->OdmTouch);

    pConnectivity = NvOdmPeripheralGetGuid(TPK_TOUCH_DEVICE_GUID);
    if (!pConnectivity)
    {
        NVODMTOUCH_PRINTF(("NvOdm Touch : pConnectivity is NULL Error \n"));
        goto fail;
    }

    if (pConnectivity->Class != NvOdmPeripheralClass_HCI)
    {
        NVODMTOUCH_PRINTF(("NvOdm Touch : didn't find any periperal in discovery query for touch device Error \n"));
        goto fail;
    }

    for (i = 0; i < pConnectivity->NumAddress; i++)
    {
        switch (pConnectivity->AddressList[i].Interface)
        {
            case NvOdmIoModule_I2c:
                hTouch->DeviceAddr = (pConnectivity->AddressList[i].Address << 1);
                I2cInstance = pConnectivity->AddressList[i].Instance;
                found |= 1;
                break;
            case NvOdmIoModule_Gpio:
                GpioPort = pConnectivity->AddressList[i].Instance;
                GpioPin = pConnectivity->AddressList[i].Address;
                found |= 2;
                break;
            case NvOdmIoModule_Vdd:
                hTouch->VddId = pConnectivity->AddressList[i].Address;
                found |= 4;
                break;
            default:
                break;
        }
    }

    if ((found & 3) != 3)
    {
        NVODMTOUCH_PRINTF(("NvOdm Touch : peripheral connectivity problem \n"));
        goto fail;
    }

    if ((found & 4) != 0)
    {
        if (NV_FALSE == TPK_PowerOnOff(&hTouch->OdmTouch, 1))
            goto fail;            
    }
    else
    {
        hTouch->VddId = 0xFF; 
    }

    hTouch->hOdmI2c = NvOdmI2cOpen(NvOdmIoModule_I2c, I2cInstance);
    if (!hTouch->hOdmI2c)
    {
        NVODMTOUCH_PRINTF(("NvOdm Touch : NvOdmI2cOpen Error \n"));
        goto fail;
    }

    hTouch->hGpio = NvOdmGpioOpen();

    if (!hTouch->hGpio)
    {        
        NVODMTOUCH_PRINTF(("NvOdm Touch : NvOdmGpioOpen Error \n"));
        goto fail;
    }

    hTouch->hPin = NvOdmGpioAcquirePinHandle(hTouch->hGpio, GpioPort, GpioPin);
    if (!hTouch->hPin)
    {
        NVODMTOUCH_PRINTF(("NvOdm Touch : Couldn't get GPIO pin \n"));
        goto fail;
    }

    NvOdmGpioConfig(hTouch->hGpio,
                    hTouch->hPin,
                    NvOdmGpioPinMode_InputData);

    /* set default capabilities */
    NvOdmOsMemcpy(&hTouch->Caps, &TPK_Capabilities, sizeof(NvOdmTouchCapabilities));

    /* set default I2C speed */
    hTouch->I2cClockSpeedKHz = TPK_I2C_SPEED_KHZ;
    
    /* disable interrupts */
    if (!TPK_WRITE(hTouch, TPK_INTR_ENABLE, 0))
        goto fail;

#if TPK_SET_MAX_POSITION
    if (!TPK_READ(hTouch, TPK_SENSOR_MAXPOSITION_0, Buf, 2)) goto fail;
    SENSOR_MAX_POSITION =  ((NvU32)Buf[0] << 8) | (NvU32)Buf[1];
    NVODMTOUCH_PRINTF(("Touch Max Postion = %d\n", SENSOR_MAX_POSITION));
    
    if (!TPK_WRITE(hTouch, TPK_SENSOR_MAXPOSITION_0, (NvU8)((SET_MAX_POSITION & 0x1fff) >> 8))) goto fail;
    if (!TPK_WRITE(hTouch, TPK_SENSOR_MAXPOSITION_1, (NvU8)(SET_MAX_POSITION & 0xff))) goto fail;   
    
    if (!TPK_READ(hTouch, TPK_SENSOR_MAXPOSITION_0, Buf, 2)) goto fail;
    SENSOR_MAX_POSITION =  ((NvU32)Buf[0] << 8) | (NvU32)Buf[1];
    NVODMTOUCH_PRINTF(("Touch Max Postion = %d\n", SENSOR_MAX_POSITION));
#endif

    /* get max positions */
    /* There is no SMBus Aliased Address to query max position, change page to 0x10 */ 
    if (!TPK_SetPage(hTouch, ((TPK_RMI_SENSOR_X_MAX_POSITION_0 >> 8) & 0xFF))) goto fail;

    if (!TPK_READ(hTouch, 0x04, Buf, 4)) goto fail;
       
    hTouch->Caps.XMaxPosition = ((NvU32)Buf[0] << 8) | (NvU32)Buf[1];
    hTouch->Caps.YMaxPosition = ((NvU32)Buf[2] << 8) | (NvU32)Buf[3];

#if TPK_QUERY_SENSOR_RESOLUTION
    /* get sensor resulution */
    /* There is no SMBus Aliased Address to query sensor resolution, change page to 0x10 */ 
    if (!TPK_SetPage(hTouch, ((TPK_RMI_SENSOR_RESOLUTION >> 8) & 0xFF))) goto fail;

    if (!TPK_READ(hTouch, TPK_RMI_SENSOR_RESOLUTION, &sensorresolution, 1)) goto fail;
    NVODMTOUCH_PRINTF(("Touch Sensor Resolution = %d\n", sensorresolution));
#endif
    /* change page back to 0x04 */
    if (!TPK_SetPage(hTouch, ((TPK_RMI_DATA_0 >> 8) & 0xFF))) goto fail;
    
    /* get chip revision id */
    if (!TPK_READ(hTouch, TPK_PRODUCT_INFO_QUERY_1, &TPKChipRevID, 1)) goto fail;

    hTouch->ChipRevisionId = (NvU32)TPKChipRevID;
    NVODMTOUCH_PRINTF(("Touch controller Revision ID = %d\n", hTouch->ChipRevisionId));

    /* boost I2c spped to 100Khz when it is new tpk chip */
    if (hTouch->ChipRevisionId == 0x02)
        hTouch->I2cClockSpeedKHz = 100;
    
    /* configure panel */
    if (!TPK_Configure(hTouch)) goto fail;

    *hDevice = &hTouch->OdmTouch;
    return NV_TRUE;

 fail:
    TPK_Close(&hTouch->OdmTouch);
    return NV_FALSE;
}

NvBool TPK_EnableInterrupt (NvOdmTouchDeviceHandle hDevice, NvOdmOsSemaphoreHandle hIntSema)
{
    TPK_TouchDevice* hTouch = (TPK_TouchDevice*)hDevice;
    NvOdmTouchCoordinateInfo coord;

    NV_ASSERT(hIntSema);
    
    /* can only be initialized once */
    if (hTouch->hGpioIntr || hTouch->hIntSema)
        return NV_FALSE;

    /* zero intr status */
    (void)TPK_GetSample(hTouch, &coord);        

    hTouch->hIntSema = hIntSema;    

    if (NvOdmGpioInterruptRegister(hTouch->hGpio, &hTouch->hGpioIntr,
        hTouch->hPin, NvOdmGpioPinMode_InputInterruptLow, TPK_GpioIsr,
        (void*)hTouch, TPK_DEBOUNCE_TIME_MS) == NV_FALSE)
    {
        return NV_FALSE;
    }

    if (!hTouch->hGpioIntr)
        return NV_FALSE;    

    /* enable interrupts -- only for absolute data */
    if (!TPK_WRITE(hTouch, TPK_INTR_ENABLE, 0x3))
    {
        NvOdmGpioInterruptUnregister(hTouch->hGpio, hTouch->hPin, hTouch->hGpioIntr);
        hTouch->hGpioIntr = NULL;
        return NV_FALSE;
    }
    
    return NV_TRUE;
}

NvBool TPK_HandleInterrupt(NvOdmTouchDeviceHandle hDevice)
{
    TPK_TouchDevice* hTouch = (TPK_TouchDevice*)hDevice;
    NvU32 pinValue;
    
    NvOdmGpioGetState(hTouch->hGpio, hTouch->hPin, &pinValue);
    if (!pinValue)
    {
        //interrupt pin is still LOW, read data until interrupt pin is released.
        return NV_FALSE;
    }
    else
        NvOdmGpioInterruptDone(hTouch->hGpioIntr);
    
    return NV_TRUE;
}

NvBool TPK_GetSampleRate (NvOdmTouchDeviceHandle hDevice, NvOdmTouchSampleRate* pTouchSampleRate)
{
    TPK_TouchDevice* hTouch = (TPK_TouchDevice*)hDevice;
    pTouchSampleRate->NvOdmTouchSampleRateHigh = 80;
    pTouchSampleRate->NvOdmTouchSampleRateLow = 40;
    pTouchSampleRate->NvOdmTouchCurrentSampleRate = (hTouch->SampleRate >> 1);
    return NV_TRUE;
}

NvBool TPK_SetSampleRate (NvOdmTouchDeviceHandle hDevice, NvU32 rate)
{
    TPK_TouchDevice* hTouch = (TPK_TouchDevice*)hDevice;

    if (rate != 0 && rate != 1)
        return NV_FALSE;
    
    rate = 1 << rate;
    
    if (hTouch->SampleRate == rate)
        return NV_TRUE;

    if (!TPK_WRITE(hTouch, TPK_DEVICE_CONTROL, (NvU8)((rate << 6) | hTouch->SleepMode)))
        return NV_FALSE;
    
    hTouch->SampleRate = rate;
    return NV_TRUE;
}


NvBool TPK_PowerControl (NvOdmTouchDeviceHandle hDevice, NvOdmTouchPowerModeType mode)
{
    TPK_TouchDevice* hTouch = (TPK_TouchDevice*)hDevice;
    NvU32 SleepMode;

    NV_ASSERT(hTouch->VddId != 0xFF);
    
    switch(mode)
    {
        case NvOdmTouch_PowerMode_0:
            SleepMode = 0x0;
            break;
        case NvOdmTouch_PowerMode_1:
        case NvOdmTouch_PowerMode_2:
        case NvOdmTouch_PowerMode_3:
            SleepMode = 0x03;
            break;
        default:
            return NV_FALSE;
    }

    if (hTouch->SleepMode == SleepMode)
        return NV_TRUE;
    
    if (!TPK_WRITE(hTouch, TPK_DEVICE_CONTROL, (NvU8)((hTouch->SampleRate << 6) | SleepMode)))
        return NV_FALSE;
    
    hTouch->SleepMode = SleepMode;    
    return NV_TRUE;
}

NvBool TPK_GetCalibrationData(NvOdmTouchDeviceHandle hDevice, NvU32 NumOfCalibrationData, NvS32* pRawCoordBuffer)
{
#if TPK_SCREEN_ANGLE
    //Portrait
    static const NvS32 RawCoordBuffer[] = {2054, 3624, 3937, 809, 3832, 6546, 453, 6528, 231, 890};
#else
    //Landscape
    static NvS32 RawCoordBuffer[] = {2054, 3624, 3832, 6546, 453, 6528, 231, 890, 3937, 809};
#endif

    if (NumOfCalibrationData*2 != (sizeof(RawCoordBuffer)/sizeof(NvS32)))
    {
        NVODMTOUCH_PRINTF(("WARNING: number of calibration data isn't matched\n"));
        return NV_FALSE;
    }
    
    NvOdmOsMemcpy(pRawCoordBuffer, RawCoordBuffer, sizeof(RawCoordBuffer));

    return NV_TRUE;
}


void TPK_Close (NvOdmTouchDeviceHandle hDevice)
{
    TPK_TouchDevice* hTouch = (TPK_TouchDevice*)hDevice;

    if (!hTouch) return;
        
    if (hTouch->hGpio)
    {
        if (hTouch->hPin)
        {
            if (hTouch->hGpioIntr)
                NvOdmGpioInterruptUnregister(hTouch->hGpio, hTouch->hPin, hTouch->hGpioIntr);

            NvOdmGpioReleasePinHandle(hTouch->hGpio, hTouch->hPin);
        }

        NvOdmGpioClose(hTouch->hGpio);
    }

    if (hTouch->hOdmI2c)
        NvOdmI2cClose(hTouch->hOdmI2c);

    NvOdmOsFree(hTouch);
}


