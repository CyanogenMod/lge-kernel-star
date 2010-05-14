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
 
/*  NVIDIA Tegra ODM Kit Sample Accelerometer Adaptation of the
 *  WinCE Accelerometer Driver
 */


#include "nvodm_accelerometer_adi340.h"
#include "nvodm_services.h"
#include "nvodm_query_discovery.h"
#include "nvos.h"

#define NV_ACCELEROMETER_REGISTER_RANGE 8
// When acc is put in horizontal, the max value from acc.
#define NV_ADI340_ACCELEROMETER_NORMAL_THRESHOLD 30
#define NV_ADI340_ACCELEROMETER_TAP_THRESHOLD 40
#define NV_ADI340_LOW_POWER_SAMPLERATE 3
#define NV_ADI340_FULL_RUN_SAMPLERATE  100
#define NV_ADI340_FORCE_FACTOR 1000
#define NV_ADI340_MAX_FORCE_IN_REG 128 // It indicates force register length. 
#define NV_DEBOUNCE_TIME_MS 0
//static NvU32 g_thresholdG_shadow = 70;

// For interrupt handle, set GPIO when an interrupt happens.
static void GpioInterruptHandler(void *arg);
NvBool NvAccelerometerI2COpen(NvOdmServicesI2cHandle* hI2CDevice, NvU32 id);
void   NvAccelerometerI2CClose(NvOdmServicesI2cHandle hI2CDevice);
NvBool NvAccelerometerI2CSetRegs(NvOdmAccelHandle hDevice, NvU8 offset, NvU8* value, NvU32 len);
NvBool NvAccelerometerI2CGetRegs(NvOdmAccelHandle hDevice, NvU8 offset, NvU8* value, NvU32 len);
NvBool NvAccelerometerConnectSemaphore(NvOdmAccelHandle hDevice);
void NvAccelerometerSetPowerRail(NvOdmServicesPmuHandle hPMUDevice, NvU32 Id, NvBool IsEnable);
void NvAccelerometerGetInterruptSouce(NvOdmAccelHandle hDevice, 
                                         NvOdmAccelIntType  *IntType,
                                         NvOdmAccelAxisType *IntMotionAxis,
                                         NvOdmAccelAxisType *IntTapAxis);

/*
 * Set accelerometer registers.
 * [in] attrib: The register flag. 
 * [out] info: The value to be set into the register of accelerometer.
 * Returns NV_TRUE if successful, or NV_FALSE otherwise.
 */
NvBool NvOdmAccelerometerSetParameter(NvOdmAccelHandle hDevice, NvU8 attrib, NvU32 info)
{
    // Because there are only 8 bits for one accelerometer register.
    NvU8 LocalInfo = 0;
    LocalInfo = (NvU8)(info);
    // Due to the register length, we only accept the lowest 8 bits.
    NvOdmOsMemcpy(&LocalInfo, &info, sizeof(NvU8));
    //NVODMACCELEROMETER_PRINTF("NV ODM ACCELEROMETER NvOdmAccelerometerSetParameter +++\n");
    switch (attrib)
    {
        case XLR_CTL:
            //NvOdmOsDebugPrintf("set XLR_CTL = 0x%x\n", LocalInfo);
            hDevice->RegsWrite( hDevice, XLR_CTL, &LocalInfo, 1);
            break;
        case XLR_INTCONTROL:
            //NvOdmOsDebugPrintf("set XLR_INTCONTROL = 0x%x\n", LocalInfo);
            hDevice->RegsWrite( hDevice, XLR_INTCONTROL, &LocalInfo, 1);
            break;
        case XLR_INTCONTROL2:
            //NvOdmOsDebugPrintf("set XLR_INTCONTROL2 = 0x%x\n", LocalInfo);
            hDevice->RegsWrite( hDevice, XLR_INTCONTROL2, &LocalInfo, 1);
             break;
        case XLR_THRESHG:
            //NVODMACCELEROMETER_PRINTF("set XLR_THRESHG = 0x%x\n", LocalInfo);
            hDevice->RegsWrite( hDevice, XLR_THRESHG, &LocalInfo, 1);
            break;
        case XLR_THRESHC:
            hDevice->RegsWrite( hDevice, XLR_THRESHC, &LocalInfo, 1);
            break;
        case XLR_OFSX:
            hDevice->RegsWrite( hDevice, XLR_OFSX, &LocalInfo, 1);
            break;
        case XLR_OFSY:
            hDevice->RegsWrite( hDevice, XLR_OFSY, &LocalInfo, 1);
            break;
        case XLR_OFSZ:
            hDevice->RegsWrite( hDevice, XLR_OFSZ, &LocalInfo, 1);
            break;
        case XLR_DUR:
            hDevice->RegsWrite( hDevice, XLR_DUR, &LocalInfo, 1);
            break;
        case XLR_LATENT:
            hDevice->RegsWrite( hDevice, XLR_LATENT, &LocalInfo, 1);
            break;
        case XLR_INTVL:
            hDevice->RegsWrite( hDevice, XLR_INTVL, &LocalInfo, 1);
            break;
        default:
            //NVODMACCELEROMETER_PRINTF("NV ODM ACCELEROMETER NvOdmAccelerometerSetParameter DONT SUPPORT SUCH ATTRIBUTE ---\n");
            return NV_FALSE;
    }
    //NVODMACCELEROMETER_PRINTF("NV ODM ACCELEROMETER NvOdmAccelerometerSetParameter ---\n");
    return NV_TRUE;
}



/*
 * Get acceleromter registers.
 * [in] attrib: The regsiter flag. 
 * [out] info: The value from register of accelerometer.
 * Returns NV_TRUE if successful, or NV_FALSE otherwise.
 */
NvBool NvOdmAccelerometerGetParameter(NvOdmAccelHandle hDevice, NvU8 attrib, NvU32* info)
{
    NvU8 LocalInfo = 0;
    NvS32 temp;

    //NVODMACCELEROMETER_PRINTF("NV ODM ACCELEROMETER NvOdmAccelerometerGetParameter +++\n");
    switch (attrib)
    {
        case XLR_DEVID:
            hDevice->RegsRead( hDevice, XLR_DEVID, &LocalInfo, 1);
            break;
        case XLR_WHOAMI:
            hDevice->RegsRead( hDevice, XLR_WHOAMI, &LocalInfo, 1);
            break;
        case XLR_STATUS:
            hDevice->RegsRead( hDevice, XLR_STATUS, &LocalInfo, 1);
            break;
        case XLR_INTSOURCE:
            hDevice->RegsRead( hDevice, XLR_INTSOURCE, &LocalInfo, 1);
            break;
        case XLR_CTL:
            hDevice->RegsRead( hDevice, XLR_CTL, &LocalInfo, 1);
            break;
        case XLR_INTCONTROL:
            hDevice->RegsRead( hDevice, XLR_INTCONTROL, &LocalInfo, 1);
            break;
        case XLR_INTCONTROL2:
            hDevice->RegsRead( hDevice, XLR_INTCONTROL2, &LocalInfo, 1);
            break;
        case XLR_DATAX:
            // Because it is a signed char.
            hDevice->RegsRead( hDevice, XLR_DATAX, &LocalInfo, 1);
            temp = (LocalInfo<128)?LocalInfo:(LocalInfo-256);
            NvOdmOsMemcpy(info, &temp, sizeof(NvU32));
            //NVODMACCELEROMETER_PRINTF("NV ODM ACCELEROMETER NvOdmAccelerometerGetParameter ---\n");
            return NV_TRUE;
        case XLR_DATAY:
            // Because it is a signed char.
            hDevice->RegsRead( hDevice, XLR_DATAY, &LocalInfo, 1);
            temp = (LocalInfo<128)?LocalInfo:(LocalInfo-256);
            NvOdmOsMemcpy(info, &temp, sizeof(NvU32));
            //NVODMACCELEROMETER_PRINTF("NV ODM ACCELEROMETER NvOdmAccelerometerGetParameter ---\n");
            return NV_TRUE;
        case XLR_DATAZ:
            // Because it is a signed char.
            hDevice->RegsRead( hDevice, XLR_DATAZ, &LocalInfo, 1);
            temp = (LocalInfo<128)?LocalInfo:(LocalInfo-256);
            NvOdmOsMemcpy(info, &temp, sizeof(NvU32));
            //NVODMACCELEROMETER_PRINTF("NV ODM ACCELEROMETER NvOdmAccelerometerGetParameter ---\n");
            return NV_TRUE;
        case XLR_MOREINFO:
            hDevice->RegsRead( hDevice, XLR_MOREINFO, &LocalInfo, 1);
            break;
        case XLR_THRESHG:
            hDevice->RegsRead( hDevice, XLR_THRESHG, &LocalInfo, 1);
            break;
        case XLR_THRESHC:
            hDevice->RegsRead( hDevice, XLR_THRESHC, &LocalInfo, 1);
            break;
        case XLR_OFSX:
            hDevice->RegsRead( hDevice, XLR_OFSX, &LocalInfo, 1);
            break;
        case XLR_OFSY:
            hDevice->RegsRead( hDevice, XLR_OFSY, &LocalInfo, 1);
            break;
        case XLR_OFSZ:
            hDevice->RegsRead( hDevice, XLR_OFSZ, &LocalInfo, 1);
            break;
        case XLR_DUR:
            hDevice->RegsRead( hDevice, XLR_DUR, &LocalInfo, 1);
            break;
        case XLR_LATENT:
            hDevice->RegsRead( hDevice, XLR_LATENT, &LocalInfo, 1);
            break;
        case XLR_INTVL:
            hDevice->RegsRead( hDevice, XLR_INTVL, &LocalInfo, 1);
            break;
        case XLR_SCALE:
            hDevice->RegsRead( hDevice, XLR_CTL, &LocalInfo, 1);            
            if ((LocalInfo&(0x01)) == 0)
            {
                LocalInfo = 2;
            }
            else
            {
                LocalInfo = 8;
            }
            break;
        case XLR_ROTATE:
            *info = 1;
            break;
        case XLR_GYRO:
            *info = 0;
            break;
        default:
            //NVODMACCELEROMETER_PRINTF("NV ODM ACCELEROMETER NvOdmAccelerometerGetParameter DONT SUPPORT SUCH ATTRIBUTE ---\n");
            return NV_FALSE;
    }
    *info = LocalInfo;
    //NVODMACCELEROMETER_PRINTF("NV ODM ACCELEROMETER NvOdmAccelerometerGetParameter ---\n");
    return NV_TRUE;
}

void NvAccelerometerSetPowerRail(NvOdmServicesPmuHandle hPMUDevice, NvU32 Id, NvBool IsEnable)
{
    NvOdmServicesPmuVddRailCapabilities vddrailcap;
    NvU32 settletime;

    if (hPMUDevice)
    {       
        NvOdmServicesPmuGetCapabilities(hPMUDevice, Id, &vddrailcap);
        if (IsEnable)
        {
            NvOdmServicesPmuSetVoltage(hPMUDevice, Id, vddrailcap.requestMilliVolts, &settletime);
        }
        else
        {
            NvOdmServicesPmuSetVoltage(hPMUDevice, Id, vddrailcap.MinMilliVolts, &settletime);
        }
        NvOdmOsWaitUS(settletime);  // wait to settle power
    }
}


/*
 * Get interrupt type and source.
 */
void NvAccelerometerGetInterruptSouce(NvOdmAccelHandle hDevice, 
                                              NvOdmAccelIntType  *IntType,
                                              NvOdmAccelAxisType *IntMotionAxis,
                                              NvOdmAccelAxisType *IntTapAxis)
{
    NvU32    reg_val;

    NV_ASSERT(hDevice != 0);
    NV_ASSERT(IntType != 0);
    NV_ASSERT(IntMotionAxis != 0); 
    NV_ASSERT(IntTapAxis != 0); 

    *IntType = NvOdmAccelInt_None;
    *IntMotionAxis = NvOdmAccelAxis_None;
    *IntTapAxis = NvOdmAccelAxis_None;
    
    if(NULL != hDevice)
    {
        NvOdmAccelerometerGetParameter(hDevice, XLR_INTSOURCE, &reg_val);
        if(reg_val & XLR_INTSOURCE_X_COM_MASK)
        {
            *IntType |= NvOdmAccelInt_MotionThreshold;
            *IntMotionAxis |= NvOdmAccelAxis_X;
        }
        if(reg_val & XLR_INTSOURCE_Y_COM_MASK)
        {
            *IntType |= NvOdmAccelInt_MotionThreshold;
            *IntMotionAxis |= NvOdmAccelAxis_Y;
        }
        if(reg_val & XLR_INTSOURCE_Z_COM_MASK)
        {
            *IntType |= NvOdmAccelInt_MotionThreshold;
            *IntMotionAxis |= NvOdmAccelAxis_Z;
        }
        if(reg_val & XLR_INTSOURCE_X_TAP_MASK)
        {
            *IntType |= NvOdmAccelInt_TapThreshold;
            *IntTapAxis |= NvOdmAccelAxis_X;
        }
        if(reg_val & XLR_INTSOURCE_Y_TAP_MASK)
        {
            *IntType |= NvOdmAccelInt_TapThreshold;
            *IntTapAxis |= NvOdmAccelAxis_Y;
        }
        if(reg_val & XLR_INTSOURCE_Z_TAP_MASK)
        {
            *IntType |= NvOdmAccelInt_TapThreshold;
            *IntTapAxis |= NvOdmAccelAxis_Z;
        }
   }
   //NvOdmOsDebugPrintf("IntType =%d, IntAxis = %d\n", *IntType, *IntAxis);
}


static void 
GpioInterruptHandler(void *arg)
{
    NvOdmGpioPinMode mode; 
    
    NvU32 pinValue;
    NvOdmAccelHandle hDevice =  (NvOdmAccelHandle)arg;
    //NvOdmOsSemaphoreHandle s = (NvOdmOsSemaphoreHandle)arg;
    
    NvOdmGpioGetState(hDevice->hGpioINT, hDevice->hPinINT, &pinValue); 
    if (pinValue == 1)
    {
        mode = NvOdmGpioPinMode_InputInterruptLow;
    }
    else
    {
        mode = NvOdmGpioPinMode_InputInterruptHigh;
    }
    
    NvOdmGpioConfig(hDevice->hGpioINT, hDevice->hPinINT, mode);
    
    if (pinValue == 1) 
    {
        NvOdmOsSemaphoreSignal(hDevice->SemaphoreForINT);
    }
    NvOdmGpioInterruptDone(hDevice->hGpioInterrupt);
    return;
}

/*
 * Connect semaphore with interrupt pins according to your configuration.
 */
NvBool NvAccelerometerConnectSemaphore(NvOdmAccelHandle hDevice)
{
    NvOdmGpioPinMode mode; 
    NvOdmInterruptHandler callback = (NvOdmInterruptHandler)GpioInterruptHandler;
         
    hDevice->hGpioINT = (NvOdmServicesGpioHandle)NvOdmGpioOpen();
    if(!(hDevice->hGpioINT))
    {
        //NVODMACCELEROMETER_PRINTF("NvOdm Accelerometer : NvOdmGpioOpen Error \n");
        return NV_FALSE;
    }

    hDevice->hPinINT = NvOdmGpioAcquirePinHandle(hDevice->hGpioINT, 
                                                 hDevice->GPIOPortINT, 
                                                 hDevice->GPIOPinINT);

    hDevice->SemaphoreForINT = NvOdmOsSemaphoreCreate(0);

    if(!(hDevice->SemaphoreForINT))
    {
        //NVODMACCELEROMETER_PRINTF("NvOdm Accelerometer : NvOdmOsSemaphoreCreate Error \n");
        NvOdmGpioClose(hDevice->hGpioINT);
        return NV_FALSE;
    }
          
    mode = NvOdmGpioPinMode_InputInterruptHigh;
    
    if (NvOdmGpioInterruptRegister(hDevice->hGpioINT, &hDevice->hGpioInterrupt,
        hDevice->hPinINT, mode, callback, hDevice, NV_DEBOUNCE_TIME_MS) == NV_FALSE)
    {
        return NV_FALSE;
    }

    if(!(hDevice->hGpioInterrupt))
    {
        //NVODMACCELEROMETER_PRINTF("NvOdm Accelerometer : NvOdmGpioInterruptRegister Error \n");
        NvOdmGpioClose(hDevice->hGpioINT);
        NvOdmOsSemaphoreDestroy(hDevice->SemaphoreForINT);
        return NV_FALSE;
    }
    
    return NV_TRUE;
}

/*
 * Initialize I2C for accelerometer.
 */
NvBool NvAccelerometerI2COpen(NvOdmServicesI2cHandle* hI2CDevice, NvU32 id)
{
    // Open I2C handle.
    *hI2CDevice = NvOdmI2cOpen(NvOdmIoModule_I2c, id);
    if (*hI2CDevice == NULL)
    {
        return NV_FALSE;
    }
    return NV_TRUE;
}


/*
 * De-initialize I2C for accelerometer.
 */
void NvAccelerometerI2CClose(NvOdmServicesI2cHandle hI2CDevice)
{
    // Close I2C handle.
    if(NULL != hI2CDevice)
    {
        NvOdmI2cClose(hI2CDevice);
    }
}


/*
 * Write I2C register function.
 * offset[Input]: I2C register offset of accelerometer.
 * value[Input]: register value you will write.
 * len[Input]: requested bytes.
 * Returns NV_TRUE if successful, or NV_FALSE otherwise.
 */
NvBool NvAccelerometerI2CSetRegs(NvOdmAccelHandle hDevice, NvU8 offset, NvU8* value, NvU32 len)
{
    NvOdmI2cTransactionInfo TransactionInfo;

    if( (NULL == hDevice) || (NULL == value) || (len > I2C_ACCELRATOR_PACKET_SIZE-1 ))
    {
        //NVODMACCELEROMETER_PRINTF("NvOdmI2c Set Regs Failed, max size is %d bytes\n", I2C_ACCELRATOR_PACKET_SIZE-1);
        return NV_FALSE;
    }
    
    NvOdmOsMemset(s_WriteBuffer, 0, sizeof(s_WriteBuffer));
    s_WriteBuffer[0] = offset;
    NvOdmOsMemcpy(&s_WriteBuffer[1], value, len);

    TransactionInfo.Address = hDevice->nDevAddr;
    TransactionInfo.Buf = s_WriteBuffer;
    TransactionInfo.Flags = NVODM_I2C_IS_WRITE;
    TransactionInfo.NumBytes = len+1;

    // Write the accelerator offset (from where data is to be read).
    if(NvOdmI2cStatus_Success != NvOdmI2cTransaction(hDevice->hOdmI2C, &TransactionInfo, 1, 400, I2C_ACCELRATOR_TRANSACTION_TIMEOUT))
    {
        return NV_FALSE;
    };    
        
    return NV_TRUE;
}

/*
 * Read I2C register function.
 * offset[Input]: I2C register offset of accelerometer.
 * value[Output]: Fegister value you get.
 * len[Input]: Requested bytes.
 * Returns NV_TRUE if successful, or NV_FALSE otherwise.
 */

NvBool NvAccelerometerI2CGetRegs(NvOdmAccelHandle hDevice, NvU8 offset, NvU8* value, NvU32 len)
{
    NvOdmI2cTransactionInfo TransactionInfo;

    if( (NULL == hDevice) || (NULL == value) || (len > I2C_ACCELRATOR_PACKET_SIZE-1 ))
    {
        //NVODMACCELEROMETER_PRINTF("NvOdmI2c Get Regs Failed, max size is %d bytes\n", I2C_ACCELRATOR_PACKET_SIZE-1);
        return NV_FALSE;
    }
    
    NvOdmOsMemset(s_WriteBuffer, 0, sizeof(s_WriteBuffer));
    s_WriteBuffer[0] = offset;

    TransactionInfo.Address = hDevice->nDevAddr;
    TransactionInfo.Buf = s_WriteBuffer;
    TransactionInfo.Flags = NVODM_I2C_IS_WRITE;
    TransactionInfo.NumBytes = 1;

    // Write the accelerometor offset (from where data is to be read).
    if(NvOdmI2cStatus_Success != NvOdmI2cTransaction(hDevice->hOdmI2C, &TransactionInfo, 1, 400, I2C_ACCELRATOR_TRANSACTION_TIMEOUT))
    {
        return NV_FALSE;
    };
  
    NvOdmOsMemset(s_ReadBuffer, 0, sizeof(s_ReadBuffer));    
    s_ReadBuffer[0] = 0;

    TransactionInfo.Address = (hDevice->nDevAddr| 0x1);
    TransactionInfo.Buf = s_ReadBuffer;
    TransactionInfo.Flags = 0;
    TransactionInfo.NumBytes = len;

    //Read the data from the eeprom at the specified offset
    if(NvOdmI2cStatus_Success != NvOdmI2cTransaction(hDevice->hOdmI2C, &TransactionInfo, 1, 400, I2C_ACCELRATOR_TRANSACTION_TIMEOUT))
    {
        return NV_FALSE;
    };    

    NvOdmOsMemcpy(value, &s_ReadBuffer[0], len);
        
    return NV_TRUE;
} 

//-----------------------------------------------------------------
//--------------------------------New API--------------------------
//-----------------------------------------------------------------
NvBool
NvOdmAccelOpen(NvOdmAccelHandle* hDevice)
{
    NvU32    test_val;
    NvU32 i;
    NvBool foundGpio = NV_FALSE, foundI2cModule = NV_FALSE;
    const NvOdmPeripheralConnectivity *pConnectivity;
    NvOdmAccelHandle  hAccel;

    hAccel = NvOdmOsAlloc(sizeof(NvOdmAccel));
    if (hAccel == NULL)
    {
        //NVODMACCELEROMETER_PRINTF("Error Allocating NvOdmAccel. \n");
        return NV_FALSE;
    }
    NvOdmOsMemset(hAccel, 0, sizeof(NvOdmAccel));

    hAccel->hPmu = NULL;
    hAccel->hOdmI2C =  NULL;
    hAccel->nBusType = NV_ACCELEROMETER_BUS_I2C;
    
    // Chip init cfg info here, here just a sample for common interrupt now!
    // This part will move to a configuration table later.
    // Start here.
    // Only enable common interrupt
    // Enable common and single tap at the same time.
    hAccel->CtrlRegsList[0].RegAddr = XLR_CTL; //0x12
    hAccel->CtrlRegsList[0].RegValue = 0x20;
    hAccel->CtrlRegsList[1].RegAddr = XLR_INTCONTROL; //0x13
    hAccel->CtrlRegsList[1].RegValue = 0xF3;  // modify so that sw is compatible
    hAccel->CtrlRegsList[2].RegAddr = XLR_INTCONTROL2; //0x14
    hAccel->CtrlRegsList[2].RegValue = 0xe0;
    hAccel->CtrlRegsList[3].RegAddr = XLR_THRESHG; //0x1C
    hAccel->CtrlRegsList[3].RegValue = NV_ADI340_ACCELEROMETER_NORMAL_THRESHOLD;
    hAccel->CtrlRegsList[4].RegAddr = XLR_OFSX; //0x1E
    hAccel->CtrlRegsList[4].RegValue = 0;
    hAccel->CtrlRegsList[5].RegAddr = XLR_OFSY; //0x1F
    hAccel->CtrlRegsList[5].RegValue = 0;
    hAccel->CtrlRegsList[6].RegAddr = XLR_OFSZ; //0x20
    hAccel->CtrlRegsList[6].RegValue = 0;
    hAccel->CtrlRegsList[7].RegAddr = XLR_THRESHC; //0x1D
    hAccel->CtrlRegsList[7].RegValue = NV_ADI340_ACCELEROMETER_TAP_THRESHOLD;
    hAccel->CtrlRegsList[8].RegAddr = XLR_DUR; //0x21
    hAccel->CtrlRegsList[8].RegValue = 0x40;
    hAccel->CtrlRegsList[9].RegAddr = XLR_LATENT; //0x22
    hAccel->CtrlRegsList[9].RegValue = 0xff;
    hAccel->CtrlRegsList[10].RegAddr = XLR_INTVL; //0x23
    hAccel->CtrlRegsList[10].RegValue = 0;
    hAccel->CtrlRegsList[11].RegAddr = XLR_INTCONTROL2; //0x14
    hAccel->CtrlRegsList[11].RegValue = 0xe1;
    hAccel->CtrlRegsList[12].RegAddr = XLR_INTCONTROL2; //0x14
    hAccel->CtrlRegsList[12].RegValue = 0xe0;
    hAccel->nLength = 13;
    // Stop here.
    // Info of accelerometer with current setting.
    hAccel->Caption.MaxForceInGs = 2000;
    hAccel->Caption.MaxTapTimeDeltaInUs = 255;
    hAccel->Caption.NumMotionThresholds = 1;
    hAccel->Caption.SupportsFreefallInt = 0;
    hAccel->Caption.MaxSampleRate = 100;
    hAccel->Caption.MinSampleRate = 3;
    hAccel->PowerState = NvOdmAccelPower_Fullrun;
    hAccel->AxisXMapping = NvOdmAccelAxis_X;
    hAccel->AxisXDirection = 1;
    hAccel->AxisYMapping = NvOdmAccelAxis_Y;
    hAccel->AxisYDirection = 1;
    hAccel->AxisZMapping = NvOdmAccelAxis_Z;
    hAccel->AxisZDirection = -1;
    
    hAccel->hPmu = NvOdmServicesPmuOpen();
    if (!hAccel->hPmu)
    {
        //NVODMACCELEROMETER_PRINTF("NvOdmServicesPmuOpen Error \n");
        goto error;
    }
    
    pConnectivity = (NvOdmPeripheralConnectivity*)NvOdmPeripheralGetGuid(NV_ODM_GUID('a','c','c','e','l','e','r','o'));
    if (!pConnectivity)
    {
        NvOdmOsDebugPrintf("NvOdmPeripheralGetGuid doesn't detect accelerometer device\n");
        goto error;
    }
 
    if(pConnectivity->Class != NvOdmPeripheralClass_Other)
    {
        goto error;
    }
        
    for( i = 0; i < pConnectivity->NumAddress; i++)
    {
        switch(pConnectivity->AddressList[i].Interface)
        {
            case NvOdmIoModule_I2c:
                hAccel->I2CChannelId = pConnectivity->AddressList[i].Instance;
                hAccel->nDevAddr = (NvU8)pConnectivity->AddressList[i].Address;
                foundI2cModule = NV_TRUE;
                break;
            case NvOdmIoModule_Gpio:
                hAccel->GPIOPortINT = pConnectivity->AddressList[i].Instance;
                hAccel->GPIOPinINT = pConnectivity->AddressList[i].Address;
                foundGpio = NV_TRUE;
                break;
            case NvOdmIoModule_Vdd:
                hAccel->VddId = pConnectivity->AddressList[i].Address;
                // Power on accelerometer according to Vddid
                NvAccelerometerSetPowerRail(hAccel->hPmu, hAccel->VddId, NV_TRUE);
                break;
            default:
                break;
        }
    }

    if(foundGpio != NV_TRUE || foundI2cModule != NV_TRUE)
    {
        //NVODMACCELEROMETER_PRINTF("Accelerometer : didn't find any periperal in discovery query for touch device Error \n");
        goto error;
    }

    
    // Set up I2C bus.
    if(NV_FALSE == NvAccelerometerI2COpen(&hAccel->hOdmI2C, hAccel->I2CChannelId))
    {
        goto error;
    };
    hAccel->RegsRead  = NvAccelerometerI2CGetRegs;
    hAccel->RegsWrite = NvAccelerometerI2CSetRegs;
    
    NvOdmAccelerometerGetParameter(hAccel, XLR_WHOAMI, &test_val);
    if(XLR_IDNUM != test_val)
    {
        goto error;
    }
    
    NvOdmAccelerometerGetParameter(hAccel, XLR_DEVID, &test_val);
    if (test_val == XLR_NEWCHIPID)
    {
        // This chip is ADXL345
        //NvOdmOsDebugPrintf("This chip is ADXL345!!!\n");
        hAccel->CtrlRegsList[4].RegValue = 0x0A; // offset X
        hAccel->CtrlRegsList[5].RegValue = 0x0B; // offset Y
        hAccel->CtrlRegsList[6].RegValue = 0x14; // offset Z
    }
    //NVODMACCELEROMETER_PRINTF("ID is 0x%x\n", test_val);
    
    /* We don't know the reset state of the accelerometer. So, program the
     * accelerometer to disable generation of interrupts.
     *  
     *  Write to INTCONTROL register to disable genetration of the interrupts.
     *  Write to INTCONTROL2 to clear the already latched interrupts.
     */
    NvOdmAccelerometerSetParameter(hAccel, XLR_ATTR_INTCONTROL, 0x0);
    NvOdmAccelerometerSetParameter(hAccel, XLR_ATTR_INTCONTROL2, 0x1);
    if(NV_FALSE == NvAccelerometerConnectSemaphore(hAccel))
    {
        goto error;
    }
    
    //init accelerometer
    for(i=0; i<hAccel->nLength; i++)
    {
            NvOdmAccelerometerSetParameter(hAccel,
                                           hAccel->CtrlRegsList[i].RegAddr, 
                                           hAccel->CtrlRegsList[i].RegValue);
    }
    // Set up event.
    
    //NvOdmAccelerometerGetParameter(XLR_SCALE, hAccel);
    *hDevice = hAccel;
    return NV_TRUE;
    error:
        // Release all of resources requested.
        if(NULL != hAccel)
        {
            NvAccelerometerSetPowerRail(hAccel->hPmu, hAccel->VddId, NV_FALSE);
            NvOdmServicesPmuClose(hAccel->hPmu);
            hAccel->hPmu = NULL;
            NvAccelerometerI2CClose(hAccel->hOdmI2C);
            hAccel->hOdmI2C = NULL;
            NvOdmOsFree(hAccel);
            *hDevice = NULL;
        }
        return NV_FALSE;
}

void
NvOdmAccelClose(NvOdmAccelHandle hDevice)
{
    if(NULL != hDevice)
    {
        if(NULL != hDevice->SemaphoreForINT &&
           NULL != hDevice->hGpioINT &&
           NULL != hDevice->hPinINT &&
           NULL != hDevice->hGpioInterrupt)
        {
            NvOdmGpioInterruptUnregister(hDevice->hGpioINT,
                                         hDevice->hPinINT,
                                         hDevice->hGpioInterrupt);
            NvOdmOsSemaphoreDestroy(hDevice->SemaphoreForINT);   
            NvOdmGpioReleasePinHandle(hDevice->hGpioINT, hDevice->hPinINT);
            NvOdmGpioClose(hDevice->hGpioINT);
        }
        NvAccelerometerI2CClose(hDevice->hOdmI2C);

        // Power off accelermeter
        NvAccelerometerSetPowerRail(hDevice->hPmu, hDevice->VddId, NV_FALSE);
        if (hDevice->hPmu)
        {
            //NvAccelerometerSetPowerOn(0);
            NvOdmServicesPmuClose(hDevice->hPmu);
        }
        
        return;
    }
}

/*
 * After setting the force threshold, we should remove all of interrupt flag
 * that may be left from the last threshold.
 */
NvBool
NvOdmAccelSetIntForceThreshold(NvOdmAccelHandle  hDevice,
                               NvOdmAccelIntType IntType,
                               NvU32             IntNum,
                               NvU32             Threshold)
{
    // At first, we need to translate the threshold to register value for acc.
    // The first step is get current work range.
    NvU32 uMaxThreshold;
    NvU32 uThresholdToRegister;
    NvU32 temp;

    
    NV_ASSERT(NULL != hDevice);
    uMaxThreshold = hDevice->Caption.MaxForceInGs;
    if(Threshold > uMaxThreshold)
    {
        //NVODMACCELEROMETER_PRINTF("Current accelerometer supprt max threshold is %d g\n", uMaxThreshold);
        Threshold = uMaxThreshold;
    }

    uThresholdToRegister = (NvU8)(Threshold*((1<<(NV_ACCELEROMETER_REGISTER_RANGE-1))-1)/uMaxThreshold);
    
    // ADI345 can't receive interrupt while threshold = 0
    if (uThresholdToRegister == 0)
        uThresholdToRegister = 1;
    
    // We only enable 2 interrupt pins, INT2 for Motion, INT1 for single tap.
    switch(IntType)
    {
        case NvOdmAccelInt_MotionThreshold:
            //NvOdmOsDebugPrintf("Current motion setting threshold is %d \n", uThresholdToRegister);
            //NvOdmOsDebugPrintf("the motion threshold program into reg is %d \n", uThresholdToRegister);
            NvOdmAccelerometerSetParameter(hDevice, XLR_THRESHG, uThresholdToRegister);
            break;
        case NvOdmAccelInt_TapThreshold:
            //NvOdmOsDebugPrintf("Current tap setting threshold is %d \n", uThresholdToRegister);
            //NvOdmOsDebugPrintf("the tap threshold program into reg is %d \n", uThresholdToRegister);
            NvOdmAccelerometerSetParameter(hDevice, XLR_THRESHC, uThresholdToRegister);
            break;
        default:
            //NVODMACCELEROMETER_PRINTF("Do not support such Interrupt!\n");
            return NV_FALSE;
    }

    // Clear interrupt flag.
    NvOdmAccelerometerGetParameter(hDevice, XLR_INTCONTROL2, &temp); 
    temp |= XLR_INTCONTROL2_CLR_INT;
    NvOdmAccelerometerSetParameter(hDevice, XLR_INTCONTROL2, temp);
    temp &= XLR_INTCONTROL2_CLR_INT_MASK;
    NvOdmAccelerometerSetParameter(hDevice, XLR_INTCONTROL2, temp);
    return NV_TRUE;
}

/*
 * After setting the time threshold, we should remove all of interrupt flag
 * that may be left from the last threshold.
 */
NvBool
NvOdmAccelSetIntTimeThreshold(NvOdmAccelHandle  hDevice,
                              NvOdmAccelIntType IntType,
                              NvU32             IntNum,
                              NvU32             Threshold)
{
    NvU32 temp;
    NV_ASSERT(NULL != hDevice);
    //its due adi340 limitation.
    if(Threshold > 0xff)
    {
        //NVODMACCELEROMETER_PRINTF("The max threshold support is 255 ms for adi340\n");
        return NV_FALSE;
    }
    //NvOdmOsDebugPrintf("The threshold is %d\n", Threshold);
    switch(IntType)
    {
        case NvOdmAccelInt_TapThreshold:
            NvOdmAccelerometerSetParameter(hDevice, XLR_DUR, Threshold);
            NvOdmAccelerometerSetParameter(hDevice, XLR_INTVL, 0);
            NvOdmAccelerometerSetParameter(hDevice, XLR_LATENT, 0);
            break;
        default:
            //NVODMACCELEROMETER_PRINTF("Do not need set time threshold for such Interrupt!\n");
            return NV_FALSE;
    }

    // Clear interrupt flag.
    NvOdmAccelerometerGetParameter(hDevice, XLR_INTCONTROL2, &temp); 
    temp |= XLR_INTCONTROL2_CLR_INT;
    NvOdmAccelerometerSetParameter(hDevice, XLR_INTCONTROL2, temp);
    temp &= XLR_INTCONTROL2_CLR_INT_MASK;
    NvOdmAccelerometerSetParameter(hDevice, XLR_INTCONTROL2, temp);
    return NV_TRUE;
}

/*
 * After enable/disable threshold, we should remove all of interrupt flag
 * that may be left from that last threshold.
 */
NvBool
NvOdmAccelSetIntEnable(NvOdmAccelHandle  hDevice,
                       NvOdmAccelIntType  IntType,
                       NvOdmAccelAxisType IntAxis,
                       NvU32              IntNum,
                       NvBool             Toggle)
{
    NvU32 uTemp = 0;
    NV_ASSERT(NULL != hDevice);
    
    switch(IntType)
    {
        case NvOdmAccelInt_MotionThreshold:
            NvOdmAccelerometerGetParameter(hDevice, XLR_INTCONTROL, &uTemp);
            //NvOdmOsDebugPrintf("INTCONTROL is 0x%x g\n", uTemp);
            uTemp |= XLR_INTCONTROL_COM_INT_ENABLE;
            switch(IntAxis)
            {
                case NvOdmAccelAxis_X:
                {
                    if(Toggle == NV_TRUE)
                    {
                        uTemp |= XLR_INTCONTROL_COM_SRC_X;
                    }
                    else
                    {
                        uTemp &= XLR_INTCONTROL_COM_SRC_X_MASK;
                    }
                    break;
                }
                case NvOdmAccelAxis_Y:
                {
                    if(Toggle == NV_TRUE)
                    {
                        uTemp |= XLR_INTCONTROL_COM_SRC_Y;
                    }
                    else
                    {
                        uTemp &= XLR_INTCONTROL_COM_SRC_Y_MASK;
                    }
                    break;
                }
                case NvOdmAccelAxis_Z:
                {
                    if(Toggle == NV_TRUE)
                    {
                        uTemp |= XLR_INTCONTROL_COM_SRC_Z;
                    }
                    else
                    {
                        uTemp &= XLR_INTCONTROL_COM_SRC_Z_MASK;
                    }
                    break;
                }
                case NvOdmAccelAxis_All:
                {
                    if(Toggle == NV_TRUE)
                    {
                        uTemp |= XLR_INTCONTROL_COM_SRC_X;
                        uTemp |= XLR_INTCONTROL_COM_SRC_Y;
                        uTemp |= XLR_INTCONTROL_COM_SRC_Z;
                    }
                    else
                    {
                        uTemp &= XLR_INTCONTROL_COM_SRC_X_MASK;
                        uTemp &= XLR_INTCONTROL_COM_SRC_Y_MASK;
                        uTemp &= XLR_INTCONTROL_COM_SRC_Z_MASK;
                    }
                    break;
                }
                default:
                    return NV_FALSE;
            }
            NvOdmAccelerometerSetParameter(hDevice, XLR_INTCONTROL, uTemp);
            break;
        case NvOdmAccelInt_TapThreshold:
            NvOdmAccelerometerGetParameter(hDevice, XLR_INTCONTROL, &uTemp);
            //NvOdmOsDebugPrintf("INTCONTROL is 0x%x \n", uTemp);
            uTemp |= XLR_INTCONTROL_TAP_INT_ENABLE;
            NvOdmAccelerometerSetParameter(hDevice, XLR_INTCONTROL, uTemp);
            NvOdmAccelerometerGetParameter(hDevice, XLR_INTCONTROL2, &uTemp);
            //NvOdmOsDebugPrintf("INTCONTROL2 is 0x%x \n", uTemp);
            switch(IntAxis)
            {
                case NvOdmAccelAxis_X:
                {
                    if(Toggle == NV_TRUE)
                    {
                        uTemp |= XLR_INTCONTROL2_TAP_SRC_X;
                    }
                    else
                    {
                        uTemp &= XLR_INTCONTROL2_TAP_SRC_X_MASK;
                    }
                        break;
                }
                case NvOdmAccelAxis_Y:
                {
                    if(Toggle == NV_TRUE)
                    {
                        uTemp |= XLR_INTCONTROL2_TAP_SRC_Y;
                    }
                    else
                    {
                        uTemp &= XLR_INTCONTROL2_TAP_SRC_Y_MASK;
                    }
                        break;
                }
                case NvOdmAccelAxis_Z:
                {
                    if(Toggle == NV_TRUE)
                    {
                        uTemp |= XLR_INTCONTROL2_TAP_SRC_Z;
                    }
                    else
                    {
                        uTemp &= XLR_INTCONTROL2_TAP_SRC_Z_MASK;
                    }
                        break;
                }
                case NvOdmAccelAxis_All:
                {
                    if(Toggle == NV_TRUE)
                    {
                        uTemp |= XLR_INTCONTROL2_TAP_SRC_X;
                        uTemp |= XLR_INTCONTROL2_TAP_SRC_Y;
                        uTemp |= XLR_INTCONTROL2_TAP_SRC_Z;
                    }
                    else
                    {
                        uTemp &= XLR_INTCONTROL2_TAP_SRC_X_MASK;
                        uTemp &= XLR_INTCONTROL2_TAP_SRC_Y_MASK;
                        uTemp &= XLR_INTCONTROL2_TAP_SRC_Z_MASK;
                    }
                        break;
                }
                default:
                    return NV_FALSE;
            }
            NvOdmAccelerometerSetParameter(hDevice, XLR_INTCONTROL2, uTemp);
            break;
        default:
            //NVODMACCELEROMETER_PRINTF("Do not support such Interrupt!\n");
            return NV_FALSE;
    }

    // Clear interrupt flag.
    NvOdmAccelerometerGetParameter(hDevice, XLR_INTCONTROL2, &uTemp); 
    uTemp |= XLR_INTCONTROL2_CLR_INT;
    NvOdmAccelerometerSetParameter(hDevice, XLR_INTCONTROL2, uTemp);
    uTemp &= XLR_INTCONTROL2_CLR_INT_MASK;
    NvOdmAccelerometerSetParameter(hDevice, XLR_INTCONTROL2, uTemp);
    return NV_TRUE;
}


void
NvOdmAccelWaitInt(NvOdmAccelHandle    hDevice,
                          NvOdmAccelIntType  *IntType,
                          NvOdmAccelAxisType *IntMotionAxis,
                          NvOdmAccelAxisType *IntTapAxis)
{
    NvU32  temp;
    NV_ASSERT(NULL != hDevice);
    NV_ASSERT(NULL != IntType);
    NV_ASSERT(NULL != IntMotionAxis);
    NV_ASSERT(NULL != IntTapAxis);
    
    NvOdmOsSemaphoreWait( hDevice->SemaphoreForINT);
    
    NvAccelerometerGetInterruptSouce( hDevice , IntType, IntMotionAxis, IntTapAxis);
    //NvOdmOsDebugPrintf("Captured interrupt!!!\n");
    NvOdmAccelerometerGetParameter(hDevice, XLR_INTCONTROL2, &temp); 
    temp |= XLR_INTCONTROL2_CLR_INT;
    NvOdmAccelerometerSetParameter(hDevice, XLR_INTCONTROL2, temp);
    temp &= XLR_INTCONTROL2_CLR_INT_MASK;
    NvOdmAccelerometerSetParameter(hDevice, XLR_INTCONTROL2, temp);

    //This is a WAR for ADI340 to prevent I2C register from unstable state
    NvOdmAccelerometerGetParameter(hDevice, XLR_INTCONTROL, &temp);
    //NvOdmOsDebugPrintf("XLR_INTCONTROL is 0x%x\n", temp);
    NvOdmAccelerometerGetParameter(hDevice, XLR_THRESHG, &temp);
    //NvOdmOsDebugPrintf("XLR_THRESHG is 0x%x\n", temp);
    NvOdmAccelerometerGetParameter(hDevice, XLR_INTSOURCE, &temp);
    //NvOdmOsDebugPrintf("XLR_INTSOURCE is 0x%x\n", temp);
    return ;
}


void NvOdmAccelSignal(NvOdmAccelHandle hDevice)
{
    NvOdmOsSemaphoreSignal(hDevice->SemaphoreForINT);
}

NvBool
NvOdmAccelGetAcceleration(NvOdmAccelHandle hDevice,
                          NvS32           *AccelX,
                          NvS32           *AccelY,
                          NvS32           *AccelZ)
{
    NvS32 data = 0;
    NV_ASSERT(NULL != hDevice);
    NV_ASSERT(NULL != AccelX);
    NV_ASSERT(NULL != AccelY);
    NV_ASSERT(NULL != AccelZ);
    
    //fix error for adi340 i2c bug. XLR_OFSZ will be set to 0xA0 randomly.
    NvOdmAccelerometerSetParameter(hDevice, XLR_OFSZ, 0);

    NvOdmAccelerometerGetParameter(hDevice, XLR_DATAX, (NvU32*)&data);
    //NvOdmOsDebugPrintf("DATAX is %d , after normalization is ", data);
    data = (((NvS32)(hDevice->Caption.MaxForceInGs))*data+(NvS32)(NV_ADI340_MAX_FORCE_IN_REG/2))/
             (NvS32)NV_ADI340_MAX_FORCE_IN_REG;
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
    //NvOdmOsDebugPrintf("%d \n", data);
    NvOdmAccelerometerGetParameter(hDevice, XLR_DATAY, (NvU32*)&data);
    //NvOdmOsDebugPrintf("DATAY is %d , after normalization is ", data);
    data = (((NvS32)(hDevice->Caption.MaxForceInGs))*data+(NvS32)(NV_ADI340_MAX_FORCE_IN_REG/2))/
             (NvS32)NV_ADI340_MAX_FORCE_IN_REG;
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
    //NvOdmOsDebugPrintf("%d \n", data);
    NvOdmAccelerometerGetParameter(hDevice, XLR_DATAZ, (NvU32*)&data);
    //NvOdmOsDebugPrintf("DATAZ is %d , after normalization is ", data);
    data = (((NvS32)(hDevice->Caption.MaxForceInGs))*data+(NvS32)(NV_ADI340_MAX_FORCE_IN_REG/2))/
             (NvS32)NV_ADI340_MAX_FORCE_IN_REG;
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
    //NvOdmOsDebugPrintf("%d \n", data);
    return NV_TRUE;
}

NvOdmAccelerometerCaps
NvOdmAccelGetCaps(NvOdmAccelHandle hDevice)
{
    NV_ASSERT(NULL != hDevice);
    
    return hDevice->Caption;
}

NvBool
NvOdmAccelSetSampleRate(NvOdmAccelHandle hDevice, NvU32 SampleRate)
{
    NvU32 PowerFlag = 0;
    NV_ASSERT(NULL != hDevice);
    NvOdmAccelerometerGetParameter(hDevice, XLR_CTL, &PowerFlag);
    if(SampleRate > NV_ADI340_LOW_POWER_SAMPLERATE)////3 is the info get from datasheet
    {
        //if(NvOdmAccelPower_Fullrun != hDevice->PowerState)
        {
                PowerFlag &= XLR_CTL_POWER_MASK;
                PowerFlag |= XLR_CTL_FULL_RUN;
                PowerFlag &= XLR_CTL_MODE_MASK;
                PowerFlag |= XLR_CTL_MEASURE_MODE;
                NvOdmAccelerometerSetParameter(hDevice, XLR_CTL, PowerFlag);
                hDevice->PowerState = NvOdmAccelPower_Fullrun;
        }
    }
    else
    {
        //if(NvOdmAccelPower_Low != hDevice->PowerState)
        {
                PowerFlag &= XLR_CTL_POWER_MASK;
                PowerFlag |= XLR_CTL_LOW_POWER;
                PowerFlag &= XLR_CTL_MODE_MASK;
                PowerFlag |= XLR_CTL_MEASURE_MODE;
                NvOdmAccelerometerSetParameter(hDevice, XLR_CTL, PowerFlag);
                hDevice->PowerState = NvOdmAccelPower_Low;
        }
    }
    return NV_TRUE;
}

NvBool
NvOdmAccelGetSampleRate(NvOdmAccelHandle hDevice, NvU32 *pSampleRate)
{
    NvU32 SampleFlag = 0;
    NV_ASSERT(NULL != hDevice);
    NvOdmAccelerometerGetParameter(hDevice, XLR_CTL, &SampleFlag);
    
    if((SampleFlag&XLR_CTL_LOW_POWER))
    {
        *pSampleRate = NV_ADI340_LOW_POWER_SAMPLERATE;
    }
    else
    {
        *pSampleRate = NV_ADI340_FULL_RUN_SAMPLERATE;
    }
    return NV_TRUE;
}

NvBool
NvOdmAccelSetPowerState(NvOdmAccelHandle hDevice, NvOdmAccelPowerType PowerState)
{
    NvU32 PowerFlag = 0;
    NV_ASSERT(NULL != hDevice);
    NvOdmAccelerometerGetParameter(hDevice, XLR_CTL, &PowerFlag);
    switch(PowerState)
    {
        case NvOdmAccelPower_Fullrun:
            if(NvOdmAccelPower_Fullrun != hDevice->PowerState)
            {
                PowerFlag &= XLR_CTL_POWER_MASK;
                PowerFlag |= XLR_CTL_FULL_RUN;
                PowerFlag &= XLR_CTL_MODE_MASK;
                PowerFlag |= XLR_CTL_MEASURE_MODE;
                NvOdmAccelerometerSetParameter(hDevice, XLR_CTL, PowerFlag);
                hDevice->PowerState = NvOdmAccelPower_Fullrun;
            }
            break;
        case NvOdmAccelPower_Low:
            if(NvOdmAccelPower_Low != hDevice->PowerState)
            {
                PowerFlag &= XLR_CTL_POWER_MASK;
                PowerFlag |= XLR_CTL_LOW_POWER;
                PowerFlag &= XLR_CTL_MODE_MASK;
                PowerFlag |= XLR_CTL_MEASURE_MODE;
                NvOdmAccelerometerSetParameter(hDevice, XLR_CTL, PowerFlag);
                hDevice->PowerState = NvOdmAccelPower_Low;
            }
            break;
        case NvOdmAccelPower_Standby:
            if(NvOdmAccelPower_Standby != hDevice->PowerState)
            {
                PowerFlag &= XLR_CTL_MODE_MASK;
                PowerFlag |= XLR_CTL_STANDBY_MODE;
                NvOdmAccelerometerSetParameter(hDevice, XLR_CTL, PowerFlag);
                hDevice->PowerState = NvOdmAccelPower_Standby;
            }
            break;
        case NvOdmAccelPower_Off:
            //the implementation should consider real board connection
            //sometimes we need use acc's interrupt to resum the whole
            //system, so the power are alway supplied.
            return NV_FALSE;
        default:
            return NV_FALSE;
    }
    return NV_TRUE;
}

