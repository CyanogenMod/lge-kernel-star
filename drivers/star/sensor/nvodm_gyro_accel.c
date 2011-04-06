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

/*  NVIDIA Tegra ODM Kit Sample GyroAccel Adaptation of the
 *  WinCE GyroAccel Driver
 */

#include "nvodm_gyro_accel.h"
#include "nvodm_services.h"
#include "nvodm_query_discovery.h"
#include "nvos.h"
#include <nvodm_gyroscope_accel.h>
#include <linux/delay.h>
#include <linux/kernel.h>

#define NV_DEBOUNCE_TIME_MS 	0
#define DEBUG_LOG             	0
#define DEBUG_NV_ACCEL         	0
#define TIMEOUT 5

extern int reboot;

// For interrupt handle, set GPIO when an interrupt happens.
static void GpioInterruptHandler(void *arg);
NvBool NvGyroAccelI2COpen(NvOdmServicesI2cHandle* hI2CDevice, NvU32 id);
void   NvGyroAccelI2CClose(NvOdmServicesI2cHandle hI2CDevice);
NvBool NvGyroAccelI2CSetRegs(NvOdmGyroAccelHandle hDevice, NvU8 offset, NvU8* value, NvU32 len);
NvBool NvGyroAccelI2CGetRegs(NvOdmGyroAccelHandle hDevice, NvU8 offset, NvU8* value, NvU32 len);
NvBool NvGyroAccelConnectSemaphore(NvOdmGyroAccelHandle hDevice);
void NvGyroAccelSetPowerRail(NvOdmServicesPmuHandle hPMUDevice, NvU32 Id, NvBool IsEnable);
void NvGyroAccelGetInterruptSouce(NvOdmGyroAccelHandle hDevice,
		NvOdmGyroAccelIntType  *IntType,
		NvOdmGyroAccelAxisType *IntMotionAxis);
void NvOdmResetI2C(NvOdmGyroAccelHandle hDevice); 

NvU32 Accel_PRail;

/*
 * Set accelerometer registers.
 * [in] attrib: The register flag.
 * [out] info: The value to be set into the register of accelerometer.
 * Returns NV_TRUE if successful, or NV_FALSE otherwise.
 */
NvBool NvOdmGyroAccelSetParameter(NvOdmGyroAccelHandle hDevice, NvU8 attrib, NvU32 info)
{
	// Because there are only 8 bits for one accelerometer register.
	NvU8 LocalInfo = 0;
	LocalInfo = (NvU8)(info);
	// Due to the register length, we only accept the lowest 8 bits.
	NvOdmOsMemcpy(&LocalInfo, &info, sizeof(NvU8));
#if DEBUG_NV_ACCEL
	printk(" ## MPU3050 : %s attrib %x  =>  ", __FUNCTION__, attrib) ;
#endif

	switch (attrib) {
		/* case KIONIX_ACCEL_I2C_CTRL_REG1:
		   hDevice->RegsWrite( hDevice, KIONIX_ACCEL_I2C_CTRL_REG1, &LocalInfo, 1);
#if DEBUG_NV_ACCEL
printk("[%s] 1:  \n "__FUNCTION__) ;
#endif
break;*/
		default:
			printk("NV ODM ACCELEROMETER NvOdmGyroAccelSetParameter DONT SUPPORT SUCH ATTRIBUTE ---\n");
			return NV_FALSE;
	}
	return NV_TRUE;
}

/*
 * Get acceleromter registers.
 * [in] attrib: The regsiter flag.
 * [out] info: The value from register of accelerometer.
 * Returns NV_TRUE if successful, or NV_FALSE otherwise.
 */
NvBool NvOdmGyroAccelGetParameter(NvOdmGyroAccelHandle hDevice, NvU8 attrib, NvU32* info)
{
	NvU8 LocalInfo = 0;
#if 1
	printk(" ## MPU3050 : NvOdmGyroAccelGetParameter\n ") ;
	printk(" ## MPU3050 : attrib=%x => ", attrib) ;
#endif

	switch (attrib) {
#if 1
		/* case KIONIX_ACCEL_I2C_WHO_AM_I:
		   hDevice->RegsRead( hDevice, KIONIX_ACCEL_I2C_WHO_AM_I, &LocalInfo, 1);

#if DEBUG_NV_ACCEL
printk("[%s] 8:  \n "__FUNCTION__) ;
#endif
break;    */
		default:
			printk("NV ODM MPU3050  NvOdmGyroAccelGetParameter DONT SUPPORT SUCH ATTRIBUTE ---\n");
			return NV_FALSE;
#endif
	}
#if 1
	printk("LocalInfo : %x/  info : %x/ *info : %d/  \n", LocalInfo, info, *info);
#endif

	*info = LocalInfo;

#if 1
	printk(" ## MPU3050 : [%s:%d] \n",__FUNCTION__, __LINE__) ;
	printk("LocalInfo : %x \n", LocalInfo);
#endif
	//NVODMACCELEROMETER_PRINTF("NV ODM ACCELEROMETER NvOdmGyroAccelGetParameter ---\n");
	return NV_TRUE;

}

void NvGyroAccelSetPowerRail(NvOdmServicesPmuHandle hPMUDevice, NvU32 Id, NvBool IsEnable)
{
	NvOdmServicesPmuVddRailCapabilities vddrailcap;
	NvU32 settletime;
	printk(" ## MPU3050 : 3 \n") ;

	Accel_PRail = Id;

	if (hPMUDevice) {
		NvOdmServicesPmuGetCapabilities(hPMUDevice, Id, &vddrailcap);
		if (IsEnable) {
			NvOdmServicesPmuSetVoltage(hPMUDevice, Id, vddrailcap.requestMilliVolts, &settletime);
		} else {
			NvOdmServicesPmuSetVoltage(hPMUDevice, Id, NVODM_VOLTAGE_OFF, &settletime);
		}

		if (settletime)
			NvOdmOsWaitUS(settletime);  // wait to settle power
	}
	NvOdmServicesPmuClose(hPMUDevice);
}


/*
 * Get interrupt type and source.
 */
void NvGyroAccelGetInterruptSouce(NvOdmGyroAccelHandle hDevice,
		NvOdmGyroAccelIntType  *IntType,
		NvOdmGyroAccelAxisType *IntMotionAxis)
{
	NvU32    reg_val ;

	NV_ASSERT(hDevice != 0);
	NV_ASSERT(IntType != 0);
	NV_ASSERT(IntMotionAxis != 0);

	*IntType = NvOdmGyroAccelInt_None;
	*IntMotionAxis = NvOdmGyroAccelAxis_None;
#if 0

	if(NULL != hDevice)
	{
		NvOdmGyroAccelGetParameter(hDevice, KXTF9_I2C_INT_SRC_REG2, &reg_val);
#if DEBUG_LOG
		printk(" ## MPU3050 : 4 \n") ;
		printk(" ## MPU3050 : MPU3050_I2C_INT_SRC_REG2 = %x ## \n", reg_val) ;
#endif
		if( reg_val & 0x01 )   // TPS : Screen Rotation
		{
			*IntType |= NvOdmGyroAccelInt_MotionThreshold;
		}
	}
#endif

}

static void
GpioInterruptHandler(void *arg)
{
	NvOdmGpioPinMode mode;

	NvU32 pinValue = 0;
	NvOdmGyroAccelHandle hDevice =  (NvOdmGyroAccelHandle)arg;
	printk(" ## MPU3050 : 5 \n") ;

	NvOdmGpioGetState(hDevice->hGpioINT, hDevice->hPinINT, &pinValue);

	if (pinValue == 1) {
		mode = NvOdmGpioPinMode_InputInterruptLow;
	} else {
		mode = NvOdmGpioPinMode_InputInterruptHigh;
	}

	NvOdmGpioConfig(hDevice->hGpioINT, hDevice->hPinINT, mode);

	if (pinValue == 1) {
		NvOdmOsSemaphoreSignal(hDevice->SemaphoreForINT);
	}
	NvOdmGpioInterruptDone(hDevice->hGpioInterrupt);

	return;
}

/*
 * Connect semaphore with interrupt pins according to your configuration.
 */

NvBool NvGyroAccelConnectSemaphore(NvOdmGyroAccelHandle hDevice)
{
	NvOdmGpioPinMode mode;
	NvOdmInterruptHandler callback = (NvOdmInterruptHandler)GpioInterruptHandler;
	printk(" ## MPU3050 : NvGyroAccelConnectSemaphore \n") ;
	hDevice->hGpioINT = (NvOdmServicesGpioHandle)NvOdmGpioOpen();

	if (!(hDevice->hGpioINT)) {
		printk("## NvOdm GyroAccel : NvOdmGpioOpen Error ##  \n");
		return NV_FALSE;
	}

	hDevice->hPinINT = NvOdmGpioAcquirePinHandle(hDevice->hGpioINT,
			hDevice->GPIOPortINT,
			hDevice->GPIOPinINT);

	hDevice->SemaphoreForINT = NvOdmOsSemaphoreCreate(0);

	if (!(hDevice->SemaphoreForINT)) {
		printk("## NvOdm GyroAccel : NvOdmOsSemaphoreCreate Error ## \n");
		NvOdmGpioClose(hDevice->hGpioINT);
		return NV_FALSE;
	}

	mode = NvOdmGpioPinMode_InputInterruptHigh;

	if (NvOdmGpioInterruptRegister(hDevice->hGpioINT, &hDevice->hGpioInterrupt,
				hDevice->hPinINT, mode, callback, hDevice, NV_DEBOUNCE_TIME_MS) == NV_FALSE)
	{
		printk("NvOdm GyroAccel : cannot register interrupt.\n") ;
		return NV_FALSE;
	}

	if (!(hDevice->hGpioInterrupt)) {
		NvOdmGpioClose(hDevice->hGpioINT);
		NvOdmOsSemaphoreDestroy(hDevice->SemaphoreForINT);
		return NV_FALSE;
	}

	return NV_TRUE;
}

/*
 * Initialize I2C for accelerometer.
 */
NvBool NvGyroAccelI2COpen(NvOdmServicesI2cHandle* hI2CDevice, NvU32 id)
{
	// Open I2C handle.

	*hI2CDevice  =  NvOdmI2cPinMuxOpen(NvOdmIoModule_I2c, 1, NvOdmI2cPinMap_Config2);
	printk(" ## MPU3050 : NvGyroAccelI2COpen \n") ;

	if (*hI2CDevice == NULL) {
		printk(" ## MPU3050 NvGyroAccelI2COpen Error ##\n");
		return NV_FALSE;
	}

	return NV_TRUE;
}

/*
 * De-initialize I2C for accelerometer.
 */
void NvGyroAccelI2CClose(NvOdmServicesI2cHandle hI2CDevice)
{
	// Close I2C handle.
	printk(" ## MPU3050 :  NvGyroAccelI2CClose \n") ;
	if (NULL != hI2CDevice) {
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

NvBool NvGyroAccelI2CSetRegs(NvOdmGyroAccelHandle hDevice, NvU8 offset, NvU8* value, NvU32 len)
{
	int i;  
        NvOdmI2cStatus i2c_status = NvOdmI2cStatus_Timeout;
	NvOdmI2cTransactionInfo TransactionInfo;

	if ((NULL == hDevice) || (NULL == value) || (len > I2C_GYROACCEL_PACKET_SIZE-1)) {
		return NV_FALSE;
	}

	for(i = 0; i < TIMEOUT && i2c_status != NvOdmI2cStatus_Success; i++)
        {
	    NvOdmOsMemset(s_WriteBuffer, 0, sizeof(s_WriteBuffer));
	    s_WriteBuffer[0] = offset;
	    NvOdmOsMemcpy(&s_WriteBuffer[1], value, len);

	    TransactionInfo.Address = hDevice->nDevAddr;
	    TransactionInfo.Buf = s_WriteBuffer;
	    TransactionInfo.Flags = NVODM_I2C_IS_WRITE;
	    TransactionInfo.NumBytes = len+1;

	    i2c_status = NvOdmI2cTransaction(hDevice->hOdmI2C, &TransactionInfo, 1, 400, I2C_GYROACCEL_TRANSACTION_TIMEOUT);

            if ((i % 2) == 1)
                mdelay(1);
	}
	
	if (i2c_status != NvOdmI2cStatus_Success) {
	    printk(" ## MPU3050 _ Give up!! NvGyroAccelI2CSetRegs failed: register %d \n", offset);

           //reboot sensors
           reboot = 1;
		
            return NV_FALSE;
	}

	return NV_TRUE;
}

/*
 * Read I2C register function.
 * offset[Input]: I2C register offset of accelerometer.
 * value[Output]: Fegister value you get.
 * len[Input]: Requested bytes.
 * Returns NV_TRUE if successful, or NV_FALSE otherwise.
 */
NvBool NvGyroAccelI2CGetRegs(NvOdmGyroAccelHandle hDevice, NvU8 offset, NvU8* value, NvU32 len)
{
        int i;
        NvOdmI2cStatus i2c_status = NvOdmI2cStatus_Timeout;
	NvOdmI2cTransactionInfo TransactionInfo[2];

	if ((NULL == hDevice) || (NULL == value) || (len > I2C_GYROACCEL_PACKET_SIZE-1)) {
		printk("NvOdmI2c Get Regs Failed, max size is %d bytes\n", I2C_GYROACCEL_PACKET_SIZE-1);
		return NV_FALSE;
	}

        for (i = 0; i < TIMEOUT && i2c_status != NvOdmI2cStatus_Success; i++)
        {
	    NvOdmOsMemset(s_WriteBuffer, 0, sizeof(s_WriteBuffer));
	    s_WriteBuffer[0] = offset;

	    TransactionInfo[0].Address = hDevice->nDevAddr;
	    TransactionInfo[0].Buf = s_WriteBuffer;
	    TransactionInfo[0].Flags = NVODM_I2C_IS_WRITE;
	    TransactionInfo[0].NumBytes = 1;

	    NvOdmOsMemset(s_ReadBuffer, 0, sizeof(s_ReadBuffer));

	    TransactionInfo[1].Address = (hDevice->nDevAddr| 0x1);
	    TransactionInfo[1].Buf = s_ReadBuffer;
	    TransactionInfo[1].Flags = 0;
	    TransactionInfo[1].NumBytes = len;

	    i2c_status = NvOdmI2cTransaction(hDevice->hOdmI2C, TransactionInfo, 2, 400, I2C_GYROACCEL_TRANSACTION_TIMEOUT);
            if ((i % 2) == 1)
                mdelay(1);
        }

	if (i2c_status != NvOdmI2cStatus_Success) {
	    printk(" ## MPU3050 _ Give up!! NvGyroAccelI2CSetRegs failed: register %d \n", offset);

           //reboot sensors
           reboot = 1;
		
            return NV_FALSE;
	}

        NvOdmOsMemcpy(value, &s_ReadBuffer[0], len);
	return NV_TRUE;
}

//-----------------------------------------------------------------
//--------------------------------New API--------------------------
//-----------------------------------------------------------------
NvBool
NvOdmGyroAccelOpen(NvOdmGyroAccelHandle* hDevice)
{
	NvU32 i;
	NvBool foundGpio = NV_FALSE, foundI2cModule = NV_FALSE;
	const NvOdmPeripheralConnectivity *pConnectivity;
	NvOdmGyroAccelHandle  hGyro;
	NvU32    reg_val = 0 ;
#if 1
	printk(" ## MPU3050 : [NvOdmGyroOpen:%d] \n",__LINE__) ;
#endif

	hGyro = NvOdmOsAlloc(sizeof(NvOdmGyro));
	if (hGyro == NULL) {
		printk("Error Allocating NvOdmAccel. \n");
		return NV_FALSE;
	}
	NvOdmOsMemset(hGyro, 0, sizeof(NvOdmGyro));

	hGyro->hPmu = NULL;
	hGyro->hOdmI2C =  NULL;

	hGyro->hPmu = NvOdmServicesPmuOpen();
	if (!hGyro->hPmu) {
		printk("NvOdmServicesPmuOpen Error \n");
		goto error;
	}

	pConnectivity = (NvOdmPeripheralConnectivity*)NvOdmPeripheralGetGuid(NV_ODM_GUID('g','y','r','o','s','c','o','p'));
	if (!pConnectivity) {
		printk("NvOdmPeripheralGetGuid doesn't detect gyro_accel device\n");
		goto error;
	}

	if(pConnectivity->Class != NvOdmPeripheralClass_Other) {
		goto error;
	}

	for (i = 0; i < pConnectivity->NumAddress; i++) {
		switch(pConnectivity->AddressList[i].Interface) {
			case NvOdmIoModule_I2c:
				hGyro->I2CChannelId = pConnectivity->AddressList[i].Instance;
				hGyro->nDevAddr = (pConnectivity->AddressList[i].Address << 1);
				foundI2cModule = NV_TRUE;
				foundGpio = NV_TRUE; //test
#if 1
				printk("## MPU3050 I2CChannelId = %x. ## \n", hGyro->I2CChannelId);
				printk("## MPU3050 i2c address = %x. ## \n", hGyro->nDevAddr);
#endif
				break;
				/*
				   case NvOdmIoModule_Gpio:
				   hGyro->GPIOPortINT = pConnectivity->AddressList[i].Instance;
				   hGyro->GPIOPinINT = pConnectivity->AddressList[i].Address;
				   foundGpio = NV_TRUE;
#if 1
printk("## MPU3050 GPIOPortINT = %x. ## \n",hGyro->GPIOPortINT);
printk("## MPU3050 GPIOPinINT = %x. ## \n", hGyro->GPIOPinINT);
#endif
break;*/
			case NvOdmIoModule_Vdd:
				hGyro->VddId = pConnectivity->AddressList[i].Address;
#if 1
				printk("## MPU3050 NvOdmIoModule_VddId = %x. ## \n", hGyro->VddId);
#endif
				// Power on accelerometer according to Vddid
				NvGyroAccelSetPowerRail(hGyro->hPmu, hGyro->VddId, NV_TRUE);
				break;
			default:
				break;
		}
	}

	if (foundGpio != NV_TRUE || foundI2cModule != NV_TRUE) {
		printk("GyroAccel : didn't find any periperal in discovery query for touch device Error \n");
		goto error;
	}

	// Set up I2C bus.
	if (NV_FALSE == NvGyroAccelI2COpen(&hGyro->hOdmI2C, hGyro->I2CChannelId)) {
		printk("GyroAccel : NvGyroAccelI2COpen Error \n");
		goto error;
	};
	printk(" ##1## GyroAccel : NvGyroAccelI2COpen check1 \n");
	hGyro->RegsRead = NvGyroAccelI2CGetRegs;
	hGyro->RegsWrite = NvGyroAccelI2CSetRegs;
	printk(" ##2## GyroAccel : NvGyroAccelI2COpen check2 \n");
	/*
		if(NV_FALSE == NvGyroAccelConnectSemaphore(hGyro))
		{
		printk("GyroAccel : NvGyroAccelConnectSemaphore Error \n");

		goto error;
		}
		*/

	*hDevice = hGyro;
	return NV_TRUE;
error:
	// Release all of resources requested.
	if (NULL != hGyro) {
		NvGyroAccelSetPowerRail(hGyro->hPmu, hGyro->VddId, NV_FALSE);
		NvOdmServicesPmuClose(hGyro->hPmu);
		hGyro->hPmu = NULL;
		NvGyroAccelI2CClose(hGyro->hOdmI2C);
		hGyro->hOdmI2C = NULL;
		NvOdmOsFree(hGyro);
		*hDevice = NULL;
	}
	return NV_FALSE;
}

void
NvOdmGyroAccelClose(NvOdmGyroAccelHandle hDevice)
{
	if (NULL != hDevice) {
		if (NULL != hDevice->SemaphoreForINT &&
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
		NvGyroAccelI2CClose(hDevice->hOdmI2C);
#if 1
		printk(" ## MPU3050  : [%s:%d] \n",__FUNCTION__, __LINE__) ;
#endif

		// Power off accelermeter
		NvGyroAccelSetPowerRail(hDevice->hPmu, hDevice->VddId, NV_FALSE);
		if (hDevice->hPmu) {
			//NvGyroAccelSetPowerOn(0);
			NvOdmServicesPmuClose(hDevice->hPmu);
		}

		return;
	}
}


NvBool
NvOdmGyroSetPowerState(NvOdmGyroAccelHandle hDevice, NvOdmGyroAccelPowerType PowerState)
{
	return NV_TRUE;
}

void NvOdmResetI2C(NvOdmGyroAccelHandle hDevice)
{
	printk("## MPU3050 GyroAccel : NvGyroAccelI2COpen Error \n");

	// close I2C
	NvGyroAccelI2CClose(hDevice->hOdmI2C);
	
	// reopen I2C
	if (NV_FALSE == NvGyroAccelI2COpen(&hDevice->hOdmI2C, hDevice->I2CChannelId)) {
		printk("## MPU305 GyroAccel : NvGyroAccelI2COpen Error !!!!!!!!!!! \n");
	}
}
