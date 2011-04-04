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


#include "nvodm_accelerometer_accel_kxtf9.h"
#include "nvodm_services.h"
#include "nvodm_query_discovery.h"
#include "nvos.h"
#include <linux/delay.h>
#include <linux/kernel.h>

#define NV_DEBOUNCE_TIME_MS 	0
#define DEBUG_LOG             	1
#define DEBUG_NV_ACCEL         	0

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
		NvOdmAccelAxisType *IntMotionAxis);


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
#if DEBUG_NV_ACCEL
	printk(" ## KXTF9 : %s attrib %x  =>  ", __FUNCTION__, attrib) ;
#endif

	switch (attrib) {
		case KIONIX_ACCEL_I2C_CTRL_REG1:
			hDevice->RegsWrite(hDevice, KIONIX_ACCEL_I2C_CTRL_REG1, &LocalInfo, 1);
#if DEBUG_NV_ACCEL
			printk("[%s] 1:  \n ", __FUNCTION__);
#endif
			break;
		case KIONIX_ACCEL_I2C_CTRL_REG3:
			hDevice->RegsWrite(hDevice, KIONIX_ACCEL_I2C_CTRL_REG3, &LocalInfo, 1);
#if DEBUG_NV_ACCEL
			printk("[%s] 2:  \n ", __FUNCTION__);
#endif
			break;
		case KXTF9_I2C_INT_CTRL_REG1:
			hDevice->RegsWrite(hDevice, KXTF9_I2C_INT_CTRL_REG1, &LocalInfo, 1);
#if DEBUG_NV_ACCEL
			printk("[%s] 3:  \n ", __FUNCTION__);
#endif
			break;
		case KIONIX_ACCEL_I2C_INT_CTRL_REG2:
			hDevice->RegsWrite(hDevice, KIONIX_ACCEL_I2C_INT_CTRL_REG2, &LocalInfo, 1);
#if DEBUG_NV_ACCEL
			printk("[%s] 4:  \n ", __FUNCTION__);
#endif
			break;
		case KXTF9_I2C_DATA_CTRL_REG:
			hDevice->RegsWrite(hDevice, KXTF9_I2C_DATA_CTRL_REG, &LocalInfo, 1);
#if DEBUG_NV_ACCEL
			printk("[%s] 5:  \n ", __FUNCTION__);
#endif
			break;
		case KIONIX_ACCEL_I2C_TILT_TIMER:
			hDevice->RegsWrite(hDevice, KIONIX_ACCEL_I2C_TILT_TIMER, &LocalInfo, 1);
#if DEBUG_NV_ACCEL
			printk("[%s] 6:  \n ", __FUNCTION__);
#endif
			break;
		case KIONIX_ACCEL_I2C_TDT_TIMER:
			hDevice->RegsWrite(hDevice, KIONIX_ACCEL_I2C_TDT_TIMER, &LocalInfo, 7);
#if DEBUG_NV_ACCEL
			printk("[%s] 7:  \n ", __FUNCTION__);
#endif
			break;
		case KIONIX_ACCEL_I2C_CTRL_REG2:
			hDevice->RegsWrite(hDevice, KIONIX_ACCEL_I2C_CTRL_REG2, &LocalInfo, 1);

#if DEBUG_NV_ACCEL
			printk("[%s] 8:  \n ", __FUNCTION__);
#endif
			break;
		default:
			printk("NV ODM ACCELEROMETER NvOdmAccelerometerSetParameter DONT SUPPORT SUCH ATTRIBUTE ---\n");
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
NvBool NvOdmAccelerometerGetParameter(NvOdmAccelHandle hDevice, NvU8 attrib, NvU32* info)
{
	NvU8 LocalInfo = 0;
#if DEBUG_NV_ACCEL
	printk(" ## KXTF9 : NvOdmAccelerometerGetParameter\n ");
	printk(" ## KXTF9 : attrib=%x => ", attrib);
#endif

	switch (attrib) {
#if 1
		case KIONIX_ACCEL_I2C_WHO_AM_I:
			hDevice->RegsRead(hDevice, KIONIX_ACCEL_I2C_WHO_AM_I, &LocalInfo, 1);

#if DEBUG_NV_ACCEL
			printk("[%s] 8:  \n ", __FUNCTION__);
#endif
			break;
		case KIONIX_ACCEL_I2C_CTRL_REG1:
			hDevice->RegsRead(hDevice, KIONIX_ACCEL_I2C_CTRL_REG1, &LocalInfo, 1);

#if DEBUG_NV_ACCEL
			printk("[%s] 8:  \n ", __FUNCTION__);
#endif
			break;
		case KIONIX_ACCEL_I2C_CTRL_REG3:
			hDevice->RegsRead(hDevice, KIONIX_ACCEL_I2C_CTRL_REG3, &LocalInfo, 1);

#if DEBUG_NV_ACCEL
			printk("[%s] 8:  \n ", __FUNCTION__);
#endif
			break;
		case KXTF9_I2C_INT_CTRL_REG1:
			hDevice->RegsRead(hDevice, KXTF9_I2C_INT_CTRL_REG1, &LocalInfo, 1);

#if DEBUG_NV_ACCEL
			printk("[%s] 8:  \n ", __FUNCTION__);
#endif
			break;
		case KIONIX_ACCEL_I2C_INT_CTRL_REG2:
			hDevice->RegsRead(hDevice, KIONIX_ACCEL_I2C_INT_CTRL_REG2, &LocalInfo, 1);

#if DEBUG_NV_ACCEL
			printk("[%s] 8:  \n ", __FUNCTION__);
#endif
			break;
		case KXTF9_I2C_INT_SRC_REG1:
			hDevice->RegsRead(hDevice, KXTF9_I2C_INT_SRC_REG1, &LocalInfo, 1);

#if DEBUG_NV_ACCEL
			printk("[%s] 8:  \n ", __FUNCTION__);
#endif
			break;
		case KXTF9_I2C_INT_SRC_REG2:
			hDevice->RegsRead(hDevice, KXTF9_I2C_INT_SRC_REG2, &LocalInfo, 1);

#if DEBUG_NV_ACCEL
			printk("[%s] 8:  \n ", __FUNCTION__);
#endif
			break;
		case KIONIX_ACCEL_I2C_TILT_POS_CUR:
			hDevice->RegsRead(hDevice, KIONIX_ACCEL_I2C_TILT_POS_CUR, &LocalInfo, 2);\

#if DEBUG_NV_ACCEL
				printk("[%s] 8:  \n ", __FUNCTION__);
#endif
			break;
		case KIONIX_ACCEL_I2C_TILT_POS_PRE:
			hDevice->RegsRead(hDevice, KIONIX_ACCEL_I2C_TILT_POS_PRE, &LocalInfo, 1);

#if DEBUG_NV_ACCEL
			printk("[%s] 8:  \n ", __FUNCTION__);
#endif
			break;
		case KXTF9_I2C_XOUT_L:
			hDevice->RegsRead(hDevice, KXTF9_I2C_XOUT_L, &LocalInfo, 1);

#if DEBUG_NV_ACCEL
			printk("[%s] 8:  \n ", __FUNCTION__);
#endif
			break;
		case KXTF9_I2C_XOUT_H:
			hDevice->RegsRead(hDevice, KXTF9_I2C_XOUT_H, &LocalInfo, 1);

#if DEBUG_NV_ACCEL
			printk("[%s] 8:  \n ", __FUNCTION__);
#endif
			break;
		case KXTF9_I2C_YOUT_L:
			hDevice->RegsRead(hDevice, KXTF9_I2C_YOUT_L, &LocalInfo, 1);

#if DEBUG_NV_ACCEL
			printk("[%s] 8:  \n ", __FUNCTION__);
#endif
			break;
		case KXTF9_I2C_YOUT_H:
			hDevice->RegsRead(hDevice, KXTF9_I2C_YOUT_H, &LocalInfo, 1);

#if DEBUG_NV_ACCEL
			printk("[%s] 8:  \n ", __FUNCTION__);
#endif
			break;
		case KXTF9_I2C_ZOUT_L:
			hDevice->RegsRead(hDevice, KXTF9_I2C_ZOUT_L, &LocalInfo, 1);

#if DEBUG_NV_ACCEL
			printk("[%s] 8:  \n ", __FUNCTION__);
#endif
			break;
		case KXTF9_I2C_ZOUT_H:
			hDevice->RegsRead(hDevice, KXTF9_I2C_ZOUT_H, &LocalInfo, 1);

#if DEBUG_NV_ACCEL
			printk("[%s] 8:  \n ", __FUNCTION__);
#endif
			break;
		case KIONIX_ACCEL_I2C_INT_REL:
			hDevice->RegsRead(hDevice, KIONIX_ACCEL_I2C_INT_REL, &LocalInfo, 1);

#if DEBUG_NV_ACCEL
			printk("[%s] 8:  \n ", __FUNCTION__);
#endif
			break;
		case 0x21:
			hDevice->RegsRead(hDevice, 0x21, &LocalInfo, 1);

#if DEBUG_NV_ACCEL
			printk("[%s] 8:  \n ", __FUNCTION__);
#endif
			break;
		default:
			printk("NV ODM ACCELEROMETER NvOdmAccelerometerGetParameter DONT SUPPORT SUCH ATTRIBUTE ---\n");
			return NV_FALSE;
#endif
	}
#if DEBUG_LOG
	printk("LocalInfo : %x/  info : %x/ *info : %d/  \n", LocalInfo, info, *info);
#endif

	*info = LocalInfo;

#if DEBUG_LOG
	printk(" ## KXTF9 : [%s:%d] \n", __FUNCTION__, __LINE__);
	printk("LocalInfo : %x \n", LocalInfo);
#endif
	//NVODMACCELEROMETER_PRINTF("NV ODM ACCELEROMETER NvOdmAccelerometerGetParameter ---\n");
	return NV_TRUE;

}

void NvAccelerometerSetPowerRail(NvOdmServicesPmuHandle hPMUDevice, NvU32 Id, NvBool IsEnable)
{
	NvOdmServicesPmuVddRailCapabilities vddrailcap;
	NvU32 settletime;
	printk(" ## KXTF9 : 3 \n");

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
void NvAccelerometerGetInterruptSouce(NvOdmAccelHandle hDevice,
		NvOdmAccelIntType  *IntType,
		NvOdmAccelAxisType *IntMotionAxis)
{
	NvU32    reg_val ;

	NV_ASSERT(hDevice != 0);
	NV_ASSERT(IntType != 0);
	NV_ASSERT(IntMotionAxis != 0);

	*IntType = NvOdmAccelInt_None;
	*IntMotionAxis = NvOdmAccelAxis_None;

	if (NULL != hDevice) {
		NvOdmAccelerometerGetParameter(hDevice, KXTF9_I2C_INT_SRC_REG2, &reg_val);
#if DEBUG_LOG
		printk(" ## KXTF9 : 4 \n") ;
		printk(" ## KXTF9 : KXTF9_I2C_INT_SRC_REG2 = %x ## \n", reg_val) ;
#endif
		if (reg_val & 0x01) {  // TPS : Screen Rotation
			*IntType |= NvOdmAccelInt_MotionThreshold;
		}
	}

}


	static void
GpioInterruptHandler(void *arg)
{
	NvOdmGpioPinMode mode;

	NvU32 pinValue = 0;
	NvOdmAccelHandle hDevice =  (NvOdmAccelHandle)arg;
	printk(" ## KXTF9 : 5 \n") ;

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

NvBool NvAccelerometerConnectSemaphore(NvOdmAccelHandle hDevice)
{
	NvOdmGpioPinMode mode;
	NvOdmInterruptHandler callback = (NvOdmInterruptHandler)GpioInterruptHandler;
	printk(" ## KXTF9 : NvAccelerometerConnectSemaphore \n") ;
	hDevice->hGpioINT = (NvOdmServicesGpioHandle)NvOdmGpioOpen();
	if (!(hDevice->hGpioINT)) {
		printk("## NvOdm Accelerometer : NvOdmGpioOpen Error ##  \n");
		return NV_FALSE;
	}

	hDevice->hPinINT = NvOdmGpioAcquirePinHandle(hDevice->hGpioINT,
			hDevice->GPIOPortINT,
			hDevice->GPIOPinINT);

	hDevice->SemaphoreForINT = NvOdmOsSemaphoreCreate(0);

	if (!(hDevice->SemaphoreForINT)) {
		printk("## NvOdm Accelerometer : NvOdmOsSemaphoreCreate Error ## \n");
		NvOdmGpioClose(hDevice->hGpioINT);
		return NV_FALSE;
	}

	mode = NvOdmGpioPinMode_InputInterruptHigh;

	if (NvOdmGpioInterruptRegister(hDevice->hGpioINT, &hDevice->hGpioInterrupt,
				hDevice->hPinINT, mode, callback, hDevice, NV_DEBOUNCE_TIME_MS) == NV_FALSE)
	{
		printk("NvOdm Accelerometer : cannot register interrupt.\n") ;
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
NvBool NvAccelerometerI2COpen(NvOdmServicesI2cHandle* hI2CDevice, NvU32 id)
{
	// Open I2C handle.

	*hI2CDevice  =  NvOdmI2cPinMuxOpen(NvOdmIoModule_I2c, 1, NvOdmI2cPinMap_Config2);
	//	*hI2CDevice  =  NvOdmI2cPinMuxOpen(NvOdmIoModule_I2c, 1, NvOdmI2cPinMap_Multiplexed);
	//	*hI2CDevice  =  NvOdmI2cOpen(NvOdmIoModule_I2c, 1);
	printk(" ## KXTF9 : NvAccelerometerI2COpen \n") ;

	if (*hI2CDevice == NULL) {
		printk(" ## KXTF9 NvAccelerometerI2COpen Error ##\n");
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
	printk(" ## KXTF9 :  NvAccelerometerI2CClose \n") ;
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

NvBool NvAccelerometerI2CSetRegs(NvOdmAccelHandle hDevice, NvU8 offset, NvU8* value, NvU32 len)
{
	NvOdmI2cTransactionInfo TransactionInfo;

	if ((NULL == hDevice) || (NULL == value) || (len > I2C_ACCELRATOR_PACKET_SIZE-1)) {
		return NV_FALSE;
	}
	NvOdmOsMemset(s_WriteBuffer, 0, sizeof(s_WriteBuffer));
	s_WriteBuffer[0] = offset;
	NvOdmOsMemcpy(&s_WriteBuffer[1], value, len);

	TransactionInfo.Address = hDevice->nDevAddr;
	TransactionInfo.Buf = s_WriteBuffer;
	TransactionInfo.Flags = NVODM_I2C_IS_WRITE;
	TransactionInfo.NumBytes = len+1;

	if (NvOdmI2cStatus_Success != NvOdmI2cTransaction(hDevice->hOdmI2C, &TransactionInfo, 1, 400, I2C_ACCELRATOR_TRANSACTION_TIMEOUT))
	{
		if (NvOdmI2cStatus_Success != NvOdmI2cTransaction(hDevice->hOdmI2C, &TransactionInfo, 1, 400, I2C_ACCELRATOR_TRANSACTION_TIMEOUT))
		{
			printk(" ## KXTF9 _ NvAccelerometerI2CSetRegs failed: register %d \n", offset);
			return NV_FALSE;
		}
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
NvBool NvAccelerometerI2CGetRegs(NvOdmAccelHandle hDevice, NvU8 offset, NvU8* value, NvU32 len)
{
	NvOdmI2cTransactionInfo TransactionInfo;

	if ((NULL == hDevice) || (NULL == value) || (len > I2C_ACCELRATOR_PACKET_SIZE-1)) {
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
	if (NvOdmI2cStatus_Success != NvOdmI2cTransaction(hDevice->hOdmI2C, &TransactionInfo, 1, 400, I2C_ACCELRATOR_TRANSACTION_TIMEOUT))
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
	if (NvOdmI2cStatus_Success != NvOdmI2cTransaction(hDevice->hOdmI2C, &TransactionInfo, 1, 400, I2C_ACCELRATOR_TRANSACTION_TIMEOUT))
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
	NvU32 i;
	NvBool foundGpio = NV_FALSE, foundI2cModule = NV_FALSE;
	const NvOdmPeripheralConnectivity *pConnectivity;
	NvOdmAccelHandle  hAccel;
	NvU32    reg_val = 0 ;
#if DEBUG_NV_ACCEL
	printk(" ## KXTF9 : [NvOdmAccelOpen:%d] \n",__LINE__) ;
#endif

	hAccel = NvOdmOsAlloc(sizeof(NvOdmAccel));
	if (hAccel == NULL) {
		printk("Error Allocating NvOdmAccel. \n");
		return NV_FALSE;
	}
	NvOdmOsMemset(hAccel, 0, sizeof(NvOdmAccel));

	hAccel->CtrlRegsList[0].RegAddr = KIONIX_ACCEL_I2C_TILT_TIMER;
	hAccel->CtrlRegsList[0].RegValue = 0x01;
	hAccel->CtrlRegsList[1].RegAddr = KIONIX_ACCEL_I2C_CTRL_REG2;
	hAccel->CtrlRegsList[1].RegValue = 0x3F;
	hAccel->CtrlRegsList[2].RegAddr = KIONIX_ACCEL_I2C_CTRL_REG3;
	hAccel->CtrlRegsList[2].RegValue = 0x4D;
	hAccel->CtrlRegsList[3].RegAddr = KXTF9_I2C_INT_CTRL_REG1;
	hAccel->CtrlRegsList[3].RegValue = 0x10;


	hAccel->hPmu = NULL;
	hAccel->hOdmI2C =  NULL;

	hAccel->hPmu = NvOdmServicesPmuOpen();
	if (!hAccel->hPmu) {
		printk("NvOdmServicesPmuOpen Error \n");
		goto error;
	}

	pConnectivity = (NvOdmPeripheralConnectivity*)NvOdmPeripheralGetGuid(NV_ODM_GUID('a','c','c','e','l','e','r','o'));
	if (!pConnectivity) {
		printk("NvOdmPeripheralGetGuid doesn't detect accelerometer device\n");
		goto error;
	}

	if (pConnectivity->Class != NvOdmPeripheralClass_Other) {
		goto error;
	}

	for (i = 0; i < pConnectivity->NumAddress; i++) {
		switch(pConnectivity->AddressList[i].Interface) {
			case NvOdmIoModule_I2c:
				hAccel->I2CChannelId = pConnectivity->AddressList[i].Instance;
				hAccel->nDevAddr = (pConnectivity->AddressList[i].Address << 1) ;
				foundI2cModule = NV_TRUE;
#if DEBUG_LOG
				printk("## KXTF9 I2CChannelId = %x. ## \n", hAccel->I2CChannelId);
				printk("## KXTF9 i2c address = %x. ## \n", hAccel->nDevAddr);
#endif
				break;
			case NvOdmIoModule_Gpio:
				hAccel->GPIOPortINT = pConnectivity->AddressList[i].Instance;
				hAccel->GPIOPinINT = pConnectivity->AddressList[i].Address;
				foundGpio = NV_TRUE;
#if DEBUG_LOG
				printk("## KXTF9 GPIOPortINT = %x. ## \n",hAccel->GPIOPortINT);
				printk("## KXTF9 GPIOPinINT = %x. ## \n", hAccel->GPIOPinINT);
#endif
				break;
			case NvOdmIoModule_Vdd:
				hAccel->VddId = pConnectivity->AddressList[i].Address;
#if DEBUG_LOG
				printk("## KXTF9 NvOdmIoModule_VddId = %x. ## \n", hAccel->VddId);
#endif
				// Power on accelerometer according to Vddid
				NvAccelerometerSetPowerRail(hAccel->hPmu, hAccel->VddId, NV_TRUE);
				break;
			default:
				break;
		}
	}

	if (foundGpio != NV_TRUE || foundI2cModule != NV_TRUE) {
		printk("Accelerometer : didn't find any periperal in discovery query for touch device Error \n");
		goto error;
	}


	// Set up I2C bus.
	if (NV_FALSE == NvAccelerometerI2COpen(&hAccel->hOdmI2C, hAccel->I2CChannelId)) {
		goto error;
	};

	hAccel->RegsRead = NvAccelerometerI2CGetRegs;
	hAccel->RegsWrite = NvAccelerometerI2CSetRegs;

	if (NV_FALSE == NvAccelerometerConnectSemaphore(hAccel)) {
		goto error;
	}
#if 0
	mdelay(20); //Power Up time

	NvOdmAccelerometerGetParameter(hAccel, KIONIX_ACCEL_I2C_WHO_AM_I, &reg_val);
	printk(" ## KXTF9 register : KIONIX_ACCEL_I2C_WHO_AM_I = %x ##\n", reg_val);
	if(reg_val != 0x01) {
		printk("-E- Invalid KXTF9 device id = 0x%x\n", reg_val);
		// TODO:
		return NV_FALSE;
	}

	NvOdmAccelerometerGetParameter(hAccel, KIONIX_ACCEL_I2C_CTRL_REG1, &reg_val);
	printk("[%s:%d] CTRL_REG1 = 0x%x\n", __FUNCTION__,__LINE__, reg_val);
	if(reg_val & 0x80) {
		reg_val &= ~(0x80);
		printk("[%s:%d]CTRL_REG1 = 0x%x\n", __FUNCTION__,__LINE__, reg_val);
		NvOdmAccelerometerSetParameter(hAccel, KIONIX_ACCEL_I2C_CTRL_REG1, &reg_val);
	}

	NvOdmAccelerometerGetParameter(hAccel, KIONIX_ACCEL_I2C_CTRL_REG3, &reg_val);
	printk("[%s, %d] ACCEL SRST Setting, 0x%x \n",__FUNCTION__,__LINE__ , reg_val);
	reg_val |= (1<<7);
	printk("[%s, %d] ACCEL SRST Setting, 0x%x \n",__FUNCTION__,__LINE__ , reg_val);
	NvOdmAccelerometerSetParameter(hAccel, KIONIX_ACCEL_I2C_CTRL_REG3, reg_val);   // sets SRST bit to reboot
	mdelay(20);

	NvOdmAccelerometerGetParameter(hAccel, KIONIX_ACCEL_I2C_CTRL_REG3, &reg_val);
	printk("[%s, %d] Checking SRST bit, 0x%x \n",__FUNCTION__,__LINE__ , reg_val);

	for(i=0; i<4; i++)
	{
		NvOdmAccelerometerSetParameter(hAccel,
				hAccel->CtrlRegsList[i].RegAddr,
				hAccel->CtrlRegsList[i].RegValue);
	}

#if DEBUG_NV_ACCEL
	printk(" ## KXTF9 : [NvOdmAccelOpen:%d] \n",__LINE__) ;
#endif

	NvOdmAccelerometerGetParameter(hAccel, KIONIX_ACCEL_I2C_CTRL_REG1, &reg_val);
	reg_val |= CTRL_REG1_TPS;
	reg_val |= CTRL_REG1_RES;
	NvOdmAccelerometerSetParameter(hAccel, KIONIX_ACCEL_I2C_CTRL_REG1, reg_val);   // sets TPE bit to enable tilt position function

	NvOdmAccelerometerGetParameter(hAccel, KIONIX_ACCEL_I2C_CTRL_REG1, &reg_val);
	reg_val |= CTRL_REG1_PC1;
	NvOdmAccelerometerSetParameter(hAccel, KIONIX_ACCEL_I2C_CTRL_REG1, reg_val);   // eable outputs
#endif
#if 0
	NvOdmAccelerometerGetParameter(hAccel, KIONIX_ACCEL_I2C_WHO_AM_I, &reg_val);
	printk(" ## KXTF9 register : KIONIX_ACCEL_I2C_WHO_AM_I = %x ##\n", reg_val);

	NvOdmAccelerometerGetParameter(hAccel, KIONIX_ACCEL_I2C_CTRL_REG1, &reg_val);
	printk(" ## KXTF9 register : KIONIX_ACCEL_I2C_CTRL_REG1 = %x ##\n", reg_val);

	NvOdmAccelerometerGetParameter(hAccel, 0x21, &reg_val);
	printk(" ## KXTF9 register : KXTF9_I2C_DATA_CTRL_REG = %x ##\n", reg_val);
#endif

	*hDevice = hAccel;
	return NV_TRUE;
error:
	// Release all of resources requested.
	if (NULL != hAccel) {
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
		NvAccelerometerI2CClose(hDevice->hOdmI2C);
#if DEBUG_NV_ACCEL
		printk(" ## KXTF9 : [%s:%d] \n", __FUNCTION__, __LINE__) ;
#endif

		// Power off accelermeter
		NvAccelerometerSetPowerRail(hDevice->hPmu, hDevice->VddId, NV_FALSE);
		if (hDevice->hPmu) {
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
	printk("MUST ITEM \n");
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

#if DEBUG_NV_ACCEL
	printk(" ## KXTF9 : [%s:%d] \n", __FUNCTION__, __LINE__) ;
	printk(" ## KXTF9 : IntType %x, IntAxis  %x,IntNum %x, Toggle  %x\n",IntType, IntAxis,IntNum, Toggle);
#endif


	switch (IntType) {
		case NvOdmAccelInt_MotionThreshold:
			{
#if DEBUG_NV_ACCEL
				printk(" ## KXTF9 : [%s:%d] \n", __FUNCTION__, __LINE__) ;
#endif
				NvOdmAccelerometerGetParameter(hDevice, KIONIX_ACCEL_I2C_INT_CTRL_REG2, &uTemp);
				//NvOdmOsDebugPrintf("INTCONTROL is 0x%x g\n", uTemp);
				switch (IntAxis)
				{
					case NvOdmAccelAxis_X:
						{
#if DEBUG_NV_ACCEL
							printk(" ## KXTF9 : [%s:%d] \n", __FUNCTION__, __LINE__) ;
#endif
							if (Toggle == NV_TRUE) {
								uTemp |= INT_CTRL_REG2_XBW;
							} else {
								uTemp &= INT_CTRL_REG2_XBW_MASK;
							}
							break;
						}
					case NvOdmAccelAxis_Y:
						{
#if DEBUG_NV_ACCEL
							printk(" ## KXTF9 : [%s:%d] \n", __FUNCTION__, __LINE__) ;
#endif
							if (Toggle == NV_TRUE) {
								uTemp |= INT_CTRL_REG2_YBW;
							} else {
								uTemp &= INT_CTRL_REG2_YBW_MASK;
							}
							break;
						}
					case NvOdmAccelAxis_Z:
						{
#if DEBUG_NV_ACCEL
							printk(" ## KXTF9 : [%s:%d] \n", __FUNCTION__, __LINE__) ;
#endif
							if (Toggle == NV_TRUE) {
								uTemp |= INT_CTRL_REG2_ZBW;
							} else {
								uTemp &= INT_CTRL_REG2_ZBW_MASK;
							}
							break;
						}
					case NvOdmAccelAxis_All:
						{
#if DEBUG_NV_ACCEL
							printk(" ## KXTF9 : [%s:%d] \n", __FUNCTION__, __LINE__) ;
#endif
							if (Toggle == NV_TRUE) {
								uTemp |= INT_CTRL_REG2_XBW;
								uTemp |= INT_CTRL_REG2_YBW;
								uTemp |= INT_CTRL_REG2_ZBW;
							} else {
#if DEBUG_NV_ACCEL
								printk(" ## KXTF9 : [%s:%d] \n", __FUNCTION__, __LINE__) ;
#endif
								uTemp &= INT_CTRL_REG2_XBW_MASK;
								uTemp &= INT_CTRL_REG2_YBW_MASK;
								uTemp &= INT_CTRL_REG2_ZBW_MASK;
							}
							break;
						}
					default:
						{
							return NV_FALSE;
						}
				}
				//printk(" ## KXTF9 111 12\n") ;
				NvOdmAccelerometerSetParameter(hDevice, KIONIX_ACCEL_I2C_INT_CTRL_REG2, uTemp);
				break;
			}
		default:
			{
				printk(" ## KXTF9: Do not support such Interrupt!\n");
				return NV_FALSE;
				break ;
			}
	}

	NvOdmAccelerometerGetParameter(hDevice, KXTF9_I2C_INT_CTRL_REG1, &uTemp);
	//printk(" ## KXTF9 111 10\n") ;
	uTemp |= KXTF9_INT_CTRL_REG1_IEN;                    // enables the physical interrupt.
#if DEBUG_LOG
	printk(" ## KXTF9 _ NvOdmAccelSetIntEnable : uTemp = %x ## \n", uTemp) ;
#endif
	NvOdmAccelerometerSetParameter(hDevice, KXTF9_I2C_INT_CTRL_REG1, uTemp);
	return NV_TRUE;

}

void
NvOdmAccelWaitInt(NvOdmAccelHandle	 hDevice,
		NvOdmAccelIntType  *IntType,
		NvOdmAccelAxisType *IntMotionAxis,
		NvOdmAccelAxisType *IntTapAxis)
{
	NvU32 uTemp ;
	NV_ASSERT(NULL != hDevice);
	NV_ASSERT(NULL != IntType);
	NV_ASSERT(NULL != IntMotionAxis);

#if defined(CONFIG_MACH_STAR)
	NvOdmOsSemaphoreWaitTimeout( hDevice->SemaphoreForINT,400);
#endif
#if DEBUG_NV_ACCEL
	printk(" ## KXTF9 : [%s:%d] \n", __FUNCTION__, __LINE__) ;
#endif

	// disables the physical interrupt.
	NvOdmAccelerometerGetParameter(hDevice, KXTF9_I2C_INT_CTRL_REG1, &uTemp);
	uTemp &= KXTF9_INT_CTRL_REG1_IEN_MASK;
	NvOdmAccelerometerSetParameter(hDevice, KXTF9_I2C_INT_CTRL_REG1, uTemp);

	NvAccelerometerGetInterruptSouce( hDevice , IntType, IntMotionAxis);

	// clear the interrupt source information along with interrupt pin
	NvOdmAccelerometerGetParameter(hDevice, KIONIX_ACCEL_I2C_INT_REL, &uTemp);

	// enables the physical interrupt again.
	NvOdmAccelerometerGetParameter(hDevice, KXTF9_I2C_INT_CTRL_REG1, &uTemp);
	uTemp |= KXTF9_INT_CTRL_REG1_IEN;
	NvOdmAccelerometerSetParameter(hDevice, KXTF9_I2C_INT_CTRL_REG1, uTemp);

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
	NvU32 Res, G_range;
	NvU32 data_L = 0,  data_H = 0 ;
	NvS8 x_sign, y_sign, z_sign ;
	NvS32 sensitivity, range ;

	NV_ASSERT(NULL != hDevice);
	NV_ASSERT(NULL != AccelX);
	NV_ASSERT(NULL != AccelY);
	NV_ASSERT(NULL != AccelZ);
#if DEBUG_NV_ACCEL
	printk(" ## KXTF9 : [%s:%d] \n", __FUNCTION__, __LINE__) ;
#endif

	if (NvOdmAccelerometerGetParameter(hDevice, KIONIX_ACCEL_I2C_CTRL_REG1, &Res) == NV_TRUE)
	{
		if (NvOdmAccelerometerGetParameter(hDevice, KIONIX_ACCEL_I2C_CTRL_REG1, &G_range) == NV_TRUE)
		{
			G_range = G_range & 0x18;
			G_range = G_range >> 3;
			switch(G_range){
				case 0:
					range = 2;
					break;
				case 1:
					range = 4;
					break;
				case 2:
					range = 8;
					break;
				default:
					break;
			}
			Res = Res & 0x40 ;
			switch(Res) {
				case 0x00 :   // low-resolution state(8 bit)
					{
						if (NvOdmAccelerometerGetParameter(hDevice, KXTF9_I2C_XOUT_H, &data_H) == NV_TRUE)
						{
							*AccelX = ((NvS32)data_H) ;
							x_sign = *AccelX >> 7 ; 	 // 1 : negative, 0 : positive
							if (x_sign == 1) {
								*AccelX = ((~(*AccelX) + 0x01) & 0x0FF);  // 2's complement
								*AccelX = -(*AccelX);
							}
						}
						if (NvOdmAccelerometerGetParameter(hDevice, KXTF9_I2C_YOUT_H, &data_H) == NV_TRUE)
						{
							*AccelY = ((NvS32)data_H) ;
							y_sign = *AccelY >> 7 ; 	 // 1 : negative, 0 : positive
							if (y_sign == 1) {
								*AccelY = ((~(*AccelY) + 0x01) & 0x0FF);  // 2's complement
								*AccelY = -(*AccelY);
							}
						}
						if (NvOdmAccelerometerGetParameter(hDevice, KXTF9_I2C_ZOUT_H, &data_H) == NV_TRUE)
						{
							*AccelZ = ((NvS32)data_H) ;
							z_sign = *AccelZ >> 7 ; 	 // 1 : negative, 0 : positive
							if (z_sign == 1) {
								*AccelZ = ((~(*AccelZ) + 0x01) & 0x0FF);  // 2's complement
								*AccelZ = -(*AccelZ);
							}
						}
						sensitivity = (256) / (2 * range);
						// calculate milli-G's
						*AccelX = 1000 * (*AccelX) / sensitivity;
						*AccelY = 1000 * (*AccelY) / sensitivity;
						*AccelZ = 1000 * (*AccelZ) / sensitivity;
					}
					break ;
				case 0x40 :   // high-resolution state(12 bit)
					{
						if (NvOdmAccelerometerGetParameter(hDevice, KXTF9_I2C_XOUT_H, &data_H) == NV_TRUE)
						{
							if (NvOdmAccelerometerGetParameter(hDevice, KXTF9_I2C_XOUT_L, &data_L) == NV_TRUE)
							{
								*AccelX = ((NvS32)data_L) >> 4 ;
								*AccelX = *AccelX + (((NvS32)data_H) << 4 ) ;
								x_sign = *AccelX >> 11 ;
								if (x_sign == 1) {
									*AccelX = ((~(*AccelX) + 0x01) & 0x0FF);  // 2's complement
									*AccelX = -(*AccelX);
								}
							}
						}
						if (NvOdmAccelerometerGetParameter(hDevice, KXTF9_I2C_YOUT_H, &data_H) == NV_TRUE)
						{
							if (NvOdmAccelerometerGetParameter(hDevice, KXTF9_I2C_YOUT_L, &data_L) == NV_TRUE)
							{
								*AccelY = ((NvS32)data_L) >> 4 ;
								*AccelY = *AccelY + (((NvS32)data_H) << 4 ) ;
								y_sign = *AccelY >> 11 ;
								if (y_sign == 1) {
									*AccelY = ((~(*AccelY) + 0x01) & 0x0FF);  // 2's complement
									*AccelY = -(*AccelY);
								}
							}
						}
						if (NvOdmAccelerometerGetParameter(hDevice, KXTF9_I2C_ZOUT_H, &data_H) == NV_TRUE)
						{
							if (NvOdmAccelerometerGetParameter(hDevice, KXTF9_I2C_ZOUT_L, &data_L) == NV_TRUE)
							{
								*AccelZ = ((NvS32)data_L) >> 4 ;
								*AccelZ = *AccelZ + (((NvS32)data_H) << 4 ) ;
								z_sign = *AccelZ >> 11 ;
								if (z_sign == 1) {
									*AccelZ = ((~(*AccelZ) + 0x01) & 0x0FF);  // 2's complement
									*AccelZ = -(*AccelZ);
								}
							}
						}
						sensitivity = (4096) / (2 * range);
						/* calculate milli-G's */
						*AccelX = 1000 * (*AccelX) / sensitivity;
						*AccelY = 1000 * (*AccelY) / sensitivity;
						*AccelZ = 1000 * (*AccelZ) / sensitivity;
					}
					break;
				default:
					printk("KIONIX_ACCEL_read_LPF_cnt: NO TYPE\n");
					break;
			}

		}

	}

	return NV_TRUE;
}


NvOdmAccelerometerCaps
NvOdmAccelGetCaps(NvOdmAccelHandle hDevice)
{
	NV_ASSERT(NULL != hDevice);

	return hDevice->Caption;
}

/*
 * This function sets the low pass filter roll off for the accelerometer outputs.
 * SampleRate[Input]: roll off frequency (6, 12, 25, 50, 100, 200, 400)
 * Returns NV_TRUE if successful, or NV_FALSE otherwise.
 */

NvBool
NvOdmAccelSetSampleRate(NvOdmAccelHandle hDevice, NvU32 SampleRate)
{
	NV_ASSERT(NULL != hDevice);

	switch (SampleRate) {
		case 12 :      /* set ( Output data rate : 12.5 Hz / LPF rolloff : 6.25 Hz ) */
			NvOdmAccelerometerSetParameter(hDevice, KXTF9_I2C_DATA_CTRL_REG, 0x00);
			break ;
		case 25 : 	   /* set ( Output data rate : 25 Hz / LPF rolloff : 12.5 Hz ) */
			NvOdmAccelerometerSetParameter(hDevice, KXTF9_I2C_DATA_CTRL_REG, 0x01);
			break ;
		case 50 : 	   /* set ( Output data rate : 50 Hz / LPF rolloff : 25 Hz ) */
			NvOdmAccelerometerSetParameter(hDevice, KXTF9_I2C_DATA_CTRL_REG, 0x02);
			break ;
		case 100 : 	   /* set ( Output data rate : 100 Hz / LPF rolloff : 50 Hz ) */
			NvOdmAccelerometerSetParameter(hDevice, KXTF9_I2C_DATA_CTRL_REG, 0x03);
			break ;
		case 200 : 	   /* set ( Output data rate : 200 Hz / LPF rolloff : 100 Hz ) */
			NvOdmAccelerometerSetParameter(hDevice, KXTF9_I2C_DATA_CTRL_REG, 0x04);
			break ;
		case 400 : 	   /* set ( Output data rate : 400 Hz / LPF rolloff : 200 Hz ) */
			NvOdmAccelerometerSetParameter(hDevice, KXTF9_I2C_DATA_CTRL_REG, 0x05);
			break ;
		case 800 : 	   /* set ( Output data rate : 800 Hz / LPF rolloff : 400 Hz ) */
			NvOdmAccelerometerSetParameter(hDevice, KXTF9_I2C_DATA_CTRL_REG, 0x06);
			break ;
		default:
			printk(" ## NvOdmAccelSetSampleRate ERROR : No Type ## \n");
			break;
	}
	return NV_TRUE;
}


NvBool
NvOdmAccelGetSampleRate(NvOdmAccelHandle hDevice, NvU32 *pSampleRate)
{
	NvU32 CurrentIndex = 0;
	NvU32 data_ctrl_reg = 0 ;

	NV_ASSERT(NULL != hDevice);
#if DEBUG_NV_ACCEL
	printk(" ## KXTF9 : [%s:%d] \n", __FUNCTION__, __LINE__) ;
#endif

	if (NvOdmAccelerometerGetParameter(hDevice, KXTF9_I2C_DATA_CTRL_REG, &data_ctrl_reg) != NV_TRUE)
	{
		return NV_FALSE ;
	}

	CurrentIndex = data_ctrl_reg & 0x07 ;

	switch (CurrentIndex) {
		case 0x00 :
			*pSampleRate = 12 ;
			break ;
		case 0x01 :
			*pSampleRate = 25 ;
			break ;
		case 0x02 :
			*pSampleRate = 50 ;
			break ;
		case 0x03 :
			*pSampleRate = 100 ;
			break ;
		case 0x04 :
			*pSampleRate = 200 ;
			break ;
		case 0x05 :
			*pSampleRate = 400 ;
			break ;
		case 0x06 :
			*pSampleRate = 800 ;
			break ;
		default :
			printk(" ## NvOdmAccelGetSampleRate ERROR : No Type ## \n");
			break;
	}
	return NV_TRUE;
}

NvBool
NvOdmAccelSetPowerState(NvOdmAccelHandle hDevice, NvOdmAccelPowerType PowerState)
{
	return NV_TRUE;
}

