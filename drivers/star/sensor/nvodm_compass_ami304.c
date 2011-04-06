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

/*  Reference code :
	NVIDIA Tegra ODM Kit Sample Accelerometer Adaptation of the
 *  WinCE Accelerometer Driver
 */
#include "nvodm_accelerometer_accel_kxtf9.h"
#include <nvodm_compass.h>

#include "nvodm_compass_ami304.h"
#include "nvodm_services.h"
#include "nvodm_query_discovery.h"
#include "nvos.h"
#include <linux/kernel.h>
#include <linux/delay.h>


#define NV_DEBOUNCE_TIME_MS	0
#define DEBUG_LOG			0
#define COMPASS_I2C_MAX_RETRY   5

extern int reboot;

// For interrupt handle, set GPIO when an interrupt happens.
static void GpioInterruptHandler(void *arg);
NvBool NvCompassI2COpen(NvOdmServicesI2cHandle* hI2CDevice, NvU32 id);
void   NvCompassI2CClose(NvOdmServicesI2cHandle hI2CDevice);
NvBool NvCompassI2CSetRegs(NvOdmCompassHandle hDevice, NvU8 offset, NvU8* value, NvU32 len);
NvBool NvCompassI2CGetRegs(NvOdmCompassHandle hDevice, NvU8 offset, NvU8* value, NvU32 len);
NvBool NvCompassConnectSemaphore(NvOdmCompassHandle hDevice);
void NvCompassSetPowerRail(NvOdmServicesPmuHandle hPMUDevice, NvU32 Id, NvBool IsEnable);
void NvCompassGetInterruptSouce(NvOdmCompassHandle hDevice,
		NvOdmCompassIntType  *IntType,
		NvOdmCompassAxisType *IntMotionAxis);

/*
 * Set compass registers.
 * [in] attrib: The register flag.
 * [out] info: The value to be set into the register of compass.
 * Returns NV_TRUE if successful, or NV_FALSE otherwise.
 */

NvBool NvOdmCompassSetParameter(NvOdmCompassHandle hDevice, NvU8 attrib, NvU32 info)
{

	NvU8 LocalInfo = 0;
	LocalInfo = (NvU8)(info);
	// Due to the register length, we only accept the lowest 8 bits.
	NvOdmOsMemcpy(&LocalInfo, &info, sizeof(NvU8));

	//printk("%s : attrib %d \n", __FUNCTION__, attrib);

	switch (attrib) {
		case AMI304_REG_CTRL1:
			hDevice->RegsWrite(hDevice, AMI304_REG_CTRL1, &LocalInfo, 1);
			//printk("%s -1 \n", __FUNCTION__);
			break;
		case AMI304_REG_CTRL3:
			hDevice->RegsWrite(hDevice, AMI304_REG_CTRL3, &LocalInfo, 1);
			//printk("%s -2 \n", __FUNCTION__);
			break;
		case AMI304_I2C_INS1:
			hDevice->RegsWrite(hDevice, AMI304_I2C_INS1, &LocalInfo, 1);
			//printk("%s -3 \n", __FUNCTION__);
			break;
		case AMI304_I2C_INL:
			hDevice->RegsWrite(hDevice, AMI304_I2C_INL, &LocalInfo, 1);
			//printk("%s -4 \n", __FUNCTION__);
			break;
		case AMI304_I2C_INC1:
			hDevice->RegsWrite(hDevice, AMI304_I2C_INC1, &LocalInfo, 1);
			//printk("%s -5 \n", __FUNCTION__);
			break;
		case AMI304_REG_CTRL2:
			hDevice->RegsWrite(hDevice, AMI304_REG_CTRL2, &LocalInfo, 1);
			//printk("%s -6 \n", __FUNCTION__);
			break;
		default:
			printk("NV ODM COMPASS NvOdmCompassSetParameter DONT SUPPORT SUCH ATTRIBUTE ---\n");
			return NV_FALSE;
	}

	return NV_TRUE;
}



/*
 * Get acceleromter registers.
 * [in] attrib: The regsiter flag.
 * [out] info: The value from register of compass.
 * Returns NV_TRUE if successful, or NV_FALSE otherwise.
 */
NvBool NvOdmCompassGetParameter(NvOdmCompassHandle hDevice, NvU8 attrib, NvU32* info)
{
	NvU8 LocalInfo = 0;
	//printk("%s : attrib %d \n", __FUNCTION__, attrib);

	switch (attrib) {
#if 1
		case AMI304_I2C_WHO_AM_I:
			hDevice->RegsRead(hDevice, AMI304_I2C_WHO_AM_I, &LocalInfo, 1);
			//printk("%s -1 \n", __FUNCTION__);
			break;
		case AMI304_I2C_STA1:
			hDevice->RegsRead(hDevice, AMI304_REG_CTRL1, &LocalInfo, 1);
			//printk("%s -2 \n", __FUNCTION__);
			break;
		case AMI304_REG_CTRL1:
			hDevice->RegsRead(hDevice, AMI304_REG_CTRL1, &LocalInfo, 1);
			//printk("%s -3 \n", __FUNCTION__);
			break;
		case AMI304_REG_CTRL2:
			hDevice->RegsRead(hDevice, AMI304_REG_CTRL2, &LocalInfo, 1);
			//printk("%s -4 \n", __FUNCTION__);
			break;
		case AMI304_REG_CTRL3:
			hDevice->RegsRead(hDevice, AMI304_REG_CTRL3, &LocalInfo, 1);
			//printk("%s -5 \n", __FUNCTION__);
			break;
		case AMI304_REG_DATAXH:
			hDevice->RegsRead(hDevice, AMI304_REG_DATAXH, &LocalInfo, 6);
			//printk("%s -6 \n", __FUNCTION__);
			break;
#if 0
		case AMI304_I2C_XOUT_L:
			hDevice->RegsRead(hDevice, AMI304_I2C_XOUT_L, &LocalInfo, 1);
			break;
		case AMI304_I2C_XOUT_H:
			hDevice->RegsRead(hDevice, AMI304_I2C_XOUT_H, &LocalInfo, 1);
			break;
#endif
		case AMI304_I2C_YOUT_L:
			hDevice->RegsRead(hDevice, AMI304_I2C_YOUT_L, &LocalInfo, 1);
			//printk("%s -7 \n", __FUNCTION__);
			break;
		case AMI304_I2C_YOUT_H:
			hDevice->RegsRead(hDevice, AMI304_I2C_YOUT_H, &LocalInfo, 1);
			//printk("%s -8 \n", __FUNCTION__);
			break;
		case AMI304_I2C_ZOUT_L:
			hDevice->RegsRead(hDevice, AMI304_I2C_ZOUT_L, &LocalInfo, 1);
			//printk("%s -9 \n", __FUNCTION__);
			break;
		case AMI304_I2C_ZOUT_H:
			hDevice->RegsRead(hDevice, AMI304_I2C_ZOUT_H, &LocalInfo, 1);
			//printk("%s -10 \n", __FUNCTION__);
			break;
		default:
			printk("NV ODM COMPASS NvOdmCompassGetParameter DONT SUPPORT SUCH ATTRIBUTE ---\n");
			return NV_FALSE;
#else

#endif

	}
	*info = LocalInfo;
	printk(" *info : %6x", *info);
	return NV_TRUE;

}

void NvCompassSetPowerRail(NvOdmServicesPmuHandle hPMUDevice, NvU32 Id, NvBool IsEnable)
{
	NvOdmServicesPmuVddRailCapabilities vddrailcap;
	NvU32 settletime;

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
#if 1
void NvCompassGetInterruptSouce(NvOdmCompassHandle hDevice,
		NvOdmCompassIntType  *IntType,
		NvOdmCompassAxisType *IntMotionAxis)
{
	NvU32    reg_val ;

	NV_ASSERT(hDevice != 0);
	NV_ASSERT(IntType != 0);
	NV_ASSERT(IntMotionAxis != 0);

	*IntType = NvOdmAccelInt_None;
	*IntMotionAxis = NvOdmAccelAxis_None;

	if (NULL != hDevice) {
		NvOdmCompassGetParameter(hDevice, KXTF9_I2C_INT_SRC_REG2, &reg_val);
#if DEBUG_LOG
		printk(" ## KXTF9 : KXTF9_I2C_INT_SRC_REG2 = %x ## \n", reg_val) ;
#endif
		if (reg_val & 0x01) {  // TPS : Screen Rotation
			*IntType |= NvOdmAccelInt_MotionThreshold;
		}
	}

}
#endif

	static void
GpioInterruptHandler(void *arg)
{
	NvOdmGpioPinMode mode;

	NvU32 pinValue = 0;
	NvOdmCompassHandle hDevice =  (NvOdmCompassHandle)arg;

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

NvBool NvCompassConnectSemaphore(NvOdmCompassHandle hDevice)
{
	NvOdmGpioPinMode mode;
	NvOdmInterruptHandler callback = (NvOdmInterruptHandler)GpioInterruptHandler;

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
 * Initialize I2C for compass.
 */
NvBool NvCompassI2COpen(NvOdmServicesI2cHandle* hI2CDevice, NvU32 id)
{
	// Open I2C handle.

	*hI2CDevice  =  NvOdmI2cPinMuxOpen(NvOdmIoModule_I2c, 1, NvOdmI2cPinMap_Config2);
	//	*hI2CDevice  =  NvOdmI2cOpen(NvOdmIoModule_I2c, 1);
	//printk(" ## NvCompassI2COpen %d %d ##\n", hI2CDevice.hRmDev.ChipId.Id);

	if (*hI2CDevice == NULL) {
		printk(" ## AMI304 NvCompassI2COpen Error ##\n");
		return NV_FALSE;
	}

	return NV_TRUE;
}

/*
 * De-initialize I2C for compass.
 */
void NvCompassI2CClose(NvOdmServicesI2cHandle hI2CDevice)
{
	// Close I2C handle.
	if (NULL != hI2CDevice) {
		NvOdmI2cClose(hI2CDevice);
	}
}

/*
 * Write I2C register function.
 * offset[Input]: I2C register offset of compass.
 * value[Input]: register value you will write.
 * len[Input]: requested bytes.
 * Returns NV_TRUE if successful, or NV_FALSE otherwise.
 */

NvBool NvCompassI2CSetRegs(NvOdmCompassHandle hDevice, NvU8 offset, NvU8* value, NvU32 len)
{
        int i;
        NvOdmI2cStatus i2c_status = NvOdmI2cStatus_Timeout;
	NvOdmI2cTransactionInfo TransactionInfo;

	if ((NULL == hDevice) || (NULL == value) || (len > I2C_COMPASS_PACKET_SIZE-1)) {
		return NV_FALSE;
	}

        for (i = 0; i < COMPASS_I2C_MAX_RETRY && i2c_status != NvOdmI2cStatus_Success; i++)
        {
	    NvOdmOsMemset(s_WriteBuffer, 0, sizeof(s_WriteBuffer));
	    s_WriteBuffer[0] = offset;
	    NvOdmOsMemcpy(&s_WriteBuffer[1], value, len);

	    TransactionInfo.Address = hDevice->nDevAddr;
#if DEBUG_LOG
	    printk("##   TransactionInfo.Address  %x\n", hDevice->nDevAddr );
#endif

	    TransactionInfo.Buf = s_WriteBuffer;
	    TransactionInfo.Flags = NVODM_I2C_IS_WRITE;
	    TransactionInfo.NumBytes = len+1;

	    i2c_status = NvOdmI2cTransaction(hDevice->hOdmI2C, &TransactionInfo, 1, 400, I2C_COMPASS_TRANSACTION_TIMEOUT);
        }

        if (i2c_status != NvOdmI2cStatus_Success )
        {
            printk("[star compass driver] %s : i2c transaction error(Number = %d)!\n", __func__, i2c_status);

            //reboot sensors
            reboot = 1;
			
            return NV_FALSE;
        }

	return NV_TRUE;
}

/*
 * Read I2C register function.
 * offset[Input]: I2C register offset of compass.
 * value[Output]: Fegister value you get.
 * len[Input]: Requested bytes.
 * Returns NV_TRUE if successful, or NV_FALSE otherwise.
 */
NvBool NvCompassI2CGetRegs(NvOdmCompassHandle hDevice, NvU8 offset, NvU8* value, NvU32 len)
{
        int i;
        NvOdmI2cStatus i2c_status = NvOdmI2cStatus_Timeout;
	NvOdmI2cTransactionInfo TransactionInfo[2];

	if ((NULL == hDevice) || (NULL == value) || (len > I2C_COMPASS_PACKET_SIZE-1)) {
		return NV_FALSE;
	}

        for (i = 0; i < COMPASS_I2C_MAX_RETRY && i2c_status != NvOdmI2cStatus_Success; i++)
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
	    // Write the accelerometor offset (from where data is to be read).
	    i2c_status = NvOdmI2cTransaction(hDevice->hOdmI2C, TransactionInfo, 2, 400, I2C_COMPASS_TRANSACTION_TIMEOUT);
        }

	//Read the data from the eeprom at the specified offset
	if (i2c_status != NvOdmI2cStatus_Success)
	{
		printk("Error : NvCompassI2CGetRegs -2n" );

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
NvOdmCompassOpen(NvOdmCompassHandle* hDevice)
{
	NvU32 i;
	NvBool foundGpio = NV_FALSE, foundI2cModule = NV_FALSE;
	const NvOdmPeripheralConnectivity *pConnectivity;
	NvOdmCompassHandle  hCompass;
	NvU32    reg_val = 0 ;

	////////////////////////////////////////////////////// Hyunjin.Kim
	hCompass = NvOdmOsAlloc(sizeof(NvOdmCompass));
	if (hCompass == NULL) {
		//NVODMCOMPASS_PRINTF("Error Allocating NvOdmCompass. \n");
		return NV_FALSE;
	}
	NvOdmOsMemset(hCompass, 0, sizeof(NvOdmCompass));


	hCompass->hPmu = NULL;
	hCompass->hOdmI2C =  NULL;

	hCompass->hPmu = NvOdmServicesPmuOpen();
	if (!hCompass->hPmu) {
		//NVODMACCELEROMETER_PRINTF("NvOdmServicesPmuOpen Error \n");
		goto error;
	}

	pConnectivity = (NvOdmPeripheralConnectivity*)NvOdmPeripheralGetGuid(NV_ODM_GUID('c','o','m','p','a','s','s','-'));
	if (!pConnectivity) {
		printk("NvOdmPeripheralGetGuid doesn't detect compass device\n");
		goto error;
	}

	if (pConnectivity->Class != NvOdmPeripheralClass_Other) {
		goto error;
	}

	for (i = 0; i < pConnectivity->NumAddress; i++) {
		switch(pConnectivity->AddressList[i].Interface) {
			case NvOdmIoModule_I2c:
				hCompass->I2CChannelId = pConnectivity->AddressList[i].Instance;
				printk("## AMI304 (pConnectivity->AddressList[i].Address  i2c address = %x. (1)##\n", pConnectivity->AddressList[i].Address);
				hCompass->nDevAddr = (pConnectivity->AddressList[i].Address  << 1 ) ;
				foundI2cModule = NV_TRUE;
#if DEBUG_LOG
				printk("## AMI304 I2CChannelId = %x. ## \n", hCompass->I2CChannelId);
				printk("## AMI304 i2c address = %x. (2)##\n", hCompass->nDevAddr);
#endif
				break;
			case NvOdmIoModule_Gpio:
				hCompass->GPIOPortINT = pConnectivity->AddressList[i].Instance;
				hCompass->GPIOPinINT = pConnectivity->AddressList[i].Address;
				foundGpio = NV_TRUE;
#if DEBUG_LOG
				printk("## AMI304 GPIOPortINT = %x. ## \n",hCompass->GPIOPortINT);
				printk("## AMI304 GPIOPinINT = %x. ## \n", hCompass->GPIOPinINT);
#endif
				break;
			case NvOdmIoModule_Vdd:
				hCompass->VddId = pConnectivity->AddressList[i].Address;
#if DEBUG_LOG
				printk("## AMI304 NvOdmIoModule_VddId = %x. ## \n", hCompass->VddId);
#endif
				// Power on accelerometer according to Vddid
				NvCompassSetPowerRail(hCompass->hPmu, hCompass->VddId, NV_TRUE);
				msleep(1);
				//				mdelay(1000);

				break;
			default:
				break;
		}
	}

	if (foundGpio != NV_TRUE || foundI2cModule != NV_TRUE) {
#if DEBUG_LOG
		//NVODMACCELEROMETER_PRINTF("Accelerometer : didn't find any periperal in discovery query for touch device Error \n");
		printk("## AMI304 didn't find any periperal in discovery query for touch device Error \n");
#endif
		goto error;
	}

	// Set up I2C bus.
	if (NV_FALSE == NvCompassI2COpen(&hCompass->hOdmI2C, hCompass->I2CChannelId)) {
#if DEBUG_LOG
		printk("## AMI304 Set up I2C bus: Error \n");
#endif
		goto error;
	};

	//printk("################ AMI304 Set up I2C bus: No Error-1 \n");
	hCompass->RegsRead = NvCompassI2CGetRegs;
	hCompass->RegsWrite = NvCompassI2CSetRegs;
	//printk("################ AMI304 Set up I2C bus: No Error-2 \n");

	if (NV_FALSE == NvCompassConnectSemaphore(hCompass)) {
#if DEBUG_LOG
		printk("## AMI304 NvCompassConnectSemaphore(hCompass): Error \n");
#endif
		goto error;
	}

	*hDevice = hCompass;
	return NV_TRUE;
error:
	// Release all of resources requested.
	if (NULL != hCompass) {
		NvCompassSetPowerRail(hCompass->hPmu, hCompass->VddId, NV_FALSE);
		NvOdmServicesPmuClose(hCompass->hPmu);
		hCompass->hPmu = NULL;
		NvCompassI2CClose(hCompass->hOdmI2C);
		hCompass->hOdmI2C = NULL;
		NvOdmOsFree(hCompass);
		*hDevice = NULL;
	}
	return NV_FALSE;
}

void
NvOdmCompassClose(NvOdmCompassHandle hDevice)
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
		NvCompassI2CClose(hDevice->hOdmI2C);

		// Power off accelermeter
		NvCompassSetPowerRail(hDevice->hPmu, hDevice->VddId, NV_FALSE);
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
NvOdmCompassSetIntForceThreshold(NvOdmCompassHandle  hDevice,
		NvOdmCompassIntType IntType,
		NvU32             IntNum,
		NvU32             Threshold)
{
	return NV_TRUE;
}

/*
 * After setting the time threshold, we should remove all of interrupt flag
 * that may be left from the last threshold.
 */

NvBool
NvOdmCompassSetIntTimeThreshold(NvOdmCompassHandle	hDevice,
		NvOdmCompassIntType IntType,
		NvU32 			IntNum,
		NvU32 			Threshold)
{
	return NV_TRUE;
}

/*
 * After enable/disable threshold, we should remove all of interrupt flag
 * that may be left from that last threshold.
 */
NvBool
NvOdmCompassSetIntEnable(NvOdmCompassHandle  hDevice,
		NvOdmCompassIntType  IntType,
		NvOdmCompassAxisType IntAxis,
		NvU32              IntNum,
		NvBool             Toggle)
{
	NvU32 uTemp = 0;
	NV_ASSERT(NULL != hDevice);

	switch(IntType) {
		case NvOdmAccelInt_MotionThreshold:
			NvOdmCompassGetParameter(hDevice, KIONIX_ACCEL_I2C_INT_CTRL_REG2, &uTemp);
			//NvOdmOsDebugPrintf("INTCONTROL is 0x%x g\n", uTemp);
			switch(IntAxis) {
				case NvOdmAccelAxis_X:
					{
						if (Toggle == NV_TRUE) {
							uTemp |= INT_CTRL_REG2_XBW;
						} else {
							uTemp &= INT_CTRL_REG2_XBW_MASK;
						}
						break;
					}
				case NvOdmAccelAxis_Y:
					{
						if (Toggle == NV_TRUE) {
							uTemp |= INT_CTRL_REG2_YBW;
						} else {
							uTemp &= INT_CTRL_REG2_YBW_MASK;
						}
						break;
					}
				case NvOdmAccelAxis_Z:
					{
						if (Toggle == NV_TRUE) {
							uTemp |= INT_CTRL_REG2_ZBW;
						} else {
							uTemp &= INT_CTRL_REG2_ZBW_MASK;
						}
						break;
					}
				case NvOdmAccelAxis_All:
					{
						if (Toggle == NV_TRUE) {
							uTemp |= INT_CTRL_REG2_XBW;
							uTemp |= INT_CTRL_REG2_YBW;
							uTemp |= INT_CTRL_REG2_ZBW;
						} else {
							uTemp &= INT_CTRL_REG2_XBW_MASK;
							uTemp &= INT_CTRL_REG2_YBW_MASK;
							uTemp &= INT_CTRL_REG2_ZBW_MASK;
						}
						break;
					}
				default:
					return NV_FALSE;
			}
			NvOdmCompassSetParameter(hDevice, KIONIX_ACCEL_I2C_INT_CTRL_REG2, uTemp);
			break;
		default:
			//NVODMACCELEROMETER_PRINTF("Do not support such Interrupt!\n");
			return NV_FALSE;
			break ;
	}

	NvOdmCompassGetParameter(hDevice, KXTF9_I2C_INT_CTRL_REG1, &uTemp);
	uTemp |= KXTF9_INT_CTRL_REG1_IEN;                    // enables the physical interrupt.
#if DEBUG_LOG
	printk(" ## KXTF9 _ NvOdmAccelSetIntEnable : uTemp = %x ## \n", uTemp) ;
#endif
	NvOdmCompassSetParameter(hDevice, KXTF9_I2C_INT_CTRL_REG1, uTemp);
	return NV_TRUE;

}

void
NvOdmCompassWaitInt(NvOdmCompassHandle hDevice,
		NvOdmCompassIntType  *IntType,
		NvOdmCompassAxisType *IntMotionAxis)
{
	NvU32 uTemp;
	NV_ASSERT(NULL != hDevice);
	NV_ASSERT(NULL != IntType);
	NV_ASSERT(NULL != IntMotionAxis);

	NvOdmOsSemaphoreWait( hDevice->SemaphoreForINT);

	// disables the physical interrupt.
	NvOdmCompassGetParameter(hDevice, KXTF9_I2C_INT_CTRL_REG1, &uTemp);
	uTemp &= KXTF9_INT_CTRL_REG1_IEN_MASK;
	NvOdmCompassSetParameter(hDevice, KXTF9_I2C_INT_CTRL_REG1, uTemp);

	NvCompassGetInterruptSouce( hDevice , IntType, IntMotionAxis);

	// clear the interrupt source information along with interrupt pin
	NvOdmCompassGetParameter(hDevice, KIONIX_ACCEL_I2C_INT_REL, &uTemp);

	// enables the physical interrupt again.
	NvOdmCompassGetParameter(hDevice, KXTF9_I2C_INT_CTRL_REG1, &uTemp);
	uTemp |= KXTF9_INT_CTRL_REG1_IEN;
	NvOdmCompassSetParameter(hDevice, KXTF9_I2C_INT_CTRL_REG1, uTemp);

	return ;
}


void NvOdmCompassSignal(NvOdmCompassHandle hDevice)
{
	NvOdmOsSemaphoreSignal(hDevice->SemaphoreForINT);
}


NvBool
NvOdmCompassGetMagnetic(NvOdmCompassHandle hDevice,
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

	if (NvOdmCompassGetParameter(hDevice, AMI304_REG_DATAXH, &data_H) == NV_TRUE)
	{
		*AccelX = ((NvS32)0xff& ( data_H >> 16)) ;
		x_sign = *AccelX >> 7 ; 	 // 1 : negative, 0 : positive
		if (x_sign == 1) {
			*AccelX = ((~(*AccelX) + 0x01) & 0x0FF);  // 2's complement
			*AccelX = -(*AccelX);
		}
		*AccelY = ((NvS32)0x00ff& ( data_H >> 8)) ;
		x_sign = *AccelY >> 7 ; 	 // 1 : negative, 0 : positive
		if (x_sign == 1) {
			*AccelY = ((~(*AccelY) + 0x01) & 0x0FF);  // 2's complement
			*AccelY = -(*AccelY);
		}/*
		  *AccelZ = ((NvS32)0x0000ff& ( data_H )) ;
		  x_sign = *AccelZ >> 7 ; 	 // 1 : negative, 0 : positive
		  if( x_sign == 1 )
		  {
		  *AccelZ = ((~(*AccelZ) + 0x01) & 0x0FF);  // 2's complement
		  *AccelZ = -(*AccelZ);
		  }*/
	}

	return NV_TRUE;
}


#if 0
	NvOdmCompassCaps
NvOdmCompGetCaps(NvOdmCompassHandle hDevice)
{
	NV_ASSERT(NULL != hDevice);

	return hDevice->Caption;
}
#endif
/*
 * This function sets the low pass filter roll off for the compass outputs.
 * SampleRate[Input]: roll off frequency (6, 12, 25, 50, 100, 200, 400)
 * Returns NV_TRUE if successful, or NV_FALSE otherwise.
 * Don't need !!!!!
 */

NvBool
NvOdmCompassSetSampleRate(NvOdmCompassHandle hDevice, NvU32 SampleRate)
{
	NV_ASSERT(NULL != hDevice);

	switch (SampleRate) {
		case 12 :      /* set ( Output data rate : 12.5 Hz / LPF rolloff : 6.25 Hz ) */
			NvOdmCompassSetParameter(hDevice, KXTF9_I2C_DATA_CTRL_REG, 0x00);
			break ;
		case 25 : 	   /* set ( Output data rate : 25 Hz / LPF rolloff : 12.5 Hz ) */
			NvOdmCompassSetParameter(hDevice, KXTF9_I2C_DATA_CTRL_REG, 0x01);
			break ;
		case 50 : 	   /* set ( Output data rate : 50 Hz / LPF rolloff : 25 Hz ) */
			NvOdmCompassSetParameter(hDevice, KXTF9_I2C_DATA_CTRL_REG, 0x02);
			break ;
		case 100 : 	   /* set ( Output data rate : 100 Hz / LPF rolloff : 50 Hz ) */
			NvOdmCompassSetParameter(hDevice, KXTF9_I2C_DATA_CTRL_REG, 0x03);
			break ;
		case 200 : 	   /* set ( Output data rate : 200 Hz / LPF rolloff : 100 Hz ) */
			NvOdmCompassSetParameter(hDevice, KXTF9_I2C_DATA_CTRL_REG, 0x04);
			break ;
		case 400 : 	   /* set ( Output data rate : 400 Hz / LPF rolloff : 200 Hz ) */
			NvOdmCompassSetParameter(hDevice, KXTF9_I2C_DATA_CTRL_REG, 0x05);
			break ;
		case 800 : 	   /* set ( Output data rate : 800 Hz / LPF rolloff : 400 Hz ) */
			NvOdmCompassSetParameter(hDevice, KXTF9_I2C_DATA_CTRL_REG, 0x06);
			break ;
		default:
			printk(" ## NvOdmAccelSetSampleRate ERROR : No Type ## \n");
			break;
	}
	return NV_TRUE;
}


NvBool
NvOdmCompassGetSampleRate(NvOdmCompassHandle hDevice, NvU32 *pSampleRate)
{
	NvU32 CurrentIndex = 0;
	NvU32 data_ctrl_reg = 0 ;
	/* This code dont need. */

	NV_ASSERT(NULL != hDevice);

	if (NvOdmCompassGetParameter(hDevice, KXTF9_I2C_DATA_CTRL_REG, &data_ctrl_reg) != NV_TRUE)
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
			printk(" ## NvOdmCompassGetSampleRate ERROR : No Type ## \n");
			break;
	}
	return NV_TRUE;
}

NvBool
NvOdmCompassSetPowerState(NvOdmCompassHandle hDevice, NvOdmCompassPowerType PowerState)
{
	return NV_TRUE;
}

