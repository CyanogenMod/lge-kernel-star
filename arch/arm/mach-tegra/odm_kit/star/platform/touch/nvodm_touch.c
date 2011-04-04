/*
 * Copyright (c) 2006-2009 NVIDIA Corporation.
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

#include "nvodm_touch.h"
#include "nvodm_touch_int.h"

// 20100414 joseph.jung@lge.com LGE Touch Customization [START]
#include "nvodm_touch_synaptics.h"
#include "nvodm_touch_cypress.h"
// 20100414 joseph.jung@lge.com  LGE Touch Customization [END]


// 20100414 joseph.jung@lge.com LGE Touch Dual Support - Synaptics & Cypress [START]
#ifdef FEATURE_LGE_TOUCH_DUAL_SUPPORT
#include "../../adaptations/pmu/max8907/max8907_supply_info_table.h"

NvU32 pinValue = 0xFFFFFFFF;
#endif /* FEATURE_LGE_TOUCH_DUAL_SUPPORT */
// 20100402 joseph.jung@lge.com LGE Touch Dual Support - Synaptics & Cypress [END]


/** Implementation for the NvOdm TouchPad */

NvBool
NvOdmTouchDeviceOpen( NvOdmTouchDeviceHandle *hDevice, NvOdmOsSemaphoreHandle* hIntSema )
{
    NvBool ret = NV_TRUE;

// 20100414 joseph.jung@lge.com LGE Touch Dual Support - Synaptics & Cypress [START]
#ifdef FEATURE_LGE_TOUCH_DUAL_SUPPORT
	printk("[TOUCH] Touch maker gpio pin value = %d\n", pinValue);

	if(pinValue == 0xFFFFFFFF)
	{
		NvOdmServicesPmuHandle hPmu;
		NvU32 settletime;
		
		hPmu = NvOdmServicesPmuOpen();

		if(!hPmu)
		{
			printk("[TOUCH] NvOdmServicesPmuOpen Error\n");
			return NV_FALSE;
		}

		NvOdmServicesPmuSetVoltage(hPmu, Max8907PmuSupply_LDO19, MAX8907_REQUESTVOLTAGE_LDO19, &settletime);

		if (settletime)
			NvOdmOsWaitUS(settletime);

		NvOdmGpioGetState(touch_maker_gpio, touch_maker_pin, &pinValue);

		printk("[Touch Driver] Touch maker gpio pin value = %d\n", pinValue);

		NvOdmServicesPmuSetVoltage(hPmu, Max8907PmuSupply_LDO19, NVODM_VOLTAGE_OFF, &settletime);
	    if (settletime)
			NvOdmOsWaitUS(settletime);
			
		NvOdmServicesPmuClose(hPmu);
	}

	if (!pinValue)
    	ret = Synaptics_Open(hDevice, hIntSema);
	else
    	ret = Cypress_Open(hDevice, hIntSema);
#else
	ret = Synaptics_Open(hDevice, hIntSema);
#endif /* FEATURE_LGE_TOUCH_DUAL_SUPPORT */
// 20100402 joseph.jung@lge.com LGE Touch Dual Support - Synaptics & Cypress [END]

    return ret;
}


void
NvOdmTouchDeviceGetCapabilities(NvOdmTouchDeviceHandle hDevice, NvOdmTouchCapabilities* pCapabilities)
{
    hDevice->GetCapabilities(hDevice, pCapabilities);
}


NvBool
NvOdmTouchReadCoordinate( NvOdmTouchDeviceHandle hDevice, NvOdmTouchCoordinateInfo *coord)
{
    return hDevice->ReadCoordinate(hDevice, coord);
}

NvBool
NvOdmTouchGetSampleRate(NvOdmTouchDeviceHandle hDevice, NvOdmTouchSampleRate* pTouchSampleRate)
{
    return hDevice->GetSampleRate(hDevice, pTouchSampleRate);
}

void NvOdmTouchDeviceClose(NvOdmTouchDeviceHandle hDevice)
{
    hDevice->Close(hDevice);    
}

NvBool NvOdmTouchEnableInterrupt(NvOdmTouchDeviceHandle hDevice, NvOdmOsSemaphoreHandle hInterruptSemaphore)
{
    return hDevice->EnableInterrupt(hDevice, hInterruptSemaphore);
}

NvBool NvOdmTouchHandleInterrupt(NvOdmTouchDeviceHandle hDevice)
{
    return hDevice->HandleInterrupt(hDevice);
}

NvBool
NvOdmTouchSetSampleRate(NvOdmTouchDeviceHandle hDevice, NvU32 SampleRate)
{
    return hDevice->SetSampleRate(hDevice, SampleRate);
}


NvBool
NvOdmTouchPowerControl(NvOdmTouchDeviceHandle hDevice, NvOdmTouchPowerModeType mode)
{
    return hDevice->PowerControl(hDevice, mode);
}

void
NvOdmTouchPowerOnOff(NvOdmTouchDeviceHandle hDevice, NvBool OnOff)
{
    hDevice->PowerOnOff(hDevice, OnOff);
}


NvBool
NvOdmTouchOutputDebugMessage(NvOdmTouchDeviceHandle hDevice)
{
    return hDevice->OutputDebugMessage;
}

NvBool
NvOdmTouchGetCalibrationData(NvOdmTouchDeviceHandle hDevice, NvU32 NumOfCalibrationData, NvS32* pRawCoordBuffer)
{
    return hDevice->GetCalibrationData(hDevice, NumOfCalibrationData, pRawCoordBuffer);
}

// 20101020 joseph.jung@lge.com Interrupt Enable/Disable [START]
void
NvOdmTouchInterruptMask(NvOdmTouchDeviceHandle hDevice, NvBool mask)
{
	hDevice->InterruptMask(hDevice, mask);
}
// 20101020 joseph.jung@lge.com Interrupt Enable/Disable [END]


// 20100718 joseph.jung@lge.com grip suppression [START]
NvU8 touch_grip_suppression_value = 0;

void setTouchGripSuppressionValue(int value)
{
	touch_grip_suppression_value = (NvU8)value;
}

int getTouchGripSuppressionValue(void)
{
	return (int)touch_grip_suppression_value;
}
// 20100718 joseph.jung@lge.com grip suppression [END]

// 20100906 joseph.jung@lge.com Touch F/W version [START]
int touch_fw_version = 0;

void storeTouchFWversion(int value)
{
	touch_fw_version = value;
}

int showTouchFWversion(void)
{
	return touch_fw_version;
}
// 20100906 joseph.jung@lge.com Touch F/W version [END]

