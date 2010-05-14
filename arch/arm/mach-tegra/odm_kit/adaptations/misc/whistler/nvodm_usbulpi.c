/*
 * Copyright (c) 2009 NVIDIA Corporation.
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

/**
 * @file          nvodm_usbulpi.c
 * @brief         <b>Adaptation for USB ULPI </b>
 *
 * @Description : Implementation of the USB ULPI adaptation.
 */
#include "nvodm_usbulpi.h"
#include "nvodm_services.h"
#include "nvodm_query_discovery.h"
#include "nvodm_services.h"
#include "nvodm_keylist_reserved.h"
#include "nvrm_drf.h"
#include "nvos.h"

typedef struct NvOdmUsbUlpiRec {
	NvU64 CurrentGUID;
} NvOdmUsbUlpi;

/* ST-Ericsson U3XX modem power control */
struct ste_u3xx_info {
	NvU32 ste_u3xx_uart_port;
	NvU32 ste_u3xx_reset_port;
	NvU32 ste_u3xx_reset_pin;
	NvU32 ste_u3xx_power_port;
	NvU32 ste_u3xx_power_pin;
	NvU32 ste_u3xx_awr_port;
	NvU32 ste_u3xx_awr_pin;
	NvU32 ste_u3xx_cwr_port;
	NvU32 ste_u3xx_cwr_pin;
	NvU32 ste_u3xx_spi_int_port;
	NvU32 ste_u3xx_spi_int_pin;
	NvU32 ste_u3xx_slave_select_port;
	NvU32 ste_u3xx_slave_select_pin;
	NvU32 ste_u3xx_slink_instance;
};

/* ST-Ericsson U3XX modem control */
static struct ste_u3xx_info ste_u3xx_info;
static NvOdmServicesGpioHandle ste_u3xx_gpio;
static NvOdmGpioPinHandle ste_u3xx_reset_gpio_pin;
static NvOdmGpioPinHandle ste_u3xx_power_gpio_pin;
static NvOdmGpioPinHandle ste_u3xx_awr_gpio_pin;
static NvOdmGpioPinHandle ste_u3xx_cwr_gpio_pin;

static int ste_u3xx_query(struct ste_u3xx_info *info)
{
	NvU64 guid = NV_ODM_GUID('e', 'm', 'p', ' ', 'M', '5', '7', '0');
	NvOdmPeripheralConnectivity *pConnectivity;

	/* query odm kit for modem support */
	pConnectivity =
	    (NvOdmPeripheralConnectivity *) NvOdmPeripheralGetGuid(guid);
	if (pConnectivity == NULL)
		return -1;
	NV_ASSERT(pConnectivity->NumAddress >= 5);

	/* query for uart port */
	NV_ASSERT(pConnectivity->AddressList[0].Interface ==
		  NvOdmIoModule_Uart);
	info->ste_u3xx_uart_port = pConnectivity->AddressList[0].Instance;

	/* query for reset pin */
	NV_ASSERT(pConnectivity->AddressList[1].Interface ==
		  NvOdmIoModule_Gpio);
	info->ste_u3xx_reset_port = pConnectivity->AddressList[1].Instance;
	info->ste_u3xx_reset_pin = pConnectivity->AddressList[1].Address;

	/* query for power pin */
	NV_ASSERT(pConnectivity->AddressList[2].Interface ==
		  NvOdmIoModule_Gpio);
	info->ste_u3xx_power_port = pConnectivity->AddressList[2].Instance;
	info->ste_u3xx_power_pin = pConnectivity->AddressList[2].Address;

	/* query for ACPU wakeup request pin */
	NV_ASSERT(pConnectivity->AddressList[3].Interface ==
		  NvOdmIoModule_Gpio);
	info->ste_u3xx_awr_port = pConnectivity->AddressList[3].Instance;
	info->ste_u3xx_awr_pin = pConnectivity->AddressList[3].Address;

	/* query for CCPU wakeup request pin */
	NV_ASSERT(pConnectivity->AddressList[4].Interface ==
		  NvOdmIoModule_Gpio);
	info->ste_u3xx_cwr_port = pConnectivity->AddressList[4].Instance;
	info->ste_u3xx_cwr_pin = pConnectivity->AddressList[4].Address;

	return 0;
}

static void ste_u3xx_turn_on_modem(struct ste_u3xx_info *info)
{
	/* get odm gpio handle */
	ste_u3xx_gpio = NvOdmGpioOpen();
	if (!ste_u3xx_gpio)
		return;

	/* acquire pin handle for reset pin */
	ste_u3xx_reset_gpio_pin =
	    NvOdmGpioAcquirePinHandle(ste_u3xx_gpio, info->ste_u3xx_reset_port,
				      info->ste_u3xx_reset_pin);
	if (!ste_u3xx_reset_gpio_pin) {
		NvOdmGpioClose(ste_u3xx_gpio);
		return;
	}

	/* acquire pin handle for power pin */
	ste_u3xx_power_gpio_pin =
	    NvOdmGpioAcquirePinHandle(ste_u3xx_gpio, info->ste_u3xx_power_port,
				      info->ste_u3xx_power_pin);
	if (!ste_u3xx_power_gpio_pin) {
		NvOdmGpioReleasePinHandle(ste_u3xx_gpio,
					  ste_u3xx_reset_gpio_pin);
		NvOdmGpioClose(ste_u3xx_gpio);
		return;
	}

	/* acquire pin handle for ACPU wakeup request pin */
	ste_u3xx_awr_gpio_pin =
	    NvOdmGpioAcquirePinHandle(ste_u3xx_gpio, info->ste_u3xx_awr_port,
				      info->ste_u3xx_awr_pin);
	if (!ste_u3xx_awr_gpio_pin) {
		NvOdmGpioReleasePinHandle(ste_u3xx_gpio,
					  ste_u3xx_power_gpio_pin);
		NvOdmGpioReleasePinHandle(ste_u3xx_gpio,
					  ste_u3xx_reset_gpio_pin);
		NvOdmGpioClose(ste_u3xx_gpio);
		return;
	}

	/* acquire pin handle for CCPU wakeup request pin */
	ste_u3xx_cwr_gpio_pin =
	    NvOdmGpioAcquirePinHandle(ste_u3xx_gpio, info->ste_u3xx_cwr_port,
				      info->ste_u3xx_cwr_pin);
	if (!ste_u3xx_cwr_gpio_pin) {
		NvOdmGpioReleasePinHandle(ste_u3xx_gpio, ste_u3xx_awr_gpio_pin);
		NvOdmGpioReleasePinHandle(ste_u3xx_gpio,
					  ste_u3xx_power_gpio_pin);
		NvOdmGpioReleasePinHandle(ste_u3xx_gpio,
					  ste_u3xx_reset_gpio_pin);
		NvOdmGpioClose(ste_u3xx_gpio);
		return;
	}

	/* set output levels - start with modem power off, reset deasserted */
	NvOdmGpioSetState(ste_u3xx_gpio, ste_u3xx_power_gpio_pin, 0);
	NvOdmGpioSetState(ste_u3xx_gpio, ste_u3xx_reset_gpio_pin, 0);
	NvOdmGpioConfig(ste_u3xx_gpio, ste_u3xx_power_gpio_pin,
			NvOdmGpioPinMode_Output);
	NvOdmGpioConfig(ste_u3xx_gpio, ste_u3xx_reset_gpio_pin,
			NvOdmGpioPinMode_Output);
	NvOdmGpioConfig(ste_u3xx_gpio, ste_u3xx_cwr_gpio_pin,
			NvOdmGpioPinMode_InputData);

	NvOdmOsSleepMS(300);
	NvOdmGpioSetState(ste_u3xx_gpio, ste_u3xx_reset_gpio_pin, 1);

	/* pulse modem power on for 300 ms */
	NvOdmOsSleepMS(300);
	NvOdmGpioSetState(ste_u3xx_gpio, ste_u3xx_power_gpio_pin, 1);
	NvOdmOsSleepMS(300);
	NvOdmGpioSetState(ste_u3xx_gpio, ste_u3xx_power_gpio_pin, 0);
	NvOdmOsSleepMS(100);

	NvOdmGpioReleasePinHandle(ste_u3xx_gpio, ste_u3xx_cwr_gpio_pin);
	NvOdmGpioReleasePinHandle(ste_u3xx_gpio, ste_u3xx_awr_gpio_pin);
	NvOdmGpioReleasePinHandle(ste_u3xx_gpio, ste_u3xx_power_gpio_pin);
	NvOdmGpioReleasePinHandle(ste_u3xx_gpio, ste_u3xx_reset_gpio_pin);
	NvOdmGpioClose(ste_u3xx_gpio);
}

NvOdmUsbUlpiHandle NvOdmUsbUlpiOpen(NvU32 Instance)
{
	const NvOdmUsbProperty *pUsbProperty =
		NvOdmQueryGetUsbProperty(NvOdmIoModule_Usb, Instance);
	NvOdmUsbUlpi *pDevice = NULL;

	pDevice = NvOdmOsAlloc(sizeof(NvOdmUsbUlpi));
	if (pDevice == NULL)
		goto ExitUlpiOdm;

	if (pUsbProperty->UsbInterfaceType ==
		NvOdmUsbInterfaceType_UlpiNullPhy) {
		/* query the modem control pins */
		if (ste_u3xx_query(&ste_u3xx_info) < 0)
			goto ExitUlpiOdm;

		NvOsDebugPrintf("turn modem on\n");
		ste_u3xx_turn_on_modem(&ste_u3xx_info);
	}
	return pDevice;

ExitUlpiOdm:
	if (pDevice)
		NvOdmOsFree(pDevice);
	return NULL;
}

void NvOdmUsbUlpiClose(NvOdmUsbUlpiHandle hOdmUlpi)
{
	if (hOdmUlpi) {
		NvOdmOsFree(hOdmUlpi);
		hOdmUlpi = NULL;
	}
}
