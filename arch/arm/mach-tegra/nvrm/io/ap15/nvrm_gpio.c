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

#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <mach/gpio.h>

#include "nvrm_gpio.h"
#include "nvos.h"
#include "nvrm_structure.h"
#include "nvrm_pmu.h"
#include "nvrm_pinmux_utils.h"
#include "ap15/ap15rm_private.h"
#include "ap15/ap15rm_gpio_vi.h"
#include "nvodm_gpio_ext.h"
#include "nvodm_query_discovery.h"
#include "nvrm_hwintf.h"
#include "nvassert.h"

/* Treats GPIO pin handle releases like the pin is completely invalidated:
 * returned to SFIO state and tristated. */
#define RELEASE_IS_INVALIDATE 1
#define NV_ENABLE_GPIO_POWER_RAIL 1

#define TOTAL_GPIO_BANK    7
#define GPIO_PORT_PER_BANK 4
#define GPIO_PIN_PER_PORT  8
#define GPIO_PORT_ID(bank, port)  ((((bank)&0xFF) << 2) | (((port) & 0x3)))
#define GPIO_PIN_ID(bank, port, pin)  ((((bank)&0xFF) << 5) | \
					(((port) & 0x3) <<3) | ((pin) & 0x7))

#define GET_PIN(h)      ((((NvU32)(h))) & 0xFF)
#define GET_PORT(h)     ((((NvU32)(h)) >> 8) & 0xFF)
#define GET_BANK(h)     ((((NvU32)(h)) >> 16) & 0xFF)

#define NVRM_GPIO_CAP_FEAT_EDGE_INTR  0x000000001
#define GPIO_ARCH_FEATURE  (NVRM_GPIO_CAP_FEAT_EDGE_INTR)

extern int tegra_gpio_io_power_config(int gpio_nr, unsigned int enable);

typedef struct NvRmGpioPinInfoRec {
	NvBool used;
	NvU32 port;
	NvU32 inst;
	NvU32 pin;
	NvRmGpioPinMode mode;
	/* Sets up a chain of pins associated by one semaphore.
	 * Usefull to parse the pins when an interrupt is received. */
	NvU32 nextPin;
	NvU16 irqNumber;
} NvRmGpioPinInfo;

typedef struct NvRmGpioRec {
	NvU32 RefCount;
	NvRmDeviceHandle hRm;
	NvRmGpioPinInfo *pPinInfo;
} NvRmGpio;

static NvRmGpioHandle s_hGpio = NULL;
static NvOsMutexHandle s_GpioMutex = NULL;

NvError NvRmGpioOpen(NvRmDeviceHandle hRm, NvRmGpioHandle * phGpio)
{
	NvError err = NvSuccess;
	NvU32 total_pins;
	NvU32 i;

	NV_ASSERT(hRm);
	NV_ASSERT(phGpio);

	if (!s_GpioMutex) {
		err = NvOsMutexCreate(&s_GpioMutex);
		if (err != NvSuccess)
			goto fail;
	}

	NvOsMutexLock(s_GpioMutex);
	if (s_hGpio) {
		s_hGpio->RefCount++;
		goto exit;
	}

	s_hGpio = (NvRmGpio *) NvOsAlloc(sizeof(NvRmGpio));
	if (!s_hGpio) {
		err = NvError_InsufficientMemory;
		goto exit;
	}
	NvOsMemset(s_hGpio, 0, sizeof(NvRmGpio));
	s_hGpio->hRm = hRm;

	total_pins = TOTAL_GPIO_BANK * GPIO_PORT_PER_BANK * GPIO_PIN_PER_PORT;
	s_hGpio->pPinInfo = NvOsAlloc(sizeof(NvRmGpioPinInfo) * total_pins);
	if (s_hGpio->pPinInfo == NULL) {
		NvOsFree(s_hGpio);
		err = NvError_InsufficientMemory;
		s_hGpio = NULL;
		goto exit;
	}
	NvOsMemset(s_hGpio->pPinInfo, 0, sizeof(NvRmGpioPinInfo) * total_pins);
	for (i = 0; i < total_pins; i++)
		s_hGpio->pPinInfo[i].irqNumber = NVRM_IRQ_INVALID;
	s_hGpio->RefCount++;

exit:
	*phGpio = s_hGpio;
	NvOsMutexUnlock(s_GpioMutex);

fail:
	return err;
}

void NvRmGpioClose(NvRmGpioHandle hGpio)
{
	if (!hGpio)
		return;

	NV_ASSERT(hGpio->RefCount);

	NvOsMutexLock(s_GpioMutex);
	hGpio->RefCount--;
	if (hGpio->RefCount == 0) {
		NvOsFree(s_hGpio->pPinInfo);
		NvOsFree(s_hGpio);
		s_hGpio = NULL;
	}
	NvOsMutexUnlock(s_GpioMutex);
}

NvError
NvRmGpioAcquirePinHandle(NvRmGpioHandle hGpio,
			 NvU32 port, NvU32 pinNumber, NvRmGpioPinHandle * phPin)
{
	int gpio_nr;
	int ret_status;

	NV_ASSERT(hGpio != NULL);

	if (port == NVRM_GPIO_CAMERA_PORT) {
		NvOsMutexLock(s_GpioMutex);
		/* The Camera has dedicated gpio pins that must be controlled
		 * through a non-standard gpio port control. */
		NvRmPrivGpioViAcquirePinHandle(hGpio->hRm, pinNumber);
		*phPin = GPIO_MAKE_PIN_HANDLE(NVRM_GPIO_CAMERA_INST, port,
		 					pinNumber);
		NvOsMutexUnlock(s_GpioMutex);
	} else if ((port >= NVODM_GPIO_EXT_PORT_0) &&
		   (port <= NVODM_GPIO_EXT_PORT_F)) {
		/* Create a pin handle for GPIOs that are
		 * sourced by external (off-chip) peripherals */
		*phPin = GPIO_MAKE_PIN_HANDLE((port & 0xFF), port, pinNumber);
	} else {
		gpio_nr = GPIO_PIN_ID((port >> 2), (port & 0x3), pinNumber);
		if ((gpio_nr >= ARCH_NR_GPIOS) ||
			 (pinNumber >= GPIO_PIN_PER_PORT)) {
			printk(KERN_ERR "Requested port %d or pin %d  number "
					" is not supported", port, pinNumber);
			return NvError_NotSupported;
		}
		ret_status = gpio_request(gpio_nr, "nvrm_gpio");
		if (unlikely(ret_status != 0)) {
			return NvError_AlreadyAllocated;
		}
		*phPin = GPIO_MAKE_PIN_HANDLE((port >> 2), (port & 0x3),
						 pinNumber);
	}
	return NvSuccess;
}

void NvRmGpioReleasePinHandles(NvRmGpioHandle hGpio,
			       NvRmGpioPinHandle * hPin, NvU32 pinCount)
{
	NvU32 i;
	NvU32 port;
	NvU32 pin;
	NvU32 bank;
	int gpio_nr;
	NvU32 alphaPort;

	if (hPin == NULL)
		return;

	for (i = 0; i < pinCount; i++) {
		bank = GET_BANK(hPin[i]);
		port = GET_PORT(hPin[i]);
		pin = GET_PIN(hPin[i]);
		NvOsMutexLock(s_GpioMutex);
		if (port == NVRM_GPIO_CAMERA_PORT) {
			NvRmPrivGpioViReleasePinHandles(hGpio->hRm, pin);
		} else if ((port >= NVODM_GPIO_EXT_PORT_0) &&
			   (port <= NVODM_GPIO_EXT_PORT_F)) {
			/* Do nothing for now... */
		} else {
			gpio_nr = GPIO_PIN_ID(bank, port & 0x3, pin);
			if (gpio_nr >= ARCH_NR_GPIOS) {
				printk(KERN_ERR "Illegal pin handle at place "
						" %u of the list\n",i);
				NvOsMutexUnlock(s_GpioMutex);
				continue;
			}
			gpio_free(gpio_nr);
			alphaPort = (NvU32) GPIO_PORT_ID(bank, port);
			if (s_hGpio->pPinInfo[gpio_nr].used) {
				NV_DEBUG_PRINTF(("Warning: Releasing in-use "
					"GPIO pin handle GPIO_P%c.%02u "
					"(%c=%u)\n", 'A' + alphaPort, pin,
					'A' + alphaPort, alphaPort));
#if RELEASE_IS_INVALIDATE
				tegra_gpio_disable(gpio_nr);
				NvRmSetGpioTristate(hGpio->hRm,
					alphaPort, pin, NV_TRUE);
				s_hGpio->pPinInfo[gpio_nr].used = NV_FALSE;
#endif
			}
		}
		NvOsMutexUnlock(s_GpioMutex);
	}
	return;
}

void NvRmGpioReadPins(NvRmGpioHandle hGpio,
		      NvRmGpioPinHandle * hPin,
		      NvRmGpioPinState * pPinState, NvU32 pinCount)
{
	NvU32 bank;
	NvU32 port;
	NvU32 pin;
	NvU32 i;
	int gpio_nr;

	NV_ASSERT(hPin != NULL);
	NV_ASSERT(hGpio != NULL);

	for (i = 0; i < pinCount; i++) {
		port = GET_PORT(hPin[i]);
		pin = GET_PIN(hPin[i]);
		bank = GET_BANK(hPin[i]);

		if (port == NVRM_GPIO_CAMERA_PORT) {
			pPinState[i] = NvRmPrivGpioViReadPins(hGpio->hRm, pin);
		} else if ((port >= (NvU32) NVODM_GPIO_EXT_PORT_0) &&
			   (port <= (NvU32) NVODM_GPIO_EXT_PORT_F)) {
			pPinState[i] = NvOdmExternalGpioReadPins(port, pin);
		} else {
			gpio_nr = GPIO_PIN_ID(bank, port, pin);
			if (gpio_nr >= ARCH_NR_GPIOS) {
				printk(KERN_ERR "Illegal pin handle at place "
						" %u of the list\n",i);
				continue;
			}
			pPinState[i] = gpio_get_value(gpio_nr) & 0x1;
		}
	}
}

void NvRmGpioWritePins(NvRmGpioHandle hGpio,
		       NvRmGpioPinHandle * hPin,
		       NvRmGpioPinState * pPinState, NvU32 pinCount)
{
	NvU32 port;
	NvU32 pin;
	NvU32 bank;
	NvU32 i;
	int gpio_nr;

	NV_ASSERT(hPin != NULL);
	NV_ASSERT(hGpio != NULL);

	for (i = 0; i < pinCount; i++) {
		port = GET_PORT(hPin[i]);
		pin = GET_PIN(hPin[i]);
		bank = GET_BANK(hPin[i]);

		if (port == NVRM_GPIO_CAMERA_PORT) {
			NvRmPrivGpioViWritePins(hGpio->hRm, pin, pPinState[i]);
		} else if ((port >= (NvU32) NVODM_GPIO_EXT_PORT_0) &&
			   (port <= (NvU32) NVODM_GPIO_EXT_PORT_F)) {
			NvOdmExternalGpioWritePins(port, pin, pPinState[i]);
		} else {
			gpio_nr = GPIO_PIN_ID(bank, port, pin);
			if (gpio_nr >= ARCH_NR_GPIOS) {
				printk(KERN_ERR "Illegal pin handle at place "
						" %u of the list\n",i);
				continue;
			}
			gpio_set_value(gpio_nr, pPinState[i] & 0x1);
		}
	}

	return;
}

NvError NvRmGpioConfigPins(NvRmGpioHandle hGpio,
			   NvRmGpioPinHandle * hPin,
			   NvU32 pinCount, NvRmGpioPinMode Mode)
{
	NvError err = NvSuccess;
	NvU32 i;
	NvU32 bank;
	NvU32 port;
	NvU32 pin;
	NvU32 alphaPort;
	int ret_status;
	int gpio_nr;
	struct irq_chip *chip;
	int gpio_irq;

	NvOsMutexLock(s_GpioMutex);

	for (i = 0; i < pinCount; i++) {
		bank = GET_BANK(hPin[i]);
		port = GET_PORT(hPin[i]);
		pin = GET_PIN(hPin[i]);

		if (port == NVRM_GPIO_CAMERA_PORT) {
			/* If they are trying to do the wrong thing, assert.
			 * If they are trying to do the only allowed thing,
			 * quietly skip it, as nothing needs to be done. */
			if (Mode != NvOdmGpioPinMode_Output) {
				NV_ASSERT(!"Only output is supported for "
						"camera gpio.\n");
			}
			continue;
		}

		/* Absolute pin number to index into pPinInfo array and
		 * the alphabetic port names. */
		gpio_nr = GPIO_PIN_ID(bank, port, pin);
		gpio_irq = gpio_to_irq(gpio_nr);
		if (gpio_nr >= ARCH_NR_GPIOS) {
			printk(KERN_ERR "Illegal pin handle at place "
					" %u of the list\n",i);
			continue;
		}

		alphaPort = GPIO_PORT_ID(bank, port);

		s_hGpio->pPinInfo[gpio_nr].mode = Mode;
		s_hGpio->pPinInfo[gpio_nr].inst = bank;
		s_hGpio->pPinInfo[gpio_nr].port = port;
		s_hGpio->pPinInfo[gpio_nr].pin = pin;

		/* Don't try to colapse this swtich as the ordering of
		 * the register writes matter. */
		switch (Mode) {
		case NvRmGpioPinMode_Output:
			tegra_gpio_enable(gpio_nr);
			ret_status = gpio_direction_output(gpio_nr, 0);
			if (unlikely(ret_status != 0)) {
				NV_ASSERT(!"Not initialized");
				return NvError_NotInitialized;
			}
			break;

		case NvRmGpioPinMode_InputData:
			tegra_gpio_enable(gpio_nr);
			ret_status = gpio_direction_input(gpio_nr);
			if (unlikely(ret_status != 0)) {
				NV_ASSERT(!"Not initialized");
				return NvError_NotInitialized;
			}
			break;

		case NvRmGpioPinMode_InputInterruptLow:
			gpio_direction_input(gpio_nr);
			tegra_gpio_enable(gpio_nr);
			chip = get_irq_chip(gpio_irq);
			if ((chip) && (chip->set_type))
				chip->set_type(gpio_irq, IRQ_TYPE_LEVEL_LOW);
			break;

		case NvRmGpioPinMode_InputInterruptHigh:
			gpio_direction_input(gpio_nr);
			tegra_gpio_enable(gpio_nr);
			chip = get_irq_chip(gpio_irq);
			if ((chip) && (chip->set_type))
				chip->set_type(gpio_irq, IRQ_TYPE_LEVEL_HIGH);
			break;

		case NvRmGpioPinMode_InputInterruptAny:
			if (GPIO_ARCH_FEATURE & NVRM_GPIO_CAP_FEAT_EDGE_INTR) {
				gpio_direction_input(gpio_nr);
				tegra_gpio_enable(gpio_nr);
				chip = get_irq_chip(gpio_irq);
				if ((chip) && (chip->set_type))
					chip->set_type(gpio_irq,
						       IRQ_TYPE_EDGE_BOTH);
			} else {
				NV_ASSERT(!"Not supported");
			}
			break;

		case NvRmGpioPinMode_Function:
			tegra_gpio_disable(gpio_nr);
			break;

		case NvRmGpioPinMode_Inactive:
			chip = get_irq_chip(gpio_irq);
			if ((chip) && (chip->set_type))
				chip->mask(gpio_irq);
			tegra_gpio_disable(gpio_nr);
			break;

		case NvRmGpioPinMode_InputInterruptRisingEdge:
			if (GPIO_ARCH_FEATURE & NVRM_GPIO_CAP_FEAT_EDGE_INTR) {
				gpio_direction_input(gpio_nr);
				tegra_gpio_enable(gpio_nr);

				chip = get_irq_chip(gpio_irq);
				if ((chip) && (chip->set_type))
					chip->set_type(gpio_irq,
						       IRQ_TYPE_EDGE_RISING);
			} else {
				NV_ASSERT(!"Not supported");
			}
			break;

		case NvRmGpioPinMode_InputInterruptFallingEdge:
			if (GPIO_ARCH_FEATURE & NVRM_GPIO_CAP_FEAT_EDGE_INTR) {
				gpio_direction_input(gpio_nr);
				tegra_gpio_enable(gpio_nr);
				chip = get_irq_chip(gpio_irq);
				if ((chip) && (chip->set_type))
					chip->set_type(gpio_irq,
						       IRQ_TYPE_EDGE_FALLING);
			} else {
				NV_ASSERT(!"Not supported");
			}
			break;

		default:
			NV_ASSERT(!"Invalid gpio mode");
			break;
		}

		/*  Pad group global tristates are only modified when
		 *  the pin transitions from an inactive state to an
		 *  active one.  Active-to-active and inactive-to-inactive
		 * transitions are ignored */
		if ((!s_hGpio->pPinInfo[gpio_nr].used)
		    && (Mode != NvRmGpioPinMode_Inactive)) {
#if NV_ENABLE_GPIO_POWER_RAIL
			ret_status = tegra_gpio_io_power_config(gpio_nr, true);
			err = (NvError)ret_status;
#endif
			NvRmSetGpioTristate(hGpio->hRm,
					 alphaPort, pin, NV_FALSE);
		} else if ((s_hGpio->pPinInfo[gpio_nr].used)
			   && (Mode == NvRmGpioPinMode_Inactive)) {
#if NV_ENABLE_GPIO_POWER_RAIL
			ret_status = tegra_gpio_io_power_config(gpio_nr, false);
			err = (NvError)ret_status;
#endif
			NvRmSetGpioTristate(hGpio->hRm,
				 	alphaPort, pin, NV_TRUE);
		}
		if (Mode != NvRmGpioPinMode_Inactive)
			s_hGpio->pPinInfo[gpio_nr].used = NV_TRUE;
		else
			s_hGpio->pPinInfo[gpio_nr].used = NV_FALSE;
	}

	NvOsMutexUnlock(s_GpioMutex);
	return err;
}

NvError NvRmGpioGetIrqs(NvRmDeviceHandle hRmDevice,
			NvRmGpioPinHandle * hPin, NvU32 * Irq, NvU32 pinCount)
{
	NvU32 i;
	int irq_base;
	NvU32 bank;
	NvU32 port;
	NvU32 pin;
	int gpio_nr;

	irq_base = gpio_to_irq(0);
	for (i = 0; i < pinCount; i++) {
		bank = GET_BANK(hPin[i]);
		port = GET_PORT(hPin[i]);
		pin = GET_PIN(hPin[i]);
		gpio_nr = GPIO_PIN_ID(bank, port, pin);
		if (gpio_nr >= ARCH_NR_GPIOS) {
			printk(KERN_ERR "Illegal pin handle at place "
					" %u of the list\n",i);
			continue;
		}
		Irq[i] = irq_base + gpio_nr;
	}
	return NvSuccess;
}
