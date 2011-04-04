/*
 * tegra_exports.c
 *
 * Export Tegra-specific functions for use by kernel loadable modules.
 *
 * Copyright (c) 2009, NVIDIA Corporation.
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

#include <linux/kernel.h>
#include <linux/module.h>
#include "nvrm_analog.h"
#include "nvrm_diag.h"
#include "nvrm_dma.h"
#include "nvrm_gpio.h"
#include "nvrm_hardware_access.h"
#include "nvrm_i2c.h"
#include "nvrm_init.h"
#include "nvrm_interrupt.h"
#include "nvrm_keylist.h"
#include "nvrm_memctrl.h"
#include "nvrm_memmgr.h"
#include "nvrm_module.h"
#include "nvrm_owr.h"
#include "nvrm_pinmux.h"
#include "nvrm_pmu.h"
#include "nvrm_power.h"
#include "nvrm_pwm.h"
#include "nvrm_rmctrace.h"
#include "nvrm_spi.h"
#include "nvodm_services.h"
#include "nvodm_query_discovery.h"
#include "nvos.h"

EXPORT_SYMBOL(NvOdmGpioOpen);
EXPORT_SYMBOL(NvOdmGpioClose);
EXPORT_SYMBOL(NvOdmGpioConfig);
EXPORT_SYMBOL(NvOdmGpioGetState);
EXPORT_SYMBOL(NvOdmGpioSetState);
EXPORT_SYMBOL(NvOdmGpioAcquirePinHandle);
EXPORT_SYMBOL(NvOdmGpioReleasePinHandle);
EXPORT_SYMBOL(NvOdmGpioInterruptRegister);
EXPORT_SYMBOL(NvOdmGpioInterruptUnregister);
EXPORT_SYMBOL(NvOdmGpioInterruptDone);
EXPORT_SYMBOL(NvOdmOsSemaphoreCreate);
EXPORT_SYMBOL(NvOdmOsSemaphoreDestroy);
EXPORT_SYMBOL(NvOdmOsSemaphoreSignal);
EXPORT_SYMBOL(NvOdmOsSemaphoreWait);
EXPORT_SYMBOL(NvOdmOsSleepMS);
EXPORT_SYMBOL(NvOdmPeripheralGetGuid);
EXPORT_SYMBOL(NvRmOpen);
EXPORT_SYMBOL(NvRmClose);
EXPORT_SYMBOL(NvRegrb);
EXPORT_SYMBOL(NvRegrm);
EXPORT_SYMBOL(NvRegwb);
EXPORT_SYMBOL(NvRegwm);
EXPORT_SYMBOL(NvRmGetIrqForLogicalInterrupt);
EXPORT_SYMBOL(NvRmGetRmcFile);
EXPORT_SYMBOL(NvRmInterruptDone);
EXPORT_SYMBOL(NvRmInterruptRegister);
EXPORT_SYMBOL(NvRmInterruptUnregister);
EXPORT_SYMBOL(NvRmMemAlloc);
EXPORT_SYMBOL(NvRmMemGetAddress);
EXPORT_SYMBOL(NvRmMemGetId);
EXPORT_SYMBOL(NvRmMemHandleCreate);
EXPORT_SYMBOL(NvRmMemHandleFree);
EXPORT_SYMBOL(NvRmMemHandleFromId);
EXPORT_SYMBOL(NvRmMemMap);
EXPORT_SYMBOL(NvRmMemPin);
EXPORT_SYMBOL(NvRmMemPinMult);
EXPORT_SYMBOL(NvRmMemRd32);
EXPORT_SYMBOL(NvRmMemUnmap);
EXPORT_SYMBOL(NvRmMemUnpin);
EXPORT_SYMBOL(NvRmMemUnpinMult);
EXPORT_SYMBOL(NvRmMemWr32);
EXPORT_SYMBOL(NvRmMemWrite);
EXPORT_SYMBOL(NvRmModuleGetBaseAddress);
EXPORT_SYMBOL(NvRmModuleGetCapabilities);
EXPORT_SYMBOL(NvRmModuleReset);
EXPORT_SYMBOL(NvRmPhysicalMemMap);
EXPORT_SYMBOL(NvRmPhysicalMemUnmap);
EXPORT_SYMBOL(NvRmPowerBusyHint);
EXPORT_SYMBOL(NvRmPowerGetEvent);
EXPORT_SYMBOL(NvRmPowerModuleClockConfig);
EXPORT_SYMBOL(NvRmPowerModuleClockControl);
EXPORT_SYMBOL(NvRmPowerRegister);
EXPORT_SYMBOL(NvRmPowerVoltageControl);
EXPORT_SYMBOL(NvRmRmcTrace);
EXPORT_SYMBOL(NvRmSpiOpen);
EXPORT_SYMBOL(NvRmSpiClose);
EXPORT_SYMBOL(NvRmSpiStartTransaction);
EXPORT_SYMBOL(NvRmSpiGetTransactionData);

EXPORT_SYMBOL(NvOdmServicesPmuOpen);
EXPORT_SYMBOL(NvOdmServicesPmuGetCapabilities);
EXPORT_SYMBOL(NvOdmI2cClose);
EXPORT_SYMBOL(NvOdmOsFree);
EXPORT_SYMBOL(NvOdmOsMemset);
EXPORT_SYMBOL(NvOdmServicesPmuSetVoltage);
EXPORT_SYMBOL(NvOdmI2cTransaction);
EXPORT_SYMBOL(NvOdmOsAlloc);
EXPORT_SYMBOL(NvOdmOsWaitUS);
EXPORT_SYMBOL(NvOdmI2cPinMuxOpen);
EXPORT_SYMBOL(NvOdmOsMemcpy);
EXPORT_SYMBOL(NvOdmServicesPmuClose);

EXPORT_SYMBOL(NvOdmOsSemaphoreWaitTimeout);
