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
 * @file
 * @brief <b>NVIDIA Driver Development Kit:
 *           NvDDK USB PHY functions</b>
 *
 * @b Description: Defines USB PHY private functions
 *
 */

#ifndef INCLUDED_NVDDK_USBPHY_PRIV_H
#define INCLUDED_NVDDK_USBPHY_PRIV_H

#include "nvddk_usbphy.h"
#include "nvodm_query.h"
#include "nvodm_query_discovery.h"
#include "nvodm_usbulpi.h"
#include "nvrm_power.h"
#include "nvassert.h"
#include "nvrm_memmgr.h"
#include "nvodm_query_gpio.h"
#include "nvrm_gpio.h"

#if defined(__cplusplus)
extern "C"
{
#endif

/**
 * Minimum system frequency required for USB to work in High speed mode
 */
enum {USB_HW_MIN_SYSTEM_FREQ_KH = 100000};

/**
 * Minimum cpu frequency required for USB for optimal performance
 */
enum {USB_HW_MIN_CPU_FREQ_KH = 300000};

/**
 * Wait time(1 second) for controller H/W status to change before giving up.
 */
enum {USB_PHY_HW_TIMEOUT_US = 1000000};

/**
 *  Maximum elements in the controller context
 */
enum {USB_PHY_MAX_CONTEXT_REGS = 10};

/**
 * Defines possible USB Port Speed types.
 */
typedef enum
{
    /// Defines the port full speed.
    NvDdkUsbPhyPortSpeedType_Full = 0,

    /// Defines the port low speed.
    NvDdkUsbPhyPortSpeedType_Low,

    /// Defines the port high speed.
    NvDdkUsbPhyPortSpeedType_High,

    /// Ignore -- Forces compilers to make 32-bit enums.
    NvDdkUsbPhyPortSpeedType_Force32 = 0x7FFFFFF
} NvDdkUsbPhyPortSpeedType;


/**
 * USB Phy capabilities structure
 */
typedef struct NvDdkUsbPhyCapabilitiesRec
{
    /// Inidcates USB phy share the common clock and reset.
    NvBool PhyRegInController;
    /// Inidcates USB phy share the common clock and reset.
    NvBool CommonClockAndReset;
} NvDdkUsbPhyCapabilities;

/**
 * USB UTMIP Pad config control structure
 */
typedef struct NvDdkUsbPhyUtmiPadConfigRec
{
    // Usb controller virtual address
    volatile NvU32 *pVirAdr;
    // Usb controller bank size
    NvU32 BankSize;
    // Utmi Pad On reference count
    NvU32 PadOnRefCount;
} NvDdkUsbPhyUtmiPadConfig;

/**
 * USB Controller Context
 */
typedef struct NvDdkUsbPhyControllerContextRec
{
    /// Inidcates USB phy controller context is valid or not
    NvBool IsValid;
    /// USB Regsisters
    NvU32 UsbRegs[USB_PHY_MAX_CONTEXT_REGS];
    ///  Vlaid Register count
    NvU16 UsbRegCount;
    /// USB Port speed
    NvDdkUsbPhyPortSpeedType UsbPortSpeed;
} NvDdkUsbPhyControllerContext;

/**
 * Usb Phy record structure.
 */
typedef struct NvDdkUsbPhyRec
{
    // RM device handle
    NvRmDeviceHandle hRmDevice;
    // Usb controller virtual address
    volatile NvU32 *UsbVirAdr;
    // Usb controller bank size
    NvU32 UsbBankSize;
    // Misc virtual address
    volatile NvU32 *MiscVirAdr;
    // Misc bank size
    NvU32 MiscBankSize;
    // Usb phy open reference count
    NvU32  RefCount;
    // instance number
    NvU32 Instance;
    // capabilities structure
    NvDdkUsbPhyCapabilities Caps;
    // Usb odm property
    const NvOdmUsbProperty *pProperty;
    // Power Manager semaphore. 
    NvOsSemaphoreHandle hPwrEventSem;
    // Id returned from driver's registration with Power Manager
    NvU32 RmPowerClientId;
    // Ulpi Handle
    NvOdmUsbUlpiHandle hOdmUlpi;
    // guid
    NvU64 Guid;
    // peripheral connectivity
    NvOdmPeripheralConnectivity const *pConnectivity;
    // Indicates Phy is Powered up
    NvBool IsPhyPoweredUp;
    // Utmpi Pad Config control structure
    NvDdkUsbPhyUtmiPadConfig *pUtmiPadConfig;
    // Usb Controller context
    NvDdkUsbPhyControllerContext Context;
    // Contains the mutex for providing the thread safety
    NvOsMutexHandle ThreadSafetyMutex;
    // Indicator for turning off the USB power rail
    NvBool TurnOffPowerRail;
    // Indicates phy powered up for the host mode
    NvBool IsHostMode;
    // Handle to the GPIO
    NvOdmServicesGpioHandle hGpio;
    // Handle to the Pin
    NvOdmGpioPinHandle hPin;
    // Set of function pointers to access the usb phy hardware interface.
    // Pointer to the h/w specific PowerUp function.
    NvError (*PowerUp)(NvDdkUsbPhyHandle hUsbPhy);
    // Pointer to the h/w specific PowerDown function.
    NvError (*PowerDown)(NvDdkUsbPhyHandle hUsbPhy);
    // Pointer to the h/w specific WaitForStableClock function.
    NvError (*WaitForStableClock)(NvDdkUsbPhyHandle hUsbPhy);
    // Pointer to the h/w specific usb prefetcher function
    void (*MemoryPrefetch)(NvDdkUsbPhyHandle hUsbPhy, NvBool Enable);
    // Pointer to the h/w specific CloseHwInterface function.
    void (*CloseHwInterface)(NvDdkUsbPhyHandle hUsbPhy);
    // Pointer to save context function
    void (* SaveContext)(NvDdkUsbPhyHandle hUsbPhy);
    // Pointer to restore context function
    void (* RestoreContext)(NvDdkUsbPhyHandle hUsbPhy);
    // Pointer to the h/w specific Ioctl function.
    NvError (*Ioctl)(NvDdkUsbPhyHandle hUsbPhy,
        NvDdkUsbPhyIoctlType IoctlType,
        const void *pInputArgs,
        void *pOutputArgs);
} NvDdkUsbPhy;

/**
 * Opens the AP20 specifi H/W Usb Phy interface.
 *
 * @param hUsbPhy handle to the USB phy.
 *
 * @retval None
 */
void Ap20UsbPhyOpenHwInterface(NvDdkUsbPhy *pUsbPhy);

#if defined(__cplusplus)
}
#endif

/** @}*/
#endif // INCLUDED_NVDDK_USBPHY_H

