/*
 * Copyright (c) 2010 NVIDIA Corporation.
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

#ifndef INCLUDED_NVDDK_FUSE_H
#define INCLUDED_NVDDK_FUSE_H

#include "nvrm_module.h"
/**
 * @defgroup nvddk_fuse_group Fuse Programming APIs
 *
 * Enables fuse programming.
 *
 * @par Library Usage
 *
 * - Calling the NvDdkDisableFuseProgram() API before exiting the boot loader
 *    disables further fuse programming until the next system reset. (See the
 *    example code.)
 * - It is advised that you disable the JTAG while using this code in
 *    the final version.
 * - Enabling clock and voltage to program the fuse is out of scope of this
 *    library. You must ensure that these are enabled prior to using the library
 *    functions.
 * - The library works on physical mapped memory or on virtual mapped with
 *    one-to-one mapping for the fuse registers.
 *
 * @par Example Code
 *
 * The following example shows one way to use Fuse Programming APIs.
 * @note You must ensure fuse clock and fuse voltage is enabled prior
 * to programming fuses.
 *
 * @code
 * //
 * // Fuse Programming Example
 * //
 * #include "nvddk_fuse.h"
 * #include "nvtest.h"
 * #include "nvrm_init.h"
 * #include "nvrm_pmu.h"
 * #include "nvodm_query_discovery.h"
 * #include "nvodm_services.h"
 *
 * #define DISABLE_PROGRMING_TEST 0
 *
 * static NvError NvddkFuseProgramTest(NvTestApplicationHandle h )
 * {
 *     NvError err = NvSuccess;
 *     err = NvDdkFuseProgram();
 *     if(err != NvError_Success)
 *     {
 *         NvOsDebugPrintf("NvDdkFuseProgram failed \n");
 *         return err;
 *     }
 *     // Check fuse sense.
 *     NvDdkFuseSense();
 *     // Verify the fuses and return the result.
 *     err = NvDdkFuseVerify();
 *     if(err != NvError_Success)
 *         NvOsDebugPrintf("NvDdkFuseVerify failed \n");
 *     return err;
 * }
 *
 * static NvError PrepareFuseData(void)
 * {
 *     // Initialize argument sizes to zero to perform initial queries.
 *     NvU32 BootDevSel_Size = 0;
 *     NvU32 BootDevConfig_Size = 0;
 *     NvU32 ResevOdm_size = 0;
 *     NvU32 size = 0;
 *
 *     // Specify values to be programmed.
 *     NvU16 BootDevConfig_Data = 0x9; // Set device config value to 0x9.
 *     NvU8 BootDevSel_Data = 0x1;     // Set boot select to 0x1 for NAND.
 *     NvU8 ResevOdm_Data[32] =  {0xEF,0xCD,0xAB,0x89,
 *                                                     0x78,0x56,0x34,0x12,
 *                                                     0xa,0xb,0xc,0xd,
 *                                                     0xAA,0xBB,0xCC,0xDD,
 *                                                     0,0,0,0,
 *                                                     0,0,0,0,
 *                                                     0x78,0x56,0x34,0x12,
 *                                                     0x78,0x56,0x34,0x12};
 *
 *     NvU8 skpDevSelStrap_data = 1;
 *     NvError e;
 *
 *     // Query the sizes of the fuse values.
 *     e = NvDdkFuseGet(NvDdkFuseDataType_SecBootDeviceSelect,
 *                                     &BootDevSel_Data, &BootDevSel_Size);
 *     if (e != NvError_Success) return e;
 *
 *     e = NvDdkFuseGet(NvDdkFuseDataType_SecBootDeviceConfig,
 *                                     &BootDevConfig_Data, &BootDevConfig_Size);
 *     if (e != NvError_Success) return e;
 *
 * #ifdef DISABLE_PROGRMING_TEST
 *     NvDdkDisableFuseProgram();
 * #endif
 *
 *     e = NvDdkFuseGet(NvDdkFuseDataType_ReservedOdm,
 *                                     &ResevOdm_Data, &ResevOdm_size);
 *     if (e != NvError_Success) return e;
 *
 *     e = NvDdkFuseGet(NvDdkFuseDataType_SkipDevSelStraps,
 *                                     &skpDevSelStrap_data, &size);
 *     if (e != NvError_Success) return e;
 *
 *
 *     // Set the fuse values.
 *     e = NvDdkFuseSet(NvDdkFuseDataType_SecBootDeviceSelect,
 *                                     &BootDevSel_Data, &BootDevSel_Size);
 *     if (e != NvError_Success) return e;
 *
 *     e = NvDdkFuseSet(NvDdkFuseDataType_SecBootDeviceConfig,
 *                                     &BootDevConfig_Data, &BootDevConfig_Size);
 *     if (e != NvError_Success) return e;
 *
 *     e = NvDdkFuseSet(NvDdkFuseDataType_SkipDevSelStraps,
 *                                     &skpDevSelStrap_data, &size);
 *     if (e != NvError_Success) return e;
 *
 *
 *     e = NvDdkFuseSet(NvDdkFuseDataType_ReservedOdm,
 *                                     &ResevOdm_Data, &ResevOdm_size);
 *     if (e != NvError_Success) return e;
 *
 *     return e;
 * }
 *
 *
 * NvError NvTestMain(int argc, char *argv[])
 * {
 *     NvTestApplication h;
 *     NvError err;
 *
 *     NVTEST_INIT( &h );
 *     // Enable fuse clock and fuse voltage prior to programming fuses.
 *     err = PrepareFuseData();
 *     if( err != NvSuccess)
 *         return err;
 *
 *     NvOdmOsWaitUS(10000);
 *
 *     NVTEST_RUN(&h, NvddkFuseProgramTest);
 *
 *     NVTEST_RESULT( &h );
 * }
 * @endcode
 *
 * @ingroup nvddk_modules
 * @{
 */

#if defined(__cplusplus)
extern "C"
{
#endif


#include "nvcommon.h"
#include "nverror.h"


/* ----------- Program/Verify API's -------------- */

/**
 * Defines types of fuse data to set or query.
 */
typedef enum
{
    /// Specifies a default (unset) value.
    NvDdkFuseDataType_None = 0,

    /// Specifies a device key (DK).
    NvDdkFuseDataType_DeviceKey,

    /**
     * Specifies a JTAG disable flag:
     * - NV_TRUE specifies to permanently disable JTAG-based debugging.
     * - NV_FALSE indicates that JTAG-based debugging is not permanently
     *   disabled.
     */
    NvDdkFuseDataType_JtagDisable,

    /**
     * Specifies a key programmed flag (read-only):
     * - NV_TRUE indicates that either the SBK or the DK value is nonzero.
     * - NV_FALSE indicates that both the SBK and DK values are zero (0).
     *
     * @note Once the ODM production fuse is set to NV_TRUE, applications
     * can no longer read back the SBK or DK value; however, by using
     * this \c KeyProgrammed query it is still possible to determine
     * whether or not the SBK or DK has been programmed to a nonzero value.
     */
    NvDdkFuseDataType_KeyProgrammed,

    /**
     * Specifies an ODM production flag:
     * - NV_TRUE specifies the chip is in ODM production mode.
     * - NV_FALSE indicates that the chip is not in ODM production mode.
     */
    NvDdkFuseDataType_OdmProduction,

    /**
     * Specifies a secondary boot device configuration.
     * This value is chip-dependent.
     * For Tegra APX 2600, use the NVBOOT_FUSE_*_CONFIG_*
     * defines from:
     * <pre>
     *  /src/drivers/hwinc/ap15/nvboot_fuse.h
     * </pre>
     */
    NvDdkFuseDataType_SecBootDeviceConfig,

    /**
     * Specifies a secondary boot device selection.
     * This value is chip-independent and is described in \c nvddk_bootdevices.h.
     * The chip-dependent version of this data is
     * ::NvDdkFuseDataType_SecBootDeviceSelectRaw.
     * For Tegra APX 2600, the values for \c SecBootDeviceSelect and
     * \c SecBootDeviceSelectRaw are identical.
     */
    NvDdkFuseDataType_SecBootDeviceSelect,

    /** Specifies a secure boot key (SBK). */
    NvDdkFuseDataType_SecureBootKey,

    /**
     * Specifies a stock-keeping unit (read-only).
     * This value is chip-dependent.
     * See chip-specific documentation for legal values.
     */
    NvDdkFuseDataType_Sku,

    /**
     * Specifies spare fuse bits (read-only).
     * Reserved for future use by NVIDIA.
     */
    NvDdkFuseDataType_SpareBits,

    /**
     * Specifies software reserved fuse bits (read-only).
     * Reserved for future use by NVIDIA.
     */
    NvDdkFuseDataType_SwReserved,

    /**
     * Specifies skip device select straps (applies to Tegra 200 series only):
     * - NV_TRUE specifies to ignore the device selection straps setting
     *   and that the boot device is specified via the
     *   \c SecBootDeviceSelect and \c SecBootDeviceConfig fuse settings.
     * - NV_FALSE indicates that the boot device is specified via the device
     *   selection straps setting.
     */
    NvDdkFuseDataType_SkipDevSelStraps,

    /**
     * Specifies a secondary boot device selection.
     * This value is chip-dependent.
     * The chip-independent version of this data is
     * ::NvDdkFuseDataType_SecBootDeviceSelect.
     * For Tegra APX 2600, use the \c NvBootFuseBootDevice enum
     * values found at:
     * <pre>
     *  /src/drivers/hwinc/ap15/nvboot_fuse.h
     * </pre>
     */
    NvDdkFuseDataType_SecBootDeviceSelectRaw,

    /**
     * Specifies raw field for reserved the ODM.
     * This value is ODM-specific. Reserved for customers.
     */
    NvDdkFuseDataType_ReservedOdm,

    /** The following must be last. */
    NvDdkFuseDataType_Num,
    /** Ignore -- Forces compilers to make 32-bit enums. */
    NvDdkFuseDataType_Force32 = 0x7FFFFFFF
} NvDdkFuseDataType;

// Gets the base address for Fuse anc CAR modules.
// Call this function before accessing any other API of Fuse.
NvError NvDdkFuseOpen(NvRmDeviceHandle hRmDevice);

// to free up all resources
void NvDdkFuseClose(void);
/**
 * Gets a value from the fuse registers.
 *
 *@pre NvDdkFuseOpen should have been called before using this API
 * @pre Do not call this function while the programming power is applied!
 *
 * After programming fuses and removing the programming power,
 * NvDdkFuseSense() must be called to read the new values.
 *
 * By passing a size of 0, the caller is requesting to be told the
 * expected size.
 *
 * Treatment of fuse data depends upon its size:
 * - if \a *pSize == 1, treat \a *pData as an NvU8
 * - if \a *pSize == 2, treat \a *pData as an NvU16
 * - if \a *pSize == 4, treat \a *pData as an NvU32
 * - else, treat \a *pData as an array of NvU8 values (i.e., NvU8[]).
 */
NvError NvDdkFuseGet(NvDdkFuseDataType Type, void *pData, NvU32 *pSize);

/**
 * Schedules fuses to be programmed to the specified values when the next
 * NvDdkFuseProgram() operation is performed.
 *
 *@pre NvDdkFuseOpen should have been called before using this API
 *
 * @note Attempting to program the ODM production fuse at the same
 * time as the SBK or DK causes an error, because it is not
 * possible to verify that the SBK or DK were programmed correctly.
 * After triggering this error, all further attempts to set fuse
 * values will fail, as will \c NvDdkFuseProgram, until NvDdkFuseClear()
 * has been called.
 *
 * By passing a size of 0, the caller is requesting to be told the
 * expected size.
 *
 * Treatment of fuse data depends upon its size:
 * - if \a *pSize == 1, treat \a *pData as an NvU8
 * - if \a *pSize == 2, treat \a *pData as an NvU16
 * - if \a *pSize == 4, treat \a *pData as an NvU32
 * - else, treat \a *pData as an array of NvU8 values (i.e., NvU8[]).
 *
 * @retval NvError_BadValue If other than "reserved ODM fuse" is set in ODM
 *             production  mode.
 * @retval NvError_AccessDenied If programming to fuse registers is disabled.
 */
NvError NvDdkFuseSet(NvDdkFuseDataType Type, void *pData, NvU32 *pSize);

/**
 * Reads the current fuse data into the fuse registers.
 *
 *@pre NvDdkFuseOpen should have been called before using this API
 *
 * \c NvDdkFuseSense must be called at least once, either:
 * - After programming fuses and removing the programming power,
 * - Prior to calling NvDdkFuseVerify(), or
 * - Prior to calling NvDdkFuseGet().
 */
void NvDdkFuseSense(void);

/**
 * Programs all fuses based on cache data changed via the NvDdkFuseSet() API.
 *
 *@pre NvDdkFuseOpen should have been called before using this API
 *
 * @pre Prior to invoking this routine, the caller is responsible for supplying
 * valid fuse programming voltage.
 *
 * @retval NvError_AccessDenied If programming to fuse registers is disabled.
 * @return An error if an invalid combination of fuse values was provided.
 */
NvError NvDdkFuseProgram(void);

/**
 * Verify all fuses scheduled via the NvDdkFuseSet() API.
 *
 *@pre NvDdkFuseOpen should have been called before using this API
 *
 * @pre Prior to invoking this routine, the caller is responsible for ensuring
 * that fuse programming voltage is removed and subsequently calling
 * NvDdkFuseSense().
 */
NvError NvDdkFuseVerify(void);

/**
 * Clears the cache of fuse data, once NvDdkFuseProgram() and NvDdkFuseVerify()
 * API are called to clear all buffers.
 *
 *@pre NvDdkFuseOpen should have been called before using this API
 */
void NvDdkFuseClear(void);


/**
 * Disables further fuse programming until the next system reset.
 *
 *@pre NvDdkFuseOpen should have been called before using this API
 */
void NvDdkDisableFuseProgram(void);


#if defined(__cplusplus)
}
#endif

/** @} */
#endif // INCLUDED_NVDDK_FUSE_H

