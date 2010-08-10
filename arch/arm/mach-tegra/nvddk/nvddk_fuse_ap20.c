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

#include "fuse_bitmap.h"
#include "ap20/arclk_rst.h"
#include "ap20/arfuse.h"
#include "nvddk_bootdevices.h"
#include "nvddk_operatingmodes.h"
#include "nvddk_fuse.h"
#include "nvrm_hardware_access.h"
#include "nvassert.h"
#include "nvrm_drf.h"
#include "nvos.h"
#include "nvodm_query_discovery.h"
#include "nvodm_services.h"
#include "nvrm_pmu.h"

// Set below macro to 0, to disable voltage required for Fuse programming.
#define ENABLE_FUSE_VOLTAGE 1
// By setting below macro to 1, a dummy interface of fuse is enabled for
// verifying sysfs interface.
#define FUSE_DUMMY_CODE  0
// Set below macro to 1 to get prints during fuse read/ program operations.
#define ENABLE_DEBUG_PRINTS 0

#define FUSE_BOOT_DEVICE_INFO_0_BOOT_DEVICE_CONFIG_RANGE 13:0
#define FUSE_RESERVED_SW_0_BOOT_DEVICE_SELECT_RANGE       2:0
#define FUSE_RESERVED_SW_0_SKIP_DEV_SEL_STRAPS_RANGE      3:3
#define FUSE_RESERVED_SW_0_SW_RESERVED_RANGE              7:4

#define FLAGS_PROGRAM_SECURE_BOOT_KEY     0x001
#define FLAGS_PROGRAM_DEVICE_KEY          0x002
#define FLAGS_PROGRAM_JTAG_DISABLE        0x004
#define FLAGS_PROGRAM_BOOT_DEV_SEL        0x008
#define FLAGS_PROGRAM_BOOT_DEV_CONFIG     0x010
#define FLAGS_PROGRAM_SW_RESERVED         0x020
#define FLAGS_PROGRAM_ODM_PRODUCTION      0x040
#define FLAGS_PROGRAM_SPARE_BITS          0x080
#define FLAGS_PROGRAM_SKIP_DEV_SEL_STRAPS 0x100
#define FLAGS_PROGRAM_RESERVED_ODM 0x200

#define FLAGS_PROGRAM_ILLEGAL         0x80000000

#define NVDDK_DEVICE_KEY_BYTES       (4)
#define NVDDK_SECURE_BOOT_KEY_BYTES  (16)
#define NVDDK_RESERVED_ODM_BYTES  (32)

#define NVDDK_TPROGRAM_VALUE_CORRECTED 0

// Number of words of fuses as defined by the hardware.
#define FUSE_WORDS 64

/*
 * Representation of the fuse data.
 *
 */
typedef struct NvDdkFuseDataRec
{
    // Specifies the Secure Boot Key (SBK).
    NvU8   SecureBootKey[NVDDK_SECURE_BOOT_KEY_BYTES];

    // Specifies the Device Key (DK).
    NvU8   DeviceKey[NVDDK_DEVICE_KEY_BYTES];

    // Specifies the JTAG Disable fuse value.
    NvBool JtagDisable;

    // Specifies the device selection value in terms of the API.
    NvU32  SecBootDeviceSelect;

    // Specifies the device selection value in the actual fuses.
    NvU32  SecBootDeviceSelectRaw;

    // Specifies the device configuration value (right aligned).
    NvU16  SecBootDeviceConfig;

    // Specifies the SwReserved value.
    NvU32  SwReserved;

    // Specifies the ODM Production fuse value.
    NvBool OdmProduction;

    // Specifies the SkipDevSelStraps value.
    NvU32  SkipDevSelStraps;

    // Specifies the Reserved Odm value
    NvU8    ReservedOdm[NVDDK_RESERVED_ODM_BYTES];

    // Flags that indicate what to program.
    NvU32  ProgramFlags;

    // Flag indicates whether reservedodm fuses are setting or not
    NvBool ReservedOdmFlag;
} NvDdkFuseData;

static NvDdkFuseData s_FuseData = {{0}};

typedef enum
{
    FuseBootDev_Sdmmc,
    FuseBootDev_SnorFlash,
    FuseBootDev_SpiFlash,
    FuseBootDev_NandFlash,
    FuseBootDev_NandFlash_x8  = FuseBootDev_NandFlash,
    FuseBootDev_NandFlash_x16 = FuseBootDev_NandFlash,
    FuseBootDev_MobileLbaNand,
    FuseBootDev_MuxOneNand,
    FuseBootDev_Max, /* Must appear after the last legal item */
    FuseBootDev_Force32 = 0x7fffffff
} FuseBootDev;

// Holds various params that are needed by Fuse module.
typedef struct NvDdkFuseRec
{
    // Fuse registers physical base address
    NvRmPhysAddr pBaseAddress;
    // Holds the virtual address for accessing registers.
    NvU32 *pVirtualAddress;
    // Holds the register map size.
    NvU32 BankSize;

    // Fuse registers physical base address
    NvRmPhysAddr pCarBaseAddress;
    // Holds the virtual address for accessing registers.
    NvU32 *pCarVirtualAddress;
    // Holds the register map size.
    NvU32 CarBankSize;

    // Mutex for fuse
    NvOsMutexHandle Mutex;
}NvDdkFuse;

static NvDdkFuse* s_pFuseRec = NULL;

// Size of the data items, in bytes
static NvU32 s_DataSize[] =
{
    0, // FuseDataType_None
    sizeof(NvU64),  // FuseDataType_DeviceKey
    sizeof(NvBool), // FuseDataType_JtagDisable
    sizeof(NvBool), // FuseDataType_KeyProgrammed
    sizeof(NvBool), // FuseDataType_OdmProductionEnable
    sizeof(NvU16),  // FuseDataType_SecBootDeviceConfig
    sizeof(NvU32),  // FuseDataType_SecBootDeviceSelect
    NVDDK_SECURE_BOOT_KEY_BYTES, // FuseDataType_SecureBootKey
    sizeof(NvU32),  // FuseDataType_Sku
    sizeof(NvU32),  // FuseDataType_SpareBits
    sizeof(NvU32),  // FuseDataType_SwReserved
    sizeof(NvBool), // FuseDataType_SkipDevSelStraps
    sizeof(NvU32),  // FuseDataType_SecBootDeviceSelectRaw
    NVDDK_RESERVED_ODM_BYTES  // NvDdkFuseDataType_ReservedOdm fuse
};


// Map NvDdkSecBootDeviceType enum onto AP20 Boot Device Selection values.
static NvU32 NvDdkToFuseBootDeviceSel[] =
{
    FuseBootDev_Sdmmc,         // None
    FuseBootDev_NandFlash,     // NvDdkSecBootDeviceType_Nand
    FuseBootDev_SnorFlash,     // NvDdkSecBootDeviceType_Nor
    FuseBootDev_SpiFlash,      // NvDdkSecBootDeviceType_Spi_Flash
    FuseBootDev_Sdmmc,         // NvDdkSecBootDeviceType_eMMC
    FuseBootDev_NandFlash,     // NvDdkSecBootDeviceType_Nand_x16
    FuseBootDev_MobileLbaNand, // NvDdkSecBootDeviceType_MobileLbaNand
    FuseBootDev_Sdmmc,         // NvDdkSecBootDeviceType_eMMC
    FuseBootDev_MuxOneNand,    // NvDdkSecBootDeviceType_MuxOneNand
};

// Map AP20 fuse values onto NvDdkSecBootDeviceType enum values.
static NvU32 FuseToNvDdkBootDeviceSel[] =
{
    NvDdkSecBootDeviceType_Sdmmc,         // FuseBootDev_Sdmmc
    NvDdkSecBootDeviceType_Nor,           // FuseBootDev_SnorFlash
    NvDdkSecBootDeviceType_Spi,           // FuseBootDev_SpiFlash
    NvDdkSecBootDeviceType_Nand,          // FuseBootDev_NandFlash
    NvDdkSecBootDeviceType_MobileLbaNand, // FuseBootDev_MobileLbaNand
    NvDdkSecBootDeviceType_MuxOneNand,    // FuseBootDev_MuxOneNand
};

// Storage for the array of fuse words & mask words.
static NvU32 s_FuseArray   [FUSE_WORDS] = { 0 };
static NvU32 s_MaskArray   [FUSE_WORDS] = { 0 };
static NvU32 s_TempFuseData[FUSE_WORDS];


#define FUSE_NV_READ32(offset) \
    NV_READ32(s_pFuseRec->pVirtualAddress + (offset >> 2))

#define FUSE_NV_WRITE32(offset, val) \
    NV_WRITE32((s_pFuseRec->pVirtualAddress + (offset >> 2)), val)

#define CLOCK_NV_READ32(offset) \
    NV_READ32(s_pFuseRec->pCarVirtualAddress + (offset >> 2))

#define CLOCK_NV_WRITE32(offset, val) \
    NV_WRITE32((s_pFuseRec->pCarVirtualAddress + (offset >> 2)), val)

// Macro to get the difference between two numbers T1 & T2 and T1 > T2.
#define DIFF(T1, T2) \
 (((T1) > (T2)) ? ((T1) - (T2)) : ((T2) - (T1)))
// For rollover proof,
// (((T1) > (T2)) ? ((T1) - (T2)) : ((T1) + ((NvU64)0xFFFFFFFFFFFFFFFF - (T2))))

static void NvFuseUtilWaitUS(NvU32 usec)
{
    NvU64 t0;
    NvU64 t1;

    t0 = NvOsGetTimeUS();
    t1 = t0;
    // Use the difference for the comparison to be wraparound safe
    while (DIFF(t1, t0) < usec)
    {
        t1 = NvOsGetTimeUS();
    }
}

static void fusememset(void* Source, NvU8 val, NvU32 size)
{
    NvOsMemset(Source, val, size);
}

static void fusememcpy(void* Destination, void* Source, NvU32 size)
{
    NvOsMemcpy(Destination, Source, size);
}
/**
 * Reports whether any of the SBK fuses are set (burned)
 *
 * @param none
 *
 * @return NV_TRUE if the SBK is non-zero
 * @return NV_FALSE otherwise
 */
static NvBool NvDdkFuseIsSbkSet(void)
{
    NvU32 AllKeysOred;
    AllKeysOred  = FUSE_NV_READ32(FUSE_PRIVATE_KEY0_NONZERO_0);
    AllKeysOred |= FUSE_NV_READ32(FUSE_PRIVATE_KEY1_NONZERO_0);
    AllKeysOred |= FUSE_NV_READ32(FUSE_PRIVATE_KEY2_NONZERO_0);
    AllKeysOred |= FUSE_NV_READ32(FUSE_PRIVATE_KEY3_NONZERO_0);
    if (AllKeysOred)
        return NV_TRUE;
    else
        return NV_FALSE;
}

/**
 * Reports whether any of the SBK or DK fuses are set (burned)
 *
 * @param none
 *
 * @return NV_TRUE if the SBK or the DK is non-zero
 * @return NV_FALSE otherwise
 */
static NvBool IsSbkOrDkSet(void)
{
    NvU32 AllKeysOred;

    AllKeysOred  = NvDdkFuseIsSbkSet();
    AllKeysOred |= FUSE_NV_READ32(FUSE_PRIVATE_KEY4_NONZERO_0);

    if (AllKeysOred)
        return NV_TRUE;
    else
        return NV_FALSE;
}


/**
 * Reports whether the ODM Production Mode fuse is set (burned)
 *
 * Note that this fuse by itself does not determine whether the chip is in
 * ODM Production Mode.
 *
 * @param none
 *
 * @return NV_TRUE if ODM Production Mode fuse is set (burned); else NV_FALSE
 */
static NvBool NvDdkFuseIsOdmProductionModeFuseSet(void)
{
    NvU32 RegValue;
    RegValue = FUSE_NV_READ32(FUSE_SECURITY_MODE_0);
    if (RegValue)
    {
        return NV_TRUE;
    }
    else
    {
        return NV_FALSE;
    }
}


/**
 * Reports whether the Disable Register Program register is set.
 *
 * @param none
 *
 * @return NV_TRUE if  Disable reg program is iset, else NV_FALSE
 */

static NvBool NvDdkPrivIsDisableRegProgramSet(void)
{
    NvU32 RegValue;
    RegValue = FUSE_NV_READ32(FUSE_DISABLEREGPROGRAM_0);
    if (RegValue)
    {
        return NV_TRUE;
    }
    else
    {
        return NV_FALSE;
    }
}

static NvBool NvDdkFuseIsOdmProductionMode(void)
{
    if (NvDdkFuseIsOdmProductionModeFuseSet())
    {
        return NV_TRUE;
    }
    return NV_FALSE;
}

static NvU32 NvDdkFuseGetSecBootDeviceRaw(void)
{
    NvU32 RegData;

    RegData = FUSE_NV_READ32(FUSE_RESERVED_SW_0);
    RegData = NV_DRF_VAL(FUSE, RESERVED_SW, BOOT_DEVICE_SELECT, RegData);
    if (RegData >= (NvU32) FuseBootDev_Max)
    {
        RegData = FuseBootDev_Sdmmc;
    }

    return RegData;
}

static NvDdkSecBootDeviceType NvDdkFuseGetSecBootDevice(void)
{
    NvU32                 RegData;
    NvDdkSecBootDeviceType SecBootDevice;

    RegData = NvDdkFuseGetSecBootDeviceRaw();

    // Map from AP20 definitions onto NvDdkSecBootDeviceType definitions.
    SecBootDevice = FuseToNvDdkBootDeviceSel[RegData];

    return SecBootDevice;
}

// p = prefix, c = copy, i = index, d = data

#define FUSE_BASE(p, c, i) FUSE_##p##__##c##_ALIAS_##i
#define FUSE_DATA(p, c, i) FUSE_##p##__##c##_ALIAS_##i##_DATA
#define FUSE_WIDTH(p, c, i) FUSE_##p##__##c##_ALIAS_##i##_WIDTH

#define SET_FUSE(p, c, i, d)                       \
    s_FuseArray[FUSE_BASE(p, c, i)] =              \
    (s_FuseArray[FUSE_BASE(p, c, i)] &             \
    ~NV_FIELD_SHIFTMASK(FUSE_DATA(p, c, i))) |     \
    ((d & (NV_FIELD_MASK(FUSE_DATA(p, c, i)))) <<  \
    (NV_FIELD_SHIFT(FUSE_DATA(p, c, i))))

#define SET_MASK(p, c, i) \
    s_MaskArray[FUSE_BASE(p,c,i)] |= NV_FIELD_SHIFTMASK(FUSE_DATA(p,c,i))

#define SET_FUSE_DATA(p, c, i, d) \
    SET_FUSE(p,c,i,d);            \
    SET_MASK(p,c,i);

#define SET_FUSE_PRI(p, i, d) SET_FUSE_DATA(p, PRI, i, d);
#define SET_FUSE_RED(p, i, d) SET_FUSE_DATA(p, RED, i, d);

#define SET_FUSE_BOTH(p, i, d)    \
    SET_FUSE_PRI(p,i,d);  \
    SET_FUSE_RED(p,i,d);

// Update fuses with an explicit mask.
#define SET_FUSE_BOTH_WITH_MASK(p, i, d, md)         \
    SET_FUSE(p, PRI, i, d);                       \
    SET_FUSE(p, RED, i, d);                       \
    s_MaskArray[FUSE_BASE(p, PRI, i)] |=          \
      (md << NV_FIELD_SHIFT(FUSE_DATA(p, PRI, i))); \
    s_MaskArray[FUSE_BASE(p, RED, i)] |=          \
      (md << NV_FIELD_SHIFT(FUSE_DATA(p, RED, i)));

#define UPDATE_DATA(p,i,d) (d >>= FUSE_WIDTH(p,PRI,i));

#define SET_SPARE_BIT(n)                   \
    SET_FUSE_PRI(SPARE_BIT_##n, 0, Data);  \
    UPDATE_DATA (SPARE_BIT_##n, 0, Data);


static void MapDataToFuseArrays(void)
{
    NvU32 *Src;
    NvU32  Data = 0;
    NvU32  MaskData;

    // Start by clearing the arrays.
    fusememset(s_FuseArray, 0, sizeof(s_FuseArray));
    fusememset(s_MaskArray, 0, sizeof(s_MaskArray));
    // Set ENABLE_FUSE_PROGRAM bit to 1
    SET_FUSE_BOTH(ENABLE_FUSE_PROGRAM, 0, 1);

    // Reserved Odm fuse
    if (s_FuseData.ProgramFlags & FLAGS_PROGRAM_RESERVED_ODM)
    {
        Src = (NvU32*)(s_FuseData.ReservedOdm);
        Data = Src[0];
        SET_FUSE_BOTH(RESERVED_ODM0, 0, Data);
        UPDATE_DATA  (RESERVED_ODM0, 0, Data);
        SET_FUSE_BOTH(RESERVED_ODM0, 1, Data);

        Data = Src[1];
        SET_FUSE_BOTH(RESERVED_ODM1, 0, Data);
        UPDATE_DATA  (RESERVED_ODM1, 0, Data);
        SET_FUSE_BOTH(RESERVED_ODM1, 1, Data);

        Data = Src[2];
        SET_FUSE_BOTH(RESERVED_ODM2, 0, Data);
        UPDATE_DATA  (RESERVED_ODM2, 0, Data);
        SET_FUSE_BOTH(RESERVED_ODM2, 1, Data);

        Data = Src[3];
        SET_FUSE_BOTH(RESERVED_ODM3, 0, Data);
        UPDATE_DATA  (RESERVED_ODM3, 0, Data);
        SET_FUSE_BOTH(RESERVED_ODM3, 1, Data);

        Data = Src[4];
        SET_FUSE_BOTH(RESERVED_ODM4, 0, Data);
        UPDATE_DATA  (RESERVED_ODM4, 0, Data);
        SET_FUSE_BOTH(RESERVED_ODM4, 1, Data);

        Data = Src[5];
        SET_FUSE_BOTH(RESERVED_ODM5, 0, Data);
        UPDATE_DATA  (RESERVED_ODM5, 0, Data);
        SET_FUSE_BOTH(RESERVED_ODM5, 1, Data);

        Data = Src[6];
        SET_FUSE_BOTH(RESERVED_ODM6, 0, Data);
        UPDATE_DATA  (RESERVED_ODM6, 0, Data);
        SET_FUSE_BOTH(RESERVED_ODM6, 1, Data);

        Data = Src[7];
        SET_FUSE_BOTH(RESERVED_ODM7, 0, Data);
        UPDATE_DATA  (RESERVED_ODM7, 0, Data);
        SET_FUSE_BOTH(RESERVED_ODM7, 1, Data);
    }

    // If Secure mode is set, we can not burn any other fuses, so return.
    if (NvDdkFuseIsOdmProductionModeFuseSet())
    {
        return;
    }

    if (s_FuseData.ProgramFlags & FLAGS_PROGRAM_JTAG_DISABLE)
    {
        Data = (NvU32)(s_FuseData.JtagDisable);
        SET_FUSE_BOTH(ARM_DEBUG_DIS, 0, Data);
    }

    if (s_FuseData.ProgramFlags & FLAGS_PROGRAM_ODM_PRODUCTION)
    {
        Data = (NvU32)(s_FuseData.OdmProduction);
        SET_FUSE_BOTH(SECURITY_MODE, 0, Data);
    }

    // SBK & DK
    if (s_FuseData.ProgramFlags & FLAGS_PROGRAM_SECURE_BOOT_KEY)
    {
        Src = (NvU32*)(s_FuseData.SecureBootKey);
        Data = Src[0];
        SET_FUSE_BOTH(PRIVATE_KEY0, 0, Data);
        UPDATE_DATA  (PRIVATE_KEY0, 0, Data);
        SET_FUSE_BOTH(PRIVATE_KEY0, 1, Data);

        Data = Src[1];
        SET_FUSE_BOTH(PRIVATE_KEY1, 0, Data);
        UPDATE_DATA  (PRIVATE_KEY1, 0, Data);
        SET_FUSE_BOTH(PRIVATE_KEY1, 1, Data);

        Data = Src[2];
        SET_FUSE_BOTH(PRIVATE_KEY2, 0, Data);
        UPDATE_DATA  (PRIVATE_KEY2, 0, Data);
        SET_FUSE_BOTH(PRIVATE_KEY2, 1, Data);

        Data = Src[3];
        SET_FUSE_BOTH(PRIVATE_KEY3, 0, Data);
        UPDATE_DATA  (PRIVATE_KEY3, 0, Data);
        SET_FUSE_BOTH(PRIVATE_KEY3, 1, Data);
    }

    if (s_FuseData.ProgramFlags & FLAGS_PROGRAM_DEVICE_KEY)
    {
        Src = (NvU32*)(s_FuseData.DeviceKey);
        Data = Src[0];
        SET_FUSE_BOTH(PRIVATE_KEY4, 0, Data);
        UPDATE_DATA  (PRIVATE_KEY4, 0, Data);
        SET_FUSE_BOTH(PRIVATE_KEY4, 1, Data);
    }

    if (s_FuseData.ProgramFlags & FLAGS_PROGRAM_BOOT_DEV_CONFIG)
    {
        // BootDeviceConfig is 14 bits so mask other bits
        Data = (s_FuseData.SecBootDeviceConfig  & 0x3FFF);
        SET_FUSE_BOTH(BOOT_DEVICE_INFO, 0, Data);
    }

    // Assemble RESERVED_SW
    if ((s_FuseData.ProgramFlags & FLAGS_PROGRAM_BOOT_DEV_SEL       ) ||
        (s_FuseData.ProgramFlags & FLAGS_PROGRAM_SKIP_DEV_SEL_STRAPS) ||
        (s_FuseData.ProgramFlags & FLAGS_PROGRAM_SW_RESERVED        ))
    {
        Data = 0;
        MaskData = 0;

        if (s_FuseData.ProgramFlags & FLAGS_PROGRAM_BOOT_DEV_SEL)
        {
            Data     |= (s_FuseData.SecBootDeviceSelectRaw & 0x7);
            MaskData |= 0x7;
        }

        if (s_FuseData.ProgramFlags & FLAGS_PROGRAM_SKIP_DEV_SEL_STRAPS)
        {
            Data     |= ((s_FuseData.SkipDevSelStraps & 0x1) << 3);
            MaskData |= (0x1 << 3);
        }

        if (s_FuseData.ProgramFlags & FLAGS_PROGRAM_SW_RESERVED)
        {
            Data     |= ((s_FuseData.SwReserved & 0xF) << 4);
            MaskData |= (0xF <<  4);
        }

        SET_FUSE_BOTH_WITH_MASK(RESERVED_SW, 0, Data, MaskData);
    }
}


static void FuseCopyBytes(NvU32 RegAddress, NvU8 *pByte, const NvU32 nBytes)
{
    NvU32 RegData;
    NvU32 i;

     NV_ASSERT((pByte != NULL) || (nBytes == 0));
     NV_ASSERT (RegAddress != 0);

    for (i = 0, RegData = 0; i < nBytes; i++)
    {
        if ((i&3) == 0)
        {
            RegData = FUSE_NV_READ32(RegAddress);
            RegAddress += 4;
        }
        pByte[i] = RegData & 0xFF;
        RegData >>= 8;
    }
}

// Expose (Visibility = 1) or hide (Visibility = 0) the fuse registers.
static void SetFuseRegVisibility(NvU32 Visibility)
{
    NvU32 RegData;

    RegData = CLOCK_NV_READ32(CLK_RST_CONTROLLER_MISC_CLK_ENB_0);
    RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER,
                                 MISC_CLK_ENB,
                                 CFG_ALL_VISIBLE,
                                 Visibility,
                                 RegData);
    CLOCK_NV_WRITE32(CLK_RST_CONTROLLER_MISC_CLK_ENB_0, RegData);
}

// Wait for completion (state machine goes idle).
static void WaitForFuseIdle(void)
{
    NvU32 RegData;

    do
    {
        RegData = FUSE_NV_READ32(FUSE_FUSECTRL_0);
    } while (NV_DRF_VAL(FUSE, FUSECTRL, FUSECTRL_STATE, RegData) !=
             FUSE_FUSECTRL_0_FUSECTRL_STATE_STATE_IDLE);
}


// Start the regulator and wait
static void StartRegulator(void)
{
    NvU32 RegData;
    RegData = NV_DRF_DEF(FUSE, PWR_GOOD_SW, PWR_GOOD_SW_VAL, PWR_GOOD_OK);
    FUSE_NV_WRITE32(FUSE_PWR_GOOD_SW_0, RegData);
    /*
     * Wait for at least 150ns. In HDEFUSE_64X32_E2F2R2_L1_A.pdf,
     * this is found on p.11 as TSUP_PWRGD.
     */
    NvFuseUtilWaitUS(1);
}

// Stop the regulator and wait
static void StopRegulator(void)
{
    NvU32 RegData;

    RegData = NV_DRF_DEF(FUSE, PWR_GOOD_SW, PWR_GOOD_SW_VAL, PWR_GOOD_NOT_OK);
    FUSE_NV_WRITE32(FUSE_PWR_GOOD_SW_0, RegData);

    /*
     * Wait for at least 150ns. In HDEFUSE_64X32_E2F2R2_L1_A.pdf,
     *  this is found on p.11 as TSUP_PWRGD
     */
    NvFuseUtilWaitUS(1);
}

 /*
  * Read a word from the fuses.
  * Note: The fuses must already have been sensed, and
  * the programming power should be off.
  */
static NvU32 ReadFuseWord(NvU32 Addr)
{
    NvU32 RegData;

    // Prepare the data
    FUSE_NV_WRITE32(FUSE_FUSEADDR_0, Addr);

    // Trigger the read
    RegData = FUSE_NV_READ32(FUSE_FUSECTRL_0);
    RegData = NV_FLD_SET_DRF_DEF(FUSE, FUSECTRL, FUSECTRL_CMD, READ, RegData);
    FUSE_NV_WRITE32(FUSE_FUSECTRL_0, RegData);

    // Wait for completion (state machine goes idle).
    WaitForFuseIdle();

    RegData = FUSE_NV_READ32(FUSE_FUSERDATA_0);

    return RegData;
}

/*
 * Write a word to the fuses.
 * Note: The programming power should be on, and the only non-zero
 * bits should be fuses that have not yet been blown.
 */
static void WriteFuseWord(NvU32 Addr, NvU32 Data)
{
    NvU32 RegData;

    if (Data == 0) return;

    // Prepare the data
    FUSE_NV_WRITE32(FUSE_FUSEADDR_0,  Addr);
    FUSE_NV_WRITE32(FUSE_FUSEWDATA_0, Data);

    // Trigger the write
    RegData = FUSE_NV_READ32(FUSE_FUSECTRL_0);
    RegData = NV_FLD_SET_DRF_DEF(FUSE, FUSECTRL, FUSECTRL_CMD, WRITE, RegData);
    FUSE_NV_WRITE32(FUSE_FUSECTRL_0, RegData);

    // Wait for completion (state machine goes idle).
    WaitForFuseIdle();
}

// Sense the fuse array & wait until done.
static void SenseFuseArray(void)
{
    NvU32 RegData;

    RegData = FUSE_NV_READ32(FUSE_FUSECTRL_0);
    RegData = NV_FLD_SET_DRF_DEF(FUSE,
                                 FUSECTRL,
                                 FUSECTRL_CMD,
                                 SENSE_CTRL,
                                 RegData);
    FUSE_NV_WRITE32(FUSE_FUSECTRL_0, RegData);

    // Wait for completion (state machine goes idle).
    WaitForFuseIdle();
}


// The following has not appeared in any netlist headers.
#ifndef FUSE_ENABLE_FUSE_PROGRAM__PRI_ALIAS_0
#define FUSE_ENABLE_FUSE_PROGRAM__PRI_ALIAS_0       0x0
#define FUSE_ENABLE_FUSE_PROGRAM__PRI_ALIAS_0_DATA  0:0
#define FUSE_ENABLE_FUSE_PROGRAM__PRI_ALIAS_0_WIDTH 1
#define FUSE_ENABLE_FUSE_PROGRAM__RED_ALIAS_0       0x1
#define FUSE_ENABLE_FUSE_PROGRAM__RED_ALIAS_0_DATA  0:0
#define FUSE_ENABLE_FUSE_PROGRAM__RED_ALIAS_0_WIDTH 1
#endif
#ifndef FUSE_ENABLE_FUSE_PROGRAM__PRI_ALIAS_0_SHIFT
#define FUSE_ENABLE_FUSE_PROGRAM__PRI_ALIAS_0_SHIFT 0
#define FUSE_ENABLE_FUSE_PROGRAM__RED_ALIAS_0_SHIFT 0
#endif

static void
NvDdkFuseProgramFuseArray(
                                NvU32 *FuseData,
                                NvU32 *MaskData,
                                NvU32  NumWords,
                                NvU32  TProgramCycles)
{
    NvU32  RegData;
    NvU32  i;
    NvU32  Addr = 0;
    NvU32 *Data = NULL;
    NvU32 *Mask = NULL;
    NvU32  FuseWord0; // Needed for initially enabling fuse programming.
    NvU32  FuseWord1; // Needed for initially enabling fuse programming.

    // Make all registers visible first
    SetFuseRegVisibility(1);
    // Read the first two fuse words.
    SenseFuseArray();
    FuseWord0 = ReadFuseWord(FUSE_ENABLE_FUSE_PROGRAM__PRI_ALIAS_0);
    FuseWord1 = ReadFuseWord(FUSE_ENABLE_FUSE_PROGRAM__RED_ALIAS_0);
    // also indicate power good to start the regulator and wait (again)
    StartRegulator();
    /*
     *In AP20, the fuse macro is a high density macro, with a completely
    * different interface from the procedure used in AP15.  Fuses are
    * burned using an addressing mechanism, so no need to prepare
    * the full list, but more write to control registers are needed, not
    * a big deal
    * The only bit that can be written at first is bit 0, a special write
    * protection bit by assumptions all other bits are at 0
    * Note that the bit definitions are no more part of arfuse.h
    * (unfortunately). So we define them here
    *
    * Modify the fuse programming time.
    *
    * For AP20, the programming pulse must have a precise width of
    * [9000, 11000] ns.
    */
    if (TProgramCycles > 0)
    {
        RegData = NV_DRF_NUM(FUSE,
                             FUSETIME_PGM2,
                             FUSETIME_PGM2_TWIDTH_PGM,
                             TProgramCycles);
        FUSE_NV_WRITE32(FUSE_FUSETIME_PGM2_0, RegData);
    }
    /* FuseWord0 and FuseWord1 should be left with only the Fuse Enable fuse
     * set to 1, and then only if this fuse has not yet been burned.
     */
    FuseWord0 = (0x1 << FUSE_ENABLE_FUSE_PROGRAM__PRI_ALIAS_0_SHIFT) &
      ~FuseWord0;
    FuseWord1 = (0x1 << FUSE_ENABLE_FUSE_PROGRAM__RED_ALIAS_0_SHIFT) &
      ~FuseWord1;

    WriteFuseWord(FUSE_ENABLE_FUSE_PROGRAM__PRI_ALIAS_0, FuseWord0);
    WriteFuseWord(FUSE_ENABLE_FUSE_PROGRAM__RED_ALIAS_0, FuseWord1);
    // Remove power good as we may toggle fuse_src in the model, wait
    StopRegulator();
    /*
     * Sense the fuse macro, this will allow programming of other fuses
     * and the reading of the existing fuse values
     */
    SenseFuseArray();
    /*
     * Clear out all bits in FuseData that have already been burned
     * or that have been masked out.
     */

    fusememcpy(s_TempFuseData, FuseData, sizeof(NvU32) * 64);
    Addr = 0;
    Data = s_TempFuseData;
    Mask = MaskData;
    for (i = 0; i < NumWords; i++, Addr++, Data++, Mask++)
    {
        RegData = ReadFuseWord(Addr);
        *Data = (*Data & ~RegData) & *Mask;
    }

    // Enable power good
    StartRegulator();
    /*
      * Finally loop on all fuses, program the non zero ones
     * Words 0 and 1 are written last and they contain control fuses and we
     * need a sense after writing to a control word (with the exception of
     * the master enable fuse) this is also the reason we write them last
     */
    Addr = 2;
    Data = s_TempFuseData + Addr;
    for (; Addr < NumWords; Addr++, Data++)
    {
        WriteFuseWord(Addr, *Data);
    }
    // write the two control words, we need a sense after each write
    Addr = 0;
    Data = s_TempFuseData;
    for (; Addr < NV_MIN(2, NumWords); Addr++, Data++)
    {
        WriteFuseWord(Addr, *Data);
        // Remove power good as we may toggle fuse_src in the model, wait
        StopRegulator();
        SenseFuseArray();
        StartRegulator();
    }
    // Read all data into the chip options
    RegData = NV_DRF_NUM(FUSE, PRIV2INTFC_START, PRIV2INTFC_START_DATA, 0x1);
    FUSE_NV_WRITE32(FUSE_PRIV2INTFC_START_0, RegData);
    /*
     * Not sure if still needs to be set back to 0
     * we wait a little bit, then set it back to 0, then loop on state
     */

    NvFuseUtilWaitUS(1);

    RegData = NV_DRF_NUM(FUSE, PRIV2INTFC_START, PRIV2INTFC_START_DATA, 0x0);
    FUSE_NV_WRITE32(FUSE_PRIV2INTFC_START_0, RegData);
    /*
      * Wait until done (polling)
      * this one needs to use fuse_sense done, the FSM follows a periodic
      * sequence that includes idle
      */
    do {
        RegData = FUSE_NV_READ32(FUSE_FUSECTRL_0);
  } while (NV_DRF_VAL(FUSE, FUSECTRL, FUSECTRL_FUSE_SENSE_DONE, RegData) != 0x1);
}

static NvError
NvDdkFuseVerifyFuseArray(
                                    NvU32 *FuseData,
                                    NvU32 *MaskData,
                                    NvU32 NumWords)
{
    NvU32        RegData;
    NvU32        i;
    NvU32        Addr = 0;
    NvU32       *Data = NULL;
    NvU32       *Mask = NULL;
    NvError  e   = NvSuccess;

    // Make all registers visible first
    SetFuseRegVisibility(1);

    // Sense the fuse array
    SenseFuseArray();

    // Loop over the data, checking fuse registers.
    Addr = 0;
    Data = FuseData;
    Mask = MaskData;
    for (i = 0; i < NumWords; i++, Addr++, Data++, Mask++)
    {
        RegData = (ReadFuseWord(Addr) & (*Mask));

        /*
          * Once Odm production mode fuse word(0th word) is set, it can
         * not set any other fuse word  in the fuse cell, including the
         * redundent fuse word (1st word).
         * Hence it makes no sense to verify it.
         */
        if ((i == 1) && NvDdkFuseIsOdmProductionMode())
        {
            continue;
        }

        // Check the results.
        if (RegData != ((*Data) & (*Mask)))
        {
            // Abort with an error on a miscompare
            e = NvError_BadValue;
            goto fail;
        }
    }

    // Fallthrough on sucessful completion.

 fail:
    // Hide back registers
    SetFuseRegVisibility(0);
    return e;
}

/*
 *************************************
 * PUBLIC API
 *************************************
 */

/**
 * Clears the cache of fuse data.
 */
void NvDdkFuseClear(void)
{
    if(!s_pFuseRec)
    {
        // NV_ASSERT(0);
        NvOsDebugPrintf("\r\n Plz call NvDdkFuseOpen before using this API");
        return;
    }
    NvOsMutexLock(s_pFuseRec->Mutex);
    fusememset(&s_FuseData, 0, sizeof(NvDdkFuseData));
    NvOsMutexUnlock(s_pFuseRec->Mutex);
}

/**
 * Read the current fuse data into the fuse registers.
 */
void NvDdkFuseSense(void)
{
    if(!s_pFuseRec)
    {
        // NV_ASSERT(0);
        NvOsDebugPrintf("\r\n Plz call NvDdkFuseOpen before using this API");
        return;
    }
    NvOsMutexLock(s_pFuseRec->Mutex);
    SetFuseRegVisibility(1);
    SenseFuseArray();
    SetFuseRegVisibility(0);
    NvOsMutexUnlock(s_pFuseRec->Mutex);
}

/**
 * Macro's to swap byte order for an NvU32
 */

#define EXTRACT_BYTE_NVU32(Data32, ByteNum) \
    (((Data32) >> ((ByteNum)*8)) & 0xFF)

#define SWAP_BYTES_NVU32(Data32)                    \
    do {                                            \
         NV_ASSERT(sizeof(Data32)==4);               \
        (Data32) =                                  \
            (EXTRACT_BYTE_NVU32(Data32, 0) << 24) | \
            (EXTRACT_BYTE_NVU32(Data32, 1) << 16) | \
            (EXTRACT_BYTE_NVU32(Data32, 2) <<  8) | \
            (EXTRACT_BYTE_NVU32(Data32, 3) <<  0);  \
    } while (0)

#if FUSE_DUMMY_CODE
// Sizes of different fuses in bytes.
static NvU8 DeviceKey[4];
static NvU8 JtagDisable[1];
static NvU8 KeyProgrammed[1];
static NvU8 OdmProduction[1];
static NvU8 SecBootDeviceConfig[2];
static NvU8 SecBootDeviceSelect[1];
static NvU8 SecureBootKey[16];
static NvU8 Sku[4];
static NvU8 SpareBits[4];
static NvU8 SwReserved[1];
static NvU8 SkipDevSelStraps[1];
static NvU8 SecBootDeviceSelectRaw[4];
static NvU8 ReservedOdm[32];

NvError NvDdkFuseGet(NvDdkFuseDataType Type, void *pData, NvU32 *pSize)
{
    switch (Type)
    {
        case NvDdkFuseDataType_DeviceKey:
            fusememcpy(pData, DeviceKey, sizeof(DeviceKey));
            break;
        case NvDdkFuseDataType_JtagDisable:
            fusememcpy(pData, JtagDisable, sizeof(JtagDisable));
            break;
        case NvDdkFuseDataType_KeyProgrammed:
            fusememcpy(pData, KeyProgrammed, sizeof(KeyProgrammed));
            break;
        case NvDdkFuseDataType_OdmProduction:
            fusememcpy(pData, OdmProduction, sizeof(OdmProduction));
            break;
        case NvDdkFuseDataType_SecBootDeviceConfig:
            fusememcpy(pData, SecBootDeviceConfig, sizeof(SecBootDeviceConfig));
            break;
        case NvDdkFuseDataType_SecBootDeviceSelect:
            fusememcpy(pData, SecBootDeviceSelect, sizeof(SecBootDeviceSelect));
            break;
        case NvDdkFuseDataType_SecureBootKey:
            fusememcpy(pData, SecureBootKey, sizeof(SecureBootKey));
            break;
        case NvDdkFuseDataType_Sku:
            fusememcpy(pData, Sku, sizeof(Sku));
            break;
        case NvDdkFuseDataType_SpareBits:
            fusememcpy(pData, SpareBits, sizeof(SpareBits));
            break;
        case NvDdkFuseDataType_SwReserved:
            fusememcpy(pData, SwReserved, sizeof(SwReserved));
            break;
        case NvDdkFuseDataType_SkipDevSelStraps:
            fusememcpy(pData, SkipDevSelStraps, sizeof(SkipDevSelStraps));
            break;
        case NvDdkFuseDataType_SecBootDeviceSelectRaw:
            fusememcpy(pData, SecBootDeviceSelectRaw, sizeof(SecBootDeviceSelectRaw));
            break;
        case NvDdkFuseDataType_ReservedOdm:
            fusememcpy(pData, ReservedOdm, sizeof(ReservedOdm));
            break;
        default:
            NvOsDebugPrintf("\r\n Invalid fuse type selected");
            break;
    }
    return NvSuccess;
}
#else
/**
 * This gets value from the fuse cache.
 *
 * To read from the actual fuses, NvDdkFuseSense() must be called first.
 * Note that NvDdkFuseSet() follwed by NvDdkFuseGet() for the same data will
 * return the set data, not the actual fuse values.
 *
 * By passing a size of zero, the caller is requesting tfor the
 * expected size.
 */
NvError NvDdkFuseGet(NvDdkFuseDataType Type, void *pData, NvU32 *pSize)
{
    NvU32 RegData;
    NvU32 Size;
    NvU32 DataSizeArrayLen;

    if(!s_pFuseRec)
    {
        NvOsDebugPrintf("\r\n Plz call NvDdkFuseOpen before using this API");
        return 1;
    }
    DataSizeArrayLen = ((sizeof(s_DataSize)) / (sizeof(NvU32)));
    // Check the arguments
    // NvDdkFuseDataType_Num is not ap20 specific as s_DataSize
    if (Type == NvDdkFuseDataType_None ||
        (Type > (NvDdkFuseDataType)DataSizeArrayLen))
            return NvError_BadValue;
    if (pSize == NULL) return NvError_BadParameter;

    Size = s_DataSize[(NvU32)Type];

    if (*pSize == 0)
    {
        *pSize = Size;
        return NvError_Success;
    }

//    if (*pSize > Size) return NvError_InsufficientMemory;
    if (pData == NULL) return NvError_BadParameter;

    NvOsMutexLock(s_pFuseRec->Mutex);
    switch (Type)
    {
        case NvDdkFuseDataType_DeviceKey:
            /*
             * Boot ROM expects DK to be stored in big-endian byte order;
             * since cpu is little-endian and client treats data as an NvU32,
             * perform byte swapping here
             */
            RegData = FUSE_NV_READ32(FUSE_PRIVATE_KEY4_0);
            SWAP_BYTES_NVU32(RegData);
            *((NvU32*)pData) = RegData;
            break;

        case NvDdkFuseDataType_JtagDisable:
            RegData = FUSE_NV_READ32(FUSE_ARM_DEBUG_DIS_0);
            *((NvBool*)pData) = RegData ? NV_TRUE : NV_FALSE;
            break;

        case NvDdkFuseDataType_KeyProgrammed:
            *((NvBool*)pData) = IsSbkOrDkSet();
            break;

        case NvDdkFuseDataType_OdmProduction:
            *((NvBool*)pData) = NvDdkFuseIsOdmProductionModeFuseSet();
            break;

        case NvDdkFuseDataType_SecBootDeviceConfig:
            RegData = FUSE_NV_READ32(FUSE_BOOT_DEVICE_INFO_0);
            RegData = NV_DRF_VAL(FUSE,
                                 BOOT_DEVICE_INFO,
                                 BOOT_DEVICE_CONFIG,
                                 RegData);
            *((NvU8 *)pData) = (RegData >> 0x8) & 0xFF;
            *((NvU8 *)pData + 1) = RegData & 0xFF;
            break;

        case NvDdkFuseDataType_SecBootDeviceSelect:
            RegData = (NvU32)NvDdkFuseGetSecBootDevice();
            *((NvU32*)pData) = RegData;
            break;

        case NvDdkFuseDataType_SecBootDeviceSelectRaw:
            RegData = NvDdkFuseGetSecBootDeviceRaw();
            *((NvU32*)pData) = RegData;
            break;

        case NvDdkFuseDataType_SecureBootKey:
            FuseCopyBytes(FUSE_PRIVATE_KEY0_0,
                          pData,
                          NVDDK_SECURE_BOOT_KEY_BYTES);
            break;

        case NvDdkFuseDataType_Sku:
            *((NvU32*)pData) = FUSE_NV_READ32(FUSE_SKU_INFO_0);
            break;

        case NvDdkFuseDataType_SwReserved:
            RegData = FUSE_NV_READ32(FUSE_RESERVED_SW_0);
            RegData = NV_DRF_VAL(FUSE, RESERVED_SW, SW_RESERVED, RegData);
            *((NvU32*)pData) = RegData;
            break;

        case NvDdkFuseDataType_SkipDevSelStraps:
            RegData = FUSE_NV_READ32(FUSE_RESERVED_SW_0);
            RegData = NV_DRF_VAL(FUSE, RESERVED_SW, SKIP_DEV_SEL_STRAPS, RegData);
            *((NvU32*)pData) = RegData;
            break;

        case NvDdkFuseDataType_ReservedOdm:
            FuseCopyBytes(FUSE_RESERVED_ODM0_0,
                          pData,
                          NVDDK_RESERVED_ODM_BYTES);
            break;

        default:
            NvOsMutexUnlock(s_pFuseRec->Mutex);
            return(NvError_BadValue);
    }
    NvOsMutexUnlock(s_pFuseRec->Mutex);
    return NvError_Success;
}
#endif

#if ENABLE_FUSE_VOLTAGE
static NvRmDeviceHandle *ps_hRmDevice = NULL;
#endif
NvError NvDdkFuseOpen(NvRmDeviceHandle hRmDevice)
{
    NvError e;

    // Allocate memory for handle.
    s_pFuseRec = NvOsAlloc(sizeof(NvDdkFuse));
    if (s_pFuseRec == NULL)
    {
        e = NvError_InsufficientMemory;
        NvOsDebugPrintf("\r\n NvDdkFuseOpen malloc failed");
        return e;
    }

    e = NvOsMutexCreate(&s_pFuseRec->Mutex);
    if (e)
    {
        NvOsDebugPrintf("\r\n NvDdkFuseOpen Mutex creation failed");
        goto fail;
    }

#if ENABLE_FUSE_VOLTAGE
    ps_hRmDevice = NvOsAlloc(sizeof(NvRmDeviceHandle));
    if (ps_hRmDevice == NULL)
    {
        e = NvError_InsufficientMemory;
        NvOsDebugPrintf("\r\n NvDdkFuseOpen malloc failed");
        return e;
    }

    *ps_hRmDevice = hRmDevice;
#endif
    // Get the base address of the Fuse registers
    NvRmModuleGetBaseAddress(hRmDevice,
        NVRM_MODULE_ID(NvRmModuleID_Fuse, 0),
        &(s_pFuseRec->pBaseAddress), &(s_pFuseRec->BankSize));
    NV_CHECK_ERROR_CLEANUP(NvRmPhysicalMemMap(s_pFuseRec->pBaseAddress,
        s_pFuseRec->BankSize, NVOS_MEM_READ_WRITE, NvOsMemAttribute_Uncached,
        (void **)&(s_pFuseRec->pVirtualAddress)));

    // Get the base address of the CAR registers
    NvRmModuleGetBaseAddress(hRmDevice,
        NVRM_MODULE_ID(NvRmPrivModuleID_ClockAndReset, 0),
        &(s_pFuseRec->pCarBaseAddress), &(s_pFuseRec->CarBankSize));
    NV_CHECK_ERROR_CLEANUP(NvRmPhysicalMemMap(s_pFuseRec->pCarBaseAddress,
        s_pFuseRec->CarBankSize, NVOS_MEM_READ_WRITE, NvOsMemAttribute_Uncached,
        (void **)&(s_pFuseRec->pCarVirtualAddress)));

    return NvSuccess;
fail:
    NvOsFree(s_pFuseRec);
    return e;
}

void NvDdkFuseClose()
{
    NvOsDebugPrintf("\r\n NvDdkFuseClose ");
    NvOsMutexDestroy(s_pFuseRec->Mutex);
    NvOsFree(s_pFuseRec);
    s_pFuseRec = NULL;
}

#if FUSE_DUMMY_CODE
NvError NvDdkFuseSet(NvDdkFuseDataType Type, void *pData, NvU32 *pSize)
{
#if ENABLE_DEBUG_PRINTS
    NvU8 i;
    NvOsDebugPrintf("\n data to be written:");
    for(i = 0; i < *pSize; i++)
        NvOsDebugPrintf("\t 0x%x", *((NvU8 *)pData + i));
#endif
    switch (Type)
    {
        case NvDdkFuseDataType_DeviceKey:
            fusememcpy(DeviceKey, pData, sizeof(DeviceKey));
            break;
        case NvDdkFuseDataType_JtagDisable:
            fusememcpy(JtagDisable, pData, sizeof(JtagDisable));
            break;
        case NvDdkFuseDataType_KeyProgrammed:
            fusememcpy(KeyProgrammed, pData, sizeof(KeyProgrammed));
            break;
        case NvDdkFuseDataType_OdmProduction:
            fusememcpy(OdmProduction, pData, sizeof(OdmProduction));
            break;
        case NvDdkFuseDataType_SecBootDeviceConfig:
            fusememcpy(SecBootDeviceConfig, pData, sizeof(SecBootDeviceConfig));
            break;
        case NvDdkFuseDataType_SecBootDeviceSelect:
            fusememcpy(SecBootDeviceSelect, pData, sizeof(SecBootDeviceSelect));
            break;
        case NvDdkFuseDataType_SecureBootKey:
            fusememcpy(SecureBootKey, pData, sizeof(SecureBootKey));
            break;
        case NvDdkFuseDataType_Sku:
            fusememcpy(Sku, pData, sizeof(Sku));
            break;
        case NvDdkFuseDataType_SpareBits:
            fusememcpy(SpareBits, pData, sizeof(SpareBits));
            break;
        case NvDdkFuseDataType_SwReserved:
            fusememcpy(SwReserved, pData, sizeof(SwReserved));
            break;
        case NvDdkFuseDataType_SkipDevSelStraps:
            fusememcpy(SkipDevSelStraps, pData, sizeof(SkipDevSelStraps));
            break;
        case NvDdkFuseDataType_SecBootDeviceSelectRaw:
            fusememcpy(SecBootDeviceSelectRaw, pData, sizeof(SecBootDeviceSelectRaw));
            break;
        case NvDdkFuseDataType_ReservedOdm:
            fusememcpy(ReservedOdm, pData, sizeof(ReservedOdm));
            break;
        default:
            NvOsDebugPrintf("\r\n Invalid fuse type selected");
            break;
    }
    return NvSuccess;
}

#else

/**
 * Schedule fuses to be programmed to the specified values when the next
 * NvDdkFuseProgram() operation is performed
 *
 * By passing a size of zero, the caller is requesting to be told the
 * expected size.
 */
#define FUSE_SET(name, flag)                                              \
    case NvDdkFuseDataType_##name:                                         \
    {                                                                     \
        NvU8 *p = (NvU8 *)&(p_FuseData.name);                           \
        NvU32 i;                                                          \
        NvError e = NvSuccess;  \
        /* read existing fuse value */                                    \
        e = NvDdkFuseGet(NvDdkFuseDataType_##name, p, &Size);   \
        if (e != NvSuccess)     \
        {       \
            NvOsDebugPrintf("\r\n Err returned from Fuse Get:0x%x in Set",e);   \
            goto fail;  \
        }       \
        if (Type == NvDdkFuseDataType_SecBootDeviceConfig)      \
        {   \
            Data = *(NvU16 *)p;     \
            *p = (Data >> 0x8)  & 0xFF;     \
            *(p + 1) = Data & 0xFF;     \
        }   \
        /* check consistency between existing and desired fuse values. */ \
        /* fuses cannot be unburned, so desired value cannot specify   */ \
        /* any unburned (0x0) bits where the existing value already    */ \
        /* contains burned (0x1) bits.                                 */ \
        for (i=0; i<Size; i++)                                            \
            if ((p[i] | pDataPtr[i]) != pDataPtr[i])          \
            { \
                    e = NvError_InvalidState;                     \
                    NvOsDebugPrintf("\n p[%d] = 0x%x, pDataptr[%d] = 0x%x",i, p[i],i,*(NvU32*)pDataPtr); \
                    NvOsDebugPrintf("\r\n Consistency check failure in Fuse Set:0x%x",e); \
                    goto fail; \
            } \
        /* consistency check passed; schedule fuses to be burned */       \
        fusememcpy(&(s_FuseData.name), (void *)pDataPtr, Size);                    \
        s_FuseData.ProgramFlags |= FLAGS_PROGRAM_##flag;                  \
    }                                                                     \
    break

NvError NvDdkFuseSet(NvDdkFuseDataType Type, void *pData, NvU32 *pSize)
{
    NvError e = NvError_Success;
    NvU32 Size;
    NvU32 DataSizeArrayLen;
    NvDdkFuseData p_FuseData;
    volatile NvU8* pDataPtr = (volatile NvU8*)pData;
    NvU16 Data;
    if(!s_pFuseRec)
    {
        // NV_ASSERT(0);
        NvOsDebugPrintf("\r\n Plz call NvDdkFuseOpen before using this API");
        return 1;
    }
    NvOsMutexLock(s_pFuseRec->Mutex);
    DataSizeArrayLen = ((sizeof(s_DataSize)) / (sizeof(NvU32)));

    if (Type == NvDdkFuseDataType_None ||
        (Type > (NvDdkFuseDataType)DataSizeArrayLen))
   {
        e = NvError_BadValue;
        NvOsDebugPrintf("\r\n NvDdkFuseDataType is incorrect");
        goto fail;
    }

    if (pSize == NULL)
   {
        e = NvError_BadParameter;
        goto fail;
    }

    Size = s_DataSize[(NvU32)Type];

    // Return the required size, if requested.
    if (*pSize == 0)
    {
        *pSize = Size;
        e = NvError_Success;
        goto fail;
    }

    if (*pSize > Size)
   {
        NvOsDebugPrintf("\n passed size = %d, fuse size = %d", *pSize, Size);
        e = NvError_InsufficientMemory;
        goto fail;
    }

    if (pData == NULL)
   {
        e = NvError_BadParameter;
        goto fail;
    }

    /*
     * If Disable Reg program is set, chip can not be fused, so return as
     *  "Access denied".
     */
    if (NvDdkPrivIsDisableRegProgramSet())
    {
        e = NvError_AccessDenied;
        goto fail;
    }


    // Only reserved odm fuses are allowed to burn in secure mode
    if (NvDdkFuseIsOdmProductionModeFuseSet() &&
                    (Type != NvDdkFuseDataType_ReservedOdm))
    {
        e = NvError_BadValue;
        NvOsDebugPrintf("\r\n only reserved odm fuses are allowed to burn \
         in secure mode");
        goto fail;
    }

    if (Type == NvDdkFuseDataType_SecBootDeviceConfig)
    {
        Data = *(NvU16 *)pData;
        *((NvU8 *)pDataPtr) = (Data >> 0x8) & 0xFF;
        *((NvU8 *)pDataPtr + 1) = Data & 0xFF;
    }

    switch (Type)
    {
        FUSE_SET(DeviceKey,                 DEVICE_KEY              );
        FUSE_SET(JtagDisable,                JTAG_DISABLE           );
        FUSE_SET(OdmProduction,          ODM_PRODUCTION     );
        FUSE_SET(SecBootDeviceConfig, BOOT_DEV_CONFIG    );

        case NvDdkFuseDataType_SecBootDeviceSelect:
        case NvDdkFuseDataType_SecBootDeviceSelectRaw:
            {
                NvU8 *p = (NvU8 *)&(s_FuseData.SecBootDeviceSelectRaw);
                NvU32 ApiData;  // API symbolic value for device selection
                NvU32 FuseData; // Raw fuse data for the device selection

                if (Type == NvDdkFuseDataType_SecBootDeviceSelect)
                {
                    /* Read out the argument. */

                    fusememcpy(&(ApiData), pData, Size);

                    if (ApiData == 0 || ApiData >= NvDdkSecBootDeviceType_Max)
                    {
                        e = NvError_BadValue;
                        NvOsDebugPrintf("\r\n ApiData is incorrect");
                        goto fail;
                    }

                    /* Map the symbolic value to a fuse value. */
                    FuseData = NvDdkToFuseBootDeviceSel[ApiData];
                }
                else
                {
                    /*
                     * Type == NvDdkFuseDataType_SecBootDeviceSelectRaw
                     * Read out the argument.
                     */
                    fusememcpy(&(FuseData), pData, Size);
                    if (FuseData >= FuseBootDev_Max)
                    {
                        e = NvError_BadValue;
                        NvOsDebugPrintf("\r\n FuseData is incorrect");
                        goto fail;
                    }

                    /* Map the fuse value to a symbolic value */
                    ApiData = FuseToNvDdkBootDeviceSel[FuseData];
                }

                /* read existing fuse value */
                NV_CHECK_ERROR(NvDdkFuseGet(NvDdkFuseDataType_SecBootDeviceSelectRaw,
                                           p, &Size));

                /*
                 * Check consistency between existing and desired fuse values.
                 * fuses cannot be unburned, so desired value cannot specify
                 * any unburned (0x0) bits where the existing value already
                 * contains burned (0x1) bits.
                 */
                if ((s_FuseData.SecBootDeviceSelectRaw | FuseData) != FuseData)
                {
                    e = NvError_InvalidState;
                    goto fail;
                }


                /* Consistency check passed; schedule fuses to be burned */
                s_FuseData.SecBootDeviceSelect    = ApiData;
                s_FuseData.SecBootDeviceSelectRaw = FuseData;
                s_FuseData.ProgramFlags |= FLAGS_PROGRAM_BOOT_DEV_SEL;
            }
            break;

        FUSE_SET(SecureBootKey,      SECURE_BOOT_KEY       );
        FUSE_SET(SwReserved,          SW_RESERVED               );
        FUSE_SET(SkipDevSelStraps,  SKIP_DEV_SEL_STRAPS  );
        FUSE_SET(ReservedOdm,        RESERVED_ODM             );

        default:
        {
            e = NvError_BadValue;
            NvOsDebugPrintf("\r\n Entered default case");
            goto fail;
        }
    }
    // Set the ReservOdm flag if it present
    if (Type == NvDdkFuseDataType_ReservedOdm)
    {
        s_FuseData.ReservedOdmFlag = NV_TRUE;
    }

    // Check for invalid state combinations.
    if ((s_FuseData.ProgramFlags & FLAGS_PROGRAM_ODM_PRODUCTION) &&
        (s_FuseData.ProgramFlags & (FLAGS_PROGRAM_SECURE_BOOT_KEY |
                                    FLAGS_PROGRAM_DEVICE_KEY)))
    {
        s_FuseData.ProgramFlags |= FLAGS_PROGRAM_ILLEGAL;
    }

    if (s_FuseData.ProgramFlags & FLAGS_PROGRAM_ILLEGAL)
    {
        e = NvError_InvalidState;
        goto fail;
    }

fail:
    NvOsMutexUnlock(s_pFuseRec->Mutex);
    if (e != NvSuccess)
        NvOsDebugPrintf("\r\n NvDdkFuseSet exit with err value e = 0x%x",e);
    return e;
}
#endif

#if FUSE_DUMMY_CODE
NvError NvDdkFuseProgram(void)
{

}
#else
#if ENABLE_FUSE_VOLTAGE
static void EnableFuseVoltage(void)
{
    const NvOdmPeripheralConnectivity *pConnectivity = NULL;
    NvRmPmuVddRailCapabilities RailCaps;
    NvU32 i;

    pConnectivity = NvOdmPeripheralGetGuid(NV_VDD_FUSE_ODM_ID);
    if (pConnectivity != NULL)
    {
        for (i = 0; i < pConnectivity->NumAddress; i++)
        {
            // Search for the vdd rail entry
            if (pConnectivity->AddressList[i].Interface == NvOdmIoModule_Vdd)
            {
                NvRmPmuGetCapabilities((*ps_hRmDevice),
                            pConnectivity->AddressList[i].Address, &RailCaps);

                NvRmPmuSetVoltage((*ps_hRmDevice),
                            pConnectivity->AddressList[i].Address,
                            RailCaps.requestMilliVolts, NULL);
            }
        }
    }
}
#endif
/**
 * Program all fuses based on cache data changed via the NvDdkFuseSet() API.
 *
 * Caller is responsible for supplying valid fuse programming voltage prior to
 * invoking this routine.
 *
 * NOTE: All values that are not intended to be programmed must have
 *       value 0. There is no need to conditionalize this code based on which
 *       logical values were actually set through the API.
 */
NvError NvDdkFuseProgram(void)
{
    if(!s_pFuseRec)
    {
        NvOsDebugPrintf("\r\n Plz call NvDdkFuseOpen before using this API");
        return 1;
    }
#if ENABLE_FUSE_VOLTAGE
    EnableFuseVoltage();
#endif
    NvOsMutexLock(s_pFuseRec->Mutex);
    /*
     * Validate the current state.
     * If Secure mode is already burned, only reserved odm fuses are allowed to burn
     */
    if (NvDdkFuseIsOdmProductionModeFuseSet() && (!s_FuseData.ReservedOdmFlag))
    {
        NvOsDebugPrintf("\r\n NvDdkFuseProgram err - InvalidState");
        return NvError_InvalidState;
    }
    /*
     * If Disable Reg program is set, chip can not be fused, so return as
     *  "Access denied".
     */
    if (NvDdkPrivIsDisableRegProgramSet())
    {
        NvOsDebugPrintf("\r\n NvDdkFuseProgram err - AccessDenied");
        return NvError_AccessDenied;
    }

    if (s_FuseData.ProgramFlags & FLAGS_PROGRAM_ILLEGAL)
    {
        NvOsDebugPrintf("\r\n NvDdkFuseProgram err - InvalidState");
        return NvError_InvalidState;
    }

    // Map data onto a fuse array.
    MapDataToFuseArrays();
    NvDdkFuseProgramFuseArray(s_FuseArray, s_MaskArray, FUSE_WORDS,
                               NVDDK_TPROGRAM_VALUE_CORRECTED);
    NvOsMutexUnlock(s_pFuseRec->Mutex);
    // Clear reserved odm flag
    s_FuseData.ReservedOdmFlag = NV_FALSE;
    return NvError_Success;
}
#endif

#if FUSE_DUMMY_CODE
NvError NvDdkFuseVerify(void)
{}
#else
/**
 * Verify all fuses scheduled via the NvDdkFuseSet*() API's
 */

NvError NvDdkFuseVerify(void)
{
    NvError e = NvError_Success;
    if(!s_pFuseRec)
    {
        NvOsDebugPrintf("\r\n Plz call NvDdkFuseOpen before using this API");
        return 1;
    }
    NvOsMutexLock(s_pFuseRec->Mutex);
    // Map data onto a fuse array.
    MapDataToFuseArrays();
    // Check to see if the data matches the real fuses.
    e = NvDdkFuseVerifyFuseArray(s_FuseArray, s_MaskArray, FUSE_WORDS);
    NvOsMutexUnlock(s_pFuseRec->Mutex);
    if (e != NvError_Success)
    {
        e = NvError_InvalidState;
        NvOsDebugPrintf("\r\n NvDdkFuseVerify returning modified err:0x%x",e);
    }
    return e;
}
#endif
/*
 * NvDdkDisableFuseProgram API disables the fuse programming until the next
 * next system reset.
 */

void NvDdkDisableFuseProgram(void)
{
    NvU32 RegData;
    if(!s_pFuseRec)
    {
        // NV_ASSERT(0);
        NvOsDebugPrintf("\r\n Plz call NvDdkFuseOpen before using this API");
        return;
    }
    NvOsMutexLock(s_pFuseRec->Mutex);
    RegData = NV_DRF_DEF(FUSE, DISABLEREGPROGRAM, DISABLEREGPROGRAM_VAL, ENABLE);
    FUSE_NV_WRITE32(FUSE_DISABLEREGPROGRAM_0, RegData);
    NvOsMutexUnlock(s_pFuseRec->Mutex);
}

/** @} */
