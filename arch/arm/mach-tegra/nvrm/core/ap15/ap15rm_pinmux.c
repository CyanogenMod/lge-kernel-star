/*
 * Copyright (c) 2009-2010 NVIDIA Corporation.
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

#include "nvcommon.h"
#include "nvrm_pinmux.h"
#include "nvrm_drf.h"
#include "nvassert.h"
#include "nvrm_hwintf.h"
#include "ap15/ap15rm_private.h"
#include "ap15/arapb_misc.h"
#include "nvrm_pinmux_utils.h"
#include "ap15/ap15rm_pinmux_utils.h"
#include "nvodm_query_pinmux.h"

/*  FindConfigStart searches through an array of configuration data to find the
 *  starting position of a particular configuration in a module instance array.
 *  The stop position is programmable, so that sub-routines can be placed after
 *  the last valid true configuration */

const NvU32* NvRmPrivAp15FindConfigStart(
    const NvU32* Instance,
    NvU32 Config,
    NvU32 EndMarker)
{
    NvU32 Cnt = 0;
    while ((Cnt < Config) && (*Instance!=EndMarker))
    {
        switch (NV_DRF_VAL(MUX, ENTRY, STATE, *Instance))
        {
            case PinMuxConfig_BranchLink:
            case PinMuxConfig_OpcodeExtend:
                if (*Instance==CONFIGEND())
                    Cnt++;
                Instance++;
                break;
            default:
                Instance += NVRM_PINMUX_SET_OPCODE_SIZE;
                break;
        }
    }

    /* Ugly postfix.  In modules with bonafide subroutines, the last
     * configuration CONFIGEND() will be followed by a MODULEDONE()
     * token, with the first Set/Unset/Branch of the subroutine
     * following that.  To avoid leaving the "PC" pointing to a
     * MODULEDONE() in the case where the first subroutine should be
     * executed, fudge the "PC" up by one, to point to the subroutine. */
    if (EndMarker==SUBROUTINESDONE() && *Instance==MODULEDONE())
        Instance++;

    if (*Instance==EndMarker)
        Instance = NULL;

    return Instance;
}

/*  NvRmSetPadTristates will increment/decrement the reference count for
 *  each pad group's global tristate value for each "ConfigSet" command in
 *  a pad group configuration, and update the register as needed */
void NvRmPrivAp15SetPadTristates(
    NvRmDeviceHandle hDevice,
    const NvU32* Module,
    NvU32 Config,
    NvBool EnableTristate)
{
    int StackDepth = 0;
    const NvU32 *Instance = NULL;
    const NvU32 *ReturnStack[MAX_NESTING_DEPTH+1];

    /* The re-multiplexing configuration is stored in program 0,
     * along with the reset config. */
    if (Config==NVODM_QUERY_PINMAP_MULTIPLEXED)
        Config = 0;

    Instance = NvRmPrivAp15FindConfigStart(Module, Config, MODULEDONE());
    /* The first stack return entry is NULL, so that when a ConfigEnd is
     * encountered in the "main" configuration program, we pop off a NULL
     * pointer, which causes the configuration loop to terminate. */
    ReturnStack[0] = NULL;

    /*  This loop iterates over all of the pad groups that need to be updated,
     *  and updates the reference count for each appropriately.  */

    NvOsMutexLock(hDevice->mutex);

    while (Instance)
    {
        switch (NV_DRF_VAL(MUX,ENTRY, STATE, *Instance))
        {
            case PinMuxConfig_OpcodeExtend:
                /* Pop the most recent return address off of the return stack
                 * (which will be NULL if no values have been pushed onto the
                 * stack) */
                if (NV_DRF_VAL(MUX,ENTRY, OPCODE_EXTENSION,
                               *Instance)==PinMuxOpcode_ConfigEnd)
                {
                    Instance = ReturnStack[StackDepth--];
                }
                /* ModuleDone & SubroutinesDone should never be encountered
                 * during execution, for properly-formatted tables. */
                else
                {
                    NV_ASSERT(0 && "Logical entry in table!\n");
                }
                break;
            case PinMuxConfig_BranchLink:
                /*  Push the next instruction onto the return stack if nesting space
                    is available, and jump to the target. */
                NV_ASSERT(StackDepth<MAX_NESTING_DEPTH);
                ReturnStack[++StackDepth] = Instance+1;
                Instance = NvRmPrivAp15FindConfigStart(Module,
                               NV_DRF_VAL(MUX,ENTRY,BRANCH_ADDRESS,*Instance),
                               SUBROUTINESDONE());
                NV_ASSERT(Instance && "Invalid branch configuration in table!\n");
                break;
            case PinMuxConfig_Set:
            {
                NvS16 SkipUpdate;
                NvU32 TsOffs = NV_DRF_VAL(MUX,ENTRY, TS_OFFSET, *Instance);
                NvU32 TsShift = NV_DRF_VAL(MUX,ENTRY, TS_SHIFT, *Instance);

/*  abuse pre/post-increment, to ensure that skipUpdate is 0 when the
 *  register needs to be programmed (i.e., enabling and previous value was 0,
 *  or disabling and new value is 0).
 */
                if (EnableTristate)
#if (SKIP_TRISTATE_REFCNT == 0)
                    SkipUpdate =  --hDevice->TristateRefCount[TsOffs*32 + TsShift];
                else
                    SkipUpdate = hDevice->TristateRefCount[TsOffs*32 + TsShift]++;
#else
                    SkipUpdate = 1;
                else
                    SkipUpdate = 0;
#endif

#if (SKIP_TRISTATE_REFCNT == 0)
                if (SkipUpdate < 0)
                {
                    hDevice->TristateRefCount[TsOffs*32 + TsShift] = 0;
                    NV_DEBUG_PRINTF(("(%s:%s) Negative reference count detected "
                                     "on TRISTATE_REG_%c_0, bit %u\n",
                        __FILE__, __LINE__, ('A'+(TsOffs)), TsShift));
                    //NV_ASSERT(SkipUpdate>=0);
                }
#endif

                if (!SkipUpdate)
                {
                    NvU32 Curr = NV_REGR(hDevice,
                        NvRmModuleID_Misc, 0,
                        APB_MISC_PP_TRISTATE_REG_A_0 + 4*TsOffs);
                    Curr &= ~(1<<TsShift);
#if (SKIP_TRISTATE_REFCNT == 0)
                    Curr |= (EnableTristate?1:0)<<TsShift;
#endif
                    NV_REGW(hDevice, NvRmModuleID_Misc, 0,
                        APB_MISC_PP_TRISTATE_REG_A_0 + 4*TsOffs, Curr);

#if NVRM_PINMUX_DEBUG_FLAG
                    NV_DEBUG_PRINTF(("Setting TRISTATE_REG_%s to %s\n",
                        (const char*)Instance[2],
                        (EnableTristate)?"TRISTATE" : "NORMAL"));
#endif
                }
            }
            /* fall through.
             * The "Unset" configurations are not applicable to tristate
             * configuration, so skip over them. */
            case PinMuxConfig_Unset:
                Instance += NVRM_PINMUX_SET_OPCODE_SIZE;
                break;
        }
    }
    NvOsMutexUnlock(hDevice->mutex);
}

/*  NvRmSetPinMuxCtl will apply new pin mux configurations to the pin mux
 *  control registers.  */
void NvRmPrivAp15SetPinMuxCtl(
    NvRmDeviceHandle hDevice,
    const NvU32* Module,
    NvU32 Config)
{
    NvU32 MuxCtlOffset, MuxCtlShift, MuxCtlMask, MuxCtlSet, MuxCtlUnset;
    const NvU32 *ReturnStack[MAX_NESTING_DEPTH+1];
    const NvU32 *Instance;
    int StackDepth = 0;
    NvU32 Curr;

    ReturnStack[0] = NULL;
    Instance = Module;

    NvOsMutexLock(hDevice->mutex);

    /* The re-multiplexing configuration is stored in program 0,
     * along with the reset config. */
    if (Config==NVODM_QUERY_PINMAP_MULTIPLEXED)
        Config = 0;

    Instance = NvRmPrivAp15FindConfigStart(Module, Config, MODULEDONE());

    //  Apply the new configuration, setting / unsetting as appropriate
    while (Instance)
    {
        switch (NV_DRF_VAL(MUX,ENTRY, STATE, *Instance))
        {
            case PinMuxConfig_OpcodeExtend:
                if (NV_DRF_VAL(MUX,ENTRY, OPCODE_EXTENSION,
                               *Instance)==PinMuxOpcode_ConfigEnd)
                {
                    Instance = ReturnStack[StackDepth--];
                }
                else
                {
                    NV_ASSERT(0 && "Logical entry in table!\n");
                }
                break;
            case PinMuxConfig_BranchLink:
                NV_ASSERT(StackDepth<MAX_NESTING_DEPTH);
                ReturnStack[++StackDepth] = Instance+1;
                Instance = NvRmPrivAp15FindConfigStart(Module,
                               NV_DRF_VAL(MUX,ENTRY,BRANCH_ADDRESS,*Instance),
                               SUBROUTINESDONE());
                NV_ASSERT(Instance && "Invalid branch configuration in table!\n");
                break;
            default:
            {
                MuxCtlOffset = NV_DRF_VAL(MUX,ENTRY, MUX_CTL_OFFSET, *Instance);
                MuxCtlShift = NV_DRF_VAL(MUX,ENTRY, MUX_CTL_SHIFT, *Instance);
                MuxCtlUnset = NV_DRF_VAL(MUX,ENTRY, MUX_CTL_UNSET, *Instance);
                MuxCtlSet = NV_DRF_VAL(MUX,ENTRY, MUX_CTL_SET, *Instance);
                MuxCtlMask = NV_DRF_VAL(MUX, ENTRY, MUX_CTL_MASK, *Instance);

                Curr = NV_REGR(hDevice, NvRmModuleID_Misc, 0,
                    APB_MISC_PP_PIN_MUX_CTL_A_0 + 4*MuxCtlOffset);

                if (NV_DRF_VAL(MUX,ENTRY,STATE,*Instance)==PinMuxConfig_Set)
                {
                    Curr &= ~(MuxCtlMask<<MuxCtlShift);
                    Curr |= (MuxCtlSet<<MuxCtlShift);
#if NVRM_PINMUX_DEBUG_FLAG
                    NV_DEBUG_PRINTF(("Configuring PINMUX_CTL_%s\n",
                                     (const char *)Instance[1]));
#endif

                }
                else if (((Curr>>MuxCtlShift)&MuxCtlMask)==MuxCtlUnset)
                {
                    NV_ASSERT(NV_DRF_VAL(MUX,ENTRY,STATE,
                                         *Instance)==PinMuxConfig_Unset);
                    Curr &= ~(MuxCtlMask<<MuxCtlShift);
                    Curr |= (MuxCtlSet<<MuxCtlShift);
#if NVRM_PINMUX_DEBUG_FLAG
                    NV_DEBUG_PRINTF(("Unconfiguring PINMUX_CTL_%s\n",
                                     (const char *)Instance[1]));
#endif
                }

                NV_REGW(hDevice, NvRmModuleID_Misc, 0,
                    APB_MISC_PP_PIN_MUX_CTL_A_0 + 4*MuxCtlOffset, Curr);
                Instance += NVRM_PINMUX_SET_OPCODE_SIZE;
                break;
            }
        }
    }
    NvOsMutexUnlock(hDevice->mutex);
}

void NvRmPrivAp15InitTrisateRefCount(NvRmDeviceHandle hDevice)
{
    NvU32 i, j, curr;

    NvOsMutexLock(hDevice->mutex);
    NvOsMemset(hDevice->TristateRefCount, 0,
        sizeof(hDevice->TristateRefCount));

    for (i=0; i<=((APB_MISC_PP_TRISTATE_REG_D_0-
                   APB_MISC_PP_TRISTATE_REG_A_0)>>2); i++)
    {
        curr = NV_REGR(hDevice, NvRmModuleID_Misc, 0,
                       APB_MISC_PP_TRISTATE_REG_A_0 + 4*i);
        // swap from 0=normal, 1=tristate to 0=tristate, 1=normal
        curr = ~curr;
        for (j=0; curr; j++, curr>>=1)
        {
            /* the oppositely-named tristate reference count keeps track
             * of the number of active users of each pad group, and
             * enables tristate when the count reaches zero. */
            hDevice->TristateRefCount[i*32 + j] = (NvS16)(curr & 0x1);
        }
    }
    NvOsMutexUnlock(hDevice->mutex);
}

void NvRmAp15SetDefaultTristate(NvRmDeviceHandle hDevice)
{
    return;
}

void NvRmPrivAp15SetGpioTristate(
    NvRmDeviceHandle hDevice,
    NvU32 Port,
    NvU32 Pin,
    NvBool EnableTristate)
{
    NvU32 Mapping = 0;
    NvS16 SkipUpdate;
    NvBool ret = NV_FALSE;

    NV_ASSERT(hDevice);

    switch (hDevice->ChipId.Id)
    {
        case 0x15:
        case 0x16:
            ret = NvRmAp15GetPinGroupForGpio(hDevice, Port, Pin, &Mapping);
            break;
        default:
            NV_ASSERT(!"Chip ID not supported");
            return;
    }

    if (ret)
    {
        NvU32 TsOffs  = NV_DRF_VAL(MUX, GPIOMAP, TS_OFFSET, Mapping);
        NvU32 TsShift = NV_DRF_VAL(MUX, GPIOMAP, TS_SHIFT, Mapping);

        NvOsMutexLock(hDevice->mutex);

        if (EnableTristate)
#if (SKIP_TRISTATE_REFCNT == 0)
            SkipUpdate = --hDevice->TristateRefCount[TsOffs*32 + TsShift];
        else
            SkipUpdate = hDevice->TristateRefCount[TsOffs*32 + TsShift]++;
#else
            SkipUpdate = 1;
        else
            SkipUpdate = 0;
#endif

#if (SKIP_TRISTATE_REFCNT == 0)
        if (SkipUpdate < 0)
        {
            hDevice->TristateRefCount[TsOffs*32 + TsShift] = 0;
            NV_DEBUG_PRINTF(("(%s:%s) Negative reference count detected on "
                "TRISTATE_REG_%c_0, bit %u\n", __FILE__, __LINE__,
                ('A'+(TsOffs)), TsShift));
            //NV_ASSERT(SkipUpdate>=0);
        }
#endif

        if (!SkipUpdate)
        {
            NvU32 Curr = NV_REGR(hDevice,
                NvRmModuleID_Misc, 0,
                APB_MISC_PP_TRISTATE_REG_A_0 + 4*TsOffs);
            Curr &= ~(1<<TsShift);
#if (SKIP_TRISTATE_REFCNT == 0)
            Curr |= (EnableTristate?1:0)<<TsShift;
#endif
            NV_REGW(hDevice, NvRmModuleID_Misc, 0,
                APB_MISC_PP_TRISTATE_REG_A_0 + 4*TsOffs, Curr);
        }

        NvOsMutexUnlock(hDevice->mutex);
    }
}

