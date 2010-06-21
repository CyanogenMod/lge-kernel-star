/*
 * Copyright (c) 2007-2010 NVIDIA Corporation.
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

#if NV_IS_AVP
#define NV_DEF_RMC_TRACE                0   // NO TRACING FOR AVP
#endif

#include "nvcommon.h"
#include "nvassert.h"
#include "nvos.h"
#include "nvrm_hwintf.h"
#include "nvrm_module.h"
#include "nvrm_module_private.h"
#include "nvrm_moduleids.h"
#include "nvrm_chipid.h"
#include "nvrm_drf.h"
#include "nvrm_power.h"
#include "nvrm_structure.h"
#include "ap15/ap15rm_private.h"
#include "ap20/ap20rm_misc_private.h"
#include "ap15/arclk_rst.h"

#define NVRM_ENABLE_PRINTF          0  // Module debug: 0=disable, 1=enable

#if (NV_DEBUG && NVRM_ENABLE_PRINTF)
#define NVRM_MODULE_PRINTF(x)   NvOsDebugPrintf x
#else
#define NVRM_MODULE_PRINTF(x)
#endif


// FIXME: this is hacked
// Handled thru RmTransportMessaging to CPU
void ap15Rm_AvpModuleReset(NvRmDeviceHandle hDevice, NvRmModuleID ModuleId);

void AP15ModuleReset(NvRmDeviceHandle hDevice, NvRmModuleID ModuleId, NvBool hold);
void AP20ModuleReset(NvRmDeviceHandle hDevice, NvRmModuleID ModuleId, NvBool hold);

void
NvRmModuleResetWithHold(NvRmDeviceHandle hDevice, NvRmModuleID ModuleId, NvBool hold)
{
#if !NV_IS_AVP
    if ( hDevice->ChipId.Id == 0x15 || hDevice->ChipId.Id == 0x16)
    {
        AP15ModuleReset(hDevice, ModuleId, hold);
    } else
    {
        AP20ModuleReset(hDevice, ModuleId, hold);
    }
#else
    ap15Rm_AvpModuleReset(hDevice, ModuleId);
#endif
}

void NvRmModuleReset(NvRmDeviceHandle hDevice, NvRmModuleID ModuleId)
{
    NvRmModuleResetWithHold(hDevice, ModuleId, NV_FALSE);
}

NvError
NvRmModuleGetCapabilities(
    NvRmDeviceHandle hDevice,
    NvRmModuleID ModuleId,
    NvRmModuleCapability *pCaps,
    NvU32 NumCaps,
    void **Capability )
{
    NvError err;
    NvRmModuleCapability *cap;
    NvRmModuleInstance *inst;
    NvBool found = NV_FALSE;
    void *ret = 0;
    NvU32 i;
    NvRmChipId *id;

    err = NvRmPrivGetModuleInstance( hDevice, ModuleId, &inst );
    if( err != NvSuccess )
    {
        return err;
    }

    id = NvRmPrivGetChipId( hDevice );

    for( i = 0; i < NumCaps; i++ )
    {
        cap = &pCaps[i];
        ret = cap->Capability;

        /* HW bug 574527 - version numbers for USB are wrong in the AP20 relocation table.*/
        if (NVRM_MODULE_ID_MODULE( ModuleId ) == NvRmModuleID_Usb2Otg)
        {
            if (id->Id == 0x20)
            {
                NvU32 instance = NVRM_MODULE_ID_INSTANCE( ModuleId );

                if (((cap->MinorVersion == 5) && (instance == 0)) ||
                    ((cap->MinorVersion == 6) && (instance == 1))||
                    ((cap->MinorVersion == 7) && (instance == 2)))
                {
                    found = NV_TRUE;
                    break;
                }
                continue;
            }
        }

        if( ( cap->MajorVersion == inst->MajorVersion ) &&
            ( cap->MinorVersion == inst->MinorVersion ) )
        {
            // FIMXE: ignoring eco levels for now (properly)
            // HACK:  except display with AP16 A03/sim/emul (bug 515059)
            if ( NVRM_MODULE_ID_MODULE( ModuleId ) == NvRmModuleID_Display )
            {
                if (id->Id == 0x16 &&
                        (id->Minor == 0x3 || id->Major == 0))
                {
                    // Only accepts cap of (1,2,3) for this chipId
                    if (cap->MajorVersion == 1 && cap->MinorVersion == 2 &&
                            cap->EcoLevel == 0x3)
                    {
                        found = NV_TRUE;
                        break;
                    }
                    // else not found and continue on to next cap
                    continue;
                }
            }

            found = NV_TRUE;
            break;
        }
    }

    if( !found )
    {
        NV_ASSERT(!"Could not find matching version of module in table");
        *Capability = 0;
        return NvError_NotSupported;
    }

    *Capability = ret;
    return NvSuccess;
}

NvError
NvRmPrivFindModule( NvRmDeviceHandle hDevice, NvU32 Address,
    NvRmPrivModuleID* pModuleId )
{
    NvU32 i;
    NvU32 devid;
    NvRmModuleTable *tbl;
    NvRmModuleInstance *inst;
    NvRmModule *mod;
    NvU16 num;

    NV_ASSERT((pModuleId != NULL) && (hDevice != NULL));

    tbl = NvRmPrivGetModuleTable( hDevice );

    mod = tbl->Modules;
    for( i = 0; i < NvRmPrivModuleID_Num; i++ )
    {
        if( mod[i].Index == NVRM_MODULE_INVALID )
        {
            continue;
        }

        // For each instance of the module id ...
        inst = tbl->ModInst + mod[i].Index;
        devid = inst->DeviceId;
        num = 0;

        while( devid == inst->DeviceId )
        {
            // Is the device address matches
            if( inst->PhysAddr == Address )
            {
                // Return the module id and instance information.
                *pModuleId = (NvRmPrivModuleID)NVRM_MODULE_ID(i, num);
                return NvSuccess;
            }

            inst++;
            num++;
        }
    }

    // we are here implies no matching module was found.
    return NvError_ModuleNotPresent;
}

NvError
NvRmQueryChipUniqueId(NvRmDeviceHandle hDevHandle, NvU32 IdSize, void* pId)
{
    NvU32   Size = IdSize;    // Size of the output buffer
    NvError err = NvError_NotSupported;

    NV_ASSERT(hDevHandle);
    NV_ASSERT(pId);
   // Update the intended size
    IdSize = sizeof(NvU64);
    if ((pId == NULL)||(Size < IdSize))
    {
        return NvError_BadParameter;
    }

    NvOsMemset(pId, 0, Size);
    switch (hDevHandle->ChipId.Id)
    {
    case 0x15:
    case 0x16:      // ap16 should use same space of ap15 for fuse info.
        err = NvRmPrivAp15ChipUniqueId(hDevHandle,pId);
        break;
    case 0x20:
        err = NvRmPrivAp20ChipUniqueId(hDevHandle,pId);
        break;
    default:
        NV_ASSERT(!"Unsupported chip ID");
        return err;
    }
    return err;
}

NvError NvRmGetRandomBytes(
    NvRmDeviceHandle hRm,
    NvU32 NumBytes,
    void *pBytes)
{
    NvU8 *Array = (NvU8 *)pBytes;
    NvU16 Val;

    if (!hRm || !pBytes)
        return NvError_BadParameter;

    while (NumBytes)
    {
        Val = (NvU16) NV_REGR(hRm, NvRmPrivModuleID_ClockAndReset, 0,
                              CLK_RST_CONTROLLER_PLL_LFSR_0);
        *Array++ = (Val & 0xff);
        Val>>=8;
        NumBytes--;
        if (NumBytes)
        {
            *Array++ = (Val & 0xff);
            NumBytes--;
        }
    }

    return NvSuccess;
}

NvError
NvRmPrivModuleInit( NvRmModuleTable *mod_table, NvU32 *reloc_table )
{
    NvError err;
    NvU32 i;

    /* invalidate the module table */
    for( i = 0; i < NvRmPrivModuleID_Num; i++ )
    {
        mod_table->Modules[i].Index = NVRM_MODULE_INVALID;
    }

    /* clear the irq map */
    NvOsMemset( &mod_table->IrqMap, 0, sizeof(mod_table->IrqMap) );

    err = NvRmPrivRelocationTableParse( reloc_table,
        &mod_table->ModInst, &mod_table->LastModInst,
        mod_table->Modules, &mod_table->IrqMap );
    if( err != NvSuccess )
    {
        NV_ASSERT( !"NvRmPrivModuleInit failed" );
        return err;
    }

    NV_ASSERT( mod_table->LastModInst);
    NV_ASSERT( mod_table->ModInst );

    mod_table->NumModuleInstances = mod_table->LastModInst -
        mod_table->ModInst;

    return NvSuccess;
}

void
NvRmPrivModuleDeinit( NvRmModuleTable *mod_table )
{
}

NvError
NvRmPrivGetModuleInstance( NvRmDeviceHandle hDevice, NvRmModuleID ModuleId,
    NvRmModuleInstance **out )
{
    NvRmModuleTable *tbl;
    NvRmModule *module;             // Pointer to module table
    NvRmModuleInstance *inst;       // Pointer to device instance
    NvU32 DeviceId;                 // Hardware device id
    NvU32 Module;
    NvU32 Instance;
    NvU32 Bar;
    NvU32 idx;

    *out = NULL;

    NV_ASSERT( hDevice );

    tbl = NvRmPrivGetModuleTable( hDevice );

    Module   = NVRM_MODULE_ID_MODULE( ModuleId );
    Instance = NVRM_MODULE_ID_INSTANCE( ModuleId );
    Bar      = NVRM_MODULE_ID_BAR( ModuleId );
    NV_ASSERT( (NvU32)Module < (NvU32)NvRmPrivModuleID_Num );

    // Get a pointer to the first instance of this module id type.
    module = tbl->Modules;

    // Check whether the index is valid or not.
    if (module[Module].Index == NVRM_MODULE_INVALID)
    {
        return NvError_NotSupported;
    }

    inst = tbl->ModInst + module[Module].Index;

    // Get its device id.
    DeviceId = inst->DeviceId;

    // find the right instance and bar
    idx = 0;
    while( inst->DeviceId == DeviceId )
    {
        if( idx == Instance && inst->Bar == Bar )
        {
            break;
        }
        if( inst->Bar == 0 )
        {
            idx++;
        }

        inst++;
    }

    // Is this a valid instance and is it of the same hardware type?
    if( (inst >= tbl->LastModInst) || (DeviceId != inst->DeviceId) )
    {
        // Invalid instance.
        return NvError_BadValue;
    }

    *out = inst;

    // Check if instance is still valid and not bonded out.
    // Still returning inst structure.
    if ( (NvU8)-1 == inst->DevIdx )
        return NvError_NotSupported;

    return NvSuccess;
}

void
NvRmModuleGetBaseAddress( NvRmDeviceHandle hDevice,
    NvRmModuleID ModuleId, NvRmPhysAddr* pBaseAddress,
    NvU32* pSize )
{
    NvRmModuleInstance *inst;

    NV_ASSERT_SUCCESS(
        NvRmPrivGetModuleInstance(hDevice, ModuleId, &inst)
    );

    if (pBaseAddress)
        *pBaseAddress = inst ? inst->PhysAddr : 0;
    if (pSize)
        *pSize = inst ? inst->Length : 0;
}

NvU32
NvRmModuleGetNumInstances(
    NvRmDeviceHandle hDevice,
    NvRmModuleID Module)
{
    NvError e;
    NvRmModuleInstance *inst;
    NvU32 n;
    NvU32 id;

    e = NvRmPrivGetModuleInstance( hDevice, NVRM_MODULE_ID(Module, 0), &inst);
    if( e != NvSuccess )
    {
        return 0;
    }

    n = 0;
    id = inst->DeviceId;
    while( inst->DeviceId == id )
    {
        if( inst->Bar == 0 )
        {
            n++;
        }

        inst++;
    }

    return n;
}

NvError
NvRmModuleGetModuleInfo(
    NvRmDeviceHandle    hDevice,
    NvRmModuleID        module,
    NvU32 *             pNum,
    NvRmModuleInfo      *pModuleInfo )
{
    NvU32   instance = 0;
    NvU32   i = 0;

    if ( NULL == pNum )
        return NvError_BadParameter;

    // if !pModuleInfo, returns total number of entries
    while ( (NULL == pModuleInfo) || (i < *pNum) )
    {
        NvRmModuleInstance *inst;
        NvError e = NvRmPrivGetModuleInstance(
            hDevice, NVRM_MODULE_ID(module, instance), &inst);
        if (e != NvSuccess)
        {
            if ( !(inst && ((NvU8)-1 == inst->DevIdx)) )
                break;

             /* else if a module instance not avail (bonded out), continue
              *  looking for next instance
              */
        }
        else
        {
            if ( pModuleInfo )
            {
                pModuleInfo->Instance = instance;
                pModuleInfo->Bar = inst->Bar;
                pModuleInfo->BaseAddress = inst->PhysAddr;
                pModuleInfo->Length = inst->Length;
                pModuleInfo++;
            }

            i++;
        }

        instance++;
    }

    *pNum = i;

    return NvSuccess;
}
