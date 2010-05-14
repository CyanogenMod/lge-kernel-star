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

#include "nvos.h"
#include "nvrm_module.h"
#include "nvrm_interrupt.h"
#include "nvrm_processor.h"
#include "nvassert.h"
#include "nvrm_relocation_table.h"
#include "nvrm_chiplib.h"
#include "nvrm_hwintf.h"
#include "nvrm_drf.h"
#include "nvrm_structure.h"
#include "ap15rm_private.h"
#include "ap15/arictlr.h"

#define NVRM_ENABLE_PRINTF          0  // Module debug: 0=disable, 1=enable

#if (NV_DEBUG && NVRM_ENABLE_PRINTF)
#define NVRM_INTERRUPT_PRINTF(x)   NvOsDebugPrintf x
#else
#define NVRM_INTERRUPT_PRINTF(x)
#endif

//-----------------------------------------------------------------------------
// Register access macros
//-----------------------------------------------------------------------------

#define NV_INTR_REGR(rm,inst,reg)              NV_REGR(rm, NvRmPrivModuleID_Interrupt, inst, ICTLR_##reg##_0)
#define NV_INTR_REGW(rm,inst,reg,data)         NV_REGW(rm, NvRmPrivModuleID_Interrupt, inst, ICTLR_##reg##_0, data)

#define NV_REGA(rm, aperture, instance, offset) \
    ((volatile void*)((NvUPtr)(((rm)->ModuleTable.ModInst + (rm)->ModuleTable.Modules[(aperture)].Index + (instance))->VirtAddr) + (offset)))

#define NV_INTR_REG_READ(pIntr, reg)    NV_READ32((((NvUPtr)(pIntr)) + ICTLR_##reg##_0))

NvRmIntrDecoder gs_Ap15PrimaryDecoder =
    /* AP15 Primary interrupt controller */
    {NvRmPrivModuleID_Interrupt,
        NVRM_IRQS_PER_INTR_CTLR, 0, {0}, {0}, {0} };

NvRmIntrDecoder gs_Ap20PrimaryDecoder =
    /* AP20 Primary interrupt controller */
    {NvRmPrivModuleID_ArmPerif,
        NVRM_IRQS_PER_INTR_CTLR * 5, 0, {0}, {0}, {0} };


NvRmIntrDecoder *gs_PrimaryDecoder = &gs_Ap15PrimaryDecoder;

NvRmIntrDecoder gs_SubDecoder[] =
{
    /* Secondary interrupt controllers */

    /* Secondary interrupt controller for APB DMA */
    {NvRmPrivModuleID_ApbDma,
        NVRM_MAX_DMA_CHANNELS, 0, {0}, {0}, {0}},

    /* GPIO secondary interrupt controller */
    {NvRmPrivModuleID_Gpio,
        NVRM_IRQS_PER_GPIO_CTLR, 0, {0}, {0}, {0}},
};



static NvU16
NvRmPrivSubControllerInit(
    NvRmDeviceHandle hRmDevice,
    NvRmIntrDecoderHandle pDecoder,
    NvU16 Irq)
{
    NvRmModuleInstance  *inst;      // Pointer to the module instance
    NvU8                num;        // Number of instances/loop index
    NvU32               devid;      // Hardware device id
    NvError             e;

    NV_ASSERT( hRmDevice );

    NV_CHECK_ERROR_CLEANUP( NvRmPrivGetModuleInstance( hRmDevice, 
        pDecoder->ModuleID, &inst) );
    NV_ASSERT(inst != NULL);

    num = 0;
    devid = inst->DeviceId;
    /* Get all the instances of that sub-controller */
    while ( devid == inst->DeviceId )
    {
        NV_ASSERT( inst->IrqMap != NULL );
        NV_ASSERT( num < NVRM_MAX_INSTANCES);

        /* For modules which are sub-interrupt controllers, IRQ value in the
         * IrqMap[0] represents the IRQ of the main interrupt controller. Sub
         * IRQs for that controller can be computed from IndexMax and IndexBase
         * members */
        inst->IrqMap->IndexMax  = pDecoder->SubIrqCount;
        inst->IrqMap->IndexBase = Irq;

        pDecoder->MainIrq[num] = inst->IrqMap->Irq[0];
        pDecoder->SubIrqFirst[num] = Irq;
        pDecoder->SubIrqLast[num]  = Irq + pDecoder->SubIrqCount - 1;

        Irq += pDecoder->SubIrqCount;
        inst++;
        num++;
    }
    pDecoder->NumberOfInstances = num;

    return Irq;
fail:
    NV_ASSERT(!"Invalid ModuleID or Instance in ap15rm_interrupt");
    return 0;
}

static
NvU16 NvRmPrivMainControllerInit(NvRmDeviceHandle hRmDevice,
    NvRmIntrDecoder *pDecoder)
{
    NvRmModuleInstance  *inst;  // Pointer to the module instance
    NvU32 num = 0;
    NvU16 irq = 0;  // Primary controller will start with IRQ 0.
    NvU16 devid;
    NvError e;
    
    NV_CHECK_ERROR_CLEANUP(
        NvRmPrivGetModuleInstance( hRmDevice, pDecoder->ModuleID, &inst)
    );

    NV_ASSERT(inst != NULL);
    devid = inst->DeviceId;

    while( devid == inst->DeviceId )
    {
        pDecoder->SubIrqFirst[num] = irq;
        pDecoder->SubIrqLast[num] = irq + pDecoder->SubIrqCount - 1;
        pDecoder->MainIrq[num] = NVRM_IRQ_INVALID;

        irq += pDecoder->SubIrqCount;
        num++;
        inst++;
    }

    pDecoder->NumberOfInstances = num;
    return irq;
fail:
    NV_ASSERT(!"Invalid ModuleID or Instance in ap15rm_interrupt");
    return 0;
}

void NvRmPrivInterruptTableInit( NvRmDeviceHandle hRmDevice )
{
    NvU16 irq;
    NvU32 subDecoder;

    NV_ASSERT( hRmDevice );

    NVRM_CAP_CLEAR(hRmDevice,  NvRmCaps_HasFalconInterruptController);
    NVRM_CAP_CLEAR(hRmDevice,  NvRmCaps_Has128bitInterruptSerializer);

    // WARNING: the falcon interrupt controller is not in simulation!
    if( NvRmIsSimulation() == NV_FALSE && hRmDevice->ChipId.Id >= 0x20)
    {
        NVRM_CAP_SET(hRmDevice,  NvRmCaps_HasFalconInterruptController);

        if (hRmDevice->ChipId.Major == 0
        &&  hRmDevice->ChipId.Netlist != 0
        &&  hRmDevice->ChipId.Minor != 0 )
        {
            /* PALAU has 128-bit interrupt serializer and needs some WARs to
             * compensate for the delays in interrupt arrival at interrupt
             * controller */
            NVRM_CAP_SET(hRmDevice,  NvRmCaps_Has128bitInterruptSerializer);
        }
    }

    if (!NVRM_IS_CAP_SET(hRmDevice, NvRmCaps_HasFalconInterruptController))
    {
        gs_PrimaryDecoder = &gs_Ap15PrimaryDecoder;
    }
    else
    {
        gs_PrimaryDecoder = &gs_Ap20PrimaryDecoder;
    }

    irq = NvRmPrivMainControllerInit(hRmDevice, gs_PrimaryDecoder);

    subDecoder = NV_ARRAY_SIZE(gs_SubDecoder);
    while (subDecoder)
    {
        subDecoder --;
        irq = NvRmPrivSubControllerInit(hRmDevice,
             &(gs_SubDecoder[subDecoder]), irq);
    }

    hRmDevice->MaxIrqs = irq;
    NVRM_INTERRUPT_PRINTF(("MAX IRQs: %d\n", irq));
}

NvU32 NvRmGetIrqForLogicalInterrupt(
    NvRmDeviceHandle hRmDevice,
    NvRmModuleID ModuleID,
    NvU32 Index)
{
    NvRmModuleInstance* inst = NULL;       // Pointer to module instance
    NvRmModuleIrqMap*   pIrqMap;    // Pointer to module IRQ map
    NvU16 irq = 0;
    NvError e;
    NV_ASSERT( hRmDevice );


    NV_CHECK_ERROR_CLEANUP( NvRmPrivGetModuleInstance( hRmDevice, 
        ModuleID, &inst) );
    if ( inst == NULL || inst->IrqMap == NULL)
    {
        // NV_ASSERT(!"Illegal call\n");
        // Is this legal? Some clients like NVBL
        // is calling this API blindly as they don't know if this module
        // supports interrupt or not. I don't know if this good or bad. Why
        // would a clinet request IRQ, if they know the underying module
        // doesn'tsupport interrupts.
        return NVRM_IRQ_INVALID;
    }

    pIrqMap = inst->IrqMap;

    /* Check if this the interrupt for this module is routed to secondary
     * interrupt controller or to the main controller */
    /* FIXME rename IndexMax and IndexBase variables to SubInterruptCount and
     * SubInterruptBase */
    if (pIrqMap->IndexMax == 0)
    {
        NV_ASSERT (Index < pIrqMap->IrqCount);
        NV_ASSERT(pIrqMap->Irq[Index] != NVRM_IRQ_INVALID);

        irq = pIrqMap->Irq[Index];
    }
    /* Secondary interrupt controller */
    else
    {
        // Requesting controller's main interrupt? This is a hack used by the
        // OAL to get the main IRQ line for the sub-deocders. OAL builds a list
        // of all the main IRQs for the sub-decoders and asserts if someone
        // tries to register an interrupt handler for the main IRQ line.
        if (Index == 0xFF)
        {
            NV_ASSERT (pIrqMap->Irq[0] != NVRM_IRQ_INVALID);
            irq = pIrqMap->Irq[0];
        } else
        {
            /* Index cannot be more than the Max IRQs registered by that
             * secondary interrupt controller */
            NV_ASSERT( Index < pIrqMap->IndexMax );
            irq = pIrqMap->IndexBase + Index;
        }
    }
    return irq;
fail:
    NV_ASSERT(!"Invalid ModuleID or Instance in ap15rm_interrupt");
    return 0;
}

NvU32 NvRmGetIrqCountForLogicalInterrupt(
    NvRmDeviceHandle hRmDevice,
    NvRmModuleID ModuleID)
{
    NvRmModuleInstance *inst = NULL;
    NvError e;
    
    NV_ASSERT( hRmDevice );

    NV_CHECK_ERROR_CLEANUP( NvRmPrivGetModuleInstance( hRmDevice, 
        ModuleID, &inst) );
    if ( inst == NULL || inst->IrqMap == NULL)
    {
        // NV_ASSERT(!"Illegal call\n");
        // Is this legal? Some clients like NVBL are calling this API blindly
        // as they don't know if this module supports interrupt or not.
        // I don't know if this good or bad. Why would a clinet request IRQ,
        // if they know the underying module doesn't support interrupts.
        return 0;
    }

    return inst->IrqMap->IrqCount;
fail:
    NV_ASSERT(!"Invalid ModuleID or Instance in ap15rm_interrupt");
    return 0;
}
