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

#include "nvcommon.h"
#include "nvos.h"
#include "nvassert.h"
#include "nvrm_relocation_table.h"
#include "nvrm_hardware_access.h"
#include "nvrm_module.h"
#include "nvrm_moduleids.h"
#include "nvrm_hw_devids.h"

#define NVRM_ENABLE_PRINTF  0 // Module debug: 0=disable, 1=enable

#if (NV_DEBUG && NVRM_ENABLE_PRINTF)
#define NVRM_MODULE_PRINTF(x)   NvOsDebugPrintf x
#else
#define NVRM_MODULE_PRINTF(x)
#endif

// Relocation table unpacking macros
#define DEVICE_ID( i )          ( ( (i) & ( 0xFFFFUL << 16 ) ) >> 16 )
#define DEVICE_MAJOR_REV( i )   ( ( (i) & ( 0xFUL    << 12 ) ) >> 12 )
#define DEVICE_MINOR_REV( i )   ( ( (i) & ( 0xFUL    << 8  ) ) >> 8  )
#define DEVICE_POWER_GROUP( i ) ( ( (i) & ( 0xFUL    << 4  ) ) >> 4  )
#define DEVICE_BAR( i )         ( ( (i) & ( 0xFUL          ) ) )
#define IRQ_VALID( i )          ( (i) >> 31 )
#define IRQ_TARGET( i )         ( ( (i) & ( 0x3UL    << 29 ) ) >> 29 )
#define IRQ_INT_DEV_INDEX( i )  ( ( (i) & ( 0x1FFUL  << 20 ) ) >> 20 )
#define IRQ_DEVICE_INDEX( i )   ( ( (i) & ( 0xFFUL   << 8  ) ) >> 8  )
#define IRQ_INT_NUM(i)          ( (i) & 0xFFul )

NvRmModuleInstance s_InstanceTable[NVRM_MAX_MODULE_INSTANCES];

/**
 * Maps relocation table device ids to software module ids.
 * NVRM_DEVICE_UNKNOWN for unknown ids (will keep parsing table), 
 * or NVRM_DEVICE_ERROR if something bad happened
 * (will stop parsing the table).
 *
 * NVRM_DEVICE_UNKOWN can be used to cull the device list to save space by
 * not allocating memory for devices that won't be used.
 */
NvU32
NvRmPrivDevToModuleID(NvU32 devid)
{
    switch( devid ) {
    /* actual module with registers */
    case NVRM_DEVID_AC97:
        return NvRmModuleID_Ac97;
    case NVRM_DEVID_APB_DMA:
        return NvRmPrivModuleID_ApbDma;
    case NVRM_DEVID_APB_DMA_CH:
        return NvRmPrivModuleID_ApbDmaChannel;
    case NVRM_DEVID_ARB_PRI:
        return NvRmModuleID_ArbPriority;
    case NVRM_DEVID_ARB_SEM:
        return NvRmModuleID_ArbitrationSema;
    case NVRM_DEVID_CAR:
        return NvRmPrivModuleID_ClockAndReset;
    case NVRM_DEVID_CC:
        return NvRmPrivModuleID_CC;
    case NVRM_DEVID_CMC:
        return NvRmModuleID_CacheMemCtrl;
    case NVRM_DEVID_BSEA:
    case NVRM_DEVID_AVPBSEA:
        /* Module name changed to NVRM_DEVID_AVPBSEA in AP20 */
        return NvRmModuleID_BseA;
    case NVRM_DEVID_VDE:
        return NvRmModuleID_Vde;
    case NVRM_DEVID_CPU_INTR:
        return NvRmPrivModuleID_InterruptCpu;
    case NVRM_DEVID_DISPLAY:
        return NvRmModuleID_Display;
    case NVRM_DEVID_DSI:
        return NvRmModuleID_Dsi;
    case NVRM_DEVID_DVC:
        return NvRmModuleID_Dvc;
    case NVRM_DEVID_EIDE:
        return NvRmModuleID_Ide;
    case NVRM_DEVID_EMC:
        return NvRmPrivModuleID_ExternalMemoryController;
    case NVRM_DEVID_EPP:
        return NvRmModuleID_Epp;
    case NVRM_DEVID_EVENT:
        return NvRmModuleID_EventCtrl;
    case NVRM_DEVID_FLOW:
        return NvRmModuleID_FlowCtrl;
    case NVRM_DEVID_FUSE:
        return NvRmModuleID_Fuse;
    case NVRM_DEVID_KFUSE:
        return NvRmModuleID_KFuse;
    case NVRM_DEVID_GPIO:
        return NvRmPrivModuleID_Gpio;
    case NVRM_DEVID_GR2D:
        return NvRmModuleID_2D;
    case NVRM_DEVID_GR3D:
        return NvRmModuleID_3D;
    case NVRM_DEVID_HDMI:
        return NvRmModuleID_Hdmi;
    case NVRM_DEVID_HOST1X:
        return NvRmModuleID_GraphicsHost;
    case NVRM_DEVID_HSMMC:
        return NvRmModuleID_Hsmmc;
    case NVRM_DEVID_I2C:
        return NvRmModuleID_I2c;
    case NVRM_DEVID_I2S:
        return NvRmModuleID_I2s;
    case NVRM_DEVID_ICTLR:
        return NvRmPrivModuleID_Interrupt;
    case NVRM_DEVID_ICTLR_ARBGNT:
        return NvRmPrivModuleID_InterruptArbGnt;
    case NVRM_DEVID_ICTLR_DRQ:
        return NvRmPrivModuleID_InterruptDrq;
    case NVRM_DEVID_ISP:
        return NvRmModuleID_Isp;
    case NVRM_DEVID_KBC:
        return NvRmModuleID_Kbc;
    case NVRM_DEVID_MC:
        return NvRmPrivModuleID_MemoryController;
    case NVRM_DEVID_MIPI_HS:
        return NvRmModuleID_Mipi;
    case NVRM_DEVID_MISC:
        return NvRmModuleID_Misc;
    case NVRM_DEVID_MPE:
        return NvRmModuleID_Mpe;
    case NVRM_DEVID_MSELECT:
        return NvRmPrivModuleID_Mselect;
    case NVRM_DEVID_NANDFLASH:
        return NvRmModuleID_Nand;
    case NVRM_DEVID_PMIF:
        return NvRmModuleID_Pmif;
    case NVRM_DEVID_PWFM:
        return NvRmModuleID_Pwm;
    case NVRM_DEVID_RTC:
        return NvRmModuleID_Rtc;
    case NVRM_DEVID_SDMMC:
    case NVRM_DEVID_SDIO:
        return NvRmModuleID_Sdio;
    case NVRM_DEVID_SHR_SEM:
        return NvRmModuleID_ResourceSema;

    // Supporting only the slink controller for now, returning error for old
    // slink controller.
    case NVRM_DEVID_SLINK:
        return NvRmModuleID_Slink;
    case NVRM_DEVID_SPDIF:
        return NvRmModuleID_Spdif;
    case NVRM_DEVID_SPI:
        return NvRmModuleID_Spi;
    case NVRM_DEVID_STAT:
        return NvRmModuleID_SysStatMonitor;
    case NVRM_DEVID_SW_INTR:
        return NvRmPrivModuleID_InterruptSw;
    case NVRM_DEVID_TMR:
        return NvRmModuleID_Timer;
    case NVRM_DEVID_TMRUS:
        return NvRmModuleID_TimerUs;
    case NVRM_DEVID_TVO:
        return NvRmModuleID_Tvo;
    case NVRM_DEVID_TWC:
        return NvRmModuleID_Twc;
    case NVRM_DEVID_UART:
        return NvRmModuleID_Uart;
    case NVRM_DEVID_UCQ:
        return NvRmModuleID_Ucq;
    case NVRM_DEVID_AVPUCQ:
        return NvRmModuleID_AvpUcq;
    case NVRM_DEVID_USB:
        return NvRmModuleID_Usb2Otg;
    case NVRM_DEVID_VCP:
        return NvRmModuleID_Vcp;
    case NVRM_DEVID_VECTOR:
        // FIXME: does this make sense?
        return NvRmModuleID_ExceptionVector;
    case NVRM_DEVID_VFIR:
        return NvRmModuleID_Vfir;
    case NVRM_DEVID_VI:
        return NvRmModuleID_Vi;
    case NVRM_DEVID_XIO:
        return NvRmModuleID_Xio;
    case NVRM_DEVID_UPTAG:
        return NvRmPrivModuleID_ProcId;
    case NVRM_DEVID_AHB_ARBC:
        return NvRmPrivModuleID_Ahb_Arb_Ctrl;

    /* memory (internal, external, etc - no registers) */
    case NVRM_DEVID_EMEM:
        return NvRmPrivModuleID_ExternalMemory;

    case NVRM_DEVID_IMEM:
        return NvRmPrivModuleID_InternalMemory;

    case NVRM_DEVID_TCRAM:
        return NvRmPrivModuleID_Tcram;

    case NVRM_DEVID_IRAM:
        return NvRmPrivModuleID_Iram;

    case NVRM_DEVID_GART:
        return NvRmPrivModuleID_Gart;

    case NVRM_DEVID_EXIO:
        return NvRmPrivModuleID_Mio_Exio;

    case NVRM_DEVID_PMU_EXT:
         return NvRmPrivModuleID_PmuExt;
            
    case NVRM_DEVID_NOR:
        return NvRmModuleID_Nor;

    case NVRM_DEVID_CSI:
        return NvRmModuleID_Csi;

    case NVRM_DEVID_OWR:
        return NvRmModuleID_OneWire;
    case NVRM_DEVID_SNOR:
        return NvRmModuleID_SyncNor;

    case NVRM_DEVID_ARM_PERIPH:
        return NvRmPrivModuleID_ArmPerif;

    case NVRM_DEVID_ARM_ICTLR:
        return NvRmPrivModuleID_ArmInterruptctrl;

    case NVRM_DEVID_PCIE:
        return NvRmPrivModuleID_Pcie;

    case NVRM_DEVID_AHB_EMEM:
        return NvRmPrivModuleID_AhbRemap;

    case NVRM_DEVID_ARM_PL310:
        return NvRmPrivModuleID_Pl310;

    /* unknown or don't care */
    default:
        return NVRM_DEVICE_UNKNOWN;
    }
}

static NvError
NvRmPrivParseDevices( const NvU32 *table,
    NvRmModuleInstance **instances,
    NvRmModuleInstance **instanceLast,
    NvRmModule *modules )
{
    NvError ret = NvSuccess;
    NvU32 info;
    NvU32 devid;
    NvS32 index;
    NvU32 count;
    NvU8 devidx;
    NvRmModuleInstance *inst = 0;
    NvU32 modid;
    NvU32 start;
    NvU32 length;
    NvS32 tmp_index;
    NvU32 tmp_devid;
    NvU8 tmp_devidx;
    NvBool skip;

    /* The first 32 bits of the table is the table version number */
    index = 1;

    /* count the total number of devices and allocate space for them.
     * for each device, check if the device has already been found (multiply
     * instantiated), if this is the first device instance, then find
     * all of the rest of the devices to compact all devices together.
     *
     * after the module instances have been compacted, count the number of
     * unique non-memory device ids and setup the module index table.
     *
     * only count devices that the NvRmPrivDevToModuleID function returns a
     * valid module id for (don't count memory or unknown devices).  it is ok
     * for NvRmPrivDevToModuleID to return unknown for devices it doesn't care
     * about.
     */
    count = 0;
    while( NV_READ32( &table[index] ) )
    {
        info = NV_READ32( &table[index] );
        devid = DEVICE_ID( info );
        modid = NvRmPrivDevToModuleID(devid);
        if( modid != NVRM_DEVICE_UNKNOWN )
        {
            count++;
        }

        if( modid == NVRM_DEVICE_ERROR )
        {
            NV_ASSERT( !"relocation table parsing error" );
            goto fail;
        }

        index += 3;
    }

    /* reset index to the first device */
    index = 1;

    /* Use Instance array */
    inst = s_InstanceTable;
    /* Make sure we are not over stepping the array boundaries */
    NV_ASSERT(NVRM_MAX_MODULE_INSTANCES >= (count + 1));

    *instances = inst;
    devidx = (NvU8)-1;      /* -1 is the invalid/unavailable indicator */

    /* pass over the relocation table again to fill in the instance table */
    while( NV_READ32( &table[index] ) )
    {
        skip = NV_FALSE;
        info = NV_READ32( &table[index++] );
        start = NV_READ32( &table[index++] );
        length = NV_READ32( &table[index++] );
        devidx++;

        devid = DEVICE_ID( info );
        modid = NvRmPrivDevToModuleID( devid );

        if( modid == NVRM_DEVICE_UNKNOWN )
        {
            /* keep going */
            NVRM_MODULE_PRINTF(( "[Unknwn] devidx: %d devid: %d start: 0x%x "
                "length: 0x%x\n", devidx, devid, start, length ));
            continue;
        }
        else if( modid == NVRM_DEVICE_ERROR )
        {
            NVRM_MODULE_PRINTF(( "[Error] devidx: %d devid: %d start: 0x%x "
                "length: 0x%x\n", devidx, devid, start, length ));
            NV_ASSERT( !"relocation table parsing failure" );
            goto fail;
        }

        /* search backwards to detect an already found instance */
        tmp_index = index - 6;
        while( tmp_index > 1 )
        {
            tmp_devid = DEVICE_ID( NV_READ32( &table[tmp_index] ) );
            if( tmp_devid == devid )
            {
                skip = NV_TRUE;
                break;
            }

            tmp_index -= 3;
        }

        /* already found this instance, continue to the next device */
        if( skip )
        {
            continue;
        }

        /* scan forward to find all instances of this devid */
        tmp_devid = devid;
        tmp_index = index;
        tmp_devidx = devidx;
        for( ;; )
        {
            if( tmp_devid == devid )
            {
                inst->PhysAddr = start;
                inst->Length = length;
                inst->MajorVersion = (NvU8)DEVICE_MAJOR_REV(info);
                inst->MinorVersion = (NvU8)DEVICE_MINOR_REV(info);
                inst->DevPowerGroup = (NvU8)DEVICE_POWER_GROUP(info);
                inst->Bar = (NvU8)DEVICE_BAR(info);
                inst->VirtAddr = 0;
                inst->DeviceId = devid;
                inst->DevIdx = tmp_devidx;

                NVRM_MODULE_PRINTF(( "[Device] devidx: %d devid: %d "
                    "addr: 0x%x length: 0x%x major: %d minor: %d\n",
                    tmp_devidx, devid, start, length,
                    inst->MajorVersion, inst->MinorVersion ));

                NV_ASSERT( tmp_devidx < (NvU8)-1 );
                    /* (NvU8)-1 is the indicator for invalid/unavailable
                       instance and safeguard against overflow on idx too. */

                inst++;
            }

            if( !NV_READ32( &table[tmp_index] ) )
            {
                break;
            }

            info = NV_READ32( &table[tmp_index++] );
            start = NV_READ32( &table[tmp_index++] );
            length = NV_READ32( &table[tmp_index++] );
            tmp_devidx++;

            tmp_devid = DEVICE_ID( info );
        }
    }

    /* zero out the last instance */
    NvOsMemset( inst, 0, sizeof(*inst) );
    *instanceLast = inst;
    inst = *instances;

    /* setup the module index table:
     * walk to instances - setup the module table.
     */
    index = 0;
    devid = inst->DeviceId;
    while( inst->DeviceId ) // null terminated instance array
    {
        if( devid == inst->DeviceId )
        {
            modid = NvRmPrivDevToModuleID(devid);
            if(( modid != NVRM_DEVICE_UNKNOWN ) && 
               ( modid != NVRM_DEVICE_ERROR ))
            {
                modules[modid].Index = (NvU16)index;
            }
            else
            {
                NV_ASSERT( !"relocation table parsing error" );
            }
        }

        /* skip the rest of the instances */
        do
        {
            inst++;
            index++;
        } while( inst->DeviceId == devid );

        devid = inst->DeviceId;
    }

    return NvSuccess;

fail:
    *instances = 0;
    return ret;
}

static NvRmModuleInstance *
NvRmPrivGetInstance( NvRmModuleInstance *inst, NvU8 devidx )
{
    while( inst->DeviceId )
    {
        if( inst->DevIdx == devidx )
        {
            return inst;
        }

        inst++;
    }

    return 0;
}

static NvError
NvRmPrivParseIrqs( const NvU32 *table, NvRmIrqMap *irqs,
    NvRmModuleInstance *instances )
{
    NvU32 info;
    NvU32 minor;
    NvU32 index;
    NvU16 ctlr_index[NVRM_MAX_INTERRUPT_CTLRS];
    NvU16 ctlr;
    NvU16 irq;
    NvU8 devidx = 0;
    NvU16 intridx = 0;
    NvRmModuleInstance *inst;
    NvRmModuleIrqMap *map;
    NvU8 Valid;
    NvU16 IntDevIndex;
    NvU16 IntNum;
    NvU32 devid;
    NvU32 Affinity;
    NvU32 Processor = NV_IS_AVP ? 2 : 1;
    NvU8 irqBase = 0;

    for (ctlr = 0; ctlr < NVRM_MAX_INTERRUPT_CTLRS; ++ctlr)
    {
        ctlr_index[ctlr] = 0xFFFF;
    }

    /* skip version */
    index = 1;

    /* find the interrupt controllers */
    while( NV_READ32( &table[index] ) )
    {
        info  = NV_READ32( &table[index] );
        devid = DEVICE_ID( info );

        // Main interrupt controller?
        if (devid == NVRM_DEVID_ICTLR)
        {
            // The main interrupt controller instances are identified
            // by their minor revision number.
            minor = DEVICE_MINOR_REV(info);
            NV_ASSERT(minor < NVRM_MAX_MAIN_INTR_CTLRS);
            NV_ASSERT(ctlr_index[minor] == 0xFFFF);
            ctlr_index[minor] = devidx;
        }
        else if (devid == NVRM_DEVID_ARM_ICTLR) 
        {
            /* If the falcon interrupt controller is present then the IRQs
             * start from 32. Falcon controller cannot be used when running on
             * QT using
             * EMUTRANS. 
             */
#if !(NVCPU_IS_X86 || NV_IS_AVP)
            irqBase = 32;
#endif
        }

        index += 3;
        devidx++;
    }

    /* skip the null separator between the device and irq table */
    index++;

    while( NV_READ32( &table[index] ) )
    {
        info = NV_READ32( &table[index++] );

        // Extract the interrupt entry fields.
        Valid       = (NvU8)IRQ_VALID(info);
        IntDevIndex = (NvU16)IRQ_INT_DEV_INDEX(info);
        devidx      = (NvU8)IRQ_DEVICE_INDEX(info);
        IntNum      = (NvU16)IRQ_INT_NUM(info);
        Affinity    = IRQ_TARGET(info);

        NV_ASSERT(IntNum < NVRM_IRQS_PER_INTR_CTLR);

        // Retrieve the device instance to which this interrupt belongs.
        inst = NvRmPrivGetInstance( instances, devidx );
        if( inst == NULL )
        {
            /* interrupt pointing to something that's unknown, skip it. */
            continue;
        }

        // Locate the interrupt controller that manages this interrupt
        irq = NVRM_IRQ_INVALID;
        for( ctlr = 0; ctlr < NVRM_MAX_INTERRUPT_CTLRS; ctlr++ )
        {
            if (ctlr_index[ctlr] != 0xFFFF)
            {
                if( ctlr_index[ctlr] == IntDevIndex )
                {
                    irq = irqBase + ( ctlr * 32 ) + IntNum;
                    break;
                }
            }
        }

        /* Don't take care of interrupts routed to interrupts other than main
         * interrupt controller.
         * */
        if (irq == NVRM_IRQ_INVALID) continue;

        map = inst->IrqMap;
        if( map == 0 )
        {
            // Allocate a new device IRQ map.
            NV_ASSERT(irqs->DeviceCount < NVRM_MAX_IRQ_DEVICES);
            map = &irqs->DeviceIrq[ irqs->DeviceCount++ ];
            inst->IrqMap = map;
        }
        else
        {
            NV_ASSERT(map->IrqCount < NVRM_MAX_DEVICE_IRQS);
        }

        if (Valid)
        {
            /* HW bug 562244 - Affinities are wrong in the relocation table. */
            if (irq == 29 + irqBase) 
            {
                Affinity = 1;
            }
            if (irq == 28 + irqBase)
            {
                Affinity = 2;
            }


            // Consider this IRQ for mapping only if the IRQ's processor
            // affinity matches this processor or if the IRQ has no affinity.
            if ((Affinity == 0)
            ||  (Affinity == Processor))
            {
                // Add the IRQ to the device's IRQ list
                map->Irq[ map->IrqCount++ ] = irq;
                NVRM_MODULE_PRINTF(("[Interrupt %d] Device Index: %d "
                    "IntCtlr: %d IntNum: %d Irq: %d Affinity: %d\n",
                    intridx, devidx, ctlr, IntNum, irq, Affinity));
            }
            else
            {
                // This IRQ belongs to the other processor.
                NVRM_MODULE_PRINTF(("[Interrupt %d] Device Index: %d "
                    "IntCtlr: %d IntNum: %d Irq: %d Affinity: %d mapped on "
                    "other processor\n",
                    intridx, devidx, ctlr, IntNum, irq, Affinity));
            }
        }
        else
        {
            // Add placeholder to the device's IRQ list
            map->Irq[ map->IrqCount++ ] = NVRM_IRQ_INVALID;
        }
        intridx++;
    }

    NVRM_MODULE_PRINTF(("\n"));

    return NvSuccess;
}

NvError
NvRmPrivRelocationTableParse(
    const NvU32 *table,
    NvRmModuleInstance **instances, NvRmModuleInstance **instanceLast,
    NvRmModule *modules, NvRmIrqMap *irqs )
{
    NvError err;

    /* only know how to parse version 1 tables */
    NV_ASSERT( NV_READ32( &table[0] ) == 1 );

    NVRM_MODULE_PRINTF(( "Relocation Table:\n" ));

    /* parse the devices */
    err = NvRmPrivParseDevices( table, instances, instanceLast,
        modules );
    if( err != NvSuccess )
    {
        return err;
    }

    /* parse the irq entries */
    err = NvRmPrivParseIrqs( table, irqs, *instances );
    if( err != NvSuccess )
    {
        return err;
    }

    return NvSuccess;
}
