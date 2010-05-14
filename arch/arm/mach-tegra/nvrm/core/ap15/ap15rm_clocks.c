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
#include "nvassert.h"
#include "nvrm_clocks.h"
#include "nvrm_hwintf.h"
#include "nvrm_module.h"
#include "nvrm_drf.h"
#include "ap15/aremc.h"
#include "ap15/arclk_rst.h"
#include "ap15/arapbpm.h"
#include "ap16/arapb_misc.h"
#include "ap15rm_clocks.h"
#include "ap15rm_private.h"



/*****************************************************************************/

static void NvRmPrivWaitUS(
    NvRmDeviceHandle hDevice,
    NvU32 usec)
{
    NvU32 t, start;

    start = NV_REGR(hDevice, NvRmModuleID_TimerUs, 0, 0);
    for (;;)
    {
        t = NV_REGR(hDevice, NvRmModuleID_TimerUs, 0, 0);
        if ( ((NvU32)(t - start)) >= usec )
            break;
    }
}

#define CLOCK_ENABLE( rm, offset, field, EnableState ) \
    do { \
        regaddr = (CLK_RST_CONTROLLER_##offset##_0); \
        NvOsMutexLock((rm)->CarMutex); \
        reg = NV_REGR((rm), NvRmPrivModuleID_ClockAndReset, 0, regaddr); \
        reg = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, offset, field, EnableState, reg); \
        NV_REGW((rm), NvRmPrivModuleID_ClockAndReset, 0, regaddr, reg); \
        NvOsMutexUnlock((rm)->CarMutex); \
    } while( 0 )

/*****************************************************************************/
void
Ap15EnableModuleClock(
    NvRmDeviceHandle hDevice,
    NvRmModuleID ModuleId,
    ModuleClockState ClockState)
{
    // Extract module and instance from composite module id.
    NvU32 Module   = NVRM_MODULE_ID_MODULE( ModuleId );
    NvU32 Instance = NVRM_MODULE_ID_INSTANCE( ModuleId );
    NvU32 reg;
    NvU32 regaddr;

    if (ClockState == ModuleClockState_Enable)
    {
        NvRmPrivConfigureClockSource(hDevice, ModuleId, NV_TRUE);
    }

    switch ( Module ) {
        case NvRmModuleID_CacheMemCtrl:
            NV_ASSERT( Instance < 2 );
            if( Instance == 0 )
            {
                CLOCK_ENABLE( hDevice, CLK_OUT_ENB_L, CLK_ENB_CACHE1, ClockState );
            }
            else if( Instance == 1 )
            {
                CLOCK_ENABLE( hDevice, CLK_OUT_ENB_L, CLK_ENB_CACHE2, ClockState );
            }
            break;
        case NvRmModuleID_Vcp:
            NV_ASSERT( Instance == 0 );
            CLOCK_ENABLE( hDevice, CLK_OUT_ENB_L, CLK_ENB_VCP, ClockState );
            break;
        case NvRmModuleID_GraphicsHost:
            // FIXME: should this be allowed?
            NV_ASSERT( Instance == 0 );
            CLOCK_ENABLE( hDevice, CLK_OUT_ENB_L, CLK_ENB_HOST1X, ClockState );
            break;
        case NvRmModuleID_Display:
            NV_ASSERT( Instance < 2 );
            if( Instance == 0 )
            {
                CLOCK_ENABLE( hDevice, CLK_OUT_ENB_L, CLK_ENB_DISP1, ClockState );
            }
            else if( Instance == 1 )
            {
                CLOCK_ENABLE( hDevice, CLK_OUT_ENB_L, CLK_ENB_DISP2, ClockState );
            }
            break;
        case NvRmModuleID_Ide:
            NV_ASSERT( Instance == 0 );
            CLOCK_ENABLE( hDevice, CLK_OUT_ENB_L, CLK_ENB_IDE, ClockState );
            break;
        case NvRmModuleID_3D:
            NV_ASSERT( Instance == 0 );
            CLOCK_ENABLE( hDevice, CLK_OUT_ENB_L, CLK_ENB_3D, ClockState );
            break;
        case NvRmModuleID_Isp:
            NV_ASSERT( Instance == 0 );
            CLOCK_ENABLE( hDevice, CLK_OUT_ENB_L, CLK_ENB_ISP, ClockState );
            break;
        case NvRmModuleID_Usb2Otg:
            NV_ASSERT( Instance < 2 );
            if ((hDevice->ChipId.Id == 0x16) && (ClockState == NV_FALSE))
            {
                NvU32 RegVal = 0;
                // On AP16 USB clock source is shared for both USB controllers
                // Disabling the main clock source will disable both controllers
                // when disabling the clock make sure that both controllers are disabled.
                RegVal = NV_REGR(hDevice, NvRmModuleID_Misc, 0, APB_MISC_PP_MISC_USB_CLK_RST_CTL_0);

                if (!(NV_DRF_VAL(APB_MISC_PP, MISC_USB_CLK_RST_CTL, MISC_USB_CE, RegVal)) &&
                    !(NV_DRF_VAL(APB_MISC_PP, MISC_USB_CLK_RST_CTL, MISC_USB2_CE, RegVal)) )
                {
                    /// Disable USBD clock for both the instances 0 and 1
                    CLOCK_ENABLE( hDevice, CLK_OUT_ENB_L, CLK_ENB_USBD, ClockState );
                }
            }
            else
            {
                /// Enable/Disable USBD clock 
                CLOCK_ENABLE( hDevice, CLK_OUT_ENB_L, CLK_ENB_USBD, ClockState );
            }
            break;
        case NvRmModuleID_2D:
            NV_ASSERT( Instance == 0 );
            CLOCK_ENABLE( hDevice, CLK_OUT_ENB_L, CLK_ENB_2D, ClockState );
            break;
        case NvRmModuleID_Epp:
            NV_ASSERT( Instance == 0 );
            CLOCK_ENABLE( hDevice, CLK_OUT_ENB_L, CLK_ENB_EPP, ClockState );
            break;
        case NvRmModuleID_Vi:
            NV_ASSERT( Instance == 0 );
            CLOCK_ENABLE( hDevice, CLK_OUT_ENB_L, CLK_ENB_VI, ClockState );
            break;
        case NvRmModuleID_I2s:
            NV_ASSERT( Instance < 2 );
            if( Instance == 0 )
            {
                CLOCK_ENABLE( hDevice, CLK_OUT_ENB_L, CLK_ENB_I2S1, ClockState );
            }
            else if( Instance == 1 )
            {
                CLOCK_ENABLE( hDevice, CLK_OUT_ENB_L, CLK_ENB_I2S2, ClockState );
            }
            break;
        case NvRmModuleID_Hsmmc:
            NV_ASSERT( Instance == 0 );
            CLOCK_ENABLE( hDevice, CLK_OUT_ENB_L, CLK_ENB_HSMMC, ClockState );
            break;
        case NvRmModuleID_Twc:
            NV_ASSERT( Instance == 0 );
            CLOCK_ENABLE( hDevice, CLK_OUT_ENB_L, CLK_ENB_TWC, ClockState );
            break;
        case NvRmModuleID_Pwm:
            NV_ASSERT( Instance == 0 );
            CLOCK_ENABLE( hDevice, CLK_OUT_ENB_L, CLK_ENB_PWM, ClockState );
            break;
        case NvRmModuleID_Sdio:
            NV_ASSERT( Instance < 2 );
            if( Instance == 0 )
            {
                CLOCK_ENABLE( hDevice, CLK_OUT_ENB_L, CLK_ENB_SDIO1, ClockState );
            }
            else if( Instance == 1 )
            {
                CLOCK_ENABLE( hDevice, CLK_OUT_ENB_L, CLK_ENB_SDIO2, ClockState );
            }
            break;
        case NvRmModuleID_Spdif:
            NV_ASSERT( Instance < 1 );
            CLOCK_ENABLE( hDevice, CLK_OUT_ENB_L, CLK_ENB_SPDIF, ClockState );
            break;
        case NvRmModuleID_Nand:
            NV_ASSERT( Instance == 0 );
            CLOCK_ENABLE( hDevice, CLK_OUT_ENB_L, CLK_ENB_NDFLASH, ClockState );
            break;
        case NvRmModuleID_I2c:
            NV_ASSERT( Instance < 2 );
            if( Instance == 0 )
            {
                CLOCK_ENABLE( hDevice, CLK_OUT_ENB_L, CLK_ENB_I2C1, ClockState );
            }
            else if( Instance == 1 )
            {
                CLOCK_ENABLE( hDevice, CLK_OUT_ENB_H, CLK_ENB_I2C2, ClockState );
            }
            break;
        case NvRmPrivModuleID_Gpio:
            NV_ASSERT( Instance == 0 );
            CLOCK_ENABLE( hDevice, CLK_OUT_ENB_L, CLK_ENB_GPIO, ClockState );
            break;
        case NvRmModuleID_Uart:
            NV_ASSERT( Instance < 3 );
            if( Instance == 0 )
            {
                CLOCK_ENABLE( hDevice, CLK_OUT_ENB_L, CLK_ENB_UARTA, ClockState );
            }
            else if( Instance == 1 )
            {
                CLOCK_ENABLE( hDevice, CLK_OUT_ENB_L, CLK_ENB_UARTB, ClockState );
            }
            else if ( Instance == 2)
            {
                CLOCK_ENABLE( hDevice, CLK_OUT_ENB_H, CLK_ENB_UARTC, ClockState );
            }
            break;
        case NvRmModuleID_Vfir:
            // Same as UARTB
            NV_ASSERT( Instance == 0 );
            CLOCK_ENABLE( hDevice, CLK_OUT_ENB_L, CLK_ENB_UARTB, ClockState );
            break;
        case NvRmModuleID_Ac97:
            NV_ASSERT( Instance == 0 );
            CLOCK_ENABLE( hDevice, CLK_OUT_ENB_L, CLK_ENB_AC97, ClockState );
            break;
        case NvRmModuleID_Rtc:
            NV_ASSERT( Instance == 0 );
            CLOCK_ENABLE( hDevice, CLK_OUT_ENB_L, CLK_ENB_RTC, ClockState );
            break;
        case NvRmModuleID_Timer:
            NV_ASSERT( Instance == 0 );
            CLOCK_ENABLE( hDevice, CLK_OUT_ENB_L, CLK_ENB_TMR, ClockState );
            break;
        case NvRmModuleID_BseA:
            NV_ASSERT( Instance == 0 );
            CLOCK_ENABLE( hDevice, CLK_OUT_ENB_H, CLK_ENB_BSEA, ClockState );
            break;
        case NvRmModuleID_Vde:
            NV_ASSERT( Instance == 0 );
            CLOCK_ENABLE( hDevice, CLK_OUT_ENB_H, CLK_ENB_VDE, ClockState );
            CLOCK_ENABLE( hDevice, CLK_OUT_ENB_H, CLK_ENB_BSEV, ClockState );
            break;
        case NvRmModuleID_Mpe:
            NV_ASSERT( Instance == 0 );
            CLOCK_ENABLE( hDevice, CLK_OUT_ENB_H, CLK_ENB_MPE, ClockState );
            break;
        case NvRmModuleID_Tvo:
            NV_ASSERT( Instance == 0 );
            CLOCK_ENABLE( hDevice, CLK_OUT_ENB_H, CLK_ENB_TVO, ClockState );
            CLOCK_ENABLE( hDevice, CLK_OUT_ENB_H, CLK_ENB_TVDAC, ClockState );
            break;
        case NvRmModuleID_Csi:
            NV_ASSERT( Instance == 0 );
            CLOCK_ENABLE( hDevice, CLK_OUT_ENB_H, CLK_ENB_CSI, ClockState );
            break;
        case NvRmModuleID_Hdmi:
            NV_ASSERT( Instance == 0 );
            CLOCK_ENABLE( hDevice, CLK_OUT_ENB_H, CLK_ENB_HDMI, ClockState );
            break;
        case NvRmModuleID_Mipi:
            NV_ASSERT( Instance == 0 );
            CLOCK_ENABLE( hDevice, CLK_OUT_ENB_H, CLK_ENB_MIPI, ClockState );
            break;
        case NvRmModuleID_Dsi:
            NV_ASSERT( Instance == 0 );
            CLOCK_ENABLE( hDevice, CLK_OUT_ENB_H, CLK_ENB_DSI, ClockState );
            break;
        case NvRmModuleID_Xio:
            NV_ASSERT( Instance == 0 );
            CLOCK_ENABLE( hDevice, CLK_OUT_ENB_H, CLK_ENB_XIO, ClockState );
            break;
        case NvRmModuleID_Spi:
            NV_ASSERT( Instance == 0 );
            CLOCK_ENABLE( hDevice, CLK_OUT_ENB_H, CLK_ENB_SPI1, ClockState );
            break;
        case NvRmModuleID_Fuse:
            NV_ASSERT( Instance == 0 );
            CLOCK_ENABLE( hDevice, CLK_OUT_ENB_H, CLK_ENB_FUSE, ClockState );
            break;
        case NvRmModuleID_Slink:
            // Supporting only the slink controller.
            NV_ASSERT( Instance < 3 );
            if( Instance == 0 )
            {
                CLOCK_ENABLE( hDevice, CLK_OUT_ENB_H, CLK_ENB_SBC1, ClockState );
            }
            else if( Instance == 1 )
            {
                CLOCK_ENABLE( hDevice, CLK_OUT_ENB_H, CLK_ENB_SBC2, ClockState );
            }
            else if ( Instance == 2)
            {
                CLOCK_ENABLE( hDevice, CLK_OUT_ENB_H, CLK_ENB_SBC3, ClockState );
            }
            break;
        case NvRmModuleID_Dvc:
            NV_ASSERT( Instance == 0 );
            CLOCK_ENABLE( hDevice, CLK_OUT_ENB_H, CLK_ENB_DVC_I2C, ClockState );
            break;
        case NvRmModuleID_Pmif:
            NV_ASSERT( Instance == 0 );
            // PMC clock must not be disabled
            if (ClockState == ModuleClockState_Enable)
                CLOCK_ENABLE( hDevice, CLK_OUT_ENB_H, CLK_ENB_PMC, ClockState );
            break;
        case NvRmModuleID_SysStatMonitor:
            NV_ASSERT( Instance == 0 );
            CLOCK_ENABLE( hDevice, CLK_OUT_ENB_H, CLK_ENB_STAT_MON, ClockState );
            break;
        case NvRmModuleID_Kbc:
            NV_ASSERT( Instance == 0 );
            CLOCK_ENABLE( hDevice, CLK_OUT_ENB_H, CLK_ENB_KBC, ClockState );
            break;
        case NvRmPrivModuleID_ApbDma:
            NV_ASSERT( Instance == 0 );
            CLOCK_ENABLE( hDevice, CLK_OUT_ENB_H, CLK_ENB_APBDMA, ClockState );
            break;
        case NvRmPrivModuleID_MemoryController:
            // FIXME: should this be allowed?
            NV_ASSERT( Instance == 0 );
            CLOCK_ENABLE( hDevice, CLK_OUT_ENB_H, CLK_ENB_MEM, ClockState );
            break;
        case NvRmPrivModuleID_ExternalMemoryController:
            // FIXME: should this be allowed?
            NV_ASSERT( Instance == 0 );
            CLOCK_ENABLE( hDevice, CLK_OUT_ENB_H, CLK_ENB_EMC, ClockState );
            CLOCK_ENABLE( hDevice, CLK_SOURCE_EMC, EMC_2X_CLK_ENB, ClockState );
            CLOCK_ENABLE( hDevice, CLK_SOURCE_EMC, EMC_1X_CLK_ENB, ClockState );
            break;
        case NvRmModuleID_Cpu:
            // FIXME: should this be allowed?
            NV_ASSERT( Instance == 0 );
            CLOCK_ENABLE( hDevice, CLK_OUT_ENB_L, CLK_ENB_CPU, ClockState );
            break;
        default:
            NV_ASSERT(!" Unknown NvRmModuleID passed to Ap15EnableModuleClock(). ");
    }

    if (ClockState == ModuleClockState_Disable)
    {
        NvRmPrivConfigureClockSource(hDevice, ModuleId, NV_FALSE);
    }
}

void
Ap15EnableTvDacClock(
    NvRmDeviceHandle hDevice,
    ModuleClockState ClockState)
{
    NvU32 reg;
    NvU32 regaddr;

    CLOCK_ENABLE( hDevice, CLK_OUT_ENB_H, CLK_ENB_TVDAC, ClockState );
}

/*****************************************************************************/

    // Note that VDE has different reset sequence requirement
    // FIMXE: NV blocks - hot reset issues
    #define RESET( rm, offset, field, delay ) \
        do { \
             regaddr = (CLK_RST_CONTROLLER_##offset##_0); \
            NvOsMutexLock((rm)->CarMutex); \
            reg = NV_REGR((rm), NvRmPrivModuleID_ClockAndReset, 0, regaddr); \
            reg = NV_FLD_SET_DRF_NUM( \
                CLK_RST_CONTROLLER, offset, field, 1, reg); \
            NV_REGW((rm), NvRmPrivModuleID_ClockAndReset, 0, regaddr, reg); \
            if (Hold) \
            {\
                NvOsMutexUnlock((rm)->CarMutex); \
                break; \
            }\
            NvRmPrivWaitUS( (rm), (delay) ); \
            reg = NV_FLD_SET_DRF_NUM( \
                CLK_RST_CONTROLLER, offset, field, 0, reg); \
            NV_REGW((rm), NvRmPrivModuleID_ClockAndReset, 0, regaddr, reg); \
            NvOsMutexUnlock((rm)->CarMutex); \
        } while( 0 )

// KBC reset is available in the pmc control register.
static void RESET_KBC(NvRmDeviceHandle rm, NvU32 delay, NvBool Hold)
{
    NvU32 reg;
    NvU32 regaddr;

    regaddr = (APBDEV_PMC_CNTRL_0);
    NvOsMutexLock((rm)->CarMutex);
    reg = NV_REGR((rm), NvRmModuleID_Pmif, 0, regaddr);
    reg = NV_FLD_SET_DRF_DEF(APBDEV_PMC, CNTRL, KBC_RST, ENABLE, reg);
    NV_REGW((rm), NvRmModuleID_Pmif, 0, regaddr, reg);
    if (Hold)
    {
        NvOsMutexUnlock((rm)->CarMutex);
        return;
    }
    NvRmPrivWaitUS( (rm), (delay) );
    reg = NV_FLD_SET_DRF_DEF(APBDEV_PMC, CNTRL, KBC_RST, DISABLE, reg);
    NV_REGW((rm), NvRmModuleID_Pmif, 0, regaddr, reg);
    NvOsMutexUnlock((rm)->CarMutex);
}
        

// Use PMC control to reset the entire SoC. Just wait forever after reset is
// issued - h/w would auto-clear it and restart SoC
static void RESET_SOC(NvRmDeviceHandle rm)
{
    NvU32 reg;
    NvU32 regaddr;

    volatile NvBool b = NV_TRUE;
    regaddr = (APBDEV_PMC_CNTRL_0);
    reg = NV_REGR((rm), NvRmModuleID_Pmif, 0, regaddr);
    reg = NV_FLD_SET_DRF_DEF(APBDEV_PMC, CNTRL, MAIN_RST, ENABLE, reg);
    NV_REGW((rm), NvRmModuleID_Pmif, 0, regaddr, reg);
    while (b) { ; }
}


void AP15ModuleReset(
    NvRmDeviceHandle hDevice,
    NvRmModuleID ModuleId,
    NvBool Hold)
{
    // Extract module and instance from composite module id.
    NvU32 Module   = NVRM_MODULE_ID_MODULE( ModuleId );
    NvU32 Instance = NVRM_MODULE_ID_INSTANCE( ModuleId );
    NvU32 reg;
    NvU32 regaddr;

    switch( Module ) {
    case NvRmPrivModuleID_MemoryController:
        // FIXME: should this be allowed?
        NV_ASSERT( Instance == 0 );
        RESET( hDevice, RST_DEVICES_H, SWR_MEM_RST, NVRM_RESET_DELAY );
        break;
    case NvRmModuleID_Kbc:
        NV_ASSERT( Instance == 0 );
        RESET_KBC(hDevice, NVRM_RESET_DELAY, Hold);
        break;
    case NvRmModuleID_SysStatMonitor:
        NV_ASSERT( Instance == 0 );
        RESET( hDevice, RST_DEVICES_H, SWR_STAT_MON_RST, NVRM_RESET_DELAY );
        break;
    case NvRmModuleID_Pmif:
        NV_ASSERT( Instance == 0 );
        NV_ASSERT(!"PMC reset is not allowed, and does nothing on AP15");
        // RESET( hDevice, RST_DEVICES_H, SWR_PMC_RST, NVRM_RESET_DELAY );
        break;
    case NvRmModuleID_Fuse:
        NV_ASSERT( Instance == 0 );
        RESET( hDevice, RST_DEVICES_H, SWR_FUSE_RST, NVRM_RESET_DELAY );
        break;
    case NvRmModuleID_Slink:
        // Supporting only the slink controller.
        NV_ASSERT( Instance < 3 );
        if( Instance == 0 )
        {
            RESET( hDevice, RST_DEVICES_H, SWR_SBC1_RST, NVRM_RESET_DELAY );
        }
        else if( Instance == 1 )
        {
            RESET( hDevice, RST_DEVICES_H, SWR_SBC2_RST, NVRM_RESET_DELAY );
        }
        else if ( Instance == 2)
        {
            RESET( hDevice, RST_DEVICES_H, SWR_SBC3_RST, NVRM_RESET_DELAY );
        }
        break;
    case NvRmModuleID_Spi:
        NV_ASSERT( Instance == 0 );
        RESET( hDevice, RST_DEVICES_H, SWR_SPI1_RST, NVRM_RESET_DELAY );
        break;
    case NvRmModuleID_Xio:
        NV_ASSERT( Instance == 0 );
        RESET( hDevice, RST_DEVICES_H, SWR_XIO_RST, NVRM_RESET_DELAY );
        break;
    case NvRmModuleID_Dvc:
        NV_ASSERT( Instance == 0 );
        RESET( hDevice, RST_DEVICES_H, SWR_DVC_I2C_RST, NVRM_RESET_DELAY );
        break;
    case NvRmModuleID_Dsi:
        NV_ASSERT( Instance == 0 );
        RESET( hDevice, RST_DEVICES_H, SWR_DSI_RST, NVRM_RESET_DELAY );
        break;
    case NvRmModuleID_Tvo:
        NV_ASSERT( Instance == 0 );
        RESET( hDevice, RST_DEVICES_H, SWR_TVO_RST, NVRM_RESET_DELAY );
        RESET( hDevice, RST_DEVICES_H, SWR_TVDAC_RST, NVRM_RESET_DELAY );
        break;
    case NvRmModuleID_Mipi:
        NV_ASSERT( Instance == 0 );
        RESET( hDevice, RST_DEVICES_H, SWR_MIPI_RST, NVRM_RESET_DELAY );
        break;
    case NvRmModuleID_Hdmi:
        NV_ASSERT( Instance == 0 );
        RESET( hDevice, RST_DEVICES_H, SWR_HDMI_RST, NVRM_RESET_DELAY );
        break;
    case NvRmModuleID_Csi:
        NV_ASSERT( Instance == 0 );
        RESET( hDevice, RST_DEVICES_H, SWR_CSI_RST, NVRM_RESET_DELAY );
        break;
    case NvRmModuleID_I2c:
        NV_ASSERT( Instance < 2 );
        if( Instance == 0 )
        {
            RESET( hDevice, RST_DEVICES_L, SWR_I2C1_RST, NVRM_RESET_DELAY );
        }
        else if( Instance == 1 )
        {
            RESET( hDevice, RST_DEVICES_H, SWR_I2C2_RST, NVRM_RESET_DELAY );
        }
        break;
    case NvRmModuleID_Mpe:
        NV_ASSERT( Instance == 0 );
        RESET( hDevice, RST_DEVICES_H, SWR_MPE_RST, NVRM_RESET_DELAY );
        break;
    case NvRmModuleID_Vde:
        NV_ASSERT( Instance == 0 );
        {
            NvU32 reg;
            NvOsMutexLock(hDevice->CarMutex);
            reg = NV_REGR(hDevice, NvRmPrivModuleID_ClockAndReset, 0,
                            CLK_RST_CONTROLLER_RST_DEVICES_H_0);
            reg = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, RST_DEVICES_H,
                            SWR_VDE_RST, 1, reg);
            NV_REGW(hDevice, NvRmPrivModuleID_ClockAndReset, 0,
                            CLK_RST_CONTROLLER_RST_DEVICES_H_0, reg);
            reg = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, RST_DEVICES_H,
                            SWR_BSEV_RST, 1, reg);
            NV_REGW(hDevice, NvRmPrivModuleID_ClockAndReset, 0,
                            CLK_RST_CONTROLLER_RST_DEVICES_H_0, reg);
            if (Hold)
            {
                NvOsMutexUnlock(hDevice->CarMutex);
                break;
            }
            NvRmPrivWaitUS( hDevice, NVRM_RESET_DELAY );
            reg = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, RST_DEVICES_H,
                            SWR_BSEV_RST, 0, reg);
            NV_REGW(hDevice, NvRmPrivModuleID_ClockAndReset, 0,
                            CLK_RST_CONTROLLER_RST_DEVICES_H_0, reg);
            reg = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, RST_DEVICES_H,
                            SWR_VDE_RST, 0, reg);
            NV_REGW(hDevice, NvRmPrivModuleID_ClockAndReset, 0,
                            CLK_RST_CONTROLLER_RST_DEVICES_H_0, reg);
            NvOsMutexUnlock(hDevice->CarMutex);
        }
        break;
    case NvRmModuleID_BseA:
        NV_ASSERT( Instance == 0 );
        RESET( hDevice, RST_DEVICES_H, SWR_BSEA_RST, NVRM_RESET_DELAY );
        break;
    case NvRmModuleID_Cpu:
        // FIXME: should this be allowed?
        NV_ASSERT( Instance == 0 );
        RESET( hDevice, RST_DEVICES_L, SWR_CPU_RST, NVRM_RESET_DELAY );
        break;
    case NvRmModuleID_Avp:
        // FIXME: should this be allowed?
        NV_ASSERT( Instance == 0 );
        RESET( hDevice, RST_DEVICES_L, SWR_COP_RST, NVRM_RESET_DELAY );
        break;
    case NvRmPrivModuleID_System:
        // THIS WILL DO A FULL SYSTEM RESET
        NV_ASSERT( Instance == 0 );
        RESET_SOC(hDevice);
        break;
    case NvRmModuleID_Ac97:
        NV_ASSERT( Instance == 0 );
        RESET( hDevice, RST_DEVICES_L, SWR_AC97_RST, NVRM_RESET_DELAY );
        break;
    case NvRmModuleID_Rtc:
        NV_ASSERT( Instance == 0 );
        RESET( hDevice, RST_DEVICES_L, SWR_RTC_RST, NVRM_RESET_DELAY );
        break;
    case NvRmModuleID_Timer:
        NV_ASSERT( Instance == 0 );
        // Timer reset (which also affects microsecond timer) is not allowed
        // RESET( hDevice, RST_DEVICES_L, SWR_TMR_RST, NVRM_RESET_DELAY );
        NV_ASSERT(!"Timer reset is not allowed");
        break;
    case NvRmModuleID_Uart:
        NV_ASSERT( Instance < 3 );
        if( Instance == 0 )
        {
            RESET( hDevice, RST_DEVICES_L, SWR_UARTA_RST, NVRM_RESET_DELAY );
        }
        else if( Instance == 1 )
        {
            RESET( hDevice, RST_DEVICES_L, SWR_UARTB_RST, NVRM_RESET_DELAY );
        }
        else if ( Instance == 2)
        {
            RESET( hDevice, RST_DEVICES_H, SWR_UARTC_RST, NVRM_RESET_DELAY );
        }
        break;
    case NvRmModuleID_Vfir:
        // Same as UARTB
        NV_ASSERT( Instance == 0 );
        RESET( hDevice, RST_DEVICES_L, SWR_UARTB_RST, NVRM_RESET_DELAY );
        break;
    case NvRmModuleID_Sdio:
        NV_ASSERT( Instance < 2 );
        if( Instance == 0 )
        {
            RESET( hDevice, RST_DEVICES_L, SWR_SDIO1_RST, NVRM_RESET_DELAY );
        }
        else if( Instance == 1 )
        {
            RESET( hDevice, RST_DEVICES_L, SWR_SDIO2_RST, NVRM_RESET_DELAY );
        }
        break;
    case NvRmModuleID_Spdif:
        NV_ASSERT( Instance == 0 );
        RESET( hDevice, RST_DEVICES_L, SWR_SPDIF_RST, NVRM_RESET_DELAY );
        break;
    case NvRmModuleID_I2s:
        NV_ASSERT( Instance < 2 );
        if( Instance == 0 )
        {
            RESET( hDevice, RST_DEVICES_L, SWR_I2S1_RST, NVRM_RESET_DELAY );
        }
        else if( Instance == 1 )
        {
            RESET( hDevice, RST_DEVICES_L, SWR_I2S2_RST, NVRM_RESET_DELAY );
        }
        break;
    case NvRmModuleID_Nand:
        NV_ASSERT( Instance == 0 );
        RESET( hDevice, RST_DEVICES_L, SWR_NDFLASH_RST, NVRM_RESET_DELAY );
        break;
    case NvRmModuleID_Hsmmc:
        NV_ASSERT( Instance == 0 );
        RESET( hDevice, RST_DEVICES_L, SWR_HSMMC_RST, NVRM_RESET_DELAY );
        break;
    case NvRmModuleID_Twc:
        NV_ASSERT( Instance == 0 );
        RESET( hDevice, RST_DEVICES_L, SWR_TWC_RST, NVRM_RESET_DELAY );
        break;
    case NvRmModuleID_Pwm:
        NV_ASSERT( Instance == 0 );
        RESET( hDevice, RST_DEVICES_L, SWR_PWM_RST, NVRM_RESET_DELAY );
        break;
    case NvRmModuleID_Epp:
        NV_ASSERT( Instance == 0 );
        RESET( hDevice, RST_DEVICES_L, SWR_EPP_RST, NVRM_RESET_DELAY );
        break;
    case NvRmModuleID_Vi:
        NV_ASSERT( Instance == 0 );
        RESET( hDevice, RST_DEVICES_L, SWR_VI_RST, NVRM_RESET_DELAY );
        break;
    case NvRmModuleID_3D:
        NV_ASSERT( Instance == 0 );
        RESET( hDevice, RST_DEVICES_L, SWR_3D_RST, NVRM_RESET_DELAY );
        break;
    case NvRmModuleID_2D:
        NV_ASSERT( Instance == 0 );
        // RESET( hDevice, RST_DEVICES_L, SWR_2D_RST, NVRM_RESET_DELAY );
        // WAR for bug 364497, se also NvRmPrivAP15Reset2D()
        NV_ASSERT(!"2D reset after RM open is no longer allowed");
        break;
    case NvRmModuleID_Usb2Otg:
        {
#if !NV_OAL
            NvU32 RegVal = 0;
            NV_ASSERT( Instance < 2 );
            if (hDevice->ChipId.Id == 0x16)
            {
                RegVal = NV_REGR(hDevice, NvRmModuleID_Misc, 0, APB_MISC_PP_MISC_USB_CLK_RST_CTL_0);
                if (!(NV_DRF_VAL(APB_MISC_PP, MISC_USB_CLK_RST_CTL, MISC_USB_CE, RegVal)) &&
                    !(NV_DRF_VAL(APB_MISC_PP, MISC_USB_CLK_RST_CTL, MISC_USB2_CE, RegVal)) )
                {
                    /// Reset USBD if USB1/USB2 is not enabled already
                    RESET( hDevice, RST_DEVICES_L, SWR_USBD_RST, NVRM_RESET_DELAY );
                }
            }
            else
           {
               /// Reset USBD
               RESET( hDevice, RST_DEVICES_L, SWR_USBD_RST, NVRM_RESET_DELAY );
           }
#else
            /// Reset USBD
            RESET( hDevice, RST_DEVICES_L, SWR_USBD_RST, NVRM_RESET_DELAY );
#endif
        }
        break;
    case NvRmModuleID_Isp:
        NV_ASSERT( Instance == 0 );
        RESET( hDevice, RST_DEVICES_L, SWR_ISP_RST, NVRM_RESET_DELAY );
        break;
    case NvRmModuleID_Ide:
        NV_ASSERT( Instance == 0 );
        RESET( hDevice, RST_DEVICES_L, SWR_IDE_RST, NVRM_RESET_DELAY );
        break;
    case NvRmModuleID_Display:
        NV_ASSERT( Instance < 2 );
        if( Instance == 0 )
        {
            RESET( hDevice, RST_DEVICES_L, SWR_DISP1_RST, NVRM_RESET_DELAY );
        }
        else if( Instance == 1 )
        {
            RESET( hDevice, RST_DEVICES_L, SWR_DISP2_RST, NVRM_RESET_DELAY );
        }
        break;
    case NvRmModuleID_Vcp:
        NV_ASSERT( Instance == 0 );
        RESET( hDevice, RST_DEVICES_L, SWR_VCP_RST, NVRM_RESET_DELAY );
        break;
    case NvRmModuleID_CacheMemCtrl:
        NV_ASSERT( Instance < 2 );
        if( Instance == 0 )
        {
            RESET( hDevice, RST_DEVICES_L, SWR_CACHE1_RST, NVRM_RESET_DELAY );
        }
        else if( Instance == 1 )
        {
            RESET( hDevice, RST_DEVICES_L, SWR_CACHE2_RST, NVRM_RESET_DELAY );
        }
        break;
    case NvRmPrivModuleID_ApbDma:
        NV_ASSERT( Instance == 0 );
        RESET( hDevice, RST_DEVICES_H, SWR_APBDMA_RST, NVRM_RESET_DELAY );
        break;
    case NvRmPrivModuleID_Gpio:
        NV_ASSERT( Instance == 0 );
        RESET( hDevice, RST_DEVICES_L, SWR_GPIO_RST, NVRM_RESET_DELAY );
        break;
    case NvRmModuleID_GraphicsHost:
        // FIXME: should this be allowed?
        NV_ASSERT( Instance == 0 );
        RESET( hDevice, RST_DEVICES_L, SWR_HOST1X_RST, NVRM_RESET_DELAY );
        break;
    default:
        NV_ASSERT(!"Invalid ModuleId");
    }

    #undef RESET
}

/*****************************************************************************/

void
NvRmPrivAp15Reset2D(NvRmDeviceHandle hRmDevice)
{
#if !NV_OAL
    NvU32 reg, offset;
    /*
     * WAR for bug 364497: 2D can not be taken out of reset if VI clock is
     * running. Therefore, make sure VI clock is disabled and reset 2D here
     * during RM initialization.
     */
    Ap15EnableModuleClock(hRmDevice, NvRmModuleID_Vi, ModuleClockState_Disable);

    // Assert reset to 2D module
    offset = CLK_RST_CONTROLLER_RST_DEVICES_L_0;
    reg = NV_REGR(hRmDevice, NvRmPrivModuleID_ClockAndReset, 0, offset);
    reg = NV_FLD_SET_DRF_DEF(
        CLK_RST_CONTROLLER, RST_DEVICES_L, SWR_2D_RST, ENABLE, reg);
    NV_REGW(hRmDevice, NvRmPrivModuleID_ClockAndReset, 0, offset, reg);

    // Enable "known good" configuartion for 2D clock (PLLM divided by 2)
    offset = CLK_RST_CONTROLLER_CLK_SOURCE_G2D_0;
    NV_REGW(hRmDevice, NvRmPrivModuleID_ClockAndReset, 0, offset,
            (NV_DRF_NUM(CLK_RST_CONTROLLER, CLK_SOURCE_G2D, G2D_CLK_DIVISOR, 2) |
             NV_DRF_DEF(CLK_RST_CONTROLLER, CLK_SOURCE_G2D, G2D_CLK_SRC, PLLM_OUT0)));
    Ap15EnableModuleClock(hRmDevice, NvRmModuleID_2D, ModuleClockState_Enable);
    NvOsWaitUS(NVRM_RESET_DELAY);

    // Take 2D out of reset and disable 2D clock. Both VI and 2D clocks are
    // left disabled -it is up to the resepctive drivers to configure and enable
    // them later.
    offset = CLK_RST_CONTROLLER_RST_DEVICES_L_0;
    reg = NV_FLD_SET_DRF_DEF(
        CLK_RST_CONTROLLER, RST_DEVICES_L, SWR_2D_RST, DISABLE, reg);
    NV_REGW(hRmDevice, NvRmPrivModuleID_ClockAndReset, 0, offset, reg);
    Ap15EnableModuleClock(hRmDevice, NvRmModuleID_2D, ModuleClockState_Disable);
#endif
}

void
NvRmPrivAp15ClockConfigEx(
    NvRmDeviceHandle hDevice,
    NvRmModuleID Module,
    NvU32 ClkSourceOffset,
    NvU32 flags)
{
    NvU32 reg;

    if ((Module == NvRmModuleID_Vi) &&
        (!(flags & NvRmClockConfig_SubConfig)) &&
         (flags & (NvRmClockConfig_InternalClockForPads |
                   NvRmClockConfig_ExternalClockForPads |
                   NvRmClockConfig_InternalClockForCore |
                   NvRmClockConfig_ExternalClockForCore)))
    {
#ifdef CLK_RST_CONTROLLER_CLK_SOURCE_VI_0_PD2VI_CLK_SEL_FIELD
        reg = NV_REGR(
            hDevice, NvRmPrivModuleID_ClockAndReset, 0, ClkSourceOffset);

        /* Default is pads use External and Core use internal */
        reg = NV_FLD_SET_DRF_NUM(
            CLK_RST_CONTROLLER, CLK_SOURCE_VI, PD2VI_CLK_SEL, 0, reg);
        reg = NV_FLD_SET_DRF_NUM(
            CLK_RST_CONTROLLER, CLK_SOURCE_VI, VI_CLK_SEL, 0, reg);

        /* This is an invalid setting. */
        NV_ASSERT(!((flags & NvRmClockConfig_InternalClockForPads) &&
                    (flags & NvRmClockConfig_ExternalClockForCore)));

        if (flags & NvRmClockConfig_InternalClockForPads)
            reg = NV_FLD_SET_DRF_NUM(
                CLK_RST_CONTROLLER, CLK_SOURCE_VI, PD2VI_CLK_SEL, 1, reg);
        if (flags & NvRmClockConfig_ExternalClockForCore)
            reg = NV_FLD_SET_DRF_NUM(
                CLK_RST_CONTROLLER, CLK_SOURCE_VI, VI_CLK_SEL, 1, reg);

        NV_REGW(hDevice, NvRmPrivModuleID_ClockAndReset, 0,
                ClkSourceOffset, reg);
#endif
    }
    if (Module == NvRmModuleID_I2s)
    {
        reg = NV_REGR(
            hDevice, NvRmPrivModuleID_ClockAndReset, 0, ClkSourceOffset);

        if (flags & NvRmClockConfig_ExternalClockForCore)
        {
            // Set I2S in slave mode (field definition is the same for I2S1 and I2S2)
            reg = NV_FLD_SET_DRF_NUM(
                CLK_RST_CONTROLLER, CLK_SOURCE_I2S1, I2S1_MASTER_CLKEN, 0, reg);
            NV_REGW(hDevice, NvRmPrivModuleID_ClockAndReset, 0,
                    ClkSourceOffset, reg);
        }
        else if (flags & NvRmClockConfig_InternalClockForCore)
        {
            // Set I2S in master mode (field definition is the same for I2S1 and I2S2)
            reg = NV_FLD_SET_DRF_NUM(
                CLK_RST_CONTROLLER, CLK_SOURCE_I2S1, I2S1_MASTER_CLKEN, 1, reg);
            NV_REGW(hDevice, NvRmPrivModuleID_ClockAndReset, 0,
                    ClkSourceOffset, reg);
        }
    }
}

void NvRmPrivAp15SimPllInit(NvRmDeviceHandle hRmDevice)
{
    NvU32 RegData;

    //Enable the plls in simulation. We can just use PLLC as the template
    //and replicate across pllM and pllP since the offsets are the same.
    RegData = NV_DRF_NUM (CLK_RST_CONTROLLER, PLLC_BASE, PLLC_DIVP, 0) 
          | NV_DRF_NUM (CLK_RST_CONTROLLER, PLLC_BASE, PLLC_DIVM, 0) 
          | NV_DRF_NUM (CLK_RST_CONTROLLER, PLLC_BASE, PLLC_DIVN, 0)
          | NV_DRF_DEF (CLK_RST_CONTROLLER, PLLC_BASE, PLLC_BYPASS, DISABLE)
          | NV_DRF_DEF (CLK_RST_CONTROLLER, PLLC_BASE, PLLC_ENABLE, ENABLE) ;

    NV_REGW(hRmDevice, NvRmPrivModuleID_ClockAndReset, 0,
            CLK_RST_CONTROLLER_PLLM_BASE_0, RegData);
    NV_REGW(hRmDevice, NvRmPrivModuleID_ClockAndReset, 0,
            CLK_RST_CONTROLLER_PLLC_BASE_0, RegData);
    NV_REGW(hRmDevice, NvRmPrivModuleID_ClockAndReset, 0,
            CLK_RST_CONTROLLER_PLLP_BASE_0, RegData);
}

NvError
NvRmPrivAp15OscDoublerConfigure(
    NvRmDeviceHandle hRmDevice,
    NvRmFreqKHz OscKHz)
{
    NvU32 reg, Taps;
    NvError error = NvRmPrivGetOscDoublerTaps(hRmDevice, OscKHz, &Taps);

    if (error == NvSuccess)
    {
        // Program delay
        reg = NV_REGR(hRmDevice, NvRmPrivModuleID_ClockAndReset, 0,
                      CLK_RST_CONTROLLER_PROG_DLY_CLK_0);
        reg = NV_FLD_SET_DRF_NUM(
            CLK_RST_CONTROLLER, PROG_DLY_CLK, CLK_D_DELCLK_SEL, Taps, reg);
        NV_REGW(hRmDevice, NvRmPrivModuleID_ClockAndReset, 0,
                CLK_RST_CONTROLLER_PROG_DLY_CLK_0, reg);
        // Enable doubler
        reg = NV_REGR(hRmDevice, NvRmPrivModuleID_ClockAndReset, 0,
                      CLK_RST_CONTROLLER_MISC_CLK_ENB_0);
        reg = NV_FLD_SET_DRF_NUM(
            CLK_RST_CONTROLLER, MISC_CLK_ENB, CLK_M_DOUBLER_ENB, 1, reg);
    }
    else
    {
        // Disable doubler
        reg = NV_REGR(hRmDevice, NvRmPrivModuleID_ClockAndReset, 0,
                      CLK_RST_CONTROLLER_MISC_CLK_ENB_0);
        reg = NV_FLD_SET_DRF_NUM(
            CLK_RST_CONTROLLER, MISC_CLK_ENB, CLK_M_DOUBLER_ENB, 0, reg);
    }
    NV_REGW(hRmDevice, NvRmPrivModuleID_ClockAndReset, 0,
            CLK_RST_CONTROLLER_MISC_CLK_ENB_0, reg);
    return error;
}

/*****************************************************************************/

