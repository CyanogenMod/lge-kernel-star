/*
 * Copyright (c) 2008-2009 NVIDIA Corporation.
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
#include "nvrm_chiplib.h"
#include "nvrm_hwintf.h"
#include "nvrm_module.h"
#include "nvrm_drf.h"
#include "nvrm_pmu_private.h"
#include "ap20/arclk_rst.h"
#include "ap20/arahb_arbc.h"
#include "ap20/arapbpm.h"
#include "ap15/ap15rm_private.h"
#include "ap20rm_clocks.h"
#include "ap20/arfuse.h"


// This list requires pre-sorted info in bond-out registers order and bond-out
// register bit shift order (MSB-to-LSB).
static const NvU32 s_Ap20BondOutTable[] =
{
    // BOND_OUT_L bits
    NVRM_DEVICE_UNKNOWN,  // NV_DEVID_CPU
    NVRM_DEVICE_UNKNOWN,
    NVRM_DEVICE_UNKNOWN,
    NVRM_MODULE_ID( NvRmModuleID_Ac97,                          0 ),
    NVRM_MODULE_ID( NvRmModuleID_Rtc,                           0 ),
    NVRM_MODULE_ID( NvRmModuleID_Timer,                         0 ),
    NVRM_MODULE_ID( NvRmModuleID_Uart,                          0 ),
    NVRM_MODULE_ID( NvRmModuleID_Uart,                          1 ),
    NVRM_MODULE_ID( NvRmPrivModuleID_Gpio,                      0 ),
    NVRM_MODULE_ID( NvRmModuleID_Sdio,                          1 ),
    NVRM_MODULE_ID( NvRmModuleID_Spdif,                         0 ),
    NVRM_MODULE_ID( NvRmModuleID_I2s,                           0 ),
    NVRM_MODULE_ID( NvRmModuleID_I2c,                           0 ),
    NVRM_MODULE_ID( NvRmModuleID_Nand,                          0 ),
    NVRM_MODULE_ID( NvRmModuleID_Sdio,                          0 ),
    NVRM_MODULE_ID( NvRmModuleID_Sdio,                          3 ),
    NVRM_MODULE_ID( NvRmModuleID_Twc,                           0 ),
    NVRM_MODULE_ID( NvRmModuleID_Pwm,                           0 ),
    NVRM_MODULE_ID( NvRmModuleID_I2s,                           1 ),
    NVRM_MODULE_ID( NvRmModuleID_Epp,                           0 ),
    NVRM_MODULE_ID( NvRmModuleID_Vi,                            0 ),
    NVRM_MODULE_ID( NvRmModuleID_2D,                            0 ),
    NVRM_MODULE_ID( NvRmModuleID_Usb2Otg,                       0 ),
    NVRM_MODULE_ID( NvRmModuleID_Isp,                           0 ),
    NVRM_MODULE_ID( NvRmModuleID_3D,                            0 ),
    NVRM_MODULE_ID( NvRmModuleID_Ide,                           0 ),
    NVRM_MODULE_ID( NvRmModuleID_Display,                       1 ),
    NVRM_MODULE_ID( NvRmModuleID_Display,                       0 ),
    NVRM_MODULE_ID( NvRmModuleID_GraphicsHost,                  0 ),
    NVRM_MODULE_ID( NvRmModuleID_Vcp,                           0 ),
    NVRM_MODULE_ID( NvRmModuleID_CacheMemCtrl,                  0 ),
    NVRM_DEVICE_UNKNOWN,    // NV_DEVID_COP_CACHE

    // BOND_OUT_H bits
    NVRM_MODULE_ID( NvRmPrivModuleID_MemoryController,          0 ),
    NVRM_DEVICE_UNKNOWN,    // NV_DEVID_AHB_DMA
    NVRM_MODULE_ID( NvRmPrivModuleID_ApbDma,                    0 ),
    NVRM_DEVICE_UNKNOWN,
    NVRM_MODULE_ID( NvRmModuleID_Kbc,                           0 ),
    NVRM_MODULE_ID( NvRmModuleID_SysStatMonitor,                0 ),
    NVRM_DEVICE_UNKNOWN,    // PMC
    NVRM_MODULE_ID( NvRmModuleID_Fuse,                          0 ),
    NVRM_MODULE_ID( NvRmModuleID_KFuse,                         0 ),
    NVRM_DEVICE_UNKNOWN,    // SBC1
    NVRM_MODULE_ID( NvRmModuleID_Nor,                           0 ),
    NVRM_MODULE_ID( NvRmModuleID_Spi,                           0 ),
    NVRM_DEVICE_UNKNOWN,    // SBC2
    NVRM_MODULE_ID( NvRmModuleID_Xio,                           0 ),
    NVRM_DEVICE_UNKNOWN,    // SBC3
    NVRM_MODULE_ID( NvRmModuleID_Dvc,                           0 ),
    NVRM_MODULE_ID( NvRmModuleID_Dsi,                           0 ),
    NVRM_MODULE_ID( NvRmModuleID_Tvo,                           0 ),
    NVRM_MODULE_ID( NvRmModuleID_Mipi,                          0 ),
    NVRM_MODULE_ID( NvRmModuleID_Hdmi,                          0 ),
    NVRM_MODULE_ID( NvRmModuleID_Csi,                           0 ),
    NVRM_DEVICE_UNKNOWN,    // TVDAC
    NVRM_MODULE_ID( NvRmModuleID_I2c,                           1 ),
    NVRM_MODULE_ID( NvRmModuleID_Uart,                          2 ),
    NVRM_DEVICE_UNKNOWN,    // SPROM
    NVRM_MODULE_ID( NvRmPrivModuleID_ExternalMemoryController,  0 ),
    NVRM_MODULE_ID( NvRmModuleID_Usb2Otg,                       1 ),
    NVRM_MODULE_ID( NvRmModuleID_Usb2Otg,                       2 ),
    NVRM_MODULE_ID( NvRmModuleID_Mpe,                           0 ),
    NVRM_MODULE_ID( NvRmModuleID_Vde,                           0 ),
    NVRM_MODULE_ID( NvRmModuleID_BseA,                          0 ),
    NVRM_DEVICE_UNKNOWN,    // BSEV

    // BOND_OUT_U bits
    NVRM_DEVICE_UNKNOWN,    // SPEEDO
    NVRM_MODULE_ID( NvRmModuleID_Uart, 3),
    NVRM_MODULE_ID( NvRmModuleID_Uart, 4),
    NVRM_MODULE_ID( NvRmModuleID_I2c, 2),
    NVRM_DEVICE_UNKNOWN,    // SBC4
    NVRM_MODULE_ID( NvRmModuleID_Sdio, 2),
    NVRM_MODULE_ID( NvRmPrivModuleID_Pcie, 0),
    NVRM_MODULE_ID( NvRmModuleID_OneWire, 0),
    NVRM_DEVICE_UNKNOWN,    // AFI
    NVRM_DEVICE_UNKNOWN,    // CSTIE
    NVRM_DEVICE_UNKNOWN,
    NVRM_MODULE_ID( NvRmModuleID_AvpUcq, 0),
    NVRM_DEVICE_UNKNOWN,
    NVRM_DEVICE_UNKNOWN,
    NVRM_DEVICE_UNKNOWN,
    NVRM_DEVICE_UNKNOWN,
    NVRM_DEVICE_UNKNOWN,
    NVRM_DEVICE_UNKNOWN,
    NVRM_DEVICE_UNKNOWN,
    NVRM_DEVICE_UNKNOWN,
    NVRM_DEVICE_UNKNOWN,    // IRAMA
    NVRM_DEVICE_UNKNOWN,    // IRAMB
    NVRM_DEVICE_UNKNOWN,    // IRAMC
    NVRM_DEVICE_UNKNOWN,    // IRAMD
    NVRM_DEVICE_UNKNOWN,    // CRAM2
    NVRM_DEVICE_UNKNOWN,    // SYNC_CLOCK_DOUBLER
    NVRM_DEVICE_UNKNOWN,    // CLK_M_DOUBLER
    NVRM_DEVICE_UNKNOWN,
    NVRM_DEVICE_UNKNOWN,    // SUS_OUT
    NVRM_DEVICE_UNKNOWN,    // DEV2_OUT
    NVRM_DEVICE_UNKNOWN,    // DEV1_OUT
    NVRM_DEVICE_UNKNOWN,
};

void
NvRmPrivAp20GetBondOut( NvRmDeviceHandle hDevice,
                        const NvU32      **pTable,
                        NvU32            *bondOut )
{
    *pTable = s_Ap20BondOutTable;
    bondOut[0] = NV_REGR(hDevice, NvRmPrivModuleID_ClockAndReset, 0,
                                    CLK_RST_CONTROLLER_BOND_OUT_L_0);
    bondOut[1] = NV_REGR(hDevice, NvRmPrivModuleID_ClockAndReset, 0,
                                    CLK_RST_CONTROLLER_BOND_OUT_H_0);
    bondOut[2] = NV_REGR(hDevice, NvRmPrivModuleID_ClockAndReset, 0,
                                    CLK_RST_CONTROLLER_BOND_OUT_U_0);
}


// Top level AP20 clock enable register control macro
#define CLOCK_ENABLE( rm, offset, field, Enable) \
    do \
    {   \
        NvU32 regaddr; \
        NvU32 reg = 0; \
        if (Enable == ModuleClockState_Enable) \
        {   \
            reg = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, CLK_ENB_##offset##_SET, SET_CLK_ENB_##field, 1, reg); \
            regaddr = CLK_RST_CONTROLLER_CLK_ENB_##offset##_SET_0; \
        }   \
        else \
        {   \
            reg = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, CLK_ENB_##offset##_CLR, CLR_CLK_ENB_##field, 1, reg); \
            regaddr = CLK_RST_CONTROLLER_CLK_ENB_##offset##_CLR_0; \
        }   \
        NV_REGW((rm), NvRmPrivModuleID_ClockAndReset, 0, regaddr, reg); \
    } while (0)



void
Ap20EnableModuleClock(
    NvRmDeviceHandle hDevice,
    NvRmModuleID ModuleId,
    ModuleClockState ClockState)
{
    // Extract module and instance from composite module id.
    NvU32 Module   = NVRM_MODULE_ID_MODULE( ModuleId );
    NvU32 Instance = NVRM_MODULE_ID_INSTANCE( ModuleId );

    if (ClockState == ModuleClockState_Enable)
    {
        NvRmPrivConfigureClockSource(hDevice, ModuleId, NV_TRUE);
    }
    switch ( Module ) {
        case NvRmModuleID_CacheMemCtrl:
            NV_ASSERT( Instance < 2 );
            if( Instance == 0 )
            {
                NV_ASSERT(!"AP20 doesn't have such device");
            }
            else if( Instance == 1 )
            {
                CLOCK_ENABLE( hDevice, L, CACHE2, ClockState );
            }
            break;
        case NvRmModuleID_Vcp:
            NV_ASSERT( Instance == 0 );
            CLOCK_ENABLE( hDevice, L, VCP, ClockState );
            break;
        case NvRmModuleID_GraphicsHost:
            NV_ASSERT( Instance == 0 );
            CLOCK_ENABLE( hDevice, L, HOST1X, ClockState );
            break;
        case NvRmModuleID_Display:
            NV_ASSERT( Instance < 2 );
            if( Instance == 0 )
            {
                CLOCK_ENABLE( hDevice, L, DISP1, ClockState );
            }
            else if( Instance == 1 )
            {
                CLOCK_ENABLE( hDevice, L, DISP2, ClockState );
            }
            break;
        case NvRmModuleID_Ide:
            NV_ASSERT( Instance == 0 );
            CLOCK_ENABLE( hDevice, L, IDE, ClockState );
            break;
        case NvRmModuleID_3D:
            NV_ASSERT( Instance == 0 );
            CLOCK_ENABLE( hDevice, L, 3D, ClockState );
            break;
        case NvRmModuleID_Isp:
            NV_ASSERT( Instance == 0 );
            CLOCK_ENABLE( hDevice, L, ISP, ClockState );
            break;
        case NvRmModuleID_Usb2Otg:
            if (Instance == 0)
            {
                CLOCK_ENABLE( hDevice, L, USBD, ClockState );
            }
            else if (Instance == 1)
            {
                CLOCK_ENABLE( hDevice, H, USB2, ClockState );
            }
            else if (Instance == 2)
            {
                CLOCK_ENABLE( hDevice, H, USB3, ClockState );
            }
            else
            {
                NV_ASSERT(!"Invalid USB instance");
            }
            break;
        case NvRmModuleID_2D:
            NV_ASSERT( Instance == 0 );
            CLOCK_ENABLE( hDevice, L, 2D, ClockState );
            break;
        case NvRmModuleID_Epp:
            NV_ASSERT( Instance == 0 );
            CLOCK_ENABLE( hDevice, L, EPP, ClockState );
            break;
        case NvRmModuleID_Vi:
            NV_ASSERT( Instance == 0 );
            CLOCK_ENABLE( hDevice, L, VI, ClockState );
            break;
        case NvRmModuleID_I2s:
            if( Instance == 0 )
            {
                CLOCK_ENABLE( hDevice, L, I2S1, ClockState );
            }
            else if( Instance == 1 )
            {
                CLOCK_ENABLE( hDevice, L, I2S2, ClockState );
            } else
            {
                NV_ASSERT(!"Invalid I2S instance");
            }
            break;
        case NvRmModuleID_Twc:
            NV_ASSERT( Instance == 0 );
            CLOCK_ENABLE( hDevice, L, TWC, ClockState );
            break;
        case NvRmModuleID_Pwm:
            NV_ASSERT( Instance == 0 );
            CLOCK_ENABLE( hDevice, L, PWM, ClockState );
            break;
        case NvRmModuleID_Sdio:
            if( Instance == 0 )
            {
                CLOCK_ENABLE( hDevice, L, SDMMC1, ClockState );
            }
            else if( Instance == 1 )
            {
                CLOCK_ENABLE( hDevice, L, SDMMC2, ClockState );
            } else if (Instance == 2)
            {
                CLOCK_ENABLE( hDevice, U, SDMMC3, ClockState );
            } else if (Instance == 3)
            {
                CLOCK_ENABLE( hDevice, L, SDMMC4, ClockState );
            } else
            {
                NV_ASSERT(!"Invalid SDIO instance");
            }
            break;
        case NvRmModuleID_Spdif:
            NV_ASSERT( Instance < 1 );
            CLOCK_ENABLE( hDevice, L, SPDIF, ClockState );
            break;
        case NvRmModuleID_Nand:
            NV_ASSERT( Instance == 0 );
            CLOCK_ENABLE( hDevice, L, NDFLASH, ClockState );
            break;
        case NvRmModuleID_I2c:
            if( Instance == 0 )
            {
                CLOCK_ENABLE( hDevice, L, I2C1, ClockState );
            }
            else if( Instance == 1 )
            {
                CLOCK_ENABLE( hDevice, H, I2C2, ClockState );
            } else if (Instance == 2)
            {
                CLOCK_ENABLE( hDevice, U, I2C3, ClockState );
            } else
            {
                NV_ASSERT(!"Invalid I2C instance");
            }
            break;
        case NvRmPrivModuleID_Gpio:
            NV_ASSERT( Instance == 0 );
            CLOCK_ENABLE( hDevice, L, GPIO, ClockState );
            break;
        case NvRmModuleID_Uart:
            if( Instance == 0 )
            {
                CLOCK_ENABLE( hDevice, L, UARTA, ClockState );
            }
            else if( Instance == 1 )
            {
                CLOCK_ENABLE( hDevice, L, UARTB, ClockState );
            }
            else if ( Instance == 2)
            {
                CLOCK_ENABLE( hDevice, H, UARTC, ClockState );
            } else if (Instance == 3)
            {
                CLOCK_ENABLE( hDevice, U, UARTD, ClockState );
            } else if ( Instance == 4)
            {
                CLOCK_ENABLE( hDevice, U, UARTE, ClockState );
            } else
            {
                NV_ASSERT(!"Invlaid UART instance");
            }
            break;
        case NvRmModuleID_Vfir:
            // Same as UARTB
            NV_ASSERT( Instance == 0 );
            CLOCK_ENABLE( hDevice, L, UARTB, ClockState );
            break;
        case NvRmModuleID_Ac97:
            NV_ASSERT( Instance == 0 );
            CLOCK_ENABLE( hDevice, L, AC97, ClockState );
            break;
        case NvRmModuleID_Rtc:
            NV_ASSERT( Instance == 0 );
            CLOCK_ENABLE( hDevice, L, RTC, ClockState );
            break;
        case NvRmModuleID_Timer:
            NV_ASSERT( Instance == 0 );
            CLOCK_ENABLE( hDevice, L, TMR, ClockState );
            break;
        case NvRmModuleID_BseA:
            NV_ASSERT( Instance == 0 );
            CLOCK_ENABLE( hDevice, H, BSEA, ClockState );
            break;
        case NvRmModuleID_Vde:
            NV_ASSERT( Instance == 0 );
            CLOCK_ENABLE( hDevice, H, VDE, ClockState );
            CLOCK_ENABLE( hDevice, H, BSEV, ClockState );
            break;
        case NvRmModuleID_Mpe:
            NV_ASSERT( Instance == 0 );
            CLOCK_ENABLE( hDevice, H, MPE, ClockState );
            break;
        case NvRmModuleID_Tvo:
            NV_ASSERT( Instance == 0 );
            CLOCK_ENABLE( hDevice, H, TVO, ClockState );
            CLOCK_ENABLE( hDevice, H, TVDAC, ClockState );
            break;
        case NvRmModuleID_Csi:
            NV_ASSERT( Instance == 0 );
            CLOCK_ENABLE( hDevice, H, CSI, ClockState );
            break;
        case NvRmModuleID_Hdmi:
            NV_ASSERT( Instance == 0 );
            CLOCK_ENABLE( hDevice, H, HDMI, ClockState );
            break;
        case NvRmModuleID_Mipi:
            NV_ASSERT( Instance == 0 );
            CLOCK_ENABLE( hDevice, H, MIPI, ClockState );
            break;
        case NvRmModuleID_Dsi:
            NV_ASSERT( Instance == 0 );
            CLOCK_ENABLE( hDevice, H, DSI, ClockState );
            break;
        case NvRmModuleID_Xio:
            NV_ASSERT( Instance == 0 );
            CLOCK_ENABLE( hDevice, H, XIO, ClockState );
            break;
        case NvRmModuleID_Spi:
            NV_ASSERT( Instance == 0 );
            CLOCK_ENABLE( hDevice, H, SPI1, ClockState );
            break;
        case NvRmModuleID_Fuse:
            NV_ASSERT( Instance == 0 );
            CLOCK_ENABLE( hDevice, H, FUSE, ClockState );
            break;
        case NvRmModuleID_KFuse:
            NV_ASSERT( Instance == 0 );
            CLOCK_ENABLE( hDevice, H, KFUSE, ClockState );
            break;
        case NvRmModuleID_Slink:
            // Supporting only the slink controller.
            NV_ASSERT( Instance < 4 );
            if( Instance == 0 )
            {
                CLOCK_ENABLE( hDevice, H, SBC1, ClockState );
            }
            else if( Instance == 1 )
            {
                CLOCK_ENABLE( hDevice, H, SBC2, ClockState );
            }
            else if ( Instance == 2)
            {
                CLOCK_ENABLE( hDevice, H, SBC3, ClockState );
            }
            else if ( Instance == 3)
            {
                CLOCK_ENABLE( hDevice, U, SBC4, ClockState );
            }
            break;
        case NvRmModuleID_Dvc:
            NV_ASSERT( Instance == 0 );
            CLOCK_ENABLE( hDevice, H, DVC_I2C, ClockState );
            break;
        case NvRmModuleID_Pmif:
            NV_ASSERT( Instance == 0 );
            // PMC clock must not be disabled
            if (ClockState == ModuleClockState_Enable)
                CLOCK_ENABLE( hDevice, H, PMC, ClockState );
            break;
        case NvRmModuleID_SysStatMonitor:
            NV_ASSERT( Instance == 0 );
            CLOCK_ENABLE( hDevice, H, STAT_MON, ClockState );
            break;
        case NvRmModuleID_Kbc:
            NV_ASSERT( Instance == 0 );
            CLOCK_ENABLE( hDevice, H, KBC, ClockState );
            break;
        case NvRmPrivModuleID_ApbDma:
            NV_ASSERT( Instance == 0 );
            CLOCK_ENABLE( hDevice, H, APBDMA, ClockState );
            break;
        case NvRmPrivModuleID_MemoryController:
            // FIXME: should this be allowed?
            NV_ASSERT( Instance == 0 );
            CLOCK_ENABLE( hDevice, H, MEM, ClockState );
            break;
        case NvRmPrivModuleID_ExternalMemoryController:
            {
                // FIXME: should this be allowed?
                NvU32 reg;

                NV_ASSERT( Instance == 0 );
                CLOCK_ENABLE( hDevice, H, EMC, ClockState );

                reg = NV_REGR(hDevice, NvRmPrivModuleID_ClockAndReset, 0, CLK_RST_CONTROLLER_CLK_SOURCE_EMC_0);
                reg = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, CLK_SOURCE_EMC, EMC_2X_CLK_ENB, 1, reg);
                reg = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, CLK_SOURCE_EMC, EMC_1X_CLK_ENB, 1, reg);
                NV_REGW(hDevice, NvRmPrivModuleID_ClockAndReset, 0, CLK_RST_CONTROLLER_CLK_SOURCE_EMC_0, reg);
            }
            break;
        case NvRmModuleID_Cpu:
            // FIXME: should this be allowed?
            NV_ASSERT( Instance == 0 );
            CLOCK_ENABLE( hDevice, L, CPU, ClockState );
            break ;
        case NvRmModuleID_SyncNor:
            NV_ASSERT( Instance == 0 );
            CLOCK_ENABLE( hDevice, H, SNOR, ClockState );
            break;
        case NvRmModuleID_AvpUcq:
            NV_ASSERT( Instance == 0 );
            CLOCK_ENABLE( hDevice, U, AVPUCQ, ClockState );
            break;
        case NvRmModuleID_OneWire:
            NV_ASSERT( Instance == 0 );
            CLOCK_ENABLE( hDevice, U, OWR, ClockState );
            break;
        case NvRmPrivModuleID_Pcie:
            NV_ASSERT( Instance == 0 );
            // Keep in sync both PCIE wrapper (AFI) and core clocks
            CLOCK_ENABLE( hDevice, U, PCIE, ClockState );
            CLOCK_ENABLE( hDevice, U, AFI, ClockState );
            break;

        default:
            NV_ASSERT(!" Unknown NvRmModuleID passed to Ap20EnableModuleClock(). ");
    }

    if (ClockState == ModuleClockState_Disable)
    {
        NvRmPrivConfigureClockSource(hDevice, ModuleId, NV_FALSE);
    }
}

void
Ap20EnableTvDacClock(
    NvRmDeviceHandle hDevice,
    ModuleClockState ClockState)
{
    CLOCK_ENABLE( hDevice, H, TVDAC, ClockState );
}

/*****************************************************************************/

void
NvRmPrivAp20SetPmuIrqPolarity(
    NvRmDeviceHandle hRmDevice,
    NvOdmInterruptPolarity Polarity)
{
    NvU32 value = (Polarity == NvOdmInterruptPolarity_Low) ? 1 : 0;

    // PMU interrupt polarity is set via PMC control register. OS kernel access
    // to this register is limited to single thread env. On RM level this r-m-w
    // is protected by RmOpen() serialization.
    NvU32 reg = NV_REGR(hRmDevice, NvRmModuleID_Pmif, 0, APBDEV_PMC_CNTRL_0);
    reg = NV_FLD_SET_DRF_NUM(APBDEV_PMC, CNTRL, INTR_POLARITY, value, reg);
    NV_REGW(hRmDevice, NvRmModuleID_Pmif, 0, APBDEV_PMC_CNTRL_0, reg);
}

// KBC reset is available in the pmc control register.
#define RESET_KBC( rm, delay ) \
    do { \
        NvU32 reg; \
        reg = NV_REGR((rm), NvRmModuleID_Pmif, 0, APBDEV_PMC_CNTRL_0); \
        reg = NV_FLD_SET_DRF_DEF(APBDEV_PMC, CNTRL, KBC_RST, ENABLE, reg); \
        NV_REGW((rm), NvRmModuleID_Pmif, 0, APBDEV_PMC_CNTRL_0, reg); \
        if (hold) \
        {\
            break; \
        }\
        NvOsWaitUS(delay); \
        reg = NV_REGR((rm), NvRmModuleID_Pmif, 0, APBDEV_PMC_CNTRL_0); \
        reg = NV_FLD_SET_DRF_DEF(APBDEV_PMC, CNTRL, KBC_RST, DISABLE, reg); \
        NV_REGW((rm), NvRmModuleID_Pmif, 0, APBDEV_PMC_CNTRL_0, reg); \
    } while( 0 )

// Use PMC control to reset the entire SoC. Just wait forever after reset is
// issued - h/w would auto-clear it and restart SoC
#define RESET_SOC( rm ) \
    do { \
        volatile NvBool b = NV_TRUE; \
        NvU32 reg; \
        reg = NV_REGR((rm), NvRmModuleID_Pmif, 0, APBDEV_PMC_CNTRL_0); \
        reg = NV_FLD_SET_DRF_DEF(APBDEV_PMC, CNTRL, MAIN_RST, ENABLE, reg); \
        NV_REGW((rm), NvRmModuleID_Pmif, 0, APBDEV_PMC_CNTRL_0, reg); \
        while (b) { ; } \
    } while( 0 )

void AP20ModuleReset(NvRmDeviceHandle hDevice, NvRmModuleID ModuleId, NvBool hold)
{
    // Extract module and instance from composite module id.
    NvU32 Module   = NVRM_MODULE_ID_MODULE( ModuleId );
    NvU32 Instance = NVRM_MODULE_ID_INSTANCE( ModuleId );

    // Note that VDE has different reset sequence requirement
    // FIMXE: NV blocks - hot reset issues
    #define RESET( rm, offset, field, delay ) \
        do { \
            NvU32 reg; \
            reg = NV_DRF_NUM(CLK_RST_CONTROLLER, RST_DEV_##offset##_SET, SET_##field##_RST, 1); \
            NV_REGW((rm), NvRmPrivModuleID_ClockAndReset, 0, CLK_RST_CONTROLLER_RST_DEV_##offset##_SET_0, reg); \
            if (hold) \
            {  \
                break; \
            } \
            NvOsWaitUS( (delay) ); \
            reg = NV_DRF_NUM(CLK_RST_CONTROLLER, RST_DEV_##offset##_CLR, CLR_##field##_RST, 1); \
            NV_REGW((rm), NvRmPrivModuleID_ClockAndReset, 0, CLK_RST_CONTROLLER_RST_DEV_##offset##_CLR_0, reg); \
        } while( 0 )


    switch( Module ) {
    case NvRmPrivModuleID_MemoryController:
        // FIXME: should this be allowed?
        NV_ASSERT( Instance == 0 );
        RESET( hDevice, H, MEM, NVRM_RESET_DELAY );
        break;
    case NvRmModuleID_Kbc:
        NV_ASSERT( Instance == 0 );
        RESET_KBC(hDevice, NVRM_RESET_DELAY );
        break;
    case NvRmModuleID_SysStatMonitor:
        NV_ASSERT( Instance == 0 );
        RESET( hDevice, H, STAT_MON, NVRM_RESET_DELAY );
        break;
    case NvRmModuleID_Pmif:
        NV_ASSERT( Instance == 0 );
        NV_ASSERT(!"PMC reset is not allowed, and does nothing on AP20");
        break;
    case NvRmModuleID_Fuse:
        NV_ASSERT( Instance == 0 );
        RESET( hDevice, H, FUSE, NVRM_RESET_DELAY );
        break;
    case NvRmModuleID_KFuse:
        NV_ASSERT( Instance == 0 );
        RESET( hDevice, H, KFUSE, NVRM_RESET_DELAY );
        break;
    case NvRmModuleID_Slink:
        // Supporting only the slink controller.
        NV_ASSERT( Instance < 4 );
        if( Instance == 0 )
        {
            RESET( hDevice, H, SBC1, NVRM_RESET_DELAY );
        }
        else if( Instance == 1 )
        {
            RESET( hDevice, H, SBC2, NVRM_RESET_DELAY );
        }
        else if (Instance == 2)
        {
            RESET( hDevice, H, SBC3, NVRM_RESET_DELAY );
        } else if (Instance == 3)
        {
            RESET( hDevice, U, SBC4, NVRM_RESET_DELAY );
        }
        break;
    case NvRmModuleID_Spi:
        NV_ASSERT( Instance == 0 );
        RESET( hDevice, H, SPI1, NVRM_RESET_DELAY );
        break;
    case NvRmModuleID_Xio:
        NV_ASSERT( Instance == 0 );
        RESET( hDevice, H, XIO, NVRM_RESET_DELAY );
        break;
    case NvRmModuleID_Dvc:
        NV_ASSERT( Instance == 0 );
        RESET( hDevice, H, DVC_I2C, NVRM_RESET_DELAY );
        break;
    case NvRmModuleID_Dsi:
        NV_ASSERT( Instance == 0 );
        RESET( hDevice, H, DSI, NVRM_RESET_DELAY );
        break;
    case NvRmModuleID_Tvo:
        NV_ASSERT( Instance == 0 );
        RESET( hDevice, H, TVO, NVRM_RESET_DELAY );
        RESET( hDevice, H, TVDAC, NVRM_RESET_DELAY );
        break;
    case NvRmModuleID_Mipi:
        NV_ASSERT( Instance == 0 );
        RESET( hDevice, H, MIPI, NVRM_RESET_DELAY );
        break;
    case NvRmModuleID_Hdmi:
        NV_ASSERT( Instance == 0 );
        RESET( hDevice, H, HDMI, NVRM_RESET_DELAY );
        break;
    case NvRmModuleID_Csi:
        NV_ASSERT( Instance == 0 );
        RESET( hDevice, H, CSI, NVRM_RESET_DELAY );
        break;
    case NvRmModuleID_I2c:
        if( Instance == 0 )
        {
            RESET( hDevice, L, I2C1, NVRM_RESET_DELAY );
        }
        else if( Instance == 1 )
        {
            RESET( hDevice, H, I2C2, NVRM_RESET_DELAY );
        } else if (Instance == 2)
        {
            RESET( hDevice, U, I2C3, NVRM_RESET_DELAY );
        } else
        {
            NV_ASSERT(!"Invalid I2C instace");
        }
        break;
    case NvRmModuleID_Mpe:
        NV_ASSERT( Instance == 0 );
        RESET( hDevice, H, MPE, NVRM_RESET_DELAY );
        break;
    case NvRmModuleID_Vde:
        NV_ASSERT( Instance == 0 );
        {
            NvU32 reg;

            reg = NV_DRF_NUM(CLK_RST_CONTROLLER, RST_DEV_H_SET,
                    SET_VDE_RST, 1)
                | NV_DRF_NUM(CLK_RST_CONTROLLER, RST_DEV_H_SET,
                    SET_BSEV_RST, 1);
            NV_REGW(hDevice, NvRmPrivModuleID_ClockAndReset, 0,
                CLK_RST_CONTROLLER_RST_DEV_H_SET_0, reg);

            if (hold)
            {
                break;
            }
            NvOsWaitUS( NVRM_RESET_DELAY );

            reg = NV_DRF_NUM(CLK_RST_CONTROLLER, RST_DEV_H_CLR,
                    CLR_VDE_RST, 1)
                | NV_DRF_NUM(CLK_RST_CONTROLLER, RST_DEV_H_CLR,
                    CLR_BSEV_RST, 1);
            NV_REGW(hDevice, NvRmPrivModuleID_ClockAndReset, 0,
                CLK_RST_CONTROLLER_RST_DEV_H_CLR_0, reg);
        }
        break;
    case NvRmModuleID_BseA:
        NV_ASSERT( Instance == 0 );
        RESET( hDevice, H, BSEA, NVRM_RESET_DELAY );
        break;
    case NvRmModuleID_Cpu:
        // FIXME: should this be allowed?
        NV_ASSERT( Instance == 0 );
        RESET( hDevice, L, CPU, NVRM_RESET_DELAY );
        break;
    case NvRmModuleID_Avp:
        // FIXME: should this be allowed?
        NV_ASSERT( Instance == 0 );
        RESET( hDevice, L, COP, NVRM_RESET_DELAY );
        break;
    case NvRmPrivModuleID_System:
        /* THIS WILL DO A FULL SYSTEM RESET */
        NV_ASSERT( Instance == 0 );
        RESET_SOC(hDevice);
        break;
    case NvRmModuleID_Ac97:
        NV_ASSERT( Instance == 0 );
        RESET( hDevice, L, AC97, NVRM_RESET_DELAY );
        break;
    case NvRmModuleID_Rtc:
        NV_ASSERT( Instance == 0 );
        RESET( hDevice, L, RTC, NVRM_RESET_DELAY );
        break;
    case NvRmModuleID_Timer:
        NV_ASSERT( Instance == 0 );
        RESET( hDevice, L, TMR, NVRM_RESET_DELAY );
        break;
    case NvRmModuleID_Uart:
        if( Instance == 0 )
        {
            RESET( hDevice, L, UARTA, NVRM_RESET_DELAY );
        }
        else if( Instance == 1 )
        {
            RESET( hDevice, L, UARTB, NVRM_RESET_DELAY );
        }
        else if ( Instance == 2)
        {
            RESET( hDevice, H, UARTC, NVRM_RESET_DELAY );
        } else if (Instance == 3)
        {
            RESET( hDevice, U, UARTD, NVRM_RESET_DELAY );
        } else if (Instance == 4)
        {
            RESET( hDevice, U, UARTE, NVRM_RESET_DELAY );
        } else
        {
            NV_ASSERT(!"Invalid UART instance");
        }
        break;
    case NvRmModuleID_Vfir:
        // Same as UARTB
        NV_ASSERT( Instance == 0 );
        RESET( hDevice, L, UARTB, NVRM_RESET_DELAY );
        break;
    case NvRmModuleID_Sdio:
        if( Instance == 0 )
        {
            RESET( hDevice, L, SDMMC1, NVRM_RESET_DELAY );
        }
        else if( Instance == 1 )
        {
            RESET( hDevice, L, SDMMC2, NVRM_RESET_DELAY );
        } else if (Instance == 2)
        {
            RESET( hDevice, U, SDMMC3, NVRM_RESET_DELAY );
        } else if (Instance == 3)
        {
            RESET( hDevice, L, SDMMC4, NVRM_RESET_DELAY );
        } else
        {
            NV_ASSERT(!"Invalid SDIO instance");
        }
        break;
    case NvRmModuleID_Spdif:
        NV_ASSERT( Instance == 0 );
        RESET( hDevice, L, SPDIF, NVRM_RESET_DELAY );
        break;
    case NvRmModuleID_I2s:
        if( Instance == 0 )
        {
            RESET( hDevice, L, I2S1, NVRM_RESET_DELAY );
        }
        else if( Instance == 1 )
        {
            RESET( hDevice, L, I2S2, NVRM_RESET_DELAY );
        } else
        {
            NV_ASSERT(!"Invalid I2S instance");
        }
        break;
    case NvRmModuleID_Nand:
        NV_ASSERT( Instance == 0 );
        RESET( hDevice, L, NDFLASH, NVRM_RESET_DELAY );
        break;
    case NvRmModuleID_Twc:
        NV_ASSERT( Instance == 0 );
        RESET( hDevice, L, TWC, NVRM_RESET_DELAY );
        break;
    case NvRmModuleID_Pwm:
        NV_ASSERT( Instance == 0 );
        RESET( hDevice, L, PWM, NVRM_RESET_DELAY );
        break;
    case NvRmModuleID_Epp:
        NV_ASSERT( Instance == 0 );
        RESET( hDevice, L, EPP, NVRM_RESET_DELAY );
        break;
    case NvRmModuleID_Vi:
        NV_ASSERT( Instance == 0 );
        RESET( hDevice, L, VI, NVRM_RESET_DELAY );
        break;
    case NvRmModuleID_3D:
        NV_ASSERT( Instance == 0 );
        RESET( hDevice, L, 3D, NVRM_RESET_DELAY );
        break;
    case NvRmModuleID_2D:
        NV_ASSERT( Instance == 0 );
        // RESET( hDevice, L, 2D, NVRM_RESET_DELAY );
        // WAR for bug 364497, se also NvRmPrivAP20Reset2D()
        NV_ASSERT(!"2D reset after RM open is no longer allowed");
        break;
    case NvRmModuleID_Usb2Otg:
        if (Instance == 0)
        {
            RESET( hDevice, L, USBD, NVRM_RESET_DELAY );
        } else if (Instance == 1)
        {
            RESET( hDevice, H, USB2, NVRM_RESET_DELAY );
        } else if (Instance == 2)
        {
            RESET( hDevice, H, USB3, NVRM_RESET_DELAY );
        } else
        {
            NV_ASSERT(!"Invalid USB instance");
        }
        break;
    case NvRmModuleID_Isp:
        NV_ASSERT( Instance == 0 );
        RESET( hDevice, L, ISP, NVRM_RESET_DELAY );
        break;
    case NvRmModuleID_Ide:
        NV_ASSERT( Instance == 0 );
        RESET( hDevice, L, IDE, NVRM_RESET_DELAY );
        break;
    case NvRmModuleID_Display:
        NV_ASSERT( Instance < 2 );
        if( Instance == 0 )
        {
            RESET( hDevice, L, DISP1, NVRM_RESET_DELAY );
        }
        else if( Instance == 1 )
        {
            RESET( hDevice, L, DISP2, NVRM_RESET_DELAY );
        }
        break;
    case NvRmModuleID_Vcp:
        NV_ASSERT( Instance == 0 );
        RESET( hDevice, L, VCP, NVRM_RESET_DELAY );
        break;
    case NvRmModuleID_CacheMemCtrl:
        NV_ASSERT( Instance < 2 );
        if( Instance == 0 )
        {
            NV_ASSERT(!"There is not such module on AP20");
        }
        else if ( Instance == 1 )
        {
            RESET( hDevice, L, CACHE2, NVRM_RESET_DELAY );
        }
        break;
    case NvRmPrivModuleID_ApbDma:
        NV_ASSERT( Instance == 0 );
        RESET( hDevice, H, APBDMA, NVRM_RESET_DELAY );
        break;
    case NvRmPrivModuleID_Gpio:
        NV_ASSERT( Instance == 0 );
        RESET( hDevice, L, GPIO, NVRM_RESET_DELAY );
        break;
    case NvRmModuleID_GraphicsHost:
        // FIXME: should this be allowed?
        NV_ASSERT( Instance == 0 );
        RESET( hDevice, L, HOST1X, NVRM_RESET_DELAY );
        break;
    case NvRmPrivModuleID_PcieXclk:
        NV_ASSERT( Instance == 0 );
        RESET( hDevice, U, PCIEXCLK, NVRM_RESET_DELAY );
        break;
    case NvRmPrivModuleID_Pcie:
        NV_ASSERT( Instance == 0 );
        RESET( hDevice, U, PCIE, NVRM_RESET_DELAY );
        break;
    case NvRmPrivModuleID_Afi:
        NV_ASSERT( Instance == 0 );
        RESET( hDevice, U, AFI, NVRM_RESET_DELAY );
        break;
    case NvRmModuleID_SyncNor:
        NV_ASSERT( Instance == 0 );
        RESET( hDevice, H, SNOR, NVRM_RESET_DELAY );
        break;
    case NvRmModuleID_AvpUcq:
        NV_ASSERT( Instance == 0 );
        RESET( hDevice, U, AVPUCQ, NVRM_RESET_DELAY );
        break;
    case NvRmModuleID_OneWire:
        NV_ASSERT( Instance == 0 );
        RESET( hDevice, U, OWR, NVRM_RESET_DELAY );
        break;

    default:
        NV_ASSERT(!"Invalid ModuleId");
    }

    #undef RESET
}

static void
NvRmPrivContentProtectionFuses( NvRmDeviceHandle hRm )
{
    NvU32 reg;
    NvU32 clk_rst;

    /* need to set FUSE_RESERVED_PRODUCTION_0 to 0x3,
     * enable the bypass and write access
     *
     * bit 0: macrovision
     * bit 1: hdcp
     */

#if NV_USE_FUSE_CLOCK_ENABLE
    // Enable fuse clock
    Ap20EnableModuleClock(hRm, NvRmModuleID_Fuse, NV_TRUE);
#endif

    /**
     * This order is IMPORTANT. Fuse bypass doesn't seem to work with
     * different ordering.
     */

    clk_rst = NV_REGR( hRm, NvRmPrivModuleID_ClockAndReset, 0,
        CLK_RST_CONTROLLER_MISC_CLK_ENB_0 );
    clk_rst = NV_FLD_SET_DRF_NUM( CLK_RST_CONTROLLER, MISC_CLK_ENB,
        CFG_ALL_VISIBLE, 1, clk_rst );
    NV_REGW( hRm, NvRmPrivModuleID_ClockAndReset, 0,
        CLK_RST_CONTROLLER_MISC_CLK_ENB_0, clk_rst );

    reg = NV_REGR( hRm, NvRmModuleID_Fuse, 0, FUSE_FUSEBYPASS_0);
    reg = NV_FLD_SET_DRF_DEF( FUSE, FUSEBYPASS, FUSEBYPASS_VAL, ENABLED, reg );
    NV_REGW( hRm, NvRmModuleID_Fuse, 0, FUSE_FUSEBYPASS_0, reg );

    reg = NV_REGR( hRm, NvRmModuleID_Fuse, 0, FUSE_WRITE_ACCESS_SW_0);
    reg = NV_FLD_SET_DRF_DEF( FUSE, WRITE_ACCESS_SW, WRITE_ACCESS_SW_CTRL,
            READWRITE, reg);
    NV_REGW( hRm, NvRmModuleID_Fuse, 0, FUSE_WRITE_ACCESS_SW_0, reg );

    reg = NV_REGR( hRm, NvRmModuleID_Fuse, 0, FUSE_RESERVED_PRODUCTION_0);
    reg = NV_FLD_SET_DRF_NUM( FUSE, RESERVED_PRODUCTION,
            RESERVED_PRODUCTION, 0x3, reg );
    NV_REGW( hRm, NvRmModuleID_Fuse, 0, FUSE_RESERVED_PRODUCTION_0, reg );

    clk_rst = NV_FLD_SET_DRF_NUM( CLK_RST_CONTROLLER, MISC_CLK_ENB,
        CFG_ALL_VISIBLE, 0, clk_rst );
    NV_REGW( hRm, NvRmPrivModuleID_ClockAndReset, 0,
        CLK_RST_CONTROLLER_MISC_CLK_ENB_0, clk_rst );

#if NV_USE_FUSE_CLOCK_ENABLE
    // Disable fuse clock
    Ap20EnableModuleClock(hRm, NvRmModuleID_Fuse, NV_FALSE);
#endif
}

// Safe PLLM (max 1000MHz) divider for GPU modules
#define NVRM_SAFE_GPU_DIVIDER (10)

void
NvRmPrivAp20Reset2D(NvRmDeviceHandle hRmDevice)
{
#if !NV_OAL
    NvU32 reg, offset;
    /*
     * WAR for bug 364497: 2D can not be taken out of reset if VI clock is
     * running. Therefore, make sure VI clock is disabled and reset 2D here
     * during RM initialization.
     */
    Ap20EnableModuleClock(hRmDevice, NvRmModuleID_Vi,
        ModuleClockState_Disable);

    // Assert reset to 2D module
    offset = CLK_RST_CONTROLLER_RST_DEV_L_SET_0;
    reg = NV_DRF_NUM(CLK_RST_CONTROLLER, RST_DEV_L_SET, SET_2D_RST, 1);
    NV_REGW(hRmDevice, NvRmPrivModuleID_ClockAndReset, 0, offset, reg);

    // Enable "known good" configuartion for 2D clock (PLLM as a source)
    offset = CLK_RST_CONTROLLER_CLK_SOURCE_G2D_0;
    NV_REGW(hRmDevice, NvRmPrivModuleID_ClockAndReset, 0, offset,
            (NV_DRF_NUM(CLK_RST_CONTROLLER, CLK_SOURCE_G2D, G2D_CLK_DIVISOR,
                        NVRM_SAFE_GPU_DIVIDER) |
             NV_DRF_DEF(CLK_RST_CONTROLLER, CLK_SOURCE_G2D, G2D_CLK_SRC,
                        PLLM_OUT0))
            );
    Ap20EnableModuleClock(hRmDevice, NvRmModuleID_2D, ModuleClockState_Enable);
    NvOsWaitUS(NVRM_RESET_DELAY);

    // Take 2D out of reset and disable 2D clock. Both VI and 2D clocks are
    // left disabled -it is up to the resepctive drivers to configure and
    // enable them later.
    offset = CLK_RST_CONTROLLER_RST_DEV_L_CLR_0;
    reg = NV_DRF_NUM(CLK_RST_CONTROLLER, RST_DEV_L_CLR, CLR_2D_RST, 1);
    NV_REGW(hRmDevice, NvRmPrivModuleID_ClockAndReset, 0, offset, reg);
    Ap20EnableModuleClock(hRmDevice, NvRmModuleID_2D,
        ModuleClockState_Disable);
#endif
}

#define NVRM_CONFIG_CLOCK(Module, SrcDef, DivNum) \
do\
{\
    reg = NV_REGR(rm, NvRmPrivModuleID_ClockAndReset, 0, \
                  CLK_RST_CONTROLLER_CLK_SOURCE_##Module##_0); \
    if ((DivNum) > NV_DRF_VAL(CLK_RST_CONTROLLER, CLK_SOURCE_##Module, \
                            Module##_CLK_DIVISOR, reg)) \
    {\
        reg = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, CLK_SOURCE_##Module, \
                                 Module##_CLK_DIVISOR, (DivNum), reg); \
        NV_REGW(rm, NvRmPrivModuleID_ClockAndReset, 0, \
                CLK_RST_CONTROLLER_CLK_SOURCE_##Module##_0, reg); \
        NvOsWaitUS(NVRM_CLOCK_CHANGE_DELAY); \
    }\
    reg = NV_FLD_SET_DRF_DEF(CLK_RST_CONTROLLER, CLK_SOURCE_##Module, \
                             Module##_CLK_SRC, SrcDef, reg); \
    NV_REGW(rm, NvRmPrivModuleID_ClockAndReset, 0, \
            CLK_RST_CONTROLLER_CLK_SOURCE_##Module##_0, reg); \
    NvOsWaitUS(NVRM_CLOCK_CHANGE_DELAY); \
    if ((DivNum) < NV_DRF_VAL(CLK_RST_CONTROLLER, CLK_SOURCE_##Module, \
                            Module##_CLK_DIVISOR, reg))\
    {\
        reg = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, CLK_SOURCE_##Module, \
                                 Module##_CLK_DIVISOR, (DivNum), reg); \
        NV_REGW(rm, NvRmPrivModuleID_ClockAndReset, 0, \
                CLK_RST_CONTROLLER_CLK_SOURCE_##Module##_0, reg); \
        NvOsWaitUS(NVRM_CLOCK_CHANGE_DELAY);\
    }\
} while(0)

void
NvRmPrivAp20BasicReset( NvRmDeviceHandle rm )
{
#if !NV_OAL
    NvU32 reg, ClkOutL, ClkOutH, ClkOutU;
    ExecPlatform env;

    if (NvRmIsSimulation())
    {
        /* the memory system can't be used until the mem_init_done bit has
         * been set.  This is done by the bootrom for production systems.
         */
        reg = NV_REGR( rm, NvRmPrivModuleID_Ahb_Arb_Ctrl, 0,
                       AHB_ARBITRATION_XBAR_CTRL_0 );
        reg = NV_FLD_SET_DRF_DEF( AHB_ARBITRATION, XBAR_CTRL, MEM_INIT_DONE,
                                  DONE, reg );
        NV_REGW( rm, NvRmPrivModuleID_Ahb_Arb_Ctrl, 0,
                 AHB_ARBITRATION_XBAR_CTRL_0, reg );
    }

    // FIXME: this takes the Big Hammer Approach.  Take everything out
    // of reset and enable all of the clocks. Then keep enabled only boot
    // clocks and graphics host.

    // save boot clock enable state
    ClkOutL = NV_REGR(rm, NvRmPrivModuleID_ClockAndReset, 0,
                      CLK_RST_CONTROLLER_CLK_OUT_ENB_L_0);
    ClkOutH = NV_REGR(rm, NvRmPrivModuleID_ClockAndReset, 0,
                      CLK_RST_CONTROLLER_CLK_OUT_ENB_H_0);
    ClkOutU = NV_REGR(rm, NvRmPrivModuleID_ClockAndReset, 0,
                      CLK_RST_CONTROLLER_CLK_OUT_ENB_U_0);

    // Enable module clocks
    // (for U register module clocks are in the low word only)
    NV_REGW( rm, NvRmPrivModuleID_ClockAndReset, 0,
        CLK_RST_CONTROLLER_CLK_ENB_L_SET_0, 0xFFFFFFFF );
    NV_REGW( rm, NvRmPrivModuleID_ClockAndReset, 0,
        CLK_RST_CONTROLLER_CLK_ENB_H_SET_0, 0xFFFFFFFF );
    NV_REGW( rm, NvRmPrivModuleID_ClockAndReset, 0,
        CLK_RST_CONTROLLER_CLK_ENB_U_SET_0, 0x0000FFFF );

    // For AP20 default clock source selection is out of range for some modules
    // Just copnfigure safe clocks so that reset is propagated correctly
    env = NvRmPrivGetExecPlatform(rm);
    if (env == ExecPlatform_Soc)
    {
        /*
         * For peripheral modules default clock source is oscillator, and
         * it is safe. Special case SPDIFIN - set on PLLP_OUT0/(1+10/2)
         * and VDE - set on PLLP_OUT0/(1+1/2)
         */
        reg = NV_REGR(rm, NvRmPrivModuleID_ClockAndReset, 0,
                      CLK_RST_CONTROLLER_RST_DEVICES_L_0);
        if (reg & CLK_RST_CONTROLLER_RST_DEVICES_L_0_SWR_SPDIF_RST_FIELD)
        {
            reg = NV_DRF_DEF(CLK_RST_CONTROLLER, CLK_SOURCE_SPDIF_IN,
                             SPDIFIN_CLK_SRC, PLLP_OUT0) |
                  NV_DRF_NUM(CLK_RST_CONTROLLER, CLK_SOURCE_SPDIF_IN,
                             SPDIFIN_CLK_DIVISOR, 10);
            NV_REGW( rm, NvRmPrivModuleID_ClockAndReset, 0,
                CLK_RST_CONTROLLER_CLK_SOURCE_SPDIF_IN_0, reg);
        }
        NVRM_CONFIG_CLOCK(VDE, PLLP_OUT0, 1);

        /*
         * For graphic clocks use PLLM_OUT0 as a source, and set divider
         * so that initial frequency is below maximum module limit
         */
        NVRM_CONFIG_CLOCK(HOST1X, PLLM_OUT0, NVRM_SAFE_GPU_DIVIDER);
        NVRM_CONFIG_CLOCK(EPP, PLLM_OUT0, NVRM_SAFE_GPU_DIVIDER);
        NVRM_CONFIG_CLOCK(G2D, PLLM_OUT0, NVRM_SAFE_GPU_DIVIDER);
        NVRM_CONFIG_CLOCK(G3D, PLLM_OUT0, NVRM_SAFE_GPU_DIVIDER);
        NVRM_CONFIG_CLOCK(MPE, PLLM_OUT0, NVRM_SAFE_GPU_DIVIDER);
        NVRM_CONFIG_CLOCK(VI, PLLM_OUT0, NVRM_SAFE_GPU_DIVIDER);
        NVRM_CONFIG_CLOCK(VI_SENSOR, PLLM_OUT0, NVRM_SAFE_GPU_DIVIDER);

        /* Using 144MHz for coresight */
        NVRM_CONFIG_CLOCK(CSITE, PLLP_OUT0, 1);

        NvOsWaitUS(NVRM_RESET_DELAY);
    }
    // Make sure Host1x clock will be kept enabled
    ClkOutL = NV_FLD_SET_DRF_DEF(CLK_RST_CONTROLLER, CLK_OUT_ENB_L,
                                 CLK_ENB_HOST1X, ENABLE, ClkOutL);
    // Make sure VDE, BSEV and BSEA clocks will be kept disabled
    ClkOutH = NV_FLD_SET_DRF_DEF(CLK_RST_CONTROLLER, CLK_OUT_ENB_H,
                                 CLK_ENB_VDE, DISABLE, ClkOutH);
    ClkOutH = NV_FLD_SET_DRF_DEF(CLK_RST_CONTROLLER, CLK_OUT_ENB_H,
                                 CLK_ENB_BSEV, DISABLE, ClkOutH);
    ClkOutH = NV_FLD_SET_DRF_DEF(CLK_RST_CONTROLLER, CLK_OUT_ENB_H,
                                 CLK_ENB_BSEA, DISABLE, ClkOutH);
    // Make sure SNOR clock will be kept disabled
    ClkOutH = NV_FLD_SET_DRF_DEF(CLK_RST_CONTROLLER, CLK_OUT_ENB_H,
                                 CLK_ENB_SNOR, DISABLE, ClkOutH);

    // restore clock enable state (= disable those clocks that
    // were disabled on boot)
    NV_REGW( rm, NvRmPrivModuleID_ClockAndReset, 0,
        CLK_RST_CONTROLLER_CLK_OUT_ENB_L_0, ClkOutL );
    NV_REGW( rm, NvRmPrivModuleID_ClockAndReset, 0,
        CLK_RST_CONTROLLER_CLK_OUT_ENB_H_0, ClkOutH );
    NV_REGW( rm, NvRmPrivModuleID_ClockAndReset, 0,
        CLK_RST_CONTROLLER_CLK_OUT_ENB_U_0, ClkOutU );

    /* enable hdcp and macrovision */
    NvRmPrivContentProtectionFuses( rm );

    // AP15 BasicReset() sets DRAM_CLKSTOP and DRAM_ACPD here.
    // Should be done by BCT - removing for AP20.

    // AP15 BasicReset() enables stop clock to CPU, while it is halted.
    // Removed in AP20 as halt on dual core is actually WFE
#endif // !NV_OAL
}

void NvRmPrivAp20IoPowerDetectReset(NvRmDeviceHandle hRmDeviceHandle)
{
    NV_REGW(hRmDeviceHandle, NvRmModuleID_Pmif, 0,
            APBDEV_PMC_PWR_DET_VAL_0, APBDEV_PMC_PWR_DET_VAL_0_RESET_VAL);
}

/*****************************************************************************/

NvError
NvRmPrivAp20OscDoublerConfigure(
    NvRmDeviceHandle hRmDevice,
    NvRmFreqKHz OscKHz)
{
    NvU32 reg, Taps;
#if NVRM_AP20_USE_OSC_DOUBLER
    NvError error = NvRmPrivGetOscDoublerTaps(hRmDevice, OscKHz, &Taps);
#else
    NvError error = NvError_NotSupported;
#endif

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
        reg = NV_DRF_NUM(
            CLK_RST_CONTROLLER, CLK_ENB_U_SET, SET_CLK_M_DOUBLER_ENB, 1);
        NV_REGW(hRmDevice, NvRmPrivModuleID_ClockAndReset, 0,
                CLK_RST_CONTROLLER_CLK_ENB_U_SET_0, reg);
    }
    else
    {
        // Disable doubler
        reg = NV_DRF_NUM(
            CLK_RST_CONTROLLER, CLK_ENB_U_CLR, CLR_CLK_M_DOUBLER_ENB, 1);
        NV_REGW(hRmDevice, NvRmPrivModuleID_ClockAndReset, 0,
                CLK_RST_CONTROLLER_CLK_ENB_U_CLR_0, reg);
    }
    return error;
}

#define APBDEV_PMC_SCRATCH42_0_PCX_CLAMP_RANGE  0:0

#define NVRM_PCIE_REF_FREQUENCY (12000)

void NvRmPrivAp20PllEControl(NvRmDeviceHandle hRmDevice, NvBool Enable)
{
    static NvBool s_Started = NV_FALSE;

    NvU32 base, reg, offset;

    if (NvRmPrivGetExecPlatform(hRmDevice) != ExecPlatform_Soc)
        return;

    if (NvRmPowerGetPrimaryFrequency(hRmDevice) != NVRM_PCIE_REF_FREQUENCY)
    {
        NV_ASSERT(!"Not supported primary frequency");
        return;
    }

    // No run time power management for PCIE PLL - once started, it will never
    // be disabled
    if (s_Started || !Enable)
        return;

    // Do not start PLLE while it is clamped
    offset = APBDEV_PMC_SCRATCH42_0;
    reg = NV_REGR(hRmDevice, NvRmModuleID_Pmif, 0, offset);
    if (NV_DRF_VAL(APBDEV_PMC, SCRATCH42, PCX_CLAMP, reg) && Enable)
        return;

    s_Started = NV_TRUE;

    // Set PLLE base = 0x0D18C801 (configured, but disabled)
    offset = CLK_RST_CONTROLLER_PLLE_BASE_0;
    base= NV_DRF_DEF(CLK_RST_CONTROLLER, PLLE_BASE, PLLE_ENABLE_CML, DISABLE) |
          NV_DRF_DEF(CLK_RST_CONTROLLER, PLLE_BASE, PLLE_ENABLE, DISABLE)     |
          NV_DRF_NUM(CLK_RST_CONTROLLER, PLLE_BASE, PLLE_PLDIV_CML,  0x0D)    |
          NV_DRF_NUM(CLK_RST_CONTROLLER, PLLE_BASE, PLLE_PLDIV,      0x18)    |
          NV_DRF_NUM(CLK_RST_CONTROLLER, PLLE_BASE, PLLE_NDIV,       0xC8)    |
          NV_DRF_NUM(CLK_RST_CONTROLLER, PLLE_BASE, PLLE_MDIV,       0x01);
    NV_REGW(hRmDevice, NvRmPrivModuleID_ClockAndReset, 0, offset, base);

    // Pulse IDDQ/clamp signal to start training
    offset = APBDEV_PMC_SCRATCH42_0;
    reg = NV_REGR(hRmDevice, NvRmModuleID_Pmif, 0, offset);
    reg = NV_FLD_SET_DRF_NUM(APBDEV_PMC, SCRATCH42, PCX_CLAMP, 0x1, reg);
    NV_REGW(hRmDevice, NvRmModuleID_Pmif, 0, offset, reg);

    NvOsWaitUS(NVRM_CLOCK_CHANGE_DELAY); // wait > 1us

    reg = NV_FLD_SET_DRF_NUM(APBDEV_PMC, SCRATCH42, PCX_CLAMP, 0x0, reg);
    NV_REGW(hRmDevice, NvRmModuleID_Pmif, 0, offset, reg);

    // Poll PLLE ready
    offset = CLK_RST_CONTROLLER_PLLE_MISC_0;
    reg = NV_REGR(hRmDevice, NvRmPrivModuleID_ClockAndReset, 0, offset);
    while (!(NV_DRF_VAL(CLK_RST_CONTROLLER, PLLE_MISC, PLLE_PLL_READY, reg)))
    {
        reg = NV_REGR(hRmDevice, NvRmPrivModuleID_ClockAndReset, 0, offset);
    }

    // Set PLLE base = 0xCD18C801 (configured and enabled)
    offset = CLK_RST_CONTROLLER_PLLE_BASE_0;
    base = NV_FLD_SET_DRF_DEF(
        CLK_RST_CONTROLLER, PLLE_BASE, PLLE_ENABLE_CML, ENABLE, base);
    base = NV_FLD_SET_DRF_DEF(
        CLK_RST_CONTROLLER, PLLE_BASE, PLLE_ENABLE, ENABLE, base);
    NV_REGW(hRmDevice, NvRmPrivModuleID_ClockAndReset, 0, offset, base);

    // use MIPI PLL delay for now - TODO: confirm or find PLLE specific
    NvOsWaitUS(NVRM_PLL_MIPI_STABLE_DELAY_US);
}

void
NvRmPrivAp20PowerPcieXclkControl(
    NvRmDeviceHandle hRmDevice,
    NvBool Enable)
{
    NvU32 reg, offset;

    offset = APBDEV_PMC_SCRATCH42_0;
    reg = NV_REGR(hRmDevice, NvRmModuleID_Pmif, 0, offset);
    reg = NV_FLD_SET_DRF_NUM(
        APBDEV_PMC, SCRATCH42, PCX_CLAMP, Enable ? 0x0 : 0x1, reg);
    NV_REGW(hRmDevice, NvRmModuleID_Pmif, 0, offset, reg);
}

