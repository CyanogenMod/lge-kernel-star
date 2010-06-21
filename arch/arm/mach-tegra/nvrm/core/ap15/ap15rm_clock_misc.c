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

#include "nvcommon.h"
#include "nvassert.h"
#include "nvos.h"
#include "nvrm_init.h"
#include "nvrm_drf.h"
#include "nvrm_hwintf.h"
#include "nvrm_clocks.h"
#include "nvrm_chiplib.h"
#include "nvrm_hardware_access.h"
#include "ap15rm_private.h"
#include "ap15rm_clocks.h"
#include "ap16/arapb_misc.h"
#include "ap15/arahb_arbc.h"
#include "ap15/armc.h"
#include "ap15/aremc.h"
#include "ap15/arfuse.h"
#include "ap15/arclk_rst.h"


// This list requires pre-sorted info in bond-out registers order and bond-out
// register bit shift order (MSB-to-LSB).
static const NvU32 s_Ap15BondOutTable[] =
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
    NVRM_MODULE_ID( NvRmModuleID_Hsmmc,                         0 ),
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
    NVRM_MODULE_ID( NvRmModuleID_Slink,                         0 ),
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
    NVRM_DEVICE_UNKNOWN,
    NVRM_DEVICE_UNKNOWN,
    NVRM_MODULE_ID( NvRmModuleID_Mpe,                           0 ),
    NVRM_MODULE_ID( NvRmModuleID_Vde,                           0 ),
    NVRM_MODULE_ID( NvRmModuleID_BseA,                          0 ),
    NVRM_DEVICE_UNKNOWN,    //BSEV
};

/**
 * Enable HDCP and Macrovision
 */
static void
NvRmPrivContentProtectionFuses( NvRmDeviceHandle hRm )
{
    NvU32 reg;
    NvU32 clk_rst;

    /* need to set FUSEWRDATA3_RESERVED_PRODUCTION__PRI_ALIAS to 0x3 and
     * enable the bypass.
     *
     * bit 0: macrovision
     * bit 1: hdcp
     */

#if NV_USE_FUSE_CLOCK_ENABLE
    // Enable fuse clock
    Ap15EnableModuleClock(hRm, NvRmModuleID_Fuse, NV_TRUE);
#endif

    clk_rst = NV_REGR( hRm, NvRmPrivModuleID_ClockAndReset, 0,
        CLK_RST_CONTROLLER_MISC_CLK_ENB_0 );
    clk_rst = NV_FLD_SET_DRF_NUM( CLK_RST_CONTROLLER, MISC_CLK_ENB,
        CFG_ALL_VISIBLE, 1, clk_rst );
    NV_REGW( hRm, NvRmPrivModuleID_ClockAndReset, 0,
        CLK_RST_CONTROLLER_MISC_CLK_ENB_0, clk_rst );

    reg = NV_DRF_NUM( FUSE, FUSEWRDATA3,
        FUSEWRDATA_RESERVED_PRODUCTION__PRI_ALIAS_0, 0x3 );
    NV_REGW( hRm, NvRmModuleID_Fuse, 0, FUSE_FUSEWRDATA3_0, reg );

    reg = NV_DRF_DEF( FUSE, FUSEBYPASS, FUSEBYPASS_VAL, ENABLED );
    NV_REGW( hRm, NvRmModuleID_Fuse, 0, FUSE_FUSEBYPASS_0, reg );

    clk_rst = NV_FLD_SET_DRF_NUM( CLK_RST_CONTROLLER, MISC_CLK_ENB,
        CFG_ALL_VISIBLE, 0, clk_rst );
    NV_REGW( hRm, NvRmPrivModuleID_ClockAndReset, 0,
        CLK_RST_CONTROLLER_MISC_CLK_ENB_0, clk_rst );

#if NV_USE_FUSE_CLOCK_ENABLE
    // Disable fuse clock
    Ap15EnableModuleClock(hRm, NvRmModuleID_Fuse, NV_FALSE);
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

#define NVRM_SET_OSC_CLOCK(ClkModule, RstModule, H_L) \
do\
{\
    if (RstOut##H_L & \
        CLK_RST_CONTROLLER_RST_DEVICES_##H_L##_0_SWR_##RstModule##_RST_FIELD) \
    {\
        reg = NV_REGR(rm, NvRmPrivModuleID_ClockAndReset, 0, \
                      CLK_RST_CONTROLLER_CLK_SOURCE_##ClkModule##_0); \
        reg = NV_FLD_SET_DRF_DEF(CLK_RST_CONTROLLER, CLK_SOURCE_##ClkModule, \
                                 ClkModule##_CLK_SRC, CLK_M, reg); \
        NV_REGW(rm, NvRmPrivModuleID_ClockAndReset, 0, \
                CLK_RST_CONTROLLER_CLK_SOURCE_##ClkModule##_0, reg); \
    }\
} while(0)

/**
 * brings the minimum modules out of reset.
 */
void
NvRmPrivAp15BasicReset( NvRmDeviceHandle rm )
{
#if !NV_OAL
    NvU32 reg, RstOutL, RstOutH, ClkOutL, ClkOutH;
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

    // get boot module reset state
    RstOutL = NV_REGR(rm, NvRmPrivModuleID_ClockAndReset, 0,
                      CLK_RST_CONTROLLER_RST_DEVICES_L_0);
    RstOutH = NV_REGR(rm, NvRmPrivModuleID_ClockAndReset, 0,
                      CLK_RST_CONTROLLER_RST_DEVICES_H_0);

    // save boot clock enable state
    ClkOutL = NV_REGR(rm, NvRmPrivModuleID_ClockAndReset, 0,
                      CLK_RST_CONTROLLER_CLK_OUT_ENB_L_0);
    ClkOutH = NV_REGR(rm, NvRmPrivModuleID_ClockAndReset, 0,
                      CLK_RST_CONTROLLER_CLK_OUT_ENB_H_0);

    /* write clk_out_enb_l */
    NV_REGW( rm, NvRmPrivModuleID_ClockAndReset, 0,
        CLK_RST_CONTROLLER_CLK_OUT_ENB_L_0, 0xFFFFFFFF );

    /* write clk_out_enb_h */
    NV_REGW( rm, NvRmPrivModuleID_ClockAndReset, 0,
        CLK_RST_CONTROLLER_CLK_OUT_ENB_H_0, 0xFFFFFFFF );

    // For AP15 default clock source selection is out of range for many modules
    // Just copnfigure clocks so that reset is propagated correctly
    env = NvRmPrivGetExecPlatform(rm);
    if (env == ExecPlatform_Soc)
    {
        /*
         * For peripheral modules that are not taken from reset, yet,
         * use oscillator as a safe clock
         */
        NVRM_SET_OSC_CLOCK(I2S1, I2S1, L);
        NVRM_SET_OSC_CLOCK(I2S2, I2S2, L);

        NVRM_SET_OSC_CLOCK(I2C1, I2C1, L);
        NVRM_SET_OSC_CLOCK(I2C2, I2C2, H);
        NVRM_SET_OSC_CLOCK(DVC_I2C, DVC_I2C, H);

        NVRM_SET_OSC_CLOCK(PWM, PWM, L);
        NVRM_SET_OSC_CLOCK(XIO, XIO, H);
        NVRM_SET_OSC_CLOCK(TWC, TWC, L);
        NVRM_SET_OSC_CLOCK(HSMMC, HSMMC, L);

        NVRM_SET_OSC_CLOCK(VFIR, UARTB, L);
        NVRM_SET_OSC_CLOCK(UARTA, UARTA, L);
        NVRM_SET_OSC_CLOCK(UARTB, UARTB, L);
        NVRM_SET_OSC_CLOCK(UARTC, UARTC, H);

        NVRM_SET_OSC_CLOCK(NDFLASH, NDFLASH, L);
        NVRM_SET_OSC_CLOCK(IDE, IDE, L);
        NVRM_SET_OSC_CLOCK(MIPI, MIPI, H);
        NVRM_SET_OSC_CLOCK(SDIO1, SDIO1, L);
        NVRM_SET_OSC_CLOCK(SDIO2, SDIO2, L);

        NVRM_SET_OSC_CLOCK(SPI1, SPI1, H);
        NVRM_SET_OSC_CLOCK(SBC1, SBC1, H);
        NVRM_SET_OSC_CLOCK(SBC2, SBC2, H);
        NVRM_SET_OSC_CLOCK(SBC3, SBC3, H);

        NVRM_SET_OSC_CLOCK(DISP1, DISP1, L);
        NVRM_SET_OSC_CLOCK(DISP2, DISP2, L);
        NVRM_SET_OSC_CLOCK(TVO, TVO, H);
        NVRM_SET_OSC_CLOCK(CVE, TVO, H);
        NVRM_SET_OSC_CLOCK(HDMI, HDMI, H);
        NVRM_SET_OSC_CLOCK(TVDAC, TVDAC, H);

        // Special case SPDIF (set OUT on OSC; IN on PLLP_OUT0/(1+10/2))
        if (RstOutL & CLK_RST_CONTROLLER_RST_DEVICES_L_0_SWR_SPDIF_RST_FIELD)
        {
            reg = NV_DRF_DEF(CLK_RST_CONTROLLER, CLK_SOURCE_SPDIF,
                             SPDIFOUT_CLK_SRC, CLK_M) |
                  NV_DRF_DEF(CLK_RST_CONTROLLER, CLK_SOURCE_SPDIF,
                             SPDIFIN_CLK_SRC, PLLP_OUT0) |
                  NV_DRF_NUM(CLK_RST_CONTROLLER, CLK_SOURCE_SPDIF,
                             SPDIFIN_CLK_DIVISOR, 10);
            NV_REGW( rm, NvRmPrivModuleID_ClockAndReset, 0,
                CLK_RST_CONTROLLER_CLK_SOURCE_SPDIF_0, reg);
        }

        /*
         * For graphic clocks use PLLM_OUT0 (max 400MHz) as a source, and set
         * divider so that initial frequency is below maximum module limit
         * (= PLLM_OUT0 / (1 + DIVIDER/2)
         */
        #define G_DIVIDER (2)
        NVRM_CONFIG_CLOCK(HOST1X, PLLM_OUT0, G_DIVIDER);
        NVRM_CONFIG_CLOCK(EPP, PLLM_OUT0, G_DIVIDER);
        NVRM_CONFIG_CLOCK(G2D, PLLM_OUT0, G_DIVIDER);
        NVRM_CONFIG_CLOCK(G3D, PLLM_OUT0, G_DIVIDER);
        NVRM_CONFIG_CLOCK(MPE, PLLM_OUT0, G_DIVIDER);
        #define VI_DIVIDER (4)
        NVRM_CONFIG_CLOCK(VI, PLLM_OUT0, VI_DIVIDER);
        NVRM_CONFIG_CLOCK(VI_SENSOR, PLLM_OUT0, VI_DIVIDER);

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

    /* write rst_devices_l */
    NV_REGW( rm, NvRmPrivModuleID_ClockAndReset, 0,
        CLK_RST_CONTROLLER_RST_DEVICES_L_0, 0 );

    /* write rst_devies_h */
    NV_REGW( rm, NvRmPrivModuleID_ClockAndReset, 0,
        CLK_RST_CONTROLLER_RST_DEVICES_H_0, 0 );

    // restore clock enable state (= disable those clocks that
    // were disabled on boot)
    NV_REGW( rm, NvRmPrivModuleID_ClockAndReset, 0,
        CLK_RST_CONTROLLER_CLK_OUT_ENB_L_0, ClkOutL );
    NV_REGW( rm, NvRmPrivModuleID_ClockAndReset, 0,
        CLK_RST_CONTROLLER_CLK_OUT_ENB_H_0, ClkOutH );

    /* enable hdcp and macrovision */
    NvRmPrivContentProtectionFuses( rm );

    // 10jun2008: jn turning this back on.  Still need to solve the QT
    //            issue.
    // FIXME:  On Quickturn and FPGA we are using normal sdram, not mobile
    //         ram.  Need some way to determine if we have normal sdram
    //         or mobile sdram. Actually these bits should be set by BCT.
    reg = NV_REGR(rm, NvRmPrivModuleID_ExternalMemoryController, 0,
        EMC_CFG_0);
    reg = NV_FLD_SET_DRF_DEF(EMC, CFG, DRAM_CLKSTOP, ENABLED, reg);
    reg = NV_FLD_SET_DRF_DEF(EMC, CFG, DRAM_ACPD, ACTIVE_POWERDOWN, reg);
    NV_REGW( rm, NvRmPrivModuleID_ExternalMemoryController, 0,
             EMC_CFG_0, reg);

    // Enable stop clock to CPU, while it is halted
    reg = NV_REGR(rm, NvRmPrivModuleID_ClockAndReset, 0,
        CLK_RST_CONTROLLER_CLK_MASK_ARM_0);
    reg = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, CLK_MASK_ARM,
        CLK_MASK_CPU_HALT, 1, reg );
    NV_REGW(rm, NvRmPrivModuleID_ClockAndReset, 0,
        CLK_RST_CONTROLLER_CLK_MASK_ARM_0, reg);

    if (rm->ChipId.Id == 0x16)
    {
        NvU32 Reg = 0;
        // If USB main clock source is not enabled then disable the clocks to USB0 and USB1
        Reg = NV_REGR(rm, NvRmPrivModuleID_ClockAndReset, 0, CLK_RST_CONTROLLER_CLK_OUT_ENB_L_0);
        if (!NV_DRF_VAL(CLK_RST_CONTROLLER, CLK_OUT_ENB_L, CLK_ENB_USBD, Reg))
        {
            // Disable clocks for USB1 and USB2 controllers.Should be enabled on need basis.
            Reg = NV_REGR(rm, NvRmModuleID_Misc, 0, APB_MISC_PP_MISC_USB_CLK_RST_CTL_0);
            Reg = NV_FLD_SET_DRF_DEF(APB_MISC_PP,MISC_USB_CLK_RST_CTL, MISC_USB_CE, DISABLE, Reg);
            Reg = NV_FLD_SET_DRF_DEF(APB_MISC_PP,MISC_USB_CLK_RST_CTL, MISC_USB2_CE, DISABLE, Reg);
            NV_REGW(rm, NvRmModuleID_Misc, 0, APB_MISC_PP_MISC_USB_CLK_RST_CTL_0, Reg);
        }
    }

#endif // !NV_OAL
}

static void
NvRmPrivAp15GetBondOut( NvRmDeviceHandle hDevice,
                        const NvU32      **pTable,
                        NvU32            *bondOut )
{
    *pTable = s_Ap15BondOutTable;
    bondOut[0] = NV_REGR(hDevice, NvRmPrivModuleID_ClockAndReset, 0,
                                    CLK_RST_CONTROLLER_BOND_OUT_L_0);
    bondOut[1] = NV_REGR(hDevice, NvRmPrivModuleID_ClockAndReset, 0,
                                    CLK_RST_CONTROLLER_BOND_OUT_H_0);
}


#define NVRM_MAX_BOND_OUT_REG    3

/*
 * Check BondOut register to determine which module and/or module instance
 * is not available.
 */
void
NvRmPrivCheckBondOut( NvRmDeviceHandle hDevice )
{
    NvRmModuleTable     *mod_table = 0;
    NvRmModule          *modules = 0;
    NvRmModuleInstance  *instance = 0;
    NvRmChipId          *id = 0;
    NvU32               bondOut[NVRM_MAX_BOND_OUT_REG] = {0, 0, 0};
    NvU32               j, i, k;
    const NvU32         *table = NULL;
    NvU8                *pb = NULL;
    NvU8                val;

    NV_ASSERT( hDevice );

    id = NvRmPrivGetChipId( hDevice );
    switch (id->Id)
    {
        case 0x15:
        case 0x16:
            NvRmPrivAp15GetBondOut(hDevice, &table, bondOut);
            break;
        case 0x20:
            NvRmPrivAp20GetBondOut(hDevice, &table, bondOut);
            break;
        default:
            return;     // no support
    }

    if ( !bondOut[0] && !bondOut[1] && !bondOut[2] )
        return;

    mod_table = NvRmPrivGetModuleTable( hDevice );
    modules = mod_table->Modules;

    for ( i = 0, j = 0; j < NVRM_MAX_BOND_OUT_REG; j++ )
    {
        if ( !bondOut[j] )
        {
            i += 32;        // skip full 32-bit
            continue;
        }
        pb = (NvU8 *)&bondOut[j];
        for ( k = 0; k < 4; k++ )
        {
            val = *pb++;
            if ( !val )
            {
                i += 8;
                continue;
            }
            for( ; ; )
            {
                if ( val & 1 )
                {
                    NvU32   moduleIdInst = table[i];
                    if ( NVRM_DEVICE_UNKNOWN != moduleIdInst )
                    {
                        if ( NvSuccess == NvRmPrivGetModuleInstance(hDevice,
                                                        moduleIdInst, &instance) )
                        {
                            /* Mark instance's DevIdx to invalid value -1.  if all
                               instances for the module are invalid, mark the module
                               itself INVALID.
                               Keep instance->DeviceId to maintain instance ordering
                               since we could be bonding out, say, UARTA but UARTB and
                               UARTC still available. */
                            NvRmModuleID moduleId =
                                            NVRM_MODULE_ID_MODULE( moduleIdInst );
                            instance->DevIdx = (NvU8)-1;
                            if (0 == NvRmModuleGetNumInstances( hDevice, moduleId ))
                                modules[moduleId].Index = NVRM_MODULE_INVALID;
                        }
                    }
                }
                val = val >> 1;     // Use ARM's clz?
                if ( !val )
                {
                    i = (i + 8) & ~7;       // skip to next byte
                    break;
                }
                i++;
            }
        }
    }
}
