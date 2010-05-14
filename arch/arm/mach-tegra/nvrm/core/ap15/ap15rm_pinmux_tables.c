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
#include "nvrm_pinmux.h"
#include "nvrm_drf.h"
#include "nvassert.h"
#include "nvrm_hwintf.h"
#include "ap15rm_private.h"
#include "ap15/arapb_misc.h"
#include "ap15/arclk_rst.h"
#include "ap15rm_pinmux_utils.h"
#include "nvodm_query_pinmux.h"
#include "nvrm_clocks.h"

/**
 *  Each of the pin mux configurations defined in the pin mux spreadsheet are
 *  stored in tables below.  For each configuration, every pad group that
 *  must be programmed is stored as a single 32b entry, where the register
 *  offset (for both the tristate and pin mux control registers), field bit
 *  position (ditto), pin mux mask, and new pin mux state are programmed.
 *
 *  Furthermore, a simple state machine is implemented, so that pin mux
 *  registers can be "unprogrammed," in order to disown pad groups which
 *  may be pointing to a controller which is about to be programmed.  The
 *  state machine also has no-op states which indicate when all necessary
 *  register programming for a configuration is complete, as well as when the
 *  last configuration for a module instance has been reached.
 *
 *  Each module instance array has a reserved "reset" configuration at index
 *  zero.  This special configuration is used in order to disown all pad
 *  groups whose reset state refers to the module instance.  When a module
 *  instance configuration is to be applied, the reset configuration will
 *  first be applied, to ensure that no conflicts will arise between register
 *  reset values and the new configuration, followed by the application of
 *  the requested configuration.
 *
 *  Furthermore, for controllers which support dynamic pinmuxing (i.e.,
 *  the "Multiplexed" pin map option), the last table entry is reserved for
 *  a "global unset," which will ensure that all configurations are disowned.
 *  This Multiplexed configuration should be applied before transitioning
 *  from one configuration to a second one.
 *
 *  The table data has been packed into a single 32b entry to minimize code
 *  footprint using macros similar to the hardware register definitions, so
 *  that all of the shift and mask operations can be performed with the DRF
 *  macros.
 */

/*  Below are the tables for all of the pin mux configurations for each
 *  controller.  The first (zero-index) entry in each table is a "reset"
 *  configuration.  This is used to disown all pads whose reset state
 *  corresponds to the controller function.  When a new configuration is
 *  applied, the driver will first apply the reset configuration to ensure
 *  that no conflicts will occur due to identical signals being routed to
 *  multiple pad groups.
 */

const NvU32 g_Ap15MuxI2c1[] = {
    //  Reset config -- disown GEN1_I2C pads
    UNCONFIG(A, RM,I2C, RSVD1), CONFIGEND(),
    //  I2C1, Config 1 (GEN1_I2C pads)
    CONFIG(A,A,RM,I2C), CONFIGEND(),
    //  I2C1, Config 2 (SPDIF pads) -- disown GEN1_I2C pads
    CONFIG(B,D,SPDO,I2C), CONFIG(B,D,SPDI,I2C), CONFIGEND(),
    //  I2C1, Config 3 (SPI2 pads)
    CONFIG(B,D,SPIG,I2C),CONFIG(B,D,SPIH,I2C), CONFIGEND(),
    MODULEDONE()
};

/*  I2C_2 instance 1 supports dynamic pin-muxing for CAM_I2C and GEN2_I2C;
 *  PinMap_Multiplex is intended to release all pads to a nominal
 *  state, so it is implemented at the end of the list using UNCONFIG
 *  options, so that no pad groups are trying to use I2C_2.
 */
const NvU32 g_Ap15MuxI2c2[] = {
    //  Reset & multiplexed config -- disown GEN2_I2C2 pads
    UNCONFIG(G,PTA,I2C2,RSVD1),UNCONFIG(G,DTF,I2C2,RSVD1),UNCONFIG(E,LVP0,I2C2,RSVD),
    UNCONFIG(E,LM1,I2C2,DISPLAYA),UNCONFIG(G,LHP0,I2C2,DISPLAYA),
    UNCONFIG(G,LVP1,I2C2,DISPLAYA),CONFIGEND(),
    //  CAM_I2C pads
    CONFIG(D,G,DTF,I2C2), CONFIGEND(),
    //  GEN2_I2C pads
    CONFIG(A,G,PTA,I2C2), CONFIGEND(),
    //  LCD control pads
    CONFIG(C,E,LVP0,I2C2), CONFIG(C,E,LM1,I2C2), CONFIGEND(),
    //  alternate LCD control pads
    CONFIG(C,G,LHP0,I2C2), CONFIG(C,G,LVP1,I2C2), CONFIGEND(),
    MODULEDONE()
};

const NvU32* g_Ap15MuxI2c[] = {
    &g_Ap15MuxI2c1[0],
    &g_Ap15MuxI2c2[0],
    NULL
};

const NvU32 g_Ap15MuxI2c_Pmu[] = {
    //  Reset config -- disown I2CP pads
    UNCONFIG(C,I2CP,I2C, RSVD2), CONFIGEND(),
    //  I2CP pads
    CONFIG(A,C,I2CP,I2C), CONFIGEND(),
    MODULEDONE()
};

const NvU32* g_Ap15MuxI2cPmu[] = {
    &g_Ap15MuxI2c_Pmu[0],
    NULL
};

const NvU32 g_Ap15Mux_Mmc[] = {
    CONFIGEND(), // no pad groups reset to MMC, so nothing to disown for reset config
    CONFIG(A,A,ATB,HSMMC), CONFIG(A,A,ATD,HSMMC), CONFIG(B,A,ATE,HSMMC), CONFIGEND(),
    CONFIG(A,A,ATB,HSMMC),CONFIG(A,A,ATD,HSMMC),CONFIGEND(),
    MODULEDONE()
};

const NvU32* g_Ap15MuxMmc[] = {
    &g_Ap15Mux_Mmc[0],
    NULL
};

const NvU32 g_Ap15MuxSdio2[] = {
    //  Reset config - abandon SDB, SLXK,SLXA,SLXB,SLXC,SLXD .chosen RSVD,SLINK4B
    UNCONFIG(D,SDB,SDIO2,RSVD), UNCONFIG(B,SLXK,SDIO1,SLINK4B), UNCONFIG(B,SLXB,SDIO1,SLINK4B),
    UNCONFIG(B,SLXC,SDIO1,SLINK4B),UNCONFIG(B,SLXD,SDIO1,SLINK4B),UNCONFIG(B,SLXA,SDIO1,SLINK4B),
    CONFIGEND(),
    //  config 1 SDB + SLXK,SLXA,SLXB,SLXC,SLXD pads
    CONFIG(B,D,SDB,SDIO2), CONFIG(B,B,SLXK,SDIO1), CONFIG(B,B,SLXB,SDIO1),
    CONFIG(B,B,SLXC,SDIO1), CONFIG(B,B,SLXD,SDIO1), CONFIG(B,B,SLXA,SDIO1),CONFIGEND(),
    // config 2 KBCB,KBCE,KBCD pads
    CONFIG(A,C,KBCB,SDIO1),CONFIG(A,A,KBCE,SDIO1),CONFIG(D,G,KBCD,SDIO1),
    CONFIGEND(),
    //config 3   KBCB pads
    CONFIG(A,C,KBCB,SDIO1), CONFIGEND(),
    //  config 4 DAP1, SPDO, SPDI pads
    CONFIG(A,C,DAP1,SDIO1), CONFIG(B,D,SPDO,SDIO1), CONFIG(B,D,SPDI,SDIO1), CONFIGEND(),
    //  config 5  DTA,DTD pads
    CONFIG(A,B,DTA,SDIO1), CONFIG(A,B,DTD,SDIO1), CONFIGEND(),
    MODULEDONE()
};

const NvU32 g_Ap15MuxSdio3[] = {
    // no pad groups reset to SDIO3, so nothing to disown for reset config
    CONFIGEND(),
    //  config1  SDD + SDC+SLXK+SLXA+SLXB pads
    CONFIG(B,D,SDD,SDIO2), CONFIG(B,D,SDC,SDIO2), CONFIG(B,D,SDB,SDIO2_ALT),
    CONFIG(B,B,SLXA,SDIO2), CONFIG(B,B,SLXK,SDIO2), CONFIG(B,B,SLXB,SDIO2), CONFIGEND(),
    //  congig 2 SDD, SDC  pads
    CONFIG(B,D,SDD,SDIO2), CONFIG(B,D,SDC,SDIO2), CONFIGEND(),
    MODULEDONE()
};

const NvU32* g_Ap15MuxSdio[] = {
    &g_Ap15MuxSdio2[0],
    &g_Ap15MuxSdio3[0],
    NULL
};

const NvU32 g_Ap15Mux_Spdif[] = {
    //  Reset config - abandon SPDO, SPDI .chosen RSVD.
    UNCONFIG(D,SPDO,SPDIF,RSVD), UNCONFIG(D,SPDI,SPDIF,RSVD),CONFIGEND(),
    //  config1  SPDO+ SPDI pads
    CONFIG(B,D,SPDO,SPDIF), CONFIG(B,D,SPDI,SPDIF), CONFIGEND(),
    //  congig 2 SLXD, SLXC  pads
    CONFIG(B,B,SLXD,SPDIF), CONFIG(B,B,SLXC,SPDIF), CONFIGEND(),
    //  congig 3 UAD, pads
    CONFIG(B,A,UAD,SPDIF), CONFIGEND(),
    MODULEDONE()
};

const NvU32* g_Ap15MuxSpdif[] = {
    &g_Ap15Mux_Spdif[0],
    NULL
};

static const NvU32 g_Ap15MuxUart1[] = {
    //  Reset config - abandon IRRX, IRTX &amp; SDD
    UNCONFIG(C,IRRX,UARTA,RSVD2), UNCONFIG(C,IRTX,UARTA,RSVD2), UNCONFIG(D,SDD,UARTA,PWM), CONFIGEND(),
    //  8b UAA + UAB pads
    CONFIG(B,A,UAA,UARTA), CONFIG(B,A,UAB,UARTA), CONFIGEND(),
    //  4b UAA pads
    CONFIG(B,A,UAA,UARTA_ALT3), CONFIGEND(),
    //  8b GPU pads
    CONFIG(A,D,GPU,UARTA), CONFIGEND(),
    //  4b VFIR + UAD pads
    CONFIG(A,C,IRRX,UARTA), CONFIG(A,C,IRTX,UARTA), CONFIG(B,A,UAD,UARTA), CONFIGEND(),
    //  2b VFIR pads
    CONFIG(A,C,IRRX,UARTA), CONFIG(A,C,IRTX,UARTA), CONFIGEND(),
    //  2b SDIO pads
    CONFIG(B,D,SDD,UARTA), CONFIGEND(),
    MODULEDONE()
};

static const NvU32 g_Ap15MuxUart2[] = {
//  Reset config - abandon UAD. pads.chosen SFLASH pads
    UNCONFIG(A,UAD,IRDA,SFLASH), CONFIGEND(),
//  4b UAD + IRRX + IRTX pads
    CONFIG(B,A,UAD,IRDA), CONFIG(A,C,IRRX,UARTB), CONFIG(A,C,IRTX,UARTB), CONFIGEND(),
//  4b UAB pads
    CONFIG(B,A,UAB,UARTB), CONFIGEND(),
//..2b UAB pads
    CONFIG(B,A,UAD,IRDA), CONFIGEND(),
    MODULEDONE()
};

static const NvU32 g_Ap15MuxUart3[] = {
    //  Reset config - abandon UCA. chosen RSVD1
    UNCONFIG(B,UCA,UARTC,RSVD1), CONFIGEND(),
    //  4b UCA + UCB pads
    CONFIG(B,B,UCA,UARTC), CONFIG(B,B,UCB,UARTC), CONFIGEND(),
    //  2b UCA pads
    CONFIG(B,B,UCA,UARTC), CONFIGEND(),
    MODULEDONE()
};

static const NvU32* g_Ap15MuxUart[] = {
    &g_Ap15MuxUart1[0],
    &g_Ap15MuxUart2[0],
    &g_Ap15MuxUart3[0],
    NULL
};

const NvU32 g_Ap15MuxSpi1[] = {
    //  Reset config - abandon SPIC, SPIB, SPIA, pads.
    UNCONFIG(D,SPIC,SPI1,RSVD), UNCONFIG(D,SPIB,SPI1,RSVD),
    UNCONFIG(D,SPIA,SPI1,RSVD), CONFIGEND(),
    //  SPIE,SPIF,SPID pads
    CONFIG(B,D,SPIE,SPI1),CONFIG(B,D,SPIF,SPI1),CONFIG(B,D,SPID,SPI1), CONFIGEND(),
    //  DTE, DTB pads
    CONFIG(A,B,DTE,SPI1), CONFIG(A,B,DTB,SPI1), CONFIGEND(),
    //  SPIC,SPIB,SPIA pads
    CONFIG(B,D,SPIC,SPI1), CONFIG(B,D,SPIB,SPI1), CONFIG(B,D,SPIA,SPI1), CONFIGEND(),
    //  LHP2,LHP1,LHP0,LVP1,LDI,LPP pads
    CONFIG(C,G,LHP2,SPI1), CONFIG(C,G,LHP1,SPI1), CONFIG(C,G,LHP0,SPI1),
    CONFIG(C,G,LVP1,SPI1), CONFIG(D,G,LDI,SPI1), CONFIG(D,G,LPP,SPI1), CONFIGEND(),
    MODULEDONE()
};

const NvU32 g_Ap15MuxSpi2[] = {
    //  Reset config - abandon UAB, pads.  MIPI_HS chosen
    UNCONFIG(A,UAB,SPI2,MIPI_HS), UNCONFIG(D,SPID,SPI2,RSVD),
    UNCONFIG(D,SPIE,SPI2,RSVD), CONFIGEND(),
    //..SPIC,SPIB,SPIA,SPIG, SPIH Pads
    CONFIG(B,D,SPIC,SPI2), CONFIG(B,D,SPIB,SPI2), CONFIG(B,D,SPIA,SPI2),
    CONFIG(B,D,SPIG,SPI2), CONFIG(B,D,SPIH,SPI2), CONFIGEND(),
    //  UAB pads
    CONFIG(B,A,UAB,SPI2), CONFIGEND(),
    //  SPIE,SPIF,SPID,SPIG,SPIH pads
    CONFIG(B,D,SPIE,SPI2_ALT),CONFIG(B,D,SPIF,SPI2),CONFIG(B,D,SPID,SPI2_ALT),
    CONFIG(B,D,SPIG,SPI2_ALT),CONFIG(B,D,SPIH,SPI2_ALT), CONFIGEND(),
    //  SLXC,SLXK,SLXA,SLXB,SLXD pads
    CONFIG(B,B,SLXC,SPI2), CONFIG(B,B,SLXK,SPI2), CONFIG(B,B,SLXA,SPI2),
    CONFIG(B,B,SLXB,SPI2),CONFIG(B,B,SLXD,SPI2), CONFIGEND(),
    MODULEDONE()
};

/*  SPI instance 3 supports dynamic pin-muxing for audio-codec &amp;
 *  display, PinMap_Multiplex is intended to release all pads to a nominal
 *  state, so it is implemented at the end of the list using UNCONFIG
 *  options, so that no pad groups are trying to use SPI3.
 */
const NvU32 g_Ap15MuxSpi3[] = {
/*  Reset config - abandon UAA, SPIF, SPIG, SPIH pads.  SPI2_ALT chosen
 *  as the reset state for SPIG/SPIH, since this will either be clobbered
 *  by Spi2 SpiPinMap_Config1, I2c1 I2cPinMap_Config3, correct (for Spi2
 *  SpiPinMap_Config3), or irrelevant */
    UNCONFIG(A,UAA,SPI3,MIPI_HS), UNCONFIG(D,SPIF,SPI3,RSVD),
    UNCONFIG(D,SPIG,SPI3,SPI2_ALT), UNCONFIG(D,SPIH,SPI3,SPI2_ALT),
    //  multiplex unconfiguration
    UNCONFIG(C,XM2A,SPI3,SPROM), // multiplex config 1 to SPROM
    UNCONFIG(E,LSC1,SPI3,DISPLAYA), UNCONFIG(E,LPW2,SPI3,DISPLAYA), // mux config 2 to displaya
    UNCONFIG(E,LPW0,SPI3,DISPLAYA), UNCONFIG(E,LM0,SPI3,DISPLAYA),
    UNCONFIG(E,LSCK,SPI3,DISPLAYA), UNCONFIG(E,LSDI,SPI3,DISPLAYA), // mux config 3 to displaya
    UNCONFIG(D,SPIC,SPI3,RSVD),UNCONFIG(D,SPIB,SPI3,RSVD), // config 5 to rsvd
    UNCONFIG(D,SPIA,SPI3,RSVD),
    UNCONFIG(D,SDD,SPI3,PWM),UNCONFIG(D,SDC,SPI3,TWC), // config 6 to PWM & TWC
    CONFIGEND(),
    //  XM2A pads
    CONFIG(B,C,XM2A,SPI3), CONFIGEND(),
    //  LCD pads
    CONFIG(C,E,LSC1,SPI3), CONFIG(D,E,LPW2,SPI3), CONFIG(D,E,LPW0,SPI3), CONFIG(C,E,LM0,SPI3), CONFIGEND(),
    //  Alternate LCD pads
    CONFIG(C,E,LSCK,SPI3), CONFIG(D,E,LSDI,SPI3), CONFIG(D,E,LSDA,SPI3), CONFIG(C,E,LCSN,SPI3), CONFIGEND(),
    //  UAA pads
    CONFIG(B,A,UAA,SPI3), CONFIGEND(),
    //  SPI pads
    CONFIG(B,D,SPIA,SPI3), CONFIG(B,D,SPIB,SPI3), CONFIG(B,D,SPIC,SPI3), CONFIGEND(),
    //  2CS SPI3 on SDIO pads
    CONFIG(B,D,SDC,SPI3), CONFIG(B,D,SDD,SPI3), CONFIGEND(),
    MODULEDONE()
};

const NvU32* g_Ap15MuxSpi[] = {
    &g_Ap15MuxSpi1[0],
    &g_Ap15MuxSpi2[0],
    &g_Ap15MuxSpi3[0],
    NULL
};

//  Sflash should always be after PWM in the module order, since
//  the reset value for UCB muxes from both controllers, so the
//  reset configuration for Sflash assumes that Pwm has executed first.
NV_CT_ASSERT((NvU32)NvOdmIoModule_Sflash > (NvU32)NvOdmIoModule_Pwm);

const NvU32 g_Ap15Mux_Sflash[] = {
    /*  Reset config.  Normally, this would disown the UCB pads; HOWEVER,
     *  the reset value for this pad group actually muxes from 2 controllers:
     *  PWM goes to UART3_RTS, and SFLASH goes to UART3_CTS.  Since the PWM
     *  controller is initialized before Spi Flash, it is possible for the
     *  UCB pads to be correctly configured to mux 0 before reaching here.
     *  Therefore, the correct thing to do is to skip the UNCONFIG for this
     *  pad group, since PWM will already handle this.
     */
    /*UNCONFIG(B,UCB,PWM0,RSVD2),*/ CONFIGEND(),
    //  config 1 XM2S + XM2A pads
    CONFIG(B,C,XM2S,SPI), CONFIG(B,C,XM2A,SPI), CONFIGEND(),
    //  config2 XM2S + UAD +XM2A pads
    CONFIG(B,C,XM2S,SPI), CONFIG(B,A,UAD,SFLASH), CONFIG(B,C,XM2A,SPI), CONFIGEND(),
    //  config 3 XM2S + UCB +XM2A pads
    CONFIG(B,C,XM2S,SPI), CONFIG(B,B,UCB,PWM0), CONFIG(B,C,XM2A,SPI), CONFIGEND(),
    //  config 4 XM2A UAD UCB XM2A pads
    CONFIG(B,C,XM2S,SPI), CONFIG(B,A,UAD,SFLASH), CONFIG(B,B,UCB,PWM0),
    CONFIG(B,C,XM2A,SPI), CONFIGEND(),
    MODULEDONE()
};

const NvU32* g_Ap15MuxSflash[] = {
    &g_Ap15Mux_Sflash[0],
    NULL
};


const NvU32 g_Ap15Mux_Twc[] = {
    // no pad groups reset to TWC, so nothing to disown for reset config
     CONFIGEND(),
    //  DAP2 pads
    CONFIG(A,C,DAP2,TWC), CONFIGEND(),
    //  SDC pads
    CONFIG(B,D,SDC,TWC), CONFIGEND(),
    MODULEDONE()
};

const NvU32* g_Ap15MuxTwc[] = {
    &g_Ap15Mux_Twc[0],
    NULL
};

const NvU32 g_Ap15Mux_Ata[] = {
    // Reset config --  abandon ATA, ATC, ATB, ATD, ATE pads. NAND RSVD as chosenpads
    UNCONFIG(A,ATC,IDE,RSVD), UNCONFIG(A,ATD,IDE,NAND), UNCONFIG(A,ATE,IDE,NAND),
    UNCONFIG(A,ATA,IDE,RSVD), UNCONFIG(A,ATB,IDE,NAND), CONFIGEND(),
    // ATA, Config 1 (Nand pads)
    CONFIG(A,A,ATC,IDE), CONFIG(A,A,ATD,IDE), CONFIG(B,A,ATE,IDE), CONFIG(A,A,ATA,IDE),
    CONFIG(A,A,ATB,IDE), CONFIGEND(),
    MODULEDONE()
};

const NvU32* g_Ap15MuxAta[] = {
    &g_Ap15Mux_Ata[0],
    NULL
};

const NvU32 g_Ap15Mux_Pwm[] = {
    // Reset config -- disown SDC,UCB pads SDIO2, RSVD2 as chosen pads
    UNCONFIG(D,SDC,PWM,SDIO2), UNCONFIG(B,UCB,PWM0,RSVD2), CONFIGEND(),
    // PWM, Config 1 (SDC pads)
    CONFIG(B,D,SDC,PWM), CONFIGEND(),
    // PWM, Config 2 (UCB ,SDDpads)
    CONFIG(B,B,UCB,PWM0), CONFIG(B,D,SDD,PWM), CONFIGEND(),
    // PWM, Config 2 (UCB ,SDDpads)
    CONFIG(B,B,UCB,PWM0), CONFIGEND(),
    CONFIG(B,D,SDD,PWM), CONFIGEND(),
    MODULEDONE()
};

const NvU32* g_Ap15MuxPwm[] = {
    &g_Ap15Mux_Pwm[0],
    NULL
};

const NvU32 g_Ap15Mux_Hsi[] = {
    CONFIGEND(), // no pad groups reset to HSI, so nothing to disown for reset config
    CONFIG(B,A,UAA,MIPI_HS), CONFIG(B,A,UAB,MIPI_HS), CONFIGEND(),
    MODULEDONE()
};

const NvU32 *g_Ap15MuxHsi[] = {
    &g_Ap15Mux_Hsi[0],
    NULL
};

const NvU32 g_Ap15Mux_Nand[] = {
    CONFIGEND(), // no pad groups reset to NAND, so nothing to disown for reset config
    //  config 1 ATA,ATB,ATC,ATD,ATE pads
    CONFIG(A,A,ATA,NAND_ALT), CONFIG(A,A,ATB,NAND_ALT), CONFIG(A,A,ATC,NAND),
    CONFIG(A,A,ATD,NAND), CONFIG(B,A,ATE,NAND), CONFIGEND(),
    //  config 1 ATA,ATB,ATC,ATD,ATE pads
    CONFIG(A,A,ATA,NAND), CONFIG(A,A,ATB,NAND), CONFIG(A,A,ATC,NAND),
    CONFIG(A,A,ATD,NAND), CONFIG(B,A,ATE,NAND), CONFIGEND(),
    //  config 1 ATA,ATC,ATE pads
    CONFIG(A,A,ATA,NAND), CONFIG(A,A,ATC,NAND),
    CONFIG(B,A,ATE,NAND_ALT), CONFIGEND(),
    //  config 1 ATA,ATB,ATC,ATD,ATE pads
    CONFIG(A,A,ATA,NAND), CONFIG(A,A,ATB,NAND), CONFIG(A,A,ATC,NAND),
    CONFIG(A,A,ATD,NAND_ALT), CONFIG(B,A,ATE,NAND_ALT), CONFIGEND(),
    //  config 1 ATA,ATC pads
    CONFIG(A,A,ATA,NAND), CONFIG(A,A,ATC,NAND), CONFIGEND(),
    //  config 1 ATA,ATB,ATC pads
    CONFIG(A,A,ATA,NAND), CONFIG(A,A,ATB,NAND),
    CONFIG(A,A,ATC,NAND), CONFIGEND(),
    MODULEDONE()
};

const NvU32* g_Ap15MuxNand[] = {
    &g_Ap15Mux_Nand[0],
    NULL
};

const NvU32 g_Ap15MuxDap1[] = {
    //  Reset config - abandon ,DAP1.. RSVD2 chosen
    UNCONFIG(C,DAP1,DAP1,RSVD2), CONFIGEND(),
    //  config1  DAP1 pads
    CONFIG(A,C,DAP1,DAP1), CONFIGEND(),
    MODULEDONE()
};

const NvU32 g_Ap15MuxDap2[] = {
    //  Reset config - abandon ,DAP2... RSVD3 chosen
    UNCONFIG(C,DAP2,DAP2,RSVD3), CONFIGEND(),
    //  config1  DAP2 pads
    CONFIG(A,C,DAP2,DAP2), CONFIGEND(),
    //  congig 2 SLXD, SLXC  pads
    MODULEDONE()
};

const NvU32 g_Ap15MuxDap3[] = {
    //  Reset config - abandon ,DAP3... RSVD2 chosen
    UNCONFIG(C,DAP3,DAP3,RSVD2), CONFIGEND(),
    //  config1  DAP3 pads
    CONFIG(A,C,DAP3,DAP3), CONFIGEND(),
    MODULEDONE()
};

const NvU32 g_Ap15MuxDap4[] = {
    //  Reset config - abandon ,DAP4...RSVD2 chosen
    UNCONFIG(C,DAP4,DAP4,RSVD2), CONFIGEND(),
    //  config1  DAP4 pads
    CONFIG(A,C,DAP4,DAP4), CONFIGEND(),
    MODULEDONE()
};

const NvU32* g_Ap15MuxDap[] = {
    &g_Ap15MuxDap1[0],
    &g_Ap15MuxDap2[0],
    &g_Ap15MuxDap3[0],
    &g_Ap15MuxDap4[0],
    NULL
};

const NvU32 g_Ap15Mux_Kbc[] = {
    //  Reset config - abandon ,RSVD2, RSVD1 chosen
    UNCONFIG(C,KBCA,KBC,RSVD2), UNCONFIG(C,KBCB,KBC,RSVD2), UNCONFIG(A,KBCE,KBC,RSVD1),
    UNCONFIG(C,KBCC,KBC,RSVD2), UNCONFIG(G,KBCD,KBC,RSVD2), UNCONFIG(A,KBCF,KBC,RSVD1), CONFIGEND(),
    //  KBCA,KBCB,KBCC,KBCD,KBCE,KBCF  pads
    CONFIG(A,C,KBCA,KBC), CONFIG(A,C,KBCB,KBC), CONFIG(A,A,KBCE,KBC),
    CONFIG(B,C,KBCC,KBC), CONFIG(D,G,KBCD,KBC), CONFIG(A,A,KBCF,KBC), CONFIGEND(),
    //  KBCA,KBCC,KBCD,KBCE,KBCF  pads
    CONFIG(A,C,KBCA,KBC), CONFIG(A,A,KBCE,KBC),
    CONFIG(B,C,KBCC,KBC), CONFIG(D,G,KBCD,KBC), CONFIG(A,A,KBCF,KBC), CONFIGEND(),
    //  KBCA,KBCC,KBCF,  pads
    CONFIG(A,C,KBCA,KBC), CONFIG(B,C,KBCC,KBC), CONFIG(A,A,KBCF,KBC), CONFIGEND(),
    //  KBCA,KBCC  pads
    CONFIG(A,C,KBCA,KBC), CONFIG(B,C,KBCC,KBC), CONFIGEND(),
    MODULEDONE()
};

const NvU32* g_Ap15MuxKbc[] = {
    &g_Ap15Mux_Kbc[0],
    NULL
};

NvU32 g_Ap15Mux_Hdcp[] = {
    CONFIGEND(), // no pad groups reset to HDCP, so nothing to disown for reset config
    CONFIG(A,G,PTA,HDMI), CONFIGEND(),
    CONFIG(C,E,LSCK,HDMI), CONFIG(D,E,LSDA,HDMI), CONFIGEND(),
    CONFIG(D,E,LPW2,HDMI), CONFIG(D,E,LPW0,HDMI), CONFIGEND(),
    CONFIG(C,E,LSC1,HDMI), CONFIG(D,E,LPW0,HDMI), CONFIGEND(),
    MODULEDONE()
};

const NvU32* g_Ap15MuxHdcp[] = {
    &g_Ap15Mux_Hdcp[0],
    NULL
};

const NvU32 g_Ap15Mux_Hdmi[] = {
    //  HDINT resets to HDINT, so move it to a reserved pin
    UNCONFIG(B,HDINT,RSVD1,RSVD2), CONFIGEND(),
    CONFIG(C,B,HDINT,RSVD1), CONFIGEND(),
    MODULEDONE()
};

const NvU32* g_Ap15MuxHdmi[] = {
    &g_Ap15Mux_Hdmi[0],
    NULL
};

const NvU32 g_Ap15Mux_Mio[] = {
    CONFIGEND(), // no pad groups reset to MIO, so nothing to disown for reset config
    CONFIG(A,A,KBCF,MIO), CONFIG(D,G,KBCD,MIO), CONFIG(A,C,KBCB,MIO), CONFIGEND(),
    MODULEDONE()
};

const NvU32* g_Ap15MuxMio[] = {
    &g_Ap15Mux_Mio[0],
    NULL
};

const NvU32 g_Ap15Mux_Slink[] = {
    CONFIGEND(), // no pad groups reset to SLINK, so nothing to disown for reset config
    CONFIG(B,B,SLXK,SLINK4B), CONFIG(B,B,SLXA,SLINK4B), CONFIG(B,B,SLXB,SLINK4B),
    CONFIG(B,B,SLXC,SLINK4B), CONFIG(B,B,SLXD,SLINK4B), CONFIGEND(),
    MODULEDONE()
};

const NvU32* g_Ap15MuxSlink[] = {
    &g_Ap15Mux_Slink[0],
    NULL
};

const NvU32 g_Ap15Mux_Vi[] = {
    CONFIGEND(), // no pad groups reset to VI so nothing to disown for reset config
    //  config 1 DTA - DTF pads
    BRANCH(NvOdmVideoInputPinMap_Config2), CONFIG(D,G,DTF,VI), CONFIGEND(),
    //  config 2 DTA - DTE and CSUS pads
    CONFIG(A,B,DTA,VI), CONFIG(A,B,DTB,VI), CONFIG(A,B,DTC,VI),
    CONFIG(A,B,DTD,VI), CONFIG(A,B,DTE,VI), CONFIGEND(),
    MODULEDONE(),
    SUBROUTINESDONE(),
};

const NvU32* g_Ap15MuxVi[] = {
    &g_Ap15Mux_Vi[0],
    NULL
};

const NvU32 g_Ap15Mux_Crt[] = {
    //  Need  to confirm and fix it ,but none of docs specifies about tv pad group
    CONFIGEND(), // no pad groups reset to CRT so nothing to disown for reset config
    //  config 1 LHS, LVS,  pads
    CONFIG(D,E,LHS,CRT), CONFIG(C,E,LVS,CRT), CONFIGEND(),
    //  config 2 LHP2,LPW1  pads
    CONFIG(C,G,LHP2,CRT), CONFIG(D,E,LPW1,CRT), CONFIGEND(),
    //  config 3 LM1,LPW1  pads
    CONFIG(C,E,LM1,CRT), CONFIG(D,E,LPW1,CRT), CONFIGEND(),
    //  config 4 LHP2,LCSN  pads
    CONFIG(C,G,LHP2,CRT), CONFIG(C,E,LCSN,CRT), CONFIGEND(),
    MODULEDONE()
};

const NvU32* g_Ap15MuxCrt[] = {
    &g_Ap15Mux_Crt[0],
    NULL
};

const NvU32 g_Ap15Mux_BacklightDisplay1Pwm0[] = {
    CONFIGEND(),
    // Config 1 LPW0 pad
    CONFIG(D,E,LPW0,DISPLAYA), CONFIGEND(),
    // Config 2 LPW2 pad
    CONFIG(D,E,LPW2,DISPLAYA), CONFIGEND(),
    // Config 3 LM0 pad
    CONFIG(C,E,LM0,DISPLAYA), CONFIGEND(),
    MODULEDONE()
};

const NvU32 g_Ap15Mux_BacklightDisplay1Pwm1[] = {
    CONFIGEND(),
    // Config 1 LM1 pad
    CONFIG(C,E,LM1,DISPLAYA), CONFIGEND(),
    // Config 2 LDC pad
    CONFIG(C,E,LDC,DISPLAYA), CONFIGEND(),
    // Config 3 LPW1 pad
    CONFIG(D,E,LPW1,DISPLAYA), CONFIGEND(),
    MODULEDONE()
};

const NvU32 g_Ap15Mux_BacklightDisplay2Pwm0[] = {
    CONFIGEND(),
    // Config 1 LPW0 pad
    CONFIG(D,E,LPW0,DISPLAYB), CONFIGEND(),
    // Config 2 LPW2 pad
    CONFIG(D,E,LPW2,DISPLAYB), CONFIGEND(),
    // Config 3 LM0 pad
    CONFIG(C,E,LM0,DISPLAYB), CONFIGEND(),
    MODULEDONE()
};

const NvU32 g_Ap15Mux_BacklightDisplay2Pwm1[] = {
    CONFIGEND(),
    // Config 1 LM1 pad
    CONFIG(C,E,LM1,DISPLAYB), CONFIGEND(),
    // Config 2 LDC pad
    CONFIG(C,E,LDC,DISPLAYB), CONFIGEND(),
    // Config 3 LPW1 pad
    CONFIG(D,E,LPW1,DISPLAYB), CONFIGEND(),
    MODULEDONE()
};

const NvU32* g_Ap15MuxBacklight[] = {
    &g_Ap15Mux_BacklightDisplay1Pwm0[0],
    &g_Ap15Mux_BacklightDisplay1Pwm1[0],
    &g_Ap15Mux_BacklightDisplay2Pwm0[0],
    &g_Ap15Mux_BacklightDisplay2Pwm1[0],
    NULL
};

const NvU32 g_Ap15Mux_Display1[] = {
    CONFIGEND(),
    //  config 1, 24b RGB.  Pure superset of Config2 (18b RGB)
    BRANCH(2),
    CONFIG(C,G,LHP1,DISPLAYA),CONFIG(C,G,LHP2,DISPLAYA),CONFIG(C,G,LVP1,DISPLAYA),
    CONFIG(C,G,LHP0,DISPLAYA),CONFIG(D,G,LDI,DISPLAYA),CONFIG(D,G,LPP,DISPLAYA),
    CONFIGEND(),
    //  config 2, 18b RGB.
    BRANCH(7),
    CONFIG(C,E,LVS,DISPLAYA), CONFIG(D,E,LHS,DISPLAYA), CONFIG(D,E,LSPI,DISPLAYA),
    CONFIGEND(),
    // config 3, 8 & 9b CPU.
    CONFIG(C,G,LHP1,DISPLAYA), CONFIG(C,G,LHP2,DISPLAYA), CONFIG(C,G,LVP1,DISPLAYA),
    CONFIG(C,G,LHP0,DISPLAYA), CONFIG(D,G,LDI,DISPLAYA), CONFIG(D,G,LPP,DISPLAYA),
    CONFIG(D,E,LPW0,DISPLAYA), CONFIG(D,E,LPW1,DISPLAYA), CONFIG(D,E,LPW2,DISPLAYA),
    CONFIG(C,E,LSC1,DISPLAYA), CONFIG(C,E,LM1,DISPLAYA),
    CONFIG(C,E,LVP0,DISPLAYA), CONFIGEND(),
    // config 4.  SPI
    CONFIG(D,E,LPW0,DISPLAYA), CONFIG(D,E,LPW2,DISPLAYA), CONFIG(C,E,LSC1,DISPLAYA),
    CONFIG(C,E,LM0,DISPLAYA), CONFIG(C,E,LVP0,DISPLAYA), CONFIGEND(),
    // Config 5. Panel 86
    BRANCH(7),CONFIG(C,E,LSC1,DISPLAYA),CONFIG(C,E,LM1,DISPLAYA),CONFIGEND(),
    // config 6. 16/18b smart panels
    BRANCH(7),CONFIG(C,E,LDC,DISPLAYA),CONFIG(D,E,LSPI,DISPLAYA),CONFIGEND(),
    MODULEDONE(),
    //  subroutine 1. - 18b data + clock
    CONFIG(C,F,LD0,DISPLAYA), CONFIG(C,F,LD1,DISPLAYA), CONFIG(C,F,LD2,DISPLAYA),
    CONFIG(C,F,LD3,DISPLAYA), CONFIG(C,F,LD4,DISPLAYA), CONFIG(C,F,LD5,DISPLAYA),
    CONFIG(C,F,LD6,DISPLAYA), CONFIG(C,F,LD7,DISPLAYA), CONFIG(C,F,LD8,DISPLAYA),
    CONFIG(C,F,LD9,DISPLAYA), CONFIG(C,F,LD10,DISPLAYA), CONFIG(C,F,LD11,DISPLAYA),
    CONFIG(C,F,LD12,DISPLAYA), CONFIG(C,F,LD13,DISPLAYA), CONFIG(C,F,LD14,DISPLAYA),
    CONFIG(C,F,LD15,DISPLAYA), CONFIG(C,G,LD16,DISPLAYA), CONFIG(C,G,LD17,DISPLAYA),
    CONFIG(C,E,LSC0,DISPLAYA), CONFIGEND(),
    SUBROUTINESDONE(),  // This is required, since BRANCH is used.
/* For handy reference, here is the complete list of CONFIG macros for the display
   pad groups, in case any more configurations are defined in the future.
    CONFIG(C,F,LD0,DISPLAYA), CONFIG(C,F,LD1,DISPLAYA), CONFIG(C,F,LD2,DISPLAYA),
    CONFIG(C,F,LD3,DISPLAYA), CONFIG(C,F,LD4,DISPLAYA), CONFIG(C,F,LD5,DISPLAYA),
    CONFIG(C,F,LD6,DISPLAYA), CONFIG(C,F,LD7,DISPLAYA), CONFIG(C,F,LD8,DISPLAYA),
    CONFIG(C,F,LD9,DISPLAYA), CONFIG(C,F,LD10,DISPLAYA), CONFIG(C,F,LD11,DISPLAYA),
    CONFIG(C,F,LD12,DISPLAYA),
    CONFIG(C,F,LD13,DISPLAYA), CONFIG(C,F,LD14,DISPLAYA), CONFIG(C,F,LD15,DISPLAYA),
    CONFIG(C,G,LD16,DISPLAYA), CONFIG(C,G,LD17,DISPLAYA),CONFIG(C,E,LSC0,DISPLAYA),
    CONFIG(C,E,LVS,DISPLAYA), CONFIG(D,E,LHS,DISPLAYA), CONFIG(D,E,LSPI,DISPLAYA),
    CONFIG(C,G,LHP1,DISPLAYA), CONFIG(C,G,LHP2,DISPLAYA), CONFIG(C,G,LHP0,DISPLAYA),
    CONFIG(C,G,LVP1,DISPLAYA), CONFIG(D,G,LDI,DISPLAYA), CONFIG(D,G,LPP,DISPLAYA),
    CONFIG(C,E,LCSN,DISPLAYA), CONFIG(C,E,LM1,DISPLAYA),CONFIG(C,E,LM0,DISPLAYA),
    CONFIG(D,E,LPW0,DISPLAYA),CONFIG(D,E,LPW2,DISPLAYA), CONFIG(D,E,LPW1,DISPLAYA),
    CONFIG(C,E,LVP0,DISPLAYA), CONFIG(C,E,LDC,DISPLAYA), CONFIG(C,E,LSC1,DISPLAYA),
    CONFIG(D,E,LSDI,DISPLAYA),
    */
};

const NvU32 g_Ap15Mux_Display2[] = {
    CONFIGEND(),
    //  config 1, 24b RGB.  Pure superset of Config2 (18b RGB)
    BRANCH(2),
    CONFIG(C,G,LHP1,DISPLAYB),CONFIG(C,G,LHP2,DISPLAYB),CONFIG(C,G,LVP1,DISPLAYB),
    CONFIG(C,G,LHP0,DISPLAYB),CONFIG(D,G,LDI,DISPLAYB),CONFIG(D,G,LPP,DISPLAYB),
    CONFIGEND(),
    //  config 2, 18b RGB.
    BRANCH(7),
    CONFIG(C,E,LVS,DISPLAYB), CONFIG(D,E,LHS,DISPLAYB), CONFIG(D,E,LSPI,DISPLAYB),
    CONFIGEND(),
    // config 3, 8 & 9b CPU.
    CONFIG(C,G,LHP1,DISPLAYB), CONFIG(C,G,LHP2,DISPLAYB), CONFIG(C,G,LVP1,DISPLAYB),
    CONFIG(C,G,LHP0,DISPLAYB), CONFIG(D,G,LDI,DISPLAYB), CONFIG(D,G,LPP,DISPLAYB),
    CONFIG(D,E,LPW0,DISPLAYB), CONFIG(D,E,LPW1,DISPLAYB), CONFIG(D,E,LPW2,DISPLAYB),
    CONFIG(C,E,LSC1,DISPLAYB), CONFIG(C,E,LM1,DISPLAYB),
    CONFIG(C,E,LVP0,DISPLAYB), CONFIGEND(),
    // config 4.  SPI
    CONFIG(D,E,LPW0,DISPLAYB), CONFIG(D,E,LPW2,DISPLAYB), CONFIG(C,E,LSC1,DISPLAYB),
    CONFIG(C,E,LM0,DISPLAYB), CONFIG(C,E,LVP0,DISPLAYB), CONFIGEND(),
    // Config 5. USed only for Sony VGA panel
    BRANCH(7),CONFIG(C,E,LSC1,DISPLAYB),CONFIG(C,E,LM1,DISPLAYB),CONFIGEND(),
    // config 6. 16/18b smart panels
    BRANCH(7),CONFIG(C,E,LDC,DISPLAYB),CONFIG(D,E,LSPI,DISPLAYB),CONFIGEND(),
    MODULEDONE(),
    //  subroutine 1. (config 7)
    CONFIG(C,F,LD0,DISPLAYB), CONFIG(C,F,LD1,DISPLAYB), CONFIG(C,F,LD2,DISPLAYB),
    CONFIG(C,F,LD3,DISPLAYB), CONFIG(C,F,LD4,DISPLAYB), CONFIG(C,F,LD5,DISPLAYB),
    CONFIG(C,F,LD6,DISPLAYB), CONFIG(C,F,LD7,DISPLAYB), CONFIG(C,F,LD8,DISPLAYB),
    CONFIG(C,F,LD9,DISPLAYB), CONFIG(C,F,LD10,DISPLAYB), CONFIG(C,F,LD11,DISPLAYB),
    CONFIG(C,F,LD12,DISPLAYB), CONFIG(C,F,LD13,DISPLAYB), CONFIG(C,F,LD14,DISPLAYB),
    CONFIG(C,F,LD15,DISPLAYB), CONFIG(C,G,LD16,DISPLAYB), CONFIG(C,G,LD17,DISPLAYB),
    CONFIG(C,E,LSC0,DISPLAYB), CONFIGEND(),
    SUBROUTINESDONE(),
};

const NvU32* g_Ap15MuxDisplay[] = {
    &g_Ap15Mux_Display1[0],
    &g_Ap15Mux_Display2[0],
    NULL
};

const NvU32 g_Ap15Mux_Cdev1[] = {
    //  reset config - no-op
    CONFIGEND(),
    CONFIG(A,C,CDEV1,PLLA_OUT), CONFIGEND(),
    CONFIG(A,C,CDEV1,OSC), CONFIGEND(),
    MODULEDONE()
};

const NvU32 g_Ap15Mux_Cdev2[] = {
    CONFIGEND(),
    CONFIG(A,C,CDEV2,AHB_CLK), CONFIGEND(),
    CONFIG(A,C,CDEV2,OSC), CONFIGEND(),
    MODULEDONE()
};

const NvU32 g_Ap15Mux_Csus[] = {
    CONFIGEND(),
    CONFIG(A,C,CSUS,VI_SENSOR_CLK),  CONFIGEND(),
    MODULEDONE()
};

const NvU32* g_Ap15MuxCdev[] =
{
    &g_Ap15Mux_Cdev1[0],
    &g_Ap15Mux_Cdev2[0],
    &g_Ap15Mux_Csus[0],
    NULL
};

/*  Array of all the controller types in the system, pointing to the array of
 *  instances of each controller.  Indexed using the NvRmIoModule value.
 */
static const NvU32** g_Ap15MuxControllers[] = {
    &g_Ap15MuxAta[0],
    &g_Ap15MuxCrt[0],
    NULL, // no options for CSI
    &g_Ap15MuxDap[0],
    &g_Ap15MuxDisplay[0],
    NULL, // no options for DSI
    NULL, // no options for GPIO
    &g_Ap15MuxHdcp[0],
    &g_Ap15MuxHdmi[0],
    &g_Ap15MuxHsi[0],
    &g_Ap15MuxMmc[0],
    NULL, // no options for I2S
    &g_Ap15MuxI2c[0],
    &g_Ap15MuxI2cPmu[0],
    &g_Ap15MuxKbc[0],
    &g_Ap15MuxMio[0],
    &g_Ap15MuxNand[0],
    &g_Ap15MuxPwm[0],
    &g_Ap15MuxSdio[0],
    &g_Ap15MuxSflash[0],
    &g_Ap15MuxSlink[0],
    &g_Ap15MuxSpdif[0],
    &g_Ap15MuxSpi[0],
    &g_Ap15MuxTwc[0],
    NULL, //  no options for TVO
    &g_Ap15MuxUart[0],
    NULL, //  no options for USB
    NULL, //  no options for VDD
    &g_Ap15MuxVi[0],
    NULL, //  no options for XIO
    &g_Ap15MuxCdev[0],
    NULL, //  no options for Ulpi
    NULL, //  no options for one wire
    NULL, //  no options for sync NOR
    NULL, //  no options for PCI-E
    NULL, //  no options for ETM
    NULL, //  no options for TSENSor
    &g_Ap15MuxBacklight[0],
};

NV_CT_ASSERT(NV_ARRAY_SIZE(g_Ap15MuxControllers)==NvOdmIoModule_Num);

const NvU32***
NvRmAp15GetPinMuxConfigs(NvRmDeviceHandle hDevice)
{
    NV_ASSERT(hDevice);
    return (const NvU32***) g_Ap15MuxControllers;
}

/* Define the GPIO port/pin to tristate mappings */

const NvU16 g_Ap15GpioPadGroupMapping[] =
{
    //  Port A
    GPIO_TRISTATE(B,SDB), GPIO_TRISTATE(B,UCB), GPIO_TRISTATE(A,DAP2), GPIO_TRISTATE(A,DAP2),
    GPIO_TRISTATE(A,DAP2), GPIO_TRISTATE(A,DAP2), GPIO_TRISTATE(B,SDD), GPIO_TRISTATE(B,SDD),
    //  Port B
    GPIO_TRISTATE(B,XM2A), GPIO_TRISTATE(B,XM2A), GPIO_TRISTATE(D,LPW0), GPIO_TRISTATE(C,LSC0),
    GPIO_TRISTATE(B,SDC), GPIO_TRISTATE(B,SDC), GPIO_TRISTATE(B,SDC), GPIO_TRISTATE(B,SDC),
    //  Port C
    GPIO_TRISTATE(B,UCB), GPIO_TRISTATE(D,LPW1), GPIO_TRISTATE(B,UAD), GPIO_TRISTATE(B,UAD),
    GPIO_TRISTATE(A,RM), GPIO_TRISTATE(A,RM), GPIO_TRISTATE(D,LPW2), GPIO_TRISTATE(B,XM2C),
    //  Port D
    GPIO_TRISTATE(B,SLXK), GPIO_TRISTATE(B,SLXA), GPIO_TRISTATE(B,SLXB), GPIO_TRISTATE(B,SLXC),
    GPIO_TRISTATE(B,SLXD), GPIO_TRISTATE(A,DTA), GPIO_TRISTATE(A,DTC), GPIO_TRISTATE(A,DTC),
    //  Port E
    GPIO_TRISTATE(C,LD0), GPIO_TRISTATE(C,LD1), GPIO_TRISTATE(C,LD2), GPIO_TRISTATE(C,LD3),
    GPIO_TRISTATE(C,LD4), GPIO_TRISTATE(C,LD5), GPIO_TRISTATE(C,LD6), GPIO_TRISTATE(C,LD7),
    //  Port F
    GPIO_TRISTATE(C, LD8), GPIO_TRISTATE(C,LD9), GPIO_TRISTATE(C,LD10), GPIO_TRISTATE(C,LD11),
    GPIO_TRISTATE(C, LD12), GPIO_TRISTATE(C,LD13), GPIO_TRISTATE(C, LD14), GPIO_TRISTATE(C,LD15),
    //  Port G
    GPIO_TRISTATE(A,ATC), GPIO_TRISTATE(A,ATC),GPIO_TRISTATE(A,ATC), GPIO_TRISTATE(A,ATC),
    GPIO_TRISTATE(A,ATC), GPIO_TRISTATE(A,ATC),GPIO_TRISTATE(A,ATC), GPIO_TRISTATE(A,ATC),
    //  Port H
    GPIO_TRISTATE(A,ATD), GPIO_TRISTATE(A,ATD),GPIO_TRISTATE(A,ATD), GPIO_TRISTATE(A,ATD),
    GPIO_TRISTATE(B,ATE), GPIO_TRISTATE(B,ATE),GPIO_TRISTATE(B,ATE), GPIO_TRISTATE(B,ATE),
    //  Port I
    GPIO_TRISTATE(A,ATC), GPIO_TRISTATE(A,ATC), GPIO_TRISTATE(A,ATA), GPIO_TRISTATE(A,ATA),
    GPIO_TRISTATE(A,ATA), GPIO_TRISTATE(A,ATB), GPIO_TRISTATE(A,ATB), GPIO_TRISTATE(A,ATC),
    //  Port J
    GPIO_TRISTATE(B,XM2S), GPIO_TRISTATE(D,LSPI), GPIO_TRISTATE(B,XM2S), GPIO_TRISTATE(D,LHS),
    GPIO_TRISTATE(C,LVS), GPIO_TRISTATE(A,IRTX), GPIO_TRISTATE(A,IRRX), GPIO_TRISTATE(B,XM2A),
    //  Port K
    GPIO_TRISTATE(A,ATC), GPIO_TRISTATE(A,ATC), GPIO_TRISTATE(A,ATC), GPIO_TRISTATE(A,ATC),
    GPIO_TRISTATE(A,ATC), GPIO_TRISTATE(B,SPDO), GPIO_TRISTATE(B,SPDI), GPIO_TRISTATE(B,XM2A),
    //  Port L
    GPIO_TRISTATE(A,DTD), GPIO_TRISTATE(A,DTD), GPIO_TRISTATE(A,DTD), GPIO_TRISTATE(A,DTD),
    GPIO_TRISTATE(A,DTD), GPIO_TRISTATE(A,DTD), GPIO_TRISTATE(A,DTD), GPIO_TRISTATE(A,DTD),
    //  Port M
    GPIO_TRISTATE(C,LD16), GPIO_TRISTATE(C,LD17), GPIO_TRISTATE(C,LHP1), GPIO_TRISTATE(C,LHP2),
    GPIO_TRISTATE(C,LVP1), GPIO_TRISTATE(C,LHP0), GPIO_TRISTATE(D,LDI), GPIO_TRISTATE(D,LPP),
    //  Port N
    GPIO_TRISTATE(A,DAP1), GPIO_TRISTATE(A,DAP1), GPIO_TRISTATE(A,DAP1), GPIO_TRISTATE(A,DAP1),
    GPIO_TRISTATE(C,LCSN), GPIO_TRISTATE(D,LSDA), GPIO_TRISTATE(C,LDC), GPIO_TRISTATE(C,HDINT),
    //  Port O
    GPIO_TRISTATE(B,UAB), GPIO_TRISTATE(B,UAA), GPIO_TRISTATE(B,UAA), GPIO_TRISTATE(B,UAA),
    GPIO_TRISTATE(B,UAA), GPIO_TRISTATE(B,UAB), GPIO_TRISTATE(B,UAB), GPIO_TRISTATE(B,UAB),
    //  Port P
    GPIO_TRISTATE(A,DAP3), GPIO_TRISTATE(A,DAP3), GPIO_TRISTATE(A,DAP3), GPIO_TRISTATE(A,DAP3),
    GPIO_TRISTATE(A,DAP4), GPIO_TRISTATE(A,DAP4), GPIO_TRISTATE(A,DAP4), GPIO_TRISTATE(A,DAP4),
    //  Port Q
    GPIO_TRISTATE(A,KBCF), GPIO_TRISTATE(A,KBCF), GPIO_TRISTATE(A,KBCF), GPIO_TRISTATE(A,KBCF),
    GPIO_TRISTATE(A,PMC), GPIO_TRISTATE(A,PMC), GPIO_TRISTATE(A,I2CP), GPIO_TRISTATE(A,I2CP),
    //  Port R
    GPIO_TRISTATE(A,KBCA), GPIO_TRISTATE(A,KBCA), GPIO_TRISTATE(A,KBCA), GPIO_TRISTATE(A,KBCE),
    GPIO_TRISTATE(D,KBCD), GPIO_TRISTATE(D,KBCD), GPIO_TRISTATE(D,KBCD), GPIO_TRISTATE(A,KBCB),
    //  Port S
    GPIO_TRISTATE(A,KBCB), GPIO_TRISTATE(A,KBCB), GPIO_TRISTATE(A,KBCB), GPIO_TRISTATE(A,KBCB),
    GPIO_TRISTATE(A,KBCB), GPIO_TRISTATE(B,KBCC), GPIO_TRISTATE(B,KBCC), GPIO_TRISTATE(B,KBCC),
    //  Port T
    GPIO_TRISTATE(A,DTD), GPIO_TRISTATE(A,CSUS), GPIO_TRISTATE(A,DTB), GPIO_TRISTATE(A,DTB),
    GPIO_TRISTATE(A,PTA), GPIO_TRISTATE(A,PTA), GPIO_TRISTATE(A,PTA), GPIO_TRISTATE(A,PTA),
    //  Port U
    GPIO_TRISTATE(A,GPU), GPIO_TRISTATE(A,GPU), GPIO_TRISTATE(A,GPU), GPIO_TRISTATE(A,GPU),
    GPIO_TRISTATE(A,GPU), GPIO_TRISTATE(A,GPU), GPIO_TRISTATE(A,GPU), GPIO_TRISTATE(D,GPU7),
    //  Port V
    GPIO_TRISTATE(B,UAC), GPIO_TRISTATE(B,UAC), GPIO_TRISTATE(B,UAC), GPIO_TRISTATE(B,UAC),
    GPIO_TRISTATE(A,GPV), GPIO_TRISTATE(A,GPV), GPIO_TRISTATE(A,GPV), GPIO_TRISTATE(C,LVP0),
    //  Port W
    GPIO_TRISTATE(C,LM0), GPIO_TRISTATE(C,LM1), GPIO_TRISTATE(B,SPIG), GPIO_TRISTATE(B,SPIH),
    GPIO_TRISTATE(A,CDEV1), GPIO_TRISTATE(A,CDEV2), GPIO_TRISTATE(B,UCA), GPIO_TRISTATE(B,UCA),
    //  Port X
    GPIO_TRISTATE(B,SPIA), GPIO_TRISTATE(B,SPIB), GPIO_TRISTATE(B,SPIC), GPIO_TRISTATE(B,SPIC),
    GPIO_TRISTATE(B,SPID), GPIO_TRISTATE(B,SPIE), GPIO_TRISTATE(B,SPIE), GPIO_TRISTATE(B,SPIF)
};

NvBool
NvRmAp15GetPinGroupForGpio(NvRmDeviceHandle hDevice,
                           NvU32 Port,
                           NvU32 Pin,
                           NvU32 *pMapping)
{
    const NvU32 GpiosPerPort = 8;
    NvU32 Index = Port*GpiosPerPort + Pin;

    if ((Pin >= GpiosPerPort) || (Index >= NV_ARRAY_SIZE(g_Ap15GpioPadGroupMapping)))
        return NV_FALSE;

    *pMapping = (NvU32)g_Ap15GpioPadGroupMapping[Index];
    return NV_TRUE;
}

// Top level AP15 clock enable register control macro
#define CLOCK_ENABLE( rm, offset, field, EnableState ) \
    do { \
        regaddr = (CLK_RST_CONTROLLER_##offset##_0); \
        NvOsMutexLock((rm)->CarMutex); \
        reg = NV_REGR((rm), NvRmPrivModuleID_ClockAndReset, 0, regaddr); \
        reg = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, offset, field, EnableState, reg); \
        NV_REGW((rm), NvRmPrivModuleID_ClockAndReset, 0, regaddr, reg); \
        NvOsMutexUnlock((rm)->CarMutex); \
    } while( 0 )

void NvRmPrivAp15EnableExternalClockSource(
    NvRmDeviceHandle hDevice,
    const NvU32* Instance,
    NvU32 Config,
    NvBool ClockState)
{
    NvU32 MuxCtlShift, MuxCtlSet;
    NvU32 reg;
    NvU32 regaddr;

    MuxCtlShift = NV_DRF_VAL(MUX,ENTRY, MUX_CTL_SHIFT, *Instance);
    MuxCtlSet = NV_DRF_VAL(MUX,ENTRY, MUX_CTL_SET, *Instance);

    if (MuxCtlShift == APB_MISC_PP_PIN_MUX_CTL_C_0_CDEV1_SEL_SHIFT)
    {
        if (MuxCtlSet == APB_MISC_PP_PIN_MUX_CTL_C_0_CDEV1_SEL_PLLA_OUT)
        {
            NvRmPrivExternalClockAttach(
                hDevice, NvRmClockSource_PllA0, ClockState);
        }
        CLOCK_ENABLE(hDevice, MISC_CLK_ENB, CLK_ENB_DEV1_OUT, ClockState);
    }
    else if (MuxCtlShift == APB_MISC_PP_PIN_MUX_CTL_C_0_CDEV2_SEL_SHIFT)
    {
        CLOCK_ENABLE(hDevice, MISC_CLK_ENB, CLK_ENB_DEV2_OUT, ClockState);
    }
    else if (MuxCtlShift == APB_MISC_PP_PIN_MUX_CTL_C_0_CSUS_SEL_SHIFT)
    {
        CLOCK_ENABLE(hDevice, MISC_CLK_ENB, CLK_ENB_SUS_OUT, ClockState);
    }
}

NvU32
NvRmPrivAp15GetExternalClockSourceFreq(
    NvRmDeviceHandle hDevice,
    const NvU32* Instance,
    NvU32 Config)
{
    NvU32 MuxCtlShift, MuxCtlSet;
    NvU32 ClockFreqInKHz = 0;

    MuxCtlShift = NV_DRF_VAL(MUX,ENTRY, MUX_CTL_SHIFT, *Instance);
    MuxCtlSet = NV_DRF_VAL(MUX,ENTRY, MUX_CTL_SET, *Instance);

    if (MuxCtlShift == APB_MISC_PP_PIN_MUX_CTL_C_0_CDEV1_SEL_SHIFT)
    {
        if (MuxCtlSet == APB_MISC_PP_PIN_MUX_CTL_C_0_CDEV1_SEL_PLLA_OUT)
            ClockFreqInKHz = NvRmPrivGetClockSourceFreq(NvRmClockSource_PllA0);

        else if (MuxCtlSet == APB_MISC_PP_PIN_MUX_CTL_C_0_CDEV1_SEL_OSC)
            ClockFreqInKHz = NvRmPrivGetClockSourceFreq(NvRmClockSource_ClkM);
    }
    else if (MuxCtlShift == APB_MISC_PP_PIN_MUX_CTL_C_0_CDEV2_SEL_SHIFT)
    {
        if (MuxCtlSet == APB_MISC_PP_PIN_MUX_CTL_C_0_CDEV2_SEL_AHB_CLK)
            ClockFreqInKHz = NvRmPrivGetClockSourceFreq(NvRmClockSource_Ahb);

        else if (MuxCtlSet == APB_MISC_PP_PIN_MUX_CTL_C_0_CDEV2_SEL_OSC)
            ClockFreqInKHz = NvRmPrivGetClockSourceFreq(NvRmClockSource_ClkM);
    }
    else if (MuxCtlShift == APB_MISC_PP_PIN_MUX_CTL_C_0_CSUS_SEL_SHIFT)
    {
        if (MuxCtlSet == APB_MISC_PP_PIN_MUX_CTL_C_0_CSUS_SEL_VI_SENSOR_CLK)
        {
            if (NvRmPowerModuleClockConfig(hDevice, NvRmModuleID_Vi, 0, 0, 0,
                NULL, 0, &ClockFreqInKHz, NvRmClockConfig_SubConfig) != NvSuccess)
            {
                ClockFreqInKHz = 0;
            }
        }
    }
    return ClockFreqInKHz;
}

/*  These functions will map from the RM's internal definition of module
 *  instances to the ODM definition.  Since the RM is controller-centric,
 *  and the ODM pin mux query is interface-centric, the mapping is not
 *  always one-to-one */

NvBool NvRmPrivAp15RmModuleToOdmModule(
    NvRmModuleID RmModule,
    NvOdmIoModule *OdmModule,
    NvU32 *OdmInstance,
    NvU32 *pCnt)
{
    NvRmModuleID Module = NVRM_MODULE_ID_MODULE(RmModule);

    switch (Module)
    {
    case NvRmPrivModuleID_Mio_Exio:
        *OdmModule = NvOdmIoModule_Mio;
        *OdmInstance = 0; // since there is only one MIO bus on AP15/AP16.
        *pCnt = 1;
        return NV_TRUE;
    default:
        break;
    }

    return NV_FALSE;
}

NvError
NvRmPrivAp15GetModuleInterfaceCaps(
    NvOdmIoModule Module,
    NvU32 Instance,
    NvU32 PinMap,
    void *pCaps)
{
    NvError err = NvError_NotSupported;

    switch (Module)
    {
        case NvOdmIoModule_Sdio:
        {
            NvRmModuleSdmmcInterfaceCaps *pSdmmcCaps =
                (NvRmModuleSdmmcInterfaceCaps *)pCaps;
            if (Instance==0 &&
                (PinMap == NvOdmSdioPinMap_Config2 ||
                 PinMap == NvOdmSdioPinMap_Config5))
                pSdmmcCaps->MmcInterfaceWidth = 8;
            else if (Instance==1 && PinMap==NvOdmSdioPinMap_Config1)
                pSdmmcCaps->MmcInterfaceWidth = 8;
            else
                pSdmmcCaps->MmcInterfaceWidth = 4;
            err = NvSuccess;
            break;
        }
        case NvOdmIoModule_Hsmmc:
        {
            NvRmModuleSdmmcInterfaceCaps *pSdmmcCaps =
                (NvRmModuleSdmmcInterfaceCaps *)pCaps;
            if (Instance==0 && PinMap==NvOdmHsmmcPinMap_Config2)
                pSdmmcCaps->MmcInterfaceWidth = 4;
            else
                pSdmmcCaps->MmcInterfaceWidth = 8;
            err = NvSuccess;
            break;
        }
        case NvOdmIoModule_Pwm:
        {
            NvRmModulePwmInterfaceCaps *pPwmCaps =
                (NvRmModulePwmInterfaceCaps *)pCaps;
            err = NvSuccess;
            if (Instance == 0 && (PinMap == NvOdmPwmPinMap_Config1))
                pPwmCaps->PwmOutputIdSupported = 15;
            else if (Instance == 0 && (PinMap == NvOdmPwmPinMap_Config2))
                pPwmCaps->PwmOutputIdSupported = 13;
            else if (Instance == 0 && (PinMap == NvOdmPwmPinMap_Config3))
                pPwmCaps->PwmOutputIdSupported = 1;
            else if (Instance == 0 && (PinMap == NvOdmPwmPinMap_Config4))
                pPwmCaps->PwmOutputIdSupported = 12;
            else
            {
                pPwmCaps->PwmOutputIdSupported = 0;
                err = NvError_NotSupported;
            }
            break;
        }
        case NvOdmIoModule_Nand:
        {
            NvRmModuleNandInterfaceCaps *pNandCaps =
                (NvRmModuleNandInterfaceCaps *)pCaps;
            if (Instance == 0)
            {
                pNandCaps->IsCombRbsyMode = NV_TRUE;
                pNandCaps->NandInterfaceWidth = 8;

                if (PinMap == NvOdmNandPinMap_Config4)
                    pNandCaps->IsCombRbsyMode = NV_FALSE;

                if ((PinMap == NvOdmNandPinMap_Config1) ||
                    (PinMap == NvOdmNandPinMap_Config2))
                    pNandCaps->NandInterfaceWidth = 16;

                err =  NvSuccess;
            }
            else
            {
                NV_ASSERT(NV_FALSE);
                err = NvError_NotSupported;
            }
            break;
        }
        case NvOdmIoModule_Uart:
        {
            NvRmModuleUartInterfaceCaps *pUartCaps =
                (NvRmModuleUartInterfaceCaps *)pCaps;
            err =  NvSuccess;
            if (Instance == 0)
            {
                if (PinMap == NvOdmUartPinMap_Config1)
                    pUartCaps->NumberOfInterfaceLines = 8;
                else if (PinMap == NvOdmUartPinMap_Config3)
                    pUartCaps->NumberOfInterfaceLines = 7;
                else if ((PinMap == NvOdmUartPinMap_Config2) ||
                         (PinMap == NvOdmUartPinMap_Config4))
                    pUartCaps->NumberOfInterfaceLines = 4;
                else if ((PinMap == NvOdmUartPinMap_Config5) ||
                         (PinMap == NvOdmUartPinMap_Config6))
                    pUartCaps->NumberOfInterfaceLines = 2;
                else
                    pUartCaps->NumberOfInterfaceLines = 0;
            }
            else if (Instance == 1)
            {
                if ((PinMap == NvOdmUartPinMap_Config1) ||
                    (PinMap == NvOdmUartPinMap_Config2))
                    pUartCaps->NumberOfInterfaceLines = 4;
                else if (PinMap == NvOdmUartPinMap_Config3)
                    pUartCaps->NumberOfInterfaceLines = 2;
                else
                    pUartCaps->NumberOfInterfaceLines = 0;
            }
            else if (Instance == 2)
            {
                if (PinMap == NvOdmUartPinMap_Config1)
                    pUartCaps->NumberOfInterfaceLines = 4;
                else if (PinMap == NvOdmUartPinMap_Config2)
                    pUartCaps->NumberOfInterfaceLines = 2;
                else
                    pUartCaps->NumberOfInterfaceLines = 0;
            }
            else
            {
                NV_ASSERT(NV_FALSE);
                err = NvError_NotSupported;
            }
            break;
        }
        default:
            break;
    }
    return err;
}

NvError
NvRmAp15GetStraps(
    NvRmDeviceHandle hDevice,
    NvRmStrapGroup StrapGroup,
    NvU32* pStrapValue)
{
    NvU32 reg = NV_REGR(
        hDevice, NvRmModuleID_Misc, 0, APB_MISC_PP_STRAPPING_OPT_A_0);

    switch (StrapGroup)
    {
        case NvRmStrapGroup_RamCode:
            reg = NV_DRF_VAL(APB_MISC_PP, STRAPPING_OPT_A, RAM_CODE, reg);
            break;
        default:
            return NvError_NotSupported;
    }
    *pStrapValue = reg;
    return NvSuccess;
}

