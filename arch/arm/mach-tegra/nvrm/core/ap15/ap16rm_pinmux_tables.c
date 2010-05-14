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
#include "ap16/arapb_misc.h"
#include "ap15/arclk_rst.h"
#include "ap15rm_pinmux_utils.h"
#include "nvodm_query_pinmux.h"
#include "nvrm_clocks.h"

extern const NvU32 g_Ap15MuxI2c1[];
extern const NvU32 g_Ap15MuxI2c2[];
extern const NvU32* g_Ap15MuxI2c[];

extern const NvU32 g_Ap15MuxI2c_Pmu[];
extern const NvU32* g_Ap15MuxI2cPmu[];

extern const NvU32 g_Ap15Mux_Mmc[];
extern const NvU32* g_Ap15MuxMmc[];

extern const NvU32 g_Ap15MuxSdio2[];
extern const NvU32 g_Ap15MuxSdio3[];
extern const NvU32* g_Ap15MuxSdio[];

extern const NvU32 g_Ap15Mux_Spdif[];
extern const NvU32* g_Ap15MuxSpdif[];

static const NvU32 g_Ap16MuxUart1[] = {
    //  Reset config - abandon IRRX, IRTX &amp; SDD
    UNCONFIG(C,IRRX,UARTA,RSVD2), UNCONFIG(C,IRTX,UARTA,RSVD2), UNCONFIG(D,SDD,UARTA,PWM), CONFIGEND(),
    //  8b UAA + UAB pads
    CONFIG(B,A,UAA,UARTA), CONFIG(B,A,UAB,UARTA), CONFIGEND(),
    //  4b UAA pads
    CONFIG(B,A,UAA,UARTA), CONFIGEND(),
     //  7b GPU pads
    CONFIG(A,D,GPU,UARTA), CONFIGEND(),
    //  4b VFIR + UAD pads
    CONFIG(A,C,IRRX,UARTA), CONFIG(A,C,IRTX,UARTA), CONFIG(B,A,UAD,UARTA), CONFIGEND(),
    //  2b VFIR pads
    CONFIG(A,C,IRRX,UARTA), CONFIG(A,C,IRTX,UARTA), CONFIGEND(),
    //  2b SDIO pads
    CONFIG(B,D,SDD,UARTA), CONFIGEND(),
    MODULEDONE()
};
static const NvU32 g_Ap16MuxUart2[] = {
//  Reset config - abandon UAD. pads.chosen SFLASH pads
    UNCONFIG(A,UAD,IRDA,SFLASH), CONFIGEND(),
//  4b UAD + IRRX + IRTX pads
    CONFIG(B,A,UAD,IRDA), CONFIG(A,C,IRRX,UARTB), CONFIG(A,C,IRTX,UARTB), CONFIGEND(),
//..2b UAD pads
    CONFIG(B,A,UAD,IRDA), CONFIGEND(),
    MODULEDONE()
};

static const NvU32 g_Ap16MuxUart3[] = {
    //  Reset config - abandon UCA. chosen RSVD1
    UNCONFIG(B,UCA,UARTC,RSVD1), CONFIGEND(),
    //  4b UCA + UCB pads
    CONFIG(B,B,UCA,UARTC), CONFIG(B,B,UCB,UARTC), CONFIGEND(),
    //  2b UCA pads
    CONFIG(B,B,UCA,UARTC), CONFIGEND(),
    MODULEDONE()
};

static const NvU32* g_Ap16MuxUart[] = {
    &g_Ap16MuxUart1[0],
    &g_Ap16MuxUart2[0],
    &g_Ap16MuxUart3[0],
    NULL
};
extern const NvU32 g_Ap15MuxSpi1[];
extern const NvU32 g_Ap15MuxSpi2[];
extern const NvU32 g_Ap15MuxSpi3[];
extern const NvU32* g_Ap15MuxSpi[];

extern const NvU32 g_Ap15Mux_Sflash[];
extern const NvU32* g_Ap15MuxSflash[];

extern const NvU32 g_Ap15Mux_Twc[];
extern const NvU32* g_Ap15MuxTwc[];

extern const NvU32 g_Ap15Mux_Ata[];
extern const NvU32* g_Ap15MuxAta[];

extern const NvU32 g_Ap15Mux_Pwm[];
extern const NvU32* g_Ap15MuxPwm[];

extern const NvU32 g_Ap15Mux_Hsi[];
extern const NvU32 *g_Ap15MuxHsi[];

extern const NvU32 g_Ap15Mux_Nand[];
extern const NvU32* g_Ap15MuxNand[];

extern const NvU32 g_Ap15MuxDap1[];
extern const NvU32 g_Ap15MuxDap2[];
extern const NvU32 g_Ap15MuxDap3[];
extern const NvU32 g_Ap15MuxDap4[];
extern const NvU32* g_Ap15MuxDap[];

extern const NvU32 g_Ap15Mux_Kbc[];
extern const NvU32* g_Ap15MuxKbc[];

extern const NvU32 g_Ap15Mux_Hdcp[];
extern const NvU32* g_Ap15MuxHdcp[];

extern const NvU32 g_Ap15Mux_Hdmi[];
extern const NvU32* g_Ap15MuxHdmi[];

extern const NvU32 g_Ap15Mux_Mio[];
extern const NvU32* g_Ap15MuxMio[];

extern const NvU32 g_Ap15Mux_Slink[];
extern const NvU32* g_Ap15MuxSlink[];

extern const NvU32 g_Ap15Mux_Vi[];
extern const NvU32* g_Ap15MuxVi[];

extern const NvU32 g_Ap15Mux_Crt[];
extern const NvU32* g_Ap15MuxCrt[];

extern const NvU32 g_Ap15Mux_Display1[];
extern const NvU32 g_Ap15Mux_Display2[];
extern const NvU32* g_Ap15MuxDisplay[];

extern const NvU32 g_Ap15Mux_Cdev1[];

extern const NvU32 g_Ap15Mux_Cdev2[];
extern const NvU32 g_Ap15Mux_Csus[];
extern const NvU32* g_Ap15MuxCdev[];

extern const NvU32 g_Ap15Mux_BacklightDisplay1Pwm0[];
extern const NvU32 g_Ap15Mux_BacklightDisplay1Pwm1[];
extern const NvU32 g_Ap15Mux_BacklightDisplay2Pwm0[];
extern const NvU32 g_Ap15Mux_BacklightDisplay2Pwm1[];
extern const NvU32* g_Ap15MuxBacklight[];

static const NvU32 g_Ap16Mux_Ulpi[] = {
    CONFIGEND(), // no pad groups reset to ULPI, so nothing to disown for reset config
    CONFIG(B,A,UAA,ULPI), CONFIG(B,A,UAB,ULPI), CONFIG(B,A,UAC,ULPI), CONFIGEND(),
    MODULEDONE()
};
static const NvU32* g_Ap16MuxUlpi[] = {
    &g_Ap16Mux_Ulpi[0],
    NULL
};
/*  Array of all the controller types in the system, pointing to the array of
 *  instances of each controller.  Indexed using the NvRmIoModule value.
 */
static const NvU32** g_Ap16MuxControllers[] = {
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
    &g_Ap16MuxUart[0],
    NULL, //  no options for USB
    NULL, //  no options for VDD
    &g_Ap15MuxVi[0],
    NULL, //  no options for XIO
    &g_Ap15MuxCdev[0],
    &g_Ap16MuxUlpi[0],
    NULL,
    NULL,
    NULL,
    NULL,
    NULL, //  no options for TSENSor
    &g_Ap15MuxBacklight[0],
};

NV_CT_ASSERT(NV_ARRAY_SIZE(g_Ap16MuxControllers)==NvOdmIoModule_Num);

const NvU32***
NvRmAp16GetPinMuxConfigs(NvRmDeviceHandle hDevice)
{
    NV_ASSERT(hDevice);
    return (const NvU32***) g_Ap16MuxControllers;
}

NvBool NvRmPrivAp16RmModuleToOdmModule(
    NvRmModuleID RmModule,
    NvOdmIoModule *OdmModule,
    NvU32 *OdmInstance,
    NvU32 *pCnt)
{
    NvRmModuleID Module = NVRM_MODULE_ID_MODULE(RmModule);
    NvU32 Instance = NVRM_MODULE_ID_INSTANCE(RmModule);

    *OdmInstance = Instance;

    switch (Module)
    {
    case NvRmModuleID_Usb2Otg:
        if (Instance == 0)
        {
            *OdmModule = NvOdmIoModule_Usb;
            *OdmInstance = 0;
        }
        else
        {
            // stop here for instance otherthan one
            NV_ASSERT(Instance == 1);
            *OdmModule = NvOdmIoModule_Ulpi;
            *OdmInstance = 0;
        }
        *pCnt = 1;
        return NV_TRUE;
    default:
        break;
    }

    return NvRmPrivAp15RmModuleToOdmModule(RmModule,
               OdmModule, OdmInstance, pCnt);
}


NvError
NvRmPrivAp16GetModuleInterfaceCaps(
    NvOdmIoModule Module,
    NvU32 Instance,
    NvU32 PinMap,
    void *pCaps)
{
    switch (Module)
    {
        case NvOdmIoModule_Uart:
            if (Instance == 0)
            {
                if (PinMap == NvOdmUartPinMap_Config1)
                    ((NvRmModuleUartInterfaceCaps *)pCaps)->NumberOfInterfaceLines = 8;
                else if (PinMap == NvOdmUartPinMap_Config3)
                    ((NvRmModuleUartInterfaceCaps *)pCaps)->NumberOfInterfaceLines = 7;
                else if ((PinMap == NvOdmUartPinMap_Config2) || (PinMap == NvOdmUartPinMap_Config4))
                    ((NvRmModuleUartInterfaceCaps *)pCaps)->NumberOfInterfaceLines = 4;
                else if ((PinMap == NvOdmUartPinMap_Config5) || (PinMap == NvOdmUartPinMap_Config6))
                    ((NvRmModuleUartInterfaceCaps *)pCaps)->NumberOfInterfaceLines = 2;
                else
                    ((NvRmModuleUartInterfaceCaps *)pCaps)->NumberOfInterfaceLines = 0;
            }
            else if (Instance == 1)
            {
                if (PinMap == NvOdmUartPinMap_Config1)
                    ((NvRmModuleUartInterfaceCaps *)pCaps)->NumberOfInterfaceLines = 4;
                else if (PinMap == NvOdmUartPinMap_Config2)
                    ((NvRmModuleUartInterfaceCaps *)pCaps)->NumberOfInterfaceLines = 2;
                else
                    ((NvRmModuleUartInterfaceCaps *)pCaps)->NumberOfInterfaceLines = 0;
            }
            else if (Instance == 2)
            {
                if (PinMap == NvOdmUartPinMap_Config1)
                    ((NvRmModuleUartInterfaceCaps *)pCaps)->NumberOfInterfaceLines = 4;
                else if (PinMap == NvOdmUartPinMap_Config2)
                    ((NvRmModuleUartInterfaceCaps *)pCaps)->NumberOfInterfaceLines = 2;
                else
                    ((NvRmModuleUartInterfaceCaps *)pCaps)->NumberOfInterfaceLines = 0;
            }
            else
            {
                NV_ASSERT(NV_FALSE);
                return NvError_NotSupported;
            }
            return NvSuccess;

        default:
            break;
    }
    return NvRmPrivAp15GetModuleInterfaceCaps(Module, Instance, PinMap, pCaps);
}

