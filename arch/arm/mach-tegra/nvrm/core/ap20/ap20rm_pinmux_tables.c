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
#include "ap20/arapb_misc.h"
#include "ap20/arclk_rst.h"
#include "ap15/ap15rm_pinmux_utils.h"
#include "nvrm_clocks.h"
#include "nvodm_query_pinmux.h"

NvU32
NvRmPrivAp20GetExternalClockSourceFreq(
    NvRmDeviceHandle hDevice,
    struct tegra_pingroup_config *pin_config,
    int len)
{
	NvU32 ClockFreqInKHz = 0;
	if (len != 1)
		return 0;

	switch(pin_config[0].pingroup) {
	case TEGRA_PINGROUP_CDEV1:
		if (pin_config[0].func == TEGRA_MUX_PLLA_OUT)
		    ClockFreqInKHz = NvRmPrivGetClockSourceFreq(NvRmClockSource_PllA0);
		else if (pin_config[0].func == TEGRA_MUX_OSC)
		    ClockFreqInKHz = NvRmPrivGetClockSourceFreq(NvRmClockSource_ClkM);
		break;

	case TEGRA_PINGROUP_CDEV2:
		if (pin_config[0].func == TEGRA_MUX_AHB_CLK)
		    ClockFreqInKHz = NvRmPrivGetClockSourceFreq(NvRmClockSource_Ahb);
		else if (pin_config[0].func == TEGRA_MUX_OSC)
		    ClockFreqInKHz = NvRmPrivGetClockSourceFreq(NvRmClockSource_ClkM);
		else if (pin_config[0].func == TEGRA_MUX_PLLP_OUT4)
		    ClockFreqInKHz = NvRmPrivGetClockSourceFreq(NvRmClockSource_PllP4);
		break;

	case TEGRA_PINGROUP_CSUS:
		if (pin_config[0].func == TEGRA_MUX_VI_SENSOR_CLK)
		{
		    if (NvRmPowerModuleClockConfig(hDevice, NvRmModuleID_Vi, 0, 0, 0,
			NULL, 0, &ClockFreqInKHz, NvRmClockConfig_SubConfig) != NvSuccess)
		    {
			ClockFreqInKHz = 0;
		    }
		}
		break;
	default:
		ClockFreqInKHz = 0;
		break;
    }

    return ClockFreqInKHz;
}

void NvRmPrivAp20EnableExternalClockSource(
    NvRmDeviceHandle hDevice,
    struct tegra_pingroup_config *pin_config,
    int len,
    NvBool ClockState)
{
    NvU32 ClkEnbShift = ~0;

    if (len != 1)
	    return;

    switch(pin_config[0].pingroup) {
    case TEGRA_PINGROUP_CDEV1:
	    ClkEnbShift = CLK_RST_CONTROLLER_CLK_ENB_U_SET_0_SET_CLK_ENB_DEV1_OUT_SHIFT;
	    if (pin_config[0].func == TEGRA_MUX_PLLA_OUT) {
		    NvRmPrivExternalClockAttach(
			hDevice, NvRmClockSource_PllA0, ClockState);
	    }
	    break;

     case TEGRA_PINGROUP_CDEV2:
	     ClkEnbShift = CLK_RST_CONTROLLER_CLK_ENB_U_SET_0_SET_CLK_ENB_DEV2_OUT_SHIFT;
	     if (pin_config[0].func == TEGRA_MUX_PLLP_OUT4)
	     {
		 NvRmPrivExternalClockAttach(
		     hDevice, NvRmClockSource_PllP4, ClockState);
	     }
	     break;

    case TEGRA_PINGROUP_CSUS:
	    ClkEnbShift = CLK_RST_CONTROLLER_CLK_ENB_U_SET_0_SET_CLK_ENB_SUS_OUT_SHIFT;
	    break;
    default:
	return;
    }
    if (ClockState)
    {
        NV_REGW(hDevice, NvRmPrivModuleID_ClockAndReset, 0,
                CLK_RST_CONTROLLER_CLK_ENB_U_SET_0, (1UL<<ClkEnbShift));
    }
    else
    {
        NV_REGW(hDevice, NvRmPrivModuleID_ClockAndReset, 0,
                CLK_RST_CONTROLLER_CLK_ENB_U_CLR_0, (1UL<<ClkEnbShift));
    }
}

NvBool NvRmPrivAp20RmModuleToOdmModule(
    NvRmModuleID RmModule,
    NvOdmIoModule *OdmModule,
    NvU32 *OdmInstance,
    NvU32 *pCnt)
{
    NvRmModuleID Module = NVRM_MODULE_ID_MODULE(RmModule);
    NvU32 Instance = NVRM_MODULE_ID_INSTANCE(RmModule);
    NvBool Success = NV_TRUE;
    *pCnt = 1;
    switch (Module)
    {
    case NvRmModuleID_Usb2Otg:
        switch (Instance)
        {
        case 0:
            *OdmModule = NvOdmIoModule_Usb;
            *OdmInstance = 0;
            break;
        case 1:
            *OdmModule = NvOdmIoModule_Ulpi;
            *OdmInstance = 0;
            break;
        case 2:
            *OdmModule = NvOdmIoModule_Usb;
            *OdmInstance = 1;
            break;
        default:
            NV_ASSERT(!"Invalid USB instance");
            break;
        }
        break;
    case NvRmModuleID_OneWire:
        *OdmModule = NvOdmIoModule_OneWire;
        *OdmInstance = Instance;
        break;
    case NvRmModuleID_SyncNor:
        *OdmModule = NvOdmIoModule_SyncNor;
        *OdmInstance = Instance;
        break;
    case NvRmPrivModuleID_Pcie:
        *OdmModule = NvOdmIoModule_PciExpress;
        *OdmInstance = Instance;
        break;
    default:
        Success = NV_FALSE;
        *pCnt = 0;
        break;
    }
    return  Success;
}

NvError
NvRmPrivAp20GetModuleInterfaceCaps(
    NvOdmIoModule Module,
    NvU32 Instance,
    NvU32 PinMap,
    void *pCaps)
{
    switch (Module)
    {
    case NvOdmIoModule_Sdio:
        if (Instance == 1)
        {
            if (PinMap == NvOdmSdioPinMap_Config2 || PinMap == NvOdmSdioPinMap_Config4)
                ((NvRmModuleSdmmcInterfaceCaps *)pCaps)->MmcInterfaceWidth = 8;
            else if (PinMap == NvOdmSdioPinMap_Config1 ||
            PinMap == NvOdmSdioPinMap_Config3 || PinMap == NvOdmSdioPinMap_Config5)
                ((NvRmModuleSdmmcInterfaceCaps *)pCaps)->MmcInterfaceWidth = 4;
            else
            {
                NV_ASSERT(NV_FALSE);
                return NvError_NotSupported;
            }
        }
        else if (Instance==2 && PinMap==NvOdmSdioPinMap_Config1)
            ((NvRmModuleSdmmcInterfaceCaps *)pCaps)->MmcInterfaceWidth = 8;
        else if (Instance==3 && (PinMap==NvOdmSdioPinMap_Config1 || PinMap==NvOdmSdioPinMap_Config2))
            ((NvRmModuleSdmmcInterfaceCaps *)pCaps)->MmcInterfaceWidth = 8;
        else
            ((NvRmModuleSdmmcInterfaceCaps *)pCaps)->MmcInterfaceWidth = 4;
        return NvError_Success;

    case NvOdmIoModule_Pwm:
        if (Instance == 0 && (PinMap == NvOdmPwmPinMap_Config1))
            ((NvRmModulePwmInterfaceCaps *)pCaps)->PwmOutputIdSupported = 15;
        else if (Instance == 0 && (PinMap == NvOdmPwmPinMap_Config2))
            ((NvRmModulePwmInterfaceCaps *)pCaps)->PwmOutputIdSupported = 13;
        else if (Instance == 0 && (PinMap == NvOdmPwmPinMap_Config3))
            ((NvRmModulePwmInterfaceCaps *)pCaps)->PwmOutputIdSupported = 1;
        else if (Instance == 0 && (PinMap == NvOdmPwmPinMap_Config4))
            ((NvRmModulePwmInterfaceCaps *)pCaps)->PwmOutputIdSupported = 12;
        else if (Instance == 0 && (PinMap == NvOdmPwmPinMap_Config5))
            ((NvRmModulePwmInterfaceCaps *)pCaps)->PwmOutputIdSupported = 15;
        else if (Instance == 0 && (PinMap == NvOdmPwmPinMap_Config6))
            ((NvRmModulePwmInterfaceCaps *)pCaps)->PwmOutputIdSupported = 3;
        else
        {
            ((NvRmModulePwmInterfaceCaps *)pCaps)->PwmOutputIdSupported = 0;
            return NvError_NotSupported;
        }
        return NvError_Success;
    case NvOdmIoModule_Nand:
        if (Instance == 0 && (PinMap == NvOdmNandPinMap_Config1 || PinMap ==
        NvOdmNandPinMap_Config3))
        {
            ((NvRmModuleNandInterfaceCaps*)pCaps)->IsCombRbsyMode = NV_TRUE;
            ((NvRmModuleNandInterfaceCaps*)pCaps)->NandInterfaceWidth = 16;
        }
        else if (Instance == 0 && (PinMap == NvOdmNandPinMap_Config2 ||
            PinMap == NvOdmNandPinMap_Config4))
        {
            ((NvRmModuleNandInterfaceCaps*)pCaps)->IsCombRbsyMode = NV_TRUE;
            ((NvRmModuleNandInterfaceCaps*)pCaps)->NandInterfaceWidth = 8;
        }
        else
        {
            NV_ASSERT(NV_FALSE);
            return NvError_NotSupported;
        }
        return NvSuccess;
    case NvOdmIoModule_Uart:
        if (Instance == 0)
        {
            if (PinMap == NvOdmUartPinMap_Config1)
                ((NvRmModuleUartInterfaceCaps *)pCaps)->NumberOfInterfaceLines = 8;
            else if (PinMap == NvOdmUartPinMap_Config2)
                ((NvRmModuleUartInterfaceCaps *)pCaps)->NumberOfInterfaceLines = 7;
            else if ((PinMap == NvOdmUartPinMap_Config3) || (PinMap == NvOdmUartPinMap_Config6))
                ((NvRmModuleUartInterfaceCaps *)pCaps)->NumberOfInterfaceLines = 4;
            else if ((PinMap == NvOdmUartPinMap_Config4) || (PinMap == NvOdmUartPinMap_Config5))
                ((NvRmModuleUartInterfaceCaps *)pCaps)->NumberOfInterfaceLines = 2;
            else if (PinMap == NvOdmUartPinMap_Config7)
                ((NvRmModuleUartInterfaceCaps *)pCaps)->NumberOfInterfaceLines = 6;
            else
                ((NvRmModuleUartInterfaceCaps *)pCaps)->NumberOfInterfaceLines = 0;
        }
        else if ((Instance == 1) || (Instance == 2))
        {
            if (PinMap == NvOdmUartPinMap_Config1)
                ((NvRmModuleUartInterfaceCaps *)pCaps)->NumberOfInterfaceLines = 4;
            else if (PinMap == NvOdmUartPinMap_Config2)
                ((NvRmModuleUartInterfaceCaps *)pCaps)->NumberOfInterfaceLines = 2;
            else
                ((NvRmModuleUartInterfaceCaps *)pCaps)->NumberOfInterfaceLines = 0;
        }
        else if ((Instance == 3) || (Instance == 4))
        {
            if ((PinMap == NvOdmUartPinMap_Config1) || (PinMap == NvOdmUartPinMap_Config2))
                ((NvRmModuleUartInterfaceCaps *)pCaps)->NumberOfInterfaceLines = 4;
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

    return NvError_NotSupported;
}

NvError
NvRmAp20GetStraps(
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

void NvRmAp20SetDefaultTristate(NvRmDeviceHandle hDevice)
{
    tegra_pinmux_set_vddio_tristate(TEGRA_VDDIO_NAND, TEGRA_TRI_TRISTATE);
    tegra_pinmux_set_tristate(TEGRA_PINGROUP_PTA, TEGRA_TRI_NORMAL);
    tegra_pinmux_set_tristate(TEGRA_PINGROUP_PTA, TEGRA_TRI_TRISTATE);
}

