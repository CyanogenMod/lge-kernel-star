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
#include "nvrm_structure.h"
#include "nvrm_analog.h"
#include "nvrm_drf.h"
#include "nvassert.h"
#include "nvrm_hwintf.h"
#include "nvrm_power.h"
#include "ap16/arapb_misc.h"
#include "ap15/arclk_rst.h"
#include "ap15/arfuse.h"
#include "nvodm_query.h"
#include "nvodm_pmu.h"
#include "nvrm_clocks.h"
#include "nvrm_module.h"

static NvError
NvRmPrivTvDcControl( NvRmDeviceHandle hDevice, NvBool enable, NvU32 inst,
    void *Config, NvU32 ConfigLength )
{
    NvRmAnalogTvDacConfig *cfg;
    NvU32 ctrl, source;
    NvU32 src_id;
    NvU32 src_inst;

    NV_ASSERT( ConfigLength == 0 ||
        ConfigLength == sizeof(NvRmAnalogTvDacConfig) );

    if( enable )
    {
        cfg = (NvRmAnalogTvDacConfig *)Config;
        NV_ASSERT( cfg );

        src_id = NVRM_MODULE_ID_MODULE( cfg->Source );
        src_inst = NVRM_MODULE_ID_INSTANCE( cfg->Source );

        ctrl = NV_DRF_DEF( APB_MISC_ASYNC, TVDACCNTL, DAC_IDDQ, DISABLE )
            | NV_DRF_DEF( APB_MISC_ASYNC, TVDACCNTL, DAC_POWERDOWN, DISABLE )
            | NV_DRF_DEF( APB_MISC_ASYNC, TVDACCNTL, DAC_DETECT_EN, ENABLE )
            | NV_DRF_DEF( APB_MISC_ASYNC, TVDACCNTL, DAC_SLEEPR, DISABLE )
            | NV_DRF_DEF( APB_MISC_ASYNC, TVDACCNTL, DAC_SLEEPG, DISABLE )
            | NV_DRF_DEF( APB_MISC_ASYNC, TVDACCNTL, DAC_SLEEPB, DISABLE )
            | NV_DRF_DEF( APB_MISC_ASYNC, TVDACCNTL, DAC_COMPR_EN, ENABLE )
            | NV_DRF_DEF( APB_MISC_ASYNC, TVDACCNTL, DAC_COMPG_EN, ENABLE )
            | NV_DRF_DEF( APB_MISC_ASYNC, TVDACCNTL, DAC_COMPB_EN, ENABLE );

        if( src_id == NvRmModuleID_Tvo )
        {
            source = NV_DRF_DEF( APB_MISC_ASYNC, TVDACDINCONFIG,
                DAC_SOURCE, TVO );
        }
        else
        {
            NV_ASSERT( src_id == NvRmModuleID_Display );
            if( src_inst == 0 )
            {
                source = NV_DRF_DEF( APB_MISC_ASYNC, TVDACDINCONFIG,
                    DAC_SOURCE, DISPLAY );
            }
            else
            {
                source = NV_DRF_DEF( APB_MISC_ASYNC, TVDACDINCONFIG,
                    DAC_SOURCE, DISPLAYB );
            }
        }

        source = NV_FLD_SET_DRF_NUM( APB_MISC_ASYNC, TVDACDINCONFIG, DAC_AMPIN,
            cfg->DacAmplitude, source );
    }
    else
    {
        ctrl = NV_DRF_DEF( APB_MISC_ASYNC, TVDACCNTL, DAC_IDDQ, ENABLE )
            | NV_DRF_DEF( APB_MISC_ASYNC, TVDACCNTL, DAC_POWERDOWN, ENABLE )
            | NV_DRF_DEF( APB_MISC_ASYNC, TVDACCNTL, DAC_DETECT_EN, DISABLE )
            | NV_DRF_DEF( APB_MISC_ASYNC, TVDACCNTL, DAC_SLEEPR, ENABLE )
            | NV_DRF_DEF( APB_MISC_ASYNC, TVDACCNTL, DAC_SLEEPG, ENABLE )
            | NV_DRF_DEF( APB_MISC_ASYNC, TVDACCNTL, DAC_SLEEPB, ENABLE )
            | NV_DRF_DEF( APB_MISC_ASYNC, TVDACCNTL, DAC_COMPR_EN, DISABLE )
            | NV_DRF_DEF( APB_MISC_ASYNC, TVDACCNTL, DAC_COMPG_EN, DISABLE )
            | NV_DRF_DEF( APB_MISC_ASYNC, TVDACCNTL, DAC_COMPB_EN, DISABLE );
        source = NV_DRF_DEF( APB_MISC_ASYNC, TVDACDINCONFIG,
                    DAC_SOURCE, TVDAC_OFF );
    }

    NV_REGW( hDevice, NvRmModuleID_Misc, 0, APB_MISC_ASYNC_TVDACCNTL_0,
        ctrl );
    NV_REGW( hDevice, NvRmModuleID_Misc, 0,
        APB_MISC_ASYNC_TVDACDINCONFIG_0, source );

    return NvSuccess;
}

static NvError
NvRmPrivVideoInputControl( NvRmDeviceHandle hDevice, NvBool enable,
    NvU32 inst, void *Config, NvU32 ConfigLength )
{
    NvU32 val;

    NV_ASSERT(ConfigLength == 0);
    NV_ASSERT(Config == 0);
    NV_ASSERT(inst == 0);

    if( enable )
    {
        val = NV_DRF_DEF( APB_MISC_ASYNC, VCLKCTRL, VCLK_PAD_IE, ENABLE );
    }
    else
    {
        val = NV_DRF_DEF( APB_MISC_ASYNC, VCLKCTRL, VCLK_PAD_IE, DISABLE );
    }

    NV_REGW( hDevice, NvRmModuleID_Misc, 0, APB_MISC_ASYNC_VCLKCTRL_0,
        val );

    return NvSuccess;
}


NvError
NvRmAnalogInterfaceControl(
    NvRmDeviceHandle hDevice,
    NvRmAnalogInterface Interface,
    NvBool Enable,
    void *Config,
    NvU32 ConfigLength )
{
    NvError err = NvSuccess;
    NvU32 id;
    NvU32 inst;

    NV_ASSERT( hDevice );

    id = NVRM_ANALOG_INTERFACE_ID( Interface );
    inst = NVRM_ANALOG_INTERFACE_INSTANCE( Interface );

    NvOsMutexLock( hDevice->mutex );

    switch( id ) {
    case NvRmAnalogInterface_Dsi:
        break;
    case NvRmAnalogInterface_ExternalMemory:
        break;
    case NvRmAnalogInterface_Hdmi:
        break;
    case NvRmAnalogInterface_Lcd:
        break;
    case NvRmAnalogInterface_Uart:
        break;
    case NvRmAnalogInterface_Sdio:
        break;
    case NvRmAnalogInterface_Tv:
        err = NvRmPrivTvDcControl( hDevice, Enable, inst, Config,
            ConfigLength );
        break;
    case NvRmAnalogInterface_VideoInput:
        err = NvRmPrivVideoInputControl( hDevice, Enable, inst, Config,
            ConfigLength);
        break;
    default:
        NV_ASSERT(!"Unknown Analog interface passed. ");
    }

    NvOsMutexUnlock( hDevice->mutex );

    return err;
}

NvBool
NvRmUsbIsConnected(
    NvRmDeviceHandle hDevice)
{
    //Do nothing
    return NV_TRUE;
}

NvU32
NvRmUsbDetectChargerState(
    NvRmDeviceHandle hDevice,
    NvU32 wait)
{
    //Do nothing
    return NvOdmUsbChargerType_UsbHost;
}

NvU8
NvRmAnalogGetTvDacConfiguration(
    NvRmDeviceHandle hDevice,
    NvRmAnalogTvDacType Type)
{
    NvU8 RetVal = 0;
    NvU32 OldRegVal = 0;
    NvU32 NewRegVal = 0;

    NV_ASSERT(hDevice);
    
#if NV_USE_FUSE_CLOCK_ENABLE
    // Enable fuse clock
    NvRmPowerModuleClockControl(hDevice, NvRmModuleID_Fuse, 0, NV_TRUE);
#endif    

    // Enable fuse values to be visible before reading the fuses.
    OldRegVal = NV_REGR(hDevice, NvRmPrivModuleID_ClockAndReset, 0, 
        CLK_RST_CONTROLLER_MISC_CLK_ENB_0);
    NewRegVal = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, MISC_CLK_ENB, 
        CFG_ALL_VISIBLE, 1, OldRegVal);
    NV_REGW(hDevice, NvRmPrivModuleID_ClockAndReset, 0, 
        CLK_RST_CONTROLLER_MISC_CLK_ENB_0, NewRegVal);

    switch (Type)
    {
        case NvRmAnalogTvDacType_CRT:
            RetVal = NV_REGR(hDevice, NvRmModuleID_Fuse, 0, FUSE_DAC_CRT_CALIB_0);
            break;
        case NvRmAnalogTvDacType_SDTV:
            RetVal = NV_REGR(hDevice, NvRmModuleID_Fuse, 0, FUSE_DAC_SDTV_CALIB_0);
            break;
        case NvRmAnalogTvDacType_HDTV:
            RetVal = NV_REGR(hDevice, NvRmModuleID_Fuse, 0, FUSE_DAC_HDTV_CALIB_0);
            break;
        default:
            NV_ASSERT(!"Unsupported this Dac type");
            break;
    }

    // Disable fuse values visibility
    NV_REGW(hDevice, NvRmPrivModuleID_ClockAndReset, 0, 
        CLK_RST_CONTROLLER_MISC_CLK_ENB_0, OldRegVal);

#if NV_USE_FUSE_CLOCK_ENABLE
    // Disable fuse clock
    NvRmPowerModuleClockControl(hDevice, NvRmModuleID_Fuse, 0, NV_FALSE);
#endif

    return RetVal; 
}
