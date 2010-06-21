/*
 * Copyright (c) 2008-2010 NVIDIA Corporation.
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
#include "ap15/ap15rm_private.h"
#include "nvrm_pinmux_utils.h"
#include "nvodm_query_pinmux.h"
#include <linux/module.h>

#include <mach/pinmux.h>
#include "../../../gpio-names.h"

/*  Each of the pin mux configurations defined in the pin mux spreadsheet are
 *  stored in chip-specific tables.  For each configuration, every pad group
 *  that must be programmed is stored as a single 32b entry, where the register
 *  offset (for both the tristate and pin mux control registers), field bit
 *  position (ditto), pin mux mask, and new pin mux state are programmed.
 *
 *  The tables are microcode for a simple state machine.  The state machine
 *  supports subroutine call/return (up to 4 levels of nesting), so that
 *  pin mux configurations which have substantial repetition can be
 *  represented compactly by separating common portion of the configurations
 *  into a subroutine.  Additionally, the state machine supports
 *  "unprogramming" of the pin mux registers, so that pad groups which are
 *  incorrectly programmed to mux from a controller may be safely disowned,
 *  ensuring that no conflicts exist where multiple pad groups are muxing
 *  the same set of signals.
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

#define IO_MODULES() \
    iomodule_devid(Ata, 1,  "ide")                                         \
    iomodule_devid(Crt, 1, "crt")                                          \
    iomodule_devid(Csi, 1, "csi")                                          \
    iomodule_devid(Dap, 5, "i2s.0", "i2s.1", "i2s.2",                      \
                                "i2s.3", "i2s.4")                          \
    iomodule_devid(Display,2, "displaya", "displayb")                      \
    iomodule_devid(Dsi, 1,  "dsi")                                         \
    iomodule_devid(Gpio,1,  "gpio")                                        \
    iomodule_devid(Hdcp,1,  "hdcp")                                        \
    iomodule_devid(Hdmi,1,  "hdmi")                                        \
    iomodule_devid(Hsi,1, "hsi")                                           \
    iomodule_devid(Hsmmc,1, "hsmmc")                                       \
    iomodule_devid(I2s,1, "i2s")                                           \
    iomodule_devid(I2c,3, "tegra_i2c.1", "tegra_i2c.2", "tegra_i2c.3")     \
    iomodule_devid(I2c_Pmu,1, "tegra_i2c.0")                               \
    iomodule_devid(Kbd, 1, "tegra_kbc")                                    \
    iomodule_devid(Mio, 1, "mio")                                          \
    iomodule_devid(Nand,1, "tegra_nand")                                   \
    iomodule_devid(Pwm, 1, "pwm")                                          \
    iomodule_devid(Sdio,4, "tegra-sdhci.0", "tegra-sdhci.1",               \
                        "tegra-sdhci.2", "tegra-sdhci.3")                  \
    iomodule_devid(Sflash,1, "tegra_spi.4")                                \
    iomodule_devid(Slink,1, "slink")                                       \
    iomodule_devid(Spdif,1, "spdif")                                       \
    iomodule_devid(Spi, 4, "tegra_spi.0", "tegra_spi.1", "tegra_spi.2",    \
                        "tegra_spi.3")                                     \
    iomodule_devid(Twc, 1, "twc")                                          \
    iomodule_devid(Tvo, 1, "tvo")                                          \
    iomodule_devid(Uart, 5, "tegra_uart.0", "tegra_uart.1",                \
                        "tegra_uart.2", "tegra_uart.3",                    \
                        "tegra_uart.4")                                    \
    iomodule_devid(Usb, 1, "Usb")                                          \
    iomodule_devid(Vdd, 1, "Vdd")                                          \
    iomodule_devid(VideoInput, 1, "vi")                                    \
    iomodule_devid(Xio, 1, "xio")                                          \
    iomodule_devid(ExternalClock, 3, "extclk.0", "extclk.1", "extclk.2")   \
    iomodule_devid(Ulpi, 1, "tegra-ehci.1")                                \
    iomodule_devid(OneWire, 1, "tegra_w1")                                 \
    iomodule_devid(SyncNor, 1, "snor")                                     \
    iomodule_devid(PciExpress, 1, "tegra_pcie")                            \
    iomodule_devid(Trace, 1, "etm")                                        \
    iomodule_devid(Tsense,1, "tsensor")                                    \
    iomodule_devid(BacklightPwm,4, "blight.d1.p0", "blight.d1.p1",         \
                                 "blight.d2.p0", "blight.d2.p1")

struct tegra_iomodule_devlist
{
    int t_inst;
    const char **dev_list;
};

#define iomodule_devid(mod, tinst, ...) \
    static const char *tegra_iomodule_##mod[] = {__VA_ARGS__};
IO_MODULES()

#undef iomodule_devid
#define iomodule_devid(mod, tinst, ...)    \
    [NvOdmIoModule_##mod] = {              \
         .t_inst = tinst,                  \
         .dev_list = tegra_iomodule_##mod, \
     },

static const struct  tegra_iomodule_devlist iomodule_devlist[NvOdmIoModule_Num] = {
    IO_MODULES()
};

extern struct tegra_pingroup_config *tegra_pinmux_get(const char *dev_id,
         int config, int *len);
extern int gpio_get_pinmux_group(int gpio_nr);

typedef struct
{
    void
    (*pfnEnableExtClock)(
        NvRmDeviceHandle hDevice,
        struct tegra_pingroup_config *pin_config,
        int len,
        NvBool ClockState);
    NvU32
    (*pfnGetExtClockFreq)(
        NvRmDeviceHandle hDevice,
        struct tegra_pingroup_config *pin_config,
        int len);
    NvError
    (*pfnInterfaceCaps)(
        NvOdmIoModule Module,
        NvU32 Instance,
        NvU32 PinMap,
        void *pCaps);
    NvError
    (*pfnGetStraps)(
        NvRmDeviceHandle hDevice,
        NvRmStrapGroup StrapGroup,
        NvU32* pStrapValue);
    void
    (*pfnSetDefaultTristate)(
        NvRmDeviceHandle hDevice);
} NvPinmuxPrivMethods;

    static NvPinmuxPrivMethods s_PinmuxMethods =
    {
#ifdef CONFIG_ARCH_TEGRA_2x_SOC
    NvRmPrivAp20EnableExternalClockSource,
        NvRmPrivAp20GetExternalClockSourceFreq,
        NvRmPrivAp20GetModuleInterfaceCaps,
        NvRmAp20GetStraps,
        NvRmAp20SetDefaultTristate
#else
#error "Unsupported Tegra architecture"
#endif
};

static void NvRmPrivApplyAllModuleTypePinMuxes(
    NvRmDeviceHandle hDevice,
    NvU32 IoModule,
    NvBool ApplyReset,
    NvBool ApplyActual)
{
    const NvU32 *OdmConfigs;
    NvU32 NumOdmConfigs;
    struct tegra_pingroup_config *pin_config;
    int len = 0;
    int Instance = 0;
    NvU32 config;

    if (ApplyActual)
        NvOdmQueryPinMux(IoModule, &OdmConfigs, &NumOdmConfigs);
    else
    {
        OdmConfigs = NULL;
        NumOdmConfigs = 0;
    }

    for (Instance = 0; Instance < iomodule_devlist[IoModule].t_inst; ++Instance)
    {
        /*  Apply the reset configuration to ensure that the module is in
         *  a sane state, then apply the ODM configuration, if one is specified
         */
        if (ApplyReset)
        {
            if (iomodule_devlist[IoModule].dev_list[Instance] != NULL)
            {
                pin_config = tegra_pinmux_get(iomodule_devlist[IoModule].dev_list[Instance],
                            0, &len);
                    if (pin_config != NULL)
                    {
                        tegra_pinmux_config_pinmux_table(pin_config, len, false);
                    }
            }
        }
        if (NumOdmConfigs && ApplyActual)
        {
            config = *OdmConfigs;
            if (config)
            {
                if (iomodule_devlist[IoModule].dev_list[Instance] != NULL)
                {
                    pin_config = tegra_pinmux_get(
                                        iomodule_devlist[IoModule].dev_list[Instance],
                                        config, &len);
                    if (pin_config != NULL)
                    {
                        tegra_pinmux_config_pinmux_table(pin_config, len, true);
                    }
                }
            }
            NumOdmConfigs--;
            OdmConfigs++;
        }
    }

    /*  If the ODM pin mux table is created correctly, there should be
    *  the same number of ODM configs as module instances; however, we
    *  allow the ODM to specify fewer configs than instances with assumed
    *  zeros for undefined modules */
    while (NumOdmConfigs)
    {
        NV_ASSERT((*OdmConfigs==0) &&
                    "More ODM configs than module instances!\n");
        NumOdmConfigs--;
        OdmConfigs++;
    }
}

static void NvRmPrivApplyAllPinMuxes(
    NvRmDeviceHandle hDevice,
    NvBool First)
{
    NvOdmIoModule IoModule;

    for (IoModule = NvOdmIoModule_Ata; IoModule < NvOdmIoModule_Num; IoModule++)
    {
        NvBool ApplyActual = NV_TRUE;
        /* During early boot, the only device that has its pin mux correctly
         * initialized is the I2C PMU controller, so that primitive peripherals
         * (EEPROMs, PMU, RTC) can be accessed during the boot process */
        if (First)
            ApplyActual = (IoModule==NvOdmIoModule_I2c_Pmu);

        NvRmPrivApplyAllModuleTypePinMuxes(hDevice, IoModule, First,
                                           ApplyActual);
    }
}
/**
 * RmInitPinMux will program the pin mux settings for all IO controllers to
 * the ODM-selected value (or a safe reset value, if no value is defined in
 * the ODM query.
 *
 * It will also read the current value of the tristate registers, to
 * initialize the reference count
 */
void NvRmInitPinMux(
    NvRmDeviceHandle hDevice,
    NvBool First)
{
    if (First) {
        tegra_pinmux_init_pingroups();
        (s_PinmuxMethods.pfnSetDefaultTristate)(hDevice);
    }

    NvRmPrivApplyAllPinMuxes(hDevice, First);
}

/* RmPinMuxConfigSelect sets a specific module to a specific configuration.
 * It is used for multiplexed controllers, and should only be called by the
 * ODM service function NvOdmPinMuxSet */
void NvRmPinMuxConfigSelect(
    NvRmDeviceHandle hDevice,
    NvOdmIoModule IoModule,
    NvU32 Instance,
    NvU32 Configuration)
{
    struct tegra_pingroup_config *pin_config;
    int len = 0;
    NvU32 newConfiguration = Configuration;
    int isNonMultiplexed = true;

    NV_ASSERT(hDevice);
    if (!hDevice)
        return;
    if (Instance >= iomodule_devlist[IoModule].t_inst)
	return;

    /* For multiplexed config, get the 0 configuration and
       Cancel the current setting */
    if (Configuration == NVODM_QUERY_PINMAP_MULTIPLEXED)
    {
         newConfiguration = 0;
         isNonMultiplexed = false;
    }

    if (iomodule_devlist[IoModule].dev_list[Instance] != NULL)
    {
        pin_config = tegra_pinmux_get(iomodule_devlist[IoModule].dev_list[Instance],
                            newConfiguration, &len);
        if (pin_config != NULL)
        {
            tegra_pinmux_config_pinmux_table(pin_config, len, isNonMultiplexed);
        }
    }
}

/* RmPinMuxConfigSetTristate will either enable or disable the tristate for a
 * specific IO module configuration.  It is called by the ODM service function
 * OdmPinMuxConfigSetTristate, and by the RM function SetModuleTristate.  RM
 * client drivers should only call RmSetModuleTristate, which will program the
 * tristate correctly based on the ODM query configuration. */
void NvRmPinMuxConfigSetTristate(
    NvRmDeviceHandle hDevice,
    NvOdmIoModule IoModule,
    NvU32 Instance,
    NvU32 Configuration,
    NvBool EnableTristate)
{
    struct tegra_pingroup_config *pin_config;
    int len = 0;
    tegra_tristate_t tristate;

    NV_ASSERT(hDevice);
    if (!hDevice)
        return;

    if (Instance >= iomodule_devlist[IoModule].t_inst)
        return;

    if (iomodule_devlist[IoModule].dev_list[Instance] != NULL)
    {
        pin_config = tegra_pinmux_get(iomodule_devlist[IoModule].dev_list[Instance],
                        Configuration, &len);
        if (pin_config != NULL)
        {
            tristate = (EnableTristate)? TEGRA_TRI_TRISTATE: TEGRA_TRI_NORMAL;
            tegra_pinmux_config_tristate_table(pin_config, len, tristate);
        }
    }
}

NvError NvRmSetOdmModuleTristate(
    NvRmDeviceHandle hDevice,
    NvU32 OdmModule,
    NvU32 OdmInstance,
    NvBool EnableTristate)
{
    const NvU32 *OdmConfigs;
    NvU32 NumOdmConfigs;

    NV_ASSERT(hDevice);
    if (!hDevice)
        return NvError_BadParameter;

    NvOdmQueryPinMux(OdmModule, &OdmConfigs, &NumOdmConfigs);

    if ((OdmInstance >= NumOdmConfigs) || !OdmConfigs[OdmInstance])
        return NvError_NotSupported;

    NvRmPinMuxConfigSetTristate(hDevice, OdmModule,
        OdmInstance, OdmConfigs[OdmInstance], EnableTristate);

    return NvSuccess;
}

NvU32 NvRmPrivRmModuleToOdmModule(
    NvU32 ChipId,
    NvU32 RmModule,
    NvOdmIoModule *pOdmModules,
    NvU32 *pOdmInstances)
{
    NvU32 Cnt = 0;
    NvBool Result = NV_FALSE;

    NV_ASSERT(pOdmModules && pOdmInstances);

#if defined(CONFIG_ARCH_TEGRA_2x_SOC)
    NV_ASSERT(ChipId==0x20);
    Result = NvRmPrivAp20RmModuleToOdmModule(RmModule,
         pOdmModules, pOdmInstances, &Cnt);
#else
#error "Unsupported Tegra architecture"
#endif

    /*  A default mapping is provided for all standard I/O controllers,
     *  if the chip-specific implementation does not implement a mapping */
    if (!Result)
    {
        NvRmModuleID Module = NVRM_MODULE_ID_MODULE(RmModule);
        NvU32 Instance = NVRM_MODULE_ID_INSTANCE(RmModule);

        Cnt = 1;
        switch (Module) {
        case NvRmModuleID_Display:
            *pOdmModules = NvOdmIoModule_Display;
            *pOdmInstances = Instance;
            break;
        case NvRmModuleID_Ide:
            *pOdmModules = NvOdmIoModule_Ata;
            *pOdmInstances = Instance;
            break;
        case NvRmModuleID_Vi:
            *pOdmModules = NvOdmIoModule_VideoInput;
            *pOdmInstances = Instance;
            break;
        case NvRmModuleID_Usb2Otg:
            *pOdmModules = NvOdmIoModule_Usb;
            *pOdmInstances = Instance;
            break;
        case NvRmModuleID_Pwm:
            *pOdmModules = NvOdmIoModule_Pwm;
            *pOdmInstances = Instance;
            break;
        case NvRmModuleID_Twc:
            *pOdmModules = NvOdmIoModule_Twc;
            *pOdmInstances = Instance;
            break;
        case NvRmModuleID_Hsmmc:
            *pOdmModules = NvOdmIoModule_Hsmmc;
            *pOdmInstances = Instance;
            break;
        case NvRmModuleID_Sdio:
            *pOdmModules = NvOdmIoModule_Sdio;
            *pOdmInstances = Instance;
            break;
        case NvRmModuleID_Nand:
            *pOdmModules = NvOdmIoModule_Nand;
            *pOdmInstances = Instance;
            break;
        case NvRmModuleID_I2c:
            *pOdmModules = NvOdmIoModule_I2c;
            *pOdmInstances = Instance;
            break;
        case NvRmModuleID_Spdif:
            *pOdmModules = NvOdmIoModule_Spdif;
            *pOdmInstances = Instance;
            break;
        case NvRmModuleID_Uart:
            *pOdmModules = NvOdmIoModule_Uart;
            *pOdmInstances = Instance;
            break;
        case NvRmModuleID_Csi:
            *pOdmModules = NvOdmIoModule_Csi;
            *pOdmInstances = Instance;
            break;
        case NvRmModuleID_Hdmi:
            *pOdmModules = NvOdmIoModule_Hdmi;
            *pOdmInstances = Instance;
            break;
        case NvRmModuleID_Mipi:
            *pOdmModules = NvOdmIoModule_Hsi;
            *pOdmInstances = Instance;
            break;
        case NvRmModuleID_Tvo:
            pOdmModules[0] = NvOdmIoModule_Tvo;
            pOdmModules[1] = NvOdmIoModule_Crt;
            pOdmInstances[0] = 0;
            pOdmInstances[1] = 0;
            Cnt = 2;
            break;
        case NvRmModuleID_Dsi:
            *pOdmModules = NvOdmIoModule_Dsi;
            *pOdmInstances = Instance;
            break;
        case NvRmModuleID_Dvc:
            *pOdmModules = NvOdmIoModule_I2c_Pmu;
            *pOdmInstances = Instance;
            break;
        case NvRmPrivModuleID_Mio_Exio:
            *pOdmModules = NvOdmIoModule_Mio;
            *pOdmInstances = Instance;
            break;
        case NvRmModuleID_Xio:
            *pOdmModules = NvOdmIoModule_Xio;
            *pOdmInstances = Instance;
            break;
        case NvRmModuleID_Spi:
            *pOdmModules = NvOdmIoModule_Sflash;
            *pOdmInstances = Instance;
            break;
        case NvRmModuleID_Slink:
            *pOdmModules = NvOdmIoModule_Spi;
            *pOdmInstances = Instance;
            break;
        case NvRmModuleID_Kbc:
            *pOdmModules = NvOdmIoModule_Kbd;
            *pOdmInstances = Instance;
            break;
        default:
            //  all the RM modules which have no ODM analogs (like 3d)
            Cnt = 0;
            break;
        }
    }

    return Cnt;
}

/* RmSetModuleTristate will enable / disable the pad tristates for the
 * selected pin mux configuration of an IO module.  */
NvError NvRmSetModuleTristate(
    NvRmDeviceHandle hDevice,
    NvRmModuleID RmModule,
    NvBool EnableTristate)
{
    const NvU32 *OdmConfigs;
    NvU32 NumOdmConfigs;
    NvU32 OdmModules[4];
    NvU32 OdmInstances[4];
    NvU32 NumOdmModules = 0;
    NvU32 i;

    NV_ASSERT(hDevice);
    if (!hDevice)
        return NvError_BadParameter;

    NumOdmModules =
        NvRmPrivRmModuleToOdmModule(hDevice->ChipId.Id,
            RmModule, (NvOdmIoModule*)OdmModules, OdmInstances);
    if (!NumOdmModules)
        return NvError_NotSupported;

    /* return NotSupported if the ODM has not defined a pin mux configuration
     * for this module. */
    for (i=0; i<NumOdmModules; i++)
    {
        NvOdmQueryPinMux(OdmModules[i], &OdmConfigs, &NumOdmConfigs);
        if ((!NumOdmConfigs) || (!OdmConfigs[OdmInstances[i]]))
            return NvError_NotSupported;
        if (OdmInstances[i] >= NumOdmConfigs)
        {
            NV_DEBUG_PRINTF(("Attempted to set TRISTATE for Module %u, Instance"
                " %u (ODM module %u instance %u) with undefined config\n",
                NVRM_MODULE_ID_MODULE(RmModule),
                NVRM_MODULE_ID_INSTANCE(RmModule),
                OdmModules[i], OdmInstances[i]));
            return NvError_NotSupported;
            //  NV_ASSERT(OdmInstances[i] < NumOdmConfigs);
        }
    }

    for (i=0; i<NumOdmModules; i++)
    {
        NvOdmQueryPinMux(OdmModules[i], &OdmConfigs, &NumOdmConfigs);
        NV_ASSERT(OdmInstances[i] < NumOdmConfigs);
        NvRmPinMuxConfigSetTristate(hDevice, OdmModules[i],
            OdmInstances[i], OdmConfigs[OdmInstances[i]], EnableTristate);
    }
    return NvSuccess;
}

void NvRmSetGpioTristate(
    NvRmDeviceHandle hDevice,
    NvU32 Port,
    NvU32 Pin,
    NvBool EnableTristate)
{
    int gpio_nr;
    tegra_pingroup_t pg;
    tegra_tristate_t ts;
    int err;

    gpio_nr = Port*8 + Pin;

    pg = gpio_get_pinmux_group(gpio_nr);
    if (pg >= 0)
    {
        ts = (EnableTristate == NV_TRUE)?TEGRA_TRI_TRISTATE: TEGRA_TRI_NORMAL;
        err = tegra_pinmux_set_tristate(pg, ts);
        if (err < 0)
            printk(KERN_ERR "pinmux: can't set pingroup %d tristate"
                " to %d: %d\n", pg,
                ts, err);
    }
}

NvU32 NvRmExternalClockConfig(
    NvRmDeviceHandle hDevice,
    NvU32 IoModule,
    NvU32 Instance,
    NvU32 Config,
    NvBool EnableTristate)
{
    NvU32 ret = 0;
    struct tegra_pingroup_config *pin_config;
    int len = 0;
    tegra_tristate_t tristate;


    NV_ASSERT(hDevice);

    if (!hDevice)
        return 0;

    if (IoModule != NvOdmIoModule_ExternalClock)
        return 0;

    if (Instance >= iomodule_devlist[IoModule].t_inst)
        return 0;

    if (iomodule_devlist[IoModule].dev_list[Instance] == NULL)
        return 0;

    if (!EnableTristate)
    {
        if (Config)
        {
            pin_config = tegra_pinmux_get(iomodule_devlist[IoModule].dev_list[Instance],
                            Config, &len);
            if (pin_config != NULL)
            {
                tegra_pinmux_config_pinmux_table(pin_config, len, true);
            }
        }
    }

    /* setting tri state */
    pin_config = tegra_pinmux_get(iomodule_devlist[IoModule].dev_list[Instance],
                                   Config, &len);
    if (pin_config != NULL)
    {
        tristate = (EnableTristate)?TEGRA_TRI_TRISTATE: TEGRA_TRI_NORMAL;
        tegra_pinmux_config_tristate_table(pin_config, len, tristate);
        (s_PinmuxMethods.pfnEnableExtClock)(hDevice, pin_config, len, !EnableTristate);
        ret = (s_PinmuxMethods.pfnGetExtClockFreq)(hDevice, pin_config, len);
    }
    return ret;
}

NvError NvRmGetModuleInterfaceCapabilities(
    NvRmDeviceHandle hRm,
    NvRmModuleID ModuleId,
    NvU32 CapStructSize,
    void *pCaps)
{
    NvU32 NumOdmConfigs;
    const NvU32 *OdmConfigs;
    NvOdmIoModule OdmModules[4];
    NvU32 OdmInstances[4];
    NvU32 NumOdmModules;

    NV_ASSERT(hRm);
    NV_ASSERT(pCaps);

    if (!hRm || !pCaps)
        return NvError_BadParameter;

    NumOdmModules =
        NvRmPrivRmModuleToOdmModule(hRm->ChipId.Id, ModuleId,
            (NvOdmIoModule *)OdmModules, OdmInstances);
    NV_ASSERT(NumOdmModules<=1);

    if (!NumOdmModules)
        return NvError_NotSupported;

    switch (OdmModules[0]) {
    case NvOdmIoModule_Hsmmc:
    case NvOdmIoModule_Sdio:
        if (CapStructSize != sizeof(NvRmModuleSdmmcInterfaceCaps))
        {
            NV_ASSERT(!"Invalid cap struct size");
            return NvError_BadParameter;
        }
        break;
    case NvOdmIoModule_Pwm:
        if (CapStructSize != sizeof(NvRmModulePwmInterfaceCaps))
        {
            NV_ASSERT(!"Invalid cap struct size");
            return NvError_BadParameter;
        }
        break;
    case NvOdmIoModule_Nand:
        if (CapStructSize != sizeof(NvRmModuleNandInterfaceCaps))
        {
            NV_ASSERT(!"Invalid cap struct size");
            return NvError_BadParameter;
        }
        break;

    case NvOdmIoModule_Uart:
        if (CapStructSize != sizeof(NvRmModuleUartInterfaceCaps))
        {
            NV_ASSERT(!"Invalid cap struct size");
            return NvError_BadParameter;
        }
        break;

    default:
        return NvError_NotSupported;
    }

    NvOdmQueryPinMux(OdmModules[0], &OdmConfigs, &NumOdmConfigs);
    if (OdmInstances[0]>=NumOdmConfigs || !OdmConfigs[OdmInstances[0]])
        return NvError_NotSupported;

    return (s_PinmuxMethods.pfnInterfaceCaps)(OdmModules[0],OdmInstances[0],
                            OdmConfigs[OdmInstances[0]],pCaps);
}

NvError NvRmGetStraps(
    NvRmDeviceHandle hDevice,
    NvRmStrapGroup StrapGroup,
    NvU32* pStrapValue)
{
    NV_ASSERT(hDevice && pStrapValue);

    if (!hDevice || !pStrapValue)
        return NvError_BadParameter;
    return (s_PinmuxMethods.pfnGetStraps)(hDevice, StrapGroup, pStrapValue);
}
