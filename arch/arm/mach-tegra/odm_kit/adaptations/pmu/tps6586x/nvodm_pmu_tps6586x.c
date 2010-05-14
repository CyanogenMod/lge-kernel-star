
/*
 * Copyright (c) 2009 NVIDIA Corporation.
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

#include "nvodm_query_discovery.h"
#include "nvodm_query.h"
#include "nvodm_query_gpio.h"
#include "nvodm_pmu_tps6586x_i2c.h"
#include "nvodm_pmu_tps6586x_interrupt.h"
#include "nvodm_pmu_tps6586x_batterycharger.h"
#include "nvodm_pmu_tps6586x_adc.h"
#include "nvodm_pmu_tps6586x_rtc.h"
#include "pmu_hal.h"

/**************************************************************************
 * NOTE:
 *  + Please search "FIXME" and then change the code according to your tps6586x fuse
 *      * each LDO's voltage range and default value
 *      * each SM's voltage range and default value
 *      * Interrupt Masks
 *      + Battery settings
 **************************************************************************/

/**************************************************************************
 * TPS6586x has two sets of power supliers:
 * + 3 DC/DC converters: DCD0/1/2
 * + 11 Regulators: LDO0-9 and RTC_OUT
 * Besides that, TPS6586x has 
 * + 2 sets drivers for LED or other open drain outputs
 * + 1 dedicated driver for keyboard backlight LED
 * + 1 dedicated driver for external vibrator motor
 * + 1 dedicated driver for white LED(SM3)
 **************************************************************************/

NvBool pmuPresented;
NvBool battPresence;
NvBool pmuInterruptSupported;
TPS6586xStatus pmuStatus;
NvOdmPmuDeviceTPS *hPmu;


// threshold for battery status. need to fine tune based on battery/system characterisation
#define NVODM_BATTERY_FULL_VOLTAGE_MV      4200
#define NVODM_BATTERY_HIGH_VOLTAGE_MV      3900
#define NVODM_BATTERY_LOW_VOLTAGE_MV       3300
#define NVODM_BATTERY_CRITICAL_VOLTAGE_MV  3100

#define DUMB_CHARGER_LIMIT_MA      250
#define DEDICATED_CHARGER_LIMIT_MA 1000   // 1000mA for dedicated charger

#define T_START_TIME  210
#define K_RAMP        7

/* 
  Only 4 options for charger current; Page36
  According to TPS658x spec, the charge current = scaling factor / R_ISET.
  From the schematic, R_ISET shows "1K". So the charge current = scaling factor.
  Because the min value of the scaling factor is 0.25 and the R_ISET is fixed, 
  the min charging current = 0.25 / 1  = 0.25 A = 250 mA.
  Thus, it will only support 1000mA, 750mA, 500mA, 250mA.
   TODO: In future, we need to change to the actual R_IST value
*/
#define R_IST                     1000
#define MAX_CHARGING_CURRENT      1000000/R_IST
#define L1_CHARGING_CURRENT        750
#define L2_CHARGING_CURRENT        500
#define L3_CHARGING_CURRENT        250

/* Valtage tables for LDOs */
/* FIXME: Modify those tables according to your fuse */
static const NvU32 VLDOx[] = {1250, 1500, 1800, 2500, 2700, 2850, 3100, 3300};
static const NvU32 VLDO2[] = { 725,  750,  775,  800,  825,  850,  875,  900,
                               925,  950,  975, 1000, 1025, 1050, 1075, 1100,
                              1125, 1150, 1175, 1200, 1225, 1250, 1275, 1300,
                              1325, 1350, 1375, 1400, 1425, 1450, 1475, 1500};
/* FIXME tps65860 only */
static const NvU32 VLDO4[] = {1700, 1725, 1750, 1775, 1800, 1825, 1850, 1875,
                              1900, 1925, 1950, 1975, 2000, 2000, 2000, 2000,
                              2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000,
                              2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000};

typedef NvU32  (*TPS6586xPmuVoltageFunc)(const NvU32 bits);
typedef NvBool (*TPS6586xPmuRailCtrlFunc)(const NvU32 rail, NvBool enable);

typedef struct {
    NvU8    addr;
    NvU8    start;
    NvU8    bits;
    NvU8    flag;                           /*!< see each settings */
} TPS6586xPmuRegisterInfo;

typedef struct TPS6586xPmuSupplyInfoRec
{
    TPS6586xPmuSupply supply;

    TPS6586xPmuRegisterInfo supplyRegInfo;  /*!< Register info to set/get supply voltage */
    TPS6586xPmuRegisterInfo ctrlRegInfo;    /*!< Register info to enable/disable supply, flag indicates another addr */
    TPS6586xPmuVoltageFunc getVoltage;      /*!< Func to convert register bits to real voltage */
    TPS6586xPmuVoltageFunc setVoltage;      /*!< Func to convert real voltage to register bits */
    TPS6586xPmuRailCtrlFunc railControl;    /*!< Func to enable/disable output for each rail */
    NvU8 Gpio;                              /*!< GPIO pin used to enable/disable external supplies */

    NvOdmPmuVddRailCapabilities cap;

    
} TPS6586xPmuSupplyInfo;

static NvU32 TPS6586xPmuVoltageGetSM0(const NvU32 bits);
static NvU32 TPS6586xPmuVoltageGetSM1(const NvU32 bits);
static NvU32 TPS6586xPmuVoltageGetSM2(const NvU32 bits);
static NvU32 TPS6586xPmuVoltageGetVLDOx(const NvU32 bits);
static NvU32 TPS6586xPmuVoltageGetVLDO1(const NvU32 bits);
static NvU32 TPS6586xPmuVoltageGetVLDO2(const NvU32 bits);
static NvU32 TPS6586xPmuVoltageGetVLDO4(const NvU32 bits);

static NvU32 TPS6586xPmuVoltageSetSM0(const NvU32 bits);
static NvU32 TPS6586xPmuVoltageSetSM1(const NvU32 bits);
static NvU32 TPS6586xPmuVoltageSetSM2(const NvU32 bits);
static NvU32 TPS6586xPmuVoltageSetVLDOx(const NvU32 bits);
static NvU32 TPS6586xPmuVoltageSetVLDO1(const NvU32 bits);
static NvU32 TPS6586xPmuVoltageSetVLDO2(const NvU32 bits);
static NvU32 TPS6586xPmuVoltageSetVLDO4(const NvU32 bits);

/* FIXME: Change getVoltage/setVoltage according to your fuse */
static const TPS6586xPmuSupplyInfo tps6586xSupplyInfoTable[] =
{
    {
        TPS6586xPmuSupply_Invalid,
        {TPS6586x_RFF_INVALID, 0, 0, 0},
        {TPS6586x_RFF_INVALID, 0, 0, 0},
        NULL, NULL, NULL,
        TPS6586x_RFF_INVALID,
        {NV_TRUE,0,0,0,0},
    },

    //DCD0
    {
        TPS6586xPmuSupply_DCD0,

        {TPS6586x_R26_SM0V1, 0, 5, 0},   
        {TPS6586x_R10_SUPPLYENA, 1, 1, TPS6586x_R11_SUPPLYENB},   
        TPS6586xPmuVoltageGetSM0,
        TPS6586xPmuVoltageSetSM0,
        NULL,
        TPS6586x_RFF_INVALID,

        {
            NV_FALSE,
#if defined(CONFIG_TEGRA_ODM_HARMONY)
            625, 25, 2700, 1200 
#else
            900, 25, 1200, 1200 
#endif
        },
    },

    //DCD1
    {
        TPS6586xPmuSupply_DCD1,

        {TPS6586x_R23_SM1V1, 0, 5, 0},
        {TPS6586x_R10_SUPPLYENA, 0, 1, TPS6586x_R11_SUPPLYENB},
        TPS6586xPmuVoltageGetSM1,
        TPS6586xPmuVoltageSetSM1,
        NULL,
        TPS6586x_RFF_INVALID,
        {
            NV_FALSE,
#if defined(CONFIG_TEGRA_ODM_HARMONY)
            625, 25, 2700, 1000
#else
            1800, 0, 1800, 1800 
#endif
        },
    },

    //DCD2
    {
        TPS6586xPmuSupply_DCD2,

        {TPS6586x_R42_SUPPLYV2, 0, 5, 0},   
        {TPS6586x_R12_SUPPLYENC, 7, 1, TPS6586x_R13_SUPPLYEND},   
        TPS6586xPmuVoltageGetSM2,
        TPS6586xPmuVoltageSetSM2,
        NULL,
        TPS6586x_RFF_INVALID,
        {
            NV_FALSE,
#if defined(CONFIG_TEGRA_ODM_HARMONY)
            3000, 50, 4550, 3700
#else
            3000, 50, 4550, 3250  // fixme
#endif
        },
    },

    // LD00
    {
        TPS6586xPmuSupply_LDO0,
        
        {TPS6586x_R41_SUPPLYV1, 5, 3, 0},
        {TPS6586x_R12_SUPPLYENC, 0, 1, TPS6586x_R13_SUPPLYEND},
        TPS6586xPmuVoltageGetVLDOx,
        TPS6586xPmuVoltageSetVLDOx,
        NULL,
        TPS6586x_RFF_INVALID,
   
        {
            NV_FALSE,
#if defined(CONFIG_TEGRA_ODM_HARMONY)
            1250, 25, 3350, 3300
#else
            2800, 0, 2800, 2800
#endif
        },
    },

    // LD01 - V-1V2
    {
        TPS6586xPmuSupply_LDO1,
        
        {TPS6586x_R41_SUPPLYV1, 0, 5, 0},
        {TPS6586x_R12_SUPPLYENC, 1, 1, TPS6586x_R13_SUPPLYEND},
        TPS6586xPmuVoltageGetVLDO1,
        TPS6586xPmuVoltageSetVLDO1,
        NULL,
        TPS6586x_RFF_INVALID,
        {
            NV_FALSE,
#if defined(CONFIG_TEGRA_ODM_HARMONY)
            725, 25, 1500, 1100
#else
            1200, 0, 1200, 1200
#endif
        },
    },

    //LD02 - V-RTC
    {
        TPS6586xPmuSupply_LDO2,
        
#if defined(CONFIG_TEGRA_ODM_HARMONY)
        {TPS6586x_R2F_LDO2BV1, 0, 5, 0},
#else
        {TPS6586x_R29_LDO2AV1, 0, 5, 0},
#endif
        {TPS6586x_R10_SUPPLYENA, 2, 2, TPS6586x_R11_SUPPLYENB},
        TPS6586xPmuVoltageGetVLDO2,
        TPS6586xPmuVoltageSetVLDO2,
        NULL,
        TPS6586x_RFF_INVALID,
   
        {
            NV_FALSE,
#if defined(CONFIG_TEGRA_ODM_HARMONY)
            725, 25, 1500, 1200 
#else
            900, 25, 1200, 1200 
#endif
        },
    },

    //LDO3
    {
        TPS6586xPmuSupply_LDO3,

        {TPS6586x_R44_SUPPLYV4, 0, 3, 0},
        {TPS6586x_R12_SUPPLYENC, 2, 1, TPS6586x_R13_SUPPLYEND},
        TPS6586xPmuVoltageGetVLDOx,
        TPS6586xPmuVoltageSetVLDOx,
        NULL,
        TPS6586x_RFF_INVALID,

        {
            NV_FALSE,
#if defined(CONFIG_TEGRA_ODM_HARMONY)
            1250, 25, 3350, 3300
#else
            1800, 0, 1800, 1800
#endif
        },
    },

    //LDO4
    {
        TPS6586xPmuSupply_LDO4,

        {TPS6586x_R32_LDO4V1, 0, 5, 0},
        {TPS6586x_R12_SUPPLYENC, 3, 1, TPS6586x_R13_SUPPLYEND},
        TPS6586xPmuVoltageGetVLDO4,
        TPS6586xPmuVoltageSetVLDO4,
        NULL,
        TPS6586x_RFF_INVALID,

        {
            NV_FALSE, 
#if defined(CONFIG_TEGRA_ODM_HARMONY)
            1700, 25, 2000, 1800
#else
            1800, 0, 1800, 1800
#endif
        },
    },        

    //LDO5
    {
        TPS6586xPmuSupply_LDO5,

        {TPS6586x_R46_SUPPLYV6, 0, 3, 0},
        {TPS6586x_R14_SUPPLYENE, 6, 1, TPS6586x_RFF_INVALID},
        TPS6586xPmuVoltageGetVLDOx,
        TPS6586xPmuVoltageSetVLDOx,
        NULL,
        TPS6586x_RFF_INVALID,

        {
            NV_FALSE,
#if defined(CONFIG_TEGRA_ODM_HARMONY)
            1250, 25, 3350, 2850
#else
            2800, 0, 2800, 2800
#endif
        }, 
    },

    //LDO6 - V-3V3 USB
    {
        TPS6586xPmuSupply_LDO6,
        
        {TPS6586x_R43_SUPPLYV3, 0, 3, 0},
        {TPS6586x_R12_SUPPLYENC, 4, 1, TPS6586x_R13_SUPPLYEND},
        TPS6586xPmuVoltageGetVLDOx,
        TPS6586xPmuVoltageSetVLDOx,
        NULL,
        TPS6586x_RFF_INVALID,

        {
            NV_FALSE,
#if defined(CONFIG_TEGRA_ODM_HARMONY)
            1250, 25, 3350, 2850
#else
            3300, 0, 3300, 3300
#endif
        }, 
    },

    //LDO7 - V-SDIO
    {
        TPS6586xPmuSupply_LDO7,

        {TPS6586x_R43_SUPPLYV3, 3, 3, 0},
        {TPS6586x_R12_SUPPLYENC, 5, 1, TPS6586x_R13_SUPPLYEND},
        TPS6586xPmuVoltageGetVLDOx,
        TPS6586xPmuVoltageSetVLDOx,
        NULL,
        TPS6586x_RFF_INVALID,

        {
            NV_FALSE,
#if defined(CONFIG_TEGRA_ODM_HARMONY)
            1250, 25, 3350, 3300
#else
            2800, 0, 2800, 2800
#endif
        },
    },

    //LDO8 - V-2V8 
    {
        TPS6586xPmuSupply_LDO8,

        {TPS6586x_R42_SUPPLYV2, 5, 3, 0},
        {TPS6586x_R12_SUPPLYENC, 6, 1, TPS6586x_R13_SUPPLYEND},
        TPS6586xPmuVoltageGetVLDOx,
        TPS6586xPmuVoltageSetVLDOx,
        NULL,
        TPS6586x_RFF_INVALID,

        {
            NV_FALSE,
#if defined(CONFIG_TEGRA_ODM_HARMONY)
            1250, 25, 3350, 1800
#else
            2800, 0, 2800, 2800
#endif
        },
    },

    //LDO9
    {
        TPS6586xPmuSupply_LDO9,            
        
        {TPS6586x_R46_SUPPLYV6, 3, 3, 0},
        {TPS6586x_R14_SUPPLYENE, 7, 1, TPS6586x_RFF_INVALID},
        TPS6586xPmuVoltageGetVLDOx,
        TPS6586xPmuVoltageSetVLDOx,
        NULL,
        TPS6586x_RFF_INVALID,

        {
            NV_FALSE,
#if defined(CONFIG_TEGRA_ODM_HARMONY)
            1250, 25, 3350, 2850
#else
            2500, 0, 2500, 2500,
#endif
        },
    },

    //RTC_OUT
    {
        TPS6586xPmuSupply_RTC_OUT,

        {TPS6586x_R44_SUPPLYV4, 3, 3, 0},
        {TPS6586x_RFF_INVALID, 0, 0, TPS6586x_RFF_INVALID},
        TPS6586xPmuVoltageGetVLDOx,
        TPS6586xPmuVoltageSetVLDOx,
        NULL,
        TPS6586x_RFF_INVALID,

        {
            NV_FALSE,
#if defined(CONFIG_TEGRA_ODM_HARMONY)
            1250, 25, 3350, 3300
#else
            2500, 0, 2500, 2500
#endif
        },
    },


    //RED1
    {
        TPS6586xPmuSupply_RED1,

        {TPS6586x_R51_RGB1RED, 0, 5, 0},   
        {TPS6586x_R52_RGB1GREEN, 7, 1, TPS6586x_RFF_INVALID},
        NULL, NULL, NULL,
        TPS6586x_RFF_INVALID,

        {
            NV_FALSE,
            0, 1, 0x1f, 0
        },
    },

    //GREEN1
    {
        TPS6586xPmuSupply_GREEN1,

        {TPS6586x_R52_RGB1GREEN, 0, 5, 0},   
        {TPS6586x_R52_RGB1GREEN, 7, 1, TPS6586x_RFF_INVALID},
        NULL, NULL, NULL,
        TPS6586x_RFF_INVALID,

        {
            NV_FALSE,
            0, 1, 0x1f, 0
        },
    },

    //BLUE1
    {
        TPS6586xPmuSupply_BLUE1,

        {TPS6586x_R53_RGB1BLUE, 0, 5, 0},   
        {TPS6586x_R52_RGB1GREEN, 7, 1, TPS6586x_RFF_INVALID},
        NULL, NULL, NULL,
        TPS6586x_RFF_INVALID,

        {
            NV_FALSE,
            0, 1, 0x1f, 0
        },
    },

    //RED2
    {
        TPS6586xPmuSupply_RED2,

        {TPS6586x_R54_RGB2RED, 0, 5, 0},   
        {TPS6586x_R55_RGB2GREEN, 7, 1, TPS6586x_RFF_INVALID},
        NULL, NULL, NULL,
        TPS6586x_RFF_INVALID,

        {
            NV_FALSE,
            0, 1, 0x1f, 0
        },
    },

    //GREEN2
    {
        TPS6586xPmuSupply_GREEN2,

        {TPS6586x_R55_RGB2GREEN, 0, 5, 0},   
        {TPS6586x_R55_RGB2GREEN, 7, 1, TPS6586x_RFF_INVALID},
        NULL, NULL, NULL,
        TPS6586x_RFF_INVALID,

        {
            NV_FALSE,
            0, 1, 0x1f, 0
        },
    },
 
    //BLUE2
    {
        TPS6586xPmuSupply_BLUE2,

        {TPS6586x_R56_RGB2BLUE, 0, 5, 0},
        {TPS6586x_R55_RGB2GREEN, 7, 1, TPS6586x_RFF_INVALID},
        NULL, NULL, NULL,
        TPS6586x_RFF_INVALID,

        {
            NV_FALSE,
            0, 1, 0x1f, 0
        },
    },

    //LED_PWM
    {
        TPS6586xPmuSupply_LED_PWM,

        {TPS6586x_RFF_INVALID, 0, 0, 0},   /* FIXME: how to get the output */
        {TPS6586x_RFF_INVALID, 0, 0, TPS6586x_RFF_INVALID},
        NULL, NULL, NULL,
        TPS6586x_RFF_INVALID,

        {
            NV_TRUE,
            0, 0, 0, 0
        },
    },

    //PWM
    {
        TPS6586xPmuSupply_PWM,

        {TPS6586x_RFF_INVALID, 0, 0, 0},   /* FIXME: how to get the output */
        {TPS6586x_RFF_INVALID, 0, 0, TPS6586x_RFF_INVALID},
        NULL, NULL, NULL,
        TPS6586x_RFF_INVALID,

        {
            NV_TRUE,
            0, 0, 0, 0
        },
    },

    //White LED(SM3)
    {
        TPS6586xPmuSupply_WHITE_LED,

        {TPS6586x_R58_SM3_SET1, 0, 3, 0},   
        {TPS6586x_R57_SM3_SET0, 0, 8, TPS6586x_RFF_INVALID}, 
        NULL, NULL, NULL,
        TPS6586x_RFF_INVALID,

        {
            NV_FALSE,
            25000, 0, 25000, 0
        },
    },
#if defined(CONFIG_TEGRA_ODM_HARMONY)
    {
        TPS6586xPmuSupply_SoC,

        {TPS6586x_RFF_INVALID, 0, 0, 0},
        {TPS6586x_R14_SUPPLYENE, 3, 1, TPS6586x_RFF_INVALID},
        NULL, NULL, NULL,
        TPS6586x_RFF_INVALID,
        {NV_FALSE,0,0,0,0},
    },
    // External Supplies

    {
        Ext_TPS62290PmuSupply_BUCK,

        {TPS6586x_RFF_INVALID, 0, 0, 0},
        {TPS6586x_RFF_INVALID, 0, 0, 0},
        NULL, NULL, NULL,
        3,
        {NV_FALSE,0,1050,1050,1050},
    },

    {
        Ext_TPS72012PmuSupply_LDO,

        {TPS6586x_RFF_INVALID, 0, 0, 0},
        {TPS6586x_RFF_INVALID, 0, 0, 0},
        NULL, NULL, NULL,
        2,
        {NV_FALSE,0,1200,1200,1200},
    },

    {
        Ext_TPS74201PmuSupply_LDO,

        {TPS6586x_RFF_INVALID, 0, 0, 0},
        {TPS6586x_RFF_INVALID, 0, 0, 0},
        NULL, NULL, NULL,
        1,
        {NV_FALSE,0,1500,1500,1500},
    },
    {
        Ext_TPS2051BPmuSupply_VDDIO_VID,

        {TPS6586x_RFF_INVALID, 0, 0, 0},
        {TPS6586x_RFF_INVALID, 0, 0, 0},
        NULL, NULL, NULL,
        TPS6586x_RFF_INVALID,
        {NV_FALSE,0,5000,5000,5000},
    },

    {
        Ext_SWITCHPmuSupply_VDDIO_SD,

        {TPS6586x_RFF_INVALID, 0, 0, 0},
        {TPS6586x_RFF_INVALID, 0, 0, 0},
        NULL, NULL, NULL,
        TPS6586x_RFF_INVALID,
        {NV_FALSE,0,3300,3300,3300},
    },

    {
        Ext_SWITCHPmuSupply_VDDIO_SDMMC,

        {TPS6586x_RFF_INVALID, 0, 0, 0},
        {TPS6586x_RFF_INVALID, 0, 0, 0},
        NULL, NULL, NULL,
        TPS6586x_RFF_INVALID,
        {NV_FALSE,0,3300,3300,3300},
    },
    {
        Ext_SWITCHPmuSupply_VDD_BL,

        {TPS6586x_RFF_INVALID, 0, 0, 0},
        {TPS6586x_RFF_INVALID, 0, 0, 0},
        NULL, NULL, NULL,
        TPS6586x_RFF_INVALID,
        {NV_FALSE,0,11100,11100,11100},
    },

    {
        Ext_SWITCHPmuSupply_VDD_PNL,

        {TPS6586x_RFF_INVALID, 0, 0, 0},
        {TPS6586x_R14_SUPPLYENE, 3, 1, TPS6586x_RFF_INVALID},
        NULL, NULL, NULL,
        TPS6586x_RFF_INVALID,
        {NV_FALSE,0,3300,3300,3300},
    }
#endif
};

static NvU32 TPS6586xPmuVoltageGetSM0(const NvU32 bits)
{
    NV_ASSERT(bits <= 0x1F);

    return (725 + bits * 25);
}

static NvU32 TPS6586xPmuVoltageGetSM1(const NvU32 bits)
{
    NV_ASSERT(bits <= 0x1F);

#if defined (CONFIG_TEGRA_ODM_HARMONY)
    return (725 + bits * 25);
#else
    return (1450 + bits * 50);
#endif
}

static NvU32 TPS6586xPmuVoltageGetSM2(const NvU32 bits)
{
    NV_ASSERT(bits <= 0x1F);

    return (3000 + bits * 25);
}

static NvU32 TPS6586xPmuVoltageGetVLDOx(const NvU32 bits)
{
    NV_ASSERT(bits <= 0x7);

    return VLDOx[bits];
}

static NvU32 TPS6586xPmuVoltageGetVLDO1(const NvU32 bits)
{
    /* VLDO1 and VLDO2 use the same table.
     * See pp. 64 - 66 of TPS658621 data sheet.
     */
    NV_ASSERT(bits <= 0x1F);

    return VLDO2[bits];
}

static NvU32 TPS6586xPmuVoltageGetVLDO2(const NvU32 bits)
{
    NV_ASSERT(bits <= 0x1F);
    return VLDO2[bits];
}
static NvU32 TPS6586xPmuVoltageGetVLDO4(const NvU32 bits)
{
    NV_ASSERT(bits <= 0x1F);
    return VLDO4[bits];
}

#ifndef MIN
#define MIN(a, b)   (a) <= (b) ? (a) : (b)
#endif

static NvU32 TPS6586xPmuVoltageSetSM0(const NvU32 millivolts)
{
    if (millivolts < 725)
        return 0;
    else
        return MIN((millivolts - 725) / 25, 0x1f);
}

static NvU32 TPS6586xPmuVoltageSetSM1(const NvU32 millivolts)
{
#if defined(CONFIG_TEGRA_ODM_HARMONY)
    if (millivolts < 725)
        return 0;
    else
        return MIN((millivolts - 725) / 25, 0x1f);
#else
    if (millivolts < 1450)
        return 0;
    else
        return MIN((millivolts - 1450) / 50, 0x1f);
#endif
}

static NvU32 TPS6586xPmuVoltageSetSM2(const NvU32 millivolts)
{
    if (millivolts < 3000)
        return 0;
    else 
        return MIN((millivolts - 3000) / 25, 0x1f);
}

#define VOLTAGE_SEARCH(mv, vtbl)        \
    const int cnt = sizeof(vtbl)/sizeof(NvU32);     \
    int i;                                          \
    for (i=0; i<cnt; i++)                           \
    {                                               \
        if (mv <= (vtbl)[i])                        \
            break;                                  \
    }                                               \
    return MIN(i, cnt-1)


static NvU32 TPS6586xPmuVoltageSetVLDOx(const NvU32 millivolts)
{
    VOLTAGE_SEARCH(millivolts, VLDOx);
}

static NvU32 TPS6586xPmuVoltageSetVLDO1(const NvU32 millivolts)
{
    /* VLDO1 and VLDO2 use the same table.
     * See pp. 64 - 66 of TPS658621 data sheet.
     */
    VOLTAGE_SEARCH(millivolts, VLDO2);
}

static NvU32 TPS6586xPmuVoltageSetVLDO2(const NvU32 millivolts)
{
    VOLTAGE_SEARCH(millivolts, VLDO2);
}
static NvU32 TPS6586xPmuVoltageSetVLDO4(const NvU32 millivolts)
{
    VOLTAGE_SEARCH(millivolts, VLDO4);
}


void
Tps6586xGetCapabilities(
    NvU32 vddRail,
    NvOdmPmuVddRailCapabilities* pCapabilities)
{
    NV_ASSERT(pCapabilities);
    NV_ASSERT(vddRail < TPS6586xPmuSupply_Num);

    *pCapabilities = tps6586xSupplyInfoTable[vddRail].cap;
}

#if defined(CONFIG_TEGRA_ODM_HARMONY)
static NvBool g_ExternalSupplyEnabled[TPS6586x_EXTERNAL_SUPPLY_NUM] = { 0 };

static NvBool
Tps6586xGetExternalSupply(
    NvOdmPmuDeviceHandle hDevice, 
    NvU32 vddRail,
    NvU32* pMilliVolts)
{
    if (g_ExternalSupplyEnabled[vddRail - (NvU32)(Ext_TPS62290PmuSupply_BUCK)] == NV_TRUE)
        *pMilliVolts = tps6586xSupplyInfoTable[vddRail].cap.requestMilliVolts;
    else
        *pMilliVolts = 0;

    return NV_TRUE;
}
#endif

static NvBool
Tps6586xReadVoltageReg(
        NvOdmPmuDeviceHandle hDevice,
        NvU32 vddRail,
        NvU32* pMilliVolts)
{
    const TPS6586xPmuSupplyInfo *pSupplyInfo = &tps6586xSupplyInfoTable[vddRail];
    NvU32 data = 0;

#if defined(CONFIG_TEGRA_ODM_HARMONY)
    // External supplies are fixed.
    if ((vddRail == Ext_TPS62290PmuSupply_BUCK)      ||
        (vddRail == Ext_TPS72012PmuSupply_LDO)       ||
        (vddRail == Ext_TPS74201PmuSupply_LDO)       ||
        (vddRail == Ext_TPS2051BPmuSupply_VDDIO_VID) ||
        (vddRail == Ext_SWITCHPmuSupply_VDDIO_SD)    ||
        (vddRail == Ext_SWITCHPmuSupply_VDDIO_SDMMC) ||
        (vddRail == Ext_SWITCHPmuSupply_VDD_BL)      ||
        (vddRail == Ext_SWITCHPmuSupply_VDD_PNL))
    {
        if (!Tps6586xGetExternalSupply(hDevice, vddRail, pMilliVolts))
            return NV_FALSE;
        else
            return NV_TRUE;
    }
#endif

    if (pSupplyInfo->supplyRegInfo.addr == TPS6586x_RFF_INVALID) 
    {
        return NV_FALSE;
    }

    if (! Tps6586xI2cRead8(hDevice, pSupplyInfo->supplyRegInfo.addr, &data))
        return NV_FALSE;

    if (pSupplyInfo->getVoltage)
        *pMilliVolts = pSupplyInfo->getVoltage((data >> pSupplyInfo->supplyRegInfo.start) & ((1<<pSupplyInfo->supplyRegInfo.bits)-1));
    else
        *pMilliVolts = data;

    Tps6586xI2cRead8(hDevice, pSupplyInfo->ctrlRegInfo.addr, &data);
    data &= (((1<<pSupplyInfo->ctrlRegInfo.bits)-1)<<pSupplyInfo->ctrlRegInfo.start);
    if (data == 0)
    {
        //since Voltage table is  {1250, 1500, 1800, 2500, 2700, 2850, 3100, 3300} 
        //need to fill 0 voltage if needed
        *pMilliVolts = data;
    } 


    return NV_TRUE;
}

#if 0
static NvBool
Tps6586xSupplyCtrl(
        const NvU32 vddRail, 
        NvBool ctrl)
{
    return NV_TRUE;
}
#endif

#if defined(CONFIG_TEGRA_ODM_HARMONY)
#define NVODM_PORT(x) ((x) - 'a')

static NvBool
Tps6586xSetExternalSupply(
    NvOdmPmuDeviceHandle hDevice, 
    NvU32 vddRail,
    NvBool Enable)
{
    const TPS6586xPmuSupplyInfo* pSupplyInfo = &tps6586xSupplyInfoTable[vddRail];
    NvU32 data = 0;
    NvU8 Gpio;
    NvU32 GpioPort;
    NvU32 GpioPin;

    NV_ASSERT((vddRail > TPS6586xPmuSupply_Invalid) && (vddRail < TPS6586xPmuSupply_Num));

    // FIXME: Clean this up!  This includes embedding more of these settings in
    // the supply info table to simplify the code.

    // Switched by PMU GPIO
    if ((vddRail == Ext_TPS62290PmuSupply_BUCK)||
        (vddRail == Ext_TPS72012PmuSupply_LDO) ||
        (vddRail == Ext_TPS74201PmuSupply_LDO))
    {
        // Set output mode (TEST: FOR GPIO1 only)

        Gpio = pSupplyInfo->Gpio;
        switch (Gpio)
        {
        case 1:
            if (!Tps6586xI2cRead8(hDevice, TPS6586x_R5D_GPIOSET1, &data))
                return NV_FALSE;

            // Reset mode field
            data &= ~(TPS6586x_R5D_GPIOSET1_GPIO1_MODE_MASK <<
                      TPS6586x_R5D_GPIOSET1_GPIO1_MODE_SHIFT);
            // Apply new mode setting (output)
            data |= (TPS6586x_R5D_GPIO_MODE_OUTPUT <<
                     TPS6586x_R5D_GPIOSET1_GPIO1_MODE_SHIFT);
            if (!Tps6586xI2cWrite8(hDevice, TPS6586x_R5D_GPIOSET1, data))
                return NV_FALSE;

            if (Enable)
            {
                // Enable output
                if (!Tps6586xI2cRead8(hDevice, TPS6586x_R5E_GPIOSET2, &data))
                    return NV_FALSE;

                data |= (TPS6586x_R5E_GPIOSET2_GPIO1_OUT_MASK <<
                         TPS6586x_R5E_GPIOSET2_GPIO1_OUT_SHIFT);
                if (!Tps6586xI2cWrite8(hDevice, TPS6586x_R5E_GPIOSET2, data))
                    return NV_FALSE;
            }
            else
            {
                // Disable output
                if (!Tps6586xI2cRead8(hDevice, TPS6586x_R5E_GPIOSET2, &data))
                    return NV_FALSE;

                data &= ~(TPS6586x_R5E_GPIOSET2_GPIO1_OUT_MASK <<
                          TPS6586x_R5E_GPIOSET2_GPIO1_OUT_SHIFT);
                if (!Tps6586xI2cWrite8(hDevice, TPS6586x_R5E_GPIOSET2, data))
                    return NV_FALSE;
            }
            break;

        case 2:
            if (!Tps6586xI2cRead8(hDevice, TPS6586x_R5D_GPIOSET1, &data))
                return NV_FALSE;

            // Reset mode field
            data &= ~(TPS6586x_R5D_GPIOSET1_GPIO2_MODE_MASK <<
                      TPS6586x_R5D_GPIOSET1_GPIO2_MODE_SHIFT);
            // Apply new mode setting (output)
            data |= (TPS6586x_R5D_GPIO_MODE_OUTPUT <<
                     TPS6586x_R5D_GPIOSET1_GPIO2_MODE_SHIFT);
            if (!Tps6586xI2cWrite8(hDevice, TPS6586x_R5D_GPIOSET1, data))
                return NV_FALSE;

            if (Enable)
            {
                // Enable output
                if (!Tps6586xI2cRead8(hDevice, TPS6586x_R5E_GPIOSET2, &data))
                    return NV_FALSE;

                data |= (TPS6586x_R5E_GPIOSET2_GPIO2_OUT_MASK <<
                         TPS6586x_R5E_GPIOSET2_GPIO2_OUT_SHIFT);
                if (!Tps6586xI2cWrite8(hDevice, TPS6586x_R5E_GPIOSET2, data))
                    return NV_FALSE;
            }
            else
            {
                // Disable output
                if (!Tps6586xI2cRead8(hDevice, TPS6586x_R5E_GPIOSET2, &data))
                    return NV_FALSE;

                data &= ~(TPS6586x_R5E_GPIOSET2_GPIO2_OUT_MASK <<
                          TPS6586x_R5E_GPIOSET2_GPIO2_OUT_SHIFT);
                if (!Tps6586xI2cWrite8(hDevice, TPS6586x_R5E_GPIOSET2, data))
                    return NV_FALSE;
            }
            break;

        case 3:
            if (!Tps6586xI2cRead8(hDevice, TPS6586x_R5D_GPIOSET1, &data))
                return NV_FALSE;
            // Reset mode field
            data &= ~(TPS6586x_R5D_GPIOSET1_GPIO3_MODE_MASK <<
                      TPS6586x_R5D_GPIOSET1_GPIO3_MODE_SHIFT);
            // Apply new mode setting (output)
            data |= (TPS6586x_R5D_GPIO_MODE_OUTPUT <<
                     TPS6586x_R5D_GPIOSET1_GPIO3_MODE_SHIFT);
            if (!Tps6586xI2cWrite8(hDevice, TPS6586x_R5D_GPIOSET1, data))
                return NV_FALSE;

            if (Enable)
            {
                // Enable output
                if (!Tps6586xI2cRead8(hDevice, TPS6586x_R5E_GPIOSET2, &data))
                    return NV_FALSE;

                data |= (TPS6586x_R5E_GPIOSET2_GPIO3_OUT_MASK <<
                         TPS6586x_R5E_GPIOSET2_GPIO3_OUT_SHIFT);
                if (!Tps6586xI2cWrite8(hDevice, TPS6586x_R5E_GPIOSET2, data))
                    return NV_FALSE;
            }
            else
            {
                // Disable output
                if (!Tps6586xI2cRead8(hDevice, TPS6586x_R5E_GPIOSET2, &data))
                    return NV_FALSE;

                data &= ~(TPS6586x_R5E_GPIOSET2_GPIO3_OUT_MASK <<
                          TPS6586x_R5E_GPIOSET2_GPIO3_OUT_SHIFT);
                if (!Tps6586xI2cWrite8(hDevice, TPS6586x_R5E_GPIOSET2, data))
                    return NV_FALSE;
            }
            break;

        default:
            return NV_FALSE; // bad parameter
        }
    }
    // Switched by AP GPIO
    else
    {
        // Open GPIO
        if (!hPmu->hGpio)
        {
            hPmu->hGpio = NvOdmGpioOpen();

            if (!hPmu->hGpio)
                return NV_FALSE;
        }

        // Get AP GPIO pin assigned to the respective voltage rail
        // FIXME:  This should be driven by supply info table.
        switch (vddRail)
        {
        case Ext_TPS2051BPmuSupply_VDDIO_VID:
            GpioPort = NVODM_PORT('t');
            GpioPin  = 2;
            break;

        case Ext_SWITCHPmuSupply_VDDIO_SD:
            GpioPort = NVODM_PORT('t');
            GpioPin  = 3;
            break;

        case Ext_SWITCHPmuSupply_VDDIO_SDMMC:
            GpioPort = NVODM_PORT('i');
            GpioPin  = 6;
            break;

        case Ext_SWITCHPmuSupply_VDD_BL:
            GpioPort = NVODM_PORT('w');
            GpioPin  = 0;
            break;

        case Ext_SWITCHPmuSupply_VDD_PNL:
            GpioPort = NVODM_PORT('c');
            GpioPin  = 6;
            break;

        default:
            return NV_FALSE;
        }

        NV_ASSERT((NVODM_EXT_AP_GPIO_RAIL(vddRail) >= 0) &&
                  (NVODM_EXT_AP_GPIO_RAIL(vddRail) < TPS6586x_EXTERNAL_SUPPLY_AP_GPIO_NUM));

        // Acquire Pin Handle
        if (!hPmu->hPin[NVODM_EXT_AP_GPIO_RAIL(vddRail)])
        {
            hPmu->hPin[NVODM_EXT_AP_GPIO_RAIL(vddRail)] = 
                NvOdmGpioAcquirePinHandle(hPmu->hGpio, GpioPort, GpioPin);

            if (!hPmu->hPin[NVODM_EXT_AP_GPIO_RAIL(vddRail)])
                return NV_FALSE;
        }

        if (Enable)
        {
            if (vddRail == Ext_TPS2051BPmuSupply_VDDIO_VID)
            {
                NvOdmGpioConfig(hPmu->hGpio,
                                hPmu->hPin[NVODM_EXT_AP_GPIO_RAIL(vddRail)],
                                NvOdmGpioPinMode_Tristate);
            }
            else
            {
                NvOdmGpioSetState(hPmu->hGpio,
                                  hPmu->hPin[NVODM_EXT_AP_GPIO_RAIL(vddRail)],
                                  NvOdmGpioPinActiveState_High);

                NvOdmGpioConfig(hPmu->hGpio,
                                hPmu->hPin[NVODM_EXT_AP_GPIO_RAIL(vddRail)],
                                NvOdmGpioPinMode_Output);
            }
        }
        else
        {
            if (vddRail != Ext_TPS2051BPmuSupply_VDDIO_VID)
            {
                NvOdmGpioSetState(hPmu->hGpio,
                              hPmu->hPin[NVODM_EXT_AP_GPIO_RAIL(vddRail)],
                              NvOdmGpioPinActiveState_Low);
                NvOdmGpioConfig(hPmu->hGpio,
                            hPmu->hPin[NVODM_EXT_AP_GPIO_RAIL(vddRail)],
                            NvOdmGpioPinMode_Output);
            }
        }
    }

    // This isn't thread safe, but it's highly unlikely that will be an issue for these rails.
    if (Enable)
    {
        g_ExternalSupplyEnabled[vddRail - (NvU32)(Ext_TPS62290PmuSupply_BUCK)] = NV_TRUE;
    }
    else
    {
        g_ExternalSupplyEnabled[vddRail - (NvU32)(Ext_TPS62290PmuSupply_BUCK)] = NV_FALSE;
    }
    return NV_TRUE;
}
#endif

static NvBool 
Tps6586xWriteVoltageReg(
    NvOdmPmuDeviceHandle hDevice, 
    NvU32 vddRail, 
    NvU32  MilliVolts, 
    NvU32* pSettleMicroSeconds)
{
    const TPS6586xPmuSupplyInfo* pSupplyInfo = &tps6586xSupplyInfoTable[vddRail];
    //const TPS6586xPmuSupplyInfo* pSupplyInputInfo = &tps6586xSupplyInfoTable[pSupplyInfo->supplyInput];
    NvBool status = NV_FALSE;
    NvU32  settleTime = 0;
#if !defined(CONFIG_TEGRA_ODM_HARMONY)
    NvU32  volChange = 0;
#endif

    NV_ASSERT(pSupplyInfo->supply == (TPS6586xPmuSupply)vddRail);

#if defined(CONFIG_TEGRA_ODM_HARMONY)
    if ((vddRail != Ext_TPS62290PmuSupply_BUCK)      &&
        (vddRail != Ext_TPS72012PmuSupply_LDO)       &&
        (vddRail != Ext_TPS74201PmuSupply_LDO)       &&
        (vddRail != Ext_TPS2051BPmuSupply_VDDIO_VID) &&
        (vddRail != Ext_SWITCHPmuSupply_VDDIO_SD)    &&
        (vddRail != Ext_SWITCHPmuSupply_VDDIO_SDMMC) &&
        (vddRail != Ext_SWITCHPmuSupply_VDD_BL)      &&
        (vddRail != Ext_SWITCHPmuSupply_VDD_PNL))
#endif
    {
        if (pSupplyInfo->ctrlRegInfo.addr == TPS6586x_RFF_INVALID)
        {
            NVODMPMU_PRINTF(("TPS:The required ctrl register address is INVALID...\n"));
#if defined(CONFIG_TEGRA_ODM_HARMONY)
            return NV_TRUE;
            //return NV_FALSE;
#else
            return NV_FALSE;
#endif
        }
    }

    if (MilliVolts == ODM_VOLTAGE_OFF) 
    {
        NvU32 data = 0;

        // check if the supply can be turned off
        //NV_ASSERT(hDevice->supplyRefCntTable[vddRail]);

#if defined(CONFIG_TEGRA_ODM_HARMONY)
        // Disable external supplies
        if ((vddRail == Ext_TPS62290PmuSupply_BUCK)      ||
            (vddRail == Ext_TPS72012PmuSupply_LDO)       ||
            (vddRail == Ext_TPS74201PmuSupply_LDO)       ||
            (vddRail == Ext_TPS2051BPmuSupply_VDDIO_VID) ||
            (vddRail == Ext_SWITCHPmuSupply_VDDIO_SD)    ||
            (vddRail == Ext_SWITCHPmuSupply_VDDIO_SDMMC) ||
            (vddRail == Ext_SWITCHPmuSupply_VDD_BL)      ||
            (vddRail == Ext_SWITCHPmuSupply_VDD_PNL))
        {
            status = Tps6586xSetExternalSupply(hDevice, vddRail, NV_FALSE);
        }
        else
#endif
        if (((NvOdmPmuDeviceTPS *)(hDevice->pPrivate))->supplyRefCntTable[vddRail] == 1)
        {
            /* Disable */
            NvOdmServicesPmuSetSocRailPowerState(
                ((NvOdmPmuDeviceTPS *)(hDevice->pPrivate))->hOdmPmuSevice, pSupplyInfo->supply, NV_FALSE); 
            Tps6586xI2cRead8(hDevice, pSupplyInfo->ctrlRegInfo.addr, &data);
#if defined(CONFIG_TEGRA_ODM_HARMONY)
            if (vddRail == TPS6586xPmuSupply_SoC)
            {
                // SOC Super power rail don't hold the sleep bit
                data |= (((1<<pSupplyInfo->ctrlRegInfo.bits)-1)<<pSupplyInfo->ctrlRegInfo.start);
            }
            else
#endif
            {
                data &= ~(((1<<pSupplyInfo->ctrlRegInfo.bits)-1)<<pSupplyInfo->ctrlRegInfo.start);
            }
            status = Tps6586xI2cWrite8(hDevice, pSupplyInfo->ctrlRegInfo.addr, data);
#if defined(CONFIG_TEGRA_ODM_HARMONY)
            if (vddRail == TPS6586xPmuSupply_SoC)
            {
                // Wait 10 secs for PMU to shutdown
                NvOdmOsWaitUS(100000000);
            }
#endif

            if (status && (pSupplyInfo->ctrlRegInfo.flag != TPS6586x_RFF_INVALID))
            {
                status = Tps6586xI2cRead8(hDevice, pSupplyInfo->ctrlRegInfo.flag, &data);
                if (status) 
                {
                    data &= ~(((1<<pSupplyInfo->ctrlRegInfo.bits)-1)<<pSupplyInfo->ctrlRegInfo.start);
                    status = Tps6586xI2cWrite8(hDevice, pSupplyInfo->ctrlRegInfo.flag, data);
                }
            }

            /* Reset to voltage to 0th */
            MilliVolts = 0;
            
#if !defined(CONFIG_TEGRA_ODM_HARMONY)
            // Calcuate this voltage change
            volChange = ((NvOdmPmuDeviceTPS *)(hDevice->pPrivate))->curVoltageTable[vddRail];

            ((NvOdmPmuDeviceTPS *)(hDevice->pPrivate))->curVoltageTable[vddRail] = 0;
            
            // DCD0/DCD1/DCD2
            if ((vddRail == TPS6586xPmuSupply_DCD0) ||
                (vddRail == TPS6586xPmuSupply_DCD1) ||
                (vddRail == TPS6586xPmuSupply_DCD2))
            {
                // delay = Tstart(210) + Voltage change/Kramp
                settleTime = T_START_TIME + volChange/K_RAMP;
            }
            else if ((vddRail == TPS6586xPmuSupply_LDO2) ||
                     (vddRail == TPS6586xPmuSupply_LDO4))
            {
                // Voltage change/Kramp
                settleTime = volChange/K_RAMP;
            }
            else
            {
                settleTime = 0;
            }
#endif
        }

        if (((NvOdmPmuDeviceTPS *)(hDevice->pPrivate))->supplyRefCntTable[vddRail] != 0)
        {
#if defined(CONFIG_TEGRA_ODM_HARMONY)
            if(--((NvOdmPmuDeviceTPS *)(hDevice->pPrivate))->supplyRefCntTable[vddRail] != 0)
            {
                if (pSettleMicroSeconds)
                    *pSettleMicroSeconds = 0;
                return NV_TRUE;
            }
#else
            --((NvOdmPmuDeviceTPS *)(hDevice->pPrivate))->supplyRefCntTable[vddRail];
#endif
        }

#if !defined(CONFIG_TEGRA_ODM_HARMONY)        
        if (pSettleMicroSeconds != NULL)
            *pSettleMicroSeconds = settleTime;
        else
            NvOdmOsWaitUS(settleTime);
            
        return NV_TRUE;
#endif
    }
    else
    {    
        NvU32 data = 0;

        if (((NvOdmPmuDeviceTPS *)(hDevice->pPrivate))->supplyRefCntTable[vddRail]++ == 0)
        {
            // Enable external supplies
#if defined(CONFIG_TEGRA_ODM_HARMONY)
            if ((vddRail == Ext_TPS62290PmuSupply_BUCK)      ||
                (vddRail == Ext_TPS72012PmuSupply_LDO)       ||
                (vddRail == Ext_TPS74201PmuSupply_LDO)       ||
                (vddRail == Ext_TPS2051BPmuSupply_VDDIO_VID) ||
                (vddRail == Ext_SWITCHPmuSupply_VDDIO_SD)    ||
                (vddRail == Ext_SWITCHPmuSupply_VDDIO_SDMMC) ||
                (vddRail == Ext_SWITCHPmuSupply_VDD_BL)      ||
                (vddRail == Ext_SWITCHPmuSupply_VDD_PNL))
            {
                status = Tps6586xSetExternalSupply(hDevice, vddRail, NV_TRUE);
            }
            else
#endif
            {
#if !defined(CONFIG_TEGRA_ODM_HARMONY)
                NvOdmServicesPmuSetSocRailPowerState(
                    ((NvOdmPmuDeviceTPS *)(hDevice->pPrivate))->hOdmPmuSevice, pSupplyInfo->supply, NV_TRUE); 
#endif
                status = Tps6586xI2cRead8(hDevice, pSupplyInfo->ctrlRegInfo.addr, &data);
                if (status && ((data >> pSupplyInfo->ctrlRegInfo.start) & 0x1) == 0)
                {
#if defined(CONFIG_TEGRA_ODM_HARMONY)
                    /* Enable */
                    NvOdmServicesPmuSetSocRailPowerState(
                        ((NvOdmPmuDeviceTPS *)(hDevice->pPrivate))->hOdmPmuSevice,
                        pSupplyInfo->supply, NV_TRUE);
#endif
                    data |= (((1<<pSupplyInfo->ctrlRegInfo.bits)-1)<<pSupplyInfo->ctrlRegInfo.start);
                    if (NV_FALSE == Tps6586xI2cWrite8(hDevice, pSupplyInfo->ctrlRegInfo.addr, data))
#if defined(CONFIG_TEGRA_ODM_HARMONY)
                        return NV_TRUE;
#else
                        return NV_FALSE;
#endif
                }
            }
        }

        /* Disable Flash mode 
         * FIXME: please check whether you need to disable flash */
        /*
        if (vddRail == TPS6586xPmuSupply_RED1 ||
            vddRail == TPS6586xPmuSupply_GREEN1 ||
            vddRail == TPS6586xPmuSupply_BLUE1)
        {
            if (NV_FALSE == Tps6586xI2cWrite8(hDevice, TPS6586x_R50_RGB1FLASH, 0xFF))
                return NV_FALSE;
        } 
        */
    }

    if (pSupplyInfo->supplyRegInfo.addr == TPS6586x_RFF_INVALID) 
    {
#if defined(CONFIG_TEGRA_ODM_HARMONY)
        return NV_TRUE;
#else
        return NV_FALSE;
#endif
    }
    else
    {
        const int bits = pSupplyInfo->setVoltage ? pSupplyInfo->setVoltage(MilliVolts) : MilliVolts;
        NvU32 data = 0;

#if defined(CONFIG_TEGRA_ODM_HARMONY)
        status = Tps6586xI2cRead8(hDevice, pSupplyInfo->supplyRegInfo.addr, &data);
        if (NV_FALSE == status)
            NVODMPMU_PRINTF(("TPS:Writing to PMU I2C fails 1... supplyaddress: %d\n", pSupplyInfo->supplyRegInfo.addr));
#else
        if ((vddRail == TPS6586xPmuSupply_DCD0) ||
            (vddRail == TPS6586xPmuSupply_DCD1) ||
            (vddRail == TPS6586xPmuSupply_LDO4) ||
            (vddRail == TPS6586xPmuSupply_LDO2))
        {
            status = NV_TRUE;
        }
        else
        {
            status = Tps6586xI2cRead8(hDevice, pSupplyInfo->supplyRegInfo.addr, &data);
        }
#endif

        if (status) 
        {
            data &= ~(((1<<pSupplyInfo->supplyRegInfo.bits)-1)<<pSupplyInfo->supplyRegInfo.start);
            data |= (bits << pSupplyInfo->supplyRegInfo.start);
            status = Tps6586xI2cWrite8(hDevice, pSupplyInfo->supplyRegInfo.addr, data);

            /* Trigger a voltage change for SM0/SM1/LDO2/LDO4 */
            if ((vddRail == TPS6586xPmuSupply_DCD0) ||
                (vddRail == TPS6586xPmuSupply_DCD1) ||
                (vddRail == TPS6586xPmuSupply_LDO4) ||
                (vddRail == TPS6586xPmuSupply_LDO2))
            {
                data = 0;
                switch (vddRail) 
                {
                case TPS6586xPmuSupply_LDO2:
                    data |=  (1<<4);
                    data &=  ~(1<<5);
                    break;

                case TPS6586xPmuSupply_LDO4:
                    data |=  (1<<6);
                    data &=  ~(1<<7);
                    break;

                case TPS6586xPmuSupply_DCD0:
                    data |=  (1<<2);
                    data &=  ~(1<<3);
                    break;

                case TPS6586xPmuSupply_DCD1:
                    data |=  (1<<0);
                    data &=  ~(1<<1);
                    break;
                }
                status = Tps6586xI2cWrite8(hDevice, TPS6586x_R20_VCC1, data);
            }
            
#if !defined(CONFIG_TEGRA_ODM_HARMONY)
            // Calcuate this voltage change
            if (((NvOdmPmuDeviceTPS *)(hDevice->pPrivate))->curVoltageTable[vddRail] < MilliVolts)
                volChange = MilliVolts - ((NvOdmPmuDeviceTPS *)(hDevice->pPrivate))->curVoltageTable[vddRail];
            else
                volChange = ((NvOdmPmuDeviceTPS *)(hDevice->pPrivate))->curVoltageTable[vddRail] - MilliVolts;
            
            ((NvOdmPmuDeviceTPS *)(hDevice->pPrivate))->curVoltageTable[vddRail] = MilliVolts;
            
            // DCD0/DCD1/DCD2
            if ((vddRail == TPS6586xPmuSupply_DCD0) ||
                (vddRail == TPS6586xPmuSupply_DCD1) ||
                (vddRail == TPS6586xPmuSupply_DCD2))
            {
                // delay = Tstart(210) + Voltage change/Kramp
                settleTime = T_START_TIME + volChange/K_RAMP;
            }
            else if ((vddRail == TPS6586xPmuSupply_LDO2) ||
                     (vddRail == TPS6586xPmuSupply_LDO4))
            {
                // Voltage change/Kramp
                settleTime = volChange/K_RAMP;
            }
            else
            {
                settleTime = 0;
            }
#else
            settleTime = 250;
#endif
            
            if (pSettleMicroSeconds)
                *pSettleMicroSeconds = settleTime;
            else
                NvOdmOsWaitUS(settleTime);
            return status;
        }
    }

    if (pSupplyInfo->supplyRegInfo.addr == TPS6586x_RFF_INVALID) 
    {
#if defined(CONFIG_TEGRA_ODM_HARMONY)
        return NV_TRUE;
#else
        return NV_FALSE;
#endif
    }

    if (pSettleMicroSeconds)
#if defined(CONFIG_TEGRA_ODM_HARMONY)
        *pSettleMicroSeconds = 250;
    else
        NvOdmOsWaitUS(250);
#else
        *pSettleMicroSeconds = 0;
#endif
 
    return NV_TRUE;
}

NvBool
Tps6586xGetVoltage(
    NvOdmPmuDeviceHandle hDevice,
    NvU32 vddRail,
    NvU32* pMilliVolts)
{   
    NV_ASSERT(hDevice);
    NV_ASSERT(pMilliVolts);
    NV_ASSERT(vddRail < TPS6586xPmuSupply_Num);

    if(! Tps6586xReadVoltageReg(hDevice, vddRail,pMilliVolts))
        return NV_FALSE;
    
    return NV_TRUE;
}

NvBool
Tps6586xSetVoltage(
    NvOdmPmuDeviceHandle hDevice,
    NvU32 vddRail,
    NvU32 MilliVolts,
    NvU32* pSettleMicroSeconds)
{
    NV_ASSERT(hDevice);
    NV_ASSERT(vddRail < TPS6586xPmuSupply_Num);

    if (pSettleMicroSeconds)
        *pSettleMicroSeconds = 0;

    if (tps6586xSupplyInfoTable[vddRail].cap.OdmProtected == NV_TRUE)
    {
        NVODMPMU_PRINTF(("The voltage is protected and can not be set.\n"));
        return NV_TRUE;
    }

    if ((MilliVolts == ODM_VOLTAGE_OFF) ||
        ((MilliVolts <= tps6586xSupplyInfoTable[vddRail].cap.MaxMilliVolts) &&
         (MilliVolts >= tps6586xSupplyInfoTable[vddRail].cap.MinMilliVolts)))
    {
        if (! Tps6586xWriteVoltageReg(hDevice, vddRail, MilliVolts, pSettleMicroSeconds))
            return NV_FALSE;
    }
    else
    {
        NVODMPMU_PRINTF(("The required voltage is not supported..\n"));
#if defined(CONFIG_TEGRA_ODM_HARMONY)
        return NV_TRUE;
        //return NV_FALSE;
#else
        return NV_FALSE;
#endif
    }

    return NV_TRUE;
}

#if 0
void DumpTps6586x(NvOdmPmuDeviceHandle hDevice)
{
    int i;
    NvU32 data;
    for (i=0; i<0xFF; i++) 
    {
        data = 0;
        Tps6586xI2cRead8(hDevice, i, &data);
        NVODMPMU_PRINTF(("Register 0x%x = 0x%x\n", i, data));
    }
}
#endif

NvBool Tps6586xSetup(NvOdmPmuDeviceHandle hDevice)
{
    NvOdmIoModule I2cModule = NvOdmIoModule_I2c;
    NvU32 I2cInstance = 0;
    NvU32 I2cAddress  = 0;
    NvBool status     = NV_FALSE;
    NvU32 data = 0;    
//    static TPS6586xDevice s_tps6586x = {0};
    
    const NvOdmPeripheralConnectivity *pConnectivity = 
                           NvOdmPeripheralGetGuid(PMUGUID);

    NV_ASSERT(hDevice);
    
    hPmu = (NvOdmPmuDeviceTPS *)NvOdmOsAlloc(sizeof(NvOdmPmuDeviceTPS));
    if (hPmu == NULL)
    {
        NVODMPMU_PRINTF(("Error Allocating NvOdmPmuDeviceTPS. \n"));
        return NV_FALSE;
    }
    NvOdmOsMemset(hPmu, 0, sizeof(NvOdmPmuDeviceTPS));

    hDevice->pPrivate = hPmu;

    if (pConnectivity != NULL) // PMU is in database
    {
        NvU32 i = 0;
        pmuPresented = NV_TRUE;
        
        for (i = 0; i < pConnectivity->NumAddress; i ++)
        {
            if (pConnectivity->AddressList[i].Interface == NvOdmIoModule_I2c_Pmu)
            {
                I2cModule   = NvOdmIoModule_I2c_Pmu;
                I2cInstance = pConnectivity->AddressList[i].Instance;
                I2cAddress  = pConnectivity->AddressList[i].Address;
                break;
            }
        }

        NV_ASSERT(I2cModule  == NvOdmIoModule_I2c_Pmu);
        NV_ASSERT(I2cAddress != 0);

        hPmu->hOdmI2C = NvOdmI2cOpen(I2cModule, I2cInstance);
        if (!hPmu->hOdmI2C)
        {
            NVODMPMU_PRINTF(("[NVODM PMU]Tps6586xSetup: Error Open I2C device. \n"));
            NVODMPMU_PRINTF(("[NVODM PMU]Please check PMU device I2C settings. \n"));
            goto OPEN_FAILED;
        }
        hPmu->DeviceAddr = I2cAddress;
        hPmu->hOdmPmuSevice = NvOdmServicesPmuOpen();
#if defined(CONFIG_TEGRA_ODM_HARMONY)
        //if (NV_FALSE == Tps6586xWriteVoltageReg(hDevice, TPS6586xPmuSupply_LDO5, 3300, NULL))
        if (NV_FALSE == Tps6586xWriteVoltageReg(hDevice, TPS6586xPmuSupply_LDO5, 2850, NULL))
            NVODMPMU_PRINTF(("TPS: Fail to set the NVDDIO_NAND to 2.85V\n"));
        else
            NVODMPMU_PRINTF(("TPS: set the NVDDIO_NAND to 2.85V\n"));
#endif
    }   
    else
    {
        // if PMU is not presented in the database, then the platform is PMU-less.
        NVODMPMU_PRINTF(("[NVODM PMU]Tps6586xSetup: The system did not doscover PMU fromthe data base. \n"));     
        NVODMPMU_PRINTF(("[NVODM PMU]Tps6586xSetup: If this is not intended, please check the peripheral database for PMU settings. \n"));     

        //uncomment below line if you really need to run pmu adaptation on  pmu-less system.
        // the system will run in pmu-less mode.
        //pmuPresented = NV_FALSE;

        goto OPEN_FAILED;
    }

    //hDevice->priv = &s_tps6586x;

    /* Check Chip Device ID to verify it */
#if 0
    I2cInstance = 0;
    if (!Tps6586xI2cRead8(hDevice, TPS6586x_RCD_VERSIONID, &I2cInstance) || I2cInstance != 0x60)
    {
        NV_ASSERT(!"did not find TSP6586x");
        goto OPEN_FAILED;
    }
#endif

#if defined(CONFIG_TEGRA_ODM_HARMONY)
    /* Initialize the refcont of the rails which are ON by default */
    hPmu->supplyRefCntTable[TPS6586xPmuSupply_SoC] = 1;
#endif

#if !defined(CONFIG_TEGRA_ODM_HARMONY)
    // If your project doesn't use this GPIO, please delete them!
    // Enable GPIO3 to HIGH
    // Set GPIO Configure to output
    data = 0x10;
    if (!Tps6586xI2cWrite8(hDevice, TPS6586x_R5D_GPIOSET1, data))
        return NV_FALSE;
        
    // Set GPIO to HI, NON-Inverting
    data = 0x04;
    if (!Tps6586xI2cWrite8(hDevice, TPS6586x_R5E_GPIOSET2, data))
        return NV_FALSE;
#endif

    //NvOdmOsMemset(&s_tps6586x, 0, sizeof(s_tps6586x));
    //s_tps6586x.pmuPresented = NV_TRUE;
    pmuPresented = NV_TRUE;

#if NV_DEBUG
    //DumpTps6586x(hDevice);
#endif 

    if (!Tps6586xBatteryChargerSetup(hDevice))
        return NV_FALSE;

    // The interrupt assumes not supported until tps6586xInterruptHandler() is called. 
    pmuInterruptSupported = NV_FALSE;

    // setup the interrupt any way.
    if (!Tps6586xSetupInterrupt(hDevice, &pmuStatus))
        return NV_FALSE;

    //Check battery presence
    if (!Tps6586xBatteryChargerCBCMainBatt(hDevice, &battPresence))
        return NV_FALSE;
   
    // Check battery Fullness
    if (battPresence == NV_TRUE)
    {   
        if (!Tps6586xBatteryChargerCBCBattFul(hDevice, &status))
            return NV_FALSE;    
        pmuStatus.batFull = status;
    }
    else
    {
        pmuStatus.batFull = NV_FALSE;
    }

    return NV_TRUE;

OPEN_FAILED:
    Tps6586xRelease(hDevice);
    return NV_FALSE;
}

void Tps6586xRelease(NvOdmPmuDeviceHandle hDevice)
{
#if defined(CONFIG_TEGRA_ODM_HARMONY)
    NvU32 i;
#endif
    if (hDevice != NULL && hPmu->hOdmI2C)
    {
        NvOdmServicesPmuClose(hPmu->hOdmPmuSevice);
        hPmu->hOdmPmuSevice = NULL;
        NvOdmI2cClose(hPmu->hOdmI2C);
        hPmu->hOdmI2C = NULL;
        NvOdmOsFree(hPmu);
        hPmu = NULL;

#if defined(CONFIG_TEGRA_ODM_HARMONY)

        // Release & Close the GPIOs
        for (i=0; i<(TPS6586x_EXTERNAL_SUPPLY_AP_GPIO_NUM); i++)
        {
            NvOdmGpioReleasePinHandle(hPmu->hGpio, hPmu->hPin[i]);
        }
        NvOdmGpioClose(hPmu->hGpio);
#endif
    }    
}

NvBool
Tps6586xGetAcLineStatus(
    NvOdmPmuDeviceHandle hDevice,
    NvOdmPmuAcLineStatus *pStatus)
{
    NvBool acLineStatus = NV_FALSE;
   
    NV_ASSERT(hDevice);
    NV_ASSERT(pStatus);
   
    // check if charger presents
    if (battPresence == NV_FALSE)
    {
        *pStatus = NvOdmPmuAcLine_Online;
        return NV_TRUE;
    }

    if (pmuInterruptSupported == NV_TRUE)
    {
        if ( pmuStatus.mChgPresent == NV_TRUE )
        {
            *pStatus = NvOdmPmuAcLine_Online;
            acLineStatus = NV_TRUE;
        }
        else
        {
            *pStatus = NvOdmPmuAcLine_Offline;
            acLineStatus = NV_FALSE;
        }
    }
    else
    {
        // battery is present, now check if charger presents
        if (!Tps6586xBatteryChargerMainChgPresent(hDevice, &acLineStatus))
        {
            NVODMPMU_PRINTF(("Error in checking main charger presence.\n"));
            return NV_FALSE;
        }

        if (acLineStatus == NV_TRUE)
            *pStatus = NvOdmPmuAcLine_Online;
        else
            *pStatus = NvOdmPmuAcLine_Offline;
    }    
    return NV_TRUE;
}

NvBool 
Tps6586xGetBatteryStatus(
    NvOdmPmuDeviceHandle hDevice, 
    NvOdmPmuBatteryInstance batteryInst,
    NvU8 *pStatus)
{
    NvU8 status = 0;
    
    NV_ASSERT(hDevice);
    NV_ASSERT(pStatus);
    NV_ASSERT(batteryInst <= NvOdmPmuBatteryInst_Num);

    if (batteryInst == NvOdmPmuBatteryInst_Main)
    {
        if (battPresence == NV_TRUE)
        {
            NvOdmPmuAcLineStatus stat = NvOdmPmuAcLine_Offline;
            NvU32 VBatSense = 0;
            if (!Tps6586xGetAcLineStatus(hDevice, &stat))
                return NV_FALSE;
            
            if (stat == NvOdmPmuAcLine_Online)
            {
                if (pmuInterruptSupported == NV_TRUE)
                {
                    if (pmuStatus.batFull == NV_FALSE)
                        status = NVODM_BATTERY_STATUS_CHARGING;
                }
                else
                {
                    NvBool batFull = NV_FALSE;    
                    if (!Tps6586xBatteryChargerCBCBattFul(hDevice, &batFull))
                        return NV_FALSE;
                    if (batFull == NV_FALSE)
                        status = NVODM_BATTERY_STATUS_CHARGING;
                }
            }
            
            // Get VBatSense
            if (!Tps6586xAdcVBatSenseRead(hDevice, &VBatSense))
                return NV_FALSE;

            if (VBatSense > NVODM_BATTERY_HIGH_VOLTAGE_MV)  // maybe modify these parameters
                status |= NVODM_BATTERY_STATUS_HIGH;
            else if ((VBatSense < NVODM_BATTERY_LOW_VOLTAGE_MV)&& 
                (VBatSense > NVODM_BATTERY_CRITICAL_VOLTAGE_MV))
                status |= NVODM_BATTERY_STATUS_LOW;
            else if (VBatSense <= NVODM_BATTERY_CRITICAL_VOLTAGE_MV)
                status |= NVODM_BATTERY_STATUS_CRITICAL;
                
        }
        else
        {
            /* Battery is actually not present */
            status = NVODM_BATTERY_STATUS_NO_BATTERY;
        }

        *pStatus = status;
    }    
    else 
    {
        *pStatus = NVODM_BATTERY_STATUS_UNKNOWN;
    }

    return NV_TRUE;
}

NvBool
Tps6586xGetBatteryData(
    NvOdmPmuDeviceHandle hDevice, 
    NvOdmPmuBatteryInstance batteryInst,
    NvOdmPmuBatteryData *pData)
{
    NvOdmPmuBatteryData batteryData;
    
    batteryData.batteryLifePercent     = NVODM_BATTERY_DATA_UNKNOWN;
    batteryData.batteryLifeTime        = NVODM_BATTERY_DATA_UNKNOWN;
    batteryData.batteryVoltage         = NVODM_BATTERY_DATA_UNKNOWN;
    batteryData.batteryCurrent         = NVODM_BATTERY_DATA_UNKNOWN;
    batteryData.batteryAverageCurrent  = NVODM_BATTERY_DATA_UNKNOWN;
    batteryData.batteryAverageInterval = NVODM_BATTERY_DATA_UNKNOWN;
    batteryData.batteryMahConsumed     = NVODM_BATTERY_DATA_UNKNOWN;
    batteryData.batteryTemperature     = NVODM_BATTERY_DATA_UNKNOWN;
    batteryData.batteryVoltage         = NVODM_BATTERY_DATA_UNKNOWN;

    NV_ASSERT(hDevice);
    NV_ASSERT(pData);
    NV_ASSERT(batteryInst <= NvOdmPmuBatteryInst_Num);

    if (batteryInst == NvOdmPmuBatteryInst_Main)
    {
        NvU32 VBatSense = 0;
        NvU32 VBatTemp  = 0;

        if (battPresence == NV_TRUE)
        {
            /* retrieve Battery voltage and temperature */
            
            // Get VBatSense
            if (!Tps6586xAdcVBatSenseRead(hDevice, &VBatSense))
            {
                NVODMPMU_PRINTF(("Error reading VBATSense. \n"));
                return NV_FALSE;
            }

            // Get VBatTemp
            if (!Tps6586xAdcVBatTempRead(hDevice, &VBatTemp))
            {
                NVODMPMU_PRINTF(("Error reading VBATSense. \n"));
                return NV_FALSE;
            }

            batteryData.batteryVoltage     = VBatSense;
            batteryData.batteryTemperature = Tps6586xBatteryTemperature(VBatSense, VBatTemp);
        }
        
        *pData = batteryData;
    }
    else
    {
        *pData = batteryData;
    }

    return NV_TRUE;
}

void
Tps6586xGetBatteryFullLifeTime(
    NvOdmPmuDeviceHandle hDevice, 
    NvOdmPmuBatteryInstance batteryInst,
    NvU32 *pLifeTime)
{
    *pLifeTime = NVODM_BATTERY_DATA_UNKNOWN;
}

void
Tps6586xGetBatteryChemistry(
    NvOdmPmuDeviceHandle hDevice, 
    NvOdmPmuBatteryInstance batteryInst,
    NvOdmPmuBatteryChemistry *pChemistry)
{
    *pChemistry = NvOdmPmuBatteryChemistry_LION;
}


NvBool 
Tps6586xSetChargingCurrent(
    NvOdmPmuDeviceHandle hDevice,
    NvOdmPmuChargingPath chargingPath,
    NvU32 chargingCurrentLimitMa,
    NvOdmUsbChargerType ChargerType)
{
    NvU32 data = 0;
    NV_ASSERT(hDevice);

    // if no battery, then do nothing
    if (battPresence == NV_FALSE)
        return NV_TRUE;

    if (chargingCurrentLimitMa > DEDICATED_CHARGER_LIMIT_MA)
        chargingCurrentLimitMa = DEDICATED_CHARGER_LIMIT_MA;

    if (!Tps6586xI2cRead8(hDevice, TPS6586x_R49_CHG1, &data))
        return NV_FALSE;

    if (chargingPath == NvOdmPmuChargingPath_UsbBus)
    {
        switch (ChargerType)
        {
            case NvOdmUsbChargerType_SJ:
                chargingCurrentLimitMa = DEDICATED_CHARGER_LIMIT_MA;
                break;
            case NvOdmUsbChargerType_SK:
                chargingCurrentLimitMa = DEDICATED_CHARGER_LIMIT_MA;
                break;
            case NvOdmUsbChargerType_SE1:
                chargingCurrentLimitMa = DEDICATED_CHARGER_LIMIT_MA;
                break;
            case NvOdmUsbChargerType_SE0:
                chargingCurrentLimitMa = DEDICATED_CHARGER_LIMIT_MA;
                break;
            case NvOdmUsbChargerType_Dummy:
                chargingCurrentLimitMa = DUMB_CHARGER_LIMIT_MA;
                break;
            case NvOdmUsbChargerType_UsbHost:
                default:
                // USB Host based charging, nothing to do. Just pass current limit to PMU.
                break;
        }
    }
    
    if (chargingCurrentLimitMa >= MAX_CHARGING_CURRENT)
    {
        data = data & 0xf3;
        data = data | 0xC;
    }
    else if (chargingCurrentLimitMa >= L2_CHARGING_CURRENT)
    {
        data = data & 0xf3;
        data = data | 0x4;
    }
    else if (chargingCurrentLimitMa >= L3_CHARGING_CURRENT)
    {
        data = data & 0xf3;
        data = data | 0x0;
    }
    // 0 mA
    else
    {
        chargingCurrentLimitMa = 0;
    }

    //data = (NvU8)((( chargingCurrentLimitMa << 8 ) - chargingCurrentLimitMa ) 
    //            / CHARGER_CONSTANT_CURRENT_SET_MA );

    if (!Tps6586xI2cWrite8(hDevice, TPS6586x_R49_CHG1, data))
        return NV_FALSE;
     
    if (!Tps6586xI2cRead8(hDevice, TPS6586x_R4A_CHG2, &data))
        return NV_FALSE;
        
    if (chargingCurrentLimitMa == 0)
    {
        data = data & 0xfd; // Disable charging!
    }
    else
    {
        data = data | 0x02; // Enable Charging!
    }
    if (!Tps6586xI2cWrite8(hDevice, TPS6586x_R4A_CHG2, data))
        return NV_FALSE;

    return NV_TRUE;
}

void Tps6586xInterruptHandler( NvOdmPmuDeviceHandle  hDevice)
{
    //TPS6586xHandle tps6586x = hDevice->priv;
    //tps6586x->pmuStatus.batFull = NV_FALSE;
    
    // If the interrupt handle is called, the interrupt is supported.
    pmuInterruptSupported = NV_TRUE;
    
    Tps6586xInterruptHandler_int(hDevice, &pmuStatus);
}

NvBool Tps6586xReadRtc( NvOdmPmuDeviceHandle  hDevice, NvU32 *Count)
{
    *Count = 0;
    return (Tps6586xRtcCountRead(hDevice, Count));
}

NvBool Tps6586xWriteRtc( NvOdmPmuDeviceHandle  hDevice, NvU32 Count)
{
    return (Tps6586xRtcCountWrite(hDevice, Count));
}

NvBool Tps6586xIsRtcInitialized( NvOdmPmuDeviceHandle  hDevice)
{
    return ((Tps6586xRtcWasStartUpFromNoPower(hDevice))? NV_FALSE: NV_TRUE);
}
