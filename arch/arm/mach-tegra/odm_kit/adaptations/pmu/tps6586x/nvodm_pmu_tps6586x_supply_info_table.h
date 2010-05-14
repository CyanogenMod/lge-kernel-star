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

#ifndef TPS6586x_SUPPLY_INFO_TABLE_H_
#define TPS6586x_SUPPLY_INFO_TABLE_H_

#if defined(__cplusplus)
extern "C"
{
#endif

#define PMUGUID NV_ODM_GUID('t','p','s','6','5','8','6','x')

#if defined(CONFIG_TEGRA_ODM_HARMONY)

/// The total number of external supplies (which use both AP and PMU GPIOs)
#define TPS6586x_EXTERNAL_SUPPLY_NUM \
    (NvU32)(TPS6586xPmuSupply_Num - Ext_TPS62290PmuSupply_BUCK)

/// Macro for converting a vddRail to AP GPIO pin index.
#define NVODM_EXT_AP_GPIO_RAIL(x) ((x) - Ext_TPS2051BPmuSupply_VDDIO_VID)

/// The total number of external supplies which use AP GPIO pins for enable
#define TPS6586x_EXTERNAL_SUPPLY_AP_GPIO_NUM \
    (NvU32)NVODM_EXT_AP_GPIO_RAIL(TPS6586xPmuSupply_Num)

#else

/* FIXME: modify this table according to your schematics */
#define V_CORE      TPS6586xPmuSupply_DCD0      
#define V_1V8       TPS6586xPmuSupply_DCD1      
#define LCD_2V8     TPS6586xPmuSupply_LDO0      
#define V_1V2       TPS6586xPmuSupply_LDO1      
#define V_RTC       TPS6586xPmuSupply_LDO2      
#define V_CAM_1V8   TPS6586xPmuSupply_LDO3      
#define V_CODEC_1V8 TPS6586xPmuSupply_LDO4      
#define V_CAM_2V8   TPS6586xPmuSupply_LDO5      
#define V_3V3       TPS6586xPmuSupply_LDO6      
#define V_SDIO      TPS6586xPmuSupply_LDO7      
#define V_2V8       TPS6586xPmuSupply_LDO8      
#define V_2V5       TPS6586xPmuSupply_LDO9      
#define V_25V       TPS6586xPmuSupply_WHITE_LED
#define V_CHARGE    TPS6586xPmuSupply_DCD2
#define V_MODEM     V_1V8   /* Alias for V_1V8 */
#define V_GND       TPS6586xPmuSupply_Invalid
#define V_INVALID   TPS6586xPmuSupply_Invalid
#define VRAILCOUNT  TPS6586xPmuSupply_Num
#endif

typedef enum
{
    TPS6586xPmuSupply_Invalid = 0x0,

    //DCD0
    TPS6586xPmuSupply_DCD0,

    //DCD1
    TPS6586xPmuSupply_DCD1,

    //DCD2
    TPS6586xPmuSupply_DCD2,


    //LDO0
    TPS6586xPmuSupply_LDO0,

    //LDO1
    TPS6586xPmuSupply_LDO1,

    //LDO2
    TPS6586xPmuSupply_LDO2,

    //LDO3
    TPS6586xPmuSupply_LDO3,
    
    //LDO4
    TPS6586xPmuSupply_LDO4,
     
    //LDO5
    TPS6586xPmuSupply_LDO5,

    //LDO6
    TPS6586xPmuSupply_LDO6,

    //LDO7
    TPS6586xPmuSupply_LDO7,

    //LDO8
    TPS6586xPmuSupply_LDO8,

    //LDO9
    TPS6586xPmuSupply_LDO9,

    //RTC_OUT
    TPS6586xPmuSupply_RTC_OUT,

    //RED1
    TPS6586xPmuSupply_RED1,

    //GREEN1
    TPS6586xPmuSupply_GREEN1,

    //BLUE1
    TPS6586xPmuSupply_BLUE1,

    //RED2
    TPS6586xPmuSupply_RED2,

    //GREEN2
    TPS6586xPmuSupply_GREEN2,

    //BLUE2
    TPS6586xPmuSupply_BLUE2,

    //LED_PWM
    TPS6586xPmuSupply_LED_PWM,

    //PWM
    TPS6586xPmuSupply_PWM,

    //White LED(SW3)
    TPS6586xPmuSupply_WHITE_LED,
#if defined(CONFIG_TEGRA_ODM_HARMONY)
    //SOC
    TPS6586xPmuSupply_SoC,

    /*--- Secondary/External PMU Rails ---*/

    // PMU GPIO-3: VDD_1V05
    Ext_TPS62290PmuSupply_BUCK,

    // PMU GPIO-2: VDD_1V2
    Ext_TPS72012PmuSupply_LDO,

    // PMU GPIO-1: VDD_1V5
    Ext_TPS74201PmuSupply_LDO,

    // AP GPIO(T,2): VDDIO_HDMI, VDDIO_VGA (5V @ 500ma)
    Ext_TPS2051BPmuSupply_VDDIO_VID,

    // AP GPIO(T,3): VDDIO_SD
    Ext_SWITCHPmuSupply_VDDIO_SD,

    // AP GPIO(I,6): VDDIO_SDMMC
    Ext_SWITCHPmuSupply_VDDIO_SDMMC,

    // AP GPIO(W,0): VDD_BL
    // FIXME: This is already supplied by nvodm_query_gpio in the display GPIO settings.
    Ext_SWITCHPmuSupply_VDD_BL,

    // AP GPIO(C,6): VDD_PNL
    // FIXME: This is already supplied by nvodm_query_gpio in the display GPIO settings.
    Ext_SWITCHPmuSupply_VDD_PNL,
#endif

    TPS6586xPmuSupply_Num,
    TPS6586xPmuSupply_Force32 = 0x7FFFFFFF
} TPS6586xPmuSupply;

#if defined(__cplusplus)
}
#endif

#endif /* TPS6586x_SUPPLY_INFO_TABLE_H_ */
