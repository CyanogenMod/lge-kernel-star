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

#include "nvodm_pmu_tps6586x_adc.h"
#include "nvodm_pmu_tps6586x_i2c.h"
#include "tps6586x_reg.h"
#include "nvodm_pmu_tps6586x_supply_info_table.h"

#define ADC_CONVERSION_DELAY_USEC      70
#define ADC_CONVERSION_TIMEOUT_USEC    500
#define ADC_CONVERSION_VOLTAGE_RANGE   2000
#define ADC_CONVERSION_DIVIDOR         3
#define ADC_CONVERSION_PRECISION       10
#define ADC_CONVERSION_SUB_OFFSET      2250
#define ADC_FULL_SCALE_READING_MV_BAT  4622
#define ADC_FULL_SCALE_READING_MV_TS   2600
#define ADC_CONVERSION_PREWAIT_MS      26

/* read voltage from ADC CH10(battery) */
NvBool 
Tps6586xAdcVBatSenseRead(NvOdmPmuDeviceHandle hDevice, NvU32 *volt)
{
    NvU32 timeout  = 0;
    NvU32  dataS1  = 0;
    NvU32  dataH   = 0;
    NvU32  dataL   = 0;

    NV_ASSERT(hDevice);
    NV_ASSERT(volt);
    
    *volt = 0;    // Default is 0mV. 
    // Configuring the adc conversion cycle
    // ADC0_WAIT register(0x62)
    // Reset all ADC engines and return them to the idle state; ADC0_RESET: 1
    if(! Tps6586xI2cWrite8(hDevice, TPS6586x_R62_ADC0_WAIT, 0x80))
        return NV_FALSE;
        
    // ADC0_SET register(0x61)
    // ADC0_EN: 0(Don't start conversion); Number of Readings: 16; CHANNEL: CH10(battery)
    if(! Tps6586xI2cWrite8(hDevice, TPS6586x_R61_ADC0_SET, 0x19))
        return NV_FALSE;
        
    // ADC0_WAIT register(0x62)
    // REF_EN: 0; AUTO_REF: 1; Wait time: 0.062ms
    if(! Tps6586xI2cWrite8(hDevice, TPS6586x_R62_ADC0_WAIT, 0x21))
        return NV_FALSE;

    // Start conversion!!
    if(! Tps6586xI2cWrite8(hDevice, TPS6586x_R61_ADC0_SET, 0x99))
        return NV_FALSE;

    // Wait for conversion
    //NvOdmOsWaitUS(ADC_CONVERSION_DELAY_USEC);
    NvOdmOsSleepMS(ADC_CONVERSION_PREWAIT_MS);

    // Make sure the conversion is completed - check for ADC error.
    while (1)
    {
        // Read ADC status register
        if(! Tps6586xI2cRead8(hDevice, TPS6586x_R9A_ADC0_INT, &dataS1))
            return NV_FALSE;

        // Conversion is done!
        if (dataS1 & 0x80)
            break;
        
        // ADC error!
        if (dataS1 & 0x40)
        {
            NVODMPMU_PRINTF(("ADC conversion error.\n"));
            return NV_FALSE;
        }

        NvOdmOsWaitUS(ADC_CONVERSION_DELAY_USEC);
        timeout += ADC_CONVERSION_DELAY_USEC;
        if (timeout >= ADC_CONVERSION_TIMEOUT_USEC)
            return NV_FALSE;
    }

    // Read the ADC conversion Average (SUM).
    if (!Tps6586xI2cRead8(hDevice, TPS6586x_R94_ADC0_SUM2, &dataH))
        return NV_FALSE;
        
    if (!Tps6586xI2cRead8(hDevice, TPS6586x_R95_ADC0_SUM1, &dataL))
        return NV_FALSE;

    // Get a result value with mV.
    *volt = (((dataH << 8) | dataL) * ADC_FULL_SCALE_READING_MV_BAT) / 1023 / 16;

    return NV_TRUE;
}

/* read voltage from ADC CH5(temperature) */
NvBool
Tps6586xAdcVBatTempRead(NvOdmPmuDeviceHandle hDevice, NvU32 *volt)
{
    NvU32 timeout  = 0;
    NvU32  dataS1  = 0;
    NvU32  dataH   = 0;
    NvU32  dataL   = 0;

    NV_ASSERT(hDevice);
    NV_ASSERT(volt);
    
    *volt = 0;    // Default is 0'C. 
    
    // Configuring the adc conversion cycle    
    // ADC0_WAIT register(0x62)
    // Reset all ADC engines and return them to the idle state; ADC0_RESET: 1
    if(! Tps6586xI2cWrite8(hDevice, TPS6586x_R62_ADC0_WAIT, 0x80))
        return NV_FALSE; 
    // ADC0_SET register(0x61)
    // ADC0_EN: 0(Don't start conversion); Number of Readings: 16; CHANNEL: CH5(temperature)
    if(! Tps6586xI2cWrite8(hDevice, TPS6586x_R61_ADC0_SET, 0x14))
        return NV_FALSE;
        
    // ADC0_WAIT register(0x62)
    // REF_EN: 0; AUTO_REF: 1; Wait time: 0.062ms
    if(! Tps6586xI2cWrite8(hDevice, TPS6586x_R62_ADC0_WAIT, 0x21))
        return NV_FALSE;

    // Start conversion!!
    if(! Tps6586xI2cWrite8(hDevice, TPS6586x_R61_ADC0_SET, 0x94))
        return NV_FALSE;

    // Wait for conversion
    // NvOdmOsWaitUS(ADC_CONVERSION_DELAY_USEC);
    NvOdmOsSleepMS(ADC_CONVERSION_PREWAIT_MS);

    // make sure the conversion is completed, or adc error.
    while (1)
    {
        // Read ADC status register
        if(! Tps6586xI2cRead8(hDevice, TPS6586x_R9A_ADC0_INT, &dataS1))
            return NV_FALSE;

        // Conversion is done!
        if (dataS1 & 0x80)
            break;
        
        // ADC error!
        if (dataS1 & 0x40)
        {
            NVODMPMU_PRINTF(("ADC conversion error.\n"));
            return NV_FALSE;
        }

        NvOdmOsWaitUS(ADC_CONVERSION_DELAY_USEC);
        timeout += ADC_CONVERSION_DELAY_USEC;
        if (timeout >= ADC_CONVERSION_TIMEOUT_USEC)
            return NV_FALSE;
    }

    // Read the ADC conversion Average (SUM).
    if (!Tps6586xI2cRead8(hDevice, TPS6586x_R94_ADC0_SUM2, &dataH))
        return NV_FALSE;
        
    if (!Tps6586xI2cRead8(hDevice, TPS6586x_R95_ADC0_SUM1, &dataL))
        return NV_FALSE;

    // Get a result value with mV.
    *volt = (((dataH << 8) | dataL) *  ADC_FULL_SCALE_READING_MV_TS) / 1023 / 16;

    return NV_TRUE;
}

/* Calculate the battery temperature */
NvU32 Tps6586xBatteryTemperature(NvU32 VBatSense, NvU32 VBatTemp)
{
    return 0;
}
