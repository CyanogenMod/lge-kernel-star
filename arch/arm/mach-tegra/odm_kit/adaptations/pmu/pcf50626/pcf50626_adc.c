/*
 * Copyright (c) 2009 NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include "pcf50626_adc.h"
#include "pcf50626_i2c.h"
#include "pcf50626_reg.h"
//#include "pcf50626_supply_info.h"

#define ADC_CONVERSION_DELAY_USEC      70
#define ADC_CONVERSION_TIMEOUT_USEC    500
#define ADC_CONVERSION_VOLTAGE_RANGE   2000
#define ADC_CONVERSION_DIVIDOR         3
#define ADC_CONVERSION_PRECISION       10
#define ADC_CONVERSION_SUB_OFFSET      2250


static NvBool 
Pcf50626AdcIn1Read(NvOdmPmuDeviceHandle hDevice, NvU32 *volt);

static NvBool 
Pcf50626AdcIn2Read(NvOdmPmuDeviceHandle hDevice, NvU32 *volt);


/* read voltage from VBATSENSE */
NvBool 
Pcf50626AdcVBatSenseRead(NvOdmPmuDeviceHandle hDevice, NvU32 *volt)
{
    return Pcf50626AdcIn1Read(hDevice, volt);
}

static NvBool 
Pcf50626AdcIn1Read(NvOdmPmuDeviceHandle hDevice, NvU32 *volt)
{
    
    NvU32 timeout = 0;
    NvU8  dataS1  = 0;
    NvU8  dataS3  = 0;

    NV_ASSERT(hDevice);
    NV_ASSERT(volt);

    // Turn off GPIO7
    if(! Pcf50626I2cWrite8(hDevice, PCF50626_GPIO7C1_ADDR, 0x0))
        return NV_FALSE;


    //ADCC3 - Division sel    
    if(! Pcf50626I2cWrite8(hDevice, PCF50626_ADCC3_ADDR, PCF50626_ADCC3_RESET))
        return NV_FALSE;


    //ADCC1 -  Resolustion, Mux Sel, Avg sel
    if(! Pcf50626I2cWrite8(hDevice, PCF50626_ADCC1_ADDR, 0x0C))
        return NV_FALSE;

    // Start Converstion
    if(! Pcf50626I2cWrite8(hDevice, PCF50626_ADCC1_ADDR, 0x0D))
        return NV_FALSE;          

    // Wait for conversion
    NvOdmOsWaitUS(ADC_CONVERSION_DELAY_USEC);

    // make sure the conversion is completed, or timeout.
    while (timeout < ADC_CONVERSION_TIMEOUT_USEC)
    {
        if(! Pcf50626I2cRead8(hDevice, PCF50626_ADCS3_ADDR, &dataS3))
            return NV_FALSE;

        if (dataS3 & 0x80)
            break;

        NvOdmOsWaitUS(ADC_CONVERSION_DELAY_USEC);        
        timeout += ADC_CONVERSION_DELAY_USEC;
    }

    if (timeout >= ADC_CONVERSION_TIMEOUT_USEC)
    {
        NVODMPMU_PRINTF(("ADC conversion timeout.\n"));
        return NV_FALSE;
    }

    // read the conversion result
    if (!Pcf50626I2cRead8(hDevice, PCF50626_ADCS1_ADDR, &dataS1)) 
        return NV_FALSE;

    // Get result
    *volt = (((NvU32)((dataS1 << 2) | (dataS3 & 0x03))) * 
            ADC_CONVERSION_VOLTAGE_RANGE * ADC_CONVERSION_DIVIDOR) 
            >> ADC_CONVERSION_PRECISION; 

    return NV_TRUE;
}



/* read bat temperature voltage from ADC2 */
NvBool 
Pcf50626AdcVBatTempRead(NvOdmPmuDeviceHandle hDevice, NvU32 *volt)
{
    return Pcf50626AdcIn2Read(hDevice, volt);
}

static NvBool 
Pcf50626AdcIn2Read(NvOdmPmuDeviceHandle hDevice, NvU32 *volt)
{
    NvU32 timeout = 0;
    NvU8  dataS1  = 0;
    NvU8  dataS3  = 0;

    NV_ASSERT(hDevice);
    NV_ASSERT(volt);

    // Turn off GPIO7
    if(! Pcf50626I2cWrite8(hDevice, PCF50626_GPIO7C1_ADDR, 0x0))
        return NV_FALSE;


    //ADCC3 - Division sel    
    if(! Pcf50626I2cWrite8(hDevice, PCF50626_ADCC3_ADDR, PCF50626_ADCC3_RESET))
        return NV_FALSE;


    //ADCC1 -  Resolustion, Mux Sel, Avg sel
    if(! Pcf50626I2cWrite8(hDevice, PCF50626_ADCC1_ADDR, 0x2C))
        return NV_FALSE;

    // Start Converstion
    if(! Pcf50626I2cWrite8(hDevice, PCF50626_ADCC1_ADDR, 0x2D))
        return NV_FALSE;          

    // Wait for conversion
    NvOdmOsWaitUS(ADC_CONVERSION_DELAY_USEC);

    // make sure the conversion is completed, or timeout.
    while (timeout < ADC_CONVERSION_TIMEOUT_USEC)
    {
        if(! Pcf50626I2cRead8(hDevice, PCF50626_ADCS3_ADDR, &dataS3))
            return NV_FALSE;

        if (dataS3 & 0x80)
            break;

        NvOdmOsWaitUS(ADC_CONVERSION_DELAY_USEC);        
        timeout += ADC_CONVERSION_DELAY_USEC;
    }

    if (timeout >= ADC_CONVERSION_TIMEOUT_USEC)
    {
        NVODMPMU_PRINTF(("ADC conversion timeout.\n"));
        return NV_FALSE;
    }

    // read the conversion result
    if (!Pcf50626I2cRead8(hDevice, PCF50626_ADCS1_ADDR, &dataS1)) 
        return NV_FALSE;

    // Get result
    *volt = (((NvU32)((dataS1 << 2) | (dataS3 & 0x03))) * 
            ADC_CONVERSION_VOLTAGE_RANGE * ADC_CONVERSION_DIVIDOR) 
            >> ADC_CONVERSION_PRECISION; 

    return NV_TRUE;
}




