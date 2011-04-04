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

#include "max8907.h"
#include "max8907_adc.h"
#include "max8907_i2c.h"
#include "max8907_reg.h"

//20100428, jh.ahn@lge.com, This define for Debug Message function [START]
#include <linux/kernel.h>
#include <mach/lprintk.h>

//#define LG_DEBUG_PMU
#undef LG_DEBUG_PMU  // Define for Debug Serial

#ifdef LG_DEBUG_PMU
#define LDPA(fmt, arg...) lprintk(D_BATT, "%s : " fmt "\n", __func__, ## arg)
#else
#define LDPA(fmt, arg...) do {} while (0)
#endif // LG_DEBUG_PMU
//20100428, jh.ahn@lge.com, This define for Debug Message function [END]

//20100622, jh.ahn@lge.com, setup ADC function in PMU for VCHG, VBBATT, VMBATT, THM [START]
NvBool
Max8907AdcSetup(NvOdmPmuDeviceHandle hDevice)
{
    NvU8 data;

    NV_ASSERT(hDevice);

    // set resolution to 12bit : default all 12bit 0x00
    data = 0;

    if (!Max8907AdcI2cWrite8(hDevice, MAX8907_ADC_RES_CNFG1, data))
        return NV_FALSE;

    // set Average conversion : default single, set avg for VCHG, VBBATT, VMBATT, THM
    data = 0;
    data |= (AVG_8AVG_MASK << AVG_VCHG_EN_SHIFT)
            | (AVG_8AVG_MASK << AVG_VBBATT_EN_SHIFT)
            | (AVG_8AVG_MASK << AVG_VMBATT_EN_SHIFT)
            | (AVG_8AVG_MASK << AVG_THM_EN_SHIFT);

    if (!Max8907AdcI2cWrite8(hDevice, MAX8907_ADC_AVG_CNFG1, data))
        return NV_FALSE;

    // set Acuisition time for 128us : default all 128us
    data = 0x55; // set reserved bit default value
    data |= (T_128_PERIOD_MASK << T_ACQ_AUX1_SHIFT)
            | (T_128_PERIOD_MASK << T_ACQ_AUX2_SHIFT)
            | (T_128_PERIOD_MASK << T_ACQ_VCHG_SHIFT)
            | (T_128_PERIOD_MASK << T_ACQ_VBBATT_SHIFT);

    if (!Max8907AdcI2cWrite8(hDevice, MAX8907_ADC_ACQ_CNFG1, data))
        return NV_FALSE;

    data = 0xAA; // set reserved bit default value
    data |= (T_128_PERIOD_MASK << T_ACQ_VMBATT_SHIFT)
            | (T_128_PERIOD_MASK << T_ACQ_ISNS_SHIFT)
            | (T_128_PERIOD_MASK << T_ACQ_THM_SHIFT)
            | (T_128_PERIOD_MASK << T_ACQ_TDIE_SHIFT);

    if (!Max8907AdcI2cWrite8(hDevice, MAX8907_ADC_ACQ_CNFG2, data))
        return NV_FALSE;

    // Enable ADC Scheduler, period 10s : default en / 10s
    data = 0;
    data |= (AUTOSCH_ENABLE_MASK << AUTOSCH_EN_SHIFT)
            |(AUTOSCH_T_10S_MASK << AUTOSCH_T_SHIFT);

    if (!Max8907AdcI2cWrite8(hDevice, MAX8907_ADC_SCHED, data))
        return NV_FALSE;

    // Enable int_ref_en bit in RESET_CNFG Reg
    if (!Max8907I2cRead8(hDevice, MAX8907_RESET_CNFG, &data))
        return NV_FALSE;

    data |= (MAX8907_RESET_CNFG_INT_REF_EN_MASK <<
                 MAX8907_RESET_CNFG_INT_REF_EN_SHIFT);

    if (!Max8907I2cWrite8(hDevice, MAX8907_RESET_CNFG, data))
        return NV_FALSE;

    LDPA("[jh.ahn] ADC_Settinge_Done!");
    return NV_TRUE;
}
//20100622, jh.ahn@lge.com, setup ADC function in PMU for VCHG, VBBATT, VMBATT, THM [END]

#if defined(CONFIG_MACH_STAR) //jongik2.kim 20100803 HOOK_DETECTION
NvU32 
Max8907AdcHookAdcRead(
    NvOdmPmuDeviceHandle hDevice)
{
    NvU8 data, msb_data, lsb_data;
    NvU32 readdata = 0;

    // Enable int_ref_en bit in RESET_CNFG Reg
    if (!Max8907I2cRead8(hDevice, MAX8907_RESET_CNFG, &data))
        return 0;

    data |= (MAX8907_RESET_CNFG_INT_REF_EN_MASK <<
                 MAX8907_RESET_CNFG_INT_REF_EN_SHIFT);

    if (!Max8907I2cWrite8(hDevice, MAX8907_RESET_CNFG, data))
        return 0;

    // Enable Internal voltage reference
    if (!Max8907AdcI2cWrite8(hDevice, MAX8907_TSC_CNFG1, 0x12))
        return 0;

    // Send command to powerup and the ADC perform a conversion
    if (!Max8907AdcI2cWrite8(hDevice, CONV_REG_AUX2_ON, 0x00))
        return 0;

    // Get result
    if (!Max8907AdcI2cRead8(hDevice, MAX8907_AUX2_MSB, &msb_data))
        return 0;

    if (!Max8907AdcI2cRead8(hDevice, MAX8907_AUX2_LSB, &lsb_data))
        return 0;

    readdata = (msb_data << 4) | (lsb_data >> 4) ;
	LDPA("IGGIKIM :: Max8907AdcHookAdcRead %d\n",readdata);
    
    return readdata;
}
#endif

NvBool 
Max8907AdcVBatSenseRead(
    NvOdmPmuDeviceHandle hDevice, 
    NvU32 *volt)
{
//20100622, jh.ahn@lge.com, Write the description here in detail [START]
    NvU8 data, msb_data, lsb_data;
    NvU32 batvdata;

    NV_ASSERT(hDevice);
    NV_ASSERT(volt);

    // Enable int_ref_en bit in RESET_CNFG Reg
    if (!Max8907I2cRead8(hDevice, MAX8907_RESET_CNFG, &data))
        return NV_FALSE;

    data |= (MAX8907_RESET_CNFG_INT_REF_EN_MASK <<
                 MAX8907_RESET_CNFG_INT_REF_EN_SHIFT);

    if (!Max8907I2cWrite8(hDevice, MAX8907_RESET_CNFG, data))
        return NV_FALSE;

    // Enable Internal voltage reference
    if (!Max8907AdcI2cWrite8(hDevice, MAX8907_TSC_CNFG1, 0x12))
        return NV_FALSE;

    // Send command to powerup and the ADC perform a conversion
    if (!Max8907AdcI2cWrite8(hDevice, CONV_REG_VMBATT_ON, 0x00))
        return NV_FALSE;

    // Get result
    if (!Max8907AdcI2cRead8(hDevice, MAX8907_VMBATT_MSB, &msb_data))
        return NV_FALSE;

    LDPA("[jh.ahn] VMBATT [VMBATT_MSB] = %x\n", msb_data);
    if (!Max8907AdcI2cRead8(hDevice, MAX8907_VMBATT_LSB, &lsb_data))
        return NV_FALSE;
    LDPA("[jh.ahn] VMBATT [VMBATT_LSB,] = %x\n", lsb_data);

    batvdata = (msb_data << 4) | (lsb_data >> 4) ;
    *volt = batvdata*2; //This conversion is for 12bit ADC result : 8.192*CODE/2^N = 8.192*CODE/4096 = CODE*2 [mV]
    LDPA("[jh.ahn] VMBATT[mV] = %d\n", *volt);
//20100622, jh.ahn@lge.com, Write the description here in detail [END]

    return NV_TRUE;
}

NvBool 
Max8907AdcVBatTempRead(
    NvOdmPmuDeviceHandle hDevice, 
    NvU32 *btemp)
{
//20100622, jh.ahn@lge.com, Write the description here in detail [START]
    NvU8 data, msb_data, lsb_data;
    NvU32 btempdata;

    NV_ASSERT(hDevice);
    NV_ASSERT(btemp);

    // Enable int_ref_en bit in RESET_CNFG Reg
    if (!Max8907I2cRead8(hDevice, MAX8907_RESET_CNFG, &data))
        return NV_FALSE;

    data |= (MAX8907_RESET_CNFG_INT_REF_EN_MASK <<
                 MAX8907_RESET_CNFG_INT_REF_EN_SHIFT);

    if (!Max8907I2cWrite8(hDevice, MAX8907_RESET_CNFG, data))
        return NV_FALSE;

    // Enable Internal voltage reference
    if (!Max8907AdcI2cWrite8(hDevice, MAX8907_TSC_CNFG1, 0x12))
        return NV_FALSE;

    // Send command to powerup and the ADC perform a conversion
    if (!Max8907AdcI2cWrite8(hDevice, CONV_REG_THM_ON, 0x00))
        return NV_FALSE;

    // Get result
    if (!Max8907AdcI2cRead8(hDevice, MAX8907_THM_MSB, &msb_data))
        return NV_FALSE;

    LDPA("[jh.ahn] bTHM [THM_MSB] = %x\n", msb_data);
    if (!Max8907AdcI2cRead8(hDevice, MAX8907_THM_LSB, &lsb_data))
        return NV_FALSE;
    LDPA("[jh.ahn] bTHM [THM_LSB] = %x\n", lsb_data);

    btempdata = (msb_data << 4) | (lsb_data >> 4) ;
    *btemp = (NvU32)(68000*btempdata/(4096-btempdata)); //This conversion is for 12bit ADC result : R = (CODE/2^12*Rpu)/(1-CODE/2^12)
    LDPA("[jh.ahn] bTHM [Rthm] = %d\n", *btemp);
    *btemp = btempdata;
//20100622, jh.ahn@lge.com, Write the description here in detail [END]

    return NV_TRUE;
}

NvU32
Max8907BatteryTemperature(
    NvU32 VBatSense,
    NvU32 VBatTemp)
{
//20100624, jh.ahn@lge.com, Write the description here in detail [START]
    static NvU16 BAT_TEMP_TABLE[] = BAT_T_TABLE;

    LDPA("[jh.ahn] BatTemp[K] = %d(%d[C]) \n", BAT_TEMP_TABLE[VBatTemp], (BAT_TEMP_TABLE[VBatTemp]-2730));
    return BAT_TEMP_TABLE[VBatTemp];
//20100624, jh.ahn@lge.com, Write the description here in detail [END]
}

