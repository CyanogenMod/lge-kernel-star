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
#include "nvodm_services.h"
#include "pcf50626_i2c.h"
#include "pcf50626.h"
#include "pcf50626_batterycharger.h"
#include "pcf50626_adc.h"
#include "pcf50626_interrupt.h"
#include "pcf50626_supply_info_table.h"

#ifndef PMU_MAX
#define PMU_MAX(a,b)        ((a)<(b)?(b):(a))
#endif

#define BATTEMP_CONTROL     (0)

// Board IDs
#define NVODM_PMU_BOARD_ID_E924   0x0918
#define NVODM_PMU_BOARD_ID_E934   0x0922

// SKUs
#define NVODM_PMU_BOARD_SAMSUNG_26_MHZ_OSC    0x0A00
#define NVODM_PMU_BOARD_HYNIX_12_MHZ_XTAL     0x0A01

// h/w configuration
#define CHARGER_CONSTANT_CURRENT_SET_MA  (NvU32)(125000/127)
#define MAX_CHARGER_LIMIT_MA 850

// This PMU does not have differnet charger programming 
// So setting all types of charger limit to the default charger limit
#define SE0_TYPE_CHARGER_LIMIT_MA MAX_CHARGER_LIMIT_MA
#define SE1_TYPE_CHARGER_LIMIT_MA MAX_CHARGER_LIMIT_MA
#define SJ_TYPE_CHARGER_LIMIT_MA MAX_CHARGER_LIMIT_MA
#define SK_TYPE_CHARGER_LIMIT_MA MAX_CHARGER_LIMIT_MA


// threshold for battery status. need to fine tune based on battery/system characterisation
#define NVODM_BATTERY_FULL_VOLTAGE_MV      4150
#define NVODM_BATTERY_HIGH_VOLTAGE_MV      3900
#define NVODM_BATTERY_LOW_VOLTAGE_MV       3300
#define NVODM_BATTERY_CRITICAL_VOLTAGE_MV  3100

#define NVODM_BATTERY_OVERHEAT_THRESHOLD   70


Pcf50626PrivData *pPrivData;
//Concorde WAR for the USB Host mode
NvBool UsbHostMode;

#define PMUGUID NV_ODM_GUID('p','c','f','_','p','m','u','0')


// Calulate the battery life percentage according to the battery voltage.
static NvU32 
Pcf50626CalulateBatteryLifePercent_int(NvU32 vBatSense);

#if BATTEMP_CONTROL
// switch off the chargeer if the battery temperature is too high
static NvBool 
Pcf50626BatteryTemperatureControl_int(
    NvOdmPmuDeviceHandle hDevice,
    NvU32 batTemp);
#endif

// Read the voltage setting from PCF50626 registers
static NvBool 
Pcf50626ReadVoltageReg(
    NvOdmPmuDeviceHandle hDevice, 
    NvU32 vddRail, 
    NvU32* pMilliVolts);

// Write the voltage setting from PCF50626 registers
static NvBool 
Pcf50626WriteVoltageReg(
    NvOdmPmuDeviceHandle hDevice, 
    NvU32 vddRail, 
    NvU32 MilliVolts, 
    NvU32* pSettleMicroSeconds);


void
Pcf50626GetCapabilities(
    NvU32 vddRail,
    NvOdmPmuVddRailCapabilities* pCapabilities)
{
    NvOdmBoardInfo BoardInfo;
    NvBool Status = NV_FALSE;

    NV_ASSERT(pCapabilities);
    NV_ASSERT(vddRail < PCF50626PmuSupply_Num);

    *pCapabilities = pcf50626SupplyInfoTable[vddRail].cap;

    if (vddRail == PCF50626PmuSupply_DCD2)
    {
        Status = NvOdmPeripheralGetBoardInfo(NVODM_PMU_BOARD_ID_E924, &BoardInfo);
        if (Status == NV_TRUE)
        {
            if ((BoardInfo.SKU == NVODM_PMU_BOARD_SAMSUNG_26_MHZ_OSC) ||
                (BoardInfo.SKU == NVODM_PMU_BOARD_HYNIX_12_MHZ_XTAL))
            {
                // Use 1.8v DDR (TO DO: Don't use a magic number here; define this.)
                pCapabilities->requestMilliVolts = 1800;
            }
        }
        else
        {
            Status = NvOdmPeripheralGetBoardInfo(NVODM_PMU_BOARD_ID_E934, &BoardInfo);
            if (Status == NV_TRUE)
            {
                if (BoardInfo.SKU == NVODM_PMU_BOARD_HYNIX_12_MHZ_XTAL)
                {
                    // Use 1.8v DDR (TO DO: Don't use a magic number here; define this.)
                    pCapabilities->requestMilliVolts = 1800;
                }
            }
            else
            {
                // Use default DDR voltage (1.925v)
                ;
            }
        }
    }
}


NvBool Pcf50626Setup(NvOdmPmuDeviceHandle hDevice)
{
    NvOdmIoModule I2cModule = NvOdmIoModule_I2c;
    NvU32  I2cInstance = 0;
    NvU32  I2cAddress  = 0;    
    NvU32  i           = 0;
    NvBool status      = NV_FALSE;
    
    const NvOdmPeripheralConnectivity *pConnectivity = 
                           NvOdmPeripheralGetGuid(PMUGUID);
    
    NV_ASSERT(hDevice);

    
    pPrivData = (Pcf50626PrivData*) NvOdmOsAlloc(sizeof(Pcf50626PrivData));
    if (pPrivData == NULL)
    {
        NVODMPMU_PRINTF(("Error Allocating Pcf50626PrivData. \n"));
        return NV_FALSE;
    }
    NvOdmOsMemset(pPrivData, 0, sizeof(Pcf50626PrivData));
    hDevice->pPrivate = pPrivData;

    ((Pcf50626PrivData*)hDevice->pPrivate)->supplyRefCntTable = NvOdmOsAlloc(sizeof(NvU32) * PCF50626PmuSupply_Num);
    if (((Pcf50626PrivData*)hDevice->pPrivate)->supplyRefCntTable == NULL)
    {     
        NVODMPMU_PRINTF(("Error Allocating RefCntTable. \n"));
        goto fail;
    }
        
    // memset
    for (i = 0; i < PCF50626PmuSupply_Num; i++)
    {
        ((Pcf50626PrivData*)hDevice->pPrivate)->supplyRefCntTable[i] = 0;
    }


    if (pConnectivity != NULL) // PMU is in database
    {        
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

        ((Pcf50626PrivData*)hDevice->pPrivate)->hOdmI2C = NvOdmI2cOpen(I2cModule, I2cInstance);
        if (!((Pcf50626PrivData*)hDevice->pPrivate)->hOdmI2C)
        {
            NVODMPMU_PRINTF(("[NVODM PMU]Pcf50626Setup: Error Open I2C device. \n"));     
            NVODMPMU_PRINTF(("[NVODM PMU]Please check PMU device I2C settings. \n"));  
            goto fail;        
        }
        
        ((Pcf50626PrivData*)hDevice->pPrivate)->DeviceAddr = I2cAddress;
        ((Pcf50626PrivData*)hDevice->pPrivate)->hOdmPmuSevice = NvOdmServicesPmuOpen();
        if (!((Pcf50626PrivData*)hDevice->pPrivate)->hOdmPmuSevice)
        {
            NVODMPMU_PRINTF(("[NVODM PMU]Pcf50626Setup: Error Open PMU Odm service. \n"));
            goto fail;        
        }
    }   
    else
    {
        // if PMU is not presented in the database, then the platform is PMU-less.
        NVODMPMU_PRINTF(("[NVODM PMU]Pcf50626Setup: The system did not doscover PMU fromthe data base. \n"));     
        NVODMPMU_PRINTF(("[NVODM PMU]Pcf50626Setup: If this is not intended, please check the peripheral database for PMU settings. \n"));     
        goto fail;
    }
    
    if (!Pcf50626BatteryChargerSetup(hDevice))
    {
        NVODMPMU_PRINTF(("[NVODM PMU]Pcf50626Setup: Pcf50626BatteryChargerSetup() failed. \n"));
        goto fail;
    }

    //Check battery presence
    if (!Pcf50626BatteryChargerCBCMainBatt(hDevice,&((Pcf50626PrivData*)hDevice->pPrivate)->battPresence))
    {
        NVODMPMU_PRINTF(("[NVODM PMU]Pcf50626Setup: Pcf50626BatteryChargerCBCMainBatt() failed. \n"));
        goto fail;
    }
    
    // The interrupt assumes not supported until pcf50626InterruptHandler() is called. 
    ((Pcf50626PrivData*)hDevice->pPrivate)->pmuInterruptSupported = NV_FALSE;

    // setup the interrupt any way.
    if (!Pcf50626SetupInterrupt(hDevice, &((Pcf50626PrivData*)hDevice->pPrivate)->pmuStatus))
    {
        NVODMPMU_PRINTF(("[NVODM PMU]Pcf50626Setup: Pcf50626SetupInterrupt() failed. \n"));
        goto fail;
    }

    // Check battery Fullness
    if (((Pcf50626PrivData*)hDevice->pPrivate)->battPresence == NV_TRUE)
    {   
        if (!Pcf50626BatteryChargerCBCBattFul(hDevice,&status))
        {
            NVODMPMU_PRINTF(("[NVODM PMU]Pcf50626Setup: Pcf50626BatteryChargerCBCBattFul() failed. \n"));
            goto fail;
        }

        ((Pcf50626PrivData*)hDevice->pPrivate)->pmuStatus.batFull = status;
    }
    else
    {
        ((Pcf50626PrivData*)hDevice->pPrivate)->pmuStatus.batFull = NV_FALSE;
    }

    return NV_TRUE;

fail:
    Pcf50626Release(hDevice);
    return NV_FALSE;
    
    
}

void Pcf50626Release(NvOdmPmuDeviceHandle hDevice)
{
    if (hDevice->pPrivate != NULL)
    {
        if (((Pcf50626PrivData*)hDevice->pPrivate)->hOdmPmuSevice != NULL)
        {
            NvOdmServicesPmuClose(((Pcf50626PrivData*)hDevice->pPrivate)->hOdmPmuSevice);
            ((Pcf50626PrivData*)hDevice->pPrivate)->hOdmPmuSevice = NULL;
        }
        
        if (((Pcf50626PrivData*)hDevice->pPrivate)->hOdmI2C != NULL)
        {
            NvOdmI2cClose(((Pcf50626PrivData*)hDevice->pPrivate)->hOdmI2C);
            ((Pcf50626PrivData*)hDevice->pPrivate)->hOdmI2C = NULL;
        }

        if (((Pcf50626PrivData*)hDevice->pPrivate)->supplyRefCntTable != NULL)
        {
            NvOdmOsFree(((Pcf50626PrivData*)hDevice->pPrivate)->supplyRefCntTable);
            ((Pcf50626PrivData*)hDevice->pPrivate)->supplyRefCntTable = NULL;
        }

        NvOdmOsFree(hDevice->pPrivate);
        hDevice->pPrivate = NULL;
    }
}


NvBool
Pcf50626GetVoltage(
    NvOdmPmuDeviceHandle hDevice,
    NvU32 vddRail,
    NvU32* pMilliVolts)
{   
    NV_ASSERT(hDevice);
    NV_ASSERT(pMilliVolts);
    NV_ASSERT(vddRail < PCF50626PmuSupply_Num);

    if(! Pcf50626ReadVoltageReg(hDevice, vddRail,pMilliVolts))
        return NV_FALSE;
    
    return NV_TRUE;
}


NvBool
Pcf50626SetVoltage(
    NvOdmPmuDeviceHandle hDevice,
    NvU32 vddRail,
    NvU32 MilliVolts,
    NvU32* pSettleMicroSeconds)
{
    NvU8 data = 0;

    NV_ASSERT(hDevice);
    NV_ASSERT(vddRail < PCF50626PmuSupply_Num);

    if (pcf50626SupplyInfoTable[vddRail].cap.OdmProtected == NV_TRUE)
    {
        NVODMPMU_PRINTF(("[NVODM PMU] Pcf50626SetVoltage Warning: The voltage is protected and can not be set: %d.\n", vddRail));
        return NV_TRUE;
    }

    if ((MilliVolts == ODM_VOLTAGE_OFF) ||
           ((MilliVolts <= pcf50626SupplyInfoTable[vddRail].cap.MaxMilliVolts)
           && (MilliVolts >= pcf50626SupplyInfoTable[vddRail].cap.MinMilliVolts)))
    {
        if (! Pcf50626WriteVoltageReg(hDevice, vddRail, MilliVolts, pSettleMicroSeconds))
            return NV_FALSE;
    }
    else
    {
        NVODMPMU_PRINTF(("[NVODM OPMU] Pcf50626SetVoltage Error: The required voltage is not supported..\n"));
        return NV_FALSE;
    }

    if (vddRail == PCF50626PmuSupply_DCUD)
    {
        // VBUs rail is enabled bydefault, so no need to enable  set voltage.
        // "Millivolts"  field is used as Enable or disable VBUS GPIO 
        if (MilliVolts == ODM_VOLTAGE_OFF) 
        {
            data = 0x7; // all bits to low fixed 0
        }
        else
        {
            data = 0x0; // default reset value high impedence state
        }
        if (!Pcf50626I2cWrite8(hDevice,PCF50626_GPIO5C1_ADDR, data))
            return NV_FALSE;
    }
    return NV_TRUE;
}

static NvBool 
Pcf50626ReadVoltageReg(
    NvOdmPmuDeviceHandle hDevice, 
    NvU32 vddRail, 
    NvU32* pMilliVolts)
{
    NvU32 milliVolts = 0;
    NvU8 data = 0;
    const PCF50626PmuSupplyInfo *pSupplyInfo = &pcf50626SupplyInfoTable[vddRail];

    NV_ASSERT(pSupplyInfo->supply == (PCF50626PmuSupply)vddRail);

    if(! Pcf50626I2cRead8(hDevice, pSupplyInfo->control2Addr, &data))
        return NV_FALSE;

    data >>= PCF50626_C2_OPMOD_SHIFT;
    if (!data) //OFF
        milliVolts = 0;
    else
    {
        if (!Pcf50626I2cRead8(hDevice, pSupplyInfo->control1Addr, &data))
            return NV_FALSE;
        
        if ( (vddRail == PCF50626PmuSupply_DCD1) 
            |(vddRail == PCF50626PmuSupply_DCD2) 
            |(vddRail == PCF50626PmuSupply_DCUD))
        {
            milliVolts = pSupplyInfo->offsetVoltage + pSupplyInfo->cap.StepMilliVolts * ((NvU32)(data & 0x7F));
        }
        else if (vddRail == PCF50626PmuSupply_LCREG)
        {
            milliVolts = pSupplyInfo->offsetVoltage + pSupplyInfo->cap.StepMilliVolts * ((NvU32)(data & 0x7F) >> 1);    
        }
        else
        {
            milliVolts = pSupplyInfo->offsetVoltage + pSupplyInfo->cap.StepMilliVolts * ((NvU32)(data & 0x7F) >> 2);    
        }
    }

    *pMilliVolts = milliVolts;
    return NV_TRUE;
}


static NvBool 
Pcf50626WriteVoltageReg(
    NvOdmPmuDeviceHandle hDevice, 
    NvU32 vddRail, 
    NvU32  MilliVolts, 
    NvU32* pSettleMicroSeconds)
{
    NvU8 data = 0;
    NvU8 reg = 0;
    NvU32 settleTime = 0;

    const PCF50626PmuSupplyInfo* pSupplyInfo = &pcf50626SupplyInfoTable[vddRail];
    const PCF50626PmuSupplyInfo* pSupplyInputInfo = &pcf50626SupplyInfoTable[pSupplyInfo->supplyInput];

    NV_ASSERT(pSupplyInfo->supply == (PCF50626PmuSupply)vddRail);

    // Require to turn off the supply
    if (MilliVolts == ODM_VOLTAGE_OFF)
    {
        // check if the supply can be turned off
        if (((Pcf50626PrivData*)hDevice->pPrivate)->supplyRefCntTable[pSupplyInfo->supply] == 1)
        {
            // turn off the supply
            data = PCF50626_C2_OPMOD_OFF;
            NvOdmServicesPmuSetSocRailPowerState(
                ((Pcf50626PrivData*)hDevice->pPrivate)->hOdmPmuSevice, pSupplyInfo->supply, NV_FALSE); 
            if (!Pcf50626I2cWrite8(hDevice, pSupplyInfo->control2Addr, data))
                return NV_FALSE;
        }

        //check if the supply input can be turned off
        if (((Pcf50626PrivData*)hDevice->pPrivate)->supplyRefCntTable[pSupplyInputInfo->supply] == 1)
        {
            // turn off the supply input
            data = PCF50626_C2_OPMOD_OFF;
            NvOdmServicesPmuSetSocRailPowerState(
                ((Pcf50626PrivData*)hDevice->pPrivate)->hOdmPmuSevice, pSupplyInputInfo->supply, NV_FALSE); 
            if(! Pcf50626I2cWrite8(hDevice, pSupplyInputInfo->control2Addr, data))
                return NV_FALSE;
        }

        if (((Pcf50626PrivData*)hDevice->pPrivate)->supplyRefCntTable[pSupplyInfo->supply] != 0)
            ((Pcf50626PrivData*)hDevice->pPrivate)->supplyRefCntTable[pSupplyInfo->supply] --;
        if (((Pcf50626PrivData*)hDevice->pPrivate)->supplyRefCntTable[pSupplyInputInfo->supply] != 0)
            ((Pcf50626PrivData*)hDevice->pPrivate)->supplyRefCntTable[pSupplyInputInfo->supply] --;

        settleTime = PMU_MAX (pSupplyInfo->switchTimeMicroSec, pSupplyInputInfo->switchTimeMicroSec);

        if (pSettleMicroSeconds != NULL)
            *pSettleMicroSeconds = settleTime;
        else
            NvOdmOsWaitUS(settleTime);
        return NV_TRUE; 
    }

    // set voltage
    if ( (vddRail == PCF50626PmuSupply_HCREG) || 
         ((vddRail == PCF50626PmuSupply_LCREG) &&
          (MilliVolts > PCF50626_LCREGOUT_VOLTAGE_RESCHANGE_MV)))
    {
        data = (NvU8)((MilliVolts - pSupplyInfo->offsetVoltage) / pSupplyInfo->cap.StepMilliVolts);
        if (data % 2)
            data --;
    }
    else
    {
        data = (NvU8)((MilliVolts - pSupplyInfo->offsetVoltage) / pSupplyInfo->cap.StepMilliVolts);
    }

    reg = 0;
    reg &= ~PCF50626_C1_OUTPUT_MASK;
    if ( (pSupplyInfo->supply == PCF50626PmuSupply_DCD1)
        |(pSupplyInfo->supply == PCF50626PmuSupply_DCD2)
        |(pSupplyInfo->supply == PCF50626PmuSupply_DCUD))
    {
        reg |= data;
    }
    else if (pSupplyInfo->supply == PCF50626PmuSupply_LCREG)
    {
        reg |= (data << 1);
    }   
    else
    {
        reg |= (data << 2);
    }
    
    if(! Pcf50626I2cWrite8(hDevice, pSupplyInfo->control1Addr, reg))
        return NV_FALSE;

    settleTime = pSupplyInfo->switchTimeMicroSec;

    // turn on supply
    if (((Pcf50626PrivData*)hDevice->pPrivate)->supplyRefCntTable[pSupplyInfo->supply] == 0)
    {
        if (! Pcf50626I2cRead8(hDevice, pSupplyInfo->control2Addr, &data))
            return NV_FALSE;
        data >>= PCF50626_C2_OPMOD_SHIFT;
        if (!data)
        {
            // Require to turn on the supply
            data = PCF50626_C2_OPMOD_ON;
            NvOdmServicesPmuSetSocRailPowerState(
                ((Pcf50626PrivData*)hDevice->pPrivate)->hOdmPmuSevice, pSupplyInfo->supply, NV_TRUE); 
            if(! Pcf50626I2cWrite8(hDevice, pSupplyInfo->control2Addr, data))
                return NV_FALSE;

            settleTime += pSupplyInfo->turnOnTimeMicroSec;
        }
    }

    // turn on supply input if necessary
    if (((Pcf50626PrivData*)hDevice->pPrivate)->supplyRefCntTable[pSupplyInputInfo->supply] == 0)
    {
        if(! Pcf50626I2cRead8(hDevice, pSupplyInputInfo->control2Addr, &data))
            return NV_FALSE;

        data >>= PCF50626_C2_OPMOD_SHIFT;
        if (!data)
        {
            // Require to turn on the supply input
            data = PCF50626_C2_OPMOD_ON;
            NvOdmServicesPmuSetSocRailPowerState(
                ((Pcf50626PrivData*)hDevice->pPrivate)->hOdmPmuSevice, pSupplyInputInfo->supply, NV_TRUE); 
            if(! Pcf50626I2cWrite8(hDevice,pSupplyInputInfo->control2Addr, data))
                return NV_FALSE;

            settleTime += pSupplyInputInfo->turnOnTimeMicroSec;
       }
    }

    ((Pcf50626PrivData*)hDevice->pPrivate)->supplyRefCntTable[pSupplyInputInfo->supply] ++;
    ((Pcf50626PrivData*)hDevice->pPrivate)->supplyRefCntTable[pSupplyInfo->supply] ++;


    if (pSettleMicroSeconds != NULL)
        *pSettleMicroSeconds = settleTime;
    else
        NvOdmOsWaitUS(settleTime);

    return NV_TRUE;
}

NvBool 
Pcf50626GetAcLineStatus(
    NvOdmPmuDeviceHandle hDevice, 
    NvOdmPmuAcLineStatus *pStatus)
{
    NvBool acLineStatus = NV_FALSE;
   
    NV_ASSERT(hDevice);
    NV_ASSERT(pStatus);
   
    // check if charger presents
    if (((Pcf50626PrivData*)hDevice->pPrivate)->battPresence == NV_FALSE)
    {
        *pStatus = NvOdmPmuAcLine_Online;
        return NV_TRUE;
    }

    if (((Pcf50626PrivData*)hDevice->pPrivate)->pmuInterruptSupported == NV_TRUE)
    {
        if (( ((Pcf50626PrivData*)hDevice->pPrivate)->pmuStatus.mChgPresent == NV_TRUE ) &&
            (UsbHostMode == NV_FALSE))
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
        if (!Pcf50626BatteryChargerMainChgPresent(hDevice, &acLineStatus))
        {
            NVODMPMU_PRINTF(("[NVODM PMU] Pcf50626GetAcLineStatus: Error in checking main charger presence.\n"));
            return NV_FALSE;
        }

        if ((acLineStatus == NV_TRUE) && (UsbHostMode == NV_FALSE)) 
            *pStatus = NvOdmPmuAcLine_Online;
        else
            *pStatus = NvOdmPmuAcLine_Offline;
    }    
    return NV_TRUE;
}


NvBool 
Pcf50626GetBatteryStatus(
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
        if (((Pcf50626PrivData*)hDevice->pPrivate)->battPresence == NV_TRUE)
        {   
            NvOdmPmuAcLineStatus stat = NvOdmPmuAcLine_Offline;
            NvU32 VBatSense = 0;
            if (!Pcf50626GetAcLineStatus(hDevice, &stat))
                return NV_FALSE;
            
            if (stat == NvOdmPmuAcLine_Online)
            {
                if (((Pcf50626PrivData*)hDevice->pPrivate)->pmuInterruptSupported == NV_TRUE)
                {
                    if (((Pcf50626PrivData*)hDevice->pPrivate)->pmuStatus.batFull == NV_FALSE)
                        status = NVODM_BATTERY_STATUS_CHARGING;
                }
                else
                {
                    NvBool batFull = NV_FALSE;
                    if (!Pcf50626BatteryChargerCBCBattFul(hDevice, &batFull))
                        return NV_FALSE;
                    if (batFull == NV_FALSE)
                        status = NVODM_BATTERY_STATUS_CHARGING;
                }
            }
            
            // Get VBatSense
            if (!Pcf50626AdcVBatSenseRead(hDevice, &VBatSense))
                return NV_FALSE;

            if (VBatSense > NVODM_BATTERY_HIGH_VOLTAGE_MV)
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
Pcf50626GetBatteryData(
    NvOdmPmuDeviceHandle hDevice, 
    NvOdmPmuBatteryInstance batteryInst,
    NvOdmPmuBatteryData *pData)
{
    NvOdmPmuBatteryData batteryData;
    batteryData.batteryAverageCurrent  = NVODM_BATTERY_DATA_UNKNOWN;
    batteryData.batteryAverageInterval = NVODM_BATTERY_DATA_UNKNOWN;
    batteryData.batteryCurrent         = NVODM_BATTERY_DATA_UNKNOWN;
    batteryData.batteryLifePercent     = NVODM_BATTERY_DATA_UNKNOWN;
    batteryData.batteryLifeTime        = NVODM_BATTERY_DATA_UNKNOWN;
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

        if (((Pcf50626PrivData*)hDevice->pPrivate)->battPresence == NV_TRUE)
        {
            /* retrieve Battery voltage and temperature */
            
            // Get VBatSense
            if (!Pcf50626AdcVBatSenseRead(hDevice, &VBatSense))
            {
                NVODMPMU_PRINTF(("[NVODM PMU] Pcf50626GetBatteryData: Error reading VBATSense. \n"));
                return NV_FALSE;
            }

            // Get VBatTemp
            if (!Pcf50626AdcVBatTempRead(hDevice, &VBatTemp))
            {
                NVODMPMU_PRINTF(("[NVODM PMU] Pcf50626GetBatteryData: Error reading VBATSense. \n"));
                return NV_FALSE;
            }

            batteryData.batteryLifePercent = 
                    Pcf50626CalulateBatteryLifePercent_int(VBatSense);     

#if BATTEMP_CONTROL
            if (!Pcf50626BatteryTemperatureControl_int(hDevice, VBatTemp))
            {
                NVODMPMU_PRINTF(("[NVODM PMU] Pcf50626GetBatteryData: Error in battery ctemperature controls. \n"));
                return NV_FALSE;
            }
#endif

            batteryData.batteryVoltage = VBatSense;
            batteryData.batteryTemperature = Pcf50626BatteryTemperature(VBatSense, 
                                                                        VBatTemp);
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
Pcf50626GetBatteryFullLifeTime(
    NvOdmPmuDeviceHandle hDevice, 
    NvOdmPmuBatteryInstance batteryInst,
    NvU32 *pLifeTime)
{
    *pLifeTime = NVODM_BATTERY_DATA_UNKNOWN;
}

void
Pcf50626GetBatteryChemistry(
    NvOdmPmuDeviceHandle hDevice, 
    NvOdmPmuBatteryInstance batteryInst,
    NvOdmPmuBatteryChemistry *pChemistry)
{
    //return fixed data for now.
    *pChemistry = NvOdmPmuBatteryChemistry_LION;
}

NvBool 
Pcf50626SetChargingCurrent( 
NvOdmPmuDeviceHandle hDevice, 
NvOdmPmuChargingPath chargingPath, 
NvU32 chargingCurrentLimitMa,
NvOdmUsbChargerType chargerType)
{
    NvU8 data = 0;
    NV_ASSERT(hDevice);

    // if no battery, then do nothing
    if (((Pcf50626PrivData*)hDevice->pPrivate)->battPresence == NV_FALSE)
        return NV_TRUE;
    //Concorde s/w WAR for USB Host mode
    if (chargingCurrentLimitMa == NVODM_USB_HOST_MODE_LIMIT) 
    {
        chargingCurrentLimitMa = 0; // turn off the charging path
        UsbHostMode = NV_TRUE;
    }
    else
    {
        UsbHostMode = NV_FALSE;
    }

    // if requested current is more than max supported current then limit to supported
    if ( chargingCurrentLimitMa > MAX_CHARGER_LIMIT_MA )
        chargingCurrentLimitMa = MAX_CHARGER_LIMIT_MA;

    if (chargingPath == NvOdmPmuChargingPath_UsbBus)
    {
        switch (chargerType)
        {
            case NvOdmUsbChargerType_SJ:
                chargingCurrentLimitMa = SJ_TYPE_CHARGER_LIMIT_MA;
                break;
            case NvOdmUsbChargerType_SK:
                chargingCurrentLimitMa = SK_TYPE_CHARGER_LIMIT_MA;
                break;
            case NvOdmUsbChargerType_SE1:
                chargingCurrentLimitMa = SE1_TYPE_CHARGER_LIMIT_MA;
                break;
            case NvOdmUsbChargerType_SE0:
                chargingCurrentLimitMa = SE0_TYPE_CHARGER_LIMIT_MA;
                break;
            case NvOdmUsbChargerType_UsbHost:
                default:
                // USB Host based charging, nothing to do. Just pass current limit to PMU.
                break;
        }
    }

    data = (NvU8)((( chargingCurrentLimitMa << 8 ) - chargingCurrentLimitMa ) 
                / CHARGER_CONSTANT_CURRENT_SET_MA );

    if (!Pcf50626I2cWrite8(hDevice, PCF50626_CBCC3_ADDR, data))
        return NV_FALSE;

    // turn off the charger path if the requested current limit is 0mA. Turn on the path otherwise. 
    data = 0;
    if ( !Pcf50626I2cRead8(hDevice, PCF50626_CBCC1_ADDR, &data) )
        return NV_FALSE;
  
    if ( chargingCurrentLimitMa == 0 )
        data &= ~(PCF50626_CBCC1_CHGENA_MASK); //off
    else
        data |= PCF50626_CBCC1_CHGENA_MASK;    //on

    if ( !Pcf50626I2cWrite8(hDevice, PCF50626_CBCC1_ADDR, data) )
        return NV_FALSE;


    data = 0;
    if ( !Pcf50626I2cRead8(hDevice, PCF50626_CBCC2_ADDR, &data) )
       return NV_FALSE;      
    if ( chargingCurrentLimitMa == 0 )
    {
        //enable USB suspend mode regardless of the SCUSB pin state
        data |= (PCF50626_CBCC2_SUSPENA_MASK << PCF50626_CBCC2_SUSPENA_SHIFT);
    }
    else
    {
        //disable USB suspend mode regardless of the SCUSB pin state
        data &= ~(PCF50626_CBCC2_SUSPENA_MASK << PCF50626_CBCC2_SUSPENA_SHIFT);    
    }
    if ( !Pcf50626I2cWrite8(hDevice, PCF50626_CBCC2_ADDR, data) )
        return NV_FALSE;

    //Dump the register value for debug purpose, can be commented out is undesired..
    NVODMPMU_PRINTF(("NvOdmPmuSetChargingCurrent: \n"));
    NVODMPMU_PRINTF(("  chargingCurrentLimitMa:%d\n", chargingCurrentLimitMa));    

    if ( !Pcf50626I2cRead8(hDevice, PCF50626_CBCC1_ADDR, &data) )
      return NV_FALSE;
    NVODMPMU_PRINTF(("  CBCC1:0x%02x\n", data));    

    if ( !Pcf50626I2cRead8(hDevice, PCF50626_CBCC2_ADDR, &data) )
          return NV_FALSE;
    NVODMPMU_PRINTF(("  CBCC2:0x%02x\n", data));    

    if ( !Pcf50626I2cRead8(hDevice, PCF50626_CBCC3_ADDR, &data) )
          return NV_FALSE;
    NVODMPMU_PRINTF(("  CBCC3:0x%02x\n", data));    
    
    return NV_TRUE;
}

void Pcf50626InterruptHandler( NvOdmPmuDeviceHandle  hDevice)
{
    // If the interrupt handle is called, the interrupt is supported.
    ((Pcf50626PrivData*)hDevice->pPrivate)->pmuInterruptSupported = NV_TRUE;
    
    Pcf50626InterruptHandler_int(hDevice, &((Pcf50626PrivData*)hDevice->pPrivate)->pmuStatus);
}


static NvU32 Pcf50626CalulateBatteryLifePercent_int(NvU32 vBatSense)
{
    NvU32 lifePerc = 0;
    NvU32 vbat = vBatSense;

    if (vbat < NVODM_BATTERY_CRITICAL_VOLTAGE_MV)
        vbat = NVODM_BATTERY_CRITICAL_VOLTAGE_MV;

    // using the linear mapping between the battery voltage and the life percentage.
    lifePerc = ( ( vbat - NVODM_BATTERY_CRITICAL_VOLTAGE_MV ) * 50
                / ( NVODM_BATTERY_FULL_VOLTAGE_MV - NVODM_BATTERY_CRITICAL_VOLTAGE_MV ) )  << 1;

    if (lifePerc > 100)
        lifePerc = 100;
    
    return lifePerc;
}


#if BATTEMP_CONTROL
static NvBool Pcf50626BatteryTemperatureControl_int(
                   NvOdmPmuDeviceHandle hDevice,
                   NvU32 batTemp)
{
    NvU8 data = 0;
    
    //turn off the charger if the battery is overheating. 
    if ( batTemp > NVODM_BATTERY_OVERHEAT_THRESHOLD )  
    {
        if (!Pcf50626I2cRead8(hDevice, PCF50626_CBCC1_ADDR, &data))
            return NV_FALSE;

        data &= 0xFE;
        if (!Pcf50626I2cWrite8(hDevice, PCF50626_CBCC1_ADDR, data))
            return NV_FALSE;
    }
    // turn it on otherwise
    else
    {    
        if (!Pcf50626I2cRead8(hDevice, PCF50626_CBCC1_ADDR, &data))
            return NV_FALSE;
        
        data |= 0x01;
        if (!Pcf50626I2cWrite8(hDevice, PCF50626_CBCC1_ADDR, data))
            return NV_FALSE;
    }

    return NV_TRUE;
}
#endif



