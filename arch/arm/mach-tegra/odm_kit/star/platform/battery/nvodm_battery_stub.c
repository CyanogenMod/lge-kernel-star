/*
 * Copyright (c) 2009-2010 NVIDIA Corporation.
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

#include "nvcommon.h"
#include "nvodm_battery.h"

typedef struct NvOdmBatteryDeviceRec
{
    NvBool                      bBattPresent;
    NvBool                      bBattFull;
} NvOdmBatteryDevice;

/**
 * Gets the battery event.
 *
 * @param hDevice A handle to the EC.
 * @param pBatteryEvent Battery events
 * 
 */
void NvOdmBatteryGetEvent(
     NvOdmBatteryDeviceHandle hDevice,
     NvU8   *pBatteryEvent)
{
    NvOdmBatteryDevice *pBattContext = NULL;

    pBattContext = (NvOdmBatteryDevice *)hDevice;

    *pBatteryEvent = 0;
}

NvBool NvOdmBatteryDeviceOpen(NvOdmBatteryDeviceHandle *hDevice,
                              NvOdmOsSemaphoreHandle *hOdmSemaphore)
{
    *hDevice = NULL;
    return NV_FALSE;
}

void NvOdmBatteryDeviceClose(NvOdmBatteryDeviceHandle hDevice)
{
}

/**
 * Gets the AC line status.
 *
 * @param hDevice A handle to the EC.
 * @param pStatus A pointer to the AC line
 *  status returned by the ODM.
 * 
 * @return NV_TRUE if successful, or NV_FALSE otherwise.
 */
NvBool NvOdmBatteryGetAcLineStatus(
       NvOdmBatteryDeviceHandle hDevice,
       NvOdmBatteryAcLineStatus *pStatus)
{
    *pStatus = NvOdmBatteryAcLine_Offline;
    return NV_FALSE;
}


/**
 * Gets the battery status.
 *
 * @param hDevice A handle to the EC.
 * @param batteryInst The battery type.
 * @param pStatus A pointer to the battery
 *  status returned by the ODM.
 * 
 * @return NV_TRUE if successful, or NV_FALSE otherwise.
 */
NvBool NvOdmBatteryGetBatteryStatus(
       NvOdmBatteryDeviceHandle hDevice,
       NvOdmBatteryInstance batteryInst,
       NvU8 *pStatus)
{
    *pStatus = NVODM_BATTERY_STATUS_UNKNOWN;
    return NV_FALSE;
}

/**
 * Gets the battery data.
 *
 * @param hDevice A handle to the EC.
 * @param batteryInst The battery type.
 * @param pData A pointer to the battery
 *  data returned by the ODM.
 * 
 * @return NV_TRUE if successful, or NV_FALSE otherwise.
 */
NvBool NvOdmBatteryGetBatteryData(
       NvOdmBatteryDeviceHandle hDevice,
       NvOdmBatteryInstance batteryInst,
       NvOdmBatteryData *pData)
{
    NvOdmBatteryData BatteryData;

    BatteryData.BatteryAverageCurrent  = NVODM_BATTERY_DATA_UNKNOWN;
    BatteryData.BatteryAverageInterval = NVODM_BATTERY_DATA_UNKNOWN;
    BatteryData.BatteryCurrent         = NVODM_BATTERY_DATA_UNKNOWN;
    BatteryData.BatteryLifePercent     = NVODM_BATTERY_DATA_UNKNOWN;
    BatteryData.BatteryLifeTime        = NVODM_BATTERY_DATA_UNKNOWN;
    BatteryData.BatteryMahConsumed     = NVODM_BATTERY_DATA_UNKNOWN;
    BatteryData.BatteryTemperature     = NVODM_BATTERY_DATA_UNKNOWN;
    BatteryData.BatteryVoltage         = NVODM_BATTERY_DATA_UNKNOWN;

    *pData = BatteryData;
    return NV_FALSE;
}

/**
 * Gets the battery full life time.
 *
 * @param hDevice A handle to the EC.
 * @param batteryInst The battery type.
 * @param pLifeTime A pointer to the battery
 *  full life time returned by the ODM.
 * 
 */
void NvOdmBatteryGetBatteryFullLifeTime(
     NvOdmBatteryDeviceHandle hDevice,
     NvOdmBatteryInstance batteryInst,
     NvU32 *pLifeTime)
{
    *pLifeTime = NVODM_BATTERY_DATA_UNKNOWN;
}


/**
 * Gets the battery chemistry.
 *
 * @param hDevice A handle to the EC.
 * @param batteryInst The battery type.
 * @param pChemistry A pointer to the battery
 *  chemistry returned by the ODM.
 * 
 */
void NvOdmBatteryGetBatteryChemistry(
     NvOdmBatteryDeviceHandle hDevice,
     NvOdmBatteryInstance batteryInst,
     NvOdmBatteryChemistry *pChemistry)
{
    *pChemistry = NVODM_BATTERY_DATA_UNKNOWN;
}



