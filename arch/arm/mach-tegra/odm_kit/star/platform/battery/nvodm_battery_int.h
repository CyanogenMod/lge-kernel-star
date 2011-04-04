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

#ifndef INCLUDED_NVODM_BATTERY_INT_H
#define INCLUDED_NVODM_BATTERY_INT_H

#include "nvodm_query_gpio.h"
#include "nvrm_gpio.h"
#include "nvodm_services.h"
#include "nvec.h"

/* Module debug msg: 0=disable, 1=enable */
#define NVODMBATTERY_ENABLE_PRINTF (0)

#if (NVODMBATTERY_ENABLE_PRINTF)
#define NVODMBATTERY_PRINTF(x)   NvOdmOsDebugPrintf x
#else
#define NVODMBATTERY_PRINTF(x)
#endif

#if defined(__cplusplus)
extern "C"
{
#endif

/****************************************************************************/
/*
 * Macro to disable EC calls for battery operations
 * until EC firware supports it
 */
#define NVEC_BATTERY_DISABLED 0
/*
 * Some extra battery info added which is not yet part of the
 * BatteryData struct.
 * Enable it to verify the these extra info with the EC firware
 */
#define BATTERY_EXTRA_INFO    0

/* Enable to wakeup the AP from suspend */
#define NVODM_WAKEUP_FROM_BATTERY_EVENT 1
#define NVODM_WAKEUP_FROM_AC_EVENT      1

/* Enable the Low Battery GPIO Interrupt */
#define NVODM_LOWBATTERY_GPIO_INT       1

/* Enable low capacity alarm wakeup */
#define NVODM_BATTERY_LOW_CAPACITY_ALARM 1
/****************************************************************************/

#define NVODM_BATTERY_NUM_BATTERY_SLOTS_MASK 0x0F

/* Battery Slot Status and Capacity Gauge Report */
/* Data Byte 3 : Battery Slot Status */
#define NVODM_BATTERY_SLOT_STATUS_DATA  0
/*
 * Data Byte 4 : Battery Capacity Gauge :
 * Battery's relative remaining capacity in %
 */
#define NVODM_BATTERY_CAPACITY_GUAGE_DATA  1

/*
 * Battery Slot Status :
 * Bit 0 = Present State:
 * 1 = Battery is present in the respective slot
 */
#define NVODM_BATTERY_PRESENT_IN_SLOT  0x01

#define NVODM_BATTERY_CHARGING_STATE_SHIFT  1
#define NVODM_BATTERY_CHARGING_STATE_MASK   0x03

/* Battery Slot Status : Bits 1-2 = Charging state */
#define NVODM_BATTERY_CHARGING_STATE_IDLE         0x00
#define NVODM_BATTERY_CHARGING_STATE_CHARGING     0x01
#define NVODM_BATTERY_CHARGING_STATE_DISCHARGING  0x02
#define NVODM_BATTERY_CHARGING_STATE_RESERVED     0x03

/* Remaining capacity alarm bit is 3rd in slot status */
#define NVODM_BATTERY_REM_CAP_ALARM_SHIFT 3
#define NVODM_BATTERY_REM_CAP_ALARM_IS_SET 1

/* Response System Status : Data Byte 3 System State Bits 7-0 */
#define NVODM_BATTERY_SYSTEM_STATE_DATA1 0
/* Response System Status : Data Byte 4 System State Bits 15-8 */
#define NVODM_BATTERY_SYSTEM_STATE_DATA2 1
/* System State Flags : AC Present : System State Bit 0 */
#define NVODM_BATTERY_SYSTEM_STATE_AC_PRESENT 0x01

#define NVODM_BATTERY_CHARGING_RATE_DATA_BYTES 3
#define NVODM_BATTERY_CHARGING_RATE_UNIT 3

/* Threshold for battery status.*/
#define NVODM_BATTERY_FULL_VOLTAGE_MV      12600
#define NVODM_BATTERY_HIGH_VOLTAGE_MV      10200
#define NVODM_BATTERY_LOW_VOLTAGE_MV       10000
#define NVODM_BATTERY_CRITICAL_VOLTAGE_MV   9500

#define NVODM_BATTERY_EC_FIRMWARE_VER_R01 2
#define NVODM_BATTERY_EC_FIRMWARE_VER_R04 8

/* Bit 0 = Present State event */
/* Bit 1 = Charging State event */
/* Bit 2 = Remaining Capacity Alaram event */
#define NVODM_BATTERY_SET_PRESENT_EVENT       0x01
#define NVODM_BATTERY_SET_CHARGING_EVENT      0x02
#define NVODM_BATTERY_SET_REM_CAP_ALARM_EVENT 0x04

/*
 * Bit 0   => 0=Not Present, 1=Present
 * Bit 1:2 => 00=Idle, 01=Charging,10=Discharging, 11=Reserved
 * Bit 3   => 1=Remaining Capacity Alaram set
 */
#define NVODM_BATTERY_EVENT_MASK 0x0F

typedef enum
{
    NvOdmBattCharingUnit_mW, /* Milli Watt */
    NvOdmBattCharingUnit_mA, /* Milli Amps */
    NvOdmBattCharingUnit_10mW, /* Milli Watt * 10 */

    NvOdmBattCharingUnit_Num,
    NvOdmBattCharingUnit_Max = 0x7fffffff

} NvOdmBattCharingUnit;

typedef struct NvOdmBatteryDeviceRec
{
    NvEcHandle     hEc;
    NvEcEventRegistrationHandle hEcEventReg;
    NvOdmOsSemaphoreHandle      hBattEventSem;
    NvOdmOsSemaphoreHandle      hClientBattEventSem;
    NvOdmOsThreadHandle         hBattEventThread;
#if NVODM_LOWBATTERY_GPIO_INT
    const NvOdmGpioPinInfo      *pGpioPinInfo;
    NvRmGpioPinHandle           hPin;
    NvRmGpioInterruptHandle     GpioIntrHandle;
    NvU32                       PinCount;
    NvRmDeviceHandle            hRm;
    NvRmGpioHandle              hGpio;
#endif
    NvU8                        BatteryEvent;
    NvU8                        ECVersion;
    NvBool                      ExitThread;
} NvOdmBatteryDevice;

#if defined(__cplusplus)
}
#endif

/** @} */

#endif /* INCLUDED_NVODM_BATTERY_INT_H */

