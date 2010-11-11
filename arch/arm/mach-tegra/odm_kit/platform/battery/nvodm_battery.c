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

#include "nvodm_query_discovery.h"
#include "nvodm_services.h"
#include "nvodm_battery_int.h"
#include "nvodm_battery.h"
#include "nvec.h"

NvBool NvOdmBatteryPrivGetSlotStatusAndCapacityGuage(
       NvOdmBatteryDevice *pBattContext,
       NvOdmBatteryInstance BatteryInst,
       NvU8 *BatterySlotStatus,
       NvU8 *BatteryCapacityGuage);

NvBool NvOdmBatteryPrivGetLifeTime(
       NvOdmBatteryDevice *pBattContext,
       NvOdmBatteryInstance BatteryInst,
       NvU32 *BatteryLifeTime);

NvBool NvOdmPrivBattGetBatteryVoltage(
       NvOdmBatteryDevice *pBattContext,
       NvOdmBatteryInstance BatteryInst,
       NvU32 *BatteryVoltage);

NvBool NvOdmBatteryPrivGetCurrent(
       NvOdmBatteryDeviceHandle hDevice,
       NvOdmBatteryInstance BatteryInst,
       NvS32 *pCurrent);

NvBool NvOdmBatteryPrivGetAverageCurrent(
       NvOdmBatteryDeviceHandle hDevice,
       NvOdmBatteryInstance BatteryInst,
       NvS32 *pAverageCurrent);

NvBool NvOdmBatteryPrivGetAverageTimeInterval(
       NvOdmBatteryDeviceHandle hDevice,
       NvOdmBatteryInstance BatteryInst,
       NvU32 *pAverageTimeInterval);

NvBool NvOdmBatteryPrivGetTemperature(
       NvOdmBatteryDeviceHandle hDevice,
       NvOdmBatteryInstance BatteryInst,
       NvU32 *pBatteryTemp);

NvBool NvOdmBatteryPrivGetRemainingCapacity(
       NvOdmBatteryDeviceHandle hDevice,
       NvOdmBatteryInstance BatteryInst,
       NvU32 *pRemCapacity);

NvBool NvOdmBatteryPrivGetLastFullChargeCapacity(
       NvOdmBatteryDeviceHandle hDevice,
       NvOdmBatteryInstance BatteryInst,
       NvU32 *pLastFullChargeCapacity);

NvBool NvOdmBatteryPrivGetCriticalCapacity(
       NvOdmBatteryDeviceHandle hDevice,
       NvOdmBatteryInstance BatteryInst,
       NvU32 *pCriticalCapacity);

NvBool NvOdmBatterySetRemCapacityAlarm(
       NvOdmBatteryDeviceHandle hDevice,
       NvOdmBatteryInstance BatteryInst,
       NvU16 RemCapAlarm);

#if BATTERY_EXTRA_INFO
NvBool NvOdmBatteryGetRemCapacityAlarm(
       NvOdmBatteryDeviceHandle hDevice,
       NvOdmBatteryInstance BatteryInst,
       NvU16 *RemCapAlarm);

NvBool NvOdmBatterySetConfiguration(
       NvOdmBatteryDeviceHandle hDevice,
       NvOdmBatteryInstance BatteryInst,
       NvU8 Configuration);

NvBool NvOdmBatteryGetConfiguration(
       NvOdmBatteryDeviceHandle hDevice,
       NvOdmBatteryInstance BatteryInst,
       NvU8 *Configuration);
#endif

#if NVODM_LOWBATTERY_GPIO_INT
/*
 * GPIO interrupt handle for battery events
 */
static void
NvOdmBatteryGpioInterruptHandler(void *args)
{
    NvOdmBatteryDevice *pBattContext = args;

    NVODMBATTERY_PRINTF(("NvOdmBatteryGpioInterruptHandler:\n"));

    if (pBattContext)
    {
        pBattContext->BatteryEvent = NvOdmBatteryEventType_RemainingCapacityAlarm;

        if (pBattContext->hClientBattEventSem)
            NvOdmOsSemaphoreSignal(pBattContext->hClientBattEventSem);
    }

    NvRmGpioInterruptDone(pBattContext->GpioIntrHandle);
}
#endif

/*
 * Gets the EC Firmware version number
 */
static NvError NvOdmBatteryPrivEcGetFirmwareVersion(
               NvOdmBatteryDevice *pBattContext,
               NvS32 *MajorVersion,
               NvS32 *MinorVersion)
{
    NvEcControlGetFirmwareVersionResponsePayload FirmwareVersion;
    NvError      RetVal = NvSuccess;
    NvEcRequest  EcRequest = {0};
    NvEcResponse EcResponse = {0};
    NvS32        Version = 0;

    EcRequest.PacketType = NvEcPacketType_Request;
    EcRequest.RequestType = NvEcRequestResponseType_Control;
    EcRequest.RequestSubtype = (NvEcRequestResponseSubtype)
                          NvEcControlSubtype_GetFirmwareVersion;
    EcRequest.NumPayloadBytes = 0;
    RetVal = NvEcSendRequest(pBattContext->hEc, &EcRequest, &EcResponse,
                            sizeof(EcRequest), sizeof(EcResponse));
    if (RetVal != NvSuccess)
        return RetVal;

    if (EcResponse.Status != NvEcStatus_Success)
        return NvError_BadValue;

    NvOdmOsMemcpy(&FirmwareVersion, EcResponse.Payload,
                EcResponse.NumPayloadBytes);

    Version = (FirmwareVersion.VersionMajor[1] << 24) |
              (FirmwareVersion.VersionMajor[0] << 16) |
              (FirmwareVersion.VersionMinor[1] << 8) |
              (FirmwareVersion.VersionMinor[0] << 0);

    *MajorVersion = Version & 0xFF00;
    *MinorVersion = Version & 0xFF;
    return NvSuccess;
}

static void NvOdmBatteryEventHandler(void *args)
{
    NvOdmBatteryDevice *pBattContext = args;
    NvError NvStatus = NvError_Success;
    NvEcEvent EcEvent = {0};
    NvU8      BattEvent = 0, ChargingState = 0;

    for (;;)
    {
        NvOdmOsSemaphoreWait(pBattContext->hBattEventSem);

        if (pBattContext->ExitThread == NV_TRUE)
            break;

        NVODMBATTERY_PRINTF(("NvOdmBatteryEventHandler:hBattEventSem signaled\n"));

        if (pBattContext->hEcEventReg)
        {
            NvStatus = NvEcGetEvent(pBattContext->hEcEventReg,
                                    &EcEvent, sizeof(NvEcEvent));
            if (NvStatus != NvError_Success)
            {
                NV_ASSERT(!"Could not receive EC event\n");
                continue;
            }

            if (EcEvent.NumPayloadBytes == 0)
            {
                NV_ASSERT(!"Received Battery event with no data\n");
                continue;
            }

            /* EcEvent.Payload[0] is Slot number */

            /* EcEvent.Payload[1] has 4 lsb bits for battery events */
            BattEvent = EcEvent.Payload[1] & NVODM_BATTERY_EVENT_MASK;

            pBattContext->BatteryEvent = 0;
            /* Read the Battery Slot status to set the proper event */
            if (BattEvent & NVODM_BATTERY_PRESENT_IN_SLOT)
                pBattContext->BatteryEvent |= NvOdmBatteryEventType_Present;

            ChargingState = BattEvent >> NVODM_BATTERY_CHARGING_STATE_SHIFT;
            ChargingState &= NVODM_BATTERY_CHARGING_STATE_MASK;
            if (ChargingState == NVODM_BATTERY_CHARGING_STATE_IDLE)
                pBattContext->BatteryEvent |= NvOdmBatteryEventType_Idle;
            else if (ChargingState == NVODM_BATTERY_CHARGING_STATE_CHARGING)
                pBattContext->BatteryEvent |= NvOdmBatteryEventType_Charging;
            else if (ChargingState == NVODM_BATTERY_CHARGING_STATE_DISCHARGING)
                pBattContext->BatteryEvent |= NvOdmBatteryEventType_Disharging;

            ChargingState = BattEvent >> NVODM_BATTERY_REM_CAP_ALARM_SHIFT;
            if (ChargingState == NVODM_BATTERY_REM_CAP_ALARM_IS_SET)
                pBattContext->BatteryEvent |= NvOdmBatteryEventType_RemainingCapacityAlarm;

            /* Signal the Battery Client for arrival of the valid event */
            if ((pBattContext->hClientBattEventSem != 0) &&
                (pBattContext->BatteryEvent != 0))
                 NvOdmOsSemaphoreSignal(pBattContext->hClientBattEventSem);
        }
    }
}

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

    NVODMBATTERY_PRINTF(("[ENTER] NvOdmBatteryGetEvent.\n"));

    pBattContext = (NvOdmBatteryDevice *)hDevice;

    *pBatteryEvent = pBattContext->BatteryEvent;
}

/**
 * Sets the battery remaining capacity alarm threshold.
 *
 * @param hDevice [IN] A handle to the EC.
 * @param BatteryInst [IN] The battery type.
 * @param RemCapAlarm [IN] Capacity Units
 */
NvBool NvOdmBatterySetRemCapacityAlarm(
       NvOdmBatteryDeviceHandle hDevice,
       NvOdmBatteryInstance BatteryInst,
       NvU16 RemCapAlarm)
{
#if NVEC_BATTERY_DISABLED
    ;
#else
    NvError NvStatus = NvError_Success;
    NvEcRequest EcRequest = {0};
    NvEcResponse EcResponse = {0};
    NvOdmBatteryDevice *pBattContext = NULL;

    NVODMBATTERY_PRINTF(("[ENTER]NvOdmBatterySetRemCapacityAlarm.\n"));
    pBattContext = (NvOdmBatteryDevice *)hDevice;

    /* Fill up request structure */
    EcRequest.PacketType = NvEcPacketType_Request;
    EcRequest.RequestType = NvEcRequestResponseType_Battery;
    EcRequest.RequestSubtype = NvEcBatterySubtype_SetRemainingCapacityAlarm;
    EcRequest.NumPayloadBytes = 2;
    EcRequest.Payload[0] = (RemCapAlarm & 0x00FF);
    EcRequest.Payload[1] = (RemCapAlarm & 0xFF00) >> 8;

    /* Request to EC */
    NvStatus = NvEcSendRequest(pBattContext->hEc, &EcRequest, &EcResponse,
                              sizeof(EcRequest), sizeof(EcResponse));
    if (NvSuccess != NvStatus)
    {
        NVODMBATTERY_PRINTF(("NvEcSendRequest failed for NvOdmBatterySetRemCapacityAlarm\n"));
        return NV_FALSE;
    }

    if(EcResponse.Status == NvEcStatus_Success)
    {
        NVODMBATTERY_PRINTF(("EcResponse.Status failed for NvOdmBatterySetRemCapacityAlarm\n"));
        return NV_FALSE;
    }

    NVODMBATTERY_PRINTF(("[EXIT]NvOdmBatterySetRemCapacityAlarm.\n"));
#endif /* end of NVEC_BATTERY_DISABLED */
    return NV_TRUE;
}

#if BATTERY_EXTRA_INFO
/**
 * Gets the battery remaining capacity alarm threshold.
 *
 * @param hDevice [IN] A handle to the EC.
 * @param BatteryInst [IN] The battery type.
 * @param RemCapAlarm [OUT] Capacity Units
 */
NvBool NvOdmBatteryGetRemCapacityAlarm(
       NvOdmBatteryDeviceHandle hDevice,
       NvOdmBatteryInstance BatteryInst,
       NvU16 *RemCapAlarm)
{
#if NVEC_BATTERY_DISABLED
    *RemCapAlarm = 0;
#else
    NvError NvStatus = NvError_Success;
    NvEcRequest EcRequest = {0};
    NvEcResponse EcResponse = {0};
    NvOdmBatteryDevice *pBattContext = NULL;

    NVODMBATTERY_PRINTF(("[ENTER]NvOdmBatteryGetRemCapacityAlarm.\n"));
    *RemCapAlarm = 0;
    pBattContext = (NvOdmBatteryDevice *)hDevice;

    /* Fill up request structure */
    EcRequest.PacketType = NvEcPacketType_Request;
    EcRequest.RequestType = NvEcRequestResponseType_Battery;
    EcRequest.RequestSubtype = NvEcBatterySubtype_GetRemainingCapacityAlarm;
    EcRequest.NumPayloadBytes = 0;
    EcRequest.Payload[0] = 0;

    /* Request to EC */
    NvStatus = NvEcSendRequest(pBattContext->hEc, &EcRequest, &EcResponse,
                              sizeof(EcRequest), sizeof(EcResponse));
    if (NvSuccess != NvStatus)
    {
        NVODMBATTERY_PRINTF(("NvEcSendRequest failed for NvOdmBatteryGetRemCapacityAlarm\n"));
        return NV_FALSE;
    }

    if(EcResponse.Status == NvEcStatus_Success)
    {
        *RemCapAlarm = EcResponse.Payload[0];
        *RemCapAlarm |= EcResponse.Payload[1] << 8;
    }

    NVODMBATTERY_PRINTF(("[EXIT]NvOdmBatteryGetRemCapacityAlarm.\n"));
#endif /* end of NVEC_BATTERY_DISABLED */
    return NV_TRUE;
}

/**
 * Sets the battery capacity unit configuration (mAh or 10mWh)
 *
 * @param hDevice [IN] A handle to the EC.
 * @param BatteryInst [IN] The battery type.
 * @param Configuration [IN] Capacity Unit
 */
NvBool NvOdmBatterySetConfiguration(
       NvOdmBatteryDeviceHandle hDevice,
       NvOdmBatteryInstance BatteryInst,
       NvU8 Configuration)
{
#if NVEC_BATTERY_DISABLED
    ;
#else
    NvError NvStatus = NvError_Success;
    NvEcRequest EcRequest = {0};
    NvEcResponse EcResponse = {0};
    NvOdmBatteryDevice *pBattContext = NULL;

    NVODMBATTERY_PRINTF(("[ENTER]NvOdmBatterySetConfiguration.\n"));
    pBattContext = (NvOdmBatteryDevice *)hDevice;

    /* Fill up request structure */
    EcRequest.PacketType = NvEcPacketType_Request;
    EcRequest.RequestType = NvEcRequestResponseType_Battery;
    EcRequest.RequestSubtype = NvEcBatterySubtype_SetConfiguration;
    EcRequest.NumPayloadBytes = 1;
    EcRequest.Payload[0] = Configuration;

    /* Request to EC */
    NvStatus = NvEcSendRequest(pBattContext->hEc, &EcRequest, &EcResponse,
                              sizeof(EcRequest), sizeof(EcResponse));
    if (NvSuccess != NvStatus)
    {
        NVODMBATTERY_PRINTF(("NvEcSendRequest failed for NvOdmBatterySetConfiguration\n"));
        return NV_FALSE;
    }

    if(EcResponse.Status == NvEcStatus_Success)
    {
        NVODMBATTERY_PRINTF(("EcResponse.Status failed for NvOdmBatterySetConfiguration\n"));
        return NV_FALSE;
    }

    NVODMBATTERY_PRINTF(("[EXIT]NvOdmBatterySetConfiguration.\n"));
#endif /* end of NVEC_BATTERY_DISABLED */
    return NV_TRUE;
}

/**
 * Gets the battery capacity unit configuration (mAh or 10mWh).
 *
 * @param hDevice [IN] A handle to the EC.
 * @param BatteryInst [IN] The battery type.
 * @param Configuration [OUT] Capacity Unit
 */
NvBool NvOdmBatteryGetConfiguration(
       NvOdmBatteryDeviceHandle hDevice,
       NvOdmBatteryInstance BatteryInst,
       NvU8 *Configuration)
{
#if NVEC_BATTERY_DISABLED
    *Configuration = 0;
#else
    NvError NvStatus = NvError_Success;
    NvEcRequest EcRequest = {0};
    NvEcResponse EcResponse = {0};
    NvOdmBatteryDevice *pBattContext = NULL;

    NVODMBATTERY_PRINTF(("[ENTER]NvOdmBatteryGetConfiguration.\n"));
    *Configuration = 0;
    pBattContext = (NvOdmBatteryDevice *)hDevice;

    /* Fill up request structure */
    EcRequest.PacketType = NvEcPacketType_Request;
    EcRequest.RequestType = NvEcRequestResponseType_Battery;
    EcRequest.RequestSubtype = NvEcBatterySubtype_GetConfiguration;
    EcRequest.NumPayloadBytes = 0;
    EcRequest.Payload[0] = 0;

    /* Request to EC */
    NvStatus = NvEcSendRequest(pBattContext->hEc, &EcRequest, &EcResponse,
                              sizeof(EcRequest), sizeof(EcResponse));
    if (NvSuccess != NvStatus)
    {
        NVODMBATTERY_PRINTF(("NvEcSendRequest failed for NvOdmBatteryGetConfiguration\n"));
        return NV_FALSE;
    }

    if(EcResponse.Status == NvEcStatus_Success)
    {
        *Configuration = EcResponse.Payload[0];
    }

    NVODMBATTERY_PRINTF(("[EXIT]NvOdmBatteryGetConfiguration.\n"));
#endif /* end of NVEC_BATTERY_DISABLED */
    return NV_TRUE;
}
#endif

/**
 * Gets the battery Current.
 *
 * @param hDevice [IN] A handle to the EC.
 * @param BatteryInst [IN] The battery type.
 * @param pCurrent [OUT] A pointer to the battery current
 */
NvBool NvOdmBatteryPrivGetCurrent(
       NvOdmBatteryDeviceHandle hDevice,
       NvOdmBatteryInstance BatteryInst,
       NvS32 *pCurrent)
{
#if NVEC_BATTERY_DISABLED
    *pCurrent = NVODM_BATTERY_DATA_UNKNOWN;
#else
    NvError NvStatus = NvError_Success;
    NvEcRequest EcRequest = {0};
    NvEcResponse EcResponse = {0};
    NvEcBatteryGetCurrentResponsePayload BatteryCurrent;
    NvOdmBatteryDevice *pBattContext = NULL;

    NVODMBATTERY_PRINTF(("[ENTER]NvOdmBatteryPrivGetCurrent.\n"));
    *pCurrent = NVODM_BATTERY_DATA_UNKNOWN;
    pBattContext = (NvOdmBatteryDevice *)hDevice;

    /* Fill up request structure */
    EcRequest.PacketType = NvEcPacketType_Request;
    EcRequest.RequestType = NvEcRequestResponseType_Battery;
    EcRequest.RequestSubtype = NvEcBatterySubtype_GetCurrent;
    EcRequest.NumPayloadBytes = 0;
    EcRequest.Payload[0] = 0;

    /* Request to EC */
    NvStatus = NvEcSendRequest(pBattContext->hEc, &EcRequest, &EcResponse,
                               sizeof(EcRequest), sizeof(EcResponse));
    if (NvSuccess != NvStatus)
    {
        NVODMBATTERY_PRINTF(("NvEcSendRequest failed for \
                              NvOdmBatteryPrivGetCurrent\n"));
        return NV_FALSE;
    }

    if(EcResponse.Status == NvEcStatus_Success)
    {
        NvOdmOsMemcpy(&BatteryCurrent, EcResponse.Payload,
                    EcResponse.NumPayloadBytes);
        *pCurrent = BatteryCurrent.PresentCurrent[0];
        *pCurrent |= BatteryCurrent.PresentCurrent[1] << 8;
    }

    NVODMBATTERY_PRINTF(("[EXIT]NvOdmBatteryPrivGetCurrent.\n"));
#endif /* end of NVEC_BATTERY_DISABLED */
    return NV_TRUE;
}

/**
 * Gets the Battery Average Current.
 *
 * @param hDevice [IN] A handle to the EC.
 * @param BatteryInst [IN] The battery type.
 * @param pAverageCurrent [OUT] A pointer to the battery average current
 */
NvBool NvOdmBatteryPrivGetAverageCurrent(
       NvOdmBatteryDeviceHandle hDevice,
       NvOdmBatteryInstance BatteryInst,
       NvS32 *pAverageCurrent)
{
#if NVEC_BATTERY_DISABLED
    *pAverageCurrent = NVODM_BATTERY_DATA_UNKNOWN;
#else
    NvError NvStatus = NvError_Success;
    NvEcRequest EcRequest = {0};
    NvEcResponse EcResponse = {0};
    NvEcBatteryGetAverageCurrentResponsePayload BatteryAverageCurrent;
    NvOdmBatteryDevice *pBattContext = NULL;

    NVODMBATTERY_PRINTF(("[ENTER]NvOdmBatteryPrivGetAverageCurrent.\n"));
    *pAverageCurrent = NVODM_BATTERY_DATA_UNKNOWN;
    pBattContext = (NvOdmBatteryDevice *)hDevice;

    /* Fill up request structure */
    EcRequest.PacketType = NvEcPacketType_Request;
    EcRequest.RequestType = NvEcRequestResponseType_Battery;
    EcRequest.RequestSubtype = NvEcBatterySubtype_GetAverageCurrent;
    EcRequest.NumPayloadBytes = 0;
    EcRequest.Payload[0] = 0;

    /* Request to EC */
    NvStatus = NvEcSendRequest(pBattContext->hEc, &EcRequest, &EcResponse,
                              sizeof(EcRequest), sizeof(EcResponse));
    if (NvSuccess != NvStatus)
    {
        NVODMBATTERY_PRINTF(("NvEcSendRequest failed for \
                              NvOdmBatteryPrivGetAverageCurrent\n"));
        return NV_FALSE;
    }

    if(EcResponse.Status == NvEcStatus_Success)
    {
        NvOdmOsMemcpy(&BatteryAverageCurrent, EcResponse.Payload,
                    EcResponse.NumPayloadBytes);
        *pAverageCurrent = BatteryAverageCurrent.AverageCurrent[0];
        *pAverageCurrent |= BatteryAverageCurrent.AverageCurrent[1] << 8;
    }

    NVODMBATTERY_PRINTF(("[EXIT]NvOdmBatteryPrivGetAverageCurrent.\n"));
#endif /* end of NVEC_BATTERY_DISABLED */
    return NV_TRUE;
}

/**
 * Gets the Battery Average time interval.
 *
 * @param hDevice [IN] A handle to the EC.
 * @param BatteryInst [IN] The battery type.
 * @param pAverageTimeInterval [OUT] A pointer to the battery average
 *        time interval
 */
NvBool NvOdmBatteryPrivGetAverageTimeInterval(
       NvOdmBatteryDeviceHandle hDevice,
       NvOdmBatteryInstance BatteryInst,
       NvU32 *pAverageTimeInterval)
{
#if NVEC_BATTERY_DISABLED
    *pAverageTimeInterval = NVODM_BATTERY_DATA_UNKNOWN;
#else
    NvError NvStatus = NvError_Success;
    NvEcRequest EcRequest = {0};
    NvEcResponse EcResponse = {0};
    NvEcBatteryGetAveragingTimeIntervalResponsePayload \
                            BatteryAverageTimeInterval;
    NvOdmBatteryDevice *pBattContext = NULL;

    NVODMBATTERY_PRINTF(("[ENTER]NvOdmBatteryPrivGetAverageTimeInterval.\n"));
    *pAverageTimeInterval = NVODM_BATTERY_DATA_UNKNOWN;
    pBattContext = (NvOdmBatteryDevice *)hDevice;

    /* Fill up request structure */
    EcRequest.PacketType = NvEcPacketType_Request;
    EcRequest.RequestType = NvEcRequestResponseType_Battery;
    EcRequest.RequestSubtype = NvEcBatterySubtype_GetAveragingTimeInterval;
    EcRequest.NumPayloadBytes = 0;
    EcRequest.Payload[0] = 0;

    /* Request to EC */
    NvStatus = NvEcSendRequest(pBattContext->hEc, &EcRequest, &EcResponse,
                              sizeof(EcRequest), sizeof(EcResponse));
    if (NvSuccess != NvStatus)
    {
        NVODMBATTERY_PRINTF(("NvEcSendRequest failed for \
                              NvOdmBatteryPrivGetAverageTimeInterval\n"));
        return NV_FALSE;
    }

    if(EcResponse.Status == NvEcStatus_Success)
    {
        NvOdmOsMemcpy(&BatteryAverageTimeInterval, EcResponse.Payload,
                    EcResponse.NumPayloadBytes);
        *pAverageTimeInterval = BatteryAverageTimeInterval.TimeInterval[0];
        *pAverageTimeInterval |= \
         BatteryAverageTimeInterval.TimeInterval[1] << 8;
    }

    NVODMBATTERY_PRINTF(("[EXIT]NvOdmBatteryPrivGetAverageTimeInterval.\n"));
#endif /* end of NVEC_BATTERY_DISABLED */
    return NV_TRUE;
}

/**
 * Gets the Battery Temperature.
 *
 * @param hDevice [IN] A handle to the EC.
 * @param BatteryInst [IN] The battery type.
 * @param pBatteryTemp [OUT] A pointer to the battery Temperature
 */
NvBool NvOdmBatteryPrivGetTemperature(
       NvOdmBatteryDeviceHandle hDevice,
       NvOdmBatteryInstance BatteryInst,
       NvU32 *pBatteryTemp)
{
#if NVEC_BATTERY_DISABLED
    *pBatteryTemp = NVODM_BATTERY_DATA_UNKNOWN;
#else
    NvError NvStatus = NvError_Success;
    NvEcRequest EcRequest = {0};
    NvEcResponse EcResponse = {0};
    NvEcBatteryGetTemperatureResponsePayload BatteryTemperature;
    NvOdmBatteryDevice *pBattContext = NULL;

    NVODMBATTERY_PRINTF(("[ENTER]NvOdmBatteryPrivGetTemperature.\n"));
    *pBatteryTemp = NVODM_BATTERY_DATA_UNKNOWN;
    pBattContext = (NvOdmBatteryDevice *)hDevice;

    /* Fill up request structure */
    EcRequest.PacketType = NvEcPacketType_Request;
    EcRequest.RequestType = NvEcRequestResponseType_Battery;
    EcRequest.RequestSubtype = NvEcBatterySubtype_GetTemperature;
    EcRequest.NumPayloadBytes = 0;
    EcRequest.Payload[0] = 0;

    /* Request to EC */
    NvStatus = NvEcSendRequest(pBattContext->hEc, &EcRequest, &EcResponse,
                              sizeof(EcRequest), sizeof(EcResponse));
    if (NvSuccess != NvStatus)
    {
        NVODMBATTERY_PRINTF(("NvEcSendRequest failed for \
                              NvOdmBatteryPrivGetTemperature\n"));
        return NV_FALSE;
    }

    if(EcResponse.Status == NvEcStatus_Success)
    {
        NvOdmOsMemcpy(&BatteryTemperature, EcResponse.Payload,
                    EcResponse.NumPayloadBytes);
        *pBatteryTemp = BatteryTemperature.Temperature[0];
        *pBatteryTemp |= BatteryTemperature.Temperature[1] << 8;
    }

    NVODMBATTERY_PRINTF(("[EXIT]NvOdmBatteryPrivGetTemperature.\n"));
#endif /* end of NVEC_BATTERY_DISABLED */
    return NV_TRUE;
}

/**
 * Gets the Battery Manufacturer.
 *
 * @param hDevice [IN] A handle to the EC.
 * @param BatteryInst [IN] The battery type.
 * @param pBatteryManufacturer [OUT] A pointer to the battery Manufacturer
 */
NvBool NvOdmBatteryGetManufacturer(
       NvOdmBatteryDeviceHandle hDevice,
       NvOdmBatteryInstance BatteryInst,
       NvU8 *pBatteryManufacturer)
{
#if NVEC_BATTERY_DISABLED
    *pBatteryManufacturer = '\0';
#else
    NvError NvStatus = NvError_Success;
    NvEcRequest EcRequest = {0};
    NvEcResponse EcResponse = {0};
    NvEcBatteryGetManufacturerResponsePayload BatteryManufacturer;
    NvOdmBatteryDevice *pBattContext = NULL;

    NVODMBATTERY_PRINTF(("[ENTER]NvOdmBatteryGetManufacturer.\n"));
    *pBatteryManufacturer = '\0';
    pBattContext = (NvOdmBatteryDevice *)hDevice;

    /* Fill up request structure */
    EcRequest.PacketType = NvEcPacketType_Request;
    EcRequest.RequestType = NvEcRequestResponseType_Battery;
    EcRequest.RequestSubtype = NvEcBatterySubtype_GetManufacturer;
    EcRequest.NumPayloadBytes = 0;
    EcRequest.Payload[0] = 0;

    /* Request to EC */
    NvStatus = NvEcSendRequest(pBattContext->hEc, &EcRequest, &EcResponse,
                              sizeof(EcRequest), sizeof(EcResponse));
    if (NvSuccess != NvStatus)
    {
        NVODMBATTERY_PRINTF(("NvEcSendRequest failed for \
                              NvOdmBatteryGetManufacturer\n"));
        return NV_FALSE;
    }

    if(EcResponse.Status == NvEcStatus_Success)
    {
        NvOdmOsMemcpy(&BatteryManufacturer, EcResponse.Payload,
                    EcResponse.NumPayloadBytes);
        NvOdmOsMemcpy(pBatteryManufacturer, BatteryManufacturer.Manufacturer,
                    EcResponse.NumPayloadBytes);
    }

    NVODMBATTERY_PRINTF(("[EXIT]NvOdmBatteryGetManufacturer.\n"));
#endif /* end of NVEC_BATTERY_DISABLED */
    return NV_TRUE;
}

/**
 * Gets the Battery Model.
 *
 * @param hDevice [IN] A handle to the EC.
 * @param BatteryInst [IN] The battery type.
 * @param pBatteryModel [OUT] A pointer to the battery model
 */
NvBool NvOdmBatteryGetModel(
       NvOdmBatteryDeviceHandle hDevice,
       NvOdmBatteryInstance BatteryInst,
       NvU8 *pBatteryModel)
{
#if NVEC_BATTERY_DISABLED
    *pBatteryModel = '\0';
#else
    NvError NvStatus = NvError_Success;
    NvEcRequest EcRequest = {0};
    NvEcResponse EcResponse = {0};
    NvEcBatteryGetModelResponsePayload BatteryModel;
    NvOdmBatteryDevice *pBattContext = NULL;

    NVODMBATTERY_PRINTF(("[ENTER]NvOdmBatteryGetModel.\n"));
    *pBatteryModel = '\0';
    pBattContext = (NvOdmBatteryDevice *)hDevice;

    /* Fill up request structure */
    EcRequest.PacketType = NvEcPacketType_Request;
    EcRequest.RequestType = NvEcRequestResponseType_Battery;
    EcRequest.RequestSubtype = NvEcBatterySubtype_GetModel;
    EcRequest.NumPayloadBytes = 0;
    EcRequest.Payload[0] = 0;

    /* Request to EC */
    NvStatus = NvEcSendRequest(pBattContext->hEc, &EcRequest, &EcResponse,
                              sizeof(EcRequest), sizeof(EcResponse));
    if (NvSuccess != NvStatus)
    {
        NVODMBATTERY_PRINTF(("NvEcSendRequest failed for \
                              NvOdmBatteryGetModel\n"));
        return NV_FALSE;
    }

    if(EcResponse.Status == NvEcStatus_Success)
    {
        NvOdmOsMemcpy(&BatteryModel, EcResponse.Payload,
                    EcResponse.NumPayloadBytes);
        NvOdmOsMemcpy(pBatteryModel, BatteryModel.Model,
                   EcResponse.NumPayloadBytes);
    }

    NVODMBATTERY_PRINTF(("[EXIT]NvOdmBatteryGetModel.\n"));
#endif /* end of NVEC_BATTERY_DISABLED */
    return NV_TRUE;
}
/**
 * Gets the Battery Remaining Capacity.
 *
 * @param hDevice [IN] A handle to the EC.
 * @param BatteryInst [IN] The battery type.
 * @param pRemCapacity [OUT] A pointer to the battery remaining capacity
 */
NvBool NvOdmBatteryPrivGetRemainingCapacity(
       NvOdmBatteryDeviceHandle hDevice,
       NvOdmBatteryInstance BatteryInst,
       NvU32 *pRemCapacity)
{
#if NVEC_BATTERY_DISABLED
    *pRemCapacity = NVODM_BATTERY_DATA_UNKNOWN;
#else
    NvError NvStatus = NvError_Success;
    NvEcRequest EcRequest = {0};
    NvEcResponse EcResponse = {0};
    NvEcBatteryGetCapacityRemainingResponsePayload BatteryRemCap;
    NvOdmBatteryDevice *pBattContext = NULL;

    NVODMBATTERY_PRINTF(("[ENTER]NvOdmBatteryPrivGetRemainingCapacity.\n"));
    *pRemCapacity = NVODM_BATTERY_DATA_UNKNOWN;
    pBattContext = (NvOdmBatteryDevice *)hDevice;

    /* Fill up request structure */
    EcRequest.PacketType = NvEcPacketType_Request;
    EcRequest.RequestType = NvEcRequestResponseType_Battery;
    EcRequest.RequestSubtype = NvEcBatterySubtype_GetCapacityRemaining;
    EcRequest.NumPayloadBytes = 0;
    EcRequest.Payload[0] = 0;

    /* Request to EC */
    NvStatus = NvEcSendRequest(pBattContext->hEc, &EcRequest, &EcResponse,
                              sizeof(EcRequest), sizeof(EcResponse));
    if (NvSuccess != NvStatus)
    {
        NVODMBATTERY_PRINTF(("NvEcSendRequest failed for \
                              NvOdmBatteryPrivGetRemainingCapacity\n"));
        return NV_FALSE;
    }

    if(EcResponse.Status == NvEcStatus_Success)
    {
        NvOdmOsMemcpy(&BatteryRemCap, EcResponse.Payload,
                    EcResponse.NumPayloadBytes);
        *pRemCapacity = BatteryRemCap.CapacityRemaining[0];
        *pRemCapacity |= BatteryRemCap.CapacityRemaining[1] << 8;
    }

    NVODMBATTERY_PRINTF(("[EXIT]NvOdmBatteryPrivGetRemainingCapacity.\n"));
#endif /* end of NVEC_BATTERY_DISABLED */
    return NV_TRUE;
}

/**
 * Gets the Battery Last FullCharge Capacity
 *
 * @param hDevice [IN] A handle to the EC.
 * @param BatteryInst [IN] The battery type.
 * @param pLastFullChargeCapacity [OUT] A pointer to the battery
 *        last fullcharge capacity
 */
NvBool NvOdmBatteryPrivGetLastFullChargeCapacity(
       NvOdmBatteryDeviceHandle hDevice,
       NvOdmBatteryInstance BatteryInst,
       NvU32 *pLastFullChargeCapacity)
{
#if NVEC_BATTERY_DISABLED
    *pLastFullChargeCapacity = NVODM_BATTERY_DATA_UNKNOWN;
#else
    NvError NvStatus = NvError_Success;
    NvEcRequest EcRequest = {0};
    NvEcResponse EcResponse = {0};
    NvEcBatteryGetLastFullChargeCapacityResponsePayload \
        BatteryLastFullChargeCap;
    NvOdmBatteryDevice *pBattContext = NULL;

    NVODMBATTERY_PRINTF(("+NvOdmBatteryPrivGetLastFullChargeCapacity.\n"));
    *pLastFullChargeCapacity = NVODM_BATTERY_DATA_UNKNOWN;
    pBattContext = (NvOdmBatteryDevice *)hDevice;

    /* Fill up request structure */
    EcRequest.PacketType = NvEcPacketType_Request;
    EcRequest.RequestType = NvEcRequestResponseType_Battery;
    EcRequest.RequestSubtype = NvEcBatterySubtype_GetLastFullChargeCapacity;
    EcRequest.NumPayloadBytes = 0;
    EcRequest.Payload[0] = 0;

    /* Request to EC */
    NvStatus = NvEcSendRequest(pBattContext->hEc, &EcRequest, &EcResponse,
                              sizeof(EcRequest), sizeof(EcResponse));
    if (NvSuccess != NvStatus)
    {
        NVODMBATTERY_PRINTF(("NvEcSendRequest failed for \
                        NvOdmBatteryPrivGetLastFullChargeCapacity\n"));
        return NV_FALSE;
    }

    if(EcResponse.Status == NvEcStatus_Success)
    {
        NvOdmOsMemcpy(&BatteryLastFullChargeCap, EcResponse.Payload,
                    EcResponse.NumPayloadBytes);
        *pLastFullChargeCapacity = \
            BatteryLastFullChargeCap.LastFullChargeCapacity[0];
        *pLastFullChargeCapacity |= \
            BatteryLastFullChargeCap.LastFullChargeCapacity[1] << 8;
    }

    NVODMBATTERY_PRINTF(("-NvOdmBatteryPrivGetLastFullChargeCapacity.\n"));
#endif /* end of NVEC_BATTERY_DISABLED */
    return NV_TRUE;
}

/**
 * Gets the Battery critical capacity
 *
 * @param hDevice [IN] A handle to the EC.
 * @param BatteryInst [IN] The battery type.
 * @param pCriticalCapacity [OUT] A pointer to the battery critical capacity
 */
NvBool NvOdmBatteryPrivGetCriticalCapacity(
       NvOdmBatteryDeviceHandle hDevice,
       NvOdmBatteryInstance BatteryInst,
       NvU32 *pCriticalCapacity)
{
#if NVEC_BATTERY_DISABLED
    *pCriticalCapacity = NVODM_BATTERY_DATA_UNKNOWN;
#else
    NvError NvStatus = NvError_Success;
    NvEcRequest EcRequest = {0};
    NvEcResponse EcResponse = {0};
    NvEcBatteryGetCriticalCapacityResponsePayload BatteryCriticalCap;
    NvOdmBatteryDevice *pBattContext = NULL;

    NVODMBATTERY_PRINTF(("[ENTER]NvOdmBatteryPrivGetCriticalCapacity.\n"));
    *pCriticalCapacity = NVODM_BATTERY_DATA_UNKNOWN;
    pBattContext = (NvOdmBatteryDevice *)hDevice;

    /* Fill up request structure */
    EcRequest.PacketType = NvEcPacketType_Request;
    EcRequest.RequestType = NvEcRequestResponseType_Battery;
    EcRequest.RequestSubtype = NvEcBatterySubtype_GetCriticalCapacity;
    EcRequest.NumPayloadBytes = 0;
    EcRequest.Payload[0] = 0;

    /* Request to EC */
    NvStatus = NvEcSendRequest(pBattContext->hEc, &EcRequest, &EcResponse,
                              sizeof(EcRequest), sizeof(EcResponse));
    if (NvSuccess != NvStatus)
    {
        NVODMBATTERY_PRINTF(("NvEcSendRequest failed for \
                        NvOdmBatteryPrivGetCriticalCapacity\n"));
        return NV_FALSE;
    }

    if(EcResponse.Status == NvEcStatus_Success)
    {
        NvOdmOsMemcpy(&BatteryCriticalCap, EcResponse.Payload,
                    EcResponse.NumPayloadBytes);
        *pCriticalCapacity = BatteryCriticalCap.CriticalCapacity[0];
        *pCriticalCapacity |= BatteryCriticalCap.CriticalCapacity[1] << 8;
    }

    NVODMBATTERY_PRINTF(("[EXIT]NvOdmBatteryPrivGetCriticalCapacity.\n"));
#endif /* end of NVEC_BATTERY_DISABLED */
    return NV_TRUE;
}

/**
 * Gets Battery Slot status and Capacity Guage.
 *
 * @param pBattContext [IN] Handle to the Battery Context.
 * @param BatteryInst  [IN] battery type.
 * @param BatterySlotStatus [OUT] gives battery presence and charging status
 * @param BatteryCapacityGuage [OUT] Battery's relative remaining capacity in %
 *
 * @return NV_TRUE if successful, or NV_FALSE otherwise.
 */
NvBool
NvOdmBatteryPrivGetSlotStatusAndCapacityGuage(NvOdmBatteryDevice *pBattContext,
                                              NvOdmBatteryInstance BatteryInst,
                                              NvU8 *BatterySlotStatus,
                                              NvU8 *BatteryCapacityGuage)
{
#if NVEC_BATTERY_DISABLED
    return NV_FALSE;
#else
    NvError NvStatus = NvError_Success;
    NvEcRequest EcRequest = {0};
    NvEcResponse EcResponse = {0};

    NVODMBATTERY_PRINTF(("NvOdmBatteryPrivGetSlotStatusAndCapacityGuage:Enter"));

    /* Fill up request structure */
    EcRequest.PacketType = NvEcPacketType_Request;
    EcRequest.RequestType = NvEcRequestResponseType_Battery;
    EcRequest.RequestSubtype = NvEcBatterySubtype_GetSlotStatus;
    EcRequest.NumPayloadBytes = 0;
    EcRequest.Payload[0] = 0;

    /* Request to EC */
    NvStatus = NvEcSendRequest(pBattContext->hEc, &EcRequest, &EcResponse,
                               sizeof(EcRequest), sizeof(EcResponse));
    if (NvSuccess != NvStatus)
    {
        NVODMBATTERY_PRINTF(("NvEcSendRequest failed for \
                NvOdmBatteryPrivGetSlotStatusAndCapacityGuage\n"));
        return NV_FALSE;
    }

    if(EcResponse.Status == NvEcStatus_Success)
    {
        *BatterySlotStatus = \
            EcResponse.Payload[NVODM_BATTERY_SLOT_STATUS_DATA];
        *BatteryCapacityGuage = \
            EcResponse.Payload[NVODM_BATTERY_CAPACITY_GUAGE_DATA];
    }
    else
    {
        *BatterySlotStatus = 0;
        *BatteryCapacityGuage = 0;
        /*
         * if the response status is unavailable (0x03) that means
         *  HW is unavailable
         * in that case return TRUE to tell that Battery is not present.
         */
        if (EcResponse.Status == NvEcStatus_Unavailable)
            return NV_TRUE;

        return NV_FALSE;
    }

    NVODMBATTERY_PRINTF(("NvOdmBatteryPrivGetSlotStatusAndCapacityGuage:Exit"));
    return NV_TRUE;
#endif /* end of NVEC_BATTERY_DISABLED */
}

/**
 * Gets Battery's Life time
 *
 * @param pBattContext [IN] Handle to the Battery Context.
 * @param BatteryInst [IN] battery type.
 * @param BatteryLifeTime [OUT] Estimated remaining time to empty
 *                        for discharging.
 *                        battery at present rate (in [min])
 *                        Report 0FFFFh if battery is not discharging
 *
 * @return NV_TRUE if successful, or NV_FALSE otherwise.
 */
NvBool NvOdmBatteryPrivGetLifeTime(NvOdmBatteryDevice *pBattContext,
                                   NvOdmBatteryInstance BatteryInst,
                                   NvU32 *BatteryLifeTime)
{
#if NVEC_BATTERY_DISABLED
    return NV_FALSE;
#else
    NvError NvStatus = NvError_Success;
    NvEcRequest EcRequest = {0};
    NvEcResponse EcResponse = {0};
    NvEcBatteryGetTimeRemainingResponsePayload BattTimeRemain;

    /* Fill up request structure */
    EcRequest.PacketType = NvEcPacketType_Request;
    EcRequest.RequestType = NvEcRequestResponseType_Battery;
    EcRequest.RequestSubtype = NvEcBatterySubtype_GetTimeRemaining;
    EcRequest.NumPayloadBytes = 0;
    EcRequest.Payload[0] = 0;

    /* Request to EC */
    NvStatus = NvEcSendRequest(pBattContext->hEc, &EcRequest, &EcResponse,
                               sizeof(EcRequest), sizeof(EcResponse));
    if (NvSuccess != NvStatus)
    {
        NVODMBATTERY_PRINTF(("NvEcSendRequest failed for \
                NvOdmBatteryPrivGetLifeTime\n"));
        return NV_FALSE;
    }

    if(EcResponse.Status == NvEcStatus_Success)
    {
        NvOdmOsMemcpy(&BattTimeRemain, EcResponse.Payload,
            EcResponse.NumPayloadBytes);
        *BatteryLifeTime  = BattTimeRemain.TimeRemaining[0];
        *BatteryLifeTime |= BattTimeRemain.TimeRemaining[1] << 8;
    }
    else
    {
        NVODMBATTERY_PRINTF(("EcResponse.Status failed for \
                    NvOdmBatteryPrivGetLifeTime\n"));
        *BatteryLifeTime = 0;
        return NV_FALSE;
    }

    return NV_TRUE;
#endif /* end of NVEC_BATTERY_DISABLED */
}

/**
 * Gets Battery's Present voltage
 *
 * @param pBattContext [IN] Handle to the Battery Context.
 * @param BatteryInst  [IN] battery type.
 * @param BatteryVoltage [OUT] Battery's present voltage
 *        (16-bit unsigned  value, in [mV])
 * @return NV_TRUE if successful, or NV_FALSE otherwise.
 */
NvBool NvOdmPrivBattGetBatteryVoltage(NvOdmBatteryDevice *pBattContext,
                                      NvOdmBatteryInstance BatteryInst,
                                      NvU32 *BatteryVoltage)
{
#if NVEC_BATTERY_DISABLED
    return NV_FALSE;
#else
    NvError      NvStatus   = NvError_Success;
    NvEcRequest  EcRequest  = {0};
    NvEcResponse EcResponse = {0};
    NvEcBatteryGetVoltageResponsePayload BattVoltage;

    /* Fill up request structure */
    EcRequest.PacketType = NvEcPacketType_Request;
    EcRequest.RequestType = NvEcRequestResponseType_Battery;
    EcRequest.RequestSubtype = NvEcBatterySubtype_GetVoltage;
    EcRequest.NumPayloadBytes = 0;
    EcRequest.Payload[0] = 0;

    /* Request to EC */
    NvStatus = NvEcSendRequest(pBattContext->hEc, &EcRequest, &EcResponse,
                               sizeof(EcRequest), sizeof(EcResponse));
    if (NvSuccess != NvStatus)
    {
        NVODMBATTERY_PRINTF(("NvEcSendRequest failed \
                    NvOdmPrivBattGetBatteryVoltage\n"));
        return NV_FALSE;
    }

    if(EcResponse.Status == NvEcStatus_Success)
    {
        NvOdmOsMemcpy(&BattVoltage, EcResponse.Payload,
                    EcResponse.NumPayloadBytes);
        NvOdmOsMemcpy(BatteryVoltage, BattVoltage.PresentVoltage,
                    sizeof(BattVoltage.PresentVoltage));
    }
    else
    {
        NVODMBATTERY_PRINTF(("EcResponse.Status failed in \
                    NvOdmPrivBattGetBatteryVoltage\n"));
        *BatteryVoltage = 0;
        return NV_FALSE;
    }

    return NV_TRUE;
#endif /* end of NVEC_BATTERY_DISABLED */
}

/**
 *  Openes the battery device handle
 *
 * @param hDevice [OUT] handle to Battery Device.
 *
 * @return NV_TRUE on success.
 */
NvBool NvOdmBatteryDeviceOpen(NvOdmBatteryDeviceHandle *hDevice,
                              NvOdmOsSemaphoreHandle *hOdmSemaphore)
{
    NvOdmBatteryDevice *pBattContext = NULL;
    NvError NvStatus = NvError_Success;
    NvEcEventType EventTypes[] = {NvEcEventType_Battery};
    NvS32    MajorVersion = 0, MinorVersion = 0;
    NvEcRequest  EcRequest = {0};
    NvEcResponse EcResponse = {0};
#if NVODM_LOWBATTERY_GPIO_INT
    NvOsInterruptHandler IntrHandler = {NULL};
#endif
    NvU32 BatteryDesignCap = 0;
    NvU16 RemCapAlarm = 0;

    NVODMBATTERY_PRINTF(("[ENTER] NvOdmBatteryDeviceOpen. \n"));

    /* Allocate the context */
    pBattContext = NvOdmOsAlloc(sizeof(NvOdmBatteryDevice));
    if (!pBattContext)
    {
        NVODMBATTERY_PRINTF(("NvOdmOsAlloc failed to allocate pBattContext."));
        return NV_FALSE;
    }

    NvOdmOsMemset(pBattContext, 0, sizeof(NvOdmBatteryDevice));

    /* Get nvec handle */
    NvStatus = NvEcOpen(&pBattContext->hEc, 0 /* instance */);
    if (NvStatus != NvError_Success)
    {
        NVODMBATTERY_PRINTF(("NvEcOpen failed for NvOdmBatteryDeviceOpen.\n"));
        goto Cleanup;
    }

    /* Get the EC Firmware version */
    NvStatus = NvOdmBatteryPrivEcGetFirmwareVersion(pBattContext,
                                                    &MajorVersion,
                                                    &MinorVersion);
    if (NvStatus != NvError_Success)
    {
        goto Cleanup;
    }

    pBattContext->ECVersion = MinorVersion;
    NVODMBATTERY_PRINTF(("EC Firmware Version = 0x%x\n", MinorVersion));

    if (hOdmSemaphore != NULL && *hOdmSemaphore != NULL)
    {
        pBattContext->hClientBattEventSem = *hOdmSemaphore;

        /* Semaphore to receive Battery events from EC */
        pBattContext->hBattEventSem = NvOdmOsSemaphoreCreate(0);
        if (!pBattContext->hBattEventSem)
        {
            goto Cleanup;
        }

        /* Thread to handle Battery events */
        pBattContext->hBattEventThread = NvOdmOsThreadCreate(
                               (NvOdmOsThreadFunction)NvOdmBatteryEventHandler,
                               (void *)pBattContext);
        if (!pBattContext->hBattEventThread)
        {
            goto Cleanup;
        }

        if (pBattContext->ECVersion >= NVODM_BATTERY_EC_FIRMWARE_VER_R04)
        {
#if NVODM_WAKEUP_FROM_BATTERY_EVENT
           /* Configure the Batter present event as a wakeup */
            EcRequest.PacketType = NvEcPacketType_Request;
            EcRequest.RequestType = NvEcRequestResponseType_Battery;
            EcRequest.RequestSubtype = (NvEcRequestResponseSubtype)
                                  NvEcBatterySubtype_ConfigureWake;
            EcRequest.NumPayloadBytes = 2;
            EcRequest.Payload[0] = NVEC_BATTERY_REPORT_ENABLE_0_ACTION_ENABLE;
            EcRequest.Payload[1] = NVODM_BATTERY_SET_PRESENT_EVENT
#if NVODM_BATTERY_LOW_CAPACITY_ALARM
                | NVODM_BATTERY_SET_REM_CAP_ALARM_EVENT
#endif
                ;

            NvStatus = NvEcSendRequest(pBattContext->hEc, &EcRequest, &EcResponse,
                     sizeof(EcRequest), sizeof(EcResponse));
            if (NvStatus != NvSuccess)
                goto Cleanup;

            if (EcResponse.Status != NvEcStatus_Success)
                goto Cleanup;
#endif

#if NVODM_WAKEUP_FROM_AC_EVENT
           /* Configure the AC present event  as a wakeup */
            EcRequest.PacketType = NvEcPacketType_Request;
            EcRequest.RequestType = NvEcRequestResponseType_System;
            EcRequest.RequestSubtype = (NvEcRequestResponseSubtype)
                                  NvEcSystemSubtype_ConfigureWake;
            EcRequest.NumPayloadBytes = 2;
            EcRequest.Payload[0] = NVEC_SYSTEM_REPORT_ENABLE_0_ACTION_ENABLE;
            EcRequest.Payload[1] = NVEC_SYSTEM_STATE1_0_AC_PRESENT;

            NvStatus = NvEcSendRequest(pBattContext->hEc, &EcRequest, &EcResponse,
                     sizeof(EcRequest), sizeof(EcResponse));
            if (NvStatus != NvSuccess)
                goto Cleanup;

            if (EcResponse.Status != NvEcStatus_Success)
                goto Cleanup;
#endif
            /* Configure the Battery events */
            EcRequest.PacketType = NvEcPacketType_Request;
            EcRequest.RequestType = NvEcRequestResponseType_Battery;
            EcRequest.RequestSubtype = (NvEcRequestResponseSubtype)
                                  NvEcBatterySubtype_ConfigureEventReporting;
            EcRequest.NumPayloadBytes = 2;
            EcRequest.Payload[0] = NVEC_BATTERY_REPORT_ENABLE_0_ACTION_ENABLE;
             /* Bit 0 = Present State event */
             /* Bit 1 = Charging State event */
             /* Bit 2 = Remaining Capacity Alaram event */
            EcRequest.Payload[1] = NVODM_BATTERY_SET_PRESENT_EVENT |
                                   NVODM_BATTERY_SET_CHARGING_EVENT|
                                   NVODM_BATTERY_SET_REM_CAP_ALARM_EVENT;

            NvStatus = NvEcSendRequest(pBattContext->hEc, &EcRequest, &EcResponse,
                     sizeof(EcRequest), sizeof(EcResponse));
            if (NvStatus != NvSuccess)
                goto Cleanup;

            if (EcResponse.Status != NvEcStatus_Success)
                goto Cleanup;

            /* Get the design capacity */
            NvOdmBatteryGetBatteryFullLifeTime(pBattContext, NvOdmBatteryInst_Main,
                                               &BatteryDesignCap);
            /* Set the remaining capacity alarm for 10% of the design capacity */
            RemCapAlarm = (BatteryDesignCap * 10)/100;
            NvOdmBatterySetRemCapacityAlarm(pBattContext, NvOdmBatteryInst_Main, RemCapAlarm);
        }

        /* Register for Battery events */
        NvStatus = NvEcRegisterForEvents(
                   pBattContext->hEc,
                   &pBattContext->hEcEventReg,
                   (NvOsSemaphoreHandle)pBattContext->hBattEventSem,
                   sizeof(EventTypes)/sizeof(NvEcEventType),
                   EventTypes,
                   1,
                   sizeof(NvEcEvent));
        if (NvStatus != NvError_Success)
        {
            goto Cleanup;
        }

#if NVODM_LOWBATTERY_GPIO_INT
        NvStatus = NvRmOpen(&pBattContext->hRm, 0);
        if (NvStatus != NvError_Success)
            goto Cleanup;

        NvStatus = NvRmGpioOpen(pBattContext->hRm, &pBattContext->hGpio);
        if (NvStatus != NvError_Success)
            goto Cleanup;

        pBattContext->pGpioPinInfo = NvOdmQueryGpioPinMap(
                                     NvOdmGpioPinGroup_Battery,
                                     0,
                                     &pBattContext->PinCount);
        if (pBattContext->pGpioPinInfo != NULL)
        {

            IntrHandler = (NvOsInterruptHandler)NvOdmBatteryGpioInterruptHandler;

            if (pBattContext->pGpioPinInfo[0].Port != 0 &&
                pBattContext->pGpioPinInfo[0].Pin != 0)
            {

                NvRmGpioAcquirePinHandle(
                        pBattContext->hGpio,
                        pBattContext->pGpioPinInfo[0].Port,
                        pBattContext->pGpioPinInfo[0].Pin,
                        &pBattContext->hPin);
                if (!pBattContext->hPin)
                {
                    goto Cleanup;
                }

                /* Register to receive GPIO events */
                NvStatus = NvRmGpioInterruptRegister(
                           pBattContext->hGpio,
                           pBattContext->hRm,
                           pBattContext->hPin,
                           IntrHandler,
                           NvRmGpioPinMode_InputInterruptAny,
                           pBattContext,
                           &pBattContext->GpioIntrHandle,
                           0);
                if (NvStatus != NvError_Success)
                {
                    goto Cleanup;
                }

                NvStatus = NvRmGpioInterruptEnable(pBattContext->GpioIntrHandle);
                if (NvStatus != NvError_Success)
                {
                    goto Cleanup;
                }
            }
        }
#endif
    }

    *hDevice = pBattContext;
    NVODMBATTERY_PRINTF(("[EXIT] NvOdmBatteryDeviceOpen.\n"));
    return NV_TRUE;

Cleanup:
    NvOdmBatteryDeviceClose(pBattContext);

    return NV_FALSE;
}

/**
 *  Closes the battery device
 *
 * @param hDevice [IN] handle to Battery Device.
 * 
 * @return void.
 */
void NvOdmBatteryDeviceClose(NvOdmBatteryDeviceHandle hDevice)
{
    NvOdmBatteryDevice *pBattContext = NULL;

    pBattContext = (NvOdmBatteryDevice *)hDevice;

#if NVODM_LOWBATTERY_GPIO_INT
    if (pBattContext->hGpio)
    {
        if (pBattContext->GpioIntrHandle)
        {
            NvRmGpioInterruptUnregister(pBattContext->hGpio, pBattContext->hRm,
                pBattContext->GpioIntrHandle);
            pBattContext->GpioIntrHandle = NULL;
        }

        NvRmGpioReleasePinHandles(pBattContext->hGpio, &pBattContext->hPin,
            pBattContext->PinCount);
        NvRmGpioClose(pBattContext->hGpio);
    }

    if (pBattContext->hRm)
    {
        NvRmClose(pBattContext->hRm);
        pBattContext->hRm = NULL;
    }
#endif

    if (pBattContext->hBattEventSem)
    {
        pBattContext->ExitThread = NV_TRUE;
        NvOdmOsSemaphoreSignal(pBattContext->hBattEventSem);
        NvOdmOsSemaphoreDestroy(pBattContext->hBattEventSem);
        pBattContext->hBattEventSem = NULL;
    }

    if (pBattContext->hBattEventThread)
    {
        NvOdmOsThreadJoin(pBattContext->hBattEventThread);
        pBattContext->hBattEventThread = NULL;
    }

    if (pBattContext->hEc)
    {
        if (pBattContext->hEcEventReg)
        {
            NvEcUnregisterForEvents(pBattContext->hEcEventReg);
            pBattContext->hEcEventReg = NULL;
        }

        NvEcClose(pBattContext->hEc);
        pBattContext->hEc = NULL;
    }

    if (pBattContext)
        NvOdmOsFree(pBattContext);
}

/**
 * Gets the AC line status.
 *
 * @param hDevice [IN] A handle to the EC.
 * @param pStatus [OUT] A pointer to the AC line
 *  status returned by the ODM.
 *
 * @return NV_TRUE if successful, or NV_FALSE otherwise.
 */
NvBool NvOdmBatteryGetAcLineStatus(
       NvOdmBatteryDeviceHandle hDevice,
       NvOdmBatteryAcLineStatus *pStatus)
{
#if NVEC_BATTERY_DISABLED
    *pStatus = NvOdmBatteryAcLine_Online;
#else
    NvError NvStatus = NvError_Success;
    NvEcRequest EcRequest = {0};
    NvEcResponse EcResponse = {0};
    NvU8 ACStatus = 0;
    NvEcSystemGetStateResponsePayload SysQueryState;

    NvOdmBatteryDevice *pBattContext = NULL;

    NVODMBATTERY_PRINTF(("[ENTER] NvOdmBatteryGetAcLineStatus.\n"));

    pBattContext = (NvOdmBatteryDevice *)hDevice;

    /* For R01 EC Firware report AC is online as it has not support for this */
    if (pBattContext->ECVersion == NVODM_BATTERY_EC_FIRMWARE_VER_R01)
    {
        *pStatus = NvOdmBatteryAcLine_Online;
        return NV_TRUE;
    }

    /* Fill up request structure */
    EcRequest.PacketType = NvEcPacketType_Request;
    EcRequest.RequestType = NvEcRequestResponseType_System;
    EcRequest.RequestSubtype = NvEcSystemSubtype_GetStatus;
    EcRequest.NumPayloadBytes = 0;
    EcRequest.Payload[0] = 0;

    /* Request to EC */
    NvStatus = NvEcSendRequest(pBattContext->hEc, &EcRequest, &EcResponse,
                              sizeof(EcRequest), sizeof(EcResponse));
    if (NvSuccess != NvStatus)
    {
        NVODMBATTERY_PRINTF(("NvEcSendRequest failed for \
                              NvOdmBatteryGetAcLineStatus\n"));
        *pStatus = NvOdmBatteryAcLine_Num;
        return NV_FALSE;
    }

    if(EcResponse.Status == NvEcStatus_Success)
    {
        NvOdmOsMemcpy(&SysQueryState, EcResponse.Payload,
                    EcResponse.NumPayloadBytes);

        ACStatus = SysQueryState.State[NVODM_BATTERY_SYSTEM_STATE_DATA1];

        /* AC is present or not */
        if (ACStatus & NVODM_BATTERY_SYSTEM_STATE_AC_PRESENT)
        {
            *pStatus = NvOdmBatteryAcLine_Online;
        }
        else
        {
            *pStatus = NvOdmBatteryAcLine_Offline;
        }
    }

    NVODMBATTERY_PRINTF(("[EXIT] NvOdmBatteryGetAcLineStatus.\n"));
#endif /* end of NVEC_BATTERY_DISABLED */
    return NV_TRUE;
}

/**
 * Gets the battery status.
 *
 * @param hDevice [IN] A handle to the EC.
 * @param BatteryInst [IN] The battery type.
 * @param pStatus [OUT] A pointer to the battery
 *  status returned by the ODM.
 *
 * @return NV_TRUE if successful, or NV_FALSE otherwise.
 */
NvBool NvOdmBatteryGetBatteryStatus(
       NvOdmBatteryDeviceHandle hDevice,
       NvOdmBatteryInstance BatteryInst,
       NvU8 *pStatus)
{
#if NVEC_BATTERY_DISABLED
    *pStatus = NVODM_BATTERY_STATUS_UNKNOWN;
#else
    NvU8  BatterySlotStatus = 0, BatteryCapacityGuage = 0,
          BattPresentState = 0, BattChargingState = 0;
    NvU32 BatteryVoltage = 0; /* in mV */
    NvOdmBatteryDevice *pBattContext = NULL;

    pBattContext = (NvOdmBatteryDevice *)hDevice;

    NVODMBATTERY_PRINTF(("[ENTER] NvOdmBatteryGetBatteryStatus.\n"));

    *pStatus = 0;

    /*
     * For R01 firmware Battery support is not present.
     */
    if (pBattContext->ECVersion == NVODM_BATTERY_EC_FIRMWARE_VER_R01)
    {
        *pStatus = NVODM_BATTERY_STATUS_UNKNOWN;
         NVODMBATTERY_PRINTF(("NvOdmBatteryGetBatteryStatus:EC Firmware R01"));
        return NV_TRUE;
    }

    if (BatteryInst == NvOdmBatteryInst_Main)
    {
        if(NvOdmBatteryPrivGetSlotStatusAndCapacityGuage(pBattContext,
                                                NvOdmBatteryInst_Main,
                                                &BatterySlotStatus,
                                                &BatteryCapacityGuage))
        {
            BattPresentState = BatterySlotStatus & NVODM_BATTERY_PRESENT_IN_SLOT;
            if (BattPresentState == NVODM_BATTERY_PRESENT_IN_SLOT)
            {
                BattChargingState = BatterySlotStatus >> NVODM_BATTERY_CHARGING_STATE_SHIFT;
                BattChargingState &= NVODM_BATTERY_CHARGING_STATE_MASK;
                if (BattChargingState == NVODM_BATTERY_CHARGING_STATE_CHARGING)
                    *pStatus |= NVODM_BATTERY_STATUS_CHARGING;
                else if  (BattChargingState == NVODM_BATTERY_CHARGING_STATE_DISCHARGING)
                    *pStatus |= NVODM_BATTERY_STATUS_DISCHARGING;
                else if  (BattChargingState == NVODM_BATTERY_CHARGING_STATE_IDLE)
                    *pStatus |= NVODM_BATTERY_STATUS_IDLE;
            }
            else
            {
                *pStatus = NVODM_BATTERY_STATUS_NO_BATTERY;
                return NV_TRUE;
            }
        }
        else
        {
            *pStatus = NVODM_BATTERY_STATUS_UNKNOWN;
            return NV_TRUE;
        }

        /* Get the battery voltage to detetmine the Battery Flag */
        if (NvOdmPrivBattGetBatteryVoltage(pBattContext, NvOdmBatteryInst_Main,
                                        &BatteryVoltage))
        {
            if (BatteryVoltage >= NVODM_BATTERY_HIGH_VOLTAGE_MV)
                *pStatus |= NVODM_BATTERY_STATUS_HIGH;
            else if (BatteryVoltage >= NVODM_BATTERY_LOW_VOLTAGE_MV)
                *pStatus |= NVODM_BATTERY_STATUS_LOW;
            else
            {
                *pStatus |= NVODM_BATTERY_STATUS_CRITICAL;
                /*
                 * Additional flag which tells battery is very critical
                 * and needs system shutdown.
                 */
                if (BatteryVoltage <= NVODM_BATTERY_CRITICAL_VOLTAGE_MV)
                    *pStatus |= NVODM_BATTERY_STATUS_VERY_CRITICAL;
            }
        }
        else
        {
            *pStatus = NVODM_BATTERY_STATUS_UNKNOWN;
            return NV_TRUE;
        }
    }
    else
    {
        *pStatus = NVODM_BATTERY_STATUS_UNKNOWN;
    }

    NVODMBATTERY_PRINTF(("[EXIT] NvOdmBatteryGetBatteryStatus.\n"));
#endif /* end of NVEC_BATTERY_DISABLED */
    return NV_TRUE;
}

/**
 * Gets the battery data.
 *
 * @param hDevice [IN] A handle to the EC.
 * @param BatteryInst [IN] The battery type.
 * @param pData [OUT] A pointer to the battery
 *  data returned by the ODM.
 *
 * @return NV_TRUE if successful, or NV_FALSE otherwise.
 */
NvBool NvOdmBatteryGetBatteryData(
       NvOdmBatteryDeviceHandle hDevice,
       NvOdmBatteryInstance BatteryInst,
       NvOdmBatteryData *pData)
{
    NvOdmBatteryData BatteryData;
    NvU32 BatteryVoltage = 0, BatteryLifeTime = 0;
    NvS32 BatteryCurrent = 0, BatteryAvgCurrent = 0;
    NvU32 BatteryAvgTimeInterval = 0;
    NvU32 BatteryTemp = 0;
    NvU32 BattRemCap = 0, BattLastChargeFullCap = 0, BattCriticalCap = 0;
#if BATTERY_EXTRA_INFO
    NvU16 RemCapAlarm = 0;
    NvU8 ConfigurationUnit = NVEC_BATTERY_CONFIGURATION_0_CAPACITY_UNITS_MAH;
    NvU8 BattManufact[NVEC_MAX_RESPONSE_STRING_SIZE] = {0},
         BattModel[NVEC_MAX_RESPONSE_STRING_SIZE] = {0};
#endif
    NvOdmBatteryDevice *pBattContext = NULL;
    NvU8  BatterySlotStatus = 0, BatteryCapacityGuage = 0;

    NVODMBATTERY_PRINTF(("[ENTER] NvOdmBatteryGetBatteryData.\n"));

    pBattContext = (NvOdmBatteryDevice *)hDevice;

    BatteryData.BatteryAverageCurrent  = NVODM_BATTERY_DATA_UNKNOWN;
    BatteryData.BatteryAverageInterval = NVODM_BATTERY_DATA_UNKNOWN;
    BatteryData.BatteryCurrent         = NVODM_BATTERY_DATA_UNKNOWN;
    BatteryData.BatteryLifePercent     = NVODM_BATTERY_DATA_UNKNOWN;
    BatteryData.BatteryLifeTime        = NVODM_BATTERY_DATA_UNKNOWN;
    BatteryData.BatteryMahConsumed     = NVODM_BATTERY_DATA_UNKNOWN;
    BatteryData.BatteryTemperature     = NVODM_BATTERY_DATA_UNKNOWN;
    BatteryData.BatteryVoltage         = NVODM_BATTERY_DATA_UNKNOWN;
    BatteryData.BatteryRemainingCapacity = NVODM_BATTERY_DATA_UNKNOWN;
    BatteryData.BatteryLastChargeFullCapacity = NVODM_BATTERY_DATA_UNKNOWN;
    BatteryData.BatteryCriticalCapacity = NVODM_BATTERY_DATA_UNKNOWN;

    NV_ASSERT(hDevice);
    NV_ASSERT(pData);
    NV_ASSERT(BatteryInst <= NvOdmBatteryInst_Num);

    if (BatteryInst == NvOdmBatteryInst_Main)
    {
        if (NvOdmPrivBattGetBatteryVoltage(pBattContext, NvOdmBatteryInst_Main,
                                           &BatteryVoltage))
        {
            BatteryData.BatteryVoltage = BatteryVoltage;
        }

        if(NvOdmBatteryPrivGetSlotStatusAndCapacityGuage(pBattContext,
           NvOdmBatteryInst_Main, &BatterySlotStatus, &BatteryCapacityGuage))
        {
            BatteryData.BatteryLifePercent = BatteryCapacityGuage;
        }

#if BATTERY_EXTRA_INFO
        /* ConfigurationUnit = NVEC_BATTERY_CONFIGURATION_0_CAPACITY_UNITS_10MWH; */
        NvOdmBatterySetConfiguration(pBattContext, NvOdmBatteryInst_Main, ConfigurationUnit);
        NvOdmBatteryGetConfiguration(pBattContext, NvOdmBatteryInst_Main, &ConfigurationUnit);
#endif

        if(NvOdmBatteryPrivGetLifeTime(pBattContext, NvOdmBatteryInst_Main, &BatteryLifeTime))
        {
            BatteryData.BatteryLifeTime = BatteryLifeTime;
        }

        if(NvOdmBatteryPrivGetCurrent(pBattContext, NvOdmBatteryInst_Main,
           &BatteryCurrent))
        {
            BatteryData.BatteryCurrent = BatteryCurrent;
        }

        if(NvOdmBatteryPrivGetAverageCurrent(pBattContext,
           NvOdmBatteryInst_Main, &BatteryAvgCurrent))
        {
            BatteryData.BatteryAverageCurrent = BatteryAvgCurrent;
        }

        if(NvOdmBatteryPrivGetAverageTimeInterval(pBattContext,
           NvOdmBatteryInst_Main, &BatteryAvgTimeInterval))
        {
            BatteryData.BatteryAverageInterval = BatteryAvgTimeInterval;
        }

        if(NvOdmBatteryPrivGetTemperature(pBattContext, NvOdmBatteryInst_Main,
           &BatteryTemp))
        {
            BatteryData.BatteryTemperature = BatteryTemp;
        }

        if(NvOdmBatteryPrivGetRemainingCapacity(pBattContext,
           NvOdmBatteryInst_Main, &BattRemCap))
        {
            BatteryData.BatteryRemainingCapacity = BattRemCap;
        }

        if(NvOdmBatteryPrivGetLastFullChargeCapacity(pBattContext,
           NvOdmBatteryInst_Main, &BattLastChargeFullCap))
        {
            BatteryData.BatteryLastChargeFullCapacity = BattLastChargeFullCap;
        }

        if(NvOdmBatteryPrivGetCriticalCapacity(pBattContext,
           NvOdmBatteryInst_Main, &BattCriticalCap))
        {
            BatteryData.BatteryCriticalCapacity = BattCriticalCap;
        }

#if BATTERY_EXTRA_INFO
        /* RemCapAlarm = 0x0101;*/ /* Some random value */
        NvOdmBatterySetRemCapacityAlarm(pBattContext, NvOdmBatteryInst_Main, RemCapAlarm);
        RemCapAlarm = 0;
        NvOdmBatteryGetRemCapacityAlarm(pBattContext, NvOdmBatteryInst_Main, &RemCapAlarm);

        NvOdmBatteryGetManufacturer(pBattContext, NvOdmBatteryInst_Main, BattManufact);
        NvOdmBatteryGetModel(pBattContext, NvOdmBatteryInst_Main, BattModel);
#endif

        *pData = BatteryData;
    }
    else
    {
        *pData = BatteryData;
    }

    NVODMBATTERY_PRINTF(("[EXIT] NvOdmBatteryGetBatteryData.\n"));
#if NVEC_BATTERY_DISABLED
    *pData = BatteryData;
#endif
    return NV_TRUE;
}

/**
 * Gets the battery full life time.
 *
 * @param hDevice [IN] A handle to the EC.
 * @param BatteryInst [IN] The battery type.
 * @param pLifeTime [OUT] A pointer to the battery
 *  full life time returned by the ODM.
 *
 */
void NvOdmBatteryGetBatteryFullLifeTime(
     NvOdmBatteryDeviceHandle hDevice,
     NvOdmBatteryInstance BatteryInst,
     NvU32 *pLifeTime)
{
#if NVEC_BATTERY_DISABLED
    *pLifeTime = NVODM_BATTERY_DATA_UNKNOWN;
#else
    NvError NvStatus = NvError_Success;
    NvEcRequest EcRequest = {0};
    NvEcResponse EcResponse = {0};
    NvEcBatteryGetDesignCapacityResponsePayload BatteryCapacity;
    NvOdmBatteryDevice *pBattContext = NULL;

    NVODMBATTERY_PRINTF(("[ENTER]NvOdmBatteryGetBatteryFullLifeTime.\n"));
    *pLifeTime = NVODM_BATTERY_DATA_UNKNOWN;
    pBattContext = (NvOdmBatteryDevice *)hDevice;

    /* Fill up request structure */
    EcRequest.PacketType = NvEcPacketType_Request;
    EcRequest.RequestType = NvEcRequestResponseType_Battery;
    EcRequest.RequestSubtype = NvEcBatterySubtype_GetDesignCapacity;
    EcRequest.NumPayloadBytes = 0;
    EcRequest.Payload[0] = 0;

    /* Request to EC */
    NvStatus = NvEcSendRequest(pBattContext->hEc, &EcRequest, &EcResponse,
                              sizeof(EcRequest), sizeof(EcResponse));
    if (NvSuccess != NvStatus)
    {
        NVODMBATTERY_PRINTF(("NvEcSendRequest failed for \
                        NvOdmBatteryGetBatteryFullLifeTime\n"));
    }
    else
    {
        if(EcResponse.Status == NvEcStatus_Success)
        {
            NvOdmOsMemcpy(&BatteryCapacity, EcResponse.Payload,
                        EcResponse.NumPayloadBytes);
            *pLifeTime = BatteryCapacity.DesignCapacity[0];
            *pLifeTime |= BatteryCapacity.DesignCapacity[1] << 8;
        }
    }

    NVODMBATTERY_PRINTF(("[EXIT]NvOdmBatteryGetBatteryFullLifeTime.\n"));
#endif /* end of NVEC_BATTERY_DISABLED */
}


/**
 * Gets the battery chemistry.
 *
 * @param hDevice [IN] A handle to the EC.
 * @param BatteryInst [IN] The battery type.
 * @param pChemistry [OUT] A pointer to the battery
 *  chemistry returned by the ODM.
 *
 */
void NvOdmBatteryGetBatteryChemistry(
     NvOdmBatteryDeviceHandle hDevice,
     NvOdmBatteryInstance BatteryInst,
     NvOdmBatteryChemistry *pChemistry)
{
#if NVEC_BATTERY_DISABLED
    *pChemistry = NVODM_BATTERY_DATA_UNKNOWN;
#else
    NvError NvStatus = NvError_Success;
    NvEcRequest EcRequest = {0};
    NvEcResponse EcResponse = {0};
    NvEcBatteryGetTypeResponsePayload BatteryType;

    NvOdmBatteryDevice *pBattContext = NULL;

    NVODMBATTERY_PRINTF(("[ENTER] NvOdmBatteryGetBatteryChemistry.\n"));
    *pChemistry = NVODM_BATTERY_DATA_UNKNOWN;

    pBattContext = (NvOdmBatteryDevice *)hDevice;

    /* Fill up request structure */
    EcRequest.PacketType = NvEcPacketType_Request;
    EcRequest.RequestType = NvEcRequestResponseType_Battery;
    EcRequest.RequestSubtype = NvEcBatterySubtype_GetType;
    EcRequest.NumPayloadBytes = 0;
    EcRequest.Payload[0] = 0;

    /* Request to EC */
    NvStatus = NvEcSendRequest(pBattContext->hEc, &EcRequest, &EcResponse,
                              sizeof(EcRequest), sizeof(EcResponse));
    if (NvSuccess != NvStatus)
    {
        NVODMBATTERY_PRINTF(("NvEcSendRequest failed for \
                              NvOdmBatteryGetBatteryChemistry\n"));
}
    else
    {
        if(EcResponse.Status == NvEcStatus_Success)
        {
            NvOdmOsMemcpy(&BatteryType, EcResponse.Payload,
                        EcResponse.NumPayloadBytes);

            if(!NvOsStrncmp(BatteryType.Type, "LION",
                            EcResponse.NumPayloadBytes))
                *pChemistry = NvOdmBatteryChemistry_LION;
            else if(!NvOsStrncmp(BatteryType.Type, "Alkaline",
                                 EcResponse.NumPayloadBytes))
                *pChemistry = NvOdmBatteryChemistry_Alkaline;
            else if(!NvOsStrncmp(BatteryType.Type, "NICD",
                                 EcResponse.NumPayloadBytes))
                *pChemistry = NvOdmBatteryChemistry_NICD;
            else if(!NvOsStrncmp(BatteryType.Type, "NIMH",
                                 EcResponse.NumPayloadBytes))
                *pChemistry = NvOdmBatteryChemistry_NIMH;
            else if(!NvOsStrncmp(BatteryType.Type, "LIPOLY",
                                 EcResponse.NumPayloadBytes))
                *pChemistry = NvOdmBatteryChemistry_LIPOLY;
            else if(!NvOsStrncmp(BatteryType.Type, "XINCAIR",
                                 EcResponse.NumPayloadBytes))
                *pChemistry = NvOdmBatteryChemistry_XINCAIR;
        }
    }

    NVODMBATTERY_PRINTF(("[EXIT] NvOdmBatteryGetBatteryChemistry.\n"));
#endif /* end of NVEC_BATTERY_DISABLED */
}
