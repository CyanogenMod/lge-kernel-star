/*
 * arch/arm/mach-tegra/odm_kit/query/ventana/nvodm_query_discovery.c
 *
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
#include "nvodm_query_gpio.h"
#include "nvodm_modules.h"
#include "nvodm_query_discovery.h"
#include "nvodm_query.h"
#include "nvrm_drf.h"

#include "subboards/nvodm_query_discovery_pm275_addresses.h"

static NvOdmPeripheralConnectivity s_Peripherals_Default[] =
{
#include "subboards/nvodm_query_discovery_pm275_peripherals.h"
};

#define PROCESSOR_BOARD_ID_I2C_ADDRESS ((0x56)<<1)

#define NVODM_QUERY_I2C_CLOCK_SPEED      100    // kHz
#define NVODM_QUERY_BOARD_HEADER_START   0x04   // Offset to Part Number in EERPOM
#define NVODM_QUERY_I2C_EEPROM_ADDRESS   0xA0   // I2C device base address for EEPROM (7'h50)

#define EEPROM_ID_PM275 0x024B
#define VENTANA_REV_C_ACCEL NV_ODM_GUID('k','x','t','9','-','0','0','0')
#define VENTANA_REV_A_ACCEL NV_ODM_GUID('k','x','t','9','-','0','9','0')

static NvOdmI2cStatus NvOdmPeripheralI2cRead8(
    NvOdmServicesI2cHandle hOdmI2c,
    NvU8 I2cAddr,
    NvU8 Offset,
    NvU8 *pData)
{
    NvU8 ReadBuffer[1];
    NvOdmI2cStatus Error;
    NvOdmI2cTransactionInfo TransactionInfo;

    ReadBuffer[0] = Offset;

    TransactionInfo.Address = I2cAddr;
    TransactionInfo.Buf = ReadBuffer;
    TransactionInfo.Flags = NVODM_I2C_IS_WRITE;
    TransactionInfo.NumBytes = 1;

    Error = NvOdmI2cTransaction(
        hOdmI2c, &TransactionInfo, 1, NVODM_QUERY_I2C_CLOCK_SPEED, NV_WAIT_INFINITE);
    if (Error != NvOdmI2cStatus_Success)
    {
        return Error;
    }

    NvOdmOsMemset(ReadBuffer, 0, sizeof(ReadBuffer));

    TransactionInfo.Address = (I2cAddr | 0x1);
    TransactionInfo.Buf = ReadBuffer;
    TransactionInfo.Flags = 0;
    TransactionInfo.NumBytes = 1;

    // Read data from ROM at the specified offset
    Error = NvOdmI2cTransaction(
        hOdmI2c, &TransactionInfo, 1, NVODM_QUERY_I2C_CLOCK_SPEED, NV_WAIT_INFINITE);
    if (Error != NvOdmI2cStatus_Success)
    {
        return Error;
    }
    *pData = ReadBuffer[0];
    return Error;
}

static NvBool NvOdmPeripheralReadPartNumber(
    NvOdmServicesI2cHandle hOdmI2c,
    NvU8 EepromInst,
    NvOdmBoardInfo *pBoardInfo)
{
    NvOdmI2cStatus Error;
    NvU32 i;
    NvU8 I2cAddr, Offset;
    NvU8 ReadBuffer[sizeof(NvOdmBoardInfo)];

    NvOdmOsMemset(ReadBuffer, 0, sizeof(ReadBuffer));

    // EepromInst*2, since 7-bit addressing
    I2cAddr = NVODM_QUERY_I2C_EEPROM_ADDRESS + (EepromInst << 1);

    /**
     * Offset to the board number entry in EEPROM.
     */
    Offset = NVODM_QUERY_BOARD_HEADER_START;

    for (i=0; i<sizeof(NvOdmBoardInfo); i++)
    {
        Error = NvOdmPeripheralI2cRead8(
            hOdmI2c, I2cAddr, Offset+i, (NvU8 *)&ReadBuffer[i]);
        if (Error != NvOdmI2cStatus_Success)
        {
            return NV_FALSE;
        }
    }
    NvOdmOsMemcpy(pBoardInfo, &ReadBuffer[0], sizeof(NvOdmBoardInfo));
    return NV_TRUE;
}

NvBool NvOdmPeripheralGetBoardInfo(
    NvU16 BoardId,
    NvOdmBoardInfo *pBoardInfo)
{
    NvBool RetVal = NV_FALSE;
    NvOdmServicesI2cHandle hOdmI2c = NULL;
    NvU8 EepromInst=0;
    static NvOdmBoardInfo BoardModuleTable;
    static NvBool s_ReadBoardInfoDone = NV_FALSE;

    if (!s_ReadBoardInfoDone)
        hOdmI2c = NvOdmI2cOpen(NvOdmIoModule_I2c_Pmu, 0);

    if (!s_ReadBoardInfoDone)
    {
        if (!hOdmI2c)
        {
            // Exit
            pBoardInfo = NULL;
            return NV_FALSE;
        }
        RetVal = NvOdmPeripheralReadPartNumber(
                   hOdmI2c, EepromInst, &BoardModuleTable);
        if (RetVal)
            s_ReadBoardInfoDone = NV_TRUE;
    }
    if (hOdmI2c)
        NvOdmI2cClose(hOdmI2c);

        // Linear search for given BoardId; if found, return entry
    if (BoardModuleTable.BoardID == BoardId)
    {
        // Match found
        pBoardInfo->BoardID  = BoardModuleTable.BoardID;
        pBoardInfo->SKU      = BoardModuleTable.SKU;
        pBoardInfo->Fab      = BoardModuleTable.Fab;
        pBoardInfo->Revision = BoardModuleTable.Revision;
        pBoardInfo->MinorRevision = BoardModuleTable.MinorRevision;
        return NV_TRUE;
    }

    // Match not found
    pBoardInfo = NULL;
    return NV_FALSE;
}

static const NvOdmPeripheralConnectivity *NvApGetAllPeripherals(NvU32 *pNum)
{
    *pNum = NV_ARRAY_SIZE(s_Peripherals_Default);
    return s_Peripherals_Default;
}

static NvU32 ReadBoardFabVersion(void)
{
    NvOdmBoardInfo BoardInfo;
    NvBool IsBoardPresent;
    static NvBool s_IsFabRead = NV_FALSE;
    static NvU32 BoardFabNumber = -1;

    if (!s_IsFabRead)
    {
        IsBoardPresent = NvOdmPeripheralGetBoardInfo(EEPROM_ID_PM275, &BoardInfo);
        if (!IsBoardPresent)
        {
            return BoardFabNumber;
        }
        s_IsFabRead = NV_TRUE;
        BoardFabNumber = BoardInfo.Fab;
    }
    return BoardFabNumber;
}

// This implements a simple linear search across the entire set of currently-
// connected peripherals to find the set of GUIDs that Match the search
// criteria.  More clever implementations are possible, but given the
// relatively small search space (max dozens of peripherals) and the relative
// infrequency of enumerating peripherals, this is the easiest implementation.
const NvOdmPeripheralConnectivity *NvOdmPeripheralGetGuid(NvU64 SearchGuid)
{
    const NvOdmPeripheralConnectivity *pAllPeripherals;
    NvU32 NumPeripherals;
    NvU32 i;
    NvU32 BoardFabNum;

    pAllPeripherals = NvApGetAllPeripherals(&NumPeripherals);

    if (!pAllPeripherals || !NumPeripherals)
        return NULL;
    BoardFabNum = ReadBoardFabVersion();
    if ((SearchGuid == VENTANA_REV_A_ACCEL) && (BoardFabNum != 0))
        return NULL;

    if ((SearchGuid == VENTANA_REV_C_ACCEL) && (BoardFabNum != 2))
        return NULL;

    for (i = 0; i < NumPeripherals; i++)
    {
        if (SearchGuid == pAllPeripherals[i].Guid)
        {
            return &pAllPeripherals[i];
        }
    }
    return NULL;
}

static NvBool IsBusMatch(
    const NvOdmPeripheralConnectivity *pPeriph,
    const NvOdmPeripheralSearch *pSearchAttrs,
    const NvU32 *pSearchVals,
    NvU32 offset,
    NvU32 NumAttrs)
{
    NvU32 i, j;
    NvBool IsMatch = NV_FALSE;

    for (i=0; i<pPeriph->NumAddress; i++)
    {
        j = offset;
        do
        {
            switch (pSearchAttrs[j])
            {
            case NvOdmPeripheralSearch_IoModule:
                IsMatch = (pSearchVals[j] ==
                           (NvU32)(pPeriph->AddressList[i].Interface));
                break;
            case NvOdmPeripheralSearch_Address:
                IsMatch = (pSearchVals[j] == pPeriph->AddressList[i].Address);
                break;
            case NvOdmPeripheralSearch_Instance:
                IsMatch = (pSearchVals[j] == pPeriph->AddressList[i].Instance);
                break;
            case NvOdmPeripheralSearch_PeripheralClass:
            default:
                NV_ASSERT(!"Bad Query!");
                break;
            }
            j++;
        } while (IsMatch && j<NumAttrs &&
                 pSearchAttrs[j]!=NvOdmPeripheralSearch_IoModule);

        if (IsMatch)
        {
            return NV_TRUE;
        }
    }
    return NV_FALSE;
}

static NvBool IsPeripheralMatch(
    const NvOdmPeripheralConnectivity *pPeriph,
    const NvOdmPeripheralSearch *pSearchAttrs,
    const NvU32 *pSearchVals,
    NvU32 NumAttrs)
{
    NvU32 i;
    NvBool IsMatch = NV_TRUE;

    for (i=0; i<NumAttrs && IsMatch; i++)
    {
        switch (pSearchAttrs[i])
        {
        case NvOdmPeripheralSearch_PeripheralClass:
            IsMatch = (pSearchVals[i] == (NvU32)(pPeriph->Class));
            break;
        case NvOdmPeripheralSearch_IoModule:
            IsMatch = IsBusMatch(pPeriph, pSearchAttrs, pSearchVals, i, NumAttrs);
            break;
        case NvOdmPeripheralSearch_Address:
        case NvOdmPeripheralSearch_Instance:
            // In correctly-formed searches, these parameters will be parsed by
            // IsBusMatch, so we ignore them here.
            break;
        default:
            NV_ASSERT(!"Bad search attribute!");
            break;
        }
    }
    return IsMatch;
}

NvU32 NvOdmPeripheralEnumerate(
    const NvOdmPeripheralSearch *pSearchAttrs,
    const NvU32 *pSearchVals,
    NvU32 NumAttrs,
    NvU64 *pGuidList,
    NvU32 NumGuids)
{
    const NvOdmPeripheralConnectivity *pAllPeripherals;
    NvU32 NumPeripherals;
    NvU32 Matches;
    NvU32 i;

    pAllPeripherals = NvApGetAllPeripherals(&NumPeripherals);

    if (!pAllPeripherals || !NumPeripherals)
    {
        return 0;
    }

    if (!pSearchAttrs || !pSearchVals)
    {
        NumAttrs = 0;
    }
    for (i=0, Matches=0; i<NumPeripherals &&
             (Matches < NumGuids || !pGuidList); i++)
    {
        if ( !NumAttrs || IsPeripheralMatch(&pAllPeripherals[i],
                                            pSearchAttrs, pSearchVals,
                                            NumAttrs) )
        {
            if (pGuidList)
                pGuidList[Matches] = pAllPeripherals[i].Guid;
            Matches++;
        }
    }
    return Matches;
}

