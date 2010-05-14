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

/**
 * @file
 * <b>NVIDIA APX ODM Kit::
 *         Implementation of the ODM Peripheral Discovery API</b>
 *
 * @b Description: The peripheral connectivity database implementation.
 */

#include "nvcommon.h"
#include "nvodm_query_gpio.h"
#include "nvodm_modules.h"
#include "nvodm_query_discovery.h"
#include "nvodm_keylist_reserved.h"
#include "tegra_devkit_custopt.h"
#include "nvodm_query.h"
#include "nvrm_drf.h"

#include "subboards/nvodm_query_discovery_e1162_addresses.h"

static NvOdmPeripheralConnectivity s_Peripherals_Default[] =
{
#include "subboards/nvodm_query_discovery_e1162_peripherals.h"
};

#define NVODM_QUERY_BOARD_ID_UNKNOWN    0xFFFF

#define NVODM_QUERY_MAX_PERIPHERALS     0x400
#define NVODM_QUERY_MAX_IO_ADDRESSES    0x400

#define NVODM_QUERY_MAX_EEPROMS         8   // Maximum number of EEPROMs per bus segment

#define NVODM_QUERY_ERASED_EEPROM_VALUE 0xFF

#define PROCESSOR_BOARD_ID_I2C_ADDRESS ((0x56)<<1)
#define PROCESSOR_BOARD_ID_I2C_SEGMENT (0x00)

// The following are used to store entries read from EEPROMs at runtime.
static NvOdmPeripheralConnectivity s_Peripherals[NVODM_QUERY_MAX_PERIPHERALS];
static NvOdmIoAddress s_Peripheral_IoAddresses[NVODM_QUERY_MAX_IO_ADDRESSES];
static NvOdmBoardInfo s_BoardModuleTable[NVODM_QUERY_MAX_EEPROMS];

#define NVODM_QUERY_I2C_CLOCK_SPEED      100    // kHz

#define NVODM_QUERY_ENTRY_HEADER_SIZE    0x30   // Size of EERPOM "Entry Header"
#define NVODM_QUERY_BOARD_HEADER_START   0x04   // Offset to Part Number in EERPOM

#define NVODM_QUERY_I2C_EEPROM_ADDRESS   0xA0   // I2C device base address for EEPROM (7'h50)

#define NVODM_QUERY_PERIPH_CONN_STRUCT_COMPRESSED   10  // See EEPROM_format.txt
#define NVODM_PERIPH_IO_ADDR_STRUCT_SZ_COMPRESSED   2   // See EEPROM_format.txt


static NvOdmI2cStatus
NvOdmPeripheralI2cRead8(
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

static NvBool
NvOdmPeripheralReadNumPeripherals(
    NvOdmServicesI2cHandle hOdmI2c,
    NvU8 EepromInst,
    NvU8 *pNumModulePeripherals)
{
    NvOdmI2cStatus Error;    
    NvU8 I2cAddr, Offset;

    // EepromInst*2, since 7-bit addressing
    I2cAddr = NVODM_QUERY_I2C_EEPROM_ADDRESS + (EepromInst << 1);

    /**
     * Offset to numPeripherals in NvOdmPeripheralConnectivity Structure.
     * It's the first parameter after the "Entry Header."
     */
    Offset = NVODM_QUERY_ENTRY_HEADER_SIZE;

    Error = NvOdmPeripheralI2cRead8(
        hOdmI2c, I2cAddr, Offset, pNumModulePeripherals);
    if (Error != NvOdmI2cStatus_Success)
    {
        return NV_FALSE;
    }
    return NV_TRUE;
}

static NvBool
NvOdmPeripheralReadPeripheral(
    NvOdmServicesI2cHandle hOdmI2c,
    NvU8 EepromInst,
    NvU8 Peripheral,
    NvU64 *pGuid,
    NvU8 *pEepromAddressListOffset,
    NvU32 *pNumAddress,
    NvOdmPeripheralClass *pClass)
{
    NvOdmI2cStatus Error;    
    NvU32 i;
    NvU8 ConnMemberIndex=0; // Offset to members in NvOdmPeripheralConnectivity
    NvU8 I2cAddr, Offset;
    NvU8 ReadBuffer[NVODM_QUERY_PERIPH_CONN_STRUCT_COMPRESSED];
    NvU8 NumAddrAndClass;

    // EepromInst*2, since 7-bit addressing
    I2cAddr = NVODM_QUERY_I2C_EEPROM_ADDRESS + (EepromInst << 1);

    /**
     * Calculate offset to pGuid in NvOdmPeripheralConnectivity Structure
     * 
     *  Offset = sizeof(eeprom Entry Header) + 
     *           sizeof(NvOdmPeripheralConnectivity)*peripheral + 
     *           pGuid offset    <-- First field, so this is 0
     */
    Offset = NVODM_QUERY_ENTRY_HEADER_SIZE + 
        sizeof(NvOdmPeripheralConnectivity)*Peripheral;

    for (i=0; i<NVODM_QUERY_PERIPH_CONN_STRUCT_COMPRESSED; i++)
    {
        Error = NvOdmPeripheralI2cRead8(
            hOdmI2c, I2cAddr, Offset, (NvU8 *)&ReadBuffer[i]);
        if (Error != NvOdmI2cStatus_Success)
        {
            return NV_FALSE;
        }
    }
    // Save pGuid entry
    NvOdmOsMemcpy(pGuid, &ReadBuffer[0], sizeof(NvU64));

    // Save EEPROM offset
    ConnMemberIndex += sizeof(NvU64); // Increment to next member
    *pEepromAddressListOffset = ReadBuffer[ConnMemberIndex];

    // Save pNumAddress & Class
    ConnMemberIndex += sizeof(NvU8); // Increment to next member
    NumAddrAndClass = ReadBuffer[ConnMemberIndex];
    *pNumAddress = (NvU32)((NumAddrAndClass >> 3) & 0x0000001F);
    *pClass = (NvOdmPeripheralClass)(NumAddrAndClass & 0x00000007);

    return NV_TRUE;
}

static NvBool
NvOdmPeripheralReadIoAddressData(
    NvOdmServicesI2cHandle hOdmI2c,
    NvU8 EepromInst,
    NvU8 EepromAddressListOffset,
    NvOdmIoAddress *pIoAddressEntry)
{
    NvOdmI2cStatus Error;    
    NvU32 i;
    NvU8 I2cAddr;
    NvU8 ReadBuffer[NVODM_PERIPH_IO_ADDR_STRUCT_SZ_COMPRESSED];
    NvU16 CompressedIoAddressEntry;

    // EepromInst*2, since 7-bit addressing
    I2cAddr = NVODM_QUERY_I2C_EEPROM_ADDRESS + (EepromInst << 1);

    for (i=0; i<NVODM_PERIPH_IO_ADDR_STRUCT_SZ_COMPRESSED; i++)
    {
        Error = NvOdmPeripheralI2cRead8(
            hOdmI2c, I2cAddr, EepromAddressListOffset, (NvU8 *)&ReadBuffer[i]);
        if (Error != NvOdmI2cStatus_Success)
        {
            return NV_FALSE;
        }
    }
    // Save pIoAddressEntry: interface, instance, address
    CompressedIoAddressEntry = ((((NvU16)ReadBuffer[1]) << 8) & 0xFF00) | ReadBuffer[0];

    pIoAddressEntry->Interface = (NvOdmIoModule)((CompressedIoAddressEntry >> 11) & 0x1F);

    if (pIoAddressEntry->Interface != NvOdmIoModule_Gpio)
    {
        pIoAddressEntry->Instance = (NvU32)((CompressedIoAddressEntry >> 7) & 0xF);
        pIoAddressEntry->Address = (NvU32)(CompressedIoAddressEntry & 0x7F);
    }
    else
    {
        pIoAddressEntry->Address = (NvU32)((CompressedIoAddressEntry >> 6) & 0x3F);
        pIoAddressEntry->Instance = (NvU32)(CompressedIoAddressEntry & 0x3F);
    }

    return NV_TRUE;
}

static NvBool NvOdmPeripheralGetEntries(NvU32 *pNum)
{
    NvBool RetVal;
    NvBool IsMatch = NV_FALSE;
    NvOdmServicesI2cHandle hOdmI2c = NULL;
    NvU8 EepromInst;

    // Peripheral counters
    NvU8 NumPeripherals = 0;
    NvU8 CurrentPeripheral = 0;
    NvU32 TotalPeripherals = 0;
    NvU32 StaticPeripherals;

    NvU32 CurrentIoAddressNum = 0;
    NvU32 TotalIoAddressEntries = 0;

    NvU32 i,j;
    NvU8 EepromAddressListOffset;

    if (!pNum) {
        return NV_FALSE;
    }

    // Auto-detect -- Read I2C-EEPROMs on each sub-board

    hOdmI2c = NvOdmI2cOpen(NvOdmIoModule_I2c_Pmu, 0);
    if (!hOdmI2c)
        return NV_FALSE;

    for (EepromInst=0; EepromInst < NVODM_QUERY_MAX_EEPROMS; EepromInst++)
    {
        RetVal = NvOdmPeripheralReadNumPeripherals(
            hOdmI2c, EepromInst, &NumPeripherals);

        if ( (RetVal == NV_TRUE) &&
             (NumPeripherals != NVODM_QUERY_ERASED_EEPROM_VALUE) )
        {
            if (NumPeripherals > 0)
            {
                if ((NumPeripherals + TotalPeripherals) > NVODM_QUERY_MAX_PERIPHERALS)
                {
                    NV_ASSERT( !"ERROR: s_Peripherals[] is too small to accommodate entries!" );

                    // Break out of loop and use static/default configuration
                    break;
                }

                for (CurrentPeripheral=0; \
                      CurrentPeripheral < NumPeripherals; \
                      CurrentPeripheral++)
                {
                    RetVal = NvOdmPeripheralReadPeripheral(
                        hOdmI2c,
                        EepromInst,
                        CurrentPeripheral,
                        &s_Peripherals[TotalPeripherals+CurrentPeripheral].Guid,
                        &EepromAddressListOffset,
                        &s_Peripherals[TotalPeripherals+CurrentPeripheral].NumAddress,
                        &s_Peripherals[TotalPeripherals+CurrentPeripheral].Class);

                    if (RetVal == NV_FALSE)
                    {
                        NV_ASSERT(!"Unable to read EEPROM peripheral entry!");
                        break;  // Go to next EEPROM
                    }
                    else // Process peripheral entry
                    {
                        /**
                         * Process NvOdmIoAddress arrays --
                         * 
                         * These are separate data structures.  The addressList value
                         * read from the EEPROM (EepromAddressListOffset) represents
                         * an offset address within the I2C-EEPROM.  This offset value
                         * identifies where to find the first instance of the
                         * NvOdmIoAddress data.
                         * 
                         * The total number of NvOdmIoAddress entries is identified
                         * by the numAddress variable following the addressList entry
                         * in EEPROM.
                         * 
                         * Once the offset and number of entries are determined (from
                         * above NvOdmPeripheralReadPeripheral function call), a loop
                         * fills in entries within the fixed storage area
                         * (e.g., s_Peripheral_IoAddresses) and the actual
                         * addressList pointer is assigned a value that corresponds
                         * to the first entry of the current class within this array.
                         * In other words, there might be prior entries in the
                         * s_Peripheral_IoAddresses array, but the first entry
                         * corresponding to the current class might be the third
                         * element in this array.  Therefore, the actual addressList
                         * pointer for the current NvOdmPeripheralConnectivity.addressList
                         * parameter would be the address of the third entry, which is
                         * &s_Peripheral_IoAddresses[2] in this example.
                         */

                        // Read all of the entries and save them in s_Peripheral_IoAddresses
                        for (CurrentIoAddressNum=0; \
                              CurrentIoAddressNum < s_Peripherals[TotalPeripherals+CurrentPeripheral].NumAddress; \
                              CurrentIoAddressNum++)
                        {
                            if (TotalIoAddressEntries > NVODM_QUERY_MAX_IO_ADDRESSES)
                            {
                                NV_ASSERT( !"ERROR: s_Peripheral_IoAddresses[] is too small to accommodate entries!" );

                                // Cannot recover from this error.
                                NvOdmI2cClose(hOdmI2c);
                                return NV_FALSE;
                            }

                            RetVal = NvOdmPeripheralReadIoAddressData(
                                hOdmI2c,
                                EepromInst,
                                EepromAddressListOffset,
                                &s_Peripheral_IoAddresses[TotalIoAddressEntries+CurrentIoAddressNum]);

                            if (RetVal == NV_FALSE)
                            {
                                NV_ASSERT(!"Unable to read EEPROM (IoAddresses)!");

                                // Cannot recover from this error.
                                NvOdmI2cClose(hOdmI2c);
                                return NV_FALSE;
                            }
                            else // Process IoAddresses entry
                            {
                                /**
                                 * Save the addressList pointer. This points to the first 
                                 * IoAddresses entry of this class.  Then update the overall 
                                 * IoAddresses array counter (TotalIoAddressEntries).
                                 */
                                s_Peripherals[TotalPeripherals+CurrentPeripheral].AddressList = 
                                    &s_Peripheral_IoAddresses[TotalIoAddressEntries];

                                TotalIoAddressEntries += CurrentIoAddressNum;

                                // >-- End of NvOdmIoAddress array processing --<
                            }
                        }
                    }
                }
            }
            TotalPeripherals += NumPeripherals;
        }
    }

    // Done reading I2C-EEPROM; close it.
    NvOdmI2cClose(hOdmI2c);

    /** 
     *  Append static peripheral entries (if any) to dynamic list
     *  read from EEPROMs (this list may also be empty), except for
     *  duplicate GUIDs.  The dynamic list takes precedence when
     *  duplicate entries are found in the static list.
     */
    StaticPeripherals = NV_ARRAY_SIZE(s_Peripherals_Default);
    for (i=0; i<StaticPeripherals; i++)
    {
        for (j=0; j<TotalPeripherals; j++)
        {
            if (s_Peripherals_Default[i].Guid == s_Peripherals[j].Guid)
            {
                IsMatch = NV_TRUE;
                break;  // Ignore duplicate entry from static list.
            }
        }
        if (IsMatch != NV_TRUE)
        {
            // Append unique entry to dynamic list

            s_Peripherals[TotalPeripherals].Guid = 
                s_Peripherals_Default[i].Guid;

            s_Peripherals[TotalPeripherals].AddressList = 
                s_Peripherals_Default[i].AddressList;

            s_Peripherals[TotalPeripherals].NumAddress = 
                s_Peripherals_Default[i].NumAddress;

            s_Peripherals[TotalPeripherals].Class = 
                s_Peripherals_Default[i].Class;

            TotalPeripherals++;
        }
    }
    *pNum = TotalPeripherals;
    return NV_TRUE;
}

static NvBool
NvOdmPeripheralReadPartNumber(
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

NvBool
NvOdmPeripheralGetBoardInfo(
    NvU16 BoardId,
    NvOdmBoardInfo *pBoardInfo)
{
    NvBool RetVal = NV_FALSE;
    NvOdmServicesI2cHandle hOdmI2c = NULL;
    NvU8 EepromInst, CurrentBoard;
    static NvU8 NumBoards = 0;
    static NvBool s_ReadBoardInfoDone = NV_FALSE;

    if (!s_ReadBoardInfoDone)
    {
        s_ReadBoardInfoDone = NV_TRUE;
        hOdmI2c = NvOdmI2cOpen(NvOdmIoModule_I2c_Pmu, 0);
        if (!hOdmI2c)
        {
            // Exit
            pBoardInfo = NULL;
            return NV_FALSE;
        }

        for (EepromInst=0; EepromInst < NVODM_QUERY_MAX_EEPROMS; EepromInst++)
        {
            RetVal = NvOdmPeripheralReadPartNumber(
                    hOdmI2c, EepromInst, &s_BoardModuleTable[NumBoards]);
            if (RetVal == NV_TRUE)
                NumBoards++;
        }
        NvOdmI2cClose(hOdmI2c);
    }

    if (NumBoards)
    {
        // Linear search for given BoardId; if found, return entry
        for (CurrentBoard=0; CurrentBoard < NumBoards; CurrentBoard++)
        {
            if (s_BoardModuleTable[CurrentBoard].BoardID == BoardId)
            {
                // Match found
                pBoardInfo->BoardID  = s_BoardModuleTable[CurrentBoard].BoardID;
                pBoardInfo->SKU      = s_BoardModuleTable[CurrentBoard].SKU;
                pBoardInfo->Fab      = s_BoardModuleTable[CurrentBoard].Fab;
                pBoardInfo->Revision = s_BoardModuleTable[CurrentBoard].Revision;
                pBoardInfo->MinorRevision = s_BoardModuleTable[CurrentBoard].MinorRevision;
                return NV_TRUE;
            }           
        }
    }

    // Match not found
    pBoardInfo = NULL;
    return NV_FALSE;
}

// This will compare the peripheral GUID against a list of known-bad GUIDs
// for certain development kit personalities, and return NV_TRUE if it is
// known to be unsupported (filtered) on the current configuration
static NvBool
NvIsFilteredPeripheral(const NvOdmPeripheralConnectivity* pConnectivity)
{
    NvOdmServicesKeyListHandle hKeyList;
    NvU32 Personality = 0;
    NvU32 opt = 0;
    NvOdmIoModule OdmModule;
    const NvU32 *OdmConfigs=NULL;
    NvU32 NumOdmConfigs = 0;
    const NvOdmPeripheralConnectivity* pFilteredPeriph = pConnectivity;

    if((!pConnectivity) || (!pConnectivity->NumAddress))
        return NV_TRUE;

    hKeyList = NvOdmServicesKeyListOpen();

    if (hKeyList)
    {
        Personality =
            NvOdmServicesGetKeyValue(hKeyList,
                                     NvOdmKeyListId_ReservedBctCustomerOption);
        NvOdmServicesKeyListClose(hKeyList);
        Personality = 
            NV_DRF_VAL(TEGRA_DEVKIT, BCT_CUSTOPT, PERSONALITY, Personality);
        opt =
            NV_DRF_VAL(TEGRA_DEVKIT, BCT_CUSTOPT, DISPLAY_OPTION, opt);
    }

    if (!Personality)
        Personality = TEGRA_DEVKIT_DEFAULT_PERSONALITY;

    OdmModule = pFilteredPeriph->AddressList[0].Interface;

    if(OdmModule != NvOdmIoModule_Gpio)
        NvOdmQueryPinMux(OdmModule, &OdmConfigs, &NumOdmConfigs);

    switch (OdmModule)
    {
    case NvOdmIoModule_Gpio:
        // Filter scroll wheel when trace is enabled
        if ( (pConnectivity->Guid == NV_ODM_GUID('s','c','r','o','l','w','h','l')) &&
             ((Personality == TEGRA_DEVKIT_BCT_CUSTOPT_0_PERSONALITY_11) ||
              (Personality == TEGRA_DEVKIT_BCT_CUSTOPT_0_PERSONALITY_15) ||
              (Personality == TEGRA_DEVKIT_BCT_CUSTOPT_0_PERSONALITY_C1)) )
            return NV_TRUE;
        else
            return NV_FALSE;

    default:
        return NV_FALSE;
    }
}

static const NvOdmPeripheralConnectivity* 
NvApGetAllPeripherals (NvU32 *pNum)
{
    static NvBool s_AutoDetectDone = NV_FALSE;
    NvBool RetVal = NV_TRUE;
    static NvU32 s_TotalPeripherals;
    NvOdmBoardInfo BoardInfo;

    if (!pNum)
        return NULL;

    if (!s_AutoDetectDone)
    {
        /**
         *  Read & cache the board ID info from the I2C-EEPROMs.  This
         *  is necessary because once Whistler's thermal power rail is
         *  enabled, the ID ROMs cannot be read. NvApGetAllPeripherals()
         *  is called before that rail is enabled.
         */
        NvOdmPeripheralGetBoardInfo(NVODM_QUERY_BOARD_ID_UNKNOWN, &BoardInfo);

        RetVal = NvOdmPeripheralGetEntries(&s_TotalPeripherals);
        if (RetVal == NV_FALSE)
        {
            *pNum = 0;
            return NULL;
        }
        s_AutoDetectDone = NV_TRUE;
    }

    *pNum = s_TotalPeripherals;
    return (const NvOdmPeripheralConnectivity *)s_Peripherals;
}

// This implements a simple linear search across the entire set of currently-
// connected peripherals to find the set of GUIDs that Match the search
// criteria.  More clever implementations are possible, but given the
// relatively small search space (max dozens of peripherals) and the relative
// infrequency of enumerating peripherals, this is the easiest implementation.
const NvOdmPeripheralConnectivity *
NvOdmPeripheralGetGuid(NvU64 SearchGuid)
{
    const NvOdmPeripheralConnectivity *pAllPeripherals;
    NvU32 NumPeripherals;
    NvU32 i;

    pAllPeripherals = NvApGetAllPeripherals(&NumPeripherals);

    if (!pAllPeripherals || !NumPeripherals)
        return NULL;

    for (i=0; i<NumPeripherals; i++) 
    {
        if (SearchGuid == pAllPeripherals[i].Guid)
        {
            if (NvIsFilteredPeripheral(&pAllPeripherals[i]))
                return NULL;
            return &pAllPeripherals[i];
        }
    }

    return NULL;
}

static NvBool
IsBusMatch(
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

static NvBool
IsPeripheralMatch(
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

NvU32
NvOdmPeripheralEnumerate(
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
            if (NvIsFilteredPeripheral(&pAllPeripherals[i]))
                continue;

            if (pGuidList)
                pGuidList[Matches] = pAllPeripherals[i].Guid;
            Matches++;
        }
    }
    return Matches;
}

