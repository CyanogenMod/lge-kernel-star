/*
 * Copyright (c) 2007-2009 NVIDIA Corporation.
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

#ifndef PCF50626_SUPPLY_INFO_HEADER
#define PCF50626_SUPPLY_INFO_HEADER

#include "pcf50626.h"

#if defined(__cplusplus)
extern "C"
{
#endif


 typedef enum
{
    PCF50626PmuSupply_Invalid = 0x0,

    //DCD1
    PCF50626PmuSupply_DCD1,

    //DCD2
    PCF50626PmuSupply_DCD2,

    //DCUD
    PCF50626PmuSupply_DCUD,
    
    //DCULED
    PCF50626PmuSupply_DCULED,
     
    //RF1REG
    PCF50626PmuSupply_RF1REG,

    //RF2REG
    PCF50626PmuSupply_RF2REG,

    //RF3REG
    PCF50626PmuSupply_RF3REG,

    //RF4REG
    PCF50626PmuSupply_RF4REG,

    //D1REG
    PCF50626PmuSupply_D1REG,

    //D2REG
    PCF50626PmuSupply_D2REG,

    //D3REG
    PCF50626PmuSupply_D3REG,

    //D4REG
    PCF50626PmuSupply_D4REG,

    //D5REG
    PCF50626PmuSupply_D5REG,

    //D6REG
    PCF50626PmuSupply_D6REG,

    //D7REG
    PCF50626PmuSupply_D7REG,

    //D8REG
    PCF50626PmuSupply_D8REG,

    //HCREG
    PCF50626PmuSupply_HCREG,

    //IOREG
    PCF50626PmuSupply_IOREG,

    //USIMREG
    PCF50626PmuSupply_USIMREG,

    //USBREG
    PCF50626PmuSupply_USBREG,

    //LCREG
    PCF50626PmuSupply_LCREG,

    //VBAT
    PCF50626PmuSupply_VBAT,

    PCF50626PmuSupply_Num,
    PCF50626PmuSupply_Force32 = 0x7FFFFFFF
} PCF50626PmuSupply;


typedef struct PCF50626PmuSupplyInfoRec
{
    PCF50626PmuSupply supply;
    PCF50626PmuSupply supplyInput;
    NvU8 control1Addr;
    NvU8 control2Addr;
    NvU8 control3Addr;
    NvU8 control4Addr;

    NvU8 dvm1Addr;
    NvU8 dvm2Addr;
    NvU8 dvm3Addr;
    NvU8 dvmTimAddr;

    NvOdmPmuVddRailCapabilities cap;
    NvU32 offsetVoltage;    
    NvU32 turnOnTimeMicroSec;
    NvU32 switchTimeMicroSec;    
} PCF50626PmuSupplyInfo;

#if defined(__cplusplus)
}
#endif

#endif //PCF50626_VOLTAGE_INFO_TABLE_HEADER

