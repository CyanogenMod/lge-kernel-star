/*
 * Copyright (c) 2007-2009 NVIDIA Corporation.
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

