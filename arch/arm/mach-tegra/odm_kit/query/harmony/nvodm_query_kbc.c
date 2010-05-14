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

/**
 * @file
 * @brief <b>NVIDIA Driver Development Kit:
 *                  ODM Kbc interface</b>
 *
 */

#include "nvodm_query_kbc.h"
#include "nvodm_query_kbc_gpio_def.h"
#include "nvodm_query_kbc_qwerty_def.h"

static NvU32 RowNumbers[] = {1, 15};
static NvU32 ColNumbers[] = {7, 0};

void
NvOdmKbcGetParameter(
        NvOdmKbcParameter Param,
        NvU32 SizeOfValue,
        void * pValue)
{
    NvU32 *pTempVar;
    switch (Param)
    {
        case NvOdmKbcParameter_DebounceTime:
            pTempVar = (NvU32 *)pValue;
            *pTempVar = 2;
            break;
        case NvOdmKbcParameter_RepeatCycleTime:
            pTempVar = (NvU32 *)pValue;
            *pTempVar = 5;
            break;
        default:
            break;
    }
}

NvU32 
NvOdmKbcGetKeyCode(
    NvU32 Row, 
    NvU32 Column,
    NvU32 RowCount,
    NvU32 ColumnCount)
{
    NvU32 CodeData;
    if (Row < KBC_QWERTY_FUNCTION_KEY_ROW_BASE)
    {
        CodeData = KBC_QWERTY_NORMAL_KEY_CODE_BASE + ((Row * ColumnCount) + Column);
    }    
    else
    {
        CodeData = KBC_QWERTY_FUNCTION_KEY_CODE_BASE + 
                        (((Row - KBC_QWERTY_FUNCTION_KEY_ROW_BASE) * ColumnCount) + Column);
    }
    return CodeData;
}

NvBool
NvOdmKbcIsSelectKeysWkUpEnabled(
    NvU32 **pRowNumber,
    NvU32 **pColNumber,
    NvU32 *NumOfKeys)
{
    *pRowNumber = &RowNumbers[0];
    *pColNumber = &ColNumbers[0];
    *NumOfKeys = 2;
    return NV_TRUE;
}

