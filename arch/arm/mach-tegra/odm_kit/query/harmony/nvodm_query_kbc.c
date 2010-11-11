/*
 * arch/arm/mach-tegra/odm_kit/query/harmony/nvodm_query_kbc.c
 *
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

