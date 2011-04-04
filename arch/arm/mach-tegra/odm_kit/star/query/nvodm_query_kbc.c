/*
 * arch/arm/mach-tegra/odm_kit/query/whistler/nvodm_query_kbc.c
 *
 * Copyright (c) 2009 NVIDIA Corporation.
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


//#define WAKEUP_ONLY_SELECTED_KEYS 1
// Queries if wake-up only on selected keys is enabled for WPC

#ifndef WAKEUP_ONLY_SELECTED_KEYS
static NvU32 *RowNumbers = NULL;
static NvU32 *ColNumbers = NULL;
#else
static NvU32 RowNumbers_test[] = {0, 1, 2};
static NvU32 ColNumbers_test[] = {0, 1, 2};
#endif

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
            *pTempVar = 10;
            break;
        case NvOdmKbcParameter_RepeatCycleTime:
            pTempVar = (NvU32 *)pValue;
            *pTempVar = 32;
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
    return ((Row * ColumnCount) + Column +1);
}

NvBool
NvOdmKbcIsSelectKeysWkUpEnabled(
    NvU32 **pRowNumber,
    NvU32 **pColNumber,
    NvU32 *NumOfKeys)
{
#ifndef WAKEUP_ONLY_SELECTED_KEYS
    *pRowNumber = RowNumbers;
    *pColNumber = ColNumbers;
    *NumOfKeys = 0;
    return NV_FALSE;
#else
    *pRowNumber = &RowNumbers_test[0];
    *pColNumber = &ColNumbers_test[0];
    *NumOfKeys = ( NV_ARRAY_SIZE(RowNumbers_test) *  NV_ARRAY_SIZE(ColNumbers_test) );
    return NV_TRUE;
#endif
}

