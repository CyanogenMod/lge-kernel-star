/*
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

#include "nvodm_kbc_keymapping.h"
#include <linux/input.h>

/* The total number of soc scan codes will be (first - last) */
#define NV_SOC_NORMAL_KEY_SCAN_CODE_TABLE_FIRST    0
#define NV_SOC_NORMAL_KEY_SCAN_CODE_TABLE_LAST     3

static NvU32 KbcLayOutVirtualKey[] =
{
 KEY_MENU,
 0,
 KEY_HOME,
 KEY_BACK
};

static struct NvOdmKeyVirtTableDetail s_ScvkKeyMap =
{
    NV_SOC_NORMAL_KEY_SCAN_CODE_TABLE_FIRST,    // scan code start
    NV_SOC_NORMAL_KEY_SCAN_CODE_TABLE_LAST,     // scan code end
    KbcLayOutVirtualKey          // Normal Qwerty keyboard
};


static const struct NvOdmKeyVirtTableDetail *s_pVirtualKeyTables[] =
     {&s_ScvkKeyMap};
     

NvU32 
NvOdmKbcKeyMappingGetVirtualKeyMappingList(
    const struct NvOdmKeyVirtTableDetail ***pVirtKeyTableList)
{
   *pVirtKeyTableList = s_pVirtualKeyTables;
   return NV_ARRAY_SIZE(s_pVirtualKeyTables);
}

