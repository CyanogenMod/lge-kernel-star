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
 * <b>NVIDIA Tegra ODM Kit:
 *         Keyboard Controller virtual key mapping</b>
 *
 * @b Description: Implement the ODM keyboard mapping to the platform 
 *                  specific.
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

