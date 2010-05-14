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


#define KBC_QWERTY_NORMAL_KEY_CODE_BASE          0x1000
#define KBC_QWERTY_FUNCTION_KEY_CODE_BASE        0x2000

#define KBC_QWERTY_FUNCTION_KEY_ROW_BASE         0x100
#define KBC_QWERTY_FUNCTION_KEY_ROW_NUMBER       0
#define KBC_QWERTY_FUNCTION_KEY_COLUMN_NUMBER    7

/**
 * @brief Scan Code to Virtual Key mappings.
 */


/* The total number of soc scan codes will be (first - last) */
#define NV_SOC_NORMAL_KEY_SCAN_CODE_TABLE_FIRST      KBC_QWERTY_NORMAL_KEY_CODE_BASE
#define NV_SOC_NORMAL_KEY_SCAN_CODE_TABLE_LAST       (KBC_QWERTY_NORMAL_KEY_CODE_BASE +0x7F)

#define NV_SOC_FUNCTION_KEY_SCAN_CODE_TABLE_FIRST    KBC_QWERTY_FUNCTION_KEY_CODE_BASE
#define NV_SOC_FUNCTION_KEY_SCAN_CODE_TABLE_LAST     (KBC_QWERTY_FUNCTION_KEY_CODE_BASE +0x7F)

/**
 * @brief This is the actual Scan-code-to-VKey mapping table. For new layouts
 *        this is the only structure which needs to be modified to return the
 *        proper vkey depending on the scan code.
 */

#define KEY_UNUSED 0

static NvU32 ScanCodeToVKeyTableKbcQwertyNormal[] =
{
    // Row 0-> Unused, Unused, 'W',     'S',    'A',     'Z',       Unused, Function,
    // Row 1 ->Unused, Unused, Unused,  Unused, Unused,  unused,     Unused, WIN_SPECIAL
    // Row 2 ->Unused, Unused, Unused,  Unused, Unused,  unused,     Alt,    Alt2
    // Row 3 ->'5',    '4',    'R',     'E',    'F',     'D',        'X',    Unused,
    // Row 4 ->'7',    '6',    'T',     'H',    'G',     'V',        'C',    SPACEBAR,
    // Row 5 ->'9',    '8',    'U',     'Y',    'J',     'N',        'B',    '|\',
    // Row 6 ->Minus,  '0',    'O',     'I',    'L',     'K',        '<',    M,
    // Row 7 ->unused, '+',    '}]',    '#',    Unused,  Unused,     Unused, WinSpecial,
    // Row 8 ->Unused, Unused, Unused,  Unused, SHIFT,   SHIFT,      UnUsed, Unused ,
    // Row 9 ->Unused, Unused, Unused,  Unused, unused,  Ctrl,       UnUsed, Control,
    // Row A ->Unused, Unused, Unused,  Unused, unused,  unused,     UnUsed, Unused,
    // Row B ->'{[',   'P',    '"',     ':;',   '/?,      '>',       UnUsed, Unused,
    // Row C ->'F10',  'F9',   'BckSpc','3',    '2',     'Up,        Prntscr,Pause
    // Row D ->INS,    DEL,    Unused,  Pgup,   PgDn,    right,      Down,   Left,
    // Row E ->F11,    F12,    F8,      'Q',    F4,      F3,         '1',    F7,
    // Row F ->ESC,    '~',     F5,      TAB,    F1,      F2,         CAPLOCK,F6,
       KEY_UNUSED,     KEY_UNUSED,     KEY_W,           KEY_S,
                KEY_A,         KEY_Z,         KEY_UNUSED,    KEY_FN,
       KEY_UNUSED,       KEY_UNUSED,     KEY_UNUSED,      KEY_UNUSED,
                KEY_UNUSED,    KEY_UNUSED,    KEY_UNUSED,    KEY_MENU,
       KEY_UNUSED,     KEY_UNUSED,     KEY_UNUSED,      KEY_UNUSED,
                KEY_UNUSED,    KEY_UNUSED,    KEY_LEFTALT,     KEY_RIGHTALT,
       KEY_5,          KEY_4,          KEY_R,           KEY_E,         
                KEY_F,         KEY_D,         KEY_X,         KEY_UNUSED,
       KEY_7,          KEY_6,          KEY_T,           KEY_H,         
                KEY_G,         KEY_V,         KEY_C,         KEY_SPACE,
       KEY_9,          KEY_8,          KEY_U,           KEY_Y,         
                KEY_J,         KEY_N,         KEY_B,         KEY_BACKSLASH,
       KEY_MINUS,      KEY_0,          KEY_O,           KEY_I,         
                KEY_L,         KEY_K,         KEY_COMMA,     KEY_M,
       KEY_UNUSED,     KEY_EQUAL,      KEY_RIGHTBRACE,    KEY_ENTER,     
                KEY_UNUSED,    KEY_UNUSED,     KEY_UNUSED,     KEY_MENU,
       KEY_UNUSED,     KEY_UNUSED,     KEY_UNUSED,     KEY_UNUSED,     
                KEY_LEFTSHIFT,     KEY_RIGHTSHIFT,     KEY_UNUSED,     KEY_UNUSED,
       KEY_UNUSED,     KEY_UNUSED,     KEY_UNUSED,     KEY_UNUSED,     
                KEY_UNUSED,     KEY_LEFTCTRL,    KEY_UNUSED,     KEY_RIGHTCTRL,
       KEY_UNUSED,     KEY_UNUSED,     KEY_UNUSED,     KEY_UNUSED,     
                KEY_UNUSED,     KEY_UNUSED,     KEY_UNUSED,     KEY_UNUSED,
       KEY_LEFTBRACE,  KEY_P,          KEY_APOSTROPHE,    KEY_SEMICOLON,    
                KEY_SLASH,    KEY_DOT,    KEY_UNUSED,   KEY_UNUSED,
       KEY_F10,        KEY_F9,         KEY_BACKSPACE,       KEY_3,      
                KEY_2,           KEY_UP,         KEY_PRINT,   KEY_PAUSE,
       KEY_INSERT,     KEY_DELETE,     KEY_UNUSED,     KEY_PAGEUP,      
                KEY_PAGEDOWN,       KEY_RIGHT,      KEY_DOWN,       KEY_LEFT,
       KEY_F11,        KEY_F12,        KEY_F8,         KEY_Q,           
                KEY_F4,         KEY_F3,         KEY_1,           KEY_F7,
       KEY_ESC,        KEY_GRAVE,      KEY_F5,         KEY_TAB,         
                KEY_F1,         KEY_F2,          KEY_CAPSLOCK ,    KEY_F6
};

static NvU32 ScanCodeToVKeyTableKbcQwertyFunction[] =
{
    // Row 0-> Unused, Unused, 'W',     'S',    'A',     'Z',       Unused, Function,
    // Row 1 ->WINSPECIAL, Unused, Unused,  Unused, Unused,  unused,     Unused, Win_special
    // Row 2 ->Unused, Unused, Unused,  Unused, Unused,  unused,     Alt,    Alt2
    // Row 3 ->'5',    '4',    'R',     'E',    'F',     'D',        'X',    Unused,
    // Row 4 ->'7',    '6',    'T',     'H',    'G',     'V',        'C',    SPACEBAR,
    // Row 5 ->'9',    '8',    'U',     'Y',    'J',     'N',        'B',    '|\',
    // Row 6 ->Minus,  '0',    'O',     'I',    'L',     'K',        '<',    M,
    // Row 7 ->unused, '+',    '}]',    '#',    Unused,  Unused,     Unused, WinSpecial,
    // Row 8 ->Unused, Unused, Unused,  Unused, SHIFT,   SHIFT,      UnUsed, Unused ,
    // Row 9 ->Unused, Unused, Unused,  Unused, unused,  Ctrl,       UnUsed, Control,
    // Row A ->Unused, Unused, Unused,  Unused, unused,  unused,     UnUsed, Unused,
    // Row B ->'{[',   'P',    '"',     ':;',   '/?,      '>',       UnUsed, Unused,
    // Row C ->'F10',  'F9',   'BckSpc','3',    '2',     'Up,        Prntscr,Pause
    // Row D ->INS,    DEL,    Unused,  Pgup,   PgDn,    right,      Down,   Left,
    // Row E ->F11,    F12,    F8,      'Q',    F4,      F3,         '1',    F7,
    // Row F ->ESC,    '~',     F5,      TAB,    F1,      F2,         CAPLOCK,F6,

    KEY_UNUSED,  KEY_UNUSED,  KEY_UNUSED,  KEY_UNUSED,  KEY_UNUSED,  KEY_UNUSED,  KEY_UNUSED,  KEY_UNUSED,
    KEY_UNUSED,  KEY_UNUSED,  KEY_UNUSED,  KEY_UNUSED,  KEY_UNUSED,  KEY_UNUSED,  KEY_UNUSED,  KEY_UNUSED,
    KEY_UNUSED,  KEY_UNUSED,  KEY_UNUSED,  KEY_UNUSED,  KEY_UNUSED,  KEY_UNUSED,  KEY_UNUSED,  KEY_UNUSED,
    KEY_UNUSED,  KEY_UNUSED,  KEY_UNUSED,  KEY_UNUSED,  KEY_UNUSED,  KEY_UNUSED,  KEY_UNUSED,  KEY_UNUSED,
    KEY_7,       KEY_UNUSED,  KEY_UNUSED,  KEY_UNUSED,  KEY_UNUSED,  KEY_UNUSED,  KEY_UNUSED,  KEY_UNUSED,
    KEY_9,       KEY_8,       KEY_4,       KEY_UNUSED,  KEY_1,       KEY_UNUSED,  KEY_UNUSED,  KEY_UNUSED,
    KEY_UNUSED,  KEY_SLASH,   KEY_6,       KEY_5,       KEY_3,       KEY_2,       KEY_UNUSED,  KEY_0,
    KEY_UNUSED,  KEY_UNUSED,  KEY_UNUSED,  KEY_UNUSED,  KEY_UNUSED,  KEY_UNUSED,  KEY_UNUSED,  KEY_UNUSED,
    KEY_UNUSED,  KEY_UNUSED,  KEY_UNUSED,  KEY_UNUSED,  KEY_UNUSED,  KEY_UNUSED,  KEY_UNUSED,  KEY_UNUSED,
    KEY_UNUSED,  KEY_UNUSED,  KEY_UNUSED,  KEY_UNUSED,  KEY_UNUSED,  KEY_UNUSED,  KEY_UNUSED,  KEY_UNUSED,
    KEY_UNUSED,  KEY_UNUSED,  KEY_UNUSED,  KEY_UNUSED,  KEY_UNUSED,  KEY_UNUSED,  KEY_UNUSED,  KEY_UNUSED,
    KEY_UNUSED,  KEY_KPASTERISK,  KEY_UNUSED,  KEY_KPMINUS,  KEY_KPPLUS,  KEY_DOT,  KEY_UNUSED,  KEY_UNUSED,
    KEY_UNUSED,  KEY_UNUSED,  KEY_UNUSED,  KEY_UNUSED,  KEY_UNUSED,  KEY_VOLUMEUP,  KEY_UNUSED,  KEY_UNUSED,
    KEY_UNUSED,  KEY_UNUSED,  KEY_UNUSED,  KEY_HOME,    KEY_END,     KEY_BRIGHTNESSUP,  KEY_VOLUMEDOWN,  KEY_BRIGHTNESSDOWN,
    KEY_NUMLOCK, KEY_SCROLLLOCK,  KEY_MUTE,KEY_UNUSED,  KEY_UNUSED,  KEY_UNUSED,  KEY_UNUSED,  KEY_UNUSED,
    KEY_UNUSED,  KEY_UNUSED,  KEY_UNUSED,  KEY_UNUSED,  KEY_QUESTION,KEY_UNUSED,  KEY_UNUSED,  KEY_UNUSED
};
static struct NvOdmKeyVirtTableDetail s_ScvkQwertyNormalEngUS =
{
    NV_SOC_NORMAL_KEY_SCAN_CODE_TABLE_FIRST,    // scan code start
    NV_SOC_NORMAL_KEY_SCAN_CODE_TABLE_LAST,     // scan code end
    ScanCodeToVKeyTableKbcQwertyNormal          // Normal Qwerty keyboard
};

static struct NvOdmKeyVirtTableDetail s_ScvkQwertyFunctionEngUS =
{
    NV_SOC_FUNCTION_KEY_SCAN_CODE_TABLE_FIRST,      // scan code start
    NV_SOC_FUNCTION_KEY_SCAN_CODE_TABLE_LAST,       // scan code end
    ScanCodeToVKeyTableKbcQwertyFunction            // Function Qwerty keyboard
};

static const struct NvOdmKeyVirtTableDetail *s_pVirtualKeyTables[] =
     {&s_ScvkQwertyNormalEngUS, &s_ScvkQwertyFunctionEngUS};


NvU32
NvOdmKbcKeyMappingGetVirtualKeyMappingList(
    const struct NvOdmKeyVirtTableDetail ***pVirtKeyTableList)
{
   *pVirtKeyTableList = s_pVirtualKeyTables;
   return NV_ARRAY_SIZE(s_pVirtualKeyTables);
}
