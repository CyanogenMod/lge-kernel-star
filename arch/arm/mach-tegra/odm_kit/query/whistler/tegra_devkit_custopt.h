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
 *         Definition of bitfields inside the BCT customer option</b>
 *
 * @b Description: Defines the board-specific bitfields of the 
 *                 BCT customer option parameter, for NVIDIA
 *                 Tegra development platforms.
 *  
 *                 This file pertains to Whistler and Voyager.
 */

#ifndef NVIDIA_TEGRA_DEVKIT_CUSTOPT_H
#define NVIDIA_TEGRA_DEVKIT_CUSTOPT_H

#if defined(__cplusplus)
extern "C"
{
#endif

//---------- BOARD PERSONALITIES (BEGIN) ----------//
// On the Whistler boards, be sure to match the following
// switches with the personality setting you choose.
// 
//      SW2 = bits 3:0 (low nibble)
//      SW3 = bits 7:4 (high nibble)

#define TEGRA_DEVKIT_DEFAULT_PERSONALITY \
    TEGRA_DEVKIT_BCT_CUSTOPT_0_PERSONALITY_75

#define TEGRA_DEVKIT_BCT_CUSTOPT_0_PERSONALITY_RANGE     7:0
#define TEGRA_DEVKIT_BCT_CUSTOPT_0_PERSONALITY_DEFAULT  0x0UL

// VOYAGER, eMMC, NO TRACE (10x8 keypad)
#define TEGRA_DEVKIT_BCT_CUSTOPT_0_PERSONALITY_01       0x01UL  // ULPI = baseband
#define TEGRA_DEVKIT_BCT_CUSTOPT_0_PERSONALITY_05       0x05UL  // ULPI = UART1

// VOYAGER, eMMC, with TRACE (7x1 keypad)
#define TEGRA_DEVKIT_BCT_CUSTOPT_0_PERSONALITY_11       0x11UL  // ULPI = baseband
#define TEGRA_DEVKIT_BCT_CUSTOPT_0_PERSONALITY_15       0x15UL  // ULPI = UART1

// VOYAGER, NAND, NO TRACE (10x8 keypad)
#define TEGRA_DEVKIT_BCT_CUSTOPT_0_PERSONALITY_75       0x75UL  // Voyager, NAND

// WHISTLER, stand-alone
#define TEGRA_DEVKIT_BCT_CUSTOPT_0_PERSONALITY_C1       0xC1UL  // KB = 13x1, TRACE, GMI = A/D NOR
#define TEGRA_DEVKIT_BCT_CUSTOPT_0_PERSONALITY_C3       0xC3UL  // KB = 16x8, NO TRACE, GMI = NAND
#define TEGRA_DEVKIT_BCT_CUSTOPT_0_PERSONALITY_C4       0xC4UL  


// VOYAGER, USB2-ULPI (No UART1)
// Personality 71 is similar to the 75, except ULPI is enabled instead of UART1.
#define TEGRA_DEVKIT_BCT_CUSTOPT_0_PERSONALITY_71       0x71UL  


//---------- BOARD PERSONALITIES (END) ----------//

/// Download transport
#define TEGRA_DEVKIT_BCT_CUSTOPT_0_TRANSPORT_RANGE      10:8
#define TEGRA_DEVKIT_BCT_CUSTOPT_0_TRANSPORT_DEFAULT    0x0UL
#define TEGRA_DEVKIT_BCT_CUSTOPT_0_TRANSPORT_NONE       0x1UL
#define TEGRA_DEVKIT_BCT_CUSTOPT_0_TRANSPORT_UART       0x2UL
#define TEGRA_DEVKIT_BCT_CUSTOPT_0_TRANSPORT_USB        0x3UL
#define TEGRA_DEVKIT_BCT_CUSTOPT_0_TRANSPORT_ETHERNET   0x4UL

/// Transport option (bus selector), for UART and Ethernet transport
#define TEGRA_DEVKIT_BCT_CUSTOPT_0_UART_OPTION_RANGE   12:11
#define TEGRA_DEVKIT_BCT_CUSTOPT_0_UART_OPTION_DEFAULT 0x0UL
#define TEGRA_DEVKIT_BCT_CUSTOPT_0_UART_OPTION_A       0x1UL
#define TEGRA_DEVKIT_BCT_CUSTOPT_0_UART_OPTION_B       0x2UL
#define TEGRA_DEVKIT_BCT_CUSTOPT_0_UART_OPTION_C       0x3UL

#define TEGRA_DEVKIT_BCT_CUSTOPT_0_ETHERNET_OPTION_RANGE   12:11
#define TEGRA_DEVKIT_BCT_CUSTOPT_0_ETHERNET_OPTION_DEFAULT 0x0UL
#define TEGRA_DEVKIT_BCT_CUSTOPT_0_ETHERNET_OPTION_SPI     0x1UL

/// RIL selection
#define TEGRA_DEVKIT_BCT_CUSTOPT_0_RIL_RANGE              14:13
#define TEGRA_DEVKIT_BCT_CUSTOPT_0_RIL_DEFAULT            0x0UL
#define TEGRA_DEVKIT_BCT_CUSTOPT_0_RIL_EMP_RAINBOW        0x1UL
#define TEGRA_DEVKIT_BCT_CUSTOPT_0_RIL_EMP_RAINBOW_ULPI   0x2UL
#define TEGRA_DEVKIT_BCT_CUSTOPT_0_RIL_IFX                0x3UL

#define TEGRA_DEVKIT_BCT_CUSTOPT_0_CONSOLE_OPTION_RANGE    17:15
#define TEGRA_DEVKIT_BCT_CUSTOPT_0_CONSOLE_OPTION_DEFAULT  0
#define TEGRA_DEVKIT_BCT_CUSTOPT_0_CONSOLE_OPTION_UARTA    0
#define TEGRA_DEVKIT_BCT_CUSTOPT_0_CONSOLE_OPTION_UARTB    1
#define TEGRA_DEVKIT_BCT_CUSTOPT_0_CONSOLE_OPTION_UARTC    2
#define TEGRA_DEVKIT_BCT_CUSTOPT_0_CONSOLE_OPTION_UARTD    3
#define TEGRA_DEVKIT_BCT_CUSTOPT_0_CONSOLE_OPTION_UARTE    4

#define TEGRA_DEVKIT_BCT_CUSTOPT_0_CONSOLE_RANGE           19:18
#define TEGRA_DEVKIT_BCT_CUSTOPT_0_CONSOLE_DEFAULT         0
#define TEGRA_DEVKIT_BCT_CUSTOPT_0_CONSOLE_NONE            1
#define TEGRA_DEVKIT_BCT_CUSTOPT_0_CONSOLE_DCC             2
#define TEGRA_DEVKIT_BCT_CUSTOPT_0_CONSOLE_UART            3

// display options
#define TEGRA_DEVKIT_BCT_CUSTOPT_0_DISPLAY_OPTION_RANGE      22:20
#define TEGRA_DEVKIT_BCT_CUSTOPT_0_DISPLAY_OPTION_DEFAULT    0x0UL
// embedded panel (lvds, dsi, etc)
#define TEGRA_DEVKIT_BCT_CUSTOPT_0_DISPLAY_OPTION_EMBEDDED   0x0UL
// no panels (external or embedded)
#define TEGRA_DEVKIT_BCT_CUSTOPT_0_DISPLAY_OPTION_NULL       0x1UL
// use hdmi as the primary
#define TEGRA_DEVKIT_BCT_CUSTOPT_0_DISPLAY_OPTION_HDMI       0x2UL
// use crt as the primary 
#define TEGRA_DEVKIT_BCT_CUSTOPT_0_DISPLAY_OPTION_CRT        0x3UL


// Enable DHCP
#define TEGRA_DEVKIT_BCT_CUSTOPT_0_DHCP_RANGE           23:23
#define TEGRA_DEVKIT_BCT_CUSTOPT_0_DHCP_DEFAULT         0x0UL
#define TEGRA_DEVKIT_BCT_CUSTOPT_0_DHCP_ENABLE          0x1UL

/// Carveout RAM
#define TEGRA_DEVKIT_BCT_CARVEOUT_0_MEMORY_RANGE    27:24
#define TEGRA_DEVKIT_BCT_CARVEOUT_0_MEMORY_DEFAULT  0x0UL
#define TEGRA_DEVKIT_BCT_CARVEOUT_0_MEMORY_1        0x1UL
#define TEGRA_DEVKIT_BCT_CARVEOUT_0_MEMORY_2        0x2UL
#define TEGRA_DEVKIT_BCT_CARVEOUT_0_MEMORY_3        0x3UL
#define TEGRA_DEVKIT_BCT_CARVEOUT_0_MEMORY_4        0x4UL
#define TEGRA_DEVKIT_BCT_CARVEOUT_0_MEMORY_5        0x5UL
#define TEGRA_DEVKIT_BCT_CARVEOUT_0_MEMORY_6        0x6UL
#define TEGRA_DEVKIT_BCT_CARVEOUT_0_MEMORY_7        0x7UL
#define TEGRA_DEVKIT_BCT_CARVEOUT_0_MEMORY_8        0x8UL //32 MB


/// Total RAM
#define TEGRA_DEVKIT_BCT_SYSTEM_0_MEMORY_RANGE    30:28
#define TEGRA_DEVKIT_BCT_SYSTEM_0_MEMORY_DEFAULT  0x0UL
#define TEGRA_DEVKIT_BCT_SYSTEM_0_MEMORY_1        0x1UL
#define TEGRA_DEVKIT_BCT_SYSTEM_0_MEMORY_2        0x2UL
#define TEGRA_DEVKIT_BCT_SYSTEM_0_MEMORY_3        0x3UL
#define TEGRA_DEVKIT_BCT_SYSTEM_0_MEMORY_4        0x4UL
#define TEGRA_DEVKIT_BCT_SYSTEM_0_MEMORY_5        0x5UL
#define TEGRA_DEVKIT_BCT_SYSTEM_0_MEMORY_6        0x6UL
#define TEGRA_DEVKIT_BCT_SYSTEM_0_MEMORY_7        0x7UL
/// Soc low power state
#define TEGRA_DEVKIT_BCT_CUSTOPT_0_LPSTATE_RANGE            31:31
#define TEGRA_DEVKIT_BCT_CUSTOPT_0_LPSTATE_LP0              0x0UL
#define TEGRA_DEVKIT_BCT_CUSTOPT_0_LPSTATE_LP1              0x1UL

#if defined(__cplusplus)
}
#endif

#endif
