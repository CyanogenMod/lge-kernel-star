/*
 * arch/arm/mach-tegra/odm_kit/query/whistler/tegra_devkit_custopt.h
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
#define TEGRA_DEVKIT_BCT_CARVEOUT_0_MEMORY_9        0x9UL //48 MB
#define TEGRA_DEVKIT_BCT_CARVEOUT_0_MEMORY_A        0xaUL //64 MB
#define TEGRA_DEVKIT_BCT_CARVEOUT_0_MEMORY_B        0xbUL //128 MB


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
