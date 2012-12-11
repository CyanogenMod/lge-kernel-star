/*
 * spi_mdm6600.h -- Serial peheripheral interface framing layer for MDM modem.
 *
 * Derived from ifx_n721_spi.h
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#ifndef SPI_MDM6600_H
#define SPI_MDM6600_H

/*------------------------------------------------------------------*/
/* Platform-specific configuration                                  */
/*------------------------------------------------------------------*/

/* Extended header support for SPI frames */
#define MSPI_EXTENDED_HEADER

/* Wakelocks support */
#define WAKE_LOCK_RESUME

/* SPI-to-SPI feature */
#define SPI2SPI_TEST

/* Always set Header.next to either 0 or max value */
//#define MSPI_NEXTMAXED

/* Use tty_insert_flip_string */
//#define USE_TTY_INSERT

#define IFX_SPI_MAJOR                   153 /* assigned */
#define IFX_N_SPI_MINORS                2   /* ... up to 256 */

/*------------------------------------------------------------------*/
/* Platform-specific defines                                        */
/*------------------------------------------------------------------*/

// TODO: Add the platform config defines here

/* OMAP specific */
#define OMAP_MODEM_WAKE                 121 // P2_CDMA REV.B


/* TEGRA specific */
/* Star ICS config*/
//#define MSPI_TEGRA_PMC_WAKEUP_PAD

#ifdef MSPI_TEGRA_PMC_WAKEUP_PAD
#include <mach/iomap.h>

#define PMC_WAKE_STATUS                 0x14
#define WAKEUP_IFX_SRDY_MASK            (1 << 7) // Wake Event 0 - IFX_SRDY
#endif /* MSPI_TEGRA_PMC_WAKEUP_PAD */

#include <mach/gpio-names.h>
//#define MSPI_MRDY_GPIO                  TEGRA_GPIO_PO5
//#define MSPI_SRDY_GPIO                  TEGRA_GPIO_PO0
#define MSPI_MRDY_GPIO                  TEGRA_GPIO_PU6
#define MSPI_SRDY_GPIO                  TEGRA_GPIO_PJ6

/*------------------------------------------------------------------*/
/* Buffer setup                                                     */
/*------------------------------------------------------------------*/

#ifdef MSPI_EXTENDED_HEADER

/* Default (minimal) mSPI frame size incl. header */
#define MSPI_DEF_BUFF_SIZE              1536

/* Extended header format for dynamic SPI buffer feat */
struct ifx_spi_frame_header {
    unsigned int unused_curr_data_size:12;
    unsigned int more:1;
    unsigned int res1:1;
    unsigned int res2:2;      
    unsigned int unused_next_data_size:12;
    unsigned int ri:1;
    unsigned int dcd:1;
    unsigned int cts_rts:1;
    unsigned int dsr_dtr:1;
    unsigned int curr_data_size:16;
    unsigned int next_data_size:16;
};

#else /* MSPI_EXTENDED_HEADER */

/* Default mSPI frame size incl. header */
#define MSPI_DEF_BUFF_SIZE              3200

/* Old header format for static SPI buffer feat */
struct ifx_spi_frame_header {
    unsigned int curr_data_size:12;
    unsigned int more:1;
    unsigned int res1:1;
    unsigned int res2:2;      
    unsigned int next_data_size:12;
    unsigned int ri:1;
    unsigned int dcd:1;
    unsigned int cts_rts:1;
    unsigned int dsr_dtr:1;
};

#endif  /* MSPI_EXTENDED_HEADER */

/* Size of the mSPI frame header */
#define MSPI_HEADER_SIZE                sizeof(struct ifx_spi_frame_header)

/* Maximum size of the data payload in the default sized frame */
#define MSPI_DEF_DATALOAD               (MSPI_DEF_BUFF_SIZE - MSPI_HEADER_SIZE)

/* Maximum mSPI frame size incl. header */
#define MSPI_MAX_BUFF_SIZE              4096

/* Maximum size of the data payload in the max sized frame */
#define MSPI_MAX_DATALOAD               (MSPI_MAX_BUFF_SIZE - MSPI_HEADER_SIZE)


#endif /* SPI_MDM6600_H */


