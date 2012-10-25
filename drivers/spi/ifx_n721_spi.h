/*
 * ifx_n721_spi.h -- Serial peheripheral interface framing layer for IFX modem.
 *
 * Copyright (C) 2009 Texas Instruments
 * Authors:	Umesh Bysani <bysani@ti.com> and
 *		Shreekanth D.H <sdh@ti.com>
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

//20110608 ws.yang@lge.com fix to ifx modem [S]
#ifndef IFX_N721_SPI_H
#define IFX_N721_SPI_H

#define IFX_SPI_MAJOR			153	/* assigned */
#define IFX_N_SPI_MINORS		2 // 4	/* ... up to 256 */

#define IFX_SPI_MAX_BUF_SIZE			2044	/* Max buffer size */
#define IFX_SPI_DEFAULT_BUF_SIZE		2044 	/* Default buffer size*/
#define IFX_SPI_HEADER_SIZE			4

#define SPI_MODE_0			(0|0)
#define SPI_MODE_1			(0|SPI_CPHA)
#define SPI_MODE_2			(SPI_CPOL|0)
#define SPI_MODE_3			(SPI_CPOL|SPI_CPHA)

#define SPI_CPHA			0x01
#define SPI_CPOL			0x02
#define SPI_CS_HIGH			0x04
#define SPI_LSB_FIRST			0x08
#define SPI_3WIRE			0x10
#define SPI_LOOP			0x20

/* Bit masks for spi_device.mode management.  Note that incorrect
 * settings for CS_HIGH and 3WIRE can cause *lots* of trouble for other
 * devices on a shared bus:  CS_HIGH, because this device will be
 * active when it shouldn't be;  3WIRE, because when active it won't
 * behave as it should.
 *
 * REVISIT should changing those two modes be privileged?
 */
#define SPI_MODE_MASK			(SPI_CPHA | SPI_CPOL | SPI_CS_HIGH \
					| SPI_LSB_FIRST | SPI_3WIRE | SPI_LOOP)


/* tegra_set_lp0_wake_type() 에서 
    case IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING 처리.. */
#define IFX_TEGRA_EDGE_TRIGGER

//20110614 ws.yang@lge.com ..add to measure of spi speed [S]
//#define IFX_SPI_SPEED_MEASUREMENT
#ifdef IFX_SPI_SPEED_MEASUREMENT
#define IFX_SPI_TX_RX_THROUGHTPUT
#undef IFX_SPI_TEGRA_TRANSFER_DURATION
#else
//#define IFX_SPI_TEGRA_TRANSFER_DURATION
#endif
//20110614 ws.yang@lge.com ..add to measure of spi speed [E]


//#define IFX_DUMP_SPI_BIFFUER

//#define IFX_SPI_TX_RX_BUF
//#define IFX_SPI_DUMP_LOG
#ifdef IFX_SPI_DUMP_LOG
typedef struct {
	__u8 ea:1;
	__u8 cr:1;
	__u8 d:1;
	__u8 server_chn:5;
} __attribute__ ((packed)) address_field;

typedef struct {
	__u8 ea:1;
	__u8 len:7;
} __attribute__ ((packed)) short_length;

typedef struct {
	__u8 ea:1;
	__u8 l_len:7;
	__u8 h_len;
} __attribute__ ((packed)) long_length;

typedef struct {
	address_field addr;
	__u8 control;
	short_length length;
} __attribute__ ((packed)) short_frame_head;

typedef struct {
	short_frame_head h;
	__u8 data[0];
} __attribute__ ((packed)) short_frame;

typedef struct {
	address_field addr;
	__u8 control;
	long_length length;
	__u8 data[0];
} __attribute__ ((packed)) long_frame_head;

typedef struct {
	long_frame_head h;
	__u8 data[0];
} __attribute__ ((packed)) long_frame;

#define GET_LONG_LENGTH(a) ( ((a).h_len << 7) | ((a).l_len) )
#endif //IFX_SPI_DUMP_LOG

#endif /* IFX_N721_SPI_H */

//#define IFX_SPI_DEBUG_MODE
#ifdef IFX_SPI_DEBUG_MODE
#define IFX_SPI_DEBUG(format, args...) printk("[SPI] : %s (%d line): " format "\n", __FUNCTION__, __LINE__, ## args)
#else
#define IFX_SPI_DEBUG(format, args...) 
#endif

#define IFX_SPI_TX_DEBUG(format, args...)

#define IFX_SPI_PRINTK(format, args...) printk("[SPI] : %s (%d line): " format "\n", __FUNCTION__, __LINE__, ## args)

//20110608 ws.yang@lge.com fix to ifx modem [S]

