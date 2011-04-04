/*
 * mux_macro.h
 *
 * Copyright (C) 2002 2005 Motorola
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
 *
 *
 *  11/18/2002  (Motorola) - Initial version
 *
 */

/*
* This header file should be included by both MUX and other applications
* which access MUX device files. It gives the additional macro definitions
* shared between MUX and applications.
*/

/* MUX DLCI(Data Link Connection Identifier) Configuration */
/*
*  DLCI     Service
*   0    Control Channel
*   1    Voice Call & Network-related
*   2    SMS MO
*   3    SMS MT
*   4    Phonebook & related
*   5    MISC
*   6    CSD/FAX
*   7    GPRS1
*   8    GPRS2
*   9    Logger CMD
*   10   Logger Data
*   11   Test CMD
*   12   AGPS
*   13   Net Monitor
*/

/* Mapping between DLCI and MUX device files */
/*
*   File Name   Minor  DLCI  AT Command/Data
*   /dev/mux0     0     1     AT Command
*   /dev/mux1     1     2     AT Command
*   /dev/mux2     2     3     AT Command
*   /dev/mux3     3     4     AT Command
*   /dev/mux4     4     5     AT Command
*   /dev/mux5     5     6     AT Command
*   /dev/mux6     6     7     AT Command
*   /dev/mux7     7     8     AT Command
*   /dev/mux8     8     6     Data
*   /dev/mux9     9     7     Data
*   /dev/mux10    10    8     Data
*   /dev/mux11    11    9     Data
*   /dev/mux12    12    10    Data
*   /dev/mux13    13    11    Data
*   /dev/mux14    14    12    Data
*   /dev/mux15    15    13    Data
*/

#define MUX_CMD_FILE_VOICE_CALL   "/dev/mux0"
#define MUX_CMD_FILE_SMS_MO       "/dev/mux1"
#define MUX_CMD_FILE_SMS_MT       "/dev/mux2"
#define MUX_CMD_FILE_PHONEBOOK    "/dev/mux3"
#define MUX_CMD_FILE_MISC         "/dev/mux4"
#define MUX_CMD_FILE_CSD          "/dev/mux5"
#define MUX_CMD_FILE_GPRS1        "/dev/mux6"
#define MUX_CMD_FILE_GPRS2        "/dev/mux7"

#define MUX_DATA_FILE_CSD         "/dev/mux8"
#define MUX_DATA_FILE_GPRS1       "/dev/mux9"
#define MUX_DATA_FILE_GPRS2       "/dev/mux10"
#define MUX_DATA_FILE_LOGGER_CMD  "/dev/mux11"
#define MUX_DATA_FILE_LOGGER_DATA "/dev/mux12"
#define MUX_DATA_FILE_TEST_CMD    "/dev/mux13"
#define MUX_DATA_FILE_AGPS        "/dev/mux14"
#define MUX_DATA_FILE_NET_MONITOR "/dev/mux15"

#define NUM_MUX_CMD_FILES 8
#define NUM_MUX_DATA_FILES 8
#define NUM_MUX_FILES ( NUM_MUX_CMD_FILES  +  NUM_MUX_DATA_FILES )

/* Special ioctl() upon a MUX device file for hanging up a call */
#define TS0710MUX_IO_MSC_HANGUP 0x54F0

/* Special ioctl() upon a MUX device file for MUX loopback test */
#define TS0710MUX_IO_TEST_CMD 0x54F1

/* Special Error code might be return from write() to a MUX device file  */
#define EDISCONNECTED 900	/* Logical data link is disconnected */

/* Special Error code might be return from open() to a MUX device file  */
#define EREJECTED 901		/* Logical data link connection request is rejected */

#ifdef LGE_KERNEL_MUX
#define TS0710MUX_MAJOR 0  /* Use dynamic allocation*/
#else
#define TS0710MUX_MAJOR 250 VBB 
#endif

#define TS0710MUX_MINOR_START 0




#define TS0710MUX_TIME_OUT 250	/* 2500ms, for BP UART hardware flow control AP UART  */

#define TS0710MUX_IO_DLCI_FC_ON 0x54F2
#define TS0710MUX_IO_DLCI_FC_OFF 0x54F3
#define TS0710MUX_IO_FC_ON 0x54F4
#define TS0710MUX_IO_FC_OFF 0x54F5

#define TS0710MUX_MAX_BUF_SIZE 2048

#define TS0710MUX_SEND_BUF_OFFSET 10
#define TS0710MUX_SEND_BUF_SIZE (DEF_TS0710_MTU + TS0710MUX_SEND_BUF_OFFSET + 34)
#define TS0710MUX_RECV_BUF_SIZE TS0710MUX_SEND_BUF_SIZE

#define ACK_SPACE 66		/* 6 * 11(ACK frame size)  */

#define TS0710MUX_SERIAL_BUF_SIZE (DEF_TS0710_MTU + TS0710_MAX_HDR_SIZE + ACK_SPACE)	/* For BP UART problem: ACK_SPACE  */

#define TS0710MUX_MAX_TOTAL_FRAME_SIZE (DEF_TS0710_MTU + TS0710_MAX_HDR_SIZE + FLAG_SIZE)
#define TS0710MUX_MAX_CHARS_IN_BUF 65535
#define TS0710MUX_THROTTLE_THRESHOLD DEF_TS0710_MTU

#define TEST_PATTERN_SIZE 250

#define CMDTAG 0x55
#define DATATAG 0xAA

#define ACK 0x4F		/*For BP UART problem */


#define SLIDE_BP_SEQ_OFFSET 1	/*offset from start flag */
#ifdef LGE_KERNEL_MUX
#define SEQ_FIELD_SIZE 0
#else
#define SEQ_FIELD_SIZE 1
#endif

#define ADDRESS_FIELD_OFFSET (1 + SEQ_FIELD_SIZE)	/*offset from start flag */

#ifndef UNUSED_PARAM
#define UNUSED_PARAM(v) (void)(v)
#endif

#define BUF_BUSY 0

#define RECV_RUNNING 0

#define MUX_INVALID(x) ( (x < 0) || (x >= NR_MUXS) )
#define MUX_ALL_STOPPED(x) (x->dlci[0].state == FLOW_STOPPED)
#define MUX_STOPPED(x,n) (x->dlci[n].state == FLOW_STOPPED)
#define MUX_CONNECTED(x,n) (x->dlci[n].state == CONNECTED)
#define MUX_STOPPED_FULL(x,n) (MUX_ALL_STOPPED(x) || MUX_STOPPED(x,n))
#define MUX_USABLE(x,n) (MUX_CONNECTED(x,n) && !MUX_STOPPED_FULL(x,n))

static void fcs_init(void);

static void send_sabm(ts0710_con * ts0710, __u8 dlci);
static void send_dm(ts0710_con * ts0710, __u8 dlci);
static void send_disc(ts0710_con * ts0710, __u8 dlci);
static void send_ua(ts0710_con * ts0710, __u8 dlci);
static void send_pn_msg(ts0710_con * ts0710, __u8 prior, __u32 frame_size,
		       __u8 credit_flow, __u8 credits, __u8 dlci, __u8 cr);

static void send_nsc_msg(ts0710_con * ts0710, mcc_type cmd, __u8 cr);
static void mux_send_uih(ts0710_con * ts0710, __u8 cr,__u8 type, __u8 *data, int len);
static void ts0710_fcoff_msg(ts0710_con * ts0710, __u8 cr);
static void ts0710_fcon_msg(ts0710_con * ts0710, __u8 cr);
static void send_ack(ts0710_con * ts0710, __u8 seq_num);
