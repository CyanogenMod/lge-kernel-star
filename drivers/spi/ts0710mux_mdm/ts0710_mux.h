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

/* Special ioctl() upon a MUX device file for hanging up a call */
#define TS0710MUX_IO_MSC_HANGUP 0x54F0

/* Special ioctl() upon a MUX device file for MUX loopback test */
#define TS0710MUX_IO_TEST_CMD 0x54F1

/* Special Error code might be return from write() to a MUX device file  */
#define EDISCONNECTED 900	/* Logical data link is disconnected */

/* Special Error code might be return from open() to a MUX device file  */
#define EREJECTED 901		/* Logical data link connection request is rejected */

#define TS0710MUX_MAJOR 0  /* Use dynamic allocation*/

#define TS0710MUX_MINOR_START 0

#define TS0710MUX_SABM_TRIES 11
#define TS0710MUX_DISC_TRIES 3



#define TS0710MUX_TIME_OUT 250	/* 250ms, for BP UART hardware flow control AP UART  */

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
#define TS0710MUX_MAX_RECEIVE_ROOM 65536
#define TS0710MUX_THROTTLE_THRESHOLD DEF_TS0710_MTU

#define TEST_PATTERN_SIZE 250

#define CMDTAG 0x55
#define DATATAG 0xAA

#define ACK 0x4F		/*For BP UART problem */


#define SLIDE_BP_SEQ_OFFSET 1	/*offset from start flag */

#define SEQ_FIELD_SIZE 0
#define FRAME_TAIL_SIZE 2

#define ADDRESS_FIELD_OFFSET (1 + SEQ_FIELD_SIZE)	/*offset from start flag */

#ifndef UNUSED_PARAM
#define UNUSED_PARAM(v) (void)(v)
#endif

#define BUF_BUSY 0

#define RECV_RUNNING 0

#define MUX_INVALID(x) ( (x < 0) || (x >= TS0710_MAX_CHANNELS) )
#define MUX_ALL_STOPPED(x) (x->dlci[0].state == FLOW_STOPPED)
#define MUX_STOPPED(x,n) (x->dlci[n].state == FLOW_STOPPED)
#define MUX_CONNECTED(x,n) (x->dlci[n].state == CONNECTED)
#define MUX_STOPPED_FULL(x,n) (MUX_ALL_STOPPED(x) || MUX_STOPPED(x,n))
#define MUX_USABLE(x,n) (MUX_CONNECTED(x,n) && !MUX_STOPPED_FULL(x,n))

static inline int mux_short_frame_size(short_frame_head *h)
{
    return sizeof(short_frame_head) + h->length.len + SEQ_FIELD_SIZE + FCS_SIZE + FLAG_SIZE;
}
    
static inline int mux_long_frame_size(long_frame_head *h)
{
    return sizeof(long_frame_head) + GET_LONG_LENGTH(h->length) + SEQ_FIELD_SIZE + FCS_SIZE + FLAG_SIZE;
}

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

