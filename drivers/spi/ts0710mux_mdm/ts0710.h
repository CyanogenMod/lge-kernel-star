/*
 * File: ts0710.h
 *
 * Portions derived from rfcomm.c, original header as follows:
 *
 * Copyright (C) 2000, 2001  Axis Communications AB
 *
 * Author: Mats Friden <mats.friden@axis.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 *
 * Exceptionally, Axis Communications AB grants discretionary and
 * conditional permissions for additional use of the text contained
 * in the company's release of the AXIS OpenBT Stack under the
 * provisions set forth hereunder.
 *
 * Provided that, if you use the AXIS OpenBT Stack with other files,
 * that do not implement functionality as specified in the Bluetooth
 * System specification, to produce an executable, this does not by
 * itself cause the resulting executable to be covered by the GNU
 * General Public License. Your use of that executable is in no way
 * restricted on account of using the AXIS OpenBT Stack code with it.
 *
 * This exception does not however invalidate any other reasons why
 * the executable file might be covered by the provisions of the GNU
 * General Public License.
 *
 */
/*
 * Copyright (C) 2002  Motorola
 *
 *  07/28/2002  Initial version based on rfcomm.c
 *  11/18/2002  Modified
 */

#include <linux/module.h>

#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/fcntl.h>
#include <linux/string.h>
#include <linux/major.h>
#include <linux/mm.h>
#include <linux/init.h>

#include <asm/uaccess.h>
#include <asm/system.h>
#include <asm/bitops.h>

#include <asm/byteorder.h>
#include <asm/types.h>

#define TS0710_MAX_CHANNELS 32

#define SET_PF(ctr) ((ctr) | (1 << 4))
#define CLR_PF(ctr) ((ctr) & 0xef)
#define GET_PF(ctr) (((ctr) >> 4) & 0x1)

#define GET_PN_MSG_FRAME_SIZE(pn) ( ((pn)->frame_sizeh << 8) | ((pn)->frame_sizel))
#define SET_PN_MSG_FRAME_SIZE(pn, size) ({ (pn)->frame_sizel = (size) & 0xff; \
                                           (pn)->frame_sizeh = (size) >> 8; })

#define GET_LONG_LENGTH(a) ( ((a).h_len << 7) | ((a).l_len) )
#define SET_LONG_LENGTH(a, length) ({ (a).ea = 0; \
                                      (a).l_len = length & 0x7F; \
                                      (a).h_len = (length >> 7) & 0xFF; })

#define SHORT_CRC_CHECK 3
#define LONG_CRC_CHECK 4

/* FIXME: Should thsi one be define here? */
#define SHORT_PAYLOAD_SIZE 127

#define EA 1
#define FCS_SIZE 1
#define FLAG_SIZE 2

#define TS0710_MAX_HDR_SIZE 5

//                                                 
#if defined(CONFIG_LU6500) || defined (CONFIG_LU8800)
#define TS0710_IP_RAW_DATA
#endif
//                                                 

#ifdef TS0710_IP_RAW_DATA
#define MAX_FRAME_SIZE 2617
#else
#define MAX_FRAME_SIZE 1516
#endif

#define DEF_TS0710_MTU (MAX_FRAME_SIZE - (TS0710_MAX_HDR_SIZE+2))

#define TS0710_BASIC_FLAG 0xF9
/* the control field */
#define SABM 0x2f
#define SABM_SIZE 4
#define UA 0x63
#define UA_SIZE 4
#define DM 0x0f
#define DISC 0x43
#define UIH 0xef

/* the type field in a multiplexer command packet */
#define TEST 0x8
#define FCON 0x28
#define FCOFF 0x18
#define MSC 0x38
#define RPN 0x24
#define RLS 0x14
#define PN 0x20
#define NSC 0x4

/* V.24 modem control signals */
#define FC 0x2
#define RTC 0x4
#define RTR 0x8
#define IC 0x40
#define DV 0x80

#define CTRL_CHAN 0     /* The control channel is defined as DLCI 0 */
#define MCC_CMD 1       /* Multiplexer command cr */
#define MCC_RSP 0       /* Multiplexer response cr */

#ifdef __LITTLE_ENDIAN_BITFIELD

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
} __attribute__ ((packed)) long_frame_head;

typedef struct {
    long_frame_head h;
    __u8 data[0];
} __attribute__ ((packed)) long_frame;

/* Typedefinitions for structures used for the multiplexer commands */
typedef struct {
    __u8 ea:1;
    __u8 cr:1;
    __u8 type:6;
} __attribute__ ((packed)) mcc_type;

typedef struct {
    mcc_type type;
    short_length length;
    __u8 value[0];
} __attribute__ ((packed)) mcc_short_frame_head;

typedef struct {
    mcc_short_frame_head h;
    __u8 value[0];
} __attribute__ ((packed)) mcc_short_frame;



/* MSC-command */
typedef struct {
    __u8 ea:1;
    __u8 fc:1;
    __u8 rtc:1;
    __u8 rtr:1;
    __u8 reserved:2;
    __u8 ic:1;
    __u8 dv:1;
} __attribute__ ((packed)) v24_sigs;

typedef struct {
    __u8 ea:1;
    __u8 b1:1;
    __u8 b2:1;
    __u8 b3:1;
    __u8 len:4;
} __attribute__ ((packed)) brk_sigs;


typedef struct {
    address_field dlci;
    __u8 v24_sigs;
} __attribute__ ((packed)) msc_t;



/* PN-command */
typedef struct {
    short_frame_head s_head;
    mcc_short_frame_head mcc_s_head;
    __u8 dlci:6;
    __u8 res1:2;
    __u8 frame_type:4;
    __u8 credit_flow:4;
    __u8 prior:6;
    __u8 res2:2;
    __u8 ack_timer;
    __u8 frame_sizel;
    __u8 frame_sizeh;
    __u8 max_nbrof_retrans;
    __u8 credits;
    __u8 fcs;
} __attribute__ ((packed)) pn_msg;

/* PN-command */
typedef struct {
    __u8 dlci:6;
    __u8 res1:2;
    __u8 frame_type:4;
    __u8 credit_flow:4;
    __u8 prior:6;
    __u8 res2:2;
    __u8 ack_timer;
    __u8 frame_sizel;
    __u8 frame_sizeh;
    __u8 max_nbrof_retrans;
    __u8 credits;
    __u8 fcs;
} __attribute__ ((packed)) pn_t;

/* NSC-command */
typedef struct {
    short_frame_head s_head;
    mcc_short_frame_head mcc_s_head;
    mcc_type command_type;
    __u8 fcs;
} __attribute__ ((packed)) nsc_msg;

#else
#error Only littel-endianess supported now!
#endif

#define REJECTED 0
#define DISCONNECTED (1<<0)
#define CONNECTING  (1<<1)
#define NEGOTIATING  (1<<2)
#define CONNECTED  (1<<3)
#define DISCONNECTING (1<<4)
#define FLOW_STOPPED (1<<5)

enum ts0710_events {
    CONNECT_IND,
    CONNECT_CFM,
    DISCONN_CFM
};

typedef struct
{
    struct work_struct          work;
    int                         dlci;

} mux_rx_work;

typedef struct
{
    bool                        flow_disabled;
    struct workqueue_struct     *workqueue;
    mux_rx_work                 work;
    struct semaphore            rx_completion;

} mux_rx_struct;

typedef struct
{
    int                         frame_count;
    int                         chars_in_buffer;
    volatile int                wakeup_flag;
    struct semaphore            write_sema;

} mux_tx_struct;

typedef struct
{
    int                         dlci;
    volatile __u8               state;
    volatile __u16              mtu;
    wait_queue_head_t           open_wait;
    wait_queue_head_t           close_wait;
    volatile __u8               priority;

    mux_rx_struct               rx;
    mux_tx_struct               tx;

} dlci_struct;


/* user space interfaces */
typedef struct {
    volatile __u8 initiator;
    volatile __u8 c_dlci;
    volatile __u16 mtu;
    volatile __u8 be_testing;
    volatile __u32 test_errs;
    wait_queue_head_t test_wait;

    dlci_struct dlci[TS0710_MAX_CHANNELS];
} ts0710_con;

#define valid_dlci(x) ( (x>0) && (x<TS0710_MAX_CHANNELS) )
#define valid_line(x) ( (x>0) && (x<NR_MUXS) )

#define MUX_EA          1
#define MUX_BASIC_FLAG_SEQ  0xf9
#define MUX_ADVANCED_FLAG_SEQ   0x7e
#define MUX_CONTROL_ESCAPE  0x7d

#define MUX_SHORT_FRAME_SIZE(short_frame)

#define TS0710_PRINTK(fmt, arg...) printk(KERN_INFO "TS07.10: " fmt, ## arg)  

enum mux_frametype {
    MUX_SABM    = 0x2f,
    MUX_UA      = 0x63,
    MUX_DM      = 0x0f,
    MUX_DISC    = 0x43,
    MUX_UIH     = 0xef,
    MUX_UI      = 0x03,
    MUX_PN      = 0x20,
};

