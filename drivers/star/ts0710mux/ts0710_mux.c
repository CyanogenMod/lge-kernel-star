/*
 * Motorola TS0710 driver.
 *
 * Copyright (C) 2002-2004  Motorola
 * Copyright (C) 2006 Harald Welte <laforge@openezx.org>
 * Copyright (C) 2009 Ilya Petrov <ilya.muromec@gmail.com>
 * Copyright (C) 2009 Daniel Ribeiro <drwyrm@gmail.com>
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
#define LGE_FROYO_BSP
#include <linux/module.h>
#include <linux/types.h>

#include <linux/kernel.h>
#include <linux/proc_fs.h>

#include <linux/serial.h>

#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/fcntl.h>
#include <linux/string.h>
#include <linux/major.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/poll.h>

#include <asm/uaccess.h>
#include <asm/system.h>
#include <asm/bitops.h>
#include <asm/mach-types.h>

/*
	RECOVERY_MODE
		description : This mode is used to recover the at command pending state
				     Step.1 : CP reset
				     Step.2 : Close all mux channels
				     Step.3 : Wait until CP is rebooting for 5 seconds
*/
#define RECOVERY_MODE
#ifdef RECOVERY_MODE
#include "nvos.h"
#include "nvcommon.h"
#include "nvodm_services.h"
#include "nvodm_query_discovery.h"

static NvOdmServicesGpioHandle hGpio = NULL;
static NvOdmGpioPinHandle hPin = NULL; 
#endif


#ifdef LGE_KERNEL_MUX
#include <linux/spinlock.h>
#include <linux/semaphore.h>
#include <linux/mutex.h>
#include <linux/kthread.h>
#define TS0710MAX_CHANNELS 32
#define TS0710MAX_PRIORITY_NUMBER 64
#else
#define TS0710MAX_CHANNELS 23
#include "ts0710_mux_usb.h"
#endif

#include "ts0710.h"
#include "ts0710_mux.h"

/*
	macro : ENABLE_MUX_WAKE_LOCK
	description : This macro enables wakelock if ts_ldisc_close function is called because of RILD exit
*/
#define ENABLE_MUX_WAKE_LOCK
#ifdef ENABLE_MUX_WAKE_LOCK

#include <linux/wakelock.h>
#define MUX_WAKELOCK_TIME		(30*HZ)
struct wake_lock	 	s_wake_lock;
static unsigned int	s_wake_lock_flag;
#endif

#define LOCK_T          spinlock_t
#define CREATELOCK(_l)  spin_lock_init(&(_l))
#define DELETELOCK(_l)
#define LOCK(_l)        spin_lock_bh(&(_l))
#define WAIT_UNLOCK(_l)	spin_unlock_wait(&(_l))
#define UNLOCK(_l)      spin_unlock_bh(&(_l))
#define ATOMIC(_l,_f)   spin_lock_irqsave(&(_l),(_f))
#define UNATOMIC(_l,_f) spin_unlock_irqrestore(&(_l),(_f))
static int NR_MUXS;

static struct tty_struct *ipc_tty;

typedef struct {
	u8 cmdtty;
	u8 datatty;
} dlci_tty;

static u8 *iscmdtty;

typedef struct {
	volatile u8 buf[TS0710MUX_SEND_BUF_SIZE];
	volatile u8 *frame;
	unsigned long flags;
	volatile u16 length;
	volatile u8 filled;
	volatile u8 dummy;	/* Allignment to 4*n bytes */
} mux_send_struct;

#define MAX_PACKET_SIZE 2048
#ifdef LGE_KERNEL_MUX
struct mux_data {
	enum {OUT_OF_FRAME, INSIDE_FRAME_HEADER, INSIDE_FRAME_BODY } state;
	size_t chunk_size;
    int copy_index;
    int framelen;
	unsigned char chunk[MAX_FRAME_SIZE];
};
#else
struct mux_data {
	enum {INSIDE_PACKET, OUT_OF_PACKET} state;
	size_t chunk_size;
	unsigned char chunk[MAX_PACKET_SIZE];
};
#endif
/* Bit number in flags of mux_send_struct */

struct mux_recv_packet_tag {
	u8 *data;
	u32 length;
	struct mux_recv_packet_tag *next;
};
typedef struct mux_recv_packet_tag mux_recv_packet;

struct mux_recv_struct_tag {
	u8 data[TS0710MUX_RECV_BUF_SIZE];
	u32 length;
	u32 total;
	mux_recv_packet *mux_packet;
	struct mux_recv_struct_tag *next;
	int no_tty;
	volatile u8 post_unthrottle;
};
typedef struct mux_recv_struct_tag mux_recv_struct;

static unsigned long mux_recv_flags = 0;

static mux_send_struct *mux_send_info[TS0710MAX_CHANNELS];
static volatile u8 mux_send_info_flags[TS0710MAX_CHANNELS];
static volatile u8 mux_send_info_idx = TS0710MAX_CHANNELS;

static mux_recv_struct *mux_recv_info[TS0710MAX_CHANNELS];
static volatile u8 mux_recv_info_flags[TS0710MAX_CHANNELS];
static mux_recv_struct *mux_recv_queue = NULL;

/* Local for 2.6? */
static struct tty_driver *mux_driver;

static struct tty_struct *mux_table[TS0710MAX_CHANNELS];
static struct ktermios *mux_termios[TS0710MAX_CHANNELS];
static struct ktermios *mux_termios_locked[TS0710MAX_CHANNELS];
static volatile short int mux_tty[TS0710MAX_CHANNELS];
static volatile struct file * mux_filp[TS0710MAX_CHANNELS];

#ifdef min
#undef min
#define min(a, b)	((a) < (b) ? (a) : (b))
#endif

static ts0710_con ts0710_connection;

#ifdef LGE_KERNEL_MUX
static DECLARE_MUTEX(spi_write_sema); /* use semaphore to synchronize different threads*/
static struct semaphore spi_write_data_sema[33];

struct spi_data_recived_struct {
    struct tty_struct *tty; 
    const u8 *data; 
    char *flags; 
    int size;
    int updated;
};
#define MAX_WAITING_FRAMES 50
struct spi_data_send_struct;
struct spi_data_send_struct {
    struct spi_data_send_struct *next;
    u8 *data;
    int size;
};

static LOCK_T frame_nodes_lock = SPIN_LOCK_UNLOCKED;
static unsigned long lock_flag ;
static struct spi_data_send_struct frame_node[MAX_WAITING_FRAMES];
static struct spi_data_send_struct *frame_to_send[TS0710MAX_PRIORITY_NUMBER];          
static struct spi_data_recived_struct spi_data_recieved;
static struct task_struct *task;
static struct task_struct *write_task;
static u8 default_priority_table[] = {
 0,                      /* multiplexer control channel */
 7, 7, 7, 7, 7, 7, 7,    /* 1-7 DLC   */
15,15,15,15,15,15,15,15, /* 8-15 DLC  */
23,23,23,23,23,23,23,23, /* 16-23 DLC */
31,31,31,31,31,31,31,31, /* 24-31 DLC */
39,39,39,39,39,39,39,39, /* 32-39 DLC */
47,47,47,47,47,47,47,47, /* 40-47 DLC */
55,55,55,55,55,55,55,55, /* 48-55 DLC */
61,61,61,61,61,61,       /* 56-61 DLC */
63,63                    /* 62-63 DLC */
};


static int max_frame_usage = 0;
static void nodes_init(void)
{
    int i;
    spin_lock_irqsave(&frame_nodes_lock, lock_flag);
    for (i=0; i<MAX_WAITING_FRAMES;i++){
        frame_node[i].next = NULL;
        frame_node[i].data = NULL;
        frame_node[i].size = 0;
    }
    for (i=0; i<TS0710MAX_PRIORITY_NUMBER;i++){
        frame_to_send[i]=NULL;
    }
    spin_unlock_irqrestore(&frame_nodes_lock, lock_flag);
}

static int node_put_to_send(u8 dlci, u8 *data, int size)
{
    int i;
    int retval = 0; 
    int free_node = MAX_WAITING_FRAMES+1;
    struct spi_data_send_struct *new_frame;
//    u8 * buf;
    u8 priority;
    int frame_count = 0;

    priority = ts0710_connection.dlci[dlci].priority;    
//    buf = kmalloc(size, GFP_ATOMIC);
//    if (buf)
//         memcpy(buf, data, size);
//    else
//    {
//         printk("\nTS0710:node_put_to_send:buf null, return ENOMEM\r\n");
//         return -ENOMEM;
//    }

    spin_lock_irqsave(&frame_nodes_lock, lock_flag);
    for (i=0; i<MAX_WAITING_FRAMES;i++){
        if(frame_node[i].size == 0){
            free_node = i;
            if(free_node > max_frame_usage){
                max_frame_usage = free_node;
            }
            break;    
        }
    }
    if(free_node < MAX_WAITING_FRAMES){
        frame_node[free_node].next = NULL;
        frame_node[free_node].data = data;
        frame_node[free_node].size = size;
        retval = size;
        if(frame_to_send[priority] == NULL){
            frame_to_send[priority]= &frame_node[free_node];
        }
        else{
            new_frame = frame_to_send[priority];
            while (new_frame->next != NULL){
                new_frame = new_frame->next;
                frame_count++;
                if(frame_count>MAX_WAITING_FRAMES) {
                  printk("\n Never hit this case : frame_count(%d) > MAX_WAITING_FRAMES[%d]\r\n", frame_count, MAX_WAITING_FRAMES);
                  break;
                }
            }
            new_frame->next = &frame_node[free_node];
        }
    }
    spin_unlock_irqrestore(&frame_nodes_lock, lock_flag);

    if(free_node >= MAX_WAITING_FRAMES){
       	TS0710_DEBUG("\nTS0710 free_node[%d] < MAX_WAITING_FRAMES\r\n", free_node);
        kfree(data);
        retval = -ENOMEM; 
    }

    if(retval > 0 ){
        up(&spi_write_sema);
    }

    return retval;
} 

#endif

static void ts0710_reset_dlci(u8 j)
{
	if (j >= TS0710_MAX_CHN)
		return;

	ts0710_connection.dlci[j].state = DISCONNECTED;
	ts0710_connection.dlci[j].flow_control = 0;
	ts0710_connection.dlci[j].mtu = DEF_TS0710_MTU;
	ts0710_connection.dlci[j].initiated = 0;
	ts0710_connection.dlci[j].initiator = 0;
	init_waitqueue_head(&ts0710_connection.dlci[j].open_wait);
	init_waitqueue_head(&ts0710_connection.dlci[j].close_wait);
}

#ifdef LGE_KERNEL_MUX
static void ts0710_reset_dlci_priority()
{
	u8 j;

	for (j = 0; j < TS0710_MAX_CHN; j++)
		ts0710_connection.dlci[j].priority = default_priority_table[j];
}
#endif

static void ts0710_reset_con(void)
{
	u8 j;

	ts0710_connection.initiator = 0;
	ts0710_connection.mtu = DEF_TS0710_MTU + TS0710_MAX_HDR_SIZE;
	ts0710_connection.be_testing = 0;
	ts0710_connection.test_errs = 0;
	init_waitqueue_head(&ts0710_connection.test_wait);

#ifdef LGE_KERNEL_MUX
	ts0710_reset_dlci_priority();
#endif

	for (j = 0; j < TS0710_MAX_CHN; j++)
		ts0710_reset_dlci(j);
}

static void ts0710_init(void)
{
	fcs_init();

	ts0710_reset_con();
}

static void ts0710_upon_disconnect(void)
{
	ts0710_con *ts0710 = &ts0710_connection;
	u8 j;

	for (j = 0; j < TS0710_MAX_CHN; j++) {
		ts0710->dlci[j].state = DISCONNECTED;
		wake_up_interruptible(&ts0710->dlci[j].open_wait);
		wake_up_interruptible(&ts0710->dlci[j].close_wait);
	}
	ts0710->be_testing = 0;
	wake_up_interruptible(&ts0710->test_wait);
	ts0710_reset_con();
}

/* Sending packet functions */

/* See TS 07.10's Section 5.2.1.6 */
static u_int8_t fcs_table[0x100];
static void fcs_init(void)
{
	int i, bit;
	u8 reg, nreg;

	for (i = 0; i < 0x100; i++) {
		for (reg = bit = 0; bit < 8; bit++, reg = nreg) {
			nreg = reg >> 1;
			if (((i >> bit) ^ reg) & 1)
				nreg ^= 0xe0;	/* x^8 + x^2 + x + 1 */
		}

		fcs_table[i] = reg;
	}
}

/* Computes the FCS checksum for a chunk of data.  */
static u_int8_t mux_fcs_compute(const u8 payload[], int len)
{
	u8 gen_reg;

	gen_reg = ~0;
	while (len--)
		gen_reg = fcs_table[gen_reg ^ *(payload++)];

	return ~gen_reg;
}

/* Returns 1 if the given chunk of data has a correct FCS appended.  */
static int mux_fcs_check(const u8 payload[], int len)
{
	return mux_fcs_compute(payload, len + 1) == 0xcf;
}

/* See TS 07.10's Section 5.2.1 */
static int mux_send_frame(u8 dlci, int initiator,
		enum mux_frametype frametype, const u8 data[], int len)
{
	int pos = 0;
	int pf, crc_len, res;
	int cr = initiator & 0x1;
  u_int8_t fcs;

  u8 *framebuf = kmalloc(2048, GFP_ATOMIC);

  if (!framebuf) {
    printk(KERN_ERR "mux_send_frame:: Alloc Failed \n");
    return -1;
  }

	/* FIXME: bitmask? */
	switch (frametype) {
		case MUX_UIH:
		case ACK:
			pf = 0;
			crc_len = len;
			break;
		case MUX_UI:
			pf = 0;
			crc_len = 0;
			break;
		default:
			pf = 1;
			crc_len = 0;
	}

	/* if (dlc->muxer->option == mux_option_basic) */
		framebuf[pos++] = MUX_BASIC_FLAG_SEQ;
	/* else
		framebuf[pos++] = MUX_ADVANCED_FLAG_SEQ; */

	/* Address field.  */
	framebuf[pos++] = MUX_EA | (cr << 1) | (dlci << 2);

	/* Control field.  */
	framebuf[pos++] = frametype | (pf << 4);

	/* Length indicator.  */
	/* if (dlc->muxer->option == mux_option_basic) { */
		if (len & ~0x7f) {
			framebuf[pos ++] = 0 | ((len & 0x7f) << 1);
			framebuf[pos ++] = len >> 7;
		} else
			framebuf[pos ++] = 1 | (len << 1);
	/* } */

	/* Information field.  */
	if (len)
		memcpy(&framebuf[pos], data, len);
	pos += len;

    fcs = mux_fcs_compute(framebuf + 1, (pos - 1 - crc_len));
     framebuf[pos++] = fcs;

	/*if (dlc->muxer->option == mux_option_advanced)
		pos = mux_frame_escape(framebuf + 1, pos - 1) + 1;

	if (dlc->muxer->option == mux_option_basic) */
		framebuf[pos ++] = MUX_BASIC_FLAG_SEQ;
	/* else
		framebuf[pos ++] = MUX_ADVANCED_FLAG_SEQ; */
#ifdef LGE_KERNEL_MUX
    res = node_put_to_send( dlci, framebuf, pos);
    if(res == -ENOMEM){
      return res;
    }
#else    
	res = ipc_tty->ops->write(ipc_tty, framebuf, pos);
     kfree(framebuf);
#endif

	if (res != pos) {
		TS0710_PRINTK("mux_send_frame error %d\n", res);
		return -1;
	}

#ifdef LGE_KERNEL_MUX
	/* Function should return number of sent information bytes. 
	Therefore need to decrease ret variable */
	if (len & ~0x7f) {
		/* Length indicator have two octet size. */
		res -= 7;
	} else {
		res -= 6;
	}
#endif

	return res;
}

/* Creates a UA packet and puts it at the beginning of the pkt pointer */

static void send_ua(ts0710_con * ts0710, u8 dlci)
{
	mux_send_frame(dlci, !ts0710->initiator, MUX_UA, 0, 0);
}

/* Creates a DM packet and puts it at the beginning of the pkt pointer */

static void send_dm(ts0710_con * ts0710, u8 dlci)
{
	mux_send_frame(dlci, !ts0710->initiator, MUX_DM, 0, 0);
}

static void send_sabm(ts0710_con * ts0710, u8 dlci)
{
	mux_send_frame(dlci, ts0710->initiator, MUX_SABM, 0, 0);
}

static void send_disc(ts0710_con * ts0710, u8 dlci)
{
	mux_send_frame(dlci, !ts0710->initiator, MUX_DISC, 0, 0);
}

/* Multiplexer command packets functions */

/* Turns on the ts0710 flow control */

static void ts0710_fcon_msg(ts0710_con * ts0710, u8 cr)
{
	mux_send_uih(ts0710, cr, FCON, 0, 0);
}

/* Turns off the ts0710 flow control */

static void ts0710_fcoff_msg(ts0710_con * ts0710, u8 cr)
{
	mux_send_uih(ts0710, cr, FCOFF, 0, 0);
}


/* Sends an PN-messages and sets the not negotiable parameters to their
   default values in ts0710 */

static void send_pn_msg(ts0710_con * ts0710, u8 prior, u32 frame_size,
		       u8 credit_flow, u8 credits, u8 dlci, u8 cr)
{
	u8 data[8];
	pn_t *send = (pn_t *)data;

	send->res1 = 0;
	send->res2 = 0;
	send->dlci = dlci;
	send->frame_type = 0;
	send->credit_flow = credit_flow;
	send->prior = prior;
	send->ack_timer = 0;

	send->frame_sizel = frame_size & 0xFF;
	send->frame_sizeh = frame_size >> 8;

	send->credits = credits;
	send->max_nbrof_retrans = 0;

	mux_send_uih(ts0710, cr, PN, data, 8);
}

/* Send a Not supported command - command, which needs 3 bytes */

static void send_nsc_msg(ts0710_con * ts0710, mcc_type cmd, u8 cr)
{
	mux_send_uih(ts0710, cr, NSC, (u8 *) &cmd, 1);
}

static void mux_send_uih(ts0710_con * ts0710, u8 cr, u8 type, u8 *data, int len)
{
     u8 *send = kmalloc(len + 2, GFP_ATOMIC);

	mcc_short_frame_head *head = (mcc_short_frame_head *)send;
	head->type.ea = 1;
	head->type.cr = cr;
	head->type.type = type;
	head->length.ea = 1;
	head->length.len = len;

	if (len)
		memcpy(send + 2, data, len);

	mux_send_frame(CTRL_CHAN, ts0710->initiator, MUX_UIH, send, len + 2);

	kfree(send);
}

static int mux_send_uih_data(ts0710_con * ts0710, u8 dlci, u8 *data, int len)
{
	int ret =0;
#ifdef LGE_KERNEL_MUX
    u8 *send = NULL;
	if (len){
        send = kmalloc(len + 1, GFP_ATOMIC);
        if(send != NULL){
            memcpy(send, data, len);
            ret = mux_send_frame(dlci, ts0710->initiator, MUX_UIH, send, len);
            kfree(send);
        }
    }
#else
	u8 *send = kmalloc(len + 2, GFP_ATOMIC);
	*send = CMDTAG;

	if (len)
		memcpy(send + 1, data, len);

	ret = mux_send_frame(dlci, ts0710->initiator, MUX_UIH, send, len + 1);

	kfree(send);
#endif    
	return ret;
}

static int ts0710_msc_msg(ts0710_con * ts0710, u8 value, u8 cr, u8 dlci)
{
	u8 buf[2];
	msc_t *send = (msc_t *)buf;

	send->dlci.ea = 1;
	send->dlci.cr = 1;
	send->dlci.d = dlci & 1;
	send->dlci.server_chn = (dlci >> 1) & 0x1f;

	send->v24_sigs = value;

	mux_send_uih(ts0710, cr, MSC, buf, 2);

	return 0;
}

/* Parses a multiplexer control channel packet */

void process_mcc(u8 * data, u32 len, ts0710_con * ts0710, int longpkt)
{
	mcc_short_frame *mcc_short_pkt;
	int j;

	if (longpkt)
		mcc_short_pkt = (mcc_short_frame *) (((long_frame *) data)->data);
	else
		mcc_short_pkt = (mcc_short_frame *) (((short_frame *) data)->data);


	switch (mcc_short_pkt->h.type.type) {

	case FCON:		/*Flow control on command */
		TS0710_PRINTK("MUX Received Flow control(all channels) on command\n");
		if (mcc_short_pkt->h.type.cr == MCC_CMD) {
			for (j = 0; j < TS0710_MAX_CHN; j++) {
				ts0710->dlci[j].state = CONNECTED;
				up(&spi_write_data_sema[j]);
				down_trylock(&spi_write_data_sema[j]);
			}
			ts0710_fcon_msg(ts0710, MCC_RSP);
		}
		break;

	case FCOFF:		/*Flow control off command */
		TS0710_PRINTK("MUX Received Flow control(all channels) off command\n");
		if (mcc_short_pkt->h.type.cr == MCC_CMD) {
			for (j = 0; j < TS0710_MAX_CHN; j++) {
				ts0710->dlci[j].state = FLOW_STOPPED;
			}
			ts0710_fcoff_msg(ts0710, MCC_RSP);
		}
		break;

	case MSC:		/*Modem status command */
		{
			u8 dlci;
			u8 v24_sigs;

			dlci = (mcc_short_pkt->value[0]) >> 2;
			v24_sigs = mcc_short_pkt->value[1];

			if ((ts0710->dlci[dlci].state != CONNECTED)
			    && (ts0710->dlci[dlci].state != FLOW_STOPPED)) {
				send_dm(ts0710, dlci);
				break;
			}

			if (mcc_short_pkt->h.type.cr == MCC_CMD) {
				TS0710_DEBUG("Received Modem status command\n");
				if ((v24_sigs & 2) && (ts0710->dlci[dlci].state == CONNECTED)) {
					TS0710_LOG ("MUX Received Flow off on dlci %d\n", dlci);
					ts0710->dlci[dlci].state = FLOW_STOPPED;
				} else if (MUX_STOPPED(ts0710,dlci)) {
					ts0710->dlci[dlci].state = CONNECTED;
					TS0710_LOG ("MUX Received Flow on on dlci %d\n", dlci);
					up(&spi_write_data_sema[dlci]);
					down_trylock(&spi_write_data_sema[dlci]);
				}

				ts0710_msc_msg(ts0710, v24_sigs, MCC_RSP, dlci);
			} else {
				TS0710_DEBUG("Received Modem status response\n");

				if (v24_sigs & 2) {
					TS0710_DEBUG("Flow stop accepted\n");
				}
			}
			break;
		}

	case PN:		/*DLC parameter negotiation */
		{
			u8 dlci;
			u16 frame_size;
			pn_msg *pn_pkt;

			pn_pkt = (pn_msg *) data;
			dlci = pn_pkt->dlci;
			frame_size = GET_PN_MSG_FRAME_SIZE(pn_pkt);
			TS0710_DEBUG("Received DLC parameter negotiation, PN\n");

			if (pn_pkt->mcc_s_head.type.cr == MCC_CMD) {
				TS0710_DEBUG("received PN command with:\n");
				TS0710_DEBUG("Frame size:%d\n", frame_size);

				frame_size = min(frame_size, ts0710->dlci[dlci].mtu);
				send_pn_msg(ts0710, pn_pkt->prior, frame_size, 0, 0, dlci, MCC_RSP);
				ts0710->dlci[dlci].mtu = frame_size;
				TS0710_DEBUG("process_mcc : mtu set to %d\n", ts0710->dlci[dlci].mtu);
			} else {
				TS0710_DEBUG("received PN response with:\n");
				TS0710_DEBUG("Frame size:%d\n", frame_size);

				frame_size = min(frame_size, ts0710->dlci[dlci].mtu);
				ts0710->dlci[dlci].mtu = frame_size;


				if (ts0710->dlci[dlci].state == NEGOTIATING) {
					ts0710->dlci[dlci].state = CONNECTING;
					wake_up_interruptible(&ts0710->dlci[dlci].open_wait);
				}
			}
			break;
		}

	case NSC:		/*Non supported command resonse */
		TS0710_LOG("MUX Received Non supported command response\n");
		break;

	default:		/*Non supported command received */
		TS0710_LOG("MUX Received a non supported command\n");
		send_nsc_msg(ts0710, mcc_short_pkt->h.type, MCC_RSP);
		break;
	}
}

static void free_mux_recv_struct(mux_recv_struct * recv_info)
{
	if (!recv_info)
		return;

	kfree(recv_info);
}

static inline void add_post_recv_queue(mux_recv_struct ** head,
				       mux_recv_struct * new_item)
{
	new_item->next = *head;
	*head = new_item;
}

#define MUX_CONTROL_BUG_FIX //by eunae.kim 2011-01-07
static void ts0710_flow_on(u8 dlci, ts0710_con * ts0710)
{
#if defined(MUX_CONTROL_BUG_FIX) /*  2011-01-07 */
           if (!((ts0710->dlci[0].state) & (CONNECTED | FLOW_STOPPED)))
                     return;

           if (!((ts0710->dlci[dlci].state) & (CONNECTED | FLOW_STOPPED)))
                     return;
#else
	if (!(ts0710->dlci[0].state) & (CONNECTED | FLOW_STOPPED))
		return;

	if (!(ts0710->dlci[dlci].state) & (CONNECTED | FLOW_STOPPED))
		return;
#endif

	if (!(ts0710->dlci[dlci].flow_control))
		return;

	ts0710_msc_msg(ts0710, EA | RTC | RTR | DV, MCC_CMD, dlci);

	ts0710->dlci[dlci].flow_control = 0;
}

static void ts0710_flow_off(struct tty_struct *tty, u8 dlci,
		ts0710_con * ts0710)
{
	int i;

	TS0710_PRINTK("flow off\n");

	if (test_and_set_bit(TTY_THROTTLED, &tty->flags))
		return;


	if ((ts0710->dlci[0].state != CONNECTED) && (ts0710->dlci[0].state != FLOW_STOPPED)) 
		return;

	if ((ts0710->dlci[dlci].state != CONNECTED)
			&& (ts0710->dlci[dlci].state != FLOW_STOPPED))
		return;

	if (ts0710->dlci[dlci].flow_control)
		return;

	for (i = 0; i < 3; i++) {
		if (ts0710_msc_msg(ts0710, EA | FC | RTC | RTR | DV, MCC_CMD, dlci) < 0) 
			continue;

		TS0710_LOG("\nMUX send Flow off on dlci %d\n", dlci);
		ts0710->dlci[dlci].flow_control = 1;
		break;
	}
}

void ts0710_recv_data_server(ts0710_con * ts0710, short_frame *short_pkt, int len)
{
	u8 be_connecting;
	u8 *uih_data_start;
	u32 uih_len;
	long_frame *long_pkt;

	switch (CLR_PF(short_pkt->h.control)) {
	case SABM:
		TS0710_DEBUG("SABM-packet received\n");
		TS0710_DEBUG("server channel == 0\n");
		ts0710->dlci[0].state = CONNECTED;

		TS0710_DEBUG("sending back UA - control channel\n");
		send_ua(ts0710, 0);
		wake_up_interruptible(&ts0710->dlci[0].open_wait);

		break;
	case UA:
		TS0710_DEBUG("UA packet received\n");

		TS0710_DEBUG("server channel == 0\n");

		if (ts0710->dlci[0].state == CONNECTING) {
			ts0710->dlci[0].state = CONNECTED;
			wake_up_interruptible(&ts0710->dlci[0].
						      open_wait);
		} else if (ts0710->dlci[0].state == DISCONNECTING) {
			ts0710_upon_disconnect();
		} else {
			TS0710_DEBUG(" Something wrong receiving UA packet\n");
		}

		break;
	case DM:
		TS0710_DEBUG("DM packet received\n");
		TS0710_DEBUG("server channel == 0\n");

		if (ts0710->dlci[0].state == CONNECTING) {
			be_connecting = 1;
		} else {
			be_connecting = 0;
		}
		ts0710_upon_disconnect();
		if (be_connecting) {
			ts0710->dlci[0].state = REJECTED;
		}

		break;
	case DISC:
		TS0710_DEBUG("DISC packet received\n");

		TS0710_DEBUG("server channel == 0\n");

		send_ua(ts0710, 0);
		TS0710_DEBUG("DISC, sending back UA\n");

		ts0710_upon_disconnect();

		break;

	case UIH:
		TS0710_DEBUG("UIH packet received\n");

		if (GET_PF(short_pkt->h.control)) {
			TS0710_LOG("MUX Error %s: UIH packet with P/F set, discard it!\n",
				__FUNCTION__);
			break;
		}

		/* FIXME: connected and flow */

		if ((short_pkt->h.length.ea) == 0) {
			TS0710_DEBUG("Long UIH packet received\n");
			long_pkt = (long_frame *) short_pkt;
			uih_len = GET_LONG_LENGTH(long_pkt->h.length);
			uih_data_start = long_pkt->h.data;
			TS0710_DEBUG("long packet length %d\n", uih_len);

		} else {
			TS0710_DEBUG("Short UIH pkt received\n");
			uih_len = short_pkt->h.length.len;
			uih_data_start = short_pkt->data;

		}
		TS0710_DEBUG("UIH on serv_channel 0\n");
		process_mcc((u8 *)short_pkt, len, ts0710,
				!(short_pkt->h.length.ea));

		break;

	default:
		TS0710_DEBUG("illegal packet\n");
		break;
	}

}

void process_uih(ts0710_con * ts0710, char *data, int len, u8 dlci) {

	short_frame *short_pkt = (short_frame *) data;
	long_frame *long_pkt;
	u8 *uih_data_start;
	u32 uih_len;
#ifndef LGE_KERNEL_MUX
	u8 tag;
#endif    
	u8 tty_idx;
	struct tty_struct *tty;
	u8 flow_control;
	mux_recv_struct *recv_info;
	int recv_room;


	if ((ts0710->dlci[dlci].state != CONNECTED)
			&& (ts0710->dlci[dlci].state != FLOW_STOPPED)) {
		TS0710_LOG("MUX Error %s: DLCI %d not connected, discard it!\n",
				__FUNCTION__, dlci);
		send_dm(ts0710, dlci);
		return;
	}

	if ((short_pkt->h.length.ea) == 0) {
		TS0710_DEBUG("Long UIH packet received\n");
		long_pkt = (long_frame *) data;
		uih_len = GET_LONG_LENGTH(long_pkt->h.length);
		uih_data_start = long_pkt->h.data;
		TS0710_DEBUG("long packet length %d\n", uih_len);

	} else {
		TS0710_DEBUG("Short UIH pkt received\n");
		uih_len = short_pkt->h.length.len;
		uih_data_start = short_pkt->data;

	}

	TS0710_DEBUG("UIH on channel %d\n", dlci);

	if (uih_len > ts0710->dlci[dlci].mtu) {
		TS0710_PRINTK
			("MUX Error:  DLCI:%d, uih_len:%d is bigger than mtu:%d, discard data!\n",
			 dlci, uih_len, ts0710->dlci[dlci].mtu);
		return;
	}
#ifndef LGE_KERNEL_MUX
	tag = *uih_data_start;
	uih_data_start++;
	uih_len--;

	if (!uih_len)
		return;

	printk("TS07.10:uih tag %x on dlci %d\n",tag,dlci);
#endif

	tty_idx = dlci;
	tty = mux_table[tty_idx];
	if ((!mux_tty[tty_idx]) || (!tty)) {
		TS0710_PRINTK
			("MUX: No application waiting for, discard it! /dev/mux%d\n",
			 tty_idx);
		return;

	}

	if ((!mux_recv_info_flags[tty_idx])
			|| (!mux_recv_info[tty_idx])) {
		TS0710_PRINTK
			("MUX Error: No mux_recv_info, discard it! /dev/mux%d\n",
			 tty_idx);
		return;
	}

	recv_info = mux_recv_info[tty_idx];
	if (recv_info->total > 8192) {
		TS0710_PRINTK
			("MUX : discard data for tty_idx:%d, recv_info->total > 8192 \n",
			 tty_idx);
		return;
	}

	flow_control = 0;
	recv_room = 65535;
    
	if (tty->receive_room)
		recv_room = tty->receive_room;

	if ((recv_room - (uih_len + recv_info->total)) <
			ts0710->dlci[dlci].mtu) {
		flow_control = 1;
	}
#ifdef LGE_FROYO_BSP
	tty->ldisc->ops->receive_buf(tty, uih_data_start, NULL, uih_len);
#else
	tty->ldisc.ops->receive_buf(tty, uih_data_start, NULL, uih_len);
#endif
	if (flow_control)
		tty_throttle(tty);	//20101127-3, , Teleca's patch for mux flow control
//		ts0710_flow_off(tty, dlci, ts0710);
}

void ts0710_recv_data(ts0710_con * ts0710, char *data, int len)
{
	short_frame *short_pkt;
	u8 dlci;

	short_pkt = (short_frame *) data;

	dlci = short_pkt->h.addr.server_chn << 1 | short_pkt->h.addr.d;
	if (!dlci)
		return ts0710_recv_data_server(ts0710, short_pkt, len);

	switch (CLR_PF(short_pkt->h.control)) {
	case SABM:
		TS0710_PRINTK("Incomming connect on channel %d\n", dlci);

		send_ua(ts0710, dlci);

		ts0710->dlci[dlci].state = CONNECTED;
		wake_up_interruptible(&ts0710->dlci[dlci].open_wait);

		break;
	case UA:
		TS0710_DEBUG("Incomming UA on channel %d\n", dlci);

		if (ts0710->dlci[dlci].state == CONNECTING) {
			ts0710->dlci[dlci].state = CONNECTED;
			wake_up_interruptible(&ts0710->dlci[dlci].
					open_wait);
		} else if (ts0710->dlci[dlci].state == DISCONNECTING) {
			ts0710->dlci[dlci].state = DISCONNECTED;
			wake_up_interruptible(&ts0710->dlci[dlci].
					open_wait);
			wake_up_interruptible(&ts0710->dlci[dlci].
					close_wait);
			ts0710_reset_dlci(dlci);
		}

		break;
	case DM:
		TS0710_DEBUG("Incomming DM on channel %d\n", dlci);

		if (ts0710->dlci[dlci].state == CONNECTING)
			ts0710->dlci[dlci].state = REJECTED;
		else
			ts0710->dlci[dlci].state = DISCONNECTED;

		wake_up_interruptible(&ts0710->dlci[dlci].open_wait);
		wake_up_interruptible(&ts0710->dlci[dlci].close_wait);
		ts0710_reset_dlci(dlci);


		break;
	case DISC:
		send_ua(ts0710, dlci);

		ts0710->dlci[dlci].state = DISCONNECTED;
		wake_up_interruptible(&ts0710->dlci[dlci].open_wait);
		wake_up_interruptible(&ts0710->dlci[dlci].close_wait);
		ts0710_reset_dlci(dlci);

		break;
	case UIH:
		process_uih(ts0710, data, len, dlci);

		break;
	default:
		TS0710_PRINTK("illegal packet\n");
        printk("\n TS0710:ts0710_recv_data:illegal packet for DLCi %d, hdr=%x \n",dlci,CLR_PF(short_pkt->h.control));
		break;
	}
}

/* Close ts0710 channel */
static void ts0710_close_channel(u8 dlci)
{
	ts0710_con *ts0710 = &ts0710_connection;
	int try;
	unsigned long t;

	TS0710_DEBUG("ts0710_disc_command on channel %d\n", dlci);

	if ((ts0710->dlci[dlci].state == DISCONNECTED)
	    || (ts0710->dlci[dlci].state == REJECTED)) {
		return;
	} else if (ts0710->dlci[dlci].state == DISCONNECTING) {
		/* Reentry */
		return;
	}

	ts0710->dlci[dlci].state = DISCONNECTING;
	try = 3;
	while (try--) {
		t = jiffies;
		send_disc(ts0710, dlci);
		interruptible_sleep_on_timeout(&ts0710->dlci[dlci].
					       close_wait,
					       TS0710MUX_TIME_OUT);
		if (ts0710->dlci[dlci].state == DISCONNECTED) {
			break;
		} else if (signal_pending(current)) {
			TS0710_PRINTK ("MUX DLCI %d Send DISC got signal!\n", dlci);
			break;
		} else if ((jiffies - t) >= TS0710MUX_TIME_OUT) {
			TS0710_PRINTK ("MUX DLCI %d Send DISC timeout!\n", dlci);
			continue;
		}
	}

	if (ts0710->dlci[dlci].state != DISCONNECTED) {
		if (dlci == 0) {	/* Control Channel */
			ts0710_upon_disconnect();
		} else {	/* Other Channel */
			ts0710->dlci[dlci].state = DISCONNECTED;
			wake_up_interruptible(&ts0710->dlci[dlci].
					      close_wait);
			ts0710_reset_dlci(dlci);
		}
	}
}

int ts0710_open_channel(u8 dlci)
{
	ts0710_con *ts0710 = &ts0710_connection;
	int try;
	int retval;
	unsigned long t;

	retval = -ENODEV;
	if (dlci == 0) {	/* control channel */
		if (ts0710->dlci[0].state & (CONNECTED | FLOW_STOPPED)) {
			return 0;
		} else if (ts0710->dlci[0].state == CONNECTING) {
			/* Reentry */
			TS0710_PRINTK("MUX DLCI: 0, reentry to open DLCI 0, pid: %d, %s !\n",
				current->pid, current->comm);
			try = 11;
			while (try--) {
				t = jiffies;
				interruptible_sleep_on_timeout(&ts0710->dlci[0].
							       open_wait,
							       TS0710MUX_TIME_OUT);
				if ((ts0710->dlci[0].state == CONNECTED)
				    || (ts0710->dlci[0].state ==
					FLOW_STOPPED)) {
					retval = 0;
					break;
				} else if (ts0710->dlci[0].state == REJECTED) {
					retval = -EREJECTED;
					break;
				} else if (ts0710->dlci[0].state ==
					   DISCONNECTED) {
					break;
				} else if (signal_pending(current)) {
					TS0710_PRINTK ("MUX DLCI:%d Wait for connecting got signal!\n", dlci);
					retval = -EAGAIN;
					break;
				} else if ((jiffies - t) >= TS0710MUX_TIME_OUT) {
					TS0710_PRINTK
					    ("MUX DLCI:%d Wait for connecting timeout!\n",
					     dlci);
					continue;
				} else if (ts0710->dlci[0].state == CONNECTING) {
					continue;
				}
			}

			if (ts0710->dlci[0].state == CONNECTING) {
				ts0710->dlci[0].state = DISCONNECTED;
			}
		} else if ((ts0710->dlci[0].state != DISCONNECTED)
			   && (ts0710->dlci[0].state != REJECTED)) {
			TS0710_PRINTK("MUX DLCI:%d state is invalid!\n", dlci);
			return retval;
		} else {
			ts0710->initiator = 1;
			ts0710->dlci[0].state = CONNECTING;
			ts0710->dlci[0].initiator = 1;

			t = jiffies;
			send_sabm(ts0710, 0);
			interruptible_sleep_on_timeout(&ts0710->dlci[0].
					open_wait,
					TS0710MUX_TIME_OUT);
			if ((ts0710->dlci[0].state == CONNECTED)
					|| (ts0710->dlci[0].state ==
						FLOW_STOPPED)) {
				retval = 0;
			} else if (ts0710->dlci[0].state == REJECTED) {
				TS0710_PRINTK
					("MUX DLCI:%d Send SABM got rejected!\n",
					 dlci);
				retval = -EREJECTED;
			} else if (signal_pending(current)) {
				TS0710_PRINTK ("MUX DLCI:%d Send SABM got signal!\n", dlci);
				retval = -EAGAIN;
			} else if ((jiffies - t) >= TS0710MUX_TIME_OUT) {
				TS0710_PRINTK
					("MUX DLCI:%d Send SABM timeout!\n",
					 dlci);
				retval = -ENODEV;
			}

			if (ts0710->dlci[0].state == CONNECTING) {
				ts0710->dlci[0].state = DISCONNECTED;
			}
			wake_up_interruptible(&ts0710->dlci[0].open_wait);
		}
	} else {		/* other channel */
		if ((ts0710->dlci[0].state != CONNECTED)
		    && (ts0710->dlci[0].state != FLOW_STOPPED)) {
			return retval;
		} else if ((ts0710->dlci[dlci].state == CONNECTED)
			   || (ts0710->dlci[dlci].state == FLOW_STOPPED)) {
			return 0;
		} else if ((ts0710->dlci[dlci].state == NEGOTIATING)
			   || (ts0710->dlci[dlci].state == CONNECTING)) {
			/* Reentry */

			t = jiffies;
			interruptible_sleep_on_timeout(&ts0710->
						       dlci[dlci].
						       open_wait,
						       TS0710MUX_TIME_OUT);
			if ((ts0710->dlci[dlci].state == CONNECTED)
			    || (ts0710->dlci[dlci].state ==
				FLOW_STOPPED)) {
				retval = 0;
			} else if (ts0710->dlci[dlci].state == REJECTED) {
				retval = -EREJECTED;
			} else if (signal_pending(current)) {
				retval = -EAGAIN;
			} 

			if ((ts0710->dlci[dlci].state == NEGOTIATING)
			    || (ts0710->dlci[dlci].state == CONNECTING)) {
				ts0710->dlci[dlci].state = DISCONNECTED;
			}
		} else if ((ts0710->dlci[dlci].state != DISCONNECTED)
			   && (ts0710->dlci[dlci].state != REJECTED)) {
			TS0710_PRINTK("MUX DLCI:%d state is invalid!\n", dlci);
			return retval;
		} else {
#ifdef LGE_KERNEL_MUX
            ts0710->dlci[dlci].state = CONNECTING;
#else
			ts0710->dlci[dlci].state = NEGOTIATING;
			ts0710->dlci[dlci].initiator = 1;

			t = jiffies;
			send_pn_msg(ts0710, 7, ts0710->dlci[dlci].mtu,
				    0, 0, dlci, 1);
			interruptible_sleep_on_timeout(&ts0710->
						       dlci[dlci].
						       open_wait,
						       TS0710MUX_TIME_OUT);
			if (signal_pending(current)) {
				TS0710_PRINTK
				    ("MUX DLCI:%d Send pn_msg got signal!\n",
				     dlci);
				retval = -EAGAIN;
			}
#endif
			if (ts0710->dlci[dlci].state == CONNECTING) {

				t = jiffies;
				send_sabm(ts0710, dlci);
				interruptible_sleep_on_timeout(&ts0710->
							       dlci
							       [dlci].
							       open_wait,
							       TS0710MUX_TIME_OUT);
				if ((ts0710->dlci[dlci].state ==
				     CONNECTED)
				    || (ts0710->dlci[dlci].state ==
					FLOW_STOPPED)) {
					retval = 0;
				} else if (ts0710->dlci[dlci].state ==
					   REJECTED) {
					TS0710_PRINTK
					    ("MUX DLCI:%d Send SABM got rejected!\n",
					     dlci);
					retval = -EREJECTED;
				} else if (signal_pending(current)) {
					TS0710_PRINTK
					    ("MUX DLCI:%d Send SABM got signal!\n",
					     dlci);
					retval = -EAGAIN;
				} 
			}

			if ((ts0710->dlci[dlci].state == NEGOTIATING)
			    || (ts0710->dlci[dlci].state == CONNECTING)) {
				ts0710->dlci[dlci].state = DISCONNECTED;
			}
			wake_up_interruptible(&ts0710->dlci[dlci].open_wait);
		}
	}
	return retval;
}

/****************************
 * TTY driver routines
*****************************/

static void mux_close(struct tty_struct *tty, struct file *filp)
{
	ts0710_con *ts0710 = &ts0710_connection;
	int line;
	u8 dlci;

	UNUSED_PARAM(filp);

	line = tty->index;
	if (MUX_INVALID(line))
		return;
	if (mux_tty[line] > 0)
		mux_tty[line]--; 

	dlci = line;
	if (mux_tty[line] == 0)
#ifdef LGE_KERNEL_MUX
/*Workaround Infineon modem - DLS opened once will never be closed*/
//20100915-1, , Fix VT can't recevie data from CP [START]
		if(dlci==12)
		{	//When VT close DLC 12 with flow off status, must send flow on status to CP
			//TS0710_PRINTK("%s - close channel[%d]\n", __FUNCTION__, dlci);
			ts0710_flow_on(dlci, &ts0710_connection);
		}
	TS0710_PRINTK("%s - close channel[%d]\n", __FUNCTION__, dlci);	
//20100915, , Fix VT can't recevie data from CP [END]
        return;
#else        
		ts0710_close_channel(dlci);
#endif    

     mux_filp[line] = NULL;

	if (mux_tty[line] != 0)
		return;

	if ((mux_send_info_flags[line]) && (mux_send_info[line]))
	{
		mux_send_info_flags[line] = 0;
		kfree(mux_send_info[line]);
		mux_send_info[line] = 0;
		TS0710_DEBUG("Free mux_send_info for /dev/mux%d\n", line);
	}

	if ((mux_recv_info_flags[line])
	    && (mux_recv_info[line])
	    && (mux_recv_info[line]->total == 0)) {
		mux_recv_info_flags[line] = 0;
		free_mux_recv_struct(mux_recv_info[line]);
		mux_recv_info[line] = 0;
		TS0710_DEBUG("Free mux_recv_info for /dev/mux%d\n", line);
	}
        tty_unthrottle(tty);	//20101127-3, , Teleca's patch for mux flow control
//	ts0710_flow_on(dlci, ts0710);

	wake_up_interruptible(&tty->read_wait);
	wake_up_interruptible(&tty->write_wait);
	tty->packet = 0;
}

static void mux_throttle(struct tty_struct *tty)
{
	ts0710_con *ts0710 = &ts0710_connection;
	int line;
	int i;
	u8 dlci;

	line = tty->index;
	if (MUX_INVALID(line))
		return;


	TS0710_DEBUG("Enter into %s, minor number is: %d\n", __FUNCTION__,
		     line);

	dlci = line;
	if ((ts0710->dlci[0].state != CONNECTED)
	    && (ts0710->dlci[0].state != FLOW_STOPPED)) {
		return;
	} else if ((ts0710->dlci[dlci].state != CONNECTED)
		   && (ts0710->dlci[dlci].state != FLOW_STOPPED)) {
		return;
	}

	if (ts0710->dlci[dlci].flow_control) {
		return;
	}

	for (i = 0; i < 3; i++) {
		if (ts0710_msc_msg
		    (ts0710, EA | FC | RTC | RTR | DV, MCC_CMD, dlci) < 0) {
			continue;
		} else {
			TS0710_LOG("MUX Send Flow off on dlci %d\n", dlci);
			ts0710->dlci[dlci].flow_control = 1;
			break;
		}
	}
}

static void mux_unthrottle(struct tty_struct *tty)
{
	ts0710_con *ts0710 = &ts0710_connection;
	int line;
	u8 dlci;
	mux_recv_struct *recv_info;

	line = tty->index;
	if (MUX_INVALID(line))
		return;

	if ((!mux_recv_info_flags[line]) || (!mux_recv_info[line])) {
		return;
	}

	TS0710_DEBUG("Enter into %s, minor number is: %d\n", __FUNCTION__,
		     line);

	recv_info = mux_recv_info[line];
	dlci = line;

	if (recv_info->total) {
		recv_info->post_unthrottle = 1;
		/* schedule_work(&post_recv_tqueue); */
	} else {
		ts0710_flow_on(dlci, ts0710);
	}
}

static int mux_chars_in_buffer(struct tty_struct *tty)
{
	ts0710_con *ts0710 = &ts0710_connection;
	int retval;
	int line;
	u8 dlci;
	mux_send_struct *send_info;

	retval = TS0710MUX_MAX_CHARS_IN_BUF;

	line = tty->index;
	if (MUX_INVALID(line)) {
		goto out;
	}

	dlci = line;

	if (! MUX_USABLE(ts0710, dlci))
		goto out;

	if (!(mux_send_info_flags[line])) {
		goto out;
	}
	send_info = mux_send_info[line];
	if (!send_info) {
		goto out;
	}
	if (send_info->filled) {
		goto out;
	}

	retval = 0;

out:
	return retval;
}

//#define LGE_DUMP_MUX_BIFFUER
#ifdef LGE_DUMP_MUX_BIFFUER
#define COL_SIZE 50
static void dump_mux_buffer(const unsigned char *txt, const unsigned char *buf, int count)
{
    char dump_buf_str[COL_SIZE+1];

    if (buf != NULL) 
    {
        int j = 0;
        char *cur_str = dump_buf_str;
        unsigned char ch;
        while((j < COL_SIZE) && (j < count))
        {
            ch = buf[j];
            if ((ch < 32) || (ch > 126))
            {
                *cur_str = '.';
            } else
            {
                *cur_str = ch;
            }
            cur_str++;
            j++;
        }
        *cur_str = 0;
        printk("%s:count:%d [%s]\n", txt, count, dump_buf_str);                        
    }
    else
    {
        printk("%s: buffer is NULL\n", txt);                 
    }
}
#else
#define dump_mux_buffer(...)
#endif

static int mux_write(struct tty_struct *tty,
		     const unsigned char *buf, int count)
{
	ts0710_con *ts0710 = &ts0710_connection;
	u8 dlci;
	int written = 0;
#ifdef LGE_KERNEL_MUX
	int frame_size = 0;
	int frame_written = 0;
#endif

	if (!count)
		return 0;

	dlci = tty->index;

	/*
	 * FIXME: split big packets into small one
	 * FIXME: support DATATAG
	 * */
#ifdef LGE_KERNEL_MUX
/* To remove Motorola's modem specific*/
  if(ts0710->dlci[dlci].state == FLOW_STOPPED){
    TS0710_DEBUG("TS0710 Write:Flow OFF state = %d \n",ts0710->dlci[dlci].state);
    if(mux_filp[dlci]->f_flags & O_NONBLOCK){
      TS0710_DEBUG("TS0710 Write: returning  EWOULDBLOCK for flow stopped channel\n");
      return -EWOULDBLOCK;
    }
    else {
      while (down_interruptible(&spi_write_data_sema[dlci])!=0){};
    }
  }

/* spliting big packets into small one */
	while(count)
	{
		frame_size = min(count, ts0710->dlci[dlci].mtu);

		frame_written = mux_send_uih_data(ts0710, dlci, (u8 *)(buf + written), frame_size);

          if(frame_written == -ENOMEM){
            if(mux_filp[dlci]->f_flags & O_NONBLOCK){
              return -EWOULDBLOCK;
            }
            else if(ts0710->dlci[dlci].state == FLOW_STOPPED){
              down(&spi_write_data_sema[dlci]);
            }
            else
              TS0710_DEBUG("\nTS0710:mux_write(), NOMEM, returning 0, should be stopped by mtx \n");
            }

            if(frame_written < 0){
              TS0710_DEBUG("\nTS0710:mux_write(): returning -1 \n");
			/* send frame error */
			return -1;
		}else{
			written += frame_written;
			count -= frame_written;
		}
	};
#else
	written = mux_send_uih_data(ts0710, dlci, (u8 *)buf, count) - 7;
#endif    
	return written;
}

static int mux_write_room(struct tty_struct *tty)
{
	ts0710_con *ts0710 = &ts0710_connection;
	int retval;
	int line;
	u8 dlci;
	mux_send_struct *send_info;

	retval = 0;

	line = tty->index;
	if (MUX_INVALID(line))
		goto out;

	dlci = line;
	if (ts0710->dlci[0].state == FLOW_STOPPED) {
		TS0710_DEBUG("Flow stopped on all channels, returning ZERO\n");
		goto out;
	} else if (ts0710->dlci[dlci].state == FLOW_STOPPED) {
		TS0710_DEBUG("Flow stopped, returning ZERO\n");
		goto out;
	} else if (ts0710->dlci[dlci].state != CONNECTED) {
		TS0710_DEBUG("DLCI %d not connected\n", dlci);
		goto out;
	}

	if (!(mux_send_info_flags[line]))
		goto out;

	send_info = mux_send_info[line];
	if (!send_info)
		goto out;

	if (send_info->filled)
		goto out;

	retval = ts0710->dlci[dlci].mtu - 1;

out:
 //printk("\nTS0710:write_room retval %d \n",retval);
	return retval;
}

static void mux_flush_buffer(struct tty_struct *tty)
{
	int line;

	line = tty->index;
	if (MUX_INVALID(line))
		return;


	TS0710_PRINTK("MUX %s: line is:%d\n", __FUNCTION__, line);

	if ((mux_send_info_flags[line])
	    && (mux_send_info[line])
	    && (mux_send_info[line]->filled)) {

		mux_send_info[line]->filled = 0;
	}

	wake_up_interruptible(&tty->write_wait);
#ifdef LGE_FROYO_BSP
	if ((tty->flags & (1 << TTY_DO_WRITE_WAKEUP)) &&
	    tty->ldisc->ops->write_wakeup) {
		tty->ldisc->ops->write_wakeup(tty);
#else
	if ((tty->flags & (1 << TTY_DO_WRITE_WAKEUP)) &&
	    tty->ldisc.ops->write_wakeup) {
		tty->ldisc.ops->write_wakeup(tty);
#endif
	}
}

static int mux_open(struct tty_struct *tty, struct file *filp)
{
	int retval;
	int line;
	u8 dlci;
	mux_send_struct *send_info;
	mux_recv_struct *recv_info;

	retval = -ENODEV;

	line = tty->index;
     mux_filp[line] = filp;

	if (!(ipc_tty && line))
		return -ENODEV;

    mux_tty[line]++;
	dlci = line;
	mux_table[line] = tty;

	/* Open server channel 0 first */
	if ((retval = ts0710_open_channel(0)) != 0) {
		TS0710_PRINTK("MUX: Can't connect server channel 0!\n");
		ts0710_init();

		mux_tty[line]--;
		goto out;
	}

	/* Allocate memory first. As soon as connection has been established, MUX may receive */
	if (mux_send_info_flags[line] == 0) {
		send_info =
		    (mux_send_struct *) kmalloc(sizeof(mux_send_struct),
						GFP_KERNEL);
		if (!send_info) {
			retval = -ENOMEM;

			mux_tty[line]--;
			goto out;
		}
		send_info->length = 0;
		send_info->flags = 0;
		send_info->filled = 0;
		mux_send_info[line] = send_info;
		mux_send_info_flags[line] = 1;
		TS0710_DEBUG("Allocate mux_send_info for /dev/pts%d", line);
	}

	if (mux_recv_info_flags[line] == 0) {
		recv_info =
		    (mux_recv_struct *) kmalloc(sizeof(mux_recv_struct),
						GFP_KERNEL);
		if (!recv_info) {
			mux_send_info_flags[line] = 0;
			kfree(mux_send_info[line]);
			mux_send_info[line] = 0;
			TS0710_DEBUG("Free mux_send_info for /dev/pts%d", line);
			retval = -ENOMEM;

			mux_tty[line]--;
			goto out;
		}
		recv_info->length = 0;
		recv_info->total = 0;
		recv_info->next = 0;
		recv_info->no_tty = line;
		recv_info->post_unthrottle = 0;
		mux_recv_info[line] = recv_info;
		mux_recv_info_flags[line] = 1;
		TS0710_DEBUG("Allocate mux_recv_info for /dev/mux%d", line);
	}

	/* Now establish DLCI connection */
	if (mux_tty[dlci] > 0) {
		if ((retval = ts0710_open_channel(dlci)) != 0) {
			TS0710_PRINTK("MUX: Can't connected channel %d!\n",
				      dlci);
			ts0710_reset_dlci(dlci);

			mux_send_info_flags[line] = 0;
			kfree(mux_send_info[line]);
			mux_send_info[line] = 0;
			TS0710_DEBUG("Free mux_send_info for /dev/mux%d", line);

			mux_recv_info_flags[line] = 0;
			free_mux_recv_struct(mux_recv_info[line]);
			mux_recv_info[line] = 0;
			TS0710_DEBUG("Free mux_recv_info for /dev/mux%d", line);

			mux_tty[line]--;
			goto out;
		}
	}


	retval = 0;
out:
	return retval;
}

/* mux dispatcher, call from serial.c receiver_chars() */
void mux_dispatcher(struct tty_struct *tty)
{
	UNUSED_PARAM(tty);

	/* schedule_work(&receive_tqueue); */
}

static void send_ack(ts0710_con * ts0710, u8 seq_num)
{
	mux_send_frame(CTRL_CHAN, ts0710->initiator, ACK, &seq_num, 1);
}

static void ts_ldisc_rx_post(struct tty_struct *tty, const u8 *data, char *flags, int count)
{
#ifndef LGE_KERNEL_MUX
	static u8 expect_seq = 0;
#endif    
	int framelen;
	short_frame *short_pkt;
	long_frame *long_pkt;

	short_pkt = (short_frame *) (data + ADDRESS_FIELD_OFFSET);
	if (short_pkt->h.length.ea == 1) {
		framelen = TS0710_MAX_HDR_SIZE + short_pkt->h.length.len + 1 + SEQ_FIELD_SIZE;
	} else {
		long_pkt = (long_frame *) (data + ADDRESS_FIELD_OFFSET);
		framelen = TS0710_MAX_HDR_SIZE + GET_LONG_LENGTH(long_pkt->h.length) + 2 + SEQ_FIELD_SIZE;
	}
#ifdef LGE_KERNEL_MUX
/* To remove Motorola's modem specific*/
    ts0710_recv_data(&ts0710_connection,
				(char*)(data + 1),
				framelen - 2);
#else
    TS0710_PRINTK("MUX: ts_ldisc_rx_post: expect_seq = %x \n", expect_seq );
    TS0710_PRINTK("MUX: ts_ldisc_rx_post: *(data + SLIDE_BP_SEQ_OFFSET) = %x \n", *(data + SLIDE_BP_SEQ_OFFSET) );
	if (expect_seq == *(data + SLIDE_BP_SEQ_OFFSET)) {
		expect_seq++;
		if (expect_seq >= 4)
			expect_seq = 0;

		send_ack(&ts0710_connection, expect_seq);

		ts0710_recv_data(&ts0710_connection,
				(char*)(data + ADDRESS_FIELD_OFFSET),
				framelen - 2 - SEQ_FIELD_SIZE);
	} else {
		TS0710_PRINTK("miss seq. possibly lost sync!\n");
	}
#endif

}

void ts_ldisc_rx(struct tty_struct *tty, const u8 *data, char *flags, int size)
{
    struct mux_data *st = (struct mux_data *)tty->disc_data;
#ifdef LGE_KERNEL_MUX
    int i;
    short_frame *short_pkt;
    long_frame *long_pkt;
    for(i=0; i < size; i++){
        switch (st->state){
        case OUT_OF_FRAME:
            if(data[i] == (u8)TS0710_BASIC_FLAG){
                st->state = INSIDE_FRAME_HEADER;
                st->copy_index = 0;
            }
            else{
                printk("\nTS0710:ts_ldisc_rx:bad_data,%x\n",data[i]);
                break;
            }
        case INSIDE_FRAME_HEADER:
            if(st->copy_index < TS0710_MAX_HDR_SIZE){
                if((st->copy_index == 1) && (data[i] == (u8)TS0710_BASIC_FLAG)) /* this is fix of out of sync with seq F9 F9*/
                    st->copy_index = 0;
                st->chunk[st->copy_index++]=data[i];
                break;
            }
            else{
                short_pkt = (short_frame *) (st->chunk + ADDRESS_FIELD_OFFSET);
                if (short_pkt->h.length.ea == 1) {
                    st->framelen = TS0710_MAX_HDR_SIZE + short_pkt->h.length.len + 1 + SEQ_FIELD_SIZE;
                } else {
                    long_pkt = (long_frame *) short_pkt;
                    st->framelen = TS0710_MAX_HDR_SIZE + GET_LONG_LENGTH(long_pkt->h.length) + 2 + SEQ_FIELD_SIZE;
                }
                if(st->framelen > MAX_FRAME_SIZE){
                    printk("\nTS0710:ts_ldisc_rx:Too big frame to copy %d!\n",st->framelen);
                    st->state = OUT_OF_FRAME ;
                    break;
                }
                st->state = INSIDE_FRAME_BODY; 
            }
        case INSIDE_FRAME_BODY:
            if(st->copy_index < st->framelen){
                st->chunk[st->copy_index++]=data[i];
            }
            if(!(st->copy_index < st->framelen)){
                ts_ldisc_rx_post(tty, st->chunk, flags,	st->framelen);
                st->state = OUT_OF_FRAME;
            }
            break;
        default:
            printk("\nTS0710:ts_ldisc_rx:unknown state!!!!!!!\n");
            return;
        }
    }
#else
	u8 *packet_start = data;
	while (size--) {
		if (*data == 0xF9) {
			if (st->state == OUT_OF_PACKET) {
				st->state = INSIDE_PACKET;
				packet_start = data;
			} else {
				/* buffer points at ending 0xF9 */
				int packet_size = (data - packet_start) + 1;

				if (packet_size + st->chunk_size == 2) {
					packet_start++;
					data++;
					continue;
				}

				st->state = OUT_OF_PACKET;

				if (!st->chunk_size)
					ts_ldisc_rx_post(
						tty,
						packet_start,
						flags,
						packet_size
					);
				else { /* use existing chunk */
					memcpy(st->chunk + st->chunk_size, packet_start, packet_size);
					ts_ldisc_rx_post(tty, st->chunk, flags,
						st->chunk_size + packet_size);

					st->chunk_size = 0;
				}
			}
		}
		data++;
	}
	if (st->state == INSIDE_PACKET) /* create/update chunk */
	{
		size_t new_chunk_size = data - packet_start; /* buffer points right after data end */
		memcpy(st->chunk + st->chunk_size, packet_start, new_chunk_size);
		st->chunk_size += new_chunk_size;
	}
#endif
}

#ifdef LGE_KERNEL_MUX
static int ts_ldisc_tx_looper(void *param)
{
    int i,res;
    u8 *data_ptr;
    int data_size;
    void * next_ptr;     
    while(!kthread_should_stop()){
		//printk("%s:CPU%d-%d\n",__FUNCTION__, smp_processor_id(), __LINE__);	
		WAIT_UNLOCK(frame_nodes_lock);
		LOCK(frame_nodes_lock);
		//printk("%s-%d\n",__FUNCTION__, __LINE__);	
        data_size = 0;
        data_ptr  = NULL;
        for(i = 0; i < TS0710MAX_PRIORITY_NUMBER ; i++){
            if(frame_to_send[i] != NULL){
                data_ptr = frame_to_send[i]->data;
                data_size = frame_to_send[i]->size;
                next_ptr = frame_to_send[i]->next;
                frame_to_send[i]->size = 0;
                frame_to_send[i]->data = NULL;
                frame_to_send[i]->next = NULL;
                frame_to_send[i]= next_ptr;
              /*  printk("\nTS0710:ts_ldisc_tx_looper:send to dlci=%d",i);*/
                break;
            }
        }
		UNLOCK(frame_nodes_lock);
        if((data_size > 0) && (data_ptr != NULL)){
			//printk("%s-%d\n",__FUNCTION__, __LINE__);	
            res = ipc_tty->ops->write(ipc_tty, data_ptr, data_size);
			//printk("%s-%d\n",__FUNCTION__, __LINE__);	
            if (res != data_size)
                printk("\nTS0710:ts_ldisc_tx_looper:data_size=%d,res=%d\n",data_size,res);
            kfree(data_ptr);    
        }
        down(&spi_write_sema);
		//printk("%s-%d\n",__FUNCTION__, __LINE__);	
    }
    return 0;
}

#endif

static int ts_ldisc_open(struct tty_struct *tty)
{
	struct mux_data *disc_data;
     int i;

	tty->receive_room = 65536;

	disc_data = kzalloc(sizeof(struct mux_data), GFP_KERNEL);

	disc_data->state = OUT_OF_FRAME;
	disc_data->chunk_size = 0;

	tty->disc_data = disc_data;


    for (i=0; i<32; i++){
        spi_write_data_sema[i].lock = __SPIN_LOCK_UNLOCKED(spi_write_data_sema[i].lock);
        spi_write_data_sema[i].count = 0;
        spi_write_data_sema[i].wait_list.next = &(spi_write_data_sema[i].wait_list);
        spi_write_data_sema[i].wait_list.prev = &(spi_write_data_sema[i].wait_list);
    }

	ipc_tty = tty;
#ifdef LGE_KERNEL_MUX
    spin_lock_init(&frame_nodes_lock) ;
    nodes_init();
    ts0710_reset_dlci_priority();
    spi_data_recieved.tty = NULL;
    spi_data_recieved.data = NULL;
    spi_data_recieved.flags = NULL;
    spi_data_recieved.size = 0;
    spi_data_recieved.updated = 0;
    // LGE_UPDATE_S // 20100826 , initialize task handle [START]
    task = NULL;
    // LGE_UPDATE_E // 20100826 , initialize task handle [START]
    write_task = NULL;
    write_task = kthread_run(ts_ldisc_tx_looper,NULL,"%s","ts_ldisc_tx_looper");
    if(write_task == NULL)
        TS0710_DEBUG("ts_ldisc_open WRITE_THREAD is not started!!!\n");

// LGE_UPDATE_S // 20100722  
//	set_user_nice(write_task, 19); // 19:O 5:O 2:O 0:O
//	set_user_nice(write_task, 0);
// LGE_UPDATE_E
#endif    
	TS0710_PRINTK("ts_ldisc_open executed\n");

	return 0;
}

static void ts_ldisc_close(struct tty_struct *tty)
{
	ipc_tty = 0;
#ifdef LGE_KERNEL_MUX
    if(task != NULL){
        kthread_stop(task);
        TS0710_DEBUG("ts_ldisc_close READ_THREAD is stopped\n");    
    }
    if(write_task != NULL){
        // LGE_UPDATE_S // 20100826 , if write semaphore holds on the thread, release it [START]
        up(&spi_write_sema);
        // LGE_UPDATE_E // 20100826 , if write semaphore holds on the thread, release it [START]
        kthread_stop(write_task);
        TS0710_DEBUG("ts_ldisc_close WRITE_THREAD is stopped\n");    
    }
	
#ifdef RECOVERY_MODE
	{
		ts0710_con *ts0710 = &ts0710_connection;
		u8 i;

#ifdef ENABLE_MUX_WAKE_LOCK
		wake_lock_timeout(&s_wake_lock, MUX_WAKELOCK_TIME);
#endif
		LOCK(frame_nodes_lock);
		for(i = 0; i < TS0710MAX_PRIORITY_NUMBER ; i++)
		{
			if(frame_to_send[i] != NULL && frame_to_send[i]->data !=NULL)
			{
				TS0710_PRINTK("%s - frame_to_send[%d]->data = %x\n", __FUNCTION__, i, frame_to_send[i]->data);
				kfree(frame_to_send[i]->data);
				frame_to_send[i]->size = 0;
				frame_to_send[i]->data = NULL;
				frame_to_send[i]->next = NULL;
			}
		}
		UNLOCK(frame_nodes_lock);

		ts0710_upon_disconnect();
		tty_ldisc_flush(tty);
		if(tty->disc_data)
		{
			TS0710_PRINTK("%s - tty->disc_data = %x\n", __FUNCTION__, tty->disc_data);
			kfree(tty->disc_data);
			tty->disc_data = NULL;
		}

		TS0710_PRINTK("%s - start recovery mode!!\n", __FUNCTION__);
		if(hGpio==NULL)
			hGpio =  (NvOdmServicesGpioHandle)NvOdmGpioOpen();
		if(hPin==NULL)
			hPin = NvOdmGpioAcquirePinHandle(hGpio, 'v'-'a', 0);
		NvOdmGpioSetState(hGpio,hPin, 1);	//20100607, , remain only this code
		NvOdmGpioConfig(hGpio, hPin, NvOdmGpioPinMode_Output);
		NvOsSleepMS(200);
		NvOdmGpioSetState(hGpio,hPin, 0);	//20100607, , remain only this code
		NvOsSleepMS(200);
		NvOdmGpioSetState(hGpio,hPin, 1);	//20100607, , remain only this code
		//NvOdmGpioReleasePinHandle(hGpio, hPin);
		//NvOsSleepMS(1000);
		TS0710_PRINTK("%s - end recovery mode!!\n", __FUNCTION__);
	}
#endif //RECOVERY_MODE
#endif //LGE_KERNEL_MUX

}

static void ts_ldisc_wake(struct tty_struct *tty)
{
	printk("ts wake\n");
}

static ssize_t ts_ldisc_read(struct tty_struct *tty, struct file *file,
					unsigned char __user *buf, size_t nr)
{
	return 0;
}

static ssize_t ts_ldisc_write(struct tty_struct *tty, struct file *file,
					const unsigned char *data, size_t count)
{
	return 0;
}

static int ts_ldisc_ioctl(struct tty_struct *tty, struct file * file,
					unsigned int cmd, unsigned long arg)
{
	return -1;
}

static unsigned int ts_ldisc_poll(struct tty_struct *tty,
					struct file *filp, poll_table *wait)
{
	return 0;
}


struct tty_operations mux_ops = {
	.open = mux_open,
	.close = mux_close,
	.write = mux_write,
	.write_room = mux_write_room,
	.flush_buffer = mux_flush_buffer,
	.chars_in_buffer = mux_chars_in_buffer,
	.throttle = mux_throttle,
	.unthrottle = mux_unthrottle,
};

static struct tty_ldisc_ops ts_ldisc = {
	.owner		= THIS_MODULE,
	.magic		= TTY_LDISC_MAGIC,
	.name		= "ts07.10",
	.open		= ts_ldisc_open,
	.close		= ts_ldisc_close,
	.read		= ts_ldisc_read,
	.write		= ts_ldisc_write,
	.poll		= ts_ldisc_poll,
	.receive_buf	= ts_ldisc_rx,
	.write_wakeup	= ts_ldisc_wake,
	.ioctl		= ts_ldisc_ioctl,
};

#ifndef LGE_KERNEL_MUX
static u8 iscmdtty_gen1[16] =
	{ 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0 };

static u8 iscmdtty_gen2[ 23 ] =
	{ 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
#endif    

static int __init mux_init(void)
{
	u8 j;
    int result;
#ifdef LGE_KERNEL_MUX
    NR_MUXS = TS0710MAX_CHANNELS;
    iscmdtty = NULL;
    mux_send_info_idx = TS0710MAX_CHANNELS;
#else
	if (machine_is_ezx_a1200() || machine_is_ezx_e6()) {
		NR_MUXS = 23;
		iscmdtty = iscmdtty_gen2;
		mux_send_info_idx = 23;
	}else{
		NR_MUXS = 16;
		iscmdtty = iscmdtty_gen1;
		mux_send_info_idx = 16;
	}
#endif
	TS0710_PRINTK("initializing mux with %d channels\n", NR_MUXS);

	ts0710_init();

	for (j = 0; j < NR_MUXS; j++) {
		mux_send_info_flags[j] = 0;
		mux_send_info[j] = 0;
		mux_recv_info_flags[j] = 0;
		mux_recv_info[j] = 0;
	}
	mux_send_info_idx = NR_MUXS;
	mux_recv_queue = NULL;
	mux_recv_flags = 0;

	mux_driver = alloc_tty_driver(NR_MUXS);
	if (!mux_driver)
		return -ENOMEM;

	mux_driver->owner = THIS_MODULE;
	mux_driver->driver_name = "ts0710mux";
#ifdef LGE_KERNEL_MUX
    mux_driver->name = "pts";
#else
	mux_driver->name = "mux";
#endif
	mux_driver->major = TS0710MUX_MAJOR;
	mux_driver->minor_start = TS0710MUX_MINOR_START;
	mux_driver->type = TTY_DRIVER_TYPE_SERIAL;
	mux_driver->subtype = SERIAL_TYPE_NORMAL;
	mux_driver->flags = TTY_DRIVER_RESET_TERMIOS | TTY_DRIVER_REAL_RAW | TTY_DRIVER_DYNAMIC_DEV;


	mux_driver->init_termios = tty_std_termios;
	mux_driver->init_termios.c_iflag = 0;
	mux_driver->init_termios.c_oflag = 0;
	mux_driver->init_termios.c_cflag = B38400 | CS8 | CREAD;
	mux_driver->init_termios.c_lflag = 0;

	/* mux_driver.ttys = mux_table; */
	mux_driver->termios = mux_termios;
	mux_driver->termios_locked = mux_termios_locked;
	/* mux_driver.driver_state = mux_state; */
	mux_driver->other = NULL;

	tty_set_operations(mux_driver, &mux_ops);

	/* FIXME: No panic() here */
	if (result=tty_register_driver(mux_driver)){
        printk("TS0710:tty_register_driver returns %d",result);
		panic("TS0710 :Couldn't register mux driver");
        }

	for (j = 0; j < NR_MUXS; j++)
		tty_register_device(mux_driver, j, NULL);
#ifdef LGE_FROYO_BSP
	if (result=tty_register_ldisc(N_TS0710, &ts_ldisc)){
#else
	if (result=tty_register_ldisc(N_TS0710, &ts_ldisc)){
#endif
		printk("TS0710 :oops. cant register ldisc\n");
        }

#ifdef ENABLE_MUX_WAKE_LOCK
	wake_lock_init(&s_wake_lock, WAKE_LOCK_SUSPEND, "mux_wake");
#endif

	return 0;
}

static void __exit mux_exit(void)
{
	u8 j;


	mux_send_info_idx = NR_MUXS;
	mux_recv_queue = NULL;
	for (j = 0; j < NR_MUXS; j++) {
		if ((mux_send_info_flags[j]) && (mux_send_info[j])) {
			kfree(mux_send_info[j]);
		}
		mux_send_info_flags[j] = 0;
		mux_send_info[j] = 0;

		if ((mux_recv_info_flags[j]) && (mux_recv_info[j])) {
			free_mux_recv_struct(mux_recv_info[j]);
		}
		mux_recv_info_flags[j] = 0;
		mux_recv_info[j] = 0;
	}

	for (j = 0; j < NR_MUXS; j++)
		tty_unregister_device(mux_driver, j);

	if (tty_unregister_driver(mux_driver))
		panic("Couldn't unregister mux driver");
}

module_init(mux_init);
module_exit(mux_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Harald Welte <laforge@openezx.org>");
MODULE_DESCRIPTION("TS 07.10 Multiplexer");
