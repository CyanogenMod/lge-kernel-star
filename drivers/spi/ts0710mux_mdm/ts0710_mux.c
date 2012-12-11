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

// -------------------- Start of Changes History -------------------------------------
// when       who     what, where, why
// --------   ---     ----------------------------------------------------------------
//                                                             
// -------------------- End of Changes History ---------------------------------------

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
#include <linux/delay.h>

#include <asm/uaccess.h>
#include <asm/system.h>
#include <asm/bitops.h>
#include <asm/mach-types.h>

#include <linux/spinlock.h>
#include <linux/semaphore.h>
#include <linux/mutex.h>
#include <linux/kthread.h>
#include <linux/version.h>

#include "ts0710.h"
#include "ts0710_mux.h"


/*------------------------------------------------------------------*/
/* Features                                                         */
/*------------------------------------------------------------------*/
/* MSPI to MSPI test mode support */
#define MSPI2MSPI_TEST

/* This enables wakelock if ts_ldisc_close function is called because of RILD exit */
#define ENABLE_MUX_WAKE_LOCK

/* RIL Recovery feature */
#define RECOVERY_MODE

/* Whether to use tty_insert_flip_string */
#define USE_TTY_INSERT
/*------------------------------------------------------------------*/
/* END Features                                                     */
/*------------------------------------------------------------------*/

/*------------------------------------------------------------------*/
/* Driver Name and Version                                          */
/*------------------------------------------------------------------*/
#define MUX_DRIVER_NAME                             "ts0710mux"
#define MUX_DRIVER_VERSION                          "23-mar-12"
static const char mux_drv_driver_name[]             = MUX_DRIVER_NAME;
static const char mux_drv_driver_version[]          = MUX_DRIVER_VERSION;
/*------------------------------------------------------------------*/
/* END Driver Name and Version                                      */
/*------------------------------------------------------------------*/

/*------------------------------------------------------------------*/
/* Debug                                                            */
/*------------------------------------------------------------------*/
#define MUX_DRV_DEBUG_LEVEL                         10
#define MUX_EXTENDED_DEBUG_OUTPUT_FORMAT
#ifdef MUX_DRV_DEBUG_LEVEL
static unsigned int debug_level                     = 0;
#endif
#ifdef MUX_EXTENDED_DEBUG_OUTPUT_FORMAT
#define MUX_DEBUG_OUTPUT_FORMAT                     MUX_DRIVER_NAME "(" MUX_DRIVER_VERSION ")"
#else
#define MUX_DEBUG_OUTPUT_FORMAT                     MUX_DRIVER_NAME
#endif
#define MUX_ERR(msg,args...)                        {printk(KERN_ERR MUX_DEBUG_OUTPUT_FORMAT ":%s: " msg "\n",__func__, ## args);}
#ifdef MUX_DRV_DEBUG_LEVEL
#define MUX_ISDBG(level)                            (((level) <= MUX_DRV_DEBUG_LEVEL) && ((level) <= debug_level))
#define MUX_DBG(level,msg,args...)                  { if (MUX_ISDBG(level)) {printk(KERN_INFO MUX_DEBUG_OUTPUT_FORMAT ": " msg "\n", ## args);}; }
#define MUX_TRACE(msg,args...)                      { if (MUX_ISDBG(MUX_DRV_DEBUG_LEVEL)) {printk(KERN_INFO MUX_DEBUG_OUTPUT_FORMAT ":%s: " msg "\n", __func__, ## args);}; }
/* Unconditional message */
#define UNMUX_DBG(msg,args...)                      { {printk(KERN_INFO MUX_DEBUG_OUTPUT_FORMAT ": " msg "\n", ## args);}; }
#define MUX_DRV_DEBUG_FS_LEVEL
#else
#define MUX_ISDBG(level)                            0
#define MUX_DBG(level,msg,args...)
#endif
/*------------------------------------------------------------------*/
/* End Debug                                                        */
/*------------------------------------------------------------------*/


/*------------------------------------------------------------------*/
/* Feature specific                                                 */
/*------------------------------------------------------------------*/
#ifdef RECOVERY_MODE
#include <mach/gpio-names.h>
#include <mach/gpio.h>

#define GPIO_CP_RESET        TEGRA_GPIO_PV0 //CP_RESET
#define GPIO_CP_PWRON        TEGRA_GPIO_PV1 //CP_PWRON
#endif

#ifdef ENABLE_MUX_WAKE_LOCK
#include <linux/wakelock.h>
#define MUX_WAKELOCK_TIME                           (30*HZ)
struct wake_lock                s_wake_lock;
#endif
/*------------------------------------------------------------------*/
/* END Feature specific                                             */
/*------------------------------------------------------------------*/

/*------------------------------------------------------------------*/
/* General definitions                                              */
/*------------------------------------------------------------------*/
#ifndef MIN
#define MIN(a, b)                                   ((a) < (b) ? (a) : (b))
#endif

#define TS0710MAX_PRIORITY_NUMBER                   2

volatile int                    ts_ldisc_called     = 0;

static struct tty_struct        *ipc_tty;

struct mux_data
{
    enum {OUT_OF_FRAME, INSIDE_FRAME_HEADER, INSIDE_FRAME_BODY } state;
    int         framelen;
    u8          fcs;
};

struct mux_frame_struct
{
    struct mux_frame_struct     *next;
    u8                          *data;
    int                         size;
    int                         dlci;
};

struct mux_frames_pool_struct
{
    spinlock_t                  lock;
    struct mux_frame_struct     *list_head;
    int                         capacity;
    int                         usage;
};

struct mux_queue_struct;

typedef void (*mux_queue_wm_cb)(struct mux_queue_struct*);
typedef bool (*mux_queue_find_cb)(struct mux_queue_struct*, struct mux_frame_struct*);

struct mux_queue_struct
{
    spinlock_t                      nodes_lock;
    int                             count;

    void                            *user_data;
    struct mux_frames_pool_struct   *pool;

    // Nodes list
    struct mux_frame_struct         *list_head;
    struct mux_frame_struct         *list_tail;

    // Watermarks
    mux_queue_wm_cb                 hiwater_cb;
    int                             hiwater;
    mux_queue_wm_cb                 lowater_cb;
    int                             lowater;
};

static struct tty_driver        *mux_driver;
static struct tty_struct        *mux_table[TS0710_MAX_CHANNELS];
static volatile short int       mux_tty[TS0710_MAX_CHANNELS];
static volatile struct file     *mux_filp[TS0710_MAX_CHANNELS];

static ts0710_con               ts0710_connection;

#define MUX_FLOW_CONTROLLED(dlci_data) ((dlci_data)->state == FLOW_STOPPED || all_flow_off_from_modem)

static u8 default_priority_table[] =
{
    0,          /* multiplexer control channel */
    1           /* ALL DLC */
};
/*------------------------------------------------------------------*/
/* END General definitions                                          */
/*------------------------------------------------------------------*/


/*------------------------------------------------------------------*/
/* TX definitions                                                   */
/*------------------------------------------------------------------*/
#define HI_WM_MUX_TX                                20
#define LO_WM_MUX_TX                                6

#define MAX_WAITING_FRAMES                          70
#define MAX_WAITING_FRAMES_OVER                     (MAX_WAITING_FRAMES + (2*TS0710_MAX_CHANNELS))

static struct work_struct       mux_tx_work;
static struct workqueue_struct  *mux_tx_wq;

static struct mux_queue_struct  mux_tx_q[TS0710MAX_PRIORITY_NUMBER];

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,36))
static DECLARE_MUTEX(ipc_write_completion);
#else
static DEFINE_SEMAPHORE(ipc_write_completion);
#endif
static spinlock_t               tx_lock             = SPIN_LOCK_UNLOCKED;
/*------------------------------------------------------------------*/
/* END TX definitions                                               */
/*------------------------------------------------------------------*/


/*------------------------------------------------------------------*/
/* RX definitions                                                   */
/*------------------------------------------------------------------*/
#define HI_WM_MUX_RX                                50
#define LO_WM_MUX_RX                                20

static struct mux_queue_struct  mux_rx_q[TS0710_MAX_CHANNELS];

static volatile int all_flow_off_from_modem                  = 0;

static u8 *mux_cbn_splt_start_frame                 = NULL;
static u16 mux_cbn_splt_frame_len                   = 0;

static void mux_rx_flush_queue(int dlci);
static void mux_rx_destroy_queue(int dlci);
static void mux_rx_handle_work(struct work_struct *pwork);
/*------------------------------------------------------------------*/
/* END RX definitions                                               */
/*------------------------------------------------------------------*/


/*------------------------------------------------------------------*/
/* MSPI2MSPI test mode variables                                    */
/*------------------------------------------------------------------*/
#ifdef MSPI2MSPI_TEST
#define TIOCMSPI2MSPI_T1                            0x5504
#define TIOCMSPI2MSPI_T2                            0x5505
#define TIOCMSPI2MSPI_T3                            0x5506
#define TIOCMSPI2MSPISTOP                           0x5507
#define TIOCGSTATMSPI                               0x5508

#define MSPI2MSPI_DEF_PACKET_SIZE                   2048

static volatile bool            mspitest_enable     = false;
static volatile bool            mspitest_tx_enable  = false;
static volatile bool            mspitest_rx_enable  = false;

static int                      mspitest_frame_size = MSPI2MSPI_DEF_PACKET_SIZE;
static u8                       *mspi_data;

static uint64_t                 mspi2mspi_tx_counter = 0;
static uint64_t                 mspi2mspi_rx_counter = 0;
static spinlock_t               mspi2mspi_lock;

struct mspi2mspi_stat
{
    uint64_t tx_counter;
    uint64_t rx_counter;
    uint64_t timestamp;
};

#endif//MSPI2MSPI_TEST
/*------------------------------------------------------------------*/
/* END MSPI2MSPI test mode variables                                */
/*------------------------------------------------------------------*/


#ifdef MUX_DRV_DEBUG_FS_LEVEL
/*--------------------------------------------------------------------*/
/* Proc file system related DBG functionality for MUX.                */
/*--------------------------------------------------------------------*/
#define MUX_PROC_DBG_FILE                           "driver/mux_dbg_level"
#define MUX_DBG_PROC_BUF_MAX_LEN                    3UL

static struct proc_dir_entry *mux_proc_dbg_file;

static int mux_proc_dbg_read(char *page, char **start, off_t off, int count, int *eof, void *data)
{
    int len;

    MUX_DBG(8, "%s called, curr debug_level = %d\n", __FUNCTION__, debug_level);
    len = sprintf(page, "%d\n", debug_level);

    return len;

}

static int mux_proc_dbg_write(struct file *file, const char __user *buffer, unsigned long count, void *data)
{
    char buff[MUX_DBG_PROC_BUF_MAX_LEN];
    int val;
    unsigned long len = min(MUX_DBG_PROC_BUF_MAX_LEN, count);
    MUX_DBG(8, "%s called, curr debug_level = %u, passed count %lu\n", __FUNCTION__, debug_level, count);

    if(copy_from_user(buff, buffer, len))
        return -EFAULT;

    buff[len-1] = '\0';
    val = simple_strtoul(buff, NULL, 10);

    if(val >= 0 && val <= MUX_DRV_DEBUG_LEVEL)
    {
        debug_level = (unsigned int)val;
        MUX_DBG(8, "%s: updated debug_level = %u\n", __FUNCTION__, debug_level);
    }

    return len;
}

static void create_mux_proc_dbg_file(void)
{
    mux_proc_dbg_file = create_proc_entry(MUX_PROC_DBG_FILE, 0664, NULL);

    if(mux_proc_dbg_file != NULL)
    {
        mux_proc_dbg_file->read_proc = mux_proc_dbg_read;
        mux_proc_dbg_file->write_proc = mux_proc_dbg_write;
    }
    else
        MUX_ERR(" MUX DBG proc file create failed!\n");
}

static void remove_mux_proc_dbg_file(void)
{
    remove_proc_entry(MUX_PROC_DBG_FILE, NULL);
}

#endif //MUX_DRV_DEBUG_FS_LEVEL
/*--------------------------------------------------------------------*/
/* END Proc file system related DBG functionality for MUX.            */
/*--------------------------------------------------------------------*/

/*--------------------------------------------------------------------*/
/* Queue and pool interfaces.                                         */
/*--------------------------------------------------------------------*/
static struct mux_frame_struct          frame_node[MAX_WAITING_FRAMES_OVER];
static struct mux_frames_pool_struct    frames_pool;
static volatile int                     frame_usage                                 = 0;

/* Initialize the frames pool */
static int mux_pool_init( struct mux_frames_pool_struct *pool,
                          struct mux_frame_struct *frames_array,
                          int count )
{
    int i;

    if( (pool == NULL) || (count <= 0) || (frames_array == NULL) )
    {
        return -EINVAL;
    }

    spin_lock_init(&pool->lock);

    pool->capacity = count;

    count--;

    for(i = 0; i < count; i++)
    {
        frames_array[i].next = &frames_array[i+1];
        frames_array[i].data = NULL;
        frames_array[i].size = 0;
        frames_array[i].dlci = 0;
    }

    frames_array[count].next = NULL;
    frames_array[count].data = NULL;
    frames_array[count].size = 0;
    frames_array[count].dlci = 0;

    pool->list_head = &frames_array[0];

    pool->usage = 0;

    return 0;
}

/* Assign the pool to the queue to be used for mux_queue_alloc */
static int mux_queue_pool_set( struct mux_queue_struct    *q,
                               struct mux_frames_pool_struct *pool )
{
    if((q == NULL) || (pool == NULL) || (pool->capacity == 0))
        return -EINVAL;

    q->pool = pool;

    return 0;
}

/* Initialize the queue */
static int mux_queue_init( struct mux_queue_struct    *q,
                           mux_queue_wm_cb            hiwater_cb,
                           int                        hiwater,
                           mux_queue_wm_cb            lowater_cb,
                           int                        lowater,
                           void                       *user_data )
{
    spin_lock_init(&q->nodes_lock);
    q->count = 0;

    q->list_head = q->list_tail = 0;

    q->hiwater_cb = hiwater_cb;
    q->hiwater = hiwater;

    q->lowater_cb = lowater_cb;
    q->lowater = lowater;

    q->user_data = user_data;

    return 0;
}

/* Allocate the new node for the specified queue */
static struct mux_frame_struct*
mux_queue_alloc(struct mux_queue_struct *q)
{
    struct mux_frame_struct     *node = NULL;

    if(q->pool != NULL)
    {
        struct mux_frames_pool_struct   *pool = q->pool;

        spin_lock_bh(&pool->lock);

        if (pool->list_head != NULL)
        {
            node = pool->list_head;
            pool->list_head = pool->list_head->next;
        }

        spin_unlock_bh(&pool->lock);

        if(node != NULL)
            node->next = NULL;
    }
    else
    {
        node = (struct mux_frame_struct*)kzalloc(sizeof(struct mux_frame_struct), GFP_KERNEL);
    }

    return node;
}

/* Free the node of the specified queue */
static int mux_queue_free(struct mux_queue_struct *q, struct mux_frame_struct *node)
{
    if(q->pool != NULL)
    {
        struct mux_frames_pool_struct *pool = q->pool;

        spin_lock_bh(&pool->lock);
    
        node->next = pool->list_head;
        pool->list_head = node;
    
        spin_unlock_bh(&pool->lock);
    }
    else
    {
        kfree(node);
    }

    return 0;
}

/* Returns packet count in the queue */
static int mux_queue_count(struct mux_queue_struct *q)
{
    int count;

    spin_lock_bh(&q->nodes_lock);
    count = q->count;
    spin_unlock_bh(&q->nodes_lock);

    return count;
}

/* Add the packet to the queue tail */
static int mux_queue_enqueue(struct mux_queue_struct *q, struct mux_frame_struct *node)
{
    if(node == NULL)
    {
        return -EINVAL;
    }

    spin_lock_bh(&q->nodes_lock);

    if(q->list_head == NULL)
    {
        q->list_head = node;
        q->list_tail = node;
    }
    else
    {
        q->list_tail->next = node;
        q->list_tail = node;
    }

    q->count++;

    if((q->count > q->hiwater) && (q->hiwater_cb != NULL))
    {
        (q->hiwater_cb)(q);
    }

    spin_unlock_bh(&q->nodes_lock);

    return 0;
}

/* Remove the packet from the queue head */
static struct mux_frame_struct*
mux_queue_dequeue(struct mux_queue_struct *q)
{
    struct mux_frame_struct *node = NULL;

    spin_lock_bh(&q->nodes_lock);

    if(q->list_head != NULL)
    {
        node = q->list_head;
        q->list_head = node->next;

        q->count--;

        if((q->count < q->lowater) && (q->lowater_cb != NULL))
        {
            (q->lowater_cb)(q);
        }
    }

    spin_unlock_bh(&q->nodes_lock);

    return node;
}

/* Clear the queue. All nodes will be passed to flush_fn callback */
static void mux_queue_flush(struct mux_queue_struct *q, mux_queue_find_cb flush_fn)
{
    struct mux_frame_struct *node = NULL;

    if(spin_is_locked(&q->nodes_lock))
        spin_unlock(&q->nodes_lock);

    spin_lock_bh(&q->nodes_lock);
    while(q->list_head != NULL)
    {
        node = q->list_head;
        q->list_head = node->next;

        if(flush_fn != NULL)
        {
            flush_fn(q, node);
        }

        mux_queue_free(q, node);
    }

    q->count = 0;
    spin_unlock_bh(&q->nodes_lock);
}

/* Search the queue for the specific node and dequeue it.
   find_fn is a search callback, node will be dequeued if it returns true */
static struct mux_frame_struct*
mux_queue_search(struct mux_queue_struct *q, mux_queue_find_cb find_fn)
{
    struct mux_frame_struct **prev;
    struct mux_frame_struct *cur;

    if(q == NULL || find_fn == NULL)
    {
        return NULL;
    }

    spin_lock_bh(&q->nodes_lock);

    prev = &q->list_head;
    cur = q->list_head;
    while(cur != NULL)
    {
        if(find_fn(q, cur))
        {
            *prev = cur->next;
            break;
        }

        prev = &cur->next;
        cur = cur->next;
    }

    spin_unlock_bh(&q->nodes_lock);

    return cur;
}
/*--------------------------------------------------------------------*/
/* END Queue and pool interfaces.                                     */
/*--------------------------------------------------------------------*/


/*--------------------------------------------------------------------*/
/* Forward declarations                                               */
/*--------------------------------------------------------------------*/
static int mux_tx_send_frame(u8 dlci, int initiator, enum mux_frametype frametype, const u8 data[], int len);
static void mux_rx_queue_hiwater_cb(struct mux_queue_struct *q);
static void mux_rx_queue_lowater_cb(struct mux_queue_struct *q);
static void ts_ldisc_flush_buffer(struct tty_struct *tty);
/*--------------------------------------------------------------------*/
/* END Forward declarations                                           */
/*--------------------------------------------------------------------*/

/*--------------------------------------------------------------------*/
/* MUX General code                                                   */
/*--------------------------------------------------------------------*/

static void ts0710_reset_dlci(u8 j)
{
    if (j >= TS0710_MAX_CHANNELS)
        return;

    ts0710_connection.dlci[j].state             = DISCONNECTED;
    ts0710_connection.dlci[j].rx.flow_disabled  = false;
    ts0710_connection.dlci[j].mtu               = DEF_TS0710_MTU;
    init_waitqueue_head(&ts0710_connection.dlci[j].open_wait);
    init_waitqueue_head(&ts0710_connection.dlci[j].close_wait);
}

static void ts0710_reset_dlci_priority(void)
{
    u8 j;

    ts0710_connection.dlci[0].priority = default_priority_table[0];

    for (j = 1; j < TS0710_MAX_CHANNELS; j++)
        ts0710_connection.dlci[j].priority = default_priority_table[1];
}

static void ts0710_reset_con(void)
{
    u8 j;

    ts0710_connection.initiator     = 0;
    ts0710_connection.mtu           = DEF_TS0710_MTU + TS0710_MAX_HDR_SIZE;
    ts0710_connection.be_testing    = 0;
    ts0710_connection.test_errs     = 0;
    init_waitqueue_head(&ts0710_connection.test_wait);

    ts0710_reset_dlci_priority();

    for (j = 0; j < TS0710_MAX_CHANNELS; j++)
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

    for (j = 0; j < TS0710_MAX_CHANNELS; j++) {
        ts0710->dlci[j].state = DISCONNECTED;
        wake_up_interruptible(&ts0710->dlci[j].open_wait);
        wake_up_interruptible(&ts0710->dlci[j].close_wait);
    }
    ts0710->be_testing = 0;
    wake_up_interruptible(&ts0710->test_wait);
    ts0710_reset_con();
}

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
                nreg ^= 0xe0;   /* x^8 + x^2 + x + 1 */
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


/* Creates a UA packet and puts it at the beginning of the pkt pointer */
static void send_ua(ts0710_con * ts0710, u8 dlci)
{
    mux_tx_send_frame(dlci, !ts0710->initiator, MUX_UA, 0, 0);
}

/* Creates a DM packet and puts it at the beginning of the pkt pointer */
static void send_dm(ts0710_con * ts0710, u8 dlci)
{
    mux_tx_send_frame(dlci, !ts0710->initiator, MUX_DM, 0, 0);
}

static void send_sabm(ts0710_con * ts0710, u8 dlci)
{
    mux_tx_send_frame(dlci, ts0710->initiator, MUX_SABM, 0, 0);
}

static void send_disc(ts0710_con * ts0710, u8 dlci)
{
    mux_tx_send_frame(dlci, !ts0710->initiator, MUX_DISC, 0, 0);
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

    send->res1              = 0;
    send->res2              = 0;
    send->dlci              = dlci;
    send->frame_type        = 0;
    send->credit_flow       = credit_flow;
    send->prior             = prior;
    send->ack_timer         = 0;

    send->frame_sizel       = frame_size & 0xFF;
    send->frame_sizeh       = frame_size >> 8;

    send->credits           = credits;
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
    u8 *send = kmalloc(len + 2, GFP_KERNEL);
    mcc_short_frame_head *head = NULL;

    if (send == NULL)
        return;

    head                        = (mcc_short_frame_head *)send;
    head->type.ea               = 1;
    head->type.cr               = cr;
    head->type.type             = type;
    head->length.ea             = 1;
    head->length.len            = len;

    if (len > 0)
    {
        memcpy(send + 2, data, len);
    }

    mux_tx_send_frame(CTRL_CHAN, ts0710->initiator, MUX_UIH, send, len + 2);

    kfree(send);
}

static int ts0710_msc_msg(ts0710_con * ts0710, u8 value, u8 cr, u8 dlci)
{
    u8 buf[2];
    msc_t *send = (msc_t *)buf;

    send->dlci.ea           = 1;
    send->dlci.cr           = 1;
    send->dlci.d            = dlci & 1;
    send->dlci.server_chn   = (dlci >> 1) & 0x1f;
    send->v24_sigs          = value;

    mux_send_uih(ts0710, cr, MSC, buf, 2);

    return 0;
}

/* Parses a multiplexer control channel packet */
static void process_mcc(u8 * data, ts0710_con * ts0710, int longpkt)
{
    mcc_short_frame *mcc_short_pkt;
    int             i;

    if (longpkt)
        mcc_short_pkt = (mcc_short_frame *) (((long_frame *) data)->data);
    else
        mcc_short_pkt = (mcc_short_frame *) (((short_frame *) data)->data);

    switch (mcc_short_pkt->h.type.type)
    {

        case FCON:      /*Flow control on command */
            MUX_DBG(2,"MUX Received Flow control(all channels) on command\n");
            if (mcc_short_pkt->h.type.cr == MCC_CMD) {
                all_flow_off_from_modem = 0;

                for(i = 1; i < TS0710_MAX_CHANNELS; i++)
                {
                    dlci_struct *dlci_data = &ts0710->dlci[i];
                    if(!MUX_FLOW_CONTROLLED(dlci_data))
                    {
                        up(&dlci_data->tx.write_sema);
                        (void)down_trylock(&dlci_data->tx.write_sema);
                    }
                }
            }
            break;

        case FCOFF:     /*Flow control off command */
            MUX_DBG(2,"MUX Received Flow control(all channels) off command\n");
            if (mcc_short_pkt->h.type.cr == MCC_CMD)
            {
                all_flow_off_from_modem = 1;
            }
            break;

        case MSC:       /*Modem status command */
            {
                int                 dlci;
                u8                  v24_sigs;
                dlci_struct         *dlci_data;

                dlci = (mcc_short_pkt->value[0]) >> 2;
                v24_sigs = mcc_short_pkt->value[1];
                dlci_data = &ts0710->dlci[dlci];

                if ((dlci_data->state != CONNECTED)
                    && (dlci_data->state != FLOW_STOPPED)) {
                    send_dm(ts0710, dlci);
                    break;
                }

                if (mcc_short_pkt->h.type.cr == MCC_CMD) {
                    MUX_DBG(7,"Received Modem status command\n");
                    if ((v24_sigs & 2) && (dlci_data->state == CONNECTED))
                    {
                        MUX_DBG(2,"MUX Received Flow off on dlci %d\n", dlci);
                        dlci_data->state = FLOW_STOPPED;
                    }
                    else if(MUX_STOPPED(ts0710,dlci))
                    {
                        dlci_data->state = CONNECTED;
                        MUX_DBG(2,"MUX Received Flow on on dlci %d\n", dlci);
                        if(!MUX_FLOW_CONTROLLED(dlci_data))
                        {
                            up(&dlci_data->tx.write_sema);
                            down_trylock(&dlci_data->tx.write_sema);
                        }
                    }

                    ts0710_msc_msg(ts0710, v24_sigs, MCC_RSP, dlci);
                } else {
                    MUX_DBG(7,"Received Modem status response\n");
                    if (v24_sigs & 2) {
                        MUX_DBG(3,"Flow stop accepted\n");
                    }
                }
                break;
            }

        case PN:        /*DLC parameter negotiation */
            {
                int                 dlci;
                u16                 frame_size;
                pn_msg              *pn_pkt;
                dlci_struct         *dlci_data;

                pn_pkt = (pn_msg *) data;
                dlci = pn_pkt->dlci;
                frame_size = GET_PN_MSG_FRAME_SIZE(pn_pkt);
                dlci_data = &ts0710->dlci[dlci];
                MUX_DBG(7,"Received DLC parameter negotiation, PN\n");

                if (pn_pkt->mcc_s_head.type.cr == MCC_CMD) {
                    MUX_DBG(7,"received PN command with frame size:%d\n", frame_size);

                    frame_size = MIN(frame_size, dlci_data->mtu);
                    send_pn_msg(ts0710, pn_pkt->prior, frame_size, 0, 0, dlci, MCC_RSP);
                    dlci_data->mtu = frame_size;
                    MUX_DBG(7,"process_mcc : mtu set to %d\n", dlci_data->mtu);
                } else {
                    MUX_DBG(7,"received PN response with frame size:%d\n", frame_size);

                    frame_size = MIN(frame_size, dlci_data->mtu);
                    dlci_data->mtu = frame_size;


                    if (dlci_data->state == NEGOTIATING) {
                        dlci_data->state = CONNECTING;
                        wake_up_interruptible(&dlci_data->open_wait);
                    }
                }
                break;
            }

        case NSC:       /*Non supported command resonse */
            MUX_ERR("MUX Received Non supported command response\n");
            break;

        default:        /*Non supported command received */
            MUX_ERR("MUX Received a non supported command\n");
            send_nsc_msg(ts0710, mcc_short_pkt->h.type, MCC_RSP);
            break;
    }
}

static void ts0710_flow_on(int dlci, ts0710_con *ts0710)
{
    if ((!((ts0710->dlci[0].state) & (CONNECTED | FLOW_STOPPED)))
        || (!((ts0710->dlci[dlci].state) & (CONNECTED | FLOW_STOPPED)))
        || (ts0710->dlci[dlci].rx.flow_disabled == false))
        return;

    MUX_DBG(2,"MUX send Flow on on dlci %d\n", dlci);

    ts0710_msc_msg(ts0710, EA | RTC | RTR | DV, MCC_CMD, dlci);

    ts0710->dlci[dlci].rx.flow_disabled = false;
}

static void ts0710_flow_off(int dlci, ts0710_con *ts0710)
{
    if (((ts0710->dlci[0].state != CONNECTED) && (ts0710->dlci[0].state != FLOW_STOPPED))
        || ((ts0710->dlci[dlci].state != CONNECTED) && (ts0710->dlci[dlci].state != FLOW_STOPPED))
        || (ts0710->dlci[dlci].rx.flow_disabled == true))
        return;

    MUX_DBG(2,"MUX send Flow off on dlci %d\n", dlci);

    ts0710_msc_msg(ts0710, EA | FC | RTC | RTR | DV, MCC_CMD, dlci);

    ts0710->dlci[dlci].rx.flow_disabled = true;
}


/* Close ts0710 channel */
static void ts0710_close_channel(int dlci)
{
    ts0710_con                    *ts0710 = &ts0710_connection;
    int                           try;
    unsigned long                 t;

    MUX_DBG(5,"Close channel %d\n", dlci);

    if ((ts0710->dlci[dlci].state == DISCONNECTED)
        || (ts0710->dlci[dlci].state == REJECTED)
        || (ts0710->dlci[dlci].state == DISCONNECTING))
            return;

    ts0710->dlci[dlci].state = DISCONNECTING;
    try = TS0710MUX_DISC_TRIES;
    while (try--)
    {
        t = jiffies;
        send_disc(ts0710, dlci);
        interruptible_sleep_on_timeout(&ts0710->dlci[dlci].
                           close_wait,
                           TS0710MUX_TIME_OUT);
        if (ts0710->dlci[dlci].state == DISCONNECTED) {
            break;
        } else if (signal_pending(current)) {
            MUX_DBG(6,"MUX DLCI %d Send DISC got signal!\n", dlci);
            break;
        } else if ((jiffies - t) >= TS0710MUX_TIME_OUT) {
            MUX_DBG(6,"MUX DLCI %d Send DISC timeout!\n", dlci);
            continue;
        }
    }

    if (ts0710->dlci[dlci].state != DISCONNECTED)
    {
        if (dlci == 0) {
            ts0710_upon_disconnect();
        } else {
            ts0710->dlci[dlci].state = DISCONNECTED;
            wake_up_interruptible(&ts0710->dlci[dlci].
                          close_wait);
            ts0710_reset_dlci(dlci);
        }
    }


    mux_rx_flush_queue(dlci);
    mux_rx_destroy_queue(dlci);
}

static int ts0710_open_channel(int dlci)
{
    ts0710_con *ts0710 = &ts0710_connection;
    int try;
    int retval;
    unsigned long t;
    unsigned char wq_name[10];
    dlci_struct      *dlci_data = &ts0710->dlci[dlci];

    if(dlci_data->rx.workqueue == NULL)
    {
        INIT_WORK(&(dlci_data->rx.work.work), mux_rx_handle_work);
        dlci_data->rx.work.dlci = dlci;
        sprintf(wq_name, "muxrx_%d", dlci);
        dlci_data->rx.workqueue = create_singlethread_workqueue(wq_name);
        dlci_data->rx.flow_disabled = false;
        sema_init(&dlci_data->rx.rx_completion, 0);
        dlci_data->dlci = dlci;

        mux_queue_init( &mux_rx_q[dlci],
                        mux_rx_queue_hiwater_cb,
                        HI_WM_MUX_RX,
                        mux_rx_queue_lowater_cb,
                        LO_WM_MUX_RX,
                        dlci_data );

    }

    retval = -ENODEV;
    if (dlci == 0) {    /* control channel */
        if (ts0710->dlci[0].state & (CONNECTED | FLOW_STOPPED)) {
            return 0;
        } else if (ts0710->dlci[0].state == CONNECTING) {
            /* Reentry */
            MUX_DBG(5,"MUX DLCI: 0, reentry to open DLCI 0, pid: %d, %s !\n",
                current->pid, current->comm);

            try = TS0710MUX_SABM_TRIES;
            while (try--) {
                t = jiffies;
                interruptible_sleep_on_timeout(&ts0710->dlci[0].open_wait,
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
                    MUX_DBG(6,"MUX DLCI:%d Wait for connecting got signal!\n", dlci);
                    retval = -EAGAIN;
                    break;
                } else if ((jiffies - t) >= TS0710MUX_TIME_OUT) {
                    MUX_DBG(6,"MUX DLCI:%d Wait for connecting timeout!\n",dlci);
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
            MUX_DBG(6,"MUX DLCI:%d state is invalid!\n", dlci);
            return retval;
        } else {
            ts0710->initiator = 1;
            ts0710->dlci[0].state = CONNECTING;

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
                MUX_DBG(6,"MUX DLCI:%d Send SABM got rejected!\n",dlci);
                retval = -EREJECTED;
            } else if (signal_pending(current)) {
                MUX_DBG(6,"MUX DLCI:%d Send SABM got signal!\n", dlci);
                retval = -EAGAIN;
            } else if ((jiffies - t) >= TS0710MUX_TIME_OUT) {
                MUX_DBG(6,"MUX DLCI:%d Send SABM timeout!\n",dlci);
                retval = -ENODEV;
            }

            if (ts0710->dlci[0].state == CONNECTING) {
                ts0710->dlci[0].state = DISCONNECTED;
            }
            wake_up_interruptible(&ts0710->dlci[0].open_wait);
        }
    } else {        /* other channel */
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
            MUX_DBG(6,"MUX DLCI:%d state is invalid!\n", dlci);
            return retval;
        } else {
            ts0710->dlci[dlci].state = CONNECTING;
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
                    MUX_DBG(6,"MUX DLCI:%d Send SABM got rejected!\n",dlci);
                    retval = -EREJECTED;
                } else if (signal_pending(current)) {
                    MUX_DBG(6,"MUX DLCI:%d Send SABM got signal!\n",dlci);
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

/*--------------------------------------------------------------------*/
/* END MUX General code                                               */
/*--------------------------------------------------------------------*/


/*--------------------------------------------------------------------*/
/* MUX TX code                                                        */
/*--------------------------------------------------------------------*/

static void mux_tx_queues_init(void)
{
    int                 i;

    mux_pool_init( &frames_pool,
                   frame_node,
                   MAX_WAITING_FRAMES_OVER );

    frame_usage = 0;

    for (i=0; i < TS0710MAX_PRIORITY_NUMBER; i++)
    {
        mux_queue_init( &mux_tx_q[i],
                        NULL,
                        0,
                        NULL,
                        0,
                        NULL );

        mux_queue_pool_set( &mux_tx_q[i], &frames_pool);
    }
}


static bool mux_tx_queue_search_cb(struct mux_queue_struct *q, struct mux_frame_struct *node)
{
    if(ts0710_connection.dlci[node->dlci].state != FLOW_STOPPED)
        return true;

    return false;
}

static void mux_tx_handle_work(struct work_struct *work)
{
    struct mux_frame_struct     *frame;
    dlci_struct                     *dlci_data;
    u8                              *data_ptr;
    int                             data_size = 1; //just to pass first cycle in the main loop
    int                             dlci;
    int                             i;


#ifdef MSPI2MSPI_TEST
    if(mspitest_enable == true)
    {
        if(mspitest_tx_enable == true)
        {
            mspi_data = (u8 *)kmalloc(mspitest_frame_size, GFP_KERNEL | __GFP_ZERO);
            data_ptr = mspi_data;
            data_size = mspitest_frame_size;
            if((data_size > 0) && (data_ptr != NULL))
            {
                while(1)
                {
                    set_bit(TTY_DO_WRITE_WAKEUP, &ipc_tty->flags);
                    while(data_size != ipc_tty->ops->write(ipc_tty, data_ptr, data_size))
                    {
                        down(&ipc_write_completion);
                        set_bit(TTY_DO_WRITE_WAKEUP, &ipc_tty->flags);
                    }
                    spin_lock_bh(&mspi2mspi_lock);
                    mspi2mspi_tx_counter += data_size;
                    spin_unlock_bh(&mspi2mspi_lock);
                    yield();
                }
            }
        }
        else
        {
            return;
        }
    }
#endif//MSPI2MSPI_TEST


    while( (data_size != 0) &&
             (ts_ldisc_called == 0) &&
             (frame_usage > 0) )
    {
        data_ptr = NULL;
        data_size = 0;
        dlci = 0;

        for(i = 0; i < TS0710MAX_PRIORITY_NUMBER ; i++)
        {
            if(all_flow_off_from_modem && (i > 0))
            {
                MUX_DBG(6,"mux_tx_looper flow off - returns\n");
                return;
            }

            frame = mux_queue_search(&mux_tx_q[i], mux_tx_queue_search_cb);

            if(frame != NULL)
            {
                data_ptr = frame->data;
                data_size = frame->size;
                dlci = frame->dlci;

                MUX_TRACE("found data %p sz %d dlci %d", data_ptr, data_size, dlci);

                dlci_data = &ts0710_connection.dlci[dlci];

                dlci_data->tx.chars_in_buffer -= data_size;

                mux_queue_free(&mux_tx_q[i], frame);

                spin_lock_bh(&tx_lock);
                frame_usage--;

                if(dlci_data->tx.frame_count > 0)
                    dlci_data->tx.frame_count--;

                if((dlci_data->tx.wakeup_flag == 1) && (dlci_data->tx.frame_count < LO_WM_MUX_TX) && (mux_table[dlci] != NULL))
                {
                    dlci_data->tx.wakeup_flag = 0;
                    MUX_DBG(8,"tty_wakeup on %d\n",dlci);
                    spin_unlock_bh(&tx_lock);
                    wake_up_interruptible_poll(&(mux_table[dlci]->write_wait), POLLOUT);
                    break;
                }
                spin_unlock_bh(&tx_lock);
                break;
            }
        }

        if((data_size > 0) && (data_ptr != NULL))
        {
            set_bit(TTY_DO_WRITE_WAKEUP, &ipc_tty->flags);
            while((ts_ldisc_called == 0) && (data_size != ipc_tty->ops->write(ipc_tty, data_ptr, data_size)))
            {
                if((dlci_data->tx.wakeup_flag == 1) && (mux_table[dlci] != NULL))
                {
                    dlci_data->tx.wakeup_flag = 0;
                    MUX_DBG(8,"tty_wakeup on %d\n",dlci);
                    wake_up_interruptible_poll(&(mux_table[dlci]->write_wait), POLLOUT);
                }
                down_interruptible(&ipc_write_completion);
                set_bit(TTY_DO_WRITE_WAKEUP, &ipc_tty->flags);
            }

            kfree(data_ptr);
        }

    } 
}

static int mux_tx_put_to_send(int dlci, u8 *data, int size)
{
    dlci_struct                     *dlci_data = &ts0710_connection.dlci[dlci];
    int                             priority = dlci_data->priority;
    struct mux_queue_struct         *queue = &mux_tx_q[priority];
    struct mux_frame_struct         *node = NULL;
    int                             retval = 0;

    spin_lock_bh(&tx_lock);
    if( ((dlci == 0) && (frame_usage < MAX_WAITING_FRAMES_OVER)) ||
        ((dlci > 0)  && (frame_usage < MAX_WAITING_FRAMES) && (dlci_data->tx.frame_count < HI_WM_MUX_TX)))
    {
        node = mux_queue_alloc(queue);
        if (node != NULL)
        {
            dlci_data->tx.frame_count++;
            frame_usage++;
            MUX_DBG(8,"frame_usage = %d\n",frame_usage);
            spin_unlock_bh(&tx_lock);

            node->data = data;
            node->size = size;
            node->dlci = dlci;

            retval = size;

            MUX_TRACE("enqueue send data %p sz %d dlci %d", data, size, dlci);
            mux_queue_enqueue(queue, node);

            dlci_data->tx.chars_in_buffer += size;
        }
    }

    if(node == NULL)
    {
        spin_unlock_bh(&tx_lock);
        MUX_DBG(7,"ERROR: NOMEM node = %p\n", node);
        kfree(data);
        retval = -ENOMEM;
    }
    else
    {
        queue_work(mux_tx_wq, &mux_tx_work);
    }

    return retval;
}

/* See TS 07.10's Section 5.2.1 */
static int mux_tx_send_frame(u8 dlci, int initiator, enum mux_frametype frametype, const u8 data[], int len)
{

    int pos = 0;
    int pf, crc_len, res;
    int cr = initiator & 0x1;
    u_int8_t fcs;
    u8 *framebuf = kmalloc(MAX_FRAME_SIZE, GFP_ATOMIC);

    if (framebuf == NULL)
    {
        MUX_ERR("Alloc Failed \n");
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

    framebuf[pos++] = MUX_BASIC_FLAG_SEQ;

    /* Address field.  */
    framebuf[pos++] = MUX_EA | (cr << 1) | (dlci << 2);

    /* Control field.  */
    framebuf[pos++] = frametype | (pf << 4);

    if (len & ~0x7f) {
        framebuf[pos ++] = 0 | ((len & 0x7f) << 1);
        framebuf[pos ++] = len >> 7;
    } else
        framebuf[pos ++] = 1 | (len << 1);

    /* Information field.  */
    if (len > 0)
    {
        memcpy(&framebuf[pos], data, len);
        pos += len;
    }

    fcs = mux_fcs_compute(framebuf + 1, (pos - 1 - crc_len));

    framebuf[pos++] = fcs;

    framebuf[pos ++] = MUX_BASIC_FLAG_SEQ;

    MUX_TRACE("put to send buf %p sz %d dlci %d", framebuf, pos, dlci);
    res = mux_tx_put_to_send( dlci, framebuf, pos);

    if(res == -ENOMEM){
      return res;
    }

    if (res != pos) {
        MUX_DBG(3,"mux_send_frame error %d\n", res);
        return -1;
    }

    /* Function should return number of sent information bytes.
    Therefore need to decrease ret variable */
    if (len & ~0x7f) {
        /* Length indicator have two octet size. */
        res -= 7;
    } else {
        res -= 6;
    }

    return res;
}
static int mux_tx_uih_data(ts0710_con * ts0710, u8 dlci, u8 *data, int len)
{
    int ret =0;
    if (len > 0)
    {
        ret = mux_tx_send_frame(dlci, ts0710->initiator, MUX_UIH, data, len);
    }
    return ret;
}
/*--------------------------------------------------------------------*/
/* END MUX TX code                                                    */
/*--------------------------------------------------------------------*/


/*--------------------------------------------------------------------*/
/* MUX RX code                                                        */
/*--------------------------------------------------------------------*/

static void mux_rx_queue_hiwater_cb(struct mux_queue_struct *q)
{
    dlci_struct                 *dlci_data = (dlci_struct*)q->user_data;

    ts0710_flow_off(dlci_data->dlci, &ts0710_connection);
}

static void mux_rx_queue_lowater_cb(struct mux_queue_struct *q)
{
    dlci_struct                 *dlci_data = (dlci_struct*)q->user_data;

    ts0710_flow_on(dlci_data->dlci, &ts0710_connection);
}


static bool mux_rx_queue_flush_cb(struct mux_queue_struct *q, struct mux_frame_struct *node)
{
    kfree(node->data);

    return true;
}
static void mux_rx_flush_queue(int dlci)
{
    struct mux_queue_struct     *queue = &mux_rx_q[dlci];
    dlci_struct                 *dlci_data = (dlci_struct*)queue->user_data;

    if(dlci_data && dlci_data->rx.workqueue != NULL)
    {
        mux_queue_flush(queue, mux_rx_queue_flush_cb);

        dlci_data->rx.flow_disabled = false;
        flush_workqueue(dlci_data->rx.workqueue);
    }
}

static void mux_rx_destroy_queue(int dlci)
{
    dlci_struct                 *dlci_data = (dlci_struct*)mux_rx_q[dlci].user_data;

    if(dlci_data && dlci_data->rx.workqueue != NULL)
    {
        destroy_workqueue(dlci_data->rx.workqueue);
        dlci_data->rx.workqueue = NULL;
    }
}

static void mux_rx_uih(ts0710_con * ts0710, u8 *payload, int payload_len, int dlci)
{
    struct tty_struct *tty  = NULL;
    dlci_struct             *dlci_data = &ts0710->dlci[dlci];

    if ((dlci_data->state != CONNECTED)
            && (dlci_data->state != FLOW_STOPPED))
    {
        MUX_ERR("MUX Error: DLCI %d not connected, discard it!\n", dlci);
        send_dm(ts0710, dlci);
        return;
    }

    MUX_DBG(7,"UIH on channel %d\n", dlci);

    if (payload_len > dlci_data->mtu)
    {
        MUX_ERR("MUX Error:  DLCI:%d, uih_len:%d is bigger than mtu:%d, discard data!\n",
                dlci, payload_len, dlci_data->mtu);
        return;
    }

    tty = mux_table[dlci];
    if ((!mux_tty[dlci]) || (!tty)) {
        return;
    }

#ifdef USE_TTY_INSERT
    // Ensure that tty does have enough space to receive packet, otherwise wait for unthrottle
    while(tty_buffer_request_room(tty, payload_len) < payload_len)
    {
        MUX_DBG(9, "not enough receive room on dlci %d", dlci);
        set_bit(TTY_THROTTLED, &tty->flags);
        while(down_interruptible(&dlci_data->rx.rx_completion) != 0) {};
    }

    tty_insert_flip_string(tty, payload, payload_len);
    tty_flip_buffer_push(tty);

#else
   
    tty->ldisc->ops->receive_buf(tty, payload, NULL, payload_len);

#endif
    
}

static void ts0710_recv_data_server(ts0710_con * ts0710, short_frame *short_pkt)
{
    u8 be_connecting;

    switch (CLR_PF(short_pkt->h.control))
    {
        case SABM:
            MUX_DBG(7,"SABM-packet received\n");
            ts0710->dlci[0].state = CONNECTED;
            send_ua(ts0710, 0);
            wake_up_interruptible(&ts0710->dlci[0].open_wait);
            break;

        case UA:
            MUX_DBG(7,"UA packet received\n");
            if (ts0710->dlci[0].state == CONNECTING) {
                ts0710->dlci[0].state = CONNECTED;
                wake_up_interruptible(&ts0710->dlci[0].
                                  open_wait);
            } else if (ts0710->dlci[0].state == DISCONNECTING) {
                ts0710_upon_disconnect();
            } else {
                MUX_DBG(7," Something wrong receiving UA packet\n");
            }
            break;

        case DM:
            MUX_DBG(7,"DM packet received\n");
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
            MUX_DBG(7,"DISC packet received\n");
            send_ua(ts0710, 0);
            ts0710_upon_disconnect();
            break;

        case UIH:
            MUX_DBG(7,"UIH packet received\n");

            if (GET_PF(short_pkt->h.control)) {
                MUX_ERR("MUX Error: UIH packet with P/F set, discard it!\n");
                break;
            }

            process_mcc((u8 *)short_pkt, ts0710,
                    !(short_pkt->h.length.ea));
            break;

        default:
            MUX_ERR("Received illegal packet on channel 0\n");
            break;
    }

}

static void ts0710_recv_data(ts0710_con * ts0710, short_frame *short_pkt, int payload_len, int dlci)
{
    u8                  *payload_start;

    if (dlci == 0)
        return ts0710_recv_data_server(ts0710, short_pkt);

    switch (CLR_PF(short_pkt->h.control))
    {
        case SABM:
            MUX_DBG(7,"Incoming connect on channel %d\n", dlci);
            send_ua(ts0710, dlci);
            ts0710->dlci[dlci].state = CONNECTED;
            wake_up_interruptible(&ts0710->dlci[dlci].open_wait);
            break;

        case UA:
            MUX_DBG(7,"Incoming UA on channel %d\n", dlci);
            if (ts0710->dlci[dlci].state == CONNECTING) {
                ts0710->dlci[dlci].state = CONNECTED;
                wake_up_interruptible(&ts0710->dlci[dlci].open_wait);
            } else if (ts0710->dlci[dlci].state == DISCONNECTING) {
                ts0710->dlci[dlci].state = DISCONNECTED;
                wake_up_interruptible(&ts0710->dlci[dlci].open_wait);
                wake_up_interruptible(&ts0710->dlci[dlci].close_wait);
                ts0710_reset_dlci(dlci);
            }
            break;

        case DM:
            MUX_DBG(7,"Incoming DM on channel %d\n", dlci);
            if (ts0710->dlci[dlci].state == CONNECTING)
                ts0710->dlci[dlci].state = REJECTED;
            else
                ts0710->dlci[dlci].state = DISCONNECTED;

            wake_up_interruptible(&ts0710->dlci[dlci].open_wait);
            wake_up_interruptible(&ts0710->dlci[dlci].close_wait);
            ts0710_reset_dlci(dlci);
            break;

        case DISC:
            MUX_DBG(7,"Incoming DISC on channel %d\n", dlci);
            send_ua(ts0710, dlci);
            ts0710->dlci[dlci].state = DISCONNECTED;
            if(waitqueue_active(&ts0710->dlci[dlci].open_wait))
                wake_up_interruptible(&ts0710->dlci[dlci].open_wait);
            if(waitqueue_active(&ts0710->dlci[dlci].close_wait))
                wake_up_interruptible(&ts0710->dlci[dlci].close_wait);
            ts0710_reset_dlci(dlci);
            break;

    case UIH:
            if(short_pkt->h.length.ea == 1)
            {
                payload_start = short_pkt->data;
            }
            else
            {
                long_frame *long_pkt = (long_frame*)short_pkt;
                payload_start = long_pkt->data;
            }
            mux_rx_uih(ts0710, payload_start, payload_len, dlci);
            break;

        default:
            MUX_ERR("Received invalid packet on channel %d\n",dlci);
            break;
    }
}

static void mux_rx_handle_work(struct work_struct *pwork)
{
    mux_rx_work                     *work = container_of(pwork, mux_rx_work, work);
    int                             dlci = work->dlci;
    struct mux_frame_struct     *mux_frame_data = NULL;
    struct mux_queue_struct         *queue = &mux_rx_q[dlci];
    dlci_struct                     *dlci_data = (dlci_struct*)queue->user_data;

    mux_frame_data = mux_queue_dequeue(queue);

    if(mux_frame_data != NULL)
    {
        MUX_DBG(8,"MUX DLCI:%d we have data\n", dlci);

        ts0710_recv_data( &ts0710_connection,
                          (short_frame*)mux_frame_data->data,
                          mux_frame_data->size,
                          mux_frame_data->dlci );

        kfree(mux_frame_data->data);

        mux_queue_free(queue, mux_frame_data);

        if(mux_queue_count(queue) > 0)
        {
            queue_work(dlci_data->rx.workqueue, &(dlci_data->rx.work.work));
        }
    }
}

static void mux_rx_post(struct tty_struct *tty, const u8 *data, int framelen)
{
    struct mux_queue_struct         *queue;
    dlci_struct                     *dlci_data;
    struct mux_frame_struct         *mux_frame_data = NULL;
    short_frame                     *short_pkt;
    int                             payload_len;
    int                             dlci = 0;

    short_pkt = (short_frame *) (data + ADDRESS_FIELD_OFFSET);

    dlci = short_pkt->h.addr.server_chn << 1 | short_pkt->h.addr.d;

    queue = &mux_rx_q[dlci];
    dlci_data = (dlci_struct*)queue->user_data;

    if(dlci >= 0 && (dlci_data == NULL || dlci_data->rx.workqueue == NULL) )
    {
        MUX_ERR("RX queue was not created; dlci = %d\n",dlci);
        return;
    }

    // We will no longer need the start-stop flags and FCS, so cut it off
    framelen -= FLAG_SIZE + SEQ_FIELD_SIZE + FCS_SIZE;

    MUX_DBG(8,"MUX DLCI:%d prepare data for RX proccessing\n", dlci);

    mux_frame_data = mux_queue_alloc(queue);

    if (mux_frame_data != NULL)
    {
        mux_frame_data->data = kmalloc(framelen, GFP_KERNEL);
        if (mux_frame_data->data == NULL)
        {
            MUX_ERR("RX work->data alloc failed\n");
            kfree(mux_frame_data);
            mux_queue_free(queue, mux_frame_data);
            return;
        }

        if (short_pkt->h.length.ea == 1)
        {
            payload_len = short_pkt->h.length.len;
        }
        else
        {
            long_frame *long_pkt = (long_frame*)short_pkt;
            payload_len = GET_LONG_LENGTH(long_pkt->h.length);
        }

        mux_frame_data->size = payload_len;
        mux_frame_data->dlci = dlci;

        memcpy(mux_frame_data->data, data + ADDRESS_FIELD_OFFSET, framelen);

        mux_queue_enqueue(queue, mux_frame_data);

        MUX_DBG(8,"MUX DLCI:%d data RX proccessing %p \n", dlci, mux_frame_data);

        queue_work(dlci_data->rx.workqueue, &(dlci_data->rx.work.work));
    }
    else
    {
        MUX_ERR("RX mux_frame_data alloc failed\n");
    }
}

static void mux_rx_raw_data(struct tty_struct *tty, const u8 *data, char *flags, int size)
{
    struct mux_data *st         = (struct mux_data *)tty->disc_data;
    u8 *frame                   = NULL;
    short_frame *short_pkt      = NULL;
    long_frame *long_pkt        = NULL;
    int offset                  = 0;
    int fcs_len                 = 0;
    u8 fcs;
    u8 *mux_cbn_start_frame     = NULL;
    u8 *mux_cbn_end_frame       = (u8 *)&data[0];
    unsigned int bad_count      = 0;

    while(offset < size)
    {
        switch (st->state)
        {
            case OUT_OF_FRAME:
                if(data[offset] == (u8)TS0710_BASIC_FLAG)
                {
                    frame = (u8 *)&data[offset];
                    if(frame[1] == (u8)TS0710_BASIC_FLAG) /* this is fix of out of sync with seq F9 F9*/
                    {
                        MUX_DBG(9,"bad packet second F9 pos=%d\n", offset);
                        offset++;
                        break;
                    }
                    st->state = INSIDE_FRAME_HEADER;
                    mux_cbn_start_frame = frame;
                }
                else
                {
                    bad_count++;
                    offset++;
                    break;
                }

            case INSIDE_FRAME_HEADER:
                short_pkt = (short_frame *) (frame + ADDRESS_FIELD_OFFSET);
                if (short_pkt->h.length.ea == 1)
                {
                    st->framelen = mux_short_frame_size(&short_pkt->h);
                    fcs_len = sizeof(short_frame);
                }
                else
                {
                    long_pkt = (long_frame *) short_pkt;
                    st->framelen = mux_long_frame_size(&long_pkt->h);
                    fcs_len = sizeof(long_frame);
                }

                if(st->framelen > MAX_FRAME_SIZE)
                {
                    MUX_DBG(3,"bad packet size %d\n", st->framelen);

                    // Loop exit condition
                    st->state = OUT_OF_FRAME;
                    offset++;
                    break;
                }

                // Calculate checksum
                st->fcs = mux_fcs_compute(frame + ADDRESS_FIELD_OFFSET, fcs_len);
                st->state = INSIDE_FRAME_BODY;

            case INSIDE_FRAME_BODY:
                st->state = OUT_OF_FRAME;
                if(offset + st->framelen > size)
                {
                    offset = size;
                    break;
                }

                fcs = frame[st->framelen - FRAME_TAIL_SIZE];
                MUX_TRACE("FCS computed = %X received = %X", st->fcs, fcs);

                if(st->fcs == fcs)
                {
                    mux_rx_post(tty, frame, st->framelen);
                }
                else
                {
                    MUX_ERR("Bad checksum, discarding packet\n");
                }

                offset += st->framelen;
                mux_cbn_end_frame = (u8 *)&data[offset];
                break;
            default:
                MUX_ERR("Bad state in packet proccessing\n");
                return;
        }
    }

    if(bad_count != 0)
        MUX_DBG(1,"bad packet data len=%d from %d\n", bad_count,size);

    if(mux_cbn_end_frame < &data[size])
    {
        mux_cbn_splt_frame_len = &data[size] - mux_cbn_end_frame;
        mux_cbn_splt_start_frame = kmalloc(mux_cbn_splt_frame_len,GFP_KERNEL);

        if(mux_cbn_splt_start_frame != NULL)
        {
            memcpy(mux_cbn_splt_start_frame, mux_cbn_end_frame, mux_cbn_splt_frame_len);
        }
        else
        {
            mux_cbn_splt_frame_len = 0;
            MUX_ERR("cannot allocate memory for split frame");
        }
    }
}
/*--------------------------------------------------------------------*/
/* END MUX RX code                                                    */
/*--------------------------------------------------------------------*/


/*--------------------------------------------------------------------*/
/* TTY driver routines                                                */
/*--------------------------------------------------------------------*/

static int mux_open(struct tty_struct *tty, struct file *filp)
{
    int                     retval = -ENODEV;
    int                     dlci = tty->index;
    dlci_struct             *dlci_data = &ts0710_connection.dlci[dlci];

    mux_filp[dlci] = filp;

    if (!(ipc_tty && dlci))
        return retval;

    mux_tty[dlci]++;
    mux_table[dlci] = tty;

    /* Open server channel 0 first */
    if ((retval = ts0710_open_channel(0)) != 0) {
        MUX_ERR("MUX: Can't connect server channel 0! ret = %d\n", retval);
        ts0710_init();

        mux_tty[dlci]--;
        return retval;
    }

    /* Now establish DLCI connection */
    if ((retval = ts0710_open_channel(dlci)) != 0)
    {
        MUX_ERR("MUX: Can't connected channel %d!\n",
                  dlci);
        ts0710_reset_dlci(dlci);

        mux_tty[dlci]--;
        return retval;
    }

    dlci_data->tx.wakeup_flag = 0;
    tty->low_latency = 1;

    return 0;
}

static void mux_close(struct tty_struct *tty, struct file *filp)
{
    u8 dlci;

    UNUSED_PARAM(filp);
    MUX_DBG(7, "%s called \n", __FUNCTION__);
    dlci = tty->index;
    if (MUX_INVALID(dlci))
        return;

    if (mux_tty[dlci] > 0)
        mux_tty[dlci]--;

    if (mux_tty[dlci] == 0)
#ifdef LGE_KERNEL_MUX
    {
        MUX_DBG(6,"skip to close channel[%d]\n", dlci);
        if(dlci==12)
        {
            ts0710_flow_on(dlci, &ts0710_connection);
        }
    }
#else
    {
        MUX_DBG(6,"close channel[%d]\n", dlci);
        ts0710_close_channel(dlci);
    }
#endif

    mux_filp[dlci] = NULL;
    mux_table[dlci] = NULL;

    if (mux_tty[dlci] != 0)
        return;

    tty_unthrottle(tty);
//  ts0710_flow_on(dlci, ts0710);

    wake_up_interruptible(&tty->read_wait);
    wake_up_interruptible(&tty->write_wait);
    tty->packet = 0;
}


static int mux_write(struct tty_struct *tty, const unsigned char *buf, int count)
{
    ts0710_con                  *ts0710  = &ts0710_connection;
    int                         dlci             = tty->index;
    int                         written         = 0;
    int                         frame_size      = 0;
    int                         frame_written   = 0;
    bool                        is_nonblocking = false;
    dlci_struct                 *dlci_data = &ts0710_connection.dlci[dlci];

    if(mux_filp[dlci] != NULL && mux_filp[dlci]->f_flags & O_NONBLOCK)
        is_nonblocking = true;

    MUX_TRACE("packet buf %p count %d", buf, count);

    if(ts_ldisc_called)
    {
    	MUX_DBG(3, "%s: ts_ldisc_called = %d, return ECANCELED", __FUNCTION__, ts_ldisc_called);
    	return -ECANCELED;
    }

    if (count <= 0)
    {
        MUX_ERR("mux_write:ERROR. count %d\n", count );
        return -EINVAL;
    }

    if(MUX_FLOW_CONTROLLED(dlci_data))
    {
        MUX_DBG(6,"TS0710 Write:Flow OFF state = %d \n",dlci_data->state);

        if(is_nonblocking)
        {
            dlci_data->tx.wakeup_flag = 1;
            return -EWOULDBLOCK;
        }
        else
        {
            MUX_DBG(6,"TS0710 Write: down\n");
            while (down_interruptible(&dlci_data->tx.write_sema)!=0)
            {};
        }
    }

    while(count)
    {
        frame_size = MIN(count, dlci_data->mtu);

        MUX_TRACE("send packet sz %d dlci %d", frame_size, dlci);

        frame_written = mux_tx_uih_data(ts0710, dlci, (u8 *)(buf + written), frame_size);

        if(frame_written <= 0)
        {
            if(MUX_FLOW_CONTROLLED(dlci_data))
            {
                if(is_nonblocking)
                {
                    dlci_data->tx.wakeup_flag = 1;
                    break;
                }
                else
                {
                    MUX_DBG(6,"TS0710 Write: down\n");
                    while (down_interruptible(&dlci_data->tx.write_sema)!=0)
                    {};
                }
            }
            else
            {
                MUX_DBG(7,"TS0710:mux_write(), NOMEM, written %d bytes, should be stopped by client \n",written);
                dlci_data->tx.wakeup_flag = 1;
                return written;
            }
        }
        else
        {
            written += frame_written;
            count -= frame_written;
        }
    };
    return written;
}

static void mux_throttle(struct tty_struct *tty)
{
    MUX_DBG(9, "throttle on pts%d", tty->index);
}

static void mux_unthrottle(struct tty_struct *tty)
{
    int                     dlci = tty->index;
    dlci_struct             *dlci_data = &ts0710_connection.dlci[dlci];

    MUX_DBG(9, "unthrottle on pts%d", tty->index);

    if (MUX_INVALID(dlci))
        return;

    up(&dlci_data->rx.rx_completion);
    down_trylock(&dlci_data->rx.rx_completion);
}

static int mux_chars_in_buffer(struct tty_struct *tty)
{
    ts0710_con *ts0710  = &ts0710_connection;
    u8 dlci             = tty->index;

    if ((MUX_INVALID(dlci))
        || (! MUX_USABLE(ts0710, dlci)))
        return 0;
        
    if(dlci == 0)
    {
		return 65535;
    }
  
    return ts0710->dlci[dlci].tx.chars_in_buffer;
}

static int mux_write_room(struct tty_struct *tty)
{
    dlci_struct                     *dlci_data;
    int dlci = tty->index;

    if (MUX_INVALID(dlci))
        return 0;

    dlci_data = &ts0710_connection.dlci[dlci];

    if (all_flow_off_from_modem
        || (dlci_data->state == FLOW_STOPPED))
    {
        MUX_DBG(6,"write_room: Flow stopped, returning ZERO\n");
        return 0;
    }
    else if (dlci_data->state != CONNECTED)
    {
        MUX_DBG(6,"write_room: DLCI %d not connected\n", dlci);
        return 0;
    }
    else if((frame_usage >= MAX_WAITING_FRAMES) || (dlci_data->tx.frame_count >= HI_WM_MUX_TX))
    {
        MUX_DBG(6,"write_room: Queue overflow\n");
        return 0;
    }

    return ((HI_WM_MUX_TX - dlci_data->tx.frame_count) * dlci_data->mtu);
}


static bool mux_tx_queue_search_dlci_cb(struct mux_queue_struct *q, struct mux_frame_struct *node)
{
    int             dlci = (int)q->user_data;

    if(node->dlci == dlci)
        return true;

    return false;
}


static void mux_flush_dlci(int dlci)
{
    struct mux_frame_struct     *frame;
    dlci_struct                     *dlci_data = &ts0710_connection.dlci[dlci];
    int                             priority = dlci_data->priority;

    if(dlci_data->state == DISCONNECTED)
        return;

    MUX_DBG(8, "flush buffer on pts%d", dlci);

    mux_tx_q->user_data = (void*)dlci;
    while( (frame = mux_queue_search(&mux_tx_q[priority], mux_tx_queue_search_dlci_cb)) != NULL )
    {
        kfree(frame->data);
        mux_queue_free(&mux_tx_q[priority], frame);
    }

    spin_lock_bh(&tx_lock);
    dlci_data->tx.frame_count = 0;
    dlci_data->tx.chars_in_buffer = 0;
    spin_unlock_bh(&tx_lock);
}

static void mux_flush_buffer(struct tty_struct *tty)
{
    mux_flush_dlci(tty->index);
}

static void mux_hangup(struct tty_struct *tty)
{
    MUX_DBG(9, "hangup on pts%d", tty->index);
}


static void mux_set_ldisc(struct tty_struct *tty)
{
    MUX_DBG(9, "line discipline changed for pts%d to %s", tty->index, tty->ldisc->ops->name);
}

/*--------------------------------------------------------------------*/
/* END TTY driver routines                                            */
/*--------------------------------------------------------------------*/

/*--------------------------------------------------------------------*/
/* Line Discipline routines                                           */
/*--------------------------------------------------------------------*/

static int ts_ldisc_open(struct tty_struct *tty)
{
    struct mux_data *disc_data;
    int i;

    tty->receive_room = TS0710MUX_MAX_RECEIVE_ROOM;

    disc_data = kzalloc(sizeof(struct mux_data), GFP_KERNEL);

    disc_data->state = OUT_OF_FRAME;
    disc_data->framelen = 0;

    tty->disc_data = disc_data;

    for (i=0; i<TS0710_MAX_CHANNELS; i++)
    {
        sema_init(&ts0710_connection.dlci[i].tx.write_sema, 0);
    }

    ipc_tty = tty;

    spin_lock_init(&tx_lock);
    mux_tx_queues_init();
    ts0710_reset_dlci_priority();

    all_flow_off_from_modem = 0;
    // Keep it always locked
    MUX_DBG(5,"ts_ldisc_open executed\n");

    return 0;
}

static void ts_ldisc_receive_buf(struct tty_struct *tty, const u8 *data, char *flags, int size)
{
#ifdef MSPI2MSPI_TEST
    if(mspitest_enable == true)
    {
        if(mspitest_rx_enable == true)
        {
            spin_lock_bh(&mspi2mspi_lock);
            mspi2mspi_rx_counter += size;
            spin_unlock_bh(&mspi2mspi_lock);
        }
        return;
    }
#endif//MSPI2MSPI_TEST

    if(mux_cbn_splt_start_frame != NULL)
    {
        u8 *data_tmp = NULL;
        int size_tmp = size;
        data_tmp = kmalloc(mux_cbn_splt_frame_len + size, GFP_KERNEL);
        if(data_tmp != NULL)
        {
            memcpy(data_tmp,mux_cbn_splt_start_frame,mux_cbn_splt_frame_len);
            memcpy(data_tmp + mux_cbn_splt_frame_len,data,size);
            size_tmp += mux_cbn_splt_frame_len;
            kfree(mux_cbn_splt_start_frame);
            mux_cbn_splt_start_frame = NULL;
            mux_cbn_splt_frame_len = 0;
            mux_rx_raw_data(tty, data_tmp, flags, size_tmp);
            kfree(data_tmp);
        }
        else
        {
            MUX_ERR("RX MEM ALLOC for frame failed - previous part was released\n");
            kfree(mux_cbn_splt_start_frame);
            mux_cbn_splt_start_frame = NULL;
            mux_cbn_splt_frame_len = 0;
            mux_rx_raw_data(tty, data, flags, size);
        }
    }
    else
    {
        mux_rx_raw_data(tty, data, flags, size);
    }

    tty_unthrottle(tty);
}


/*
    RECOVERY_MODE
        description : This mode is used to recover the at command pending state
                     Step.1 : CP reset
                     Step.2 : Close all mux channels
                     Step.3 : Wait until CP is rebooting for 5 seconds
*/
static void ts_ldisc_close(struct tty_struct *tty)
{
    struct tty_struct *mspi_tty = ipc_tty;
    MUX_DBG(7, "%s called \n", __FUNCTION__);

    all_flow_off_from_modem = 0;

#ifdef RECOVERY_MODE
    {
        u8 i;

        ts_ldisc_called = 1;

#ifdef ENABLE_MUX_WAKE_LOCK
        wake_lock_timeout(&s_wake_lock, MUX_WAKELOCK_TIME);
#endif


        for(i = 0; i < TS0710MAX_PRIORITY_NUMBER ; i++)
        {
            mux_queue_flush(&mux_tx_q[i], mux_rx_queue_flush_cb);
        }

        if(mux_cbn_splt_start_frame != NULL)
        {
            kfree(mux_cbn_splt_start_frame);
            mux_cbn_splt_start_frame = NULL;
        }
        mux_cbn_splt_frame_len = 0;

        up(&ipc_write_completion);
        down_trylock(&ipc_write_completion);

        for(i = 0; i < TS0710_MAX_CHANNELS; i++)
        {
            if(mux_table[i] != NULL)
            {
                tty_hangup(mux_table[i]);
            }

            mux_flush_dlci(i);
        }

        flush_workqueue(mux_tx_wq);

        ipc_tty = NULL;


        tty_driver_flush_buffer(mspi_tty);

        ts_ldisc_flush_buffer(tty);
        for(i = 0; i < TS0710_MAX_CHANNELS; i++)
        {
            mux_rx_destroy_queue(i);
        }

        ts0710_upon_disconnect();
        tty_ldisc_flush(tty);
        if(tty->disc_data)
        {
            kfree(tty->disc_data);
            tty->disc_data = NULL;
        }

        MUX_ERR("Start RIL Recorvery Mode ...Restart QCT Modem\n");

        gpio_request(GPIO_CP_RESET, "ifx_reset_n");
        tegra_gpio_enable(GPIO_CP_RESET);
        gpio_direction_output(GPIO_CP_RESET, 1);

        UNMUX_DBG("GPIO_CP_RESET: %d", gpio_get_value(GPIO_CP_RESET));

        gpio_set_value(GPIO_CP_RESET, 1);
        UNMUX_DBG("GPIO_CP_RESET: %d", gpio_get_value(GPIO_CP_RESET));

        mdelay(200);
        gpio_set_value(GPIO_CP_RESET, 0);
        UNMUX_DBG("GPIO_CP_RESET: %d", gpio_get_value(GPIO_CP_RESET));

        mdelay(200);
        gpio_set_value(GPIO_CP_RESET, 1); 
        UNMUX_DBG("GPIO_CP_RESET: %d", gpio_get_value(GPIO_CP_RESET));
        
        ts_ldisc_called = 0;
        MUX_ERR("End recovery mode!!\n");
    }
#endif //RECOVERY_MODE
}

static void ts_ldisc_wake(struct tty_struct *tty)
{
    MUX_DBG(8,"Write to MSPI wake up\n");
    clear_bit(TTY_DO_WRITE_WAKEUP, &tty->flags);
    up(&ipc_write_completion);
}

static ssize_t ts_ldisc_read(struct tty_struct *tty, struct file *file,
                             unsigned char __user *buf, size_t nr)
{
    return -EOPNOTSUPP;
}

static ssize_t ts_ldisc_write(struct tty_struct *tty, struct file *file,
                              const unsigned char *data, size_t count)
{
    return -EOPNOTSUPP;
}

static void ts_ldisc_flush_buffer(struct tty_struct *tty)
{
    int i;

    MUX_DBG(9, "flush buffer on MUX");

    for (i = 0; i < TS0710_MAX_CHANNELS; i++)
    {
        mux_rx_flush_queue(i);
    }

    for(i = 0; i < TS0710_MAX_CHANNELS; i++)
    {
        if(mux_table[i] != NULL)
            mux_flush_buffer(mux_table[i]);
    }
}

static int ts_ldisc_ioctl(struct tty_struct *tty, struct file * file,
                          unsigned int cmd, unsigned long arg)
{
    int retval = 0;

    switch (cmd)
    {
#ifdef MSPI2MSPI_TEST
        case TIOCMSPI2MSPI_T1:
            mspitest_tx_enable = true;
            mspitest_rx_enable = true;
            mspitest_enable = true;
            if (arg) mspitest_frame_size = (int) arg;
            MUX_DBG(0,"MSPITEST STARTED in -t%d mode\n",cmd-0x5503);
            queue_work(mux_tx_wq, &mux_tx_work);
            break;
        case TIOCMSPI2MSPI_T2:
            mspitest_tx_enable = false;
            mspitest_rx_enable = true;
            mspitest_enable = true;
            if (arg) mspitest_frame_size = (int) arg;
            MUX_DBG(0,"MSPITEST STARTED in -t%d mode\n",cmd-0x5503);
            queue_work(mux_tx_wq, &mux_tx_work);
            break;
        case TIOCMSPI2MSPI_T3:
            mspitest_tx_enable = true;
            mspitest_rx_enable = false;
            mspitest_enable = true;
            if (arg) mspitest_frame_size = (int) arg;
            MUX_DBG(0,"MSPITEST STARTED in -t%d mode\n",cmd-0x5503);
            queue_work(mux_tx_wq, &mux_tx_work);
            break;
        case TIOCMSPI2MSPISTOP:
            mspitest_rx_enable = false;
            mspitest_tx_enable = false;
            mspitest_enable = false;
            kfree(mspi_data);
            mspi_data = NULL;
            break;
        case TIOCGSTATMSPI:
            {
                struct mspi2mspi_stat stat;
                struct timespec ts;
                spin_lock_bh(&mspi2mspi_lock);
                stat.tx_counter = mspi2mspi_tx_counter;
                stat.rx_counter = mspi2mspi_rx_counter;
                spin_unlock_bh(&mspi2mspi_lock);
                ts = current_kernel_time();
                stat.timestamp = ts.tv_sec * 1000000 + ts.tv_nsec / 1000;

                if (copy_to_user((void*)arg, &stat, sizeof stat))
                    return -EFAULT;
            }
            break;
#endif//MSPI2MSPI_TEST

        case TCFLSH:
            tty_perform_flush(tty, arg);
            break;
        default:
            MUX_DBG(0,"ts_ldisc_ioctl: No such ioctl %x", cmd);
            return -ENOIOCTLCMD;
            break;
    }
    return retval;
}

static unsigned int ts_ldisc_poll(struct tty_struct *tty,
                                  struct file *filp, poll_table *wait)
{
    unsigned int mask = 0;
    u8 dlci = tty->index;

    MUX_TRACE("dlci = %d", dlci);

    poll_wait(filp, &tty->read_wait, wait);
    poll_wait(filp, &tty->write_wait, wait);

    if (test_bit(TTY_OTHER_CLOSED, &tty->flags) ||
        tty_hung_up_p(filp) ||
        ts0710_connection.dlci[dlci].state == DISCONNECTED)
        mask |= POLLHUP;

    if(!tty_is_writelocked(tty) && tty_write_room(tty) > 0)
        mask |= POLLOUT | POLLWRNORM;

    if(mux_queue_count(&mux_rx_q[dlci]) != 0)
        mask |= POLLIN | POLLRDNORM;

    MUX_TRACE("mask = %X", mask);

    return mask;
}

static int ts_ldisc_hangup(struct tty_struct *tty)
{
    ts_ldisc_close(tty);
    return 0;
}

static void ts_ldisc_set_termios(struct tty_struct *tty, struct ktermios * old)
{
    UNUSED_PARAM(old);

    MUX_DBG(9, "termios changed for pts%d", tty->index);
}

/*--------------------------------------------------------------------*/
/* END Line Discipline routines                                       */
/*--------------------------------------------------------------------*/

/*--------------------------------------------------------------------*/
/* Module routines                                                    */
/*--------------------------------------------------------------------*/

struct tty_operations mux_ops = {
    .open               = mux_open,
    .close              = mux_close,
    .write              = mux_write,
    .write_room         = mux_write_room,
    .chars_in_buffer    = mux_chars_in_buffer,
    .throttle           = mux_throttle,
    .unthrottle         = mux_unthrottle,
    .flush_buffer       = mux_flush_buffer,
    .hangup             = mux_hangup,
    .set_ldisc          = mux_set_ldisc,
};

static struct tty_ldisc_ops ts_ldisc = {
    .owner              = THIS_MODULE,
    .magic              = TTY_LDISC_MAGIC,
    .name               = MUX_DRIVER_NAME,
    .open               = ts_ldisc_open,
    .close              = ts_ldisc_close,
    .read               = ts_ldisc_read,
    .write              = ts_ldisc_write,
    .poll               = ts_ldisc_poll,
    .receive_buf        = ts_ldisc_receive_buf,
    .write_wakeup       = ts_ldisc_wake,
    .ioctl              = ts_ldisc_ioctl,
    .flush_buffer       = ts_ldisc_flush_buffer,
    .hangup             = ts_ldisc_hangup,
    .set_termios        = ts_ldisc_set_termios
};

static int __init mux_init(void)
{
    u8 j;
    int result;

#ifdef MUX_DRV_DEBUG_FS_LEVEL
    create_mux_proc_dbg_file();
#endif


    UNMUX_DBG("initializing mux with %d channels\n", TS0710_MAX_CHANNELS);

    ts0710_init();

    mux_driver = alloc_tty_driver(TS0710_MAX_CHANNELS);

    if (!mux_driver)
        return -ENOMEM;

    mux_driver->owner       = THIS_MODULE;
    mux_driver->driver_name = MUX_DRIVER_NAME;
    mux_driver->name        = "pts";
    mux_driver->major       = TS0710MUX_MAJOR;
    mux_driver->minor_start = TS0710MUX_MINOR_START;
    mux_driver->type        = TTY_DRIVER_TYPE_SERIAL;
    mux_driver->subtype     = SERIAL_TYPE_NORMAL;
    mux_driver->flags       = TTY_DRIVER_RESET_TERMIOS | TTY_DRIVER_REAL_RAW | TTY_DRIVER_DYNAMIC_DEV;


    mux_driver->init_termios            = tty_std_termios;
    mux_driver->init_termios.c_iflag    = 0;
    mux_driver->init_termios.c_oflag    = 0;
    mux_driver->init_termios.c_cflag    = B921600 | CS8 | CREAD;
    mux_driver->init_termios.c_lflag    = 0;

    tty_set_operations(mux_driver, &mux_ops);

    if ((result=tty_register_driver(mux_driver)) < 0)
    {
        MUX_ERR("tty_register_driver returns %d",result);
        panic("TS0710 :Couldn't register mux driver");
    }

    for (j = 0; j < TS0710_MAX_CHANNELS; j++)
        tty_register_device(mux_driver, j, NULL);

    INIT_WORK(&mux_tx_work,mux_tx_handle_work);
    mux_tx_wq = create_singlethread_workqueue("mux_tx"); //create_singlethread_workqueue

#ifdef N_TS0710
    if ((result=tty_register_ldisc(N_TS0710, &ts_ldisc)) < 0)
#else
    if ((result=tty_register_ldisc(N_TS2710, &ts_ldisc)) < 0)
#endif
    {
        MUX_ERR("oops. cant register ldisc\n");
    }
#ifdef ENABLE_MUX_WAKE_LOCK
    wake_lock_init(&s_wake_lock, WAKE_LOCK_SUSPEND, "mux_wake");
#endif

    return 0;
}

static void __exit mux_exit(void)
{
    dlci_struct             *dlci_data;
    int                     i;
    MUX_DBG(7, "%s called \n", __FUNCTION__);
#ifdef MUX_DRV_DEBUG_FS_LEVEL
    remove_mux_proc_dbg_file();
#endif


    for (i = 0; i < TS0710_MAX_CHANNELS; i++)
    {
        mux_rx_flush_queue(i);
        mux_rx_destroy_queue(i);
    }

    for(i = 0; i < TS0710MAX_PRIORITY_NUMBER ; i++)
    {
        mux_queue_flush(&mux_tx_q[i], mux_rx_queue_flush_cb);
    }

    spin_lock_bh(&tx_lock);
    frame_usage = 0;
    for(i = 0; i < TS0710_MAX_CHANNELS; i++)
    {
        dlci_data = &ts0710_connection.dlci[i];
        dlci_data->tx.frame_count = 0;
        dlci_data->tx.chars_in_buffer = 0;
        dlci_data->tx.wakeup_flag = 0;
    }
    spin_unlock_bh(&tx_lock);

    if (mux_tx_wq != NULL)
    {
        ts_ldisc_called = 1;
        up(&ipc_write_completion);
        down_trylock(&ipc_write_completion);
        flush_workqueue(mux_tx_wq);
        destroy_workqueue(mux_tx_wq);
        ts_ldisc_called = 0;
        mux_tx_wq = NULL;
    }

    for (i = 0; i < TS0710_MAX_CHANNELS; i++)
        tty_unregister_device(mux_driver, i);

    if (tty_unregister_driver(mux_driver))
        panic("Couldn't unregister mux driver");

#ifdef ENABLE_MUX_WAKE_LOCK
    wake_lock_destroy(&s_wake_lock);
#endif
}

/*--------------------------------------------------------------------*/
/* END Module routines                                                */
/*--------------------------------------------------------------------*/

module_init(mux_init);
module_exit(mux_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Harald Welte <laforge@openezx.org>, Extentded version - Teleca ");
MODULE_DESCRIPTION("TS 07.10 Multiplexer - Extended version");
