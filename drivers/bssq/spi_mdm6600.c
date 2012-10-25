/*
 * spi_mdm6600.c -- Serial peheripheral interface framing layer for MDM6600 modem.
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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/slab.h>
//#include <linux/smp_lock.h>
#include <asm/uaccess.h>
#include <linux/irq.h>
#include <mach/gpio.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <linux/interrupt.h>
#include <linux/completion.h>
#include <linux/spi/spi.h>
#include <linux/workqueue.h>
#include <mach/gpio-names.h>
#include <mach/spi.h>
#include <mach/hardware.h>
#include <linux/hrtimer.h>
#include <linux/delay.h>
#include <linux/err.h>

#ifdef CONFIG_HAS_WAKELOCK
#include <linux/wakelock.h>

#define SPI_WAKELOCK_TIMEOUT 3
#endif

#include "spi_mdm6600.h"
#define TX_BUFFER_QUEUE
#define MDM6600_SPI_HEADER
/*
* create dummy traffic using a polling method with a timer setting 
* of 10 sec to release the AP from an AT command pending state 
*/
#define SPI_STATISTICS_CHECK 

//#define WORK_QUEUE_DELAY		msecs_to_jiffies(5000)

#if defined(CONFIG_BSSQ_FOTA)
extern int mdm_reset(void);
#endif

/* define for spi1, spi2 port gpio information */
static int IPC_MRDY1;
static int IPC_SRDY1;
#ifdef CONFIG_DUAL_SPI
static int IPC_MRDY2;
static int IPC_SRDY2;
#endif

/* ################################################################################################################ */

struct spi_device        *global_spi; //for time out callback function

/* Structure used to store private data */
struct mdm_spi_data {
    dev_t                    devt;
    spinlock_t               spi_lock;
    struct spi_device        *spi;
    struct list_head         device_entry;
    struct completion        mdm_read_write_completion;
    struct tty_struct        *mdm_tty;
    unsigned int             index;

#ifdef TX_BUFFER_QUEUE
    bool is_waiting;
#endif

    /* buffer is NULL unless this device driver is open (users > 0) */
    struct mutex             buf_lock;
    unsigned  int             users;
    unsigned int             throttle;
    struct work_struct      mdm_work;
    struct workqueue_struct  *mdm_wq;

    /* For dual spi */
    /* Tx/Rx Buffer */
    unsigned char            *mdm_tx_buffer;
    unsigned char            *mdm_rx_buffer;

    unsigned int             mdm_master_initiated_transfer;

    /* Buffer management */
    unsigned int             mdm_sender_buf_size;
    unsigned int             mdm_receiver_buf_size;
    unsigned int             mdm_current_frame_size;
    unsigned int             mdm_valid_frame_size;
    unsigned int             mdm_ret_count;
    unsigned int             mdm_spi_count;
    const unsigned char      *mdm_spi_buf;

    struct hrtimer  timer; /* for timeout callback function */

#ifdef CONFIG_HAS_WAKELOCK
	struct wake_lock  spi_wakelock;
	unsigned int		wake_lock_flag;
#endif

    int is_suspended;

    atomic_t is_syncing;
};

union mdm_spi_frame_header{
	struct{
		unsigned int curr_data_size:12;
		unsigned int more:1;
		unsigned int res1:1;
		unsigned int res2:2;
		unsigned int next_data_size:12;
		unsigned int ri:1;
		unsigned int dcd:1;
		unsigned int cts_rts:1;
		unsigned int dsr_dtr:1;
	}mdm_spi_header;
#ifdef MDM6600_SPI_HEADER
    unsigned char framesbytes[4];
    unsigned int framecount;
#else
    unsigned char framesbytes[IFX_SPI_HEADER_SIZE];
#endif
};

struct mdm_spi_data *gspi_data[2];
struct tty_driver   *mdm_spi_tty_driver;
unsigned int        assign_num = 0;

struct allocation_table {
    unsigned short allocated;
	atomic_t in_use;
};

struct allocation_table spi_table[4];

#ifdef CONFIG_PM
#include <mach/iomap.h>
#include <linux/io.h>

#define PMC_WAKE_STATUS 0x14
#if defined (CONFIG_LU8800) || defined (CONFIG_KS1103)  // Wake Event 0 - IFX_SRDY
#define WAKEUP_MDM_SRDY_MASK    (1 << 0)    
#else
#define WAKEUP_MDM_SRDY_MASK    (1 << 7)     
#endif

static void __iomem *pmc_base = IO_ADDRESS(TEGRA_PMC_BASE);
#endif

//#define LGE_DUMP_SPI_BUFFER
#ifdef LGE_DUMP_SPI_BUFFER
#define COL_SIZE 50
static void dump_spi_buffer(const unsigned char *txt, const unsigned char *buf, int count)
{
    char dump_buf_str[COL_SIZE+1];

    if (buf != NULL) 
    {
        int j = 0;
        char *cur_str = dump_buf_str;
        unsigned char ch;
        while((j < COL_SIZE) && (j  < count))
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
#endif

#ifdef SPI_BUFFER_DUMP_LOG
void dump_atcmd(char *data) 
{

	short_frame *short_pkt = (short_frame *) data;
	long_frame *long_pkt;
	u8 *uih_data_start;
	u32 uih_len;
	int i;

	if ((short_pkt->h.length.ea) == 0) 
	{
		long_pkt = (long_frame *) data;
		uih_len = GET_LONG_LENGTH(long_pkt->h.length);
		uih_data_start = long_pkt->h.data;
//		SPI_PRINTK("long packet length %d\n", uih_len);
	} 
	else 
	{
		uih_len = short_pkt->h.length.len;
		uih_data_start = short_pkt->data;
//		SPI_PRINTK("long packet length %d\n", uih_len);
	}
	
	if(uih_len>10)
	{
		uih_len = 10;
	}
	
	for(i=0; i<uih_len; i++)
	{
		if(uih_data_start[i]>=32 && uih_data_start[i]<=126)
		{
			printk("%c ", uih_data_start[i]);
		}
		else
		{
			printk("%x ", uih_data_start[i]);
		}
	}
	printk("\n");
}
#endif

#ifdef SPI_SPEED_MEASUREMENT
#include <linux/time.h>
#define MAX_USED_SPI_DREVICE	1 /* 2*/

static inline unsigned long getuSecTime()
{
	struct timeval tv;
	do_gettimeofday(&tv);
	return (unsigned long)(tv.tv_sec * 1000 * 1000 + tv.tv_usec);
}

static struct timeval ulStart[MAX_USED_SPI_DREVICE], ulEnd[MAX_USED_SPI_DREVICE];
static unsigned long uiTxlen[MAX_USED_SPI_DREVICE], uiRxlen[MAX_USED_SPI_DREVICE], uidiff[MAX_USED_SPI_DREVICE];
static unsigned long ulTxThroughtput[MAX_USED_SPI_DREVICE], ulRxThroughtput[MAX_USED_SPI_DREVICE];
static bool fWrite[MAX_USED_SPI_DREVICE] = {0,0};
#endif

/* ################################################################################################################ */
/* Global Declarations */

/* Function Declarations */
static void mdm_spi_set_header_info(unsigned char *header_buffer, unsigned int curr_buf_size, unsigned int next_buf_size);
static int mdm_spi_get_header_info(struct mdm_spi_data *spi_data, unsigned int *valid_buf_size);
static void mdm_spi_set_mrdy_signal(s16 bus_num, int value);
static irqreturn_t mdm_spi_handle_srdy_irq(int irq, void *handle);
#ifndef TX_BUFFER_QUEUE
static void mdm_spi_setup_transmission(struct mdm_spi_data *spi_data);
static int mdm_spi_get_next_frame_size(int count);
static void mdm_spi_send_and_receive_data(struct mdm_spi_data *spi_data);
#else
static void mdm_spi_send_and_receive_data(struct mdm_spi_data *spi_data, int tx_pending);
#endif
static int mdm_spi_allocate_frame_memory(struct mdm_spi_data *spi_data, unsigned int memory_size);
static void mdm_spi_buffer_initialization(struct mdm_spi_data *spi_data);
static unsigned int mdm_spi_sync_read_write(struct mdm_spi_data *spi_data, unsigned int len);
static void mdm_spi_handle_work(struct work_struct *work);
static int mdm_spi_callback(void *client_data);
#ifdef TX_BUFFER_QUEUE
int get_tx_pending_data(struct mdm_spi_data *spi_data, int *anymore);
#endif

int mdm_modem_communicating(void);

#ifdef MDM6600_SPI_HEADER
static unsigned int tx_count[2] = {0, 0};
static unsigned int rx_count[2] = {0, 0};
#endif

unsigned char rx_dummy[]={0xff,0xff,0xff,0xff};

//#define ENABLE_SRDY_IRQ(irq)
//#define DISABLE_SRDY_IRQ(irq)

/* ################################################################################################################ */

static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);

#ifdef TX_BUFFER_QUEUE
#define MAX_SPI_DATA_QUEUE 200

struct spi_data_send_struct;
struct spi_data_send_struct {
    struct spi_data_send_struct *curr;
    struct spi_data_send_struct *next;
    struct spi_data_send_struct *prev;
    u8  count;
    u8 *data;
    int size;
    u8 *bkp_data;
    int bkp_size;
};
static struct spi_data_send_struct *spi_data_send_pending = NULL;
static int queue_first_time = 1;

//static spinlock_t spi_nodes_lock = SPIN_LOCK_UNLOCKED;
DEFINE_SPINLOCK(spi_nodes_lock);
static unsigned long spi_lock_flag;
static atomic_t next_transfer_flag = ATOMIC_INIT(0);
//static int timeout_count = 0;
#endif

/* ################################################################################################################ */

/* MDM SPI Operations */

static enum hrtimer_restart spi_timeout_cb_func(struct hrtimer *timer)
{
    struct spi_device *temp_spi = (struct spi_device *)global_spi;
	SPI_DEBUG("called ...........................................................\n");
    spi_tegra_abort_transfer(temp_spi);
    return HRTIMER_NORESTART;
}

#ifdef SPI_STATISTICS_CHECK
#define SPI_STAT_POLL_PERIOD (10)    // sec
static struct  timer_list  spi_statistics_timer;
static int statistics_count = 0;
static int dummy_data_flag = 0;

typedef enum {
    SPI_STAT_STATE_MIN=0,
    SPI_STAT_STATE_TXRX_NOT_CHG,
    SPI_STAT_STATE_RX_ONLY_NOT_CHG,
    SPI_STAT_STATE_STILL_NOT_CHG = SPI_STAT_STATE_RX_ONLY_NOT_CHG,
    SPI_STAT_STATE_DUMMY_TRANSACTION,
    SPI_STAT_STATE_DUMMY_TRANSACTION2,
    SPI_STAT_STATE_NO_TRANSACTION,
    SPI_STAT_STATE_MAX,
}spi_statistics_state_type;

static spi_statistics_state_type spi_s_state = SPI_STAT_STATE_MIN;


static void spi_statistic_cb_func(unsigned long unused)
{
    static unsigned int spi_tx_count = -1;
    static unsigned int spi_rx_count = -1;
    struct mdm_spi_data *spi_data = (struct mdm_spi_data *)gspi_data[0];
    int ret;
    int tx_chg_flg = (spi_tx_count == tx_count[0])?0:1;
    int rx_chg_flg = (spi_rx_count == rx_count[0])?0:1;

    statistics_count++;

    SPI_DEBUG("statistics_count=%d, spi_h_state=%d\n", statistics_count, spi_s_state);

    if(tx_chg_flg || rx_chg_flg) 
    {
        spi_s_state = SPI_STAT_STATE_MIN;
    } 
     else 
    {
        if((tx_chg_flg == 1)&&(rx_chg_flg==0)) 
        {
            spi_s_state=SPI_STAT_STATE_RX_ONLY_NOT_CHG;
        }
		
        switch(spi_s_state) 
        {
            case SPI_STAT_STATE_MIN:
                spi_s_state = SPI_STAT_STATE_TXRX_NOT_CHG;
                SPI_DEBUG("STATE_MIN: transaction is %d \n", spi_s_state);						
            break;
			
            case SPI_STAT_STATE_TXRX_NOT_CHG:
                spi_s_state = SPI_STAT_STATE_STILL_NOT_CHG;
                SPI_DEBUG("TXRX_NOT_CHG: transaction is %d \n", spi_s_state);						
            break;
			
            case SPI_STAT_STATE_STILL_NOT_CHG:
                spi_s_state = SPI_STAT_STATE_DUMMY_TRANSACTION;
                SPI_DEBUG("STILL_NOT_CHG: transaction is %d \n", spi_s_state);						
            break;
			
            case SPI_STAT_STATE_DUMMY_TRANSACTION:
                spi_s_state = SPI_STAT_STATE_DUMMY_TRANSACTION2;
                SPI_DEBUG("DUMMY_TRANSACTION: transaction is %d \n", spi_s_state);						
            break;
			
            case SPI_STAT_STATE_DUMMY_TRANSACTION2:
            case SPI_STAT_STATE_NO_TRANSACTION:
                spi_s_state = SPI_STAT_STATE_NO_TRANSACTION;
                SPI_DEBUG("DUMMY or NO_TRANSACTION: transaction is %d \n", spi_s_state);		
            break;
			
            case SPI_STAT_STATE_MAX:
            default:
                SPI_DEBUG(" Unknown SPI statistics State %d \n",  spi_s_state);
                spi_s_state = SPI_STAT_STATE_NO_TRANSACTION;
            break;
        }
    }

     SPI_DEBUG("spi_s_state = %d \n", spi_s_state);				

    if(spi_data->is_suspended) 
    {
        if(spi_s_state != SPI_STAT_STATE_NO_TRANSACTION) 
        {
            spi_s_state = SPI_STAT_STATE_MIN;
        }
    }

//    SPI_PRINTK("spi Tx : prev %d curr %d flg=%d\n", spi_tx_count, tx_count[0], tx_chg_flg);
//    SPI_PRINTK("spi Rx : prev %d curr %d flg=%d spi_h_state=%d\n", spi_rx_count, rx_count[0], rx_chg_flg, spi_s_state);

    if((spi_s_state == SPI_STAT_STATE_DUMMY_TRANSACTION)
      ||(spi_s_state == SPI_STAT_STATE_DUMMY_TRANSACTION2)) 
    {
        dummy_data_flag++;
        SPI_PRINTK("spi dummy transaction dummy_data_flag=%d, spi_s_state=%d \n", dummy_data_flag, spi_s_state);

        //prevent spi transmission being registered as workqueue event
        if(atomic_read(&spi_table[spi_data->index].in_use) == 1) 
        {
	    	queue_work(spi_data->mdm_wq, &spi_data->mdm_work);
        }
    }

    spi_tx_count = tx_count[0];
    spi_rx_count = rx_count[0];

    ret = mod_timer(&(spi_statistics_timer), round_jiffies(jiffies + (SPI_STAT_POLL_PERIOD*HZ)));
    if (ret)
    {
        SPI_PRINTK(" Error in mod_timer\n");
    }

    SPI_DEBUG("ended ...(%d, %d)\n", statistics_count, spi_s_state);

}
#endif

/*
 * Function copies the TX_BUFFER and RX_BUFFER pointer to a spi_transfer structure and add it to SPI tasks.
 * And calls SPI Driver function "spi_sync" to start data transmission and reception to from MODEM
 */

#define MAX_TRANSFER_FAILED 20
int count_transfer_failed = 0;

//check communication with modem
int mdm_modem_communicating(void)
{
	int ret = 1;

	if(count_transfer_failed >= MAX_TRANSFER_FAILED) 
	{     
		ret = 0;
	}

	return ret;
}

/*
 * Function opens a tty device when called from user space
 */
static int mdm_spi_open(struct tty_struct *tty, struct file *filp)
{
	int status = 0;
	struct mdm_spi_data *spi_data;
	int i;
	bool bfound = false;

    //prevent duplicated open of this driver by mux
    if(atomic_read(&spi_table[tty->index].in_use) == 1) 
    {
        SPI_PRINTK("spi already open, Open failed!!!\n");
        return -1;
    }

    //process only when 'spi_table[tty->index].allocated' is set to 1 in  mdm_spi_probe
    if(spi_table[tty->index].allocated)
    {
		//increase assign_num only in mdm_spi_probe
             for (i=0; i<assign_num; i++) 
             {
                 if (gspi_data[i]->index == tty->index) 
                 {
                     bfound = true;
                     break;
                 }
             }

            if (bfound) 
            {
                 spi_data = gspi_data[i];
                 spi_data->mdm_tty = tty;
                 tty->driver_data = spi_data;
                 mdm_spi_buffer_initialization(spi_data);
                 spi_data->throttle = 0;
     
                 atomic_set(&spi_table[tty->index].in_use, 1);
                 
                 SPI_PRINTK("spi Open success!!!\n");
                 
                 SPI_DEBUG(" tty->index=%d, spi_data->index=:%d\n", tty->index, spi_data->index);
     
               //set spi transfer wait timeout function: intialize timer, register the timeout callback function, assign global_spi to spi_data->spi
                 hrtimer_init(&(spi_data->timer), CLOCK_MONOTONIC, HRTIMER_MODE_REL);
                 spi_data->timer.function = spi_timeout_cb_func;
                 global_spi = spi_data->spi;
             }
             else
             {
                 SPI_PRINTK(" failed - no matching spi found!!\n");
                 status=-1;
             }

	}
	else
	{
		SPI_PRINTK(" failed!!\n");
		status=-1;
	}

#ifdef SPI_STATISTICS_CHECK
    spi_s_state = SPI_STAT_STATE_NO_TRANSACTION;
#endif

	return status;
}

/*
 * Function closes an opened tty device driver when called from user space
 */
static void mdm_spi_close(struct tty_struct *tty, struct file *filp)
{
#ifdef TX_BUFFER_QUEUE
	struct mdm_spi_data *spi_data = (struct mdm_spi_data *)tty->driver_data;
	int tx_anymore = 0;
	int queue_count = 0;
#endif

	//spi_data->mdm_tty = NULL;
	//tty->driver_data = NULL;

	SPI_PRINTK("start [tty id: %d]", tty->index);

	//assign the value 0 to 'in_use', inactivating it and blocking additional write requests occuring from the workqueue
	atomic_set(&spi_table[tty->index].in_use, 0);

#ifdef SPI_STATISTICS_CHECK
	spi_s_state = SPI_STAT_STATE_NO_TRANSACTION;
	SPI_DEBUG("spi_s_state %d \n", spi_s_state);
#endif

#if 0
    //wait until spi_sync is finished
    while(atomic_read(&spi_data->is_syncing) == 1) 
    {
	SPI_PRINTK(" is spi_syncing \n");
        msleep(500);
    }

    //prevent calling NULL timer callback function after processing 'mdm_spi_close'
    do
    {
        //when the timer is currently active apply 1 ms of sleep
        if (hrtimer_try_to_cancel(&(spi_data->timer)) == (-1)) 
	{
			//actual scheduling occurs through the execution of msleep(1)
            msleep(1);
        } 
	else 
	{
            SPI_DEBUG("hrtimer_try_to_cancel success!!!\n");
            break;
        }
    }while(1);

    //this log is for checking whether 'flush_workqueue' is finished
//    SPI_DEBUG("flush_workqueue()++ \n");

    //flush all threads currently queued in the workqueue
	flush_workqueue(spi_data->mdm_wq);

    //this log is for checking whether 'flush_workqueue' is finished
//	SPI_DEBUG(" flush_workqueue()-- \n");
#endif

#ifdef TX_BUFFER_QUEUE
	while( (0 != get_tx_pending_data(spi_data, &tx_anymore)) && (queue_count < MAX_SPI_DATA_QUEUE) )
	{
		queue_count++;
		SPI_PRINTK("tx_buffer_queue_count:%d",queue_count);
	}
#endif

	SPI_DEBUG("end \n");

}

/*
 * Function is called from user space to send data to MODEM, it setups the transmission, enable MRDY signal and
 * waits for SRDY signal HIGH from MDOEM. Then starts transmission and reception of data to and from MODEM.
 * Once data read from MODEM is transferred to TTY core flip buffers, then "mdm_read_write_completion" is set
 * and this function returns number of bytes sent to MODEM
 */
#ifdef TX_BUFFER_QUEUE
static int mdm_spi_write(struct tty_struct *tty, const unsigned char *buf, int count)
{
    struct mdm_spi_data *spi_data = (struct mdm_spi_data *)tty->driver_data;
    
    struct spi_data_send_struct *pSpiToSend;
    struct spi_data_send_struct *curr_ptr;
    
    int queue_size=0;
    u8 *send = NULL;
    
    if(spi_data==NULL)
    {
        SPI_PRINTK(" no spi handle\n");
        return 0;
    }
    
    //do not execute mdm_spi_write when mdm_spi_close has been processed
    if (atomic_read(&spi_table[spi_data->index].in_use) == 0)
    {
        SPI_PRINTK("open ttyspi first(#%d)\n");
        return 0;
    }
    
    //tty can be changable because of dual spi
    spi_data->mdm_tty = tty;
    spi_data->mdm_tty->low_latency = 1;
    
    if( !buf )
    {
        SPI_PRINTK("Buffer is NULL\n");
        return 0;
    }
    if(!count)
    {
        SPI_PRINTK("Count is ZERO\n");
        return 0;
    }
    
    curr_ptr = spi_data_send_pending;
//    SPI_PRINTK("mdm_spi_write()++ curr_ptr=0x%x, index %d, tx_len= %d\n", (uint)curr_ptr, spi_data->index, count);
    
    if((curr_ptr!=NULL)&&(curr_ptr->count>MAX_SPI_DATA_QUEUE))
    {
        //wait until completely processing "mdm_read_write" so that the buffer queue is filled
        spi_data->is_waiting = true;
        //SPI_PRINTK("waiting start\n");
        wait_for_completion(&spi_data->mdm_read_write_completion);
        //SPI_PRINTK("waiting end\n");
        init_completion(&spi_data->mdm_read_write_completion);
        
        //printk(KERN_ERR "mdm_spi_write()-- ERROR curr_ptr=0x%x count=%d FULL", (uint)curr_ptr, curr_ptr->count);
        //return 0;
    }
    
    send = kmalloc(count, GFP_ATOMIC);
    
    //WBT #196463
    if(NULL == send)
    {
        SPI_PRINTK("memory allocation for 'send' failed!\n");
        return 0;
    }
    
    pSpiToSend = kmalloc(sizeof(struct spi_data_send_struct), GFP_ATOMIC);
    
    //WBT #196462, #196464
    if(NULL == pSpiToSend)
    {
        SPI_PRINTK("memory allocation for 'pSpiToSend' failed!\n");
        //WBT #219191
        if(send)
            kfree(send);
        return 0;
    }
    
    memset((void*)pSpiToSend, 0, sizeof(struct spi_data_send_struct));
    memcpy(send, buf, count);
    
    pSpiToSend->data = send;
    pSpiToSend->size = count;
    
    spin_lock_irqsave(&spi_nodes_lock, spi_lock_flag);
    pSpiToSend->next = spi_data_send_pending;
    
    if(spi_data_send_pending!=NULL)
    {
        spi_data_send_pending->prev = pSpiToSend;
        queue_size = spi_data_send_pending->count;
    }
    else{
        queue_size = 0;
    }
    
    spi_data_send_pending = pSpiToSend;
    spi_data_send_pending->count = queue_size+1;
    spin_unlock_irqrestore(&spi_nodes_lock, spi_lock_flag);
    
    curr_ptr = spi_data_send_pending;
    
//    SPI_PRINTK("mdm_spi_write()::  curr_ptr=0x%x count=%d \n", (uint)curr_ptr, curr_ptr->count);
    
    //prevent spi transmission being registered as workqueue event
    if(atomic_read(&spi_table[spi_data->index].in_use) == 1)
    {
        queue_work(spi_data->mdm_wq, &spi_data->mdm_work);
    }
    
    return count;
}
#else // TX_BUFFER_QUEUE
static int mdm_spi_write(struct tty_struct *tty, const unsigned char *buf, int count)
{
	struct mdm_spi_data *spi_data = (struct mdm_spi_data *)tty->driver_data;

	SPI_DEBUG("(index=%d, %d size)\n", spi_data->index, count);

#ifdef SPI_SPEED_MEASUREMENT
	int id = tty->index; // spi no.
	unsigned long diff;
	fWrite[id] = 1;
	uiTxlen[id] = spi_data->mdm_spi_count + IFX_SPI_HEADER_SIZE;
	//ulStart = getuSecTime();
	do_gettimeofday(&ulStart[id]); //RTC(Real Time Clock)의 현재 실행시간
#endif

	if(spi_data==NULL)
	{
		SPI_PRINTK("no spi handle\n");
		return 0;
	}

	if( !buf )
	{
		SPI_PRINTK("buffer is NULL\n");
		return 0;
	}
	
	if(!count)
	{
		SPI_PRINTK("count is ZERO\n");
		return 0;
	}

#ifdef LGE_DUMP_SPI_BUFFER
	dump_spi_buffer("ifx_spi_write()", buf, count);	
#endif
	spi_data->mdm_ret_count = 0;	
	spi_data->mdm_tty = tty;
	spi_data->mdm_tty->low_latency = 1;	
	spi_data->mdm_master_initiated_transfer = 1;
	spi_data->mdm_spi_buf = buf;
	spi_data->mdm_spi_count = count;

//	DISABLE_SRDY_IRQ(spi_data->spi->irq);
	//mdm_spi_set_mrdy_signal(spi_data->spi->master->bus_num, 1);

	//prevent spi transmission being registered as workqueue event
	if(atomic_read(&spi_table[spi_data->index].in_use) == 1) 
	{			    	
		queue_work(spi_data->mdm_wq, &spi_data->mdm_work);
	}

	wait_for_completion(&spi_data->mdm_read_write_completion);

#ifdef SPI_SPEED_MEASUREMENT
		//ulEnd = getuSecTime() - ulStart;
		do_gettimeofday(&ulEnd[id]);
		diff = (ulEnd[id].tv_sec - ulStart[id].tv_sec) * 1000 * 1000; //sec
		diff = (diff + (ulEnd[id].tv_usec - ulStart[id].tv_usec));
		ulTxThroughtput[id] = ((uiTxlen[id]*8*1000)/diff);
		SPI_PRINTK("                    [SPI %d] : TX time = %09d usec; %04d bytes; %06lu Kbps", 
					id, diff, IFX_SPI_MAX_BUF_SIZE+IFX_SPI_HEADER_SIZE, 
					((IFX_SPI_MAX_BUF_SIZE+IFX_SPI_HEADER_SIZE)*8*1000)/diff);
		uiTxlen[id];
		fWrite[id] = 0;
#endif

//	ENABLE_SRDY_IRQ(spi_data->spi->irq);
	init_completion(&spi_data->mdm_read_write_completion);
	return spi_data->mdm_ret_count; /* Number of bytes sent to the device */
}
#endif // TX_BUFFER_QUEUE


/* This function should return number of free bytes left in the write buffer, in this case always return 2048 */
static int mdm_spi_write_room(struct tty_struct *tty)
{
	return IFX_SPI_MAX_BUF_SIZE;
}


/* ################################################################################################################ */
/* These two functions are to be used in future to implement flow control (RTS & CTS)*/
/*static void
mdm_spi_throttle(struct tty_struct *tty)
{
	unsigned int flags;
	struct mdm_spi_data *spi_data = (struct mdm_spi_data *)tty->driver_data;
	spi_data->mdm_tty = tty;
	spin_lock_irqsave(&spi_data->spi_lock, flags);
	spi_data->throttle = 1;
	spin_unlock_irqrestore(&spi_data->spi_lock, flags);
}

static void
mdm_spi_unthrottle(struct tty_struct *tty)
{
	unsigned int flags;
	struct mdm_spi_data *spi_data = (struct mdm_spi_data *)tty->driver_data;
	spi_data->mdm_tty = tty;
	spin_lock_irqsave(&spi_data->spi_lock, flags);
	spi_data->throttle = 0;
	if( mdm_rx_buffer != NULL ){
	     tty_insert_flip_string(spi_data->mdm_tty, mdm_rx_buffer, valid_buffer_count);
	}
	spin_unlock_irqrestore(&spi_data->spi_lock, flags);
}*/
/* ################################################################################################################ */

/* End of MDM SPI Operations */

/* ################################################################################################################ */

/* TTY - SPI driver Operations */
static int mdm_spi_probe(struct spi_device *spi)
{
    int status;
    struct mdm_spi_data *spi_data;
    
    SPI_DEBUG("(assign_num=%d)\n", assign_num);
    
    /* memory allocation for SPI driver data */
    spi_data = (struct mdm_spi_data*)kmalloc(sizeof(struct mdm_spi_data), GFP_KERNEL);
    if (!spi_data)
    {
        SPI_PRINTK("spi_data is null\n");		
        return -ENOMEM;
     }
    
    //spi_data->mdm_tty = NULL;
    //initialize all member variables of mdm_spi_data
    memset((void*)spi_data, 0, sizeof(struct mdm_spi_data));

//    SPI_PRINTK ("mdm_spi_buf = %d, mdm_spi_count = %d", spi_data->mdm_spi_buf, spi_data->mdm_spi_count);
     spi_data->mdm_tty = NULL;
#ifdef CONFIG_HAS_WAKELOCK	 
     spi_data->wake_lock_flag = 0;
     spi_data->is_suspended = 0;
#endif
    status = mdm_spi_allocate_frame_memory(spi_data, IFX_SPI_FRAME_SIZE);
    if(status != 0) 
    {
    	SPI_PRINTK("Failed to allocate memory for buffers\n");
    
    	if(spi_data) 
    	{
    		kfree(spi_data);
    	}
    	return -ENOMEM;
     }

    dev_set_drvdata(&spi->dev,spi_data);
    spin_lock_init(&spi_data->spi_lock);
    INIT_WORK(&spi_data->mdm_work, mdm_spi_handle_work);

    //workqueue thread must be created in the probe function, because of the mrdy interrupt coming from cp
    spi_data->mdm_wq = create_singlethread_workqueue("mdm6600");
    if(!spi_data->mdm_wq)
    {
        SPI_PRINTK("Failed to setup workqueue - mdm_wq \n");
    }

    init_completion(&spi_data->mdm_read_write_completion);

#ifdef TX_BUFFER_QUEUE
    spi_data->is_waiting = false;
#endif

//config srdy,mrdy gpio value according to hw revision
#if defined (CONFIG_LU8800) || defined (CONFIG_KS1103)
    IPC_MRDY1 = TEGRA_GPIO_PO0; //ULPI_DATA7
    IPC_SRDY1 = TEGRA_GPIO_PO5; //ULPI_DATA4
#ifdef CONFIG_DUAL_SPI
    IPC_MRDY2 = TEGRA_GPIO_PJ6; 
    IPC_SRDY2 = TEGRA_GPIO_PU6;    
#endif //CONFIG_DUAL_SPI
#else
    IPC_MRDY1 = TEGRA_GPIO_PJ6; 
    IPC_SRDY1 = TEGRA_GPIO_PU6;
#ifdef CONFIG_DUAL_SPI
    IPC_MRDY2 = TEGRA_GPIO_PO0; 
    IPC_SRDY2 = TEGRA_GPIO_PO5;
#endif //CONFIG_DUAL_SPI	
#endif //CONFIG_LU8800

    /* configure SPI controller */
    spi_data->spi = spi;
    spi->mode = SPI_MODE_1 | SPI_CS_HIGH;
    spi->bits_per_word = 32;

    /* get the spi bus id : ex) spi0.0 */
    spi_data->index = spi->master->bus_num;

    /* configure SPI chip_select */
    switch (spi_data->index) 
    {
    	case 0:
    	case 1:
        	spi->chip_select = 0;
        	break;
			
    	case 2:
        	spi->chip_select = 2;
        	break;
			
    	default:
        	spi->chip_select = 0;
        	break;
    }

    SPI_DEBUG("chip_select %d", spi->chip_select);
    //must be set to 4 times the speed of the actual running clock
    spi->max_speed_hz = 4*24*1000*1000;

    status = spi_setup(spi);
    if(status < 0)
    {
        SPI_PRINTK("Failed to setup SPI \n");
    }

    SPI_DEBUG(" (bus_num=%d)\n", spi->master->bus_num);
    spi_tegra_register_callback(spi, mdm_spi_callback, (void*)&spi->master->bus_num);

#ifdef CONFIG_DUAL_SPI //disables spi secondary port
	if(0 == spi_data->index) 
#endif		
	{
		/* IPC_MRDY1 */
		gpio_request(IPC_MRDY1, "ipc_mrdy1");
		tegra_gpio_enable(IPC_MRDY1);
		gpio_direction_output(IPC_MRDY1, 0);

		/* IPC_SRDY1 */
		gpio_request(IPC_SRDY1, "ipc_srdy1");
		tegra_gpio_enable(IPC_SRDY1);
		gpio_direction_input(IPC_SRDY1);
		spi->irq = gpio_to_irq(IPC_SRDY1);

		SPI_DEBUG(" irq = %d, name = %s\n", spi->irq, spi->dev.driver->name);

		status = request_irq(spi->irq, mdm_spi_handle_srdy_irq, 
						  (IRQF_TRIGGER_FALLING |IRQF_TRIGGER_RISING),
						  spi->dev.driver->name, spi_data);
						  
#ifdef CONFIG_DUAL_SPI
		// set the IPC_SRDY2 gpio pin as an ap_suspend_state gpio pin  for checking the suspend state of the ap
		gpio_request(IPC_SRDY2, "ap_suspend_state");
		tegra_gpio_enable(IPC_SRDY2);
		gpio_direction_output(IPC_SRDY2, 0);
#endif		
	}
#ifdef CONFIG_DUAL_SPI //disables spi secondary port
	else 
	{
		/* IPC_MRDY2 */
		gpio_request(IPC_MRDY2, "ipc_mrdy2");
		tegra_gpio_enable(IPC_MRDY2);
		gpio_direction_output(IPC_MRDY2, 0);

		/* IPC_SRDY2 */
		gpio_request(IPC_SRDY2, "ipc_srdy2");
		tegra_gpio_enable(IPC_SRDY2);
		gpio_direction_input(IPC_SRDY2);
		spi->irq = gpio_to_irq(IPC_SRDY2);

		SPI_DEBUG(" irq = %d, name = %s\n", spi->irq, spi->dev.driver->name);

        status = request_irq(spi->irq, mdm_spi_handle_srdy_irq, 
    					(IRQF_TRIGGER_FALLING |IRQF_TRIGGER_RISING), 
    					spi->dev.driver->name, spi_data);
	}
#endif

	if (status != 0) 
	{
	    SPI_PRINTK("Failed to request IRQ for MRDY\n");

	    if(spi_data->mdm_tx_buffer) 
	    {
		    kfree(spi_data->mdm_tx_buffer);
	    }
		
	    if(spi_data->mdm_rx_buffer) 
	    {
		    kfree(spi_data->mdm_rx_buffer);
	    }
	    if(spi_data) 
	    {
		    kfree(spi_data);
	    }
	    return status;
    }

   enable_irq_wake(spi->irq);

    spi_data->mdm_master_initiated_transfer = 0;

    gspi_data[assign_num] = spi_data;

	//mdm_spi_sync_read_write(spi_data, IFX_SPI_HEADER_SIZE); /* 4 bytes for header */
    spi_table[spi_data->index].allocated = 1;
    /* increase assigned number of spi driver */
    assign_num++;

#ifdef CONFIG_HAS_WAKELOCK
    wake_lock_init(&spi_data->spi_wakelock, WAKE_LOCK_SUSPEND, "mspi_wake");
#endif

#ifdef SPI_STATISTICS_CHECK
    if(status == 0) 
    {
        int ret = 0;
        SPI_DEBUG("statistics timer setting !!\n");
        setup_timer(&spi_statistics_timer, spi_statistic_cb_func, 0);
        ret = mod_timer( &spi_statistics_timer, round_jiffies(jiffies + SPI_STAT_POLL_PERIOD*HZ));
        if (ret) 
        {
            SPI_PRINTK(" Error in mod_timer !!\n");
        }
    }
#endif
    SPI_DEBUG(" end\n");

	return status;
}

static int mdm_spi_remove(struct spi_device *spi)
{
	struct mdm_spi_data *spi_data;

	spi_data = spi_get_drvdata(spi);

#ifdef CONFIG_HAS_WAKELOCK
	wake_lock_destroy(&spi_data->spi_wakelock);
#endif

	spin_lock_irq(&spi_data->spi_lock);
	spi_data->spi = NULL;
	spi_set_drvdata(spi, NULL);
	spin_unlock_irq(&spi_data->spi_lock);

	if(spi_data->mdm_tx_buffer) 
	{
		kfree(spi_data->mdm_tx_buffer);
	}
	
	if(spi_data->mdm_rx_buffer) 
	{
		kfree(spi_data->mdm_rx_buffer);
	}
	
       if(spi_data) 
       {
		kfree(spi_data);
        }
    return 0;
}


#ifdef CONFIG_PM
static int mdm_spi_suspend(struct spi_device *spi, pm_message_t mesg)
{
	struct mdm_spi_data *spi_data;
	unsigned long flags;
	unsigned long reg;

	SPI_PRINTK("is enable ");

	spi_data = spi_get_drvdata(spi);

        //add to flush workqueue
	flush_workqueue(spi_data->mdm_wq);

	//prevent duplicated access to 'spi_data->is_suspended'
	spin_lock_irqsave(&spi_data->spi_lock, flags);
	spi_data->is_suspended = 1;
	spin_unlock_irqrestore(&spi_data->spi_lock, flags);

	SPI_PRINTK("is_suspended=%d\n", spi_data->is_suspended);

        // address를 주면 값을 read
	reg = readl(pmc_base + PMC_WAKE_STATUS);

	// Clear power key wakeup pad bit.
	if (reg & WAKEUP_MDM_SRDY_MASK)
	{
		SPI_PRINTK("wakeup pad : 0x%lx", reg);
             //주소와 값을 주면 주소에 값을 write
		writel(WAKEUP_MDM_SRDY_MASK, pmc_base + PMC_WAKE_STATUS);
	}
//	else
//	{
//		SPI_PRINTK("do not pmc  writel()!!, reg = 0x%lx\n", reg);		
//	}

#ifdef CONFIG_DUAL_SPI 
	//set 'ap_suspend_state' gpio value to 'true'
	gpio_set_value(IPC_SRDY2, spi_data->is_suspended);
	SPI_DEBUG("(gpio=%d, ap_suspend_state=%d)", IPC_SRDY2, gpio_get_value(IPC_SRDY2));
#endif
	SPI_PRINTK("is end ");
	return 0;
}

static int mdm_spi_resume(struct spi_device *spi)
{
	struct mdm_spi_data *spi_data;
	unsigned long flags;
	unsigned long reg;

	SPI_PRINTK("is enable");

	spi_data = spi_get_drvdata(spi);

	SPI_PRINTK(" (is_suspend=%d)", spi_data->is_suspended);

	reg = readl(pmc_base + PMC_WAKE_STATUS);

	if (reg & WAKEUP_MDM_SRDY_MASK)
	{
		//writel(WAKEUP_MDM_SRDY_MASK, pmc_base + PMC_WAKE_STATUS);
		SPI_PRINTK(" wakeup pad : 0x%lx\n", reg);

#ifdef CONFIG_HAS_WAKELOCK
		if(&gspi_data[0]->spi_wakelock)
		{
			wake_lock_timeout(&gspi_data[0]->spi_wakelock, msecs_to_jiffies(3000));   //500ms
		}

		gspi_data[0]->wake_lock_flag = 1;		
#endif
		queue_work(gspi_data[0]->mdm_wq, &gspi_data[0]->mdm_work);
	}
//	else
//	{
//		SPI_PRINTK("do not queue work!, reg = 0x%lx\n", reg);		
//	}	

	//prevent duplicated access to 'spi_data->is_suspended'
	spin_lock_irqsave(&spi_data->spi_lock, flags);
	spi_data->is_suspended = 0;
	spin_unlock_irqrestore(&spi_data->spi_lock, flags);

#ifdef CONFIG_DUAL_SPI 
    //set ap_suspend state gpio value to 'false'
    gpio_set_value(IPC_SRDY2, spi_data->is_suspended);
    SPI_DEBUG(" (gpio=%d, ap_suspend_state=%d)", IPC_SRDY2, gpio_get_value(IPC_SRDY2));
#endif
	SPI_PRINTK("is end ");
	return 0;
}
#endif

/* End of TTY - SPI driver Operations */

/* ################################################################################################################ */

static struct spi_driver mdm_spi_driver = {
	.driver = {
		.name = "mdm6600",
		.bus = &spi_bus_type,
		.owner = THIS_MODULE,
	},
	.probe = mdm_spi_probe,
	.remove = __devexit_p(mdm_spi_remove),
//	.shutdown = mdm_spi_shutdown,
#ifdef CONFIG_PM
	.suspend = mdm_spi_suspend,
	.resume = mdm_spi_resume,
#endif
};

/*
 * Structure to specify info for the tty core about tty driver operations supported in TTY SPI driver.
 */
static const struct tty_operations mdm_spi_ops = {
    .open = mdm_spi_open,
    .close = mdm_spi_close,
    .write = mdm_spi_write,
    .write_room = mdm_spi_write_room,
    //.throttle = mdm_spi_throttle,
    //.unthrottle = mdm_spi_unthrottle,
    //.set_termios = mdm_spi_set_termios,
};

/* ################################################################################################################ */

/*
 * Intialize frame sizes as "IFX_SPI_DEFAULT_BUF_SIZE"(2044) bytes for first SPI frame transfer
 */
static void mdm_spi_buffer_initialization(struct mdm_spi_data *spi_data)
{
         spi_data->mdm_sender_buf_size = IFX_SPI_DEFAULT_BUF_SIZE;
         spi_data->mdm_receiver_buf_size = IFX_SPI_DEFAULT_BUF_SIZE;
}

/*
 * Allocate memory for TX_BUFFER and RX_BUFFER
 */
static int mdm_spi_allocate_frame_memory(struct mdm_spi_data *spi_data, unsigned int memory_size)
{
	int status = 0;

	SPI_DEBUG(" (assign_num=%d)\n", assign_num);

	spi_data->mdm_rx_buffer = kmalloc(memory_size+IFX_SPI_HEADER_SIZE, GFP_KERNEL);
	if (!spi_data->mdm_rx_buffer) 
	{
		SPI_PRINTK( "Open Failed ENOMEM\n");
		status = -ENOMEM;
	}
	
	spi_data->mdm_tx_buffer = kmalloc(memory_size+IFX_SPI_HEADER_SIZE, GFP_KERNEL);
	if (!spi_data->mdm_tx_buffer) 
	{
		SPI_PRINTK( "Open Failed ENOMEM\n");
		status = -ENOMEM;
	}
	
	if(status == -ENOMEM)
	{
		if(spi_data->mdm_tx_buffer)
		{
			kfree(spi_data->mdm_tx_buffer);
		}
		
		if(spi_data->mdm_rx_buffer)
		{
			kfree(spi_data->mdm_rx_buffer);
		}
	}
	return status;
}

/*
 * Function to set header information according to IFX SPI framing protocol specification
 */
static void mdm_spi_set_header_info(unsigned char *header_buffer, unsigned int curr_buf_size, unsigned int next_buf_size)
{
	int i;
	union mdm_spi_frame_header header;

	for(i=0; i<4; i++)
	{
		header.framesbytes[i] = 0;
	}

	header.mdm_spi_header.curr_data_size = curr_buf_size;
	if(next_buf_size)
	{
		header.mdm_spi_header.more=1;
		header.mdm_spi_header.next_data_size = next_buf_size;
	}
	else
	{
		header.mdm_spi_header.more=0;
		header.mdm_spi_header.next_data_size = 128;
	}

	for(i=3; i>=0; i--)
	{
		header_buffer[i] = header.framesbytes[i];
	}
}

/*
 * Function to get header information according to IFX SPI framing protocol specification
 */
static int mdm_spi_get_header_info(struct mdm_spi_data *spi_data, unsigned int *valid_buf_size)
{
	int i;
	union mdm_spi_frame_header header;

	for(i=0; i<4; i++)
	{
		header.framesbytes[i] = 0;
	}

	for(i=3; i>=0; i--)
	{
		header.framesbytes[i] = spi_data->mdm_rx_buffer[i];
	}

	if(header.mdm_spi_header.curr_data_size<=IFX_SPI_DEFAULT_BUF_SIZE)
	{
		// check rx size
		SPI_DEBUG( "check rx data .. curr_data_size = %d\n", header.mdm_spi_header.curr_data_size);		
	    *valid_buf_size = header.mdm_spi_header.curr_data_size;
	}
	else
	{
		*valid_buf_size = 0;
		SPI_PRINTK( " rx data exceed buffer size = %d\n", header.mdm_spi_header.curr_data_size);
	}

	if(header.mdm_spi_header.more)
	{
		// check next rx size
		if(header.mdm_spi_header.next_data_size<=IFX_SPI_DEFAULT_BUF_SIZE)
		{
			return header.mdm_spi_header.next_data_size;
		}
	}
	return 0;
}

static void mdm_spi_clear_header_info(unsigned int *hdr_ptr)
{
    *(hdr_ptr) = 0;
    *(hdr_ptr+1) = 0;
}

#ifdef MDM6600_SPI_HEADER
static void mdm_spi_set_tx_frame_count(unsigned int *hdr_ptr, unsigned int id)
{
    *(hdr_ptr+1) = ++(tx_count[id]);
    if(*(hdr_ptr+1)==0) *(hdr_ptr+1) = ++(tx_count[id]);
//    SPI_PRINTK( "Tx Count %d\n", tx_count[id]);
}

static unsigned int mdm_spi_get_rx_frame_count(unsigned int *hdr_ptr)
{
    return *(hdr_ptr+1);
}

static int mdm_spi_get_remote_chk(unsigned long *hdr_ptr)
{
    if(*(hdr_ptr) & (0x00002000))
    {
	    return 1;
    }
    return 0;
}
#endif

/*
 * Function to set/unset SRDY signal
 */
static void mdm_spi_set_mrdy_signal(s16 bus_num, int value)
{
	int gpio_num = 0;
//	SPI_DEBUG("  bus_num=%d\n",  bus_num);

#ifdef CONFIG_DUAL_SPI //disables spi secondary port
	if(bus_num == 0) 
#endif
	{
		gpio_set_value(IPC_MRDY1, value);
		gpio_num = IPC_MRDY1;
	} 
#ifdef CONFIG_DUAL_SPI //disables spi secondary port
	else if(bus_num == 1) 
	{
		gpio_set_value(IPC_MRDY2, value);
		gpio_num = IPC_MRDY2;
	}
#endif
//	SPI_DEBUG(" gpio=%d, srdy=%d\n",  gpio_num, value);

}


static int mdm_spi_get_srdy_signal(s16 bus_num)
{
	int gpio_num = 0;
    int value = 0;

//	SPI_DEBUG("  bus_num=%d\n",  bus_num);

#ifdef CONFIG_DUAL_SPI //disables spi secondary port
	if(bus_num == 0) 
#endif
	{
		value = gpio_get_value(IPC_SRDY1);
		gpio_num = IPC_SRDY1;
	} 
#ifdef CONFIG_DUAL_SPI //disables spi secondary port
	else if(bus_num == 1) 
	{
		value = gpio_get_value(IPC_SRDY2);
		gpio_num = IPC_SRDY2;
	}
#endif

//	SPI_DEBUG("  gpio=%d, srdy=%d\n", gpio_num, value);

    return value;
}


static int	mdm_spi_callback(void *client_data)
{
	s16 *bus_num;
	bus_num = (s16*)client_data;

//	SPI_DEBUG("called\n");
//	SPI_DEBUG("bus_num=%d\n",*bus_num);
	mdm_spi_set_mrdy_signal(*bus_num, 1);
	return 0;
}

#ifndef TX_BUFFER_QUEUE
/*
 * Function to calculate next_frame_size required for filling in SPI frame Header
 */
static int mdm_spi_get_next_frame_size(int count)
{
//  SPI_PRINTK("mdm_spi_count=%d\n",count);
	if(count > IFX_SPI_MAX_BUF_SIZE)
	{
		return IFX_SPI_MAX_BUF_SIZE;
	}
	else
	{
		return count;
	}
}

/*
 * Function to setup transmission and reception. It implements a logic to find out the mdm_current_frame_size,
 * valid_frame_size and sender_next_frame_size to set in SPI header frame. Copys the data to be transferred from
 * user space to TX buffer and set MRDY signal to HIGH to indicate Master is ready to transfer data.
 */
static void mdm_spi_setup_transmission(struct mdm_spi_data *spi_data)
{
    if( (spi_data->mdm_sender_buf_size != 0) || (spi_data->mdm_receiver_buf_size != 0) )
    {

        // current_frame_size must be IFX_SPI_MAX_BUF_SIZE
        spi_data->mdm_current_frame_size = IFX_SPI_MAX_BUF_SIZE;

        if(spi_data->mdm_spi_count > 0) // tx
        {
            if(spi_data->mdm_spi_count > spi_data->mdm_current_frame_size)
            {
                spi_data->mdm_valid_frame_size = spi_data->mdm_current_frame_size;
                spi_data->mdm_spi_count = spi_data->mdm_spi_count - spi_data->mdm_current_frame_size;
            }
            else
            {
                spi_data->mdm_valid_frame_size = spi_data->mdm_spi_count;
                spi_data->mdm_spi_count = 0;
            }
        }
        else // rx
        {
            spi_data->mdm_valid_frame_size = 0;
            spi_data->mdm_sender_buf_size = 0;
        }
        spi_data->mdm_sender_buf_size = mdm_spi_get_next_frame_size(spi_data->mdm_spi_count);

        /* memset buffers to 0 */
        memset(spi_data->mdm_tx_buffer, 0, IFX_SPI_MAX_BUF_SIZE+IFX_SPI_HEADER_SIZE);
        memset(spi_data->mdm_rx_buffer, 0, IFX_SPI_MAX_BUF_SIZE+IFX_SPI_HEADER_SIZE);

        /* Set header information */
        mdm_spi_set_header_info(spi_data->mdm_tx_buffer, spi_data->mdm_valid_frame_size, spi_data->mdm_sender_buf_size);
        if( spi_data->mdm_valid_frame_size > 0 ) // tx
        {
#ifdef MDM6600_SPI_HEADER
            mdm_spi_set_tx_frame_count((unsigned int *)spi_data->mdm_tx_buffer, spi_data->index);
#endif
            memcpy(spi_data->mdm_tx_buffer+IFX_SPI_HEADER_SIZE, spi_data->mdm_spi_buf, spi_data->mdm_valid_frame_size);
            spi_data->mdm_spi_buf = spi_data->mdm_spi_buf + spi_data->mdm_valid_frame_size;
        }
    }
}
#endif

#define MORE_DATA   0x00001000
#define CURR_DATA   0x00000FFF
#define NEXT_DATA   0x0FFF0000

static int
check_valid_rx_frame_header(unsigned int *hdr_ptr, int index)
{
  int retval=1;
  if(*(hdr_ptr) == 0xFFFFFFFF){
    retval = 0;
  }
  else if(*(hdr_ptr) == 0){
    retval = 0;
  }
  else if( (*(hdr_ptr)&(~(0x0FFF3FFF))) != 0 ){
    retval = 0;
  }
  else if((*(hdr_ptr+1)==0)){
    if( *(hdr_ptr) == 0x00800000 ) retval = 1;
    else if( *(hdr_ptr) == 0x00802000 ) retval = 1;
    else retval = 0;
  }
  else if( (*(hdr_ptr+1)!=0) ){
    if(( (*(hdr_ptr)&MORE_DATA) != 0 )&&( (*(hdr_ptr)&((NEXT_DATA))) == 0 )) retval = 0;
    else if(( (*(hdr_ptr)&MORE_DATA) == 0 )&&( (*(hdr_ptr)&((NEXT_DATA))) != 0x00800000 )) retval = 0;
    else if( *(hdr_ptr+1) == rx_count[index]) retval = 0;
    else retval = 1;
  }
  else{
    retval = 1;
  }
  SPI_DEBUG("check_valid_rx_frame_header *(hdr_ptr)=0x%x retval=%d \n", *(hdr_ptr), retval);
  return(retval);
}

/*
 * Function starts Read and write operation and transfers received data to TTY core. It pulls down MRDY signal
 * in case of single frame transfer then sets "mdm_read_write_completion" to indicate transfer complete.
 */
#define MAX_RETRY_COUNT 3
#ifdef TX_BUFFER_QUEUE
static void mdm_spi_send_and_receive_data(struct mdm_spi_data *spi_data, int tx_pending)
{
    unsigned int rx_valid_buf_size;
    int status = 0;
    int retry_count = 0;
    
    //WBT #196460
    unsigned int prev_rx_frame = 0;
    int mReTransFlag=0;
    int mValidRxFrame=0;
    
    if(atomic_read(&spi_table[spi_data->index].in_use)==0)
    {
        return;
    }
    
    status = mdm_spi_sync_read_write(spi_data, IFX_SPI_FRAME_SIZE); /* 4 bytes for header */
    
    //resend tx buffer when spi transfer timeout happens and max_retry_count is smaller than MAX_RETRY_COUNT
#if 0
    while((status == -EAGAIN)
      && (retry_count < MAX_RETRY_COUNT)
      && (tx_pending != 0)
      && (check_valid_rx_frame_header((unsigned int *)spi_data->mdm_rx_buffer) == 0) )
    {
        TTYSPI_DEBUG_PRINT("\n SPI Transaction Error : status = %d, retry_count=%d tx_pending=%d \n", status, retry_count, tx_pending);
        mdelay(10);
        status = mdm_spi_sync_read_write(spi_data, IFX_SPI_FRAME_SIZE);
        retry_count++;
    }
#else
    do{
        mReTransFlag = 0;
        
        mValidRxFrame = check_valid_rx_frame_header((unsigned int *)spi_data->mdm_rx_buffer, spi_data->index);
        
        if((status == -EAGAIN)
          &&(tx_pending != 0)
          &&(mValidRxFrame == 0))
        {  // TX Frame & spi internal error checking
            SPI_PRINTK("SPI Transaction Tx Error : status = %d, retry_count=%d tx_pending=%d \n", status, retry_count, tx_pending);
            mReTransFlag = 1;
            mdelay(10);
        }
        else
        { // Rx frame checking
            if(mValidRxFrame == 0)
            {
                SPI_PRINTK("SPI Transaction Rx Error : status = %d, retry_count=%d tx_pending=%d \n", status, retry_count, tx_pending);
                mReTransFlag = 1;
                mdelay(50);
            }
            else
            {
                mReTransFlag = 0;
            }
        }
        
        if(mReTransFlag == 1)
        {
            status = mdm_spi_sync_read_write(spi_data, IFX_SPI_FRAME_SIZE);
            retry_count++;
        }
    }
    while((mReTransFlag == 1)&&(retry_count<MAX_RETRY_COUNT));
#endif //if 0
    
    if(memcmp(rx_dummy, spi_data->mdm_rx_buffer, sizeof(rx_dummy)) ==0)
    {
      spi_data->mdm_receiver_buf_size = 0;
      SPI_PRINTK("Rx data is dummy !!! \n");
      return;
    }
    
    /* Handling Received data */
    spi_data->mdm_receiver_buf_size = mdm_spi_get_header_info(spi_data, &rx_valid_buf_size);
    
#ifdef MDM6600_SPI_HEADER
    if(rx_valid_buf_size !=0 )
    {
        prev_rx_frame = rx_count[spi_data->index];
        rx_count[spi_data->index] = mdm_spi_get_rx_frame_count((unsigned int *)spi_data->mdm_rx_buffer);
        
        SPI_DEBUG("Rx Count : prev_rx_frame = %d, rx_count=%d \n", prev_rx_frame, rx_count[spi_data->index]);
        
        if(rx_count[spi_data->index] == prev_rx_frame)
        {
            SPI_PRINTK("SPI RX Error : duplicated : drop the frame :: prev_rx_frame = %d, rx_count=%d \n", prev_rx_frame, rx_count[spi_data->index]);
            rx_valid_buf_size = 0;   // drop
            rx_count[spi_data->index] = prev_rx_frame;
        }
        else if(tx_count[spi_data->index] == 0)
        {
            SPI_PRINTK("SPI RX Error : NO Tx count rx bytes :: prev_rx_frame = %d, rx_count=%d \n", prev_rx_frame, rx_count[spi_data->index]);
        }
        else if(rx_count[spi_data->index] != (prev_rx_frame+1))
        {
            SPI_PRINTK("SPI RX Error : prev_rx_frame = %d, rx_count=%d \n", prev_rx_frame, rx_count[spi_data->index]);
        }
    }
#endif
    
    //below statement needs to be disabled for spi echo command test, but needs to be enabled for uploading data from spi driver to ttyspi driver.
    if((spi_data->throttle == 0)
      && (rx_valid_buf_size != 0)
      && (spi_data->mdm_tty!=NULL)
      && (atomic_read(&spi_table[spi_data->index].in_use) != 0)) //do not send tty layer after process mdm_spi_close
    {
        tty_insert_flip_string(spi_data->mdm_tty, spi_data->mdm_rx_buffer+IFX_SPI_HEADER_SIZE, rx_valid_buf_size);
        tty_flip_buffer_push(spi_data->mdm_tty);
    }
    
#ifdef MDM6600_SPI_HEADER
    status = mdm_spi_get_remote_chk((unsigned long *)spi_data->mdm_rx_buffer);
    if(status != 0 )
    {
        SPI_PRINTK("SPI Remote CHK : prev_rx_frame = %d, rx_count=%d \n", prev_rx_frame, rx_count[spi_data->index]);
    }
#endif
}

#else // TX_BUFFER_QUEUE

static void mdm_spi_send_and_receive_data(struct mdm_spi_data *spi_data)
{
    unsigned int rx_valid_buf_size;
    int status = 0;

    SPI_DEBUG(" start\n");

	//status = mdm_spi_sync_read_write(spi_data, spi_data->mdm_current_frame_size+IFX_SPI_HEADER_SIZE); /* 4 bytes for header */
    status = mdm_spi_sync_read_write(spi_data, IFX_SPI_MAX_BUF_SIZE+IFX_SPI_HEADER_SIZE); /* 4 bytes for header */
#if defined(LGE_DUMP_SPI_BUFFER)
    dump_spi_buffer("SPI TX", &spi_data->mdm_tx_buffer[4], COL_SIZE);
#endif
#ifdef SPI_BUFFER_DUMP_LOG
    if (spi_data->mdm_tx_buffer)
    {
        SPI_PRINTK("SPI TX  ----------------- \n");        
        dump_atcmd(spi_data->mdm_tx_buffer+IFX_SPI_HEADER_SIZE+2);	
    }
    else
    {
        SPI_PRINTK("SPI TX  Buffer  is NULL !!!!!");	
    }		
#endif	
    if(status > 0)
    {
        memset(spi_data->mdm_tx_buffer, 0, IFX_SPI_MAX_BUF_SIZE+IFX_SPI_HEADER_SIZE);
        spi_data->mdm_ret_count = spi_data->mdm_ret_count + spi_data->mdm_valid_frame_size;
    }
    SPI_DEBUG(" mdm_ret_count = %d\n", spi_data->mdm_ret_count);

    if(memcmp(rx_dummy, spi_data->mdm_rx_buffer, sizeof(rx_dummy)) ==0) {
      spi_data->mdm_receiver_buf_size = 0;
      SPI_PRINTK("Rx data is dummy !!! \n");
      return;
    }

    /* Handling Received data */
    spi_data->mdm_receiver_buf_size = mdm_spi_get_header_info(spi_data, &rx_valid_buf_size);
    SPI_DEBUG(" mdm_receiver_buf_size = %d, rx_valid_buf_size = %d\n", spi_data->mdm_receiver_buf_size, rx_valid_buf_size);


	//if(!rx_valid_buf_size)
	//{
		//rx_valid_buf_size = 6;
	//}

    //below statement needs to be disabled for spi echo command test, but needs to be enabled for uploading data from spi driver to ttyspi driver.
	if ((spi_data->throttle == 0) 
	  && (rx_valid_buf_size != 0) 
	  && (spi_data->mdm_tty!=NULL) 
	  && (atomic_read(&spi_table[spi_data->index].in_use) != 0)
	  )
	{
#if defined(LGE_DUMP_SPI_BUFFER)
    dump_spi_buffer("SPI RX", &spi_data->mdm_rx_buffer[4], COL_SIZE);
#endif	
#ifdef SPI_BUFFER_DUMP_LOG	
		 if (spi_data->mdm_rx_buffer)
		 {
			 SPI_PRINTK("SPI RX  ----------------- \n");
			 dump_atcmd(spi_data->mdm_rx_buffer+IFX_SPI_HEADER_SIZE+2); 	 
		 }
		 else
		 {
			 SPI_PRINTK("SPI RX  Buffer  is NULL !!!!!");	 
		 }
#endif
	 /* spi에서 rx data가 존재하면 tty 로 전송	*/
#ifdef SPI_SPEED_MEASUREMENT
		uiRxlen[spi_data->mdm_tty->index] = rx_valid_buf_size+IFX_SPI_HEADER_SIZE;
#endif	

		if  (status == (-EIO))
		{
			if ((count_transfer_failed >= 20) && (ts_ldisc_close_is_called == 0))      
			{
				SPI_PRINTK("SPI flow control Reject ");
				//memset(spi_data->mdm_rx_buffer, 0, IFX_SPI_MAX_BUF_SIZE+IFX_SPI_HEADER_SIZE);	//rx buf init
				return;
			}
		}

		tty_insert_flip_string(spi_data->mdm_tty, spi_data->mdm_rx_buffer+IFX_SPI_HEADER_SIZE, rx_valid_buf_size);
		tty_flip_buffer_push(spi_data->mdm_tty);
	}
	else
	{
#ifdef SPI_BUFFER_DUMP_LOG  	
		//	handle RTS and CTS in SPI flow control Reject the packet as of now
	   if (spi_data->throttle == 0)
	   	SPI_PRINTK("throttle = NULL \n");
	   if (rx_valid_buf_size == 0)
		SPI_PRINTK("rx_valid_buf_size = NULL \n");
	   if (spi_data->mdm_tty ==NULL)
		SPI_PRINTK("mdm_tty = NULL \n");	   	
	   if (atomic_read(&spi_table[spi_data->index].in_use) == 0)
		SPI_PRINTK("spi_table[%d].in_use = NULL \n", spi_data->index);	   	
#endif
	}

#ifdef CONFIG_HAS_WAKELOCK
	if(spi_data->wake_lock_flag)
	{
		spi_data->wake_lock_flag = 0;
	}	
#endif
    SPI_DEBUG(" end\n");
}
#endif // TX_BUFFER_QUEUE


//#define SPI_TIMEDOUT_NSEC     20000000     //20ms
#define SPI_TIMEDOUT_SEC      2     //2s
extern void kernel_restart(char *cmd);

static unsigned int mdm_spi_sync_read_write(struct mdm_spi_data *spi_data, unsigned int len)
{
	bool spi_suspend_failed;
	int status;
  
	struct spi_message	m;
	struct spi_transfer	t = {
        .tx_buf		= spi_data->mdm_tx_buffer,
        .rx_buf		= spi_data->mdm_rx_buffer,
        .len			= len,
	};

	SPI_DEBUG("start");

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);

    //condition added for checking the 'in_use' variable in order to prevent spi transmitting after 'mdm_spi_close'
	if ((spi_data == NULL) 
	|| (spi_data->spi == NULL) 
	|| (atomic_read(&spi_table[spi_data->index].in_use)==0)) 
	{
		//WBT #196461
		//status = -ESHUTDOWN;
		SPI_PRINTK(" Cannot send after transport endpoint shutdown\n");
		return -ESHUTDOWN;
	}
	else 
	{
		//hrtimer_start(&(spi_data->timer), ktime_set(0, SPI_TIMEDOUT_NSEC), HRTIMER_MODE_REL);
		hrtimer_start(&(spi_data->timer), ktime_set(SPI_TIMEDOUT_SEC,0), HRTIMER_MODE_REL);

		//check if 'spi_sync' is running(when running, is_syncing is 1, when finished, is_syncing is 0)
		atomic_set(&spi_data->is_syncing, 1);

		status = spi_sync(spi_data->spi, &m);
		
		atomic_set(&spi_data->is_syncing, 0);

		do{
			//when the timer is currently active apply 1 ms of sleep
			if (hrtimer_try_to_cancel(&(spi_data->timer))==(-1)) 
			{
				msleep(1);
			}
			else 
			{
				SPI_DEBUG("hrtimer is inactive of sleep");			
				break;
			}
		}while(1);

	}

	mdm_spi_set_mrdy_signal(spi_data->spi->master->bus_num, 0);

	if (status == 0)  // success
	{
		status = m.status;
		if (status == 0)
			status = m.actual_length;

		//reset 'count_transfer_failed' to zero, if spi transter succeeds at least one out of five times
		count_transfer_failed = 0;
	}
	else   // fail
	{
		if(mdm_modem_communicating() == 0) 
		{
			SPI_PRINTK("Reset MDM6600 forcely : %d\n", count_transfer_failed);
#if defined(CONFIG_BSSQ_FOTA)
			mdm_reset();
#endif
			count_transfer_failed = 0;
			return status;
		}

		//increase 'count_transfer_failed', when spi transter fails
		count_transfer_failed++;
	}

	return status;
}

/*
 * Function is a Interrupt service routine, is called when MRDY signal goes HIGH. It set up srdy pin when ready and
 * reception if it is a Master initiated data transfer. For both the cases Master intiated/Slave intiated
 * transfer it starts data transfer.
 */
static irqreturn_t mdm_spi_handle_srdy_irq(int irq, void *handle)
{
    struct mdm_spi_data *spi_data = (struct mdm_spi_data *)handle;
    int value;

    SPI_DEBUG("");
	
    value = mdm_spi_get_srdy_signal(spi_data->index);

    if(value == 0)
    {
        SPI_DEBUG(" in the mrdy irq value(=%d) --> ignore \n", value);
        return IRQ_HANDLED;
    }

#ifdef TX_BUFFER_QUEUE
    if(atomic_read(&next_transfer_flag) == 0)
#else
    if(!spi_data->mdm_master_initiated_transfer)
#endif
    {
        //prevent spi transmission being registered as workqueue event
        if(atomic_read(&spi_table[spi_data->index].in_use) == 1) 
        {
            // when prevent MRDY until spi openning,,,to prevent interrupt during transmit			
            if(spi_data && spi_data->mdm_tty)	
            {
#ifdef CONFIG_HAS_WAKELOCK
                wake_lock_timeout(&spi_data->spi_wakelock, msecs_to_jiffies(3000)); //500ms
                SPI_DEBUG("spi_data->spi_wakelock is enable 50 ms"); 
#endif		
                queue_work(spi_data->mdm_wq, &spi_data->mdm_work);
            }
            else
            {
                SPI_PRINTK("Unexpected interrupt happen!");
            }	

            SPI_DEBUG(": queue_work executed!!!\n");
        }
    }

    return IRQ_HANDLED;
}

#ifdef TX_BUFFER_QUEUE
int get_tx_pending_data(struct mdm_spi_data *spi_data, int *anymore)
{
    int cumu_data_size = 0;
    int data_size = 0;
    u8 *data_ptr;
    int tx_data_found = 0;
    int remain_count =0;
    int queue_count = 0;
    int initial_queue_count = 0;
    int cumu_queue_count = 0;

    struct spi_data_send_struct *prev_ptr;
    struct spi_data_send_struct **ppMS;
    struct spi_data_send_struct *curr_ptr=spi_data_send_pending;

//    SPI_PRINTK("get_tx_pending_data()++ spi_data_send_pending 0x%x\n", (uint)curr_ptr);

    spin_lock_irqsave(&spi_nodes_lock, spi_lock_flag);

    if(spi_data_send_pending == NULL)
    {
        spin_unlock_irqrestore(&spi_nodes_lock, spi_lock_flag);
        return 0;
    }

    initial_queue_count = spi_data_send_pending->count;

    do{
        prev_ptr = NULL;

		//fixed the queue boundary check routine for when ONLY ONE queue remains
        for(ppMS = &spi_data_send_pending, queue_count = 0;
            (((*ppMS)->next !=NULL) && (queue_count<=spi_data_send_pending->count));
            ppMS = &((*ppMS)->next), queue_count++);

        if(*ppMS == NULL)
        {
            SPI_PRINTK("Never Hit this case\n");
            return 0;
        }
        else if(((*ppMS)->next != NULL))
        {
            SPI_PRINTK("Never Hit this case *ppMS->next 0x%x\n", (uint)(*ppMS)->next);
            return 0;
        }
        else
        {
            // Good Queue
        }

        if(((*ppMS)->size+cumu_data_size) <= IFX_SPI_MAX_BUF_SIZE )
        {
            memcpy(&(spi_data->mdm_tx_buffer[IFX_SPI_HEADER_SIZE+cumu_data_size]),(*ppMS)->data,(*ppMS)->size);

            if((*ppMS)->bkp_data!=NULL)
            {
                data_ptr = (*ppMS)->bkp_data;
                data_size = (*ppMS)->bkp_size;
                (*ppMS)->bkp_data = NULL;
                (*ppMS)->bkp_size = 0;
            }
            else 
            {
                data_ptr = (*ppMS)->data;
                data_size = (*ppMS)->size;
            }

            cumu_data_size = cumu_data_size + (*ppMS)->size;

            (*ppMS)->size = 0;
            (*ppMS)->data = NULL;

            prev_ptr = (*ppMS)->prev;
            (*ppMS)->next = NULL;
            (*ppMS)->prev = NULL;

            spi_data_send_pending->count--;
            tx_data_found = 1;
            kfree(data_ptr);
            kfree((*ppMS));

            if(prev_ptr == NULL)
            {
                spi_data_send_pending = NULL;
                break;
            }
            else
            {
                prev_ptr->next = NULL;
            }
        }
        else
        {
            remain_count = ((*ppMS)->size+cumu_data_size)-IFX_SPI_MAX_BUF_SIZE;
            memcpy(&(spi_data->mdm_tx_buffer[IFX_SPI_HEADER_SIZE+cumu_data_size]),(*ppMS)->data, ((*ppMS)->size-remain_count));
            cumu_data_size += ((*ppMS)->size-remain_count);

            (*ppMS)->bkp_data = (*ppMS)->data;
            (*ppMS)->bkp_size = (*ppMS)->size;
            (*ppMS)->data = (*ppMS)->data+((*ppMS)->size-remain_count);
            (*ppMS)->size = remain_count;

            *anymore = 1;
            break;
        }

        if(prev_ptr == NULL)
        {
            SPI_PRINTK("Never Hit this case prev_ptr=0x%x\n", (uint)prev_ptr);
            break;
        }
        cumu_queue_count++;
    }
    while((cumu_data_size <= IFX_SPI_MAX_BUF_SIZE) && (cumu_queue_count<initial_queue_count));

    curr_ptr=spi_data_send_pending;
    spin_unlock_irqrestore(&spi_nodes_lock, spi_lock_flag);

//    SPI_PRINTK("get_tx_pending_data()-- curr_ptr=0x%x cumu_data_size=%d anymore=%d\n", (uint)curr_ptr, cumu_data_size, *anymore);

    return cumu_data_size;
}
#endif


#ifdef TX_BUFFER_QUEUE
static void
mdm_spi_handle_work(struct work_struct *work)
{
    struct mdm_spi_data *spi_data;
    int tx_pending = 0;
    int tx_anymore = 0;

    int pm_off_count;
    //unsigned int tx_starttime;
    //unsigned int tx_endtime;

//    SPI_PRINTK("%s()++ %d\n", __func__, atomic_read(&next_transfer_flag));
    atomic_set(&next_transfer_flag, 0);
    do{
        spi_data = container_of(work, struct mdm_spi_data, mdm_work);

        if(spi_data==NULL) {
            SPI_PRINTK( "error spi_data(0x%x) is NULL \n", (int)spi_data);
            break;
        }

        if (atomic_read(&spi_table[spi_data->index].in_use) == 0) {
            SPI_PRINTK( "open ttyspi first(#%d)\n", spi_data->index);
            return;
        }

        //replace to the header cleaning.
        //memset(spi_data->mdm_tx_buffer, 0, IFX_SPI_FRAME_SIZE);
        //memset(spi_data->mdm_rx_buffer, 0, IFX_SPI_FRAME_SIZE);

        tx_anymore =0;
        tx_pending = get_tx_pending_data(spi_data, &tx_anymore);

        pm_off_count = 0;

        //need to wait transferring tx/rx data because ap is in a suspended state
        if(1 == spi_data->is_suspended){
            pm_off_count = 1;
            SPI_PRINTK("mdm_spi_handle_work INFO is_suspended is (0x%x)\n", (int)spi_data->is_suspended);

            //wait for ap to return to resume state with a worst case scenario of 5sec
            do{
                mdelay(5);
                pm_off_count++;
            }while((1 == spi_data->is_suspended) && (pm_off_count<(5*200)));

            SPI_PRINTK("mdm_spi_handle_work INFO EXIT is_suspend = 0x%x pm_off_count=%d\n", (int)spi_data->is_suspended, pm_off_count);
            if(1 == spi_data->is_suspended) {
               // To Do how to handle the PM OFF state during 1sec
               SPI_PRINTK("mdm_spi_handle_work error is_suspended is (0x%x)\n", (int)spi_data->is_suspended);
            }
        }

#if 0 // Sometimes, the AP response time takes a long time, so a AT command pending happens. --> allow the null traffic because the CP seem to be ready to send.
        if((tx_pending == 0)
          && (atomic_read(&next_transfer_flag) == 0)
#ifdef SPI_STATISTICS_CHECK
          && (dummy_data_flag == 0)
#endif
          && (spi_data->mdm_receiver_buf_size == 0)
          && (mdm_spi_get_mrdy_signal(spi_data->index) == 0)
          && (pm_off_count == 0) ) {
            TTYSPI_DEBUG_PRINT("NULL Transfer will be triggered...(%d %d) %d %d %d %d %d\n",
            tx_count[0], rx_count[0], tx_pending, atomic_read(&next_transfer_flag), spi_data->mdm_receiver_buf_size,
            mdm_spi_get_mrdy_signal(spi_data->index), pm_off_count);
            break; // just return;
        }
#endif

        spi_data->mdm_receiver_buf_size = 0;  // next rx frame size

        //0. Frame Setup
        mdm_spi_clear_header_info((unsigned int *)spi_data->mdm_tx_buffer);

        if(tx_anymore) mdm_spi_set_header_info(spi_data->mdm_tx_buffer, tx_pending, IFX_SPI_MAX_BUF_SIZE);
        else mdm_spi_set_header_info(spi_data->mdm_tx_buffer, tx_pending, 0);

        if(tx_pending !=0) mdm_spi_set_tx_frame_count((unsigned int *)spi_data->mdm_tx_buffer, spi_data->index);

        mdm_spi_clear_header_info((unsigned int *)spi_data->mdm_rx_buffer);

        // 1. SPI Transmit
        //tx_starttime= NvOsGetTimeMS();
        mdm_spi_send_and_receive_data(spi_data, tx_pending);
        //tx_endtime = NvOsGetTimeMS();

#if 0
        if((tx_endtime - tx_starttime) > 1500) {
            printk(KERN_ERR "\n SPI TX Error : tx_endtime = %d, tx_starttime=%d (diff=%d) \n", tx_endtime, tx_starttime, (tx_endtime - tx_starttime));
            //if(timeout_count++>5) print_spi_log(3);
            timeout_count++;
        }
        else {
            timeout_count = 0;
        }
#endif

        if(tx_anymore || spi_data->mdm_receiver_buf_size) {
            SPI_DEBUG("tx_anymore: %d, receiver_buf_size:  %d \n", tx_anymore, spi_data->mdm_receiver_buf_size);
            atomic_set(&next_transfer_flag, 1);
        }
        else {
            atomic_set(&next_transfer_flag, 0);
        }

        if(spi_data->is_waiting == true)
        {
            complete(&spi_data->mdm_read_write_completion);
            spi_data->is_waiting = false;
        }

    }
    while(atomic_read(&next_transfer_flag) != 0);

//    SPI_PRINTK("%s()-- %d\n", __func__, atomic_read(&next_transfer_flag));
    atomic_set(&next_transfer_flag, 0);
#ifdef SPI_STATISTICS_CHECK
    dummy_data_flag = 0;
#endif
}

#else //TX_BUFFER_QUEUE

static void mdm_spi_handle_work(struct work_struct *work)
{
	struct mdm_spi_data *spi_data = container_of(work, struct mdm_spi_data, mdm_work);
	bool spi_suspended;	

#ifdef SPI_SPEED_MEASUREMENT
	int id = 0;
	unsigned long diff;
#endif

	SPI_DEBUG(" start\n");
//	SPI_DEBUG(" bus_num=%d\n", bus_num);

	spi_suspended = spi_tegra_is_suspended(spi_data->spi);

	if (spi_suspended) 
	{
		SPI_PRINTK("spi_tegra_slave is not resume!, spi_suspended=%d\n", spi_suspended);
		complete(&spi_data->mdm_read_write_completion);    //do not exit in kthread_stop when ril recovery			
		return;
	}

	if (atomic_read(&spi_table[spi_data->index].in_use) == 0) 
	{
		SPI_PRINTK("spi_data->index:%d\n", spi_data->index);
		complete(&spi_data->mdm_read_write_completion);    //do not exit in kthread_stop when ril recovery
		SPI_PRINTK("mdm_read_write_completion !!! \n");
		return;
	}

	/* read condition */
	if (!spi_data->mdm_master_initiated_transfer)
	{
#ifdef SPI_SPEED_MEASUREMENT
		do_gettimeofday(&ulStart[id]);
#endif	
		SPI_DEBUG("[spi_mdm6600 #%d]mdm_spi_handle_work came from mdm6600\n", spi_data->index);
		mdm_spi_setup_transmission(spi_data);
		//mdm_spi_set_mrdy_signal(bus_num, 1);
		mdm_spi_send_and_receive_data(spi_data);

#ifdef SPI_SPEED_MEASUREMENT
		do_gettimeofday(&ulEnd[id]);
		diff = (ulEnd[id].tv_sec - ulStart[id].tv_sec) * 1000 * 1000 ;
		diff = (diff + (ulEnd[id].tv_usec - ulStart[id].tv_usec));
		ulRxThroughtput[id] = ((uiRxlen[id]*8*1000)/diff);
		SPI_PRINTK("[SPI %d] : RX time = %09d usec; %04d bytes; %06lu Kbps", 
					id, diff, IFX_SPI_MAX_BUF_SIZE+IFX_SPI_HEADER_SIZE, 
					((IFX_SPI_MAX_BUF_SIZE+IFX_SPI_HEADER_SIZE)*8*1000)/diff);
#endif		
		/* Once data transmission is completed, the MRDY signal is lowered */
		if((spi_data->mdm_sender_buf_size == 0)  && (spi_data->mdm_receiver_buf_size == 0)) 
		{
			//mdm_spi_set_mrdy_signal(bus_num, 0);

		    //Intialize frame sizes
		    mdm_spi_buffer_initialization(spi_data);				
		}

		/* We are processing the slave initiated transfer in the mean time Mux has requested master initiated data transfer */
		/* Once Slave initiated transfer is complete then start Master initiated transfer */
		if(spi_data->mdm_master_initiated_transfer == 1)
		{
		/* It is a condition where Slave has initiated data transfer and both SRDY and MRDY are high and at the end of data transfer
	 	* MUX has some data to transfer. MUX initiates Master initiated transfer rising MRDY high, which will not be detected at Slave-MODEM.
	 	* So it was required to rise MRDY high again */
            //mdm_spi_set_mrdy_signal(bus_num, 1);
		}
	}
	/* write condition */
	else
	{
		SPI_DEBUG("[spi_mdm6600 #%d]mdm_spi_handle_work came from t20\n", spi_data->index);
		mdm_spi_setup_transmission(spi_data);
		mdm_spi_send_and_receive_data(spi_data);
		/* Once data transmission is completed, the MRDY signal is lowered */
		if(spi_data->mdm_sender_buf_size == 0)
		{
			if(spi_data->mdm_receiver_buf_size == 0)
			{
				//mdm_spi_set_mrdy_signal(bus_num, 0);

			    //Intialize frame sizes				
			    mdm_spi_buffer_initialization(spi_data);
			}
			spi_data->mdm_master_initiated_transfer = 0;

            //  fix RIL holding problem - in case of  spi_data->mdm_sender_buf_size > 0, must send complete also.
			//complete(&spi_data->mdm_read_write_completion);
		}
		//spi_data->mdm_master_initiated_transfer = 0;
		complete(&spi_data->mdm_read_write_completion);
        // fix RIL holding problem - in case of  spi_data->mdm_sender_buf_size > 0, must send complete also.
	}

#ifdef SPI_TX_RX_THROUGHTPUT
	if(uiTxlen[spi_data->mdm_tty->index] || uiRxlen[spi_data->mdm_tty->index]) 
	{
             //ulEnd = getuSecTime() - ulStart;
             do_gettimeofday(&ulEnd[spi_data->mdm_tty->index]);
             
             uidiff[spi_data->mdm_tty->index] = (ulEnd[spi_data->mdm_tty->index].tv_sec - ulStart[spi_data->mdm_tty->index].tv_sec) * 1000 * 1000 ;
             uidiff[spi_data->mdm_tty->index] = uidiff[spi_data->mdm_tty->index] + (ulEnd[spi_data->mdm_tty->index].tv_usec - ulStart[spi_data->mdm_tty->index].tv_usec);
             ulTxThroughtput[spi_data->mdm_tty->index] = ((uiTxlen[spi_data->mdm_tty->index]*8*1000)/uidiff[spi_data->mdm_tty->index]);
             ulRxThroughtput[spi_data->mdm_tty->index] = ((uiRxlen[spi_data->mdm_tty->index]*8*1000)/uidiff[spi_data->mdm_tty->index]);

	     SPI_PRINTK("[SPI %d] time  = %d us, Tx(%dbytes) = %luKbps, Rx(%dbytes) = %luKbps, Max(%dbytes) = %luKbps\n", 
		 	spi_data->mdm_tty->index, uidiff[spi_data->mdm_tty->index], 
			uiTxlen[spi_data->mdm_tty->index], ulTxThroughtput[spi_data->mdm_tty->index], 
			uiRxlen[spi_data->mdm_tty->index], ulRxThroughtput[spi_data->mdm_tty->index],
			IFX_SPI_MAX_BUF_SIZE+IFX_SPI_HEADER_SIZE, 
			((IFX_SPI_MAX_BUF_SIZE+IFX_SPI_HEADER_SIZE)*8*1000)/uidiff[spi_data->mdm_tty->index]);
	
            uiTxlen[spi_data->mdm_tty->index] = uiRxlen[spi_data->mdm_tty->index] = 0;
            fWrite[spi_data->mdm_tty->index] = 0;
	}
#endif

}
#endif //TX_BUFFER_QUEUE


/* ################################################################################################################ */


/* ################################################################################################################ */

/* Initialization Functions */

/*
 * Initialization function which allocates and set different parameters for TTY SPI driver. Registers the tty driver
 * with TTY core and register SPI driver with the Kernel. It validates the GPIO pins for MRDY and then request an IRQ
 * on SRDY GPIO pin for SRDY signal going HIGH. In case of failure of SPI driver register cases it unregister tty driver
 * from tty core.
 */
static int __init mdm_spi_init(void)
{
	int status = 0;
	SPI_DEBUG("start\n");
	memset(spi_table, 0x0, sizeof(struct allocation_table)*4);

	/* Allocate and Register a TTY device */
	mdm_spi_tty_driver = alloc_tty_driver(IFX_N_SPI_MINORS);
	if (!mdm_spi_tty_driver){
		SPI_PRINTK("Fail to allocate TTY Driver\n");
		return -ENOMEM;
	}

	/* initialize the tty driver */
	mdm_spi_tty_driver->owner = THIS_MODULE;
	mdm_spi_tty_driver->driver_name = "tty_mdm6600";
	mdm_spi_tty_driver->name = "ttyspi";
	mdm_spi_tty_driver->major = IFX_SPI_MAJOR;
	mdm_spi_tty_driver->minor_start = 0;
	mdm_spi_tty_driver->type = TTY_DRIVER_TYPE_SERIAL;
	mdm_spi_tty_driver->subtype = SERIAL_TYPE_NORMAL;
	mdm_spi_tty_driver->flags = TTY_DRIVER_REAL_RAW;
	mdm_spi_tty_driver->init_termios = tty_std_termios;
	mdm_spi_tty_driver->init_termios.c_cflag = B921600 | CS8 | CREAD | HUPCL | CLOCAL;

	tty_set_operations(mdm_spi_tty_driver, &mdm_spi_ops);

	status = tty_register_driver(mdm_spi_tty_driver);
	if (status)
	{
		SPI_PRINTK("Failed to register mdm_spi_tty_driver");
		put_tty_driver(mdm_spi_tty_driver);
		return status;
	}
	else 
	{		
		SPI_DEBUG("register mdm_spi_tty_driver\n");
	}

	/* Register SPI Driver */
	status = spi_register_driver(&mdm_spi_driver);
	if (status < 0)
	{
		SPI_PRINTK("Failed to register SPI device");
		tty_unregister_driver(mdm_spi_tty_driver);
		put_tty_driver(mdm_spi_tty_driver);
		return status;
	}
	else 
	{
		SPI_DEBUG("register mdm_spi_driver\n");
	}

#ifdef TX_BUFFER_QUEUE
    if(queue_first_time == 1){
        spin_lock_init(&spi_nodes_lock);
        spi_data_send_pending = NULL;
        queue_first_time = 0;
    }
#endif

	SPI_DEBUG(" end\n");
	return status;
}

module_init(mdm_spi_init);


/*
 * Exit function to unregister SPI driver and tty SPI driver
 */
static void __exit mdm_spi_exit(void)
{
    SPI_DEBUG("\n");
    spi_unregister_driver(&mdm_spi_driver);
    tty_unregister_driver(mdm_spi_tty_driver);
    put_tty_driver(mdm_spi_tty_driver);
}

module_exit(mdm_spi_exit);

/* End of Initialization Functions */

/* ################################################################################################################ */

MODULE_DESCRIPTION("MDM6600 SPI framing layer for dual spi");
MODULE_LICENSE("GPL");
