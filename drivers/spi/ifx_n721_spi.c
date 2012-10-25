/*
 * ifx_n721_spi.c -- Serial peheripheral interface framing layer for IFX modem.
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
#include <linux/smp_lock.h>
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

//20110607 ws.yang@lge.com add to ifx modem [S]
#include <linux/spi-tegra.h>

#include <mach/gpio-names.h>
#include "ifx_n721_spi.h"
#include <linux/mutex.h>

/* define for spi1 port gpio information */
static int IFX_SRDY;
static int IFX_MRDY;
//20110607 ws.yang@lge.com add to ifx modem [E]

#ifdef CONFIG_PM
//Hold wake-lock for cp interrupt
#define WAKE_LOCK_RESUME
#ifdef WAKE_LOCK_RESUME
#include <linux/wakelock.h>
#endif

#include <mach/iomap.h>
#include <linux/io.h>

#define PMC_WAKE_STATUS 0x14
#define WAKEUP_IFX_SRDY_MASK    (1 << 0)     // Wake Event 0 - IFX_SRDY

static void __iomem *pmc_base = IO_ADDRESS(TEGRA_PMC_BASE);

static bool bSuspend = 0;
#endif

/* srdy interrupt is gradually inputed when spi tegra is suspended */
//#define INTERRUPT_DELAY		msecs_to_jiffies(5000)

//Change delay time for transcation : 1000us -> 400us
#include <linux/delay.h>
#define MRDY_DELAY_TIME 100 // to throughput ..  400   //1000

#ifdef IFX_SPI_TEGRA_TRANSFER_DURATION	
#include <linux/time.h>
#endif

#ifdef IFX_SPI_SPEED_MEASUREMENT
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

/* Structure used to store private data */
struct ifx_spi_data {
	dev_t			devt;
	spinlock_t		spi_lock;
	struct spi_device	*spi;
	struct list_head	device_entry;
        struct completion       ifx_read_write_completion;
        struct tty_struct       *ifx_tty;

	/* buffer is NULL unless this device is open (users > 0) */
	struct mutex		buf_lock;
	unsigned		users;
        unsigned int		throttle;
        struct work_struct      ifx_work;
        struct workqueue_struct *ifx_wq;

        unsigned int		ifx_master_initiated_transfer;
        unsigned int		ifx_spi_count;
        unsigned int		ifx_sender_buf_size;
        unsigned int		ifx_receiver_buf_size;
        unsigned int		ifx_current_frame_size;
        unsigned int		ifx_valid_frame_size;
        unsigned int		ifx_ret_count;
        const unsigned char 	*ifx_spi_buf;
        unsigned char		*ifx_tx_buffer;
        unsigned char           *ifx_rx_buffer;

#ifdef WAKE_LOCK_RESUME
	struct wake_lock wake_lock;
	unsigned int	wake_lock_flag;
#endif		

	int is_suspended;

	//int ifx_spi_lock;
		
};

union ifx_spi_frame_header{
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
	}ifx_spi_header;
	unsigned char framesbytes[IFX_SPI_HEADER_SIZE];
};

struct ifx_spi_data	*ifx_gspi_data;
struct tty_driver 	*ifx_spi_tty_driver;

int spi_err_flag=0;

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

#ifdef IFX_SPI_DUMP_LOG
void ifx_dump_atcmd(char *data) 
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
//		IFX_SPI_PRINTK("long packet length %d\n", uih_len);
	} 
	else 
	{
		uih_len = short_pkt->h.length.len;
		uih_data_start = short_pkt->data;
//		IFX_SPI_PRINTK("long packet length %d\n", uih_len);
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
	IFX_SPI_PRINTK("\n");
}
#endif

#ifdef LGE_DUMP_SPI_BUFFER
#define COL_SIZE 20
static void ifx_dump_spi_buffer(const unsigned char *txt, const unsigned char *buf, int count)
{
    char dump_buf_str[COL_SIZE+1];

    if (buf != NULL) 
    {
        int j = 0;
        char *cur_str = dump_buf_str;
        unsigned char ch;
        while((j < COL_SIZE) && (j  < COL_SIZE/* count */))
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


/* ################################################################################################################ */
/* Global Declarations */
unsigned long		minors[IFX_N_SPI_MINORS / BITS_PER_LONG];



/* Function Declarations */
static void ifx_spi_set_header_info(unsigned char *header_buffer, unsigned int curr_buf_size, unsigned int next_buf_size);
static int ifx_spi_get_header_info(unsigned char *rx_buffer, unsigned int *valid_buf_size);
static void ifx_spi_set_mrdy_signal(int value);
static void ifx_spi_setup_transmission(struct ifx_spi_data *spi_data);
static void ifx_spi_send_and_receive_data(struct ifx_spi_data *spi_data);
static int ifx_spi_get_next_frame_size(int count);
static int ifx_spi_allocate_frame_memory(struct ifx_spi_data *spi_data, unsigned int memory_size);
static void ifx_spi_free_frame_memory(struct ifx_spi_data *spi_data);
static void ifx_spi_buffer_initialization(struct ifx_spi_data *spi_data);
static unsigned int ifx_spi_sync_read_write(struct ifx_spi_data *spi_data, unsigned int len);
static irqreturn_t ifx_spi_handle_srdy_irq(int irq, void *handle);
static void ifx_spi_handle_work(struct work_struct *work);


/* ################################################################################################################ */

static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);

/* ################################################################################################################ */

/* IFX SPI Operations */

#define MAX_TRANSFER_FAILED 5
int count_transfer_failed = 0;

//check communication with modem
int ifx_modem_communicating(void)
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
static int ifx_spi_open(struct tty_struct *tty, struct file *filp)
{
	int status = 0;
	struct ifx_spi_data *spi_data;

	IFX_SPI_DEBUG("[id: %d]", tty->index);
	
	if(ifx_gspi_data)
	{
		spi_data = ifx_gspi_data;
		spi_data->ifx_tty = tty;
		tty->driver_data = spi_data;
		ifx_spi_buffer_initialization(spi_data);
		spi_data->throttle = 0;
		IFX_SPI_PRINTK(" success!!");
	}
	else
	{
		tty->driver_data = NULL;	
		IFX_SPI_PRINTK("failed!!");
		status = -ENODEV;		
	}

	return status;
}

/*
 * Function closes a opened a tty device when called from user space
 */
static void ifx_spi_close(struct tty_struct *tty, struct file *filp)
{  
	//struct ifx_spi_data *spi_data = (struct ifx_spi_data *)tty->driver_data;
	//spi_data->ifx_tty = NULL;
	//tty->driver_data = NULL;
}

/*
 * Function is called from user space to send data to MODEM, it setups the transmission, enable MRDY signal and
 * waits for SRDY signal HIGH from MDOEM. Then starts transmission and reception of data to and from MODEM.
 * Once data read from MODEM is transferred to TTY core flip buffers, then "ifx_read_write_completion" is set
 * and this function returns number of bytes sent to MODEM
 */

static int ifx_spi_write(struct tty_struct *tty, const unsigned char *buf, int count)
{	
	struct ifx_spi_data *spi_data = (struct ifx_spi_data *)tty->driver_data;

	if(spi_data==NULL)
	{
		IFX_SPI_PRINTK("failed : spi_data is null");
		return 0;
	}

	// spi suspend check and spi_tegra suspend check 
	if(spi_data->is_suspended ||spi_tegra_is_suspend(spi_data->spi))
	{
		IFX_SPI_PRINTK("SPI suspend : %d, SPI Tegra suspend : %d",spi_data->is_suspended,spi_tegra_is_suspend(spi_data->spi));
		return -1;
	}



	spi_data->ifx_ret_count = 0;
	spi_data->ifx_tty = tty;
	spi_data->ifx_tty->low_latency = 1;

#ifdef IFX_SPI_SPEED_MEASUREMENT
	int id = tty->index; // spi no.
	unsigned long diff;
	fWrite[id] = 1;
	uiTxlen[id] = count + IFX_SPI_HEADER_SIZE;
	//ulStart = getuSecTime();
	do_gettimeofday(&ulStart[id]); //RTC(Real Time Clock)의 현재 실행시간
#endif

	if( !buf )
	{
		IFX_SPI_PRINTK("\t Buffer NULL");
		return spi_data->ifx_ret_count;
	}
	
	if(!count)
	{
		IFX_SPI_PRINTK("\t Count is ZERO");
		return spi_data->ifx_ret_count;
	}
	
	IFX_SPI_DEBUG("**** \n");

#ifdef LGE_DUMP_SPI_BUFFER
	dump_spi_buffer("ifx_spi_write()", buf, count); 
#endif

	spi_data->ifx_master_initiated_transfer  = 1;
	spi_data->ifx_spi_buf  = buf;
	spi_data->ifx_spi_count  = count;	
	ifx_spi_set_mrdy_signal(1);
	wait_for_completion_timeout(&spi_data->ifx_read_write_completion,  2*HZ);  //3ms -> 500ms


	//To check the spi retry count when data transmit..
	if(spi_data->ifx_ret_count == 0) 
	{	
		int pin_val;	
		pin_val = gpio_get_value(IFX_SRDY);
		ifx_spi_set_mrdy_signal(0); 
		IFX_SPI_PRINTK("spi tx timeout!! SRDY : %d , buf : 0x%x, count : %d",pin_val,buf,count);
		IFX_SPI_PRINTK("ifx_master_initiated_transfer : %d, is_suspended : %d, tegra_suspend : %d",
		spi_data->ifx_master_initiated_transfer,spi_data->is_suspended,spi_tegra_is_suspend(spi_data->spi));

#ifdef LGE_DUMP_SPI_BUFFER		
		ifx_dump_spi_buffer("ifx_spi_write -- fail()", buf, count);		
#endif

		init_completion(&spi_data->ifx_read_write_completion);

		return -2; /* To Check error */

	}

#ifdef IFX_SPI_SPEED_MEASUREMENT
	//ulEnd = getuSecTime() - ulStart;
	do_gettimeofday(&ulEnd[id]);
	diff = (ulEnd[id].tv_sec - ulStart[id].tv_sec) * 1000 * 1000; //sec
	diff = (diff + (ulEnd[id].tv_usec - ulStart[id].tv_usec));
	ulTxThroughtput[id] = ((uiTxlen[id]*8*1000)/diff);
	IFX_SPI_PRINTK("					[SPI %d] : TX time = %09d usec; %04d bytes; %06lu Kbps", 
				id, diff, IFX_SPI_MAX_BUF_SIZE+IFX_SPI_HEADER_SIZE, 
				((IFX_SPI_MAX_BUF_SIZE+IFX_SPI_HEADER_SIZE)*8*1000)/diff);
	uiTxlen[id];
	fWrite[id] = 0;
#endif
	
	init_completion(&spi_data->ifx_read_write_completion);

	return spi_data->ifx_ret_count; /* Number of bytes sent to the device */
}

/* This function should return number of free bytes left in the write buffer in this case always return 2048 */

static int ifx_spi_write_room(struct tty_struct *tty)
{	
	return IFX_SPI_MAX_BUF_SIZE;
}


/* ################################################################################################################ */
/* These two functions are to be used in future to implement flow control (RTS & CTS)*/
/*static void 
ifx_spi_throttle(struct tty_struct *tty)
{
	unsigned int flags;
	struct ifx_spi_data *spi_data = (struct ifx_spi_data *)tty->driver_data;
	spi_data->ifx_tty = tty;
	spin_lock_irqsave(&spi_data->spi_lock, flags);
	spi_data->throttle = 1;
	spin_unlock_irqrestore(&spi_data->spi_lock, flags);
}

static void 
ifx_spi_unthrottle(struct tty_struct *tty)
{
	unsigned int flags;
	struct ifx_spi_data *spi_data = (struct ifx_spi_data *)tty->driver_data;
	spi_data->ifx_tty = tty;
	spin_lock_irqsave(&spi_data->spi_lock, flags);
	spi_data->throttle = 0;
	if( ifx_rx_buffer != NULL ){
	     tty_insert_flip_string(spi_data->ifx_tty, ifx_rx_buffer, valid_buffer_count);
	}
	spin_unlock_irqrestore(&spi_data->spi_lock, flags);  
}*/
/* ################################################################################################################ */

/* End of IFX SPI Operations */

/* ################################################################################################################ */

/* TTY - SPI driver Operations */

static int ifx_spi_probe(struct spi_device *spi)
{
     int status;
     struct ifx_spi_data *spi_data;
     

    spi_data = (struct ifx_spi_data*)kmalloc(sizeof(struct ifx_spi_data), GFP_KERNEL);
    if (!spi_data)
    {    
	IFX_SPI_PRINTK("Failed to allocate memory for spi_data");    
	return -ENOMEM;
    }

     IFX_SPI_PRINTK("start !!");

    //initialize all member variables of ifx_spi_data
    memset((void*)spi_data, 0, sizeof(struct ifx_spi_data));

	spi_data->ifx_tty = NULL;
#ifdef WAKE_LOCK_RESUME
	spi_data->wake_lock_flag = 0;
#endif

    status = ifx_spi_allocate_frame_memory(spi_data, (IFX_SPI_MAX_BUF_SIZE + IFX_SPI_HEADER_SIZE));
    if(status != 0)
    {
	IFX_SPI_PRINTK("Failed to allocate memory for buffers");
 	
	return -ENOMEM;
    }

    dev_set_drvdata(&spi->dev,spi_data);
    spin_lock_init(&spi_data->spi_lock);
    INIT_WORK(&spi_data->ifx_work,ifx_spi_handle_work);   
	
    spi_data->ifx_wq = create_singlethread_workqueue("ifxn721");
    if(!spi_data->ifx_wq)
    {
	IFX_SPI_PRINTK("Failed to setup workqueue - ifx_wq");          
    }
	
	init_completion(&spi_data->ifx_read_write_completion);
	
      //config srdy,mrdy gpio value according to hw revision
       IFX_MRDY = TEGRA_GPIO_PO0;  //ULPI_DATA7
       IFX_SRDY =  TEGRA_GPIO_PO5;  //ULPI_DATA4

        /* Configure SPI */
	spi_data->spi = spi;

	spi->mode = SPI_MODE_1 ;
	spi->bits_per_word = 8; //32 is mdm only
	spi->chip_select = 0 ;
	spi->max_speed_hz = 24000000; //48000000; //to 24Mhz	

	status = spi_setup(spi);
        if(status < 0)
	{
		IFX_SPI_PRINTK("Failed to setup SPI \n");
        }             

#ifdef WAKE_LOCK_RESUME
	wake_lock_init(&spi_data->wake_lock, WAKE_LOCK_SUSPEND, "mspi_wake");
#endif

	gpio_request(IFX_MRDY, "ifx_mrdy");	
 	tegra_gpio_enable(IFX_MRDY);
		
	gpio_request(IFX_SRDY, "ifx_srdy");	
 	tegra_gpio_enable(IFX_SRDY);	
	
	gpio_direction_input(IFX_SRDY);

/* changed to level trigger */

	status = request_irq(gpio_to_irq(IFX_SRDY), 
					    ifx_spi_handle_srdy_irq,
					    IRQF_TRIGGER_RISING, 
					    spi->dev.driver->name, 
					    spi_data);

	if (status != 0)
	{
		IFX_SPI_PRINTK("Failed to request IRQ for SRDY");
		IFX_SPI_PRINTK("IFX SPI Probe Failed");
		ifx_spi_free_frame_memory(spi_data);
		if(spi_data)
		{
			kfree(spi_data);
		}          
	}
 	else
	{
		ifx_gspi_data = spi_data;
	}

	enable_irq_wake(gpio_to_irq(IFX_SRDY)); //wake irq...	
	
	IFX_SPI_PRINTK(" end !!!  ");
	return status;
}

static int ifx_spi_remove(struct spi_device *spi)
{	
	struct ifx_spi_data *spi_data;
	spi_data = spi_get_drvdata(spi);
	spin_lock_irq(&spi_data->spi_lock);
	spi_data->spi = NULL;
	spi_set_drvdata(spi, NULL);
	spin_unlock_irq(&spi_data->spi_lock);

	ifx_spi_free_frame_memory(spi_data);

        if(spi_data)
	{
		kfree(spi_data);
        }          
        return 0;
}


#ifdef CONFIG_PM
static int ifx_spi_suspend(struct spi_device *spi, pm_message_t mesg)
{
    struct ifx_spi_data *spi_data;
    unsigned long flags;	
    unsigned long status;

    IFX_SPI_PRINTK("is start");

     spi_data = spi_get_drvdata(spi);

    //prevent duplicated access to 'spi_data->is_suspended'
    spin_lock_irqsave(&spi_data->spi_lock, flags);
    spi_data->is_suspended = 1;
    spin_unlock_irqrestore(&spi_data->spi_lock, flags);

    IFX_SPI_PRINTK("(is_suspend=%d)", spi_data->is_suspended);

     if(bSuspend)
     {
     	IFX_SPI_PRINTK("bSuspend = %d ", bSuspend);
     	return 0;
     }
     
     bSuspend = 1;

   // address를 주면 값을 read
    status = readl(pmc_base + PMC_WAKE_STATUS);

    // Clear power key wakeup pad bit.
    if (status & WAKEUP_IFX_SRDY_MASK)
    {
        IFX_SPI_PRINTK(" wakeup pad : 0x%lx", status);
        //주소와 값을 주면 주소에 값을 write		
        writel(WAKEUP_IFX_SRDY_MASK, pmc_base + PMC_WAKE_STATUS);
    }

    IFX_SPI_PRINTK("end");

    return 0;
}

static int ifx_spi_resume(struct spi_device *spi)
{
    struct ifx_spi_data *spi_data;
    unsigned long flags;
    unsigned long status;

    IFX_SPI_PRINTK("is start");

    spi_data = spi_get_drvdata(spi);
    
    IFX_SPI_PRINTK(" (is_suspend=%d)", spi_data->is_suspended);

     if(!bSuspend)
     {
     	IFX_SPI_PRINTK("bSuspend = %d ", bSuspend);
     	return 0;
     }
     
     bSuspend = 0;

    status = readl(pmc_base + PMC_WAKE_STATUS);

    if (status & WAKEUP_IFX_SRDY_MASK) 
    {
        IFX_SPI_PRINTK("wakeup pad : 0x%lx", status);    
#ifdef WAKE_LOCK_RESUME
//	IFX_SPI_PRINTK(" wake lock : 0x%lx", &ifx_gspi_data->wake_lock);
	if(&ifx_gspi_data->wake_lock)
	{
		wake_lock_timeout(&ifx_gspi_data->wake_lock, msecs_to_jiffies(500));	// for power consumption
	}

	 ifx_gspi_data->wake_lock_flag = 1;
#endif	 
        queue_work(ifx_gspi_data->ifx_wq, &ifx_gspi_data->ifx_work);
    }
    
    //prevent duplicated access to 'spi_data->is_suspended'
    spin_lock_irqsave(&spi_data->spi_lock, flags);
    spi_data->is_suspended = 0;
    spin_unlock_irqrestore(&spi_data->spi_lock, flags);


    IFX_SPI_PRINTK("end");

    return 0;
}
#endif //CONFIG_PM

/* End of TTY - SPI driver Operations */

/* ################################################################################################################ */

static const struct spi_device_id star_spi_ids[] = {
	{ "ifxn721", 0 },
	{ /* end of list */ },
};

static struct spi_driver ifx_spi_driver = {
	.probe = ifx_spi_probe,
	.remove = __devexit_p(ifx_spi_remove),
#ifdef CONFIG_PM	
	.suspend = ifx_spi_suspend,
	.resume = ifx_spi_resume,
#endif
	 //.id_table  = star_spi_ids,
	.driver = {
		.name = "ifxn721",
                .bus = &spi_bus_type,
		.owner = THIS_MODULE,
	}	

};

/*
 * Structure to specify tty core about tty driver operations supported in TTY SPI driver.
 */
static const struct tty_operations ifx_spi_ops = {
    .open = ifx_spi_open,
    .close = ifx_spi_close,
    .write = ifx_spi_write,
    .write_room = ifx_spi_write_room,
    //.throttle = ifx_spi_throttle,
    //.unthrottle = ifx_spi_unthrottle,
    //.set_termios = ifx_spi_set_termios,
};

/* ################################################################################################################ */

/*
 * Intialize frame sizes as "IFX_SPI_DEFAULT_BUF_SIZE"(128) bytes for first SPI frame transfer
 */
static void ifx_spi_buffer_initialization(struct ifx_spi_data *spi_data)
{
	spi_data->ifx_sender_buf_size = IFX_SPI_DEFAULT_BUF_SIZE;
       spi_data->ifx_receiver_buf_size = IFX_SPI_DEFAULT_BUF_SIZE;
}

/*
 * Allocate memeory for TX_BUFFER and RX_BUFFER
 */
static int ifx_spi_allocate_frame_memory(struct ifx_spi_data *spi_data, unsigned int memory_size)
{
	int status = 0;
	spi_data->ifx_rx_buffer = kmalloc(memory_size+IFX_SPI_HEADER_SIZE, GFP_KERNEL);
	if (!spi_data->ifx_rx_buffer)
	{
		IFX_SPI_PRINTK("Open Failed ENOMEM");
		status = -ENOMEM;
	}
	
	spi_data->ifx_tx_buffer = kmalloc(memory_size+IFX_SPI_HEADER_SIZE, GFP_KERNEL);
	if (!spi_data->ifx_tx_buffer)
	{		
		IFX_SPI_PRINTK("Open Failed ENOMEM");
		status = -ENOMEM;
	}
	
	if(status == -ENOMEM)
	{
		if(spi_data->ifx_tx_buffer)
		{
			kfree(spi_data->ifx_tx_buffer);
		}
		
		if(spi_data->ifx_rx_buffer)
		{
			kfree(spi_data->ifx_rx_buffer);            
		}
	}
	return status;
}

static void ifx_spi_free_frame_memory(struct ifx_spi_data *spi_data)
{
	if(spi_data->ifx_tx_buffer)
	{
		kfree(spi_data->ifx_tx_buffer);
	}
	
	if(spi_data->ifx_rx_buffer)
	{
		kfree(spi_data->ifx_rx_buffer);            
	}
}


/*
 * Function to set header information according to IFX SPI framing protocol specification
 */
static void ifx_spi_set_header_info(unsigned char *header_buffer, unsigned int curr_buf_size, unsigned int next_buf_size)
{
	int i;
	union ifx_spi_frame_header header;
	for(i=0; i<4; i++)
	{
		header.framesbytes[i] = 0;
	}

	header.ifx_spi_header.curr_data_size = curr_buf_size;
	if(next_buf_size)
	{
		header.ifx_spi_header.more=1;
		header.ifx_spi_header.next_data_size = next_buf_size;
	}
	else
	{
		header.ifx_spi_header.more=0;
		header.ifx_spi_header.next_data_size = 128;
	}

	for(i=3; i>=0; i--)
	{
		header_buffer[i] = header.framesbytes[/*3-*/i];
	}

	IFX_SPI_DEBUG("header (more=%d, next_data_size=%d)", 
					header.ifx_spi_header.more, header.ifx_spi_header.next_data_size);
	
}

/*
 * Function to get header information according to IFX SPI framing protocol specification
 */
static int ifx_spi_get_header_info(unsigned char *rx_buffer, unsigned int *valid_buf_size)
{
	int i;
	union ifx_spi_frame_header header;

	for(i=0; i<4; i++)
	{
		header.framesbytes[i] = 0;
	}

	for(i=3; i>=0; i--)
	{
		header.framesbytes[i] = rx_buffer[/*3-*/i];
	}

//	Discard if mux size is bigger than MAX SIZE
	if(header.ifx_spi_header.curr_data_size>IFX_SPI_MAX_BUF_SIZE)	
	{
		IFX_SPI_PRINTK("Invalid Frame Header : 0x%x 0x%x 0x%x 0x%x!!!\n", 
				header.framesbytes[0], header.framesbytes[1], 
				header.framesbytes[2], header.framesbytes[3]);
		
		*valid_buf_size = 0;
	}
	else
	{
		*valid_buf_size = header.ifx_spi_header.curr_data_size;
	}
	
	if(header.ifx_spi_header.more)
	{
		return header.ifx_spi_header.next_data_size;
	}
	return 0;
}

/*
 * Function to set/reset MRDY signal
 */
static void ifx_spi_set_mrdy_signal(int value)
{
	//gpio_set_value(IFX_MRDY_GPIO, value);
	 IFX_SPI_DEBUG(", value : %x",value);
	// gpio_set_value(IFX_MRDY_GPIO, value);
	gpio_direction_output(IFX_MRDY, value);
}

/*
 * Function to calculate next_frame_size required for filling in SPI frame Header
 */
static int ifx_spi_get_next_frame_size(int count)
{
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
 * Function to setup transmission and reception. It implements a logic to find out the ifx_current_frame_size,
 * valid_frame_size and sender_next_frame_size to set in SPI header frame. Copys the data to be transferred from 
 * user space to TX buffer and set MRDY signal to HIGH to indicate Master is ready to transfer data.
 */

static void ifx_spi_setup_transmission(struct ifx_spi_data *spi_data)
{
	IFX_SPI_DEBUG("sender buf=%d, reciver buf=%d", spi_data->ifx_sender_buf_size, spi_data->ifx_receiver_buf_size);

	if( (spi_data->ifx_sender_buf_size != 0) || (spi_data->ifx_receiver_buf_size != 0) )
	{
		if(spi_data->ifx_sender_buf_size > spi_data->ifx_receiver_buf_size)
		{
			spi_data->ifx_current_frame_size = spi_data->ifx_sender_buf_size;
		}
		else
		{ 
			spi_data->ifx_current_frame_size = spi_data->ifx_receiver_buf_size;    
		}
		
		if(spi_data->ifx_spi_count > 0)
		{
			if(spi_data->ifx_spi_count > spi_data->ifx_current_frame_size)
			{
				spi_data->ifx_valid_frame_size = spi_data->ifx_current_frame_size;
				spi_data->ifx_spi_count = spi_data->ifx_spi_count - spi_data->ifx_current_frame_size;
			}
			else
			{
				spi_data->ifx_valid_frame_size = spi_data->ifx_spi_count;
				spi_data->ifx_spi_count = 0;
			}
                }
		else
		{
			spi_data->ifx_valid_frame_size = 0;
			spi_data->ifx_sender_buf_size = 0;
		}

		spi_data->ifx_sender_buf_size = ifx_spi_get_next_frame_size(spi_data->ifx_spi_count);
		IFX_SPI_DEBUG("reciver buf=%d",spi_data->ifx_sender_buf_size);

		/* memset buffers to 0 */
		memset(spi_data->ifx_tx_buffer,0,IFX_SPI_MAX_BUF_SIZE+IFX_SPI_HEADER_SIZE);
		memset(spi_data->ifx_rx_buffer,0,IFX_SPI_MAX_BUF_SIZE+IFX_SPI_HEADER_SIZE);

		/* Set header information */
		ifx_spi_set_header_info(spi_data->ifx_tx_buffer, spi_data->ifx_valid_frame_size, spi_data->ifx_sender_buf_size);
		if( spi_data->ifx_valid_frame_size > 0 )
		{      
			memcpy(spi_data->ifx_tx_buffer+IFX_SPI_HEADER_SIZE, spi_data->ifx_spi_buf, spi_data->ifx_valid_frame_size);
			spi_data->ifx_spi_buf = spi_data->ifx_spi_buf + spi_data->ifx_valid_frame_size;
		}

	}
}


/*
 * Function starts Read and write operation and transfers received data to TTY core. It pulls down MRDY signal
 * in case of single frame transfer then sets "ifx_read_write_completion" to indicate transfer complete.
 */
static void ifx_spi_send_and_receive_data(struct ifx_spi_data *spi_data)
{
	unsigned int rx_valid_buf_size;
	int status = 0; 
	int recieve_copied ;
	
	IFX_SPI_DEBUG(" curr frame size = %d, total size=%d\n ",
					spi_data->ifx_current_frame_size, spi_data->ifx_current_frame_size+IFX_SPI_HEADER_SIZE) ;
	
	status = ifx_spi_sync_read_write(spi_data, spi_data->ifx_current_frame_size+IFX_SPI_HEADER_SIZE); /* 4 bytes for header */ 
	IFX_SPI_DEBUG("status = %d ", status);	

#ifdef IFX_SPI_DUMP_LOG
		IFX_SPI_PRINTK("SPI TX	----------------- ");
		ifx_dump_atcmd(spi_data->ifx_tx_buffer+IFX_SPI_HEADER_SIZE+2);	
#endif	
	
	if(status > 0)
	{
		memset(spi_data->ifx_tx_buffer,0,IFX_SPI_MAX_BUF_SIZE+IFX_SPI_HEADER_SIZE);
		spi_data->ifx_ret_count = spi_data->ifx_ret_count + spi_data->ifx_valid_frame_size;
	}

       //ifx modem에서만 rx buf에 data 없으면 return... 
	if (*((int*)spi_data->ifx_rx_buffer) == 0xFFFFFFFF)
	{
		spi_data->ifx_receiver_buf_size = 0;
	        IFX_SPI_DEBUG("received data is nothing..");
			
		return;
	}

	/* Handling Received data */
	spi_data->ifx_receiver_buf_size = ifx_spi_get_header_info(spi_data->ifx_rx_buffer, &rx_valid_buf_size);

        // spi에서 rx data가 존재하면 tty 로 전송
	if((spi_data->throttle == 0) && (rx_valid_buf_size != 0))
	{ 
		if(spi_data->ifx_tty)
		{
#if defined(LGE_DUMP_SPI_BUFFER)
		dump_spi_buffer("SPI RX", &spi_data->ifx_rx_buffer[4], COL_SIZE);
#endif		
#ifdef IFX_SPI_DUMP_LOG
		 IFX_SPI_PRINTK("SPI RX ----------------- ");
		 ifx_dump_atcmd(spi_data->ifx_rx_buffer+IFX_SPI_HEADER_SIZE+2) ;	 
#endif	

		 /* spi에서 rx data가 존재하면 tty 로 전송	*/
#ifdef IFX_SPI_SPEED_MEASUREMENT
			uiRxlen[spi_data->ifx_tty->index] = rx_valid_buf_size+IFX_SPI_HEADER_SIZE;
#endif		
			recieve_copied = tty_insert_flip_string(spi_data->ifx_tty,
											(spi_data->ifx_rx_buffer+IFX_SPI_HEADER_SIZE),
											rx_valid_buf_size);
			// if(recieve_copied)
			tty_flip_buffer_push(spi_data->ifx_tty);
			//else 
			//	printk("tty_insert_flip_string err") ;
		}
		else 
		{
			IFX_SPI_DEBUG("no tty err ") ;
		}
	}  
	/*else
  	{ 
	handle RTS and CTS in SPI flow control
	Reject the packet as of now 
	}*/
#ifdef WAKE_LOCK_RESUME
	if(spi_data->wake_lock_flag)
	{
		spi_data->wake_lock_flag = 0;
	}
#endif	
}

/*
 * Function copies the TX_BUFFER and RX_BUFFER pointer to a spi_transfer structure and add it to SPI tasks.
 * And calls SPI Driver function "spi_sync" to start data transmission and reception to from MODEM
 */
 extern void kernel_restart(char *cmd);

static unsigned int ifx_spi_sync_read_write(struct ifx_spi_data *spi_data, unsigned int len)
{
	bool spi_suspend_failed;

	int status;
	int ipc_check;
	struct spi_message	m;
	struct spi_transfer	t = {
					    .tx_buf		= spi_data->ifx_tx_buffer,
			                    .rx_buf		= spi_data->ifx_rx_buffer,
					    .len		= len,
				    };

#ifdef IFX_SPI_TEGRA_TRANSFER_DURATION	
	static struct timeval transfer_start_time;
	static struct timeval transfer_end_time;
	unsigned long duration_time;
#endif

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);

	IFX_SPI_DEBUG("");	
	if (spi_data->spi == NULL)
	{
		IFX_SPI_PRINTK("spi_data->spi = NULL");			
		status = -ESHUTDOWN;
	}
	else
	{
#ifdef IFX_SPI_TEGRA_TRANSFER_DURATION		
		IFX_SPI_PRINTK("!!! spi_transfer start !!!");
		do_gettimeofday(&transfer_start_time);		
		status = spi_sync(spi_data->spi, &m);
		do_gettimeofday(&transfer_end_time);
		duration_time = (transfer_end_time.tv_usec - transfer_start_time.tv_usec); //sec		
		IFX_SPI_PRINTK("!!! spi_transfer end is %06d ms  !!!", (duration_time/1000)); //milisec
#else		
		status = spi_sync(spi_data->spi, &m);
#endif
	}

	if (status == 0)
	{          
		status = m.status;
		if (status == 0)
		{
			status = m.actual_length;
		}
        	//reset 'count_transfer_failed' to zero, if spi transter succeeds at least one out of five times
	        count_transfer_failed = 0;		
	}
        else
	{
		ipc_check = ifx_modem_communicating();
		if(ipc_check == 0) 
		{
			IFX_SPI_PRINTK("transmission unsuccessful, [spi_sync] status:%d, count_Failed:%d\n", status, count_transfer_failed);
				
			spi_suspend_failed = spi_tegra_suspend_failed(spi_data->spi);
			if (spi_suspend_failed)
			{
				IFX_SPI_PRINTK("kernel_restart!!!, spi_suspend_failed=%d \n", spi_suspend_failed);		 
				kernel_restart(NULL);
			 }				
		}
		//increase 'count_transfer_failed', when spi transter fails
		count_transfer_failed++;		
    }
#ifdef IFX_SPI_TX_RX_BUF
        IFX_SPI_PRINTK("SPI TX BUFFER: ");
        for(i=0;i<16;i++)
	{
        	printk( "%02x ",spi_data->ifx_tx_buffer[i]);
        }
        IFX_SPI_PRINTK("\n");
        
        IFX_SPI_PRINTK("SPI RX BUFFER : ");
        for(i=0;i<16;i++)
	{	
        	printk( "%02x ",spi_data->ifx_rx_buffer[i]);
        }
#endif
	return status;
}

/*
 * Function is a Interrupt service routine, is called when SRDY signal goes HIGH. It set up transmission and
 * reception if it is a Slave initiated data transfer. For both the cases Master intiated/Slave intiated
 * transfer it starts data transfer. 
 */
static irqreturn_t ifx_spi_handle_srdy_irq(int irq, void *handle)
{
	struct ifx_spi_data *spi_data = (struct ifx_spi_data *)handle;
	int pin_val;	
	IFX_SPI_DEBUG("");

#ifdef IFX_TEGRA_EDGE_TRIGGER
	pin_val = gpio_get_value(IFX_SRDY);
	IFX_SPI_DEBUG("pin_val = %d", pin_val);

       if(pin_val == 0)
       {
	       printk("[SPI][SRDY_IRQ] pin value is ZERO.. Return!! \n");
		 IFX_SPI_DEBUG(" IRQF_TRIGGER_FALLING in the srdy irq is ignore !!! \n");
        	 return IRQ_HANDLED;
       }
#endif	   

#if 0	  
	if(spi_data && spi_data->ifx_tty)	//add to prevent the irq of srdy until spi opening
	{
		IFX_SPI_DEBUG("queue_work is done!");		
		queue_work(spi_data->ifx_wq, &spi_data->ifx_work);    
	}
	else
	{
		IFX_SPI_PRINTK("Unexpected interrupt happen!");	
		IFX_SPI_PRINTK("spi_data = 0x%p, 0x%p, spi_data->ifx_tty =0x%p", spi_data, spi_data->ifx_tty);			
	}
#else

#ifdef WAKE_LOCK_RESUME // HZ is 1sec
		IFX_SPI_DEBUG("[IFX_SRDY] wake lock : 0x%lx", &ifx_gspi_data->wake_lock);
		wake_lock_timeout(&ifx_gspi_data->wake_lock, msecs_to_jiffies(500));	//5,, Unexpected interrupt or power consumption
#endif	
	IFX_SPI_DEBUG("queue_work is done!");		
	queue_work(spi_data->ifx_wq, &spi_data->ifx_work);	  
#endif
	return IRQ_HANDLED; 
}


// ifx_master_initiated_transfer = 1; --> set called by ifx_spi_write() 
// ifx_master_initiated_transfer = 0; --> default 
static void ifx_spi_handle_work(struct work_struct *work)
{
	bool spi_tegra_suspended;

	struct ifx_spi_data *spi_data = container_of(work, struct ifx_spi_data, ifx_work);

#ifdef IFX_SPI_SPEED_MEASUREMENT
	int id = 0;
	unsigned long diff;
#endif

	IFX_SPI_DEBUG( " start");		
	// 20120904 jisil.park
    unsigned long reg;
    int pm_off_count;

    if(1 == spi_data->is_suspended)
	{
	   pm_off_count = 1;
	   printk("[SPI][handle_work] ifx_spi_handle_work INFO spi_data->is_suspended is (0x%x)\n", spi_data->is_suspended);

	   //wait for ap to return to resume state with a worst case scenario of 5sec
	   do
	   {		   
		   mdelay(1);
		   pm_off_count++;
		   
	   }while((1 == spi_data->is_suspended) && (pm_off_count<(5*200)));

	   printk("[EBS] ifx_spi_handle_work INFO EXIT is_suspend = 0x%x pm_off_count=%d\n", spi_data->is_suspended, pm_off_count);

	   if(1 == spi_data->is_suspended)
	   {
		  // To Do how to handle the PM OFF state during 1sec
		  printk("[SPI][handle_work] ifx_spi_handle_work error is_suspended is (0x%x)\n",spi_data->is_suspended);
	   }
    }
    // 20120904 jisil.park
	
	/* add to wait tx/rx data when tegra spi is suspended*/
	spi_tegra_suspended = spi_tegra_is_suspend(spi_data->spi);
	if (spi_tegra_suspended) 
	{
		IFX_SPI_PRINTK("spi_tegra is not resume !, spi_tegra_suspended = %d\n",spi_tegra_suspended);
		return;		
	}

	if (!spi_data->ifx_master_initiated_transfer)
	{
		IFX_SPI_TX_DEBUG("CP Start =================> \n");
#ifdef IFX_SPI_SPEED_MEASUREMENT
		do_gettimeofday(&ulStart[id]);
#endif		
		ifx_spi_setup_transmission(spi_data);
		ifx_spi_set_mrdy_signal(1);
		ifx_spi_send_and_receive_data(spi_data);

#ifdef IFX_SPI_SPEED_MEASUREMENT
		do_gettimeofday(&ulEnd[id]);
		diff = (ulEnd[id].tv_sec - ulStart[id].tv_sec) * 1000 * 1000 ;
		diff = (diff + (ulEnd[id].tv_usec - ulStart[id].tv_usec));
		ulRxThroughtput[id] = ((uiRxlen[id]*8*1000)/diff);
		IFX_SPI_PRINTK("[SPI %d] : RX time = %09d usec; %04d bytes; %06lu Kbps", 
						id, diff, IFX_SPI_MAX_BUF_SIZE+IFX_SPI_HEADER_SIZE, 
						((IFX_SPI_MAX_BUF_SIZE+IFX_SPI_HEADER_SIZE)*8*1000)/diff);
#endif

		
		/* Once data transmission is completed, the MRDY signal is lowered */
		if((spi_data->ifx_sender_buf_size == 0)  && (spi_data->ifx_receiver_buf_size == 0))
		{
			ifx_spi_set_mrdy_signal(0);
			ifx_spi_buffer_initialization(spi_data);
		}
		/* We are processing the slave initiated transfer in the mean time Mux has requested master initiated data transfer */
		/* Once Slave initiated transfer is complete then start Master initiated transfer */
		if(spi_data->ifx_master_initiated_transfer == 1)  //why check ? already ifx_master_initiated_transfer = 0
		{
		/* It is a condition where Slave has initiated data transfer and both SRDY and MRDY are high and at the end of data transfer		
	 	* MUX has some data to transfer. MUX initiates Master initiated transfer rising MRDY high, which will not be detected at Slave-MODEM.
	 	* So it was required to rise MRDY high again */
			udelay(MRDY_DELAY_TIME) ;	 	
	                ifx_spi_set_mrdy_signal(1);    		
		}
		IFX_SPI_TX_DEBUG("CP End =================> \n");			
	}
	else
	{
		IFX_SPI_TX_DEBUG("Interrupt by AP25 ===========> \n");		
		
		ifx_spi_setup_transmission(spi_data);     
			
#if defined(LGE_DUMP_SPI_BUFFER)
	dump_spi_buffer("SPI TX", &spi_data->ifx_tx_buffer[4], COL_SIZE);
#endif		
		
		ifx_spi_send_and_receive_data(spi_data);

		/* Once data transmission is completed, the MRDY signal is lowered */
		if(spi_data->ifx_sender_buf_size == 0)
		{
			if(spi_data->ifx_receiver_buf_size == 0)
			{		
				ifx_spi_set_mrdy_signal(0);
				udelay(MRDY_DELAY_TIME) ;				
				ifx_spi_buffer_initialization(spi_data);
			}

			IFX_SPI_TX_DEBUG("ifx_master_initiated_transfer set =  0 <============== \n");		
			spi_data->ifx_master_initiated_transfer = 0;
			complete(&spi_data->ifx_read_write_completion);
		}
	}

#ifdef IFX_SPI_TX_RX_THROUGHTPUT
	if(uiTxlen[spi_data->mdm_tty->index] || uiRxlen[spi_data->mdm_tty->index]) 
	{
		 //ulEnd = getuSecTime() - ulStart;
		 do_gettimeofday(&ulEnd[spi_data->ifx_tty->index]);
		 
		 uidiff[spi_data->ifx_tty->index] = (ulEnd[spi_data->ifx_tty->index].tv_sec - ulStart[spi_data->ifx_tty->index].tv_sec) * 1000 * 1000 ;
		 uidiff[spi_data->ifx_tty->index] = uidiff[spi_data->ifx_tty->index] + (ulEnd[spi_data->ifx_tty->index].tv_usec - ulStart[spi_data->ifx_tty->index].tv_usec);
		 ulTxThroughtput[spi_data->ifx_tty->index] = ((uiTxlen[spi_data->ifx_tty->index]*8*1000)/uidiff[spi_data->ifx_tty->index]);
		 ulRxThroughtput[spi_data->ifx_tty->index] = ((uiRxlen[spi_data->ifx_tty->index]*8*1000)/uidiff[spi_data->ifx_tty->index]);

	 	IFX_SPI_PRINTK("[SPI %d] time	= %d us, Tx(%dbytes) = %luKbps, Rx(%dbytes) = %luKbps, Max(%dbytes) = %luKbps\n", 
						spi_data->ifx_tty->index, uidiff[spi_data->mdm_tty->index], 
						uiTxlen[spi_data->ifx_tty->index], ulTxThroughtput[spi_data->ifx_tty->index], 
						uiRxlen[spi_data->ifx_tty->index], ulRxThroughtput[spi_data->ifx_tty->index],
						IFX_SPI_MAX_BUF_SIZE+IFX_SPI_HEADER_SIZE, 
						((IFX_SPI_MAX_BUF_SIZE+IFX_SPI_HEADER_SIZE)*8*1000)/uidiff[spi_data->ifx_tty->index]);

		uiTxlen[spi_data->ifx_tty->index] = uiRxlen[spi_data->ifx_tty->index] = 0;
		fWrite[spi_data->ifx_tty->index] = 0;
	}
#endif

	IFX_SPI_DEBUG( " end");
}


/* ################################################################################################################ */


/* ################################################################################################################ */

/* Initialization Functions */

/*
 * Initialization function which allocates and set different parameters for TTY SPI driver. Registers the tty driver 
 * with TTY core and register SPI driver with the Kernel. It validates the GPIO pins for MRDY and then request an IRQ
 * on SRDY GPIO pin for SRDY signal going HIGH. In case of failure of SPI driver register cases it unregister tty driver
 * from tty core.
 */
static int __init ifx_spi_init(void)
{
	int status = 0;

	/* Allocate and Register a TTY device */
	ifx_spi_tty_driver = alloc_tty_driver(IFX_N_SPI_MINORS);
	if (!ifx_spi_tty_driver)
	{
		IFX_SPI_PRINTK("Fail to allocate TTY Driver");
		return -ENOMEM;
	}

	/* initialize the tty driver */
	ifx_spi_tty_driver->owner = THIS_MODULE;
	ifx_spi_tty_driver->driver_name = "ifxn721";
	ifx_spi_tty_driver->name = "ttyspi";
	ifx_spi_tty_driver->major = IFX_SPI_MAJOR;
	ifx_spi_tty_driver->minor_start = 0;
	ifx_spi_tty_driver->type = TTY_DRIVER_TYPE_SERIAL;
	ifx_spi_tty_driver->subtype = SERIAL_TYPE_NORMAL;
	ifx_spi_tty_driver->flags = TTY_DRIVER_REAL_RAW;
	ifx_spi_tty_driver->init_termios = tty_std_termios;
	ifx_spi_tty_driver->init_termios.c_cflag = B38400/*B9600*/ | CS8 | CREAD | HUPCL | CLOCAL;
	tty_set_operations(ifx_spi_tty_driver, &ifx_spi_ops);

	status = tty_register_driver(ifx_spi_tty_driver);
	if (status)
	{
		IFX_SPI_PRINTK("Failed to register IFX SPI tty driver");
		put_tty_driver(ifx_spi_tty_driver);
		return status;
	}

	/* Register SPI Driver */

	status = spi_register_driver(&ifx_spi_driver);
	if (status < 0)
	{ 
		IFX_SPI_PRINTK("Failed to register SPI device");
		tty_unregister_driver(ifx_spi_tty_driver);
		put_tty_driver(ifx_spi_tty_driver);
		return status;
	}

	printk("**** %s \n",__FUNCTION__);
	return status;
}

module_init(ifx_spi_init);


/*
 * Exit function to unregister SPI driver and tty SPI driver
 */
static void 
__exit ifx_spi_exit(void)
{  
	spi_unregister_driver(&ifx_spi_driver);
	tty_unregister_driver(ifx_spi_tty_driver);
        put_tty_driver(ifx_spi_tty_driver);
}

module_exit(ifx_spi_exit);

/* End of Initialization Functions */

/* ################################################################################################################ */

MODULE_AUTHOR("Umesh Bysani and Shreekanth D.H, <bysani@ti.com> <sdh@ti.com>");
MODULE_DESCRIPTION("IFX SPI Framing Layer");
MODULE_LICENSE("GPL");
