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
#include <linux/earlysuspend.h>
#include <linux/spinlock.h>

#define NV_DEBUG 0
//
#include "nvos.h"
#include "nvcommon.h"
#include "nvodm_services.h"
#include "nvodm_query_discovery.h"

#include <linux/spi/ifx_n721_spi.h>
//#define CONFIG_EARLY_SUSPEND_TEST

#define WAKE_LOCK_RESUME
#ifdef WAKE_LOCK_RESUME
#include <linux/wakelock.h>
#endif

#define SPI_GUID                NV_ODM_GUID('s','t','a','r','-','s','p','i')

//LGE_TELECA_CR:1056_SPI/MUX_IMPROVEMENT START
#define SPI_SEND_MORE_ATTR
//LGE_TELECA_CR:1056_SPI/MUX_IMPROVEMENT END
//LGE_TELECA_CR1317_DATA_THROUGHPUT START
#define SPI_MORE_ATTR 0x8000

//LGE_TELECA_CR:707_DATA_THROUGHPUT START
//#define CONFIG_SPI_DEBUG
#ifdef CONFIG_SPI_DEBUG
#define SPI_DEBUG_PRINT(format, args...) printk(format, ## args)
#define SPI_LOG(format, args...) printk("[  SPI] : %s %d: " format "\n", __FUNCTION__, __LINE__, ## args)
#else
#if CONFIG_LPRINTK
#include <mach/lprintk.h> //20100426, , Change printk to lprintk
#define SPI_DEBUG_PRINT(format, args...) lprintk(D_SPI, format, ## args)
#define SPI_LOG(format, args...) lprintk(D_SPI, "%s %d: " format "\n", __FUNCTION__, __LINE__, ## args)
#else
#define SPI_DEBUG_PRINT(format, args...) {}
#define SPI_LOG(format, args...) {}
#endif
#endif

//#define LGE_SPI_DEBUG_LOG
#ifdef LGE_SPI_DEBUG_LOG
#define SPI_DEBUG_LOG SPI_LOG
#else
#define SPI_DEBUG_LOG(...)
#endif

#define LOCK_T          spinlock_t
#define CREATELOCK(_l)  spin_lock_init(&(_l))
#define DELETELOCK(_l)
#define LOCK(_l)        spin_lock_bh(&(_l))
#define WAIT_UNLOCK(_l)	spin_unlock_wait(&(_l))
#define UNLOCK(_l)      spin_unlock_bh(&(_l))
#define ATOMIC(_l,_f)   spin_lock_irqsave(&(_l),(_f))
#define UNATOMIC(_l,_f) spin_unlock_irqrestore(&(_l),(_f))

static LOCK_T spi_suspend_lock;


static int ifx_shutdown = 0;
static int is_suspended = 0;


#ifdef LGE_SPI_DEBUG_LOG
void DUMP_SPI_BUFFER(unsigned char *data, unsigned int len)
{
  int begin_count = 20;
  int end_count = 20;
  int j;

#ifdef CONFIG_LPRINTK
  if (!lge_debug[D_SPI].enable)
    return;
#endif //CONFIG_SPI_DEBUG

  if (len < begin_count)
  {
    begin_count = len;
  }
  
  if (data != NULL && len > 0)
  {
    for (j=0; j < begin_count; ++j)
    {
      if(data[j]>=32 && data[j]<=126)
        printk("%c ", data[j]);
      else
        printk("%02X ", data[j]);
    }

    if (len > begin_count)
    {
      if (len > (begin_count + end_count))
      {
        printk("... ");
        j = len - end_count;
      }

      for (; j < len; ++j)
      {
        if(data[j]>=32 && data[j]<=126)
          printk("%c ", data[j]);
        else
          printk("%02X ", data[j]);
      }
    }
    
    printk("\n");
  }
  else
  {
    printk("no data [len=%d]\n", len);
  }
}
#endif
//LGE_TELECA_CR:707_DATA_THROUGHPUT END
//LGE_TELECA_CR1317_DATA_THROUGHPUT END

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
//LGE_TELECA_CR:1056_SPI/MUX_IMPROVEMENT START
//LGE_TELECA_CR1317_DATA_THROUGHPUT START
	unsigned		more_rx;
	unsigned		more_tx;
//LGE_TELECA_CR1317_DATA_THROUGHPUT END
//LGE_TELECA_CR:1056_SPI/MUX_IMPROVEMENT END
	unsigned		users;
        unsigned int		throttle;
        struct work_struct      ifx_work;
        struct work_queue_struct *ifx_wq;
//20100811-1, , move global variables [START]
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
//20100811, , move global variables [END]
#ifdef WAKE_LOCK_RESUME
	struct wake_lock wake_lock;
#endif
#ifdef CONFIG_EARLY_SUSPEND_TEST
		struct early_suspend early_suspend;
#endif

	unsigned int		wake_lock_flag;
	unsigned int 		is_suspended; //EBS 
	NvOdmServicesGpioHandle hGpio;
	NvOdmGpioPinHandle hMrdyPin;
	NvOdmServicesGpioIntrHandle hMrdyInterrupt;
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

/*
struct GpioHandle{
	struct NvOdmServicesGpioHandle hGpio;
	struct NvOdmGpioPinHandle hPin; 
};*/
#define MAX_USED_SPI_DREVICE	2
struct ifx_spi_data	*gspi_data[MAX_USED_SPI_DREVICE];
struct tty_driver 	*ifx_spi_tty_driver;

/* ################################################################################################################ */
/* Global Declarations */
unsigned long		minors[IFX_N_SPI_MINORS / BITS_PER_LONG];
/*//20100811-1, , move global variables [START]
unsigned int		ifx_master_initiated_transfer = 0;
unsigned int		ifx_spi_count;
unsigned int		ifx_sender_buf_size;
unsigned int		ifx_receiver_buf_size;
unsigned int		ifx_current_frame_size;
unsigned int		ifx_valid_frame_size;
unsigned int		ifx_ret_count;
const unsigned char 	*ifx_spi_buf;
unsigned char		*ifx_tx_buffer;
unsigned char           *ifx_rx_buffer;
*///20100811, , move global variables [END]
/* Function Declarations */

#ifdef CONFIG_EARLY_SUSPEND_TEST
static void ifx_spi_early_suspend(struct early_suspend *h);
static void ifx_spi_late_resume(struct early_suspend *h);
#endif

//LGE_TELECA_CR1317_DATA_THROUGHPUT START
static void ifx_spi_set_header_info(unsigned char *header_buffer, unsigned int curr_buf_size, unsigned int next_buf_size, int more);
//LGE_TELECA_CR1317_DATA_THROUGHPUT END
static int ifx_spi_get_header_info(struct ifx_spi_data *spi_data, unsigned int *valid_buf_size);
static void ifx_spi_set_srdy_signal(int value);
//static irqreturn_t ifx_spi_handle_mrdy_irq(int irq, void *handle);
static void ifx_spi_handle_mrdy_irq(void *handle);
static void ifx_spi_setup_transmission(struct ifx_spi_data *spi_data);
static void ifx_spi_send_and_receive_data(struct ifx_spi_data *spi_data);
static int ifx_spi_get_next_frame_size(int count);
static int ifx_spi_allocate_frame_memory(struct ifx_spi_data *spi_data, unsigned int memory_size);
static void ifx_spi_buffer_initialization(struct ifx_spi_data *spi_data);
static unsigned int ifx_spi_sync_read_write(struct ifx_spi_data *spi_data, unsigned int len);
static void ifx_spi_handle_work(struct work_struct *work);

// hgahn
unsigned char rx_dummy[]={0xff,0xff,0xff,0xff};

//To check the step of SPI process
static unsigned int uiStepCount = 0;

#define USE_SRDY
#ifndef USE_SRDY
static NvOdmServicesGpioHandle hGpio = NULL;
static NvOdmGpioPinHandle hPin = NULL; 

#define ENABLE_SRDY_IRQ(irq)	enable_irq(irq);
#define DISABLE_SRDY_IRQ(irq)	disable_irq(irq);
#else
#define ENABLE_SRDY_IRQ(irq)	
#define DISABLE_SRDY_IRQ(irq)	
#endif

//20100626-1, , throughput check [START]
//#define SPEED_CHECK
#ifdef SPEED_CHECK
#include <linux/time.h>
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
//20100626-1, , throughput check [END]

//20100701-1, , delay time until CP can be ready again [START]
#include <linux/delay.h>
#define MRDY_DELAY_TIME	100
//20100701-1, , delay time until CP can be ready again [END]

/* ################################################################################################################ */

static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);

/* ################################################################################################################ */

/* IFX SPI Operations */

/*
 * Function opens a tty device when called from user space
 */
static int 
ifx_spi_open(struct tty_struct *tty, struct file *filp)
{
	int status = 0;
	struct ifx_spi_data *spi_data;
	if (ifx_shutdown) return 0;

	SPI_LOG("[id: %d]", tty->index);

	if(gspi_data[tty->index])
	{
		spi_data = gspi_data[tty->index];
		spi_data->ifx_tty = tty;
		tty->driver_data = spi_data;
		ifx_spi_buffer_initialization(spi_data);
		spi_data->throttle = 0;
		printk("%s - buff size = %d\n", __FUNCTION__, IFX_SPI_MAX_BUF_SIZE);
	}
	else
	{
		//ifx_spi_buffer_initialization();
		tty->driver_data = NULL;
		SPI_LOG("ifx_spi_open failed!");
		status = -ENODEV;
	}

	return status;
}

/*
 * Function closes a opened a tty device when called from user space
 */
static void 
ifx_spi_close(struct tty_struct *tty, struct file *filp)
{  
	//struct ifx_spi_data *spi_data = (struct ifx_spi_data *)tty->driver_data;
	//spi_data->ifx_tty = NULL;
	//tty->driver_data = NULL;
	printk("[LGE-SPI] ifx_spi_close [id: %d] \n", tty->index);
}

/*
 * Function is called from user space to send data to MODEM, it setups the transmission, enable MRDY signal and
 * waits for SRDY signal HIGH from MDOEM. Then starts transmission and reception of data to and from MODEM.
 * Once data read from MODEM is transferred to TTY core flip buffers, then "ifx_read_write_completion" is set
 * and this function returns number of bytes sent to MODEM
 */
static int 
ifx_spi_write(struct tty_struct *tty, const unsigned char *buf, int count)
{
//LGE_TELECA_CR1317_DATA_THROUGHPUT START
	struct ifx_spi_data *spi_data = (struct ifx_spi_data *)tty->driver_data;
	if (ifx_shutdown) return 0;

	int _count = (count & ~SPI_MORE_ATTR);

//20100626-1, , throughput check [START]
#ifdef SPEED_CHECK
	int id = tty->index;
	unsigned long diff;
	fWrite[id] = 1;
	uiTxlen[id] = _count + IFX_SPI_HEADER_SIZE;
	//ulStart = getuSecTime();
	do_gettimeofday(&ulStart[id]);
#endif
//20100626-1, , throughput check [END]
	if(spi_data==NULL)
	{
		SPI_LOG("no spi handle");
		return 0;
	}

	if( !buf ){
		SPI_LOG("buffer is NULL");
		return 0;
	}
	if(!_count){
		SPI_LOG("count is ZERO");
		return 0;
	}

	disable_irq(spi_data->spi->irq);
	spi_data->more_tx = (count & SPI_MORE_ATTR);
	spi_data->ifx_ret_count = 0;
	spi_data->ifx_tty = tty;
	spi_data->ifx_tty->low_latency = 1;
	spi_data->ifx_master_initiated_transfer = 1;
	spi_data->ifx_spi_buf = buf;
	spi_data->ifx_spi_count = _count;
	enable_irq(spi_data->spi->irq);

	ifx_spi_set_srdy_signal(1);
#ifdef USE_SRDY
	queue_work(spi_data->ifx_wq, &spi_data->ifx_work);    
#endif
	SPI_DEBUG_LOG("[id: %d]", tty->index);
//LGE_TELECA_CR1317_DATA_THROUGHPUT END
	wait_for_completion(&spi_data->ifx_read_write_completion);
//20100626-1, , throughput check [END]
#ifdef SPEED_CHECK
	//ulEnd = getuSecTime() - ulStart;
	do_gettimeofday(&ulEnd[id]);
	diff = (ulEnd[id].tv_sec - ulStart[id].tv_sec) * 1000 * 1000 ;
	diff = (diff + (ulEnd[id].tv_usec - ulStart[id].tv_usec));
	ulTxThroughtput[id] = ((uiTxlen[id]*8000)/diff);
	SPI_LOG("SPI%d : TX time = %09d usec; %04d bytes; %06lu Kbps", id, diff, 
		IFX_SPI_MAX_BUF_SIZE+IFX_SPI_HEADER_SIZE, ((IFX_SPI_MAX_BUF_SIZE+IFX_SPI_HEADER_SIZE)*8000)/diff);
	uiTxlen[id];
	fWrite[id] = 0;
#endif
//20100626-1, , throughput check [END]
	init_completion(&spi_data->ifx_read_write_completion);
	SPI_DEBUG_LOG("%d bytes sent to the device", spi_data->ifx_ret_count);
	return spi_data->ifx_ret_count; /* Number of bytes sent to the device */
}

/* This function should return number of free bytes left in the write buffer in this case always return 2048 */

static int 
ifx_spi_write_room(struct tty_struct *tty)
{	
	//printk( "[ SYBLUE.LEE %d]ifx_spi_write_room\n", uiStepCount++);
//LGE_TELECA_CR:707_DATA_THROUGHPUT START
	return IFX_SPI_MAX_BUF_SIZE;
//LGE_TELECA_CR:707_DATA_THROUGHPUT END
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
static int 
ifx_spi_probe(struct spi_device *spi)
{
	int status;
	struct ifx_spi_data *spi_data;
	static int index = 0;

	const NvOdmPeripheralConnectivity *pConnectivity = NULL;
	NvU32 MrdyPort;
	NvU32 MrdyPin;
	
	/* Allocate SPI driver data */
	spi_data = (struct ifx_spi_data*)kmalloc(sizeof(struct ifx_spi_data), GFP_KERNEL);
	if (!spi_data){
		return -ENOMEM;
	}

//LGE_TELECA_CR1317_DATA_THROUGHPUT START
	memset(spi_data, 0, sizeof(struct ifx_spi_data));
//LGE_TELECA_CR1317_DATA_THROUGHPUT END

//20100711-3, , init ifx_tty [START]
	spi_data->ifx_tty = NULL;
//20100711, , init ifx_tty [END]
	spi_data->wake_lock_flag = 0;

	dev_set_drvdata(&spi->dev,spi_data);
	spin_lock_init(&spi_data->spi_lock);

	CREATELOCK(spi_suspend_lock);
	
	INIT_WORK(&spi_data->ifx_work,ifx_spi_handle_work);
#ifdef LGE_DUAL_SPI_1
	if (index == 0) {
#endif
		SPI_LOG("workqueue name(%s)", dev_name(&spi->dev));
		spi_data->ifx_wq = create_singlethread_workqueue(dev_name(&spi->dev));
#ifdef LGE_DUAL_SPI_1
	}
	else {
		spi_data->ifx_wq = gspi_data[0]-> ifx_wq;
	}
#endif
	if(!spi_data->ifx_wq) {
		SPI_LOG("Failed to setup workqueue - ifx_wq");
	}
//#ifdef LGE_DUAL_SPI_1
//	 if (index == 0) 
//#endif
	init_completion(&spi_data->ifx_read_write_completion);

	/* Configure SPI */
	spi_data->spi = spi;
	spi->mode = SPI_MODE_1;
	spi->bits_per_word = 32;
	spi->chip_select = 0;
	spi->max_speed_hz = 30*1000*1000; //24Mhz	
        status = spi_setup(spi);

	if(status < 0){
		SPI_LOG("Failed to setup SPI");
	}

	gspi_data[index] = spi_data;
#ifdef LGE_DUAL_SPI_1
	if (index == 0) {
#endif
		status = ifx_spi_allocate_frame_memory(spi_data, IFX_SPI_MAX_BUF_SIZE + IFX_SPI_HEADER_SIZE);
		if(status != 0){
			SPI_LOG("Failed to allocate memory for buffers");
			return -ENOMEM;
		}

#ifdef WAKE_LOCK_RESUME
		wake_lock_init(&spi_data->wake_lock, WAKE_LOCK_SUSPEND, "mspi_wake");
#endif
#ifdef CONFIG_EARLY_SUSPEND_TEST
			spi_data->early_suspend.level = 40;
			spi_data->early_suspend.suspend = ifx_spi_early_suspend;
			spi_data->early_suspend.resume = ifx_spi_late_resume;
			register_early_suspend(&spi_data->early_suspend);
#endif

		pConnectivity = NvOdmPeripheralGetGuid(SPI_GUID);	   
		MrdyPort = pConnectivity->AddressList[0].Instance;
		MrdyPin = pConnectivity->AddressList[0].Address;

		spi_data->hGpio =  (NvOdmServicesGpioHandle)NvOdmGpioOpen();
		if(spi_data->hGpio==NULL){
			printk(KERN_ERR "Failed to get GPIO handle\n");
			return -ENODEV;
		}
		printk("GPIO handle = %x, MRDY = %d-%d\n", spi_data->hGpio, MrdyPort, MrdyPin);
		spi_data->hMrdyPin = NvOdmGpioAcquirePinHandle(spi_data->hGpio, MrdyPort, MrdyPin);
		if(spi_data->hMrdyPin==NULL){
			printk(KERN_ERR "Failed to get MRDY pin handle\n");
			return -ENODEV;
		}
		SPI_LOG("MRDY handle = %x", spi_data->hMrdyPin);
		NvOdmGpioSetState(spi_data->hGpio,spi_data->hMrdyPin,0);
		NvOdmGpioConfig(spi_data->hGpio, spi_data->hMrdyPin, NvOdmGpioPinMode_InputData);

	/* Enable MRDY Interrupt request - If the MRDY signal is high then ifx_spi_handle_mrdy_irq() is called */
		//status = request_irq(spi->irq, ifx_spi_handle_mrdy_irq,  IRQF_TRIGGER_RISING, spi->dev.driver->name, spi_data);
		status = NvOdmGpioInterruptRegister(spi_data->hGpio, &spi_data->hMrdyInterrupt,
					spi_data->hMrdyPin, NvOdmGpioPinMode_InputInterruptRisingEdge , ifx_spi_handle_mrdy_irq,
					(void*)spi_data, 0);
		if (status == NV_FALSE){
			printk(KERN_ERR "Failed to request IRQ for SRDY\n");
		printk(KERN_ERR "IFX SPI Probe Failed\n");
		if(spi_data->ifx_tx_buffer){
			kfree(spi_data->ifx_tx_buffer);
		}
		if(spi_data->ifx_rx_buffer){
			kfree(spi_data->ifx_rx_buffer);            
		}
		if(spi_data){
			kfree(spi_data);
		}          
		return status;
	}
	index ++;
		status = 0;
#ifdef LGE_DUAL_SPI_1
	 	}
#endif
	SPI_LOG("return status=%d", status);
	return status;
}

static int 
ifx_spi_remove(struct spi_device *spi)
{	
	struct ifx_spi_data *spi_data;

	
	//printk( "[ SYBLUE.LEE %d]ifx_spi_remove\n", uiStepCount++);

	spi_data = spi_get_drvdata(spi);
	spin_lock_irq(&spi_data->spi_lock);
	spi_data->spi = NULL;
	spi_set_drvdata(spi, NULL);
	spin_unlock_irq(&spi_data->spi_lock);

        if(spi_data->ifx_tx_buffer){
		kfree(spi_data->ifx_tx_buffer);
	}
        if(spi_data->ifx_rx_buffer){
		kfree(spi_data->ifx_rx_buffer);
	}
        if(spi_data){
		kfree(spi_data);
        }          
        return 0;
}

static void ifx_spi_shutdown(struct spi_device *spi)
{

	struct ifx_spi_data *spi_data;
	struct irq_desc *desc = irq_to_desc(spi->irq);

   
	ifx_shutdown = 1;

   spi_data= spi_get_drvdata(spi);


   if(spi_data->spi->irq)
   {
   	  disable_irq(spi_data->spi->irq);
	  printk("ifx_spi_shutdown: disable_irq\n");
   }


   if(spi_data->spi->irq && (desc!=NULL && desc->action))
   {
	free_irq(spi_data->spi->irq, spi_data);
	printk("ifx_spi_shutdown: free_irq\n");
   }

	printk("ifx_spi_shutdown: completed\n");

	
}
//20100908  deepsleep wakeup issue [START]
#include <mach/iomap.h>
#include <linux/io.h>

#define PMC_WAKE_STATUS 0x14
#define WAKEUP_IFX_SRDY_MASK    (1 << 7)     // Wake Event 0 - IFX_SRDY

static void __iomem *pmc_base = IO_ADDRESS(TEGRA_PMC_BASE);
static bool bSuspend = 0;


#ifdef CONFIG_EARLY_SUSPEND_TEST
static void 
ifx_spi_early_suspend(struct early_suspend *h)
{
#if 0
	unsigned long reg;

		printk("[EARLY_SUSPEND_SPI] bSuspend=%d\n", bSuspend);
	
	if(bSuspend)
		return 0;
	bSuspend = 1;

	reg = readl(pmc_base + PMC_WAKE_STATUS);

	// Clear power key wakeup pad bit.
	if (reg & WAKEUP_IFX_SRDY_MASK)
	{
			printk("[IFX_SRDY] wakeup pad : 0x%lx", reg);
		writel(WAKEUP_IFX_SRDY_MASK, pmc_base + PMC_WAKE_STATUS);
	}
#endif
	return;
}
static void 
ifx_spi_late_resume(struct early_suspend *h)
{
#if 0
	NvU32	pinValue;
		unsigned long reg;
	
		printk("[LATE_RESUME_SPI] bSuspend=%d\n", bSuspend);
		if(!bSuspend)
			return 0;
	
		bSuspend = 0;
	
		reg = readl(pmc_base + PMC_WAKE_STATUS);
	
		if (reg & WAKEUP_IFX_SRDY_MASK) 
		{
	//#ifdef	WAKE_LOCK_RESUME
	//		if(&gspi_data[0]->wake_lock)
	//			wake_lock_timeout(&gspi_data[0]->wake_lock, 50);	//20101203-1, , change 3 to 1 for power consumption
	//#endif		
			printk("[IFX_SRDY] wakeup pad : 0x%lx", reg);
		
//		writel(WAKEUP_IFX_SRDY_MASK, pmc_base + PMC_WAKE_STATUS); // clearing  wakeup bit 

#if 1//def	WAKE_LOCK_RESUME
		if(&gspi_data[0]->wake_lock)
			wake_lock_timeout(&gspi_data[0]->wake_lock, 5);	//20101203-1, , change 3 to 1 for power consumption
#endif		
	
#ifndef CONFIG_SPI_DEBUG
			lge_debug[D_SPI].enable = 1;
#endif
			gspi_data[0]->wake_lock_flag = 1;
			queue_work(gspi_data[0]->ifx_wq, &gspi_data[0]->ifx_work);
		}

#endif
	return;
}
#endif
#define EBS_TEST
static int ifx_spi_suspend(struct platform_device *pdev, pm_message_t state)
{
	printk("ifx_spi_suspend\n");
#if 1
	unsigned long reg;

#ifdef EBS_TEST
	unsigned long flags;
	spin_lock_irqsave(&spi_suspend_lock, flags);
	is_suspended = 1;
	spin_unlock_irqrestore(&spi_suspend_lock, flags);
#endif 
	printk("#####################\n");
	printk("[IFX_SPI SUSPEND] flag=%d is_suspend_flag = %d\n", bSuspend, is_suspended);

	if(bSuspend)
		return 0;
	bSuspend = 1;

	reg = readl(pmc_base + PMC_WAKE_STATUS);

	// Clear power key wakeup pad bit.
	if (reg & WAKEUP_IFX_SRDY_MASK)
	{
		SPI_LOG("[IFX_SPI SUSPEND]  wakeup pad : 0x%lx\n", reg);
		writel(WAKEUP_IFX_SRDY_MASK, pmc_base + PMC_WAKE_STATUS);
	}
	printk("#####################\n");
#endif
	return 0;
}

static int ifx_spi_resume(struct platform_device *pdev)
{
	
#if 1
	NvU32   pinValue;
	unsigned long reg;

#ifdef EBS_TEST
	unsigned long flags;
	spin_lock_irqsave(&spi_suspend_lock, flags);
	is_suspended = 0;
	spin_unlock_irqrestore(&spi_suspend_lock, flags);
#endif 
	printk("#####################\n");

	printk("[IFX_SPI RESUME] flag=%d is_suspend_flag = %d\n", bSuspend, is_suspended);

	if(!bSuspend)
		return 0;

	bSuspend = 0;

	reg = readl(pmc_base + PMC_WAKE_STATUS);

	if (reg & WAKEUP_IFX_SRDY_MASK)
	{
		//writel(WAKEUP_IFX_SRDY_MASK, pmc_base + PMC_WAKE_STATUS);
		printk("[IFX_SPI RESUME] wakeup pad : 0x%lx\n", reg);
#ifdef CONFIG_LPRINTK
		lge_debug[D_SPI].enable = 1;
#endif

#ifdef WAKE_LOCK_RESUME
		if(&gspi_data[0]->wake_lock)
			wake_lock_timeout(&gspi_data[0]->wake_lock, 5); //20101203-1, , change 3 to 1 for power consumption
#endif 

		gspi_data[0]->wake_lock_flag = 1;
		queue_work(gspi_data[0]->ifx_wq, &gspi_data[0]->ifx_work);
	}
#endif 
	printk("#####################\n");


	return 0;
}
//20100908  deepsleep wakeup issue [END]

/* End of TTY - SPI driver Operations */

/* ################################################################################################################ */

static struct spi_driver ifx_spi_driver = {
	.driver = {
		.name = "mdm6600",
                .bus = &spi_bus_type,
		.owner = THIS_MODULE,
	},
	.probe = ifx_spi_probe,
	.remove = __devexit_p(ifx_spi_remove),
	.shutdown = ifx_spi_shutdown, 
//20100908  deepsleep wakeup issue [START]
	.suspend = ifx_spi_suspend,
	.resume = ifx_spi_resume,
//20100908  deepsleep wakeup issue [END]
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
static void 
ifx_spi_buffer_initialization(struct ifx_spi_data *spi_data)
{
	//printk("[ SYBLUE.LEE %d]ifx_spi_buffer_initialization\n", uiStepCount++);

	spi_data->ifx_sender_buf_size = IFX_SPI_DEFAULT_BUF_SIZE;
       spi_data->ifx_receiver_buf_size = IFX_SPI_DEFAULT_BUF_SIZE;
}

/*
 * Allocate memeory for TX_BUFFER and RX_BUFFER
 */
static int 
ifx_spi_allocate_frame_memory(struct ifx_spi_data *spi_data, unsigned int memory_size)
{
	int status = 0;

	//printk( "[ SYBLUE.LEE %d]ifx_spi_allocate_frame_memory\n", uiStepCount++);

	spi_data->ifx_rx_buffer = kmalloc(memory_size+IFX_SPI_HEADER_SIZE, GFP_KERNEL);
	if (!spi_data->ifx_rx_buffer){
		SPI_LOG("Open Failed ENOMEM");
		status = -ENOMEM;
	}
	spi_data->ifx_tx_buffer = kmalloc(memory_size+IFX_SPI_HEADER_SIZE, GFP_KERNEL);
	if (!spi_data->ifx_tx_buffer){
		SPI_LOG("Open Failed ENOMEM");
		status = -ENOMEM;
	}

	if(status == -ENOMEM){
		if(spi_data->ifx_tx_buffer){
			kfree(spi_data->ifx_tx_buffer);
		}
		if(spi_data->ifx_rx_buffer){
			kfree(spi_data->ifx_rx_buffer);            
		}
	}
	return status;
}

/*
 * Function to set header information according to IFX SPI framing protocol specification
 */
//LGE_TELECA_CR1317_DATA_THROUGHPUT START
static void 
ifx_spi_set_header_info(unsigned char *header_buffer, unsigned int curr_buf_size, unsigned int next_buf_size, int more)
//LGE_TELECA_CR1317_DATA_THROUGHPUT END
{
	int i;
	union ifx_spi_frame_header header;

	//printk( "[ SYBLUE.LEE %d]ifx_spi_set_header_info\n", uiStepCount++);

	for(i=0; i<4; i++){
		header.framesbytes[i] = 0;
	}

	header.ifx_spi_header.curr_data_size = curr_buf_size;
//LGE_TELECA_CR1317_DATA_THROUGHPUT START
#ifdef SPI_SEND_MORE_ATTR
	if(next_buf_size || more){
#else
	if(next_buf_size){
#endif
//LGE_TELECA_CR1317_DATA_THROUGHPUT END
		header.ifx_spi_header.more=1;
		header.ifx_spi_header.next_data_size = next_buf_size;
	}
	else{
		header.ifx_spi_header.more=0;
		header.ifx_spi_header.next_data_size = 128;
	}

	for(i=3; i>=0; i--){
		header_buffer[i] = header.framesbytes[/*3-*/i];
	}
}

/*
 * Function to get header information according to IFX SPI framing protocol specification
 */
static int 
ifx_spi_get_header_info(struct ifx_spi_data *spi_data, unsigned int *valid_buf_size)
{
	int i;
	union ifx_spi_frame_header header;

	//printk( "[ SYBLUE.LEE %d]ifx_spi_get_header_info\n", uiStepCount++);

	for(i=0; i<4; i++){
		header.framesbytes[i] = 0;
	}

	for(i=3; i>=0; i--){
		header.framesbytes[i] = spi_data->ifx_rx_buffer[/*3-*/i];
	}

	if(header.ifx_spi_header.curr_data_size<=IFX_SPI_DEFAULT_BUF_SIZE)	
	{	//20100709-1, , check rx size
		*valid_buf_size = header.ifx_spi_header.curr_data_size;
	}
	else
	{
		*valid_buf_size = 0;
	}
	
	if(header.ifx_spi_header.more){
//LGE_TELECA_CR:1056_SPI/MUX_IMPROVEMENT START
//LGE_TELECA_CR1317_DATA_THROUGHPUT START
		spi_data->more_rx = 1;
//LGE_TELECA_CR1317_DATA_THROUGHPUT END
//LGE_TELECA_CR:1056_SPI/MUX_IMPROVEMENT END
		if(header.ifx_spi_header.next_data_size<=IFX_SPI_DEFAULT_BUF_SIZE)		//20100709-1, , check next rx size
			return header.ifx_spi_header.next_data_size;
	}
//LGE_TELECA_CR:1056_SPI/MUX_IMPROVEMENT START
	else {
//LGE_TELECA_CR1317_DATA_THROUGHPUT START
		spi_data->more_rx = 0;
//LGE_TELECA_CR1317_DATA_THROUGHPUT END
	}
//LGE_TELECA_CR:1056_SPI/MUX_IMPROVEMENT END

	return 0;
}

/*
 * Function to set/reset SRDY signal
 */
static void 
ifx_spi_set_srdy_signal(int value)
{
#ifndef USE_SRDY
	NvOdmGpioSetState(hGpio,hPin,(NvU32)value);
#endif
}

/*
 * Function to calculate next_frame_size required for filling in SPI frame Header
 */
static int 
ifx_spi_get_next_frame_size(int count)
{
	//printk( "[ SYBLUE.LEE %d]ifx_spi_get_next_frame_size\n", uiStepCount++);

	if(count > IFX_SPI_MAX_BUF_SIZE){
		return IFX_SPI_MAX_BUF_SIZE;    
	}
	else{   
		return count;
	}
}

/*
 * Function to setup transmission and reception. It implements a logic to find out the ifx_current_frame_size,
 * valid_frame_size and sender_next_frame_size to set in SPI header frame. Copys the data to be transferred from 
 * user space to TX buffer and set MRDY signal to HIGH to indicate Master is ready to transfer data.
 */
static void 
ifx_spi_setup_transmission(struct ifx_spi_data *spi_data)
{
	if( (spi_data->ifx_sender_buf_size > 0 && spi_data->ifx_sender_buf_size <= IFX_SPI_MAX_BUF_SIZE+IFX_SPI_HEADER_SIZE)
		|| (spi_data->ifx_receiver_buf_size > 0 && spi_data->ifx_receiver_buf_size <= IFX_SPI_MAX_BUF_SIZE+IFX_SPI_HEADER_SIZE) ){
		if(spi_data->ifx_sender_buf_size > spi_data->ifx_receiver_buf_size){
			spi_data->ifx_current_frame_size = spi_data->ifx_sender_buf_size;
		}
		else{ 
			spi_data->ifx_current_frame_size = spi_data->ifx_receiver_buf_size;    
		}
		if(spi_data->ifx_spi_count > 0 && spi_data->ifx_spi_count <= IFX_SPI_MAX_BUF_SIZE){
			if(spi_data->ifx_spi_count > spi_data->ifx_current_frame_size){
				spi_data->ifx_valid_frame_size = spi_data->ifx_current_frame_size;
				spi_data->ifx_spi_count = spi_data->ifx_spi_count - spi_data->ifx_current_frame_size;
			}
			else{
				spi_data->ifx_valid_frame_size = spi_data->ifx_spi_count;
				spi_data->ifx_spi_count = 0;
			}
                }
		else{
			spi_data->ifx_valid_frame_size = 0;
			spi_data->ifx_sender_buf_size = 0;
		}
		spi_data->ifx_sender_buf_size = ifx_spi_get_next_frame_size(spi_data->ifx_spi_count);

		/* memset buffers to 0 */
		if(spi_data->ifx_tx_buffer && spi_data->ifx_rx_buffer)
		{
//LGE_TELECA_CR:707_DATA_THROUGHPUT START
			memset(spi_data->ifx_tx_buffer,0,IFX_SPI_HEADER_SIZE);
			memset(spi_data->ifx_rx_buffer,0,IFX_SPI_HEADER_SIZE);
//LGE_TELECA_CR:707_DATA_THROUGHPUT END

			/* Set header information */
//LGE_TELECA_CR1317_DATA_THROUGHPUT START
			ifx_spi_set_header_info(spi_data->ifx_tx_buffer, spi_data->ifx_valid_frame_size, spi_data->ifx_sender_buf_size, spi_data->more_tx);
//LGE_TELECA_CR1317_DATA_THROUGHPUT END
			if( spi_data->ifx_valid_frame_size > 0 )
			{      
				memcpy(spi_data->ifx_tx_buffer+IFX_SPI_HEADER_SIZE, spi_data->ifx_spi_buf, spi_data->ifx_valid_frame_size);
				spi_data->ifx_spi_buf = spi_data->ifx_spi_buf + spi_data->ifx_valid_frame_size;
			}
		}
		else
		{
			SPI_LOG("TX or RX buffer is NULL");
		}
	}
}

/*
 * Function starts Read and write operation and transfers received data to TTY core. It pulls down MRDY signal
 * in case of single frame transfer then sets "ifx_read_write_completion" to indicate transfer complete.
 */
static void 
ifx_spi_send_and_receive_data(struct ifx_spi_data *spi_data)
{
	unsigned int rx_valid_buf_size;
	int status = 0; 

	if (ifx_shutdown) return;
	SPI_DEBUG_PRINT("%s %d: " "TX(%04lu) : ", __FUNCTION__, __LINE__, (((*(unsigned int *)spi_data->ifx_tx_buffer)&0xFFF) + IFX_SPI_HEADER_SIZE));
//LGE_TELECA_CR1317_DATA_THROUGHPUT START
#ifdef LGE_SPI_DEBUG_LOG
	DUMP_SPI_BUFFER(spi_data->ifx_tx_buffer, ((*(unsigned int *)spi_data->ifx_tx_buffer)&0xFFF) + IFX_SPI_HEADER_SIZE);
#endif
//LGE_TELECA_CR1317_DATA_THROUGHPUT END

	status = ifx_spi_sync_read_write(spi_data, spi_data->ifx_current_frame_size+IFX_SPI_HEADER_SIZE); /* 4 bytes for header */                         
	if(status > 0){
//LGE_TELECA_CR:707_DATA_THROUGHPUT START
		memset(spi_data->ifx_tx_buffer,0,IFX_SPI_HEADER_SIZE);
//LGE_TELECA_CR:707_DATA_THROUGHPUT END
		spi_data->ifx_ret_count = spi_data->ifx_ret_count + spi_data->ifx_valid_frame_size;
	}

	// hgahn
	if(memcmp(rx_dummy, spi_data->ifx_rx_buffer, IFX_SPI_HEADER_SIZE) ==0) {
		spi_data->ifx_receiver_buf_size = 0;
		return;
	}

	/* Handling Received data */
	spi_data->ifx_receiver_buf_size = ifx_spi_get_header_info(spi_data, &rx_valid_buf_size);
	SPI_DEBUG_PRINT("%s %d: " "RX(%04lu) : ", __FUNCTION__, __LINE__, (((*(unsigned int *)spi_data->ifx_rx_buffer)&0xFFF) + IFX_SPI_HEADER_SIZE));
//LGE_TELECA_CR1317_DATA_THROUGHPUT START
#ifdef LGE_SPI_DEBUG_LOG
	DUMP_SPI_BUFFER(spi_data->ifx_rx_buffer, rx_valid_buf_size+IFX_SPI_HEADER_SIZE);
#endif
//LGE_TELECA_CR1317_DATA_THROUGHPUT END

	if((spi_data->throttle == 0) && (rx_valid_buf_size != 0) && (spi_data->ifx_tty!=NULL))	//20100711-3, , check ifx_tty
	{
//20100626-1, , throughput check [START]
#ifdef SPEED_CHECK
		uiRxlen[spi_data->ifx_tty->index] = rx_valid_buf_size+IFX_SPI_HEADER_SIZE;
#endif
//20100626-1, , throughput check [END]
		tty_insert_flip_string(spi_data->ifx_tty, spi_data->ifx_rx_buffer+IFX_SPI_HEADER_SIZE, rx_valid_buf_size);
		tty_flip_buffer_push(spi_data->ifx_tty);
	}  
	/*else
  	{ 
	handle RTS and CTS in SPI flow control
	Reject the packet as of now 
	}*/
#ifdef CONFIG_LPRINTK
	if(spi_data->wake_lock_flag)
	{
		spi_data->wake_lock_flag = 0;
		lge_debug[D_SPI].enable = 0;
	}
#endif	
}


#ifdef LGE_DUAL_SPI_1
static void spi_complete(void *arg)
{
	complete(arg);
}
#endif
/*
 * Function copies the TX_BUFFER and RX_BUFFER pointer to a spi_transfer structure and add it to SPI tasks.
 * And calls SPI Driver function "spi_sync" to start data transmission and reception to from MODEM
 */
static unsigned int 
ifx_spi_sync_read_write(struct ifx_spi_data *spi_data, unsigned int len)

#ifdef LGE_DUAL_SPI_1
{
	/* define the index of second spi */
	int status;
	int i, sec_spi_num = 0;
	int rwstat[2] = {0, 0};
	/* define the half-length buffer */
	unsigned int hf_buffer = IFX_SPI_MAX_BUF_SIZE + IFX_SPI_HEADER_SIZE;
	if (ifx_shutdown) return 0;

	SPI_DEBUG_LOG("spi%d", spi_data->spi->master->bus_num);
	for (i = 0; i < MAX_USED_SPI_DREVICE; i++) {
		if (gspi_data[i]) {
			if (gspi_data[i]->spi->master->bus_num != spi_data->spi->master->bus_num) {
				sec_spi_num = i;
			}
		}
	}

	if ((hf_buffer % 2) == 0) {
		hf_buffer /= 2; 
		struct spi_message	m, m1;
		struct spi_transfer	t = {
						.tx_buf		= spi_data->ifx_tx_buffer,
                        			.rx_buf		= spi_data->ifx_rx_buffer,
						.len		= hf_buffer,
					    };
		struct spi_transfer	t1 = {
						.tx_buf		= spi_data->ifx_tx_buffer + hf_buffer,
	                        		.rx_buf		= spi_data->ifx_rx_buffer + hf_buffer,
						.len		= hf_buffer,
					    };

		if (!spi_data->ifx_master_initiated_transfer)
			//came from mdm6600 master
//LGE_TELECA_CR:707_DATA_THROUGHPUT START
			memset(spi_data->ifx_tx_buffer,0,IFX_SPI_HEADER_SIZE);
//LGE_TELECA_CR:707_DATA_THROUGHPUT END

		spi_message_init(&m);
		spi_message_init(&m1);
		spi_message_add_tail(&t, &m);
		spi_message_add_tail(&t1, &m1);
	
		if (spi_data->spi == NULL )
			rwstat[0] = -ESHUTDOWN;
		else {
			DECLARE_COMPLETION_ONSTACK(done);
			m.complete = spi_complete;
			m.context = &done;
			rwstat[0] = spi_async(spi_data->spi, &m);
			
			//SPI_DEBUG_LOG("spi%d", rwstat[0]);
			rwstat[1] = spi_sync(gspi_data[sec_spi_num]->spi, &m1);
			//SPI_DEBUG_LOG("spi%d", rwstat[1]);
			if (rwstat[0] == 0) {
				wait_for_completion(&done);
				rwstat[0] = m.status;
				}
			m.context = NULL;
			//SPI_DEBUG_LOG("spi%d", rwstat[0]);
		}
	
		if ((rwstat[0] == 0) && (rwstat[1] == 0)){
			rwstat[0] = m.status;
			//SPI_DEBUG_LOG("spi%d", rwstat[0]);
			rwstat[1] = m1.status;
			//SPI_DEBUG_LOG("spi%d", rwstat[1]);
			if ((rwstat[0] == 0) && (rwstat[1] == 0)) {
				rwstat[0] = m.actual_length;
				//SPI_DEBUG_LOG("spi%d", rwstat[0]);
				rwstat[1] = m1.actual_length;
				//SPI_DEBUG_LOG("spi%d", rwstat[1]);
			}
		}
		else{
			SPI_LOG("Transmission unsuccessful");
		}
	}
	else {
		SPI_LOG("Failed to devide buffer: incorrect size");
		return -EINVAL;
		}
		
	if (rwstat[0] < 0)
		status = rwstat[0];
	else if (rwstat[1] < 0)
		status = rwstat[1];
	else
		status = rwstat[0] + rwstat[1];

	return status;
 	}
#else
	
#endif
/*
 * Function is a Interrupt service routine, is called when MRDY signal goes HIGH. It set up srdy pin when ready and
 * reception if it is a Master initiated data transfer. For both the cases Master intiated/Slave intiated
 * transfer it starts data transfer. 
 */
//static irqreturn_t ifx_spi_handle_mrdy_irq(int irq, void *handle)
static void ifx_spi_handle_mrdy_irq(void *handle)
{
	struct ifx_spi_data *spi_data = (struct ifx_spi_data *)handle;
	if (ifx_shutdown) return ;
	if(spi_data && spi_data->ifx_tty)	//20101104-1, , prevent MRDY until spi openning
	{	//20101104-1, , prevent MRDY until spi openning
// EBS LGE_UPDATE_S  - 20110610 
#if 1//def CONFIG_HAS_WAKELOCK
    	wake_lock_timeout(&gspi_data[0]->wake_lock, 5);	
#endif
		queue_work(spi_data->ifx_wq, &spi_data->ifx_work);
// EBS LGE_UPDATE_E  - 20110610  
	}
	else
	{
		SPI_LOG("Unexpected interrupt happen!");
	}
	NvOdmGpioInterruptDone(spi_data->hMrdyInterrupt);
	//return IRQ_HANDLED; 
}

static void 
ifx_spi_handle_work(struct work_struct *work)
{
	struct ifx_spi_data *spi_data = container_of(work, struct ifx_spi_data, ifx_work);
	int id = 0;	//20101104-2, , ignore this value
	unsigned long diff;
	unsigned long reg;
	int pm_off_count;
	if (ifx_shutdown) return;

#ifdef EBS_TEST
	pm_off_count = 0;

   //need to wait transferring tx/rx data because ap is in a suspended state
   if(1 == is_suspended)
   {
	   pm_off_count = 1;
	   printk("[EBS] mdm_spi_handle_work INFO is_suspended is (0x%x)\n", is_suspended);

	   //wait for ap to return to resume state with a worst case scenario of 5sec
	   do
	   {		   
		   mdelay(1);
		   pm_off_count++;
		   
	   }while((1 == is_suspended) && (pm_off_count<(5*200)));

	   printk("[EBS] mdm_spi_handle_work INFO EXIT is_suspend = 0x%x pm_off_count=%d\n", is_suspended, pm_off_count);

	   if(1 == is_suspended)
	   {
		  // To Do how to handle the PM OFF state during 1sec
		  printk("[EBS] mdm_spi_handle_work error is_suspended is (0x%x)\n",is_suspended);
	   }
   }
#endif

	
//LGE_TELECA_CR:1056_SPI/MUX_IMPROVEMENT START
#ifdef SPI_SEND_MORE_ATTR
	do {
#endif
//LGE_TELECA_CR:1056_SPI/MUX_IMPROVEMENT END
#if 0//def	WAKE_LOCK_RESUME
		if(&gspi_data[0]->wake_lock)
		{
			wake_lock_timeout(&gspi_data[0]->wake_lock, 5);	//20101203-1, , change 3 to 1 for power consumption
		}
#endif		

	if (!spi_data->ifx_master_initiated_transfer){
	//SPI_DEBUG_LOG("[id: %d]", id);
//20100626-1, , throughput check [START]
#ifdef SPEED_CHECK
		do_gettimeofday(&ulStart[id]);
#endif
//20100626-1, , throughput check [END]
		ifx_spi_setup_transmission(spi_data);
		ifx_spi_set_srdy_signal(1);
		ifx_spi_send_and_receive_data(spi_data);
//20100626-1, , throughput check [END]
#ifdef SPEED_CHECK
		do_gettimeofday(&ulEnd[id]);
		diff = (ulEnd[id].tv_sec - ulStart[id].tv_sec) * 1000 * 1000 ;
		diff = (diff + (ulEnd[id].tv_usec - ulStart[id].tv_usec));
		ulRxThroughtput[id] = ((uiRxlen[id]*8000)/diff);
		SPI_LOG("SPI%d : RX time = %09d usec; %04d bytes; %06lu Kbps", id, diff, 
			IFX_SPI_MAX_BUF_SIZE+IFX_SPI_HEADER_SIZE, ((IFX_SPI_MAX_BUF_SIZE+IFX_SPI_HEADER_SIZE)*8000)/diff);
#endif
//20100626-1, , throughput check [END]
		//SPI_DEBUG_LOG("[id: %d]", id);
		/* Once data transmission is completed, the MRDY signal is lowered */
		if((spi_data->ifx_sender_buf_size == 0)  && (spi_data->ifx_receiver_buf_size == 0)){
			ifx_spi_set_srdy_signal(0);
			ifx_spi_buffer_initialization(spi_data);
			//SPI_DEBUG_LOG("[id: %d]", id);
		}

		/* We are processing the slave initiated transfer in the mean time Mux has requested master initiated data transfer */
		/* Once Slave initiated transfer is complete then start Master initiated transfer */
		if(spi_data->ifx_master_initiated_transfer == 1){
		/* It is a condition where Slave has initiated data transfer and both SRDY and MRDY are high and at the end of data transfer		
	 	* MUX has some data to transfer. MUX initiates Master initiated transfer rising MRDY high, which will not be detected at Slave-MODEM.
	 	* So it was required to rise MRDY high again */

		}
	}
	else{
		//SPI_DEBUG_LOG("[id: %d]", id);
		ifx_spi_setup_transmission(spi_data);     
		//	spi_data->ifx_tx_buffer[10], spi_data->ifx_tx_buffer[11], spi_data->ifx_tx_buffer[12], spi_data->ifx_tx_buffer[13] );
		ifx_spi_send_and_receive_data(spi_data);
		/* Once data transmission is completed, the MRDY signal is lowered */
		if(spi_data->ifx_sender_buf_size == 0){
			if(spi_data->ifx_receiver_buf_size == 0){		
				ifx_spi_set_srdy_signal(0);
//20100701-1, , delay time until CP can be ready again [START]
//				udelay(MRDY_DELAY_TIME);
//20100701-1, , delay time until CP can be ready again [END]
				ifx_spi_buffer_initialization(spi_data);
				//SPI_DEBUG_LOG("[id: %d]", id);
			}
			spi_data->ifx_master_initiated_transfer = 0;
			complete(&spi_data->ifx_read_write_completion);
		}
	}

//20100626-1, , throughput check [END]
#if	0	//def SPEED_CHECK
	if(uiTxlen[spi_data->ifx_tty->index] || uiRxlen[spi_data->ifx_tty->index]) {
		//ulEnd = getuSecTime() - ulStart;
		do_gettimeofday(&ulEnd[spi_data->ifx_tty->index]);
		uidiff[spi_data->ifx_tty->index] = (ulEnd[spi_data->ifx_tty->index].tv_sec - ulStart[spi_data->ifx_tty->index].tv_sec) * 1000 * 1000 ;
		uidiff[spi_data->ifx_tty->index] = uidiff[spi_data->ifx_tty->index] + (ulEnd[spi_data->ifx_tty->index].tv_usec - ulStart[spi_data->ifx_tty->index].tv_usec);
		ulTxThroughtput[spi_data->ifx_tty->index] = ((uiTxlen[spi_data->ifx_tty->index]*8000)/uidiff[spi_data->ifx_tty->index]);
		ulRxThroughtput[spi_data->ifx_tty->index] = ((uiRxlen[spi_data->ifx_tty->index]*8000)/uidiff[spi_data->ifx_tty->index]);
		printk("[SPI%d]time  = %d us, Tx(%dbytes) = %luKbps, Rx(%dbytes) = %luKbps, Max(%dbytes) = %luKbps\n", spi_data->ifx_tty->index, uidiff[spi_data->ifx_tty->index], 
			uiTxlen[spi_data->ifx_tty->index], ulTxThroughtput[spi_data->ifx_tty->index], 
			uiRxlen[spi_data->ifx_tty->index], ulRxThroughtput[spi_data->ifx_tty->index],
			IFX_SPI_MAX_BUF_SIZE+IFX_SPI_HEADER_SIZE, ((IFX_SPI_MAX_BUF_SIZE+IFX_SPI_HEADER_SIZE)*8000)/uidiff[spi_data->ifx_tty->index]);
		uiTxlen[spi_data->ifx_tty->index] = uiRxlen[spi_data->ifx_tty->index] = 0;
		fWrite[spi_data->ifx_tty->index] = 0;
	}
#endif
//20100626-1, , throughput check [END]

//LGE_TELECA_CR:1056_SPI/MUX_IMPROVEMENT START
#ifdef SPI_SEND_MORE_ATTR
//LGE_TELECA_CR1317_DATA_THROUGHPUT START
	} while (spi_data->more_rx);
//LGE_TELECA_CR1317_DATA_THROUGHPUT END
#endif
//LGE_TELECA_CR:1056_SPI/MUX_IMPROVEMENT END
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
static int 
__init ifx_spi_init(void)
{
	int status = 0;

	printk("ifx_spi_init\n");

	/* Allocate and Register a TTY device */
	ifx_spi_tty_driver = alloc_tty_driver(MAX_USED_SPI_DREVICE);
	if (!ifx_spi_tty_driver){
		printk(KERN_ERR "Fail to allocate TTY Driver\n");
		return -ENOMEM;
	}
	SPI_LOG("alloc_tty_driver OK");

	/* initialize the tty driver */
	ifx_spi_tty_driver->owner = THIS_MODULE;
	ifx_spi_tty_driver->driver_name = "tty_ifxn721";	//20100607-1, , modify ifxn721 -> tty_ifxn721
	ifx_spi_tty_driver->name = "ttyspi";
	ifx_spi_tty_driver->major = IFX_SPI_MAJOR;
	ifx_spi_tty_driver->minor_start = 0;
	ifx_spi_tty_driver->type = TTY_DRIVER_TYPE_SERIAL;
	ifx_spi_tty_driver->subtype = SERIAL_TYPE_NORMAL;
	ifx_spi_tty_driver->flags = TTY_DRIVER_REAL_RAW;
	ifx_spi_tty_driver->init_termios = tty_std_termios;
	ifx_spi_tty_driver->init_termios.c_cflag = B230400 | CS8 | CREAD | HUPCL | CLOCAL;
	tty_set_operations(ifx_spi_tty_driver, &ifx_spi_ops);
	SPI_LOG("tty_set_operations OK");

	status = tty_register_driver(ifx_spi_tty_driver);
	if (status){
		printk(KERN_ERR "Failed to register IFX SPI tty driver");
		put_tty_driver(ifx_spi_tty_driver);
		return status;
	}
	SPI_LOG("tty_register_driver OK");
	
	/* Register SPI Driver */
	status = spi_register_driver(&ifx_spi_driver);
	if (status < 0){ 
		printk(KERN_ERR "Failed to register SPI device");
		tty_unregister_driver(ifx_spi_tty_driver);
		put_tty_driver(ifx_spi_tty_driver);
		return status;
	}
	SPI_LOG("spi_register_driver OK");
	SPI_LOG("return status=%d", status);
	return status;
}

module_init(ifx_spi_init);


/*
 * Exit function to unregister SPI driver and tty SPI driver
 */
static void 
__exit ifx_spi_exit(void)
{  
	printk("ifx_spi_exit\n");

	spi_unregister_driver(&ifx_spi_driver);
	tty_unregister_driver(ifx_spi_tty_driver);
        put_tty_driver(ifx_spi_tty_driver);
}

module_exit(ifx_spi_exit);

/* End of Initialization Functions */

/* ################################################################################################################ */

MODULE_AUTHOR("Sangyun Lee <>");
MODULE_DESCRIPTION("MDM6600 SPI Framing Layer");
MODULE_LICENSE("GPL");
