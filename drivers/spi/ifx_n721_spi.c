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
#define NV_DEBUG 0
//
#include "nvos.h"
#include "nvcommon.h"
#include "nvodm_services.h"
#include "nvodm_query_discovery.h"
#include <linux/mutex.h>


#include <linux/spi/ifx_n721_spi.h>

#include <linux/io.h>
#include <mach/iomap.h>

//20100927-1, , Hold wake-lock for cp interrupt [START]
#define WAKE_LOCK_RESUME
#ifdef WAKE_LOCK_RESUME
#include <linux/wakelock.h>
#endif
//20100927, , Hold wake-lock for cp interrupt [END]

//20100701-1, , delay time until CP can be ready again [START]
#include <linux/delay.h>
#define MRDY_DELAY_TIME	400	//20101127-1, , Change delay time for transcation : 1000us -> 400us
//20100701-1, , delay time until CP can be ready again [END]

#define SPI_GUID                NV_ODM_GUID('s','t','a','r','-','s','p','i')	//20100607, , add spi guid


//#define CONFIG_SPI_DEBUG
#ifdef CONFIG_SPI_DEBUG
#define SPI_DEBUG_PRINT(format, args...)  printk(D_SPI,format , ## args)
void dump_atcmd(char *data, int len) 
{
	int j ;
	
	if(len>20)
		len = 20;
	
	for(j=0;j<len;j++)
	{
		if(data[j]>=32 && data[j]<=126)
			printk("%c,",data[j]);	
		else
			printk("%x,",data[j]);	
	}
	printk("\n");
}

#else
#if CONFIG_LPRINTK
#include <mach/lprintk.h> //20100426, , Change printk to lprintk
#define SPI_DEBUG_PRINT(format, args...)  lprintk(D_SPI,format , ## args)
void dump_atcmd(char *data, int len) 
{
	int j ;

	if(!lge_debug[D_SPI].enable)
		return ;

	if(len>20)
		len = 20;

	for(j=0;j<len;j++)
	{
		if(data[j]>=32 && data[j]<=126)
			printk("%c,",data[j]);	
		else
			printk("%x,",data[j]);	
	}
	printk("\n");
}
#else
#define SPI_DEBUG_PRINT(format, args...)  {}
void dump_atcmd(char *data, int len)  {}
#endif
#endif

/* Cannot use spinlocks as the NvRm SPI apis uses mutextes and one cannot use
 * mutextes inside a spinlock.
 */
#define USE_SPINLOCK 0
#if USE_SPINLOCK
#define LOCK_T          spinlock_t
#define CREATELOCK(_l)  spin_lock_init(&(_l))
#define DELETELOCK(_l)
#define LOCK(_l)        spin_lock(&(_l))
#define UNLOCK(_l)      spin_unlock(&(_l))
#define ATOMIC(_l,_f)   spin_lock_irqsave(&(_l),(_f))
#define UNATOMIC(_l,_f) spin_unlock_irqrestore(&(_l),(_f))
#else
#define LOCK_T          struct mutex
#define CREATELOCK(_l)  mutex_init(&(_l))
#define DELETELOCK(_l) 
#define LOCK(_l)        mutex_lock(&(_l))
#define UNLOCK(_l)      mutex_unlock(&(_l))
#define ATOMIC(_l,_f)   local_irq_save((_f))
#define UNATOMIC(_l,_f) local_irq_restore((_f))
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
        struct work_queue_struct *ifx_wq;
//20100927-1, , Hold wake-lock for cp interrupt [START]
#ifdef WAKE_LOCK_RESUME
	struct wake_lock wake_lock;
	unsigned int		wake_lock_flag;
	unsigned int            srdy_high_counter;
#endif
//20100927, , Hold wake-lock for cp interrupt [END]
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

static NvOdmServicesGpioHandle hGpio = NULL;
static NvOdmGpioPinHandle hPin = NULL; 
static NvOdmGpioPinHandle hSrdyPin = NULL; 
static NvOdmServicesGpioIntrHandle hSrdyInterrupt =  NULL;

struct ifx_spi_data	*gspi_data;
struct tty_driver 	*ifx_spi_tty_driver;

/* ################################################################################################################ */
/* Global Declarations */
unsigned long		minors[IFX_N_SPI_MINORS / BITS_PER_LONG];
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

/* Function Declarations */
static void ifx_spi_set_header_info(unsigned char *header_buffer, unsigned int curr_buf_size, unsigned int next_buf_size);
static int ifx_spi_get_header_info(unsigned int *valid_buf_size);
static void ifx_spi_set_mrdy_signal(int value);
static void ifx_spi_setup_transmission(void);
static void ifx_spi_send_and_receive_data(struct ifx_spi_data *spi_data);
static int ifx_spi_get_next_frame_size(int count);
static int ifx_spi_allocate_frame_memory(unsigned int memory_size);
static void ifx_spi_buffer_initialization(void);
static unsigned int ifx_spi_sync_read_write(struct ifx_spi_data *spi_data, unsigned int len);
static irqreturn_t ifx_spi_handle_srdy_irq(int irq, void *handle);
static void ifx_spi_handle_work(struct work_struct *work);


// hgahn
unsigned char rx_dummy[]={0xff,0xff,0xff,0xff};

//#define SPI_DRIVER_TEST

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
//20100607, , add else code [START]
	struct ifx_spi_data *spi_data;
	if(gspi_data)
	{
		spi_data = gspi_data;
		spi_data->ifx_tty = tty;
		tty->driver_data = spi_data;
		ifx_spi_buffer_initialization();
		spi_data->throttle = 0;
		SPI_DEBUG_PRINT("ifx_spi_open\n");
	}
	else
	{
		ifx_spi_buffer_initialization();
		SPI_DEBUG_PRINT("ifx_spi_open failed!!\n");
	}
//20100607, , add else code [END]

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
	unsigned int u32register = 0 ;
	unsigned int value  = 0;
	struct ifx_spi_data *spi_data = (struct ifx_spi_data *)tty->driver_data;

	if(spi_data==NULL)
	{
		printk("ifx_spi_write failed : spi_data is null\n");	//syblue.lee 100604
		return 0;
	}

        ifx_ret_count = 0;
	spi_data->ifx_tty = tty;
	spi_data->ifx_tty->low_latency = 1;
	if( !buf ){
		printk("File: ifx_n721_spi.c\tFunction: int ifx_spi_write()\t Buffer NULL\n");
		return ifx_ret_count;
	}
	if(!count){
		printk("File: ifx_n721_spi.c\tFunction: int ifx_spi_write()\t Count is ZERO\n");
		return ifx_ret_count;
	}
	ifx_master_initiated_transfer = 1;
	ifx_spi_buf = buf;
	ifx_spi_count = count;
	ifx_spi_set_mrdy_signal(1);  
	wait_for_completion_timeout(&spi_data->ifx_read_write_completion, 2*HZ);	
	if(ifx_ret_count==0)
	{
		ifx_spi_set_mrdy_signal(0);	
		u32register = readl(IO_ADDRESS(0x6000d1b8));
		//lge_debug[D_SPI].enable = 1;
		printk("%s -u32register = %08X, SRDY = %d\n", __FUNCTION__, u32register, ((u32register>>5)&0x00000001)); 
	}
	init_completion(&spi_data->ifx_read_write_completion);

	SPI_DEBUG_PRINT("%s - %d\n", __FUNCTION__, __LINE__); 

	return ifx_ret_count; /* Number of bytes sent to the device */
}

/* This function should return number of free bytes left in the write buffer in this case always return 2048 */

static int 
ifx_spi_write_room(struct tty_struct *tty)
{	
	return 2048;
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
	const NvOdmPeripheralConnectivity *pConnectivity = NULL;
	NvU32 MrdyPort;
	NvU32 MrdyPin;

	SPI_DEBUG_PRINT("ifx_spi_probe\n");

	/* Allocate SPI driver data */
	spi_data = (struct ifx_spi_data*)kmalloc(sizeof(struct ifx_spi_data), GFP_KERNEL);
	if (!spi_data){
		return -ENOMEM;
        }

//20100711-3, , init ifx_tty [START]
	spi_data->ifx_tty = NULL;
//20100711, , init ifx_tty [END]

        status = ifx_spi_allocate_frame_memory(IFX_SPI_MAX_BUF_SIZE + IFX_SPI_HEADER_SIZE);
        if(status != 0){
		printk("File: ifx_n721_spi.c\tFunction: int ifx_spi_probe\tFailed to allocate memory for buffers\n");
		return -ENOMEM;
        }
	
        dev_set_drvdata(&spi->dev,spi_data);
        spin_lock_init(&spi_data->spi_lock);
        INIT_WORK(&spi_data->ifx_work,ifx_spi_handle_work);

        spi_data->ifx_wq = create_singlethread_workqueue("ifxn721");
        if(!spi_data->ifx_wq){
		printk("Failed to setup workqueue - ifx_wq \n");          
        }

	init_completion(&spi_data->ifx_read_write_completion);

        /* Configure SPI */
        spi_data->spi = spi;
        spi->mode = SPI_MODE_1;
        spi->bits_per_word = 8;
//20100607, , add more setup code [START]	
        spi->chip_select = 0;
        spi->max_speed_hz = 24*1000*1000; //24Mhz
//20100607, , add more setup code [END] 
        status = spi_setup(spi);
        if(status < 0){
		printk("Failed to setup SPI \n");
        }             

//20100927-1, , Hold wake-lock for cp interrupt [START]
#ifdef WAKE_LOCK_RESUME
	wake_lock_init(&spi_data->wake_lock, WAKE_LOCK_SUSPEND, "mspi_wake");
	spi_data->wake_lock_flag = 0;
	spi_data->srdy_high_counter = 0;
#endif
//20100927-1, , Hold wake-lock for cp interrupt [END]

//20100607, , add MRDY pin setup[START] 
	pConnectivity = NvOdmPeripheralGetGuid(SPI_GUID);      
	MrdyPort = pConnectivity->AddressList[1].Instance;
	MrdyPin = pConnectivity->AddressList[1].Address;
	hGpio =  (NvOdmServicesGpioHandle)NvOdmGpioOpen();
	if(hGpio==NULL){
		printk(KERN_ERR "Failed to get GPIO handle\n");
		return -ENODEV;
	}
	SPI_DEBUG_PRINT("GPIO handle = %x, MRDY = %d-%d\n", (int)hGpio, MrdyPort, MrdyPin);
	hPin = NvOdmGpioAcquirePinHandle(hGpio, MrdyPort, MrdyPin);
	if(hPin==NULL){
		printk(KERN_ERR "Failed to get MRDY pin handle\n");
		return -ENODEV;
	}
	SPI_DEBUG_PRINT("MRDY handle = %x\n", (int)hPin);
	NvOdmGpioConfig(hGpio, hPin, NvOdmGpioPinMode_Output);
//20100607, , add MRDY pin setup[END] 

	status = request_irq(spi->irq, ifx_spi_handle_srdy_irq,  IRQF_TRIGGER_RISING, spi->dev.driver->name, spi_data);
	if (status != 0){
		printk(KERN_ERR "Failed to request IRQ for SRDY\n");
		printk(KERN_ERR "IFX SPI Probe Failed\n");
		if(ifx_tx_buffer){
			kfree(ifx_tx_buffer);
		}
		if(ifx_rx_buffer){
			kfree(ifx_rx_buffer);            
		}
		if(spi_data){
			kfree(spi_data);
		}          
	}
	else{
		gspi_data = spi_data;
		SPI_DEBUG_PRINT("ifx_spi_probe ; spi_data is %x \n", (int)spi_data);
	}
	return status;
}

static int 
ifx_spi_remove(struct spi_device *spi)
{	
	struct ifx_spi_data *spi_data;
	spi_data = spi_get_drvdata(spi);
	spin_lock_irq(&spi_data->spi_lock);
	spi_data->spi = NULL;
	spi_set_drvdata(spi, NULL);
	spin_unlock_irq(&spi_data->spi_lock);

        if(ifx_tx_buffer){
		kfree(ifx_tx_buffer);
	}
        if(ifx_rx_buffer){
		kfree(ifx_rx_buffer);
	}
        if(spi_data){
		kfree(spi_data);
        }
	NvOdmGpioReleasePinHandle(hGpio, hPin);
	NvOdmGpioClose(hGpio);
        return 0;
}


//20100908 deepsleep wakeup issue [START]
#include <mach/iomap.h>
#include <linux/io.h>

#define PMC_WAKE_STATUS 0x14
#define WAKEUP_IFX_SRDY_MASK    (1 << 0)     // Wake Event 0 - IFX_SRDY

static void __iomem *pmc_base = IO_ADDRESS(TEGRA_PMC_BASE);

static int ifx_spi_suspend(struct platform_device *dev, pm_message_t state)
{
    unsigned long reg;

    reg = readl(pmc_base + PMC_WAKE_STATUS);

    // Clear power key wakeup pad bit.
    if (reg & WAKEUP_IFX_SRDY_MASK)
    {
        //printk("[IFX_SRDY] %s() wakeup pad : 0x%lx\n", __func__, reg);
        writel(WAKEUP_IFX_SRDY_MASK, pmc_base + PMC_WAKE_STATUS);
    }

    return 0;
}

static int ifx_spi_resume(struct platform_device *dev)
{
    NvU32   pinValue;
    unsigned long reg;

    reg = readl(pmc_base + PMC_WAKE_STATUS);

    if (reg & WAKEUP_IFX_SRDY_MASK) {
//20100927-1, , Hold wake-lock for cp interrupt [START]
#ifdef	WAKE_LOCK_RESUME
	//printk("[IFX_SRDY] %s() wake lock : 0x%lx\n", __func__, &gspi_data->wake_lock);
	if(&gspi_data->wake_lock)
		wake_lock_timeout(&gspi_data->wake_lock, 50);	//20101203-1, , change 3 to 1 for power consumption
#endif
//20100927-1, , Hold wake-lock for cp interrupt [END]
        printk("[IFX_SRDY] %s() wakeup pad : 0x%lx\n", __func__, reg);
#ifdef CONFIG_LPRINTK
	 lge_debug[D_SPI].enable = 1;
#endif
	 gspi_data->wake_lock_flag = 1;
        queue_work(gspi_data->ifx_wq, &gspi_data->ifx_work);
    }

    return 0;
}
//20100908 deepsleep wakeup issue [END]

/* End of TTY - SPI driver Operations */

/* ################################################################################################################ */

static struct spi_driver ifx_spi_driver = {
	.driver = {
		.name = "spi_ifxn721",	
                .bus = &spi_bus_type,
		.owner = THIS_MODULE,
	},
	.probe = ifx_spi_probe,
	.remove = __devexit_p(ifx_spi_remove),
    //20100908 deepsleep wakeup issue [START]
	.suspend = ifx_spi_suspend,
	.resume = ifx_spi_resume,
    //20100908 deepsleep wakeup issue [END]
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
ifx_spi_buffer_initialization(void)
{
	ifx_sender_buf_size = IFX_SPI_DEFAULT_BUF_SIZE;
        ifx_receiver_buf_size = IFX_SPI_DEFAULT_BUF_SIZE;
}

/*
 * Allocate memeory for TX_BUFFER and RX_BUFFER
 */
static int 
ifx_spi_allocate_frame_memory(unsigned int memory_size)
{
	int status = 0;
	ifx_rx_buffer = kmalloc(memory_size+IFX_SPI_HEADER_SIZE, GFP_KERNEL);
	if (!ifx_rx_buffer){
		printk("Open Failed ENOMEM\n");
		status = -ENOMEM;
	}
	ifx_tx_buffer = kmalloc(memory_size+IFX_SPI_HEADER_SIZE, GFP_KERNEL);
	if (!ifx_tx_buffer){		
		printk("Open Failed ENOMEM\n");
		status = -ENOMEM;
	}
	if(status == -ENOMEM){
		if(ifx_tx_buffer){
			kfree(ifx_tx_buffer);
		}
		if(ifx_rx_buffer){
			kfree(ifx_rx_buffer);            
		}
	}
	return status;
}

/*
 * Function to set header information according to IFX SPI framing protocol specification
 */
static void 
ifx_spi_set_header_info(unsigned char *header_buffer, unsigned int curr_buf_size, unsigned int next_buf_size)
{
	int i;
	union ifx_spi_frame_header header;
	for(i=0; i<4; i++){
		header.framesbytes[i] = 0;
	}

	header.ifx_spi_header.curr_data_size = curr_buf_size;
	if(next_buf_size){
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
ifx_spi_get_header_info(unsigned int *valid_buf_size)
{
	int i;
	union ifx_spi_frame_header header;

	for(i=0; i<4; i++){
		header.framesbytes[i] = 0;
	}

	for(i=3; i>=0; i--){
		header.framesbytes[i] = ifx_rx_buffer[/*3-*/i];
	}

//20101127-2, , Discard if mux size is bigger than MAX SIZE [START]
	if(header.ifx_spi_header.curr_data_size>IFX_SPI_MAX_BUF_SIZE)	//20101201-1, , bug fix : >= -> >
	{
		printk("%s - invalid header : 0x%x 0x%x 0x%x 0x%x!!!\n", __FUNCTION__, header.framesbytes[0], header.framesbytes[1], header.framesbytes[2], header.framesbytes[3]);
		*valid_buf_size = 0;
	}
	else
		*valid_buf_size = header.ifx_spi_header.curr_data_size;
//20101127-2, , Discard if mux size is bigger than MAX SIZE [END]

	if(header.ifx_spi_header.more){
		return header.ifx_spi_header.next_data_size;
	}
	return 0;
}

/*
 * Function to set/reset MRDY signal
 */
static void 
ifx_spi_set_mrdy_signal(int value)
{
	//struct GpioHandle ifxGpioHandle;
	//gpio_set_value(IFX_MRDY_GPIO, value);

	NvOdmGpioSetState(hGpio,hPin,(NvU32)value);	//20100607, , remain only this code
	//NvOdmGpioConfig(hGpio, hPin, NvOdmGpioPinMode_Output);
}

/*
 * Function to calculate next_frame_size required for filling in SPI frame Header
 */
static int 
ifx_spi_get_next_frame_size(int count)
{
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
ifx_spi_setup_transmission(void)
{
	if( (ifx_sender_buf_size != 0) || (ifx_receiver_buf_size != 0) ){
		if(ifx_sender_buf_size > ifx_receiver_buf_size){
			ifx_current_frame_size = ifx_sender_buf_size;
		}
		else{ 
			ifx_current_frame_size = ifx_receiver_buf_size;    
		}
		if(ifx_spi_count > 0){
			if(ifx_spi_count > ifx_current_frame_size){
				ifx_valid_frame_size = ifx_current_frame_size;
				ifx_spi_count = ifx_spi_count - ifx_current_frame_size;
			}
			else{
				ifx_valid_frame_size = ifx_spi_count;
				ifx_spi_count = 0;
			}
                }
		else{
			ifx_valid_frame_size = 0;
			ifx_sender_buf_size = 0;
		}
		ifx_sender_buf_size = ifx_spi_get_next_frame_size(ifx_spi_count);

		/* memset buffers to 0 */
		memset(ifx_tx_buffer,0,IFX_SPI_MAX_BUF_SIZE+IFX_SPI_HEADER_SIZE);
		memset(ifx_rx_buffer,0,IFX_SPI_MAX_BUF_SIZE+IFX_SPI_HEADER_SIZE);

		/* Set header information */
		ifx_spi_set_header_info(ifx_tx_buffer, ifx_valid_frame_size, ifx_sender_buf_size);
		if( ifx_valid_frame_size > 0 ){      
			memcpy(ifx_tx_buffer+IFX_SPI_HEADER_SIZE, ifx_spi_buf, ifx_valid_frame_size);
			ifx_spi_buf = ifx_spi_buf + ifx_valid_frame_size;
		}
	}
}

#define MAX_SRDY_ABNORMAL_HIGH 10
/*
 * Function starts Read and write operation and transfers received data to TTY core. It pulls down MRDY signal
 * in case of single frame transfer then sets "ifx_read_write_completion" to indicate transfer complete.
 */
static void 
ifx_spi_send_and_receive_data(struct ifx_spi_data *spi_data)
{
	unsigned int rx_valid_buf_size;
	int status = 0; 

	SPI_DEBUG_PRINT("SPI TX : ");
	dump_atcmd(ifx_tx_buffer, ifx_tx_buffer[0]) ; 	
	status = ifx_spi_sync_read_write(spi_data, ifx_current_frame_size+IFX_SPI_HEADER_SIZE); /* 4 bytes for header */                         
	if(status > 0){
		memset(ifx_tx_buffer,0,IFX_SPI_MAX_BUF_SIZE+IFX_SPI_HEADER_SIZE);
		ifx_ret_count = ifx_ret_count + ifx_valid_frame_size;
	}

	// hgahn
	if(memcmp(rx_dummy, ifx_rx_buffer, IFX_SPI_HEADER_SIZE) ==0) {
#ifdef WAKE_LOCK_RESUME
		/*
		   CP wake up AP with SRDY interrupt, but CP didn't send any data!!
		   Count up the SRDY state when SRDY is still high
		   */
		if(spi_data->wake_lock_flag)
		{
			unsigned int u32register = 0 ;
			u32register = readl(IO_ADDRESS(0x6000d1b8));
			if(((u32register>>5)&0x00000001))
			{
				spi_data->srdy_high_counter++;
				printk("%s -u32register = %08X, SRDY = %d, spi_data->srdy_high_counter = %d\n", 
						__FUNCTION__, u32register, ((u32register>>5)&0x00000001), spi_data->srdy_high_counter); 
			}
			if(spi_data->srdy_high_counter>MAX_SRDY_ABNORMAL_HIGH)
			{       //CP SRDY is still high for max counter times, start RIL Recovery
				const char g_FAKE_RX_DATA[] = 
				{0x19,0x00,0xfc,0x07,0xf9,0x05,0xef,0x27,0x0d,0x0a,0x2b,'X','C','A','L','L','S','T','A','T',':',' ','1',',','6',0x0d,0x0a,0x8a,0xf9,0x00};
				//Send fake unsol '+XCALLSTAT=1,6' to rild
				memcpy(ifx_rx_buffer, g_FAKE_RX_DATA, g_FAKE_RX_DATA[0]+IFX_SPI_HEADER_SIZE);
				printk("%s : Send fake unsol '+XCALLSTAT=1,6' to rild!! : spi_data->srdy_high_counter = %d\n", 
						__FUNCTION__, spi_data->srdy_high_counter); 
			}
			else
			{
				ifx_receiver_buf_size = 0;
				return;
			}
		}
		else
		{       //This is not a resume case, just normal operation case
			spi_data->srdy_high_counter = 0;
			//printk("%s : Normal case : spi_data->srdy_high_counter = %d\n", __FUNCTION__, spi_data->srdy_high_counter); 
			ifx_receiver_buf_size = 0;
			return;
		}
#else          
		ifx_receiver_buf_size = 0;
		return;
#endif
	}

	/* Handling Received data */
	ifx_receiver_buf_size = ifx_spi_get_header_info(&rx_valid_buf_size);

	SPI_DEBUG_PRINT("SPI RX : ");
	dump_atcmd(ifx_rx_buffer, 20) ; 	

	if((spi_data->throttle == 0) && (rx_valid_buf_size != 0) && (spi_data->ifx_tty!=NULL)){	//20100711-3, , check ifx_tty
		tty_insert_flip_string(spi_data->ifx_tty, ifx_rx_buffer+IFX_SPI_HEADER_SIZE, rx_valid_buf_size);
		tty_flip_buffer_push(spi_data->ifx_tty);
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
#ifdef CONFIG_LPRINTK
			lge_debug[D_SPI].enable = 0;
#endif
			spi_data->srdy_high_counter = 0;
		}
#endif
}

/*
 * Function copies the TX_BUFFER and RX_BUFFER pointer to a spi_transfer structure and add it to SPI tasks.
 * And calls SPI Driver function "spi_sync" to start data transmission and reception to from MODEM
 */
static unsigned int 
ifx_spi_sync_read_write(struct ifx_spi_data *spi_data, unsigned int len)
{
	int status;
	struct spi_message	m;
	struct spi_transfer	t = {
					.tx_buf		= ifx_tx_buffer,
                        		.rx_buf		= ifx_rx_buffer,
					.len		= len,
				    };
        spi_message_init(&m);
	spi_message_add_tail(&t, &m);

	//SPI_DEBUG_PRINT("ifx_spi_sync_read_write\n");	
	if (spi_data->spi == NULL)
		status = -ESHUTDOWN;
	else
		status = spi_sync(spi_data->spi, &m);
	
	if (status == 0){          
		status = m.status;
		if (status == 0)
			status = m.actual_length;
	}
        else{
		printk("File: ifx_n721_spi.c\tFunction: unsigned int ifx_spi_sync\tTransmission UNsuccessful\n");
        }
	return status;
}

/*
 * Function is a Interrupt service routine, is called when SRDY signal goes HIGH. It set up transmission and
 * reception if it is a Slave initiated data transfer. For both the cases Master intiated/Slave intiated
 * transfer it starts data transfer. 
 */
static irqreturn_t 
ifx_spi_handle_srdy_irq(int irq, void *handle)
{
	struct ifx_spi_data *spi_data = (struct ifx_spi_data *)handle;

	queue_work(spi_data->ifx_wq, &spi_data->ifx_work);    
	return IRQ_HANDLED; 
}

static void 
ifx_spi_handle_work(struct work_struct *work)
{
	struct ifx_spi_data *spi_data = container_of(work, struct ifx_spi_data, ifx_work);

	SPI_DEBUG_PRINT("ifx_spi_handle_work start\n");	
	if (!ifx_master_initiated_transfer){
		ifx_spi_setup_transmission();
		ifx_spi_set_mrdy_signal(1);
		ifx_spi_send_and_receive_data(spi_data);
		/* Once data transmission is completed, the MRDY signal is lowered */
		if((ifx_sender_buf_size == 0)  && (ifx_receiver_buf_size == 0)){
			ifx_spi_set_mrdy_signal(0);
			ifx_spi_buffer_initialization();
		}

		/* We are processing the slave initiated transfer in the mean time Mux has requested master initiated data transfer */
		/* Once Slave initiated transfer is complete then start Master initiated transfer */
		if(ifx_master_initiated_transfer == 1){
		/* It is a condition where Slave has initiated data transfer and both SRDY and MRDY are high and at the end of data transfer		
	 	* MUX has some data to transfer. MUX initiates Master initiated transfer rising MRDY high, which will not be detected at Slave-MODEM.
	 	* So it was required to rise MRDY high again */
//20100701-1, , delay time until CP can be ready again [START]
		udelay(MRDY_DELAY_TIME);
//20100701-1, , delay time until CP can be ready again [END]
                ifx_spi_set_mrdy_signal(1);    		
		}
	}
	else{
		ifx_spi_setup_transmission();     
		ifx_spi_send_and_receive_data(spi_data);
		/* Once data transmission is completed, the MRDY signal is lowered */
		if(ifx_sender_buf_size == 0){
			if(ifx_receiver_buf_size == 0){		
				ifx_spi_set_mrdy_signal(0);
//20100701-1, , delay time until CP can be ready again [START]
				udelay(MRDY_DELAY_TIME);
//20100701-1, , delay time until CP can be ready again [END]
				ifx_spi_buffer_initialization();
			}
			ifx_master_initiated_transfer = 0;
			complete(&spi_data->ifx_read_write_completion);
		}
	}
	SPI_DEBUG_PRINT("ifx_spi_handle_work end\n");	
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

	SPI_DEBUG_PRINT("ifx_spi_init\n");

	/* Allocate and Register a TTY device */
	ifx_spi_tty_driver = alloc_tty_driver(IFX_N_SPI_MINORS);
	if (!ifx_spi_tty_driver){
		printk(KERN_ERR "Fail to allocate TTY Driver\n");
		return -ENOMEM;
	}

	/* initialize the tty driver */
	ifx_spi_tty_driver->owner = THIS_MODULE;
	ifx_spi_tty_driver->driver_name = "tty_ifxn721";	//20100607, , modify ifxn721 -> tty_ifxn721
	ifx_spi_tty_driver->name = "ttyspi";
	ifx_spi_tty_driver->major = IFX_SPI_MAJOR;
	ifx_spi_tty_driver->minor_start = 0;
	ifx_spi_tty_driver->type = TTY_DRIVER_TYPE_SERIAL;
	ifx_spi_tty_driver->subtype = SERIAL_TYPE_NORMAL;
	ifx_spi_tty_driver->flags = TTY_DRIVER_REAL_RAW;
	ifx_spi_tty_driver->init_termios = tty_std_termios;
	ifx_spi_tty_driver->init_termios.c_cflag = B38400 | CS8 | CREAD | HUPCL | CLOCAL;
	tty_set_operations(ifx_spi_tty_driver, &ifx_spi_ops);

	status = tty_register_driver(ifx_spi_tty_driver);
	if (status){
		printk(KERN_ERR "Failed to register IFX SPI tty driver");
		put_tty_driver(ifx_spi_tty_driver);
		return status;
	}

	/* Register SPI Driver */
	status = spi_register_driver(&ifx_spi_driver);
	if (status < 0){ 
		printk("Failed to register SPI device");
		tty_unregister_driver(ifx_spi_tty_driver);
		put_tty_driver(ifx_spi_tty_driver);
		return status;
	}
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
