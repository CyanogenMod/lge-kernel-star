/*
 * drivers/spi/tegra_spi_slave.c
 *
 * SPI bus slave driver for NVIDIA Tegra SoCs, based on NvRm APIs
 *
 * Copyright (c) 2009-2010, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#define NV_DEBUG 0

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/err.h>
#include <linux/workqueue.h>
#include <linux/delay.h>

#include <mach/spi.h>
#include <mach/nvrm_linux.h>
#include <nvrm_spi.h>
#include <nvodm_query.h>
#include <nvodm_services.h>
#include <nvodm_query_discovery.h>
#include <rm_spi_slink.h>

//LGE_TELECA_CR:707_DATA_THROUGHPUT START
#if NV_DEBUG
//#define CONFIG_SPI_DEBUG
#ifdef CONFIG_SPI_DEBUG
#define SPI_DEBUG_PRINT(format, args...) printk(format , ## args)
#else
#include <mach/lprintk.h>
#define SPI_DEBUG_PRINT(format, args...) lprintk(D_SPI, format , ## args)
#endif
#else  /* NV_DEBUG */
#define SPI_DEBUG_PRINT(format, args...)
#endif
//LGE_TELECA_CR:707_DATA_THROUGHPUT END

/* Cannot use spinlocks as the NvRm SPI apis uses mutextes and one cannot use
 * mutextes inside a spinlock.
 */
#define USE_SPINLOCK 1
#if USE_SPINLOCK
#define LOCK_T          spinlock_t
#define CREATELOCK(_l)  spin_lock_init(&(_l))
#define DELETELOCK(_l)
#define LOCK(_l)        spin_lock(&(_l))
#define UNLOCK(_l)      spin_unlock(&(_l))
#define ATOMIC(_l,_f)   spin_lock_irqsave(&(_l),(_f))
#define UNATOMIC(_l,_f) spin_unlock_irqrestore(&(_l),(_f))
#else
#include <linux/mutex.h>

#define LOCK_T          struct mutex
#define CREATELOCK(_l)  mutex_init(&(_l))
#define DELETELOCK(_l) 
#define LOCK(_l)        mutex_lock(&(_l))
#define UNLOCK(_l)      mutex_unlock(&(_l))
#define ATOMIC(_l,_f)   local_irq_save((_f))
#define UNATOMIC(_l,_f) local_irq_restore((_f))
#endif

#define SPI_GUID                NV_ODM_GUID('s','t','a','r','-','s','p','i')
//LGE_TELECA_CR:568_DUAL_SPI START
#ifdef LGE_DUAL_SPI_1
static LOCK_T     spi_sync_lock;
static bool        bOnce = 0;
#endif
//LGE_TELECA_CR:568_DUAL_SPI END

struct tegra_spi {
	NvU32			index;		//20100811-1, , spi dev id
	NvRmSpiHandle		rm_spi;
	NvU32			pinmux;
	NvU32			Mode;
	struct list_head	msg_queue;
	LOCK_T			lock;
	struct work_struct	work;
	struct workqueue_struct	*queue;
}; 

static int spi_shutdown = 0;

/* Only these signaling mode are supported */
#define NV_SUPPORTED_MODE_BITS (SPI_CPOL | SPI_CPHA)

//To check the step of SPI process
static unsigned int uiStepCount = 0;

#define SPI_SLAVE_ACTIVE			1
#define ENABLE_TX_RX_DUMP		0

#define USE_SRDY
#ifdef USE_SRDY
#define MAX_USED_SPI_DREVICE	2
static NvOdmServicesGpioHandle hGpio[MAX_USED_SPI_DREVICE] = {NULL, NULL};
static NvOdmGpioPinHandle hPin[MAX_USED_SPI_DREVICE] = {NULL, NULL};

/*
 * Function to set/reset SRDY signal
 */
static void 
tegra_spi_set_srdy_signal(NvU32 id, NvU32 value)
{

    if (spi_shutdown) return;
//LGE_TELECA_CR1317_DATA_THROUGHPUT START
	if(id < MAX_USED_SPI_DREVICE) {
		NvOdmGpioSetState(hGpio[id],hPin[id], value);

		if(!value) {
			udelay(1); // Delay time for a recharge
		}
	} else {
		printk("%s : Can't handle id[%d]\n", __FUNCTION__, id);
	}
//LGE_TELECA_CR1317_DATA_THROUGHPUT END
	SPI_DEBUG_PRINT("%s : id[%d]=%d\n", __FUNCTION__, id, value);
}
#endif	//USE_SRDY

static int tegra_spi_setup(struct spi_device *device)
{
	struct tegra_spi *spi;

	spi = spi_master_get_devdata(device->master);

	SPI_DEBUG_PRINT("tegra_spi_setup : device->mode(%d)\n", device->mode);

	if (device->mode & ~NV_SUPPORTED_MODE_BITS) {
		dev_dbg(&device->dev, "setup: unsupported mode bits 0x%x\n",
			device->mode & ~NV_SUPPORTED_MODE_BITS);
	}

	spi->Mode = device->mode & NV_SUPPORTED_MODE_BITS;
	switch (spi->Mode) {
	case SPI_MODE_0:
		spi->Mode = NvOdmQuerySpiSignalMode_0;
		break;
	case SPI_MODE_1:
		spi->Mode = NvOdmQuerySpiSignalMode_1;
		break;
	case SPI_MODE_2:
		spi->Mode = NvOdmQuerySpiSignalMode_2;
		break;
	case SPI_MODE_3:
		spi->Mode = NvOdmQuerySpiSignalMode_3;
		break;
	}

	if (device->bits_per_word == 0)
		device->bits_per_word = 8;

	NvRmSpiSetSignalMode(spi->rm_spi, device->chip_select, spi->Mode);
	return 0;
}

static int tegra_spi_transfer(struct spi_device *device,
	struct spi_message *msg)
{
	struct tegra_spi *spi;
    if (spi_shutdown) return 0;

	if (unlikely(list_empty(&msg->transfers) || !device->max_speed_hz))
		return -EINVAL;

	/* FIXME validate the msg */

	spi = spi_master_get_devdata(device->master);

	SPI_DEBUG_PRINT("%s-%d\n",__FUNCTION__, __LINE__);

	/* Add the message to the queue and signal the worker thread */
	LOCK(spi->lock);
	list_add_tail(&msg->queue, &spi->msg_queue);
	queue_work(spi->queue, &spi->work);
	UNLOCK(spi->lock);

	return 0;
}

static void tegra_spi_cleanup(struct spi_device *device)
{
	return;
}

static int tegra_spi_do_message(struct tegra_spi *spi, struct spi_message *m)
{
	NvRmSpiTransactionInfo trans[64];
	struct spi_transfer *t;
	unsigned int len = 0;
	NvU32 j, i= 0;
	NvU32 actual_length = 0;
	NvU32 u32Transfered, u32WaitTimeout = 200;	//NV_WAIT_INFINITE;
	NvU32 BitsPerWord;
	NvU32 ClockInKHz;
	NvU32 ChipSelect;
	NvBool IsReadTransfer = NV_FALSE;
	NvError nvErr = NvSuccess;
#ifdef LGE_DUAL_SPI_1
//LGE_TELECA_CR:568_DUAL_SPI START
	static int SPI_IN_RDY = 1;
	static int SPI_OUT_RDY = 1;
//LGE_TELECA_CR:568_DUAL_SPI END
#endif

      if (spi_shutdown) return 0;
	/* Get slave config.  PinMuxConfig is a property of the master. */
	BitsPerWord = m->spi->bits_per_word;
	ClockInKHz = m->spi->max_speed_hz / 1000;
	ChipSelect = m->spi->chip_select;

	list_for_each_entry(t, &m->transfers, transfer_list) {
		if (i==ARRAY_SIZE(trans))
			return -EIO;

		if (t->len && !t->tx_buf && !t->rx_buf)
			return -EINVAL;

		if (t->cs_change) {
			WARN_ON_ONCE(1);
			return -EIO;
		}

		if (t->len) {
			trans[i].rxBuffer = t->rx_buf;
			trans[i].txBuffer = (NvU8*)t->tx_buf;
			trans[i].len = t->len;
			len += t->len;
#if SPI_SLAVE_ACTIVE
#if ENABLE_TX_RX_DUMP
			if(t->tx_buf) {
				printk("spi[%d] tx =", spi->index);
				for(j=0;j<20;j++)
				{
#if 0				
					if(spi->index){
						((NvU8 *)t->tx_buf)[j] = j+0x41;
					}
#endif
					if(((NvU8 *)t->tx_buf)[j]>=32 && ((NvU8 *)t->tx_buf)[j]<126)
						printk("%c,",((NvU8 *)t->tx_buf)[j]);	
					else
						printk("%x,",((NvU8 *)t->tx_buf)[j]);	
				}
			}
#endif	//DEBUG_PRINT				
			IsReadTransfer = NV_TRUE;
			nvErr = NvRmSpiStartTransaction(
				spi->rm_spi,
				ChipSelect,
				ClockInKHz,
				IsReadTransfer,
				t->tx_buf,
				t->len,
				BitsPerWord);
//LGE_TELECA_CR:568_DUAL_SPI START
#ifdef LGE_DUAL_SPI_1
			LOCK(spi_sync_lock);
			SPI_IN_RDY = !SPI_IN_RDY;
			UNLOCK(spi_sync_lock);
#endif
//LGE_TELECA_CR:568_DUAL_SPI END
			if(nvErr!=NvSuccess)
			{
				printk("tegra_spi_workerthread[ID:%d] ; Start error : %d\n", spi->index, nvErr);
				break;
			}
#ifdef USE_SRDY
#ifdef LGE_DUAL_SPI_1
//LGE_TELECA_CR:568_DUAL_SPI START
			SPI_DEBUG_PRINT("SPI_IN_RDY %s\n", (SPI_IN_RDY)?"true":"false");
			if (SPI_IN_RDY) 
//LGE_TELECA_CR:568_DUAL_SPI END
				tegra_spi_set_srdy_signal(0, 1);
#else
			tegra_spi_set_srdy_signal(spi->index, 1);
#endif
#endif
			nvErr = NvRmSpiGetTransactionData(
				spi->rm_spi,
				t->rx_buf,
				t->len,
				&u32Transfered,
				u32WaitTimeout);

#ifdef USE_SRDY
#ifdef LGE_DUAL_SPI_1
//LGE_TELECA_CR:568_DUAL_SPI START
			LOCK(spi_sync_lock);
			SPI_OUT_RDY = !SPI_OUT_RDY;
			UNLOCK(spi_sync_lock);

			SPI_DEBUG_PRINT("SPI_OUT_RDY %s \n", (SPI_OUT_RDY)?"true":"false");
			if (SPI_OUT_RDY) 
//LGE_TELECA_CR:568_DUAL_SPI END
				tegra_spi_set_srdy_signal(0, 0);
#else
			tegra_spi_set_srdy_signal(spi->index, 0);
#endif
#endif
			if(nvErr != NvSuccess)
			{	//20110120-1, , Add workaround code for short SCK
				printk("%s[ID:%d] : WaitTimeout error %d\n", __FUNCTION__, spi->index, nvErr);
				NvRmSpiClose(spi->rm_spi);
				nvErr = NvRmSpiOpen(s_hRmGlobal, NvOdmIoModule_Spi, spi->index, 0, &spi->rm_spi);
				printk("%s[ID:%d] : Restart NvRmSpiOpen %d\n", __FUNCTION__, spi->index, nvErr);
				break;
			}
#if ENABLE_TX_RX_DUMP
			if(t->rx_buf) {
				printk("spi[%d] rx =", spi->index);
				for(j=0;j<20;j++)
				{
					if(((NvU8 *)t->rx_buf)[j]>=32 && ((NvU8 *)t->rx_buf)[j]<126)
						printk("%c,",((NvU8 *)t->rx_buf)[j]);	
					else
						printk("%x,",((NvU8 *)t->rx_buf)[j]);	
				}
			}
			printk("\n");
#endif	//DEBUG_PRINT				
#else	//ACT AS MASTER
			printk("spi tx buffer / before =");
			for(j=0;j<10;j++)
			{
				printk("%x ",((NvU8 *)t->tx_buf)[j]);	
			}
			printk("\n");
			NvRmSpiTransaction(spi->rm_spi, spi->pinmux, ChipSelect, 
				ClockInKHz, t->rx_buf, t->tx_buf, t->len, BitsPerWord);
			printk("spi rx buffer / after =");
			for(j=0;j<10;j++)
			{
				printk("%x ",((NvU8 *)t->rx_buf)[j]);	
			}
			printk("\n");
#endif				
		}

		i++;
	}

	if (!i)
		return 0;
	
	if(nvErr == NvSuccess)
		m->actual_length += len;
	
#if !(SPI_SLAVE_ACTIVE)
	NvRmSpiMultipleTransactions(spi->rm_spi, spi->pinmux,
		m->spi->chip_select, m->spi->max_speed_hz / 1000,
		m->spi->bits_per_word, trans, i);
#endif //!SPI_SLAVE_ACTIVE

	return 0;
}

static void tegra_spi_workerthread(struct work_struct *w)
{
	struct tegra_spi *spi;

	spi = container_of(w, struct tegra_spi, work);

	SPI_DEBUG_PRINT("tegra_spi_transfer start\n");
	LOCK(spi->lock);

	while (!list_empty(&spi->msg_queue)) {
		struct spi_message *m;
        if (spi_shutdown )
        {
           printk("tegra_spi_workthread stopped\n");
		   return;
        }  
		 
		m = container_of(spi->msg_queue.next, struct spi_message, queue);
		list_del_init(&m->queue);
		UNLOCK(spi->lock);

		if (!m->spi) {
			WARN_ON(1);
			return;
		}
		m->status = tegra_spi_do_message(spi, m);
		m->complete(m->context);

		LOCK(spi->lock);
	}

	UNLOCK(spi->lock);
	SPI_DEBUG_PRINT("tegra_spi_transfer end\n");
}

static int __init tegra_spi_probe(struct platform_device *pdev)
{
	struct spi_master *master;
	struct tegra_spi *spi;
	struct tegra_spi_platform_data *plat = pdev->dev.platform_data;
	int status= 0;
	NvError e;
#if SPI_SLAVE_ACTIVE
	bool isMaster = 0;
#else
	bool isMaster = 1;
#endif
	
#ifdef USE_SRDY
	NvOdmPeripheralConnectivity *pConnectivity = NULL;
	NvU32 port;
	NvU32 pin;

	printk("%s : name(%s), id(%d), num(%d)\n", __FUNCTION__, pdev->name, pdev->id, pdev->num_resources);
	if(pdev->id >=MAX_USED_SPI_DREVICE)
	{
		printk(KERN_ERR "%s : Can't handle spi dev id[%d]\n", __FUNCTION__, pdev->id);
		return -ENODEV;
	}		
	//Set MRDY pin as output
	pConnectivity = NvOdmPeripheralGetGuid(SPI_GUID);	   
	hGpio[pdev->id] =  (NvOdmServicesGpioHandle)NvOdmGpioOpen();
	if(hGpio[pdev->id]==NULL){
		printk(KERN_ERR "Failed to get GPIO handle\n");
		return -ENODEV;
	}

	port = pConnectivity->AddressList[1+pdev->id*3].Instance;
	pin = pConnectivity->AddressList[1+pdev->id*3].Address;
	hPin[pdev->id] = NvOdmGpioAcquirePinHandle(hGpio[pdev->id], port, pin);
	if(hPin[pdev->id]==NULL){
		printk(KERN_ERR "Failed to get MRDY pin handle\n");
		return -ENODEV;
	}
	NvOdmGpioSetState(hGpio[pdev->id], hPin[pdev->id], 0x00);
	NvOdmGpioConfig(hGpio[pdev->id], hPin[pdev->id], NvOdmGpioPinMode_Output);

#endif
	SPI_DEBUG_PRINT("tegra_spi_probe\n");

	master = spi_alloc_master(&pdev->dev, sizeof(*spi));
	if (IS_ERR_OR_NULL(master)) {
		dev_err(&pdev->dev, "master allocation failed\n");
		return -ENOMEM;
	}

//20100711-1, , add mode_bits [START]
	/* the spi->mode bits understood by this driver: */
	master->mode_bits = NV_SUPPORTED_MODE_BITS;
//20100711, , add mode_bits [END]

	master->setup = tegra_spi_setup;
	master->transfer = tegra_spi_transfer;
	master->cleanup = tegra_spi_cleanup;
	master->num_chipselect = MAX_USED_SPI_DREVICE;
	master->bus_num = pdev->id;

	dev_set_drvdata(&pdev->dev, master);
	spi = spi_master_get_devdata(master);

	spi->pinmux = plat->pinmux;
	spi->index = pdev->id;	//20100811-1, , Save spi dev id

	SPI_DEBUG_PRINT("tegra_spi_probe : NvRmSpiOpen\n");
	if (plat->is_slink) { 
		e = NvRmSpiOpen(s_hRmGlobal, NvOdmIoModule_Spi,
				pdev->id, isMaster, &spi->rm_spi);
	} else {
		e = NvRmSpiOpen(s_hRmGlobal, NvOdmIoModule_Sflash,
				0, isMaster, &spi->rm_spi);
	}
	if (e != NvSuccess) {
		dev_err(&pdev->dev, "NvRmSpiOpen returned 0x%x\n", e);
		status = -ENODEV;
		goto spi_open_failed;
	}

	SPI_DEBUG_PRINT("tegra_spi_probe : Create work queue(%s)\n", dev_name(&pdev->dev));
	spi->queue = create_singlethread_workqueue(dev_name(&pdev->dev));
	if (!spi->queue) {
		dev_err(&pdev->dev, "Failed to create work queue\n");
		goto workQueueCreate_failed;
	}

	INIT_WORK(&spi->work, tegra_spi_workerthread);
//LGE_TELECA_CR:568_DUAL_SPI START
#ifdef LGE_DUAL_SPI_1
	CREATELOCK(spi->lock);
	if(bOnce==0) {
           bOnce = 1;
           CREATELOCK(spi_sync_lock);
	}
#endif
//LGE_TELECA_CR:568_DUAL_SPI END
	INIT_LIST_HEAD(&spi->msg_queue);

	SPI_DEBUG_PRINT("tegra_spi_probe : spi register master(bus num = %d)\n", master->bus_num);
	status = spi_register_master(master);
	SPI_DEBUG_PRINT("tegra_spi_probe : spi register master(%d)\n", status);
	if (status < 0) {
		dev_err(&pdev->dev, "spi_register_master failed %d\n", status);
		goto spi_register_failed;
	}

	return status;
	
spi_register_failed:
	destroy_workqueue(spi->queue);
workQueueCreate_failed:
	NvRmSpiClose(spi->rm_spi);
spi_open_failed:
	spi_master_put(master);
	return status;
}
	
static int tegra_spi_remove(struct platform_device *pdev)
{
	struct spi_master *master;
	struct tegra_spi *spi;

	master = dev_get_drvdata(&pdev->dev);
	spi = spi_master_get_devdata(master);

	spi_unregister_master(master);
	NvRmSpiClose(spi->rm_spi);
	destroy_workqueue(spi->queue);

	return 0;
}

static void star_spi_shutdown(struct platform_device *pdev)
{
	struct tegra_spi *spi;
    struct spi_master *master;

	spi_shutdown = 1;
	
	master = dev_get_drvdata(&pdev->dev);
	
	if (master)
	{
	   spi = spi_master_get_devdata(master);
	}
	else 
	{
	    printk("star_spi_shutdown : can not acure master\n");
		return;
	}	
	if (spi->queue)
	{
	   flush_workqueue(spi->queue);
	   printk("star_spi_shutdown : flush_workqueue\n");
	}
	else
	{
	    printk("star_spi_shutdown : fail to flush workqueue\n");
	    return;
	}
	
	spi_unregister_master(master);
    if (spi->rm_spi)
    {
	  NvRmSpiClose(spi->rm_spi);
	  printk("star_spi_shutdown : NvRmSpiClose\n");
    } 
	else
	{
	    printk("star_spi_shutdown : NvRmSpiClose\n");
		return;
	}	

    if (spi->queue)
    {
	  destroy_workqueue(spi->queue);
	  printk("star_spi_shutdown :  destroy_workqueue\n");
    } 
	else
	{
	   printk("star_spi_shutdown :  fail to destroy_workqueue\n");
	   return;
	}

    tegra_spi_set_srdy_signal(spi->index, 0);
	printk("star_spi_shutdown :  tegra_spi_set_srdy_signal(0)\n");
	if (hGpio[spi->index]) 
	{
	  NvOdmGpioClose(hGpio[spi->index]);
	  printk("star_spi_shutdown :  NvOdmGpioClose\n");
	} 	
 
     printk("star_spi_shutdown :  completed\n");
}

//20100607-1, , power management	[START]
#ifdef CONFIG_PM
static int tegra_spi_suspend(struct platform_device *pdev, pm_message_t msg)
{
	struct spi_master	*pSpi;
	struct NvSpiShim	*pShimSpi;

	pSpi = dev_get_drvdata(&pdev->dev);
	pShimSpi = spi_master_get_devdata(pSpi);

	printk( "tegra_spi_suspend\n");	//syblue.lee 100602 ; test

	return 0;
}

static int tegra_spi_resume(struct platform_device *pdev)
{
	struct spi_master	*pSpi;
	struct NvSpiShim	*pShimSpi;

	pSpi = dev_get_drvdata(&pdev->dev);
	pShimSpi = spi_master_get_devdata(pSpi);

	printk( "tegra_spi_resume\n");

	return 0;
}
#endif	
//20100607, , power management	[START]
MODULE_ALIAS("platform:tegra_spi");
static struct platform_driver tegra_spi_driver = {
	.probe = tegra_spi_probe,
	.remove = tegra_spi_remove,
	.shutdown = star_spi_shutdown,
	.driver	= {
		.name	= "tegra_spi",
		.owner	= THIS_MODULE,
	},
//20100607-1, , power management	[START]
#ifdef CONFIG_PM
			.suspend = tegra_spi_suspend,
			.resume = tegra_spi_resume, 
#endif	
//20100607, , power management	[START]
};

static int __init tegra_spi_init(void)
{
	int status;
	status =platform_driver_register(&tegra_spi_driver); 
	SPI_DEBUG_PRINT("tegra_spi_init : %d\n", status);
	return status;
}
module_init(tegra_spi_init);

static void __exit tegra_spi_exit(void)
{
	platform_driver_unregister(&tegra_spi_driver);
}
module_exit(tegra_spi_exit);

MODULE_AUTHOR("Sangyun Lee, <>");
MODULE_DESCRIPTION("Tegra SPI slave driver");
MODULE_LICENSE("GPL");
