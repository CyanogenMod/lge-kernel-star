/*
 * mtv250.c
 *
 * RAONTECH MTV818 driver.
 *
 * (c) COPYRIGHT 2010 RAONTECH, Inc. ALL RIGHTS RESERVED.
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */
 
#include <linux/version.h>

#include "mtv250.h"
#include "mtv250_ioctl.h"
#include "mtv250_gpio.h"
  
#include "raontv.h"
#include "raontv_internal.h"

#if defined(RTV_IF_MPEG2_SERIAL_TSIF) || defined(RTV_IF_MPEG2_PARALLEL_TSIF) || defined(RTV_IF_QUALCOMM_TSIF) || defined(RTV_IF_SPI_SLAVE)	
	#include "mtv250_i2c.h"
#elif defined(RTV_IF_SPI)
	#include "mtv250_spi.h"
#endif

extern irqreturn_t mtv250_isr(int irq, void *param);

struct mtv250_cb *mtv250_cb_ptr = NULL;
extern int mtv250_isr_thread(void *data);


static const U8 mtv_reg_page_addr[] = 
{
	0x07/*HOST*/, 0x0F/*RF*/, 0x04/*COMM*/, 0x09/*DD*/,
	0x0B/*MSC0*/, 0x0C/*MSC1*/
};

#if defined(RTV_IF_SPI)

	#define _DEBUG_TSP_POOL
	#ifdef _DEBUG_TSP_POOL
		static unsigned int max_used_tsp_cnt;
	#endif

static MTV250_TSP_QUEUE_INFO mtv250_tsp_pool;
static MTV250_TSP_QUEUE_INFO mtv250_tsp_queue;


unsigned int mtv250_get_total_tsp(void)
{
	unsigned int total_tsp_bytes = mtv250_tsp_queue.cnt * MTV_TS_THRESHOLD_SIZE;
	
	if(mtv250_cb_ptr->prev_tsp != NULL)
		total_tsp_bytes += mtv250_cb_ptr->prev_tsp->len;

	return total_tsp_bytes;
}

void mtv250_reset_tsp(void)
{
	MTV250_TS_PKT_INFO *tsp;

	if(max_used_tsp_cnt == 0) //!!!!
		return;
	
  	DMBMSG("[mtv250_reset_tsp] Max used TSP count: %d\n", max_used_tsp_cnt);

  	if(mtv250_cb_ptr->prev_tsp != NULL)
	{
		mtv250_free_tsp(mtv250_cb_ptr->prev_tsp);
		mtv250_cb_ptr->prev_tsp = NULL; 
	}
	while ((tsp=mtv250_get_tsp()) != NULL) 
	{
		mtv250_free_tsp(tsp);
	}

	max_used_tsp_cnt = 0;

	DMBMSG("[mtv250_reset_tsp] Pool TSP count: %d\n", mtv250_tsp_pool.cnt);
}

// Dequeue a ts packet from ts data queue.
MTV250_TS_PKT_INFO * mtv250_get_tsp(void)
{
	MTV250_TS_PKT_INFO *tsp = NULL;
	struct list_head *head_ptr = &mtv250_tsp_queue.head;
		
	if(mtv250_tsp_queue.cnt != 0) //if(!list_empty(head_ptr))
	{
		spin_lock(&mtv250_tsp_queue.lock);
		
		tsp = list_first_entry(head_ptr, MTV250_TS_PKT_INFO, link);
		list_del(&tsp->link);
		mtv250_tsp_queue.cnt--;

		spin_unlock(&mtv250_tsp_queue.lock);
	}
	
	return tsp;
}

// Enqueue a ts packet into ts data queue.
void mtv250_put_tsp(MTV250_TS_PKT_INFO *tsp)
{
	spin_lock(&mtv250_tsp_queue.lock);

	list_add_tail(&tsp->link, &mtv250_tsp_queue.head);
	mtv250_tsp_queue.cnt++;

	spin_unlock(&mtv250_tsp_queue.lock);
}


void mtv250_init_tsp_queue(void)
{
	mtv250_cb_ptr->prev_tsp = NULL;
	
	spin_lock_init(&mtv250_tsp_queue.lock);
	INIT_LIST_HEAD(&mtv250_tsp_queue.head);
	mtv250_tsp_queue.cnt = 0;
}


void mtv250_free_tsp(MTV250_TS_PKT_INFO *tsp)
{	
	spin_lock(&mtv250_tsp_pool.lock);

	tsp->len = 0;
	list_add_tail(&tsp->link, &mtv250_tsp_pool.head);
	mtv250_tsp_pool.cnt++;

	spin_unlock(&mtv250_tsp_pool.lock);
}


MTV250_TS_PKT_INFO *mtv250_alloc_tsp(void)
{	
	MTV250_TS_PKT_INFO *tsp = NULL;
	struct list_head *head_ptr = &mtv250_tsp_pool.head;
		
	if(mtv250_tsp_pool.cnt != 0) //if(!list_empty(head_ptr))
	{
		spin_lock(&mtv250_tsp_pool.lock);
		
		tsp = list_first_entry(head_ptr, MTV250_TS_PKT_INFO, link);
		list_del(&tsp->link);
		mtv250_tsp_pool.cnt--;
#ifdef _DEBUG_TSP_POOL
		max_used_tsp_cnt = MAX(max_used_tsp_cnt, MAX_NUM_TS_PKT_BUF-mtv250_tsp_pool.cnt);
#endif	
		spin_unlock(&mtv250_tsp_pool.lock);
	}
	
	return tsp;
	
}


static int mtv250_delete_tsp_pool(void)
{
	struct list_head *head_ptr = &mtv250_tsp_pool.head;
	MTV250_TS_PKT_INFO *tsp;

  	if(mtv250_cb_ptr->prev_tsp != NULL)
	{
		kfree(mtv250_cb_ptr->prev_tsp);
		mtv250_cb_ptr->prev_tsp = NULL; 
	}

	while ((tsp=mtv250_get_tsp()) != NULL) 
	{
		kfree(tsp);
	}

	while (!list_empty(head_ptr)) 
	{		
		tsp = list_entry(head_ptr->next, MTV250_TS_PKT_INFO, link);
		list_del(&tsp->link);		
		kfree(tsp);		
	}

	return 0;
}


static int mtv250_create_tsp_pool(void)
{
	unsigned int i;
	MTV250_TS_PKT_INFO *tsp;

	spin_lock_init(&mtv250_tsp_pool.lock);
	INIT_LIST_HEAD(&mtv250_tsp_pool.head);

	mtv250_tsp_pool.cnt = 0;
#ifdef _DEBUG_TSP_POOL
	max_used_tsp_cnt = 0;
#endif	

	for(i=0; i<MAX_NUM_TS_PKT_BUF; i++)
	{
		tsp = (MTV250_TS_PKT_INFO *)kmalloc(sizeof(MTV250_TS_PKT_INFO), GFP_DMA); 
		if(tsp == NULL)
		{
			mtv250_delete_tsp_pool();
			DMBERR("[mtv250_create_tsp_pool] %d TSP allocation failed!\n", i);
			return -ENOMEM;
		}

		tsp->len = 0;
		list_add_tail(&tsp->link, &mtv250_tsp_pool.head);
		mtv250_tsp_pool.cnt++;
	}	

 	return 0;
}

static void mtv250_exit_read_function(void)
{
#ifndef MTV250_NON_BLOCKING_READ_MODE
  	/* Stop read() function. User function may blocked at read(). */
	mtv250_cb_ptr->stop = 1;
	wake_up(&mtv250_cb_ptr->read_wq);

	/* Wait fo read() to free a previous tsp. */
	wait_for_completion(&mtv250_cb_ptr->read_exit);
#endif
}

static void mtv250_start_read_function(void)
{
#ifndef MTV250_NON_BLOCKING_READ_MODE
	mtv250_cb_ptr->stop = 0;
	init_completion(&mtv250_cb_ptr->read_exit);	
#endif
}
#endif




#if defined(RTV_IF_SPI) || (defined(RTV_TDMB_ENABLE) && !defined(RTV_TDMB_FIC_POLLING_MODE))	
static int mtv250_isr_thread_run(void)
{	
	if(mtv250_cb_ptr->isr_thread_cb != NULL)
		return 0;

	mtv250_cb_ptr->isr_cnt = 0;	
	init_waitqueue_head(&mtv250_cb_ptr->isr_wq); 	
	
	mtv250_cb_ptr->isr_thread_cb = kthread_run(mtv250_isr_thread, NULL, "mtv_isr_thread");
	if (IS_ERR(mtv250_cb_ptr->isr_thread_cb)) 
	{
		mtv250_cb_ptr->isr_thread_cb = NULL;
		return PTR_ERR(mtv250_cb_ptr->isr_thread_cb);
	}

	return 0;
}

static void mtv250_isr_thread_stop(void)
{
	if(mtv250_cb_ptr->isr_thread_cb == NULL)
		return;

	kthread_stop(mtv250_cb_ptr->isr_thread_cb);
	mtv250_cb_ptr->isr_thread_cb = NULL;	
}
#endif

static int mtv250_power_off(void)
{
	DMBMSG("[mtv250_power_off] START\n");

	if(mtv250_cb_ptr->is_power_on == FALSE)
		return 0;

	mtv250_cb_ptr->is_power_on = FALSE;
	
	rtvISDBT_DisableStreamOut();

	RTV_MASTER_CHIP_SEL;
	rtvOEM_PowerOn(0);

#if defined(RTV_IF_SPI) || (defined(RTV_TDMB_ENABLE) && !defined(RTV_TDMB_FIC_POLLING_MODE))		
	mtv250_reset_tsp();
#endif	

	DMBMSG("[mtv250_power_off] END\n");

	return 0;
}


static int mtv250_power_on(void)
{
	int ret = 0;

//DMBMSG("[mtv250_power_on] 1\n");

	if(mtv250_cb_ptr->is_power_on == TRUE)	
		return 0;

	RTV_MASTER_CHIP_SEL;
	rtvOEM_PowerOn(1);

//DMBMSG("[mtv250_power_on] 2\n");
	
	mtv250_cb_ptr->stop = 0;

	mtv250_cb_ptr->start_ts_int_cnt = 0;
	mtv250_cb_ptr->ovf_err_cnt = 0;
	mtv250_cb_ptr->max_read_jiffies = 0;
	
	ret = rtvISDBT_Initialize(mtv250_cb_ptr->country_band_type, MTV_TS_THRESHOLD_SIZE);
	if(ret  != RTV_SUCCESS)
	{
		DMBERR("[mtv250_power_on] rtvISDBT_Initialize() failed: %d\n", ret);
		ret = -EFAULT;		
		goto err_return;
	}	

//DMBMSG("[mtv250_power_on] 3\n");

	
#if defined(RTV_IF_SPI)
	mtv250_start_read_function(); /* To change stop flag when Power On/OFF. */
#endif	

#if defined(RTV_IF_SPI) || (defined(RTV_TDMB_ENABLE) && !defined(RTV_TDMB_FIC_POLLING_MODE))	
	mtv250_cb_ptr->isr_cnt = 0;	
#endif	

	mtv250_cb_ptr->is_power_on = TRUE;
	
	DMBMSG("[mtv250_power_on] Power on done\n" );

	return 0;

err_return:

	mtv250_power_off();

	return ret;
}



static void mtv250_deinit_device(void)
{
#if defined(RTV_IF_SPI) || (defined(RTV_TDMB_ENABLE) && !defined(RTV_TDMB_FIC_POLLING_MODE))	
	free_irq(gpio_to_irq(RAONTV_IRQ_INT), NULL);
	mtv250_isr_thread_stop();
#endif	

	mtv250_power_off();

#if defined(RTV_IF_MPEG2_SERIAL_TSIF) || defined(RTV_IF_MPEG2_PARALLEL_TSIF) || defined(RTV_IF_QUALCOMM_TSIF) || defined(RTV_IF_SPI_SLAVE)	
	mtv250_i2c_exit(); 
  
#elif defined(RTV_IF_SPI)	
	/* Wait for read() compeltion. memory crasy... */
	mtv250_delete_tsp_pool();

	mtv250_spi_exit(); 
#endif	

	if(mtv250_cb_ptr != NULL)
		kfree(mtv250_cb_ptr);


	DMBMSG("[mtv250_deinit_device] END\n");
}


static int mtv250_init_device(void)
{
	int ret = 0;

	mtv250_cb_ptr = kzalloc(sizeof(struct mtv250_cb), GFP_KERNEL);
	if (!mtv250_cb_ptr)
	{
		DMBERR("[mtv250_init_device] kzalloc failed\n");
		return -ENOMEM;
	}
	
	mtv250_cb_ptr->is_power_on = 0; // for prove()	
	mtv250_cb_ptr->stop = 0;

	mtv250_configure_gpio();

#if defined(RTV_IF_MPEG2_SERIAL_TSIF) || defined(RTV_IF_SPI_SLAVE) || defined(RTV_IF_MPEG2_PARALLEL_TSIF) || defined(RTV_IF_QUALCOMM_TSIF)	
	ret = mtv250_i2c_init();
	if(ret < 0)
	{
		DMBERR("RAONTV I2C driver registe failed\n");
		goto err_free_mem;
	}   
	
#elif defined(RTV_IF_SPI)	
	ret = mtv250_spi_init();
	if(ret < 0)
	{
		DMBERR("RAONTV SPI driver registe failed\n");
		goto err_free_mem;
	}

	/* Init tsp queue.*/
	mtv250_init_tsp_queue();

	ret = mtv250_create_tsp_pool(); 
	if(ret < 0)
	{
		DMBERR("RAONTV SPI TS buffer creation failed\n");
		goto err_free_mem;
	}	
#endif	
	
#if defined(RTV_IF_SPI) || (defined(RTV_TDMB_ENABLE) && !defined(RTV_TDMB_FIC_POLLING_MODE))	
	mtv250_cb_ptr->isr_thread_cb = NULL;	
	if ((ret=mtv250_isr_thread_run()) != 0) 
	{		
		DMBERR("[mtv250_power_on] mtv250_isr_thread_run() error\n");
		goto err_free_mem;
	}
	ret = request_irq(gpio_to_irq(RAONTV_IRQ_INT), mtv250_isr, IRQ_TYPE_EDGE_FALLING, RAONTV_DEV_NAME, NULL);	
	if (ret != 0) 
	{
		DMBERR("[mtv250_init_device] Failed to install irq (%d)\n", ret);
		mtv250_isr_thread_stop();
		goto err_free_mem;
	}
#endif	
		
	return 0;
	
err_free_mem:
	kfree(mtv250_cb_ptr);
	mtv250_cb_ptr = NULL;
   
	return ret;	
}


static ssize_t mtv250_read(struct file *filp, char *buf, size_t count, loff_t *pos)
{    	
	int ret = -ENODEV;
	
#if defined(RTV_IF_SPI)		
	int copy_bytes;
	unsigned int copy_offset;
	MTV250_TS_PKT_INFO *tsp = NULL;	
#ifdef MTV250_NON_BLOCKING_READ_MODE 
	ssize_t read_len = 0;
#else
	ssize_t read_len = count;
#endif

	if(count == 0)
	{
		DMBERR("[mtv250_read] Invalid length: %d.\n", count);
		return -EAGAIN;
	}

	do
	{
#ifndef MTV250_NON_BLOCKING_READ_MODE 	
		/* To release the read() of application as possible as soon, we use a queue count only. */
		ret = wait_event_interruptible(mtv250_cb_ptr->read_wq, 
							         mtv250_tsp_queue.cnt || mtv250_cb_ptr->stop); 	
		if(ret < 0)
		{
			mtv250_cb_ptr->stop = 1; // In case, ERESTARTSYS
			DMBERR("[mtv250_read] woken up fail: %d\n", ret);
			goto read_fail;
		}

		if(mtv250_cb_ptr->stop)
		{
			DMBMSG("[mtv250_read] Device stopped. q:%d, stop:%d\n", mtv250_tsp_queue.cnt, mtv250_cb_ptr->stop);			
			ret = -ENODEV;
			goto read_fail;
		}			
#endif		

		if(mtv250_cb_ptr->prev_tsp == NULL)
		{
			tsp = mtv250_get_tsp(); /* Get a new tsp from tsp_queue. */
			if(tsp == NULL)
			{
				ret = -EAGAIN;
#ifdef MTV250_NON_BLOCKING_READ_MODE 
				break; // Stop 
#else				
				DMBERR("[mtv250_read] Abnormal case\n");
				goto read_fail;
#endif				
			}
			
			copy_offset = 0;
			mtv250_cb_ptr->prev_tsp = tsp; // Save to use in next time if not finishded.	
			mtv250_cb_ptr->prev_org_tsp_size = tsp->len; 
		}
		else
		{
			tsp = mtv250_cb_ptr->prev_tsp;			
			copy_offset = mtv250_cb_ptr->prev_org_tsp_size - tsp->len;
		}

		copy_bytes = MIN(tsp->len, count);		

		ret = copy_to_user(buf, (&tsp->msc_buf[1] + copy_offset), copy_bytes);
		if (ret < 0) 
		{
			DMBERR("[mtv250_read] copy user fail: %d\n", ret);
			ret = -EFAULT;
			goto read_fail;
		}
		else
		{
			//DMBMSG("[mtv250_read] copy user ret: %d\n", ret);
#ifdef MTV250_NON_BLOCKING_READ_MODE 
			read_len += copy_bytes;
#endif
			buf        += copy_bytes;
			count     -= copy_bytes;
			tsp->len  -= copy_bytes;
			if(tsp->len == 0)
			{
				mtv250_free_tsp(tsp);  

				if(mtv250_cb_ptr->prev_tsp == tsp)
					mtv250_cb_ptr->prev_tsp = NULL; /* All used. */
			}
		}		
	} while(count != 0);

	return  read_len;

read_fail:	
	if(mtv250_cb_ptr->prev_tsp != NULL)
	{
		mtv250_free_tsp(mtv250_cb_ptr->prev_tsp);
		mtv250_cb_ptr->prev_tsp = NULL; 
	}

#ifndef MTV250_NON_BLOCKING_READ_MODE 
	/* Complete the read()*/
	if(mtv250_cb_ptr->stop)
	{		
		complete(&mtv250_cb_ptr->read_exit); //			
	}
#endif

#endif // #if defined(RTV_IF_SPI)		

	return ret;
}



#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,36)
static long mtv250_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
#else
static int mtv250_ioctl(struct inode *inode, struct file *filp,
				        unsigned int cmd,  unsigned long arg)
#endif
{
	int ret = 0;
	void __user *argp = (void __user *)arg;	
	RTV_IOCTL_REGISTER_ACCESS ioctl_register_access;	
	RTV_IOCTL_TEST_GPIO_INFO gpio_info;
	U8 reg_page = 0;
	UINT lock_mask;	
	UINT isdbt_ch_num;
	RTV_ISDBT_TMCC_INFO isdbt_tmcc_info;
	IOCTL_ISDBT_SIGNAL_INFO isdbt_signal_info;
	IOCTL_ISDBT_LGE_TUNER_INFO lge_tuner_info;

	switch( cmd )
	{
		case IOCTL_ISDBT_POWER_ON: // with adc clk type
			mtv250_cb_ptr->tv_mode = DMB_TV_MODE_1SEG;
			mtv250_cb_ptr->country_band_type = RTV_COUNTRY_BAND_JAPAN;
			
			DMBMSG("[mtv] IOCTL_ISDBT_POWER_ON:  country_band_type(%d)\n", mtv250_cb_ptr->country_band_type);
	
			ret = mtv250_power_on();
			if(ret  != 0)
			{
				ret = -EFAULT;		
				goto IOCTL_EXIT;
			}	
			break;

		case IOCTL_ISDBT_POWER_OFF:
			DMBMSG("[mtv] IOCTL_ISDBT_POWER_OFF\n");
			mtv250_power_off();
			break;

		case IOCTL_ISDBT_SCAN_FREQ:
			if (copy_from_user(&isdbt_ch_num, argp, sizeof(UINT)))
			{
				ret = -EFAULT;
				goto IOCTL_EXIT;
			}
			
			ret = rtvISDBT_ScanFrequency(isdbt_ch_num);
		//	DMBMSG("[mtv] ISDBT SCAN(%d) result: %d\n", isdbt_ch_num, ret);
			if(ret != RTV_SUCCESS)
			{
				ret = -EFAULT;	
				goto IOCTL_EXIT;
			}
			break;
			
		case IOCTL_ISDBT_SET_FREQ:
			if (copy_from_user(&isdbt_ch_num, argp, sizeof(UINT)))
			{
				ret = -EFAULT;
				goto IOCTL_EXIT;
			}
			
		//	DMBMSG("[mtv] IOCTL_ISDBT_SET_FREQ: %d\n", isdbt_ch_num);
			ret=rtvISDBT_SetFrequency(isdbt_ch_num);
			if(ret != RTV_SUCCESS)
			{
				DMBERR("[mtv] IOCTL_ISDBT_SET_FREQ error %d\n", ret);
				ret = -EFAULT;	
				goto IOCTL_EXIT;
			}
			break;		

		case IOCTL_ISDBT_GET_LOCK_STATUS:
		//	DMBMSG("[mtv] IOCTL_ISDBT_GET_LOCK_STATUS\n");
			lock_mask=rtvISDBT_GetLockStatus();
			if (copy_to_user(argp,&lock_mask, sizeof(UINT)))
			{
				ret = -EFAULT;		
				goto IOCTL_EXIT;
			}
			break;

		case IOCTL_ISDBT_GET_TMCC:		
		//	DMBMSG("[mtv] IOCTL_ISDBT_GET_TMCC\n");
			rtvISDBT_GetTMCC(&isdbt_tmcc_info);
			if (copy_to_user(argp,&isdbt_tmcc_info, sizeof(RTV_ISDBT_TMCC_INFO)))
			{
				ret = -EFAULT;		
				goto IOCTL_EXIT;
			}		
			break;

		case IOCTL_ISDBT_GET_SIGNAL_INFO:
		//	DMBMSG("[mtv] IOCTL_ISDBT_GET_SIGNAL_INFO\n");
			isdbt_signal_info.ber = rtvISDBT_GetBER(); 
			if(isdbt_signal_info.ber > 1500) 
				isdbt_signal_info.ber = 100000;
			
			isdbt_signal_info.cnr = rtvISDBT_GetCNR() / RTV_ISDBT_CNR_DIVIDER; 
			isdbt_signal_info.cnr = 10 * isdbt_signal_info.cnr;
			isdbt_signal_info.per = rtvISDBT_GetPER() * 3125; 
			isdbt_signal_info.rssi = rtvISDBT_GetRSSI() / RTV_ISDBT_RSSI_DIVIDER; 
			
			if (copy_to_user(argp, &isdbt_signal_info, sizeof(IOCTL_ISDBT_SIGNAL_INFO)))
			{
				ret = -EFAULT;		
				goto IOCTL_EXIT;
			}
			break;	

		case IOCTL_ISDBT_LGE_GET_TUNER_INFO:
		//	DMBMSG("[mtv] IOCTL_ISDBT_LGE_GET_TUNER_INFO\n");
	
			lge_tuner_info.LOCK = rtvISDBT_GetLockStatus() == RTV_ISDBT_CHANNEL_LOCK_OK ? 1: 0;

			lge_tuner_info.CNo = rtvISDBT_GetCNR() / RTV_ISDBT_CNR_DIVIDER; 
			lge_tuner_info.CNo = 10 * lge_tuner_info.CNo;
		
			lge_tuner_info.BER = rtvISDBT_GetBER(); 	
			if(lge_tuner_info.BER > 1500) 
				lge_tuner_info.BER = 100000;
				
			lge_tuner_info.PER = rtvISDBT_GetPER() * 3125; 

#if 1
			if(lge_tuner_info.LOCK == 0)
				lge_tuner_info.BER = lge_tuner_info.PER = 100000;
#endif
			
			lge_tuner_info.AGC= rtvISDBT_GetAGC(); 
			
			lge_tuner_info.RSSI= rtvISDBT_GetRSSI() / RTV_ISDBT_RSSI_DIVIDER; 

			// temp!!!!
			lge_tuner_info.start_ts_int_cnt = mtv250_cb_ptr->start_ts_int_cnt;
			lge_tuner_info.ovf_err_cnt = mtv250_cb_ptr->ovf_err_cnt;
			
			if (copy_to_user(argp, &lge_tuner_info, sizeof(IOCTL_ISDBT_LGE_TUNER_INFO)))
			{
				ret = -EFAULT;		
				goto IOCTL_EXIT;
			}
			break;

		case IOCTL_ISDBT_START_TS:			 
			RTV_GUARD_LOCK;
			mtv250_cb_ptr->start_ts_int_cnt = 0;
			mtv250_cb_ptr->ovf_err_cnt = 0;
			mtv250_cb_ptr->max_read_jiffies = 0;
			rtv_StreamEnable();
			RTV_GUARD_FREE;
			
			break;

		case IOCTL_ISDBT_STOP_TS:
			rtvISDBT_DisableStreamOut();
			mtv250_reset_tsp();
			break;

		case IOCTL_TEST_DMB_POWER_ON:	
			DMBMSG("[mtv250_ioctl] IOCTL_TEST_DMB_POWER_ON\n");
			
			RTV_MASTER_CHIP_SEL;
			rtvOEM_PowerOn(1);
			mtv250_cb_ptr->is_power_on = TRUE;
			break;

		case IOCTL_TEST_DMB_POWER_OFF:			
			RTV_MASTER_CHIP_SEL;
			rtvOEM_PowerOn(0);
			mtv250_cb_ptr->is_power_on = FALSE;
			break;
			
		case IOCTL_REGISTER_READ:
			if(mtv250_cb_ptr->is_power_on == FALSE)
			{			
				DMBMSG("[mtv] Power Down state!Must Power ON\n");
				ret = -EFAULT;
				goto IOCTL_EXIT;
			}
			
			if (copy_from_user(&ioctl_register_access, argp, sizeof(RTV_IOCTL_REGISTER_ACCESS)))
			{
				ret = -EFAULT;		
				goto IOCTL_EXIT;
			}		

			//DMBMSG("[mtv] IOCTL_REGISTER_READ: [%d] 0x%02X\n", ioctl_register_access.page, ioctl_register_access.Addr); 	

			switch( mtv250_cb_ptr->tv_mode )
			{
				case DMB_TV_MODE_TDMB:
				case DMB_TV_MODE_FM:
					switch( ioctl_register_access.page )
					{
						case 6: reg_page = 0x06; break; // OFDM
						case 7: reg_page = 0x09; break; // FEC
						case 8: reg_page = 0x0A; break; // FEC
						default: reg_page = mtv_reg_page_addr[ioctl_register_access.page];					
					}
					break;

				case DMB_TV_MODE_1SEG:
					switch( ioctl_register_access.page )
					{
						case 6: reg_page = 0x02; break; // OFDM
						case 7: reg_page = 0x03; break; // FEC
						default: reg_page = mtv_reg_page_addr[ioctl_register_access.page];					
					}
					break;
				default: break;
			}
					
			RTV_REG_MAP_SEL(reg_page); 
			ioctl_register_access.data[0] = RTV_REG_GET(ioctl_register_access.Addr);
			if (copy_to_user(argp, &ioctl_register_access, sizeof(RTV_IOCTL_REGISTER_ACCESS)))
			{
				ret = -EFAULT;		
				goto IOCTL_EXIT;
			}
			break;

		case IOCTL_REGISTER_WRITE:
			if(mtv250_cb_ptr->is_power_on == FALSE)
			{			
				DMBMSG("[mtv] Power Down state!Must Power ON\n");
				ret = -EFAULT;
				goto IOCTL_EXIT;
			}
			
			if (copy_from_user(&ioctl_register_access, argp, sizeof(RTV_IOCTL_REGISTER_ACCESS)))
			{
				ret = -EFAULT;		
				goto IOCTL_EXIT;
			}	

			switch( mtv250_cb_ptr->tv_mode )
			{
				case DMB_TV_MODE_TDMB:
				case DMB_TV_MODE_FM:
					switch( ioctl_register_access.page )
					{
						case 6: reg_page = 0x06; break; // OFDM
						case 7: reg_page = 0x09; break; // FEC
						case 8: reg_page = 0x0A; break; // FEC
						default: reg_page = mtv_reg_page_addr[ioctl_register_access.page];					
					}
					break;

				case DMB_TV_MODE_1SEG:
					switch( ioctl_register_access.page )
					{
						case 6: reg_page = 0x02; break; // OFDM
						case 7: reg_page = 0x03; break; // FEC
						default: reg_page = mtv_reg_page_addr[ioctl_register_access.page];					
					}
					break;
				default: break;
			}
					
			RTV_REG_MAP_SEL(reg_page); 
			RTV_REG_SET(ioctl_register_access.Addr, ioctl_register_access.data[0]);
			break;

		case IOCTL_TEST_GPIO_SET:
			if(mtv250_cb_ptr->is_power_on == FALSE)
			{			
				DMBMSG("[mtv] Power Down state!Must Power ON\n");
				ret = -EFAULT;
				goto IOCTL_EXIT;
			}
			
			if (copy_from_user(&gpio_info, argp, sizeof(RTV_IOCTL_TEST_GPIO_INFO)))
			{
				ret = -EFAULT;		
				goto IOCTL_EXIT;
			}	
			gpio_set_value(gpio_info.pin, gpio_info.value);
			break;

		case IOCTL_TEST_GPIO_GET:
			if(mtv250_cb_ptr->is_power_on == FALSE)
			{			
				DMBMSG("[mtv] Power Down state!Must Power ON\n");
				ret = -EFAULT;
				goto IOCTL_EXIT;
			}
			
			if (copy_from_user(&gpio_info, argp, sizeof(RTV_IOCTL_TEST_GPIO_INFO)))
			{
				ret = -EFAULT;		
				goto IOCTL_EXIT;
			}	
			
			gpio_info.value = gpio_get_value(gpio_info.pin);
			if(copy_to_user(argp, &gpio_info, sizeof(RTV_IOCTL_TEST_GPIO_INFO)))
			{
				ret = -EFAULT;		
				goto IOCTL_EXIT;
			}				
			break;
		
		default:
			DMBERR("[mtv] Invalid ioctl command: %d\n", cmd);
			ret = -ENOTTY;
			break;
	}

IOCTL_EXIT:

	return ret;
}



static int mtv250_open(struct inode *inode, struct file *filp)
{
	DMBMSG("[mtv250_open] called\n");

#if defined(RTV_IF_SPI)
   #ifndef MTV250_NON_BLOCKING_READ_MODE
	if( filp->f_flags & O_NONBLOCK )
	{
		DMBERR("[mtv250_read] Must open with Blocking I/O mode\n");
		return -EAGAIN;
	}
   #endif

	mtv250_start_read_function(); /* Can be used in read() function. */

	/* Init tsp read wait-queue and etc ...*/
	init_waitqueue_head(&mtv250_cb_ptr->read_wq); // Must init before poll().. open().	
	mtv250_tsp_queue.cnt = 0;
#endif

	return 0;
}

static int mtv250_close(struct inode *inode, struct file *filp)
{
	DMBMSG("\n[mtv250_close] called\n");

	mtv250_power_off();

#if defined(RTV_IF_SPI)	
	mtv250_exit_read_function();
#endif

	DMBMSG("[mtv250_close] END\n");

	return 0;
}


#if defined(RTV_IF_SPI)
// Wakeup to user poll() function
static unsigned int	mtv250_poll(struct file *file, poll_table *wait)
{
	unsigned int mask = 0;

	/* To release the poll() of application as possible as soon, we use a queue count only. */
	poll_wait(file, &mtv250_cb_ptr->read_wq, wait);

	if(mtv250_cb_ptr->stop)
	{
		mask |= POLLHUP;
	}

	if(mtv250_tsp_queue.cnt)
	{
		mask |= (POLLIN | POLLRDNORM);
	}

	return mask;
}
#endif

static struct file_operations mtv250_fops =
{
    .owner = THIS_MODULE,
    .open = mtv250_open,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,36)
	.unlocked_ioctl = mtv250_ioctl,
#else
	.ioctl = mtv250_ioctl,
#endif    
    .read = mtv250_read,
#if defined(RTV_IF_SPI)    
    .poll = mtv250_poll,
#endif    
    .release = mtv250_close,
};

static struct miscdevice mtv250_misc_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = RAONTV_DEV_NAME,
    .fops = &mtv250_fops,
};


static const char *build_data = __DATE__;
static const char *build_time = __TIME__;


static int __init mtv250_module_init(void)
{
    	int ret;

	DMBMSG("\t==========================================================\n");
	DMBMSG("\t %s Module Build Date/Time: %s, %s\n", mtv250_misc_device.name, build_data, build_time);
	DMBMSG("\t==========================================================\n\n");
	
    ret = mtv250_init_device();
	if(ret<0)
		return ret;

	/* misc device registration */
	ret = misc_register(&mtv250_misc_device);
	if( ret )
	{
		DMBERR("[mtv250_module_init] misc_register() failed! : %d", ret);
		return ret; 	  	
	}

	return 0;
}


static void __exit mtv250_module_exit(void)
{
	mtv250_deinit_device();
	
	misc_deregister(&mtv250_misc_device);
}

module_init(mtv250_module_init);
module_exit(mtv250_module_exit);
MODULE_DESCRIPTION("MTV250 driver");
MODULE_LICENSE("GPL");

