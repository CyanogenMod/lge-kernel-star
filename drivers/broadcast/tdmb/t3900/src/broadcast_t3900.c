#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/pm.h>

#include <linux/spi/spi.h>         /* HOST Interface SPI */
#include <linux/spi/spidev.h>   /* HOST Interface SPI */
#include <linux/interrupt.h>        /* HOST Interface SPI */

#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/wakelock.h> 		/* wake_lock, unlock */

#include <linux/lge_hw_rev.h> 

#include "../../broadcast_tdmb_drv_ifdef.h"
#include "../inc/broadcast_t3900.h"


#define DELAY_USING_WAIT_EVENT_TIMEOUT  /* wait_event_timeout instead of msleep */
#ifdef DELAY_USING_WAIT_EVENT_TIMEOUT
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/jiffies.h>
#endif

/* T3900 driver contrl block */
struct tdmb_t3900_ctrl_blk 
{
	int 					is_power_on;
	struct spi_device* 			spi_ptr;
	struct work_struct 			spi_work;
	struct workqueue_struct* 		spi_wq;
	struct mutex				mutex;
	struct wake_lock 			wake_lock;	/* wake_lock,wake_unlock */
	boolean 				_status;
	spinlock_t				spin_lock;
};

static struct tdmb_t3900_ctrl_blk t3900_ctrl_info;


// -------------------------------------------------------------------- 
// SU660 DMB GPIOs
#define DMB_EN			38
#define DMB_INT			37
#define DMB_RESET		36
#define DMB_SPI3_SDI	10
#define DMB_SPI3_SDO	22
#define DMB_SPI3_CS		176
#define DMB_SPI3_CLK	203


static uint32 user_stop_flg = 0;
static const char *id = "TDMB";
static struct msm_xo_voter *xo_handle_tdmb;

/* ============================================================== */
/*  Internal Functions                                                                                                                */
/* ============================================================== */
static int broadcast_t3900_probe(struct spi_device *spi);
static int broadcast_t3900_remove(struct spi_device *spi);
static int broadcast_t3900_suspend(struct spi_device *spi, pm_message_t mesg);
static int broadcast_t3900_resume(struct spi_device *spi);
static irqreturn_t broadcast_tdmb_spi_isr(int irq, void *handle);

void tdmb_t3900_set_userstop(int mode)
{
	user_stop_flg = mode;
}

int tdmb_t3900_mdelay(int32 ms)
{
	int		rc = 1;  /* 0 : false, 1 : ture */
	int32	wait_loop =0;
	int32	wait_ms = ms;

	if(ms > 100)
	{
		wait_loop = (ms /100);   /* 100, 200, 300 more only , Otherwise this must be modified e.g (ms + 40)/50 */
		wait_ms = 100;
	}

	do
	{
		mdelay(wait_ms);
		if(user_stop_flg == 1)
		{
			printk("~~~~~~~~ Ustop flag is set so return false ms =(%d)~~~~~~~\n", ms);
			rc = 0;
			break;
		}
	}while((--wait_loop) > 0);

	return rc;
}

void tdmb_t3900_must_mdelay(int32 ms)
{
	mdelay(ms);
}

int tdmb_t3900_power_on(void)
{
	int rc;

	if ( t3900_ctrl_info.is_power_on == FALSE )
	{
		wake_lock(&t3900_ctrl_info.wake_lock);

		if(REV_E <= get_lge_pcb_revision())
		{
			gpio_direction_input(DMB_INT);
			gpio_direction_output(DMB_RESET, false);
			gpio_direction_output(DMB_EN, true);
			
			gpio_set_value(DMB_EN, 1);
			udelay(1000); //500us
			gpio_set_value(DMB_RESET, 1);
			udelay(1000);
			gpio_set_value(DMB_RESET, 0);		
			udelay(1000);
			gpio_set_value(DMB_RESET, 1);
			udelay(5); //500us
		}
		else
		{
			gpio_direction_input(118);
			gpio_direction_output(119, false);
			gpio_direction_output(43, true);
			
			gpio_set_value(43, 1);
			udelay(1000); //500us
			gpio_set_value(119, 1);
			udelay(1000);
			gpio_set_value(119, 0);		
			udelay(1000);
			gpio_set_value(119, 1);
			udelay(5); //500us	
		}	

		rc = request_irq(t3900_ctrl_info.spi_ptr->irq, broadcast_tdmb_spi_isr, IRQF_DISABLED | IRQF_TRIGGER_FALLING, t3900_ctrl_info.spi_ptr->dev.driver->name, &t3900_ctrl_info);

		tdmb_t3900_interrupt_free();
		t3900_ctrl_info.is_power_on = TRUE;

	}
	else
	{
		printk("tdmb_t3900_power_on the power already turn on \n");
	}

	printk("tdmb_t3900_power_on completed \n");

	return TRUE;

}
int tdmb_t3900_power_off(void)
{
	if ( t3900_ctrl_info.is_power_on == TRUE )
	{
		tdmb_t3900_interrupt_lock();
		t3900_ctrl_info.is_power_on = FALSE;

		if(REV_E <= get_lge_pcb_revision())
		{
			gpio_set_value(DMB_RESET, 0);
			gpio_set_value(DMB_EN, 0);
		}
		else
		{
			gpio_set_value(119, 0);
			gpio_set_value(43, 0);
		}
		free_irq(t3900_ctrl_info.spi_ptr->irq, &t3900_ctrl_info);

		wake_unlock(&t3900_ctrl_info.wake_lock);
	}
	else
	{
		printk("tdmb_t3900_power_on the power already turn off \n");
	}	

	printk("tdmb_t3900_power_off completed \n");
		
	return TRUE;
}

//modified by wonhee.jeong@lge.com 20110404 for B_Qwerty TDMB Porting(Start)	
static struct spi_driver broadcast_tdmb_driver = {
	.probe = broadcast_t3900_probe,
	.remove = __devexit_p(broadcast_t3900_remove),
	.suspend = broadcast_t3900_suspend,
	.resume  = broadcast_t3900_resume,
	.driver = {
		.name = "tdmb_t3900",
		.bus	= &spi_bus_type,
		.owner = THIS_MODULE,
	},
};

int tdmb_t3900_select_antenna(unsigned int sel)
{
	return FALSE;
}

void tdmb_t3900_interrupt_lock(void)
{
	if (t3900_ctrl_info.spi_ptr == NULL)
	{
		printk("tdmb_t3900_interrupt_lock fail\n");
		return;
	}

	disable_irq(t3900_ctrl_info.spi_ptr->irq);
	return;
}

void tdmb_t3900_interrupt_free(void)
{
	if (t3900_ctrl_info.spi_ptr == NULL)
	{
		printk("tdmb_t3900_interrupt_free fail\n");
		return;
	}

	enable_irq(t3900_ctrl_info.spi_ptr->irq);
	return;
}


int tdmb_t3900_spi_write_read(uint8* tx_data, int tx_length, uint8 *rx_data, int rx_length)
{
	int rc;
	
	struct spi_transfer	t = {
			.tx_buf		= tx_data,
			.rx_buf		= rx_data,
			.len			= tx_length+rx_length,
		};
	struct spi_message	m;	

	//printk("tdmb_t3900_spi_write_read start\n");

	if (t3900_ctrl_info.spi_ptr == NULL)
	{
		printk("tdmb_t3900_spi_write_read error txdata=0x%x, length=%d\n", (unsigned int)tx_data, tx_length+rx_length);
		return FALSE;
	}

	mutex_lock(&t3900_ctrl_info.mutex);

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	rc = spi_sync(t3900_ctrl_info.spi_ptr, &m);
	//printk("rc = %d in tdmb_t3900_spi_write_read\n", rc);

	if ( rc < 0 )
	{
		printk("tdmb_t3900_spi_read_burst result(%d), actual_len=%d\n",rc, m.actual_length);
	}

	mutex_unlock(&t3900_ctrl_info.mutex);


	return TRUE;
}


#if 0
void tdmb_rw_test(void)
{
	unsigned short i = 0;
	unsigned short w_val = 0;
	unsigned short r_val = 0;
	unsigned short err_cnt = 0;

	err_cnt = 0;
	for(i=1;i<11;i++)
	{
		w_val = (i%0xFF);
		tdmb_t3900_spi_write_read( 0x0a00+ 0x05, w_val, 0x0a00+ 0x05, &r_val );
		//tdmb_t3900_i2c_read16(0x0a00+ 0x05, &r_val );
		if(r_val != w_val)
		{
			err_cnt++;
			printk("w_val:%x, r_val:%x\n", w_val,r_val);
		}
	}
}
#endif

static irqreturn_t broadcast_tdmb_spi_isr(int irq, void *handle)
{
	struct tdmb_t3900_ctrl_blk* t3900_info_p;
	unsigned long flag;

	t3900_info_p = (struct tdmb_t3900_ctrl_blk *)handle;	
	if ( t3900_info_p && t3900_info_p->is_power_on )
	{
		if (t3900_info_p->_status)
		{			
			printk("######### T3900 spi read funtion is so late skip #########\n");			
			return IRQ_HANDLED;
		}
		queue_work(t3900_info_p->spi_wq, &t3900_info_p->spi_work);
	}
	else
	{
		printk("broadcast_tdmb_spi_isr is called, but device is off state\n");
	}

	return IRQ_HANDLED; 
}

static void broacast_tdmb_spi_work(struct work_struct *tdmb_work)
{
	struct tdmb_t3900_ctrl_blk *t3900_info_p;

	t3900_info_p = container_of(tdmb_work, struct tdmb_t3900_ctrl_blk, spi_work);
	if ( t3900_info_p )
	{
		t3900_info_p->_status = TRUE;
		broadcast_drv_if_read_data();
		t3900_info_p->_status = FALSE;
	}
}

static int tdmb_t3900_configure_gpios(void)
{
#if 0
	int rc= 0;
rc = gpio_request(DMB_EN, "dmb_enable"); if(rc)
	{
		pr_warning("T3900 DMB_EN gpio request error\n");
	}
	gpio_direction_output(DMB_EN, 0);  /* output and low */

	rc = gpio_request(DMB_RESET_N, "dmb_reset");
	if(rc)
	{
		pr_warning("T3900 DMB_RESET gpio request error\n");
	}
	gpio_direction_output(DMB_RESET_N, 0);  /* output and low */

	rc = gpio_request(DMB_INT_N, "dmb_");
	if(rc)
	{
		pr_warning("T3900 DMB_INT_N gpio request error\n");
	}
	gpio_direction_input(DMB_INT_N);

	return 0;
#endif
}


static int  broadcast_t3900_probe(struct spi_device *spi)
{
	int rc;
	int ret1, ret2, ret3;

	t3900_ctrl_info.spi_ptr 				= spi;
	t3900_ctrl_info.spi_ptr->mode 			= SPI_MODE_0;
	t3900_ctrl_info.spi_ptr->bits_per_word 	= 8;
	t3900_ctrl_info.spi_ptr->max_speed_hz 	= (5000*1000);

	if(REV_E <= get_lge_pcb_revision())
		t3900_ctrl_info.spi_ptr->irq = TEGRA_GPIO_TO_IRQ(37);//TEGRA_GPIO_PE5
	else
		t3900_ctrl_info.spi_ptr->irq = TEGRA_GPIO_TO_IRQ(118);//TEGRA_GPIO_PO6

	rc = spi_setup(spi);
	printk("%s is called and spi_setup\n", "broadcast_t3900_probe");

	INIT_WORK(&t3900_ctrl_info.spi_work, broacast_tdmb_spi_work);

	t3900_ctrl_info.spi_wq = create_singlethread_workqueue("tdmb_spi_wq");
	// t3900_ctrl_info.spi_wq = create_rt_workqueue("tdmb_spi_wq");
	if(t3900_ctrl_info.spi_wq == NULL){
		printk("Failed to setup tdmb spi workqueue \n");
		return -ENOMEM;
	}

	if(REV_E <= get_lge_pcb_revision())
	{
		ret1 = gpio_request(DMB_EN, "dmb enable");
		ret2 = gpio_request(DMB_RESET, "dmb reset");
		ret3 = gpio_request(DMB_INT, "dmb interrupt" );

		if ((ret1 < 0) || (ret2 < 0) || (ret3 < 0))
			printk("%d, %d,%d", ret1, ret2, ret3);

		ret1 = gpio_direction_output(DMB_EN, 0);
		ret2 = gpio_direction_output(DMB_RESET, 0);
	}
	else 
	{
		ret1 = gpio_request(43, "dmb enable");
		ret2 = gpio_request(119, "dmb reset");
		ret3 = gpio_request(118, "dmb interrupt" );

		if ((ret1 < 0) || (ret2 < 0) || (ret3 < 0))
			printk("%d, %d,%d", ret1, ret2, ret3);

		ret1 = gpio_direction_output(43, 0);
		ret2 = gpio_direction_output(119, 0);
		ret3 = gpio_direction_input(118);
	}

	tdmb_t3900_interrupt_lock();

	mutex_init(&t3900_ctrl_info.mutex);

	wake_lock_init(&t3900_ctrl_info.wake_lock,  WAKE_LOCK_SUSPEND, dev_name(&spi->dev));

	spin_lock_init(&t3900_ctrl_info.spin_lock);

	printk("broadcast_t3900_probe End. \n");
	
	return rc;
}

static int broadcast_t3900_remove(struct spi_device *spi)
{
	if (t3900_ctrl_info.spi_wq)
	{
		flush_workqueue(t3900_ctrl_info.spi_wq);
		destroy_workqueue(t3900_ctrl_info.spi_wq);
	}

	free_irq(spi->irq, &t3900_ctrl_info);
	mutex_destroy(&t3900_ctrl_info.mutex);
	wake_lock_destroy(&t3900_ctrl_info.wake_lock);

	memset((unsigned char*)&t3900_ctrl_info, 0x0, sizeof(struct tdmb_t3900_ctrl_blk));
	return 0;
}


static int broadcast_t3900_suspend(struct spi_device *spi, pm_message_t mesg)
{
	printk("broadcast_t3900_suspend \n");
	return 0;
}

static int broadcast_t3900_resume(struct spi_device *spi)
{
	printk("broadcast_t3900_resume \n");
	return 0;
}

int __devinit broadcast_tdmb_drv_init(void)
{
	int rc;
	//printk("broadcast_tdmb_drv_init\n");
	printk("%s is called\n", "broadcast_tdmb_drv_init");
	rc = broadcast_tdmb_drv_start();
	
	if (rc) {
		printk("broadcast_tdmb_drv_start %s failed to load\n", __func__);
		return rc;
	}
	rc = spi_register_driver(&broadcast_tdmb_driver);
	printk("broadcast_spi_add_driver rc = (%d)\n", rc);

	return rc;
}

static void __exit broadcast_tdmb_drv_exit(void)
{
	spi_unregister_driver(&broadcast_tdmb_driver);
}


/* EXPORT_SYMBOL() : when we use external symbol 
which is not included in current module - over kernel 2.6 */
//EXPORT_SYMBOL(broadcast_tdmb_is_on);

module_init(broadcast_tdmb_drv_init);
module_exit(broadcast_tdmb_drv_exit);
MODULE_DESCRIPTION("broadcast_tdmb_drv_init");
MODULE_LICENSE("INC");
