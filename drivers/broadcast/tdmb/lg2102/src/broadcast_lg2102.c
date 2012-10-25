#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#include <linux/interrupt.h>

#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/wakelock.h> 		/* wake_lock, unlock */

#include "../../broadcast_tdmb_drv_ifdef.h"
#include "../inc/broadcast_lg2102.h"
#include "../inc/tdmb_tunerbbdrv_lg2102def.h"

// PM QOS
#include <linux/pm_qos_params.h>


/* proto type declare */
static int broadcast_tdmb_lg2102_probe(struct spi_device *spi);
static int broadcast_tdmb_lg2102_remove(struct spi_device *spi);
static int broadcast_tdmb_lg2102_suspend(struct spi_device *spi, pm_message_t mesg);
static int broadcast_tdmb_lg2102_resume(struct spi_device *spi);

extern void tunerbb_drv_lg2102_rw_test(void);

#define DMB_EN 			143 	//GPIO PR7 = 143
#define DMB_INT_N		118		//GPIO PO6 = 118
#define DMB_RESET_N		119 	//GPIO PO7 = 119
#define DMB_EARANT 		191 	//GPIO PX7 = 191
#define DMB_SPI2_SDI	184 	//SPI2_MOSI, GPIO PX0 = 184
#define DMB_SPI2_SDO	185  	//SPI2_MISO, GPIO PX1 = 185
#define DMB_SPI2_CLK	186 	//SPI2_SCK, GPIO PX2 =186
#define DMB_SPI2_CS		187 	//SPI2_CS0_N, GPIO PX3 =187

/************************************************************************/
/* LINUX Driver Setting                                                 */
/************************************************************************/
static uint32 user_stop_flg = 0;
struct tdmb_lg2102_ctrl_blk
{
	boolean                             TdmbPowerOnState;
	struct spi_device*          	    spi_ptr;
	struct work_struct        	        spi_work;
	struct workqueue_struct*         	spi_wq;
	struct mutex                        mutex;
	struct wake_lock                    wake_lock;	/* wake_lock,wake_unlock */
	boolean 	                        spi_irq_status;
	spinlock_t                          spin_lock;
	struct pm_qos_request_list    	pm_req_list;	
};

static struct tdmb_lg2102_ctrl_blk lg2102_ctrl_info;


void tdmb_lg2102_set_userstop(int mode)
{
	user_stop_flg = mode;
}

int tdmb_lg2102_select_antenna(int mode)
{
	if ( lg2102_ctrl_info.TdmbPowerOnState == TRUE )
	{
		if(mode == 0) //default antenna
		{
			gpio_set_value(DMB_EARANT, 0);
			printk("EAR ANTENNA SET VALUE (0)\n" );
	        return TRUE;
		}
		else if(mode == 1)//ear anteena
		{
			gpio_set_value(DMB_EARANT, 1);
			printk("EAR ANTENNA SET VALUE (1)\n" );
	        return TRUE;
		}
		else
		{
			/*error mode*/
		}
	}

	return FALSE;
	
}


int tdmb_lg2102_mdelay(int32 ms)
{
	int32	wait_loop =0;
	int32	wait_ms = ms;
	int		rc = 1;  /* 0 : false, 1 : ture */

	if(ms > 100)
	{
		/* 100, 200, 300 more only , Otherwise this must be modified e.g (ms + 40)/50 */
		wait_loop = (ms /100);   
		wait_ms = 100;
	}

	do
	{
		msleep(wait_ms);
		if(user_stop_flg == 1)
		{
			printk("~~~~~~~~ Ustop flag is set so return false ms =(%d)~~~~~~~\n", ms);
			rc = 0;
			break;
		}
	}while((--wait_loop) > 0);

	return rc;
}

void tdmb_lg2102_must_mdelay(int32 ms)
{
	mdelay(ms);
}

int tdmb_lg2102_power_on(void)
{
	printk("lg2102_ctrl_info.TdmbPowerOnState = (%d)\n", lg2102_ctrl_info.TdmbPowerOnState);
	
	if ( lg2102_ctrl_info.TdmbPowerOnState == FALSE )
	{
		/* Qos */
		if(pm_qos_request_active(&lg2102_ctrl_info.pm_req_list)) {
			pm_qos_update_request(&lg2102_ctrl_info.pm_req_list, 20);
		}
		
		wake_lock(&lg2102_ctrl_info.wake_lock);
	
		gpio_direction_input(DMB_INT_N);
		gpio_direction_output(DMB_RESET_N, false);
		gpio_direction_output(DMB_EN, true);
		
	    gpio_set_value(DMB_EN, 1);
		udelay(1000); //1ms
				
		gpio_set_value(DMB_RESET_N, 1);
		udelay(1000); //1ms
		
		gpio_set_value(DMB_RESET_N, 0);
		udelay(1000); //1ms
		
		gpio_set_value(DMB_RESET_N, 1);
		udelay(1000); //1ms

		tdmb_lg2102_interrupt_free();
		lg2102_ctrl_info.TdmbPowerOnState = TRUE;
	}
	else
	{
		printk("tdmb_lg2102_power_on the power already turn on \n");
	}

	printk("tdmb_lg2102_power_on completed \n");

	return TRUE;
}

int tdmb_lg2102_power_off(void)
{
	if ( lg2102_ctrl_info.TdmbPowerOnState == TRUE )
	{
		tdmb_lg2102_interrupt_lock();
		lg2102_ctrl_info.TdmbPowerOnState = FALSE;

		gpio_set_value(DMB_RESET_N, 0);
		gpio_set_value(DMB_EN, 0);
		gpio_set_value(DMB_EARANT, 0);

		wake_unlock(&lg2102_ctrl_info.wake_lock);

		/* Qos release */
		if(pm_qos_request_active(&lg2102_ctrl_info.pm_req_list)) {
			pm_qos_update_request(&lg2102_ctrl_info.pm_req_list, PM_QOS_DEFAULT_VALUE);	
		}
	}
	else
	{
		printk("tdmb_lg2102_power_on the power already turn off \n");
	}	

	printk("tdmb_lg2102_power_off completed \n");
	
	return TRUE;
}

static struct spi_driver broadcast_tdmb_driver = {
	.probe = broadcast_tdmb_lg2102_probe,
	.remove	= __devexit_p(broadcast_tdmb_lg2102_remove),
	.suspend = broadcast_tdmb_lg2102_suspend,
	.resume  = broadcast_tdmb_lg2102_resume,
	.driver = {
		.name = "tdmb_lg2102",
		.bus	= &spi_bus_type,
		.owner = THIS_MODULE,
	},
};

void tdmb_lg2102_interrupt_lock(void)
{
	if (lg2102_ctrl_info.spi_ptr == NULL)
	{
		printk("tdmb_lg2102_interrupt_lock fail\n");
		return;
	}

	disable_irq(lg2102_ctrl_info.spi_ptr->irq);
	return;
}

void tdmb_lg2102_interrupt_free(void)
{
	if (lg2102_ctrl_info.spi_ptr == NULL)
	{
		printk("tdmb_lg2102_interrupt_free fail\n");
		return;
	}

	enable_irq(lg2102_ctrl_info.spi_ptr->irq);
	return;
}

int tdmb_lg2102_spi_write_read(uint8* tx_data, int tx_length, uint8 *rx_data, int rx_length)
{
	int rc;
	
	struct spi_transfer	t = {
			.tx_buf		= tx_data,
			.rx_buf		= rx_data,
			.len			= tx_length+rx_length,
		};
	struct spi_message	m;	

	if (lg2102_ctrl_info.spi_ptr == NULL)
	{
		printk("tdmb_lg2102_spi_write_read error txdata=0x%x, length=%d\n", (unsigned int)tx_data, tx_length+rx_length);
		return FALSE;
	}

	mutex_lock(&lg2102_ctrl_info.mutex);

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	rc = spi_sync(lg2102_ctrl_info.spi_ptr, &m);
	if ( rc < 0 )
	{
		printk("tdmb_lg2102_spi_read_burst result(%d), actual_len=%d\n",rc, m.actual_length);
	}

	mutex_unlock(&lg2102_ctrl_info.mutex);

	return TRUE;
}

static irqreturn_t broadcast_tdmb_spi_isr(int irq, void *handle)
{
	struct tdmb_lg2102_ctrl_blk* pTdmbInfo;
	unsigned long flag;

	pTdmbInfo = (struct tdmb_lg2102_ctrl_blk *)handle;	
	
	if ( pTdmbInfo && pTdmbInfo->TdmbPowerOnState )
	{
		if (pTdmbInfo->spi_irq_status)
		{
			printk("########### DMB SPI ISR but funtion is so late skip ###########\n");
			return IRQ_HANDLED;
		}
		spin_lock_irqsave(&pTdmbInfo->spin_lock, flag);
		queue_work(pTdmbInfo->spi_wq, &pTdmbInfo->spi_work);
		spin_unlock_irqrestore(&pTdmbInfo->spin_lock, flag);
	}
	else
	{
		printk("broadcast_tdmb_spi_isr is called, but device is off state\n");
	}

	return IRQ_HANDLED; 
}

static void broacast_tdmb_spi_work(struct work_struct *tdmb_work)
{
	struct tdmb_lg2102_ctrl_blk *pTdmbWorkData;

	pTdmbWorkData = container_of(tdmb_work, struct tdmb_lg2102_ctrl_blk, spi_work);
	if ( pTdmbWorkData )
	{
		pTdmbWorkData->spi_irq_status = TRUE;
		broadcast_drv_if_read_data();
		pTdmbWorkData->spi_irq_status = FALSE;
	}
}

static int broadcast_tdmb_lg2102_probe(struct spi_device *spi)
{
	int rc;

	lg2102_ctrl_info.spi_ptr 					= spi;
	lg2102_ctrl_info.spi_ptr->mode 			= SPI_MODE_0;
	lg2102_ctrl_info.spi_ptr->bits_per_word 	= 8;
	lg2102_ctrl_info.spi_ptr->max_speed_hz 	= (6000*1000);

	lg2102_ctrl_info.spi_ptr->irq = TEGRA_GPIO_TO_IRQ(118);
	
	rc = spi_setup(spi);
	
	INIT_WORK(&lg2102_ctrl_info.spi_work, broacast_tdmb_spi_work);
	lg2102_ctrl_info.spi_wq = create_singlethread_workqueue("tdmb_spi_wq");
	if(lg2102_ctrl_info.spi_wq == NULL){
		printk("Failed to setup tdmb spi workqueue \n");          
	}

	gpio_request(DMB_RESET_N, "dmb reset");
	gpio_request(DMB_EN, "dmb enable");
	gpio_request(DMB_INT_N, "dmb interrupt");
	gpio_request(DMB_EARANT, "dmb earantenna");

	tegra_gpio_enable(DMB_INT_N);
	tegra_gpio_enable(DMB_RESET_N);
	tegra_gpio_enable(DMB_EN);
	tegra_gpio_enable(DMB_EARANT);

	// Setting the ON/OFF pin to output mode and setting to Low.
    gpio_direction_output(DMB_RESET_N, 0);
	gpio_direction_output(DMB_EN, 0);
	gpio_direction_output(DMB_EARANT, 0);

	gpio_set_value(DMB_RESET_N, 0);
	gpio_set_value(DMB_EN, 0);
	gpio_set_value(DMB_EARANT, 0);

	rc = request_irq(lg2102_ctrl_info.spi_ptr->irq, broadcast_tdmb_spi_isr,  IRQF_DISABLED|IRQF_TRIGGER_FALLING , lg2102_ctrl_info.spi_ptr->dev.driver->name, &lg2102_ctrl_info);
	printk("broadcast_tdmb_lg2102_probe request_irq=%d\n", rc);
			
	tdmb_lg2102_interrupt_lock();

	mutex_init(&lg2102_ctrl_info.mutex);
	wake_lock_init(&lg2102_ctrl_info.wake_lock,  WAKE_LOCK_SUSPEND, dev_name(&spi->dev));

	spin_lock_init(&lg2102_ctrl_info.spin_lock);
	pm_qos_add_request(&lg2102_ctrl_info.pm_req_list, PM_QOS_CPU_DMA_LATENCY, PM_QOS_DEFAULT_VALUE);

	printk("[lg2102_probe] probe complete");
	
	return rc;
}

static int broadcast_tdmb_lg2102_remove(struct spi_device *spi)
{
	printk("broadcast_tdmb_lg2102_remove \n");

	if (lg2102_ctrl_info.spi_wq)
	{
		flush_workqueue(lg2102_ctrl_info.spi_wq);
		destroy_workqueue(lg2102_ctrl_info.spi_wq);
	}

	free_irq(spi->irq, &lg2102_ctrl_info);
	mutex_destroy(&lg2102_ctrl_info.mutex);

	wake_lock_destroy(&lg2102_ctrl_info.wake_lock);

	pm_qos_remove_request(&lg2102_ctrl_info.pm_req_list);

	memset((unsigned char*)&lg2102_ctrl_info, 0x0, sizeof(struct tdmb_lg2102_ctrl_blk));
	return 0;
}

static int broadcast_tdmb_lg2102_suspend(struct spi_device *spi, pm_message_t mesg)
{
	printk("broadcast_tdmb_lg2102_suspend \n");
	return 0;
}

static int broadcast_tdmb_lg2102_resume(struct spi_device *spi)
{
	printk("broadcast_tdmb_lg2102_resume \n");
	return 0;
}

int __devinit broadcast_tdmb_drv_init(void)
{
	int rc;

	rc = broadcast_tdmb_drv_start();
	printk("broadcast_tdmb_lg2102_probe start %d\n", rc);

	return spi_register_driver(&broadcast_tdmb_driver);
}

static void __exit broadcast_tdmb_drv_exit(void)
{
	spi_unregister_driver(&broadcast_tdmb_driver);
}

int broadcast_tdmb_is_on(void)
{
	return (int)lg2102_ctrl_info.TdmbPowerOnState;
}

EXPORT_SYMBOL(broadcast_tdmb_is_on);

module_init(broadcast_tdmb_drv_init);
module_exit(broadcast_tdmb_drv_exit);
MODULE_DESCRIPTION("broadcast_tdmb_drv_init");
MODULE_LICENSE("INC");

