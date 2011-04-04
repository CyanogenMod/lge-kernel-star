

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
//#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#include <linux/interrupt.h>

#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/wakelock.h> 		/* wake_lock, unlock */
#include <linux/broadcast/broadcast_lg2102.h>
#include <linux/broadcast/broadcast_tdmb.h>
#include <linux/broadcast/board_broadcast.h>

/* external function */
extern void broadcast_tdmb_read_data(void);

/* proto type declare */
static int broadcast_tdmb_lg2102_probe(struct spi_device *spi);
static int broadcast_tdmb_lg2102_remove(struct spi_device *spi);
static int broadcast_tdmb_lg2102_suspend(struct spi_device *spi, pm_message_t mesg);
static int broadcast_tdmb_lg2102_resume(struct spi_device *spi);

//#define DMB_EN 			28 //GPIO28
//#define DMB_INT_N 		29 //GPIO29
//#define DMB_RESET_N 	62 //GPIO62

#define TDMB_PORT_GUID (NV_ODM_GUID('s','p','i','_','t','d','m','b'))

unsigned int dmb_gpio_port[4];
unsigned int dmb_gpio_pin[4];

//temp hardcoding port/pin
#define DMB_RESET_PORT 		'o' - 'a'
#define DMB_ENABLE_PORT 	'r' - 'a'
#define DMB_EARANT_PORT 	'x' - 'a'
#define DMB_IRQ_PORT 		'o' - 'a'

#define DMB_RESET_PIN 	7
#define DMB_ENABLE_PIN 	7
#define DMB_EARANT_PIN 	7
#define DMB_IRQ_PIN 	6

static NvOdmGpioPinHandle hDmbResetPin;
static NvOdmGpioPinHandle hDmbEnPin;
static NvOdmGpioPinHandle hDmbIntPin;
static NvOdmGpioPinHandle hDmbEarAntPin;


/************************************************************************/
/* LINUX Driver Setting                                                 */
/************************************************************************/
static uint32 user_stop_flg = 0;
static uint32 mdelay_in_flg = 0;
struct TDMB_LG2102_CTRL
{
	boolean 					TdmbPowerOnState;
	struct i2c_client*			pClient;
	struct spi_device* 		pSpiDevice;
	struct work_struct 		spi_work;
	struct workqueue_struct* 	spi_wq;
	struct mutex				mutex;
	struct wake_lock 			wake_lock;	/* wake_lock,wake_unlock */
	//test
	boolean 					spi_irq_status;
};

//static broadcast_pwr_func pwr_func;

static struct TDMB_LG2102_CTRL TdmbCtrlInfo;

struct i2c_client* INC_GET_I2C_DRIVER(void)
{
	return TdmbCtrlInfo.pClient;
}

void LGD_RW_TEST(void);


void tdmb_lg2102_set_userstop(void)
{
	user_stop_flg = ((mdelay_in_flg == 1)? 1: 0 );
}

void tdmb_lg2102_select_antenna(int mode)
{
	NvOdmServicesGpioHandle hGpio;

	if ( TdmbCtrlInfo.TdmbPowerOnState == TRUE )
	{
		 hGpio = NvOdmGpioOpen();
		if (!hGpio)
		{
			printk("tdmb_lg2102_select_antenna GPIO Open fail \n");
			return;
		}
		 
		if(mode == 0) //default antenna
		{
			NvOdmGpioSetState(hGpio, hDmbEarAntPin, 0);
		}
		else if(mode == 1)//ear anteena
		{
			NvOdmGpioSetState(hGpio, hDmbEarAntPin, 1);
		}
		else
		{
			/*error mode*/
		}
	}

	NvOdmGpioClose(hGpio);

	return;
	
}


int tdmb_lg2102_mdelay(int32 ms)
{
#if 0

	int rc = 1;

	printk("[LG2102] mdelay (IN) ms = (%d)\n", ms);
	mdelay_in_flg = 1;
	msleep(ms);
	if(user_stop_flg == 1)
	{
		rc = 0;
		printk("~~~ tdmb_lg2102_mdelay user stop ~~~~ rc = (%d)\n", rc);
	}
	mdelay_in_flg = 0;
	user_stop_flg = 0;
	printk("[LG2102]mdelay(OUT) ms = (%d)\n", ms);
#else
	int32 wait_loop =0;
	int32 wait_ms = ms;
	int       rc = 1;  /* 0 : false, 1 : ture */

	if(ms > 100)
	{
		wait_loop = (ms /100);   /* 100, 200, 300 more only , Otherwise this must be modified e.g (ms + 40)/50 */
		wait_ms = 100;
	}

	//printk("[LG2102] mdelay (IN) ms = (%d)\n", ms);
	mdelay_in_flg = 1;

	do
	{
		msleep(wait_ms);
		if(user_stop_flg == 1)
		{
			printk("~~~~~~~~ Ustop flag is set so return false ~~~~~~~~\n");
			rc = 0;
			break;
		}
	}while((--wait_loop) > 0);

	mdelay_in_flg = 0;
	user_stop_flg = 0;
	//printk("[LG2102]mdelay(OUT) ms = (%d) ustop_flg = (%d) wait_loop = (%d)\n", ms, user_stop_flg, wait_loop);
#endif	
	return rc;
}

int tdmb_lg2102_power_on(void)
{
	//int rc = 0;
	// DMB_INT = GPIO29
	// DMB_EN = GPIO28(1.2V) , 1.8V_VIO(alyways on)
	// DMB_RESET = GPIO62

	// Gpio Handle
    NvOdmServicesGpioHandle hGpio;
	
	if ( TdmbCtrlInfo.TdmbPowerOnState == FALSE )
	{
		wake_lock(&TdmbCtrlInfo.wake_lock);
		// Getting the OdmGpio Handle
	    hGpio = NvOdmGpioOpen();
	    if (!hGpio)
	    {
	        printk("tdmb_lg2102_power_on GPIO Open fail \n");
			return FALSE;
	    }
		
	    NvOdmGpioSetState(hGpio, hDmbEnPin, 1);
		udelay(1000); //1ms
		NvOdmGpioSetState(hGpio, hDmbResetPin, 1);
		udelay(1000); //1ms
		NvOdmGpioSetState(hGpio, hDmbResetPin, 0);
		udelay(1000); //1ms
		NvOdmGpioSetState(hGpio, hDmbResetPin, 1);
		udelay(1000); //1ms

		tdmb_lg2102_interrupt_free();
		TdmbCtrlInfo.TdmbPowerOnState = TRUE;

		//NvOdmGpioClose(hGpio);
	}
	else
	{
		printk("tdmb_lg2102_power_on the power already turn on \n");
	}

	NvOdmGpioClose(hGpio);
	printk("tdmb_lg2102_power_on completed \n");

	return TRUE;
}

int tdmb_lg2102_power_off(void)
{
	//int rc = FALSE;
	// Gpio Handle
    NvOdmServicesGpioHandle hGpio;

	if ( TdmbCtrlInfo.TdmbPowerOnState == TRUE )
	{
		tdmb_lg2102_interrupt_lock();
		TdmbCtrlInfo.TdmbPowerOnState = FALSE;

		// Getting the OdmGpio Handle
	    hGpio = NvOdmGpioOpen();
	    if (!hGpio)
	    {
	        printk("tdmb_lg2102_power_on GPIO Open fail \n");
			return FALSE;
	    }

		NvOdmGpioSetState(hGpio, hDmbResetPin, 0);
		NvOdmGpioSetState(hGpio, hDmbEnPin, 0);

		NvOdmGpioSetState(hGpio, hDmbEarAntPin, 0);
		
		wake_unlock(&TdmbCtrlInfo.wake_lock);
	}
	else
	{
		printk("tdmb_lg2102_power_on the power already turn off \n");
	}	

	NvOdmGpioClose(hGpio);
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
	if (TdmbCtrlInfo.pSpiDevice == NULL)
	{
		printk("tdmb_lg2102_interrupt_lock fail\n");
		return;
	}

	disable_irq(TdmbCtrlInfo.pSpiDevice->irq);
	return;
}

void tdmb_lg2102_interrupt_free(void)
{
	if (TdmbCtrlInfo.pSpiDevice == NULL)
	{
		printk("tdmb_lg2102_interrupt_free fail\n");
		return;
	}

	enable_irq(TdmbCtrlInfo.pSpiDevice->irq);
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

	if (TdmbCtrlInfo.pSpiDevice == NULL)
	{
		printk("tdmb_lg2102_spi_write_read error txdata=0x%x, length=%d\n", (unsigned int)tx_data, tx_length+rx_length);
		return FALSE;
	}

	mutex_lock(&TdmbCtrlInfo.mutex);

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	rc = spi_sync(TdmbCtrlInfo.pSpiDevice, &m);
	if ( rc < 0 )
	{
		printk("tdmb_lg2102_spi_read_burst result(%d), actual_len=%d\n",rc, m.actual_length);
	}

	mutex_unlock(&TdmbCtrlInfo.mutex);

	return TRUE;
}

static irqreturn_t broadcast_tdmb_spi_isr(int irq, void *handle)
{
	struct TDMB_LG2102_CTRL* pTdmbInfo;

	pTdmbInfo = (struct TDMB_LG2102_CTRL *)handle;	
	if ( pTdmbInfo && pTdmbInfo->TdmbPowerOnState )
	{
		if (pTdmbInfo->spi_irq_status)
		{
			printk("########### broadcast_tdmb_spi_isr ###########\n");
			printk("######### spi read funtion is so late skip #########\n");
			return IRQ_HANDLED;
		}
		queue_work(pTdmbInfo->spi_wq, &pTdmbInfo->spi_work);    
	}
	else
	{
		printk("broadcast_tdmb_spi_isr is called, but device is off state\n");
	}

	return IRQ_HANDLED; 
}

static void broacast_tdmb_spi_work(struct work_struct *tdmb_work)
{
	struct TDMB_LG2102_CTRL *pTdmbWorkData;

	pTdmbWorkData = container_of(tdmb_work, struct TDMB_LG2102_CTRL, spi_work);
	if ( pTdmbWorkData )
	{
		pTdmbWorkData->spi_irq_status = TRUE;
		broadcast_tdmb_read_data();
		pTdmbWorkData->spi_irq_status = FALSE;
		//printk("broacast_tdmb_spi_work is called handle=0x%x\n", (unsigned int)pTdmbWorkData);
	}
}

static int broadcast_tdmb_lg2102_probe(struct spi_device *spi)
{
	int rc;

	struct broadcast_tdmb_data *pdata;

	// Gpio Handle
    NvOdmServicesGpioHandle hGpio;
	//NvOdmGpioPinHandle hDmbResetPin;
	//NvOdmGpioPinHandle hDmbEnPin;
	//NvOdmGpioPinHandle hDmbIntPin;

	TdmbCtrlInfo.pSpiDevice 					= spi;
	TdmbCtrlInfo.pSpiDevice->mode 			= SPI_MODE_0;
	TdmbCtrlInfo.pSpiDevice->bits_per_word 	= 8;
	TdmbCtrlInfo.pSpiDevice->max_speed_hz 	= 4000*1000;
	rc = spi_setup(spi);
	printk("broadcast_tdmb_lg2102_probe spi_setup=%d\n", rc);

	INIT_WORK(&TdmbCtrlInfo.spi_work, broacast_tdmb_spi_work);
	TdmbCtrlInfo.spi_wq = create_singlethread_workqueue("tdmb_spi_wq");
        if(TdmbCtrlInfo.spi_wq == NULL){
		printk("Failed to setup tdmb spi workqueue \n");          
        }

	rc = request_irq(spi->irq, broadcast_tdmb_spi_isr, IRQF_DISABLED | IRQF_TRIGGER_RISING, spi->dev.driver->name, &TdmbCtrlInfo);
	printk("broadcast_tdmb_lg2102_probe request_irq=%d\n", rc);

	// Getting the OdmGpio Handle
    hGpio = NvOdmGpioOpen();
    if (!hGpio)
    {
        printk("tdmb_lg2102_probe GPIO Open fail \n");
		return FALSE;
    }

	hDmbResetPin = NvOdmGpioAcquirePinHandle(hGpio, DMB_RESET_PORT, DMB_RESET_PIN);
	hDmbEnPin = NvOdmGpioAcquirePinHandle(hGpio, DMB_ENABLE_PORT, DMB_ENABLE_PIN);

	pdata = spi->dev.platform_data;
	hDmbIntPin = pdata->hNVODM_DmbIntPin;

	hDmbEarAntPin = NvOdmGpioAcquirePinHandle(hGpio, DMB_EARANT_PORT, DMB_EARANT_PIN);

	// Setting the ON/OFF pin to output mode and setting to Low.
    NvOdmGpioConfig(hGpio, hDmbResetPin, NvOdmGpioPinMode_Output);
	NvOdmGpioConfig(hGpio, hDmbEnPin, NvOdmGpioPinMode_Output);
	NvOdmGpioConfig(hGpio, hDmbEarAntPin, NvOdmGpioPinMode_Output);

	NvOdmGpioSetState(hGpio, hDmbResetPin, 0);
	NvOdmGpioSetState(hGpio, hDmbEnPin, 0);

	NvOdmGpioSetState(hGpio, hDmbEarAntPin, 0);
	//NvOdmGpioConfig(hGpio, hDmbIntPin, NvOdmGpioPinMode_Tristate);

	//NvOdmGpioClose(hGpio);
	
	tdmb_lg2102_interrupt_lock();

	mutex_init(&TdmbCtrlInfo.mutex);
	wake_lock_init(&TdmbCtrlInfo.wake_lock,  WAKE_LOCK_SUSPEND, dev_name(&spi->dev));		

	NvOdmGpioClose(hGpio);
	return rc;
}

static int broadcast_tdmb_lg2102_remove(struct spi_device *spi)
{
	printk("broadcast_tdmb_lg2102_remove \n");

	if (TdmbCtrlInfo.spi_wq)
	{
		flush_workqueue(TdmbCtrlInfo.spi_wq);
		destroy_workqueue(TdmbCtrlInfo.spi_wq);
	}

	free_irq(spi->irq, &TdmbCtrlInfo);
	mutex_destroy(&TdmbCtrlInfo.mutex);

	wake_lock_destroy(&TdmbCtrlInfo.wake_lock);

	memset((unsigned char*)&TdmbCtrlInfo, 0x0, sizeof(struct TDMB_LG2102_CTRL));
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
	return (int)TdmbCtrlInfo.TdmbPowerOnState;
}

EXPORT_SYMBOL(broadcast_tdmb_is_on);

module_init(broadcast_tdmb_drv_init);
module_exit(broadcast_tdmb_drv_exit);
MODULE_DESCRIPTION("broadcast_tdmb_drv_init");
MODULE_LICENSE("INC");

