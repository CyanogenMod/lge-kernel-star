#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/input.h>
#include <linux/sysfs.h>
#include <linux/uaccess.h>
//#include <linux/miscdevice.h>
//#include <linux/timer.h>
#include <linux/delay.h>

#include "nvcommon.h"
#include "nvodm_services.h"
#include "nvodm_query.h"
#include "nvodm_query_discovery.h"

#define HALL_DEBUG 1

typedef struct star_hall_device_data
{
	NvOdmServicesGpioHandle h_hall_gpio;
	NvOdmGpioPinHandle h_hall_gpio_pin;
	NvOdmServicesGpioIntrHandle h_hall_intr;
	NvOdmServicesPmuHandle h_hall_pmu;

	NvU32 vdd_id;
	NvU32 hall_intr_port;
	NvU32 hall_intr_pin;	
	bool is_int_mode;
	NvU8 int_even_odd;
	struct delayed_work delayed_work_hall;
	struct input_dev* input_device;
}star_hall_device;

static star_hall_device *g_hall;
static atomic_t sensing_hall;
static bool power_enabled = false;

// 20100903 taewan.kim@lge.com Power control bug fix [START]
static int star_hall_set_power_rail( NvU32 vdd_id, NvBool is_enable )
{
    NvOdmServicesPmuHandle h_pmu = NvOdmServicesPmuOpen();
    NvOdmServicesPmuVddRailCapabilities vddrailcap;
    NvU32 settletime;

    if(h_pmu)
    {   
        NvOdmServicesPmuGetCapabilities( h_pmu, vdd_id, &vddrailcap );
        if( is_enable )
        {   
            #if HALL_DEBUG
            printk("HALL IC PMU enable\n");
            #endif
            NvOdmServicesPmuSetVoltage(h_pmu, vdd_id, vddrailcap.requestMilliVolts, &settletime);
        }   
        else
        {   
            #if HALL_DEBUG
            printk("HALL IC PMU do not enable\n");
            #endif
            NvOdmServicesPmuSetVoltage(h_pmu, vdd_id, NVODM_VOLTAGE_OFF, &settletime);
        }   

        if(settletime)
            NvOdmOsWaitUS(settletime);

    	NvOdmServicesPmuClose(h_pmu);
#if HALL_DEBUG
        printk("[skhwang] HALL IC voltage =  %d or %d \n", vddrailcap.requestMilliVolts, vddrailcap.MinMilliVolts);
#endif
        return 0;
    }

    return -1;
}
// 20100903 taewan.kim@lge.com Power control bug fix [END]

static void star_hall_work_func( struct work_struct* work )
{
	NvU32 gpio_status;

	printk("Enter the %s\n", __func__);

	NvOdmGpioGetState(g_hall->h_hall_gpio, g_hall->h_hall_gpio_pin, &gpio_status );
	
	printk("+++++++++++++++++++++++++Sensing Value = %d\n",   gpio_status );
	
	 schedule_delayed_work(&g_hall->delayed_work_hall, 100 ); 
}

static ssize_t star_hall_sensing_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	sprintf(buf, "%d\n", atomic_read(&sensing_hall));
	return (ssize_t)(strlen(buf) + 1);
}

static ssize_t star_hall_sensing_store(struct device *dev, struct device_attribute *attr, char *buf, size_t count)
{
		
}

static ssize_t star_hall_onoff_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	sprintf(buf, "%d\n", (power_enabled == true));
	return (ssize_t)(strlen(buf) + 1);
}
static ssize_t star_hall_onoff_store(struct device *dev, struct device_attribute *attr, char *buf, size_t count)
{
	u32 val = 0;
	val = simple_strtoul(buf, NULL, 10);
	if(val){
		printk("Turn Hall IC on\n");
		star_hall_set_power_rail(12, NV_TRUE);
		power_enabled = true;
	}
	else{
		printk("Turn Hall IC off\n");
		star_hall_set_power_rail(12, NV_FALSE);
		power_enabled = false;
	}
	return count;
}

static DEVICE_ATTR(sensing, 0666, star_hall_sensing_show, star_hall_sensing_store);
static DEVICE_ATTR(onoff, 0666, star_hall_onoff_show, star_hall_onoff_store);

static struct  attribute *star_hall_attributes[] = {
	&dev_attr_sensing.attr,
	&dev_attr_onoff.attr,
	NULL
};

static const struct attribute_group star_hall_group = {
	.attrs = star_hall_attributes,
};

static void star_hall_intr_handler( void *arg )
{
	NvU32 gpio_status;
	//NvOdmGpioInterruptMask(g_hall->h_hall_intr, NV_TRUE );// NV_FALSE --> enable intr , NV_TRUE --> disable
	NvOdmGpioGetState(g_hall->h_hall_gpio, g_hall->h_hall_gpio_pin, &gpio_status );
	printk("--------------------------------------------------detecting value = %d\n", gpio_status);
	if( gpio_status == 0 )
		{atomic_set( &sensing_hall, 0 );input_report_abs(g_hall->input_device, ABS_HAT2X, 0);input_sync(g_hall->input_device);}
	else if( gpio_status == 1 )
		{atomic_set( &sensing_hall, 1 );input_report_abs(g_hall->input_device, ABS_HAT2X, 1);input_sync(g_hall->input_device);}

	//NvOdmGpioInterruptMask(g_hall->h_hall_intr, NV_FALSE);// NV_FALSE --> enable intr , NV_TRUE --> disable
	NvOdmGpioInterruptDone(g_hall->h_hall_intr);
}

static int __init star_hall_probe( struct platform_device *pdev )
{
	int err = 0;
	struct device *dev = &pdev->dev;
	atomic_set( &sensing_hall, 1 );
	printk("\n================================================================\n");
	printk("[HALL:%s] probing hall ic driver\n", __func__);
	printk("================================================================\n");

	g_hall = kzalloc( sizeof(*g_hall), GFP_KERNEL );
	if(g_hall == NULL)
	{
		err = -1;
		printk("[HALL:%s] Fail to alloc the memory for HALL IC\n",__func__);
		goto error;
	}

	/*g_hall->h_hall_pmu = NvOdmServicesPmuOpen();
	if(!g_hall->h_hall_pmu)
	{
		err = -1;
		printk("[HALL:%s] Fail to Open the pmu!\n", __func__);
		goto error;
	}*/

	if (star_hall_set_power_rail(12, NV_TRUE) != 0)
    {
		err = -1;
		goto error;
	}

	g_hall->h_hall_gpio = NvOdmGpioOpen();
	if(!g_hall->h_hall_gpio)
	{
		printk("[HALL:%s] Fail to open gpio\n", __func__);
		err = -1;
		goto error;
	}

	g_hall->is_int_mode = true;

	g_hall->hall_intr_port = 'u' - 'a';
	g_hall->hall_intr_pin  = 5;

	g_hall->h_hall_gpio_pin = NvOdmGpioAcquirePinHandle(g_hall->h_hall_gpio, g_hall->hall_intr_port, g_hall->hall_intr_pin );
	if( !g_hall->h_hall_gpio_pin )
	{
		printk("[HALL:%s] Fail to acquire pin handle!\n", __func__);
		err = -1;
		goto error;
	}

	NvOdmGpioConfig(g_hall->h_hall_gpio, g_hall->h_hall_gpio_pin, NvOdmGpioPinMode_InputData);
	
//	NvOdmGpioConfig( g_hall->h_hall_gpio, g_hall->hall_intr_port, NvOdmGpioPinMode_InputData );
	if( !g_hall->is_int_mode ){	
	//for polling mode
		INIT_DELAYED_WORK(&g_hall->delayed_work_hall, star_hall_work_func);
		schedule_delayed_work(&g_hall->delayed_work_hall, 100 );
	} else{
	//int mode
	#if 0
		if(NvOdmGpioInterruptRegister(g_hall->h_hall_gpio, &g_hall->h_hall_intr, g_hall->h_hall_gpio_pin, 
		NvOdmGpioPinMode_InputInterruptFallingEdge, star_hall_intr_handler, (void*)g_hall, 0 ) == NV_FALSE ){
			printk("[HALL:%s] Fail to register hall's interrupt\n", __func__ );	
			goto err_irq_request;
		} 
	#endif
        if (NvOdmGpioInterruptRegister(g_hall->h_hall_gpio, &(g_hall->h_hall_intr),
            g_hall->h_hall_gpio_pin, NvOdmGpioPinMode_InputInterruptAny, /*NvOdmGpioPinMode_InputInterruptFallingEdge,*/ star_hall_intr_handler, (void*)&g_hall, 0) == NV_FALSE){
			printk("[HALL:%s] Fail to register hall's interrupt\n", __func__ );	
			goto err_irq_request;
		} 
	}

	g_hall->input_device = input_allocate_device();
	if(!g_hall->input_device){
		printk("Error to allocate the input device of HALL IC\n");
		err = -ENOMEM;
		goto err_irq_request;
	}
	set_bit(EV_KEY, g_hall->input_device->evbit);
	set_bit(KEY_POWER, g_hall->input_device->keybit);
	set_bit(EV_ABS, g_hall->input_device->evbit);
	input_set_abs_params(g_hall->input_device, ABS_HAT2X, 0, 1, 0, 0);
	g_hall->input_device->name = "hall_ic";
	err = input_register_device(g_hall->input_device);
	if(err){
		printk("error to register the input device of hall\n");
		err = -ENOMEM;
		goto err_irq_request;
	}
	err = sysfs_create_group(&dev->kobj, &star_hall_group );
	if(err){
		printk("error to create sys file system\n");
		err = -ENOMEM;
		goto err_irq_request;
	}

	return 0;

error:
	return err;
err_irq_request:
	NvOdmGpioClose(g_hall->h_hall_gpio);
	return err;
}

static int star_hall_remove( struct platform_device *pdev )
{
	return 0;
}

static struct platform_driver star_hall_driver = {
	.probe = star_hall_probe,
	.remove = star_hall_remove,
	.driver = {
		.name = "star_hall",
	},
};

static int __init star_hall_init( void )
{
	return platform_driver_register(&star_hall_driver);
}

static void __exit star_hall_exit( void )
{
	platform_driver_unregister(&star_hall_driver);
}

module_init(star_hall_init);
module_exit(star_hall_exit);

MODULE_AUTHOR("sk.hwang@lge.com");
MODULE_DESCRIPTION("driver of star hall ic");
MODULE_LICENSE("GPL");
