/*
 * arch/arm/mach-tegra/tegra_gps.c
 *
 *GPS device using NVIDIA Tegra ODM kit
 * board_nvodm.c
  */

#include <linux/kernel.h>
#include <linux/device.h>

#include <linux/platform_device.h>
#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <mach/gpio.h>

//GPIO control 을 위해서 추가 함. 
#include <nvodm_services.h>
#include "nvodm_query_discovery.h" //NvOdmPeripheralConnectivity *pConnectivity = NULL;
#include "nvos.h"

#define GPS_GUID 	NV_ODM_GUID('N','V','O','D','M','G','P','S')

typedef struct GPSDeviceRec
{    
    NvOdmServicesGpioHandle hGpio;
    NvOdmGpioPinHandle s_hResetGPSGpioPin;
    NvOdmGpioPinHandle s_hStandbyGPSGpioPin;
    NvOdmGpioPinHandle s_hExtLNAGPSGpioPin;
    NvU32 pin[3], port[3];

} GPS_Device;

static GPS_Device s_hGPSHandle;


static ssize_t gps_gpio_reset_show(struct device *dev, 
		struct device_attribute *attr, char *buf)
{
/*
	int pinValue = gpio_get_value(GPS_GPIO_RESET);
*/
	NvU32 pinValue;
	NvOdmGpioGetState(s_hGPSHandle.hGpio, s_hGPSHandle.s_hResetGPSGpioPin, &pinValue); 

	printk(KERN_DEBUG "gps_gpio_reset_show\n");

	return sprintf(buf, "%d\n", pinValue);
}

static ssize_t gps_gpio_reset_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int value;
	printk(KERN_DEBUG "gps_gpio_reset_store\n");
	sscanf(buf, "%d", &value);
/*
	gpio_set_value(GPS_GPIO_RESET, value);
*/
	NvOdmGpioSetState(s_hGPSHandle.hGpio, s_hGPSHandle.s_hResetGPSGpioPin, value);	
	return size;
}



static ssize_t gps_gpio_poweron_show(struct device *dev, struct device_attribute *attr, char *buf)
{
/*
	int pinValue = gpio_get_value(GPS_GPIO_POWERON);
*/
	NvU32 pinValue;
	NvOdmGpioGetState(s_hGPSHandle.hGpio, s_hGPSHandle.s_hStandbyGPSGpioPin, &pinValue); 
	printk(KERN_DEBUG "gps_gpio_poweron_show\n");
	NvOdmGpioGetState(s_hGPSHandle.hGpio, s_hGPSHandle.s_hExtLNAGPSGpioPin, &pinValue);
	printk(KERN_DEBUG "gps_gpio_ExtLNA_show\n");

	return sprintf(buf, "%d\n", pinValue);
}
 
static ssize_t gps_gpio_poweron_store(
                struct device *dev, struct device_attribute *attr,
                const char *buf, size_t size)
{
	int value;
	printk(KERN_DEBUG "gps_gpio_poweron_store\n");
	printk(KERN_DEBUG "gps_gpio_ExtLNA_store\n");

	sscanf(buf, "%d", &value);
/*
	gpio_set_value(GPS_GPIO_POWERON, value);
*/
	NvOdmGpioSetState(s_hGPSHandle.hGpio, s_hGPSHandle.s_hStandbyGPSGpioPin, value);
	NvOdmGpioSetState(s_hGPSHandle.hGpio, s_hGPSHandle.s_hExtLNAGPSGpioPin, value);

	return size;
}

static DEVICE_ATTR(reset, S_IRUGO | S_IWUSR, gps_gpio_reset_show, gps_gpio_reset_store);
static DEVICE_ATTR(poweron, S_IRUGO | S_IWUSR, gps_gpio_poweron_show, gps_gpio_poweron_store);


static int tegra_gps_gpio_probe(struct platform_device *pdev)
{
	int retval;
	const NvOdmPeripheralConnectivity *pConnectivity = NULL;
  	int i,j;
	
	printk(KERN_DEBUG "tegra_gps_reset_probe\n");


	pConnectivity = NvOdmPeripheralGetGuid(GPS_GUID);

	if (!pConnectivity)
	{
		printk("pConnectivity fail.");
		return 0;
	}
	
	s_hGPSHandle.hGpio = NvOdmGpioOpen();

	for (i = 0,j=0 ; i < pConnectivity->NumAddress; i++)
	{
		//only GPIO
		if (pConnectivity->AddressList[i].Interface ==NvOdmIoModule_Gpio )
		{
			s_hGPSHandle.port[j] = pConnectivity->AddressList[i].Instance;
			s_hGPSHandle.pin[j] = pConnectivity->AddressList[i].Address;
			printk("port = %d  pin = %d\n",s_hGPSHandle.port[j],s_hGPSHandle.pin[j]);
			j++;
		}
	}

	s_hGPSHandle.s_hResetGPSGpioPin = NvOdmGpioAcquirePinHandle(s_hGPSHandle.hGpio, s_hGPSHandle.port[0], s_hGPSHandle.pin[0]);
	s_hGPSHandle.s_hStandbyGPSGpioPin = NvOdmGpioAcquirePinHandle(s_hGPSHandle.hGpio, s_hGPSHandle.port[1], s_hGPSHandle.pin[1]);
	s_hGPSHandle.s_hExtLNAGPSGpioPin = NvOdmGpioAcquirePinHandle(s_hGPSHandle.hGpio, s_hGPSHandle.port[2], s_hGPSHandle.pin[2]);

	NvOdmGpioConfig(s_hGPSHandle.hGpio, s_hGPSHandle.s_hResetGPSGpioPin, NvOdmGpioPinMode_Output);
	NvOdmGpioConfig(s_hGPSHandle.hGpio, s_hGPSHandle.s_hStandbyGPSGpioPin, NvOdmGpioPinMode_Output);
	NvOdmGpioConfig(s_hGPSHandle.hGpio, s_hGPSHandle.s_hExtLNAGPSGpioPin, NvOdmGpioPinMode_Output);

	retval = device_create_file(&pdev->dev, &dev_attr_reset);
	if (retval)
		goto error;
	
	retval = device_create_file(&pdev->dev, &dev_attr_poweron);
	if (retval)
		goto error;

	return retval;
error:
	printk(KERN_ERR "tegra_gps_reset_probe -Error\n");
	device_remove_file(&pdev->dev, &dev_attr_reset);
	device_remove_file(&pdev->dev, &dev_attr_poweron);
	
	return 0;
}



static int tegra_gps_gpio_remove(struct platform_device *pdev)
{
	printk(KERN_DEBUG "tegra_gps_reset_remove\n");
	device_remove_file(&pdev->dev, &dev_attr_reset);
	device_remove_file(&pdev->dev, &dev_attr_poweron);
	return 0;
}



// platform_driver
static struct platform_driver tegra_gps_gpio_driver = {
    .probe      = tegra_gps_gpio_probe,
    .remove     = tegra_gps_gpio_remove,
    .driver     = {
        .name   = "tegra_gps_gpio"
    },
};



static int __devinit gps_gpio_init(void)
{
	printk(KERN_DEBUG "gps_gpio_init\n");
    return platform_driver_register(&tegra_gps_gpio_driver);
}

static void __exit gps_gpio_exit(void)
{
	printk(KERN_DEBUG "gps_gpio_exit\n");
    platform_driver_unregister(&tegra_gps_gpio_driver);
}

module_init(gps_gpio_init);
module_exit(gps_gpio_exit);

MODULE_DESCRIPTION("heaven GPS Driver");

