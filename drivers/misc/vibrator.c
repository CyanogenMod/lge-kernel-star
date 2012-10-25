#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/input.h>
#include <linux/sysfs.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/timer.h>
#include <linux/pwm.h>
#include <linux/gpio.h>
#include <linux/hrtimer.h>
#include <linux/regulator/consumer.h>
#include <mach/vibrator.h>

#define VIB_DEBUG	0
#define VIB_FINETUNE	1

struct pwm_vib_data {
	struct pwm_device*	pwm;
	struct device*		dev;
	struct hrtimer		timer;
	int		duty;
	int		period;
	int		gpio_enable;
	int 	(*power)(char* reg_id, int on);	
};

static struct pwm_vib_data*	pvib	=	NULL;
//static struct regulator*	regulator	=	NULL;
static volatile int vib_stay_time	=	0; 

#if	0	// by dongjin73.kim
static void star_vib_work_func(struct work_struct *work)
{
	u32 gpio_state = 0;	
	#if VIB_DEBUG 
	printk("vibrator test... value = %d\n", toggle);
	#endif

	NvOdmGpioConfig( g_vib->h_vib_gpio, g_vib->h_vib_gpio_pin, NvOdmGpioPinMode_Output);
	
	NvOdmGpioSetState( g_vib->h_vib_gpio, g_vib->h_vib_gpio_pin, toggle );
	if( toggle == 0 )
		toggle++;
	else	
		toggle = 0;

	NvOdmGpioGetState(g_vib->h_vib_gpio, g_vib->h_vib_gpio_pin, &gpio_state); 	
	#if VIB_DEBUG
	printk("[skhwang] VIB_EN = %d\n", gpio_state);
	#endif

	NvOdmOsSleepMS(1000);
	schedule_delayed_work(&g_vib->delayed_work_vib,150);
}


// 20100903 taewan.kim@lge.com Power control bug fix [START]
static int star_vib_set_power_rail( u32 vdd_id, u8 is_enable )
{
printk("vib: %s,%d\n", __FUNCTION__, __LINE__);
#if	0	// by DENNIS
    NvOdmServicesPmuHandle h_pmu = NvOdmServicesPmuOpen();
    NvOdmServicesPmuVddRailCapabilities vddrailcap;
    u32 settletime;

    if(h_pmu)
    {
        NvOdmServicesPmuGetCapabilities( h_pmu, vdd_id, &vddrailcap );
        if( is_enable )
        {
#if VIB_DEBUG
            printk("[skhwang] vibrator PMU enable\n");
#endif
            NvOdmServicesPmuSetVoltage(h_pmu, vdd_id, vddrailcap.requestMilliVolts, &settletime);
        }
        else
        {
#if VIB_DEBUG
            printk("[skhwang] vibrator PMU do not enable\n");
#endif
            NvOdmServicesPmuSetVoltage(h_pmu, vdd_id, NVODM_VOLTAGE_OFF, &settletime);
        }

        if(settletime)
            NvOdmOsWaitUS(settletime);

        NvOdmServicesPmuClose(h_pmu);
#if VIB_DEBUG
        printk("[skhwang] vibrator voltage =  %d or %d \n", vddrailcap.requestMilliVolts, vddrailcap.MinMilliVolts);
#endif

        return 0;
    }

    return -1;
#endif
}
// 20100903 taewan.kim@lge.com Power control bug fix [END]
#endif	

#define STAR_VIB_ENABLE()	\
		gpio_set_value(pvib->gpio_enable, 1)
#define STAR_VIB_DISABLE()	\
		gpio_set_value(pvib->gpio_enable, 0)		

#if 0		
static void	star_vib_enable()
{
	gpio_set_value(pvib->gpio_enable, 1);
}

static void	star_vib_disable()
{
	gpio_set_value(pvib->gpio_enable, 0);
}
#endif

static enum hrtimer_restart	star_vib_timeout(struct hrtimer *timer)
{
//	star_vib_disable();
	STAR_VIB_DISABLE();

	return	HRTIMER_NORESTART;
}

static ssize_t star_vib_stay_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int	time	=	vib_stay_time;
	return	sprintf(buf, "%d\n", time);	
}

static ssize_t star_vib_stay_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int		value	=	simple_strtoul(buf, NULL, 10);
	unsigned long timeout;

	vib_stay_time	=	value;

  if(vib_stay_time == 0)
  {
    STAR_VIB_DISABLE();
    return 0;
  }  
	if (value < 1000) {
		timeout	=	(unsigned long)(value * 1000 * 1000 );
		hrtimer_start(&pvib->timer, ktime_set(0, timeout), HRTIMER_MODE_REL);
	}
	else {
		timeout	=	(long)value / 1000;	
		hrtimer_start(&pvib->timer, ktime_set((long)timeout, 0), HRTIMER_MODE_REL);
	}
	//star_vib_enable();
	STAR_VIB_ENABLE();
	
	return	count;
}

static ssize_t star_vib_onoff_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int	count	=	sprintf(buf, "%d\n", gpio_get_value(pvib->gpio_enable));

printk("vib: %s,%d -- buf : [%s]\n", __FUNCTION__, __LINE__, buf);
	return	count;
}

static ssize_t star_vib_onoff_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int		val	=	simple_strtoul(buf, NULL, 10);

	if (val == 1)
		//star_vib_enable();
		STAR_VIB_ENABLE();
	else if (val == 0)
		//star_vib_disable();
		STAR_VIB_DISABLE();

	return	count;
}

#ifdef	VIB_FINETUNE
static	int	duty	=	0;
static	int	period	=	0;

static ssize_t star_vib_duty_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	duty	=	simple_strtoul(buf, NULL, 10);

	printk("vib: %s,%d - duty : %d, period : %d\n", __FUNCTION__, __LINE__, duty, period);

	pwm_disable(pvib->pwm);

	pwm_config(pvib->pwm, duty, period);
	pwm_enable(pvib->pwm);

	return	count;
}

static ssize_t star_vib_period_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	printk("vib: %s,%d - duty : %d, period : %d\n", __FUNCTION__, __LINE__, duty, period);

	//star_vib_disable();
	STAR_VIB_DISABLE();
	period	=	simple_strtoul(buf, NULL, 10);

	//star_vib_enable();
	STAR_VIB_ENABLE();

	return	count;
}
#endif

static DEVICE_ATTR(onoff, 0666, star_vib_onoff_show, star_vib_onoff_store);
static DEVICE_ATTR(stay, 0666, star_vib_stay_show, star_vib_stay_store);
#ifdef	VIB_FINETUNE
static DEVICE_ATTR(duty, 0666, NULL, star_vib_duty_store);
static DEVICE_ATTR(period, 0666, NULL, star_vib_period_store);
#endif

static struct attribute *star_vib_attributes[] = {
	&dev_attr_onoff.attr,	
	&dev_attr_stay.attr,
#ifdef	VIB_FINETUNE
	&dev_attr_duty.attr,
	&dev_attr_period.attr,
#endif
	NULL,
};

static const struct attribute_group star_vib_group = {
	.attrs = star_vib_attributes,
};

int star_vib_suspend(struct platform_device *dev, pm_message_t state)
{
	printk("vib: %s,%s,%d\n", __FILE__, __FUNCTION__, __LINE__);

	pwm_disable(pvib->pwm);

	if(pvib->power)
		pvib->power("vcc_motor_3v0", 0);

    return	0;
}

int star_vib_resume(struct platform_device *dev)
{
	printk("vib: %s,%s,%d\n", __FILE__, __FUNCTION__, __LINE__);

	if(pvib->power)
		pvib->power("vcc_motor_3v0", 1);
	
	pwm_config(pvib->pwm, pvib->duty, pvib->period);
	pwm_enable(pvib->pwm);

    return	0;
}

static int __init star_vib_probe(struct platform_device *pdev )
{
	struct pwm_vib_platform_data*	pdata;
//	struct pwm_device*	pwm;
	int		err;

	printk("vib: %s,%s,%d\n", __FILE__, __FUNCTION__, __LINE__);

	pvib	=	kzalloc(sizeof(struct pwm_vib_data), GFP_KERNEL);
	if (!pvib) {
		dev_err(&pdev->dev, "no memory for state\n");
		err	=	-ENOMEM;
		goto	err_alloc;
	}

	pdata	=	pdev->dev.platform_data;
	if(pdata){
		pvib->dev			=	&pdev->dev;
		pvib->gpio_enable	=	pdata->enable;
		pvib->duty			=	pdata->duty_ns;
		pvib->period		=	pdata->period_ns;
		pvib->power			=	pdata->power;
	} else {
		err	=   -EBUSY;
		goto    err_exit;
	}

	if(pvib->power)
		pvib->power("vcc_motor_3v0", 1);

	gpio_request(pdata->enable, "vib_enable");
	gpio_direction_output(pdata->enable, 1);
	gpio_set_value(pdata->enable, 0);
	
	pvib->pwm	=	pwm_request(pdata->pwm_id, "vibrator");
	if (IS_ERR(pvib->pwm)) {
		dev_err(&pdev->dev, "unable to request PWM for vibrator\n");
		err	=	PTR_ERR(pvib->pwm);
		goto	err_pwm;
	}
	else
		dev_dbg(&pdev->dev, "got pwm for vibrator\n");

	hrtimer_init(&pvib->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	pvib->timer.function	=	star_vib_timeout;

	platform_set_drvdata(pdev, pvib);

	// sysfs setup
	if (sysfs_create_group(&pvib->dev->kobj, &star_vib_group)) {
		dev_err(&pdev->dev, "failed to create sys filesystem\n");
		err	=	-ENOSYS;
		goto	err_sysfs;
	}

	pwm_config(pvib->pwm, pvib->duty, pvib->period);
	pwm_enable(pvib->pwm);

	printk("vibrator probe success\n");

	#if VIB_DEBUG 
	printk("testing vibrator\n");
	// hrtimer_start(&pvib->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
	hrtimer_start(&pvib->timer, ktime_set(0, 500 * 1000 * 1000), HRTIMER_MODE_REL);
	STAR_VIB_ENABLE();
	#endif

	return	0;

err_sysfs:
	pwm_free(pvib->pwm);
err_pwm:
err_alloc:
err_exit:
	return	err;
}

static int star_vib_remove(struct platform_device *dev)
{
//	struct platform_device*	pdev	=	to_platform_device(dev);
//	struct pwm_vib_data*	pvib	=	platform_get_drvdata(pdev);

	pwm_free(pvib->pwm);

	return	0;
}

static struct platform_driver star_vib_driver = {
    .probe		=	star_vib_probe,
    .remove		=	star_vib_remove,
    .suspend	=	star_vib_suspend,
    .resume		=	star_vib_resume,    
    .driver		=	{
        .name = "star_vib_name",
    },
};

static int __init star_vib_init(void)
{
    return platform_driver_register(&star_vib_driver);
}

static void __exit star_vib_exit(void)
{
    platform_driver_unregister(&star_vib_driver);
}

module_init(star_vib_init);
module_exit(star_vib_exit);

MODULE_AUTHOR("dongjin73.kim@lge.com");
MODULE_DESCRIPTION("driver of star viberator");
MODULE_LICENSE("GPL");
