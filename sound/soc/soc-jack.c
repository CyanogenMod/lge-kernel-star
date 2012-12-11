/*
 * soc-jack.c  --  ALSA SoC jack handling
 *
 * Copyright 2008 Wolfson Microelectronics PLC.
 *
 * Author: Mark Brown <broonie@opensource.wolfsonmicro.com>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 */
#define DEBUG

#include <sound/jack.h>
#include <sound/soc.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <trace/events/asoc.h>

// LGE_CHANGE_S [heejeong.seo@lge.com] 2011-12-20 [LGE_AP20]
#if defined (CONFIG_MACH_STAR) || defined (CONFIG_MACH_BSSQ)

#include <linux/wakelock.h>

#if defined(CONFIG_MACH_STAR)//jongik2.kim 20100910 i2c_fix 
#define HOOK_USE_ADC 1
#else
#define HOOK_USE_ADC 0
#endif
#if defined(CONFIG_MACH_STAR)
#define HEADSET_CONNECTED 1
#define HEADSET_DISCONNECTED 0
#define HEADSET_NONE 2 		// MOBII_S [shhong@mobii.co.kr] 2012-06-19: Code From IS11LG.
#else
#define HEADSET_CONNECTED 0
#define HEADSET_DISCONNECTED 1
#define HEADSET_NONE 2 		// MOBII_S [shhong@mobii.co.kr] 2012-06-19: Code From IS11LG.
#endif

// MOBII_S [shhong@mobii.co.kr] 2012-06-19: Codes From IS11LG.
#if defined(CONFIG_MACH_BSSQ) || (CONFIG_MACH_STAR)
#define INT_NONE 0
#define INT_ING 1
unsigned int int_status = INT_NONE;
#endif
// MOBII_E [shhong@mobii.co.kr] 2012-06-19: Codes From IS11LG.

unsigned int hook_status = HOOK_RELEASED;
headset_type_enum headset_type = STAR_NONE;
int hook_detection_time = 700; 

extern void star_headsetdet_bias(int bias);	//20100421 bergkamp.cho@lge.com for framwork function used in kernel ==> error [LGE]
extern void star_Mic_bias(int bias); //heejeong.seo@lge.com 20110726 detecting headset when resuming

struct wake_lock headset_wake_lock;  //20111017 heejeong.seo@lge.com Problem that no wake up when disconn headset in calling

// MOBII_S [shhong@mobii.co.kr] 2012-06-19: Codes From IS11LG.
#if defined(CONFIG_MACH_BSSQ) || (CONFIG_MACH_STAR)
unsigned int headset_status = HEADSET_NONE;
#else
unsigned int headset_status = HEADSET_DISCONNECTED;
#endif
// MOBII_E [shhong@mobii.co.kr] 2012-06-19: Codes From IS11LG.

unsigned int headset_gpio_status = 0;
unsigned int hookkey_gpio_status = 0;
unsigned int headset_detecting = 0;

bool block_hook_int = false;
bool is_hook_test = false;

int type_detection_time = 700; 
int remove_detection_time = 60; 
#if !defined(STAR_COUNTRY_KR) //20101103 seki.par@lge.com heaset detect time[LGE_LAB1]
int hook_press_time = 100; 
int hook_release_time = 20; 
#else
int hook_press_time = 100; 
int hook_release_time = 20; 
#endif

// MOBII_S [shhong@mobii.co.kr] 2012-06-19: Codes From IS11LG.
#if defined (CONFIG_MACH_BSSQ) || (CONFIG_MACH_STAR)
int hook_press_delay = 500;
#endif
// MOBII_S [shhong@mobii.co.kr] 2012-06-19: Codes From IS11LG.

#endif
// LGE_CHANGE_E [heejeong.seo@lge.com] 2011-12-20 [LGE_AP20]

/**
 * snd_soc_jack_new - Create a new jack
 * @card:  ASoC card
 * @id:    an identifying string for this jack
 * @type:  a bitmask of enum snd_jack_type values that can be detected by
 *         this jack
 * @jack:  structure to use for the jack
 *
 * Creates a new jack object.
 *
 * Returns zero if successful, or a negative error code on failure.
 * On success jack will be initialised.
 */
int snd_soc_jack_new(struct snd_soc_codec *codec, const char *id, int type,
		     struct snd_soc_jack *jack)
{
	jack->codec = codec;
	INIT_LIST_HEAD(&jack->pins);
	INIT_LIST_HEAD(&jack->jack_zones);
	BLOCKING_INIT_NOTIFIER_HEAD(&jack->notifier);

	return snd_jack_new(codec->card->snd_card, id, type, &jack->jack);
}
EXPORT_SYMBOL_GPL(snd_soc_jack_new);

/**
 * snd_soc_jack_report - Report the current status for a jack
 *
 * @jack:   the jack
 * @status: a bitmask of enum snd_jack_type values that are currently detected.
 * @mask:   a bitmask of enum snd_jack_type values that being reported.
 *
 * If configured using snd_soc_jack_add_pins() then the associated
 * DAPM pins will be enabled or disabled as appropriate and DAPM
 * synchronised.
 *
 * Note: This function uses mutexes and should be called from a
 * context which can sleep (such as a workqueue).
 */
void snd_soc_jack_report(struct snd_soc_jack *jack, int status, int mask)
{
	struct snd_soc_codec *codec;
	struct snd_soc_dapm_context *dapm;
	struct snd_soc_jack_pin *pin;
	int enable;
	int oldstatus;

	trace_snd_soc_jack_report(jack, mask, status);

	if (!jack)
		return;

	codec = jack->codec;
	dapm =  &codec->dapm;

	mutex_lock(&codec->mutex);

	oldstatus = jack->status;

	jack->status &= ~mask;
	jack->status |= status & mask;

	/* The DAPM sync is expensive enough to be worth skipping.
	 * However, empty mask means pin synchronization is desired. */
	if (mask && (jack->status == oldstatus))
		goto out;

	trace_snd_soc_jack_notify(jack, status);

	list_for_each_entry(pin, &jack->pins, list) {
		enable = pin->mask & jack->status;

		if (pin->invert)
			enable = !enable;

		if (enable)
			snd_soc_dapm_enable_pin(dapm, pin->pin);
		else
			snd_soc_dapm_disable_pin(dapm, pin->pin);
	}

	/* Report before the DAPM sync to help users updating micbias status */
	blocking_notifier_call_chain(&jack->notifier, status, jack);

	snd_soc_dapm_sync(dapm);

	snd_jack_report(jack->jack, status);

out:
	mutex_unlock(&codec->mutex);
}
EXPORT_SYMBOL_GPL(snd_soc_jack_report);

/**
 * snd_soc_jack_add_zones - Associate voltage zones with jack
 *
 * @jack:  ASoC jack
 * @count: Number of zones
 * @zone:  Array of zones
 *
 * After this function has been called the zones specified in the
 * array will be associated with the jack.
 */
int snd_soc_jack_add_zones(struct snd_soc_jack *jack, int count,
			  struct snd_soc_jack_zone *zones)
{
	int i;

	for (i = 0; i < count; i++) {
		INIT_LIST_HEAD(&zones[i].list);
		list_add(&(zones[i].list), &jack->jack_zones);
	}
	return 0;
}
EXPORT_SYMBOL_GPL(snd_soc_jack_add_zones);

/**
 * snd_soc_jack_get_type - Based on the mic bias value, this function returns
 * the type of jack from the zones delcared in the jack type
 *
 * @micbias_voltage:  mic bias voltage at adc channel when jack is plugged in
 *
 * Based on the mic bias value passed, this function helps identify
 * the type of jack from the already delcared jack zones
 */
int snd_soc_jack_get_type(struct snd_soc_jack *jack, int micbias_voltage)
{
	struct snd_soc_jack_zone *zone;

	list_for_each_entry(zone, &jack->jack_zones, list) {
		if (micbias_voltage >= zone->min_mv &&
			micbias_voltage < zone->max_mv)
				return zone->jack_type;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(snd_soc_jack_get_type);

/**
 * snd_soc_jack_add_pins - Associate DAPM pins with an ASoC jack
 *
 * @jack:  ASoC jack
 * @count: Number of pins
 * @pins:  Array of pins
 *
 * After this function has been called the DAPM pins specified in the
 * pins array will have their status updated to reflect the current
 * state of the jack whenever the jack status is updated.
 */
int snd_soc_jack_add_pins(struct snd_soc_jack *jack, int count,
			  struct snd_soc_jack_pin *pins)
{
	int i;

	for (i = 0; i < count; i++) {
		if (!pins[i].pin) {
			printk(KERN_ERR "No name for pin %d\n", i);
			return -EINVAL;
		}
		if (!pins[i].mask) {
			printk(KERN_ERR "No mask for pin %d (%s)\n", i,
			       pins[i].pin);
			return -EINVAL;
		}

		INIT_LIST_HEAD(&pins[i].list);
		list_add(&(pins[i].list), &jack->pins);
	}

	/* Update to reflect the last reported status; canned jack
	 * implementations are likely to set their state before the
	 * card has an opportunity to associate pins.
	 */
	snd_soc_jack_report(jack, 0, 0);

	return 0;
}
EXPORT_SYMBOL_GPL(snd_soc_jack_add_pins);

/**
 * snd_soc_jack_notifier_register - Register a notifier for jack status
 *
 * @jack:  ASoC jack
 * @nb:    Notifier block to register
 *
 * Register for notification of the current status of the jack.  Note
 * that it is not possible to report additional jack events in the
 * callback from the notifier, this is intended to support
 * applications such as enabling electrical detection only when a
 * mechanical detection event has occurred.
 */
void snd_soc_jack_notifier_register(struct snd_soc_jack *jack,
				    struct notifier_block *nb)
{
	blocking_notifier_chain_register(&jack->notifier, nb);
}
EXPORT_SYMBOL_GPL(snd_soc_jack_notifier_register);

/**
 * snd_soc_jack_notifier_unregister - Unregister a notifier for jack status
 *
 * @jack:  ASoC jack
 * @nb:    Notifier block to unregister
 *
 * Stop notifying for status changes.
 */
void snd_soc_jack_notifier_unregister(struct snd_soc_jack *jack,
				      struct notifier_block *nb)
{
	blocking_notifier_chain_unregister(&jack->notifier, nb);
}
EXPORT_SYMBOL_GPL(snd_soc_jack_notifier_unregister);

#ifdef CONFIG_GPIOLIB
/* gpio detect */
static void snd_soc_jack_gpio_detect(struct snd_soc_jack_gpio *gpio)
{
	struct snd_soc_jack *jack = gpio->jack;
	int enable;
	int report;

	enable = gpio_get_value_cansleep(gpio->gpio);
	if (gpio->invert)
		enable = !enable;

	if (enable)
		report = gpio->report;
	else
		report = 0;

	if (gpio->jack_status_check)
		report = gpio->jack_status_check();

	snd_soc_jack_report(jack, report, gpio->report);
}

// MOBII_S [shhong@mobii.co.kr] 2012-06-19: Codes From IS11LG.
#if defined (CONFIG_MACH_BSSQ) || (CONFIG_MACH_STAR)
#else
/* irq handler for gpio pin */
static irqreturn_t gpio_handler(int irq, void *data)
{
	struct snd_soc_jack_gpio *gpio = data;
	struct device *dev = gpio->jack->codec->card->dev;

	trace_snd_soc_jack_irq(gpio->name);

	if (device_may_wakeup(dev))
		pm_wakeup_event(dev, gpio->debounce_time + 50);

	schedule_delayed_work(&gpio->work,
			      msecs_to_jiffies(gpio->debounce_time));

	return IRQ_HANDLED;
}
#endif
// MOBII_E [shhong@mobii.co.kr] 2012-06-19: Codes From IS11LG.

// LGE_CHANGE_S [heejeong.seo@lge.com] 2011-12-20 [LGE_AP20]
#if defined (CONFIG_MACH_STAR) || defined (CONFIG_MACH_BSSQ)
extern unsigned int max8907c_adc_read_hook_adc(void);

static void headset_det_work(struct work_struct *work)
{
	headset_gpio_status = gpio_get_value (headset_sw_data->gpio);

	printk(KERN_INFO "@@[Soc-jack.c][headset_det_work()]@@ headset_gpio=%d , headset_status=%d, headset_gpio_status = %d \n",
					headset_sw_data->gpio, headset_status, headset_gpio_status);    

// MOBII_S [shhong@mobii.co.kr] 2012-06-19: Codes From IS11LG.
#if defined(CONFIG_MACH_BSSQ) || (CONFIG_MACH_STAR)
	if(headset_status == HEADSET_NONE && headset_gpio_status != HEADSET_CONNECTED) {
		return;
	}
	else {
		headset_status = headset_gpio_status;
	}
#else
	headset_status = headset_gpio_status;
#endif
// MOBII_E [shhong@mobii.co.kr] 2012-06-19: Codes From IS11LG.

// MOBII_S [shhong@mobii.co.kr] 2012-06-19: Codes From IS11LG.
#if defined (CONFIG_MACH_BSSQ) || (CONFIG_MACH_STAR)
	if( (headset_status == HEADSET_CONNECTED) && (headset_type != STAR_NONE)){
		return;
	}
#endif
// MOBII_E [shhong@mobii.co.kr] 2012-06-19: Codes From IS11LG.

	if(headset_status == HEADSET_DISCONNECTED)
	{
		schedule_delayed_work(headset_sw_data->pdelayed_work, msecs_to_jiffies(remove_detection_time));
// MOBII_S [shhong@mobii.co.kr] 2012-06-19: Codes From IS11LG.
#if defined (CONFIG_MACH_BSSQ)	
		gpio_set_value(headset_sw_data->ear_mic, 0);  // Implement Hook Key.from STAR.
#endif		
// MOBII_E [shhong@mobii.co.kr] 2012-06-19: Codes From IS11LG.

#if defined(STAR_COUNTRY_KR) && !defined(CONFIG_MACH_STAR_SKT_REV_A)	  
		headset_Mic_Bias(0);
#endif
	}
	else /*HEADSET_CONNECTED*/
	{
	        schedule_delayed_work(headset_sw_data->pdelayed_work, msecs_to_jiffies(type_detection_time));  // gpio_work()
// MOBII_S [shhong@mobii.co.kr] 2012-06-19: Codes From IS11LG.
#if defined (CONFIG_MACH_BSSQ) || (CONFIG_MACH_STAR)	
		gpio_set_value(headset_sw_data->ear_mic, 1);  // Implement Hook Key.from STAR.
#endif		
// MOBII_E [shhong@mobii.co.kr] 2012-06-19: Codes From IS11LG.
	}
}

// MOBII_CHANGE_S [shhong@mobii.co.kr] 2012.04.26 STAR Feature Added.
//LGE_CHANGE_S [jung.chanmin@lge.com] 2012.04.17 Enable Headset Detect in Sleep Mode
#if defined (CONFIG_MACH_BSSQ) || defined (CONFIG_MACH_STAR)
void headset_enable()
{
    printk(KERN_INFO "[Soc-jack.c] enter [%s()] int_status=%d \n", __func__, int_status);
// MOBII_S [shhong@mobii.co.kr] 2012-06-19: Codes From IS11LG.
#if defined (CONFIG_MACH_BSSQ) || (CONFIG_MACH_STAR)
    if(int_status == INT_ING) 
    {
        // MOBII_S [shhong@mobii.co.kr] 2012-07-23 : Mistakes
        int_status = INT_NONE; 
        return;
    }
#endif
// MOBII_E [shhong@mobii.co.kr] 2012-06-19: Codes From IS11LG.
    schedule_work(&headset_sw_data->work);  // headset_det_work()
}
EXPORT_SYMBOL_GPL(headset_enable);
#endif
//LGE_CHANGE_E [jung.chanmin@lge.com] 2012.04.17 Enable Headset Detect in Sleep Mode
// MOBII_CHANGE_E [shhong@mobii.co.kr] 2012.04.26 STAR Feature Added.

//LGE_CHANGE_S [chahee.kim@lge.com] 2011-11-14 
static void hook_det_work(struct work_struct *work)
{
    //unsigned int gpio_headset = 0, gpio_hook = 0;
    int hook_adc_value =0;

    if(is_hook_test == false){
        hook_adc_value = 20;
    }
    else{
        printk(KERN_ERR "##(hook_det_work)## IS FACTORYMODE\n");
        hook_adc_value = 65;
    }
    //gpio_headset = gpio_get_value(headset_sw_data->gpio);
#if HOOK_USE_ADC
    hookkey_gpio_status = max8907c_adc_read_hook_adc();
    msleep(10);
    hookkey_gpio_status = max8907c_adc_read_hook_adc();
	    printk(KERN_ERR "##(hook_det_work)## Hmax8907c_adc_read_hook_adc: %d\n", hookkey_gpio_status);
#else
    hookkey_gpio_status = gpio_get_value(headset_sw_data->hook_gpio);
    printk(KERN_ERR "##(hook_det_work) hook_gpio=%d headset_type=%d hook_status=%d hookkey_gpio_status=%d\n", 
			headset_sw_data->hook_gpio, headset_type, hook_status, hookkey_gpio_status);
#endif
    
// MOBII_S [shhong@mobii.co.kr] 2012-06-21: QE #100123 Issue Solved.
#if defined(CONFIG_MACH_STAR)
    if(headset_type != STAR_HEADSET || headset_status != HEADSET_CONNECTED) {
	printk(KERN_ERR "##(hook_det_work) Not STAR_HEADSET || Not HEADSET_CONN.");
	return;
    }
#else
    if(headset_type != STAR_HEADSET)
	return;
#endif
// MOBII_E [shhong@mobii.co.kr] 2012-06-21: QE #100123 Issue Solved.
	
    if(hook_status == HOOK_RELEASED){
#if HOOK_USE_ADC
    if(hookkey_gpio_status <= hook_adc_value){
#else
    if(hookkey_gpio_status == 0){ 
#endif
            hook_status = HOOK_PRESSED; 
	    input_report_key(headset_sw_data->ip_dev, KEY_HOOK, 1);
	    input_sync(headset_sw_data->ip_dev);
	    printk(KERN_ERR "##(hook_det_work)## HOOK PRESSED ADC: %d\n", hookkey_gpio_status);
	}        
    }
    else{
#if HOOK_USE_ADC
    if(hookkey_gpio_status > hook_adc_value){
#else
    if(hookkey_gpio_status == 1){ 
#endif
	    hook_status = HOOK_RELEASED; 
	    input_report_key(headset_sw_data->ip_dev, KEY_HOOK, 0);
	    input_sync(headset_sw_data->ip_dev);
	    printk( KERN_ERR "##(hook_det_work)## HOOK RELEASED ADC: %d\n", hookkey_gpio_status);
	}
    }
}

static irqreturn_t headset_int_handler(int irq, void *dev_id)
{
	struct headset_switch_data *switch_data =
		(struct headset_switch_data *)dev_id;

	struct snd_soc_jack_gpio *gpio = switch_data->jack_gpio;
	struct device *dev = gpio->jack->codec->card->dev;

// MOBII_S [shhong@mobii.co.kr] 2012-07-17: Headset Overdetection Blocked.
#if defined(CONFIG_MACH_STAR)
#define	HEADSET_DET_GPIO_NUM	51
	disable_irq_nosync(gpio_to_irq(HEADSET_DET_GPIO_NUM));
	if(headset_detecting) {
	//	printk(KERN_ERR "##[soc-jack.c][headset_int_handler()]## Headset Already Detected->Ignore IRQ\n");
		return IRQ_HANDLED;
	}
#undef	HEADSET_DET_GPIO_NUM
#endif
// MOBII_E [shhong@mobii.co.kr] 2012-07-17: Headset Overdetection Blocked.

	trace_snd_soc_jack_irq(gpio->name);

	if (device_may_wakeup(dev))
		pm_wakeup_event(dev, gpio->debounce_time + 50);

	headset_detecting = 1;

// MOBII_S [shhong@mobii.co.kr] 2012-06-19: Codes From IS11LG.
#if defined(CONFIG_MACH_BSSQ) || (CONFIG_MACH_STAR)
	int_status = INT_ING;
#endif
// MOBII_E [shhong@mobii.co.kr] 2012-06-19: Codes From IS11LG.
	
	//cancel_delayed_work_sync(&headset_sw_data->hook_delayed_work);
	//schedule_work(&switch_data->work);
	headset_gpio_status = gpio_get_value (headset_sw_data->gpio);
	headset_status = headset_gpio_status;

	printk(KERN_ERR "##[Soc-jack.c][headset_int_handler()]##  headset_gpio=%d , headset_gpio_status=%d , ear_mic_gpio=%d \n", 
				headset_sw_data->gpio, headset_gpio_status, headset_sw_data->ear_mic);

	if(headset_status == HEADSET_DISCONNECTED)
   	{
		schedule_work(headset_sw_data->pdelayed_work);
		#if defined(STAR_COUNTRY_KR) && !defined(CONFIG_MACH_STAR_SKT_REV_A)  
		headset_Mic_Bias(0);
		#endif
		gpio_set_value(headset_sw_data->ear_mic, 0);  //BCH_CHECK. Implement Hook Key. from STAR.
   	}
	else
	{
		gpio_set_value(headset_sw_data->ear_mic, 1);  //BCH_CHECK. Implement Hook Key.from STAR.
		schedule_delayed_work(headset_sw_data->pdelayed_work,	msecs_to_jiffies(type_detection_time));	
		wake_lock_timeout(&headset_wake_lock, msecs_to_jiffies(type_detection_time + 50));	 //20111017 heejeong.seo@lge.com Problem that no wake up when disconn headset in calling
	}
	// Need to do Interrupt Done action

	return IRQ_HANDLED;
}

static irqreturn_t headset_hook_int_handler(int irq, void *dev_id)
{
    unsigned int gpio_hook = 1;	

    printk(KERN_ERR "##[Soc-jack.c][headset_hook_int_handler()]##  headset type %d, detecting %d, block_hook_int %d\n", 
			headset_type, headset_detecting, block_hook_int);

    if( block_hook_int ){
	// hook interrupt done !!!
        return IRQ_HANDLED;
    }
    
    struct headset_switch_data	*switch_data = (struct headset_switch_data *)dev_id;

    if((headset_detecting == 1) || headset_type != STAR_HEADSET) 
    {
    }
    else 
    {
    	if(gpio_hook){
            schedule_delayed_work(&switch_data->hook_delayed_work,	msecs_to_jiffies(hook_release_time));
        }
        else{
// MOBII_S [shhong@mobii.co.kr] 2012-06-19: Codes From IS11LG.
#if defined (CONFIG_MACH_BSSQ) || (CONFIG_MACH_STAR)
            schedule_delayed_work(&switch_data->hook_delayed_work,	msecs_to_jiffies(hook_press_delay));
#else
            schedule_delayed_work(&switch_data->hook_delayed_work,	msecs_to_jiffies(hook_press_time));
#endif			
// MOBII_E [shhong@mobii.co.kr] 2012-06-19: Codes From IS11LG.
        }
    }

    return IRQ_HANDLED;
}
//LGE_CHANGE_E [chahee.kim@lge.com] 2011-11-14 
#endif
// LGE_CHANGE_E [heejeong.seo@lge.com] 2011-12-20 [LGE_AP20]

/* gpio work */
static void gpio_work(struct work_struct *work)
{
    struct snd_soc_jack_gpio *gpio;

//LGE_CHANGE_S [chahee.kim@lge.com] 2011-11-14 
    unsigned int hook_value =0;
//LGE_CHANGE_E [chahee.kim@lge.com] 2011-11-14 

    gpio = container_of(work, struct snd_soc_jack_gpio, work.work);
    snd_soc_jack_gpio_detect(gpio);

// LGE_CHANGE_S [heejeong.seo@lge.com] 2011-12-22 [LGE_AP20]
#if defined (CONFIG_MACH_STAR) || defined (CONFIG_MACH_BSSQ)

//LGE_CHANGE_S [chahee.kim@lge.com] 2011-11-14 
    headset_status = headset_gpio_status;

    printk(KERN_ERR "##[Soc-jack] enter [gpio_work()] ## START : headset_gpio_status(%d) , headset_status(%d) , headset_type=%d \n", 
				headset_gpio_status, headset_status, headset_type);

    if( (headset_status == HEADSET_CONNECTED) && (headset_type == STAR_NONE))
    {
        headset_type = STAR_NONE;
        
        star_headsetdet_bias(1);

        //msleep(100);

#if HOOK_USE_ADC
        hook_value = max8907c_adc_read_hook_adc();
        msleep(10);
        hook_value = max8907c_adc_read_hook_adc();

        printk(KERN_INFO "##(gpio_work)## max8907c_adc_read_hook_adc() : hook_value(%d) \n", hook_value);
        
        if(hook_value > 350) //20101127 seki.par@lge.com detect adc 1200==>350[LGE_LAB1]
        {
            headset_type = STAR_HEADSET;
            hook_status = HOOK_RELEASED;
#if defined(CONFIG_MACH_STAR_SU660) || defined(CONFIG_KS1103) || defined(CONFIG_MACH_STAR_P990) || defined(CONFIG_MACH_STAR_P999)
	    gpio_set_value(headset_sw_data->ear_mic, 1);	//LGE_CHANGE_S [chahee.kim@lge.com] 2012-02-01 
#endif
            printk(KERN_INFO "##(gpio_work)## Headset Detected \n");   
        }
        else
        {
            headset_type = STAR_HEADPHONE;
            hook_status = HOOK_PRESSED;
#if defined(CONFIG_MACH_STAR_SU660) || defined(CONFIG_KS1103) || defined(CONFIG_MACH_STAR_P990) || defined(CONFIG_MACH_STAR_P999)
	    gpio_set_value(headset_sw_data->ear_mic, 0);	//LGE_CHANGE_S [chahee.kim@lge.com] 2012-02-01 
#endif
            printk(KERN_INFO "##(gpio_work)## Headphone Detected \n");
        }
#else	// If HOOK_USE_ADC Disable
        hookkey_gpio_status = gpio_get_value (headset_sw_data->hook_gpio);

        printk(KERN_INFO "##[Soc-jack][gpio_work()]## : hookkey status(%d)\n", hookkey_gpio_status);

#if defined(CONFIG_MACH_STAR_P990) || defined(CONFIG_MACH_STAR_P999) 
        if (hookkey_gpio_status == 1)
#else
        if (hookkey_gpio_status == 0)
#endif
        {
            headset_type = STAR_HEADPHONE;
            hook_status = HOOK_PRESSED;
            printk(KERN_INFO "##[Soc-jack][gpio_work()]## Headphone Detected \n");
        }
        else
        {
            headset_type = STAR_HEADSET;
            hook_status = HOOK_RELEASED;
            printk(KERN_INFO "##[Soc-jack][gpio_work()]## Headset Detected \n");
        }
#endif 
    }
    else
    {
        headset_type = STAR_NONE;
        hook_status = HOOK_RELEASED; 
// MOBII_S [shhong@mobii.co.kr] 2012-06-19: Codes From IS11LG.
#if defined(CONFIG_MACH_BSSQ) || (CONFIG_MACH_STAR)
        headset_status = HEADSET_NONE;
#endif
// MOBII_E [shhong@mobii.co.kr] 2012-06-19: Codes From IS11LG.

#if defined(CONFIG_MACH_STAR_SU660) || defined(CONFIG_KS1103) || defined(CONFIG_MACH_STAR_P990) || defined(CONFIG_MACH_STAR_P999)
	gpio_set_value(headset_sw_data->ear_mic, 0);	//LGE_CHANGE_S [chahee.kim@lge.com] 2012-02-01 
#endif

	input_report_key(headset_sw_data->ip_dev, KEY_HOOK, 0);
	input_sync(headset_sw_data->ip_dev);
        printk(KERN_INFO "##[Soc-jack][gpio_work()]## Headset isn't Detected \n");
    }

    if(headset_type != STAR_HEADSET){
        block_hook_int = true;
        star_headsetdet_bias(0);    //20100419 bergkamp.cho@lge.com  for Headset MIC Bias  ==> framwork function used in kernel ==> error [LGE]
    }
    else
    {
        block_hook_int = false;
    }

    headset_detecting = 0;

    switch_set_state(&headset_sw_data->sdev, headset_type);

    printk(KERN_INFO "##[Soc-jack] exit [gpio_work()]## END : Detected type : %d\n", headset_type);
//LGE_CHANGE_E [chahee.kim@lge.com] 2011-11-14 
#endif
// LGE_CHANGE_E [heejeong.seo@lge.com] 2011-12-22 [LGE_AP20]

// MOBII_S [shhong@mobii.co.kr] 2012-07-17: Headset Overdetection Blocked.
#if defined(CONFIG_MACH_STAR)
#define	HEADSET_DET_GPIO_NUM	51
    int_status = INT_NONE;
    enable_irq(gpio_to_irq(HEADSET_DET_GPIO_NUM));
    printk(KERN_INFO "##[Soc-jack] [gpio_work()]## Enable IRQ\n");
#undef	HEADSET_DET_GPIO_NUM
#endif
// MOBII_E [shhong@mobii.co.kr] 2012-07-17: Headset Overdetection Blocked.
}

/**
 * snd_soc_jack_add_gpios - Associate GPIO pins with an ASoC jack
 *
 * @jack:  ASoC jack
 * @count: number of pins
 * @gpios: array of gpio pins
 *
 * This function will request gpio, set data direction and request irq
 * for each gpio in the array.
 */
int snd_soc_jack_add_gpios(struct snd_soc_jack *jack, int count,
			struct snd_soc_jack_gpio *gpios)
{
	int i, ret;

	for (i = 0; i < count; i++) {
		if (!gpio_is_valid(gpios[i].gpio)) {
			printk(KERN_ERR "Invalid gpio %d\n",
				gpios[i].gpio);
			ret = -EINVAL;
			goto undo;
		}
        
		if (!gpios[i].name) {
			printk(KERN_ERR "No name for gpio %d\n",
				gpios[i].gpio);
			ret = -EINVAL;
			goto undo;
		}

		headset_sw_data->jack_gpio = &gpios[i];
        
		ret = gpio_request(gpios[i].gpio, gpios[i].name);
		if (ret)
			goto undo;

		ret = gpio_direction_input(gpios[i].gpio);
		if (ret)
			goto err;

		tegra_gpio_enable (gpios[i].gpio);  //LGE_CHANGE_S [chahee.kim@lge.com] 2011-11-17 

		INIT_WORK(&headset_sw_data->work, headset_det_work);
		INIT_DELAYED_WORK(&gpios[i].work, gpio_work);
		gpios[i].jack = jack;

		headset_sw_data->pdelayed_work = &gpios[i].work;

		ret = request_any_context_irq(gpio_to_irq(gpios[i].gpio),
// LGE_CHANGE_S [heejeong.seo@lge.com] 2011-12-22 [LGE_AP20]
#if defined (CONFIG_MACH_STAR) || defined (CONFIG_MACH_BSSQ)
					      headset_int_handler,
#else
					      gpio_handler,
#endif
// LGE_CHANGE_E [heejeong.seo@lge.com] 2011-12-22 [LGE_AP20]
					      IRQF_TRIGGER_RISING |IRQF_TRIGGER_FALLING,
					      jack->codec->dev->driver->name,
    // LGE_CHANGE_S [heejeong.seo@lge.com] 2011-12-22 [LGE_AP20]
#if defined (CONFIG_MACH_STAR) || defined (CONFIG_MACH_BSSQ)
					      headset_sw_data);
#else
					      &gpios[i]);
#endif
// LGE_CHANGE_E [heejeong.seo@lge.com] 2011-12-22 [LGE_AP20]

		if (ret)
			goto err;

		if (gpios[i].wake) {
			ret = irq_set_irq_wake(gpio_to_irq(gpios[i].gpio), 1);
			if (ret != 0)
				printk(KERN_ERR
				  "Failed to mark GPIO %d as wake source: %d\n",
					gpios[i].gpio, ret);
		}

		printk(KERN_ERR "##(snd_soc_jack_add_gpios)## : HP IRQ %d\n", gpio_to_irq(gpios[i].gpio));
// MOBII_S [shhong@mobii.co.kr] 2012-06-19: Codes From IS11LG.
#if defined(CONFIG_MACH_BSSQ) || (CONFIG_MACH_STAR)
		ret = enable_irq_wake(gpio_to_irq(gpios[i].gpio));
		if (ret) {
			printk("%s: enable_irq_wake error\n", __func__);
			free_irq(gpio_to_irq(gpios[i].gpio), NULL);
			return ret;
		}
#endif
// MOBII_E [shhong@mobii.co.kr] 2012-06-19: Codes From IS11LG.

// LGE_CHANGE_S [heejeong.seo@lge.com] 2011-12-22 [LGE_AP20]
#if defined (CONFIG_MACH_STAR) || defined (CONFIG_MACH_BSSQ)

//LGE_CHANGE_S [chahee.kim@lge.com] 2011-11-14 
#if defined(CONFIG_MACH_STAR_SU660) || defined(CONFIG_KS1103) || defined(CONFIG_MACH_STAR_P990) || defined(CONFIG_MACH_STAR_P999)
		ret = gpio_request(headset_sw_data->ear_mic, "ear_mic");
		ret = gpio_direction_output(headset_sw_data->ear_mic, 0);
		if (ret)
			goto err;
		tegra_gpio_enable (headset_sw_data->ear_mic);
#endif

		printk(KERN_ERR "##(snd_soc_jack_add_gpios)## : Hook Gpio %d\n", headset_sw_data->hook_gpio);

		ret = gpio_request(headset_sw_data->hook_gpio, "hook_det");
		ret = gpio_direction_input(headset_sw_data->hook_gpio);
		if (ret)
			goto err;
		tegra_gpio_enable (headset_sw_data->hook_gpio);		
		INIT_DELAYED_WORK(&headset_sw_data->hook_delayed_work, hook_det_work);

		headset_sw_data->hook_irq = gpio_to_irq(headset_sw_data->hook_gpio);

	        printk(KERN_ERR "##(snd_soc_jack_add_gpios)## : Hook IRQ %d\n", headset_sw_data->hook_irq);
        
		ret = request_irq(headset_sw_data->hook_irq, headset_hook_int_handler,
			(IRQF_TRIGGER_FALLING|IRQF_TRIGGER_RISING), "hook_det", headset_sw_data);
		if (ret)
			goto err;

// MOBII_S [shhong@mobii.co.kr] 2012-06-19: Codes From IS11LG.
#if defined(CONFIG_MACH_BSSQ) || (CONFIG_MACH_STAR)
		// 20111028 not in progress on booting with Headphone
		// please, make hook interrupt enable after detecting headset or headphone 
		ret = enable_irq_wake(headset_sw_data->hook_irq); 
		if (ret) {
			printk("%s: [enable_irq_wake()] hook_irq error\n", __func__);
			free_irq(headset_sw_data->hook_irq, NULL);
			return ret;
		}
#endif
// MOBII_E [shhong@mobii.co.kr] 2012-06-19: Codes From IS11LG.

        	block_hook_int = true;
//LGE_CHANGE_E [chahee.kim@lge.com] 2011-11-14 
#endif
// LGE_CHANGE_E [heejeong.seo@lge.com] 2011-12-22 [LGE_AP20]


#ifdef CONFIG_GPIO_SYSFS
		/* Expose GPIO value over sysfs for diagnostic purposes */
		gpio_export(gpios[i].gpio, false);
#endif

		/* Update initial jack status */
		snd_soc_jack_gpio_detect(&gpios[i]);
	}

	return 0;

err:
	gpio_free(gpios[i].gpio);
	if(!headset_sw_data->hook_gpio)
        	gpio_free(headset_sw_data->hook_gpio);
undo:
	snd_soc_jack_free_gpios(jack, i, gpios);

	return ret;
}
EXPORT_SYMBOL_GPL(snd_soc_jack_add_gpios);

/**
 * snd_soc_jack_free_gpios - Release GPIO pins' resources of an ASoC jack
 *
 * @jack:  ASoC jack
 * @count: number of pins
 * @gpios: array of gpio pins
 *
 * Release gpio and irq resources for gpio pins associated with an ASoC jack.
 */
void snd_soc_jack_free_gpios(struct snd_soc_jack *jack, int count,
			struct snd_soc_jack_gpio *gpios)
{
	int i;

	for (i = 0; i < count; i++) {
#ifdef CONFIG_GPIO_SYSFS
		gpio_unexport(gpios[i].gpio);
#endif
		free_irq(gpio_to_irq(gpios[i].gpio), &gpios[i]);
		cancel_delayed_work_sync(&gpios[i].work);
		gpio_free(gpios[i].gpio);
		gpios[i].jack = NULL;
	}
}
EXPORT_SYMBOL_GPL(snd_soc_jack_free_gpios);
#endif	/* CONFIG_GPIOLIB */
