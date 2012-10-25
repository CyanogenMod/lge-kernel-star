/*
 * Driver for keys on GPIO lines capable of generating interrupts.
 *
 * Copyright 2005 Phil Blundell
 *
 * Copyright 2010-2011 NVIDIA Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>

#include <linux/init.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/gpio_keys.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>

// 20110712 youngjin.yoo@lge.com bug fix when gpio-keys interrupted [S]
#if defined (CONFIG_KS1103) || defined(CONFIG_LU6500) || defined(CONFIG_SU880)|| defined(CONFIG_KU8800)	//20120517 youngmin.kim@lge.com
#define CONFIG_POWERKEY_LP1
#endif
static bool is_suspend = false;
// 20110712 youngjin.yoo@lge.com bug fix when gpio-keys interrupted [E]
//MOBII_CHNANGE_S 20120716 jd.park@mobii.co.kr : FOTA UA Upgrade for ICS
#if defined(CONFIG_MACH_STAR_SU660)
static bool is_ua_mode = false;
#endif
//MOBII_CHNANGE_E 20120716 jd.park@mobii.co.kr : FOTA UA Upgrade for ICS

struct gpio_button_data {
	struct gpio_keys_button *button;
	struct input_dev *input;
	struct timer_list timer;
	struct work_struct work;
	int timer_debounce;	/* in msecs */
	bool disabled;
// 20110712 youngjin.yoo@lge.com bug fix when gpio-keys interrupted [S]
#if defined(CONFIG_POWERKEY_LP1)
	int button_state;
#endif
// 20110712 youngjin.yoo@lge.com bug fix when gpio-keys interrupted [E]
};

struct gpio_keys_drvdata {
	struct input_dev *input;
	struct mutex disable_lock;
	unsigned int n_buttons;
	int (*enable)(struct device *dev);
	void (*disable)(struct device *dev);
	struct gpio_button_data data[0];
};

// MOBII_S [shhong@mobii.co.kr] 2012-07-09 : From X2_KDDI Release Git.
#if defined(CONFIG_MACH_STAR_SU660)
extern bool in_call_state();
bool islp1checkon = false;
#endif
// MOBII_E [shhong@mobii.co.kr] 2012-07-09 : From X2_KDDI Release Git.

#ifdef CONFIG_MACH_STAR
extern void write_cmd_reserved_buffer(unsigned char *buf, size_t len);
extern 	void read_cmd_reserved_buffer(unsigned char *buf, size_t len);
extern void emergency_restart(void);
static ssize_t star_reset_show(struct device *dev, 
		struct device_attribute *attr, char *buf)
{

	unsigned char tmpbuf[3];
	int ret;
	int tag;

	read_cmd_reserved_buffer(tmpbuf, 3);
	printk(" power key reserved_buffer = %c%c%c\n",tmpbuf[0],tmpbuf[1],tmpbuf[2]);

#define _COLDBOOT          0   
#define _NORMAL_WARMBOOT   1
#define _HIDDEN_RESET      2

	if (('p' == tmpbuf[0])||('z' == tmpbuf[0]))
	{
		printk("star_powekey : hidden reset detected\n");
		tag = _HIDDEN_RESET;
		ret = sprintf(buf,"%d\n",tag);
		tmpbuf[1] = NULL; tmpbuf[2] = NULL;
		write_cmd_reserved_buffer(tmpbuf,3);
		return ret;
	}

	if ('w' == tmpbuf[0])
	{
		switch (tmpbuf[1])
		{
			case 'm': // reboot immediately
			case 'a': // panic
				printk("star_powekey : hidden reset detected\n");
				tag = _HIDDEN_RESET;
				tmpbuf[2] = NULL;
				break;
			case 'e': //recovery
				printk("star_powekey : factory reset detected\n");
				tag = _NORMAL_WARMBOOT;
				tmpbuf[2] = NULL;
				break;
			default : // reboot other case (ex adb)
				printk("star_powekey : warm boot detected\n");
				tag = _NORMAL_WARMBOOT;
				tmpbuf[1] = NULL; tmpbuf[2] = NULL;
				break;
		}
	}
	else
	{
		printk("star_powekey : cold boot detected\n");
		tag = _COLDBOOT; 
		memset(tmpbuf,NULL,3);
	}    
	ret = sprintf(buf,"%d\n",tag);
	write_cmd_reserved_buffer(tmpbuf,3);
	printk("star_powekey : hidden----\n");
	return ret;
}

static ssize_t star_reset_store(struct device *dev, 
		struct device_attribute *attr, char *buf, size_t count)
{

	unsigned char tmpbuf[3] = { NULL, };
	tmpbuf[0] = 'w'; // index for warm-boot
	write_cmd_reserved_buffer(tmpbuf,3);
	emergency_restart();
	return count;
}

DEVICE_ATTR(reset, S_IRUGO | S_IWUGO, star_reset_show, star_reset_store);
#endif

//MOBII_CHNANGE_S 20120716 jd.park@mobii.co.kr : FOTA UA Upgrade for ICS
#if defined(CONFIG_MACH_STAR_SU660)
static ssize_t ignore_key_event_show(struct device *dev, 
		struct device_attribute *attr, char *buf)
{
    printk("%s() : is_ua_mode = %d", __FUNCTION__, is_ua_mode);

    return sprintf(buf, "%d\n", is_ua_mode);
}

static ssize_t ignore_key_event_store(struct device *dev, 
		struct device_attribute *attr, char *buf, size_t count)
{
    unsigned long value = simple_strtoul(buf, NULL, 10);

    printk("%s() : value = %d", __FUNCTION__, value);

    if(value == 1) {
        is_ua_mode = true;
    } else if(value == 0) {
        is_ua_mode = false;
    }

	return count;
}
DEVICE_ATTR(ignore_key_event, S_IRUGO|S_IWUSR|S_IWGRP, ignore_key_event_show, ignore_key_event_store);
#endif
//MOBII_CHNANGE_E 20120716 jd.park@mobii.co.kr : FOTA UA Upgrade for ICS

/*
 * SYSFS interface for enabling/disabling keys and switches:
 *
 * There are 4 attributes under /sys/devices/platform/gpio-keys/
 *	keys [ro]              - bitmap of keys (EV_KEY) which can be
 *	                         disabled
 *	switches [ro]          - bitmap of switches (EV_SW) which can be
 *	                         disabled
 *	disabled_keys [rw]     - bitmap of keys currently disabled
 *	disabled_switches [rw] - bitmap of switches currently disabled
 *
 * Userland can change these values and hence disable event generation
 * for each key (or switch). Disabling a key means its interrupt line
 * is disabled.
 *
 * For example, if we have following switches set up as gpio-keys:
 *	SW_DOCK = 5
 *	SW_CAMERA_LENS_COVER = 9
 *	SW_KEYPAD_SLIDE = 10
 *	SW_FRONT_PROXIMITY = 11
 * This is read from switches:
 *	11-9,5
 * Next we want to disable proximity (11) and dock (5), we write:
 *	11,5
 * to file disabled_switches. Now proximity and dock IRQs are disabled.
 * This can be verified by reading the file disabled_switches:
 *	11,5
 * If we now want to enable proximity (11) switch we write:
 *	5
 * to disabled_switches.
 *
 * We can disable only those keys which don't allow sharing the irq.
 */

/**
 * get_n_events_by_type() - returns maximum number of events per @type
 * @type: type of button (%EV_KEY, %EV_SW)
 *
 * Return value of this function can be used to allocate bitmap
 * large enough to hold all bits for given type.
 */
static inline int get_n_events_by_type(int type)
{
	BUG_ON(type != EV_SW && type != EV_KEY);

	return (type == EV_KEY) ? KEY_CNT : SW_CNT;
}

// 20110712 youngjin.yoo@lge.com bug fix when gpio-keys interrupted [S]
#if defined(CONFIG_POWERKEY_LP1)
static inline void send_events_power_key_forcely(struct input_dev* input)
{
	input_event(input, EV_KEY, KEY_POWER, 1);
	input_event(input, EV_KEY, KEY_POWER, 0);
	input_sync(input);
}
#endif
// 20110712 youngjin.yoo@lge.com bug fix when gpio-keys interrupted [E]

/**
 * gpio_keys_disable_button() - disables given GPIO button
 * @bdata: button data for button to be disabled
 *
 * Disables button pointed by @bdata. This is done by masking
 * IRQ line. After this function is called, button won't generate
 * input events anymore. Note that one can only disable buttons
 * that don't share IRQs.
 *
 * Make sure that @bdata->disable_lock is locked when entering
 * this function to avoid races when concurrent threads are
 * disabling buttons at the same time.
 */
static void gpio_keys_disable_button(struct gpio_button_data *bdata)
{
	if (!bdata->disabled) {
		/*
		 * Disable IRQ and possible debouncing timer.
		 */
		disable_irq(gpio_to_irq(bdata->button->gpio));
		if (bdata->timer_debounce)
			del_timer_sync(&bdata->timer);

		bdata->disabled = true;
	}
}

/**
 * gpio_keys_enable_button() - enables given GPIO button
 * @bdata: button data for button to be disabled
 *
 * Enables given button pointed by @bdata.
 *
 * Make sure that @bdata->disable_lock is locked when entering
 * this function to avoid races with concurrent threads trying
 * to enable the same button at the same time.
 */
static void gpio_keys_enable_button(struct gpio_button_data *bdata)
{
	if (bdata->disabled) {
		enable_irq(gpio_to_irq(bdata->button->gpio));
		bdata->disabled = false;
	}
}

/**
 * gpio_keys_attr_show_helper() - fill in stringified bitmap of buttons
 * @ddata: pointer to drvdata
 * @buf: buffer where stringified bitmap is written
 * @type: button type (%EV_KEY, %EV_SW)
 * @only_disabled: does caller want only those buttons that are
 *                 currently disabled or all buttons that can be
 *                 disabled
 *
 * This function writes buttons that can be disabled to @buf. If
 * @only_disabled is true, then @buf contains only those buttons
 * that are currently disabled. Returns 0 on success or negative
 * errno on failure.
 */
static ssize_t gpio_keys_attr_show_helper(struct gpio_keys_drvdata *ddata,
		char *buf, unsigned int type,
		bool only_disabled)
{
	int n_events = get_n_events_by_type(type);
	unsigned long *bits;
	ssize_t ret;
	int i;

	bits = kcalloc(BITS_TO_LONGS(n_events), sizeof(*bits), GFP_KERNEL);
	if (!bits)
		return -ENOMEM;

	for (i = 0; i < ddata->n_buttons; i++) {
		struct gpio_button_data *bdata = &ddata->data[i];

		if (bdata->button->type != type)
			continue;

		if (only_disabled && !bdata->disabled)
			continue;

		__set_bit(bdata->button->code, bits);
	}

	ret = bitmap_scnlistprintf(buf, PAGE_SIZE - 2, bits, n_events);
	buf[ret++] = '\n';
	buf[ret] = '\0';

	kfree(bits);

	return ret;
}

/**
 * gpio_keys_attr_store_helper() - enable/disable buttons based on given bitmap
 * @ddata: pointer to drvdata
 * @buf: buffer from userspace that contains stringified bitmap
 * @type: button type (%EV_KEY, %EV_SW)
 *
 * This function parses stringified bitmap from @buf and disables/enables
 * GPIO buttons accordinly. Returns 0 on success and negative error
 * on failure.
 */
static ssize_t gpio_keys_attr_store_helper(struct gpio_keys_drvdata *ddata,
		const char *buf, unsigned int type)
{
	int n_events = get_n_events_by_type(type);
	unsigned long *bits;
	ssize_t error;
	int i;

	bits = kcalloc(BITS_TO_LONGS(n_events), sizeof(*bits), GFP_KERNEL);
	if (!bits)
		return -ENOMEM;

	error = bitmap_parselist(buf, bits, n_events);
	if (error)
		goto out;

	/* First validate */
	for (i = 0; i < ddata->n_buttons; i++) {
		struct gpio_button_data *bdata = &ddata->data[i];

		if (bdata->button->type != type)
			continue;

		if (test_bit(bdata->button->code, bits) &&
				!bdata->button->can_disable) {
			error = -EINVAL;
			goto out;
		}
	}

	mutex_lock(&ddata->disable_lock);

	for (i = 0; i < ddata->n_buttons; i++) {
		struct gpio_button_data *bdata = &ddata->data[i];

		if (bdata->button->type != type)
			continue;

		if (test_bit(bdata->button->code, bits))
			gpio_keys_disable_button(bdata);
		else
			gpio_keys_enable_button(bdata);
	}

	mutex_unlock(&ddata->disable_lock);

out:
	kfree(bits);
	return error;
}

#define ATTR_SHOW_FN(name, type, only_disabled)				\
	static ssize_t gpio_keys_show_##name(struct device *dev,		\
			struct device_attribute *attr,	\
			char *buf)				\
{									\
	struct platform_device *pdev = to_platform_device(dev);		\
	struct gpio_keys_drvdata *ddata = platform_get_drvdata(pdev);	\
	\
	return gpio_keys_attr_show_helper(ddata, buf,			\
			type, only_disabled);		\
}

ATTR_SHOW_FN(keys, EV_KEY, false);
ATTR_SHOW_FN(switches, EV_SW, false);
ATTR_SHOW_FN(disabled_keys, EV_KEY, true);
ATTR_SHOW_FN(disabled_switches, EV_SW, true);

/*
 * ATTRIBUTES:
 *
 * /sys/devices/platform/gpio-keys/keys [ro]
 * /sys/devices/platform/gpio-keys/switches [ro]
 */
static DEVICE_ATTR(keys, S_IRUGO, gpio_keys_show_keys, NULL);
static DEVICE_ATTR(switches, S_IRUGO, gpio_keys_show_switches, NULL);

#define ATTR_STORE_FN(name, type)					\
	static ssize_t gpio_keys_store_##name(struct device *dev,		\
			struct device_attribute *attr,	\
			const char *buf,			\
			size_t count)			\
{									\
	struct platform_device *pdev = to_platform_device(dev);		\
	struct gpio_keys_drvdata *ddata = platform_get_drvdata(pdev);	\
	ssize_t error;							\
	\
	error = gpio_keys_attr_store_helper(ddata, buf, type);		\
	if (error)							\
	return error;						\
	\
	return count;							\
}
//20110717 deukgi.shin@lge.com // sleep on/off for diag test mode[S]
int slide_state = 0; //20110805 deukgi.shin@lge.com // slide open/close state check.
EXPORT_SYMBOL(slide_state);

static ssize_t pwrbutton_test_mode_store(struct device *dev,  struct device_attribute *attr,  const char *buf, size_t count)
{
	struct gpio_keys_drvdata *ddata = dev_get_drvdata(dev);
	
	input_report_key(ddata->input, KEY_POWER, 1);	
	input_sync(ddata->input);
	mdelay(100);
	input_report_key(ddata->input, KEY_POWER, 0);
	input_sync(ddata->input);
	mdelay(100);
	printk("Diag SLEEP Mode.\n");
}
//20110805 deukgi.shin@lge.com // slide open/close state check.[S]
#if defined (CONFIG_LU6500) || (CONFIG_KS1001)
static ssize_t slide_state_show(struct device *dev,  struct device_attribute *attr,  const char *buf)
{
	printk("%s, %s, %d",__FILE__, __FUNCTION__, slide_state);
	return sprintf(buf, "%d\n", slide_state);
}
static ssize_t slide_state_store(struct device *dev,  struct device_attribute *attr,  const char *buf, size_t count)
{
	unsigned long value = simple_strtoul(buf, NULL, 10);		
	struct gpio_keys_drvdata *ddata = dev_get_drvdata(dev);

	if(value == 2)
	{
		input_event(ddata->input, EV_SW, SW_LID, slide_state);
		input_sync(ddata->input);
	}
	else if((value==0)||(value==1)){
		input_event(ddata->input, EV_SW, SW_LID, value);
		input_sync(ddata->input);
	}
}
#endif
//20110805 deukgi.shin@lge.com // slide open/close state check.[E]
static DEVICE_ATTR(pwrbutton_test_mode, S_IRUGO|S_IWUSR|S_IWGRP, NULL, pwrbutton_test_mode_store);
#if defined (CONFIG_LU6500) || (CONFIG_KS1001)
static DEVICE_ATTR(slide_state, S_IRUGO|S_IWUSR|S_IWGRP, slide_state_show, slide_state_store);
#endif
// 20110717 deukgi.shin@lge.com // sleep on/off for diag test mode[E]

ATTR_STORE_FN(disabled_keys, EV_KEY);
ATTR_STORE_FN(disabled_switches, EV_SW);

/*
 * ATTRIBUTES:
 *
 * /sys/devices/platform/gpio-keys/disabled_keys [rw]
 * /sys/devices/platform/gpio-keys/disables_switches [rw]
 */
static DEVICE_ATTR(disabled_keys, S_IWUSR | S_IRUGO,
		gpio_keys_show_disabled_keys,
		gpio_keys_store_disabled_keys);
static DEVICE_ATTR(disabled_switches, S_IWUSR | S_IRUGO,
		gpio_keys_show_disabled_switches,
		gpio_keys_store_disabled_switches);

static struct attribute *gpio_keys_attrs[] = {
	&dev_attr_keys.attr,
	&dev_attr_switches.attr,
	&dev_attr_disabled_keys.attr,
	&dev_attr_disabled_switches.attr,
	&dev_attr_pwrbutton_test_mode.attr,//20110717 deukgi.shin@lge.com // sleep on/off for diag test mode
#if defined (CONFIG_LU6500) || (CONFIG_KS1001)
	&dev_attr_slide_state.attr,//20110805 deukgi.shin@lge.com // slide open/close state check.
#endif
//MOBII_CHNANGE_S 20120716 jd.park@mobii.co.kr : FOTA UA Upgrade for ICS
#if defined(CONFIG_MACH_STAR_SU660)
   	&dev_attr_ignore_key_event.attr,
#endif
//MOBII_CHNANGE_E 20120716 jd.park@mobii.co.kr : FOTA UA Upgrade for ICS
	NULL,
};

static struct attribute_group gpio_keys_attr_group = {
	.attrs = gpio_keys_attrs,
};

static void gpio_keys_report_event(struct gpio_button_data *bdata)
{
	struct gpio_keys_button *button = bdata->button;
	struct input_dev *input = bdata->input;
	unsigned int type = button->type ?: EV_KEY;
	// 20110712 youngjin.yoo@lge.com bug fix when gpio-keys interrupted [S]
#if !defined(CONFIG_POWERKEY_LP1)
	int state = (gpio_get_value_cansleep(button->gpio) ? 1 : 0) ^ button->active_low;
	input_event(input, type, button->code, !!state);
#else
	bdata->button_state = (gpio_get_value(button->gpio) ? 1 : 0) ^ button->active_low;

	printk("GPIO_KEY %s KEY : %d, STATE : %d\n", __FUNCTION__, button->code, bdata->button_state);
#if defined (CONFIG_LU6500) || (CONFIG_KS1001)
	if(button->code == SW_LID)
	{
		slide_state = !!bdata->button_state;
		printk("SLIDE STAT IS : %d\n", slide_state);
	}
#endif	
	input_event(input, type, button->code, !!bdata->button_state);
#endif
	input_sync(input);
	// LGE_CHANGE_S [yehan.ahn@lge.com] 2011-01-05, [LGE_AP20] GPIO_KEY DEBUG
#if !defined(CONFIG_POWERKEY_LP1)
	printk(KERN_INFO "GPIO_KEY EVENT OCCUR[%u] : %d", button->code, state);
#endif
	// LGE_CHANGE_E [yehan.ahn@lge.com] 2011-01-05, [LGE_AP20] GPIO_KEY DEBUG
}

static void gpio_keys_work_func(struct work_struct *work)
{
	struct gpio_button_data *bdata =
		container_of(work, struct gpio_button_data, work);
	// 20110712 youngjin.yoo@lge.com bug fix when gpio-keys interrupted [S]
#if !defined(CONFIG_POWERKEY_LP1)
	gpio_keys_report_event(bdata);
#else
	struct gpio_keys_button *button = bdata->button;
	struct input_dev *input = bdata->input;
	unsigned int type = button->type ?: EV_KEY;
	bdata->button_state = (gpio_get_value(button->gpio) ? 1 : 0) ^ button->active_low;

	printk("ENTER %s KEY : %d, STATE : %d\n", __FUNCTION__, button->code, bdata->button_state);
#if defined (CONFIG_LU6500) || (CONFIG_KS1001)
	if(button->code == SW_LID)
	{
		slide_state = !!bdata->button_state;
		printk("SLIDE STAT IS : %d\n", slide_state);
	}
#endif	
	input_event(input, type, button->code, !!bdata->button_state);
	input_sync(input);
#endif
	// 20110712 youngjin.yoo@lge.com bug fix when gpio-keys interrupted [E]
}

static void gpio_keys_timer(unsigned long _data)
{
	struct gpio_button_data *data = (struct gpio_button_data *)_data;

	schedule_work(&data->work);
}

static irqreturn_t gpio_keys_isr(int irq, void *dev_id)
{
	struct gpio_button_data *bdata = dev_id;
	struct gpio_keys_button *button = bdata->button;

//MOBII_CHNANGE_S 20120716 jd.park@mobii.co.kr : FOTA UA Upgrade for ICS
#if defined(CONFIG_MACH_STAR_SU660)
    if(is_ua_mode)
    {
        return IRQ_HANDLED;
    }
#endif
//MOBII_CHNANGE_E 20120716 jd.park@mobii.co.kr : FOTA UA Upgrade for ICS

	BUG_ON(irq != gpio_to_irq(button->gpio));
	// 20110712 youngjin.yoo@lge.com bug fix when gpio-keys interrupted [S]
#if defined(CONFIG_POWERKEY_LP1)
	if(is_suspend)
	{
		printk("ENTER : %s, KEYCODE = %d, STATE = 1\n", __FUNCTION__, button->code);
		bdata->button_state = 1;
		input_event(bdata->input, EV_KEY, button->code, 1);
		input_sync(bdata->input);
		is_suspend = false;
	}
	else
	{
#endif	
		if (bdata->timer_debounce)
			mod_timer(&bdata->timer,
					jiffies + msecs_to_jiffies(bdata->timer_debounce));
		else
			schedule_work(&bdata->work);
#if defined(CONFIG_POWERKEY_LP1)            
	}
#endif    
	// 20110712 youngjin.yoo@lge.com bug fix when gpio-keys interrupted [E]
	return IRQ_HANDLED;
}

static int __devinit gpio_keys_setup_key(struct platform_device *pdev,
		struct gpio_button_data *bdata,
		struct gpio_keys_button *button)
{
	char *desc = button->desc ? button->desc : "gpio_keys";
	struct device *dev = &pdev->dev;
	unsigned long irqflags;
	int irq, error;

	setup_timer(&bdata->timer, gpio_keys_timer, (unsigned long)bdata);
	INIT_WORK(&bdata->work, gpio_keys_work_func);

	error = gpio_request(button->gpio, desc);
	if (error < 0) {
		dev_err(dev, "failed to request GPIO %d, error %d\n",
				button->gpio, error);
		goto fail2;
	}

	error = gpio_direction_input(button->gpio);
	if (error < 0) {
		dev_err(dev, "failed to configure"
				" direction for GPIO %d, error %d\n",
				button->gpio, error);
		goto fail3;
	}

	if (button->debounce_interval) {
		error = gpio_set_debounce(button->gpio,
				button->debounce_interval * 1000);
		/* use timer if gpiolib doesn't provide debounce */
		if (error < 0)
			bdata->timer_debounce = button->debounce_interval;
	}

	irq = gpio_to_irq(button->gpio);
	if (irq < 0) {
		error = irq;
		dev_err(dev, "Unable to get irq number for GPIO %d, error %d\n",
				button->gpio, error);
		goto fail3;
	}

	irqflags = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING;
	/*
	 * If platform has specified that the button can be disabled,
	 * we don't want it to share the interrupt line.
	 */
	if (!button->can_disable)
		irqflags |= IRQF_SHARED;

	error = request_any_context_irq(irq, gpio_keys_isr, irqflags, desc, bdata);
	if (error < 0) {
		dev_err(dev, "Unable to claim irq %d; error %d\n",
				irq, error);
		goto fail3;
	}

	return 0;

fail3:
	gpio_free(button->gpio);
fail2:
	return error;
}

static int gpio_keys_open(struct input_dev *input)
{
	struct gpio_keys_drvdata *ddata = input_get_drvdata(input);

	return ddata->enable ? ddata->enable(input->dev.parent) : 0;
}

static void gpio_keys_close(struct input_dev *input)
{
	struct gpio_keys_drvdata *ddata = input_get_drvdata(input);

	if (ddata->disable)
		ddata->disable(input->dev.parent);
}

static int __devinit gpio_keys_probe(struct platform_device *pdev)
{
	struct gpio_keys_platform_data *pdata = pdev->dev.platform_data;
	struct gpio_keys_drvdata *ddata;
	struct device *dev = &pdev->dev;
	struct input_dev *input;
	int i, error;
	int wakeup = 0;

	// LGE_CHANGE_S [yehan.ahn@lge.com] 2011-01-05, [LGE_AP20] GPIO_KEY DEBUG
	printk(KERN_INFO "gpio_keys: gpio_keys_probe( ) is called\n");
	// LGE_CHANGE_E [yehan.ahn@lge.com] 2011-01-05, [LGE_AP20] GPIO_KEY DEBUG

	ddata = kzalloc(sizeof(struct gpio_keys_drvdata) +
			pdata->nbuttons * sizeof(struct gpio_button_data),
			GFP_KERNEL);
	input = input_allocate_device();
	if (!ddata || !input) {
		dev_err(dev, "failed to allocate state\n");
		error = -ENOMEM;
		goto fail1;
	}

	ddata->input = input;
	ddata->n_buttons = pdata->nbuttons;
	ddata->enable = pdata->enable;
	ddata->disable = pdata->disable;
	mutex_init(&ddata->disable_lock);

	platform_set_drvdata(pdev, ddata);
	input_set_drvdata(input, ddata);

	input->name = pdev->name;
	input->phys = "gpio-keys/input0";
	input->dev.parent = &pdev->dev;
	input->open = gpio_keys_open;
	input->close = gpio_keys_close;

	input->id.bustype = BUS_HOST;
	input->id.vendor = 0x0001;
	input->id.product = 0x0001;
	input->id.version = 0x0100;

	/* Enable auto repeat feature of Linux input subsystem */
	if (pdata->rep)
		__set_bit(EV_REP, input->evbit);

	for (i = 0; i < pdata->nbuttons; i++) {
		struct gpio_keys_button *button = &pdata->buttons[i];
		struct gpio_button_data *bdata = &ddata->data[i];
		unsigned int type = button->type ?: EV_KEY;

		bdata->input = input;
		bdata->button = button;
		// 20110712 youngjin.yoo@lge.com bug fix when gpio-keys interrupted [S]
#if defined(CONFIG_POWERKEY_LP1)
		bdata->button_state = 0;
#endif
		// 20110712 youngjin.yoo@lge.com bug fix when gpio-keys interrupted [E]
		error = gpio_keys_setup_key(pdev, bdata, button);
		if (error)
			goto fail2;

		if (button->wakeup)
			wakeup = 1;

		input_set_capability(input, type, button->code);
	}
	
	input_set_capability(input, EV_KEY, KEY_TESTMODE_UNLOCK);// MOBII_CHNANGE 20120703 jslee@mobii.co.kr - AT%PTNCLR

	error = sysfs_create_group(&pdev->dev.kobj, &gpio_keys_attr_group);
	if (error) {
		dev_err(dev, "Unable to export keys/switches, error: %d\n",
				error);
		goto fail2;
	}

	error = input_register_device(input);
	if (error) {
		dev_err(dev, "Unable to register input device, error: %d\n",
				error);
		goto fail3;
	}

	/* get current state of buttons */
#if defined (CONFIG_LU6500) || (CONFIG_KS1001)	
	input_event(input, EV_SW, SW_LID, 1); // deukgi.shin@lge.com //20110805 // change slide state.
	input_sync(input);
#endif
	for (i = 0; i < pdata->nbuttons; i++)
		gpio_keys_report_event(&ddata->data[i]);
	input_sync(input);

	device_init_wakeup(&pdev->dev, wakeup);
#ifdef CONFIG_MACH_STAR
	int ret;
	ret = device_create_file(&pdev->dev, &dev_attr_reset);
	if (ret) {
		goto dev_attr_reset_file_create_fail;
	}
#endif
	return 0;

#ifdef CONFIG_MACH_STAR
dev_attr_reset_file_create_fail:
	device_remove_file(&pdev->dev, &dev_attr_reset);
#endif
fail3:
	sysfs_remove_group(&pdev->dev.kobj, &gpio_keys_attr_group);
fail2:
	while (--i >= 0) {
		free_irq(gpio_to_irq(pdata->buttons[i].gpio), &ddata->data[i]);
		if (ddata->data[i].timer_debounce)
			del_timer_sync(&ddata->data[i].timer);
		cancel_work_sync(&ddata->data[i].work);
		gpio_free(pdata->buttons[i].gpio);
	}

	platform_set_drvdata(pdev, NULL);
fail1:
	input_free_device(input);
	kfree(ddata);

	return error;
}

static int __devexit gpio_keys_remove(struct platform_device *pdev)
{
	struct gpio_keys_platform_data *pdata = pdev->dev.platform_data;
	struct gpio_keys_drvdata *ddata = platform_get_drvdata(pdev);
	struct input_dev *input = ddata->input;
	int i;

	sysfs_remove_group(&pdev->dev.kobj, &gpio_keys_attr_group);
#ifdef	CONFIG_MACH_STAR
	device_remove_file(&pdev->dev, &dev_attr_reset);
#endif
	device_init_wakeup(&pdev->dev, 0);

	for (i = 0; i < pdata->nbuttons; i++) {
		int irq = gpio_to_irq(pdata->buttons[i].gpio);
		free_irq(irq, &ddata->data[i]);
		if (ddata->data[i].timer_debounce)
			del_timer_sync(&ddata->data[i].timer);
		cancel_work_sync(&ddata->data[i].work);
		gpio_free(pdata->buttons[i].gpio);
	}

	input_unregister_device(input);

	return 0;
}


#ifdef CONFIG_PM
static int gpio_keys_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct gpio_keys_platform_data *pdata = pdev->dev.platform_data;
	int i;

	if (device_may_wakeup(&pdev->dev)) {
		for (i = 0; i < pdata->nbuttons; i++) {
			struct gpio_keys_button *button = &pdata->buttons[i];
			if (button->wakeup) {
				int irq = gpio_to_irq(button->gpio);
				enable_irq_wake(irq);
			}
// MOBII_S [shhong@mobii.co.kr] 2012-07-09 : From X2_KDDI Release Git.
#if defined(CONFIG_MACH_STAR_SU660)
			else
			{
				printk(KERN_ERR "GPIO_KEYS: Suspend Setting for Non-wakeup keys .\n");
				if(in_call_state())
				{
					int irq = gpio_to_irq(button->gpio);
					enable_irq_wake(irq);
					islp1checkon = true;
					printk(KERN_ERR "GPIO_KEYS: IRQ[0x%x] enable_irq_wake'd.\n", irq);
				}
			}
#endif
// MOBII_E [shhong@mobii.co.kr] 2012-07-09 : From X2_KDDI Release Git.
		}
	}
	is_suspend = true;	//LGE_CHANGE
	return 0;
}

static int gpio_keys_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct gpio_keys_drvdata *ddata = platform_get_drvdata(pdev);
	struct gpio_keys_platform_data *pdata = pdev->dev.platform_data;
	int wakeup_key = KEY_RESERVED;
	int i;

// 20110712 youngjin.yoo@lge.com bug fix when gpio-keys interrupted [S]
#if !defined(CONFIG_POWERKEY_LP1)
	if (pdata->wakeup_key)
		wakeup_key = pdata->wakeup_key();

	for (i = 0; i < pdata->nbuttons; i++) {

		struct gpio_keys_button *button = &pdata->buttons[i];
		if (button->wakeup && device_may_wakeup(&pdev->dev)) {
			int irq = gpio_to_irq(button->gpio);
			disable_irq_wake(irq);

			if (wakeup_key == button->code) {
				unsigned int type = button->type ?: EV_KEY;

				input_event(ddata->input, type, button->code, 1);
				input_event(ddata->input, type, button->code, 0);
				input_sync(ddata->input);
			}
		}
// MOBII_S [shhong@mobii.co.kr] 2012-07-09 : From X2_KDDI Release Git.
#if defined(CONFIG_MACH_STAR_SU660)
		else
		{
			printk(KERN_ERR "GPIO_KEYS: Resume Setting for Non-wakeup keys .\n");
			if(islp1checkon == true)
			{
				int irq = gpio_to_irq(button->gpio);
				disable_irq_wake(irq);
				printk(KERN_ERR "GPIO_KEYS: IRQ[0x%x] disable_irq_wake'd.\n", irq);
			}
		}
#endif
// MOBII_E [shhong@mobii.co.kr] 2012-07-09 : From X2_KDDI Release Git.

		gpio_keys_report_event(&ddata->data[i]);
	}
// MOBII_S [shhong@mobii.co.kr] 2012-07-19 : Make 2 GPIO_KEY Disable(Up/Down).
#if defined(CONFIG_MACH_STAR_SU660)
	if(islp1checkon == true) {
		islp1checkon = false;
		printk(KERN_ERR "GPIO_KEYS: islp1checkon flag disabled.");
	}
#endif 
// MOBII_E [shhong@mobii.co.kr] 2012-07-19 : Make 2 GPIO_KEY Disable(Up/Down).
	input_sync(ddata->input);
#else
	for (i = 0; i < pdata->nbuttons; i++) {

		struct gpio_keys_button *button = &pdata->buttons[i];
		if (button->wakeup && device_may_wakeup(&pdev->dev)) {
			int irq = gpio_to_irq(button->gpio);
			disable_irq_wake(irq);
		}

		gpio_keys_report_event(&ddata->data[i]);
	}
	input_sync(ddata->input);
	is_suspend = false;
#endif
// 20110712 youngjin.yoo@lge.com bug fix when gpio-keys interrupted [E]

	return 0;
}

static const struct dev_pm_ops gpio_keys_pm_ops = {
	.suspend	= gpio_keys_suspend,
	.resume		= gpio_keys_resume,
};
#endif

static struct platform_driver gpio_keys_device_driver = {
	.probe		= gpio_keys_probe,
	.remove		= __devexit_p(gpio_keys_remove),
	.driver		= {
		.name	= "gpio-keys",
		.owner	= THIS_MODULE,
#ifdef CONFIG_PM
		.pm	= &gpio_keys_pm_ops,
#endif
	}
};

static int __init gpio_keys_init(void)
{
	// LGE_CHANGE_S [yehan.ahn@lge.com] 2011-01-05, [LGE_AP20] GPIO_KEY DEBUG
	printk(KERN_INFO "gpio_keys: gpio_keys_init(void) is called\n");
	// LGE_CHANGE_E [yehan.ahn@lge.com] 2011-01-05, [LGE_AP20] GPIO_KEY DEBUG
	return platform_driver_register(&gpio_keys_device_driver);
}

static void __exit gpio_keys_exit(void)
{
	platform_driver_unregister(&gpio_keys_device_driver);
}

module_init(gpio_keys_init);
module_exit(gpio_keys_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Phil Blundell <pb@handhelds.org>");
MODULE_DESCRIPTION("Keyboard driver for CPU GPIOs");
MODULE_ALIAS("platform:gpio-keys");
