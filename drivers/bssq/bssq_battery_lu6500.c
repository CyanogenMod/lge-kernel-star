/*
 * linux/drivers/power/bssq_battery.c
 *
 * battery driver for Linux
 *
 * Copyright (C) 2008 Texas Instruments, Inc.
 * Author: hyeongwon.oh@lge.com
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/i2c/twl.h>
#include <linux/power_supply.h>
#include <linux/mfd/max8907c.h>

#include "bssq_battery.h"
#include "bssq_battery_temp.h"

#if defined(CONFIG_BSSQ_MUIC_TI)
#include "bssq_muic_ti.h"
#endif
#if defined(CONFIG_BSSQ_CHARGER_RT)
#include <lge/bssq_charger_rt.h>
#endif
#if defined(CONFIG_BSSQ_CHARGER_MAX)
#include <lge/bssq_charger_max.h>
#endif
#if defined(CONFIG_BSSQ_FUELGAUGE)
#include <lge/max17043_fuelgauge.h>
#endif

#define READ_TEMP_ADC
#if defined(READ_TEMP_ADC)
static int TEMP_ADC_VALUE = 0;
ssize_t show_tempadc(struct device *dev, struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", TEMP_ADC_VALUE);
}
DEVICE_ATTR(readtempadc,0644,show_tempadc,NULL);
#endif

//#define CONFIG_HW_EVENT // HW Event Charging

#if defined (CONFIG_HW_EVENT)
#define TEMP_CRITICAL_UPPER	(650)
#define TEMP_CRITICAL_LOWER	(-180)
#define TEMP_LIMIT_UPPER	(600)
#define TEMP_LIMIT_LOWER	(-150)
#else
#define TEMP_CRITICAL_UPPER	(550)
#define TEMP_CRITICAL_LOWER	(-100)
#define TEMP_LIMIT_UPPER	(450)
#define TEMP_LIMIT_LOWER	(-50)
#endif

#if defined (CONFIG_MFD_MAX8907C)
#define MAX8907_LBCNFG 		(0x60)
#define MAX8907_CUTTOFF_33V400 (0xBB) // Threshold3.3V, Hysteresis 400mV
#endif

#define MONITOR_WORK_TIME	(10 * HZ)

#define TEMP_CONTROL 1

#define TEMP_CONTROL_ON 1
#define TEMP_CONTROL_OFF 0

static enum power_supply_property battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_TECHNOLOGY,
    POWER_SUPPLY_PROP_PRESENT,
};

static enum power_supply_property bssq_ac_usb_battery_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

struct battery_info {
	struct i2c_client	*i2c_adc;	// voltage, temperature adc read
	struct i2c_client	*i2c_power;	// for MAX8907_RESET_CNFG setting

	struct device		*dev;

	struct power_supply	ac;
	struct power_supply	usb;
	struct power_supply	bat;
	struct power_supply	usb_bat;

	struct delayed_work	bssq_monitor_work;
	unsigned long	update_time;

	int				charge_type;
	int				charge_status;
	int				present;
	int				voltage;
	int				temperature;
	int				capacity;

	int				prev_charge_type;
	int				prev_charge_status;
	int				prev_present;
	int				prev_voltage;
	int				prev_temperature;
	int				prev_capacity;
#if defined(TEMP_CONTROL)
unsigned int		temp_control; //for Charging test
#endif
};

// Battery Spec.
static struct battery_info *refer_batt_info = NULL;

static int start_monitor = 0;	//20101104 taehwan.kim@lge.com to fix trickle chg
static int test_mode = FALSE; //20110621 hyeongwon.oh@lge.com testmode 255-25-3
static int end_of_charge = 0;
static int EOC_VOLTAGE = 4220; // hg.park@lge.com eoc voltage level

static DEFINE_MUTEX(battery_mutex);

int set_end_of_charge(int complete)
{
	mutex_lock(&battery_mutex);

	if(refer_batt_info == NULL) {
		end_of_charge = 0;
		mutex_unlock(&battery_mutex);
		return 0;
	}
	if(refer_batt_info->charge_type == POWER_SUPPLY_TYPE_BATTERY) {
		end_of_charge = 0;
		mutex_unlock(&battery_mutex);
		return 0;
	}

	if(complete) {
		end_of_charge = 1;
	} else {
		end_of_charge = 0;
	}

	mutex_unlock(&battery_mutex);

	return end_of_charge;
}
EXPORT_SYMBOL(set_end_of_charge);

static int battery_read_temperature(struct battery_info *batt_info)
{
    unsigned char 	data, msb_data, lsb_data;
    unsigned int 	btempdata, btemp;

	// Enable int_ref_en bit in RESET_CNFG Reg
	data = max8907c_reg_read(batt_info->i2c_power, MAX8907_RESET_CNFG);

	data |= (MAX8907_RESET_CNFG_INT_REF_EN_MASK <<
		 MAX8907_RESET_CNFG_INT_REF_EN_SHIFT);

	if (max8907c_reg_write(batt_info->i2c_power, MAX8907_RESET_CNFG, data) < 0)
		return 0;

	// Enable Internal voltage reference
	if (max8907c_reg_write(batt_info->i2c_adc, MAX8907_TSC_CNFG1, 0x12) < 0)
		return 0;

	// Send command to powerup and the ADC perform a conversion
	if (max8907c_reg_write(batt_info->i2c_adc, CONV_REG_THM_ON, 0x00) < 0)
		return 0;

	// Get result
	msb_data = max8907c_reg_read(batt_info->i2c_adc, MAX8907_THM_MSB);
	lsb_data = max8907c_reg_read(batt_info->i2c_adc, MAX8907_THM_LSB);

	btemp = btempdata = (msb_data << 4) | (lsb_data >> 4) ;

#if defined(READ_TEMP_ADC)
	TEMP_ADC_VALUE = btemp;
#endif

	return batt_temp_tbl[btemp];
}

/*
 * Return battery voltage
 * Or < 0 on failure.
 */
static int battery_read_voltage(struct battery_info *batt_info)
{
	int voltage;

#if defined (CONFIG_MFD_MAX8907C)
	unsigned char	data, msb_data, lsb_data;
	unsigned int	batvdata;


	// Enable int_ref_en bit in RESET_CNFG Reg
	data = max8907c_reg_read(batt_info->i2c_power, MAX8907_RESET_CNFG);

	data |= (MAX8907_RESET_CNFG_INT_REF_EN_MASK <<
				 MAX8907_RESET_CNFG_INT_REF_EN_SHIFT);

	if (max8907c_reg_write(batt_info->i2c_power, MAX8907_RESET_CNFG, data) < 0)
		return 0;

	// Enable Internal voltage reference
	if (max8907c_reg_write(batt_info->i2c_adc, MAX8907_TSC_CNFG1, 0x12) < 0)
		return 0;

	// Send command to powerup and the ADC perform a conversion
	if (max8907c_reg_write(batt_info->i2c_adc, CONV_REG_VMBATT_ON, 0x00) < 0)
		return 0;

	// Get result
	msb_data = max8907c_reg_read(batt_info->i2c_adc, MAX8907_VMBATT_MSB);
	lsb_data = max8907c_reg_read(batt_info->i2c_adc, MAX8907_VMBATT_LSB);

	batvdata = (msb_data << 4) | (lsb_data >> 4) ;
	voltage = batvdata*2; //This conversion is for 12bit ADC result : 8.192*CODE/2^N = 8.192*CODE/4096 = CODE*2 [mV]

#elif defined (CONFIG_BSSQ_FUELGAUGE)
	voltage = max17043_get_voltage();
#endif

	return voltage;

}

void battery_setEOCVoltage(void)
{
	mutex_lock(&battery_mutex);

    EOC_VOLTAGE = battery_read_voltage(refer_batt_info);

	mutex_unlock(&battery_mutex);

	printk("[BATT] %s EOC_VOLTAGE(%d) \n",__func__, EOC_VOLTAGE);
}

EXPORT_SYMBOL(battery_setEOCVoltage);


/*
 * Return battery capacity
 * Or < 0 on failure.
 */
static int battery_read_capacity(void)
{
	int ret = 0;

	ret = max17043_get_capacity();

	if(ret < 0)
		ret = 0;
	else if(ret > 100)
		ret = 100;

	return ret;
}

/*
 * @brief  To check battery present from other drivers
 * @author hyeongwon.oh@lge.com
 * @date   2011.05.03
 */
int battery_check_present(void)
{
	if(refer_batt_info == NULL)
	{
		printk("[BATT] battery_check_present() : No refer_batt_info !\n");
		return 0;
	}

//20110621 hyeongwon.oh@lge.com testmode 255-25-3 [S]
	if(test_mode)
		return 1;
//20110621 hyeongwon.oh@lge.com testmode 255-25-3 [E]

    if(refer_batt_info->temperature <  -400 || refer_batt_info->temperature > 900) // // Temperature range -40 ~ 90 [C]
    	return 0;
    return 1;
}
EXPORT_SYMBOL(battery_check_present);

static void battery_update(struct battery_info *batt_info)
{
	mutex_lock(&battery_mutex);

	/* Read Battery Status */
	batt_info->voltage		= battery_read_voltage(batt_info);		// Read Voltage
	batt_info->temperature	= battery_read_temperature(batt_info);	// Read Temperature
	batt_info->capacity 	= battery_read_capacity();				// Read Capacity
	batt_info->present		= battery_check_present();				// Set Battery Present


#if defined(TEMP_CONTROL)
if(batt_info->temp_control == TEMP_CONTROL_ON)
{
	batt_info->temperature = 270; 

	if(batt_info->temp_control == TEMP_CONTROL_ON)
	{
		printk("[TEMP CONTROL]Unlimited Temp Charging ON~~~~~~~~~~~~~~~~", 0);
	}

}
else
{
	printk("[TEMP CONTROL]Unlimited Temp Charging OFF~~~~~~~~~~~~~~~~", 0);

}
#endif

#if defined(CONFIG_BSSQ_MUIC_TI)
	MUIC_CHARGER_TYPE muic_mode = get_muic_charger_type();
	// Set Power Resources
	switch (muic_mode) {
		case MUIC_CHARGER_FACTORY:
			batt_info->charge_type = POWER_SUPPLY_TYPE_UPS;
			break;
		case MUIC_CHARGER_TA:
			batt_info->charge_type = POWER_SUPPLY_TYPE_MAINS;
			break;
		case MUIC_CHARGER_USB:
			batt_info->charge_type = POWER_SUPPLY_TYPE_USB;
			break;
		default :
			batt_info->charge_type = POWER_SUPPLY_TYPE_BATTERY;
		break;
	}
#endif
	// Set Charging Status
	if(batt_info->charge_type == POWER_SUPPLY_TYPE_UPS ||		// JIG
	   batt_info->charge_type == POWER_SUPPLY_TYPE_BATTERY) 
	{	// Battery
		batt_info->charge_status = POWER_SUPPLY_STATUS_DISCHARGING;
	} 
	else 
	{
		if(batt_info->present == 0) // Battery not present. Display as not charging
		{			
			batt_info->charge_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		} 
		else if(batt_info->temperature < TEMP_CRITICAL_LOWER ||
			  batt_info->temperature > TEMP_CRITICAL_UPPER) // Charging Stoped
		{	
			batt_info->charge_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		} 
		else if(batt_info->temperature < TEMP_LIMIT_LOWER ||
			  batt_info->temperature > TEMP_LIMIT_UPPER) // Charging Limited
		{	
			batt_info->charge_status = POWER_SUPPLY_STATUS_CHARGING;
		} 
		else if(start_monitor) // Charging FULL
		{			
			batt_info->charge_status = POWER_SUPPLY_STATUS_FULL;
		} 
		else // Normal Charging
		{					
			batt_info->charge_status = POWER_SUPPLY_STATUS_CHARGING;
		}
	}

	// backup battery info
	batt_info->prev_voltage = batt_info->voltage;
	batt_info->prev_temperature = batt_info->temperature;
	batt_info->prev_capacity = batt_info->capacity;
	batt_info->prev_present = batt_info->present;
	batt_info->prev_charge_type = batt_info->charge_type;
	batt_info->prev_charge_status = batt_info->charge_status;

	mutex_unlock(&battery_mutex);
}

static int set_battery_charging(struct battery_info *batt_info)
{
	int trickle_chg_max, trickle_chg_min, trickle_chg_timer_start;
	charger_ic_status status = charger_ic_get_status();

	if(batt_info->present == 0 || 						// No Battery State
		batt_info->charge_type == POWER_SUPPLY_TYPE_BATTERY) {	// No Charging Source
		start_monitor = 0;
		set_end_of_charge(0);
		return 0;
	}

	/*
	 * Recharging algorithm
     *  - High Temperature : up to 4.0V
     *  - Normal : up to 4.2V
     */
    // Set maximum charging voltage
    if(batt_info->temperature < TEMP_CRITICAL_LOWER ||
       batt_info->temperature > TEMP_CRITICAL_UPPER) {		// Critical Temperature! Must stop charging
		start_monitor = 0;
		set_end_of_charge(0);
       	charger_ic_disable();
        return 0;
    } else if(batt_info->temperature < TEMP_LIMIT_LOWER ||
              batt_info->temperature > TEMP_LIMIT_UPPER) {	// Charging Limit
		trickle_chg_max = 4000;
		trickle_chg_min = 3900;
		set_end_of_charge(1);
	} else {									// Normal Charging
#if defined (CONFIG_BSSQ_CHARGER_RT)
		trickle_chg_max = EOC_VOLTAGE; //4220;					// to unintentional charging stop
		trickle_chg_min = EOC_VOLTAGE - 60; //4140;
#else
		trickle_chg_max = 4220;					// to unintentional charging stop
		trickle_chg_min = 4140;
#endif
		if(batt_info->prev_temperature < TEMP_LIMIT_LOWER ||
		   batt_info->prev_temperature > TEMP_LIMIT_UPPER) {
			start_monitor = 0;
			set_end_of_charge(0);
		}
	}

	if(!end_of_charge)
		return 0;

	if(batt_info->capacity == 100) {
		start_monitor = 1;
		batt_info->charge_status = POWER_SUPPLY_STATUS_FULL;
	} else {
		printk(KERN_DEBUG "[BATT] %s - NOT EOC !!!!! capacity(%d)",__func__, batt_info->capacity);
		start_monitor = 0;
		batt_info->charge_status = POWER_SUPPLY_STATUS_CHARGING;
#if defined (CONFIG_HW_EVENT)
		if(batt_info->temperature < 0) {
			//printk(KERN_DEBUG "[BATT] %s - NOT EOC !!!!! temperature(%d)",__func__, batt_info->temperature);
#if defined (CONFIG_BSSQ_CHARGER_RT)
			charger_ic_set_irq(0);
#endif
			if(status == CHARGER_DISBALE) {
				switch(batt_info->charge_type) {
					case POWER_SUPPLY_TYPE_MAINS:
						charger_ic_set_mode(CHARGER_ISET);
						break;
					case POWER_SUPPLY_TYPE_USB:
						charger_ic_set_mode(CHARGER_USB500);
						break;
					case POWER_SUPPLY_TYPE_UPS:
						charger_ic_set_mode(CHARGER_FACTORY);
						break;
					default:
						break;
				}
			}
			return 0;
		}
#endif
	}

	// Trickle charging
	if(batt_info->voltage >= trickle_chg_max) {
		if(status != CHARGER_DISBALE)
			charger_ic_disable();
	}
	else if(batt_info->voltage < trickle_chg_min) {	// active charger for recharging
#if defined (CONFIG_BSSQ_CHARGER_RT)
		charger_ic_set_irq(1);
#endif
		switch(batt_info->charge_type) {
			case POWER_SUPPLY_TYPE_MAINS:
				charger_ic_set_mode(CHARGER_ISET);
				break;
			case POWER_SUPPLY_TYPE_USB:
				charger_ic_set_mode(CHARGER_USB500);
				break;
			case POWER_SUPPLY_TYPE_UPS:
				charger_ic_set_mode(CHARGER_FACTORY);
				break;
			default:
				break;
		}
	}
	return 0;
}

#if defined (CONFIG_SU880) || defined (CONFIG_KU8800)
extern int temp_cpu;
#endif
static void battery_update_changes(struct battery_info *batt_info)
{
	battery_update(batt_info);
	if(!test_mode)
		set_battery_charging(batt_info);

	if(batt_info->present) { // Adjust RCOMP for fuelgauge
		max17043_set_rcomp_by_temperature(batt_info->temperature);
	}

#if defined (CONFIG_SU880) || defined (CONFIG_KU8800)
	temp_cpu = batt_info->temperature;
#endif

//ntyeongon.moon 2003-03-23 detect charging mode [START_LGE]
//    if (power_supply_am_i_supplied(&batt_info->bat))
//    	batt_info->charge_status = POWER_SUPPLY_STATUS_CHARGING;
//ntyeongon.moon 2003-03-23 detect charging mode [END_LGE]

//	if (batt_info->temperature < -50 || batt_info->temperature > 500 || batt_info->capacity > 95 || batt_info->capacity < 5)
		printk(KERN_DEBUG "[BATT] voltage(%d|%d) temperature(%d) capacity(%d) present(%d) [CHARGER] type(%d) status(%d) \n", \
					batt_info->voltage, max17043_get_voltage(), batt_info->temperature, batt_info->capacity, batt_info->present, \
					batt_info->charge_type, batt_info->charge_status);

	power_supply_changed(&batt_info->bat);
}

#if defined(CONFIG_MACH_BSSQ)
void notification_of_changes_to_battery(void)
{
	if(refer_batt_info == NULL)
		return;

	// Update Battery Information
	cancel_delayed_work_sync(&refer_batt_info->bssq_monitor_work);
	battery_update_changes(refer_batt_info);
	schedule_delayed_work(&refer_batt_info->bssq_monitor_work, MONITOR_WORK_TIME);
}
EXPORT_SYMBOL(notification_of_changes_to_battery);
#endif

static void battery_work(struct work_struct *work)
{
	struct battery_info *batt_info = container_of(work,
			struct battery_info, bssq_monitor_work.work);

	battery_update_changes(batt_info);
	schedule_delayed_work(&batt_info->bssq_monitor_work, 15 * HZ);
}


#define to_battery_info(x) container_of((x), \
			struct battery_info, bat);

static int battery_get_property(struct power_supply *psy,
					    enum power_supply_property psp,
					    union power_supply_propval *val)
{
	struct battery_info *batt_info;

	batt_info = to_battery_info(psy);

	mutex_lock(&battery_mutex);

	switch (psp) {
		case POWER_SUPPLY_PROP_STATUS:
			if(test_mode)
			{
				if(charger_ic_get_status() == CHARGER_ISET)
					val->intval = POWER_SUPPLY_STATUS_CHARGING;
				else
					val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
			}
			else
			{
#if 1 //MBksjung stauts change
				if (batt_info->present == 0)
				{
					val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
					//			DBG("BATTERY_STATUS_NO_BATTERY(%d)", val->intval);
				}
				else if ((batt_info->capacity == 100) 
						&& (charger_ic_get_status() != CHARGER_DISBALE))
				{
					val->intval = POWER_SUPPLY_STATUS_FULL;
					//			DBG("STATUS_FULL(%d)", val->intval);
				}
				else if (charger_ic_get_status() != CHARGER_DISBALE) // POWER_SUPPLY_STATUS_CHARGING
				{
					val->intval = POWER_SUPPLY_STATUS_CHARGING;
					//			DBG("STATUS_CHARGING(%d)", val->intval);
				}
				else if (charger_ic_get_status() == CHARGER_DISBALE) // NVODM_BATTERY_STATUS_DISCHARGING
				{
					val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
					//			DBG("STATUS_DISCHARGING(%d)", val->intval);
				}
				else 
				{
					val->intval = batt_info->charge_status;
					//			DBG("[Critical] Why this is occured?? Battery Status Unknown");
				}

#else
				val->intval = batt_info->charge_status;
#endif				
			}
			break;
		case POWER_SUPPLY_PROP_VOLTAGE_NOW:
			val->intval = battery_read_voltage(batt_info); // hg.park@lge.com current value set!
			break;
		case POWER_SUPPLY_PROP_TEMP:
			if(batt_info->present == 0)		// No Battery or Dummy Battery
				val->intval = 200;
			else
				val->intval = batt_info->temperature;
			break;
		case POWER_SUPPLY_PROP_ONLINE:
			if(test_mode){
				if(charger_ic_get_status() == CHARGER_ISET)
					val->intval = POWER_SUPPLY_TYPE_MAINS;
				else
					val->intval = POWER_SUPPLY_TYPE_BATTERY;
			}else{
				val->intval = batt_info->charge_type;
			}
//2011-03-23 ntyeongon.moon@lge.com get muic INT_STAT  VBUS [START_LGE]
//			 if(!is_muic_mvbus_on()) val->intval = 0;
//2011-03-23 ntyeongon.moon@lge.com get muic INT_STAT  VBUSVBUS [END_LGE]
			break;
		case POWER_SUPPLY_PROP_PRESENT:
#if defined(CONFIG_BSSQ_MUIC_TI)
			val->intval = batt_info->present;
			if (!val->intval)
			{
				MUIC_CHARGER_TYPE muic_mode = MUIC_CHARGER_NONE;
				muic_mode = get_muic_charger_type();
				if(muic_mode == MUIC_CHARGER_FACTORY) 	// JIG or  Dummy Battery
					val->intval = 1;
			}
#endif  // CONFIG_BSSQ_MUIC_TI
            break;

		case POWER_SUPPLY_PROP_CAPACITY:
			val->intval = battery_read_capacity();
			break;
#if defined(CONFIG_MACH_BSSQ)	/* LGE_CHANGE [HUB: newcomet@lge.com] on 2009-9-23, for <HAL:battery info. path> */
			/* FIXME : It depends on H/W specific */
		case POWER_SUPPLY_PROP_HEALTH:
			if(!batt_info->present)
				val->intval = POWER_SUPPLY_HEALTH_UNKNOWN;
			else if(batt_info->voltage > 5000)				// Over voltage
				val->intval = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
			else if(batt_info->temperature < TEMP_CRITICAL_LOWER)	// Cold
				val->intval = POWER_SUPPLY_HEALTH_COLD;
			else if(batt_info->temperature > TEMP_CRITICAL_UPPER)	// Hot
				val->intval = POWER_SUPPLY_HEALTH_OVERHEAT;
			else
				val->intval = POWER_SUPPLY_HEALTH_GOOD;
			break;
		case POWER_SUPPLY_PROP_TECHNOLOGY:
			val->intval = POWER_SUPPLY_TECHNOLOGY_LION;	// Fixed value : Li-ion
			break;
#endif  // CONFIG_MACH_BSSQ

		default:
			mutex_unlock(&battery_mutex);
			return -EINVAL;
	}

	mutex_unlock(&battery_mutex);
	return 0;
}

#if defined(CONFIG_MACH_BSSQ)	/* LGE_CHANGE [HUB: newcomet@lge.com] on 2009-9-29, for <HAL:battery info. path> */
#define to_bssq_ac_device_info(x) container_of((x), \
		struct battery_info, ac);

#define to_bssq_usb_device_info(x) container_of((x), \
		struct battery_info, usb);

static int bssq_ac_battery_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	switch (psp) {
		case POWER_SUPPLY_PROP_ONLINE:
#if defined(CONFIG_BSSQ_MUIC_TI)
		{
			MUIC_CHARGER_TYPE muic_mode = MUIC_CHARGER_NONE;
			muic_mode = get_muic_charger_type();
			if ( muic_mode == MUIC_CHARGER_TA )
				val->intval = 1;
			else
				val->intval = 0;
			break;
		}
#else
			val->intval = 0;
			break;
#endif
		default:
			return -EINVAL;
	}

	return 0;
}

static int bssq_usb_battery_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	switch (psp) {
		case POWER_SUPPLY_PROP_ONLINE:
#if defined(CONFIG_BSSQ_MUIC_TI)
		{
			MUIC_CHARGER_TYPE muic_mode = MUIC_CHARGER_NONE;
			muic_mode = get_muic_charger_type();
			if ( muic_mode == MUIC_CHARGER_USB )
				val->intval = 1;
			else
				val->intval = 0;
			break;
		}
#else
			val->intval = 1;
			break;
#endif
		default:
			return -EINVAL;
	}

	return 0;
}
#endif

/*
 * This attribute file for testmode 250-25-2
 */
ssize_t show_complete(struct device *dev, struct device_attribute *attr, char *buf)
{
	printk("[TESTMODE] %s\n",__func__);
	return snprintf(buf, PAGE_SIZE, "%d\n", (end_of_charge)? 1: 0);
}

DEVICE_ATTR(complete,0644,show_complete,NULL);

/*
 * This attribute file for testmode 250-25-3
 */
ssize_t charger_show_status(struct device *dev,
             struct device_attribute *attr,
             char *buf)
{
    if(charger_ic_get_status() == CHARGER_DISBALE){
		printk("[TESTMODE] %s : CHARGER_DISBALE\n",__func__);
        return snprintf(buf, PAGE_SIZE, "1\n");
    }else{
		printk("[TESTMODE] %s : CHARGER_ISET\n",__func__);
        return snprintf(buf, PAGE_SIZE, "0\n");
	}
}
ssize_t charger_store_status(struct device *dev,
              struct device_attribute *attr,
              const char *buf,
              size_t count)
{
	test_mode = TRUE;
	end_of_charge = 0;
	static spinlock_t test_lock;
	static int skip_flag = 0;

	spin_lock(&test_lock);
		if (skip_flag)
			return;
		skip_flag = 1;
	spin_unlock(&test_lock);

    if(buf[0] == '1') {
		printk("[TESTMODE] %s : set HARGER_DISBALE\n",__func__);
        charger_ic_disable();
    } else if(buf[0] == '0') {
        printk("[TESTMODE] %s : set CHARGER_ISET\n",__func__);
        charger_ic_set_mode(CHARGER_ISET);
    }
	notification_of_changes_to_battery();

	skip_flag = 0;
    return count;
}

#if defined(TEMP_CONTROL)
/* HW requirement: temp control  _S*/
static ssize_t temp_control_show_property(
		struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct battery_info *batt_info = refer_batt_info;
	printk("[TEMP CONTROL]. [%d]", batt_info->temp_control);
	return sprintf(buf, "%d\n", batt_info->temp_control);
}
static ssize_t temp_control_store_property(
		struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t count)
{
		struct battery_info *batt_info = refer_batt_info;

		static unsigned int value = 0;

		value = (unsigned int)(simple_strtoul(buf, NULL, 0));
	
		printk("[TEMP CONTROL] Value : [%d]", value);
		batt_info->temp_control = value;
        printk("[TEMP CONTROL] 1 is On, 0 is OFF :result[%d]", batt_info->temp_control);
		printk("[TEMP CONTROL] Battery Update Now!!!!", 0);
		battery_update_changes(batt_info);
		return count;

}
/* HW requirement: temp control  _E*/

#endif
DEVICE_ATTR(charger_state, 0644, charger_show_status, charger_store_status);
#if defined(TEMP_CONTROL)
DEVICE_ATTR(temp_control, S_IRUGO | S_IWUGO, temp_control_show_property, temp_control_store_property);/* HW requirement: temp control  */	
#endif
static char *bssq_supplied_to[] = {
	"battery",
	"ac",
	"usb",
};

extern void tegra_cpu_lock_speed(int min_rate, int timeout_ms);

static int __init battery_probe(struct platform_device *pdev)
{
	struct max8907c *max8907c = dev_get_drvdata(pdev->dev.parent);
	struct battery_info *batt_info;
	int irq;
	int ret;

	batt_info = kzalloc(sizeof(*batt_info), GFP_KERNEL);
	if (!batt_info)
		return -ENOMEM;

	batt_info->dev = &pdev->dev;

	batt_info->bat.name = "battery";
	batt_info->bat.supplied_to = bssq_supplied_to;
	batt_info->bat.num_supplicants = ARRAY_SIZE(bssq_supplied_to);
	batt_info->bat.type = POWER_SUPPLY_TYPE_BATTERY;
	batt_info->bat.properties = battery_props;
	batt_info->bat.num_properties = ARRAY_SIZE(battery_props);
	batt_info->bat.get_property = battery_get_property;
	batt_info->bat.external_power_changed = NULL;
	batt_info->bat.set_charged = NULL;

	batt_info->ac.name = "ac";
	batt_info->ac.type = POWER_SUPPLY_TYPE_MAINS;
	batt_info->ac.properties = bssq_ac_usb_battery_props;
	batt_info->ac.num_properties = ARRAY_SIZE(bssq_ac_usb_battery_props);
	batt_info->ac.get_property = bssq_ac_battery_get_property;
	batt_info->ac.external_power_changed = NULL;
	batt_info->ac.set_charged = NULL;

	batt_info->usb.name = "usb";
	batt_info->usb.type = POWER_SUPPLY_TYPE_USB;
	batt_info->usb.properties = bssq_ac_usb_battery_props;
	batt_info->usb.num_properties = ARRAY_SIZE(bssq_ac_usb_battery_props);
	batt_info->usb.get_property = bssq_usb_battery_get_property;
	batt_info->usb.external_power_changed = NULL;
	batt_info->usb.set_charged = NULL;

	batt_info->i2c_adc = max8907c->i2c_adc;
	batt_info->i2c_power = max8907c->i2c_power;

#if defined(TEMP_CONTROL)
	batt_info->temp_control = TEMP_CONTROL_OFF;
#endif

	platform_set_drvdata(pdev, batt_info);

#if 0	// hyeongwon.oh

	/* enabling GPCH09 for read back battery voltage */
// 	hyeongwon.oh
//	ret = bssqbackupbatt_voltage_setup();
	if (ret)
		goto voltage_setup_fail;

	/* REVISIT do we need to request both IRQs ?? */

	/* request BCI interruption */
	irq = platform_get_irq(pdev, 1);
	ret = request_irq(irq, battery_interrupt,
		0, pdev->name, NULL);
	if (ret) {
		dev_dbg(&pdev->dev, "could not request irq %d, status %d\n",
			irq, ret);
		goto batt_irq_fail;
	}

	/* request Power interruption */
	irq = platform_get_irq(pdev, 0);
	ret = request_irq(irq, bssq_charger_interrupt,
		0, pdev->name, batt_info);

	if (ret) {
		dev_dbg(&pdev->dev, "could not request irq %d, status %d\n",
			irq, ret);
		goto chg_irq_fail;
	}
#endif

#if 0 //defined (CONFIG_MFD_MAX8907C) // hg.park@lge.com
	ret = max8907c_reg_write(batt_info->i2c_power, MAX8907_LBCNFG, MAX8907_CUTTOFF_33V400);
	if(ret < 0)
	{
		printk("[MAX8907C] Low Battery DAC Falling Threshold Set error!\n");
		goto cuttoff_threshold_failed;
	}
#endif

	ret = power_supply_register(&pdev->dev, &batt_info->bat);
	if (ret) {
		dev_dbg(&pdev->dev, "failed to register main battery\n");
		goto batt_failed;
	}

	INIT_DELAYED_WORK_DEFERRABLE(&batt_info->bssq_monitor_work, battery_work);

	schedule_delayed_work(&batt_info->bssq_monitor_work, 5 * HZ);

	ret = power_supply_register(&pdev->dev, &batt_info->ac);
	if (ret) {
		dev_dbg(&pdev->dev, "failed to register battery ac online\n");
		goto ac_online_failed;
	}

	ret = power_supply_register(&pdev->dev, &batt_info->usb);
	if (ret) {
		dev_dbg(&pdev->dev, "failed to register battery usb online\n");
		goto usb_online_failed;
	}

	ret = device_create_file(&pdev->dev, &dev_attr_complete);
	if (ret < 0) {
		pr_err("%s:File device creation failed: %d\n", __func__, ret);
		ret = -ENODEV;
		goto file_create_fail;
	}

#if defined(READ_TEMP_ADC)
	ret = device_create_file(&pdev->dev, &dev_attr_readtempadc);
	if (ret < 0) {
		pr_err("%s:File device creation failed: %d\n", __func__, ret);
		ret = -ENODEV;
		goto file_create_fail;
	}
#endif

    // for AT Command AT%CHARGE
    // sysfs path : /sys/devices/platform/tegra-i2c.3/i2c-4/4-003c/bssq_battery/charger_state
    ret = device_create_file(&pdev->dev, &dev_attr_charger_state);
    if (ret < 0) {
        pr_err("%s:File device creation failed: %d\n", __func__, ret);
        ret = -ENODEV;
        goto charger_state_create_fail;
    }

#if defined(TEMP_CONTROL)
/* Battery Temp control Info. */
	ret = device_create_file(&pdev->dev, &dev_attr_temp_control);
	if (ret)
	{
		goto temp_control_file_create_fail;
	}
#endif
	// Update First Battery Information
	refer_batt_info = batt_info;

	battery_update(batt_info);

	printk("[BATT] %s is OK\n",__func__);

	return 0;

#if defined(TEMP_CONTROL)
temp_control_file_create_fail:
	device_remove_file(&pdev->dev, &dev_attr_temp_control);	
#endif
charger_state_create_fail:
	device_remove_file(&pdev->dev, &dev_attr_complete);
file_create_fail:
	power_supply_unregister(&batt_info->usb);
usb_online_failed:
	power_supply_unregister(&batt_info->ac);
ac_online_failed:
	power_supply_unregister(&batt_info->bat);
#if 0
batt_failed:
	free_irq(irq, batt_info);
chg_irq_fail:
	irq = platform_get_irq(pdev, 1);
	free_irq(irq, NULL);
#endif
batt_failed:
	kfree(batt_info);
#if defined (CONFIG_MFD_MAX8907C)
cuttoff_threshold_failed:
#endif

	return ret;
}

static int __exit battery_remove(struct platform_device *pdev)
{
	struct battery_info *batt_info = platform_get_drvdata(pdev);
#if 0
	int irq;
	irq = platform_get_irq(pdev, 0);
	free_irq(irq, batt_info);

	irq = platform_get_irq(pdev, 1);
	free_irq(irq, NULL);
#endif
	flush_scheduled_work();
	power_supply_unregister(&batt_info->bat);
	power_supply_unregister(&batt_info->ac);
	power_supply_unregister(&batt_info->usb);
	device_remove_file(&pdev->dev, &dev_attr_complete);
	device_remove_file(&pdev->dev, &dev_attr_charger_state);
#if defined(TEMP_CONTROL)
    device_remove_file(&pdev->dev, &dev_attr_temp_control);
#endif
	platform_set_drvdata(pdev, NULL);
	kfree(batt_info);

	return 0;
}

static int battery_suspend(struct platform_device *pdev,
	pm_message_t state)
{
	struct battery_info *batt_info = platform_get_drvdata(pdev);

	cancel_delayed_work_sync(&batt_info->bssq_monitor_work); //20111031 hyongbink@nvidia.com : cancel_delayed_work -> cancel_delayed_work_sync 
	batt_info->charge_status = POWER_SUPPLY_STATUS_UNKNOWN; //20101116 taehwan.kim@lge.com , 20111031 hyongbink@nvidia.com : change position. 

	return 0;
}

static int battery_resume(struct platform_device *pdev)
{
	struct battery_info *batt_info = platform_get_drvdata(pdev);
	battery_update_changes(batt_info);

	schedule_delayed_work(&batt_info->bssq_monitor_work, 15 * HZ);

	return 0;
}

static struct platform_driver battery_driver = {
	.probe		= battery_probe,
	.remove		= __exit_p(battery_remove),
	.suspend	= battery_suspend,
	.resume		= battery_resume,
	.driver		= {
		.name	= "bssq_battery",
	},
};

static int __init battery_init(void)
{
	return platform_driver_register(&battery_driver);
}

static void __exit battery_exit(void)
{
	platform_driver_unregister(&battery_driver);
}

module_init(battery_init);
module_exit(battery_exit);

MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:battery");
MODULE_AUTHOR("LGE");
