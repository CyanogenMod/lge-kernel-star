/*
 * battery driver for LGE Battery 
 *
 * Copyright (C) 2012 LGE, Inc.
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
#include <linux/hrtimer.h>
#include <linux/delay.h>
#include <linux/time.h>
#include <linux/slab.h>
//#include <linux/rtc.h> 
#include <linux/mfd/max8907c.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>

/* Include Related Files */
#include <linux/muic.h>
#include <linux/max8922l.h>
#include <linux/mfd/max8907c.h>
#include <linux/su660_battery.h>
//#include <linux/mfd/max8907_rtc.h>
#include "su660_battery_temp.h"

/* Extern MUIC Function */
extern TYPE_CHARGING_MODE get_muic_charger_type(void);

#define TRICKLE_RECHECK
#define BATT_DEBUG
#define BATT_RTC 0//MBksjung 2012.04.24: remove for battery rtc

#define TEMP_CONTROL_ON 1
#define TEMP_CONTROL_OFF 0
asmlinkage int bprintk(const char *fmt, ...)
{
	va_list args;
	int r;
	va_start(args, fmt);
	r = vprintk(fmt, args);
	va_end(args);
	return r;
}
#ifdef  BATT_DEBUG 
#define DBG(fmt, arg...) bprintk("[BATTERY] : %s : " fmt "\n", __func__, ## arg)
#else
#define DBG(fmt, arg...) do {} while (0)
#endif

#define TEMP_CRITICAL_UPPER	(550)
#define TEMP_CRITICAL_LOWER	(-100)
#define TEMP_LIMIT_UPPER	(450)
#define TEMP_LIMIT_LOWER	(-50)

#if BATT_RTC
/* Base Year For Linux */
#define LINUX_RTC_BASE_YEAR	1900
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

static enum {
	CHGSB_PGB_OFF_OFF = 0,
	CHGSB_PGB_ON_ON,
	CHGSB_PGB_OFF_ON,
	CHGSB_PGB_ON_OFF, 
};

static enum power_supply_property battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_ONLINE,
};

/* Charger String For Debug */
static const char *charger_ic_status_name[] = {
	"CHARGER_USB500",
	"CHARGER_ISET",
	"CHARGER_USB100",
	"CHARGER_FACTORY",
	"CHARGER_DISABLE"
};

/* Charger State String For Debug */
static const char *charger_ic_state_machine_name[] = {
	"CHARGER_STATE_SHUTDOWN",
	"CHARGER_STATE_STANDBY",
	"CHARGER_STATE_CHARGE",
	"CHARGER_STATE_FULLBATTERY",
	"CHARGER_STATE_RECHARGE"
};


/* BatteryHealth String For Debug */
static const char *BatteryHealth_name[] = {
	"UNKNOWN",
	"GOOD",
	"OVERHEAT",
	"DEAD",
	"OVERVOLTAGE",
	"UNSPEC_FAILURE",
	"COLD",
	"CRITICAL_OVERHEAT",
};

/* ChargerType_name String For Debug */
static const char *ChargerType_name[] = {
	"Battery",
	"Factory",
	"AC",
	"USB",
	"USB_DCP",
	"USB_CDP",
	"USB_ACA",
};


static enum power_supply_property ac_usb_battery_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

struct battery_info {
	struct device		*dev;

	struct power_supply	ac;
	struct power_supply	usb;
	struct power_supply	bat;
	struct power_supply	usb_bat;

	struct workqueue_struct	*battery_workqueue;
	struct delayed_work	star_monitor_work;
	struct delayed_work	star_id_monitor_work;
        unsigned int		temp_control; //for Charging test

	unsigned long		update_time;
	unsigned long		last_cbc_time;
	int			RIL_ready;
	int			boot_TA_setting;
	int			charge_setting_chcomp;

#if BATT_RTC 
	/* RTC Related Variables */
	unsigned int		old_alarm_sec;
	unsigned int		old_checkbat_sec;
#endif
	/* Polling Related Variables */
	int			polling_interval;
	int			id_polling_interval;
	int			sleep_polling_interval;

	/* Capacity References */
	int 			gauge_on;
	int			CBC_value;
	int			capacity_gauge;
	int			capacity_voltage;
	int			capacity_first_time;

	/* Battery Information Variables */
	int				charge_type;
	int				charge_status;
	int				present;
	int				voltage;
	int				temperature;
	int				capacity;
	int				health;

	int				prev_charge_type;
	int				prev_charge_status;
	int				prev_present;
	int				prev_voltage;
	int				prev_temperature;
	int				prev_capacity;
	int				prev_health;

	int				force_update;
#if defined (TRICKLE_RECHECK)
	unsigned char			trickle_charge_check;
#endif
	int 			voltage_now;
	int 			high_temp_overvoltage;

};
static struct battery_info *refer_batt_info = NULL;

/*for battery update*/
extern g_is_suspend;
/* Battery Mutex */
static DEFINE_MUTEX(battery_mutex);

/* Some Globals */
static unsigned int bat_shutdown = 0;
static unsigned int previous_gauge = 100;

/* AT Cmd. Device */
static int at_boot_state = 0;
static unsigned char at_charge_on = 0;
static unsigned char at_charge_comp = 0; 
static unsigned char at_charge_index = 0;
static unsigned char ELT_test_mode = 0;

//MOBII_CHNANGE_S 20120716 jd.park@mobii.co.kr : FOTA UA Upgrade for ICS
#if defined(CONFIG_MACH_STAR_P990) || defined(CONFIG_MACH_STAR_SU660) || defined(CONFIG_MACH_STAR_P999)
static int ignore_send_uevent = FALSE; // for FOTA
#endif
//MOBII_CHNANGE_E 20120716 jd.park@mobii.co.kr : FOTA UA Upgrade for ICS

/* AT Cmd. */
unsigned char ARRAY_TP_BOOT(void)
{
	return at_boot_state;
}
EXPORT_SYMBOL(ARRAY_TP_BOOT);

static unsigned int get_current_time(void)
{
	struct timespec ts;
	signed long	nsec;
	getnstimeofday(&ts);
	nsec = timespec_to_ns(&ts);
	do_div(nsec, 1000000);
	return (unsigned int) nsec;
}

/* Static Declares */
static void star_gauge_follower_func(void);
static void charger_contol_with_battery_temp(void);
static void star_capacity_from_voltage_via_calculate(void);

static int battery_read_temperature()
{
	static unsigned short BAT_TEMP_TABLE[] = BAT_T_TABLE;
	unsigned int mili_temp;
	unsigned int temp;
#if 0 //temp table
	max8907c_adc_battery_read_temp(&mili_temp);
	temp = BAT_TEMP_TABLE[mili_temp];

	if ((at_boot_state == 1) 
			|| (at_charge_index == 1) 
			|| (ELT_test_mode == 1))
		temp = 270; 		// Factory Test Mode, fake Temperature value
	else
		temp = temp - 2730; // Conversion from [K] to [C] <- *10
#else
max8907c_adc_battery_read_temp(&mili_temp);
#if defined(READ_TEMP_ADC)
	TEMP_ADC_VALUE = mili_temp;
#endif

temp = batt_temp_tbl[mili_temp];
if ((at_boot_state == 1) 
		|| (at_charge_index == 1)
		|| (ELT_test_mode == 1))
	temp = 270; 		// Factory Test Mode, fake Temperature value

#endif
	return (int)temp;	// Temp
}

static int battery_read_voltage()
{
	int voltage = 0;

	max8907c_adc_battery_read_volt(&voltage);
	return voltage;
}

/*
 * Return battery capacity
 * Or < 0 on failure.
 */
static int battery_read_capacity(void)
{
	int ret = 0;
	ret = refer_batt_info->capacity;
	//	DBG("Cap [%d]", refer_batt_info->capacity);
	return ret;
}

/*
                                                      
                               
                     
 */
int is_no_batt_or_dummy=0;
int battery_check_present(void)
{
	if(refer_batt_info == NULL)
	{
		DBG("No refer_batt_info !");
		return 0;
	}

	if(refer_batt_info->temperature <  -400 || refer_batt_info->temperature > 900) // // Temperature range -40 ~ 90 [C]
	{
	is_no_batt_or_dummy=1;
			return 0;
	}
	return 1;
}
EXPORT_SYMBOL(battery_check_present);

static void battery_update(struct battery_info *batt_info)
{
	TYPE_CHARGING_MODE muic_mode;

	mutex_lock(&battery_mutex);

	/* Backup Battery Info */	
	batt_info->prev_voltage			= batt_info->voltage;
	batt_info->prev_temperature		= batt_info->temperature;
	batt_info->prev_capacity		= batt_info->capacity;
	batt_info->prev_present			= batt_info->present;

	/* 
	 * Read Battery Status : Voltage / Temp / Cap / Presence
	 */	
	batt_info->voltage		= battery_read_voltage();			// Read Voltage
	batt_info->temperature		= battery_read_temperature();			// Read Temperature
	batt_info->capacity 		= battery_read_capacity();			// Read Capacity
	batt_info->present		= battery_check_present();			// Set Battery Present
	//temp resize
	
	if(batt_info->temperature < 0 )
	{
		batt_info->temperature = batt_info->temperature - 60;
	}
	else if(batt_info->temperature >= 0)
	{
		batt_info->temperature = batt_info->temperature - 20;
	}
//	DBG("Read battery status success. [%d] [%d] [%d] [%d]"
//			, batt_info->voltage, batt_info->temperature
//			, batt_info->capacity, batt_info->present);

	/*
	 * Look Over Current Charger and Restate of Charger type 
	 */
	muic_mode = get_muic_charger_type();
	switch (muic_mode) {
		case CHARGING_USB:
			batt_info->charge_type = POWER_SUPPLY_TYPE_USB;
			break;
		case CHARGING_NA_TA:
		case CHARGING_LG_TA:
		case CHARGING_TA_1A:
			batt_info->charge_type = POWER_SUPPLY_TYPE_MAINS;
			break;
		case CHARGING_FACTORY:
			batt_info->charge_type = POWER_SUPPLY_TYPE_UPS;
			break;
		case CHARGING_NONE:
		default:
#if defined (TRICKLE_RECHECK)
			batt_info->trickle_charge_check = 0;
#endif
			batt_info->charge_type = POWER_SUPPLY_TYPE_BATTERY;
			break;
	}
//	DBG("Get MUIC mode [%d].", muic_mode);

	/* AT Boot State */
	if( (at_boot_state == 1) && (batt_info->present == 0) /*|| is_no_batt_or_dummy*/) 
		batt_info->present = 1;


	/* Battery Info. Calibration */
	if( (3200 < batt_info->voltage) && (batt_info->voltage< 4400) ) {
		if (batt_info->capacity_first_time) {
			batt_info->voltage = batt_info->voltage;
			batt_info->capacity_first_time = 0;
		} else {
			if( 3400 >= batt_info->voltage )
				batt_info->voltage = batt_info->voltage;
			else if( (3450 < batt_info->voltage) && (4050 > batt_info->voltage) )
				batt_info->voltage = (batt_info->prev_voltage*3 + batt_info->voltage)/4; 
			else
				batt_info->voltage = (batt_info->prev_voltage + batt_info->voltage)/2; 
		} // End if(batt_info->capaci.... )
	} 
	else 
	{
		DBG("Voltage Range is out of Normal[O:%d, N:%d]", batt_info->prev_voltage, batt_info->voltage);
		if (3200 > batt_info->voltage) 
		{
			if ( 3400 >= batt_info->prev_voltage ) 
			{
				batt_info->voltage = batt_info->voltage;
			} 
			else 
			{
				int tmp_volt = battery_read_voltage();
				int tmp_temp = battery_read_temperature(); 
				//	if ( ) // Recheck Battery Data
				{
					DBG("Recheck Battery [O:%d, N:%d]", batt_info->voltage, tmp_volt);
					batt_info->voltage = tmp_volt; 
				}
			} // End if( 3400 >= .... )
		} // end if(3200 > ....)
	}
	// Capacity check through Voltage 
	if (batt_info->voltage <= 3400) {
		if (batt_info->capacity <= 1 )		batt_info->capacity = 0;
		else if (batt_info->capacity > 1)	batt_info->capacity_gauge = 0;
	}

	/* Factory Test Mode Fake Temp */
	if (( at_boot_state == 1 ) || ( at_charge_index == 1 ) || ( ELT_test_mode == 1 )/*|| batt_info->temp_control == TEMP_CONTROL_ON*/ /*|| is_no_batt_or_dummy*/)
	{
	
		batt_info->temperature = 270; 
		if(batt_info->temp_control == TEMP_CONTROL_ON)
		{
			DBG("[TEMP CONTROL]Unlimited Temp Charging start.", 0);
		}
	}

//	DBG("Read battery status success. [%d] [%d] [%d] [%d]"
//			, batt_info->voltage, batt_info->temperature
//			, batt_info->capacity, batt_info->present);
	
	mutex_unlock(&battery_mutex);
}

#if defined(CONFIG_MACH_STAR_P990)
static void charger_contorl_unlimited_temp(void)
{
	struct battery_info *batt_info = refer_batt_info;
	TYPE_CHARGING_MODE muic_mode;
	muic_mode = get_muic_charger_type();

	DBG("charger_contorl_unlimited_temp present=%d",batt_info->present);
	if ( batt_info->present == 1 ) 
	{
		batt_info->prev_health = batt_info->health;

		DBG("charger_contorl_unlimited_temp present=%d",batt_info->health);
		switch ( batt_info->health ) 
		{
			case POWER_SUPPLY_HEALTH_GOOD:
				if (batt_info->temperature >= 750) {
					// Deactivate Charger : Battery Critical Overheat
					DBG("POWER_SUPPLY_HEALTH_GOOD 750 over");
					batt_info->health = POWER_SUPPLY_HEALTH_CRITICAL_OVERHEAT;
				}
				else if ((batt_info->temperature >= 550) && (batt_info->temperature < 750)) {
					// Change Charger Setting : Battery Overheat, USB_500 mode
					DBG("POWER_SUPPLY_HEALTH_GOOD 550~750 ");
					batt_info->health = POWER_SUPPLY_HEALTH_OVERHEAT;
					if ( charger_ic_get_status() != CHARGER_DISABLE ) {
						if ( charger_ic_get_state() != CHARGER_STATE_FULLBATTERY ) 
						{
							DBG("POWER_SUPPLY_HEALTH_GOOD CHARGER_STATE_CHARGE 550~750 ");
							batt_info->charge_setting_chcomp = charger_ic_get_status();
							charger_ic_set_state(CHARGER_STATE_CHARGE);
							charger_ic_set_mode(CHARGER_USB500);
						}
					}
				} else if (batt_info->temperature <= (-100)) {
					// Deactivate Charger : Battery Cold
					DBG("POWER_SUPPLY_HEALTH_GOOD temperature -100 ");
					batt_info->health = POWER_SUPPLY_HEALTH_COLD;
				} else {
					DBG("POWER_SUPPLY_HEALTH_GOOD temperature -100 else ");
					batt_info->health = POWER_SUPPLY_HEALTH_GOOD;
				}
				break;

			case POWER_SUPPLY_HEALTH_OVERHEAT:
				if (batt_info->temperature >= 750) {
					// Deactivate Charger : Battery Critical Overheat
					DBG("POWER_SUPPLY_HEALTH_OVERHEAT 750 over ");
					batt_info->health = POWER_SUPPLY_HEALTH_CRITICAL_OVERHEAT;
				}
				else if(batt_info->temperature >= 520 && batt_info->temperature <= 750)
				{
					DBG("POWER_SUPPLY_HEALTH_OVERHEAT 520~750  ");
					batt_info->health = POWER_SUPPLY_HEALTH_OVERHEAT;
					if ( charger_ic_get_status() != CHARGER_DISABLE ){
						    if (charger_ic_get_state() != CHARGER_STATE_FULLBATTERY) 
							{
								charger_ic_set_state(CHARGER_STATE_CHARGE);
								charger_ic_set_mode(CHARGER_USB500);
								batt_info->high_temp_overvoltage=0;
								DBG("POWER_SUPPLY_HEALTH_OVERHEAT CHARGER_STATE_CHARGE 520~750");

							}	
						}
				}				
				else if (batt_info->temperature <= 520) {
					if ( charger_ic_get_status() != CHARGER_DISABLE ) {
						// Reactivate Charger : Battery Normal again
						DBG("POWER_SUPPLY_HEALTH_OVERHEAT temperature 520 ");
						batt_info->health = POWER_SUPPLY_HEALTH_GOOD;
						if (charger_ic_get_state() != CHARGER_STATE_FULLBATTERY) 
						{
							charger_ic_set_state(CHARGER_STATE_CHARGE);
							charger_ic_set_mode(batt_info->charge_setting_chcomp);
						}
					}
					else
						batt_info->health = POWER_SUPPLY_HEALTH_GOOD;
				} else {
					DBG(" HEALTH_OVERHEAT Battery Critical Overheat");
					batt_info->health = POWER_SUPPLY_HEALTH_OVERHEAT;
				}
				break;

			case POWER_SUPPLY_HEALTH_CRITICAL_OVERHEAT:
				if (batt_info->temperature <= 720) {
					if ( charger_ic_get_status() != CHARGER_DISABLE ) {
						// Reactivate Charger : Battery USB mode again
						DBG("POWER_SUPPLY_HEALTH_CRITICAL_OVERHEAT 720 over");
						batt_info->health = POWER_SUPPLY_HEALTH_OVERHEAT;
						if (charger_ic_get_state() != CHARGER_STATE_FULLBATTERY) 
						{
							charger_ic_set_state(CHARGER_STATE_CHARGE);
							charger_ic_set_mode(CHARGER_USB500);
						}
					}
					else
					batt_info->health = POWER_SUPPLY_HEALTH_OVERHEAT;
				}
				else {
					DBG("POWER_SUPPLY_HEALTH_CRITICAL_OVERHEAT Battery Critical Overheat");
					batt_info->health = POWER_SUPPLY_HEALTH_CRITICAL_OVERHEAT;
				}
				break;

			case POWER_SUPPLY_HEALTH_COLD:
				if ( charger_ic_get_status() != CHARGER_DISABLE ) {
					// Reactivate Charger : Battery Normal again
					DBG("POWER_SUPPLY_HEALTH_COLD Battery");
					batt_info->health = POWER_SUPPLY_HEALTH_GOOD;
					if ( charger_ic_get_state() != CHARGER_STATE_FULLBATTERY ) 
					{
						DBG("POWER_SUPPLY_HEALTH_COLD Battery  CHARGER_STATE_CHARGE");
						charger_ic_set_state(CHARGER_STATE_CHARGE);
						charger_ic_set_mode(batt_info->charge_setting_chcomp);
					}
					
				}
				else
					batt_info->health = POWER_SUPPLY_HEALTH_GOOD;
				
				break;

			default:
				DBG("Battery Unknown Health State");
				batt_info->health = POWER_SUPPLY_HEALTH_UNKNOWN;
				break;
		} // switch end
	}// if end
}// function end
#else
/* Charger Control with Temp. */
static void charger_contorl_unlimited_temp(void)
{
		struct battery_info *batt_info = refer_batt_info;
		TYPE_CHARGING_MODE muic_mode;
		muic_mode = get_muic_charger_type();
		DBG();
		if ( batt_info->present == 1 ) 
		{
			batt_info->prev_health = batt_info->health;
	
			switch ( batt_info->health ) 
			{
				case POWER_SUPPLY_HEALTH_GOOD:
					if (batt_info->temperature >= 550) {
						// Deactivate Charger : Battery Critical Overheat
						DBG("Battery Critical Overheat");
						batt_info->health = POWER_SUPPLY_HEALTH_CRITICAL_OVERHEAT;

					}
					else if ((batt_info->temperature >= 450) && (batt_info->temperature < 550)) {
						// Change Charger Setting : Battery Overheat, USB_500 mode
						DBG("Battery Overheat");
						batt_info->health = POWER_SUPPLY_HEALTH_OVERHEAT;

					} else if (batt_info->temperature <= (-100)) {
						// Deactivate Charger : Battery Cold
						DBG("Battery Cold");
						batt_info->health = POWER_SUPPLY_HEALTH_COLD;

					} else
						batt_info->health = POWER_SUPPLY_HEALTH_GOOD;
					break;
	
				case POWER_SUPPLY_HEALTH_OVERHEAT:
					if (batt_info->temperature >= 550) {
						// Deactivate Charger : Battery Critical Overheat
						DBG("Battery Critical Overheat");
						batt_info->health = POWER_SUPPLY_HEALTH_CRITICAL_OVERHEAT;

					} else if (batt_info->temperature <= 420) {

							batt_info->health = POWER_SUPPLY_HEALTH_GOOD;
					} else {
						DBG("Battery Critical Overheat");
						batt_info->health = POWER_SUPPLY_HEALTH_OVERHEAT;
					}
					break;
	
				case POWER_SUPPLY_HEALTH_CRITICAL_OVERHEAT:
					if (batt_info->temperature <= 520) {
						batt_info->health = POWER_SUPPLY_HEALTH_OVERHEAT;
					}
					else {
						DBG("Battery Critical Overheat");
						batt_info->health = POWER_SUPPLY_HEALTH_CRITICAL_OVERHEAT;
					}
					break;
	
				case POWER_SUPPLY_HEALTH_COLD:
					if (batt_info->temperature >= (-50)) {
							batt_info->health = POWER_SUPPLY_HEALTH_GOOD;
					}
					else {
						DBG("Battery Cold");
						batt_info->health = POWER_SUPPLY_HEALTH_COLD;
					}
					break;
	
				default:
					DBG("Battery Unknown Health State");
					batt_info->health = POWER_SUPPLY_HEALTH_UNKNOWN;
					break;
			} // switch end
		}// if end
}
#endif
static void charger_contol_with_battery_temp(void)
{
	struct battery_info *batt_info = refer_batt_info;
    TYPE_CHARGING_MODE muic_mode;
	muic_mode = get_muic_charger_type();

	if ( batt_info->present == 1 ) 
	{
		batt_info->prev_health = batt_info->health;

		switch ( batt_info->health ) 
		{
			case POWER_SUPPLY_HEALTH_GOOD:
				if (batt_info->temperature >= 550) {
					// Deactivate Charger : Battery Critical Overheat
					DBG("Battery Critical Overheat");
					batt_info->health = POWER_SUPPLY_HEALTH_CRITICAL_OVERHEAT;
					if ( charger_ic_get_status() != CHARGER_DISABLE )
					{
						batt_info->charge_setting_chcomp = charger_ic_get_status();
						charger_ic_disable_for_recharge();
						charger_ic_set_state(CHARGER_STATE_SHUTDOWN);
					}
				}
				else if ((batt_info->temperature >= 450) && (batt_info->temperature < 550)) {
					// Change Charger Setting : Battery Overheat, USB_500 mode
					DBG("Battery Overheat");
					batt_info->health = POWER_SUPPLY_HEALTH_OVERHEAT;
					if ( charger_ic_get_status() != CHARGER_DISABLE ) {
						if ( charger_ic_get_state() != CHARGER_STATE_FULLBATTERY ) 
						{
							batt_info->charge_setting_chcomp = charger_ic_get_status();
							charger_ic_set_state(CHARGER_STATE_CHARGE);
							charger_ic_set_mode(CHARGER_USB500);
						}
					}
				} else if (batt_info->temperature <= (-100)) {
					// Deactivate Charger : Battery Cold
					DBG("Battery Cold");
					batt_info->health = POWER_SUPPLY_HEALTH_COLD;
					if ( charger_ic_get_status() != CHARGER_DISABLE )
					{
						batt_info->charge_setting_chcomp = charger_ic_get_status();
						charger_ic_disable_for_recharge();
						charger_ic_set_state(CHARGER_STATE_SHUTDOWN);
					}
				} else
					batt_info->health = POWER_SUPPLY_HEALTH_GOOD;
				break;

			case POWER_SUPPLY_HEALTH_OVERHEAT:
				if (batt_info->temperature >= 550) {
					// Deactivate Charger : Battery Critical Overheat
					DBG("Battery Critical Overheat");
					batt_info->health = POWER_SUPPLY_HEALTH_CRITICAL_OVERHEAT;
					if ( charger_ic_get_status() != CHARGER_DISABLE ) 
					{
						batt_info->charge_setting_chcomp = charger_ic_get_status();
						charger_ic_disable_for_recharge();
						charger_ic_set_state(CHARGER_STATE_SHUTDOWN);
					}
				}
				else if(batt_info->temperature >= 420 && batt_info->temperature <= 550)
				{
						batt_info->health = POWER_SUPPLY_HEALTH_OVERHEAT;
					if ( charger_ic_get_status() != CHARGER_DISABLE ){
						    if (charger_ic_get_state() != CHARGER_STATE_FULLBATTERY) 
							{
								if(batt_info->voltage >= 4000)
								{
								   	charger_ic_disable_for_recharge();
									charger_ic_set_state(CHARGER_STATE_SHUTDOWN);
									batt_info->high_temp_overvoltage=1;
									DBG("#Battery Overtemp USBmode Overvoltage Charger Disable #");
								}
								else
								{
									charger_ic_set_state(CHARGER_STATE_CHARGE);
									charger_ic_set_mode(CHARGER_USB500);
									batt_info->high_temp_overvoltage=0;
									DBG("#Battery Overtemp USBmode Recovery #");
								}

							}	
						}
				}				
				
				else if (batt_info->temperature <= 420) {
					if ( charger_ic_get_status() != CHARGER_DISABLE ) {
						// Reactivate Charger : Battery Normal again
						DBG("#####################Battery Normal Again#############################");
						batt_info->health = POWER_SUPPLY_HEALTH_GOOD;
						if (charger_ic_get_state() != CHARGER_STATE_FULLBATTERY) 
						{
							charger_ic_set_state(CHARGER_STATE_CHARGE);
							charger_ic_set_mode(batt_info->charge_setting_chcomp);
						}
					}
					else
						batt_info->health = POWER_SUPPLY_HEALTH_GOOD;
				} else {
					DBG("Battery Critical Overheat");
					batt_info->health = POWER_SUPPLY_HEALTH_OVERHEAT;
				}
				break;

			case POWER_SUPPLY_HEALTH_CRITICAL_OVERHEAT:
				if (batt_info->temperature <= 520) {
					if ( charger_ic_get_status() != CHARGER_DISABLE ) {
						// Reactivate Charger : Battery USB mode again
						DBG("#####################Battery USB mode again##########################");
						batt_info->health = POWER_SUPPLY_HEALTH_OVERHEAT;
						if (charger_ic_get_state() != CHARGER_STATE_FULLBATTERY) 
						{
							charger_ic_set_state(CHARGER_STATE_CHARGE);
							charger_ic_set_mode(CHARGER_USB500);
						}
					}
					else
					batt_info->health = POWER_SUPPLY_HEALTH_OVERHEAT;
				}
				else {
					DBG("Battery Critical Overheat");
					batt_info->health = POWER_SUPPLY_HEALTH_CRITICAL_OVERHEAT;
				}
				break;

			case POWER_SUPPLY_HEALTH_COLD:
				if (batt_info->temperature >= (-50)) {
					if ( charger_ic_get_status() != CHARGER_DISABLE ) {
						// Reactivate Charger : Battery Normal again
						DBG("################Battery Normal again########################");
						batt_info->health = POWER_SUPPLY_HEALTH_GOOD;
						if ( charger_ic_get_state() != CHARGER_STATE_FULLBATTERY ) 
						{
							charger_ic_set_state(CHARGER_STATE_CHARGE);
							charger_ic_set_mode(batt_info->charge_setting_chcomp);
						}
						
					}
					else
						batt_info->health = POWER_SUPPLY_HEALTH_GOOD;
				}
				else {
					DBG("Battery Cold");
					batt_info->health = POWER_SUPPLY_HEALTH_COLD;
				}
				break;

			default:
				DBG("Battery Unknown Health State");
				batt_info->health = POWER_SUPPLY_HEALTH_UNKNOWN;
				break;
		} // switch end
	}// if end
}// function end

void determine_charger_state_with_charger_ic(void) 
{
// MOBII_CHANGE_S [shhong@mobii.co.kr] 2012-04-27 : Kernel Panic Issue Solution.
	if(refer_batt_info == NULL) {
		DBG("Battery Info No Longer Exists, Then This Function will be ignored.");
		return;
	}
// MOBII_CHANGE_E [shhong@mobii.co.kr] 2012-04-27 : Kernel Panic Issue Solution.
	struct battery_info *batt_info = refer_batt_info;
	unsigned int value_status = 0;
	unsigned int value_pgb = 0;
	unsigned int next_state;
	unsigned int charger_gpio_state;

	value_status 	= read_gpio_status();
	value_pgb 	= read_gpio_pgb();

	//DBG("GPIO STATE[%d], PGB[%d]", value_status, value_pgb);

	if (( value_status == 1 )&& ( value_pgb == 1 )) {
		charger_gpio_state = CHGSB_PGB_OFF_OFF;
	}
	else if (( value_status == 0 )&& ( value_pgb == 0 )) {
		charger_gpio_state = CHGSB_PGB_ON_ON;
	}
	else if (( value_status == 1 )&& ( value_pgb == 0 )) {
		charger_gpio_state = CHGSB_PGB_OFF_ON;
	}
	else if (( value_status == 0 )&& ( value_pgb == 1 ))
	{
		charger_gpio_state = CHGSB_PGB_ON_OFF;
		DBG("[Critical] Unknown Charger state GPIO!!!(CHGSB_PGB_ON_OFF)");
	}
	//DBG("CHARGER GPIO STATE[%d]", charger_gpio_state);

	//DBG("CHARGER STATE [%s]", charger_ic_state_machine_name[charger_ic_get_state()]);
	switch ( charger_ic_get_state() ) // chg_state_past
	{
		case CHARGER_STATE_SHUTDOWN: // CHGSB_PGB_OFF_OFF
			switch(charger_gpio_state)
			{
				case CHGSB_PGB_ON_ON:
					next_state = CHARGER_STATE_CHARGE;
					break;
				case CHGSB_PGB_OFF_ON:
					next_state = CHARGER_STATE_STANDBY;
					break;
				case CHGSB_PGB_OFF_OFF:
					next_state = charger_ic_get_state();
					break;
				default:
					DBG("[Critical] Unknown Charger state GPIO!!!(CHGSB_PGB_ON_OFF)");
					break;
			}
			break;

		case CHARGER_STATE_STANDBY: 	// CHGSB_PGB_OFF_ON
			switch(charger_gpio_state)
			{
				case CHGSB_PGB_ON_ON:
					next_state = CHARGER_STATE_CHARGE;
					break;
				case CHGSB_PGB_OFF_OFF:
					next_state = CHARGER_STATE_SHUTDOWN;
					break;
				case CHGSB_PGB_OFF_ON:
					next_state = charger_ic_get_state();
					break;
				default:
					DBG("[Critical] Unknown Charger state GPIO!!!(CHGSB_PGB_ON_OFF)");
					break;
			}
			break;

		case CHARGER_STATE_CHARGE: 	// CHGSB_PGB_ON_ON
		case CHARGER_STATE_RECHARGE: 	// CHGSB_PGB_ON_ON
			switch(charger_gpio_state)
			{
				case CHGSB_PGB_OFF_OFF:
					next_state = CHARGER_STATE_SHUTDOWN;
					break;
				case CHGSB_PGB_OFF_ON:
					next_state = CHARGER_STATE_FULLBATTERY;
					break;
				case CHGSB_PGB_ON_ON:
					next_state = charger_ic_get_state();
					break;
				default:
					DBG("[Critical] Unknown Charger state GPIO!!!(CHGSB_PGB_ON_OFF)");
					break;
			}
			break;

		case CHARGER_STATE_FULLBATTERY: // CHGSB_PGB_OFF_ON
			switch(charger_gpio_state)
			{
				case CHGSB_PGB_ON_ON:
					next_state = CHARGER_STATE_RECHARGE;
					break;
				case CHGSB_PGB_OFF_OFF:
					next_state = CHARGER_STATE_SHUTDOWN;
					break;
				case CHGSB_PGB_OFF_ON:
					next_state = charger_ic_get_state();
					break;
				default:
					DBG("[Critical] Unknown Charger state GPIO!!!(CHGSB_PGB_ON_OFF)");
					break;
			}
			break;

		default:
			DBG("[Critical] Unknown Charger State!!!!!");
			next_state = CHARGER_STATE_SHUTDOWN;
			break;
	}

	//DBG("NEXT CHARGER STATE [%s]", charger_ic_state_machine_name[next_state]);

	/* Recharging Control */
	if ( at_charge_index == 0 )
	{
		switch ( next_state )
		{
			case CHARGER_STATE_FULLBATTERY:
				if (charger_ic_get_state() != CHARGER_STATE_FULLBATTERY)
				{
					// Disable Charger for protecting Battery
					if ((batt_info->capacity == 100) 
							&& (batt_info->CBC_value >= 99) 
							&& (batt_info->voltage > 4165))
					{
						if ( charger_ic_get_status() != CHARGER_DISABLE )
						{
					#if defined (TRICKLE_RECHECK)
							batt_info->trickle_charge_check = 1;
					#endif // TRICKLE_RECHECK
							batt_info->charge_setting_chcomp = charger_ic_get_status();
							//charger_ic_disable();
							charger_ic_disable_for_recharge();
						}
					}
					else
						next_state = CHARGER_STATE_CHARGE;
				}
				break;

			default:
				break;
		}
	}

	charger_ic_set_state(next_state);
	//DBG("FINAL CHARGER STATE [%s]", charger_ic_state_machine_name[next_state]);
}
EXPORT_SYMBOL(determine_charger_state_with_charger_ic);

#define CRITICAL_BAT_CHECK_PERIOD 	90
#define BATTERY_POLLING_INTERVAL	390
#define SLEEP_BATTERY_CHECK_PERIOD	2090
#define BAT_MIN(x, y) ((x) < (y) ? (x) : (y))
#define BAT_MAX(x, y) ((x) > (y) ? (x) : (y))

static void battery_data_polling_period_change(void)
{
	struct battery_info *batt_info = refer_batt_info;
	unsigned int temp_period_data, temp_period_wake;
	unsigned int vol_period_data, vol_period_wake;
	unsigned int gauge_period_data, gauge_period_wake;
	unsigned int critical_period_data, critical_period_wake;

	// Critical Status
	if ((batt_info->capacity <= 5) || (charger_ic_get_status() != CHARGER_DISABLE)
			|| (batt_info->health != POWER_SUPPLY_HEALTH_GOOD) || (batt_info->voltage <= 3500)
			|| (charger_ic_get_state() == CHARGER_STATE_FULLBATTERY)) {
		critical_period_data = CRITICAL_BAT_CHECK_PERIOD * HZ;
		critical_period_wake = CRITICAL_BAT_CHECK_PERIOD + 10;
	} else {
		critical_period_data = BATTERY_POLLING_INTERVAL * HZ;
		critical_period_wake = SLEEP_BATTERY_CHECK_PERIOD;
	}

	// Voltage Status
	vol_period_data = BATTERY_POLLING_INTERVAL * HZ;
	vol_period_wake = SLEEP_BATTERY_CHECK_PERIOD;

	// Temperatue Status
	if ((BAT_MAX(batt_info->prev_temperature, batt_info->temperature)
				- BAT_MIN(batt_info->prev_temperature, batt_info->temperature)) < 50) // 5 degree
	{
		temp_period_data = BATTERY_POLLING_INTERVAL * HZ;
		temp_period_wake = SLEEP_BATTERY_CHECK_PERIOD;
	} else {
		temp_period_data = CRITICAL_BAT_CHECK_PERIOD*2 * HZ;
		temp_period_wake = CRITICAL_BAT_CHECK_PERIOD*2 + 10;
	}

	// Gauge Status
	gauge_period_data = BATTERY_POLLING_INTERVAL * HZ;
	gauge_period_wake = SLEEP_BATTERY_CHECK_PERIOD;

	batt_info->polling_interval = BAT_MIN(BAT_MIN(critical_period_data, vol_period_data), BAT_MIN(temp_period_data, gauge_period_data));
	batt_info->sleep_polling_interval = BAT_MIN(BAT_MIN(critical_period_wake, vol_period_wake), BAT_MIN(temp_period_wake, gauge_period_wake));
#if 0
	DBG("Poll/Wake[%d, %d]", batt_info->polling_interval/HZ, batt_info->sleep_polling_interval);
	DBG("Cri[%d, %d]/Vol[%d, %d]", critical_period_data/HZ, critical_period_wake, vol_period_data, vol_period_wake);
	DBG("Tem[%d, %d]/Gau[%d, %d]", temp_period_data/HZ, temp_period_wake, gauge_period_data/HZ, gauge_period_wake);
#endif	
}

#if defined(TRICKLE_RECHECK)
static void star_set_recharge_again(void)
{
	struct battery_info *batt_info = refer_batt_info;
	DBG("Re-enable Charger : AGAIN!!");
	charger_ic_set_state(CHARGER_STATE_RECHARGE);
	switch (batt_info->health)
	{
		case POWER_SUPPLY_HEALTH_OVERHEAT:
			DBG("[Warning]: Reconfigure Charger, BUT batt_health is POWER_SUPPLY_HEALTH_OVERHEAT : 400mA current setting");
			charger_ic_set_mode(CHARGER_USB500);
			break;

		case POWER_SUPPLY_HEALTH_CRITICAL_OVERHEAT:
			DBG("[Warning]: Reconfigure Charger, BUT batt_health is POWER_SUPPLY_HEALTH_CRITICAL_OVERHEAT : Do nothing");
			break;

		case POWER_SUPPLY_HEALTH_COLD:
			DBG("[Warning]: Reconfigure Charger, BUT batt_health is POWER_SUPPLY_HEALTH_COLD : Do nothing");
			break;

		case POWER_SUPPLY_HEALTH_GOOD:
		default:
			charger_ic_set_mode(batt_info->charge_setting_chcomp);
			break;
	}
}
#endif // TRICKLE_RECHECK

static void battery_update_changes(struct battery_info *batt_info, int force_update)
{
	battery_update(batt_info);

	/* Determine Charger State with GPIO Status */
	determine_charger_state_with_charger_ic();

	/* Control Charger for Battery Health */
#if 0
	charger_contol_with_battery_temp();
#else
if (batt_info->temp_control == TEMP_CONTROL_OFF)
	charger_contol_with_battery_temp();
else //TEMP_CONTROL_ON
	charger_contorl_unlimited_temp();
#endif
	/* Reference Calculation for Capacity */
	star_capacity_from_voltage_via_calculate();

	/* Recharging Algos */
	if ( (( batt_info->CBC_value <= 97 ) || (batt_info->voltage < 4135)) 
			&& (charger_ic_get_state() == CHARGER_STATE_FULLBATTERY)) 
	{
		DBG("Recharging Starts.");
		charger_ic_set_state(CHARGER_STATE_RECHARGE);
		switch ( batt_info->health ) {
			case POWER_SUPPLY_HEALTH_CRITICAL_OVERHEAT:
			case POWER_SUPPLY_HEALTH_COLD:
				break;

			case POWER_SUPPLY_HEALTH_OVERHEAT:
				charger_ic_set_mode(CHARGER_USB500);
				break;
			case POWER_SUPPLY_HEALTH_GOOD:
			default:
				charger_ic_set_mode(batt_info->charge_setting_chcomp);
				break;
		}
	}

	//	if( force_update == 0 )
	{
		/* Trickle Recheck */
#if defined(TRICKLE_RECHECK)
		if ((batt_info->trickle_charge_check == 1) && (batt_info->voltage < 4000))
			star_set_recharge_again();
#endif // TRICKLE_RECHECK

		/* Gauge Follower Function Called */
		if( (batt_info->gauge_on == 1) && (batt_info->capacity != batt_info->capacity_gauge) )
			star_gauge_follower_func();

		/* Changes Polling Period */
		battery_data_polling_period_change();
	}



//MOBII_CHNANGE_S 20120716 jd.park@mobii.co.kr : FOTA UA Upgrade for ICS
#if defined(CONFIG_MACH_STAR_P990) || defined(CONFIG_MACH_STAR_SU660) || defined(CONFIG_MACH_STAR_P999)
	if (ignore_send_uevent == FALSE) {
		power_supply_changed(&batt_info->bat);
	}
#else
	power_supply_changed(&batt_info->bat);
#endif
//MOBII_CHNANGE_E 20120716 jd.park@mobii.co.kr : FOTA UA Upgrade for ICS
	//	power_supply_changed(&batt_info->usb);
	//	power_supply_changed(&batt_info->ac);
	DBG("[BATT] voltage [%d], Temperature [%d], Capacity[%d], Present [%d], BattHealth[%s]\n"
		"[CHARGER] Type [%s]", 
		batt_info->voltage, batt_info->temperature, batt_info->capacity, batt_info->present,BatteryHealth_name[batt_info->health], 
		ChargerType_name[batt_info->charge_type]);	
	DBG("Update Power Supply.");
}

/* MODE For MUIC */
void charger_ic_set_mode_for_muic(unsigned int mode)
{
	if(refer_batt_info == NULL)
		return;

	DBG();

	if ( (charger_ic_get_status() != CHARGER_FACTORY)||(mode != CHARGER_FACTORY) ) 
	{
		if( refer_batt_info->present == 1 ) {
			charger_ic_set_mode(mode);
			switch( charger_ic_get_status() ) {
				case CHARGER_USB500:
				case CHARGER_USB100:
				case CHARGER_ISET:
				case CHARGER_FACTORY:
//MOBII_CHNANGE_S 20120716 jd.park@mobii.co.kr : FOTA UA Upgrade for ICS
#if defined(CONFIG_MACH_STAR_P990) || defined(CONFIG_MACH_STAR_SU660) || defined(CONFIG_MACH_STAR_P999)
					if (ignore_send_uevent == FALSE) {
						power_supply_changed(&refer_batt_info->bat);
					}
#else
					power_supply_changed(&refer_batt_info->bat);
#endif
//MOBII_CHNANGE_E 20120716 jd.park@mobii.co.kr : FOTA UA Upgrade for ICS
					//power_supply_changed(&refer_batt_info->usb);
					//power_supply_changed(&refer_batt_info->ac);
					break;

				default:
					DBG("[Critical] Unknown Charger Exception!!!");
					break;
			} // End switch( charger... )
		} else {
			DBG("[No_Battery] Status[%s]", charger_ic_status_name[mode]);
		}
	}

	if (refer_batt_info->health != POWER_SUPPLY_HEALTH_GOOD)
	{
		refer_batt_info->health = POWER_SUPPLY_HEALTH_GOOD;
#if 0
		charger_contol_with_battery_temp();
#else
		if (refer_batt_info->temp_control == TEMP_CONTROL_OFF)
		{
			charger_contol_with_battery_temp();
		}
		else //TEMP_CONTROL_ON
		{
			charger_contorl_unlimited_temp();
		}
#endif		

	}
}
EXPORT_SYMBOL(charger_ic_set_mode_for_muic);

void charger_ic_disable_for_muic(void)
{
	if(refer_batt_info == NULL)
		return;

	DBG();
	charger_ic_disable();
#if defined(TRICKLE_RECHECK)
	refer_batt_info->trickle_charge_check = 0;
#endif
//MOBII_CHNANGE_S 20120716 jd.park@mobii.co.kr : FOTA UA Upgrade for ICS
#if defined(CONFIG_MACH_STAR_P990) || defined(CONFIG_MACH_STAR_SU660) || defined(CONFIG_MACH_STAR_P999)
	if (ignore_send_uevent == FALSE) {
		power_supply_changed(&refer_batt_info->bat);
	}
#else
	power_supply_changed(&refer_batt_info->bat);
#endif
//MOBII_CHNANGE_E 20120716 jd.park@mobii.co.kr : FOTA UA Upgrade for ICS
}
EXPORT_SYMBOL(charger_ic_disable_for_muic);

void notification_of_changes_to_battery(void)
{

	//DBG("[BATTERY SHUTDOWN] notification_of_changes_to_battery(%d)\n",update_on);
	if(refer_batt_info == NULL)
		return;

	// Update Battery Information
	cancel_delayed_work_sync(&refer_batt_info->star_monitor_work);
	battery_update_changes(refer_batt_info, 1);
	queue_delayed_work(refer_batt_info->battery_workqueue, &refer_batt_info->star_monitor_work, 10 * HZ);
}


EXPORT_SYMBOL(notification_of_changes_to_battery);

/* Battery Status Polling */
static void battery_work(struct work_struct *work)
{
	DBG();
	struct battery_info *batt_info = refer_batt_info;
	battery_update_changes(batt_info, 0);
	queue_delayed_work(batt_info->battery_workqueue, &batt_info->star_monitor_work, batt_info->polling_interval);
}

/* Battery ID Polling */
static void battery_id_check() 
{
	struct battery_info *batt_info = refer_batt_info;
	int batt_voltage;
	int batt_temperature;
	batt_voltage 		= battery_read_voltage();
	batt_temperature	= battery_read_temperature();


	if ( at_charge_index == 0 )
	{
		if (( -400 > batt_temperature ) || ( 900 < batt_temperature )) // Temperature range -40 ~ 90 [C]: Battery present
		{
			DBG("No Battery State OR Dummy Battery State");
			if ( (charger_ic_get_status() != CHARGER_DISABLE) && ( at_boot_state == 0 ) )
			{
				DBG("No Battery or Dummy Battery State");
        		batt_info->present = 0;
//MOBII_CHNANGE_S 20120716 jd.park@mobii.co.kr : FOTA UA Upgrade for ICS
#if defined(CONFIG_MACH_STAR_P990) || defined(CONFIG_MACH_STAR_SU660) || defined(CONFIG_MACH_STAR_P999)
				if (ignore_send_uevent == FALSE) {
					power_supply_changed(&batt_info->bat);
				}
#else
				power_supply_changed(&batt_info->bat);
#endif
//MOBII_CHNANGE_E 20120716 jd.park@mobii.co.kr : FOTA UA Upgrade for ICS
			}
		}
	}
	else // ( at_charge_index == 1 )
	{
		if ((charger_ic_get_status() == CHARGER_DISABLE) && (at_charge_on == 0))
		{
			at_charge_on = 1;
			charger_ic_set_mode_for_muic(CHARGER_ISET);
		}
	}

}
static void battery_id_work(struct work_struct *work)
{
	struct battery_info *batt_info = container_of(work,
			struct battery_info, star_id_monitor_work.work);

	if ((( at_charge_index == 0 ) && ( charger_ic_get_status() != CHARGER_DISABLE ))
			|| (( at_charge_index == 1 ) && ( charger_ic_get_status() == CHARGER_DISABLE )))
	{
		battery_id_check();
	}

	queue_delayed_work(batt_info->battery_workqueue, &batt_info->star_id_monitor_work, batt_info->id_polling_interval);
}

#define to_battery_info(x) container_of((x), \
		struct battery_info, bat);

static int battery_get_property(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	struct battery_info *batt_info;
	batt_info = to_battery_info(psy);
	TYPE_CHARGING_MODE muic_charging_mode = CHARGING_NONE;

	muic_charging_mode = get_muic_charger_type();

	switch (psp) {
		case POWER_SUPPLY_PROP_STATUS:
			if (batt_info->gauge_on == 0) // Not yet receive CBC from CP
			{
				val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
				//			DBG("[Warning] Cannot receive CBC from CP until now, Display Battery loading Icon!!");
			}
//			else if (batt_info->present == 0)
//			{
//				val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
				//			DBG("BATTERY_STATUS_NO_BATTERY(%d)", val->intval);
//			}
			else if ((batt_info->health == POWER_SUPPLY_HEALTH_CRITICAL_OVERHEAT) 
					|| (batt_info->health == POWER_SUPPLY_HEALTH_COLD))
			{
					  if(charger_ic_get_status() != CHARGER_DISABLE)
					  {
					  	if((batt_info->capacity != 100))
					  	{
					  		if(batt_info->temp_control == TEMP_CONTROL_OFF)
								val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
							else
								val->intval = POWER_SUPPLY_STATUS_CHARGING;
					  	}
						else
						{
							val->intval = POWER_SUPPLY_STATUS_FULL;
						}
					  }
					  else
					  	val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
				}
			else if((batt_info->health == POWER_SUPPLY_HEALTH_OVERHEAT))
			{
					 if(charger_ic_get_status() != CHARGER_DISABLE)
					 {
						if(batt_info->temp_control == TEMP_CONTROL_OFF)
					 	{
						   if(batt_info->high_temp_overvoltage==1)
						   	{
						   		val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
						   	}
						    else
					    	{
					    		val->intval = POWER_SUPPLY_STATUS_CHARGING;	
					    	}
					 	}
						else
						{
							if((batt_info->capacity != 100))
						  	{
								val->intval = POWER_SUPPLY_STATUS_CHARGING;
							}
							else
							{
								val->intval = POWER_SUPPLY_STATUS_FULL;
							}
						}
					 }
	         		 else
					 	val->intval = POWER_SUPPLY_STATUS_DISCHARGING;	
				}

			else if ((batt_info->capacity == 100) 
					&& (charger_ic_get_status() != CHARGER_DISABLE))
			{
				val->intval = POWER_SUPPLY_STATUS_FULL;
				//			DBG("STATUS_FULL(%d)", val->intval);
			}
			else if (charger_ic_get_status() != CHARGER_DISABLE) // POWER_SUPPLY_STATUS_CHARGING
			{
			    if(muic_charging_mode == CHARGING_NONE)
			    {
			        val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
					charger_ic_disable();
					//DBG("POWER_SUPPLY_STATUS_DISCHARGING\n");
					//DBG("charger_ic_get_status() ==[%d]\n",charger_ic_get_status());
			    }
				else
				{
				val->intval = POWER_SUPPLY_STATUS_CHARGING;
					//DBG("POWER_SUPPLY_STATUS_CHARGING\n");
				}
				//			DBG("STATUS_CHARGING(%d)", val->intval);
			}
			else if (charger_ic_get_status() == CHARGER_DISABLE) // NVODM_BATTERY_STATUS_DISCHARGING
			{
				val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
				//			DBG("STATUS_DISCHARGING(%d)", val->intval);
			}
			else // POWER_SUPPLY_STATUS_UNKNOWN
			{
				val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
				//			DBG("[Critical] Why this is occured?? Battery Status Unknown");
			}

			break;

		case POWER_SUPPLY_PROP_PRESENT:
			val->intval = batt_info->present;
			break;

		case POWER_SUPPLY_PROP_TECHNOLOGY:
			val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
			break;

		case POWER_SUPPLY_PROP_HEALTH:
			if (batt_info->health == POWER_SUPPLY_HEALTH_CRITICAL_OVERHEAT)
				val->intval = POWER_SUPPLY_HEALTH_OVERHEAT;
			else
				val->intval = batt_info->health;

			#if 0
			switch(batt_info->health)
			{
				case POWER_SUPPLY_HEALTH_UNKNOWN:
					DBG("POWER_SUPPLY_HEALTH_UNKNOWN");
					break;
				case POWER_SUPPLY_HEALTH_GOOD:
					DBG("POWER_SUPPLY_HEALTH_GOOD");
					break;
				case POWER_SUPPLY_HEALTH_OVERHEAT:
					DBG("POWER_SUPPLY_HEALTH_OVERHEAT");
					break;
				case POWER_SUPPLY_HEALTH_DEAD:
					DBG("POWER_SUPPLY_HEALTH_DEAD");
					break;
				case POWER_SUPPLY_HEALTH_OVERVOLTAGE:
					DBG("POWER_SUPPLY_HEALTH_OVERVOLTAGE");
					break;
				case POWER_SUPPLY_HEALTH_UNSPEC_FAILURE:
					DBG("POWER_SUPPLY_HEALTH_UNSPEC_FAILURE");
					break;
				case POWER_SUPPLY_HEALTH_COLD:
					DBG("POWER_SUPPLY_HEALTH_COLD");
					break;	
				case POWER_SUPPLY_HEALTH_CRITICAL_OVERHEAT:
					DBG("POWER_SUPPLY_HEALTH_CRITICAL_OVERHEAT");
					break;						
			}
			#endif
			break;

		case POWER_SUPPLY_PROP_VOLTAGE_NOW:
			val->intval = batt_info->voltage;
			break;

		case POWER_SUPPLY_PROP_CAPACITY:
			if (batt_info->gauge_on == 1)
				val->intval = batt_info->capacity;
			else
				val->intval = 999;
			break;

		case POWER_SUPPLY_PROP_TEMP:
			if(batt_info->present ==0)
				val->intval=200;
			else
				val->intval = batt_info->temperature;
			//DBG("PROP_TEMP(%d)", val->intval);
			break;
		case POWER_SUPPLY_PROP_ONLINE:
			{				
				val->intval = batt_info->charge_type;
			}
		break;


		default:
			DBG("[Critical] Why this is occured?? Unknown battery property");
			return -EINVAL;
	}

	return 0;
}

static int ac_battery_get_property(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	struct battery_info *batt_info;
	batt_info = to_battery_info(psy);

	switch (psp) {
		case POWER_SUPPLY_PROP_ONLINE:
			{				
				TYPE_CHARGING_MODE muic_mode = CHARGING_NONE;
				muic_mode = get_muic_charger_type();
				if ( muic_mode == CHARGING_TA_1A
						|| muic_mode == CHARGING_NA_TA
						|| muic_mode == CHARGING_LG_TA)
					val->intval = 1;
				else
					val->intval = 0;
			}
			break;
		default:
			val->intval = 0;
			return -EINVAL;
	}
	//	DBG("INTVAL : [%d]", val->intval);
	return 0;
}

static int usb_battery_get_property(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	struct battery_info *batt_info;
	batt_info = to_battery_info(psy);

	switch (psp) {
		case POWER_SUPPLY_PROP_ONLINE:
			{
				TYPE_CHARGING_MODE muic_mode = CHARGING_NONE;
				muic_mode = get_muic_charger_type();
				if ( muic_mode == CHARGING_USB)
					val->intval = 1;
				else
					val->intval = 0;		
			}
			break;
		default:
			val->intval = 0;
			return -EINVAL;
	}
	//	DBG("INTVAL : [%d]", val->intval);
	return 0;
}
//#endif

/* Sysfs Cmd. Func Declaration - AT CHARGE */
static ssize_t star_at_charge_show_property(
		struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	size_t count = 0;
	if( at_charge_index == 1 )	count = sprintf(buf, "1\n");
	else if( at_charge_index == 0 )	count = sprintf(buf, "0\n");
	else				count = sprintf(buf, "0\n");
	return count;
}
static ssize_t star_at_charge_store_property(
		struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t count)
{
	struct battery_info *batt_info = refer_batt_info;
	static unsigned int value = 0;

	value = (unsigned int)(simple_strtoul(buf, NULL, 0));
	if ( value == 1 )
	{
		at_charge_index = 1;
		battery_update_changes(batt_info, 1);	// 0 or 1 Doesn't care
		charger_ic_set_mode_for_muic(CHARGER_ISET);
	}
	return count;
}

/* Sysfs Cmd. Func Declaration - AT CHOMP */
static ssize_t star_at_chcomp_show_property(
		struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct battery_info *batt_info = refer_batt_info;
	size_t count = 0;

	battery_update_changes(batt_info, 1);	// 0 or 1 Doesn't care
	if (( batt_info->voltage >= 4100 ) || ( at_charge_comp == 1 ))
	{
		batt_info->capacity = 95;
//MOBII_CHNANGE_S 20120716 jd.park@mobii.co.kr : FOTA UA Upgrade for ICS
#if defined(CONFIG_MACH_STAR_P990) || defined(CONFIG_MACH_STAR_SU660) || defined(CONFIG_MACH_STAR_P999)
		if (ignore_send_uevent == FALSE) {
			power_supply_changed(&batt_info->bat);
		}
#else
		power_supply_changed(&batt_info->bat);
#endif
//MOBII_CHNANGE_E 20120716 jd.park@mobii.co.kr : FOTA UA Upgrade for ICS
		batt_info->capacity = 100;
//MOBII_CHNANGE_S 20120716 jd.park@mobii.co.kr : FOTA UA Upgrade for ICS
#if defined(CONFIG_MACH_STAR_P990) || defined(CONFIG_MACH_STAR_SU660) || defined(CONFIG_MACH_STAR_P999)
		if (ignore_send_uevent == FALSE) {
			power_supply_changed(&batt_info->bat);
		}
#else
		power_supply_changed(&batt_info->bat);
#endif
//MOBII_CHNANGE_E 20120716 jd.park@mobii.co.kr : FOTA UA Upgrade for ICS
		count = sprintf(buf, "1\n");
	}
	else
	{
		batt_info->capacity = 95;
//MOBII_CHNANGE_S 20120716 jd.park@mobii.co.kr : FOTA UA Upgrade for ICS
#if defined(CONFIG_MACH_STAR_P990) || defined(CONFIG_MACH_STAR_SU660) || defined(CONFIG_MACH_STAR_P999)
		if (ignore_send_uevent == FALSE) {
			power_supply_changed(&batt_info->bat);
		}
#else
		power_supply_changed(&batt_info->bat);
#endif
//MOBII_CHNANGE_E 20120716 jd.park@mobii.co.kr : FOTA UA Upgrade for ICS
		count = sprintf(buf, "0\n");
	}
	return count;
}
static ssize_t star_at_chcomp_store_property(
		struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t count)
{
	struct battery_info *batt_info = refer_batt_info;
	static unsigned int value = 0;
	value = (unsigned int)(simple_strtoul(buf, NULL, 0));

	if ( value == 0 )
	{
		batt_info->capacity = 95;
		at_charge_comp = 0;
//MOBII_CHNANGE_S 20120716 jd.park@mobii.co.kr : FOTA UA Upgrade for ICS
#if defined(CONFIG_MACH_STAR_P990) || defined(CONFIG_MACH_STAR_SU660) || defined(CONFIG_MACH_STAR_P999)
		if (ignore_send_uevent == FALSE) {
			power_supply_changed(&batt_info->bat);
		}
#else
		power_supply_changed(&batt_info->bat);
#endif
//MOBII_CHNANGE_E 20120716 jd.park@mobii.co.kr : FOTA UA Upgrade for ICS
	}
	else if ( value == 1 )
	{
		batt_info->capacity = 100;
		at_charge_comp = 1;
//MOBII_CHNANGE_S 20120716 jd.park@mobii.co.kr : FOTA UA Upgrade for ICS
#if defined(CONFIG_MACH_STAR_P990) || defined(CONFIG_MACH_STAR_SU660) || defined(CONFIG_MACH_STAR_P999)
		if (ignore_send_uevent == FALSE) {
			power_supply_changed(&batt_info->bat);
		}
#else
		power_supply_changed(&batt_info->bat);
#endif
//MOBII_CHNANGE_S 20120716 jd.park@mobii.co.kr : FOTA UA Upgrade for ICS
	}
	return count;
}

static ssize_t star_debug_show_property(
		struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct battery_info *batt_info = refer_batt_info;
	size_t count = 0;

	count = sprintf(buf, "%d\n", batt_info->polling_interval);
	return count;
}

static ssize_t star_debug_store_property(
		struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t count)
{
	struct battery_info *batt_info = refer_batt_info;
	static unsigned int value = 0;

	value = (unsigned int)(simple_strtoul(buf, NULL, 0));

	if ((value == 2171) && (ELT_test_mode == 0))		ELT_test_mode = 1;
	else if ((value == 2170) && (ELT_test_mode == 1))	ELT_test_mode = 0;
	else
	{
		batt_info->polling_interval = HZ*value;
		cancel_delayed_work_sync(&batt_info->star_monitor_work);
		queue_delayed_work(batt_info->battery_workqueue, &batt_info->star_monitor_work, HZ/60);
	}
	return count;
}

static unsigned char star_charger_disable_ok_check(void)
{
	struct battery_info *batt_info = refer_batt_info;

	if (batt_info->charge_type == POWER_SUPPLY_TYPE_MAINS 
			|| batt_info->charge_type == POWER_SUPPLY_TYPE_USB)
	{
		if (batt_info->voltage > 3400)	return 1;
		else				return 0;
	}
	return 0;
}
static ssize_t star_cbc_show_property (
		struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct battery_info *batt_info = refer_batt_info;

	if( batt_info->gauge_on == 0 )
	{
		if ((at_charge_index == 0) 
				&& (charger_ic_get_state() != CHARGER_STATE_FULLBATTERY) 
				&& (charger_ic_get_status() != CHARGER_DISABLE))
		{
			if  (((batt_info->health == POWER_SUPPLY_HEALTH_OVERHEAT) 
						||(batt_info->health == POWER_SUPPLY_HEALTH_GOOD))
					&& (star_charger_disable_ok_check() == 1))
			{
				batt_info->boot_TA_setting = 1;
				charger_ic_set_state(CHARGER_STATE_SHUTDOWN);
				batt_info->charge_setting_chcomp = charger_ic_get_status();
				charger_ic_disable_for_recharge();
				DBG("Should be shut down.");
			}
		}
	}
	return sprintf(buf, "%d\n", batt_info->capacity);
}
static ssize_t star_cbc_store_property(
		struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t count)
{
	struct battery_info *batt_info = refer_batt_info;
	static unsigned int value = 0;
	value = (unsigned int)(simple_strtoul(buf, NULL, 0));

	if (value != 999)
	{
		if (value <= 3)
			batt_info->capacity = 1;
		else if (value >= 95)
			batt_info->capacity = 100;
		else if ((value >= 92) && (value < 95))
			batt_info->capacity = 99;
		else
			batt_info->capacity = ( (value-2) * 100 / 91 );

		batt_info->gauge_on = 1;
		batt_info->CBC_value = value;
	}

	if (batt_info->boot_TA_setting == 1)
	{
		if  (batt_info->health== POWER_SUPPLY_HEALTH_GOOD)
		{
			charger_ic_set_state(CHARGER_STATE_CHARGE);
			charger_ic_set_mode(batt_info->charge_setting_chcomp);
			batt_info->boot_TA_setting = 0;
		}
		else if (batt_info->health == POWER_SUPPLY_HEALTH_OVERHEAT)
		{
			charger_ic_set_state(CHARGER_STATE_CHARGE);
			charger_ic_set_mode(CHARGER_USB500);	
			batt_info->boot_TA_setting = 0;
		}
	}

	battery_update_changes(batt_info,0); //for half charging
	return count;
}

/* Sysfs Cmd. Func Declaration - BAT Gauge */
static void star_capacity_from_voltage_via_calculate(void) 
{
	struct battery_info *batt_info = refer_batt_info;
	unsigned int calculate_capacity = 0;

	DBG("CHARGER [%s] / MACHINE [%s]", 
			charger_ic_status_name[charger_ic_get_status()], 
			charger_ic_state_machine_name[charger_ic_get_state()]);
	if ( (charger_ic_get_status() != CHARGER_DISABLE) 
			&& (charger_ic_get_state() == CHARGER_STATE_RECHARGE 
				|| charger_ic_get_state() == CHARGER_STATE_CHARGE)) 
	{
		if ( batt_info->voltage <= 3828 )
		{
			if (  batt_info->voltage < 3656 ) 
				calculate_capacity = 1;
			else	
				calculate_capacity = (unsigned int)((batt_info->voltage*1000000 - 3655943209)/6239571);
		} 
		else if (( batt_info->voltage > 3828 ) && ( batt_info->voltage <= 3878 ))
			calculate_capacity = (unsigned int)((batt_info->voltage*1000000 - 3814262889)/5230892);
		else if (( batt_info->voltage > 3878 ) && ( batt_info->voltage <= 3943 ))
			calculate_capacity = (unsigned int)((batt_info->voltage*1000000 - 3804342997)/6035547);
		else if (( batt_info->voltage > 3943 ) && ( batt_info->voltage <= 4005 ))
			calculate_capacity = (unsigned int)((batt_info->voltage*1000000 - 3891922385)/2243847);
		else if (( batt_info->voltage > 4005 ) && ( batt_info->voltage <= 4133 ))
			calculate_capacity = (unsigned int)((batt_info->voltage*1000000 - 3736359651)/5330744);
		else if ( batt_info->voltage > 4133 )
		{
			if ( batt_info->voltage >= 4200 )
				calculate_capacity = 100;
			else
				calculate_capacity = (unsigned int)((batt_info->voltage*1000000 - 3953389838)/2419419);
		}

		if ( calculate_capacity < 0 )	calculate_capacity = 1;
		if ( calculate_capacity > 100 )	calculate_capacity = 99;

		DBG("with Charger Batt CAP [%d] / Calc CAP [%d]", batt_info->capacity, calculate_capacity);
		batt_info->capacity_voltage = calculate_capacity;		
	}
	else if ( (charger_ic_get_status() == CHARGER_DISABLE) 
			|| (charger_ic_get_state() == CHARGER_STATE_FULLBATTERY) )
	{
		if ( batt_info->voltage >= 3970 )
		{
			if ( batt_info->voltage >= 4200 )
				calculate_capacity = 100;
			else
				calculate_capacity = (unsigned int)((batt_info->voltage*1000000 - 3223959227)/8766094);
		}
		else if (( batt_info->voltage >= 3798 ) && ( batt_info->voltage< 3970 ))
			calculate_capacity = (unsigned int)((batt_info->voltage*1000000 - 3382701485)/6900848);
		else if (( batt_info->voltage >= 3713 ) && ( batt_info->voltage < 3798 ))
			calculate_capacity = (unsigned int)((batt_info->voltage*1000000 - 3582377778)/3588889);
		else if (( batt_info->voltage >= 3675 ) && ( batt_info->voltage < 3713 ))
			calculate_capacity = (unsigned int)((batt_info->voltage*1000000 - 3629265282)/2307994);
		else if (( batt_info->voltage >= 3575 ) && ( batt_info->voltage < 3675 ))
			calculate_capacity = (unsigned int)((batt_info->voltage*1000000 - 3548445378)/6386555);
		else if (( batt_info->voltage >= 3500 ) && ( batt_info->voltage< 3675 ))
			calculate_capacity = (unsigned int)((batt_info->voltage*1000000 - 3410416667)/39583333);
		else if ( batt_info->voltage < 3500 )
		{
			if (  batt_info->voltage <= 3400 )
				calculate_capacity = 1;
			else
				calculate_capacity = (unsigned int)((batt_info->voltage*1000000 - 3300357143)/88214286);
		}

		if ( calculate_capacity < 0 )	calculate_capacity = 1;
		if ( calculate_capacity > 100 )	calculate_capacity = 99;

		DBG("Without Charger Batt CAP [%d] / Calc CAP [%d]", batt_info->capacity, calculate_capacity);
		batt_info->capacity_voltage = calculate_capacity;
	}
}
static void star_gauge_follower_func(void)
{
	struct battery_info *batt_info = refer_batt_info;

	batt_info->prev_capacity = batt_info->capacity;
	if(charger_ic_get_status() != CHARGER_DISABLE)
	{
		if( batt_info->capacity_gauge > batt_info->capacity )		batt_info->capacity += 1;
		else if( batt_info->capacity_gauge < batt_info->capacity )	batt_info->capacity -= 1;
	} 
	else	
	{
		if( batt_info->capacity_gauge < batt_info->capacity )		batt_info->capacity -= 1;
	}

	if (batt_info->capacity < 0)   batt_info->capacity = 0;
	if (batt_info->capacity > 100) batt_info->capacity = 100;
}
static void valid_cbc_check_and_process(unsigned int value)  // not used  yet 
{
	DBG("CDC Value [%d]", value);
	struct battery_info *batt_info = refer_batt_info;
	static unsigned int display_cbc = 0;

	if( bat_shutdown == 1 )
		return;

	//	if( at_charge_index == 0 )
	{
#if defined(TRICKLE_RECHECK)
		if ((batt_info->trickle_charge_check == 1) && (value <=94))
		{
			if (batt_info->CBC_value >= value) // gauge_value reduce continuously... even if charger is in Recharge condition!!!
			{
				star_set_recharge_again();
			}
		}
#endif // TRICKLE_RECHECK

		batt_info->CBC_value = value;
		battery_update(batt_info);
		star_capacity_from_voltage_via_calculate();

		if ( value <= 3 )
			display_cbc = 1;
		else if ( value >= 95 )
			display_cbc = 100;
		else if (( value >= 92 ) && ( value < 95 ))
			display_cbc = 99;
		else
			display_cbc = ( (value-2) * 100 / 91 );

		if (batt_info->voltage <= 3400)
			display_cbc = 0;

		DBG("Display Gauge Value [%d]", display_cbc);

		if( batt_info->gauge_on == 0 )
		{
			if( value <= 2 )
			{
				if (((batt_info->voltage < 3500) || (batt_info->capacity <= 5)))
				{
					batt_info->capacity = 1;
					batt_info->capacity_gauge = 1;
					batt_info->gauge_on = 1;
				}
				else	
				{
					batt_info->capacity_gauge = display_cbc;
#if defined(CONFIG_MACH_STAR_P990)
					batt_info->capacity = display_cbc;
					batt_info->gauge_on = 1;
#endif
				}
			}
			else if( value == 100 )
			{
				if (((batt_info->voltage > 4100) || (batt_info->capacity >= 93)))
				{
					batt_info->capacity = 100;
					batt_info->capacity_gauge = 100;
					batt_info->gauge_on = 1;
				}
				else 	batt_info->capacity_gauge = display_cbc;
			}
			else
			{
				batt_info->capacity = display_cbc;
				batt_info->capacity_gauge = display_cbc;
				batt_info->gauge_on = 1;
			}
		}

		if ( batt_info->gauge_on == 1 )
		{
			batt_info->prev_capacity = batt_info->capacity;
			if( batt_info->capacity != 104 )			// at first process, do not update
				previous_gauge = batt_info->capacity;		// save previous value

			if( value <= 2 )
			{
				if((batt_info->voltage < 3450) || (batt_info->capacity <= 3)) 
				{
					batt_info->capacity = 1;
					batt_info->capacity_gauge = 1;
				}
				else if((batt_info->voltage < 3400) || (batt_info->capacity <= 3))
				{
					batt_info->capacity = 0;
					batt_info->capacity_gauge = 0;
				}
			}
			else if( value == 100 )
			{
				if((batt_info->voltage > 4130) || (batt_info->capacity >= 96))
				{	
					batt_info->capacity = 100;
					batt_info->capacity_gauge = 100;
				}
			}
			else // ( 0 < cbc_value < 100 )
			{
#if 1

				if( display_cbc >= batt_info->capacity ) 
				{
					if((display_cbc - batt_info->capacity) <= 1)
					{
						batt_info->capacity = display_cbc;
						batt_info->capacity_gauge = display_cbc;
					}
					else if((display_cbc - batt_info->capacity) > 1)
					{	
						batt_info->capacity_gauge = display_cbc;
						if(g_is_suspend)
						{
							batt_info->capacity=display_cbc;
							g_is_suspend=0;
						}
						else
						{
							star_gauge_follower_func();
						}
					}
				}
				else if( display_cbc < batt_info->capacity ) 
				{
					if ((batt_info->capacity - display_cbc) <= 1)
					{
						batt_info->capacity = display_cbc;
						batt_info->capacity_gauge = display_cbc;
					}
					else if ((batt_info->capacity - display_cbc) > 1)
					{
						batt_info->capacity_gauge = display_cbc;
						if(g_is_suspend)
						{
							batt_info->capacity=display_cbc;
							g_is_suspend=0;
						}
						else
						{
							star_gauge_follower_func();
						}
					}
				}
#else
                batt_info->capacity = display_cbc;
#endif						
				batt_info->capacity_gauge = display_cbc;

				if (batt_info->capacity < 0)	batt_info->capacity = 0;
				if (batt_info->capacity > 100)	batt_info->capacity = 100;
			}

#if 0 //MBksjung not used code : Update isuue of capacity when TA/USB disconnect
			if( ((charger_ic_get_status() == CHARGER_DISABLE) 
						|| (charger_ic_get_state() == CHARGER_STATE_FULLBATTERY))
					&& previous_gauge < batt_info->capacity )
				batt_info->capacity = previous_gauge;
#endif
            DBG(" #############batt_info->capacity[%d]##################\n",batt_info->capacity);
			//battery_update(batt_info);
//MOBII_CHNANGE_S 20120716 jd.park@mobii.co.kr : FOTA UA Upgrade for ICS
#if defined(CONFIG_MACH_STAR_P990) || defined(CONFIG_MACH_STAR_SU660) || defined(CONFIG_MACH_STAR_P999)
			if (ignore_send_uevent == FALSE) {
				power_supply_changed(&batt_info->bat);
			}
#else
			power_supply_changed(&batt_info->bat);
#endif
//MOBII_CHNANGE_E 20120716 jd.park@mobii.co.kr : FOTA UA Upgrade for ICS
		}
	}
}
/* HW requirement: temp control  _S*/
static ssize_t star_temp_control_show_property(
		struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct battery_info *batt_info = refer_batt_info;
	DBG("[TEMP CONTROL]. [%d]", batt_info->temp_control);
	return sprintf(buf, "%d\n", batt_info->temp_control);
}
static ssize_t star_temp_control_store_property(
		struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t count)
{
		struct battery_info *batt_info = refer_batt_info;

		static unsigned int value = 0;

		value = (unsigned int)(simple_strtoul(buf, NULL, 0));
	
		DBG("[TEMP CONTROL] Value : [%d]", value);
		batt_info->temp_control = value;
        DBG("[TEMP CONTROL] 1 is On, 0 is OFF :result[%d]", batt_info->temp_control);
		DBG("[TEMP CONTROL] Battery Update Now!!!!", 0);
		batt_info->health = POWER_SUPPLY_HEALTH_GOOD;
		battery_update_changes(batt_info, 0);
		return count;

}
/* HW requirement: temp control  _E*/

static ssize_t star_battery_show_property(
		struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct battery_info *batt_info = refer_batt_info;
	//DBG("Cap. [%d]", batt_info->capacity);
	return sprintf(buf, "%d\n", batt_info->capacity);
}
static ssize_t star_battery_store_property(
		struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t count)
{
	struct battery_info *batt_info = refer_batt_info;
#if BATT_RTC	
	static unsigned int value = 0, last_cbc = 0;
#else
	static unsigned int value = 0;
#endif
	value = (unsigned int)(simple_strtoul(buf, NULL, 0));

	//DBG("Value : [%d]", value);

	/* Request CBC Cmd. */
	if (value == 52407)
	{
		batt_info->RIL_ready = 1;
		battery_data_polling_period_change();
	}

	if( /*(batt_info->present == 1) &&*/ (value >= 0 && value <= 100) )
	{
#if BATT_RTC	
		if(max8907c_rtc_alarm_count_read(&last_cbc))
			batt_info->last_cbc_time = last_cbc; 
#else
	//              batt_info->last_cbc_time = get_current_time();	
#endif		
		valid_cbc_check_and_process(value);
		//DBG("Battery Info Changed [%d]", value);
	}
	else if( (value < 0 || value > 100) && (value != 52407) )
	{
		batt_info->capacity = 101; 
		return count;
	}
	return count;
}

static ssize_t star_battery_voltage_now_show_property(
		struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct battery_info *batt_info = refer_batt_info;
	DBG("AT_CBC write sysfs Voltage now = [%d]", batt_info->voltage_now);
	return sprintf(buf, "%d\n", batt_info->voltage_now);
}
static ssize_t star_battery_voltage_now_store_property(
		struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t count)
{
	struct battery_info *batt_info = refer_batt_info;
#if BATT_RTC	
	static unsigned int value = 0, last_cbc = 0;
#else
	static unsigned int value = 0;
#endif
	value = (unsigned int)(simple_strtoul(buf, NULL, 0));

	DBG("AT_CBC event Voltage now : [%d]", value);
    batt_info->voltage_now = value;
	return count;
}

//MOBII_CHNANGE_S 20120716 jd.park@mobii.co.kr : FOTA UA Upgrade for ICS
#if defined(CONFIG_MACH_STAR_P990) || defined(CONFIG_MACH_STAR_SU660) || defined(CONFIG_MACH_STAR_P999)
ssize_t ignore_uevent_store(struct device *dev,
              struct device_attribute *attr,
              const char *buf,
              size_t count)
{
	if(buf[0] == '1') {
    	printk("IGNORE SEND UEVENT FOR FOTA\n");
    	ignore_send_uevent = TRUE;
    } else if(buf[0] == '0') {
    	printk("AVAILABLE SEND UEVENT FOR FOTA\n");
    	ignore_send_uevent = FALSE;
    }

    return count;
}
#endif
//MOBII_CHNANGE_E 20120716 jd.park@mobii.co.kr : FOTA UA Upgrade for ICS


/* Sysfs Cmd. Declaration */
DEVICE_ATTR(at_charge, S_IRUGO | S_IWUGO, star_at_charge_show_property, star_at_charge_store_property);
DEVICE_ATTR(at_chomp, S_IRUGO | S_IWUGO, star_at_chcomp_show_property, star_at_chcomp_store_property);
DEVICE_ATTR(dbatt, S_IRUGO | S_IWUGO, star_debug_show_property, star_debug_store_property);
DEVICE_ATTR(true_gauge, S_IRUGO | S_IWUGO, star_cbc_show_property, star_cbc_store_property);
DEVICE_ATTR(bat_gauge, S_IRUGO | S_IWUGO, star_battery_show_property, star_battery_store_property);	// hardware/ril/lge-ril/lge-ril.c
DEVICE_ATTR(temp_control, S_IRUGO | S_IWUGO, star_temp_control_show_property, star_temp_control_store_property);/* HW requirement: temp control  */	
DEVICE_ATTR(voltage_now, S_IRUGO | S_IWUGO, star_battery_voltage_now_show_property, star_battery_voltage_now_store_property);
//MOBII_CHNANGE_S 20120716 jd.park@mobii.co.kr : FOTA UA Upgrade for ICS
#if defined(CONFIG_MACH_STAR_P990) || defined(CONFIG_MACH_STAR_SU660) || defined(CONFIG_MACH_STAR_P999)
DEVICE_ATTR(ignore_uevent, 0644, NULL, ignore_uevent_store);
#endif
//MOBII_CHNANGE_E 20120716 jd.park@mobii.co.kr : FOTA UA Upgrade for ICS

static char *ac_usb_supplied_to[] = {
	"battery",
};

/* Initialize Charger State */
static void star_initial_charger_state(void)
{
	struct battery_info *batt_info = refer_batt_info;
	unsigned int value_status;
	unsigned int value_pgb;
	unsigned char state;

	value_status = read_gpio_status();
	value_pgb = read_gpio_pgb();

	/* Read GPIO State */
	if (( value_status == 1 )&& ( value_pgb == 1 ))
		state = CHGSB_PGB_OFF_OFF;
	else if (( value_status == 0 )&& ( value_pgb == 0 ))
		state = CHGSB_PGB_ON_ON;
	else if (( value_status == 1 )&& ( value_pgb == 0 ))
		state = CHGSB_PGB_OFF_ON;
	else if (( value_status == 0 )&& ( value_pgb == 1 )) {
		state = CHGSB_PGB_ON_OFF;
		DBG("Unknown Charger GPIO");
	}

	/* Determine State */
	switch( state ) { 
		case CHGSB_PGB_OFF_OFF:
			charger_ic_set_state(CHARGER_STATE_SHUTDOWN);
			batt_info->charge_type = POWER_SUPPLY_TYPE_BATTERY;
			DBG("[FULLBAT]: charger_state_init(OFF_OFF: CHARGER_STATE_SHUTDOWN)");
			break;

		case CHGSB_PGB_ON_ON:
			charger_ic_set_state(CHARGER_STATE_CHARGE);
			batt_info->charge_type = POWER_SUPPLY_TYPE_MAINS;
			DBG("[FULLBAT]: charger_state_init(ON_ON: CHARGER_STATE_CHARGE)");
			break;

		case CHGSB_PGB_OFF_ON:
			if ( at_boot_state == 0 ) {
				charger_ic_set_state(CHARGER_STATE_CHARGE);
				batt_info->charge_type = POWER_SUPPLY_TYPE_MAINS;
				DBG("[FULLBAT]: charger_state_init(OFF_ON: CHARGER_STATE_CHARGE)");
			} else if ( at_boot_state == 1 ) {
				charger_ic_set_state(CHARGER_STATE_STANDBY);
				batt_info->charge_type = POWER_SUPPLY_TYPE_MAINS;
				DBG("[FULLBAT]: charger_state_init(OFF_ON: CHARGER_STATE_STANDBY)");
			}
			break;

		default:
			charger_ic_set_state(CHARGER_STATE_SHUTDOWN);
			batt_info->charge_type = POWER_SUPPLY_TYPE_BATTERY;
			break;		
	}
	DBG("[FULLBAT]: charger state machine(%s)", charger_ic_state_machine_name[charger_ic_get_state()]);
}

static int __init battery_probe(struct platform_device *pdev)
{
	DBG();
	struct battery_info *batt_info;
	int ret;

	batt_info = kzalloc(sizeof(*batt_info), GFP_KERNEL);
	if (!batt_info)
		return -ENOMEM;

	memset(batt_info, 0x00, sizeof(*batt_info));

	/* Add driver Pointer */
	batt_info->dev = &pdev->dev;

	/* Add reference Pointer */
	refer_batt_info = batt_info;

	/* Initialize Charger State */
	star_initial_charger_state();

	/* Initialize Variables */
	previous_gauge = 100;
	batt_info->present = 1;
	batt_info->gauge_on = 0;
	batt_info->capacity = 104;
	batt_info->capacity_gauge = 111;
	batt_info->health = POWER_SUPPLY_HEALTH_GOOD;
	batt_info->id_polling_interval = 2 * HZ;
#if	defined (TRICKLE_RECHECK)
	batt_info->trickle_charge_check = 0;
#endif
	batt_info->polling_interval = BATTERY_POLLING_INTERVAL * HZ;
	batt_info->sleep_polling_interval = SLEEP_BATTERY_CHECK_PERIOD;
#if BATT_RTC
	/* Initialize RTC Related Variables */
	batt_info->last_cbc_time = 0;
	batt_info->old_alarm_sec = 0;
	batt_info->old_checkbat_sec = 0;
#endif
	/* Added Variable Initialization : In GB */
	batt_info->RIL_ready = 0;
	batt_info->boot_TA_setting = 0;
	batt_info->voltage = 3700;
	batt_info->temperature = 270;
	batt_info->prev_voltage = batt_info->voltage;
	batt_info->prev_temperature = batt_info->temperature;

    batt_info->temp_control = TEMP_CONTROL_OFF;

	/* Create Battery Workqueue */
	batt_info->battery_workqueue = create_workqueue("SU660_Battery_Workqueue");
	if( batt_info->battery_workqueue == NULL ) {
		ret = -ENOMEM;
		goto create_work_queue_fail;
	}

	/* Initialize Power */
	batt_info->bat.name = "battery";
	batt_info->bat.type = POWER_SUPPLY_TYPE_BATTERY;
	batt_info->bat.properties = battery_props;
	batt_info->bat.num_properties = ARRAY_SIZE(battery_props);
	batt_info->bat.get_property = battery_get_property;

	batt_info->usb.name = "usb";
	batt_info->usb.type = POWER_SUPPLY_TYPE_USB;
	batt_info->usb.supplied_to = ac_usb_supplied_to;
	batt_info->usb.num_supplicants = ARRAY_SIZE(ac_usb_supplied_to);
	batt_info->usb.properties = ac_usb_battery_props;
	batt_info->usb.num_properties = ARRAY_SIZE(ac_usb_battery_props);
	batt_info->usb.get_property = usb_battery_get_property;

	batt_info->ac.name = "ac";
	batt_info->ac.type = POWER_SUPPLY_TYPE_MAINS;
	batt_info->ac.supplied_to = ac_usb_supplied_to;
	batt_info->ac.num_supplicants = ARRAY_SIZE(ac_usb_supplied_to);
	batt_info->ac.properties = ac_usb_battery_props;
	batt_info->ac.num_properties = ARRAY_SIZE(ac_usb_battery_props);
	batt_info->ac.get_property = ac_battery_get_property;

	platform_set_drvdata(pdev, batt_info);

	ret = power_supply_register(&pdev->dev, &batt_info->bat);
	if (ret) {
		DBG("Failed to register main battery");
		goto batt_failed;
	}

	/* Initialize Work Queue */
	INIT_DELAYED_WORK_DEFERRABLE(&batt_info->star_monitor_work, battery_work);
//MBksjung  INIT_DELAYED_WORK_DEFERRABLE(&batt_info->star_id_monitor_work, battery_id_work);
	queue_delayed_work(batt_info->battery_workqueue, &batt_info->star_monitor_work, 6 * HZ);
//MBksjung 	queue_delayed_work(batt_info->battery_workqueue, &batt_info->star_id_monitor_work, 2 * HZ);

	/* Power Supply Register */
	ret = power_supply_register(&pdev->dev, &batt_info->usb);
	if (ret) {
		DBG("Failed to register battery usb online");
		goto usb_online_failed;
	}

	ret = power_supply_register(&pdev->dev, &batt_info->ac);
	if (ret) {
		DBG("Failed to register battery ac online");
		goto ac_online_failed;
	}

	/* AT Cmd. */
	ret = device_create_file(&pdev->dev, &dev_attr_at_charge);
	if (ret) {
		goto at_charge_file_create_fail;
	}

	ret = device_create_file(&pdev->dev, &dev_attr_at_chomp);
	if (ret) {
		goto at_chomp_file_create_fail;
	}

	/* Battery Debug Cmd. */
	ret = device_create_file(&pdev->dev, &dev_attr_dbatt);
	if (ret) {
		goto dbatt_file_create_fail;
	}

	/* Fuel Gauge Cmd. */
	ret = device_create_file(&pdev->dev, &dev_attr_true_gauge);
	if (ret) {
		goto true_gauge_file_create_fail;
	}

	/* Battery Attribute Info. */
	ret = device_create_file(&pdev->dev, &dev_attr_bat_gauge);
	if (ret)
	{
		goto bat_gauge_file_create_fail;
	} 

	/* Battery Temp control Info. */
	ret = device_create_file(&pdev->dev, &dev_attr_temp_control);
	if (ret)
	{
		goto temp_control_file_create_fail;
	}

	ret = device_create_file(&pdev->dev, &dev_attr_voltage_now);
	if (ret)
	{
		goto voltage_now_file_create_fail;
	}

#if defined(READ_TEMP_ADC)
	ret = device_create_file(&pdev->dev, &dev_attr_readtempadc);
	if (ret < 0) {
		pr_err("%s:File device creation failed: %d\n", __func__, ret);
		ret = -ENODEV;
		goto read_file_create_fail;
	}
#endif
//MOBII_CHNANGE_S 20120716 jd.park@mobii.co.kr : FOTA UA Upgrade for ICS
#if defined(CONFIG_MACH_STAR_P990) || defined(CONFIG_MACH_STAR_SU660) || defined(CONFIG_MACH_STAR_P999)
	ret = device_create_file(&pdev->dev, &dev_attr_ignore_uevent);
	if (ret)
	{
		goto ignore_uevent_file_create_fail;
	}
#endif
//MOBII_CHNANGE_E 20120716 jd.park@mobii.co.kr : FOTA UA Upgrade for ICS

	DBG("Sysfs register finished.");

	/* Battery Information Updates */ 
	batt_info->force_update = 1;
	battery_update_changes(batt_info, 0);
	//battery_update(batt_info);
	DBG("Driver probing finished.");

	return 0;

read_file_create_fail:
	device_remove_file(&pdev->dev, &dev_attr_readtempadc);

voltage_now_file_create_fail:
	device_remove_file(&pdev->dev, &dev_attr_voltage_now);

temp_control_file_create_fail:
	device_remove_file(&pdev->dev, &dev_attr_temp_control);

bat_gauge_file_create_fail:
	device_remove_file(&pdev->dev, &dev_attr_true_gauge);
true_gauge_file_create_fail:
	device_remove_file(&pdev->dev, &dev_attr_dbatt);
dbatt_file_create_fail:
	device_remove_file(&pdev->dev, &dev_attr_at_chomp);
at_chomp_file_create_fail:
	device_remove_file(&pdev->dev, &dev_attr_at_charge);
//MOBII_CHNANGE_S 20120716 jd.park@mobii.co.kr : FOTA UA Upgrade for ICS
#if defined(CONFIG_MACH_STAR_P990) || defined(CONFIG_MACH_STAR_SU660) || defined(CONFIG_MACH_STAR_P999)
ignore_uevent_file_create_fail:
	device_remove_file(&pdev->dev, &dev_attr_ignore_uevent);
#endif
//MOBII_CHNANGE_E 20120716 jd.park@mobii.co.kr : FOTA UA Upgrade for ICS
at_charge_file_create_fail:
	power_supply_unregister(&batt_info->usb);
ac_online_failed:
	power_supply_unregister(&batt_info->bat);
usb_online_failed:
	power_supply_unregister(&batt_info->ac);
batt_failed:
	kfree(batt_info);
create_work_queue_fail:
	destroy_workqueue(batt_info->battery_workqueue);
	return ret;
}

#if 1 /* hyunk */
static int battery_remove(struct platform_device *pdev)
#else
static int __exit battery_remove(struct platform_device *pdev)
#endif
{
// MOBII_CHANGE_S [shhong@mobii.co.kr] 2012-04-27 : Kernel Panic Issue Solved.
	DBG("Remove Sequence : Battery No Longer Exists.");
// MOBII_CHANGE_E [shhong@mobii.co.kr] 2012-04-27 : Kernel Panic Issue Solved.
	struct battery_info *batt_info = platform_get_drvdata(pdev);
	bat_shutdown = 1;

	/* Disable Charger IC */
	charger_ic_set_irq(0);
	charger_ic_disable();

	/* Remove Device Works */
	cancel_delayed_work_sync(&batt_info->star_monitor_work);
//MBksjung	cancel_delayed_work_sync(&batt_info->star_id_monitor_work);
	flush_workqueue(batt_info->battery_workqueue);
	destroy_workqueue(batt_info->battery_workqueue);

	/* Remove Powers */
	power_supply_unregister(&batt_info->bat);
	power_supply_unregister(&batt_info->ac);
	power_supply_unregister(&batt_info->usb);

	/* Remove Device Files */
	device_remove_file(&pdev->dev, &dev_attr_at_charge);
	device_remove_file(&pdev->dev, &dev_attr_at_chomp);
	device_remove_file(&pdev->dev, &dev_attr_dbatt);
	device_remove_file(&pdev->dev, &dev_attr_true_gauge);
	device_remove_file(&pdev->dev, &dev_attr_bat_gauge);
	device_remove_file(&pdev->dev, &dev_attr_temp_control);
	device_remove_file(&pdev->dev, &dev_attr_voltage_now);
	device_remove_file(&pdev->dev, &dev_attr_readtempadc);
//MOBII_CHNANGE_S 20120716 jd.park@mobii.co.kr : FOTA UA Upgrade for ICS
#if defined(CONFIG_MACH_STAR_P990) || defined(CONFIG_MACH_STAR_SU660) || defined(CONFIG_MACH_STAR_P999)
	device_remove_file(&pdev->dev, &dev_attr_ignore_uevent);
#endif
//MOBII_CHNANGE_E 20120716 jd.park@mobii.co.kr : FOTA UA Upgrade for ICS
	/* Remove Driver Data */
	platform_set_drvdata(pdev, NULL);
	if(batt_info) {
		kfree(batt_info);
		refer_batt_info = NULL;
	}
	return 0;
}

static int battery_suspend(struct platform_device *pdev,
		pm_message_t state)
{
	struct battery_info *batt_info = platform_get_drvdata(pdev);
#if BATT_RTC	
	static unsigned int alarm_sec = 0, now_sec = 0, checkbat_sec = 0, next_alarm_sec = 0;
	struct rtc_time tm;
#endif
	/* Not Factory Mode */
	if( at_charge_index == 0 ) {
#if BATT_RTC
		/* RTC Setting & Alarm Setting Function */
		if(max8907c_rtc_alarm_count_read(&alarm_sec) 
			&& max8907c_rtc_count_read(&now_sec)) {
	
			if (alarm_sec == batt_info->old_checkbat_sec) {
				alarm_sec = batt_info->old_alarm_sec;
			}
			checkbat_sec = now_sec + batt_info->sleep_polling_interval;
			DBG("[CHG_RTC] alarm_sec=0x%x, now_sec=0x%x", alarm_sec, now_sec);
			DBG("[CHG_RTC] old_alarm_sec=0x%x, checkbat_sec=0x%x", batt_info->old_alarm_sec, checkbat_sec);

			if(batt_info->old_alarm_sec < now_sec) {
				batt_info->old_alarm_sec = 0;
			}

			if ((BAT_MAX(checkbat_sec, alarm_sec) - BAT_MIN(checkbat_sec, alarm_sec)) > 20) {

				if (batt_info->old_alarm_sec == 0) {
					if (checkbat_sec < alarm_sec)   // next battery checking time is earlier than alarm time
					{
						next_alarm_sec = checkbat_sec;
						batt_info->old_alarm_sec = alarm_sec;
						batt_info->old_checkbat_sec = checkbat_sec;
						if (max8907c_rtc_alarm_write(next_alarm_sec))
						{
							rtc_time_to_tm(next_alarm_sec, &tm);
							DBG("rtc_write_to_tm: 1[%04d-%02d-%02d %02d:%02d:%02d]: write_time=0x%x ", 
								(tm.tm_year + LINUX_RTC_BASE_YEAR), tm.tm_mon+1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec, next_alarm_sec);
						} else {
							DBG("[Warning] 1:CHG_RTC write Fail!!");
						}
						DBG("[CHG_RTC : final] next_alarm_sec=0x%x, old_alarm_sec=0x%x", next_alarm_sec, batt_info->old_alarm_sec);
					}
				} else {
					if ((checkbat_sec <= alarm_sec) && (checkbat_sec <= batt_info->old_alarm_sec)) {
						next_alarm_sec = checkbat_sec;
						batt_info->old_alarm_sec = alarm_sec; // Assume: if alarm is changed, system determine new alarm is earlier than old alarm
						batt_info->old_checkbat_sec = checkbat_sec;
						if (max8907c_rtc_alarm_write(next_alarm_sec)) {
							rtc_time_to_tm(next_alarm_sec, &tm);
							DBG("rtc_write_to_tm: 2[%04d-%02d-%02d %02d:%02d:%02d]: write_time=0x%x ", 
								(tm.tm_year + LINUX_RTC_BASE_YEAR), tm.tm_mon+1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec, next_alarm_sec);
						} else {
							DBG("[Warning] 2:CHG_RTC write Fail!!");
						}
						DBG("[CHG_RTC : final] next_alarm_sec=0x%x, old_alarm_sec=0x%x", next_alarm_sec, batt_info->old_alarm_sec);
					} else if ( (batt_info->old_alarm_sec <= alarm_sec) && (batt_info->old_alarm_sec <= checkbat_sec) ) {
						next_alarm_sec = batt_info->old_alarm_sec;
						batt_info->old_alarm_sec = 0;
						batt_info->old_checkbat_sec = 0;
						if (max8907c_rtc_alarm_write(next_alarm_sec)) {
							rtc_time_to_tm(next_alarm_sec, &tm);
							DBG("rtc_write_to_tm: 3[%04d-%02d-%02d %02d:%02d:%02d]: write_time=0x%x ", 
								(tm.tm_year + LINUX_RTC_BASE_YEAR), tm.tm_mon+1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec, next_alarm_sec);
						} else {
							DBG("[Warning] 3:CHG_RTC write Fail!!");
						}
						DBG("[CHG_RTC : final] next_alarm_sec=0x%x, old_alarm_sec=0x%x", next_alarm_sec, batt_info->old_alarm_sec);
					}
				}	// End of if (batt_info->old_alarm_sec == 0) 
			}
			DBG("Skipped this Process");
		} else {
			DBG("Reading RTC Value Failed.");
		}	
#else
	// Do Nothing
#endif
	}	// End of if(at_charge_index == ...)
	pdev->dev.power.power_state = state;

	/* Cancel All works */
	cancel_delayed_work_sync(&batt_info->star_monitor_work);
//MBksjung	cancel_delayed_work_sync(&batt_info->star_id_monitor_work);

	return 0;
}

static int battery_resume(struct platform_device *pdev)
{
	struct battery_info *batt_info = platform_get_drvdata(pdev);

	pdev->dev.power.power_state = PMSG_ON;

	// Update & Resume All works 
//Mobii_Change_S sgkim@mobii.co.kr 20120608- Speed up LCD On
	battery_update_changes(batt_info, 1);
	queue_delayed_work(batt_info->battery_workqueue, &batt_info->star_monitor_work, HZ*40);
//MBksjung	queue_delayed_work(batt_info->battery_workqueue, &batt_info->star_id_monitor_work, HZ*2);
//Mobii_Change_E sgkim@mobii.co.kr 20120608- Speed up LCD On
	return 0;
}

static int battery_shutdown(struct platform_device *pdev)
{
// MOBII_CHANGE_S [shhong@mobii.co.kr] 2012-04-27 : Kernel Panic Issue Solved.
	DBG("Shutdown Sequence : Battery No Longer Exists.");
// MOBII_CHANGE_E [shhong@mobii.co.kr] 2012-04-27 : Kernel Panic Issue Solved.
	struct battery_info *batt_info = platform_get_drvdata(pdev);
	bat_shutdown = 1;

	/* Disable Charger IC */
	charger_ic_set_irq(0);
	charger_ic_disable();

	/* Remove Device Works */
	cancel_delayed_work_sync(&batt_info->star_monitor_work);
//MBksjung	cancel_delayed_work_sync(&batt_info->star_id_monitor_work);
	flush_workqueue(batt_info->battery_workqueue);
	destroy_workqueue(batt_info->battery_workqueue);

	/* Remove Device Files */
	device_remove_file(&pdev->dev, &dev_attr_at_charge);
	device_remove_file(&pdev->dev, &dev_attr_at_chomp);
	device_remove_file(&pdev->dev, &dev_attr_dbatt);
	device_remove_file(&pdev->dev, &dev_attr_true_gauge); 
	device_remove_file(&pdev->dev, &dev_attr_bat_gauge);
    device_remove_file(&pdev->dev, &dev_attr_temp_control);
	device_remove_file(&pdev->dev, &dev_attr_voltage_now);
	device_remove_file(&pdev->dev, &dev_attr_readtempadc);
//MOBII_CHNANGE_S 20120716 jd.park@mobii.co.kr : FOTA UA Upgrade for ICS
#if defined(CONFIG_MACH_STAR_P990) || defined(CONFIG_MACH_STAR_SU660) || defined(CONFIG_MACH_STAR_P999)
	device_remove_file(&pdev->dev, &dev_attr_ignore_uevent);
#endif
//MOBII_CHNANGE_E 20120716 jd.park@mobii.co.kr : FOTA UA Upgrade for ICS
	/* Remove Driver Data */
	platform_set_drvdata(pdev, NULL);
	if(batt_info) {
		kfree(batt_info);
		refer_batt_info = NULL;
	}
	return 0;
}

static struct platform_driver battery_driver =
{
	.probe  	= battery_probe,
	.remove 	= battery_remove,
	.suspend	= battery_suspend,
	.resume 	= battery_resume,
	.shutdown	= battery_shutdown,
	.driver = {
		.name   = "star_battery_charger",
		.owner  = THIS_MODULE,
	},
};

static int __init battery_init(void)
{
	DBG();
	return platform_driver_register(&battery_driver);
}
static int __init array_tp_boot_state(char *str)
{
	int array_tp_boot = (int)simple_strtol(str, NULL, 0);
	if ( 1 == array_tp_boot )
		at_boot_state = 1;

	if (at_boot_state == 1)
		at_charge_index = 1;

	printk(KERN_INFO "jh.kernel: array_tp_boot_state = %d\n", array_tp_boot);

	return 1;
}
__setup("array_tp_boot=", array_tp_boot_state);
static void __exit battery_exit(void)
{
	DBG();
	platform_driver_unregister(&battery_driver);
}
module_init(battery_init);
module_exit(battery_exit);

MODULE_AUTHOR("sagnhyun.hong@lge.com");
MODULE_DESCRIPTION("Battery Driver");
MODULE_LICENSE("GPL");

