/*
 * drivers/power/tegra_odm_battery.c
 *
 * Battery driver for batteries implemented using NVIDIA Tegra ODM kit PMU
 * adaptation interface
 *
 * Copyright (c) 2009, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#define NV_DEBUG 0

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/debugfs.h>
#include <linux/power_supply.h>
#include <linux/wakelock.h>
#include <linux/time.h>
#include <linux/rtc.h>

#include "nvcommon.h"
#include "nvos.h"
#include "nvrm_pmu.h"
//#include "nvodm_pmu.h" //20100929, , RTC setting for checking battery status during sleep
#include "mach/nvrm_linux.h" // for s_hRmGlobal

//20100609, , Charger Code [START]
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/hrtimer.h>
#include "nvodm_query_discovery.h"
//20100609, , Charger Code [END]

#define DEBUGFS_STAR_BATT_TEST_MODE
#define STAR_BATTERY_AT_COMMAND // AT+CBC & AT%FUELRST : power_supply property or switch sysfs
#define USE_ONETIME_VOLTAGE_CAPACITY // Determine Battery capacity using voltage

//20100926, , Communication with BatteryService for AT Command [START]
#ifndef STAR_BATTERY_AT_COMMAND
#include <linux/mutex.h>
#include <linux/switch.h>
#endif // STAR_BATTERY_AT_COMMAND
//20100926, , Communication with BatteryService for AT Command [END]
//20100824, , get capacity using battery voltage for demo [START]
#if defined (USE_ONETIME_VOLTAGE_CAPACITY)
#include "star_battery_charger.h" // 100823 
#endif // USE_ONETIME_VOLTAGE_CAPACITY
//20100824, , get capacity using battery voltage for demo [END]

//20100428, , This define for Debug Message function [START]
#include <mach/lprintk.h>

#define LG_DEBUG_BATT  // Define for Debug Serial
#define LG_DEBUG_CHG // Define for Debug Serial
//#undef LG_DEBUG_BATT
//#undef LG_DEBUG_CHG

#ifdef LG_DEBUG_BATT
#define LDB(fmt, arg...) lprintk(D_BATT, "%s : " fmt "\n", __func__, ## arg)
#else
#define LDB(fmt, arg...) do {} while (0)
#endif

#ifdef LG_DEBUG_CHG
#define LDC(fmt, arg...) lprintk(D_CHARGER, "%s : " fmt "\n", __func__, ## arg)
#else
#define LDC(fmt, arg...) do {} while (0)
#endif
//20100428, , This define for Debug Message function [END]

#define LINUX_RTC_BASE_YEAR 1900

#define STAR_BAT_MIN(x, y) ((x) < (y) ? (x) : (y))
#define STAR_BAT_MAX(x, y) ((x) > (y) ? (x) : (y))

//20100520, , Set Delay time of Charger setting [START]
#define CHG_IC_DELAY            200     // 200us for RT9524 - Star RevA, (100us < delay < 700us)
#define CHG_IC_SET_DELAY        1500    // 1.5ms for RT9524
//20100520, , Set Delay time of Charger setting [END]

#define NVBATTERY_POLLING_INTERVAL 390 /* 90+300 seconds */ // Use with HZ(1sec) const
//20100929, , RTC setting for checking battery status during sleep
#define SLEEP_BAT_CHECK_PERIOD 2090 /* 90 + 2000 seconds */
#define CRITICAL_BAT_CHECK_PERIOD 90 // second
#define CBC_REQUEST_TIME_FIRST	35000 // ms
#define CBC_REQUEST_TIME_SECOND	2000 // ms
#define CBC_REQUEST_TIME_NORMAL	180000 // ms
#define CBC_REQUEST_TIME_CRITICAL 480000 // ms
#define GAUGE_FOLLOW_TIME	120 // second

typedef enum {
	NvCharger_Type_Battery = 0,
	NvCharger_Type_USB,
	NvCharger_Type_AC,
#if defined (STAR_BATTERY_AT_COMMAND)
	NvCharger_Type_AT_Command,
#endif // STAR_BATTERY_AT_COMMAND
	NvCharger_Type_Num,
	NvCharger_Type_Force32 = 0x7FFFFFFF
} NvCharger_Type;

typedef enum {
	NvCharge_Control_Charging_Disable = 0,
	NvCharge_Control_Charging_Enable,
	NvCharge_Control_Num,
	NvCharge_Control_Force32 = 0x7FFFFFFF
} NvCharge_Control;

//20100702, , for Fuel gauge reset [START]
typedef enum {
	Gauge_CTL_Reset = 199,
	Gauge_CTL_Reset_Done = 198
} Gauge_CTL;
//20100702, , for Fuel gauge reset [END]

typedef enum {
	Update_Battery_Data =0,
	Update_USB_Data,
	Update_AC_Data,
	Update_Power_Data,
#if defined (STAR_BATTERY_AT_COMMAND)
	Update_AT_Command,
#endif // STAR_BATTERY_AT_COMMAND
	Update_ALL
} OneTime_Update;

typedef enum {
	Request_First_Time = 0,
	Request_Second_Time,
	Request_Normal_Time,
	Request_Resume_Time,
} CBC_Request_State;

typedef enum {
	Clear_No_Response_0 = 0,
	Clear_No_Response_1,
	Clear_No_Response_2,
	Clear_No_Response_3,
} CBC_Clear_State;

static enum power_supply_property tegra_battery_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_TEMP
};

static enum power_supply_property tegra_power_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

#if defined (STAR_BATTERY_AT_COMMAND)
static enum power_supply_property star_at_command_properties[] = {
	POWER_SUPPLY_PROP_CAPACITY_DUTY_TIME,
};
#endif // STAR_BATTERY_AT_COMMAND

static char *supply_list[] = {
	"battery",
};

static int at_boot_state = 0;
static NvBool at_charge_index = NV_FALSE;
static NvBool at_charge_on = NV_FALSE;
static NvBool at_charge_comp = NV_FALSE;
static NvBool ELT_test_mode = NV_FALSE;

static NvU32 previous_guage=100;

static int bat_shutdown = 0;

//20100706, , For Full Battery process [START]
typedef enum {
	FULL_BATT_BOTH_NO = 0,
	FULL_BATT_GAUGE,
	FULL_BATT_CHARGER,
	FULL_BATT_BOTH,
	FULL_BATT_SET,
} full_battery_status;

typedef enum {
	CHGSB_PGB_OFF_OFF = 0,
	CHGSB_PGB_ON_ON,
	CHGSB_PGB_OFF_ON,
	CHGSB_PGB_ON_OFF, // Infact, this is not ued
} charge_ic_status_transition;

typedef enum {
	CHARGER_STATE_SHUTDOWN = 0,
	CHARGER_STATE_STANDBY,
	CHARGER_STATE_CHARGE,
	CHARGER_STATE_FULLBATTERY,
	CHARGER_STATE_RECHARGE,
} charge_ic_state_machine;
//20100706, , For Full Battery process [END]

//20100609, , Charger Code [START]
typedef enum {
	CHG_IC_DEFAULT_MODE=0,    		/* 0  */
	CHG_IC_TA_MODE,
	CHG_IC_USB_LO_MODE,
	CHG_IC_FACTORY_MODE,

	CHG_IC_DEACTIVE_MODE,			/* 4  */
	CHG_IC_INIT_MODE,
} max8922_status;

typedef struct CHG_IC_DeviceRec
{
	NvOdmServicesGpioHandle hGpio;
	NvOdmGpioPinHandle hSetPin;
	NvOdmGpioPinHandle hStatusPin;
	NvOdmGpioPinHandle hPGBPin;
	NvOdmServicesGpioIntrHandle hCHGDone_int;
	max8922_status status;
	//20100702, , for Fuel gauge reset [START]
	max8922_status status_past;
	//20100702, , for Fuel gauge reset [END]
} CHG_IC_Device;

static CHG_IC_Device *charging_ic;

max8922_status get_charging_ic_status(void)
{
	return charging_ic->status;
}
EXPORT_SYMBOL(get_charging_ic_status);
//20100609, , Charger Code [END]

//20100915, , For AT_BOOT [START]
NvBool ARRAY_TP_BOOT(void)
{
	return at_boot_state;
}
EXPORT_SYMBOL(ARRAY_TP_BOOT);
//20100915, , For AT_BOOT  [END]

static int tegra_power_get_property(struct power_supply *psy, 
	enum power_supply_property psp, union power_supply_propval *val);

static int tegra_battery_get_property(struct power_supply *psy, 
	enum power_supply_property psp, union power_supply_propval *val);

static int star_at_command_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val);

static void star_debug_show_battery_status(void);
static int star_battery_infomation_update(void);
static void charger_control_with_battery_temp(void);
static void star_battery_data_onetime_update(NvU8 update_option);
static void star_gauge_follower_func(void);
static void star_cbc_gauge_request_timer_func(unsigned long unused);

static struct power_supply tegra_supplies[] = {
	{
		.name = "battery",
		.type = POWER_SUPPLY_TYPE_BATTERY,
		.properties = tegra_battery_properties,
		.num_properties = ARRAY_SIZE(tegra_battery_properties),
		.get_property = tegra_battery_get_property,
	},
	{
		.name = "usb",
		.type = POWER_SUPPLY_TYPE_USB,
		.supplied_to = supply_list,
		.num_supplicants = ARRAY_SIZE(supply_list),
		.properties = tegra_power_properties,
		.num_properties = ARRAY_SIZE(tegra_power_properties),
		.get_property = tegra_power_get_property,
	},
	{
		.name = "ac",
		.type = POWER_SUPPLY_TYPE_MAINS,
		.supplied_to = supply_list,
		.num_supplicants = ARRAY_SIZE(supply_list),
		.properties = tegra_power_properties,
		.num_properties = ARRAY_SIZE(tegra_power_properties),
		.get_property = tegra_power_get_property,
	},
#if defined (STAR_BATTERY_AT_COMMAND)
	{
		.name = "battery_monitoring",
		.type = POWER_SUPPLY_TYPE_BATTERY_MONITORING,
		.properties = star_at_command_properties,
		.num_properties = ARRAY_SIZE(star_at_command_properties),
		.get_property = star_at_command_get_property,
	},
#endif // STAR_BATTERY_AT_COMMAND
};

//20100926, , Communication with BatteryService for AT Command [START]
typedef enum {
	AT_COMM_NO = 0,
	AT_COMM_CBC,
	AT_COMM_FUELRST,
	AT_COMM_CBCDC,
} at_comm_status;

#ifndef STAR_BATTERY_AT_COMMAND
typedef struct switch_for_at_command {
	struct switch_dev	sdev;
	struct mutex	mutex_lock;
	int	flag_at_comm_want;
	int	flag_at_comm_ready;
} at_command_switch;

static at_command_switch *AtComm;

static ssize_t at_comm_print_name(struct switch_dev *sdev, char *buf)
{
	switch (switch_get_state( &AtComm->sdev))
	{
		case AT_COMM_NO:
			return sprintf(buf, "No AT Command\n");
		case AT_COMM_CBC:
			return sprintf(buf, "AT+CBC\n");
		case AT_COMM_FUELRST:
			return sprintf(buf, "AT%%FUELVAL\n");
		case AT_COMM_CBCDC:
			return sprintf(buf, "AT%%CBCDC\n");
	}
	return -EINVAL;
}
#endif // STAR_BATTERY_AT_COMMAND
//20100926, , Communication with BatteryService for AT Command [END]

typedef struct tegra_battery_dev {
	struct	workqueue_struct *battery_workqueue;
	struct	delayed_work battery_status_poll_work;
	struct	delayed_work battery_id_poll_work;
	//struct	delayed_work battery_charge_done_work;
	struct	timer_list charger_state_read_timer;
	struct	timer_list battery_gauge_timer;

	//NvU32	batt_id;		/* battery ID from ADC */
	NvU32	batt_vol;		/* voltage from ADC */
	NvS32	batt_temp;		/* temperature (degrees C) */
	NvU32	batt_health;
	NvU32	batt_chemtech;
	//NvU32	batt_current;		/* current from ADC */
	NvU32	charging_source;	/* 0: no cable, 1:usb, 2:AC */
	NvU32	charging_enabled;	/* 0: Disable, 1: Enable */
	//NvU32	full_bat;		/* max capacity of battery (mAh) */
	//20100706, , For Full Battery process [START]
	NvU32	cbc_request_time;
	NvU8	charger_state_machine;
	max8922_status	charger_setting_chcomp;
	NvU32	sleep_bat_check_period;
	NvU32	sleep_bat_check_period_to_cp;
	//20100706, , For Full Battery process [END]
	NvU32	BatteryLifePercent;
	NvU32	BatteryGauge;
	NvU32	CBC_Value;
	NvBool	BatteryGauge_on;
	NvU32	Capacity_Voltage;
	NvBool	Capacity_first_time;
	NvBool	Boot_TA_setting;
	NvBool	RIL_ready;
	//20100824, , get capacity using battery voltage for demo [START]
#if defined (USE_ONETIME_VOLTAGE_CAPACITY)
	NvBool	repeat_index;
	NvU32	vol_for_capacity;
#endif // USE_ONETIME_VOLTAGE_CAPACITY
	//20100824, , get capacity using battery voltage for demo [END]
	//NvU32	BatteryLifeTime;
	//NvU32	BatteryMahConsumed;
	NvU32	ACLineStatus;
	NvU32	battery_poll_interval;
	NvBool	present;
#if defined (STAR_BATTERY_AT_COMMAND)
	at_comm_status	at_comm_want;
	NvBool	at_comm_ready;
#endif // STAR_BATTERY_AT_COMMAND
	NvU32	old_alarm_sec;
	NvU32	old_checkbat_sec;
	NvU32	last_cbc_time;
	NvU32	gauge_follow_time;
	CBC_Request_State	cbc_request_state;
	CBC_Clear_State		cbc_clear_state;
} tegra_battery_dev;

static tegra_battery_dev *batt_dev;

typedef struct star_battery_dev {
	NvU32	batt_vol;
	NvS32	batt_temp;
	NvU32	batt_health;
	NvU32	charging_source;	/* 0: no cable, 1:usb, 2:AC */
	NvU32	charging_enabled;	/* 0: Disable, 1: Enable */
	NvU32	BatteryLifePercent;
	NvU32	ACLineStatus;
	NvU8	charger_state_machine;
	NvBool	present;
}	star_battery_dev;
static star_battery_dev *star_batt_dev;

#if defined(DEBUGFS_STAR_BATT_TEST_MODE)
typedef struct debug_battery_dev {
	NvU32	batt_vol;
	NvS32	batt_temp;
	NvU32	BatteryLifePercent;
	NvBool	present;
}	debug_battery_dev;
static debug_battery_dev *debug_batt_dev;

enum {
	BATT_TEST_MODE = 0,
	BATT_TEST_CAPACITY,
	BATT_TEST_VOLT,
	BATT_TEST_TEMP,
	BATT_TEST_PRESENT,
};
enum {
	STAR_BATT_TEST_MODE_OFF = 0,
	STAR_BATT_TEST_MODE_ON = 1,
};
int	star_battery_test_mode=STAR_BATT_TEST_MODE_OFF;

struct star_batt_test_reg {
	const char *name;
	unsigned int code;
};

#define STAR_BATT_TEST_REG(_name, _code) { .name = _name, .code = _code}

static struct star_batt_test_reg star_batt_test_regs[] = {
	STAR_BATT_TEST_REG("TestMode",  	 BATT_TEST_MODE),
	STAR_BATT_TEST_REG("TestCapacity",    BATT_TEST_CAPACITY),
	STAR_BATT_TEST_REG("TestVoltage",    BATT_TEST_VOLT),
	STAR_BATT_TEST_REG("TestTemperature", BATT_TEST_TEMP),
	STAR_BATT_TEST_REG("TestPresent", BATT_TEST_PRESENT),
};

static int star_battery_debug_set(void *data, u64 val)
{
	struct star_batt_test_reg *p_star_batt_test_reg = data;
	int rtn = 0;

	lprintk(D_BATT, "%s() : p_star_batt_test_reg->code = %d	\n", __func__ , p_star_batt_test_reg->code );

	switch (p_star_batt_test_reg->code)
	{
		case BATT_TEST_MODE:
			star_battery_test_mode = (int)val;
			if (star_battery_test_mode == STAR_BATT_TEST_MODE_ON)
			{
				debug_batt_dev->batt_vol = 4000;
				debug_batt_dev->batt_temp = 260;
				debug_batt_dev->BatteryLifePercent = 50;
				debug_batt_dev->present = NV_TRUE;
			}
			break;
		case BATT_TEST_CAPACITY:
			if ( star_battery_test_mode == STAR_BATT_TEST_MODE_ON )
			{
				debug_batt_dev->BatteryLifePercent = (NvU32)val;
			}
			break;
		case BATT_TEST_VOLT:
			if ( star_battery_test_mode == STAR_BATT_TEST_MODE_ON )
			{
				debug_batt_dev->batt_vol = (NvU32)val;
			}
			break;
		case BATT_TEST_TEMP :
			if ( star_battery_test_mode == STAR_BATT_TEST_MODE_ON )
			{
				debug_batt_dev->batt_temp = (NvS32)val;
			}
			break;
		case BATT_TEST_PRESENT:
			if ( star_battery_test_mode == STAR_BATT_TEST_MODE_ON )
			{
				debug_batt_dev->present = (NvBool)val;
			}
			break;
		default:
			rtn = -EINVAL;
			break;
	}

	cancel_delayed_work_sync(&batt_dev->battery_status_poll_work);
	queue_delayed_work(batt_dev->battery_workqueue, &batt_dev->battery_status_poll_work, 1*HZ);

	return rtn;

}

static int star_battery_debug_get(void *data, u64 *val)
{
	struct star_batt_test_reg *p_star_batt_test_reg = data;
	int rtn = 0;

	lprintk(D_BATT, "%s() : p_star_batt_test_reg->code = %d	\n", __func__ , p_star_batt_test_reg->code );
	switch (p_star_batt_test_reg->code)
	{
		case BATT_TEST_MODE:
			*val = (u64)star_battery_test_mode;
			break;
		case BATT_TEST_CAPACITY:
			*val = (u64)(debug_batt_dev->BatteryLifePercent);
			break;
		case BATT_TEST_VOLT:
			*val = (u64)(debug_batt_dev->batt_vol);
			break;
		case BATT_TEST_TEMP:
			*val = (u64)(debug_batt_dev->batt_temp);
			break;
		case BATT_TEST_PRESENT:
			*val = (u64)(debug_batt_dev->present);
			break;
		default:
			rtn = -EINVAL;
			break;
	}

	return rtn;
}

DEFINE_SIMPLE_ATTRIBUTE(star_battery_test_fops, star_battery_debug_get, star_battery_debug_set, "%llu\n");

void star_battery_debug_init(void)
{
	struct dentry *dent;
	int n;

	dent = debugfs_create_dir("star_battery_test", 0);
	if (IS_ERR(dent))
		lprintk(D_BATT, "%s() : debugfs_create_dir() Error !!!! 	\n", __func__ );

	for (n = 0; n < ARRAY_SIZE(star_batt_test_regs); n++)
	{
		debugfs_create_file(star_batt_test_regs[n].name, 0666, dent, \
			               &star_batt_test_regs[n],	&star_battery_test_fops);
	}
}
#endif // DEBUGFS_STAR_BATT_TEST_MODE
static void change_battery_value_for_test(void)
{
	batt_dev->batt_vol = 			debug_batt_dev->batt_vol;
	batt_dev->batt_temp = 		debug_batt_dev->batt_temp;
	batt_dev->BatteryLifePercent = debug_batt_dev->BatteryLifePercent;
	batt_dev->present = 			debug_batt_dev->present;
}

static void tegra_battery_status_poll_work(struct work_struct *work);
static void star_battery_id_poll_work(struct work_struct *work);
static void tegra_battery_gpio_read_timer_func(unsigned long unused); // BatteryTest
static void star_cp_at_comm_request(void); // CBCDC Test

//20100609, , Charger Code [START]
static void charging_ic_active_for_recharge(NvU32 Mode)
{
	NvU32 pulse;

	switch (batt_dev->charger_state_machine)
	{
		case CHARGER_STATE_CHARGE:
		case CHARGER_STATE_RECHARGE:
			batt_dev->charger_state_machine = CHARGER_STATE_SHUTDOWN;

		case CHARGER_STATE_STANDBY:
		case CHARGER_STATE_FULLBATTERY:
		case CHARGER_STATE_SHUTDOWN:
			if ( at_boot_state == 0 )
			{
				// Initialize Chager before charging mode setting
				NvOdmGpioSetState( charging_ic->hGpio, charging_ic->hSetPin, 0x1);
				NvOdmOsWaitUS( CHG_IC_SET_DELAY );

				NvOdmGpioSetState( charging_ic->hGpio, charging_ic->hSetPin, 0x0);
				NvOdmOsWaitUS( CHG_IC_SET_DELAY );
			}
			else
				LDC("[FACTORY] at_boot_state == 1, no effect in operation");
			break;

		default:
			LDB("[Critical] Unknown Charger state!!!");
			break;
	}

	for ( pulse=0; pulse<Mode; pulse++ )
	{
		NvOdmGpioSetState( charging_ic->hGpio, charging_ic->hSetPin, 0x1);
		NvOdmOsWaitUS(CHG_IC_DELAY);

		NvOdmGpioSetState( charging_ic->hGpio, charging_ic->hSetPin, 0x0);
		NvOdmOsWaitUS(CHG_IC_DELAY);
	}

	NvOdmGpioSetState( charging_ic->hGpio, charging_ic->hSetPin, 0);
	NvOdmOsWaitUS(CHG_IC_SET_DELAY);

	batt_dev->charger_state_machine = CHARGER_STATE_CHARGE;

	//LDC("[chg] Charger setting(%d)", Mode);
}

static void star_charger_activation_work(NvU32 Mode)
{
	charging_ic_active_for_recharge(Mode);

	charging_ic->status = (max8922_status)Mode;
	//LDC("[chg] charging_ic->status = (%d)", charging_ic->status);

	batt_dev->ACLineStatus = NV_TRUE;
	batt_dev->charging_enabled = NV_TRUE;

	switch( charging_ic->status )
	{
		case CHG_IC_DEFAULT_MODE:
		case CHG_IC_USB_LO_MODE:
			batt_dev->charging_source = NvCharger_Type_USB;
			star_battery_data_onetime_update(Update_Power_Data);
			break;

		case CHG_IC_TA_MODE:
		case CHG_IC_FACTORY_MODE:
			batt_dev->charging_source = NvCharger_Type_AC;
			star_battery_data_onetime_update(Update_Power_Data);
			break;

		default:
			star_battery_data_onetime_update(Update_Power_Data);
			LDC("[Critical] Unknown Charger setting :: Exception!!!");
			break;
	}
}

void charging_ic_active(NvU32 Mode)
{
	{
		if ((charging_ic->status != CHG_IC_FACTORY_MODE) || ( Mode != 3 /* CHG_IC_FACTORY_MODE */))
		{
			if ( batt_dev->present == NV_TRUE )
			{
				charging_ic->status_past = charging_ic->status; // Why??
				//charging_ic->status = (max8922_status)Mode;
				//LDC("[Present_Battery] charging_ic->status(%d)", charging_ic->status);
				star_charger_activation_work(Mode);
			}
			else // batt_dev->present == NV_FALSE : for developer
			{
				charging_ic->status_past = charging_ic->status; // Why??
				charging_ic->status = (max8922_status)Mode;
				LDC("[No_Battery] charging_ic->status(%d)", charging_ic->status);
				//star_charger_activation_work(Mode);
			}
		}

	//20100528, , for debug [START]
#ifdef LG_DEBUG_CHG
		switch ((max8922_status)Mode)
		{
			case CHG_IC_TA_MODE:
				LDC("[chg]: charging_ic->status = CHG_IC_TA_MODE(%d)", charging_ic->status);
				break;

			case CHG_IC_USB_LO_MODE:
				LDC("[chg]: charging_ic->status = CHG_IC_USB_LO_MODE(%d)", charging_ic->status);
				break;

			case CHG_IC_FACTORY_MODE:
				LDC("[chg]: charging_ic->status = CHG_IC_FACTORY_MODE(%d)", charging_ic->status);
				break;

			case CHG_IC_DEFAULT_MODE:
				LDC("[chg]: charging_ic->status = CHG_IC_DEFAULT_MODE(%d)", charging_ic->status);
				break;

			default:
				LDC("[Critical]: charging_ic->status = Unknown MODE???(%d)", charging_ic->status);
		}
#endif // LG_DEBUG_CHG
	//20100528, , for debug [END]

		if (batt_dev->batt_health != POWER_SUPPLY_HEALTH_GOOD)
		{
			batt_dev->batt_health = POWER_SUPPLY_HEALTH_GOOD;
			charger_control_with_battery_temp();
		}
	}
}
EXPORT_SYMBOL(charging_ic_active);

static void charging_ic_deactive_for_rechrge(void)
{
	if ( at_boot_state == 0 )
	{
		// Deactivation IC
		NvOdmGpioSetState( charging_ic->hGpio, charging_ic->hSetPin, 0x1);
		NvOdmOsWaitUS( CHG_IC_SET_DELAY );
		LDC("[chg] charger deactive");
	}
	else
		LDC("[FACTORY] at_boot_state ==0, no operation");
}

static void star_charger_deactivation_work(void)
{
	// Deactivation IC
	//LDC("[chg] Start charging_ic->status = %d", charging_ic->status);
	charging_ic_deactive_for_rechrge();

	charging_ic->status = CHG_IC_DEACTIVE_MODE;
	//LDC("[chg] End charging_ic->status = CHG_IC_DEACTIVE_MODE(%d)", charging_ic->status);

	batt_dev->ACLineStatus = NV_FALSE;
	batt_dev->charging_enabled = NV_FALSE;
	batt_dev->charging_source = NvCharger_Type_Battery;

	star_battery_data_onetime_update(Update_Power_Data);
}

void charging_ic_deactive(void)
{
	star_charger_deactivation_work();
}
EXPORT_SYMBOL(charging_ic_deactive);
//20100609, , Charger Code [END]

//20100813, , For Debugging [START]
static void star_debug_show_battery_status(void)
{
	lprintk(D_BATT, "%s : \n\n BAT : SOC(%d), Vol(%d), Temp(%d), present(%d),\n       CV(%d), CBC(%d, %d, %d), Boot_TA(%d) \n       StateMachine(%d), AT_Boot_ELT(%d, %d, %d), poll/wake(%d, %d)\n CHG : AcLine(%d), src(%d), en(%d),\n       past(%d) full_set(%d) now(%d)\n TIME : Last_CBC(%u) GaugeFollow(%u)\n\n"
						, __func__
						, batt_dev->BatteryLifePercent
						, batt_dev->batt_vol
						, batt_dev->batt_temp
						, batt_dev->present
						, batt_dev->Capacity_Voltage
						, batt_dev->BatteryGauge
						, batt_dev->CBC_Value
						, batt_dev->BatteryGauge_on
						, batt_dev->Boot_TA_setting
						, batt_dev->charger_state_machine
						, at_boot_state , at_charge_index, ELT_test_mode
						, (batt_dev->battery_poll_interval/HZ)
						, batt_dev->sleep_bat_check_period
						, batt_dev->ACLineStatus
						, batt_dev->charging_source
						, batt_dev->charging_enabled
						, charging_ic->status_past
						, batt_dev->charger_setting_chcomp
						, charging_ic->status
						,batt_dev->last_cbc_time
						,batt_dev->gauge_follow_time
	);
}
//20100813, , For Debugging [END]

static void star_capacity_from_voltage_via_calculate(void)
{
	NvU32	calculate_capacity = 0;

	if ((batt_dev->ACLineStatus == NV_TRUE) && ((batt_dev->charger_state_machine == CHARGER_STATE_RECHARGE) || (batt_dev->charger_state_machine == CHARGER_STATE_CHARGE)))
	{
		if ( batt_dev->batt_vol <= 3828 )
		{
			if (  batt_dev->batt_vol < 3656 )
				calculate_capacity = 1;
			else
				calculate_capacity = (NvU32)((batt_dev->batt_vol*1000000 - 3655943209)/6239571);
		}
		else if (( batt_dev->batt_vol > 3828 ) && ( batt_dev->batt_vol <= 3878 ))
		{
			calculate_capacity = (NvU32)((batt_dev->batt_vol*1000000 - 3814262889)/5230892);
		}
		else if (( batt_dev->batt_vol > 3878 ) && ( batt_dev->batt_vol <= 3943 ))
		{
			calculate_capacity = (NvU32)((batt_dev->batt_vol*1000000 - 3804342997)/6035547);
		}
		else if (( batt_dev->batt_vol > 3943 ) && ( batt_dev->batt_vol <= 4005 ))
		{
			calculate_capacity = (NvU32)((batt_dev->batt_vol*1000000 - 3891922385)/2243847);
		}
		else if (( batt_dev->batt_vol > 4005 ) && ( batt_dev->batt_vol <= 4133 ))
		{
			calculate_capacity = (NvU32)((batt_dev->batt_vol*1000000 - 3736359651)/5330744);
		}
		else if ( batt_dev->batt_vol > 4133 )
		{
			if ( batt_dev->batt_vol >= 4200 )
				calculate_capacity = 100;
			else
				calculate_capacity = (NvU32)((batt_dev->batt_vol*1000000 - 3953389838)/2419419);
		}

		if ( calculate_capacity < 0 ) calculate_capacity = 1;
		if ( calculate_capacity > 100 ) calculate_capacity = 99;
		batt_dev->Capacity_Voltage = calculate_capacity;

		LDB("[CBC] : With Charger : CAL_CBC(%d)", calculate_capacity);
	}
	else if ((batt_dev->ACLineStatus == NV_FALSE) || (batt_dev->charger_state_machine == CHARGER_STATE_FULLBATTERY))
	{
		if ( batt_dev->batt_vol >= 3970 )
		{
			if ( batt_dev->batt_vol >= 4200 )
				calculate_capacity = 100;
			else
				calculate_capacity = (NvU32)((batt_dev->batt_vol*1000000 - 3223959227)/8766094);
		}
		else if (( batt_dev->batt_vol >= 3798 ) && ( batt_dev->batt_vol < 3970 ))
		{
			calculate_capacity = (NvU32)((batt_dev->batt_vol*1000000 - 3382701485)/6900848);
		}
		else if (( batt_dev->batt_vol >= 3713 ) && ( batt_dev->batt_vol < 3798 ))
		{
			calculate_capacity = (NvU32)((batt_dev->batt_vol*1000000 - 3582377778)/3588889);
		}
		else if (( batt_dev->batt_vol >= 3675 ) && ( batt_dev->batt_vol < 3713 ))
		{
			calculate_capacity = (NvU32)((batt_dev->batt_vol*1000000 - 3629265282)/2307994);
		}
		else if (( batt_dev->batt_vol >= 3575 ) && ( batt_dev->batt_vol < 3675 ))
		{
			calculate_capacity = (NvU32)((batt_dev->batt_vol*1000000 - 3548445378)/6386555);
		}
		else if (( batt_dev->batt_vol >= 3500 ) && ( batt_dev->batt_vol < 3675 ))
		{
			calculate_capacity = (NvU32)((batt_dev->batt_vol*1000000 - 3410416667)/39583333);
		}
		else if ( batt_dev->batt_vol < 3500 )
		{
			if (  batt_dev->batt_vol <= 3400 )
				calculate_capacity = 1;
			else
				calculate_capacity = (NvU32)((batt_dev->batt_vol*1000000 - 3300357143)/88214286);
		}

		if ( calculate_capacity < 0 ) calculate_capacity = 1;
		if ( calculate_capacity > 100 ) calculate_capacity = 99;
		batt_dev->Capacity_Voltage = calculate_capacity;

		LDB("[CBC] : Without Charger : CAL_CBC(%d)", calculate_capacity);
	}
}

static void true_valid_cbc_process(NvU32 cbc_value)
{
	static NvU32 display_cbc = 0;

	if( batt_dev->BatteryLifePercent !=104 )									// at first process, do not update
		previous_guage= batt_dev->BatteryLifePercent;		// save previous value
		
	batt_dev->CBC_Value = cbc_value;

	if (cbc_value <= 0)
		display_cbc = 1;
	else if (cbc_value >= 92)
		display_cbc = 100;
	else
		display_cbc = ( cbc_value * 100 / 92 );

	batt_dev->BatteryLifePercent = display_cbc;
	batt_dev->BatteryGauge = display_cbc;
	batt_dev->BatteryGauge_on = NV_TRUE;

	printk("[BATT] previous value = %d, current_value=%d\n", previous_guage , batt_dev->BatteryLifePercent);
	if( ((batt_dev->ACLineStatus == NV_FALSE) || (batt_dev->charger_state_machine == CHARGER_STATE_FULLBATTERY)) 
		&& previous_guage< batt_dev->BatteryLifePercent   )
	{
		batt_dev->BatteryLifePercent = previous_guage;
	}
	printk("[BATT] final value = %d\n", batt_dev->BatteryLifePercent);
	
	star_battery_data_onetime_update(Update_Battery_Data);
}

static void valid_cbc_check_and_process(NvU32 cbc_value)
{
	static NvU32 display_cbc = 0;
	if (bat_shutdown) return;

	//if  ( at_charge_index == NV_FALSE )
	{
//20100926, , Communication with BatteryService for AT Command [START]
#if defined (STAR_BATTERY_AT_COMMAND)
		if (batt_dev->at_comm_want == AT_COMM_CBC)
		{
			//LDB("[at_comm]:[Receive] AT Command(%d)", batt_dev->at_comm_want);
			batt_dev->at_comm_want = AT_COMM_NO;
			batt_dev->at_comm_ready = NV_TRUE;
		}
#else // STAR_BATTERY_AT_COMMAND
		if (AtComm->flag_at_comm_want == AT_COMM_CBC)
		{
			//LDB("[at_comm]:[Receive] AT Command(%d)", switch_get_state(&AtComm->sdev));
			mutex_lock(&AtComm->mutex_lock);
			switch_set_state(&AtComm->sdev, (switch_get_state(&AtComm->sdev) & AT_COMM_NO));
			mutex_unlock(&AtComm->mutex_lock);
			AtComm->flag_at_comm_want = AT_COMM_NO;
			AtComm->flag_at_comm_ready = NV_TRUE;
		}
#endif // STAR_BATTERY_AT_COMMAND
//20100926, , Communication with BatteryService for AT Command [END]

		batt_dev->CBC_Value = cbc_value;

		star_battery_infomation_update();
		star_capacity_from_voltage_via_calculate();

		if ( cbc_value <= 3 )
			display_cbc = 1;
		else if ( cbc_value >= 95 )
			display_cbc = 100;
		else if (( cbc_value >= 92 ) && ( cbc_value < 95 ))
			display_cbc = 99;
		else
			display_cbc = ( (cbc_value-2) * 100 / 91 );

		if (batt_dev->batt_vol <= 3400)
			display_cbc = 0;

		LDB("[gauge]: Display_gauge_value(%d)", display_cbc);

		if (batt_dev->BatteryGauge_on == NV_FALSE)
		{
			if (cbc_value <= 2)
			{
				if (((batt_dev->batt_vol < 3500) || (batt_dev->BatteryLifePercent <= 5)))
				{
					batt_dev->BatteryLifePercent = 1;
					batt_dev->BatteryGauge = 1;
					batt_dev->BatteryGauge_on = NV_TRUE;
				}
				else
				{
					batt_dev->BatteryGauge = display_cbc;
					//mod_timer(&(batt_dev->battery_gauge_timer), jiffies + msecs_to_jiffies(100));
				}
			}
			else if (cbc_value == 100)
			{
				if (((batt_dev->batt_vol > 4100) || (batt_dev->BatteryLifePercent >= 93)))
				{
					batt_dev->BatteryLifePercent = 100;
					batt_dev->BatteryGauge = 100;
					batt_dev->BatteryGauge_on = NV_TRUE;
				}
				else
				{
					batt_dev->BatteryGauge = display_cbc;
					//mod_timer(&(batt_dev->battery_gauge_timer), jiffies + msecs_to_jiffies(100));
				}
			}
			else
			{
			    batt_dev->BatteryLifePercent = display_cbc;
			    batt_dev->BatteryGauge = display_cbc;
			    batt_dev->BatteryGauge_on = NV_TRUE;
			}
		}

		if (batt_dev->BatteryGauge_on == NV_TRUE)
		{
			star_batt_dev->BatteryLifePercent = batt_dev->BatteryLifePercent;
			if( batt_dev->BatteryLifePercent !=104 )									// at first process, do not update
				previous_guage = batt_dev->BatteryLifePercent;		// save previous value

			if (cbc_value <= 2)
			{
				if (((batt_dev->batt_vol < 3450) || (batt_dev->BatteryLifePercent <= 3)))
				{
					batt_dev->BatteryLifePercent = 1;
					batt_dev->BatteryGauge = 1;
				}
				else if (((batt_dev->batt_vol < 3400) || (batt_dev->BatteryLifePercent <= 3)))
				{
					batt_dev->BatteryLifePercent = 0;
					batt_dev->BatteryGauge = 0;
				}
			}
			else if (cbc_value == 100)
			{
				if (((batt_dev->batt_vol > 4130) || (batt_dev->BatteryLifePercent >= 96)))
				{
					batt_dev->BatteryLifePercent = 100;
					batt_dev->BatteryGauge = 100;
				}
			}
			else // ( 0 < cbc_value < 100 )
			{
				if ( display_cbc >= batt_dev->BatteryLifePercent ) //((batt_dev->ACLineStatus == NV_TRUE) && ((batt_dev->batt_health == POWER_SUPPLY_HEALTH_GOOD) || (batt_dev->batt_health == POWER_SUPPLY_HEALTH_OVERHEAT)))
				{
					if ((display_cbc - batt_dev->BatteryLifePercent) <= 1)
					{
						batt_dev->BatteryLifePercent = display_cbc;
						batt_dev->BatteryGauge = display_cbc;
					}
					else if ((display_cbc - batt_dev->BatteryLifePercent) > 1)
					{
							batt_dev->BatteryGauge = display_cbc;
						star_gauge_follower_func();
					}
				}
				else if ( display_cbc < batt_dev->BatteryLifePercent ) // ((batt_dev->ACLineStatus == NV_FALSE) || ((batt_dev->batt_health != POWER_SUPPLY_HEALTH_GOOD) && (batt_dev->batt_health != POWER_SUPPLY_HEALTH_OVERHEAT)))
				{
					if ((batt_dev->BatteryLifePercent - display_cbc) <= 1)
					{
						batt_dev->BatteryLifePercent = display_cbc;
						batt_dev->BatteryGauge = display_cbc;
					}
					else if ((batt_dev->BatteryLifePercent - display_cbc) > 1)
					{
						batt_dev->BatteryGauge = display_cbc;
							star_gauge_follower_func();
					}
				}
				batt_dev->BatteryGauge = display_cbc;

				if (batt_dev->BatteryLifePercent < 0) batt_dev->BatteryLifePercent = 0;
				if (batt_dev->BatteryLifePercent > 100) batt_dev->BatteryLifePercent = 100;
			}

			// if previous gauge is larger than current gauge during discharging, keep the previous gauge value.
			if( ((batt_dev->ACLineStatus == NV_FALSE) || (batt_dev->charger_state_machine == CHARGER_STATE_FULLBATTERY)) 
				&& previous_guage < batt_dev->BatteryLifePercent )
			{
				batt_dev->BatteryLifePercent = previous_guage ;
			}
			
			star_battery_data_onetime_update(Update_Battery_Data);
		}
	}
}

static ssize_t tegra_battery_show_property(
		struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	return sprintf(buf, "%d\n", batt_dev->BatteryLifePercent);
}

static ssize_t tegra_battery_store_property(
		struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t count)
{
	static NvU32 value = 0;

	value = (NvU32)(simple_strtoul(buf, NULL, 0));
	LDB("[gauge]: gauge_value(%d)", value);

//20100702, , for Fuel gauge reset [START]
	if ( value == Gauge_CTL_Reset_Done )
	{
		//star_battery_gauge_ic_reset();
#if defined (STAR_BATTERY_AT_COMMAND)
		if (batt_dev->at_comm_want == AT_COMM_FUELRST)
		{
			//LDB("[at_comm]: Battery AT Command_RST(%d)", batt_dev->at_comm_want);
			batt_dev->at_comm_want = AT_COMM_NO;
			batt_dev->at_comm_ready = NV_TRUE;
		}
#else //
		if (AtComm->flag_at_comm_want == AT_COMM_FUELRST)
		{
			//LDB("[at_comm]: AT Command_RST(%d)", switch_get_state(&AtComm->sdev));
			mutex_lock(&AtComm->mutex_lock);
			switch_set_state(&AtComm->sdev, (switch_get_state(&AtComm->sdev) & AT_COMM_NO));
			mutex_unlock(&AtComm->mutex_lock);
			AtComm->flag_at_comm_ready = NV_TRUE;
		}
#endif // STAR_BATTERY_AT_COMMAND
	}
//20100702, , for Fuel gauge reset [END]

	if (value == 52407)
	{
		batt_dev->RIL_ready = NV_TRUE;
		mod_timer(&(batt_dev->battery_gauge_timer), jiffies + msecs_to_jiffies(3000));
	}

	if ((batt_dev->present == NV_TRUE) && (value >= 0 && value <= 100))
	{
		batt_dev->last_cbc_time = NvOsGetTimeMS();
		valid_cbc_check_and_process(value);
	}
	else if ((value < 0 || value > 100) && (value != 52407))
	{
		lprintk(D_BATT, "%s: [Critical] Unexpected Battery gauge value from CP & RIL!!!(%d) \n", __func__, value);
		batt_dev->BatteryLifePercent = 101; // default setting
		return count;
	}
//20100702, , for Fuel gauge reset [END]
	return count;
}

static struct device_attribute tegra_battery_attr = {
	.attr = { .name = "bat_gauge", .mode = S_IRUGO | S_IWUGO,
			  .owner = THIS_MODULE },
	.show = tegra_battery_show_property,
	.store = tegra_battery_store_property,
};

static NvBool star_charger_disable_ok_check(void)
{
	if (batt_dev->ACLineStatus == NV_TRUE)
	{
		if (batt_dev->charging_source == NvCharger_Type_AC)
		{
			if (batt_dev->batt_vol > 3400)
				return NV_TRUE;
			else
				return NV_FALSE;
		}
		else if (batt_dev->charging_source == NvCharger_Type_USB)
		{
			if (batt_dev->batt_vol > 3400)
				return NV_TRUE;
			else
				return NV_FALSE;
		}
	}
	else // batt_dev->ACLineStatus == NV_FALSE --> batt_dev->charging_source = NvCharger_Type_Battery
	{
		return NV_FALSE;
	}
}

static ssize_t star_cbc_show_property(
		struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	LDB("Execute OK!");
	if (batt_dev->BatteryGauge_on == NV_FALSE)
	{
		if ((at_charge_index == NV_FALSE) && (batt_dev->charger_state_machine != CHARGER_STATE_FULLBATTERY) && (charging_ic->status != CHG_IC_DEACTIVE_MODE))
		{
			LDB("Health(%d), Vol(%d)", batt_dev->batt_health, batt_dev->batt_vol);
			if  (((batt_dev->batt_health == POWER_SUPPLY_HEALTH_OVERHEAT) ||(batt_dev->batt_health == POWER_SUPPLY_HEALTH_GOOD)) && (star_charger_disable_ok_check() == NV_TRUE))
			{
				LDB("[Gauge]: Charger control for Fuel Gauge");
				batt_dev->Boot_TA_setting = NV_TRUE;
				batt_dev->charger_state_machine = CHARGER_STATE_SHUTDOWN;
				batt_dev->charger_setting_chcomp = charging_ic->status;
				charging_ic_deactive_for_rechrge();
			}
		}
	}

	return sprintf(buf, "%d\n", batt_dev->BatteryLifePercent);
}

static ssize_t star_cbc_store_property(
		struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t count)
{
	static NvU32 value = 0;

	value = (NvU32)(simple_strtoul(buf, NULL, 0));
	LDB("[gauge]: sk_gauge_value(%d)", value);

	if (value != 999)
	{
		if (value <= 3)
			batt_dev->BatteryLifePercent = 1;
		else if (value >= 95)
			batt_dev->BatteryLifePercent = 100;
		else if ((value >= 92) && (value < 95))
			batt_dev->BatteryLifePercent = 99;
		else
			batt_dev->BatteryLifePercent = ( (value-2) * 100 / 91 );

		batt_dev->CBC_Value = value;
		batt_dev->BatteryGauge_on = NV_TRUE;
	}

	if (batt_dev->Boot_TA_setting == NV_TRUE)
	{
		LDB("[Gauge]: Charger control for Fuel Gauge : rollback state");
		if  (batt_dev->batt_health == POWER_SUPPLY_HEALTH_GOOD)
		{
			batt_dev->charger_state_machine = CHARGER_STATE_CHARGE;
			charging_ic_active_for_recharge(batt_dev->charger_setting_chcomp);
			batt_dev->Boot_TA_setting = NV_FALSE;
		}
		else if (batt_dev->batt_health == POWER_SUPPLY_HEALTH_OVERHEAT)
		{
			batt_dev->charger_state_machine = CHARGER_STATE_CHARGE;
			charging_ic_active_for_recharge(CHG_IC_DEFAULT_MODE);
			batt_dev->Boot_TA_setting = NV_FALSE;
		}
	}

	return count;
}

static struct device_attribute star_cbc_attr = {
	.attr = { .name = "true_gauge", .mode = S_IRUGO | S_IWUGO,
			  .owner = THIS_MODULE },
	.show = star_cbc_show_property,
	.store = star_cbc_store_property,
};

//20100810, , for Debug [START]
static ssize_t star_debug_show_property(
		struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	size_t count=0;

	star_debug_show_battery_status();

	count = sprintf(buf, "%d\n", batt_dev->battery_poll_interval);

	return count;
}

static ssize_t star_debug_store_property(
		struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t count)
{
	static NvU32 value = 0;

	value = (NvU32)(simple_strtoul(buf, NULL, 0));

	if ((value == 2171) && (ELT_test_mode == NV_FALSE))
	{
		ELT_test_mode = NV_TRUE;
	}
	else if ((value == 2170) && (ELT_test_mode == NV_TRUE))
	{
		ELT_test_mode = NV_FALSE;
	}
	else
	{
		batt_dev->battery_poll_interval = HZ*value;
		cancel_delayed_work_sync(&batt_dev->battery_status_poll_work);
		queue_delayed_work(batt_dev->battery_workqueue, &batt_dev->battery_status_poll_work, HZ/60);
		//LDB("[bat_sysfs]: Change battery_poll_interval(%d, %d)", value, batt_dev->battery_poll_interval );
	}

	return count;
}

static struct device_attribute star_debug_attr = {
	.attr = { .name = "dbatt", .mode = S_IRUGO | S_IWUGO,
			  .owner = THIS_MODULE },
	.show = star_debug_show_property,
	.store = star_debug_store_property,
};
//20100810, , for Debug [END]

//20100810, , for CBCDC [START]
static ssize_t star_cbcdc_show_property(
		struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	size_t count=0;

	count = sprintf(buf, "%d\n", batt_dev->sleep_bat_check_period_to_cp);

	return count;
}

static ssize_t star_cbcdc_store_property(
		struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t count)
{
	static NvU32 value = 0;

	value = (NvU32)(simple_strtoul(buf, NULL, 0));

	if ( value == AT_COMM_NO )
	{
//20100926, , Communication with BatteryService for AT Command [START]
#if defined (STAR_BATTERY_AT_COMMAND)
		if (batt_dev->at_comm_want != AT_COMM_NO)
		{
			//LDB("[at_comm]:[Receive] AT Command(%d)", batt_dev->at_comm_want);
			batt_dev->at_comm_want = AT_COMM_NO;
			batt_dev->at_comm_ready = NV_TRUE;
		}
#else // STAR_BATTERY_AT_COMMAND
		if (AtComm->flag_at_comm_want != AT_COMM_NO)
		{
			//LDB("[at_comm]:[Receive] AT Command(%d)", switch_get_state(&AtComm->sdev));
			mutex_lock(&AtComm->mutex_lock);
			switch_set_state(&AtComm->sdev, (switch_get_state(&AtComm->sdev) & AT_COMM_NO));
			mutex_unlock(&AtComm->mutex_lock);
			AtComm->flag_at_comm_want = AT_COMM_NO;
			AtComm->flag_at_comm_ready = NV_TRUE;
		}
#endif // STAR_BATTERY_AT_COMMAND
//20100926, , Communication with BatteryService for AT Command [END]
	}

	return count;
}

static struct device_attribute star_cbcdc_attr = {
	.attr = { .name = "cbc_dc", .mode = S_IRUGO | S_IWUGO,
			  .owner = THIS_MODULE },
	.show = star_cbcdc_show_property,
	.store = star_cbcdc_store_property,
};
//20100810, , for CBCDC [END]

//20100706, , Do not use battery chemistry [START]
// PMU just return fixed value "NICD" so do not use this function
// Some Battery Appl. requires technology property... so use again.. 20101028
static void tegra_get_battery_tech(int *Value,
	NvRmPmuBatteryInstance NvBatteryInst)
{
	NvRmPmuBatteryChemistry Chemistry = {0};

	NvRmPmuGetBatteryChemistry(s_hRmGlobal, NvBatteryInst, &Chemistry);

	switch(Chemistry)
	{
		case NvRmPmuBatteryChemistry_NICD:
			 *Value = POWER_SUPPLY_TECHNOLOGY_NiCd;
			 break;

		case NvRmPmuBatteryChemistry_NIMH:
			 *Value = POWER_SUPPLY_TECHNOLOGY_NiMH;
			 break;

		case NvRmPmuBatteryChemistry_LION:
			 *Value = POWER_SUPPLY_TECHNOLOGY_LION;
			 break;

		case NvRmPmuBatteryChemistry_LIPOLY:
			 *Value = POWER_SUPPLY_TECHNOLOGY_LIPO;
			 break;

		default:
			 *Value = POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
			 break;
	}
}
//20100706, , Do not use battery chemistry [END]

#if 0
static void tegra_battery_convert(NvRmPmuBatteryData *pPmuData)
{
	//STAR only use Voltage & Temperature data
	
	//if (pPmuData->batteryLifePercent == NVODM_BATTERY_DATA_UNKNOWN)
	//	pPmuData->batteryLifePercent = 0;

	//if (pPmuData->batteryLifeTime == NVODM_BATTERY_DATA_UNKNOWN)
	//	pPmuData->batteryLifeTime = 0;

	if (pPmuData->batteryVoltage == NVODM_BATTERY_DATA_UNKNOWN)
		pPmuData->batteryVoltage = 0;

	//if (pPmuData->batteryCurrent == NVODM_BATTERY_DATA_UNKNOWN)
	//	pPmuData->batteryCurrent = 0;

	//if (pPmuData->batteryMahConsumed == NVODM_BATTERY_DATA_UNKNOWN)
	//	pPmuData->batteryMahConsumed = 0;

	if (pPmuData->batteryTemperature == NVODM_BATTERY_DATA_UNKNOWN)
		pPmuData->batteryTemperature = 0;
}

static int tegra_battery_data(NvRmPmuBatteryInstance NvBatteryInst)
{	
	NvRmPmuBatteryData pPmuData = {0};

	if (!NvRmPmuGetBatteryData(s_hRmGlobal, NvBatteryInst, &pPmuData))
		return -1;

	if (NvBatteryInst == NvRmPmuBatteryInst_Main) {
		tegra_battery_convert(&pPmuData);
		batt_dev->batt_vol = pPmuData.batteryVoltage; // Already in [mV] scale..
		//batt_dev->batt_vol *= 1000; // Convert volt to mV
		//batt_dev->batt_current = pPmuData.batteryCurrent;
		batt_dev->batt_temp = pPmuData.batteryTemperature-2730; // Conversion from [K] to [C] <- *10
		//batt_dev->batt_temp *= 10;  // FIXME : Why is this here? <- Framework need 10*temperature, ex) 25.2C -> 252
		//batt_dev->BatteryLifePercent = pPmuData.batteryLifePercent;
		//batt_dev->BatteryLifeTime = pPmuData.batteryLifeTime;
		//batt_dev->BatteryMahConsumed = pPmuData.batteryMahConsumed;
	}

	return 0;
}
#endif // STAR_BATTERY_UPDATE

static int star_battery_infomation_update(void)
{
	NvRmPmuAcLineStatus AcStatus = NvRmPmuAcLine_Offline;
	NvU8 BatStatus = 0;
	NvRmPmuBatteryData BatData = {0};

	star_batt_dev->batt_vol = batt_dev->batt_vol;
	star_batt_dev->batt_temp = batt_dev->batt_temp;
	star_batt_dev->charging_source = batt_dev->charging_source;
	star_batt_dev->charging_enabled = batt_dev->charging_enabled;
	star_batt_dev->ACLineStatus = batt_dev->ACLineStatus;
	star_batt_dev->present = batt_dev->present;

	//LDB("[bat_poll] Start!!!");
	if (NvRmPmuUpdateBatteryInfo(s_hRmGlobal, &AcStatus, &BatStatus, &BatData) && NvRmPmuGetAcLineStatus(s_hRmGlobal, &AcStatus))
	{
		switch (AcStatus)
		{
			case NvRmPmuAcLine_Offline:
				batt_dev->ACLineStatus = NV_FALSE;
				batt_dev->charging_enabled = NV_FALSE;
				batt_dev->charging_source = NvCharger_Type_Battery;
				break;

			case NvRmPmuAcLine_Online:
				batt_dev->ACLineStatus = NV_TRUE;
				switch (get_charging_ic_status())
				{
					case CHG_IC_DEFAULT_MODE:
					case CHG_IC_USB_LO_MODE:
						batt_dev->charging_enabled = NV_TRUE;
						batt_dev->charging_source = NvCharger_Type_USB;
						//LDB("[bat_poll] NvCharger_Type_USB(%d)", NvCharger_Type_USB);
						break;

					case CHG_IC_TA_MODE:
					case CHG_IC_FACTORY_MODE:
						batt_dev->charging_enabled = NV_TRUE;
						batt_dev->charging_source = NvCharger_Type_AC;
						//LDB("[bat_poll] NvCharger_Type_AC(%d)", NvCharger_Type_AC);
						break;

					case CHG_IC_DEACTIVE_MODE:
						batt_dev->charging_enabled = NV_FALSE;
						batt_dev->charging_source = NvCharger_Type_Battery;
						//LDB("[bat_poll] NvCharger_Type_Battery(%d)", NvCharger_Type_Battery);
						break;

					default:
						LDB("[Critical]: Exceptional Charger Setting!!!!");
						return -EINVAL;
				}
				break;

			default:
				LDB("[Critical]: Exceptional ACLine Status!!!!");
				break;
		}

		//20100916, , For AT_Boot [START]
		if (( at_boot_state == 1 ) && (BatStatus == NVODM_BATTERY_STATUS_NO_BATTERY))
		{
			if ( charging_ic->status == CHG_IC_DEACTIVE_MODE )
				BatStatus = NVODM_BATTERY_STATUS_DISCHARGING;
			else
				BatStatus = NVODM_BATTERY_STATUS_CHARGING;
		}
		//20100916, , For AT_Boot [END]

		switch (BatStatus)
		{
			case NVODM_BATTERY_STATUS_NO_BATTERY:
				batt_dev->present = NV_FALSE;
				batt_dev->BatteryLifePercent = 0; // default setting
				//batt_dev->repeat_index = NV_TRUE;
				//LDB("[bat_poll] NVODM_BATTERY_STATUS_NO_BATTERY(BatStatus:%d)", BatStatus);
				break;

			case NVODM_BATTERY_STATUS_CHARGING:
				batt_dev->present = NV_TRUE;
				//LDB("[bat_poll] POWER_SUPPLY_STATUS_CHARGING(BatStatus:%d)", BatStatus);
				break;

			case NVODM_BATTERY_STATUS_DISCHARGING:
				batt_dev->present = NV_TRUE;
				//LDB("[bat_poll] POWER_SUPPLY_STATUS_DISCHARGING(BatStatus:%d)", BatStatus);
				break;

			default:
				LDB("[Critical]: Exception in BatStatus!!!");
				batt_dev->present = NV_TRUE;
				batt_dev->BatteryLifePercent = 102; // default setting
				//batt_dev->repeat_index = NV_TRUE;
				//LDB("[bat_poll] POWER_SUPPLY_STATUS_default(BatStatus:%d)", BatStatus);
				break;
		}

		if ((BatData.batteryVoltage != NVODM_BATTERY_DATA_UNKNOWN) && (BatData.batteryTemperature != NVODM_BATTERY_DATA_UNKNOWN))
		{
			//if (3250 > BatData.batteryVoltage) BatData.batteryVoltage = 3250;
			//if (4250 < BatData.batteryVoltage) BatData.batteryVoltage = 4250;
			if ((3200 < BatData.batteryVoltage) && (BatData.batteryVoltage < 4400))
			{
				if (batt_dev->Capacity_first_time)
				{
					batt_dev->vol_for_capacity = BatData.batteryVoltage;
					batt_dev->batt_vol = BatData.batteryVoltage;
				}
				else
				{
					batt_dev->vol_for_capacity = batt_dev->batt_vol;
					if (3400 >= BatData.batteryVoltage)
						batt_dev->batt_vol = (NvU32)BatData.batteryVoltage;
					else if ((3450 < BatData.batteryVoltage) && (4050 > BatData.batteryVoltage))
						batt_dev->batt_vol = (NvU32)((batt_dev->batt_vol*3 + BatData.batteryVoltage)/4); // Already in [mV] scale..
					else
						batt_dev->batt_vol = (NvU32)((batt_dev->batt_vol + BatData.batteryVoltage)/2); // Already in [mV] scale..
				}
			}
			else
			{
				LDB("[Warning]: Battery voltage range is out of range for normal battery (old:%d, now:%d)", batt_dev->batt_vol, BatData.batteryVoltage);
				if (3200 > BatData.batteryVoltage)
				{
					if ( 3400 >= batt_dev->batt_vol ) // previous value also Low Battery state... use this value.
					{
						batt_dev->batt_vol = (NvU32)BatData.batteryVoltage;
					}
					else // previous value is high... why this is occur??
					{
						if (NvRmPmuUpdateBatteryInfo(s_hRmGlobal, &AcStatus, &BatStatus, &BatData)) // Recheck Battery Data
						{
							LDB("[Warning]: Recheck Battery because of abnormal battery voltage (old:%d, now:%d)", batt_dev->batt_vol, BatData.batteryVoltage);
							batt_dev->batt_vol = (NvU32)BatData.batteryVoltage; // system will be shut down when low voltage again. if not, use re-read value.
						}
						else
						{
							LDB("[Critical]: NvRmPmuUpdateBatteryInfo failed!!!");
							return NV_FALSE;
						}
					}
				}
			}

			if (batt_dev->batt_vol <= 3400)
			{
				if (batt_dev->BatteryLifePercent <= 1 )
					batt_dev->BatteryLifePercent = 0;
				else if (batt_dev->BatteryLifePercent > 1)
					batt_dev->BatteryGauge = 0;
			}

			if ((at_boot_state == 1) || ( at_charge_index == NV_TRUE ) || (ELT_test_mode == NV_TRUE))
				batt_dev->batt_temp = 270; // Factory Test Mode, fake Temperature value
			else
				batt_dev->batt_temp = BatData.batteryTemperature-2730; // Conversion from [K] to [C] <- *10

			tegra_get_battery_tech(&batt_dev->batt_chemtech, NvRmPmuBatteryInst_Main);
		}
		//LDB("[bat_poll] Complete !!!");
		return NV_TRUE;
	}
	else
	{
		LDB("[Critical]: NvRmPmuUpdateBatteryInfo or NvRmPmuGetAcLineStatus failed!!!");
		return NV_FALSE;
	}
}

static int tegra_power_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val)
{
	//LDB("[bat_poll]");
	switch (psp)
	{
		case POWER_SUPPLY_PROP_ONLINE:
			switch (psy->type)
			{
				case POWER_SUPPLY_TYPE_MAINS:
					val->intval = (batt_dev->charging_source ==  NvCharger_Type_AC);
					//LDB("[bat_poll] type: POWER_SUPPLY_TYPE_MAINS(%d), intval: NvCharger_Type_AC(%d)", psp, val->intval);
					break;

				case POWER_SUPPLY_TYPE_USB:
					val->intval = (batt_dev->charging_source ==  NvCharger_Type_USB);
					//LDB("[bat_poll] type: POWER_SUPPLY_TYPE_USB(%d), intval: NvCharger_Type_USB(%d)", psp, val->intval);
					break;

				default:
					val->intval = 0;
					LDB("[Critical]: Unknown charge source property");
					break;
			}
			break;

		default:
			LDB("[Critical] Exception in Online Property!!");
			val->intval = 0;
			return -EINVAL;
	}
	return 0;
}

static int tegra_battery_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val)
{
	//LDB("[bat_poll]");
	if (star_battery_test_mode == STAR_BATT_TEST_MODE_ON)
		change_battery_value_for_test();

	switch (psp)
	{
		case POWER_SUPPLY_PROP_STATUS:
			if (batt_dev->BatteryGauge_on == NV_FALSE) // Not yet receive CBC from CP
			{
				val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
				LDB("[Warning] Cannot receive CBC from CP until now, Display Battery loading Icon!!");
			}
			else if (batt_dev->present == NV_FALSE) // NVODM_BATTERY_STATUS_NO_BATTERY
			{
				val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
				//LDB("[bat_poll] intval: NVODM_BATTERY_STATUS_NO_BATTERY(%d)", val->intval);
			}
			else if ((batt_dev->batt_health == POWER_SUPPLY_HEALTH_CRITICAL_OVERHEAT) || (batt_dev->batt_health == POWER_SUPPLY_HEALTH_COLD))
			{
				val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
				//LDB("[bat_poll] intval: POWER_SUPPLY_STATUS_NOT_CHARGING(%d)", val->intval);
			}
			else if ((batt_dev->BatteryLifePercent == 100) && (batt_dev->charging_enabled == NV_TRUE))
			{
				val->intval = POWER_SUPPLY_STATUS_FULL;
				//LDB("[bat_poll] intval: POWER_SUPPLY_STATUS_FULL(%d)", val->intval);
			}
			else if (batt_dev->charging_enabled == NV_TRUE) // POWER_SUPPLY_STATUS_CHARGING
			{
				val->intval = POWER_SUPPLY_STATUS_CHARGING;
				//LDB("[bat_poll] intval: POWER_SUPPLY_STATUS_CHARGING(%d)", val->intval);
			}
			else if (batt_dev->charging_enabled == NV_FALSE) // NVODM_BATTERY_STATUS_DISCHARGING
			{
				val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
				//LDB("[bat_poll] intval: POWER_SUPPLY_STATUS_DISCHARGING(%d)", val->intval);
			}
			else // POWER_SUPPLY_STATUS_UNKNOWN
			{
				val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
				LDB("[Critical] Why this is occured?? Battery Status Unknown");
			}
			break;

		case POWER_SUPPLY_PROP_PRESENT:
			val->intval = batt_dev->present;
			//LDB("[bat_poll] intval: POWER_SUPPLY_PROP_PRESENT(%d)", val->intval);
			break;

		case POWER_SUPPLY_PROP_TECHNOLOGY:
			val->intval = batt_dev->batt_chemtech;
			//LDB("[bat_poll] intval: POWER_SUPPLY_PROP_TECHNOLOGY(%d)", val->intval);
			break;

		case POWER_SUPPLY_PROP_HEALTH:
			if (batt_dev->batt_health == POWER_SUPPLY_HEALTH_CRITICAL_OVERHEAT)
				val->intval = POWER_SUPPLY_HEALTH_OVERHEAT;
			else
				val->intval = batt_dev->batt_health;
			//LDB("[bat_poll] intval: POWER_SUPPLY_PROP_HEALTH(%d)", val->intval);
			break;

		case POWER_SUPPLY_PROP_VOLTAGE_NOW:
			val->intval = batt_dev->batt_vol * 1000;
			//LDB("[bat_poll] intval: POWER_SUPPLY_PROP_VOLTAGE_NOW(%d)", val->intval);
			break;

		case POWER_SUPPLY_PROP_CAPACITY:
			if (batt_dev->BatteryGauge_on == NV_TRUE)
				val->intval = batt_dev->BatteryLifePercent;
			else
				val->intval = 999;
			//LDB("[bat_poll] intval: POWER_SUPPLY_PROP_CAPACITY(%d)", val->intval);
			break;

		case POWER_SUPPLY_PROP_TEMP:
			val->intval = batt_dev->batt_temp;
			//LDB("[bat_poll] intval: POWER_SUPPLY_PROP_TEMP(%d)", val->intval);
			break;

		default:
			LDB("[Critical] Why this is occured?? Unknown battery property");
			return -EINVAL;
	}
	return 0;
}

#if defined (STAR_BATTERY_AT_COMMAND)
static int star_at_command_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val)
{
	//LDB("[Debug] psp(%d) : POWER_SUPPLY_PROP_CAPACITY_DUTY_TIME==31", psp);
	switch (psp)
	{
		case POWER_SUPPLY_PROP_CAPACITY_DUTY_TIME:
			if (batt_dev->at_comm_want == AT_COMM_NO) // Everytime, you execute AT command, AT_COMM_NO is returned to framework
			{
				val->intval = AT_COMM_NO;
				batt_dev->at_comm_ready = NV_FALSE;
				//LDB("[at_comm]:[Send] AT_COMM_NO(%d)", val->intval);
			}
			else // if (batt_dev->at_comm_ready == NV_TRUE) // If you want to execute AT command, set AT_COMM_CBC or AT_COMM_FUELRST and must set at_comm_ready to NV_TURE
			{
				if (batt_dev->at_comm_want == AT_COMM_CBCDC)
				{
					val->intval = AT_COMM_CBCDC;
					batt_dev->at_comm_ready = NV_FALSE;
					//LDB("[at_comm]:[Send] AT_COMM_CBCDC(%d)", val->intval);
				}
				else if (batt_dev->at_comm_want == AT_COMM_CBC)
				{
					val->intval = AT_COMM_CBC;
					batt_dev->at_comm_ready = NV_FALSE;
					//LDB("[at_comm]:[Send] AT_COMM_CBC(%d)", val->intval);
				}
				else if (batt_dev->at_comm_want == AT_COMM_FUELRST)
				{
					// Infact, if you want execute FUELRST, you have to disable charger before update sysfs.
					val->intval = AT_COMM_FUELRST;
					batt_dev->at_comm_ready = NV_FALSE;
					LDB("[at_comm]:[Send] AT_COMM_FUELRST(%d)", val->intval);
				}
				else val->intval = AT_COMM_NO;
			}
			break;

		default:
			LDB("[Critical]:[Send] No AT_COMM property!!!");
			return -EINVAL;
	}
	return 0;
}
#endif // STAR_BATTERY_AT_COMMAND

//20100824, , get capacity using battery voltage for demo [START]
#if defined (USE_ONETIME_VOLTAGE_CAPACITY)
static NvBool valid_capacity_gauge(void)
{
	NvU32 capacity_gauge_temp;

	star_batt_dev->BatteryLifePercent = batt_dev->BatteryLifePercent;

	if (batt_dev->Capacity_first_time)
	{
		batt_dev->Capacity_first_time = NV_FALSE; // Let Capacity_first_time to NV_FALSE here only. It is require for determine capacity from battery voltage in first time, No dependent to previous voltage level
		if (batt_dev->BatteryGauge_on == NV_FALSE)
			batt_dev->BatteryLifePercent = STAR_BAT_MIN(batt_dev->Capacity_Voltage + 3 , 100);
		star_batt_dev->BatteryLifePercent = batt_dev->BatteryLifePercent;
	}
	else
	{
		if (batt_dev->ACLineStatus == NV_TRUE)
		{
			if (batt_dev->BatteryLifePercent >= batt_dev->Capacity_Voltage)
			{
				capacity_gauge_temp = batt_dev->BatteryLifePercent;
			}
			else // higher then previous gauge
			{
				capacity_gauge_temp = batt_dev->BatteryLifePercent + 1; // Gauge tracking is juct one step!
			}
		}
		else // batt_dev->ACLineStatus == NV_FALSE
		{
			if (batt_dev->BatteryLifePercent <= batt_dev->Capacity_Voltage)
			{
				capacity_gauge_temp = batt_dev->BatteryLifePercent;
			}
			else
			{
				capacity_gauge_temp = batt_dev->BatteryLifePercent - 1; // Gauge tracking is juct one step!
			}
		}

		if (batt_dev->BatteryGauge_on == NV_FALSE)
		{
			batt_dev->BatteryLifePercent = capacity_gauge_temp;
			if (STAR_BAT_MAX(batt_dev->BatteryGauge, batt_dev->Capacity_Voltage) - STAR_BAT_MIN(batt_dev->BatteryGauge, batt_dev->Capacity_Voltage) <= 5)
			{
				batt_dev->BatteryLifePercent = batt_dev->BatteryGauge;
				batt_dev->BatteryGauge_on = NV_TRUE;
			}
		}
	}

	if (batt_dev->BatteryLifePercent < 0)
	{
		batt_dev->BatteryLifePercent = 0;
		LDB("[Critical] Capacity gauging has exceptinal value!!!: below 0");
	}
	else if (batt_dev->BatteryLifePercent > 100)
	{
		batt_dev->BatteryLifePercent = 100;
		LDB("[Critical] Capacity gauging has exceptinal value!!!: above 100");
	}

	return NV_TRUE;
}

static NvBool capacity_with_charger_source_USB(void)
{
	static NvU16 capacity_table[] = BAT_CV_USB_TABLE;
	static NvU8 capacity_index = 0, range_index = 0;
	static NvU32 temp_vol;

	//temp_vol = (NvU32)((batt_dev->batt_vol + 2*batt_dev->vol_for_capacity)/3);
	temp_vol = batt_dev->batt_vol;
	if (batt_dev->Capacity_first_time) // In first time, search range is from 0 to 100 (whole range)
	{
		for (capacity_index = 100; capacity_index >= 0 ; capacity_index--)
		{
			if ((capacity_table[capacity_index] <= temp_vol) || (capacity_index == 0))
				goto set_BatteryLifePercent_w_USB;
		}
		capacity_index = 0;
		goto set_BatteryLifePercent_w_USB;
	}
	else // After first time, search range is start from slightly reverse direction of previous gauge, and go forward to increase direction (because "Charging state" now) 
	{
		range_index = batt_dev->BatteryLifePercent;
		for (; range_index >= 0; range_index--)
		{
			if ((capacity_table[range_index] <= temp_vol) || (range_index == 0))
				break;
		}
		capacity_index = range_index;

		for (; capacity_index <= 100 ; capacity_index++ )
		{
			if ((capacity_table[capacity_index] >= temp_vol) || (capacity_index == 100))
			{
				goto set_BatteryLifePercent_w_USB;
			}
		}
		capacity_index = 100;
		goto set_BatteryLifePercent_w_USB;
	}

	return NV_FALSE;

set_BatteryLifePercent_w_USB:
	if ((capacity_index >= 0) || (capacity_index <= 100))
	{
		batt_dev->Capacity_Voltage = capacity_index;
		//LDB("[bat_cv] CV_w gauge USB(%d)", batt_dev->Capacity_Voltage);
	}
	else // After first time, search range is start from slightly reverse direction of previous gauge, and go forward to increase direction (because "Charging state" now) 
	{
		lprintk(D_BATT, "%s: [Critical] Unexpected Battery gauge value!!!(%d) \n", __func__, capacity_index);
		batt_dev->Capacity_Voltage = 30; // default setting
	}

	return NV_TRUE;
}

static NvBool capacity_with_charger_source_TA(void)
{
	static NvU16 capacity_table[] = BAT_CV_TA_TABLE;
	static NvU8 capacity_index = 0, range_index = 0;
	static NvU32 temp_vol;

	//temp_vol = (NvU32)((batt_dev->batt_vol + 2*batt_dev->vol_for_capacity)/3);
	temp_vol = batt_dev->batt_vol;
	if (batt_dev->Capacity_first_time) // In first time, search range is from 0 to 100 (whole range)
	{
		for (capacity_index = 100; capacity_index >= 0 ; capacity_index--)
		{
			if ((capacity_table[capacity_index] <= temp_vol) || (capacity_index == 0))
				goto set_BatteryLifePercent_w_TA;
		}
		capacity_index = 0;
		goto set_BatteryLifePercent_w_TA;
	}
	else
	{
		range_index = batt_dev->BatteryLifePercent;
		for (; range_index >= 0; range_index-- )
		{
			if ((capacity_table[range_index] <= temp_vol) || (range_index == 0))
				break;
		}
		capacity_index = range_index;

		for (; capacity_index <= 100 ; capacity_index++ )
		{
			if ((capacity_table[capacity_index] >= temp_vol) || (capacity_index == 100))
			{
				goto set_BatteryLifePercent_w_TA;
			}
		}
		capacity_index = 100;
		goto set_BatteryLifePercent_w_TA;
	}

	return NV_FALSE;

set_BatteryLifePercent_w_TA:
	if ((capacity_index >= 0) || (capacity_index <= 100))
	{
		batt_dev->Capacity_Voltage = capacity_index;
		//LDB("[bat_cv] CV_w gauge TA(%d)", batt_dev->Capacity_Voltage);
	}
	else
	{
		lprintk(D_BATT, "%s: [Critical] Unexpected Battery gauge value!!!(%d) \n", __func__, capacity_index);
		batt_dev->Capacity_Voltage = 30; // In first time, search range is from 0 to 100 (whole range)
	}

	return NV_TRUE;
}

static NvBool capacity_without_charger_source(void)
{
	static NvU16 capacity_table[] = BAT_CV_TABLE;
	static NvU8 capacity_index = 100, range_index = 0;
	static NvU32 temp_vol;

	//temp_vol = (NvU32)((batt_dev->batt_vol + 2*batt_dev->vol_for_capacity)/3);
	temp_vol = batt_dev->batt_vol;
	if (batt_dev->Capacity_first_time) // default setting
	{
		for (capacity_index = 0; capacity_index <= 100; capacity_index++)
		{
			if ((capacity_table[capacity_index] >= temp_vol) || (capacity_index == 100))
				goto set_BatteryLifePercent_wo;
		}
		capacity_index = 100;
		goto set_BatteryLifePercent_wo;
	}
	else // After first time, search range is start from slightly reverse direction of previous gauge, and go forward to decrease direction (because "Discharging state" now) 
	{
		range_index = batt_dev->BatteryLifePercent;
		for (; range_index <= 100; range_index++ )
		{
			if ((capacity_table[range_index] >= temp_vol) || (range_index == 100))
				break;
		}
		capacity_index = range_index;

		for (; capacity_index >= 0 ; capacity_index-- )
		{
			if ((capacity_table[capacity_index] <= temp_vol) || (capacity_index == 0))
			{
				goto set_BatteryLifePercent_wo;
			}
		}
		capacity_index = 0;
		goto set_BatteryLifePercent_wo;
	}

	return NV_FALSE;

set_BatteryLifePercent_wo:
	if ((capacity_index >= 0) || (capacity_index <= 100))
	{
		batt_dev->Capacity_Voltage = capacity_index;
		//LDB("[bat_cv] CV_wo gauge Battery(%d)", batt_dev->Capacity_Voltage);
	}
	else
	{
		lprintk(D_BATT, "%s: [Critical] Unexpected Battery gauge value in OneTime CV w/o Charger!!!(%d) \n", __func__, capacity_index);
		batt_dev->Capacity_Voltage = 30; // default setting
	}

	return NV_TRUE;
}

static NvBool determine_capacity_for_demo(void)
{
	NvRmPmuAcLineStatus AcStatus = NvRmPmuAcLine_Offline;
	NvU8 BatStatus = 0;
	NvRmPmuBatteryData BatData = {0};
	NvU32 Average_Vol = 0, Average_count, Valid_count = 0;

	if (batt_dev->Capacity_first_time)
	{
		for (Average_count = 0; Average_count < 7; Average_count++)
		{
			// Warning!! NvRmPmuUpdateBatteryInfo operate correctly, but return value is false.. why??
			// Never use "if (!func)" for NvRmPmuUpdatebatteryInfo.....  , 2010-10-14
			if (NvRmPmuUpdateBatteryInfo(s_hRmGlobal, &AcStatus, &BatStatus, &BatData))
			{
				if ((2500 < BatData.batteryVoltage) && (BatData.batteryVoltage < 4500))
				{
					Average_Vol += BatData.batteryVoltage; // Already in [mV] scale..
					Valid_count++;
				}
			}
			else
			{
				LDB("[Critical]: NvRmPmuUpdateBatteryInfo is failed!!!");
			}
		}
		batt_dev->batt_vol = (NvU32)(Average_Vol /Valid_count); // Booting time, current consumption is high, so determine capacity with bias...
		batt_dev->vol_for_capacity = batt_dev->batt_vol + 200;
	}
/*
	switch (get_charging_ic_status())
	{
		case CHG_IC_DEFAULT_MODE:
		case CHG_IC_USB_LO_MODE:
			//LDB("[bat_cv]: Charging Source: USB");
			if (!capacity_with_charger_source_USB())
			{
				LDB("[Critical]: CV_USB(%d)", batt_dev->Capacity_Voltage);
				return NV_FALSE;
			}
			break;

		case CHG_IC_TA_MODE:
		case CHG_IC_FACTORY_MODE:
			//LDB("[bat_cv]: Charging Source: TA");
			if (!capacity_with_charger_source_TA())
			{
				LDB("[Critical]: CV_TA(%d)", batt_dev->Capacity_Voltage);
				return NV_FALSE;
			}
			break;

		case CHG_IC_DEACTIVE_MODE:
			//LDB("[bat_cv]: Charging Source: NONE");
			if (!capacity_without_charger_source())
			{
				LDB("[Critical]: CV_BATT(%d)", batt_dev->Capacity_Voltage);
				return NV_FALSE;
			}
			break;

		default:
			LDB("[Critical] Unknown state - CV");
			if (!capacity_without_charger_source())
			{
				LDB("[Critical]: CV_Unknown(%d)", batt_dev->Capacity_Voltage);
				return NV_FALSE;
			}
			break;
	}
*/
	star_capacity_from_voltage_via_calculate();
	valid_capacity_gauge();

	return NV_TRUE;
}
#endif // USE_ONETIME_VOLTAGE_CAPACITY
//20100824, , get capacity using battery voltage for demo [END]

//20100915, , For AT command [START]
static ssize_t star_at_charge_show_property(
		struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	size_t count=0;

	if ( at_charge_index == NV_TRUE )
		count = sprintf(buf, "1\n");
	else if ( at_charge_index == NV_FALSE )
		count = sprintf(buf, "0\n");
	else
		count = sprintf(buf, "0\n");

	return count;
}

static ssize_t star_at_charge_store_property(
		struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t count)
{
	static NvU32 value = 0;

	LDC("[at_factory]");
	value = (NvU32)(simple_strtoul(buf, NULL, 0));

	if ( value == 1 )
	{
		at_charge_index = NV_TRUE;
		batt_dev->repeat_index = NV_FALSE;

		star_battery_infomation_update();
		star_battery_data_onetime_update(Update_Battery_Data);

		charging_ic_active(CHG_IC_TA_MODE); // star_battery_date_onetime_update() for TA/USB is executed in active / deactive func.

	}

	return count;
}

static struct device_attribute star_at_charge_attr = {
	.attr = { .name = "at_charge", .mode = S_IRUGO | S_IWUGO,
			  .owner = THIS_MODULE },
	.show = star_at_charge_show_property,
	.store = star_at_charge_store_property,
};

static ssize_t star_at_chcomp_show_property(
		struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	size_t count=0;

	star_battery_infomation_update();
	star_battery_infomation_update();
	star_battery_infomation_update();
	star_battery_infomation_update();

	if (( batt_dev->batt_vol >= 4100 ) || ( at_charge_comp == NV_TRUE ))
	{
		batt_dev->BatteryLifePercent = 95;
		star_battery_data_onetime_update(Update_Battery_Data);
		batt_dev->BatteryLifePercent = 100;
		star_battery_data_onetime_update(Update_Battery_Data);

		count = sprintf(buf, "1\n");
	}
	else
	{
		batt_dev->BatteryLifePercent = 95;
		star_battery_data_onetime_update(Update_Battery_Data);
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
	static NvU32 value = 0;

	LDC("[at_factory]");
	value = (NvU32)(simple_strtoul(buf, NULL, 0));

	if ( value == 0 )
	{
		batt_dev->BatteryLifePercent = 95;
		at_charge_comp = NV_FALSE;
		star_battery_data_onetime_update(Update_Battery_Data);
	}
	else if ( value == 1 )
	{
		batt_dev->BatteryLifePercent = 100;
		at_charge_comp = NV_TRUE;
		star_battery_data_onetime_update(Update_Battery_Data);
	}

	return count;
}

static struct device_attribute star_at_chcomp_attr = {
	.attr = { .name = "at_chcomp", .mode = S_IRUGO | S_IWUGO,
			  .owner = THIS_MODULE },
	.show = star_at_chcomp_show_property,
	.store = star_at_chcomp_store_property,
};
//20100915, , For AT command [END]

//20101029, , For Overheat charge control [START]
static void charger_control_with_battery_temp(void)
{
	if (batt_dev->present == NV_TRUE)
	{
		star_batt_dev->batt_health = batt_dev->batt_health;

		switch (batt_dev->batt_health)
		{
			case POWER_SUPPLY_HEALTH_GOOD:
			{
				if (batt_dev->batt_temp >= 550)
				{
					// Deactivate Charger : Battery Critical Overheat
					batt_dev->batt_health = POWER_SUPPLY_HEALTH_CRITICAL_OVERHEAT;
					if ( charging_ic->status != CHG_IC_DEACTIVE_MODE )
					{
						LDC("[OVERHEAT]: Deactive Charger");
						batt_dev->charger_setting_chcomp = charging_ic->status;
						charging_ic_deactive_for_rechrge();
						batt_dev->charger_state_machine = CHARGER_STATE_SHUTDOWN;
					}
				}
				else if ((batt_dev->batt_temp >= 450) && (batt_dev->batt_temp < 550))
				{
					// Change Charger Setting : Battery Overheat, USB_500 mode
					batt_dev->batt_health = POWER_SUPPLY_HEALTH_OVERHEAT;
					if ( charging_ic->status != CHG_IC_DEACTIVE_MODE )
					{
						LDC("[OVERHEAT]: Change Charger setting to USB mode");
						if (batt_dev->charger_state_machine == CHARGER_STATE_FULLBATTERY)
						{
							LDB("[BATFULL] Recharge condition, BUT charger_state_machine is CHARGER_STATE_FULLBATTERY : Do nothing");
						}
						else
						{
							batt_dev->charger_setting_chcomp = charging_ic->status;
							batt_dev->charger_state_machine = CHARGER_STATE_CHARGE;
							charging_ic_active_for_recharge(CHG_IC_DEFAULT_MODE);
						}
					}
				}
				else if (batt_dev->batt_temp <= (-100))
				{
					// Deactivate Charger : Battery Cold
					batt_dev->batt_health = POWER_SUPPLY_HEALTH_COLD;
					if ( charging_ic->status != CHG_IC_DEACTIVE_MODE )
					{
						LDC("[BATTCOLD]: Deactive Charger");
						batt_dev->charger_setting_chcomp = charging_ic->status;
						charging_ic_deactive_for_rechrge();
						batt_dev->charger_state_machine = CHARGER_STATE_SHUTDOWN;
					}
				}
				else
					batt_dev->batt_health = POWER_SUPPLY_HEALTH_GOOD;
			}
			break;

			case POWER_SUPPLY_HEALTH_OVERHEAT:
			{
				if (batt_dev->batt_temp >= 550)
				{
					// Deactivate Charger : Battery Critical Overheat
					batt_dev->batt_health = POWER_SUPPLY_HEALTH_CRITICAL_OVERHEAT;
					if ( charging_ic->status != CHG_IC_DEACTIVE_MODE )
					{
						LDC("[OVERHEAT]: Deactive Charger");
						batt_dev->charger_setting_chcomp = charging_ic->status;
						charging_ic_deactive_for_rechrge();
						batt_dev->charger_state_machine = CHARGER_STATE_SHUTDOWN;
					}
				}
				else if (batt_dev->batt_temp <= 420)
				{
					if ( charging_ic->status != CHG_IC_DEACTIVE_MODE )
					{
						// Reactivate Charger : Battery Normal again
						LDB("[OVERHEAT]: Re-enable Charger to Original setting");
						batt_dev->batt_health = POWER_SUPPLY_HEALTH_GOOD;
						if (batt_dev->charger_state_machine == CHARGER_STATE_FULLBATTERY)
						{
							LDB("[BATFULL] Recharge condition, BUT charger_state_machine is CHARGER_STATE_FULLBATTERY : Do nothing");
						}
						else
						{
							batt_dev->charger_state_machine = CHARGER_STATE_CHARGE;
							charging_ic_active_for_recharge(batt_dev->charger_setting_chcomp);
						}
					}
				}
				else
					batt_dev->batt_health = POWER_SUPPLY_HEALTH_OVERHEAT;
			}
			break;

			case POWER_SUPPLY_HEALTH_CRITICAL_OVERHEAT:
			{
				if (batt_dev->batt_temp <= 520)
				{
					if ( charging_ic->status != CHG_IC_DEACTIVE_MODE )
					{
						// Reactivate Charger : Battery USB mode again
						LDB("[OVERHEAT]: Re-enable Charger to USB mode setting");
						batt_dev->batt_health = POWER_SUPPLY_HEALTH_OVERHEAT;
						if (batt_dev->charger_state_machine == CHARGER_STATE_FULLBATTERY)
						{
							LDB("[BATFULL] Recharge condition, BUT charger_state_machine is CHARGER_STATE_FULLBATTERY : Do nothing");
						}
						else
						{
							batt_dev->charger_state_machine = CHARGER_STATE_CHARGE;
							charging_ic_active_for_recharge(CHG_IC_DEFAULT_MODE);
						}
					}
				}
				else
					batt_dev->batt_health = POWER_SUPPLY_HEALTH_CRITICAL_OVERHEAT;
			}
			break;

			case POWER_SUPPLY_HEALTH_COLD:
			{
				if (batt_dev->batt_temp >= (-50))
				{
					if ( charging_ic->status != CHG_IC_DEACTIVE_MODE )
					{
						// Reactivate Charger : Battery Normal again
						LDB("[BATTCOLD]: Re-enable Charger");
						batt_dev->batt_health = POWER_SUPPLY_HEALTH_GOOD;
						if (batt_dev->charger_state_machine == CHARGER_STATE_FULLBATTERY)
						{
							LDB("[BATFULL] Recharge condition, BUT charger_state_machine is CHARGER_STATE_FULLBATTERY : Do nothing");
						}
						else
						{
							batt_dev->charger_state_machine = CHARGER_STATE_CHARGE;
							charging_ic_active_for_recharge(batt_dev->charger_setting_chcomp);
						}
					}
				}
				else
					batt_dev->batt_health = POWER_SUPPLY_HEALTH_COLD;
			}
			break;

			default:
				batt_dev->batt_health = POWER_SUPPLY_HEALTH_UNKNOWN;
				LDB("[Critical]: Charger control with Battery Temperature has Exception!!!");
				break;
		} // switch end
	}// if end
}// function end
//20101029, , For Overheat charge control [END]

static void star_battery_data_poll_period_change(void)
{
	NvU32 temp_period_data, temp_period_wake;
	NvU32 vol_period_data, vol_period_wake;
	NvU32 gauge_period_data, gauge_period_wake;
	NvU32 critical_period_data, critical_period_wake;

	// Dynamic change of battery data polling time.. If difference between previous battery gauge and current gauge is huge, reduce polling time
	// Critical Status
	if ((batt_dev->BatteryLifePercent <= 5) || (batt_dev->ACLineStatus == NV_TRUE)
		|| (batt_dev->batt_health != POWER_SUPPLY_HEALTH_GOOD) || (batt_dev->batt_vol <= 3500)
		|| (batt_dev->charger_state_machine == CHARGER_STATE_FULLBATTERY))
	{
		critical_period_data = CRITICAL_BAT_CHECK_PERIOD*HZ;
		critical_period_wake = CRITICAL_BAT_CHECK_PERIOD+10;
	}
	else
	{
		critical_period_data = NVBATTERY_POLLING_INTERVAL*HZ;
		critical_period_wake = SLEEP_BAT_CHECK_PERIOD;
	}

	// Voltage Status
	vol_period_data = NVBATTERY_POLLING_INTERVAL*HZ;
	vol_period_wake = SLEEP_BAT_CHECK_PERIOD;

	// Temperatue Status
	if ((STAR_BAT_MAX(star_batt_dev->batt_temp, batt_dev->batt_temp)-STAR_BAT_MIN(star_batt_dev->batt_temp, batt_dev->batt_temp)) < 100) // 10 degree
	{
		temp_period_data = NVBATTERY_POLLING_INTERVAL*HZ;
		temp_period_wake = SLEEP_BAT_CHECK_PERIOD;
	}
	else
	{
		temp_period_data = CRITICAL_BAT_CHECK_PERIOD*2*HZ;
		temp_period_wake = CRITICAL_BAT_CHECK_PERIOD*2;
	}

	// Gauge Status
	gauge_period_data = NVBATTERY_POLLING_INTERVAL*HZ;
	gauge_period_wake = SLEEP_BAT_CHECK_PERIOD;

	batt_dev->battery_poll_interval = STAR_BAT_MIN(STAR_BAT_MIN(critical_period_data, vol_period_data), STAR_BAT_MIN(temp_period_data, gauge_period_data));
	batt_dev->sleep_bat_check_period = STAR_BAT_MIN(STAR_BAT_MIN(critical_period_wake, vol_period_wake), STAR_BAT_MIN(temp_period_wake, gauge_period_wake));

	LDB("[bat_sysfs] polling_interval: poll/wake[%d, %d] : Cri[%d, %d]/Vol[%d, %d]/Tem[%d, %d]/Gau[%d, %d]"
		, batt_dev->battery_poll_interval/HZ, batt_dev->sleep_bat_check_period
		, critical_period_data/HZ, critical_period_wake, vol_period_data/HZ, vol_period_wake
		, temp_period_data/HZ, temp_period_wake, gauge_period_data/HZ, gauge_period_wake
		);
}

static void star_gauge_follower_func(void)
{
	NvU32 gauge_follower_time;
	NvU32 gauge_follower_time_mask;

	//LDC("");
	star_batt_dev->BatteryLifePercent = batt_dev->BatteryLifePercent;

		if (batt_dev->ACLineStatus == NV_TRUE)
		{
			if (batt_dev->BatteryGauge > batt_dev->BatteryLifePercent)
				batt_dev->BatteryLifePercent += 1;
			else if (batt_dev->BatteryGauge < batt_dev->BatteryLifePercent)
				batt_dev->BatteryLifePercent -= 1;
		}
		else if (batt_dev->ACLineStatus == NV_FALSE)
		{
			//if (batt_dev->BatteryGauge > batt_dev->BatteryLifePercent)
			//	batt_dev->BatteryLifePercent += 1;
			//else 
			if (batt_dev->BatteryGauge < batt_dev->BatteryLifePercent)
				batt_dev->BatteryLifePercent -= 1;
		}

		if (batt_dev->BatteryLifePercent < 0) batt_dev->BatteryLifePercent = 0;
		if (batt_dev->BatteryLifePercent > 100) batt_dev->BatteryLifePercent = 100;
}

static void tegra_battery_status_poll_work(struct work_struct *work)
{
	//LDB("");
//20100926, , Communication with BatteryService for AT Command [START]
	star_battery_infomation_update();
	mod_timer(&(batt_dev->charger_state_read_timer), jiffies + msecs_to_jiffies(0));
	charger_control_with_battery_temp();

//20100824, , get capacity using battery voltage for demo [START]
#if defined (USE_ONETIME_VOLTAGE_CAPACITY)
	if (batt_dev->repeat_index == NV_TRUE)
	{
		if (!determine_capacity_for_demo())  // 100823 
		{
			batt_dev->BatteryLifePercent = 103;
			LDB("[Critical]: capacity for demo is failed!!!!");
		}
	}
#endif // USE_ONETIME_VOLTAGE_CAPACITY
//20100824, , get capacity using battery voltage for demo [END]
//20100924, , For Battery recharge [START]
	//if (( batt_dev->batt_vol < 4130 ) && (batt_dev->charger_state_machine == CHARGER_STATE_FULLBATTERY))
	if ((( batt_dev->CBC_Value <= 97 ) || (batt_dev->batt_vol < 4135)) && (batt_dev->charger_state_machine == CHARGER_STATE_FULLBATTERY))
	{
		LDB("[FULLBAT]: Re-enable Charger");
		batt_dev->charger_state_machine = CHARGER_STATE_RECHARGE;
		switch (batt_dev->batt_health)
		{
			case POWER_SUPPLY_HEALTH_OVERHEAT:
				LDB("[Warning]: Recharge condition, BUT batt_health is POWER_SUPPLY_HEALTH_OVERHEAT : 400mA current setting");
				charging_ic_active_for_recharge(CHG_IC_DEFAULT_MODE);
				break;

			case POWER_SUPPLY_HEALTH_CRITICAL_OVERHEAT:
				LDB("[Warning]: Recharge condition, BUT batt_health is POWER_SUPPLY_HEALTH_CRITICAL_OVERHEAT : Do nothing");
				break;

			case POWER_SUPPLY_HEALTH_COLD:
				LDB("[Warning]: Recharge condition, BUT batt_health is POWER_SUPPLY_HEALTH_COLD : Do nothing");
				break;

			case POWER_SUPPLY_HEALTH_GOOD:
			default:
				charging_ic_active_for_recharge(batt_dev->charger_setting_chcomp);
				break;
		}
	}
//20100924, , For Battery recharge [END]
	if ((batt_dev->BatteryGauge_on == NV_TRUE) && (batt_dev->BatteryLifePercent != batt_dev->BatteryGauge))
		star_gauge_follower_func();

	star_battery_data_poll_period_change();
	if ( ((batt_dev->BatteryLifePercent == 1) || (batt_dev->BatteryLifePercent == 0) || (batt_dev->batt_vol <= 3400))
		|| (star_batt_dev->BatteryLifePercent != batt_dev->BatteryLifePercent)
		|| ((batt_dev->batt_health != POWER_SUPPLY_HEALTH_GOOD) || (star_batt_dev->batt_health != batt_dev->batt_health))
		|| (star_batt_dev->charger_state_machine != batt_dev->charger_state_machine)
		|| (star_batt_dev->present != batt_dev->present)
		|| ((STAR_BAT_MAX(star_batt_dev->batt_vol, batt_dev->batt_vol) - STAR_BAT_MIN(star_batt_dev->batt_vol, batt_dev->batt_vol)) >= 100)
		|| ((STAR_BAT_MAX(star_batt_dev->batt_temp, batt_dev->batt_temp) - STAR_BAT_MIN(star_batt_dev->batt_temp, batt_dev->batt_temp)) >= 50)
		|| ((star_batt_dev->ACLineStatus != batt_dev->ACLineStatus) ||(star_batt_dev->charging_source != batt_dev->charging_source) || (star_batt_dev->charging_enabled != batt_dev->charging_enabled)))
		{
			star_battery_data_onetime_update(Update_Battery_Data);
		}
	//star_debug_show_battery_status();

	queue_delayed_work(batt_dev->battery_workqueue, &batt_dev->battery_status_poll_work, batt_dev->battery_poll_interval);
}

//20101117, , AT command request function [START]
static void star_cp_at_comm_request(void)
{
#if defined (STAR_BATTERY_AT_COMMAND)
	if (batt_dev->at_comm_ready == NV_TRUE)
	{
		//LDB("[at_comm] Call Update_AT_Command:::");;
		star_battery_data_onetime_update(Update_AT_Command);
	}
#else // STAR_BATTERY_AT_COMMAND
	if ((!at_charge_index) && (AtComm->flag_at_comm_ready == NV_TRUE))
	{
		if (AtComm->flag_at_comm_want == AT_COMM_CBC)
		{
			mutex_lock(&AtComm->mutex_lock);
			switch_set_state(&AtComm->sdev, (switch_get_state(&AtComm->sdev) | AT_COMM_CBC));
			mutex_unlock(&AtComm->mutex_lock);
			AtComm->flag_at_comm_ready = NV_FALSE;
			//LDB("[at_comm]: AT Command(%d)", switch_get_state(&AtComm->sdev));
		}
	}
#endif // STAR_BATTERY_AT_COMMAND
}

static void star_cbc_gauge_request_timer_func(unsigned long unused)
{
	if (batt_dev->at_comm_ready == NV_FALSE)
	{
		if (batt_dev->cbc_clear_state != Clear_No_Response_1)
	    {
		    //LDB("[CBC]: Clear NO Response STATE");
		    batt_dev->cbc_clear_state += 1;
		    batt_dev->at_comm_want = AT_COMM_NO;
		    batt_dev->at_comm_ready = NV_TRUE;

		    star_cp_at_comm_request();

			mod_timer(&(batt_dev->battery_gauge_timer), jiffies + msecs_to_jiffies(3000));
		}
		else
			batt_dev->at_comm_ready = NV_TRUE;
	}
	else if (batt_dev->BatteryGauge_on == NV_FALSE)
	{
		if (batt_dev->RIL_ready == NV_TRUE)
	    {
		    LDB("[CBC]: Request FUELRST with clear(%d)", batt_dev->cbc_clear_state);
			batt_dev->at_comm_want = AT_COMM_NO;
		    batt_dev->at_comm_ready = NV_TRUE;
		    star_cp_at_comm_request();
		    batt_dev->cbc_clear_state = Clear_No_Response_0;
	    }
		else
			mod_timer(&(batt_dev->battery_gauge_timer), jiffies + msecs_to_jiffies(3000));
	}
}
//20101117, , AT command request function [END]

//20100917, , Battery ID polling function [START]
static void star_battery_id_check(void)
{
	NvRmPmuAcLineStatus AcStatus = NvRmPmuAcLine_Offline;
	NvU8 BatStatus = 0;
	NvRmPmuBatteryData BatData = {0};

	if (NvRmPmuUpdateBatteryInfo(s_hRmGlobal, &AcStatus, &BatStatus, &BatData) && NvRmPmuGetAcLineStatus(s_hRmGlobal, &AcStatus))
	{
		if ( at_charge_index == NV_FALSE )
		{
			//LDB("[bat_id] UpdateBatteryInfo(%d, %d)", AcStatus, NvRmPmuAcLine_Online );
			if (( 2330 > BatData.batteryTemperature ) || ( 3630 < BatData.batteryTemperature )) // Temperature range -40 ~ 90 [C]: Battery present
			{
				//LDB("[bat_id] No Battery!!!(%d, %d)", AcStatus, NvRmPmuAcLine_Online );
				if ( ( AcStatus == NvRmPmuAcLine_Online ) && ( at_boot_state == 0 ) )
				{
					//LDB("[bat_id] ACStatus == Online");
				        // No Battery & Ac_Online, system shut-down forcibly!!!
					    batt_dev->present = NV_FALSE;
					    star_battery_data_onetime_update(Update_Battery_Data);
				}
			}
		}
		else // ( at_charge_index == NV_TRUE )
		{
			if (( AcStatus == NvRmPmuAcLine_Online ) && (charging_ic->status == CHG_IC_DEACTIVE_MODE) && (at_charge_on == NV_FALSE))
			{
				at_charge_on = NV_TRUE;
				charging_ic_active(CHG_IC_TA_MODE);
			}
		}
	}
	else
	{
		LDB("[Critical]: NvRmPmuUpdateBatteryInfo and NvRmPmuGetAcLineStatus failed!!!");
	}
}

static void star_battery_id_poll_work(struct work_struct *work)
{
	//LDB("[bat_id]");

	if ((( at_charge_index == NV_FALSE ) && ( charging_ic->status != CHG_IC_DEACTIVE_MODE ))
		|| (( at_charge_index == NV_TRUE ) && ( charging_ic->status == CHG_IC_DEACTIVE_MODE )))
	{
		star_battery_id_check();
	}

	queue_delayed_work(batt_dev->battery_workqueue, &batt_dev->battery_id_poll_work, HZ*2);
}
//20100917, , Battery ID polling function [END]

static void star_battery_data_onetime_update(NvU8 update_option)
{
	//LDB("[bat_poll] OneTime_Update(%d)", update_option);

	switch ((OneTime_Update)update_option)
	{
		case Update_Battery_Data:
			power_supply_changed(&tegra_supplies[NvCharger_Type_Battery]);
			//LDB("[bat_poll] OneTime_Update(Update_Battery_Data:%d)", update_option);
			break;

		case Update_USB_Data:
			power_supply_changed(&tegra_supplies[NvCharger_Type_USB]);
			//LDB("[bat_poll]  OneTime_Update(Update_USB_Data:%d)", update_option);
			break;

		case Update_AC_Data:
			power_supply_changed(&tegra_supplies[NvCharger_Type_AC]);
			//LDB("[bat_poll]  OneTime_Update(Update_AC_Data:%d)", update_option);
			break;

		case Update_Power_Data:
			power_supply_changed(&tegra_supplies[NvCharger_Type_USB]);
			power_supply_changed(&tegra_supplies[NvCharger_Type_AC]);
			//LDB("[bat_poll]  OneTime_Update(Update_Power_Data:%d)", update_option);
			break;

#if defined (STAR_BATTERY_AT_COMMAND)
		case Update_AT_Command:
			power_supply_changed(&tegra_supplies[NvCharger_Type_AT_Command]);
			//LDB("[bat_poll]  OneTime_Update(Update_AT_Command:%d)", update_option);
			break;
#endif // STAR_BATTERY_AT_COMMAND

		case Update_ALL:
			power_supply_changed(&tegra_supplies[NvCharger_Type_Battery]);
			power_supply_changed(&tegra_supplies[NvCharger_Type_USB]);
			power_supply_changed(&tegra_supplies[NvCharger_Type_AC]);
			//LDB("[bat_poll]  OneTime_Update(Update_ALL:%d)", update_option);
			break;

		default:
			LDB("[Critical] why this function is called???");
			break;
	}
	star_debug_show_battery_status();
}
	
/*
static void star_battery_charge_done_work(struct work_struct *work)
{
	LDC("[FULLBAT] Do nothing!!!");
}
*/
static void star_read_charger_state_gpio(NvU8 *chg_state)
{
	NvU32 Status_Value = 0xff, PGB_Value = 0xff;

	NvOdmGpioGetState( charging_ic->hGpio, charging_ic->hStatusPin, &Status_Value );
	NvOdmGpioGetState( charging_ic->hGpio, charging_ic->hPGBPin, &PGB_Value );

	if (( Status_Value == 1 )&& ( PGB_Value == 1 ))
		*chg_state = CHGSB_PGB_OFF_OFF;
	else if (( Status_Value == 0 )&& ( PGB_Value == 0 ))
		*chg_state = CHGSB_PGB_ON_ON;
	else if (( Status_Value == 1 )&& ( PGB_Value == 0 ))
		*chg_state = CHGSB_PGB_OFF_ON;
	else if (( Status_Value == 0 )&& ( PGB_Value == 1 ))
	{
		*chg_state = CHGSB_PGB_ON_OFF;
		LDB("[Critical] Unknown Charger state GPIO!!!(CHGSB_PGB_ON_OFF) why this is occured???");
	}
}

static void star_determine_charger_state(NvU8 *chg_state_new, NvU8 *chg_state_now)
{
	switch ((charge_ic_state_machine)batt_dev->charger_state_machine) // chg_state_past
	{
		case CHARGER_STATE_SHUTDOWN: // CHGSB_PGB_OFF_OFF
			switch ((charge_ic_status_transition)*chg_state_new)
			{
				case CHGSB_PGB_ON_ON:
					LDC("[chg_state] SHUTDOWN/ON_ON/CHARGE");
					*chg_state_now = CHARGER_STATE_CHARGE;
					break;
				case CHGSB_PGB_OFF_ON:
					LDC("[chg_state] SHUTDOWN/OFF_ON/STANDBY");
					*chg_state_now = CHARGER_STATE_STANDBY;
					break;
				case CHGSB_PGB_OFF_OFF:
					LDC("[chg_state] SHUTDOWN/OFF_OFF/SHUTDOWN");
					*chg_state_now = batt_dev->charger_state_machine;
					break;
				default:
					LDC("[Critical] Unknown Charger state GPIO!!!(CHGSB_PGB_ON_OFF)");
					break;
			}
			break;

		case CHARGER_STATE_STANDBY: // CHGSB_PGB_OFF_ON
			switch ((charge_ic_status_transition)*chg_state_new)
			{
				case CHGSB_PGB_ON_ON:
					LDC("[chg_state] STANDBY/ON_ON/CHARGE");
					*chg_state_now = CHARGER_STATE_CHARGE;
					break;
				case CHGSB_PGB_OFF_OFF:
					LDC("[chg_state] STANDBY/OFF_OFF/SHUTDOWN");
					*chg_state_now = CHARGER_STATE_SHUTDOWN;
					break;
				case CHGSB_PGB_OFF_ON:
					LDC("[chg_state] STANDBY/OFF_ON/STANDBY :: Why This occur? :: Check charger setting");
					*chg_state_now = batt_dev->charger_state_machine;
					break;
				default:
					LDC("[Critical] Unknown Charger state GPIO!!!(CHGSB_PGB_ON_OFF)");
					break;
			}
			break;

		case CHARGER_STATE_CHARGE: // CHGSB_PGB_ON_ON
		case CHARGER_STATE_RECHARGE: // CHGSB_PGB_ON_ON
			switch ((charge_ic_status_transition)*chg_state_new)
			{
				case CHGSB_PGB_OFF_OFF:
					LDC("[chg_state] CHARGE/OFF_OFF/SHUTDOWN");
					*chg_state_now = CHARGER_STATE_SHUTDOWN;
					break;
				case CHGSB_PGB_OFF_ON:
					LDC("[chg_state] CHARGE/OFF_ON/FULLBATTERY");
					*chg_state_now = CHARGER_STATE_FULLBATTERY;
					break;
				case CHGSB_PGB_ON_ON:
					LDC("[chg_state] CHARGE/ON_ON/CHARGE");
					*chg_state_now = batt_dev->charger_state_machine;
					break;
				default:
					LDC("[Critical] Unknown Charger state GPIO!!!(CHGSB_PGB_ON_OFF)");
					break;
			}
			break;

		case CHARGER_STATE_FULLBATTERY: // CHGSB_PGB_OFF_ON
			switch ((charge_ic_status_transition)*chg_state_new)
			{
				case CHGSB_PGB_ON_ON:
					LDC("[chg_state] FULLBATTERY/ON_ON/RECHARGE");
					*chg_state_now = CHARGER_STATE_RECHARGE;
					break;
				case CHGSB_PGB_OFF_OFF:
					LDC("[chg_state] FULLBATTERY/OFF_OFF/SHUTDOWN");
					*chg_state_now = CHARGER_STATE_SHUTDOWN;
					break;
				case CHGSB_PGB_OFF_ON:
					LDC("[chg_state] FULLBATTERY/OFF_ON/FULLBATTERY");
					*chg_state_now = batt_dev->charger_state_machine;
					break;
				default:
					LDC("[Critical] Unknown Charger state GPIO!!!(CHGSB_PGB_ON_OFF)");
					break;
			}
			break;

		default:
			LDC("[Critical] Unknown Charger State!!!!!");
			*chg_state_now = CHARGER_STATE_SHUTDOWN;
			break;
	}
}

static void tegra_battery_gpio_read_timer_func(unsigned long unused)
{
	NvU8 charger_ic_state_new = 0;
	NvU8 charger_ic_state_now = 0;

	star_read_charger_state_gpio(&charger_ic_state_new);
	star_determine_charger_state(&charger_ic_state_new, &charger_ic_state_now);
	LDB("[FULLBAT]: charger_is_state old(%d) & new(%d) & now(%d)", batt_dev->charger_state_machine, charger_ic_state_new, charger_ic_state_now);

	if ( at_charge_index == NV_FALSE )
	{
		switch ((charge_ic_state_machine)charger_ic_state_now)
		{
			case CHARGER_STATE_FULLBATTERY:
				if (batt_dev->charger_state_machine != CHARGER_STATE_FULLBATTERY)
				{
					//if ((batt_dev->BatteryLifePercent >= 99) && (batt_dev->BatteryLifePercent < 100) && (batt_dev->ACLineStatus == NV_TRUE))
						//batt_dev->BatteryLifePercent += 1;
					// Disable Charger for protecting Battery
					if ((batt_dev->BatteryLifePercent == 100) && (batt_dev->CBC_Value >= 99) && (batt_dev->batt_vol > 4165))
					{
						//star_battery_data_onetime_update(Update_Battery_Data);
						if ( charging_ic->status != CHG_IC_DEACTIVE_MODE )
						{
							//LDC("[FULLBAT]: Deactive Charger with state_now(%d)", charger_ic_state_now);
							batt_dev->charger_setting_chcomp = charging_ic->status;
							charging_ic_deactive_for_rechrge();
						}
					}
					else
						charger_ic_state_now = CHARGER_STATE_CHARGE;
				}
				break;
/*
// Infact, recharge start with voltage check -> charger setting -> charger state machine change -> this func..
// So, If you excute charging_ic_active() then you excute charger active function twice!!!
			case CHARGER_STATE_RECHARGE:
				// Re-setting Charger for recharging Battery
				if ( charging_ic->status == CHG_IC_DEACTIVE_MODE )
				{
					LDC("[FULLBAT]: Deactive Charger with state_now(%d)", charger_ic_state_now);
					charging_ic_active_for_recharge(batt_dev->charger_setting_chcomp);
				}
				break;
*/
			default:
				break;
		}
	}

	star_batt_dev->charger_state_machine = batt_dev->charger_state_machine;
	batt_dev->charger_state_machine = charger_ic_state_now;
	LDB("[FULLBAT]: State change Result : machine(%d) & now(%d)", batt_dev->charger_state_machine, charger_ic_state_now);
}
// BatteryTest

static void star_full_battery_interrupt_handler(void* arg)
{
	// Some delay is required in GPIO read for accuracy!!
	mod_timer(&(batt_dev->charger_state_read_timer), jiffies + msecs_to_jiffies(25));

	NvOdmGpioInterruptDone(charging_ic->hCHGDone_int);
}

static void star_initial_charger_state(void)
{
	NvU8 charger_state_init;

	star_read_charger_state_gpio(&charger_state_init);
	switch ((charge_ic_status_transition)charger_state_init) // Prepare for initial state of charger state machine
	{
		case CHGSB_PGB_OFF_OFF:
			batt_dev->charger_state_machine = CHARGER_STATE_SHUTDOWN;
			charging_ic->status = CHG_IC_DEACTIVE_MODE;
			charging_ic->status_past = CHG_IC_DEACTIVE_MODE;
			batt_dev->ACLineStatus = NV_FALSE;
			batt_dev->charging_source = NvCharger_Type_Battery;
			batt_dev->charging_enabled = NvCharge_Control_Charging_Disable;
			LDB("[FULLBAT]: charger_state_init(OFF_OFF: CHARGER_STATE_SHUTDOWN)");
			break;

		case CHGSB_PGB_ON_ON:
			batt_dev->charger_state_machine = CHARGER_STATE_CHARGE;
			charging_ic->status = CHG_IC_TA_MODE;
			charging_ic->status_past = CHG_IC_DEACTIVE_MODE;
			batt_dev->ACLineStatus = NV_TRUE;
			batt_dev->charging_source = NvCharger_Type_AC;
			batt_dev->charging_enabled = NvCharge_Control_Charging_Enable;
			LDB("[FULLBAT]: charger_state_init(ON_ON: CHARGER_STATE_CHARGE)");
			break;

		case CHGSB_PGB_OFF_ON:
			if ( at_boot_state == 0 )
			{
				batt_dev->charger_state_machine = CHARGER_STATE_CHARGE;
				charging_ic->status = CHG_IC_FACTORY_MODE;
				charging_ic->status_past = CHG_IC_DEACTIVE_MODE;
				batt_dev->ACLineStatus = NV_TRUE;
				batt_dev->charging_source = NvCharger_Type_AC;
				batt_dev->charging_enabled = NvCharge_Control_Charging_Enable;
				LDB("[FULLBAT]: charger_state_init(OFF_ON: CHARGER_STATE_CHARGE)");
			}
			else if ( at_boot_state == 1 )
			{
				batt_dev->charger_state_machine = CHARGER_STATE_STANDBY;
				charging_ic->status = CHG_IC_FACTORY_MODE;
				charging_ic->status_past = CHG_IC_FACTORY_MODE;
				batt_dev->ACLineStatus = NV_TRUE;
				batt_dev->charging_source = NvCharger_Type_AC;
				batt_dev->charging_enabled = NvCharge_Control_Charging_Enable;
				LDB("[FULLBAT]: charger_state_init(OFF_ON: CHARGER_STATE_STANDBY)");
			}
			break;

		default:
			batt_dev->charger_state_machine = CHARGER_STATE_SHUTDOWN;
			charging_ic->status = CHG_IC_DEACTIVE_MODE;
			charging_ic->status_past = CHG_IC_DEACTIVE_MODE;
			batt_dev->ACLineStatus = NV_FALSE;
			batt_dev->charging_source = NvCharger_Type_Battery;
			batt_dev->charging_enabled = NvCharge_Control_Charging_Disable;
			break;
	}
	LDB("[FULLBAT]: charger state machine(%d)", batt_dev->charger_state_machine);
}

static int tegra_battery_probe(struct platform_device *pdev)
{
	int i, rc;
//20100609, , Charger Code [START]
	NvU32 pin[3], port[3];
	const NvOdmPeripheralConnectivity *pConn = NULL;
//20100609, , Charger Code [END]
	LDB("[battery_probe] Start!!!");

	// Charger Dev
	charging_ic = kzalloc(sizeof(*charging_ic), GFP_KERNEL);
	if (!charging_ic) {
		return -ENOMEM;
	}
	memset(charging_ic, 0, sizeof(*charging_ic));

	// Battery Dev
	batt_dev = kzalloc(sizeof(*batt_dev), GFP_KERNEL);
	if (!batt_dev) {
		return -ENOMEM;
	}
	memset(batt_dev, 0, sizeof(*batt_dev));

	// Battery update Dev
	star_batt_dev = kzalloc(sizeof(*star_batt_dev), GFP_KERNEL);
	if (!star_batt_dev) {
		return -ENOMEM;
	}
	memset(star_batt_dev, 0, sizeof(*star_batt_dev));

#if defined(DEBUGFS_STAR_BATT_TEST_MODE)
	debug_batt_dev = kzalloc(sizeof(*debug_batt_dev), GFP_KERNEL);
	if (!debug_batt_dev) {
		return -ENOMEM;
	}
	memset(star_batt_dev, 0, sizeof(*star_batt_dev));
#endif // DEBUGFS_STAR_BATT_TEST_MODE

//20100926, , Communication with BatteryService for AT Command [START]
	batt_dev->sleep_bat_check_period_to_cp = 30;
#if defined (STAR_BATTERY_AT_COMMAND)
	batt_dev->at_comm_want = AT_COMM_NO;
	batt_dev->at_comm_ready = NV_FALSE;
#else // STAR_BATTERY_AT_COMMAND
	AtComm = kzalloc(sizeof(*AtComm), GFP_KERNEL);
	if (!AtComm)
		return -ENOMEM;
	memset(AtComm, 0, sizeof(*AtComm));

	AtComm->flag_at_comm_want = AT_COMM_CBC;
	AtComm->flag_at_comm_ready = NV_TRUE;
	mutex_init(&AtComm->mutex_lock);
	AtComm->sdev.name = "at_comm";
	AtComm->sdev.print_name = at_comm_print_name;

	rc = switch_dev_register(&AtComm->sdev);
	if (rc < 0)
		goto err_switch_dev_register;
#endif // STAR_BATTERY_AT_COMMAND
//20100926, , Communication with BatteryService for AT Command [END]

	setup_timer(&(batt_dev->charger_state_read_timer), tegra_battery_gpio_read_timer_func, 0);
	setup_timer(&(batt_dev->battery_gauge_timer), star_cbc_gauge_request_timer_func, 0);

//20100609, , Charger Code : GPIO Setting [START]
	pConn = NvOdmPeripheralGetGuid(NV_ODM_GUID('c','h','a','r','g','i','n','g'));

	for (i=0; i< pConn->NumAddress; i++)
	{
		switch (pConn->AddressList[i].Interface)
		{
			case NvOdmIoModule_Gpio:
				port[i] = pConn->AddressList[i].Instance;
				pin[i] = pConn->AddressList[i].Address;
				break;
			default:
				break;
		}
	}

	charging_ic->hGpio = NvOdmGpioOpen();
	if (!charging_ic->hGpio)
	{
		LDC("[Critical] NvOdmGpioOpen() Error");
		rc = -ENOSYS;
		goto err_open_gpio_fail;
	}

	charging_ic->hSetPin = NvOdmGpioAcquirePinHandle(charging_ic->hGpio, port[0], pin[0]);
	if (!charging_ic->hSetPin)
	{
		LDC("[Critical] NvodmGpioAcquirePinHandle(hSetPin) Error");
		rc = -ENOSYS;
		goto err_gpio_pin_acquire_fail;
	}

	charging_ic->hStatusPin = NvOdmGpioAcquirePinHandle(charging_ic->hGpio, port[1], pin[1]);
	if (!charging_ic->hStatusPin)
	{
		LDC("[Critical] NvodmGpioAcquirePinHandle(hStatusPin) Error");
		rc = -ENOSYS;
		goto err_gpio_pin_acquire_fail;
	}

	charging_ic->hPGBPin = NvOdmGpioAcquirePinHandle(charging_ic->hGpio, port[2], pin[2]);
	if (!charging_ic->hPGBPin)
	{
		LDC("[Critical] NvodmGpioAcquirePinHandle(hPGBPin) Error");
		rc = -ENOSYS;
		goto err_gpio_pin_acquire_fail;
	}

	NvOdmGpioSetState( charging_ic->hGpio, charging_ic->hSetPin, 0x0);
	NvOdmGpioConfig( charging_ic->hGpio, charging_ic->hSetPin, NvOdmGpioPinMode_Output);
	NvOdmGpioConfig( charging_ic->hGpio, charging_ic->hStatusPin, NvOdmGpioPinMode_InputData);
	NvOdmGpioConfig( charging_ic->hGpio, charging_ic->hPGBPin, NvOdmGpioPinMode_InputData);

	star_initial_charger_state();

	if (NvOdmGpioInterruptRegister(charging_ic->hGpio, &charging_ic->hCHGDone_int,
	                                                    charging_ic->hStatusPin, NvOdmGpioPinMode_InputInterruptAny,
	                                                    star_full_battery_interrupt_handler, (void*)&charging_ic, 0) == NV_FALSE)
	{
		LDC("[Critical] NvOdmGpioInterruptRegister Fail!");
		rc = -ENOSYS;
		goto err_gpio_interrupt_register_fail;
	}

	LDC("[Debug]: Star charging_ic Initialization was done");

//20100609, , Charger Code : GPIO Setting [END]

	batt_dev->present = NV_TRUE; // Assume battery is present at start
	batt_dev->batt_health = POWER_SUPPLY_HEALTH_GOOD;
	batt_dev->BatteryLifePercent = 104; // default setting
	batt_dev->BatteryGauge = 111;
	batt_dev->sleep_bat_check_period = 1200; // second
	batt_dev->old_alarm_sec = 0;
	batt_dev->old_checkbat_sec = 0;
	batt_dev->last_cbc_time = 0;
	batt_dev->gauge_follow_time = NvOsGetTimeMS();
	batt_dev->cbc_request_time = CBC_REQUEST_TIME_FIRST;
	batt_dev->cbc_request_state = Request_First_Time;
	batt_dev->cbc_clear_state = Clear_No_Response_0;
//20100824, , get capacity using battery voltage for demo [START]
#if defined (USE_ONETIME_VOLTAGE_CAPACITY)
	batt_dev->repeat_index = NV_TRUE; //NV_TRUE for getting capacity from Battery voltage
	batt_dev->Capacity_first_time = NV_TRUE;
	batt_dev->BatteryGauge_on = NV_FALSE;
	batt_dev->RIL_ready = NV_FALSE;
	batt_dev->Boot_TA_setting = NV_FALSE;
#endif // USE_ONETIME_VOLTAGE_CAPACITY
//20100824, , get capacity using battery voltage for demo [END]
	previous_guage = 100;

	for (i = 0; i < ARRAY_SIZE(tegra_supplies); i++) {
		rc = power_supply_register(&pdev->dev, &tegra_supplies[i]);
		if (rc) {
			printk(KERN_ERR "Failed to register power supply\n");
			LDB("[Critical] Failed to register power supply");
			while (i--)
				power_supply_unregister(&tegra_supplies[i]);
//20100630, , Change probe function structure [START]
#if defined (CONFIG_MACH_STAR)
			goto err_device_register_fail;
#else // Original Code
			kfree(batt_dev);
			return rc;
#endif // CONFIG_MACH_STAR
//20100630, , Change probe function structure [END]
		}
	}

	printk(KERN_INFO "%s: battery driver registered\n", pdev->name);

	batt_dev->battery_poll_interval = NVBATTERY_POLLING_INTERVAL*HZ;
	batt_dev->battery_workqueue = create_singlethread_workqueue("battery_workqueue");
	if (batt_dev->battery_workqueue == NULL)
	{
		rc = -ENOMEM;
		goto err_create_work_queue_fail;
	}

	INIT_DELAYED_WORK(&batt_dev->battery_status_poll_work, tegra_battery_status_poll_work);
	INIT_DELAYED_WORK(&batt_dev->battery_id_poll_work, star_battery_id_poll_work);
	//INIT_DELAYED_WORK(&batt_dev->battery_charge_done_work, star_battery_charge_done_work);

//20100915, , For AT command [START]
	rc = device_create_file(&pdev->dev, &star_at_charge_attr);
	if (rc) {goto err_device_create_fail;}

	rc = device_create_file(&pdev->dev, &star_at_chcomp_attr);
	if (rc) {goto err_device_create_fail;}
//20100915, , For AT command [END]
//20100810, , battery debugging [START]
	rc = device_create_file(&pdev->dev, &star_debug_attr);
	if (rc) {goto err_device_create_fail;}
//20100810, , battery debugging [END]

	rc = device_create_file(&pdev->dev, &star_cbc_attr);
	if (rc) {goto err_device_create_fail;}

	rc = device_create_file(&pdev->dev, &star_cbcdc_attr);
	if (rc) {goto err_device_create_fail;}

	rc = device_create_file(&pdev->dev, &tegra_battery_attr);
	if (rc)
	{
//20100630, , Change probe function structure [START]
#if defined (CONFIG_MACH_STAR)
		LDB("[Critical] device_create_file FAILED");
		goto err_device_create_fail;
#else // Original Code
		for (i = 0; i < ARRAY_SIZE(tegra_supplies); i++) {
			power_supply_unregister(&tegra_supplies[i]);
		}

		del_timer_sync(&(batt_dev->charger_state_read_timer));

		pr_err("tegra_battery_probe:device_create_file FAILED");
		return rc;
#endif // CONFIG_MACH_STAR
//20100630, , Change probe function structure [END]
	}

	queue_delayed_work(batt_dev->battery_workqueue, &batt_dev->battery_status_poll_work, HZ*6);
	queue_delayed_work(batt_dev->battery_workqueue, &batt_dev->battery_id_poll_work, HZ*2);

#if defined(DEBUGFS_STAR_BATT_TEST_MODE)
	star_battery_debug_init();
#endif // DEBUGFS_STAR_BATT_TEST_MODE
	return 0;

//20100609, , Charger Code [START]
err_device_create_fail:
	destroy_workqueue(batt_dev->battery_workqueue);
err_create_work_queue_fail:
	for (i = 0; i < ARRAY_SIZE(tegra_supplies); i++)
	{
		power_supply_unregister(&tegra_supplies[i]);
	}
err_device_register_fail:
	NvOdmGpioInterruptUnregister(charging_ic->hGpio, charging_ic->hStatusPin, charging_ic->hCHGDone_int);
err_gpio_interrupt_register_fail:
	NvOdmGpioReleasePinHandle(charging_ic->hGpio, charging_ic->hSetPin);
	NvOdmGpioReleasePinHandle(charging_ic->hGpio, charging_ic->hStatusPin);
	NvOdmGpioReleasePinHandle(charging_ic->hGpio, charging_ic->hPGBPin);
err_gpio_pin_acquire_fail:
	NvOdmGpioClose(charging_ic->hGpio);
err_open_gpio_fail:
	del_timer_sync(&(batt_dev->charger_state_read_timer));
	del_timer_sync(&(batt_dev->battery_gauge_timer));
//20100926, , Communication with BatteryService for AT Command [START]
#ifndef STAR_BATTERY_AT_COMMAND
	switch_dev_unregister(&AtComm->sdev);
err_switch_dev_register:
	kfree(AtComm);
#endif // STAR_BATTERY_AT_COMMAND
//20100926, , Communication with BatteryService for AT Command [END]
#if defined(DEBUGFS_STAR_BATT_TEST_MODE)
	kfree(debug_batt_dev);
#endif // DEBUGFS_STAR_BATT_TEST_MODE
	kfree(star_batt_dev);
	kfree(batt_dev);
	kfree(charging_ic);
	LDC("[Critical] Charging IC probe failed");
	return rc;
//20100609, , Charger Code [END]
}


static void tegra_battery_shutdown(struct platform_device *pdev)
{

    int i =0;
	printk("tegra_battery_shutdown\n");
	charging_ic_deactive();
#if 0
    if (&batt_dev->battery_id_poll_work)
    {
      cancel_delayed_work_sync(&batt_dev->battery_id_poll_work);
	  printk("battery_id_poll_work canceled\n");
    }

    if (&batt_dev->battery_status_poll_work) 
    {
	  cancel_delayed_work_sync(&batt_dev->battery_status_poll_work);
	  printk("battery_status_poll_work canceled\n");
    }
	
    if (batt_dev->battery_workqueue)
    {
	    flush_workqueue(batt_dev->battery_workqueue);
	    destroy_workqueue(batt_dev->battery_workqueue);
		printk("battery_workqueue canceled\n");
    }

	if (&(batt_dev->charger_state_read_timer))
	{
	    del_timer_sync(&(batt_dev->charger_state_read_timer));
		printk("harger_state_read_timer deleted\n");
	}	

	if (&(batt_dev->battery_gauge_timer))
	{
	   del_timer_sync(&(batt_dev->battery_gauge_timer));
	   printk("battery_gauge_timer deleted\n");
	}
#endif
	bat_shutdown = 1;

	NvOdmGpioInterruptUnregister(charging_ic->hGpio, charging_ic->hStatusPin, charging_ic->hCHGDone_int);
	NvOdmGpioSetState( charging_ic->hGpio, charging_ic->hStatusPin, 0x0);
	NvOdmGpioConfig( charging_ic->hGpio, charging_ic->hStatusPin, NvOdmGpioPinMode_Output);

	NvOdmGpioSetState( charging_ic->hGpio, charging_ic->hPGBPin, 0x0);
	NvOdmGpioConfig( charging_ic->hGpio, charging_ic->hPGBPin, NvOdmGpioPinMode_Output);

	NvOdmGpioSetState( charging_ic->hGpio, charging_ic->hSetPin, 0x0);

	NvOdmGpioReleasePinHandle(charging_ic->hGpio, charging_ic->hStatusPin);
	NvOdmGpioReleasePinHandle(charging_ic->hGpio, charging_ic->hPGBPin);
	NvOdmGpioReleasePinHandle(charging_ic->hGpio, charging_ic->hSetPin);

    if (charging_ic->hGpio) NvOdmGpioClose(charging_ic->hGpio); 

#if 0

	for (i = 0; i < ARRAY_SIZE(tegra_supplies); i++) {
		power_supply_unregister(&tegra_supplies[i]);
	}

	if (star_batt_dev)
	{
		kfree(star_batt_dev);
		star_batt_dev = NULL;
	}

	if (charging_ic)
	{
		kfree(charging_ic);
		charging_ic = NULL;
	}

	if (batt_dev) {
		device_remove_file(&pdev->dev, &tegra_battery_attr);

		kfree(batt_dev);
		batt_dev = NULL;
	}

	//20100926, , Communication with BatteryService for AT Command [START]
#ifndef STAR_BATTERY_AT_COMMAND
	if (AtComm)
	{
		switch_dev_unregister(&AtComm->sdev);
		kfree(AtComm);
		AtComm = NULL;
	}
#endif // STAR_BATTERY_AT_COMMAND
	//20100926, , Communication with BatteryService for AT Command [END]
#endif

}


static int tegra_battery_remove(struct platform_device *pdev)
{
	int i;

	//LDB("");
//20100609, , Charger Code [START]
#if defined (CONFIG_MACH_STAR)
	charging_ic_deactive();
	NvOdmGpioInterruptUnregister(charging_ic->hGpio, charging_ic->hStatusPin, charging_ic->hCHGDone_int);

	NvOdmGpioReleasePinHandle(charging_ic->hGpio, charging_ic->hSetPin);
	NvOdmGpioReleasePinHandle(charging_ic->hGpio, charging_ic->hStatusPin);
	NvOdmGpioReleasePinHandle(charging_ic->hGpio, charging_ic->hPGBPin);

	NvOdmGpioClose(charging_ic->hGpio);
#endif // CONFIG_MACH_STAR
//20100609, , Charger Code [END]

	cancel_delayed_work_sync(&batt_dev->battery_status_poll_work);
	flush_workqueue(batt_dev->battery_workqueue);
	destroy_workqueue(batt_dev->battery_workqueue);

	del_timer_sync(&(batt_dev->charger_state_read_timer));
	del_timer_sync(&(batt_dev->battery_gauge_timer));

	for (i = 0; i < ARRAY_SIZE(tegra_supplies); i++) {
		power_supply_unregister(&tegra_supplies[i]);
	}

	if (star_batt_dev)
	{
		kfree(star_batt_dev);
		star_batt_dev = NULL;
	}

	if (charging_ic)
	{
		kfree(charging_ic);
		charging_ic = NULL;
	}

	if (batt_dev) {
		device_remove_file(&pdev->dev, &tegra_battery_attr);

		kfree(batt_dev);
		batt_dev = NULL;
	}

//20100926, , Communication with BatteryService for AT Command [START]
#ifndef STAR_BATTERY_AT_COMMAND
	if (AtComm)
	{
		switch_dev_unregister(&AtComm->sdev);
		kfree(AtComm);
		AtComm = NULL;
	}
#endif // STAR_BATTERY_AT_COMMAND
//20100926, , Communication with BatteryService for AT Command [END]

	return 0;
}

static int tegra_battery_suspend(struct platform_device *dev,
	pm_message_t state)
{
	//20100929, , RTC setting for checking battery status during sleep
	static NvU32 alarm_sec = 0, now_sec = 0, checkbat_sec = 0, next_alarm_sec = 0;
	struct rtc_time tm;

	if (at_charge_index == NV_FALSE)
	{
		//20100929, , RTC setting for checking battery status during sleep [START]
		if(NvRmPmuReadAlarm(s_hRmGlobal, &alarm_sec) && NvRmPmuReadRtc(s_hRmGlobal, &now_sec) )
		{
			if ( alarm_sec == batt_dev->old_checkbat_sec )
				alarm_sec = batt_dev->old_alarm_sec;
			checkbat_sec = now_sec + batt_dev->sleep_bat_check_period;

			if( batt_dev->old_alarm_sec < now_sec )
			{
				batt_dev->old_alarm_sec = 0;
			}

			if (( STAR_BAT_MAX(checkbat_sec, alarm_sec) - STAR_BAT_MIN(checkbat_sec, alarm_sec) ) > 20) // RTC alram set when Battery check time has many different from system alram time
			{
				if ( batt_dev->old_alarm_sec == 0 )
				{
					if ( checkbat_sec < alarm_sec)   // next battery checking time is earlier than alarm time
					{
						next_alarm_sec = checkbat_sec;
						batt_dev->old_alarm_sec = alarm_sec;
						batt_dev->old_checkbat_sec = checkbat_sec;
						if (NvRmPmuWriteAlarm(s_hRmGlobal, next_alarm_sec))
						{
							rtc_time_to_tm(next_alarm_sec, &tm);
							LDB("rtc_write_to_tm: 1[%04d-%02d-%02d %02d:%02d:%02d]: write_time=0x%x ", 
								(tm.tm_year + LINUX_RTC_BASE_YEAR), tm.tm_mon+1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec, next_alarm_sec);
						}
						else
							LDB("[Warning] 1:CHG_RTC write Fail!!");
						//printk("[CHG_RTC : final] next_alarm_sec=0x%x, old_alarm_sec=0x%x\n", next_alarm_sec, batt_dev->old_alarm_sec);
					}
				}
				else
				{
					if ( (checkbat_sec <= alarm_sec) && (checkbat_sec <= batt_dev->old_alarm_sec) )
					{
						next_alarm_sec = checkbat_sec;
						batt_dev->old_alarm_sec = alarm_sec; // Assume: if alarm is changed, system determine new alarm is earlier than old alarm
						batt_dev->old_checkbat_sec = checkbat_sec;
						if (NvRmPmuWriteAlarm(s_hRmGlobal, next_alarm_sec))
						{
							rtc_time_to_tm(next_alarm_sec, &tm);
							LDB("rtc_write_to_tm: 2[%04d-%02d-%02d %02d:%02d:%02d]: write_time=0x%x ", 
								(tm.tm_year + LINUX_RTC_BASE_YEAR), tm.tm_mon+1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec, next_alarm_sec);
						}
						else
							LDB("[Warning] 2:CHG_RTC write Fail!!");
						//printk("[CHG_RTC : final] next_alarm_sec=0x%x, old_alarm_sec=0x%x\n", next_alarm_sec, batt_dev->old_alarm_sec);
					}
					else if ( (batt_dev->old_alarm_sec <= alarm_sec) && (batt_dev->old_alarm_sec <= checkbat_sec) )
					{
						next_alarm_sec = batt_dev->old_alarm_sec;
						batt_dev->old_alarm_sec = 0;
						batt_dev->old_checkbat_sec = 0;
						if (NvRmPmuWriteAlarm(s_hRmGlobal, next_alarm_sec))
						{
							rtc_time_to_tm(next_alarm_sec, &tm);
							LDB("rtc_write_to_tm: 3[%04d-%02d-%02d %02d:%02d:%02d]: write_time=0x%x ", 
								(tm.tm_year + LINUX_RTC_BASE_YEAR), tm.tm_mon+1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec, next_alarm_sec);
						}
						else
							LDB("[Warning] 3:CHG_RTC write Fail!!");
						//printk("[CHG_RTC : final] next_alarm_sec=0x%x, old_alarm_sec=0x%x", next_alarm_sec, batt_dev->old_alarm_sec);
					}
				}
			}
		}
		else
			LDB("[Critical]: ReadAlarm && ReadRTC are failed!!!");
		//20100929, , RTC setting for checking battery status during sleep [END]
	}

//20100609, , Charger Code [START]
	dev->dev.power.power_state = state;
//20100609, , Charger Code [END]
	/* Kill the Battery Polling timer */
	cancel_delayed_work_sync(&batt_dev->battery_id_poll_work);
	cancel_delayed_work_sync(&batt_dev->battery_status_poll_work);

	printk("[CHG_SLEEP] %s() end !!! \n", __func__);

	return 0;
}

static int tegra_battery_resume(struct platform_device *dev)
{
	//LDB("");
//20100609, , Charger Code [START]
	dev->dev.power.power_state = PMSG_ON;
//20100609, , Charger Code [END]
	/*Create Battery Polling timer */
	queue_delayed_work(batt_dev->battery_workqueue, &batt_dev->battery_id_poll_work, HZ/60);
	queue_delayed_work(batt_dev->battery_workqueue, &batt_dev->battery_status_poll_work, HZ/60);

	printk("[CHG_SLEEP] %s() end !!! \n", __func__);
	return 0;
}

static struct platform_driver star_battery_charger_driver =
{
	.probe	= tegra_battery_probe,
	.remove	= tegra_battery_remove,
	.suspend= tegra_battery_suspend,
	.resume	= tegra_battery_resume,
	.shutdown = tegra_battery_shutdown,
	.driver	= {
		.name	= "star_battery_charger",
		.owner	= THIS_MODULE,
	},
};

static int __init tegra_battery_init(void)
{
	LDB("");
	
	platform_driver_register(&star_battery_charger_driver);
	return 0;
}

//20100911, , MUIC kernel command line parsing [START]
static int __init array_tp_boot_state(char *str)
{
	int array_tp_boot = (int)simple_strtol(str, NULL, 0);
	if ( 1 == array_tp_boot )
		at_boot_state = 1;

	if (at_boot_state == 1)
		at_charge_index = NV_TRUE;

	printk(KERN_INFO "jh.kernel: array_tp_boot_state = %d\n",array_tp_boot);

	return 1;
}
__setup("array_tp_boot=", array_tp_boot_state);
//20100911, , MUIC kernel command line parsing [END]

static void __exit tegra_battery_exit(void)
{
	LDB("");
	
	platform_driver_unregister(&star_battery_charger_driver);
}

module_init(tegra_battery_init);
module_exit(tegra_battery_exit);
MODULE_DESCRIPTION("Star Battery Charger Driver");
