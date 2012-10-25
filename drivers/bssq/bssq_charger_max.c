/* 
 *  MAXIM MAX8971 Charger Driver
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/sysfs.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>

#include <linux/gpio.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/debugfs.h>
#include <linux/workqueue.h>
#include <lge/bssq_charger_max.h>
#include <lge/lge_hw_rev.h>
#include <mach/gpio-names.h>

#include "bssq_battery.h"

// define register map
#define MAX8971_REG_CHGINT      0x0F

#define MAX8971_REG_CHGINT_MASK 0x01

#define MAX8971_REG_CHG_STAT    0x02

#define MAX8971_DCV_MASK        0x80
#define MAX8971_DCV_SHIFT       7
#define MAX8971_TOPOFF_MASK     0x40
#define MAX8971_TOPOFF_SHIFT    6
#define MAX8971_DCI_OK          0x40    // for status register
#define MAX8971_DCI_OK_SHIFT    6
#define MAX8971_DCOVP_MASK      0x20
#define MAX8971_DCOVP_SHIFT     5
#define MAX8971_DCUVP_MASK      0x10
#define MAX8971_DCUVP_SHIFT     4
#define MAX8971_CHG_MASK        0x08
#define MAX8971_CHG_SHIFT       3
#define MAX8971_BAT_MASK        0x04
#define MAX8971_BAT_SHIFT       2
#define MAX8971_THM_MASK        0x02
#define MAX8971_THM_SHIFT       1
#define MAX8971_PWRUP_OK_MASK   0x01
#define MAX8971_PWRUP_OK_SHIFT  0
#define MAX8971_I2CIN_MASK      0x01
#define MAX8971_I2CIN_SHIFT     0

#define MAX8971_REG_DETAILS1    0x03
#define MAX8971_DC_V_MASK       0x80
#define MAX8971_DC_V_SHIFT      7
#define MAX8971_DC_I_MASK       0x40
#define MAX8971_DC_I_SHIFT      6
#define MAX8971_DC_OVP_MASK     0x20
#define MAX8971_DC_OVP_SHIFT    5
#define MAX8971_DC_UVP_MASK     0x10
#define MAX8971_DC_UVP_SHIFT    4
#define MAX8971_THM_DTLS_MASK   0x07
#define MAX8971_THM_DTLS_SHIFT  0

#define MAX8971_THM_DTLS_COLD   1       // charging suspended(temperature<T1)
#define MAX8971_THM_DTLS_COOL   2       // (T1<temperature<T2)
#define MAX8971_THM_DTLS_NORMAL 3       // (T2<temperature<T3)
#define MAX8971_THM_DTLS_WARM   4       // (T3<temperature<T4)
#define MAX8971_THM_DTLS_HOT    5       // charging suspended(temperature>T4)

#define MAX8971_REG_DETAILS2    0x04
#define MAX8971_BAT_DTLS_MASK   0x30
#define MAX8971_BAT_DTLS_SHIFT  4
#define MAX8971_CHG_DTLS_MASK   0x0F
#define MAX8971_CHG_DTLS_SHIFT  0

#define MAX8971_BAT_DTLS_BATDEAD        0   // VBAT<2.1V
#define MAX8971_BAT_DTLS_TIMER_FAULT    1   // The battery is taking longer than expected to charge
#define MAX8971_BAT_DTLS_BATOK          2   // VBAT is okay.
#define MAX8971_BAT_DTLS_GTBATOV        3   // VBAT > BATOV

#define MAX8971_CHG_DTLS_DEAD_BAT           0   // VBAT<2.1V, TJSHDN<TJ<TJREG
#define MAX8971_CHG_DTLS_PREQUAL            1   // VBAT<3.0V, TJSHDN<TJ<TJREG
#define MAX8971_CHG_DTLS_FAST_CHARGE_CC     2   // VBAT>3.0V, TJSHDN<TJ<TJREG
#define MAX8971_CHG_DTLS_FAST_CHARGE_CV     3   // VBAT=VBATREG, TJSHDN<TJ<TJREG
#define MAX8971_CHG_DTLS_TOP_OFF            4   // VBAT>=VBATREG, TJSHDN<TJ<TJREG
#define MAX8971_CHG_DTLS_DONE               5   // VBAT>VBATREG, T>Ttopoff+16s, TJSHDN<TJ<TJREG
#define MAX8971_CHG_DTLS_TIMER_FAULT        6   // VBAT<VBATOV, TJ<TJSHDN
#define MAX8971_CHG_DTLS_TEMP_SUSPEND       7   // TEMP<T1 or TEMP>T4
#define MAX8971_CHG_DTLS_USB_SUSPEND        8   // charger is off, DC is invalid or chaarger is disabled(USBSUSPEND)
#define MAX8971_CHG_DTLS_THERMAL_LOOP_ACTIVE    9   // TJ > REGTEMP
#define MAX8971_CHG_DTLS_CHG_OFF            10  // charger is off and TJ >TSHDN

#define MAX8971_REG_CHGCNTL1    0x05
#define MAX8971_DCMON_DIS_MASK  0x02
#define MAX8971_DCMON_DIS_SHIFT 1
#define MAX8971_USB_SUS_MASK    0x01
#define MAX8971_USB_SUS_SHIFT   0

#define MAX8971_REG_FCHGCRNT    0x06
#define MAX8971_CHGCC_MASK      0x1F
#define MAX8971_CHGCC_SHIFT     0
#define MAX8971_FCHGTIME_MASK   0xE0
#define MAX8971_FCHGTIME_SHIFT  5


#define MAX8971_REG_DCCRNT      0x07
#define MAX8971_CHGRSTRT_MASK   0x40
#define MAX8971_CHGRSTRT_SHIFT  6
#define MAX8971_DCILMT_MASK     0x3F
#define MAX8971_DCILMT_SHIFT    0

#define MAX8971_REG_TOPOFF          0x08
#define MAX8971_TOPOFFTIME_MASK     0xE0
#define MAX8971_TOPOFFTIME_SHIFT    5
#define MAX8971_IFST2P8_MASK        0x10
#define MAX8971_IFST2P8_SHIFT       4
#define MAX8971_TOPOFFTSHLD_MASK    0x0C
#define MAX8971_TOPOFFTSHLD_SHIFT   2
#define MAX8971_CHGCV_MASK          0x03
#define MAX8971_CHGCV_SHIFT         0

#define MAX8971_REG_TEMPREG     0x09
#define MAX8971_REGTEMP_MASK    0xC0
#define MAX8971_REGTEMP_SHIFT   6
#define MAX8971_THM_CNFG_MASK   0x08
#define MAX8971_THM_CNFG_SHIFT  3
#define MAX8971_SAFETYREG_MASK  0x01
#define MAX8971_SAFETYREG_SHIFT 0

#define MAX8971_REG_PROTCMD     0x0A
#define MAX8971_CHGPROT_MASK    0x0C
#define MAX8971_CHGPROT_SHIFT   2

#define DEBUG_LOG 1

struct max8971_chip {
	struct i2c_client *client;
	struct power_supply charger;
	struct max8971_platform_data *pdata;
	
	int irq;
	int chg_online;
	int mode;
	struct wake_lock  wlock;
	int cleard_first;
};

static struct max8971_chip *max8971_chg;

static struct max8971_platform_data max8971_pdata_usb500 = 
{
	.chgcc = 0x0a,				// Fast Charge Current = 500mA

	.fchgtime = 0x00,				// Fast Charge Time = disable

	.chgrstrt = 0x01,				// Fast Charge Restart Threshold = -100mV
	.dcilmt = (DCI_LIMIT(1000)),	// Input Current Limit Selection = 1000mA

	.topofftime = 0x00,			// Top Off Timer Setting = 0min
	.topofftshld = 0x03,			// Done Current Threshold = 200mA
	.chgcv = 0x00,				// Charger Termination Voltage = 4.2V

	.regtemp = 0x03,				// Die temperature thermal regulation loop setpoint = Disabled
	.safetyreg = 0x00,			// JEITA Safety region selection = Safety region 1
	.thm_config = 0x01,			// Thermal monitor configuration = NOT being monitored

	.int_mask = (u8)(0xbf),		// CHGINT_MASK = ~(MAX8971_IRQ_TOPOFF)

	.dc_mon_dis = 0x01,			// Disable DC monitor
};

static struct max8971_platform_data max8971_pdata_ta = 
{
	.chgcc = 0x0f,				// Fast Charge Current = 750mA

	.fchgtime = 0x00,				// Fast Charge Time = disable

	.chgrstrt = 0x01,				// Fast Charge Restart Threshold = -100mV
	.dcilmt = (DCI_LIMIT(1000)),	// Input Current Limit Selection = 1000mA

	.topofftime = 0x00,			// Top Off Timer Setting = 0min
	.topofftshld = 0x03,			// Done Current Threshold = 200mA
	.chgcv = 0x00,				// Charger Termination Voltage = 4.2V

	.regtemp = 0x03,				// Die temperature thermal regulation loop setpoint = Disabled
	.safetyreg = 0x00,			// JEITA Safety region selection = Safety region 1
	.thm_config = 0x01,			// Thermal monitor configuration = NOT being monitored

	.int_mask =  (u8)(0xbf),		// CHGINT_MASK = ~(MAX8971_IRQ_TOPOFF)

	.dc_mon_dis = 0x01,			// Disable DC monitor
};

static struct max8971_platform_data max8971_pdata_factory = 
{
	.chgcc = 0x1c,				// Fast Charge Current = 1400mA

	.fchgtime = 0x00,				// Fast Charge Time = disable

	.chgrstrt = 0x01,				// Fast Charge Restart Threshold = -100mV
	.dcilmt = (DCI_LIMIT(1500)),	// Input Current Limit Selection = 1500mA

	.topofftime = 0x00,			// Top Off Timer Setting = 0min
	.topofftshld = 0x03,			// Done Current Threshold = 200mA
	.chgcv = 0x00,				// Charger Termination Voltage = 4.2V

	.regtemp = 0x03,				// Die temperature thermal regulation loop setpoint = Disabled
	.safetyreg = 0x00,			// JEITA Safety region selection = Safety region 1
	.thm_config = 0x01,			// Thermal monitor configuration = NOT being monitored

	.int_mask =  (u8)(0xbf),		// CHGINT_MASK = ~(MAX8971_IRQ_TOPOFF)

	.dc_mon_dis = 0x01,			// Disable DC monitor
};

static int max8971_write_reg(struct i2c_client *client, u8 reg, u8 value)
{
	int ret = i2c_smbus_write_byte_data(client, reg, value);

	if (ret < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);
	
#if DEBUG_LOG
	printk(KERN_DEBUG "[CHARGER] WRITE : 0x%02x , 0x%02x\n", reg, value);
#endif

	return ret;
}

static int max8971_read_reg(struct i2c_client *client, u8 reg)
{
	int ret = i2c_smbus_read_byte_data(client, reg);

	if (ret < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);

#if DEBUG_LOG
	printk(KERN_DEBUG "[CHARGER] READ : 0x%02x , 0x%02x\n", reg, ret);
#endif

	return ret;
}

static int max8971_set_bits(struct i2c_client *client, u8 reg, u8 mask, u8 data)
{
	u8 value = 0;
	int ret;

	ret = max8971_read_reg(client, reg);
	if (ret < 0)
		goto out;
	value = (u8)ret; // Bug fix in max8971 driver
	value &= ~mask;
	value |= data;
	ret = max8971_write_reg(client, reg, value);
out:
	return ret;
}

#if 0
#ifdef MAX8971_PASS2
static void max8971_charge(struct work_struct *max8971_work)
{
	u8 intr, det1, det2;
	struct max8971_chip *chip = max8971_chg;

	dev_dbg(&chip->client->dev, "%s\n", __func__);

	if(chip->chg_online)
	{
		intr = max8971_read_reg(chip->client, MAX8971_REG_CHGINT);

		dev_info(&chip->client->dev, "CHG Interrupt : 0x%x\n", intr);

		if(intr)
		{
			det1 = max8971_read_reg(chip->client, MAX8971_REG_DETAILS1);
			det2 = max8971_read_reg(chip->client, MAX8971_REG_DETAILS2);

			printk(KERN_DEBUG "[CHARGER] Detail 1 : 0x%02x\n", det1);
			printk(KERN_DEBUG "[CHARGER] Detail 2 : 0x%02x\n", det2);

//			dev_info(&chip->client->dev, ">>>> Detail 1 : 0x%x\n", det1);
//			dev_info(&chip->client->dev, ">>>> Detail 2 : 0x%x\n", det2);

//			if(intr & 0x01)
//			{
//				max8971_start_charging();
//				return;
//			}

			if((det1 & 0x20) || !(det1 & 0x10) || ((det1 & 0x07) < 2 && (det1 & 0x07) > 4)) // Invalid VBUS and temperature
			{
				charger_ic_disable();
				return;
			}
#if 1
			else if((det2 & 0x0f) > 0x06) // charger off
			{
				charger_ic_disable();
				return;
			}
#endif
			else if((det2 & 0x0f) == 0x04 || (det2 & 0x0f) == 0x05) // top off or done
			{
				//charger_ic_set_irq(0);
#if defined (CONFIG_BSSQ_BATTERY)
				set_end_of_charge(1);
				battery_setEOCVoltage();
#endif
				charger_ic_disable();
#if defined (CONFIG_BSSQ_BATTERY)
				notification_of_changes_to_battery();
#endif
				return;
			}
		}

#if 0
	chip = container_of(max8971_work, struct max8971_chip, charge_work.work);

	dev_dbg(&chip->client->dev, "%s\n", __func__);

	if (chip->chg_online) 
	{
		temp = max8971_read_reg(chip->client, MAX8971_REG_CHGINT);
		dev_info(&chip->client->dev, "CHG Interrupt: 0x%x\n", temp);
		if ((temp & MAX8971_CHG_MASK) == MAX8971_CHG_MASK) 
		{
			temp = max8971_read_reg(chip->client, MAX8971_REG_DETAILS2);
			dev_info(&chip->client->dev, "Fast Charge Interrupt: details-0x%x\n", (temp & MAX8971_CHG_DTLS_MASK));
			switch ((temp & MAX8971_CHG_DTLS_MASK)) 
			{
				case MAX8971_CHG_DTLS_DEAD_BAT:
					// insert event if a customer need to do something //
					break;
				case MAX8971_CHG_DTLS_PREQUAL:
					// insert event if a customer need to do something //
					break;
				case MAX8971_CHG_DTLS_FAST_CHARGE_CC:
					// insert event if a customer need to do something //
					break;
				case MAX8971_CHG_DTLS_FAST_CHARGE_CV:
					// insert event if a customer need to do something //
					break;
				case MAX8971_CHG_DTLS_TOP_OFF:
					// insert event if a customer need to do something //
					break;
				case MAX8971_CHG_DTLS_DONE:
					// insert event if a customer need to do something //
					// Charging done and charge off automatically
					break;
				case MAX8971_CHG_DTLS_TIMER_FAULT:
					// insert event if a customer need to do something //
					break;
				case MAX8971_CHG_DTLS_TEMP_SUSPEND:
					// insert event if a customer need to do something //
					break;
				case MAX8971_CHG_DTLS_USB_SUSPEND:
					// insert event if a customer need to do something //
					break;
				case MAX8971_CHG_DTLS_THERMAL_LOOP_ACTIVE:
					// insert event if a customer need to do something //
					break;
				case MAX8971_CHG_DTLS_CHG_OFF:
					// insert event if a customer need to do something //
					break;
				default:
					break;
			}
		}
		else if ((temp & MAX8971_BAT_MASK) == MAX8971_BAT_MASK) 
		{
			switch ((temp & MAX8971_BAT_MASK)>>MAX8971_BAT_SHIFT) 
			{
				case MAX8971_BAT_DTLS_BATDEAD:
					break;
				case MAX8971_BAT_DTLS_TIMER_FAULT:
					break;
				case MAX8971_BAT_DTLS_BATOK:
					break;
				case MAX8971_BAT_DTLS_GTBATOV:
					break;
				default:
					break;
			}
		}
#endif
		schedule_delayed_work(&max8971_chg->charge_work, MAX8971_CHG_PERIOD);
	}
	else 
	{
		// Charger remove
		cancel_delayed_work(&max8971_chg->charge_work);
	}
}
#endif
#endif

static int __set_charger(struct max8971_chip *chip, int enable)
{
	u8  reg_val = 0;

	// unlock charger protection
	reg_val = MAX8971_CHGPROT_UNLOCKED<<MAX8971_CHGPROT_SHIFT;
	max8971_write_reg(chip->client, MAX8971_REG_PROTCMD, reg_val);   

	if (enable) {
		/* enable charger */

		// Set fast charge current and timer
		reg_val = ((chip->pdata->chgcc<<MAX8971_CHGCC_SHIFT) |
		(chip->pdata->fchgtime<<MAX8971_FCHGTIME_SHIFT));
		max8971_write_reg(chip->client, MAX8971_REG_FCHGCRNT, reg_val);

		// Set input current limit and charger restart threshold
		reg_val = ((chip->pdata->chgrstrt<<MAX8971_CHGRSTRT_SHIFT) |
		(chip->pdata->dcilmt<<MAX8971_DCILMT_SHIFT));
		max8971_write_reg(chip->client, MAX8971_REG_DCCRNT, reg_val);

		// Set topoff condition
		reg_val = ((chip->pdata->topofftime<<MAX8971_TOPOFFTIME_SHIFT) |
		(chip->pdata->topofftshld<<MAX8971_TOPOFFTSHLD_SHIFT) |
		(chip->pdata->chgcv<<MAX8971_CHGCV_SHIFT));
		max8971_write_reg(chip->client, MAX8971_REG_TOPOFF, reg_val);

		// Set temperature condition
		reg_val = ((chip->pdata->regtemp<<MAX8971_REGTEMP_SHIFT) |
		(chip->pdata->thm_config<<MAX8971_THM_CNFG_SHIFT) |
		(chip->pdata->safetyreg<<MAX8971_SAFETYREG_SHIFT));
		max8971_write_reg(chip->client, MAX8971_REG_TEMPREG, reg_val);       

		// Set interrupt mask
		max8971_write_reg(chip->client, MAX8971_REG_CHGINT_MASK, chip->pdata->int_mask);

		// USB Suspend and DC Voltage Monitoring
		// Set DC Voltage Monitoring to Enable and USB Suspend to Disable
		reg_val = (chip->pdata->dc_mon_dis<<MAX8971_DCMON_DIS_SHIFT) | (0<<MAX8971_USB_SUS_SHIFT);
		max8971_write_reg(chip->client, MAX8971_REG_CHGCNTL1, reg_val);  

	} else {
		/* disable charge */
		max8971_set_bits(chip->client, MAX8971_REG_CHGCNTL1, MAX8971_USB_SUS_MASK, 1);
	}
	
	dev_info(&chip->client->dev, "%s\n", (enable) ? "Enable charger" : "Disable charger");

	return 0;
}

#ifdef CONFIG_SYSFS
static ssize_t max8971_charger_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int val1, val2;
	
	val1 = max8971_read_reg(max8971_chg->client, MAX8971_REG_DETAILS1);
	val2 = max8971_read_reg(max8971_chg->client, MAX8971_REG_DETAILS2);
	
	return sprintf(buf, "[CHARGER] max8971_charger_show : Details 1: 0x%02x, Details 2: 0x%02x\n", val1, val2);
}

static ssize_t max8971_charger_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
//	int mode =0;

//	if(sscanf(buf, "%d", &mode) != 1)
//		return size;

//	return size;
	return size;//sprintf(buf, "max_8971_charger_store\n");
}

static DEVICE_ATTR(status, S_IRUGO | S_IWUGO, max8971_charger_show, max8971_charger_store);
#endif

static int max8971_charger_detail_irq(int irq, void *data, u8 *val)
{
	struct max8971_chip *chip = (struct max8971_chip *)data;

    switch (irq) 
    {
    case MAX8971_IRQ_PWRUP_OK:
        dev_info(&chip->client->dev, "Power Up OK Interrupt\n");
#if 0
        if ((val[0] & MAX8971_DCUVP_MASK) == 0) {
            // check DCUVP_OK bit in CGH_STAT
            // Vbus is valid //
            // Mask interrupt regsiter //
            max8971_write_reg(chip->client, MAX8971_REG_CHGINT_MASK, chip->pdata->int_mask);            
            // DC_V valid and start charging
            chip->chg_online = 1;
            __set_charger(chip, 1);
        }
#endif
        break;

    case MAX8971_IRQ_THM:
        dev_info(&chip->client->dev, "Thermistor Interrupt: details-0x%x\n", (val[1] & MAX8971_THM_DTLS_MASK));
        break;

    case MAX8971_IRQ_BAT:
        dev_info(&chip->client->dev, "Battery Interrupt: details-0x%x\n", (val[2] & MAX8971_BAT_DTLS_MASK));
        switch ((val[2] & MAX8971_BAT_MASK)>>MAX8971_BAT_SHIFT) 
        {
        case MAX8971_BAT_DTLS_BATDEAD:
            break;
        case MAX8971_BAT_DTLS_TIMER_FAULT:
            break;
        case MAX8971_BAT_DTLS_BATOK:
            break;
        case MAX8971_BAT_DTLS_GTBATOV:
            break;
        default:
            break;
        }
        break;

    case MAX8971_IRQ_CHG:
        dev_info(&chip->client->dev, "Fast Charge Interrupt: details-0x%x\n", (val[2] & MAX8971_CHG_DTLS_MASK));
        switch (val[2] & MAX8971_CHG_DTLS_MASK) 
        {
        case MAX8971_CHG_DTLS_DEAD_BAT:
            // insert event if a customer need to do something //
            break;
        case MAX8971_CHG_DTLS_PREQUAL:
            // insert event if a customer need to do something //
            break;
        case MAX8971_CHG_DTLS_FAST_CHARGE_CC:
            // insert event if a customer need to do something //
            break;
        case MAX8971_CHG_DTLS_FAST_CHARGE_CV:
            // insert event if a customer need to do something //
            break;
        case MAX8971_CHG_DTLS_TOP_OFF:
            // insert event if a customer need to do something //
            break;
        case MAX8971_CHG_DTLS_DONE:
            // insert event if a customer need to do something //
            // Charging done and charge off automatically
            break;
        case MAX8971_CHG_DTLS_TIMER_FAULT:
            // insert event if a customer need to do something //
            break;
        case MAX8971_CHG_DTLS_TEMP_SUSPEND:
            // insert event if a customer need to do something //
            break;
        case MAX8971_CHG_DTLS_USB_SUSPEND:
            // insert event if a customer need to do something //
            break;
        case MAX8971_CHG_DTLS_THERMAL_LOOP_ACTIVE:
            // insert event if a customer need to do something //
            break;
        case MAX8971_CHG_DTLS_CHG_OFF:
            // insert event if a customer need to do something //
            break;
        default:
            break;
        }
        break;

    case MAX8971_IRQ_DCUVP:
#if 0
        if ((val[1] & MAX8971_DC_UVP_MASK) == 0) {
            // VBUS is invalid. VDC < VDC_UVLO
            __set_charger(chip, 0);
        }
#endif
        dev_info(&chip->client->dev, "DC Under voltage Interrupt: details-0x%x\n", (val[1] & MAX8971_DC_UVP_MASK));
        break;

    case MAX8971_IRQ_DCOVP:
#if 0
        if (val[1] & MAX8971_DC_OVP_MASK) {
            // VBUS is invalid. VDC > VDC_OVLO
            __set_charger(chip, 0);
        }
#endif
        dev_info(&chip->client->dev, "DC Over voltage Interrupt: details-0x%x\n", (val[1] & MAX8971_DC_OVP_MASK));
        break;

    case MAX8971_IRQ_TOPOFF:  // MAX8971_IRQ_DCI for Rev A,B , If done on recharging, top off occurred again !!!!!
        dev_info(&chip->client->dev, "DC Input Current Limit Interrupt: details-0x%x\n", (val[1] & MAX8971_DC_I_MASK));
	// Charging is done
#if defined (CONFIG_BSSQ_BATTERY)
	printk(KERN_DEBUG "[CHARGER] Charging is DONE!!!\n");
	set_end_of_charge(1);
//	battery_setEOCVoltage();
	notification_of_changes_to_battery();

	return 1;
#endif
        break;

    case MAX8971_IRQ_DCV:
        dev_info(&chip->client->dev, "DC Input Voltage Limit Interrupt: details-0x%x\n", (val[1] & MAX8971_DC_V_MASK));
        break;

    }
    return 0;
}

static irqreturn_t max8971_charger_handler(int irq, void *data)
{
	struct max8971_chip *chip = (struct max8971_chip *)data;
	int irq_val, irq_mask, irq_name;
	u8 val[3];
	int maskAllInt = 0;

	if(chip->cleard_first && 1 == gpio_get_value(TEGRA_GPIO_PW2))
	{
//		printk(KERN_DEBUG "[CHARGER] Interrupt ignored rising edge !!!n");
		return IRQ_HANDLED;
	}
	
#ifdef CONFIG_HAS_WAKELOCK
	wake_lock_timeout(&(chip->wlock), 300);
#endif

	irq_val = max8971_read_reg(chip->client, MAX8971_REG_CHGINT);
	irq_mask = max8971_read_reg(chip->client, MAX8971_REG_CHGINT_MASK);

	val[0] = max8971_read_reg(chip->client, MAX8971_REG_CHG_STAT);
	val[1] = max8971_read_reg(chip->client, MAX8971_REG_DETAILS1);
	val[2] = max8971_read_reg(chip->client, MAX8971_REG_DETAILS2);

	printk(KERN_DEBUG "[CHARGER] Interrupt : Int(0x%02x), Stat(0x%02x), Detail 1(0x%02x), Detail 2(0x%02x)\n", irq_val, val[0], val[1], val[2]);

	for (irq_name = MAX8971_IRQ_PWRUP_OK; irq_name<MAX8971_NR_IRQS; irq_name++) {
		if ((irq_val & (0x01<<irq_name)) && !(irq_mask & (0x01<<irq_name))) {
			maskAllInt += max8971_charger_detail_irq(irq_name, data, val);
		}
	}

	if(maskAllInt > 0)
		max8971_write_reg(chip->client, MAX8971_REG_CHGINT_MASK, 0xff);
	else
		max8971_write_reg(chip->client, MAX8971_REG_CHGINT_MASK, chip->pdata->int_mask);

	chip->cleard_first = 1;

	return IRQ_HANDLED;
}

int max8971_stop_charging(void)
{
	// Charger removed
	max8971_chg->chg_online = 0;
	__set_charger(max8971_chg, 0);
	// Disable GSM TEST MODE
	// max8971_set_bits(max8971_chg->client, MAX8971_REG_TOPOFF, MAX8971_IFST2P8_MASK, 0<<MAX8971_IFST2P8_SHIFT);

	return 0;
}
EXPORT_SYMBOL(max8971_stop_charging);

int max8971_start_charging(void)
{
	// Charger inserted
	max8971_chg->chg_online = 1;
	__set_charger(max8971_chg, 1);

	return 0;
}
EXPORT_SYMBOL(max8971_start_charging);

// Some functions called by MUIC, BATTERY driver
#ifdef CONFIG_MACH_BSSQ
void charger_ic_disable(void)
{
	max8971_stop_charging();

	max8971_chg->mode = CHARGER_DISBALE;

	printk(KERN_DEBUG "[CHARGER] %s : max8971 disabled\n",__func__);
}
EXPORT_SYMBOL(charger_ic_disable);

void charger_ic_set_mode(charger_ic_status mode)
{
	switch(mode)
	{
		case CHARGER_USB500:
			max8971_chg->pdata = &max8971_pdata_usb500;
			max8971_start_charging();
			break;

		case CHARGER_ISET:
			max8971_chg->pdata = &max8971_pdata_ta;
			max8971_start_charging();
			break;

		case CHARGER_FACTORY:
			max8971_chg->pdata = &max8971_pdata_factory;
			max8971_start_charging();
			break;

		case CHARGER_DISBALE:
		case CHARGER_USB100:
		default:
			max8971_stop_charging();
			break;

	}

	max8971_chg->mode = mode;
	
	printk(KERN_DEBUG "[CHARGER] %s : max8971 enabled, mode = %d\n",__func__, mode);
}
EXPORT_SYMBOL(charger_ic_set_mode);

charger_ic_status charger_ic_get_status(void)
{
	return max8971_chg->mode;
}
EXPORT_SYMBOL(charger_ic_get_status);
#endif

static int max8971_charger_get_property(struct power_supply *psy,
                                        enum power_supply_property psp,
                                        union power_supply_propval *val)
{
	struct max8971_chip *chip = container_of(psy, struct max8971_chip, charger);
	int ret = 0;
    int chg_dtls_val;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = chip->chg_online;
		break;
    case POWER_SUPPLY_PROP_STATUS:
		ret = max8971_read_reg(chip->client, MAX8971_REG_DETAILS2);
        chg_dtls_val = (ret & MAX8971_CHG_DTLS_MASK);
        if (chip->chg_online) {
            if (chg_dtls_val == MAX8971_CHG_DTLS_DONE) {
                val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
            }
            else if ((chg_dtls_val == MAX8971_CHG_DTLS_TIMER_FAULT) ||
                     (chg_dtls_val == MAX8971_CHG_DTLS_TEMP_SUSPEND) ||
                     (chg_dtls_val == MAX8971_CHG_DTLS_USB_SUSPEND) ||
                     (chg_dtls_val == MAX8971_CHG_DTLS_CHG_OFF)) {
                val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
            }
            else {
                val->intval = POWER_SUPPLY_STATUS_CHARGING;
            }
        }
        else {
            val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
        }
        ret = 0;
		break;	
    default:
		ret = -ENODEV;
		break;
	}
	return ret;
}

static enum power_supply_property max8971_charger_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
    POWER_SUPPLY_PROP_STATUS,
};

static struct power_supply max8971_charger_ps = {
   .name = "max8971-charger",
   .type = POWER_SUPPLY_TYPE_MAINS,
   .properties = max8971_charger_props,
   .num_properties = ARRAY_SIZE(max8971_charger_props),
   .get_property = max8971_charger_get_property,
};

static __devinit int max8971_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{

	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct max8971_chip *chip;
	int ret;

	printk("[CHARGER] max8971 probe\n");

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE))
		return -EIO;

	chip = kzalloc(sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	memset(chip, 0x00, sizeof(*chip));

	chip->client = client;
	chip->pdata = &max8971_pdata_usb500;//client->dev.platform_data;
	chip->irq=gpio_to_irq(TEGRA_GPIO_PW2);
	chip->mode = CHARGER_DISBALE;

	gpio_request(TEGRA_GPIO_PW2, "charger_irqb");
	tegra_gpio_enable(TEGRA_GPIO_PW2);
	gpio_direction_input(TEGRA_GPIO_PW2);

	max8971_chg = chip;

	i2c_set_clientdata(client, chip);

	if(get_lge_pcb_revision() < REV_D)
	{
		max8971_pdata_usb500.int_mask = 0xff;
		max8971_pdata_ta.int_mask = 0xff;
		max8971_pdata_factory.int_mask = 0xff;
	}

#ifdef CONFIG_SYSFS
	if(device_create_file(&client->dev, &dev_attr_status))
	{
		printk("[CHARGER] sysfs file create error\n");
	}
#endif

#if 0
	ret = power_supply_register(&client->dev, &max8971_charger_ps);
	if (ret) {
		dev_err(&client->dev, "failed: power supply register\n");
		i2c_set_clientdata(client, NULL);
		goto out;
	}
#endif

#ifdef CONFIG_HAS_WAKELOCK
	wake_lock_init(&(chip->wlock), WAKE_LOCK_SUSPEND, "charger_wake_lock");
#endif

	ret = request_threaded_irq(chip->irq, NULL, max8971_charger_handler,
		/*IRQF_ONESHOT |IRQF_TRIGGER_LOW*/IRQF_TRIGGER_FALLING|IRQF_TRIGGER_RISING, "charger_irqb", chip); // For tegra wake

	if (unlikely(ret < 0))
	{
		pr_debug("max8971: failed to request IRQ	%X\n", ret);
		goto out;
	}

	enable_irq_wake(chip->irq);

#if 0
	chip->chg_online = 0;
	ret = max8971_read_reg(chip->client, MAX8971_REG_CHG_STAT);
	if (ret >= 0) 
    {
		chip->chg_online = (ret & MAX8971_DCUVP_MASK) ? 0 : 1;
        if (chip->chg_online) 
        {
            // Set IRQ MASK register
            max8971_write_reg(chip->client, MAX8971_REG_CHGINT_MASK, chip->pdata->int_mask);
            __set_charger(chip, 1);
        }
	}
#endif

	return 0;
out:
	kfree(chip);
	return ret;
}

static __devexit int max8971_remove(struct i2c_client *client)
{
	struct max8971_chip *chip = i2c_get_clientdata(client);

	free_irq(client->irq, chip);
	
#ifdef CONFIG_HAS_WAKELOCK
	wake_lock_destroy(&(chip->wlock));
#endif

#if 0
	power_supply_unregister(&max8971_charger_ps);
#endif
	kfree(chip);

	return 0;
}

static const struct i2c_device_id max8971_id[] = {
	{ "bssq_charger_max", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, max8971_id);

static struct i2c_driver max8971_i2c_driver = {
	.driver = {
		.name = "bssq_charger_max",
	},
	.probe		= max8971_probe,
	.remove		= __devexit_p(max8971_remove),
	.id_table	= max8971_id,
};

static int __init max8971_init(void)
{
	return i2c_add_driver(&max8971_i2c_driver);
}
module_init(max8971_init);

static void __exit max8971_exit(void)
{
	i2c_del_driver(&max8971_i2c_driver);
}
module_exit(max8971_exit);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Clark Kim <clark.kim@maxim-ic.com>");
MODULE_DESCRIPTION("Power supply driver for MAX8971");
MODULE_VERSION("4.0");
MODULE_ALIAS("platform:max8971-charger");
