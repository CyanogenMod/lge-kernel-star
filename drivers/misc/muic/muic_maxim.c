/*
 * Cosmo MUIC MAX14526 driver
 *
 * Copyright (C) 2010 LGE, Inc.
 *
 * Author: Sookyoung Kim <sookyoung.kim@lge.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include <linux/module.h>
#include <linux/kernel.h>		/* printk() */
#include <linux/init.h>			/* __init, __exit */
#include <linux/uaccess.h>		/* copy_from/to_user() */
#include <linux/interrupt.h>	/* request_irq() */
#include <linux/irq.h>			/* set_irq_type() */
#include <linux/types.h>		/* kernel data types */
#include <asm/system.h>
#include "../gpio-names.h"

/*
 * kernel/arch/arm/include/asm/gpio.h includes kernel/arch/arm/plat-omap/include/mach/gpio.h which,
 * in turn, includes kernel/include/asm-generic/gpio.h.
 * <mach/gpio.h> declares gpio_get|set_value(), gpio_to_irq().
 * <asm-generic/gpio.h> declares struct gpio_chip, gpio_request(), gpio_free(), gpio_direction_input|output().
 * The actual descriptions are in kernel/drivers/gpio/gpiolib.c and kernel/arch/arm/plat-omap/gpio.c.
 */
#include <asm/gpio.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/delay.h>		/* usleep() */
#include <linux/proc_fs.h>
#include <linux/workqueue.h>	/* INIT_WORK() */
#include <linux/wakelock.h>

#include <linux/muic.h>

/* if you want to display AP_UART for debugging, you must define V56K_CONVERT_CP_USB_TO_AP_UART */
//#define V56K_CONVERT_CP_USB_TO_AP_UART

// LGE_UPDATE yoolje.cho@lge.com [[ 4 charger
extern int muic_send_cable_type(TYPE_MUIC_MODE mode);


#ifdef CONFIG_MACH_STAR
extern int half_boot_enable;
#endif

void muic_init_max14526(TYPE_RESET reset)
{
	printk(KERN_WARNING "[MUIC] max14526_init()\n");

	//[jongho3.lee@lge.com]  muic detecting...
	//atomic_set(&muic_charger_detected,0);
	if (reset == RESET) {
		/* Clears default switch position (0x03=0x24) */
		muic_i2c_write_byte(SW_CONTROL, COMP2_TO_HZ | COMN1_TO_HZ); 
	}

	/*
  	 * Iniialize MAX14526 for Detection of Accessories
  	 * Enable 200K pull-up and ADC (0x01=0x12)
  	 * Enable Interrupt [XX REMOVED AUD related XX and set AUD Click/Pop resistor(CP_EN) XX XX] (0x02=0x50)
  	 */
	muic_i2c_write_byte(CONTROL_1, ID_200 | ADC_EN);
	muic_i2c_write_byte(CONTROL_2, INT_EN);
#ifdef CONFIG_MACH_STAR_SU660
    dp3t_switch_ctrl(DP3T_NC);
    usif_switch_ctrl(USIF_AP);	
#endif	
}
EXPORT_SYMBOL(muic_init_max14526);




void set_max14526_ap_uart_mode(void) //UART_MODE
{
	printk(KERN_WARNING "[MUIC] set_max14526_ap_uart_mode\n" );

	/* Connect CP UART signals to AP */
	usif_switch_ctrl(USIF_AP);

	/* Connect AP UART to MUIC UART */
	dp3t_switch_ctrl(DP3T_AP_UART);

	/* Enables 200K, Charger Pump, and ADC (0x01=0x13) */
	muic_i2c_write_byte(CONTROL_1, ID_200 | ADC_EN | CP_EN);

	/* Enables UART Path (0x03=0x09) */
	muic_i2c_write_byte(SW_CONTROL, COMP2_TO_U2 | COMN1_TO_U1);
}


void set_max14526_ap_usb_mode(void)	//USB_MODE
{
	s32 ret;
	
	printk(KERN_WARNING "[MUIC] set_max14526_ap_usb_mode\n" );
	//KIMCS TEST	
	usif_switch_ctrl(USIF_AP);
	dp3t_switch_ctrl(DP3T_CP_UART);

	/* Enables USB Path (0x03=0x00) */
	muic_i2c_write_byte(SW_CONTROL, COMP2_TO_DP2 | COMN1_TO_DN1);

	/* Enables 200K, Charger Pump, and ADC (0x01=0x13) */
	muic_i2c_write_byte(CONTROL_1, ID_200 | ADC_EN | CP_EN);
}


void set_max14526_cp_uart_mode(void) //UART_MODE
{
	printk(KERN_WARNING "[MUIC] set_max14526_cp_uart_mode\n" );

	/* Connect CP UART signals to DP3T */
	usif_switch_ctrl(USIF_DP3T);
	
	/* Connect CP UART to MUIC UART */
	dp3t_switch_ctrl(DP3T_CP_UART);

	/* Enable 200K, Charger Pump, and ADC (0x01=0x13) */
	muic_i2c_write_byte(CONTROL_1, ID_200 | ADC_EN | CP_EN);

	/* Enable UART Path (0x03=0x09) */
	muic_i2c_write_byte(SW_CONTROL, COMP2_TO_U2 | COMN1_TO_U1);
}


void set_max14526_cp_usb_mode(void) //USB_MODE
{
	printk(KERN_WARNING "[MUIC] set_max14526_cp_usb_mode\n" );
	/* Connect CP UART signals to AP */
	usif_switch_ctrl(USIF_AP);
	
	/* Connect CP USB to MUIC UART */
	dp3t_switch_ctrl(DP3T_CP_USB);

	/* Enable 200K, Charger Pump, and ADC (0x01=0x13) */
	muic_i2c_write_byte(CONTROL_1, ID_200 | ADC_EN | CP_EN);

	/* Enable UART Path (0x03=0x09) */
	muic_i2c_write_byte(SW_CONTROL, COMP2_TO_U2 | COMN1_TO_U1);
}

#if 0 // [gieseo.park@lge.com] - not used in Cosmo
void set_max14526_charger_mode(unsigned char int_stat_value)
{
	unsigned char reg_value;

	printk(KERN_WARNING "[MUIC] set_max14526_charger_mode, int_stat_value = %x \n", int_stat_value );

	if (((int_stat_value & IDNO) == IDNO_0101) || ((int_stat_value & IDNO) == IDNO_1011)) {
		/* LG Proprietary TA Detected 180K ohm on ID */
		muic_mode = MUIC_LG_TA;
	} else if ((int_stat_value & IDNO) == IDNO_0110) {
		/* 1A charger detected */
		muic_mode = MUIC_TA_1A;
	} else {
		/* Enable interrpt and charger type detection (0x02=0x42) */
		muic_i2c_write_byte(CONTROL_2, INT_EN | CHG_TYPE);

		///////////////////////////////////////////////
		// This line should be modified by a customer.
		// 
		// Wait for Interrupt
		//
		////////////////////////////////////////////////

		/* Read INT_STAT */
		muic_i2c_read_byte(INT_STAT, &reg_value);
		muic_i2c_read_byte(STATUS, &reg_value);

		if (reg_value & DCPORT) {
			/* Dedicated charger(TA) detected */
			muic_mode = MUIC_NA_TA;
		} else if (reg_value & CHPORT) {
			set_max14526_ap_usb_mode();
		}
	}
}
#endif

void set_max14526_muic_mode(unsigned char int_stat_value)
{
	unsigned char reg_value;
	
	printk(KERN_WARNING "[MUIC] set_max14526_muic_mode, int_stat_value = 0x%02x \n", int_stat_value);

	//[gieseo.park@lge.com] - Cosmo MUIC detection code
	if (int_stat_value & V_VBUS) {
#if 0
		if ((int_stat_value & IDNO) == IDNO_0010 || 
			(int_stat_value & IDNO) == IDNO_1001 ||
			(int_stat_value & IDNO) == IDNO_1010) {
			set_max14526_cp_usb_mode();
			muic_mode = MUIC_CP_USB;
			charging_mode = CHARGING_FACTORY;
		} 
#else
        if ((int_stat_value & IDNO) == IDNO_1001 || 
			(int_stat_value & IDNO) == IDNO_1010) {
			set_max14526_cp_usb_mode();
			muic_mode = MUIC_CP_USB;
			charging_mode = CHARGING_FACTORY;
		} else if ((int_stat_value & IDNO) == IDNO_0010) {
#ifdef V56K_CONVERT_CP_USB_TO_AP_UART		
			set_max14526_ap_uart_mode();
			muic_mode = MUIC_AP_UART;
#else
			set_max14526_cp_usb_mode();
			muic_mode = MUIC_CP_USB;
#endif
			charging_mode = CHARGING_FACTORY;
		}
#endif
        else if ((int_stat_value & IDNO) == IDNO_0100) {
			set_max14526_cp_uart_mode();
			muic_mode = MUIC_CP_UART;
			charging_mode = CHARGING_FACTORY;
#if defined(CONFIG_MHL_TX_SII9244)
		} else if ((int_stat_value & IDNO) == IDNO_0000) {	//[gieseo.park@lge.com] - added MHL to Cosmo base
			muic_set_mhl_mode_detect();
			muic_mode = MUIC_MHL;
			charging_mode = CHARGING_USB;
#endif
		} else if (int_stat_value & CHGDET) {
#ifdef CONFIG_MACH_STAR_SU660
			printk("[****MUIC****] Detect Charger CHGDET!!!!");
			muic_i2c_write_byte(SW_CONTROL, COMP2_TO_HZ | COMN1_TO_HZ);
			
			muic_i2c_write_byte(CONTROL_1,ID_200 | ADC_EN  | CP_EN );
#else
			muic_i2c_write_byte(SW_CONTROL, COMP2_TO_HZ | COMN1_TO_HZ);
#endif
			muic_mode = MUIC_LG_TA;
			charging_mode = CHARGING_LG_TA;
		} else {
			muic_i2c_write_byte(SW_CONTROL, COMP2_TO_HZ | COMN1_TO_C1COMP);

			muic_mdelay(2);

			muic_i2c_read_byte(STATUS, &reg_value);

			if (reg_value & C1COMP) {
#ifdef CONFIG_MACH_STAR_SU660
				printk("[****MUIC****] Detect Charger C1COMP!!!!");
					muic_i2c_write_byte(SW_CONTROL, COMP2_TO_HZ | COMN1_TO_HZ);
				
				muic_i2c_write_byte(CONTROL_1,ID_200 | ADC_EN  | CP_EN );
				muic_i2c_write_byte(CONTROL_2, INT_EN);				
#else
				muic_i2c_write_byte(SW_CONTROL, COMP2_TO_HZ | COMN1_TO_HZ);
#endif
				muic_mode = MUIC_LG_TA;
				charging_mode = CHARGING_LG_TA;
			} else {
				set_max14526_ap_usb_mode();
				muic_mode = MUIC_AP_USB;
				charging_mode = CHARGING_USB;
			}
		}
	} else {
		if ((int_stat_value & IDNO) == IDNO_0010) {
			set_max14526_ap_uart_mode();
			muic_mode = MUIC_AP_UART;
			charging_mode = CHARGING_NONE;
		} else if ((int_stat_value & IDNO) == IDNO_0100) {
			set_max14526_cp_uart_mode();
			muic_mode = MUIC_CP_UART;
			charging_mode = CHARGING_NONE;
		} else {
			muic_mode = MUIC_UNKNOWN;
			charging_mode = CHARGING_NONE;
		}
	}
#if 0 //[gieseo.park@lge.com] - X3 original code
		if(retain_mode  == BOOT_CP_USB){
			set_max14526_cp_usb_mode_detect();
		}if ((int_stat_value & IDNO) == IDNO_0100 || (int_stat_value == 0x11)) { /* 0x11 for Special Test with MIC */  
			/* IDNO=0100? 130Kohm :: UART_MODE */
			set_max14526_cp_uart_mode_detect();
		} else if ((int_stat_value & IDNO ) == IDNO_0010) {
			/* IDNO=0010? 56Kohm  :: USB_MODE */
			set_max14526_ap_uart_mode_detect();
	#if defined(CONFIG_MHL_TX_SII9244)
		} else if (int_stat_value == 0x10) {
			/* IDNO=0000? 0Kohm  :: MHL_MODE */
			muic_set_mhl_mode_detect();    
	#else		
		} else if ((int_stat_value == 0x11) || (int_stat_value == 0x10)) {
	    	//Set_MAX14526_SpecialTest_Mode_Detect();
	#endif//
		} else if ((int_stat_value & IDNO) == IDNO_1010) {
			set_max14526_cp_usb_mode_detect();
		} else if (int_stat_value & CHGDET) {
			/* CHGDET=1? */
			set_max14526_charger_detect(int_stat_value);
		} else if (int_stat_value & V_VBUS) {
			/* V_VBUS=1? */

			/* Standard USB Host Detected (0x03=0x23) */
			muic_i2c_write_byte(SW_CONTROL, COMP2_TO_HZ | COMN1_TO_C1COMP);

			muic_udelay(1000);

			/* Read Status Reigster(0x05) */
			muic_i2c_read_byte(STATUS, &reg_value);

			if (reg_value & C1COMP) {
				/* Charger Detected COMP2/COMN1 to H-Z (0x03=0x24) */
				muic_i2c_write_byte(SW_CONTROL, COMP2_TO_HZ | COMN1_TO_HZ);
				/* Dedicated Charger(TA) Detected */
				muic_mode = MUIC_NA_TA;
			} else {
				/* Turn on USB switches */
	#if defined(CONFIG_MHL_TX_SII9244)
				if ((int_stat_value & IDNO ) == IDNO_0000) {
						muic_set_mhl_mode_detect(); 					
				} else {
						///* Standard USB Host Detected (0x03=0x23) */							
				set_max14526_ap_usb_mode_detect();
				muic_mode = MUIC_AP_USB;
			}
	#else
	            set_max14526_ap_usb_mode_detect();
				muic_mode = MUIC_AP_USB;
	#endif//
			}
		} else if ((int_stat_value & IDNO) == IDNO_0001) { 
			/* Vidoe Cable 24k registor Detected - No TV Attached */
			//muic_mode = DEVICE_VIDEO_CABLE_NO_LOAD;
		} else if ((int_stat_value & IDNO) == IDNO_0000 ) {
				muic_mode = MUIC_NONE;
		} else {
			/* Accessory Not Supported */
			muic_mode = MUIC_NONE;
		}
#endif
}


s32 muic_max14526_detect_accessory(s32 upon_irq)
{
	s32 ret = 0;

	u8 int_stat_value;
		
	/* Upon an MUIC IRQ (MUIC_INT_N falls),
	 * wait 70ms before reading INT_STAT and STATUS.
	 * After the reads, MUIC_INT_N returns to high
	 * (but the INT_STAT and STATUS contents will be held).
	 *
	 * Do this only if muic_max14526_detect_accessory() was called upon IRQ. */
	muic_mdelay(250);
	
	/* Reads INT_STAT */
	ret = muic_i2c_read_byte(INT_STAT, &int_stat_value);
	printk(KERN_WARNING "[MUIC] muic_max14526_detect_accessory, int_stat_value = 0x%02x \n", int_stat_value);
	
	if (ret < 0) {
		printk(KERN_INFO "[MUIC] INT_STAT reading failed\n");
		muic_mode = MUIC_UNKNOWN;
		charging_mode = CHARGING_UNKNOWN;
		return ret;
	}
    	//ret = muic_i2c_write_byte(CONTROL_2, 0x00); //interrupt masked by ks.kwon@lge.com for debugging
  	//muic_mode = int_stat_value & IDNO;

	//set_max14526_device_none_detect(int_stat_value);		   

	/* Branches according to the previous muic_mode */
	switch (muic_mode) {

	/* MUIC_UNKNOWN is reached in two cases both do not have nothing to do with IRQ.
	 * First, at the initialization time where the muic_mode is not available yet.
	 * Second, whenever the current muic_mode detection is failed.
	 */
	case MUIC_UNKNOWN :

	/* If the previous muic_mode was MUIC_NONE,
	 * the only possible condition for a MUIC IRQ is plugging in an accessory.
	 */
	case MUIC_NONE :
		set_max14526_muic_mode(int_stat_value);           
		break;

	/* If the previous muic_mode was MUIC_NA_TA, MUIC_LG_TA, MUIC_TA_1A, MUIC_INVALID_CHG,
	 * MUIC_AP_UART, MUIC_CP_UART, MUIC_AP_USB, MUIC_OTG, or MUIC_CP_USB,
	 * the only possible condition for a MUIC IRQ is plugging out the accessory.
	 * 
	 * In this case, initialize MUIC and wait an IRQ.
	 * We don't need to wait 250msec because this is not an erronous case
	 * (we need to reset the facility to set STATUS for an erronous case and
	 * have to wait 250msec) and, if this is not an erronous case, the facility
	 * was already initialized at the system booting.
	 */
	case MUIC_NA_TA:
	case MUIC_TA_1A:
	case MUIC_INVALID_CHG :
	case MUIC_LG_TA :
	case MUIC_AP_UART :
	case MUIC_CP_UART :
	case MUIC_AP_USB :
	case MUIC_CP_USB :
		if ((int_stat_value & V_VBUS) != 0) {					// V_VBUS == 1
			set_max14526_muic_mode(int_stat_value);
		} else if ((int_stat_value & IDNO) == IDNO_1011) {	// V_VBUS == 0
			charging_mode = CHARGING_NONE;
			muic_mode = MUIC_NONE;
#ifdef COFIG_MACH_STAR_SU660
			printk(KERN_INFO "[******MUIC*********] VBUS NONE UNPLUG\n");
#endif
		} else
			set_max14526_muic_mode(int_stat_value);
		break;

#if defined(CONFIG_MHL_TX_SII9244)
	case MUIC_MHL :
			printk(KERN_WARNING "[MUIC] Detect step3  MHL \n");
			if ((int_stat_value & V_VBUS) == 0) {    
				MHL_On(0);
			muic_mode = MUIC_NONE;
			charging_mode = CHARGING_NONE;
		}
		break;
#endif
		
	default:
		printk(KERN_WARNING "[MUIC] Failed to detect an accessory. Try again!");
		muic_mode = MUIC_UNKNOWN;
		charging_mode = CHARGING_UNKNOWN;
		ret = -1;
		break;
	}	

	
	printk(KERN_WARNING "[MUIC] muic_max14526_detect_accessory, muic_mode = %s (%d) \n", muic_mode_str[muic_mode], muic_mode );

	if (muic_mode == MUIC_UNKNOWN || muic_mode == MUIC_NONE) {
		muic_init_max14526(RESET);
		gpio_set_value(IFX_USB_VBUS_EN_GPIO, 0);
        printk(KERN_INFO "[MUIC] charging_ic_deactive()\n");
    }

	// LGE_UPDATE yoolje.cho@lge.com [[ 4 charger
	//muic_send_cable_type(muic_mode);	//[gieseo.park@lge.com] - charger type is now sent by muic.c part.

	return ret;
}
EXPORT_SYMBOL(muic_max14526_detect_accessory);
