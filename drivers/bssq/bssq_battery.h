/*
 * bssq battery driver
 *
 * Copyright (C) 2011 LGE, Inc.
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
 *
 */

// ADC and Touch Screen Controller
#define MAX8907_TSC_STA_INT        0x00
#define MAX8907_TSC_INT_MASK       0x01
#define MAX8907_TSC_CNFG1          0x02
#define MAX8907_TSC_CNFG2          0x03
#define MAX8907_TSC_CNFG3          0x04
#define MAX8907_ADC_RES_CNFG1      0x06
#define MAX8907_ADC_AVG_CNFG1      0x07
#define MAX8907_ADC_ACQ_CNFG1      0x08
#define MAX8907_ADC_ACQ_CNFG2      0x09
#define MAX8907_ADC_SCHED          0x10
#define MAX8907_X_MSB              0x50
#define MAX8907_X_LSB              0x51
#define MAX8907_Y_MSB              0x52
#define MAX8907_Y_LSB              0x53
#define MAX8907_Z1_MSB             0x54
#define MAX8907_Z1_LSB             0x55
#define MAX8907_Z2_MSB             0x56
#define MAX8907_Z2_LSB             0x57
#define MAX8907_AUX1_MSB           0x60
#define MAX8907_AUX1_LSB           0x61
#define MAX8907_AUX2_MSB           0x62
#define MAX8907_AUX2_LSB           0x63
#define MAX8907_VCHG_MSB           0x64
#define MAX8907_VCHG_LSB           0x65
#define MAX8907_VBBATT_MSB         0x66
#define MAX8907_VBBATT_LSB         0x67
#define MAX8907_VMBATT_MSB         0x68
#define MAX8907_VMBATT_LSB         0x69
#define MAX8907_ISNS_MSB           0x6A
#define MAX8907_ISNS_LSB           0x6B
#define MAX8907_THM_MSB            0x6C
#define MAX8907_THM_LSB            0x6D
#define MAX8907_TDIE_MSB           0x6E
#define MAX8907_TDIE_LSB           0x6F

#define MAX8907_AUX1_MEASURE		0xC0
#define MAX8907_AUX2_MEASURE		0xC8
#define MAX8907_VCHG_MEASURE		0xD0
#define MAX8907_VBBATT_MEASURE		0xD8
#define MAX8907_VMBATT_MEASURE		0xE0
#define MAX8907_ISNS_MEASURE		0xE8
#define MAX8907_THM_MEASURE		0xF0
#define MAX8907_TDIE_MEASURE		0xF8

// ON/OFF Controller
#define MAX8907_SYSENSEL           0x00
#define MAX8907_ON_OFF_IRQ1        0x01
#define MAX8907_ON_OFF_IRQ1_MASK   0x02
#define MAX8907_ON_OFF_STAT        0x03
#define MAX8907_ON_OFF_IRQ2        0x0D
#define MAX8907_ON_OFF_IRQ2_MASK   0x0E
#define MAX8907_RESET_CNFG         0x0F

// // ON/OFF controller
#define MAX8907_RESET_CNFG_INT_REF_EN_SHIFT    0x0
#define MAX8907_RESET_CNFG_INT_REF_EN_MASK     0x1


// ADC & Reference Power Down after Measurement
#define CONV_REG_AUX1_OFF			0xC0
#define CONV_REG_AUX2_OFF			0xC8
#define CONV_REG_VCHG_OFF			0xD0
#define CONV_REG_VBBATT_OFF		        0xD8
#define CONV_REG_VMBATT_OFF		        0xE0
#define CONV_REG_ISNS_OFF			0xE8
#define CONV_REG_THM_OFF			0xF0
#define CONV_REG_TDIE_OFF			0xF8
// ADC & Reference Stay Powered Up after Measurement
#define CONV_REG_AUX1_ON			0xC7
#define CONV_REG_AUX2_ON			0xC9
#define CONV_REG_VCHG_ON			0xD1
#define CONV_REG_VBBATT_ON		        0xD9
#define CONV_REG_VMBATT_ON		        0xE1
#define CONV_REG_ISNS_ON			0xE9
#define CONV_REG_THM_ON			        0xF1
#define CONV_REG_TDIE_ON			0xF9

#ifndef TRUE
#define TRUE 1
#endif
#ifndef FALSE
#define FALSE 0
#endif

struct bssq_platform_data {
	int *battery_tmp_tbl;
	unsigned int tblsize;
};

extern void notification_of_changes_to_battery(void);
extern int set_end_of_charge(int complete);
extern void battery_setEOCVoltage(void);
