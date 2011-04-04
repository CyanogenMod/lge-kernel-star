/*
 * Copyright (c) 2009 NVIDIA Corporation.
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

#ifndef INCLUDED_MAX8907_REG_HEADER
#define INCLUDED_MAX8907_REG_HEADER

#if defined(__cplusplus)
extern "C"
{
#endif

// -- MAX8907 Addresses (See Table 71 of data sheet) --

/* MAX8907 Slave Addresses */

#define MAX8907_PMU_SLAVE_ADDR         0x78
#define MAX8907_RTC_SLAVE_ADDR         0xD0
#define MAX8907_ADC_SLAVE_ADDR         0x8E

/* MAX8907 Register Addresses  */

// Main-Battery Charger
#define MAX8907_CHG_CNTL1          0x7C
#define MAX8907_CHG_CNTL2          0x7D
#define MAX8907_CHG_IRQ1           0x7E
#define MAX8907_CHG_IRQ2           0x7F
#define MAX8907_CHG_IRQ1_MASK      0x80
#define MAX8907_CHG_IRQ2_MASK      0x81
#define MAX8907_CHG_STAT           0x82

// Backup-Battery Charger
#define MAX8907_BBATT_CNFG         0x78
#define MAX8907_SDBYSEQCNT         0x13

// V1 Step Down Regulator
#define MAX8907_SDCTL1             0x04
#define MAX8907_SDSEQCNT1          0x05
#define MAX8907_SDV1               0x06

// V2 Step Down Regulator
#define MAX8907_SDCTL2             0x07
#define MAX8907_SDSEQCNT2          0x08
#define MAX8907_SDV2               0x09

// V3 Step Down Regulator
#define MAX8907_SDCTL3             0x0A
#define MAX8907_SDSEQCNT3          0x0B
#define MAX8907_SDV3               0x0C

// LDO1 Regulator
#define MAX8907_LDOCTL1            0x18
#define MAX8907_LDOSEQCNT1         0x19
#define MAX8907_LDO1VOUT           0x1A

// LDO2 Regulator
#define MAX8907_LDOCTL2            0x1C
#define MAX8907_LDOSEQCNT2         0x1D
#define MAX8907_LDO2VOUT           0x1E

// LDO3 Regulator
#define MAX8907_LDOCTL3            0x20
#define MAX8907_LDOSEQCNT3         0x21
#define MAX8907_LDO3VOUT           0x22

// LDO4 Regulator
#define MAX8907_LDOCTL4            0x24
#define MAX8907_LDOSEQCNT4         0x25
#define MAX8907_LDO4VOUT           0x26

// LDO5 Regulator
#define MAX8907_LDOCTL5            0x28
#define MAX8907_LDOSEQCNT5         0x29
#define MAX8907_LDO5VOUT           0x2A

// LDO6 Regulator
#define MAX8907_LDOCTL6            0x2C
#define MAX8907_LDOSEQCNT6         0x2D
#define MAX8907_LDO6VOUT           0x2E

// LDO7 Regulator
#define MAX8907_LDOCTL7            0x30
#define MAX8907_LDOSEQCNT7         0x31
#define MAX8907_LDO7VOUT           0x32

// LDO8 Regulator
#define MAX8907_LDOCTL8            0x34
#define MAX8907_LDOSEQCNT8         0X35
#define MAX8907_LDO8VOUT           0x36

// LDO9 Regulator
#define MAX8907_LDOCTL9            0x38
#define MAX8907_LDOSEQCNT9         0x39
#define MAX8907_LDO9VOUT           0x3A

// LDO10 Regulator
#define MAX8907_LDOCTL10           0x3C
#define MAX8907_LDOSEQCNT10        0x3D
#define MAX8907_LDO10VOUT          0x3E

// LDO11 Regulator
#define MAX8907_LDOCTL11           0x40
#define MAX8907_LDOSEQCNT11        0x41
#define MAX8907_LDO11VOUT          0x42

// LDO12 Regulator
#define MAX8907_LDOCTL12           0x44
#define MAX8907_LDOSEQCNT12        0x45
#define MAX8907_LDO12VOUT          0x46

// LDO13 Regulator
#define MAX8907_LDOCTL13           0x48
#define MAX8907_LDOSEQCNT13        0x49
#define MAX8907_LDO13VOUT          0x4A

// LDO14 Regulator
#define MAX8907_LDOCTL14           0x4C
#define MAX8907_LDOSEQCNT14        0x4D
#define MAX8907_LDO14VOUT          0x4E

// LDO15 Regulator
#define MAX8907_LDOCTL15           0x50
#define MAX8907_LDOSEQCNT15        0x51
#define MAX8907_LDO15VOUT          0x52

// LDO16 Regulator
#define MAX8907_LDOCTL16           0x10
#define MAX8907_LDOSEQCNT16        0x11
#define MAX8907_LDO16VOUT          0x12

// LDO17 Regulator
#define MAX8907_LDOCTL17           0x14
#define MAX8907_LDOSEQCNT17        0x15
#define MAX8907_LDO17VOUT          0x16

// LDO18 Regulator
#define MAX8907_LDOCTL18           0x72
#define MAX8907_LDOSEQCNT18        0x73
#define MAX8907_LDO18VOUT          0x74

// LDO19 Regulator
#define MAX8907_LDOCTL19           0x5C
#define MAX8907_LDOSEQCNT19        0x5D
#define MAX8907_LDO19VOUT          0x5E

// LDO20 Regulator
#define MAX8907_LDOCTL20           0x9C
#define MAX8907_LDOSEQCNT20        0x9D
#define MAX8907_LDO20VOUT          0x9E

// OUT5V Regulator
#define MAX8907_OUT5VEN            0x54
#define MAX8907_OUT5VSEQ           0x55

// OUT3.3V Regulator
#define MAX8907_OUT_3_3VEN         0x58
#define MAX8907_OUT_3_3VSEQ        0x59

// Main Bias Register
#define MAX8907_LBCNFG             0x60

// ON/OFF Controller
#define MAX8907_SYSENSEL           0x00
#define MAX8907_ON_OFF_IRQ1        0x01
#define MAX8907_ON_OFF_IRQ1_MASK   0x02
#define MAX8907_ON_OFF_STAT        0x03
#define MAX8907_ON_OFF_IRQ2        0x0D
#define MAX8907_ON_OFF_IRQ2_MASK   0x0E
#define MAX8907_RESET_CNFG         0x0F

// Flexible Power Sequencer
#define MAX8907_SEQ1CNFG           0x64
#define MAX8907_SEQ2CNFG           0x65
#define MAX8907_SEQ3CNFG           0x66
#define MAX8907_SEQ4CNFG           0x67
#define MAX8907_SEQ5CNFG           0x68
#define MAX8907_SEQ6CNFG           0x69
#define MAX8907_SEQ7CNFG           0x6A

// RTC Registers
#define MAX8907_RTC_SEC            0x00
#define MAX8907_RTC_MIN            0x01
#define MAX8907_RTC_HOURS          0x02
#define MAX8907_RTC_WEEKDAY        0x03
#define MAX8907_RTC_DATE           0x04
#define MAX8907_RTC_MONTH          0x05
#define MAX8907_RTC_YEAR1          0x06
#define MAX8907_RTC_YEAR2          0x07
#define MAX8907_ALARM0_SEC         0x08
#define MAX8907_ALARM0_MIN         0x09
#define MAX8907_ALARM0_HOURS       0x0A
#define MAX8907_ALARM0_WEEKDAY     0x0B
#define MAX8907_ALARM0_DATE        0x0C
#define MAX8907_ALARM0_MONTH       0x0D
#define MAX8907_ALARM0_YEAR1       0x0E
#define MAX8907_ALARM0_YEAR2       0x0F
#define MAX8907_ALARM1_SEC         0x10
#define MAX8907_ALARM1_MIN         0x11
#define MAX8907_ALARM1_HOURS       0x12
#define MAX8907_ALARM1_WEEKDAY     0x13
#define MAX8907_ALARM1_DATE        0x14
#define MAX8907_ALARM1_MONTH       0x15
#define MAX8907_ALARM1_YEAR1       0x16
#define MAX8907_ALARM1_YEAR2       0x17
#define MAX8907_ALARM0_CNTL        0x18
#define MAX8907_ALARM1_CNTL        0x19
#define MAX8907_RTC_STATUS         0x1A
#define MAX8907_RTC_CNTL           0x1B
#define MAX8907_RTC_IRQ            0x1C
#define MAX8907_RTC_IRQ_MASK       0x1D
#define MAX8907_MPL_CNTL           0x1E

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

// WLED Driver
#define MAX8907_WLED_MODE_CNTL     0x84
#define MAX8907_ILED_CNTL          0x85

// Chip Identification
#define MAX8907_II1RR              0x8E
#define MAX8907_II2RR              0x8F

#define MAX8907_REG_INVALID        0xFF

/* field defines for register bit ops */
#define MAX8907_OUT_VOLTAGE_MASK           0x3F
#define MAX8907_OUT_VOLTAGE_ENABLE_BIT     0x01
#define MAX8907_OUT_VOLTAGE_DISABLE_MASK   0x3E

#define MAX8907_CTL_SEQ_SHIFT              0x02
#define MAX8907_CTL_SEQ_MASK               0x07

// CHG_CNTL_1
#define MAX8907_CHG_CNTL1_NOT_CHGEN_SHIFT      0x7
#define MAX8907_CHG_CNTL1_NOT_CHGEN_MASK       0x1
#define MAX8907_CHG_CNTL1_CHG_TOPOFF_SHIFT     0x5
#define MAX8907_CHG_CNTL1_CHG_TOPOFF_MASK      0x3
#define MAX8907_CHG_CNTL1_CHG_RST_HYS_SHIFT    0x3
#define MAX8907_CHG_CNTL1_CHG_RST_HYS_MASK     0x3
#define MAX8907_CHG_CNTL1_FCHG_SHIFT           0x0
#define MAX8907_CHG_CNTL1_FCHG_MASK            0x7

#define MAX8907_CHG_CNTL1_FCHG_85MA            0
#define MAX8907_CHG_CNTL1_FCHG_300MA           1
#define MAX8907_CHG_CNTL1_FCHG_460MA           2
#define MAX8907_CHG_CNTL1_FCHG_600MA           3
#define MAX8907_CHG_CNTL1_FCHG_700MA           4
#define MAX8907_CHG_CNTL1_FCHG_800MA           5
#define MAX8907_CHG_CNTL1_FCHG_900MA           6
#define MAX8907_CHG_CNTL1_FCHG_1000MA          7

// CHG_CNTL_1
#define MAX8907_CHG_CNTL2_FCHG_TMR_SHIFT       0x4
#define MAX8907_CHG_CNTL2_FCHG_TMR_MASK        0x3
#define MAX8907_CHG_CNTL2_MBAT_REG_TH_SHIFT    0x3
#define MAX8907_CHG_CNTL2_MBAT_REG_TH_MASK     0x1
#define MAX8907_CHG_CNTL2_TDIE_THERM_REG_SHIFT 0x0
#define MAX8907_CHG_CNTL2_TDIE_THERM_REG_MASK  0x3

// Interrupts
#define MAX8907_CHG_STAT_VCHG_OK_SHIFT     0x7
#define MAX8907_CHG_STAT_VCHG_OK_MASK      0x1
#define MAX8907_CHG_STAT_CHG_TMR_FLT_SHIFT 0x5
#define MAX8907_CHG_STAT_CHG_TMR_FLT_MASK  0x1
#define MAX8907_CHG_STAT_CHG_EN_STAT_SHIFT 0x4
#define MAX8907_CHG_STAT_CHG_EN_STAT_MASK  0x1
#define MAX8907_CHG_STAT_CHG_MODE_SHIFT    0x2
#define MAX8907_CHG_STAT_CHG_MODE_MASK     0x3
#define MAX8907_CHG_STAT_MBDET_SHIFT       0x1
#define MAX8907_CHG_STAT_MBDET_MASK        0x1
#define MAX8907_CHG_STAT_MBATTLOW_SHIFT    0x0
#define MAX8907_CHG_STAT_MBATTLOW_MASK     0x1

#define MAX8907_CHG_IRQ1_VCHG_R_SHIFT      0x2
#define MAX8907_CHG_IRQ1_VCHG_R_MASK       0x1
#define MAX8907_CHG_IRQ1_VCHG_F_SHIFT      0x1
#define MAX8907_CHG_IRQ1_VCHG_F_MASK       0x1
#define MAX8907_CHG_IRQ1_VCHG_OVP_SHIFT    0x0
#define MAX8907_CHG_IRQ1_VCHG_OVP_MASK     0x1

#define MAX8907_CHG_IRQ2_CHG_TMR_FAULT_SHIFT   0x7
#define MAX8907_CHG_IRQ2_CHG_TMR_FAULT_MASK    0x1
#define MAX8907_CHG_IRQ2_CHG_TOPOFF_SHIFT      0x6
#define MAX8907_CHG_IRQ2_CHG_TOPOFF_MASK       0x1
#define MAX8907_CHG_IRQ2_CHG_DONE_SHIFT        0x5
#define MAX8907_CHG_IRQ2_CHG_DONE_MASK         0x1
#define MAX8907_CHG_IRQ2_CHG_RST_SHIFT         0x4
#define MAX8907_CHG_IRQ2_CHG_RST_MASK          0x1
#define MAX8907_CHG_IRQ2_MBATTLOW_R_SHIFT      0x3
#define MAX8907_CHG_IRQ2_MBATTLOW_R_MASK       0x1
#define MAX8907_CHG_IRQ2_MBATTLOW_F_SHIFT      0x2
#define MAX8907_CHG_IRQ2_MBATTLOW_F_MASK       0x1
#define MAX8907_CHG_IRQ2_THM_OK_F_SHIFT        0x1
#define MAX8907_CHG_IRQ2_THM_OK_F_MASK         0x1
#define MAX8907_CHG_IRQ2_THM_OK_R_SHIFT        0x0
#define MAX8907_CHG_IRQ2_THM_OK_R_MASK         0x1

#define MAX8907_ON_OFF_IRQ1_SW_R_SHIFT         0x7
#define MAX8907_ON_OFF_IRQ1_SW_R_MASK          0x1
#define MAX8907_ON_OFF_IRQ1_SW_F_SHIFT         0x6
#define MAX8907_ON_OFF_IRQ1_SW_F_MASK          0x1
#define MAX8907_ON_OFF_IRQ1_SW_1SEC_SHIFT      0x5
#define MAX8907_ON_OFF_IRQ1_SW_1SEC_MASK       0x1
#define MAX8907_ON_OFF_IRQ1_EXTON_R_SHIFT      0x4
#define MAX8907_ON_OFF_IRQ1_EXTON_R_MASK       0x1
#define MAX8907_ON_OFF_IRQ1_EXTON_F_SHIFT      0x3
#define MAX8907_ON_OFF_IRQ1_EXTON_F_MASK       0x1
#define MAX8907_ON_OFF_IRQ1_SW_3SEC_SHIFT      0x2
#define MAX8907_ON_OFF_IRQ1_SW_3SEC_MASK       0x1
#define MAX8907_ON_OFF_IRQ1_MPL_EVENT_SHIFT    0x1
#define MAX8907_ON_OFF_IRQ1_MPL_EVENT_MASK     0x1
#define MAX8907_ON_OFF_IRQ1_RSTIN_F_SHIFT      0x0
#define MAX8907_ON_OFF_IRQ1_RSTIN_F_MASK       0x1

#define MAX8907_ON_OFF_IRQ2_SYSCKEN_R_SHIFT    0x1
#define MAX8907_ON_OFF_IRQ2_SYSCKEN_R_MASK     0x1
#define MAX8907_ON_OFF_IRQ2_SYSCKEN_F_SHIFT    0x0
#define MAX8907_ON_OFF_IRQ2_SYSCKEN_F_MASK     0x1

#define MAX8907_RTC_IRQ_ALARM0_R_SHIFT         0x3
#define MAX8907_RTC_IRQ_ALARM0_R_MASK          0x1
#define MAX8907_RTC_IRQ_ALARM1_R_SHIFT         0x2
#define MAX8907_RTC_IRQ_ALARM1_R_MASK          0x1

// ON/OFF controller
#define MAX8907_SYSENSEL_HRDSTEN_SHIFT         0x7
#define MAX8907_SYSENSEL_HRDSTEN_MASK          0x1
#define MAX8907_SYSENSEL_SYSCKEN_SHIFT         0x6
#define MAX8907_SYSENSEL_SYSCKEN_MASK          0x1
#define MAX8907_SYSENSEL_RSTINEN_SHIFT         0x5
#define MAX8907_SYSENSEL_RSTINEN_MASK          0x1
#define MAX8907_SYSENSEL_WKEXTON_SHIFT         0x4
#define MAX8907_SYSENSEL_WKEXTON_MASK          0x1
#define MAX8907_SYSENSEL_WKCHG_SHIFT           0x3
#define MAX8907_SYSENSEL_WKCHG_MASK            0x1
#define MAX8907_SYSENSEL_WKALRM1R_SHIFT        0x2
#define MAX8907_SYSENSEL_WKALRM1R_MASK         0x1
#define MAX8907_SYSENSEL_WKALRM0R_SHIFT        0x1
#define MAX8907_SYSENSEL_WKALRM0R_MASK         0x1
#define MAX8907_SYSENSEL_WKSW_SHIFT            0x0
#define MAX8907_SYSENSEL_WKSW_MASK             0x1

#define MAX8907_RESET_CNFG_PWREN_EN_SHIFT      0x7
#define MAX8907_RESET_CNFG_PWREN_EN_MASK       0x1

//20100413, cs77.ha@lge.com, add [START]
#define MAX8907_RESET_CNFG_SFT_RST_SHIFT       0x5
#define MAX8907_RESET_CNFG_SFT_RST_MASK        0x1
#define MAX8907_SYSENSEL_POWER_OFF_SHIFT       0x6
#define MAX8907_SYSENSEL_POWER_OFF_MASK        0x1 
//20100413, cs77.ha@lge.com, add [END]

//20100427, cs77.ha@lge.com, SMPL [START]
#define MAX8907_MPL_EN_SHIFT                   0x4
#define MAX8907_MPL_EN_MASK                    0x1

#define MAX8907_MPL_TIME_SHIFT                 0x2
#define MAX8907_MPL_TIME_0_5_SEC               0x0
#define MAX8907_MPL_TIME_1_SEC                 0x1
#define MAX8907_MPL_TIME_1_5_SEC               0x2
#define MAX8907_MPL_TIME_2_SEC                 0x3
//20100427, cs77.ha@lge.com, SMPL [END]

//20100727, byoungwoo.yoon@lge.com, change SEQ [START]
#define MAX8907_LDOCTL_SEQ_MASK	0x7
#define MAX8907_LDOCTL_SEQ_SHIFT	0x2
//20100727, byoungwoo.yoon@lge.com, change SEQ [END]

//20100928, byoungwoo.yoon@lge.com, RTC alarm enable [START]
#define MAX8907_ALARM0_CNTL_ENABLE	0x77
#define MAX8907_RTC_IRQ_ALARM0_R		0x08
#define MAX8907_RTC_IRQ_ALARM1_R		0x04
#define MAX8907_RTC_IRQ_MASK_ALARM0_R	0x08
//20100928, byoungwoo.yoon@lge.com, RTC alarm enable [END]

#if defined(__cplusplus)
}
#endif

#endif //INCLUDED_MAX8907_REG_HEADER
