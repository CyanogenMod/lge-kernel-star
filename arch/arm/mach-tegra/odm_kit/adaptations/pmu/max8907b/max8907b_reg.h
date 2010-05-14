/*
 * Copyright (c) 2009-2010 NVIDIA Corporation.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * Neither the name of the NVIDIA Corporation nor the names of its contributors
 * may be used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef INCLUDED_MAX8907B_REG_HEADER
#define INCLUDED_MAX8907B_REG_HEADER

#if defined(__cplusplus)
extern "C"
{
#endif

// -- MAX8907B Addresses (See Table 71 of data sheet) --

/* MAX8907B Slave Addresses */

#define MAX8907B_PMU_SLAVE_ADDR         0x78
#define MAX8907B_RTC_SLAVE_ADDR         0xD0
#define MAX8907B_ADC_SLAVE_ADDR         0x8E

/* MAX8907B Register Addresses  */

// Main-Battery Charger
#define MAX8907B_CHG_CNTL1          0x7C
#define MAX8907B_CHG_CNTL2          0x7D
#define MAX8907B_CHG_IRQ1           0x7E
#define MAX8907B_CHG_IRQ2           0x7F
#define MAX8907B_CHG_IRQ1_MASK      0x80
#define MAX8907B_CHG_IRQ2_MASK      0x81
#define MAX8907B_CHG_STAT           0x82

// Backup-Battery Charger
#define MAX8907B_BBATT_CNFG         0x78
#define MAX8907B_SDBYSEQCNT         0x13

// V1 Step Down Regulator
#define MAX8907B_SDCTL1             0x04
#define MAX8907B_SDSEQCNT1          0x05
#define MAX8907B_SDV1               0x06

// V2 Step Down Regulator
#define MAX8907B_SDCTL2             0x07
#define MAX8907B_SDSEQCNT2          0x08
#define MAX8907B_SDV2               0x09

// V3 Step Down Regulator
#define MAX8907B_SDCTL3             0x0A
#define MAX8907B_SDSEQCNT3          0x0B
#define MAX8907B_SDV3               0x0C

// LDO1 Regulator
#define MAX8907B_LDOCTL1            0x18
#define MAX8907B_LDOSEQCNT1         0x19
#define MAX8907B_LDO1VOUT           0x1A

// LDO2 Regulator
#define MAX8907B_LDOCTL2            0x1C
#define MAX8907B_LDOSEQCNT2         0x1D
#define MAX8907B_LDO2VOUT           0x1E

// LDO3 Regulator
#define MAX8907B_LDOCTL3            0x20
#define MAX8907B_LDOSEQCNT3         0x21
#define MAX8907B_LDO3VOUT           0x22

// LDO4 Regulator
#define MAX8907B_LDOCTL4            0x24
#define MAX8907B_LDOSEQCNT4         0x25
#define MAX8907B_LDO4VOUT           0x26

// LDO5 Regulator
#define MAX8907B_LDOCTL5            0x28
#define MAX8907B_LDOSEQCNT5         0x29
#define MAX8907B_LDO5VOUT           0x2A

// LDO6 Regulator
#define MAX8907B_LDOCTL6            0x2C
#define MAX8907B_LDOSEQCNT6         0x2D
#define MAX8907B_LDO6VOUT           0x2E

// LDO7 Regulator
#define MAX8907B_LDOCTL7            0x30
#define MAX8907B_LDOSEQCNT7         0x31
#define MAX8907B_LDO7VOUT           0x32

// LDO8 Regulator
#define MAX8907B_LDOCTL8            0x34
#define MAX8907B_LDOSEQCNT8         0X35
#define MAX8907B_LDO8VOUT           0x36

// LDO9 Regulator
#define MAX8907B_LDOCTL9            0x38
#define MAX8907B_LDOSEQCNT9         0x39
#define MAX8907B_LDO9VOUT           0x3A

// LDO10 Regulator
#define MAX8907B_LDOCTL10           0x3C
#define MAX8907B_LDOSEQCNT10        0x3D
#define MAX8907B_LDO10VOUT          0x3E

// LDO11 Regulator
#define MAX8907B_LDOCTL11           0x40
#define MAX8907B_LDOSEQCNT11        0x41
#define MAX8907B_LDO11VOUT          0x42

// LDO12 Regulator
#define MAX8907B_LDOCTL12           0x44
#define MAX8907B_LDOSEQCNT12        0x45
#define MAX8907B_LDO12VOUT          0x46

// LDO13 Regulator
#define MAX8907B_LDOCTL13           0x48
#define MAX8907B_LDOSEQCNT13        0x49
#define MAX8907B_LDO13VOUT          0x4A

// LDO14 Regulator
#define MAX8907B_LDOCTL14           0x4C
#define MAX8907B_LDOSEQCNT14        0x4D
#define MAX8907B_LDO14VOUT          0x4E

// LDO15 Regulator
#define MAX8907B_LDOCTL15           0x50
#define MAX8907B_LDOSEQCNT15        0x51
#define MAX8907B_LDO15VOUT          0x52

// LDO16 Regulator
#define MAX8907B_LDOCTL16           0x10
#define MAX8907B_LDOSEQCNT16        0x11
#define MAX8907B_LDO16VOUT          0x12

// LDO17 Regulator
#define MAX8907B_LDOCTL17           0x14
#define MAX8907B_LDOSEQCNT17        0x15
#define MAX8907B_LDO17VOUT          0x16

// LDO18 Regulator
#define MAX8907B_LDOCTL18           0x72
#define MAX8907B_LDOSEQCNT18        0x73
#define MAX8907B_LDO18VOUT          0x74

// LDO19 Regulator
#define MAX8907B_LDOCTL19           0x5C
#define MAX8907B_LDOSEQCNT19        0x5D
#define MAX8907B_LDO19VOUT          0x5E

// LDO20 Regulator
#define MAX8907B_LDOCTL20           0x9C
#define MAX8907B_LDOSEQCNT20        0x9D
#define MAX8907B_LDO20VOUT          0x9E

// OUT5V Regulator
#define MAX8907B_OUT5VEN            0x54
#define MAX8907B_OUT5VSEQ           0x55

// OUT3.3V Regulator
#define MAX8907B_OUT_3_3VEN         0x58
#define MAX8907B_OUT_3_3VSEQ        0x59

// Main Bias Register
#define MAX8907B_LBCNFG             0x60

// ON/OFF Controller
#define MAX8907B_SYSENSEL           0x00
#define MAX8907B_ON_OFF_IRQ1        0x01
#define MAX8907B_ON_OFF_IRQ1_MASK   0x02
#define MAX8907B_ON_OFF_STAT        0x03
#define MAX8907B_ON_OFF_IRQ2        0x0D
#define MAX8907B_ON_OFF_IRQ2_MASK   0x0E
#define MAX8907B_RESET_CNFG         0x0F

// Flexible Power Sequencer
#define MAX8907B_SEQ1CNFG           0x64
#define MAX8907B_SEQ2CNFG           0x65
#define MAX8907B_SEQ3CNFG           0x66
#define MAX8907B_SEQ4CNFG           0x67
#define MAX8907B_SEQ5CNFG           0x68
#define MAX8907B_SEQ6CNFG           0x69
#define MAX8907B_SEQ7CNFG           0x6A

// RTC Registers
#define MAX8907B_RTC_SEC            0x00
#define MAX8907B_RTC_MIN            0x01
#define MAX8907B_RTC_HOURS          0x02
#define MAX8907B_RTC_WEEKDAY        0x03
#define MAX8907B_RTC_DATE           0x04
#define MAX8907B_RTC_MONTH          0x05
#define MAX8907B_RTC_YEAR1          0x06
#define MAX8907B_RTC_YEAR2          0x07
#define MAX8907B_ALARM0_SEC         0x08
#define MAX8907B_ALARM0_MIN         0x09
#define MAX8907B_ALARM0_HOURS       0x0A
#define MAX8907B_ALARM0_WEEKDAY     0x0B
#define MAX8907B_ALARM0_DATE        0x0C
#define MAX8907B_ALARM0_MONTH       0x0D
#define MAX8907B_ALARM0_YEAR1       0x0E
#define MAX8907B_ALARM0_YEAR2       0x0F
#define MAX8907B_ALARM1_SEC         0x10
#define MAX8907B_ALARM1_MIN         0x11
#define MAX8907B_ALARM1_HOURS       0x12
#define MAX8907B_ALARM1_WEEKDAY     0x13
#define MAX8907B_ALARM1_DATE        0x14
#define MAX8907B_ALARM1_MONTH       0x15
#define MAX8907B_ALARM1_YEAR1       0x16
#define MAX8907B_ALARM1_YEAR2       0x17
#define MAX8907B_ALARM0_CNTL        0x18
#define MAX8907B_ALARM1_CNTL        0x19
#define MAX8907B_RTC_STATUS         0x1A
#define MAX8907B_RTC_CNTL           0x1B
#define MAX8907B_RTC_IRQ            0x1C
#define MAX8907B_RTC_IRQ_MASK       0x1D
#define MAX8907B_MPL_CNTL           0x1E

// ADC and Touch Screen Controller
#define MAX8907B_TSC_STA_INT        0x00
#define MAX8907B_TSC_INT_MASK       0x01
#define MAX8907B_TSC_CNFG1          0x02
#define MAX8907B_TSC_CNFG2          0x03
#define MAX8907B_TSC_CNFG3          0x04
#define MAX8907B_ADC_RES_CNFG1      0x06
#define MAX8907B_ADC_AVG_CNFG1      0x07
#define MAX8907B_ADC_ACQ_CNFG1      0x08
#define MAX8907B_ADC_ACQ_CNFG2      0x09
#define MAX8907B_ADC_SCHED          0x10
#define MAX8907B_X_MSB              0x50
#define MAX8907B_X_LSB              0x51
#define MAX8907B_Y_MSB              0x52
#define MAX8907B_Y_LSB              0x53
#define MAX8907B_Z1_MSB             0x54
#define MAX8907B_Z1_LSB             0x55
#define MAX8907B_Z2_MSB             0x56
#define MAX8907B_Z2_LSB             0x57
#define MAX8907B_AUX1_MSB           0x60
#define MAX8907B_AUX1_LSB           0x61
#define MAX8907B_AUX2_MSB           0x62
#define MAX8907B_AUX2_LSB           0x63
#define MAX8907B_VCHG_MSB           0x64
#define MAX8907B_VCHG_LSB           0x65
#define MAX8907B_VBBATT_MSB         0x66
#define MAX8907B_VBBATT_LSB         0x67
#define MAX8907B_VMBATT_MSB         0x68
#define MAX8907B_VMBATT_LSB         0x69
#define MAX8907B_ISNS_MSB           0x6A
#define MAX8907B_ISNS_LSB           0x6B
#define MAX8907B_THM_MSB            0x6C
#define MAX8907B_THM_LSB            0x6D
#define MAX8907B_TDIE_MSB           0x6E
#define MAX8907B_TDIE_LSB           0x6F

// WLED Driver
#define MAX8907B_WLED_MODE_CNTL     0x84
#define MAX8907B_ILED_CNTL          0x85

// Chip Identification
#define MAX8907B_II1RR              0x8E
#define MAX8907B_II2RR              0x8F

#define MAX8907B_REG_INVALID        0xFF

/* field defines for register bit ops */
#define MAX8907B_OUT_VOLTAGE_MASK           0x3F
#define MAX8907B_OUT_VOLTAGE_ENABLE_BIT     0x01
#define MAX8907B_OUT_VOLTAGE_DISABLE_MASK   0x3E

#define MAX8907B_CTL_SEQ_SHIFT              0x02
#define MAX8907B_CTL_SEQ_MASK               0x07

// CHG_CNTL_1
#define MAX8907B_CHG_CNTL1_NOT_CHGEN_SHIFT      0x7
#define MAX8907B_CHG_CNTL1_NOT_CHGEN_MASK       0x1
#define MAX8907B_CHG_CNTL1_CHG_TOPOFF_SHIFT     0x5
#define MAX8907B_CHG_CNTL1_CHG_TOPOFF_MASK      0x3
#define MAX8907B_CHG_CNTL1_CHG_RST_HYS_SHIFT    0x3
#define MAX8907B_CHG_CNTL1_CHG_RST_HYS_MASK     0x3
#define MAX8907B_CHG_CNTL1_FCHG_SHIFT           0x0
#define MAX8907B_CHG_CNTL1_FCHG_MASK            0x7

#define MAX8907B_CHG_CNTL1_FCHG_85MA            0
#define MAX8907B_CHG_CNTL1_FCHG_300MA           1
#define MAX8907B_CHG_CNTL1_FCHG_460MA           2
#define MAX8907B_CHG_CNTL1_FCHG_600MA           3
#define MAX8907B_CHG_CNTL1_FCHG_700MA           4
#define MAX8907B_CHG_CNTL1_FCHG_800MA           5
#define MAX8907B_CHG_CNTL1_FCHG_900MA           6
#define MAX8907B_CHG_CNTL1_FCHG_1000MA          7

// CHG_CNTL_1
#define MAX8907B_CHG_CNTL2_FCHG_TMR_SHIFT       0x4
#define MAX8907B_CHG_CNTL2_FCHG_TMR_MASK        0x3
#define MAX8907B_CHG_CNTL2_MBAT_REG_TH_SHIFT    0x3
#define MAX8907B_CHG_CNTL2_MBAT_REG_TH_MASK     0x1
#define MAX8907B_CHG_CNTL2_TDIE_THERM_REG_SHIFT 0x0
#define MAX8907B_CHG_CNTL2_TDIE_THERM_REG_MASK  0x3

// Interrupts
#define MAX8907B_CHG_STAT_VCHG_OK_SHIFT     0x7
#define MAX8907B_CHG_STAT_VCHG_OK_MASK      0x1
#define MAX8907B_CHG_STAT_CHG_TMR_FLT_SHIFT 0x5
#define MAX8907B_CHG_STAT_CHG_TMR_FLT_MASK  0x1
#define MAX8907B_CHG_STAT_CHG_EN_STAT_SHIFT 0x4
#define MAX8907B_CHG_STAT_CHG_EN_STAT_MASK  0x1
#define MAX8907B_CHG_STAT_CHG_MODE_SHIFT    0x2
#define MAX8907B_CHG_STAT_CHG_MODE_MASK     0x3
#define MAX8907B_CHG_STAT_MBDET_SHIFT       0x1
#define MAX8907B_CHG_STAT_MBDET_MASK        0x1
#define MAX8907B_CHG_STAT_MBATTLOW_SHIFT    0x0
#define MAX8907B_CHG_STAT_MBATTLOW_MASK     0x1

#define MAX8907B_CHG_IRQ1_VCHG_R_SHIFT      0x2
#define MAX8907B_CHG_IRQ1_VCHG_R_MASK       0x1
#define MAX8907B_CHG_IRQ1_VCHG_F_SHIFT      0x1
#define MAX8907B_CHG_IRQ1_VCHG_F_MASK       0x1
#define MAX8907B_CHG_IRQ1_VCHG_OVP_SHIFT    0x0
#define MAX8907B_CHG_IRQ1_VCHG_OVP_MASK     0x1

#define MAX8907B_CHG_IRQ2_CHG_TMR_FAULT_SHIFT   0x7
#define MAX8907B_CHG_IRQ2_CHG_TMR_FAULT_MASK    0x1
#define MAX8907B_CHG_IRQ2_CHG_TOPOFF_SHIFT      0x6
#define MAX8907B_CHG_IRQ2_CHG_TOPOFF_MASK       0x1
#define MAX8907B_CHG_IRQ2_CHG_DONE_SHIFT        0x5
#define MAX8907B_CHG_IRQ2_CHG_DONE_MASK         0x1
#define MAX8907B_CHG_IRQ2_CHG_RST_SHIFT         0x4
#define MAX8907B_CHG_IRQ2_CHG_RST_MASK          0x1
#define MAX8907B_CHG_IRQ2_MBATTLOW_R_SHIFT      0x3
#define MAX8907B_CHG_IRQ2_MBATTLOW_R_MASK       0x1
#define MAX8907B_CHG_IRQ2_MBATTLOW_F_SHIFT      0x2
#define MAX8907B_CHG_IRQ2_MBATTLOW_F_MASK       0x1
#define MAX8907B_CHG_IRQ2_THM_OK_F_SHIFT        0x1
#define MAX8907B_CHG_IRQ2_THM_OK_F_MASK         0x1
#define MAX8907B_CHG_IRQ2_THM_OK_R_SHIFT        0x0
#define MAX8907B_CHG_IRQ2_THM_OK_R_MASK         0x1

#define MAX8907B_ON_OFF_IRQ1_SW_R_SHIFT         0x7
#define MAX8907B_ON_OFF_IRQ1_SW_R_MASK          0x1
#define MAX8907B_ON_OFF_IRQ1_SW_F_SHIFT         0x6
#define MAX8907B_ON_OFF_IRQ1_SW_F_MASK          0x1
#define MAX8907B_ON_OFF_IRQ1_SW_1SEC_SHIFT      0x5
#define MAX8907B_ON_OFF_IRQ1_SW_1SEC_MASK       0x1
#define MAX8907B_ON_OFF_IRQ1_EXTON_R_SHIFT      0x4
#define MAX8907B_ON_OFF_IRQ1_EXTON_R_MASK       0x1
#define MAX8907B_ON_OFF_IRQ1_EXTON_F_SHIFT      0x3
#define MAX8907B_ON_OFF_IRQ1_EXTON_F_MASK       0x1
#define MAX8907B_ON_OFF_IRQ1_SW_3SEC_SHIFT      0x2
#define MAX8907B_ON_OFF_IRQ1_SW_3SEC_MASK       0x1
#define MAX8907B_ON_OFF_IRQ1_MPL_EVENT_SHIFT    0x1
#define MAX8907B_ON_OFF_IRQ1_MPL_EVENT_MASK     0x1
#define MAX8907B_ON_OFF_IRQ1_RSTIN_F_SHIFT      0x0
#define MAX8907B_ON_OFF_IRQ1_RSTIN_F_MASK       0x1

#define MAX8907B_ON_OFF_IRQ2_SYSCKEN_R_SHIFT    0x1
#define MAX8907B_ON_OFF_IRQ2_SYSCKEN_R_MASK     0x1
#define MAX8907B_ON_OFF_IRQ2_SYSCKEN_F_SHIFT    0x0
#define MAX8907B_ON_OFF_IRQ2_SYSCKEN_F_MASK     0x1

#define MAX8907B_RTC_IRQ_ALARM0_R_SHIFT         0x3
#define MAX8907B_RTC_IRQ_ALARM0_R_MASK          0x1
#define MAX8907B_RTC_IRQ_ALARM1_R_SHIFT         0x2
#define MAX8907B_RTC_IRQ_ALARM1_R_MASK          0x1

// ON/OFF controller
#define MAX8907B_SYSENSEL_HRDSTEN_SHIFT         0x7
#define MAX8907B_SYSENSEL_HRDSTEN_MASK          0x1

#define MAX8907B_RESET_CNFG_PWREN_EN_SHIFT      0x7
#define MAX8907B_RESET_CNFG_PWREN_EN_MASK       0x1

#if defined(__cplusplus)
}
#endif

#endif //INCLUDED_MAX8907B_REG_HEADER
