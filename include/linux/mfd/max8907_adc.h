/*
 * Copyright (c) 2009 NVIDIA Corporation.
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

#ifndef INCLUDED_MAX8907_ADC_HEADER
#define INCLUDED_MAX8907_ADC_HEADER

#if defined(__cplusplus)
extern "C"
{
#endif

//20100622, jh.ahn@lge.com, Definition for ADC control[START]

//#define REG_ADC_SCHED 				0x10
// use MAX8907_ADC_SCHED
#define AUTOSCH_EN_SHIFT 			0x01
#define AUTOSCH_T_SHIFT 			0x00
#define AUTOSCH_ENABLE_MASK		        0x01
#define AUTOSCH_DISABLE_MASK		        0x00
#define AUTOSCH_T_10S_MASK		        0x01
#define AUTOSCH_T_1S_MASK			0x00

//#define REG_ADC_RES_CNFG1			0x06
// use MAX8907_ADC_RES_CNFG1
#define RES_AUX1_SHIFT				0x07
#define RES_AUX2_SHIFT				0x06
#define RES_VCHG_SHIFT				0x05
#define RES_VBBATT_SHIFT			0x04
#define RES_VMBATT_SHIFT			0x03
#define RES_ISNS_SHIFT				0x02
#define RES_THM_SHIFT				0x01
#define RES_TDIE_SHIFT				0x00
#define RES_8bit_MASK				0x01
#define RES_12bit_MASK				0x00

//#define REG_ADC_AVG_CNFG1			0x07
// use MAX8907_ADC_AVG_CNFG1
#define AVG_AUX1_EN_SHIFT			0x07
#define AVG_AUX2_EN_SHIFT			0x06
#define AVG_VCHG_EN_SHIFT			0x05
#define AVG_VBBATT_EN_SHIFT		        0x04
#define AVG_VMBATT_EN_SHIFT		        0x03
#define AVG_ISNS_EN_SHIFT			0x02
#define AVG_THM_EN_SHIFT			0x01
#define AVG_TDIE_EN_SHIFT			0x00
#define AVG_8AVG_MASK				0x01
#define AVG_SINGLE_MASK			        0x00

//#define REG_ADC_ACQ_CNFG1			0x08
// use MAX8907_ADC_ACQ_CNFG1
#define T_ACQ_AUX1_SHIFT			0x07
#define T_ACQ_AUX2_SHIFT			0x05
#define T_ACQ_VCHG_SHIFT			0x03
#define T_ACQ_VBBATT_SHIFT			0x01
//#define REG_ADC_ACQ_CNFG2			0x09
// use MAX8907_ADC_ACQ_CNFG2
#define T_ACQ_VMBATT_SHIFT		        0x06
#define T_ACQ_ISNS_SHIFT			0x04
#define T_ACQ_THM_SHIFT			        0x02
#define T_ACQ_TDIE_SHIFT			0x00
#define T_128_PERIOD_MASK			0x01
#define T_64_PERIOD_MASK			0x00

/*
#define REG_AUX1_MSB				0x60 -> MAX8907_AUX1_MSB
#define REG_AUX1_LSB				0x61 -> MAX8907_AUX1_LSB...
#define REG_AUX2_MSB				0x62
#define REG_AUX2_LSB				0x63
#define REG_VCHG_MSB				0x64
#define REG_VCHG_LSB				0x65
#define REG_VBBATT_MSB				0x66
#define REG_VBBATT_LSB				0x67
#define REG_VMBATT_MSB				0x68
#define REG_VMBATT_LSB				0x69
#define REG_ISNS_MSB				0x6A
#define REG_ISNS_LSB				0x6B
#define REG_THM_MSB				0x6C
#define REG_THM_LSB				0x6D
#define REG_TDIE_MSB				0x6E
#define REG_TDIE_LSB				0x6F
*/

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

// // ON/OFF controller
#define MAX8907_RESET_CNFG_INT_REF_EN_SHIFT    0x0
#define MAX8907_RESET_CNFG_INT_REF_EN_MASK     0x1
//20100622, jh.ahn@lge.com, Definition for ADC control [END]


#if defined(__cplusplus)
}
#endif

#endif // INCLUDED_MAX8907_ADC_HEADER
