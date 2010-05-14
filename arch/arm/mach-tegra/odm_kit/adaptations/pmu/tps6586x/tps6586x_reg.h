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

#ifndef TPS6586X_REG_HEADER
#define TPS6586X_REG_HEADER

#if defined(__cplusplus)
extern "C"
{
#endif



/* TPS6586x registers */

/* Supply Control and Voltage Settings */
#define TPS6586x_R10_SUPPLYENA              0x10
#define TPS6586x_R11_SUPPLYENB              0x11
#define TPS6586x_R12_SUPPLYENC              0x12
#define TPS6586x_R13_SUPPLYEND              0x13
#define TPS6586x_R14_SUPPLYENE              0x14
#define TPS6586x_R20_VCC1                   0x20
#define TPS6586x_R21_VCC2                   0x21
#define TPS6586x_R23_SM1V1                  0x23
#define TPS6586x_R24_SM1V2                  0x24
#define TPS6586x_R25_SM1SL                  0x25
#define TPS6586x_R26_SM0V1                  0x26
#define TPS6586x_R27_SM0V2                  0x27
#define TPS6586x_R28_SM0SL                  0x28
#define TPS6586x_R29_LDO2AV1                0x29
#define TPS6586x_R2A_LDO2AV2                0x2A
#define TPS6586x_R2F_LDO2BV1                0x2F
#define TPS6586x_R30_LDO2BV2                0x30
#define TPS6586x_R32_LDO4V1                 0x32
#define TPS6586x_R33_LDO4V2                 0x33

/* Converter Settings */
#define TPS6586x_R41_SUPPLYV1               0x41
#define TPS6586x_R42_SUPPLYV2               0x42
#define TPS6586x_R43_SUPPLYV3               0x43
#define TPS6586x_R44_SUPPLYV4               0x44
#define TPS6586x_R45_SUPPLYV5               0x45
#define TPS6586x_R46_SUPPLYV6               0x46
#define TPS6586x_R47_SMODE1                 0x47
#define TPS6586x_R48_SMODE2                 0x48

/* Charger Setup */
#define TPS6586x_R49_CHG1                   0x49
#define TPS6586x_R4A_CHG2                   0x4A
#define TPS6586x_R4B_CHG3                   0x4B

/* Power Path Setup */
#define TPS6586x_R4C_PPATH2                 0x4C

/* Sequencing */
#define TPS6586x_R4D_PGFLTMSK1              0x4D
#define TPS6586x_R4E_PGFLTMSK2              0x4E

/* Peripheral Control */
#define TPS6586x_R50_RGB1FLASH              0x50
#define TPS6586x_R51_RGB1RED                0x51
#define TPS6586x_R52_RGB1GREEN              0x52
#define TPS6586x_R53_RGB1BLUE               0x53
#define TPS6586x_R54_RGB2RED                0x54
#define TPS6586x_R55_RGB2GREEN              0x55
#define TPS6586x_R56_RGB2BLUE               0x56
#define TPS6586x_R57_SM3_SET0               0x57
#define TPS6586x_R58_SM3_SET1               0x58
#define TPS6586x_R59_LED_PWM                0x59
#define TPS6586x_R5A_DIG_PWM                0x5A
#define TPS6586x_R5B_PWM                    0x5B
#define TPS6586x_R5C_DIG_PWM1               0x5C
#define TPS6586x_R5D_GPIOSET1               0x5D
#define TPS6586x_R5E_GPIOSET2               0x5E

#if defined(CONFIG_TEGRA_ODM_HARMONY)
/*-- GPIO Register Bit Shifts/Masks --*/
// GPIO1
#define TPS6586x_R5D_GPIOSET1_GPIO1_MODE_SHIFT  0x0
#define TPS6586x_R5D_GPIOSET1_GPIO1_MODE_MASK   0x3
#define TPS6586x_R5E_GPIOSET2_GPIO1_OUT_SHIFT   0x0
#define TPS6586x_R5E_GPIOSET2_GPIO1_OUT_MASK    0x1
#define TPS6586x_R5E_GPIOSET2_GPIO1_INV_SHIFT   0x4
#define TPS6586x_R5E_GPIOSET2_GPIO1_INV_MASK    0x1

// GPIO2
#define TPS6586x_R5D_GPIOSET1_GPIO2_MODE_SHIFT  0x2
#define TPS6586x_R5D_GPIOSET1_GPIO2_MODE_MASK   0x3
#define TPS6586x_R5E_GPIOSET2_GPIO2_OUT_SHIFT   0x1
#define TPS6586x_R5E_GPIOSET2_GPIO2_OUT_MASK    0x1
#define TPS6586x_R5E_GPIOSET2_GPIO2_INV_SHIFT   0x5
#define TPS6586x_R5E_GPIOSET2_GPIO2_INV_MASK    0x1

// GPIO3
#define TPS6586x_R5D_GPIOSET1_GPIO3_MODE_SHIFT  0x4
#define TPS6586x_R5D_GPIOSET1_GPIO3_MODE_MASK   0x3
#define TPS6586x_R5E_GPIOSET2_GPIO3_OUT_SHIFT   0x2
#define TPS6586x_R5E_GPIOSET2_GPIO3_OUT_MASK    0x1
#define TPS6586x_R5E_GPIOSET2_GPIO3_INV_SHIFT   0x6
#define TPS6586x_R5E_GPIOSET2_GPIO3_INV_MASK    0x1

// GPIO4
#define TPS6586x_R5D_GPIOSET1_GPIO4_MODE_SHIFT  0x6
#define TPS6586x_R5D_GPIOSET1_GPIO4_MODE_MASK   0x3
#define TPS6586x_R5E_GPIOSET2_GPIO4_OUT_SHIFT   0x3
#define TPS6586x_R5E_GPIOSET2_GPIO4_OUT_MASK    0x1
#define TPS6586x_R5E_GPIOSET2_GPIO4_INV_SHIFT   0x7
#define TPS6586x_R5E_GPIOSET2_GPIO4_INV_MASK    0x1

#define TPS6586x_R5D_GPIO_MODE_NOT_CONFIG   0x0
#define TPS6586x_R5D_GPIO_MODE_OUTPUT       0x1
#define TPS6586x_R5D_GPIO_MODE_INPUT_ADC    0x2
#define TPS6586x_R5D_GPIO_MODE_INPUT_LDO    0x3
#endif

/* ADC0 Engine Setup */
#define TPS6586x_R60_ADCANLG                0x60
   /* Not finish yet */

/* ADC0 Engine Data */
#define TPS6586x_R61_ADC0_SET               0x61
#define TPS6586x_R62_ADC0_WAIT              0x62
#define TPS6586x_R94_ADC0_SUM2              0x94
#define TPS6586x_R95_ADC0_SUM1              0x95
#define TPS6586x_R9A_ADC0_INT               0x9A

/* Interrupt Control */
#define TPS6586x_RB0_INT_MASK1              0xB0
#define TPS6586x_RB1_INT_MASK2              0xB1
#define TPS6586x_RB2_INT_MASK3              0xB2
#define TPS6586x_RB3_INT_MASK4              0xB3
#define TPS6586x_RB4_INT_MASK5              0xB4
#define TPS6586x_RB5_INT_ACK1               0xB5
#define TPS6586x_RB6_INT_ACK2               0xB6
#define TPS6586x_RB7_INT_ACK3               0xB7
#define TPS6586x_RB8_INT_ACK4               0xB8

/* System Status */
#define TPS6586x_RB9_STAT1                  0xB9
#define TPS6586x_RBA_STAT2                  0xBA
#define TPS6586x_RBB_STAT3                  0xBB
#define TPS6586x_RBC_STAT4                  0xBC

/* RTC */
#define TPS6586x_RC0_RTC_CTRL               0xC0
#define TPS6586x_RC1_RTC_ALARM1_HI          0xC1
#define TPS6586x_RC2_RTC_ALARM1_MID         0xC2
#define TPS6586x_RC3_RTC_ALARM1_LO          0xC3
#define TPS6586x_RC4_RTC_ALARM2_HI          0xC4
#define TPS6586x_RC5_RTC_ALARM2_LO          0xC5
#define TPS6586x_RC6_RTC_COUNT4             0xC6
#define TPS6586x_RC7_RTC_COUNT3             0xC7
#define TPS6586x_RC8_RTC_COUNT2             0xC8
#define TPS6586x_RC9_RTC_COUNT1             0xC9
#define TPS6586x_RCA_RTC_COUNT0             0xCA

/* Device ID */
#define TPS6586x_RCD_VERSIONID              0xCD

#define TPS6586x_RFF_INVALID                0xFF

/* RTC */
   /* Not finish yet */
   
   
#if defined(__cplusplus)
}
#endif


#endif //TPS6586X_REG_HEADER
