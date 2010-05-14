/*
 * Copyright (c) 2007-2009 NVIDIA Corporation.
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
 
/*  NVIDIA Tegra ODM Kit Sample Accelerometer Adaptation of the
 *  WinCE Accelerometer Driver
 */

#ifndef INCLUDED_NVODM_ACCELEROMETER_ADI340_H
#define INCLUDED_NVODM_ACCELEROMETER_ADI340_H

#if defined(_cplusplus)
extern "C"
{
#endif

#include "nvodm_services.h"
#include "nvodm_accelerometer.h"

 /* Will keep some attributes from customer in the future */
 #define XLR_ATTR_CTL 0
 #define XLR_ATTR_INTCONTROL 1
 #define XLR_ATTR_INTCONTROL2 2
 #define XLR_ATTR_THRESHG 3
 #define XLR_ATTR_THRESHC 4
 #define XLR_ATTR_OFSX 5
 #define XLR_ATTR_OFSY 6
 #define XLR_ATTR_OFSZ 7
 #define XLR_ATTR_DUR  8
 #define XLR_ATTR_LATENT 9
 #define XLR_ATTR_INTVL 10
 #define XLR_ATTR_WHOAMI 11
 #define XLR_ATTR_STATUS 12
 #define XLR_ATTR_INTSOURCE 13
 #define XLR_ATTR_DATAX 14
 #define XLR_ATTR_DATAY 15
 #define XLR_ATTR_DATAZ 16
 #define XLR_ATTR_MOREINFO 17
 #define XLR_ATTR_SCALE  18
 #define XLR_ATTR_ROTATE 19
 #define XLR_ATTR_GYRO  20
 
// AD22345 register definitions
#define XLR_DEVID 0x00

// AD22340 register definitions
#define XLR_WHOAMI      0x0f        // RO - device identification
#define XLR_STATUS      0x10        // RO - device status bits
#define XLR_STATUS_DATA_READY            0x80 //0b10000000  indicate that data in XLR_DATAX,Y,Z can be read.
#define XLR_STATUS_DATA_OVERRUN         0x02 //0b00000010  indicate that the old data in XLR_DATAX,Y,Z have not be read
#define XLR_STATUS_INTRRUPT_PENDING     0x01 //0b00000001  indicate that interrupt is active.

#define XLR_INTSOURCE   0x11        // RO - interrupt source
#define XLR_INTSOURCE_X_TAP_MASK        0x80 //0b10000000  tap interrupt is from x-axis
#define XLR_INTSOURCE_Y_TAP_MASK        0x40 //0b01000000  tap interrupt is from y-axis
#define XLR_INTSOURCE_Z_TAP_MASK        0x20 //0b00100000  tap interrupt is from z-axis
#define XLR_INTSOURCE_X_COM_MASK        0x10 //0b00010000  comm interrupt is from x-axis
#define XLR_INTSOURCE_Y_COM_MASK        0x8  //0b00001000  comm interrupt is from y-axis
#define XLR_INTSOURCE_Z_COM_MASK        0x4  //0b00000100  comm interrupt is from z-axis
#define XLR_INTSOURCE_SINGLE_TAP_MASK    0x2  //0b00000010  single tap mask
#define XLR_INTSOURCE_DOUBLE_TAP_MASK    0x1  //0b00000001  double tap mask

#define XLR_CTL         0x12        // RW - device control reg
#define XLR_CTL_POWER_MASK              0xbf //0b10111111
#define XLR_CTL_LOW_POWER               0x40 //0b01000000  low power mode
#define XLR_CTL_FULL_RUN                0x00 //0b00000000  full run mode
#define XLR_CTL_MODE_MASK               0xdf //0b11011111
#define XLR_CTL_STANDBY_MODE            0x0  //0b00000000  standby mode
#define XLR_CTL_MEASURE_MODE            0x20 //0b00100000  measure mode
#define XLR_CTL_SPI_3_BUS               0x0  //0b00000000  3 wires SPI mode
#define XLR_CTL_SPI_4_BUS               0x10 //0b00010000  4 wires SPI mode
#define XLR_CTL_SELF_TEST_MODE          0x4  //0b00000100  self test mode
#define XLR_CTL_INT1_FUNC_MASK            0xfd //0b11111101  range mask
#define XLR_CTL_INT1_DATA_READY_MODE      0x2  //0b00000010  INT1 pin is data ready flag
#define XLR_CTL_INT1_INTRUPT_MODE         0x0  //0b00000000  INT1 pin is interrupt flag
#define XLR_CTL_RANGE_MASK              0xfe //0b11111110  range mask
#define XLR_CTL_8G_RANGE                0x1  //0b00000001  range is 8g
#define XLR_CTL_2G_RANGE                0x0  //0b00000000  range is 2g

#define XLR_INTCONTROL  0x13        // RW - interrupt control/config reg
#define XLR_INTCONTROL_COM_SRC_X_MASK 0x7f //0b01111111
#define XLR_INTCONTROL_COM_SRC_X      0x80 //0b10000000  x participate common interrupt
#define XLR_INTCONTROL_COM_SRC_Y_MASK 0xbf //0b10111111
#define XLR_INTCONTROL_COM_SRC_Y      0x40 //0b01000000  y participate common interrupt
#define XLR_INTCONTROL_COM_SRC_Z_MASK 0xdf //0b11011111
#define XLR_INTCONTROL_COM_SRC_Z      0x20 //0b00100000  z participate common interrupt
#define XLR_INTCONTROL_SINGLE_DOUBLE_MASK 0xef //0b11101111
#define XLR_INTCONTROL_SINGLE_TAP       0x10 //0b00010000  detect single tap
#define XLR_INTCONTROL_DOUBLE_TAP       0x0  //0b00000000  detect double tap
#define XLR_INTCONTROL_INTERRUPT_MAP_MASK 0xfb //0b11111011
#define XLR_INTCONTROL_INTERRUPT_MAP1   0x4  //0b00000100  common interrupt map to INT2, tap interrupt map to INT1
#define XLR_INTCONTROL_INTERRUPT_MAP2   0x0  //0b00000000  common interrupt map to INT1, tap interrupt map to INT2
#define XLR_INTCONTROL_TAP_INT_MASK     0xfd //0b11111101
#define XLR_INTCONTROL_TAP_INT_ENABLE   0x2  //0b00000010  enable the tap interrupt.
#define XLR_INTCONTROL_COM_INT_MASK     0xfe //0b11111110
#define XLR_INTCONTROL_COM_INT_ENABLE   0x1  //0b00000001  enable the common interrupt.

#define XLR_INTCONTROL2 0x14        // RW - interrupt control/config reg 2
#define XLR_INTCONTROL2_TAP_SRC_X_MASK 0x7f //0b01111111
#define XLR_INTCONTROL2_TAP_SRC_X      0x80 //0b10000000  x participate tap interrupt
#define XLR_INTCONTROL2_TAP_SRC_Y_MASK 0xbf //0b10111111
#define XLR_INTCONTROL2_TAP_SRC_Y      0x40 //0b01000000  y participate tap interrupt
#define XLR_INTCONTROL2_TAP_SRC_Z_MASK 0xdf //0b11011111
#define XLR_INTCONTROL2_TAP_SRC_Z      0x20 //0b00100000  z participate tap interrupt
#define XLR_INTCONTROL2_CLR_INT_MASK   0xfe //0b11111110
#define XLR_INTCONTROL2_CLR_INT        0x1  //0b00000001  clear interrupt bit and pin

#define XLR_DATAX       0x15        // RO - data from X axis
#define XLR_DATAY       0x16        // RO - data from Y axis
#define XLR_DATAZ       0x17        // RO - data from Z axis
#define XLR_MOREINFO    0x1b        // RO - additional device info

#define XLR_THRESHG     0x1c        // RW - common interrupt threshold reg
#define XLR_THRESHC     0x1d        // RW - click threshold reg

#define XLR_OFSX        0x1e        // RW - x axis offset reg
#define XLR_OFSY        0x1f        // RW - y axis offset reg
#define XLR_OFSZ        0x20        // RW - z axis offset reg
#define XLR_DUR         0x21        // RW - click duration reg
#define XLR_LATENT      0x22        // RW - click latency reg
#define XLR_INTVL       0x23        // RW - click interval reg

//virtual register which should translated in driver.
#define XLR_SCALE       0x24
#define XLR_ROTATE      0x25
#define XLR_GYRO        0x26

/*
 * Defines the threshold source for the accelerometer.
 */
typedef enum
{
    /// Indicates the accelerometer generated interrupt by exceeding the x threshold.
    NvOdmAccelerometerThresholdSource_X = 0,

    /// Indicates the accelerometer generated interrupt by exceeding the y threshold.
    NvOdmAccelerometerThresholdSource_Y,

    /// Indicates the accelerometer generated interrupt by exceeding the z threshold.
    NvOdmAccelerometerThresholdSource_Z,

    NvOdmAccelerometerThresholdSource_Force32 = 0x7FFFFFFF
} NvOdmAccelerometerThresholdSource;

// I2C device address from accelerator.
enum { I2C_ACCELRATOR_ADDRESS = 0x3A};
// Timeout for I2C transaction.
enum { I2C_ACCELRATOR_TRANSACTION_TIMEOUT = 1000 };
// Maximum number of packetsize supported by the I2C controller.
enum { I2C_ACCELRATOR_PACKET_SIZE = 8};
static NvU8 s_ReadBuffer[I2C_ACCELRATOR_PACKET_SIZE];
static NvU8 s_WriteBuffer[I2C_ACCELRATOR_PACKET_SIZE];

// Fixed device identification code.
#define XLR_IDNUM       0x4A

#define XLR_NEWCHIPID   0xE5

#define INT_EVENT_TIMEOUT 100
#define NV_ACCELEROMETER_BUS_I2C 0
#define NV_ACCELEROMETER_BUS_SPI_3 1
#define NV_ACCELEROMETER_BUS_SPI_4 2
/*
// All of interrupt pins from accelerometer are connected to one interrupt pin of ap15.
#define NV_ACCELEROMETER_INTERRUPT_GPIO_PORT   'C'-'A'
#define NV_ACCELEROMETER_INTERRUPT_GPIO_PIN    7
// only work for ap10 hw connection
#define NV_ACCELEROMETER_INTERRUPT1_GPIO_PORT   'C'-'A'
#define NV_ACCELEROMETER_INTERRUPT1_GPIO_PIN    7
#define NV_ACCELEROMETER_INTERRUPT2_GPIO_PORT   'C'-'A'
#define NV_ACCELEROMETER_INTERRUPT2_GPIO_PIN    1
*/
/*g value under 8g*/
#define NVODM_ACCELEROMETER_G_UNDER_8G   16
/*g value under 2g*/
#define NVODM_ACCELEROMETER_G_UNDER_2G   64

// Module debug: 0=disable, 1=enable
#define NVODMACCELEROMETER_ENABLE_PRINTF (0)

#if NVODMACCELEROMETER_ENABLE_PRINTF
    #define NVODMACCELEROMETER_PRINTF   NvOdmOsDebugPrintf
#else
    #define NVODMACCELEROMETER_PRINTF   (void)
#endif
/*
 * Defines the way to read accelerometer registers.
 */
typedef NvBool (*AccelerometerRegsRead)(NvOdmAccelHandle hDevice, NvU8 nRegOffset, NvU8* nData, NvU32 nLen);
/*
 * Defines the way to write accelerometer registers.
 */
typedef NvBool (*AccelerometerRegsWrite)(NvOdmAccelHandle hDevice, NvU8 nRegOffset, NvU8* nData, NvU32 nLen);
/*
 * Holds register address and value pairs. 
 */
typedef struct NvDevCtrlRegRec {
   /// Holds the register offset.
   NvU8 RegAddr;
   /// Holds the value programmed into the upper address.
   NvU8 RegValue;
} NvDevCtrlReg;
/*
 * Max accelerometer registers number.
 */
#define ACCELEROMETER_CONTROL_REGS_MAX_LENGHT 100
/*
 * Max accelerometer callback functions number.
 */
#define ACCELEROMETER_CALLBACK_ARRAY_LENGTH   5

/*
 * Gets parameters relating to the accelerometer.
 * attrib: Specifies the attributes to get.
 * - XLR_ATTR_DATAX--gets data from x-axis.
 * - XLR_ATTR_DATAY--gets data from y-axis.
 * - XLR_ATTR_DATAZ--gets data from z-axis.
 * - XLR_ATTR_OFSX--gets the x-axis offset.
 * - XLR_ATTR_OFSY--gets the y-axis offset.
 * - XLR_ATTR_OFSZ--gets the z-axis offset.
 *   
 * info: A pointer to the returned parameters.
 * Returns NV_TRUE if successful, or NV_FALSE otherwise.
 */
NvBool
NvOdmAccelerometerGetParameter(NvOdmAccelHandle hDevice, NvU8 attrib, NvU32* info);

/*
 * Sets parameters relating to the accelerometer,
 * according to the caller's requirements.
 * attrib: Specifies the attributes to set.
 * - XLR_ATTR_OFSX--sets the x-axis offset.
 * - XLR_ATTR_OFSY--sets the y-axis offset.
 * - XLR_ATTR_OFSZ--sets the z-axis offset.
 * info: Specifies the parameter information to set.
 * Returns NV_TRUE if successful, or NV_FALSE otherwise.
 */
 
NvBool
NvOdmAccelerometerSetParameter(NvOdmAccelHandle hDevice, NvU8 attrib, NvU32 info);

//-----------------------------------------------------------------
//--------------------------New API--------------------------------
//-----------------------------------------------------------------

typedef struct NvOdmAccelRec
{
    // Specifies use I2C or SPI to configure accelerometer registers.
    NvU8 nBusType;
    // Specifies accelerometer device address, for example, I2C write address.
    NvU8 nDevAddr;
    // Specifies the initial value that make accelerometer work, ACCELEROMETER_CONTROL_REGS_MAX_LENGHT is always 100.
    NvDevCtrlReg CtrlRegsList[ACCELEROMETER_CONTROL_REGS_MAX_LENGHT];
    // Specifies the initial CtrlRegsList length.
    NvU8 nLength;
    // Specifies accelerometer chip ID.
    NvU8 nChipID;
    // Specifies the way to get accelerometer register information.
    AccelerometerRegsRead RegsRead;
    // Specifies the way to set accelerometer register information.
    AccelerometerRegsWrite RegsWrite;
    // Specifies I2C handle from the system.
    NvOdmServicesI2cHandle  hOdmI2C;
    // Interrupt pin to ap15.
    NvOdmServicesGpioHandle hGpioINT;
    NvOdmGpioPinHandle      hPinINT;
    NvU32 GPIOPortINT;
    NvU32 GPIOPinINT;
    NvOdmOsSemaphoreHandle SemaphoreForINT;
    NvOdmServicesGpioIntrHandle hGpioInterrupt;
    NvOdmAccelIntType Data;
    NvOdmServicesPmuHandle hPmu;
    NvU32 VddId;
    NvU32 I2CChannelId;
    NvOdmAccelerometerCaps Caption;
    NvOdmAccelPowerType PowerState;
    // In real case, when the board put in frontispiece, the value from z axis
    // should be g, but due to physical connect on different board, the axis
    // should be remapped to the correct one.
    NvOdmAccelAxisType AxisXMapping;
    // If the physical direct is the same with our expection, the value
    // should be set to 1, or else the value should be -1.
    NvS32              AxisXDirection;
    NvOdmAccelAxisType AxisYMapping;
    NvS32              AxisYDirection;
    NvOdmAccelAxisType AxisZMapping;
    NvS32              AxisZDirection;
} NvOdmAccel;

#if defined(__cplusplus)
}
#endif
#endif
