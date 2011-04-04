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

#ifndef INCLUDED_NVODM_ACCELEROMETER_KXTF9_H
#define INCLUDED_NVODM_ACCELEROMETER_KXTF9_H

#if defined(_cplusplus)
extern "C"
{
#endif

#include "nvodm_services.h"
#include "nvodm_accelerometer.h"

/*=====================================================================
  KIONIX ACCEL REGISTER MAP
  =======================================================================*/
#define KIONIX_ACCEL_I2C_SLAVE_ADDR 		0x0F

/*======================================================================
  COMMON REGISTERS
  ========================================================================*/
#define KIONIX_ACCEL_I2C_ST_RESP			0x0C
#define KIONIX_ACCEL_I2C_WHO_AM_I			0x0F
#define KIONIX_ACCEL_I2C_TILT_POS_CUR		0x10
#define KIONIX_ACCEL_I2C_TILT_POS_PRE		0x11
#define KIONIX_ACCEL_I2C_STATUS_REG			0x18
#define KIONIX_ACCEL_I2C_INT_REL			0x1A
#define KIONIX_ACCEL_I2C_CTRL_REG1			0x1B
#define KIONIX_ACCEL_I2C_CTRL_REG2			0x1C
#define KIONIX_ACCEL_I2C_CTRL_REG3			0x1D
#define KIONIX_ACCEL_I2C_INT_CTRL_REG2		0x1F
#define KIONIX_ACCEL_I2C_TILT_TIMER			0x28
#define KIONIX_ACCEL_I2C_WUF_TIMER			0x29
#define KIONIX_ACCEL_I2C_WUF_THRESH			0x5A
#define KIONIX_ACCEL_I2C_TDT_TIMER			0x2B /*TDT_TIMER*/

/*=======================================================================
  KXTF9-SPECIFIC REGISTERS
  ========================================================================*/
#define KXTF9_I2C_XOUT_HPF_L				0x00
#define KXTF9_I2C_XOUT_HPF_H				0x01
#define KXTF9_I2C_YOUT_HPF_L				0x02
#define KXTF9_I2C_YOUT_HPF_H				0x03
#define KXTF9_I2C_ZOUT_HPF_L				0x04
#define KXTF9_I2C_ZOUT_HPF_H				0x05
#define KXTF9_I2C_XOUT_L					0x06
#define KXTF9_I2C_XOUT_H					0x07
#define KXTF9_I2C_YOUT_L					0x08
#define KXTF9_I2C_YOUT_H					0x09
#define KXTF9_I2C_ZOUT_L					0x0A
#define KXTF9_I2C_ZOUT_H					0x0B
#define KXTF9_I2C_INT_SRC_REG1				0x15
#define KXTF9_I2C_INT_SRC_REG2				0x16
#define KXTF9_I2C_INT_CTRL_REG1				0x1E
#define KXTF9_I2C_INT_CTRL_REG3				0x20
#define KXTF9_I2C_DATA_CTRL_REG				0x21
#define KXTF9_I2C_TDT_TIMER					0x2B
#define KXTF9_I2C_TDT_H_THRESH				0x2C
#define KXTF9_I2C_TDT_L_THRESH				0x2D
#define KXTF9_I2C_TDT_TAP_TIMER				0x2E
#define KXTF9_I2C_TDT_TOTAL_TIMER			0x2F
#define KXTF9_I2C_TDT_LATENCY_TIMER			0x30
#define KXTF9_I2C_TDT_WINDOW_TIMER			0x31

/*=======================================================================
  KXTF9-SPECIFIC CONTROL BITS
  ========================================================================*/
#define INT_CTRL_REG2_XBW_MASK				0x7f  /* X-axis motion mask */
#define INT_CTRL_REG2_XBW      				0x80
#define INT_CTRL_REG2_YBW_MASK 				0xbf  /* Y-axis motion mask */
#define INT_CTRL_REG2_YBW      				0x40
#define INT_CTRL_REG2_ZBW_MASK 				0xdf  /* Z-axis motion mask */
#define INT_CTRL_REG2_ZBW     				0x20
#define KXTF9_INT_CTRL_REG1_IEN				0x20  /* enables/disables the physical interrupt pin; 1=enable; 0=disable */
#define KXTF9_INT_CTRL_REG1_IEN_MASK		0xdf
#define CTRL_REG1_PC1						0x80  /* operating mode 1 = full power mode; 0 = stand by mode */
#define CTRL_REG1_TPS						0x01  /* enables tilt position function */
#define CTRL_REG1_WUFE						0x02	/* enables wake up function */
#define CTRL_REG1_RES						0x40	/* performance mode on KXTF9 */

#if 1
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

// Timeout for I2C transaction.
//jinsoo@lge.com 0608 audio test
enum { I2C_ACCELRATOR_TRANSACTION_TIMEOUT = 10 };	// 500 -> 10 (ms) // 2010.10.16 byoungwoo.yoon@lge.com
//enum { I2C_ACCELRATOR_TRANSACTION_TIMEOUT = 1000 };
// Maximum number of packetsize supported by the I2C controller.
enum { I2C_ACCELRATOR_PACKET_SIZE = 8};
static NvU8 s_ReadBuffer[I2C_ACCELRATOR_PACKET_SIZE];
static NvU8 s_WriteBuffer[I2C_ACCELRATOR_PACKET_SIZE];

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
	// Specifies accelerometer device address, for example, I2C write address.
	NvU8 nDevAddr;
	// Specifies the initial value that make accelerometer work, ACCELEROMETER_CONTROL_REGS_MAX_LENGHT is always 100.
	NvDevCtrlReg CtrlRegsList[ACCELEROMETER_CONTROL_REGS_MAX_LENGHT];
	// Specifies the initial CtrlRegsList length.
	NvU8 nLength;
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
#endif //Temp

#if defined(__cplusplus)
}
#endif
#endif
