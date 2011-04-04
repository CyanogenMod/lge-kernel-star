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

#ifndef INCLUDED_NVODM_GYROACCEL_H
#define INCLUDED_NVODM_GYROACCEL_H

#if defined(_cplusplus)
extern "C"
{
#endif

#include "nvodm_services.h"
#include "nvodm_gyroscope_accel.h"

/*=====================================================================
  KIONIX ACCEL REGISTER MAP
  =======================================================================*/
#define MPU3050_I2C_SLAVE_ADDR 		0x69


/*======================================================================
  COMMON REGISTERS
  ========================================================================*/


/*=======================================================================
  KXTF9-SPECIFIC REGISTERS
  ========================================================================*/


/*=======================================================================
  KXTF9-SPECIFIC CONTROL BITS
  ========================================================================*/


#if 1
/*
 * Defines the threshold source for the accelerometer.
 */
typedef enum
{
	/// Indicates the accelerometer generated interrupt by exceeding the x threshold.
	NvOdmGyroAccelThresholdSource_X = 0,

	/// Indicates the accelerometer generated interrupt by exceeding the y threshold.
	NvOdmGyroAccelThresholdSource_Y,

	/// Indicates the accelerometer generated interrupt by exceeding the z threshold.
	NvOdmGyroAccelThresholdSource_Z,

	NvOdmGyroAccelThresholdSource_Force32 = 0x7FFFFFFF
} NvOdmGyroAccelThresholdSource;

// Timeout for I2C transaction.
enum { I2C_GYROACCEL_TRANSACTION_TIMEOUT = 10 }; // 1000 -> 10 (ms) // 2010.10.16 byoungwoo.yoon@lge.com
// Maximum number of packetsize supported by the I2C controller.
//enum { I2C_GYROACCEL_PACKET_SIZE = 8}; //original
enum { I2C_GYROACCEL_PACKET_SIZE = 200};

static NvU8 s_ReadBuffer[I2C_GYROACCEL_PACKET_SIZE];
static NvU8 s_WriteBuffer[I2C_GYROACCEL_PACKET_SIZE];

/*
 * Defines the way to read accelerometer registers.
 */
typedef NvBool (*GyroAccelRegsRead)(NvOdmGyroAccelHandle hDevice, NvU8 nRegOffset, NvU8* nData, NvU32 nLen);
/*
 * Defines the way to write accelerometer registers.
 */
typedef NvBool (*GyroAccelRegsWrite)(NvOdmGyroAccelHandle hDevice, NvU8 nRegOffset, NvU8* nData, NvU32 nLen);
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
#define GYROACCEL_CONTROL_REGS_MAX_LENGHT 100
/*
 * Max accelerometer callback functions number.
 */
#define GYROACCEL_CALLBACK_ARRAY_LENGTH   5

/*
*/
NvBool
	NvOdmGyroAccelGetParameter(NvOdmGyroAccelHandle hDevice, NvU8 attrib, NvU32* info);

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
	NvOdmGyroAccelSetParameter(NvOdmGyroAccelHandle hDevice, NvU8 attrib, NvU32 info);

//-----------------------------------------------------------------
//--------------------------New API--------------------------------
//-----------------------------------------------------------------

typedef struct NvOdmGyroAccelRec
{
	// Specifies accelerometer device address, for example, I2C write address.
	NvU8 nDevAddr;
	// Specifies the initial value that make accelerometer work, ACCELEROMETER_CONTROL_REGS_MAX_LENGHT is always 100.
	NvDevCtrlReg CtrlRegsList[GYROACCEL_CONTROL_REGS_MAX_LENGHT];
	// Specifies the initial CtrlRegsList length.
	NvU8 nLength;
	// Specifies the way to get accelerometer register information.
	GyroAccelRegsRead RegsRead;
	// Specifies the way to set accelerometer register information.
	GyroAccelRegsWrite RegsWrite;
	// Specifies I2C handle from the system.
	NvOdmServicesI2cHandle  hOdmI2C;
	// Interrupt pin to ap15.
	NvOdmServicesGpioHandle hGpioINT;
	NvOdmGpioPinHandle      hPinINT;
	NvU32 GPIOPortINT;
	NvU32 GPIOPinINT;
	NvOdmOsSemaphoreHandle SemaphoreForINT;
	NvOdmServicesGpioIntrHandle hGpioInterrupt;
	NvOdmGyroAccelIntType Data;
	NvOdmServicesPmuHandle hPmu;
	NvU32 VddId;
	NvU32 I2CChannelId;
	NvOdmGyroAccelCaps Caption;
	NvOdmGyroAccelPowerType PowerState;
	// In real case, when the board put in frontispiece, the value from z axis
	// should be g, but due to physical connect on different board, the axis
	// should be remapped to the correct one.
	NvOdmGyroAccelAxisType AxisXMapping;
	// If the physical direct is the same with our expection, the value
	// should be set to 1, or else the value should be -1.
	NvS32              AxisXDirection;
	NvOdmGyroAccelAxisType AxisYMapping;
	NvS32              AxisYDirection;
	NvOdmGyroAccelAxisType AxisZMapping;
	NvS32              AxisZDirection;
} NvOdmGyro;

int open_def_odm_accl(void);

#endif //Temp

#if defined(__cplusplus)
}
#endif
#endif
