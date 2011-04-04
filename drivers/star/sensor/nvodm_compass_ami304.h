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

#ifndef INCLUDED_NVODM_COMPASS_AMI304_H
#define INCLUDED_NVODM_COMPASS_AMI304_H

#if defined(_cplusplus)
extern "C"
{
#endif

#include <linux/ioctl.h>  //ADD
#include "nvodm_services.h"
#include "nvodm_compass.h"

/*=====================================================================
  AICHI STEEL COMMPASS AMI304 REGISTER MAP
  =======================================================================*/
#define AMI304_I2C_SLAVE_ADDR 		0x0E//new Addr=0x0E(Low), old Addr=0x0F(High)

/*=======================================================================
  AMI304 Internal Register Address
  ========================================================================*/
#define AMI304_REG_CTRL1			0x1B
#define AMI304_REG_CTRL2			0x1C
#define AMI304_REG_CTRL3			0x1D
#define AMI304_REG_DATAXH			0x10
#define AMI304_REG_DATAXL			0x11
#define AMI304_REG_DATAYH			0x12
#define AMI304_REG_DATAYL			0x13
#define AMI304_REG_DATAZH			0x14
#define AMI304_REG_DATAZL			0x15
#define AMI304_REG_WIA				0x0F

/*=======================================================================
  AMI304 CONTROL BITS
  ========================================================================*/
#define AMI304_CTRL1_PC1			0x80
#define AMI304_CTRL1_FS1_NORMAL			0x00 //Normal
#define AMI304_CTRL1_FS1_FORCE			0x02 //Force
#define AMI304_CTRL1_ODR1			0x10 //20SPS(20HZ)
#define AMI304_CTRL2_DREN			0x08
#define AMI304_CTRL2_DREN_DISABLE	0x00 //diyu@lge.com
#define AMI304_CTRL2_DRP			0x04
#define AMI304_CTRL3_NOFORCE_BIT		0x00
#define AMI304_CTRL3_FORCE_BIT			0x40
#define AMI304_CTRL3_B0_LO_CLR			0x00

/*=======================================================================
  IOCTLs for ami304 misc. device library
  ========================================================================*/
#define AMI304IO						   0x83
#define AMI304_IOCTL_INIT                  _IO(AMI304IO, 0x01)
#define AMI304_IOCTL_READ_CHIPINFO         _IOR(AMI304IO, 0x02, int)
#define AMI304_IOCTL_READ_SENSORDATA       _IOR(AMI304IO, 0x03, int)
#define AMI304_IOCTL_READ_POSTUREDATA      _IOR(AMI304IO, 0x04, int)
#define AMI304_IOCTL_READ_CALIDATA         _IOR(AMI304IO, 0x05, int)
#define AMI304_IOCTL_READ_CONTROL          _IOR(AMI304IO, 0x06, int)
#define AMI304_IOCTL_SET_CONTROL           _IOW(AMI304IO, 0x07, int)
#define AMI304_IOCTL_SET_MODE              _IOW(AMI304IO, 0x08, int)

/*=======================================================================
  IOCTLs for AMI304 middleware misc. device library
  ========================================================================*/
#define AMI304MIDIO						   0x84
#define AMI304MID_IOCTL_GET_SENSORDATA     _IOR(AMI304MIDIO, 0x01, int)
#define AMI304MID_IOCTL_SET_POSTURE        _IOW(AMI304MIDIO, 0x02, int)
#define AMI304MID_IOCTL_SET_CALIDATA       _IOW(AMI304MIDIO, 0x03, int)
#define AMI304MID_IOCTL_SET_CONTROL        _IOW(AMI304MIDIO, 0x04, int)
#define AMI304MID_IOCTL_GET_CONTROL        _IOR(AMI304MIDIO, 0x05, int)
#define AMI304MID_IOCTL_SET_MODE           _IOW(AMI304MIDIO, 0x06, int)

/*======================================================================
  COMMON REGISTERS
  ========================================================================*/
#define AMI304_I2C_WHO_AM_I			AMI304_I2C_SLAVE_ADDR
#define AMI304_I2C_XOUT_L			AMI304_REG_DATAXL
#define AMI304_I2C_XOUT_H			AMI304_REG_DATAXH
#define AMI304_I2C_YOUT_L			AMI304_REG_DATAYL
#define AMI304_I2C_YOUT_H			AMI304_REG_DATAYH
#define AMI304_I2C_ZOUT_L			AMI304_REG_DATAZL
#define AMI304_I2C_ZOUT_H			AMI304_REG_DATAZH
#define AMI304_I2C_INS1				0x16  /**/
#define AMI304_I2C_STA1				0x18  /**/
#define AMI304_I2C_INL				0x1A
#define AMI304_I2C_CNTL1			AMI304_REG_CTRL1  /**/
#define AMI304_I2C_CNTL2			AMI304_REG_CTRL2  /**/
#define AMI304_I2C_CNTL3			AMI304_REG_CTRL3  /**/
#define AMI304_I2C_CNTL4L			0x5C  /**/
#define AMI304_I2C_CNTL4H			0x5C  /**/
#define AMI304_I2C_CNTL5			0x40  /**/
#define AMI304_I2C_INC1				0x1E  /**/
#define AMI304_I2C_B0XL				0x20  /**/
#define AMI304_I2C_B0XH				0x21  /**/
#define AMI304_I2C_B0YL				0x22  /**/
#define AMI304_I2C_B0YH				0x23  /**/
#define AMI304_I2C_B0ZL				0x24  /**/
#define AMI304_I2C_B0ZH				0x25  /**/
#define AMI304_I2C_ITHR1			0x26  /**/
#define AMI304_I2C_PRET				0x27  /**/
#define AMI304_I2C_OFFXL			0x6C  /*X Offset registers Low byte*/
#define AMI304_I2C_OFFXH			0x6D  /*X Offset registers High byte*/
#define AMI304_I2C_OFFYL			0x72  /*Y Offset registers Low byte*/
#define AMI304_I2C_OFFYH			0x73  /*Y Offset registers High byte*/
#define AMI304_I2C_OFFZL			0x78  /*Z Offset registers Low byte*/
#define AMI304_I2C_OFFZH			0x79  /*Z Offset registers High byte*/
#define AMI304_I2C_DELAYX			0x68  /**/
#define AMI304_I2C_DELAYY			0x6E  /**/
#define AMI304_I2C_DELAYZ			0x74  /**/
#define AMI304_I2C_VERL				0xE8  /*Firmware version*/
#define AMI304_I2C_VERH				0xE9  /*Firmware version*/
#define AMI304_I2C_SNL				0xEA  /*Serial Number Low byte*/
#define AMI304_I2C_SNH				0xEB  /*Serial Number High byte*/

/*
 * Defines the threshold source for the accelerometer.
 */
typedef enum
{
	/// Indicates the accelerometer generated interrupt by exceeding the x threshold.
	NvOdmCompassThresholdSource_X = 0,

	/// Indicates the accelerometer generated interrupt by exceeding the y threshold.
	NvOdmCompassThresholdSource_Y,

	/// Indicates the accelerometer generated interrupt by exceeding the z threshold.
	NvOdmCompassThresholdSource_Z,

	NvOdmCompassThresholdSource_Force32 = 0x7FFFFFFF
} NvOdmCompassThresholdSource;

// Timeout for I2C transaction.
//jinsoo.oh@lge.com 0608 audio test
//enum { I2C_COMPASS_TRANSACTION_TIMEOUT = 1000 };
enum { I2C_COMPASS_TRANSACTION_TIMEOUT = 10 };	// 500 -> 10 (ms) // 2010.10.16 byoungwoo.yoon@lge.com
// Maximum number of packetsize supported by the I2C controller.
enum { I2C_COMPASS_PACKET_SIZE = 8};
static NvU8 s_ReadBuffer[I2C_COMPASS_PACKET_SIZE];
static NvU8 s_WriteBuffer[I2C_COMPASS_PACKET_SIZE];

// Module debug: 0=disable, 1=enable
#define NVODMCOMPASS_ENABLE_PRINTF (1)

#if NVODMACCELEROMETER_ENABLE_PRINTF
#define NVODMCOMPASS_PRINTF   NvOdmOsDebugPrintf
#else
#define NVODMCOMPASS_PRINTF   (void)
#endif
/*
 * Defines the way to read accelerometer registers.
 */
typedef NvBool (*CompassRegsRead)(NvOdmCompassHandle hDevice, NvU8 nRegOffset, NvU8* nData, NvU32 nLen);
/*
 * Defines the way to write accelerometer registers.
 */
typedef NvBool (*CompassRegsWrite)(NvOdmCompassHandle hDevice, NvU8 nRegOffset, NvU8* nData, NvU32 nLen);
/*
 * Holds register address and value pairs.
 */
#if 0 /*redefinition*/
typedef struct NvDevCtrlRegRec {
	/// Holds the register offset.
	NvU8 RegAddr;
	/// Holds the value programmed into the upper address.
	NvU8 RegValue;
} NvDevCtrlReg;
#endif
/*
 * Max COMPASS registers number.
 */
//#define ACCELEROMETER_CONTROL_REGS_MAX_LENGHT 100
#define COMPASS_CONTROL_REGS_MAX_LENGHT 100 /*Check Please, by diyu*/

/*
 * Max COMPASS callback functions number.
 */
//#define ACCELEROMETER_CALLBACK_ARRAY_LENGTH   5
#define COMPASS_CALLBACK_ARRAY_LENGTH   5


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
/*NvBool
  NvOdmAccelerometerGetParameter(NvOdmAccelHandle hDevice, NvU8 attrib, NvU32* info);*/
NvBool
	NvOdmCompassGetParameter(NvOdmCompassHandle hDevice, NvU8 attrib, NvU32* info);

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

/*NvBool
NvOdmAccelerometerSetParameter(NvOdmAccelHandle hDevice, NvU8 attrib, NvU32 info);*/

NvBool
	NvOdmCompassSetParameter(NvOdmCompassHandle hDevice, NvU8 attrib, NvU32 info);

//-----------------------------------------------------------------
//--------------------------New API--------------------------------
//-----------------------------------------------------------------
//typedef struct NvOdmAccelRec

typedef struct NvOdmCompassRec
{
	// Specifies accelerometer device address, for example, I2C write address.
	NvU8 nDevAddr;
	// Specifies the initial value that make accelerometer work, ACCELEROMETER_CONTROL_REGS_MAX_LENGHT is always 100.
	NvDevCtrlReg CtrlRegsList[COMPASS_CONTROL_REGS_MAX_LENGHT];
	// Specifies the initial CtrlRegsList length.
	NvU8 nLength;
	// Specifies the way to get accelerometer register information.
	CompassRegsRead RegsRead;
	// Specifies the way to set accelerometer register information.
	CompassRegsWrite RegsWrite;
	// Specifies I2C handle from the system.
	NvOdmServicesI2cHandle  hOdmI2C;
	// Interrupt pin to ap15.
	NvOdmServicesGpioHandle hGpioINT;
	NvOdmGpioPinHandle      hPinINT;
	NvU32 GPIOPortINT;
	NvU32 GPIOPinINT;
	NvOdmOsSemaphoreHandle SemaphoreForINT;
	NvOdmServicesGpioIntrHandle hGpioInterrupt;
	NvOdmCompassIntType Data;
	NvOdmServicesPmuHandle hPmu;
	NvU32 VddId;
	NvU32 I2CChannelId;
	NvOdmCompassCaps Caption;
	NvOdmCompassPowerType PowerState;
	// In real case, when the board put in frontispiece, the value from z axis
	// should be g, but due to physical connect on different board, the axis
	// should be remapped to the correct one.
	NvOdmCompassAxisType AxisXMapping;
	// If the physical direct is the same with our expection, the value
	// should be set to 1, or else the value should be -1.
	NvS32              AxisXDirection;
	NvOdmCompassAxisType AxisYMapping;
	NvS32              AxisYDirection;
	NvOdmCompassAxisType AxisZMapping;
	NvS32              AxisZDirection;
} NvOdmCompass;

#if defined(__cplusplus)
}
#endif
#endif
