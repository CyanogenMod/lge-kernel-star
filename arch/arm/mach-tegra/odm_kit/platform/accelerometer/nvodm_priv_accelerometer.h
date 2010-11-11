/*
 * Copyright (c) 2010 NVIDIA Corporation.
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

#ifndef INCLUDED_NVODM_PRIV_ACCELEROMETER_H
#define INCLUDED_NVODM_PRIV_ACCELEROMETER_H

#if defined(__cplusplus)
extern "C"
{
#endif

#include "nvodm_accelerometer.h"
#include "nvodm_services.h"
#include "nvodm_query_discovery.h"
#include "nvos.h"

#define NVODMACCELEROMETER_ENABLE_PRINTF 0

#if NVODMACCELEROMETER_ENABLE_PRINTF
    #define NVODMACCELEROMETER_PRINTF(x) \
    do { \
        NvOdmOsPrintf x; \
    } while (0)
#else
    #define NVODMACCELEROMETER_PRINTF(x)
#endif

// Set 1 to have display orientation aligning correctly in 3 orientations
// (0, 90 & 270 degrees) on Tango with (froyo + K32).
// set 0 to have acceleration values on different axes matching to android
// phones, but display orientation not changing properly.
#define AXES_MAPPING_FOR_PROPER_DISPLAY_ALIGNMENT 1

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
enum { I2C_ACCELRATOR_TRANSACTION_TIMEOUT = 1000 };
// Maximum number of packetsize supported by the I2C controller.
enum { I2C_ACCELRATOR_PACKET_SIZE = 8};

NvBool bma150_init(NvOdmAccelHandle* hDevice);
NvBool kxtf9_init(NvOdmAccelHandle* hDevice);

typedef NvBool (*NvOdmPrivAccelOpen)(NvOdmAccelHandle hDevice);

typedef void (*NvOdmPrivAccelClose)(NvOdmAccelHandle hDevice);

typedef NvBool
(*NvOdmPrivAccelSetIntForceThreshold)(NvOdmAccelHandle  hDevice,
                               NvOdmAccelIntType IntType,
                               NvU32             IntNum,
                               NvU32             Threshold);

typedef NvBool
(*NvOdmPrivAccelSetIntTimeThreshold)(NvOdmAccelHandle  hDevice,
                              NvOdmAccelIntType IntType,
                              NvU32             IntNum,
                              NvU32             Threshold);

typedef NvBool
(*NvOdmPrivAccelSetIntEnable)(NvOdmAccelHandle  hDevice,
                           NvOdmAccelIntType  IntType,
                           NvOdmAccelAxisType IntAxis,
                           NvU32              IntNum,
                           NvBool             Toggle);
typedef void
(*NvOdmPrivAccelWaitInt)(NvOdmAccelHandle    hDevice,
                  NvOdmAccelIntType  *IntType,
                  NvOdmAccelAxisType *IntMotionAxis,
                  NvOdmAccelAxisType *IntTapAxis);

typedef void (*NvOdmPrivAccelSignal)(NvOdmAccelHandle hDevice);

typedef NvBool (*NvOdmPrivAccelGetAcceleration)(NvOdmAccelHandle hDevice,
                          NvS32           *AccelX,
                          NvS32           *AccelY,
                          NvS32           *AccelZ);

typedef NvOdmAccelerometerCaps (*NvOdmPrivAccelGetCaps)(NvOdmAccelHandle hDevice);

typedef NvBool (*NvOdmPrivAccelSetPowerState)(NvOdmAccelHandle hDevice,
                NvOdmAccelPowerType PowerState);

typedef NvBool
(*NvOdmPrivAccelSetSampleRate)(NvOdmAccelHandle hDevice,
                             NvU32 SampleRate);

typedef NvBool
(*NvOdmPrivAccelGetSampleRate)(NvOdmAccelHandle hDevice, NvU32* pSampleRate);

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
 * Defines the way to read accelerometer registers.
 */
typedef NvBool
(*AccelerometerRegsRead)(
    NvOdmAccelHandle hDevice,
    NvU8 nRegOffset,
    NvU8* nData,
    NvU32 nLen);
/*
 * Defines the way to write accelerometer registers.
 */
typedef NvBool
(*AccelerometerRegsWrite)(
    NvOdmAccelHandle hDevice,
    NvU8 nRegOffset,
    NvU8* nData,
    NvU32 nLen);

typedef struct NvOdmAccelRec
{
    // Specifies use I2C or SPI to configure accelerometer registers.
    NvU8 nBusType;
    // Specifies accelerometer device address, for example, I2C write address.
    NvU8 nDevAddr;
    // Specifies the initial value that make accelerometer work,
    // ACCELEROMETER_CONTROL_REGS_MAX_LENGHT is always 100.
    NvDevCtrlReg CtrlRegsList[ACCELEROMETER_CONTROL_REGS_MAX_LENGHT];
    // Specifies the initial CtrlRegsList length.
    NvU8 nLength;
    // Specifies accelerometer chip ID.
    NvU8 nChipID;
    // Specifies the way to get accelerometer register information.
    AccelerometerRegsRead RegsRead;
    // Specifies the way to set accelerometer register information.
    AccelerometerRegsWrite RegsWrite;
    // various pointers to access accelerometer functions
    NvOdmPrivAccelOpen AccelOpen;
    NvOdmPrivAccelClose AccelClose;
    NvOdmPrivAccelSetIntForceThreshold AccelSetIntForceThreshold;
    NvOdmPrivAccelSetIntTimeThreshold AccelSetIntTimeThreshold;
    NvOdmPrivAccelSetIntEnable AccelSetIntEnable;
    NvOdmPrivAccelWaitInt AccelWaitInt;
    NvOdmPrivAccelSignal AccelSignal;
    NvOdmPrivAccelGetAcceleration AccelGetAcceleration;
    NvOdmPrivAccelGetCaps AccelGetCaps;
    NvOdmPrivAccelSetPowerState AccelSetPowerState;
    NvOdmPrivAccelSetSampleRate AccelSetSampleRate;
    NvOdmPrivAccelGetSampleRate AccelGetSampleRate;

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
/** @} */
#endif // INCLUDED_NVODM_PRIV_ACCELEROMETER_H
