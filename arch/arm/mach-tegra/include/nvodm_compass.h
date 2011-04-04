/*
 * Copyright (c) 2006-2009 NVIDIA Corporation.
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

/**
 * @file
 * <b>NVIDIA Tegra ODM Kit:
 *         Compass Interface</b> //di.yu@lge.com
 *
 * @b Description: Defines the ODM interface for compass devices.
 *
 */

#ifndef INCLUDED_NVODM_COMPASS_H
#define INCLUDED_NVODM_COMPASS_H

#if defined(__cplusplus)
extern "C"
{
#endif

#include "nvodm_services.h"
#include "nvassert.h"

/**
 * 
 * 
 */
typedef struct NvOdmCompassRec *NvOdmCompassDeviceHandle;

/**
 * Defines interrupt events that accelerometers may generate during
 * operation.
 */

typedef enum
{
    /// Indicates that no interrupt has been generated (this value is returned
    /// when interrupt time-outs occur).
    NvOdmCompassInt_None = 0,

    /// Indicates that an interrupt has been generated due to motion across
    /// any axis crossing the specified threshold level.
    NvOdmCompassInt_MotionThreshold,

    /// Indicates that an interrupt has been generated due to a swinging
    /// (forward and back motion) ocurring within the specified time threshold.
    NvOdmCompassInt_TapThreshold,

    /// Indicates that an interrupt has been generated due to detection of
    /// linear freefall motion.
    NvOdmCompassInt_Freefall,

    NvOdmCompassInt_Num,

    /// Ignore -- Forces compilers to make 32-bit enums.
    NvOdmCompassInt_Force32 = 0x7fffffffUL,
} NvOdmCompassIntType;

/**
 * Defines axis types for compass. Interrupts are trigger by the axis.
 * An interrupt is triggered for enabled interrupts whenever a forced value
 * on an axis is greater than the threshold.
 */
typedef enum {
    NvOdmCompassAxis_None = 0x0,
    NvOdmCompassAxis_X = 0x1,
    NvOdmCompassAxis_Y = 0x2,
    NvOdmCompassAxis_Z = 0x4,
    NvOdmCompassAxis_All = 0x7,
    NvOdmCompassAxis_Force32 = 0x7fffffffUL,
} NvOdmCompassAxisType;

/**
 * Defines the compass power state.
 */
typedef enum {
   /// Specifies the accelerometer is working normally -- sample rate is high.
   NvOdmCompassPower_Fullrun = 0,
   /// Specifies the accelerometer is working normally -- sample rate is lower
   /// than \c NvOdmAccelPower_Fullrun.
   NvOdmCompassPower_Low,
   /// Specifies the accelerometer is not working, but the power supply is there.
   NvOdmCompassPower_Standby,
   /// Specifies the accelerometer is not working, and there is no power supply
   /// to the device.
   NvOdmCompassPower_Off,
   NvOdmCompassPower_None,
   /// Ignore -- Forces compilers to make 32-bit enums.
   NvOdmCompassPower_Force32 =0x7fffffffUL,
} NvOdmCompassPowerType;

/**
 * Holds device-specific compass capabilities.
 */
typedef struct NvOdmCompassCapsRec
{
    ///  Holds the maximum force in g-force (\em g) registered by this
    ///  accelerometer.
    ///  The value is in increments of 1000. For example, when the maximum
    ///  force is 2 \em g, the value should return 2000.
    NvU32 MaxForceInGs;

    ///  Holds the size of the register for the g-force values in bits.
    ///  This is to specify the resolution of the force value range.
    NvU32 ForceResolution;

    ///  Holds the number of motion thresholds that clients may use to generate
    ///  interrupts. 0 indicates that no threshold motion interrupts
    ///  are supported.
    NvU32 NumMotionThresholds;

    ///  Holds the maximum amount of time in microseconds (Usecs) that may
    ///  be specified as the threshold for a tap-style interrupt. 0
    ///  indicates that tap interrupts are not supported by the accelerometer.
    NvU32 MaxTapTimeDeltaInUs;

    ///  Holds TRUE if the accelerometer can generate an interrupt when
    ///  linear free-fall motion is detected.
    NvBool SupportsFreefallInt;

    ///  Holds the maximum sample rate the accelerometer supports.
    NvU32 MaxSampleRate;

    ///  Holds the minimum sample rate the accelerometer supports.
    NvU32 MinSampleRate;
} NvOdmCompassCaps;

///  Opaque handle to an compass object.
typedef struct NvOdmCompassRec *NvOdmCompassHandle;


/**
 * Initializes the compass and allocates resources used by the ODM
 * adaptation driver.
 *
 * @return A handle to the accelerometer if initialization is successful, or
 *         NULL if unsuccessful or no accelerometer exists.
 */
NvBool
NvOdmCompassOpen(NvOdmCompassHandle* hDevice);

/**
 * Disables the compass and frees any resources used by the driver.
 *
 * @param hDevice The accelerometer handle.
 */
void
NvOdmCompassClose(NvOdmCompassHandle hDevice);

/**
 */
NvBool
NvOdmCompassSetIntForceThreshold(NvOdmCompassHandle  hDevice,
                               NvOdmCompassIntType IntType,
                               NvU32             IntNum,
                               NvU32             Threshold);

/**
 */
NvBool
NvOdmCompassSetIntTimeThreshold(NvOdmCompassHandle  hDevice,
                              NvOdmCompassIntType IntType,
                              NvU32             IntNum,
                              NvU32             Threshold);


/**
 * Enables/disables the specified interrupt source. If the interrupt
 * thresholds were not set prior to enabling the interrupt, the ODM-defined
 * default values are used. If enabling a previously-enabled interrupt,
 * or disabling a previously-disabled interrupt, this function returns
 * silently.
 *
 * @param hDevice The accelerometer handle.
 * @param IntType The type of interrupt being configured (::NvOdmAccelIntType).
 * @param IntAxis The axis interrupt type (::NvOdmAccelAxisType).
 * @param IntNum For accelerometers that support multiple interrupt thresholds
 *               (::NvOdmAccelerometerCaps), specifies which threshold to
 *               configure. If the accelerometer supports a single threshold for
 *               the specified interrupt type, this parameter should be 0.
 * @param Toggle NV_TRUE specifies to enable the interrupt source, NV_FALSE to
 *               disable.
 *
 * @return NV_TRUE if successful, or NV_FALSE otherwise.
 */
NvBool
NvOdmCompassSetIntEnable(NvOdmCompassHandle  hDevice,
                           NvOdmCompassIntType  IntType,
                           NvOdmCompassAxisType IntAxis,
                           NvU32              IntNum,
                           NvBool             Toggle);

/**
 * Waits for any enabled interrupt, and returns the type of interrupt to the
 * caller. If multiple interrupts occur simultaneously, returns each
 * separately.
 *
 */
void
NvOdmCompassWaitInt(NvOdmCompassHandle    hDevice,
                  NvOdmCompassIntType  *IntType,
                  NvOdmCompassAxisType *IntMotionAxis);

/**
 * Signals the waiting semaphore.
 *
 */
void
NvOdmCompassSignal(NvOdmCompassHandle hDevice);


/**
 * Returns the current compass data 
 * @return NV_TRUE if successful, or NV_FALSE otherwise.
 */
NvBool
NvOdmCompassGetMagnetic(NvOdmCompassHandle hDevice,
                          NvS32           *CompassX,
                          NvS32           *CompassY,
                          NvS32           *CompassZ);

/**
 * Gets the compass's character.
 */
 #if 0
NvOdmCompassCaps
NvOdmCompGetCaps(NvOdmCompassHandle hDevice);

#endif
/**
 * Sets the compass's power state.
 */
NvBool
NvOdmCompassSetPowerState(NvOdmCompassHandle hDevice, NvOdmCompassPowerType PowerState);


/**
 * Sets the compass's current sample rate state.
 */
NvBool
NvOdmCompassSetSampleRate(NvOdmCompassHandle hDevice, NvU32 SampleRate);

/**
 * Gets the compass's current sample rate state.
 */
NvBool
NvOdmCompassGetSampleRate(NvOdmCompassHandle hDevice, NvU32* pSampleRate);



NvBool 
NvOdmCompassSetParameter(NvOdmCompassHandle hDevice, NvU8 attrib, NvU32 info);

NvBool 
NvOdmCompassGetParameter(NvOdmCompassHandle hDevice, NvU8 attrib, NvU32* info);

NvBool NvCompassI2CGetRegs(NvOdmCompassHandle hDevice, NvU8 offset, NvU8* value, NvU32 len);
NvBool NvCompassI2CSetRegs(NvOdmCompassHandle hDevice, NvU8 offset, NvU8* value, NvU32 len);


#if defined(__cplusplus)
}
#endif
/** @} */
#endif // INCLUDED_NVODM_ACCELEROMETER_H
