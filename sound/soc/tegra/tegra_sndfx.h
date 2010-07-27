/*
 * Copyright (c) 2010 NVIDIA Corporation.
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

#ifndef INCLUDED_nvddk_audiofx_H
#define INCLUDED_nvddk_audiofx_H


#if defined(__cplusplus)
extern "C"
{
#endif

#include "nvrm_module.h"
#include "nvrm_memmgr.h"
#include "nvrm_transport.h"
#include "nvrm_init.h"

/** @file
 * @brief <b>NVIDIA Driver Development Kit: NvAudioFx APIs</b>
 *
 * @b Description: Declares Interface for NvAudioFx APIs.
 */

/**
 *  @brief API Object Handles.
 *
 *  NvAudioFxObjectHandle is the base handle for every type in the API.
 */

typedef struct NvAudioFxObjectRec *NvAudioFxObjectHandle;

typedef struct NvAudioFxRec *NvAudioFxHandle;

typedef struct NvAudioFxMixBufferRec *NvAudioFxMixBufferHandle;

typedef struct NvAudioFxMixerRec *NvAudioFxMixerHandle;

typedef struct NvAudioFxNotifierRec *NvAudioFxNotifierHandle;

typedef struct NvAudioFxStreamRec *NvAudioFxStreamHandle;

/**
 *  @brief Object IDs.
 */

typedef NvS32 NvObjectId;
#define NvObjectNullId (0x0)
#define NvAudioFxObjectId (0x10000000)
#define NvAudioFxId (0x11000000)
#define NvAudioFxIoId (0x11100000)
#define NvAudioFxPluginId (0x11200000)
#define NvAudioFxMixBufferId (0x12000000)
#define NvAudioFx3dGroupId (0x14000000)
#define NvAudioFxNotifierId (0x30000000)
#define NvAudioFxConvertId (0x11000001)
#define NvAudioFxDrcId (0x11000002)
#define NvAudioFxEqId (0x11000003)
#define NvAudioFxMixerId (0x11000004)
#define NvAudioFxMixId (0x11000005)
#define NvAudioFxPeqId (0x11000006)
#define NvAudioFxResizeId (0x11000007)
#define NvAudioFxSplitId (0x11000008)
#define NvAudioFxSpreaderId (0x11000009)
#define NvAudioFxSrcId (0x1100000a)
#define NvAudioFxSwitchId (0x1100000b)
#define NvAudioFxVolumeId (0x1100000c)
#define NvAudioFxStreamId (0x11100000)
#define NvAudioFxSpdifId (0x11100001)
#define NvAudioFxSilenceId (0x11100002)
#define NvAudioFxI2s1Id (0x11110000)
#define NvAudioFxI2s2Id (0x11110001)
#define NvAudioFxPlaybackMixId (0x11300000)
#define NvAudioFxPlaybackSplitId (0x11300001)
#define NvAudioFxRecordMixId (0x11300002)
#define NvAudioFxRecordSplitId (0x11300003)
#define NvAudioFxLoopbackMixId (0x1130000d)
#define NvAudioFxLoopbackSplitId (0x1130000e)
#define NvAudioFxSpdifPlaybackMixId (0x11300004)
#define NvAudioFxSpdifRecordSplitId (0x11300005)
#define NvAudioFxSpdifLoopbackSplitId (0x11300006)
#define NvAudioFxSpdifVolumeId (0x1130000c)
#define NvAudioFxMusicMixId (0x11300007)
#define NvAudioFxMusicSplitId (0x11300008)
#define NvAudioFxRingtoneMixId (0x11300009)
#define NvAudioFxMusicVolumeId (0x1130000a)
#define NvAudioFxRingtoneVolumeId (0x1130000b)
#define NvAudioFxI2s1PlaybackMixId (0x11310000)
#define NvAudioFxI2s1RecordSplitId (0x11310001)
#define NvAudioFxI2s1LoopbackSplitId (0x11310002)
#define NvAudioFxI2s1VolumeId (0x11310003)
#define NvAudioFxI2s2PlaybackMixId (0x11311000)
#define NvAudioFxI2s2RecordSplitId (0x11311001)
#define NvAudioFxI2s2LoopbackSplitId (0x11311002)
#define NvAudioFxI2s2VolumeId (0x11311003)

//
//                  Mixer
//                +------+
//                |      |
//                |  FX  |--@ ScratchSource ------------+
//                |      |                              |
//                +------+                              |
//
//                 Source                              Sink
//                +------+                           +------+
//   Copy Sink @--|      |--@ Loopback  Copy Sink @--|      |--@ Loopback
//                |  FX  |                           |  FX  |
//  ----> Sink @--|      |--@ Source ------> Sink @--|      |--@ Source ---->
//                +------+                           +------+
//

typedef NvS32 NvAudioFxPin;
#define NvAudioFxScratchSourcePin (-4)
#define NvAudioFxLoopbackPin (-3)
#define NvAudioFxCopySinkPin (-2)
#define NvAudioFxInvalidPin (-1)
#define NvAudioFxSinkPin (0)
#define NvAudioFxSourcePin (1)

typedef NvS32 NvAudioFxProperty;
#define NvAudioFxProperty_Attach (0x1000)
#define NvAudioFxProperty_Detach (0x1010)
#define NvAudioFxProperty_Format (0x1020)
#define NvAudioFxProperty_Method (0x1030)
#define NvAudioFxProperty_PowerState (0x1040)
#define NvAudioFxProperty_SampleRate (0x1050)
#define NvAudioFxProperty_State (0x1060)
#define NvAudioFxProperty_ProcessBufferSize (0x1070)
#define NvAudioFxPinProperty_Format (0x2000)
#define NvAudioFxDrcProperty_Drc (0x3000)
#define NvAudioFxEqProperty_Eq (0x4000)
#define NvAudioFxIoProperty_AllocChannel (0x4a01)
#define NvAudioFxIoProperty_InputAvailable (0x4a02)
#define NvAudioFxIoProperty_InputEnable (0x4a03)
#define NvAudioFxIoProperty_InputDisable (0x4a04)
#define NvAudioFxIoProperty_InputSelect (0x4a05)
#define NvAudioFxIoProperty_OutputAvailable (0x4a06)
#define NvAudioFxIoProperty_OutputEnable (0x4a07)
#define NvAudioFxIoProperty_OutputDisable (0x4a08)
#define NvAudioFxIoProperty_OutputSelect (0x4a09)
#define NvAudioFxIoProperty_IoDeviceVolume (0x4a0a)
#define NvAudioFxIoProperty_GenericOdmConfig (0x4a0b)
#define NvAudioFxIoProperty_AddEvent (0x5000)
#define NvAudioFxIoProperty_Position (0x5010)
#define NvAudioFxIoProperty_RemoveEvent (0x5020)
#define NvAudioFxIoProperty_SetMappedPositionBuffer (0x5030)
#define NvAudioFxMixerProperty_ModeAvailable (0x6001)
#define NvAudioFxMixerProperty_ModeDisable (0x6002)
#define NvAudioFxMixerProperty_ModeEnable (0x6003)
#define NvAudioFxMixerProperty_ModeSelect (0x6004)
#define NvAudioFxMixProperty_Headroom (0x7000)
#define NvAudioFxNotifierProperty_Connect (0x8000)
#define NvAudioFxNotifierProperty_Disconnect (0x8010)
#define NvAudioFxPeqProperty_Peq (0x9000)
#define NvAudioFxResizeProperty_OutputSize (0xa000)
#define NvAudioFxSpreaderProperty_Spreader (0xb000)
#define NvAudioFxSrcProperty_SampleRateShift (0xc000)
#define NvAudioFxVolumeProperty_Ramping (0xd000)
#define NvAudioFxVolumeProperty_Volume (0xd010)

// Description of the NvAudioFxProperty_Attach and
// NvAudioFxProperty_Detach properties.

typedef struct NvAudioFxConnectionDescriptorRec
{
    NvAudioFxHandle hSource;
    NvAudioFxPin SourcePin;
    NvAudioFxHandle hSink;
    NvAudioFxPin SinkPin;
} NvAudioFxConnectionDescriptor;

// Audio format information of the stream or buffer.

typedef struct NvAudioFxFormatRec
{
    NvU32 FormatTag;
    NvU32 SampleRate;
    NvU32 Channels;
    NvU32 BitsPerSample;
    NvU32 ChannelMask;
    NvU32 ValidBitsPerSample;
} NvAudioFxFormat;

// Description of the Mode property.

typedef NvS32 NvAudioFxMode;
#define NvAudioFxMode_Normal (0x0)
#define NvAudioFxMode_Bluetooth_Sco (0x1)
#define NvAudioFxMode_Ringtone (0x2)
#define NvAudioFxMode_InCall (0x4)
#define NvAudioFxMode_Radio (0x8)
#define NvAudioFxMode_All (0xffffffff)

// Description of the position property.

typedef NvU64 NvAudioFxPosition;

// Description of supported power states in AudioFx.

typedef NvS32 NvAudioFxPowerState;
#define NvAudioFxPowerState_Low (0x1000)
#define NvAudioFxPowerState_High (0x1010)
#define NvAudioFxPowerState_Full (0x1020)
#define NvAudioFxPowerState_Ready (0x1030)
#define NvAudioFxPowerState_Off (0x1040)

// Description of the supported priority states.

typedef NvS32 NvAudioFxPriority;
#define NvAudioFxPriority_Normal (0x1000)
#define NvAudioFxPriority_Medium (0x1010)
#define NvAudioFxPriority_High (0x1020)
#define NvAudioFxPriority_Critical (0x1030)

// Description of the NvAudioFxProperty_State property.

typedef NvS32 NvAudioFxState;
#define NvAudioFxState_Uninitialized (0x1000)
#define NvAudioFxState_Initialized (0x1010)
#define NvAudioFxState_Stop (0x1020)
#define NvAudioFxState_Run (0x1030)
#define NvAudioFxState_Pause (0x1040)
#define NvAudioFxState_Disable (0x1050)
#define NvAudioFxState_EndOfStream (0x1060)
#define NvAudioFxState_Locked (0x1070)

// Audio DRC information.

typedef struct NvAudioFxDrcDescriptorRec
{
    NvS32 EnableDrc;
    NvS32 NoiseGateThreshold;
    NvS32 LowerCompThreshold;
    NvS32 UpperCompThreshold;
    NvS32 ClippingThreshold;
} NvAudioFxDrcDescriptor;

// Audio EQ information.
#define NvAudioFxEqNumFilters (5)
#define NvAudioFxEqNumChannels (2)

// AudioFx Default Volume Settings
#define NvAudioFxVolumeDefault (256)
#define NvAudioFxVolumeMax (1024)

typedef struct NvAudioFxEqDescriptorRec
{
    NvS32 dBGain[NvAudioFxEqNumChannels][NvAudioFxEqNumFilters];

} NvAudioFxEqDescriptor;

// Audio Spreader information.

typedef struct NvAudioFxSpreaderDescriptorRec
{
    NvU32 SpeakerWidth;
} NvAudioFxSpreaderDescriptor;

typedef enum
{
    NvAudioFxIirFilter_Undefined,
    NvAudioFxIirFilter_Bandpass,
    NvAudioFxIirFilter_Highpass,
    NvAudioFxIirFilter_Lowpass,
    NvAudioFxIirFilter_Num,
    NvAudioFxIirFilter_Force32 = 0x7FFFFFFF
} NvAudioFxIirFilter;

// IO Devices

typedef NvS32 NvAudioFxIoDevice;

// Default is configurable based on the device.
#define NvAudioFxIoDevice_Default (0x0)
#define NvAudioFxIoDevice_All (0xffffffff)

// Inputs
#define NvAudioFxIoDevice_BuiltInMic (0x1)
#define NvAudioFxIoDevice_Mic (0x2)
#define NvAudioFxIoDevice_LineIn (0x4)

// Outputs
#define NvAudioFxIoDevice_BuiltInSpeaker (0x100)
#define NvAudioFxIoDevice_EarSpeaker (0x200)
#define NvAudioFxIoDevice_LineOut (0x400)
#define NvAudioFxIoDevice_HeadphoneOut (0x800)
#define NvAudioFxIoDevice_Bluetooth_A2dp (0x1000)

// Both
#define NvAudioFxIoDevice_Aux (0x10000)
#define NvAudioFxIoDevice_Phone (0x20000)
#define NvAudioFxIoDevice_Radio (0x40000)
#define NvAudioFxIoDevice_Bluetooth_Sco (0x80000)

typedef struct NvAudioFxPeqDescriptorRec
{
    NvU32 Enable;
    NvU32 FilterType[NvAudioFxIirFilter_Num];
    NvS32 CenterFrequency[NvAudioFxIirFilter_Num];
    NvS32 Bandwidth[NvAudioFxIirFilter_Num];
    NvS32 dBGain[NvAudioFxIirFilter_Num];

} NvAudioFxPeqDescriptor;

// Audio pin-specific format information of the stream or buffer.

typedef struct NvAudioFxPinFormatDescriptorRec
{
    NvAudioFxFormat Format;
    NvAudioFxPin Pin;
} NvAudioFxPinFormatDescriptor;

// Audio volume information of the stream or buffer.

typedef struct NvAudioFxVolumeDescriptorRec
{
    NvS32 LeftVolume;
    NvS32 RightVolume;
    NvU32 Mute;
} NvAudioFxVolumeDescriptor;

// Description of the IoDeviceVolume property.

typedef struct NvAudioFxIoDeviceVolumeDescriptorRec
{
    NvAudioFxIoDevice IoDevice;
    NvAudioFxVolumeDescriptor Volume;
} NvAudioFxIoDeviceVolumeDescriptor;

// Description of the NvAudioFxVoiceProperty_SetMappedPositionBuffer property.

typedef struct NvAudioFxMappedBufferDescriptorRec
{
    NvAudioFxMixBufferHandle hMixBuffer;
    NvU32 Offset;
} NvAudioFxMappedBufferDescriptor;

 /**
 * @brief Defines the structure for adding a buffer to a stream.
 */

typedef struct NvAudioFxBufferDescriptorRec
{

    // A pointer which may be used for already mapped buffers.
    void* pAddr;

    // A physical address for accessing the buffer.
    NvRmPhysAddr PhysicalAddr;

    // The MixBuffer handle returned from NvAudioFxMixerMapBuffer.
    NvAudioFxMixBufferHandle hMixBuffer;

    // Buffer offset in bytes.
    NvU32 Offset;

    // Buffer size in bytes.
    NvU32 Size;

    // Size of non-Zero Fill data in buffer, usually same as Size
    NvU32 ValidSize;

    // The buffer format information.
    NvAudioFxFormat Format;
} NvAudioFxBufferDescriptor;

// Description of the NvAudioFxNotifierProperty_Connect property.

typedef struct NvAudioFxNotifierConnectionDescriptorRec
{
    NvU8 PortName[16];
} NvAudioFxNotifierConnectionDescriptor;

// Description of the NvAudioFxI2sProperty_AllocChannel property.

typedef struct NvAudioFxIoChannelDescriptorRec
{
    NvAudioFxPin Pin;
    NvU32 Id;
} NvAudioFxIoChannelDescriptor;

// Buffer has a header of type NvAudioFxOdmConfigHeader followed by the data buffer.

typedef struct NvAudioFxOdmConfigHeaderRec
{
    NvU32 Size;
    NvError Result;
} NvAudioFxOdmConfigHeader;

// Description of the NvAudioFxIoProperty_GenericOdmConfig property.

typedef struct NvAudioFxOdmConfigDescriptorRec
{
    NvU32 hRmMemId;
    NvU32 Size;
    void* hSemaphore;
} NvAudioFxOdmConfigDescriptor;

// Description of the NvAudioFxProperty_AddEvent and
// NvAudioFxProperty_RemoveEvent properties.

typedef NvS32 NvAudioFxEvent;
#define NvAudioFxEventBufferDone (0x1)
#define NvAudioFxEventStateChange (0x2)
#define NvAudioFxEventFormatChange (0x4)
#define NvAudioFxEventEndOfStream (0x8)
#define NvAudioFxEventPowerStateChange (0x10)
#define NvAudioFxEventIoChange (0x20)
#define NvAudioFxEventControlChange (0x40)
#define NvAudioFxEventControlQuery (0x80)
#define NvAudioFxEventAll (0xffffffff)

typedef struct NvAudioFxMessageRec
{
    NvAudioFxEvent Event;
    NvAudioFxHandle hFx;
    void* pContext;
} NvAudioFxMessage;

typedef struct NvAudioFxBufferDoneMessageRec
{
    NvAudioFxMessage m;
    NvAudioFxPosition Position;
} NvAudioFxBufferDoneMessage;

typedef struct NvAudioFxControlChangeMessageRec
{
    NvAudioFxMessage m;
    NvAudioFxProperty Property;
} NvAudioFxControlChangeMessage;

typedef struct NvAudioFxIoDeviceControlChangeMessageRec
{
    NvAudioFxControlChangeMessage m;
    NvAudioFxIoDevice IoDevice;
} NvAudioFxIoDeviceControlChangeMessage;

typedef struct NvAudioFxIoDeviceVolumeControlChangeMessageRec
{
    NvAudioFxControlChangeMessage m;
    NvAudioFxIoDeviceVolumeDescriptor idv;
} NvAudioFxIoDeviceVolumeControlChangeMessage;

typedef struct NvAudioFxOdmConfigChangeMessageRec
{
    NvAudioFxControlChangeMessage m;
    NvAudioFxOdmConfigDescriptor OdmConfig;
} NvAudioFxOdmConfigChangeMessage;

typedef struct NvAudioFxModeControlChangeMessageRec
{
    NvAudioFxControlChangeMessage m;
    NvAudioFxMode Mode;
} NvAudioFxModeControlChangeMessage;

typedef struct NvAudioFxStateChangeMessageRec
{
    NvAudioFxMessage m;
    NvAudioFxState State;
} NvAudioFxStateChangeMessage;

typedef struct NvAudioFxFormatChangeMessageRec
{
    NvAudioFxMessage m;
    NvAudioFxFormat Format;
} NvAudioFxFormatChangeMessage;

typedef struct NvAudioFxPowerStateChangeMessageRec
{
    NvAudioFxMessage m;
    NvAudioFxPowerState PowerState;
} NvAudioFxPowerStateChangeMessage;

typedef struct NvAudioFxIoChangeMessageRec
{
    NvAudioFxMessage m;
    NvAudioFxProperty Property;
    NvAudioFxConnectionDescriptor Connection;
} NvAudioFxIoChangeMessage;

 /**
 * @brief Initializes and opens the AudioFX Mixer.
 *
 * @retval NvAudioFxMixerHandle A non-NULL value will be returned if the mixer
 * is successfully opened.
 */

 NvAudioFxMixerHandle NvddkAudioFxMixerOpen(
    void  );

 /**
 * @brief Closes the AudioFX Mixer. This function frees the resources associated
 * with the Mixer handle and cannot fail.
 *
 * @param hMixer A handle from NvAudioFxMixerOpen().  If hMixer is NULL or
 * invalid, this API does nothing.
 */

 void NvddkAudioFxMixerClose(
    NvAudioFxMixerHandle hMixer );

 /**
 * @brief Creates and initializes an AudioFX object.
 *
 * @param hMixer A handle from NvAudioFxMixerOpen().  If hMixer is NULL the
 * object will be created by the Global Mixer.  A NULL handle should only be
 * used from the driver and not a normal client application.
 * @param Id The ID of the object to create.
 *
 * @retval NvAudioFxObjectHandle A non-NULL value will be returned if the
 * object is successfully created.
 */

 NvAudioFxObjectHandle NvddkAudioFxMixerCreateObject(
    NvAudioFxMixerHandle hMixer,
    NvObjectId Id );

 /**
 * @brief Destroys the AudioFX object.
 *
 * @param hObject A handle from NvAudioFxMixerCreateObject.  If hObject is
 * NULL, this API does nothing.
 */

 void NvddkAudioFxMixerDestroyObject(
    NvAudioFxObjectHandle hObject );

 /**
 * @brief Maps a buffer to AudioFX address space.
 */

 NvAudioFxMixBufferHandle NvddkAudioFxMixerMapBuffer(
    NvAudioFxMixerHandle hMixer,
    NvU32 NvRmMemHandleId,
    NvU32 Offset,
    NvU32 Size );

 /**
 * @brief Unmaps a buffer from the AudioFX address space.
 */

 void NvddkAudioFxMixerUnmapBuffer(
    NvAudioFxMixBufferHandle hMixBuffer );

 /**
 * @brief Adds a buffer to a Stream object.
 *
 * @param hStream Handle to the Stream object to add the buffer.
 * @param pDescriptor Description of the buffer to add to the stream.
 *
 * @retval NvSuccess Indicates the operation succeeded.
 */

 NvError NvddkAudioFxStreamAddBuffer(
    NvAudioFxStreamHandle hStream,
    NvAudioFxBufferDescriptor * pDescriptor );

/**
 * @brief Get the property value from an AudioFX object.
 *
 * @param hObject Handle of the object to get the property value.
 * @param Property The property of interest.
 * @param Size Size of the descriptor.
 * @param pDescriptor Holds the value of the current property.
 *
 * @retval NvSuccess Indicates the operation succeeded.
 */

 NvError NvddkAudioFxGetProperty(
    NvAudioFxObjectHandle hObject,
    NvAudioFxProperty Property,
    NvU32 Size,
    void* pProperty );

/**
 * @brief Set a property value on an AudioFX object.
 *
 * @param hObject Handle of the object to set the property on.
 * @param Property The property of interest.
 * @param Size Size of the property descriptor.
 * @param pProperty Holds the value of the property to set.
 *
 * @retval NvSuccess Indicates the operation succeeded.
 */

 NvError NvddkAudioFxSetProperty(
    NvAudioFxObjectHandle hObject,
    NvAudioFxProperty Property,
    NvU32 Size,
    void* pProperty );

/**
 * @brief AVP API Function Table
 */

typedef struct NvddkAudioFxFxnTableRec
{
    NvAudioFxMixerHandle (*MixerOpen)(void);
    void (*MixerClose)(NvAudioFxMixerHandle hMixer);
    NvAudioFxObjectHandle (*MixerCreateObject)(NvAudioFxMixerHandle hMixer, NvObjectId Id);
    void (*MixerDestroyObject)(NvAudioFxObjectHandle hObject);
    NvAudioFxMixBufferHandle (*MixerMapBuffer)(NvAudioFxMixerHandle hMixer, NvU32 NvRmMemHandleId, NvU32 Offset, NvU32 Size);
    void (*MixerUnmapBuffer)(NvAudioFxMixBufferHandle hMixBuffer);
    NvError (*StreamAddBuffer)(NvAudioFxStreamHandle hStream, NvAudioFxBufferDescriptor* pDescriptor);
    NvError (*GetProperty)(NvAudioFxObjectHandle hObject, NvAudioFxProperty Property, NvU32 Size, void* pProperty);
    NvError (*SetProperty)(NvAudioFxObjectHandle hObject, NvAudioFxProperty Property, NvU32 Size, void* pProperty);

} NvddkAudioFxFxnTable;

static const NvU32 NvddkAudioFxFxnTableId = 0x6e766178;  // 'nvax'

typedef enum
{
    NvAudioFxIoctl_Generic   = 5020,
    NvAudioFxIoctl_ForceWord = 0x7FFFFFFF,

} NvAudioFxIoctl;

#if defined(__cplusplus)
}
#endif

#endif
