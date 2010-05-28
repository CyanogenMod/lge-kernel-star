/*
 * sound/soc/tegra/tegra_transport.h
 *
 * ALSA SOC driver for NVIDIA Tegra SoCs
 *
 * Copyright (C) 2010 NVIDIA Corporation
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
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

#ifndef _TEGRA_TRANSPORT_H_
#define _TEGRA_TRANSPORT_H_

#include <linux/module.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>
#include <linux/kthread.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <linux/mutex.h>

#include <mach/nvrm_linux.h>
#include "nvrm_memmgr.h"
#include "nvassert.h"
#include "nvrm_transport.h"
#include "tegra_sndfx.h"


#define INIT_TIMEOUT 5000
#define PLAY_TIMEOUT 5000
#define REC_TIMEOUT 5000
#define WHISTLER_CODEC_ADDRESS 0x1a
#define WHISTLER_CODEC_BUS 0
#define NVALSA_BUFFER_COUNT 1
#define TEGRA_DEFAULT_BUFFER_SIZE 8192
#define NVALSA_INVALID_STATE -1
#define TEGRA_SAMPLE_RATES (SNDRV_PCM_RATE_8000_48000)
#define TEGRA_SAMPLE_FORMATS (SNDRV_PCM_FMTBIT_S8 | SNDRV_PCM_FMTBIT_U8 |\
		     SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE |\
		     SNDRV_PCM_FMTBIT_S32_LE | SNDRV_PCM_FMTBIT_S16_BE)

#define TEGRA_TRANSPORT_SEND_TIMEOUT           5000
#define TEGRA_TRANSPORT_CONNECT_TIMEOUT        60000
#define FXTRANSPORT_MSG_BUFFER_SIZE           256
#define FXTRANSPORT_MSG_BUFFER_PROPERTY_SIZE  FXTRANSPORT_MSG_BUFFER_SIZE - 32
#define FXTRANSPORT_MSG_RESULT_DATA_SIZE      FXTRANSPORT_MSG_BUFFER_SIZE - 32

typedef enum {
	NVFXTRANSPORT_MESSAGE_MIXER_OPEN = 0,
	NVFXTRANSPORT_MESSAGE_MIXER_CLOSE,
	NVFXTRANSPORT_MESSAGE_CREATE_OBJECT,
	NVFXTRANSPORT_MESSAGE_DESTROY_OBJECT,
	NVFXTRANSPORT_MESSAGE_MAP_BUFFER,
	NVFXTRANSPORT_MESSAGE_UNMAP_BUFFER,
	NVFXTRANSPORT_MESSAGE_GET_PROPERTY,
	NVFXTRANSPORT_MESSAGE_SET_PROPERTY,
	NVFXTRANSPORT_MESSAGE_STREAM_ADD_BUFFER,

	NVFXTRANSPORT_MESSAGE_Force32 = 0x7FFFFFFF

} NVFXTRANSPORT_MESSAGE;


typedef struct NvFxTransportMessageRec {
#define NVFXTRANSPORT_MESSAGE_VARS          \
	NVFXTRANSPORT_MESSAGE Message;      \
	NvOsSemaphoreHandle Semaphore;      \
	void* pPrivateData;                 \
	NvU32 SendAck

	NVFXTRANSPORT_MESSAGE_VARS;

} NvFxTransportMessage;

typedef struct NvFxTransportMessageResultRec {
#define NVFXTRANSPORT_MESSAGE_RESULT_VARS       \
	NVFXTRANSPORT_MESSAGE Message;          \
	NvOsSemaphoreHandle Semaphore;          \
	void* pPrivateData

	NVFXTRANSPORT_MESSAGE_RESULT_VARS;

}  NvFxTransportMessageResult;


typedef struct NvFxTransportMessageMixerOpenRec {
	NVFXTRANSPORT_MESSAGE_VARS;
	NvAudioFxMixerHandle* phMixer;

} NvFxTransportMessageMixerOpen;

typedef struct NvFxTransportMessageResultMixerOpenRec {
	NVFXTRANSPORT_MESSAGE_RESULT_VARS;
	NvAudioFxMixerHandle hMixer;
	NvAudioFxMixerHandle* phMixer;

} NvFxTransportMessageResultMixerOpen;


typedef struct NvFxTransportMessageMixerCloseRec {
	NVFXTRANSPORT_MESSAGE_VARS;
	NvAudioFxMixerHandle hMixer;

} NvFxTransportMessageMixerClose;

typedef NvFxTransportMessageResult NvFxTransportMessageResultMixerClose;


typedef struct NvFxTransportMessageCreateObjectRec {
	NVFXTRANSPORT_MESSAGE_VARS;
	NvAudioFxMixerHandle hMixer;
	NvObjectId Id;

	NvAudioFxObjectHandle* phObject;

} NvFxTransportMessageCreateObject;

typedef struct NvFxTransportMessageResultCreateObjectRec {
	NVFXTRANSPORT_MESSAGE_RESULT_VARS;
	NvAudioFxObjectHandle hObject;
	NvAudioFxObjectHandle* phObject;

}  NvFxTransportMessageResultCreateObject;


typedef struct NvFxTransportMessageDestroyObjectRec {
	NVFXTRANSPORT_MESSAGE_VARS;
	NvAudioFxObjectHandle hObject;

} NvFxTransportMessageDestroyObject;

typedef NvFxTransportMessageResult NvFxTransportMessageResultDestroyObject;


typedef struct NvFxTransportMessageMapBufferRec {
	NVFXTRANSPORT_MESSAGE_VARS;
	NvAudioFxMixerHandle hMixer;
	NvU32 NvRmMemHandleId;
	NvU32 Offset;
	NvU32 Size;

	NvAudioFxMixBufferHandle* phMixBuffer;

} NvFxTransportMessageMapBuffer;

typedef struct NvFxTransportMessageResultMapBufferRec {
	NVFXTRANSPORT_MESSAGE_RESULT_VARS;
	NvAudioFxMixBufferHandle hMixBuffer;
	NvAudioFxMixBufferHandle* phMixBuffer;

}  NvFxTransportMessageResultMapBuffer;


typedef struct NvFxTransportMessageUnmapBufferRec {
	NVFXTRANSPORT_MESSAGE_VARS;
	NvAudioFxMixBufferHandle hMixBuffer;

} NvFxTransportMessageUnmapBuffer;

typedef NvFxTransportMessageResult NvFxTransportMessageResultUnmapBuffer;


typedef struct NvFxTransportMessageGetPropertyRec {
	NVFXTRANSPORT_MESSAGE_VARS;
	NvAudioFxObjectHandle hObject;
	NvAudioFxProperty Property;
	NvU32 Size;

	void* pProperty;
	NvError* pReturnError;

} NvFxTransportMessageGetProperty;

typedef struct NvFxTransportMessageResultGetPropertyRec {
	NVFXTRANSPORT_MESSAGE_RESULT_VARS;
	NvU32 Size;
	NvError ReturnError;
	void* pProperty;
	NvError* pReturnError;
	NvU8 PropertyData[FXTRANSPORT_MSG_BUFFER_PROPERTY_SIZE];

}  NvFxTransportMessageResultGetProperty;


typedef struct NvFxTransportMessageSetPropertyRec {
	NVFXTRANSPORT_MESSAGE_VARS;
	NvAudioFxObjectHandle hObject;
	NvAudioFxProperty Property;
	NvU32 Size;
	NvError* pReturnError;
	NvU8 PropertyData[FXTRANSPORT_MSG_BUFFER_PROPERTY_SIZE];
} NvFxTransportMessageSetProperty;

typedef struct NvFxTransportMessageResultSetPropertyRec {
	NVFXTRANSPORT_MESSAGE_RESULT_VARS;
	NvError ReturnError;
	NvError* pReturnError;

}  NvFxTransportMessageResultSetProperty;


typedef struct NvFxTransportMessageStreamAddBufferRec {
	NVFXTRANSPORT_MESSAGE_VARS;
	NvAudioFxStreamHandle hStream;
	NvAudioFxBufferDescriptor Descriptor;

	NvError* pReturnError;

} NvFxTransportMessageStreamAddBuffer;

typedef struct NvFxTransportMessageResultStreamAddBufferRec {
	NVFXTRANSPORT_MESSAGE_RESULT_VARS;
	NvError ReturnError;
	NvError* pReturnError;

}  NvFxTransportMessageResultStreamAddBuffer;

typedef union NvFxTranspportMessageBuffer {
	NvFxTransportMessage Message;
	NvFxTransportMessageMixerOpen MixerOpen;
	NvFxTransportMessageMixerClose MixerClose;
	NvFxTransportMessageCreateObject CreateObject;
	NvFxTransportMessageDestroyObject DestroyObject;
	NvFxTransportMessageMapBuffer MapBuffer;
	NvFxTransportMessageUnmapBuffer UnmapBuffer;
	NvFxTransportMessageGetProperty GetProperty;
	NvFxTransportMessageSetProperty SetProperty;
	NvFxTransportMessageStreamAddBuffer StreamAddBuffer;

} NvFxTransportMessageBuffer;

typedef union NvFxTranspportMessageResultBuffer {
	NvFxTransportMessageResult Message;
	NvFxTransportMessageResultMixerOpen MixerOpen;
	NvFxTransportMessageResultMixerClose MixerClose;
	NvFxTransportMessageResultCreateObject CreateObject;
	NvFxTransportMessageResultDestroyObject DestroyObject;
	NvFxTransportMessageResultMapBuffer MapBuffer;
	NvFxTransportMessageResultUnmapBuffer UnmapBuffer;
	NvFxTransportMessageResultGetProperty GetProperty;
	NvFxTransportMessageResultSetProperty SetProperty;
	NvFxTransportMessageResultStreamAddBuffer StreamAddBuffer;

} NvFxTransportMessageResultBuffer;

typedef struct AlsaTransportRec {
	NvddkAudioFxFxnTable*   hFxnTable;
	NvOsThreadHandle        hServiceThread;
	NvOsSemaphoreHandle     hServiceSema;

	NvRmDeviceHandle        hRmDevice;
	NvRmTransportHandle     hRmTransport;

	volatile NvU32          TransportConnected;
	NvU32                   RefCount;
	NvU32                   ShutDown;
	spinlock_t              lock;

} AlsaTransport;


enum {
	/* Default Playback Path*/
	GlobalFx_DefaultPlaybackMix = 0,
	GlobalFx_DefaultPlaybackSplit,

	/* I2S Playback Path*/
	GlobalFx_I2sPlaybackMix,
	GlobalFx_I2sPlaybackVolume,
	GlobalFx_I2s,

	/* I2S2 Playback Path*/
	GlobalFx_I2s2PlaybackMix,
	GlobalFx_I2s2PlaybackVolume,
	GlobalFx_I2s2,

	/* SPDIF Playback Path*/
	GlobalFx_SpdifPlaybackMix,
	GlobalFx_SpdifPlaybackVolume,
	GlobalFx_Spdif,


	/* Default Record Path*/
	GlobalFx_DefaultRecordMix,
	GlobalFx_DefaultRecordSplit,

	/* I2S Record Path*/
	GlobalFx_I2sRecordVolume,
	GlobalFx_I2sRecordSplit,

	/* I2S2 Record Path*/
	GlobalFx_I2s2RecordVolume,
	GlobalFx_I2s2RecordSplit,

	/* SPDIF Record Path*/
	GlobalFx_SpdifRecordVolume,
	GlobalFx_SpdifRecordSplit,


	/* Loopbacks*/
	GlobalFx_I2sLoopbackSplit,
	GlobalFx_I2s2LoopbackSplit,
	GlobalFx_SpdifLoopbackSplit,


	/* Music Path*/
	GlobalFx_MusicMix,
	GlobalFx_MusicEq,
	GlobalFx_MusicDrc,
	GlobalFx_MusicSpreader,
	GlobalFx_MusicPeq,
	GlobalFx_MusicVolume,
	GlobalFx_MusicSplit,


	/* Phone Path*/
	GlobalFx_PhoneMix,
	GlobalFx_PhoneSplit,

	GlobalFx_Num

};

typedef struct GlobalFxListRec {
	/* Default Playback Path*/
	NvAudioFxObjectHandle hFx[GlobalFx_Num];
} GlobalFxList;


typedef struct FxNotifierRec {
	NvAudioFxNotifierHandle hNotifier;
	NvRmTransportHandle hTransport;
	NvOsSemaphoreHandle hTransportSemaphore;
	NvOsThreadHandle hTransportThread;

	NvU32 Exit;
	NvU32 Connected;
	NvU8 RcvMessageBuffer[256];

	NvAudioFxEvent Event;

} FxNotifier;

typedef enum {
	NvAudioInputSelect_Record = 0,
	NvAudioInputSelect_Loopback

} InputSelection;

typedef struct StandardPathRec {
	NvAudioFxObjectHandle Stream;
	NvAudioFxObjectHandle Src;
	NvAudioFxObjectHandle Convert;
	NvAudioFxObjectHandle Volume;
	NvAudioFxObjectHandle Resize;

	/*NvAudioFxVolumeDescriptor VolumeDesc;*/
	/*NvU32 VolumeRamping;*/
	/*StandardPosition StandardPosition;*/

} StandardPath;

typedef struct NvAudioBufferRec {
	NvRmMemHandle hRmMem;
	NvAudioFxMixBufferHandle hMixBuffer;
	void* pVirtAddr;
	NvU32 Size;
} NvAudioBuffer;

struct pcm_runtime_data {
	spinlock_t lock;
	struct task_struct *play_thread,*rec_thread;
	int timeout;
	int state;
	int stream;
	int shutdown_thrd;
	unsigned int audiofx_frames;
	struct completion thread_comp;
	wait_queue_head_t buf_wait;
	struct semaphore buf_done_sem;
	StandardPath* stdoutpath;
	StandardPath* stdinpath;
	u64 cur_pos;
	u64 last_pos;
	NvAudioFxMixBufferHandle mixer_buffer;
};

struct tegra_audio_data {
	NvddkAudioFxFxnTable xrt_fxn;
	NvAudioFxMixerHandle mixer_handle;
	FxNotifier m_FxNotifier;
	NvRmDeviceHandle m_hRm;
	unsigned int mapped_buf_size;
	NvAudioFxMixBufferHandle mixer_buffer[2];
	NvRmMemHandle mem_handle[2];
	struct mutex lock;
};


NvError tegra_audiofx_createfx(struct tegra_audio_data *audio_context);
void tegra_audiofx_destroyfx(struct tegra_audio_data *audio_context);
NvError tegra_audiofx_create_output(NvRmDeviceHandle,
				    NvAudioFxMixerHandle,
				    StandardPath*);
NvError tegra_audiofx_destroy_output(StandardPath* pPath);
NvError tegra_audiofx_create_input(NvRmDeviceHandle hRmDevice,
				NvAudioFxMixerHandle hMixer,
				StandardPath* pPath,
				InputSelection InputSelect);
NvError tegra_audiofx_destroy_input(StandardPath* pPath);
NvError tegra_transport_init(NvddkAudioFxFxnTable* FxTransportFxFxnTable);
void tegra_transport_deinit(void);

#endif
