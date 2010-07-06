/*
 * sound/soc/tegra/tegra_transport.c
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

#include "tegra_transport.h"
#include <linux/completion.h>

#define transport_send_message(msg_type)                                     \
	message.Semaphore = 0;                                               \
	message.SendAck = 1;                                                 \
	status = NvRmTransportSendMsg(atrans->hRmTransport,                  \
	                              &message,                              \
	                              sizeof(msg_type),                      \
	                              TEGRA_TRANSPORT_SEND_TIMEOUT);         \
	if (status != NvSuccess) {                                           \
		snd_printk(KERN_ERR "NvRmTransportSendMsg failed!\n");       \
		goto EXIT_WITH_ERROR;                                        \
	}                                                                    \
	wait_for_completion(&comp);                                          \


static AlsaTransport* atrans = 0;


static NvAudioFxMixerHandle tegra_transport_mixer_open(void)
{
	NvError status = NvSuccess;
	NvAudioFxMixerHandle hMixer = 0;
	NvFxTransportMessageMixerOpen message;
	struct completion comp;

	init_completion(&comp);
	message.Message = NVFXTRANSPORT_MESSAGE_MIXER_OPEN;
	message.pPrivateData = (void*)&comp;
	message.phMixer = &hMixer;

	transport_send_message(NvFxTransportMessageMixerOpen);

EXIT_WITH_ERROR:
	return hMixer;
}

static void tegra_transport_mixer_close(NvAudioFxMixerHandle hMixer)
{
	NvError status = NvSuccess;
	NvFxTransportMessageMixerClose message;
	struct completion comp;

	init_completion(&comp);
	if (hMixer == NULL) {
		snd_printk(KERN_ERR "NULL NvAudioFxMixerHandle!\n");
		goto EXIT_WITH_ERROR;
	}

	message.Message = NVFXTRANSPORT_MESSAGE_MIXER_CLOSE;
	message.pPrivateData = (void*)&comp;
	message.hMixer = hMixer;

	transport_send_message(NvFxTransportMessageMixerClose);

EXIT_WITH_ERROR:
	return;
}

static NvAudioFxObjectHandle tegra_transport_mixer_create_object(
						NvAudioFxMixerHandle hMixer,
						NvObjectId Id)
{
	NvError status = NvSuccess;
	NvAudioFxObjectHandle hObject = 0;
	NvFxTransportMessageCreateObject message;
	struct completion comp;

	init_completion(&comp);
	message.Message = NVFXTRANSPORT_MESSAGE_CREATE_OBJECT;
	message.pPrivateData = (void*)&comp;
	message.hMixer = hMixer;
	message.Id = Id;
	message.phObject = &hObject;

	transport_send_message(NvFxTransportMessageCreateObject);

EXIT_WITH_ERROR:
	return hObject;
}

static void tegra_transport_mixer_destroy_object(NvAudioFxObjectHandle hObject)
{
	NvError status = NvSuccess;
	NvFxTransportMessageDestroyObject message;
	struct completion comp;

	init_completion(&comp);
	message.Message = NVFXTRANSPORT_MESSAGE_DESTROY_OBJECT;
	message.pPrivateData = (void*)&comp;
	message.hObject = hObject;

	transport_send_message(NvFxTransportMessageDestroyObject);

EXIT_WITH_ERROR:
	return;
}

static NvAudioFxMixBufferHandle tegra_transport_mixer_map_buffer(
						NvAudioFxMixerHandle hMixer,
						NvU32 NvRmMemHandleId,
						NvU32 Offset,
						NvU32 Size)
{
	NvError status = NvSuccess;
	NvAudioFxMixBufferHandle mixer_buffer = 0;
	NvFxTransportMessageMapBuffer message;
	struct completion comp;

	init_completion(&comp);
	message.Message = NVFXTRANSPORT_MESSAGE_MAP_BUFFER;
	message.pPrivateData = (void*)&comp;
	message.hMixer = hMixer;
	message.NvRmMemHandleId = NvRmMemHandleId;
	message.Offset = Offset;
	message.Size = Size;
	message.phMixBuffer = &mixer_buffer;

	transport_send_message(NvFxTransportMessageMapBuffer);
EXIT_WITH_ERROR:
	return mixer_buffer;
}


static void tegra_transport_mixer_unmap_buffer(
					NvAudioFxMixBufferHandle mixer_buffer)
{
	NvError status = NvSuccess;
	NvFxTransportMessageUnmapBuffer message;
	struct completion comp;

	init_completion(&comp);
	message.Message = NVFXTRANSPORT_MESSAGE_UNMAP_BUFFER;
	message.pPrivateData = (void*)&comp;
	message.hMixBuffer = mixer_buffer;

	transport_send_message(NvFxTransportMessageUnmapBuffer);

EXIT_WITH_ERROR:
	return;
}

static NvError tegra_transport_get_property(NvAudioFxObjectHandle hObject,
					    NvAudioFxProperty Property,
					    NvU32 Size,
					    void* pProperty)
{
	NvError status = NvSuccess;
	NvError retStatus = NvSuccess;
	NvFxTransportMessageGetProperty message;
	struct completion comp;

	if (FXTRANSPORT_MSG_BUFFER_PROPERTY_SIZE < Size) {
		snd_printk(KERN_ERR "Property length too long!\n");
		goto EXIT_WITH_ERROR;
	}

	init_completion(&comp);
	message.Message = NVFXTRANSPORT_MESSAGE_GET_PROPERTY;
	message.pPrivateData = (void*)&comp;
	message.hObject = hObject;
	message.Property = Property;
	message.Size = Size;
	message.pProperty = pProperty;
	message.pReturnError = &retStatus;

	transport_send_message(NvFxTransportMessageGetProperty);
	goto EXIT;

EXIT_WITH_ERROR:

	retStatus = status;

EXIT:
	return retStatus;
}

static NvError tegra_transport_set_property(NvAudioFxObjectHandle hObject,
					    NvAudioFxProperty Property,
					    NvU32 Size,
					    void* pProperty)
{
	NvError status = NvSuccess;
	NvError retStatus = NvSuccess;
	NvFxTransportMessageSetProperty message;
	struct completion comp;

	if (FXTRANSPORT_MSG_BUFFER_PROPERTY_SIZE < Size) {
		snd_printk(KERN_ERR "Property length too long!\n");
		goto EXIT_WITH_ERROR;
	}

	init_completion(&comp);
	message.Message = NVFXTRANSPORT_MESSAGE_SET_PROPERTY;
	message.pPrivateData = (void*)&comp;
	message.hObject = hObject;
	message.Property = Property;
	message.Size = Size;
	NvOsMemcpy(message.PropertyData, pProperty, Size);
	message.pReturnError = &retStatus;

	transport_send_message(NvFxTransportMessageSetProperty);
	goto EXIT;

EXIT_WITH_ERROR:
	retStatus = status;

EXIT:
	return retStatus;
}

static NvError tegra_transport_stream_add_buffer(NvAudioFxStreamHandle hStream,
					NvAudioFxBufferDescriptor* pDescriptor)
{
	NvError status = NvSuccess;
	NvError retStatus;
	NvFxTransportMessageStreamAddBuffer message;
	struct completion comp;

	init_completion(&comp);
	message.Message = NVFXTRANSPORT_MESSAGE_STREAM_ADD_BUFFER;
	message.pPrivateData =  (void*)&comp;
	message.hStream = hStream;
	message.Descriptor = *pDescriptor;
	message.pReturnError = &retStatus;

	transport_send_message(NvFxTransportMessageStreamAddBuffer);
	goto EXIT;

EXIT_WITH_ERROR:
	retStatus = status;

EXIT:
	return retStatus;
}

static void AlsaTransportServiceThread(void *arg)
{
	NvError status = NvSuccess;
	static NvFxTransportMessageResultBuffer in;
	NvU32 messageSize = 0;
	int retry = 0;
#define transport_complete(comp)                                               \
	complete((struct completion*)comp);

	while (retry < 5 ) {
		status = NvRmTransportConnect(atrans->hRmTransport,
					TEGRA_TRANSPORT_CONNECT_TIMEOUT);
		if (status == NvSuccess)
			break;
		retry++;
	}

	if (status) {
		snd_printk(KERN_ERR "AlsaTransportServiceThread: Failed to \
			connect to remote end. Giving up ! \n");
		atrans->TransportConnected = 0;
		return;
	}

	atrans->TransportConnected = 1;

	while (!atrans->ShutDown) {
		NvOsSemaphoreWait(atrans->hServiceSema);
		if (atrans->ShutDown)
			break;

		status = NvRmTransportRecvMsg(atrans->hRmTransport,
					&in,
					sizeof(NvFxTransportMessageResultBuffer),
					&messageSize);
		if (status == NvSuccess) {
			switch (in.Message.Message) {
			case NVFXTRANSPORT_MESSAGE_MIXER_OPEN: {
				*in.MixerOpen.phMixer = in.MixerOpen.hMixer;
				transport_complete(in.MixerOpen.pPrivateData);
			}
			break;

			case NVFXTRANSPORT_MESSAGE_MIXER_CLOSE: {
				transport_complete(in.MixerClose.pPrivateData);
			}
			break;

			case NVFXTRANSPORT_MESSAGE_CREATE_OBJECT: {
				*in.CreateObject.phObject =
				        in.CreateObject.hObject;
				transport_complete(in.CreateObject.pPrivateData);
			}
			break;

			case NVFXTRANSPORT_MESSAGE_DESTROY_OBJECT: {
				transport_complete(in.DestroyObject.pPrivateData);
			}
			break;

			case NVFXTRANSPORT_MESSAGE_MAP_BUFFER: {
				*in.MapBuffer.phMixBuffer =
				        in.MapBuffer.hMixBuffer;
				transport_complete(in.MapBuffer.pPrivateData);
			}
			break;

			case NVFXTRANSPORT_MESSAGE_UNMAP_BUFFER: {
				transport_complete(in.UnmapBuffer.pPrivateData);
			}
			break;

			case NVFXTRANSPORT_MESSAGE_GET_PROPERTY: {
				*in.GetProperty.pReturnError =
				        in.GetProperty.ReturnError;
				if (in.GetProperty.ReturnError == NvSuccess) {
					NvOsMemcpy(in.GetProperty.pProperty,
					           in.GetProperty.PropertyData,
					           in.GetProperty.Size);
				}
				transport_complete(in.GetProperty.pPrivateData);
			}
			break;

			case NVFXTRANSPORT_MESSAGE_SET_PROPERTY: {
				*in.SetProperty.pReturnError =
				        in.SetProperty.ReturnError;
				transport_complete(in.SetProperty.pPrivateData);
			}
			break;

			case NVFXTRANSPORT_MESSAGE_STREAM_ADD_BUFFER: {
				*in.StreamAddBuffer.pReturnError =
				        in.StreamAddBuffer.ReturnError;
				transport_complete(in.StreamAddBuffer.pPrivateData);
			}
			break;

			default:
				break;
			}
		}
	}

}

NvError tegra_transport_init(NvddkAudioFxFxnTable* xrt_fxns)
{
	NvError status = NvSuccess;

	xrt_fxns->MixerOpen          = tegra_transport_mixer_open;
	xrt_fxns->MixerClose         = tegra_transport_mixer_close;
	xrt_fxns->MixerCreateObject  = tegra_transport_mixer_create_object;
	xrt_fxns->MixerDestroyObject = tegra_transport_mixer_destroy_object;
	xrt_fxns->MixerMapBuffer     = tegra_transport_mixer_map_buffer;
	xrt_fxns->MixerUnmapBuffer   = tegra_transport_mixer_unmap_buffer;
	xrt_fxns->StreamAddBuffer    = tegra_transport_stream_add_buffer;
	xrt_fxns->GetProperty        = tegra_transport_get_property;
	xrt_fxns->SetProperty        = tegra_transport_set_property;

	/* Check if the FX Transport is already open.*/
	if (atrans) {
		spin_lock(&atrans->lock);
		atrans->RefCount++;
		spin_unlock(&atrans->lock);
		goto EXIT;
	}
	/* Map a shared memory buffer.*/
	atrans = (AlsaTransport*)kzalloc(sizeof(AlsaTransport),GFP_KERNEL);
	if (!atrans) {
		snd_printk(KERN_ERR "AlsaTransportInit kalloc failed! \n");
		goto EXIT_WITH_ERROR;
	}

	spin_lock_init(&atrans->lock);
	memset(atrans, 0, sizeof(AlsaTransport));
	spin_lock(&atrans->lock);
	atrans->RefCount++;
	spin_unlock(&atrans->lock);

	atrans->hRmDevice = s_hRmGlobal;

	status = NvOsSemaphoreCreate(&atrans->hServiceSema, 0);
	if (status != NvSuccess) {
		snd_printk(KERN_ERR "NvOsSemaphoreCreate failed!\n");
		goto EXIT_WITH_ERROR;
	}

	status = NvRmTransportOpen(atrans->hRmDevice,
	                           "ALSA_TRANSPORT",
	                           atrans->hServiceSema,
	                           &atrans->hRmTransport);
	if (status != NvSuccess) {
		snd_printk(KERN_ERR "NvRmTransportOpen failed!\n");
		goto EXIT_WITH_ERROR;
	}

	status = NvOsThreadCreate(AlsaTransportServiceThread,
	                          NULL,
	                          &atrans->hServiceThread);
	if (status != NvSuccess) {
		snd_printk(KERN_ERR "NvOsThreadCreate failed!\n");
		goto EXIT_WITH_ERROR;
	}

	while (!atrans->TransportConnected) {
		NvOsThreadYield();
	}

	goto EXIT;

EXIT_WITH_ERROR:
	if (atrans) {
		tegra_transport_deinit();
	}

EXIT:
	return status;
}

void tegra_transport_deinit(void)
{
	if (!atrans)
		goto EXIT;

	spin_lock(&atrans->lock);
	atrans->RefCount--;

	if (atrans->RefCount > 0){
		spin_unlock(&atrans->lock);
		goto EXIT;
	}
	spin_unlock(&atrans->lock);

	atrans->ShutDown = 1;

	if (atrans->hRmTransport) {
		NvRmTransportClose(atrans->hRmTransport);
		atrans->hRmTransport = 0;
		atrans->TransportConnected = 0;
	}

	if (atrans->hServiceThread) {
		NvOsSemaphoreSignal(atrans->hServiceSema);
		NvOsThreadJoin(atrans->hServiceThread);
		atrans->hServiceThread = 0;
	}

	if (atrans->hServiceSema) {
		NvOsSemaphoreDestroy(atrans->hServiceSema);
		atrans->hServiceSema = 0;
	}
	atrans->hRmDevice = 0;
	kfree(atrans);
	atrans = 0;

EXIT:
	return;
}

int tegra_audiofx_init(struct tegra_audio_data* tegra_snd_cx)
{
	NvError e = NvSuccess;
	int ret = 0;
	NvAudioFxMessage message;

	if (!tegra_snd_cx->mixer_handle) {
		mutex_lock(&tegra_snd_cx->lock);
		e = tegra_transport_init(&tegra_snd_cx->xrt_fxn);
		mutex_unlock(&tegra_snd_cx->lock);

		if (e != NvSuccess) {
			snd_printk(KERN_ERR "tegra_transport_init failed \n");
			return -EFAULT;
		}

		tegra_snd_cx->mixer_handle =
					tegra_snd_cx->xrt_fxn.MixerOpen();

		if (!tegra_snd_cx->mixer_handle) {
			ret = -EFAULT;
			goto fail;
		}

		e = tegra_audiofx_createfx(tegra_snd_cx);
		if (e != NvSuccess) {
			snd_printk(KERN_ERR "tegra_audiofx_createfx failed \n");
			ret = -EFAULT;
			goto fail;
		}

		tegra_snd_cx->mixer_buffer[0] =
			tegra_snd_cx->xrt_fxn.MixerMapBuffer(
			tegra_snd_cx->mixer_handle,
			NvRmMemGetId(tegra_snd_cx->mem_handle[0]),
			0,
			tegra_snd_cx->mapped_buf_size);

		if (!tegra_snd_cx->mixer_buffer[0]) {
			snd_printk(KERN_ERR"TransportMixerMapBuffer failed!\n");
			ret = -EFAULT;
			goto fail;
		}

		tegra_snd_cx->mixer_buffer[1] =
			tegra_snd_cx->xrt_fxn.MixerMapBuffer(
			tegra_snd_cx->mixer_handle,
			NvRmMemGetId(tegra_snd_cx->mem_handle[1]),
			0,
			tegra_snd_cx->mapped_buf_size);

		if (!tegra_snd_cx->mixer_buffer[1]) {
			snd_printk(KERN_ERR"TransportMixerMapBuffer failed!\n");
			ret = -EFAULT;
			goto fail;
		}

		tegra_snd_cx->mvolume = tegra_snd_cx->xrt_fxn.MixerCreateObject(
						tegra_snd_cx->mixer_handle,
						NvAudioFxI2s1VolumeId);
		tegra_snd_cx->i2s1volume = NvAudioFxVolumeDefault;

		tegra_snd_cx->mroute = tegra_snd_cx->xrt_fxn.MixerCreateObject(
						tegra_snd_cx->mixer_handle,
						NvAudioFxSpdifId);
		tegra_snd_cx->spdif_plugin = 1;

		tegra_snd_cx->mi2s1 = tegra_snd_cx->xrt_fxn.MixerCreateObject(
						tegra_snd_cx->mixer_handle,
						NvAudioFxI2s1Id);

		memset(&message, 0, sizeof(NvAudioFxMessage));
		message.Event = NvAudioFxEventControlChange;
		message.hFx = (NvAudioFxHandle)tegra_snd_cx->mi2s1;
		message.pContext = tegra_snd_cx;

		e = tegra_snd_cx->xrt_fxn.SetProperty(
		    (NvAudioFxObjectHandle)tegra_snd_cx->m_FxNotifier.hNotifier,
		    NvAudioFxIoProperty_AddEvent,
		    sizeof(NvAudioFxMessage),
		    &message);

		if (e != NvSuccess) {
			snd_printk(KERN_ERR "TransportSetProperty failed\n");
			ret = -EFAULT;
			goto fail;
		}

		memset(&message, 0, sizeof(NvAudioFxMessage));
		message.Event = NvAudioFxEventControlChange;
		message.hFx = (NvAudioFxHandle)tegra_snd_cx->mroute;
		message.pContext = tegra_snd_cx;

		e = tegra_snd_cx->xrt_fxn.SetProperty(
		    (NvAudioFxObjectHandle)tegra_snd_cx->m_FxNotifier.hNotifier,
		    NvAudioFxIoProperty_AddEvent,
		    sizeof(NvAudioFxMessage),
		    &message);

		if (e != NvSuccess) {
			snd_printk(KERN_ERR "TransportSetProperty failed\n");
			ret = -EFAULT;
			goto fail;
		}

		tegra_snd_cx->m_FxNotifier.Event |= NvAudioFxEventControlChange;
	}

	return 0;
fail:
	snd_printk(KERN_ERR "tegra_audiofx_init failed \n");
	if (tegra_snd_cx->mixer_handle) {
		tegra_audiofx_destroyfx(tegra_snd_cx);

		if (tegra_snd_cx->mixer_handle) {
			tegra_snd_cx->xrt_fxn.MixerClose(
					tegra_snd_cx->mixer_handle);
		}
	}
	mutex_lock(&tegra_snd_cx->lock);
	tegra_transport_deinit();
	mutex_unlock(&tegra_snd_cx->lock);

	return ret;
}

static void tegra_audiofx_notifier_thread(void *arg)
{
	struct tegra_audio_data *audio_context = (struct tegra_audio_data *)arg;
	FxNotifier *m_FxNotifier = (FxNotifier*)&audio_context->m_FxNotifier;
	NvError e;
	int retry = 0;
	NvU32 messageSize;
	NvAudioFxMessage* message =
	        (NvAudioFxMessage*)m_FxNotifier->RcvMessageBuffer;

	while (retry < 5) {
		e = NvRmTransportConnect(m_FxNotifier->hTransport, 5000);
		if (e == NvSuccess)
			break;

		retry++;
	}
	if (e != NvSuccess) {
			snd_printk(KERN_ERR "NvRmTransportConnect failed!\n");
		m_FxNotifier->Connected = 0;
		goto EXIT;
	}

	m_FxNotifier->Connected = 1;
	while (1) {
		NvOsSemaphoreWait(m_FxNotifier->hTransportSemaphore);
		if (m_FxNotifier->Exit) {
			break;
		}

		e = NvRmTransportRecvMsg(m_FxNotifier->hTransport,
		                         message,
		                         256,
		                         &messageSize);
		if (e == NvSuccess) {
			switch (message->Event) {
			case NvAudioFxEventBufferDone:{
				NvAudioFxBufferDoneMessage* bdm =
				      (NvAudioFxBufferDoneMessage*)message;
				struct pcm_runtime_data* prtd =
				      (struct pcm_runtime_data*)bdm->m.pContext;

				 up(&prtd->buf_done_sem);
			}
			break;

			case NvAudioFxEventStateChange:{
				NvAudioFxStateChangeMessage* scm =
				      (NvAudioFxStateChangeMessage*)message;
				struct pcm_runtime_data* prtd =
				      (struct pcm_runtime_data*)scm->m.pContext;
				up(&prtd->stop_done_sem);
			}
			break;

			case NvAudioFxEventControlChange:{
				NvAudioFxControlChangeMessage* ccm =
					(NvAudioFxControlChangeMessage*)message;

				if (message->hFx ==
					(NvAudioFxHandle)audio_context->mi2s1)
				{
					if (ccm->Property == NvAudioFxIoProperty_OutputAvailable)
					{
						NvAudioFxIoDeviceControlChangeMessage* iccm =
							(NvAudioFxIoDeviceControlChangeMessage*)message;
						NvAudioFxIoDevice device_available = iccm->IoDevice;
						NvAudioFxIoDevice device_select = device_available;

						if (device_available &
							NvAudioFxIoDevice_HeadphoneOut)
						{
							device_select = NvAudioFxIoDevice_HeadphoneOut;
						}
						else if (device_available &
							NvAudioFxIoDevice_BuiltInSpeaker)
						{
							device_select = NvAudioFxIoDevice_BuiltInSpeaker;
						}

						audio_context->xrt_fxn.SetProperty(
							audio_context->mi2s1,
							NvAudioFxIoProperty_OutputSelect,
							sizeof(NvAudioFxIoDevice),
							&device_select);
					}

				}
				else if (message->hFx ==
					(NvAudioFxHandle)audio_context->mroute)
				{
					if (ccm->Property == NvAudioFxIoProperty_OutputAvailable)
					{
						NvAudioFxIoDeviceControlChangeMessage* iccm =
							(NvAudioFxIoDeviceControlChangeMessage*)message;
						NvAudioFxIoDevice device_available = iccm->IoDevice;
						NvAudioFxIoDevice device_select = 0;

						if ((device_available &
							NvAudioFxIoDevice_Aux) &&
							audio_context->spdif_plugin)
						{
							device_select = NvAudioFxIoDevice_Aux;
						}
						else if ((device_available &
							NvAudioFxIoDevice_BuiltInSpeaker) &&
							audio_context->spdif_plugin)
						{
							device_select = NvAudioFxIoDevice_BuiltInSpeaker;
						}

						audio_context->xrt_fxn.SetProperty(
							audio_context->mroute,
							NvAudioFxIoProperty_OutputSelect,
							sizeof(NvAudioFxIoDevice),
							&device_select);
					}
				}
			}
			break;

			default:
				snd_printk(KERN_ERR"Unhandled event\n");
				break;
			}
		}
	}

EXIT:
	return;
}

NvError tegra_audiofx_createfx(struct tegra_audio_data *audio_context)
{
	NvAudioFxMixerHandle m_hMixer =
	        (NvAudioFxMixerHandle)audio_context->mixer_handle;

	FxNotifier *m_FxNotifier = (FxNotifier*)&audio_context->m_FxNotifier;
	NvRmDeviceHandle m_hRm = (NvRmDeviceHandle)audio_context->m_hRm;
	NvError e = NvSuccess;
	NvAudioFxNotifierConnectionDescriptor connectionDesciptor;
	NvAudioFxMessage message;

	memset(&connectionDesciptor,
	           0,
	           sizeof(NvAudioFxNotifierConnectionDescriptor));
	memset(&message, 0, sizeof(NvAudioFxMessage));

	e = NvOsSemaphoreCreate(&m_FxNotifier->hTransportSemaphore, 0);
	if (e != NvSuccess) {
		snd_printk(KERN_ERR "NvOsSemaphoreCreate failed!\n");
		goto EXIT_WITH_ERROR;
	}

	m_FxNotifier->hNotifier =
	        (NvAudioFxNotifierHandle)tegra_transport_mixer_create_object(
	                        m_hMixer,
	                        NvAudioFxNotifierId);
	if (!m_FxNotifier->hNotifier) {
		snd_printk(KERN_ERR "transport_mixer_create_object failed!\n");
		goto EXIT_WITH_ERROR;
	}

	e = NvRmTransportOpen(m_hRm,
	                      0,
	                      m_FxNotifier->hTransportSemaphore,
	                      &m_FxNotifier->hTransport);
	if (e != NvSuccess) {
		snd_printk(KERN_ERR "NvRmTransportOpen failed!\n");
		goto EXIT_WITH_ERROR;
	}

	NvRmTransportGetPortName(m_FxNotifier->hTransport,
	                         (NvU8*)&connectionDesciptor.PortName,
	                         sizeof(NvU8) * 16);

	e = NvOsThreadCreate(tegra_audiofx_notifier_thread,
	                     audio_context,
	                     &m_FxNotifier->hTransportThread);
	if (e != NvSuccess) {
		snd_printk(KERN_ERR "NvOsThreadCreate failed!\n");
		goto EXIT_WITH_ERROR;
	}

	e = tegra_transport_set_property(
		    (NvAudioFxObjectHandle)m_FxNotifier->hNotifier,
		    NvAudioFxNotifierProperty_Connect,
		    sizeof(NvAudioFxNotifierConnectionDescriptor),
		    &connectionDesciptor);
	if (e != NvSuccess) {
		snd_printk(KERN_ERR "tegra_transport_set_property failed!\n");
		goto EXIT_WITH_ERROR;
	}

	goto EXIT;

EXIT_WITH_ERROR:
	tegra_audiofx_destroyfx(audio_context);

EXIT:
	return e;
}

void tegra_audiofx_destroyfx(struct tegra_audio_data *audio_context)
{
	FxNotifier *m_FxNotifier = (FxNotifier*)&audio_context->m_FxNotifier;

	if (m_FxNotifier->Connected) {
		m_FxNotifier->Exit = 1;
		NvOsSemaphoreSignal(m_FxNotifier->hTransportSemaphore);
		NvOsThreadJoin(m_FxNotifier->hTransportThread);
		m_FxNotifier->hTransportThread = 0;
		tegra_transport_set_property(
			(NvAudioFxObjectHandle)m_FxNotifier->hNotifier,
			NvAudioFxNotifierProperty_Disconnect,
			0,
			0);
	}

	if (m_FxNotifier->hTransport) {
		NvRmTransportClose(m_FxNotifier->hTransport);
		m_FxNotifier->hTransport = 0;
	}

	if (m_FxNotifier->hNotifier) {
		tegra_transport_mixer_destroy_object(
		        (NvAudioFxObjectHandle)m_FxNotifier->hNotifier);
		m_FxNotifier->hNotifier = 0;
	}

	if (m_FxNotifier->hTransportSemaphore) {
		NvOsSemaphoreDestroy(m_FxNotifier->hTransportSemaphore);
		m_FxNotifier->hTransportSemaphore = 0;
	}

	return;
}

#define audiofx_create_object(path_object, FxId)                          \
	path_object = tegra_transport_mixer_create_object(hMixer, FxId);  \
	if(!path_object) {                                                \
		snd_printk(KERN_ERR "audiofx_create_object failed!");     \
	}

#define audiofx_path_connect(path_object, sink_object)                     \
	connection.hSource = (NvAudioFxHandle)path_object,                     \
	connection.SourcePin = NvAudioFxSourcePin;                             \
	connection.SinkPin = NvAudioFxSinkPin;                                 \
	connection.hSink = (NvAudioFxHandle)sink_object;                       \
	e = tegra_transport_set_property(path_object,                          \
	                             NvAudioFxProperty_Attach,                 \
	                             sizeof(NvAudioFxConnectionDescriptor),    \
	                             &connection);                             \
	if(e != NvSuccess) {                                                   \
		snd_printk(KERN_ERR "audiofx_path_connect failed!");           \
		goto EXIT_WITH_ERROR;                                          \
	}

NvError tegra_audiofx_create_output(NvRmDeviceHandle hRmDevice,
                             NvAudioFxMixerHandle hMixer,
                             StandardPath* pPath)
{
	NvError e = NvSuccess;
	NvAudioFxConnectionDescriptor connection;

	/*	Standard Output
		[stream]->[SRC]->[Convert]->[Resize]->[Volume]->Default Output
	*/

	memset(pPath, 0, sizeof(StandardPath));

	audiofx_create_object(pPath->Stream, NvAudioFxStreamId);
	audiofx_create_object(pPath->Src, NvAudioFxSrcId);
	audiofx_create_object(pPath->Convert, NvAudioFxConvertId);
	audiofx_create_object(pPath->Resize, NvAudioFxResizeId);
	audiofx_create_object(pPath->Volume, NvAudioFxVolumeId);

	audiofx_path_connect(pPath->Stream, pPath->Src);
	audiofx_path_connect(pPath->Src, pPath->Convert);
	audiofx_path_connect(pPath->Convert, pPath->Resize);
	audiofx_path_connect(pPath->Resize, pPath->Volume);

	connection.hSource = (NvAudioFxHandle)pPath->Volume;
	connection.SourcePin = NvAudioFxSourcePin;
	connection.hSink = 0;
	connection.SinkPin = NvAudioFxSinkPin;
	e = tegra_transport_set_property(pPath->Volume,
	                             NvAudioFxProperty_Attach,
	                             sizeof(NvAudioFxConnectionDescriptor),
	                             &connection);
	if (e != NvSuccess) {
		snd_printk(KERN_ERR "tegra_transport_set_property failed!\n");
		goto EXIT_WITH_ERROR;
	}

	goto EXIT;

EXIT_WITH_ERROR:

	tegra_audiofx_destroy_output(pPath);

EXIT:

	return e;
}

NvError tegra_audiofx_destroy_output(StandardPath* pPath)
{
	if (pPath->Volume) {
		tegra_transport_mixer_destroy_object(pPath->Volume);
		pPath->Volume = 0;
	}

	if (pPath->Resize) {
		tegra_transport_mixer_destroy_object(pPath->Resize);
		pPath->Resize = 0;
	}

	if (pPath->Convert) {
		tegra_transport_mixer_destroy_object(pPath->Convert);
		pPath->Convert = 0;
	}

	if (pPath->Src) {
		tegra_transport_mixer_destroy_object(pPath->Src);
		pPath->Src = 0;
	}

	if (pPath->Stream) {
		tegra_transport_mixer_destroy_object(pPath->Stream);
		pPath->Stream = 0;
	}

	return NvSuccess;
}

NvError tegra_audiofx_create_input(NvRmDeviceHandle hRmDevice,
                            NvAudioFxMixerHandle hMixer,
                            StandardPath* pInput,
                            InputSelection InputSelect)
{
	NvError e = NvSuccess;
	NvAudioFxConnectionDescriptor connection;

	/*
	   Standard Input (record or loopback)

	   +--------+ (2) +--------+ (3)  +---------+ (4)  +--------+  (5)
	+--| Stream |<----| Resize |<-----| Convert |<-----|   SRC  |<--From I2S
	|  |        |     |        |<--+  | (opt.)  |      | (opt.) |
	|  +--------+     +--------+   |  +---------+      +--------+
	|           (1)                |
	+------------------------------+

	*/

	memset(pInput, 0, sizeof(StandardPath));

	audiofx_create_object(pInput->Stream, NvAudioFxStreamId);
	audiofx_create_object(pInput->Resize,NvAudioFxResizeId);
	audiofx_create_object(pInput->Src,NvAudioFxSrcId);
	audiofx_create_object(pInput->Convert,NvAudioFxConvertId);

	/* Wire 1 */
	connection.hSource = (NvAudioFxHandle)(pInput->Stream);
	connection.SourcePin = NvAudioFxSourcePin;
	connection.hSink = (NvAudioFxHandle)pInput->Resize;
	connection.SinkPin = NvAudioFxCopySinkPin;
	e = tegra_transport_set_property(pInput->Stream,
	                             NvAudioFxProperty_Attach,
	                             sizeof(NvAudioFxConnectionDescriptor),
	                             &connection);
	if (e != NvSuccess) {
		snd_printk(KERN_ERR "tegra_transport_set_property failed!\n");
		goto EXIT_WITH_ERROR;
	}

	audiofx_path_connect(pInput->Resize, pInput->Stream);
	audiofx_path_connect(pInput->Convert, pInput->Resize);
	audiofx_path_connect(pInput->Src, pInput->Convert);

	/* Wire 5 */
	connection.hSource = 0;
	connection.SourcePin = (InputSelect == NvAudioInputSelect_Record) ?
				NvAudioFxSourcePin :  NvAudioFxLoopbackPin;
	connection.hSink = (NvAudioFxHandle)pInput->Src;
	connection.SinkPin = NvAudioFxSinkPin;
	e = tegra_transport_set_property(0,
	                             NvAudioFxProperty_Attach,
	                             sizeof(NvAudioFxConnectionDescriptor),
	                             &connection);
	if (e != NvSuccess) {
		snd_printk(KERN_ERR "tegra_transport_set_property failed!\n");
		goto EXIT_WITH_ERROR;
	}

	goto EXIT;

EXIT_WITH_ERROR:

	tegra_audiofx_destroy_input(pInput);

EXIT:

	return e;
}


NvError tegra_audiofx_destroy_input(StandardPath* pInput)
{
	if (pInput->Src) {
		tegra_transport_mixer_destroy_object(pInput->Src);
		pInput->Src = 0;
	}

	if (pInput->Convert) {
		tegra_transport_mixer_destroy_object(pInput->Convert);
		pInput->Convert = 0;
	}

	if (pInput->Resize) {
		tegra_transport_mixer_destroy_object(pInput->Resize);
		pInput->Resize = 0;
	}

	if (pInput->Stream) {
		tegra_transport_mixer_destroy_object(pInput->Stream);
		pInput->Stream = 0;
	}

	return NvSuccess;
}

