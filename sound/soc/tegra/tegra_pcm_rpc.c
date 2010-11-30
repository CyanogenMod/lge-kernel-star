/*
 * sound/soc/tegra/tegra_pcm_rpc.c
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

struct tegra_audio_data* tegra_snd_cx[2];

static const struct snd_pcm_hardware tegra_pcm_hardware = {
	.info = SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_PAUSE |\
	        SNDRV_PCM_INFO_RESUME | SNDRV_PCM_INFO_MMAP |\
	        SNDRV_PCM_INFO_MMAP_VALID,
	.rates = TEGRA_SAMPLE_RATES,
	.formats = TEGRA_SAMPLE_FORMATS,
	.channels_min = 1,
	.channels_max = 2,
	.buffer_bytes_max = 32*1024,
	.period_bytes_min = TEGRA_DEFAULT_BUFFER_SIZE,
	.period_bytes_max = TEGRA_DEFAULT_BUFFER_SIZE,
	.periods_min = 4,
	.periods_max = 8,
	.fifo_size = 8,
};

static int tegra_pcm_hw_params(struct snd_pcm_substream *substream,
			       struct snd_pcm_hw_params *params)
{
	snd_pcm_set_runtime_buffer(substream, &substream->dma_buffer);
	return 0;
}

static int tegra_pcm_hw_free(struct snd_pcm_substream *substream)
{
	snd_pcm_set_runtime_buffer(substream, NULL);
	return 0;
}

static int tegra_pcm_prepare(struct snd_pcm_substream *substream)
{
	return 0;
}

static inline NvAudioFxState play_state(struct tegra_audio_data *ptscx,
					struct pcm_runtime_data *prtd,
					NvAudioFxState cur_state)
{
	NvAudioFxState state = cur_state;

	switch (prtd->state) {
	case SNDRV_PCM_TRIGGER_START:
		if (state != NvAudioFxState_Run) {
			state = NvAudioFxState_Run;
			ptscx->xrt_fxn.SetProperty(
					 prtd->stdoutpath->Stream,
					 NvAudioFxProperty_State,
					 sizeof(NvAudioFxState),
					 &state);
		}
		break;
	case SNDRV_PCM_TRIGGER_STOP:
		if (state != NvAudioFxState_Stop) {
			state = NvAudioFxState_Stop;
			ptscx->xrt_fxn.SetProperty(
					 prtd->stdoutpath->Stream,
					 NvAudioFxProperty_State,
					 sizeof(NvAudioFxState),
					 &state);
			down(&prtd->stop_done_sem);
		}
		break;
	default:
		;
	}

	return state;
}

static inline int queue_next_buffer(void *arg, int cur_offset)
{
	struct snd_pcm_substream *substream = (struct snd_pcm_substream *)arg;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct pcm_runtime_data *prtd = substream->runtime->private_data;
	struct snd_pcm *pcm = substream->pcm;
	struct tegra_audio_data *ptscx = tegra_snd_cx[pcm->device];
	int offset = cur_offset;
	int size, rtbuffersize;
	NvAudioFxBufferDescriptor abd;

	rtbuffersize = frames_to_bytes(runtime, runtime->buffer_size);
	memset(&abd, 0, sizeof(NvAudioFxBufferDescriptor));
	size = TEGRA_DEFAULT_BUFFER_SIZE;
	if ((offset + size) > rtbuffersize) {
		size = rtbuffersize - offset;
	}

	abd.hMixBuffer = prtd->mixer_buffer;
	abd.Offset = offset;
	abd.Size = size;
	abd.Format.FormatTag = 1;
	abd.Format.SampleRate = runtime->rate;
	abd.Format.BitsPerSample = runtime->sample_bits;
	abd.Format.Channels = runtime->channels;
	abd.Format.ChannelMask = 0;
	abd.Format.ValidBitsPerSample = 0;

	ptscx->xrt_fxn.StreamAddBuffer(
		(NvAudioFxStreamHandle)prtd->stdoutpath->Stream,
		&abd);

	offset += size;
	if (offset >= rtbuffersize)
		offset =0;

	prtd->audiofx_frames += bytes_to_frames(runtime, size);

	return offset;
}

static inline void play_buffer_done (void *arg)
{
	struct snd_pcm_substream *substream = (struct snd_pcm_substream *)arg;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct pcm_runtime_data *prtd = substream->runtime->private_data;
	int size, period_offset, rtbuffersize;

	rtbuffersize = frames_to_bytes(runtime, runtime->buffer_size);
	down(&prtd->buf_done_sem);

	if ((frames_to_bytes(runtime, prtd->cur_pos) +
		TEGRA_DEFAULT_BUFFER_SIZE) > rtbuffersize) {
		size = rtbuffersize -
			frames_to_bytes(runtime, prtd->cur_pos);
	} else {
		size = TEGRA_DEFAULT_BUFFER_SIZE;
	}

	prtd->cur_pos += bytes_to_frames(runtime, size);

	if (prtd->cur_pos < prtd->last_pos) {
		period_offset = (runtime->buffer_size +
				prtd->cur_pos) - prtd->last_pos;
	} else {
		period_offset = prtd->cur_pos - prtd->last_pos;
	}

	if (period_offset >= runtime->period_size) {
		prtd->last_pos = prtd->cur_pos;
		snd_pcm_period_elapsed(substream);
	}

	if (prtd->cur_pos >= runtime->buffer_size) {
		prtd->cur_pos -= runtime->buffer_size;
	}
}

static int play_thread( void *arg)
{
	struct snd_pcm_substream *substream = (struct snd_pcm_substream *)arg;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct pcm_runtime_data *prtd = substream->runtime->private_data;
	struct snd_pcm *pcm = substream->pcm;
	struct tegra_audio_data *ptscx = tegra_snd_cx[pcm->device];
	int offset = 0;
	int rtbuffersize = 0;
	int buffer_to_prime = 0, buffer_in_queue = 0;
	NvAudioFxState state = NVALSA_INVALID_STATE;

	wait_for_completion(&prtd->thread_comp);
	rtbuffersize = frames_to_bytes(runtime, runtime->buffer_size);
	buffer_to_prime  = (rtbuffersize / TEGRA_DEFAULT_BUFFER_SIZE);

	if (runtime->control->appl_ptr)
	for (;;) {
		state = play_state(ptscx, prtd, state);
		if (state == SNDRV_PCM_TRIGGER_STOP)
			buffer_in_queue = 0;
		if (kthread_should_stop())
			return 0;
		if ((prtd->audiofx_frames < runtime->control->appl_ptr) &&
				(state != NvAudioFxState_Stop)) {
			offset = queue_next_buffer(substream, offset);
			buffer_in_queue++;
		}
		if (buffer_in_queue == 0) {
			DEFINE_WAIT(wq);
			prepare_to_wait(&prtd->buf_wait, &wq, TASK_INTERRUPTIBLE);
			schedule();
			finish_wait(&prtd->buf_wait, &wq);
			continue;
		}
		if ((buffer_to_prime == buffer_in_queue) ||
			(prtd->audiofx_frames >= runtime->control->appl_ptr)) {
			play_buffer_done(substream);
			buffer_in_queue--;
		}
	}

	for (;;) {
		state = play_state(ptscx, prtd, state);
		if (state == SNDRV_PCM_TRIGGER_STOP)
			buffer_in_queue = 0;
		if (kthread_should_stop())
			break;
		if ((prtd->state != SNDRV_PCM_TRIGGER_STOP) &&
			 (runtime->status->state != SNDRV_PCM_STATE_DRAINING)) {
			offset = queue_next_buffer(substream, offset);
			buffer_in_queue++;
		}
		if (buffer_in_queue == 0) {
			DEFINE_WAIT(wq);
			prepare_to_wait(&prtd->buf_wait, &wq, TASK_INTERRUPTIBLE);
			schedule();
			finish_wait(&prtd->buf_wait, &wq);
			continue;
		}
		if ((buffer_to_prime == buffer_in_queue) &&
			(prtd->state != SNDRV_PCM_TRIGGER_STOP)) {
			play_buffer_done(substream);
			buffer_in_queue--;
		}
	}
	return 0;
}

static int rec_thread( void *arg )
{
	struct snd_pcm_substream *substream = arg;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct pcm_runtime_data *prtd = substream->runtime->private_data;
	NvError e;
	int size = 0;
	int offset = 0;
	int period_offset = 0;
	int rtbuffersize = 0;
	int buffer_to_prime = 0, buffer_in_queue = 0;
	NvAudioFxBufferDescriptor abd;
	NvAudioFxState state = NVALSA_INVALID_STATE;
	NvAudioFxPinFormatDescriptor pin_format;
	struct snd_pcm *pcm = substream->pcm;
	struct tegra_audio_data *ptscx = tegra_snd_cx[pcm->device];

	wait_for_completion(&prtd->thread_comp);
	rtbuffersize = frames_to_bytes(runtime, runtime->buffer_size);
	buffer_to_prime  = (rtbuffersize / TEGRA_DEFAULT_BUFFER_SIZE);

	for (;;) {
		switch (prtd->state) {
		case SNDRV_PCM_TRIGGER_START:
			if (state != NvAudioFxState_Run) {
				pin_format.Format.FormatTag = 1;
				pin_format.Format.SampleRate = runtime->rate;
				pin_format.Format.BitsPerSample = runtime->sample_bits;
				pin_format.Format.Channels = runtime->channels;
				pin_format.Format.ChannelMask = 0;
				pin_format.Format.ValidBitsPerSample = 0;
				pin_format.Pin = NvAudioFxSourcePin;

				e = ptscx->xrt_fxn.SetProperty(
						 prtd->stdinpath->Convert,
						 NvAudioFxPinProperty_Format,
						 sizeof(NvAudioFxPinFormatDescriptor),
						 &pin_format);
				if (e != NvSuccess) {
					snd_printk(KERN_ERR"set_property failed!\n");
				}

				e = ptscx->xrt_fxn.SetProperty(
						 prtd->stdinpath->Src,
						 NvAudioFxProperty_SampleRate,
						 sizeof(NvS32),
						 &pin_format.Format.SampleRate);
				if (e != NvSuccess) {
					snd_printk(KERN_ERR "set_property failed!\n");
				}

				state = NvAudioFxState_Run;
				ptscx->xrt_fxn.SetProperty(
						 prtd->stdinpath->Stream,
						 NvAudioFxProperty_State,
						 sizeof(NvAudioFxState),
						 &state);
			}
			break;
		case SNDRV_PCM_TRIGGER_STOP:
			if (state != NvAudioFxState_Stop) {
				state = NvAudioFxState_Stop;
				ptscx->xrt_fxn.SetProperty(
						 prtd->stdinpath->Stream,
						 NvAudioFxProperty_State,
						 sizeof(NvAudioFxState),
						 &state);
				down(&prtd->stop_done_sem);
				buffer_in_queue = 0;
			}
			goto EXIT;
		default:
			;
		}

		if ((state == NvAudioFxState_Run) &&
			(buffer_in_queue < buffer_to_prime)) {
			memset(&abd, 0, sizeof(NvAudioFxBufferDescriptor));

			size = TEGRA_DEFAULT_BUFFER_SIZE;
			if ((offset + size) > rtbuffersize) {
				size = rtbuffersize - offset;
			}

			abd.hMixBuffer = prtd->mixer_buffer;
			abd.Offset = offset;
			abd.Size = size;
			abd.Format.FormatTag = 1;
			abd.Format.SampleRate = runtime->rate;
			abd.Format.BitsPerSample = runtime->sample_bits;
			abd.Format.Channels = runtime->channels;
			abd.Format.ChannelMask = 0;

			e = ptscx->xrt_fxn.StreamAddBuffer(
				(NvAudioFxStreamHandle)prtd->stdinpath->Stream, &abd);
			buffer_in_queue++;
			offset += size;

			if (offset >= rtbuffersize)
				offset =0;
		}

		if ((buffer_to_prime == buffer_in_queue) &&
		    ((runtime->status->hw_ptr - runtime->control->appl_ptr) <
		    (runtime->buffer_size -runtime->period_size))) {
			down(&prtd->buf_done_sem);

			buffer_in_queue--;

			if ((frames_to_bytes(runtime, prtd->cur_pos) +
				TEGRA_DEFAULT_BUFFER_SIZE) > rtbuffersize) {
				size = rtbuffersize -
				       frames_to_bytes(runtime, prtd->cur_pos);
			} else {
				size = TEGRA_DEFAULT_BUFFER_SIZE;
			}

			prtd->cur_pos += bytes_to_frames(runtime, size);

			if (prtd->cur_pos < prtd->last_pos) {
				period_offset = (runtime->buffer_size +
						prtd->cur_pos) - prtd->last_pos;
			} else {
				period_offset = prtd->cur_pos - prtd->last_pos;
			}

			if (period_offset >= runtime->period_size) {
				prtd->last_pos = prtd->cur_pos;
				snd_pcm_period_elapsed(substream);
			}

			if (prtd->cur_pos >= runtime->buffer_size) {
				prtd->cur_pos -= runtime->buffer_size;
			}
		}
	}
EXIT:

	while (!kthread_should_stop()) {
	}

	return 0;
}

static int tegra_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct pcm_runtime_data *prtd = substream->runtime->private_data;
	int ret = 0;
	int state = prtd->state;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		prtd->state = cmd;
		prtd->cur_pos = 0;
		prtd->last_pos = 0;
		prtd->audiofx_frames = 0;
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			prtd->timeout = PLAY_TIMEOUT;
		else
			prtd->timeout = REC_TIMEOUT;

		if (state == NVALSA_INVALID_STATE)
			complete(&prtd->thread_comp);
		else
			up(&prtd->buf_done_sem);
		break;
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			prtd->timeout = PLAY_TIMEOUT;
		} else {
			prtd->timeout = REC_TIMEOUT;
		}
		break;
	case SNDRV_PCM_TRIGGER_STOP:
		prtd->state = cmd;
		break;
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		prtd->timeout = NV_WAIT_INFINITE;
		break;
	default:
		prtd->state = state;
		ret = -EINVAL;
		break;
	}
	return ret;
}

static snd_pcm_uframes_t
tegra_pcm_pointer(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct pcm_runtime_data *prtd = runtime->private_data;
	int size;

	size = prtd->last_pos;
	if (size >= runtime->buffer_size) {
		prtd->last_pos = size - runtime->buffer_size;
		size = 0;
	}

	return (size);
}

static int pcm_common_close(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct pcm_runtime_data *prtd = runtime->private_data;
	struct snd_pcm *pcm = substream->pcm;
	struct tegra_audio_data *ptscx = tegra_snd_cx[pcm->device];
	NvAudioFxMessage message;
	NvError e;

	if (!prtd)
		snd_printk(KERN_ERR "pcm_close called with prtd = NULL\n");

	prtd->state = SNDRV_PCM_TRIGGER_STOP;

	if (completion_done(&prtd->thread_comp) == 0)
		complete(&prtd->thread_comp);

	wake_up_all(&prtd->buf_wait);

	if (prtd->play_thread)
		kthread_stop(prtd->play_thread);

	if (prtd->rec_thread)
		kthread_stop(prtd->rec_thread);

	if (ptscx->m_FxNotifier.Event) {

		memset(&message, 0, sizeof(NvAudioFxMessage));
		message.Event = NvAudioFxEventAll;
		message.pContext = NULL;
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			message.hFx = (NvAudioFxHandle)prtd->stdoutpath->Stream;
		else
			message.hFx = (NvAudioFxHandle)prtd->stdinpath->Stream;

		e = ptscx->xrt_fxn.SetProperty(
		    (NvAudioFxObjectHandle)ptscx->m_FxNotifier.hNotifier,
		    NvAudioFxIoProperty_RemoveEvent,
		    sizeof(NvAudioFxMessage),
		    &message);

		ptscx->m_FxNotifier.Event &= ~NvAudioFxEventAll;
	}

	if (prtd->stdoutpath) {
		tegra_audiofx_destroy_output(prtd->stdoutpath);
		kfree(prtd->stdoutpath);
	}

	if (prtd->stdinpath) {
		tegra_audiofx_destroy_input(prtd->stdinpath);
		kfree(prtd->stdinpath);
	}

	if (prtd)
		kfree(prtd);

	return 0;
}

static int tegra_pcm_open(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_pcm *pcm = substream->pcm;
	struct pcm_runtime_data *prtd;
	int ret = 0;
	NvError e = NvSuccess;
	NvAudioFxMessage message;
	NvAudioFxObjectHandle hSource = 0;
	struct tegra_audio_data *ptscx = tegra_snd_cx[pcm->device];

	prtd = kzalloc(sizeof(struct pcm_runtime_data), GFP_KERNEL);
	if (prtd == NULL)
		return -ENOMEM;

	runtime->private_data = prtd;
	snd_soc_set_runtime_hwparams(substream, &tegra_pcm_hardware);
	spin_lock_init(&prtd->lock);
	prtd->timeout = INIT_TIMEOUT;
	prtd->stdoutpath = 0;
	prtd->stdinpath = 0;
	prtd->state = NVALSA_INVALID_STATE;
	prtd->stream = substream->stream;

	if (!ptscx->mixer_handle) {
		ret = tegra_audiofx_init(ptscx);
		if (ret)
			goto fail;
	}

	init_completion(&prtd->thread_comp);
	init_waitqueue_head(&prtd->buf_wait);
	sema_init(&prtd->buf_done_sem, 0);
	sema_init(&prtd->stop_done_sem, 0);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK){
		hSource = ptscx->i2s1_play_mix;
		if (pcm->device == I2S2)
			hSource = ptscx->i2s2_play_mix;

		prtd->mixer_buffer = ptscx->mixer_buffer[0];
		prtd->stdoutpath = (StandardPath*)kzalloc(sizeof(StandardPath),
							  GFP_KERNEL);
		if (prtd->stdoutpath == NULL) {
			snd_printk(KERN_ERR "pcm_open kzalloc failed \n");
			ret = -ENOMEM;
			goto fail;
		}

		e = tegra_audiofx_create_output(ptscx->m_hRm,
						ptscx->mixer_handle,
						prtd->stdoutpath,
						hSource);
		if (e != NvSuccess) {
			snd_printk(KERN_ERR "audiofx_create_output failed \n");
			ret = -EFAULT;
			goto fail;
		}

		memset(&message, 0, sizeof(NvAudioFxMessage));
		message.Event = (NvAudioFxEventBufferDone |
				 NvAudioFxEventStateChange);
		message.hFx = (NvAudioFxHandle)prtd->stdoutpath->Stream;
		message.pContext = prtd;

		e = ptscx->xrt_fxn.SetProperty(
		    (NvAudioFxObjectHandle)ptscx->m_FxNotifier.hNotifier,
		    NvAudioFxIoProperty_AddEvent,
		    sizeof(NvAudioFxMessage),
		    &message);

		if (e != NvSuccess) {
			snd_printk(KERN_ERR "TransportSetProperty failed\n");
			ret = -EFAULT;
			goto fail;
		}

		ptscx->m_FxNotifier.Event |= (NvAudioFxEventBufferDone |
						     NvAudioFxEventStateChange);

		prtd->play_thread = kthread_run(play_thread,
						substream,
						"%sthread",
						"play");
		if (IS_ERR(prtd->play_thread)) {
			snd_printk(KERN_ERR "KTHREAD RUN FAIL\n");
			ret = PTR_ERR(prtd->play_thread);
			goto fail;
		}
	} else {
		hSource = ptscx->i2s1_rec_split;
		if (pcm->device == I2S2)
			hSource = ptscx->i2s2_rec_split;

		prtd->mixer_buffer = ptscx->mixer_buffer[1];
		prtd->stdinpath = (StandardPath*)kzalloc(sizeof(StandardPath),
							  GFP_KERNEL);
		if (prtd->stdinpath == NULL) {
			snd_printk(KERN_ERR "pcm_open kzalloc failed \n");
			ret = -ENOMEM;
			goto fail;
		}
		e = tegra_audiofx_create_input(ptscx->m_hRm,
						ptscx->mixer_handle,
						prtd->stdinpath,
						NvAudioInputSelect_Record,
						hSource);
		if (e != NvSuccess) {
			snd_printk(KERN_ERR "audiofx_create_input failed \n");
			ret = -EFAULT;
			goto fail;
		}

		memset(&message, 0, sizeof(NvAudioFxMessage));
		message.Event = (NvAudioFxEventBufferDone |
				 NvAudioFxEventStateChange);
		message.hFx = (NvAudioFxHandle)prtd->stdinpath->Stream;
		message.pContext = prtd;

		e = ptscx->xrt_fxn.SetProperty(
		    (NvAudioFxObjectHandle)ptscx->m_FxNotifier.hNotifier,
		    NvAudioFxIoProperty_AddEvent,
		    sizeof(NvAudioFxMessage),
		    &message);
		if (e != NvSuccess) {
			snd_printk(KERN_ERR "TransportSetProperty failed\n");
			ret = -EFAULT;
			goto fail;
		}
		ptscx->m_FxNotifier.Event |= (NvAudioFxEventBufferDone |
						     NvAudioFxEventStateChange);

		prtd->rec_thread = kthread_run(rec_thread,
						substream,
						"%sthread",
						"rec" );
		if (IS_ERR(prtd->rec_thread)) {
			snd_printk(KERN_ERR "Kthread Run Fail\n");
			ret = PTR_ERR(prtd->rec_thread);
			goto fail;
		}
	}
	return ret;
fail:
	snd_printk(KERN_ERR "tegra_pcm_open - failed \n");
	pcm_common_close(substream);
	return ret;
}

static int tegra_pcm_close(struct snd_pcm_substream *substream)
{
	pcm_common_close(substream);
	return 0;
}

static int tegra_pcm_mmap(struct snd_pcm_substream *substream,
			  struct vm_area_struct *vma)
{
	int err = 0;
	int size = 0;
	char *vmalloc_area_ptr = NULL;
	unsigned long start = 0;
	unsigned long pfn = 0;

	start = vma->vm_start;
	vmalloc_area_ptr = substream->dma_buffer.area;
	size = vma->vm_end - vma->vm_start;
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

	while (size > 0) {
		pfn = vmalloc_to_pfn(vmalloc_area_ptr);
		err = io_remap_pfn_range(vma, start, pfn,
					 PAGE_SIZE, vma->vm_page_prot);
		if (err < 0) {
			snd_printk(KERN_ERR "io_remap_pfn_range failed \n");
			return err;
		}
		start += PAGE_SIZE;
		vmalloc_area_ptr += PAGE_SIZE;
		size -= PAGE_SIZE;
	}
	return err;
}

static int tegra_pcm_ack(struct snd_pcm_substream *substream)
{
	struct pcm_runtime_data *prtd = substream->runtime->private_data;

	wake_up(&prtd->buf_wait);
	return 0;
}

static struct snd_pcm_ops tegra_pcm_ops = {
	.open = tegra_pcm_open,
	.close = tegra_pcm_close,
	.ioctl = snd_pcm_lib_ioctl,
	.hw_params = tegra_pcm_hw_params,
	.hw_free = tegra_pcm_hw_free,
	.prepare = tegra_pcm_prepare,
	.trigger = tegra_pcm_trigger,
	.pointer = tegra_pcm_pointer,
	.mmap = tegra_pcm_mmap,
	.ack = tegra_pcm_ack,
};

static int tegra_pcm_preallocate_dma_buffer(struct snd_pcm *pcm, int stream)
{
	struct snd_pcm_substream *substream = pcm->streams[stream].substream;
	struct snd_dma_buffer *buf = &substream->dma_buffer;
	size_t size = tegra_pcm_hardware.buffer_bytes_max;
	void *virt_buf_ptr = NULL;
	NvRmPhysAddr phy_address;
	struct tegra_audio_data *ptscx = tegra_snd_cx[pcm->device];
	int ret = 0;
	NvError e;

	e = NvRmMemHandleCreate(ptscx->m_hRm,
				&ptscx->mem_handle[stream],
				size);

	if (e == NvSuccess) {
		e = NvRmMemAlloc(ptscx->mem_handle[stream],
				NULL,
				0,
				PAGE_SIZE,
				NvOsMemAttribute_Uncached);
	}

	if (e == NvSuccess) {
		phy_address = (NvU32)(NvRmMemPin(ptscx->mem_handle[stream]));
	}

	if (e != NvSuccess) {
		NvRmMemHandleFree(ptscx->mem_handle[stream]);
		ret = -ENOMEM;
		goto end;
	}

	e = NvRmMemMap(ptscx->mem_handle[stream],
			0,
			size,
			NVOS_MEM_READ_WRITE,
			(void**)&virt_buf_ptr);

	if (e != NvSuccess) {
		NvRmMemHandleFree(ptscx->mem_handle[stream]);
		ret = -ENOMEM;
		goto end;
	}

	ptscx->mapped_buf_size = size;

	buf->dev.type = SNDRV_DMA_TYPE_DEV;
	buf->dev.dev = pcm->card->dev;
	buf->private_data = NULL;
	buf->area = virt_buf_ptr;
	buf->bytes = size;
	buf->addr = phy_address;
end:
	return ret;
}

static void DestroyMemoryHandle(NvRmMemHandle hMemHandle)
{
	if (hMemHandle != NULL) {
		NvRmMemUnpin(hMemHandle);
		NvRmMemHandleFree(hMemHandle);
	}
}

static void tegra_pcm_deallocate_dma_buffer(struct snd_pcm *pcm, int stream)
{
	struct snd_pcm_substream *substream = pcm->streams[stream].substream;
	struct snd_dma_buffer *buf = &substream->dma_buffer;
	struct tegra_audio_data *ptscx = tegra_snd_cx[pcm->device];

	if (ptscx->mixer_buffer[stream])
		ptscx->xrt_fxn.MixerUnmapBuffer(
					    ptscx->mixer_buffer[stream]);

	NvRmMemUnmap(ptscx->mem_handle[stream],buf->area,buf->bytes);
	DestroyMemoryHandle(ptscx->mem_handle[stream]);
}

static void tegra_pcm_free_dma_buffers(struct snd_pcm *pcm)
{
	struct snd_pcm_substream *substream;
	struct snd_dma_buffer *buf;
	int stream;
	for (stream = 0; stream < 2; stream++) {
		substream = pcm->streams[stream].substream;
		buf = &substream->dma_buffer;
		if (!buf) {
			continue;
		}
		tegra_pcm_deallocate_dma_buffer(pcm ,stream);
	}
}

static int tegra_pcm_new(struct snd_card *card,
			 struct snd_soc_dai *dai,
			 struct snd_pcm *pcm)
{
	struct tegra_audio_data *ptscx;
	int ret = 0;

	ptscx = kzalloc(sizeof(struct tegra_audio_data),
			GFP_KERNEL);
	if (ptscx == NULL)
		return -ENOMEM;

	ptscx->m_hRm = s_hRmGlobal;
	ptscx->device_id = pcm->device;
	mutex_init(&ptscx->lock);

	tegra_snd_cx[pcm->device] = ptscx;

	if (dai->playback.channels_min) {
		ret = tegra_pcm_preallocate_dma_buffer(pcm,
						     SNDRV_PCM_STREAM_PLAYBACK);
		if (ret) {
			goto out;
		}
	}

	if (dai->capture.channels_min) {
		ret = tegra_pcm_preallocate_dma_buffer(pcm,
						      SNDRV_PCM_STREAM_CAPTURE);
		if (ret)
			goto out;
	}
	return 0;

out:
	 if (ptscx)
		kfree(ptscx);
	snd_printk(KERN_ERR "pcm_new failed\n");
	return ret;
}

static int tegra_pcm_probe(struct platform_device *pdev)
{
	return 0;
}

static int tegra_pcm_remove(struct platform_device *pdev)
{
	if (tegra_snd_cx[I2S1])
		kfree(tegra_snd_cx[I2S1]);
	if (tegra_snd_cx[I2S2])
		kfree(tegra_snd_cx[I2S2]);

	return 0;
}

#ifdef CONFIG_PM
static int tegra_pcm_suspend(struct snd_soc_dai *dai)
{
	return 0;
}

static int tegra_pcm_resume(struct snd_soc_dai *dai)
{
	return 0;
}

#else
#define tegra_pcm_suspend	NULL
#define tegra_pcm_resume	NULL
#endif

struct snd_soc_platform tegra_soc_platform = {
	.name = "tegra-audio",
	.probe = tegra_pcm_probe,
	.remove = tegra_pcm_remove,
	.pcm_ops = &tegra_pcm_ops,
	.pcm_new = tegra_pcm_new,
	.pcm_free = tegra_pcm_free_dma_buffers,
	.suspend = tegra_pcm_suspend,
	.resume = tegra_pcm_resume,
};

EXPORT_SYMBOL_GPL(tegra_soc_platform);

static int __init tegra_soc_platform_init(void)
{
	return snd_soc_register_platform(&tegra_soc_platform);
}

module_init(tegra_soc_platform_init);

static void __exit tegra_soc_platform_exit(void)
{
	snd_soc_unregister_platform(&tegra_soc_platform);
}

module_exit(tegra_soc_platform_exit);

MODULE_DESCRIPTION("Tegra PCM RPC PCM output");
MODULE_LICENSE("GPL");
