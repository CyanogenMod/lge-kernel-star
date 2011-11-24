/*
 * include/trace/events/nvhost.h
 *
 * Nvhost event logging to ftrace.
 *
 * Copyright (c) 2010-2011, NVIDIA Corporation.
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

#undef TRACE_SYSTEM
#define TRACE_SYSTEM nvhost

#if !defined(_TRACE_NVHOST_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_NVHOST_H

#include <linux/ktime.h>
#include <linux/tracepoint.h>

DECLARE_EVENT_CLASS(nvhost,
	TP_PROTO(const char *name),
	TP_ARGS(name),
	TP_STRUCT__entry(__field(const char *, name)),
	TP_fast_assign(__entry->name = name;),
	TP_printk("name=%s", __entry->name)
);

DEFINE_EVENT(nvhost, nvhost_channel_open,
	TP_PROTO(const char *name),
	TP_ARGS(name)
);

DEFINE_EVENT(nvhost, nvhost_channel_release,
	TP_PROTO(const char *name),
	TP_ARGS(name)
);

DEFINE_EVENT(nvhost, nvhost_ioctl_channel_flush,
	TP_PROTO(const char *name),
	TP_ARGS(name)
);

TRACE_EVENT(nvhost_channel_write_submit,
	TP_PROTO(const char *name, ssize_t count, u32 cmdbufs, u32 relocs,
			u32 syncpt_id, u32 syncpt_incrs),

	TP_ARGS(name, count, cmdbufs, relocs, syncpt_id, syncpt_incrs),

	TP_STRUCT__entry(
		__field(const char *, name)
		__field(ssize_t, count)
		__field(u32, cmdbufs)
		__field(u32, relocs)
		__field(u32, syncpt_id)
		__field(u32, syncpt_incrs)
	),

	TP_fast_assign(
		__entry->name = name;
		__entry->count = count;
		__entry->cmdbufs = cmdbufs;
		__entry->relocs = relocs;
		__entry->syncpt_id = syncpt_id;
		__entry->syncpt_incrs = syncpt_incrs;
	),

	TP_printk("name=%s, count=%d, cmdbufs=%u, relocs=%u, syncpt_id=%u, syncpt_incrs=%u",
	  __entry->name, __entry->count, __entry->cmdbufs, __entry->relocs,
	  __entry->syncpt_id, __entry->syncpt_incrs)
);

TRACE_EVENT(nvhost_ioctl_channel_submit,
	TP_PROTO(const char *name, u32 version, u32 cmdbufs, u32 relocs,
		 u32 waitchks, u32 syncpt_id, u32 syncpt_incrs),

	TP_ARGS(name, version, cmdbufs, relocs, waitchks,
			syncpt_id, syncpt_incrs),

	TP_STRUCT__entry(
		__field(const char *, name)
		__field(u32, version)
		__field(u32, cmdbufs)
		__field(u32, relocs)
		__field(u32, waitchks)
		__field(u32, syncpt_id)
		__field(u32, syncpt_incrs)
	),

	TP_fast_assign(
		__entry->name = name;
		__entry->version = version;
		__entry->cmdbufs = cmdbufs;
		__entry->relocs = relocs;
		__entry->waitchks = waitchks;
		__entry->syncpt_id = syncpt_id;
		__entry->syncpt_incrs = syncpt_incrs;
	),

	TP_printk("name=%s, version=%u, cmdbufs=%u, relocs=%u, waitchks=%u, syncpt_id=%u, syncpt_incrs=%u",
	  __entry->name, __entry->version, __entry->cmdbufs, __entry->relocs,
	  __entry->waitchks, __entry->syncpt_id, __entry->syncpt_incrs)
);

TRACE_EVENT(nvhost_channel_write_cmdbuf,
	TP_PROTO(const char *name, u32 mem_id,
			u32 words, u32 offset),

	TP_ARGS(name, mem_id, words, offset),

	TP_STRUCT__entry(
		__field(const char *, name)
		__field(u32, mem_id)
		__field(u32, words)
		__field(u32, offset)
	),

	TP_fast_assign(
		__entry->name = name;
		__entry->mem_id = mem_id;
		__entry->words = words;
		__entry->offset = offset;
	),

	TP_printk("name=%s, mem_id=%08x, words=%u, offset=%d",
	  __entry->name, __entry->mem_id,
	  __entry->words, __entry->offset)
);

TRACE_EVENT(nvhost_channel_write_cmdbuf_data,
	TP_PROTO(const char *name, u32 mem_id,
			u32 words, u32 offset, void *cmdbuf),

	TP_ARGS(name, mem_id, words, offset, cmdbuf),

	TP_STRUCT__entry(
		__field(const char *, name)
		__field(u32, mem_id)
		__field(u32, words)
		__field(u32, offset)
		__field(bool, cmdbuf)
		__dynamic_array(u32, cmdbuf, words)
	),

	TP_fast_assign(
		if (cmdbuf) {
			memcpy(__get_dynamic_array(cmdbuf), cmdbuf+offset,
					words * sizeof(u32));
		}
		__entry->cmdbuf = cmdbuf;
		__entry->name = name;
		__entry->mem_id = mem_id;
		__entry->words = words;
		__entry->offset = offset;
	),

	TP_printk("name=%s, mem_id=%08x, words=%u, offset=%d, contents=[%s]",
	  __entry->name, __entry->mem_id,
	  __entry->words, __entry->offset,
	  __print_hex(__get_dynamic_array(cmdbuf),
		  __entry->cmdbuf ? __entry->words * 4 : 0))
);

TRACE_EVENT(nvhost_channel_write_reloc,
	TP_PROTO(const char *name),

	TP_ARGS(name),

	TP_STRUCT__entry(
		__field(const char *, name)
	),

	TP_fast_assign(
		__entry->name = name;
	),

	TP_printk("name=%s",
	  __entry->name)
);

TRACE_EVENT(nvhost_channel_write_waitchks,
	TP_PROTO(const char *name, u32 waitchks, u32 waitmask),

	TP_ARGS(name, waitchks, waitmask),

	TP_STRUCT__entry(
		__field(const char *, name)
		__field(u32, waitchks)
		__field(u32, waitmask)
	),

	TP_fast_assign(
		__entry->name = name;
		__entry->waitchks = waitchks;
		__entry->waitmask = waitmask;
	),

	TP_printk("name=%s, waitchks=%u, waitmask=%08x",
	  __entry->name, __entry->waitchks, __entry->waitmask)
);

TRACE_EVENT(nvhost_channel_context_switch,
	TP_PROTO(const char *name, void *old_ctx, void *new_ctx),

	TP_ARGS(name, old_ctx, new_ctx),

	TP_STRUCT__entry(
	    __field(const char *, name)
	    __field(void*, old_ctx)
	    __field(void*, new_ctx)
	),

	TP_fast_assign(
	    __entry->name = name;
	    __entry->old_ctx = old_ctx;
	    __entry->new_ctx = new_ctx;
	),

	TP_printk("name=%s, old=%p, new=%p",
	  __entry->name, __entry->old_ctx, __entry->new_ctx)
);

TRACE_EVENT(nvhost_ctrlopen,
	TP_PROTO(const char *name),
	TP_ARGS(name),
	TP_STRUCT__entry(
	    __field(const char *, name)
	),
	TP_fast_assign(
	    __entry->name = name
	),
	TP_printk("name=%s", __entry->name)
);

TRACE_EVENT(nvhost_ctrlrelease,
	TP_PROTO(const char *name),
	TP_ARGS(name),
	TP_STRUCT__entry(
	    __field(const char *, name)
	),
	TP_fast_assign(
	    __entry->name = name
	),
	TP_printk("name=%s", __entry->name)
);

TRACE_EVENT(nvhost_ioctl_ctrl_module_mutex,
	TP_PROTO(u32 lock, u32 id),

	TP_ARGS(lock, id),

	TP_STRUCT__entry(
	    __field(u32, lock);
	    __field(u32, id);
	),

	TP_fast_assign(
		__entry->lock = lock;
		__entry->id = id;
	),

	TP_printk("lock=%u, id=%d",
		__entry->lock, __entry->id)
	);

TRACE_EVENT(nvhost_ioctl_ctrl_syncpt_incr,
	TP_PROTO(u32 id),

	TP_ARGS(id),

	TP_STRUCT__entry(
	    __field(u32, id);
	),

	TP_fast_assign(
	   __entry->id = id;
	),

	TP_printk("id=%d", __entry->id)
);

TRACE_EVENT(nvhost_ioctl_ctrl_syncpt_read,
	TP_PROTO(u32 id),

	TP_ARGS(id),

	TP_STRUCT__entry(
	    __field(u32, id);
	),

	TP_fast_assign(
		__entry->id = id;
	),

	TP_printk("id=%d", __entry->id)
);

TRACE_EVENT(nvhost_ioctl_ctrl_syncpt_wait,
	TP_PROTO(u32 id, u32 threshold, s32 timeout),

	TP_ARGS(id, threshold, timeout),

	TP_STRUCT__entry(
		__field(u32, id)
		__field(u32, threshold)
		__field(s32, timeout)
	),

	TP_fast_assign(
		__entry->id = id;
		__entry->threshold = threshold;
		__entry->timeout = timeout;
	),

	TP_printk("id=%u, threshold=%u, timeout=%d",
	  __entry->id, __entry->threshold, __entry->timeout)
);

TRACE_EVENT(nvhost_ioctl_ctrl_module_regrdwr,
	TP_PROTO(u32 id, u32 num_offsets, bool write),

	TP_ARGS(id, num_offsets, write),

	TP_STRUCT__entry(
		__field(u32, id)
		__field(u32, num_offsets)
		__field(bool, write)
	),

	TP_fast_assign(
		__entry->id = id;
		__entry->num_offsets = num_offsets;
		__entry->write = write;
	),

	TP_printk("id=%u, num_offsets=%u, write=%d",
	  __entry->id, __entry->num_offsets, __entry->write)
);

TRACE_EVENT(nvhost_channel_submitted,
	TP_PROTO(const char *name, u32 syncpt_base, u32 syncpt_max),

	TP_ARGS(name, syncpt_base, syncpt_max),

	TP_STRUCT__entry(
		__field(const char *, name)
		__field(u32, syncpt_base)
		__field(u32, syncpt_max)
	),

	TP_fast_assign(
		__entry->name = name;
		__entry->syncpt_base = syncpt_base;
		__entry->syncpt_max = syncpt_max;
	),

	TP_printk("name=%s, syncpt_base=%d, syncpt_max=%d",
		__entry->name, __entry->syncpt_base, __entry->syncpt_max)
);

TRACE_EVENT(nvhost_channel_submit_complete,
	TP_PROTO(const char *name, int count),

	TP_ARGS(name, count),

	TP_STRUCT__entry(
		__field(const char *, name)
		__field(int, count)
	),

	TP_fast_assign(
		__entry->name = name;
		__entry->count = count;
	),

	TP_printk("name=%s, count=%d", __entry->name, __entry->count)
);

TRACE_EVENT(nvhost_wait_cdma,
	TP_PROTO(const char *name, u32 eventid),

	TP_ARGS(name, eventid),

	TP_STRUCT__entry(
		__field(const char *, name)
		__field(u32, eventid)
	),

	TP_fast_assign(
		__entry->name = name;
		__entry->eventid = eventid;
	),

	TP_printk("name=%s, event=%d", __entry->name, __entry->eventid)
);

#endif /*  _TRACE_NVHOST_H */

/* This part must be outside protection */
#include <trace/define_trace.h>
