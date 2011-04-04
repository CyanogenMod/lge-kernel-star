/* include/linux/logger.h
 *
 * Copyright (C) 2007-2008 Google, Inc.
 * Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 * Author: Robert Love <rlove@android.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef _LINUX_LOGGER_H
#define _LINUX_LOGGER_H

#include <linux/types.h>
#include <linux/ioctl.h>

struct logger_entry {
	__u16		len;	/* length of the payload */
	__u16		__pad;	/* no matter what, we get 2 bytes of padding */
	__s32		pid;	/* generating process's pid */
	__s32		tid;	/* generating process's tid */
	__s32		sec;	/* seconds since Epoch */
	__s32		nsec;	/* nanoseconds */
	char		msg[0];	/* the entry's payload */
};

#define LOGGER_LOG_RADIO	"log_radio"	/* radio-related messages */
#define LOGGER_LOG_EVENTS	"log_events"	/* system/hardware events */
#define LOGGER_LOG_MAIN		"log_main"	/* everything else */

#define LOGGER_ENTRY_MAX_LEN		(4*1024)
#define LOGGER_ENTRY_MAX_PAYLOAD	\
	(LOGGER_ENTRY_MAX_LEN - sizeof(struct logger_entry))

#define __LOGGERIO	0xAE

#define LOGGER_GET_LOG_BUF_SIZE		_IO(__LOGGERIO, 1) /* size of log */
#define LOGGER_GET_LOG_LEN		_IO(__LOGGERIO, 2) /* used log len */
#define LOGGER_GET_NEXT_ENTRY_LEN	_IO(__LOGGERIO, 3) /* next entry len */
#define LOGGER_FLUSH_LOG		_IO(__LOGGERIO, 4) /* flush log */

#ifdef __KERNEL__
enum {
    LOG_PRIORITY_UNKNOWN = 0,
    LOG_PRIORITY_DEFAULT,    /* only for SetMinPriority() */
    LOG_PRIORITY_VERBOSE,
    LOG_PRIORITY_DEBUG,
    LOG_PRIORITY_INFO,
    LOG_PRIORITY_WARN,
    LOG_PRIORITY_ERROR,
    LOG_PRIORITY_FATAL,
    LOG_PRIORITY_SILENT,     /* only for SetMinPriority(); must be last */
};

enum logidx {
	LOG_MAIN_IDX = 0,
	LOG_EVENTS_IDX,
	LOG_RADIO_IDX,
	LOG_INVALID_IDX,
};

int logger_write(const enum logidx index,
		const unsigned char priority,
		const char __kernel * const tag,
		const char __kernel * const fmt,
		...);

#endif /* __KERNEL__ */

#endif /* _LINUX_LOGGER_H */
