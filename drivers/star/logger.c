/*
 * drivers/misc/logger.c
 *
 * A Logging Subsystem
 *
 * Copyright (C) 2007-2008 Google, Inc.
 * Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 *
 * Robert Love <rlove@google.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/poll.h>
#include <linux/time.h>
#include "logger.h"
#include <linux/kobject.h>
#include <linux/spinlock.h>

#include <asm/ioctls.h>
#include <asm/atomic.h>

/*
 * struct logger_log - represents a specific log, such as 'main' or 'radio'
 *
 * This structure lives from module insertion until module removal, so it does
 * not need additional reference counting. The structure is protected by the
 * spinlock 'bufflock'.
 */
struct logger_log {
	unsigned char *		buffer;	/* the ring buffer itself */
	struct miscdevice	misc;	/* misc device representing the log */
	wait_queue_head_t	wq;	/* wait queue for readers */
	struct list_head	readers; /* this log's readers */
	spinlock_t		bufflock;	/* spinlock for buffer */
	size_t			w_off;	/* current write head offset */
	size_t			head;	/* new readers start here */
	const size_t		size;	/* size of the log */
	atomic_t		enabled;
	atomic_t		priority;
	atomic_t		was_overrun;
	spinlock_t		taglist_lock;	/* spinlock for tag list */
	struct list_head	tags;
	struct kobject		kobj;
};
#define to_log(a) container_of(a, struct logger_log, kobj)

/*
 * struct logger_reader - a logging device open for reading
 *
 * This object lives from open to release, so we don't need additional
 * reference counting. The structure is protected by log->bufflock.
 */
struct logger_reader {
	struct logger_log *	log;	/* associated log */
	struct list_head	list;	/* entry in logger_log's list */
	size_t			r_off;	/* current read head offset */
};

/*
 * struct logger_tag - a tag element based on a character string that allows
 * fine grained control of logging.
 */

struct logger_tag {
	struct logger_log	*log;	/* associated log */
	struct list_head	list;	/* entry in logger_log's list */
	atomic_t		priority;
	atomic_t		enabled;
	struct kobject		kobj;
	/* next element must be last in struct because it is dynamic length */
	char			name[0]; /* must be last in struct */
};
#define to_tag(t) container_of(t, struct logger_tag, kobj)

static struct kset *logger_kset;

/* ricky_kwak@lge.com
 * July 1, 2009
 * enabled logd
 */
#if defined(CONFIG_MACH_STAR)
unsigned char logger_default_priority = LOG_PRIORITY_DEFAULT;
#else
unsigned char logger_default_priority = LOG_PRIORITY_INFO;
#endif
/**/

EXPORT_SYMBOL(logger_default_priority);

int logger_default_enabled = 1;
EXPORT_SYMBOL(logger_default_enabled);

MODULE_PARM_DESC(default_priority,
	"Set initial default logging message priority for all entities");

MODULE_PARM_DESC(default_enabled,
	"Set initial default enabled state for all entities");

module_param_named(default_priority,
		logger_default_priority,
		byte,
		S_IRUGO | S_IWUSR);
module_param_named(default_enabled,
		logger_default_enabled,
		bool,
		S_IRUGO | S_IWUSR);

#define DEV_NAME "logger"

#define LOGGER_ATTR(_name, _mode, _show, _store) {            \
	.attr = {.name = __stringify(_name), .mode = _mode }, \
	.show = _show,                                        \
	.store = _store,                                      \
}

/* logger_offset - returns index 'n' into the log via (optimized) modulus */
#define logger_offset(n)	((n) & (log->size - 1))

/*
 * file_get_log - Given a file structure, return the associated log
 *
 * This isn't aesthetic. We have several goals:
 *
 * 	1) Need to quickly obtain the associated log during an I/O operation
 * 	2) Readers need to maintain state (logger_reader)
 * 	3) Writers need to be very fast (open() should be a near no-op)
 *
 * In the reader case, we can trivially go file->logger_reader->logger_log.
 * For a writer, we don't want to maintain a logger_reader, so we just go
 * file->logger_log. Thus what file->private_data points at depends on whether
 * or not the file was opened for reading. This function hides that dirtiness.
 */
static inline struct logger_log *file_get_log(const struct file * const file)
{
	if (file->f_mode & FMODE_READ) {
		struct logger_reader *reader = file->private_data;
		return reader->log;
	} else
		return file->private_data;
}

/*
 * get_entry_len - Grabs the length of the payload of the next entry starting
 * from 'off'.
 *
 * Caller needs to hold log->bufflock.
 */
static __u32 get_entry_len(const struct logger_log * const log,
			const size_t off)
{
	__u16 val;

	switch (log->size - off) {
	case 1:
		memcpy(&val, log->buffer + off, 1);
		memcpy(((char *) &val) + 1, log->buffer, 1);
		break;
	default:
		memcpy(&val, log->buffer + off, 2);
	}

	return sizeof(struct logger_entry) + val;
}

/*
 * do_read_log_to_user - reads exactly 'count' bytes from 'log' into the
 * user-space buffer 'buf'. Returns 'count' on success.
 *
 * log->bufflock must not be held on entry.
 */
static ssize_t do_read_log_to_user(struct logger_log *log,
				   struct logger_reader *reader,
				   char __user *buf,
				   size_t count)
{
	size_t len;
	char *temp_buf = kmalloc(LOGGER_ENTRY_MAX_LEN, GFP_KERNEL);
	unsigned long flags;
	ssize_t ret = count;
	/*
	 * Can't call any user copy functions while holding a spinlock,
	 * so use a temp buffer.
	 */

	if (!temp_buf)
		return -ENOMEM;

	spin_lock_irqsave(&log->bufflock, flags);

	len = min(count, log->size - reader->r_off);

	/*
	 * We read from the log in two disjoint operations. First, we read from
	 * the current read head offset up to 'count' bytes or to the end of
	 * the log, whichever comes first.
	 */
	memcpy(temp_buf, log->buffer + reader->r_off, len);

	/*
	 * Second, we read any remaining bytes, starting back at the head of
	 * the log.
	 */
	if (count != len)
		memcpy(temp_buf + len, log->buffer, count - len);

	reader->r_off = logger_offset(reader->r_off + count);

	spin_unlock_irqrestore(&log->bufflock, flags);

	if (copy_to_user(buf, temp_buf, count))
		ret = -EFAULT;

	kfree(temp_buf);
	return ret;
}

/*
 * logger_read - our log's read() method
 *
 * Behavior:
 *
 * 	- O_NONBLOCK works
 * 	- If there are no log entries to read, blocks until log is written to
 * 	- Atomically reads exactly one log entry
 *
 * Optimal read size is LOGGER_ENTRY_MAX_LEN. Will set errno to EINVAL if read
 * buffer is insufficient to hold next entry.
 * Entered with spinlock not held.
 */
static ssize_t logger_read(struct file *file, char __user *buf,
			   size_t count, loff_t *pos)
{
	struct logger_reader *reader = file->private_data;
	struct logger_log *log = reader->log;
	unsigned long flags;
	ssize_t ret;
	DEFINE_WAIT(wait);

start:
	while (1) {
		prepare_to_wait(&log->wq, &wait, TASK_INTERRUPTIBLE);

		spin_lock_irqsave(&log->bufflock, flags);
		ret = (log->w_off == reader->r_off);
		spin_unlock_irqrestore(&log->bufflock, flags);

		if (!ret)
			break;

		if (file->f_flags & O_NONBLOCK) {
			ret = -EAGAIN;
			break;
		}

		if (signal_pending(current)) {
			ret = -EINTR;
			break;
		}

		schedule();
	}

	finish_wait(&log->wq, &wait);
	if (ret)
		return ret;

	spin_lock_irqsave(&log->bufflock, flags);

	/* is there still something to read or did we race? */
	if (unlikely(log->w_off == reader->r_off)) {
		spin_unlock_irqrestore(&log->bufflock, flags);
		goto start;
	}

	/* get the size of the next entry */
	ret = get_entry_len(log, reader->r_off);

	/*
	 * Unlock spinlock because copy to user is used in do_read_log_to_user
	 * which manages the spinlock itself. Function returns with spinlock
	 * not held.
	 */
	spin_unlock_irqrestore(&log->bufflock, flags);

	/* get exactly one entry from the log unless short buffer */
	return (count < ret) ?
		-EINVAL : do_read_log_to_user(log, reader, buf, ret);
}

/*
 * get_next_entry - return the offset of the first valid entry at least 'len'
 * bytes after 'off'.
 *
 * Caller must hold log->bufflock.
 */
static size_t get_next_entry(const struct logger_log * const log,
			size_t off,
			const size_t len)
{
	size_t count = 0;

	do {
		size_t nr = get_entry_len(log, off);
		off = logger_offset(off + nr);
		count += nr;
	} while (count < len);

	return off;
}

/*
 * clock_interval - is a < c < b in mod-space? Put another way, does the line
 * from a to b cross c?
 */
static inline int clock_interval(const size_t a, const size_t b, const size_t c)
{
	if (b < a) {
		if (a < c || b >= c)
			return 1;
	} else {
		if (a < c && b >= c)
			return 1;
	}

	return 0;
}

/*
 * fix_up_readers - walk the list of all readers and "fix up" any who were
 * lapped by the writer; also do the same for the default "start head".
 * We do this by "pulling forward" the readers and start head to the first
 * entry after the new write head.
 *
 * The caller needs to hold log->bufflock.
 */
static void fix_up_readers(struct logger_log * const log, const size_t len)
{
	size_t old = log->w_off;
	size_t new = logger_offset(old + len);
	struct logger_reader *reader;

	if (clock_interval(old, new, log->head)) {
		if (list_empty(&log->readers))
			atomic_set(&log->was_overrun, 1);
		log->head = get_next_entry(log, log->head, len);
	}

	list_for_each_entry(reader, &log->readers, list)
		if (clock_interval(old, new, reader->r_off)) {
			atomic_set(&log->was_overrun, 1);
			reader->r_off = get_next_entry(log, reader->r_off, len);
		}
}

struct tag_attr {
	struct attribute attr;
	ssize_t(*show) (const struct logger_tag * const, char * const);
	ssize_t(*store) (struct logger_tag * const,
		const char * const, const size_t count);
};
#define to_tag_attr(a) container_of(a, struct tag_attr, attr)

#define RW_TAG_ATTR(name)  \
static struct tag_attr tag_attr_## name = \
 LOGGER_ATTR(name, S_IRUGO | S_IWUSR, show_tag_## name, store_tag_## name)

static ssize_t show_tag(struct kobject *kobj, struct attribute *attr, char *buf)
{
	struct tag_attr *a = to_tag_attr(attr);
	return a->show ? a->show(to_tag(kobj), buf) : -EIO;
}

static ssize_t store_tag(struct kobject *kobj, struct attribute *attr,
		     const char *buf, size_t count)
{
	struct tag_attr *a = to_tag_attr(attr);
	return a->store ? a->store(to_tag(kobj), buf, count) : -EIO;
}

static struct sysfs_ops tag_ops = {
	.show = show_tag,
	.store = store_tag,
};

static ssize_t show_tag_enable(const struct logger_tag * const tag,
				char * const buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&tag->enabled));
}

static ssize_t store_tag_enable(struct logger_tag * const tag,
			const char * const buf,
			const size_t count)
{
	int enabled;

	if (sscanf(buf, "%d", &enabled) != 1)
		return -EINVAL;

	atomic_set(&tag->enabled, enabled);
	return strnlen(buf, count);
}
RW_TAG_ATTR(enable);

static ssize_t show_tag_priority(const struct logger_tag * const tag,
				char * const buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&tag->priority));
}

static ssize_t store_tag_priority(struct logger_tag * const tag,
			const char * const buf,
			const size_t count)
{
	unsigned int priority;

	if (sscanf(buf, "%u", &priority) != 1 || priority > LOG_PRIORITY_SILENT)
		return -EINVAL;

	atomic_set(&tag->priority, priority);
	return strnlen(buf, count);
}
RW_TAG_ATTR(priority);

static struct attribute *tag_attrs[] = {
	&tag_attr_enable.attr,
	&tag_attr_priority.attr,
	NULL
};

static struct kobj_type tag_ktype = {
	.sysfs_ops = &tag_ops,
	.default_attrs = tag_attrs,
};

static struct logger_tag *get_tag(struct logger_log * const log,
				const char __kernel * const tag_name,
				const int tag_name_len)
{
	struct logger_tag *tag;
	int ret = -ENOMEM;
	unsigned long flags;

	spin_lock_irqsave(&log->taglist_lock, flags);
	list_for_each_entry(tag, &log->tags, list)
		if (!strncmp(tag->name, tag_name, tag_name_len)) {
			spin_unlock_irqrestore(&log->taglist_lock, flags);
			return tag;
		}
	spin_unlock_irqrestore(&log->taglist_lock, flags);

	/* Register tag name automatically if able to.
	 * Use GFP_ATOMIC due to inability to reliably know if we
	 * are able to sleep while allocating memory here or not.
	 */
	tag = kzalloc(sizeof *tag + tag_name_len, GFP_ATOMIC);
	if (!tag)
		goto out;

	memcpy(tag->name, tag_name, tag_name_len);
	tag->log = log;
	atomic_set(&tag->priority, logger_default_priority);
	atomic_set(&tag->enabled, logger_default_enabled);
	INIT_LIST_HEAD(&tag->list);

	spin_lock_irqsave(&log->taglist_lock, flags);
	list_add(&tag->list, &log->tags);
	spin_unlock_irqrestore(&log->taglist_lock, flags);

	memset(&tag->kobj, 0, sizeof(log->kobj));
	ret = kobject_init_and_add(&tag->kobj,
		&tag_ktype,
		&log->kobj,
		"%s", tag->name);
	if (!ret)
		return tag;

	if (ret > 0) /* Huh? Make sure it's negative! */
		ret = -EFAULT;

	kobject_put(&tag->kobj);

	spin_lock_irqsave(&log->taglist_lock, flags);
	list_del(&tag->list);
	spin_unlock_irqrestore(&log->taglist_lock, flags);

	kfree(tag);
out:
	return ERR_PTR(ret);
}

int check_tag(struct logger_log * const log,
		const unsigned char priority,
		const char *tag_name,
		const int tag_name_len)
{
	int ret = 0;
	if (tag_name_len > 1) {
		const struct logger_tag * const tag =
			get_tag(log, tag_name, tag_name_len);

		if (!tag || IS_ERR(tag))
			ret = !tag ? -ENODEV : PTR_ERR(tag);
		else if (atomic_read(&tag->enabled) &&
				atomic_read(&tag->priority) <= priority)
			ret = 1;
	}
	return ret;
}

/*
 * do_write_log - writes 'len' bytes from 'buf' to 'log'
 *
 * The caller needs to hold log->bufflock.
 */
static void do_write_log(struct logger_log *log, const void *buf, size_t count)
{
	size_t len;

	len = min(count, log->size - log->w_off);
	memcpy(log->buffer + log->w_off, buf, len);

	if (count != len)
		memcpy(log->buffer, buf + len, count - len);

	log->w_off = logger_offset(log->w_off + count);
}

static int write_log_entry(struct logger_log * const log,
		const unsigned char *priority,
		const char * const tag,
		const int tag_bytes,
		const char * const msg,
		const int msg_bytes)
{
	struct timespec now;
	struct logger_entry header;
	unsigned long flags;

	/*
	 * pid and tid may or may not be meaningful or relevant depending
	 * on where and how we got here from the logging driver. Might as
	 * well log them in any event, just in case.
	 */
	header.pid = current->tgid;
	header.tid = current->pid;

	now = current_kernel_time();
	header.sec = now.tv_sec;
	header.nsec = now.tv_nsec;

	header.len = min_t(size_t, tag_bytes + msg_bytes + 1,
			LOGGER_ENTRY_MAX_PAYLOAD);

	spin_lock_irqsave(&log->bufflock, flags);

	/*
	 * Fix up any readers, pulling them forward to the first readable
	 * entry after (what will be) the new write offset. We do this now
	 * because if we partially fail, we can end up with clobbered log
	 * entries that encroach on readable buffer.
	 */
	fix_up_readers(log, sizeof(struct logger_entry) + header.len);

	do_write_log(log, &header, sizeof(struct logger_entry));

	do_write_log(log, priority, 1);

	do_write_log(log, tag, tag_bytes);

	do_write_log(log, msg, msg_bytes);

	spin_unlock_irqrestore(&log->bufflock, flags);

	/* wake up any blocked readers */
	wake_up_interruptible(&log->wq);
	return header.len;
}

/*
 * logger_aio_write - our write method, implementing support for write(),
 * writev(), and aio_write(). Writes are our fast path, and we try to optimize
 * them above all else.
 */
ssize_t logger_aio_write(struct kiocb *iocb, const struct iovec *iov,
			 unsigned long nr_segs, loff_t ppos)
{
	struct logger_log * const log = file_get_log(iocb->ki_filp);
	ssize_t ret = 0;
	int i, total_log_bytes = min_t(size_t,
					iocb->ki_left,
					LOGGER_ENTRY_MAX_PAYLOAD);
	char *temp_buf, *curr_buf;

	/* null writes succeed, return zero */
	if (!total_log_bytes)
		return 0;

	if (!log)
		return -EFAULT;

	/* This API is heavily dependent on a user space assumption that
	 * the full log entry comprising 3 vectors will be passed to it in the
	 * format:
	 * (from user space logger file - system/core/liblog/logd_write.c):
	 *    vec[0].iov_base  = (unsigned char *) &prio;
	 *    vec[0].iov_len    = 1;
	 *    vec[1].iov_base   = (void *) tag;
	 *    vec[1].iov_len    = strlen(tag) + 1;
	 *    vec[2].iov_base   = (void *) msg;
	 *    vec[2].iov_len    = strlen(msg) + 1;
	 * Note: vec in userspace is "iov" here.
	 * Since this driver supplies a function for aio_write, there is
	 * no aio queueing or retry done. Once we are here we consume all of
	 * what is passed to us, with or without error. That means that no
	 * partial vector sets should ever be passed in.
	 */
	if (nr_segs < 3 || iov[0].iov_len != 1)
		return -EINVAL;

	if (nr_segs > 3)
		printk(KERN_WARNING
			"logger: possible misformatted iovec"
			" - extra ignored\n");
	/*
	 * Use GFP_KERNEL since we are guaranteed to be in user context here
	 * and can sleep if need be.
	 */
	temp_buf = kmalloc(total_log_bytes + sizeof(struct logger_entry),
			GFP_KERNEL);
	if (!temp_buf)
		return 	-ENOMEM;

	for (curr_buf = temp_buf, i = 0; i < 3; i++) {
		/* figure out how much of this vector we can keep */
		size_t len = min_t(size_t, iov[i].iov_len,
				total_log_bytes - ret);
		if (!len)
			break;

		if (copy_from_user(curr_buf, iov[i].iov_base, len)) {
			ret = -EFAULT;
			goto out_free_buf;
		}
		curr_buf += len;
	}

	/*
	 * Disabled writes, or not high enough priority writes succeed, return
	 * zero. Since the priority is a user pointer, it can't be checked
	 * until it is copied in from user space, so we can't do this any
	 * earlier, which means that the memory allocation must already have
	 * been done.
	 */
	if (!atomic_read(&log->enabled) ||
	    atomic_read(&log->priority)  > *(unsigned char *)temp_buf)
		goto out_free_buf;

	ret = check_tag(log,
		*(unsigned char *)temp_buf, /* priority */
		temp_buf + 1, /* tag name */
		iov[1].iov_len);
	if (ret <= 0)
		goto out_free_buf;

	ret = write_log_entry(log,
			temp_buf, /* priority */
			temp_buf + 1, /* tag */
			iov[1].iov_len, /* tag bytes */
			temp_buf + iov[1].iov_len + 1, /* message */
			iov[2].iov_len); /* message bytes */
out_free_buf:
	kfree(temp_buf);
	return ret;
}

static struct logger_log *get_log_from_minor(const int);

/*
 * logger_open - the log's open() file operation
 *
 * Note how near a no-op this is in the write-only case. Keep it that way!
 */
static int logger_open(struct inode *inode, struct file *file)
{
	unsigned long flags;
	struct logger_log * const log =
		get_log_from_minor(MINOR(inode->i_rdev));
	int ret = nonseekable_open(inode, file);
	if (ret)
		return ret;

	if (!log)
		return -ENODEV;

	if (file->f_mode & FMODE_READ) {
		/*
		 * GFP_KERNEL here because we are in user context and can
		 * sleep if necessary.
		 */
		struct logger_reader * const reader =
			kmalloc(sizeof *reader, GFP_KERNEL);
		if (!reader)
			return -ENOMEM;

		reader->log = log;
		INIT_LIST_HEAD(&reader->list);

		spin_lock_irqsave(&log->bufflock, flags);
		reader->r_off = log->head;
		list_add_tail(&reader->list, &log->readers);
		spin_unlock_irqrestore(&log->bufflock, flags);

		file->private_data = reader;
	} else
		file->private_data = log;

	return 0;
}

/*
 * logger_release - the log's release file operation
 *
 * Note this is a total no-op in the write-only case. Keep it that way!
 */
static int logger_release(struct inode *ignored, struct file *file)
{
	if (file->f_mode & FMODE_READ) {
		struct logger_reader * const reader = file->private_data;
		list_del(&reader->list);
		kfree(reader);
	}

	return 0;
}

/*
 * logger_poll - the log's poll file operation, for poll/select/epoll
 *
 * Note we always return POLLOUT, because you can always write() to the log.
 * Note also that, strictly speaking, a return value of POLLIN does not
 * guarantee that the log is readable without blocking, as there is a small
 * chance that the writer can lap the reader in the interim between poll()
 * returning and the read() request.
 */
static unsigned int logger_poll(struct file * const file,
				poll_table * const wait)
{
	struct logger_reader *reader;
	struct logger_log *log;
	unsigned int ret = POLLOUT | POLLWRNORM;
	unsigned long flags;

	if (!(file->f_mode & FMODE_READ))
		return ret;

	reader = file->private_data;
	log = reader->log;

	poll_wait(file, &log->wq, wait);

	spin_lock_irqsave(&log->bufflock, flags);
	if (log->w_off != reader->r_off)
		ret |= POLLIN | POLLRDNORM;
	spin_unlock_irqrestore(&log->bufflock, flags);
	
	return ret;
}

static void flush_log(struct logger_log * const log)
{
	struct logger_reader *reader;

	/* Expects log->bufflock to be held by caller */
	list_for_each_entry(reader, &log->readers, list)
		reader->r_off = log->w_off;
	log->head = log->w_off;
}

static long logger_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct logger_log * const log = file_get_log(file);
	struct logger_reader *reader;
	long ret = -ENOTTY;
	unsigned long flags;

	spin_lock_irqsave(&log->bufflock, flags);

	switch (cmd) {
	case LOGGER_GET_LOG_BUF_SIZE:
		ret = log->size;
		break;
	case LOGGER_GET_LOG_LEN:
		if (!(file->f_mode & FMODE_READ)) {
			ret = -EBADF;
			break;
		}
		reader = file->private_data;
		if (log->w_off >= reader->r_off)
			ret = log->w_off - reader->r_off;
		else
			ret = (log->size - reader->r_off) + log->w_off;
		break;
	case LOGGER_GET_NEXT_ENTRY_LEN:
		if (!(file->f_mode & FMODE_READ)) {
			ret = -EBADF;
			break;
		}
		reader = file->private_data;
		if (log->w_off != reader->r_off)
			ret = get_entry_len(log, reader->r_off);
		else
			ret = 0;
		break;
	case LOGGER_FLUSH_LOG:
		if (!(file->f_mode & FMODE_WRITE)) {
			ret = -EBADF;
			break;
		}
		flush_log(log);
		ret = 0;
		break;
	}

	spin_unlock_irqrestore(&log->bufflock, flags);

	return ret;
}

struct global_attr {
	struct attribute attr;
	ssize_t(*show) (struct logger_log * const, char * const);
	ssize_t(*store) (struct logger_log * const,
		const char * const, const size_t count);
};
#define to_global_attr(a) container_of(a, struct global_attr, attr)

#define RW_GLOBAL_ATTR(name)  \
static struct global_attr global_attr_## name = \
 LOGGER_ATTR(name, \
	S_IRUGO | S_IWUSR, \
	show_global_## name, \
	store_global_## name)

#define RO_GLOBAL_ATTR(name)  \
static struct global_attr global_attr_## name = \
 LOGGER_ATTR(name, S_IRUGO, show_global_## name, NULL)

#define WO_GLOBAL_ATTR(name)  \
static struct global_attr global_attr_## name = \
 LOGGER_ATTR(name, S_IWUSR, NULL, store_global_## name)

static ssize_t show_global(struct kobject *kobj,
			struct attribute *attr,
			char *buf)
{
	struct global_attr *a = to_global_attr(attr);
	return a->show ? a->show(to_log(kobj), buf) : -EIO;
}

static ssize_t store_global(struct kobject *kobj, struct attribute *attr,
		     const char *buf, size_t count)
{
	struct global_attr *a = to_global_attr(attr);
	return a->store ? a->store(to_log(kobj), buf, count) : -EIO;
}

static struct sysfs_ops global_ops = {
	.show = show_global,
	.store = store_global,
};

static ssize_t show_global_enable(struct logger_log * const log, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&log->enabled));
}

static ssize_t store_global_enable(struct logger_log * const log,
			const char * const buf,
			const size_t count)
{
	int enabled;

	if (sscanf(buf, "%d", &enabled) != 1)
		return -EINVAL;

	atomic_set(&log->enabled, enabled);
	return strnlen(buf, count);
}
RW_GLOBAL_ATTR(enable);

static ssize_t show_global_priority(struct logger_log * const log,
			char * const buf)
{
	return snprintf(buf, PAGE_SIZE,
			"%d\n", atomic_read(&log->priority));
}

static ssize_t store_global_priority(struct logger_log * const log,
			const char * const buf,
			const size_t count)
{
	unsigned int priority;

	if (sscanf(buf, "%u", &priority) != 1 || priority > LOG_PRIORITY_SILENT)
		return -EINVAL;

	atomic_set(&log->priority, priority);
	return strnlen(buf, count);
}
RW_GLOBAL_ATTR(priority);

static ssize_t store_global_flush(struct logger_log * const log,
			const char * const buf,
			const size_t count)
{
	unsigned long flags;

	spin_lock_irqsave(&log->bufflock, flags);
	flush_log(log);
	spin_unlock_irqrestore(&log->bufflock, flags);
	return strnlen(buf, count);
}
WO_GLOBAL_ATTR(flush);

static ssize_t show_global_was_overrun(struct logger_log * const log,
				char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n",
			atomic_xchg(&log->was_overrun, 0));
}
RO_GLOBAL_ATTR(was_overrun);

static struct attribute *global_attrs[] = {
	&global_attr_enable.attr,
	&global_attr_priority.attr,
	&global_attr_flush.attr,
	&global_attr_was_overrun.attr,
	NULL
};

static struct kobj_type global_ktype = {
	.sysfs_ops = &global_ops,
	.default_attrs = global_attrs,
};

static struct file_operations logger_fops = {
	.owner = THIS_MODULE,
	.read = logger_read,
	.aio_write = logger_aio_write,
	.poll = logger_poll,
	.unlocked_ioctl = logger_ioctl,
	.compat_ioctl = logger_ioctl,
	.open = logger_open,
	.release = logger_release,
};

/*
 * Defines a log structure with name 'NAME' and a size of 'SIZE' bytes, which
 * must be a power of two, greater than LOGGER_ENTRY_MAX_LEN, and less than
 * LONG_MAX minus LOGGER_ENTRY_MAX_LEN.
 */
#define DEFINE_LOGGER_DEVICE(VAR, NAME, SIZE) \
static unsigned char _buf_ ## VAR[SIZE]; \
static struct logger_log VAR = { \
	.buffer = _buf_ ## VAR, \
	.misc = { \
		.minor = MISC_DYNAMIC_MINOR, \
		.name = NAME, \
		.fops = &logger_fops, \
		.parent = NULL, \
	}, \
	.wq = __WAIT_QUEUE_HEAD_INITIALIZER(VAR .wq), \
	.readers = LIST_HEAD_INIT(VAR .readers), \
	.bufflock = __SPIN_LOCK_UNLOCKED(VAR .bufflock), \
	.w_off = 0, \
	.head = 0, \
	.size = SIZE, \
	.was_overrun = ATOMIC_INIT(0), \
	.taglist_lock = __SPIN_LOCK_UNLOCKED(VAR .taglist_lock), \
	.tags = LIST_HEAD_INIT(VAR .tags), \
};

DEFINE_LOGGER_DEVICE(log_main, LOGGER_LOG_MAIN, 64*1024)
DEFINE_LOGGER_DEVICE(log_events, LOGGER_LOG_EVENTS, 256*1024)
DEFINE_LOGGER_DEVICE(log_radio, LOGGER_LOG_RADIO, 64*1024)

/* Number of elements in this array must match 'enum logidx' */
static struct logger_log *logs[] = {
	&log_main,
	&log_events,
	&log_radio,
};

static struct logger_log *get_log_from_minor(const int minor)
{
	if (log_main.misc.minor == minor)
		return &log_main;
	if (log_events.misc.minor == minor)
		return &log_events;
	if (log_radio.misc.minor == minor)
		return &log_radio;
	return NULL;
}

static int kernel_logger_write(struct logger_log * const log,
				const unsigned char priority,
				const char __kernel * const tag,
				const char __kernel * const fmt,
				const va_list args)
{
	int ret = 0, tag_bytes = strlen(tag) + 1, msg_bytes;
	char *msg;

	if (!atomic_read(&log->enabled) ||
			atomic_read(&log->priority) > priority ||
			(ret = check_tag(log,
					 priority,
					 tag,
					 tag_bytes)) <= 0)
		return ret;

	/*
	 * Use GFP_ATOMIC due to inability to reliably know if we
	 * are able to sleep while allocating memory here or not.
	 */
	msg = kvasprintf(GFP_ATOMIC, fmt, args);
	if (!msg)
		return -ENOMEM;

	msg_bytes = strlen(msg) + 1;
	if (msg_bytes <= 1) /* empty message? */
		goto out_free_message; /* don't bother, then */

	if ((msg_bytes + tag_bytes + 1) > LOGGER_ENTRY_MAX_PAYLOAD) {
		ret = -E2BIG;
		goto out_free_message;
	}

	ret = write_log_entry(log,
			&priority,
			tag,
			tag_bytes,
			msg,
			msg_bytes);
out_free_message:
	kfree(msg);
	return ret;
}

int logger_write(const enum logidx index,
		const unsigned char prio,
		const char __kernel * const tag,
		const char __kernel * const fmt,
		...)
{
	va_list vargs;
	int ret;

	if (index >= 0 && index < LOG_INVALID_IDX) {
		va_start(vargs, fmt);
		ret = kernel_logger_write(logs[index], prio, tag, fmt, vargs);
		va_end(vargs);
	} else {
		ret = -EINVAL;
	}
	return ret;
}
EXPORT_SYMBOL(logger_write);

static void __init destroy_log(struct logger_log * const log)
{
	kobject_put(&log->kobj);
	misc_deregister(&log->misc);
}

static int __init init_log(struct logger_log * const log)
{
	int ret;

	atomic_set(&log->priority, logger_default_priority);
	atomic_set(&log->enabled, logger_default_enabled);

	ret = misc_register(&log->misc);
	if (unlikely(ret)) {
		printk(KERN_ERR "logger: failed to register misc "
		       "device for log '%s', err: %d!\n",
			log->misc.name, ret);
		goto out;
	}
	memset(&log->kobj, 0, sizeof(log->kobj));
	log->kobj.kset = logger_kset;
	ret = kobject_init_and_add(&log->kobj,
				&global_ktype,
				NULL,
				"%s", log->misc.name);
	if (ret)
		goto out_put_kobj;

	printk(KERN_INFO "logger: created %luK log '%s'\n",
	       (unsigned long) log->size >> 10, log->misc.name);
	goto out;

out_put_kobj:
	kobject_put(&log->kobj);
	misc_deregister(&log->misc);
out:
	return ret;
}

static int __init logger_init(void)
{
	int ret;

	if (logger_default_priority > LOG_PRIORITY_SILENT ||
		logger_default_enabled <= 0 ||
		logger_default_enabled > 1) {
		ret = -EINVAL;
		goto out;
	}

	/* create /sys/kernel/logger directory */
	logger_kset = kset_create_and_add(DEV_NAME, NULL, kernel_kobj);
	if (!logger_kset) {
		ret = -ENOMEM;
		printk(KERN_ERR "logger(%s):kset_create_and_add fail\n",
			__func__);
		goto out;
	}

	ret = init_log(&log_main);
	if (unlikely(ret))
		goto out_unregister_kset;

	ret = init_log(&log_events);
	if (unlikely(ret))
		goto out_destroy_main;

	ret = init_log(&log_radio);
	if (unlikely(ret))
		goto out_destroy_events;

/* leave room for more init functionality */
	goto out;

out_destroy_events:
	destroy_log(&log_events);
out_destroy_main:
	destroy_log(&log_main);
out_unregister_kset:
	kset_unregister(logger_kset);
out:
	return ret;
}
device_initcall(logger_init);
