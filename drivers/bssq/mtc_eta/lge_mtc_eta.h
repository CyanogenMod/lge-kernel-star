#ifndef _LGE_MTC_ETA_H_
#define _LGE_MTC_ETA_H_

#ifdef __KERNEL__
#include <linux/types.h>
#include <asm/sizes.h>
#include <linux/ioctl.h>
#include <linux/slab.h>
#else
#include <stdint.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <stdlib.h>
#endif  

#define LGE_ETA_IOC_MAGIC 'l'
#define LGE_ETA_IOC_CHANGE_LOG_MASK _IOW(LGE_ETA_IOC_MAGIC, 0, unsigned int)
#define LGE_ETA_IOC_GETLOG _IOR(LGE_ETA_IOC_MAGIC, 1, struct mtc_eta_log *)

struct log_data_key_event {
	int hold;
	unsigned int key_code;
};

struct log_data_touch_event {
	unsigned int action;
	int x;
	int y;
};

struct mtc_eta_log {
        int id;
	unsigned int time;
	union {
		struct log_data_key_event key;
		struct log_data_touch_event touch;
	} data;
};

#ifdef __KERNEL__
struct mtc_eta_event_log_q_entry {
	struct mtc_eta_log *log;
	struct list_head list;
};
#endif

enum {
        MTC_ETA_LOG_ID_KEY = 1,
        MTC_ETA_LOG_ID_TOUCH = 2,
        MTC_ETA_LOG_ID_MAX,
};

enum {
	ETA_TOUCH_MOVETO = 0, /*Move the pointer to the specified location*/
	ETA_TOUCH_MOVEBY = 1, /*Move the pointer by the specified values*/
	ETA_TOUCH_TAB = 2, /*Tab at the current location*/
	ETA_TOUCH_DOUBLETAB = 3, /*Double tab at the current location*/
	ETA_TOUCH_DOWN = 4, /*Touch down at the current location*/
	ETA_TOUCH_UP = 5, /*Touch up at the current location*/
	ETA_TOUCH_DEFAULT = 0xff,
};
#endif /*_LGE_MTC_ETA_H_*/

