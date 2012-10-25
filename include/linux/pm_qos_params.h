#ifndef _LINUX_PM_QOS_PARAMS_H
#define _LINUX_PM_QOS_PARAMS_H
/* interface for the pm_qos_power infrastructure of the linux kernel.
 *
 * Mark Gross <mgross@linux.intel.com>
 */
#include <linux/plist.h>
#include <linux/notifier.h>
#include <linux/miscdevice.h>

enum {
	PM_QOS_RESERVED = 0,
	PM_QOS_CPU_DMA_LATENCY,
	PM_QOS_NETWORK_LATENCY,
	PM_QOS_NETWORK_THROUGHPUT,
	PM_QOS_MIN_ONLINE_CPUS,
	PM_QOS_MAX_ONLINE_CPUS,
	PM_QOS_CPU_FREQ_MIN,
	PM_QOS_CPU_FREQ_MAX,

	/* insert new class ID */

	PM_QOS_NUM_CLASSES,
};

#define PM_QOS_DEFAULT_VALUE -1

#define PM_QOS_CPU_DMA_LAT_DEFAULT_VALUE	(2000 * USEC_PER_SEC)
#define PM_QOS_NETWORK_LAT_DEFAULT_VALUE	(2000 * USEC_PER_SEC)
#define PM_QOS_NETWORK_THROUGHPUT_DEFAULT_VALUE	0
#define PM_QOS_MIN_ONLINE_CPUS_DEFAULT_VALUE	1
#define PM_QOS_MAX_ONLINE_CPUS_DEFAULT_VALUE	LONG_MAX
#define PM_QOS_CPU_FREQ_MIN_DEFAULT_VALUE	0
#define PM_QOS_CPU_FREQ_MAX_DEFAULT_VALUE	LONG_MAX

struct pm_qos_request_list {
	struct plist_node list;
	int pm_qos_class;
};

void pm_qos_add_request(struct pm_qos_request_list *l, int pm_qos_class, s32 value);
void pm_qos_update_request(struct pm_qos_request_list *pm_qos_req,
		s32 new_value);
void pm_qos_remove_request(struct pm_qos_request_list *pm_qos_req);

int pm_qos_request(int pm_qos_class);
int pm_qos_add_notifier(int pm_qos_class, struct notifier_block *notifier);
int pm_qos_remove_notifier(int pm_qos_class, struct notifier_block *notifier);
int pm_qos_request_active(struct pm_qos_request_list *req);

#endif
