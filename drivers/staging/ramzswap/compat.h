#ifndef _RAMZSWAP_COMPAT_H_
#define _RAMZSWAP_COMPAT_H_

/* Uncomment this if you are using swap free notify patch */
//#define CONFIG_SWAP_FREE_NOTIFY

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,31))
#define blk_queue_physical_block_size(q, size) \
	blk_queue_hardsect_size(q, size)
#define blk_queue_logical_block_size(q, size)
#endif

#endif
