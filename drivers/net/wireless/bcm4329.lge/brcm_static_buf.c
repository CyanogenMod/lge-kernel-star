#include <linux/module.h>
#include <linux/init.h>
#include <linux/memory.h>
#include <linux/skbuff.h>
#define DEBUG
#ifdef DEBUG
#define DBG(args)     printk args
#else
#define DBG(args)
#endif

#define USE_STATIC_SKB	/* Use DHD_USE_STATIC_BUF at SKB */

#define MAX_WIFI_SECTION		4
#define MAX_STATIC_PKT_NUM		8

#define MAX_STATIC_BUF_NUM 16
#define STATIC_BUF_SIZE	(PAGE_SIZE*2)
#define STATIC_BUF_TOTAL_LEN (MAX_STATIC_BUF_NUM*STATIC_BUF_SIZE)

#define MAX_DATA_BUF	(32 * 1024)	/* Must be large enough to hold biggest possible glom */
#define MAX_OTH_BUF		(12 * 1024)
	
struct mem_prealloc_data{
	char * data;
	int use;
	unsigned long size;
};
#ifdef USE_STATIC_SKB
struct mem_prealloc_skb{
	struct sk_buff *skb_4k[MAX_STATIC_PKT_NUM];
	struct sk_buff *skb_12k[MAX_STATIC_PKT_NUM];
};
#endif
struct mem_prealloc_data 	wifi_alloc_data[MAX_WIFI_SECTION];
#ifdef USE_STATIC_SKB
struct mem_prealloc_skb 	wifi_alloc_skb;
#endif

void * mem_prealloc(int section, unsigned long size)
{	
	DBG(("%s: section [%d] size[%ld]\n",__func__,section, size));
	if( section > MAX_WIFI_SECTION ){
		DBG(("%s: all allocation section is use !!!\n",__func__));
		return NULL;
	}
#ifdef USE_STATIC_SKB
	if( section == 4 ){
		return (void *)(&wifi_alloc_skb);
	}else
#endif
	if( wifi_alloc_data[section].use == 0 ){
		if((wifi_alloc_data[section].data = kmalloc(size, GFP_KERNEL)) == NULL){
			DBG(("%s: memory allocation fail !!!\n",__func__));
			return NULL;
		}
		wifi_alloc_data[section].use = 1;
		wifi_alloc_data[section].size = size;
	}else if (wifi_alloc_data[section].use == 1){
		if(wifi_alloc_data[section].size >= size)
			DBG(("%s: This data is using!!!\n",__func__));
		else if(wifi_alloc_data[section].size < size) {
			DBG(("%s: Size is wrong!!!\n",__func__));
			kfree(wifi_alloc_data[section].data);
			if((wifi_alloc_data[section].data = kmalloc(size, GFP_KERNEL)) == NULL){
				DBG(("%s: memory allocation fail !!!\n",__func__));
				return NULL;
			}
			wifi_alloc_data[section].size = size;
		}
	}
	return (void *)(wifi_alloc_data[section].data);
}
EXPORT_SYMBOL(mem_prealloc);

static int __init brcm_static_buf_init(void)
{
	int i;
	unsigned long size = 0;
	DBG(("%s: initialize wifi_alloc_data\n",__func__));
	memset(&wifi_alloc_data, 0, sizeof(struct mem_prealloc_data)*MAX_WIFI_SECTION);
	
	for(i =(MAX_WIFI_SECTION-1); 0 <= i ; i--){
		switch( i ){
			case 0:
			case 1:
				size=MAX_OTH_BUF;
				break;
			case 2:
				size=MAX_DATA_BUF;
				break;
			case 3:
				size=STATIC_BUF_SIZE + STATIC_BUF_TOTAL_LEN;
				break;
		}

		if( (wifi_alloc_data[i].data = kmalloc(size, GFP_KERNEL)) ==NULL){ 
			printk("Section [%d] initial allocation fail!!!\n",i);
		}else
		{
			wifi_alloc_data[i].use = 1;
			wifi_alloc_data[i].size = size;
			printk("Section [%d] initial allocation success size[%ld]!!!\n"
					,i, wifi_alloc_data[i].size);

		}
		size =0;
	}
#ifdef USE_STATIC_SKB
	for(i=0 ; i < MAX_STATIC_PKT_NUM; i++){
		if(!(wifi_alloc_skb.skb_4k[i] = dev_alloc_skb(PAGE_SIZE))){
			printk("%s: skb_4k [%d] allocation fail!!!\n",__func__, i);
		}
	}
	for(i=0 ; i < MAX_STATIC_PKT_NUM; i++){
		if(!(wifi_alloc_skb.skb_12k[i] = dev_alloc_skb(PAGE_SIZE*3))){
			printk("%s: skb_12k [%d] allocation fail!!!\n",__func__, i);
		}
	}
#endif
	return 0;
}

static void __exit brcm_static_buf_exit(void)
{
	int i;
	DBG(("%s: exit ~~  wifi_alloc_data\n",__func__));
	
	for ( i =0 ; i < MAX_WIFI_SECTION ; i++){
		if (wifi_alloc_data[i].use == 1)
			kfree(wifi_alloc_data[i].data);
	}
#ifdef USE_STATIC_SKB
	for(i=0 ; i < MAX_STATIC_PKT_NUM; i++)
		dev_kfree_skb(wifi_alloc_skb.skb_4k[i]);
	for(i=0 ; i < MAX_STATIC_PKT_NUM; i++)
		dev_kfree_skb(wifi_alloc_skb.skb_12k[i]);
#endif	
	return;
}

module_init(brcm_static_buf_init);
module_exit(brcm_static_buf_exit);
