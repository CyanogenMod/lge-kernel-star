/*
*/

#ifndef __BU9889GULW_MAIN_H__
#define __BU9889GULW_MAIN_H__

#include <linux/ioctl.h> /* For IOCTL macros */

#define BU9889GULW_IOCTL_GET_E2PROM_DATA		_IOW('o', 1, struct bu9889gulw_register_info)
#define BU9889GULW_IOCTL_PUT_E2PROM_DATA		_IOW('o', 2, struct bu9889gulw_register_info)

struct bu9889gulw_register_info {
	u8 e2prom_data[1024];
	u16 reg_addr;
	u16 length;
} __attribute__ ((packed));

struct bu9889gulw_info {
	struct i2c_client *i2c_client;
	struct bu9889gulw_register_info reg_info;
} __attribute__ ((packed));


#ifdef __KERNEL__
struct bu9889gulw_platform_data {
	int (*power_on)(void);
	int (*power_off)(void);

};
#endif /* __KERNEL__ */

#endif
