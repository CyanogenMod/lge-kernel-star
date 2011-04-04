/**===================================================================
 * Copyright(c) 2009 LG Electronics Inc. All Rights Reserved
 *
 * File Name : broadcast_lg2102.h
 * Description : EDIT HISTORY FOR MODULE
 * This section contains comments describing changes made to the module.
 * Notice that changes are listed in reverse chronological order.
 *
 * when			model		who			what
 * 10.27.2009		android		inb612		Create for Android platform
====================================================================**/
#ifndef _BROADCAST_LG2102_H_
#define _BROADCAST_LG2102_H_
#include <linux/broadcast/broadcast_tdmb_typedef.h>

int tdmb_lg2102_power_on(void);
int tdmb_lg2102_power_off(void);
int tdmb_lg2102_i2c_write_burst(uint16 waddr, uint8* wdata, int length);
int tdmb_lg2102_i2c_read_burst(uint16 raddr, uint8* rdata, int length);
int tdmb_lg2102_mdelay(int32 ms);
void tdmb_lg2102_interrupt_lock(void);
void tdmb_lg2102_interrupt_free(void);
int tdmb_lg2102_spi_write_read(uint8* tx_data, int tx_length, uint8 *rx_data, int rx_length);
void tdmb_lg2102_set_userstop(void);
void tdmb_lg2102_select_antenna(int mode);
#endif

