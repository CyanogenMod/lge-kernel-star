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

#include "../../broadcast_tdmb_typedef.h"

int tdmb_lg2102_power_on(void);
int tdmb_lg2102_power_off(void);
void tdmb_lg2102_must_mdelay(int32 ms);
int tdmb_lg2102_mdelay(int32 ms);
void tdmb_lg2102_interrupt_lock(void);
void tdmb_lg2102_interrupt_free(void);
int tdmb_lg2102_spi_write_read(uint8* tx_data, int tx_length, uint8 *rx_data, int rx_length);
void tdmb_lg2102_set_userstop(int mode);
int tdmb_lg2102_select_antenna(int mode);
#endif

