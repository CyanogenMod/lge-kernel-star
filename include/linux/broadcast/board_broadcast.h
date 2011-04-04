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
#ifndef _BOARD_BROADCAST_H_
#define _BOARD_BROADCAST_H_
#include "nvrm_gpio.h"
#include "nvodm_query.h"
#include "nvodm_query_pinmux.h"
#include "nvodm_query_discovery.h"
#include "nvodm_query_gpio.h"
#include "nvrm_pinmux.h"

struct broadcast_tdmb_data
{
	NvOdmGpioPinHandle hNVODM_DmbIntPin;
};

#endif

