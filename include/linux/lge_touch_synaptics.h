/* drivers/input/keyboard/lge_touch_driver.h
 *
 * Copyright (C) 2011 LGE. 
 * 
 * Writer: yehan.ahn@lge.com
 *
 * This file is used by LGE_touch_synaptics.c.
 * it is related with specific touch-device.
 * You should check the document or contact the writer in order to understand whole process.
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

#ifndef INCLUDED_TOUCH_SYNAPTICS_H
#define INCLUDED_TOUCH_SYNAPTICS_H

#ifndef DEFINE_TOUCH_NAME
#define DEFINE_TOUCH_NAME
#define LGE_TOUCH_NAME "lge_synaptics"
#endif

#define LGE_TOUCH_ADDR 0x20

struct lge_synaptics_platform_data {
	u32 gpio;	
	int (*power)(char* reg_id, bool on);	
	unsigned long irqflags;
};

#define LGE_TOUCH_RESOLUTION_X				480
#define LGE_TOUCH_RESOLUTION_Y				800

#define SYNAPTICS_TOUCH_DEVICE_GUID				NV_ODM_GUID('s','y','n','t','o','u','c','h')

#define SYNAPTICS_I2C_SPEED_KHZ					400
#define SYNAPTICS_I2C_TIMEOUT					10
#define SYNAPTICS_I2C_RETRY_COUNT				5
#define SYNAPTICS_LOW_SAMPLE_RATE				0		//40 reports per-second
#define SYNAPTICS_HIGH_SAMPLE_RATE				1		//80 reports per-second

#define SYNAPTICS_SCREEN_ANGLE_MODE				1		//0=Landscape, 1=Portrait
#define SYNAPTICS_POR_DELAY_MS					100		//Delay after Power-On Reset
#define SYNAPTICS_DEBOUNCE_TIME_MS				0
#define SYNAPTICS_FINGER_MAX					10

#define SYNAPTICS_MELT_SUPPORT_VER				4

#define SYNAPTICS_NEW_PANEL_BASE_FW_VER			11

#define SYNAPTICS_DELTA_THRESHOLD				0x01

#define SYNAPTICS_FLASH_CONTROL_REG				0x12
#define SYNAPTICS_DATA_BASE_REG					0x13
#define SYNAPTICS_INT_STATUS_REG				0x14

#define SYNAPTICS_REG_FINGER_DATA_START_ADDR	0x18

#define SYNAPTICS_DEVICE_CONTROL_REG			0x4F
#define SYNAPTICS_INTERRUPT_ENABLE_REG			0x50
#define SYNAPTICS_REPORT_MODE_REG				0x51
#define SYNAPTICS_PALM_DETECT_REG				0x52
#define SYNAPTICS_DELTA_X_THRES_REG				0x53
#define SYNAPTICS_DELTA_Y_THRES_REG				0x54
#define SYNAPTICS_VELOCITY_REG					0x55
#define SYNAPTICS_ACCELERATION_REG				0x56
#define SYNAPTICS_MAX_X_POSITION_LOW_REG		0x57
#define SYNAPTICS_MAX_X_POSITION_HIGH_REG		0x58
#define SYNAPTICS_MAX_Y_POSITION_LOW_REG		0x59
#define SYNAPTICS_MAX_Y_POSITION_HIGH_REG		0x5A
#define SYNAPTICS_2D_GESTURE_ENABLE1			0x5B
#define SYNAPTICS_2D_GESTURE_ENABLE2			0x5C

#define SYNAPTICS_MAX_TAP_TIME_REG				0x9A
#define SYNAPTICS_MIN_PRESS_TIME_REG			0x9B
#define SYNAPTICS_MIN_TAP_DIST_REG				0x9C
#define SYNAPTICS_MIN_FLICK_DIST_REG			0x9D
#define SYNAPTICS_MIN_FLICK_SPEED_REG			0x9E

#define SYNAPTICS_RMI_QUERY_BASE_REG			0xE3
#define SYNAPTICS_RMI_CMD_BASE_REG				0xE4
#define SYNAPTICS_FLASH_QUERY_BASE_REG			0xE9
#define SYNAPTICS_FLASH_DATA_BASE_REG			0xEC

#define SYNAPTICS_MELT_CONTROL					0xF0

#define SYNAPTICS_INT_FLASH						1<<0
#define SYNAPTICS_INT_STATUS 					1<<1
#define SYNAPTICS_INT_ABS0 						1<<2

#define SYNAPTICS_CONTROL_SLEEP 				(1<<0)
#define SYNAPTICS_CONTROL_NOSLEEP				(1<<2)

#define SYNAPTICS_REG_FINGER_DATA_GAP			0x05
#define SYNAPTICS_REG_FINGER_VALID_DATA_SIZE	0x05

#define SYNAPTICS_NO_MELT						0x00
#define SYNAPTICS_MELT							0x01

#define SYNAPTICS_FLASH_CMD_FW_CRC				0x01
#define SYNAPTICS_FLASH_CMD_FW_WRITE			0x02
#define SYNAPTICS_FLASH_CMD_ERASEALL			0x03
#define SYNAPTICS_FLASH_CMD_CONFIG_READ			0x05
#define SYNAPTICS_FLASH_CMD_CONFIG_WRITE		0x06
#define SYNAPTICS_FLASH_CMD_CONFIG_ERASE		0x07
#define SYNAPTICS_FLASH_CMD_ENABLE				0x0F
#define SYNAPTICS_FLASH_NORMAL_RESULT			0x80

#endif /* _LINUX_STAR_SYNAPTICS_H*/

