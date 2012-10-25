/* drivers/input/keyboard/LGE_touch_driver.h
 *
 * Copyright (C) 2011 LGE. 
 * 
 * Writer: yehan.ahn@lge.com
 *
 * This device driver can be used for any touch device
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

#ifndef LGE_TOUCH_DRIVER_H
#define LGE_TOUCH_DRIVER_H

#include <linux/platform_device.h>

#ifndef DEFINE_TOUCH_NAME
#define DEFINE_TOUCH_NAME
#define LGE_TOUCH_NAME "star_synaptics"
#endif

#define SYNAPTICS_TOUCH_DEBUG_LEVEL 3

#ifdef SYNAPTICS_TOUCH_DEBUG_LEVEL
#define DEBUG_MSG(_level, args...)  \
		if(_level >= SYNAPTICS_TOUCH_DEBUG_LEVEL)	\
			printk(KERN_INFO args);
#else
#define DEBUG_MSG(_level, args...)
#endif

enum{M=1,	// touch_movement_event_log || check_some_log
	 B,		// touch_button_event_log || sync_event_log
	 E,		// essential
	 ERR,	//error
	 NOTHING
};

#define DO_SAFE(_do, _log, _err_handling)	\
if(unlikely(_do)){	\
	DEBUG_MSG(ERR, "[TOUCH] ERROR : %s[%u] %s\n", __FUNCTION__, __LINE__, _log);	\
	_err_handling;	\
}

// do safely when allocate...
#define DO_A(_statement, _jump_place)	DO_SAFE((_statement) == NULL, "", goto _jump_place)

// do safely when call the function...
#define DO_F(_statement, _jump_place)	DO_SAFE((_statement) == 0, "", goto _jump_place)

// do safely when compare...
#define DO_C(_statement, _jump_place)	DO_SAFE((_statement), "", goto _jump_place)

#define SYNAPTICS_POWERMODE_0	0x0
#define SYNAPTICS_POWERMODE_1	0x1
#define SYNAPTICS_POWERMODE_2	0x2
#define SYNAPTICS_POWERMODE_3	0x3

enum {KEY_NULL=0, KEY_PANEL, KEY_BOUNDARY};
enum {BEFORE_WHILE=1, AFTER_GET_DATA, BEFORE_SYNC, AFTER_SYNC};
enum {FINGER_RELEASE=0, SINGLE_FINGER, MULTI_FINGER};
enum {DO_NOT_ANYTHING=0, ABS_PRESS, ABS_RELEASE, BUTTON_PRESS, BUTTON_RELEASE, TOUCH_LOCK};


typedef void* LGE_Device_Handle;

typedef struct
{
    bool 	IsMultiTouchSupported; // single touch[0] / multi-touch[1]
    bool	IsButtonSupported; // yes[1] / no[0]
    u32 	MaxNumberOfFingerCoordReported;
    bool	IsRelativeDataSupported; // support[1] / not[0]
    u32 	MaxNumberOfRelativeCoordReported;
    u32 	MaxNumberOfWidthReported;
    u32 	MaxNumberOfPressureReported;
    u32 	Gesture;
    bool 	IsWidthSupported;
    bool 	IsPressureSupported;
    bool 	IsFingersSupported;
    u32 	XMinPosition;
    u32 	YMinPosition;
    u32 	XMaxPosition;
    u32 	YMaxPosition;
    u32 	Orientation;
} LGE_Touch_Device_Capabilities;
typedef LGE_Touch_Device_Capabilities* LGE_Touch_DeviceCap_Handle;

typedef struct {
	u16 X_position;
	u16 Y_position;
	u16 width;
	u16 pressure;
} ts_finger_data;

typedef struct {
	u8		total_num;
	ts_finger_data	curr_data[10];
	ts_finger_data	prev_data[10];
	u8		curr_button;
	u8		prev_button;
	u8		state;
} LGE_Touch_FingerData;

typedef struct {
	bool	(*get_finger_data)	(LGE_Device_Handle h_dev, LGE_Touch_FingerData* data);
	u8		(*check_button)		(LGE_Device_Handle h_dev, LGE_Touch_FingerData* data);
	bool	(*additional_job)	(LGE_Device_Handle h_dev, LGE_Touch_FingerData* data, u32 whereis);
} LGE_Touch_FingerData_Func;
typedef LGE_Touch_FingerData_Func* LGE_Touch_FingerData_Handle;

typedef struct{
	bool	(*open)				(LGE_Device_Handle h_dev);
	void	(*close)			(LGE_Device_Handle h_dev);
	bool	(*send_ABS)			(LGE_Device_Handle h_dev, ts_finger_data data, u8 isPress);
	bool	(*send_ABS_Multi)	(LGE_Device_Handle h_dev, ts_finger_data* data, u8 total_num);
	bool	(*send_Button)		(LGE_Device_Handle h_dev, u32 type, bool isPress);
	bool 	(*send_Sync)		(LGE_Device_Handle h_dev, u8 state);
} LGE_Touch_InputDev;
typedef LGE_Touch_InputDev* LGE_Touch_InputDev_Handle;

typedef struct{
	bool	(*open)		(LGE_Device_Handle h_dev);
	void	(*close)	(LGE_Device_Handle h_dev);
	bool	(*start)	(LGE_Device_Handle h_dev);
	bool	(*stop)		(LGE_Device_Handle h_dev);
} LGE_Touch_Task;
typedef LGE_Touch_Task* LGE_Touch_Task_Handle;

typedef struct{
	bool	(*open)		(LGE_Device_Handle h_dev);
	void	(*close)	(LGE_Device_Handle h_dev);
	bool	(*enable)	(LGE_Device_Handle h_dev);
	bool	(*disable)	(LGE_Device_Handle h_dev);
	bool	(*start)	(LGE_Device_Handle h_dev);
	bool	(*wait) 	(LGE_Device_Handle h_dev);
} LGE_Touch_Interrupt;
typedef LGE_Touch_Interrupt* LGE_Touch_Interrupt_Handle;

typedef struct{
	bool	(*on)		(LGE_Device_Handle h_dev);
	bool	(*off)		(LGE_Device_Handle h_dev);
} LGE_Touch_PowerCtrl;
typedef LGE_Touch_PowerCtrl* LGE_Touch_PowerCtrl_Handle;

typedef struct{
	bool	(*control)	(LGE_Device_Handle h_dev, bool onoff, u8 mode);
} LGE_Touch_SleepMode;
typedef LGE_Touch_SleepMode* LGE_Touch_SleepMode_Handle;

bool Device_Open(LGE_Device_Handle* h_touch, struct i2c_client *client, u32 touch);
void Device_Close(LGE_Device_Handle h_dev);

void Init_InputDev(LGE_Device_Handle h_dev, LGE_Touch_InputDev_Handle h_inputDev);
void Init_Task(LGE_Device_Handle h_dev, LGE_Touch_Task_Handle h_task);
void Init_Interrupt(LGE_Device_Handle h_dev, LGE_Touch_Interrupt_Handle h_interrupt);
void Init_PowerCtrl(LGE_Device_Handle h_dev, LGE_Touch_PowerCtrl_Handle h_powerCtrl);
void Init_FingerData(LGE_Device_Handle h_dev, LGE_Touch_FingerData_Handle h_fingerData);
void Init_DeviceCap(LGE_Device_Handle h_dev, LGE_Touch_DeviceCap_Handle h_caps);
bool Init_Specific_Device_Setting(LGE_Device_Handle h_dev);
void Init_SleepMode(LGE_Device_Handle h_dev, LGE_Touch_SleepMode_Handle h_sleepMode);

irqreturn_t Interrupt_Handler(int irq, void *dev_id);
int  Task_Handler(void* arg);

#endif
