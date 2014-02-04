/*
 * drivers/input/touchscreen/tegra_odm.c
 *
 * Touchscreen class input driver for platforms using NVIDIA's Tegra ODM kit
 * driver interface
 *
 * Copyright (c) 2009, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */
#include <linux/module.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/freezer.h>
//#include <linux/tegra_devices.h>
#include <nvodm_services.h>
#include <nvodm_touch.h>


#define NVODM_TOUCH_NAME "nvodm_touch"

#define READ_BUFFER_LENGTH 20
#define swapv(x, y) do { typeof(x) z = x; x = y; y = z; } while (0)


struct tegra_touch_driver_data
{
	struct input_dev	*input_dev;
	struct task_struct	*task;
	NvOdmOsSemaphoreHandle	semaphore;
	NvOdmOsMutexHandle hMutex;
	NvOdmTouchDeviceHandle	hTouchDevice;
	NvBool			bPollingMode;
	NvU32			pollingIntervalMS;
	NvOdmTouchCapabilities	caps;
	NvU32			MaxX;
	NvU32			MinX;
	NvU32			MaxY;
	NvU32			MinY;
	int			shutdown;
	struct early_suspend	early_suspend;
	/* wait_queue needed for kernel thread freeze support */
	wait_queue_head_t	ts_wait;
	NvBool bIsSuspended;
};

	NvOdmOsMutexHandle htouchMutex;

// 20100825  Debug Message Control (Temporary) [START]
#define touch_fingerprint(enable, fmt, args...) do { \
			if (enable) \
				printk(fmt, ##args); \
} while (0)

NvBool DebugMsgPrint = NV_FALSE;

ssize_t touch_fingerprint_store(struct device *dev, struct device_attribute *attr, const char *buffer, size_t count)
{
	NvS8 *input;
	NvU32 value;

	count = count > READ_BUFFER_LENGTH ? READ_BUFFER_LENGTH : count;
	input = kzalloc(((int)count+1), GFP_KERNEL);
	if (!input) {
		return 0;
	}

	memcpy(input, buffer, count);

	input[count] = '\0';
	value = simple_strtoul(&input[0], '\0', 10);

	DebugMsgPrint = !!value;

	kfree(input);
	return count;
}

DEVICE_ATTR(fingerprint, 0666, NULL, touch_fingerprint_store);
// 20100825  Debug Message Control (Temporary) [END]


// 20100906  Touch F/W version [START]
ssize_t touch_fw_version_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	sprintf(buf, "%d\n", showTouchFWversion());
	return (ssize_t)(strlen(buf)+1);
}

DEVICE_ATTR(fw_version, 0666, touch_fw_version_show, NULL);
// 20100906  Touch F/W version [END]


// 20100718  grip suppression [START]
#ifdef FEATURE_LGE_TOUCH_GRIP_SUPPRESSION
ssize_t touch_gripsuppression_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	sprintf(buf, "%d\n", getTouchGripSuppressionValue());	
	return (ssize_t)(strlen(buf)+1);
}

ssize_t touch_gripsuppression_store(struct device *dev, struct device_attribute *attr, const char *buffer, size_t count)
{
	NvS8 *input;
	NvU32 value;

	count = count > READ_BUFFER_LENGTH ? READ_BUFFER_LENGTH : count;
	input = kzalloc(((int)count+1), GFP_KERNEL);
	if (!input) {
		return 0;
	}

	memcpy(input, buffer, count);

	input[count] = '\0';
	value = simple_strtoul(&input[0], '\0', 10);

	setTouchGripSuppressionValue(value);

	kfree(input);
	return count;
}

DEVICE_ATTR(gripsuppression, 0666, touch_gripsuppression_show, touch_gripsuppression_store);
#endif /* FEATURE_LGE_TOUCH_GRIP_SUPPRESSION */
// 20100718  grip suppression [END]


// 20100820  touch LED control [START]
#ifdef CONFIG_STAR_TOUCH_LED
extern void touchLED_enable(NvBool status);
#endif
// 20100820  touch LED control [END]


#ifdef CONFIG_HAS_EARLYSUSPEND
static void tegra_touch_early_suspend(struct early_suspend *es)
{
	struct tegra_touch_driver_data *touch;
	touch = container_of(es, struct tegra_touch_driver_data, early_suspend);

	printk("[TOUCH] tegra_touch_early_suspend\n");
	
	if (touch && touch->hTouchDevice) {
        if (!touch->bIsSuspended) {
            NvOdmOsMutexLock(touch->hMutex);
		NvOdmTouchInterruptMask(touch->hTouchDevice, NV_TRUE);
			
		NvOdmTouchPowerControl(touch->hTouchDevice, NvOdmTouch_PowerMode_3);

            touch->bIsSuspended = NV_TRUE;
            if (!touch->bPollingMode) {
                /* allow touch thread to call wake_event_freezer */
                NvOdmOsSemaphoreSignal(touch->semaphore);
            }
            NvOdmOsMutexUnlock(touch->hMutex);
        } 
        else printk("[TOUCH]!: bIsSuspended = TURE!@tegra_touch_early_suspend\n");
	}
	else {
		pr_err("tegra_touch_early_suspend: NULL handles passed\n");
	}
}

static void tegra_touch_late_resume(struct early_suspend *es)
{
	struct tegra_touch_driver_data *touch;
	touch = container_of(es, struct tegra_touch_driver_data, early_suspend);

	printk("[TOUCH] tegra_touch_late_resume\n");

	if (touch && touch->hTouchDevice) {
		if(!NvOdmTouchPowerControl(touch->hTouchDevice, NvOdmTouch_PowerMode_0))
		{
			NvOdmTouchDeviceClose(touch->hTouchDevice);
			NvOdmTouchDeviceOpen(&touch->hTouchDevice, &touch->semaphore);

             // 20101130  for ESD [START]
			NvOdmTouchPowerControl(touch->hTouchDevice, NvOdmTouch_PowerMode_3);
			NvOdmTouchPowerControl(touch->hTouchDevice, NvOdmTouch_PowerMode_1);
             // 20101130  for ESD [END]
		}
		else
			NvOdmTouchInterruptMask(touch->hTouchDevice, NV_FALSE);

         if (touch->bIsSuspended) {
             touch->bIsSuspended = NV_FALSE;
             wake_up(&touch->ts_wait);
         }
         else printk("[TOUCH]!:bIsSuspended = FALSE!@tegra_touch_late_resume\n");

	}
	else {
		pr_err("tegra_touch_late_resume: NULL handles passed\n");
	}
}
#else
static int tegra_touch_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct tegra_touch_driver_data *touch = platform_get_drvdata(pdev);

	printk("[TOUCH] tegra_touch_suspend\n");

	if (touch && touch->hTouchDevice) {
        if (!touch->bIsSuspended) {
		NvOdmTouchInterruptMask(touch->hTouchDevice, NV_TRUE);
		NvOdmOsSleepMS(50);
			
		NvOdmTouchPowerControl(touch->hTouchDevice, NvOdmTouch_PowerMode_3);
            touch->bIsSuspended = NV_TRUE;
            if (!touch->bPollingMode) {
                /* allow touch thread to call wake_event_freezer */
                NvOdmOsSemaphoreSignal(touch->semaphore);
            }

		return 0;
	}
    }
	pr_err("tegra_touch_suspend: NULL handles passed\n");
	return -1;
}

static int tegra_touch_resume(struct platform_device *pdev)
{
	struct tegra_touch_driver_data *touch = platform_get_drvdata(pdev);

	printk("[TOUCH] tegra_touch_resume\n");

	if (touch && touch->hTouchDevice) {
        if (touch->bIsSuspended) {
		if(!NvOdmTouchPowerControl(touch->hTouchDevice, NvOdmTouch_PowerMode_0))
		{
                touch->bIsSuspended = NV_FALSE;
                wake_up(&touch->ts_wait);
			NvOdmTouchDeviceClose(touch->hTouchDevice);
			NvOdmTouchDeviceOpen(&touch->hTouchDevice, &touch->semaphore);
		}
        }
		else
			NvOdmTouchInterruptMask(touch->hTouchDevice, NV_FALSE);

		return 0;
	}
	pr_err("tegra_touch_resume: NULL handles passed\n");
	return -1;
}
#endif

// 20101022  touch smooth moving improve [START]
#ifdef FEATURE_LGE_TOUCH_MOVING_IMPROVE
#define ADJUST_FACTOR_LEVEL_5			8
#define ADJUST_FACTOR_LEVEL_4			6
#define ADJUST_FACTOR_LEVEL_3			4
#define ADJUST_FACTOR_LEVEL_2			2
#define ADJUST_FACTOR_LEVEL_1			1
#define ADJUST_FACTOR_BASE				4

#define ADJUST_BASIS_LEVEL_5			5
#define ADJUST_BASIS_LEVEL_4			11
#define ADJUST_BASIS_LEVEL_3			19
#define ADJUST_BASIS_LEVEL_2			29
#define ADJUST_BASIS_LEVEL_1			40

#define SQUARE(x)		((x) * (x))


static NvU32 adjust_X[NVODM_MAX_INPUT_COORDS]; 
static NvU32 adjust_Y[NVODM_MAX_INPUT_COORDS];


static void tegra_touch_adjust_position(NvU32 finger_num, NvU32 x_value, NvU32 y_value, NvBool wasToolDown)
{
	if(!wasToolDown)
	{
		adjust_X[finger_num] = x_value;
		adjust_Y[finger_num] = y_value;
	}
	else
	{
		NvU32 distant = int_sqrt(SQUARE(adjust_X[finger_num] - x_value) + SQUARE(adjust_Y[finger_num] - y_value));

		if(distant <= ADJUST_BASIS_LEVEL_5)
		{
			adjust_X[finger_num] = (adjust_X[finger_num] * ADJUST_FACTOR_LEVEL_5 + x_value * ADJUST_FACTOR_BASE + (ADJUST_FACTOR_LEVEL_5 + ADJUST_FACTOR_BASE)/2) / (ADJUST_FACTOR_LEVEL_5 + ADJUST_FACTOR_BASE);
			adjust_Y[finger_num] = (adjust_Y[finger_num] * ADJUST_FACTOR_LEVEL_5 + y_value * ADJUST_FACTOR_BASE + (ADJUST_FACTOR_LEVEL_5 + ADJUST_FACTOR_BASE)/2) / (ADJUST_FACTOR_LEVEL_5 + ADJUST_FACTOR_BASE);
		}		
		else if(distant <= ADJUST_BASIS_LEVEL_4)
		{
			adjust_X[finger_num] = (adjust_X[finger_num] * ADJUST_FACTOR_LEVEL_4 + x_value * ADJUST_FACTOR_BASE + (ADJUST_FACTOR_LEVEL_4 + ADJUST_FACTOR_BASE)/2) / (ADJUST_FACTOR_LEVEL_4 + ADJUST_FACTOR_BASE);
			adjust_Y[finger_num] = (adjust_Y[finger_num] * ADJUST_FACTOR_LEVEL_4 + y_value * ADJUST_FACTOR_BASE + (ADJUST_FACTOR_LEVEL_4 + ADJUST_FACTOR_BASE)/2) / (ADJUST_FACTOR_LEVEL_4 + ADJUST_FACTOR_BASE);
		}
		else if(distant <= ADJUST_BASIS_LEVEL_3)
		{
			adjust_X[finger_num] = (adjust_X[finger_num] * ADJUST_FACTOR_LEVEL_3 + x_value * ADJUST_FACTOR_BASE + (ADJUST_FACTOR_LEVEL_3 + ADJUST_FACTOR_BASE)/2) / (ADJUST_FACTOR_LEVEL_3 + ADJUST_FACTOR_BASE);
			adjust_Y[finger_num] = (adjust_Y[finger_num] * ADJUST_FACTOR_LEVEL_3 + y_value * ADJUST_FACTOR_BASE + (ADJUST_FACTOR_LEVEL_3 + ADJUST_FACTOR_BASE)/2) / (ADJUST_FACTOR_LEVEL_3 + ADJUST_FACTOR_BASE);
		}
		else if(distant <= ADJUST_BASIS_LEVEL_2)
		{
			adjust_X[finger_num] = (adjust_X[finger_num] * ADJUST_FACTOR_LEVEL_2 + x_value * ADJUST_FACTOR_BASE + (ADJUST_FACTOR_LEVEL_2 + ADJUST_FACTOR_BASE)/2) / (ADJUST_FACTOR_LEVEL_2 + ADJUST_FACTOR_BASE);
			adjust_Y[finger_num] = (adjust_Y[finger_num] * ADJUST_FACTOR_LEVEL_2 + y_value * ADJUST_FACTOR_BASE + (ADJUST_FACTOR_LEVEL_2 + ADJUST_FACTOR_BASE)/2) / (ADJUST_FACTOR_LEVEL_2 + ADJUST_FACTOR_BASE);
		}
		else if(distant <= ADJUST_BASIS_LEVEL_1)
		{
			adjust_X[finger_num] = (adjust_X[finger_num] * ADJUST_FACTOR_LEVEL_1 + x_value * ADJUST_FACTOR_BASE + (ADJUST_FACTOR_LEVEL_1 + ADJUST_FACTOR_BASE)/2) / (ADJUST_FACTOR_LEVEL_1 + ADJUST_FACTOR_BASE);
			adjust_Y[finger_num] = (adjust_Y[finger_num] * ADJUST_FACTOR_LEVEL_1 + y_value * ADJUST_FACTOR_BASE + (ADJUST_FACTOR_LEVEL_1 + ADJUST_FACTOR_BASE)/2) / (ADJUST_FACTOR_LEVEL_1 + ADJUST_FACTOR_BASE);
		}
		else
		{
			adjust_X[finger_num] = x_value;
			adjust_Y[finger_num] = y_value;
		}
	}
}
#endif /* FEATURE_LGE_TOUCH_MOVING_IMPROVE */
// 20101022  [STAR] apply touch smooth moving improve [END]


// 20100402  LGE Touch thread Customization [START]
#ifdef FEATURE_LGE_TOUCH_CUSTOMIZE
static int tegra_touch_thread(void *pdata)
{
    struct tegra_touch_driver_data *touch = (struct tegra_touch_driver_data*)pdata;
	
    NvOdmTouchCoordinateInfo c = {0};
	NvU32 i = 0;
	NvU32 x[LGE_SUPPORT_FINGERS_NUM] = {0}, y[LGE_SUPPORT_FINGERS_NUM] = {0};
    NvU32 pressure[LGE_SUPPORT_FINGERS_NUM] = {0}, width[LGE_SUPPORT_FINGERS_NUM] = {0};
    NvBool bKeepReadingSamples;

    NvBool ToolDown[LGE_SUPPORT_FINGERS_NUM] = {NV_FALSE, NV_FALSE};
	NvBool Prev_ToolDown[LGE_SUPPORT_FINGERS_NUM] = {NV_FALSE, NV_FALSE};

	NvU8 curr_event_type = TOUCH_EVENT_NULL;
	NvU8 prev_event_type = TOUCH_EVENT_NULL;

	NvU16 pressed_button_type = KEY_REJECT;

	NvU8 grip_suppression_value = 0;
	NvU8 touch_active_crop = 1;
	NvU8 valid_fingers = 0;
	
	NvU8 lcd_finger_num = 0;

#ifdef CONFIG_TOUCHSCREEN_ANDROID_VIRTUALKEYS
	unsigned long timeout_jiffies = 0;
#endif

	/* touch event thread should be frozen before suspend */
    /* 20110730 
       BUG 853092
       use set_freezeable() instead of set_freezable_with_signal()
	set_freezable_with_signal();
     */	
    set_freezable();
	
	while(!kthread_should_stop())
	{
		if (touch->bPollingMode)
			msleep(touch->pollingIntervalMS); 
		else
			/* FIXME should have a timeout so, that we can exit thread */
			if (!NvOdmOsSemaphoreWaitTimeout(touch->semaphore, NV_WAIT_INFINITE))
				BUG();
        if (touch->bIsSuspended) {
            /*
             * kernel threads need to wait on event freezable in order
             * to be freezable.
             * Refer kernel Documentation power->freezing-of-tasks
             */
            wait_event_freezable(touch->ts_wait,
                    !touch->bIsSuspended ||
                    kthread_should_stop());
	        printk("[TOUCH] ktrhead_wakeup@tegra_touch_thread\n");
            continue;
        }

        bKeepReadingSamples = NV_TRUE;
        while (bKeepReadingSamples)
        {
        	NvOdmOsMutexLock(touch->hMutex);
			if (!NvOdmTouchReadCoordinate(touch->hTouchDevice, &c))
			{
				pr_err("What the... Nvidia!! Why does it happen i2c error and why can't I recover it??\n");
				NvOdmTouchDeviceClose(touch->hTouchDevice);
				NvOdmTouchDeviceOpen(&touch->hTouchDevice, &touch->semaphore);
        		NvOdmOsMutexUnlock(touch->hMutex);
				break;
			}
	   		NvOdmOsMutexUnlock(touch->hMutex);

			if (c.fingerstate & NvOdmTouchSampleIgnore)
			{
				pr_err("Invalid sample\n");

				goto DoneWithSample;
			}

			for (i = 0; i < LGE_SUPPORT_FINGERS_NUM; i++)
			{
				Prev_ToolDown[i] = ToolDown[i];
#ifdef FEATURE_LGE_TOUCH_ACTIVE_CROP
#ifdef FEATURE_LGE_TOUCH_GRIP_SUPPRESSION
				touch_active_crop = getTouchGripSuppressionValue();	// default = 2
#endif /* FEATURE_LGE_TOUCH_GRIP_SUPPRESSION */
				if(c.additionalInfo.multi_XYCoords[i][0] < touch_active_crop && c.additionalInfo.multi_XYCoords[i][0] >= 0) 
				{
					touch_fingerprint(DebugMsgPrint, "[TOUCH] DEBUG: clipping by %d from X= %d\n",touch_active_crop,c.additionalInfo.multi_XYCoords[i][0]);
					c.additionalInfo.multi_XYCoords[i][0] = touch_active_crop;
				}
				if(c.additionalInfo.multi_XYCoords[i][0] < LGE_TOUCH_RESOLUTION_X && c.additionalInfo.multi_XYCoords[i][0] >= LGE_TOUCH_RESOLUTION_X - touch_active_crop) 
				{
					touch_fingerprint(DebugMsgPrint, "[TOUCH] DEBUG: clipping by %d from X= %d\n",touch_active_crop,c.additionalInfo.multi_XYCoords[i][0]);
					c.additionalInfo.multi_XYCoords[i][0] = LGE_TOUCH_RESOLUTION_X 				}
				if(c.additionalInfo.multi_XYCoords[i][1] < TOUCH_LCD_ACTIVE_AREA_Y && c.additionalInfo.multi_XYCoords[i][1] >= LGE_TOUCH_RESOLUTION_Y - touch_active_crop) 
				{
					touch_fingerprint(DebugMsgPrint, "[TOUCH] DEBUG: clipping by %d from Y= %d\n",touch_active_crop,c.additionalInfo.multi_XYCoords[i][1]);
					c.additionalInfo.multi_XYCoords[i][1] = LGE_TOUCH_RESOLUTION_Y - 1 - touch_active_crop;
				}
				else if(c.additionalInfo.multi_XYCoords[i][1] < TOUCH_BUTTON_AREA_Y && c.additionalInfo.multi_XYCoords[i][1] >= TOUCH_LCD_ACTIVE_AREA_Y ) 
				{
					touch_fingerprint(DebugMsgPrint, "[TOUCH] DEBUG: dead zone Y= %d\n",c.additionalInfo.multi_XYCoords[i][1]);
					c.additionalInfo.multi_fingerstate[i] = 0;
				}
#endif /* FEATURE_LGE_TOUCH_ACTIVE_CROP */

// 20100718  grip suppression [START]
#ifdef FEATURE_LGE_TOUCH_GRIP_SUPPRESSION
				grip_suppression_value = getTouchGripSuppressionValue();

				if(c.additionalInfo.multi_XYCoords[i][0] >= grip_suppression_value && c.additionalInfo.multi_XYCoords[i][0] < LGE_TOUCH_RESOLUTION_X - grip_suppression_value)
				{
#endif /* FEATURE_LGE_TOUCH_GRIP_SUPPRESSION */
// 20100718  grip suppression [END]	
				
				if(c.additionalInfo.multi_fingerstate[i])
				{
// 20101022  touch smooth moving improve [START]
#ifdef FEATURE_LGE_TOUCH_MOVING_IMPROVE

					tegra_touch_adjust_position(i, c.additionalInfo.multi_XYCoords[i][0], c.additionalInfo.multi_XYCoords[i][1], Prev_ToolDown[i]);
					x[i] = adjust_X[i];
#ifdef FEATURE_LGE_TOUCH_EXPAND_HIDDEN_ACTIVE_AREA
					if(adjust_Y[i] < TOUCH_LCD_ACTIVE_AREA_Y && adjust_Y[i] >= LGE_TOUCH_RESOLUTION_Y)
						y[i] = LGE_TOUCH_RESOLUTION_Y - 1;
					else
#endif /* FEATURE_LGE_TOUCH_EXPAND_HIDDEN_ACTIVE_AREA */
						y[i] = adjust_Y[i];

#else /* FEATURE_LGE_TOUCH_MOVING_IMPROVE */

					x[i] = c.additionalInfo.multi_XYCoords[i][0];
// 20100720  LCD Active area expansion [START]
#ifdef FEATURE_LGE_TOUCH_EXPAND_HIDDEN_ACTIVE_AREA
					if(c.additionalInfo.multi_XYCoords[i][1] < TOUCH_LCD_ACTIVE_AREA_Y && c.additionalInfo.multi_XYCoords[i][1] >= LGE_TOUCH_RESOLUTION_Y)
						y[i] = LGE_TOUCH_RESOLUTION_Y - 1;
					else
#endif /* FEATURE_LGE_TOUCH_EXPAND_HIDDEN_ACTIVE_AREA */
// 20100720  LCD Active area expansion [END]
					y[i] = c.additionalInfo.multi_XYCoords[i][1];

#endif /* FEATURE_LGE_TOUCH_MOVING_IMPROVE */
// 20101022  touch smooth moving improve [END]

					pressure[i] = c.additionalInfo.Pressure[i];
					width[i] = c.additionalInfo.width[i];
					ToolDown[i] = NV_TRUE;
					valid_fingers++;
				}
				else
					ToolDown[i] = NV_FALSE;

// 20100718  grip suppression [START]
#ifdef FEATURE_LGE_TOUCH_GRIP_SUPPRESSION
				}
				else
					ToolDown[i] = NV_FALSE;
#endif /* FEATURE_LGE_TOUCH_GRIP_SUPPRESSION */
// 20100718  grip suppression [END]
			}

			if (c.fingerstate & NvOdmTouchSampleValidFlag)
			{
				for(i = 0; i < LGE_SUPPORT_FINGERS_NUM; i++)
				{
					if(ToolDown[i] == NV_TRUE)
					{
						// First finger touch
						//if(c.additionalInfo.Fingers == 1 && i == 0)
						if(valid_fingers == 1 && i == 0)
						{
#ifdef CONFIG_TOUCHSCREEN_ANDROID_VIRTUALKEYS
							curr_event_type = TOUCH_EVENT_ABS;
#else
							if((y[i] < LGE_TOUCH_RESOLUTION_Y && prev_event_type == TOUCH_EVENT_NULL) || prev_event_type == TOUCH_EVENT_ABS)
								curr_event_type = TOUCH_EVENT_ABS;
							else if((y[i] >= LGE_TOUCH_RESOLUTION_Y && prev_event_type == TOUCH_EVENT_NULL) || prev_event_type == TOUCH_EVENT_BUTTON)
								curr_event_type = TOUCH_EVENT_BUTTON;
#endif

							if(curr_event_type == TOUCH_EVENT_ABS)
							{
#ifndef CONFIG_TOUCHSCREEN_ANDROID_VIRTUALKEYS
								if(y[i] < LGE_TOUCH_RESOLUTION_Y)
#endif
								{
#ifdef CONFIG_TOUCHSCREEN_ANDROID_VIRTUALKEYS
									if(y[i] >= LGE_TOUCH_RESOLUTION_Y) {
										if (Prev_ToolDown[i] == NV_FALSE) {
											timeout_jiffies = jiffies + msecs_to_jiffies(300);
										} else if (time_is_after_eq_jiffies(timeout_jiffies)) {
											input_report_abs(touch->input_dev, ABS_MT_TOUCH_MAJOR, 0);
											input_mt_sync(touch->input_dev);
										}
									}
#endif
									input_report_abs(touch->input_dev, ABS_MT_TOUCH_MAJOR, pressure[i]);
									input_report_abs(touch->input_dev, ABS_MT_WIDTH_MAJOR, width[i]);
									input_report_abs(touch->input_dev, ABS_MT_POSITION_X, x[i]);
									input_report_abs(touch->input_dev, ABS_MT_POSITION_Y, y[i]);

									input_mt_sync(touch->input_dev);
									touch_fingerprint(DebugMsgPrint, "[TOUCH] Finger1 Press x = %d, y = %d, width = %d\n", x[i], y[i], width[i]);

									lcd_finger_num++;
								}
							}
							else if(curr_event_type == TOUCH_EVENT_BUTTON)
							{
								if(y[0] >= TOUCH_BUTTON_AREA_Y)
								{
									//if(x[0] > 30 && x[0] < 120) //center 75
									if(x[0] > 15 && x[0] < 115) //center 75
									{
										if(Prev_ToolDown[i] == NV_FALSE)
										{
											input_report_key(touch->input_dev, KEY_MENU, 1);
											touch_fingerprint(DebugMsgPrint, "[TOUCH] Key Event KEY = %d, PRESS = %d\n", KEY_MENU, 1);
											pressed_button_type = KEY_MENU;
											input_sync(touch->input_dev);
										}
										else
										{
											if(pressed_button_type != KEY_MENU && pressed_button_type != KEY_REJECT)
											{
												input_report_key(touch->input_dev, KEY_REJECT, 1);
												touch_fingerprint(DebugMsgPrint, "[TOUCH] Key Event KEY = %d, PRESS = %d\n", KEY_REJECT, 1);
												input_report_key(touch->input_dev, KEY_REJECT, 0);
												touch_fingerprint(DebugMsgPrint, "[TOUCH] Key Event KEY = %d, PRESS = %d\n", KEY_REJECT, 0);
												input_report_key(touch->input_dev, pressed_button_type, 0);
												touch_fingerprint(DebugMsgPrint, "[TOUCH] Key Event KEY = %d, PRESS = %d\n", pressed_button_type, 0);
												pressed_button_type = KEY_REJECT;
												input_sync(touch->input_dev);
											}
										}
									}
									//else if(x[0] > 140 && x[0] < 230) //center 185
									else if(x[0] > 145 && x[0] < 225) //center 185
									{
										if(Prev_ToolDown[i] == NV_FALSE)
										{
											input_report_key(touch->input_dev, KEY_HOME, 1);
											touch_fingerprint(DebugMsgPrint, "[TOUCH] Key Event KEY = %d, PRESS = %d\n", KEY_HOME, 1);
											pressed_button_type = KEY_HOME;
											input_sync(touch->input_dev);
										}
										else
										{
											if(pressed_button_type != KEY_HOME && pressed_button_type != KEY_REJECT)
											{
												input_report_key(touch->input_dev, KEY_REJECT, 1);
												touch_fingerprint(DebugMsgPrint, "[TOUCH] Key Event KEY = %d, PRESS = %d\n", KEY_REJECT, 1);
												input_report_key(touch->input_dev, KEY_REJECT, 0);
												touch_fingerprint(DebugMsgPrint, "[TOUCH] Key Event KEY = %d, PRESS = %d\n", KEY_REJECT, 0);
												input_report_key(touch->input_dev, pressed_button_type, 0);
												touch_fingerprint(DebugMsgPrint, "[TOUCH] Key Event KEY = %d, PRESS = %d\n", pressed_button_type, 0);
												pressed_button_type = KEY_REJECT;
												input_sync(touch->input_dev);
											}
										}
									}
									//else if(x[0] > 250 && x[0] < 340) //center 295
									else if(x[0] > 255 && x[0] < 335) //center 295
									{
										if(Prev_ToolDown[i] == NV_FALSE)
										{
											input_report_key(touch->input_dev, KEY_BACK, 1);
											touch_fingerprint(DebugMsgPrint, "[TOUCH] Key Event KEY = %d, PRESS = %d\n", KEY_BACK, 1);
											pressed_button_type = KEY_BACK;
											input_sync(touch->input_dev);
										}
										else
										{
											if(pressed_button_type != KEY_BACK && pressed_button_type != KEY_REJECT)
											{
												input_report_key(touch->input_dev, KEY_REJECT, 1);
												touch_fingerprint(DebugMsgPrint, "[TOUCH] Key Event KEY = %d, PRESS = %d\n", KEY_REJECT, 1);
												input_report_key(touch->input_dev, KEY_REJECT, 0);
												touch_fingerprint(DebugMsgPrint, "[TOUCH] Key Event KEY = %d, PRESS = %d\n", KEY_REJECT, 0);
												input_report_key(touch->input_dev, pressed_button_type, 0);
												touch_fingerprint(DebugMsgPrint, "[TOUCH] Key Event KEY = %d, PRESS = %d\n", pressed_button_type, 0);
												pressed_button_type = KEY_REJECT;
												input_sync(touch->input_dev);
											}
										}
									}
									//else if(x[0] > 360 && x[0] < 450) //center 405
									else if(x[0] > 365 && x[0] < 465) //center 405
									{
										if(Prev_ToolDown[i] == NV_FALSE)
										{
											input_report_key(touch->input_dev, KEY_SEARCH, 1);
											touch_fingerprint(DebugMsgPrint, "[TOUCH] Key Event KEY = %d, PRESS = %d\n", KEY_SEARCH, 1);
											pressed_button_type = KEY_SEARCH;
											input_sync(touch->input_dev);
										}
										else
										{
											if(pressed_button_type != KEY_SEARCH && pressed_button_type != KEY_REJECT)
											{
												input_report_key(touch->input_dev, KEY_REJECT, 1);
												touch_fingerprint(DebugMsgPrint, "[TOUCH] Key Event KEY = %d, PRESS = %d\n", KEY_REJECT, 1);
												input_report_key(touch->input_dev, KEY_REJECT, 0);
												touch_fingerprint(DebugMsgPrint, "[TOUCH] Key Event KEY = %d, PRESS = %d\n", KEY_REJECT, 0);
												input_report_key(touch->input_dev, pressed_button_type, 0);
												touch_fingerprint(DebugMsgPrint, "[TOUCH] Key Event KEY = %d, PRESS = %d\n", pressed_button_type, 0);
												pressed_button_type = KEY_REJECT;
												input_sync(touch->input_dev);
											}
										}
									}
									else
									{
// 20101209  change Touch Key recognition algorithm [START]
										// Recognize as do nothing 
										if(Prev_ToolDown[i] == NV_FALSE)
											ToolDown[i] = NV_FALSE;
										/*
										if(Prev_ToolDown[i] == NV_FALSE)
										{
											pressed_button_type = KEY_REJECT;
										}
										else
										{
											if(pressed_button_type != KEY_REJECT)
											{
												input_report_key(touch->input_dev, KEY_REJECT, 1);
												touch_fingerprint(DebugMsgPrint, "[TOUCH] Key Event KEY = %d, PRESS = %d\n", KEY_REJECT, 1);
												input_report_key(touch->input_dev, KEY_REJECT, 0);
												touch_fingerprint(DebugMsgPrint, "[TOUCH] Key Event KEY = %d, PRESS = %d\n", KEY_REJECT, 0);
												input_report_key(touch->input_dev, pressed_button_type, 0);
												touch_fingerprint(DebugMsgPrint, "[TOUCH] Key Event KEY = %d, PRESS = %d\n", pressed_button_type, 0);
												pressed_button_type = KEY_REJECT;
												input_sync(touch->input_dev);
											}
										}
										*/
// 20101209  change Touch Key recognition algorithm [END]
									}

								}
								else
								{
// 20101209  change Touch Key recognition algorithm [START]
									if(y[i] < LGE_TOUCH_RESOLUTION_Y)
									{
										input_report_abs(touch->input_dev, ABS_MT_POSITION_X, x[i]);
										input_report_abs(touch->input_dev, ABS_MT_POSITION_Y, y[i]);
										input_report_abs(touch->input_dev, ABS_MT_TOUCH_MAJOR, pressure[i]);
										input_report_abs(touch->input_dev, ABS_MT_WIDTH_MAJOR, width[i]);
										
										input_mt_sync(touch->input_dev);
										touch_fingerprint(DebugMsgPrint, "[TOUCH] Finger1 Press x = %d, y = %d, width = %d\n", x[i], y[i], width[i]);

										curr_event_type = TOUCH_EVENT_ABS;

										lcd_finger_num++;

										if(pressed_button_type != KEY_REJECT)
										{
											input_report_key(touch->input_dev, KEY_REJECT, 1);
											touch_fingerprint(DebugMsgPrint, "[TOUCH] Key Event KEY = %d, PRESS = %d\n", KEY_REJECT, 1);
											input_report_key(touch->input_dev, KEY_REJECT, 0);
											touch_fingerprint(DebugMsgPrint, "[TOUCH] Key Event KEY = %d, PRESS = %d\n", KEY_REJECT, 0);
											input_report_key(touch->input_dev, pressed_button_type, 0);
											touch_fingerprint(DebugMsgPrint, "[TOUCH] Key Event KEY = %d, PRESS = %d\n", pressed_button_type, 0);
											pressed_button_type = KEY_REJECT;
										}
									}
									else
									{
										// Recognize as do nothing 
										if(Prev_ToolDown[i] == NV_FALSE)
											ToolDown[i] = NV_FALSE;
									}
/*									
// 20101021  lcd-button area touch scenario change [START]
									if(y[i] < LGE_TOUCH_RESOLUTION_Y)
									{
										input_report_abs(touch->input_dev, ABS_MT_POSITION_X, x[i]);
										input_report_abs(touch->input_dev, ABS_MT_POSITION_Y, y[i]);
										input_report_abs(touch->input_dev, ABS_MT_TOUCH_MAJOR, pressure[i]);
										input_report_abs(touch->input_dev, ABS_MT_WIDTH_MAJOR, width[i]);
										
										input_mt_sync(touch->input_dev);
										touch_fingerprint(DebugMsgPrint, "[TOUCH] Finger1 Press x = %d, y = %d, width = %d\n", x[i], y[i], width[i]);

										curr_event_type = TOUCH_EVENT_ABS;

										lcd_finger_num++;
									}
// 20101021  lcd-button area touch scenario change [END]
									
									if(Prev_ToolDown[i] == NV_FALSE)
									{
										pressed_button_type = KEY_REJECT;
									}
									else
									{
										if(pressed_button_type != KEY_REJECT)
										{
											input_report_key(touch->input_dev, KEY_REJECT, 1);
											touch_fingerprint(DebugMsgPrint, "[TOUCH] Key Event KEY = %d, PRESS = %d\n", KEY_REJECT, 1);
											input_report_key(touch->input_dev, KEY_REJECT, 0);
											touch_fingerprint(DebugMsgPrint, "[TOUCH] Key Event KEY = %d, PRESS = %d\n", KEY_REJECT, 0);
											input_report_key(touch->input_dev, pressed_button_type, 0);
											touch_fingerprint(DebugMsgPrint, "[TOUCH] Key Event KEY = %d, PRESS = %d\n", pressed_button_type, 0);
											pressed_button_type = KEY_REJECT;

// 20101021  lcd-button area touch scenario change [START]
											if(curr_event_type != TOUCH_EVENT_ABS)
// 20101021  lcd-button area touch scenario change [END]
											input_sync(touch->input_dev);
										}
									}
*/
// 20101209  change Touch Key recognition algorithm [END]
								}
							}
							else
							{
								curr_event_type = TOUCH_EVENT_NULL;
								pressed_button_type = KEY_REJECT;
							}
						}
						else // multi-finger
						{
							curr_event_type = TOUCH_EVENT_ABS;

							if(pressed_button_type != KEY_REJECT)
							{
								input_report_key(touch->input_dev, KEY_REJECT, 1);
								touch_fingerprint(DebugMsgPrint, "[TOUCH] Key Event KEY = %d, PRESS = %d\n", KEY_REJECT, 1);
								input_report_key(touch->input_dev, KEY_REJECT, 0);
								touch_fingerprint(DebugMsgPrint, "[TOUCH] Key Event KEY = %d, PRESS = %d\n", KEY_REJECT, 0);
								input_report_key(touch->input_dev, pressed_button_type, 0);
								touch_fingerprint(DebugMsgPrint, "[TOUCH] Key Event KEY = %d, PRESS = %d\n", pressed_button_type, 0);
								pressed_button_type = KEY_REJECT;
							}

							if(y[i] < LGE_TOUCH_RESOLUTION_Y)
							{
								input_report_abs(touch->input_dev, ABS_MT_POSITION_X, x[i]);
								input_report_abs(touch->input_dev, ABS_MT_POSITION_Y, y[i]);
								input_report_abs(touch->input_dev, ABS_MT_TOUCH_MAJOR, pressure[i]);
								input_report_abs(touch->input_dev, ABS_MT_WIDTH_MAJOR, width[i]);

								input_mt_sync(touch->input_dev);
								touch_fingerprint(DebugMsgPrint, "[TOUCH] Finger%d Press x = %d, y = %d, press = %d\n", i+1, x[i], y[i], width[i]);

								lcd_finger_num++;
							}
						}

					}
					else
					{
						if(pressed_button_type != KEY_REJECT && i == 0)
						{
							input_report_key(touch->input_dev, pressed_button_type, 0);
							touch_fingerprint(DebugMsgPrint, "[TOUCH] Key Event KEY = %d, PRESS = %d\n", pressed_button_type, 0);
							pressed_button_type = KEY_REJECT;
							input_sync(touch->input_dev);
						}
						else
						{

							if(Prev_ToolDown[i] == NV_TRUE)
							{
								touch_fingerprint(DebugMsgPrint, "[TOUCH] Finger%d Release\n", i+1);
							}
						}
					}
				}

				if(curr_event_type == TOUCH_EVENT_ABS)
				{
					if(lcd_finger_num == 0)
						input_mt_sync(touch->input_dev);
					input_sync(touch->input_dev);
				}
				
				//if(c.additionalInfo.Fingers == 0)
				if(valid_fingers == 0)
				{
					prev_event_type = curr_event_type = TOUCH_EVENT_NULL;
				}
				else
				{
					prev_event_type = curr_event_type;
				}

// 20100820  touch LED control [START]
#ifdef CONFIG_STAR_TOUCH_LED
				if(curr_event_type == TOUCH_EVENT_ABS || (curr_event_type == TOUCH_EVENT_BUTTON && pressed_button_type != KEY_REJECT))
	        		touchLED_enable(NV_TRUE);
#endif
// 20100820  touch LED control [END]
			}
			
			valid_fingers = 0;

			lcd_finger_num = 0;

	DoneWithSample:
			bKeepReadingSamples = NV_FALSE;
			if (!touch->bPollingMode && 
				!NvOdmTouchHandleInterrupt(touch->hTouchDevice))
			{
				/* Some more data to read keep going */
				bKeepReadingSamples = NV_TRUE;
			}
		}
	}

    return 0;
}
#else /* FEATURE_LGE_TOUCH_CUSTOMIZE */
static int tegra_touch_thread(void *pdata)
{
	struct tegra_touch_driver_data *touch =
		(struct tegra_touch_driver_data*)pdata;
	NvOdmTouchCoordinateInfo c = {0};
	NvU32 x[2] = {0}, y[2] = {0}, i = 0, prev_x[2] = {0}, prev_y[2] = {0};
	NvBool bKeepReadingSamples = NV_FALSE;
	NvU32 fingers = 0;
	NvBool ToolDown[2] = {NV_FALSE, NV_FALSE};
	NvOdmTouchCapabilities *caps = &touch->caps;
	NvU32 max_fingers = caps->MaxNumberOfFingerCoordReported;

	/* touch event thread should be frozen before suspend */
    /* 20110730 
       BUG 853092
       use set_freezeable() instead of set_freezable_with_signal()
	set_freezable_with_signal();
     */
	set_freezable();

	for (;;) {
		if (touch->bPollingMode)
			msleep(touch->pollingIntervalMS); 
		else
			NvOdmOsSemaphoreWait(touch->semaphore);

		bKeepReadingSamples = NV_TRUE;
		while (bKeepReadingSamples) {
			if (!NvOdmTouchReadCoordinate(touch->hTouchDevice, &c)){
				pr_err("Couldn't read touch sample\n");
				bKeepReadingSamples = NV_FALSE;
				continue;
			}

			fingers = c.additionalInfo.Fingers;

			/*
			 * sometimes the HW reports num of fingers greater than
			 * the max supported. This happens when previously there
			 * were 2 fingers touching and one of them was lifted.
			 * We give away the previously stored state for the
			 * first finger, and the lifted finger is
			 * sent with flags indicating PEN UP.
			 */
			if (((fingers == 1) || (fingers>max_fingers)) &&
			     (ToolDown[1] == NV_TRUE)) {
				ToolDown[0] = NV_TRUE;
				input_report_abs(touch->input_dev,
					ABS_X, prev_x[0]);
				input_report_abs(touch->input_dev,
					ABS_Y, prev_y[0]);
				input_report_key(touch->input_dev,
						BTN_TOUCH, ToolDown[0]);
				ToolDown[1] = NV_FALSE;
				input_report_abs(touch->input_dev,
					ABS_HAT0X, prev_x[1]); // x
				input_report_abs(touch->input_dev,
					ABS_HAT0Y, prev_y[1]); // y
				input_report_key(touch->input_dev,
					BTN_2, ToolDown[1]);
				input_sync(touch->input_dev);
			}

			if (c.fingerstate & NvOdmTouchSampleIgnore)
				goto DoneWithSample;

			switch (fingers) {
			case 0:
				for (i=0; i<max_fingers; i++) {
					ToolDown[i] = NV_FALSE;
				}
				break;
			case 1:
				ToolDown[0] = NV_TRUE;
				ToolDown[1] = NV_FALSE;
				break;
			case 2:
				for (i=0; i<max_fingers; i++) {
					ToolDown[i] = NV_TRUE;
				}
				break;
			default:
				/* can occur because of sensor errors */
				c.fingerstate = NvOdmTouchSampleIgnore;;
				goto DoneWithSample;
			}

			/* from 1 finger to no fingers */
			if ((fingers == 0) && (ToolDown[0] == NV_TRUE)) {
				x[0] = prev_x[0];
				y[0] = prev_y[0];
				ToolDown[0] = NV_FALSE;
			}
			else if (fingers == 1) {
				x[0] = c.xcoord;
				y[0] = c.ycoord;
			}
			else {
				for (i = 0; i < fingers; i++) {
					x[i] = c.additionalInfo.multi_XYCoords[i][0];
					y[i] = c.additionalInfo.multi_XYCoords[i][1];
				}
			}

			/* transformation from touch to screen orientation */
			if (caps->Orientation & NvOdmTouchOrientation_V_FLIP) {
				y[0] = caps->YMaxPosition +
					caps->YMinPosition - y[0];
				y[1] = caps->YMaxPosition +
					caps->YMinPosition - y[1];
			}
			if (caps->Orientation & NvOdmTouchOrientation_H_FLIP) {
				x[0] = caps->XMaxPosition +
					caps->XMinPosition - x[0];
				x[1] = caps->XMaxPosition +
					caps->XMinPosition - x[1];
			}

			if (caps->Orientation & NvOdmTouchOrientation_XY_SWAP) {
				for (i = 0; i < max_fingers; i++)
					swapv(x[i],y[i]);
			}

			if (c.fingerstate & NvOdmTouchSampleValidFlag) {
				input_report_abs(touch->input_dev, ABS_X, x[0]);
				input_report_abs(touch->input_dev, ABS_Y, y[0]);
				prev_x[0] = x[0];
				prev_y[0] = y[0];
			}

			if (caps->IsPressureSupported) {
				input_report_abs(touch->input_dev,
					ABS_PRESSURE, 
					c.additionalInfo.Pressure[0]);
			}
			if (caps->IsWidthSupported) {
				input_report_abs(touch->input_dev,
					ABS_TOOL_WIDTH, 
					c.additionalInfo.width[0]);
			}

			/* Report down or up flag */
			input_report_key(touch->input_dev,
					BTN_TOUCH, ToolDown[0]);

			/* report co-ordinates for the 2nd finger */
			if (fingers == 2) {
				input_report_abs(touch->input_dev,
					ABS_HAT0X, x[1]); // x
				input_report_abs(touch->input_dev,
					ABS_HAT0Y, y[1]); // y
				input_report_key(touch->input_dev,
					BTN_2, ToolDown[1]);
				prev_x[1] = x[1];
				prev_y[1] = y[1];
			} else if (((fingers == 1) || (fingers == 0)) &&
				   (ToolDown[1] == NV_TRUE)) {
				ToolDown[1] = NV_FALSE;
				input_report_abs(touch->input_dev,
					ABS_HAT0X, prev_x[1]); // x
				input_report_abs(touch->input_dev,
					ABS_HAT0Y, prev_y[1]); // y
				input_report_key(touch->input_dev,
					BTN_2, ToolDown[1]);
			}
			input_sync(touch->input_dev);

DoneWithSample:
			bKeepReadingSamples = NV_FALSE;
			if (!touch->bPollingMode &&
				!NvOdmTouchHandleInterrupt(touch->hTouchDevice)) {
				/* Some more data to read keep going */
				bKeepReadingSamples = NV_TRUE;
			}
		}
	}

	return 0;
}
#endif /* FEATURE_LGE_TOUCH_CUSTOMIZE */
// 20100402  LGE Touch thread Customization [END]

static int __init tegra_touch_probe(struct platform_device *pdev)
{
	struct tegra_touch_driver_data *touch = NULL;
	struct input_dev *input_dev = NULL;
	int err;
	NvOdmTouchCapabilities *caps;

	touch = kzalloc(sizeof(struct tegra_touch_driver_data), GFP_KERNEL);
	input_dev = input_allocate_device();
	if (input_dev == NULL || touch == NULL) {
		input_free_device(input_dev);
		kfree(touch);
		err = -ENOMEM;
		pr_err("tegra_touch_probe: Failed to allocate input device\n");
		return err;
	}
	
    touch->hMutex = NvOdmOsMutexCreate();
		if (!touch->hMutex) {
			err = -1;
			pr_err("tegra_touch_probe: Mutex creation failed\n");
			goto err_mutex_create_failed;
		}
		
		htouchMutex = touch->hMutex;
	touch->semaphore = NvOdmOsSemaphoreCreate(0);
	if (!touch->semaphore) {
		err = -1;
		pr_err("tegra_touch_probe: Semaphore creation failed\n");
		goto err_semaphore_create_failed;
	}

// 20100423  for Touch Interrupt Issue at booting [START]
#ifdef FEATURE_LGE_TOUCH_CUSTOMIZE
	touch->bPollingMode = NV_FALSE;
	if (!NvOdmTouchDeviceOpen(&touch->hTouchDevice, &touch->semaphore)) {
		err = -1;
		pr_err("tegra_touch_probe: NvOdmTouchDeviceOpen failed\n");
		goto err_open_failed;
	}
#else
	if (!NvOdmTouchDeviceOpen(&touch->hTouchDevice)) {
		err = -1;
		pr_err("tegra_touch_probe: NvOdmTouchDeviceOpen failed\n");
		goto err_open_failed;
	}
	touch->bPollingMode = NV_FALSE;
	if (!NvOdmTouchEnableInterrupt(touch->hTouchDevice, touch->semaphore)) {
		err = -1;
		pr_err("tegra_touch_probe: Interrupt failed, polling mode\n");
		touch->bPollingMode = NV_TRUE;
		touch->pollingIntervalMS = 10;
	}
#endif /* FEATURE_LGE_TOUCH_CUSTOMIZE */
// 20100423  for Touch Interrupt Issue at booting [END]

	touch->task =
		kthread_create(tegra_touch_thread, touch, "tegra_touch_thread");

	if(touch->task == NULL) {
		err = -1;
		goto err_kthread_create_failed;
	}
	init_waitqueue_head(&touch->ts_wait);
	wake_up_process( touch->task );

	touch->input_dev = input_dev;
	touch->input_dev->name = NVODM_TOUCH_NAME;

	/* Will generate sync at the end of all input */
	set_bit(EV_SYN, touch->input_dev->evbit);

	/* Event is key input type */
	set_bit(EV_KEY, touch->input_dev->evbit);
	/* virtual key is BTN_TOUCH */
	set_bit(BTN_TOUCH, touch->input_dev->keybit);
	/* Input values are in absoulte values */
	set_bit(EV_ABS, touch->input_dev->evbit);
	touch->bIsSuspended = NV_FALSE;

	NvOdmTouchDeviceGetCapabilities(touch->hTouchDevice, &touch->caps);

	caps = &touch->caps;

	if (caps->Orientation & NvOdmTouchOrientation_XY_SWAP) {
		touch->MaxY = caps->XMaxPosition;
		touch->MinY = caps->XMinPosition;
		touch->MaxX = caps->YMaxPosition;
		touch->MinX = caps->YMinPosition;

	} else {
		touch->MaxX = caps->XMaxPosition;
		touch->MinX = caps->XMinPosition;
		touch->MaxY = caps->YMaxPosition;
		touch->MinY = caps->YMinPosition;
	}

// 20100407  LGE Touch Customization [START]
#ifdef FEATURE_LGE_TOUCH_CUSTOMIZE
	// button
	set_bit(KEY_MENU, touch->input_dev->keybit);
	set_bit(KEY_HOME, touch->input_dev->keybit);
	set_bit(KEY_BACK, touch->input_dev->keybit);
	set_bit(KEY_SEARCH, touch->input_dev->keybit);
	set_bit(KEY_REJECT, touch->input_dev->keybit);

	input_set_abs_params(touch->input_dev, ABS_X, touch->MinX, touch->MaxX, 0, 0);
	input_set_abs_params(touch->input_dev, ABS_Y, touch->MinY, touch->MaxY, 0, 0);
	
	if (caps->IsPressureSupported)
		input_set_abs_params(touch->input_dev, ABS_PRESSURE, 0, caps->MaxNumberOfPressureReported, 0, 0);
	if (caps->IsWidthSupported)
		input_set_abs_params(touch->input_dev, ABS_TOOL_WIDTH, 0, caps->MaxNumberOfWidthReported, 0, 0);

	if (caps->IsMultiTouchSupported)
 	{
		input_set_abs_params(touch->input_dev, ABS_MT_POSITION_X, touch->MinX, touch->MaxX, 0, 0);
		input_set_abs_params(touch->input_dev, ABS_MT_POSITION_Y, touch->MinY, touch->MaxY, 0, 0);
		input_set_abs_params(touch->input_dev, ABS_MT_TOUCH_MAJOR, 0, caps->MaxNumberOfPressureReported, 0, 0);
		input_set_abs_params(touch->input_dev, ABS_MT_WIDTH_MAJOR, 0, caps->MaxNumberOfWidthReported, 0, 0);
	}
#else
	input_set_abs_params(touch->input_dev, ABS_X, touch->MinX,
		touch->MaxX, 0, 0);
	input_set_abs_params(touch->input_dev, ABS_Y, touch->MinY,
		touch->MaxY, 0, 0);
	input_set_abs_params(touch->input_dev, ABS_HAT0X, touch->MinX,
		touch->MaxX, 0, 0);
	input_set_abs_params(touch->input_dev, ABS_HAT0Y, touch->MinY,
		touch->MaxY, 0, 0);

	if (caps->IsPressureSupported)
		input_set_abs_params(touch->input_dev, ABS_PRESSURE, 0, 
			caps->MaxNumberOfPressureReported, 0, 0);
	if (caps->IsWidthSupported)
		input_set_abs_params(touch->input_dev, ABS_TOOL_WIDTH, 0, 
			caps->MaxNumberOfWidthReported, 0, 0);
#endif /* FEATURE_LGE_TOUCH_CUSTOMIZE */
// 20100407  LGE Touch Customization [END]

	platform_set_drvdata(pdev, touch);

	err = input_register_device(input_dev);
	if (err)
	{
		pr_err("tegra_touch_probe: Unable to register input device\n");
		goto err_input_register_device_failed;
	}

// 20100718  grip suppression [START]
#ifdef FEATURE_LGE_TOUCH_GRIP_SUPPRESSION
	err = device_create_file(&pdev->dev, &dev_attr_gripsuppression);
	if (err) {
		pr_err("tegra_touch_probe: grip suppression device_create_file failed\n");
		goto devfs_failed;
	}
#endif /* FEATURE_LGE_TOUCH_GRIP_SUPPRESSION */
// 20100718  grip suppression [END]

// 20100825  Debug Message Control (Temporary) [START]
	err = device_create_file(&pdev->dev, &dev_attr_fingerprint);
	if (err) {
		pr_err("tegra_touch_probe: fingerprint device_create_file failed\n");
		goto devfs_failed;
	}
// 20100825  Debug Message Control (Temporary) [END]

// 20100906  Touch F/W version [START]
	err = device_create_file(&pdev->dev, &dev_attr_fw_version);
	if (err) {
		pr_err("tegra_touch_probe: fw_version device_create_file failed\n");
		goto devfs_failed;
	}
// 20100906  Touch F/W version [END]

#ifdef CONFIG_HAS_EARLYSUSPEND
        touch->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 21;
        touch->early_suspend.suspend = tegra_touch_early_suspend;
        touch->early_suspend.resume = tegra_touch_late_resume;
        register_early_suspend(&touch->early_suspend);
#endif

	printk(KERN_INFO NVODM_TOUCH_NAME ": Successfully registered the ODM touch driver %x\n", (NvU32)touch->hTouchDevice);
	return 0;

// 20100718  device file management [START]
devfs_failed:
// 20100825  [START]
#ifdef FEATURE_LGE_TOUCH_GRIP_SUPPRESSION
	device_remove_file(&pdev->dev, &dev_attr_gripsuppression);
#endif
	device_remove_file(&pdev->dev, &dev_attr_fingerprint);
// 20100825  [END]
// 20100906  Touch F/W version [START]
	device_remove_file(&pdev->dev, &dev_attr_fw_version);
// 20100906  Touch F/W version [END]
	input_unregister_device(input_dev);
// 20100718  device file management [END]
	
err_input_register_device_failed:
	NvOdmTouchDeviceClose(touch->hTouchDevice);
err_kthread_create_failed:
	/* FIXME How to destroy the thread? Maybe we should use workqueues? */
	kthread_stop(touch->task);
err_open_failed:
	NvOdmOsSemaphoreDestroy(touch->semaphore);
err_semaphore_create_failed:
	NvOdmOsMutexDestroy(touch->hMutex);
	htouchMutex = NULL;
err_mutex_create_failed:
	kfree(touch);
	input_free_device(input_dev);
	return err;
}

static int tegra_touch_remove(struct platform_device *pdev)
{
	struct tegra_touch_driver_data *touch = platform_get_drvdata(pdev);

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&touch->early_suspend);
#endif
	touch->shutdown = 1;

	NvOdmTouchInterruptMask(touch->hTouchDevice, NV_TRUE);
	NvOdmOsSleepMS(50);

// 20100718  grip suppression [START]
#ifdef FEATURE_LGE_TOUCH_GRIP_SUPPRESSION
	device_remove_file(&pdev->dev, &dev_attr_gripsuppression);
#endif
// 20100718  grip suppression [END]

// 20100825  Debug Message Control (Temporary) [START]
	device_remove_file(&pdev->dev, &dev_attr_fingerprint);
// 20100825  Debug Message Control (Temporary) [END]

// 20100906  Touch F/W version [START]
	device_remove_file(&pdev->dev, &dev_attr_fw_version);
// 20100906  Touch F/W version [END]

	NvOdmOsSemaphoreDestroy(touch->semaphore);
	htouchMutex = NULL;
	NvOdmOsMutexDestroy(touch->hMutex);
	NvOdmTouchDeviceClose(touch->hTouchDevice);

	/* FIXME How to destroy the thread? Maybe we should use workqueues? */
	kthread_stop(touch->task);
	input_unregister_device(touch->input_dev);
	NvOdmOsSemaphoreDestroy(touch->semaphore);
	kfree(touch);
	return 0;
}


static void tegra_touch_shutdown(struct  platform_device *pdev)
{
	struct tegra_touch_driver_data *touch = platform_get_drvdata(pdev);
 
	printk("[TOUCH] tegra_touch_shutdown() \n");

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&touch->early_suspend);
#endif
	touch->shutdown = 1;

	NvOdmTouchInterruptMask(touch->hTouchDevice, NV_TRUE);
	NvOdmOsSleepMS(50);

// 20100718  grip suppression [START]
#ifdef FEATURE_LGE_TOUCH_GRIP_SUPPRESSION
	device_remove_file(&pdev->dev, &dev_attr_gripsuppression);
#endif
// 20100718  grip suppression [END]

// 20100825  Debug Message Control (Temporary) [START]
	device_remove_file(&pdev->dev, &dev_attr_fingerprint);
// 20100825  Debug Message Control (Temporary) [END]

// 20100906  Touch F/W version [START]
	device_remove_file(&pdev->dev, &dev_attr_fw_version);
// 20100906  Touch F/W version [END]

	NvOdmOsSemaphoreDestroy(touch->semaphore);
	htouchMutex = NULL;
	NvOdmOsMutexDestroy(touch->hMutex);
	NvOdmTouchDeviceClose(touch->hTouchDevice);

	input_unregister_device(touch->input_dev);
	kfree(touch);
}


static struct platform_driver tegra_touch_driver = {
	.probe   = tegra_touch_probe,
	.remove	 = tegra_touch_remove,
	.shutdown = tegra_touch_shutdown,
#ifdef CONFIG_HAS_EARLYSUSPEND
	.suspend = NULL,
	.resume	 = NULL,
#else
	.suspend = tegra_touch_suspend,
	.resume	 = tegra_touch_resume,
#endif

	.driver	 = {
		.name   = "tegra_touch",
	},
};

static int __devinit tegra_touch_init(void)
{
	return platform_driver_register(&tegra_touch_driver);
}

static void __exit tegra_touch_exit(void)
{
	platform_driver_unregister(&tegra_touch_driver);
}

module_init(tegra_touch_init);
module_exit(tegra_touch_exit);

MODULE_DESCRIPTION("Tegra ODM touch driver");

