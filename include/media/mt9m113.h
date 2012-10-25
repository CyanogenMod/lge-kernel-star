/**
 * Copyright (c) 2011 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#ifndef __MT9M113_H__
#define __MT9M113_H__

#include <linux/ioctl.h>  /* For IOCTL macros */

#define MT9M113_IOCTL_SET_MODE		_IOW('o', 1, struct mt9m113_mode)
#define MT9M113_IOCTL_GET_STATUS		_IOR('o', 2, struct mt9m113_status)

//MOBII_CHANGE_S dongki.han : customsetting
#define SEQUENCE_WAIT_MS   (0xFE)
#define SEQUENCE_END        (0xFF)

#define MT9M113_IOCTL_SET_COLOR_EFFECT                _IOW('o', 9, unsigned int)
#define MT9M113_IOCTL_SET_POWER_ON        _IOW('o', 10, __u32)
#define MT9M113_IOCTL_SET_WHITE_BALANCE           _IOW('o', 12, unsigned int)
#define MT9M113_IOCTL_SET_EXPOSURE            _IOW('o', 13, int)
#define MT9M113_IOCTL_SET_FPSRANGE            _IOW('o', 14, int)
#define MT9M113_IOCTL_SENSOR_RESET            _IOW('o', 15, int)
#define MT9M113_IOCTL_SET_SCENE_MODE          _IOW('o', 16, int)



typedef enum {
    YUVCamEffectMode_Color = 0,
    YUVCamEffectMode_WB,
    YUVCamEffectMode_Force32 = 0x7FFFFFFF
} YUVCamEffectMode;

typedef enum {
    YUVCamColorEffect_None= 0,
    YUVCamColorEffect_Negative =3,
    YUVCamColorEffect_Sketch =4,
    YUVCamColorEffect_Solarize = 10,
    YUVCamColorEffect_Posterize = 0x30000000,
    YUVCamColorEffect_Sepia,
    YUVCamColorEffect_Mono,
    YUVCamColorEffect_Aqua,
    YUVCamColorEffect_Force32 = 0x7FFFFFFF
} YUVCamColorEffect;


typedef enum {
    YUVCamWhitebalance_Off = 0,
    YUVCamWhitebalance_Auto = 1,
    YUVCamWhitebalance_SunLight,
    YUVCamWhitebalance_CloudyDayLight,
    YUVCamWhitebalance_Shade,
    YUVCamWhitebalance_Tungsten,
    YUVCamWhitebalance_Fluorescent,
    YUVCamWhitebalance_Incandescent,
    YUVCamWhitebalance_Flash,
    YUVCamWhitebalance_Horizon,
    YUVCamWhitebalance_Force32 = 0x7FFFFFFF
} YUVCamUserWhitebalance;

typedef enum {
    YUVCamSceneMode_Auto = 1,
    YUVCamSceneMode_Night = 8,
    YUVCamSceneMode_Force32 = 0x7FFFFFFF
} YUVCamSceneMode;

//MOBII_CHANGE_E dongki.han : customsetting

struct mt9m113_mode {
	int xres;
	int yres;
};

struct mt9m113_status {
	int data;
	int status;
};

#ifdef __KERNEL__
struct mt9m113_platform_data {
	int (*power_on)(void);
	int (*power_off)(void);

};
#endif /* __KERNEL__ */

#endif  /* __MT9M113_H__ */

