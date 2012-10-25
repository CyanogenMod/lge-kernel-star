/**
	@brief		 HynixVGA -Hynix VGA CMOS sensor driver setting value
	@author		 calvin.hwang@lge.com
	@date		 2011.04.23
*/


#ifndef HI702_H_
#define HI702_H_
#include <linux/ioctl.h> /* For IOCTL macros */
#include <media/videobuf-dma-sg.h>
#include <media/v4l2-int-device.h>

#if defined(__cplusplus)
extern "C"
{
#endif

#define SEQUENCE_WAIT_MS   (0xFE)
#define SEQUENCE_END        (0xFF)

#define HI702_IOCTL_SET_MODE				_IOW('o', 8, struct cam_yuv_mode)
#define HI702_IOCTL_SET_COLOR_EFFECT				_IOW('o', 9, unsigned int)
#define HI702_IOCTL_SET_POWER_ON		_IOW('o', 10, __u32)  //20111006 calvin.hwang@lge.com Camsensor sync with X2
#define HI702_IOCTL_SET_WHITE_BALANCE			_IOW('o', 12, unsigned int)
#define HI702_IOCTL_SET_EXPOSURE			_IOW('o', 13, int)
#define HI702_IOCTL_SET_FPSRANGE			_IOW('o', 14, int)
#define HI702_IOCTL_SENSOR_RESET			_IOW('o', 15, int) //20111006 calvin.hwang@lge.com Camsensor sync with X2
// LGE_CHANGE_S X2_ICS [byun.youngki@lge.com], 2012-05-05, < add the scene mode for front camera  >
#define HI702_IOCTL_SET_SCENE_MODE       _IOW('o', 16, int)
// LGE_CHANGE_E X2_ICS [byun.youngki@lge.com], 2012-05-05, < add the scene mode for front camera  >
typedef struct i2c_cam_reg8_Rec {
	u8 addr;
	u8 val;
}i2c_cam_reg8;

struct cam_yuv_mode {
	int xres;
	int yres;
	int index;
};

struct cam_yuv_info {
	int mode;
	struct i2c_client *i2c_client;
	struct cam_yuv_platform_data *pdata;
	struct device dev;
};

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
      YUVCamColorEffect_VendorStartUnused = 0x7F000000, /**< Reserved region for introducing Vendor Extensions */
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

// LGE_CHANGE_S X2_ICS [byun.youngki@lge.com], 2012-05-05, < add the scene mode for front camera  >
typedef enum {
    YUVCamSceneMode_Auto = 1,
    YUVCamSceneMode_Night = 8,
    YUVCamSceneMode_Force32 = 0x7FFFFFFF
} YUVCamSceneMode;
// LGE_CHANGE_E X2_ICS [byun.youngki@lge.com], 2012-05-05, < add the scene mode for front camera  >

#ifdef __KERNEL__
struct cam_yuv_platform_data {
	int (*power_on)(void);
	int (*power_off)(void);

};
#endif /* __KERNEL__ */

#endif
