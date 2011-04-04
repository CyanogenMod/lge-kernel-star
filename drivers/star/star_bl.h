#ifndef __STAR_BL_H__
#define __STAR_BL_H__


#include <linux/ioctl.h>






#define STAR_BL_MAGIC 0x91

typedef struct {

    unsigned long size;  
    unsigned char buff[128];  
} __attribute__ ((packed)) star_bl_info; 




// 101017 sk.jang@lge.com added some variables to adjust backlight brightness
//int remainder;
//int backlight_minimum;
//int numerator1 , numerator2;
//int turning_point;


#define STAR_BL_IOCTL_READ _IOR(STAR_BL_MAGIC, 1, star_bl_info)
#define STAR_BL_IOCTL_WRITE _IOW(STAR_BL_MAGIC, 2, star_bl_info)
#define STAR_BL_IOCTL_WRITE_READ _IOWR(STAR_BL_MAGIC, 3, star_bl_info)


#define STAR_BL_IOCTL_SET_OP_MODE _IOW(STAR_BL_MAGIC, 3, unsigned int)

#define STAR_BL_IOCTL_SET_DISP_ON _IOW(STAR_BL_MAGIC, 4, unsigned int)
#define STAR_BL_IOCTL_SET_DISP_OFF _IOW(STAR_BL_MAGIC, 5, unsigned int)

#define STAR_BL_IOCTL_SET_LIGHT_INTENSITY _IOW(STAR_BL_MAGIC, 6, unsigned int)


#endif //__STAR_BL_H__


