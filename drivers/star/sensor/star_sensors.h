#ifndef __STAR_SENSOR_H

#define STAR_SENSOR_IOCTL_NAME          "sensors"           /* for check sensor initialization */
#define STAR_IOCTL_BASE     0x90
#define MOTION_IOCTL_WAIT_SENSOR_INIT               _IOWR(STAR_IOCTL_BASE, 0x01, int)

#endif
