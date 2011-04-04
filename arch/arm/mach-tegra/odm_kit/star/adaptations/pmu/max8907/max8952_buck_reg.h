

#ifndef INCLUDE_MAX8952_BUCK_REG_HEADER
#define INCLUDE_MAX8952_BUCK_REG_HEADER


// Registers

/* field defines for register bit ops */

#define MAX8952_MODE0		0x00
#define MAX8952_MODE1		0x01
#define	MAX8952_MODE2		0x02
#define MAX8952_MODE3		0x03
#define MAX8952_CONTROL		0x04
#define MAX8952_SYNC		0x05
#define MAX8952_RAMP		0x06
#define MAX8952_CHIP_ID1	0x08
#define MAX8952_CHIP_ID2	0x09

#define MAX8952_REG_INVALID 0xFF

//20100819, taewan.kim@lge.com, Voltage bug fix
#define MAX8952_FPWM_EN0	0x80

#endif
