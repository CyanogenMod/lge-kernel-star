/*======================================================================
                        COMMON REGISTERS
========================================================================*/                                                                                                   
#define KIONIX_ACCEL_I2C_ST_RESP            0x0C
#define KIONIX_ACCEL_I2C_WHO_AM_I           0x0F
#define KIONIX_ACCEL_I2C_TILT_POS_CUR       0x10
#define KIONIX_ACCEL_I2C_TILT_POS_PRE       0x11
#define KIONIX_ACCEL_I2C_STATUS_REG         0x18
#define KIONIX_ACCEL_I2C_INT_REL            0x1A
#define KIONIX_ACCEL_I2C_CTRL_REG1          0x1B
#define KIONIX_ACCEL_I2C_CTRL_REG2          0x1C
#define KIONIX_ACCEL_I2C_CTRL_REG3          0x1D                                                                                                                             
#define KIONIX_ACCEL_I2C_INT_CTRL_REG2      0x1F
#define KIONIX_ACCEL_I2C_TILT_TIMER         0x28
#define KIONIX_ACCEL_I2C_WUF_TIMER          0x29
#define KIONIX_ACCEL_I2C_WUF_THRESH         0x5A
    
/*=======================================================================                                                                                                    
                        KXTF9-SPECIFIC REGISTERS
========================================================================*/                                                                                                   
#define KXTF9_I2C_XOUT_HPF_L                0x00
#define KXTF9_I2C_XOUT_HPF_H                0x01
#define KXTF9_I2C_YOUT_HPF_L                0x02
#define KXTF9_I2C_YOUT_HPF_H                0x03
#define KXTF9_I2C_ZOUT_HPF_L                0x04
#define KXTF9_I2C_ZOUT_HPF_H                0x05
#define KXTF9_I2C_XOUT_L                    0x06
#define KXTF9_I2C_XOUT_H                    0x07
#define KXTF9_I2C_YOUT_L                    0x08
#define KXTF9_I2C_YOUT_H                    0x09
#define KXTF9_I2C_ZOUT_L                    0x0A
#define KXTF9_I2C_ZOUT_H                    0x0B
#define KXTF9_I2C_INT_SRC_REG1              0x15
#define KXTF9_I2C_INT_SRC_REG2              0x16
#define KXTF9_I2C_INT_CTRL_REG1             0x1E
#define KXTF9_I2C_INT_CTRL_REG3             0x20
#define KXTF9_I2C_DATA_CTRL_REG             0x21
#define KXTF9_I2C_TDT_TIMER                 0x2B
#define KXTF9_I2C_TDT_H_THRESH              0x2C
#define KXTF9_I2C_TDT_L_THRESH              0x2D
#define KXTF9_I2C_TDT_TAP_TIMER             0x2E
#define KXTF9_I2C_TDT_TOTAL_TIMER           0x2F
#define KXTF9_I2C_TDT_LATENCY_TIMER         0x30
#define KXTF9_I2C_TDT_WINDOW_TIMER          0x31
/*=======================================================================
                        KXTF9-SPECIFIC CONTROL BITS
========================================================================*/
#define INT_CTRL_REG2_XBW_MASK              0x7f  /* X-axis motion mask */
#define INT_CTRL_REG2_XBW                   0x80 
#define INT_CTRL_REG2_YBW_MASK              0xbf  /* Y-axis motion mask */
#define INT_CTRL_REG2_YBW                   0x40
#define INT_CTRL_REG2_ZBW_MASK              0xdf  /* Z-axis motion mask */
#define INT_CTRL_REG2_ZBW                   0x20 
#define KXTF9_INT_CTRL_REG1_IEN             0x20  /* enables/disables the physical interrupt pin; 1=enable; 0=disable */
#define KXTF9_INT_CTRL_REG1_IEN_MASK        0xdf  
#define CTRL_REG1_PC1                       0x80  /* operating mode 1 = full power mode; 0 = stand by mode */
#define CTRL_REG1_TPS                       0x01  /* enables tilt position function */
#define CTRL_REG1_WUFE                      0x02    /* enables wake up function */
#define CTRL_REG1_RES                       0x40    /* performance mode on KXTF9 */
#define INT_CTRL_REG3_TFUM					0x01	/* Z positive tap detection mask */
#define INT_CTRL_REG3_TFDM					0x02	/* Z negative tap detection mask */
#define INT_CTRL_REG3_TUPM					0x04	/* Y positive tap detection mask */
#define INT_CTRL_REG3_TDOM					0x08	/* Y negative tap detection mask */
#define INT_CTRL_REG3_TRIM					0x10	/* X positive tap detection mask */
#define INT_CTRL_REG3_TLEM					0x20	/* X negative tap detection mask */
#define CTRL_REG2_FUM						0x01	/* face up state mask */
#define CTRL_REG2_FDM						0x02	/* face down state mask */
#define CTRL_REG2_UPM						0x04	/* up state mask */
#define CTRL_REG2_DOM						0x08	/* down state mask */
#define CTRL_REG2_RIM						0x10	/* right state mask */
#define CTRL_REG2_LEM						0x20	/* left state mask */


#define KXTF9_IOCTL_BASE 77
/** The following define the IOCTL command values via the ioctl macros */
#define KXTF9_IOCTL_SET_DELAY       _IOW(KXTF9_IOCTL_BASE, 0, int)
#define KXTF9_IOCTL_GET_DELAY       _IOR(KXTF9_IOCTL_BASE, 1, int)
#define KXTF9_IOCTL_SET_ENABLE      _IOW(KXTF9_IOCTL_BASE, 2, int)
#define KXTF9_IOCTL_GET_ENABLE      _IOR(KXTF9_IOCTL_BASE, 3, int)
#define KXTF9_IOCTL_SET_TILT_ENABLE _IOW(KXTF9_IOCTL_BASE, 5, int)
#define KXTF9_IOCTL_SET_B2S_ENABLE  _IOW(KXTF9_IOCTL_BASE, 6, int)
#define KXTF9_IOCTL_SET_WAKE_ENABLE _IOW(KXTF9_IOCTL_BASE, 7, int)
#define KXTF9_IOCTL_SELF_TEST       _IOW(KXTF9_IOCTL_BASE, 8, int)
#define KXTF9_IOCTL_READ_ACCEL_XYZ  _IOWR(KXTF9_IOCTL_BASE, 9, int)

 /* tapping type */
enum{
	  ACCEL_TAP_MODE_SINGLE = 1,
	  ACCEL_TAP_MODE_DOUBLE 	
};

/* tapping direction */
enum{
	  ACCEL_TAP_UP  = 1,		 
	  ACCEL_TAP_DOWN,	   
	  ACCEL_TAP_LEFT,
	  ACCEL_TAP_RIGHT,
	  ACCEL_TAP_FRONT,
	  ACCEL_TAP_BACK
};

/* flip type */
enum{
	  ACCEL_FLIP_UPSIDE_DOWN   = 1,
	  ACCEL_FLIP_DOWNSIDE_UP
};

/* function */
void star_accel_enable_irq(void);
void star_accel_disable_irq(void);

int lge_sensor_shutdown_kxtf9(void);
int lge_sensor_restart_kxtf9(void);
