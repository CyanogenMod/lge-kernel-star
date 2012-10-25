#ifndef __GP2A_LGE_H__
#define __GP2A_LGE_H__

#define I2C_DF_NOTIFY	0x01 /* for i2c */
#define GP2A_ADDR	0x44 /* 0x88 slave addr for i2c */
#define GP2A_NAME "gp2a"

#define REGS_PROX	0x0 // Read  Only
#define REGS_GAIN	0x1 // Write Only
#define REGS_HYS	0x2 // Write Only
#define REGS_CYCLE	0x3 // Write Only
#define REGS_OPMOD	0x4 // Write Only
#define REGS_CON	0x6 // Write Only

/* power control */
#define ON	1
#define OFF	0

/* IOCTL for proximity sensor */
#define SHARP_GP2AP_IOC_MAGIC   'C'                                 
#define SHARP_GP2AP_OPEN    _IO(SHARP_GP2AP_IOC_MAGIC,1)            
#define SHARP_GP2AP_CLOSE   _IO(SHARP_GP2AP_IOC_MAGIC,2)      
#define BSS_PRINT_PROX_VALUE   _IO(SHARP_GP2AP_IOC_MAGIC,3)      

/* input device for proximity sensor */
#define USE_INPUT_DEVICE 	1

/* initial value for sensor register */
static int gp2a_original_image[8] = {
	0x00,  
	0x08,  //Gain(LED) - REGS_GAIN
	0xc2,  //Hysteresis - REGS_HYS
	0x04,  //Detection Cycle - REGS_CYCLE ->0x04 : 8ms,0x0c : 16ms, 0x14 : 32ms, 0x1c : 64ms, 0x24 : 128ms ...
	0x02,  //Operation Mode - REGS_OPMOD
};

/* driver data */
struct gp2a_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct work_struct work_prox;
	int	irq;
	struct mutex            lock;
};

/* prototype */
void gp2a_power_control(bool enable);

#endif
