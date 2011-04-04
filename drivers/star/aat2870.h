

#ifndef __aat2870_H__ 
#define __aat2870_H__

/*
 * AAT2870 Registers
 */
#define AAT2870_REG_ENCHn       0x00        // channel enable
#define AAT2870_REG_BLM         0x01        // current Magnitude

#define AAT2870_REG_AMB 		0x11

#define AAT2870_REG_LDOAB       0x24        //LDOA, LDOB Output voltage
#define AAT2870_REG_EN_LDO      0x26        //enable LDO
#define AAT2870_REG_ALS         0x0E        //ALS mode config


//AAT2870 OP Mode
#define AAT2870_OP_MODE_NORMAL (1 << 0)
#define AAT2870_OP_MODE_ALC (1 << 1)



#define AAT2870_MAX_LIGHT_INTENSITY 0x16


#endif //__aat2870_H__

