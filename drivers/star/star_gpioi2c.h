/* include/asm-arm/arch-msm/simi2c.h
 *	
 * i2c simulation heaer.
 * you must assgin scl/sda pin, full/half delay
 *	
 * doncopy@lge.com	
 *	
 */

#ifndef __SIMI2C_H
#define __SIMI2C_H


static int StarI2cVal;


typedef struct StarI2csimRec
{
    NvOdmServicesGpioHandle hI2cServiceGpioHandle;
    NvOdmGpioPinHandle      hSclGpioPinHandle;
    NvU32 sclport;
    NvU32 sclpin;

    NvOdmGpioPinHandle      hSdaGpioPinHandle;
    NvU32 sdaport;
    NvU32 sdapin;
   
    NvOdmGpioPinHandle      hResetGpioPinHandle;
 
    NvU8 addr;
    NvU8 data;
} StarI2csim;
typedef struct StarI2csimRec *StarI2csimHandle;

StarI2csimHandle hStarI2csim;


#define I2cSimDelay(X)  NvOdmOsWaitUS(X)


//BL_DCDC_SDA GPIO_PQ0
#define SDA_PORT	'q'-'a'
#define SDA_PIN		0
//BL_DCDC_SCL GPIO_PQ1
#define SCL_PORT	'q'-'a'
#define SCL_PIN		1


#define HALF_DELAY	2
#define FULL_DELAY	2

#define I2C_SCL_LO	NvOdmGpioSetState(hStarI2csim->hI2cServiceGpioHandle,hStarI2csim->hSclGpioPinHandle , 0x0)
#define I2C_SCL_HI	NvOdmGpioSetState(hStarI2csim->hI2cServiceGpioHandle,hStarI2csim->hSclGpioPinHandle , 0x1)
#define I2C_SDA_LO	NvOdmGpioSetState(hStarI2csim->hI2cServiceGpioHandle,hStarI2csim->hSdaGpioPinHandle , 0x0)
#define I2C_SDA_HI	NvOdmGpioSetState(hStarI2csim->hI2cServiceGpioHandle,hStarI2csim->hSdaGpioPinHandle , 0x1)
#define I2C_SDA_LV	NvOdmGpioGetState(hStarI2csim->hI2cServiceGpioHandle,hStarI2csim->hSdaGpioPinHandle,&StarI2cVal)
	
#define I2C_SCL_DIR(d) 	NvOdmGpioConfig(hStarI2csim->hI2cServiceGpioHandle, hStarI2csim->hSclGpioPinHandle,(d+4)) 
#define I2C_SDA_DIR(d)	NvOdmGpioConfig(hStarI2csim->hI2cServiceGpioHandle, hStarI2csim->hSdaGpioPinHandle,(d+4)) 
#define I2C_SDA_IN	I2C_SDA_DIR(DIR_IN)
#define I2C_SDA_OUT	I2C_SDA_DIR(DIR_OUT)




#define GPIOF_TPS_OUTPUT	0x00040000
#define GPIOF_TPS_INPUT		0x00020000
#define DIR_OUT	1
#define DIR_IN 0

////////////////////////////////////////////////////////////////////////////////
// Adjust macro, global variable, 
// I2cSimInit and I2cSimDelay functions for your system

#define I2C_IGNORE_ACK          0
#define I2C_DELAY_HALF          I2cSimDelay(g_hdelay)
#define I2C_DELAY_FULL          I2cSimDelay(g_fdelay)

//static int g_sclpin;
//static int g_sdapin;
static int g_hdelay;
static int g_fdelay;

//#define _SIM_I2C_DEBUG_

__inline static void simi2c_sendstart(void)
{
#ifdef _SIM_I2C_DEBUG_
    printk("[START]");
#endif   		
    I2C_SDA_HI;
    I2C_SDA_OUT;
    I2C_SCL_HI;
    I2C_DELAY_HALF;
    I2C_SDA_LO;
    I2C_DELAY_HALF;
    I2C_SCL_LO;
    I2C_DELAY_FULL;
}

__inline static void simi2c_sendstop(void)
{
    I2C_SDA_LO;
    I2C_SDA_OUT;
    I2C_DELAY_HALF;
    I2C_SCL_HI;
    I2C_DELAY_HALF;
    I2C_SDA_HI;
#ifdef _SIM_I2C_DEBUG_
    printk("[STOP]");
#endif   		
    
}

__inline static unsigned int simi2c_getack(void)             
{
    unsigned char ret;
    I2C_SDA_IN;
    I2C_SCL_LO;
    I2C_DELAY_HALF;
    I2C_SCL_HI;
    I2C_DELAY_FULL;

    I2C_SDA_LV;
    ret = ( 0 == StarI2cVal);		
    //ret = (0==I2C_SDA_LV);
#if I2C_IGNORE_ACK
    ret = 1;
#endif
    I2C_SCL_LO;
    I2C_DELAY_HALF;
    //I2C_DELAY_FULL;
#if I2C_IGNORE_ACK
    ret = 1;
#endif

#ifdef _SIM_I2C_DEBUG_
	if (ret) printk("<A>"); else printk("<N>");
#endif	 
	
   if (ret!=1)
	printk("NAK!!!!\n");	
       
    return ret;
}

__inline static void simi2c_sendack(void)
{
    I2C_SDA_LO;
    I2C_SDA_OUT;
    I2C_SCL_HI;
    I2C_DELAY_FULL;
    //I2C_DELAY_HALF;
    I2C_SCL_LO;
    I2C_DELAY_FULL;
#ifdef _SIM_I2C_DEBUG_
    printk("[A]");
#endif     	    
}

__inline static unsigned char simi2c_getbyte(void)
{ 
    unsigned char tmp, ret = 0;
    int i;
    I2C_SDA_IN; // config as intput
    I2C_SCL_LO;
    I2C_DELAY_HALF;
    //I2C_DELAY_FULL;
    for( i = 7 ; i >=0 ; i--  ) 
    {
        I2C_SCL_HI;
        I2C_DELAY_FULL;
        //I2C_DELAY_HALF;
		I2C_SDA_LV;
		//StarI2cVal = -1;
        tmp = (0 != StarI2cVal);
		//printk("[DEBUG] get one bit: %02x\n", tmp);

#ifdef _SIM_I2C_DEBUG_
	if (tmp) printk("<H>"); else printk("<L>");
#endif	        
        tmp <<= i;
        ret |= tmp;
        I2C_SCL_LO;
        I2C_DELAY_HALF;
        //I2C_DELAY_FULL;
    }
    return ret;
}

__inline static void simi2c_sendbyte(unsigned char data)
{
    int i;
 
    I2C_SDA_LO;
    I2C_SDA_OUT; // config as output
    I2C_SCL_LO;
    for(i = 7; i >= 0; i--) 
    {   
        if(data & (1 << i))
        {
            I2C_SDA_HI;
#ifdef _SIM_I2C_DEBUG_
	    printk("[H]");
#endif	    	
        }
        else
        {     	
            I2C_SDA_LO;
#ifdef _SIM_I2C_DEBUG_
	    printk("[L]");
#endif            
        }
        //I2C_DELAY_HALF;
        I2C_SCL_HI;
        //I2C_DELAY_HALF;
        I2C_DELAY_FULL;
        I2C_SCL_LO;
        I2C_DELAY_FULL;
    }
}

__inline static void simi2c_sendaddr(unsigned char addr, unsigned char readflag)
{
    simi2c_sendbyte(((addr << 1) | readflag));
}


#ifdef NEW_GPIO_I2C_READ

__inline static unsigned int simi2c_read(unsigned char addr, unsigned char *pBuf, unsigned int size, unsigned char sendstop)
{
    unsigned int i = -1;
    unsigned int t;
   
    simi2c_sendstart();
    simi2c_sendaddr(SLAVE_ADDR, 1);//read
    if (simi2c_getack()) {

		//printk("send start dev addr ok\n");
        for (i = 0; i < size; i++)
        {
            pBuf[i] = simi2c_getbyte();
            simi2c_sendack();
        }
        if (sendstop)
        {
            simi2c_sendstop();
        }

	} else {

		printk("error: send dev start addr \n");
	}

#ifdef _SIM_I2C_DEBUG_
	   // printk("<0x%x:0x%x>",addr,pBuf[0]);
#endif     
    return i;
}
#else //old
__inline static unsigned int simi2c_read(unsigned char addr, unsigned char *pBuf, unsigned int size, unsigned char sendstop)
{
    unsigned int i = -1;
   
    simi2c_sendstart();
    simi2c_sendaddr(SLAVE_ADDR, 1);//read
    if (simi2c_getack())
    {
		//printk("send start dev addr ok\n");
        for (i = 0; i < size; i++)
        {
            pBuf[i] = simi2c_getbyte();
            simi2c_sendack();
        }
        if (sendstop)
        {
            simi2c_sendstop();
        }
    }
#ifdef _SIM_I2C_DEBUG_
	   // printk("<0x%x:0x%x>",addr,pBuf[0]);
#endif     
    return i;
}
#endif //NEW_GPIO_I2C_READ

__inline static unsigned int simi2c_write(unsigned char addr, unsigned char *pbuf, unsigned int size, unsigned char sendstop)
{
    unsigned int i = -1;
#ifdef _SIM_I2C_DEBUG_
	    //printk("[0x%x:0x%x]",addr,pbuf[0]);
#endif     
    
    simi2c_sendstart();
    simi2c_sendaddr(addr, 0);
    if (simi2c_getack())
    {
		//printk("send start addr ok\n");
        for (i = 0; i < size; i++)
        {
            simi2c_sendbyte(pbuf[i]);
            if (!simi2c_getack())
            {
				printk("write error\n");	
                goto clean;
            }
        }
        if (sendstop)
        {
            simi2c_sendstop();
        }
    }

	return 0;
    
clean:
    return i;
}
#endif //__SIMI2C_H


