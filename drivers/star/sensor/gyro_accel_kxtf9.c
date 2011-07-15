#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/input.h>
#include <linux/sysfs.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/hrtimer.h>
//#include <linux/i2c.h>

#include "nvcommon.h"
#include "nvodm_services.h"
#include "nvodm_query.h"
#include "nvodm_query_discovery.h"
#include "star_accel.h"
#include "lge_sensor_verify.h"
#include "gyro_accel.h"

#define STAR_ACCEL_DEBUG 0

#define G_MAX			8000
/* OUTPUT REGISTERS */
#define XOUT_L			0x06
#define XOUT_H			0x07
#define YOUT_L			0x08
#define YOUT_H			0x09
#define ZOUT_L			0x0A
#define ZOUT_H			0x0B
#define INT_SRC_REG1		0x15
#define INT_STATUS_REG		0x16
#define TILT_POS_CUR		0x10
#define INT_REL			0x1A
#define WHO_AM_I		0x0F
/* CONTROL REGISTERS */
#define DATA_CTRL		0x21
#define CTRL_REG1		0x1B
#define INT_CTRL1		0x1E
#define CTRL_REG3		0x1D
#define TILT_TIMER		0x28
#define WUF_TIMER		0x29
#define WUF_THRESH		0x5A
#define TDT_TIMER		0x2B
/* CONTROL REGISTER 1 BITS */
#define PC1_OFF			0x00
#define PC1_ON			0x80
/* INTERRUPT SOURCE 2 BITS */
#define TPS			0x01
#define TDTS0			0x04
#define TDTS1			0x08
/* INPUT_ABS CONSTANTS */
#define FUZZ			32
#define FLAT			32
/* RESUME STATE INDICES */
#define RES_DATA_CTRL		0
#define RES_CTRL_REG1		1
#define RES_INT_CTRL1		2
#define RES_TILT_TIMER		3
#define RES_CTRL_REG3		4
#define RES_WUF_TIMER		5
#define RES_WUF_THRESH		6
#define RES_TDT_TIMER		7
#define RES_TDT_H_THRESH	8
#define RES_TDT_L_THRESH	9
#define RES_TAP_TIMER		10
#define RES_TOTAL_TIMER		11
#define RES_LAT_TIMER		12
#define RES_WIN_TIMER		13
#define RESUME_ENTRIES		14

static atomic_t		tap_flag;
static atomic_t		flip_flag;

#define GYRO_ACCEL_I2C_MAX_RETRY 5
#define GYRO_ACCEL_I2C_TIMEOUT 10

char brdrev[5]; 
#define BD_REVB	"B" 
#define BD_REVC "C"


struct accelerometer_data {
    NvU32 x;
    NvU32 y;
    NvU32 z;
};


typedef struct star_accel_device_data
{
	struct work_struct		work;
	NvBool					use_irq;

	NvOdmServicesI2cHandle h_gen2_i2c;
	NvOdmServicesGpioHandle h_accel_gpio;
	NvOdmGpioPinHandle	h_accel_gpio_pin;
	NvOdmServicesGpioIntrHandle h_accel_intr;
	NvOdmServicesPmuHandle	h_accel_pmu;
	NvU32	i2c_address;
	NvU32	vdd_id;
	NvU32	intr_pin;
	NvU32	intr_port;
	struct  input_dev *input_dev;
	struct delayed_work delayed_work_accel;
	
    struct accelerometer_data prev_data;
    struct accelerometer_data min_data;
    struct accelerometer_data max_data;
}star_accel_device;

static star_accel_device *g_accel;
int flip_pre_pre_state = 0;
int call_once = 1;
unsigned char* transfer_data_read;
unsigned char* transfer_data_write;
extern int reboot;

void star_accel_enable_irq(void)
{
	NvOdmGpioInterruptMask(g_accel->h_accel_intr, NV_FALSE);
	//NvOdmGpioInterruptDone(g_accel->h_accel_intr);
}
void star_accel_disable_irq(void)
{
	NvOdmGpioInterruptMask(g_accel->h_accel_intr, NV_TRUE);
}

static ssize_t motion_tap_onoff_show(struct device *dev, struct device_attribute *attr, char *buf, size_t count)
{
	u32    val;
	val = atomic_read(&tap_flag);
	return sprintf(buf, "%d\n",val);
}

static ssize_t motion_tap_onoff_store(struct device *dev, struct device_attribute *attr, char *buf, size_t count)
{
	u32    val;
	val = simple_strtoul(buf, NULL, 10);
	//lprintk("[motion_set_tap_onoff_store] tap.... flag [%d]\n",val);

	if (val) {
		atomic_set(&tap_flag, 1);
	} else {
		atomic_set(&tap_flag, 0);
	}

	return count;
}

static ssize_t motion_flip_onoff_show(struct device *dev, struct device_attribute *attr, char *buf, size_t count)
{
	u32    val;
	val = atomic_read(&flip_flag);
	return sprintf(buf, "%d\n",val);
}

static ssize_t motion_flip_onoff_store(struct device *dev, struct device_attribute *attr, char *buf, size_t count)
{
	u32    val;
	val = simple_strtoul(buf, NULL, 10);
	//lprintk("[motion_set_flip_onoff_store]  flag [%d]\n",val);

	if (val) {
		atomic_set(&flip_flag, 1);
	} else {
		atomic_set(&flip_flag, 0);
	}

	return count;
}


static DEVICE_ATTR(tap_onoff, 0666, motion_tap_onoff_show, motion_tap_onoff_store);
static DEVICE_ATTR(flip_onoff, 0666, motion_flip_onoff_show, motion_flip_onoff_store);

static struct attribute *star_motion_attributes[] = {
	&dev_attr_tap_onoff.attr,
	&dev_attr_flip_onoff.attr,
	NULL
};

static const struct attribute_group star_motion_group = {
	.attrs = star_motion_attributes,
};

static bool star_accel_i2c_write_data( star_accel_device *accel, unsigned char reg, unsigned char* data, unsigned char len )
{
    int i;
    NvOdmI2cStatus i2c_status = NvOdmI2cStatus_Timeout;
    NvOdmI2cTransactionInfo info;
    //unsigned char* transfer_data;

    // this code is assumed that length should be under 10
    if (len > 10) {
        printk("[star accel driver] %s : length should be 1 ( len = %d)\n", __func__, len);
        return false;
    }

    //transfer_data = (unsigned char*)NvOdmOsAlloc(len+1);

    for (i = 0; i < GYRO_ACCEL_I2C_MAX_RETRY && i2c_status != NvOdmI2cStatus_Success; i++)
    {
        //transfer_data[0] = reg; 
        //NvOdmOsMemcpy( &transfer_data[1], data, (size_t)len);
        transfer_data_write[0] = reg; 
        NvOdmOsMemcpy( &transfer_data_write[1], data, (size_t)len);

        info.Address = accel->i2c_address;
        //info.Buf = transfer_data;
        info.Buf = transfer_data_write;
        info.Flags = NVODM_I2C_IS_WRITE;
        info.NumBytes = len+1;
	
        i2c_status = NvOdmI2cTransaction( accel->h_gen2_i2c, &info, 1, 400, GYRO_ACCEL_I2C_TIMEOUT );
    }

    if( i2c_status != NvOdmI2cStatus_Success )
    {    
        printk("[star accel driver] %s : i2c transaction error(Number = %d)!\n",__func__,i2c_status);
        //NvOdmOsFree(transfer_data);

        //reboot sensors
        reboot = 1;

        return false;
    }    
    //NvOdmOsFree(transfer_data); 
    return true;
}

static bool star_accel_i2c_read_data( star_accel_device *accel, unsigned char reg, unsigned char* data, unsigned char len )
{
    int i;
    NvOdmI2cStatus i2c_status = NvOdmI2cStatus_Timeout;
    NvOdmI2cTransactionInfo info[2];
    //unsigned char* transfer_data;

    // this code is assumed that length should be under 10
    if (len > 10) {
        printk("[star accel driver] %s : length should be 1 ( len = %d)\n", __func__, len);
        return false;
    }
    //transfer_data = (unsigned char*)NvOdmOsAlloc(len);

    for (i = 0; i < GYRO_ACCEL_I2C_MAX_RETRY && i2c_status != NvOdmI2cStatus_Success; i++)
    {
       info[0].Address = g_accel->i2c_address;
       info[0].Buf = &reg;
       info[0].Flags = NVODM_I2C_IS_WRITE;
       info[0].NumBytes = 1;

       info[1].Address = ( g_accel->i2c_address | 0x01 );
       //info[1].Buf = transfer_data;
       info[1].Buf = transfer_data_read;
       info[1].Flags = 0;
       info[1].NumBytes = len;

       i2c_status = NvOdmI2cTransaction( g_accel->h_gen2_i2c, info, 2, 400, GYRO_ACCEL_I2C_TIMEOUT );
    }

    if( i2c_status != NvOdmI2cStatus_Success )
    {
        printk("[star driver] %s : i2c transaction error(Number= %d)!\n",__func__, i2c_status);
        //NvOdmOsFree(transfer_data);

        //reboot sensors
        reboot = 1;

        return false;
    }
    NvOdmOsMemcpy( data, transfer_data_read, len );
    //NvOdmOsMemcpy( data, transfer_data, len );
    //NvOdmOsFree(transfer_data);
    return true;
}

static void star_accel_set_sample_rate( star_accel_device *accel, unsigned int samplerate )
{
	unsigned int sampledata[7][2] =
	{
		{ 12,  0x00 },
		{ 25,  0x01 },
		{ 50,  0x02 },
		{ 100, 0x03 },
		{ 200, 0x04 },
		{ 400, 0x05 },
		{ 800, 0x06 },
	};
	unsigned char i;
	unsigned char rate;
	for( i= 0; i< 7; i++ )
	{
		if(sampledata[i][0] == samplerate )
		{
			rate = (unsigned char)sampledata[i][1];
			break;
		}
	}
	#if STAR_ACCEL_DEBUG
		printk("[skhwang][%s] i = %d, rate = %d, rate of index = %d\n", __func__, i, sampledata[i][0], rate); 
	#endif
	star_accel_i2c_write_data( accel, KXTF9_I2C_DATA_CTRL_REG, &rate, 1 );
}


NvBool NvAccelerometerI2CSetRegsPassThrough(NvU8 offset, NvU8* value, NvU32 len)
{
	int status = 1;

	status = star_accel_i2c_write_data( g_accel, offset, value, 1 );

	return status;
}

NvBool NvAccelerometerI2CGetRegsPassThrough(NvU8 offset, NvU8* value, NvU32 len)
{
	int status = 1;

	status = star_accel_i2c_read_data( g_accel, offset, value, 1 );

	return status;
}

static int kxtf9_get_acceleration_data_test(int *xyz_mg)
{
#ifndef KXTF9_I2C_XOUT_L
#define KXTF9_I2C_XOUT_L 0x06
#endif

	int err;
	u8 acc_data[6]; // xyz data bytes from hardware
	char buf, Res = 0, G_range = 0;
	int range = 0, sensitivity;
	int x_sign, y_sign, z_sign;

	int hw_cnt[3] ; // calculated count value
	int hw_mg[3]; // calculated mg value
	int xyz_cnt[3];
	int temp;
	//int xyz_mg[3]; // acceleration data by mg (last return value)
	//		err = kxtf9_i2c_read(tf9, 0x1B, &buf, 1);
	//		if(err < 0 ) {
	//			dev_err(&tf9->client->dev, "can't get acceleration data, err=%d\n", err);
	//			return err;
	//		}
	star_accel_i2c_read_data(g_accel, 0x1B, &buf, 1);
	G_range = (buf & 0x18) >> 3;
	switch(G_range)
	{
		case 0:
			range = 2;
			break;
		case 1:
			range = 4;
			break;
		case 2:
			range = 8;
			break;
		default:
			break;
	}

	//		err = kxtf9_i2c_read(tf9, KXTF9_XOUT_L, acc_data, 6);
	//		if(err < 0){
	//			dev_err(&tf9->client->dev, "can't get acceleration data, err=%d\n", err);
	//			return err;
	//		}
	star_accel_i2c_read_data(g_accel, KXTF9_I2C_XOUT_L, &acc_data[0], 6);

       //printk("YJ- raw_data=[%02x, %02x, %02x, %02x, %02x, %02x] \n", acc_data[0], acc_data[1], acc_data[2], acc_data[3], acc_data[4] , acc_data[5] );

	Res = buf & 0x40;
//	switch(Res)
//	{
//		case 0x00: // 8-bit : low resolution state
//			break;
//		case 0x40: // 12-bit : high-resolution state
			temp = (int)((acc_data[1]<<4) + (acc_data[0]>>4));
			// adjust for twos complement
			if(temp < 2048)
				temp = 2048 + temp;
			else
				temp = 2048- (4096-temp);
			hw_cnt[0] = temp;
			// convert raw data 
			hw_mg[0] = ((temp-2048)*1000)/1024; 
			//printk("YJ- temp1=[%02x] \n", temp);


			temp = (int)((acc_data[3]<<4) + (acc_data[2]>>4));
			// adjust for twos complement
			if(temp < 2048)
				temp = 2048 + temp;
			else
				temp = 2048- (4096-temp);
			hw_cnt[1] = temp;
			// convert raw data 
			hw_mg[1] = ((temp-2048)*1000)/1024; 
			//printk("YJ- temp2=[%02x] \n", temp);
			temp = (int)((acc_data[5]<<4) + (acc_data[4]>>4));
			// adjust for twos complement
			if(temp < 2048)
				temp = 2048 + temp;
			else
				temp = 2048- (4096-temp);
			hw_cnt[2] = temp;
			// convert raw data 
			hw_mg[2] = ((temp-2048)*1000)/1024; 
			//printk("YJ- temp3=[%02x] \n", temp);

//			break;
//	}

#if defined(STAR_COUNTRY_COM) && defined(STAR_OPERATOR_OPEN)
	xyz_mg[0] = hw_mg[1];
	xyz_mg[1] = -hw_mg[0];
	xyz_mg[2] = hw_mg[2];
#elif defined(STAR_COUNTRY_KR) && defined(STAR_OPERATOR_SKT)
	xyz_mg[0] = -hw_mg[1];
	xyz_mg[1] = -hw_mg[0];
	xyz_mg[2] = -hw_mg[2];
#elif defined(STAR_COUNTRY_US) && defined(STAR_OPERATOR_TMO)

	if(!strncmp(brdrev,BD_REVB,1)){
		xyz_mg[0] = hw_mg[0];
		xyz_mg[1] = -hw_mg[1];
		xyz_mg[2] = -hw_mg[2];
	} else { 
		xyz_mg[0] = hw_mg[1];
		xyz_mg[1] = -hw_mg[0];
		xyz_mg[2] = hw_mg[2];  
	}
#elif defined(STAR_COUNTRY_CA) && defined(STAR_OPERATOR_AWC)
	if(!strncmp(brdrev,BD_REVB,1)){
		xyz_mg[0] = hw_mg[0];
		xyz_mg[1] = -hw_mg[1];
		xyz_mg[2] = -hw_mg[2];
	} else { 
		xyz_mg[0] = hw_mg[1];
		xyz_mg[1] = -hw_mg[0];
		xyz_mg[2] = hw_mg[2];  
	}
#elif defined(STAR_COUNTRY_CA) && defined(STAR_OPERATOR_AVC)
	if(!strncmp(brdrev,BD_REVB,1)){
	xyz_mg[0] = hw_mg[0];
	xyz_mg[1] = -hw_mg[1];
	xyz_mg[2] = -hw_mg[2];
	} else { 
		xyz_mg[0] = hw_mg[1];
		xyz_mg[1] = -hw_mg[0];
		xyz_mg[2] = hw_mg[2];  
	}
#else
	if(!strncmp(brdrev,BD_REVB,1)){
		xyz_mg[0] = hw_mg[0];
		xyz_mg[1] = -hw_mg[1];
		xyz_mg[2] = -hw_mg[2];
	} else { 
	xyz_mg[0] = hw_mg[1];
	xyz_mg[1] = -hw_mg[0];
	xyz_mg[2] = hw_mg[2];
	}
#endif


	//printk("YJ- CNT=[  %4d,  %4d,  %4d] , ACC=[  %4d,  %4d,  %4d] \n", hw_cnt[0], hw_cnt[1], hw_cnt[2], hw_mg[0], hw_mg[1] , hw_mg[2] );
	return 0;
}

int kxtf9_get_acceleration_data_passthrough(int *xyz_data)
{
	int xyz[3] = { 0 };
	int err;

	err = kxtf9_get_acceleration_data_test(xyz);

	if (err < 0){
		return err;
	} else{
		xyz_data[0] = xyz[0];
		xyz_data[1] = xyz[1];
		xyz_data[2] = xyz[2];		
		return 0;
	}	
}

static void star_accel_whoami( void )
{
	#define WHO_AM_I 0x0f
	
	unsigned char value;

	star_accel_i2c_read_data( g_accel, WHO_AM_I, &value, 1 );

	#if STAR_ACCEL_DEBUG
	printk("[skhwang][%s]: WHO AM I = %#x \n", __func__, value );
	#endif
}

// 20100702  Power control bug fix [START]
static void star_accel_set_power_rail(NvU32 vdd_id, NvBool is_enable )
{
	NvOdmServicesPmuVddRailCapabilities vddrailcap;
	NvU32 settletime;

    NvOdmServicesPmuHandle h_pmu = NvOdmServicesPmuOpen();

	if(h_pmu)
	{
		NvOdmServicesPmuGetCapabilities( h_pmu, vdd_id, &vddrailcap );
		if( is_enable )
		{
			#if STAR_ACCEL_DEBUG
			printk("[skhwang] PMU enable\n");
			#endif
			NvOdmServicesPmuSetVoltage(h_pmu, vdd_id, vddrailcap.requestMilliVolts, &settletime);
		}
		else
		{
			#if STAR_ACCEL_DEBUG
			printk("[skhwang] PMU do not enable\n");
			#endif
			NvOdmServicesPmuSetVoltage(h_pmu, vdd_id, NVODM_VOLTAGE_OFF, &settletime);
		}

		if(settletime)
			NvOdmOsWaitUS(settletime);
	}
	#if STAR_ACCEL_DEBUG
		printk("[skhwang] voltage =  %d or %d \n", vddrailcap.requestMilliVolts, vddrailcap.MinMilliVolts);
	#endif
	NvOdmServicesPmuClose(h_pmu);
}
// 20100702  Power control bug fix [END]


int lge_sensor_verify_kxtf9(void) 
{
	unsigned char value = 0;

	star_accel_i2c_read_data( g_accel, 0x0F, &value, 1 );
	printk("[%s] ### Accelerometer ## KXTF9 ## WHO_AM_I = 0x%x \n",MOD_TAG,value);

	if(KXTF9_WHID != value)
		return SENSOR_ERROR;

	return SENSOR_OK;
}

int lge_sensor_shutdown_kxtf9(void)
{
	NvU32 I2cInstance = 0;
	const NvOdmPeripheralConnectivity *pcon;
	int err;
	NvBool found_gpio=NV_FALSE, found_i2c=NV_FALSE;

	int loop;

	printk("[%s:%d] \n", __FUNCTION__, __LINE__);
	#if STAR_ACCEL_DEBUG
		printk("[skhwang]%s\n", __func__);
	#endif

	pcon = (NvOdmPeripheralConnectivity*)NvOdmPeripheralGetGuid(NV_ODM_GUID('a','c','c','e','l','e','r','o'));

	for(loop = 0; loop< pcon->NumAddress; loop++)
	{
		switch(pcon->AddressList[loop].Interface)
		{
			case NvOdmIoModule_I2c:
				g_accel->i2c_address = (pcon->AddressList[loop].Address<<1);
				I2cInstance = pcon->AddressList[loop].Instance;
				found_i2c = NV_TRUE;
				break;
			case NvOdmIoModule_Gpio:
				g_accel->intr_port = pcon->AddressList[loop].Instance;
				g_accel->intr_pin = pcon->AddressList[loop].Address;
				found_gpio = NV_TRUE;
				break;
			case NvOdmIoModule_Vdd:
				g_accel->vdd_id = pcon->AddressList[loop].Address;
				star_accel_set_power_rail(g_accel->vdd_id, NV_FALSE);
				NvOdmOsWaitUS(30);
				break;
			default:
				break;
		}
	}
    return SENSOR_OK;
}

int lge_sensor_restart_kxtf9(void)
{
	NvU32 I2cInstance = 0;
	const NvOdmPeripheralConnectivity *pcon;
	int err;
	NvBool found_gpio=NV_FALSE, found_i2c=NV_FALSE;

	int loop;

	printk("[%s:%d] \n", __FUNCTION__, __LINE__);
	#if STAR_ACCEL_DEBUG
		printk("[skhwang]%s\n", __func__);
	#endif

	pcon = (NvOdmPeripheralConnectivity*)NvOdmPeripheralGetGuid(NV_ODM_GUID('a','c','c','e','l','e','r','o'));

	for(loop = 0; loop< pcon->NumAddress; loop++)
	{
		switch(pcon->AddressList[loop].Interface)
		{
			case NvOdmIoModule_I2c:
				g_accel->i2c_address = (pcon->AddressList[loop].Address<<1);
				I2cInstance = pcon->AddressList[loop].Instance;
				found_i2c = NV_TRUE;
				break;
			case NvOdmIoModule_Gpio:
				g_accel->intr_port = pcon->AddressList[loop].Instance;
				g_accel->intr_pin = pcon->AddressList[loop].Address;
				found_gpio = NV_TRUE;
				break;
			case NvOdmIoModule_Vdd:
				g_accel->vdd_id = pcon->AddressList[loop].Address;
				star_accel_set_power_rail(g_accel->vdd_id, NV_TRUE);
				NvOdmOsWaitUS(30);
				break;
			default:
				break;
		}
	}
	
    return SENSOR_OK;

}


static int star_accel_misc_open( struct inode *inode, struct file *file )
{
	#if STAR_ACCEL_DEBUG
		printk("[skhwang][%s] function...\n");
	#endif
	return nonseekable_open(inode, file);
}

static int star_accel_misc_ioctl( struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg )
{
	void __user *argp = (void __user *)arg;
	NvS32 x=0, y=0, z=0;
	int xyz[3] = { 0 };
	#if 0	
	switch(cmd)
	{
		case KXTF9_IOCTL_READ_ACCEL_XYZ:
			star_accel_get_data( g_accel, &x, &y, &z );
			xyz[0] = x; xyz[1] = y; xyz[2] = z;
			if( copy_to_user(argp, xyz, sizeof(int)*3))
				return -EINVAL;
			break;
		default:
			return -EINVAL;
	}
	#endif
	return 0;
}

static const struct file_operations star_accel_misc_fop =
{
	.owner = THIS_MODULE,
	.open = star_accel_misc_open,
	.ioctl = star_accel_misc_ioctl,
};

static struct miscdevice star_accel_misc_device =
{
	.minor = MISC_DYNAMIC_MINOR,
	.name = "kxtf9",
	.fops = &star_accel_misc_fop,
};

int tegra_accel_hw_init(void)
{
	unsigned char val_shadow, dummy;
	int err = -1;
	u8 buf[7];
	unsigned char tdt_timer_val = 0x78;
	unsigned char tdt_tap_timer = 0xF2;
	unsigned char tdt_total_timer = 0x36;
	unsigned char tdt_latency_timer = 0x3c;
	unsigned char tdt_window_timer = 0xb4;
	unsigned char x_gain, y_gain, z_gain;
	unsigned char tap_threshold = 0x1e;

	/*----------------who am i -------------------*/
	star_accel_i2c_read_data(g_accel, KIONIX_ACCEL_I2C_WHO_AM_I, &val_shadow, 1);
	printk("[%s:%d] [MOTION] Device ID (0x%02x) \n", __FUNCTION__, __LINE__, val_shadow);

	/* Need 50ms until the RAM load is finished after Power-up	*/
	star_accel_i2c_read_data(g_accel, CTRL_REG3, &val_shadow, 1);
	val_shadow |= PC1_ON;
	star_accel_i2c_write_data(g_accel, CTRL_REG3, &val_shadow, 1);
	msleep(200);

	//KXTF9_set_G_range(2)
	star_accel_i2c_read_data(g_accel, CTRL_REG1, &val_shadow, 1);
	val_shadow &= ~(0x18); //G range 2
	star_accel_i2c_write_data(g_accel, CTRL_REG1, &val_shadow, 1);	

	//KXTF9_set_resolution(12);
	star_accel_i2c_read_data(g_accel, CTRL_REG1, &val_shadow, 1);	
	val_shadow |= 0x40; //resolution 12bit
	star_accel_i2c_write_data(g_accel, CTRL_REG1, &val_shadow, 1);

	/*---------------- tilt & shake -------------------*/
	//KXTF9_set_hpf_odr(50);
	star_accel_i2c_read_data(g_accel, DATA_CTRL, &val_shadow, 1);
	val_shadow &= ~(0x30); /* set tap ODR to 50Hz */
	star_accel_i2c_write_data(g_accel, DATA_CTRL, &val_shadow, 1);
	
	//KXTF9_set_lpf_odr(100);
	star_accel_i2c_read_data(g_accel, DATA_CTRL, &val_shadow, 1);
	val_shadow &= ~(0x03); /* set LPF rolloff to 100 Hz */
	val_shadow |= 0x04; /* set LPF rolloff to 100 Hz */
	star_accel_i2c_write_data(g_accel, DATA_CTRL, &val_shadow, 1);

	/*--------------flip - screen rotation --------------*/
	//KXTF9_set_odr_tilt(12);
	star_accel_i2c_read_data(g_accel, CTRL_REG3, &val_shadow, 1);
	val_shadow &= ~(0x20); 	/* set all ODR's to 12.5Hz */
	val_shadow |= 0x40;	/* set all ODR's to 12.5Hz */
	star_accel_i2c_write_data(g_accel, CTRL_REG3, &val_shadow, 1);

		//KIONIX_ACCEL_position_mask_fu
		star_accel_i2c_read_data(g_accel, 0x1C, &val_shadow, 1); //CTRL_REG2
		val_shadow |= 0x03;
		star_accel_i2c_write_data(g_accel, 0x1C, &val_shadow, 1);

	//KIONIX_ACCEL_enable_tilt_function
	star_accel_i2c_read_data(g_accel, CTRL_REG1, &val_shadow, 1);
	val_shadow |= 0x01;/* sets TPE bit to enable tilt position function*/
	star_accel_i2c_write_data(g_accel, CTRL_REG1, &val_shadow, 1);

	//star_accel_i2c_read_data(g_accel, TILT_TIMER, &val_shadow, 1);
	//val_shadow |= 0x05;/* tilt timer 80ms*/
	//star_accel_i2c_write_data(g_accel, TILT_TIMER, &val_shadow, 1);

	/*---------------- Tapping  -----------------------*/
	star_accel_i2c_read_data(g_accel, CTRL_REG3, &val_shadow, 1);
	val_shadow &= ~(0x04); 	/* set all ODR's to 200Hz */
	val_shadow |= 0x08;	/* set all ODR's to 200Hz */
	star_accel_i2c_write_data(g_accel, CTRL_REG3, &val_shadow, 1);

	//KXTF9_tap_mask_TFU
	star_accel_i2c_read_data(g_accel, 0x20, &val_shadow, 1);
	val_shadow |= 0x3f;
	star_accel_i2c_write_data(g_accel, 0x20, &val_shadow, 1);
	
		/* kxtf9_accel_enable_tap */
		//tapping 200 -> 400hz
		star_accel_i2c_read_data(g_accel, CTRL_REG3, &val_shadow, 1);
		val_shadow |= 0x0c;  /* set all ODR's to 400Hz */
		star_accel_i2c_write_data(g_accel, CTRL_REG3, &val_shadow, 1);

		//tap sensitivity
		star_accel_i2c_write_data(g_accel, 0x2d, &tap_threshold, 1);
		
		star_accel_i2c_write_data(g_accel, 0x2b, &tdt_timer_val, 1); // TDT_TIMER
		star_accel_i2c_write_data(g_accel, 0x2E, &tdt_tap_timer, 1); // TDT_TAP_TIMER
		star_accel_i2c_write_data(g_accel, 0x2F, &tdt_total_timer, 1); // TDT_TOTAL_TIMER
		star_accel_i2c_write_data(g_accel, 0x30, &tdt_latency_timer, 1); // TDT_LATENCY_TIMER
		star_accel_i2c_write_data(g_accel, 0x31, &tdt_window_timer, 1); // TDT_WINDOW_TIMER

	    // X-gain control
	    star_accel_i2c_read_data(g_accel, 0x50, &x_gain, 1); // X_GAIN
	    //KIONIX_ACCEL_write_byte(0x50, 200 );
	    //printk("------------------------------> YJ : x_gain = 0x%x --> 200 \n", x_gain);

	    // Y-gain control
	    star_accel_i2c_read_data(g_accel, 0x51, &y_gain, 1); // Y_GAIN
	    //KIONIX_ACCEL_write_byte(0x51, 200 );
	    //printk("------------------------------> YJ : y_gain = 0x%x --> 0x%x\n", y_gain, y_gain);

	    // Z-gain control
	    star_accel_i2c_read_data(g_accel, 0x52, &z_gain, 1); // Z_GAIN
	    //KIONIX_ACCEL_write_byte(0x52, (z_gain)>>1 );
	    //printk("------------------------------> YJ : z_gain = 0x%x --> 0x%x\n", z_gain, (z_gain)>>1 );

		//KXTF9_tap_mask_all_direction
		//KXTF9_tap_mask_TFU
		star_accel_i2c_read_data(g_accel, 0x20, &val_shadow, 1);
		val_shadow |= 0x3f;
		star_accel_i2c_write_data(g_accel, 0x20, &val_shadow, 1);

		//KXTF9_enable_tap_detection
		star_accel_i2c_read_data(g_accel, CTRL_REG1, &val_shadow, 1);
		val_shadow |= 0x04; /* set TDTE bit to enable tap function */
		star_accel_i2c_write_data(g_accel, CTRL_REG1, &val_shadow, 1);	

	/*------------ Interrupt ---------------------------*/
	//KIONIX_ACCEL_int_activeh
	star_accel_i2c_read_data(g_accel, INT_CTRL1, &val_shadow, 1);
	val_shadow |= 0x10; //the polarity of physical interrupt pin to active high. 
	star_accel_i2c_write_data(g_accel, INT_CTRL1, &val_shadow, 1);	

	//KIONIX_ACCEL_int_latch
	star_accel_i2c_read_data(g_accel, INT_CTRL1, &val_shadow, 1);
	val_shadow &= ~(0x08); 	//sets the physical interrupt to a latch state. 
	star_accel_i2c_write_data(g_accel, INT_CTRL1, &val_shadow, 1);	

	star_accel_i2c_read_data(g_accel, 0x1a, &dummy, 1);

	//KIONIX_ACCEL_enable_interrupt
	star_accel_i2c_read_data(g_accel, INT_CTRL1, &val_shadow, 1);
	val_shadow |= 0x20; //enable interrupt
	star_accel_i2c_write_data(g_accel, INT_CTRL1, &val_shadow, 1);	

	//KIONIX_ACCEL_enable_outputs
	star_accel_i2c_read_data(g_accel, CTRL_REG1, &val_shadow, 1);
	val_shadow |= 0x80; /* sets PC1 bit to be in power up state */
	star_accel_i2c_write_data(g_accel, CTRL_REG1, &val_shadow, 1);

	return 0;
}


void star_accel_process_directional_tap(unsigned char tap_mode, unsigned char tap_direction)
{
	int type =0,direction =0;

	if(tap_mode == 2)
	     type = ACCEL_TAP_MODE_DOUBLE; 
	else
	     type = ACCEL_TAP_MODE_SINGLE; 	

#if defined(STAR_COUNTRY_COM) && defined(STAR_OPERATOR_OPEN)
	switch(tap_direction)
	{
		case INT_CTRL_REG3_TFUM : // Z+
			direction = ACCEL_TAP_BACK;

			if(tap_mode == 2)		printk("[%s:%d] ---I--- Z+	Directional-Tap  : (2) Back \n",__FUNCTION__,__LINE__);  
			else					printk("[%s:%d] ---I--- Z+	Directional-Tap  : (1) Back \n",__FUNCTION__,__LINE__); 	
			break;
		case INT_CTRL_REG3_TFDM : // Z-
			   direction = ACCEL_TAP_FRONT;

			if(tap_mode == 2)		printk("[%s:%d] ---I--- Z-	Directional-Tap  : (2) Front \n",__FUNCTION__,__LINE__);
			else				printk("[%s:%d] ---I--- Z-	Directional-Tap  : (1) Front \n",__FUNCTION__,__LINE__); 
			break;
		case INT_CTRL_REG3_TUPM : // Y+
			direction = ACCEL_TAP_LEFT;

			if(tap_mode == 2)		printk("[%s:%d] ---I--- Y+	Directional-Tap  : (2) Left \n",__FUNCTION__,__LINE__); 
			else					printk("[%s:%d] ---I--- Y+	Directional-Tap  : (1) Left \n",__FUNCTION__,__LINE__);
			break;
		case INT_CTRL_REG3_TDOM : // Y-
			direction = ACCEL_TAP_RIGHT;

			if(tap_mode == 2)		printk("[%s:%d] ---I--- Y-	Directional-Tap  : (2) Right \n",__FUNCTION__,__LINE__);  
			else					printk("[%s:%d] ---I--- Y-	Directional-Tap  : (1) Right  \n",__FUNCTION__,__LINE__);  
			break;
		case INT_CTRL_REG3_TRIM : // X+
			direction = ACCEL_TAP_UP;

			if(tap_mode == 2)		printk("[%s:%d] ---I--- X+	Directional-Tap  : (2) Up \n",__FUNCTION__,__LINE__);  
			else					printk("[%s:%d] ---I--- X+	Directional-Tap  : (1) Up \n",__FUNCTION__,__LINE__);  
			break;
		case INT_CTRL_REG3_TLEM : // X-
			direction = ACCEL_TAP_DOWN;

			if(tap_mode == 2)		printk("[%s:%d] ---I--- X-	Directional-Tap  : (2) Down \n",__FUNCTION__,__LINE__);  
			else					printk("[%s:%d] ---I--- X-	Directional-Tap  : (1) Down \n",__FUNCTION__,__LINE__);  
			break;
	}
#elif defined(STAR_COUNTRY_KR) && defined(STAR_OPERATOR_SKT)
	switch(tap_direction)
	{
		case INT_CTRL_REG3_TFUM : // Z+
			direction = ACCEL_TAP_FRONT;//ACCEL_TAP_BACK;

			if(tap_mode == 2)		printk("[%s:%d] ---I--- Z+	Directional-Tap  : (2) Back \n",__FUNCTION__,__LINE__);  
			else					printk("[%s:%d] ---I--- Z+	Directional-Tap  : (1) Back \n",__FUNCTION__,__LINE__); 	
			break;
		case INT_CTRL_REG3_TFDM : // Z-
			   direction = ACCEL_TAP_BACK;//ACCEL_TAP_FRONT;

			if(tap_mode == 2)		printk("[%s:%d] ---I--- Z-	Directional-Tap  : (2) Front \n",__FUNCTION__,__LINE__);
			else				printk("[%s:%d] ---I--- Z-	Directional-Tap  : (1) Front \n",__FUNCTION__,__LINE__); 
			break;
		case INT_CTRL_REG3_TUPM : // Y+
			direction = ACCEL_TAP_RIGHT;//ACCEL_TAP_LEFT;

			if(tap_mode == 2)		printk("[%s:%d] ---I--- Y+	Directional-Tap  : (2) Left \n",__FUNCTION__,__LINE__); 
			else					printk("[%s:%d] ---I--- Y+	Directional-Tap  : (1) Left \n",__FUNCTION__,__LINE__);
			break;
		case INT_CTRL_REG3_TDOM : // Y-
			direction = ACCEL_TAP_LEFT;//ACCEL_TAP_RIGHT;

			if(tap_mode == 2)		printk("[%s:%d] ---I--- Y-	Directional-Tap  : (2) Right \n",__FUNCTION__,__LINE__);  
			else					printk("[%s:%d] ---I--- Y-	Directional-Tap  : (1) Right  \n",__FUNCTION__,__LINE__);  
			break;
		case INT_CTRL_REG3_TRIM : // X+
			direction = ACCEL_TAP_UP;

			if(tap_mode == 2)		printk("[%s:%d] ---I--- X+	Directional-Tap  : (2) Up \n",__FUNCTION__,__LINE__);  
			else					printk("[%s:%d] ---I--- X+	Directional-Tap  : (1) Up \n",__FUNCTION__,__LINE__);  
			break;
		case INT_CTRL_REG3_TLEM : // X-
			direction = ACCEL_TAP_DOWN;

			if(tap_mode == 2)		printk("[%s:%d] ---I--- X-	Directional-Tap  : (2) Down \n",__FUNCTION__,__LINE__);  
			else					printk("[%s:%d] ---I--- X-	Directional-Tap  : (1) Down \n",__FUNCTION__,__LINE__);  
			break;
	}
#elif defined(STAR_COUNTRY_US) && defined(STAR_OPERATOR_TMO)
	switch(tap_direction)
	{
		case INT_CTRL_REG3_TFUM : // Z+
			direction = ACCEL_TAP_BACK;

			if(tap_mode == 2)		printk("[%s:%d] ---I--- Z+	Directional-Tap  : (2) Back \n",__FUNCTION__,__LINE__);  
			else					printk("[%s:%d] ---I--- Z+	Directional-Tap  : (1) Back \n",__FUNCTION__,__LINE__); 	
			break;
		case INT_CTRL_REG3_TFDM : // Z-
			   direction = ACCEL_TAP_FRONT;

			if(tap_mode == 2)		printk("[%s:%d] ---I--- Z-	Directional-Tap  : (2) Front \n",__FUNCTION__,__LINE__);
			else				printk("[%s:%d] ---I--- Z-	Directional-Tap  : (1) Front \n",__FUNCTION__,__LINE__); 
			break;
		case INT_CTRL_REG3_TUPM : // Y+
			direction = ACCEL_TAP_LEFT;

			if(tap_mode == 2)		printk("[%s:%d] ---I--- Y+	Directional-Tap  : (2) Left \n",__FUNCTION__,__LINE__); 
			else					printk("[%s:%d] ---I--- Y+	Directional-Tap  : (1) Left \n",__FUNCTION__,__LINE__);
			break;
		case INT_CTRL_REG3_TDOM : // Y-
			direction = ACCEL_TAP_RIGHT;

			if(tap_mode == 2)		printk("[%s:%d] ---I--- Y-	Directional-Tap  : (2) Right \n",__FUNCTION__,__LINE__);  
			else					printk("[%s:%d] ---I--- Y-	Directional-Tap  : (1) Right  \n",__FUNCTION__,__LINE__);  
			break;
		case INT_CTRL_REG3_TRIM : // X+
			direction = ACCEL_TAP_UP;

			if(tap_mode == 2)		printk("[%s:%d] ---I--- X+	Directional-Tap  : (2) Up \n",__FUNCTION__,__LINE__);  
			else					printk("[%s:%d] ---I--- X+	Directional-Tap  : (1) Up \n",__FUNCTION__,__LINE__);  
			break;
		case INT_CTRL_REG3_TLEM : // X-
			direction = ACCEL_TAP_DOWN;

			if(tap_mode == 2)		printk("[%s:%d] ---I--- X-	Directional-Tap  : (2) Down \n",__FUNCTION__,__LINE__);  
			else					printk("[%s:%d] ---I--- X-	Directional-Tap  : (1) Down \n",__FUNCTION__,__LINE__);  
			break;
	}
#else
	switch(tap_direction)
	{
		case INT_CTRL_REG3_TFUM : // Z+
			direction = ACCEL_TAP_BACK;

			if(tap_mode == 2)		printk("[%s:%d] ---I--- Z+	Directional-Tap  : (2) Back \n",__FUNCTION__,__LINE__);  
			else					printk("[%s:%d] ---I--- Z+	Directional-Tap  : (1) Back \n",__FUNCTION__,__LINE__); 	
			break;
		case INT_CTRL_REG3_TFDM : // Z-
			   direction = ACCEL_TAP_FRONT;

			if(tap_mode == 2)		printk("[%s:%d] ---I--- Z-	Directional-Tap  : (2) Front \n",__FUNCTION__,__LINE__);
			else				printk("[%s:%d] ---I--- Z-	Directional-Tap  : (1) Front \n",__FUNCTION__,__LINE__); 
			break;
		case INT_CTRL_REG3_TUPM : // Y+
			direction = ACCEL_TAP_LEFT;

			if(tap_mode == 2)		printk("[%s:%d] ---I--- Y+	Directional-Tap  : (2) Left \n",__FUNCTION__,__LINE__); 
			else					printk("[%s:%d] ---I--- Y+	Directional-Tap  : (1) Left \n",__FUNCTION__,__LINE__);
			break;
		case INT_CTRL_REG3_TDOM : // Y-
			direction = ACCEL_TAP_RIGHT;

			if(tap_mode == 2)		printk("[%s:%d] ---I--- Y-	Directional-Tap  : (2) Right \n",__FUNCTION__,__LINE__);  
			else					printk("[%s:%d] ---I--- Y-	Directional-Tap  : (1) Right  \n",__FUNCTION__,__LINE__);  
			break;
		case INT_CTRL_REG3_TRIM : // X+
			direction = ACCEL_TAP_UP;

			if(tap_mode == 2)		printk("[%s:%d] ---I--- X+	Directional-Tap  : (2) Up \n",__FUNCTION__,__LINE__);  
			else					printk("[%s:%d] ---I--- X+	Directional-Tap  : (1) Up \n",__FUNCTION__,__LINE__);  
			break;
		case INT_CTRL_REG3_TLEM : // X-
			direction = ACCEL_TAP_DOWN;

			if(tap_mode == 2)		printk("[%s:%d] ---I--- X-	Directional-Tap  : (2) Down \n",__FUNCTION__,__LINE__);  
			else					printk("[%s:%d] ---I--- X-	Directional-Tap  : (1) Down \n",__FUNCTION__,__LINE__);  
			break;
	}
#endif

	motion_send_tap_detection(type, direction);

	return ;
	
}

void star_accel_process_screen_rotation(unsigned char tilt_pos_pre, unsigned char tilt_pos_cur)
{
	int flip_data=0;

	switch(tilt_pos_cur)
	{
	case CTRL_REG2_RIM :  // X+
		printk("[%s:%d] ---I--- X+  Screen Roation : Landscape LEFT %x\n",__FUNCTION__,__LINE__,tilt_pos_cur);
		break;
	case CTRL_REG2_LEM :  // X-
		printk("[%s:%d] ---I--- X-  Screen Roation : Landscape RIGHT %x\n",__FUNCTION__,__LINE__,tilt_pos_cur);
		break;
	case CTRL_REG2_UPM : // Y+    
		printk("[%s:%d] ---I--- Y+  Screen Roation : Potrait UP %x\n",__FUNCTION__,__LINE__,tilt_pos_cur);
		break;
	case CTRL_REG2_DOM : // Y-  // ACCEL_SNAP_ROLL_RIGHT
		printk("[%s:%d] ---I--- Y-  Screen Roation : Potrait DOWN %x\n",__FUNCTION__,__LINE__,tilt_pos_cur);
		break;
	case CTRL_REG2_FUM : // Z+
		printk("[%s:%d] ---I--- Z+  Screen Roation : Face Up %x\n",__FUNCTION__,__LINE__,tilt_pos_cur);
		break;
	case CTRL_REG2_FDM : // Z- // ACCEL_SNAP_PITCH_DOWN
		printk("[%s:%d] ---I--- Z-  Screen Roation : Face Down %x\n",__FUNCTION__,__LINE__,tilt_pos_cur);
		break;
	}

	//if((tilt_pos_cur == 0x01) || (tilt_pos_cur == 0x02))
	{
		if(call_once == 1)
		{
			flip_pre_pre_state = tilt_pos_cur;
			call_once = 0;
		}

		if(flip_pre_pre_state != tilt_pos_cur)
		{
		       if(tilt_pos_cur == 0x01)
		       {
				#if defined(STAR_COUNTRY_COM) && defined(STAR_OPERATOR_OPEN)
					flip_data = ACCEL_FLIP_DOWNSIDE_UP;
				#elif defined(STAR_COUNTRY_KR) && defined(STAR_OPERATOR_SKT)
					flip_data= ACCEL_FLIP_UPSIDE_DOWN;
				#elif defined(STAR_COUNTRY_US) && defined(STAR_OPERATOR_TMO)
					flip_data = ACCEL_FLIP_DOWNSIDE_UP;
				#else
					flip_data = ACCEL_FLIP_DOWNSIDE_UP;
				#endif
		       }
		       else if(tilt_pos_cur == 0x02)
		       {
				#if defined(STAR_COUNTRY_COM) && defined(STAR_OPERATOR_OPEN)
					flip_data= ACCEL_FLIP_UPSIDE_DOWN;
				#elif defined(STAR_COUNTRY_KR) && defined(STAR_OPERATOR_SKT)
					flip_data = ACCEL_FLIP_DOWNSIDE_UP;
				#elif defined(STAR_COUNTRY_US) && defined(STAR_OPERATOR_TMO)
					flip_data= ACCEL_FLIP_UPSIDE_DOWN;
				#else
					flip_data= ACCEL_FLIP_UPSIDE_DOWN;
				#endif	        
		       }
		       if((flip_data== ACCEL_FLIP_UPSIDE_DOWN)||(flip_data==ACCEL_FLIP_DOWNSIDE_UP))
		       {
				motion_send_flip_detection(flip_data);
		       }
		}

		flip_pre_pre_state = tilt_pos_cur;
	}
	
}

static void star_accel_irq_handler(void *arg)
{
	
	//printk("%s() -- start\n\n", __func__);
	schedule_work(&g_accel->work); //

}

static void star_accel_work_function(struct work_struct *work)
{
	unsigned char val_shadow;
	unsigned char int_src_reg1, int_src_reg2;
	unsigned char tilt_pos_cur, tilt_pos_pre;	

	//printk("%s() -- start\n\n", __func__);

	//disable accelerometer interrupt first
	star_accel_i2c_read_data(g_accel, INT_CTRL1, &val_shadow, 1);
	val_shadow &= ~(0x20); //disable interrupt
	star_accel_i2c_write_data(g_accel, INT_CTRL1, &val_shadow, 1);	

	// read status register
	star_accel_i2c_read_data(g_accel, 0x18, &val_shadow, 1);

	//printk("KIONIX_ACCEL_I2C_STATUS_REG = %x \n",val_shadow);

	//check interrupt
	if(!(val_shadow & (1<<4)) ) {
		printk("%s() -- not gesture interrupt\n\n", __func__);
		goto RELEASE_INT;
	}

	star_accel_i2c_read_data(g_accel, INT_SRC_REG1, &int_src_reg1, 1);
	star_accel_i2c_read_data(g_accel, INT_STATUS_REG, &int_src_reg2, 1);

	//printk("KXTF9_I2C_INT_SRC_REG1 = %x \n",int_src_reg1);
	//printk("KXTF9_I2C_INT_SRC_REG2 = %x \n",int_src_reg2);

	if((int_src_reg2 & (0x3<<2)) && is_tap_enabled() ) { // Direction tap
		unsigned char tap_mode ;
		tap_mode = ((int_src_reg2&(0x3<<2))>>2);
				
		//   Processing Directional-Tap 
		star_accel_process_directional_tap(tap_mode, int_src_reg1);
		
	}

	if((int_src_reg2 & (1<<0)) && is_flip_enabled()) { // TPS : Screen Rotation
		star_accel_i2c_read_data(g_accel, 0x10, &tilt_pos_cur, 1);
		star_accel_i2c_read_data(g_accel, 0x11, &tilt_pos_pre, 1);

		//printk("KIONIX_ACCEL_I2C_TILT_POS_CUR = %x KIONIX_ACCEL_I2C_TILT_POS_PRE = %x \n",tilt_pos_cur,tilt_pos_pre); 

		star_accel_process_screen_rotation(tilt_pos_pre, tilt_pos_cur);
	}	


RELEASE_INT:
	//printk("%s() -- interrupt released\n\n", __func__);
	//interrupt released
	star_accel_i2c_read_data(g_accel, 0x1A, &val_shadow, 1);

       // enable accelerometer interrupt again
	star_accel_i2c_read_data(g_accel, INT_CTRL1, &val_shadow, 1);
	val_shadow |= 0x20; //enable interrupt
	star_accel_i2c_write_data(g_accel, INT_CTRL1, &val_shadow, 1);	


	NvOdmGpioInterruptDone(g_accel->h_accel_intr);
	return ;

}

static int __init star_accel_probe( struct platform_device *pdev )
{
	NvU32 I2cInstance = 0;
	const NvOdmPeripheralConnectivity *pcon;
	struct device* dev = &pdev->dev;
	int err;
	NvBool found_gpio=NV_FALSE, found_i2c=NV_FALSE;
	unsigned char ctr_reg[4];
	unsigned char ctr_val[4];
	unsigned char reg_val;

	int loop;
	#if STAR_ACCEL_DEBUG
		printk("[skhwang]%s\n", __func__);
	#endif

	g_accel = kzalloc(sizeof(*g_accel), GFP_KERNEL);
	if( g_accel == NULL )
	{
		err = -ENOMEM;
		printk("[skhwang][%s], Failed to alloc the memory\n", __func__);
		goto failtomemorydev;
	}

    transfer_data_read  = kzalloc(12, GFP_KERNEL);
    transfer_data_write = kzalloc(12, GFP_KERNEL);
    if (transfer_data_read == NULL || transfer_data_write == NULL) {
		err = -ENOMEM;
		printk("[skhwang][%s], Failed to alloc the memory\n", __func__);
		goto failtomemorydev;
    }

    // 20100702  Power control bug fix [START]
	/*g_accel->h_accel_pmu = NvOdmServicesPmuOpen();
	if( !g_accel->h_accel_pmu )
	{err=-ENOSYS; goto failtomemorydev;}*/
    // 20100702  Power control bug fix [START]

	pcon = (NvOdmPeripheralConnectivity*)NvOdmPeripheralGetGuid(NV_ODM_GUID('a','c','c','e','l','e','r','o'));
	//pcon = (NvOdmPeripheralConnectivity*)NvOdmPeripheralGetGuid(NV_ODM_GUID('p','r','o','x','i','m','i','t'));

	for(loop = 0; loop< pcon->NumAddress; loop++)
	{
		switch(pcon->AddressList[loop].Interface)
		{
			case NvOdmIoModule_I2c:
				g_accel->i2c_address = (pcon->AddressList[loop].Address<<1);
				I2cInstance = pcon->AddressList[loop].Instance;
				found_i2c = NV_TRUE;
				break;
			case NvOdmIoModule_Gpio:
				g_accel->intr_port = pcon->AddressList[loop].Instance;
				g_accel->intr_pin = pcon->AddressList[loop].Address;
				found_gpio = NV_TRUE;
				break;
			case NvOdmIoModule_Vdd:
				g_accel->vdd_id = pcon->AddressList[loop].Address;
				#if STAR_ACCEL_DEBUG
					printk("[skhwang] KXTF9 POWER %d\n", g_accel->vdd_id );
				#endif
                // 20100702  Power control bug fix
				star_accel_set_power_rail(g_accel->vdd_id, NV_TRUE);
				NvOdmOsWaitUS(30);
				break;
			default:
				break;
		}
	}
	#if STAR_ACCEL_DEBUG
		printk("[skhwang][%s] : I2c Address = %#x, Int Port = %c, Int Pin = %d\n", __func__,
			g_accel->i2c_address, (g_accel->intr_port+'a'), g_accel->intr_pin);
	#endif
	if( found_i2c != NV_TRUE || found_gpio != NV_TRUE )
	{printk("[skhwang][%s] : I2c or Gpio not found...\n",__func__); err = -ENOMEM; goto failtomemorydev;}

	g_accel->h_accel_gpio = NvOdmGpioOpen();
	if(!g_accel->h_accel_gpio )
	{
		printk("[skhwang][%s] : Failed to open gpio\n",__func__);
		err = - ENOSYS;
		goto err_open_gpio;
	}

	g_accel->h_accel_gpio_pin = NvOdmGpioAcquirePinHandle( g_accel->h_accel_gpio, g_accel->intr_port, g_accel->intr_pin );
	if(!g_accel->h_accel_gpio_pin)
	{
		printk("[skhwang][%s] : Failed to acquire the pin handle\n",__func__);
		err = -ENOSYS;
		goto err_acquire_pin;
	}
	
	#if STAR_ACCEL_DEBUG
		printk("[skhwang]Hu~~~ accel initialization end!!\n");
	#endif

	INIT_WORK(&g_accel->work, star_accel_work_function);

	g_accel->h_gen2_i2c = NvOdmI2cPinMuxOpen(NvOdmIoModule_I2c, 1, NvOdmI2cPinMap_Config2 );
	if( !g_accel->h_gen2_i2c )
	{
		printk("[skhwang][%s] : failed to open I2c\n", __func__ );
		err = -ENOSYS;
		goto err_open_i2c;
	}	

	NvOdmGpioConfig(g_accel->h_accel_gpio, g_accel->h_accel_gpio_pin, NvOdmGpioPinMode_InputData);

	/* device interrupt registration */
	g_accel->use_irq = NvOdmGpioInterruptRegister(g_accel->h_accel_gpio, &g_accel->h_accel_intr, g_accel->h_accel_gpio_pin, NvOdmGpioPinMode_InputInterruptRisingEdge, star_accel_irq_handler, (void*)g_accel, 0);

	g_accel->input_dev = input_allocate_device();
	if(!g_accel->input_dev)
	{
		printk("[skhwang][%s] fail to allocate a input device\n", __func__ );
		err = -ENOMEM;
		goto err_input_alloc_dev;
	}

	/*---------------------------------------------------------------------------
	  init. sysfs
	  ---------------------------------------------------------------------------*/
	if ((err = sysfs_create_group(&dev->kobj, &star_motion_group))) {
#if DEBUG
		lprintk("[motion_sensor] sysfs_create_group FAIL \n");
#endif
		goto err_sysfs_create;
	}

	g_accel->input_dev->name = "nvodm_accelerometer";
	err = input_register_device(g_accel->input_dev);
	if(err)
	{
		printk("[%s] error to register input device\n",__func__);	
		goto input_register_device_failed;
	}

	//for ioctl
	if( misc_register(&star_accel_misc_device))
	{
		printk("[star accel, KXTF9] failed to register misc device\n");	
		err = -ENOSYS;
		goto err_misc_register;
	}
	return 0;

err_sysfs_create:
	printk("##  sensor: star motion misc_device_register_failed\n");
err_misc_register:
input_register_device_failed:
	input_free_device(g_accel->input_dev);
err_input_alloc_dev:
err_open_i2c:
	NvOdmOsFree(g_accel->h_gen2_i2c);
err_acquire_pin:
	NvOdmOsFree(g_accel->h_accel_gpio_pin);
err_open_gpio:
	NvOdmOsFree(g_accel->h_accel_gpio);
failtomemorydev:
	g_accel = 0;
	return err;

}

static int star_accel_remove( struct platform_device *pdev )
{
	input_unregister_device(g_accel->input_dev);

    if (transfer_data_read != NULL) {
        kfree(transfer_data_read);
    }
    if (transfer_data_write != NULL) {
        kfree(transfer_data_write);
    }

	return 0;
}

extern int star_proxi_get_status(void);

int star_accel_suspend(struct platform_device *dev, pm_message_t state)
{
    if (!star_proxi_get_status())
        star_accel_set_power_rail(g_accel->vdd_id, NV_FALSE);

    return 0;
}

int star_accel_resume(struct platform_device *dev)
{
    if (!star_proxi_get_status())
        star_accel_set_power_rail(g_accel->vdd_id, NV_TRUE);

    return 0;
}

static struct platform_driver star_accel_driver = {
	.probe = star_accel_probe,
	.remove = star_accel_remove,
	.suspend = star_accel_suspend,
	.resume = star_accel_resume,
	.driver = {
		.name = "tegra_accelerometer",
	},
};

static int __init star_accel_init(void)
{
	return platform_driver_register(&star_accel_driver);
}

static void __exit star_accel_exit(void)
{
	platform_driver_unregister(&star_accel_driver);
}

module_init(star_accel_init);
module_exit(star_accel_exit);

MODULE_AUTHOR("");
MODULE_DESCRIPTION("driver of star accelerometer sensor");
MODULE_LICENSE("GPL");
