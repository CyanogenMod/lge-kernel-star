/**
	@brief		 HynixVGA -Hynix VGA CMOS sensor driver setting value
	@author		 calvin.hwang@lge.com
	@date		 2011.04.23
*/


#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <media/hi702_vga.h>
#include <mach/gpio-names.h>
#include <mach/hardware.h>
#include <linux/gpio.h>
#include <lge/bssq_cam_pmic.h>//#include "../../../bssq/bssq_cam_pmic.h"
#include <lge/lge_hw_rev.h> // 20110611 jinkwan.kim@lge.com HW RevD GPIO

DEFINE_MUTEX(hi702_camera_lock);

typedef struct ExposureValueTypeRec{
    int index;
    int range;
}ExposureValueType;

typedef struct FpsRangeTypeRec{
    int low;
    int high;
}FpsRangeType;

#define HI702_RESET  TEGRA_GPIO_PT2
#define HI702_PWRDN  TEGRA_GPIO_PD5
//20111006 calvin.hwang@lge.com Camsensor sync with X2 [S]

#define HI702_PROBE_ADDR    0x03
#define HI702_PROBE_DATA    0x00
//20111006 calvin.hwang@lge.com Camsensor sync with X2 [E]

//younjung.park@lge.com 20110629 VGA parsing
#if 1//def CONFIG_VTCAM_SD_IMAGE_TUNNING
#include <media/register_common_init.h>
#endif



extern i2c_cam_reg8 hi702_init_176x144[];
extern i2c_cam_reg8 hi702_init_320x240[];
extern i2c_cam_reg8 hi702_init_352x288[];
extern i2c_cam_reg8 hi702_init_640x480[];
//extern i2c_cam_reg8 hi702_init_1280x720[];
extern i2c_cam_reg8 hi702_init_640x480_30fps[];
//extern i2c_cam_reg8 hi702_init_1280x720_30fps[];
extern i2c_cam_reg8 hi702_Preview[];
extern i2c_cam_reg8 hi702_Capture[];

extern i2c_cam_reg8 hi702_Color_Effect_Normal[];
extern i2c_cam_reg8 hi702_Color_Effect_MONO[];
extern i2c_cam_reg8 hi702_Color_Effect_NEGATIVE[];
extern i2c_cam_reg8 hi702_Color_Effect_SEPIA[];
extern i2c_cam_reg8 hi702_Color_Effect_Aqua[];
extern i2c_cam_reg8 hi702_Color_Effect_Sketch[];

extern i2c_cam_reg8 hi702_MWB_Auto[];
extern i2c_cam_reg8 hi702_MWB_Incandescent[];
extern i2c_cam_reg8 hi702_MWB_Fluorescent[];
extern i2c_cam_reg8 hi702_MWB_Daylight[];
extern i2c_cam_reg8 hi702_MWB_CloudyDaylight[];
extern i2c_cam_reg8 hi702_Framerate_7Fps[];
extern i2c_cam_reg8 hi702_Framerate_10Fps[];
extern i2c_cam_reg8 hi702_Framerate_15Fps[];
extern i2c_cam_reg8 hi702_Framerate_VarFps[];

// LGE_CHANGE_S X2_ICS [byun.youngki@lge.com], 2012-05-05, < add the scene mode for front camera  >
extern i2c_cam_reg8 hi702_ScenMode_Night[];
extern i2c_cam_reg8 hi702_ScenMode_Auto[];
// LGE_CHANGE_E X2_ICS [byun.youngki@lge.com], 2012-05-05, < add the scene mode for front camera  >

static struct cam_yuv_info *info;

static i2c_cam_reg8 *hi702_SetModeSequenceList[] =
{
	hi702_init_176x144,
	hi702_init_320x240,
	hi702_init_352x288,
	hi702_init_640x480,
	hi702_Capture
};

static i2c_cam_reg8 *hi702_Set_FramerateList[] =
{
	hi702_Framerate_7Fps,
	hi702_Framerate_10Fps,	
	hi702_Framerate_15Fps,
	hi702_Framerate_VarFps

};

#if 0
static int hi702_read_reg(struct i2c_client *client, u8 addr, u8 *val)
{
	int err = 0;
	struct i2c_msg msg;
	unsigned char data;

	memset (data, 0, sizeof(unsigned char));
	if (!client->adapter)
		return -ENODEV;

	data = (u8 *)val;

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = 1;
	msg.buf = &data;
	
	err = i2c_transfer(client->adapter, msg, 2);

	*val = data[4];

	return err ;
}
#endif
static int hi702_write_reg(struct i2c_client *client, u8 addr, u8 val)
{
	int err = 0;
	struct i2c_msg msg;
	unsigned char data[2];
	int retry = 0;

	memset (data, 0, 2*sizeof(unsigned char));

	if (!client->adapter)
		return -ENODEV;

    data[0] = (u8) (addr & 0xff);
    data[1] = (u8) (val & 0xff);

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = 2;
	msg.buf = data;

//s32 i2c_write_block_data(client, u8 addr_flags, 1, addr, &val)
	do {
		//pr_info("hi702: I2C Transfer start Addr = %x, val=%x\n", addr,val);	//20110525 calvin.hwang@lge.com Camsensor ks1001 merge	
		err = i2c_transfer(client->adapter, &msg, 1);
		//pr_info("hi702: I2C Transfer end Addr = %x, val=%x\n", addr,val);		//20110525 calvin.hwang@lge.com Camsensor ks1001 merge
		if (err >0 )
			return 0;
		retry++;
		pr_err("hi702: i2c transfer failed, retrying Add: 0x%x Val: 0x%x\n",
			addr, val);
		msleep(3);
	} while (retry <5);

	return err;
}


static int hi702_write_table(struct i2c_client *client,
			const i2c_cam_reg8 table[],
			const i2c_cam_reg8 override_list[],
			int num_override_regs)
{
	int err = 0;
	const i2c_cam_reg8 *next;
	int i;
	u16 val;
	printk("hi702_write_table start\n");

	for (next = table; next->addr != SEQUENCE_END; next++) {
		if (next->addr == SEQUENCE_WAIT_MS) {
			printk("hi702_delay_time: %x\n",next->val);
			msleep(next->val);
			continue;
		}

		val = next->val;

		/* When an override list is passed in, replace the reg */
		/* value to write if the reg is in the list            */
		if (override_list) {
			for (i = 0; i < num_override_regs; i++) {
				if (next->addr == override_list[i].addr) {
					val = override_list[i].val;
					break;
				}
			}
		}

		err = hi702_write_reg(client, next->addr, val);
		if (err !=0)
			return err;
	}
	return err;
}

static int hi702_set_mode(struct cam_yuv_info *info, struct cam_yuv_mode *mode)
{
	int err = 0, set_index, i;
	u16 val;

	set_index = mode->index;
	pr_info("%s : index: %d\n", __func__,set_index);

	#if 1//younjung.park@lge.com 20110629 VGA parsing
		#if 1//def CONFIG_VTCAM_SD_IMAGE_TUNNING
		common_reg_list_type* pstRegisterList = NULL, *ptr_list;
	//	int loop;

		printk(KERN_ERR "### %s: Register init\n", __func__);
		common_register_init(COMMON_REG_REG, &pstRegisterList); //here, open file and parse params. younjung.park@lge.com 20110629
		if (!pstRegisterList)
		{
			printk("&&&&&& 1 &&&&&&&&\n");
			err =  hi702_write_table(info->i2c_client,hi702_SetModeSequenceList[set_index],NULL,0);//hi702_setting_value.cÀÇ ³»¿ë :hi702_SetModeSequenceList
			if (err !=0)
				pr_info("%s : error : %d\n", __func__,err);

			return err;			
		}
		else{
				ptr_list = pstRegisterList;			
			    printk("&&&&&& 2 &&&&&&&&\n");
				
				for (i = 0;i< ptr_list->num_regs;i++) {
					if (ptr_list->list_regs[i].reg.addr== SEQUENCE_WAIT_MS) {
						printk("state2:hi702_delay_time: %x\n",ptr_list->list_regs[i].reg.val);
						msleep(ptr_list->list_regs[i].reg.val);
						continue;
					}
					else{ 

					err = hi702_write_reg(info->i2c_client, ptr_list->list_regs[i].reg.addr, ptr_list->list_regs[i].reg.val);

					if (err !=0)
						break;
					}
			
				}
			}
				return err;
				if (pstRegisterList)
				kfree(pstRegisterList);
		#endif
		
	#endif//younjung.park@lge.com VGA parsing[E]}
}
static int hi702_set_color_effect(struct cam_yuv_info *info, unsigned int color_effect)
{
	int err = 0;
	pr_info("%s : %d\n", __func__, color_effect);

	switch(color_effect)
	{
		case YUVCamColorEffect_None :
			err = hi702_write_table(info->i2c_client, hi702_Color_Effect_Normal,NULL,0);
			break;

		case YUVCamColorEffect_Negative :
			err = hi702_write_table(info->i2c_client, hi702_Color_Effect_NEGATIVE,NULL,0);

			break;

		case YUVCamColorEffect_Aqua :
			err = hi702_write_table(info->i2c_client, hi702_Color_Effect_Aqua,NULL,0);

			break;

		case YUVCamColorEffect_Posterize :
			err = hi702_write_table(info->i2c_client, hi702_Color_Effect_Sketch,NULL,0);

			break;

		case YUVCamColorEffect_Sepia :
			err = hi702_write_table(info->i2c_client, hi702_Color_Effect_SEPIA,NULL,0);

			break;

		case YUVCamColorEffect_Mono :
			err = hi702_write_table(info->i2c_client, hi702_Color_Effect_MONO,NULL,0);

			break;

		default :
			//err = hi702_write_table(info->i2c_client, hi702_Color_Effect_Normal,NULL,0);

			break;
	}
	if (err !=0)
		pr_info("%s : Color Effect : %d,  error : %d\n", __func__, color_effect, err);

	return err;
}

static int hi702_set_white_balance(struct cam_yuv_info *info, unsigned int wb_mode)
{
	int err = 0;
	pr_info("%s : %d\n", __func__, wb_mode);
	switch(wb_mode)
	{
		case YUVCamWhitebalance_Auto :
			err = hi702_write_table(info->i2c_client, hi702_MWB_Auto,NULL,0);
			break;

		case YUVCamWhitebalance_Incandescent :
			err = hi702_write_table(info->i2c_client, hi702_MWB_Incandescent,NULL,0);

			break;

		case YUVCamWhitebalance_SunLight :
			err = hi702_write_table(info->i2c_client, hi702_MWB_Daylight,NULL,0);

			break;
		case YUVCamWhitebalance_CloudyDayLight:
			err = hi702_write_table(info->i2c_client, hi702_MWB_CloudyDaylight,NULL,0);
			
			break;

		case YUVCamWhitebalance_Fluorescent :
			err = hi702_write_table(info->i2c_client, hi702_MWB_Fluorescent,NULL,0);

			break;

		default :
			//err = hi702_write_table(info->i2c_client, hi702_MWB_Auto,NULL,0);

			break;
	}
	if (err !=0)
		pr_info("%s : White Balance : %d,  error : %d\n", __func__, wb_mode, err);
	return err;
}

static int hi702_set_exposure(struct cam_yuv_info *info, ExposureValueType *exposure)
{
	int err = 0;
	s8 value;

	if(exposure == NULL || exposure->range == 0)
	{
		pr_info("%s : invalid pointer or range value\n", __func__);
	    return -1;
	}

	value = (s8)((exposure->index * 256) / exposure->range);
	if(value < 0)
	{
	    value = (value ^ 0x7f) | 0x01;
	}
	pr_info(" %s : exp(%d) value(%d) range(%d)\n", __func__, exposure->index, value, exposure->range);
	hi702_write_reg(info->i2c_client, 0x03, 0x10); // page
	hi702_write_reg(info->i2c_client, 0x40, (u8)value); // brightness
	return err;
}

//20110708 younjung.park@lge.com framerate
static int hi702_set_fpsrange(struct cam_yuv_info *info, FpsRangeType *fpsRange)
{
	int err = 0;

	int lo = fpsRange->low;
	int hi = fpsRange->high;
	pr_info("%s:lo=%d, hi=%d\n", __func__, lo, hi);		
	if (lo == hi){//fixed framerate
		if (lo==7){
			err =  hi702_write_table(info->i2c_client,hi702_Set_FramerateList[0],NULL,0);		
		}
		else if (lo == 10){
			err =  hi702_write_table(info->i2c_client,hi702_Set_FramerateList[1],NULL,0);	
		}
		else if (lo == 15){
			err =  hi702_write_table(info->i2c_client,hi702_Set_FramerateList[2],NULL,0);	
		}
	
	}
	else{//variable framerate
		err = hi702_write_table(info->i2c_client,hi702_Set_FramerateList[3],NULL,0);
	}

	return err;
}
//20111006 calvin.hwang@lge.com Camsensor sync with X2 [S]

static int hi702_power_on(struct file *file)
{
	int ret = 0;
	struct cam_yuv_info *info = file->private_data;

	ret = star_cam_VT_power_on();
	if(ret < 0)
	{
		pr_info("%s: ldo or sensor power on failure \n", __func__);
		return -EINVAL;
	}
	
    mdelay(5);
	gpio_set_value(HI702_PWRDN,1);
	mdelay(10);
#if defined(CONFIG_LU6500) || defined(CONFIG_KS1001)
	if (REV_E > get_lge_pcb_revision())
	{
		gpio_set_value(TEGRA_GPIO_PBB1,1);
	}
	else
#endif
	{
		gpio_set_value(HI702_RESET,1);
	}
	mdelay(10);

	ret = hi702_write_reg(info->i2c_client, HI702_PROBE_ADDR, HI702_PROBE_DATA);
	if(ret < 0)
	{
		pr_info("%s: sensor probe failure \n", __func__);
	}
	else
	{
		pr_info("%s: sensor probe succeeded \n", __func__);
	}

	return ret;
}

static void hi702_power_off(void)
{
	gpio_set_value(HI702_PWRDN,0);
	mdelay(5);
#if defined(CONFIG_LU6500) ||defined(CONFIG_KS1001)
	if (REV_E > get_lge_pcb_revision())
	{
		gpio_set_value(TEGRA_GPIO_PBB1,0);
	}else
#endif
	{
		gpio_set_value(HI702_RESET,0);
	}
	mdelay(5);	
	star_cam_power_off();
}
//20111006 calvin.hwang@lge.com Camsensor sync with X2 [E]


// LGE_CHANGE_S X2_ICS [byun.youngki@lge.com], 2012-05-05, < add the scene mode for front camera  >
static int hi702_set_scene_mode(struct cam_yuv_info *info, unsigned int scene_mode)
{
	int err = 0;
	pr_info("%s : %d\n", __func__, scene_mode);

	switch(scene_mode)
	{
		case YUVCamSceneMode_Auto :
			err = hi702_write_table(info->i2c_client, hi702_ScenMode_Auto,NULL,0);
			break;

		case YUVCamSceneMode_Night :
			err = hi702_write_table(info->i2c_client, hi702_ScenMode_Night,NULL,0);
			break;

		default :
			//err = hi702_write_table(info->i2c_client, hi702_ScenMode_Auto,NULL,0);

			break;
	}
	if (err !=0)
		pr_info("%s : Scene Mode : %d,  error : %d\n", __func__, scene_mode, err);

	return err;
}
// LGE_CHANGE_E X2_ICS [byun.youngki@lge.com], 2012-05-05, < add the scene mode for front camera  >

static long hi702_ioctl(struct file *file,
			unsigned int cmd, unsigned long arg)
{
	struct cam_yuv_info *info = file->private_data;
	int err;

	pr_info("%s, cmd : %d\n", __func__, cmd);

	switch (cmd) {
//20111006 calvin.hwang@lge.com Camsensor sync with X2 [s]

	case HI702_IOCTL_SET_POWER_ON:
	{
		pr_info("    :HI702_IOCTL_SET_POWER_ON\n");
		return hi702_power_on(file);
	}
//20111006 calvin.hwang@lge.com Camsensor sync with X2 [E]

	case HI702_IOCTL_SET_MODE:
	{
		struct cam_yuv_mode mode;

		if (copy_from_user(&mode,
					(const void __user *)arg,
					sizeof(struct cam_yuv_mode))) {
			return -EFAULT;
		}
		mutex_lock(&hi702_camera_lock);
		err = hi702_set_mode(info, &mode);
		mutex_unlock(&hi702_camera_lock);
		pr_info("    :HI702_IOCTL_SET_MODE(%dx%d:%d)\n", mode.xres, mode.yres, mode.index);
		return err;
	}
	case HI702_IOCTL_SET_COLOR_EFFECT :
	{
		unsigned int color_effect;

		if (copy_from_user(&color_effect,
					(const void __user *)arg,
					sizeof(unsigned int))) {
			return -EFAULT;
		}
		mutex_lock(&hi702_camera_lock);
		err = hi702_set_color_effect(info, color_effect);
		mutex_unlock(&hi702_camera_lock);
		pr_info("    :HI702_IOCTL_SET_COLOR_EFFECT(%d)\n", color_effect);
		return err; 

	}
	case HI702_IOCTL_SET_WHITE_BALANCE :
	{
		unsigned int white_balance;

		if (copy_from_user(&white_balance,
					(const void __user *)arg,
					sizeof(unsigned int))) {
			return -EFAULT;
		}
		mutex_lock(&hi702_camera_lock);	
        err = hi702_set_white_balance(info, white_balance);		
		mutex_unlock(&hi702_camera_lock);
		pr_info("    :HI702_IOCTL_SET_WHITE_BALANCE(%d)\n", white_balance);
		return err; 

	}

	case HI702_IOCTL_SET_EXPOSURE:
	{
		ExposureValueType exposure;

		if (copy_from_user(&exposure,
					(const void __user *)arg,
					sizeof(ExposureValueType))) {
			return -EFAULT;
		}
		mutex_lock(&hi702_camera_lock);
        err = hi702_set_exposure(info, &exposure);		
		mutex_unlock(&hi702_camera_lock);
		pr_info("    :HI702_IOCTL_SET_EXPOSURE(%d/%d)\n", exposure.index, exposure.range);
		return err; 

	}

	case HI702_IOCTL_SET_FPSRANGE:
	{
		FpsRangeType FpsRange;

		if (copy_from_user(&FpsRange,
					(const void __user *)arg,
					sizeof(FpsRangeType))) {
			return -EFAULT;
		}
		mutex_lock(&hi702_camera_lock);
        err = hi702_set_fpsrange(info, &FpsRange);		
		mutex_unlock(&hi702_camera_lock);
		pr_info("    :HI702_IOCTL_SET_FPSRANGE(%d:%d)\n", FpsRange.low, FpsRange.high);
		return err; 		
	}
//20111006 calvin.hwang@lge.com Camsensor sync with X2 [S]

	case HI702_IOCTL_SENSOR_RESET:
	{
		u8 status;
		pr_info("    :HI702_IOCTL_SENSOR_RESET(%d)\n", (int)arg);
		hi702_power_off();
		return 0;
	}
//20111006 calvin.hwang@lge.com Camsensor sync with X2 [E]

// LGE_CHANGE_S X2_ICS [byun.youngki@lge.com], 2012-05-05, < add the scene mode for front camera  >
    case HI702_IOCTL_SET_SCENE_MODE:
	{
		unsigned int scene_mode;

		if (copy_from_user(&scene_mode,
					(const void __user *)arg,
					sizeof(unsigned int))) {
			return -EFAULT;
		}
		mutex_lock(&hi702_camera_lock);
		err = hi702_set_scene_mode(info, scene_mode);
		mutex_unlock(&hi702_camera_lock);
		pr_info("    :HI702_IOCTL_SET_SCENE_MODE(%d)\n", scene_mode);
		return err; 	    
	}
// LGE_CHANGE_E X2_ICS [byun.youngki@lge.com], 2012-05-05, < add the scene mode for front camera  >
	default:
		return -EINVAL;
	}
	return 0;
}


static int hi702_open(struct inode *inode, struct file *file)
{
	int ret = 0;  //20111006 calvin.hwang@lge.com Camsensor sync with X2
	pr_info("%s\n", __func__);
	file->private_data = info;
	//pr_info("%s:----\n", __func__);

	return ret;  //20111006 calvin.hwang@lge.com Camsensor sync with X2
}

static int hi702_release(struct inode *inode, struct file *file)
{
	pr_info("%s\n", __func__);

	file->private_data = NULL;
	hi702_power_off();  //20111006 calvin.hwang@lge.com Camsensor sync with X2
	return 0;
}

static const struct file_operations hi702_fileops = {
	.owner = THIS_MODULE,
	.open = hi702_open,
	.unlocked_ioctl = hi702_ioctl,
	.release = hi702_release,
};

static struct miscdevice hi702_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "hi702",
	.fops = &hi702_fileops,
	.mode = S_IRWXUGO
};

/****************************************************************************
			I2C Client & Driver
 ****************************************************************************/

static int hi702_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	int err;
	pr_info("%s.(id:%s)\n", __func__, id->name);


	info = kzalloc(sizeof(struct cam_yuv_info), GFP_KERNEL);
	if (!info) {
		pr_err("hi702: Unable to allocate memory!\n");
		return -ENOMEM;
	}
	err = misc_register(&hi702_device);
	if (err) {
		pr_err("hi702: Unable to register misc device!\n");
		kfree(info);
		return err;
	}

	info->pdata = client->dev.platform_data;
	info->i2c_client = client;

	i2c_set_clientdata(client, info);
	#if defined(CONFIG_LU6500) ||defined(CONFIG_KS1001)
	if (REV_E > get_lge_pcb_revision()){
		gpio_request(TEGRA_GPIO_PBB1, "vga_reset");
		tegra_gpio_enable(TEGRA_GPIO_PBB1);
		gpio_direction_output(TEGRA_GPIO_PBB1,1);
		mdelay(5);
		gpio_set_value(TEGRA_GPIO_PBB1,0); 
	}else
	#endif
	{
		gpio_request(HI702_RESET, "vga_reset");
		tegra_gpio_enable(HI702_RESET);
		gpio_direction_output(HI702_RESET,1);
		mdelay(5);
		gpio_set_value(HI702_RESET,0); 
	}
	gpio_request(HI702_PWRDN, "vga_power_down");
	tegra_gpio_enable(HI702_PWRDN);
	gpio_direction_output(HI702_PWRDN,1);
    mdelay(5);
	gpio_set_value(HI702_PWRDN,0);	
	return 0;
}

static int hi702_remove(struct i2c_client *client)
{
	pr_info("%s\n", __func__);
	info = i2c_get_clientdata(client);
	misc_deregister(&hi702_device);
	kfree(info);
#if defined(CONFIG_LU6500) ||defined(CONFIG_KS1001)
	if (REV_E > get_lge_pcb_revision()){
		gpio_free(TEGRA_GPIO_PBB1);
	}else
#endif
	{
		gpio_free(HI702_RESET);
	}
	gpio_free(HI702_PWRDN);	
	return 0;
}

/* ----------------------------------------------------------------------- */

static const struct i2c_device_id hi702_id[] = {
	{ "hi702", 0 },
};
MODULE_DEVICE_TABLE(i2c, hi702_id);

static struct i2c_driver hi702_i2c_driver = {
	.probe = hi702_probe,
	.remove = hi702_remove,
	.id_table = hi702_id,
	.driver = {
		.name = "hi702",
		.owner = THIS_MODULE,
	},
};


static int __init hi702_init(void)
{
	pr_info("hi702 sensor driver loading\n");
	return i2c_add_driver(&hi702_i2c_driver);
}

static void __exit hi702_exit(void)
{
	pr_info("hi702 sensor driver exit\n");
	i2c_del_driver(&hi702_i2c_driver);
}

module_init(hi702_init);
module_exit(hi702_exit);

