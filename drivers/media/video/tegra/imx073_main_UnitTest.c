// LGE_CHANGE_S [sungmo.yang] 2011-01-17 [LGE_AP20], Camera unit test functions
// start imx073_UnitTest  // 815




enum CameraStatus_UnitTest{
	INIT_UnitTest = 0
};

static int 	CAMERA_STATUS_UNITTEST  	= INIT_UnitTest;
// UnitTest Command
enum Camera_CMD{
	SYSFS_TEST = 100,
	READ_REGISTER_SENSOR_MODEL_NUM,
	CAMERA_POWER_ON,
	CAMERA_INIT_MODE
};




static int testRegstierWrite(void)
{

	return true;	
}

static int testRegstierRead(void)
{
	u8 REG_OUT;
	char	reg;
	int	i2c_err;
	// 0x00
	i2c_err =imx073_read_reg((info->i2c_client), 0x0000, &REG_OUT);
	pr_info("imx073 Register Read result 0x0000 (imx): %d, err : %d\n ", REG_OUT, i2c_err );

	// 0x01
	imx073_read_reg((info->i2c_client), 0x0001, &REG_OUT);
	pr_info("imx073 Register Read result 0x0001 (imx): %d, err : %d\n ", REG_OUT, i2c_err );


	// 0x02
	imx073_read_reg((info->i2c_client), 0x0002, &REG_OUT);
	pr_info("imx073 Register Read result 0x0002 (imx):  %d, err : %d\n ", REG_OUT, i2c_err );	

	// 0x03
	imx073_read_reg((info->i2c_client), 0x0003, &REG_OUT);
	pr_info("imx073 Register Read result 0x0003 (imx):  %d, err : %d\n ", REG_OUT, i2c_err );


	// 0x04
	imx073_read_reg((info->i2c_client), 0x0004, &REG_OUT);
	pr_info("imx073 Register Read result 0x0004 (imx):  %d, err : %d\n ", REG_OUT, i2c_err );

	// 0x05
	imx073_read_reg((info->i2c_client), 0x0005, &REG_OUT);
	pr_info("imx073 Register Read result 0x0005 (imx):  %d, err : %d\n ", REG_OUT, i2c_err );
	
	// 0x06
	imx073_read_reg((info->i2c_client), 0x0006, &REG_OUT);
	pr_info("imx073 Register Read result 0x0006 (imx):  %d, err : %d\n ", REG_OUT, i2c_err );


	// 0x0306	
	imx073_read_reg((info->i2c_client), 0x0306, &REG_OUT);
	pr_info("imx073 Register Read result 0x0306 (imx): %d, err : %d\n ", REG_OUT, i2c_err );

//	{0x0307, 0x24},	// PLL multiplier
//	{0x302B, 0x38},	// PLL oscillation stable wait timer setting
//	{0x30E5, 0x04},	// Output Method Switching register [CSI2 Enable]
//	{0x3300, 0x00},	// Select subLVDS and Mipi exclusively [Mipi operation]
//	{0x0101, 0x03},	
//	{0x0202, 0x03},
//	{0x0203, 0xe8},


	return true;	
}


static ssize_t Imx073_show_cam(struct device *dev,
                struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "Camera Status UnitTest : %d \n", info->mode);
}


static ssize_t Imx073_store_cam(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t count)
{

	int unit_cmd	=	simple_strtoul(buf, NULL, 10);
	int ret;
	
	pr_info("imx073 Unit test Ver :  0.5.1  \n");
	pr_info("imx073 Unit test Command : %d \n", unit_cmd);
	
	switch (unit_cmd) {
		case SYSFS_TEST :
				pr_info("imx073 Unit Test works! \n");
			break;
		case READ_REGISTER_SENSOR_MODEL_NUM :
				pr_info("READ_REGISTER_SENSOR_MODEL_NUM \n");
				testRegstierRead();
			break;
		case CAMERA_POWER_ON :
				pr_info("CAMERA_POWER_ON \n");			
				ret = imx073_power_on();
			break;
		case CAMERA_INIT_MODE :
				pr_info("CAMERA_INIT_MODE \n");
				struct imx073_mode SET_imx073_MODE_1920x1080 ={1920, 1080, 0, 0, 0};
				imx073_set_mode(info, &SET_imx073_MODE_1920x1080);
			break;
		default :
				pr_info("Please input right command! \n");
			break;
	} // end of switch

	return count;
}


static DEVICE_ATTR(Imx073, 0666, Imx073_show_cam, Imx073_store_cam);


// LGE_CHANGE_E [sungmo.yang] 2011-01-17 [LGE_AP20], Camera unit test functions

