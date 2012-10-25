/****************************************************************************************************
*                                       BSSQ Devices & Resources
****************************************************************************************************/
#include <linux/mpu.h>
#include <linux/switch.h> 
#include <linux/gpio_keys.h>
#include <linux/lge_touch_synaptics.h>
//#include "bssq_proximity.h"
#include <linux/input/gp2a.h>
#include <linux/mfd/max8907c.h>
#include <linux/mfd/wm8994/pdata.h>
#ifdef CONFIG_BSSQ_TOUCH_LED
#include <linux/leds-bd2802.h>
#endif
#include <mach-tegra/gpio-names.h>
#ifdef CONFIG_BSSQ_VIBRATOR
#include <mach/vibrator.h>
#endif
#if defined(CONFIG_SUBPMIC_LP8720)
#include <lge/lp8720.h>
#endif
#if defined (CONFIG_BSSQ_CHARGER_RT)
#include "lge/bssq_charger_rt.h"
#endif
//20110525 calvin.hwang@lge.com Camsensor ks1001 merge [S]
#if defined(CONFIG_CAM_PMIC)
#include <lge/bssq_cam_pmic.h>
#endif
//20110525 calvin.hwang@lge.com Camsensor ks1001 merge [E]

#include "bssq_flash_led.h"
#include "lge_hw_rev.h"

// 20110524 bg80.song@lge.com AP Temp Sensor Bring-up [S]
#include <linux/nct1008.h>
// 20110524 bg80.song@lge.com AP Temp Sensor Bring-up [E]
// [START] kangsic.ham 2011.06.15 - GPS UART & GPIO Setting 
#if 0//defined(CONFIG_SU880) || defined(CONFIG_KU8800)
#include <mach/tegra_gps.h>
#endif
// [END] kangsic.ham 2011.06.15 - GPS UART & GPIO Setting 

// 20110819 woo.jung@lge.com AP - Compass HAL [S]
#if defined(CONFIG_KS1103)
#include <linux/akm8975.h>
#endif	
// 20110819 woo.jung@lge.com AP - Compass HAL [E]

#if defined(CONFIG_SPI)
#include <mach/spi.h>
#endif
extern int device_power_control(char* reg_id, bool on);

#if 0
/****************************************************************************************************
*                                       platform devices
****************************************************************************************************/
static struct usb_mass_storage_platform_data andums_plat = {
    .vendor = "LGE",
#if defined(CONFIG_KS1001) || defined(CONFIG_KS1103)
	.product = "Android Phone",
#else
    .product = "bssq",
#endif
    .release = 0x1,
    .nluns = 1,
};

static struct platform_device androidums_device = {
	.name   = "usb_mass_storage",
	.id     = -1,
	.dev    = {
		.platform_data  = &andums_plat,
	},
};
#endif //#if 0

//20110525 calvin.hwang@lge.com Camsensor ks1001 merge [S]
#if defined (CONFIG_KS1001) || defined (CONFIG_KS1103)
struct platform_device bssq_cam_pmic = {
	.name   = "bssq_cam_pmic",
	.id     = -1,
	.dev    = {
	},
};
#endif
//20110525 calvin.hwang@lge.com Camsensor ks1001 merge [E]

#if 0
#if defined(CONFIG_KS1001) || defined(CONFIG_KS1103) || defined (CONFIG_LU6500) || defined (CONFIG_SU880) || defined (CONFIG_KU8800) || defined (CONFIG_LU8800)
struct gpio_switch_platform_data bssq_headset_data = {
	.name = "h2w",
// 20110521 seki.park@lge.com add headset device [S]		
#if defined (CONFIG_LU6500) || defined (CONFIG_SU880) || defined (CONFIG_KU8800) || defined (CONFIG_LU8800)
    .gpio = TEGRA_GPIO_PE0,
#elif defined(CONFIG_KS1103)
	.gpio = TEGRA_GPIO_PB3,
#else
	 .gpio = TEGRA_GPIO_PG3,
#endif
// 20110521 seki.park@lge.com add headset device [E]	
};

struct platform_device tegra_headset_detect_device =
{
	.name = "bssq_headset",
	.id	= -1,
	.dev.platform_data = &bssq_headset_data,

};
#endif

// 20110409 hyeongwon.oh@lge.com add feulgauge device [S]
#if defined (CONFIG_BSSQ_CHARGER_RT)
struct charger_rt_platform_data bssq_charger_rt_data = {
	.gpio_en_set	= TEGRA_GPIO_PS1,
    .gpio_status	= TEGRA_GPIO_PS2,
    .gpio_pgb		= TEGRA_GPIO_PC1,
    .irqflags		= IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
};

struct platform_device bssq_charger_ic_device = {

   .name = "charger_ic_rt9524",
   .id   = -1,
   .dev.platform_data = &bssq_charger_rt_data,
};
#endif
// 20110409 hyeongwon.oh@lge.com add feulgauge device [E]

#endif //#if 0
#if defined (CONFIG_BSSQ_CHARGER_RT)
struct charger_rt_platform_data bssq_charger_rt_data = {
	.gpio_en_set	= TEGRA_GPIO_PS1,
    .gpio_status	= TEGRA_GPIO_PS2,
    .gpio_pgb		= TEGRA_GPIO_PC1,
    .irqflags		= IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
};

struct platform_device bssq_charger_ic_device = {

   .name = "charger_ic_rt9524",
   .id   = -1,
	   .dev    =   {
		   .platform_data  = &bssq_charger_rt_data,
	   },

};
#endif

#ifdef CONFIG_BSSQ_VIBRATOR

// 20110829 unyou.shim@lge.com Vibrator voltage : requested by HW Hwang Y [S]
#if defined(CONFIG_KS1001) || defined(CONFIG_KS1103)
static struct pwm_vib_platform_data	bssq_vib_platform_data = {
	.max_timeout		=	15000,
	.active_low			=	0,
	.initial_vibrate	=	0,
	.pwm_id				=	0,
	.period_ns			=	50000,
	.duty_ns			=	5500, // requested by HW Hwang Y 1250 -> 5500
	.enable				=	TEGRA_GPIO_PU4,
	.power				=	&device_power_control,
};
// 20110829 unyou.shim@lge.com Vibrator voltage : requested by HW Hwang Y [E]

#else
static struct pwm_vib_platform_data	bssq_vib_platform_data = {
	.max_timeout		=	15000,
	.active_low			=	0,
	.initial_vibrate	=	0,
	.pwm_id				=	0,
	.period_ns			=	50000,
	.duty_ns			=	1250,//5000, 20110601 seki.park@lge.com motor duty change
	.enable				=	TEGRA_GPIO_PU4,
	.power				=	&device_power_control,
};
#endif
static struct platform_device bssq_vib_device = {
	.name   =   "bssq_vib_name",
	.id     =   -1,
	.dev    =   {
		.platform_data  = &bssq_vib_platform_data,
	},
};
#endif

#ifdef CONFIG_BSSQ_QWERTY_LED
static struct max8907c_led_platform_data bssq_qwerty_led_platform_data = {
    .default_trigger = "timer",
    .max_uA          = 27899,
};

static struct platform_device bssq_qwerty_led_device = {
	.name   =   "bssq_qwerty_led",
	.id     =   -1,
	.dev	=	{
		.platform_data  = &bssq_qwerty_led_platform_data,
    },
};
#endif

#if 0
// [START] kangsic.ham 2011.06.15 - GPS UART & GPIO Setting 
#if defined(CONFIG_SU880) || defined(CONFIG_KU8800)
static struct gps_gpio_platform_data gps_pdata = {
	.pwron	 = GPS_PWR_ON_GPIO,
	.reset_n = GPS_RESET_N_GPIO,
	.lna_sd  = GPS_LNA_SD_GPIO,
};

static struct platform_device gps_gpio =
{
	.name = "gps_gpio",
	.id	  = -1,
	.dev = {
		.platform_data = &gps_pdata,
	}
};
#endif
// [END] kangsic.ham 2011.06.15 - GPS UART & GPIO Setting 

/****************************************************************************************************
*                                       I2C devices
****************************************************************************************************/
//20110615 seki.park@lge.com HW tuning [S]
#if defined  (CONFIG_SU880) || defined (CONFIG_KU8800) 
static struct wm8994_drc_cfg wm8994_drc_data[] = {
	{
		.name = "AIF1DRC1 Mode",
		.regs = {0x19B, 0x843, 0x818, 0x340, 0x1AC},
	},
	{
		.name = "AIF2DRC Mode",
	 	.regs = {0x198, 0x843, 0x818, 0x340, 0x1AC},
	},
};
#elif defined (CONFIG_LU8800)
static struct wm8994_drc_cfg wm8994_drc_data[] = {
	{
		.name = "AIF1DRC1 Mode",
		.regs = {0x19B, 0x843, 0x818, 0x340, 0x1AC},
	},
	{
		.name = "AIF2DRC Mode",
	 	.regs = {0x198, 0x84E, 0x818, 0x265, 0x187},
	},
};
#elif defined (CONFIG_KS1001) || defined (CONFIG_KS1103)   // 20111205 unyou.shim@lge.com Audio Tunning requested by H/W  
static struct wm8994_drc_cfg wm8994_drc_data[] = {
	{
		.name = "AIF1DRC1 Mode",
		.regs = {0x19B, 0x843, 0x818, 0x320, 0x30C},
	},
	{
		.name = "AIF2DRC Mode",
	 	.regs = {0x198, 0x84E, 0x818, 0x265, 0x187},
	},
};

#else
static struct wm8994_drc_cfg wm8994_drc_data[] = {
	{
		.name = "AIF1DRC1 Mode",
		.regs = {0x19B, 0x853, 0x400, 0x2A0, 0xC0},
	},
	{
		.name = "AIF2DRC Mode",
	 	.regs = {0x198, 0x84E, 0x818, 0x265, 0x187},
	},
};
#endif

static struct wm8994_pdata wm8994_data = {
	.num_drc_cfgs = 2,
	.drc_cfgs  = &wm8994_drc_data,
};

static struct i2c_board_info __initdata bssq_i2c_codec_info[] = {
	{
		I2C_BOARD_INFO("wm8994", 0x1A),
		.platform_data = &wm8994_data,
	},
};
//20110615 seki.park@lge.com HW tuning [E]

#endif

static struct mpu3050_platform_data mpu3050_data = {
	.int_config		= 0x10,
	.orientation	= {	 0, -1,  0,
						-1,  0,  0,
						 0,  0, -1 },
	.level_shifter = 0,
	.accel = {
		.get_slave_descr = kxtf9_get_slave_descr,
		//20110927 - woo.jung@lge.com - Accel does not use Interrupt Method,but it use Polling Method in HAL for events [S]
		//.irq			= TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PF4),
		//20110927 - woo.jung@lge.com - Accel does not use Interrupt Method,but it use Polling Method in HAL for events [E]
		.adapt_num		= 2, 
		.bus			= EXT_SLAVE_BUS_SECONDARY,
		.address		= 0x0F,
		.orientation	= {	 1,  0,  0,
						     0, -1,  0,
							 0,  0, -1 },
	},
//20110816 - woo.jung@lge.com - Compass HAL Porting - [S]
//20110928 - woo.jung@lge.com - Compass initialization for mpu [S]
	.compass = {
		.get_slave_descr = ak8975_get_slave_descr,
		//20110927 - woo.jung@lge.com - Compass does not use Interrupt Method,but it use Polling Method in HAL for events [S]
		//.irq			= TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PF0),
		//20110927 - woo.jung@lge.com - Compass does not use Interrupt Method,but it use Polling Method in HAL for events [E]
		.adapt_num		= 2, 
		.bus			= EXT_SLAVE_BUS_PRIMARY,
		.address		= 0x0D,
#if defined(CONFIG_KS1001)
		.orientation	= { 1,  0,  0,
							0,  1,  0,
							0,  0,  1 },
#else	
		.orientation	= { 1,	0,	0,
							0, -1,	0,
							0,  0,	-1},
#endif
	},
//20110928 - woo.jung@lge.com - Compass initialization for mpu [E]
};

#if defined(CONFIG_KS1103)
static struct akm8975_platform_data akm_platform_data_8975 = {
		.gpio_DRDY	= TEGRA_GPIO_PF0,
};

static struct i2c_board_info __initdata bssq_i2c_sensor_compass[] = {
	{
		I2C_BOARD_INFO("akm8975", 0x0D),
		.flags = I2C_CLIENT_WAKE,
		.platform_data = &akm_platform_data_8975,
		.irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PF0),
	},
};
#endif
//20110816 - woo.jung@lge.com - Compass HAL Porting - [E]

static struct i2c_board_info __initdata bssq_i2c_sensor_info[] = {
	{
		I2C_BOARD_INFO("mpu3050", 0x68),
		.platform_data = &mpu3050_data,
#if defined (CONFIG_KS1103)
		.irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PF6),
#else
		.irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PB3),
#endif
	},
};

#if defined(CONFIG_SUBPMIC_LP8720)
static struct lp8720_platform_data	lp8720_pdata = {
	.en_gpio_num = TEGRA_GPIO_PF2,
};
#endif

static struct i2c_board_info __initdata bssq_i2c_bus3_devices_info[] = {
#if defined(CONFIG_SUBPMIC_LP8720)
	{
		I2C_BOARD_INFO(LP8720_I2C_NAME, LP8720_I2C_ADDR),
		.platform_data = &lp8720_pdata,
	},
#endif
#if defined (CONFIG_KS1001) || defined (CONFIG_KS1103)

	{
		I2C_BOARD_INFO("ar0832", 			0x36),
	},
//20111119 calvin.hwang@lge.com EEPROM driver [S]
	// E2PROM_BU9889GUL-W(1K)  
	{
		I2C_BOARD_INFO("bu9889gulw", 		0x50),//0xa0 >> 1
	},
//20111119 calvin.hwang@lge.com EEPROM driver [E]
#elif defined (CONFIG_LU6500) || defined (CONFIG_SU880) || defined (CONFIG_KU8800) || defined (CONFIG_LU8800)

	{
		I2C_BOARD_INFO("imx072", 0x1A),
	},
	{
		I2C_BOARD_INFO("dw9712", 0x0C),
	},
#endif	

	{
		I2C_BOARD_INFO("hi702", 			0x30), //20110518 jinkwan.kim@lge.com Camsensor 1st merge
	},
};

static struct i2c_board_info __initdata tegra_i2c_fuelgauge_info[] = {
	{
		I2C_BOARD_INFO("max17043", 0x36),
		.platform_data = NULL,
	},
};

#if defined(CONFIG_LU6500) || defined(CONFIG_SU880)|| defined(CONFIG_KU8800)
static struct i2c_board_info __initdata tegra_i2c_muic_info[] = {
	{
		I2C_BOARD_INFO("bssq_muic_ti", 0x44), // MUIC(R:0x88)
		.platform_data = NULL,
	},
};
#else
static struct i2c_board_info __initdata bssq_i2c_bus6_devices_info[] ={
	    {
	        I2C_BOARD_INFO("max14526", 0x44),
	    },
};
#endif
static struct i2c_board_info __initdata tegra_i2c_charger_info[] = {
	{
		I2C_BOARD_INFO("bssq_charger_max", 0x35), // charger IC(R:0x6A)
		.platform_data = NULL,
	},
};

static struct lge_synaptics_platform_data bssq_ts_data = {
	.gpio	    = TEGRA_GPIO_PX6,
	.power		= &device_power_control,
	.irqflags   = IRQF_TRIGGER_FALLING,
};

static struct i2c_board_info __initdata bssq_i2c_touch_info[] = {
	{
		I2C_BOARD_INFO(LGE_TOUCH_NAME,	LGE_TOUCH_ADDR),
		.irq			=	TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PX6),
		.platform_data	=	&bssq_ts_data,
	},
};

//#ifdef CONFIG_BSSQ_PROXIMITY
#ifdef CONFIG_SENSOR_GP2A
static struct i2c_board_info __initdata bssq_i2c_proxi_info[] = {
	{
		I2C_BOARD_INFO(GP2A_NAME, GP2A_ADDR),
		.irq			=	TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PX5),
		.platform_data	=	0,
	},
};
#endif
#if defined(CONFIG_BSSQ_TOUCH_LED)
//#ifdef CONFIG_BSSQ_TOUCH_LED
static struct bd2802_led_platform_data bd2802_led_pdata = {
//	.reset_gpio = TEGRA_GPIO_PE6,
	.rgb_time = (3<<6)|(3<<4)|(4),
};

static struct i2c_board_info __initdata tegra_i2c_touch_led_info[] = {
	 {
		 I2C_BOARD_INFO("bssq_touch_led", 0x1A),
		 .platform_data = &bd2802_led_pdata,
	 },
};
//#endif
#endif
#if 0
#ifdef CONFIG_BSSQ_TOUCH_LED
static struct bd2802_led_platform_data bd2802_led_pdata = {
//	.reset_gpio = TEGRA_GPIO_PE6,
	.rgb_time = (3<<6)|(3<<4)|(4),
};

static struct i2c_board_info __initdata tegra_i2c_touch_led_info[] = {
	 {
		 I2C_BOARD_INFO("bssq_touch_led", 0x1A),
		 .platform_data = &bd2802_led_pdata,
	 },
};
#endif
#endif

#if defined(CONFIG_LU6500)	// MOBII_CHANGE 20120711 sk.jung@mobii.co.kr : Fixed flash mode
static struct bssq_flash_led_platform_data flash_led_data = {

	.gpio_hwen = TEGRA_GPIO_PBB4,
};
static const struct i2c_board_info bssq_flash_led_info[] = {
	{
		I2C_BOARD_INFO("bssq_flash_led",	0x53),
		.platform_data	=	&flash_led_data,
	},
};	
#endif


// 20110524 bg80.song@lge.com AP Temp Sensor Bring-up [S]
extern void tegra_throttling_enable(bool enable);

static struct nct1008_platform_data bssq_ap_temp_pdata = {
	.supported_hwrev = true,
	.ext_range = false,
	.conv_rate = 0x08,
	.offset = 0,
	.hysteresis = 5,
	.shutdown_ext_limit = 115,
	.shutdown_local_limit = 120,
#if defined (CONFIG_KS1103)
	.throttling_ext_limit = 60,
#else
	.throttling_ext_limit = 90,
#endif
	.alarm_fn = tegra_throttling_enable,
};

static struct i2c_board_info bssq_ap_temp_info[] = {
	{
		I2C_BOARD_INFO("nct1008", 0x4C),
		.irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PK2),
		.platform_data = &bssq_ap_temp_pdata,
	},
};
// 20110524 bg80.song@lge.com AP Temp Sensor Bring-up [E]

/****************************************************************************************************
*                                       SPI devices
****************************************************************************************************/
static struct spi_board_info __initdata bssq_spi_bus1_devices_info[] = {
#if defined(CONFIG_LU6500) || defined (CONFIG_LU8800) || defined(CONFIG_KS1001) || defined(CONFIG_KS1103)
	{
		.modalias = "mdm6600",
		.bus_num = 0,
		.chip_select = 0,
		.mode = SPI_MODE_1,
		.max_speed_hz = 24000000,
		.controller_data = &tegra_spi_slave_device1,  
		.irq = 0,
		.platform_data = 0,
	},
//20110607 ws.yang@lge.com add to feature [S]
#elif defined(CONFIG_SU880) || defined (CONFIG_KU8800)
	{
		.modalias = "ifxn721",
		.bus_num = 0,
		.chip_select = 0,
		.mode = SPI_MODE_1,
		.max_speed_hz = 24000000,
		.controller_data = &tegra_spi_device1,  
		.irq = 0, // 277 ? GPIO_IRQ(TEGRA_GPIO_PO5),
		.platform_data = 0,
	},
#endif
//20110607 ws.yang@lge.com add to feature [E]
};

#if defined(CONFIG_DUAL_SPI)
static struct spi_board_info __initdata bssq_spi_bus2_devices_info[] = {
	{
		.modalias = "mdm6600",
		.bus_num = 1,
		.chip_select = 1,
		.mode = SPI_MODE_1,
		.max_speed_hz = 24000000,
		.controller_data = &tegra_spi_slave_device2,  
		.irq = 0,
		.platform_data = 0,
	},
};
#endif	

static struct spi_board_info __initdata bssq_spi_bus3_devices_info[] = {
#if defined(CONFIG_KS1001) || defined(CONFIG_KS1103)
	{
		.modalias = "isdbt",
		.bus_num = 2,
		.chip_select = 3,
		.mode = SPI_MODE_0,
		.max_speed_hz = 8000000,
		.controller_data = &tegra_spi_device3,	
		.irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PO6),
		.platform_data = 0,
	},
#elif defined (CONFIG_LU6500) || defined (CONFIG_SU880) || defined (CONFIG_KU8800) || defined (CONFIG_LU8800)
//modified by wonhee.jeong@lge.com for B_Qwerty TDMB Porting 110413 (Start)
	{
		.modalias = "tdmb_t3900",
		.bus_num = 2,
		.chip_select = 3,//DMB_SPI_CS3_N
		.mode = SPI_MODE_0,
		.max_speed_hz = 5000*1000,
		.controller_data = &tegra_spi_device3,
		//.irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PO6),
		.irq = 0, //TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PO6),
		//.platform_data = 
	},
#endif
};
//modified by wonhee.jeong@lge.com for B_Qwerty TDMB Porting 110413 (End)
