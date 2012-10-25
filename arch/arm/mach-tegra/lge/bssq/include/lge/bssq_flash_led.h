#ifndef __bssq_flash_led_H
#define __bssq_flash_led_H

#define ENABLE_REG_INDEX					0x10
#define PRIVACY_REG_INDEX					0x11
#define INDICATOR_REG_INDEX				0x12
#define INDICATOR_BLINKING_REG_INDEX		0x13
#define PRIVACY_PWM_REG_INDEX				0x14
#define GPIO_REG_INDEX						0x20
#define VLED_MONITOR_REG_INDEX			0x30
#define ADC_DELAY_REG_INDEX				0x31
#define VIN_MONITOR_REG_INDEX				0x80
#define LAST_FLASH_REG_INDEX				0x81
#define TORCH_BRIGHTNESS_REG_INDEX		0xA0
#define FLASH_BRIGHTNESS_REG_INDEX		0xB0
#define FLASH_DURATION_REG_INDEX			0xC0
#define FLAGS_REG_INDEX					0xD0
#define CONFIGURATION1_REG_INDEX			0xE0
#define CONFIGURATION2_REG_INDEX			0xF0


#define bssq_flash_led_I2C_NAME		"bssq_flash_led"
#define bssq_flash_led_I2C_ADDR		0x53

//GPIO_PBB4 FLASH_LED_EN

struct bssq_flash_led_platform_data {
	int		gpio_hwen;
};


#endif
