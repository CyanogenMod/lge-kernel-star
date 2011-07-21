#
# Makefile for the touchscreen drivers.
#

# Each configuration option enables a list of files.

#20100802, LCD IF fast init
obj-y				+= lg_panel_gpio_set.o
#20100802, LCD IF fast init
#20101119, , Camera Flash fast init
obj-y				+= tegra_user_gpio.o
#20101119, , Camera Flash fast init
obj-$(CONFIG_STAR_BATTERY_CHARGER)	+= star_battery_charger.o
obj-$(CONFIG_STAR_MUIC_TI)	+= star_muic_TI.o
obj-$(CONFIG_STAR_MUIC)	+= star_muic.o
obj-$(CONFIG_STAR_PROXIMITY)	+= star_proximity.o
#obj-$(CONFIG_MACH_STAR)	+= star_lp8720tlx.o
#20100413, star powerkey
obj-$(CONFIG_STAR_POWERKEY) += star_powerkey.o
#20101129, , SU660 home key
obj-$(CONFIG_STAR_HOMEKEY) += star_homekey.o
#20100611, star TOUCHLED
obj-$(CONFIG_STAR_TOUCH_LED) += star_touch_led.o
#20100702, star HDMI regulator
obj-$(CONFIG_STAR_HDMI_REG) += star_hdmi_reg.o

#20100419, , headset detection
obj-$(CONFIG_STAR_HEADSET_DET) += headset_det.o

#LGE_KERNEL_MUX START
obj-$(CONFIG_STAR_RIL) += ts0710mux/
EXTRA_CFLAGS += -DLGE_KERNEL_MUX
obj-$(CONFIG_RINETWORK_DRIVER) += rindrv.o
#LGE_KERNEL_MUX END

obj-$(CONFIG_LOGGER)		+= logger.o

obj-$(CONFIG_STAR_WM8994_TEST)	+= star_wm8994.o

#20100827, 
obj-$(CONFIG_STAR_GYRO_ACCEL)	+= sensor/
obj-$(CONFIG_STAR_COMPASS)		+= sensor/

obj-$(CONFIG_STAR_SENSORS)	+= sensor.module/
obj-$(CONFIG_STAR_GYROSCOPE2)	+= sensor.module/
obj-$(CONFIG_STAR_ACCELEROMETER2)	+= sensor.module/
obj-$(CONFIG_STAR_COMPASS2)		+= sensor.module/

#obj-$(CONFIG_STAR_GYRO_ACCEL) += gyro_accel.o
#obj-$(CONFIG_STAR_GYRO_ACCEL) += mpu3050.o
#obj-$(CONFIG_STAR_GYRO_ACCEL) += gyro_accel_kxtf9.o
#obj-$(CONFIG_STAR_GYRO_ACCEL) += nvodm_gyro_accel.o
#obj-$(CONFIG_STAR_GYRO_ACCEL) += nvodm_accelerometer_accel_kxtf9.o
#obj-$(CONFIG_STAR_COMPASS)    += ami304_compass.o
#obj-$(CONFIG_STAR_COMPASS)    += nvodm_compass_ami304.o

obj-$(CONFIG_STAR_ERS_TEST)	+= star_ers.o
#20100730, , CP Watcher
ifeq ($(CONFIG_MACH_STAR_TMUS),y)
obj-$(CONFIG_STAR_CPWATCHER) += cpwatcher_p999.o
else
obj-$(CONFIG_STAR_CPWATCHER) += cpwatcher.o
endif

obj-$(CONFIG_STAR_HALL)	+= star_hall.o
obj-$(CONFIG_STAR_BL) += star_bl.o

#20101026, 
obj-$(CONFIG_LGE_ATS_INPUT_DEVICE) += star_ats.o
obj-$(CONFIG_LGE_ATS_INPUT_DEVICE) += lge_ats_eta.o
obj-$(CONFIG_LGE_ATS_INPUT_DEVICE) += lge_ats_event_log.o

#20101105, 
obj-$(CONFIG_STAR_FOTA)	+= star_fota.o

#
obj-$(CONFIG_STAR_BD_ADDRESS)	+= star_bd_address.o
obj-$(CONFIG_STAR_VIBRATOR)	+= star_vib.o
