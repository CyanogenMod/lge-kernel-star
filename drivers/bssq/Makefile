# drivers/bssq/Makefile

obj-$(CONFIG_BSSQ_FUELGAUGE)			+= bssq_gauge.o
obj-$(CONFIG_BSSQ_CHARGER_RT)		+= bssq_charger_rt.o
obj-$(CONFIG_BSSQ_CHARGER_MAX)		+= bssq_charger_max.o


ifneq (, $(filter lu6500 su880 ku8800 , $(HW_REV)))
obj-$(CONFIG_BSSQ_BATTERY)			+= bssq_battery_lu6500.o
obj-$(CONFIG_BSSQ_MUIC_TI)				+= bssq_muic_ti.o
else
obj-$(CONFIG_BSSQ_BATTERY)			+= bssq_battery.o
endif


obj-$(CONFIG_MACH_BSSQ)					+= bssq_hw_rev.o
#obj-$(CONFIG_BSSQ_PROXIMITY) 			+= bssq_proximity.o
obj-$(CONFIG_CAM_PMIC)					+= bssq_cam_pmic.o
obj-$(CONFIG_SUBPMIC_LP8720)			+= lp8720.o

obj-$(CONFIG_BSSQ_TOUCH_LED)			+= bssq_touch_led.o
obj-$(CONFIG_BSSQ_QWERTY_LED)		+= bssq_qwerty_led.o

obj-$(CONFIG_BSSQ_VIBRATOR)             += bssq_vib.o
#LGE_KERNEL_MUX START
EXTRA_CFLAGS += -DLGE_KERNEL_MUX
obj-$(CONFIG_BSSQ_RIL)			+= ts0710mux/
#20110609 ws.yang@lge.com add to target model [S]
ifneq (, $(filter su880 ku8800 , $(HW_REV)))
obj-$(CONFIG_BSSQ_RIL)			+= ifx_n721_spi.o
obj-$(CONFIG_BSSQ_CPWATCHER)		+= cpwatcher.o
else
obj-$(CONFIG_BSSQ_RIL)		+= spi_mdm6600.o
obj-$(CONFIG_BSSQ_CPWATCHER)		+= bssq_cpwatcher.o
endif
#20110609 ws.yang@lge.com add to target model [E]
obj-$(CONFIG_RINETWORK_DRIVER)		+= rindrv.o
#LGE_KERNEL_MUX END

# 20110611 changbum.kim@lge.com add mtc_eta driver [S]
obj-$(CONFIG_LGE_MTC_ETA)			+= mtc_eta/
# 20110611 changbum.kim@lge.com add mtc_eta driver [E]

# MOBII_CHANGE_S 20120711 sk.jung@mobii.co.kr : Fixed flash mode
obj-$(CONFIG_LU6500) 			+= bssq_flash_led.o
# MOBII_CHANGE_E 20120711 sk.jung@mobii.co.kr : Fixed flash mode
