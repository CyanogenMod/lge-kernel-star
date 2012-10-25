#
# Makefile for misc devices that really don't fit anywhere else.
#
GCOV_PROFILE_tegra-cryptodev.o := y

obj-$(CONFIG_IBM_ASM)		+= ibmasm/
obj-$(CONFIG_AD525X_DPOT)	+= ad525x_dpot.o
obj-$(CONFIG_AD525X_DPOT_I2C)	+= ad525x_dpot-i2c.o
obj-$(CONFIG_AD525X_DPOT_SPI)	+= ad525x_dpot-spi.o
obj-$(CONFIG_ATMEL_PWM)		+= atmel_pwm.o
obj-$(CONFIG_ATMEL_SSC)		+= atmel-ssc.o
obj-$(CONFIG_ATMEL_TCLIB)	+= atmel_tclib.o
obj-$(CONFIG_BMP085)		+= bmp085.o
obj-$(CONFIG_ICS932S401)	+= ics932s401.o
obj-$(CONFIG_LKDTM)		+= lkdtm.o
obj-$(CONFIG_TIFM_CORE)		+= tifm_core.o
obj-$(CONFIG_TIFM_7XX1)		+= tifm_7xx1.o
obj-$(CONFIG_PHANTOM)		+= phantom.o
obj-$(CONFIG_SENSORS_BH1780)	+= bh1780gli.o
obj-$(CONFIG_SENSORS_BH1770)	+= bh1770glc.o
obj-$(CONFIG_SENSORS_APDS990X)	+= apds990x.o
obj-$(CONFIG_ANDROID_PMEM)	+= pmem.o
obj-$(CONFIG_SGI_IOC4)		+= ioc4.o
obj-$(CONFIG_ENCLOSURE_SERVICES) += enclosure.o
obj-$(CONFIG_KERNEL_DEBUGGER_CORE)	+= kernel_debugger.o
obj-$(CONFIG_KGDB_TESTS)	+= kgdbts.o
obj-$(CONFIG_SGI_XP)		+= sgi-xp/
obj-$(CONFIG_SGI_GRU)		+= sgi-gru/
obj-$(CONFIG_CS5535_MFGPT)	+= cs5535-mfgpt.o
obj-$(CONFIG_HP_ILO)		+= hpilo.o
obj-$(CONFIG_APDS9802ALS)	+= apds9802als.o
obj-$(CONFIG_ISL29003)		+= isl29003.o
obj-$(CONFIG_ISL29020)		+= isl29020.o
obj-$(CONFIG_SENSORS_TSL2550)	+= tsl2550.o
obj-$(CONFIG_EP93XX_PWM)	+= ep93xx_pwm.o
obj-$(CONFIG_DS1682)		+= ds1682.o
obj-$(CONFIG_TI_DAC7512)	+= ti_dac7512.o
obj-$(CONFIG_UID_STAT)		+= uid_stat.o
obj-$(CONFIG_C2PORT)		+= c2port/
obj-$(CONFIG_IWMC3200TOP)      += iwmc3200top/
obj-$(CONFIG_HMC6352)		+= hmc6352.o
obj-y				+= eeprom/
obj-y				+= cb710/
#obj-y				+= muic/
# sanghyun.hong@lge.com Changes : Mobii, inc. #
#ifeq ($(CONFIG_LU6500),y)
#else
#obj-$(CONFIG_MAX14526)                      += muic/
#endif

ifneq (, $(filter lu6500 su880 ku8800 , $(HW_REV)))
else
obj-$(CONFIG_MAX14526)                      += muic/
endif

###############################################
obj-$(CONFIG_SPEAR13XX_PCIE_GADGET)	+= spear13xx_pcie_gadget.o
obj-$(CONFIG_VMWARE_BALLOON)	+= vmw_balloon.o
obj-$(CONFIG_ARM_CHARLCD)	+= arm-charlcd.o
obj-$(CONFIG_PCH_PHUB)		+= pch_phub.o
obj-y				+= ti-st/
obj-$(CONFIG_AB8500_PWM)	+= ab8500-pwm.o
obj-y				+= lis3lv02d/
obj-$(CONFIG_WL127X_RFKILL)	+= wl127x-rfkill.o
obj-$(CONFIG_APANIC)		+= apanic.o
obj-$(CONFIG_SENSORS_AK8975)	+= akm8975.o
obj-$(CONFIG_SENSORS_NCT1008)	+= nct1008.o
#//LGE_CHANGE_S [munho2.lee@lge.com] 2012-02-23 BT Bring-up
obj-$(CONFIG_BCM4329_RFKILL)	+= lbee9qmb-rfkill.o
obj-$(CONFIG_BD_ADDRESS)	+= bd_address.o
# +++ BRCM_LOCAL : for supporting HID profile
obj-y				+= bthid/
# --- BRCM_LOCAL
#obj-$(CONFIG_BCM4329_RFKILL)	+= bcm4329_rfkill.o
#//LGE_CHANGE_E
#MOBII_CHNANGE_S 20120213 mg.chang@mobii.co.kr : Appy Sensor Porting
obj-$(CONFIG_MPU_SENSORS_MPU3050)	+= mpu3050/
#obj-$(CONFIG_MPU_SENSORS_MPU3050)	+= inv_3050/
#MOBII_CHNANGE_E 20120213 mg.chang@mobii.co.kr : Appy Sensor Porting
obj-$(CONFIG_TEGRA_CRYPTO_DEV)	+= tegra-cryptodev.o
obj-$(CONFIG_TEGRA_BB_SUPPORT)	+= tegra-baseband/
obj-$(CONFIG_MAX1749_VIBRATOR)	+= max1749.o
#obj-$(CONFIG_BATTERY_CHARGER)   += battery_charger.o
# LGE_CHANGE_S [sungyel.bae@lge.com] 2011-01-21 [LGE_AP20] star_touch_led add
obj-$(CONFIG_STAR_TOUCH_LED)	+= star_led.o
# LGE_CHANGE_E [sungyel.bae@lge.com] 2011-01-21 [LGE_AP20] star_touch_led add
obj-$(CONFIG_STAR_VIBRATOR)		+= vibrator.o
#110618 taehyun.lim@lge.com Porting Immversion Vib[S]
obj-$(CONFIG_TSPDRV)        += tspdrv/
#110618 taehyun.lim@lge.com Porting Immversion Vib[E]
obj-$(CONFIG_STAR_HALL)		+= bu52031nvx.o

