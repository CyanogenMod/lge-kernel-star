ccflags-y += -DNV_IS_AVP=0
ccflags-y += -DNV_OAL=0
ccflags-y += -DNV_USE_FUSE_CLOCK_ENABLE=0
ifeq ($(CONFIG_MACH_TEGRA_GENERIC_DEBUG),y)
ccflags-y += -DNV_DEBUG=1
else
ccflags-y += -DNV_DEBUG=0
endif

ccflags-y += -Iarch/arm/mach-tegra/odm_kit/adaptations/pmu
ccflags-y += -Iarch/arm/mach-tegra/odm_kit/adaptations/pmu/pcf50626

obj-y += ds2482_bridge.o
obj-y += ds2482_i2c.o
obj-y += pcf50626_adc.o
obj-y += pcf50626_batterycharger.o
obj-y += pcf50626.o
obj-y += pcf50626_i2c.o
obj-y += pcf50626_interrupt.o
obj-y += pcf50626_rtc.o
obj-y += platform.o
