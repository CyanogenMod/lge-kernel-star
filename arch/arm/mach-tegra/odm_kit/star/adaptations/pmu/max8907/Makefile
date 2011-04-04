ccflags-y += -DNV_IS_AVP=0
ccflags-y += -DNV_OAL=0
ccflags-y += -DNV_USE_FUSE_CLOCK_ENABLE=0
ifeq ($(CONFIG_MACH_TEGRA_GENERIC_DEBUG),y)
ccflags-y += -DNV_DEBUG=1
else
ccflags-y += -DNV_DEBUG=0
endif

ccflags-y += -Iarch/arm/mach-tegra/odm_kit/star/adaptations/pmu
ccflags-y += -Iarch/arm/mach-tegra/odm_kit/star/adaptations/pmu/max8907

obj-y += max8907_adc.o
obj-y += max8907_batterycharger.o
obj-y += max8907.o
obj-y += max8907_i2c.o
obj-y += max8907_interrupt.o
obj-y += max8907_rtc.o
obj-y += max8952_buck_i2c.o
