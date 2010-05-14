ccflags-y += -DNV_IS_AVP=0
ccflags-y += -DNV_OAL=0
ccflags-y += -DNV_USE_FUSE_CLOCK_ENABLE=0
ifeq ($(CONFIG_MACH_TEGRA_GENERIC_DEBUG),y)
ccflags-y += -DNV_DEBUG=1
else
ccflags-y += -DNV_DEBUG=0
endif

ccflags-y += -Iarch/arm/mach-tegra/odm_kit/adaptations/pmu
ccflags-y += -Iarch/arm/mach-tegra/odm_kit/adaptations/pmu/max8907b

obj-y += ad5258_dpm.o
obj-y += fan5355_buck_i2c.o
obj-y += max8907b_adc.o
obj-y += max8907b_batterycharger.o
obj-y += max8907b.o
obj-y += max8907b_i2c.o
obj-y += max8907b_interrupt.o
obj-y += max8907b_rtc.o
obj-y += mic2826_i2c.o
obj-y += tca6416_expander_i2c.o
