ccflags-y += -DNV_IS_AVP=0
ccflags-y += -DNV_OAL=0
ccflags-y += -DNV_USE_FUSE_CLOCK_ENABLE=0
ifeq ($(CONFIG_MACH_TEGRA_GENERIC_DEBUG),y)
ccflags-y += -DNV_DEBUG=1
else
ccflags-y += -DNV_DEBUG=0
endif

ccflags-y += -Iarch/arm/mach-tegra/odm_kit/adaptations/pmu
ccflags-y += -Iarch/arm/mach-tegra/odm_kit/adaptations/pmu/tps6586x

obj-y += nvodm_pmu_tps6586x.o
obj-y += nvodm_pmu_tps6586x_adc.o
obj-y += nvodm_pmu_tps6586x_batterycharger.o
obj-y += nvodm_pmu_tps6586x_i2c.o
obj-y += nvodm_pmu_tps6586x_interrupt.o
obj-y += nvodm_pmu_tps6586x_rtc.o
