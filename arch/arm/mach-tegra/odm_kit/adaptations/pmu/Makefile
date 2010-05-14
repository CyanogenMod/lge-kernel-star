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
ccflags-y += -Iarch/arm/mach-tegra/odm_kit/adaptations/pmu/pcf50626
ccflags-y += -Iarch/arm/mach-tegra/odm_kit/adaptations/pmu/tps6586x

obj-y += pmu_hal.o
obj-y += max8907b/
obj-y += pcf50626/
obj-y += tps6586x/
