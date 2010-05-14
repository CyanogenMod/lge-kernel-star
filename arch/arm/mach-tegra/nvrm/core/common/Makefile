ccflags-y += -DNV_IS_AVP=0
ccflags-y += -DNV_OAL=0
ccflags-y += -DNV_USE_FUSE_CLOCK_ENABLE=0
ccflags-y += -DNV_SHMOO_DATA_INIT=0
ifeq ($(CONFIG_MACH_TEGRA_GENERIC_DEBUG),y)
ccflags-y += -DNV_DEBUG=1
else
ccflags-y += -DNV_DEBUG=0
endif

ccflags-y += -Iarch/arm/mach-tegra/nvrm/core/common
ccflags-y += -Iarch/arm/mach-tegra/nvrm/core

obj-y += nvrm_pinmux.o
obj-y += nvrm_keylist.o
obj-y += nvrm_configuration.o
obj-y += nvrm_pmu.o
obj-y += nvrm_module.o
obj-y += nvrm_hwintf.o
obj-y += nvrm_chiplib.o
obj-y += nvrm_clocks_limits.o
obj-y += nvrm_clocks_limits_stub.o
obj-y += nvrm_power.o
obj-y += nvrm_power_dfs.o
obj-y += nvrm_rmctrace.o
obj-y += nvrm_relocation_table.o
obj-y += nvrm_transport.o
