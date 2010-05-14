ccflags-y += -DNV_IS_AVP=0
ccflags-y += -DNV_OAL=0
ccflags-y += -DNV_USE_FUSE_CLOCK_ENABLE=0
ifeq ($(CONFIG_MACH_TEGRA_GENERIC_DEBUG),y)
ccflags-y += -DNV_DEBUG=1
else
ccflags-y += -DNV_DEBUG=0
endif

ccflags-y += -Iarch/arm/mach-tegra/nvrm/core/common
ccflags-y += -Iarch/arm/mach-tegra/nvrm/core

obj-y += ap15rm_interrupt_generic.o
obj-y += ap15rm_hwmap.o
obj-y += ap15rm_clocks.o
obj-y += ap15rm_clock_config.o
obj-y += ap15rm_clocks_info.o
obj-y += nvrm_clocks.o
obj-y += ap15rm_pinmux.o
obj-y += ap15rm_pinmux_tables.o
obj-y += ap16rm_pinmux_tables.o
obj-y += ap15rm_power.o
obj-y += ap15rm_power_dfs.o
obj-y += ap15rm_power_oalintf.o
obj-y += ap15rm_clock_misc.o
obj-y += ap15rm_memctrl.o
obj-y += ap15rm_fuse.o
obj-y += nvrm_diag.o
obj-y += ap15rm_reloctable.o
obj-y += ap16rm_reloctable.o
obj-y += ap15rm_init.o
obj-y += ap15rm_init_common.o
obj-y += ap15rm_interrupt.o
obj-y += ap15rm_xpc.o
obj-y += ap15rm_xpc_hw_private.o
