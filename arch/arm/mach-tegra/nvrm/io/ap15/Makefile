ccflags-y += -DNV_IS_AVP=0
ccflags-y += -DNV_OAL=0
ccflags-y += -DNV_USE_FUSE_CLOCK_ENABLE=0
ifeq ($(CONFIG_MACH_TEGRA_GENERIC_DEBUG),y)
ccflags-y += -DNV_DEBUG=1
else
ccflags-y += -DNV_DEBUG=0
endif

ccflags-y += -Iarch/arm/mach-tegra/nvrm/core/common
ccflags-y += -Iarch/arm/mach-tegra/nvrm/io/common
ccflags-y += -Iarch/arm/mach-tegra/nvrm/io
ccflags-y += -Iarch/arm/mach-tegra/nvrm/core

obj-y += ap15rm_analog.o
obj-y += rm_dma_hw_private.o
obj-y += ap15rm_dma_hw_private.o
obj-y += ap15rm_slink_hw_private.o
obj-y += rm_common_slink_hw_private.o
obj-y += ap15rm_i2c.o
obj-y += ap15rm_pwm.o
obj-y += ap15rm_gpio_vi.o
obj-y += nvrm_dma.o
obj-y += nvrm_gpio.o
obj-y += nvrm_gpio_stub_helper.o
obj-y += ap15rm_dma_intr.o
obj-y += rm_spi_hw_private.o
obj-y += rm_spi_slink.o
