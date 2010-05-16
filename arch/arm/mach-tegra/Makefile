obj-y                                   += common.o
obj-y                                   += io.o
obj-y                                   += irq.o
ifneq ($(CONFIG_TEGRA_NVRM),y)
obj-y                                   += clock.o
obj-$(CONFIG_ARCH_TEGRA_2x_SOC)         += tegra2_clocks.o
endif
obj-y                                   += timer.o
obj-y                                   += gpio.o
obj-y					+= suspend.o
obj-y					+= pinmux.o
obj-y					+= pinmux-t2-tables.o
obj-y                                   += delay.o
obj-$(CONFIG_SMP)                       += platsmp.o localtimer.o headsmp.o
obj-$(CONFIG_ARCH_TEGRA_2x_SOC)		+= cortex_a9_save.o
obj-$(CONFIG_MACH_TEGRA_GENERIC)	+= board-generic.o
obj-$(CONFIG_CPU_IDLE)			+= cpuidle.o
obj-$(CONFIG_TEGRA_IOVMM)		+= iovmm.o
obj-$(CONFIG_TEGRA_IOVMM_GART)		+= iovmm-gart.o
obj-$(CONFIG_TEGRA_SYSTEM_DMA)		+= dma.o

ccflags-$(CONFIG_TEGRA_NVOS)		+= -DNV_DEBUG=0
ccflags-$(CONFIG_TEGRA_NVOS)		+= -DNV_OAL=0
ccflags-$(CONFIG_TEGRA_NVOS)		+= -DNV_IS_AVP=0
ccflags-$(CONFIG_TEGRA_NVOS)		+= -DNV_USE_FUSE_CLOCK_ENABLE=0
obj-$(CONFIG_TEGRA_NVOS)		+= nvos_user.o
obj-$(CONFIG_TEGRA_NVOS)		+= nvos/

obj-$(CONFIG_TEGRA_NVRM)		+= nvrm/
obj-$(CONFIG_TEGRA_NVRM)		+= nvodm/
obj-$(CONFIG_TEGRA_NVRM)		+= odm_kit/
obj-$(CONFIG_TEGRA_NVRM)		+= nvreftrack/
obj-$(CONFIG_TEGRA_NVRM)		+= nvrm_user.o
obj-$(CONFIG_TEGRA_NVRM)		+= clock_nvrm.o