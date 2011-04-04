obj-y                                   += common.o
obj-$(CONFIG_ARCH_TEGRA_2x_SOC)         += common-t2.o
obj-y					+= cortex-a9.o
obj-y					+= board-common.o
obj-y                                   += io.o
obj-y                                   += irq.o
ifneq ($(CONFIG_TEGRA_NVRM),y)
obj-y                                   += clock.o
obj-$(CONFIG_ARCH_TEGRA_2x_SOC)         += tegra2_clocks.o
endif
obj-y                                   += timer.o
obj-y                                   += gpio.o
obj-y					+= suspend.o
obj-$(CONFIG_ARCH_TEGRA_2x_SOC)		+= suspend-t2.o
obj-y					+= pinmux.o
obj-$(CONFIG_ARCH_TEGRA_2x_SOC)		+= pinmux-t2-tables.o
obj-y                                   += delay.o
ifeq ($(CONFIG_SMP),y)
obj-y					+= platsmp.o localtimer.o headsmp.o
obj-$(CONFIG_ARCH_TEGRA_2x_SOC)		+= headsmp-t2.o
endif
obj-$(CONFIG_ARCH_TEGRA_2x_SOC)		+= tegra2_save.o

# gps gpio neo.shin@lge.com 2010/05/24
obj-y					+= tegra_gps.o

obj-$(CONFIG_ARCH_TEGRA_2x_SOC)		+= fuse-cache.o
obj-$(CONFIG_MACH_TEGRA_GENERIC)	+= board-generic.o
obj-$(CONFIG_CPU_FREQ)			+= cpufreq.o
obj-$(CONFIG_CPU_IDLE)			+= cpuidle.o
obj-$(CONFIG_TEGRA_IOVMM)		+= iovmm.o
obj-$(CONFIG_TEGRA_IOVMM_GART)		+= iovmm-gart.o
obj-$(CONFIG_TEGRA_SYSTEM_DMA)		+= dma.o
obj-$(CONFIG_TEGRA_PCI)			+= pci.o
obj-$(CONFIG_TEGRA_PCI)			+= pci-enum.o


ccflags-$(CONFIG_TEGRA_NVOS)		+= -DNV_DEBUG=0
ccflags-$(CONFIG_TEGRA_NVOS)		+= -DNV_OAL=0
ccflags-$(CONFIG_TEGRA_NVOS)		+= -DNV_IS_AVP=0
ccflags-$(CONFIG_TEGRA_NVOS)		+= -DNV_USE_FUSE_CLOCK_ENABLE=0
obj-$(CONFIG_TEGRA_NVOS)		+= nvos_user.o
obj-$(CONFIG_TEGRA_NVOS)		+= nvos/

obj-$(CONFIG_TEGRA_NVRM)		+= board-nvodm.o
obj-$(CONFIG_TEGRA_NVRM)		+= nvrm/
obj-$(CONFIG_TEGRA_NVRM)		+= nvodm/
obj-$(CONFIG_TEGRA_NVRM)		+= odm_kit/
obj-$(CONFIG_TEGRA_NVRM)		+= nvreftrack/
obj-$(CONFIG_TEGRA_NVRM)		+= nvrm_user.o
obj-$(CONFIG_TEGRA_NVRM)		+= clock_nvrm.o
obj-$(CONFIG_TEGRA_NVRM)		+= nvrpc_user.o
obj-$(CONFIG_TEGRA_NVRM)		+= tegra_exports.o
obj-y					+= nvddk/

obj-$(CONFIG_TEGRA_NVEC)		+= nvec_user.o
obj-$(CONFIG_TEGRA_NVEC)		+= nvec/

obj-$(CONFIG_TEGRA_FUSE)		+= sysfs-fuse.o
obj-$(CONFIG_TEGRA_ODM_VIBRATE)		+= vibrate.o
obj-$(CONFIG_TEGRA_AES_USER)		+= nvaes_user.o
obj-$(CONFIG_MACH_VENTANA)		+= board-ventana-wifi.o
#LGE_CHANGE[sunghoon.kim@lge.com] 2010-03-30, debug message beautifier
obj-$(CONFIG_LPRINTK)			+=lprintk.o

#20100602, cs77.ha@lge.com, pmic on/off test
obj-$(CONFIG_STAR_PMIC)			+=pmic.o
