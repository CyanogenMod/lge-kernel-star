GCOV_PROFILE := y
obj-y += dc.o
obj-y += rgb.o
obj-y += cpu.o
obj-y += hdmi.o
obj-$(CONFIG_TEGRA_NVHDCP) += nvhdcp.o
obj-y += edid.o
obj-y += nvsd.o
ifeq ($(CONFIG_MACH_BSSQ),y)
obj-y += bssq_dsi.o
else
obj-y += dsi.o
endif
obj-y += dc_sysfs.o
obj-$(CONFIG_TEGRA_OVERLAY) += overlay.o
obj-$(CONFIG_TEGRA_DC_EXTENSIONS) += ext/
