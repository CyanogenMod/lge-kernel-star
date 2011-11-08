GCOV_PROFILE := y

# Tegra platform Support
snd-soc-tegra-pcm-objs := tegra_pcm.o
snd-soc-tegra20-spdif-objs := tegra20_spdif.o
snd-soc-tegra-utils-objs += tegra_asoc_utils.o
snd-soc-tegra20-das-objs := tegra20_das.o
snd-soc-tegra20-i2s-objs := tegra20_i2s.o
snd-soc-tegra30-ahub-objs := tegra30_ahub.o
snd-soc-tegra30-i2s-objs := tegra30_i2s.o
snd-soc-tegra30-spdif-objs := tegra30_spdif.o
snd-soc-tegra30-dam-objs := tegra30_dam.o

obj-$(CONFIG_SND_SOC_TEGRA) += snd-soc-tegra-pcm.o
obj-$(CONFIG_SND_SOC_TEGRA) += snd-soc-tegra-utils.o
obj-$(CONFIG_SND_SOC_TEGRA20_DAS) += snd-soc-tegra20-das.o
obj-$(CONFIG_SND_SOC_TEGRA20_I2S) += snd-soc-tegra20-i2s.o
obj-$(CONFIG_SND_SOC_TEGRA30_AHUB) += snd-soc-tegra30-ahub.o
obj-$(CONFIG_SND_SOC_TEGRA30_DAM) += snd-soc-tegra30-dam.o
obj-$(CONFIG_SND_SOC_TEGRA30_I2S) += snd-soc-tegra30-i2s.o
obj-$(CONFIG_SND_SOC_TEGRA20_SPDIF) += snd-soc-tegra20-spdif.o
obj-$(CONFIG_SND_SOC_TEGRA30_SPDIF) += snd-soc-tegra30-spdif.o

# Tegra machine Support
snd-soc-tegra-wm8903-objs := tegra_wm8903.o
snd-soc-tegra-wm8753-objs := tegra_wm8753.o
snd-soc-tegra-max98088-objs := tegra_max98088.o

obj-$(CONFIG_SND_SOC_TEGRA_WM8903) += snd-soc-tegra-wm8903.o
obj-$(CONFIG_SND_SOC_TEGRA_WM8753) += snd-soc-tegra-wm8753.o
obj-$(CONFIG_SND_SOC_TEGRA_MAX98088) += snd-soc-tegra-max98088.o
