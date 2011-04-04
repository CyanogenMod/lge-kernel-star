#
# Makefile for sensor drivers
#
EXTRA_CFLAGS +=-DCONFIG_$(STAR_SENSOR)
obj-$(CONFIG_STAR_GYRO_ACCEL)	+= gyro.o
obj-$(CONFIG_STAR_COMPASS)		+= ami304.o
obj-$(CONFIG_LGE_STAR_SENSOR_VERIFY)		+= lge_sen_verify.o

gyro-y	:= gyro_accel.o gyro_accel_kxtf9.o nvodm_gyro_accel.o \
		  nvodm_accelerometer_accel_kxtf9.o

ami304-y := ami304_compass.o nvodm_compass_ami304.o
lge_sen_verify-y := lge_sensor_verify.o
