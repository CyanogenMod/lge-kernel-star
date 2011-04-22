#
# Makefile for sensor drivers
#

#obj-$(CONFIG_STAR_GYRO_ACCEL)	+= star_gyro.o
obj-$(CONFIG_STAR_SENSORS) +=	star_sensors.o
obj-$(CONFIG_STAR_GYROSCOPE2)		+= star_gyro.o
obj-$(CONFIG_STAR_ACCELEROMETER2)	+= star_accel.o
obj-$(CONFIG_STAR_COMPASS2)			+= star_compass.o

#star_sensor-y	:= star_sensor.o
star_gyro-y		:= gyro_mpu3050.o nvodm_gyro_accel.o
star_accel-y	:= accel_kxtf9.o nvodm_accelerometer_kxtf9.o
star_compass-y	:= compass_ami304.o nvodm_compass_ami304.o
