
IMU_NAVGO_CFLAGS  = -DUSE_IMU
IMU_NAVGO_CFLAGS += -DIMU_TYPE_H=\"boards/navgo/imu_navgo.h\"

IMU_NAVGO_SRCS    = $(SRC_SUBSYSTEMS)/imu.c             \
                    $(SRC_BOARD)/imu_navgo.c

IMU_NAVGO_I2C_DEV=i2c1
IMU_NAVGO_CFLAGS += -DUSE_I2C
IMU_NAVGO_CFLAGS += -DUSE_I2C1 -DI2C1_SCLL=25 -DI2C1_SCLH=25

IMU_NAVGO_CFLAGS += -DIMU_NAVGO_I2C_DEV=$(IMU_NAVGO_I2C_DEV)
IMU_NAVGO_SRCS += peripherals/itg3200.c
IMU_NAVGO_SRCS += peripherals/adxl345_i2c.c
IMU_NAVGO_SRCS += peripherals/hmc58xx.c

# with default NAVGO_GYRO_SMPLRT_DIV (gyro output 500Hz)
# the AHRS_PROPAGATE_FREQUENCY needs to be adjusted accordingly
AHRS_PROPAGATE_FREQUENCY ?= 500
AHRS_CORRECT_FREQUENCY ?= 500
ap.CFLAGS += -DAHRS_PROPAGATE_FREQUENCY=$(AHRS_PROPAGATE_FREQUENCY)
ap.CFLAGS += -DAHRS_CORRECT_FREQUENCY=$(AHRS_CORRECT_FREQUENCY)

ap.CFLAGS += $(IMU_NAVGO_CFLAGS)
ap.srcs   += $(IMU_NAVGO_SRCS)

#
# Simulator
#
include $(CFG_SHARED)/imu_nps.makefile
