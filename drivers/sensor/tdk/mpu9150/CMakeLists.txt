# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources(mpu9150.c mpu9150_i2c.c)
zephyr_library_sources_ifdef(CONFIG_MPU9150_TRIGGER mpu9150_trigger.c)
zephyr_library_sources_ifdef(CONFIG_MPU9150_MAGN_EN ak8975.c)
