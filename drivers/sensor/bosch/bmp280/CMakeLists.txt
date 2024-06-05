#
# Copyright (c) 2020 Facebook, Inc. and its affiliates
# Copyright (c) 2024 pck
#
# SPDX-License-Identifier: Apache-2.0
#

zephyr_library()
zephyr_library_sources(bmp280.c bmp280_i2c.c bmp280_spi.c)
