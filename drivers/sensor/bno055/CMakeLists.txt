#
# Copyright (c) 2021 Bosch Sensortec GmbH
# Copyright (c) 2022 Nordic Semiconductor ASA
#
# SPDX-License-Identifier: Apache-2.0
#

zephyr_library()

zephyr_library_sources(bno055.c)
zephyr_library_sources_ifdef(CONFIG_BNO055_BUS_I2C bno055_i2c.c)
zephyr_library_sources_ifdef(CONFIG_BNO055_BUS_SPI bno055_spi.c)
zephyr_library_sources_ifdef(CONFIG_BNO055_TRIGGER bno055_trigger.c)
