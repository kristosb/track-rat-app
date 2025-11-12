/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Bus-specific functionality for BNO055s accessed via I2C.
 */

//#include "bno055.h"
#include <app/drivers/sensor/bno055.h>

static int bno055_bus_check_i2c(const union bno055_bus *bus)
{
	return device_is_ready(bus->i2c.bus) ? 0 : -ENODEV;
}

static int bno055_reg_read_i2c(const union bno055_bus *bus,
			       uint8_t start, uint8_t *data, uint16_t len)
{
	return i2c_burst_read_dt(&bus->i2c, start, data, len);
}

static int bno055_reg_write_i2c(const union bno055_bus *bus, uint8_t start,
				const uint8_t *data, uint16_t len)
{
	return i2c_burst_write_dt(&bus->i2c, start, data, len);
}

static int bno055_bus_init_i2c(const union bno055_bus *bus)
{
	/* I2C is used by default
	 */
	return 0;
}

const struct bno055_bus_io bno055_bus_io_i2c = {
	.check = bno055_bus_check_i2c,
	.read = bno055_reg_read_i2c,
	.write = bno055_reg_write_i2c,
	.init = bno055_bus_init_i2c,
};
