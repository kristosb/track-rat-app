/*
 * Copyright (c) 2021 Bosch Sensortec GmbH
 * Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT bosch_bno055

#include <zephyr/drivers/sensor.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/logging/log.h>

#include <app/drivers/sensor/bno055.h>

LOG_MODULE_REGISTER(bno055, CONFIG_SENSOR_LOG_LEVEL);

#define BNO055_WR_LEN                           256
#define BNO055_CONFIG_FILE_RETRIES              15
#define BNO055_CONFIG_FILE_POLL_PERIOD_US       10000
#define BNO055_INTER_WRITE_DELAY_US             1000

/*  STRUCTURE DEFINITIONS   */
static struct bno055_t *p_bno055;
struct bno055_t bno055;

static inline int bno055_bus_check(const struct device *dev)
{
	const struct bno055_config *cfg = dev->config;

	return cfg->bus_io->check(&cfg->bus);
}

static inline int bno055_bus_init(const struct device *dev)
{
	const struct bno055_config *cfg = dev->config;

	return cfg->bus_io->init(&cfg->bus);
}

int bno055_reg_read(const struct device *dev, uint8_t reg, uint8_t *data, uint16_t length)
{
	const struct bno055_config *cfg = dev->config;

	return cfg->bus_io->read(&cfg->bus, reg, data, length);
}

int bno055_reg_write(const struct device *dev, uint8_t reg,
		     const uint8_t *data, uint16_t length)
{
	const struct bno055_config *cfg = dev->config;

	return cfg->bus_io->write(&cfg->bus, reg, data, length);
}

int bno055_reg_write_with_delay(const struct device *dev,
				uint8_t reg,
				const uint8_t *data,
				uint16_t length,
				uint32_t delay_us)
{
	int ret = 0;

	ret = bno055_reg_write(dev, reg, data, length);
	if (ret == 0) {
		k_usleep(delay_us);
	}
	return ret;
}

static void channel_euler_convert(struct sensor_value *val, int64_t raw_val,
				 uint16_t range)
{
	val->val1 = (int32_t)(raw_val / BNO055_EULER_DIV_DEG);
	val->val2 = (int32_t)(raw_val * 100 / (int64_t) BNO055_EULER_DIV_DEG) % 100LL;
}

static int bno055_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	int ret;

	struct bno055_euler_t *euler = dev->data;
	/*if (chan != SENSOR_CHAN_ALL) {
		return -ENOTSUP;
	}
	}*/
    uint8_t data_u8[BNO055_EULER_HRP_DATA_SIZE] = {
        BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE, BNO055_INIT_VALUE
    };

	ret = bno055_reg_read(dev, BNO055_EULER_H_LSB_VALUEH_REG, data_u8, BNO055_EULER_HRP_DATA_SIZE);
	if (ret == 0) {
		/* Data h*/
		data_u8[BNO055_SENSOR_DATA_EULER_HRP_H_LSB] = BNO055_GET_BITSLICE(
			data_u8[BNO055_SENSOR_DATA_EULER_HRP_H_LSB],
			BNO055_EULER_H_LSB_VALUEH);
		data_u8[BNO055_SENSOR_DATA_EULER_HRP_H_MSB] = BNO055_GET_BITSLICE(
			data_u8[BNO055_SENSOR_DATA_EULER_HRP_H_MSB],
			BNO055_EULER_H_MSB_VALUEH);
		euler->h =
			(int16_t)((((int32_t)((int8_t)data_u8[BNO055_SENSOR_DATA_EULER_HRP_H_MSB])) << BNO055_SHIFT_EIGHT_BITS) |
					(data_u8[BNO055_SENSOR_DATA_EULER_HRP_H_LSB]));

		/* Data r*/
		data_u8[BNO055_SENSOR_DATA_EULER_HRP_R_LSB] = BNO055_GET_BITSLICE(
			data_u8[BNO055_SENSOR_DATA_EULER_HRP_R_LSB],
			BNO055_EULER_R_LSB_VALUER);
		data_u8[BNO055_SENSOR_DATA_EULER_HRP_R_MSB] = BNO055_GET_BITSLICE(
			data_u8[BNO055_SENSOR_DATA_EULER_HRP_R_MSB],
			BNO055_EULER_R_MSB_VALUER);
		euler->r =
			(int16_t)((((int32_t)((int8_t)data_u8[BNO055_SENSOR_DATA_EULER_HRP_R_MSB])) << BNO055_SHIFT_EIGHT_BITS) |
					(data_u8[BNO055_SENSOR_DATA_EULER_HRP_R_LSB]));

		/* Data p*/
		data_u8[BNO055_SENSOR_DATA_EULER_HRP_P_LSB] = BNO055_GET_BITSLICE(
			data_u8[BNO055_SENSOR_DATA_EULER_HRP_P_LSB],
			BNO055_EULER_P_LSB_VALUEP);
		data_u8[BNO055_SENSOR_DATA_EULER_HRP_P_MSB] = BNO055_GET_BITSLICE(
			data_u8[BNO055_SENSOR_DATA_EULER_HRP_P_MSB],
			BNO055_EULER_P_MSB_VALUEP);
		euler->p =
			(int16_t)((((int32_t)((int8_t)data_u8[BNO055_SENSOR_DATA_EULER_HRP_P_MSB])) << BNO055_SHIFT_EIGHT_BITS) |
					(data_u8[BNO055_SENSOR_DATA_EULER_HRP_P_LSB]));
	} else {
		euler->h = 0;
		euler->r = 0;
		euler->p = 0;
	}
	return ret;
}

static int bno055_channel_get(const struct device *dev, enum sensor_channel chan,
			      struct sensor_value *val)
{
	struct bno055_data *data = dev->data;
	struct bno055_euler_t *euler = dev->data;

	if (chan == SENSOR_CHAN_GYRO_XYZ) {
		channel_euler_convert(&val[0], euler->h, data->gyr_range);
		channel_euler_convert(&val[1], euler->r, data->gyr_range);
		channel_euler_convert(&val[2], euler->p, data->gyr_range);

	} else {
		return -ENOTSUP;
	}
	LOG_INF("Chip ID is (%x)", p_bno055->chip_id);
	return 0;
}

#if defined(CONFIG_BNO055_TRIGGER)

/* ANYMO_1.duration conversion is 20 ms / LSB */
#define ANYMO_1_DURATION_MSEC_TO_LSB(_ms)	\
	BNO055_ANYMO_1_DURATION(_ms / 20)

static int bno055_write_anymo_threshold(const struct device *dev,
					struct sensor_value val)
{
	struct bno055_data *data = dev->data;

	/* this takes configuration in g. */
	if (val.val1 > 0) {
		LOG_DBG("anymo_threshold set to max");
		val.val2 = 1e6;
	}

	/* max = BIT_MASK(10) = 1g => 0.49 mg/LSB */
	uint16_t lsbs = (val.val2 * BNO055_ANYMO_2_THRESHOLD_MASK) / 1e6;

	if (!lsbs) {
		LOG_ERR("Threshold too low!");
		return -EINVAL;
	}

	uint16_t anymo_2 = BNO055_ANYMO_2_THRESHOLD(lsbs)
		| BNO055_ANYMO_2_OUT_CONF_BIT_6;

	data->anymo_2 = anymo_2;
	return 0;
}

static int bno055_write_anymo_duration(const struct device *dev, uint32_t ms)
{
	struct bno055_data *data = dev->data;
	uint16_t val = ANYMO_1_DURATION_MSEC_TO_LSB(ms)
		| BNO055_ANYMO_1_SELECT_XYZ;

	data->anymo_1 = val;
	return 0;
}
#endif /* CONFIG_BNO055_TRIGGER */

static int bno055_attr_set(const struct device *dev, enum sensor_channel chan,
			   enum sensor_attribute attr, const struct sensor_value *val)
{
	int ret = -ENOTSUP;

	if ((chan == SENSOR_CHAN_ACCEL_X) || (chan == SENSOR_CHAN_ACCEL_Y)
	    || (chan == SENSOR_CHAN_ACCEL_Z)
	    || (chan == SENSOR_CHAN_ACCEL_XYZ)) {
		switch (attr) {
		// case SENSOR_ATTR_SAMPLING_FREQUENCY:
		// 	ret = set_accel_odr_osr(dev, val, NULL);
		// 	break;
		// case SENSOR_ATTR_OVERSAMPLING:
		// 	ret = set_accel_odr_osr(dev, NULL, val);
		// 	break;
		// case SENSOR_ATTR_FULL_SCALE:
		// 	ret = set_accel_range(dev, val);
		// 	break;
#if defined(CONFIG_BNO055_TRIGGER)
		case SENSOR_ATTR_SLOPE_DUR:
			return bno055_write_anymo_duration(dev, val->val1);
		case SENSOR_ATTR_SLOPE_TH:
			return bno055_write_anymo_threshold(dev, *val);
#endif
		default:
			ret = -ENOTSUP;
		}
	} else if ((chan == SENSOR_CHAN_GYRO_X) || (chan == SENSOR_CHAN_GYRO_Y)
		   || (chan == SENSOR_CHAN_GYRO_Z)
		   || (chan == SENSOR_CHAN_GYRO_XYZ)) {
		switch (attr) {
		// case SENSOR_ATTR_SAMPLING_FREQUENCY:
		// 	ret = set_gyro_odr_osr(dev, val, NULL);
		// 	break;
		// case SENSOR_ATTR_OVERSAMPLING:
		// 	ret = set_gyro_odr_osr(dev, NULL, val);
		// 	break;
		// case SENSOR_ATTR_FULL_SCALE:
		// 	ret = set_gyro_range(dev, val);
		// 	break;
		default:
			ret = -ENOTSUP;
		}
	}

	return ret;
}

static int bno055_init(const struct device *dev)
{
	int ret;
	
	//uint8_t chip_id;
	uint8_t bno055_page_zero_u8 = BNO055_PAGE_ZERO;
	uint8_t data_u8 = BNO055_INIT_VALUE;
	//Array holding the Software revision id
    uint8_t a_SW_ID_u8[BNO055_REV_ID_SIZE] = { BNO055_INIT_VALUE, BNO055_INIT_VALUE };
	uint8_t adv_pwr_save;
	uint8_t bno055_op_mode_u8 = BNO055_OPERATION_MODE_NDOF;
	uint8_t bno055_euler_mode_u8 = BNO055_EULER_UNIT_DEG;

	/* stuct parameters are assign to bno055*/
    p_bno055 = &bno055;
	k_msleep(300);
	ret = bno055_bus_check(dev);
	if (ret < 0) {
		LOG_ERR("Could not initialize bus");
		return ret;
	}

	ret = bno055_bus_init(dev);
	if (ret != 0) {
		LOG_ERR("Could not initiate bus communication");
		return ret;
	}

	ret = bno055_reg_write(dev, BNO055_PAGE_ID_REG, &bno055_page_zero_u8, BNO055_GEN_READ_WRITE_LENGTH);
	if (ret != 0) {
		LOG_ERR("Could not reset");
		return ret;
	}
	

	ret = bno055_reg_read(dev, BNO055_CHIP_ID_REG, &data_u8, BNO055_GEN_READ_WRITE_LENGTH);
	if (ret != 0) {
		return ret;
	}
	p_bno055->chip_id = data_u8;
	LOG_INF("Chip ID is (%x)", p_bno055->chip_id);//, BNO055_CHIP_ID);
	ret = bno055_reg_read(dev, BNO055_ACCEL_REV_ID_REG, &data_u8, BNO055_GEN_READ_WRITE_LENGTH);
	if (ret != 0) {
		return ret;
	}
	p_bno055->accel_rev_id = data_u8;

	ret = bno055_reg_read(dev, BNO055_MAG_REV_ID_REG, &data_u8, BNO055_GEN_READ_WRITE_LENGTH);
	if (ret != 0) {
		return ret;
	}
	p_bno055->mag_rev_id = data_u8;

	ret = bno055_reg_read(dev, BNO055_GYRO_REV_ID_REG, &data_u8, BNO055_GEN_READ_WRITE_LENGTH);
	if (ret != 0) {
		return ret;
	}
	p_bno055->gyro_rev_id = data_u8;

	ret = bno055_reg_read(dev, BNO055_BL_REV_ID_REG, &data_u8, BNO055_GEN_READ_WRITE_LENGTH);
	if (ret != 0) {
		return ret;
	}
	p_bno055->bl_rev_id = data_u8;

	/* twister error
	ret = bno055_reg_read(dev, BNO055_SW_REV_ID_LSB_REG, &a_SW_ID_u8, BNO055_LSB_MSB_READ_LENGTH);
	if (ret != 0) {
		return ret;
	}*/

    a_SW_ID_u8[BNO055_SW_ID_LSB] = BNO055_GET_BITSLICE(a_SW_ID_u8[BNO055_SW_ID_LSB], BNO055_SW_REV_ID_LSB);
    p_bno055->sw_rev_id =
        (uint16_t)((((uint32_t)((uint8_t)a_SW_ID_u8[BNO055_SW_ID_MSB])) << BNO055_SHIFT_EIGHT_BITS) | (a_SW_ID_u8[BNO055_SW_ID_LSB]));

	ret = bno055_reg_read(dev, BNO055_PAGE_ID_REG, &data_u8, BNO055_GEN_READ_WRITE_LENGTH);
	if (ret != 0) {
		return ret;
	}
	p_bno055->page_id = data_u8;

	adv_pwr_save = BNO055_SET_BITS_POS_0(adv_pwr_save,
					     BNO055_PWR_CONF_ADV_PWR_SAVE,
					     BNO055_PWR_CONF_ADV_PWR_SAVE_DIS);
	ret = bno055_reg_write_with_delay(dev, BNO055_POWER_MODE_REG,
					  &adv_pwr_save, 1,
					  BNO055_INTER_WRITE_DELAY_US);
	if (ret != 0) {
		return ret;
	}
	
	ret = bno055_reg_write(dev, BNO055_OPERATION_MODE_REG, &bno055_op_mode_u8, BNO055_GEN_READ_WRITE_LENGTH);
	if (ret != 0) {
		return ret;
	}

	ret = bno055_reg_write(dev, BNO055_EULER_UNIT_REG, &bno055_euler_mode_u8, BNO055_GEN_READ_WRITE_LENGTH);
	if (ret != 0) {
		return ret;
	}

	return ret;
}

static const struct sensor_driver_api bno055_driver_api = {
	.sample_fetch = bno055_sample_fetch,
	.channel_get = bno055_channel_get,
	.attr_set = bno055_attr_set,
#if defined(CONFIG_BNO055_TRIGGER)
	.trigger_set = bno055_trigger_set,
#endif
};

#if CONFIG_BNO055_TRIGGER
#define BNO055_CONFIG_INT(inst) \
	.int1 = GPIO_DT_SPEC_INST_GET_BY_IDX_OR(inst, irq_gpios, 0, {}),\
	.int2 = GPIO_DT_SPEC_INST_GET_BY_IDX_OR(inst, irq_gpios, 1, {}),
#else
#define BNO055_CONFIG_INT(inst)
#endif

/* Initializes a struct bno055_config for an instance on a SPI bus. */
#define BNO055_CONFIG_SPI(inst)				\
	.bus.spi = SPI_DT_SPEC_INST_GET(		\
		inst, BNO055_SPI_OPERATION, 0),		\
	.bus_io = &bno055_bus_io_spi,

/* Initializes a struct bno055_config for an instance on an I2C bus. */
#define BNO055_CONFIG_I2C(inst)				\
	.bus.i2c = I2C_DT_SPEC_INST_GET(inst),		\
	.bus_io = &bno055_bus_io_i2c,

#define BNO055_CREATE_INST(inst)					\
									\
	static struct bno055_data bno055_drv_##inst;			\
									\
	static const struct bno055_config bno055_config_##inst = {	\
		COND_CODE_1(DT_INST_ON_BUS(inst, spi),			\
			    (BNO055_CONFIG_SPI(inst)),			\
			    (BNO055_CONFIG_I2C(inst)))			\
		BNO055_CONFIG_INT(inst)					\
	};								\
									\
	SENSOR_DEVICE_DT_INST_DEFINE(inst,				\
			      bno055_init,				\
			      NULL,					\
			      &bno055_drv_##inst,			\
			      &bno055_config_##inst,			\
			      POST_KERNEL,				\
			      CONFIG_SENSOR_INIT_PRIORITY,		\
			      &bno055_driver_api);

DT_INST_FOREACH_STATUS_OKAY(BNO055_CREATE_INST);

