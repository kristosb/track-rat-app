/*
 * Copyright (c) 2021 Bosch Sensortec GmbH
 * Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_BNO055_BNO055_H_
#define ZEPHYR_DRIVERS_SENSOR_BNO055_BNO055_H_

#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>
#include <zephyr/types.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>

// BNO055
#define BNO055_SYS_TRIGGER        0x3F
//#define BNO055_PAGE_ID_ADDR 	  0x07

/*Euler data registers*/
#define BNO055_EULER_H_LSB_ADDR             (0X1A)
#define BNO055_EULER_H_MSB_ADDR             (0X1B)

#define BNO055_EULER_R_LSB_ADDR             (0X1C)
#define BNO055_EULER_R_MSB_ADDR             (0X1D)

#define BNO055_EULER_P_LSB_ADDR             (0X1E)
#define BNO055_EULER_P_MSB_ADDR             (0X1F)

/* Euler unit*/
#define BNO055_EULER_UNIT_DEG                      (0x00)
#define BNO055_EULER_UNIT_RAD                      (0x01)

/*Euler division factor*/
#define BNO055_EULER_DIV_DEG                       (16.0)
#define BNO055_EULER_DIV_RAD                       (900.0)

/****************************************************/
/**\name    ARRAY SIZE DEFINITIONS      */
/***************************************************/
#define BNO055_EULER_DATA_SIZE                     (2)
#define BNO055_EULER_HRP_DATA_SIZE                 (6)

/*ARRAY INDEX DEFINITIONS*/
#define BNO055_SENSOR_DATA_EULER_LSB               (0)
#define BNO055_SENSOR_DATA_EULER_MSB               (1)

#define BNO055_SENSOR_DATA_EULER_HRP_H_LSB         (0)
#define BNO055_SENSOR_DATA_EULER_HRP_H_MSB         (1)
#define BNO055_SENSOR_DATA_EULER_HRP_R_LSB         (2)
#define BNO055_SENSOR_DATA_EULER_HRP_R_MSB         (3)
#define BNO055_SENSOR_DATA_EULER_HRP_P_LSB         (4)
#define BNO055_SENSOR_DATA_EULER_HRP_P_MSB         (5)

/* Euler data HEADING-LSB register*/
#define BNO055_EULER_H_LSB_VALUEH_POS             (0)
#define BNO055_EULER_H_LSB_VALUEH_MSK             (0xFF)
#define BNO055_EULER_H_LSB_VALUEH_LEN             (8)
#define BNO055_EULER_H_LSB_VALUEH_REG             BNO055_EULER_H_LSB_ADDR

/* Euler data HEADING-MSB register*/
#define BNO055_EULER_H_MSB_VALUEH_POS             (0)
#define BNO055_EULER_H_MSB_VALUEH_MSK             (0xFF)
#define BNO055_EULER_H_MSB_VALUEH_LEN             (8)
#define BNO055_EULER_H_MSB_VALUEH_REG             BNO055_EULER_H_MSB_ADDR

/* Euler data ROLL-LSB register*/
#define BNO055_EULER_R_LSB_VALUER_POS             (0)
#define BNO055_EULER_R_LSB_VALUER_MSK             (0xFF)
#define BNO055_EULER_R_LSB_VALUER_LEN             (8)
#define BNO055_EULER_R_LSB_VALUER_REG             BNO055_EULER_R_LSB_ADDR

/* Euler data ROLL-MSB register*/
#define BNO055_EULER_R_MSB_VALUER_POS             (0)
#define BNO055_EULER_R_MSB_VALUER_MSK             (0xFF)
#define BNO055_EULER_R_MSB_VALUER_LEN             (8)
#define BNO055_EULER_R_MSB_VALUER_REG             BNO055_EULER_R_MSB_ADDR

/* Euler data PITCH-LSB register*/
#define BNO055_EULER_P_LSB_VALUEP_POS             (0)
#define BNO055_EULER_P_LSB_VALUEP_MSK             (0xFF)
#define BNO055_EULER_P_LSB_VALUEP_LEN             (8)
#define BNO055_EULER_P_LSB_VALUEP_REG             BNO055_EULER_P_LSB_ADDR

/* Euler data HEADING-MSB register*/
#define BNO055_EULER_P_MSB_VALUEP_POS             (0)
#define BNO055_EULER_P_MSB_VALUEP_MSK             (0xFF)
#define BNO055_EULER_P_MSB_VALUEP_LEN             (8)
#define BNO055_EULER_P_MSB_VALUEP_REG             BNO055_EULER_P_MSB_ADDR

/* Unit selection register*/
#define BNO055_UNIT_SEL_ADDR                (0X3B)
#define BNO055_DATA_SELECT_ADDR             (0X3C)
/* Euler_Unit register*/
#define BNO055_EULER_UNIT_POS                     (2)
#define BNO055_EULER_UNIT_MSK                     (0X04)
#define BNO055_EULER_UNIT_LEN                     (1)
#define BNO055_EULER_UNIT_REG                     BNO055_UNIT_SEL_ADDR

/*Euler division factor*/
#define BNO055_EULER_DIV_DEG                       (16.0)


struct bno055_euler_t
{
    int16_t h; /**< Euler h data */
    int16_t r; /**< Euler r data */
    int16_t p; /**< Euler p data */
};

/*ARRAY INDEX DEFINITIONS*/
#define BNO055_SW_ID_LSB                           (0)
#define BNO055_SW_ID_MSB                           (1)
#define BNO055_SENSOR_DATA_LSB                     (0)
#define BNO055_SENSOR_DATA_MSB                     (1)
#define BNO055_SENSOR_DATA_EULER_LSB               (0)
#define BNO055_SENSOR_DATA_EULER_MSB               (1)
#define BNO055_SENSOR_DATA_QUATERNION_LSB          (0)
#define BNO055_SENSOR_DATA_QUATERNION_MSB          (1)

#define  BNO055_LSB_MSB_READ_LENGTH                (2)
#define BNO055_CHIP_ID_ADDR                 (0x00)
#define BNO055_CHIP_ID_REG                        BNO055_CHIP_ID_ADDR
#define  BNO055_INIT_VALUE                         (0)
#define  BNO055_GEN_READ_WRITE_LENGTH              (1)
#define  BNO055_SHIFT_EIGHT_BITS                   (8)
#define BNO055_REV_ID_SIZE                         (2)
#define BNO055_OPERATION_MODE_NDOF                 (0X0C)
#define BNO055_SW_REV_ID_LSB_ADDR           (0x04)
#define BNO055_SW_REV_ID_MSB_ADDR           (0x05)
#define BNO055_BL_REV_ID_ADDR               (0X06)




#define BNO055_BL_REV_ID_REG                      BNO055_BL_REV_ID_ADDR

/*Page id*/
#define BNO055_PAGE_ID_ADDR                 	(0X07)
#define BNO055_PAGE_ID_POS                        (0)
#define BNO055_PAGE_ID_MSK                        (0xFF)
#define BNO055_PAGE_ID_LEN                        (8)
#define BNO055_PAGE_ID_REG                        BNO055_PAGE_ID_ADDR

/* Mode registers*/
#define BNO055_OPR_MODE_ADDR                (0X3D)
#define BNO055_PWR_MODE_ADDR                (0X3E)
#define BNO055_OPERATION_MODE_REG                 BNO055_OPR_MODE_ADDR
#define BNO055_POWER_MODE_REG                     BNO055_PWR_MODE_ADDR

#define BNO055_ACCEL_REV_ID_ADDR            (0x01)
/* Accel revision id*/
#define BNO055_ACCEL_REV_ID_POS                   (0)
#define BNO055_ACCEL_REV_ID_MSK                   (0xFF)
#define BNO055_ACCEL_REV_ID_LEN                   (8)
#define BNO055_ACCEL_REV_ID_REG                   BNO055_ACCEL_REV_ID_ADDR

/* Mag revision id*/
#define BNO055_MAG_REV_ID_ADDR              (0x02)
#define BNO055_MAG_REV_ID_POS                     (0)
#define BNO055_MAG_REV_ID_MSK                     (0xFF)
#define BNO055_MAG_REV_ID_LEN                     (8)
#define BNO055_MAG_REV_ID_REG                     BNO055_MAG_REV_ID_ADDR

#define BNO055_GYRO_REV_ID_ADDR             (0x03)
/* Gyro revision id*/
#define BNO055_GYRO_REV_ID_POS                    (0)
#define BNO055_GYRO_REV_ID_MSK                    (0xFF)
#define BNO055_GYRO_REV_ID_LEN                    (8)
#define BNO055_GYRO_REV_ID_REG  				  BNO055_GYRO_REV_ID_ADDR

/*Software revision id LSB*/
#define BNO055_SW_REV_ID_LSB_POS                  (0)
#define BNO055_SW_REV_ID_LSB_MSK                  (0xFF)
#define BNO055_SW_REV_ID_LSB_LEN                  (8)
#define BNO055_SW_REV_ID_LSB_REG                  BNO055_SW_REV_ID_LSB_ADDR

/*Software revision id MSB*/
#define BNO055_SW_REV_ID_MSB_POS                  (0)
#define BNO055_SW_REV_ID_MSB_MSK                  (0xFF)
#define BNO055_SW_REV_ID_MSB_LEN                  (8)
#define BNO055_SW_REV_ID_MSB_REG                  BNO055_SW_REV_ID_MSB_ADDR



/*************************************************/
/**\name GET AND SET BITSLICE FUNCTIONS    */
/*************************************************/
#define BNO055_GET_BITSLICE(regvar, bitname) \
    ((regvar & bitname##_MSK) >> bitname##_POS)

#define BNO055_SET_BITSLICE(regvar, bitname, val) \
    ((regvar & ~bitname##_MSK) | ((val << bitname##_POS) & bitname##_MSK))



/*!
 *  @brief bno055 struct
 */
struct bno055_t
{
    uint8_t chip_id; /**< chip_id of bno055 */
    uint16_t sw_rev_id; /**< software revision id of bno055 */
    uint8_t page_id; /**< page_id of bno055 */
    uint8_t accel_rev_id; /**< accel revision id of bno055 */
    uint8_t mag_rev_id; /**< mag revision id of bno055 */
    uint8_t gyro_rev_id; /**< gyro revision id of bno055 */
    uint8_t bl_rev_id; /**< boot loader revision id of bno055 */
    uint8_t dev_addr; /**< i2c device address of bno055 */
};

//////////////////BNO055//////////////////////////



/* Page ID */
#define BNO055_PAGE_ZERO           0X00
#define BNO055_PAGE_ONE            0X01

#define BNO055_REG_CHIP_ID         0x00
#define BNO055_REG_ERROR           0x02
#define BNO055_REG_STATUS          0x03
#define BNO055_REG_AUX_X_LSB       0x04
#define BNO055_REG_ACC_X_LSB       0x0C
#define BNO055_REG_GYR_X_LSB       0x12
#define BNO055_REG_SENSORTIME_0    0x18
#define BNO055_REG_EVENT           0x1B
#define BNO055_REG_INT_STATUS_0    0x1C
#define BNO055_REG_SC_OUT_0        0x1E
#define BNO055_REG_WR_GEST_ACT     0x20
#define BNO055_REG_INTERNAL_STATUS 0x21
#define BNO055_REG_TEMPERATURE_0   0x22
#define BNO055_REG_FIFO_LENGTH_0   0x24
#define BNO055_REG_FIFO_DATA       0x26
#define BNO055_REG_FEAT_PAGE       0x2F
#define BNO055_REG_FEATURES_0      0x30
#define BNO055_REG_ACC_CONF        0x40
#define BNO055_REG_ACC_RANGE       0x41
#define BNO055_REG_GYR_CONF        0x42
#define BNO055_REG_GYR_RANGE       0x43
#define BNO055_REG_AUX_CONF        0x44
#define BNO055_REG_FIFO_DOWNS      0x45
#define BNO055_REG_FIFO_WTM_0      0x46
#define BNO055_REG_FIFO_CONFIG_0   0x48
#define BNO055_REG_SATURATION      0x4A
#define BNO055_REG_AUX_DEV_ID      0x4B
#define BNO055_REG_AUX_IF_CONF     0x4C
#define BNO055_REG_AUX_RD_ADDR     0x4D
#define BNO055_REG_AUX_WR_ADDR     0x4E
#define BNO055_REG_AUX_WR_DATA     0x4F
#define BNO055_REG_ERR_REG_MSK     0x52
#define BNO055_REG_INT1_IO_CTRL    0x53
#define BNO055_REG_INT2_IO_CTRL    0x54
#define BNO055_REG_INT_LATCH       0x55
#define BNO055_REG_INT1_MAP_FEAT   0x56
#define BNO055_REG_INT2_MAP_FEAT   0x57
#define BNO055_REG_INT_MAP_DATA    0x58
#define BNO055_REG_INIT_CTRL       0x59
#define BNO055_REG_INIT_ADDR_0     0x5B
#define BNO055_REG_INIT_DATA       0x5E
#define BNO055_REG_INTERNAL_ERROR  0x5F
#define BNO055_REG_AUX_IF_TRIM     0x68
#define BNO055_REG_GYR_CRT_CONF    0x69
#define BNO055_REG_NVM_CONF        0x6A
#define BNO055_REG_IF_CONF         0x6B
#define BNO055_REG_DRV             0x6C
#define BNO055_REG_ACC_SELF_TEST   0x6D
#define BNO055_REG_GYR_SELF_TEST   0x6E
#define BNO055_REG_NV_CONF         0x70
#define BNO055_REG_OFFSET_0        0x71
#define BNO055_REG_PWR_MODE        0x3E
#define BNO055_REG_PWR_CTRL        0x7D
#define BNO055_REG_CMD             0x7E
#define BNO055_REG_MASK            GENMASK(6, 0)

#define BNO055_ANYMO_1_DURATION_POS	0
#define BNO055_ANYMO_1_DURATION_MASK	BIT_MASK(12)
#define BNO055_ANYMO_1_DURATION(n)	((n) << BNO055_ANYMO_1_DURATION_POS)
#define BNO055_ANYMO_1_SELECT_X		BIT(13)
#define BNO055_ANYMO_1_SELECT_Y		BIT(14)
#define BNO055_ANYMO_1_SELECT_Z		BIT(15)
#define BNO055_ANYMO_1_SELECT_XYZ	(BNO055_ANYMO_1_SELECT_X | \
					 BNO055_ANYMO_1_SELECT_Y | \
					 BNO055_ANYMO_1_SELECT_Y)
#define BNO055_ANYMO_2_THRESHOLD_POS	0
#define BNO055_ANYMO_2_THRESHOLD_MASK	BIT_MASK(10)
#define BNO055_ANYMO_2_THRESHOLD(n)	((n) << BNO055_ANYMO_2_THRESHOLD_POS)
#define BNO055_ANYMO_2_OUT_CONF_POS	11
#define BNO055_ANYMO_2_OUT_CONF_MASK	(BIT(11) | BIT(12) | BIT(13) | BIT(14))
#define BNO055_ANYMO_2_ENABLE		BIT(15)
#define BNO055_ANYMO_2_OUT_CONF_OFF	(0x00 << BNO055_ANYMO_2_OUT_CONF_POS)
#define BNO055_ANYMO_2_OUT_CONF_BIT_0	(0x01 << BNO055_ANYMO_2_OUT_CONF_POS)
#define BNO055_ANYMO_2_OUT_CONF_BIT_1	(0x02 << BNO055_ANYMO_2_OUT_CONF_POS)
#define BNO055_ANYMO_2_OUT_CONF_BIT_2	(0x03 << BNO055_ANYMO_2_OUT_CONF_POS)
#define BNO055_ANYMO_2_OUT_CONF_BIT_3	(0x04 << BNO055_ANYMO_2_OUT_CONF_POS)
#define BNO055_ANYMO_2_OUT_CONF_BIT_4	(0x05 << BNO055_ANYMO_2_OUT_CONF_POS)
#define BNO055_ANYMO_2_OUT_CONF_BIT_5	(0x06 << BNO055_ANYMO_2_OUT_CONF_POS)
#define BNO055_ANYMO_2_OUT_CONF_BIT_6	(0x07 << BNO055_ANYMO_2_OUT_CONF_POS)
#define BNO055_ANYMO_2_OUT_CONF_BIT_8	(0x08 << BNO055_ANYMO_2_OUT_CONF_POS)

#define BNO055_INT_IO_CTRL_LVL		BIT(1) /* Output level (0 = active low, 1 = active high) */
#define BNO055_INT_IO_CTRL_OD		BIT(2) /* Open-drain (0 = push-pull, 1 = open-drain)*/
#define BNO055_INT_IO_CTRL_OUTPUT_EN	BIT(3) /* Output enabled */
#define BNO055_INT_IO_CTRL_INPUT_EN	BIT(4) /* Input enabled */

/* Applies to INT1_MAP_FEAT, INT2_MAP_FEAT, INT_STATUS_0 */
#define BNO055_INT_MAP_SIG_MOTION        BIT(0)
#define BNO055_INT_MAP_STEP_COUNTER      BIT(1)
#define BNO055_INT_MAP_ACTIVITY          BIT(2)
#define BNO055_INT_MAP_WRIST_WEAR_WAKEUP BIT(3)
#define BNO055_INT_MAP_WRIST_GESTURE     BIT(4)
#define BNO055_INT_MAP_NO_MOTION         BIT(5)
#define BNO055_INT_MAP_ANY_MOTION        BIT(6)

#define BNO055_INT_MAP_DATA_FFULL_INT1		BIT(0)
#define BNO055_INT_MAP_DATA_FWM_INT1		BIT(1)
#define BNO055_INT_MAP_DATA_DRDY_INT1		BIT(2)
#define BNO055_INT_MAP_DATA_ERR_INT1		BIT(3)
#define BNO055_INT_MAP_DATA_FFULL_INT2		BIT(4)
#define BNO055_INT_MAP_DATA_FWM_INT2		BIT(5)
#define BNO055_INT_MAP_DATA_DRDY_INT2		BIT(6)
#define BNO055_INT_MAP_DATA_ERR_INT2		BIT(7)

#define BNO055_INT_STATUS_ANY_MOTION		BIT(6)

#define BNO055_CHIP_ID 0xA0

#define BNO055_CMD_G_TRIGGER  0x02
#define BNO055_CMD_USR_GAIN   0x03
#define BNO055_CMD_NVM_PROG   0xA0
#define BNO055_CMD_FIFO_FLUSH OxB0
#define BNO055_CMD_SOFT_RESET 0x20

#define BNO055_POWER_ON_TIME                500
#define BNO055_SOFT_RESET_TIME              600000
#define BNO055_ACC_SUS_TO_NOR_START_UP_TIME 2000
#define BNO055_GYR_SUS_TO_NOR_START_UP_TIME 45000
#define BNO055_GYR_FAST_START_UP_TIME       2000
#define BNO055_TRANSC_DELAY_SUSPEND         450
#define BNO055_TRANSC_DELAY_NORMAL          2

#define BNO055_PREPARE_CONFIG_LOAD  0x00
#define BNO055_COMPLETE_CONFIG_LOAD 0x01

#define BNO055_INST_MESSAGE_MSK        0x0F
#define BNO055_INST_MESSAGE_NOT_INIT   0x00
#define BNO055_INST_MESSAGE_INIT_OK    0x01
#define BNO055_INST_MESSAGE_INIT_ERR   0x02
#define BNO055_INST_MESSAGE_DRV_ERR    0x03
#define BNO055_INST_MESSAGE_SNS_STOP   0x04
#define BNO055_INST_MESSAGE_NVM_ERR    0x05
#define BNO055_INST_MESSAGE_STRTUP_ERR 0x06
#define BNO055_INST_MESSAGE_COMPAT_ERR 0x07

#define BNO055_INST_AXES_REMAP_ERROR 0x20
#define BNO055_INST_ODR_50HZ_ERROR   0x40

#define BNO055_PWR_CONF_ADV_PWR_SAVE_MSK 0x01
#define BNO055_PWR_CONF_ADV_PWR_SAVE_EN  0x01
#define BNO055_PWR_CONF_ADV_PWR_SAVE_DIS 0x00

#define BNO055_PWR_CONF_FIFO_SELF_WKUP_MSK 0x02
#define BNO055_PWR_CONF_FIFO_SELF_WKUP_POS 0x01
#define BNO055_PWR_CONF_FIFO_SELF_WKUP_EN  0x01
#define BNO055_PWR_CONF_FIFO_SELF_WKUP_DIS 0x00

#define BNO055_PWR_CONF_FUP_EN_MSK 0x04
#define BNO055_PWR_CONF_FUP_EN_POS 0x02
#define BNO055_PWR_CONF_FUP_EN     0x01
#define BNO055_PWR_CONF_FUP_DIS    0x00

#define BNO055_PWR_CTRL_MSK     0x0F
#define BNO055_PWR_CTRL_AUX_EN  0x01
#define BNO055_PWR_CTRL_GYR_EN  0x02
#define BNO055_PWR_CTRL_ACC_EN  0x04
#define BNO055_PWR_CTRL_TEMP_EN 0x08

#define BNO055_ACC_ODR_MSK      0x0F
#define BNO055_ACC_ODR_25D32_HZ 0x01
#define BNO055_ACC_ODR_25D16_HZ 0x02
#define BNO055_ACC_ODR_25D8_HZ  0x03
#define BNO055_ACC_ODR_25D4_HZ  0x04
#define BNO055_ACC_ODR_25D2_HZ  0x05
#define BNO055_ACC_ODR_25_HZ    0x06
#define BNO055_ACC_ODR_50_HZ    0x07
#define BNO055_ACC_ODR_100_HZ   0x08
#define BNO055_ACC_ODR_200_HZ   0x09
#define BNO055_ACC_ODR_400_HZ   0x0A
#define BNO055_ACC_ODR_800_HZ   0x0B
#define BNO055_ACC_ODR_1600_HZ  0x0C

#define BNO055_ACC_BWP_MSK        0x30
#define BNO055_ACC_BWP_POS        4
#define BNO055_ACC_BWP_OSR4_AVG1  0x00
#define BNO055_ACC_BWP_OSR2_AVG2  0x01
#define BNO055_ACC_BWP_NORM_AVG4  0x02
#define BNO055_ACC_BWP_CIC_AVG8   0x03
#define BNO055_ACC_BWP_RES_AVG16  0x04
#define BNO055_ACC_BWP_RES_AVG32  0x05
#define BNO055_ACC_BWP_RES_AVG64  0x06
#define BNO055_ACC_BWP_RES_AVG128 0x07

#define BNO055_ACC_FILT_MSK      0x80
#define BNO055_ACC_FILT_POS      7
#define BNO055_ACC_FILT_PWR_OPT  0x00
#define BNO055_ACC_FILT_PERF_OPT 0x01

#define BNO055_ACC_RANGE_MSK 0x03
#define BNO055_ACC_RANGE_2G  0x00
#define BNO055_ACC_RANGE_4G  0x01
#define BNO055_ACC_RANGE_8G  0x02
#define BNO055_ACC_RANGE_16G 0x03

#define BNO055_GYR_ODR_MSK     0x0F
#define BNO055_GYR_ODR_25_HZ   0x06
#define BNO055_GYR_ODR_50_HZ   0x07
#define BNO055_GYR_ODR_100_HZ  0x08
#define BNO055_GYR_ODR_200_HZ  0x09
#define BNO055_GYR_ODR_400_HZ  0x0A
#define BNO055_GYR_ODR_800_HZ  0x0B
#define BNO055_GYR_ODR_1600_HZ 0x0C
#define BNO055_GYR_ODR_3200_HZ 0x0D

#define BNO055_GYR_BWP_MSK  0x30
#define BNO055_GYR_BWP_POS  4
#define BNO055_GYR_BWP_OSR4 0x00
#define BNO055_GYR_BWP_OSR2 0x01
#define BNO055_GYR_BWP_NORM 0x02

#define BNO055_GYR_FILT_NOISE_MSK      0x40
#define BNO055_GYR_FILT_NOISE_POS      6
#define BNO055_GYR_FILT_NOISE_PWR      0x00
#define BNO055_GYR_FILT_NOISE_PERF     0x01

#define BNO055_GYR_FILT_MSK      0x80
#define BNO055_GYR_FILT_POS      7
#define BNO055_GYR_FILT_PWR_OPT  0x00
#define BNO055_GYR_FILT_PERF_OPT 0x01

#define BNO055_GYR_RANGE_MSK     0x07
#define BNO055_GYR_RANGE_2000DPS 0x00
#define BNO055_GYR_RANGE_1000DPS 0x01
#define BNO055_GYR_RANGE_500DPS  0x02
#define BNO055_GYR_RANGE_250DPS  0x03
#define BNO055_GYR_RANGE_125DPS  0x04

#define BNO055_GYR_OIS_RANGE_MSK     0x80
#define BNO055_GYR_OIS_RANGE_POS     3
#define BNO055_GYR_OIS_RANGE_250DPS  0x00
#define BNO055_GYR_OIS_RANGE_2000DPS 0x01

#define BNO055_SET_BITS(reg_data, bitname, data)		  \
	((reg_data & ~(bitname##_MSK)) | ((data << bitname##_POS) \
					  & bitname##_MSK))
#define BNO055_SET_BITS_POS_0(reg_data, bitname, data) \
	((reg_data & ~(bitname##_MSK)) | (data & bitname##_MSK))

struct bno055_data {
	int16_t ax, ay, az, gx, gy, gz;
	uint8_t acc_range, acc_odr, gyr_odr;
	uint16_t gyr_range;

#if CONFIG_BNO055_TRIGGER
	const struct device *dev;
	struct k_mutex trigger_mutex;
	sensor_trigger_handler_t motion_handler;
	const struct sensor_trigger *motion_trigger;
	sensor_trigger_handler_t drdy_handler;
	const struct sensor_trigger *drdy_trigger;
	struct gpio_callback int1_cb;
	struct gpio_callback int2_cb;
	atomic_t int_flags;
	uint16_t anymo_1;
	uint16_t anymo_2;

#if CONFIG_BNO055_TRIGGER_OWN_THREAD
	struct k_sem trig_sem;

	K_KERNEL_STACK_MEMBER(thread_stack, CONFIG_BNO055_THREAD_STACK_SIZE);
	struct k_thread thread;

#elif CONFIG_BNO055_TRIGGER_GLOBAL_THREAD
	struct k_work trig_work;
#endif
#endif /* CONFIG_BNO055_TRIGGER */
};

struct bno055_feature_reg {
	/* Which feature page the register resides in */
	uint8_t page;
	uint8_t addr;
};

struct bno055_feature_config {
	const char *name;
	const uint8_t *config_file;
	size_t config_file_len;
	struct bno055_feature_reg *anymo_1;
	struct bno055_feature_reg *anymo_2;
};

union bno055_bus {
#if CONFIG_BNO055_BUS_SPI
	struct spi_dt_spec spi;
#endif
#if CONFIG_BNO055_BUS_I2C
	struct i2c_dt_spec i2c;
#endif
};

typedef int (*bno055_bus_check_fn)(const union bno055_bus *bus);
typedef int (*bno055_bus_init_fn)(const union bno055_bus *bus);
typedef int (*bno055_reg_read_fn)(const union bno055_bus *bus,
				  uint8_t start,
				  uint8_t *data,
				  uint16_t len);
typedef int (*bno055_reg_write_fn)(const union bno055_bus *bus,
				   uint8_t start,
				   const uint8_t *data,
				   uint16_t len);

struct bno055_bus_io {
	bno055_bus_check_fn check;
	bno055_reg_read_fn read;
	bno055_reg_write_fn write;
	bno055_bus_init_fn init;
};

struct bno055_config {
	union bno055_bus bus;
	const struct bno055_bus_io *bus_io;
	const struct bno055_feature_config *feature;
#if CONFIG_BNO055_TRIGGER
	struct gpio_dt_spec int1;
	struct gpio_dt_spec int2;
#endif
};

#if CONFIG_BNO055_BUS_SPI
#define BNO055_SPI_OPERATION (SPI_WORD_SET(8) | SPI_TRANSFER_MSB)
#define BNO055_SPI_ACC_DELAY_US 2
extern const struct bno055_bus_io bno055_bus_io_spi;
#endif

#if CONFIG_BNO055_BUS_I2C
extern const struct bno055_bus_io bno055_bus_io_i2c;
#endif

int bno055_reg_read(const struct device *dev, uint8_t reg, uint8_t *data, uint16_t length);

int bno055_reg_write(const struct device *dev, uint8_t reg,
		     const uint8_t *data, uint16_t length);

int bno055_reg_write_with_delay(const struct device *dev,
				uint8_t reg,
				const uint8_t *data,
				uint16_t length,
				uint32_t delay_us);

#ifdef CONFIG_BNO055_TRIGGER
int bno055_trigger_set(const struct device *dev,
		       const struct sensor_trigger *trig,
		       sensor_trigger_handler_t handler);

int bno055_init_interrupts(const struct device *dev);
#endif

#endif /* ZEPHYR_DRIVERS_SENSOR_BNO055_BNO055_H_ */
