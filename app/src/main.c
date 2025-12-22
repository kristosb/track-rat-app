/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/uart.h>

#include <app_version.h>

#include <zephyr/drivers/sensor.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>

#include <bno055.h> // Required for custom SENSOR_CHAN_*



LOG_MODULE_REGISTER(main, CONFIG_APP_LOG_LEVEL);
//LOG_MODULE_REGISTER(main);
/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   2000
/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

struct sensor_value gyr[3];
static const struct device *const bno055_dev = DEVICE_DT_GET(DT_NODELABEL(bno0550));

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

static void read_gyro_data(const struct device * gyro_dev)
{
	sensor_sample_fetch(gyro_dev);
	sensor_channel_get(gyro_dev, SENSOR_CHAN_GYRO_XYZ, gyr);
}

static void display_gyro_data(void)
{

	printk("pitch %d roll %d \n", gyr[1].val1 , gyr[2].val1*10);
	printk("heading %d \n", gyr[0].val1);
}


static struct sensor_trigger trig_acc_drdy;
static struct sensor_trigger trig_acc_motion;
static struct sensor_trigger trig_acc_high_g;
static struct sensor_trigger trig_gyr_any_motion;
static struct sensor_trigger trig_gyr_high_rate;

static bool trigger_anyno_motion = false;
static bool bno055_fusion = true;

void acc_drdy(const struct device *dev, const struct sensor_trigger *trigger)
{
	printk("BSX data ready!!\n");
}

void acc_motion(const struct device *dev, const struct sensor_trigger *trigger)
{
	if (trigger->type == SENSOR_TRIG_DELTA) {
		printk("ACC any motion interrupt!!\n");
	} else if (trigger->type == SENSOR_TRIG_STATIONARY) {
		printk("ACC no motion interrupt!!\n");
	} else {
		printk("Unknown interrupt!!\n");
	}
}

void acc_high_g(const struct device *dev, const struct sensor_trigger *trigger)
{
	printk("ACC high G interrupt!!\n");
}

void gyr_any_motion(const struct device *dev, const struct sensor_trigger *trigger)
{
	printk("GYR any motion interrupt!!\n");
}

void gyr_high_rate(const struct device *dev, const struct sensor_trigger *trigger)
{
	printk("GYR high RATE interrupt!!\n");
}

int main(void)
{
	int ret;
	bool led_state = true;
	uint32_t dtr = 0;

	
	printk("Track Rat Application %s\n", APP_VERSION_STRING);
	if (!gpio_is_ready_dt(&led)) {
		return 0;
	}

	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return 0;
	}

	// Initialize USB device
	const struct device *const dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));


	// k_msleep(5000);
	// const struct device *gyro_dev = DEVICE_DT_GET(DT_NODELABEL(bno055_l));
	// if (!device_is_ready(gyro_dev)) {
	// 	printk("Device %s is not ready\n", gyro_dev->name);
	// 	return 0;
	// }
	// /* Poll if the DTR flag was set */
	// while (!dtr) {
	// 	uart_line_ctrl_get(dev, UART_LINE_CTRL_DTR, &dtr);
	// 	/* Give CPU resources to low priority threads. */
	// 	k_sleep(K_MSEC(100));
	// }
	// printk("Device %s ready,starting aquisition!\n", gyro_dev->name);
	k_msleep(2000);
	if (!device_is_ready(bno055_dev)) {
		printk("Device %s is not ready\n", bno055_dev->name);
		return 1;
	}
	
	#if Z_DEVICE_DT_FLAGS(DT_NODELABEL(bno0550)) & DEVICE_FLAG_INIT_DEFERRED
		k_sleep(K_MSEC(BNO055_TIMING_STARTUP));
		device_init(bno055_dev);
	#endif

	struct sensor_value config = {
		.val1 = (bno055_fusion) ? BNO055_MODE_NDOF : BNO055_MODE_ACC_MAG_GYRO,
		.val2 = 0,
	};
	sensor_attr_set(bno055_dev, SENSOR_CHAN_ALL, SENSOR_ATTR_CONFIGURATION, &config);
	config.val1 = BNO055_POWER_NORMAL;
	config.val2 = 0;
	sensor_attr_set(bno055_dev, SENSOR_CHAN_ALL, BNO055_SENSOR_ATTR_POWER_MODE, &config);

	// trig_acc_drdy.type = SENSOR_TRIG_DATA_READY;
	// trig_acc_drdy.chan = SENSOR_CHAN_ACCEL_XYZ;
	// sensor_trigger_set(bno055_dev, &trig_acc_drdy, acc_drdy);
/*
	if (trigger_anyno_motion) {
		config.val1 = BNO055_ACC_DURATION_MOTION_ANY;
		config.val2 = 0x02; // Value
		sensor_attr_set(bno055_dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SLOPE_DUR, &config);
		config.val1 = BNO055_ACC_THRESHOLD_MOTION_ANY;
		config.val2 = 0x04; // Value
		sensor_attr_set(bno055_dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SLOPE_TH, &config);
		// trig_acc_motion.type = SENSOR_TRIG_DELTA;
		// trig_acc_motion.chan = SENSOR_CHAN_ACCEL_XYZ;
		// sensor_trigger_set(bno055_dev, &trig_acc_motion, acc_motion);
	} else {
		config.val1 = BNO055_ACC_DURATION_MOTION_NO;
		config.val2 =
			BNO055_IRQ_ACC_SET_MOTION_NO || BNO055_ACC_DURATION_MOTION_SLOWNO_1_SECONDS;
		sensor_attr_set(bno055_dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SLOPE_DUR, &config);
		config.val1 = BNO055_ACC_THRESHOLD_MOTION_NO;
		config.val2 = 0x04; // Value
		sensor_attr_set(bno055_dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SLOPE_TH, &config);
		// trig_acc_motion.type = SENSOR_TRIG_STATIONARY;
		// trig_acc_motion.chan = SENSOR_CHAN_ACCEL_XYZ;
		// sensor_trigger_set(bno055_dev, &trig_acc_motion, acc_motion);
	}

	config.val1 = BNO055_ACC_DURATION_HIGH_G;
	config.val2 = 0x7F; // Value (127 + 1) * 2 = 256 ms
	sensor_attr_set(bno055_dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SLOPE_DUR, &config);
	config.val1 = BNO055_ACC_THRESHOLD_HIGH_G;
	config.val2 = 0x48; // Value
	sensor_attr_set(bno055_dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SLOPE_TH, &config);
	// trig_acc_high_g.type = BNO055_SENSOR_TRIG_HIGH_G;
	// trig_acc_high_g.chan = SENSOR_CHAN_ACCEL_XYZ;
	// sensor_trigger_set(bno055_dev, &trig_acc_high_g, acc_high_g);

	config.val1 = 0x02; // Value
	config.val2 = BNO055_GYR_AWAKE_DURATION_MOTION_ANY_8_SAMPLES;
	sensor_attr_set(bno055_dev, SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_SLOPE_DUR, &config);
	config.val1 = 0x08; // Value
	config.val2 = 0;
	sensor_attr_set(bno055_dev, SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_SLOPE_TH, &config);
	// trig_gyr_any_motion.type = SENSOR_TRIG_DELTA;
	// trig_gyr_any_motion.chan = SENSOR_CHAN_GYRO_XYZ;
	// sensor_trigger_set(bno055_dev, &trig_gyr_any_motion, gyr_any_motion);

	config.val1 = 0x0A; // Value
	config.val2 = 0;
	sensor_attr_set(bno055_dev, SENSOR_CHAN_GYRO_X, SENSOR_ATTR_SLOPE_DUR, &config);
	config.val1 = 0x0A; // Value
	config.val2 = 0;
	sensor_attr_set(bno055_dev, SENSOR_CHAN_GYRO_Y, SENSOR_ATTR_SLOPE_DUR, &config);
	config.val1 = 0x0A; // Value
	config.val2 = 0;
	sensor_attr_set(bno055_dev, SENSOR_CHAN_GYRO_Z, SENSOR_ATTR_SLOPE_DUR, &config);
	config.val1 = 0x01; // Threshold
	config.val2 = 0x00; // Hysteresis
	sensor_attr_set(bno055_dev, SENSOR_CHAN_GYRO_X, SENSOR_ATTR_HYSTERESIS, &config);
	config.val1 = 0x01; // Threshold
	config.val2 = 0x00; // Hysteresis
	sensor_attr_set(bno055_dev, SENSOR_CHAN_GYRO_Y, SENSOR_ATTR_HYSTERESIS, &config);
	config.val1 = 0x01; // Threshold
	config.val2 = 0x00; // Hysteresis
	sensor_attr_set(bno055_dev, SENSOR_CHAN_GYRO_Z, SENSOR_ATTR_HYSTERESIS, &config);
	// trig_gyr_high_rate.type = BNO055_SENSOR_TRIG_HIGH_RATE;
	// trig_gyr_high_rate.chan = SENSOR_CHAN_GYRO_XYZ;
	// sensor_trigger_set(bno055_dev, &trig_gyr_high_rate, gyr_high_rate);

	config.val1 = BNO055_GYR_FILTER_OFF; // Deactive HR Filter
	config.val2 = BNO055_GYR_FILTER_OFF; // Deactive AM Filter
	sensor_attr_set(bno055_dev, SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_FEATURE_MASK, &config);
*/
	while (1) {
		ret = gpio_pin_toggle_dt(&led);
		if (ret < 0) {
			return 0;
		}
		//LOG_INF("Logging to usb!\n");
		led_state = !led_state;
		printk("LED state: %s\n", led_state ? "ON" : "OFF");
		k_msleep(SLEEP_TIME_MS);

		// read_gyro_data(bno055_dev);
		// display_gyro_data();

		//sensor_sample_fetch(bno055_dev);

		// Example for Linear Acceleration and Gravity
		/*if (!bno055_fusion) {
			struct sensor_value acc[3];
			sensor_channel_get(bno055_dev, SENSOR_CHAN_ACCEL_XYZ, acc);

			printk("ACCEL: X(m.s-2)[%d.%06d] Y(m.s-2)[%d.%06d] Z(m.s-2)[%d.%06d]\n",
			       acc[0].val1, acc[0].val2, acc[1].val1, acc[1].val2, acc[2].val1,
			       acc[2].val2);
		} else {
			struct sensor_value lia[3], grav[3], eul[3], quat[4], calib[4];
			sensor_channel_get(bno055_dev, BNO055_SENSOR_CHAN_LINEAR_ACCEL_XYZ, lia);
			sensor_channel_get(bno055_dev, BNO055_SENSOR_CHAN_GRAVITY_XYZ, grav);
			sensor_channel_get(bno055_dev, BNO055_SENSOR_CHAN_EULER_YRP, eul);
			sensor_channel_get(bno055_dev, BNO055_SENSOR_CHAN_QUATERNION_WXYZ, quat);
			sensor_channel_get(bno055_dev, BNO055_SENSOR_CHAN_CALIBRATION_SGAM, calib);

			printk("LINACCEL: X(m.s-2)[%d.%06d] Y(m.s-2)[%d.%06d] Z(m.s-2)[%d.%06d]\n",
			       lia[0].val1, lia[0].val2, lia[1].val1, lia[1].val2, lia[2].val1,
			       lia[2].val2);
			printk("GRAVITY: X(m.s-2)[%d.%06d] Y(m.s-2)[%d.%06d] Z(m.s-2)[%d.%06d]\n",
			       grav[0].val1, grav[0].val2, grav[1].val1, grav[1].val2, grav[2].val1,
			       grav[2].val2);
			printk("EULER: X(rad.s-1)[%d.%06d] Y(rad.s-1)[%d.%06d] "
			       "Z(rad.s-1)[%d.%06d]\n",
			       eul[0].val1, eul[0].val2, eul[1].val1, eul[1].val2, eul[2].val1,
			       eul[2].val2);
			printk("QUATERNION: W[%d.%06d] X[%d.%06d] Y[%d.%06d] Z[%d.%06d]\n",
			       quat[0].val1, quat[0].val2, quat[1].val1, quat[1].val2, quat[2].val1,
			       quat[2].val2, quat[2].val1, quat[2].val2);
			printk("CALIB: SYS[%d] GYR[%d] ACC[%d] MAG[%d]\n", calib[0].val1,
			       calib[1].val1, calib[2].val1, calib[3].val1);
		}*/
		sensor_sample_fetch(bno055_dev);
		struct sensor_value eul[3], quat[4];
		sensor_channel_get(bno055_dev, BNO055_SENSOR_CHAN_EULER_YRP, eul);
		printk("EULER: X(rad.s-1)[%d] Y(rad.s-1)[%d] "
				"Z(rad.s-1)[%d]\n",
				eul[0].val1, eul[1].val1, eul[2].val1);
		sensor_channel_get(bno055_dev, BNO055_SENSOR_CHAN_QUATERNION_WXYZ, quat);
		printk("QUATERNION: W[%d.%06d] X[%d.%06d] Y[%d.%06d] Z[%d.%06d]\n",
			       quat[0].val1, quat[0].val2, quat[1].val1, quat[1].val2, quat[2].val1,
			       quat[2].val2, quat[2].val1, quat[2].val2);

	}

	return 0;
}

