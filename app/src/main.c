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

LOG_MODULE_REGISTER(main, CONFIG_APP_LOG_LEVEL);
//LOG_MODULE_REGISTER(main);
/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   2000
/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

struct sensor_value gyr[3];

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

int main(void)
{
	int ret;
	bool led_state = true;
	uint32_t dtr = 0;

	k_msleep(5000);
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

	const struct device *gyro_dev = DEVICE_DT_GET(DT_NODELABEL(bno055_l));
	if (!device_is_ready(gyro_dev)) {
		printk("Device %s is not ready\n", gyro_dev->name);
		return 0;
	}
	// /* Poll if the DTR flag was set */
	// while (!dtr) {
	// 	uart_line_ctrl_get(dev, UART_LINE_CTRL_DTR, &dtr);
	// 	/* Give CPU resources to low priority threads. */
	// 	k_sleep(K_MSEC(100));
	// }
	printk("Device %s ready,starting aquisition!\n", gyro_dev->name);
	while (1) {
		ret = gpio_pin_toggle_dt(&led);
		if (ret < 0) {
			return 0;
		}
		//LOG_INF("Logging to usb!\n");
		led_state = !led_state;
		printk("LED state: %s\n", led_state ? "ON" : "OFF");
		k_msleep(SLEEP_TIME_MS);

		read_gyro_data(gyro_dev);
		display_gyro_data();
	}

	return 0;
}

