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

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>

LOG_MODULE_REGISTER(main, CONFIG_APP_LOG_LEVEL);

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   2000
/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

struct sensor_value gyr[3];

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

static uint8_t led_state = false;

// Service and Characteristics UUIDs
static struct bt_uuid_128 led_state_char_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x9c85a726, 0xb7f1, 0x11ec, 0xb909, 0x0242ac120002));

static const struct bt_uuid_128 vnd_long_uuid = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef3));

#define LED_SERVICE_UUID_VAL \
  BT_UUID_128_ENCODE(0xf7547938, 0x68ba, 0x11ec, 0x90d6, 0x0242ac120003)

static struct bt_uuid_128 led_svc_uuid = BT_UUID_INIT_128(LED_SERVICE_UUID_VAL);


// Advertisement Data

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, LED_SERVICE_UUID_VAL),
};


// GAP callbacks

static void connected(struct bt_conn *conn, uint8_t err) {
  if (err) {
    printk("Connection failed (err 0x%02x)\n", err);
  } else {
    printk("Connected\n");
  }
}

static void disconnected(struct bt_conn *conn, uint8_t reason) {
  printk("Disconnected (reason 0x%02x)\n", reason);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = connected,
    .disconnected = disconnected,
};

// GATT Access Callbacks

static ssize_t read_led_state(struct bt_conn *conn,
                              const struct bt_gatt_attr *attr, void *buf,
                              uint16_t len, uint16_t offset) {
  const uint8_t *value = attr->user_data;
  printk("Value 0x%x read.\n", *value);
  return bt_gatt_attr_read(conn, attr, buf, len, offset, value, sizeof(*value));
}

static ssize_t write_led_state(struct bt_conn *conn,
                               const struct bt_gatt_attr *attr, const void *buf,
                               uint16_t len, uint16_t offset, uint8_t flags) {
  uint8_t *value = attr->user_data;
  *value = *((uint8_t *)buf);
  printk("Value 0x%x written.\n", *value);

  printk("Current LED state %s - turning LED %s\n", led_state ? "off" : "on",
         led_state ? "on" : "off");
  gpio_pin_set_dt(&led, led_state);
  return len;
}

#define VND_LONG_MAX_LEN 36
static uint8_t vnd_long_value[VND_LONG_MAX_LEN + 1] = {
		  'V', 'e', 'n', 'd', 'o', 'r', ' ', 'd', 'a', 't', 'a', '1',
		  'V', 'e', 'n', 'd', 'o', 'r', ' ', 'd', 'a', 't', 'a', '2',
		  'V', 'e', 'n', 'd', 'o', 'r', ' ', 'd', 'a', 't', 'a', '3',
			};

static ssize_t write_long_vnd(struct bt_conn *conn,
		const struct bt_gatt_attr *attr, const void *buf,
		uint16_t len, uint16_t offset, uint8_t flags)
{
	uint8_t *value = attr->user_data;

	if (flags & BT_GATT_WRITE_FLAG_PREPARE) {
		return 0;
	}

	if (offset + len > VND_LONG_MAX_LEN) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	memcpy(value + offset, buf, len);
	value[offset + len] = 0;

	return len;
}

// Attribute Table

BT_GATT_SERVICE_DEFINE(
    led_svc, BT_GATT_PRIMARY_SERVICE(&led_svc_uuid),
    BT_GATT_CHARACTERISTIC(&led_state_char_uuid.uuid,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
                           BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
                           read_led_state, write_led_state, &led_state), 


	BT_GATT_CHARACTERISTIC(&vnd_long_uuid.uuid, BT_GATT_CHRC_READ |
							BT_GATT_CHRC_WRITE | BT_GATT_CHRC_EXT_PROP,
							BT_GATT_PERM_READ | BT_GATT_PERM_WRITE |
							BT_GATT_PERM_PREPARE_WRITE,
							read_led_state, write_long_vnd, &vnd_long_value),
	);



static void read_gyro_data(const struct device * gyro_dev)
{
	sensor_sample_fetch(gyro_dev);
	sensor_channel_get(gyro_dev, SENSOR_CHAN_GYRO_XYZ, gyr);
}

static void display_gyro_data(void)
{

	printk("pitch %d roll %d \n", gyr[1].val1 , gyr[2].val1);
	printk("heading %d \n", gyr[0].val1);
}

int main(void)
{
	int ret;
	//bool led_state = true;
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
	k_msleep(5000);
	const struct device *gyro_dev = DEVICE_DT_GET(DT_NODELABEL(bno055_l));
	if (!device_is_ready(gyro_dev)) {
		printk("Device %s is not ready\n", gyro_dev->name);
		return 0;
	}


	gpio_pin_set_dt(&led, led_state);
	// initialize BLE
	ret = bt_enable(NULL);
	if (ret) {
		printk("Bluetooth init failed (err %d)\n", ret);
		return;
	}
	printk("Bluetooth initialized\n");

	// start avertising
	ret = bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad, ARRAY_SIZE(ad), NULL, 0);
	if (ret) {
		printk("Advertising failed to start (err %d)\n", ret);
		return;
	}
	printk("Advertising successfully started\n");




	printk("Device %s ready,starting aquisition!\n", gyro_dev->name);
	while (1) {
		// ret = gpio_pin_toggle_dt(&led);
		// if (ret < 0) {
		// 	return 0;
		// }
		//LOG_INF("Logging to usb!\n");
		//led_state = !led_state;
		printk("LED state: %s\n", led_state ? "ON" : "OFF");
		k_msleep(SLEEP_TIME_MS);

		read_gyro_data(gyro_dev);
		display_gyro_data();
	}

	return 0;
}

