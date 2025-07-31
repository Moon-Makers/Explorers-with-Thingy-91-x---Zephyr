/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/kernel.h>
#include <zephyr/types.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/devicetree.h>
#include <string.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <bluetooth/services/nus.h>

#define MODULE ble_handler
#include "module_state_event.h"
#include "peer_conn_event.h"
#include "ble_ctrl_event.h"
#include "ble_data_event.h"
#include "uart_data_event.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(MODULE, CONFIG_BRIDGE_BLE_LOG_LEVEL);

#define BLE_RX_BLOCK_SIZE (CONFIG_BT_L2CAP_TX_MTU - 3)
#define BLE_RX_BUF_COUNT 4
#define BLE_SLAB_ALIGNMENT 4

#define BLE_TX_BUF_SIZE (CONFIG_BRIDGE_BUF_SIZE * 2)

#define BLE_AD_IDX_FLAGS 0
#define BLE_AD_IDX_NAME 1

#define ATT_MIN_PAYLOAD 20 /* Minimum L2CAP MTU minus ATT header */

static void bt_send_work_handler(struct k_work *work);

K_MEM_SLAB_DEFINE(ble_rx_slab, BLE_RX_BLOCK_SIZE, BLE_RX_BUF_COUNT, BLE_SLAB_ALIGNMENT);
RING_BUF_DECLARE(ble_tx_ring_buf, BLE_TX_BUF_SIZE);

static K_SEM_DEFINE(ble_tx_sem, 0, 1);

static K_WORK_DEFINE(bt_send_work, bt_send_work_handler);

static struct bt_conn *current_conn;
static struct bt_gatt_exchange_params exchange_params;
static uint32_t nus_max_send_len;
static atomic_t ready;
static atomic_t active;

#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)

#define SHORT_RANGE_RF_FE_EXISTS (DT_NODE_EXISTS(ZEPHYR_USER_NODE) \
		&& DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, short_range_rf_fe_enable_gpios))

static char bt_device_name[CONFIG_BT_DEVICE_NAME_MAX + 1] = CONFIG_BT_DEVICE_NAME;

static struct bt_data ad[] = {
	[BLE_AD_IDX_FLAGS] = BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	[BLE_AD_IDX_NAME] = BT_DATA(BT_DATA_NAME_COMPLETE, bt_device_name, (sizeof(CONFIG_BT_DEVICE_NAME) - 1)),
};

static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_VAL),
};

// **BMI270 BLE SERVICE UUID DEFINITIONS**
#define BMI270_SERVICE_UUID_VAL \
	BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef2)

#define BMI270_REQUEST_CHAR_UUID_VAL \
	BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef3)

#define BMI270_DATA_CHAR_UUID_VAL \
	BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef4)

#define BMI270_CONTROL_CHAR_UUID_VAL \
	BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef5)

static struct bt_uuid_128 bmi270_service_uuid = BT_UUID_INIT_128(BMI270_SERVICE_UUID_VAL);
static struct bt_uuid_128 bmi270_request_char_uuid = BT_UUID_INIT_128(BMI270_REQUEST_CHAR_UUID_VAL);
static struct bt_uuid_128 bmi270_data_char_uuid = BT_UUID_INIT_128(BMI270_DATA_CHAR_UUID_VAL);
static struct bt_uuid_128 bmi270_control_char_uuid = BT_UUID_INIT_128(BMI270_CONTROL_CHAR_UUID_VAL);

// **BME68x SENSOR DATA STRUCTURE**
struct bme68x_data {
	float temperature;    // °C
	float pressure;      // hPa
	float humidity;      // %
	int iaq;            // Indoor Air Quality index
	float co2;          // ppm
	float voc;          // ppm
	uint32_t timestamp; // seconds since boot
	bool valid;         // Data validity flag
};

static struct bme68x_data last_bme68x_data = {
	.valid = false
};

// **BMI270 SENSOR DATA STRUCTURE**
struct bmi270_data {
	float accel_x, accel_y, accel_z;  // m/s²
	float gyro_x, gyro_y, gyro_z;     // rad/s
	uint32_t timestamp;               // seconds since boot
	bool valid;                       // Data validity flag
};

static struct bmi270_data last_bmi270_data = {
	.valid = false
};

// BMI270 characteristic values
static uint8_t bmi270_request_value = 0;
static uint8_t bmi270_control_value = 0; // 0=STOP, 1=START, 2=SINGLE_READ

// **BME68x DATA PROCESSING FUNCTIONS**
static void update_bme68x_data(float temp, float press, float hum, int iaq, float co2, float voc)
{
	last_bme68x_data.temperature = temp;
	last_bme68x_data.pressure = press;
	last_bme68x_data.humidity = hum;
	last_bme68x_data.iaq = iaq;
	last_bme68x_data.co2 = co2;
	last_bme68x_data.voc = voc;
	last_bme68x_data.timestamp = k_uptime_get() / 1000;
	last_bme68x_data.valid = true;
	
	LOG_INF("-> nrf53: 💾 BME68x data updated in cache");
}

// **BMI270 DATA PROCESSING FUNCTIONS**
static void update_bmi270_data(float ax, float ay, float az, float gx, float gy, float gz)
{
	last_bmi270_data.accel_x = ax;
	last_bmi270_data.accel_y = ay;
	last_bmi270_data.accel_z = az;
	last_bmi270_data.gyro_x = gx;
	last_bmi270_data.gyro_y = gy;
	last_bmi270_data.gyro_z = gz;
	last_bmi270_data.timestamp = k_uptime_get() / 1000;
	last_bmi270_data.valid = true;
	
	LOG_INF("-> nrf53: 💾 BMI270 data updated in cache");
}

// Forward declaration
static void send_bmi270_notification(const char *json_data, size_t len);

// **BMI270 BLE CHARACTERISTICS CALLBACKS**
static ssize_t bmi270_request_write(struct bt_conn *conn, const struct bt_gatt_attr *attr,
				    const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
	const uint8_t *data = buf;
	
	if (offset != 0) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}
	
	if (len != 1) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}
	
	LOG_INF("################################################");
	LOG_INF("-> nrf53: 🏃 BMI270 DATA REQUEST RECEIVED! 🏃");
	LOG_INF("-> nrf53: Request value: 0x%02X", data[0]);
	
	if (data[0] == 0x01) {
		LOG_INF("-> nrf53: ✅ BMI270 Request: READ SENSOR DATA");
		LOG_INF("-> nrf53: 🚀 Sending 'BMI_DATA' command to nRF9151...");
		
		// Send BMI_DATA command to nRF9151
		const char *bmi_command = "BMI_DATA\r\n";
		void *cmd_buf;
		int err = k_mem_slab_alloc(&ble_rx_slab, &cmd_buf, K_NO_WAIT);
		if (err) {
			LOG_WRN("-> nrf53: ❌ Failed to allocate buffer for BMI270 command");
			return BT_GATT_ERR(BT_ATT_ERR_INSUFFICIENT_RESOURCES);
		}
		
		size_t cmd_len = strlen(bmi_command);
		memcpy(cmd_buf, bmi_command, cmd_len);
		
		struct ble_data_event *event = new_ble_data_event();
		event->buf = cmd_buf;
		event->len = cmd_len;
		APP_EVENT_SUBMIT(event);
		
		LOG_INF("-> nrf53: ✅ BMI270 command sent to nRF9151!");
		
		bmi270_request_value = data[0];
	} else {
		LOG_INF("-> nrf53: ⚠️ Unknown BMI270 request: 0x%02X", data[0]);
	}
	
	LOG_INF("################################################");
	
	return len;
}

static ssize_t bmi270_request_read(struct bt_conn *conn, const struct bt_gatt_attr *attr,
				   void *buf, uint16_t len, uint16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset, &bmi270_request_value,
				 sizeof(bmi270_request_value));
}

// **BMI270 CONTROL CHARACTERISTIC CALLBACKS**
static ssize_t bmi270_control_write(struct bt_conn *conn, const struct bt_gatt_attr *attr,
				    const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
	const uint8_t *data = buf;
	
	if (offset != 0) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}
	
	if (len != 1) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}
	
	LOG_INF("################################################");
	LOG_INF("-> nrf53: 🎛️ BMI270 CONTROL COMMAND RECEIVED! 🎛️");
	LOG_INF("-> nrf53: Control value: 0x%02X", data[0]);
	
	const char *control_command = NULL;
	
	switch (data[0]) {
		case 0x00:
			LOG_INF("-> nrf53: 🛑 BMI270 Control: STOP FAST MODE");
			control_command = "BMI_STOP\r\n";
			break;
		case 0x01:
			LOG_INF("-> nrf53: 🚀 BMI270 Control: START CONTINUOUS MODE");
			control_command = "BMI_START\r\n";
			break;
		case 0x02:
			LOG_INF("-> nrf53: 📖 BMI270 Control: SINGLE READ");
			control_command = "BMI_DATA\r\n";
			break;
		case 0x03:
			LOG_INF("-> nrf53: ⚡ BMI270 Control: FAST MODE");
			control_command = "BMI_FAST\r\n";
			break;
		case 0x04:
			LOG_INF("-> nrf53: 🐌 BMI270 Control: SLOW MODE");
			control_command = "BMI_SLOW\r\n";
			break;
		default:
			LOG_INF("-> nrf53: ⚠️ Unknown BMI270 control: 0x%02X", data[0]);
			break;
	}
	
	if (control_command) {
		const char *cmd_name;
		switch (data[0]) {
			case 0x00: cmd_name = "BMI_STOP"; break;
			case 0x01: cmd_name = "BMI_START"; break;
			case 0x02: cmd_name = "BMI_DATA"; break;
			case 0x03: cmd_name = "BMI_FAST"; break;
			case 0x04: cmd_name = "BMI_SLOW"; break;
			default: cmd_name = "UNKNOWN"; break;
		}
		
		LOG_INF("-> nrf53: 🚀 Sending '%s' to nRF9151...", cmd_name);
		
		void *cmd_buf;
		int err = k_mem_slab_alloc(&ble_rx_slab, &cmd_buf, K_NO_WAIT);
		if (err) {
			LOG_WRN("-> nrf53: ❌ Failed to allocate buffer for BMI270 control");
			return BT_GATT_ERR(BT_ATT_ERR_INSUFFICIENT_RESOURCES);
		}
		
		size_t cmd_len = strlen(control_command);
		memcpy(cmd_buf, control_command, cmd_len);
		
		struct ble_data_event *event = new_ble_data_event();
		event->buf = cmd_buf;
		event->len = cmd_len;
		APP_EVENT_SUBMIT(event);
		
		LOG_INF("-> nrf53: ✅ BMI270 control command '%s' sent!", cmd_name);
		bmi270_control_value = data[0];
	}
	
	LOG_INF("################################################");
	
	return len;
}

static ssize_t bmi270_control_read(struct bt_conn *conn, const struct bt_gatt_attr *attr,
				   void *buf, uint16_t len, uint16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset, &bmi270_control_value,
				 sizeof(bmi270_control_value));
}

// **BMI270 BLE SERVICE DEFINITION**
BT_GATT_SERVICE_DEFINE(bmi270_service,
	BT_GATT_PRIMARY_SERVICE(&bmi270_service_uuid),
	// Request characteristic - for reading sensor data
	BT_GATT_CHARACTERISTIC(&bmi270_request_char_uuid.uuid,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
			       BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
			       bmi270_request_read, bmi270_request_write, NULL),
	// Data characteristic - for notifications with sensor data
	BT_GATT_CHARACTERISTIC(&bmi270_data_char_uuid.uuid,
			       BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ,
			       NULL, NULL, NULL),
	BT_GATT_CCC(NULL, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	// Control characteristic - for start/stop commands
	BT_GATT_CHARACTERISTIC(&bmi270_control_char_uuid.uuid,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
			       BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
			       bmi270_control_read, bmi270_control_write, NULL),
);

// **BMI270 DATA NOTIFICATION FUNCTION**
static void send_bmi270_notification(const char *json_data, size_t len)
{
	if (current_conn == NULL) {
		LOG_WRN("-> nrf53: ❌ No BLE connection for BMI270 notification");
		return;
	}
	
	if (len == 0 || len > 400) {
		LOG_ERR("-> nrf53: ❌ Invalid BMI270 JSON length: %d", len);
		return;
	}
	
	const struct bt_gatt_attr *attr = &bmi270_service.attrs[5]; // BMI270 data characteristic (updated index)
	
	// Simple notification - sin retry complejo que puede causar problemas
	int err = bt_gatt_notify(current_conn, attr, json_data, len);
	if (err) {
		LOG_WRN("-> nrf53: ⚠️ BMI270 notification failed: %d (dropping data)", err);
	} else {
		LOG_INF("-> nrf53: ✅ BMI270 notification sent! (%d bytes)", len);
		LOG_INF("-> nrf53: 📡 JSON: %.*s", (int)len, json_data);
	}
}

static void exchange_func(struct bt_conn *conn, uint8_t err,
			  struct bt_gatt_exchange_params *params)
{
	if (!err) {
		nus_max_send_len = bt_nus_get_mtu(conn);
	}
}

static void connected(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	if (err) {
		LOG_WRN("-> nrf53: Connection failed (err %u)", err);
		return;
	}

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_INF("################################################");
	LOG_INF("-> nrf53:          🔗 BLE DEVICE CONNECTED! 🔗");
	LOG_INF("################################################");
	LOG_INF("-> nrf53: Connected to: %s", addr);
	LOG_INF("-> nrf53: Connection Type: random");
	LOG_INF("-> nrf53: Status: Ready to receive LED commands");
	LOG_INF("-> nrf53: 📊 BME680 Sensor Service: ACTIVE via NUS");
	LOG_INF("-> nrf53: 🏃 BMI270 Sensor Service: ACTIVE via CUSTOM SERVICE (AUTO-MODE)");
	LOG_INF("-> nrf53: Supported Commands:");
	LOG_INF("-> nrf53:   • Binary: 0x01 (ON), 0x00 (OFF)");
	LOG_INF("-> nrf53:   • ASCII: '1' (ON), '0' (OFF)");
	LOG_INF("-> nrf53:   • Text: 'ON', 'OFF'");
	LOG_INF("-> nrf53:   • RGB: 0xFF,0xFF,0xFF (WHITE ON), 0x00,0x00,0x00 (OFF)");
	LOG_INF("-> nrf53: BMI270 Text Commands to nRF9151:");
	LOG_INF("-> nrf53:   • BMI_DATA → Single sensor reading (backup - auto mode active)");
	LOG_INF("-> nrf53:   • BMI_START → Start continuous mode (backup - auto mode active)");
	LOG_INF("-> nrf53:   • BMI_STOP → Stop continuous mode");
	LOG_INF("-> nrf53:   • BMI_FAST → Fast sampling mode (backup - auto mode active)");
	LOG_INF("-> nrf53:   • BMI_SLOW → Slow sampling mode (backup - auto mode active)");
	LOG_INF("-> nrf53: Sensor Data Routing:");
	LOG_INF("-> nrf53:   • BME680: JSON format via NUS → temp, press, humidity");
	LOG_INF("-> nrf53:   • BMI270: JSON format via BMI270 Service → accel(X,Y,Z) + gyro(X,Y,Z)");
	LOG_INF("-> nrf53: BLE Services Architecture:");
	LOG_INF("-> nrf53:   • NUS (Nordic UART): BME680 data + LED commands + general communication");
	LOG_INF("-> nrf53:   • BMI270 Custom Service: 3 characteristics for motion data");
	LOG_INF("-> nrf53:     - Request: Write 0x01 for manual sensor reading (backup)");
	LOG_INF("-> nrf53:     - Data: Automatic notifications with JSON motion data (500ms)");
	LOG_INF("-> nrf53:     - Control: 0x00=STOP, 0x01=START, 0x02=SINGLE, 0x03=FAST, 0x04=SLOW");
	LOG_INF("-> nrf53: Auto-Mode Status:");
	LOG_INF("-> nrf53:     - BME680 → AUTO every 2 seconds via NUS");
	LOG_INF("-> nrf53:     - BMI270 → AUTO every 500ms via BMI270 Service");
	if (last_bme68x_data.valid) {
		LOG_INF("-> nrf53:   • Last BME680 reading: %.1f°C, %.1fhPa, %.1f%%", 
		        (double)last_bme68x_data.temperature, (double)last_bme68x_data.pressure, 
		        (double)last_bme68x_data.humidity);
	} else {
		LOG_INF("-> nrf53:   • No BME680 data available yet - auto-mode will start soon");
	}
	if (last_bmi270_data.valid) {
		LOG_INF("-> nrf53:   • Last BMI270 reading: Accel(%.3f,%.3f,%.3f), Gyro(%.3f,%.3f,%.3f)", 
		        (double)last_bmi270_data.accel_x, (double)last_bmi270_data.accel_y, (double)last_bmi270_data.accel_z,
		        (double)last_bmi270_data.gyro_x, (double)last_bmi270_data.gyro_y, (double)last_bmi270_data.gyro_z);
	} else {
		LOG_INF("-> nrf53:   • No BMI270 data available yet - auto-mode will start soon (500ms)");
	}
	LOG_INF("################################################");

	current_conn = bt_conn_ref(conn);
	exchange_params.func = exchange_func;

	err = bt_gatt_exchange_mtu(current_conn, &exchange_params);
	if (err) {
		LOG_WRN("bt_gatt_exchange_mtu: %d", err);
	}

	ring_buf_reset(&ble_tx_ring_buf);

	struct peer_conn_event *event = new_peer_conn_event();

	event->peer_id = PEER_ID_BLE;
	event->dev_idx = 0;
	event->baudrate = 0; /* Don't care */
	event->conn_state = PEER_STATE_CONNECTED;
	event->conn_state_changed = true;
	APP_EVENT_SUBMIT(event);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_INF("################################################");
	LOG_INF("-> nrf53:        ❌ BLE DEVICE DISCONNECTED! ❌");
	LOG_INF("################################################");
	LOG_INF("-> nrf53: Disconnected from: %s", addr);
	LOG_INF("-> nrf53: Reason: %u", reason);
	LOG_INF("-> nrf53: Status: No longer accepting LED commands");
	LOG_INF("################################################");

	if (current_conn) {
		bt_conn_unref(current_conn);
		current_conn = NULL;
	}

	struct peer_conn_event *event = new_peer_conn_event();

	event->peer_id = PEER_ID_BLE;
	event->dev_idx = 0;
	event->baudrate = 0; /* Don't care */
	event->conn_state = PEER_STATE_DISCONNECTED;
	event->conn_state_changed = true;
	APP_EVENT_SUBMIT(event);
}

static struct bt_conn_cb conn_callbacks = {
	.connected    = connected,
	.disconnected = disconnected,
};

static void bt_send_work_handler(struct k_work *work)
{
	uint16_t len;
	uint8_t *buf;
	int err;
	bool notif_disabled = false;

	do {
		len = ring_buf_get_claim(&ble_tx_ring_buf, &buf, nus_max_send_len);

		err = bt_nus_send(current_conn, buf, len);
		if (err == -EINVAL) {
			notif_disabled = true;
			len = 0;
		} else if (err) {
			len = 0;
		}

		err = ring_buf_get_finish(&ble_tx_ring_buf, len);
		if (err) {
			LOG_ERR("ring_buf_get_finish: %d", err);
			break;
		}
	} while (len != 0 && !ring_buf_is_empty(&ble_tx_ring_buf));

	if (notif_disabled) {
		/* Peer has not enabled notifications: don't accumulate data */
		ring_buf_reset(&ble_tx_ring_buf);
	}
}

static void bt_receive_cb(struct bt_conn *conn, const uint8_t *const data,
			  uint16_t len)
{
	void *buf;
	uint16_t remainder;
	bool is_led_command = false;
	bool led_state = false; // false = OFF, true = ON
	const char *text_command = NULL;

	// **LOG BLE DATA RECEIVED WITH DETAILED PARSING**
	LOG_INF("################################################");
	LOG_INF("-> nrf53: 🔵 BLE COMMAND RECEIVED! 🔵");
	LOG_INF("-> nrf53: Data length: %d bytes", len);
	
	// Parse and identify LED commands based on Flutter app
	if (len == 1) {
		uint8_t cmd = data[0];
		if (cmd == 0x01) {
			LOG_INF("-> nrf53: ✅ LED Command: TURN LED ON (Binary: 0x01)");
			LOG_INF("-> nrf53: 💡 LED State: ON");
			is_led_command = true;
			led_state = true;
			text_command = "LED_ON\r\n";
		} else if (cmd == 0x00) {
			LOG_INF("-> nrf53: ✅ LED Command: TURN LED OFF (Binary: 0x00)");
			LOG_INF("-> nrf53: 💡 LED State: OFF");
			is_led_command = true;
			led_state = false;
			text_command = "LED_OFF\r\n";
		} else if (cmd == 0x31) { // ASCII '1'
			LOG_INF("-> nrf53: ✅ LED Command: TURN LED ON (ASCII: '1')");
			LOG_INF("-> nrf53: 💡 LED State: ON");
			is_led_command = true;
			led_state = true;
			text_command = "LED_ON\r\n";
		} else if (cmd == 0x30) { // ASCII '0'
			LOG_INF("-> nrf53: ✅ LED Command: TURN LED OFF (ASCII: '0')");
			LOG_INF("-> nrf53: 💡 LED State: OFF");
			is_led_command = true;
			led_state = false;
			text_command = "LED_OFF\r\n";
		} else if (cmd == 0xFF) {
			LOG_INF("-> nrf53: ✅ LED Command: TURN LED ON (Max brightness: 0xFF)");
			LOG_INF("-> nrf53: 💡 LED State: ON");
			is_led_command = true;
			led_state = true;
			text_command = "LED_ON\r\n";
		} else {
			LOG_INF("-> nrf53: ⚠️ Unknown single byte command: 0x%02X", cmd);
		}
	} else if (len == 2) {
		if (data[0] == 0x4F && data[1] == 0x4E) { // "ON"
			LOG_INF("-> nrf53: ✅ LED Command: TURN LED ON (Text: 'ON')");
			LOG_INF("-> nrf53: 💡 LED State: ON");
			is_led_command = true;
			led_state = true;
			text_command = "LED_ON\r\n";
		} else if (data[0] == 0x01 && data[1] == 0x01) {
			LOG_INF("-> nrf53: ✅ LED Command: TURN LED ON (Double binary: 0x01,0x01)");
			LOG_INF("-> nrf53: 💡 LED State: ON");
			is_led_command = true;
			led_state = true;
			text_command = "LED_ON\r\n";
		} else if (data[0] == 0x00 && data[1] == 0x00) {
			LOG_INF("-> nrf53: ✅ LED Command: TURN LED OFF (Double binary: 0x00,0x00)");
			LOG_INF("-> nrf53: 💡 LED State: OFF");
			is_led_command = true;
			led_state = false;
			text_command = "LED_OFF\r\n";
		} else {
			LOG_INF("-> nrf53: ⚠️ Unknown 2-byte command: 0x%02X,0x%02X", data[0], data[1]);
		}
	} else if (len == 3) {
		if (data[0] == 0x4F && data[1] == 0x46 && data[2] == 0x46) { // "OFF"
			LOG_INF("-> nrf53: ✅ LED Command: TURN LED OFF (Text: 'OFF')");
			LOG_INF("-> nrf53: 💡 LED State: OFF");
		 is_led_command = true;
			led_state = false;
			text_command = "LED_OFF\r\n";
		} else if (data[0] == 0xFF && data[1] == 0xFF && data[2] == 0xFF) { // WHITE RGB
			LOG_INF("-> nrf53: ✅ LED Command: TURN LED ON (RGB White: 0xFF,0xFF,0xFF)");
			LOG_INF("-> nrf53: 💡 LED State: ON");
			is_led_command = true;
			led_state = true;
			text_command = "LED_ON\r\n";
		} else if (data[0] == 0xFF && data[1] == 0x00 && data[2] == 0x00) { // RED RGB
			LOG_INF("-> nrf53: ✅ LED Command: TURN LED ON (RGB Red: 0xFF,0x00,0x00)");
			LOG_INF("-> nrf53: 💡 LED State: ON");
			is_led_command = true;
			led_state = true;
			text_command = "LED_ON\r\n";
		} else if (data[0] == 0x00 && data[1] == 0xFF && data[2] == 0x00) { // GREEN RGB
			LOG_INF("-> nrf53: ✅ LED Command: TURN LED ON (RGB Green: 0x00,0xFF,0x00)");
			LOG_INF("-> nrf53: 💡 LED State: ON");
		 is_led_command = true;
			led_state = true;
			text_command = "LED_ON\r\n";
		} else if (data[0] == 0x00 && data[1] == 0x00 && data[2] == 0xFF) { // BLUE RGB
			LOG_INF("-> nrf53: ✅ LED Command: TURN LED ON (RGB Blue: 0x00,0x00,0xFF)");
			LOG_INF("-> nrf53: 💡 LED State: ON");
			is_led_command = true;
			led_state = true;
			text_command = "LED_ON\r\n";
		} else if (data[0] == 0x00 && data[1] == 0x00 && data[2] == 0x00) { // RGB OFF
			LOG_INF("-> nrf53: ✅ LED Command: TURN LED OFF (RGB Off: 0x00,0x00,0x00)");
			LOG_INF("-> nrf53: 💡 LED State: OFF");
			is_led_command = true;
			led_state = false;
			text_command = "LED_OFF\r\n";
		} else if (data[0] == 0x01 && data[1] == 0x00 && data[2] == 0x01) {
			LOG_INF("-> nrf53: ✅ LED Command: TURN LED ON (Triple binary: 0x01,0x00,0x01)");
			LOG_INF("-> nrf53: 💡 LED State: ON");
			is_led_command = true;
			led_state = true;
			text_command = "LED_ON\r\n";
		} else {
			LOG_INF("-> nrf53: ⚠️ Unknown 3-byte command: 0x%02X,0x%02X,0x%02X", 
			        data[0], data[1], data[2]);
		}
	} else {
		// For longer commands, try to display as string if printable
		char temp_buf[64];
		bool is_printable = true;
		if (len < sizeof(temp_buf)) {
			memcpy(temp_buf, data, len);
			temp_buf[len] = '\0';
			
			// Check if all characters are printable
			for (int i = 0; i < len; i++) {
				if (data[i] < 32 || data[i] > 126) {
					is_printable = false;
					break;
				}
			}
			
			if (is_printable) {
				LOG_INF("-> nrf53: BLE String Command: '%s'", temp_buf);
			} else {
				LOG_INF("-> nrf53: BLE Binary Command (%d bytes)", len);
			}
		} else {
			LOG_INF("-> nrf53: BLE Large Command (%d bytes)", len);
		}
	}
	
	// **CONVERT TO TEXT COMMAND FOR nRF9151**
	if (is_led_command && text_command) {
		LOG_INF("-> nrf53: � CONVERTING to TEXT for nRF9151: '%s'", 
		        led_state ? "LED_ON" : "LED_OFF");
		LOG_INF("-> nrf53: �🚀 FORWARDING TEXT to nRF9151 via UART...");
		
		// Send the text command instead of original binary data
		int err = k_mem_slab_alloc(&ble_rx_slab, &buf, K_NO_WAIT);
		if (err) {
			LOG_WRN("-> nrf53: BLE RX overflow");
			return;
		}
		
		size_t text_len = strlen(text_command);
		memcpy(buf, text_command, text_len);
		
		struct ble_data_event *event = new_ble_data_event();
		event->buf = buf;
		event->len = text_len;
		APP_EVENT_SUBMIT(event);
		
		LOG_INF("-> nrf53: ✅ Text command sent to nRF9151!");
	} else {
		LOG_INF("-> nrf53: 🚀 FORWARDING ORIGINAL to nRF9151 via UART...");
		
		// Send original data for non-LED commands
		remainder = len;
		do {
			uint16_t copy_len;
			int err;

			err = k_mem_slab_alloc(&ble_rx_slab, &buf, K_NO_WAIT);
			if (err) {
				LOG_WRN("-> nrf53: BLE RX overflow");
				break;
			}

			copy_len = remainder > BLE_RX_BLOCK_SIZE ?
				BLE_RX_BLOCK_SIZE : remainder;
			remainder -= copy_len;
			memcpy(buf, data + (len - remainder), copy_len);

			struct ble_data_event *event = new_ble_data_event();
			event->buf = buf;
			event->len = copy_len;
			APP_EVENT_SUBMIT(event);
		} while (remainder);
	}
	
	LOG_INF("-> nrf53: Source: BLE command from mobile app");
	LOG_INF("################################################");
}

static void bt_sent_cb(struct bt_conn *conn)
{
	if (ring_buf_is_empty(&ble_tx_ring_buf)) {
		return;
	}

	k_work_submit(&bt_send_work);
}

static struct bt_nus_cb nus_cb = {
	.received = bt_receive_cb,
	.sent = bt_sent_cb,
};

static void adv_start(void)
{
	int err;

	if (!atomic_get(&ready)) {
		/* Advertising will start when ready */
		return;
	}

	err = bt_le_adv_start(
		BT_LE_ADV_PARAM(
			BT_LE_ADV_OPT_CONNECTABLE,
			BT_GAP_ADV_SLOW_INT_MIN,
			BT_GAP_ADV_SLOW_INT_MAX,
			NULL),
		ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (err) {
		LOG_ERR("bt_le_adv_start: %d", err);
	} else {
		module_set_state(MODULE_STATE_READY);
	}
}

static void adv_stop(void)
{
	int err;

	err = bt_le_adv_stop();
	if (err) {
		LOG_ERR("bt_le_adv_stop: %d", err);
	} else {
		module_set_state(MODULE_STATE_STANDBY);
	}
}

static void name_update(const char *name)
{
	int err;

	err = bt_set_name(name);
	if (err) {
		LOG_WRN("bt_set_name: %d", err);
		return;
	}

	strcpy(bt_device_name, name);
	ad[BLE_AD_IDX_NAME].data_len = strlen(name);

	err = bt_le_adv_update_data(ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (err && err != -EAGAIN) {
		/* Ignore error return when advertising is not running */
		LOG_WRN("bt_le_adv_update_data: %d", err);
		return;
	}
}

static void bt_ready(int err)
{
	if (err) {
		LOG_ERR("%s: %d", __func__, err);
		return;
	}

	err = bt_nus_init(&nus_cb);
	if (err) {
		LOG_ERR("bt_nus_init: %d", err);
		return;
	}

	atomic_set(&ready, true);

#if CONFIG_BRIDGE_BLE_ALWAYS_ON
	atomic_set(&active, true);
#endif

	if (atomic_get(&active)) {
		adv_start();
	}
}

static inline void short_range_rf_front_end_enable(void)
{
#if SHORT_RANGE_RF_FE_EXISTS
#define RF_FE_ENABLE_FLAGS DT_GPIO_FLAGS(ZEPHYR_USER_NODE, short_range_rf_fe_enable_gpios)
	const struct gpio_dt_spec short_range_rf_fe =
			GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, short_range_rf_fe_enable_gpios);

	int err = gpio_pin_configure_dt(&short_range_rf_fe, GPIO_OUTPUT | RF_FE_ENABLE_FLAGS);

	if (err) {
		LOG_ERR("Failed to enable short range RF front end");
		__ASSERT_NO_MSG(false);
	}
#endif
}

static inline void short_range_rf_front_end_disable(void)
{
#if SHORT_RANGE_RF_FE_EXISTS
	const struct gpio_dt_spec short_range_rf_fe =
			GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, short_range_rf_fe_enable_gpios);

	int err = gpio_pin_configure_dt(&short_range_rf_fe, GPIO_DISCONNECTED);

	if (err) {
		LOG_ERR("Failed to disable short range RF front end");
		__ASSERT_NO_MSG(false);
	}

#endif
}

static bool app_event_handler(const struct app_event_header *aeh)
{
	if (is_uart_data_event(aeh)) {
		const struct uart_data_event *event =
			cast_uart_data_event(aeh);

		/* Only one BLE Service instance, mapped to UART_0 */
		if (event->dev_idx != 0) {
			return false;
		}

		// **PARSE AND LOG UART DATA WITH CONTEXT**
		static int uart_log_counter = 0;
		uart_log_counter++;
		
		// Check for different types of messages from nRF9151
		char uart_buffer[256];  // Increased buffer size for sensor data
		if (event->len < sizeof(uart_buffer)) {
			memcpy(uart_buffer, event->buf, event->len);
			uart_buffer[event->len] = '\0';
					// **BME68x SENSOR DATA DETECTION**
		// Format from nRF9151: "-> BME680: temp: 23.450000°C; press: 1013.250000 Pa; humidity: 65.200000%"
		if (strstr(uart_buffer, "-> BME680:") != NULL) {
			LOG_INF("-> nrf53: 🌡️ BME680 SENSOR DATA DETECTED!");
			LOG_INF("-> nrf53: Raw data: %s", uart_buffer);
			
			// Parse BME680 sensor values from UPDATED nRF9151 format
			float temp = 0, press = 0, humidity = 0;
			
			// Parse the sensor data using sscanf with updated format from nRF9151
			int parsed = sscanf(uart_buffer, 
				"-> BME680: temp: %f°C; press: %f Pa; humidity: %f%%",
				&temp, &press, &humidity);
			
			if (parsed == 3) {
				LOG_INF("-> nrf53: ✅ BME680 DATA PARSED SUCCESSFULLY:");
				LOG_INF("-> nrf53:   🌡️  Temperature: %.2f °C", (double)temp);
				LOG_INF("-> nrf53:   📊 Pressure: %.2f Pa", (double)press);
				LOG_INF("-> nrf53:   💧 Humidity: %.2f %%", (double)humidity);
				
				// Convert pressure from Pa to hPa for consistency
				float press_hpa = press / 100.0f;
				
				// Update cached sensor data (set default values for missing IAQ, CO2, VOC)
				update_bme68x_data(temp, press_hpa, humidity, 50, 400.0f, 0.5f);
				
				// Create structured JSON data for BLE transmission via NUS
				char sensor_json[512];
				int json_len = snprintf(sensor_json, sizeof(sensor_json),
					"{\"sensor\":\"BME680\",\"temp\":%.2f,\"press\":%.2f,\"humidity\":%.2f,\"iaq\":%d,\"co2\":%.2f,\"voc\":%.3f,\"timestamp\":%u}\r\n",
					(double)temp, (double)press_hpa, (double)humidity, 50, 400.0, 0.5, last_bme68x_data.timestamp);
				
				if (json_len > 0 && json_len < sizeof(sensor_json)) {
					LOG_INF("-> nrf53: 📡 SENDING BME680 JSON via NUS to BLE:");
					LOG_INF("-> nrf53: %s", sensor_json);
					
					// Send JSON data via NUS (Nordic UART Service) for BME680
					// if (current_conn && !ring_buf_space_get(&ble_tx_ring_buf) < json_len) {
					// 	uint32_t written = ring_buf_put(&ble_tx_ring_buf, sensor_json, json_len);
					// 	if (written == json_len) {
					// 		k_work_submit(&bt_send_work);
					// 		LOG_INF("-> nrf53: ✅ BME680 JSON sent via NUS successfully!");
					// 	} else {
					// 		LOG_WRN("-> nrf53: ⚠️ BME680 NUS buffer overflow");
					// 	}
					// } else {
					// 	LOG_WRN("-> nrf53: ⚠️ No BLE connection or NUS buffer full");
					// }
					if (current_conn && ring_buf_space_get(&ble_tx_ring_buf) >= json_len) {
						uint32_t written = ring_buf_put(&ble_tx_ring_buf, sensor_json, json_len);
						if (written != (uint32_t)json_len) {
							LOG_WRN("-> nrf53: ⚠️ NUS buffer overflow (espacio insuficiente para %d bytes)", json_len);
						}
					} else {
						LOG_WRN("-> nrf53: ⚠️ No hay espacio en buffer NUS para %d bytes", json_len);
					}
				} else {
					LOG_ERR("-> nrf53: ❌ JSON buffer overflow or error");
				}
				
				// Don't send the raw UART data, return here
				return false;
			} else {
				LOG_WRN("-> nrf53: ❌ Failed to parse BME680 data (parsed %d/3 values)", parsed);
			}
		}		// **BMI270 SENSOR DATA DETECTION - AUTOMATIC REAL-TIME MODE**
		// New format from nRF9151: "-> BMI270 ACCEL:" and "-> BMI270 GYRO:" (automatic every 500ms)
		else if (strstr(uart_buffer, "-> BMI270 ACCEL:") != NULL || strstr(uart_buffer, "-> BMI270 GYRO:") != NULL) {
			static float accel_x = 0, accel_y = 0, accel_z = 0;
			static float gyro_x = 0, gyro_y = 0, gyro_z = 0;
			static bool accel_parsed = false, gyro_parsed = false;
			static uint32_t last_bmi270_time = 0;
			
			LOG_INF("-> nrf53: 🏃 BMI270 AUTO-REALTIME DATA DETECTED!");
			LOG_INF("-> nrf53: Raw data: %s", uart_buffer);
			
			// Parse BMI270 accelerometer data
			if (strstr(uart_buffer, "-> BMI270 ACCEL:") != NULL) {
				int parsed = sscanf(uart_buffer, 
					"-> BMI270 ACCEL: X=%f, Y=%f, Z=%f m/s²",
					&accel_x, &accel_y, &accel_z);
				
				if (parsed == 3) {
					LOG_INF("-> nrf53: ✅ BMI270 AUTO-ACCEL: X=%.3f, Y=%.3f, Z=%.3f", 
					        (double)accel_x, (double)accel_y, (double)accel_z);
					accel_parsed = true;
					last_bmi270_time = k_uptime_get_32();
				} else {
					LOG_WRN("-> nrf53: ❌ Failed to parse BMI270 accelerometer");
					accel_parsed = false;
				}
			}
			
			// Parse BMI270 gyroscope data
			if (strstr(uart_buffer, "-> BMI270 GYRO:") != NULL) {
				int parsed = sscanf(uart_buffer, 
					"-> BMI270 GYRO: X=%f, Y=%f, Z=%f rad/s",
					&gyro_x, &gyro_y, &gyro_z);
				
				if (parsed == 3) {
					LOG_INF("-> nrf53: ✅ BMI270 AUTO-GYRO: X=%.3f, Y=%.3f, Z=%.3f", 
					        (double)gyro_x, (double)gyro_y, (double)gyro_z);
					gyro_parsed = true;
					last_bmi270_time = k_uptime_get_32();
				} else {
					LOG_WRN("-> nrf53: ❌ Failed to parse BMI270 gyroscope");
					gyro_parsed = false;
				}
			}
			
			// If both are ready, send notification via BMI270 Service
			if (accel_parsed && gyro_parsed) {
				LOG_INF("-> nrf53: 🎯 COMPLETE BMI270 AUTO-DATA READY!");
				
				// Update cache
				update_bmi270_data(accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z);
				
				// Create structured JSON for BMI270 Service notification
				char sensor_json[400];
				int json_len = snprintf(sensor_json, sizeof(sensor_json) - 1,
					"{\"sensor\":\"BMI270\",\"mode\":\"auto\",\"accel\":{\"x\":%.3f,\"y\":%.3f,\"z\":%.3f},\"gyro\":{\"x\":%.3f,\"y\":%.3f,\"z\":%.3f},\"timestamp\":%u}",
					(double)accel_x, (double)accel_y, (double)accel_z, 
					(double)gyro_x, (double)gyro_y, (double)gyro_z, 
					last_bmi270_data.timestamp);
				
				if (json_len > 0 && json_len < (sizeof(sensor_json) - 1)) {
					LOG_INF("-> nrf53: 📡 Sending BMI270 AUTO-DATA via BMI270 Service (%d bytes)", json_len);
					send_bmi270_notification(sensor_json, json_len);
				} else {
					LOG_ERR("-> nrf53: ❌ BMI270 JSON too large: %d bytes", json_len);
				}
				
				// Reset for next reading
				accel_parsed = false;
				gyro_parsed = false;
			} else {
				// Check for timeout - if only one part received after 100ms, reset
				uint32_t current_time = k_uptime_get_32();
				if (current_time - last_bmi270_time > 100) {
					if (accel_parsed || gyro_parsed) {
						LOG_WRN("-> nrf53: ⚠️ BMI270 timeout - resetting partial data");
						accel_parsed = false;
						gyro_parsed = false;
					}
				}
			}
			
			return false; // Don't forward raw BMI270 data via NUS
		}
			// **LED STATE AND ALIVE MESSAGE DETECTION**
			else if (strstr(uart_buffer, "LED state:") != NULL || strstr(uart_buffer, "Alive - LED:") != NULL || 
			         strstr(uart_buffer, "Fast mode - LED:") != NULL) {
				LOG_INF("-> nrf53: 🔄 nRF9151 STATUS: %s (NOT from BLE command)", uart_buffer);
				// Don't send status messages to BLE
				return false;
			}
			// **BME68x/BMI270 INITIALIZATION AND INFO MESSAGES**
			else if (strstr(uart_buffer, "BME68x") != NULL || strstr(uart_buffer, "BMI270") != NULL ||
			         strstr(uart_buffer, "Device") != NULL || strstr(uart_buffer, "funcionando") != NULL) {
				LOG_INF("-> nrf53: 🔧 nRF9151 SENSOR INFO: %s", uart_buffer);
				// Don't send init/info messages to BLE
				return false;
			}
			// **nRF9151 SYSTEM MESSAGES**
			else if (strstr(uart_buffer, "-> nrf91:") != NULL) {
				LOG_INF("-> nrf53: 🔧 nRF9151 SYSTEM: %s", uart_buffer);
				// Don't send system messages to BLE (unless they are sensor data)
				return false;
			}
			// **OTHER UART DATA**
			else if (uart_log_counter % 10 == 0) {  // Log other UART data occasionally
				LOG_INF("-> nrf53: 📡 nRF9151->nRF5340: UART data received (%d bytes) [Message #%d]", 
				        event->len, uart_log_counter);
			}
		} else if (uart_log_counter % 10 == 0) {
			LOG_INF("-> nrf53: 📡 nRF9151->nRF5340: UART data received (%d bytes) [Message #%d]", 
			        event->len, uart_log_counter);
		}

		if (current_conn == NULL) {
			return false;
		}

		uint32_t written = ring_buf_put(
			&ble_tx_ring_buf,
			event->buf,
			event->len);
		if (written != event->len) {
			LOG_WRN("UART_%d -> BLE overflow", event->dev_idx);
		}

		uint32_t buf_utilization =
			(ring_buf_capacity_get(&ble_tx_ring_buf) -
			ring_buf_space_get(&ble_tx_ring_buf));

		/* Simple check to start transmission. */
		/* If bt_send_work is already running, this has no effect */
		if (buf_utilization == written) {
			k_work_submit(&bt_send_work);
		}

		return false;
	}

	if (is_ble_data_event(aeh)) {
		const struct ble_data_event *event =
			cast_ble_data_event(aeh);

		/* All subscribers have gotten a chance to copy data at this point */
		k_mem_slab_free(&ble_rx_slab, (void *)event->buf);

		return false;
	}

	if (is_ble_ctrl_event(aeh)) {
		const struct ble_ctrl_event *event =
			cast_ble_ctrl_event(aeh);

		switch (event->cmd) {
		case BLE_CTRL_ENABLE:
			if (!atomic_set(&active, true)) {
				short_range_rf_front_end_enable();
				adv_start();
			}
			break;
		case BLE_CTRL_DISABLE:
			if (atomic_set(&active, false)) {
				short_range_rf_front_end_disable();
				adv_stop();
			}
			break;
		case BLE_CTRL_NAME_UPDATE:
			name_update(event->param.name_update);
			break;
		default:
			/* Unhandled control message */
			__ASSERT_NO_MSG(false);
			break;
		}

		return false;
	}

	if (is_module_state_event(aeh)) {
		const struct module_state_event *event =
			cast_module_state_event(aeh);

		if (check_state(event, MODULE_ID(main), MODULE_STATE_READY)) {
			int err;

			atomic_set(&active, false);

			nus_max_send_len = ATT_MIN_PAYLOAD;

			err = bt_enable(bt_ready);
			if (err) {
				LOG_ERR("bt_enable: %d", err);
				return false;
			}

			bt_conn_cb_register(&conn_callbacks);
		}

		return false;
	}

	/* If event is unhandled, unsubscribe. */
	__ASSERT_NO_MSG(false);

	return false;
}

APP_EVENT_LISTENER(MODULE, app_event_handler);
APP_EVENT_SUBSCRIBE(MODULE, module_state_event);
APP_EVENT_SUBSCRIBE(MODULE, ble_ctrl_event);
APP_EVENT_SUBSCRIBE(MODULE, uart_data_event);
APP_EVENT_SUBSCRIBE_FINAL(MODULE, ble_data_event);
