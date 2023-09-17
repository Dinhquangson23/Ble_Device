#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/kernel.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

#include <inttypes.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/sys/util.h>

#if !DT_NODE_EXISTS(DT_PATH(zephyr_user)) || \
	!DT_NODE_HAS_PROP(DT_PATH(zephyr_user), io_channels)
#error "No suitable devicetree overlay specified"
#endif

#define DT_SPEC_AND_COMMA(node_id, prop, idx) \
	ADC_DT_SPEC_GET_BY_IDX(node_id, idx),

/* Data of ADC io-channels specified in devicetree. */
static const struct adc_dt_spec adc_channels[] = {
	DT_FOREACH_PROP_ELEM(DT_PATH(zephyr_user), io_channels,
			     DT_SPEC_AND_COMMA)
};


#define BT_UUID_CUSTOM_SERVICE_BLE_DEVICE \
	BT_UUID_128_ENCODE(0xcb18950e, 0x1271, 0x48f8, 0x8e4f, 0x03e167892b9b)



static struct bt_uuid_128 ble_Deivce_UUID = BT_UUID_INIT_128(
	BT_UUID_CUSTOM_SERVICE_BLE_DEVICE);

static struct bt_uuid_128 ota_UUID = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0x01621cb0, 0x979a, 0x4f74, 0x8920, 0x1c6ff6eb3e97));

static struct bt_uuid_128 name_Char_UUID = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0x216394f2, 0x4f9b, 0x11ee, 0xbe56, 0x0242ac1200022));

static struct bt_uuid_128 hardware_Char_UUID = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0x8745acee, 0x241f, 0x4ed2, 0x883a, 0xe29c27fea761));

static struct bt_uuid_128 sensor_Service_UUID = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0xcfb227d4, 0x4fa1, 0x11ee, 0xbe56, 0x0242ac120002));

static struct bt_uuid_128 signal_UUID = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0x1e738148, 0x4f9c, 0x11ee, 0xbe56, 0x0242ac120002));

static struct bt_uuid_128 threshold_UUID = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0x3b5e0aa8, 0x4f9c, 0x11ee, 0xbe56, 0x0242ac120002));

static struct bt_uuid_128 logic_UUID = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0xbbacc8a2, 0x4f9c, 0x11ee, 0xbe56, 0x0242ac120002));

static struct bt_uuid_128 state_Control_UUID = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0xd0a78c10, 0x4f9c, 0x11ee, 0xbe56, 0x0242ac120002));

static struct bt_uuid_128 battery_UUID = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0xe18abb38, 0x4f9c, 0x11ee, 0xbe56, 0x0242ac120002));


static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_CUSTOM_SERVICE_BLE_DEVICE),
};

#define DATA_LEN 20
static uint8_t ota_value[DATA_LEN + 1];
static uint8_t name_value[DATA_LEN + 1];
static uint8_t hardware_value[DATA_LEN + 1];
static uint8_t checkNotifyMain;

static void vnd_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	checkNotifyMain = value == BT_GATT_CCC_NOTIFY; 
}

static ssize_t onReadOTA(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			void *buf, uint16_t len, uint16_t offset)
{
	const char *value = attr->user_data;

	return bt_gatt_attr_read(conn, attr, buf, len, offset, value,
				 strlen(value));
}

static ssize_t onWriteOTA(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			 const void *buf, uint16_t len, uint16_t offset,
			 uint8_t flags)
{
	uint8_t *value = attr->user_data;

	if (offset + len > DATA_LEN) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	memcpy(value + offset, buf, len);
        printk("%s\n", value);
	value[offset + len] = 0;

	return len;
}

static ssize_t onReadName(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			void *buf, uint16_t len, uint16_t offset)
{
	const char *value = attr->user_data;

	return bt_gatt_attr_read(conn, attr, buf, len, offset, value,
				 strlen(value));
}

static ssize_t onWriteName(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			 const void *buf, uint16_t len, uint16_t offset,
			 uint8_t flags)
{
	uint8_t *value = attr->user_data;

	if (offset + len > DATA_LEN) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	memcpy(value + offset, buf, len);
        printk("%s\n", value);
	value[offset + len] = 0;

	return len;
}

static ssize_t onReadHardware(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			void *buf, uint16_t len, uint16_t offset)
{
	const char *value = attr->user_data;

	return bt_gatt_attr_read(conn, attr, buf, len, offset, value,
				 strlen(value));
}

static ssize_t onWriteHardware(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			 const void *buf, uint16_t len, uint16_t offset,
			 uint8_t flags)
{
	uint8_t *value = attr->user_data;

	if (offset + len > DATA_LEN) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	memcpy(value + offset, buf, len);
        printk("%s\n", value);
	value[offset + len] = 0;

	return len;
}

/*______________________________________________________________________*/

static uint8_t signal_value[DATA_LEN + 1];
static uint8_t threshold_value[DATA_LEN + 1];
static uint8_t logic_value[DATA_LEN + 1];
static uint8_t stateControl_value[DATA_LEN + 1];
static uint8_t battery_value[DATA_LEN + 1];

static ssize_t onReadSignal(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			void *buf, uint16_t len, uint16_t offset)
{
	const char *value = attr->user_data;

	return bt_gatt_attr_read(conn, attr, buf, len, offset, value,
				 strlen(value));
}

static ssize_t onWriteSignal(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			 const void *buf, uint16_t len, uint16_t offset,
			 uint8_t flags)
{
	uint8_t *value = attr->user_data;

	if (offset + len > DATA_LEN) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	memcpy(value + offset, buf, len);
        printk("%s\n", value);
	value[offset + len] = 0;

	return len;
}

static ssize_t onReadThreshold(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			void *buf, uint16_t len, uint16_t offset)
{
	const char *value = attr->user_data;

	return bt_gatt_attr_read(conn, attr, buf, len, offset, value,
				 strlen(value));
}

static ssize_t onWriteThreshold(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			 const void *buf, uint16_t len, uint16_t offset,
			 uint8_t flags)
{
	uint8_t *value = attr->user_data;

	if (offset + len > DATA_LEN) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	memcpy(value + offset, buf, len);
        printk("%s\n", value);
	value[offset + len] = 0;

	return len;
}

static ssize_t onReadLogic(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			void *buf, uint16_t len, uint16_t offset)
{
	const char *value = attr->user_data;

	return bt_gatt_attr_read(conn, attr, buf, len, offset, value,
				 strlen(value));
}

static ssize_t onWriteLogic(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			 const void *buf, uint16_t len, uint16_t offset,
			 uint8_t flags)
{
	uint8_t *value = attr->user_data;

	if (offset + len > DATA_LEN) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	memcpy(value + offset, buf, len);
        printk("%s\n", value);
	value[offset + len] = 0;

	return len;
}

static ssize_t onReadStateControl(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			void *buf, uint16_t len, uint16_t offset)
{
	const char *value = attr->user_data;

	return bt_gatt_attr_read(conn, attr, buf, len, offset, value,
				 strlen(value));
}

static ssize_t onReadBattery(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			void *buf, uint16_t len, uint16_t offset)
{
	const char *value = attr->user_data;

	return bt_gatt_attr_read(conn, attr, buf, len, offset, value,
				 strlen(value));
}

static ssize_t onWriteBattery(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			 const void *buf, uint16_t len, uint16_t offset,
			 uint8_t flags)
{
	uint8_t *value = attr->user_data;

	if (offset + len > DATA_LEN) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	memcpy(value + offset, buf, len);
        printk("%s\n", value);
	value[offset + len] = 0;

	return len;
}

BT_GATT_SERVICE_DEFINE(nameDevice,
	BT_GATT_PRIMARY_SERVICE(&ble_Deivce_UUID),
	BT_GATT_CHARACTERISTIC(&ota_UUID.uuid,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ |
			       BT_GATT_PERM_WRITE,
			       onReadOTA, onWriteOTA, ota_value),
	BT_GATT_CCC(vnd_ccc_cfg_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	BT_GATT_CHARACTERISTIC(&name_Char_UUID.uuid,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
			       BT_GATT_PERM_READ |
			       BT_GATT_PERM_WRITE,
			       onReadName, onWriteName, name_value),
	BT_GATT_CHARACTERISTIC(&hardware_Char_UUID.uuid,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
			       BT_GATT_PERM_READ |
			       BT_GATT_PERM_WRITE,
			       onReadHardware, onWriteHardware, hardware_value),

	BT_GATT_PRIMARY_SERVICE(&sensor_Service_UUID),
	BT_GATT_CHARACTERISTIC(&signal_UUID.uuid,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ |
			       BT_GATT_PERM_WRITE,
			       onReadSignal, onWriteSignal, signal_value),
	BT_GATT_CCC(vnd_ccc_cfg_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	BT_GATT_CHARACTERISTIC(&threshold_UUID.uuid,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
			       BT_GATT_PERM_READ |
			       BT_GATT_PERM_WRITE,
			       onReadThreshold, onWriteThreshold, threshold_value),
	BT_GATT_CHARACTERISTIC(&logic_UUID.uuid,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
			       BT_GATT_PERM_READ |
			       BT_GATT_PERM_WRITE,
			       onReadLogic, onWriteLogic, logic_value),
	BT_GATT_CHARACTERISTIC(&state_Control_UUID.uuid,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ,
			       onReadStateControl, NULL, stateControl_value),
	BT_GATT_CCC(vnd_ccc_cfg_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	BT_GATT_CHARACTERISTIC(&battery_UUID.uuid,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ |
			       BT_GATT_PERM_WRITE,
			       onReadBattery, onWriteBattery, battery_value),
	BT_GATT_CCC(vnd_ccc_cfg_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

);

void mtu_updated(struct bt_conn *conn, uint16_t tx, uint16_t rx)
{
	printk("Updated MTU: TX: %d RX: %d bytes\n", tx, rx);
}

static struct bt_gatt_cb gatt_callbacks = {
	.att_mtu_updated = mtu_updated
};

static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		printk("Connection failed (err 0x%02x)\n", err);
	} else {
		printk("Connected\n");
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	printk("Disconnected (reason 0x%02x)\n", reason);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};

static void bt_ready(void)
{
	int err;

	printk("Bluetooth initialized\n");

	// cts_init();

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	// const char* name = "SON";
	err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err) {
		printk("Advertising failed to start (err %d)\n", err);
		return;
	}

	printk("Advertising successfully started\n");
}


int main(void)
{
        
        int err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return 0;
	}

        bt_ready();

        bt_gatt_cb_register(&gatt_callbacks);
        printk("alo %s\n", CONFIG_BOARD);
	
	// uint8_t temp = 0;
	// uint8_t temp1 = 100;
	// while (true)
	// {
	// 	printk("----------------------\n");
	// 	printk("%d\n", temp++);
	// 	printk("%s\n", ota_value);
	// 	printk("%s\n", name_value);
	// 	printk("%s\n", hardware_value);
	// 	printCheck();
	// 	printk("%s\n", signal_value);
	// 	printk("%s\n", threshold_value);
	// 	printk("%s\n", logic_value);
	// 	printk("%s\n", stateControl_value);
	// 	printk("%s\n", battery_value);
	// 	printk("----------------------\n");
	// 	k_sleep(K_SECONDS(5));
	// 	// printk("hello\n");
	// 	// printk("%d\n",temp);
	// 	// temp++;
	// 	// k_sleep(K_SECONDS(5));
	// 	// bt_gatt_notify_uuid(NULL, &ota_UUID.uuid, &nameDevice.attrs[5], &temp, sizeof(temp));
	// 	// k_sleep(K_SECONDS(5));
	// 	// bt_gatt_notify_uuid(NULL, &signal_UUID.uuid, &nameDevice.attrs[9], &temp1, sizeof(temp1));
	// 	// printk("ket qua: %d %02X\n",result, result);
		
	// }

	
	int error;
	uint32_t count = 0;
	uint16_t buf;
	struct adc_sequence sequence = {
		.buffer = &buf,
		/* buffer size in bytes, not number of samples */
		.buffer_size = sizeof(buf),
	};

	/* Configure channels individually prior to sampling. */
	for (size_t i = 0U; i < ARRAY_SIZE(adc_channels); i++) {
		if (!device_is_ready(adc_channels[i].dev)) {
			printk("ADC controller device %s not ready\n", adc_channels[i].dev->name);
			return 0;
		}

		error = adc_channel_setup_dt(&adc_channels[i]);
		if (error < 0) {
			printk("Could not setup channel #%d (%d)\n", i, error);
			return 0;
		}
	}

	while (1) {
		printk("ADC reading[%u]:\n", count++);
		for (size_t i = 0U; i < ARRAY_SIZE(adc_channels); i++) {
			int32_t val_mv;

			printk("- %s, channel %d: ",
			       adc_channels[i].dev->name,
			       adc_channels[i].channel_id);

			(void)adc_sequence_init_dt(&adc_channels[i], &sequence);

			error = adc_read(adc_channels[i].dev, &sequence);
			if (error < 0) {
				printk("Could not read (%d)\n", error);
				continue;
			}
			
			printk("error %d\n", buf);

			/*
			 * If using differential mode, the 16 bit value
			 * in the ADC sample buffer should be a signed 2's
			 * complement value.
			 */
			if (adc_channels[i].channel_cfg.differential) {
				val_mv = (int32_t)((int16_t)buf);
			} else {
				val_mv = (int32_t)buf;
			}
			printk("%"PRId32, val_mv);
			error = adc_raw_to_millivolts_dt(&adc_channels[i],
						       &val_mv);
			/* conversion to mV may not be supported, skip if not */
			if (error < 0) {
				printk(" (value in mV not available)\n");
			} else {
				printk(" = %"PRId32" mV\n", val_mv);
			}
		}

		k_sleep(K_MSEC(1000));
	}
        
    return 0;
}
