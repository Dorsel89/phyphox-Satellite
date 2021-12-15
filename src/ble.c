#include "ble.h"

static ssize_t read_u16(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			void *buf, uint16_t len, uint16_t offset)
{
	// this function is only called on READ, which one is called if notification is set?!
    printk("len: %i\n",len);
	printk("offset: %i\n",offset);
	//const uint16_t *u16 = attr->user_data;
	//uint16_t value = sys_cpu_to_le16(*u16);
	uint8_t *value = attr->user_data;

//	return bt_gatt_attr_read(conn, attr, buf, len, offset, value, sizeof(value));
	return bt_gatt_attr_read(conn, attr, buf, len, offset, value, 12);
}

static ssize_t write_u16(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			 const void *buf, uint16_t len, uint16_t offset,
			 uint8_t flags)
{
	uint8_t *value = attr->user_data;

	if (offset + len > sizeof(config_data)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	memcpy(value + offset, buf, len);

	return len;
}

static ssize_t dataWritten(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset,uint8_t flags)
{
	uint8_t *value = attr->user_data;
	if (offset + len > sizeof(config_data)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}
	memcpy(value + offset, buf, len);
	
	if(attr->uuid == &bmp_cnfg.uuid){
		submitConfigBMP();
	}
	return len;
}

BT_GATT_SERVICE_DEFINE(phyphoxGATT, 
	BT_GATT_PRIMARY_SERVICE(&data_service_uuid),
	//ICM42605S
	BT_GATT_CHARACTERISTIC(&bmp_uuid,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ,
			       read_u16, NULL, &bmpData.pressure),
	BT_GATT_CCC(ccc_cfg_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    BT_GATT_CHARACTERISTIC(&bmp_cnfg,
			       BT_GATT_CHRC_WRITE | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_WRITE,
			       NULL, dataWritten, &bmpData.config[0]),
	BT_GATT_CCC(bmp_config_notification,	//notification handler
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE)
);
static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL,
		BT_UUID_16_ENCODE(BT_UUID_BAS_VAL), //battery service
		BT_UUID_16_ENCODE(BT_UUID_ESS_VAL)),//general sensor service
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, 
		BT_UUID_128_ENCODE(0xcddf1001, 0x30f7, 0x4671, 0x8b43, 0x5e40ba53514a))
		//BT_UUID_128_ENCODE(0xcddf1002, 0x30f7, 0x4671, 0x8b43, 0x5e40ba53514a), 
		//BT_UUID_128_ENCODE(0xcddf1003, 0x30f7, 0x4671, 0x8b43, 0x5e40ba53514a))
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

static struct bt_conn_cb conn_callbacks = {
	.connected = connected,
	.disconnected = disconnected,
};
static void bt_ready(void)
{
	int err;

	printk("Bluetooth initialized\n");

	err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err) {
		printk("Advertising failed to start (err %d)\n", err);
		return;
	}

	printk("Advertising successfully started\n");
}

/*phyphox write data into characteristic*/
void write(float data1) {
	float array[1] = {data1};
	bt_gatt_notify(NULL, &phyphoxGATT.attrs[2], &array, 4);
}

void writeMulti(float data1,float data2,float data3, float data4, float data5, uint16_t datapoints){
	float array[5] = {data1, data2, data3, data4, data5};
	bt_gatt_notify(NULL, &phyphoxGATT.attrs[2], array, datapoints*4);
}

/*
 *  Dummy Battery Notification
 */
static void bas_notify(void)
{
	uint8_t battery_level = bt_bas_get_battery_level();

	battery_level--;

	if (!battery_level) {
		battery_level = 100U;
	}

	bt_bas_set_battery_level(battery_level);
}
extern void initBLE(){
     bt_enable(NULL);
     bt_ready();
     bt_conn_cb_register(&conn_callbacks);
};


extern void sendData(uint8_t ID, float* DATA,uint8_t LEN){
	bt_gatt_notify(NULL, &phyphoxGATT.attrs[ID], DATA, LEN);
};