#include "ble.h"

static const struct bt_le_adv_param adv_param_normal = {
	.options = BT_LE_ADV_OPT_CONNECTABLE | BT_LE_ADV_OPT_USE_NAME,
	.interval_min = BT_GAP_ADV_SLOW_INT_MIN,
	.interval_max = BT_GAP_ADV_SLOW_INT_MAX,
};

static const struct bt_le_conn_param conn_paramter = {
	.interval_min = 6,
	.interval_max = 12,
	.latency = 0,
	.timeout = 10
};

static ssize_t read_u16(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
{
	if (DEBUG) 
	{
		printk("len: %i\n",len);
		printk("offset: %i\n",offset);
	}
	uint8_t *value = attr->user_data;

	return bt_gatt_attr_read(conn, attr, buf, len, offset, value, sizeof(value));
}

static ssize_t config_submits(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset,uint8_t flags)
{
	uint8_t *value = attr->user_data;
	if (offset + len > sizeof(config_data)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}
	memcpy(value + offset, buf, len);
	
	if(attr->uuid == &bmp_cnfg.uuid){
		submit_config_bmp();
	}
	if(attr->uuid == &shtc_cnfg.uuid){
		submit_config_shtc();
	}
	if(attr->uuid == &mlx_cnfg.uuid){
		submit_config_mlx();
	}
	if(attr->uuid == &icm_cnfg.uuid){
		submit_config_icm();
	}
	if(attr->uuid == &ds18b_cnfg.uuid){
		submit_config_ds18b20();
	}
	return len;
};

BT_GATT_SERVICE_DEFINE(phyphox_gatt, 
	BT_GATT_PRIMARY_SERVICE(&data_service_uuid),
	//BMP384 
	BT_GATT_CHARACTERISTIC(&bmp_uuid,					
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ,
			       read_u16, NULL, &bmp_data.array[0]),
	BT_GATT_CCC(ccc_cfg_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    BT_GATT_CHARACTERISTIC(&bmp_cnfg,					
			       BT_GATT_CHRC_WRITE | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_WRITE,
			       NULL, config_submits, &bmp_data.config[0]),
	BT_GATT_CCC(ccc_cfg_changed,	//notification handler
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	//SHTC
	BT_GATT_CHARACTERISTIC(&shtc_uuid,					
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ,
			       read_u16, NULL, &shtc_data.array[0]),
	BT_GATT_CCC(ccc_cfg_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    BT_GATT_CHARACTERISTIC(&shtc_cnfg,					
			       BT_GATT_CHRC_WRITE | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_WRITE,
			       NULL, config_submits, &shtc_data.config[0]),
	BT_GATT_CCC(ccc_cfg_changed,	//notification handler
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

	//DS18b20			
	BT_GATT_CHARACTERISTIC(&ds18b_uuid,					
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ,
			       read_u16, NULL, &ds18b20_data.array[0]),
	BT_GATT_CCC(ccc_cfg_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    BT_GATT_CHARACTERISTIC(&ds18b_cnfg,					
			       BT_GATT_CHRC_WRITE | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_WRITE,
			       NULL, config_submits, &ds18b20_data.config[0]),
	BT_GATT_CCC(ccc_cfg_changed,	//notification handler
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),			

	//MLX
	BT_GATT_CHARACTERISTIC(&mlx_uuid,					
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ,
			       read_u16, NULL, &mlx_data.array[0]),
	BT_GATT_CCC(ccc_cfg_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    BT_GATT_CHARACTERISTIC(&mlx_cnfg,					
			       BT_GATT_CHRC_WRITE | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_WRITE,
			       NULL, config_submits, &mlx_data.config[0]),
	BT_GATT_CCC(ccc_cfg_changed,	//notification handler
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	//SHTC
	/*
	BT_GATT_CHARACTERISTIC(&shtc_uuid,					
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ,
			       read_u16, NULL, &shtcData.array[0]),
	BT_GATT_CCC(ccc_cfg_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    BT_GATT_CHARACTERISTIC(&shtc_cnfg,					
			       BT_GATT_CHRC_WRITE | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_WRITE,
			       NULL, dataWritten, &shtcData.config[0]),
	BT_GATT_CCC(shtc_config_notification,	//notification handler
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	*/
	//ICM42605
	
	BT_GATT_CHARACTERISTIC(&icm_uuid_acc,				
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ,
			       read_u16, NULL, &icm_data.a_array[0]),
	BT_GATT_CCC(ccc_cfg_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	BT_GATT_CHARACTERISTIC(&icm_uuid_gyr,				
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ,
			       read_u16, NULL, &icm_data.g_array[0]),
	BT_GATT_CCC(ccc_cfg_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	BT_GATT_CHARACTERISTIC(&icm_cnfg,					
			       BT_GATT_CHRC_WRITE | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_WRITE,
			       NULL, config_submits, &icm_data.config[0]),
	BT_GATT_CCC(ccc_cfg_changed,
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

static void bt_ready(void)
{
	int err;

	printk("Bluetooth initialized\n");

	err = bt_le_adv_start(&adv_param_normal, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err) {
		printk("Advertising failed to start (err %d)\n", err);
		return;
	}

	printk("Advertising successfully started\n");
}

static void connected(struct bt_conn *conn, uint8_t err)
{
	printk("Device with index %i trying to connect...\n",bt_conn_index(conn));
	
	bt_conn_le_param_update(conn,&conn_paramter);
	if (err) {
		printk("Connection failed (err 0x%02x)\n", err);
	} else {
		printk("Connected\n");
	}

	led_blink_times(blue_led_dev,3);
	k_timer_start(&timer_bas,K_SECONDS(1),K_SECONDS(1));
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	printk("Disconnected (reason 0x%02x)\n", reason);
	k_timer_stop(&timer_bas);
	sleep_bmp(true);
	sleep_shtc(true);
	sleep_mpr(true);
	sleep_mlx(true);
	sleep_icm(true);
}

static struct bt_conn_cb conn_callbacks = {
	.connected = connected,
	.disconnected = disconnected,
};

void init_ble(){
	bt_enable(NULL);
	bt_ready();
	bt_conn_cb_register(&conn_callbacks);
};

extern void send_data(uint8_t ID, float* DATA,uint8_t LEN){
	if(ID == SENSOR_IMU_ACC_ID){
		bt_gatt_notify_uuid(NULL, &icm_uuid_acc.uuid,&phyphox_gatt.attrs[0],DATA,LEN);
		return;
	}
	if(ID == SENSOR_IMU_GYR_ID){
		bt_gatt_notify_uuid(NULL, &icm_uuid_gyr.uuid,&phyphox_gatt.attrs[0],DATA,LEN);
		return;
	}
	if(ID == SENSOR_BMP384_ID){
		bt_gatt_notify_uuid(NULL, &bmp_uuid.uuid,&phyphox_gatt.attrs[0],DATA,LEN);
		return;
	}
	if (ID == SENSOR_SHTC_ID)
	{
		bt_gatt_notify_uuid(NULL, &shtc_uuid.uuid,&phyphox_gatt.attrs[0],DATA,LEN);
		return;
	}
	if (ID == SENSOR_MLX_ID)
	{
		bt_gatt_notify_uuid(NULL, &mlx_uuid.uuid,&phyphox_gatt.attrs[0],DATA,LEN);
		return;
	}
	if (ID == SENSOR_DS18B20_ID)
	{
		bt_gatt_notify_uuid(NULL, &ds18b_uuid.uuid,&phyphox_gatt.attrs[0],DATA,LEN);
		return;
	}	
};