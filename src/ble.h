#ifndef _BLE_H   /* Include guard */
#define _BLE_H
#include <zephyr/toolchain.h>

#include <zephyr.h>
#include <drivers/gpio.h>
#include <device.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>
#include <bluetooth/services/bas.h>
#include <sys/byteorder.h>

#include "led.h"
#include "bas.h"

#include "bmpZephyr.h"
#include "shtc3.h"
#include "mprls.h"
#include "mlxZephyr.h"
#include "icm42605.h"
#include "ds18b20.h"

static uint8_t phyphox_data[20] = {0};
static uint8_t config_data[20] = {0};

void init_ble();
void send_data(uint8_t ID, float* DATA,uint8_t LEN);
extern void set_supercap_level(uint8_t val);

static bool notify_enabled;
static void ccc_cfg_changed(const struct bt_gatt_attr *attr,
				 uint16_t value)
{
	notify_enabled = (value == BT_GATT_CCC_NOTIFY) ? 1 : 0;
}
/* Bluetooth Display Names */
#define SENSOR_BMP384_NAME			"BMP384 Umgebungssensor"
#define SENSOR_SHTC3_NAME			"SHTC3 Umgebungssensor"
#define SENSOR_MLX90393_NAME		"MLX90393 Magnetfeldsensor"
#define SENSOR_ICM42605_NAME		"ICM42605 Gyrosensor"
#define SENSOR_DS18B20_NAME			"DS18B20 Temperatursensor"
#define SENSOR_MPRLS_NAME			"MPRLS Drucksensor"
#define SENSOR_MAX31850_NAME		"MAX31850 Temperatursensor"
#define SENSOR_ADS1231_NAME			"ADS1231 Wägezelle"
#define SENSOR_DS18B20_NAME			"DS18B20 Temperatursensor"
#define SENSOR_THERMOCOUPLE_NAME	"Thermocouple Temperatursensor"

static struct bt_uuid_128 data_service_uuid = BT_UUID_INIT_128(BT_UUID_128_ENCODE(0xcddf1001, 0x30f7, 0x4671, 0x8b43, 0x5e40ba53514a));
//static struct bt_uuid_128 data_characteristic_uuid = BT_UUID_INIT_128(BT_UUID_128_ENCODE(0xcddf0002, 0x30f7, 0x4671, 0x8b43, 0x5e40ba53514a));
//static struct bt_uuid_128 config_characteristic_uuid = BT_UUID_INIT_128(BT_UUID_128_ENCODE(0xcddf0003, 0x30f7, 0x4671, 0x8b43, 0x5e40ba53514a));

static struct bt_uuid_128 icm_uuid_acc = BT_UUID_INIT_128(BT_UUID_128_ENCODE(0xcddf1002, 0x30f7, 0x4671, 0x8b43, 0x5e40ba53514a));
static struct bt_uuid_128 icm_uuid_gyr = BT_UUID_INIT_128(BT_UUID_128_ENCODE(0xcddf1003, 0x30f7, 0x4671, 0x8b43, 0x5e40ba53514a));
static struct bt_uuid_128 icm_cnfg = BT_UUID_INIT_128(BT_UUID_128_ENCODE(0xcddf1004, 0x30f7, 0x4671, 0x8b43, 0x5e40ba53514a));

static struct bt_uuid_128 shtc_uuid = BT_UUID_INIT_128(BT_UUID_128_ENCODE(0xcddf1005, 0x30f7, 0x4671, 0x8b43, 0x5e40ba53514a));
static struct bt_uuid_128 shtc_cnfg = BT_UUID_INIT_128(BT_UUID_128_ENCODE(0xcddf1006, 0x30f7, 0x4671, 0x8b43, 0x5e40ba53514a));

static struct bt_uuid_128 bmp_uuid = BT_UUID_INIT_128(BT_UUID_128_ENCODE(0xcddf1007, 0x30f7, 0x4671, 0x8b43, 0x5e40ba53514a));
static struct bt_uuid_128 bmp_cnfg = BT_UUID_INIT_128(BT_UUID_128_ENCODE(0xcddf1008, 0x30f7, 0x4671, 0x8b43, 0x5e40ba53514a)); 

static struct bt_uuid_128 mlx_uuid = BT_UUID_INIT_128(BT_UUID_128_ENCODE(0xcddf1009, 0x30f7, 0x4671, 0x8b43, 0x5e40ba53514a)); 
static struct bt_uuid_128 mlx_cnfg = BT_UUID_INIT_128(BT_UUID_128_ENCODE(0xcddf100a, 0x30f7, 0x4671, 0x8b43, 0x5e40ba53514a)); 

static struct bt_uuid_128 loadcell_uuid = BT_UUID_INIT_128(BT_UUID_128_ENCODE(0xcddf100b, 0x30f7, 0x4671, 0x8b43, 0x5e40ba53514a)); 
static struct bt_uuid_128 loadcell_cnfg = BT_UUID_INIT_128(BT_UUID_128_ENCODE(0xcddf100c, 0x30f7, 0x4671, 0x8b43, 0x5e40ba53514a)); 

static struct bt_uuid_128 mprls_uuid = BT_UUID_INIT_128(BT_UUID_128_ENCODE(0xcddf100d, 0x30f7, 0x4671, 0x8b43, 0x5e40ba53514a)); 
static struct bt_uuid_128 mprls_cnfg = BT_UUID_INIT_128(BT_UUID_128_ENCODE(0xcddf100e, 0x30f7, 0x4671, 0x8b43, 0x5e40ba53514a)); 

static struct bt_uuid_128 thermo_uuid = BT_UUID_INIT_128(BT_UUID_128_ENCODE(0xcddf100f, 0x30f7, 0x4671, 0x8b43, 0x5e40ba53514a)); 
static struct bt_uuid_128 thermo_cnfg = BT_UUID_INIT_128(BT_UUID_128_ENCODE(0xcddf1010, 0x30f7, 0x4671, 0x8b43, 0x5e40ba53514a)); 

static struct bt_uuid_128 ds18b_uuid = BT_UUID_INIT_128(BT_UUID_128_ENCODE(0xcddf1011, 0x30f7, 0x4671, 0x8b43, 0x5e40ba53514a)); 
static struct bt_uuid_128 ds18b_cnfg = BT_UUID_INIT_128(BT_UUID_128_ENCODE(0xcddf1012, 0x30f7, 0x4671, 0x8b43, 0x5e40ba53514a));

static struct bt_uuid_128 hardware_uuid = BT_UUID_INIT_128(BT_UUID_128_ENCODE(0xcddf1021, 0x30f7, 0x4671, 0x8b43, 0x5e40ba53514a)); 
static struct bt_uuid_128 hardware_cnfg = BT_UUID_INIT_128(BT_UUID_128_ENCODE(0xcddf1022, 0x30f7, 0x4671, 0x8b43, 0x5e40ba53514a));

#endif