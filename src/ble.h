#ifndef BLE_H   /* Include guard */
#define BLE_H

#include <zephyr.h>
#include <drivers/gpio.h>
#include <device.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>
#include <bluetooth/services/bas.h>
#include "workQueue.h"
#include <sys/byteorder.h>
#include "bmpZephyr.h"
#include "led.h"

/* Bluetooth Display Names */
#define SENSOR_BMP384_NAME			"BMP384 Umgebungssensor"
#define SENSOR_SHTC3_NAME			"SHTC3 Umgebungssensor"
#define SENSOR_MLX90393_NAME		"MLX90393 Magnetfeldsensor"
#define SENSOR_ICM42605_NAME		"ICM42605 Gyrosensor"
#define SENSOR_DS18B20_NAME			"DS18B20 Temperatursensor"
#define SENSOR_MPRLS_NAME			"MPRLS Drucksensor"
#define SENSOR_MAX31850_NAME		"MAX31850 Temperatursensor"
#define SENSOR_ADS1231_NAME			"ADS1231 Wägezelle"


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

static uint8_t phyphox_data[20] = {0};
static uint8_t config_data[20] = {0};

static bool notify_enabled;
static void ccc_cfg_changed(const struct bt_gatt_attr *attr,
				 uint16_t value)
{
	notify_enabled = (value == BT_GATT_CCC_NOTIFY) ? 1 : 0;
}

extern void initBLE();
extern void sendData(uint8_t, float* ,uint8_t);

static void bmp_config_notification(const struct bt_gatt_attr *attr,uint8_t value)
{
	printk("config bmp set");
}
static void imu_config_notification(const struct bt_gatt_attr *attr,uint8_t value)
{
	printk("config imu set");
}
static void shtc_config_notification(const struct bt_gatt_attr *attr,uint8_t value)
{
	printk("config imu set");
}

//supercap
#define SCAP_NODE DT_NODELABEL(button9)
#define SCAP_GPIO DT_GPIO_LABEL(SCAP_NODE, gpios)
#define SCAP_PIN DT_GPIO_PIN(SCAP_NODE, gpios)
#define SCAP_FLAGS DT_GPIO_FLAGS(SCAP_NODE, gpios)

static struct k_timer timer_bas;
static struct k_work work_bas;

void init_BAS();

#endif