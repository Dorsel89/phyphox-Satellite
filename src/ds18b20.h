#ifndef _DS18B20_H
#define _DS18B20_H
#include <zephyr/types.h>
#include "sensors.h"

// Model IDs
#define DS18S20MODEL 0x10  // also DS1820
#define DS18B20MODEL 0x28  // also MAX31820
#define DS1822MODEL  0x22
#define DS1825MODEL  0x3B  // also MAX31850
#define MAX31850     0x3B 
#define DS28EA00MODEL 0x42

#define DEVICE_DISCONNECTED_C -127
#define DEVICE_DISCONNECTED_F -196.6
#define DEVICE_DISCONNECTED_RAW -7040

#define DEVICE_FAULT_OPEN_C -254
#define DEVICE_FAULT_OPEN_F -425.199982
#define DEVICE_FAULT_OPEN_RAW -32512

#define DEVICE_FAULT_SHORTGND_C -253
#define DEVICE_FAULT_SHORTGND_F -423.399994
#define DEVICE_FAULT_SHORTGND_RAW -32384

#define DEVICE_FAULT_SHORTVDD_C -252
#define DEVICE_FAULT_SHORTVDD_F -421.599976
#define DEVICE_FAULT_SHORTVDD_RAW -32256

typedef struct {
    uint8_t n_ds18b20;
    int timer_interval;
	float temperature[10];
	float timestamp;
	float array[10+1];
    uint8_t resolution;
	uint8_t config[20];
}DS18B20;

typedef struct {
    uint8_t n_thermocouple;
	float temperature[1];
	float timestamp;
	float array[2];
	uint8_t config[20];
}THERMOCOUPLE;

extern DS18B20 ds18b20_data;
extern THERMOCOUPLE thermocouple_data;

static struct k_timer timer_ds18b20_startConversation;
static struct k_timer timer_ds18b20_getTemperature;

static struct k_work work_ds18b20_startConversation;
static struct k_work work_ds18b20_getTemperature;

static struct k_work config_work_ds18b20;

void send_data_ds18b20();
void measure_data_ds18b20();
void ds18b20_data_ready();
void set_config_ds18b20();

extern void init_ds18b20(); 
extern void sleep_ds18b20(bool sleep);
extern void submit_config_ds18b20();

static void searchForDevices();

typedef uint8_t DeviceAddress[8];

static uint8_t ds18b20_Adresses[8*10];
static uint8_t tc_Adresses[8*1];

static unsigned char ROM_NO[8];
static uint8_t LastDiscrepancy;
static uint8_t LastFamilyDiscrepancy;
static bool LastDeviceFlag;

//static uint8_t devices;
//extern uint8_t ds18Count;
//extern uint8_t tcCount;
void reset_search();
uint8_t search(uint8_t *newAddr, bool search_mode);

float ds18b20_get_temp(void);
void ds18b20_setResolution(uint8_t resolution);
float ds18b20_get_temp_method_2(void);

static uint8_t ow_read_bit(void);
static bool valid_DS18B20(const uint8_t* deviceAddress);
static bool valid_MAX31850(const uint8_t* deviceAddress);

extern void ds18b20_getTemperature();
extern void ds18b20_measureTemperature();

void submit_ds18b20_measureTemperature();
void submit_ds18b20_getTemperature();
uint8_t dsCRC8(const uint8_t *addr, uint8_t len);

static void select(const uint8_t rom[8]);

/** @} */
#endif