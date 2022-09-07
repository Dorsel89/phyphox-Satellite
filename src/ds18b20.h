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
extern void ow_init(); 

typedef uint8_t DeviceAddress[8];

static uint8_t ds18b20_Adresses[8*10];
static uint8_t tc_Adresses[8];

typedef DeviceAddress test[10];

static unsigned char ROM_NO[8];
static uint8_t LastDiscrepancy;
static uint8_t LastFamilyDiscrepancy;
static bool LastDeviceFlag;

static uint8_t devices;
extern uint8_t ds18Count;
extern uint8_t tcCount;
void reset_search();
uint8_t search(uint8_t *newAddr, bool search_mode);

float ds18b20_get_temp(void);
void ds18b20_setResolution(uint8_t resolution);
float ds18b20_get_temp_method_2(void);

static uint8_t ow_read_bit(void);
static bool valid_DS18B20(const uint8_t* deviceAddress);
static bool valid_MAX31850(const uint8_t* deviceAddress);

extern float ds18b20_getTemperature(uint8_t deviceNumber);
extern void ds18b20_measureTemperature(uint8_t deviceNumber);

static void select(const uint8_t rom[8]);

/** @} */
#endif