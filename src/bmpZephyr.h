#ifndef _BMPZEPHYR_H
#define _BMPZEPHYR_H

#include <zephyr.h>
#include <stdint.h>
#include "bmp3.h"
#include "bmp3_defs.h"
#include <drivers/i2c.h>
#include <drivers/gpio.h>
#include <drivers/sensor.h>

#include "sensors.h"
#include "ble.h"

#define BMP_NODE DT_ALIAS(i2c)
#if DT_NODE_HAS_STATUS(BMP_NODE, okay)
const static struct device *bmp_dev = DEVICE_DT_GET(BMP_NODE);
#else
#error "Node is disabled"
#endif

typedef struct {
	float pressure;
	float temperature;
	float timestamp;
	float array[3];
	uint8_t config[20];
}BMP;

extern BMP bmp_data;

static struct sensor_value bmp_press, bmp_temp;

#define BMP_INT DT_NODELABEL(button1)
static struct k_work work_bmp;
static struct k_work config_work_bmp;

extern bool init_bmp();
extern void submit_config_bmp();
extern uint8_t sleep_bmp(bool SLEEP);

// ----------------
// Driver variables
// ----------------
#define BMP3_SENSOR_OK 0x00
#define BMP384_I2C_ADDR	0x76
static int8_t bmpResult;
static uint8_t dByte;

static struct bmp3_dev bmp388_dev;
static struct bmp3_data myData;
static uint8_t sensor_comp = BMP3_PRESS | BMP3_TEMP;
static int16_t counter=0;

#endif