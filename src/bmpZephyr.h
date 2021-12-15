#ifndef _BMPZEPHYR_H
#define _BMPZEPHYR_H

#include <stdint.h>
#include "bmp3.h"
#include "bmp3_defs.h"
#include <drivers/i2c.h>
#include <drivers/gpio.h>
#include "workQueue.h"
#include "phyphox.h"

#include "ble.h"


#define BMP_INT DT_NODELABEL(button1)
static struct k_work work_data;
static struct k_work work_config;

#define BMP3_SENSOR_OK 0x00
#define BMP384_I2C_ADDR	0x76
static int8_t bmpResult;
static uint8_t dByte;

static struct bmp3_dev bmp388_dev;
static struct bmp3_data myData;
static uint8_t sensor_comp = BMP3_PRESS | BMP3_TEMP;
static int16_t counter=0;

static void setConfigBMP();
static uint8_t sleepBMP(bool SLEEP);
#endif