#ifndef _ADS1231_H
#define _ADS1231_H

#include <zephyr/zephyr.h>
#include "sensors.h"

#define SCL_PIN 21
#define DOUT_PIN 23
#define SPEED_PIN 2
#define PWDN_PIN 31

typedef struct {
	long offset;
	long calibrationWeight;
	long unweighted;
	long weighted;
	float weight;
	float timestamp;
	float array[2];
	uint8_t config[20];
}ADS1231;

extern ADS1231 ads1231_data;
extern void init_ads1231();
extern void sleep_ads1231(bool sleep);

extern long read_loadcell();

#endif