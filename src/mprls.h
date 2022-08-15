#ifndef MPRLS_H
#define MPRLS_H

#include <device.h>
#include <devicetree.h>
#include <drivers/sensor.h>

#include "sensors.h"

#define MPR_NODE DT_ALIAS(mpr_ls)
#if DT_NODE_HAS_STATUS(MPR_NODE, okay)
const static struct device *mpr_dev = DEVICE_DT_GET(MPR_NODE);
#else
#error "Node is disabled"
#endif

typedef struct {
	float pressure;
	float timestamp;
	float array[2];
	uint8_t timer_interval;
	uint8_t config[20];
}MPR;

static MPR mpr_data = { // init values
    .timer_interval = 5
};

static struct sensor_value mpr_press;

static struct k_timer timer_mpr;
static struct k_work work_mpr;
static struct k_work config_work_mpr;

extern bool init_mpr();
extern void sleep_mpr(bool sleep);
extern void submit_config_mpr();

void send_data_mpr();
void mpr_data_ready();
void set_config_mpr();


#endif