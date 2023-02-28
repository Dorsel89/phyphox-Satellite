#ifndef _ADC_H   /* Include guard */
#define _ADC_H

#include <drivers/led.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/adc.h>

#include <hal/nrf_saadc.h>

#define BAD_ANALOG_READ -123

#define MEASURE_NODE DT_ALIAS(measure_battery)
#define MEASURE_PARENT_CONTROLLER DT_PARENT(MEASURE_NODE)

#if DT_NODE_HAS_STATUS(MEASURE_PARENT_CONTROLLER, okay)
static const struct device *measure_battery_dev = DEVICE_DT_GET(MEASURE_PARENT_CONTROLLER);
#else
#error "Node is disabled"
#endif


static struct k_timer timer_bas;
static struct k_work work_bas;

void init_BAS();
void measure_battery_level();

extern void AnalogRead(); 

#endif