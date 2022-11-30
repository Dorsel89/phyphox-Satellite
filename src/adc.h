#ifndef _ADC_H   /* Include guard */
#define _ADC_H

#include <drivers/led.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/adc.h>

#define ADC_RESOLUTION		10
#define ADC_GAIN			ADC_GAIN_1_6
#define ADC_REFERENCE		ADC_REF_INTERNAL
#define ADC_ACQUISITION_TIME	ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 40)
#define BUFFER_SIZE			6
#define BAD_ANALOG_READ -123

#define MEASURE_NODE DT_ALIAS(measure_battery)
#define MEASURE_PARENT_CONTROLLER DT_PARENT(MEASURE_NODE)

#if DT_NODE_HAS_STATUS(MEASURE_PARENT_CONTROLLER, okay)
static const struct device *measure_battery_dev = DEVICE_DT_GET(MEASURE_PARENT_CONTROLLER);
#else
#error "Node is disabled"
#endif

#define SCAP_NODE DT_NODELABEL(button9)
#define SCAP_GPIO DT_GPIO_LABEL(SCAP_NODE, gpios)
#define SCAP_PIN DT_GPIO_PIN(SCAP_NODE, gpios)
#define SCAP_FLAGS DT_GPIO_FLAGS(SCAP_NODE, gpios)

static struct k_timer timer_bas;
static struct k_work work_bas;

void init_BAS();
void measure_battery_level();

float AnalogRead(int channel);


#endif