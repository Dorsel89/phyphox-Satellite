#ifndef _LED_H
#define _LED_H

#include <device.h>
#include <devicetree.h>
#include <drivers/led.h>

void led_blink_times(const struct device* led, uint8_t times);

/* 1000 msec = 1 sec */
#define LED_ON_TIME_MS      300
#define LED_SLEEP_TIME_MS   300

// LED DEVICE
#define RED_LED_NODE DT_ALIAS(red_led)
#define RED_LED_PARENT_CONTROLLER DT_PARENT(RED_LED_NODE)

#if DT_NODE_HAS_STATUS(RED_LED_PARENT_CONTROLLER, okay)
static const struct device *red_led_dev = DEVICE_DT_GET(RED_LED_PARENT_CONTROLLER);
#else
#error "Node is disabled"
#endif

#define BLUE_LED_NODE DT_ALIAS(blue_led)
#define BLUE_LED_PARENT_CONTROLLER DT_PARENT(BLUE_LED_NODE)

#if DT_NODE_HAS_STATUS(BLUE_LED_PARENT_CONTROLLER, okay)
static const struct device *blue_led_dev = DEVICE_DT_GET(BLUE_LED_PARENT_CONTROLLER);
#else
#error "Node is disabled"
#endif

#endif //LED_H