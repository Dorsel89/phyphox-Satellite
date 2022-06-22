#ifndef LED_H
#define LED_H

#include <stdint.h>
#include "zephyr.h"
#include <drivers/gpio.h>
#include <device.h>
#include "phyphox.h"

#define LED0_NODE DT_NODELABEL(led0)
#define LED0_GPIO DT_GPIO_LABEL(LED0_NODE, gpios)
#define LED0_PIN DT_GPIO_PIN(LED0_NODE, gpios)
#define LED0_FLAGS DT_GPIO_FLAGS(LED0_NODE, gpios)

#define LED1_NODE DT_NODELABEL(led1)
#define LED1_GPIO DT_GPIO_LABEL(LED1_NODE, gpios)
#define LED1_PIN DT_GPIO_PIN(LED1_NODE, gpios)
#define LED1_FLAGS DT_GPIO_FLAGS(LED1_NODE, gpios)

#define LED_BLUE_ID 0
#define LED_RED_ID 1

static struct device * led_blue_dev;
static struct device * led_red_dev;
//static uint32_t blink_interval_ms = 300; // 

bool init_LEDs(struct device* led_0, struct device* led_1);
bool blink_LED(uint8_t ID, uint8_t count);

#endif  // 