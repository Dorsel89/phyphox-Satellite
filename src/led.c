#include "led.h"

void led_blink_times(const struct device* led, uint8_t times)
{
    for (uint8_t i=0; i < times; i++) 
    {
        led_on(led,0);
        k_msleep(LED_ON_TIME_MS);
        led_off(led,0);
        k_msleep(LED_SLEEP_TIME_MS);
    }
}
