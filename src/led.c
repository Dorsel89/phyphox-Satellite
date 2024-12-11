#include "led.h"

static uint8_t blink_count = 0;
static uint8_t blink_n_times = 3;

void turn_led_on(){
    led_on(red_led_dev,0);
    k_timer_start(&timer_led_off, K_MSEC(LED_ON_TIME_MS), K_NO_WAIT);
}
void turn_led_off(){
    led_off(red_led_dev,0);
    blink_count +=1;
    if(blink_count>=blink_n_times){
        return;
    }
    k_timer_start(&timer_led_on, K_MSEC(LED_SLEEP_TIME_MS), K_NO_WAIT);
}

void init_led(){
    k_timer_init(&timer_led_on, turn_led_on, NULL);
    k_timer_init(&timer_led_off, turn_led_off, NULL);
}

void led_blink_times(const struct device* led, uint8_t times)
{
    blink_count = 0;
    blink_n_times = times;
    turn_led_on();
}

