#include "ads1231.h"
#include <drivers/gpio.h>

ADS1231 ads1231_data = {
                        .offset = 0,
                        .calibrationWeight = 1080000,
                        .unweighted = 206921,
                        .weighted = 766401
                        };
const struct device * dev_ADS1231;


void init_ads1231(){
    dev_ADS1231 = device_get_binding("GPIO_0");
    gpio_pin_configure(dev_ADS1231, SCL_PIN, GPIO_OUTPUT);
    gpio_pin_configure(dev_ADS1231, PWDN_PIN, GPIO_OUTPUT);
    gpio_pin_configure(dev_ADS1231, SPEED_PIN, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure(dev_ADS1231, DOUT_PIN, GPIO_INPUT);
    sleep_ads1231(true);
}

void sleep_ads1231(bool sleep){
    gpio_pin_set(dev_ADS1231,PWDN_PIN,!sleep);
    gpio_pin_set(dev_ADS1231,SCL_PIN,0);
}

long read_loadcell(){
    long adcvalue = 0, digit = 0;
    int i=0;
    for(i=0; i<24; i++){						// 24 pulses
            gpio_pin_set(dev_ADS1231,SCL_PIN,1);
            k_busy_wait(1);
            digit = gpio_pin_get_raw(dev_ADS1231,DOUT_PIN);
            adcvalue = (adcvalue << 1) + digit;
            gpio_pin_set(dev_ADS1231,SCL_PIN,0);
            k_busy_wait(1);
    }
    adcvalue = (adcvalue << 8) / 256;
    gpio_pin_set(dev_ADS1231,SCL_PIN,1);
    k_busy_wait(1);
    gpio_pin_set(dev_ADS1231,SCL_PIN,0);
    return adcvalue;
}