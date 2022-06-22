#include "led.h"

/*
bool init_LED(struct device* led_0, struct device* led_1) {

    led_blue_dev = led_0;
    led_red_dev = led_1;

    gpio_pin_configure(led_blue_dev, LED0_PIN, GPIO_OUTPUT | LED0_FLAGS);
    gpio_pin_configure(led_red_dev, LED1_PIN, GPIO_OUTPUT | LED1_FLAGS);

	//turnoff_LEDs();
	gpio_pin_set(led_blue_dev,LED0_PIN,GPIO_OUT_PIN0_High);
    gpio_pin_set(led_red_dev,LED1_PIN,GPIO_OUT_PIN1_High);
    return true;
}

bool blink_LED(uint8_t ID, uint8_t count) {
    for (uint8_t i = 0; i < count*2; i++)
    {
        if (ID == LED_BLUE_ID)
        {
            gpio_pin_toggle(led_blue_dev, LED0_PIN);
        }
        if (ID == LED_RED_ID)
        { 
            gpio_pin_toggle(led_red_dev, LED1_PIN);
        }
        
        k_msleep(300);
    }
    return true;
}
*/