#include <kernel.h>
#include <drivers/gpio.h>

#include "ds18b20.h"

// #define DS_PIN 26
#define DS_PIN 20

// Commands
#define STARTCONVO      0x44
#define READSCRATCH     0xBE
#define WRITESCRATCH    0x4E

// Scratchpad locations
#define TEMP_LSB        0
#define TEMP_MSB        1

// Device resolution
#define TEMP_9_BIT  0x1F //  9 bit
#define TEMP_10_BIT 0x3F // 10 bit
#define TEMP_11_BIT 0x5F // 11 bit
#define TEMP_12_BIT 0x7F // 12 bit

typedef uint8_t ScratchPad[9];

const struct device * dev;

/**@brief Custome timer.
 */
void timer(uint64_t iTimeout)
{
    uint64_t iUptime = k_uptime_ticks();

    while ((k_uptime_ticks() - iUptime) < k_us_to_ticks_near64(iTimeout));
    
}


/**@brief Function for sending one bit to bus.
 */
void ds18b20_send(char bit)
{
    dev = device_get_binding("GPIO_0");
    gpio_pin_configure(dev, DS_PIN, GPIO_OUTPUT_INACTIVE);

    
    k_busy_wait(5);

    if(bit==1)
        gpio_pin_set(dev, DS_PIN, 1);
    
    k_busy_wait(80);
    
    gpio_pin_set(dev, DS_PIN, 1);
}


/**@brief Function for reading one bit from bus.
 */
unsigned char ds18b20_read(void)
{
    unsigned char presence=0;

    dev = device_get_binding("GPIO_0");
    gpio_pin_configure(dev, DS_PIN, GPIO_OUTPUT_INACTIVE);

    k_busy_wait(2);

    gpio_pin_set(dev, DS_PIN, 1);
    k_busy_wait(15);

    gpio_pin_configure(dev, DS_PIN, GPIO_INPUT);

    if(gpio_pin_get(dev, DS_PIN))
        presence = 1;
    
    else
        presence = 0;
    
    
    return presence;
}


/**@brief Function for sending one byte to bus.
 */
void ds18b20_send_byte(char data)
{
    unsigned char i;
    unsigned char x;

    for(i=0;i<8;i++)
    {
      x = data>>i;
      x &= 0x01;
      ds18b20_send(x);
    }

    k_busy_wait(100);
}


/**@brief Function for reading one byte from bus.
 */
unsigned char ds18b20_read_byte(void)
{
    unsigned char i;
    unsigned char data = 0;

    for (i=0;i<8;i++)
    {
        if(ds18b20_read()) data|=0x01<<i;
        k_busy_wait(15);
    }

    return(data);
}


/**@brief Function for sending reset pulse.
 */
unsigned char ds18b20_reset(void)
{
    unsigned char presence;

    dev = device_get_binding("GPIO_0");
    gpio_pin_configure(dev, DS_PIN, GPIO_OUTPUT_INACTIVE);

    k_busy_wait(500);
    
    gpio_pin_set(dev, DS_PIN, 1);

    gpio_pin_configure(dev, DS_PIN, GPIO_INPUT);
    k_busy_wait(30);


    if(gpio_pin_get(dev, DS_PIN) == 0)    
        presence = 1;   
    else    
        presence = 0;
    
    k_busy_wait(470);

    if(gpio_pin_get(dev, DS_PIN) == 1)    
        presence = 1;   
    else  
        presence = 0;

    return presence;
}


/**@brief Function for reading temperature.
 */
float ds18b20_get_temp(void)
{
    unsigned char check;
    char temp1=0, temp2=0;

    check=ds18b20_reset();
    
    if(check)
    {
        ds18b20_send_byte(0xCC);
        ds18b20_send_byte(0x44);

        k_busy_wait(600);

        check=ds18b20_reset();
        ds18b20_send_byte(0xCC);
        ds18b20_send_byte(0xBE);

        temp1=ds18b20_read_byte();
        temp2=ds18b20_read_byte();

        check=ds18b20_reset();

        float temp=0;
        temp=(float)(temp1+(temp2*256))/16;

        return temp;
    }
      return 0;
}


/**@brief Function for reading bit.
 */
uint8_t OneWire_read_bit(void)
{
    uint8_t r;

    dev = device_get_binding("GPIO_0");
    gpio_pin_configure(dev, DS_PIN, GPIO_OUTPUT_INACTIVE);

    k_busy_wait(3);

    gpio_pin_configure(dev, DS_PIN, GPIO_INPUT);

    k_busy_wait(10);

    r = gpio_pin_get(dev, DS_PIN);

    k_busy_wait(53);

    return r;
}


/**@brief Function for reading.
 */
uint8_t OneWire_read()
{
    uint8_t bitMask;
    uint8_t r = 0;

    for (bitMask = 0x01; bitMask; bitMask <<= 1)
	    if ( OneWire_read_bit()) r |= bitMask;
    
    return r;
}


/**@brief Function for reading scratchpad value
 */
void ds18b20_readScratchPad(uint8_t *scratchPad, uint8_t fields)
{
    ds18b20_reset();

    ds18b20_send_byte(0xCC);
    ds18b20_send_byte(READSCRATCH);

    for(uint8_t i=0; i < fields; i++)
        scratchPad[i] = OneWire_read();
    
    ds18b20_reset();
}


/**@brief Function for request temperature reading
 */
void ds18b20_requestTemperatures(void)
{
    ds18b20_reset();
    ds18b20_send_byte(0xCC);
    ds18b20_send_byte(STARTCONVO);
}


/**@brief Function for reading temperature method 2
 */
float ds18b20_get_temp_method_2(void)
{
    ds18b20_requestTemperatures();

    ScratchPad scratchPad;
    ds18b20_readScratchPad(scratchPad, 2);

    int16_t rawTemperature = (((int16_t)scratchPad[TEMP_MSB]) << 8) | scratchPad[TEMP_LSB];
    float temp = 0.0625 * rawTemperature;

    return temp;
}


/**@brief Function for setting temperature resolution
 */
void ds18b20_setResolution(uint8_t resolution)
{
    ds18b20_reset();

    ds18b20_send_byte(0xCC);
    ds18b20_send_byte(WRITESCRATCH);
    
    // two dummy values for LOW & HIGH ALARM
    ds18b20_send_byte(0);
    ds18b20_send_byte(100);

    switch (resolution)
    {
        case 12:
            ds18b20_send_byte(TEMP_12_BIT);
            break;

        case 11:
            ds18b20_send_byte(TEMP_11_BIT);
            break;

        case 10:
            ds18b20_send_byte(TEMP_10_BIT);
            break;

        case 9:
        default:
            ds18b20_send_byte(TEMP_9_BIT);
            break;
    }

    ds18b20_reset();
}