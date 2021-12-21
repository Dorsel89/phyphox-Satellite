
#ifndef SHTC3_H
#define SHTC3_H

#include <stdint.h>
#include <drivers/i2c.h>
#include <drivers/gpio.h>
#include "workQueue.h"
#include "phyphox.h"

#define CONF_SHTC3_TIMEOUT 20
#define SHTC3_ADDRESS 0x70

    typedef enum {
        CMD_WAKEUP = 0x3517,
        CMD_RESET = 0x805D,
        CMD_SLEEP = 0xB098,
        CMD_READ_ID = 0xEFC8,
        CMD_MEASUREMENT_NORMAL = 0x58E0,
        CMD_MEASUREMENT_LOW_POWER = 0x401A,
    } shtc3_cmd_t;

    extern bool initSHTC(struct device *i2c_pointer);
    static struct device *SHTC;
    extern bool read(uint16_t *temp, uint16_t *humidity, bool low_power);

    static void reset();

    extern float toCelsius(uint16_t raw);

    static float toFahrenheit(uint16_t raw);
    extern float toPercentage(uint16_t raw);

    static bool sendCmd(shtc3_cmd_t cmd);
    static bool checkCRC(const char *data, size_t len);
    static bool getData(char *data, size_t len);

    static const int8_t _address = SHTC3_ADDRESS;





#endif  // SHTC3_H