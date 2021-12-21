#include "shtc3.h"

extern bool initSHTC(struct device *i2c_pointer) {
    char data[3];
    SHTC = i2c_pointer;
    
    // wakeup
    if (!sendCmd(CMD_WAKEUP)) {
        return false;
    }

    k_sleep(K_USEC(240));

    // request ID
    if (!sendCmd(CMD_READ_ID)) {
        return false;
    }

    // read ID
    if (!getData(data, sizeof(data))) {
        printk("Read ID failed\n");
        return false;
    }

    // check CRC
    if (!checkCRC(data, sizeof(data))) {
        return false;
    }

    // check ID type
    if ((data[0] & 0b00001000) != 0b00001000 || (data[1] & 0b00111111) != 0b00000111) {
        printk("ID doesn't match\n");
        return false;
    }

    printk("SHTC3 ID: %04X \n", data[0] | data[1]);

    return sendCmd(CMD_SLEEP);
}

extern bool read(uint16_t *temp, uint16_t *humidity, bool low_power) {
    bool ret = false;
    uint8_t i = 0;
    char data[6];

    // wakeup
    if (!sendCmd(CMD_WAKEUP)) {
        return false;
    }

    k_sleep(K_USEC(240));

    // request measurement
    shtc3_cmd_t cmd = (low_power ? CMD_MEASUREMENT_LOW_POWER : CMD_MEASUREMENT_NORMAL);
 
    if (!low_power) {
        k_sleep(K_MSEC(10)); // "normal" measuring time is 10-12ms
        
    }

    // read results
    for (; i < CONF_SHTC3_TIMEOUT; i++) { // safety timeout
        if (getData(data, sizeof(data))) {
            break;

        } else {
            k_sleep(K_MSEC(1));
        }
    }

    printk("Measuring took %ums \n", i);

    // check CRC
    if (!checkCRC(data, sizeof(data))) {
        printk("crc error \n");
    }

    // if we reached up to here, it's a success
    ret = true;

    if (humidity) {
        *humidity = (data[0] << 8) | data[1];
    }

    if (temp) {
        *temp = (data[3] << 8) | data[4];
    }

    // sleep now
    sendCmd(CMD_SLEEP);
    return ret;
}

extern float toCelsius(uint16_t raw) {
    return 175.0 * ((float)raw / 65535.0) - 45.0;
}

static float toFahrenheit(uint16_t raw) {
    return toCelsius(raw) * 9.0 / 5.0 + 32.0;
}

extern float toPercentage(uint16_t raw) {
    return 100.0 * ((float)raw / 65535.0);
}

static void reset() {
    sendCmd(CMD_RESET);
    k_sleep(K_USEC(240));
}

static bool sendCmd(shtc3_cmd_t cmd) {
    printk("Sending CMD: %04X \n", (uint16_t)cmd);
    char data[2];
    data[0] = (uint16_t)cmd >> 8;
    data[1] = (uint16_t)cmd & UCHAR_MAX;

    int ack = -1;

    ack = i2c_write(SHTC, &data, 2, _address);

    if (ack != 0) {
        printk("Write failed \n");
        return false;
    }

    return true;
}

static bool getData(char *data, size_t len) {
    int ack = -1;
	
    ack = i2c_read(SHTC, data, len, _address);
    
    if (ack != 0) {
        return false;
        printk("Read data error\n");
    }

    

    return true;
}

static bool checkCRC(const char *data, size_t len) {
    return true;
}