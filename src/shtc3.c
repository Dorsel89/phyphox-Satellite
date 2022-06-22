#include "shtc3.h"

extern bool initSHTC(struct device *i2c_pointer) {
    if (DEBUG_MODE){printk("SHTC initialising...\n");}

    char data[3];
    SHTC_dev = i2c_pointer;
    
    // wakeup
    if (DEBUG_MODE){printk("SHTC waking up...\n");}
    if (!sendCmd(CMD_WAKEUP)) {
        printk("SHTC Wake up failed\n");
        return false;
    }

    if (DEBUG_MODE){printk("SHTC initialised succesfully...\n");}

    k_sleep(K_USEC(240));

    // request ID
    if (DEBUG_MODE){printk("SHTC requesting ID...\n");}
    if (!sendCmd(CMD_READ_ID)) {
        printk("SHTC requesting ID failed\n");
        return false;
    }
    if (DEBUG_MODE){printk("SHTC requesting ID succesfully\n");}

    // read ID
    if (DEBUG_MODE){printk("SHTC reading ID...\n");}
    if (!getData(data, sizeof(data))) {
        printk("Read ID failed\n");
        return false;
    }
    if (DEBUG_MODE){printk("SHTC read ID succesfully\n");}

    // check CRC
    if (DEBUG_MODE){printk("SHTC checking CRC...\n");}
    if (!checkCRC(data, sizeof(data))) {
        printk("SHTC requesting ID...\n");
        return false;
    }
    if(DEBUG_MODE){printk("SHTC CRC checked succesfully\n");}


    // check ID type
    if (DEBUG_MODE){printk("SHTC checking ID...\n");}
    if ((data[0] & 0b00001000) != 0b00001000 || (data[1] & 0b00111111) != 0b00000111) {
        printk("ID doesn't match\n");
        return false;
    }
    if (DEBUG_MODE){
        printk("SHTC ID checked successfully\n");
        printk("SHTC ID: %04X \n", data[0] | data[1]);
    }

    if (DEBUG_MODE){printk("SHTC init successfull. Putting SHTC back to sleep\n");}
    return sendCmd(CMD_SLEEP);

    //set default power mode
    power_mode = NORMAL;
}

extern bool shtc_read(uint16_t *temp, uint16_t *humidity, bool low_power) {
    if (DEBUG_MODE){printk("SHTC reading...\n");}
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
    sendCmd(cmd);
 
    if (!low_power) {
        k_sleep(K_MSEC(12)); // "normal" measuring time is 10-12ms
        
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

float toCelsius(uint16_t raw) {
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
    if (DEBUG_MODE){printk("Sending CMD to SHTC: %04X \n", (uint16_t)cmd);}
    char data[2];
    data[0] = (uint16_t)cmd >> 8;
    data[1] = (uint16_t)cmd & UCHAR_MAX;

    int ack = -1;

    ack = i2c_write(SHTC_dev, &data, 2, _address);

    if (ack != 0) {
        printk("Write failed \n");
        return false;
    }

    return true;
}

static bool getData(char *data, size_t len) {
    int ack = -1;
	
    ack = i2c_read(SHTC_dev, data, len, _address);
    
    if (ack != 0) {
        return false;
        printk("Read data error\n");
    }

    return true;
}

static bool checkCRC(const char *data, size_t len) {
    return true;
}

extern uint8_t sleepSHTC(bool SLEEP) {
    if (SLEEP) {
        if (DEBUG_MODE){printk("Stopping timer \n");}
        int8_t error =0;
        k_timer_stop(&timer_shtc);
        error = sendCmd(CMD_SLEEP);
        return error;
    }
    else {
        if (DEBUG_MODE){printk("Setting Timer with interval of %i \n", shtcData.timer_interval);}
        int8_t error =0;
        error = sendCmd(CMD_WAKEUP);
        k_timer_start(&timer_shtc, K_MSEC(shtcData.timer_interval*10), K_MSEC(shtcData.timer_interval*10));
        return error;
    }
}

extern void sendDataSHTC(void){
    if(shtc_read(&raw_temp,&raw_humidity,power_mode)) {

        shtcData.temperature = toCelsius(raw_temp);
        shtcData.humidity = toPercentage(raw_humidity);

        if(PRINT_SENSOR_DATA){
		    printk("SHTC: temp: %f humid: %f\n", shtcData.temperature,shtcData.humidity);
	    }

        float timestamp = k_uptime_get() /1000.0;
        shtcData.timestamp = timestamp;

        shtcData.array[0] = shtcData.temperature;
        shtcData.array[1] = shtcData.humidity;
        shtcData.array[2] = shtcData.timestamp;

        sendData(SENSOR_SHTC_ID, &shtcData.array, 4*3);
    }
    else {
        printk("Error reading SHTC");
    }
}

static void shtcDataReady(const struct device *dev, struct gpio_callback *cb,uint32_t pins)
{
	k_work_submit(&work_shtc);
}

void init_Interrupt_SHTC() {
    k_work_init(&work_shtc, sendDataSHTC);
	k_work_init(&config_work_shtc, setConfigSHTC);
    k_timer_init(&timer_shtc, shtcDataReady, NULL);
}

extern void submitConfigSHTC(){
	k_work_submit(&config_work_shtc);
}

static void setConfigSHTC(){
    if (DEBUG_MODE) {printk("SHTC Setting config...\n");}
    
    shtcData.timer_interval = shtcData.config[1];
    //Minimum of 30ms
    if (shtcData.timer_interval < 3) {
        shtcData.timer_interval = 3;
    }

    //powermode
    power_mode = shtcData.config[2];

	sleepSHTC(!shtcData.config[0]);
}