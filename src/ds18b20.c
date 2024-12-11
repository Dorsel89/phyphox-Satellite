#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

#include "ds18b20.h"

// OneWire commands
#define STARTCONVO      0x44  // Tells device to take a temperature reading and put it on the scratchpad
#define COPYSCRATCH     0x48  // Copy scratchpad to EEPROM
#define READSCRATCH     0xBE  // Read from scratchpad
#define WRITESCRATCH    0x4E  // Write to scratchpad
#define RECALLSCRATCH   0xB8  // Recall from EEPROM to scratchpad
#define READPOWERSUPPLY 0xB4  // Determine if device needs parasite power
#define ALARMSEARCH     0xEC  // Query bus for devices with an alarm condition

// Scratchpad locations
#define TEMP_LSB        0
#define TEMP_MSB        1
#define HIGH_ALARM_TEMP 2
#define LOW_ALARM_TEMP  3
#define CONFIGURATION   4
#define INTERNAL_BYTE   5
#define COUNT_REMAIN    6
#define COUNT_PER_C     7
#define SCRATCHPAD_CRC  8

// DSROM FIELDS
#define DSROM_FAMILY    0
#define DSROM_CRC       7

// Device resolution
#define TEMP_9_BIT  0x1F //  9 bit
#define TEMP_10_BIT 0x3F // 10 bit
#define TEMP_11_BIT 0x5F // 11 bit
#define TEMP_12_BIT 0x7F // 12 bit

#define MAX_CONVERSION_TIMEOUT		750

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

DS18B20 ds18b20_data;
THERMOCOUPLE thermocouple_data;

typedef uint8_t ScratchPad[9];

const struct device * dev;

void searchForDevices(){
    DeviceAddress deviceAddress;
    reset_search();
    
    ds18b20_data.n_ds18b20 = 0;
    thermocouple_data.n_thermocouple = 0;

    while (search(deviceAddress,true))
    {
        if(valid_DS18B20(deviceAddress)){
            memcpy(&ds18b20_Adresses[ds18b20_data.n_ds18b20*8],&deviceAddress[0],8);
            ds18b20_data.n_ds18b20++;
        }
        if (valid_MAX31850(deviceAddress))
        {
            memcpy(&tc_Adresses[thermocouple_data.n_thermocouple*8],&deviceAddress[0],8);
            thermocouple_data.n_thermocouple++;
        }
        
    }
    if(DEBUG){
        printk("Found %d ds18b20 and %d thermocouple.\n",ds18b20_data.n_ds18b20,thermocouple_data.n_thermocouple);
    }
}

/**@brief Function to init onewire.
 */
void init_ds18b20(){
    k_work_init(&work_ds18b20_startConversation, ds18b20_measureTemperature);
    k_work_init(&work_ds18b20_getTemperature, ds18b20_getTemperature);

	k_work_init(&config_work_ds18b20, set_config_ds18b20);
    k_timer_init(&timer_ds18b20_startConversation, submit_ds18b20_measureTemperature, NULL);
    k_timer_init(&timer_ds18b20_getTemperature, submit_ds18b20_getTemperature, NULL);
    //searchForDevices();
}

/**@brief Function to check if device is ds18b20
 */
bool valid_DS18B20(const uint8_t* deviceAddress){
 return deviceAddress[DSROM_FAMILY]==DS18B20MODEL;
}

/**@brief Function to check if device is MAX31850
 */
bool valid_MAX31850(const uint8_t* deviceAddress){
 return deviceAddress[DSROM_FAMILY]==MAX31850;
}

/**@brief Function to reset search parameter.
 */
void reset_search(){
    // reset the search state
    LastDiscrepancy = 0;
    LastDeviceFlag = false;
    LastFamilyDiscrepancy = 0;
    for(int i = 7; ; i--) {
        ROM_NO[i] = 0;
        if ( i == 0) break;
    }
}

/**@brief Function for sending one bit to bus.
 */
void ow_send(char bit)
{
    dev = device_get_binding("GPIO_0");
    gpio_pin_configure(dev, DS_PIN, GPIO_OUTPUT_INACTIVE);

    
    k_busy_wait(5);

    if(bit==1)
        gpio_pin_set(dev, DS_PIN, 1);
    
    k_busy_wait(80);
    
    gpio_pin_set(dev, DS_PIN, 1);
}

/**@brief Function for sending one byte to bus.
 */
void ow_send_byte(char data)
{
    unsigned char i;
    unsigned char x;

    for(i=0;i<8;i++)
    {
      x = data>>i;
      x &= 0x01;
      ow_send(x);
    }

    k_busy_wait(100);
}

/**@brief Function for reading one byte from bus.
 */
int ow_read_byte(void)
{
    int result = 0;
    for (int loop = 0; loop < 8; loop++) {
        result >>= 1;
        if (ow_read_bit()) result |= 0x80;
    }
    return result;
}


/**@brief Function for sending reset pulse.
 */
unsigned char ow_reset(void)
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

/**@brief Function for reading bit.
 */
uint8_t ow_read_bit(void)
{
    /* timings are awkward.. feel free to contribute a propper
    / solution for the slow gpio_pin_configure
    /
    */
    uint8_t r;
    dev = device_get_binding("GPIO_0");
    gpio_pin_configure(dev, DS_PIN, GPIO_OUTPUT_INACTIVE);
    //k_busy_wait(3);
    gpio_pin_configure(dev, DS_PIN, GPIO_INPUT);
    k_busy_wait(1);
    r = gpio_pin_get_raw(dev, DS_PIN);
    k_busy_wait(45);

    return r;
}

uint8_t search(uint8_t *newAddr, bool search_mode /* = true */)
{
   uint8_t id_bit_number;
   uint8_t last_zero, rom_byte_number;
   bool    search_result;
   uint8_t id_bit, cmp_id_bit;

   unsigned char rom_byte_mask, search_direction;

   // initialize for search
   id_bit_number = 1;
   last_zero = 0;
   rom_byte_number = 0;
   rom_byte_mask = 1;
   search_result = false;

   // if the last call was not the last one
   if (!LastDeviceFlag) {
      // 1-Wire reset
      if (!ow_reset()) {
         // reset the search
         LastDiscrepancy = 0;
         LastDeviceFlag = false;
         LastFamilyDiscrepancy = 0;
         return false;
      }

      // issue the search command
      if (search_mode == true) {
        ow_send_byte(0xF0);   // NORMAL SEARCH
      } else {
        ow_send_byte(0xEC);   // CONDITIONAL SEARCH
      }

      // loop to do the search
      do
      {
         // read a bit and its complement
         id_bit = ow_read_bit();
         cmp_id_bit = ow_read_bit();

         // check for no devices on 1-wire
         if ((id_bit == 1) && (cmp_id_bit == 1)) {
            break;
         } else {
            // all devices coupled have 0 or 1
            if (id_bit != cmp_id_bit) {
               search_direction = id_bit;  // bit write value for search
            } else {
               // if this discrepancy if before the Last Discrepancy
               // on a previous next then pick the same as last time
               if (id_bit_number < LastDiscrepancy) {
                  search_direction = ((ROM_NO[rom_byte_number] & rom_byte_mask) > 0);
               } else {
                  // if equal to last pick 1, if not then pick 0
                  search_direction = (id_bit_number == LastDiscrepancy);
               }
               // if 0 was picked then record its position in LastZero
               if (search_direction == 0) {
                  last_zero = id_bit_number;

                  // check for Last discrepancy in family
                  if (last_zero < 9)
                     LastFamilyDiscrepancy = last_zero;
               }
            }

            // set or clear the bit in the ROM byte rom_byte_number
            // with mask rom_byte_mask
            if (search_direction == 1)
              ROM_NO[rom_byte_number] |= rom_byte_mask;
            else
              ROM_NO[rom_byte_number] &= ~rom_byte_mask;

            // serial number search direction write bit
            ow_send(search_direction);

            // increment the byte counter id_bit_number
            // and shift the mask rom_byte_mask
            id_bit_number++;
            rom_byte_mask <<= 1;

            // if the mask is 0 then go to new SerialNum byte rom_byte_number and reset mask
            if (rom_byte_mask == 0) {
                rom_byte_number++;
                rom_byte_mask = 1;
            }
         }
      }
      while(rom_byte_number < 8);  // loop until through all ROM bytes 0-7

      // if the search was successful then
      if (!(id_bit_number < 65)) {
         // search successful so set LastDiscrepancy,LastDeviceFlag,search_result
         LastDiscrepancy = last_zero;

         // check for last device
         if (LastDiscrepancy == 0) {
            LastDeviceFlag = true;
         }
         search_result = true;
      }
   }

   // if no device found then reset counters so next 'search' will be like a first
   if (!search_result || !ROM_NO[0]) {
      LastDiscrepancy = 0;
      LastDeviceFlag = false;
      LastFamilyDiscrepancy = 0;
      search_result = false;
   } else {
      for (int i = 0; i < 8; i++) newAddr[i] = ROM_NO[i];
   }
   return search_result;
  }

void ds18b20_getTemperature(){  
    // get temperature data from all ds18b20
    for(int deviceNumber = 0; deviceNumber < ds18b20_data.n_ds18b20 ; deviceNumber++){
        uint8_t dataBuffer[9];   
        
        memcpy(&ROM_NO[0],&ds18b20_Adresses[8*deviceNumber],8);
        ow_reset();
        select(ROM_NO);

        ow_send_byte(0xBE);

        for (int i=0;i<9;i++){
            dataBuffer[i]=ow_read_byte();
            }
        uint8_t crc_calc = dsCRC8(dataBuffer,8);  
        if(crc_calc!=dataBuffer[8]){
            //crc values does not match
            if(DEBUG){
                printk("WARNING: DS18B20 CRC do not match: ");
                printk("crc calculated: %i crc received: %i \n",crc_calc,dataBuffer[8]);
            }
            return 0.0/0.0;
        }  
        uint16_t buffer[1] = {0};
        memcpy(&buffer[0], &dataBuffer[0], 2);
        if(buffer[0]==65535){
            if (DEBUG){
                printk("error ds18b20 data");
            }
            return 0.0/0.0;
        }
        ds18b20_data.temperature[deviceNumber] = (float)0.0625 * buffer[0];
        if(PRINT_SENSOR_DATA){
            printk("temperature of ds18b20 %d: %f\n",deviceNumber,ds18b20_data.temperature[deviceNumber]);
        }
        k_busy_wait(10);
        
    }
    //send all temperature data
    ds18b20_data.timestamp = (float)k_uptime_get() /1000.0;
    ds18b20_data.array[0] = ds18b20_data.timestamp;
    for (int deviceNumber = 0; deviceNumber < ds18b20_data.n_ds18b20; deviceNumber++){
        ds18b20_data.array[deviceNumber+1]=ds18b20_data.temperature[deviceNumber];
    }
    send_data(SENSOR_DS18B20_ID, &ds18b20_data.array, 4*(1+ds18b20_data.n_ds18b20));
}
void ds18b20_measureTemperature(){
    for(uint8_t deviceNumber = 0; deviceNumber<ds18b20_data.n_ds18b20;deviceNumber++){
        memcpy(&ROM_NO[0],&ds18b20_Adresses[8*deviceNumber],8);
        ow_reset();
        select(ROM_NO);
        ow_send_byte(0x44);
        k_busy_wait(10);
    }
}

void select(const uint8_t rom[8]){
    uint8_t i;
    ow_send_byte(0x55);           // Choose ROM
    for (i = 0; i < 8; i++) ow_send_byte(rom[i]);
}

extern void sleep_ds18b20(bool sleep){
    if (sleep) {
        k_timer_stop(&timer_ds18b20_getTemperature);
        k_timer_stop(&timer_ds18b20_startConversation);
        if(DEBUG){
            printk("DS18B20: sleeping\n");
        }
    }
}

uint8_t dsCRC8(const uint8_t *addr, uint8_t len){
  uint8_t crc = 0;
  while (len--)
  {
    uint8_t inbyte = *addr++;
    for (uint8_t i = 8; i; i--)
    {
      uint8_t mix = (crc ^ inbyte) & 0x01;
      crc >>= 1;
      if (mix) crc ^= 0x8C;
      inbyte >>= 1;
    }
  }
  return crc;
}

void set_config_ds18b20(){
    sleep_ds18b20(true);
    if(DEBUG){
        if(ds18b20_data.config[0]==0){
            printk("DS18B20: sleep\n");
        }else{
            printk("DS18B20: enable\n");
        }
    }
    if(ds18b20_data.config[0]==false){
        sleep_ds18b20(true);
        return;
    }
    if(ds18b20_data.config[2]>=2){
        //SHOULD BE FIXED
        //do max 5 searches to find target number of probes
        for(int i=0;i<5;i++){
            searchForDevices();
            if(ds18b20_data.n_ds18b20==ds18b20_data.config[2]){
                break;
            }
        }

    }else{
        searchForDevices();
    }
    
    ds18b20_data.timer_interval = ds18b20_data.config[1]*1000; //in ms
    if(ds18b20_data.config[1]==0){
        ds18b20_data.timer_interval=850;//go to fastest measurement interval if user choose 0
    }
    int conversionTime = 800; //in ms; TODO: change to corrrect timing in respect to selected resolution
    k_timer_start(&timer_ds18b20_startConversation, K_NO_WAIT, K_MSEC(ds18b20_data.timer_interval));
    k_timer_start(&timer_ds18b20_getTemperature, K_MSEC(conversionTime), K_MSEC(ds18b20_data.timer_interval));
}

extern void submit_config_ds18b20(){	
    k_work_submit(&config_work_ds18b20);
}

void submit_ds18b20_measureTemperature(){
	k_work_submit(&work_ds18b20_startConversation);
}

void submit_ds18b20_getTemperature(){
	k_work_submit(&work_ds18b20_getTemperature);
}