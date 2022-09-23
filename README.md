# phyphox-Satellite
phyphox sensorbox firmware based on zephyr

## current status ##
Basics
- [x] debugging over rtt
- [ ] Firmwareupdae over ble

BMP384
  - [x] driver
  - [x] gpio data ready
  - [x] sensor can be configured via ble
  - [x] data is send via ble
  - [ ] send multiple datasets    
  
MLX90393ELW-ABA-011-RE
  - [x] driver
  - [x] data reader interrupt
  - [x] data is send via ble
  - [ ] sensor can be configured via ble (multiple datasets are missing)   
  - [ ] send multiple datasets  
  
ICM-42605
  - [ ] driver
  - [ ] data reader interrupt
  - [ ] send multiple datasets
  - [ ] data is send via ble  
  - [ ] sensor can be configured via ble      
  
MPRLS0025PA00001A
  - [ ] driver
  - [ ] data reader interrupt
  
1Wire
  - [ ] driver
  - [ ] DS18b20
  - [ ] MAX31850KATB+
  - [ ] data is send via ble
  - [ ] sensor can be configured via ble
  
LED
  - [x] LED is blinking on connection
  - [ ] LED is blinking when fully charged
  
Loadcell ADS1231IDR
  - [ ] driver
  - [ ] can be configured via ble
  - [ ] data can be send via ble
  
Supercap charge level
  - [x] Battery Service
  - [ ] current charge is send via ble

