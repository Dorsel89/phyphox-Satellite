# phyphox Satellite-Board
1. [phyphox-Experiments](#qr)
2. [General Informations](#info)
3. [Sensor Configuration](#config)
3. [Firmwareupdate](#firmware)

## 1. phyphox-Experiments <a name="qr"></a>
The following qr-code contains an phyphox-experiment collection with the following categories/experiments
- Satellite Sensors (simple readout with a fixed sensor configuration)
- Satellite Sensors Configurable (for advanced users who want to set sensor specific settings)
All experiments are in the experiments-folder of this repository

## 2. General Informations<a name="info"></a>
The phyphox-satellite board contains the following sensors
- Pressure, temperature ([BMP384](#BMP384))
- Temperature, relative humidity ([SHTC3](#temperaturehumidity-shtc3))
- Acceleration, gyrocsope ([ICM42605](#accelerometergyroscope-icm42605))
- Magnetometer ([MLX90393](#magnetometer-mlx90393))

Additional a thermodynamics-box can be connected with a
- [Thermocouple-Connector](#thermocouple) (Temperature)
- [DS18B20-Connector](#1wire-ds18b20) (Temperature)
- Pressure-Sensor ([MPRLS](#pressure-mprls))

## 3. Sensor Configuration <a name="config"></a>
### Magnetometer [MLX90393](https://media.melexis.com/-/media/files/documents/datasheets/mlx90393-datasheet-melexis.pdf)
 <a name="MLX90393"></a>
charateristic | uuid/type of value
--------------|-----
data (repeatable)         | cddf1009-30f7-4671-8b43-5e40ba53514a
byte 0-3          | x-axis (float32LittleEndian)
byte 4-7          | y-axis (float32LittleEndian)
byte 8-11         | z-axis (float32LittleEndian)
byte 12-15        | timestamp (float32LittleEndian)
config        | cddf100a-30f7-4671-8b43-5e40ba53514a

Byte | Settings
-----|---------
0    | enable magnetometer
1    | uint8_t gain-mode: <br> 0: 5x <br> 1: 4x <br> 2: 3x <br> 3: 2.5x <br> 4: 2x <br> 5: 1.667x <br> 6: 1.333x <br> 7: 1x
2    | uint8_t digital filter: 0-7
3    | uint8_t oversampling: 0-3
4    | uint8_t x resolution: 0-3
5    | uint8_t y resolution: 0-3
6    | uint8_t z resolution: 0-3
7    | number of samples in one ble-package, max 10 with mtu-size="185" (uint8_t)

### Accelerometer/Gyroscope [ICM42605](https://invensense.tdk.com/wp-content/uploads/2022/09/DS-000292-ICM-42605-v1.7.pdf)<a name="ICM42605"></a> ### 
charateristic | uuid/type of value
--------------|-----
acc data          | cddf1002-30f7-4671-8b43-5e40ba53514a
byte 0-3          | x-axis (float32LittleEndian)
byte 4-7          | y-axis (float32LittleEndian)
byte 8-11         | z-axis (float32LittleEndian)
byte 12-15        | timestamp (float32LittleEndian)
gyr data          | cddf1003-30f7-4671-8b43-5e40ba53514a
byte 0-3          | x-axis (float32LittleEndian)
byte 4-7          | y-axis (float32LittleEndian)
byte 8-11         | z-axis (float32LittleEndian)
byte 12-15        | timestamp (float32LittleEndian)
config        | cddf1004-30f7-4671-8b43-5e40ba53514a

Byte | Settings
-----|---------
0    | 0: disable IMU <br> 1: enable IMU
1    | uint8_t gyroscope range <br> 2000DPS: 0x00 <br> 1000DPS: 0x01 <br> 500DPS: 0x02 <br> 250DPS: 0x03 <br> 125DPS: 0x04 <br> 62.5DPS: 0x05 <br> 31.25DPS: 0x06 <br> 15.125DPS: 0x07
2    | uint8_t accelerometer range <br> 16G: 0x00 <br> 8G: 0x01 <br> 4G: 0x02 <br> 2G: 0x03
3    | uint8_t rate <br> 8000Hz: 0x03 <br> 4000Hz: 0x04 <br> 2000Hz: 0x05 <br> 1000Hz: 0x06 <br> 200Hz: 0x07 <br> 100Hz: 0x08 <br> 50Hz: 0x09 <br> 25Hz: 0x0A <br> 6.25Hz: 0x0B <br> 6.25Hz: 0x0C <br> 3.12Hz: 0x0D <br> 1.5625Hz: 0x0E <br> 500Hz: 0x0F
4    | number of samples in one ble-package, max 11 with mtu-size="185" (uint8_t)

### Temperature/Humidity [SHTC3](https://www.mouser.com/datasheet/2/682/Sensirion_04202018_HT_DS_SHTC3_Preliminiary_D2-1323493.pdf)<a name="SHTC3"></a> ###
charateristic | uuid/type of value
--------------|-----
data          | cddf1005-30f7-4671-8b43-5e40ba53514a
byte 0-3          | temperature (float32LittleEndian)
byte 4-7          | humidity (float32LittleEndian)
byte 8-11         | timestamp (float32LittleEndian)
config        | cddf1006-30f7-4671-8b43-5e40ba53514a

Byte | Settings
-----|---------
0    | uint8_t enable sensor
1    | uint8_t measurement interval in 10ms (0x03 equals 30ms measurement interval)

###  Pressure/Temperature [BMP384](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp384-ds003.pdf)<a name="BMP384"></a> ###
charateristic | uuid/type of value
--------------|-----
data          | cddf1007-30f7-4671-8b43-5e40ba53514a
byte 0-3          | pressure (float32LittleEndian)
byte 4-7          | temperature (float32LittleEndian)
byte 8-11         | timestamp (float32LittleEndian)
config        | cddf1008-30f7-4671-8b43-5e40ba53514a

Byte | Settings
-----|---------
0    | 0x00: disable BMP384 <br> 0x01: enable BMP384
1    | uint8_t pressure oversampling: <br> no oversampling 0x00 <br> 2x: 0x01 <br> 4x: 0x02 <br> 8x: 0x03 <br> 16x: 0x04 <br> 32x: 0x05 <br>
3    | uint8_t iir filter coefficient: <br> disable: 0x00 <br> 1: 0x01 <br> 3: 0x02 <br> 7: 0x03 <br> 15: 0x04 <br> 31: 0x05 <br> 63: 0x06 <br> 127: 0x07
4    | uint8_t data rate: <br> 200Hz: 0x00 <br> 100Hz: 0x01 <br> 50Hz: 0x02 <br> 25Hz: 0x03 <br> 12.5Hz: 0x04 <br> 6.25Hz: 0x05 <br> 3.1Hz: 0x06 <br> 1.5Hz: 0x07 <br> 0.78Hz: 0x08 <br> 0.39Hz: 0x09 <br> 0_2Hz: 0x0A

### Thermocouple<a name="Thermocouple"></a> ###
charateristic | uuid/type of value
--------------|-----
data          | cddf100f-30f7-4671-8b43-5e40ba53514a
byte 0-3          | temperature (float32LittleEndian)
byte 4-7          | timestamp (float32LittleEndian)
config        | cddf1010-30f7-4671-8b43-5e40ba53514a

Byte | Settings
-----|---------
0    | uint8_t enable sensor
1    | uint8_t measurement interval in 100ms (0x03 equals 300ms measurement interval)

### 1Wire [DS18B20](https://www.analog.com/media/en/technical-documentation/data-sheets/ds18b20.pdf)<a name="DS18B20"></a> ###
charateristic | uuid/type of value
--------------|-----
data          | cddf1011-30f7-4671-8b43-5e40ba53514a
byte 0-3          | temperature (float32LittleEndian)
byte 4-7          | timestamp (float32LittleEndian)
config        | cddf1012-30f7-4671-8b43-5e40ba53514a

Byte | Settings
-----|---------
0    | uint8_t enable sensor
1    | uint8_t measurement interval in 1000ms (0x03 equals 3000ms measurement interval, 0x00 represents the minimal measurement interval of 750ms)
2    | uint8_t number of connected probes
### Pressure ([MPRLS](https://prod-edam.honeywell.com/content/dam/honeywell-edam/sps/siot/es-mx/products/sensors/pressure-sensors/board-mount-pressure-sensors/micropressure-mpr-series/documents/sps-siot-mpr-series-datasheet-32332628-ciid-172626.pdf))<a name="MPRLS"></a> ###
charateristic | uuid/type of value
--------------|-----
data          | cddf100d-30f7-4671-8b43-5e40ba53514a
byte 0-3          | pressure (float32LittleEndian)
byte 4-7          | timestamp (float32LittleEndian)
config        | cddf100e-30f7-4671-8b43-5e40ba53514a

Byte | Settings
-----|---------
0    | uint8_t enable sensor
1    | uint8_t measurement interval in 10ms (0x03 equals 30ms measurement interval)

## 4. Firmwareupdate <a name="firmware"></a>
An compiled firmware can be found in the *Firmware* Folder.
### TODO: How to update the firmware via nrf connect

