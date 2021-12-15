# phyphox-Satellite
phyphox sensorbox firmware based on zephyr

currently working
- ble server with bmp characteristic
- debugging over rtt
- BMP384
  - driver
  - gpio data ready
  - sensor can be configured via ble
  - data is send via ble
- mlx
  - driver
  - data reader interrupt
