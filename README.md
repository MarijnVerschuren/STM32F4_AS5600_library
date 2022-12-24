# STM32F4_AS5600_library
 AS5600 library for STM32F4

How to use:
1. make a new instance
```
AS5600_TypeDef* sensor = AS5600_new();
```
2. set the i2c_handle, dir_port and dir_pin
```
sensor->i2c_handle = &hi2c1;
sensor->dir_port = dir_GPIO_Port;
sensor->dir_pin = dir_Pin;
```
3. initalize the sensor
```
AS5600_init(sensor);
```
4. read using i2c
```
uint16_t angle;
AS5600_get_angle(sensor, &angle);
```
