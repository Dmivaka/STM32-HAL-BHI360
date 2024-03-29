
#  STM32 HAL BHI360
An example of STM32Cube HAL project for communication with Bosch BHI360 IMU via I2C. 
The goal of this repo is to show how to use [BHY2-Sensor-API](https://github.com/boschsensortec/BHY2-Sensor-API) with the STM32 family MCUs and Cube software. The code is stripped of the COINES abstraction layer and built around the STM32 HAL instead. 
The project is configured for use with NUCLEO-G474RE board, but could be easily portable to any STM32 chip. 

This example shows how to configure and use three BHI360 virtual sensors: Accelerometer, Gyroscope and Euler angle sensor. For other usage examples please refer to BHY2 repository. The provided software also should work with other IMUs like BHI260, but it is untested. 

## Building
Open .ioc file in the [STM32CubeMX](https://www.st.com/en/development-tools/stm32cubemx.html) and generate source files for your preffered IDE. Then just add **`bhy2.c, bhy2_hif.c, bhy2_parse.c, common.c, bosch_imu.c `** into the sources and **`/bosch, /bosch/common`** into the search directories.

## Acknowledgements
Big thanks to [SensorLib](https://github.com/lewisxhe/SensorLib) for providing great examples of how to cook BHY2. 
