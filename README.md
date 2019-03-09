# SVCE Ebike Controller

## Description

STM32 Blue Pill (*STM32 F103C8*) powered Ebike controller made for the [Silicon Valley Clean Energy](https://www.svcleanenergy.org/) Ebike Competition. This program is developed in conunction with the [PlatformIO](https://platformio.org/) system, a substantial upgrade from the oxymoron Arduino IDE.

Relevant Peripherals:

- MPU6050 6DOF Accelerometer and Gyroscope [(*in GY521 breakout board via I2C*)](https://smile.amazon.com/HiLetgo-MPU-6050-Accelerometer-Gyroscope-Converter/dp/B00LP25V1A)
- [VESC 4.12](https://smile.amazon.com/HGLRC-FLIPSKY-SK8-ESC-Electric-Skateboard-EScooter/dp/B07GFB55NV) via UART communication
- [SD Card breakout board](https://smile.amazon.com/SenMod-Adapter-Reader-Module-Arduino/dp/B01JYNEX56) via SPI

## Libraries

- [`MPU6050_tockn` by @tockn](https://platformio.org/lib/show/2824/MPU6050_tockn)
- [`VescUart` by SolidGeek](https://platformio.org/lib/show/5830/VescUart)
- [`SD` by Arduino, SparkFun](https://platformio.org/lib/show/868/SD)

### Necessary Modifications

Unfortunately, at the time of writing, the stock `MPU6050_tockn` library has [an issue when built for the STM32 F103C8](https://community.platformio.org/t/stm32f1-compilation-erring-upon-calling-wire-library/6911/13). To fix this, the calls to `wire->requestFrom` in the library's code must be changed by removing the last `(int)true` parameter wherever it appears.

*i.e.*: `wire->requestFrom((int)MPU6050_ADDR, 14, (int) true);` --> `wire->requestFrom((int)MPU6050_ADDR, 14);`