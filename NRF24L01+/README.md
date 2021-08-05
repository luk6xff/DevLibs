# NRF24L01+ radio module library


## Description
The library was tested with mbed and stm32_cube frameworks.

## Usage
Exampel to be found here: https://github.com/luk6xff/GarageDoorKeypad/blob/master/common/Radio/radio.c


## Porting to other platform
If you want to port this library on other platform, the only thing you have to do is define HW/Platform dependent functions as it is done in `platform/stm32cube/nrf24l01p-cube.c` file.


## Author
* Lukasz Uszko aka `luk6xff` [lukasz.uszko@gmail.com]