# AT24CXX eeprom modules library


## Description
The library supports: AT24C01, AT24C02, AT24C04, AT24C08, AT24C16, AT24C32, AT24C64, AT24C128, AT24C256, AT24C512 and AT24C1024 devices.
It was tested with mbed and stm32_cube frameworks.

## Usage
Example to be found here: https://github.com/luk6xff/GarageDoorKeypad/blob/master/common/Storage/eeprom.c


## Porting to other platform
If you want to port this library on other platform, the only thing you have to do is define HW/Platform dependent functions as it is done in `platform/stm32cube/at24cxx-cube.c` file.


## Author
* Lukasz Uszko aka `luk6xff` [lukasz.uszko@gmail.com]