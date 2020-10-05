# BH1750 ambient light sensor

## Description
BH1750 ambient light sensor library

## Usage
Simple demo code how to use the library on [ARDUINO] platform
```cpp

#include "BH1750/platform/bh1750-arduino.h"

int main(void)
{
    bh1750 bh1750_dev;
    bh1750_arduino bh1750_dev_arduino;

    bh1750_dev_arduino.i2c = &Wire;
    bh1750_dev.i2c_addr = BH1750_I2C_ADDRESS1;
    bh1750_arduino_init(&dev, &bh1750_dev_arduino);
    bh1750_set_state(&bh1750_dev, BH1750_STATE_POWER_ON);
    bh1750_set_mode(&bh1750_dev, BH1750_MODE_CONT_HIGH_RES);

    float lux;
    bh1750_get_lux(&dev, &lux);
    printf("BH1750 Illuminance value: %3.2f [lx]", lux);
    return 0;
}
```

## Porting to different platforms
If you want to port this library on other platform, the only thing you have to do is to implement HW/Platform dependent functions from `bh1750.h` file as it is done for example for [ARDUINO] platform in `platform/arduino/bh1750-arduino.cpp` file.


## Author
* Lukasz Uszko aka `luk6xff` [lukasz.uszko@gmail.com]