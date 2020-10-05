# MAX44009 ambient light sensor

## Description
MAX44009 ambient light sensor library

## Usage
Simple demo code how to use the library on [ARDUINO] platform
```cpp

#include "MAX44009/platform/max44009-arduino.h"


int main(void)
{
    max44009 dev;
    max44009_arduino arduino_dev;

    max44009_dev_arduino.i2c = &Wire;
    max44009_dev.i2c_addr = MAX44009_I2C_ADDRESS1;
    max44009_arduino_init(dev, arduino_dev);

    printf("Illuminance value: %3.2f [lx]", max44009_get_llux(&dev))

    return 0;
}
```

## Porting to different platforms
If you want to port this library on other platform, the only thing you have to do is to implement HW/Platform dependent functions from `max44009.h` file as it is done for example for [ARDUINO] platform in `platform/max44009-arduino.cpp` file.


## Author
* Lukasz Uszko aka `luk6xff` [lukasz.uszko@gmail.com]