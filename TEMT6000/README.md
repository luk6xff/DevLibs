# TEMT6000 RTC module library

## Description
TEMT6000 ambient light sensorplatform independent library

## Usage
Simple demo code how to use the library on [ARDUINO] platform
```cpp

#include "TEMT6000/platform/temt6000-arduino.h"

// PINOUT FOR ESP32 WROOM DEVBOARD
#define TEMT600_ANALOG_PIN 34

int main(void)
{
    temt6000_arduino arduino_dev =
    {
        TEMT600_ANALOG_PIN, // analog_gpio_pin
    };

    temt6000 dev =
    {
        3.3f, // vdd_value
        4096, // adc_max_value
        8,    // adc_samples_num
    };
    temt6000_arduino_init(dev, arduino_dev);

    printf("Illuminance value: %3.2f [lx]", temt6000_get_light(&dev))

    return 0;
}
```

## Porting to different platforms
If you want to port this library on other platform, the only thing you have to do is to implement HW/Platform dependent functions from `temt6000.h` file as it is done for example for [ARDUINO] platform in `platform/temt6000-arduino.cpp` file.


## Author
* Lukasz Uszko aka `luk6xff` [lukasz.uszko@gmail.com]