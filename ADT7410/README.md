# ADT7410 digital temperature sensor

## Description
Platform independent driver for the Analog Devices ADT7410 digital temperature
sensor.

Datasheet: http://www.analog.com/static/imported-files/data_sheets/ADT7410.pdf

The core (`adt7410.c` / `adt7410.h`) is written in plain C and knows nothing
about the underlying hardware. Everything hardware related is provided by a thin
platform layer living in `platform/<platform>`.

* Interface: I2C (up to 400 kHz)
* Addresses: `0x48` .. `0x4B`, selected with the A1/A0 pins
* Resolution: 13-bit (0.0625 degC/LSB) or 16-bit (0.0078 degC/LSB)
* Range: -55 degC .. +150 degC

## Supported platforms
| Platform | Files |
|---|---|
| Arduino | `platform/arduino/adt7410-arduino.{h,cpp}` |
| Mbed | `platform/mbed/adt7410-mbed.{h,cpp}` |
| STM32 Cube HAL | `platform/stm32cube/adt7410-cube-hal.{h,c}` |

## Usage
Demo code showing how to use the library on the `ARDUINO` platform:

```cpp
#include "ADT7410/platform/arduino/adt7410-arduino.h"

adt7410 dev;
adt7410_arduino arduino_dev;

void setup()
{
    Serial.begin(115200);

    arduino_dev.i2c = &Wire;
    arduino_dev.i2c_freq = 400000;
    dev.i2c_addr = ADT7410_I2C_ADDR_A1_GND_A0_GND;

    if (!adt7410_arduino_init(&dev, &arduino_dev))
    {
        Serial.println("ADT7410 init failed!");
        return;
    }

    uint8_t id = 0;
    uint8_t revision = 0;
    if (adt7410_read_id(&dev, &id, &revision))
    {
        Serial.print("ADT7410 manufacturer id: 0x");
        Serial.print(id, HEX);
        Serial.print(", revision: ");
        Serial.println(revision);
    }

    // Optional - apply a full custom configuration
    adt7410_config cfg;
    cfg.fault_queue      = ADT7410_FAULT_QUEUE_1;
    cfg.ct_pin_polarity  = ADT7410_PIN_ACTIVE_LOW;
    cfg.int_pin_polarity = ADT7410_PIN_ACTIVE_LOW;
    cfg.int_ct_mode      = ADT7410_INT_CT_MODE_INTERRUPT;
    cfg.operation_mode   = ADT7410_MODE_CONTINUOUS;
    cfg.resolution       = ADT7410_RESOLUTION_16_BIT;
    adt7410_set_config(&dev, &cfg);

    // ...or change only the resolution
    adt7410_set_resolution(&dev, ADT7410_RESOLUTION_13_BIT);
}

void loop()
{
    float temperature = 0.0f;
    if (adt7410_read_temperature(&dev, &temperature))
    {
        Serial.print("Temperature: ");
        Serial.print(temperature);
        Serial.println(" [degC]");
    }
    else
    {
        Serial.println("ADT7410 read failed!");
        adt7410_reset(&dev);
    }

    // The last successful reading is cached inside the device object
    Serial.print("Cached temperature: ");
    Serial.println(adt7410_get_temperature(&dev));

    delay(1000);
}
```

## Porting to a different platform
To port this library to another platform you only have to implement the
HW dependent functions declared at the bottom of `adt7410.h`:

* `adt7410_write()`
* `adt7410_read()`
* `adt7410_delay_ms()`

and provide a `adt7410_<platform>_init()` helper that stores your platform
descriptor in `dev->platform_dev`. See `platform/arduino/adt7410-arduino.cpp`
for a reference implementation.

## License
MIT - see the repository root for details.

## Author
Lukasz Uszko - lukasz.uszko@gmail.com

* Lukasz Uszko aka `luk6xff` [lukasz.uszko@gmail.com]
