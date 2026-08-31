# BH1750 ambient light sensor

## Description
Platform independent driver for the ROHM BH1750 digital ambient light sensor.

The core (`bh1750.c` / `bh1750.h`) is written in plain C and knows nothing
about the underlying hardware. Everything hardware related is provided by a thin
platform layer living in `platform/<platform>`.

* Interface: I2C
* Addresses: `BH1750_I2C_ADDRESS1` = `0x23` (ADDR -> GND),
  `BH1750_I2C_ADDRESS2` = `0x5C` (ADDR -> VDD)
* Resolution modes (`bh1750_mode`): 0.5 lx, 1 lx and 4 lx, each in a
  continuous or a one-time flavour
* Power states (`bh1750_state`): power down, power on, reset

## Supported platforms
| Platform | Files |
|---|---|
| Arduino | `platform/arduino/bh1750-arduino.{h,cpp}` |

## Usage
Demo code showing how to use the library on the `ARDUINO` platform:

```cpp
#include "BH1750/platform/arduino/bh1750-arduino.h"

bh1750 dev;
bh1750_arduino arduino_dev;

void setup()
{
    Serial.begin(115200);

    arduino_dev.i2c = &Wire;
    dev.i2c_addr = BH1750_I2C_ADDRESS1;

    if (!bh1750_arduino_init(&dev, &arduino_dev))
    {
        Serial.println("BH1750 init failed!");
    }

    bh1750_set_state(&dev, BH1750_STATE_POWER_ON);
    bh1750_set_mode(&dev, BH1750_MODE_CONT_HIGH_RES);
}

void loop()
{
    float lux = 0.0f;
    if (bh1750_get_lux(&dev, &lux))
    {
        Serial.print("Illuminance value: ");
        Serial.print(lux);
        Serial.println(" [lx]");
    }
    delay(1000);
}
```

## Porting to a different platform
To port this library to another platform you only have to implement the
HW dependent functions declared at the bottom of `bh1750.h`:

* `bh1750_write_instruction()`
* `bh1750_read()`

and provide a `bh1750_<platform>_init()` helper that stores your platform
descriptor in `dev->platform_dev`. See `platform/arduino/bh1750-arduino.cpp`
for a reference implementation.

## License
MIT - see the repository root for details.

## Author
Lukasz Uszko - lukasz.uszko@gmail.com

* Lukasz Uszko aka `luk6xff` [lukasz.uszko@gmail.com]
