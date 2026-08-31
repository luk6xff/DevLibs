# AS3935 Franklin lightning sensor

## Description
Platform independent driver for the ams AS3935 Franklin lightning sensor.

The core (`as3935.c` / `as3935.h`) is written in plain C and knows nothing
about the underlying hardware. Everything hardware related is provided by a thin
platform layer living in `platform/<platform>`.

* Interface: I2C (up to 400 kHz) plus an IRQ output line
* Addresses: `0x01`, `0x02`, `0x03`, selected with the ADD1/ADD0 pins, most
  breakout boards tie both pins high and answer at `0x03`
* Detects lightning up to 40 km away and estimates the distance to the head of
  the storm, tells lightning apart from man-made disturbers
* Indoor and outdoor analog front-end gain, configurable noise floor, watchdog
  threshold, spike rejection and minimum number of strikes
* The interrupt source register is only valid 2 ms after the IRQ pin goes high,
  `as3935_read_interrupt_source()` takes care of that delay

Documentation regarding the AS3935 might be found here:
http://www.ams.com/eng/Products/Lightning-Sensor

## Supported platforms
| Platform | Files |
|---|---|
| Arduino | `platform/arduino/as3935-arduino.{h,cpp}` |
| Mbed | `platform/mbed/as3935-mbed.{h,cpp}` |
| STM32 Cube HAL | `platform/stm32cube/as3935-cube-hal.{h,c}` |

## Usage
Demo code showing how to use the library on the `ARDUINO` platform:

```cpp
#include "AS3935/platform/arduino/as3935-arduino.h"

as3935 dev;
as3935_arduino arduino_dev;

void setup()
{
    Serial.begin(115200);

    arduino_dev.i2c = &Wire;
    arduino_dev.i2c_freq = 400000;
    arduino_dev.irq_pin = 2;
    dev.i2c_addr = AS3935_I2C_ADDR_ADD1_HIGH_ADD0_HIGH;

    if (!as3935_arduino_init(&dev, &arduino_dev))
    {
        Serial.println("AS3935 init failed!");
        return;
    }

    // The sensor sits inside a case, mask the man-made disturbers away
    as3935_set_afe_gain(&dev, AS3935_AFE_GAIN_INDOOR);
    as3935_set_mask_disturbers(&dev, true);
}

void loop()
{
    if (!as3935_get_irq_pin(&dev))
    {
        delay(10);
        return;
    }

    as3935_int_source src = AS3935_INT_NONE;
    if (!as3935_read_interrupt_source(&dev, &src))
    {
        return;
    }

    switch (src)
    {
        case AS3935_INT_NOISE_TOO_HIGH:
            Serial.println("Noise level too high");
            break;

        case AS3935_INT_DISTURBER:
            Serial.println("Disturber detected");
            break;

        case AS3935_INT_LIGHTNING:
        {
            uint8_t distance_km = 0;
            uint32_t energy = 0;
            as3935_get_storm_distance(&dev, &distance_km);
            as3935_get_energy(&dev, &energy);

            Serial.print("Lightning detected, distance: ");
            Serial.print(distance_km);
            Serial.print(" [km], energy: ");
            Serial.println(energy);
            break;
        }

        default:
            break;
    }
}
```

## Porting to a different platform
To port this library to another platform you only have to implement the
HW dependent functions declared at the bottom of `as3935.h`:

* `as3935_write()`
* `as3935_read()`
* `as3935_get_irq_pin()`
* `as3935_delay_ms()`

and provide an `as3935_<platform>_init()` helper that stores your platform
descriptor in `dev->platform_dev`. See `platform/arduino/as3935-arduino.cpp`
for a reference implementation.

## License
MIT - see the repository root for details.

## Author
Lukasz Uszko - lukasz.uszko@gmail.com

* Lukasz Uszko aka `luk6xff` [lukasz.uszko@gmail.com]
