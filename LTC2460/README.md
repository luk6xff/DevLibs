# LTC2460 16-bit delta-sigma ADC

## Description
Platform independent driver for the Linear Technology (Analog Devices) LTC2460
ultra-tiny 16-bit delta-sigma ADC with a 10ppm/degC max precision reference.

The core (`ltc2460.c` / `ltc2460.h`) is written in plain C and knows nothing
about the underlying hardware. Everything hardware related is provided by a thin
platform layer living in `platform/<platform>`.

* Interface: SPI, 8-bit frames, MSB first, mode 0, up to 2 MHz
* Resolution: 16 bit, single ended, unipolar
* Reference: 1.25 V internal (configurable through the `vref` field)
* Output data rate: 30 SPS (1x, with auto-calibration) or 60 SPS (2x)
* Every access shifts a 16-bit configuration word in while the previous
  conversion result is shifted out
* Datasheet: http://www.linear.com/product/LTC2460

An optional external resistor divider `GND -- R1 -- ADC -- R2 -- Vmeasured` is
supported through the `divider_ratio` field (`R1/R2`), which lets
`ltc2460_read_voltage()` return the voltage in front of the divider.

## Supported platforms
| Platform | Files |
|---|---|
| Arduino | `platform/arduino/ltc2460-arduino.{h,cpp}` |
| Mbed | `platform/mbed/ltc2460-mbed.{h,cpp}` |
| STM32 Cube HAL | `platform/stm32cube/ltc2460-cube-hal.{h,c}` |

## Usage
Demo code showing how to use the library on the `ARDUINO` platform:

```cpp
#include "LTC2460/platform/arduino/ltc2460-arduino.h"

ltc2460 dev;
ltc2460_arduino arduino_dev;

void setup()
{
    Serial.begin(115200);

    arduino_dev.spi = &SPI;
    arduino_dev.cs_pin = 10;
    arduino_dev.spi_freq = 1000000;

    dev.vref = LTC2460_DEFAULT_VREF;
    dev.divider_ratio = 0.0f; // No external divider

    if (!ltc2460_arduino_init(&dev, &arduino_dev))
    {
        Serial.println("LTC2460 init failed!");
    }
}

void loop()
{
    uint16_t raw = 0;
    float voltage = 0.0f;

    if (ltc2460_read_raw(&dev, &raw) && ltc2460_read_voltage(&dev, &voltage))
    {
        Serial.print("Raw code: ");
        Serial.print(raw);
        Serial.print(", voltage: ");
        Serial.print(voltage, 5);
        Serial.println(" [V]");
    }
    else
    {
        Serial.println("LTC2460 read failed!");
    }
    delay(1000);
}
```

## Porting to a different platform
To port this library to another platform you only have to implement the
HW dependent functions declared at the bottom of `ltc2460.h`:

* `ltc2460_spi_transfer()`
* `ltc2460_set_cs_pin()`
* `ltc2460_delay_ms()`

and provide a `ltc2460_<platform>_init()` helper that stores your platform
descriptor in `dev->platform_dev`. See `platform/arduino/ltc2460-arduino.cpp`
for a reference implementation.

## License
MIT - see the repository root for details.

## Author
Lukasz Uszko - lukasz.uszko@gmail.com

* Lukasz Uszko aka `luk6xff` [lukasz.uszko@gmail.com]
