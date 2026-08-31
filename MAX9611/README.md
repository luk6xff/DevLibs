# MAX9611 high-side current-sense amplifier

## Description
Platform independent driver for the Maxim MAX9611 high-side current-sense
amplifier with a 12-bit ADC and an integrated op amp / comparator.
The pin and register compatible MAX9612 is supported as well.

The core (`max9611.c` / `max9611.h`) is written in plain C and knows nothing
about the underlying hardware. Everything hardware related is provided by a thin
platform layer living in `platform/<platform>`.

* Interface: I2C (up to 400 kHz)
* Addresses: `0x70` .. `0x7F` (7-bit), strapped with the A0 and A1 pins which
  can each be tied to GND, VCC, SDA or SCL. `0x70` corresponds to the
  `0xE0` write / `0xE1` read address pair quoted in the datasheet.
* Measured channels: current-sense amplifier output (gain 1x / 4x / 8x),
  RS+ common mode voltage, OUT voltage, SET voltage and die temperature
* Resolution: 12 bit for all channels except the temperature one, which is
  9 bit two's complement with 0.48 degC per LSB
* The sense resistor value (`shunt_res_ohm`), the amplifier gain (`gain`) and
  the zero-current code (`csa_offset`) are fields of the device object, so no
  conversion constant is hard-coded
* Datasheet: http://datasheets.maximintegrated.com/en/ds/MAX9611-MAX9612.pdf

Before the first use set `csa_offset` to the value returned by
`max9611_read_raw(&dev, MAX9611_MUX_CSA_GAIN_4X, &raw)` while no current is
flowing through the sense resistor.

## Supported platforms
| Platform | Files |
|---|---|
| Arduino | `platform/arduino/max9611-arduino.{h,cpp}` |
| Mbed | `platform/mbed/max9611-mbed.{h,cpp}` |
| STM32 Cube HAL | `platform/stm32cube/max9611-cube-hal.{h,c}` |

## Usage
Demo code showing how to use the library on the `ARDUINO` platform
(I2C pins on an Arduino Uno are A4 for SDA and A5 for SCL):

```cpp
#include "MAX9611/platform/arduino/max9611-arduino.h"

max9611 dev;
max9611_arduino arduino_dev;

void setup()
{
    Serial.begin(115200);

    arduino_dev.i2c = &Wire;
    arduino_dev.i2c_freq = 400000;

    dev.i2c_addr = MAX9611_I2C_ADDR_A1_GND_A0_GND;
    dev.shunt_res_ohm = 0.1f;      // 100 mOhm sense resistor
    dev.gain = MAX9611_GAIN_4X;
    dev.csa_offset = 0x0A;         // Raw CSA code measured at zero current

    if (!max9611_arduino_init(&dev, &arduino_dev))
    {
        Serial.println("MAX9611 init failed!");
    }
}

void loop()
{
    float current_ma = 0.0f;
    float voltage_v = 0.0f;
    float temp_c = 0.0f;

    if (max9611_read_current(&dev, &current_ma))
    {
        Serial.print("Current: ");
        Serial.print(current_ma);
        Serial.println(" [mA]");
    }
    else
    {
        Serial.println("MAX9611 CSA reading error - check all connections");
    }

    if (max9611_read_input_voltage(&dev, &voltage_v))
    {
        Serial.print("Input voltage: ");
        Serial.print(voltage_v);
        Serial.println(" [V]");
    }

    if (max9611_read_temperature(&dev, &temp_c))
    {
        Serial.print("Die temperature: ");
        Serial.print(temp_c);
        Serial.println(" [C]");
    }
    else
    {
        Serial.println("MAX9611 temperature reading error - check all connections");
    }

    delay(1000);
}
```

## Porting to a different platform
To port this library to another platform you only have to implement the
HW dependent functions declared at the bottom of `max9611.h`:

* `max9611_write()`
* `max9611_read()`
* `max9611_delay_ms()`

and provide a `max9611_<platform>_init()` helper that stores your platform
descriptor in `dev->platform_dev`. See `platform/arduino/max9611-arduino.cpp`
for a reference implementation.

## License
MIT - see the repository root for details.

## Author
Lukasz Uszko - lukasz.uszko@gmail.com

* Lukasz Uszko aka `luk6xff` [lukasz.uszko@gmail.com]
