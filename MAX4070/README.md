# MAX4070 bidirectional high-side current-sense amplifier

## Description
Platform independent driver for the Maxim MAX4070 bidirectional, high-side
current-sense amplifier with reference.

The core (`max4070.c` / `max4070.h`) is written in plain C and knows nothing
about the underlying hardware. Everything hardware related is provided by a thin
platform layer living in `platform/<platform>`, which for this part is simply a
wrapper around one ADC channel.

* Interface: analog, the amplifier OUT pin is sampled by the host ADC
* Transfer function: `OUT = ref_voltage + gain * shunt_resistance * I`
* Bidirectional: the REF pin sets the zero-current output level, so a current
  flowing in the opposite direction pulls OUT below it and the reported current
  becomes negative
* Gain and shunt resistance are configuration fields of the device object, they
  are not hard-coded in the driver
* Every reading is the average of `adc_samples_num` ADC samples
* Datasheet: http://datasheets.maximintegrated.com/en/ds/MAX4069-MAX4072.pdf

For unidirectional sensing just leave `ref_voltage` at `0.0f`.

## Supported platforms
| Platform | Files |
|---|---|
| Arduino | `platform/arduino/max4070-arduino.{h,cpp}` |
| Mbed | `platform/mbed/max4070-mbed.{h,cpp}` |
| STM32 Cube HAL | `platform/stm32cube/max4070-cube-hal.{h,c}` |

## Usage
Demo code showing how to use the library on the `ARDUINO` platform:

```cpp
#include "MAX4070/platform/arduino/max4070-arduino.h"

// PINOUT FOR ESP32 WROOM DEVBOARD
#define MAX4070_ANALOG_PIN 34

max4070 dev;
max4070_arduino arduino_dev;

void setup()
{
    Serial.begin(115200);

    arduino_dev.adc_pin = MAX4070_ANALOG_PIN;

    dev.adc_vref = 3.3f;                 // ADC full scale voltage
    dev.adc_max_value = 4095;            // 12-bit ADC
    dev.adc_samples_num = 8;             // Average 8 samples per reading
    dev.ref_voltage = 1.65f;             // REF pin held at mid supply
    dev.gain = MAX4070_DEFAULT_GAIN;     // 50 V/V
    dev.shunt_resistance = 0.05f;        // 50 mOhm sense resistor

    if (!max4070_arduino_init(&dev, &arduino_dev))
    {
        Serial.println("MAX4070 init failed!");
    }
}

void loop()
{
    float current = 0.0f;
    if (max4070_read_current(&dev, &current))
    {
        Serial.print("Current: ");
        Serial.print(current * 1000.0f);
        Serial.print(" [mA], OUT: ");
        Serial.print(max4070_get_output_voltage(&dev));
        Serial.println(" [V]");
    }

    delay(1000);
}
```

## Porting to a different platform
To port this library to another platform you only have to implement the
HW dependent function declared at the bottom of `max4070.h`:

* `max4070_read_adc()`

and provide a `max4070_<platform>_init()` helper that stores your platform
descriptor in `dev->platform_dev`. See `platform/arduino/max4070-arduino.cpp`
for a reference implementation.

## License
MIT - see the repository root for details.

## Author
Lukasz Uszko - lukasz.uszko@gmail.com

* Lukasz Uszko aka `luk6xff` [lukasz.uszko@gmail.com]
