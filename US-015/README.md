# US-015 ultrasonic distance sensor

## Description
Platform independent driver for the US-015 ultrasonic distance measuring module.
The driver is also compatible with the pin and protocol equivalent HC-SR04.

The core (`us015.c` / `us015.h`) is written in plain C and knows nothing about
the underlying hardware. Everything hardware related is provided by a thin
platform layer living in `platform/<platform>`.

* Interface: two GPIOs, one trigger output and one echo input
* Measurement: a 10 us HIGH pulse on TRIG emits a burst of eight 40 kHz cycles,
  the width of the following ECHO pulse is proportional to the distance
* Conversion: `distance [cm] = pulse_width [us] / 58`,
  `distance [inch] = pulse_width [us] / 148`
* Range: 2 cm - 400 cm
* A missing echo is detected with a configurable timeout, so a measurement can
  never block forever
* Datasheet: http://www.micropik.com/PDF/HCSR04.pdf

## Supported platforms
| Platform | Files |
|---|---|
| Arduino | `platform/arduino/us015-arduino.{h,cpp}` |
| Mbed | `platform/mbed/us015-mbed.{h,cpp}` |
| STM32 Cube HAL | `platform/stm32cube/us015-cube-hal.{h,c}` |

## Usage
Demo code showing how to use the library on the `ARDUINO` platform:

```cpp
#include "US-015/platform/arduino/us015-arduino.h"

us015 dev;
us015_arduino arduino_dev;

void setup()
{
    Serial.begin(115200);

    arduino_dev.trigger_pin = 8;
    arduino_dev.echo_pin = 9;

    // Do not wait longer than the echo of an object 400 cm away
    dev.timeout_us = us015_distance_cm_to_timeout_us(US015_MAX_DISTANCE_CM);

    if (!us015_arduino_init(&dev, &arduino_dev))
    {
        Serial.println("US-015 init failed!");
    }
}

void loop()
{
    uint32_t pulse_width_us = 0;
    float distance_cm = 0.0f;

    if (us015_read_pulse_width(&dev, &pulse_width_us))
    {
        distance_cm = us015_pulse_width_to_cm(pulse_width_us);
        Serial.print("Echo: ");
        Serial.print(pulse_width_us);
        Serial.print(" [us], distance: ");
        Serial.print(distance_cm);
        Serial.println(" [cm]");
    }
    else
    {
        Serial.println("US-015 timeout - no echo received");
    }
    delay(1000);
}
```

## Porting to a different platform
To port this library to another platform you only have to implement the
HW dependent functions declared at the bottom of `us015.h`:

* `us015_set_trigger_pin()`
* `us015_get_echo_pin()`
* `us015_delay_us()`
* `us015_get_us_ticks()`

and provide a `us015_<platform>_init()` helper that stores your platform
descriptor in `dev->platform_dev`. See `platform/arduino/us015-arduino.cpp`
for a reference implementation.

## License
MIT - see the repository root for details.

## Author
Lukasz Uszko - lukasz.uszko@gmail.com

* Lukasz Uszko aka `luk6xff` [lukasz.uszko@gmail.com]
