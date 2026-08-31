# GTS-4E-60 GPS module

## Description
Platform independent driver for the Fibocom GTS-4E-60 GPS module, including a
NMEA 0183 sentence assembler and parser.

The core (`gts4e60.c` / `gts4e60.h`) is written in plain C and knows nothing
about the underlying hardware. Everything hardware related is provided by a thin
platform layer living in `platform/<platform>`.

* Interface: UART, 9600 baud by default, 8N1
* Sentences decoded: `GGA`, `RMC`, `GSA`, `GSV` (any talker id: `GP`, `GN`, `GL`, ...)
* Reported data: UTC time and date, latitude, longitude, fix quality, fix mode,
  satellites used and in view, altitude, geoid separation, HDOP / PDOP / VDOP,
  ground speed in knots and km/h, course over ground and magnetic variation
* Every sentence is validated against its NMEA XOR checksum before it is decoded
* Non blocking: `gts4e60_process()` consumes only the bytes that are already
  available, no dynamic allocation, all buffers live inside the device object
* Optional shutdown line support

References:
* NMEA protocol reference manual: https://www.sparkfun.com/datasheets/GPS/NMEA%20Reference%20Manual1.pdf
* Degree formats and how to convert between them: http://home.online.no/~sigurdhu/Deg_formats.htm
* GTS-4E-60 module documentation: http://www.fibocom.com/product/2-1-3-2.html

## Supported platforms
| Platform | Files |
|---|---|
| Arduino | `platform/arduino/gts4e60-arduino.{h,cpp}` |
| Mbed | `platform/mbed/gts4e60-mbed.{h,cpp}` |
| STM32 Cube HAL | `platform/stm32cube/gts4e60-cube-hal.{h,c}` |

## Usage
Demo code showing how to use the library on the `ARDUINO` platform:

```cpp
#include "GPS_GTS-4E-60/platform/arduino/gts4e60-arduino.h"

gts4e60 dev;
gts4e60_arduino arduino_dev;

void setup()
{
    Serial.begin(115200);

    // Stream has no begin(), the port has to be opened by the sketch
    Serial1.begin(GTS4E60_DEFAULT_BAUDRATE);

    arduino_dev.serial = &Serial1;
    arduino_dev.baudrate = GTS4E60_DEFAULT_BAUDRATE;
    arduino_dev.shutdown_pin = GTS4E60_ARDUINO_NO_PIN;
    arduino_dev.shutdown_active_high = true;

    if (!gts4e60_arduino_init(&dev, &arduino_dev))
    {
        Serial.println("GTS4E60 init failed!");
    }
}

void loop()
{
    // Consumes whatever arrived since the last call, returns true on a new sentence
    if (gts4e60_process(&dev))
    {
        const gts4e60_data* data = gts4e60_get_data(&dev);

        if (gts4e60_has_fix(&dev))
        {
            Serial.print("UTC: ");
            Serial.print(data->time.hours);
            Serial.print(':');
            Serial.print(data->time.minutes);
            Serial.print(':');
            Serial.println(data->time.seconds);

            Serial.print("Position: ");
            Serial.print(data->latitude, 6);
            Serial.print(", ");
            Serial.println(data->longitude, 6);

            Serial.print("Altitude: ");
            Serial.print(data->altitude);
            Serial.println(" [m]");

            Serial.print("Speed: ");
            Serial.print(data->speed_kmh);
            Serial.println(" [km/h]");

            Serial.print("Satellites used/in view: ");
            Serial.print(data->satellites_used);
            Serial.print('/');
            Serial.println(data->satellites_in_view);
        }
        else
        {
            Serial.println("No GPS fix yet");
        }
    }
}
```

## Porting to a different platform
To port this library to another platform you only have to implement the
HW dependent functions declared at the bottom of `gts4e60.h`:

* `gts4e60_uart_write()`
* `gts4e60_uart_read_byte()`
* `gts4e60_uart_readable()`
* `gts4e60_set_shutdown_pin()`
* `gts4e60_delay_ms()`
* `gts4e60_get_ms_ticks()`

and provide a `gts4e60_<platform>_init()` helper that stores your platform
descriptor in `dev->platform_dev`. See `platform/arduino/gts4e60-arduino.cpp`
for a reference implementation.

## License
MIT - see the repository root for details.

## Author
Lukasz Uszko - lukasz.uszko@gmail.com

* Lukasz Uszko aka `luk6xff` [lukasz.uszko@gmail.com]
