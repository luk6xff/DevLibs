# TS3USB221 high-speed USB 2.0 1:2 multiplexer/demultiplexer

## Description
Platform independent driver for the Texas Instruments TS3USB221 high-speed
USB 2.0 (480 Mbps) 1:2 multiplexer / demultiplexer.

The core (`ts3usb221.c` / `ts3usb221.h`) is written in plain C and knows nothing
about the underlying hardware. Everything hardware related is provided by a thin
platform layer living in `platform/<platform>`.

* Interface: two GPIOs only - `OE` (output enable, active low) and `S` (channel select)
* `S` low  -> the `1D` port is routed to the common `D` port
* `S` high -> the `2D` port is routed to the common `D` port
* `OE` high -> all channels are disabled, device enters low power mode

## Supported platforms
| Platform | Files |
|---|---|
| Arduino | `platform/arduino/ts3usb221-arduino.{h,cpp}` |

## Usage
Demo code showing how to use the library on the `ARDUINO` platform:

```cpp
#include "TS3USB221/platform/arduino/ts3usb221-arduino.h"

ts3usb221 dev;
ts3usb221_arduino arduino_dev;

void setup()
{
    arduino_dev.oe_pin = 5;
    arduino_dev.s_pin = 6;

    if (!ts3usb221_arduino_init(&dev, &arduino_dev))
    {
        // Initialization failed
    }

    // Route the 1D port to the common D port
    ts3usb221_enable_1D_channel(&dev);
}

void loop()
{
    delay(5000);
    // Switch over to the 2D port
    ts3usb221_enable_2D_channel(&dev);

    delay(5000);
    // Disable both channels and save power
    ts3usb221_low_power_mode(&dev);
}
```

## Porting to a different platform
To port this library to another platform you only have to implement the
HW dependent function declared at the bottom of `ts3usb221.h`:

* `ts3usb221_set_pin()`

and provide a `ts3usb221_<platform>_init()` helper that stores your platform
descriptor in `dev->platform_dev`. See
`platform/arduino/ts3usb221-arduino.cpp` for a reference implementation.

## License
MIT - see the repository root for details.

## Author
Lukasz Uszko - lukasz.uszko@gmail.com
