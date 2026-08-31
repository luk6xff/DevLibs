# BTM222 Bluetooth SPP module

## Description
Platform independent driver for the BTM222 (Rayson) Bluetooth 2.0 SPP module.

The core (`btm222.c` / `btm222.h`) is written in plain C and knows nothing about
the underlying hardware. Everything hardware related is provided by a thin
platform layer living in `platform/<platform>`.

* Interface: UART, 19200 baud by default, 8N1
* Bluetooth 2.0 + EDR, Serial Port Profile, class 1 (up to ~100 m)
* Transparent data mode plus an AT command mode for configuration
* Built in lock free receive ring buffer (256 bytes, single producer / single
  consumer) that replaces the mbed only `BufferedSerial` helper the previous
  version of this library vendored
* Every AT exchange is bounded by an explicit timeout, no unbounded blocking
  reads, no dynamic allocation

References:
* Module documentation and the AT command set are published by Rayson:
  http://www.rayson.com (BTM-222 / BTM-112 command reference)

Note: the named AT helpers follow the Rayson BTM-222 command set
(`ATI0`..`ATI4`, `AT+BTLNM`, `AT+BTPIN`, `AT+BTURT`, `AT+BTSCAN`, `AT+BTCANCEL`,
`ATH`, `ATZ0`). Should your firmware revision use a different spelling, use
`btm222_send_at_command()` directly, it accepts any command string.

## Supported platforms
| Platform | Files |
|---|---|
| Arduino | `platform/arduino/btm222-arduino.{h,cpp}` |
| Mbed | `platform/mbed/btm222-mbed.{h,cpp}` |
| STM32 Cube HAL | `platform/stm32cube/btm222-cube-hal.{h,c}` |

## Usage
Demo code showing how to use the library on the `ARDUINO` platform:

```cpp
#include "BTM222/platform/arduino/btm222-arduino.h"

btm222 dev;
btm222_arduino arduino_dev;

void setup()
{
    Serial.begin(115200);

    // Stream has no begin(), the port has to be opened by the sketch
    Serial1.begin(BTM222_DEFAULT_BAUDRATE);

    arduino_dev.serial = &Serial1;
    arduino_dev.baudrate = BTM222_DEFAULT_BAUDRATE;

    if (!btm222_arduino_init(&dev, &arduino_dev))
    {
        Serial.println("BTM222 init failed!");
        return;
    }

    char resp[32];
    if (btm222_get_info(&dev, BTM222_INFO_FIRMWARE, resp, sizeof(resp)))
    {
        Serial.print("BTM222 firmware: ");
        Serial.println(resp);
    }

    btm222_set_name(&dev, "MySensorNode");
    btm222_set_pin(&dev, "1234");
    btm222_set_discoverable(&dev, true);
}

void loop()
{
    static uint32_t last_tx = 0;

    // Move the received bytes into the ring buffer
    btm222_poll(&dev);

    // Echo everything the peer sent back to the USB console
    while (btm222_rx_available(&dev) > 0)
    {
        const int c = btm222_read(&dev);
        if (c >= 0)
        {
            Serial.write((uint8_t)c);
        }
    }

    if ((millis() - last_tx) >= 1000)
    {
        last_tx = millis();
        btm222_send_tagged_value(&dev, 'T', 23.75f); // sends "T23.75x"
    }
}
```

## Porting to a different platform
To port this library to another platform you only have to implement the
HW dependent functions declared at the bottom of `btm222.h`:

* `btm222_uart_write()`
* `btm222_uart_read_byte()`
* `btm222_uart_readable()`
* `btm222_delay_ms()`
* `btm222_get_ms_ticks()`

and provide a `btm222_<platform>_init()` helper that stores your platform
descriptor in `dev->platform_dev`. See `platform/arduino/btm222-arduino.cpp`
for a reference implementation.

Platforms with an UART receive interrupt should call `btm222_rx_push()` from the
handler instead of relying on `btm222_poll()`; the Mbed layer shows this with
`btm222_mbed_attach_rx_irq()`.

## License
MIT - see the repository root for details.

## Author
Lukasz Uszko - lukasz.uszko@gmail.com

* Lukasz Uszko aka `luk6xff` [lukasz.uszko@gmail.com]
