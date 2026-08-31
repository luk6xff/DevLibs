# HM-11 Bluetooth Low Energy 4.0 module

## Description
Platform independent driver for the HM-11 (Jinan Huamao / HMSoft) Bluetooth Low
Energy 4.0 module.

The core (`hm11.c` / `hm11.h`) is written in plain C and knows nothing about the
underlying hardware. Everything hardware related is provided by a thin platform
layer living in `platform/<platform>`.

* Interface: UART, 9600 baud 8N1 by default, configured with AT commands
* Bluetooth Low Energy 4.0, single mode, CC2540/CC2541 based
* Factory defaults: name `HMSoft`, PIN `000000`, peripheral role, transparent
  transmission mode
* AT commands carry no CR/LF terminator, an answer is complete once the UART has
  been idle for `HM11_RESP_IDLE_MS`
* Built in lock free receive ring buffer (256 bytes, single producer / single
  consumer) that replaces the mbed only `BufferedSerial` helper the previous
  version of this library vendored
* Every AT exchange is bounded by an explicit timeout, no unbounded blocking
  reads, no dynamic allocation

References:
* https://www.microduino.cc/wiki/images/f/fc/Bluetooth40_en.pdf
* http://txyz.info/b10n1c/datasheets/hm-11_bluetooth40_en.pdf
* `docs/hm11_datasheet.pdf`

Note: the AT command set grew over the module firmware revisions. The typed
helpers cover the commands common to the V5xx firmwares; anything else
(`AT+IBEA`, `AT+ALLO`, `AT+BATT`, `AT+MODE`, `AT+RSSI`, ...) is reachable
through `hm11_send_at_command()`, which accepts any command string.

## Supported platforms
| Platform | Files |
|---|---|
| Arduino | `platform/arduino/hm11-arduino.{h,cpp}` |
| Mbed | `platform/mbed/hm11-mbed.{h,cpp}` |
| STM32 Cube HAL | `platform/stm32cube/hm11-cube-hal.{h,c}` |

## Usage
Demo code showing how to use the library on the `ARDUINO` platform:

```cpp
#include "HM11_BLE/platform/arduino/hm11-arduino.h"

hm11 dev;
hm11_arduino arduino_dev;

void setup()
{
    Serial.begin(115200);

    // Stream has no begin(), the port has to be opened by the sketch
    Serial1.begin(HM11_DEFAULT_BAUDRATE);

    arduino_dev.serial = &Serial1;
    arduino_dev.baudrate = HM11_DEFAULT_BAUDRATE;

    if (!hm11_arduino_init(&dev, &arduino_dev))
    {
        Serial.println("HM11 init failed!");
        return;
    }

    if (!hm11_test(&dev, HM11_AT_TIMEOUT_MS))
    {
        Serial.println("HM11 does not answer!");
        return;
    }

    char version[32];
    if (hm11_get_version(&dev, version, sizeof(version)))
    {
        Serial.print("HM11 firmware: ");
        Serial.println(version);
    }

    char address[HM11_ADDR_LEN + 1];
    if (hm11_get_address(&dev, address, sizeof(address)))
    {
        Serial.print("HM11 address: ");
        Serial.println(address);
    }

    hm11_set_name(&dev, "MyNode");
    hm11_set_pin(&dev, 123456);
    hm11_set_role(&dev, HM11_ROLE_PERIPHERAL);
    hm11_set_power(&dev, HM11_POWER_0_DBM);
    hm11_set_adv_interval(&dev, HM11_ADV_INTERVAL_1285MS);
    hm11_set_notify(&dev, HM11_NOTIFY_ON);
    hm11_reset(&dev);
}

void loop()
{
    static uint32_t last_tx = 0;

    // Move the received bytes into the ring buffer
    hm11_poll(&dev);

    // Echo everything the peer sent back to the USB console
    while (hm11_rx_available(&dev) > 0)
    {
        const int c = hm11_read(&dev);
        if (c >= 0)
        {
            Serial.write((uint8_t)c);
        }
    }

    if ((millis() - last_tx) >= 1000)
    {
        last_tx = millis();
        hm11_write_string(&dev, "hello\r\n");
    }
}
```

Scanning for peers from a central module:

```cpp
hm11_set_role(&dev, HM11_ROLE_CENTRAL);
hm11_set_work_type(&dev, HM11_WORK_TYPE_IMMEDIATE);

hm11_scan_result found[HM11_MAX_SCAN_RESULTS];
uint8_t count = 0;
if (hm11_discover(&dev, found, HM11_MAX_SCAN_RESULTS, &count,
                  HM11_DISCOVERY_TIMEOUT_MS))
{
    hm11_conn_status status = HM11_CONN_UNKNOWN;
    for (uint8_t i = 0; i < count; ++i)
    {
        Serial.println(found[i].addr);
    }
    if (count > 0)
    {
        hm11_connect_to_index(&dev, 0, &status, HM11_CONNECT_TIMEOUT_MS);
    }
}
```

## Porting to a different platform
To port this library to another platform you only have to implement the
HW dependent functions declared at the bottom of `hm11.h`:

* `hm11_uart_write()`
* `hm11_uart_read_byte()`
* `hm11_uart_readable()`
* `hm11_delay_ms()`
* `hm11_get_ms_ticks()`

and provide a `hm11_<platform>_init()` helper that stores your platform
descriptor in `dev->platform_dev`. See `platform/arduino/hm11-arduino.cpp` for a
reference implementation.

Platforms with an UART receive interrupt should call `hm11_rx_push()` from the
handler instead of relying on `hm11_poll()`; the Mbed layer shows this with
`hm11_mbed_attach_rx_irq()`.

## License
MIT - see the repository root for details.

## Author
Lukasz Uszko - lukasz.uszko@gmail.com

* Lukasz Uszko aka `luk6xff` [lukasz.uszko@gmail.com]
