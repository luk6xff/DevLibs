# RF22 - HopeRF RFM22B / RFM23B ISM transceiver

## Description
Platform independent driver for the HopeRF RFM22B / RFM23B (RF22B) ISM band
transceiver modules.

The core (`rf22.c` / `rf22.h`) is written in plain C and knows nothing about the
underlying hardware. Everything hardware related is provided by a thin platform
layer living in `platform/<platform>`. Nothing is allocated dynamically - the
shared Rx/Tx message buffer lives inside the `rf22` device object.

* Interface: SPI (mode 0, MSB first) plus an `nIRQ` input and an optional `SDN` output
* Modulation: FSK, GFSK and OOK with 25 canned modem configurations (`rf22_modem_config_choice`)
* Frequency: 240.0 MHz - 960.0 MHz, settable with 312 Hz precision
* Payload: up to `RF22_MAX_MESSAGE_LEN` (255) octets per message
* Datasheet: https://www.sparkfun.com/datasheets/Wireless/General/RFM22B.pdf

The register map lives in `registers.h`, following the same convention as the
`RFM69` and `SX1278` libraries in this repository.

### Protocol layers
| Layer | Files | Provides |
|---|---|---|
| `rf22` | `rf22.{h,c}` | Unaddressed, unreliable messages |
| `rf22_datagram` | `rf22_datagram.{h,c}` | Addressed, unreliable messages |
| `rf22_reliable_datagram` | `rf22_reliable_datagram.{h,c}` | Addressed, acknowledged, retransmitted messages |
| `rf22_router` | `rf22_router.{h,c}` | Multi-hop delivery via a static routing table |

Each higher layer embeds the layer below it as its first struct member, so a
`rf22_router*` can be used wherever the lower level API is needed.

## Supported platforms
| Platform | Files |
|---|---|
| Arduino | `platform/arduino/rf22-arduino.{h,cpp}` |
| Mbed | `platform/mbed/rf22-mbed.{h,cpp}` |
| STM32 Cube HAL | `platform/stm32cube/rf22-cube-hal.{h,c}` |

## Usage
Demo code showing how to use the library on the `ARDUINO` platform:

```cpp
#include "RF22/platform/arduino/rf22-arduino.h"

rf22 dev;
rf22_arduino arduino_dev;

void setup()
{
    Serial.begin(115200);

    arduino_dev.spi = &SPI;
    arduino_dev.spi_freq = 1000000;
    arduino_dev.cs_pin = 10;
    arduino_dev.irq_pin = 2;
    arduino_dev.sdn_pin = RF22_ARDUINO_PIN_UNUSED; // SDN tied low in hardware

    if (!rf22_arduino_init(&dev, &arduino_dev))
    {
        Serial.println("RF22 init failed!");
        return;
    }

    rf22_set_frequency(&dev, 434.0f, 0.05f);
    rf22_set_modem_config(&dev, RF22_GFSK_RB2_4FD36);
    rf22_set_tx_power(&dev, RF22_TXPOW_17DBM);
}

void loop()
{
    const char msg[] = "Hello from RF22";
    rf22_send(&dev, (const uint8_t*)msg, sizeof(msg) - 1);
    rf22_wait_packet_sent(&dev, 1000);

    uint8_t buf[RF22_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    if (rf22_wait_available(&dev, 1000) && rf22_recv(&dev, buf, &len))
    {
        Serial.print("Got ");
        Serial.print(len);
        Serial.print(" bytes, RSSI ");
        Serial.println(rf22_last_rssi(&dev));
    }

    delay(1000);
}
```

The driver does not require interrupts. Either call `rf22_handle_interrupt(&dev)`
from your `nIRQ` ISR, or let `rf22_available()` poll the pin for you as the
example above does.

## Porting to a different platform
To port this library to another platform you only have to implement the
HW dependent functions declared at the bottom of `rf22.h`:

* `rf22_spi_transfer()`
* `rf22_set_cs_pin()`
* `rf22_set_sdn_pin()`
* `rf22_get_irq_pin()`
* `rf22_delay_ms()`
* `rf22_get_ms_ticks()`

and provide a `rf22_<platform>_init()` helper that stores your platform
descriptor in `dev->platform_dev`. See `platform/arduino/rf22-arduino.cpp` for a
reference implementation.

## Not implemented
* **`RF22Mesh`** - the automatic route discovery / rediscovery layer from the
  original library was not ported. Its C++ implementation relies on virtual
  dispatch through `RF22Router` and on broadcast route-discovery state that could
  not be ported faithfully without guessing at the original intent. Use
  `rf22_router` with a statically configured routing table instead.

## Credits
Original RF22 library by **Mike McCauley** (mikem@open.com.au), Copyright (C) 2011.
Ported to mbed by **Karl Zweimueller**. Modified by **Lukasz Uszko**.
Restructured for the DevLibs portable-core/platform layout by luk6xff.

## License
**Not MIT.** This library is Copyright (C) 2011 Mike McCauley and is available
under GPL V2 or a commercial licence from the original author.
See [LICENSE.txt](LICENSE.txt) for the full statement.

## Maintainer
Lukasz Uszko - lukasz.uszko@gmail.com
