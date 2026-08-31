# SX1278 LoRa radio transceiver

> **Deprecated!** Use the newer [LORA library](https://github.com/luk6xff/DevLibs/tree/master/LORA) instead.

## Description
Platform independent driver for the Semtech SX1278 LoRa/FSK radio transceiver,
based on the SEMTECH https://github.com/Lora-net/LoRaMac-node code.

The core (`sx1278.c` / `sx1278.h`) is written in plain C and knows nothing
about the underlying hardware. Everything hardware related is provided by a thin
platform layer living in `platform/<platform>`.

* Interface: SPI + NSS, RESET and DIO0..DIO5 pins
* Modems (`RadioModems_t`): `MODEM_FSK`, `MODEM_LORA`
* Spreading factors `LORA_SF6` .. `LORA_SF12`, bandwidths
  `LORA_BANDWIDTH_7_8kHz` .. `LORA_BANDWIDTH_500kHz`, coding rates
  `LORA_ERROR_CODING_RATE_4_5` .. `LORA_ERROR_CODING_RATE_4_8`
* Predefined channels such as `RF_FREQUENCY_434_0` (434 MHz)
* RX buffer size: `RX_BUFFER_SIZE` (256 bytes)
* Radio events (TX/RX done, TX/RX timeout, RX error, FHSS channel change, CAD
  done) are delivered through the `RadioEvents_t` callback structure
* Sync words: `LORA_MAC_PRIVATE_SYNCWORD`, `LORA_MAC_PUBLIC_SYNCWORD`

The library was tested with [MBED](https://www.mbed.com) on RA-01 and RA-02
boards from AI-Thinker. It should work for all SX127x chips from SEMTECH.

## Supported platforms
| Platform | Files |
|---|---|
| Mbed | `platform/mbed/sx1278-mbed.{h,cpp}` |

## Usage
Demo code showing how to use the library on the `MBED` platform:

```cpp
#include "SX1278/platform/mbed/sx1278-mbed.h"

// PINOUT FOR MBED_NUCLEO_L053R8 BOARD
#define SX1278_MOSI  PB_15
#define SX1278_MISO  PB_14
#define SX1278_SCLK  PB_13
#define SX1278_NSS   PB_12
#define SX1278_RST   PB_1
#define SX1278_DIO0  PA_9
#define SX1278_DIO1  PA_8
#define SX1278_DIO2  PB_10
#define SX1278_DIO3  PB_4

// Radio LORA settings
#define RF_FREQUENCY               RF_FREQUENCY_434_0
#define TX_OUTPUT_POWER            14 // [dBm]
#define LORA_BANDWIDTH             LORA_BANDWIDTH_125kHz
#define LORA_SPREADING_FACTOR      LORA_SF8
#define LORA_CODINGRATE            LORA_ERROR_CODING_RATE_4_5
#define LORA_PREAMBLE_LENGTH       8    // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT        5    // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_FHSS_ENABLED          false
#define LORA_NB_SYMB_HOP           4
#define LORA_IQ_INVERSION_ON       false
#define LORA_CRC_ENABLED           true
#define RX_TIMEOUT_VALUE           8000 // [ms]
#define MAX_PAYLOAD_LENGTH         64

static RadioEvents_t RadioEvents;

const char ClientMsg[] = "Hello FROM LORA CLIENT!";

void OnTxDone(void)
{
    SX1278SetSleep();
    printf("> OnTxDone\r\n");
}

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
    SX1278SetSleep();
    printf("> RssiValue: %d, SnrValue: %d, PAYLOAD: %s\r\n", rssi, snr, payload);
}

void OnTxTimeout(void)
{
    SX1278SetSleep();
    printf("> OnTxTimeout\r\n");
}

void OnRxTimeout(void)
{
    SX1278SetSleep();
    printf("> OnRxTimeout\r\n");
}

void OnRxError(void)
{
    SX1278SetSleep();
    printf("> OnRxError\r\n");
}

int main()
{
    RadioEvents.TxDone = OnTxDone;
    RadioEvents.RxDone = OnRxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    RadioEvents.RxTimeout = OnRxTimeout;
    RadioEvents.RxError = OnRxError;

    SX1278MbedInit(&RadioEvents, SX1278_MOSI, SX1278_MISO, SX1278_SCLK,
                   SX1278_NSS, SX1278_RST,
                   SX1278_DIO0, SX1278_DIO1, SX1278_DIO2, SX1278_DIO3, NC, NC);
    SX1278SetChannel(RF_FREQUENCY);

    // Verify that the SX1278 is connected to the board
    while (SX1278Read(REG_VERSION) == 0x00)
    {
        printf("Radio could not be detected!\r\n");
        SX1278DelayMs(1000);
    }

    SX1278SetMaxPayloadLength(MODEM_LORA, MAX_PAYLOAD_LENGTH);
    SX1278SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                      LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                      LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                      LORA_CRC_ENABLED, LORA_FHSS_ENABLED, LORA_NB_SYMB_HOP,
                      LORA_IQ_INVERSION_ON, 4000);

    SX1278SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                      LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                      LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON, 0,
                      LORA_CRC_ENABLED, LORA_FHSS_ENABLED, LORA_NB_SYMB_HOP,
                      LORA_IQ_INVERSION_ON, true);

    while (true)
    {
        SX1278Send((uint8_t*)ClientMsg, sizeof(ClientMsg));
        SX1278SetRx(RX_TIMEOUT_VALUE);
        SX1278DelayMs(RX_TIMEOUT_VALUE);
        printf("> Data sent to the gateway\r\n");
    }
}
```

A gateway node uses exactly the same setup - only the transmitted payload and
the moment when `SX1278Send()` is called differ.

## Porting to a different platform
To port this library to another platform you only have to implement the
HW dependent functions declared at the bottom of `sx1278.h`:

* `SX1278IoInit()`
* `SX1278IoDeInit()`
* `SX1278IoIrqInit()`
* `SX1278Reset()`
* `SX1278WriteBuffer()`
* `SX1278ReadBuffer()`
* `SX1278SetTimeout()`
* `SX1278DelayMs()`

and provide an `SX1278<Platform>Init()` helper. See
`platform/mbed/sx1278-mbed.cpp` for a reference implementation.

## License
MIT - see the repository root for details.

## Author
Lukasz Uszko - lukasz.uszko@gmail.com

* Lukasz Uszko aka `luk6xff` [lukasz.uszko@gmail.com]
