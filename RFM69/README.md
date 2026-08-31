# RFM69 sub-GHz radio transceiver

## Description
Platform independent driver for the HopeRF RFM69W, RFM69HW, RFM69CW and
RFM69HCW modules (Semtech SX1231 / SX1231H), based on the great library from
[LowPowerLab](https://github.com/LowPowerLab/RFM69).

The core (`rfm69.c` / `rfm69.h`) is written in plain C and knows nothing
about the underlying hardware. Everything hardware related is provided by a thin
platform layer living in `platform/<platform>`.

* Interface: SPI + NSS, RESET and DIO0..DIO5 pins (only DIO0 is mandatory)
* Frequency bands: `RFM69_315MHZ`, `RFM69_433MHZ`, `RFM69_868MHZ`, `RFM69_915MHZ`
* Max payload: `RFM69_MAX_DATA_LEN` (FIFO size minus the 4 byte header)
* Addressing: 10-bit node id, 8-bit network id, `RFM69_BROADCAST_ADDR` for
  broadcasts, optional promiscuous mode
* Hardware AES-128 encryption (`RFM69_AES_KEY_LEN` = 16 bytes key)
* ACK / retry handling and automatic transmit power control (ATC) targeting a
  configurable RSSI
* Power range: `RFM69W_MIN_POWER_LEVEL_DBM` .. `RFM69HW_MAX_POWER_LEVEL_DBM`
* On-chip temperature sensor via `rfm69_read_temperature()`

The library was tested with [MBED](https://www.mbed.com) on HopeRF RFM69 boards.

## Supported platforms
| Platform | Files |
|---|---|
| Arduino | `platform/arduino/rfm69-arduino.{h,cpp}` |
| Mbed | `platform/mbed/rfm69-mbed.{h,cpp}` |
| STM32 Cube HAL | `platform/stm32cube/rfm69-cube-hal.{h,c}` |
| STM32 Cube LL | `platform/stm32cube_ll/rfm69-cube-ll.{h,c}` |

## Usage
Demo code showing how to use the library on the `ARDUINO` platform.

### NODE
```cpp
#include "RFM69/platform/arduino/rfm69-arduino.h"

// PINOUT FOR ESP32 WROOM DEVBOARD
#define RFM69_NSS_PIN   5
#define RFM69_RST_PIN  14
#define RFM69_DIO0_PIN 26

#define GATEWAY_ID     RFM69_DEFAULT_GATEWAY_ID

typedef struct
{
    uint8_t  node_id;
    uint32_t uptime; // Uptime in [ms]
    float    temp;
} frame_t;

rfm69 dev;
rfm69_arduino arduino_dev;

void setup()
{
    Serial.begin(115200);

    arduino_dev.spi = &SPI;
    arduino_dev.nss = RFM69_NSS_PIN;
    arduino_dev.reset = RFM69_RST_PIN;
    arduino_dev.dio0 = RFM69_DIO0_PIN;

    dev.settings.node_id = RFM69_DEFAULT_NODE_ID;
    dev.settings.network_id = RFM69_DEFAULT_NETWORKID;
    dev.settings.freq_band = RFM69_433MHZ;
    dev.settings.power_level_dbm = RFM69_DEFAULT_POWER_LEVEL_DBM;
    dev.settings.promiscuous_mode = RFM69_DEFAULT_PROMISCUOUS_MODE;
    dev.settings.is_rfm69hw = RFM69_DEFAULT_IS_RFM69HW;
    dev.settings.aes_encrypt_enable = true;
    memcpy(dev.settings.aes_encrypt_key, RFM69_DEFAULT_ENCRYPTKEY, RFM69_AES_KEY_LEN);
    dev.on_receive = NULL;

    if (!rfm69_arduino_init(&dev, &arduino_dev))
    {
        Serial.println("RFM69 init failed!");
    }

    // For RFM69HW/HCW only
    rfm69_set_high_power(&dev, true);
}

void loop()
{
    frame_t frame;
    frame.node_id = dev.settings.node_id;
    frame.uptime = rfm69_timer_read_ms();
    frame.temp = rfm69_read_temperature(&dev, 0);

    if (rfm69_send_with_retry(&dev, GATEWAY_ID, &frame, sizeof(frame),
                              RFM69_DEFAULT_SEND_RETRIES_NUM,
                              RFM69_DEFAULT_ACKTIMEOUT))
    {
        Serial.println("Frame sent!");
    }
    else
    {
        Serial.println("Frame has not been sent!");
    }

    rfm69_delay_ms(RFM69_DEFAULT_TRANSMIT_PERIOD);
}
```

### GATEWAY
```cpp
#include "RFM69/platform/arduino/rfm69-arduino.h"

// PINOUT FOR ESP32 WROOM DEVBOARD
#define RFM69_NSS_PIN   5
#define RFM69_RST_PIN  14
#define RFM69_DIO0_PIN 26

typedef struct
{
    uint8_t  node_id;
    uint32_t uptime; // Uptime in [ms]
    float    temp;
} frame_t;

rfm69 dev;
rfm69_arduino arduino_dev;

void setup()
{
    Serial.begin(115200);

    arduino_dev.spi = &SPI;
    arduino_dev.nss = RFM69_NSS_PIN;
    arduino_dev.reset = RFM69_RST_PIN;
    arduino_dev.dio0 = RFM69_DIO0_PIN;

    dev.settings.node_id = RFM69_DEFAULT_GATEWAY_ID;
    dev.settings.network_id = RFM69_DEFAULT_NETWORKID;
    dev.settings.freq_band = RFM69_433MHZ;
    dev.settings.power_level_dbm = RFM69_DEFAULT_POWER_LEVEL_DBM;
    dev.settings.promiscuous_mode = false;
    dev.settings.is_rfm69hw = RFM69_DEFAULT_IS_RFM69HW;
    dev.settings.aes_encrypt_enable = true;
    memcpy(dev.settings.aes_encrypt_key, RFM69_DEFAULT_ENCRYPTKEY, RFM69_AES_KEY_LEN);
    dev.on_receive = NULL;

    if (!rfm69_arduino_init(&dev, &arduino_dev))
    {
        Serial.println("RFM69 init failed!");
    }

    rfm69_set_high_power(&dev, true);
    rfm69_receive_begin(&dev);
}

void loop()
{
    if (!rfm69_receive_done(&dev))
    {
        return;
    }

    const rfm69_payload_data* rx = rfm69_received_data(&dev);

    Serial.print("From node: "); Serial.print(rx->sender_id);
    Serial.print(", RSSI: "); Serial.println(rx->rssi);

    if (rx->data_len == sizeof(frame_t))
    {
        frame_t frame;
        memcpy(&frame, rx->data, sizeof(frame));
        Serial.print("  node_id="); Serial.print(frame.node_id);
        Serial.print(" uptime="); Serial.print(frame.uptime);
        Serial.print(" temp="); Serial.println(frame.temp);
    }
    else
    {
        Serial.println("Invalid payload received, not matching frame_t struct!");
    }

    if (rfm69_ack_requested(&dev))
    {
        rfm69_send_ack(&dev);
        Serial.println(" - ACK sent.");
    }
}
```

### Notes
* For RFM69HW/HCW modules you **must** call `rfm69_set_high_power(&dev, true)`
  after initialization, otherwise transmission will not work. Do not enable it
  for RFM69W/CW modules.
* The AES key has to be exactly `RFM69_AES_KEY_LEN` (16) bytes long and
  identical on the node and on the gateway.
* Automatic transmit power control can be turned on with
  `rfm69_enable_auto_power(&dev, RFM69_DEFAULT_PWR_ATC_TARGET_RSSI)`.

## Porting to a different platform
To port this library to another platform you only have to implement the
HW dependent functions declared at the bottom of `rfm69.h`:

* `rfm69_io_init()`
* `rfm69_io_deinit()`
* `rfm69_ioirq_init()`
* `rfm69_reset()`
* `rfm69_write_buffer()`
* `rfm69_read_buffer()`
* `rfm69_delay_ms()`
* `rfm69_timer_read_ms()`
* `rfm69_print_all_regs()`

and provide an `rfm69_<platform>_init()` helper that stores your platform
descriptor in `dev->platform_dev`. See `platform/arduino/rfm69-arduino.cpp`
for a reference implementation.

## License
MIT - see the repository root for details.

## Authors
* Felix Rusu, LowPowerLab.com
* Lukasz Uszko - lukasz.uszko@gmail.com

* Lukasz Uszko aka `luk6xff` [lukasz.uszko@gmail.com]
