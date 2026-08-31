# LORA SX127x radio transceiver

## Description
Platform independent driver for the Semtech SX1276/77/78/79 (SX127x) LoRa radio
transceivers, deeply based on https://github.com/sandeepmistry/arduino-LoRa.

The core (`lora.c` / `lora.h`) is written in plain C and knows nothing
about the underlying hardware. Everything hardware related is provided by a thin
platform layer living in `platform/<platform>`.

* Interface: SPI + NSS, RESET and DIO0 (interrupt) pins
* Max packet length: `LORA_MAX_PKT_LEN` (255 bytes)
* Configurable frequency, spreading factor, signal bandwidth, coding rate,
  preamble length, sync word, TX power, LNA gain, OCP and CRC
* Explicit and implicit header modes, blocking and asynchronous TX
* RX/TX completion delivered through the `on_receive` / `on_tx_done` callbacks
  registered with `lora_on_receive()` / `lora_on_tx_done()`
* Link quality via `lora_packet_rssi()`, `lora_packet_snr()` and
  `lora_packet_frequency_error()`

The library was tested with [MBED](https://www.mbed.com) on RA-01 and RA-02
boards from AI-Thinker. It should work for all SX127x chips from SEMTECH.

## Supported platforms
| Platform | Files |
|---|---|
| Arduino | `platform/arduino/lora-arduino.{h,cpp}` |
| Mbed | `platform/mbed/lora-mbed.{h,cpp}` |
| STM32 Cube HAL | `platform/stm32cube/lora-cube-hal.{h,c}` |

## Usage
Demo code showing how to use the library on the `ARDUINO` platform:

```cpp
#include "LORA/platform/arduino/lora-arduino.h"

// PINOUT FOR ESP32 WROOM DEVBOARD
#define RADIO_MOSI_PIN 23
#define RADIO_MISO_PIN 19
#define RADIO_SCK_PIN  18
#define RADIO_NSS_PIN   5
#define RADIO_RST_PIN  14
#define RADIO_DI0_PIN  26

lora dev;
lora_arduino arduino_dev;

void setup()
{
    Serial.begin(115200);

    arduino_dev.spi = &SPI;
    arduino_dev.mosi = RADIO_MOSI_PIN;
    arduino_dev.miso = RADIO_MISO_PIN;
    arduino_dev.sck = RADIO_SCK_PIN;
    arduino_dev.nss = RADIO_NSS_PIN;
    arduino_dev.reset = RADIO_RST_PIN;
    arduino_dev.dio0 = RADIO_DI0_PIN;

    dev.frequency = 433E6;
    dev.on_receive = NULL;
    dev.on_tx_done = NULL;

    if (!lora_arduino_init(&dev, &arduino_dev))
    {
        Serial.println("LORA Radio cannot be detected!, check your connections.");
    }
}

void loop()
{
    const char msg[] = "Hello LORA!";

    lora_begin_packet(&dev, false);
    lora_write_data(&dev, (const uint8_t*)msg, sizeof(msg));
    lora_end_packet(&dev, false);

    // Switch back into RX mode
    lora_receive(&dev, 0);
    lora_delay_ms(5000);
}
```

Simple snippet how to quickly use the library for LORA communication between two devices on `mbed` platform:

* Common (for Sender and Receiver)
```cpp
#define LORA_RADIO_RECEIVER_ID  0xAB
#define LORA_RADIO_SENDER_ID    0xCD

/* Pinout dependednt on your hw */
#define RADIO_MOSI_PIN  PA_5
#define RADIO_MISO_PIN  PA_6
#define RADIO_SCK_PIN   PA_7
#define RADIO_NSS_PIN   PA_8
#define RADIO_RST_PIN   PA_9
#define RADIO_DI0_PIN   PA_10

/**
 * @brief Example radio layer msg header footprint
 */
typedef struct
{
	uint8_t receiver_id;           // Receiver address
	uint8_t sender_id;             // Sender address
	uint8_t msg_id;                // Message ID
	uint8_t payload_len;           // Message payload length
} radio_layer_msg_header;

/**
 * @brief Example radio sensor msg frame footprint
 */
typedef struct
{
	uint8_t hdr[3];
	uint8_t status;
	float sensor_data;
} radio_msg_sensor_frame;
```

* Sender
```cpp
#include "LORA/platform/mbed/lora-mbed.h"

void radio_send(radio_msg_sensor_frame *msgf)
{
    static uint8_t msg_cnt = 0;
    if (!msgf)
    {
        return;
    }

    // Send result data
    radio_layer_msg_header hdr;
    hdr.receiver_id = LORA_RADIO_RECEIVER_ID;
    hdr.sender_id = LORA_RADIO_SENDER_ID;
    hdr.msg_id = msg_cnt++;
    hdr.payload_len = sizeof(radio_msg_sensor_frame);
    lora_begin_packet(&dev, false);                     // start packet
    lora_write_data(&dev, (const uint8_t*)&hdr, sizeof(radio_layer_msg_header));
    lora_write_data(&dev, (const uint8_t*)msgf, sizeof(radio_msg_sensor_frame));
    lora_end_packet(&dev, false);                       // finish packet and send it
    // Switch to RX mode
    lora_receive(&dev, 0);
}

int main()
{
    lora dev;
    lora_mbed mbed_dev;

    // Set mbed_dev
    SPI spi(RADIO_MOSI_PIN, RADIO_MISO_PIN, RADIO_SCK_PIN);
    DigitalOut nss(RADIO_NSS_PIN);
    DigitalInOut rst(RADIO_RST_PIN);
    InterruptIn dio0(RADIO_DI0_PIN);

    mbed_dev.spi =   &spi;
    mbed_dev.nss =   &nss;
    mbed_dev.reset = &rst;
    mbed_dev.dio0 =  &dio0;

    // Set dev
    dev.frequency = 433E6;
    dev.on_receive = NULL;
    dev.on_tx_done = NULL;

    const uint8_t attempts_num = 5;
    uint8_t attempts = 0;
    while (!lora_mbed_init(&dev, &mbed_dev) && (attempts++ < attempts_num))
    {
        printf("LORA Radio cannot be detected!, check your connections.");
        lora_delay_ms(2000);
    }

    // Msg frame
    radio_msg_sensor_frame msgf =
    {
        .hdr = {'L','U','6'},
    };

    while (1)
    {
        /* Set data */
        msgf.status = 0;
        msgf.sensor_data = 12.6;
        /* Send data to the receiver */
	    radio_send(&msgf);
        lora_delay_ms(5000);
    }
}

```
* Receiver
```cpp
#include "LORA/platform/mbed/lora-mbed.h"

static void on_rx_done(void *ctx, int packet_size)
{
    lora *const dev = (lora*const) ctx;

    radio_layer_msg_header hdr;
    radio_msg_sensor_frame frame;
    uint8_t *p = (uint8_t*)&frame;
    size_t i = 0;

    if (packet_size == 0)
    {
        goto err;
    }

    // Read packet header bytes:
    hdr.receiver_id = lora_read(dev);   // recipient address
    hdr.sender_id = lora_read(dev);     // sender address
    hdr.msg_id = lora_read(dev);        // incoming msg ID
    hdr.payload_len = lora_read(dev);   // incoming msg length

    // If the recipient is valid,
    if (hdr.receiver_id != 0xFE)
    {
        goto err;
    }

    // Check payload length
    if (hdr.payload_len != sizeof(radio_msg_sensor_frame))
    {
        goto err;
    }

    // Read payload frame
    while (lora_available(dev) && i < hdr.payload_len)
    {
        *(p+i) = (uint8_t)lora_read(dev);
        i++;
    }

    // Do whatever you want with received data
    printf("my_received_sensor_data:%3.2f", frame.sensor_data);
    printf("rssi:%d, snr:%3.2f", lora_packet_rssi(dev), lora_packet_snr(dev));

err:
    // Switch back into rx mode
    lora_receive(dev, 0);
}

int main()
{
    lora dev;
    lora_mbed mbed_dev;

    // Set mbed_dev
    SPI spi(RADIO_MOSI_PIN, RADIO_MISO_PIN, RADIO_SCK_PIN);
    DigitalOut nss(RADIO_NSS_PIN);
    DigitalInOut rst(RADIO_RST_PIN);
    InterruptIn dio0(RADIO_DI0_PIN);

    mbed_dev.spi =   &spi;
    mbed_dev.nss =   &nss;
    mbed_dev.reset = &rst;
    mbed_dev.dio0 =  &dio0;

    // Set dev
    dev.frequency = 433E6;
    dev.on_receive = NULL;
    dev.on_tx_done = NULL;

    const uint8_t attempts_num = 5;
    uint8_t attempts = 0;
    while (!lora_mbed_init(&dev, &mbed_dev) && (attempts++ < attempts_num))
    {
        printf("LORA Radio cannot be detected!, check your connections.");
        lora_delay_ms(2000);
    }
    lora_on_receive(&dev, on_rx_done);

    while (1)
    {
    }
}


```

## Porting to a different platform
To port this library to another platform you only have to implement the
HW dependent functions declared at the bottom of `lora.h`:

* `lora_io_init()`
* `lora_io_deinit()`
* `lora_ioirq_init()`
* `lora_ioirq_deinit()`
* `lora_reset()`
* `lora_write_buffer()`
* `lora_read_buffer()`
* `lora_delay_ms()`
* `lora_timer_read_ms()`
* `lora_dump_registers()`

and provide a `lora_<platform>_init()` helper that stores your platform
descriptor in `dev->platform_dev`. See `platform/mbed/lora-mbed.cpp`
for a reference implementation.

## License
MIT - see the repository root for details.

## Author
Lukasz Uszko - lukasz.uszko@gmail.com

* Lukasz Uszko aka `luk6xff` [lukasz.uszko@gmail.com]
