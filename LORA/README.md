# LORA (for SX127x modules) library


## Description
The library was tested with [MBED](https://www.mbed.com) platform on RA-01 and RA-02 boards from AI-Thinker. It should work for all LORA SX127x chips from SEMTECH.

## Usage
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

## Porting to other platform
If you want to port this library on other platform, the only thing you have to do is define HW/Platform dependent functions as it is done in `platform/mbed/lora-mbed.cpp` file.


## Author
* Lukasz Uszko aka `luk6xff` [lukasz.uszko@gmail.com]
