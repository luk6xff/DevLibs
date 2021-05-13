# This code is deprecated! Use my new [LORA library](https://github.com/luk6xff/DevLibs/tree/master/LORA) instead.

# SX1278 LORA module library


## Description 
The library was tested with [MBED](https://www.mbed.com) platform on RA-01 and RA-02 boards from AI-Thinker. It should work for all LORA SX127x chips from SEMTECH.

## Usage
Simple snippet how to quickly use the library for LORA communication on mbed platform:
* CLIENT
```cpp

#include "SX1278/platform/sx1278-mbed.h"

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
#define SX1278_DIO4  PB_5
#define SX1278_DIO5  PB_3

// Defines
#define DEBUG_ON 1

// Radio LORA settings
#define RF_FREQUENCY                                RF_FREQUENCY_434_0
#define TX_OUTPUT_POWER                             14      // dBm
#define LORA_BANDWIDTH                              LORA_BANDWIDTH_125kHz
#define LORA_SPREADING_FACTOR                       LORA_SF8
#define LORA_CODINGRATE                             LORA_ERROR_CODING_RATE_4_5
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         5         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_FHSS_ENABLED                           false  
#define LORA_NB_SYMB_HOP                            4     
#define LORA_IQ_INVERSION_ON                        false
#define LORA_CRC_ENABLED                            true
#define RX_TIMEOUT_VALUE                            8000      // in ms


// Radio events function pointer
static RadioEvents_t RadioEvents;

// Function to be executed on Radio Tx Done event
void OnTxDone(void);

// Function to be executed on Radio Rx Done event
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);

// Function executed on Radio Tx Timeout event
void OnTxTimeout(void);

// Function executed on Radio Rx Timeout event
void OnRxTimeout(void);

// Function executed on Radio Rx Error event
void OnRxError(void);

const char GatewayMsg[] = "Hello FROM LORA GATEWAY!";
const char ClientMsg[] = "Hello FROM LORA CLIENT!";

//-----------------------------------------------------------------------------

// Main
int main()
{
    // Radio initialization
    RadioEvents.TxDone = OnTxDone;
    RadioEvents.RxDone = OnRxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    RadioEvents.RxTimeout = OnRxTimeout;
    RadioEvents.RxError = OnRxError;
    SX1278MbedInit(&RadioEvents,SX1278_MOSI, SX1278_MISO, SX1278_SCLK, SX1278_NSS, SX1278_RST, \
                    SX1278_DIO0, SX1278_DIO1, SX1278_DIO2, SX1278_DIO3, NC, NC);
    SX1278SetChannel(RF_FREQUENCY);

    // Verify if SX1278 connected to the the board
    while(SX1278Read(REG_VERSION) == 0x00)
    {
        debug("Radio could not be detected!\n\r");
        wait(1);
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

    while(true)
    {
        SX1278Send((uint8_t*)ClientMsg, sizeof(ClientMsg));
        SX1278SetRx(RX_TIMEOUT_VALUE);
        SX1278DelayMs(RX_TIMEOUT_VALUE);
        debug_if(DEBUG_ON, "> Data sent to the client\n\r");
    }
}

//-----------------------------------------------------------------------------
void OnTxDone(void)
{
    SX1278SetSleep();
    debug_if(DEBUG_ON, "> OnTxDone\n\r");
}

//-----------------------------------------------------------------------------
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
    SX1278SetSleep();
    debug_if(DEBUG_ON, "> RssiValue: %d\n\r", rssi);
    debug_if(DEBUG_ON, "> SnrValue: %d\n\r", snr);
    debug_if(DEBUG_ON, "> PAYLOAD: %s\n\r", payload);
    debug_if(DEBUG_ON, "> OnRxDone\n\r");
}

//-----------------------------------------------------------------------------
void OnTxTimeout(void)
{
    SX1278SetSleep();
    debug_if(DEBUG_ON, "> OnTxTimeout\n\r");
}
 
//-----------------------------------------------------------------------------
void OnRxTimeout(void)
{
    SX1278SetSleep();
    debug_if(DEBUG_ON, "> OnRxTimeout\n\r");
}
 
//-----------------------------------------------------------------------------
void OnRxError(void)
{
    SX1278SetSleep();
    debug_if(DEBUG_ON, "> OnRxError\n\r");
}

//-----------------------------------------------------------------------------

```

* GATEWAY
```cpp

#include "SX1278/platform/sx1278-mbed.h"

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
#define SX1278_DIO4  PB_5
#define SX1278_DIO5  PB_3

// Defines
#define DEBUG_ON 1

// Radio LORA settings
#define RF_FREQUENCY                                RF_FREQUENCY_434_0
#define TX_OUTPUT_POWER                             14      // dBm
#define LORA_BANDWIDTH                              LORA_BANDWIDTH_125kHz
#define LORA_SPREADING_FACTOR                       LORA_SF8
#define LORA_CODINGRATE                             LORA_ERROR_CODING_RATE_4_5
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         5         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_FHSS_ENABLED                           false  
#define LORA_NB_SYMB_HOP                            4     
#define LORA_IQ_INVERSION_ON                        false
#define LORA_CRC_ENABLED                            true
#define RX_TIMEOUT_VALUE                            8000      // in ms


// Radio events function pointer
static RadioEvents_t RadioEvents;

// Function to be executed on Radio Tx Done event
void OnTxDone(void);

// Function to be executed on Radio Rx Done event
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);

// Function executed on Radio Tx Timeout event
void OnTxTimeout(void);

// Function executed on Radio Rx Timeout event
void OnRxTimeout(void);

// Function executed on Radio Rx Error event
void OnRxError(void);

const char GatewayMsg[] = "Hello FROM LORA GATEWAY!";
const char ClientMsg[] = "Hello FROM LORA CLIENT!";

//-----------------------------------------------------------------------------

// Main
int main()
{
    // Radio initialization
    RadioEvents.TxDone = OnTxDone;
    RadioEvents.RxDone = OnRxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    RadioEvents.RxTimeout = OnRxTimeout;
    RadioEvents.RxError = OnRxError;
    SX1278MbedInit(&RadioEvents,SX1278_MOSI, SX1278_MISO, SX1278_SCLK, SX1278_NSS, SX1278_RST, \
                    SX1278_DIO0, SX1278_DIO1, SX1278_DIO2, SX1278_DIO3, NC, NC);
    SX1278SetChannel(RF_FREQUENCY);

    // Verify if SX1278 connected to the the board
    while(SX1278Read(REG_VERSION) == 0x00)
    {
        debug("Radio could not be detected!\n\r");
        wait(1);
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

    while(true)
    {
        SX1278SetRx(RX_TIMEOUT_VALUE);
    }
}

//-----------------------------------------------------------------------------
void OnTxDone(void)
{
    SX1278SetSleep();
    debug_if(DEBUG_ON, "> OnTxDone\n\r");
}

//-----------------------------------------------------------------------------
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
    SX1278Send((uint8_t*)GatewayMsg, sizeof(GatewayMsg));
    SX1278SetSleep();
    debug_if(DEBUG_ON, "> RssiValue: %d\n\r", rssi);
    debug_if(DEBUG_ON, "> SnrValue: %d\n\r", snr);
    debug_if(DEBUG_ON, "> PAYLOAD: %s\n\r", payload);
    debug_if(DEBUG_ON, "> OnRxDone\n\r");
}

//-----------------------------------------------------------------------------
void OnTxTimeout(void)
{
    SX1278SetSleep();
    debug_if(DEBUG_ON, "> OnTxTimeout\n\r");
}
 
//-----------------------------------------------------------------------------
void OnRxTimeout(void)
{
    SX1278SetSleep();
    debug_if(DEBUG_ON, "> OnRxTimeout\n\r");
}
 
//-----------------------------------------------------------------------------
void OnRxError(void)
{
    SX1278SetSleep();
    debug_if(DEBUG_ON, "> OnRxError\n\r");
}
```


## Porting to other platform
If you want to port this library on other platform, the only thing you have to do is define HW/Platform dependent functions as it is done in `platform/sx1278-mbed.cpp` file.


## Author
* Lukasz Uszko aka `luk6xff` [luszko@op.pl]