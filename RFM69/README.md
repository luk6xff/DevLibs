# RFM69 Library

RFM69 library for RFM69W, RFM69HW, RFM69CW, RFM69HCW (semtech SX1231, SX1231H) based on great library from [LowPowerLab](https://github.com/LowPowerLab/RFM69)

## Description
The library was tested with [MBED](https://www.mbed.com) platform on RFM69 boards Hope-RF.

## Usage
Simple snippet how to quickly use the library for communication betwwen two RFM69 modules on mbed platform:

### CLIENT
#### MBED:
```cpp
#include "RFM69/platform/rfm69-mbed.h"

// Defines
#define NODEID          99
#define NETWORKID       100
#define GATEWAYID       1
#define FREQUENCY       RF69_915MHZ
#define ENCRYPTKEY      "YourEncryptKey12" // Has to be same 16 characters/uint8_ts on gateway
#define IS_RFM69HW_HCW  // Uncomment only for RFM69HW/HCW! Remove if you have RFM69W/CW!
#define TRANSMITPERIOD  1000 // Transmit a packet to gateway so often (in ms)
#define SERIAL_BAUD     9600

// Datatypes
typedef struct 
{
  uint8_t  node_id; // Store this node_id
  uint32_t uptime; // Uptime in ms
  float    temp;   // Temperature maybe?
} Frame_t;


// Private variables
static Frame_t frame;
const RadioSettings_t settings = 
{
    FREQUENCY,
    NODEID,
    NETWORKID
};

// Functions
int main() 
{
    Serial dbg(USBTX, USBRX);
    dbg.baud(9600);
    rfm69_mbed_init(&settings, /***!!! ADD HERE YOUR PIN SETTINGS !!!***/);
#ifdef IS_RFM69HW_HCW
    rfm69_set_high_power();
#endif
    RFM69Encrypt(ENCRYPTKEY);
    debug(">>> RFM69 demo app <<<");

    long lastPeriod = -1;
    while(1)
    {
        //check for any received packets
        if (RFM69receiveDone())
        {
            debug("[SENDERID: %d]", rfm69_received_data()->SENDERID);
            for (uint8_t i = 0; i < rfm69_received_data()->DATALEN; i++)
            {
                debug("%c", (char)rfm69_received_data()->DATA[i]);
            }
            debug("[RX_RSSI: %d]", RFM69readRSSI());

            if (rfm69_ack_requested())
            {
            RFM69sendACK();
            debug(" - ACK sent");
            RFMDelayMs(10);
            }
            Blink(LED_BUILTIN,5);
        }
    
        int currPeriod = rfm69_timer_read_ms()/TRANSMITPERIOD;
        if (currPeriod != lastPeriod)
        {
            frame.node_id = NODEID;
            frame.uptime = rfm69_timer_read_ms();
            frame.temp = rfm69_read_temperature(0);
            
            debug("Sending struct of size:(%d)", sizeof(frame));
            if (RFM69sendWithRetry(GATEWAYID, (const void*)(&frame), sizeof(frame)))
            {
                debug("Frame sent!");
            }
            else 
            {
                debug("Frame has not been sent!");
            }
            lastPeriod=currPeriod;
        }
    }
}
```
#### ARDUINO:
```cpp
#include "RFM69/platform/rfm69-arduino.h"

// Defines
#define NODEID          99
#define NETWORKID       100
#define GATEWAYID       1
#define FREQUENCY       RF69_915MHZ
#define ENCRYPTKEY      "YourEncryptKey12" // Has to be same 16 characters/uint8_ts on gateway
#define IS_RFM69HW_HCW  // Uncomment only for RFM69HW/HCW! Remove if you have RFM69W/CW!
#define TRANSMITPERIOD  1000 // Transmit a packet to gateway so often (in ms)
#define SERIAL_BAUD     9600

// Datatypes
typedef struct 
{
  uint8_t  node_id; // Store this node_id
  uint32_t uptime; // Uptime in ms
  float    temp;   // Temperature maybe?
} Frame_t;


// Private variables
static Frame_t frame;
const RadioSettings_t settings = 
{
    FREQUENCY,
    NODEID,
    NETWORKID
};

// Functions
void setup() {
    Serial.begin(SERIAL_BAUD);
    rfm69_arduino_init(&settings, /***!!! ADD HERE YOUR PIN SETTINGS !!!***/);
#ifdef IS_RFM69HW_HCW
    rfm69_set_high_power();
#endif
    RFM69Encrypt(ENCRYPTKEY);
    debugn(">>> RFM69 demo app <<<");
}

long lastPeriod = -1;
void loop()
{
    //check for any received packets
    if (RFM69receiveDone())
    {
        Serial.print('[');Serial.print(RFM69SENDERID, DEC);Serial.print("] ");
        for (uint8_t i = 0; i < RFM69DATALEN; i++)
        Serial.print((char)RFM69DATA[i]);
        Serial.print("   [RX_RSSI:");Serial.print(RFM69readRSSI());Serial.print("]");

        if (rfm69_ack_requested())
        {
        RFM69sendACK();
        Serial.print(" - ACK sent");
        RFMDelayMs(10);
        }
        Blink(LED_BUILTIN,5);
        debugn();
    }
  
    int currPeriod = rfm69_timer_read_ms()/TRANSMITPERIOD;
    if (currPeriod != lastPeriod)
    {
        frame.node_id = NODEID;
        frame.uptime = rfm69_timer_read_ms();
        frame.temp = rfm69_read_temperature(0);
        
        Serial.print("Sending struct (");
        Serial.print(sizeof(frame));
        Serial.print(" uint8_ts) ... ");
        if (RFM69sendWithRetry(GATEWAYID, (const void*)(&frame), sizeof(frame)))
        {
            Serial.print(" Frame sent!");
        }
        else 
        {
            Serial.print(" Frame has not been sent!");
        }
        debugn();
        Blink(LED_BUILTIN,3);
        lastPeriod=currPeriod;
    }
}

void Blink(uint8_t PIN, int DELAY_MS)
{
  pinMode(PIN, OUTPUT);
  digitalWrite(PIN,HIGH);
  RFMDelayMs(DELAY_MS);
  digitalWrite(PIN,LOW);
}
```



### GATEWAY
#### MBED:
```cpp
#include "RFM69/platform/rfm69-mbed.h"

// Defines
#define NODEID          99
#define NETWORKID       100
#define GATEWAYID       1
#define FREQUENCY       RF69_915MHZ
#define ENCRYPTKEY      "YourEncryptKey12" // Has to be same 16 characters/uint8_ts on gateway
#define IS_RFM69HW_HCW  // Uncomment only for RFM69HW/HCW! Remove if you have RFM69W/CW!
#define TRANSMITPERIOD  1000 // Transmit a packet to gateway so often (in ms)
#define SERIAL_BAUD     9600

// Datatypes
typedef struct 
{
    uint8_t  node_id; // Store this node_id
    uint32_t uptime; // Uptime in ms
    float    temp;   // Temperature maybe?
} Frame_t;


// Private variables
static bool promiscuousMode = false; // Set to 'true' to sniff all packets on the same network
static Frame_t frame;
const RadioSettings_t settings = 
{
    FREQUENCY,
    NODEID,
    NETWORKID
};

// Functions
int main() 
{
    Serial dbg(USBTX, USBRX);
    dbg.baud(9600);
    rfm69_arduino_init(&settings, /***!!! ADD HERE YOUR PIN SETTINGS !!!***/);
#ifdef IS_RFM69HW_HCW
    rfm69_set_high_power();
#endif
    RFM69Encrypt(ENCRYPTKEY);
    rfm69_set_promiscuous_mode(promiscuousMode);
    debug(">>> RFM69 demo app <<<");

    uint8_t ackCount=0;
    while(1) 
    {
        if (rfm69_receive_done())
        {
            debug("from SENDERID[%d] ", rfm69_received_data()->SENDERID);
            debug(" [RX_RSSI: %d]", rfm69_read_rssi());
            if (promiscuousMode)
            {
                debug("to TARGETID:[%d]", rfm69_received_data()->TARGETID);
            }

            if (rfm69_received_data()->DATALEN != sizeof(Payload))
            {
                debug("Invalid payload received, not matching Payload struct!");
            }
            else
            {
                frame = *(Payload*)rfm69_received_data()->DATA; // Assume rfm69_received_data()->DATA actually contains our struct
                debug(" node_id=");
                debug(frame.node_id);
                debug(" uptime=");
                debug(frame.uptime);
                debug(" temp=");
                debug(frame.temp);
            }
            
            if (rfm69_ack_requested())
            {
                uint8_t theNodeID = rfm69_received_data()->SENDERID;
                rfm69_send_ack();
                debug(" - ACK sent.");

                // When a node requests an ACK, respond to the ACK
                // and also send a packet requesting an ACK (every 3rd one only)
                // This way both TX/RX NODE functions are tested on 1 end at the GATEWAY
                if (ackCount++%3==0)
                {
                    debug(" Pinging node ");
                    debug("%d", theNodeID);
                    debug(" - ACK...");
                    RFMDelayMs(3); // Need this when sending right after reception .. ?
                    if (rfm69_send_with_retry(theNodeID, "ACK TEST", 8, 0))  // 0 = only 1 attempt, no retries
                    {
                        debug("ok!");
                    }
                    else
                    {
                        debug("nothing");
                    }
                }
            }
        }
    }
    return 0;
}
```





#### ARDUINO:
```cpp
#include "RFM69/platform/rfm69-arduino.h"

// Defines
#define NODEID          99
#define NETWORKID       100
#define GATEWAYID       1
#define FREQUENCY       RF69_915MHZ
#define ENCRYPTKEY      "YourEncryptKey12" // Has to be same 16 characters/uint8_ts on gateway
#define IS_RFM69HW_HCW  // Uncomment only for RFM69HW/HCW! Remove if you have RFM69W/CW!
#define TRANSMITPERIOD  1000 // Transmit a packet to gateway so often (in ms)
#define SERIAL_BAUD     9600

// Datatypes
typedef struct 
{
    uint8_t  node_id; // Store this node_id
    uint32_t uptime; // Uptime in ms
    float    temp;   // Temperature maybe?
} Frame_t;


// Private variables
static bool promiscuousMode = false; // Set to 'true' to sniff all packets on the same network
static Frame_t frame;
const RadioSettings_t settings = 
{
    FREQUENCY,
    NODEID,
    NETWORKID
};

// Functions
void setup() {
    Serial.begin(SERIAL_BAUD);
    rfm69_arduino_init(&settings, /***!!! ADD HERE YOUR PIN SETTINGS !!!***/);
#ifdef IS_RFM69HW_HCW
    rfm69_set_high_power();
#endif
    RFM69Encrypt(ENCRYPTKEY);
    rfm69_set_promiscuous_mode(promiscuousMode);
    Serial.println(">>> RFM69 demo app <<<");
}

uint8_t ackCount=0;
void loop() 
{
    if (rfm69_receive_done())
    {
        Serial.print('[');Serial.print(rfm69_received_data()->SENDERID, DEC);Serial.print("] ");
        Serial.print(" [RX_RSSI:");Serial.print(rfm69_read_rssi());Serial.print("]");
        if (promiscuousMode)
        {
            Serial.print("to [");Serial.print(rfm69_received_data()->TARGETID, DEC);Serial.print("] ");
        }

        if (rfm69_received_data()->DATALEN != sizeof(Payload))
        {
            Serial.print("Invalid payload received, not matching Payload struct!");
        }
        else
        {
            frame = *(Payload*)rfm69_received_data()->DATA; // Assume rfm69_received_data()->DATA actually contains our struct
            Serial.print(" node_id=");
            Serial.print(frame.node_id);
            Serial.print(" uptime=");
            Serial.print(frame.uptime);
            Serial.print(" temp=");
            Serial.print(frame.temp);
        }
        
        if (rfm69_ack_requested())
        {
            uint8_t theNodeID = rfm69_received_data()->SENDERID;
            rfm69_send_ack();
            Serial.print(" - ACK sent.");

            // When a node requests an ACK, respond to the ACK
            // and also send a packet requesting an ACK (every 3rd one only)
            // This way both TX/RX NODE functions are tested on 1 end at the GATEWAY
            if (ackCount++%3==0)
            {
                Serial.print(" Pinging node ");
                Serial.print(theNodeID);
                Serial.print(" - ACK...");
                RFMDelayMs(3); // Need this when sending right after reception .. ?
                if (rfm69_send_with_retry(theNodeID, "ACK TEST", 8, 0))  // 0 = only 1 attempt, no retries
                {
                    Serial.print("ok!");
                }
                else
                {
                    Serial.print("nothing");
                }
            }
        }
        Serial.println();
        Blink(LED_BUILTIN,3);
    }
}

void Blink(uint8_t PIN, int DELAY_MS)
{
  pinMode(PIN, OUTPUT);
  digitalWrite(PIN,HIGH);
  RFMDelayMs(DELAY_MS);
  digitalWrite(PIN,LOW);
}
```


## Porting to other platform
If you want to port this library on other platform, the only thing you have to do is define HW/Platform dependent functions as it is done in `platform/rfm69-mbed.cpp` file.


## Authors
* Felix Rusu, LowPowerLab.com
* Lukasz Uszko aka `luk6xff` [lukasz.uszko@gmail.com]