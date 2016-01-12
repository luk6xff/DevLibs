/*
  Example of usage of the RFM22 library  
  @Author lukasz uszko(luszko@op.pl)

  Tested on most mbed platforms
  
  Copyright (c) 2015 lukasz uszko
  Released under the MIT License (see http://mbed.org/license/mit)
*/



//include your libraries
#include "RF22ReliableDatagram.h"

//// CLIENT_ MODULE
//pinout for FRDM-KL25Z platform
#define RFM_PIN_SDO  PTA17     //MISO                  
#define RFM_PIN_SDI   PTA16    //MOSI
#define RFM_PIN_SCLK  PTA15    //SCK
#define RFM_PIN_nSEL  PTA14
#define RFM_PIN_nIRQ  PTA6     //(as IRQ)
#define RFM_PIN_SDN   PTB9

#define CLIENT_ADDRESS 1
#define SERVER_ADDRESS 2
int main()
{
    RF22ReliableDatagram rfm23b(RF22ReliableDatagram(CLIENT_ADDRESS,RFM_PIN_nSEL, RFM_PIN_SDI ,RFM_PIN_SDO,RFM_PIN_SCLK,RFM_PIN_nIRQ,RFM_PIN_SDN  );
    if(!rfm23b.init()) {
        DEBUG("rfm23b init failed\r\n");
        while(1);
    } else
        DEBUG("rfm23b init succes\r\n");
    
    
    
    uint8_t data[] = "DATA_FROM_WIFI_MODULE";;
    uint8_t buf[40];
    uint8_t len = sizeof(buf);
    uint8_t from;
    std::string serializedData;
    
    
    while(1) {
        if (rfm23b.recvfromAck(buf, &len, &from)) {
            DEBUG("got request from : 0x%x",from);
            DEBUG(": ");
            DEBUG("%s",(char*)buf);
            DEBUG(serializeDataPacket().c_str());
            serializedData = serializeDataPacket(); //own method for parsing incoming data
            // Send a reply to the second module
            if (!rfm23b.sendtoWait((uint8_t*)serializedData.c_str(), serializedData.length(), from))
            {
                DEBUG("response sending failed");
            }
         
        }
    }
    
}













//// SERVER MODULE
#define RFM_PIN_SDO  PTA17     //MISO                  
#define RFM_PIN_SDI   PTA16    //MOSI
#define RFM_PIN_SCLK  PTA15    //SCK
#define RFM_PIN_nSEL  PTA14
#define RFM_PIN_nIRQ  PTA6     //(as IRQ)
#define RFM_PIN_SDN   PTB9

#define CLIENT_ADDRESS 1
#define SERVER_ADDRESS 2
int main()
{
    RF22ReliableDatagram rfm23b(RF22ReliableDatagram(SERVER_ADDRESS,RFM_PIN_nSEL, RFM_PIN_SDI ,RFM_PIN_SDO,RFM_PIN_SCLK,RFM_PIN_nIRQ,RFM_PIN_SDN  );
    if(!rfm23b.init()) {
        DEBUG("rfm23b init failed\r\n");
        while(1);
    } else
        DEBUG("rfm23b init succes\r\n");
    
    
    
    uint8_t data[] = "DATA_FROM_WIFI_MODULE";;
    uint8_t respBuf[40];
    uint8_t len = sizeof(buf);
    uint8_t from;
       
    while(1) {
             
        if (!rfm23b.sendtoWait(data, sizeof(data), CLIENT_ADDRESS)) 
        {
            DEBUG("sending request to Client failed...");
        } 
        else 
        {
            if (rfm23b.recvfromAckTimeout(respBuf, &len, 1000, &from)) {
                DEBUG("received data from Client : 0x%d", from);
                DEBUG("DATA %s", (char*)respBuf);
            }
        }
    }
    
}
