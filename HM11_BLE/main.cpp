#include "mbed.h"
#include "hm11.h"
#include <cstring>

#define HM11_PIN_TX PTE22 //PTC4   // FRDM-KL25Z UART2 //PTE22  //YELLOW wire 
#define HM11_PIN_RX PTE23//PTC3   //                  //PTE23  //ORANGE wire



#define SIMPLE_DATA_TEST_ENABLED



const uint8_t broadcast_msg1[]={0x42, 0x4C, 0x45, 0x01, 0x45, 0x53, 0x45, 0x06};

const uint8_t data_msg1[]={0x45, 0x53, 0x45, 0x06, 0x24, 0x23};

int main() {
    Serial usbDebug(USBTX, USBRX);
    usbDebug.printf("HELLO WORLD !");  
    

    HM11* hm11 = new HM11( HM11_PIN_TX, HM11_PIN_RX);

    int counter =0;
    while(1) {
        wait(0.5);
        usbDebug.printf("alive ");   
        wait(0.5);
        


#ifdef SIMPLE_DATA_TEST_ENABLED
        char buf[2];
        snprintf(buf,2,"%d",counter++);
        if(counter>9)
            counter=0;
        hm11->sendDataToDevice(buf);
        wait(0.2);
        
         while(hm11->isRxDataAvailable())                    
               usbDebug.printf("data:  %c\r\n",hm11->getDataFromRx());
#endif          


 
        
    }
}
