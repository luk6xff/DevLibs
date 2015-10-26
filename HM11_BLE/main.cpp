#include "mbed.h"
#include "hm11.h"
#include "crc16.h"

#define HM11_PIN_TX PTE22 //PTC4   // FRDM-KL25Z UART2 //PTE22  //YELLOW wire 
#define HM11_PIN_RX PTE23//PTC3   //                  //PTE23  //ORANGE wire


#define CRC_TEST_ENABLED_0

DigitalOut myled(LED1);




int main() {
    Serial usbDebug(USBTX, USBRX);
    usbDebug.printf("HELLO WORLD !");  
    

    HM11* hm11 = new HM11( HM11_PIN_TX, HM11_PIN_RX);
    myled = 0;
    int counter =0;
    while(1) {
        //myled = 1;
        wait(0.5);
        usbDebug.printf("alive ");   
        wait(0.5);
        
#ifdef CRC_TEST_ENABLED
        //if(!hm11->sendGetCommand("BAUD"))
        //    usbDebug.printf("FAIL"n);  
        const uint8_t crcBuf[]= {0x42, 0x4C, 0x45};
        uint8_t crcBufLen= sizeof(crcBuf)/sizeof(crcBuf[0]);
        usbDebug.printf("CRC_VAL:  %d\r\n",(int)computeCRC16(crcBuf, crcBufLen));
#else
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
