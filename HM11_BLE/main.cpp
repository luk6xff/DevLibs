#include "mbed.h"
#include "hm11.h"

#define BTM222_PIN_TX PTC4   //UART2
#define BTM222_PIN_RX PTC3   // 

DigitalOut myled(LED1);




int main() {
    Serial usbDebug(USBTX, USBRX);
    HM11* hm11 = new HM11( BTM222_PIN_TX, BTM222_PIN_RX);
    while(1) {
        myled = 1;
        wait(0.2);
        myled = 0;
        wait(0.2);
        hm11->sendSetCommand("BAUD");
        wait(1.2);
         while(hm11->isRxDataAvailable())                    
                usbDebug.printf("data:  %c\r\n",hm11->getDataFromRx());   
            
    wait(1.2);
        
    }
}
