#include "mbed.h"
#include "hm11.h"

#define HM11_PIN_TX PTE22 //PTC4   // FRDM-KL25Z UART2 //PTE22  //YELLOW wire 
#define HM11_PIN_RX PTE23//PTC3   //                  //PTE23  //ORANGE wire

DigitalOut myled(LED1);




int main() {
    Serial usbDebug(USBTX, USBRX);
    usbDebug.printf("HELLO WORLD !");  
    HM11* hm11 = new HM11( HM11_PIN_TX, HM11_PIN_RX);
    myled = 0;
    while(1) {
        //myled = 1;
        wait(0.5);
        usbDebug.printf("alive ");   
        wait(0.5);
        //if(!hm11->sendGetCommand("BAUD"))
        //    usbDebug.printf("FAIL");  
        hm11->sendDataToDevice("AT+BAUD?");
        wait(0.2);
         while(hm11->isRxDataAvailable())                    
                usbDebug.printf("data:  %c\r\n",hm11->getDataFromRx());
            
  
        
    }
}
