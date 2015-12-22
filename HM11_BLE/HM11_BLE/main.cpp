


#include "mbed.h"
#include "hm11.h"

#define HM11_PIN_TX PTE22 //FRDM-KL25Z UART2 
#define HM11_PIN_RX PTE23  
static void testMethod0(void){
    Serial usbDebug(USBTX, USBRX);
    usbDebug.printf("HELLO All!"); 
    
    
    HM11* hm11 = new HM11( HM11_PIN_TX, HM11_PIN_RX);
    
    while(1) {
        char addrBuf[40];
        wait(0.8);
        usbDebug.printf("\r\n--------queryModuleAddress----------\r\n");
        if(hm11->queryModuleAddress(addrBuf))
            usbDebug.printf("MAC Addr: %s\r\n", addrBuf);
        else
            usbDebug.printf("queryModuleAddress FAILED\r\n");             
    }
}

static void testMethod1(void){
    Serial usbDebug(USBTX, USBRX);
    usbDebug.printf("HELLO WORLD !");  
    

    HM11* hm11 = new HM11( HM11_PIN_TX, HM11_PIN_RX);
    while(1) {
        static int counterLen=0;
        usbDebug.printf("alive "); 
        hm11->flushBuffers();  
        hm11->sendDataToDevice("AT+ADDR?");
        //wait(0.1);      
        //while(hm11->isRxDataAvailable()&&counterLen++<8)                    
         //      usbDebug.printf("data:  %c\r\n",hm11->getDataFromRx());
        if(hm11->waitForData(1000)==false)
        {
            usbDebug.printf("!hm11->waitForData(500) ct=%d\r\n",counterLen);
            counterLen++;
        }
        else
        {
            uint8_t headerBuf[8];
            hm11->copyAvailableDataToBuf((uint8_t*)headerBuf,8);
            usbDebug.printf("data: %c%c%c%c%c%c ct=%d\r\n",headerBuf[0],headerBuf[1],headerBuf[2],headerBuf[3],headerBuf[4],headerBuf[5],counterLen);
            counterLen++;
        }

    }
}

static void testMethod2(void){
     
    Serial usbDebug(USBTX, USBRX);
    usbDebug.printf("HELLO WORLD !");  
    

    HM11* hm11 = new HM11( HM11_PIN_TX, HM11_PIN_RX);
    while(1) 
    { 
        wait(0.3); 
        hm11->flushBuffers();
        hm11->sendDataToDevice("AT+ADDR?");
        char headerBuf[9];//for OK+ADDR:
        char addrBuf[30];
        if(hm11->waitForData(1000)==false)
            usbDebug.printf("!hm11->waitForData(500)\r\n");
        else
        {
            hm11->copyAvailableDataToBuf((uint8_t*)headerBuf,8);
            headerBuf[8]='\0';
            usbDebug.printf("data1:%s ",headerBuf);
            if(strncmp(headerBuf, "OK+ADDR:",8) == 0)
            {
                if(hm11->copyAvailableDataToBuf((uint8_t*)addrBuf,12))
                {
                    addrBuf[12]='\0';
                    usbDebug.printf("MAC: %s ",addrBuf);
                }
            }
            
        }                 
    } 
}


int main() {

    testMethod0();
    //testMethod1();
    //testMethod2();
    return 0;
}
