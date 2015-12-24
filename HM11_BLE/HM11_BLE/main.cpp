


#include "mbed.h"
#include "hm11.h"

#define HM11_PIN_TX PTE22 //FRDM-KL25Z UART2 
#define HM11_PIN_RX PTE23  
static void testMethod0(void){
    Serial usbDebug(USBTX, USBRX);
    usbDebug.printf("HELLO All!"); 
    
    
    HM11* hm11 = new HM11( HM11_PIN_TX, HM11_PIN_RX);
    
    while(1) {
        char respBuf[40];
        wait(0.8);
        usbDebug.printf("\r\n--------queryModuleAddress----------\r\n");
        if(hm11->queryModuleAddress(respBuf))
            usbDebug.printf("MAC Addr: %s\r\n", respBuf);
        else
            usbDebug.printf("queryModuleAddress FAILED\r\n");  
            
            
            
        wait(0.8);
        usbDebug.printf("\r\n--------setAdvertisingInterval----------\r\n");
        if(hm11->setAdvertisingInterval(_417_5ms))
            usbDebug.printf("ADV_Interval_set: SET\r\n");
        else
            usbDebug.printf("ADV_Interval_set FAILED\r\n");   
        
        usbDebug.printf("\r\n--------queryAdvertisingInterval----------\r\n");
        if(hm11->queryAdvertisingInterval()==_417_5ms)
            usbDebug.printf("ADV_Interval: %d\r\n", hm11->queryAdvertisingInterval());
        else
            usbDebug.printf("queryAdvertisingInterval FAILED %d\r\n",hm11->queryAdvertisingInterval());            
    }
}

static void testMethod1(void){
    Serial usbDebug(USBTX, USBRX);
    usbDebug.printf("HELLO WORLD !");  
    

    HM11* hm11 = new HM11( HM11_PIN_TX, HM11_PIN_RX);
    while(1) {
        static int counterLen=0;
        usbDebug.printf("alive "); 
#if 0        hm11->flushBuffers();  
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
 #endif       
        hm11->flushBuffers();
        char buf[9]={"AT+ADVI"};        
        hm11->hexToString((uint32_t)_417_5ms, &buf[7],1);
        hm11->sendDataToDevice(buf);
        if(!hm11->waitForData(100))
            usbDebug.printf("FAILED: %c%c%c%c%c%c\r\n",buf[0],buf[1],buf[2],buf[3],buf[4],buf[5]);
        else
        {
            hm11->copyAvailableDataToBuf((uint8_t*)buf,sizeof(buf));
            if(strncmp(buf,"OK+Set:",7) == 0){
                if(hm11->strToHex(&buf[7],1)<_INVALID_ADV_INTERVAL){
                     usbDebug.printf("data: %c%c%c%c%c%c\r\n",buf[0],buf[1],buf[2],buf[3],buf[4],buf[8]);
                }
            }
            usbDebug.printf("datafailed: %c%c%c%c%c%c%c%c%c\r\n",buf[0],buf[1],buf[2],buf[3],buf[4],buf[5],buf[6],buf[7],buf[8]);         
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
        AdvertisingInterval_t retVal=_760ms;
        char respBuf[8];        
        hm11->sendDataToDevice("AT+ADVI?");
        if(!hm11->waitForData(100))
        {
            //nothing
        }
        else
        {
            hm11->copyAvailableDataToBuf((uint8_t*)respBuf,sizeof(respBuf));
            if(strncmp(respBuf,"OK+Get:",sizeof(respBuf-1)) == 0){
            usbDebug.printf("dataSSS %d\r\n",hm11->strToHex(&respBuf[sizeof(respBuf)-1],1));
            }
                                     
        }
         usbDebug.printf("data: %c%c%c%c%c%c%c%c %d\r\n",respBuf[0],respBuf[1],respBuf[2],respBuf[3],respBuf[4],respBuf[5],respBuf[6],respBuf[7],retVal);            
    } 
}


int main() {

    testMethod0();
    //testMethod1();
    //testMethod2();
    return 0;
}
