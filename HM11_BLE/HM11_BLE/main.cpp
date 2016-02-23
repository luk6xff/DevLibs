


#include "mbed.h"
#include "hm11.h"

//#define HM11_PIN_TX PTE22 //FRDM-KL25Z UART2 
//#define HM11_PIN_RX PTE23  


#define HM11_PIN_TX PC_10 //FOR STM32L053                      // FRDM-KL25Z UART2:PTE22  //YELLOW wire 
#define HM11_PIN_RX PC_11
/*
static void testMethodDefault(void){
    Serial usbDebug(USBTX, USBRX);
    usbDebug.printf("HELLO WORLD !");  
    

    HM11* hm11 = new HM11( HM11_PIN_TX, HM11_PIN_RX);
    int counter =0;
    while(1) {
        wait(0.5);
        usbDebug.printf("alive ");   
        wait(0.5);
        char buf[2];
        snprintf(buf,2,"%d",counter++);
        if(counter>9)
            counter=0;
        hm11->sendDataToDevice(buf);
        wait(0.2);
        
         while(hm11->isRxDataAvailable())                    
               usbDebug.printf("data:  %c\r\n",hm11->getDataFromRx());
    }
}
*/
static void testMethod0(void){
    Serial usbDebug(USBTX, USBRX);
    usbDebug.printf("HELLO All!"); 
    
    
    HM11* hm11 = new HM11( HM11_PIN_TX, HM11_PIN_RX);
    
    while(1) {
        char respBuf[40];
              
        //-------------get--------------
        wait(0.3);
        usbDebug.printf("---queryModuleAddress---\r\n");
        if(hm11->queryModuleAddress(respBuf))
            usbDebug.printf("GET_ok: %s\r\n", respBuf);
        else
            usbDebug.printf("GET_error\r\n");  
            
            
        //-------------set--------------    
        wait(0.3);
        usbDebug.printf("---setAdvertisingInterval---\r\n");
        if(hm11->setAdvertisingInterval(_417_5ms))
            usbDebug.printf("SET_ok\r\n");
        else
            usbDebug.printf("SET_error\r\n");
        //-------------get-------------- 
        wait(0.3); 
        if(hm11->queryAdvertisingInterval()==_417_5ms)
            usbDebug.printf("GET_ok: %d\r\n", hm11->queryAdvertisingInterval());
        else
            usbDebug.printf("GET_error %d\r\n",hm11->queryAdvertisingInterval());            
    }
}


static void testMethod2(void){
     
    Serial usbDebug(USBTX, USBRX);
    usbDebug.printf("HELLO WORLD !");  
    

    HM11* hm11 = new HM11( HM11_PIN_TX, HM11_PIN_RX);
    
    int retVal =0;
    while(1) 
    { 
  #if 0      
        //-------------set--------------
        wait(0.3);
        usbDebug.printf("---setWhitelistMacAddress---\r\n");
        if(hm11->setWhitelistMacAddress (1, "001122334455"))
            usbDebug.printf("SET_ok: \r\n");
        else
           usbDebug.printf("SET_error: \r\n");        
        //-------------get--------------
        wait(0.3);
        char addrBuf[15];   
        if(hm11->queryWhitelistMacAddress(1, addrBuf, 15))
        {
            addrBuf[14]='\0';
            usbDebug.printf("GET_ok: %s\r\n",addrBuf);
        }
        else
           usbDebug.printf("GET_error: \r\n"); 
           
           
           
           
   
        //-------------set--------------
        wait(0.3);
        usbDebug.printf("---setBatteryMonitorSwitch---\r\n");
        if(hm11->setBatteryMonitorSwitch(1))
            usbDebug.printf("SET_ok: \r\n");
        else
           usbDebug.printf("SET_error: \r\n");     
        //-------------get--------------
        wait(0.3);
        retVal= hm11->queryBatteryMonitorSwitch() ;
         
        if(retVal!=0xFF)
        {
            usbDebug.printf("GET_ok: %d\r\n",retVal);
        }
        else
           usbDebug.printf("GET_error: \r\n"); 
        
       
       
       
        //-------------get--------------
        wait(0.3);
        usbDebug.printf("---queryBatteryInformation---\r\n");
        retVal= hm11->queryBatteryInformation() ;
         
        if(retVal!=0xFF)
        {
            usbDebug.printf("GET_ok: %d\r\n",retVal);
        }
        else
           usbDebug.printf("GET_error: \r\n"); 
           
           
           
           
        //-------------set--------------
        wait(0.3);
        usbDebug.printf("---setBitFormat---\r\n");
        retVal= hm11->setBitFormat(1) ;
         
        if(retVal==true)
        {
            usbDebug.printf("SET_ok: %d\r\n",retVal);
        }
        else
           usbDebug.printf("SET_error: \r\n");    
        //-------------get--------------
        wait(0.3);
        retVal= hm11->queryBitFormat() ;        
        if(retVal!=0xFF)
        {
            usbDebug.printf("GET_ok: %d\r\n",retVal);
        }
        else
           usbDebug.printf("GET_error: \r\n"); 
           
           
           
           
           
           
           
        //-------------set--------------
        wait(0.3);
        usbDebug.printf("---setBaudRate---\r\n");
        retVal= hm11->setBaudRate(_9600) ;       
        if(retVal==true)
        {
            usbDebug.printf("SET_ok: %d\r\n",retVal);
        }
        else
           usbDebug.printf("SET_error: \r\n");    
        //-------------get--------------
        wait(0.3);
        retVal= hm11->queryBaudRate() ;        
        if(retVal!=_INVALID_BAUDRATE)
        {
            usbDebug.printf("GET_ok: %d\r\n",retVal);
        }
        else
           usbDebug.printf("GET_error: \r\n");
           
           
           
           
           
        //-------------set--------------
        wait(0.3);
        usbDebug.printf("---setCharacteristic---\r\n");
        retVal= hm11->setCharacteristic(0xEEFF) ;
         
        if(retVal==true)
        {
            usbDebug.printf("SET_ok: %d\r\n",retVal);
        }
        else
           usbDebug.printf("SET_error \r\n");    
        //-------------get--------------
        wait(0.3);
        retVal= hm11->queryCharacteristic() ;        
        if(retVal!=0xFFFF)
        {
            usbDebug.printf("GET_ok: 0x%x\r\n",retVal);
        }
        else
           usbDebug.printf("GET_error: \r\n");
           
           
           
           
           
           
           
           
        //-------------set--------------
        wait(0.3);
        usbDebug.printf("---connectToLastDevice---\r\n");
        retVal= hm11->connectToLastDevice() ;
         
        if(retVal!=_INVALID_CONECTIONSTATUS)
        {
            usbDebug.printf("SET_ok: %d\r\n",retVal);
        }
        else
           usbDebug.printf("SET_error \r\n");    
        //-------------set--------------
        wait(0.3);
        usbDebug.printf("---connectToAnAddress---\r\n");
        retVal= hm11->connectToAnAddress("001122334455") ;        
        if(retVal!=_INVALID_CONECTIONSTATUS)
        {
            usbDebug.printf("SET_ok: 0x%x\r\n",retVal);
        }
        else
           usbDebug.printf("SET_error: \r\n");   
           
 

        //-------------get--------------
        wait(0.3);
        usbDebug.printf("---queryInputOutputState---\r\n");
        retVal= hm11->queryInputOutputState() ;        
        if(retVal!=0xFF)
        {
            usbDebug.printf("GET_ok: 0x%x\r\n",retVal);
        }
        else
           usbDebug.printf("GET_error: 0x%x\r\n",retVal);
      
 #endif       
        
        
        
        //-------------set--------------
        wait(0.3);
        usbDebug.printf("---setPioCollectionRate---\r\n");
        retVal= hm11->setPioCollectionRate (10) ;
         
        if(retVal)
        {
            usbDebug.printf("SET_ok: %d\r\n",retVal);
        }
        else
           usbDebug.printf("SET_error \r\n");    
        //-------------get--------------
        wait(0.3);
        retVal= hm11->queryPioCollectionRate() ;        
        if(retVal!=0xFF)
        {
            usbDebug.printf("GET_ok: 0x%x\r\n",retVal);
        }
        else
           usbDebug.printf("GET_error: \r\n");     
    
    
    
    
    
    
    
    
    
    
    
    
      
    } 
}


int main() {
    //testMethodDefault();
    //testMethod0();
    testMethod2();
    return 0;
}





/*
static void testGetVersion()
{

    char version[20];
    if(hm11-> querySoftwareVersion(version, 20))
    {
         version[19]='\0';
         usbDebug.printf("VERSION: %s\r\n",version);
    }
    else
        usbDebug.printf("VERSION failed\r\n");  
}
*/