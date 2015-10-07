
#include "btm222.h"
#include <string.h>



BTM222::BTM222(PinName tx, PinName rx) : bufSerial(tx,rx)
{ 
bufSerial.baud(BLUETOOTH_SERIAL_DEFAULT_BAUD );
}

void BTM222::getInfo(){
    
    write("ATI0\r");
    }


int BTM222::write(const char* data){
    
      return bufSerial.printf(data);
  }
    
    
 int BTM222::isRxDataAvailable(){
      
      return bufSerial.readable();
  }
      

void BTM222::sendDataToBTM(char tag,float valueToSend){
    
  char buffer[16];
  const char errorBuffer[]={"BUF ERROR"};
  uint8_t lengthOfString;
  buffer[0]=tag;
  if(valueToSend==-999) {//TODO add as a function parameter
        lengthOfString=strlen(errorBuffer);
        memcpy(&buffer[1],errorBuffer,lengthOfString);
  }
  else{
  lengthOfString=snprintf(&buffer[1],8,"%4.2f",valueToSend);
  }
  buffer[lengthOfString+1]='x';
  buffer[lengthOfString+2]='\0';
  write(buffer);
}