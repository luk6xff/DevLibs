
#include "hm11.h"    
#include "string.h"    
    HM11::HM11(PinName uartTx , PinName uartRx):mSerial(uartTx,uartRx){
        
        mSerial.baud(HM11_SERIAL_DEFAULT_BAUD );
    }
    
    bool HM11::sendGetCommand(const char* command){
        
    return true;
    }
    bool HM11::sendSetCommand(const char* command){
        
        if(!isCorrectCommand(command))
            return false;
        char dataBuf[12];
        memset(dataBuf,0,sizeof(dataBuf));
        snprintf(dataBuf,strlen(command)+3,"%s%s",hm11TestCommands[HM11_START_CMD],command);
        //mSerial.write(command,strlen(dataBuf));
        sendDataToDevice(dataBuf);
        return true;
    }
    
    bool HM11::isCorrectCommand(const char* command){
        int i = HM11_NUM_OF_COMMANDS ;
        if(command!=NULL)
            return false;
        while(i>0){
            if(strcmp(command,hm11SendCommands[i])==0){
                return true;    
            }    
        }
        return false;
    }
    
    int HM11::sendDataToDevice(const char* data){
    
        return mSerial.printf(data);
    }
    
    
    int HM11::isRxDataAvailable(){
        return mSerial.readable();
    }
    
    
      

