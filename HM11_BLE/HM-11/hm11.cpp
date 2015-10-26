#include "hm11.h"    
#include "string.h" 
   
    HM11::HM11(PinName uartTx , PinName uartRx):mSerial(uartTx,uartRx){
        
        mSerial.baud(HM11_SERIAL_DEFAULT_BAUD );
    }
    

    
    bool HM11::sendSetCommand(const char* command, const int param){
        
        if(!isCorrectCommand(command,HM11_SEND_COMMAND))
            return false;
        char dataBuf[12];
        memset(dataBuf,0,sizeof(dataBuf));
        snprintf(dataBuf,strlen(command)+4,"%s%s%d",hm11TestCommands[HM11_START_CMD],command,param); //TODO strlen +4 ? not all params are 1 char long
        sendDataToDevice(dataBuf);
        return true;
    }
    
    
    bool HM11::sendGetCommand(const char* command){
        
        if(!isCorrectCommand(command,HM11_SEND_COMMAND))
            return false;
        char dataBuf[12];
        memset(dataBuf,0,sizeof(dataBuf));
        snprintf(dataBuf,strlen(hm11TestCommands[HM11_START_CMD])+strlen(command)+strlen(hm11TestCommands[HM11_QUERY_SIGN]+2),"%s%s%s\0",hm11TestCommands[HM11_START_CMD],command,hm11TestCommands[HM11_QUERY_SIGN]);
        sendDataToDevice(dataBuf);
        return true;
    }
    
    //@param : cmdType - 0 test cmd 
    bool HM11::isCorrectCommand(const char* command, HM11CommandType cmdType){
        int i = 0 ;
        const char**cmdPtr=NULL;
        if(cmdType>=HM11_NUM_OF_COMMAND_TYPE){
            
            return false;
        }
        if(command==NULL)
            return false;
        switch(cmdType){
            case HM11_TEST_COMMAND:
                i = HM11_NUM_OF_TEST_COMMANDS ;
                cmdPtr=hm11TestCommands;
            break;    
            case HM11_SEND_COMMAND:
                i = HM11_NUM_OF_COMMANDS;
                cmdPtr=hm11SendCommands;
            break;   
        }      
        while(i>0){
            if(strcmp(command,cmdPtr[i])==0){
                return true;    
            }
            i--;   
        }
        return false;
    }
 //comands   
    
    bool HM11::testCommand(){
        return false;
    }
    
    
    
    bool HM11:: waitForData(int timeoutMs){
        int endtime;  
        Timer timer;
        timer.start() ;
        endtime= timer.read_ms()+timeoutMs;
        while((timer.read_ms())<endtime){
            if(isRxDataAvailable())
                return true;
        }
        return false;
    }
    
    
    int HM11::sendDataToDevice(const char* data){
    
        return mSerial.printf(data);
    }
    
    
    int HM11::isRxDataAvailable(){
        return mSerial.readable();
    }
    
    
     

