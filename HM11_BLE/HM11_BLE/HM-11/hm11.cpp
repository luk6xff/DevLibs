#include "hm11.h"    
#include "string.h" 
   
   
   #define WAIT_FOR_DATA_TIMEOUT_MS 1000
   #define DEFAULT_WAIT_FOR_DATA_TIMEOUT_MS 50  //required to get correct data
   
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

    
    bool HM11:: waitForData(int timeoutMs){
        int endTime;
        int startTime; // sometimes not needed 50ms
        Timer timer;
        timer.start() ;   
        startTime=timer.read_ms();
        endTime= startTime+timeoutMs;
        while((timer.read_ms())<endTime){
            if(isRxDataAvailable()&&(timer.read_ms()-startTime)>DEFAULT_WAIT_FOR_DATA_TIMEOUT_MS)
                return true;
        }
        return false;
    }
    
    
    int HM11::sendDataToDevice(const char* data)
    {
    
        return mSerial.printf(data);
    }
    
    int HM11::sendDataToDevice(const uint8_t * byteData,uint8_t dataLength)
    {
    
        return mSerial.write(byteData,dataLength);
    }
      
    int HM11::isRxDataAvailable()
    {
        return mSerial.readable();
    }
    
    int HM11::copyAvailableDataToBuf(uint8_t *buf, uint8_t bufLength)
    {
        int lenCounter =0;
        if(buf==NULL||bufLength<1)
            return -1;
        while(isRxDataAvailable()&&lenCounter<bufLength){                    
            buf[lenCounter++]=getDataFromRx();
        }
        return lenCounter;             
    }
    
    bool HM11:: hexToString(uint32_t hex, char*str,uint8_t nrOfNibblesInHex)
    {   
        if(nrOfNibblesInHex>8||nrOfNibblesInHex==0||str==NULL)
            return false;
        for(int i=0;i<nrOfNibblesInHex;i++){
            int temp =(hex>>(i*4))&0x0F;
            if(temp>=0&&temp<=9)
            {
                str[nrOfNibblesInHex-i-1]=temp+'0';
            }
            else
            {
                str[nrOfNibblesInHex-i-1]=temp+'A'-10;
            }
        }
        return true;
    }
    
    
      //returns hex in  reverse direction
    uint32_t HM11::strToHex(char*const str,uint8_t len) //len - nr of nibbles in str
    {
        uint32_t ret=0;
        uint8_t temp;
        if(len<1||len>8||str==NULL)
            return 0xFFFFFFFF;
        
        for(int i=0;i<len&&str[i]!='\0';i++)
        {
            if(str[i]>='0'&&str[i]<='9')
            {
                temp=str[i]-'0';
                ret|=temp<<(i*4);
            }
            else if(str[i]>='A'&&str[i]<='F')
            {
                temp=str[i]-'A'+10;
                ret|=temp<<(i*4);
            }
            else if(str[i]>='a'&&str[i]<='f')
            {
                temp=str[i]-'a'+10;
                ret|=temp<<(i*4);
            }
            else 
                return 0xFFFFFFFF;  //-1 :-)
        }
        return ret;
    }
    
// public methods
    bool HM11::queryModuleAddress(char* addrBuf)
    {
        flushBuffers();
        sendDataToDevice("AT+ADDR?");
        char headerBuf[8];//for OK+ADDR:
        if(!waitForData(WAIT_FOR_DATA_TIMEOUT_MS))
            return false;
        else{
            copyAvailableDataToBuf((uint8_t*)headerBuf,sizeof(headerBuf));
            if(strncmp(headerBuf,"OK+ADDR:",sizeof(headerBuf)) == 0){
                if(copyAvailableDataToBuf((uint8_t*)addrBuf,12)){
                    addrBuf[12]='\0';
                    return true;
                }
            }
        return false;         
        }       
    }
    

    bool HM11::setAdvertisingInterval(AdvertisingInterval_t advInt)
    {
        flushBuffers();
        char buf[9]={"AT+ADVI"};        
        hexToString((uint32_t)advInt, &buf[7],1);
        buf[8]='\0';
        sendDataToDevice(buf);
        if(!waitForData(WAIT_FOR_DATA_TIMEOUT_MS))
            return false;
        else
        {
            copyAvailableDataToBuf((uint8_t*)buf,sizeof(buf));
            if(strncmp(buf,"OK+Set:",7) == 0){
                if(strToHex(&buf[7],1)<_INVALID_ADV_INTERVAL){
                    return true;
                }
            }
            return false;         
        }           
    }
    
    
    AdvertisingInterval_t HM11::queryAdvertisingInterval(void)
    {
        flushBuffers();
        AdvertisingInterval_t retVal=_INVALID_ADV_INTERVAL;
        char respBuf[8];        
        sendDataToDevice("AT+ADVI?");
        if(!waitForData(WAIT_FOR_DATA_TIMEOUT_MS))
        {
            //nothing
        }
        else
        {
            copyAvailableDataToBuf((uint8_t*)respBuf,sizeof(respBuf));
            if(strncmp(respBuf,"OK+Get:",sizeof(respBuf)-1) == 0){
                retVal=(AdvertisingInterval_t)strToHex(&respBuf[sizeof(respBuf)-1],1);
            }
                                     
        }
        return retVal;  
    }
 
     
    bool HM11::setAdvertisingType(AdvertisingType_t advInt)
    {
        flushBuffers();
        char buf[9]={"AT+ADTY"};        
        hexToString((uint32_t)advInt, &buf[7],1);
        buf[8]='\0';
        sendDataToDevice(buf);
        if(!waitForData(WAIT_FOR_DATA_TIMEOUT_MS))
            return false;
        else
        {
            copyAvailableDataToBuf((uint8_t*)buf,sizeof(buf));
            if(strncmp(buf,"OK+Set:",7) == 0){
                if(strToHex(&buf[7],1)<_INVALID_ADV_TYPE){
                    return true;
                }
            }
            return false;         
        }              
    }
    
    AdvertisingType_t HM11::queryAdvertisingType(void)
    {
        flushBuffers();
        AdvertisingType_t retVal=_INVALID_ADV_TYPE;
        char respBuf[8];        
        sendDataToDevice("AT+ADTY?");
        if(!waitForData(WAIT_FOR_DATA_TIMEOUT_MS))
        {
            //nothing
        }
        else
        {
            copyAvailableDataToBuf((uint8_t*)respBuf,sizeof(respBuf));
            if(strncmp(respBuf,"OK+Get:",sizeof(respBuf)-1) == 0){
                retVal=(AdvertisingType_t)strToHex(&respBuf[sizeof(respBuf)-1],1);
            }                                     
        }
        return retVal;  
    }
     
     
 
    bool HM11::setAncsSwitch(uint8_t enable)
    {
        flushBuffers();
        char buf[9]={"AT+ANCS"};        
        hexToString(enable, &buf[7],1);
        buf[8]='\0';
        sendDataToDevice(buf);
        if(!waitForData(WAIT_FOR_DATA_TIMEOUT_MS))
            return false;
        else
        {
            copyAvailableDataToBuf((uint8_t*)buf,sizeof(buf));
            if(strncmp(buf,"OK+Set:",7) == 0){
                if(strToHex(&buf[7],1)<0x2){
                    return true;
                }
            }
            return false;         
        }              
    }
    
    

    uint8_t HM11::queryAncsSwitch(void)
    {
        flushBuffers();
        uint8_t retVal=0xFF;
        char respBuf[8];        
        sendDataToDevice("AT+ANCS?");
        if(!waitForData(WAIT_FOR_DATA_TIMEOUT_MS))
        {
            retVal=0xFF;
        }
        else
        {
            copyAvailableDataToBuf((uint8_t*)respBuf,sizeof(respBuf));
            if(strncmp(respBuf,"OK+Get:",sizeof(respBuf)-1) == 0){
                retVal=strToHex(&respBuf[sizeof(respBuf)-1],1);
            }                                     
        }
        return retVal;  
    }
    
  
    bool HM11::setWhitelistSwitch(uint8_t enable)
    {
        flushBuffers();
        char buf[9]={"AT+ALLO"};        
        hexToString(enable, &buf[7],1);
        buf[8]='\0';
        sendDataToDevice(buf);
        if(!waitForData(WAIT_FOR_DATA_TIMEOUT_MS))
            return false;
        else
        {
            copyAvailableDataToBuf((uint8_t*)buf,sizeof(buf));
            if(strncmp(buf,"OK+Set:",7) == 0){
                if(strToHex(&buf[7],1)<0x2){
                    return true;
                }
            }
            return false;         
        }              
    }
    

    uint8_t HM11::queryWhitelistSwitch(void)
    {
        flushBuffers();
        uint8_t retVal=0xFF;
        char respBuf[8];        
        sendDataToDevice("AT+ALLO?");
        if(!waitForData(WAIT_FOR_DATA_TIMEOUT_MS))
        {
            retVal=0xFF;
        }
        else
        {
            copyAvailableDataToBuf((uint8_t*)respBuf,sizeof(respBuf));
            if(strncmp(respBuf,"OK+Get:",sizeof(respBuf)-1) == 0){
                retVal=strToHex(&respBuf[sizeof(respBuf)-1],1);
            }                                     
        }
        return retVal;  
    }
    

    bool HM11::setWhitelistMacAddress (uint8_t nrOfMacAddrLinkedToModule, const char* const macAddress)
    {
        if(nrOfMacAddrLinkedToModule>3 ||macAddress ==NULL)
        {
            return false;
        }
        char buf[19]={"AT+AD"}; 
        flushBuffers();
        hexToString(nrOfMacAddrLinkedToModule, &buf[5],1);
        memcpy(&buf[6],macAddress,12);
        buf[18]='\0';
        sendDataToDevice(buf);
        if(!waitForData(WAIT_FOR_DATA_TIMEOUT_MS))
            return false;
        else
        {
            copyAvailableDataToBuf((uint8_t*)buf,sizeof(buf));
            if(strncmp(buf,"OK+AD",5) == 0){
                return true;
            }
            return false;         
        } 
    }
    
    bool HM11::queryWhitelistMacAddress(uint8_t nrOfMacAddrLinkedToModule, char* const macAddrBuf, uint8_t macAddrBufLen)
    {
        
        if(macAddrBuf==NULL||nrOfMacAddrLinkedToModule>3||macAddrBufLen<12) //12 -len of macAddr eg. 001122334455 
            return false;
        flushBuffers();
        char buf[20]={"AT+ADx??"};
        buf[8]='\0';  
        hexToString(nrOfMacAddrLinkedToModule, &buf[5],1);
        sendDataToDevice(buf); 
        if(!waitForData(WAIT_FOR_DATA_TIMEOUT_MS))
            return false;
        else
        {
            copyAvailableDataToBuf((uint8_t*)buf,sizeof(buf));
            if(strncmp(buf,"OK+AD",5) == 0){
                memcpy(macAddrBuf,&buf[8],12);
                return true;
            }
            return false;         
        } 
    
    }


    bool HM11::setBatteryMonitorSwitch(uint8_t battSwitchEnable)
    {
        flushBuffers();
        char buf[9]={"AT+BATCx"};
        buf[8]='\0';        
        hexToString(battSwitchEnable, &buf[7],1);
        sendDataToDevice(buf);
        if(!waitForData(WAIT_FOR_DATA_TIMEOUT_MS))
            return false;
        else
        {
            copyAvailableDataToBuf((uint8_t*)buf,sizeof(buf));
            if(strncmp(buf,"OK+Set:",7) == 0){
                if(strToHex(&buf[7],1)<0x2){
                    return true;
                }
            }
            return false;         
        }              
    }
    
 
    uint8_t HM11::queryBatteryMonitorSwitch(void)
    {
        flushBuffers();
        uint8_t retVal=0xFF;
        char respBuf[8];        
        sendDataToDevice("AT+BATC?");
        if(!waitForData(WAIT_FOR_DATA_TIMEOUT_MS))
        {
            retVal=0xFF;
        }
        else
        {
            copyAvailableDataToBuf((uint8_t*)respBuf,sizeof(respBuf));
            if(strncmp(respBuf,"OK+Get:",sizeof(respBuf)-1) == 0){
                retVal=strToHex(&respBuf[sizeof(respBuf)-1],1);
            }                                     
        }
        return retVal;  
    }
    
    

    uint8_t HM11::queryBatteryInformation(void)
    {
        flushBuffers();
        uint8_t retVal=0xFF;
        char respBuf[12];        
        sendDataToDevice("AT+BATT?");
        if(!waitForData(WAIT_FOR_DATA_TIMEOUT_MS))
        {
            retVal=0xFF;
        }
        else
        {
            copyAvailableDataToBuf((uint8_t*)respBuf,sizeof(respBuf));
            if(strncmp(respBuf,"OK+BATT:",7) == 0){
                retVal=strToHex(&respBuf[7],3);
            }                                     
        }
        return retVal;      
        
    }

    

    bool HM11::setIBeaconIntoServiceMode(void)   //NOT USED ANYMORE
    {
        flushBuffers();        
        sendDataToDevice("AT+BUSHU");
        char respBuf[10];
        if(!waitForData(WAIT_FOR_DATA_TIMEOUT_MS))
        {
            return false;
        }
        else
        {
            copyAvailableDataToBuf((uint8_t*)respBuf,sizeof(respBuf));
            if(strncmp(respBuf,"OK+BUSHU",7) == 0)
            {
                return true;
            }                                     
        }
        return false;  
        
    }
    
    
  
    bool HM11::setBitFormat(uint8_t bit7Format)
    {
        if(bit7Format>1)
            return false;
        flushBuffers();
        char buf[9]={"AT+BIT7x"};
        buf[8]='\0';        
        hexToString(bit7Format, &buf[7],1);
        sendDataToDevice(buf);
        if(!waitForData(WAIT_FOR_DATA_TIMEOUT_MS))
            return false;
        else
        {
            copyAvailableDataToBuf((uint8_t*)buf,sizeof(buf));
            if(strncmp(buf,"OK+Set:",7) == 0){
                if(strToHex(&buf[7],1)<0x2){
                    return true;
                }
            }
            return false;         
        }              
    }
    
 
    uint8_t HM11::queryBitFormat(void)
    {
        flushBuffers();
        uint8_t retVal=0xFF;
        char respBuf[8];        
        sendDataToDevice("AT+BIT7?");
        if(!waitForData(WAIT_FOR_DATA_TIMEOUT_MS))
        {
            retVal=0xFF;
        }
        else
        {
            copyAvailableDataToBuf((uint8_t*)respBuf,sizeof(respBuf));
            if(strncmp(respBuf,"OK+Get:",7) == 0){
                retVal=strToHex(&respBuf[7],1);
            }                                     
        }
        return retVal; 
    }
    
  
 
    bool HM11::setBaudRate(BaudRate_t baud)
    {
        if(baud >=_INVALID_BAUDRATE)
            return false;
        flushBuffers();
        char buf[9]={"AT+BAUDx"};        
        hexToString((uint32_t)baud, &buf[7],1);
        buf[8]='\0';
        sendDataToDevice(buf);
        if(!waitForData(WAIT_FOR_DATA_TIMEOUT_MS))
            return false;
        else
        {
            copyAvailableDataToBuf((uint8_t*)buf,sizeof(buf));
            if(strncmp(buf,"OK+Set:",7) == 0){
                if(strToHex(&buf[7],1)<_INVALID_BAUDRATE){
                    return true;
                }
            }
            return false;         
        }               
    }
    
    
    BaudRate_t HM11::queryBaudRate(void)
    {
        flushBuffers();
        BaudRate_t retVal=_INVALID_BAUDRATE;
        char respBuf[8];        
        sendDataToDevice("AT+BAUD?");
        if(!waitForData(WAIT_FOR_DATA_TIMEOUT_MS))
        {
            //nothing
        }
        else
        {
            copyAvailableDataToBuf((uint8_t*)respBuf,sizeof(respBuf));
            if(strncmp(respBuf,"OK+Get:",7) == 0){
                retVal=(BaudRate_t)strToHex(&respBuf[7],1);
            }
                                     
        }
        return retVal;            
    }  
    

    bool HM11::setCharacteristic(uint16_t chValue)
    {
        if(chValue <0x0001||chValue>0xFFFE)
            return false;
        flushBuffers();
        char buf[14]={"AT+CHAR0xiiii"};        
        hexToString((uint32_t)chValue, &buf[9],4);
        buf[13]='\0';
        sendDataToDevice(buf);
        if(!waitForData(WAIT_FOR_DATA_TIMEOUT_MS))
            return false;
        else
        {
            copyAvailableDataToBuf((uint8_t*)buf,sizeof(buf));
            if(strncmp(buf,"OK+Set:0x",9) == 0){
                return true;

            }
            return false;         
        }   
    }
    
    uint16_t HM11::queryCharacteristic(void)
    {
        flushBuffers();
        uint16_t retVal=0xFFFF;
        char respBuf[14];        
        sendDataToDevice("AT+CHAR?");
        if(!waitForData(WAIT_FOR_DATA_TIMEOUT_MS))
        {
            //nothing
        }
        else
        {
            copyAvailableDataToBuf((uint8_t*)respBuf,sizeof(respBuf));
            if(strncmp(respBuf,"OK+Get:0x",9) == 0){
                retVal=(uint16_t)strToHex(&respBuf[9],4);
            }                                   
        }
        return retVal;   
    } 
  

    ConnectionStatus_t HM11::connectToLastDevice(void)
    {
        
        ConnectionStatus_t retVal=  _INVALID_CONECTIONSTATUS;
        flushBuffers();
        char buf[9]={"AT+CONNL"};        
        buf[8]='\0';
        sendDataToDevice(buf);
        if(!waitForData(WAIT_FOR_DATA_TIMEOUT_MS*5))
        {
            //nothing
        }
        else
        {
            copyAvailableDataToBuf((uint8_t*)buf,sizeof(buf));
            if(strncmp(buf,"OK+CONN",7) == 0){
                for(int i=0; i<_INVALID_CONECTIONSTATUS ;++i){
                    if(strncmp(&buf[7],&ConnectionStatusArr[i],1) == 0)
                        retVal= ( ConnectionStatus_t)i;      
                }
            }                   
        } 
        return retVal;                  
    }
    
    ConnectionStatus_t HM11::connectToAnAddress(const char* const macAddress)
    {
        ConnectionStatus_t retVal= _INVALID_CONECTIONSTATUS;
        if(macAddress ==NULL)
        {
            return retVal;
        }       
        flushBuffers();
        char buf[19]={"AT+CON"};
        memcpy(&buf[6],macAddress,12);       
        buf[18]='\0';
        sendDataToDevice(buf);
        if(!waitForData(WAIT_FOR_DATA_TIMEOUT_MS*5))
        {
            //nothing
        }
        else
        {
            copyAvailableDataToBuf((uint8_t*)buf,sizeof(buf));
            if(strncmp(buf,"OK+CONN",7) == 0){
                for(int i=0; i<_INVALID_CONECTIONSTATUS ;++i){
                    if(strncmp(&buf[7],&ConnectionStatusArr[i],1) == 0)
                        retVal= ( ConnectionStatus_t)i;      
                }
            }                   
        } 
        return retVal;   
    } 
    
  
    uint8_t HM11::queryInputOutputState(void)
    {
        flushBuffers();
        uint8_t retVal=0xFF;
        char respBuf[12];        
        sendDataToDevice("AT+COL??");
        if(!waitForData(WAIT_FOR_DATA_TIMEOUT_MS))
        {
            //nothing
        }
        else
        {
            copyAvailableDataToBuf((uint8_t*)respBuf,sizeof(respBuf));
            if(strncmp(respBuf,"OK+Col:0x",9) == 0){
                retVal=(uint8_t)strToHex(&respBuf[9],2);
            }                                   
        }
        return retVal;   
        
    
    }
    

    bool HM11::setPioCollectionRate (uint8_t colRateSec)
    {
        if(colRateSec>90)
            return false;
        flushBuffers();
        char buf[9]={"AT+CYCxx"};        
        hexToString((uint32_t)colRateSec, &buf[6],2);
        buf[8]='\0';
        sendDataToDevice(buf);
        if(!waitForData(WAIT_FOR_DATA_TIMEOUT_MS))
            return false;
        else
        {
            copyAvailableDataToBuf((uint8_t*)buf,sizeof(buf));
            if(strncmp(buf,"OK+Set:",7) == 0){
                uint8_t secVal= (uint8_t)strToHex(&buf[7],2);
                //if(secVal==colRateSec)
                    return true;
            }      
        } 
        return false;         
    }
    
   
    uint8_t HM11::queryPioCollectionRate(void)
    {
        flushBuffers();
        uint8_t retVal=0xFF;
        char respBuf[12];        
        sendDataToDevice("AT+CYC??");
        if(!waitForData(WAIT_FOR_DATA_TIMEOUT_MS))
        {
            //nothing
        }
        else
        {
            copyAvailableDataToBuf((uint8_t*)respBuf,sizeof(respBuf));
            if(strncmp(respBuf,"OK+Get:",7) == 0){
                retVal=(uint8_t)strToHex(&respBuf[7],2);
            }                                   
        }
        return retVal;     
    }
    
    
 #if 0  
    bool HM11::startDeviceDiscoveryScan(ScanResult_t* scanRes) 
    {
        
    }    
    
  
    bool HM11::connectToDiscoveryDevice(ScanResult_t* scanRes) 
    {
        
    }    
    
       

    bool HM11::setIBeaconDeployMode(DeployMode_t depMode)
    {
        
    }    
    
   
    bool HM11::setFilterOfHmModules(FilterOfHmModules_t filter) 
    {
        
    }    
    
 
    FilterOfHmModules_t HM11::queryFilterOfHmModules(void) 
    {
        
    }    
   
  
    bool HM11::removeBondInformation(void) 
    {
        
    }    

    bool HM11::getSystemHelpInformation(char* helpInformationBuf)
    {
        
    }    
  
    bool HM11::setModuleWorkType(ModuleWorkType_t modWorkType) 
    {
        
    }    
    
   
    ModuleWorkType_t HM11::queryModuleWorkType(void)
    {
        
    }    
   
   
  
   
    bool HM11::setModuleIBeaconSwitch (uint8_t turnOnOff)
    
  
    uint8_t HM11::queryModuleIBeaconSwitch (void);
     
   
       
    bool HM11::setIBeaconUuid (uint32_t uuid0,uint32_t uuid1,uint32_t uuid2, uint32_t uuid3); 
    bool HM11::setIBeaconUuid (uint32_t* uuid); 
    
 
    uint32_t HM11::queryIBeaconUuid(uint32_t* nr_of_uuid);
     
    
    
 
    bool HM11::setIBeaconMajor(uint16_t mjrVersion); 
    
   
    uint16_t HM11::queryIBeaconMajor(void);
    
    
    
  
    bool HM11::setIBeaconMinor(uint16_t mnrVersion); 
    
  
    uint16_t HM11::queryIBeaconMinor(void);
   
   
   
  
    bool HM11::setIBeaconMeasuredPower(uint16_t measuredPwr); 
    
    
   
    uint16_t HM11::queryIBeaconMeasuredPower(void);
   
   
     
    bool HM11::setModuleWorkMode(ModuleWorkMode_t workMode); 
    
    
  
    ModuleWorkMode_t  HM11::queryModuleWorkMode(void);
   
 
 
     
    bool HM11::setModuleName(char* name, uint8_t nameLength); 
    
   
    bool HM11::queryModuleName(char *name);
 
 
 
   
    bool HM11::setParityBit(ParityBit_t pBit); 
    
    
  
    ParityBit_t HM11::queryParityBit(void);
    
  
    bool HM11::setPio1OutputStatus(uint8_t status); 
    
    
  
    uint8_t HM11::queryPio1OutputStatus(void);
    
    
    
    
 
    bool HM11::setPioPinsOutput(uint8_t nrOfPio, uint8_t val ); 
    
   
    uint8_t HM11::queryPioPinsOutput(uint8_t nrOfPio);
 
 
 
  
    bool HM11::setPinCode (uint32_t pinCode ); 
    
    
 
    uint8_t HM11::queryPinCode (void);
 
 
 
  
    bool HM11::setModulePower(ModulePower_t modPower); 
    
    
  
    ModulePower_t HM11::queryModulePower (void);
 
 
  
    bool HM11::setModuleSleepType(uint8_t modSleepType ); 
    
    
  
   uint8_t HM11::queryModuleSleepType (void);
 
 
 
  
    bool HM11::restoreAll(void); 
    
  
    bool HM11::restartModule(void);
    
    
 
    bool HM11::setMasterAndSlaveRole(uint8_t role); 
    
    
  
    uint8_t HM11::queryMasterAndSlaveRole(void); 
   
   
   
  
    uint8_t HM11::queryRssiValue(void); 
   
   
   
  
    char* HM11::queryLastConnectedDeviceAddress(void);
   
   
   
 
    bool HM11::setModuleSensorWorkInterval(uint8_t interval); 
    
    
   
    uint8_t HM11::queryModuleSensorWorkInterval(void); 
    
    
 
    bool HM11::workImmediately(void);
    
    
 
    bool HM11::queryModuleIntoSleepMode(void); 
    
    
 
    bool HM11::setModuleSaveConnectedAddressParam(uint8_t saveParam); 
    
  
    uint8_t HM11::queryModuleSaveConnectedAddressParam(void);
    
    
 
    bool HM11::setSensorTypeOnModulePio(SensorType_t sensorType); 
    
    
  
    SensorType_t HM11::querySensorTypeOnModulePio(void);  
   
    
    bool HM11::setDiscoveryParameter (SensorType_t discoverParam); 
    

    uint8_t HM11::queryDiscoveryParameter (void); 
    
    
    
  
    bool HM11::queryModuleSensorTempAndHumidity(uint8_t* temp, uint8_t* hum); 
    
    

    bool HM11::queryDS18B20SensorTemperature (uint8_t* temp); 
    
    
    
   
    bool HM11::setModuleConnectRemoteDeviceTimeoutValue(uint32_t timeout); 
    
    
    

    uint32_t HM11::queryModuleConnectRemoteDeviceTimeoutValue(void); 
    
    
  
    bool HM11::setModuleBondMode(BondMode_t bondMode); 
    
    
  
    BondMode_t HM11::queryModuleBondMode(void); 
    
    
    
    
  
    bool HM11::setServiceUuid (uint16_t serviceUuid); 
    
    
  
    uint16_t HM11::queryServiceUuid(void);
    
    
    
  
    bool HM11::setUartSleepType (uint8_t sleepType); 
    
    
  
    uint8_t HM11::queryUartSleepType(void);
#endif  
    
    bool HM11::querySoftwareVersion(char* verBuf, uint8_t bufLen)
    {
        if(!verBuf)
        {
            return false;
        }
        flushBuffers();
        sendDataToDevice("AT+VERR?");
        if(!waitForData(WAIT_FOR_DATA_TIMEOUT_MS))
            return false;
        else
        {
            copyAvailableDataToBuf((uint8_t*)verBuf,bufLen);
            return true;
        }        
        
                
    }

