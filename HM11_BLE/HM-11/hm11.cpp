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
    
    int HM11::sendDataToDevice(uint8_t * byteData,uint8_t dataLength){
    
        return mSerial.write(byteData,dataLength);
    }
    
    
    
    int HM11::isRxDataAvailable(){
        return mSerial.readable();
    }
    
// public methods

    char* HM11::queryModuleAddress(void){
        
    }
    
#if 0   
    bool HM11::setAdvertisingInterval(AdvertisingInterval_t advInt);
    AdvertisingInterval_t HM11::queryAdvertisingInterval(void);
    
    bool HM11::setAdvertisingType(AdvertisingType_t advInt);
    
    AdvertisingType_t HM11::queryAdvertisingType(void);
     
     

    bool HM11::setAncsSwitch(uint8_t enable);
    
    

    uint8_t HM11::queryAncsSwitch(void);
    
  
    bool HM11::setWhitelistSwitch(uint8_t enable);
    

    uint8_t HM11::queryWhitelistSwitch(void);
    

    bool HM11::setWhitelistMacAddress (uint8_t nrOfMacAddrLinkedToModule, const char* macAddress);
    

    char* HM11::queryWhitelistMacAddress(uint8_t nrOfMacAddrLinkedToModule);

    bool HM11::setBatteryMonitorSwitch(uint8_t battSwitchEnable);
    
 
    uint8_t HM11::queryBatteryMonitorSwitch(void);
    
    

    uint8_t HM11::queryBatteryInformation(void);

    

    bool HM11::setIBeaconIntoServiceMode(void);
    
    
    
    bool HM11::setBitFormat(uint8_t bit7Format);
    
 
    uint8_t HM11::queryBitFormat(void);
    
    

    bool HM11::setBaudRate(BaudRate_t baud);
    
    

    BaudRate_t HM11::queryBaudRate(void);    
    

    bool HM11::setCharacteristic(uint16_t chValue);
    
    
 
    uint16_t HM11::queryCharacteristic(void); 
    

    ConnectionStatus_t HM11::connectToLastDevice(void); 
    
    
 
    ConnectionStatus_t HM11::connectToAnAddress(const char* address); 
    

    uint8_t HM11::queryInputOutputState(void); 
    

    bool HM11::setPioCollectionRate (uint8_t colRateSec);
    
   
    uint8_t HM11::queryPioCollectionRate(void); 
    
  
    bool HM11::startDeviceDiscoveryScan(ScanResult_t* scanRes); 
    
    
  
    bool HM11::connectToDiscoveryDevice(ScanResult_t* scanRes); 
    
    
       

    bool HM11::setIBeaconDeployMode(DeployMode_t depMode);
    
    
   
    bool HM11::setFilterOfHmModules(FilterOfHmModules_t filter); 
    
    
 
    FilterOfHmModules_t HM11::queryFilterOfHmModules(void); 
    
   
  
    bool HM11::removeBondInformation(void); 
    

    bool HM11::getSystemHelpInformation(char* helpInformationBuf); 
    
  
    bool HM11::setModuleWorkType(ModuleWorkType_t modWorkType); 
    
    
   
    ModuleWorkType_t HM11::queryModuleWorkType(void);
    
   
   
  
   
    bool HM11::setModuleIBeaconSwitch (uint8_t turnOnOff); 
    
  
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
  
    char* HM11::querySoftwareVersion(void); 

#endif