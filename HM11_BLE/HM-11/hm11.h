/*
  @file hm11.h
  
  @brief Bluetooth Low Energy v4.0 HM-11 Breakout Library      

  @Author lukasz uszko(luszko@op.pl)

  Tested on FRDM-KL25Z
  
  Copyright (c) 2014 lukasz uszko
  Released under the MIT License (see http://mbed.org/license/mit)

  Documentation regarding the HM-11 Bluetooth Low energy module can be found here: 
  https://www.microduino.cc/wiki/images/f/fc/Bluetooth40_en.pdf
  http://txyz.info/b10n1c/datasheets/hm-11_bluetooth40_en.pdf
*/

//HM-11 Device Pinout:
// Pin        Name              Description
// 1          UART_RTS          UART interface
// 2          UART_TX           UART interface 
// 3          UART_CTS          UART interface
// 4          UART_RX           UART interface
// 5          NC                   NC
// 6          NC                   NC
// 7          NC                   NC
// 8          NC                   NC
// 9          CC                   V3.3
// 10         NC                   NC or VCC
// 11         RESETB            Reset if low <100ms
// 12         GND                 Ground
// 13         PIO3                 Programmable input/output line
// 14         PIO2                 Programmable input/output line
// 15         PIO1                 System LED
// 16         PIO0                 System KEY 

// AT Commands
// Factory default setting:
// Name: HMSoft; Baud: 9600, N, 8, 1; Pin code: 000000; transmit 


//DEMO - HOW TO USE:
//compiled and built under mbed
/*
---------------------------------------- DEMO: 1st version -simple polling ----------------------------------------
#include "mbed.h"
#include "hm11.h"
#include "crc16.h"


#define HM11_PIN_TX PTE22 //FRDM-KL25Z UART2 
#define HM11_PIN_RX PTE23  

int main() {
    Serial usbDebug(USBTX, USBRX);
    usbDebug.printf("HELLO WORLD !");  
    

    HM11* hm11 = new HM11( HM11_PIN_TX, HM11_PIN_RX);
    int counter =0;
    while(1) {
        //myled = 1;
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



#ifndef HM11_H
#define HM11_H


#include "mbed.h"
#include "BufferedSerial.h"

#define HM11_SERIAL_DEFAULT_BAUD       9600
#define HM11_SERIAL_TIMEOUT            10000
#define HM11_SERIAL_EOL                "\r\n"

static const char* hm11TestCommands[]={"AT","AT+","CONNL","RENEW","RESET","START","SLEEP","?"};

static const char* hm11SendCommands[]={"ADDR","BAUD","CLEAR","CON",
                                       "FILT","HELP","IMME","MODE",
                                       "NOTI","NAME","PARI","PIO1",
                                       "PIO","PASS","PIN","POWE",
                                       "PWRM","ROLE",
                                       "RSSI","RADD","STOP",
                                       "SAVE","TCON","TYPE","VERR","VERS"
};

static const char* HM11ReceiveMsgs[]={"OK","OK+","Get:","Set:","LOST","ADDR:","CLEAR","CONN",
                    "NAME","Set","PIO:","RENEW","RESET","RSSI:","RADD:",
                    "START","SLEEP","TCON","RSSI:","RADD:","CONNF"
};

typedef enum hm11CommandsType_t{
    HM11_TEST_COMMAND=0,
    HM11_SEND_COMMAND,
    HM11_NUM_OF_COMMAND_TYPE  
}HM11CommandType;

typedef enum hm11TestCommands_t{
    HM11_AT_TEST =0,
    HM11_START_CMD,
    HM11_CONNECT_TO_LAST_SUCCEEDED_DEVICE , //
    HM11_RESTORE_ALL,   //
    HM11_RESET_MODULE,  //
    HM11_WORK_IMMEDIATELY, //
    HM11_SLEEP_MODE,  //
    HM11_QUERY_SIGN, 
    HM11_NUM_OF_TEST_COMMANDS    
}HM11TestCommands;


typedef enum hm11Commands_t{
    HM11_ADDRESS,
    HM11_BAUDRATE,
    HM11_CLEAR_LAST_CONNECTED_DEVICE_ADDRESS ,
    HM11_CONNECT_TO_ADDRESS,
    HM11_FILTER_AT_COMMAND_PARAMETER,
    HM11_HELP_INFORMATION,
    HM11_MODULE_WORK_TYPE,
    HM11_MODULE_WORK_MODE,
    HM11_NOTIFY_INFORMATION,
    HM11_MODULE_NAME,
    HM11_PARITY_BIT,
    HM11_PIO1_OUTPT_STATUS,
    HM11_PIO_PINS_HIGH_OR_LOW,
    HM11_GET_PIN_CODE,
    HM11_PIN_CODE,
    HM11_MODULE_POWER,
    HM11_MODULE_SLEEP_TYPE,
    HM11_MASTER_SLAVE_ROLL,
    HM11_RSSI_VALUE,
    HM11_LAST_CONNECTED_DEVICE_ADDRESS,
    HM11_STOP_BIT,
    HM11_MODULE_SAVE_CONNECTED_ADDR_PARAMETER,
    HM11_MODULE_CONNECT_REMOTE_DEVICE_TIMEOUT_VALUE,
    HM11_MODULE_BOND_MODE,
    HM11_SOFTWARE_VERSION,
    HM11_NUM_OF_COMMANDS    
}HM11Commands;

/**
    AT Commands parameters
*/
typedef enum AdvertisingInterval{
    _100ms=0,
    _152_5ms,
    _211_25ms,
    _318_75ms,
    _417_5ms,
    _546_25ms,
    _760ms,
    _852_5ms,
    _1022_5ms,
    _1285ms,
    _2000ms,
    _3000ms,
    _4000ms,
    _5000ms,
    _6000ms,
    _7000ms,
    _INVALID_ADV_INTERVAL
}AdvertisingInterval_t;


typedef enum AdvertisingType{
    _AdvertisingScanResponseConnectable=0,
    _LastDeviceConnectsIn1_28Seconds,
    _AdvertisingScanResponse,
    _Advertising,
    _INVALID_ADV_TYPE
}AdvertisingType_t;

typedef enum BaudRate{
    _9600=0,
    _19200,
    _38400,
    _57600,
    _115200,
    _4800,
    _2400,
    _1200,
    _230400,
    _INVALID_BAUDRATE
}BaudRate_t;

typedef enum ConnectionStatus{
      _L, // Connecting
      _E, // Connect error
      _F, // Connect Fail
      _N  // No Address   
}ConnectionStatus_t;


typedef struct ScanResult{
    char* addr;
    char*name;
    uint8_t nr; //0-5
}ScanResult_t;


typedef enum DeployMode{
    _BROAD_AND_SCAN=1, //Allowed to broadcast and scanning
    _ONLY_BROAD,      //Only allow broadcast 
    _INVALID_DEPLOY_MODE
}DeployMode_t; 


typedef enum FilterOfHmModules{
    _ALL_BLE_MODULES=0, //Will find all BLE modules 
    _ONLY_HM_MODULES, //Only find HM Modules 
    _INVALID_FILTER
}FilterOfHmModules_t;


typedef enum ModuleWorkType{
    _WAIT_UNTIL_AT_START_RECEIVED=0, //Will find all BLE modules 
    _WORK_IMMEDIATELY, //Only find HM Modules 
    _INVALID_MODULE_WORK_TYPE
}ModuleWorkType_t;


typedef enum ModuleWorkMode{
    _TRANSM_MODE=0, 
    _PIO_COLLECTION_MODE,  
    _REMOTE_CONTROL_MODE
}ModuleWorkMode_t;




typedef enum ParityBit{
    _NONE=0, 
    _EVEN,  
    _ODD
}ParityBit_t;



class HM11{
    
public:    
    HM11(PinName uartTx , PinName uartRx);
    
    HM11(const BufferedSerial & serial);    
    
    bool sendGetCommand(const char* command);
    bool sendSetCommand(const char* command,int param);
    
    bool isCorrectCommand(const char* command, HM11CommandType cmdType);
    
    int sendDataToDevice(const char* data);
    int sendDataToDevice(uint8_t * byteData,uint8_t dataLength);
    
    int isRxDataAvailable();
    
    inline uint8_t getDataFromRx() {
        return mSerial.getc();
    }
    
    
    //commandMethods  
    bool testCommand(void);
    char* queryModuleAddress(void);
    bool setAdvertisingInterval(AdvertisingInterval_t advInt);
    AdvertisingInterval_t queryAdvertisingInterval(void);
    
    bool setAdvertisingType(AdvertisingType_t advInt);
    AdvertisingType_t queryAdvertisingType(void);
     
     
    /** Set ANCS switch 
     * @param enable |0: Off  |1: On  |Default: 0
     * @return
     *   1 on success,
     *   0 on error.
     * Note1: This command added in V524.
     * Note2: Please send AT+RESET to restart module if you set value 1.
     * Note3: Must execute AT+TYPE3 first. 
    */
    bool setAncsSwitch(uint8_t enable);
    
    
    /** query ANCS switch 
     * @return
     *   1 -On,
     *   0 -Off
     *   0xFF -error
     * Note1: This command added in V524.
     * Note2: Please send AT+RESET to restart module if you set value 1.
     * Note3: Must execute AT+TYPE3 first. 
    */
    uint8_t queryAncsSwitch(void);
    
    
   /** Set Whitelist switch 
     * @param enable |0: Off  |1: On  |Default: 0
     * @return
     *   1 on success,
     *   0 on error.
     * Note1: This command added in V523
     * Note2: Whitelist allow three mac address link to module. Please use AT+AD command set whitelist mac address. 
    */  
    bool setWhitelistSwitch(uint8_t enable);
    
    
    /** query Whitelist Switch
     * @return
     *   1 - On,
     *   0 - Off,
     *   0xFF -error.
     * Note1: This command added in V523
     * Note2: Whitelist allow three mac address link to module. Please use AT+AD command set whitelist mac address. 
    */
    uint8_t queryWhitelistSwitch(void);
    
    
    /** Set whitelist mac address 
     * @param nrOfMacAddrLinkedToModule |1,2,3
     * @param macAddress |eg. 001122334455 
     * @return
     *   1 on success,
     *   0 on error.
    */  
    bool setWhitelistMacAddress (uint8_t nrOfMacAddrLinkedToModule, const char* macAddress);
    
    
    /** query whitelist mac address 
     * @param nrOfMacAddrLinkedToModule |1,2,3
     * @return
     *   nr of mac addr
     *   null -error.
    */
    char* queryWhitelistMacAddress(uint8_t nrOfMacAddrLinkedToModule);
    

    /** Set battery monitor switch   
     * @param uint8_t battSwitchEnable: |0: Off  |1: On  |Default: 0
     * @return
     *   1 on success,
     *   0 on error.
     * Note1: This command added in V520
    */
    bool setBatteryMonitorSwitch(uint8_t battSwitchEnable);
    
    
    /** query BatteryMonitorSwitch 
     * @return
     *   batt switch state: |0: Off  |1: On
     *   0xFF -error
     * Note1: This command added in V520.
    */
    uint8_t queryBatteryMonitorSwitch(void);
    
    
    
    /**Query battery information 
     * @return
     *   batt information: 000~100
     *   0xFF -error
     *
     *  There has three ways to get battery information: 
        a. Before establishing a connection, Send “AT+BATT?” through UART.
        b. After established a connection, In Mode 1 or 2, remote side send“AT+BATT?”
        Battery information has included in scan response data package, one hour update once
    */
    uint8_t queryBatteryInformation(void);

    
    
    /** Set iBeacon into service mode(*) 
     * @return
     *   1 on success,
     *   0 on error.
     * This command is added in V520, Removed in V521, Please use AT+DELO
     * This command set iBeacon into service mode until next power on.
     * In service mode, module not allow any link request.
     * BUSHU is Chinese spelling, meaning the deployment.
     * Note: Should to open iBeacon switch first (AT+IBEA). 
    */
    bool setIBeaconIntoServiceMode(void);
    
    
    
   /** Set Bit format 
     * @param bit7Format: 0:NotCompatible, 1:Compatible
     * @return
     *   1 NotCompatible,
     *   0 Compatible,
     *   0xFF Error
     * This command is used only for compatible uses 7 data bits, 2 stop bit device.  
    */
    bool setBitFormat(uint8_t bit7Format);
    
    
    /**Query BitFormat
     * @return
     *   bit7 Swu=itch
     *   0xFF -error
    */
    uint8_t queryBitFormat(void);
    
    
   /** Set  baud rate
     * @param baud: Baudrate value
     * @return
     *   1 success,
     *   0 Error,
    */
    bool setBaudRate(BaudRate_t baud);
    
    
    
    /**Query  baud rate
     * @return
     *   Baudrate_t - Baudrate value
    */
    BaudRate_t queryBaudRate(void);    
    
    /** Set  Characteristic value
     * @param chValue (characteristic value): 0x0001~0xFFFE 
     * @return
     *   1 success,
     *   0 Error,
    */
    bool setCharacteristic(uint16_t chValue);
    
    
    
    /**Query  Characteristic value
     * @return
     *  characteristic value: 0x0001~0xFFFE 
     *  error :0xFFFF
    */   
    uint16_t queryCharacteristic(void); 
    
    
   /** Try connect to last succeeded device
     * @return
     *  ConnectionStatus_t connection status.
     * Notice: Only Central role is used.
     * If remote device has already connected to other device or shut down,
     * “OK+CONNF” will received after about 10 seconds. 
    */
    ConnectionStatus_t connectToLastDevice(void); 
    
    
    /** Try connect an address 
     * @param address e.g." 0017EA090909 " 
     * @return connection status
     * Notice: Only Central role is used.
     * If remote device has already connected to other device or shut down,
     * “OK+CONNF” will received after about 10 seconds.
     *
     *      Notice: Only central role is used.
            If remote device has already connected to other device or shut down,
            “OK+CONNF” will received after about 10 Seconds.
            e.g.
            Try to connect an device which MAC address is 00:17:EA:09:09:09
            Send: AT+CON0017EA090909
            May receive a reply:
            OK+CONNA ========= Accept request, connecting 
            OK+CONNE ========= Connect error
            OK+CONN ========= Connected, if AT+NOTI1 is setup
            OK+CONNF ========= Connect Failed, After 10 seconds  
    */
    ConnectionStatus_t connectToAnAddress(const char* address); 
    
    
    /**Query PIO04~PIO11 input(output) state 
     * @return
     *  in/out state: 0x00~0xFF 
     * 
     * This command is added since V515 version. 
    */   
    uint8_t queryInputOutputState(void); 
    
    
    /**Set PIO collection rate 
     * @param colRateSec (PIO collection rate): 00-99 unit[s]
     * @return
     *   1 success,
     *   0 Error,
     * 
     * In mode 1, when PIO state is change, module will send OK+Col:[xx] to
     * UART or remote side. This command is set send interval.
     * This command is added since V515 version. 
    */
    bool setPioCollectionRate (uint8_t colRateSec);
    
    
    
    /**Query PIO collection rate 
     * @return
     *  PIO collection rate: 00-99 unit[s] 
     *  error :0xFF
    */   
    uint8_t queryPioCollectionRate(void); 
    
    
        
    /**Start a device discovery scan
     * @param [out] scanRes 
     * @return
     *   1 success,
     *   0 Error,
     
        Please set AT+ROLE1 and AT+IMME1 first.
        ---usage--:
        Send: AT+DISC? 
        Recv: OK+DISCS
        Recv: OK+DISC:123456789012 (discovered device address information)
        If AT+SHOW1 is setup, you will receive then Name information as follow
        Recv: OK+NAME: xxx
        After send Name value, will send two extra “\r\n” value ASCII byte
        Recv: OK+DISC:234567890123
        Recv: OK+NAME: xxx
        After send Name value, will send two extra “\r\n” value ASCII byte  ...(Max results is 6, use array 0~5)
        Recv: OK+DISCE
        Connect to a discovered device: AT+CONN0, AT+CONN1……AT+CONN5 
    */   
    bool startDeviceDiscoveryScan(ScanResult_t* scanRes); 
    
    
      
    /**Connect to an Discovery device 
     * @param [in] scanRes 
     * @return
     *   1 success,
     *   0 Error,
     
        This command is use after execute AT+DISC?
        This command will clear all discovery data. 
    */   
    bool connectToDiscoveryDevice(ScanResult_t* scanRes); 
    
    
       
    /**Set IBeaconDeployMode
     * @param depMode - DeployMode_t
     * @return
     *   1 success,
     *   0 Error,
     * 
     * After receive OK+DELO[para1], module will reset after 500ms.
     * This command will let module into non-connectable status until next power on.
    */
    bool setIBeaconDeployMode(DeployMode_t depMode);
    
    
   /**Set filter of HM modules 
     * @param filter -FilterOfHmModules_t 
     * @return
     *   1 success,
     *   0 Error,
    */   
    bool setFilterOfHmModules(FilterOfHmModules_t filter); 
    
    
    /**Query filter of HM modules 
     * @return
     * Type of filter
    */   
    FilterOfHmModules_t queryFilterOfHmModules(void); 
    
   
    
   /**Remove bond information
     * @return
     *   1 success,
     *   0 Error,
     * Note1: Added in V524 version. 
    */
    bool removeBondInformation(void); 
    
    
   /**System Help Information
     * @param [in] char* helpInformation
     * @return
     *   1 success,
     *   0 Error, 
    */
    bool getSystemHelpInformation(char* helpInformationBuf); 
    
       
   /**Set Module work type 
     * @param modWorkType -ModuleWorkType_t
     * @return
     *   1 success,
     *   0 Error,
     * This command is only used for Central role.
    */   
    bool setModuleWorkType(ModuleWorkType_t modWorkType); 
    
    
    /**Query Module work type  
     * @return
     *      module Work Type -ModuleWorkType_t
     * This command is only used for Central role. 
    */   
    ModuleWorkType_t queryModuleWorkType(void);
    
   
   
  
        
   /**Set Module iBeacon Switch
     * @param turnOnOff
            0: Turn off iBeacon
     *      1: Turn on iBeacon 
     * @return
     *   1 success,
     *   0 Error,
     * This command is added since V517 version. 
    */   
    bool setModuleIBeaconSwitch (uint8_t turnOnOff); 
    
    
    /**Query iBeacon switch  
     * @return
     *      0: Turn off iBeacon
     *      1: Turn on iBeacon 
     * This command is added since V517 version.  
    */   
    uint8_t queryModuleIBeaconSwitch (void);
     
   
       
   /**Set iBeacon UUID 
     * @param:  iBeacon Uuid - 0x00000001~0xFFFFFFFE 
     * @return
     *   1 success,
     *   0 Error,
     * This command is added since V520 version. 
     * Default: iBeacon UUID is: 74278BDA-B644-4520-8F0C-720EAF059935.  
     * -  uuid1 is 74278BDA
     * -  uuid2 is B644-4520
     * -  uuid1 is 8F0C-720E
     * -  uuid1 is AF059935
     */   
    bool setIBeaconUuid (uint32_t uuid0,uint32_t uuid1,uint32_t uuid2, uint32_t uuid3); 
    bool setIBeaconUuid (uint32_t* uuid); 
    
    /**Query iBeacon Uuid 
     * @param:  part of uuid - 0~3
     * @return
     *    iBeacon Uuid - 0x00000001~0xFFFFFFFE 
     *    Error - 0x00
     * This command is added since V520 version.  
    */   
    uint32_t queryIBeaconUuid(uint32_t* nr_of_uuid);
     
    
    
     
    /**Set Module iBeacon major version
     * @param:  major version- 0x0001~0xFFFE  
     * @return
     *   1 success,
     *   0 Error,
     ^ Default: 0xFFE0
     * This command is added since V517 version. 
     */   
    bool setIBeaconMajor(uint16_t mjrVersion); 
    
    
    /**Query Module iBeacon major 
     * @return
     *    iBeacon major version- 0x0001~0xFFFE  
     *    Error - 0x0000
     * This command is added since V517 version.  
    */   
    uint16_t queryIBeaconMajor(void);
    
    
    
    /**Set Module iBeacon minor version
     * @param:  minorversion- 0x0001~0xFFFE  
     * @return
     *   1 success,
     *   0 Error,
     ^ Default: 0xFFE1
     * This command is added since V517 version. 
     */   
    bool setIBeaconMinor(uint16_t mnrVersion); 
    
    
    /**Query Module iBeacon minor
     * @return
     *    iBeacon minor version- 0x0001~0xFFFE  
     *    Error - 0x0000
     * This command is added since V517 version.  
    */   
    uint16_t queryIBeaconMinor(void);
   
   
   
   
   /**Set Module iBeacon Measured power 
     * @param:  measured Power 0x0001~0xFFFE  
     * @return
     *   1 success,
     *   0 Error,
     ^ Default: 0xFFE1
     * This command is added since V519 version. 
     */   
    bool setIBeaconMeasuredPower(uint16_t measuredPwr); 
    
    
    /**Query Module iBeacon Measured power 
     * @return
     *    iBeacon minor version- 0x0001~0xFFFE  
     *    Error - 0x0000
     * This command is added since V519 version.  
    */   
    uint16_t queryIBeaconMeasuredPower(void);
   
   
   
   
    /**Set Module Work Mode 
     * @param:  WorkMode_t
     * @return
     *   1 success,
     *   0 Error,
     ^ Default: 0xFFE1
     * This command is added since V519 version. 
     */   
    bool setModuleWorkMode(ModuleWorkMode_t workMode); 
    
    
    /**Query Module Work Mode  
     * @return
     *    iBeacon minor version- 0x0001~0xFFFE  
     *    Error - 0x0000
     * This command is added since V519 version.  
    */   
    ModuleWorkMode_t  queryModuleWorkMode(void);
   
 
 
  
   /**Set Module name 
     * @param:  name, length<12
     * @return
     *   1 success,
     *   0 Error, 
     */   
    bool setModuleName(char* name, uint8_t nameLength); 
    
    
    /**Query Module name 
     * @param:  name -> ptr to response buffer
     * @return
     *   1 success,
     *   0 Error, 
    */   
    bool queryModuleName(char *name);
 
 
 
  
   /**Set Module parity bit
     * @param:  ParityBit_t pBit
     * @return
     *   1 success,
     *   0 Error, 
     */   
    bool setParityBit(ParityBit_t pBit); 
    
    
    /**Query Module parity bit
     * @return
     *    -ParityBit_t val
     */   
    ParityBit_t queryParityBit(void);
 
 
 
 
 
 
 
 
 
 
 
 
private:
    
    bool waitForData(int timeoutMs);
    BufferedSerial mSerial;
};











#endif