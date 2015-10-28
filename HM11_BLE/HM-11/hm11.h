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


class HM11{
    
public:    
    HM11(PinName uartTx , PinName uartRx);
    
    HM11(const BufferedSerial & serial);    
    
    bool sendGetCommand(const char* command);
    bool sendSetCommand(const char* command,int param);
    
    bool isCorrectCommand(const char* command, HM11CommandType cmdType);
    
    int sendDataToDevice(const char* data);
    
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
    
    
    
private:
    
    bool waitForData(int timeoutMs);
    BufferedSerial mSerial;
};











#endif