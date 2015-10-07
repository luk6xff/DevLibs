/*
  @file hm11.h
  
  @brief Bluetooth Low Energy v4.0 HM-11 Breakout Library      

  @Author lukasz uszko(luszko@op.pl)

  Tested on FRDM-KL25Z
  
  Copyright (c) 2014 lukasz uszko
  Released under the MIT License (see http://mbed.org/license/mit)

  Documentation regarding the HM-11 Bluetooth Low energy module can be found here: 
  https://www.microduino.cc/wiki/images/f/fc/Bluetooth40_en.pdf
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

#ifndef HM11_H
#define HM11_H


#include "mbed.h"
#include "BufferedSerial.h"

#define HM11_SERIAL_DEFAULT_BAUD       9600
#define HM11_SERIAL_TIMEOUT            10000
#define HM11_SERIAL_EOL                "\r\n"

static const char* hm11TestCommands[]={"AT","AT+","CONNL","RENEW","RESET","START","SLEEP","?","OK"};

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


typedef enum hm11TestCommands_t{
    HM11_AT_TEST =0,
    HM11_START_CMD,
    HM11_CONNECT_TO_LAST_SUCCEEDED_DEVICE , //
    HM11_RESTORE_ALL,   //
    HM11_RESET_MODULE,  //
    HM11_WORK_IMMEDIATELY, //
    HM11_SLEEP_MODE,  //
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

class HM11{
    
public:    
    HM11(PinName uartTx , PinName uartRx);
    
    HM11(const BufferedSerial & serial);    
    
    bool sendGetCommand(const char* command);
    bool sendSetCommand(const char* command);
    
    bool isCorrectCommand(const char* command);
    
    int sendDataToDevice(const char* data);
    
    int isRxDataAvailable();
    
    inline uint8_t getDataFromRx() {
        return mSerial.getc();
    }
    
private:

    BufferedSerial mSerial;
};











#endif