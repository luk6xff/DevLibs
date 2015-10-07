/*
  @file btm222.h

  @brief BTM222 Bluetooth SPP module Library

  @Author lukasz uszko(luszko@op.pl)

  Tested on FRDM-KL46Z and FRDM-KL25Z

  Copyright (c) 2014 lukasz uszko
  Released under the MIT License (see http://mbed.org/license/mit)

  Documentation regarding the MAX9611 might be found here:

*/


/**
 * How to use the library:


#include "btm222.h"
#include "mbed.h"

#define BTM222_PIN_TX PTE22   //UART2 FREEDOM 46KLZ
#define BTM222_PIN_RX PTE23
 BTM222* btm222;
Serial* usbDebug;



int main(){
btm222=new BTM222(BTM222_PIN_TX , BTM222_PIN_RX);
usbDebug= new Serial(USBTX, USBRX);
usbDebug->baud(9600);

while(1){

  btm222->sendDataToBTM('C', jakas_data);     // tu powiedzmy cos wysylasz
  btm222->sendDataToBTM('P',jakas_inna_data);   // tu powiedzmy cos wysylasz


  while(btm222->isRxDataAvailable())                    //tu odbierasz jak by cos bylo w buforze
         usbDebug.printf("data:  %c\r\n",btm222->getDataFromRx());                       //tu drukujemy to odebrane po jednym bajcie                                         //z bufora az do oproznienia


}

return 0;

}



 */







#ifndef __BTM222_H__
#define __BTM222_H__

#include "mbed.h"
#include "BufferedSerial.h"

#define BLUETOOTH_SERIAL_DEFAULT_BAUD       19200
#define BLUETOOTH_SERIAL_TIMEOUT            10000
#define BLUETOOTH_SERIAL_EOL                "\r\n"


class BTM222
{
public:
    BTM222 (PinName tx, PinName rx);

    void getInfo();


    void sendDataToBTM(char tag,float valueToSend);

    int write(const char* data);

    int isRxDataAvailable();

    inline int getDataFromRx() {
        return bufSerial.getc();
    }


protected:

    BufferedSerial bufSerial;
    uint8_t    _buf[64];
};

#endif // __BLUETOOTH_SERIAL_H__
