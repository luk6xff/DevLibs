/*
 * ds1621.h  
 *
 *  Created on: 04-12-2012
 *      Author:£ukasz Uszko
 */

#ifndef DS1621_H_
#define DS1621_H_
#include <avr/io.h>
#include <stdbool.h>
#define  DS1621_addrW_NO1			0x90
#define  DS1621_addrR_NO1			0x91
#define  DS1621_addrW_NO2			0x92
#define  DS1621_addrR_NO2           0x93


// DS1621 Registers & Commands
 static const uint8_t RD_TEMP    = 0xAA;        // read temperature register
 static const uint8_t ACCESS_TH  = 0xA1;        // access high temperature register
 static const uint8_t ACCESS_TL  = 0xA2;        // access low temperature register
 static const uint8_t ACCESS_CFG = 0xAC;        // access configuration register
 static const uint8_t RD_CNTR    = 0xA8;        // read counter register
 static const uint8_t RD_SLOPE   = 0xA9;        // read slope register
 static const uint8_t START_CNV  = 0xEE;        // start temperature conversion
 static const uint8_t STOP_CNV   = 0X22;        // stop temperature conversion  // DS1621 configuration bits
 static const uint8_t DONE       = 0x80; 	   	 // conversion is done
 static const uint8_t THF        = 0x40;		 // high temp flag
 static const uint8_t TLF        = 0x20; 	     // low temp flag
 static const uint8_t NVB        = 0x10; 	     // non-volatile memory is busy
 static const uint8_t POL        = 0x02; 	     // output polarity (1 = high, 0 = low)
 static const uint8_t ONE_SHOT   = 0x01; 	     // 1 = one conversion; 0 = continuous conversion
 static const char* READ_TEMP_ERROR = "-----";   // tekst na wyswietlaczu se mozesz ustawic jaki bedzie w razie bledu
 bool ds1621_init (uint8_t  );
 bool ds1621_sendCommand (uint8_t,uint8_t );
 uint8_t ds1621_readValue (  uint8_t,uint8_t);
 uint8_t* getTemperature ( uint8_t addr );
 void TIMER0_Conf(void); // timer do wyzwalania pomiarow
 extern volatile uint8_t flag_czuj;


#endif /* DS1621_H_ */
