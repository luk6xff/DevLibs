/*
 * ds1621.c  
 *
 *  Created on: 04-12-2012
 *      Author: £ukasz Uszko z wyk. funkcji do obrobki danych getTemperature od Dharmaniego.
 */
#include <avr/io.h>
#include <avr/interrupt.h>
#include "ds1621.h"
#include "i2c_twi.h"
#include "lcd44780.h"
#include <util/delay.h>

unsigned char tempDisplay[] = "+xx.x"; // format wysw temperatury
//******************************************************************
//Function inicjuj¹ca czujniki
//******************************************************************
bool ds1621_init(uint8_t addr) {
	bool ErrorStatus;
	I2C_init(100000);
	ErrorStatus = I2C_write_data(addr, ACCESS_CFG, 1, 0x03); //commmand to set o/ppolarity high, single shot conversion
	return ErrorStatus;
}

//******************************************************************
//funkcja do zapisywania komendy do czujnika
//******************************************************************
bool ds1621_sendCommand(uint8_t command, uint8_t addr) {
	bool ErrorStatus;
	ErrorStatus = I2C_write_data(addr, command, 1, 0x01);
	return ErrorStatus;

}

//******************************************************************
//funkcja do czytania z czujnika
//******************************************************************

uint8_t ds1621_readValue(uint8_t value, uint8_t addr) {
	uint8_t data;
	data = I2C_read_data(addr, value, 1);
	return (data);
}

//******************************************************************
//funkcja do odczytu temperatury   // pozyczona od darmaniego
//******************************************************************
uint8_t* getTemperature(uint8_t addr) {
	char temperature, counter, slope;
	int temp;
	float actualTemp;

	if(ds1621_sendCommand(START_CNV, addr)){return (uint8_t*) READ_TEMP_ERROR;}

	// _delay_ms(1000);

	temperature = ds1621_readValue(RD_TEMP, addr);
	counter = ds1621_readValue(RD_CNTR, addr);
	slope = ds1621_readValue(RD_SLOPE, addr);

	if (temperature == ERROR_CODE || counter == ERROR_CODE
			|| slope == ERROR_CODE) {
		return (uint8_t*) READ_TEMP_ERROR;
	}

	actualTemp = (float) temperature - 0.25
			+ ((float) (slope - counter) / (float) slope);

	temp = (int) (actualTemp * 10.0); //to include decimal point for display

	if ((actualTemp * 10.0 - temp) >= 0.5)
		temp = temp + 1;

//    tempDisplay[8]=0xdf;			//Symbol of degree

	if (temp < 0) {
		tempDisplay[0] = '-';
		temp *= -1;
	} else {
		tempDisplay[0] = '+';
	}

	tempDisplay[4] = ((uint8_t) (temp % 10)) | 0x30;
	temp = temp / 10;

	tempDisplay[2] = ((uint8_t) (temp % 10)) | 0x30;
	temp = temp / 10;

	tempDisplay[1] = ((uint8_t) (temp % 10)) | 0x30;
	temp = temp / 10;

	return tempDisplay;
}
//******************************************************************
//init Timera0 i jego interrupta do wyzwalania cyklicznego pomiaru
//******************************************************************
void TIMER0_Conf(void) { // w przerwaniu od tego timera co jakiœ czas uruchamiam konwersje temperatury i z czytuje aktualn¹ temp, coby odci¹¿yæ CPU
	TCCR0 |= (1 << WGM01 | 1 << CS00 | 1 << CS01 | 1 << CS02); // tryb CTC // prescaler 1024
	OCR0 = 156; // 16MHz/ 1024/156  ~= 100Hz
	TIMSK |= 1 << OCIE0;

}

ISR(TIMER0_COMP_vect) {
	static uint8_t i; // tu se mozesz ustawic co jaki czas wykonywany pomiar// ja daje na razie co 1sekunde na kazdy Czujnik
	i++;
	if (i == 50) {
		flag_czuj = 1;
	} else if (i == 100) {
		flag_czuj = 2;
		i = 0;
	}
}
