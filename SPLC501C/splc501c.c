/*
 * splc501c.c
 *
 *  Created on: May 9, 2012
 *      Author: Lukasz USZKO
 */

#include "splc501c.h"
#include "my_library/init_modules.h"

//-------------------------------------------------------------------------------------------------
// TUTAJ inicjuje wszystkie porty
//-------------------------------------------------------------------------------------------------
void GLCD_InitPorts(void) {
	volatile int i;
	GPIO_InitTypeDef GPIO_InitStructure;
	/*Tu nalezy umiescic konfiguracja  portow GPIO potrzebnych w programie*/
	GPIO_InitStructure.GPIO_Pin = PIN_ALL_STER_PORT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; //wyjscie push-pull
	GPIO_Init(PIN_PORT_STER, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = PIN_ALL_DATA_PORT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; //wyjscie push-pull
	GPIO_Init(PIN_PORT_DATA, &GPIO_InitStructure);
	GPIO_SetBits(PIN_PORT_STER, PIN_RD | PIN_WR | PIN_A0 | PIN_RES | PIN_CS1);
	GPIO_ResetBits(PIN_PORT_STER, PIN_RES);
	for (i = 0; i < 1000; i++)
		// petla opozniajaca
		;
	GPIO_SetBits(PIN_PORT_STER, PIN_RES); // to juz jest w inicjalizacji

}
//-------------------------------------------------------------------------------------------------
// inicjacja wyswietlacza
//-------------------------------------------------------------------------------------------------

void GLCD_Init(void) {
	volatile int i;
	GLCD_InitPorts();

	GLCD_WriteCommand(SPLC501C_ADC_NORMAL);
	GLCD_WriteCommand(SPLC501C_COM63);

	GLCD_WriteCommand(SPLC501C_BIAS_19);
	GLCD_WriteCommand(SPLC501C_POWERON);
	for (i = 0; i < 100; i++)
		;
	GLCD_WriteCommand(SPLC501C_VOLUME_MODE);
	GLCD_WriteCommand(SPLC501C_VOLUME_SET | 20);
	GLCD_WriteCommand(0xA4);
	GLCD_WriteCommand(SPLC501C_DISPLAY_ON);
	GLCD_WriteCommand(SPLC501C_DISPLAY_NORMAL);
	GLCD_WriteCommand(SPLC501C_PAGE_ADDRESS | 0);
	GLCD_WriteCommand(SPLC501C_COLUMN_ADDRESS_HI | 0);
	GLCD_WriteCommand(SPLC501C_COLUMN_ADDRESS_LO | 0);
	GLCD_WriteCommand(SPLC501C_START_LINE | 0);

}
//-------------------------------------------------------------------------------------------------
//zapis danych do wyswietlacza
//-------------------------------------------------------------------------------------------------

void GLCD_WriteData(uint8_t dataToWrite) {
	while (GLCD_ReadStatus() & 0x90)
		; // sprawdzam flage zajetosci

	PIN_PORT_DATA->CRL = GPIO_CRL_MODE0_0 | GPIO_CRL_MODE1_0 | GPIO_CRL_MODE2_0
				| GPIO_CRL_MODE3_0 | GPIO_CRL_MODE4_0 | GPIO_CRL_MODE5_0
				| GPIO_CRL_MODE6_0 | GPIO_CRL_MODE7_0; // output mode, push pull , max speed 10 MHz
		;
	GPIO_ResetBits(PIN_PORT_STER, PIN_A0 | PIN_WR | PIN_RD | PIN_CS1);
	GPIO_SetBits(PIN_PORT_STER, PIN_A0 | PIN_RD);
	GPIO_ResetBits(PIN_PORT_DATA, PIN_ALL_DATA_PORT);
	GPIO_SetBits(PIN_PORT_DATA, dataToWrite);
	GPIO_SetBits(PIN_PORT_STER, PIN_CS1 | PIN_WR); // ustawiam CS1 czyli zakoñczone zapisywanie /czytanie
}
//-------------------------------------------------------------------------------------------------
//zapis komendy narazie bedzie dzia³a³ tylko gdy linie data wyswietlacza s¹ podpiête do portów 0d 0 ->do 7
//-------------------------------------------------------------------------------------------------
void GLCD_WriteCommand(uint8_t commandToWrite) {
//while(GLCD_ReadStatus()&0x90); // sprawdzam flage zajetosci
	GPIO_ResetBits(PIN_PORT_STER, PIN_A0 | PIN_WR | PIN_RD | PIN_CS1);
	GPIO_SetBits(PIN_PORT_STER, PIN_RD);
	GPIO_ResetBits(PIN_PORT_DATA, PIN_ALL_DATA_PORT);
	GPIO_SetBits(PIN_PORT_DATA, commandToWrite);
	GPIO_SetBits(PIN_PORT_STER, PIN_CS1); // ustawiam CS1 czyli zakoñczone zapisywanie /czytanie
}
//-------------------------------------------------------------------------------------------------
//Odczyt zapisanych pikseli z wywietlacza
//-------------------------------------------------------------------------------------------------

uint8_t GLCD_ReadData() {
	while (GLCD_ReadStatus() & 0x90)
		// czytam flage zajetosci
		;

	uint8_t read_data = 0;
	/*
	 GPIO_InitTypeDef GPIO_InitStructure;

	 GPIO_InitStructure.GPIO_Pin = PIN_ALL_DATA_PORT; //zmieniam porty na wejciowe
	 GPIO_InitStructure.GPIO_Pin = GPIO_Speed_10MHz;
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; //wejcie High impedance
	 GPIO_Init(PIN_PORT_DATA, &GPIO_InitStructure);
	 */
/*
	PIN_PORT_DATA->CRL &= ~(GPIO_CRL_MODE0_0 | GPIO_CRL_MODE1_0
			| GPIO_CRL_MODE2_0 | GPIO_CRL_MODE3_0 | GPIO_CRL_MODE4_0
			| GPIO_CRL_MODE5_0 | GPIO_CRL_MODE6_0 | GPIO_CRL_MODE7_0); ///input, pull down
	PIN_PORT_DATA->CRL = GPIO_CRL_CNF0_1 | GPIO_CRL_CNF1_1 | GPIO_CRL_CNF2_1
			| GPIO_CRL_CNF3_1 | GPIO_CRL_CNF4_1 | GPIO_CRL_CNF5_1
			| GPIO_CRL_CNF6_1 | GPIO_CRL_CNF7_1; */
	GPIO_ResetBits(PIN_PORT_STER, PIN_RD | PIN_CS1);
	GPIO_SetBits(PIN_PORT_STER, PIN_RD | PIN_CS1);
	GPIO_SetBits(PIN_PORT_STER, PIN_A0 | PIN_WR); //// tu na tym WR sie zastanowic
	GPIO_ResetBits(PIN_PORT_STER, PIN_RD | PIN_CS1);
	read_data |= (PIN_PORT_DATA->IDR & PIN_ALL_DATA_PORT); // czytam stan wejsæ

	/*
	 GPIO_SetBits(PIN_PORT_STER, PIN_CS1 | PIN_RD);
	 GPIO_InitStructure.GPIO_Pin = PIN_ALL_DATA_PORT;
	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; //ustawiam wyjscie  push pullup
	 GPIO_Init(PIN_PORT_DATA, &GPIO_InitStructure);
	 */PIN_PORT_DATA->CRL = GPIO_CRL_MODE0_0 | GPIO_CRL_MODE1_0 | GPIO_CRL_MODE2_0
			| GPIO_CRL_MODE3_0 | GPIO_CRL_MODE4_0 | GPIO_CRL_MODE5_0
			| GPIO_CRL_MODE6_0 | GPIO_CRL_MODE7_0; // output mode, push pull , max speed 10 MHz
	;
	return read_data;
}

//-------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------
uint8_t GLCD_ReadStatus(void) {
	/*	GPIO_InitTypeDef GPIO_InitStructure;
	 GPIO_InitStructure.GPIO_Pin = PIN_ALL_DATA_PORT;
	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; //wejcie High impedance
	 GPIO_Init(PIN_PORT_DATA, &GPIO_InitStructure);*/
	PIN_PORT_DATA->CRL &= ~(GPIO_CRL_MODE0_0 | GPIO_CRL_MODE1_0
			| GPIO_CRL_MODE2_0 | GPIO_CRL_MODE3_0 | GPIO_CRL_MODE4_0
			| GPIO_CRL_MODE5_0 | GPIO_CRL_MODE6_0 | GPIO_CRL_MODE7_0); ///input, pull down
	PIN_PORT_DATA->CRL = GPIO_CRL_CNF0_1 | GPIO_CRL_CNF1_1 | GPIO_CRL_CNF2_1
			| GPIO_CRL_CNF3_1 | GPIO_CRL_CNF4_1 | GPIO_CRL_CNF5_1
			| GPIO_CRL_CNF6_1 | GPIO_CRL_CNF7_1;
	GPIO_ResetBits(PIN_PORT_STER, PIN_A0 | PIN_WR | PIN_RD | PIN_CS1);
	GPIO_SetBits(PIN_PORT_STER, PIN_WR);
	uint8_t read_status = 0;
	read_status = (PIN_PORT_DATA->IDR & PIN_ALL_DATA_PORT);
	GPIO_SetBits(PIN_PORT_STER, PIN_CS1 | PIN_RD | PIN_A0);

	return read_status;
}
//-----------------------------------------------------------------------------------------------------------------------------
//ustawienie kursora
//-----------------------------------------------------------------------------------------------------------------------------
void GLCD_GoTo(uint8_t x, uint8_t y) {
	if (y > PIXELS_PER_PAGE - 1) {
		y = PIXELS_PER_PAGE - 1;
	}
	if (x > SCREEN_WIDTH - 1) {
		x = SCREEN_WIDTH - 1;
	}
	GLCD_WriteCommand(SPLC501C_COLUMN_ADDRESS_HI | ((x >> 4) & 0x0F));
	GLCD_WriteCommand(SPLC501C_COLUMN_ADDRESS_LO | (x & 0x0F));
	GLCD_WriteCommand(SPLC501C_PAGE_ADDRESS | y);

}
//-------------------------------------------------------------------------------------------------
// funkcja rysuj¹ca pixel zakres wejciowych argumentów to x=64 ; y<132
//-------------------------------------------------------------------------------------------------
void GLCD_SetPixel(uint8_t x, uint8_t y, bool color) {
	uint8_t z = 0;
	GLCD_GoTo(x, y / 8);
	z = GLCD_ReadData(); // odczytuje poprzedni stan zapalonych bitów coby se nie nadpisywaæ na wyswietlaczu // rzutuje na 1 bajt
	if (color) {
		z |= (1 << (y % 8));
	} else {
		z &= ~(1 << (y % 8));
	}
	GLCD_GoTo(x, y / 8);
	GLCD_WriteData(z);
}
//-------------------------------------------------------------------------------------------------
//czyszczenie wyswietlacza
//-------------------------------------------------------------------------------------------------
void GLCD_ClearScreen(void) {
	uint8_t x = 0, y = 0;
	for (y = 0; y < (SCREEN_HEIGHT / PIXELS_PER_PAGE); y++) {
		GLCD_GoTo(0, y);
		for (x = 0; x < SCREEN_WIDTH; x++) {
			GLCD_WriteData(0); // ca³y bajt na pojedyñczej stronie se zerami wype³niam
		}
	}
}
//-------------------------------------------------------------------------------------------------
//funkcja wywietlaj¹ca znak z tablicy zapisanej we flashu
//-------------------------------------------------------------------------------------------------
void GLCD_WriteChar(char charCode) {
	unsigned char fontCollumn;
	for (fontCollumn = 0; fontCollumn < FONT_WIDTH; fontCollumn++)
		GLCD_WriteData(
				font5x7[((charCode - FONT_OFFSET) * FONT_WIDTH) + fontCollumn]);
	GLCD_WriteData(0); // coby by³ odstêp  miêdzy literami
}
//-------------------------------------------------------------------------------------------------
// funkcja wysy³aj¹ca string na ekran
//-------------------------------------------------------------------------------------------------
void GLCD_WriteString(char * string) {
	while (*string) {
		GLCD_WriteChar(*string++);
	}
}

//-------------------------------------------------------------------------------------------------
//FUNKCJE RYSUJ¥CE BITMAPY- STOSUJE DRUG¥
//-------------------------------------------------------------------------------------------------
/*
void GLCD_Bitmap(char * bitmap, unsigned char left, unsigned char top,
		unsigned char width, unsigned char height) {
	unsigned char pageIndex, columnIndex;
	for (pageIndex = 0; pageIndex < height / 8; pageIndex++) {
		GLCD_GoTo(left, top + pageIndex);
		for (columnIndex = 0; columnIndex < width; columnIndex++)
			GLCD_WriteData(*(bitmap++));
	}
}
*/
void GLCD_bmp(const unsigned char *bitmapa) {
	unsigned char i, j;
	for (j = 0; j < 8; j++) {
		GLCD_WriteCommand(0x00);
		GLCD_WriteCommand(0x10);
		GLCD_WriteCommand(0xb0 + j); // wybór strony <0,8>
		for (i = 0; i < 132; i++) // wyswietla w poziomie
			GLCD_WriteData((bitmapa[i + j * 132]));
	}
}

//-------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------
/*
 void GLCD_line_recursive(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1) {
 int x, y;
 if (x0 == x1 && y0 == y1) {
 return;
 }

 x = (x0 + x1) >> 2;
 y = (y0 + y1) >> 2;

 GLCD_SetPixel(x, y,true);
 GLCD_line_recursive(x0, y0, x, y);
 GLCD_line_recursive(x1, y1, x, y);
 }
 */

//-------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------
////to NIE MOJE FUNKCJE DO sprawdzenia
void GLCD_Rectangle(unsigned char x, unsigned char y, unsigned char b,
		unsigned char a) {
	unsigned char j; // zmienna pomocnicza
	// rysowanie linii pionowych (boki)
	for (j = 0; j < a; j++) {
		GLCD_SetPixel(x, y + j, true);
		GLCD_SetPixel(x + b - 1, y + j, true);
	}
	// rysowanie linii poziomych (podstawy)
	for (j = 0; j < b; j++) {
		GLCD_SetPixel(x + j, y, true);
		GLCD_SetPixel(x + j, y + a - 1, true);
	}
}

void GLCD_Circle(unsigned char cx, unsigned char cy, unsigned char radius) {
	int x, y, xchange, ychange, radiusError;
	x = radius;
	y = 0;
	xchange = 1 - 2 * radius;
	ychange = 1;
	radiusError = 0;
	while (x >= y) {
		GLCD_SetPixel(cx + x, cy + y, true);
		GLCD_SetPixel(cx - x, cy + y, true);
		GLCD_SetPixel(cx - x, cy - y, true);
		GLCD_SetPixel(cx + x, cy - y, true);
		GLCD_SetPixel(cx + y, cy + x, true);
		GLCD_SetPixel(cx - y, cy + x, true);
		GLCD_SetPixel(cx - y, cy - x, true);
		GLCD_SetPixel(cx + y, cy - x, true);
		y++;
		radiusError += ychange;
		ychange += 2;
		if (2 * radiusError + xchange > 0) {
			x--;
			radiusError += xchange;
			xchange += 2;
		}
	}
}

void GLCD_Line(int X1, int Y1, int X2, int Y2) {
	int CurrentX, CurrentY, Xinc, Yinc, Dx, Dy, TwoDx, TwoDy,
			TwoDxAccumulatedError, TwoDyAccumulatedError;

	Dx = (X2 - X1); // obliczenie skladowej poziomej
	Dy = (Y2 - Y1); // obliczenie sladowej pionowej

	TwoDx = Dx + Dx; // podwojona skladowa pozioma
	TwoDy = Dy + Dy; // podwojona skladowa pionowa

	CurrentX = X1; // zaczynamy od X1
	CurrentY = Y1; // oraz Y1

	Xinc = 1; // ustalamy krok zwekszania pozycji w poziomie
	Yinc = 1; // ustalamy krok zwiekszania pozycji w pionie

	if (Dx < 0) // jesli skladowa pozioma jest ujemna
			{
		Xinc = -1; // to bedziemy sie "cofc" (krok ujemny)
		Dx = -Dx; // zmieniamy znak skladowej na dodatni
		TwoDx = -TwoDx; // jak równiez podwojonej skladowej
	}

	if (Dy < 0) // jezli skladowa pionowa jest ujemna
			{
		Yinc = -1; // to bedziemy sie "cofc" (krok ujemny)
		Dy = -Dy; // zmieniamy znak skladowej na dodatki
		TwoDy = -TwoDy; // jak rowniez podwojonej skladowej
	}

	GLCD_SetPixel(X1, Y1, true); // stawiamy pierwszy krok (zapalamy pierwszy piksel)

	if ((Dx != 0) || (Dy != 0)) // sprawdzamy czy linia sklada sie z wiecej niz jednego punktu
			{
		// sprawdzamy czy skladowa pionowa jest mniejsza lub rowna skladowej poziomej
		if (Dy <= Dx) // jezli tak, to idziemy "po iksach"
				{
			TwoDxAccumulatedError = 0; // zerujemy zmienna
			do // ruszamy w droge
			{
				CurrentX += Xinc; // do aktualnej pozycji dodajemy krok
				TwoDxAccumulatedError += TwoDy; // a tu dodajemy podwojona skaadowa pionowa
				if (TwoDxAccumulatedError > Dx) // jezli TwoDxAccumulatedError jest wiekszy od Dx
						{
					CurrentY += Yinc; // zwiekszamy aktualna pozycje w pionie
					TwoDxAccumulatedError -= TwoDx; // i odejmujemy TwoDx
				}
				GLCD_SetPixel(CurrentX, CurrentY, true); // stawiamy nastepny krok (zapalamy piksel)
			} while (CurrentX != X2); // idziemy tak deugo, az osiagniemy punkt docelowy
		} else // w przeciwnym razie idziemy "po igrekach"
		{
			TwoDyAccumulatedError = 0;
			do {
				CurrentY += Yinc;
				TwoDyAccumulatedError += TwoDx;
				if (TwoDyAccumulatedError > Dy) {
					CurrentX += Xinc;
					TwoDyAccumulatedError -= TwoDy;
				}
				GLCD_SetPixel(CurrentX, CurrentY, true);
			} while (CurrentY != Y2);
		}
	}
}

