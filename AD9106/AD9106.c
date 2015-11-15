/*
 * AD9106.c
 *
 *  Created on: 25-01-2015
 *      Author: lukasz
 *      documentation: http://www.analog.com/en/digital-to-analog-converters/high-speed-da-converters/ad9106/products/product.html?src=ad9106.pdf
 */
#include "AD9106.h"
#include "em_cmu.h"
#include "utils.h"
//privates
#define SPI_SW_ENABLED 0

static const USART_InitSync_TypeDef initSpi = { usartEnable, /* Enable RX/TX when init completed. */
1000000, /* Use 1MHz reference clock */
1000, /* 1 Mbits/s. */
usartDatabits16, /* 16 databits. */
true, /* Master mode. */
true, /* Send most significant bit first. */
usartClockMode0, false, usartPrsRxCh0, false };

static StatusTypeDef spiPeripheralsConfig(void) {

	CMU_ClockEnable(cmuClock_HFPER, true);
	// Enable clock for USART2
	CMU_ClockEnable(USART_CLK, true);
	CMU_ClockEnable(cmuClock_GPIO, true);

	// Reset USART just in case
	USART_Reset(USART_USED );

	USART_InitSync(USART_USED, &initSpi);

	// Module USART2 is configured to location 0
	USART_USED ->ROUTE = (USART_USED ->ROUTE & ~_USART_ROUTE_LOCATION_MASK)
			| USART_ROUTE_LOCATION_LOC0;

	// Enable signals TX, RX, CLK, CS
	USART_USED ->ROUTE |= USART_ROUTE_TXPEN | USART_ROUTE_RXPEN
			| USART_ROUTE_CLKPEN;

	//USART_USED ->CTRL |= USART_CTRL_CLKPOL_IDLEHIGH;
	//USART_USED ->CTRL |= USART_CTRL_CLKPHA_SAMPLELEADING;
	USART_USED ->CMD = USART_CMD_TXEN | USART_CMD_RXEN;
	// Clear previous interrupts
	USART_USED ->IFC = _USART_IFC_MASK;

	GPIO_PinModeSet(AD9106_PORT_MOSI, AD9106_PIN_MOSI, gpioModePushPull, 1);
	GPIO_PinModeSet(AD9106_PORT_MISO, AD9106_PIN_MISO, gpioModeInput, 0);
	GPIO_PinModeSet(AD9106_PORT_CLK, AD9106_PIN_CLK, gpioModePushPull, 0);
	// Keep CS high to not activate slave
	GPIO_PinModeSet(AD9106_PORT_CS, AD9106_PIN_CS, gpioModePushPull, 1);
	AD9106_TRIGGER_OUTPUT(); ///tRIGGER AS OUTPUT
	return STATUS_OK;
}

static SpiHandleTypeDef spiHandle;
//*****************************************************************************
//
//! \brief Initialize ADS9106Init
//!
//! \param None.
//!
//! This function initializes ADS7843's SPI interface and .
//!
//! \return None.
//
//*****************************************************************************
void AD9106Init(void) {

#if SPI_SW_ENABLED

	AD9106spiInitSoftware();

#else
	spiHandle.spiInstance = USART_USED;
	spiHandle.init = &initSpi;
	spiHandle.spiModeHwSw = false;
	//spiHandle.spiSwTransfer=(struct _SpiHandleTypeDef* hspi);
	spiHandle.spiGpioClockInit = spiPeripheralsConfig;
	if (spiInit(&spiHandle) != STATUS_OK)
		while (1)
			;
#endif
}

static const uint16_t sinLookupTable[] = { 0x1fff, 0x1faf, 0x1f61, 0x1f13,
		0x1ec5, 0x1e79, 0x1e2d, 0x1de2, 0x1d97, 0x1d4e, 0x1d05, 0x1cbd, 0x1c75,
		0x1c2e, 0x1be8, 0x1ba3, 0x1b5e, 0x1b1a, 0x1ad7, 0x1a94, 0x1a52, 0x1a10,
		0x19cf, 0x198f, 0x1950, 0x1911, 0x18d2, 0x1895, 0x1857, 0x181b, 0x17df,
		0x17a3, 0x1769, 0x172e, 0x16f5, 0x16bc, 0x1683, 0x164b, 0x1614, 0x15dd,
		0x15a6, 0x1570, 0x153b, 0x1506, 0x14d2, 0x149e, 0x146b, 0x1438, 0x1406,
		0x13d4, 0x13a3, 0x1372, 0x1341, 0x1312, 0x12e2, 0x12b3, 0x1285, 0x1257,
		0x1229, 0x11fc, 0x11cf, 0x11a3, 0x1177, 0x114b, 0x1120, 0x10f6, 0x10cc,
		0x10a2, 0x1078, 0x104f, 0x1027, 0xfff, 0xfd7, 0xfaf, 0xf88, 0xf62,
		0xf3c, 0xf16, 0xef0, 0xecb, 0xea6, 0xe82, 0xe5e, 0xe3a, 0xe16, 0xdf3,
		0xdd1, 0xdae, 0xd8c, 0xd6b, 0xd49, 0xd28, 0xd07, 0xce7, 0xcc7, 0xca7,
		0xc88, 0xc68, 0xc4a, 0xc2b, 0xc0d, 0xbef, 0xbd1, 0xbb4, 0xb97, 0xb7a,
		0xb5d, 0xb41, 0xb25, 0xb09, 0xaee, 0xad3, 0xab8, 0xa9d, 0xa83, 0xa68,
		0xa4f, 0xa35, 0xa1c, 0xa02, 0x9e9, 0x9d1, 0x9b8, 0x9a0, 0x988, 0x971,
		0x959, 0x942, 0x92b, 0x914, 0x8fd, 0x8e7, 0x8d1, 0x8bb, 0x8a5, 0x890,
		0x87a, 0x865, 0x850, 0x83c, 0x827, 0x813, 0x7ff, 0x7eb, 0x7d7, 0x7c4,
		0x7b0, 0x79d, 0x78a, 0x778, 0x765, 0x753, 0x740, 0x72e, 0x71d, 0x70b,
		0x6f9, 0x6e8, 0x6d7, 0x6c6, 0x6b5, 0x6a4, 0x694, 0x683, 0x673, 0x663,
		0x653, 0x643, 0x634, 0x624, 0x615, 0x606, 0x5f7, 0x5e8, 0x5da, 0x5cb,
		0x5bd, 0x5ae, 0x5a0, 0x592, 0x584, 0x577, 0x569, 0x55c, 0x54e, 0x541,
		0x534, 0x527, 0x51a, 0x50d, 0x501, 0x4f4, 0x4e8, 0x4dc, 0x4d0, 0x4c4,
		0x4b8, 0x4ac, 0x4a1, 0x495, 0x48a, 0x47e, 0x473, 0x468, 0x45d, 0x452,
		0x448, 0x43d, 0x432, 0x428, 0x41e, 0x413, 0x409, 0x3ff, 0x3f5, 0x3eb,
		0x3e2, 0x3d8, 0x3ce, 0x3c5, 0x3bc, 0x3b2, 0x3a9, 0x3a0, 0x397, 0x38e,
		0x385, 0x37c, 0x374, 0x36b, 0x363, 0x35a, 0x352, 0x34a, 0x342, 0x339,
		0x331, 0x329, 0x322, 0x31a, 0x312, 0x30a, 0x303, 0x2fb, 0x2f4, 0x2ed,
		0x2e5, 0x2de, 0x2d7, 0x2d0, 0x2c9, 0x2c2, 0x2bb, 0x2b4, 0x2ae, 0x2a7,
		0x2a0, 0x29a, 0x293, 0x28d, 0x287, 0x280, 0x27a, 0x274, 0x26e, 0x268,
		0x262, 0x25c, 0x256, 0x250, 0x24a, 0x245, 0x23f, 0x23a, 0x234, 0x22e,
		0x229, 0x224, 0x21e, 0x219, 0x214, 0x20f, 0x20a, 0x205, 0x200, 0x1fb,
		0x1f6, 0x1f1, 0x1ec, 0x1e7, 0x1e2, 0x1de, 0x1d9, 0x1d4, 0x1d0, 0x1cb,
		0x1c7, 0x1c3, 0x1be, 0x1ba, 0x1b6, 0x1b1, 0x1ad, 0x1a9, 0x1a5, 0x1a1,
		0x19d, 0x199, 0x195, 0x191, 0x18d, 0x189, 0x185, 0x181, 0x17e, 0x17a,
		0x176, 0x173, 0x16f, 0x16b, 0x168, 0x164, 0x161, 0x15d, 0x15a, 0x157,
		0x153, 0x150, 0x14d, 0x14a, 0x146, 0x143, 0x140, 0x13d, 0x13a, 0x137,
		0x134, 0x131, 0x12e, 0x12b, 0x128, 0x125, 0x122, 0x11f, 0x11d, 0x11a,
		0x117, 0x114, 0x112, 0x10f, 0x10c, 0x10a, 0x107, 0x105, 0x102, 0x100,
		0xfd, 0xfb, 0xf8, 0xf6, 0xf4, 0xf1, 0xef, 0xec, 0xea, 0xe8, 0xe6, 0xe3,
		0xe1, 0xdf, 0xdd, 0xdb, 0xd9, 0xd6, 0xd4, 0xd2, 0xd0, 0xce, 0xcc, 0xca,
		0xc8, 0xc6, 0xc4, 0xc3, 0xc1, 0xbf, 0xbd, 0xbb, 0xb9, 0xb7, 0xb6, 0xb4,
		0xb2, 0xb0, 0xaf, 0xad, 0xab, 0xaa, 0xa8, 0xa6, 0xa5, 0xa3, 0xa2, 0xa0,
		0x9e, 0x9d, 0x9b, 0x9a, 0x98, 0x97, 0x95, 0x94, 0x93, 0x91, 0x90, 0x8e,
		0x8d, 0x8c, 0x8a, 0x89, 0x88, 0x86, 0x85, 0x84, 0x82, 0x81, 0x80, 0x7f,
		0x7d, 0x7c, 0x7b, 0x7a, 0x79, 0x77, 0x76, 0x75, 0x74, 0x73, 0x72, 0x71,
		0x6f, 0x6e, 0x6d, 0x6c, 0x6b, 0x6a, 0x69, 0x68, 0x67, 0x66, 0x65, 0x64,
		0x63, 0x62, 0x61, 0x60, 0x5f, 0x5e, 0x5e, 0x5d, 0x5c, 0x5b, 0x5a, 0x59,
		0x58, 0x57, 0x56, 0x56, 0x55, 0x54, 0x53, 0x52, 0x52, 0x51, 0x50, 0x4f,
		0x4e, 0x4e, 0x4d, 0x4c, 0x4b, 0x4b, 0x4a, 0x49, 0x49, 0x48, 0x47, 0x46,
		0x46, 0x45, 0x44, 0x44, 0x43, 0x42, 0x42, 0x41, 0x41, 0x40, 0x3f, 0x3f,
		0x3e, 0x3d, 0x3d, 0x3c, 0x3c, 0x3b, 0x3b, 0x3a, 0x39, 0x39, 0x38, 0x38,
		0x37, 0x37, 0x36, 0x36, 0x35, 0x35, 0x34, 0x34, 0x33, 0x33, 0x32, 0x32,
		0x31, 0x31, 0x30, 0x30, 0x2f, 0x2f, 0x2e, 0x2e, 0x2d, 0x2d, 0x2d, 0x2c,
		0x2c, 0x2b, 0x2b, 0x2a, 0x2a, 0x2a, 0x29, 0x29, 0x28, 0x28, 0x28, 0x27,
		0x27, 0x26, 0x26, 0x26, 0x25, 0x25, 0x25, 0x24, 0x24, 0x24, 0x23, 0x23,
		0x23, 0x22, 0x22, 0x22, 0x21, 0x21, 0x21, 0x20, 0x20, 0x20, 0x1f, 0x1f,
		0x1f, 0x1e, 0x1e, 0x1e, 0x1e, 0x1d, 0x1d, 0x1d, 0x1c, 0x1c, 0x1c, 0x1c,
		0x1b, 0x1b, 0x1b, 0x1b, 0x1a, 0x1a, 0x1a, 0x1a, 0x19, 0x19, 0x19, 0x19,
		0x18, 0x18, 0x18, 0x18, 0x17, 0x17, 0x17, 0x17, 0x16, 0x16, 0x16, 0x16,
		0x16, 0x15, 0x15, 0x15, 0x15, 0x15, 0x14, 0x14, 0x14, 0x14, 0x14, 0x13,
		0x13, 0x13, 0x13, 0x13, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x11, 0x11,
		0x11, 0x11, 0x11, 0x11, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0xf,
		0xf, 0xf, 0xf, 0xf, 0xf, 0xe, 0xe, 0xe, 0xe, 0xe, 0xe, 0xe, 0xe, 0xd,
		0xd, 0xd, 0xd, 0xd, 0xd, 0xd, 0xd, 0xc, 0xc, 0xc, 0xc, 0xc, 0xc, 0xc,
		0xc, 0xb, 0xb, 0xb, 0xb, 0xb, 0xb, 0xb, 0xb, 0xb, 0xa, 0xa, 0xa, 0xa,
		0xa, 0xa, 0xa, 0xa, 0xa, 0xa, 0xa, 0x9, 0x9, 0x9, 0x9, 0x9, 0x9, 0x9,
		0x9, 0x9, 0x9, 0x9, 0x8, 0x8, 0x8, 0x8, 0x8, 0x8, 0x8, 0x8, 0x8, 0x8,
		0x8, 0x8, 0x8, 0x7, 0x7, 0x7, 0x7, 0x7, 0x7, 0x7, 0x7, 0x7, 0x7, 0x7,
		0x7, 0x7, 0x7, 0x7, 0x6, 0x6, 0x6, 0x6, 0x6, 0x6, 0x6, 0x6, 0x6, 0x6,
		0x6, 0x6, 0x6, 0x6, 0x6, 0x6, 0x6, 0x5, 0x5, 0x5, 0x5, 0x5, 0x5, 0x5,
		0x5, 0x5, 0x5, 0x5, 0x5, 0x5, 0x5, 0x5, 0x5, 0x5, 0x5, 0x5, 0x5, 0x4,
		0x4, 0x4, 0x4, 0x4, 0x4, 0x4, 0x4, 0x4, 0x4, 0x4, 0x4, 0x4, 0x4, 0x4,
		0x4, 0x4, 0x4, 0x4, 0x4, 0x4, 0x4, 0x4, 0x4, 0x4, 0x4, 0x3, 0x3, 0x3,
		0x3, 0x3, 0x3, 0x3, 0x3, 0x3, 0x3, 0x3, 0x3, 0x3, 0x3, 0x3, 0x3, 0x3,
		0x3, 0x3, 0x3, 0x3, 0x3, 0x3, 0x3, 0x3, 0x3, 0x3, 0x3, 0x3, 0x3, 0x3,
		0x3, 0x3, 0x3, 0x2, 0x2, 0x2, 0x2, 0x2, 0x2, 0x2, 0x2, 0x2, 0x2, 0x2,
		0x2, 0x2, 0x2, 0x2, 0x2, 0x2, 0x2, 0x2, 0x2, 0x2, 0x2, 0x2, 0x2, 0x2,
		0x2, 0x2, 0x2, 0x2, 0x2, 0x2, 0x2, 0x2, 0x2, 0x2, 0x2, 0x2, 0x2, 0x2,
		0x2, 0x2, 0x2, 0x2, 0x2, 0x2, 0x2, 0x2, 0x2, 0x2, 0x2, 0x2, 0x2, 0x2,
		0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1,
		0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1,
		0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1,
		0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1,
		0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1,
		0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1,
		0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1,
		0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1,
		0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
		0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
		0x0, 0x0 };

//private variables:

/* @brief disables all SPI peripherals*/
void spiDisable(void) {
	USART_Reset(USART_USED );
	USART_USED ->ROUTE = _USART_ROUTE_RESETVALUE;

	/* Disable SPI pins */AD9106_MOSI_DISABLED();
	AD9106_MISO_DISABLED();
	AD9106_CLK_DISABLED();
	AD9106_CS_DISABLED();

	/* Disable USART clock - we can't disable GPIO or HFPER as we don't know who else
	 * might be using it */
	CMU_ClockEnable(USART_CLK, false);
}

uint8_t getShiftValue(uint16_t dataBitMask) {
	uint8_t shiftVal = 0;
	while (!(dataBitMask & 1)) {
		dataBitMask >>= 1;
		shiftVal++;
	}
	return shiftVal;
}

/* @brief  Perform SPI Transfer*/
static uint16_t ADS7843SpiReadReg(AD9106RegAddress regAddress,
		uint16_t dataBitMask) {
	uint8_t txBuf[4] = { regAddress, ((regAddress >> 8) & 0xFF) | 0x80 };
	uint8_t rxBuf[4] = { 0 };
	AD9106_CS_LOW();
	if (spiTransmitReceive(&spiHandle, txBuf, rxBuf, 2, 0) == STATUS_OK) {

		uint16_t retVal = ((rxBuf[3] << 8) | (rxBuf[2])) & 0xFFFF;
		if (dataBitMask != 0) {
			retVal &= dataBitMask;
			retVal >>= getShiftValue(dataBitMask);
		}
		AD91063_CS_HIGH();
		return retVal;
	} else {
		AD91063_CS_HIGH();
		return 0xffff;
	}
}

static bool ADS7843SpiWriteReg(AD9106RegAddress regAddress, uint16_t data,
		uint16_t dataBitMask) {

	if (dataBitMask != 0) {

		uint16_t tempVal = ADS7843SpiReadReg(regAddress, 0);
		tempVal &= ~(dataBitMask);
		tempVal |= data << getShiftValue(dataBitMask);
		data = tempVal;

	}
	uint8_t txBuf[4] = { regAddress, ((regAddress >> 8) & 0xFF) & (~0x80), data,
			data >> 8 };
	uint8_t rxBuf[4] = { 0 };
	AD9106_CS_LOW();

	if (spiTransmitReceive(&spiHandle, txBuf, rxBuf, 2, 0) == STATUS_OK) {
		AD91063_CS_HIGH();
		return true;
	} else {
		AD91063_CS_HIGH();
		return false;
	}
}

//software SPI

void AD9106spiInitSoftware(void) {
	CMU_ClockEnable(cmuClock_GPIO, true);
	AD9106_MOSI_OUTPUT();
	AD9106_MISO_INPUT();
	AD9106_CLK_OUTPUT();
	// Keep CS high to not activate slave
	AD9106_CS_OUTPUT();
	AD9106_CLK_LOW();
	AD9106_MOSI_LOW();
}

uint16_t ADS9106SpiWriteRegS(uint16_t addr, uint16_t data) {
	AD9106_CLK_HIGH();
	Delay(1);
	addr &= ~0x8000; //writing
	AD9106_CS_LOW();

	for (int i = 0; i < 16; i++) {

		if (addr & 0x8000)
			AD9106_MOSI_HIGH();
		else
			AD9106_MOSI_LOW();
		AD9106_CLK_LOW();
		Delay(1);
		AD9106_CLK_HIGH();
		addr <<= 1;
		Delay(1);

	}

	for (int i = 0; i < 16; i++) {

		if (data & (1 << (15 - i)))
			AD9106_MOSI_HIGH();
		else
			AD9106_MOSI_LOW();
		AD9106_CLK_LOW();
		Delay(1);
		AD9106_CLK_HIGH();
		addr <<= 1;
		Delay(1);

	}

	AD91063_CS_HIGH();
	return data;

}

uint16_t ADS9106SpiReadRegS(uint16_t addr) {
	uint16_t data = 0;
	AD9106_CLK_HIGH();
	Delay(1);
	addr |= 0x8000; //reading
	AD9106_CS_LOW();

	for (int i = 0; i < 16; i++) {

		if (addr & 0x8000)
			AD9106_MOSI_HIGH();
		else
			AD9106_MOSI_LOW();
		AD9106_CLK_LOW();
		Delay(1);
		AD9106_CLK_HIGH();
		addr <<= 1;
		Delay(1);

	}

	for (int i = 0; i < 16; i++) {
		AD9106_CLK_LOW();
		Delay(1);
		AD9106_CLK_HIGH();
		data |= ((AD9106_MISO_GET_PIN()) << (15 - i));

		Delay(1);

	}
	AD91063_CS_HIGH();
	return data;

}

static bool writeReg(uint16_t regAddress, uint16_t dataBitMask, uint16_t data) {

	uint16_t tempVal;
	//if(!read(regAddress,&tempVal,1)) return false;   TODO , wrappers for SPI hardware implementation
	tempVal &= ~(dataBitMask);
	tempVal |= data << getShiftValue(dataBitMask);
	//return write(regAddress, &tempVal,1);
	return true;

}

/*
 static inline uint16_t getRegAddrForGivenDac(uint8_t nrOfDac, uint16_t reg ){
 if(nrOfDac>4||nrOfDac<1)return 0;

 #define GET_REG(reg,nrOfDac)    reg##nrOfDac

 return GET_REG(reg,nrOfDac) ;
 }
 */

/*bool writePatternToSram(uint8_t* dataBuf, uint16_t bufLength, uint16_t sramAddr){

 ADS7843SpiWriteReg(PAT_STATUS, MEM_ACCESS,0 );



 }*/

/*void playWaveformFromSram(uint8_t nrOfDac ,uint16_t startAddr, uint16_t stopAddr,uint16_t nrOfWaveCycles, uint16_t startDelay){
 if(nrOfDac>4||nrOfDac<1) return;

 #define  GET_REG(reg,nrOfDac)    reg##nrOfDac
 #define  GET_NR_OF_DAC(nrOfDac)   4

 ADS7843SpiWriteRegS((uint16_t)WAV4_3CONFIG,0x3000);Delay(1);
 Delay(1);
 ADS7843SpiWriteRegS((uint16_t) DDS_TW1, 0x1000); //frequency settings
 Delay(1);
 ADS7843SpiWriteRegS((uint16_t) DDS_CYC4, 0x0001);
 Delay(1);
 ADS7843SpiWriteRegS((uint16_t) DAC4_CST, 0xA200);
 Delay(1);
 ADS7843SpiWriteRegS((uint16_t) DAC4_DGAIN, 0x1000);//amplitude gain
 Delay(1);
 ADS7843SpiWriteRegS((uint16_t) DAC4RSET, 0x8002);
 Delay(1);
 ADS7843SpiWriteRegS((uint16_t) DACxRANGE, 0x00A0);
 Delay(1);
 ADS7843SpiWriteRegS((uint16_t) PAT_TIMEBASE, 0x0000);
 Delay(1);
 ADS7843SpiWriteRegS((uint16_t) PAT_PERIOD, 0x0100);
 Delay(1);
 ADS7843SpiWriteRegS((uint16_t) PAT_STATUS, 0x0001);
 Delay(1);

 ADS7843SpiWriteReg(PAT_STATUS, 0,RUN );
 //ADS7843SpiWriteRegS((uint16_t) RAMUPDATE, 0x0001);
 Delay(1);
 ADS7843SpiWriteReg(GET_REG(START_ADDR,4),START_ADDRx ,startAddr );  //TODO for now only 4 channel
 //ADS7843SpiWriteRegS((uint16_t) START_ADDRx, startAddr);
 Delay(1);
 ADS7843SpiWriteReg(GET_REG(STOP_ADDR,4),STOP_ADDRx ,stopAddr );
 //ADS7843SpiWriteRegS((uint16_t) STOP_ADDRx, stopAddr);
 Delay(1);
 ADS7843SpiWriteReg(GET_REG(DDS_CYC,4),DDS_CYCx ,nrOfWaveCycles );
 //ADS7843SpiWriteRegS((uint16_t) DDS_CYCx, nrOfWaveCycles);
 Delay(1);
 ADS7843SpiWriteReg(GET_REG(START_DLY,4),START_DELAYx  ,startDelay );
 //ADS7843SpiWriteRegS((uint16_t) START_DELAYx, startDelay);
 Delay(1);
 //ADS7843SpiWriteRegS((uint16_t) PAT_STATUS, 0x0001);
 ADS7843SpiWriteReg(PAT_STATUS, RUN,0 );
 Delay(1);
 ADS7843SpiWriteRegS((uint16_t) RAMUPDATE, 0x0001);
 Delay(1);
 }*/

void playWaveformFromSram(uint8_t nrOfDac, uint16_t startAddr,
		uint16_t stopAddr, uint16_t nrOfWaveCycles, uint16_t startDelay) {
	if (nrOfDac > 4 || nrOfDac < 1)
		return;

#define  GET_REG(reg,nrOfDac)    reg##nrOfDac
#define  GET_NR_OF_DAC(nrOfDac)   4
#if SPI_SW_ENABLED
	ADS9106SpiWriteRegS((uint16_t)WAV4_3CONFIG,0x3000);
	Delay(1);
	ADS9106SpiWriteRegS((uint16_t) DDS_TW1, 0x1000); //frequency settings i dont know if its working with arbitrary pattern
	Delay(1);
	ADS9106SpiWriteRegS((uint16_t) DAC4_CST, 0xA200);
	Delay(1);
	ADS9106SpiWriteRegS((uint16_t) DAC4_DGAIN, 0x5000);//amplitude gain
	Delay(1);
	ADS9106SpiWriteRegS((uint16_t) DAC4RSET, 0x8002);
	Delay(1);
	ADS9106SpiWriteRegS((uint16_t) DACxRANGE, 0x00A0);
	Delay(1);
	ADS9106SpiWriteRegS((uint16_t) PAT_TIMEBASE, 0x0FF1);//this register changes the time between samples
	Delay(1);
	ADS9106SpiWriteRegS((uint16_t) PAT_PERIOD, 0x8000);
	Delay(1);
	ADS9106SpiWriteRegS((uint16_t) PAT_TYPE, 0x0001);//pattern repeats DAC4_3PATx number of times
	Delay(1);
	ADS9106SpiWriteRegS((uint16_t) DAC4_3PATx, 0x0202);//pattern for DAC4 repeats 2 times
	Delay(1);

	ADS9106SpiWriteRegS((uint16_t)PAT_STATUS, 0 );
	Delay(1);
	ADS9106SpiWriteRegS((uint16_t)START_ADDR4 ,startAddr );//read register description!!! this value starts from 0 and is shifted left 4 places, the same with STOP_ADDR4
	Delay(1);
	ADS9106SpiWriteRegS((uint16_t)STOP_ADDR4 ,(1023<<4) );
	Delay(1);
	ADS9106SpiWriteRegS((uint16_t)START_DLY4 ,0x00 );
	Delay(1);
	ADS9106SpiWriteRegS((uint16_t)PAT_STATUS, 0x01 );
	Delay(1);
	ADS9106SpiWriteRegS((uint16_t) RAMUPDATE, 0x0001);
	Delay(1);
#else
	ADS7843SpiWriteReg((uint16_t) WAV4_3CONFIG, 0x3000, 0);
	ADS7843SpiWriteReg((uint16_t) DDS_TW1, 0x1000, 0); //frequency settings i dont know if its working with arbitrary pattern
	ADS7843SpiWriteReg((uint16_t) DAC4_CST, 0xA200, 0);
	ADS7843SpiWriteReg((uint16_t) DAC4_DGAIN, 0x5000, 0); //amplitude gain
	ADS7843SpiWriteReg((uint16_t) DAC4RSET, 0x8002, 0);
	ADS7843SpiWriteReg((uint16_t) DACxRANGE, 0x00A0, 0);
	ADS7843SpiWriteReg((uint16_t) PAT_TIMEBASE, 0x0FF1, 0); //this register changes the time between samples
	ADS7843SpiWriteReg((uint16_t) PAT_PERIOD, 0x8000, 0);
	ADS7843SpiWriteReg((uint16_t) PAT_TYPE, 0x0001, 0); //pattern repeats DAC4_3PATx number of times
	ADS7843SpiWriteReg((uint16_t) DAC4_3PATx, 0x0202, 0); //pattern for DAC4 repeats 2 times
	ADS7843SpiWriteReg((uint16_t) PAT_STATUS, 0, 0);
	ADS7843SpiWriteReg((uint16_t) START_ADDR4, startAddr, 0); //read register description!!! this value starts from 0 and is shifted left 4 places, the same with STOP_ADDR4
	ADS7843SpiWriteReg((uint16_t) STOP_ADDR4, (1023 << 4), 0);
	ADS7843SpiWriteReg((uint16_t) START_DLY4, 0x00, 0);
	ADS7843SpiWriteReg((uint16_t) PAT_STATUS, 0x01, 0);
	ADS7843SpiWriteReg((uint16_t) RAMUPDATE, 0x0001, 0);

#endif
}

void setSinus(void) {
#if SPI_SW_ENABLED
	ADS9106SpiWriteRegS((uint16_t)WAV4_3CONFIG,0x3100);Delay(1);
	Delay(1);
	ADS9106SpiWriteRegS((uint16_t) DDS_TW1, 0x1000); //frequency settings
	Delay(1);
	ADS9106SpiWriteRegS((uint16_t) DDS_CYC4, 0x0001);
	Delay(1);
	ADS9106SpiWriteRegS((uint16_t) DAC4_CST, 0xA200);
	Delay(1);
	ADS9106SpiWriteRegS((uint16_t) DAC4_DGAIN, 0x1000);//amplitude gain
	Delay(1);
	ADS9106SpiWriteRegS((uint16_t) DAC4RSET, 0x8002);
	Delay(1);
	ADS9106SpiWriteRegS((uint16_t) DACxRANGE, 0x00A0);
	Delay(1);
	ADS9106SpiWriteRegS((uint16_t) PAT_TIMEBASE, 0x0111);
	Delay(1);
	ADS9106SpiWriteRegS((uint16_t) PAT_PERIOD, 0x8000);
	Delay(1);
	ADS9106SpiWriteRegS((uint16_t) PAT_STATUS, 0x0001);
	Delay(1);
	ADS9106SpiWriteRegS((uint16_t) RAMUPDATE, 0x0001);
	Delay(1);
#else
	ADS7843SpiWriteReg(WAV4_3CONFIG, 0x3100, 0);
	ADS7843SpiWriteReg(DDS_TW1, 0x1000, 0); //frequency settings
	ADS7843SpiWriteReg(DDS_CYC4, 0x0001, 0);
	ADS7843SpiWriteReg(DAC4_CST, 0xA200, 0);
	ADS7843SpiWriteReg(DAC4_DGAIN, 0x1000, 0); //amplitude gain
	ADS7843SpiWriteReg(DAC4RSET, 0x8002, 0);
	ADS7843SpiWriteReg(DACxRANGE, 0x00A0, 0);
	ADS7843SpiWriteReg(PAT_TIMEBASE, 0x0111, 0);
	ADS7843SpiWriteReg(PAT_PERIOD, 0x8000, 0);
	ADS7843SpiWriteReg(PAT_STATUS, 0x0001, 0);
	ADS7843SpiWriteReg(RAMUPDATE, 0x0001, 0);
#endif
}

bool writePatternToSram(uint16_t* dataBuf, uint16_t bufLength,
		uint16_t sramAddr) {

	if (sramAddr < SRAMDATA)
		return false;
#if SPI_SW_ENABLED
	ADS9106SpiWriteRegS(PAT_STATUS, 0x04 );
	Delay(1);
	ADS9106SpiWriteRegS((uint16_t) RAMUPDATE, 0x0001);
	Delay(1);
#else
	ADS7843SpiWriteReg(PAT_STATUS, 0x04, 0);
	ADS7843SpiWriteReg(RAMUPDATE, 0x0001, 0);

#endif
	for (int i = 0; i < bufLength; i++) {
#if SPI_SW_ENABLED
		ADS9106SpiWriteRegS(sramAddr+i,*(dataBuf++) );
#else
		ADS7843SpiWriteReg(sramAddr + i, *(dataBuf++), 0);
#endif
	}
	return true;
}

void testPattern() {
	static uint8_t counter = 0;
	if (counter == 0) {
		writePatternToSram(sinLookupTable, sizeof(sinLookupTable), SRAMDATA);
		//writePatternToSram(sinLookupTable,0x1000,0x6000);
		counter = 1;
	}
	playWaveformFromSram(4, 0x0000, (sizeof(sinLookupTable) << 4), 10, 10);
}

void setNoise(void) {
#if SPI_SW_ENABLED
	ADS9106SpiWriteRegS((uint16_t)WAV4_3CONFIG,0x2100);Delay(1); //noise output test
	//ADS7843SpiWriteRegS((uint16_t) WAV4_3CONFIG, 0x1111);
	Delay(1);//sawtooth test

	ADS9106SpiWriteRegS((uint16_t) DAC4_CST, 0xA200);
	Delay(1);
	ADS9106SpiWriteRegS((uint16_t) DAC4_DGAIN, 0x1000);
	Delay(1);
	ADS9106SpiWriteRegS((uint16_t) DAC4RSET, 0x8002);
	Delay(1);
	ADS9106SpiWriteRegS((uint16_t) DACxRANGE, 0x00A0);
	Delay(1);
	ADS9106SpiWriteRegS((uint16_t) PAT_TIMEBASE, 0x0000);
	Delay(1);
	ADS9106SpiWriteRegS((uint16_t) PAT_PERIOD, 0x0100);
	Delay(1);
	ADS9106SpiWriteRegS((uint16_t) PAT_STATUS, 0x0001);
	Delay(1);
	ADS9106SpiWriteRegS((uint16_t) RAMUPDATE, 0x0001);
	Delay(1);

	//sawtooth configuration
	ADS9106SpiWriteRegS((uint16_t) SAW4_3CONFIG, 0x0808);
	Delay(1);

#else
	ADS7843SpiWriteReg(WAV4_3CONFIG, 0x2100, 0);	//noise output test
	ADS7843SpiWriteReg(DAC4_CST, 0xA200, 0);
	ADS7843SpiWriteReg(DAC4_DGAIN, 0xAAAA, 0);
	ADS7843SpiWriteReg(DAC4RSET, 0x8002, 0);
	ADS7843SpiWriteReg(DACxRANGE, 0x00A0, 0);
	ADS7843SpiWriteReg(PAT_TIMEBASE, 0x0000, 0);
	ADS7843SpiWriteReg(PAT_PERIOD, 0x0100, 0);
	ADS7843SpiWriteReg(PAT_STATUS, 0x0001, 0);
	ADS7843SpiWriteReg(RAMUPDATE, 0x0001, 0);
	//sawtooth configuration
	ADS7843SpiWriteReg(SAW4_3CONFIG, 0x0808, 0);
#endif
}

void setSaw(void) {
#if SPI_SW_ENABLED

	ADS9106SpiWriteRegS((uint16_t) WAV4_3CONFIG, 0x1111);
	Delay(1);	//sawtooth test

	ADS9106SpiWriteRegS((uint16_t) DAC4_CST, 0xA200);
	Delay(1);
	ADS9106SpiWriteRegS((uint16_t) DAC4_DGAIN, 0x1000);
	Delay(1);
	ADS9106SpiWriteRegS((uint16_t) DAC4RSET, 0x8002);
	Delay(1);
	ADS9106SpiWriteRegS((uint16_t) DACxRANGE, 0x00A0);
	Delay(1);
	ADS9106SpiWriteRegS((uint16_t) PAT_TIMEBASE, 0x0000);
	Delay(1);
	ADS9106SpiWriteRegS((uint16_t) PAT_PERIOD, 0x0100);
	Delay(1);
	ADS9106SpiWriteRegS((uint16_t) PAT_STATUS, 0x0001);
	Delay(1);
	ADS9106SpiWriteRegS((uint16_t) RAMUPDATE, 0x0001);
	Delay(1);

	//sawtooth configuration
	ADS9106SpiWriteRegS((uint16_t) SAW4_3CONFIG, 0x0808);
	Delay(1);

#else
	ADS7843SpiWriteReg(WAV4_3CONFIG, 0x1111, 0);	//noise output test
	ADS7843SpiWriteReg(DAC4_CST, 0xA200, 0);
	ADS7843SpiWriteReg(DAC4_DGAIN, 0x1000, 0);
	ADS7843SpiWriteReg(DAC4RSET, 0x8002, 0);
	ADS7843SpiWriteReg(DACxRANGE, 0x00A0, 0);
	ADS7843SpiWriteReg(PAT_TIMEBASE, 0x0000, 0);
	ADS7843SpiWriteReg(PAT_PERIOD, 0x0100, 0);
	ADS7843SpiWriteReg(PAT_STATUS, 0x0001, 0);
	ADS7843SpiWriteReg(RAMUPDATE, 0x0001, 0);
	//sawtooth configuration
	ADS7843SpiWriteReg(SAW4_3CONFIG, 0x0808, 0);
#endif
}

void AD9106Test(void) {
	static uint16_t counter = 0;
	counter += 1000;
	if (counter >= 30000)
		counter = 0;
	uint16_t i = 0;
#if !SPI_SW_ENABLED
	//setSinus();
	//testPattern();
	//setNoise();
	setSaw();
	Delay(2000);
	//Clock
	//ADS7843SpiWriteReg(CLOCKCONFIG, 0, 0);
	/*	i = ADS7843SpiReadReg(CLOCKCONFIG, 0);
	 i = ADS7843SpiReadReg(POWERCONFIG, 0);
	 SegmentLCD_Number(i);
	 ADS7843SpiWriteReg(POWERCONFIG, 1, DAC1_SLEEP);
	 */
#else //SPI_SW_ENABLED
	//i = ADS7843SpiReadRegS((uint16_t) PATTERN_DLY);
	//SegmentLCD_Number(i);
	Delay(1000);
	//i = ADS7843SpiReadRegS(DAC1RSET);
	//SegmentLCD_Number(i);
	Delay(300);

	setSinus();

	//testPattern();

	Delay(200);
	i = ADS9106SpiReadRegS((uint16_t) CFG_ERROR);
	//i = ADS7843SpiReadRegS((uint16_t) PAT_STATUS);
	//SegmentLCD_Number(i);
	Delay(200);
#endif
}

void AD9106TestType(uint8_t waveformType) {
	AD9106_TRIGGER_HIGH();
	switch (waveformType){

	case 0:
	setSinus();
	break;

	case 1:
	setNoise();

	case 2:
	setSaw();
	break;
	default:
	break;
	}
	AD9106_TRIGGER_LOW();
}

