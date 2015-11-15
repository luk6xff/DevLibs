/*
 * AD9106.h
 *
 *@brief AD9106 Quad, Low Power, 12-Bit, 180 MSPS, Digital-toAnalog - Converter and Waveform Generator - SPI breakout library   

  @Author lukasz uszko(luszko@op.pl)

  Tested on Silicon Labs Arm microcontrolers
  
  Copyright (c) 2014 lukasz uszko
  Released under the MIT License (see http://mbed.org/license/mit)

  Documentation regarding the AD9106 might be found here: 
  http://www.analog.com/en/digital-to-analog-converters/high-speed-da-converters/ad9106/products/product.html?src=ad9106.pdf
 */

#ifndef AD9106_H_
#define AD9106_H_

#include "em_usart.h"
#include "spi.h"

/*********************************Hardware dependent part*****************************************/
/*********************************Hardware dependent part*****************************************/

/* USART used for SPI access */
#define USART_USED        USART2
#define USART_CLK         cmuClock_USART2

/* GPIO pins used for SPI communication. */
/*DEFAULT PINOUT USED FOR HARDWARE SPI (USART 2, Location #0)
 *
 US2_CLK-- PC4
 US2_CS -- PC5
 US2_RX -- PC3
 US2_TX -- PC2
 *
 *  */
//------------------CS----------------------
#define AD9106_PIN_CS           5
#define AD9106_PORT_CS          gpioPortD//gpioPortC
#define AD9106_CS_OUTPUT() 		GPIO_PinModeSet(AD9106_PORT_CS, AD9106_PIN_CS, gpioModePushPull, 1)
#define AD91063_CS_HIGH()   	GPIO_PinOutSet(AD9106_PORT_CS , AD9106_PIN_CS )
#define AD9106_CS_LOW()    		GPIO_PinOutClear(AD9106_PORT_CS , AD9106_PIN_CS )
#define AD9106_CS_DISABLED()    GPIO_PinModeSet(AD9106_PORT_CS, AD9106_PIN_CS, gpioModeDisabled, 0)

//------------------CLK----------------------
#define AD9106_PIN_CLK         	4
#define AD9106_PORT_CLK        	gpioPortC
#define AD9106_CLK_OUTPUT() 	GPIO_PinModeSet(AD9106_PORT_CLK, AD9106_PIN_CLK, gpioModePushPull, 1)
#define AD9106_CLK_HIGH()   	GPIO_PinOutSet(AD9106_PORT_CLK , AD9106_PIN_CLK )
#define AD9106_CLK_LOW()    	GPIO_PinOutClear(AD9106_PORT_CLK , AD9106_PIN_CLK )
#define AD9106_CLK_DISABLED()    GPIO_PinModeSet(AD9106_PORT_CLK, AD9106_PIN_CLK, gpioModeDisabled, 0)
//------------------MOSI---------------------
#define AD9106_PIN_MOSI        	2
#define AD9106_PORT_MOSI       	gpioPortC
#define AD9106_MOSI_OUTPUT() 	GPIO_PinModeSet(AD9106_PORT_MOSI,AD9106_PIN_MOSI,gpioModePushPull, 1)
#define AD9106_MOSI_HIGH()   	GPIO_PinOutSet(AD9106_PORT_MOSI, AD9106_PIN_MOSI )
#define AD9106_MOSI_LOW()    	GPIO_PinOutClear(AD9106_PORT_MOSI , AD9106_PIN_MOSI )
#define AD9106_MOSI_DISABLED()  GPIO_PinModeSet(AD9106_PORT_MOSI, AD9106_PIN_MOSI, gpioModeDisabled, 0)

//------------------MIS0---------------------
#define AD9106_PIN_MISO        	3
#define AD9106_PORT_MISO      	gpioPortC
#define AD9106_MISO_INPUT() 	GPIO_PinModeSet(AD9106_PORT_MISO, AD9106_PIN_MISO, gpioModeInput, 0)
#define AD9106_MISO_HIGH()   	GPIO_PinOutSet(AD9106_PORT_MISO , AD9106_PIN_MISO )
#define AD9106_MISO_LOW()    	GPIO_PinOutClear(AD9106_PORT_MISO , AD9106_PIN_MISO )
#define AD9106_MISO_GET_PIN()     GPIO_PinInGet(AD9106_PORT_MISO, AD9106_PIN_MISO)
#define AD9106_MISO_DISABLED()  GPIO_PinModeSet(AD9106_PORT_MISO, AD9106_PIN_MISO, gpioModeDisabled, 0)

//------------------TRIGGER PIN---------------------
#define AD9106_PIN_TRIGGER        	7
#define AD9106_PORT_TRIGGER      	gpioPortD
#define AD9106_TRIGGER_OUTPUT() 	GPIO_PinModeSet(AD9106_PORT_TRIGGER,AD9106_PIN_TRIGGER,gpioModePushPull, 1)
#define AD9106_TRIGGER_HIGH()   	GPIO_PinOutSet(AD9106_PORT_TRIGGER , AD9106_PIN_TRIGGER )
#define AD9106_TRIGGER_LOW()    	GPIO_PinOutClear(AD9106_PORT_TRIGGER , AD9106_PIN_TRIGGER )
#define AD9106_TRIGGER_GET_PIN()    GPIO_PinInGet(AD9106_PORT_TRIGGER, AD9106_PIN_TRIGGER)
#define AD9106_TRIGGER_DISABLED()  	GPIO_PinModeSet(AD9106_PORT_TRIGGER, AD9106_PIN_TRIGGER, gpioModeDisabled, 0)
/*********************************Hardware dependent part - END*****************************************/

/*hardware SPI*/

/*Software SPI*/
void AD9106spiInitSoftware(void);
uint16_t ADS9106SpiWriteRegS(uint16_t addr, uint16_t data);
uint16_t ADS9106SpiReadRegS(uint16_t addr); //

typedef enum {
	SPICONFIG_ADDR = 0x00,
	POWERCONFIG,
	CLOCKCONFIG,
	REFADJ,
	DAC4AGAIN,
	DAC3AGAIN,
	DAC2AGAIN,
	DAC1AGAIN,
	DACxRANGE,
	DAC4RSET,
	DAC3RSET,
	DAC2RSET,
	DAC1RSET,
	CALCONFIG,
	COMPOFFSET = 0x000E,
	RAMUPDATE = 0x001D,
	PAT_STATUS,
	PAT_TYPE,
	PATTERN_DLY = 0x0020,
	DAC4DOF = 0x0022,
	DAC3DOF,
	DAC2DOF,
	DAC1DOF,
	WAV4_3CONFIG,
	WAV2_1CONFIG,
	PAT_TIMEBASE,
	PAT_PERIOD = 0x0029,
	DAC4_3PATx,
	DAC2_1PATx,
	DOUT_START_DLY,
	DOUT_CONFIG,
	DAC4_CST,
	DAC3_CST,
	DAC2_CST,
	DAC1_CST,
	DAC4_DGAIN,
	DAC3_DGAIN,
	DAC2_DGAIN,
	DAC1_DGAIN,
	SAW4_3CONFIG,
	SAW2_1CONFIG,
	DDS_TW32 = 0x3E,
	DDS_TW1,
	DDS4_PW,
	DDS3_PW,
	DDS2_PW,
	DDS1_PW,
	TRIG_TW_SEL,
	DDSx_CONFIG,
	TW_RAM_CONFIG = 0x47,
	START_DLY4 = 0x50,
	START_ADDR4,
	STOP_ADDR4,
	DDS_CYC4,
	START_DLY3,
	START_ADDR3,
	STOP_ADDR3,
	DDS_CYC3,
	START_DLY2,
	START_ADDR2,
	STOP_ADDR2,
	DDS_CYC2,
	START_DLY1,
	START_ADDR1,
	STOP_ADDR1,
	DDS_CYC1,
	CFG_ERROR,
	SRAMDATA = 0x6000,   //0x6000 to 0x6FFF

} AD9106RegAddress;

//Proper BITMASKS for each register, allows us to write/ read to/from device in easy way , not overwriting whole register
typedef enum {

	LSBFIRST = 0x8000, //LSB first selection. {{ 0- MSB first per SPI standard (default). 1- LSB first per SPI standard.
	SPI3WIRE = 0x4000, //Selects if SPI is using 3-wire or 4-wire interface. {{0 - 4-wire SPI.  1- 3-wire SPI.
	RESET = 0x2000, //Executes software reset of SPI and controllers, reloads default register values, except for Register 0x00.
	DOUBLESPI = 0x1000, //Double SPI data line.
	SPI_DRV = 0x0800,   //Double-drive ability for SPI output.
	DOUT_EN = 0x0400,   //Enables DOUT signal on SDO/SDI2/DOUT pin.
	DOUT_ENM = 0x0020,  //Enable DOUT signal on SDO/SDI2/DOUT pin.
	SPI_DRVM = 0x0010,  //Double-drive ability for SPI output.
	DOUBLESPIM = 0x0008,  //Double SPI data line.
	RESETM = 0x0004, //Executes software reset of SPI and controllers, reloads default register values, except for Register 0x00.
	SPI3WIREM = 0x0002, //Selects if SPI is using 3-wire or 4-wire interface.
	LSBFIRSTM = 0x0001  //LSB first selection.
} SPICONFIG_Reg;
/*NOTE!!!SPICONFIG[10:15] should always be set to the mirror of SPICONFIG[5:0] to allow easy recovery of the SPI operation when
 * the LSBFIRST bit is set incorrectly. Bit[15] = Bit[0], Bit[14] = Bit[1], Bit[13] = Bit[2], Bit[12] = Bit[3], Bit[11] = Bit[4] and Bit[10] = Bit[5].*/

typedef enum {

	CLK_LDO_STAT = 0x0800,  //Read only flag indicating CLKVDD_1P8 LDO is on.
	DIG1_LDO_STAT = 0x0400, //Read only flag indicating DVDD1 LDO is on.
	DIG2_LDO_STAT = 0x0200, //Read only flag indicating DVDD2 LDO is on.
	PDN_LDO_CLK = 0x0100, //Disables the CLKVDD_1P8 LDO. An external supply is required.
	PDN_LDO_DIG1 = 0x0080, //Disables the DVDD1 LDO. An external supply is required.
	PDN_LDO_DIG2 = 0x0040, //Disables the DVDD2 LDO. An external supply is required.
	REF_PDN = 0x0020, //Disables 10 kOm resistor that creates REFIO voltage. User can drive with external voltage or provide external BG resistor.
	REF_EXT = 0x0010,		//Power down main BG reference including DAC bias.
	DAC1_SLEEP = 0x0008,	//Disables DAC1 output current.
	DAC2_SLEEP = 0x0004,	//Disables DAC2 output current.
	DAC3_SLEEP = 0x0002,	//Disables DAC3 output current.
	DAC4_SLEEP = 0x0001		//Disables DAC4 output current.
} POWERCONFIG_Reg;

typedef enum {

	DIS_CLK1 = 0x0800,
	DIS_CLK2 = 0x0400,
	DIS_CLK3 = 0x0200,
	DIS_CLK4 = 0x0100,
	DIS_DCLK = 0x0080,
	CLK_SLEEP = 0x0040,
	CLK_PDN = 0x0020,
	EPS = 0x0010,
	DAC1_INV_CLK = 0x0008,
	DAC2_INV_CLK = 0x0004,
	DAC3_INV_CLK = 0x0002,
	DAC4_INV_CLK = 0x0001
} CLOCKCONFIG_Reg;

typedef enum {
	BGDR = 0x003F
} REFADJ_Reg;

typedef enum {
	DACx_GAIN_CAL = 0x7F00,  //Read only
	DACx_GAIN = 0x007F
} DACxAGAIN_Reg;

typedef enum {
	DAC4_GAIN_RNG = 0xC0,
	DAC3_GAIN_RNG = 0x30,
	DAC2_GAIN_RNG = 0x0C,
	DAC1_GAIN_RNG = 0x03
} DACxRANGE_Reg;

typedef enum {
	DACx_RSET_EN = 0x8000, DACx_RSET_CAL = 0x1F00, DACx_RSET = 0x1F
} DACxRSET_Reg;

typedef enum {

	COMP_OFFSET_OF = 0x4000,   //Read only
	COMP_OFFSET_UF = 0x2000,   //Read only
	RSET_CAL_OF = 0x1000,      //Read only
	RSET_CAL_UF = 0x0800,      //Read only
	GAIN_CAL_OF = 0x0400,      //Read only
	GAIN_CAL_UF = 0x0200,      //Read only
	CAL_RESET = 0x0100,
	CAL_MODE = 0x0080,         //Read only
	CAL_MODE_EN = 0x0040,
	COMP_CAL_RNG = 0x0030,
	CAL_CLK_EN = 0x0008,
	CAL_CLK_DIV = 0x0007
} CALCONFIG_Reg;

typedef enum {

	COMP_OFFSET_CAL = 0x7F00, //Read only
	CAL_FIN = 0x0002,         //Read only
	START_CAL = 0x0001
} COMPOFFSET_Reg;

typedef enum {
	RAMPUPDATE = 0x0001
} RAMUPDATE_Reg;

typedef enum {

	BUF_READ = 0x0008, MEM_ACCESS = 0x0004, PATTERN = 0x0002,        //Read only
	RUN = 0x0001
} PAT_STATUS_Reg;

typedef enum {
	PATTERN_RPT = 0x0001
} PAT_TYPE_Reg;

typedef enum {
	PATTERN_DELAY = 0xFFFF
} PATTERN_DLY_Reg;

typedef enum {
	DACx_DIG_OFFSET = 0xFFF0
} DACxDOF_Reg;

typedef enum {
	PRESTORE_SEL4 = 0x3000,
	WAVE_SEL4 = 0x0300,
	PRESTORE_SEL3 = 0x30,
	WAVE_SEL3 = 0x03,
} WAV4_3CONFIG_Reg;

typedef enum {
	PRESTORE_SEL2 = 0x3000,
	MASK_DAC4 = 0x0800,
	CH2_ADD = 0x0400,
	WAVE_SEL2 = 0x0300,
	PRESTORE_SEL1 = 0x30,
	MASK_DAC3 = 0x08,
	CH1_ADD = 0x04,
	WAVE_SEL1 = 0x03,
} WAV2_1CONFIG_Reg;

typedef enum {
	HOLD = 0x0F00, PAT_PERIOD_BASE = 0xF0, START_DELAY_BASE = 0x0F,
} PAT_TIMEBASE_Reg;

typedef enum {
	PATTERN_PERIOD = 0xFFFF
} PAT_PERIOD_Reg;

typedef enum {
	DAC4_REPEAT_CYCLE = 0xFF00, DAC3_REPEAT_CYCLE = 0xFF
} DAC4_3PATx_Reg;

typedef enum {
	DAC2_REPEAT_CYCLE = 0xFF00, DAC1_REPEAT_CYCLE = 0xFF
} DAC2_1PATx_Reg;

typedef enum {
	DOUT_START = 0xFFFF,
} DOUT_START_DLY_Reg;

typedef enum {
	DOUT_VAL = 0x20, DOUT_MODE = 0x10, DOUT_STOP = 0x0F
} DOUT_CONFIG_Reg;

typedef enum {
	DACx_CONST = 0xFFF0
} DACx_CST_Reg;

typedef enum {
	DACx_DIG_GAIN = 0xFFF0
} DACx_DGAIN_Reg;

typedef enum {
	SAW_STEP4 = 0xFC00,  //Number of samples per step for DAC4.
	SAW_TYPE4 = 0x0300, //0- Ramp up saw wave. 1- Ramp down saw wave. 2- Triangle saw wave. 3- No wave, zero.
	SAW_STEP3 = 0x00FC,
	SAW_TYPE3 = 0x0003
} SAW4_3CONFIG_Reg;

typedef enum {
	SAW_STEP2 = 0xFC00,  //Number of samples per step for DAC2.
	SAW_TYPE2 = 0x0300, //0- Ramp up saw wave. 1- Ramp down saw wave. 2- Triangle saw wave. 3- No wave, zero.
	SAW_STEP1 = 0x00FC,
	SAW_TYPE1 = 0x0003
} SAW2_1CONFIG_Reg;

///////////////
typedef enum {
	DDSTW_MSB = 0xFFFF  //DDS tuning word MSB.
} DDS_TW32_Reg;

typedef enum {
	DDSTW_LSB = 0xFF00  //DDS tuning word LSB.
} DDS_TW1_Reg;

typedef enum {
	DDSx_PHASE = 0xFFFF //DDSx phase offset.
} DDSx_PW_Reg;

typedef enum {
	TRIG_DELAY_EN = 0x0002 //Enable start delay as trigger delay for all four channels.
	/* Settings
	 0 Delay repeats for all patterns.
	 1 Delay is only at the start of first pattern.
	 */
} TRIG_TW_SEL_Reg;

typedef enum {
	DDS_COS_EN4 = 0x8000, //Enable DDS4 cosine output of DDS instead of sine wave.
	DDS_MSB_EN4 = 0x4000, //Enable the clock for the RAM address. Increment is coming from the DDS4 MSB. Default is coming from DAC clock.
	DDS_COS_EN3 = 0x0800, //Enable DDS3 cosine output of DDS instead of sine wave.
	DDS_MSB_EN3 = 0x0400, //Enable the clock for the RAM address. Increment is coming from the DDS3 MSB. Default is coming from DAC clock.
	PHASE_MEM_EN3 = 0x0200, //Enable DDS3 phase offset input coming from RAM reading START_ADDR3. Since phase word is 8 bits and RAM data is 14 bits, only 8 MSB of RAM are taken into account. Default is coming from SPI map, DDS3_PHASE.
	DDS_COS_EN2 = 0x0080, //Enable DDS2 cosine output of DDS instead of sine wave.
	DDS_MSB_EN2 = 0x0040, //Enable the clock for the RAM address. Increment is coming from the DDS2 MSB. Default is coming from DAC clock.
	DDS_MSB_EN1 = 0x0004, //Enable the clock for the RAM address. Increment is coming from the DDS1 MSB. Default is coming from DAC clock.
	TW_MEM_EN = 0x0001 //Enable DDS tuning word input coming from RAM reading using START_ADDR1. Since tuning word is 24 bits and RAM data is 14 bits, 10 bits are set to 0s depending on the value of the TW_MEM_SHIFT bits in the TW_RAM_CONFIG register. Default is coming from SPI map, DDSTW.
} DDSx_CONFIG_Reg;

typedef enum {
	TW_MEM_SHIFT = 0x000F
//settings NOTE! TW_MEM_EN1 must be set = 1 to use this bit field.
/*
 0x00   DDS1TW = {RAM[11:0],12'b0}
 0x01   DDS1TW = {DDS1TW[23],RAM[11:0],11'b0}
 0x02	DDS1TW = {DDS1TW[23:22],RAM[11:0],10'b0}
 0x03	DDS1TW = {DDS1TW[23:21],RAM[11:0],9'b0}
 0x04	DDS1TW = {DDS1TW[23:20],RAM[11:0],8'b0}
 0x05	DDS1TW = {DDS1TW[23:19],RAM[11:0],7'b0}
 0x06	DDS1TW = {DDS1TW[23:18],RAM[11:0],6'b0}
 0x07	DDS1TW = {DDS1TW[23:17],RAM[11:0],5'b0}
 0x08	DDS1TW = {DDS1TW[23:16],RAM[11:0],3'b0}
 0x09	DDS1TW = {DDS1TW[23:15],RAM[11:0],4'b0}
 0x0A	DDS1TW = {DDS1TW[23:14],RAM[11:0],2’b0}
 0x0B	DDS1TW = {DDS1TW[23:13],RAM[11:0],1’b0}
 0x0C	DDS1TW = {DDS1TW[23:12],RAM[11:0]}
 0x0D	DDS1TW = {DDS1TW[23:11],RAM[11:1]}
 0x0E	DDS1TW = {DDS1TW[23:10],RAM[11:2]}
 0x0F	DDS1TW = {DDS1TW[23:9],RAM[11:3]}
 0x10	DDS1TW = {DDS1TW[23:8],RAM[11:4]}
 x Reserved
 */
} TW_RAM_CONFIG_Reg;

typedef enum {
	START_DELAYx = 0xFFFF,  //Start delay of DACx.
} START_DLYx_REG;

typedef enum {
	START_ADDRx = 0xFFF0,  //RAM address where DACx starts to read waveform.
} START_ADDRx_Reg;

typedef enum {
	STOP_ADDRx = 0xFFF0,  //RAM address where DACx stops to read waveform.
} STOP_ADDRx_Reg;

typedef enum {
	DDS_CYCx = 0xFFFF // Number of sine wave cycles when DDS prestored waveform with start and stop delays is selected for DACx output.
} DDS_CYCx_Reg;

typedef enum {
	ERROR_CLEAR = 0x8000,  		  //Writing this bit clears all errors.
	CFG_ERROR_ = 0x7FC0,
	DOUT_START_LG_ERR = 0x0020, //When DOUT_START is larger than pattern delay, this error is toggled.  /R only
	PAT_DLY_SHORT_ERR = 0x0010, //When pattern delay value is smaller than default value, this error is toggled. /R only
	DOUT_START_SHORT_ERR = 0x0008, //When DOUT_START value is smaller than default value, this error is toggled. /R only
	PERIOD_SHORT_ERR = 0x0004, //When period register setting value is smaller than pattern play cycle, this error is toggled. /R only
	ODD_ADDR_ERR = 0x0002, //When memory pattern play is not even in length in trigger delay mode, this error flag is toggled. /R only
	MEM_READ_ERR = 0x0001 //When there is a memory read conflict, this error flag is toggled. /R only
} CFG_ERROR_Reg;

/***************************************************************************************************************************************************/
//public methods
/** Helper method for writeReg and readReg methods
 * @param uint16_t dataBitMask
 * @returns  ShiftValue
 */

void AD9106Test(void);
void AD9106Init(void);
void AD9106TestType(uint8_t waveformType);

//SRAM methods
bool writePatternToSram(uint16_t* dataBuf, uint16_t bufLength,
		uint16_t sramAddr);
void playWaveformFromSram(uint8_t nrOfDac, uint16_t startAddr,
		uint16_t stopAddr, uint16_t nrOfWaveCycles, uint16_t startDelay);

#endif /* AD9106_H_ */
