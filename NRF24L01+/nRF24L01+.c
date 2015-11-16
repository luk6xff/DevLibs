/**
 * 07.07.2014
 */


#include "nRF24L01+.h"
#include  "stddef.h"
#include "stm32_hardware_modules.h"

/**
 * @name
 * @brief Callbacks
 * @retval
 *
 */
static void (*SPI_WriteDataCallback)(uint8_t* data);
static void (*SPI_ReadDataCallback)(uint8_t* data);
static void (*CE_PinSetCallback)(void);
static void (*CE_PinResetCallback)(void);
static void (*IRQ_PinSetupCallback)(void);

bool nRF24L01_RegisterCallbacks(void (*SPI_ReadData)(uint8_t* data),
		void (*SPI_WriteData)(uint8_t* data), void (*CE_Pin_Set)(void),
		void (*CE_Pin_Reset)(void), void (*IRQ_PinSetup)(void)) {

	SPI_WriteDataCallback = SPI_WriteData;
	SPI_ReadDataCallback = SPI_ReadData;
	CE_PinSetCallback = CE_Pin_Set;
	CE_PinResetCallback = CE_Pin_Reset;
	IRQ_PinSetupCallback = IRQ_PinSetup; // it can be assigned to NULL , if that PIN is not used

	if ((SPI_WriteDataCallback != NULL) && (SPI_ReadDataCallback != NULL)
			&& (CE_PinSetCallback != NULL) && (CE_PinResetCallback != NULL)) {
		return true;
	}
	return false;
}
/**
 * @name
 * @brief writes or reads data to/from the nRF24L01+ device
 * @retval
 *
 */

void nRF24L01_SPI_COMMAND(uint8_t* regData, int nbrOfBytesToReadWrite,
		_SPI_REGISTER_ADRESS_TypeDef regAdress, _SPI_COMMAND_TypeDef command) {
	uint8_t tempBuf[nbrOfBytesToReadWrite + 2]; // +2 more dimension for numberOfBytes to read/write and command+address
	switch (command) {

	case READ:
		tempBuf[0] = nbrOfBytesToReadWrite + 1;
		; // number of bytes to be read , only Status is read , but this amount is used as a dummy data to read
		tempBuf[1] = command | regAdress;
		(*SPI_ReadDataCallback)(tempBuf);
		break;
	case WRITE:
		tempBuf[0] = nbrOfBytesToReadWrite + 1; // number of bytes to be write
		tempBuf[1] = command | regAdress;
		memcpy(&tempBuf[2], regData, nbrOfBytesToReadWrite);
		(*SPI_WriteDataCallback)(tempBuf);
		break;
	case READ_PAYLOAD:
		tempBuf[0] = nbrOfBytesToReadWrite; // number of bytes to be write+commmand
		tempBuf[1] = command;
		(*SPI_ReadDataCallback)(tempBuf);
		break;
	case WRITE_PAYLOAD:
		tempBuf[0] = nbrOfBytesToReadWrite+1; // number of bytes to be write
		tempBuf[1] = command;
		memcpy(&tempBuf[2], regData, nbrOfBytesToReadWrite);
		(*SPI_WriteDataCallback)(tempBuf);
		break;
	case FLUSH__TX:
		tempBuf[0] = 1; // number of bytes to be write
		tempBuf[1] = command;
		(*SPI_WriteDataCallback)(tempBuf);
		break;
	case FLUSH__RX:
		tempBuf[0] = 1; // number of bytes to be write
		tempBuf[1] = command;
		(*SPI_WriteDataCallback)(tempBuf);
		break;
	case REUSE_LAST_TX_PAYLOAD:
		break;
	case READ_RX_PAYLOAD_WIDTH:
		break;
	case WRITE_PAYLOAD_WITH_ACK:
		break;
	case DISABLE_AUTO_ACK:
		break;
	case DO_NOTHING:
		tempBuf[0] = 1;
		tempBuf[1] = command;
		(*SPI_WriteDataCallback)(tempBuf);
		break;

	default:
		break;
	}

}
/**
 * @name void nRF24L01_SetTxMode(void)
 * @brief
 * @retval
 *
 */
void nRF24L01_SetTxMode(void) {
	uint8_t data = /*CONFIG_PWR_UP | */(CONFIG_PRIM_RX & 0);
	nRF24L01_SPI_COMMAND(&data, sizeof(data), CONFIG_regAdr, WRITE);
	nRF24L01_SetCE_PIN();
}
/**
 * @name nRF24L01_SetRxMode(void)
 * @brief
 * @retval
 *
 */
void nRF24L01_SetRxMode(void) {
	uint8_t data = CONFIG_PWR_UP | CONFIG_PRIM_RX;
	nRF24L01_SPI_COMMAND(&data, sizeof(data), CONFIG_regAdr, WRITE);
	nRF24L01_SetCE_PIN();
}

/**
 * @name nRF24L01_PowerUp(void)
 * @brief
 * @retval
 *
 */
void nRF24L01_PowerUp(void) {
	uint8_t data = CONFIG_PWR_UP;
	nRF24L01_SPI_COMMAND(&data, sizeof(data), CONFIG_regAdr, WRITE);

}

/**
 * @name nRF24L01_PowerDown(void)
 * @brief
 * @retval
 *
 */
void nRF24L01_PowerDown(void) {
	uint8_t data = CONFIG_PWR_UP & 0;
	nRF24L01_SPI_COMMAND(&data, sizeof(data), CONFIG_regAdr, WRITE);

}
/**
 * @name nRF24L01_SetCE_PIN(void)
 * @brief
 * @retval
 *
 */
void nRF24L01_SetCE_PIN(void) {
	if (CE_PinSetCallback != NULL)
		(*CE_PinSetCallback)();
}
/**
 * @name nRF24L01_ResetCE_PIN(void)
 * @brief
 * @retval
 *
 */
void nRF24L01_ResetCE_PIN(void) {

	if (CE_PinResetCallback != NULL)
		(*CE_PinResetCallback)();

}

/**
 * @name nRF24L01_NOP(void)
 * @brief
 * @retval
 *
 */
void nRF24L01_NOP(void) {
	nRF24L01_SPI_COMMAND(NULL, 0, 0, DO_NOTHING);

}

/**
 * @name nRF24L01_ReadSTATUS(void)
 * @brief
 * @retval
 *
 */
void nRF24L01_ReadSTATUS(void) {
	nRF24L01_SPI_COMMAND(NULL, 1, STATUS_regAdr, READ);
}

/**
 * @name  nRF24L01_FlushSTATUSReg((void)
 * @brief
 * @retval
 *
 */
void nRF24L01_FlushSTATUSReg(void) {
	uint8_t data = 0xFF;
	nRF24L01_SPI_COMMAND(&data, 1, STATUS_regAdr, WRITE);
}



/**
 * @name nRF24L01_ReadCONFIGReg(void)
 * @brief
 * @retval
 *
 */
void nRF24L01_ReadCONFIGReg(void) {
	nRF24L01_SPI_COMMAND(NULL, 1, CONFIG_regAdr, READ);
}

/**
 * @name  nRF24L01_SendDataToFifo(uint8_t*data)
 * @brief
 * @retval
 *
 */
void nRF24L01_SendDataToFifo(uint8_t*data) {
	//nRF24L01_SetTxMode();   i set up it in the init function

	nRF24L01_SPI_COMMAND(data, NUMBER_OF_BYTES_IN_FIFO, 0, WRITE_PAYLOAD);
	nRF24L01_SetCE_PIN();
	//nRF24L01_ResetCE_PIN();
}

/**
 * @name  void nRF24L01_FlushTx(void)
 * @brief
 * @retval
 *
 */
void nRF24L01_FlushTx(void) {
	nRF24L01_SPI_COMMAND(NULL, 1, 0, FLUSH__TX);
}

/**
 * @name  void nRF24L01_FlushRx(void)
 * @brief
 * @retval
 *
 */
void nRF24L01_FlushRx(void) {
	nRF24L01_SPI_COMMAND(NULL, 1, 0, FLUSH__RX);
}




/**
 * @name  void nRF24L01_Init(void)
 * @brief
 * @retval
 *
 */
void nRF24L01_Init(void) {
	if (IRQ_PinSetupCallback != NULL) {
		(*IRQ_PinSetupCallback)();
	}
	nRF24L01_ResetCE_PIN(); // setup Tx MODE
	nRF24L01 obj;
	obj.CONFIG.PWR_UP = SET;
	obj.CONFIG.MASK_MAX_RT = SET;
	obj.CONFIG.MASK_RX_DR = SET;
	obj.CONFIG.MASK_TX_DS = SET;

	obj.CONFIG.PRIM_RX=RESET; // Tx Mode

	obj.EN_AA.ENAA_P0 = RESET;
	obj.EN_RXADDR.ERX_P0 = SET;
	//obj.RF_SETUP.RF_PWR = _0dBm;
	//obj.RF_SETUP.RF_DR_HIGH = SET;

	obj.RF_CH.RF_CH=40;
	obj.RX_PW_P0.RX_PW_P0 = _10_Bytes;

	nRF24L01_SPI_COMMAND((uint8_t*) &obj.CONFIG, 1, CONFIG_regAdr, WRITE);
	nRF24L01_SPI_COMMAND((uint8_t*) &obj.EN_AA, 1, EN_AA_ADR_regAdr, WRITE);
	nRF24L01_SPI_COMMAND((uint8_t*) &obj.EN_RXADDR, 1, EN_RXADDR_regAdr, WRITE);
	//nRF24L01_SPI_COMMAND((uint8_t*) &obj.RF_SETUP, 1, RF_SETUP_regAdr, WRITE);
	nRF24L01_SPI_COMMAND((uint8_t*) &obj.RX_PW_P0, 1, RX_PW_P0_regAdr, WRITE);
	nRF24L01_SetCE_PIN(); // setup Tx MODE
}

