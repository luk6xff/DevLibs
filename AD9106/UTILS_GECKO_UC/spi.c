#include <stdio.h>
#include "em_device.h"
#include "em_usart.h"
#include "em_gpio.h"
#include "em_cmu.h"
#include "spi.h"
#include "utils.h"



StatusTypeDef spiInit(SpiHandleTypeDef *hSpi) {

	if (hSpi == NULL ) {
		return STATUS_ERROR;
	}
	hSpi->spiState = SPI_STATE_BUSY;
	if (hSpi->spiModeHwSw) {  //true means we use Software SPI

		if (hSpi->spiGpioClockInit && hSpi->spiSwTransfer) {
			(*hSpi->spiGpioClockInit)();

			return STATUS_OK;
		} else {
			return STATUS_ERROR;
		}

	}

	//HW SPI
	if (hSpi->spiInstance == NULL || hSpi->init == NULL )
		return STATUS_ERROR;
	USART_InitSync(hSpi->spiInstance, hSpi->init); // depends on hardware
	if (hSpi->spiGpioClockInit) {
		(*hSpi->spiGpioClockInit)();
	} else {
		return STATUS_ERROR;
	}
	hSpi->spiState = SPI_STATE_READY;
	return STATUS_OK;
}

StatusTypeDef spiDeInit(SpiHandleTypeDef *hSpi) {

	if (hSpi == NULL ) {
		return STATUS_ERROR;
	}
	hSpi->spiState = SPI_STATE_BUSY;
	if (hSpi->spiModeHwSw) {  //true means we use Software SPI

		if (hSpi->spiGpioClockDeInit) {
			(*hSpi->spiGpioClockDeInit)();

			return STATUS_OK;
		} else {
			return STATUS_ERROR;
		}

	}
	//HW SPI
	if (hSpi->spiInstance == NULL || hSpi->init == NULL )
		return STATUS_ERROR;
	USART_Reset(hSpi->spiInstance); // depends on hardware
	if (hSpi->spiGpioClockDeInit) {
		(*hSpi->spiGpioClockDeInit)();
	} else {
		return STATUS_ERROR;
	}
	hSpi->spiState = SPI_STATE_RESET;

	return STATUS_OK;
}

StatusTypeDef spiTransmitReceive(SpiHandleTypeDef *hSpi, uint8_t *pTxData,
		uint8_t *pRxData, uint16_t size, uint32_t timeout) {

	(void) timeout; //TODO !

	uint32_t tempState = hSpi->spiState;

	if (tempState == SPI_STATE_READY) {

		if (pTxData == NULL || pRxData == NULL || size == 0)
			return STATUS_ERROR;

		//TODO mutex  locked on hspi !
		hSpi->pRxBuffPtr = pRxData;
		hSpi->rxXferSize = size;
		hSpi->rxXferCount = size;

		hSpi->pTxBuffPtr = pTxData;
		hSpi->txXferSize = size;
		hSpi->txXferCount = size;

		hSpi->RxISR = 0;  //not used
		hSpi->TxISR = 0; //not used
		//HARDWARE DEPENDING ON PART -- might add another level of abstraction : )
		//---------- EFM32 -----------
		//supports 8 and 16 bit modes
		if (!hSpi->spiModeHwSw) {
			hSpi->spiState = SPI_STATE_BUSY_TX_RX;
			if (hSpi->init->databits == usartDatabits16) {
				/* Check that transmit buffer is empty */
				/*while (!(hSpi->spiInstance->STATUS & USART_STATUS_TXBL))
					;
				hSpi->spiInstance->TXDOUBLE = *((uint16_t*) hSpi->pTxBuffPtr);
				hSpi->pTxBuffPtr += 2;
				hSpi->txXferCount--;
				if (hSpi->txXferCount == 0) {
					while (!(hSpi->spiInstance->STATUS & USART_STATUS_TXC));
					*((uint16_t*) hSpi->pRxBuffPtr) =
							hSpi->spiInstance->RXDOUBLE;
					hSpi->pRxBuffPtr += 2;
					hSpi->rxXferCount--;

				} else {
*/
					while (hSpi->txXferCount > 0) {
						while (!(hSpi->spiInstance->STATUS & USART_STATUS_TXBL))
							;
						hSpi->spiInstance->TXDOUBLE =
								*((uint16_t*) hSpi->pTxBuffPtr);
						hSpi->pTxBuffPtr += 2;
						hSpi->txXferCount--;

						while (!(hSpi->spiInstance->STATUS & USART_STATUS_TXC))
							;
						/*while (!(hSpi->spiInstance->STATUS
								& USART_STATUS_RXFULL))
							;*/
						*((uint16_t*) hSpi->pRxBuffPtr) =
								hSpi->spiInstance->RXDOUBLE;
						hSpi->pRxBuffPtr += 2;
						hSpi->rxXferCount--;
					}

				//}

			} else if (hSpi->init->databits == usartDatabits8) {
				while (!(hSpi->spiInstance->STATUS & USART_STATUS_TXBL))
					;
				hSpi->spiInstance->TXDATA = *(hSpi->pTxBuffPtr);
				hSpi->pTxBuffPtr++;
				hSpi->txXferCount--;
				if (hSpi->txXferCount == 0) {
					while (!(hSpi->spiInstance->STATUS & USART_STATUS_TXC))
						//-------
						;
					/*while (!(hSpi->spiInstance->STATUS & USART_STATUS_RXDATAV))
						;*/
					*((uint16_t*) hSpi->pRxBuffPtr) = hSpi->spiInstance->RXDATA;
					hSpi->pRxBuffPtr++;
					hSpi->rxXferCount--;

				} else {

					while (hSpi->txXferCount > 0) {
						//while (!(hSpi->spiInstance->STATUS & USART_STATUS_TXBL))
						//	;
						hSpi->spiInstance->TXDATA = *(hSpi->pTxBuffPtr);
						hSpi->pTxBuffPtr++;
						hSpi->txXferCount--;

						while (!(hSpi->spiInstance->STATUS & USART_STATUS_TXC))
							;
						/*while (!(hSpi->spiInstance->STATUS
								& USART_STATUS_RXDATAV))
							;
							*/
						*(hSpi->pRxBuffPtr) = hSpi->spiInstance->RXDATA;
						hSpi->pRxBuffPtr++;
						hSpi->rxXferCount--;
					}

				}
			}

			else {
				hSpi->spiState = SPI_STATE_READY;
				return STATUS_ERROR;
			}
		} else { //Software version

			//*(hSpi->spiSwTransfer)(hSpi);

		}
	} else
		return STATUS_BUSY;
	hSpi->spiState = SPI_STATE_READY;
	return STATUS_OK;
}

