/*
 * utils.h
 *
 *  Created on: 01-03-2015
 *      Author: lukasz
 */

#ifndef UTILS_H_
#define UTILS_H_


#include <stdint.h>

/**************************************************************************//**
 * @brief Delays number of msTick Systicks ( 1 ms)
 * @param dlyTicks Number of ticks to delay
 *****************************************************************************/
void utilsInit(void);
void Delay(uint32_t dlyTicks);
void USB_DEBUG_PUTS(char *s);

#endif /* UTILS_H_ */
