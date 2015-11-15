/*
 * utils.c
 *
 *  Created on: 01-03-2015
 *      Author: lukasz
 */
#include "utils.h"
#include "em_system.h"
#include "em_timer.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "bsp.h"
#include <string.h>
#include "../drivers/usb/cdc.h"

#define USB_DEBUG_ENABLED 1

volatile uint32_t msTicks; /* counts 1ms timeTicks */

#define PRESCALER 4 //for HFPER =32 it gives us 8MHz
#define TIMER0_UP_TOP_VAL CMU_ClockFreqGet(cmuClock_HFPER)/PRESCALER/1000

/**************************************************************************//**
 * @brief Delays number of msTick (1 ms)
 * @param dlyTicks Number of ticks to delay
 *****************************************************************************/
void Delay(uint32_t dlyTicks) {
	uint32_t curTicks;

	curTicks = msTicks;
	while ((msTicks - curTicks) < dlyTicks)
		;
}

static void timerInit(void) {
	CMU_ClockEnable(cmuClock_TIMER0, true);
	TIMER_TopSet(TIMER0, TIMER0_UP_TOP_VAL);

	/* Select timer parameters */
	TIMER_Init_TypeDef timerInit =
			{ .enable = true, .debugRun = true, .prescale = timerPrescale4,
					.clkSel = timerClkSelHFPerClk, .fallAction =
							timerInputActionNone, .riseAction =
							timerInputActionNone, .mode = timerModeUp,
					.dmaClrAct = false, .quadModeX4 = false, .oneShot = false,
					.sync = false, };

	/* Enable overflow interrupt */
	TIMER_IntEnable(TIMER0, TIMER_IF_OF);
	NVIC_EnableIRQ(TIMER0_IRQn);

	/* Configure timer */
	TIMER_Init(TIMER0, &timerInit);

}
void utilsInit(void) {
	timerInit();

}

/**************************************************************************//**
 * @brief TIMER0_IRQHandler
 * Interrupt Service Routine TIMER0 Interrupt Line
 *****************************************************************************/
void TIMER0_IRQHandler(void) {

	msTicks++;       // increment counter necessary in Delay()
	/* Clear flag for TIMER0 overflow interrupt */
	TIMER_IntClear(TIMER0, TIMER_IF_OF);
	//BSP_LedToggle(1);

}

/**************************************************************************//**
 * @brief USB_DEBUG_PUTS(char *s); -string should be a NULL terminated string
 * sends string onto usb port
 *****************************************************************************/

void USB_DEBUG_PUTS(char *s) {

#if USB_DEBUG_ENABLED
	if (!s)
		return;
	usbWriteData(s, strlen((const char *) s));
#else
	return;
#endif

}
