/*
 * Timer.c
 *
 *  Created on: 07.12.2017
 *      Author:
 */
#include <SI_EFM8BB1_Register_Enums.h>
#include "Globals.h"
#include "Timer.h"

#define NO_TIMERS 2

SI_SEGMENT_VARIABLE(Timer_Timeout[NO_TIMERS], uint16_t, SI_SEG_XDATA) = 0x0000;
SI_SEGMENT_VARIABLE(Timer_Interval[NO_TIMERS], uint16_t, SI_SEG_XDATA) = 0x0000;

static void StartTimer(uint8_t timer, uint16_t interval, uint16_t timeout) {
	if (timer == TIMER2) {
		TMR2CN0 |= TMR2CN0_TR2__RUN;
		Timer_Timeout[0] = timeout;
		Timer_Interval[0] = interval;

	} else if (timer == TIMER3) {
		TMR3CN0 |= TMR3CN0_TR3__RUN;
		Timer_Timeout[1] = timeout;
		Timer_Interval[1] = interval;
	}
}

void SetTimerReload(uint8_t timer, uint16_t reload)
{
	if (timer == TIMER2) {
		/***********************************************************************
	 	 - Timer 2 Reload High Byte
		 ***********************************************************************/
		TMR2RLH = (((reload >> 8) & 0xFF) << TMR2RLH_TMR2RLH__SHIFT);
		/***********************************************************************
	 	 - Timer 2 Reload Low Byte
		 ***********************************************************************/
		TMR2RLL = ((reload & 0xFF) << TMR2RLL_TMR2RLL__SHIFT);
	} else if (timer == TIMER3) {
		/***********************************************************************
	 	 - Timer 3 Reload High Byte
		 ***********************************************************************/
		TMR3RLH = (((reload >> 8) & 0xFF) << TMR3RLH_TMR3RLH__SHIFT);
		/***********************************************************************
	 	 - Timer 3 Reload Low Byte
		 ***********************************************************************/
		TMR3RLL = ((reload & 0xFF) << TMR3RLL_TMR3RLL__SHIFT);
	}
}

/*
 * Init Timer with microseconds interval, maximum is 65535 s.
 */
void InitTimer_us(uint8_t timer, uint16_t interval, uint16_t timeout)
{
	SetTimerReload(timer, (uint16_t)(0x10000 - ((uint32_t) SYSCLK / (1000000 / (uint32_t) interval))));
	StartTimer(timer, interval, timeout);
}

/*
 * Init Timer with milliseconds interval, maximum is ~2.5ms.
 */
void InitTimer_ms(uint8_t timer, uint16_t interval, uint16_t timeout)
{
	SetTimerReload(timer, (uint16_t)(0x10000 - ((uint32_t) SYSCLK / (1000 / (uint32_t) interval))));
	StartTimer(timer, interval, timeout);
}

void WaitTimerFinished(uint8_t timer)
{
	// wait until timer has finished
	if (timer == TIMER2) {
		while((TMR2CN0 & TMR2CN0_TR2__BMASK) == TMR2CN0_TR2__RUN);
	} else if (timer == TIMER3) {
		while((TMR3CN0 & TMR3CN0_TR3__BMASK) == TMR3CN0_TR3__RUN);
	}
}

bool IsTimerFinished(uint8_t timer)
{
	if (timer == TIMER2) {
		return ((TMR2CN0 & TMR2CN0_TR2__BMASK) != TMR2CN0_TR2__RUN);
	} else if (timer == TIMER3) {
		return ((TMR3CN0 & TMR3CN0_TR3__BMASK) != TMR3CN0_TR3__RUN);
	} else {
		return true;
	}
}

//-----------------------------------------------------------------------------
// TIMER3_ISR
//-----------------------------------------------------------------------------
//
// TIMER3 ISR Content goes here. Remember to clear flag bits:
// TMR3CN0::TF3H (Timer # High Byte Overflow Flag)
// TMR3CN0::TF3L (Timer # Low Byte Overflow Flag)
//
//-----------------------------------------------------------------------------
SI_INTERRUPT (TIMER3_ISR, TIMER3_IRQn)
{
	// Clear Timer 3 high overflow flag
	TMR3CN0 &= ~TMR3CN0_TF3H__SET;

	// check if pulse time is over
	if (Timer_Timeout[1] == 0)
	{
		// stop timer
		TMR3CN0 &= ~TMR3CN0_TR3__RUN;
	}

	if (Timer_Timeout[1] > Timer_Interval[1]) {
		Timer_Timeout[1] -= Timer_Interval[1];
	} else {
		Timer_Timeout[1] = 0;
	}
}

//-----------------------------------------------------------------------------
// TIMER2_ISR
//-----------------------------------------------------------------------------
//
// TIMER2 ISR Content goes here. Remember to clear flag bits:
// TMR2CN0::TF2H (Timer # High Byte Overflow Flag)
// TMR2CN0::TF3L (Timer # Low Byte Overflow Flag)
//
//-----------------------------------------------------------------------------
SI_INTERRUPT (TIMER2_ISR, TIMER2_IRQn)
{
	// Clear Timer 2 high overflow flag
	TMR2CN0 &= ~TMR2CN0_TF2H__SET;

	// check if pulse time is over
	if (Timer_Timeout[0] == 0)
	{
		// stop timer
		TMR2CN0 &= ~TMR2CN0_TR2__RUN;
	}

	if (Timer_Timeout[0] > Timer_Interval[0]) {
		Timer_Timeout[0] -= Timer_Interval[0];
	} else {
		Timer_Timeout[0] = 0;
	}
}
