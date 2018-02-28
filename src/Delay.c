/*
 * Delay.c
 *
 *  Created on: 24 feb. 2018
 *      Author: Erik Zachrisson
 */

#include "Timer.h"
#include "Delay.h"

void delay_ms(uint16_t delay) {
	InitTimer_ms(TIMER3, 1, delay);
	WaitTimerFinished(TIMER3);
}
