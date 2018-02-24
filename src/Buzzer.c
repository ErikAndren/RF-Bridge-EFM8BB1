/*
 * Buzzer.c
 *
 *  Created on: 24.02.2018
 *      Author: Erik Zachrisson
 */
 
#include "Globals.h"
#include "Timer.h"

void SoundBuzzer_ms(uint16_t interval) {
	InitTimer_ms(1, interval);
	BUZZER = BUZZER_ON;
	WaitTimerFinished();
	BUZZER = BUZZER_OFF;
}

