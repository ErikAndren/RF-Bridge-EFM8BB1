/*
 * Timer.h
 *
 *  Created on: 24 feb. 2018
 *      Author: erik
 */

#ifndef INC_TIMER_H_
#define INC_TIMER_H_

#include <stdint.h>
#include <stdbool.h>

#define TIMER2 2
#define TIMER3 3

void InitTimer_us(uint8_t timer, uint16_t interval, uint16_t timeout);
void InitTimer_ms(uint8_t timer, uint16_t interval, uint16_t timeout);
void WaitTimerFinished(uint8_t timer);
bool IsTimerFinished(uint8_t timer);

#endif /* INC_TIMER_H_ */
