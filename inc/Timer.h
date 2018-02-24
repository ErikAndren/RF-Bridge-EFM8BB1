/*
 * Timer.h
 *
 *  Created on: 24 feb. 2018
 *      Author: erik
 */

#ifndef INC_TIMER_H_
#define INC_TIMER_H_

#include <stdint.h>

void InitTimer_us(uint16_t interval, uint16_t timeout);
void InitTimer_ms(uint16_t interval, uint16_t timeout);
void WaitTimerFinished(void);
bool IsTimerFinished(void);

#endif /* INC_TIMER_H_ */
