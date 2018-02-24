/*
 * Globals.h
 *
 *  Created on: 27.11.2017
 *      Author:
 */

#ifndef INC_GLOBALS_H_
#define INC_GLOBALS_H_

#include <si_toolchain.h>
#include <stdint.h>
#include <SI_EFM8BB1_Defs.h>

// USER CONSTANTS
#define LED_ON	1
#define LED_OFF	0

#define BUZZER_ON	1
#define BUZZER_OFF	0

#define TDATA_ON	1
#define TDATA_OFF	0

#define RF_LISTEN_ON_START	true

#define SYSCLK 24500000

// USER PROTOTYPES
SI_SBIT(LED, SFR_P1, 0);		// LED
SI_SBIT(T_DATA, SFR_P0, 0);		// T_DATA
SI_SBIT(R_DATA, SFR_P1, 3);		// R_DATA
SI_SBIT(BUZZER, SFR_P1, 6);		// BUZZER

#endif /* INC_GLOBALS_H_ */
