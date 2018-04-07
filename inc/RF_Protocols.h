/*
 * RF_Protocols.h
 *
 *  Created on: 28.11.2017
 *      Author:
 */

#ifndef INC_RF_PROTOCOLS_H_
#define INC_RF_PROTOCOLS_H_

//-----------------------------------------------------------------------------
// Includes
//-----------------------------------------------------------------------------
#include <stdint.h>

#define MIN_FOOTER_LENGTH	3500
#define MIN_PULSE_LENGTH	100
#define MAX_BUCKETS			8

#define RF_TX_REPEATS 8

typedef struct
{
	// Protocol specific identifier
	uint8_t identifier;
	// normal high signal time on sync pulse (us)
	uint16_t sync_high;
	// normal low signal time on sync pulse (us)
	uint16_t sync_low;
	// tolerance +- in us
 	uint16_t sync_tolerance;
	// high time of a logic bit 1 (us), only used for tx
	uint16_t bit_high_time;
	// high time of a logic bit 0 (us), only used for tx
	uint16_t bit_low_time;
	// duty cycle for logic bit 1 %,
	uint8_t bit_high_duty;
	// duty cycle for logic bit 0 % not used for rx
	uint8_t bit_low_duty;
	// bit count for this protocol
	uint8_t bit_count;
	// bit count of SYNC bits
	uint8_t additional_sync_bits;
} protocol_data_t;

#define DUTY_CYCLE_TOLERANCE 	16

#define UNKNOWN_IDENTIFIER		0x00

#define NO_PROTOCOL_FOUND 0x80

/*
 * Home Easy / Nexa / Anslut
 * http://tech.jolowe.se/home-automation-rf-protocols
 */
#define HOME_EASY_IDENTIFIER 0
#define HOME_EASY {HOME_EASY_IDENTIFIER, 275, 2750, 200, 275, 275, 50, 16, 64, 0}

/*
 * Rohrmotor24
 * https://github.com/bjwelker/Raspi-Rollo/tree/master/Arduino/Rollo_Code_Receiver
 */
#define ROHRMOTOR24_IDENTIFIER 1
#define ROHRMOTOR24	{ROHRMOTOR24_IDENTIFIER, 4800, 2000, 1500, 700, 300, 70, 30, 40, 0}

/*
 * UNDERWATER PAR56 LED LAMP, 502266
 * http://www.seamaid-lighting.com/de/produit/lampe-par56/
 */
#define Seamaid_PAR_56_RGB_IDENTIFIER 2
#define Seamaid_PAR_56_RGB {Seamaid_PAR_56_RGB_IDENTIFIER, 3000, 9000, 2000, 1100, 400, 75, 25, 24, 0}

/*
 * Wall plug Noru
  */
#define NORU_IDENTIFIER 3
#define NORU {NORU_IDENTIFIER, 9500, 3000, 2000, 900, 320, 70, 30, 24, 0}

/*
 * WS-1200 Series Wireless Weather Station
  */
#define WS_1200_IDENTIFIER 4
#define WS_1200	{WS_1200_IDENTIFIER, 0, 29400, 2000, 700, 300, 38, 64, 64, 7}

/*
 * Original RF bridge protocol
 * PT2260, http://www.princeton.com.tw/Portals/0/Product/PT2260_4.pdf
 * EV1527, https://www.sunrom.com/get/206000
 * The built-in oscillator circuitry of PT2260 allows a frequency in a range about 100-500kHz.
 * Only 100 - 300 kHz according to the datasheet
 *
 * PT2260:
 * Alpha is oscillating clock period
 * Period is 4096 Alpha
 * Sync high width is 128 Alpha
 * Sync low width is 3968 Alpha
 *
 * Each bit is 1024 alpha
 * Each bit is divided into two pulse cycles
 * bit 0 is in 128 alpha steps: -___-___
 * bit 1 is in 128 alpha steps: ---_---_
 * Float is in 128 alpha steps: -___---_
 * A message consists of A0 A1 A2 A3 A4 A5 A6 A7 A8/D3 A9/D2 D1 D0 Sync bit
 * This is only 12 bits but Sonoff format specifies 24 bit of payload
 * Takeaway is that Sonoff format transmits the encoded codeword i. e. each two bit set
 * would need to be parsed down to a one bit pattern.
 * This is probably fine and even better if floating bits are used.
 * Also the sync bits come at the end but as it is likely sending multiple iterations one might just as well interpret it as the start
 *
 * 100kHz:
 * Alpha = 10 us
 * Sync High: 128 * Alpha = 1.28 ms
 * Sync Low: 3968 * Alpha = 39.68 ms
 *
 * 500kHz:
 * Alpha = 2 us
 * Sync High: 128 * Alpha = 256 us
 * Sync Low: 3968 * Alpha = 7936 us
 *
 */

#define PT2260_IDENTIFIER 5
#define PT2260_ALPHA_STEP 128
#define PT2260_SYNC_PERIOD 4096
#define PT2260_SYNC_HIGH PT2260_ALPHA_STEP
#define PT2260_SYNC_LOW (PT2260_SYNC_PERIOD - PT2260_SYNC_HIGH)
#define PT2260_BIT_PERIOD 1024

//FIXME: If sync low is set to 12400 reception of HOME_EASY does not work
#define PT2260 {PT2260_IDENTIFIER, 0, 1240, 2000, 1080, 400, 75, 25, 24, 0}

/*
 * Protocol array
 */

/* Keil C51 compiler does not support C99 i. e. no designated initializers */
SI_SEGMENT_VARIABLE(PROTOCOLS[], static const protocol_data_t, SI_SEG_CODE) =
{
		HOME_EASY,
		ROHRMOTOR24,
		Seamaid_PAR_56_RGB,
		NORU,
		WS_1200,
		PT2260
};

static const uint8_t PROTOCOLCOUNT = sizeof(PROTOCOLS) / sizeof(protocol_data_t);

#endif /* INC_RF_PROTOCOLS_H_ */
