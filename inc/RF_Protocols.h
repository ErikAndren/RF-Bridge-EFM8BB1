/*
 * RF_Protocols.h
 *
 *  Created on: 28.11.2017
 *      Author:
 */

#ifndef INC_RF_PROTOCOLS_H_
#define INC_RF_PROTOCOLS_H_

#define MIN_FOOTER_LENGTH	3500
#define MIN_PULSE_LENGTH	100
#define MAX_BUCKETS			8

// TSYN represents the part of the period where the sync pulse is low
#define SONOFF_TSYN_SZ 2
#define SONOFF_TSYN_POS 0
#define SONOFF_TLOW_SZ 2
#define SONOFF_TLOW_POS 2
#define SONOFF_THIGH_SZ 2
#define SONOFF_THIGH_POS 4
#define SONOFF_DATA_SZ 3
#define SONOFF_DATA_POS 6
#define SONOFF_INSTR_SZ (SONOFF_TSYN_SZ + SONOFF_TLOW_SZ + SONOFF_THIGH_SZ + SONOFF_DATA_SZ)

#define CUSTOM_PROTOCOL_IDENT 0x7F
#define CUSTOM_PROTOCOL_SYNC_HIGH_SZ 2
#define CUSTOM_PROTOCOL_SYNC_HIGH_POS 1
#define CUSTOM_PROTOCOL_SYNC_LOW_SZ 2
#define CUSTOM_PROTOCOL_SYNC_LOW_POS 3
#define CUSTOM_PROTOCOL_BIT_HIGH_TIME_SZ 2
#define CUSTOM_PROTOCOL_BIT_HIGH_TIME_POS 5
#define CUSTOM_PROTOCOL_BIT_HIGH_DUTY_SZ 1
#define CUSTOM_PROTOCOL_BIT_HIGH_DUTY_POS 7
#define CUSTOM_PROTOCOL_BIT_LOW_TIME_SZ 2
#define CUSTOM_PROTOCOL_BIT_LOW_TIME_POS 8
#define CUSTOM_PROTOCOL_BIT_LOW_DUTY_SZ 1
#define CUSTOM_PROTOCOL_BIT_LOW_DUTY_POS 10
#define CUSTOM_PROTOCOL_BIT_COUNT_SZ 1
#define CUSTOM_PROTOCOL_BIT_COUNT_POS 11
#define CUSTOM_PROTOCOL_BIT_COUNT_SZ 1
#define CUSTOM_PROTOCOL_DATA_POS 12

#define RF_TX_REPEATS 6

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
	// Send pause bit at the end of each transmission
	uint8_t needs_pause_bit;
} protocol_data_t;

#define DUTY_CYCLE_TOLERANCE 	16

#define UNKNOWN_PROTOCOL		0x00

#define NO_PROTOCOL_FOUND 0x80

/*
 * Home Easy / Nexa / Anslut
 * http://tech.jolowe.se/home-automation-rf-protocols
 */
#define HOME_EASY_IDENTIFIER 0
#define HOME_EASY {HOME_EASY_IDENTIFIER, 275, 2750, 200, 275, 275, 50, 18, 64, 0, true}

/*
 * Rohrmotor24
 * https://github.com/bjwelker/Raspi-Rollo/tree/master/Arduino/Rollo_Code_Receiver
 */
#define ROHRMOTOR24_IDENTIFIER 1
#define ROHRMOTOR24	{ROHRMOTOR24_IDENTIFIER, 4800, 2000, 1500, 700, 300, 70, 30, 40, 0, false}

/*
 * UNDERWATER PAR56 LED LAMP, 502266
 * http://www.seamaid-lighting.com/de/produit/lampe-par56/
 */
#define Seamaid_PAR_56_RGB_IDENTIFIER 2
#define Seamaid_PAR_56_RGB {Seamaid_PAR_56_RGB_IDENTIFIER, 3000, 9000, 2000, 1100, 400, 75, 25, 24, 0, false}

/*
 * Wall plug Noru
  */
#define NORU_IDENTIFIER 3
#define NORU {NORU_IDENTIFIER, 9500, 3000, 2000, 900, 320, 70, 30, 24, 0, false}

/*
 * WS-1200 Series Wireless Weather Station
  */
#define WS_1200_IDENTIFIER 4
#define WS_1200	{WS_1200_IDENTIFIER, 0, 29400, 2000, 700, 300, 38, 64, 64, 7, false}

/* http://www.logicapplied.se/projects/remote_control/nexa/nexa.html */
#define ARC_IDENTIFIER 5
#define ARC {ARC_IDENTIFIER, 440, 10800, 400, 1340, 440, 78, 29, 24, 0, false}
/* FIXME: ARC Commands with double the period time has been observed, might need to add a identifier doubling all values */

/* FIXME: There is a problem with my remote where a spurious pulse may come before the sync pulse
 * it looks like this throws the rx off */
#define EV1527_IDENTIFIER 6
#define EV1527 {EV1527_IDENTIFIER, 400, 7800, 300, 200, 600, 75, 25, 24, 0, false}

/*
 * Original RF bridge protocol
 * PT2260, http://www.princeton.com.tw/Portals/0/Product/PT2260_4.pdf
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

#define PT2260_IDENTIFIER 7
#define PT2260_ALPHA_STEP 128
#define PT2260_SYNC_PERIOD 4096
#define PT2260_SYNC_HIGH PT2260_ALPHA_STEP
#define PT2260_SYNC_LOW (PT2260_SYNC_PERIOD - PT2260_SYNC_HIGH)
#define PT2260_BIT_PERIOD 1024

#define PT2260 {PT2260_IDENTIFIER, 0, 12400, 2000, 1080, 400, 75, 25, 24, 0, false}

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
		ARC,
		EV1527,
		PT2260
};

static const uint8_t PROTOCOL_COUNT = sizeof(PROTOCOLS) / sizeof(protocol_data_t);

#endif /* INC_RF_PROTOCOLS_H_ */
