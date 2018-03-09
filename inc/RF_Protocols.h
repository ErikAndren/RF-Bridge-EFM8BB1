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

typedef struct
{
	// Protocol specific identifier
	uint8_t identifier;
	// normal high signal time on sync pulse
	uint16_t sync_high;
	// normal low signal time on sync pulse
	uint16_t sync_low;
	// high time of a logic bit 1
	uint16_t bit_high_data;
	// high time of a logic bit 0
	uint16_t bit_low_time;
	// duty cycle for logic bit 1
	uint8_t bit_high_duty;
	// duty cycle for logic bit 0
	uint8_t bit_low_duty;
	// bit count for this protocol
	uint8_t bit_count;
	// bit count of SYNC bits
	uint8_t sync_bit_count;
} protocol_data_t;

#define SYNC_TOLERANCE 			200
#define SYNC_TOLERANCE_0xA1		1000
#define DUTY_CYCLE_TOLERANCE 	8

#define UNKNOWN_IDENTIFIER		0x00

#define NO_PROTOCOL_FOUND 0x80


/*
 * PT2260, EV1527,... original RF bridge protocol
 * http://www.princeton.com.tw/Portals/0/Product/PT2260_4.pdf
 * The built-in oscillator circuitry of PT2260 allows a frequency in a range about 100-500kHz.
 * Only 100 - 300 kHz according to the datasheet
 *
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
 * Where does this value come from?
 * Setting the range from 10000, 9000 - 11000
 */

#define RF_TSYN_SZ 2
#define RF_TSYN_POS 0
#define RF_TLOW_SZ 2
#define RF_TLOW_POS 2
#define RF_THIGH_SZ 2
#define RF_THIGH_POS 4
#define RF_DATA_SZ 3
#define RF_DATA_POS 6
#define RF_INSTR_SZ (RF_TSYN_SZ + RF_TLOW_SZ + RF_THIGH_SZ + RF_DATA_SZ)

#define RF_PROTOCOL_IDENT_POS 0
#define RF_PROTOCOL_IDENT_SZ 1
#define RF_PROTOCOL_START_POS 1

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

#define BUCKET_NO_POS 0
#define BUCKET_NO_SZ 1
#define BUCKET_REP_POS 1
#define BUCKET_REP_SZ 1

#define PT2260_IDENTIFIER				0x01
#define PT2260_ALPHA_STEP 128
#define PT2260_SYNC_PERIOD 4096
#define PT2260_SYNC_HIGH PT2260_ALPHA_STEP
#define PT2260_SYNC_LOW (PT2260_SYNC_PERIOD - PT2260_SYNC_HIGH)
#define PT2260_BIT_PERIOD 1024

#define PT2260				{PT2260_IDENTIFIER, 0, 10000, 1080, 360, 75, 25, 24, 0}
#define PT2260_INDEX 0
/*
 * Rohrmotor24
 * https://github.com/bjwelker/Raspi-Rollo/tree/master/Arduino/Rollo_Code_Receiver
 */
#define ROHRMOTOR24_IDENTIFIER			0x02
#define ROHRMOTOR24			{ROHRMOTOR24_IDENTIFIER, 4800, 1500, 700, 300, 70, 30, 40, 0}

/*
 * UNDERWATER PAR56 LED LAMP, 502266
 * http://www.seamaid-lighting.com/de/produit/lampe-par56/
 */
#define Seamaid_PAR_56_RGB_IDENTIFIER	0x03
#define Seamaid_PAR_56_RGB	{Seamaid_PAR_56_RGB_IDENTIFIER, 3000, 9000, 1100, 400, 75, 25, 24, 0}

/*
 * Wall plug Noru
  */
#define NORU_IDENTIFIER					0x04
#define NORU				{NORU_IDENTIFIER, 9500, 3000, 900, 320, 70, 30, 24, 0}

/*
 * WS-1200 Series Wireless Weather Station
  */
#define WS_1200_IDENTIFIER				0x05
#define WS_1200				{WS_1200_IDENTIFIER, 0, 29400, 700, 300, 38, 64, 64, 7}

/*
 * Protocol array
 */
#define PROTOCOLCOUNT 5
#if PROTOCOLCOUNT > 0x7F
#error Too many protocols are defined, stop!
#endif

SI_SEGMENT_VARIABLE(protocol_data[PROTOCOLCOUNT], static const protocol_data_t, SI_SEG_CODE) =
{
		PT2260,
		ROHRMOTOR24,
		Seamaid_PAR_56_RGB,
		NORU,
		WS_1200
};

#endif /* INC_RF_PROTOCOLS_H_ */
