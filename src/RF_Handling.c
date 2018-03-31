/*
 * RF_Handling.c
 *
 *  Created on: 27.11.2017
 *      Author:
 */

#include <SI_EFM8BB1_Register_Enums.h>
#include <string.h>
#include "Globals.h"
#include "RF_Handling.h"
#include "RF_Protocols.h"
#include "RF_Buckets.h"
#include "pca_0.h"
#include "uart.h"
#include "Timer.h"
#include "Delay.h"

SI_SEGMENT_VARIABLE(rf_data[RF_DATA_BUFFERSIZE], uint8_t, SI_SEG_XDATA);
SI_SEGMENT_VARIABLE(rf_protocol, uint8_t, SI_SEG_XDATA) = 0;
SI_SEGMENT_VARIABLE(rf_state, rf_state_t, SI_SEG_XDATA) = RF_IDLE;
SI_SEGMENT_VARIABLE(desired_rf_protocol, uint8_t, SI_SEG_XDATA) = UNKNOWN_IDENTIFIER;

SI_SEGMENT_VARIABLE(duty_cycle_high, uint8_t, SI_SEG_XDATA) = 0;
SI_SEGMENT_VARIABLE(duty_cycle_low, uint8_t, SI_SEG_XDATA) = 0;
SI_SEGMENT_VARIABLE(t0_high, uint8_t, SI_SEG_XDATA) = 0;
SI_SEGMENT_VARIABLE(t0_low, uint8_t, SI_SEG_XDATA) = 0;
SI_SEGMENT_VARIABLE(sync_high, uint16_t, SI_SEG_XDATA) = 0;
SI_SEGMENT_VARIABLE(sync_low, uint16_t, SI_SEG_XDATA) = 0;
SI_SEGMENT_VARIABLE(bit_high, uint16_t, SI_SEG_XDATA) = 0;
SI_SEGMENT_VARIABLE(bit_low, uint16_t, SI_SEG_XDATA) = 0;
SI_SEGMENT_VARIABLE(bit_count, uint8_t, SI_SEG_XDATA) = 0;

SI_SEGMENT_VARIABLE(actual_bit, uint8_t, SI_SEG_XDATA) = 0;
SI_SEGMENT_VARIABLE(actual_sync_bit, uint8_t, SI_SEG_XDATA) = 0;
SI_SEGMENT_VARIABLE(actual_byte, uint8_t, SI_SEG_XDATA) = 0;

SI_SEGMENT_VARIABLE(pos_pulse_len, uint16_t, SI_SEG_DATA) = 0;
SI_SEGMENT_VARIABLE(neg_pulse_len, uint16_t, SI_SEG_DATA) = 0;
SI_SEGMENT_VARIABLE(low_pulse_time, uint16_t, SI_SEG_DATA) = 0;

// up to 8 timing buckets for MODE_BUCKET
SI_SEGMENT_VARIABLE(bucket_sync, uint16_t, SI_SEG_XDATA);
SI_SEGMENT_VARIABLE(buckets[BUCKET_MAX], uint16_t, SI_SEG_XDATA);	// -1 because of the bucket_sync
SI_SEGMENT_VARIABLE(bucket_count, uint8_t, SI_SEG_XDATA) = 0;

//-----------------------------------------------------------------------------
// Callbacks
//-----------------------------------------------------------------------------
void PCA0_overflowCb(void)
{
}

void PCA0_channel2EventCb(void)
{
}

// Called when receiving
// Triggered when either a rising or falling edge has been detected on the input pin.
// Timer 0 is the PCA counter input and is configured to overflow every 245 cc, increment once every 10 us
void PCA0_channel1EventCb(void)
{
	// Store most recent capture value
	// Timer 0 is overflowing every 10 us, generating one increment in the PCA
	// Multiplying this with 10 yields an "increment" every 1 us (1000 kHz)

	// Rising edge detected. This is the end of a negative pulse and the start of a positive pulse
	if (R_DATA == 1) {
		// FIXME: Multiplication to reach us resolution, this is a hack to prevent having to flip flop between counting 24 and 25
		// Flip flopping would be very expensive (too many interrupts) i. e. changing the TH0 reload value every 24 cc. If we let the counter be too slow and count to 25 we could
		// compensate this by dividing by 48 and add this to the result, this would still yield an truncation (think 47 / 48)
		// What about dithering?
		// Let's talk about the error today, it could be 244 ccs or an error of almost 9 us. Is this a problem? Most likely not
		neg_pulse_len = PCA0CP1 * 10;

	// Falling edge detected. This is the end of a positive pulse and the start of a negative pulse.
	} else {
		pos_pulse_len = PCA0CP1 * 10;
	}

	/* Reset to 0 to avoid needing to handle counter wraps */
	PCA0_writeCounter(0);
	TL0 = TH0;
}

// Receive path
void StartRFListen(void)
{
	// restore timer to 100000 Hz, 10 s interval
	// 245 cc's * 40.8 ns = 50 us = 100000 Hz = 100 kHz
	TH0 = 256 - TIMER0_CC_S_TO_COUNT;
	TL0 = TH0;

	// enable interrupt for RF reception
	PCA0CPM1 |= PCA0CPM1_ECCF__ENABLED;

	rf_state = RF_IDLE;

	//FIXME: Not really necessary to start and stop the PCA counter. Disabling the interrupt is probably enough
	PCA0_writeCounter(0);
	PCA0_run();
}

void StopRFListen(void)
{
	PCA0_halt();

	// clear all interrupt flags of PCA0
	PCA0CN0 &= ~(PCA0CN0_CF__BMASK | PCA0CN0_CCF0__BMASK | PCA0CN0_CCF1__BMASK | PCA0CN0_CCF2__BMASK);

	// disable interrupt for RF receiving
	PCA0CPM1 &= ~PCA0CPM1_ECCF__ENABLED;
}

uint8_t IdentifyRFProtocol(uint8_t identifier, uint16_t period_pos, uint16_t period_neg)
{
	uint8_t protocol_found = NO_PROTOCOL_FOUND;
	uint8_t used_protocol;

	switch(identifier) {
		// protocol is undefined, do loop through all protocols
		case UNKNOWN_IDENTIFIER:
			// check all protocols
			for (used_protocol = 0; used_protocol < PROTOCOLCOUNT; used_protocol++) {
				if ((period_neg > (PROTOCOLS[used_protocol].sync_low - SYNC_TOLERANCE)) &&
					(period_neg < (PROTOCOLS[used_protocol].sync_low + SYNC_TOLERANCE))) {
					if ((PROTOCOLS[used_protocol].sync_high == 0) ||
					   ((period_pos > (PROTOCOLS[used_protocol].sync_high - SYNC_TOLERANCE)) &&
						(period_pos < (PROTOCOLS[used_protocol].sync_high + SYNC_TOLERANCE)))) {
						protocol_found = used_protocol;
						break;
					}
				}
			}
			break;

		// check other protocols
		default:
			used_protocol = GetProtocolIndex(identifier);

			// check if identifier got found in list
			if (used_protocol == NO_PROTOCOL_FOUND) {
				break;
			}

			if ((period_neg > (PROTOCOLS[used_protocol].sync_low - SYNC_TOLERANCE)) &&
				(period_neg < (PROTOCOLS[used_protocol].sync_low + SYNC_TOLERANCE))) {
				if ((PROTOCOLS[used_protocol].sync_high == 0) ||
				   ((period_pos > (PROTOCOLS[used_protocol].sync_high - SYNC_TOLERANCE)) &&
					(period_pos < (PROTOCOLS[used_protocol].sync_high + SYNC_TOLERANCE)))) {
					protocol_found = used_protocol;
					break;
				}
			}
			break;
	}

	return protocol_found;
}

uint8_t GetProtocolIndex(uint8_t identifier)
{
	uint8_t i;
	uint8_t protocol_index = NO_PROTOCOL_FOUND;

	// check first for valid identifier
	if ((identifier > UNKNOWN_IDENTIFIER) && (identifier < NO_PROTOCOL_FOUND)) {
		// find protocol index by identifier
		for (i = 0; i < PROTOCOLCOUNT; i++) {
			if (PROTOCOLS[i].identifier == identifier) {
				protocol_index = i;
				break;
			}
		}
	}

	return protocol_index;
}

// Transmission path
static void SendRFSync(void)
{
	// enable P0.0 for I/O control
	XBR1 &= ~XBR1_PCA0ME__CEX0_CEX1;

	// Send ASK/On-off keying to SYN115 chip
	T_DATA = 1;
	InitTimer_us(TIMER3, 10, sync_high);
	WaitTimerFinished(TIMER3);
	T_DATA = 0;
	InitTimer_us(TIMER3, 10, sync_low);
	WaitTimerFinished(TIMER3);

	// disable P0.0 for I/O control, enter PCA mode
	XBR1 |= XBR1_PCA0ME__CEX0_CEX1;
}

void StartRFTransmit(uint16_t sync_high_in, uint16_t sync_low_in,
					 uint16_t bit_high_time, uint8_t bit_high_duty,
		             uint16_t bit_low_time, uint8_t bit_low_duty,
				     uint8_t bitcount, uint8_t payload_pos)
{
	uint16_t bit_time;

	// set global variable
	sync_high = sync_high_in;
	sync_low = sync_low_in;

	// calculate T0_Overflow
	// This is the reload value used to reload TL0 when it is in mode 2. See 18.3.2.1 in reference manual
	// FIXME: This logic needs to be explained.
	// Multiply the bit high time in us with 100 and divide with the bit high duty cycle
	// (100 * us) / %
	// PWM counter is 8 bits i.e 256 values

	bit_time = (100 * (uint32_t) bit_high_time) / bit_high_duty;
	// will give us the bit high us per high percent
	// 256 - (sysclk / (255 * (1000000 / (100 * us) / %)))
	// Is 0xFF connected to the 8 bit PCA counting?
	t0_high = (uint8_t) (256 - ((uint32_t) SYSCLK / (0xFF * (1000000 / (uint32_t) bit_time))));

	bit_time = (100 * (uint32_t) bit_low_time) / bit_low_duty;
	t0_low = (uint8_t) (256 - ((uint32_t) SYSCLK / (0xFF * (1000000 / (uint32_t) bit_time))));

	// calculate high and low duty cycle
	duty_cycle_high = (uint16_t) ((bit_high_duty * 0xFF) / 100);
	duty_cycle_low = (uint16_t) ((bit_low_duty * 0xFF) / 100);

	bit_count = bitcount;

	// enable interrupt for RF transmitting
	PCA0CPM0 |= PCA0CPM0_ECCF__ENABLED;
	PCA0PWM |= PCA0PWM_ECOV__COVF_MASK_ENABLED;

	// Prepare for an overflow trigger right away
	PCA0L = (0xFF << PCA0L_PCA0L__SHIFT);

	actual_bit = 0;
	actual_byte = payload_pos;
	rf_state = RF_TRANSMITTING;

	// set first bit to be in sync when PCA0 is starting
	SetDutyCycle();

	// According to PT2260 docs, sync pulse comes after payload
	// Yes, but not in the EV-protocol. Doesn't matter if multiple transmits are sent
	SendRFSync();
	PCA0_run();
}

// This defines the falling edge of the pulse, in PCA0 cycles
// Of the 256 counts, where shall the falling edge occur?
static void SetDutyCycle(void)
{
	if (((rf_data[actual_byte] << (actual_bit % 8)) & 0x80) == 0x80)
	{
		// bit 1
		PCA0_writeChannel(PCA0_CHAN0, duty_cycle_high << 8);
	} else {
		// bit 0
		PCA0_writeChannel(PCA0_CHAN0, duty_cycle_low << 8);
	}
}

// Half of symbol transmitted, check if to change duty cycle length
// Called when rising edge (falling non. inv) is generated (overflow)
// This defines the length (frequency)
// Starts the high part of the pulse
void PCA0_intermediateOverflowCb(void)
{
	// This will effectively toggle how fast the counter will wrap around
	// Both timer 0 and PCA counter works in 8 bit mode
	if (((rf_data[actual_byte] << (actual_bit % 8)) & 0x80) == 0x80) {
		// bit 1
		TH0 = t0_high;
		TL0 = t0_high;
	} else {
		// bit 0
		TH0 = t0_low;
		TL0 = t0_low;
	}
}

// Called upon falling edge (rising non. inv)
// Starts the low part of the pulse
void PCA0_channel0EventCb(void)
{
	actual_bit++;

	// Move on to next byte
	if (actual_bit % 8 == 0) {
		actual_byte++;
	}

	// stop transfer if all bits are transmitted
	if (actual_bit == bit_count) {
		StopRFTransmit();
	} else {
		// Start transmission of next bit
		SetDutyCycle();
	}
}

void StopRFTransmit(void)
{
	// set duty cycle to zero
	// FIXME: Is this really necessary?
	PCA0_writeChannel(PCA0_CHAN0, 0);

	// disable interrupt for RF transmitting
	PCA0CPM0 &= ~PCA0CPM0_ECCF__ENABLED;
	PCA0PWM &= ~PCA0PWM_ECOV__COVF_MASK_ENABLED;

	PCA0_halt();

	// clear all interrupt flags of PCA0
	PCA0CN0 &= ~(PCA0CN0_CF__BMASK | PCA0CN0_CCF0__BMASK | PCA0CN0_CCF1__BMASK | PCA0CN0_CCF2__BMASK);

	// enable P0.0 for I/O control
	XBR1 &= ~XBR1_PCA0ME__CEX0_CEX1;

	T_DATA = 0;

	// disable P0.0 for I/O control, enter PCA mode
	XBR1 |= XBR1_PCA0ME__CEX0_CEX1;

	rf_state = RF_FINISHED;
}
