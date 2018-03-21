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
SI_SEGMENT_VARIABLE(rf_listen_mode, rf_sniffing_mode_t, SI_SEG_XDATA) = MODE_DUTY_CYCLE;

SI_SEGMENT_VARIABLE(duty_cycle_high, uint8_t, SI_SEG_XDATA) = 0x56;
SI_SEGMENT_VARIABLE(duty_cycle_low, uint8_t, SI_SEG_XDATA) = 0xAB;
SI_SEGMENT_VARIABLE(t0_high, uint8_t, SI_SEG_XDATA) = 0x00;
SI_SEGMENT_VARIABLE(t0_low, uint8_t, SI_SEG_XDATA) = 0x00;
SI_SEGMENT_VARIABLE(sync_high, uint16_t, SI_SEG_XDATA) = 0x00;
SI_SEGMENT_VARIABLE(sync_low, uint16_t, SI_SEG_XDATA) = 0x00;
SI_SEGMENT_VARIABLE(bit_high, uint16_t, SI_SEG_XDATA) = 0x00;
SI_SEGMENT_VARIABLE(bit_low, uint16_t, SI_SEG_XDATA) = 0x00;
SI_SEGMENT_VARIABLE(bit_count, uint8_t, SI_SEG_XDATA) = 0x00;

SI_SEGMENT_VARIABLE(actual_bit_of_byte, uint8_t, SI_SEG_XDATA) = 0;
SI_SEGMENT_VARIABLE(actual_bit, uint8_t, SI_SEG_XDATA) = 0;
SI_SEGMENT_VARIABLE(actual_sync_bit, uint8_t, SI_SEG_XDATA) = 0;
SI_SEGMENT_VARIABLE(actual_byte, uint8_t, SI_SEG_XDATA) = 0;

// up to 8 timing buckets for MODE_BUCKET
SI_SEGMENT_VARIABLE(bucket_sync, uint16_t, SI_SEG_XDATA);
SI_SEGMENT_VARIABLE(buckets[BUCKET_MAX], uint16_t, SI_SEG_XDATA);	// -1 because of the bucket_sync
SI_SEGMENT_VARIABLE(bucket_count, uint8_t, SI_SEG_XDATA) = 0;

//-----------------------------------------------------------------------------
// Callbacks
//-----------------------------------------------------------------------------
void PCA0_overflowCb()
{
}

void PCA0_channel2EventCb()
{
}


// Called when receiving
// FIXME: This is a lot of work done in interrupt context. It might be wise to move this to the main thread
// Triggered when either a rising or falling edge has been detected on the input pin.
// Timer 0 is the PCA counter input and is configured to increment once every 50 us
void PCA0_channel1EventCb()
{
	// Store most recent capture value
	// FIXME: Why do we multiply this by 10? Is it to harmonize the calculated period with the protocol definition
	uint16_t current_capture_value = PCA0CP1 * 10;
	uint16_t capture_period_neg;
	uint8_t current_duty_cycle;

	static uint16_t capture_period_pos;
	static uint16_t previous_capture_value_neg, previous_capture_value_pos;
	static uint16_t low_pulse_time;

	// positive edge
	if (R_DATA)
	{
		// Update previous capture value with most recent info.
		previous_capture_value_pos = current_capture_value;

		// Calculate capture period from last two values.
		capture_period_neg = previous_capture_value_pos - previous_capture_value_neg;

		// do sniffing by mode
		switch (rf_listen_mode)
		{
			// do sniffing by duty cycle mode
			case MODE_DUTY_CYCLE:
				switch (rf_state)
				{
					// Search for a sync from a known protocol
					case RF_IDLE:
						rf_protocol = IdentifyRFProtocol(desired_rf_protocol, capture_period_pos, capture_period_neg);

						// check if a matching protocol got found
						if (rf_protocol != NO_PROTOCOL_FOUND)
						{
							sync_high = capture_period_pos;
							sync_low = capture_period_neg;
							actual_bit_of_byte = 8;
							actual_byte = 0;
							actual_bit = 0;
							actual_sync_bit = 0;
							low_pulse_time = 0;
							rf_state = RF_IN_SYNC;
							rf_data[0] = 0;
							LED = LED_ON;
						}
						break; // switch rf_state

					// one matching sync got received
					case RF_IN_SYNC:
						// Skip SYNC bits, if any
						if (actual_sync_bit < PROTOCOLS[rf_protocol].sync_bit_count)
						{
							actual_sync_bit++;
							break;
						}

						// check the rest of the bits
						actual_bit_of_byte--;
						actual_bit++;
						LED = !LED;

						// calculate current duty cycle
						current_duty_cycle = (100 * (uint32_t) capture_period_pos) / ((uint32_t) capture_period_pos + (uint32_t) capture_period_neg);

						if (((current_duty_cycle > (PROTOCOLS[rf_protocol].bit_high_duty - DUTY_CYCLE_TOLERANCE)) &&
							(current_duty_cycle < (PROTOCOLS[rf_protocol].bit_high_duty + DUTY_CYCLE_TOLERANCE)) &&
							(actual_bit < PROTOCOLS[rf_protocol].bit_count)) ||
							// the duty cycle can not be used for the last bit because of the missing rising edge on the end
							((capture_period_pos > low_pulse_time) && (actual_bit == PROTOCOLS[rf_protocol].bit_count)))
						{
							// backup last bit high time
							bit_high = capture_period_pos;
							rf_data[(actual_bit - 1) / 8] |= (1 << actual_bit_of_byte);
						} else {
							// backup last bit low time
							bit_low = capture_period_pos;
							// backup low bit pulse time to be able to determine the last bit
							if (capture_period_pos > low_pulse_time) {
								low_pulse_time = capture_period_pos;
							}
						}

						if (actual_bit_of_byte == 0) {
							actual_bit_of_byte = 8;
							rf_data[actual_bit / 8] = 0;
						}

						// check if all bits for this protocol got received
						if (actual_bit == PROTOCOLS[rf_protocol].bit_count)
						{
							LED = LED_OFF;
							rf_state = RF_FINISHED;
						}
						break;
				}
				break; // switch(rf_listen_mode)

				// do sniffing by bucket mode
				case MODE_BUCKET:
					Bucket_Received(capture_period_neg);
					break;
		}
	}
	// negative edge
	else
	{
		// Update previous capture value with most recent info.
		previous_capture_value_neg = current_capture_value;

		// Calculate capture period from last two values.
		capture_period_pos = previous_capture_value_neg - previous_capture_value_pos;

		// do sniffing by mode
		if (rf_listen_mode == MODE_BUCKET) {
			Bucket_Received(capture_period_pos);
		}
	}
}

// Receive path
void PCA0_StartRFListen(void)
{
	// restore timer to 100000 Hz, 10 s interval
	// 245 ccs * 40.8 ns = 50 us = 100000 Hz
	TH0 = 256 - TIMER0_CC_S_TO_COUNT;

	// enable interrupt for RF reception
	PCA0CPM1 |= PCA0CPM1_ECCF__ENABLED;

	rf_state = RF_IDLE;

	PCA0_run();
}

void PCA0_StopRFListen(void)
{
	PCA0_halt();

	// clear all interrupt flags of PCA0
	PCA0CN0 &= ~(PCA0CN0_CF__BMASK | PCA0CN0_CCF0__BMASK | PCA0CN0_CCF1__BMASK | PCA0CN0_CCF2__BMASK);

	// disable interrupt for RF receiving
	PCA0CPM1 &= ~PCA0CPM1_ECCF__ENABLED;
}

static uint8_t IdentifyRFProtocol(uint8_t identifier, uint16_t period_pos, uint16_t period_neg)
{
	uint8_t protocol_found = NO_PROTOCOL_FOUND;
	uint8_t used_protocol;

	switch(identifier)
	{
		// protocol is undefined, do loop through all protocols
		case UNKNOWN_IDENTIFIER:
			// check all protocols
			for (used_protocol = 0; used_protocol < PROTOCOLCOUNT; used_protocol++)
			{
				if ((period_neg > (PROTOCOLS[used_protocol].sync_low - SYNC_TOLERANCE)) &&
					(period_neg < (PROTOCOLS[used_protocol].sync_low + SYNC_TOLERANCE)))
				{
					if ((PROTOCOLS[used_protocol].sync_high == 0) ||
					   ((period_pos > (PROTOCOLS[used_protocol].sync_high - SYNC_TOLERANCE)) &&
						(period_pos < (PROTOCOLS[used_protocol].sync_high + SYNC_TOLERANCE))))
					{
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

			if ((period_neg > (PROTOCOLS[used_protocol].sync_low - SYNC_TOLERANCE_0xA1)) &&
				(period_neg < (PROTOCOLS[used_protocol].sync_low + SYNC_TOLERANCE_0xA1)))
			{
				if ((PROTOCOLS[used_protocol].sync_high == 0) ||
				   ((period_pos > (PROTOCOLS[used_protocol].sync_high - SYNC_TOLERANCE_0xA1)) &&
					(period_pos < (PROTOCOLS[used_protocol].sync_high + SYNC_TOLERANCE_0xA1))))
				{
					protocol_found = used_protocol;
					break;
				}
			}
			break;
	}

	return protocol_found;
}

static void SetTimer0Overflow(uint8_t T0_Overflow)
{
	/***********************************************************************
	 - Timer 0 High Byte = T0_Overflow
	 ***********************************************************************/
	// This is the reload value used to reload TL0 when it is in mode 2. See 18.3.2.1 in reference manual

	TH0 = (T0_Overflow << TH0_TH0__SHIFT);
}


uint8_t GetProtocolIndex(uint8_t identifier)
{
	uint8_t i;
	uint8_t protocol_index = NO_PROTOCOL_FOUND;

	// check first for valid identifier
	if ((identifier > UNKNOWN_IDENTIFIER) && (identifier < NO_PROTOCOL_FOUND))
	{
		// find protocol index by identifier
		for (i = 0; i < PROTOCOLCOUNT; i++)
		{
			if (PROTOCOLS[i].identifier == identifier)
			{
				protocol_index = i;
				break;
			}
		}
	}

	return protocol_index;
}

// Transmission path
void PCA0_InitRFTransmit(uint16_t sync_high_in, uint16_t sync_low_in,
					   uint16_t bit_high_time, uint8_t bit_high_duty,
		               uint16_t bit_low_time, uint8_t bit_low_duty, uint8_t bitcount)
{
	uint16_t bit_time;

	// set global variable
	sync_high = sync_high_in;
	sync_low = sync_low_in;

	// calculate T0_Overflow
	bit_time = (100 * (uint32_t) bit_high_time) / bit_high_duty;
	t0_high = (uint8_t)(0x100 - ((uint32_t) SYSCLK / (0xFF * (1000000 / (uint32_t) bit_time))));

	bit_time = (100 * (uint32_t) bit_low_time) / bit_low_duty;
	t0_low = (uint8_t)(0x100 - ((uint32_t) SYSCLK / (0xFF * (1000000 / (uint32_t) bit_time))));

	// calculate high and low duty cycle
	duty_cycle_high = (uint16_t) ((bit_high_duty * 0xFF) / 100);
	duty_cycle_low = (uint16_t) ((bit_low_duty * 0xFF) / 100);

	bit_count = bitcount;

	// enable interrupt for RF transmitting
	PCA0CPM0 |= PCA0CPM0_ECCF__ENABLED;
	PCA0PWM |= PCA0PWM_ECOV__COVF_MASK_ENABLED;

	/***********************************************************************
	 - PCA Counter/Timer Low Byte = 0xFF, why?
	 ***********************************************************************/
	PCA0L = (0xFF << PCA0L_PCA0L__SHIFT);
}

void PCA0_StartRFTransmit(uint8_t payload_pos)
{
	actual_bit_of_byte = 7;
	actual_bit = 1;
	actual_byte = payload_pos;
	rf_state = RF_TRANSMITTING;

	// set first bit to be in sync when PCA0 is starting
	PCA0_SetDutyCycle();

	// make RF sync pulse
	// FIXME: According to PT2260 docs, sync pulse comes after payload
	// Yes, but not in the EV-protocol. Doesn't matter if multiple transmits are sent
	SendRF_Sync();

	PCA0_run();
}

static void PCA0_SetDutyCycle(void)
{
	if (((rf_data[actual_byte] >> actual_bit_of_byte) & 0x01) == 0x01)
	{
		// bit 1
		PCA0_writeChannel(PCA0_CHAN0, duty_cycle_high << 8);
	} else {
		// bit 0
		PCA0_writeChannel(PCA0_CHAN0, duty_cycle_low << 8);
	}
}

static void SendRF_Sync(void)
{
	// enable P0.0 for I/O control
	XBR1 &= ~XBR1_PCA0ME__CEX0_CEX1;

	// Send ASK/On-off keying to SYN115 chip
	T_DATA = 1;

	// What is happening here? Activation?
	// FIXME: Should not be needed
	InitTimer_ms(TIMER3, 1, 7);
	WaitTimerFinished(TIMER3);
	T_DATA = 0;
	InitTimer_us(TIMER3, 10, 100);
	WaitTimerFinished(TIMER3);

	// FIXME: Add sync repeats
	T_DATA = 1;
	InitTimer_us(TIMER3, 10, sync_high);
	WaitTimerFinished(TIMER3);
	T_DATA = 0;
	InitTimer_us(TIMER3, 10, sync_low);
	WaitTimerFinished(TIMER3);

	// disable P0.0 for I/O control, enter PCA mode
	XBR1 |= XBR1_PCA0ME__CEX0_CEX1;
}

// Half of symbol transmitted, check if to change output
void PCA0_intermediateOverflowCb()
{
	if (((rf_data[actual_byte] >> actual_bit_of_byte) & 0x01) == 0x01)
	{
		// bit 1
		SetTimer0Overflow(t0_high);
	} else {
		// bit 0
		SetTimer0Overflow(t0_low);
	}
}

// Called when transmission of a symbol is done
void PCA0_channel0EventCb()
{
	// Move on to next byte
	if (actual_bit_of_byte == 0)
	{
		actual_byte++;
		actual_bit_of_byte = 8;
	}

	// stop transfer if all bits are transmitted
	if (actual_bit == bit_count)
	{
		PCA0_StopRFTransmit();
	} else {
		actual_bit++;
		actual_bit_of_byte--;

		// Start transmission of next ibt
		PCA0_SetDutyCycle();
	}
}

void PCA0_StopRFTransmit(void)
{
	// set duty cycle to zero
	PCA0_writeChannel(PCA0_CHAN0, 0);

	// disable interrupt for RF transmitting
	PCA0CPM0 &= ~PCA0CPM0_ECCF__ENABLED;
	PCA0PWM &= ~PCA0PWM_ECOV__COVF_MASK_ENABLED;

	PCA0_halt();

	// clear all interrupt flags of PCA0
	PCA0CN0 &= ~(PCA0CN0_CF__BMASK | PCA0CN0_CCF0__BMASK | PCA0CN0_CCF1__BMASK | PCA0CN0_CCF2__BMASK);

	// enable P0.0 for I/O control
	XBR1 &= ~XBR1_PCA0ME__CEX0_CEX1;
	// switch to low
	T_DATA = 0;

	// disable P0.0 for I/O control, enter PCA mode
	XBR1 |= XBR1_PCA0ME__CEX0_CEX1;

	rf_state = RF_FINISHED;
}
