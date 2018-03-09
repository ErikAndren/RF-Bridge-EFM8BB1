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
#include "pca_0.h"
#include "uart.h"
#include "Timer.h"
#include "Delay.h"

SI_SEGMENT_VARIABLE(rf_data[RF_DATA_BUFFERSIZE], uint8_t, SI_SEG_XDATA);
SI_SEGMENT_VARIABLE(rf_data_status, uint8_t, SI_SEG_XDATA) = 0;
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
SI_SEGMENT_VARIABLE(buckets[15], uint16_t, SI_SEG_XDATA);	// -1 because of the bucket_sync
SI_SEGMENT_VARIABLE(bucket_count, uint8_t, SI_SEG_XDATA) = 0;

//-----------------------------------------------------------------------------
// Callbacks
//-----------------------------------------------------------------------------
void PCA0_overflowCb()
{
}

// Half of symbol transmitted, check if to change output
void PCA0_intermediateOverflowCb()
{
	if(((rf_data[actual_byte] >> actual_bit_of_byte) & 0x01) == 0x01)
	{
		// bit 1
		SetTimer0Overflow(t0_high);
	}
	else
	{
		// bit 0
		SetTimer0Overflow(t0_low);
	}
}

// Called when transmitting of a symbol is done
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
	}
	else
	{
		actual_bit++;
		actual_bit_of_byte--;

		// set duty cycle for the next bit
		SetPCA0DutyCycle();
	}
}

// Called when receiving
void PCA0_channel1EventCb()
{
	// Store most recent capture value
	// FIXME: Why do we multiply this by 10?
	uint16_t current_capture_value = PCA0CP1 * 10;
	uint16_t previous_capture_value_pos;
	uint16_t capture_period_neg;
	uint8_t current_duty_cycle;

	static uint16_t capture_period_pos;
	static uint16_t previous_capture_value_neg;
	static uint16_t low_pulse_time;
	static uint8_t used_protocol;

	// positive edge
	if (R_DATA)
	{
		// Update previous capture value with most recent info.
		previous_capture_value_pos = current_capture_value;

		// Calculate capture period from last two values.
		capture_period_neg = current_capture_value - previous_capture_value_neg;

		// do sniffing by mode
		switch (rf_listen_mode)
		{
			// do sniffing by duty cycle mode
			case MODE_DUTY_CYCLE:
				switch (rf_state)
				{
					// check if we receive a sync
					case RF_IDLE:
						// check first if last decoded RF signal was cleared
						// FIXME: Should we just drop the incoming message on the ground?
						if (rf_data_status != 0) {
							break;
						}

						used_protocol = IdentifyRFProtocol(desired_rf_protocol, capture_period_pos, capture_period_neg);

						// check if a matching protocol got found
						if (used_protocol != NO_PROTOCOL_FOUND)
						{
							sync_high = capture_period_pos;
							sync_low = capture_period_neg;
							actual_bit_of_byte = 8;
							actual_byte = 0;
							actual_bit = 0;
							actual_sync_bit = 0;
							low_pulse_time = 0;
							memset(rf_data, 0, sizeof(rf_data));
							rf_state = RF_IN_SYNC;
						}
						break; // switch rf_state

					// one matching sync got received
					case RF_IN_SYNC:
						// at first skip SYNC bits
						if ((protocol_data[used_protocol].sync_bit_count > 0) &&
							(actual_sync_bit < protocol_data[used_protocol].sync_bit_count))
						{
							actual_sync_bit++;
							break;
						}

						// check the rest of the bits
						actual_bit_of_byte--;
						actual_bit++;

						// calculate current duty cycle
						current_duty_cycle = (100 * (uint32_t) capture_period_pos) / ((uint32_t) capture_period_pos + (uint32_t) capture_period_neg);

						if (((current_duty_cycle > (protocol_data[used_protocol].bit_high_duty - DUTY_CYCLE_TOLERANCE)) &&
							(current_duty_cycle < (protocol_data[used_protocol].bit_high_duty + DUTY_CYCLE_TOLERANCE)) &&
							(actual_bit < protocol_data[used_protocol].bit_count)) ||
							// the duty cycle can not be used for the last bit because of the missing rising edge on the end
							((capture_period_pos > low_pulse_time) && (actual_bit == protocol_data[used_protocol].bit_count))
							)
						{
							// backup last bit high time
							bit_high = capture_period_pos;
							LED = LED_ON;
							rf_data[(actual_bit - 1) / 8] |= (1 << actual_bit_of_byte);
						} else {
							// backup last bit high time
							bit_low = capture_period_pos;
							LED = LED_OFF;
							// backup low bit pulse time to be able to determine the last bit
							if (capture_period_pos > low_pulse_time) {
								low_pulse_time = capture_period_pos;
							}
						}

						if (actual_bit_of_byte == 0) {
							actual_bit_of_byte = 8;
						}

						// check if all bits for this protocol got received
						if (actual_bit == protocol_data[used_protocol].bit_count)
						{
							rf_data_status = used_protocol | RF_DATA_RECEIVED_MASK;
							LED = LED_OFF;
							rf_state = RF_IDLE;
						}
						break;
				}
				break; // switch(rf_sniffing_mode)

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
		capture_period_pos = current_capture_value - previous_capture_value_pos;

		// do sniffing by mode
		if (rf_listen_mode == MODE_BUCKET) {
			Bucket_Received(capture_period_pos);
		}
	}
}

void PCA0_channel2EventCb()
{
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
				if ((period_neg > (protocol_data[used_protocol].sync_low - SYNC_TOLERANCE)) &&
					(period_neg < (protocol_data[used_protocol].sync_low + SYNC_TOLERANCE)))
				{
					if ((protocol_data[used_protocol].sync_high == 0) ||
					   ((period_pos > (protocol_data[used_protocol].sync_high - SYNC_TOLERANCE)) &&
						(period_pos < (protocol_data[used_protocol].sync_high + SYNC_TOLERANCE))))
					{
						protocol_found = used_protocol;
						break;
					}
				}
			}
			break;

		// check other protocols
		default:
			used_protocol = PCA0_GetProtocolIndex(identifier);

			// check if identifier got found in list
			if (used_protocol == NO_PROTOCOL_FOUND) {
				break;
			}

			if ((period_neg > (protocol_data[used_protocol].sync_low - SYNC_TOLERANCE_0xA1)) &&
				(period_neg < (protocol_data[used_protocol].sync_low + SYNC_TOLERANCE_0xA1)))
			{
				if ((protocol_data[used_protocol].sync_high == 0) ||
				   ((period_pos > (protocol_data[used_protocol].sync_high - SYNC_TOLERANCE_0xA1)) &&
					(period_pos < (protocol_data[used_protocol].sync_high + SYNC_TOLERANCE_0xA1))))
				{
					protocol_found = used_protocol;
					break;
				}
			}
			break;
	}

	return protocol_found;
}

//-----------------------------------------------------------------------------
// Send RF SYNC HIGH/LOW Routine
//-----------------------------------------------------------------------------
static void SendRF_Sync(void)
{
	// enable P0.0 for I/O control
	XBR1 &= ~XBR1_PCA0ME__CEX0_CEX1;

	// Send ASK/On-off keying to SYN115 chip
	T_DATA = 1;

	// What is happening here? Activation?
	InitTimer_ms(TIMER3, 1, 7);
	WaitTimerFinished(TIMER3);
	T_DATA = 0;
	InitTimer_us(TIMER3, 10, 100);
	WaitTimerFinished(TIMER3);

	T_DATA = 1;
	InitTimer_us(TIMER3, 5, sync_high);
	WaitTimerFinished(TIMER3);
	T_DATA = 0;
	InitTimer_us(TIMER3, 5, sync_low);
	WaitTimerFinished(TIMER3);

	// disable P0.0 for I/O control, enter PCA mode
	XBR1 |= XBR1_PCA0ME__CEX0_CEX1;
}

uint8_t PCA0_GetProtocolIndex(uint8_t identifier)
{
	uint8_t i;
	uint8_t protocol_index = NO_PROTOCOL_FOUND;

	// check first for valid identifier
	if ((identifier > UNKNOWN_IDENTIFIER) && (identifier < NO_PROTOCOL_FOUND))
	{
		// find protocol index by identifier
		for(i = 0; i < PROTOCOLCOUNT; i++)
		{
			if (protocol_data[i].identifier == identifier)
			{
				protocol_index = i;
				break;
			}
		}
	}

	return protocol_index;
}

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
	duty_cycle_high = (uint16_t)((bit_high_duty * 0xFF) / 100);
	duty_cycle_low = (uint16_t)((bit_low_duty * 0xFF) / 100);

	bit_count = bitcount;

	// enable interrupt for RF transmitting
	PCA0CPM0 |= PCA0CPM0_ECCF__ENABLED;
	PCA0PWM |= PCA0PWM_ECOV__COVF_MASK_ENABLED;

	// disable interrupt for RF receiving
	PCA0CPM1 &= ~PCA0CPM1_ECCF__ENABLED;

	/***********************************************************************
	 - PCA Counter/Timer Low Byte = 0xFF, why?
	 ***********************************************************************/
	PCA0L = (0xFF << PCA0L_PCA0L__SHIFT);
}

static void SetPCA0DutyCycle(void)
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

void SetTimer0Overflow(uint8_t T0_Overflow)
{
	/***********************************************************************
	 - Timer 0 High Byte = T0_Overflow
	 ***********************************************************************/
	TH0 = (T0_Overflow << TH0_TH0__SHIFT);
}

void PCA0_StartRFTransmit(uint8_t payload_pos)
{
	actual_bit_of_byte = 7;
	actual_bit = 1;
	actual_byte = payload_pos;
	rf_state = RF_TRANSMITTING;

	// set first bit to be in sync when PCA0 is starting
	SetPCA0DutyCycle();

	// make RF sync pulse
	// FIXME: According to PT2260 docs, sync pulse comes after payload
	SendRF_Sync();

	PCA0_run();
}

void PCA0_StopRFTransmit(void)
{
	// set duty cycle to zero
	PCA0_writeChannel(PCA0_CHAN0, 0);

	// disable interrupt for RF transmitting
	PCA0CPM0 &= ~PCA0CPM0_ECCF__ENABLED;
	PCA0PWM &= ~PCA0PWM_ECOV__COVF_MASK_ENABLED;

	// stop PCA
	PCA0_halt();

	// clear all interrupt flags of PCA0
	PCA0CN0 &= ~(PCA0CN0_CF__BMASK
	  		   | PCA0CN0_CCF0__BMASK
	  		   | PCA0CN0_CCF1__BMASK
	  		   | PCA0CN0_CCF2__BMASK);

	// enable P0.0 for I/O control
	XBR1 &= ~XBR1_PCA0ME__CEX0_CEX1;
	// switch to low
	T_DATA = 0;
	// disable P0.0 for I/O control, enter PCA mode
	XBR1 |= XBR1_PCA0ME__CEX0_CEX1;

	rf_state = RF_FINISHED;
}

void PCA0_StartRFListen(void)
{
	// restore timer to 100000 Hz, 10 s interval
	// FIXME: I don't understand this
	SetTimer0Overflow(0x0B);

	// enable interrupt for RF receiving
	PCA0CPM1 |= PCA0CPM1_ECCF__ENABLED;

	// disable interrupt for RF transmission
	PCA0CPM0 &= ~PCA0CPM0_ECCF__ENABLED;
	PCA0PWM &= ~PCA0PWM_ECOV__COVF_MASK_ENABLED;

	// start PCA
	PCA0_run();

	rf_state = RF_IDLE;
	rf_data_status = 0;
}

void PCA0_StopRFListen(void)
{
	// stop PCA
	PCA0_halt();

	// clear all interrupt flags of PCA0
	PCA0CN0 &= ~(PCA0CN0_CF__BMASK
	  		                       | PCA0CN0_CCF0__BMASK
	  		                       | PCA0CN0_CCF1__BMASK
	  		                       | PCA0CN0_CCF2__BMASK);

	// disable interrupt for RF receiving
	PCA0CPM1 &= ~PCA0CPM1_ECCF__ENABLED;

	rf_state = RF_IDLE;
}

//-----------------------------------------------------------------------------
// Send generic signal based on n time bucket pairs (high/low timing)
//-----------------------------------------------------------------------------
void SendRFBuckets(const uint16_t bkts[], const uint8_t rfdata[], uint8_t n, uint8_t repeats)
{
	// disable interrupts for RF receiving and transmitting
	PCA0CPM1 &= ~PCA0CPM1_ECCF__ENABLED;
	PCA0CPM0 &= ~PCA0CPM0_ECCF__ENABLED;
	PCA0PWM &= ~PCA0PWM_ECOV__COVF_MASK_ENABLED;

	XBR1 &= ~XBR1_PCA0ME__CEX0_CEX1;	// enable P0.0 for I/O control

	T_DATA = 1;							// switch to high
	InitTimer_ms(TIMER3, 1, 7);					// start timer (7ms)
	WaitTimerFinished(TIMER3);				// wait until timer has finished

	T_DATA = 0;							// switch to low
	InitTimer_us(TIMER3, 10, 100);				// start timer (1ms)
	WaitTimerFinished(TIMER3);				// wait until timer has finished

	do
	{
		uint8_t i;

		// transmit n bucket pairs
		for (i = 0; i < n; i++)
		{
			// high bucket
			uint16_t j = bkts[rfdata[i] >> 4];
			T_DATA = 1;

			delay_us(j);

			// low bucket
			j = bkts[rfdata[i] & 0x0F];
			T_DATA = 0;
			delay_us(j);
		}
		LED = !LED;
	} while (repeats-- != 0);				// how many times do I need to repeat?

	// disable P0.0 for I/O control, enter PCA mode
	XBR1 |= XBR1_PCA0ME__CEX0_CEX1;
	LED = LED_OFF;
}

static bool probablyFooter(uint16_t duration)
{
	return duration >= MIN_FOOTER_LENGTH;
}

static bool matchesFooter(uint16_t duration)
{
  uint16_t footer_delta = bucket_sync / 4;
  return (((bucket_sync - footer_delta) < duration) && (duration < (bucket_sync + footer_delta)));
}

static bool findBucket(uint16_t duration, uint8_t *index)
{
	bool ret = false;
	uint8_t i;
	for (i = 0; i < bucket_count; i++)
	{
		uint16_t delta = duration / 4 + duration / 8;
		if (((buckets[i] - delta) < duration) && (duration < (buckets[i] + delta)))
		{
			if (index != NULL) {
				*index = i;
			}

			ret = true;
			break;
		}
	}

	return ret;
}

void Bucket_Received(uint16_t duration)
{
	uint8_t bucket_index;

	switch (rf_state)
	{
		// check if we receive a sync
		case RF_IDLE:
			LED = LED_OFF;
			// check first if last decoded RF signal was cleared
			if (rf_data_status != 0)
				break;

			if (probablyFooter(duration))
			{
				bucket_count = 0;
				bucket_sync = duration;
				rf_state = RF_IN_SYNC;
				LED = LED_ON;
			}
			break;
		// one matching sync got received
		case RF_IN_SYNC:
			// check if duration is longer than sync bucket +25% and restart
			if (duration > (bucket_sync + bucket_sync / 4))
			{
				// this bucket looks like the sync bucket
				bucket_sync = duration;
				bucket_count = 0;
			}
			// check if this bucket is a sync bucket, receive is complete
			else if (matchesFooter(duration) && (bucket_count > 0))
			{
				// all buckets got received, start decoding on the next repeat
				actual_byte = 0;
				actual_bit_of_byte = 4;
				rf_state = RF_DECODE_BUCKET;
			}
			// check if useful bucket got received
			else if (duration >= MIN_PULSE_LENGTH)
			{
				// check if bucket was already received
				if (findBucket(duration, &bucket_index))
				{
					// make new average of this bucket
					buckets[bucket_index] = (buckets[bucket_index] + duration) / 2;
				}
				else
				{
					// new bucket received, add to array
					buckets[bucket_count] = duration;
					bucket_count++;

					// check if maximum of array got reached
					if (bucket_count > sizeof(buckets))
					{
						bucket_count = 0;
						// restart sync
						rf_state = RF_IDLE;
					}
				}
			}
			else
			{
				// restart sync
				rf_state = RF_IDLE;
			}
			break;

		// do decoding of the received buckets
		case RF_DECODE_BUCKET:
			// toggle led
			LED = !LED;
			// check if sync bucket got received
			if (matchesFooter(duration))
			{
				rf_data_status |= RF_DATA_RECEIVED_MASK;
				LED = LED_OFF;
				rf_state = RF_IDLE;
			}
			// check if bucket can be decoded
			else if (findBucket(duration, &bucket_index))
			{
				if (actual_bit_of_byte == 4)
				{
					rf_data[actual_byte] = bucket_index << 4;
					actual_bit_of_byte = 0;
				}
				else
				{
					rf_data[actual_byte] |= (bucket_index & 0x0F);
					actual_byte++;
					actual_bit_of_byte = 4;

					// check if maximum of array got reached
					if (actual_byte > sizeof(rf_data))
					{
						bucket_count = 0;
						// restart sync
						rf_state = RF_IDLE;
					}
				}
			}
			else
			{
				// bucket not found in list, restart
				bucket_count = 0;
				// restart sync
				rf_state = RF_IDLE;
			}
			break;
		}
}
