/*
 * RF_Buckets.c
 *
 *  Created on: 18 mars 2018
 *      Author: erik
 */

#include "Globals.h"
#include "RF_Handling.h"
#include "RF_Protocols.h"
#include "pca_0.h"
#include "uart.h"
#include "Timer.h"
#include "Delay.h"

SI_SEGMENT_VARIABLE(actual_bit_of_byte, uint8_t, SI_SEG_XDATA) = 0;

//-----------------------------------------------------------------------------
// Send generic signal based on n time bucket pairs (high/low timing)
//-----------------------------------------------------------------------------
void SendRFBuckets(const uint16_t bkts[], const uint8_t rfdata[], uint8_t bucket_pairs, uint8_t repeats)
{
	// disable interrupts for RF receiving and transmitting
	PCA0CPM1 &= ~PCA0CPM1_ECCF__ENABLED;
	PCA0CPM0 &= ~PCA0CPM0_ECCF__ENABLED;
	PCA0PWM &= ~PCA0PWM_ECOV__COVF_MASK_ENABLED;

	XBR1 &= ~XBR1_PCA0ME__CEX0_CEX1;	// enable P0.0 for I/O control

	//FIXME: Why are we doing this separately?
	T_DATA = 1;							// switch to high
	InitTimer_ms(TIMER3, 1, 7);					// start timer (7ms)
	WaitTimerFinished(TIMER3);				// wait until timer has finished

	T_DATA = 0;							// switch to low
	InitTimer_us(TIMER3, 10, 1000);				// start timer (1ms)
	WaitTimerFinished(TIMER3);				// wait until timer has finished

	do
	{
		uint8_t i;
		for (i = 0; i < bucket_pairs; i++)
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
				LED = LED_OFF;
				rf_state = RF_FINISHED;
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
