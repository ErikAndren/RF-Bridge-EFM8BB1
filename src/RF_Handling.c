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
#include <pca_0.h>
#include "uart_0.h"
#include "uart.h"
#include "Timer.h"
#include "Delay.h"

SI_SEGMENT_VARIABLE(last_uart_command, uart_command_t, SI_SEG_DATA);
SI_SEGMENT_VARIABLE(uart_command, uart_command_t, SI_SEG_DATA);
SI_SEGMENT_VARIABLE(next_uart_command, uart_command_t, SI_SEG_DATA);

SI_SEGMENT_VARIABLE(rf_data[RF_DATA_BUFFERSIZE], uint8_t, SI_SEG_XDATA);
SI_SEGMENT_VARIABLE(rf_protocol, uint8_t, SI_SEG_XDATA) = 0;
SI_SEGMENT_VARIABLE(rf_state, rf_state_t, SI_SEG_XDATA) = RF_IDLE;
SI_SEGMENT_VARIABLE(desired_rf_protocol, uint8_t, SI_SEG_XDATA) = UNKNOWN_PROTOCOL;
SI_SEGMENT_VARIABLE(last_desired_rf_protocol, uint8_t, SI_SEG_DATA);

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
SI_SEGMENT_VARIABLE(actual_byte, uint8_t, SI_SEG_XDATA) = 0;

SI_SEGMENT_VARIABLE(waiting_for_uart_ack, bool, SI_SEG_DATA) = false;
SI_SEGMENT_VARIABLE(uart_cmd_retry_cnt, uint8_t, SI_SEG_DATA) = 0;

SI_SEGMENT_VARIABLE(pos_pulse_len, uint16_t, SI_SEG_DATA) = 0;
SI_SEGMENT_VARIABLE(neg_pulse_len, uint16_t, SI_SEG_DATA) = 0;
SI_SEGMENT_VARIABLE(low_pulse_time, uint16_t, SI_SEG_DATA) = 0;

// up to 8 timing buckets for MODE_BUCKET
SI_SEGMENT_VARIABLE(bucket_sync, uint16_t, SI_SEG_XDATA);
SI_SEGMENT_VARIABLE(buckets[BUCKET_MAX], uint16_t, SI_SEG_XDATA);	// -1 because of the bucket_sync
SI_SEGMENT_VARIABLE(bucket_count, uint8_t, SI_SEG_XDATA) = 0;

#define PWM_8BIT_CNT 256

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
void rf_rx_start(void)
{
	// restore timer to 100000 Hz, 10 s interval
	// 245 cc's * 40.8 ns = 50 us = 100000 Hz = 100 kHz
	TH0 = 256 - (SYSCLK / RX_SAMPLE_RATE);

	// enable interrupt for RF reception
	PCA0CPM1 |= PCA0CPM1_ECCF__ENABLED;

	rf_state = RF_IDLE;

	PCA0_run();
}

void rf_rx_stop(void)
{
	PCA0_halt();

	// clear all interrupt flags of PCA0
	PCA0CN0 &= ~(PCA0CN0_CF__BMASK | PCA0CN0_CCF0__BMASK | PCA0CN0_CCF1__BMASK | PCA0CN0_CCF2__BMASK);

	// disable interrupt for RF receiving
	PCA0CPM1 &= ~PCA0CPM1_ECCF__ENABLED;
}

uint8_t rf_identify_protocol(uint8_t protocol, uint16_t period_pos, uint16_t period_neg)
{
	switch (protocol) {
		// protocol is undefined, do loop through all protocols
		case UNKNOWN_PROTOCOL:
			// check all protocols
			for (protocol = 0; protocol < PROTOCOL_COUNT; protocol++) {
				if ((period_neg > (PROTOCOLS[protocol].sync_low - PROTOCOLS[protocol].sync_tolerance)) &&
					(period_neg < (PROTOCOLS[protocol].sync_low + PROTOCOLS[protocol].sync_tolerance))) {

					if ((PROTOCOLS[protocol].sync_high == 0) ||
						((period_pos > (PROTOCOLS[protocol].sync_high - PROTOCOLS[protocol].sync_tolerance)) &&
						(period_pos < (PROTOCOLS[protocol].sync_high + PROTOCOLS[protocol].sync_tolerance)))) {
						return protocol;
					}
				}
			}
			break;

		// check other protocols
		default:
			// check if identifier got found in list
			if (protocol >= PROTOCOL_COUNT) {
				break;
			}

			if ((period_neg > (PROTOCOLS[protocol].sync_low - PROTOCOLS[protocol].sync_tolerance)) &&
				(period_neg < (PROTOCOLS[protocol].sync_low + PROTOCOLS[protocol].sync_tolerance))) {
				if ((PROTOCOLS[protocol].sync_high == 0) ||
				   ((period_pos > (PROTOCOLS[protocol].sync_high - PROTOCOLS[protocol].sync_tolerance)) &&
					(period_pos < (PROTOCOLS[protocol].sync_high + PROTOCOLS[protocol].sync_tolerance)))) {
					return protocol;
				}
			}
			break;
	}

	return NO_PROTOCOL_FOUND;
}
void rf_rx_handle(uart_command_t cmd) {
	// Negative pulse end implies a previous positive pulse i.e. one period is complete
	if (neg_pulse_len > 0) {
		switch (rf_state) {
		case RF_IDLE:
			rf_protocol = rf_identify_protocol(desired_rf_protocol, pos_pulse_len, neg_pulse_len);
			if (rf_protocol != NO_PROTOCOL_FOUND) {
				sync_high = pos_pulse_len;
				sync_low = neg_pulse_len;
				actual_byte = 0;
				actual_bit = 0;
				low_pulse_time = 0;
				if (PROTOCOLS[rf_protocol].additional_sync_bits > 0) {
					rf_state = RF_IN_SYNC;
				} else {
					rf_state = RF_DATA;
				}
				rf_data[0] = 0;
				LED = LED_ON;
			}
			break;

		case RF_IN_SYNC:
			// Skip additional SYNC bits
			actual_bit++;

			if (actual_bit == PROTOCOLS[rf_protocol].additional_sync_bits) {
				actual_bit = 0;
				rf_state = RF_DATA;
			}
			break;

		case RF_DATA:
		{
			uint8_t duty_cycle;
			LED = !LED;

			actual_bit++;
			duty_cycle = (100 * (uint32_t) pos_pulse_len) / ((uint32_t) pos_pulse_len + (uint32_t) neg_pulse_len);

			// Only set the bit if it passes the bit high duty filter
			if (((duty_cycle > (PROTOCOLS[rf_protocol].bit_high_duty - DUTY_CYCLE_TOLERANCE)) &&
				 (duty_cycle < (PROTOCOLS[rf_protocol].bit_high_duty + DUTY_CYCLE_TOLERANCE)) &&
				 (actual_bit < PROTOCOLS[rf_protocol].bit_count)) ||
					// The duty cycle can not be used for the last bit because of the missing rising edge on the end
					// A new rising edge will eventually come as the input pin jitters but we don't know in time when this is.
					// Another way to solve this is by setting a timer on the last positive pulse
					// This is probably good enough. But we are not checking against duty cycle limitations.
					// Instead just pulse that is the longest
					((pos_pulse_len > low_pulse_time) && (actual_bit == PROTOCOLS[rf_protocol].bit_count))) {
				bit_high = pos_pulse_len;
				rf_data[actual_byte] |= (1 << ((actual_bit - 1) % 8));

			} else if ((duty_cycle > (PROTOCOLS[rf_protocol].bit_low_duty - DUTY_CYCLE_TOLERANCE)) &&
					   (duty_cycle < (PROTOCOLS[rf_protocol].bit_low_duty + DUTY_CYCLE_TOLERANCE)) ||
					   (actual_bit == PROTOCOLS[rf_protocol].bit_count)) {
				bit_low = pos_pulse_len;

				// backup low bit pulse time to be able to determine the last bit
				if (pos_pulse_len > low_pulse_time) {
					low_pulse_time = pos_pulse_len;
				}
			} else {
				// Bit length is out of spec, revert back to idle state
				rf_state = RF_IDLE;
				break;
			}

			if ((actual_bit % 8) == 0) {
				// Clear next byte
				actual_byte++;
				rf_data[actual_byte] = 0;
			}

			if (actual_bit == PROTOCOLS[rf_protocol].bit_count) {
				LED = LED_OFF;
				rf_state = RF_FINISHED;
			}
			break;
		}

		case RF_FINISHED:
			// Do not accept more incoming RF message until the previous transmission has been
			// acknowledged (or timed out) by the host
			rf_rx_stop();

			switch (cmd) {
			case RF_CODE_IN:
				// Received RF code, transmit to ESP8266 and wait for ack
				// FIXME: Waiting for an ack makes us blind to new incoming codes.
				// One improvement would be to multi-thread and having a queue
				// Not sure the processor has memory enough to handle this though
				uart_cmd_retry_cnt = 0;
				waiting_for_uart_ack = true;

				InitTimer_ms(TIMER3, 1, RFIN_CMD_TIMEOUT_MS);
				uart_put_RF_CODE_Data(RF_CODE_IN);
				break;

			case RF_PROTOCOL_SNIFFING_ON:
				uart_cmd_retry_cnt = 0;
				waiting_for_uart_ack = true;

				InitTimer_ms(TIMER3, 1, RFIN_CMD_TIMEOUT_MS);
				uart_put_RF_Data(RF_PROTOCOL_SNIFFING_ON);
				break;

			default:
				break;
			}
			break;

		default:
			rf_state = RF_IDLE;
			break;
		}

		neg_pulse_len = 0;
	}
}

// Transmission path
void rf_tx_handle(uart_command_t cmd, uint8_t *repeats) {
	switch(rf_state)
	{
	// init and start RF transmit
	case RF_IDLE:
	{
		if (cmd == RF_CODE_OUT) {
			uint16_t sync_low = (uint16_t) ((uint32_t) (*(uint16_t *) &rf_data[SONOFF_TSYN_POS]));
			uint16_t sync_high = (sync_low / PT2260_SYNC_LOW) * PT2260_SYNC_HIGH;
			uint16_t bit_high_t = *(uint16_t *) &rf_data[SONOFF_THIGH_POS];
			uint16_t bit_low_t = *(uint16_t *) &rf_data[SONOFF_TLOW_POS];

			rf_rx_stop();
			start_rf_tx(sync_high, sync_low,
					bit_high_t, PROTOCOLS[PT2260_IDENTIFIER].bit_high_duty,
					bit_low_t, PROTOCOLS[PT2260_IDENTIFIER].bit_low_duty,
					PROTOCOLS[PT2260_IDENTIFIER].bit_count, SONOFF_DATA_POS);
		} else if (cmd == RF_PROTOCOL_OUT) {
			if (rf_data[RF_PROTOCOL_IDENT_POS] == CUSTOM_PROTOCOL_IDENT) {
				uint16_t sync_high = *(uint16_t *) &rf_data[CUSTOM_PROTOCOL_SYNC_HIGH_POS];
				uint16_t sync_low = *(uint16_t *) &rf_data[CUSTOM_PROTOCOL_SYNC_LOW_POS];
				uint16_t bit_high_t = *(uint16_t *) &rf_data[CUSTOM_PROTOCOL_BIT_HIGH_TIME_POS];
				uint8_t bit_high_duty = rf_data[CUSTOM_PROTOCOL_BIT_HIGH_DUTY_POS];
				uint16_t bit_low_t = *(uint16_t *) &rf_data[CUSTOM_PROTOCOL_BIT_LOW_TIME_POS];
				uint8_t bit_low_duty = rf_data[CUSTOM_PROTOCOL_BIT_LOW_DUTY_POS];
				uint8_t bit_count_t = rf_data[CUSTOM_PROTOCOL_BIT_COUNT_POS];

				rf_rx_stop();
				start_rf_tx(
						sync_high,
						sync_low,
						bit_high_t,
						bit_high_duty,
						bit_low_t,
						bit_low_duty,
						bit_count_t,
						CUSTOM_PROTOCOL_DATA_POS);
			} else if (rf_data[RF_PROTOCOL_IDENT_POS] < PROTOCOL_COUNT) {
				uint8_t protocol_index = rf_data[RF_PROTOCOL_IDENT_POS];

				rf_rx_stop();
				start_rf_tx(
					PROTOCOLS[protocol_index].sync_high, PROTOCOLS[protocol_index].sync_low,
					PROTOCOLS[protocol_index].bit_high_time, PROTOCOLS[protocol_index].bit_high_duty,
					PROTOCOLS[protocol_index].bit_low_time, PROTOCOLS[protocol_index].bit_low_duty,
					PROTOCOLS[protocol_index].bit_count, RF_PROTOCOL_START_POS);
			}
		}
		break;
	}

	case RF_TRANSMITTING:
		break;

	// Will be called once transmit is done
	case RF_FINISHED:
		(*repeats)--;
		if (*repeats > 0) {
			rf_state = RF_IDLE;
		} else {
			desired_rf_protocol = last_desired_rf_protocol;
			uart_command = last_uart_command;

			rf_rx_start();
			uart_put_command(RF_CODE_ACK);
		}
		break;

	default:
		rf_state = RF_IDLE;
		break;
	} // rf_state
}

static void send_rf_tx_sync(void)
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

void start_rf_tx(uint16_t sync_high_in, uint16_t sync_low_in,
					 uint16_t bit_high_time, uint8_t bit_high_duty,
		             uint16_t bit_low_time, uint8_t bit_low_duty,
				     uint8_t bitcount, uint8_t payload_pos)
{
	uint16_t bit_period_us;

	// set global variable
	sync_high = sync_high_in;
	sync_low = sync_low_in;

	// Calculate the pulse period, for high and low pulses - they might differ
	// This is the reload value used to reload TL0 when it is in mode 2. See 18.3.2.1 in reference manual
	// FIXME: This logic needs to be explained.
	// Calculate the bit period (us)
	bit_period_us = (100 * (uint32_t) bit_high_time) / bit_high_duty;
	
	// Bit period is hacked up into 256 parts, each part is incremented when the timer 0 overflows
	// What does 1000000 represent? Is it related to that the bit period is in us i. e. 1 MHz
	// Frequency of timer overflows that needs to be attained during one second
	t0_high = (uint8_t) (256 - ((uint32_t) SYSCLK / (PWM_8BIT_CNT * (1000000 / (uint32_t) bit_period_us))));

	bit_period_us = (100 * (uint32_t) bit_low_time) / bit_low_duty;
	t0_low = (uint8_t) (256 - ((uint32_t) SYSCLK / (PWM_8BIT_CNT * (1000000 / (uint32_t) bit_period_us))));

	// convert duty cycle from percent to per 256
	duty_cycle_high = (uint16_t) ((bit_high_duty * PWM_8BIT_CNT) / 100);
	duty_cycle_low = (uint16_t) ((bit_low_duty * PWM_8BIT_CNT) / 100);

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
	rf_tx_set_duty_cycle();

	// According to PT2260 docs, sync pulse comes after payload
	// Yes, but not in the EV-protocol. Doesn't matter if multiple transmits are sent
	send_rf_tx_sync();
	PCA0_run();
}

// This defines the falling edge of the pulse, in PCA0 cycles
// Of the 256 PCA counts, where shall the falling edge occur?
static void rf_tx_set_duty_cycle(void)
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

// Called when rising edge (falling non. inv) is generated (overflow)
// This defines the length (frequency)
// Starts the high part of the pulse
void PCA0_intermediateOverflowCb(void)
{
	// This will effectively toggle how fast the counter will wrap around
	// Both timer 0 and PCA counter works in 8 bit mode
	// Symbols can have different lengths, adjust for that here
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
		rf_tx_stop();
	} else {
		// Start transmission of next bit
		//FIXME: Is this really safe? Couldn't this create a problem if the low start is moved a bit ahead?
		// PCA0 counter is not reset, and so if this event is called, and the updated match value is slightly beyond it will be called again
		// Missed bits?
		rf_tx_set_duty_cycle();
	}
}

void rf_tx_stop(void)
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

	//FIXME: Is this really necessary? We only stop transmission after a full pulse i.e we're ending on low anyway
	// enable P0.0 for I/O control
	XBR1 &= ~XBR1_PCA0ME__CEX0_CEX1;

	T_DATA = 0;

	// disable P0.0 for I/O control, enter PCA mode
	XBR1 |= XBR1_PCA0ME__CEX0_CEX1;

	rf_state = RF_FINISHED;
}
