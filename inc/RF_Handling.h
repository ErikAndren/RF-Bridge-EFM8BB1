/*
 * RF_Handling.h
 *
 *  Created on: 28.11.2017
 *      Author:
 */

#ifndef INC_RF_HANDLING_H_
#define INC_RF_HANDLING_H_

#include "uart.h"

extern uint8_t rf_identify_protocol(uint8_t identifier, uint16_t period_pos, uint16_t period_neg);
extern void rf_tx_start(uint16_t sync_high_in, uint16_t sync_low_in, uint16_t bit_high_time, uint8_t bit_high_duty,
		uint16_t bit_low_time, uint8_t bit_low_duty, uint8_t bitcount, uint8_t payload_ptr);
extern void rf_tx_set_duty_cycle(void);
extern void rf_tx_stop(void);
extern void rf_rx_start(void);
extern void rf_rx_stop(void);

extern void rf_tx_handle(uart_command_t cmd, uint8_t *repeats);
extern void rf_rx_handle(uart_command_t cmd);

// 112 byte == 896 bits, so a RF signal with maximum of 896 bits is possible
// for bucket transmission, this depends on the number of buckets.
// E.g. 4 buckets have a total overhead of 11, allowing 101 bits (high/low pairs)
#define RF_DATA_BUFFERSIZE		112

#define SWB(val) ((((val) >> 8) & 0x00FF) | (((val) << 8) & 0xFF00))

typedef enum
{
	RF_IDLE,
	RF_IN_SYNC,
	RF_DATA,
	RF_DECODE_BUCKET,
	RF_TRANSMITTING,
	RF_FINISHED
} rf_state_t;

extern SI_SEGMENT_VARIABLE(rf_data[RF_DATA_BUFFERSIZE], uint8_t, SI_SEG_XDATA);

extern SI_SEGMENT_VARIABLE(rf_protocol, uint8_t, SI_SEG_XDATA);
extern SI_SEGMENT_VARIABLE(rf_state, rf_state_t, SI_SEG_XDATA);
extern SI_SEGMENT_VARIABLE(desired_rf_protocol, uint8_t, SI_SEG_XDATA);

extern SI_SEGMENT_VARIABLE(duty_cycle_high, uint8_t, SI_SEG_XDATA);
extern SI_SEGMENT_VARIABLE(duty_cycle_low, uint8_t, SI_SEG_XDATA);
extern SI_SEGMENT_VARIABLE(t0_high, uint8_t, SI_SEG_XDATA);
extern SI_SEGMENT_VARIABLE(t0_low, uint8_t, SI_SEG_XDATA);
extern SI_SEGMENT_VARIABLE(sync_high, uint16_t, SI_SEG_XDATA);
extern SI_SEGMENT_VARIABLE(sync_low, uint16_t, SI_SEG_XDATA);
extern SI_SEGMENT_VARIABLE(bit_high, uint16_t, SI_SEG_XDATA);
extern SI_SEGMENT_VARIABLE(bit_low, uint16_t, SI_SEG_XDATA);
extern SI_SEGMENT_VARIABLE(bit_count, uint8_t, SI_SEG_XDATA);

extern SI_SEGMENT_VARIABLE(actual_bit, uint8_t, SI_SEG_XDATA);
extern SI_SEGMENT_VARIABLE(actual_sync_bit, uint8_t, SI_SEG_XDATA);
extern SI_SEGMENT_VARIABLE(actual_byte, uint8_t, SI_SEG_XDATA);

extern SI_SEGMENT_VARIABLE(bucket_sync, uint16_t, SI_SEG_XDATA);
extern SI_SEGMENT_VARIABLE(buckets[BUCKET_MAX], uint16_t, SI_SEG_XDATA);
extern SI_SEGMENT_VARIABLE(bucket_count, uint8_t, SI_SEG_XDATA);

extern SI_SEGMENT_VARIABLE(pos_pulse_len, uint16_t, SI_SEG_DATA);
extern SI_SEGMENT_VARIABLE(neg_pulse_len, uint16_t, SI_SEG_DATA);
extern SI_SEGMENT_VARIABLE(low_pulse_time, uint16_t, SI_SEG_DATA);
extern SI_SEGMENT_VARIABLE(desired_rf_protocol, uint8_t, SI_SEG_XDATA);
extern SI_SEGMENT_VARIABLE(last_desired_rf_protocol, uint8_t, SI_SEG_DATA);
extern SI_SEGMENT_VARIABLE(last_uart_command, uart_command_t, SI_SEG_DATA);
extern SI_SEGMENT_VARIABLE(uart_command, uart_command_t, SI_SEG_DATA);
extern SI_SEGMENT_VARIABLE(next_uart_command, uart_command_t, SI_SEG_DATA);

extern SI_SEGMENT_VARIABLE(waiting_for_uart_ack, bool, SI_SEG_DATA);
extern SI_SEGMENT_VARIABLE(uart_cmd_retry_cnt, uint8_t, SI_SEG_DATA);

#define RFIN_CMD_TIMEOUT_MS 1000
#define RFIN_CMD_RETRIES 0

#endif /* INC_RF_HANDLING_H_ */
