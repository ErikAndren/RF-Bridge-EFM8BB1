/*
 * RF_Handling.h
 *
 *  Created on: 28.11.2017
 *      Author:
 */

#ifndef INC_RF_HANDLING_H_
#define INC_RF_HANDLING_H_

extern uint8_t IdentifyRFProtocol(uint8_t identifier, uint16_t period_pos, uint16_t period_neg);
extern uint8_t GetProtocolIndex(uint8_t identifier);
extern void PCA0_InitRFTransmit(uint16_t sync_high_in, uint16_t sync_low_in, uint16_t bit_high_time, uint8_t bit_high_duty,
		uint16_t bit_low_time, uint8_t bit_low_duty, uint8_t bitcount);
extern void PCA0_SetDutyCycle(void);
extern void PCA0_StartRFTransmit(uint8_t payload_ptr);
extern void PCA0_StopRFTransmit(void);
extern void PCA0_StartRFListen(void);
extern void PCA0_StopRFListen(void);

#define TIMER0_CC_S_TO_COUNT 245

// 112 byte == 896 bits, so a RF signal with maximum of 896 bits is possible
// for bucket transmission, this depends on the number of buckets.
// E.g. 4 buckets have a total overhead of 11, allowing 101 bits (high/low pairs)
#define RF_DATA_BUFFERSIZE		112

#define SWB(val) ((((val) >> 8) & 0x00FF) | (((val) << 8) & 0xFF00))

typedef enum
{
	RF_IDLE,
	RF_IN_SYNC,
	RF_DECODE_BUCKET,
	RF_TRANSMITTING,
	RF_FINISHED
} rf_state_t;

typedef enum
{
	// do sniffing by duty cycle
	MODE_DUTY_CYCLE,
	// do sniffing by bucket
	// https://github.com/pimatic/RFControl
	MODE_BUCKET
} rf_sniffing_mode_t;

extern SI_SEGMENT_VARIABLE(rf_data[RF_DATA_BUFFERSIZE], uint8_t, SI_SEG_XDATA);

// RF_DATA_STATUS
// Bit 7:	1 Data received, 0 nothing received
// Bit 6-0:	Protocol identifier
extern SI_SEGMENT_VARIABLE(rf_protocol, uint8_t, SI_SEG_XDATA);
extern SI_SEGMENT_VARIABLE(rf_state, rf_state_t, SI_SEG_XDATA);
extern SI_SEGMENT_VARIABLE(desired_rf_protocol, uint8_t, SI_SEG_XDATA);
extern SI_SEGMENT_VARIABLE(rf_listen_mode, rf_sniffing_mode_t, SI_SEG_XDATA);

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
extern SI_SEGMENT_VARIABLE(buckets[15], uint16_t, SI_SEG_XDATA);
extern SI_SEGMENT_VARIABLE(bucket_count, uint8_t, SI_SEG_XDATA);

#endif /* INC_RF_HANDLING_H_ */
