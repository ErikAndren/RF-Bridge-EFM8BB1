/*
 * uart.h
 *
 *  Created on: 28.11.2017
 *      Author:
 */

#ifndef INC_UART_H_
#define INC_UART_H_

//-----------------------------------------------------------------------------
// Global Constants
//-----------------------------------------------------------------------------
#define RF_CODE_START		0xAA
#define RF_CODE_STOP		0x55

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
#define BUCKET_PAIRS 2
#define BUCKET_MAX 15

#define UART_RX_BUFFER_SIZE	16 + 4
#define UART_TX_BUFFER_SIZE	32

/*
** high byte error return code of uart_getc()
*/
#define UART_NO_DATA          0x0100              /* no receive data available   */
/* FIXME: The errors below are never set are they? */
#define UART_FRAME_ERROR      0x1000              /* Framing Error by UART       */
#define UART_OVERRUN_ERROR    0x0800              /* Overrun condition by UART   */
#define UART_PARITY_ERROR     0x0400              /* Parity Error by UART        */
#define UART_BUFFER_OVERFLOW  0x0200              /* receive ringbuffer overflow */

//-----------------------------------------------------------------------------
// Global Enums
//-----------------------------------------------------------------------------
typedef enum
{
	RECEIVE_IDLE,
	RECEIVE_COMMAND,
	RECEIVE_END,
	RECEIVE_PAYLOAD_LEN,
	RECEIVE_PAYLOAD,
	TRANSMIT,
	COMMAND
} uart_state_t;

typedef enum
{
	RF_CODE_ACK = 0xA0,
	RF_CODE_LEARN = 0xA1,
	RF_CODE_LEARN_TIMEOUT = 0xA2,
	RF_CODE_LEARN_SUCCESS = 0xA3,
	RF_CODE_IN = 0xA4,
	RF_CODE_OUT = 0xA5,
	RF_PROTOCOL_SNIFFING_ON = 0xA6,
	RF_PROTOCOL_SNIFFING_OFF = 0xA7,
	RF_PROTOCOL_OUT = 0xA8,
	RF_PROTOCOL_LEARN = 0xA9,
	// FIXME: Why not reuse the RF Code ones?
	RF_PROTOCOL_LEARN_TIMEOUT = 0xAA,
	RF_PROTOCOL_LEARN_SUCCESS = 0xAB,
	RF_BUCKET_OUT = 0xB0,
	RF_BUCKET_SNIFFING_ON = 0xB1
} uart_command_t;

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------
extern void uart_buffer_reset(void);
extern void uart_wait_until_TX_finished(void);
extern uint8_t uart_getlen(void);
extern bool uart_transfer_finished(void);
extern unsigned int uart_getc(void);
extern void uart_putc(uint8_t txdata);
extern void uart_puts(uint16_t txdata);
extern void uart_put_command(uint8_t command);
extern void uart_put_uint16_t(uint8_t command, uint16_t value);
extern void uart_put_RF_Data(uint8_t command);
extern void uart_put_RF_CODE_Data(uint8_t command);
extern void uart_put_RF_buckets(uint8_t command);

#endif /* INC_UART_H_ */
