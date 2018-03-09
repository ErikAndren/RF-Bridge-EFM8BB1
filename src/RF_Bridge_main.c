//=========================================================
// src/RF_Bridge_main.c: generated by Hardware Configurator
//
// This file will be updated when saving a document.
// leave the sections inside the "$[...]" comment tags alone
// or they will be overwritten!!
//=========================================================

//-----------------------------------------------------------------------------
// Includes
//-----------------------------------------------------------------------------
#include <SI_EFM8BB1_Register_Enums.h>                  // SFR declarations
#include "Globals.h"
#include "InitDevice.h"
#include "uart_0.h"
#include "pca_0.h"
#include "uart.h"
#include "RF_Handling.h"
#include "RF_Protocols.h"
#include "Buzzer.h"
#include "Timer.h"
#include "Delay.h"

// $[Generated Includes]
// [Generated Includes]$

#define UART_COMMAND_TIMEOUT_MS 30000
#define LEARN_CMD_START_MS 50
#define LEARN_CMD_FAILURE_MS 1000
#define LEARN_CMD_SUCCESS_MS 200
#define LEARN_CMD_TIMEOUT_MS 30000

#define RFIN_CMD_TIMEOUT_MS 1000
#define RFIN_CMD_RETRIES 3

#define CMD_TIMEOUT_MS 2000

#define BOOT_BUZZ_LENGTH_MS 50

//-----------------------------------------------------------------------------
// SiLabs_Startup() Routine
// ----------------------------------------------------------------------------
// This function is called immediately after reset, before the initialization
// code is run in SILABS_STARTUP.A51 (which runs before main() ). This is a
// useful place to disable the watchdog timer, which is enable by default
// and may trigger before main() in some instances.
//-----------------------------------------------------------------------------
void SiLabs_Startup (void)
{
}

int main (void)
{
	uart_state_t uart_rx_state;
	uint8_t last_desired_rf_protocol;
	uart_command_t next_uart_command;
	uart_command_t uart_command, last_uart_command;
	uint16_t uart_rx_data;
	uint8_t uart_payload_len;
	uint8_t uart_payload_pos;
	bool waiting_for_uart_ack;
	uint8_t uart_cmd_retry_cnt;

	// Call hardware initialization routine
	enter_DefaultMode_from_RESET();

	// enter default state
	LED = LED_OFF;
	BUZZER = BUZZER_OFF;
	T_DATA = 1;
	uart_rx_state = IDLE;
	waiting_for_uart_ack = false;

	UART0_init(UART0_RX_ENABLE, UART0_WIDTH_8, UART0_MULTIPROC_DISABLE);

	desired_rf_protocol = PT2260_IDENTIFIER;
	rf_listen_mode = MODE_DUTY_CYCLE;
	PCA0_StartRFListen();
	last_uart_command = RF_CODE_RFIN;
	uart_command = RF_CODE_RFIN;

	// enable global interrupts
	IE_EA = 1;

	// Boot buzz
	SoundBuzzer_ms(BOOT_BUZZ_LENGTH_MS);

	// Main loop
	while (true)
	{
		/*------------------------------------------
		 * check if something got received by UART
		 ------------------------------------------*/
		uart_rx_data = uart_getc();

		if (uart_rx_state != IDLE) {
			if (IsTimerFinished(TIMER2) == true) {
				// Three beeps for timeout
				SoundBuzzer_ms(50);
				delay_ms(200);
				SoundBuzzer_ms(50);
				delay_ms(200);
				SoundBuzzer_ms(50);

				uart_rx_state = IDLE;
			}
		}

		if (uart_rx_data > UART_NO_DATA) {
			// Two beeps for uart rx error
			SoundBuzzer_ms(50);
			delay_ms(200);
			SoundBuzzer_ms(50);

			uart_rx_state = IDLE;
		}

		// state machine for UART rx
		switch(uart_rx_state)
		{
		// check if UART_SYNC_INIT got received
		case IDLE:
			if (uart_rx_data == RF_CODE_START) {
				uart_rx_state = SYNC_INIT;
				InitTimer_ms(TIMER2, 1, CMD_TIMEOUT_MS);
			}
			break;

			// sync byte got received, read command
		case SYNC_INIT:
			next_uart_command = uart_rx_data;

			switch(next_uart_command)
			{
			case RF_CODE_RFOUT:
				uart_rx_state = RECEIVE_PAYLOAD;
				uart_payload_pos = 0;
				uart_payload_len = RF_INSTR_SZ;
				break;

			case RF_CODE_RFOUT_NEW:
			case RF_CODE_RFOUT_BUCKET:
				uart_rx_state = RECEIVE_PAYLOAD_LEN;
				break;

			default:
				uart_rx_state = SYNC_FINISH;
			} // End uart_command switch
			break; // End SYNC_INIT

		case RECEIVE_PAYLOAD_LEN:
			uart_payload_pos = 0;
			uart_payload_len = uart_rx_data;

			//FIXME: Add check for if payload lenght is too small
			if (uart_payload_len > 0) {
				uart_rx_state = RECEIVE_PAYLOAD;
			} else {
				uart_rx_state = SYNC_FINISH;
			}
			break;

		case RECEIVE_PAYLOAD:
			rf_data[uart_payload_pos] = uart_rx_data;
			uart_payload_pos++;

			if ((uart_payload_pos == uart_payload_len) || (uart_payload_pos >= RF_DATA_BUFFERSIZE)) {
				uart_rx_state = SYNC_FINISH;
			}
			break;

		case SYNC_FINISH:
			if (uart_rx_data == RF_CODE_STOP)
			{
				uart_rx_state = IDLE;

				if (next_uart_command == RF_CODE_ACK) {
					waiting_for_uart_ack = false;
					PCA0_StartRFListen();
				} else {
					uart_command = next_uart_command;
				}

				/* Act upon received command */
				switch(uart_command) {
				case RF_CODE_LEARN:
					uart_put_command(RF_CODE_ACK);

					SoundBuzzer_ms(LEARN_CMD_START_MS);

					desired_rf_protocol = PT2260_IDENTIFIER;
					rf_listen_mode = MODE_DUTY_CYCLE;
					PCA0_StartRFListen();
					last_uart_command = RF_CODE_LEARN;

					// start learn timeout timer
					InitTimer_ms(TIMER3, 1, LEARN_CMD_TIMEOUT_MS);
					break;

				case RF_CODE_SNIFFING_ON:
					uart_put_command(RF_CODE_ACK);

					desired_rf_protocol = UNKNOWN_IDENTIFIER;
					rf_listen_mode = MODE_DUTY_CYCLE;

					PCA0_StartRFListen();
					last_uart_command = RF_CODE_SNIFFING_ON;
					break;

				case RF_CODE_SNIFFING_OFF:
					uart_put_command(RF_CODE_ACK);

					desired_rf_protocol = PT2260_IDENTIFIER;
					rf_listen_mode = MODE_DUTY_CYCLE;
					PCA0_StartRFListen();
					last_uart_command = RF_CODE_RFIN;
					break;

				case RF_CODE_SNIFFING_ON_BUCKET:
					uart_put_command(RF_CODE_ACK);

					rf_listen_mode = MODE_BUCKET;
					PCA0_StartRFListen();
					last_uart_command = RF_CODE_SNIFFING_ON_BUCKET;
					break;

				case RF_CODE_LEARN_NEW:
					uart_put_command(RF_CODE_ACK);

					SoundBuzzer_ms(LEARN_CMD_START_MS);

					// enable sniffing for all known protocols
					last_desired_rf_protocol = desired_rf_protocol;
					desired_rf_protocol = UNKNOWN_IDENTIFIER;
					rf_listen_mode = MODE_DUTY_CYCLE;
					PCA0_StartRFListen();
					last_uart_command = RF_CODE_LEARN_NEW;

					// start timeout timer
					InitTimer_ms(TIMER3, 1, LEARN_CMD_TIMEOUT_MS);
					break;

				default:
					break;
				} // switch(uart_command)
			} else {
				// Received something else then RF_CODE_STOP
				uart_rx_state = IDLE;
			}
			break;
		} // switch uart_state

		/*------------------------------------------
		 * check command byte
		 ------------------------------------------*/
		switch(uart_command)
		{
		// do original learning
		case RF_CODE_LEARN:
			// check if a RF signal got decoded
			if (rf_state == RF_FINISHED)
			{
				SoundBuzzer_ms(LEARN_CMD_SUCCESS_MS);

				PCA0_StartRFListen();
				uart_command = last_uart_command;

				uart_put_RF_CODE_Data(RF_CODE_LEARN_ACK);

			// check for learning timeout
			} else if (IsTimerFinished(TIMER3) == true) {
				SoundBuzzer_ms(LEARN_CMD_FAILURE_MS);

				PCA0_StartRFListen();
				uart_command = last_uart_command;

				uart_put_command(RF_CODE_LEARN_NACK);
			}
			break; // case RF_CODE_LEARN

		// do original sniffing
		case RF_CODE_RFIN:
			// check if a RF signal got decoded
			if (waiting_for_uart_ack == true) {
				if (IsTimerFinished(TIMER3) == true) {
					// Did not receive reply within the expected timeout, retry again
					InitTimer_ms(TIMER3, 1, RFIN_CMD_TIMEOUT_MS);
					uart_put_RF_CODE_Data(RF_CODE_RFIN);
					uart_cmd_retry_cnt++;

					if (uart_cmd_retry_cnt == RFIN_CMD_RETRIES) {
						//Give up
						waiting_for_uart_ack = false;
						PCA0_StartRFListen();
					}
				}
			} else if (rf_state == RF_FINISHED) {
				// start wait for ack timeout timer
				InitTimer_ms(TIMER3, 1, RFIN_CMD_TIMEOUT_MS);
				waiting_for_uart_ack = true;
				uart_cmd_retry_cnt = 0;
				uart_put_RF_CODE_Data(RF_CODE_RFIN);
			}
			break;

		case RF_CODE_RFOUT:
			// do transmit of the data
			switch(rf_state)
			{
			// init and start RF transmit
			case RF_IDLE:
			{
				uint16_t sync_high = (uint16_t) ((((uint32_t) (*(uint16_t *) &rf_data[RF_TSYN_POS])) * PT2260_SYNC_HIGH) / PT2260_SYNC_PERIOD);
				uint16_t sync_low = (uint16_t) ((((uint32_t) (*(uint16_t *) &rf_data[RF_TSYN_POS])) * PT2260_SYNC_LOW) / PT2260_SYNC_PERIOD);
				uint16_t bit_high_t = *(uint16_t *) &rf_data[RF_THIGH_POS];
				uint16_t bit_low_t = *(uint16_t *) &rf_data[RF_TLOW_POS];

				PCA0_StopRFListen();
				PCA0_InitRFTransmit(sync_high, sync_low,
						bit_high_t, protocol_data[PT2260_INDEX].bit_high_duty,
						bit_low_t, protocol_data[PT2260_INDEX].bit_low_duty,
						protocol_data[PT2260_INDEX].bit_count);
				PCA0_StartRFTransmit(RF_DATA_POS);
				break;
			}

			case RF_TRANSMITTING:
				break;

			// Will be called once transmit is done
			case RF_FINISHED:
				PCA0_StartRFListen();
				uart_command = last_uart_command;

				// send acknowledge
				uart_put_command(RF_CODE_ACK);
				break;
			} // rf_state
			break;

		// do new sniffing
		case RF_CODE_SNIFFING_ON:
			if (waiting_for_uart_ack == true) {
				if (IsTimerFinished(TIMER3) == true) {
					// Did not receive reply within the expected timeout, retry again
					InitTimer_ms(TIMER3, 1, RFIN_CMD_TIMEOUT_MS);
					uart_put_RF_CODE_Data(RF_CODE_RFIN);
					uart_cmd_retry_cnt++;

					if (uart_cmd_retry_cnt == RFIN_CMD_RETRIES) {
						//Give up
						waiting_for_uart_ack = false;
						PCA0_StartRFListen();
					}
				}
			} else if (rf_state == RF_FINISHED) {
				uart_put_RF_Data(RF_CODE_SNIFFING_ON, rf_protocol);
				waiting_for_uart_ack = true;
			}
			break;

		// transmit data on RF
		case RF_CODE_RFOUT_NEW:
			// do transmit of the data
			switch(rf_state)
			{
			// init and start RF transmit
			case RF_IDLE:
				PCA0_StopRFListen();

				if (rf_data[RF_PROTOCOL_IDENT_POS] == CUSTOM_PROTOCOL_IDENT)
				{
					uint16_t sync_high = *(uint16_t *) &rf_data[CUSTOM_PROTOCOL_SYNC_HIGH_POS];
					uint16_t sync_low = *(uint16_t *) &rf_data[CUSTOM_PROTOCOL_SYNC_LOW_POS];
					uint16_t bit_high_t = *(uint16_t *) &rf_data[CUSTOM_PROTOCOL_BIT_HIGH_TIME_POS];
					uint8_t bit_high_duty = rf_data[CUSTOM_PROTOCOL_BIT_HIGH_DUTY_POS];
					uint16_t bit_low_t = *(uint16_t *) &rf_data[CUSTOM_PROTOCOL_BIT_LOW_TIME_POS];
					uint8_t bit_low_duty = rf_data[CUSTOM_PROTOCOL_BIT_LOW_DUTY_POS];
					uint8_t bit_count = rf_data[CUSTOM_PROTOCOL_BIT_COUNT_POS];

					PCA0_InitRFTransmit(
							sync_high,
							sync_low,
							bit_high_t,
							bit_high_duty,
							bit_low_t,
							bit_low_duty,
							bit_count);
					PCA0_StartRFTransmit(CUSTOM_PROTOCOL_DATA_POS);
				} else {
					uint8_t protocol_index = PCA0_GetProtocolIndex(rf_data[RF_PROTOCOL_IDENT_POS]);

					if (protocol_index != NO_PROTOCOL_FOUND)
					{
						PCA0_InitRFTransmit(
								protocol_data[protocol_index].sync_high, protocol_data[protocol_index].sync_low,
								protocol_data[protocol_index].bit_high_data, protocol_data[protocol_index].bit_high_duty,
								protocol_data[protocol_index].bit_low_time, protocol_data[protocol_index].bit_low_duty,
								protocol_data[protocol_index].bit_count);
						PCA0_StartRFTransmit(RF_PROTOCOL_START_POS);
					}
				}
				break;

			case RF_TRANSMITTING:
				break;

				// wait until data got transfered
			case RF_FINISHED:
				PCA0_StartRFListen();
				uart_command = last_uart_command;

				// send acknowledge
				uart_put_command(RF_CODE_ACK);
				break;
			} // switch rf_state
			break;

			// new RF code learning
			case RF_CODE_LEARN_NEW:
				// check if a RF signal got decoded
				if (rf_state == RF_FINISHED)
				{
					SoundBuzzer_ms(LEARN_CMD_SUCCESS_MS);

					desired_rf_protocol = last_desired_rf_protocol;
					PCA0_StartRFListen();
					uart_command = last_uart_command;

					uart_put_RF_Data(RF_CODE_LEARN_OK_NEW, rf_protocol);

				}
				// check for learning timeout
				else if (IsTimerFinished(TIMER3))
				{
					SoundBuzzer_ms(LEARN_CMD_FAILURE_MS);

					desired_rf_protocol = last_desired_rf_protocol;
					PCA0_StartRFListen();
					uart_command = last_uart_command;

					// send not-acknowledge
					uart_put_command(RF_CODE_LEARN_KO_NEW);
				}
				break;

			case RF_CODE_RFOUT_BUCKET:
			{
				// FIXME: Why * 2?
				const uint8_t bkts = rf_data[BUCKET_NO_POS] * 2;
				PCA0_StopRFListen();

				// byte 2*(1..bkts):		bucket time high
				// byte 2*(1..bkts)+1:		bucket time low
				// byte 2*k+2..N:		RF buckets to send
				SendRFBuckets((uint16_t *)(rf_data + 2), rf_data + bkts + 2, uart_payload_len - bkts - 2, rf_data[BUCKET_REP_POS]);

				uart_put_command(RF_CODE_ACK);

				PCA0_StartRFListen();
				uart_command = last_uart_command;
				break;
			}

			case RF_CODE_SNIFFING_ON_BUCKET:
				// check if a RF signal got decoded
				if (rf_state == RF_FINISHED)
				{
					uart_put_RF_buckets(RF_CODE_SNIFFING_ON_BUCKET);
				}
				break;
		}
	}
}
