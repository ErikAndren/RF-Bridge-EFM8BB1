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
#include "uart.h"
#include "RF_Handling.h"
#include "RF_Protocols.h"
#include "RF_Buckets.h"
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

static bool is_uart_ack_missing(uart_command_t cmd) {
	if (waiting_for_uart_ack == true) {
		if (IsTimerFinished(TIMER3) == true) {
			if (uart_cmd_retry_cnt == RFIN_CMD_RETRIES) {
				// Give up
				waiting_for_uart_ack = false;

				// Reset listen state
				StartRFListen();
			} else {
				// Did not receive reply within the expected timeout, retry
				InitTimer_ms(TIMER3, 1, RFIN_CMD_TIMEOUT_MS);
				if (cmd == RF_CODE_IN) {
					uart_put_RF_CODE_Data(RF_CODE_IN);
				} else {
					uart_put_RF_Data(uart_command);
				}
				uart_cmd_retry_cnt++;
			}
		}
		return true;
	}
	return false;
}

static void is_learning_done(uart_command_t cmd) {
	if (rf_state == RF_FINISHED) {
		SoundBuzzer_ms(LEARN_CMD_SUCCESS_MS);

		desired_rf_protocol = last_desired_rf_protocol;
		uart_command = last_uart_command;
		if (cmd == RF_CODE_LEARN) {
			uart_put_RF_CODE_Data(RF_CODE_LEARN_SUCCESS);
		} else if (cmd == RF_PROTOCOL_LEARN) {
			uart_put_RF_Data(RF_PROTOCOL_LEARN_SUCCESS);
		}
		StartRFListen();

	// check for learning timeout
	// FIXME: Test this!
	} else if (IsTimerFinished(TIMER3) == true) {
		SoundBuzzer_ms(LEARN_CMD_FAILURE_MS);

		desired_rf_protocol = last_desired_rf_protocol;
		uart_command = last_uart_command;
		if (cmd == RF_CODE_LEARN) {
			uart_put_command(RF_CODE_LEARN_TIMEOUT);
		} else if (cmd == RF_PROTOCOL_LEARN) {
			uart_put_command(RF_PROTOCOL_LEARN_TIMEOUT);
		}
		StartRFListen();
	}
}

int main (void)
{
	uart_state_t uart_rx_state;
	uint16_t uart_rx_data;
	uint8_t uart_payload_len;
	uint8_t uart_payload_pos;
	uint8_t tr_repeats;

	// Call hardware initialization routine
	enter_DefaultMode_from_RESET();

	// enter default state
	LED = LED_OFF;
	BUZZER = BUZZER_OFF;
	T_DATA = 1;
	uart_rx_state = RECEIVE_IDLE;
	waiting_for_uart_ack = false;

	UART0_init(UART0_RX_ENABLE, UART0_WIDTH_8, UART0_MULTIPROC_DISABLE);

	last_desired_rf_protocol = PT2260_IDENTIFIER;
	desired_rf_protocol = PT2260_IDENTIFIER;
	last_uart_command = RF_CODE_IN;
	uart_command = RF_CODE_IN;

	// enable global interrupts
	IE_EA = 1;

	// Boot buzz
	SoundBuzzer_ms(BOOT_BUZZ_LENGTH_MS);

	StartRFListen();

	// Main loop
	while (true) {
		// Act upon currently executing command
		switch(uart_command) {
		case RF_CODE_LEARN:
			handle_rf_rx(uart_command);
			is_learning_done(uart_command);
			break; // case RF_CODE_LEARN

		case RF_CODE_IN:
			// check if a RF signal got decoded
			if (is_uart_ack_missing(uart_command) == false) {
				handle_rf_rx(uart_command);
			}
			break;

		case RF_CODE_OUT:
			handle_rf_tx(uart_command, &tr_repeats);
			break;

		case RF_PROTOCOL_SNIFFING_ON:
			if (is_uart_ack_missing(uart_command) == false) {
				handle_rf_rx(uart_command);
			}
			break;

		case RF_PROTOCOL_OUT:
			handle_rf_tx(uart_command, &tr_repeats);
			break;

		case RF_PROTOCOL_LEARN:
			handle_rf_rx(uart_command);
			is_learning_done(uart_command);
			break;

		case RF_BUCKET_OUT:
		{
			const uint8_t bkts = rf_data[BUCKET_NO_POS] * BUCKET_PAIRS;
			StopRFListen();

			// byte 2*(1..bkts):		bucket time high
			// byte 2*(1..bkts)+1:		bucket time low
			// byte 2*k+2..N:		RF buckets to send
			SendRFBuckets((uint16_t *)(rf_data + 2), rf_data + bkts + 2, uart_payload_len - bkts - 2, rf_data[BUCKET_REP_POS]);

			desired_rf_protocol = last_desired_rf_protocol;
			uart_command = last_uart_command;
			StartRFListen();
			uart_put_command(RF_CODE_ACK);
			break;
		}

		case RF_BUCKET_SNIFFING_ON:
			if (pos_pulse_len > 0) {
				Bucket_Received(pos_pulse_len);
			}

			if (neg_pulse_len > 0) {
				Bucket_Received(neg_pulse_len);
			}

			// check if a RF signal got decoded
			if (rf_state == RF_FINISHED)
			{
				uart_put_RF_buckets(RF_BUCKET_SNIFFING_ON);
				//FIXME: No ACK check heres
			}
			break;
		}

		/*------------------------------------------
		 * check if something got received by UART
		 ------------------------------------------*/
		uart_rx_data = uart_getc();

		if (uart_rx_state != RECEIVE_IDLE) {
			if (IsTimerFinished(TIMER2) == true) {
				// Three beeps for timeout
				SoundBuzzer_ms(50);
				delay_ms(200);
				SoundBuzzer_ms(50);
				delay_ms(200);
				SoundBuzzer_ms(50);

				uart_rx_state = RECEIVE_IDLE;
			}
		}

		if (uart_rx_data == UART_NO_DATA) {
			continue;
		} else if (uart_rx_data > UART_NO_DATA) {
			// Two beeps for uart rx error
			SoundBuzzer_ms(50);
			delay_ms(200);
			SoundBuzzer_ms(50);

			uart_rx_state = RECEIVE_IDLE;
			continue;
		}

		// state machine for UART rx
		switch(uart_rx_state)
		{
		// check if UART_SYNC_INIT got received
		case RECEIVE_IDLE:
			if (uart_rx_data == RF_CODE_START) {
				uart_rx_state = RECEIVE_COMMAND;

				InitTimer_ms(TIMER2, 100, CMD_TIMEOUT_MS);
			}
			break;

		// sync byte got received, read command
		case RECEIVE_COMMAND:
			next_uart_command = uart_rx_data;

			switch(next_uart_command)
			{
			case RF_CODE_OUT:
				uart_rx_state = RECEIVE_PAYLOAD;
				uart_payload_pos = 0;
				uart_payload_len = SONOFF_INSTR_SZ;
				break;

			case RF_PROTOCOL_OUT:
			case RF_BUCKET_OUT:
				uart_rx_state = RECEIVE_PAYLOAD_LEN;
				break;

			default:
				uart_rx_state = RECEIVE_END;
			}
			break; // End SYNC_INIT

		case RECEIVE_PAYLOAD_LEN:
			uart_payload_pos = 0;
			uart_payload_len = uart_rx_data;

			if (uart_payload_len > 0) {
				uart_rx_state = RECEIVE_PAYLOAD;
			} else {
				uart_rx_state = RECEIVE_END;
			}
			break;

		case RECEIVE_PAYLOAD:
			rf_data[uart_payload_pos] = uart_rx_data;
			uart_payload_pos++;

			if ((uart_payload_pos == uart_payload_len) || (uart_payload_pos >= RF_DATA_BUFFERSIZE)) {
				uart_rx_state = RECEIVE_END;
			}
			break;

		case RECEIVE_END:
			uart_rx_state = RECEIVE_IDLE;
			if (uart_rx_data == RF_CODE_STOP)
			{
				/* Act upon received command */
				switch(next_uart_command) {
				case RF_CODE_ACK:
					waiting_for_uart_ack = false;

					// Reset back to listen state
					StartRFListen();
					break;

				case RF_CODE_LEARN:
					SoundBuzzer_ms(LEARN_CMD_START_MS);
					desired_rf_protocol = PT2260_IDENTIFIER;
					last_uart_command = uart_command;
					uart_command = RF_CODE_LEARN;
					StartRFListen();
					uart_put_command(RF_CODE_ACK);
					InitTimer_ms(TIMER3, 1, LEARN_CMD_TIMEOUT_MS);
					break;

				case RF_CODE_OUT:
				case RF_PROTOCOL_OUT:
					tr_repeats = RF_TX_REPEATS;
					uart_command = next_uart_command;
					break;

				case RF_PROTOCOL_SNIFFING_ON:
					last_desired_rf_protocol = desired_rf_protocol;
					desired_rf_protocol = UNKNOWN_IDENTIFIER;
					last_uart_command = uart_command;
					uart_command = RF_PROTOCOL_SNIFFING_ON;
					StartRFListen();
					uart_put_command(RF_CODE_ACK);
					break;

				case RF_CODE_IN:
				// This command is unnecessary. Just set RF_CODE_IN (0xA4) state instead, keep for compatibility
				case RF_PROTOCOL_SNIFFING_OFF:
					desired_rf_protocol = PT2260_IDENTIFIER;
					last_uart_command = uart_command;
					uart_command = RF_CODE_IN;
					StartRFListen();
					uart_put_command(RF_CODE_ACK);
					break;

				case RF_BUCKET_SNIFFING_ON:
					last_uart_command = uart_command;
					uart_command = RF_BUCKET_SNIFFING_ON;
					StartRFListen();
					uart_put_command(RF_CODE_ACK);
					break;

				case RF_PROTOCOL_LEARN:
					SoundBuzzer_ms(LEARN_CMD_START_MS);
					last_desired_rf_protocol = desired_rf_protocol;
					desired_rf_protocol = UNKNOWN_IDENTIFIER;
					last_uart_command = uart_command;
					uart_command = RF_PROTOCOL_LEARN;
					uart_put_command(RF_CODE_ACK);
					StartRFListen();
					InitTimer_ms(TIMER3, 1, LEARN_CMD_TIMEOUT_MS);
					break;
				} // switch(uart_command)
			}
			break;
		} // switch uart_state
	}
}
