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

#define UART_COMMAND_TIMEOUT 30000
#define LEARN_CMD_START_MS 50
#define LEARN_CMD_FAILURE_MS 1000
#define LEARN_CMD_SUCCESS_MS 200
#define LEARN_CMD_TIMEOUT_MS 30000
#define BOOT_BUZZ_LENGTH_MS 50

SI_SEGMENT_VARIABLE(uart_command, uart_command_t, SI_SEG_XDATA) = NONE;

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

//-----------------------------------------------------------------------------
// main() Routine
// ----------------------------------------------------------------------------
int main (void)
{
	uart_state_t uart_state = IDLE;
	/* What is the purpose */
	bool ReadUARTData = true;
	uint8_t last_desired_rf_protocol;
	uint16_t l = 0;
	uart_command_t next_uart_command = NONE;

	// Call hardware initialization routine
	enter_DefaultMode_from_RESET();

	// enter default state
	LED = LED_OFF;
	BUZZER = BUZZER_OFF;
	T_DATA = 1;

	// enable UART
	UART0_init(UART0_RX_ENABLE, UART0_WIDTH_8, UART0_MULTIPROC_DISABLE);

#ifdef RF_LISTEN_ON_START
	desired_rf_protocol = PT2260_IDENTIFIER;
	rf_sniffing_mode = MODE_DUTY_CYCLE;
	PCA0_DoSniffing(RF_CODE_RFIN);
	last_sniffing_command = RF_CODE_RFIN;
#elif
	PCA0_StopSniffing();
#endif

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
		uint16_t rxdata = UART_NO_DATA;
		uint8_t len;
		uint8_t position;
		uint8_t protocol_index;

		if (ReadUARTData == true) {
			rxdata = uart_getc();
		}

		if (rxdata >= UART_NO_DATA)
		{
			if (uart_state == IDLE)
			{
				l = 0;
			} else {
				l++;

				// Timeout, revert back to idle state. Reset state if uart gets other errors
				if ((l > UART_COMMAND_TIMEOUT) || (rxdata > UART_NO_DATA))
				{
					// Three beeps for error
					SoundBuzzer_ms(50);
					delay_ms(50);
					SoundBuzzer_ms(50);
					delay_ms(50);
					SoundBuzzer_ms(50);
					delay_ms(50);

					uart_state = IDLE;
					uart_command = NONE;
					ReadUARTData = true;
				}
			}
		} else {
			l = 0;

			// state machine for UART
			switch(uart_state)
			{
			// check if UART_SYNC_INIT got received
			case IDLE:
				if (rxdata == RF_CODE_START) {
					uart_state = SYNC_INIT;
				}
				break;

				// sync byte got received, read command
			case SYNC_INIT:
				next_uart_command = rxdata;
				uart_state = SYNC_FINISH;

				switch(next_uart_command)
				{
				/* Do nothing here but wait for RF_CODE_STOP */
				case RF_CODE_LEARN:
				case RF_CODE_SNIFFING_ON:
				case RF_CODE_SNIFFING_OFF:
				case RF_CODE_SNIFFING_ON_BUCKET:
				case RF_CODE_LEARN_NEW:
				case RF_CODE_ACK:
					break;

				case RF_CODE_RFOUT:
					PCA0_StopSniffing();
					uart_state = RECEIVING;
					position = 0;
					len = 9;
					break;

				case RF_CODE_RFOUT_NEW:
				case RF_CODE_RFOUT_BUCKET:
					uart_state = RECEIVE_LEN;
					break;

				// Unknown command
				default:
					uart_command = NONE;
					uart_state = IDLE;
					ReadUARTData = true;
					break;
				} // End uart_command switch
				break; // End SYNC_INIT

				// Receiving UART data length
				case RECEIVE_LEN:
					position = 0;
					len = rxdata;
					if (len > 0) {
						uart_state = RECEIVING;
					} else {
						uart_state = SYNC_FINISH;
					}
					break;

					// Receiving UART data
				case RECEIVING:
					RF_DATA[position] = rxdata;
					position++;

					if (position == len || position >= RF_DATA_BUFFERSIZE) {
						uart_state = SYNC_FINISH;
					}
					break;

					// wait and check for UART_SYNC_END
				case SYNC_FINISH:
					if (rxdata == RF_CODE_STOP)
					{
						uart_state = IDLE;
						ReadUARTData = false;
						uart_command = next_uart_command;

						// check if ACK should be sent
						switch(uart_command)
						{
						case RF_CODE_LEARN:
						case RF_CODE_SNIFFING_ON:
						case RF_CODE_SNIFFING_OFF:
						case RF_CODE_RFIN:
						case RF_CODE_SNIFFING_ON_BUCKET:
							// send acknowledge
							uart_put_command(RF_CODE_ACK);
							ReadUARTData = true;
							break;

						case RF_CODE_ACK:
							ReadUARTData = true;
							last_sniffing_command = PCA0_DoSniffing(last_sniffing_command);
							break;
						}

						/* Act upon received command */
						switch(uart_command) {
						case RF_CODE_LEARN:
							SoundBuzzer_ms(LEARN_CMD_START_MS);

							// set desired RF protocol PT2260
							desired_rf_protocol = PT2260_IDENTIFIER;
							rf_sniffing_mode = MODE_DUTY_CYCLE;
							last_sniffing_command = PCA0_DoSniffing(RF_CODE_LEARN);

							// start timeout timer
							InitTimer_ms(1, LEARN_CMD_TIMEOUT_MS);
							break;

						case RF_CODE_SNIFFING_ON:
							desired_rf_protocol = UNKNOWN_IDENTIFIER;
							rf_sniffing_mode = MODE_DUTY_CYCLE;
							PCA0_DoSniffing(RF_CODE_SNIFFING_ON);
							last_sniffing_command = RF_CODE_SNIFFING_ON;
							break;

						case RF_CODE_SNIFFING_OFF:
							desired_rf_protocol = PT2260_IDENTIFIER;
							// re-enable default RF_CODE_RFIN sniffing
							rf_sniffing_mode = MODE_DUTY_CYCLE;
							PCA0_DoSniffing(RF_CODE_RFIN);
							last_sniffing_command = RF_CODE_RFIN;
							break;

						case RF_CODE_SNIFFING_ON_BUCKET:
							rf_sniffing_mode = MODE_BUCKET;
							last_sniffing_command = PCA0_DoSniffing(RF_CODE_SNIFFING_ON_BUCKET);
							break;

						case RF_CODE_LEARN_NEW:
							SoundBuzzer_ms(LEARN_CMD_START_MS);

							// enable sniffing for all known protocols
							last_desired_rf_protocol = desired_rf_protocol;
							desired_rf_protocol = UNKNOWN_IDENTIFIER;
							rf_sniffing_mode = MODE_DUTY_CYCLE;
							last_sniffing_command = PCA0_DoSniffing(RF_CODE_LEARN_NEW);

							// start timeout timer
							InitTimer_ms(1, LEARN_CMD_TIMEOUT_MS);
							break;

						default:
							break;
						} // switch(uart_command)
					} else {
						/* Received something else then RF_CODE_STOP */
						uart_state = IDLE;
						uart_command = NONE;
						ReadUARTData = true;
					}
					break;
			} // switch uart_state
		}

		/*------------------------------------------
		 * check command byte
		 ------------------------------------------*/
		switch(uart_command)
		{
		// do original learning
		case RF_CODE_LEARN:
			// check if a RF signal got decoded
			if ((RF_DATA_STATUS & RF_DATA_RECEIVED_MASK) != 0)
			{
				SoundBuzzer_ms(LEARN_CMD_SUCCESS_MS);

				PCA0_DoSniffing(last_sniffing_command);

				uart_put_RF_CODE_Data(RF_CODE_LEARN_OK);

				// clear RF status
				RF_DATA_STATUS = 0;

				// enable UART again
				ReadUARTData = true;
			} else if (IsTimerFinished()) {
				// check for learning timeout
				SoundBuzzer_ms(LEARN_CMD_FAILURE_MS);

				PCA0_DoSniffing(last_sniffing_command);

				// send not-acknowledge
				uart_put_command(RF_CODE_LEARN_KO);

				// enable UART again
				ReadUARTData = true;
			}
			break; // case RF_CODE_LEARN

		// do original sniffing
		case RF_CODE_RFIN:
			// check if a RF signal got decoded
			if ((RF_DATA_STATUS & RF_DATA_RECEIVED_MASK) != 0)
			{
				uart_put_RF_CODE_Data(RF_CODE_RFIN);

				// clear RF status
				RF_DATA_STATUS = 0;
			}
			break;

		// do original transfer
		case RF_CODE_RFOUT:
			// only do the job if all data got received by UART
			if (uart_state != IDLE) {
				break;
			}

			// do transmit of the data
			switch(rf_state)
			{
			// init and start RF transmit
			case RF_IDLE:
				// byte 0..1:	Tsyn
				// byte 2..3:	Tlow
				// byte 4..5:	Thigh
				// byte 6..7:	24bit Data
				// set high time of sync to (Tsyn / 3968) * 128
				// set duty cycle of high and low bit to 75 and 25 % - unknown
				PCA0_InitTransmit((uint16_t)((((uint32_t)(*(uint16_t *)&RF_DATA[0])) * 128) / 3968),
						*(uint16_t *)&RF_DATA[0], *(uint16_t *)&RF_DATA[4], 75, *(uint16_t *)&RF_DATA[2], 25, 24);

				actual_byte = 7;

				// start RF transmit
				PCA0_StartTransmit();
				break;

			// wait until data got transfered
			case RF_FINISHED:
				// restart sniffing if it was active
				PCA0_DoSniffing(last_sniffing_command);

				// send acknowledge
				uart_put_command(RF_CODE_ACK);

				// enable UART again
				ReadUARTData = true;
				break;
			} // rf_state
			break;

			// do new sniffing
			case RF_CODE_SNIFFING_ON:
				// check if a RF signal got decoded
				if ((RF_DATA_STATUS & RF_DATA_RECEIVED_MASK) != 0)
				{
					uint8_t used_protocol = RF_DATA_STATUS & 0x7F;
					uart_put_RF_Data(RF_CODE_SNIFFING_ON, used_protocol);

					// clear RF status
					RF_DATA_STATUS = 0;
				}
				break;

				// transmit data on RF
			case RF_CODE_RFOUT_NEW:
				// only do the job if all data got received by UART
				if (uart_state != IDLE) {
					break;
				}

				// do transmit of the data
				switch(rf_state)
				{
				// init and start RF transmit
				case RF_IDLE:
					PCA0_StopSniffing();

					// check if unknown protocol should be used
					// byte 0:		0x7F Protocol identifier
					// byte 1..2:	SYNC_HIGH
					// byte 3..4:	SYNC_LOW
					// byte 5..6:	BIT_HIGH_TIME
					// byte 7:		BIT_HIGH_DUTY
					// byte 8..9:	BIT_LOW_TIME
					// byte 10:		BIT_LOW_DUTY
					// byte 11:		BIT_COUNT + SYNC_BIT_COUNT in front of RF data
					// byte 12..N:	RF data to send
					if (RF_DATA[0] == 0x7F)
					{
						PCA0_InitTransmit(*(uint16_t *)&RF_DATA[1], *(uint16_t *)&RF_DATA[3],
								*(uint16_t *)&RF_DATA[5], RF_DATA[7], *(uint16_t *)&RF_DATA[8], RF_DATA[10], RF_DATA[11]);

						actual_byte = 12;
					}
					// byte 0:		Protocol identifier 0x01..0x7E
					// byte 1..N:	data to be transmitted
					else
					{
						protocol_index = PCA0_GetProtocolIndex(RF_DATA[0]);

						if (protocol_index != 0xFF)
						{
							PCA0_InitTransmit(PROTOCOL_DATA[protocol_index].SYNC_HIGH, PROTOCOL_DATA[protocol_index].SYNC_LOW,
									PROTOCOL_DATA[protocol_index].BIT_HIGH_TIME, PROTOCOL_DATA[protocol_index].BIT_HIGH_DUTY,
									PROTOCOL_DATA[protocol_index].BIT_LOW_TIME, PROTOCOL_DATA[protocol_index].BIT_LOW_DUTY,
									PROTOCOL_DATA[protocol_index].BIT_COUNT);

							actual_byte = 1;
						}
						else
						{
							uart_command = NONE;
						}
					}

					// if valid RF protocol start RF transmit
					if (uart_command != NONE) {
						PCA0_StartTransmit();
					}
					break;

					// wait until data got transfered
				case RF_FINISHED:
					// restart sniffing if it was active
					PCA0_DoSniffing(last_sniffing_command);

					// send acknowledge
					uart_put_command(RF_CODE_ACK);

					// enable UART again
					ReadUARTData = true;
					break;
				} // switch rf_state
				break;

				// new RF code learning
				case RF_CODE_LEARN_NEW:
					// check if a RF signal got decoded
					if ((RF_DATA_STATUS & RF_DATA_RECEIVED_MASK) != 0)
					{
						uint8_t used_protocol = RF_DATA_STATUS & 0x7F;
						SoundBuzzer_ms(LEARN_CMD_SUCCESS_MS);

						desired_rf_protocol = last_desired_rf_protocol;
						PCA0_DoSniffing(last_sniffing_command);

						uart_put_RF_Data(RF_CODE_LEARN_OK_NEW, used_protocol);

						// clear RF status
						RF_DATA_STATUS = 0;

						// enable UART again
						ReadUARTData = true;
					}
					// check for learning timeout
					else if (IsTimerFinished())
					{
						SoundBuzzer_ms(LEARN_CMD_FAILURE_MS);

						desired_rf_protocol = last_desired_rf_protocol;
						PCA0_DoSniffing(last_sniffing_command);
						// send not-acknowledge
						uart_put_command(RF_CODE_LEARN_KO_NEW);

						// enable UART again
						ReadUARTData = true;
					}
					break;

				case RF_CODE_RFOUT_BUCKET:
				{
					const uint8_t k = RF_DATA[0] * 2;

					// only do the job if all data got received by UART
					if (uart_state != IDLE) {
						break;
					}

					if (rf_state == RF_IDLE) {
						PCA0_StopSniffing();
					} else if (rf_state != RF_FINISHED) {
						break;
					}

					// byte 0:				number of buckets: k
					// byte 1:				number of repeats: r
					// byte 2*(1..k):		bucket time high
					// byte 2*(1..k)+1:		bucket time low
					// byte 2*k+2..N:		RF buckets to send
					if ((k == 0) || (len < 4))
					{
						uart_command = NONE;
						break;
					}
					else
					{
						SendRFBuckets((uint16_t *)(RF_DATA + 2), RF_DATA + k + 2, len - k - 2, RF_DATA[1]);
						// send acknowledgment
						uart_put_command(RF_CODE_ACK);
					}

					PCA0_DoSniffing(last_sniffing_command);
					// re-enable UART
					ReadUARTData = true;

					break;
				}

				case RF_CODE_SNIFFING_ON_BUCKET:
					// check if a RF signal got decoded
					if ((RF_DATA_STATUS & RF_DATA_RECEIVED_MASK) != 0)
					{
						uart_put_RF_buckets(RF_CODE_SNIFFING_ON_BUCKET);

						// clear RF status
						RF_DATA_STATUS = 0;
					}
					break;
		}
	}
}
