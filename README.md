# RF-Bridge-EFM8BB1
RF-Bridge-EFM8BB1

The Sonoff RF Bridge is only supporting one protocol with 24 bits.<br/>
This is an alternative firmware for the onboard EFM8BB1 chip forked from https://github.com/Portisch/RF-Bridge-EFM8BB1 but substantially rewritten to be more maintainable

All original commands 0xA0 to 0xA5 are supported and work as specified in:
https://www.itead.cc/wiki/File:RF_Universal_Transeceive_Module_Serial_Protocol_v1.0.pdf

# Hardware
There are the pins C2 & C2CK on the board. With a Arduino you can build a programmer to read/erase and program the flash.
Software for the Arduino: https://github.com/conorpp/efm8-arduino-programmer

# Software
The project is written with Simplicity Studio 4. The resulting *.hex file can be programmed on the EFM8BB1.

# Command 0xA5
This is the original implemented RF transmit command<br/>
Hex: AA A5 24 E0 01 40 03 84 D0 03 58 55<br/>

0xAA: uart sync init<br/>
0xA5: Transmit RF 
0x24-0xE0: Tsyn<br/>
0x01-0x40: Tlow<br/>
0x03-0x84: Thigh<br/>
0xD0-0x58: 24bit Data<br/>

The high time of the SYNC get calculated by the Tsyn (SYNC low time),<br/>
duty cycle of the high bit is 75% and 25% of the low bit.<br/>

# Command 0xA6, sniffs all known protocols
The reading of RF signals is already working:<br/>
Sending start sniffing: 0xAA 0xA6 0x55<br/>
Receiving AKN: 0xAA 0xA0 0x55<br/>

## Example, RF decode from Rohrmotor24.de remote (40 bit of data):
0xAA: uart sync init<br/>
0xA6: sniffing active<br/>
0x01: protocol identifier<br/>
0x06: data len<br/>
0xD0-0x55: data<br/>
0x55: uart sync end

# Command 0xA7 stops sniffing and reverts back to original Sonoff sniff
Not really necessary, 0xA4 works as well.

Sending stop sniffing: 0xAA 0xA7 0x55<br/>
Receiving AKN: 0xAA 0xA0 0x55<br/>

# Command 0xA8 - Transmits a RF command using a data length and protocol as parameter

Example (hex): AA A8 06 01 D0 F9 32 11 33 55<br/>

0xAA: uart sync init<br/>
0xA8: transmit RF data<br/>
0x01: protocol identifier<br/>
0x06: data len<br/>
0xD0-0x55: data<br/>
0x55: uart sync end

You may also specify a custom protocol 
Example (hex): AA A8 0D 7F 12 C0 05 DC 02 BC 46 01 2C 1E 08 AA 55<br/>

0xAA: uart sync init<br/>
0xA8: transmit RF data<br/>
0x0D: data len<br/>
0x7F: protocol identifier 0x7F<br/>
0x12-0xC0: SYNC_HIGH<br/>
0x05-0xDC: SYNC_LOW<br/>
0x02-0xBC: BIT_HIGH_TIME<br/>
0x46: BIT_HIGH_DUTY<br/>
0x01-0x2C: BIT_LOW_TIME<br/>
0x1E: BIT_LOW_DUTY<br/>
0x08: BIT_COUNT + SYNC_BIT_COUNT in front of RF data<br/>
0x1E...: RF data to send<br/>
0x55 uart sync end<br/>

## Command 0xA9 - Scan for all predefined protocols
Hex: AA A9 55<br/>

The first successfully decoded RF code will be sent by OK 0xAB.
If a timeout happens 0xAA will be sent.

## Bucket Transmitting using command 0xB0
This command accommodates RF protocols that can have variable bit times.
With this command, up to 16 time buckets can be defined, that denote the length of a high (mark) or low (space) transmission phase, e.g. for tri-state mode RF protocols.
This command also accommodates code repetition often used for higher reliability.

Hex: AA B0 20 04 1A 0120 01C0 0300 2710 01212122012201212121212121220121212201212203 55

0xAA: uart sync init<br/>
0xB0: transmit bucketed RF data<br/>
0x20: data len: 32 bytes<br/>
0x04: number of buckets: 4<br/>
0x19: number of repetitions: (transmit 1+25 = 26 times)<br/>
0x01-0x20: Bucket 1 length: 288µs<br/>
0x01-0xC0: Bucket 2 length: 448µs<br/>
0x03-0x00: Bucket 3 length: 768µs<br/>
0x27-0x10: Bucket 4 length: 10ms (sync)<br/>
0x05-0xDC: SYNC_LOW<br/>
0x02-0xBC: BIT_HIGH_TIME<br/>
0x01-0x03: RF data to send (high/low nibbles denote buckets to use for RF high (on) and low (off))<br/>
0x55: uart sync end

## RF decode from Rohrmotor24.de remote (40 bit of data):
0xAA: uart sync init<br/>
0xA6: sniffing active<br/>
0x06: data len<br/>
0x01: protocol identifier<br/>
0xD0-0x55: data<br/>
0x55: uart sync end

STOP:<br/>
Binary: 10101010 10100110 00000110 00000001 11010000 11111001 00110010 00010001 01010101 01010101<br/>
Hex: AA A6 06 01 D0 F9 32 11 55 55<br/>
DOWN:<br/>
Binary: 10101010 10100110 00000110 00000001 11010000 11111001 00110010 00010001 00110011 01010101<br/>
Hex: AA A6 06 01 D0 F9 32 11 33 55<br/>

## RF decode from Seamaid_PAR_56_RGB remote (24 bit of data):
Light ON:<br/>
Binary: 10101010 10100110 00000100 00000010 00110010 11111010 10001111 01010101<br/>
Hex: AA A6 04 02 32 FA 8F 55<br/>


Please note that currently, there is no learning mode for this!
However, you can use, e.g., an Arduino with the [RFControl](https://github.com/pimatic/RFControl)
library to learn the bucket times and sequences (the
[compressed](https://github.com/pimatic/RFControl/tree/master/examples/compressed) example
gives you everything you need if you convert the decimal numbers to hex).

# Next Steps
Add ESPurna support:<br/>
A new protocol have to be implemented to support more RF signals -> have to be defined!