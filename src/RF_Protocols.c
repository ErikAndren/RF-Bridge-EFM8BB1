/*
 * RF_Protocols.c
 *
 *  Created on: 18 apr. 2018
 *      Author: erik
 */

#include <stdbool.h>
#include <stdint.h>

#define MANCHESTER_0 0x1
#define MANCHESTER_1 0x2
#define DATA_0 0x0
#define DATA_1 0x1

bool dec_manchester(uint8_t *src, uint8_t *dst, uint8_t src_len_bits) {
	uint8_t i, bit_pattern;

	for (i = 0; i < src_len_bits; i += 2) {
		if (i % 8 == 0) {
			dst[i / 8] = 0;
		}

		bit_pattern = ((src[i / 8] >> (i % 8)) & 0x3);
		if (bit_pattern == MANCHESTER_0) {
			dst[i] |= (0 << (i % 8));
		} else if (bit_pattern == MANCHESTER_1) {
			dst[i] |= (1 << (i % 8));
		} else {
			/* Not a proper manchester encoding */
			return false;
		}
	}

	return true;
}

void enc_manchester(uint8_t *src, uint8_t *dst, uint8_t src_len_bits) {
	uint8_t i, bit_pattern;

	for (i = 0; i < src_len_bits; i++) {
		if (i % 8 == 0) {
			dst[i] = 0;
		}

		bit_pattern = (src[i / 8] >> (i % 8)) & 0x1;
		if (bit_pattern == DATA_0) {
			dst[i / (8 / 2)] |= (MANCHESTER_0 << (i % (8 / 2)));
		} else {
			dst[i / (8 / 2)] |= (MANCHESTER_1 << (i % (8 / 2)));
		}
	}
}
