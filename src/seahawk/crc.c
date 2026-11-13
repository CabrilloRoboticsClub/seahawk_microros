#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#include "crc.h"

uint8_t get_crc_byte(const uint8_t *buffer, uint16_t length) {
    uint8_t retVal = 0u;
    uint16_t byteIndex = 0u;

    if (buffer != NULL) {
        uint8_t bitIndex = 0u;
        retVal = CRC_8_INIT_VALUE;

        // Do calculation procedure for each byte
        for (byteIndex = 0u; byteIndex < length; byteIndex++) {
            // XOR new byte with temp result
            retVal ^= (buffer[byteIndex] << (CRC_8_RESULT_WIDTH - 8u));
            // Do calculation for current data
            for (bitIndex = 0u; bitIndex < 8u; bitIndex++) {
                if (retVal & (1u << (CRC_8_RESULT_WIDTH - 1u))) {
                    retVal = (retVal << 1u) ^ CRC_8_POLYNOMIAL;
                }
                else {
                    retVal = (retVal << 1u);
                }
            }
        }
        // XOR result with specified value
        retVal ^= CRC_8_XOR_VALUE;
    }
    return retVal;
}

void add_crc_byte(uint8_t *buffer, uint16_t length) {
	uint8_t crc = get_crc_byte(buffer, length);
	// Increase buffer size by 1
	buffer = (uint8_t *) realloc(buffer, length + 1);
	buffer[length] = crc;
}

bool check_crc(const uint8_t *buffer, uint16_t length_no_crc) {
	return get_crc_byte(buffer, length_no_crc) == buffer[length_no_crc];
}
