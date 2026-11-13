#pragma once

#include <stdbool.h>

// Constants used in CRC generation

#define CRC_8_RESULT_WIDTH                  8u
#define CRC_8_POLYNOMIAL                    0x1Du
#define CRC_8_INIT_VALUE                    0xFFu
#define CRC_8_XOR_VALUE                     0xFFu

// Given a buffer, generate a CRC byte.
uint8_t get_crc_byte(const uint8_t *buffer, uint16_t length);

// Given a buffer, append a CRC byte.
void add_crc_byte(uint8_t *buffer, uint16_t length);

// Length should be of buffer without the crc
bool check_crc(const uint8_t *buffer, uint16_t length_no_crc);
