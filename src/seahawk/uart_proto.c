#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "uart_proto.h"
#include "crc.h"

uint8_t uart_initialization(uart_inst inst) {
    gpio_set_function(inst.tx, UART_FUNCSEL_NUM(inst.uart, inst.tx));
    gpio_set_function(inst.rx, UART_FUNCSEL_NUM(inst.uart, inst.rx));
    uart_init(inst.uart, inst.baudrate);
    return 1;
}

uint8_t get_response(uart_inst inst) {
	uint8_t* resp = (uint8_t *) malloc(HEADER_SIZE);
	uart_write_blocking(inst.uart, resp, HEADER_SIZE);

	msg_type type = resp[0];
	uint8_t byte_count = resp[1];

	resp = (uint8_t *) realloc(resp, HEADER_CRC_SIZE + byte_count);
	uart_write_blocking(inst.uart, PAYLOAD_OFFSET, CRC_SIZE + byte_count);

	switch(type) {
		case data:
			if (byte_count != DATA_PAYLOAD_SIZE || !check_crc(resp, DATA_SIZE_NO_CRC)) {
				return 0;
			}
			sensor_data retval;
			retval = parse_data(resp);
			break;
		case request:
#if BMS == 1
			if (byte_count != REQUEST_PAYLOAD_SIZE || !check_crc(resp, REQUEST_SIZE_NO_CRC)) {
				return 0;
			}
			send_data(inst);
#else
			return 0;
#endif
		default:
			return 0;
	}

}

uint8_t send_request(msg_type type, uart_inst inst) {
	uint8_t* msg = (uint8_t *) malloc(REQUEST_SIZE_NO_CRC);
	msg[0] = request;
	msg[1] = REQUEST_PAYLOAD_SIZE;

	add_crc_byte(msg, REQUEST_SIZE_NO_CRC);
	uart_write_blocking(inst.uart, msg, REQUEST_SIZE);

	return 1;
}

#if BMS == 1
uint8_t send_data(uart_inst inst) {
	// Make readings
	float bme280_temperature;
	float bme280_hum;
	float bme280_press;

	bme280_read_all(*bme280_temperature, *bme280_hum, *bme280_press);

	sensor_data data = {
		gpio_kill_switch_triggered(),
		ina780_read_current(),
		ina780_read_bus_voltage(),
		ina780_read_temperature(),
		ina780_read_power(),
		ina780_read_energy(),
		bme280_temperature,
		bme280_hum,
		bme280_press,
	}

	// Initialize msg
	msg_type type = data;
	uint8_t bytes = DATA_PAYLOAD_SIZE;
	uint8_t* msg = (uint8_t *) malloc(DATA_SIZE_NO_CRC); // Header + byte count + payload

	// Fill msg with data
	msg[HEADER_OFFSET] = data;
	msg[BYTE_COUNT_OFFSET] = bytes;

	memcpy(msg + PAYLOAD_OFFSET, &data, bytes);

	// Add crc and transmit
	add_crc_byte(msg, DATA_SIZE_NO_CRC);
	uart_write_blocking(inst.uart, msg, DATA_SIZE); // Header + byte count + payload + crc

	return 1;
}
#endif

sensor_data parse_data(uint8_t* resp) {
	sensor_data retval;
	memcpy(&retval, resp + PAYLOAD_OFFSET, DATA_PAYLOAD_SIZE);
	return retval;
}

int main() {
    return 0;
}
