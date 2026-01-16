#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"

// Data message size constants
#define DATA_SIZE 40
#define DATA_PAYLOAD_SIZE 37
#define DATA_SIZE_NO_CRC 39

// Request message size constants
#define REQUEST_SIZE 3
#define REQUEST_PAYLOAD_SIZE 0
#define REQUEST_SIZE_NO_CRC 2

#define HEADER_OFFSET 0
#define BYTE_COUNT_OFFSET 1
#define PAYLOAD_OFFSET 2

#define HEADER_SIZE 2
#define CRC_SIZE 1
#define HEADER_CRC_SIZE 3

typedef struct uart_inst{ 
    uart_inst_t* uart; 
    uint tx;
    uint rx;
    uint baudrate;

} uart_inst;

typedef enum msg_type{
    request,
    data 
} msg_type;

// Could separate ina780 and bme280 readings
typedef struct sensor_data {
	uint8_t kill;
	float ina780_current;
	float ina780_voltage;
	float ina780_temperature;
	float ina780_power;
	double ina780_energy;
	float bme280_temperature;
	float bme280_hum;
	float bme280_press;
	
} sensor_data;

bool uart_initialization(uart_inst inst);

bool get_response(uart_inst inst, sensor_data* retval);

// Request all data from BMS
bool send_request(uart_inst inst);

// Return all data to Pi
bool send_data(uart_inst inst);

// Read in data from BMS
sensor_data parse_data(uint8_t* data);
