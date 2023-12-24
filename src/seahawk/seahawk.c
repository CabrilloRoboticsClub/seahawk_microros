// C libaries
#include <stdio.h>
#include <bool.h>
#include <time.h>
#include <stdint.h>

// Micro ROS libaries
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/int16_multi_array.h>
#include <rmw_microros/rmw_microros.h>

// Pico libaries
#include "pico/stdlib.h"
#include "pico_uart_transports.h"
#include "hardware/pio.h"
#include "hardware/pwm.h"

// PIO program
#include "send_dshot_packet.pio.h"

// Motor constants
const int NUM_MOTORS = 8;
const uint GPIO[8] = {10, 11, 12, 13, 18, 19, 20, 21}; // Pins on the pico we are using for motors

// Micro ROS globals
rcl_subscription_t subscriber;
std_msgs__msg__Int16MultiArray msg;

// 1.67e-6 is the time in micro seconds for one bit for DSHOT-600
// 75% of a full period sends a one
const float T1H = 1.67e-6 * 0.75;
// 37% of a full period sends a zero
const float T0H = 1.67e-6 * 0.37;

// See: https://brushlesswhoop.com/dshot-and-bidirectional-dshot/
enum dshot_cmds {
    // Commands 0 - 36 can only be executed when the motors are stopped
    DSHOT_CMD_MOTOR_STOP,
    DSHOT_CMD_BEEP1,                   // Wait at least length of beep (260ms) before next command
    DSHOT_CMD_BEEP2, 
    DSHOT_CMD_BEEP3,
    DSHOT_CMD_BEEP4,
    DSHOT_CMD_BEEP5,
    DSHOT_CMD_ESC_INFO,                // Wait at least 12ms before next command
    DSHOT_CMD_SPIN_DIRECTION_1,        // Send command 6 times
    DSHOT_CMD_SPIN_DIRECTION_2,        // Send command 6 times
    DSHOT_CMD_3D_MODE_OFF,             // Send command 6 times
    DSHOT_CMD_3D_MODE_ON,              // We want 3D mode on so motors can spin both ways, send 6x
    DSHOT_CMD_SETTINGS_REQUEST,
    DSHOT_CMD_SAVE_SETTINGS,           // Must save settings after changing any. Send 6x, wait 35ms
    DSHOT_EXTENDED_TELEMETRY_ENABLE,
    DSHOT_EXTENDED_TELEMETRY_DISABLE,
    // 15 - 19 not yet assigned
    DSHOT_CMD_SPIN_DIRECTION_NORMAL = 20, // Send command 6 times
    DSHOT_CMD_SPIN_DIRECTION_REVERSED,    // Send command 6 times
    DSHOT_CMD_LED0_ON,
    DSHOT_CMD_LED1_ON,
    DSHOT_CMD_LED2_ON,
    DSHOT_CMD_LED3_ON,
    DSHOT_CMD_LED0_OFF,
    DSHOT_CMD_LED1_OFF,
    DSHOT_CMD_LED2_OFF,
    DSHOT_CMD_LED3_OFF,
    // 30 - 31 not implemented currently
    DSHOT_CMD_SIGNAL_LINE_TELEMETRY_DISABLE = 32,           // Need 6x. Disables commands 42 to 47
    DSHOT_CMD_SIGNAL_LINE_TELEMETRY_ENABLE,                 // Need 6x. Enables commands 42 to 47
    DSHOT_CMD_SIGNAL_LINE_CONTINUOUS_ERPM_TELEMETRY,        // Need 6x. Enables commands 42 to 47 and sends erpm if normal Dshot frame
    DSHOT_CMD_SIGNAL_LINE_CONTINUOUS_ERPM_PERIOD_TELEMETRY, // Need 6x. Enables commands 42 to 47 and sends erpm period if normal Dshot frame
    // 36 - 41 not yet assigned 
    DSHOT_CMD_SIGNAL_LINE_TEMPERATURE_TELEMETRY = 42, 
    DSHOT_CMD_SIGNAL_LINE_VOLTAGE_TELEMETRY,
    DSHOT_CMD_SIGNAL_LINE_CURRENT_TELEMETRY, 
    DSHOT_CMD_SIGNAL_LINE_CONSUMPTION_TELEMETRY,
    DSHOT_CMD_SIGNAL_LINE_ERPM_TELEMETRY,
    DSHOT_CMD_SIGNAL_LINE_ERPM_PERIOD_TELEMETRY,
};

/**
 * Generates a frame given a throttle and telemetry value
 * 
 * Frames are organized in the following 16 bit pattern: SSSSSSSSSSSTCCCC
 *  (S) 11 bit throttle
 *  (T) 1 bit telemetry request
 *  (C) 4 bit Cyclic Redundancy Check (CRC) (calculated in this function)
 * 
 * @param throttle Throttle value (11 bits) 
 * @param telemetry Telemetry value (1 bit), true (1) if telemetry should be used, false (0) if not. Defaults to false
 * @return A 16 bit package of data to send following the parrern SSSSSSSSSSSTCCCC
*/
inline uint16_t create_packet(uint16_t throttle, bool telemetry=false) {
    // Shift everything in the backet over by one place then append telem
    uint16_t data = (throttle << 1) | telemetry;
    // CRC calculation
    uint8_t crc = (data ^ (data >> 4) ^ (data >> 8)) & 0x0F;

    return (data << 4) | crc;
}

/**
 * Initializes any dshot settings upon activation
 * 
 * Initialize pio, enable 3D DSHOT, and save settings
 * 
 * @param pio PIO instance
 * @param sm State machine
 * @param offset Location in memory where the assembled program is stored
*/
inline void init_dshot(PIO pio, uint sm, uint offset) {
    uint16_t dshot_3d_packet = create_packet(DSHOT_CMD_3D_MODE_ON);
    uint16_t save_settings_packet = create_packet(DSHOT_CMD_SAVE_SETTINGS);

    for (int i = 0; i < NUM_MOTORS; i++) {
        // Initialize pio program
        send_dshot_packet_program_init(pio, sm, offset, GPIO[i]);
        // Enable 3D dshot (packet must be sent 6 times)
        for (int j = 0; i < 6; j++) {
            send_packet(GPIO[i], dshot_3d_packet);
        }
        // Save settings after changing any (must be sent 6 times and wait 35 ms)
        for (int j = 0; i < 6; j++) {
            send_packet(GPIO[i], save_settings_packet);
        }
        sleep_ms(35);
    }
}

/**
 * Sends a high pulse to the specified pin
*/
inline void high(int pin) {
    // somehow use the T1H variable to wait that duration
}

/**
 * Sends a low pulse to the specified pin
*/
inline void low(int pin) {
    // nanosleep(T0H * 1000); // Is there a better wait function?
    // somehow use the T0H variable to wait that duration 
}

/**
 * Sends a packet of throttle data to a pin via bit banging
 * 
 * @param pin The pin to send values too
 * @param packet 16 bit DSHOT packet formatted in the SSSSSSSSSSSTCCCC pattern
*/
inline void send_packet(int pin, uint16_t packet) {
    for (int i = 15; i >= 0; i--) {
        // Isolate one bit then check if it is one, set to high
        if ((packet >> i) & 1) {
            high(pin);
        } 
        // Otherwise bit must be zero, set to low
        else {
            high(pin);
        }
    }
}

inline void subscription_callback(const void * msgin){
    // Set the msgin to a Int16MultiArray
    msg = *((std_msgs__msg__Int16MultiArray *) msgin);
    // msg should contain throttle values?? I think?
    set_throttle_values((uint16_t *) msg.data.data);
}

/**
 * Sends a ttrrottle value specified in `vals` for each GPIO pin
 * 
 * @param vals An array of length 8 containing throttle values in 3D format
*/
inline void set_throttle_values(uint16_t vals[]) {
    for (int i = 0; i < NUM_MOTORS; i++) {
        // Send a pin number, throttle value from vals array, and start with no telemetry
        send_packet(GPIO[i], vals[i]);
    }
}

int main() {
    const rosidl_message_type_support_t * type_support =
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32);

    rmw_uros_set_custom_transport(
		true,
		NULL,
		pico_serial_transport_open,
		pico_serial_transport_close,
		pico_serial_transport_write,
		pico_serial_transport_read
	);

    rcl_timer_t timer;
    rcl_node_t node;
    rcl_allocator_t allocator;
    rclc_support_t support;
    rclc_executor_t executor;

    allocator = rcl_get_default_allocator();

    // Wait for agent successful ping for 2 minutes.
    const int timeout_ms = 1000; 
    const uint8_t attempts = 120;

    rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);

    if (ret != RCL_RET_OK) {
        // Unreachable agent, exiting program.
        return ret;
    }
    
    rclc_support_init(&support, 0, NULL, &allocator);

    // PIO instance
    // PIO pio = pio0;

    // Load assembled program into PIO instruction memory
    // 'offset' is assigned the location of the program in memory
    uint offset = pio_add_program(pio, &send_dshot_packet_program);

    // Find unclaimed state mashine for PIO
    // True arg makes the function throw an error if none are avaliable
    uint sm = pio_claim_unused_sm(pio, true);

    // Initialize dshot
    init_dshot(pio, sm, offset);

    rclc_node_init_default(&node, "pico_node", "", &support);
    
    rclc_subscription_init_default(
        &subscriber, 
        &node,
        type_support, 
        "motor_msgs");

    // rclc_publisher_init_default(
    //     &publisher,
    //     &node,
    //     type_support,
    //     "pico_out");
    // Might be useful to have a publisher if we decide to do telemetry

    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_subscription(
        &executor, 
        &subscriber, 
        &msg,
        &subscription_callback, 
        ON_NEW_DATA);

    gpio_put(LED_PIN, 1);

    while (true) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }
    return 0;
}



// *************** Useful Sites ***************
// https://brushlesswhoop.com/dshot-and-bidirectional-dshot/
// https://www.swallenhardware.io/battlebots/2019/4/20/a-developers-guide-to-dshot-escs
// https://github.com/dmrlawson/raspberrypi-dshot/blob/master/dshotmodule.c
// https://github.com/mokhwasomssi/stm32_hal_dshot/blob/main/dshot.c
// https://github.com/cadouthat/pico-dshot/blob/main/src/dshot_encoder.cpp



// ***************** My Notes *****************
// Each frame has the following 16 bit structure 
// SSSSSSSSSSSTCCCC
// (S) 11 bit throttle
// (T) 1 bit telemetry request. Telemetry is allows the ESC to report RPM data back to the controller over the same single wire that we use to control the ESC.
// (C) 4 bit CRC. Cyclic Redundancy Check to validate data (throttle and telemetry request bit)

//  DSHOT frame are distinguished by their high time. This means that every bit has a certain (constant) length, and the length of the high part of the bit dictates if a 1 or 0 is being received.

// Every frame has duration 16 x (bit peroid time)

//             DSHOT 	Bitrate 	T1H 	T0H 	Bit (µs) 	Frame (µs)
// DSHOT300    300      300kbit/s 	2.50 	1.25 	3.33 	    53.28
// DSHOT600    600 	    600kbit/s 	1.25 	0.625 	1.67 	    26.72
// T1H duration for bit to be one. T0H duration for bit to be zero 

// Throtle values 0-47 are special commands. Cmmands, 42-47 are related to telemetry

// The check sum (CRC) can be calculated by 
// crc = (value ^ (value >> 4) ^ (value >> 8)) & 0x0F;