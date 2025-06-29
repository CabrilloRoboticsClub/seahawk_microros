#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/int16_multi_array.h>
#include <std_msgs/msg/float64_multi_array.h>
#include <rmw_microros/rmw_microros.h>

#include "pico/stdlib.h"
#include "pico_uart_transports.h"
#include "hardware/pio.h"
#include "hardware/pwm.h"

#define BMS 0
#include "uart_proto.h"
#include "crc.h"

const uart_inst inst = {uart0, 0, 1, 115200};

const uint LED_PIN = 25;

rcl_publisher_t publisher;
rcl_subscription_t subscriber;
const uint GPIO[8] = {20, 13, 19, 11, 21, 12, 18, 10};
const int NUM_MOTORS = 8;
const uint8_t CLOCK_DIV = 125;

// configures the pwm generators
void config_pwm(uint16_t wrap) {
    for (int i = 0; i < NUM_MOTORS; i++) {
      // Sets all GPIO pins to use PWM
      gpio_set_function(GPIO[i], GPIO_FUNC_PWM);
      // Sets the wrap for each slice
      pwm_set_wrap(pwm_gpio_to_slice_num(GPIO[i]), wrap);
      /*
       * If phase correct is set to false the counter
       * will reset to 0 after reaching the level,
       * otherwise it will decrease down to 0
       */
      pwm_set_phase_correct(pwm_gpio_to_slice_num(GPIO[i]), false);
      // Set clock division
      pwm_set_clkdiv_int_frac(pwm_gpio_to_slice_num(GPIO[i]), CLOCK_DIV, 0);
      // Set pins to 1500microseconds for neutral
      pwm_set_gpio_level(GPIO[i], 1500);
      // Enable PWM
      pwm_set_enabled(pwm_gpio_to_slice_num(GPIO[i]), true);
    }
}
// Sets all of the motors to the given levels
void set_duty_cycle(uint16_t levels[]) {    
    for (int i = 0; i < NUM_MOTORS; i++) {
        pwm_set_gpio_level(GPIO[i], levels[i]);
    }
}

int mothertrucker = 0;

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
    sensor_data retval;
    send_request(inst);
    get_response(inst, &retval);
    std_msgs__msg__Float64MultiArray msg;
    double float_data[9] = {
        (double) retval.kill,
        (double) retval.ina780_current,
        (double) retval.ina780_voltage,
        (double) retval.ina780_temperature,
        (double) retval.ina780_power,
        (double) retval.ina780_energy,
        (double) retval.bme280_temperature,
        (double) retval.bme280_hum,
        (double) retval.bme280_press,
    };
    msg.data.data = float_data;
    mothertrucker = (mothertrucker + 1) % 2;
    gpio_put(LED_PIN, mothertrucker);
}

void subscription_callback(const void * msgin)
{
    // Set the msgin to a Int16MultiArray
    std_msgs__msg__Int16MultiArray *msg = (std_msgs__msg__Int16MultiArray *) msgin;
    set_duty_cycle((uint16_t *) msg->data.data);
}

int main()
{
    // const rosidl_message_type_support_t * type_support =
    //     ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray);

    rmw_uros_set_custom_transport(
		true,
		NULL,
		pico_serial_transport_open,
		pico_serial_transport_close,
		pico_serial_transport_write,
		pico_serial_transport_read
	);

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    rcl_node_t node;
    rcl_allocator_t allocator;
    rclc_support_t support;
    rclc_executor_t executor;

    allocator = rcl_get_default_allocator();

    // Wait for agent successful ping for 2 minutes.
    const int timeout_ms = 1000; 
    const uint8_t attempts = 120;

    rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);

    if (ret != RCL_RET_OK)
    {
        // Unreachable agent, exiting program.
        return ret;
    }

    config_pwm(3000);

    rclc_support_init(&support, 0, NULL, &allocator);
    
    rclc_node_init_default(&node, "pico_node", "", &support);

    // Define pwm_msg
    std_msgs__msg__Int16MultiArray pwm_msg;

    // Define msg data Int16 Sequence
    rosidl_runtime_c__int16__Sequence pwm_msg_data;
    pwm_msg_data.size = 0;
    pwm_msg_data.capacity = 8;
    int16_t pwm_msg_data_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    pwm_msg_data.data = pwm_msg_data_data;

    pwm_msg.data = pwm_msg_data;

    // Define msg layout MultiArray Layout

    std_msgs__msg__MultiArrayLayout pwm_msg_layout;

    std_msgs__msg__MultiArrayDimension__Sequence pwm_msg_layout_dim;
    pwm_msg_layout.dim = pwm_msg_layout_dim;
    pwm_msg_layout.data_offset = 0;
    
    pwm_msg.layout = pwm_msg_layout;

    // Define bms_msg
    std_msgs__msg__Float64MultiArray bms_msg;

    // Define bms_msg data Float64 Sequence
    rosidl_runtime_c__double__Sequence bms_msg_data;
    bms_msg_data.size = 0;
    bms_msg_data.capacity = 9;
    double bms_msg_data_data[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    bms_msg_data.data = bms_msg_data_data;

    bms_msg.data = bms_msg_data;

    // Define bms_msg_layout

    std_msgs__msg__MultiArrayLayout bms_msg_layout;

    std_msgs__msg__MultiArrayDimension__Sequence bms_msg_layout_dim;
    bms_msg_layout.dim = bms_msg_layout_dim;
    bms_msg_layout.data_offset = 0;
    
    bms_msg.layout = bms_msg_layout;

    uart_initialization(inst);

    const unsigned int timer_period = RCL_MS_TO_NS(1000);
    rcl_timer_t timer;
    rcl_ret_t rc = rclc_timer_init_default(&timer, &support, timer_period, timer_callback);


    ret = rclc_subscription_init_default(
        &subscriber, 
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray), 
        "pwm_values");

    ret = rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
        "bms_data");

    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_subscription(
        &executor, 
        &subscriber, 
        &pwm_msg,
        &subscription_callback, 
        ON_NEW_DATA);
    
    rc = rclc_executor_add_timer(&executor, &timer);

    gpio_put(LED_PIN, 1);


    while (true)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }
    return 0;
}
