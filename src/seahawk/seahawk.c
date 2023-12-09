#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/int16_multi_array.h>
#include <rmw_microros/rmw_microros.h>

#include "pico/stdlib.h"
#include "pico_uart_transports.h"
#include "hardware/pio.h"
#include "hardware/pwm.h"


const uint LED_PIN = 25;

rcl_publisher_t publisher;
rcl_subscription_t subscriber;
std_msgs__msg__Int16MultiArray msg;
const uint GPIO[8] = {10, 11, 12, 13, 18, 19, 20, 21};
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


void subscription_callback(const void * msgin)
{
  // Process message
//   rcl_ret_t ret = rcl_publish(&publisher, (const std_msgs__msg__Int32 *)msgin, NULL);
    // Set the msgin to a Int16MultiArray
    msg = *((std_msgs__msg__Int16MultiArray *) msgin);
    set_duty_cycle((uint16_t *) msg.data.data);
}

int main()
{
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

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

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

    if (ret != RCL_RET_OK)
    {
        // Unreachable agent, exiting program.
        return ret;
    }
    config_pwm(3000);
    

    rclc_support_init(&support, 0, NULL, &allocator);

    rclc_node_init_default(&node, "pico_node", "", &support);
    
    rclc_subscription_init_default(
        &subscriber, 
        &node,
        type_support, 
        "pico_in");

    // rclc_publisher_init_default(
    //     &publisher,
    //     &node,
    //     type_support,
    //     "pico_out");

    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_subscription(
        &executor, 
        &subscriber, 
        &msg,
        &subscription_callback, 
        ON_NEW_DATA);

    gpio_put(LED_PIN, 1);

    while (true)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }
    return 0;
}
