#include "dshot.h"
#include "hardware/pio.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "pico_uart_transports.h"
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/int16_multi_array.h>
#include <stdbool.h>
#include <stdio.h>

/* TODO:
 * Motor Armed Check, Commands, Telementry --> See dshot.h & examples*

 * This code will likey be used for Cabrillo Robotics ROV:
 *   It may be prudent to split motor control to more cores/controllers
 *   IE: struct dshot_controller controller0, controller1 */

/* Credit for DSHOT Implementation goes to https://github.com/simonwunderlich
 *  Thanks! */

rcl_subscription_t subscriber;
int16_t motor_throttles[4] = {0, 0, 0, 0};
const int NUM_MOTORS = 4;
const int DEBUG_LED = 15;
const int DSHOT_PROTOCOL = 600;

struct dshot_controller controller0;

void core1_main() {
  /* Core 1 Runs the DSHOT Loop. Precise timing necessitate by DSHOT,
  causes latency problem when MicroROS and DSHOT are computed on the same core.
  */
  while (true) {
    // gpio_put(DEBUG_LED, 1);
    for (int i = 0; i < NUM_MOTORS; i++) {
      dshot_throttle(&controller0, i, motor_throttles[i]);
    }

    dshot_loop(&controller0);
    // gpio_put(DEBUG_LED, 0);
  }
}

void subscription_callback(const void *msgin) {
  // Set the msgin to an Int16MultiArray
  // Expects array len 4, values 0-2047

  // gpio_put(DEBUG_LED, 1);

  const std_msgs__msg__Int16MultiArray *msg =
      (const std_msgs__msg__Int16MultiArray *)msgin;

  for (int i = 0; i < NUM_MOTORS; ++i) {
    motor_throttles[i] = msg->data.data[i];
  }

  // gpio_put(DEBUG_LED, 0);
}

int main(void) {
  stdio_init_all();

  /* initialize controller 0 with DSHOT_PROTOCOL on pio0, state machine 0
   * with 4 channel on pin 2-5 */
  dshot_controller_init(&controller0, DSHOT_PROTOCOL, pio0, 0, 2, NUM_MOTORS);

  gpio_init(DEBUG_LED);
  gpio_set_dir(DEBUG_LED, GPIO_OUT);
  gpio_put(DEBUG_LED, 1);

  const rosidl_message_type_support_t *type_support =
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray);

  rmw_uros_set_custom_transport(
      true, NULL, pico_serial_transport_open, pico_serial_transport_close,
      pico_serial_transport_write, pico_serial_transport_read);

  rcl_node_t node;
  rcl_allocator_t allocator;
  rclc_support_t support;
  rclc_executor_t executor;

  allocator = rcl_get_default_allocator();

  // Wait for agent successful ping for 2 minutes.
  const int timeout_ms = 1000;
  const uint8_t attempts = 120;

  rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);

  rclc_support_init(&support, 0, NULL, &allocator);

  rclc_node_init_default(&node, "motor_array", "", &support);

  // Define and initialize msg
  std_msgs__msg__Int16MultiArray msg;
  std_msgs__msg__MultiArrayLayout layout;
  std_msgs__msg__MultiArrayDimension layout_dim;

  // Initialize layout_dim
  layout_dim.label.data = "";
  layout_dim.label.size = 0;
  layout_dim.label.capacity = 0;
  layout_dim.size = 4;
  layout_dim.stride = 4;

  // Initialize layout
  std_msgs__msg__MultiArrayDimension__Sequence layout_dim_seq;
  layout_dim_seq.size = 1;
  layout_dim_seq.capacity = 1;
  layout_dim_seq.data = &layout_dim;

  layout.dim = layout_dim_seq;
  layout.data_offset = 0;

  // Initialize msg data
  msg.layout = layout;
  int16_t data[4] = {0, 0, 0, 0};
  msg.data.data = data;
  msg.data.size = 4;
  msg.data.capacity = 4;

  // Subscribe to topic from Pi, motor_array
  ret = rclc_subscription_init_default(&subscriber, &node, type_support,
                                       "/motor_array");
  if (ret != RCL_RET_OK) {
    return ret; // Unreachable agent, exiting program.
  }
  rclc_executor_init(&executor, &support.context, 4, &allocator);

  // Choose callback for subscription
  rclc_executor_add_subscription(&executor, &subscriber, &msg,
                                 &subscription_callback, ON_NEW_DATA);

  // Deals with PICO Pin control
  multicore_launch_core1(core1_main);

  // gpio_put(DEBUG_LED, 0);

  while (true) {
    // gpio_put(DEBUG_LED, 1);
    rclc_executor_spin(&executor);
  }
  return 0;
}
