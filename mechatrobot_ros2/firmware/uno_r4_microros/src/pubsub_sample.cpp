// micro-ROS over USB serial on Arduino UNO R4 WiFi (no WiFi).
//
// pub: /uno_r4/counter (std_msgs/Int32, 10 Hz)
// sub: /uno_r4/led     (std_msgs/Int32) nonzero -> LED on. The value is also copied
//                      into the counter so the PC side can confirm reception.
//
// The agent may be started or stopped at any time: entities are destroyed and
// recreated without resetting the board.
#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/int32.h>

static rcl_publisher_t publisher;
static rcl_subscription_t subscriber;
static std_msgs__msg__Int32 pub_msg;
static std_msgs__msg__Int32 sub_msg;
static rclc_executor_t executor;
static rclc_support_t support;
static rcl_allocator_t allocator;
static rcl_node_t node;
static rcl_timer_t timer;

enum class AgentState { WAITING, AVAILABLE, CONNECTED, DISCONNECTED };
static AgentState state = AgentState::WAITING;

#define RC(fn) { if ((fn) != RCL_RET_OK) { return false; } }

static void timer_callback(rcl_timer_t *t, int64_t) {
  if (t != NULL) {
    (void)rcl_publish(&publisher, &pub_msg, NULL);
    pub_msg.data++;
  }
}

static void sub_callback(const void *msgin) {
  const std_msgs__msg__Int32 *m = (const std_msgs__msg__Int32 *)msgin;
  digitalWrite(LED_BUILTIN, m->data != 0 ? HIGH : LOW);
  pub_msg.data = m->data;
}

static bool create_entities() {
  allocator = rcl_get_default_allocator();
  RC(rclc_support_init(&support, 0, NULL, &allocator));
  RC(rclc_node_init_default(&node, "uno_r4_node", "", &support));
  RC(rclc_publisher_init_default(&publisher, &node,
     ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "uno_r4/counter"));
  RC(rclc_subscription_init_default(&subscriber, &node,
     ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "uno_r4/led"));
  RC(rclc_timer_init_default2(&timer, &support, RCL_MS_TO_NS(100), timer_callback, true));
  executor = rclc_executor_get_zero_initialized_executor();
  RC(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RC(rclc_executor_add_timer(&executor, &timer));
  RC(rclc_executor_add_subscription(&executor, &subscriber, &sub_msg, &sub_callback, ON_NEW_DATA));
  return true;
}

static void destroy_entities() {
  rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
  (void)rcl_publisher_fini(&publisher, &node);
  (void)rcl_subscription_fini(&subscriber, &node);
  (void)rcl_timer_fini(&timer);
  (void)rclc_executor_fini(&executor);
  (void)rcl_node_fini(&node);
  (void)rclc_support_fini(&support);
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
}

void loop() {
  static uint32_t last_check = 0;
  switch (state) {
    case AgentState::WAITING:
      if (millis() - last_check > 500) {
        last_check = millis();
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));  // slow blink: waiting for agent
        if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) {
          state = AgentState::AVAILABLE;
        }
      }
      break;
    case AgentState::AVAILABLE:
      if (create_entities()) {
        digitalWrite(LED_BUILTIN, LOW);
        state = AgentState::CONNECTED;
      } else {
        destroy_entities();
        state = AgentState::WAITING;
      }
      break;
    case AgentState::CONNECTED:
      if (millis() - last_check > 1000) {
        last_check = millis();
        if (rmw_uros_ping_agent(100, 3) != RMW_RET_OK) {
          state = AgentState::DISCONNECTED;
          break;
        }
      }
      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
      break;
    case AgentState::DISCONNECTED:
      destroy_entities();
      state = AgentState::WAITING;
      break;
  }
}
