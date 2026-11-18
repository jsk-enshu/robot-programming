// micro-ROS port of mechatrobot/sketchbook/MechatrobotDriver for Arduino UNO R4 WiFi.
// Talks to micro_ros_agent over the USB serial port; WiFi is not used.
//
// Same topics as the rosserial version, so mechatrobot_ros2 works unchanged:
//   pub  range           sensor_msgs/Range  HC-SR04 (TRIG D2, ECHO D3), 20 Hz
//   pub  led/state       std_msgs/Bool      LED_BUILTIN blinks, 2 Hz
//   pub  motor1/position std_msgs/Int64     current stepper angle [deg], 20 Hz
//   sub  motor1/command  std_msgs/Int64     target stepper angle [deg] (absolute)
//
// Differences from the rosserial version:
//   - The stepper moves one step per loop() instead of blocking inside the
//     subscriber callback, so communication keeps running while it turns.
//   - motor1/position reports the angle actually reached, not the last target.
//   - pulseIn() times out after the echo time of max_range instead of 1 s.
#include <Arduino.h>
#include <Stepper.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <sensor_msgs/msg/range.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/int64.h>

// ---- hardware ---------------------------------------------------------------
// Same wiring as the summer course (micon-programming): HC-SR04 on D2/D3,
// ULN2003 IN1-IN4 on D5-D8.
constexpr int TRIG_PIN = 2;
constexpr int ECHO_PIN = 3;
constexpr int MOTOR_PIN1 = 5;
constexpr int MOTOR_PIN2 = 6;
constexpr int MOTOR_PIN3 = 7;
constexpr int MOTOR_PIN4 = 8;
// 360 [deg] / 5.625 [deg/step] / 2 (phase excitation) * 64 (gear ratio)
constexpr long STEPS_PER_ROTATE_28BYJ48 = 2048;
constexpr int STEPPER_RPM = 5;  // must be within 1-15

constexpr float SOUND_SPEED_M_PER_US = 0.000343f;
constexpr float RANGE_MIN_M = 0.02f;
constexpr float RANGE_MAX_M = 4.5f;
constexpr float RANGE_FOV_RAD = 0.261799f;  // 15 deg
// Round trip time for max_range plus margin; pulseIn() returns 0 after this.
constexpr unsigned long ECHO_TIMEOUT_US =
    (unsigned long)(2.0f * RANGE_MAX_M / SOUND_SPEED_M_PER_US) + 1000UL;

constexpr unsigned long SENSOR_PERIOD_MS = 50;
constexpr int LED_TOGGLE_EVERY_N_TICKS = 10;  // 10 * 50 ms = 500 ms

// The pin order 1-3-2-4 matches the wiring used in the course material.
static Stepper stepper(STEPS_PER_ROTATE_28BYJ48, MOTOR_PIN1, MOTOR_PIN3, MOTOR_PIN2, MOTOR_PIN4);
static long current_steps = 0;
static long target_steps = 0;

// ---- micro-ROS --------------------------------------------------------------
static char range_frame_id[] = "range_sensor";

static rcl_publisher_t range_pub;
static rcl_publisher_t led_pub;
static rcl_publisher_t position_pub;
static rcl_subscription_t command_sub;
static sensor_msgs__msg__Range range_msg;
static std_msgs__msg__Bool led_msg;
static std_msgs__msg__Int64 position_msg;
static std_msgs__msg__Int64 command_msg;

static rclc_executor_t executor;
static rclc_support_t support;
static rcl_allocator_t allocator;
static rcl_node_t node;
static rcl_timer_t timer;

enum class AgentState { WAITING, AVAILABLE, CONNECTED, DISCONNECTED };
static AgentState state = AgentState::WAITING;

#define RC(fn) { if ((fn) != RCL_RET_OK) { return false; } }

// Rounded so that commanding N deg reports N deg back once the motor arrives.
static long angle_to_steps(long deg) {
  return lround((double)deg * STEPS_PER_ROTATE_28BYJ48 / 360.0);
}

static long steps_to_angle(long steps) {
  return lround((double)steps * 360.0 / STEPS_PER_ROTATE_28BYJ48);
}

static float measure_range_m() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  unsigned long duration_us = pulseIn(ECHO_PIN, HIGH, ECHO_TIMEOUT_US);
  return duration_us * SOUND_SPEED_M_PER_US / 2.0f;
}

static void stamp_now(builtin_interfaces__msg__Time *stamp) {
  int64_t ns = rmw_uros_epoch_nanos();
  stamp->sec = (int32_t)(ns / 1000000000LL);
  stamp->nanosec = (uint32_t)(ns % 1000000000LL);
}

static void timer_callback(rcl_timer_t *t, int64_t) {
  static int tick = 0;
  if (t == NULL) {
    return;
  }
  range_msg.range = measure_range_m();
  stamp_now(&range_msg.header.stamp);
  (void)rcl_publish(&range_pub, &range_msg, NULL);

  position_msg.data = steps_to_angle(current_steps);
  (void)rcl_publish(&position_pub, &position_msg, NULL);

  if (tick % LED_TOGGLE_EVERY_N_TICKS == 0) {
    led_msg.data = !led_msg.data;
    digitalWrite(LED_BUILTIN, led_msg.data ? HIGH : LOW);
    (void)rcl_publish(&led_pub, &led_msg, NULL);
  }
  tick++;
}

static void command_callback(const void *msgin) {
  const std_msgs__msg__Int64 *m = (const std_msgs__msg__Int64 *)msgin;
  target_steps = angle_to_steps((long)m->data);
}

static bool create_entities() {
  allocator = rcl_get_default_allocator();
  RC(rclc_support_init(&support, 0, NULL, &allocator));
  RC(rclc_node_init_default(&node, "mechatrobot_driver", "", &support));
  RC(rclc_publisher_init_default(&range_pub, &node,
     ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range), "range"));
  RC(rclc_publisher_init_default(&led_pub, &node,
     ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "led/state"));
  RC(rclc_publisher_init_default(&position_pub, &node,
     ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int64), "motor1/position"));
  RC(rclc_subscription_init_default(&command_sub, &node,
     ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int64), "motor1/command"));
  RC(rclc_timer_init_default2(&timer, &support, RCL_MS_TO_NS(SENSOR_PERIOD_MS),
     timer_callback, true));
  executor = rclc_executor_get_zero_initialized_executor();
  RC(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RC(rclc_executor_add_timer(&executor, &timer));
  RC(rclc_executor_add_subscription(&executor, &command_sub, &command_msg,
     &command_callback, ON_NEW_DATA));
  // Header stamps use the agent's clock.
  (void)rmw_uros_sync_session(100);
  return true;
}

static void destroy_entities() {
  rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
  (void)rcl_publisher_fini(&range_pub, &node);
  (void)rcl_publisher_fini(&led_pub, &node);
  (void)rcl_publisher_fini(&position_pub, &node);
  (void)rcl_subscription_fini(&command_sub, &node);
  (void)rcl_timer_fini(&timer);
  (void)rclc_executor_fini(&executor);
  (void)rcl_node_fini(&node);
  (void)rclc_support_fini(&support);
}

static void step_towards_target() {
  if (current_steps < target_steps) {
    stepper.step(1);
    current_steps++;
  } else if (current_steps > target_steps) {
    stepper.step(-1);
    current_steps--;
  }
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  stepper.setSpeed(STEPPER_RPM);

  range_msg.radiation_type = sensor_msgs__msg__Range__ULTRASOUND;
  range_msg.field_of_view = RANGE_FOV_RAD;
  range_msg.min_range = RANGE_MIN_M;
  range_msg.max_range = RANGE_MAX_M;
  range_msg.header.frame_id.data = range_frame_id;
  range_msg.header.frame_id.size = strlen(range_frame_id);
  range_msg.header.frame_id.capacity = sizeof(range_frame_id);

  Serial.begin(115200);
  set_microros_serial_transports(Serial);
}

void loop() {
  static uint32_t last_check = 0;
  switch (state) {
    case AgentState::WAITING:
      if (millis() - last_check > 500) {
        last_check = millis();
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) {
          state = AgentState::AVAILABLE;
        }
      }
      break;
    case AgentState::AVAILABLE:
      if (create_entities()) {
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
      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
      break;
    case AgentState::DISCONNECTED:
      destroy_entities();
      state = AgentState::WAITING;
      break;
  }
  // Keep turning towards the last command even while the agent is away.
  step_towards_target();
}
