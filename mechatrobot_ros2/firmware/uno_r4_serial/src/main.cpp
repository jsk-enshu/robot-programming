// Mechatrobot driver for Arduino UNO R4 WiFi: plain-text protocol over USB serial.
// mechatrobot_ros2/scripts/serial_bridge.py converts it to the same ROS 2 topics
// as the micro-ROS version (firmware/uno_r4_microros, env mechatrobot).
//
// Protocol: one line per message, '\n' terminated, readable in a serial monitor.
//   board -> PC   range <m>      HC-SR04 distance (TRIG D2, ECHO D3), 20 Hz
//                 pos <deg>      stepper angle actually reached, 20 Hz
//                 led <0|1>      LED_BUILTIN state, 2 Hz
//                 # <text>       free-form log; the bridge only prints it
//   PC -> board   cmd <deg>      stepper target angle (absolute)
//
// The control part (non-blocking stepper, pulseIn timeout, rounding) is the same
// as the micro-ROS version so that the two can be compared on communication only.
#include <Arduino.h>
#include <Stepper.h>
#include <stdlib.h>
#include <string.h>

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
constexpr float RANGE_MAX_M = 4.5f;
// Round trip time for max_range plus margin; pulseIn() returns 0 after this.
constexpr unsigned long ECHO_TIMEOUT_US =
    (unsigned long)(2.0f * RANGE_MAX_M / SOUND_SPEED_M_PER_US) + 1000UL;

constexpr unsigned long BAUD = 115200;
constexpr unsigned long SENSOR_PERIOD_MS = 50;
constexpr int LED_TOGGLE_EVERY_N_TICKS = 10;  // 10 * 50 ms = 500 ms
constexpr size_t LINE_MAX = 32;

// The pin order 1-3-2-4 matches the wiring used in the course material.
static Stepper stepper(STEPS_PER_ROTATE_28BYJ48, MOTOR_PIN1, MOTOR_PIN3, MOTOR_PIN2, MOTOR_PIN4);
static long current_steps = 0;
static long target_steps = 0;
static bool led_on = false;

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

// Handle one received line. Unknown or malformed lines are reported and ignored.
static void handle_line(const char *line) {
  if (strncmp(line, "cmd ", 4) == 0) {
    char *end = NULL;
    long deg = strtol(line + 4, &end, 10);
    if (end != line + 4) {
      target_steps = angle_to_steps(deg);
      return;
    }
  }
  Serial.print(F("# ignored: "));
  Serial.println(line);
}

// Collect characters into a line without blocking; call handle_line() on '\n'.
static void read_serial() {
  static char buf[LINE_MAX];
  static size_t len = 0;
  static bool overflow = false;
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r') {
      continue;
    }
    if (c != '\n') {
      if (len < LINE_MAX - 1) {
        buf[len++] = c;
      } else {
        overflow = true;
      }
      continue;
    }
    buf[len] = '\0';
    if (overflow) {
      Serial.println(F("# ignored: line too long"));
    } else if (len > 0) {
      handle_line(buf);
    }
    len = 0;
    overflow = false;
  }
}

static void send_sensors() {
  static int tick = 0;
  Serial.print(F("range "));
  Serial.println(measure_range_m(), 3);
  Serial.print(F("pos "));
  Serial.println(steps_to_angle(current_steps));
  if (tick % LED_TOGGLE_EVERY_N_TICKS == 0) {
    led_on = !led_on;
    digitalWrite(LED_BUILTIN, led_on ? HIGH : LOW);
    Serial.print(F("led "));
    Serial.println(led_on ? 1 : 0);
  }
  tick++;
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
  Serial.begin(BAUD);
  Serial.println(F("# mechatrobot_serial ready"));
}

void loop() {
  static unsigned long last_sensor_ms = 0;
  read_serial();
  unsigned long now = millis();
  if (now - last_sensor_ms >= SENSOR_PERIOD_MS) {
    last_sensor_ms = now;
    send_sensors();
  }
  step_towards_target();
}
