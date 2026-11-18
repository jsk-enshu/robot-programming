// Mechatrobot driver for Arduino UNO R4 WiFi: plain-text protocol over USB serial.
// mechatrobot_ros2/scripts/serial_bridge.py converts it to the same ROS 2 topics
// as the micro-ROS version (firmware/uno_r4_microros, env mechatrobot).
//
// Protocol: one line per message, '\n' terminated, readable in a serial monitor.
//   board -> PC   range <m>      HC-SR04 distance (TRIG D2, ECHO D3), 20 Hz
//                 pos <deg>      angle the servo is being driven to now, 20 Hz
//                 led <0|1>      LED_BUILTIN state, 2 Hz
//                 # <text>       free-form log; the bridge only prints it
//   PC -> board   cmd <deg>      servo target angle, -90 (one end) .. 90 (other end),
//                                0 is the centre; values outside are clamped
//
// The servo (SG90 on D8) has no position output, so "pos" is the angle this
// sketch writes to it, not a measurement. The sketch moves that angle towards
// the target at SERVO_DEG_PER_SEC, slower than the SG90 can turn, so the servo
// keeps up and "pos" stays close to where the horn actually is.
//
// The control part (non-blocking servo motion, pulseIn timeout, clamping) is the
// same as the micro-ROS version so that the two can be compared on communication
// only.
#include <Arduino.h>
#include <Servo.h>
#include <stdlib.h>
#include <string.h>

// ---- hardware ---------------------------------------------------------------
// HC-SR04 on D2/D3 as in the summer course (micon-programming, task 3),
// SG90 servo signal on D8.
constexpr int TRIG_PIN = 2;
constexpr int ECHO_PIN = 3;
constexpr int SERVO_PIN = 8;
// Servo.write() takes 0-180 deg; the protocol uses -90..90 with 0 at the centre.
constexpr long SERVO_CENTER_DEG = 90;
constexpr long ANGLE_MIN_DEG = -90;
constexpr long ANGLE_MAX_DEG = 90;
// The SG90 turns about 600 deg/s (0.1 s / 60 deg) with no load.
constexpr unsigned long SERVO_DEG_PER_SEC = 200;
constexpr unsigned long SERVO_MS_PER_DEG = 1000 / SERVO_DEG_PER_SEC;

constexpr float SOUND_SPEED_M_PER_US = 0.000343f;
constexpr float RANGE_MAX_M = 4.5f;
// Round trip time for max_range plus margin; pulseIn() returns 0 after this.
constexpr unsigned long ECHO_TIMEOUT_US =
    (unsigned long)(2.0f * RANGE_MAX_M / SOUND_SPEED_M_PER_US) + 1000UL;

constexpr unsigned long BAUD = 115200;
constexpr unsigned long SENSOR_PERIOD_MS = 50;
constexpr int LED_TOGGLE_EVERY_N_TICKS = 10;  // 10 * 50 ms = 500 ms
constexpr size_t LINE_MAX = 32;

static Servo servo;
static long current_deg = 0;  // angle written to the servo [deg], -90..90
static long target_deg = 0;
static bool led_on = false;

static long clamp_angle(long deg) {
  if (deg < ANGLE_MIN_DEG) {
    return ANGLE_MIN_DEG;
  }
  if (deg > ANGLE_MAX_DEG) {
    return ANGLE_MAX_DEG;
  }
  return deg;
}

static void write_servo(long deg) {
  servo.write((int)(SERVO_CENTER_DEG + deg));
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
      target_deg = clamp_angle(deg);
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
  Serial.println(current_deg);
  if (tick % LED_TOGGLE_EVERY_N_TICKS == 0) {
    led_on = !led_on;
    digitalWrite(LED_BUILTIN, led_on ? HIGH : LOW);
    Serial.print(F("led "));
    Serial.println(led_on ? 1 : 0);
  }
  tick++;
}

// Move the servo 1 deg towards the target every SERVO_MS_PER_DEG. When loop()
// was held up (pulseIn waits up to ~27 ms), catch up by several degrees.
static void move_towards_target() {
  static unsigned long last_ms = 0;
  unsigned long now = millis();
  if (current_deg == target_deg) {
    last_ms = now;
    return;
  }
  long steps = (long)((now - last_ms) / SERVO_MS_PER_DEG);
  if (steps == 0) {
    return;
  }
  last_ms += (unsigned long)steps * SERVO_MS_PER_DEG;
  long remaining = target_deg - current_deg;
  if (labs(remaining) <= steps) {
    current_deg = target_deg;
  } else {
    current_deg += remaining > 0 ? steps : -steps;
  }
  write_servo(current_deg);
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  // The servo goes to the centre at power-on; that is where "pos 0" starts.
  write_servo(current_deg);
  servo.attach(SERVO_PIN);
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
  move_towards_target();
}
