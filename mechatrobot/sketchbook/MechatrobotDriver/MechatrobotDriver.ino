///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2020, Kei Okada and Yuki Asano
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// * Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
// * Neither the name of Kei Okada nor the names of its
// contributors may be used to endorse or promote products derived from
// this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////

#include <math.h>

// ROS
#include <ros.h>
ros::NodeHandle  nh;

// HC-SR04 Ultrasonic sensor
// https://create.arduino.cc/projecthub/Isaac100/getting-started-with-the-hc-sr04-ultrasonic-sensor-036380
#include <sensor_msgs/Range.h>

#define TRIG_PIN  9
#define ECHO_PIN 10
sensor_msgs::Range range_msg;

ros::Publisher range_pub("range", &range_msg);

/*
 * range_setup() : setup PIN and msg for ultrasonic, see HC-SR04 data sheet.
 */
void range_setup()
{
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg.field_of_view = 0.261799;// 15degree
  range_msg.min_range = 0.02; // [m]
  range_msg.max_range = 4.5;  // [m]
  range_msg.header.frame_id = "range";

  nh.advertise(range_pub);
}

/*
 * range_loop(): send triger and read echo pulse
 */
void range_loop()
{
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  float duration = pulseIn(ECHO_PIN, HIGH);
  range_msg.range = (duration*.000343)/2; // ultrasonic speed is 340m/s = 0.00034m/us
  range_msg.header.stamp = nh.now();

  // setup range publisher
  range_pub.publish(&range_msg);
}

// Stepping motor
// Use Stepper library https://github.com/arduino-libraries/Stepper
#include <Stepper.h>
#include <std_msgs/Int64.h>

#define MOTOR_PIN1 5
#define MOTOR_PIN2 6
#define MOTOR_PIN3 7
#define MOTOR_PIN4 8
#define STEPS_PER_ROTATE_28BYJ48 2048 // steps per rotate 360[deg] / 5.625[deg/step] / 2(phase execution system) * 64(gear ratio)

// note that we swapped C1 and C2 to cope with hardware devices
Stepper stepper(STEPS_PER_ROTATE_28BYJ48, MOTOR_PIN1, MOTOR_PIN3, MOTOR_PIN2, MOTOR_PIN4);
// rpm must be between 1-15
int RPM = 5;

void stepper_cb(const std_msgs::Int64& msg) ;
ros::Subscriber<std_msgs::Int64> stepper_sub("motor1/command", stepper_cb);
std_msgs::Int64 stepper_msg;
ros::Publisher stepper_pub("motor1/position", &stepper_msg);

int absolute_angle = 0;
/*
 * stepper_setup() : set default values
 */
void stepper_setup()
{
  stepper.setSpeed(RPM);

  // setup stepper subscriber and publisher
  nh.subscribe(stepper_sub);
  nh.advertise(stepper_pub);
}

/*
 * stepper_loop() : send target value
 */
void stepper_loop()
{
  // publish current target position
  stepper_msg.data = absolute_angle;
  stepper_pub.publish(&stepper_msg);
}

/*
 * subscribe motor1/command and control stepper
 */
void stepper_cb(const std_msgs::Int64& msg) {
  long relative_angle = msg.data - absolute_angle;
  absolute_angle = msg.data;

  long value = relative_angle * STEPS_PER_ROTATE_28BYJ48 / 360;
  stepper.step(value);
}

/*
 * define setup and loop functions.
 */
void setup()
{
  // setup ros functions
  nh.initNode();

  // setup LED functions
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(2, OUTPUT);

  // setup Ultrasonic sensor
  range_setup();

  // setup Stepper motor
  stepper_setup();
}

unsigned long led_timer = 0;
unsigned long range_timer = 0;
unsigned long state_timer = 0;
unsigned long led_count = 0;

void loop()
{
  unsigned long now = millis();

  // blink LED every 0.5 sec
  if ( (now - led_timer) > 500 ) {
    digitalWrite(LED_BUILTIN, (led_count%2==0)?HIGH:LOW);
    digitalWrite(2, (led_count%2==0)?HIGH:LOW);
    led_count++;
    led_timer = now;
  }

  // measure and publich range value every 50 milliseconds
  if ( (now - range_timer) > 50 ) {
    range_loop();
    range_timer = now;
  }

  // publish motor1 position every 50 milliseconds
  if ( (now - state_timer) > 50 ) {
    stepper_loop();
    state_timer = now;
  }

  // ROS
  nh.spinOnce();
}
