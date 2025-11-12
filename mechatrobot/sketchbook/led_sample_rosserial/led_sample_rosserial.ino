#include <ros.h>
#include <std_msgs/Bool.h>

// ROS node handle
ros::NodeHandle nh;

// LED state message
std_msgs::Bool led_state_msg;

// Publisher for LED state
ros::Publisher led_state_pub("led/state", &led_state_msg);

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  // Initialize ROS
  nh.initNode();
  nh.advertise(led_state_pub);

  // Wait for connection
  while (!nh.connected()) {
    nh.spinOnce();
    delay(100);
  }
}

void loop() {
  // Turn LED ON
  digitalWrite(LED_BUILTIN, HIGH);
  led_state_msg.data = true;
  led_state_pub.publish(&led_state_msg);
  nh.spinOnce();
  delay(1000);

  // Turn LED OFF
  digitalWrite(LED_BUILTIN, LOW);
  led_state_msg.data = false;
  led_state_pub.publish(&led_state_msg);
  nh.spinOnce();
  delay(1000);
}
