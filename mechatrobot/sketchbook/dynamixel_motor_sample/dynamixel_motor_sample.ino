#include "src/DynamixelAX12A.h"

#define BAUDRATE 115200
#define LED_PIN 13

void setup()
{
    dynamixel_setup();

    Serial.begin(BAUDRATE);

    pinMode(LED_PIN, OUTPUT);
}

float angle;
int servo_id = 7;

void write_test(){
    goal_position(servo_id, 150);
    Serial.println("move to 150 deg");
    delay(1000);

    goal_position(servo_id, 120);
    Serial.println("move to 120 deg");
    delay(1000);
}

void read_test(){
    angle = read_position(servo_id);
    if (angle < 150){
        digitalWrite(LED_PIN, HIGH);
    } else {
        digitalWrite(LED_PIN, LOW);
    }
    Serial.println(angle, DEC);
    delay(1000);
}

void read_write_test(){
    goal_position(servo_id, 150);
    Serial.println("goal:150 deg");
    delay(1000);
    angle = read_position(servo_id);
    Serial.print("current:");
    Serial.println(angle);

    goal_position(servo_id, 120);;
    Serial.println("goal:120 deg");
    delay(1000);
    angle = read_position(servo_id);
    Serial.print("current:");
    Serial.println(angle);
}

void loop()
{
    /* write_test(); */
    read_test();
    /* read_write_test(); */
}
