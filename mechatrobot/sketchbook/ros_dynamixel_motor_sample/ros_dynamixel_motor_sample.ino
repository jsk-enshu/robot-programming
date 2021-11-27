///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2021, Kei Okada and Kunio Kojima
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

#define BAUDRATE 115200

// Dynamixel
#include "src/DynamixelAX12A.h"

float angle;
int servo_id = 7;

// ROS
#include <ros.h>
ros::NodeHandle  nh;
#include <std_msgs/Float32.h>

// publisher
std_msgs::Float32 angle_msg;
ros::Publisher angle_position_sub("angle/position", &angle_msg);

// subscriber
void angle_cb(const std_msgs::Float32& msg)
{
    // ROSシリアルを通じて目標角度をsubscribeするとサーボに角度指令を送る
    goal_position(servo_id, msg.data);
}
ros::Subscriber<std_msgs::Float32> angle_command_sub("angle/command", angle_cb);
std_msgs::Float32 angle_command_msgs;

void setup()
{
    dynamixel_setup();

    Serial.begin(BAUDRATE); // serial port settings

    // setup ros functions
    nh.initNode();
    angle_msg.data = 0;
    nh.advertise(angle_position_sub);
    nh.subscribe(angle_command_sub);
}

void loop()
{
    // サーボの関節角度を読みだしてROSシリアルでpublishする
    angle = read_position(servo_id);
    angle_msg.data = angle;
    angle_position_sub.publish(&angle_msg);

    nh.spinOnce();
}
