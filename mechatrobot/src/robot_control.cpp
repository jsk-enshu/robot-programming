/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Modified 2020, by Kei OKada and Yuki Asano
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
#include <signal.h>
#include <pthread.h>
#include <atomic>
#include <condition_variable>  // NOLINT(build/c++11)
#include <thread>              // NOLINT(build/c++11)
#include <chrono>              // NOLINT(build/c++11)
#include <vector>
#include <string>
using namespace std::chrono_literals;  // NOLINT(build/namespaces)

// ROS
#include <ros/ros.h>

// ros_control
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <controller_manager/controller_manager.h>

// messages
#include <std_msgs/Int64.h>
#include <std_msgs/Float64.h>

namespace robot_control
{
struct JointData
{
  std::string name_;
  double cmd_;
  double pos_;
  double vel_;
  double eff_;
  int home_encoder_offset_;
};

class JointControlInterface
{
public:
  JointControlInterface(ros::NodeHandle& nh_, int joint_id, std::string command_topic, std::string position_topic,
                        hardware_interface::JointStateInterface& jnt_stat,
                        hardware_interface::PositionJointInterface& jnt_cmd)
  {
    // create joint name
    std::stringstream ss;
    ss << "joint" << joint_id;
    joint.name_ = ss.str();

    // register joint
    // registerJoint(joint.name_, jnt_stat, jnt_cmd);
    hardware_interface::JointStateHandle state_handle(joint.name_, &joint.pos_, &joint.vel_, &joint.eff_);
    jnt_stat.registerHandle(state_handle);
    // position handle
    hardware_interface::JointHandle pos_handle(jnt_stat.getHandle(joint.name_), &joint.cmd_);
    jnt_cmd.registerHandle(pos_handle);

    joint_pub = nh_.advertise<std_msgs::Int64>(command_topic, 1);
    joint_sub = nh_.subscribe(position_topic, 1, &JointControlInterface::positionCB, this);

    // initialize joint/command
    joint.cmd_ = joint.pos_ = 0;
    joint.vel_ = joint.eff_ = 0;

    // initialize sock and signal
    last_received = ros::Time::now();
    last_read = ros::Time::now();
  }

  ~JointControlInterface(){};  // NOLINT(readability/braces)

  void read()
  {
    if (last_read >= last_received)
    {  // no received message after last read
      std::unique_lock<std::mutex> lk(cv_m);
      if (cv.wait_for(lk, 100ms, [] { return 0; }))  // NOLINT(whitespace/braces)
      {
        ROS_ERROR_STREAM("Did not receive message " << joint.name_ << " for 100ms");
      }
    }
    ROS_DEBUG_STREAM(joint.name_ << " read() : " << joint.pos_);
    joint.pos_ = joint.cmd_;  // do loop back
    last_read = ros::Time::now();
  }
  void positionCB(const std_msgs::Int64::ConstPtr& msg)
  {
    joint.pos_ = msg->data * M_PI / 180.0;
    cv.notify_all();
    last_received = ros::Time::now();
  }
  void write()
  {
    ROS_DEBUG_STREAM(joint.name_ << " write() : " << joint.cmd_);
    std_msgs::Int64 msg;
    msg.data = static_cast<int>(joint.cmd_ * 180 / M_PI);
    joint_pub.publish(msg);
  }
  void shutdown()
  {
  }

protected:
  JointData joint;
  ros::Publisher joint_pub;
  ros::Subscriber joint_sub;
  ros::Time last_received;
  ros::Time last_read;
  // lock
  std::condition_variable cv;
  std::mutex cv_m;
};

class RobotHardwareInterface : public hardware_interface::RobotHW
{
private:
  // Node Handles
  ros::NodeHandle nh_;  // no namespace

  hardware_interface::JointStateInterface joint_state_interface;
  hardware_interface::PositionJointInterface joint_position_interface;

  // Kinematic properties
  typedef std::vector<JointControlInterface*> JointControlContainer;
  JointControlContainer controls;

public:
  /**
   * \brief Constructor/Descructor
   */
  RobotHardwareInterface()
  {
    registerControl(new JointControlInterface(nh_, 1, "/motor1/command", "/motor1/position", joint_state_interface,
                                              joint_position_interface));

    // register joint state/position interface
    registerInterface(&joint_state_interface);
    registerInterface(&joint_position_interface);
  };
  ~RobotHardwareInterface()
  {
    shutdown();
  }
  void registerControl(JointControlInterface* control)
  {
    controls.push_back(control);
  }

  bool read()
  {
    BOOST_FOREACH (JointControlInterface* control, controls)  // NOLINT(whitespace/parens)
    {
      control->read();
    }
  }

  void write()
  {
    BOOST_FOREACH (JointControlInterface* control, controls)  // NOLINT(whitespace/parens)
    {
      control->write();
    }
  }

  void shutdown()
  {
    BOOST_FOREACH (JointControlInterface* control, controls)  // NOLINT(whitespace/parens)
    {
      control->shutdown();
    }
    controls.clear();
  }
  ros::Time getTime()
  {
    return ros::Time::now();
  }
  ros::Duration getPeriod()
  {
    return ros::Duration(0.001);
  }
};  // class RobotHardwareInterface
};  // namespace robot_control

static int g_quit = 0;

static const int SEC_2_NSEC = 1e+9;
static const int SEC_2_USEC = 1e6;

static inline double now()
{
  struct timespec n;
  clock_gettime(CLOCK_MONOTONIC, &n);
  return static_cast<double>(n.tv_nsec) / SEC_2_NSEC + n.tv_sec;
}

void controlLoop()
{
  ros::NodeHandle nh;
  // Initialize the hardware interface
  robot_control::RobotHardwareInterface robot;

  // Create controller manager
  controller_manager::ControllerManager cm(&robot);

  ros::Duration durp(100);  // 100 msec;

  ROS_INFO("started controlLoop");
  while (!g_quit)
  {
    struct timespec tick;
    clock_gettime(CLOCK_REALTIME, &tick);
    ros::Time this_moment(tick.tv_sec, tick.tv_nsec);
    robot.read();
    cm.update(this_moment, durp);
    robot.write();
  }

  robot.shutdown();
  ros::shutdown();
}

void quitRequested(int sig)
{
  g_quit = 1;
  std::cerr << ";; call quitRequested sig(" << sig << ")" << std::endl;
}

int main(int argc, char* argv[])
{
  // Initialize ROS and parse command-line arguments
  ros::init(argc, argv, "realtime_loop");
  ros::NodeHandle node;

  // Catch attempts to quit
  signal(SIGTERM, quitRequested);
  signal(SIGINT, quitRequested);
  signal(SIGHUP, quitRequested);

  // Start thread
  std::thread rv(controlLoop);
  ros::spin();
  rv.join();

  return 0;
}
