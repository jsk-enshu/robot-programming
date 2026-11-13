// Servo OFF for Gazebo: let named joints go limp while ros2_control keeps
// commanding them, the way the real robot's servos ignore commands with the
// torque off.
//
//   <plugin filename="jedy_servo_switch" name="jedy_sim::JedyServoSwitch">
//     <topic>/jedy_fullbody/servo/released</topic>                    <!-- in  -->
//     <state_topic>/jedy_fullbody/servo/released_state</state_topic>  <!-- out -->
//   </plugin>
//
// Input (gz.msgs.StringMsg): the comma-separated names of EVERY joint that is
// released now, so the message is idempotent and a repeat cannot toggle
// anything. "" = all driven. An unknown name is reported on the state topic
// as "!name" and nothing is released for it.
//
// Output: the released set as this plugin applies it, every 0.1 s of sim
// time and on every change, so the ROS side can tell "asked" from "done".
//
// What a released joint is. A Kondo servo with the torque off is back-driven
// through its gear train, whose dry friction holds a light load and lets a
// heavy one fall. That friction is the joint's, not the motor's: it is the
// URDF's <dynamics friction=... damping=...> (jedy_description/urdf/
// jedy_gz.xacro, README_dynamics.md), which sdformat turns into the SDF
// axis <friction>/<damping> and DART applies as joint Coulomb friction and
// viscous damping, servo on or off. So releasing a joint only removes the
// motor: DART's velocity motor (the one gz_ros2_control drives position
// joints with) keeps a zero-speed command but may push with at most 0 N*m.
// The motor stays a constraint rather than being switched to force mode,
// because force mode was not reliable here (below).
//
// Tried and dropped, for the record:
// * Stopping the controller: gz_ros2_control leaves its last JointVelocityCmd
//   in place (hold_joints true or false) and the joint stays where it was.
// * JointForceCmd = 0: gz-sim applies a force INSTEAD of the velocity command
//   when there is one, so the joint does go limp, but a joint left lying on
//   its hard limit in FORCE mode would not leave it when driven again (DART
//   JointConstraint::update skips the motor of a joint past its limit;
//   Gazebo: head_joint1 stuck at -1.570).
// * A motor limit of <friction> N*m set on this plugin, the same for every
//   joint, standing in for joint friction missing from the URDF. Replaced by
//   the URDF's per-joint friction; a <friction> element is now refused.
//
// Driven again, the joint gets its URDF effort limit back and
// gz_ros2_control's command applies from the next step.
//
// This system runs after gz_ros2_control in PreUpdate (priority below), so
// its zero-speed command replaces the controller's for released joints.

#include <algorithm>
#include <iostream>
#include <mutex>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include <gz/math/Vector2.hh>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/JointAxis.hh>
#include <gz/sim/components/JointEffortLimitsCmd.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/transport/Node.hh>

namespace jedy_sim
{

class JedyServoSwitch
  : public gz::sim::System,
    public gz::sim::ISystemConfigurePriority,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
{
public:
  /// After gz_ros2_control (default priority 0): lower values run first.
  gz::sim::System::PriorityType ConfigurePriority() override
  {
    return 100;
  }

  void Configure(
    const gz::sim::Entity & _entity,
    const std::shared_ptr<const sdf::Element> & _sdf,
    gz::sim::EntityComponentManager & /*_ecm*/,
    gz::sim::EventManager & /*_eventMgr*/) override
  {
    model_ = gz::sim::Model(_entity);
    std::string topic = "/jedy_fullbody/servo/released";
    std::string state_topic = "/jedy_fullbody/servo/released_state";
    if (_sdf->HasElement("topic")) {
      topic = _sdf->Get<std::string>("topic");
    }
    if (_sdf->HasElement("state_topic")) {
      state_topic = _sdf->Get<std::string>("state_topic");
    }
    if (_sdf->HasElement("friction")) {
      // Removed: a released joint is held by the URDF's own <dynamics
      // friction>. Accepting the old element would leave a model that
      // silently ignores a value its author meant to apply.
      std::cerr << "[Err] jedy_servo_switch: <friction> is no longer a parameter; "
            << "put the joint's friction in the URDF's <dynamics friction=...> "
            << "(jedy_description/README_dynamics.md). Not loading.\n";
      return;
    }
    node_.Subscribe(topic, &JedyServoSwitch::OnReleased, this);
    state_pub_ = node_.Advertise<gz::msgs::StringMsg>(state_topic);
  }

  void PreUpdate(
    const gz::sim::UpdateInfo & _info,
    gz::sim::EntityComponentManager & _ecm) override
  {
    if (_info.paused) {
      return;
    }
    std::set<std::string> wanted;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      wanted = wanted_;
    }
    std::vector<std::string> unknown;
    std::set<gz::sim::Entity> now;
    for (const auto & name : wanted) {
      const auto joint = model_.JointByName(_ecm, name);
      if (joint == gz::sim::kNullEntity) {
        unknown.push_back(name);
        continue;
      }
      now.insert(joint);
      SetVelocityCommand(joint, 0.0, _ecm);
      if (released_.count(joint) == 0) {
        SetEffortLimit(joint, 0.0, _ecm);
      }
    }
    // Driven again: the URDF's effort limit back. gz-sim applies a limits
    // command once and DART keeps it, so this is needed exactly once.
    for (const auto joint : released_) {
      if (now.count(joint) == 0) {
        const auto axis = _ecm.Component<gz::sim::components::JointAxis>(joint);
        if (axis != nullptr) {
          SetEffortLimit(joint, axis->Data().Effort(), _ecm);
        }
      }
    }
    released_ = now;

    if (_info.simTime - last_state_ >= std::chrono::milliseconds(100) ||
      wanted != reported_)
    {
      last_state_ = _info.simTime;
      reported_ = wanted;
      std::ostringstream out;
      bool first = true;
      for (const auto & name : wanted) {
        const bool bad =
          std::find(unknown.begin(), unknown.end(), name) != unknown.end();
        out << (first ? "" : ",") << (bad ? "!" : "") << name;
        first = false;
      }
      gz::msgs::StringMsg msg;
      msg.set_data(out.str());
      state_pub_.Publish(msg);
    }
  }

private:
  static void SetVelocityCommand(
    gz::sim::Entity _joint, double _value,
    gz::sim::EntityComponentManager & _ecm)
  {
    auto cmd = _ecm.Component<gz::sim::components::JointVelocityCmd>(_joint);
    if (cmd == nullptr) {
      _ecm.CreateComponent(
        _joint, gz::sim::components::JointVelocityCmd({_value}));
    } else if (!cmd->Data().empty()) {
      cmd->Data()[0] = _value;
    }
  }

  static void SetEffortLimit(
    gz::sim::Entity _joint, double _limit,
    gz::sim::EntityComponentManager & _ecm)
  {
    const std::vector<gz::math::Vector2d> limits{{-_limit, _limit}};
    auto cmd = _ecm.Component<gz::sim::components::JointEffortLimitsCmd>(_joint);
    if (cmd == nullptr) {
      _ecm.CreateComponent(
        _joint, gz::sim::components::JointEffortLimitsCmd(limits));
    } else {
      cmd->Data() = limits;
    }
  }

  void OnReleased(const gz::msgs::StringMsg & _msg)
  {
    std::set<std::string> names;
    std::stringstream in(_msg.data());
    std::string name;
    while (std::getline(in, name, ',')) {
      if (!name.empty()) {
        names.insert(name);
      }
    }
    std::lock_guard<std::mutex> lock(mutex_);
    wanted_ = names;
  }

  gz::sim::Model model_{gz::sim::kNullEntity};
  gz::transport::Node node_;
  gz::transport::Node::Publisher state_pub_;
  std::mutex mutex_;
  std::set<std::string> wanted_;
  std::set<std::string> reported_;
  std::set<gz::sim::Entity> released_;
  std::chrono::steady_clock::duration last_state_{};
};

}  // namespace jedy_sim

GZ_ADD_PLUGIN(
  jedy_sim::JedyServoSwitch,
  gz::sim::System,
  jedy_sim::JedyServoSwitch::ISystemConfigurePriority,
  jedy_sim::JedyServoSwitch::ISystemConfigure,
  jedy_sim::JedyServoSwitch::ISystemPreUpdate)
