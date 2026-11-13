// Mimic joints for Gazebo with DART: make every joint whose axis carries a
// <mimic> (from the URDF's <mimic joint=... multiplier=... offset=...>) follow
// its leader.
//
//   <plugin filename="jedy_mimic_joint" name="jedy_sim::JedyMimicJoint">
//     <gain>50</gain>   <!-- 1/s, optional -->
//   </plugin>
//
// Why. Jedy's second gripper finger (rarm/larm_mimic_gripper_joint) has no
// servo: a linkage turns it opposite to the driven finger. Gazebo Harmonic
// turns the URDF <mimic> into an SDF mimic constraint, but its DART engine
// does not implement those ("the chosen physics engine does not support mimic
// constraints, so no constraint will be created"), and gz_ros2_control's
// mimic support goes through the same constraint. So the finger stayed at 0
// while the other one moved: a 4.5 cm cube between them was pushed away
// instead of grasped.
//
// How. Each step, before physics, the follower gets a velocity command
//
//     v = gain * (multiplier * leader + offset - follower)
//
// clamped to the follower's URDF velocity limit, the same kind of command
// gz_ros2_control drives position joints with. DART turns it into a motor
// limited to the follower's URDF effort, so it pushes no harder than that and
// stops on an object. (The real linkage passes the one servo's torque to both
// fingers; here each finger has its own limit, so the grip is up to twice the
// real one. Stated, not hidden.)
//
// Mimic data are read from the model, nothing is named here; a model with no
// mimic joints is left alone. A follower that jedy_servo_switch releases keeps
// following: that plugin releases only the joints named to it, which the
// mimic fingers are not (they have no servo of their own).

#include <algorithm>
#include <iostream>
#include <string>
#include <vector>

#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointAxis.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>

namespace jedy_sim
{

class JedyMimicJoint
  : public gz::sim::System,
    public gz::sim::ISystemConfigurePriority,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
{
public:
  /// After gz_ros2_control (default priority 0), like jedy_servo_switch.
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
    if (_sdf->HasElement("gain")) {
      gain_ = _sdf->Get<double>("gain");
    }
  }

  void PreUpdate(
    const gz::sim::UpdateInfo & _info,
    gz::sim::EntityComponentManager & _ecm) override
  {
    if (!found_) {
      Find(_ecm);
    }
    if (_info.paused) {
      return;
    }
    for (const auto & pair : pairs_) {
      const auto leader = _ecm.Component<gz::sim::components::JointPosition>(pair.leader);
      const auto follower =
        _ecm.Component<gz::sim::components::JointPosition>(pair.follower);
      if (leader == nullptr || follower == nullptr || leader->Data().empty() ||
        follower->Data().empty())
      {
        continue;   // positions appear after the first physics step
      }
      const double target = pair.multiplier * leader->Data()[0] + pair.offset;
      double v = gain_ * (target - follower->Data()[0]);
      if (pair.max_velocity > 0.0) {
        v = std::clamp(v, -pair.max_velocity, pair.max_velocity);
      }
      auto cmd = _ecm.Component<gz::sim::components::JointVelocityCmd>(pair.follower);
      if (cmd == nullptr) {
        _ecm.CreateComponent(pair.follower, gz::sim::components::JointVelocityCmd({v}));
      } else if (!cmd->Data().empty()) {
        cmd->Data()[0] = v;
      }
    }
  }

private:
  struct Pair
  {
    gz::sim::Entity leader;
    gz::sim::Entity follower;
    double multiplier;
    double offset;
    double max_velocity;
  };

  void Find(gz::sim::EntityComponentManager & _ecm)
  {
    found_ = true;
    for (const auto joint : model_.Joints(_ecm)) {
      const auto axis = _ecm.Component<gz::sim::components::JointAxis>(joint);
      if (axis == nullptr || !axis->Data().Mimic()) {
        continue;
      }
      const auto mimic = axis->Data().Mimic().value();
      const auto leader = model_.JointByName(_ecm, mimic.Joint());
      const auto name = _ecm.Component<gz::sim::components::Name>(joint);
      if (leader == gz::sim::kNullEntity) {
        std::cerr << "[Err] jedy_mimic_joint: " << (name ? name->Data() : "?")
                  << " mimics '" << mimic.Joint() << "', which is not a joint of the model\n";
        continue;
      }
      for (const auto entity : {joint, leader}) {
        if (_ecm.Component<gz::sim::components::JointPosition>(entity) == nullptr) {
          _ecm.CreateComponent(entity, gz::sim::components::JointPosition());
        }
      }
      pairs_.push_back({leader, joint, mimic.Multiplier(), mimic.Offset(),
          axis->Data().MaxVelocity()});
      std::cout << "[Msg] jedy_mimic_joint: " << (name ? name->Data() : "?") << " = "
                << mimic.Multiplier() << " * " << mimic.Joint() << " + "
                << mimic.Offset() << "\n";
    }
  }

  gz::sim::Model model_{gz::sim::kNullEntity};
  double gain_{50.0};
  bool found_{false};
  std::vector<Pair> pairs_;
};

}  // namespace jedy_sim

GZ_ADD_PLUGIN(
  jedy_sim::JedyMimicJoint,
  gz::sim::System,
  jedy_sim::JedyMimicJoint::ISystemConfigurePriority,
  jedy_sim::JedyMimicJoint::ISystemConfigure,
  jedy_sim::JedyMimicJoint::ISystemPreUpdate)
