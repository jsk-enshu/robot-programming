#!/usr/bin/env python3
"""Servo ON/OFF for Jedy in a simulator: the real robot's SetServo action.

    ros2 launch jedy_bringup jedy_gazebo.launch.py     # starts this

Serves ``/actuo_bus_servicer/servo/set`` (actuo_msgs/action/SetServo) with the
real servicer's rules (jedy_sim.sim_servo). A released joint is handed to the
simulator, which stops applying the controller's command to it, so it falls
under gravity while ``joint_states`` keeps reporting where it is; the
trajectory controllers stay active, as on the robot. The simulator side is the
``jedy_servo_switch`` system plugin in Gazebo, on two topics:

    /jedy_fullbody/servo/released        std_msgs/String  out (to the simulator)
    /jedy_fullbody/servo/released_state  std_msgs/String  in  (its ack)

The action answers after the simulator reports the new set, so a returned
``ri.servo_off()`` means the joints are limp.

It also publishes the real servicer's ``/actuo_bus_servicer/servo/state``
(actuo_msgs/ServoState: which joints hold, which are present), so RViz's
missing-servo markers and jedy_skrobot's checks see the same thing as on the
robot. ``--missing rarm_joint3,...`` (launch: ``missing_servos:=``) rehearses
a robot with servos that do not answer: they are limp, refuse servo ON, and
are reported absent.

"Commanded" for the jump check is each trajectory controller's
``controller_state.reference`` (what it is commanding now) and, for the
wheels, mecanum_drive_controller's reference twist turned into wheel speeds.
"""

import argparse
import math
import os
import sys
import threading

import rclpy
from actuo_msgs.msg import ServoState
from control_msgs.msg import JointTrajectoryControllerState
from control_msgs.msg import MecanumDriveControllerState
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from jedy_sim.sim_servo import load_controllers
from jedy_sim.sim_servo import SERVO_ACTION
from jedy_sim.sim_servo import SimServoServer
from jedy_sim.sim_servo import SimServoSwitch
from jedy_sim.sim_servo import wheel_speeds


def share_file(package, *parts):
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory(package), *parts)


class SimServo(Node):
    """The ROS side of the simulator's servo switch."""

    def __init__(self, controllers_path, namespace, missing=()):
        super().__init__('jedy_sim_servo')
        (controller_joints, self._wheels, self._radius,
         self._k) = load_controllers(controllers_path, namespace)
        position = [j for js in controller_joints.values() for j in js]
        self.switch = SimServoSwitch(position, self._wheels, missing=missing)
        if self.switch.missing:
            self.get_logger().warning(
                'SIMULATED missing servos (always limp, servo ON refused): '
                '{}'.format(', '.join(self.switch.missing)))
        self._lock = threading.Lock()
        self._measured = {}
        self._commanded = {}
        self._applied = None
        group = ReentrantCallbackGroup()
        self.create_subscription(
            JointState, namespace + '/joint_states', self._on_joint_states,
            qos_profile_sensor_data, callback_group=group)
        for name in controller_joints:
            self.create_subscription(
                JointTrajectoryControllerState,
                '{}/{}/controller_state'.format(namespace, name),
                self._on_trajectory_state, 10, callback_group=group)
        self.create_subscription(
            MecanumDriveControllerState,
            namespace + '/mecanum_drive_controller/controller_state',
            self._on_mecanum_state, 10, callback_group=group)
        self.create_subscription(
            String, namespace + '/servo/released_state', self._on_applied, 10,
            callback_group=group)
        self._pub = self.create_publisher(
            String, namespace + '/servo/released', 10)
        # Repeat the wanted set: a message the bridge dropped while it was
        # still connecting must not leave the plugin on an old set.
        self.create_timer(0.5, self._republish, callback_group=group)
        self._server = SimServoServer(
            self, self.switch, self._snapshot, self._apply,
            applied=self._get_applied, callback_group=group)
        # The real servicer's state topic, transient_local as there.
        self._state_pub = self.create_publisher(
            ServoState, SERVO_ACTION.rsplit('/', 1)[0] + '/state',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_timer(0.2, self._publish_state, callback_group=group)

    def _on_joint_states(self, msg):
        with self._lock:
            self._measured.update(zip(msg.name, msg.position))

    def _on_trajectory_state(self, msg):
        with self._lock:
            self._commanded.update(zip(msg.joint_names,
                                       msg.reference.positions))

    def _on_mecanum_state(self, msg):
        t = msg.reference_velocity
        # NaN once the reference times out (no /cmd_vel for 0.5 s): no motion.
        vx, vy, wz = [0.0 if math.isnan(v) else v
                      for v in (t.linear.x, t.linear.y, t.angular.z)]
        with self._lock:
            self._commanded.update(zip(
                self._wheels, wheel_speeds(vx, vy, wz, self._radius, self._k)))

    def _on_applied(self, msg):
        with self._lock:
            self._applied = [n for n in msg.data.split(',') if n]

    def _get_applied(self):
        with self._lock:
            return None if self._applied is None else list(self._applied)

    def _snapshot(self):
        with self._lock:
            return dict(self._measured), dict(self._commanded)

    def _apply(self, released):
        self._pub.publish(String(data=','.join(released)))

    def _republish(self):
        self._apply(self.switch.released)

    def _publish_state(self):
        msg = ServoState()
        msg.stamp = self.get_clock().now().to_msg()
        msg.joints = self._server.joint_messages(*self._snapshot())
        msg.all_enabled = all(j.enabled for j in msg.joints)
        msg.any_would_jump = any(j.would_jump for j in msg.joints)
        msg.jump_threshold = self.switch.jump_threshold
        msg.mode = 'position'
        self._state_pub.publish(msg)


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--controllers', default=share_file(
        'jedy_bringup', 'config', 'jedy_fullbody_controllers.yaml'))
    p.add_argument('--namespace', default='/jedy_fullbody')
    p.add_argument('--missing', default='',
                   help='comma-separated joints whose servo does not answer '
                        '(rehearsal of a cable out)')
    args = p.parse_args(rclpy.utilities.remove_ros_args(
        sys.argv if argv is None else argv)[1:])
    rclpy.init(args=sys.argv if argv is None else argv)
    missing = [j.strip() for j in args.missing.split(',') if j.strip()]
    node = SimServo(args.controllers, args.namespace, missing)
    # Single-threaded, like the real servo switch (actuo_ros servo_node). The
    # action's wait for the simulator is a coroutine, so nothing blocks the
    # thread.
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
