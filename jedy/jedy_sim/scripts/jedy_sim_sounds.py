#!/usr/bin/env python3
"""Motor sounds for the simulated Jedy, through the PC's speaker.

The real robot's servos and wheels can be heard; a simulator is silent, which
makes it hard to notice that an arm is still moving or that the base was told
to drive. This node plays a recorded sound while the base is commanded
(``/cmd_vel``) and another while a joint is commanded to move, through a
sound_play node (``sound_topic``).

Each sound is played once and played again as it ends, for as long as the
motion lasts, rather than looped by sound_play (PLAY_START) and stopped later:
a loop keeps playing if the stop never arrives -- this node killed, Gazebo
stopped -- and measured, it did, until sound_play itself was restarted. This
way at most one more play of the sound follows.

A joint counts as moving while its trajectory controller's reference velocity
(``controller_state``) is not zero: what the servos are told to do, not what
the joints happen to do. In Gazebo the arms sway when the base accelerates --
0.14 rad/s measured over 0.3 s, against 0.17 rad/s for an arm told to move 30
degrees in 3 s -- so the measured joint speed cannot tell the two apart.

Parameters
----------
sound_topic : str
    sound_play's request topic (sound_play_msgs/SoundRequest).
wheel_sound, servo_sound : str
    The sounds (wav files). Empty: ``share/jedy_sim/sounds/{wheels,servos}.wav``.
volume : float
    0..1.
namespace : str
    The bring-up's namespace, which the controllers are under.
controllers : list of str
    The trajectory controllers whose motion is heard.
reference_speed : float
    A controller counts as moving while some joint's reference velocity is
    above this [rad/s].
cmd_vel_timeout : float
    The base counts as driving this long after the last non-zero command [s]
    (the base itself stops 0.5 s after the last command).
"""

import contextlib
import math
import os
import time
import wave

import rclpy
from control_msgs.msg import JointTrajectoryControllerState
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from sound_play_msgs.msg import SoundRequest

#: Below this a velocity command is zero [m/s, rad/s].
STILL = 1e-3
#: The servo sound keeps going this long after the last moving reference [s],
#: so that a motion made of several short trajectories does not stutter.
SERVO_HOLD = 0.2
#: Play a sound again this long before it ends [s], so that the plays join
#: without a gap (sound_play restarts a sound that is still playing).
REPLAY_LEAD = 0.05


def duration(path):
    """Length of a wav file [s]."""
    with contextlib.closing(wave.open(path)) as w:
        return w.getnframes() / float(w.getframerate())


def default_sound(name):
    """``share/jedy_sim/sounds/<name>``."""
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory('jedy_sim'), 'sounds', name)


class SimSounds(Node):
    """The node; see the module docstring."""

    def __init__(self):
        super(SimSounds, self).__init__('jedy_sim_sounds')
        self.declare_parameter('sound_topic', '/jedy_sim/sound_effects')
        self.declare_parameter('wheel_sound', '')
        self.declare_parameter('servo_sound', '')
        self.declare_parameter('volume', 0.6)
        self.declare_parameter('namespace', '/jedy_fullbody')
        self.declare_parameter('controllers', [
            'head_controller', 'right_arm_controller', 'left_arm_controller',
            'right_gripper_controller', 'left_gripper_controller'])
        self.declare_parameter('reference_speed', 0.01)
        self.declare_parameter('cmd_vel_timeout', 0.5)
        p = self.get_parameter
        self.sounds = {
            'wheels': p('wheel_sound').value or default_sound('wheels.wav'),
            'servos': p('servo_sound').value or default_sound('servos.wav'),
        }
        for name, path in self.sounds.items():
            if not os.path.isfile(path):
                raise FileNotFoundError('{} sound not found: {}'.format(name, path))
        self.lengths = {name: duration(path) for name, path in self.sounds.items()}
        self.volume = p('volume').value
        self.reference_speed = p('reference_speed').value
        self.cmd_vel_timeout = p('cmd_vel_timeout').value
        self.pub = self.create_publisher(SoundRequest, p('sound_topic').value, 10)
        self.replay_at = {'wheels': None, 'servos': None}   # None: not playing
        self.last_drive = -math.inf
        self.moving_until = -math.inf
        self.create_subscription(TwistStamped, '/cmd_vel', self.on_cmd_vel, 10)
        for c in p('controllers').value:
            self.create_subscription(
                JointTrajectoryControllerState,
                '{}/{}/controller_state'.format(p('namespace').value, c),
                self.on_controller_state, 10)
        self.create_timer(0.05, self.update)
        self.get_logger().info('sounds: {}'.format(self.sounds))

    def on_cmd_vel(self, msg):
        t = msg.twist
        if max(abs(t.linear.x), abs(t.linear.y), abs(t.angular.z)) > STILL:
            self.last_drive = time.monotonic()

    def on_controller_state(self, msg):
        speeds = [abs(v) for v in msg.reference.velocities if math.isfinite(v)]
        if speeds and max(speeds) > self.reference_speed:
            # Wall clock, as the sound is.
            self.moving_until = time.monotonic() + SERVO_HOLD

    def request(self, name, command):
        self.pub.publish(SoundRequest(
            sound=SoundRequest.PLAY_FILE, command=command,
            volume=self.volume, arg=self.sounds[name], arg2=''))

    def set_playing(self, name, on):
        now = time.monotonic()
        if on:
            if self.replay_at[name] is None or now >= self.replay_at[name]:
                self.request(name, SoundRequest.PLAY_ONCE)
                self.replay_at[name] = now + max(
                    self.lengths[name] - REPLAY_LEAD, REPLAY_LEAD)
        elif self.replay_at[name] is not None:
            self.request(name, SoundRequest.PLAY_STOP)
            self.replay_at[name] = None

    def update(self):
        now = time.monotonic()
        self.set_playing('wheels', now - self.last_drive < self.cmd_vel_timeout)
        self.set_playing('servos', now < self.moving_until)

    def stop_all(self):
        for name in self.replay_at:
            self.set_playing(name, False)


def main():
    rclpy.init()
    node = SimSounds()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.stop_all()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
