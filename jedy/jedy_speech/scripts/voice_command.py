#!/usr/bin/env python3
"""Move Jedy by voice: listen on /speech/heard, answer on /speech/say.

    ros2 launch jedy_speech speech_demo.launch.py          # speaker + listener + RViz
    ros2 run jedy_speech voice_command                     # this
    ros2 run jedy_speech voice_command --text "右手を上げて"  # one command, no microphone

Understood (keywords, not grammar -- read ``COMMANDS`` and add your own):
    右手/左手/両手 を 上げて / 前に出して / 下げて
    うなずいて, 首を振って (いいえ)
    前に進んで, 下がって, 右を向いて, 左を向いて, 止まって
    こんにちは, 名前は

The real robot moves: keep clear of the arms and the base.
"""

import argparse
import re
import time
import unicodedata

import numpy as np
import rclpy
from skrobot.coordinates import Coordinates
from std_msgs.msg import String

from jedy_skrobot import connect

#: Hand targets relative to the shoulder (link0) [m]; y is mirrored for larm.
#: Checked reachable from reset_pose with the Jedy model (IK error < 1 mm);
#: 6 cm forward and 22 cm up is not.
ARM_TARGETS = {'up': (0.0, -0.06, 0.20), 'forward': (0.15, -0.06, 0.0)}


def normalize(text):
    """Full-width to half-width, no spaces or punctuation."""
    text = unicodedata.normalize('NFKC', text)
    return re.sub(r'[\s、。,.!?！？「」]', '', text)


class Commander:
    """Turns recognised text into a reply and a motion."""

    def __init__(self, robot, ri, say):
        self.robot, self.ri, self.say = robot, ri, say

    # -- motions --------------------------------------------------------------
    def _arms(self, sides, pose):
        self.robot.angle_vector(self.ri.angle_vector())
        if pose == 'down':
            self.robot.reset_pose()
        else:
            for side in sides:
                arm = getattr(self.robot, side)
                dx, dy, dz = ARM_TARGETS[pose]
                if side == 'larm':
                    dy = -dy
                target = arm.link_list[0].worldpos() + np.array([dx, dy, dz])
                ok = arm.inverse_kinematics(Coordinates(pos=target), rotation_axis=False)
                if ok is False or ok is None:
                    # From an odd current pose IK may not converge: once more
                    # from the reset pose (only this arm's joints are used).
                    saved = self.robot.angle_vector()
                    self.robot.reset_pose()
                    reset = self.robot.angle_vector()
                    self.robot.angle_vector(saved)
                    for joint in arm.joint_list:
                        joint.joint_angle(reset[self.robot.joint_list.index(joint)])
                    ok = arm.inverse_kinematics(Coordinates(pos=target),
                                                rotation_axis=False)
                if ok is False or ok is None:
                    self.say('ごめんなさい、その姿勢にはなれません')
                    return
        self.ri.angle_vector(self.robot.angle_vector(), 2.0)
        self.ri.wait_interpolation()

    def _head(self, joint, angles):
        for a in angles:
            self.robot.angle_vector(self.ri.angle_vector())
            getattr(self.robot, joint).joint_angle(a)
            self.ri.angle_vector(self.robot.angle_vector(), 0.4,
                                 controller_type='head_controller')
            self.ri.wait_interpolation('head_controller')

    def _base(self, vx, vtheta, seconds=2.0):
        self.ri.go_velocity(vx, 0.0, vtheta, duration=seconds)

    # -- text -> action -------------------------------------------------------
    def handle(self, text):
        """Do what ``text`` asks; returns the reply that was said."""
        t = normalize(text)
        sides = []
        if '両手' in t or '両腕' in t:
            sides = ['rarm', 'larm']
        elif '右手' in t or '右腕' in t:
            sides = ['rarm']
        elif '左手' in t or '左腕' in t:
            sides = ['larm']
        side_name = {('rarm',): '右手', ('larm',): '左手'}.get(tuple(sides), '両手')

        if sides and re.search('上げ|あげ', t):
            return self._do('{}を上げます'.format(side_name), self._arms, sides, 'up')
        if sides and re.search('前|まえ|出し|だし|伸ば', t):
            return self._do('{}を前に出します'.format(side_name), self._arms, sides, 'forward')
        if re.search('下げ|さげ|おろ|下ろ', t) and (sides or '手' in t):
            return self._do('手を下ろします', self._arms, sides, 'down')
        if re.search('うなず|頷', t):
            return self._do('はい', self._head, 'head_joint1', [0.3, 0.0, 0.3, 0.0])
        if re.search('首を振|いいえ|ノー', t):
            return self._do('いいえ', self._head, 'head_joint0', [0.4, -0.4, 0.4, 0.0])
        if re.search('止ま|とま|ストップ', t):
            return self._do('止まります', self.ri.stop)
        if re.search('前に進|進んで|すすんで|前進', t):
            return self._do('前に進みます', self._base, 0.1, 0.0)
        if re.search('下が|さが|後ろ|うしろ|バック', t):
            return self._do('下がります', self._base, -0.1, 0.0)
        if re.search('右を向|右に回|右向', t):
            return self._do('右を向きます', self._base, 0.0, -0.4)
        if re.search('左を向|左に回|左向', t):
            return self._do('左を向きます', self._base, 0.0, 0.4)
        if re.search('名前|だれ|誰', t):
            return self._do('ジェディです。知能ロボット行動プログラミング演習のロボットです')
        if re.search('こんにち|おはよ|こんばん|ハロー', t):
            return self._do('こんにちは。右手を上げて、のように話しかけてください')
        return self._do('すみません、もう一度言ってください')

    def _do(self, reply, action=None, *args):
        self.say(reply)
        if action is not None:
            action(*args)
        return reply


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--text', help='handle this one command and exit')
    args, ros_args = p.parse_known_args()

    robot, ri = connect()
    ri.servo_on()
    node = rclpy.create_node('jedy_voice_command')
    say_pub = node.create_publisher(String, '/speech/say', 10)
    log = node.get_logger()

    def say(text):
        log.info('reply: ' + text)
        say_pub.publish(String(data=text))

    commander = Commander(robot, ri, say)
    if args.text:
        deadline = time.monotonic() + 5.0
        while say_pub.get_subscription_count() == 0 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        commander.handle(args.text)
        rclpy.spin_once(node, timeout_sec=0.5)
        return

    def on_heard(msg):
        log.info('heard: ' + msg.data)
        commander.handle(msg.data)

    node.create_subscription(String, '/speech/heard', on_heard, 10)
    log.info('waiting for /speech/heard (say "右手を上げて")')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
