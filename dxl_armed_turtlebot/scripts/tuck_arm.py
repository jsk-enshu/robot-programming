#!/usr/bin/env python


import rospy
import argparse
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState
from math import pi

class Tuck(object):
    def __init__(self, tuck_cmd):
        self._done = False
        self._tuck = tuck_cmd
        self._tuck_rate = rospy.Rate(5.0)  # Hz
        self._tuck_threshold = 0.3  # radians
        self._joint_moves = {
            'tuck': [pi/2, 0, pi/4, 0, pi/2, pi/2, 0],
            'untuck': [0, 0, -pi/2, 0, pi/2, 0, 0],
        }
        self._arm_state = 'none'
        self._pub = rospy.Publisher('/trajectory_controller/command', JointTrajectory, queue_size=1)
        self._sub = rospy.Subscriber('/trajectory_controller/state', JointTrajectoryControllerState, self._check_arm_state)

    def _check_arm_state(self, msg):
        diff_check = lambda a, b: abs(a - b) <= self._tuck_threshold
        angles = msg.actual.positions
        tuck_goal = map(diff_check, angles, self._joint_moves['tuck'])
        untuck_goal = map(diff_check, angles, self._joint_moves['untuck'])

        if all(tuck_goal):
            self._arm_state = 'tuck'
        elif all(untuck_goal):
            self._arm_state = 'untuck'
        else:
            self._arm_state = 'none'

    def _prepare_to_tuck(self):
        pass

    def _move_to(self, goal):
        traj = JointTrajectory()
        traj.joint_names  = [
            'arm_link1_joint', 
            'arm_link2_joint', 
            'arm_link3_joint',
            'arm_link4_joint',
            'arm_link5_joint',
            'arm_link6_joint',
            'arm_link7_joint'
        ]
        traj.points.append(JointTrajectoryPoint())
        traj.points[0].positions = self._joint_moves[goal]
        traj.points[0].time_from_start = rospy.Duration(3)

        while self._arm_state != goal:
            self._pub.publish(traj)
            self._tuck_rate.sleep()

    def supervised_tuck(self):
        # Update our starting state to check if arms are tucked
        self._prepare_to_tuck()
        # Tuck Arms
        if self._tuck == True:
            # If arms are already tucked, report this to user and exit.
            if self._arm_state == 'tuck':
                rospy.loginfo("Tucking: Arm already in 'Tucked' position.")
            else:
                rospy.loginfo("Tucking: arm is not Tucked.")
                self._move_to('tuck')
            self._done = True
            return

        # Untuck Arms
        else:
            # If arms already untucked
            if self._arm_state == 'untuck':
                rospy.loginfo("Untucking: Arm already Untucked")
            else:
                rospy.loginfo("Untucking: arm is not Untucked.")
                self._move_to('untuck')
            self._done = True
            return

    def clean_shutdown(self):
        if not self._done:
            rospy.logwarn('Aborting: Shutting down safely...')
    

def main():
    parser = argparse.ArgumentParser()
    tuck_group = parser.add_mutually_exclusive_group(required=True)
    tuck_group.add_argument("-t", "--tuck", dest="tuck",
        action='store_true', default=False, help="tuck arms")
    tuck_group.add_argument("-u", "--untuck", dest="untuck",
        action='store_true', default=False, help="untuck arms")
    args = parser.parse_args(rospy.myargv()[1:])
    tuck = args.tuck

    rospy.loginfo("Initializing node... ")
    rospy.init_node("tuck_arms")
    rospy.loginfo("%sucking arms" % ("T" if tuck else "Unt",))
    tucker = Tuck(tuck)
    rospy.on_shutdown(tucker.clean_shutdown)
    tucker.supervised_tuck()
    rospy.loginfo("Finished %suck" % ("T" if tuck else "Unt",))

if __name__ == "__main__":
    main()
