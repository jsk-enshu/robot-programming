#!/usr/bin/env python3
"""scikit-robot model + live interface for Jedy's whole body.

    head_joint0          servo id 32        head_joint1          servo id 34
    rarm_joint0..6       servo ids 0,2,4,6,8,10,12
    larm_joint0..6       servo ids 1,3,5,7,9,11,13
    rarm_gripper_joint   servo id 14        larm_gripper_joint   servo id 15
    the four wheels      servo ids 20..23

The board scan on 2026-08-03 found all 22 servos present, so every joint this
module names is one the bus actually drives.

What is still excluded, and why
-------------------------------
* The four mecanum wheels. They are ``continuous`` joints driven by
  kondo_ics_wheel in VELOCITY mode, so they are not part of any trajectory and
  are not in :data:`CONTROLLED_JOINTS`. Drive the base with a twist on
  ``/cmd_vel`` instead.
* ``rarm_mimic_gripper_joint`` and ``larm_mimic_gripper_joint``. They carry a
  URDF ``<mimic>`` and have no servo, so nothing can command them; they follow
  their driven side in the model and in TF.

The model is scikit-robot's own ``skrobot.models.jedy.Jedy``, which is what
gives the shell ``robot.rarm``, ``robot.larm``, ``robot.head`` and their
``end_coords`` -- so ``robot.rarm.move_end_pos(...)`` and IK work. A model built
by loading a URDF into a bare ``RobotModel`` has none of that and raises
``NotImplementedError`` on ``robot.rarm``.

It carries its own URDF, which is NOT jedy_description's. The joint names match
and the arm chains from link0 onward are identical joint by joint, so commands
are unaffected; the base frames are not, which is what :data:`GHOST_OFFSET_Z` is
about. Read that before trusting anything expressed in base_link.
"""

from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from skrobot.models.jedy import Jedy as _SkrobotJedy  # noqa: E402

#: The joints the bus drives, in the servicer's order.
#:
#: This order is the contract shared by four places: the ``<ros2_control>``
#: block in jedy_bringup's jedy_fullbody_actuo.urdf.xacro, ``joints:`` in
#: jedy_fullbody_rcb4.yaml, the shm layout actuo_hardware maps BY INDEX, and the
#: controller yaml. It is written out here rather than derived so that a
#: mismatch is a startup error (see check_against_servicer) rather than a
#: trajectory the controller rejects wholesale at the moment you ask the robot
#: to move.
CONTROLLED_JOINTS = [
    "head_joint0",
    "head_joint1",
    "rarm_joint0",
    "rarm_joint1",
    "rarm_joint2",
    "rarm_joint3",
    "rarm_joint4",
    "rarm_joint5",
    "rarm_joint6",
    "larm_joint0",
    "larm_joint1",
    "larm_joint2",
    "larm_joint3",
    "larm_joint4",
    "larm_joint5",
    "larm_joint6",
    "rarm_gripper_joint",
    "larm_gripper_joint",
]

#: Namespace the bring-up launch puts the controller manager under.
NAMESPACE = "/jedy_fullbody"

#: One trajectory controller per limb. Each claims disjoint command interfaces,
#: so all five are active at once and a head goal does not replace an arm's
#: trajectory.
#:
#: The grippers are their own controllers rather than part of the arms, which is
#: the whole point of the split: a gripper can close while the arm it is on is
#: mid-reach. It also means a prefix rule cannot derive this table --
#: rarm_gripper_joint starts with "rarm_" and is NOT right_arm_controller's.
LIMBS = {
    "head": ("head_controller", ["head_joint0", "head_joint1"]),
    "right_arm": ("right_arm_controller",
                  ["rarm_joint0", "rarm_joint1", "rarm_joint2", "rarm_joint3",
                   "rarm_joint4", "rarm_joint5", "rarm_joint6"]),
    "left_arm": ("left_arm_controller",
                 ["larm_joint0", "larm_joint1", "larm_joint2", "larm_joint3",
                  "larm_joint4", "larm_joint5", "larm_joint6"]),
    "right_gripper": ("right_gripper_controller", ["rarm_gripper_joint"]),
    "left_gripper": ("left_gripper_controller", ["larm_gripper_joint"]),
}


#: What to add to a marker's z so a model built from scikit-robot's own Jedy
#: lands on the robot RViz is drawing.
#:
#: The two descriptions do not agree on what ``base_link`` is, and this is the
#: size of that disagreement. scikit-robot's jedy.urdf puts base_link ON THE
#: GROUND -- base_footprint_joint is xyz="0 0 0", so the two frames coincide,
#: and the wheels hang off base_link at z=+0.030 against a 0.0301 m wheel
#: radius. jedy_description's jedy_gz.xacro puts base_link 0.127 m up, at the
#: chassis: the same wheels sit at z=-0.097 there.
#:
#: Read three ways, all agreeing, so it is a fact about the two files rather
#: than a number tuned until the picture looked right:
#:
#:   base_laser     z = 0.127 (skrobot) against 0.000 (jedy_description),
#:                  same joint, same parent, same physical laser
#:   head_link1     z = 0.246 against 0.119
#:   rarm_end_coords z = -0.079 against -0.206
#:
#: 0.127 m is the chassis height (the old pseudo_base_joint was
#: "xyz 0 0 0.127").
#:
#: THIS IS A PATCH OVER A REAL DISCREPANCY, not a fix for one. Two files
#: describe one robot and disagree, here about the base and elsewhere about
#: where the D405 is mounted (camera_joint is [0.02, 0.01, 0.06] in
#: jedy_description and [0, 0.005, 0.05] in scikit-robot's copy, ~2 cm apart).
#: Reconciling them upstream is what removes this constant; until then it only
#: moves the picture, and anything reasoning in base_link coordinates still has
#: to know which of the two conventions it is holding.
GHOST_OFFSET_Z = -0.127



#: The model, straight from scikit-robot.
#:
#: Not a subclass and not a local copy: ``skrobot.models.jedy.Jedy`` already
#: defines what a bare RobotModel cannot, and what an interactive shell is
#: mostly for. ``robot.rarm`` / ``.larm`` / ``.head`` are limb sub-models with
#: ``end_coords`` attached, which is what makes ``robot.rarm.move_end_pos(...)``
#: and inverse kinematics work at all -- on a plain RobotModel those raise
#: NotImplementedError, because the base class has no way to guess which links
#: are an arm or where its tip is.
#:
#: It brings its own URDF with it, downloaded to ~/.skrobot rather than read
#: from jedy_description, and that is a deliberate choice with a cost. See
#: :data:`GHOST_OFFSET_Z`: the two descriptions disagree about where base_link
#: is, so anything drawn or reasoned about in base_link coordinates has to know
#: which convention it holds. Joint COMMANDS are unaffected -- the joint names
#: are identical and the arm chains from link0 onward match exactly, verified
#: joint by joint -- so ``ri.angle_vector()`` sends the same numbers either way.
JedyFullbody = _SkrobotJedy


# A sibling module, installed next to this one in share/jedy_skrobot/scripts.
from servicer_robot_interface import ServicerRobotInterface  # noqa: E402


class JedyFullbodyROS2RobotInterface(ServicerRobotInterface):
    """Drives Jedy through the same JTC the RViz panel and the CLI use.

    The bring-up namespaces the controller manager (``/jedy_fullbody``), and
    getting that across takes care, because skrobot's `namespace` argument
    CANNOT be used for it here. The ros2 base joins unconditionally::

        controller_action = '{}/{}'.format(namespace, controller_action)

    while ServicerRobotInterface hands it an already-absolute
    ``/<controller>/follow_joint_trajectory``. Any namespace at all therefore
    produces a doubled slash and rclpy refuses the name outright::

        ValueError: invalid topic name '/jedy_fullbody//jedy_fullbody/...'
                    topic name must not contain repeated '/'

    (which is a good failure -- it is loud and names the string.) So the
    namespace goes into the controller NAME, which is what the absolute path is
    built from, and into ``joint_states_topic`` explicitly. `namespace` itself
    stays None.
    """

    def __init__(self, robot, limb="right_arm", joint_names=None, **kwargs):
        if limb not in LIMBS:
            raise ValueError(
                "limb must be one of {}, got {!r}".format(
                    ", ".join(sorted(LIMBS)), limb))
        controller, joints = LIMBS[limb]
        self.limb = limb
        kwargs.setdefault("node_name", "jedy_fullbody_skrobot_interface")
        kwargs.setdefault("joint_states_topic", NAMESPACE + "/joint_states")
        super(JedyFullbodyROS2RobotInterface, self).__init__(
            robot,
            list(joint_names or joints),
            controller_name=kwargs.pop(
                "controller_name",
                "{}/{}".format(NAMESPACE.strip("/"), controller)),
            **kwargs)

    def check_against_servicer(self, timeout=5.0):
        """Compare this interface's joint list with what the bus is driving.

        Raises rather than warns. A trajectory naming a joint the controller
        does not own is rejected **whole** -- the limb simply does not move, and
        the reason is in the controller's log, not in front of the operator. A
        list that is a subset in the wrong order is worse: it moves the wrong
        joints. Better to fail here, before anything is commanded.
        """
        from servicer_robot_interface import fetch_joint_names
        driven = list(fetch_joint_names(self, self._servo_ns, timeout))
        mine = list(self.joint_names)
        missing = [j for j in mine if j not in driven]
        if missing:
            raise RuntimeError(
                "this shell would command {}, but the servicer is not driving "
                "{}. It drives {}. A trajectory naming a joint the bus does not "
                "have is rejected whole -- the robot simply does not move, and "
                "the reason is in a log rather than in front of you.".format(
                    mine, missing, driven))
        # SUBSET, not equality: one limb per controller. The ORDER within this
        # limb still has to match the servicer's, because that order is the shm
        # layout that actuo_hardware maps by index.
        relative = [j for j in driven if j in mine]
        if relative != mine:
            raise RuntimeError(
                "this shell orders its joints {} but the servicer has them {}. "
                "Same joints, different order, which sends each angle to the "
                "wrong one.".format(mine, relative))
        return mine
