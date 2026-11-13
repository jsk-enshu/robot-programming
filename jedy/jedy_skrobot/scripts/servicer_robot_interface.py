"""A scikit-robot interface for ANY actuo-driven robot, configured at runtime.

Instead of hardcoding one robot's joint names and URDF, it asks the running
system:

* the **model** comes from the ``/robot_description`` topic, so it is by
  construction the same URDF ros2_control and RViz are using;
* the **joint list** comes from the servicer's ``ServoState``, which is the
  authoritative list of joints actuo is actually driving.

Nothing about a particular robot appears below, which is the point: a
reconfigurable arm changes its joints and a hand-kept list goes stale silently.

The one rule that matters
-------------------------
**Never route a commanded angle through the skrobot model.** ``RobotModel``
clamps every angle into the URDF's limits and gives no hint that it did. On
one actuo-driven robot a head joint genuinely sat at -5.7486 rad while the
URDF said +/-2.35 -- an offset that had not been taught yet -- so
``potentio_vector()`` reports -2.35, a 3.4 rad lie, and re-syncing or replaying
off that value would command exactly the slam the servo design exists to
prevent. Angles here are carried as raw arrays indexed by
``robot.joint_list``; the model is used for limits, velocity limits and
kinematics, never as a pass-through for numbers coming off the wire.
"""

from __future__ import annotations

import os
import tempfile
import threading

import control_msgs.action
import numpy as np
import rclpy
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)

from skrobot.interfaces.ros2.base import ROS2RobotInterfaceBase
from skrobot.models.urdf import RobotModelFromURDF

#: The servicer publishes these transient_local, so a late subscriber still
#: gets the current value rather than waiting for the next change.
LATCHED = QoSProfile(depth=1,
                     history=HistoryPolicy.KEEP_LAST,
                     reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)


def fetch_robot_description(node, topic="/robot_description", timeout=10.0):
    """Return the URDF text that ros2_control is actually running.

    Taken off the topic rather than off disk on purpose: a model that silently
    disagrees with the robot about joint order, limits or mimic joints is worse
    than no model, and "the file I expanded" and "the description the
    controller loaded" can drift apart.
    """
    from std_msgs.msg import String
    msg = wait_for_message(node, String, topic, timeout)
    if msg is None:
        raise RuntimeError(
            "no {} within {}s -- is robot_state_publisher running?".format(
                topic, timeout))
    return msg.data


def wait_for_message(node, msg_type, topic, timeout, qos=LATCHED):
    """One message off ``topic``, spinning ``node`` ourselves.

    Only safe on a node NO executor owns. Spinning a node that a background
    executor is already spinning corrupts the shared wait set -- it surfaces as
    ``RCLError: wait set index for status subscription is out of bounds`` from
    the executor thread, which then dies silently and everything downstream
    just stops receiving. Hence this function
    is used only during bootstrap, and :meth:`ServicerRobotInterface.servo_state`
    waits on a latched subscription instead.
    """
    got = []
    sub = node.create_subscription(msg_type, topic, got.append, qos)
    try:
        deadline = node.get_clock().now().nanoseconds + int(timeout * 1e9)
        while not got and node.get_clock().now().nanoseconds < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.destroy_subscription(sub)
    return got[-1] if got else None


def send_goal_blocking(client, goal, what, timeout=120.0):
    """Send an action goal, insist it was ACCEPTED, and return its result.

    ``rclpy``'s own ``ActionClient.send_goal`` looks like the thing to use and
    is not: it returns the **result response**, not a goal handle, so the
    obvious ``if not handle.accepted`` is an ``AttributeError`` on
    ``'..._GetResult_Response' object has no attribute 'accepted'`` -- which is
    exactly how this was found, at the moment torque was about to go on. It
    also never checks acceptance at all, so a REJECTED goal comes back looking
    like an empty result and the caller carries on as if the robot had agreed.

    Rejection matters more than anything else here: the servicer's refusal to
    enable torque, and the controller's refusal of a re-sync, are the safety
    mechanism. So this waits on the goal handle explicitly and raises with the
    reason.

    Requires an executor to be spinning ``client``'s node -- the futures below
    are completed by it. That is the case everywhere this is called.
    """
    import threading as _threading

    done = _threading.Event()
    future = client.send_goal_async(goal)
    future.add_done_callback(lambda _f: done.set())
    if not done.wait(timeout):
        raise RuntimeError("{}: no response within {}s".format(what, timeout))
    if future.exception() is not None:
        raise future.exception()
    handle = future.result()
    if not handle.accepted:
        raise RuntimeError("{}: the goal was REJECTED".format(what))

    done = _threading.Event()
    result_future = handle.get_result_async()
    result_future.add_done_callback(lambda _f: done.set())
    if not done.wait(timeout):
        raise RuntimeError(
            "{}: accepted but no result within {}s".format(what, timeout))
    if result_future.exception() is not None:
        raise result_future.exception()
    return result_future.result()


def fetch_joint_names(node, servo_ns="/actuo_bus_servicer", timeout=10.0):
    """The joints actuo is actually driving, from the servicer's own state.

    The authoritative list, and it costs one latched message. A constant in a
    script would go stale the first time this reconfigurable arm is rebuilt,
    and would do it silently.
    """
    from actuo_msgs.msg import ServoState
    topic = servo_ns.rstrip("/") + "/servo/state"
    msg = wait_for_message(node, ServoState, topic, timeout)
    if msg is None:
        raise RuntimeError(
            "no ServoState on {} within {}s -- the bus servicer is not "
            "running.".format(topic, timeout))
    return [j.name for j in msg.joints]


def load_robot_from_description(node, topic="/robot_description",
                                timeout=10.0):
    """Build a scikit-robot model from the live ``/robot_description``."""
    from skrobot.utils.urdf import no_mesh_load_mode

    text = fetch_robot_description(node, topic, timeout)
    handle = tempfile.NamedTemporaryFile(mode="w", suffix=".urdf", delete=False)
    handle.write(text)
    handle.close()
    try:
        # Meshes are irrelevant here and `package://` would have to be resolved
        # against the ament index; the joints are what this model is for.
        with no_mesh_load_mode():
            return RobotModelFromURDF(urdf_file=handle.name)
    finally:
        os.unlink(handle.name)


class ServicerRobotInterface(ROS2RobotInterfaceBase):
    """Drives an actuo robot through its JTC, and its torque through SetServo.

    Parameters
    ----------
    robot : skrobot RobotModel
    joint_names : list of str
        The joints the controller owns, in the controller's order.
    controller_name : str
        e.g. ``arm_controller``.
    servo_ns : str
        The bus servicer's node namespace, e.g. ``/actuo_bus_servicer``.
    """

    def __init__(self, robot, joint_names, controller_name="arm_controller",
                 servo_ns="/actuo_bus_servicer", **kwargs):
        self._joint_names = list(joint_names)
        self._controller_name = controller_name
        self._servo_ns = servo_ns.rstrip("/")
        kwargs.setdefault("node_name", "jedy_shell_interface")
        # The base defaults to 3 s, which is a DISCOVERY race on a live system,
        # not a real timeout: the action server was present the whole time and
        # `ros2 action list` showed it, but a freshly created node had not
        # matched it yet. Losing that race used to leave a half-built interface
        # (see below), so give discovery room.
        kwargs.setdefault("controller_timeout", 15)
        super(ServicerRobotInterface, self).__init__(robot, **kwargs)

        # ROS2RobotInterfaceBase does NOT raise when the controller's action
        # server never answers. It logs a warning, sets joint_action_enable =
        # False and returns early -- leaving controller_table empty and
        # controller_type at the default 'default_controller'. The program then
        # runs on, records a whole teaching session, and dies with
        # `KeyError: 'default_controller'` deep inside angle_vector_duration at
        # the moment it tries to move. Observed, and it threw away the
        # operator's recording. Fail here instead, while nothing is lost.
        if not self.joint_action_enable or \
                controller_name not in self.controller_table:
            raise RuntimeError(
                "could not reach /{0}/follow_joint_trajectory within {1}s, so "
                "this interface cannot command the robot. Check:\n"
                "  ros2 control list_controllers        # is {0} 'active'?\n"
                "  ros2 action list | grep {0}".format(
                    controller_name, kwargs["controller_timeout"]))
        self.controller_type = controller_name

        self._servo_client = None
        self._traj_client = None

        # ONE long-lived latched subscription, filled by whatever executor
        # owns this node. Reading it must never spin the node: see
        # wait_for_message for what happens when two executors share one.
        from actuo_msgs.msg import ServoState
        self._servo_state = None
        self._servo_state_seen = threading.Event()
        self._servo_state_sub = self.create_subscription(
            ServoState, self._servo_ns + "/servo/state",
            self._on_servo_state, LATCHED)

    def _on_servo_state(self, msg):
        self._servo_state = msg
        self._servo_state_seen.set()

    @property
    def joint_names(self):
        return list(self._joint_names)

    def default_controller(self):
        # A METHOD, not a @property: the base calls `self.default_controller()`
        # and a property returns a list, on which the base dies with
        # "'list' object is not callable".
        return [dict(
            controller_type=self._controller_name,
            controller_action="/{}/follow_joint_trajectory".format(
                self._controller_name),
            # `controller_state`, not `state`: jazzy's JTC renamed it and the
            # wrong name here just silently never reports.
            controller_state="/{}/controller_state".format(
                self._controller_name),
            action_type=control_msgs.action.FollowJointTrajectory,
            joint_names=list(self._joint_names),
        )]

    # --- what the robot is actually doing -----------------------------------
    def servo_state(self, timeout=5.0):
        """The servicer's own per-joint picture: enabled, mismatch, would_jump.

        Read rather than derived. ``would_jump`` is computed server-side
        precisely so that no client re-derives the safety rule and gets it
        wrong.

        Waits on the standing subscription rather than spinning: this node is
        normally owned by a background executor, and spinning it from here as
        well corrupts the wait set (see :func:`wait_for_message`).
        """
        if not self._servo_state_seen.wait(timeout):
            raise RuntimeError(
                "no ServoState on {}/servo/state within {}s -- the bus "
                "servicer is not running, or nothing is spinning this node. "
                "The topic is transient_local, so a stale value would have "
                "looked live.".format(self._servo_ns, timeout))
        return self._servo_state

    def measured(self, timeout=5.0):
        """Where the robot IS, per joint, unclamped. See the module docstring."""
        return {j.name: j.measured for j in self.servo_state(timeout).joints}

    def outside_model_limits(self, measured=None):
        """{name: (measured, lo, hi)} for joints the URDF cannot represent.

        Non-empty means ``potentio_vector()``, ``angle_vector()`` and any
        viewer are showing a pose the robot is not in.
        """
        out = {}
        for name, value in (measured or self.measured()).items():
            joint = getattr(self.robot, name, None)
            if joint is None:
                continue
            if value < joint.min_angle or value > joint.max_angle:
                out[name] = (value, joint.min_angle, joint.max_angle)
        return out

    # --- angle vectors, without the model's clamp ----------------------------
    def angle_vector_from(self, angles, base=None):
        """Build a raw angle vector indexed by ``robot.joint_list``.

        ``angles`` is ``{joint name: rad}``. Joints not mentioned keep their
        value from ``base`` (default: the model's current state). The result
        never passes through ``robot.angle_vector(v)``, so nothing is clamped
        -- which is the entire reason this helper exists rather than the
        obvious one-liner.
        """
        vector = np.array(self.robot.angle_vector() if base is None else base,
                          dtype=float)
        for name, value in angles.items():
            joint = getattr(self.robot, name, None)
            if joint is None or joint not in self.robot.joint_list:
                continue
            vector[self.robot.joint_list.index(joint)] = float(value)
        return vector

    def wait_for_trajectory(self, timeout=300.0):
        """Block until the last trajectory really finishes. Raise if it did not.

        Use this INSTEAD of ``wait_interpolation()``, which has a race that
        silently skips the wait. It reads ``action._current_goal_handle``,
        which ``send_ros_controller`` sets from an **async** goal-response
        callback; if that callback has not fired yet -- and on a freshly
        created node it has not -- ``get_result_async()`` returns None, the
        method logs "No active goal to wait for" at INFO and returns
        immediately. Measured: an 8-second move "completed" in 0.4 s, the arm
        was then torqued off mid-trajectory, and nothing in the output said so.
        Worse, it returns the same value for "finished", "rejected" and "never
        waited", so no caller can tell them apart.

        This waits on the futures themselves, which are the things that
        actually complete, and reports the controller's ``error_code``.
        """
        import threading as _threading

        results = []
        for action in self.controller_table[self.controller_type]:
            future = getattr(action, "_current_future", None)
            if future is None:
                continue

            done = _threading.Event()
            future.add_done_callback(lambda _f, e=done: e.set())
            if future.done():
                done.set()
            if not done.wait(timeout):
                raise RuntimeError(
                    "the trajectory controller never answered the goal "
                    "within {}s".format(timeout))
            handle = future.result()
            if not handle.accepted:
                raise RuntimeError(
                    "the trajectory controller REJECTED the motion -- nothing "
                    "moved. Is {} active?".format(self._controller_name))

            done = _threading.Event()
            result_future = handle.get_result_async()
            result_future.add_done_callback(lambda _f, e=done: e.set())
            if result_future.done():
                done.set()
            if not done.wait(timeout):
                raise RuntimeError(
                    "the motion did not finish within {}s".format(timeout))
            result = result_future.result().result
            if result.error_code != 0:
                raise RuntimeError(
                    "the trajectory controller aborted: error_code={} {}".format(
                        result.error_code, result.error_string))
            results.append(result)
        return results

    # --- servo (torque) ------------------------------------------------------
    def _servo(self):
        from rclpy.action import ActionClient
        from actuo_msgs.action import SetServo
        if self._servo_client is None:
            self._servo_client = ActionClient(
                self, SetServo, self._servo_ns + "/servo/set")
        if not self._servo_client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError(
                "no servo action server at {}/servo/set -- is the bus "
                "servicer running?".format(self._servo_ns))
        return self._servo_client

    def _send_servo(self, enable, joints, force):
        from actuo_msgs.action import SetServo
        goal = SetServo.Goal()
        goal.enable = enable
        goal.joints = list(joints or [])
        goal.force = force
        response = send_goal_blocking(
            self._servo(), goal,
            "servo {}".format("on" if enable else "off"))
        if not response.result.ok:
            raise RuntimeError(response.result.reason)
        return response.result.reason

    def servo_off(self, joints=None):
        """Release torque. The arm goes limp and falls under gravity."""
        return self._send_servo(False, joints, False)

    def servo_on(self, joints=None, resync=True, force=False, time=0.5):
        """Apply torque, holding wherever the robot is right now.

        The servicer refuses to enable while the controller commands somewhere
        other than where the robot is -- the normal state after hand-moving,
        and the refusal is the point, because enabling would throw the joint
        there. The servicer cannot fix that from its side: the setpoint belongs to the
        controller. So this first asks the controller for a trajectory to the
        pose the robot is ALREADY in -- a move that moves nothing, whose only
        effect is that the setpoint lands on the measured pose -- and only then
        enables. Same trick as ``on_activate`` at start-up, and as Dynamixel
        practice (write goal = present, then torque enable); taken from
        kxr_controller, which does exactly this on servo-on.
        """
        if resync:
            self.hold_current_pose(joints, time)
        return self._send_servo(True, joints, force)

    def hold_current_pose(self, joints=None, time=0.5):
        """Command the measured pose. Moves nothing; re-seats the setpoint.

        Hand-built rather than via ``angle_vector()`` because that routes the
        targets through the model, and the model clamps.
        """
        from builtin_interfaces.msg import Duration
        from control_msgs.action import FollowJointTrajectory
        from rclpy.action import ActionClient
        from trajectory_msgs.msg import JointTrajectoryPoint

        measured = self.measured()
        names = [n for n in (joints or self._joint_names) if n in measured]
        if not names:
            return

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = names
        point = JointTrajectoryPoint()
        point.positions = [measured[n] for n in names]
        point.time_from_start = Duration(
            sec=int(time), nanosec=int((time % 1.0) * 1e9))
        goal.trajectory.points = [point]

        # Created once and kept: making and destroying an ActionClient while a
        # background executor spins this node churns the wait set for no
        # reason.
        if self._traj_client is None:
            self._traj_client = ActionClient(
                self, FollowJointTrajectory,
                "/{}/follow_joint_trajectory".format(self._controller_name))
        if not self._traj_client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError(
                "no trajectory controller at /{}/follow_joint_trajectory -- "
                "cannot re-sync it to the robot, and enabling without that "
                "will be refused.".format(self._controller_name))
        send_goal_blocking(self._traj_client, goal,
                           "re-sync the controller to the measured pose")


def ensure_controller_active(node, controller, manager="/controller_manager",
                             timeout=10.0):
    """Activate ``controller`` if it is loaded but inactive.

    Returns the state it was in before. Safe while torque is off: the servicer
    never reads the shm command block until the supervisor enables a joint, so
    an active JTC with a limp robot commands nothing at all. It also does not
    jump on activation -- ``ActuoSystem`` seeded its command from the MEASURED
    pose, and JTC starts by holding that.
    """
    from controller_manager_msgs.srv import ListControllers, SwitchController

    lister = node.create_client(ListControllers, manager + "/list_controllers")
    if not lister.wait_for_service(timeout_sec=timeout):
        raise RuntimeError(
            "no {}/list_controllers -- is ros2_control running?".format(manager))
    future = lister.call_async(ListControllers.Request())
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    states = {c.name: c.state for c in future.result().controller}
    if controller not in states:
        raise RuntimeError(
            "controller {!r} is not loaded; loaded: {}".format(
                controller, ", ".join(sorted(states)) or "(none)"))
    if states[controller] == "active":
        return states[controller]

    switcher = node.create_client(SwitchController, manager + "/switch_controller")
    if not switcher.wait_for_service(timeout_sec=timeout):
        raise RuntimeError("no {}/switch_controller".format(manager))
    switch_controller(node, activate=[controller], manager=manager,
                      timeout=timeout)
    return states[controller]


def switch_controller(node, activate=(), deactivate=(),
                      manager="/controller_manager", timeout=10.0):
    """Activate/deactivate controllers, raising on refusal."""
    from controller_manager_msgs.srv import SwitchController

    switcher = node.create_client(SwitchController, manager + "/switch_controller")
    if not switcher.wait_for_service(timeout_sec=timeout):
        raise RuntimeError("no {}/switch_controller".format(manager))
    request = SwitchController.Request()
    request.activate_controllers = list(activate)
    request.deactivate_controllers = list(deactivate)
    request.strictness = SwitchController.Request.STRICT
    future = switcher.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    if future.result() is None or not future.result().ok:
        raise RuntimeError(
            "controller switch failed (activate={}, deactivate={}): {}".format(
                list(activate), list(deactivate),
                "" if future.result() is None else future.result().message))
