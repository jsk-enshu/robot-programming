"""Servo ON/OFF for the simulators, with the real robot's action and rules.

The real robot's torque switch is actuo's bus servicer
(``/actuo_bus_servicer/servo/set``, ``actuo_msgs/action/SetServo``). Gazebo
(the ``jedy_servo_switch`` system plugin) and Isaac Sim (jedy_isaac.py), both
behind ``jedy_sim_servo``, serve the same action under the same name, so
``ri.servo_off()`` / ``ri.servo_on()`` are the same calls on all three and a
released joint does what it does on the robot: it stops following the
controller and falls under gravity, and ``joint_states`` reports where it fell.

The rules are the real servicer's (actuo/drivers/servo.py), cut down to what a
simulator has:

* Turning a joint off is always allowed.
* Turning a position joint on is refused while its controller commands an
  angle more than ``jump_threshold`` away from where the joint is (the same
  0.05 rad as ``actuo_bus_node --max-start-jump``), because the joint would jump
  there. ``ri.servo_on()`` first sends the controller the pose the robot is in
  now, which is what makes it pass.
* Turning a wheel (velocity joint) on is refused while its controller asks
  for motion faster than ``velocity_threshold``.
* ``force`` skips both, as on the robot.
* A servo that does not answer (``missing``) is always limp, and turning it
  on is refused with the real servicer's words ("refusing: X not answering"),
  ``force`` or not. It exists to rehearse a robot with a cable out:
  ``ros2 launch jedy_bringup jedy_gazebo.launch.py missing_servos:=rarm_joint3``.

What a simulator does NOT have, and the real servicer does: bus faults,
temperatures, the command filter. Those fields are left at their "nothing
known" values, not made up.

:class:`SimServoSwitch` is the rule, without ROS, so it is unit-tested;
:class:`SimServoServer` is the action around it.
"""

import math
import threading

#: Same name as the real robot's bus servicer, so jedy_skrobot finds it.
SERVO_ACTION = '/actuo_bus_servicer/servo/set'

#: [rad] actuo_bus_node's --max-start-jump default.
JUMP_THRESHOLD = 0.05

#: [rad/s] actuo.drivers.servo.ServoSupervisor's velocity_threshold default.
VELOCITY_THRESHOLD = 0.05

#: ``absence_reason`` of a servo made missing on purpose, so nobody mistakes
#: the rehearsal for a real fault.
MISSING_REASON = 'simulated: missing_servos (not answering)'


#: mecanum_drive_controller's wheel order, by the yaml key naming each one.
WHEEL_KEYS = ['front_left_wheel_command_joint_name',
              'front_right_wheel_command_joint_name',
              'rear_left_wheel_command_joint_name',
              'rear_right_wheel_command_joint_name']

#: The trajectory controllers, one per limb (as jedy_skrobot.CONTROLLERS).
TRAJECTORY_CONTROLLERS = ['head_controller', 'right_arm_controller',
                          'left_arm_controller', 'right_gripper_controller',
                          'left_gripper_controller']


def load_controllers(path, namespace='/jedy_fullbody'):
    """The joints of jedy_fullbody_controllers.yaml.

    Parameters
    ----------
    path : str
    namespace : str

    Returns
    -------
    controller_joints : dict
        ``{trajectory controller: [joint, ...]}``.
    wheels : list of str
        FL, FR, RL, RR.
    radius : float
        [m]
    k : float
        ``sum_of_robot_center_projection_on_X_Y_axis`` [m].
    """
    import yaml
    with open(path) as f:
        doc = yaml.safe_load(f)[namespace]
    controller_joints = {c: list(doc[c]['ros__parameters']['joints'])
                         for c in TRAJECTORY_CONTROLLERS}
    params = doc['mecanum_drive_controller']['ros__parameters']
    kin = params['kinematics']
    return (controller_joints, [params[k] for k in WHEEL_KEYS],
            float(kin['wheels_radius']),
            float(kin['sum_of_robot_center_projection_on_X_Y_axis']))


def wheel_speeds(vx, vy, wz, radius, k):
    """mecanum_drive_controller's inverse kinematics.

    Returns
    -------
    list of float
        [rad/s] FL, FR, RL, RR.
    """
    return [(vx - vy - k * wz) / radius, (vx + vy + k * wz) / radius,
            (vx + vy - k * wz) / radius, (vx - vy + k * wz) / radius]


class SimServoSwitch:
    """Which joints are released, and whether a request may change that.

    Every joint starts driven: a simulator spawns the robot holding its pose
    (on the real robot the servos start off, and the first ``servo_on`` is
    what makes it hold).

    Parameters
    ----------
    position_joints : sequence of str
    velocity_joints : sequence of str
        The wheels.
    jump_threshold : float
        [rad]
    velocity_threshold : float
        [rad/s]
    missing : sequence of str
        Joints whose servo does not answer: always limp, never enabled.
    """

    def __init__(self, position_joints, velocity_joints=(),
                 jump_threshold=JUMP_THRESHOLD,
                 velocity_threshold=VELOCITY_THRESHOLD, missing=()):
        self.position_joints = list(position_joints)
        self.velocity_joints = list(velocity_joints)
        self.joint_names = self.position_joints + self.velocity_joints
        self.jump_threshold = float(jump_threshold)
        self.velocity_threshold = float(velocity_threshold)
        unknown = [j for j in missing if j not in self.joint_names]
        if unknown:
            raise ValueError('missing servos {} are not joints of this robot '
                             '(it has {})'.format(unknown, self.joint_names))
        self.missing = [n for n in self.joint_names if n in set(missing)]
        self._released = set(self.missing)
        self._lock = threading.Lock()

    def is_present(self, name):
        """Whether the servo of ``name`` answers.

        Returns
        -------
        bool
        """
        return name not in self.missing

    @property
    def released(self):
        """The released joints, in joint order.

        Returns
        -------
        list of str
        """
        with self._lock:
            return [n for n in self.joint_names if n in self._released]

    def is_enabled(self, name):
        """Whether ``name`` is driven.

        Returns
        -------
        bool
        """
        with self._lock:
            return name not in self._released

    def _resolve(self, joints):
        if not joints:
            return list(self.joint_names)
        unknown = [j for j in joints if j not in self.joint_names]
        if unknown:
            raise KeyError('unknown joints {} (this robot has {})'.format(
                unknown, self.joint_names))
        return list(joints)

    def disable(self, joints=None):
        """Release ``joints`` (default: all). Always allowed.

        Parameters
        ----------
        joints : sequence of str or None

        Returns
        -------
        ok : bool
            False only for an unknown joint name.
        reason : str
        """
        try:
            target = self._resolve(joints)
        except KeyError as err:
            return False, 'refusing: {}'.format(err.args[0])
        with self._lock:
            went_limp = [n for n in target if n not in self._released]
            self._released.update(target)
            holding = [n for n in self.joint_names if n not in self._released]
        if not went_limp:
            return True, 'already off: {}'.format(', '.join(target))
        return True, (
            'torque removed from {}; {} limp and will fall under gravity{}'
            .format(', '.join(went_limp),
                    'they are' if len(went_limp) > 1 else 'it is',
                    ' (still holding: {})'.format(', '.join(holding))
                    if holding else ''))

    def would_jump(self, name, measured, commanded):
        """Whether driving ``name`` now would move it by itself.

        Parameters
        ----------
        name : str
        measured : float or None
            [rad]; ignored for a wheel.
        commanded : float or None
            [rad] for a position joint, [rad/s] for a wheel.

        Returns
        -------
        bool
            True also when a value needed to judge is missing or NaN: a
            joint that cannot be checked is not called safe.
        """
        if commanded is None or math.isnan(commanded):
            return name not in self.velocity_joints
        if name in self.velocity_joints:
            return abs(commanded) > self.velocity_threshold
        if measured is None or math.isnan(measured):
            return True
        return abs(commanded - measured) > self.jump_threshold

    def enable(self, measured, commanded, joints=None, force=False):
        """Drive ``joints`` (default: all) again, unless that would jump.

        Parameters
        ----------
        measured : dict
            ``{joint: rad}`` where the joints are.
        commanded : dict
            ``{joint: rad}`` (position) or ``{joint: rad/s}`` (wheel) the
            controllers command. A wheel with no command means no motion.
        joints : sequence of str or None
        force : bool
            Skip the check.

        Returns
        -------
        ok : bool
        reason : str
        """
        try:
            target = self._resolve(joints)
        except KeyError as err:
            return False, 'refusing: {}'.format(err.args[0])
        with self._lock:
            pending = [n for n in target if n in self._released]
        if not pending:
            return True, 'already on: {}'.format(', '.join(target))
        gone = [n for n in pending if n in self.missing]
        if gone:
            return False, 'refusing: {} not answering'.format(', '.join(gone))
        jumpy = [n for n in pending
                 if self.would_jump(n, measured.get(n), commanded.get(n))]
        if jumpy and not force:
            far = [n for n in jumpy if n not in self.velocity_joints]
            moving = [n for n in jumpy if n in self.velocity_joints]
            parts = []
            if far:
                parts.append(
                    'enabling would throw the robot: {}. Threshold is {:.4f} '
                    'rad. Send the controller the pose the robot is in first '
                    '(ri.servo_on() does).'.format(
                        '; '.join('{} is at {:.4f} but the controller '
                                  'commands {:.4f}'.format(
                                      n, measured.get(n, math.nan),
                                      commanded.get(n, math.nan))
                                  for n in far),
                        self.jump_threshold))
            if moving:
                parts.append(
                    'the controller is asking for motion: {}. Threshold is '
                    '{:.4f} rad/s. Stop the base first.'.format(
                        '; '.join('{} is commanded {:+.4f} rad/s'.format(
                            n, commanded[n]) for n in moving),
                        self.velocity_threshold))
            return False, 'refusing: ' + ' AND '.join(parts)
        with self._lock:
            self._released.difference_update(pending)
        return True, 'torque on for {}, holding where {} {}'.format(
            ', '.join(pending), 'they are' if len(pending) > 1 else 'it is',
            '(FORCED past the jump check)' if jumpy else
            '(within {:.4f} rad)'.format(self.jump_threshold))


class SimServoServer:
    """``SetServo`` at :data:`SERVO_ACTION` for one :class:`SimServoSwitch`.

    Parameters
    ----------
    node : rclpy.node.Node
    switch : SimServoSwitch
    snapshot : callable
        ``snapshot() -> (measured, commanded)``, two dicts as
        :meth:`SimServoSwitch.enable` takes.
    apply : callable
        ``apply(released)`` makes the simulator release exactly the joints
        in the list ``released`` and drive the rest.
    applied : callable or None
        ``applied() -> list of str or None``: the released joints as the
        simulator reports it applies them, ``None`` when it has not said yet.
        When given, the action answers only after the simulator agrees, so a
        returned ``servo_off()`` means the joints are limp, not that it was
        asked.
    confirm_timeout : float
        Seconds (wall clock) to wait for that agreement.
    callback_group : rclpy.callback_groups.CallbackGroup or None

    Notes
    -----
    The execute callback is a coroutine that waits for the simulator's
    agreement on a steady-clock timer instead of sleeping in a thread, so it
    works on a single-threaded executor -- where the topic that brings the
    agreement is handled by the same thread, as on the real servo switch.
    Sleeping there instead needed a multi-threaded executor and held one of
    its threads for up to ``confirm_timeout``.
    """

    def __init__(self, node, switch, snapshot, apply, applied=None,
                 confirm_timeout=5.0, callback_group=None):
        from actuo_msgs.action import SetServo
        from rclpy.action import ActionServer, CancelResponse, GoalResponse
        self._node = node
        self._switch = switch
        self._snapshot = snapshot
        self._apply = apply
        self._applied = applied
        self._confirm_timeout = confirm_timeout
        self._group = callback_group
        self._server = ActionServer(
            node, SetServo, SERVO_ACTION,
            execute_callback=self._execute,
            goal_callback=lambda _goal: GoalResponse.ACCEPT,
            cancel_callback=lambda _goal: CancelResponse.REJECT,
            callback_group=callback_group)
        node.get_logger().info(
            'servo switch at {} for {} joints (all driven at start)'.format(
                SERVO_ACTION, len(switch.joint_names)))

    async def _confirm(self):
        """Wait until the simulator reports the wanted set; (ok, why)."""
        import time
        from rclpy.clock import Clock, ClockType
        from rclpy.task import Future
        want = set(self._switch.released)
        if self._applied is None:
            return True, ''
        deadline = time.monotonic() + self._confirm_timeout
        done = Future()
        last = {'got': None}

        def check():
            if done.done():
                return
            got = self._applied()
            last['got'] = got
            if got is not None and set(got) == want:
                done.set_result(True)
            elif time.monotonic() >= deadline:
                done.set_result(False)

        # Steady clock: under use_sim_time a node timer follows /clock, and a
        # simulator that stalls would stall the deadline with it.
        timer = self._node.create_timer(
            0.02, check, callback_group=self._group,
            clock=Clock(clock_type=ClockType.STEADY_TIME))
        try:
            check()
            confirmed = await done
        finally:
            self._node.destroy_timer(timer)
        if confirmed:
            return True, ''
        got = last['got']
        return False, (
            'the simulator did not confirm within {} s: asked to release {}, '
            'it reports {}'.format(self._confirm_timeout, sorted(want),
                                   'nothing' if got is None else sorted(got)))

    def joint_messages(self, measured, commanded):
        from actuo_msgs.msg import JointServoState
        out = []
        for name in self._switch.joint_names:
            msg = JointServoState()
            msg.name = name
            wheel = name in self._switch.velocity_joints
            msg.mode = 'velocity' if wheel else 'position'
            msg.enabled = self._switch.is_enabled(name)
            msg.present = self._switch.is_present(name)
            if not msg.present:
                msg.absence_reason = MISSING_REASON
            msg.measured = float(measured.get(name, math.nan))
            msg.commanded = float(commanded.get(name, math.nan))
            msg.mismatch = (msg.commanded if wheel
                            else msg.commanded - msg.measured)
            msg.would_jump = (not msg.enabled and self._switch.would_jump(
                name, measured.get(name), commanded.get(name)))
            msg.temperature = math.nan
            msg.velocity = math.nan
            msg.current_a = math.nan
            msg.effort = math.nan
            out.append(msg)
        return out

    async def _execute(self, goal_handle):
        from actuo_msgs.action import SetServo
        import time
        request = goal_handle.request
        started = time.monotonic()
        self._node.get_logger().info('servo request: {} {}'.format(
            'ON' if request.enable else 'OFF',
            ', '.join(request.joints) or 'every joint'))
        measured, commanded = self._snapshot()
        before = set(self._switch.released)
        if request.enable:
            ok, reason = self._switch.enable(
                measured, commanded, list(request.joints), request.force)
        else:
            ok, reason = self._switch.disable(list(request.joints))
        if ok and set(self._switch.released) == before:
            # Nothing to change ("already on/off"): nothing to wait for.
            pass
        elif ok:
            self._apply(self._switch.released)
            confirmed, why = await self._confirm()
            if not confirmed:
                ok, reason = False, why
            self._node.get_logger().info(
                'servo: simulator {} after {:.2f} s'.format(
                    'confirmed' if confirmed else 'did NOT confirm',
                    time.monotonic() - started))
        result = SetServo.Result()
        result.ok = ok
        result.reason = reason
        result.joints = self.joint_messages(*self._snapshot())
        if ok:
            self._node.get_logger().info('servo: {}'.format(reason))
            goal_handle.succeed()
        else:
            self._node.get_logger().warning(
                'servo REFUSED: {}'.format(reason))
            goal_handle.abort()
        return result
