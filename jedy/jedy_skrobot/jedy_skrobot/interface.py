"""The Jedy robot interface (``ri``) shared by Gazebo, the real robot and Isaac Sim.

Joints, ``wait_interpolation`` and the torque switch are actuo_ros'
:class:`~actuo_ros.robot_interface.RobotInterface`, unchanged. This module adds the
parts of Jedy that are not joints, each one a plain topic so that every backend
can provide it the same way:

=====================  ==============================  ==========================
method                 topic                           type
=====================  ==============================  ==========================
``go_velocity``        ``/cmd_vel``                    geometry_msgs/TwistStamped
``camera_info``        ``/camera/color/camera_info``   sensor_msgs/CameraInfo
``imu``                ``/imu``                        sensor_msgs/Imu
``image``              ``/camera/color/image_rect_raw``  sensor_msgs/Image
``depth_image``        ``/camera/depth/image_rect_raw``  sensor_msgs/Image
=====================  ==============================  ==========================

Recognition (``what_do_you_see``, ``sees``, ``gesture``, ``janken`` and their
``wait_until_*``) calls jedy_vision's recognition nodes, started with
``ros2 launch jedy_vision recognition.launch.py``:

==========================  ======================================  =========================================
method                      service                                 type
==========================  ======================================  =========================================
``what_do_you_see``         ``/object_detection/what_do_you_see``   jedy_vision_msgs/WhatDoYouSee
``gesture`` / ``janken``    ``/gesture/read_gesture``               jedy_vision_msgs/ReadGesture
==========================  ======================================  =========================================

The models (torch, YOLOE, MediaPipe) live in those nodes, not in the program
that uses ``ri``.

Speech (``say``, ``listen``) uses jsk_3rdparty's VOICEVOX and
ros_speech_recognition, each started with its own launch:

==========  ===============================  ==================================================
method      action / topic                   type
==========  ===============================  ==================================================
``say``     ``/robotsound_jp`` (action)      sound_play_msgs/SoundRequest
``listen``  ``/speech_to_text``              speech_recognition_msgs/SpeechRecognitionCandidates
==========  ===============================  ==================================================

The board itself stopping (it happens under heavy arm load, and when its
power is cut) is read from the servo switch's ``/diagnostics`` status
``actuo_bus_servicer: bus/link``: while it is ERROR, or has not been published
for :data:`BOARD_DIAG_TIMEOUT` (the switch is busy reconnecting), ``ri`` says
so once in the terminal, and ``angle_vector``, ``servo_on`` and driving raise
:class:`BoardNotAnswering` with what to do. Without this a program kept
reading the last angles from before the stop and saw nothing wrong.

Servos that do not answer (a cable out, a board unpowered) are read from the
servo switch's ``/actuo_bus_servicer/servo/state``: :meth:`JedyInterface.missing_servos`
lists them, :func:`connect` warns about them at once, ``servo_on`` refuses
them with the joint names and the bus's reason, and commanding them logs that
they will not move. RViz shows the same joints blinking red.

A sensor that nobody publishes is an error, not a zero: on the RCB-4mini robot
there is no IMU unless a sensor board was fitted, and a reaction behaviour
written against a stream of zeros would look like it works and do nothing.
"""

import copy
import math
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
from skrobot.models.jedy import Jedy

from actuo_ros.robot_interface import connect as _actuo_connect
from actuo_ros.robot_interface import RobotInterface
from actuo_ros.robot_interface import ServoRefused
from diagnostic_msgs.msg import DiagnosticArray

#: The namespace every Jedy bring-up (Gazebo, real, Isaac) puts its
#: controller manager, joint_states and robot_description under.
NAMESPACE = '/jedy_fullbody'

#: The servo switch: actuo's bus servicer on the real robot, jedy_sim_servo in
#: Gazebo and Isaac Sim serve the same action here.
SERVO_NS = '/actuo_bus_servicer'

CMD_VEL_TOPIC = '/cmd_vel'
IMU_TOPIC = '/imu'
#: The head camera, under the RealSense driver's names so the real robot's
#: camera node and the simulators' bridges are interchangeable.
IMAGE_TOPIC = '/camera/color/image_rect_raw'
#: The colour image's frame (a link of the robot model), for detections
#: whose message did not say.
IMAGE_FRAME = 'camera_color_optical_frame'
DEPTH_TOPIC = '/camera/depth/image_rect_raw'
CAMERA_INFO_TOPIC = '/camera/color/camera_info'

#: The trajectory controllers every Jedy bring-up runs, one per limb.
CONTROLLERS = ['head_controller', 'right_arm_controller', 'left_arm_controller',
               'right_gripper_controller', 'left_gripper_controller']

#: Seconds to wait for them to become active. Gazebo spawns them tens of
#: seconds after the launch starts.
CONTROLLER_TIMEOUT = 120.0

#: The mecanum wheels. Not in any trajectory controller, so actuo_ros'
#: servo_on() does not know them; see JedyInterface.servo_on.
#: The driving pose [deg]: shoulders drawn back 45 deg (joint0) and opened
#: 30 deg sideways (joint1), elbows bent as in the reset pose (joint3 -120),
#: so the forearms slope down to the front, and the wrists (joint5 -15) bring
#: the fingers about level. The arms stay close to the body: the hands are
#: 0.17 m ahead of base_link and the robot's centre of mass 17 mm ahead of
#: it, 27 mm behind the front axle (arms straight ahead: 25-27 mm). In
#: Gazebo from the reset pose every joint reaches it within 0.1 deg,
#: arm_floor_clearance is 0.111 m, and the hands are out of the head
#: camera's view. Mirrored for the left arm by _MIRRORED.
#:
#: It used to fold the arms up behind the shoulders (joint0 130 deg): the
#: hands ended 0.22 m behind base_link, and moving between that and the reset
#: pose swings the hands through the floor (in Gazebo they stuck there).
TUCK_POSE = {'joint0': 45.0, 'joint1': -30.0, 'joint2': 0.0, 'joint3': -120.0,
             'joint4': 0.0, 'joint5': -15.0, 'joint6': 0.0}
#: Arm joints whose left value is the right one negated for a mirror-image
#: pose. Computed from the model (same value on both sides vs negated, hand
#: pose mirrored across the x-z plane): joints 0, 1, 2, 4 and 6 mirror when
#: negated, 3 and 5 when equal. Leaving joint2 out tucks the left upper arm
#: twisted the opposite way from the right.
_MIRRORED = ('joint0', 'joint1', 'joint2', 'joint4', 'joint6')
#: The floor in base_link [m]: the bottom of the wheels (jedy_gz.xacro's
#: collision cylinders, from the CAD; the CAD model's base_footprint).
FLOOR_Z = -0.11625
#: Driving warns when an arm link is closer to the floor than this [m].
ARM_FLOOR_WARN = 0.05

WHEEL_JOINTS = ['front_right_wheel_joint', 'front_left_wheel_joint',
                'rear_right_wheel_joint', 'rear_left_wheel_joint']

#: Seconds to look for an optional topic (camera, IMU) before deciding the
#: running robot has none.
PROBE_TIMEOUT = 3.0

#: Seconds to wait for the servo action server. Every backend has one, and
#: discovering it took 0.1-3.5 s on a loaded machine (measured): the old 3 s
#: probe sometimes missed it and then reported "no servo switch".
SERVO_PROBE_TIMEOUT = 10.0
#: Seconds connect() keeps asking the controller manager whether time is
#: simulated, and how long it waits for each answer before asking again.
PARAMETER_TIMEOUT = 30.0
PARAMETER_RETRY = 2.0

#: Seconds to wait for the first ``servo/state``. It is transient_local, so a
#: running switch hands it over as soon as the subscription matches.
SERVO_STATE_TIMEOUT = 5.0

#: Seconds between repeats of the same "this joint will not move" message.
MISSING_WARN_PERIOD = 5.0

#: The servo switch's diagnostics status that says whether the board answers
#: ("Board answering." / "Board did not answer ...; check power and cable.").
BOARD_DIAG_NAME = 'actuo_bus_servicer: bus/link'
#: Seconds without that status, after it has been seen, that count as the
#: board having stopped: the switch publishes it about once a second, and not
#: at all while it is reconnecting to a silent board.
BOARD_DIAG_TIMEOUT = 3.0
#: DiagnosticStatus.ERROR.
_DIAG_ERROR = 2

#: jedy_vision's recognition nodes (recognition.launch.py).
DETECTION_SERVICE = '/object_detection/what_do_you_see'
GESTURE_SERVICE = '/gesture/read_gesture'
#: Seconds to look for a recognition node before saying it is not running.
RECOGNITION_PROBE_TIMEOUT = 3.0
#: Seconds between looks in ``wait_until_sees`` / ``wait_until_gesture``.
RECOGNITION_POLL = 0.2

#: VOICEVOX's sound_play (voicevox_texttospeech.launch.xml). Speaking through
#: this action, not the ``/robotsound_jp/text`` topic, is what lets the speech
#: recogniser (tts_action_names:="[robotsound_jp]") ignore the robot's voice.
SAY_ACTION = '/robotsound_jp'
SPEECH_TOPIC = '/speech_to_text'
#: Seconds to look for the speech nodes before saying they are not running.
SPEECH_PROBE_TIMEOUT = 3.0


class BoardNotAnswering(RuntimeError):
    """The robot's board stopped answering; the message says what to do."""


BOARD_STOPPED_MESSAGE = (
    'ロボットのボード (RCB-4mini) が応答していません ({reason})。'
    'サーボは安全のためすべて OFF になりました。'
    'ロボットの電源を入れ直してください (分からなければ TA を呼ぶ)。'
    'ボードが戻っても腕と首のサーボは OFF のままなので、'
    'ri.servo_on() で ON にしてから続けてください。')
BOARD_BACK_MESSAGE = (
    'ボードが応答を再開しました。腕と首のサーボは OFF のままです。'
    'ri.servo_on() で ON にしてから続けてください。')


class JedyInterface(RobotInterface):
    """``ri`` for Jedy: the arms, head and grippers, plus base and sensors.

    Build it with :func:`connect`, which finds the controllers and decides
    simulated time and the servo switch from what is running.

    Parameters
    ----------
    robot : skrobot.models.Jedy
    controller_joints : dict
        ``{controller name: [joint name, ...]}``.
    use_sim_time : bool
        Follow ``/clock``. Gazebo and Isaac publish it; the real robot does
        not. ``joint_states`` are stamped in that clock, so waiting for a fresh
        state against the wrong one never ends.
    **kwargs
        Passed to :class:`~actuo_ros.robot_interface.RobotInterface`.
    """

    def __init__(self, robot, controller_joints, use_sim_time=False,
                 **kwargs):
        kwargs.setdefault('node_name', 'jedy_skrobot')
        super(JedyInterface, self).__init__(robot, controller_joints, **kwargs)
        if use_sim_time:
            self.set_parameters([Parameter('use_sim_time', value=True)])
        self.use_sim_time = use_sim_time
        self._lock = threading.Lock()
        self._imu_msg = None
        self._imu_seen = threading.Event()
        self._cmd_vel_pub = self.create_publisher(
            TwistStamped, CMD_VEL_TOPIC, 1)
        self._clearance_model = None
        self._arm_check_time = 0.0
        self._arm_warn_time = 0.0
        self.create_subscription(Imu, IMU_TOPIC, self._on_imu,
                                 qos_profile_sensor_data)
        # Subscribed on first use (image()/depth_image()), not here: two
        # 640x480 streams deserialised on this node's one executor thread
        # starved its joint_states, and a program that never looked at the
        # camera got "Not updated joints" and a servo_on that timed out.
        self._image_msgs = {IMAGE_TOPIC: None, DEPTH_TOPIC: None}
        self._image_seen = {IMAGE_TOPIC: threading.Event(),
                            DEPTH_TOPIC: threading.Event()}
        self._image_subs = {}
        self._camera_info = None
        self._camera_info_seen = threading.Event()
        self._camera_info_sub = None
        self._servo_state = None
        self._servo_state_seen = threading.Event()
        self._missing_warn_time = {}
        self._board_status = None      # (monotonic time, level, message)
        self._board_reported = None    # the reason last reported, or None
        if self._servo_client is not None:
            self.create_subscription(DiagnosticArray, '/diagnostics',
                                     self._on_diagnostics, 10)
            self.create_timer(1.0, self._watch_board)
        self._recognition_clients = {}
        self._say_client = None
        self._speech_sub = None
        self._speech_text = None
        self._speech_heard = threading.Event()
        if self._servo_client is not None:
            from actuo_msgs.msg import ServoState
            self.create_subscription(
                ServoState, SERVO_NS + '/servo/state', self._on_servo_state,
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    # -- time -----------------------------------------------------------------

    def now(self):
        """Seconds on the robot's clock.

        The simulator's ``/clock`` when there is one, the wall clock on the
        real robot. Gazebo often runs slower than real time, so a loop timed
        with ``time.monotonic()`` stops early in simulated seconds and drives
        a shorter distance than the same loop on the real robot.

        Returns
        -------
        float

        Raises
        ------
        RuntimeError
            If on sim time and no ``/clock`` arrives within
            :data:`PROBE_TIMEOUT`.
        """
        stamp = self.get_clock().now().nanoseconds
        if stamp == 0 and self.use_sim_time:
            # Sim time reads 0 until the first /clock message. Taking that 0
            # as a start time made a 20 s timeout end after one cycle (the
            # next reading was 77.9 s).
            deadline = time.monotonic() + PROBE_TIMEOUT
            while stamp == 0:
                if time.monotonic() > deadline:
                    raise RuntimeError(
                        'on sim time but no /clock within {} s'.format(
                            PROBE_TIMEOUT))
                time.sleep(0.01)
                stamp = self.get_clock().now().nanoseconds
        return stamp * 1e-9

    def sleep(self, seconds):
        """Wait ``seconds`` on the robot's clock (see :meth:`now`).

        Parameters
        ----------
        seconds : float
        """
        end = self.now() + seconds
        # Short wall-clock naps: /clock arrives every 20 ms in Gazebo,
        # and on the real robot this is just a sleep cut into pieces.
        while self.now() < end:
            time.sleep(min(0.01, max(0.0, end - self.now())))

    # -- what this robot has --------------------------------------------------

    def _has_publisher(self, topic, timeout):
        deadline = time.monotonic() + timeout
        while True:
            if self.count_publishers(topic) > 0:
                return True
            if time.monotonic() >= deadline:
                return False
            time.sleep(0.1)

    def has_imu(self, timeout=PROBE_TIMEOUT):
        """Whether anything publishes the IMU (``/imu``).

        Gazebo and Isaac do; the RCB-4mini board does not.

        Parameters
        ----------
        timeout : float
            Seconds to wait for discovery.

        Returns
        -------
        bool
        """
        return self._has_publisher(IMU_TOPIC, timeout)

    @property
    def can_servo_off(self):
        """Whether the torque can be released.

        True on the real robot, Gazebo and Isaac Sim, which all serve
        ``/actuo_bus_servicer/servo/set``; False only for a bring-up without
        a servo switch (e.g. mock hardware).
        """
        return self._servo_client is not None

    # -- base -----------------------------------------------------------------

    # -- the board ------------------------------------------------------------

    def _on_diagnostics(self, msg):
        for status in msg.status:
            if status.name == BOARD_DIAG_NAME:
                level = status.level
                if isinstance(level, (bytes, bytearray)):
                    level = level[0]
                with self._lock:
                    self._board_status = (time.monotonic(), int(level), status.message)

    def board_problem(self):
        """Why the board is not answering, or None when it is (or is unknown).

        Returns
        -------
        str or None
            None while the servo switch reports the board answering, and on
            a robot whose switch publishes no such status (the simulators).
        """
        with self._lock:
            status = self._board_status
        if status is None:
            return None
        stamp, level, message = status
        age = time.monotonic() - stamp
        if age > BOARD_DIAG_TIMEOUT:
            return 'サーボの制御プロセスから {:.0f} 秒報告がない'.format(age)
        if level >= _DIAG_ERROR:
            return message
        return None

    def _watch_board(self):
        """Say once when the board stops, and once when it is back."""
        problem = self.board_problem()
        if problem is not None and self._board_reported is None:
            self._board_reported = problem
            self.get_logger().error(BOARD_STOPPED_MESSAGE.format(reason=problem))
        elif problem is None and self._board_reported is not None:
            self._board_reported = None
            self.get_logger().warning(BOARD_BACK_MESSAGE)

    def _check_board(self):
        """Raise BoardNotAnswering while the board is not answering."""
        problem = self.board_problem()
        if problem is not None:
            raise BoardNotAnswering(BOARD_STOPPED_MESSAGE.format(reason=problem))

    # -- base -----------------------------------------------------------------

    def send_cmd_vel(self, vx, vy, vtheta):
        """Send one velocity command to the base.

        The base controller stops by itself if no command arrives within
        0.5 s (``reference_timeout``), so a loop has to keep sending.

        Parameters
        ----------
        vx, vy : float
            Forward and leftward speed [m/s].
        vtheta : float
            Turning speed, counter-clockwise from above [rad/s].
        """
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = float(vx)
        msg.twist.linear.y = float(vy)
        msg.twist.angular.z = float(vtheta)
        self._cmd_vel_pub.publish(msg)
        if vx or vy or vtheta:
            # After publishing: a stop (all zero) always goes out, and a
            # drive command reaches the controller before the error.
            self._check_board()
            self._check_arms_before_driving()
            self._warn_missing(WHEEL_JOINTS, 'go_velocity')

    def tuck_arms(self, time=2.0):
        """Fold both arms into the driving pose (:data:`TUCK_POSE`) and wait.

        Call this before driving: with the arms hanging (Gazebo's start pose)
        the hands are millimetres above the floor and drag on it.
        """
        robot = self.robot
        arm_joints = [j.name for j in robot.rarm.joint_list + robot.larm.joint_list]
        if self._servo_client is not None:
            # The real robot starts with the servos off; torque on the arms
            # only (holding where they are), not the head a sensor demo
            # may have released.
            self.servo_on(joints=arm_joints)
        robot.angle_vector(self.angle_vector())
        for side, sign in (('rarm', 1.0), ('larm', -1.0)):
            for name, deg in TUCK_POSE.items():
                if name in _MIRRORED:
                    deg = sign * deg
                getattr(robot, '{}_{}'.format(side, name)).joint_angle(math.radians(deg))
        # Only the two arm controllers: the head (maybe released by
        # servo_off for a sensor demo) and the grippers are left alone.
        for controller in ('right_arm_controller', 'left_arm_controller'):
            self.angle_vector(robot.angle_vector(), time, controller_type=controller)
        for controller in ('right_arm_controller', 'left_arm_controller'):
            self.wait_interpolation(controller)
        robot.angle_vector(self.angle_vector())

    def arm_floor_clearance(self):
        """Height of the lowest arm link above the floor now [m] (measured)."""
        if self._clearance_model is None:
            # A private copy: checking must not move the user's model.
            self._clearance_model = copy.deepcopy(self.robot)
        model = self._clearance_model
        model.angle_vector(self.angle_vector())
        return min(link.worldpos()[2] for link in model.link_list
                   if link.name.startswith(('rarm', 'larm'))) - FLOOR_Z

    def _check_arms_before_driving(self):
        now = time.monotonic()
        if now - self._arm_check_time < 1.0:
            return
        self._arm_check_time = now
        try:
            clearance = self.arm_floor_clearance()
        except Exception as exc:  # do not stop the base, but say so
            self.get_logger().warn('arm clearance check failed: {}'.format(exc))
            return
        if clearance < ARM_FLOOR_WARN and now - self._arm_warn_time > 5.0:
            self._arm_warn_time = now
            self.get_logger().warn(
                'driving with an arm {:.0f} mm above the floor: it drags. '
                'Call ri.tuck_arms() first (腕を上げてから走ろう)'.format(1000 * clearance))

    def go_velocity(self, vx, vy, vtheta, duration=None, rate=10.0):
        """Drive the base.

        Parameters
        ----------
        vx, vy : float
            Forward and leftward speed [m/s]. The real base (RCB-4mini) does
            not turn its wheels below about 0.078 m/s and is capped at
            about 0.156 m/s.
        vtheta : float
            Turning speed [rad/s]. On the real base about 0.83 to 1.66 rad/s
            when turning in place.
        duration : float or None
            Seconds (robot time) to keep driving, then stop. ``None``: send
            once and return, for a loop that calls this every cycle.
        rate : float
            Hz at which the command is repeated while ``duration`` lasts.
        """
        if duration is None:
            self.send_cmd_vel(vx, vy, vtheta)
            return
        end = self.now() + duration
        while self.now() < end:
            self.send_cmd_vel(vx, vy, vtheta)
            self.sleep(min(1.0 / rate, end - self.now()))
        self.stop()

    def stop(self):
        """Stop the base."""
        self.send_cmd_vel(0.0, 0.0, 0.0)

    # -- IMU ------------------------------------------------------------------

    def _on_imu(self, msg):
        with self._lock:
            self._imu_msg = msg
        self._imu_seen.set()

    def imu(self, timeout=1.0):
        """The latest IMU reading.

        Parameters
        ----------
        timeout : float
            Seconds to wait for the first reading.

        Returns
        -------
        sensor_msgs.msg.Imu
        """
        if not self._imu_seen.wait(timeout):
            raise RuntimeError(
                'no IMU reading on {} within {} s (publishers: {}). The '
                'RCB-4mini board has no IMU of its own; in Gazebo/Isaac the '
                'bridge provides it.'.format(
                    IMU_TOPIC, timeout, self.count_publishers(IMU_TOPIC)))
        with self._lock:
            return self._imu_msg

    def roll_pitch_yaw(self, timeout=1.0):
        """The robot's tilt from the IMU, [roll, pitch, yaw] in radians.

        Uses the IMU's orientation when it has one. When it does not (it says
        so with ``orientation_covariance[0] == -1``), roll and pitch come from
        the direction of gravity in the accelerometer, and yaw, which gravity
        cannot tell, is NaN.

        Returns
        -------
        numpy.ndarray
            ``[roll, pitch, yaw]`` [rad].
        """
        msg = self.imu(timeout)
        if msg.orientation_covariance[0] != -1.0:
            q = msg.orientation
            roll = math.atan2(2.0 * (q.w * q.x + q.y * q.z),
                              1.0 - 2.0 * (q.x * q.x + q.y * q.y))
            sin_pitch = max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x)))
            pitch = math.asin(sin_pitch)
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            return np.array([roll, pitch, yaw])
        a = msg.linear_acceleration
        roll = math.atan2(a.y, a.z)
        pitch = math.atan2(-a.x, math.sqrt(a.y * a.y + a.z * a.z))
        return np.array([roll, pitch, np.nan])

    # -- camera ---------------------------------------------------------------

    def _on_image(self, topic, msg):
        with self._lock:
            self._image_msgs[topic] = msg
        self._image_seen[topic].set()

    def _latest_image(self, topic, timeout, newer_than):
        with self._lock:
            if topic not in self._image_subs:
                self._image_subs[topic] = self.create_subscription(
                    Image, topic,
                    lambda msg, topic=topic: self._on_image(topic, msg),
                    qos_profile_sensor_data)
        deadline = time.monotonic() + timeout
        while True:
            if self._image_seen[topic].wait(
                    max(0.0, deadline - time.monotonic())):
                with self._lock:
                    msg = self._image_msgs[topic]
                stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                if newer_than is None or stamp > newer_than:
                    return msg, stamp
            if time.monotonic() >= deadline:
                raise RuntimeError(
                    'no {}image on {} within {} s (publishers: {}). In '
                    'Gazebo start with render_sensors:=true; on the real '
                    'robot start the camera node.'.format(
                        '' if newer_than is None else 'new ', topic, timeout,
                        self.count_publishers(topic)))
            time.sleep(0.005)

    def _on_camera_info(self, msg):
        with self._lock:
            self._camera_info = msg
        self._camera_info_seen.set()

    def camera_info(self, timeout=5.0):
        """The colour camera's intrinsics, from ``camera_info``.

        Parameters
        ----------
        timeout : float
            Seconds (wall clock) to wait for the first message.

        Returns
        -------
        K : numpy.ndarray
            ``(3, 3)`` camera matrix [px].
        distortion : numpy.ndarray
            The distortion coefficients ``D`` as published (may be empty: no
            distortion).
        """
        with self._lock:
            if self._camera_info_sub is None:
                self._camera_info_sub = self.create_subscription(
                    CameraInfo, CAMERA_INFO_TOPIC, self._on_camera_info,
                    qos_profile_sensor_data)
        if not self._camera_info_seen.wait(timeout):
            raise RuntimeError(
                'no camera_info on {} within {} s (publishers: {})'.format(
                    CAMERA_INFO_TOPIC, timeout,
                    self.count_publishers(CAMERA_INFO_TOPIC)))
        with self._lock:
            msg = self._camera_info
        return (np.asarray(msg.k, dtype=float).reshape(3, 3),
                np.asarray(msg.d, dtype=float))

    def has_camera(self, timeout=PROBE_TIMEOUT):
        """Whether anything publishes the head camera's colour image.

        Returns
        -------
        bool
        """
        return self._has_publisher(IMAGE_TOPIC, timeout)

    def image(self, timeout=5.0, newer_than=None):
        """The latest colour image from the head camera, as RGB.

        Parameters
        ----------
        timeout : float
            Seconds (wall clock) to wait for an image.
        newer_than : float or None
            Only accept an image stamped after this time (``ri.now()``
            seconds), e.g. one taken after the robot stopped turning.

        Returns
        -------
        image : numpy.ndarray
            ``(height, width, 3)`` uint8, RGB.
        stamp : float
            When it was taken, on the robot's clock [s].
        """
        msg, stamp = self._latest_image(IMAGE_TOPIC, timeout, newer_than)
        return _decode_image(msg), stamp

    def depth_image(self, timeout=5.0, newer_than=None):
        """The latest depth image from the head camera, in metres.

        Pixels with no depth are NaN (a 16-bit image's 0 means "no reading",
        not "touching the lens").

        Returns
        -------
        depth : numpy.ndarray
            ``(height, width)`` float32 [m].
        stamp : float
        """
        msg, stamp = self._latest_image(DEPTH_TOPIC, timeout, newer_than)
        return _decode_image(msg), stamp

    # -- recognition (jedy_vision's nodes) -------------------------------------

    def _call_recognition(self, srv_type, name, request, timeout):
        """Call a recognition node's service; its response, or raise."""
        with self._lock:
            client = self._recognition_clients.get(name)
            if client is None:
                client = self.create_client(srv_type, name)
                self._recognition_clients[name] = client
        if not client.wait_for_service(timeout_sec=RECOGNITION_PROBE_TIMEOUT):
            raise RuntimeError(
                '{} がありません. 認識のノードを起動してください: '
                'ros2 launch jedy_vision recognition.launch.py'.format(name))
        done = threading.Event()
        future = client.call_async(request)
        future.add_done_callback(lambda _f: done.set())
        if not done.wait(timeout):
            raise RuntimeError('{}: no answer within {} s'.format(name, timeout))
        response = future.result()
        if not response.success:
            raise RuntimeError('{}: {}'.format(name, response.message))
        return response

    def what_do_you_see(self, words=None, timeout=10.0):
        """Objects named by ``words`` in the head camera's latest image.

        Parameters
        ----------
        words : str or list of str or None
            What to look for, in Japanese (``ペットボトル``; the words in
            jedy_vision's ``config/vocabulary.yaml``) or English (``bottle``).
            ``None``: the words of the last call (at first: 人, ペットボトル,
            コップ, 椅子).
        timeout : float
            Seconds (wall clock) to wait for the answer.

        Returns
        -------
        list of jedy_vision_msgs.msg.Detection
            Most confident first. Each has ``label`` (the word, as given),
            ``score`` (0..1) and ``box`` (``x1, y1, x2, y2`` [px] in
            ``ri.image()``), and, from the depth image, ``position`` in the
            camera's optical frame with ``has_position``
            (:meth:`detection_position` gives it in base_link). Empty when
            nothing is found.

        Raises
        ------
        RuntimeError
            The detection node is not running, has no recent image, or does
            not know a Japanese word.
        """
        from jedy_vision_msgs.srv import WhatDoYouSee
        if isinstance(words, str):
            words = [words]
        request = WhatDoYouSee.Request(words=list(words or []))
        response = self._call_recognition(
            WhatDoYouSee, DETECTION_SERVICE, request, timeout)
        self._detection_frame = response.result.header.frame_id
        return list(response.result.detections)

    def detection_position(self, detection):
        """Where a :meth:`what_do_you_see` detection is, in base_link [m].

        The camera-frame ``position`` the detection node measured with the
        depth image, moved to base_link through the robot's current pose (so
        ask before turning the head).

        Parameters
        ----------
        detection : jedy_vision_msgs.msg.Detection

        Returns
        -------
        numpy.ndarray or None
            ``(3,)`` x forward, y left, z up; None when the detection has
            no position (no depth reading in its box: too near or too far
            for the camera).
        """
        if not detection.has_position:
            return None
        frame = getattr(self, '_detection_frame', '') or IMAGE_FRAME
        robot = self.robot
        robot.angle_vector(self.angle_vector())
        links = [link for link in robot.link_list if link.name == frame]
        if not links:
            raise RuntimeError('the robot model has no link {!r} (the detection '
                               "image's frame)".format(frame))
        p = detection.position
        return links[0].copy_worldcoords().transform_vector(
            np.array([p.x, p.y, p.z]))

    def sees(self, word, min_score=0.0, timeout=10.0):
        """Whether the head camera sees ``word`` now.

        Parameters
        ----------
        word : str
            Japanese or English, as :meth:`what_do_you_see`.
        min_score : float
            Count only detections at least this confident (the node already
            drops those below its own ``min_score``, 0.25).

        Returns
        -------
        bool
        """
        return any(d.label == word and d.score >= min_score
                   for d in self.what_do_you_see([word], timeout))

    def wait_until_sees(self, word, timeout=30.0, min_score=0.0):
        """Look again and again until ``word`` is seen, or ``timeout`` [s] passes.

        Returns
        -------
        bool
            True once seen; False at the timeout.
        """
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.sees(word, min_score):
                return True
            time.sleep(RECOGNITION_POLL)
        return False

    def _gesture_msg(self, timeout):
        from jedy_vision_msgs.srv import ReadGesture
        return self._call_recognition(
            ReadGesture, GESTURE_SERVICE, ReadGesture.Request(), timeout).gesture

    def gesture(self, timeout=5.0):
        """The hand gesture held up to the head camera now.

        Returns
        -------
        str or None
            グー, チョキ, パー, 指さし, いいね, よくない or アイラブユー; None
            when there is no hand, or it has not been held for a few images.

        Raises
        ------
        RuntimeError
            The gesture node is not running or has no recent image.
        """
        return self._gesture_msg(timeout).label or None

    def janken(self, timeout=5.0):
        """The janken hand held up now: グー, チョキ, パー, or None."""
        return self._gesture_msg(timeout).janken or None

    def wait_until_gesture(self, label=None, timeout=30.0):
        """Wait for a gesture.

        Parameters
        ----------
        label : str or None
            The gesture to wait for (as :meth:`gesture` returns it); None:
            any gesture.
        timeout : float
            Seconds (wall clock).

        Returns
        -------
        str or None
            The gesture seen; None at the timeout.
        """
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            seen = self.gesture()
            if seen is not None and (label is None or seen == label):
                return seen
            time.sleep(RECOGNITION_POLL)
        return None

    def wait_until_janken(self, timeout=30.0):
        """Wait for グー, チョキ or パー; the hand, or None at the timeout."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            hand = self.janken()
            if hand is not None:
                return hand
            time.sleep(RECOGNITION_POLL)
        return None

    # -- speech (jsk_3rdparty's VOICEVOX and ros_speech_recognition) ----------

    def say(self, text, wait=True, speaker='', timeout=60.0):
        """Say ``text`` in Japanese with VOICEVOX, through the PC's speaker.

        Parameters
        ----------
        text : str
        wait : bool
            Return when it has been said (a few seconds a sentence: making
            the voice, then saying it); False returns once it has started.
        speaker : str
            A VOICEVOX voice by name, e.g. ``ずんだもん-ノーマル`` (the names
            are in ``~/.ros/voicevox/speakers.json``); empty: the default.
            A name it does not know is not an error: nothing is said.
        timeout : float
            Seconds (wall clock) to wait with ``wait``.

        Raises
        ------
        RuntimeError
            VOICEVOX is not running.
        """
        from rclpy.action import ActionClient
        from sound_play_msgs.action import SoundRequest as SoundRequestAction
        from sound_play_msgs.msg import SoundRequest
        with self._lock:
            if self._say_client is None:
                self._say_client = ActionClient(self, SoundRequestAction, SAY_ACTION)
            client = self._say_client
        if not client.wait_for_server(timeout_sec=SPEECH_PROBE_TIMEOUT):
            raise RuntimeError(
                '{} がありません. 音声合成を起動してください: '
                'ros2 launch voicevox voicevox_texttospeech.launch.xml'.format(SAY_ACTION))
        goal = SoundRequestAction.Goal()
        goal.sound_request.sound = SoundRequest.SAY
        goal.sound_request.command = SoundRequest.PLAY_ONCE
        goal.sound_request.volume = 1.0
        goal.sound_request.arg = text
        goal.sound_request.arg2 = speaker
        handle = self._wait_done(client.send_goal_async(goal), timeout, 'say')
        if not handle.accepted:
            raise RuntimeError('say: {} refused the goal'.format(SAY_ACTION))
        if wait:
            self._wait_done(handle.get_result_async(), timeout, 'say')

    def _wait_done(self, future, timeout, what):
        done = threading.Event()
        future.add_done_callback(lambda _f: done.set())
        if not done.wait(timeout):
            raise RuntimeError('{}: no answer within {} s'.format(what, timeout))
        return future.result()

    def _on_speech(self, msg):
        if msg.transcript:
            with self._lock:
                self._speech_text = msg.transcript[0]
            self._speech_heard.set()

    def listen(self, timeout=10.0):
        """The next sentence heard by the speech recogniser.

        Only what is said after the call counts.

        Parameters
        ----------
        timeout : float
            Seconds (wall clock) to wait.

        Returns
        -------
        str or None
            The sentence; None if nothing was heard within ``timeout``.

        Raises
        ------
        RuntimeError
            The speech recogniser is not running.
        """
        from speech_recognition_msgs.msg import SpeechRecognitionCandidates
        with self._lock:
            if self._speech_sub is None:
                self._speech_sub = self.create_subscription(
                    SpeechRecognitionCandidates, SPEECH_TOPIC, self._on_speech, 10)
            self._speech_heard.clear()
        if not self._has_publisher(SPEECH_TOPIC, SPEECH_PROBE_TIMEOUT):
            raise RuntimeError(
                '{} に publisher がありません. 音声認識を起動してください: '
                'ros2 launch ros_speech_recognition speech_recognition.launch.xml '
                'language:=ja-JP ...'.format(SPEECH_TOPIC))
        if not self._speech_heard.wait(timeout):
            return None
        with self._lock:
            return self._speech_text

    # -- servos ---------------------------------------------------------------

    def _on_servo_state(self, msg):
        with self._lock:
            self._servo_state = msg
        self._servo_state_seen.set()

    def missing_servos(self, timeout=SERVO_STATE_TIMEOUT):
        """The joints whose servo does not answer, and why.

        Parameters
        ----------
        timeout : float
            Seconds (wall clock) to wait for the first state.

        Returns
        -------
        dict
            ``{joint name: reason}``; empty when every servo answers. The
            reason is the bus's own words (e.g. ``never answered on ICS 5``),
            or ``'not answering'`` when it cannot say.

        Raises
        ------
        RuntimeError
            A servo switch is running but publishes no state: whether the
            servos answer is then unknown, which is not the same as "all".
        """
        if self._servo_client is None:
            return {}  # mock hardware: nothing that could be missing
        if not self._servo_state_seen.wait(timeout):
            raise RuntimeError(
                'no {}/servo/state within {} s: the servo switch is up but '
                'does not say which servos answer'.format(SERVO_NS, timeout))
        with self._lock:
            msg = self._servo_state
        return {j.name: (j.absence_reason or 'not answering')
                for j in msg.joints if not j.present}

    def _warn_missing(self, joints, what):
        """Log, at most every few seconds per joint, that ``joints`` will not move."""
        if self._servo_client is None or not self._servo_state_seen.is_set():
            return
        missing = self.missing_servos()
        now = time.monotonic()
        gone = [j for j in joints if j in missing
                and now - self._missing_warn_time.get(j, -1e9) > MISSING_WARN_PERIOD]
        if not gone:
            return
        for j in gone:
            self._missing_warn_time[j] = now
        self.get_logger().error(
            '{}: {} のサーボが応答していないので動きません ({}). '
            'ケーブルと電源を確認してください (RViz で赤く点滅している関節)'.format(
                what, ', '.join(gone), '; '.join(
                    '{}: {}'.format(j, missing[j]) for j in gone)))

    def angle_vector(self, av=None, time=None, controller_type=None, *args, **kwargs):
        """As the base class; also says if a commanded joint has no servo.

        Raises
        ------
        BoardNotAnswering
            The board stopped: the angles would be the last ones read
            before it did.
        """
        self._check_board()
        if av is not None:
            names = (self.joints_of(controller_type) if controller_type
                     else [j for c in self.controller_names for j in self.joints_of(c)])
            self._warn_missing(names, 'angle_vector')
        return super(JedyInterface, self).angle_vector(
            av, time, controller_type, *args, **kwargs)

    def servo_on(self, joints=None, time=0.1, timeout=10.0, attempts=3):
        """Turn the torque on, holding where the robot is now.

        With no ``joints``, the wheels too. ``servo_off()`` with no joints
        releases every joint on the bus, wheels included, and the base
        controller cannot turn them back on, so without this a servo_off /
        servo_on round trip leaves a base that silently does not drive. A
        wheel has no pose to jump to, so it needs no hold first.

        Parameters
        ----------
        joints : list of str or None
            ``None``: every arm, head and gripper joint, and the wheels.
        time : float
            Seconds for the hold.
        timeout : float
            Seconds to wait for each answer.
        attempts : int
            Re-syncs to try (see actuo_ros.robot_interface.RobotInterface.servo_on).
        """
        self._check_board()
        if self._servo_client is not None:
            wanted = (list(joints) if joints is not None else
                      [j for c in self.controller_names for j in self.joints_of(c)]
                      + WHEEL_JOINTS)
            missing = self.missing_servos()
            gone = [j for j in wanted if j in missing]
            if gone:
                # The switch would refuse too ("X not answering"); saying it
                # here, with the reason and what to do, saves a round trip
                # through a terse refusal.
                raise ServoRefused(
                    'サーボが応答していません: {}. ケーブル・電源を確認してから '
                    'もう一度. 応答しない関節を除いて動かすなら '
                    'ri.servo_on(joints=[...]) で関節を指定する. '
                    '(RViz で赤く点滅している関節です)'.format('; '.join(
                        '{} ({})'.format(j, missing[j]) for j in gone)))
        super(JedyInterface, self).servo_on(joints, time, timeout, attempts)
        if joints is None and self._servo_client is not None:
            self._servo(True, WHEEL_JOINTS, timeout)

    def _servo(self, enable, joints, timeout):
        if self._servo_client is None:
            if enable:
                # True, not pretended: with no switch the joints are driven.
                self.get_logger().info(
                    'no {}/servo/set: the joints are always driven, so '
                    'servo_on has nothing to switch'.format(SERVO_NS))
                return
            raise RuntimeError(
                'servo off is not available here: nothing serves {}/servo/set '
                '(the real robot, jedy_gazebo.launch.py and '
                'jedy_isaac.launch.py all do; is the bring-up running?)'
                .format(SERVO_NS))
        return super(JedyInterface, self)._servo(enable, joints, timeout)

def _decode_image(msg):
    """sensor_msgs/Image to a numpy array, without cv_bridge.

    Colour comes back as RGB uint8; depth as float32 metres with NaN where
    there is no reading. Any other encoding is refused, not guessed.
    """
    big = '>' if msg.is_bigendian else '<'
    channels = {'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4,
                'mono8': 1}.get(msg.encoding)
    if channels is not None:
        flat = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        rows = flat.reshape(msg.height, msg.step)[:, :msg.width * channels]
        img = rows.reshape(msg.height, msg.width, channels)
        if msg.encoding == 'mono8':
            return np.repeat(img, 3, axis=2)
        if msg.encoding.startswith('bgr'):
            img = img[:, :, 2::-1]
        return np.ascontiguousarray(img[:, :, :3])
    depth_types = {'32FC1': (big + 'f4', 1.0), '16UC1': (big + 'u2', 1e-3)}
    if msg.encoding in depth_types:
        dtype, scale = depth_types[msg.encoding]
        size = np.dtype(dtype).itemsize
        flat = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        rows = flat.reshape(msg.height, msg.step)[:, :msg.width * size]
        raw = rows.copy().view(dtype).reshape(msg.height, msg.width)
        depth = raw.astype(np.float32) * scale
        depth[(raw == 0) | ~np.isfinite(depth)] = np.nan
        return depth
    raise ValueError('image encoding {!r} is not supported'.format(
        msg.encoding))


def _probe(namespace, servo_timeout=SERVO_PROBE_TIMEOUT):
    """Look at the running system: simulated time? a servo switch?

    Simulated time is read from the controller manager's ``use_sim_time``
    parameter (it is already known to be up: :func:`_wait_for_controllers`
    ran first), not guessed from whether a ``/clock`` publisher is discovered
    within a few seconds. That guess raced discovery: on a loaded machine it
    sometimes saw no /clock in a simulator, and ``ri.now()`` then silently ran
    on the wall clock while every image and joint state was stamped in
    simulated time.

    Returns
    -------
    use_sim_time : bool
    has_servo : bool
    """
    from actuo_msgs.action import SetServo
    from rcl_interfaces.msg import ParameterType
    from rcl_interfaces.srv import GetParameters
    node = rclpy.create_node('jedy_skrobot_probe')
    try:
        service = namespace + '/controller_manager/get_parameters'
        params = node.create_client(GetParameters, service)
        if not params.wait_for_service(timeout_sec=CONTROLLER_TIMEOUT):
            raise RuntimeError('no {} within {} s'.format(service, CONTROLLER_TIMEOUT))
        # Ask again when no answer comes: right after discovery a server can
        # drop its response ("failed to send response ... client will not
        # receive response" in the launch log) because it has not matched
        # this client's reader yet. Asking for a parameter is safe to repeat.
        deadline = time.monotonic() + PARAMETER_TIMEOUT
        result = None
        while result is None and time.monotonic() < deadline:
            future = params.call_async(
                GetParameters.Request(names=['use_sim_time']))
            rclpy.spin_until_future_complete(
                node, future, timeout_sec=PARAMETER_RETRY)
            result = future.result()
            if result is None:
                params.remove_pending_request(future)
        if result is None:
            raise RuntimeError('{} did not answer within {} s'.format(
                service, PARAMETER_TIMEOUT))
        value = result.values[0]
        use_sim_time = (value.type == ParameterType.PARAMETER_BOOL
                        and value.bool_value)

        client = ActionClient(node, SetServo, SERVO_NS + '/servo/set')
        deadline = time.monotonic() + servo_timeout
        has_servo = False
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if client.server_is_ready():
                has_servo = True
                break
        client.destroy()
        return use_sim_time, has_servo
    finally:
        node.destroy_node()


def _wait_for_controllers(namespace, timeout=CONTROLLER_TIMEOUT):
    """Block until every one of :data:`CONTROLLERS` is active.

    Taking whatever happens to be loaded at the moment of the call gave an
    ``ri`` that silently drove three of the five limbs when a script started
    while the launch was still spawning.
    """
    from controller_manager_msgs.srv import ListControllers
    node = rclpy.create_node('jedy_skrobot_wait')
    try:
        client = node.create_client(
            ListControllers, namespace + '/controller_manager/list_controllers')
        deadline = time.monotonic() + timeout
        state = {}
        while time.monotonic() < deadline:
            if client.wait_for_service(timeout_sec=1.0):
                future = client.call_async(ListControllers.Request())
                rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
                if future.done() and future.result() is not None:
                    state = {c.name: c.state
                             for c in future.result().controller}
                    if all(state.get(c) == 'active' for c in CONTROLLERS):
                        return
            time.sleep(0.5)
        raise RuntimeError(
            'not every Jedy controller became active within {} s: {} '
            '(is the bring-up running?)'.format(
                timeout, {c: state.get(c, 'not loaded') for c in CONTROLLERS}))
    finally:
        node.destroy_node()


def connect(namespace=NAMESPACE, load_meshes=False):
    """Connect to whichever Jedy is running.

    Parameters
    ----------
    namespace : str
        The bring-up's namespace; every Jedy launch uses ``/jedy_fullbody``.
    load_meshes : bool
        Load the meshes, which only a viewer needs.

    Returns
    -------
    robot : skrobot.models.Jedy
        A model set to the robot's current pose. ``robot.rarm``,
        ``robot.larm`` and ``robot.head`` with their ``end_coords`` work, so
        does inverse kinematics.
    ri : JedyInterface
    """
    if not rclpy.ok():
        # Ctrl-C must not shut ROS down under the program: rclpy's own SIGINT
        # handler invalidates the context first, so a sample's
        # ``finally: ri.stop()`` then fails with "publisher's context is
        # invalid" and the base gets no stop command. Without it Ctrl-C is a
        # plain KeyboardInterrupt, cleanup still publishes, and
        # actuo_ros' atexit hook shuts ROS down afterwards.
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    _wait_for_controllers(namespace)
    use_sim_time, has_servo = _probe(namespace)
    robot, ri = _actuo_connect(
        servo_ns=SERVO_NS if has_servo else None,
        load_meshes=load_meshes,
        model_class=Jedy,
        namespace=namespace,
        interface_class=JedyInterface,
        use_sim_time=use_sim_time)
    # Say at once if servos are missing: otherwise the first sign is a
    # servo_on refusal or an arm that does not move, far from the cause.
    missing = ri.missing_servos()
    if missing:
        ri.get_logger().error(
            '応答していないサーボがあります ({} 個): {}. ケーブル・電源を確認. '
            'これらの関節は動きません (RViz で赤く点滅)'.format(
                len(missing), '; '.join('{} ({})'.format(j, r)
                                        for j, r in sorted(missing.items()))))
    return robot, ri
