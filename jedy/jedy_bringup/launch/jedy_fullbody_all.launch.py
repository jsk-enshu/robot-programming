#!/usr/bin/env python3
"""Everything, from nothing running: Jedy's whole body on real hardware.

    ros2 launch jedy_bringup jedy_fullbody_all.launch.py              # ARM-H7
    ros2 launch jedy_bringup jedy_fullbody_all.launch.py board:=rcb4  # RCB-4mini
    ros2 launch jedy_bringup jedy_fullbody_all.launch.py board:=rcb4 \
        link:=wifi relay:=192.168.1.50      # RCB-4mini through a UNO R4 WiFi

Starts the bus servicer, ros2_control with all seven controllers, the wheel
odometry, RViz and the PS4 teleop. Arguments turn pieces off (`rviz:=false`,
`joy:=false`, `wheels:=false`).

Nothing starts until the robot is plugged in
--------------------------------------------
The first thing this launch does is wait for the serial adapter the Kondo
config names (`/dev/actuo_kondo_rcb4mini` or `/dev/actuo_kondo_armh7`), and it starts nothing until that port opens.
`wait_port:=false` skips the wait; `wait_timeout:=<seconds>` gives up instead of
waiting forever.

That is not caution, it is the difference between one readable line and a
crash. `actuo_bus_node --real` opens the port in its first second and does not
survive not finding it, and everything downstream is built on it having
succeeded. Measured on 2026-08-06 with nothing plugged in, this launch produced:

    actuo_bus_node        died, exit 1     SerialException: could not open port
                                          /dev/actuo_kondo_armh7
    ros2_control_node    died, exit -6    std::runtime_error 10 s later, with a
                                          C++ backtrace: "no state from the bus
                                          servicer ... Refusing to activate"
    7 x spawner          alive forever, "Failed to acquire lock in 20 seconds.
                                        Attempt 1 of 5 ... Retrying"

Three failures, an abort backtrace and an endless retry, for one unplugged
cable -- and the line that says which cable scrolled past a minute earlier.

Waiting is also the more useful behaviour: the launch can be started before the
robot is switched on, and it comes up by itself when someone switches it on.

Only the FIRST open needs this. `actuo_bus_node --reconnect` is on by default, so
once the port has opened, the servicer already survives the robot being switched
off and on: it releases every joint, waits for the board, and carries on with
torque off.

Two 7-DOF arms, two grippers, a 2-DOF head and the four-wheel mecanum base: 22
joints, against the board scanned on 2026-08-03 which reported all 22 servos
present.

Why this exists rather than three terminals
-------------------------------------------
The order matters and it is not guessable, and getting it wrong is how the
first bring-up of this robot was lost. The servicer and ros2_control hand each
other the shared-memory segment: the C++ ActuoSystem CREATES it and blocks until
the servicer has written the MEASURED state into it, and only then seeds its
command from that. So the servicer is the side that WAITS, and it must be told
to wait long enough -- started on its own first, with the stock 30 s timeout, it
gives up before ros2_control has even reached `activate` and exits, leaving a
launch that then hangs forever against nobody. Started here it waits 90 s for a
segment that appears a second or two from now.

The stale segment, and why this deletes it
------------------------------------------
`ShmBridge` is an mmap'd FILE under /tmp, not a POSIX shm object, so it outlives
every process that touched it. A leftover from the last run lets the servicer
attach and complete its handshake against nobody, before ros2_control has taken
the segment -- and then both sides wait. So the file is removed first.

That is safe rather than merely convenient: actuo_bus_node takes a lock keyed to
the shm name, so a second servicer cannot start on the same robot however many
times this launch is run. Removing the file cannot orphan a live one into a
private segment without that lock catching it.

What comes up driven
--------------------
The four wheels. A continuous-rotation servo has no position, so the stale-pose
hazard that servo-on exists for cannot happen to it, and the servicer enables
them at startup. Everything else comes up LIMP and needs TRIANGLE on the pad.
With `wheels:=true` the base will drive the moment a twist reaches /cmd_vel and
the deadman is held, so bring it up with the wheels off the ground the first
time.
"""

import os
import sys

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, ExecuteProcess, GroupAction,
    OpaqueFunction,
    IncludeLaunchDescription, LogInfo, RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration, PathJoinSubstitution, PythonExpression)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackagePrefix, FindPackageShare

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from jedy_preflight import preflight_action  # noqa: E402

NAMESPACE = "jedy_fullbody"
SHM_NAME = "jedy_fullbody"


def _channel_from_config(path):
    """The serial device a bus config names.

    Parameters
    ----------
    path : str
        Path to the installed bus config YAML.

    Returns
    -------
    str
        ``bus.transport.channel``, e.g. ``/dev/actuo_kondo_rcb4mini``.

    Raises
    ------
    RuntimeError
        If the file cannot be read or names no channel. Raised rather than
        defaulted: a guessed device path would make the wait pass on a machine
        where the servicer is about to fail, which is the one outcome this
        whole mechanism exists to prevent.
    """
    try:
        with open(path) as handle:
            document = yaml.safe_load(handle)
    except OSError as error:
        raise RuntimeError(
            "cannot read the bus config {}: {}".format(path, error))
    try:
        channel = document["bus"]["transport"]["channel"]
    except (KeyError, TypeError):
        raise RuntimeError(
            "{} has no bus.transport.channel".format(path))
    if not isinstance(channel, str) or channel == "":
        raise RuntimeError(
            "bus.transport.channel in {} is not a device path".format(path))
    return channel


def _on_port_ready(event, context, stack):
    """Start the robot, or shut the launch down.

    Parameters
    ----------
    event : launch.events.process.ProcessExited
        Carries the waiter's exit status.
    context : launch.LaunchContext
        Unused; required by the event-handler signature.
    stack : list of launch.Action
        The robot, to start once the port is there.

    Returns
    -------
    list of launch.Action
        The robot stack on success. On failure, a shutdown -- there is no
        point starting a servicer that is about to die on the same port,
        and jedy_wait_port has already printed what to do about it.
    """
    del context
    if event.returncode == 0:
        return stack
    return [
        LogInfo(msg="[jedy_fullbody_all] the robot's link (its serial port, "
                    "or with link:=wifi its relay) never came up, so nothing "
                    "was started. See the waiter's message above."),
        EmitEvent(event=Shutdown(reason="serial port unavailable")),
    ]


#: Bus config per board. Same joints, same order, same URDF and controllers.
BOARD_CONFIGS = {
    "h7": "jedy_fullbody_kondo.yaml",
    "rcb4": "jedy_fullbody_rcb4.yaml",
}


def _config_path(context):
    """The bus config to run: ``config:=`` if given, else the board's.

    Parameters
    ----------
    context : launch.LaunchContext

    Returns
    -------
    str
        Path of the config the servicer is given and the port wait reads.
    """
    explicit = LaunchConfiguration("config").perform(context)
    if explicit:
        return explicit
    board = LaunchConfiguration("board").perform(context)
    return os.path.join(get_package_share_directory("jedy_bringup"),
                        "config", BOARD_CONFIGS[board])


def _link_args(context):
    """The servicer's transport overrides for ``link:=``.

    ``serial`` (default): none, the config's cable. ``wifi``: the RCB-4mini's
    packets go to a UART relay (an Arduino UNO R4 WiFi running
    the UART relay firmware) at ``relay:=HOST[:PORT]``, over UDP; everything else in
    the config is the cable's. The board runs at 115200 there: whether the UNO
    R4's UART keeps up with the 1.25 Mbit/s the cable uses is not verified.

    Parameters
    ----------
    context : launch.LaunchContext

    Returns
    -------
    list of str
    """
    link = LaunchConfiguration("link").perform(context)
    if link == "serial":
        return []
    board = LaunchConfiguration("board").perform(context)
    relay = LaunchConfiguration("relay").perform(context)
    if board != "rcb4":
        raise RuntimeError("link:=wifi is for board:=rcb4 (the relay forwards "
                           "the RCB-4mini's COM port), not board:={}"
                           .format(board))
    if not relay:
        raise RuntimeError("link:=wifi needs relay:=HOST[:PORT], the "
                           "relay's address (its sketch prints it on USB "
                           "serial when it joins the network)")
    return ["--transport", "kondo_rcb4_udp", "--channel", relay,
            "--bitrate", LaunchConfiguration("relay_bitrate").perform(context),
            "--transport-param", "failsafe_silence={}".format(
                LaunchConfiguration("failsafe_silence").perform(context)),
            # A USB reset is meaningless for a network address.
            "--no-usb-reset"]


#: D405 streams. Its colour image comes from the left depth imager, so depth
#: and colour are pixel-aligned at equal profiles: ri.depth_image()[v, u] is
#: the depth of colour pixel (u, v), as in Gazebo, with no align_depth.
D405_PROFILE = "640x480x30"


def _camera(context):
    """camera:=true -> the head D405 through realsense2_camera.

    Topics land where jedy_skrobot reads them (and Gazebo publishes them):
    /camera/color/image_rect_raw, /camera/depth/image_rect_raw. TF is off:
    camera_link and the optical frames already come from the robot's URDF
    via robot_state_publisher; the driver publishing them again would fight
    it.
    """
    if LaunchConfiguration("camera").perform(context) != "true":
        return []
    try:
        get_package_share_directory("realsense2_camera")
    except Exception:  # PackageNotFoundError
        raise RuntimeError(
            "camera:=true needs the RealSense ROS driver: "
            "sudo apt install ros-jazzy-realsense2-camera")
    # The node itself rather than its rs_launch.py, for the remapping: on a
    # D405 the colour image comes out as color/image_raw (it is not rectified
    # by a separate stage, and realsense2_camera names it so), and every
    # reader here -- jedy_skrobot, the samples, Gazebo -- uses
    # color/image_rect_raw. Remapped here once rather than in each reader.
    return [Node(
        package="realsense2_camera", executable="realsense2_camera_node",
        name="camera", namespace="", output="screen",
        parameters=[{
            "camera_name": "camera",
            "enable_color": True,
            "enable_depth": True,
            "depth_module.depth_profile": D405_PROFILE,
            "depth_module.color_profile": D405_PROFILE,
            "align_depth.enable": False,
            "publish_tf": False,
        }],
        remappings=[("/camera/color/image_raw", "/camera/color/image_rect_raw")])]


def generate_launch_description():
    """Build the launch description.

    Returns
    -------
    launch.LaunchDescription
        The stale-segment removal, the bus servicer, the ros2_control stack and
        the teleop.
    """
    pkg = FindPackageShare("jedy_bringup")
    poses = PathJoinSubstitution([pkg, "config", "jedy_fullbody_poses.yaml"])

    def servicer(config, link_args):
        """The bus servicer, as a fresh action each call.

        Parameters
        ----------
        config : str
            The bus config, see :func:`_config_path`.
        link_args : list of str
            Extra arguments for the link (see :func:`_link_args`).

        Returns
        -------
        launch_ros.actions.Node
            actuo_bus_node against the real board.
        """
        return Node(
            package="actuo_ros", executable="actuo_bus_node",
            output="screen",
            arguments=[
                "--real",
                "--config", config,
                "--shm-name", SHM_NAME,
                "--rate", LaunchConfiguration("rate"),
                # Long, because this is the side that waits: the segment does
                # not exist until ros2_control's on_init creates it a second or
                # two from now. The stock 30 s is not enough here.
                "--timeout", "90",
                # RAISED FROM THE 3.0 DEFAULT, from a measurement rather than
                # to quieten a light. The default puts the single-cycle ERROR
                # at 3x the 10 ms period, and on the desktop it was measured on the isolated
                # spikes form a tight population that straddles exactly that
                # line. Sampled over 120 one-second windows with RViz and the
                # teleop running:
                #
                #   loop_period_max  min 22.1 median 27.8 p90 29.9 MAX 32.0 ms
                #   windows over 30 ms (the 3.0x ERROR)  10 / 120  = 8.3 %
                #   windows over 35 ms                    0 / 120
                #
                # So the threshold was cutting through the middle of ordinary
                # scheduling jitter and going red 8 % of the time while
                # reporting nothing new: drop_rate stayed 0.000000,
                # consecutive_misses 0, and the loop's own p95 never left
                # 10.0-10.1 ms against a 10.0 ms target. The servicer is plain
                # SCHED_OTHER python, so 20-30 ms preemptions on a desktop
                # running RViz are the expected shape.
                #
                # 4.0 puts the line 25 % above the worst spike observed
                # there, so a stall genuinely worse than that desktop's normal
                # still trips it, and it is still only four control periods.
                #
                # --diag-loop-error-ratio is deliberately LEFT AT ITS DEFAULT.
                # That is the p95 check -- "the loop is not keeping up", the
                # one that means work is actually being lost -- and it has 3 ms
                # of headroom (p95 peaks at 10.08 ms against a 13 ms
                # threshold). Widening both would have bought quiet by removing
                # the alarm worth having.
                "--diag-loop-spike-error-ratio", "4.0",
                "--ros",
            ] + link_args,
        )

    def robot_stack(config, link_args):
        """The actions that assume the bus is there, as fresh objects.

        A factory rather than a list, because these are needed on two
        mutually exclusive paths -- after the port wait, or immediately when
        the wait is skipped -- and a launch action belongs to one place in one
        tree.

        Returns
        -------
        list of launch.Action
            Stale-segment removal, the servicer, and the ros2_control stack.
        """
        return [
            # Before anything else. See the module docstring.
            ExecuteProcess(
                cmd=["rm", "-f",
                     os.path.join("/tmp", "actuo_{}.shm".format(SHM_NAME))],
                output="log"),

            servicer(config, link_args),

            # A moment later, so the servicer is already waiting when the
            # segment appears. Not required for correctness -- the handshake
            # works either way round -- but it keeps the startup log readable,
            # which matters when the thing you are reading it for is why a
            # robot did not come up.
            TimerAction(period=3.0, actions=[
                # scoped: launch_arguments are ordinary SetLaunchConfiguration
                # actions in the INCLUDING context, so without this, `wheels`
                # and `rviz` would leak outward and silently overwrite this
                # launch's own arguments.
                GroupAction([
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(PathJoinSubstitution(
                            [pkg, "launch", "jedy_fullbody.launch.py"])),
                        launch_arguments={
                            "wheels": LaunchConfiguration("wheels"),
                            "rviz": LaunchConfiguration("rviz"),
                            "backend": "actuo",
                            "use_sim": "false",
                        }.items()),
                ], scoped=True),
            ]),
        ]

    # The channel is read out of the SAME config the servicer is given, so the
    # port this waits for and the port that gets opened cannot drift apart. A
    # launch that waited for the right device and then started a servicer
    # pointed at a different one would be worse than not waiting at all.
    #
    # Read here and passed as --channel, rather than handing the file to the
    # waiter: `#!/usr/bin/env python3` resolves against PATH, and on a machine
    # with a project venv activated that is the venv's interpreter, which has
    # no PyYAML. Measured 2026-08-06 -- the waiter died on `import yaml` while
    # `ros2 launch` itself, running under /usr/bin/python3, was fine. This
    # launch is already under an interpreter with PyYAML by definition, so the
    # parse happens here and jedy_wait_port stays pure stdlib. It is the
    # program that has to work when everything else is broken.
    def setup(context):
        """What depends on the chosen board: the port wait and the robot.

        Parameters
        ----------
        context : launch.LaunchContext

        Returns
        -------
        list of launch.Action
        """
        config = _config_path(context)
        link_args = _link_args(context)
        if link_args:
            # No serial device here: the relay is on the network. Wait until
            # it answers, as the cable path waits for its port, because a
            # servicer whose first open fails exits.
            relay = link_args[link_args.index("--channel") + 1]
            wait_relay = ExecuteProcess(
                cmd=[PathJoinSubstitution([FindPackagePrefix("actuo_ros"),
                                           "lib", "actuo_ros",
                                           "actuo-uart-relay-ping"]),
                     "--relay", relay,
                     "--timeout", LaunchConfiguration("wait_timeout")],
                output="screen")

            def on_relay_ready(event, context):
                return _on_port_ready(event, context,
                                      robot_stack(config, link_args))

            return [
                LogInfo(msg="[jedy_fullbody_all] link:=wifi: the board "
                            "through the relay at {}".format(relay)),
                wait_relay,
                RegisterEventHandler(OnProcessExit(target_action=wait_relay,
                                                   on_exit=on_relay_ready)),
            ]
        channel = _channel_from_config(config)
        wait_port = ExecuteProcess(
            cmd=[PathJoinSubstitution([FindPackagePrefix("jedy_bringup"),
                                       "lib", "jedy_bringup",
                                       "jedy_wait_port"]),
                 "--channel", channel,
                 "--timeout", LaunchConfiguration("wait_timeout")],
            output="screen",
            condition=IfCondition(LaunchConfiguration("wait_port")))

        def on_port_ready(event, context):
            return _on_port_ready(event, context, robot_stack(config, []))

        return [
            # Nothing below this line runs until the port opens. See the
            # module docstring for what happened when it did not.
            wait_port,
            RegisterEventHandler(OnProcessExit(target_action=wait_port,
                                               on_exit=on_port_ready)),
            # ...unless the wait was turned off, in which case straight in.
            GroupAction(robot_stack(config, []),
                        condition=UnlessCondition(
                            LaunchConfiguration("wait_port"))),
        ]

    return LaunchDescription([
        # Fail now, with the PIDs, if a robot or an old Gazebo is running.
        preflight_action(),
        DeclareLaunchArgument("camera", default_value="false",
                              choices=["true", "false"],
                              description="start the head RealSense D405 "
                                          "(USB) as /camera/color, "
                                          "/camera/depth"),
        OpaqueFunction(function=_camera),
        DeclareLaunchArgument("wheels", default_value="true",
                              choices=["true", "false"],
                              description="drive the four mecanum wheels. "
                                          "Adds their velocity interfaces to "
                                          "the URDF and starts "
                                          "mecanum_drive_controller and the "
                                          "wheel odometry."),
        DeclareLaunchArgument("rviz", default_value="true",
                              choices=["true", "false"]),
        DeclareLaunchArgument("joy", default_value="true",
                              choices=["true", "false"],
                              description="PS4 teleop. Harmless with no pad "
                                          "plugged in: joy_linux opens the "
                                          "device when one appears."),
        DeclareLaunchArgument("rate", default_value="100.0",
                              description="servicer loop rate. Pairs with "
                                          "controller_manager's update_rate "
                                          "and with bus.move_time_ms in the "
                                          "Kondo config."),
        DeclareLaunchArgument("wait_port", default_value="true",
                              choices=["true", "false"],
                              description="wait for the serial adapter named "
                                          "in the Kondo config before "
                                          "starting anything. false restores "
                                          "the old behaviour, where an "
                                          "unplugged robot takes the servicer "
                                          "and then ros2_control down with "
                                          "it."),
        DeclareLaunchArgument("wait_timeout", default_value="0",
                              description="seconds to wait for that port. 0 "
                                          "(default) waits forever, so a "
                                          "launch started before the robot "
                                          "was switched on comes up when "
                                          "someone switches it on."),

        DeclareLaunchArgument("board", default_value="h7",
                              choices=sorted(BOARD_CONFIGS),
                              description="the servo board: h7 = Kondo "
                                          "ARM-H7, rcb4 = Kondo RCB-4mini"),
        DeclareLaunchArgument("link", default_value="serial",
                              choices=["serial", "wifi"],
                              description="serial = the config's cable; "
                                          "wifi = through a UNO R4 WiFi "
                                          "relay (board:=rcb4 only)"),
        DeclareLaunchArgument("relay", default_value="",
                              description="link:=wifi: the relay's "
                                          "HOST[:PORT] (port 9750 default)"),
        DeclareLaunchArgument("relay_bitrate", default_value="115200",
                              description="link:=wifi: the board's COM rate "
                                          "behind the relay"),
        # 0.5, not 0.3: through the emulated relay, actuo_bus_node's loop was
        # measured pausing 0.3-0.35 s now and then (3 times in ~4 min, e.g.
        # while a new ri client connected), and each pause stopped the wheels
        # until the next cycle. 0.5 s at the course's 0.1 m/s is 5 cm.
        DeclareLaunchArgument("failsafe_silence", default_value="0.5",
                              description="link:=wifi: seconds without the "
                                          "host after which the relay stops "
                                          "the wheels"),
        DeclareLaunchArgument("config", default_value="",
                              description="a bus config to use instead of "
                                          "the board's (e.g. the copy "
                                          "actuo-kondo-rcb4-emulator writes)"),

        OpaqueFunction(function=setup),

        GroupAction([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution(
                    [FindPackageShare("jedy_teleop"), "launch",
                     "jedy_joy.launch.py"])),
                launch_arguments={
                    "namespace": "/" + NAMESPACE,
                    # This robot's second head joint is head_joint1, where
                    # jedy_four_dof calls it head_module1_joint1.
                    "head_joints": "head_joint0,head_joint1",
                    # This robot's own poses, and the file jedy_joy reads its
                    # controller/joint grouping from.
                    "poses": poses,
                    # Geometry, derived from this URDF rather than four_dof's:
                    # the wheels' collision cylinders (from the CAD), and
                    # |x| + |y| = 0.0435 + 0.0505 for a wheel from the centre.
                    "wheel_radius": "0.02525",
                    "centre_projection": "0.094",
                    # The usable wheel speed range of the chosen board's
                    # config (deadband_deg / saturate_deg x the slope), as
                    # rad/s: RCB-4mini 5..10 deg at ~0.0985 rev/s per deg.
                    "min_wheel_rad_s": PythonExpression([
                        "'3.09' if '", LaunchConfiguration("board"),
                        "' == 'rcb4' else '2.83'"]),
                    "max_wheel_rad_s": PythonExpression([
                        "'6.19' if '", LaunchConfiguration("board"),
                        "' == 'rcb4' else '6.85'"]),
                }.items(),
                condition=IfCondition(LaunchConfiguration("joy"))),
        ], scoped=True),
    ])
