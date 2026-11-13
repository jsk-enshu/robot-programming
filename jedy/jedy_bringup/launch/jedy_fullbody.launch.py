"""Bring up Jedy's whole body under ros2_control, namespaced /jedy_fullbody.

Two 7-DOF arms, two grippers, a 2-DOF head, and optionally the mecanum base.
The kinematics are jedy_description's jedy_gz.xacro, the model brought over from
jsk-enshu/robot-programming; the <ros2_control> that drives it is
jedy_bringup/urdf/jedy_fullbody_actuo.urdf.xacro.

No hardware, nothing to plug in (mock backend, every joint a perfect servo)::

    ros2 launch jedy_bringup jedy_fullbody.launch.py backend:=mock

Real Kondo backend::

    ros2 launch jedy_bringup jedy_fullbody.launch.py
    ros2 run actuo_ros actuo_bus_node --real \\
        --config <share>/jedy_bringup/config/jedy_fullbody_rcb4.yaml \\
        --shm-name jedy_fullbody --rate 100

Add wheels:=true to both the launch and the URDF expansion to drive the base.
It is off by default so that nothing on this robot can be told to drive until
someone asks for it.

--rate is the servicer's own argument, not a launch argument -- this launch does
not start the servicer, so a launch arg for it would be a knob that does
nothing. 100 Hz matches this stack's controller_manager update_rate
(jedy_fullbody_controllers.yaml), so one bus cycle maps to one control period on
a request/response board. It pairs with the move bytes in the bus config,
which must move with it.

The C++ ActuoSystem creates the shared memory segment "jedy_fullbody"; run the
matching backend servicer alongside. The two hand each other that segment, so if
you restart one, restart both.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    Command, LaunchConfiguration, PathJoinSubstitution)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

NAMESPACE = "jedy_fullbody"


def generate_launch_description():
    """Build the launch description.

    Returns
    -------
    launch.LaunchDescription
        controller_manager, the state publishers, and one spawner per
        controller.
    """
    pkg = FindPackageShare("jedy_bringup")
    urdf = PathJoinSubstitution([pkg, "urdf", "jedy_fullbody_actuo.urdf.xacro"])
    controllers = PathJoinSubstitution(
        [pkg, "config", "jedy_fullbody_controllers.yaml"])
    rviz_config = PathJoinSubstitution([pkg, "rviz", "jedy_fullbody.rviz"])

    wheels = LaunchConfiguration("wheels")
    # ParameterValue(..., value_type=str): a URDF is a string, but launch
    # otherwise tries to parse it as YAML and fails.
    robot_description = {
        "robot_description": ParameterValue(Command([
            "xacro ",
            urdf,
            " backend:=", LaunchConfiguration("backend"),
            " use_sim:=", LaunchConfiguration("use_sim"),
            " wheels:=", wheels,
        ]), value_type=str)
    }

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=NAMESPACE,
        parameters=[robot_description, controllers],
        output="screen",
    )

    # Only the 18 position joints go through joint_state_broadcaster, so the
    # wheels and the two mimic fingers would have no TF and their links would
    # collapse to the origin in RViz. joint_state_publisher fills the gap: it
    # takes the measured joints from the broadcaster (source_list) and derives
    # or zeroes the rest, publishing the merged set that robot_state_publisher
    # consumes for full TF.
    #
    # IT RUNS IN ITS OWN SUB-NAMESPACE, and that is the only reason it works.
    # The obvious construction is to leave it in NAMESPACE and remap its output
    # with `joint_states:=<NAMESPACE>/joint_states_full`. That remap is not
    # output-only: a remap rule matches by RESOLVED name, and this node's
    # source_list subscription to "<NAMESPACE>/joint_states" resolves to the
    # same name as its publisher. So the rule catches the subscription too, the
    # node subscribes to its own output instead of the broadcaster's, and it
    # publishes a robot frozen at the zero pose forever. Nothing errors:
    # /joint_states carries the real measurement the whole time, TF quietly
    # disagrees with it, and RViz shows a robot that never moves.
    #
    # Diagnosed by `ros2 topic info <NAMESPACE>/joint_states --verbose`
    # reporting "Subscription count: 0" while the broadcaster published.
    #
    # A sub-namespace resolves the publisher to <NAMESPACE>/state_merge/
    # joint_states, leaves the absolute source_list name alone, and needs no
    # remap rule on this node at all.
    merged_topic = "/%s/state_merge/joint_states" % NAMESPACE
    joint_state_pub = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        namespace="%s/state_merge" % NAMESPACE,
        parameters=[
            {"source_list": ["/%s/joint_states" % NAMESPACE], "rate": 30},
        ],
        # This node takes the URDF from the robot_description TOPIC, not from a
        # parameter of that name, and the sub-namespace above puts the topic it
        # would subscribe to out of reach of the one robot_state_publisher
        # latches. Without this it sits at
        # "Waiting for robot_description to be published" and never emits a
        # single message. Remapping the input is safe where remapping the
        # output was not: no publisher of this node resolves to that name.
        remappings=[("robot_description", "/%s/robot_description" % NAMESPACE)],
        output="screen",
    )
    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=NAMESPACE,
        parameters=[robot_description],
        remappings=[("joint_states", merged_topic)],
        output="screen",
    )

    def spawner(name, condition=None, extra=()):
        # --service-call-timeout: the default 10 s can run out for
        # load_controller on a loaded PC; the spawner then retries, gets
        # "already loaded" and exits, leaving the controller unconfigured.
        return Node(package="controller_manager", executable="spawner",
                    arguments=[name, "-c", "/%s/controller_manager" % NAMESPACE,
                               "--service-call-timeout", "60",
                               *extra],
                    condition=condition,
                    output="screen")

    controller_nodes = [
        spawner("joint_state_broadcaster"),
        spawner("head_controller"),
        spawner("right_arm_controller"),
        spawner("left_arm_controller"),
        spawner("right_gripper_controller"),
        spawner("left_gripper_controller"),
        # Claims four velocity interfaces that only exist when the URDF was
        # expanded with wheels:=true, so it is spawned under the same flag.
        # Both halves must stay on the same argument: the URDF decides whether
        # the interfaces exist at all, and a spawner for a controller whose
        # interfaces are missing fails loudly rather than coming up half
        # configured.
        spawner("mecanum_drive_controller", condition=IfCondition(wheels),
                extra=[
                    # The controller subscribes to <its name>/reference; remap
                    # it to the name every teleop tool already publishes. The
                    # type is TwistStamped, not Twist -- teleop_twist_joy and
                    # `teleop_twist_keyboard --ros-args -p stamped:=true` both
                    # send that, but a bare
                    # `ros2 topic pub ... geometry_msgs/Twist` will silently
                    # not connect.
                    "--controller-ros-args",
                    "-r /%s/mecanum_drive_controller/reference:=/cmd_vel"
                    % NAMESPACE,
                ]),
    ]

    # Wheel odometry. A separate node and not the controller's own, because
    # mecanum_drive_controller integrates a wheel VELOCITY state interface and
    # this base has none to give: the driver publishes NaN there deliberately
    # and the controller correctly skips its integration.
    odometry = Node(
        package="jedy_bringup", executable="jedy_wheel_odometry",
        namespace=NAMESPACE,
        condition=IfCondition(wheels),
        output="screen",
    )

    # Links of servos that do not answer blink red in RViz, with the bus's
    # reason (actuo_presence_markers, fed by /actuo_bus_servicer/servo/state).
    # Without it, an unplugged servo leaves its link drawn where it was, which
    # looks exactly like a healthy robot standing still.
    presence = Node(
        package="actuo_ros", executable="actuo_presence_markers",
        remappings=[("robot_description", "/%s/robot_description" % NAMESPACE)],
        output="screen",
    )

    rviz = Node(
        package="rviz2", executable="rviz2", name="rviz2",
        arguments=["-d", rviz_config],
        condition=IfCondition(LaunchConfiguration("rviz")),
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="false",
                              description="launch RViz2 with the Jedy config"),
        DeclareLaunchArgument("backend", default_value="actuo",
                              choices=["actuo", "mock"],
                              description="actuo = ActuoSystem backend; "
                                          "mock = GenericSystem backend"),
        DeclareLaunchArgument("use_sim", default_value="false",
                              choices=["true", "false"],
                              description="passed to ActuoSystem. true = sim "
                                          "bus servicer, false = real bus "
                                          "servicer"),
        DeclareLaunchArgument("wheels", default_value="false",
                              choices=["true", "false"],
                              description="true = expose and drive the four "
                                          "mecanum wheels"),
        control_node, robot_state_pub, joint_state_pub, *controller_nodes,
        odometry, presence, rviz,
    ])
