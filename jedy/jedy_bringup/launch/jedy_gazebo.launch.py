"""Jedy's whole body in Gazebo Harmonic, with the real robot's ROS interface.

    ros2 launch jedy_bringup jedy_gazebo.launch.py
    ros2 launch jedy_bringup jedy_gazebo.launch.py gui:=false            # headless
    ros2 launch jedy_bringup jedy_gazebo.launch.py render_sensors:=false # no GL at all
    ros2 launch jedy_bringup jedy_gazebo.launch.py box:=true   # a red box to track
    ros2 launch jedy_bringup jedy_gazebo.launch.py person:=true  # a person (jedy_vision)
    ros2 launch jedy_bringup jedy_gazebo.launch.py demo:=true camera:=d405  # demo stage
    ros2 launch jedy_bringup jedy_gazebo.launch.py tags:=true  # 10 AprilTags around
    ros2 launch jedy_bringup jedy_gazebo.launch.py world:=empty   # no room, an endless floor
    ros2 launch jedy_bringup jedy_gazebo.launch.py legacy:=false  # no 2025 names, no ball

By default the robot stands in Room 13BC, the exercise room
(room13bc_description), at its jedy_spot.

The point of this launch is that a client cannot tell it from
jedy_fullbody_all.launch.py by topic names. It loads the SAME
jedy_fullbody_controllers.yaml into gz_ros2_control under the SAME namespace,
through the SAME <ros2_control> joint list (jedy_fullbody_actuo.urdf.xacro with
backend:=gazebo), so these are identical on both:

    /jedy_fullbody/joint_states
    /jedy_fullbody/{head,right_arm,left_arm,right_gripper,left_gripper}_controller/follow_joint_trajectory
    /cmd_vel                         geometry_msgs/TwistStamped -> mecanum_drive_controller

What the simulator has that the board does not, and the other way round:

* /imu comes from Gazebo's IMU sensor. The RCB-4mini has no IMU; on the real
  robot /imu exists only if a sensor board publishes it.
* /clock: every node here runs on sim time. jedy_skrobot's interface notices
  /clock and switches itself to sim time, so client code needs no flag. It
  ticks at 50 Hz (every 4th physics step; clock_every:=1 for all 200).
* Servo ON/OFF: jedy_sim_servo serves the real robot's
  /actuo_bus_servicer/servo/set with the same rules, and the jedy_servo_switch
  Gazebo plugin makes a released joint stop following its controller, so it
  falls under gravity and joint_states shows where it fell.
* The head's RGB-D camera needs rendering. render_sensors:=false drops it
  (and the Sensors system) for machines with no usable GL; the IMU still runs.
  There is no LiDAR: the exercises do not use one, and rendering it slowed
  the simulation.

legacy:=true (the default) also serves the 2025 names, for the robot system
lecture and other programs written for the 2025 simulation: jedy_legacy_bridge
maps /joint_states, /mecanum_drive_controller/reference and
/{head,rarm,larm}_controller/{joint_trajectory,follow_joint_trajectory} onto
the ones above, and ball:= (default: legacy) puts back the 2025 world's red
ball.
"""

import math
import os
import struct
import sys
import tempfile
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, OpaqueFunction,
    SetEnvironmentVariable, Shutdown)
from launch.conditions import IfCondition
from launch.substitutions import (
    Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from jedy_preflight import preflight_action  # noqa: E402

NAMESPACE = "jedy_fullbody"

#: The pose the robot is spawned in [rad]: scikit-robot's Jedy.reset_pose()
#: (as the 2025 EusLisp :reset-pose), upper arms a little back, forearms
#: forward; the hands are 0.087 m above the floor. So the first
#: ``robot.reset_pose()`` a program sends does not move the arms at all.
#:
#: Not the arms-folded-back pose (TUCK_POSE's old joint0 130 deg) the robot
#: used to spawn in: from it, joint-space interpolation to the reset pose
#: swings the upper arm down through horizontal with the forearm pointing at
#: the floor, and the hand met the floor at joint0 91.7 deg and stayed there
#: (0.48 N m cannot push through it; measured 2026-11-13) -- the arms then
#: never left the back. Nor the 2025 simulation's arms-down pose, which put
#: the hands on the floor with the CAD's 25.25 mm wheels. The xacro's
#: all-zero default hangs both arms through the floor.
START_POSE = {
    "rarm_joint0": 0.5236, "rarm_joint3": -2.0944,
    "larm_joint0": -0.5236, "larm_joint3": -2.0944,
}


#: The exercise room (world:=room13bc): room13bc_description's room with its
#: 48 desks. The robot always spawns at the world origin facing +x, and the
#: props (box, checkerboard, person, demo stage) are placed relative to that,
#: so the ROOM is what gets placed: such that its jedy_spot comes to the
#: origin.
ROOM_PACKAGE = "room13bc_description"
ROOM_URDF = "room13bc_scene.urdf"
#: The spot used when the room's URDF has no `jedy_spot` link: the open floor
#: between the wooden cabinet (x = 0) and the first desks (x = 4.3), in the
#: middle of the room's width, facing the desks (+x) [m, m, rad].
ROOM_DEFAULT_SPOT = (2.0, 4.78, 0.0)
ROOM_SPOT_LINK = "jedy_spot"
#: Room collision shapes lying entirely above this height [m] are dropped:
#: Jedy's head is at about 0.47 m (camera 0.296 m above base_link, base_link
#: 0.127 m up), so it can only meet them by raising an arm under a desk. The
#: desk tops and their frames (0.58 m and up, 192 of the desks' 576 shapes)
#: otherwise cost the physics a third of its speed.
ROOM_REACH_HEIGHT = 0.55
#: The room's floor slab: its collision boxes duplicate the world's ground
#: plane (both top out at z = 0), so the wheels would touch two floors.
ROOM_FLOOR_LINK = "room_floor"


def room_spot(root):
    """``(x, y, yaw)`` of the room's jedy_spot link, or the default."""
    for joint in root.findall("joint"):
        if joint.find("child").get("link") == ROOM_SPOT_LINK:
            origin = joint.find("origin")
            xyz = [float(v) for v in origin.get("xyz", "0 0 0").split()]
            rpy = [float(v) for v in origin.get("rpy", "0 0 0").split()]
            return xyz[0], xyz[1], rpy[2]
    return ROOM_DEFAULT_SPOT


def _origin_matrix(origin):
    """4x4 transform (nested lists) of a URDF <origin>, identity if None."""
    xyz, rpy = [0.0] * 3, [0.0] * 3
    if origin is not None:
        xyz = [float(v) for v in origin.get("xyz", "0 0 0").split()]
        rpy = [float(v) for v in origin.get("rpy", "0 0 0").split()]
    cr, sr = math.cos(rpy[0]), math.sin(rpy[0])
    cp, sp = math.cos(rpy[1]), math.sin(rpy[1])
    cy, sy = math.cos(rpy[2]), math.sin(rpy[2])
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, xyz[0]],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, xyz[1]],
            [-sp, cp * sr, cp * cr, xyz[2]],
            [0.0, 0.0, 0.0, 1.0]]


def _matmul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)]
            for i in range(4)]


def _stl_z_range(path, scale_z):
    """``(zmin, zmax)`` of a binary STL's vertices, or None if unreadable."""
    try:
        with open(path, "rb") as f:
            data = f.read()
        n = struct.unpack_from("<I", data, 80)[0]
        if len(data) != 84 + 50 * n:  # ASCII STL
            return None
        zs = [struct.unpack_from("<f", data, 84 + 50 * i + 12 + 12 * v + 8)[0]
              for i in range(n) for v in range(3)]
    except (OSError, struct.error):
        return None
    lo, hi = min(zs) * scale_z, max(zs) * scale_z
    return min(lo, hi), max(lo, hi)


def _collision_z_min(transform, geometry, share):
    """Lowest world z of a collision geometry, or None if unknown.

    Exact for boxes and cylinders; for a mesh, only when it is a binary STL
    and the collision is not rotated about x or y (then its z range is the
    file's z range, scaled and shifted).
    """
    rot = [row[:3] for row in transform[:3]]
    z = transform[2][3]
    if geometry.tag == "box":
        half = [float(v) / 2 for v in geometry.get("size").split()]
        return z - sum(abs(rot[2][k]) * half[k] for k in range(3))
    if geometry.tag == "cylinder":
        radius = float(geometry.get("radius"))
        length = float(geometry.get("length"))
        axis_z = abs(rot[2][2])
        return z - (axis_z * length / 2
                    + radius * math.sqrt(max(0.0, 1 - axis_z ** 2)))
    if geometry.tag == "sphere":
        return z - float(geometry.get("radius"))
    if geometry.tag == "mesh" and abs(rot[2][2] - 1.0) < 1e-6:
        filename = geometry.get("filename", "")
        prefix = "package://%s/" % ROOM_PACKAGE
        if not filename.startswith(prefix):
            return None
        scale = [float(v) for v in geometry.get("scale", "1 1 1").split()]
        zr = _stl_z_range(os.path.join(share, filename[len(prefix):]),
                          scale[2]) if filename.lower().endswith(".stl") else None
        return None if zr is None else z + zr[0]
    return None


def static_room(urdf_path):
    """The room as a static model: the desk tops' hinges fixed, flat.

    Each desk top hangs on a revolute hinge that holds it flat only by
    resting on its limit under gravity -- 48 joints for the physics to solve
    every step, for furniture nobody moves. Fixed at 0 (flat) and the whole
    model static, the room costs the physics nothing.

    A static model's collision shapes still cost the collision check every
    step, though, and the room has about 650 (on an RTX 4090 desktop the
    room ran at real_time_factor 0.25 against the empty floor's 0.52, and
    0.43 with the desks' and floor's collisions removed). So the shapes Jedy
    cannot reach (entirely above ROOM_REACH_HEIGHT) and the floor slab's,
    which the ground plane already does, are removed.

    Returns
    -------
    root : xml.etree.ElementTree.Element
        The modified URDF.
    hinges : int
        How many hinges were fixed.
    dropped : int
        How many collision shapes were removed.
    """
    root = ET.parse(urdf_path).getroot()
    share = os.path.dirname(os.path.dirname(urdf_path))
    hinges = 0
    parent = {}
    for joint in root.findall("joint"):
        if joint.get("type") == "revolute":
            joint.set("type", "fixed")
            for tag in ("limit", "dynamics", "axis"):
                for e in joint.findall(tag):
                    joint.remove(e)
            hinges += 1
        parent[joint.find("child").get("link")] = (
            joint.find("parent").get("link"),
            _origin_matrix(joint.find("origin")))

    def link_in_room(name):
        transform = _origin_matrix(None)
        while name in parent:
            name, joint_tf = parent[name]
            transform = _matmul(joint_tf, transform)
        return transform

    dropped = 0
    for link in root.findall("link"):
        collisions = link.findall("collision")
        if not collisions:
            continue
        link_tf = link_in_room(link.get("name"))
        for collision in collisions:
            if link.get("name") == ROOM_FLOOR_LINK:
                link.remove(collision)
                dropped += 1
                continue
            z_min = _collision_z_min(
                _matmul(link_tf, _origin_matrix(collision.find("origin"))),
                collision.find("geometry")[0], share)
            if z_min is not None and z_min > ROOM_REACH_HEIGHT:
                link.remove(collision)
                dropped += 1
    ET.SubElement(ET.SubElement(root, "gazebo"), "static").text = "true"
    return root, hinges, dropped


def room_nodes(context):
    """Spawn the room (world:=room13bc) so that its spot is at the origin."""
    if LaunchConfiguration("world").perform(context) != "room13bc":
        return []
    share = get_package_share_directory(ROOM_PACKAGE)
    root, _, _ = static_room(os.path.join(share, "urdf", ROOM_URDF))
    sx, sy, syaw = room_spot(root)
    # Room pose such that the spot maps to (0, 0) facing +x: yaw -syaw, and
    # the spot's position rotated by that yaw, negated.
    yaw = -syaw
    x = -(math.cos(yaw) * sx - math.sin(yaw) * sy)
    y = -(math.sin(yaw) * sx + math.cos(yaw) * sy)
    out = tempfile.NamedTemporaryFile("w", prefix="jedy_room13bc_",
                                      suffix=".urdf", delete=False)
    out.write(ET.tostring(root, encoding="unicode"))
    out.close()
    return [Node(
        package="ros_gz_sim", executable="create",
        arguments=["-file", out.name, "-name", "room13bc",
                   "-x", str(x), "-y", str(y), "-Y", str(yaw)],
        output="screen",
    )]


#: A 0.15 x 0.15 x 0.5 m red box, static, for the camera exercise.
RED_BOX_SDF = (
    "<sdf version='1.9'><model name='red_box'><static>true</static>"
    "<link name='link'>"
    "<collision name='c'><geometry><box><size>0.15 0.15 0.5</size></box>"
    "</geometry></collision>"
    "<visual name='v'><geometry><box><size>0.15 0.15 0.5</size></box>"
    "</geometry><material><ambient>0.8 0 0 1</ambient>"
    "<diffuse>0.8 0 0 1</diffuse></material></visual>"
    "</link></model></sdf>")


#: The 2025 world's red ball (empty.world's "sphere"): radius 0.5 m, its
#: centre 1.5 m to the left of the spawn pose, resting on the floor. Static
#: like the box. In the room it is in the open floor: the nearest collision
#: shapes are the end wall 1.5 m behind it and the desks 2.0 m ahead.
RED_BALL_SDF = (
    "<sdf version='1.9'><model name='red_ball'><static>true</static>"
    "<link name='link'>"
    "<collision name='c'><geometry><sphere><radius>0.5</radius></sphere>"
    "</geometry></collision>"
    "<visual name='v'><geometry><sphere><radius>0.5</radius></sphere>"
    "</geometry><material><ambient>1 0 0 1</ambient>"
    "<diffuse>1 0 0 1</diffuse></material></visual>"
    "</link></model></sdf>")
RED_BALL_XYZ = (0.0, 1.5, 0.5)


#: The checkerboard of the 2025 day-1 exercise (checkerboard_detector): 7 x 4
#: inner corners. Squares are 0.04 m here, not the printed board's 0.02 m,
#: so the corners are found from 0.7 m away at 640 x 480.
CHECKERBOARD_COLS, CHECKERBOARD_ROWS, CHECKERBOARD_SQUARE = 8, 5, 0.04
#: Where its centre is: 0.7 m ahead of the spawn pose, 0.30 m above
#: base_link (the head camera is at 0.296), facing the robot.
CHECKERBOARD_X, CHECKERBOARD_HEIGHT = 0.7, 0.30
#: base_link above the ground in Gazebo: the lowest point of the wheels'
#: collision cylinders, which are the CAD's (and where the CAD model puts its
#: base_footprint).
BASE_HEIGHT = 0.11625

#: Gazebo Fuel human models (OpenRobotics) for person:=true, by
#: person_model:= name/version. "Casual female/4" (CC0) stands with arms
#: down and faces the robot at the default person_yaw. Other models face
#: other ways at the same yaw (e.g. "MaleVisitorOnPhone/2", CC-BY 4.0 --
#: credit OpenRobotics if you show it -- is seen in profile), so set
#: person_yaw for them.
PERSON_MODELS_URL = "https://fuel.gazebosim.org/1.0/OpenRobotics/models/"

#: AprilTag 36h11 bit patterns (1 = black), the 8x8 black-bordered square as
#: OpenCV's aruco module draws ids 0-9 (cv2.aruco.generateImageMarker; 7-9
#: generated with OpenCV 5.0.0, which also reproduces 0-6 here). Built from
#: boxes like the checkerboard, so no texture file is needed.
TAG36H11 = {
    0: ["11111111", "11101111", "11001011", "11110101",
        "11110011", "10100011", "10101001", "11111111"],
    1: ["11111111", "10110111", "10100101", "11110011",
        "11100001", "10001011", "11001001", "11111111"],
    2: ["11111111", "11000111", "11101111", "10110111",
        "11111101", "11011011", "10001001", "11111111"],
    3: ["11111111", "11110011", "11011001", "10110101",
        "10001101", "10001111", "10110001", "11111111"],
    4: ["11111111", "11011101", "11111101", "11010111",
        "10000101", "11100001", "11010001", "11111111"],
    5: ["11111111", "11100101", "11000111", "10010101",
        "11001001", "10111001", "11100001", "11111111"],
    6: ["11111111", "10101111", "11011111", "10100011",
        "10110101", "11001011", "10111111", "11111111"],
    7: ["11111111", "11101011", "10001111", "11000101",
        "11101011", "11001111", "11101111", "11111111"],
    8: ["11111111", "10100101", "11010001", "10000011",
        "10001111", "10010101", "11110111", "11111111"],
    9: ["11111111", "11011001", "10111011", "11111011",
        "10010101", "10101101", "10110111", "11111111"],
}
#: tags:=true: ten AprilTags (ids 0-9, 64 mm black square, as printed on A4
#: for the real robot) on thin grey posts on the floor around the robot, each
#: facing the spawn pose, for tag_pose / approach_tag. Not with demo:=true,
#: whose stand and cube use ids 0-6 too. The tags' centres are 0.30 m above
#: the floor: the head camera is at about 0.41 m, so with the head tilted
#: ~10 deg down a tag 1-2 m away is in view. (id, distance [m], bearing
#: [deg], counter-clockwise from the robot's front): spread all round, near
#: and far, inside the open floor between the cabinet and the desks.
ROOM_TAG_SIZE = 0.064
ROOM_TAG_HEIGHT = 0.30
ROOM_TAGS = [
    (0, 1.2, 0), (1, 1.6, 35), (2, 1.3, 75), (3, 1.7, 110), (4, 1.2, 150),
    (5, 1.5, 180), (6, 1.3, 215), (7, 1.7, 250), (8, 1.2, 290), (9, 1.6, 325),
]
#: Edge of the stand's black square [m] (jedy_vision DEMO_TAG_SIZE).
TAG_SIZE = 0.08
#: demo:=true: a stand with tag 0 on its front, for approach_tag/touch_tag
#: (big enough to be read from the start pose), and on it a cube small enough
#: for one gripper (its fingers open to ~6 cm) with a tag on every face, for
#: grasp_tag. x, y: the stand's centre; stand: its height, depth, width.
#: The cube sits near the stand's front edge so the hand reaches it once
#: approach_tag has stopped in front of the stand. Height: the arm can hold
#: its hand level through reach-in and lift only with the cube's centre near
#: base_link's height (0.127 m up), measured by IK over a grid in Gazebo.
TAG_STAND = {"x": 0.85, "y": 0.0, "height": 0.11, "depth": 0.10, "width": 0.14}
#: The cube: edge, tag edge (one cell of white margin all round), mass, the
#: tag id on each face (jedy_vision TAG_CUBE_FACES must agree).
TAG_CUBE = {"edge": 0.045, "tag": 0.036, "mass": 0.05,
            "ids": {"-x": 1, "+y": 2, "+x": 3, "-y": 4, "+z": 5, "-z": 6}}
#: Each face of a cube as (outward normal, the tag's "up" on it). The tag's
#: "right" is then up x normal, as a viewer facing the face sees it -- the
#: same frame OpenCV's aruco gives a detected tag (x right, y up, z out).
CUBE_FACES = {
    "-x": ((-1, 0, 0), (0, 0, 1)), "+x": ((1, 0, 0), (0, 0, 1)),
    "-y": ((0, -1, 0), (0, 0, 1)), "+y": ((0, 1, 0), (0, 0, 1)),
    "+z": ((0, 0, 1), (1, 0, 0)), "-z": ((0, 0, -1), (1, 0, 0)),
}
#: Things YOLO (yolo11s) recognises reliably in Gazebo, around the robot so
#: it has to turn to find them: chair 0.81-0.93, cup 0.35-0.93 by distance.
#: (A Fuel banana was tried and dropped: from afar YOLO called it a bird or a
#: kite.) Fuel models, CC BY 4.0 -- credit OpenRobotics / Google Research.
#: (name, Fuel model, x, y, z, roll, yaw)
DEMO_OBJECTS = [
    ("chair", "OpenRobotics/models/Chair/3", 2.4, 1.6, 0.0, 0.0, -2.3),
    ("cup", "GoogleResearch/models/ACE_Coffee_Mug_Kristen_16_oz_cup/1",
     1.2, 0.9, 0.0, 0.0, 0.0),
]


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def tag_visuals(tag_id, size, center, normal, up, prefix):
    """The black cells of one tag, as SDF <visual> boxes on a model face.

    Parameters
    ----------
    tag_id : int
        A key of :data:`TAG36H11`.
    size : float
        Edge of the black square [m].
    center : tuple of float
        Centre of the tag on the face, in the model frame [m].
    normal, up : tuple of int
        The face's outward normal and the tag's up direction (axis-aligned).
    prefix : str
        Makes the visual names unique within the link.

    Returns
    -------
    str
    """
    rows = TAG36H11[tag_id]
    cell = size / len(rows)
    right = _cross(up, normal)
    visuals = []
    for r, row in enumerate(rows):
        # One box per run of black cells in a row: a tag is ~40 cells and a
        # cube has six, so per-cell boxes would be ~240 visuals.
        c = 0
        while c < len(row):
            if row[c] != "1":
                c += 1
                continue
            end = c
            while end < len(row) and row[end] == "1":
                end += 1
            u = -size / 2 + (c + end) / 2.0 * cell     # along "right"
            v = size / 2 - (r + 0.5) * cell            # along "up"
            pos = [center[k] + 0.001 * normal[k] + u * right[k] + v * up[k]
                   for k in range(3)]
            dims = [0.002 * abs(normal[k]) + (end - c) * cell * abs(right[k])
                    + cell * abs(up[k]) for k in range(3)]
            visuals.append(
                "<visual name='{}_{}_{}'><pose>{:.4f} {:.4f} {:.4f} 0 0 0</pose>"
                "<geometry><box><size>{:.4f} {:.4f} {:.4f}</size></box></geometry>"
                "<material><ambient>0 0 0 1</ambient><diffuse>0 0 0 1</diffuse>"
                "</material></visual>".format(prefix, r, c, *(pos + dims)))
            c = end
    return "".join(visuals)


def _white(name, pose, size):
    # Partly self-lit: Gazebo's one overhead light leaves a vertical face in
    # shadow, dark grey, and the cube's front tag was then found in 0 of 10
    # images. Paper in a room is lit from every side; this stands for that.
    return ("<visual name='{}'><pose>{:.4f} {:.4f} {:.4f} 0 0 0</pose><geometry>"
            "<box><size>{:.4f} {:.4f} {:.4f}</size></box></geometry><material>"
            "<ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse>"
            "<emissive>0.5 0.5 0.5 1</emissive></material>"
            "</visual>".format(name, *(tuple(pose) + tuple(size))))


def tag_cube_sdf(edge, tag, mass, ids):
    """A white cube with a different AprilTag on each of its six faces.

    Not static: it is there to be picked up. Whichever faces the camera sees,
    each tag gives the cube's centre (half an edge behind the tag) and its
    full orientation (which face it is, from the id).
    """
    tags = "".join(
        tag_visuals(ids[face], tag,
                    tuple(edge / 2 * n for n in CUBE_FACES[face][0]),
                    CUBE_FACES[face][0], CUBE_FACES[face][1], "t" + str(ids[face]))
        for face in CUBE_FACES)
    inertia = mass * edge * edge / 6.0
    return ("<sdf version='1.9'><model name='tag_cube'><link name='link'>"
            "<inertial><mass>{m}</mass><inertia><ixx>{i}</ixx><iyy>{i}</iyy>"
            "<izz>{i}</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>"
            "<collision name='c'><geometry><box><size>{e} {e} {e}</size></box>"
            "</geometry><surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode>"
            "</friction></surface></collision>"
            "{body}{tags}</link></model></sdf>").format(
                m=mass, i=inertia, e=edge, body=_white("v", (0, 0, 0), (edge,) * 3),
                tags=tags)


def stand_sdf(height, depth, width, tag_id=0, size=TAG_SIZE):
    """A grey static stand, tag ``tag_id`` on a white plate on its -x face."""
    plate = size + 2 * size / 8          # one cell of white margin
    z = height / 2 - 0.005 - plate / 2   # near the top of the face
    face = -depth / 2 - 0.001
    return ("<sdf version='1.9'><model name='stand'><static>true</static>"
            "<link name='link'><collision name='c'><geometry><box>"
            "<size>{d} {w} {h}</size></box></geometry></collision>"
            "<visual name='v'><geometry><box><size>{d} {w} {h}</size></box>"
            "</geometry><material><ambient>0.4 0.4 0.45 1</ambient>"
            "<diffuse>0.4 0.4 0.45 1</diffuse></material></visual>"
            "{plate}{tag}</link></model></sdf>").format(
                d=depth, w=width, h=height,
                plate=_white("plate", (face, 0, z), (0.002, plate, plate)),
                tag=tag_visuals(tag_id, size, (face - 0.001, 0, z), (-1, 0, 0),
                                (0, 0, 1), "t"))


def tag_post_sdf(tag_id, size=ROOM_TAG_SIZE, center_height=ROOM_TAG_HEIGHT):
    """A thin grey static post, tag ``tag_id`` on a white plate on its -x face.

    The model's origin is on the floor under the post's centre; spawn it with
    a yaw that turns the -x face towards whatever should see the tag.
    """
    depth, width = 0.03, size + 2 * size / 8 + 0.02
    height = center_height + size / 2 + size / 8 + 0.02
    plate = size + 2 * size / 8          # one cell of white margin
    face = -depth / 2 - 0.001
    return ("<sdf version='1.9'><model name='tag_post_{i}'><static>true</static>"
            "<link name='link'><collision name='c'><pose>0 0 {hz} 0 0 0</pose>"
            "<geometry><box><size>{d} {w} {h}</size></box></geometry></collision>"
            "<visual name='v'><pose>0 0 {hz} 0 0 0</pose><geometry><box>"
            "<size>{d} {w} {h}</size></box></geometry><material>"
            "<ambient>0.45 0.45 0.5 1</ambient><diffuse>0.45 0.45 0.5 1</diffuse>"
            "</material></visual>{plate}{tag}</link></model></sdf>").format(
                i=tag_id, d=depth, w=width, h=height, hz=height / 2,
                plate=_white("plate", (face, 0, center_height), (0.002, plate, plate)),
                tag=tag_visuals(tag_id, size, (face - 0.001, 0, center_height),
                                (-1, 0, 0), (0, 0, 1), "t"))


def room_tag_nodes(context):
    """tags:=true: the ROOM_TAGS posts, each facing the spawn pose."""
    if LaunchConfiguration("tags").perform(context) != "true":
        return []
    nodes = []
    for tag_id, distance, bearing in ROOM_TAGS:
        a = math.radians(bearing)
        x, y = distance * math.cos(a), distance * math.sin(a)
        # The -x face (the tag) points at the origin: yaw = atan2(y, x).
        nodes.append(Node(
            package="ros_gz_sim", executable="create",
            arguments=["-name", "tag_post_%d" % tag_id,
                       "-string", tag_post_sdf(tag_id),
                       "-x", "%.4f" % x, "-y", "%.4f" % y, "-z", "0",
                       "-Y", "%.4f" % math.atan2(y, x)],
            output="screen"))
    return nodes


def checkerboard_sdf(cols=CHECKERBOARD_COLS, rows=CHECKERBOARD_ROWS,
                     square=CHECKERBOARD_SQUARE):
    """A static checkerboard in the model's y-z plane, seen from -x.

    A white plate with one square of margin, and a thin black box in front of
    it for each black square: geometry only, no texture file to find.

    Returns
    -------
    str
        SDF.
    """
    margin = square
    width, height = cols * square + 2 * margin, rows * square + 2 * margin
    visuals = [
        "<visual name='plate'><pose>0 0 0 0 0 0</pose><geometry><box>"
        "<size>0.01 {:.4f} {:.4f}</size></box></geometry><material>"
        "<ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse></material>"
        "</visual>".format(width, height)]
    for i in range(cols):
        for j in range(rows):
            if (i + j) % 2:
                continue
            y = (i - (cols - 1) / 2.0) * square
            z = (j - (rows - 1) / 2.0) * square
            visuals.append(
                "<visual name='b{}_{}'><pose>-0.006 {:.4f} {:.4f} 0 0 0</pose>"
                "<geometry><box><size>0.002 {:.4f} {:.4f}</size></box>"
                "</geometry><material><ambient>0 0 0 1</ambient>"
                "<diffuse>0 0 0 1</diffuse></material></visual>".format(
                    i, j, y, z, square, square))
    return (
        "<sdf version='1.9'><model name='checkerboard'><static>true</static>"
        "<link name='link'><collision name='c'><geometry><box>"
        "<size>0.01 {:.4f} {:.4f}</size></box></geometry></collision>"
        "{}</link></model></sdf>".format(width, height, "".join(visuals)))


def gz_sim_process(context, world):
    """``gz sim`` as launch's own child in its own process group.

    Gazebo stops in about 2 s on ONE SIGINT. Two ways it did not:

    * Stopped with ``kill -INT <launch pid>`` (scripts, ``timeout``):
      ros_gz_sim's gz_sim.launch.py runs Gazebo through a shell (``/bin/sh -c
      ruby gz sim ...``); launch's SIGINT went to the shell, which waits on
      Gazebo and did not pass it on; 5 s later launch's SIGTERM ended only the
      shell, and Gazebo was left running with parent pid 1, still stepping the
      physics. So no shell here.
    * Ctrl-C in the terminal: the terminal sends SIGINT to the whole process
      group, and launch sends Gazebo another one; with two, Gazebo did not
      stop and launch killed it 15 s later (SIGKILL). ``setsid`` puts Gazebo
      in a process group of its own, so only launch's SIGINT reaches it.
      Launch's child is not a group leader, so setsid execs in place: the pid
      launch watches is Gazebo's.

    Measured on an RTX 4090 desktop, old and new: the old launch left Gazebo
    behind after ``kill -INT``; this one stops it in 2 s either way.

    The rest is what gz_sim.launch.py does: Gazebo finds system plugins
    (gz_ros2_control, jedy_sim's) on GZ_SIM_SYSTEM_PLUGIN_PATH, which gets
    LD_LIBRARY_PATH.
    """
    gui = LaunchConfiguration("gui").perform(context) == "true"
    # -s: server only. --headless-rendering lets the server render sensors
    # through EGL without a display.
    cmd = ["setsid", "gz", "sim", "-r"]
    if not gui:
        cmd += ["-s", "--headless-rendering"]
    cmd.append(world.perform(context))
    plugin_path = os.pathsep.join(
        p for p in (os.environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH", ""),
                    os.environ.get("LD_LIBRARY_PATH", "")) if p)
    return [ExecuteProcess(
        cmd=cmd, name="gazebo", output="screen",
        additional_env={"GZ_SIM_SYSTEM_PLUGIN_PATH": plugin_path},
        on_exit=Shutdown(),
    )]


def generate_launch_description():
    """Build the launch description.

    Returns
    -------
    launch.LaunchDescription
        Gazebo, the robot, its controllers and the sensor bridges.
    """
    bringup = get_package_share_directory("jedy_bringup")
    description = get_package_share_directory("jedy_description")
    urdf = os.path.join(bringup, "urdf", "jedy_fullbody_actuo.urdf.xacro")
    controllers = os.path.join(bringup, "config",
                               "jedy_fullbody_controllers.yaml")
    worlds = os.path.join(bringup, "worlds")

    render = LaunchConfiguration("render_sensors")
    wheels = LaunchConfiguration("wheels")
    use_sim_time = {"use_sim_time": True}

    # package://jedy_description/... in the URDF resolves through this: gz
    # rewrites package:// to model:// and looks under each resource path.
    # The room's meshes (package://room13bc_description/...) likewise.
    shares = [os.path.dirname(description)]
    try:
        shares.append(os.path.dirname(get_package_share_directory(ROOM_PACKAGE)))
    except Exception:  # not installed: world:=room13bc fails in room_nodes
        pass
    resource_path = os.pathsep.join(
        shares + [p for p in os.environ.get("GZ_SIM_RESOURCE_PATH", "").split(
            os.pathsep) if p])

    # jedy_<empty|room>[_no_render].sdf
    world = PythonExpression([
        "'", worlds, "/jedy_' + ('room' if '", LaunchConfiguration("world"),
        "' == 'room13bc' else 'empty') + ('.sdf' if '", render,
        "' == 'true' else '_no_render.sdf')"])
    gazebo = OpaqueFunction(function=gz_sim_process, args=[world])

    robot_description = {
        "robot_description": ParameterValue(Command([
            "xacro ", urdf,
            " backend:=gazebo",
            " wheels:=", wheels,
            " camera_model:=", LaunchConfiguration("camera"),
            " camera_rate:=", LaunchConfiguration("camera_rate"),
            " controllers:=", controllers,
        ] + [" init_%s:=%r" % item for item in START_POSE.items()]),
            value_type=str)
    }

    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=NAMESPACE,
        parameters=[robot_description, use_sim_time],
        remappings=[("joint_states",
                     "/%s/state_merge/joint_states" % NAMESPACE)],
        output="screen",
    )
    # Same merge as jedy_fullbody.launch.py: the mimic gripper joints and the
    # wheels are not on /joint_states (joint_state_broadcaster lists the 18
    # position joints only), so this fills them for TF.
    joint_state_pub = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        namespace="%s/state_merge" % NAMESPACE,
        parameters=[{"source_list": ["/%s/joint_states" % NAMESPACE],
                     "rate": 30}, use_sim_time],
        remappings=[("robot_description",
                     "/%s/robot_description" % NAMESPACE)],
        output="screen",
    )

    spawn = Node(
        package="ros_gz_sim", executable="create",
        arguments=["-topic", "/%s/robot_description" % NAMESPACE,
                   "-name", "jedy", "-z", "0.2"],
        output="screen",
    )

    bridge_args = ["/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
                   # Servo OFF: jedy_sim_servo <-> the jedy_servo_switch plugin.
                   "/%s/servo/released@std_msgs/msg/String]gz.msgs.StringMsg"
                   % NAMESPACE,
                   "/%s/servo/released_state@std_msgs/msg/String"
                   "[gz.msgs.StringMsg" % NAMESPACE]
    bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge",
        arguments=bridge_args, output="screen",
    )
    # /clock, thinned. Gazebo publishes it every physics step (200 Hz), and
    # every rclpy node on sim time takes each one in Python: a connected
    # jedy_skrobot program then runs YOLO 0.20 s per image instead of 0.04
    # (the GIL; 0.05 without /clock). Timers and ri.now() do not need 5 ms;
    # gz_ros2_control steps the controllers from Gazebo itself, not from
    # /clock. clock_every:=1 passes every tick.
    clock_bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge",
        name="clock_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        remappings=[("/clock", "/clock_gz")],
        output="screen",
    )
    clock_every = LaunchConfiguration("clock_every")
    clock_drop = Node(
        package="topic_tools", executable="drop", name="clock_drop",
        # Drop (every - 1) of every `every` messages: keep one.
        arguments=["/clock_gz",
                   PythonExpression([clock_every, " - 1"]), clock_every,
                   "/clock"],
        output="screen",
    )
    render_bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge",
        name="render_sensor_bridge",
        arguments=[
            "/camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
        ],
        remappings=[
            ("/camera/image", "/camera/color/image_rect_raw"),
            ("/camera/camera_info", "/camera/color/camera_info"),
            ("/camera/depth_image", "/camera/depth/image_rect_raw"),
        ],
        condition=IfCondition(render),
        output="screen",
    )

    # The target of the camera exercise (jedy_skrobot track_box): a red box
    # ahead and to the left, inside the head camera's 60 deg field of view
    # from the spawn pose, so the robot has to turn left to face it.
    box = Node(
        package="ros_gz_sim", executable="create",
        arguments=["-name", "red_box", "-string", RED_BOX_SDF,
                   "-x", "1.2", "-y", "0.5", "-z", "0.25"],
        condition=IfCondition(LaunchConfiguration("box")),
        output="screen",
    )

    # legacy:=true: the 2025 red ball, for the camshift exercises.
    ball = Node(
        package="ros_gz_sim", executable="create",
        arguments=["-name", "red_ball", "-string", RED_BALL_SDF,
                   "-x", str(RED_BALL_XYZ[0]), "-y", str(RED_BALL_XYZ[1]),
                   "-z", str(RED_BALL_XYZ[2])],
        condition=IfCondition(LaunchConfiguration("ball")),
        output="screen",
    )

    # The day-1 checkerboard (jedy_skrobot checkerboard_pose).
    checkerboard = Node(
        package="ros_gz_sim", executable="create",
        arguments=["-name", "checkerboard", "-string", checkerboard_sdf(),
                   "-x", str(CHECKERBOARD_X), "-y", "0.0",
                   "-z", str(BASE_HEIGHT + CHECKERBOARD_HEIGHT)],
        condition=IfCondition(LaunchConfiguration("checkerboard")),
        output="screen",
    )

    # A person for the jedy_vision demos (follow_person, look_at_person,
    # mimic_arms): a CC0 human model from Gazebo Fuel, pinned by version and
    # fetched on first use into ~/.gz/fuel (staff can prefetch with
    # `gz fuel download -u <url>`). PERSON_X ahead so the 0.3 m-high head
    # camera sees the whole body; a little to the left so the robot has to
    # turn towards them.
    person = Node(
        package="ros_gz_sim", executable="create",
        arguments=["-name", "person", "-file",
                   PythonExpression(["'", PERSON_MODELS_URL, "' + '",
                                     LaunchConfiguration("person_model"),
                                     "'"]),
                   "-x", LaunchConfiguration("person_x"),
                   "-y", LaunchConfiguration("person_y"),
                   "-z", "0.0", "-Y", LaunchConfiguration("person_yaw")],
        condition=IfCondition(LaunchConfiguration("person")),
        output="screen",
    )

    # demo:=true: the jedy_vision demo stage (tag_pose, approach_tag,
    # touch_tag, grasp_tag, find_object).
    demo_on = IfCondition(LaunchConfiguration("demo"))
    st, cube = TAG_STAND, TAG_CUBE
    demo_nodes = [
        Node(package="ros_gz_sim", executable="create",
             arguments=["-name", "stand", "-string",
                        stand_sdf(st["height"], st["depth"], st["width"]),
                        "-x", str(st["x"]), "-y", str(st["y"]),
                        "-z", str(st["height"] / 2)],
             condition=demo_on, output="screen"),
        # Near the stand's front edge (1 cm in), dropped from 2 mm up.
        Node(package="ros_gz_sim", executable="create",
             arguments=["-name", "tag_cube", "-string",
                        tag_cube_sdf(cube["edge"], cube["tag"], cube["mass"], cube["ids"]),
                        "-x", str(st["x"] - st["depth"] / 2 + 0.01 + cube["edge"] / 2),
                        "-y", str(st["y"]),
                        "-z", str(st["height"] + cube["edge"] / 2 + 0.002)],
             condition=demo_on, output="screen"),
    ] + [
        Node(package="ros_gz_sim", executable="create",
             arguments=["-name", name, "-file",
                        "https://fuel.gazebosim.org/1.0/" + uri,
                        "-x", str(x), "-y", str(y), "-z", str(z),
                        "-R", str(roll), "-Y", str(yaw)],
             condition=demo_on, output="screen")
        for name, uri, x, y, z, roll, yaw in DEMO_OBJECTS
    ]

    def spawner(name, condition=None, extra=()):
        # --controller-manager-timeout: gz_ros2_control only starts the
        # controller manager once the model is spawned and robot_description
        # has been read, which on a cold Gazebo is tens of seconds.
        # --service-call-timeout: the default 10 s ran out for
        # load_controller on a loaded machine; the spawner then retried, got
        # "already loaded" and exited, leaving the controller unconfigured.
        # --switch-timeout: the default 5 s for activating it ran out while
        # the room was still loading (2 launches out of 2: "Switch
        # controller timed out after 5 seconds!", right_arm_controller left
        # inactive).
        return Node(package="controller_manager", executable="spawner",
                    arguments=[name, "-c", "/%s/controller_manager" % NAMESPACE,
                               "--controller-manager-timeout", "120",
                               "--service-call-timeout", "60",
                               "--switch-timeout", "60",
                               *extra],
                    condition=condition, output="screen")

    controller_nodes = [
        spawner("joint_state_broadcaster"),
        spawner("head_controller"),
        spawner("right_arm_controller"),
        spawner("left_arm_controller"),
        spawner("right_gripper_controller"),
        spawner("left_gripper_controller"),
        spawner("mecanum_drive_controller", condition=IfCondition(wheels),
                extra=[
                    "--controller-ros-args",
                    "-r /%s/mecanum_drive_controller/reference:=/cmd_vel"
                    % NAMESPACE,
                ]),
    ]

    # The real robot's servo switch (/actuo_bus_servicer/servo/set), so
    # ri.servo_off() releases joints here too.
    servo = Node(
        package="jedy_sim", executable="jedy_sim_servo",
        arguments=[["--missing=", LaunchConfiguration("missing_servos")]],
        parameters=[use_sim_time], output="screen",
    )

    # Links of servos that do not answer blink red in RViz, with the bus's
    # reason (actuo_presence_markers, fed by /actuo_bus_servicer/servo/state).
    # Without it, an unplugged servo leaves its link drawn where it was, which
    # looks exactly like a healthy robot standing still.
    presence = Node(
        package="actuo_ros", executable="actuo_presence_markers",
        remappings=[("robot_description", "/%s/robot_description" % NAMESPACE)],
        # Sim time like RViz here: stamped in wall time, the markers lie
        # "in the future" for TF and RViz silently draws nothing.
        parameters=[use_sim_time],
        output="screen",
    )

    # Motor sounds through the PC's speaker (sounds:=true): the simulator is
    # silent, the real robot is not. Its own sound_play, so that the effects
    # neither queue behind nor get mistaken for the robot's speech
    # (VOICEVOX's sound_play is robotsound_jp).
    sounds = IfCondition(LaunchConfiguration("sounds"))
    sound_play = Node(
        package="sound_play", executable="soundplay_node.py",
        name="jedy_sound_effects",
        remappings=[("robotsound", "/jedy_sim/sound_effects")],
        condition=sounds, output="screen",
    )
    sim_sounds = Node(
        package="jedy_sim", executable="jedy_sim_sounds",
        parameters=[{"sound_topic": "/jedy_sim/sound_effects"}],
        condition=sounds, output="screen",
    )

    # legacy:=true: the 2025 topic and action names, on top of the ones
    # above (jedy_legacy_bridge's docstring has the mapping).
    legacy_bridge = Node(
        package="jedy_bringup", executable="jedy_legacy_bridge",
        parameters=[{"namespace": NAMESPACE}, use_sim_time],
        condition=IfCondition(LaunchConfiguration("legacy")),
        output="screen",
    )

    rviz = Node(
        package="rviz2", executable="rviz2", name="rviz2",
        arguments=["-d", PathJoinSubstitution(
            [bringup, "rviz", "jedy_fullbody.rviz"])],
        parameters=[use_sim_time],
        condition=IfCondition(LaunchConfiguration("rviz")),
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("world", default_value="room13bc",
                              choices=["room13bc", "empty"],
                              description="room13bc = the exercise room "
                                          "(room13bc_description), the robot "
                                          "at its jedy_spot; empty = an "
                                          "endless floor"),
        DeclareLaunchArgument("gui", default_value="true",
                              choices=["true", "false"],
                              description="false = Gazebo server only"),
        DeclareLaunchArgument("render_sensors", default_value="true",
                              choices=["true", "false"],
                              description="false = no camera, for "
                                          "machines without usable GL"),
        DeclareLaunchArgument("wheels", default_value="true",
                              choices=["true", "false"],
                              description="drive the mecanum base from "
                                          "/cmd_vel"),
        DeclareLaunchArgument("checkerboard", default_value="false",
                              choices=["true", "false"],
                              description="the day-1 checkerboard, 0.7 m "
                                          "ahead (checkerboard_pose)"),
        DeclareLaunchArgument("box", default_value="false",
                              choices=["true", "false"],
                              description="a red box for the camera "
                                          "exercise (track_box)"),
        DeclareLaunchArgument("tags", default_value="false",
                              choices=["true", "false"],
                              description="ten 64 mm AprilTags (ids 0-9) on "
                                          "posts around the robot (tag_pose, "
                                          "approach_tag); not with demo:=true"),
        DeclareLaunchArgument("legacy", default_value="true",
                              choices=["true", "false"],
                              description="also serve the 2025 topic/action "
                                          "names (/joint_states, "
                                          "/mecanum_drive_controller/"
                                          "reference, /{head,rarm,larm}_"
                                          "controller/...) for the robot "
                                          "system lecture and older "
                                          "programs"),
        # Default: legacy, except with tags:=true, whose posts 2 and 3
        # stand where the ball would be. Declared after tags, which it reads.
        DeclareLaunchArgument("ball", default_value=PythonExpression([
            "'true' if '", LaunchConfiguration("legacy"), "' == 'true' and '",
            LaunchConfiguration("tags"), "' != 'true' else 'false'"]),
                              choices=["true", "false"],
                              description="the 2025 world's red ball (r 0.5 "
                                          "m, 1.5 m to the left); default: "
                                          "legacy, but false with tags:=true"),
        DeclareLaunchArgument("clock_every", default_value="4",
                              description="publish /clock every N physics "
                                          "steps (5 ms each): 4 = 50 Hz"),
        DeclareLaunchArgument("rviz", default_value="false",
                              choices=["true", "false"]),
        DeclareLaunchArgument("demo", default_value="false",
                              choices=["true", "false"],
                              description="the jedy_vision demo stage: an "
                                          "AprilTag box on a stand, a chair "
                                          "and a cup (Fuel)"),
        DeclareLaunchArgument("camera", default_value="wide",
                              choices=["wide", "d405"],
                              description="d405 = the real robot's camera: "
                                          "87 deg, depth only 0.07-0.5 m"),
        DeclareLaunchArgument("camera_rate", default_value="5",
                              description="head camera frame rate [Hz]; "
                                          "rendering it is most of Gazebo's "
                                          "CPU (5 Hz: 77%, 30 Hz: 123% on "
                                          "one PC)"),
        DeclareLaunchArgument("person", default_value="false",
                              choices=["true", "false"],
                              description="a person for the jedy_vision "
                                          "demos (Gazebo Fuel model, "
                                          "downloaded on first use)"),
        DeclareLaunchArgument("person_model", default_value="Casual female/4",
                              description="Fuel model under OpenRobotics, "
                                          "'name/version'"),
        DeclareLaunchArgument("person_x", default_value="3.5",
                              description="[m] ahead of the robot"),
        DeclareLaunchArgument("person_y", default_value="0.6",
                              description="[m] to the robot's left"),
        DeclareLaunchArgument("person_yaw", default_value="-1.5708",
                              description="[rad]; the default makes 'Casual female' face the robot"),
        DeclareLaunchArgument("sounds", default_value="false",
                              choices=["true", "false"],
                              description="motor sounds through the PC's "
                                          "speaker while the base is "
                                          "commanded or a joint moves"),
        DeclareLaunchArgument("missing_servos", default_value="",
                              description="comma-separated joints whose "
                                          "servo does not answer (rehearse "
                                          "a cable out: limp, servo ON "
                                          "refused, red in RViz)"),
        # Fail now, with the PIDs, if a robot or an old Gazebo is running.
        preflight_action(),
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_path),
        gazebo, OpaqueFunction(function=room_nodes),
        OpaqueFunction(function=room_tag_nodes),
        robot_state_pub, joint_state_pub, spawn, bridge, clock_bridge, clock_drop,
        render_bridge, box, ball, checkerboard, person, *demo_nodes,
        *controller_nodes, servo, presence, sound_play, sim_sounds,
        legacy_bridge, rviz,
    ])
