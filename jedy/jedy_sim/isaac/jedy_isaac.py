#!/usr/bin/env python3
"""Isaac Sim side of Jedy: the USD, physics, and its joints on ROS 2.

Run inside the Isaac Sim container (see run_jedy_isaac.sh):

    ./python.sh jedy_isaac.py --usd jedy_fullbody.usdc --out /out

It publishes and subscribes only standard messages, under one namespace:

    /jedy_fullbody/isaac/joint_states    sensor_msgs/JointState   out
    /jedy_fullbody/isaac/joint_command   sensor_msgs/JointState   in
    /clock                               rosgraph_msgs/Clock      out
    /imu                                 sensor_msgs/Imu          out (base_link)
    /camera/color/image_rect_raw         sensor_msgs/Image        out (head camera, rgb8)
    /camera/depth/image_rect_raw         sensor_msgs/Image        out (head camera, 32FC1 m)
    /camera/color/camera_info            sensor_msgs/CameraInfo   out (its intrinsics, as Gazebo's)
    /jedy_fullbody/servo/released        std_msgs/String          in  (servo OFF, from jedy_sim_servo)
    /jedy_fullbody/servo/released_state  std_msgs/String          out (its ack)

The head camera uses the RealSense driver's topic names, as the real robot and
Gazebo do, so jedy_skrobot's ri.image() reads all three. ``--box`` adds the
red box of the camera exercise (track_box), at the same place as Gazebo's.

What is simulated depends on the USD (make_jedy_usd.py --base):

* ``planar`` (``--base planar``): the arm, head and gripper joints under their drives, and the
  base's planar motion. No contact with the floor or anything else (the floor
  is drawn only, see below), and no mecanum roller slip.
* ``wheels`` (default): the robot stands on a floor
  that collides, on velocity-driven wheels with mecanum rollers, so the base
  drives, slips and bumps by physics. The box collides too. Then

    /jedy_fullbody/isaac/ground_truth    nav_msgs/Odometry        out (world -> base_link)

  is the simulator's true base pose, for checking what the base did. It is
  named as ground truth on purpose: the real robot has no such topic.

It is NOT the robot's ROS interface: jedy_isaac_servicer turns these into
actuo's shared memory, so ros2_control and every controller run exactly as on
the real robot (ros2 launch jedy_bringup jedy_isaac.launch.py).

``--out`` gets ``isaac_view.png``, a third-person render rewritten every
``--shot-period`` seconds of sim time, so a headless run leaves a picture.

The OmniGraph node and attribute names are the ones verified against the
examples shipped in the image.
"""

import argparse
import os

parser = argparse.ArgumentParser()
parser.add_argument("--usd", required=True,
                    help="USD from jedy_sim/isaac/make_jedy_usd.py")
parser.add_argument("--robot-prim", default="/robot")
parser.add_argument("--ns", default="jedy_fullbody/isaac",
                    help="namespace of the joint topics (no leading slash)")
parser.add_argument("--out", default="",
                    help="directory for isaac_view.png; empty = no pictures")
parser.add_argument("--shot-period", type=float, default=2.0)
parser.add_argument("--base", choices=["planar", "wheels"], default="wheels",
                    help="which USD this is (make_jedy_usd.py --base)")
parser.add_argument("--friction", type=float, default=1.0,
                    help="floor/roller friction (--base wheels)")
parser.add_argument("--no-camera", action="store_true",
                    help="do not render the head camera (it costs GPU time)")
parser.add_argument("--box", action="store_true",
                    help="the red box of the camera exercise at (1.2, 0.5)")
parser.add_argument("--ground-z", type=float, default=-0.11625,
                    help="height of the floor drawn under the robot [m]; "
                         "base_link is 0.11625 m above the wheel bottoms")
parser.add_argument("--checkerboard", action="store_true",
                    help="the day-1 checkerboard 0.7 m ahead (as Gazebo's "
                         "checkerboard:=true)")
parser.add_argument("--physics-hz", type=float, default=60.0,
                    help="physics steps per second (rendering and the ROS "
                         "graph stay at 60 Hz)")
parser.add_argument("--released-position-iterations", type=int, default=255,
                    help="the articulation's solver position iterations "
                         "while any joint is released (PhysX's maximum is "
                         "255); the USD's own count otherwise. See "
                         "ServoSwitch")
parser.add_argument("--servo-ns", default="/jedy_fullbody/servo",
                    help="topics of the servo switch (jedy_sim_servo)")
args = parser.parse_args()

from isaacsim import SimulationApp  # noqa: E402
simulation_app = SimulationApp({"headless": True,
                                "renderer": "RaytracedLighting"})

from isaacsim.core.utils.extensions import enable_extension  # noqa: E402
enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

import numpy as np  # noqa: E402
import omni.graph.core as og  # noqa: E402
import omni.replicator.core as rep  # noqa: E402
import omni.usd  # noqa: E402
import usdrt.Sdf  # noqa: E402
from isaacsim.core.api import SimulationContext  # noqa: E402
from pxr import Gf, UsdGeom, UsdLux  # noqa: E402

stage = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.Xform.Define(stage, "/World")
robot_path = "/World" + args.robot_prim
stage.DefinePrim(robot_path, "Xform").GetReferences().AddReference(
    args.usd, args.robot_prim)

wheels = args.base == "wheels"
# planar: the floor is DRAWN, not collided with. The base rides position-driven planar
# joints, so nothing needs the floor to hold it up; and with a collider the
# wheels (and an arm swung low) fought those stiff drives until the
# articulation blew up -- joint angles in the hundreds of thousands within
# seconds, measured on the first run. So the robot neither rests on nor bumps
# into anything here: contact is one more thing this simulation does not do.
floor = UsdGeom.Cube.Define(stage, "/World/floor")
floor.CreateSizeAttr(1.0)
floor.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, args.ground_z - 0.005))
floor.AddScaleOp().Set(Gf.Vec3f(20.0, 20.0, 0.01))
floor.CreateDisplayColorPrimvar("constant").Set([Gf.Vec3f(0.7, 0.7, 0.7)])
if wheels:
    # wheels: the floor holds the robot up, and grips the rollers.
    from pxr import UsdPhysics, UsdShade
    material = UsdShade.Material.Define(stage, "/World/Physics/floor_material")
    friction = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    friction.CreateStaticFrictionAttr(args.friction)
    friction.CreateDynamicFrictionAttr(args.friction)
    friction.CreateRestitutionAttr(0.0)
    UsdPhysics.CollisionAPI.Apply(floor.GetPrim())
    UsdShade.MaterialBindingAPI.Apply(floor.GetPrim()).Bind(
        material, UsdShade.Tokens.weakerThanDescendants, "physics")
UsdLux.DistantLight.Define(stage, "/World/Light/Distant").CreateIntensityAttr(
    1500.0)
UsdLux.DomeLight.Define(stage, "/World/Light/Dome").CreateIntensityAttr(600.0)


def look_at(path, eye, target, focal):
    """A world-fixed camera at ``eye`` looking at ``target``."""
    cam = UsdGeom.Camera.Define(stage, path)
    eye, target = Gf.Vec3d(*eye), Gf.Vec3d(*target)
    z = (eye - target).GetNormalized()
    x = Gf.Cross(Gf.Vec3d(0.0, 0.0, 1.0), z).GetNormalized()
    y = Gf.Cross(z, x)
    cam.AddTransformOp().Set(Gf.Matrix4d(
        x[0], x[1], x[2], 0.0, y[0], y[1], y[2], 0.0,
        z[0], z[1], z[2], 0.0, eye[0], eye[1], eye[2], 1.0))
    cam.CreateFocalLengthAttr(focal)
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.01, 1000.0))
    return path


view_cam = look_at("/World/view_camera", (0.9, -0.9, 0.6), (0.1, 0.0, 0.05),
                   18.0)

# The head camera, matching Gazebo's (jedy_gz.xacro): 640x480, 60 deg
# horizontal field of view, mounted on camera_color_frame (x forward, z up).
# A USD camera looks down its -Z with +Y up, hence the rotation.
CAMERA_SIZE = (640, 480)
CAMERA_HFOV = np.deg2rad(60.0)
head_cam = None
if not args.no_camera:
    head_cam = robot_path + "/camera_color_frame/head_camera"
    cam = UsdGeom.Camera.Define(stage, head_cam)
    cam.AddTransformOp().Set(Gf.Matrix4d(
        0.0, -1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
        -1.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 1.0))
    aperture = 20.955
    cam.CreateHorizontalApertureAttr(aperture)
    cam.CreateVerticalApertureAttr(
        aperture * CAMERA_SIZE[1] / CAMERA_SIZE[0])
    cam.CreateFocalLengthAttr(aperture / 2.0 / np.tan(CAMERA_HFOV / 2.0))
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.05, 10.0))

if args.box:
    # Same size and place as jedy_gazebo.launch.py's box:=true. Drawn only
    # like the floor, or a static collider with --base wheels.
    box = UsdGeom.Cube.Define(stage, "/World/red_box")
    box.CreateSizeAttr(1.0)
    box.AddTranslateOp().Set(Gf.Vec3d(1.2, 0.5, args.ground_z + 0.25))
    box.AddScaleOp().Set(Gf.Vec3f(0.15, 0.15, 0.5))
    box.CreateDisplayColorPrimvar("constant").Set([Gf.Vec3f(0.8, 0.0, 0.0)])
    if wheels:
        UsdPhysics.CollisionAPI.Apply(box.GetPrim())

if args.checkerboard:
    # Same board, size and place as jedy_gazebo.launch.py's
    # checkerboard:=true: 8 x 5 squares of 0.04 m (7 x 4 inner corners), a
    # white plate with black boxes in front, 0.7 m ahead, centre 0.30 m above
    # base_link, facing the robot. Geometry only, no texture to find.
    cols, rows, square = 8, 5, 0.04
    centre = Gf.Vec3d(0.7, 0.0, 0.30)
    plate = UsdGeom.Cube.Define(stage, "/World/checkerboard/plate")
    plate.CreateSizeAttr(1.0)
    plate.AddTranslateOp().Set(centre)
    plate.AddScaleOp().Set(Gf.Vec3f(0.01, cols * square + 2 * square,
                                    rows * square + 2 * square))
    plate.CreateDisplayColorPrimvar("constant").Set([Gf.Vec3f(1.0, 1.0, 1.0)])
    if wheels:
        UsdPhysics.CollisionAPI.Apply(plate.GetPrim())
    for i in range(cols):
        for j in range(rows):
            if (i + j) % 2:
                continue
            cell = UsdGeom.Cube.Define(
                stage, "/World/checkerboard/b{}_{}".format(i, j))
            cell.CreateSizeAttr(1.0)
            cell.AddTranslateOp().Set(centre + Gf.Vec3d(
                -0.006, (i - (cols - 1) / 2.0) * square,
                (j - (rows - 1) / 2.0) * square))
            cell.AddScaleOp().Set(Gf.Vec3f(0.002, square, square))
            cell.CreateDisplayColorPrimvar("constant").Set(
                [Gf.Vec3f(0.0, 0.0, 0.0)])

sim = SimulationContext(physics_dt=1.0 / args.physics_hz,
                        rendering_dt=1.0 / 60.0, stage_units_in_meters=1.0)
simulation_app.update()

og.Controller.edit(
    {"graph_path": "/JedyGraph", "evaluator_name": "execution"},
    {
        og.Controller.Keys.CREATE_NODES: [
            ("OnTick", "omni.graph.action.OnPlaybackTick"),
            ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
            ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
            ("SubscribeJointState",
             "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
            ("ArticulationController",
             "isaacsim.core.nodes.IsaacArticulationController"),
        ],
        og.Controller.Keys.CONNECT: [
            ("OnTick.outputs:tick", "PublishClock.inputs:execIn"),
            ("OnTick.outputs:tick", "PublishJointState.inputs:execIn"),
            ("OnTick.outputs:tick", "SubscribeJointState.inputs:execIn"),
            # Apply a command when one ARRIVES, not every tick: the
            # subscriber keeps its last message, and re-applying it every tick
            # put every joint the servicer stopped commanding (a released one)
            # back on its pre-release target -- with the whole body limp
            # nothing newer replaced it, and servo ON pulled a fallen arm
            # 14 deg back up (2026-11-07). PhysX keeps a target until the
            # next command, so nothing is lost between messages.
            ("SubscribeJointState.outputs:execOut",
             "ArticulationController.inputs:execIn"),
            ("ReadSimTime.outputs:simulationTime",
             "PublishClock.inputs:timeStamp"),
            ("ReadSimTime.outputs:simulationTime",
             "PublishJointState.inputs:timeStamp"),
            ("SubscribeJointState.outputs:jointNames",
             "ArticulationController.inputs:jointNames"),
            ("SubscribeJointState.outputs:positionCommand",
             "ArticulationController.inputs:positionCommand"),
            ("SubscribeJointState.outputs:velocityCommand",
             "ArticulationController.inputs:velocityCommand"),
            ("SubscribeJointState.outputs:effortCommand",
             "ArticulationController.inputs:effortCommand"),
        ],
        og.Controller.Keys.SET_VALUES: [
            ("PublishJointState.inputs:targetPrim",
             [usdrt.Sdf.Path(robot_path)]),
            ("ArticulationController.inputs:robotPath", robot_path),
            ("PublishJointState.inputs:nodeNamespace", args.ns),
            ("SubscribeJointState.inputs:nodeNamespace", args.ns),
            ("PublishJointState.inputs:topicName", "joint_states"),
            ("SubscribeJointState.inputs:topicName", "joint_command"),
        ],
    },
)

# IMU on base_link. If this image's API differs, say so and run without it:
# jedy_skrobot then reports "no IMU" instead of a value made up here.
imu_ok = False
try:
    from isaacsim.sensors.physics import IMUSensor
    imu_prim = robot_path + "/base_link/imu_sensor"
    IMUSensor(prim_path=imu_prim, name="imu", frequency=100)
    og.Controller.edit(
        {"graph_path": "/JedyImuGraph", "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnPlaybackTick"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("ReadIMU", "isaacsim.sensors.physics.IsaacReadIMU"),
                ("PublishImu", "isaacsim.ros2.bridge.ROS2PublishImu"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnTick.outputs:tick", "ReadIMU.inputs:execIn"),
                ("ReadIMU.outputs:execOut", "PublishImu.inputs:execIn"),
                ("ReadSimTime.outputs:simulationTime",
                 "PublishImu.inputs:timeStamp"),
                ("ReadIMU.outputs:angVel", "PublishImu.inputs:angularVelocity"),
                ("ReadIMU.outputs:linAcc", "PublishImu.inputs:linearAcceleration"),
                ("ReadIMU.outputs:orientation", "PublishImu.inputs:orientation"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("ReadIMU.inputs:imuPrim", [usdrt.Sdf.Path(imu_prim)]),
                ("ReadIMU.inputs:readGravity", True),
                ("PublishImu.inputs:frameId", "base_link"),
                ("PublishImu.inputs:topicName", "imu"),
            ],
        },
    )
    imu_ok = True
except Exception as error:  # noqa: BLE001 -- reported, not hidden
    print("[jedy_isaac] NO IMU: {}: {}".format(type(error).__name__, error),
          flush=True)

if head_cam is not None:
    og.Controller.edit(
        {"graph_path": "/JedyCameraGraph", "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnPlaybackTick"),
                ("RenderProduct",
                 "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("PublishRgb", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("PublishDepth", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("PublishInfo", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnTick.outputs:tick", "RenderProduct.inputs:execIn"),
                ("RenderProduct.outputs:execOut", "PublishRgb.inputs:execIn"),
                ("RenderProduct.outputs:execOut",
                 "PublishDepth.inputs:execIn"),
                ("RenderProduct.outputs:execOut", "PublishInfo.inputs:execIn"),
                ("RenderProduct.outputs:renderProductPath",
                 "PublishInfo.inputs:renderProductPath"),
                ("RenderProduct.outputs:renderProductPath",
                 "PublishRgb.inputs:renderProductPath"),
                ("RenderProduct.outputs:renderProductPath",
                 "PublishDepth.inputs:renderProductPath"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("RenderProduct.inputs:cameraPrim",
                 [usdrt.Sdf.Path(head_cam)]),
                ("RenderProduct.inputs:width", CAMERA_SIZE[0]),
                ("RenderProduct.inputs:height", CAMERA_SIZE[1]),
                ("PublishRgb.inputs:type", "rgb"),
                ("PublishRgb.inputs:topicName", "camera/color/image_rect_raw"),
                ("PublishRgb.inputs:frameId", "camera_color_optical_frame"),
                ("PublishDepth.inputs:type", "depth"),
                ("PublishDepth.inputs:topicName",
                 "camera/depth/image_rect_raw"),
                ("PublishDepth.inputs:frameId", "camera_depth_optical_frame"),
                ("PublishInfo.inputs:topicName", "camera/color/camera_info"),
                ("PublishInfo.inputs:frameId", "camera_color_optical_frame"),
            ],
        },
    )

if wheels:
    og.Controller.edit(
        {"graph_path": "/JedyGroundTruthGraph", "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnPlaybackTick"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("Odometry", "isaacsim.core.nodes.IsaacComputeOdometry"),
                ("Publish", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnTick.outputs:tick", "Odometry.inputs:execIn"),
                ("Odometry.outputs:execOut", "Publish.inputs:execIn"),
                ("Odometry.outputs:position", "Publish.inputs:position"),
                ("Odometry.outputs:orientation", "Publish.inputs:orientation"),
                ("Odometry.outputs:globalLinearVelocity",
                 "Publish.inputs:linearVelocity"),
                ("Odometry.outputs:angularVelocity",
                 "Publish.inputs:angularVelocity"),
                ("ReadSimTime.outputs:simulationTime",
                 "Publish.inputs:timeStamp"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("Odometry.inputs:chassisPrim",
                 [usdrt.Sdf.Path(robot_path + "/base_link")]),
                ("Publish.inputs:nodeNamespace", args.ns),
                ("Publish.inputs:topicName", "ground_truth"),
                ("Publish.inputs:odomFrameId", "world"),
                ("Publish.inputs:chassisFrameId", "base_link"),
                ("Publish.inputs:publishRawVelocities", False),
            ],
        },
    )

class ServoSwitch:
    """Servo OFF: the joints jedy_sim_servo names go limp, the rest hold.

    The same contract as Gazebo's jedy_servo_switch plugin: ``--servo-ns``
    ``/released`` (std_msgs/String) is the comma-separated names of EVERY
    released joint ("" = all driven), and ``/released_state`` reports the set
    as applied, on every change and every 0.1 s of sim time ("!name" for a
    name this robot does not have).

    A released joint is what Gazebo's is (see jedy_servo_switch.cpp): a Kondo
    servo with its torque off is back-driven through a gear train whose dry
    friction holds a light load and lets a heavy one fall. That friction is
    the joint's own, from the URDF's ``<dynamics friction damping>`` and the
    transmission's rotor inertia, which ``urdf_to_usd`` wrote as the joint's
    ``PhysxJointAxisAPI`` (static/dynamic friction effort, viscous friction,
    armature; jedy_description/README_dynamics.md). PhysX applies it servo on
    or off. So releasing only removes the motor: the drive's stiffness,
    damping and force limit go to 0. A gripper's mimic joint goes with its
    gripper. Driven again, the drive gets back what the USD gave it, and
    first its target is set to where the joint is now: nothing commands a
    released joint (jedy_isaac_servicer leaves it out), so the target PhysX
    still holds is the pose from BEFORE the release, and restoring the
    stiffness on it pulled a fallen arm back up by 14-18 deg before the
    servicer's next command arrived (jedy_skrobot's sample_servo.py on
    Isaac at ~0.3x real time, 2026-11-07).

    Leaving the drive its damping plus one force limit shared by every joint
    instead does not hold anything: the joint does not stop and creeps down
    at a few deg/s. A USD without the joint friction is refused (see
    ``__init__``).

    While any joint is released the articulation's solver position
    iterations go up to ``released_iterations`` (the USD's own count when
    none is). PhysX's joint static friction leaks on the wheeled robot: a
    released joint well inside its friction (elbow at 75 deg, tau_g 0.03 <
    tau_f 0.13 N m) crept at a constant rate that goes as 1 / (physics Hz x
    position iterations) -- 60 Hz x 32: 0.73 deg/s, 60 Hz x 255: 0.083 deg/s,
    against gravity as often as with it (measured with the --base wheels
    USD, 2026-11-07). Velocity iterations did not matter. With nothing commanded
    it looked held only because the articulation fell asleep (the same run
    with sleepThreshold 0 crept identically). 255 iterations halve Isaac's
    real-time factor, hence only while something is released.
    """

    def __init__(self, robot, namespace, state_topic, released_iterations):
        from pxr import Usd, UsdPhysics
        import rclpy
        from sensor_msgs.msg import JointState
        from std_msgs.msg import String
        self.drives = {}
        self._angular = {}
        self.friction = {}
        for prim in Usd.PrimRange(stage.GetPrimAtPath(robot)):
            for kind in ("angular", "linear"):
                if prim.HasAPI(UsdPhysics.DriveAPI, kind):
                    drive = UsdPhysics.DriveAPI(prim, kind)
                    self._angular[prim.GetName()] = kind == "angular"
                    self.drives[prim.GetName()] = (
                        drive, drive.GetStiffnessAttr().Get(),
                        drive.GetDampingAttr().Get(),
                        drive.GetMaxForceAttr().Get())
                    attr = prim.GetAttribute(
                        "physxJointAxis:{}:staticFrictionEffort".format(kind))
                    if attr and attr.HasAuthoredValue():
                        self.friction[prim.GetName()] = attr.Get()
        if not self.drives:
            raise RuntimeError("no joint drives under {}".format(robot))
        if not self.friction:
            # An old USD: releasing would leave a joint with nothing at all
            # holding it, which looks like a servo OFF and is not one.
            raise RuntimeError(
                "no joint under {} has physxJointAxis staticFrictionEffort: "
                "this USD predates the URDF's joint friction. Rebuild it with "
                "make_jedy_usd.py from the current jedy_gz.xacro".format(robot))
        print("[jedy_isaac] joint static friction from the USD: {}".format(
            ", ".join("{} {:.3g}".format(k, v)
                      for k, v in sorted(self.friction.items()))), flush=True)
        self._iterations = None
        for prim in Usd.PrimRange(stage.GetPrimAtPath(robot)):
            attr = prim.GetAttribute(
                "physxArticulation:solverPositionIterationCount")
            if attr and attr.HasAuthoredValue():
                self._iterations = (attr, attr.Get(), released_iterations)
                break
        if self._iterations is None:
            raise RuntimeError(
                "no physxArticulation:solverPositionIterationCount under {}: "
                "rebuild the USD with make_jedy_usd.py".format(robot))
        self.applied = []      # as asked, so the ack matches the request
        self.limp = set()      # applied, mimic joints included
        rclpy.init()
        self.node = rclpy.create_node("jedy_isaac_servo_switch")
        self.String = String
        self.pub = self.node.create_publisher(
            String, namespace + "/released_state", 10)
        self.node.create_subscription(
            String, namespace + "/released", self._on_released, 10)
        self.measured = {}     # [rad or m], Isaac's own joint_states
        self.node.create_subscription(
            JointState, state_topic,
            lambda m: self.measured.update(zip(m.name, m.position)), 10)
        self.next_report = 0.0

    def _members(self, name):
        mimic = name.replace("_gripper_joint", "_mimic_gripper_joint")
        return [name] + ([mimic] if mimic != name and mimic in self.drives
                         else [])

    def _on_released(self, msg):
        asked = [n for n in msg.data.split(",") if n]
        known = [n for n in asked if n in self.drives]
        limp = set(m for n in known for m in self._members(n))
        if limp != self.limp:
            for name, (drive, stiffness, damping, max_force) in \
                    self.drives.items():
                if name in limp and name not in self.limp:
                    drive.GetStiffnessAttr().Set(0.0)
                    drive.GetDampingAttr().Set(0.0)
                    drive.GetMaxForceAttr().Set(0.0)
                elif name not in limp and name in self.limp:
                    if name not in self.measured:
                        raise RuntimeError(
                            "servo on for {}: no joint_states for it yet, so "
                            "the drive target cannot be put where the joint "
                            "is".format(name))
                    q = self.measured[name]
                    # USD angular drive targets are in degrees.
                    drive.GetTargetPositionAttr().Set(
                        float(np.rad2deg(q)) if self._angular[name]
                        else float(q))
                    drive.GetStiffnessAttr().Set(stiffness)
                    drive.GetDampingAttr().Set(damping)
                    drive.GetMaxForceAttr().Set(max_force)
            attr, own, released = self._iterations
            attr.Set(int(released if limp else own))
            print("[jedy_isaac] servo: released {} (solver position "
                  "iterations {})".format(sorted(limp) or "nothing",
                                          attr.Get()), flush=True)
            self.limp = limp
        self.applied = known + ["!" + n for n in asked if n not in known]
        self._report()

    def _report(self):
        self.pub.publish(self.String(data=",".join(self.applied)))

    def spin(self, sim_time):
        import rclpy
        rclpy.spin_once(self.node, timeout_sec=0.0)
        if sim_time >= self.next_report:
            self.next_report = sim_time + 0.1
            self._report()


servo = ServoSwitch(robot_path, args.servo_ns,
                    "/{}/joint_states".format(args.ns),
                    args.released_position_iterations)

annotator = None
if args.out:
    os.makedirs(args.out, exist_ok=True)
    product = rep.create.render_product(view_cam, (960, 720))
    annotator = rep.AnnotatorRegistry.get_annotator("rgb")
    annotator.attach([product])

sim.play()
print("[jedy_isaac] running: /{0}/joint_states, /{0}/joint_command, /clock{1}{2}"
      .format(args.ns, ", /imu" if imu_ok else " (no /imu)",
              ", /camera/*" if head_cam else " (no camera)"), flush=True)

next_shot = args.shot_period
while simulation_app.is_running():
    sim.step(render=True)
    servo.spin(sim.current_time)
    if annotator is not None and sim.current_time >= next_shot:
        next_shot += args.shot_period
        data = annotator.get_data()
        if data is not None and getattr(data, "size", 0):
            from PIL import Image
            Image.fromarray(np.asarray(data)[:, :, :3]).save(
                os.path.join(args.out, "isaac_view.png"))
simulation_app.close()
