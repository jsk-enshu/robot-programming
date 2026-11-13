#!/usr/bin/env python3
"""Write the Isaac Sim USD of Jedy's whole body from jedy_description.

    xacro $(ros2 pkg prefix jedy_description)/share/jedy_description/urdf/jedy_gz.xacro \
        gz_control:=false > /tmp/jedy_fullbody.urdf
    python make_jedy_usd.py /tmp/jedy_fullbody.urdf jedy_fullbody.usdc
    python make_jedy_usd.py --base planar /tmp/jedy_fullbody.urdf jedy_planar.usdc

The URDF is jedy_gz.xacro, the same kinematics Gazebo and the real bring-up
expand, so the three agree on joint names, axes and limits. Needs scikit-robot
(skrobot.export.usd) and usd-core; no GPU.

Two ways to carry the base:

``--base planar`` mounts it on three position-driven planar joints
(base_x_joint, base_y_joint, base_yaw_joint), and jedy_isaac_servicer turns the
wheel speeds the base controller asks for into a planar motion. Nothing touches
the floor.

``--base wheels`` (default) leaves the robot free (a floating base) to stand on the floor
on its wheels, which are velocity-driven. A plain cylinder can only roll
forwards, so each wheel's collider is replaced by a ring of mecanum rollers:
spheres on passive joints whose axes lie at 45 deg to the wheel axis in the
plane tangent to the rim. A sphere spinning freely about that axis lets the
contact slide across it and grips along it, which is what a real roller does.
The ring dips by about 1 mm between rollers.

Inertia: urdf_to_usd treats an isotropic URDF tensor as a placeholder and
rebuilds it from the mesh (same mass and centre of mass). The arm links' URDF
tensors are isotropic 1e-4 kg m^2 (fix_inertials_from_mesh.py floors them so
Gazebo's friction joints move), so Isaac gets the mesh tensors (~1e-6..2e-5).
``--urdf-inertia`` writes the URDF tensors as they are instead. Measured on the
planar base, rarm_joint3 released at 60 deg (tau_g 0.07, tau_f 0.13): mesh
tensors creep ~1 deg in 5 s, URDF tensors ~18 deg in 8 s, speeding up
(2026-11-07), so the mesh rebuild
stays the default. Caveat: those planar runs had a floor collider under the
held wheels (Isaac started with --base wheels), which shook the arm; the
comparison has not been repeated without it (jedy_description/
README_dynamics.md). The static centre-of-mass load, which sets where a joint
falls, is the same either way.
"""

import argparse

import numpy as np

from skrobot.export.usd import urdf_to_usd

#: Collision-free resting pose the robot starts in and the joint drives hold
#: before any command: arms folded at the sides, derived from an earlier
#: Gazebo start pose with elbows (joint3) and wrists (joint5) bent 0.3 rad
#: further. That Gazebo pose left the grippers 7 mm BELOW the wheels' contact
#: plane (lowest point -0.134 m vs -0.127 m from base_link, computed from the
#: collision meshes); standing on a floor, the robot started with its hands in
#: it and was thrown on its back. This one keeps them 31 mm above.
#: NOT the current Gazebo start pose (jedy_gazebo.launch.py now spawns in
#: reset_pose, rarm_joint0 0.5236, rarm_joint3 -2.0944), which has not been
#: tried in Isaac: changing HOME needs an Isaac run to check the robot stands.
HOME = {
    "rarm_joint0": 1.5708, "rarm_joint2": -0.5236, "rarm_joint3": -2.0453,
    "rarm_joint5": -0.4745,
    "larm_joint0": -1.5708, "larm_joint2": 0.5236, "larm_joint3": -2.0453,
    "larm_joint5": -0.4745,
}

#: Wheel -> which way its rollers lean (+1/-1 about the rim tangent). Opposite
#: corners match, as on a mecanum base; which pair is +1 decides whether a
#: sideways command goes left or right, and is checked by driving vy in Isaac.
ROLLER_HAND = {
    "front_left_wheel": 1.0, "rear_right_wheel": 1.0,
    "front_right_wheel": -1.0, "rear_left_wheel": -1.0,
}


def add_mecanum_rollers(stage, wheels, n_rollers, roller_radius,
                        wheel_damping, wheel_max_torque, roller_mass,
                        hand_sign=1.0):
    """Replace each wheel's cylinder collider with a ring of rollers.

    Parameters
    ----------
    stage : pxr.Usd.Stage
        Written by ``urdf_to_usd``: links are ``/robot/<link>`` with their
        world pose at the home configuration, joints ``/robot/joints/<joint>``.
    wheels : list of str
        Wheel names; ``<name>_link`` and ``<name>_joint`` must exist.
    n_rollers : int
    roller_radius : float
        [m]. Each roller's centre sits this far inside the rim.
    wheel_damping : float
        Velocity drive gain of the wheel joint [N m s/deg, USD's unit].
    wheel_max_torque : float
        [N m]. The servo's stall torque, so a blocked wheel stops.
    roller_mass : float
        [kg].
    hand_sign : float
        Multiplies every :data:`ROLLER_HAND` entry, to flip the whole base.
    """
    from pxr import Gf
    from pxr import Sdf
    from pxr import UsdGeom
    from pxr import UsdPhysics

    for wheel in wheels:
        link = stage.GetPrimAtPath("/robot/{}_link".format(wheel))
        joint = stage.GetPrimAtPath("/robot/joints/{}_joint".format(wheel))
        cylinder = stage.GetPrimAtPath(
            "/robot/{}_link/collision_0".format(wheel))
        if not (link and joint and cylinder):
            raise RuntimeError("no {}_link / _joint / collision_0 in the "
                               "USD".format(wheel))
        # The rim: the cylinder's axis, centre and radius in the wheel frame.
        cyl = UsdGeom.Cylinder(cylinder)
        rim_radius = float(cyl.GetRadiusAttr().Get())
        local = UsdGeom.Xformable(cylinder).GetLocalTransformation()
        centre = np.array(local.ExtractTranslation())
        axis = np.array(local.TransformDir(Gf.Vec3d(0.0, 0.0, 1.0)))
        axis /= np.linalg.norm(axis)
        if n_rollers > 0:
            UsdPhysics.CollisionAPI(cylinder).CreateCollisionEnabledAttr(False)

        # The wheel: velocity-driven (no stiffness), torque-limited.
        drive = UsdPhysics.DriveAPI.Get(joint, "angular")
        drive.GetStiffnessAttr().Set(0.0)
        drive.GetDampingAttr().Set(float(wheel_damping))
        drive.GetMaxForceAttr().Set(float(wheel_max_torque))

        wheel_world = UsdGeom.Xformable(link).GetLocalTransformation()
        # Two unit vectors spanning the rim's plane.
        u = np.cross(axis, [0.0, 0.0, 1.0])
        if np.linalg.norm(u) < 1e-6:
            u = np.cross(axis, [0.0, 1.0, 0.0])
        u /= np.linalg.norm(u)
        v = np.cross(axis, u)
        lean = ROLLER_HAND[wheel] * hand_sign
        inertia = 0.4 * roller_mass * roller_radius ** 2
        for i in range(n_rollers):
            theta = 2.0 * np.pi * i / n_rollers
            radial = np.cos(theta) * u + np.sin(theta) * v
            tangent = -np.sin(theta) * u + np.cos(theta) * v
            pos = centre + (rim_radius - roller_radius) * radial
            roller_axis = (axis + lean * tangent) / np.sqrt(2.0)
            name = "{}_roller{}".format(wheel, i)
            path = Sdf.Path("/robot/" + name)
            xf = UsdGeom.Xform.Define(stage, path)
            xf.AddTransformOp().Set(
                Gf.Matrix4d().SetTranslate(Gf.Vec3d(*pos)) * wheel_world)
            UsdPhysics.RigidBodyAPI.Apply(xf.GetPrim())
            mass = UsdPhysics.MassAPI.Apply(xf.GetPrim())
            mass.CreateMassAttr(float(roller_mass))
            mass.CreateDiagonalInertiaAttr(Gf.Vec3f(inertia, inertia, inertia))
            mass.CreateCenterOfMassAttr(Gf.Vec3f(0.0, 0.0, 0.0))
            sphere = UsdGeom.Sphere.Define(stage, path.AppendChild("collision"))
            sphere.CreateRadiusAttr(float(roller_radius))
            sphere.CreatePurposeAttr(UsdGeom.Tokens.guide)
            UsdPhysics.CollisionAPI.Apply(sphere.GetPrim())

            rot = Gf.Quatf(Gf.Rotation(
                Gf.Vec3d(1.0, 0.0, 0.0),
                Gf.Vec3d(*roller_axis)).GetQuat())
            rj = UsdPhysics.RevoluteJoint.Define(
                stage, Sdf.Path("/robot/joints/{}_joint".format(name)))
            rj.CreateAxisAttr("X")
            rj.CreateBody0Rel().SetTargets([link.GetPath()])
            rj.CreateBody1Rel().SetTargets([path])
            rj.CreateLocalPos0Attr(Gf.Vec3f(*pos))
            rj.CreateLocalRot0Attr(rot)
            rj.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
            rj.CreateLocalRot1Attr(rot)


def require_start_at_home(stage):
    """Refuse a USD whose joints would start at 0 instead of :data:`HOME`.

    scikit-robot's urdf_to_usd writes each joint's initial position
    (``state:angular:physics:position``) only from the version that has
    "urdf_to_usd: the robot starts at home_positions". An older one writes the links at zero and every joint
    starts at 0: with the planar base the drives fold the arms up harmlessly,
    but standing on a floor the arms start through it and Isaac throws the
    robot metres away. That USD would load and look plausible, so stop here.
    """
    joint = next(iter(HOME))
    prim = stage.GetPrimAtPath("/robot/joints/" + joint)
    if not prim.GetAttribute("state:angular:physics:position").IsValid():
        import skrobot
        raise SystemExit(
            "the scikit-robot in use ({}, {}) does not write the joints' "
            "start pose (no state:angular:physics:position on {}): the robot "
            "would start with its arms through the floor. Use a scikit-robot "
            "new enough to write the joints' start pose.".format(
                getattr(skrobot, "__version__", "?"), skrobot.__file__,
                joint))


def set_solver(stage, position_iterations, velocity_iterations):
    """PhysX articulation solver iterations on the robot root."""
    from pxr import Sdf
    root = stage.GetPrimAtPath("/robot")
    root.CreateAttribute(
        "physxArticulation:solverPositionIterationCount",
        Sdf.ValueTypeNames.Int).Set(int(position_iterations))
    root.CreateAttribute(
        "physxArticulation:solverVelocityIterationCount",
        Sdf.ValueTypeNames.Int).Set(int(velocity_iterations))


def main():
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    p.add_argument("urdf")
    p.add_argument("out")
    p.add_argument("--base", choices=["planar", "wheels"], default="wheels")
    p.add_argument("--rollers", type=int, default=16,
                   help="rollers per wheel (--base wheels); 0 keeps the plain "
                        "cylinder, which cannot drive sideways")
    p.add_argument("--roller-radius", type=float, default=0.012, help="[m]")
    p.add_argument("--roller-mass", type=float, default=0.004, help="[kg]")
    p.add_argument("--wheel-damping", type=float, default=0.05,
                   help="wheel velocity gain [N m s/deg]")
    p.add_argument("--wheel-max-torque", type=float, default=2.0,
                   help="[N m]")
    p.add_argument("--hand", type=float, default=1.0, choices=[1.0, -1.0],
                   help="-1 flips every roller's lean")
    p.add_argument("--armature", type=float, default=0.0,
                   help="joint armature passed to urdf_to_usd")
    p.add_argument("--urdf-inertia", action="store_true",
                   help="write the URDF inertia tensors as they are instead "
                        "of urdf_to_usd's mesh rebuild of isotropic ones")
    p.add_argument("--drive-stiffness", type=float, default=4000.0,
                   help="joint drive stiffness before urdf_to_usd scales it "
                        "by gravity load [N m/deg]; the steady sag under "
                        "gravity goes as its inverse")
    p.add_argument("--drive-damping", type=float, default=400.0,
                   help="[N m s/deg], scaled the same way")
    # While a joint is released jedy_isaac.py raises this to
    # --released-position-iterations (a released joint's static friction
    # leaks at a rate that goes as 1 / (Hz x position iterations)); 32 is
    # the cost of everything else.
    p.add_argument("--position-iterations", type=int, default=32)
    p.add_argument("--velocity-iterations", type=int, default=4)
    args = p.parse_args()
    wheels_mode = args.base == "wheels"
    stage = urdf_to_usd(args.urdf, args.out, home_positions=HOME,
                        mobile_base=not wheels_mode,
                        floating_base=wheels_mode, armature=args.armature,
                        drive_stiffness=args.drive_stiffness,
                        drive_damping=args.drive_damping,
                        recompute_inertia=not args.urdf_inertia)
    if wheels_mode:
        require_start_at_home(stage)
        add_mecanum_rollers(
            stage, sorted(ROLLER_HAND), args.rollers, args.roller_radius,
            args.wheel_damping, args.wheel_max_torque, args.roller_mass,
            args.hand)
    set_solver(stage, args.position_iterations, args.velocity_iterations)
    stage.GetRootLayer().Save()
    print("wrote {}".format(args.out))


if __name__ == "__main__":
    main()
