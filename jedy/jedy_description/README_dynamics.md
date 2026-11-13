# Joint passive dynamics: decision log

What holds a Jedy joint whose servo is OFF, and where the numbers live.
GOAL phase 4 (2026-11-07). The values and the derivation of each one are in
`urdf/jedy_gz.xacro` (the block at the top and a comment on every joint).

## Decisions

1. **The URDF is the only place.** `jedy_gz.xacro` carries, per joint,
   `<dynamics friction=τ_f damping=b>` and, in a `<transmission>`, the rotor
   inertia and gear ratio. Gazebo reads `<dynamics>` itself; MuJoCo and Isaac
   get the same numbers through scikit-robot's `urdf_to_mjcf` / `urdf_to_usd`.
   The per-simulator settings (Gazebo plugin `<friction>`, `jedy_isaac.py
   --servo-friction`) are removed.

2. **Armature goes in `<transmission><actuator>` as Drake's
   `<drake:rotor_inertia value=…/>` and `<drake:gear_ratio value=…/>`**, next
   to the standard `<mechanicalReduction>`. armature = rotor_inertia ×
   gear_ratio².
   - URDF has no armature tag.
   - Drake documents exactly these two tags, inside the actuator, for
     reflected inertia: https://drake.mit.edu/doxygen_cxx/group__multibody__parsing.html
     (`/robot/joint/actuator/drake:rotor_inertia@value`,
     `/robot/joint/actuator/drake:gear_ratio@value`). The source is
     the two physical quantities, not the product: a measured rotor inertia or
     a different gearbox changes one number.
   - MuJoCo's armature is a joint attribute and is documented as the reflected
     rotor inertia I_rotor·N² (https://mujoco.readthedocs.io/en/latest/modeling.html);
     it has no URDF tag either.
   - Isaac Sim's URDF importer does not document reading armature from a URDF
     (https://docs.isaacsim.omniverse.nvidia.com/latest/importer_exporter/import_urdf.html);
     PhysX takes it per axis (`physxJointAxis:angular:armature`).
   - `mechanicalReduction` alone is a ratio, not an inertia; it is kept for
     ROS tools and must agree with `drake:gear_ratio`.
   - Rejected: an `armature=` attribute on `<dynamics>` (non-standard,
     urdfdom and sdformat would silently drop it, and it would hide which
     of the two inputs was measured).

3. **Mapping per simulator** (needs a scikit-robot new enough to have `urdf_to_mjcf` and `urdf_to_usd`):

   | URDF | MuJoCo (`urdf_to_mjcf`) | Isaac Sim 5.0 / PhysX (`urdf_to_usd`) | Gazebo Harmonic (DART) |
   | --- | --- | --- | --- |
   | `friction` τ_f [N m] | `frictionloss` (+ stiff `solimpfriction`, see below) | `physxJointAxis:angular:staticFrictionEffort` and `dynamicFrictionEffort` | SDF `<axis><dynamics><friction>` → DART joint Coulomb friction |
   | `damping` b [N m s/rad] | `damping` | `physxJointAxis:angular:viscousFrictionCoefficient` = b·π/180 (PhysX uses N m s/**deg**) | SDF `<damping>` |
   | rotor_inertia·ratio² | `armature` | `physxJointAxis:angular:armature` | **not applied**: SDF 1.11 has no armature (no `armature` string in libsdformat14 14.9.0 or the gz-physics7 dartsim plugin, checked 2026-11-07) |

   PhysX attribute names and units: `omni.usd.schema.physx` 107.3.18,
   `PhysxSchemaAddition/resources/generatedSchema.usda` inside
   `nvcr.io/nvidia/isaac-sim:5.0.0` (class `PhysxJointAxisAPI`, multiple-apply,
   instance `angular`). The older `physxJoint:jointFriction` is a unitless
   coefficient scaled by the joint force (https://github.com/isaac-sim/IsaacSim/issues/856)
   and is not used. Isaac Lab writes the same per-axis values for Isaac Sim ≥ 5.0
   (`write_joint_friction_coefficient_to_sim`,
   https://github.com/isaac-sim/IsaacLab/blob/main/source/isaaclab/isaaclab/assets/articulation/articulation.py).

   Gazebo: URDF `<dynamics friction>` becomes SDF `<friction>` (checked: `gz sdf
   -p` on the expanded URDF gives 18 `<friction>0.13</friction>`), and the
   dartsim plugin reads `sdf::JointAxis::Friction()` (symbol present in
   libgz-physics7-dartsim-plugin.so.7).

4. **MuJoCo's friction creeps unless made stiff.** `frictionloss` is a soft
   constraint: under a steady load below it, the joint moves at a constant
   speed set by the constraint impedance, the same at 1.2× and 2× the load.
   Measured with a 0.2 kg pendulum (mujoco 4.0.0): default solimp 5.9 °/s,
   d = 0.95 2.9 °/s, d = 0.999 0.056 °/s. `urdf_to_mjcf` writes
   `solimpfriction="0.999 0.9999 0.001 0.5 2"` on joints with friction. The
   remaining creep belongs to the simulator, not to the joint.

   **Isaac Sim (PhysX 5, Isaac Sim 5.0) leaks too, at a rate set by the
   solver.** On the wheeled robot (standing on its rollers) a released joint
   well under its static friction moves at a constant speed that goes as
   1 / (physics Hz × articulation position iterations), against gravity as
   often as with it: rarm_joint3 at 75°, τ_g 0.03 < τ_f 0.13 N·m —
   60 Hz × 32 iterations 0.73 °/s, 120 × 32 0.34, 60 × 128 0.17,
   60 × 255 and 120 × 128 0.083 °/s. Velocity iterations (32 instead of 4)
   change nothing. Without a floor contact (base on planar joints, no
   collider) it holds (≤ 0.1° in 6 s). An articulation that nothing commands
   falls asleep and then looks held; with `sleepThreshold` 0 it leaks
   exactly as when commanded. So `jedy_isaac.py` raises the position
   iterations to 255 (PhysX's maximum) while any joint is released and puts
   the USD's 32 back afterwards (live, wheels: 0.03–0.10 °/s at 40/60/75°;
   the real-time factor drops from ~0.45 to ~0.27 during the release only).
   Probe: `tools/probes/p4_isaac_jedy_release.py` in the work root (offline,
   no ROS; FLOOR/RAMP/NOSLEEP/ROOTATTR), results
   `evidence/p4/isaac_offline_wheels_ramp_60hz.txt`, 2026-11-07.
   PhysX's stick test compares the impulse needed to stop the joint's
   current velocity with `staticFrictionEffort`
   (https://nvidia-omniverse.github.io/PhysX/physx/5.6.1/docs/Articulations.html);
   with the roller contacts in the same solve that test is only as exact as
   the solver has converged — our reading of the measurements, not a
   documented limitation.

   Retracted (2026-11-07): earlier "planar base" traces that crept 1–18° were
   taken with Isaac started as `--base wheels` on a planar USD, i.e. a floor
   collider under wheels held by stiff planar drives; that contact, not the
   joint, shook the arm.

5. **Servo OFF in each simulator = remove the motor, keep the joint.** The
   drive is made to produce no torque at all; what remains is the URDF's
   friction/damping/armature, which the physics engine applies whether or
   not the servo is on (as on a real geared servo).

## Unknowns (each is "不明" in the xacro, not a blank)

Servo model (KRS-3302 ICS assumed from the URDF's speed), rotor inertia,
gear efficiency, torque constant, winding resistance, backlash, and whether
ICS "free" coasts or brakes the motor. The identification procedure that
replaces the τ_f estimate with a measurement is in `HUMAN_CHECKLIST.md`.
