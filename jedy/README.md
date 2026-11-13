# jedy — the Jedy robot, on ROS 2

ロボットプログラミング演習 (2026 年度) で使う Jedy のパッケージ．
演習の手順・環境構築は [オンラインマニュアル](http://jsk-enshu.github.io/robot-programming/) を見よ
(環境構築: `docs/environment-setup.md`，演習: `docs/robot-programming-*-2026.md`)．

- 実機が無くても，Gazebo のシミュレーション (`ros2 launch jedy_bringup jedy_gazebo.launch.py`) で
  演習のプログラムがそのまま動く．実機ではロボットを USB でつないだ PC で同じプログラムを動かす．
- `jedyeus/` と `jedy_ros1_bridge/` は 2025 年度 (ROS 1 / EusLisp) 版で，`COLCON_IGNORE` してありビルドされない．
- 同じネットワークで複数人が ROS 2 を動かすと通信が混ざるので，必ず
  `export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` を設定する (`~/.bashrc` に書いておく)．

## 2025 互換 (ロボットシステムの講義)

`jedy_gazebo.launch.py` は既定 (`legacy:=true`) で 2025 年度のシミュレーションの名前も提供する
(`jedy_bringup/scripts/jedy_legacy_bridge` が 2026 年度の `/jedy_fullbody/...` に中継する)．不要なら `legacy:=false`．

- `/joint_states` (`/jedy_fullbody/joint_states` の中継)
- `/mecanum_drive_controller/reference` (`geometry_msgs/TwistStamped`) → `/cmd_vel`
- `/{head,rarm,larm}_controller/joint_trajectory` と `/{head,rarm,larm}_controller/follow_joint_trajectory`
  (関節名で 2026 年度の `head` / `right_arm`+`right_gripper` / `left_arm`+`left_gripper` コントローラに振り分ける．腕は 2025 年度と同じくグリッパ関節も含められる．一部の関節だけの指令も可)
- 2025 年度の world の赤い球 (半径 0.5 m，ロボットの左 1.5 m)．`ball:=false` で消す (`tags:=true` のときは既定で出ない)
- `use_sim_time` を付けずに実行した 2025 年度のプログラムの時刻 (壁時計) は，ブリッジがシミュレーション時刻に付け直す

2025 年度のまま戻したもの (`jedy_bringup`): `ros2 run jedy_bringup` で
`send_joint_position.py`，`send_joint_position_action.py`，`track_box_to_cmd_vel_0.py`，`track_box_to_cmd_vel_1.py`
(`opencv_apps` の `/camshift/track_box` を使う)，`checkerboard_detector.py` と `launch/checkerboard_detector.launch.py`，
`enshu.perspective` (rqt; コントローラマネージャは `/jedy_fullbody/controller_manager`)．
シミュレーションのチェッカーボード (`checkerboard:=true`) は内側コーナー 7x4，1 マス 0.04 m なので

```
ros2 launch jedy_bringup checkerboard_detector.launch.py use_sim_time:=true \
  input_topic:=/camera/color/image_rect_raw camera_info_topic:=/camera/color/camera_info \
  checkerboard_size:='[7, 4]' square_size:=0.04
```

The rest of this file is for whoever runs and maintains the real robot.

Everything that is *about this robot*: its description, its ros2_control
bring-up, its simulation. The driver it runs on is
[`actuo`](https://github.com/iory/actuo) (the Python package, and its ROS 2
packages under `ros/`), depended on and never modified from here.

| package | what it is |
| --- | --- |
| `jedy_description` | URDF, meshes and the servo map. No code, no ros2_control. |
| `jedy_bringup` | The `<ros2_control>` xacro, controller + Kondo bus configs, launch files, RViz layout. Depends on actuo. |
| `jedy_sim` | Gazebo plugins (servo OFF, mimic joints), the simulated servo switch and motor sounds, and a fake servicer for exercising the pipeline. |
| `jedy_safety` | Self-collision filter, plugged into actuo's real servicer. |
| `jedy_odometry` | Wheel odometry. An ESTIMATE, which is why it is not in actuo. |
| `jedy_teleop` | PS4 pad: the base, the head, torque on and off, go-home. |
| `jedy_skrobot` | `connect()` → `robot, ri`: the scikit-robot model and robot interface the exercises program against. |
| `jedy_vision` | Head-camera demos (people, pose, AprilTag), and the recognition nodes behind `ri.what_do_you_see()` / `ri.gesture()` (`ros2 launch jedy_vision recognition.launch.py`). |
| `jedy_vision_msgs` | The recognition nodes' messages and services, so `ri` can call them without `jedy_vision`'s venv. |
| `jedy_speech` | An offline speech demo on the CPU (Open JTalk, Whisper, a speech bubble in RViz). `ri.say()` / `ri.listen()` use jsk_3rdparty's VOICEVOX and ros_speech_recognition instead. |
| `jedy_enshu` | Nothing of its own: depends on every package the exercise uses, so `colcon build --packages-up-to jedy_enshu` builds those and no others. |
| `room13bc_description` | The exercise room (13BC) and its desks, for RViz and Gazebo (`world:=room13bc`). |

## Starting it

```
ros2 launch jedy_bringup jedy_fullbody_all.launch.py
ros2 launch jedy_bringup jedy_fullbody_all.launch.py camera:=true  # + head D405
```

That is the whole thing from nothing running, in the `/jedy_fullbody`
namespace: it waits for the board's serial port, then starts the bus servicer
(`jedy_fullbody_rcb4.yaml`, for the Kondo RCB-4mini), ros2_control with `jedy_fullbody_controllers.yaml`,
RViz with the servo and diagnostics panels, the wheel odometry and the PS4
teleop. Turn pieces off with `rviz:=false`, `joy:=false`, `wheels:=false`.
`camera:=true` adds the head RealSense D405 (`/camera/color`, `/camera/depth`).

The order is not guessable, which is why it is a launch and not three
terminals. The servicer and ros2_control hand each other a shared-memory
segment: the C++ side creates it and blocks until the servicer has written the
MEASURED state into it, and only then seeds its command from that. Starting
them together is fine. Starting ros2_control alone, against a servicer that has
already finished its handshake, deadlocks — so if you restart one, restart
both. The launch also deletes the leftover segment file first, because it is an
mmap'd file under /tmp and outlives every process that touched it.

### The pad

```
L1 (hold)          deadman -- nothing moves without it
left stick         drive and strafe        right stick  turn
D-pad              head pan and tilt       R1 + stick   slow
L2 (hold)          head twice as fast      circle       RELEASE all torque
triangle           torque ON               L1+R1+cross, HELD  go home (5 s)
```

**L2 speeds up the head, not the base.** The base has no faster to go: a full
stick already asks for the servos' top speed and `deadband.py` caps there --
past that one wheel saturates while the others keep theirs, which does not slow
the base down, it aims it somewhere else. A fast button for the base could only
work by making the plain stick slower first, which is a worse default than no
fast button. So the base stays at its top speed unmodified (0.156 m/s) and R1 scales it by 0.55, and L2 takes the head from 1.0 to 2.0 rad/s. The head's ceiling is `--head-lead`
rather than the hardware: the target may not run more than 0.20 rad ahead of
the measured angle, which at 20 Hz puts the limit at 4.0 rad/s. The node checks
that pair at startup and says so rather than letting a held D-pad crawl.

The wheels come up DRIVEN and need no `triangle`: a continuous-rotation servo
has no position, so the stale-pose hazard that servo-on exists for cannot happen
to it. `triangle` is for the arms and head, and it re-syncs the trajectory
controllers first -- without that the enable is refused, because it is only
allowed when the controller already commands where the robot is. It reports
what came back, refusals included; a button that swallows the answer teaches you
the feature is broken while the robot is telling you what is wrong.

Going home is a HELD combination rather than a press because a whole-body move
over five seconds is not something a pad brushed against a table should start.
It refuses if any joint involved is still limp, naming them.

Unplugging the pad stops the base within 0.3 s and it picks up again by itself.

**Check the numbering on any new pad** rather than trusting these:

```
ros2 run jedy_teleop jedy_joy_monitor
```

It draws every axis and button live with the role the NODE assigns it -- read
from the same module the node imports, so the label cannot drift from the
behaviour -- and shows at the bottom what the robot would do right now, so "is
L1 the deadman on this pad" is answered by holding it and looking.

The driver is `joy_linux`, not `joy`, and that is not a preference. The `joy`
package goes through SDL, and SDL drops joystick events when the application
does not have input focus: with RViz focused it opened the pad, read its state
ONCE and then published that frozen state forever. Measured while chasing it:
3207 evdev events arriving from the kernel and not one change in `/joy`.
`joy_linux` reads joydev directly and has no such notion. Its axis and button
numbering is NOT interchangeable with SDL's, so a mapping verified under one
says nothing about the other.

### What to expect in the first few seconds

The odometry publishes zeros and a NaN quality figure until each wheel has
turned far enough for the estimator to time a revolution and learn its own
pulses-per-revolution. That is a couple of seconds of driving. It reports
nothing rather than guessing, which is the intended behaviour and not a fault.

## Why the split, and which way the dependency runs

`actuo` is an actuator abstraction: transports, actuator models, buses, and
(in `ros/`) the ros2_control shim and the bus servicer node. It must not know
Jedy's joint names, servo ids or link lengths. So every per-robot fact lives
here, and `jedy_bringup/package.xml` lists the actuo packages as
`exec_depend`.

`jedy_description` being a real ament package is load-bearing, not tidiness.
As a catkin package it was invisible to the ament index, so every
`package://jedy_description/meshes/...` in its own URDF failed to resolve and
RViz drew a complete, correctly articulated, **entirely invisible** robot — no
error, no warning, just no geometry. The bring-up launch briefly carried a
path-rewriting workaround for exactly this; porting the package deleted it.

## Run it

Everything, from nothing running, is the `Starting it` section above. The pieces
underneath, for when one of them needs to be run on its own:

```bash
# The servicer. Talks to the board; owns torque. Add
#   --command-filter jedy_safety.collision_filter:build
# for the self-collision gate.
ros2 run actuo_ros actuo_bus_node --real \
    --config "$(ros2 pkg prefix jedy_bringup)/share/jedy_bringup/config/jedy_fullbody_rcb4.yaml" \
    --shm-name jedy_fullbody --rate 100 --timeout 90

# Controllers, RViz with the servo + /diagnostics panels, wheel odometry.
ros2 launch jedy_bringup jedy_fullbody.launch.py wheels:=true rviz:=true

# The pad, separately (jedy_fullbody_all.launch.py joy:=false).
ros2 launch jedy_teleop jedy_joy.launch.py

# Torque, without a pad or a panel. It starts OFF for every joint that HAS a
# pose; the wheels come up driven, because a continuous-rotation servo has no
# stale pose to be thrown to and so the hazard this guard exists for cannot
# happen to it.
ros2 action send_goal /actuo_bus_servicer/servo/set actuo_msgs/action/SetServo "{enable: true}"
ros2 action send_goal /actuo_bus_servicer/servo/set actuo_msgs/action/SetServo "{enable: false}"

# Back to nothing running. Stops the launches and their nodes, keeps the
# servicer -- and therefore torque -- unless asked otherwise. --dry-run lists
# what it would stop. Use this rather than pkill before relaunching: a
# half-stopped robot is the two-controller-manager failure below.
ros2 run jedy_bringup jedy_stop
ros2 run jedy_bringup jedy_stop --servicer   # torque goes with it
```

One trajectory controller per limb, and one per gripper, all five active at
once (they claim disjoint joints). Moving the head does not disturb an arm
mid-motion, and a gripper can close while its arm is mid-reach, which a single
whole-body controller cannot offer -- a JTC holds exactly one trajectory. With
`wheels:=true` the sixth is `mecanum_drive_controller` (below).

```
/jedy_fullbody/head_controller/follow_joint_trajectory
/jedy_fullbody/right_arm_controller/follow_joint_trajectory
/jedy_fullbody/left_arm_controller/follow_joint_trajectory
/jedy_fullbody/right_gripper_controller/follow_joint_trajectory
/jedy_fullbody/left_gripper_controller/follow_joint_trajectory
```

Logic only, no board and no servicer: `backend:=mock`.

## The mecanum base

`wheels:=true` adds the four wheels' velocity interfaces and
`mecanum_drive_controller`, remapped to `/cmd_vel`. The type is **TwistStamped**,
not Twist: `teleop_twist_keyboard --ros-args -p stamped:=true` and
`teleop_twist_joy` both publish that, and a bare
`ros2 topic pub /cmd_vel geometry_msgs/msg/Twist` silently does not connect.

Three things about this base are worth knowing before driving it.

**It has a floor, not just a ceiling.** The wheels do not turn at a predictable
speed below about 0.078 m/s and are capped at 0.156: the wheel speeds measured on the RCB-4mini times the
CAD's 25.25 mm wheel radius. A twist below that gets a
stopped wheel and a logged refusal rather than a crawl, and the pad's stick is
mapped onto that range rather than onto 0..max so its first 40 % is not dead.

**Which servo is at which corner was derived from the URDF**, not read off the
names, and here the names agree: `front_right_wheel_joint` is the front right
wheel, and so on (table in `jedy_fullbody_controllers.yaml`). The wheel
`direction:` signs are ported from the legacy servo map and are NOT yet verified
on hardware: hand-turn each wheel and watch the model before driving.

**There is no odometry from the controller.** These servos report no speed, so
the wheels' velocity state interface is NaN and `mecanum_drive_controller`
correctly skips its integration; `enable_odom_tf` is false so nothing reaches
TF. `jedy_odometry` publishes an estimate instead, on
`/jedy_fullbody/jedy_wheel_odometry/odometry`, with `~/scale_spread` beside it
saying how much to trust it. Its covariance is left at zero deliberately: the
dominant error is mecanum roller slip, nothing in that node can see it, and a
plausible covariance is what would make a filter believe the pose.

## Poses

`jedy_bringup/config/jedy_fullbody_poses.yaml` holds named whole-body poses and
which controller commands which joint. The grouping is written out rather than
inferred from joint-name prefixes: a trajectory naming joints a controller does
not own is rejected outright, so a wrong guess there is not a partial move, it
is no move at all.

`home` is all zeros and that was CHECKED -- run through actuo.safety's
`SelfCollisionMonitor` against this robot's URDF, it comes back with no
colliding link pairs beyond those already touching at the reference pose.
Anything added there deserves the same before a robot is asked to drive to it
over five seconds.

## A joint parked outside its limit

This happens: the arm droops, or the configured envelope is tighter than the
hardware.

It used to be a deadlock. The joint could not be commanded to hold where it was
-- correctly -- so torque could not be enabled, so it could not be driven back,
so the only way out was pushing it by hand. Now enabling is allowed for such a
joint, its commands are held at the nearest edge of the envelope so it can only
travel INWARD, and both the enable and the servicer say so:

```
<joint> is <angle> rad OUTSIDE its envelope [<lower>, <upper>]
and will move <distance> rad back to <edge> as soon as torque is applied
```

So the recovery is: torque on, then the home pose.

If a joint keeps stopping short of its edge, that is the envelope disagreeing
with the hardware rather than a fault -- re-measure it with `actuo-teach-limits`
rather than widening the number by hand.

## State of the hardware

All 22 servos are on the bus: two 7-DOF arms (ids 0-13), two grippers (14/15),
the 2-DOF head (32/34) and the four wheels (20-23). A board scan on 2026-08-03
(`actuo-kondo-probe --scan`, on the ARM-H7 board Jedy had then) found every one of them present.

**A servo's ICS id is decided by wiring, not by a dial**: the six UARTs split
into two groups of three, one giving `2 * n` and the other `2 * n + 1`, `n`
being the position in that port's chain. So these ids are only valid for one
cabling — if a servo "goes missing", check which port group it is plugged into
before touching any id.

## When something does not come up

Five failures that cost real time here, each with the thing that identifies it.

**The spawners contradict each other**: `Controller already loaded, skipping
load_controller` and, a line later, `no controller with this name exists` — or
`Failed loading controller` for every one of them. That is two bring-ups of
the same robot. Both controller managers answer on the same service names, so
each spawner request lands on whichever one, and the controllers end up loaded
but never activated: the action servers still accept goals and report success,
and the arm does not move. `ros2 node list` will not show it — the two share a
node name — so look at the process list. Both halves are now guarded and it
should not be reachable: the servicer takes a lock keyed to the shm name, and
`jedy_fullbody_all.launch.py` refuses to start while another robot launch
(itself or Gazebo) is running (`jedy_preflight.py`). To clear it, `ros2 run jedy_bringup jedy_stop`, which
interrupts the launch processes first so each takes its own nodes down, leaves
the bus servicer (and therefore torque) alone unless given `--servicer`, and
deletes the leftover shm file once nothing holds it.

**The robot comes up but never moves, and nothing errors.** Restarting
ros2_control alone against a servicer that has already finished its
shared-memory handshake deadlocks. Restart both, or use
`jedy_fullbody_all.launch.py`, which deletes the leftover segment file first.

**The pose on `/joint_states` is frozen but every counter looks healthy.** The
FTDI adapter holds replies until its USB latency timer expires and ships at
16 ms, so a loop faster than ~62 Hz reads nothing at all. The servicer refuses
to start rather than publish a frozen pose and its error says how to fix it;
the permanent fix is
`sudo $(ros2 pkg prefix actuo_ros)/share/actuo_ros/udev/install.sh`.

**RViz dies with `what(): vector::reserve`.** A `.msg` was changed and a
dependent package was not rebuilt, so it deserialises with the old layout and
reads an array length out of the wrong bytes. `colcon build --packages-above
actuo_msgs`, never `--packages-select`.

**The board stops answering with nothing raised.** The usb-serial link can wedge
while the device stays enumerated and the port still opens; neither reopening it
nor unplugging and replugging clears it, and a USB device reset does. The
servicer does this by itself now -- releasing torque first, and coming back
released, because the robot may have moved while it was gone.
