# EusLisp と scikit-robot の対応

2025 年度までの資料 (EusLisp) と 2026 年度の資料 (Python の scikit-robot) で，Jedy を動かす書き方の対応表である．

| 2025 年版 (EusLisp) | 2026 年版 (scikit-robot) |
| --- | --- |
| `(jedy-init)` | `robot, ri = connect()` |
| `(send *jedy* :reset-pose)` | `robot.reset_pose()` |
| `(send *jedy* :head_joint1 :joint-angle 30)` [deg] | `robot.head_joint1.joint_angle(np.deg2rad(30))` [rad] |
| `(send *ri* :angle-vector (send *jedy* :angle-vector) 3000)` [ms] | `ri.angle_vector(robot.angle_vector(), 3.0)` [s] |
| `(send *ri* :wait-interpolation)` | `ri.wait_interpolation()` |
| `(send *ri* :state :potentio-vector)` | `ri.angle_vector()` |
| `(send *ri* :go-velocity 100 0 0)` [mm/s, deg/s] | `ri.go_velocity(0.1, 0.0, 0.0)` [m/s, rad/s] |
| `(send *ri* :servo-on :names (list "head_joint0"))` | `ri.servo_on(joints=['head_joint0'])` |
| `(send *jedy* :rarm :inverse-kinematics c)` | `robot.rarm.inverse_kinematics(c)` |

**単位が違う**ことに注意: 角度は rad，時間は秒，速度は m/s・rad/s (SI 単位)．度で考えたいときは `np.deg2rad` / `np.rad2deg` を使う．
