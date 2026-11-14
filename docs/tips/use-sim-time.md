# use_sim_timeパラメータとは

`use_sim_time` は，ROS 2 のノードが**どの時計を使うか**を決めるパラメータである．
Gazebo のシミュレーションと実機の違いのうち，いちばん気づきにくいのがこの時計の違いである．
このページでは，ROS 2 Jazzy と Gazebo Harmonic での `use_sim_time` を説明する．

## 2 つの時計

- 実機では，ノードは PC の時計 (システム時刻) を使う．`node.get_clock().now()` は今の時刻 (1970 年からの秒) を返す．
- Gazebo の中には，シミュレーションの時計 (シミュレーション時刻) がある．
  Gazebo を起動したときに 0 から始まり，シミュレーションが進んだぶんだけ進む．
  シミュレータが重くて実時間より遅く進むと，シミュレーション時刻も遅く進む．一時停止すると止まる．

`use_sim_time` を `true` にしたノードは，PC の時計ではなく `/clock` トピック (`rosgraph_msgs/Clock`) から時刻を取る．
`false` (既定) のノードは PC の時計を使う．

`use_sim_time` はノードごとのパラメータである．全部のノードに一度に効く設定は無いので，ノードごとに設定する．

`true` にしたノードは，最初の `/clock` が届くまで時刻が 0 になる．
`/clock` が来なければ，時刻は 0 のまま進まない．

## なぜシミュレーション時刻を使うのか

- センサのデータ (画像，関節角度，TF) には，Gazebo のシミュレーション時刻が付いている．
  PC の時計を使うノードがこれを読むと，時刻が 50 年以上ずれて，TF の変換や時刻の比較ができない．
- 「2 秒進む」を PC の時計で測ると，シミュレータが遅いときに，シミュレーションの中では 2 秒より短くしか進まない．
  シミュレーション時刻で測れば，実機と同じ距離だけ進む．

## Gazebo の `/clock`

Gazebo Harmonic は，自分の `/clock` を Gazebo の通信 (gz-transport) に出す．
これを ROS 2 の `/clock` にするのが `ros_gz_bridge` である．

`jedy_gazebo.launch.py` では次のようになっている．

1. `ros_gz_bridge` の `parameter_bridge` (ノード名 `clock_bridge`) が，Gazebo の `/clock` を ROS 2 の `/clock_gz` に出す．
   物理シミュレーションの 1 ステップ (5 ms) ごとに 1 回，シミュレーション時刻で 200 Hz である．
2. `topic_tools` の `drop` (ノード名 `clock_drop`) が，`/clock_gz` を `clock_every` 回に 1 回だけ `/clock` に出す．
   既定の `clock_every:=4` で，シミュレーション時刻の 50 Hz になる．

間引いているのは，`use_sim_time` の Python のノードが `/clock` を受け取るたびに Python の処理が走り，
画像の認識などが遅くなるためである．全部のステップを出したいときは `clock_every:=1` を付ける．

```{code-block} console
$ ros2 launch jedy_bringup jedy_gazebo.launch.py clock_every:=1
```

`/clock` が出ているかは次で確かめる．

```{code-block} console
$ ros2 topic echo --once /clock
clock:
  sec: 15
  nanosec: 965000000
---
$ ros2 topic hz /clock
average rate: 24.741
```

`ros2 topic hz` は PC の時計で測るので，シミュレータが実時間より遅いと 50 Hz より少なく出る (上は実時間の約半分の速さで動いていたとき)．

`jedy_gazebo.launch.py` が起動するロボットのノード (`robot_state_publisher`，`controller_manager` など) は，すべて `use_sim_time` が `true` になっている．

```{code-block} console
$ ros2 param get /jedy_fullbody/controller_manager use_sim_time
Boolean value is: True
```

実機の `jedy_fullbody_all.launch.py` は `/clock` を出さず，どのノードも `use_sim_time` は `false` である．

## jedy_skrobot では設定しなくてよい

`from jedy_skrobot import connect` の `connect()` は，シミュレーションか実機かを自分で調べる．
起動しているロボットの `/jedy_fullbody/controller_manager` に `use_sim_time` パラメータを問い合わせ，
`true` なら `ri` のノードも `use_sim_time` を `true` にする (`jedy_skrobot/interface.py` の `_probe()`)．
同じプログラムが，Gazebo でも実機でも，そのまま動く．

```{code-block} python
from jedy_skrobot import connect

robot, ri = connect()
print(ri.use_sim_time)   # Gazebo なら True，実機なら False
t0 = ri.now()            # ロボットの時計の今の時刻 [秒]
ri.sleep(2.0)            # ロボットの時計で 2 秒待つ
print(ri.now() - t0)     # 2.0
```

時間を測るときは，`time.time()` や `time.sleep()` ではなく `ri.now()` と `ri.sleep()` を使う．
`ri.go_velocity(..., duration=...)` や `ri.wait_interpolation()` も同じ時計で測っている．

Gazebo が止まって `/clock` が来なくなると，`ri.wait_interpolation()` は 30 秒待ってから
`the clock has not moved for 30.0 s (is the simulator running and publishing /clock?)` のエラーで止まる．

## 自分のノードで設定する

`jedy_skrobot` を使わずに自分で rclpy のノードを書くとき，Gazebo と一緒に使うなら `use_sim_time` を `true` にする．

### コマンドラインで

`ros2 run` で起動するノードは，`--ros-args -p` で設定できる．

```{code-block} console
$ ros2 run <パッケージ名> <実行ファイル名> --ros-args -p use_sim_time:=true
```

`rclpy.init()` を引数なしで呼ぶ Python のプログラムも，同じように渡せる．

```{code-block} console
$ python3 my_node.py --ros-args -p use_sim_time:=true
```

起動しているノードの値は `ros2 param get` で確かめる．

```{code-block} console
$ ros2 param get /my_node use_sim_time
Boolean value is: True
```

### rclpy で

ノードを作るときに `parameter_overrides` で渡す．

```{code-block} python
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

rclpy.init()
node = Node('my_node',
            parameter_overrides=[Parameter('use_sim_time', value=True)])
while node.get_clock().now().nanoseconds == 0:   # 最初の /clock が来るまでは 0
    rclpy.spin_once(node, timeout_sec=0.1)
print('sim time:', node.get_clock().now().nanoseconds * 1e-9)
```

作った後のノードなら `node.set_parameters([Parameter('use_sim_time', value=True)])` でもよい．

### launch ファイルで

1 つのノードだけなら，`Node` の `parameters` に書く．

```{code-block} python
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='demo_nodes_cpp', executable='talker',
             parameters=[{'use_sim_time': True}]),
    ])
```

その launch ファイルの全部のノードに設定するなら，`SetParameter` を先頭に置く．
`SetParameter` は `launch_ros.actions` にある．`from launch.actions import SetParameter` は `ImportError` になる．

```{code-block} python
from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    return LaunchDescription([
        SetParameter(name='use_sim_time', value=True),   # この後のノードすべてに効く
        Node(package='demo_nodes_cpp', executable='talker'),
        Node(package='demo_nodes_cpp', executable='listener'),
    ])
```

Gazebo と実機の両方で使う launch ファイルでは，`use_sim_time` を launch の引数にしておくとよい．
たとえば `jedy_slam.launch.py` は `use_sim_time` 引数 (既定 `true`) を持ち，それを slam_toolbox と RViz に渡している．

## 実機では `true` にしない

実機には `/clock` が無い．`use_sim_time` が `true` のノードは，時刻が 0 のまま進まない．

- ROS の時計で動くタイマーのコールバックが呼ばれない．
- ROS の時計で待つ処理 (`ri.sleep()` に当たるもの) が終わらない．

実機で動かないときは，自分のノードの `use_sim_time` を `ros2 param get` で確かめる．

## トラブルシューティング

### 時刻が 0 のまま進まない

`/clock` が出ていない．`ros2 topic echo --once /clock` で確かめる．
Gazebo が起動していない，Gazebo が一時停止している，`ROS_DOMAIN_ID` が Gazebo を起動した端末と違う，のどれかが多い．

### TF が `Lookup would require extrapolation` で失敗する

Gazebo の TF (シミュレーション時刻) を，`use_sim_time` が `false` のノード (PC の時計) が読んでいる．
そのノードの `use_sim_time` を `true` にする．RViz も同じで，Gazebo と使うときは `use_sim_time` を `true` で起動する
(`jedy_gazebo.launch.py rviz:=true` で開く RViz は設定済み)．

```{code-block} console
$ ros2 run rviz2 rviz2 --ros-args -p use_sim_time:=true
```

### Gazebo を起動し直したら動かなくなった

Gazebo を起動し直すと，シミュレーション時刻は 0 に戻る．
前の Gazebo のときから動いているノードには，時刻が過去に戻ったように見える．
Gazebo を起動し直したら，自分のプログラムも起動し直す．

## 参考資料

- [ROS 2 Design: Clock and Time](https://design.ros2.org/articles/clock_and_time.html)
- [ros_gz_bridge (ros_gz)](https://github.com/gazebosim/ros_gz/tree/jazzy/ros_gz_bridge)
- [ROS 2 Jazzy: Launch のチュートリアル](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Launch/Launch-Main.html)
