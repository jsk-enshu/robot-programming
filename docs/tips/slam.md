# SLAMによる地図作成とNav2によるナビゲーション（シミュレーション）

本章では，Gazeboシミュレーションの演習室（Room 13BC）で，

1. SLAMで部屋の地図を作り，
2. 地図をファイルに保存し，
3. その地図の上でNav2を使ってロボットを目標地点まで自律移動させる（RVizで目標を置く／Pythonから目標を送る）

までを行う．

```{important}
この章の手順は**シミュレーション専用**である．
SLAMとNav2には2次元のLiDAR（`/scan`）とオドメトリ（`odom`→`base_link`のTF）が必要だが，
実機のJedyにはLiDARが無く，`odom`フレームも出ていない（台車の車輪は速さを測れないため，
`mecanum_drive_controller`のオドメトリは使わない設定になっている）．
そこでGazeboでは`lidar:=true`を付けたときだけ，`base_laser`リンクにLiDARを付け，
Gazebo上のロボットの位置から`odom`→`base_link`を出す．
実機でこの章の手順が動くかは**未確認**である（下の「実機について」）．
```

少し詳細に踏み込んだ内容は [Nav2のドキュメント](https://docs.nav2.org/) と
[slam_toolbox](https://github.com/SteveMacenski/slam_toolbox) にある．

## 準備

使うパッケージ（slam_toolbox，Nav2）は`jedy_bringup`の依存に入っているので，
`rosdep install`でインストールされる．入っていなければ次で入れる．

```{code-block} console
$ sudo apt install ros-jazzy-slam-toolbox ros-jazzy-nav2-bringup ros-jazzy-nav2-simple-commander
```

この章の端末では，毎回次を実行しておく．

```{code-block} console
$ source /opt/ros/jazzy/setup.bash
$ source ~/ros2_ws/install/setup.bash
```

## SLAM

SLAM (Simultaneous Localization And Mapping)とは，移動ロボットなどにおいてレーザや視覚センサなどを利用し，自己位置同定(Localization)と地図作成(Mapping)を同時(Simultaneous)に行う手法である．自分がどこにいるのかといった位置や姿勢情報を取得するための自己位置同定や，まわりの環境がどのようになっているかというデータを構造化した地図は，環境中を移動するロボットにとって非常に重要な要素である．ここではSLAMのライブラリとして [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox) を使う．

### シミュレーションの起動（LiDAR付き）

演習室（`world:=room13bc`，既定）のGazeboを，`lidar:=true`を付けて起動する．

```{code-block} console
$ ros2 launch jedy_bringup jedy_gazebo.launch.py lidar:=true
```

`lidar:=true`で増えるのは次の3つで，付けなければ演習の既定のシミュレーションと同じである．

| トピック / TF | 型 | 中身 |
| --- | --- | --- |
| `/scan` | `sensor_msgs/LaserScan` | `base_laser`（`base_link`の0.1 m前，床から0.116 m）のLiDAR．360本（1°ごと），0.2〜12 m，10 Hz |
| `/odom` | `nav_msgs/Odometry` | Gazebo上のロボットの位置（車輪の滑りや誤差を含まない） |
| `odom`→`base_link` | TF | 同上 |

LiDARはロボット自身の体（台車，車輪，腕）が0.05〜0.17 mのところに見えるので，0.2 mより近いものは測らない設定にしてある．
LiDARはカメラと同じく描画（`render_sensors:=true`，既定）を使う．

別の端末で`/scan`が出ていることを確かめておこう．

```{code-block} console
$ ros2 topic hz /scan
average rate: 9.339
```

### SLAMの起動

別の端末で，SLAM ToolboxとRVizを起動する．

```{code-block} console
$ ros2 launch jedy_bringup jedy_slam.launch.py rviz:=true
```

このlaunchファイルはslam_toolboxの`online_async_launch.py`を`jedy_bringup/config/slam_toolbox_params_simple.yaml`で起動する．
Jazzyのslam_toolboxはライフサイクルノードで，configure・activateされるまで何もしないが，このlaunchが自動で行う．
`/scan`とTF（`odom`→`base_link`）から地図`/map`を作り，`map`→`odom`のTFを出す．

RVizには地図（Map），LiDARの点（LaserScan），ロボットのモデル（RobotModel）が表示される．
Fixed Frameは`map`である．

### ロボットを動かして地図を作る

腕を下げたまま走ると手が床に擦れるので，まず腕をたたむ（{doc}`../robot-programming-1-2026` の`ri.tuck_arms()`）．
Pythonで次を実行すればよい．

```{code-block} python
from jedy_skrobot import connect

robot, ri = connect()
ri.tuck_arms()
```

さらに別の端末でキーボード操縦を起動し，ロボットを部屋の中でゆっくり走らせる（キーの意味は1日目の資料を参照）．

```{code-block} console
$ ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true -p speed:=0.15 -p turn:=1.0
```

ロボットを動かすと，RViz上で地図の白い（通れる）部分と黒い（障害物）部分が増えていく．
ロボットが動く度に自分が地図の中のどこにいるか（自己位置同定）を繰り返しつつ，センサ情報から新しい領域の地図を作っている．
演習室は26.8 m×9.6 mあり，LiDARは12 mまでしか届かないので，部屋の奥まで走らせること．
たとえば机の島の間の中央の通路（出現位置から左へ0.4 m）を奥へ21 m進んで戻る（約43 m，0.15 m/sで約5分）と，下の図のように部屋全体の地図ができる．
机の脚や壁はLiDARの高さ（床から0.116 m）で見えているものだけが地図に入る．

:::{figure} ../fig/jedy-slam-room13bc.png
:align: center
:name: fig:jedy-slam-room13bc

演習室を中央の通路に沿って往復して作った地図（RViz）．白が通れるところ，黒が壁や机の脚，灰色がまだ見ていないところ．左寄りの丸は2025年のシミュレーションの赤いボール（`legacy:=true`，既定）
:::

:::{note}
Gazeboシミュレーション中にロボットが倒れてしまった場合は，{doc}`gazebo-sim` の「ロボットが倒れた場合の復帰方法」を参照すること．
:::

### 地図の保存

ロボットが作成した地図は，ファイルに保存することができる．SLAMを起動したまま，別の端末で次を実行する．

```{code-block} console
$ ros2 run nav2_map_server map_saver_cli -f ~/room13bc_map
```

`~/room13bc_map.pgm`と`~/room13bc_map.yaml`の2つのファイルが書き出される．
前者は地図を画像として書き出したもので，画像ビューアで表示できる．
後者はYAMLというファイル形式で，地図の解像度（1画素0.05 m）や原点，`room13bc_map.pgm`のファイル名が書かれている．

```{code-block} console
$ ls ~/room13bc_map.*
/home/student/room13bc_map.pgm  /home/student/room13bc_map.yaml
```

:::{note}
地図ファイルは`/tmp`に保存しないこと．再起動時に消える．
:::

保存したらSLAM（`jedy_slam.launch.py`）はCtrl-Cで止める．Gazeboはそのままでよい．

## Nav2によるナビゲーション

[Nav2](https://docs.nav2.org/)は，地図の上での自己位置推定（AMCL），経路計画，経路追従，障害物の回避をまとめたROS 2のナビゲーションの仕組みである．

### Nav2の起動

SLAMで作った地図を使うには，`map:=`でYAMLファイルを指定する．

```{code-block} console
$ ros2 launch jedy_bringup jedy_nav2.launch.py map:=$HOME/room13bc_map.yaml rviz:=true
```

`map:=`を省くと，`jedy_bringup`に入っている演習室の地図（`jedy_bringup/maps/room13bc.yaml`，上と同じ手順で作ったもの，{numref}`fig:jedy-slam-room13bc`）を使う．SLAMを飛ばしてNav2だけ試すときはこちらを使えばよい．
この地図はGazeboを`lidar:=true`だけ付けて起動したときのもので，赤いボールが入っている．`box:=true`などで置くものを変えると部屋は地図と少し違ってくる．Nav2のコストマップはLiDARで見えたものも障害物にするが，その組み合わせは試していない．

```{code-block} console
$ ros2 launch jedy_bringup jedy_nav2.launch.py rviz:=true
```

このlaunchファイルはnav2_bringupの`bringup_launch.py`を`jedy_bringup/config/nav2_params.yaml`で起動する．主な設定は次の通り．

- **自己位置推定**: AMCL．LiDARの点を地図に合わせてロボットの位置を推定する．地図はロボットの出現位置（Gazeboを起動した時の位置）から作ったので，初期位置は地図の原点(0, 0, 0)にしてある．
  別の場所から始めるときは，RVizの「2D Pose Estimate」でロボットのいる位置と向きを地図上でドラッグして教える．
- **経路追従**: MPPIコントローラの`Omni`モデル．Jedyの台車はメカナムホイールで横にも動けるので，横移動も使う．
- **速さ**: 実機の台車の範囲（{doc}`../robot-programming-1-2026`）に合わせて，前後・左右は0.156 m/sまで，旋回は1.66 rad/sまで．
- **ロボットの形（footprint）**: 腕をたたんだ姿勢の台車と腕を囲む長方形（前0.20 m，後ろ0.085 m，左右0.135 m）．

```{warning}
実機の台車は前後・左右0.078 m/s，旋回0.83 rad/s程度より遅い指令では**止まる**（{doc}`../robot-programming-1-2026`）．
Nav2は目標の近くなどでこれより遅い速さを出すので，実機ではこの設定のままでは目標の手前で止まったり，
ゆっくり回ろうとして回らなかったりすると考えられる（未確認）．シミュレーションの台車はどんなに遅い指令でも動く．
```

### RVizで目標を置く

RVizの上のツールバーの「Nav2 Goal」を押し，地図上の目標地点でマウスを押したまま，向けたい方向へドラッグして離す．
緑の線（計画された経路）が出て，ロボットがそれに沿って走り出す．左の「Navigation 2」パネルに残りの距離や経過時間が出る．

:::{figure} ../fig/jedy-nav2-room13bc.png
:align: center
:name: fig:jedy-nav2-room13bc

RVizでNav2 Goalを置いたところ．地図の上に障害物の周りのコストマップと，計画された経路が表示される
:::

### Pythonから目標を送る

プログラムから目標を送るには，nav2_simple_commanderの`BasicNavigator`を使う．
`jedy_skrobot`の`nav2_goal`は，腕をたたんでから，引数の目標へ順に走る例である．
目標は`x,y`または`x,y,向き`で，`map`座標系（m）と，`map`のx軸から反時計回りの角度（度）で与える．
同梱の地図ではx軸がロボットの出現時の正面（部屋の奥），y軸が左である．

```{code-block} console
$ ros2 run jedy_skrobot nav2_goal 4.4,-3.6,90 8.3,3.0,180 0,0
goal (4.40, -3.60, 90 deg): SUCCEEDED  32.2 s (robot clock 32.2 s), last pose (4.39, -3.59)
...
```

上の例（机の島をよけて部屋の手前の左右を回る）では，Gazebo上の本当の位置と目標の差は位置で0.02〜0.08 m，向きで約11°だった．
目標の許容誤差（`nav2_params.yaml`の`general_goal_checker`）が位置0.15 m，向き0.2 rad（11.5°）なので，ロボットは向きが許容範囲に入ったところで止まる．

中身の要点は次の通り（`jedy_skrobot/scripts/enshu/nav2_goal.py`）．

```{code-block} python
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

rclpy.init()                         # nav2_goal では connect() が行う
navigator = BasicNavigator()
navigator.waitUntilNav2Active(localizer='robot_localization')

goal = PoseStamped()
goal.header.frame_id = 'map'
goal.header.stamp = navigator.get_clock().now().to_msg()
goal.pose.position.x = 3.0
goal.pose.position.y = 0.4
goal.pose.orientation.w = 1.0       # 向き 0 度

navigator.goToPose(goal)
while not navigator.isTaskComplete():
    feedback = navigator.getFeedback()  # feedback.distance_remaining など
print(navigator.getResult() == TaskResult.SUCCEEDED)
```

`waitUntilNav2Active()`は，引数を省くとAMCLの初期位置として(0, 0, 0)を送ってしまい，ロボットが原点にいないと自己位置がずれる．
ここではAMCLの初期位置はlaunch側で与えているので，`localizer='robot_localization'`として初期位置を送らないようにしている．

## 実機について（未確認）

実機のJedyについて，この章の手順は確かめていない．コードから分かるのは次のことである．

- 実機を起動するlaunchは`jedy_fullbody_all.launch.py`などで（{doc}`../robot-programming-1-2026`），`jedy_bringup.launch.py`というlaunchファイルはROS 2の`jedy_bringup`には無い．
- 実機のlaunchにはLiDARのドライバも，`/scan`を出すノードも無い．
- 実機の台車の車輪は速さを返さないので，`mecanum_drive_controller`のオドメトリは原点のままで，`odom`のTFも出していない．
  `jedy_wheel_odometry`（サーボの角度から推定する）は`--publish-tf`を付けたときだけ`odom`→`base_link`を出すが，SLAMやNav2と組み合わせて試してはいない．

実機で同じことをするには，少なくとも2次元のLiDAR（またはデプスカメラから作った`/scan`）とオドメトリが要る．
