# Gazebo Simulation

## Jedy環境のsimulation
以下のコマンドでシミュレーションのGazebo環境を立ち上げる．

```{code-block} console
$ source /opt/ros/jazzy/setup.bash
$ source ~/ros2_ws/install/setup.bash
$ ros2 launch jedy_bringup jedy_gazebo.launch.py
```

ロボットは演習室（13BC）の机の手前に置かれる．

### 起動オプション

`引数:=値`を後ろに付けると，置く物や重さを変えられる（複数指定してよい）．主なものは次の通り．

| 引数 | 内容 |
| --- | --- |
| `box:=true` | カメラの演習に使う赤い箱を置く |
| `demo:=true` | AprilTagの付いた台と立方体，椅子，コップを置く |
| `person:=true` | 人を置く（初回はダウンロードする） |
| `tags:=true` | AprilTag（id 0〜9）を周りに立てる |
| `checkerboard:=true` | チェッカーボードを置く |
| `lidar:=true` | LiDARと`/odom`を足す（{doc}`slam`用） |
| `camera:=d405` | 頭のカメラを実機のD405と同じ画角・距離にする |
| `camera_rate:=30` | 頭のカメラを1秒に30枚にする（既定は5，PCが重くなる） |
| `gui:=false` | Gazeboの画面を出さない（ロボットは動く） |
| `render_sensors:=false` | カメラを止める（GPUの無いPC向け） |
| `world:=empty` | 演習室を置かず，何も無い床にする |

使い方は{doc}`../robot-programming-1-2026` の「Gazebo シミュレーション」にもある．
全部の引数と説明は次で見られる．

```{code-block} console
$ ros2 launch jedy_bringup jedy_gazebo.launch.py --show-args
```

(gazebo-robot-reset)=
## ロボットが倒れた場合の復帰方法

Gazeboシミュレーション中に，ロボットが倒れてしまうことがある．腕を大きく振ったり，台車で机や壁にぶつかったりすると転倒することがある．

:::{figure} ../fig/jedy-gazebo-fall-down.png
:align: center
:name: fig:jedy-gazebo-fall-down

Gazeboシミュレーション中に倒れてしまったjedyロボット
:::

このような場合，Gazeboの`gz service`コマンドを使用してロボットの姿勢をリセットすることができる．以下のコマンドを実行すると，ロボットを指定した位置・姿勢に復帰させることができる．

```{code-block} console
$ source /opt/ros/jazzy/setup.bash
$ gz service -s /world/jedy_world/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 3000 --req "
name: 'jedy',
position: {x: 0, y: 0, z: 0.2},
orientation: {w: 1, x: 0, y: 0, z: 0}
"
data: true
```

このコマンドは以下のパラメータを指定している：

- `/world/jedy_world/set_pose`：`jedy_world`はGazeboのワールドの名前（`gz service -l | grep set_pose`で確かめられる）
- `name: 'jedy'`：ロボットのモデル名
- `position: {x: 0, y: 0, z: 0.2}`：ロボットの位置（単位：メートル）．原点(0, 0)は起動したときのロボットの位置である．z座標を0.2mに設定することで，地面から少し浮かせた位置に配置する
- `orientation: {w: 1, x: 0, y: 0, z: 0}`：ロボットの姿勢（クォータニオン）．この値は起動したときの向き（直立して部屋の奥を向く）を表す

コマンド実行後，ロボットは指定した位置・姿勢に復帰する．
`data: true`が出ずに`Service call timed out`となるときは，Gazeboが起動しているか，ワールドの名前が合っているかを確かめる．

<video width="100%" controls preload="metadata">
  <source src="../fig/jedy-gazebo-reset-pose.mp4" type="video/mp4">
  お使いのブラウザは動画タグをサポートしていません．
</video>

**動画：** ロボットを倒し（`set_pose`で傾けて落とした），上のコマンドで戻したところ（実時間）．

:::{figure} ../fig/jedy-gazebo-reset-pose.png
:align: center
:name: fig:jedy-gazebo-reset-pose

`gz service`コマンドで復帰したjedyロボット
:::

:::{note}
ロボット名（`name`パラメータ）やz座標の値は，使用するロボットモデルによって適切に調整する必要がある．Gazebo GUI上でEntity Treeを確認することで，ロボットのモデル名を確認できる．
:::
