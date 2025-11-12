# Gazebo Simulation

## Jedy環境のsimulation
以下のコマンドでシミュレーションのGazebo環境を立ち上げる．

```{code-block} console
$ source /opt/ros/jazzy/setup.bash
$ source ~/ros2_ws/install/setup.bash
$ ros2 launch jedy_bringup jedy_gazebo.launch.py
```

## ロボットが倒れた場合の復帰方法

Gazeboシミュレーション中に，ロボットが倒れてしまうことがある．特にjedyロボットのような小型ヒューマノイドロボットでは，関節の動作や外力によって転倒することがある．

:::{figure} ../fig/jedy-gazebo-fall-down.png
:align: center
:name: fig:jedy-gazebo-fall-down

Gazeboシミュレーション中に倒れてしまったjedyロボット
:::

このような場合，Gazeboの`gz service`コマンドを使用してロボットの姿勢をリセットすることができる．以下のコマンドを実行すると，ロボットを指定した位置・姿勢に復帰させることができる．

```{code-block} console
$ gz service -s /world/default/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --req "
name: 'jedy',
position: {x: 0, y: 0, z: 0.2},
orientation: {w: 1, x: 0, y: 0, z: 0}
"
```

このコマンドは以下のパラメータを指定している：

- `name: 'jedy'`：ロボットのモデル名
- `position: {x: 0, y: 0, z: 0.2}`：ロボットの位置（単位：メートル）．z座標を0.2mに設定することで，地面から少し浮かせた位置に配置する
- `orientation: {w: 1, x: 0, y: 0, z: 0}`：ロボットの姿勢（クォータニオン）．この値は初期姿勢（直立状態）を表す

コマンド実行後，ロボットは指定した位置・姿勢に復帰する．

:::{figure} ../fig/jedy-gazebo-reset-pose.png
:align: center
:name: fig:jedy-gazebo-reset-pose

`gz service`コマンドで復帰したjedyロボット
:::

:::{note}
ロボット名（`name`パラメータ）やz座標の値は，使用するロボットモデルによって適切に調整する必要がある．Gazebo GUI上でEntity Treeを確認することで，ロボットのモデル名を確認できる．
:::

