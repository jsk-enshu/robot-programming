# 画像・深度センサによるSLAM，地図作成，自己位置同定（シミュレーションも対応）

本章では深度データを利用して，ロボットが周辺環境の地図を作り自己位置を同定する方法について説明する．

**ケーブルが絡まらないように，ケーブルをしっかり固定するなど工夫すること．**

なお，少し詳細に踏み込んだ内容などは [ROS 2 Navigation](https://docs.nav2.org/) のページにあるので，興味のある人は参照してみるとよい．

## SLAM

SLAM (Simultaneous Localization And Mapping)とは，移動ロボットなどにおいてレーザや視覚センサなどを利用し，自己位置同定(Localization)と地図作成(Mapping)を同時(Simultaneous)に行う手法である．自分がどこにいるのかといった位置や姿勢情報を取得するための自己位置同定や，まわりの環境がどのようになっているかというデータを構造化した地図は，環境中を移動するロボットにとって非常に重要な要素である．ここでは，JedyでSLAMを行う方法について紹介する．

:::{figure} ../fig/jedy-slam-toolbox.jpg
:align: center
:name: fig:jedy-slam-toolbox

RVizでのSLAM Toolbox実行結果．灰色の部分が作成された地図を示す
:::

## 実機またはシミュレーションの起動

まず，ロボットを起動する．

**実機の場合：**
```{code-block} console
$ ssh jedy@<ロボットPCのIPアドレス>
$ ros2 launch jedy_bringup jedy_bringup.launch.py
```

別のターミナルでROS_DOMAIN_IDを設定する．

```{code-block} console
$ export ROS_DOMAIN_ID=<Your Jedy's ROS_DOMAIN_ID>
```

**シミュレーションの場合：**
```{code-block} console
$ source /opt/ros/jazzy/setup.bash
$ source ~/ros2_ws/install/setup.bash
$ ros2 launch jedy_bringup jedy_gazebo.launch.py
```

## SLAMの起動

別のターミナルで，SLAM Toolboxを起動する．

```{code-block} console
$ source /opt/ros/jazzy/setup.bash
$ source ~/ros2_ws/install/setup.bash
$ ros2 launch jedy_bringup jedy_slam.launch.py
```

このlaunchファイルは以下を起動する：

- **slam_toolbox**: ROS 2のSLAMライブラリ．LiDARセンサ（`/scan`トピック）を使用して2次元の地図を作成する
- **slam_activator**: SLAM Toolboxを自動的に有効化するノード

## RVizでの可視化

別のターミナルでRVizを起動して，SLAMの進行状況を可視化する．

```{code-block} console
$ source /opt/ros/jazzy/setup.bash
$ source ~/ros2_ws/install/setup.bash
$ rviz2
```

RVizで以下のトピックを追加する：

1. **Map** (`/map`トピック) - SLAMが作成している地図
2. **LaserScan** (`/scan`トピック) - LiDARセンサのデータ
3. **RobotModel** - ロボットのモデル表示

Fixed Frameを`map`に設定する．

## ロボットの操縦とマッピング

さらに別のターミナルでキーボード操縦を起動し，ゆっくり左右旋回をしてロボットを動かしてみよう．

```{code-block} console
$ source /opt/ros/jazzy/setup.bash
$ source ~/ros2_ws/install/setup.bash
$ ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap /cmd_vel:=/mecanum_drive_controller/reference
```

ロボットを動かすと，RViz上で灰色の部分が増えていくのが分かる．この灰色の部分がロボットが生成している地図に相当し，ロボットが動く度に自分が地図の中のどこにいるか（自己位置同定）を繰り返しつつ，センサ情報から新しい領域の地図領域を作成している．

:::{note}
Gazeboシミュレーション中にロボットが倒れてしまった場合は，{doc}`gazebo-sim` の「ロボットが倒れた場合の復帰方法」を参照すること．
:::

## 地図の保存

ロボットが作成した地図は，ファイルに保存することができる．

```{code-block} console
$ source /opt/ros/jazzy/setup.bash
$ source ~/ros2_ws/install/setup.bash
$ mkdir -p ~/my_map
$ ros2 run nav2_map_server map_saver_cli -f ~/my_map
```

を実行すると，地図情報をファイル書き出しすることができる．`~/my_map.pgm`と`~/my_map.yaml`という2種類のファイルが書き出されているのがわかる．前者は地図情報を画像として書き出したものであり，画像ビューアで表示できる．一方，後者はYAMLというファイル形式であり，中では地図の情報や`my_map.pgm`のパスが記載されている．

地図ファイルの確認：

```{code-block} console
$ ls ~/my_map.*
# ~/my_map.pgm と ~/my_map.yaml が作成される
```

:::{note}
地図ファイルは重要なデータなので，ホームディレクトリや管理しているディレクトリに保存すること．`/tmp`ディレクトリに保存すると，再起動時に削除される可能性がある．
:::

## 保存した地図の利用

保存した地図を使用して，ロボットの自己位置推定（Localization）やナビゲーションを行うことができる．詳細は [ROS 2 Navigation Tutorials](https://docs.nav2.org/tutorials/index.html) を参照のこと．
