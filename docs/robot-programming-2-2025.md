(day-whole-body)=
# ロボットの全身行動プログラミング

```{eval-rst}
.. raw:: html

   <style>.main-content { counter-reset: exercise 0; }</style>
```

## 本日の演習内容

本日の演習では順運動学や逆運動学を通してロボットアームの目標姿勢を制御する技術に触れる．
順運動学や逆運動学については講義や他の演習で学んできたと思う．
本演習ではそれらの理論に基づいて実際のロボットを動かすためのシステムの使い方を学んでいく．

また，本演習でも{doc}`robot-programming-1-2025`と同様に`ROS`を用いたシステムでロボットの行動をプログラミングしていく．
{doc}`robot-programming-1-2025`では`ROS`のトピックやサービス等を直接的に呼び出していたが本演習ではロボットインターフェース`*ri*`（`Robot Interface`の頭文字を取って`*ri*`）を用いて間接的に`ROS`のコマンドを送る．
口頭だと`コメアールアイ`だとか`アールアイ`と言う．
ロボットインターフェース`*ri*`ではロボットに搭載される複数の関節やセンサとの`ROS`通信を取りまとめて行っており`*ri*`を用いることで異なる種類のロボットの行動制御プログラムを共通することができるようになる．
異なる種類のロボットは異なるハードウェアを持つためハードウェアに依存する部分のプログラムはロボット毎に異なる．
それらのロボット毎に異なるハードウェアに依存する部分をロボットインターフェースよりも下位にまとめることでロボット毎の違いを吸収している．
これにより同一の上位プログラムから異なるロボットを行動を制御できるようになっている．

さらに付録には順・逆運動学を応用することで`Gazebo`シミュレーション上のヒューマノイドロボット`JAXON`を動かせるものもあるので余裕のある人は必須課題が終わった後や演習後に試してみて欲しい．

本日の目標は以下である．

<div class="screen">

- ロボットモデル(`*jedy*`)とロボットインターフェース(`*ri*`)の違いを理解する

  - ロボットインターフェースによってロボットとデータをやり取りする通信の窓口をまとめておくと便利であることを理解する

  - ロボットモデルによってロボットのリンク長や関節角度に基づいた幾何学的な計算や可視化ができることを理解する

- ロボットインターフェースのメソッドを使えるようになる

  `:state`
  センサ情報を取得する

  `:go-velocity`
  目標の移動速度を送る

  `:angle-vector`
  目標の関節角度指令を送る

- ロボットモデルのメソッドを使えるようになる

  `:angle-vector`
  順運動学メソッド

  ``:inverse-kinematics``
  逆運動学メソッド

- `lisp`の文法に慣れる

</div>

```{important}
各チェックポイントを達成しながら，進捗報告シートへの記入を進め，全てのチェックポイントの進捗報告を完了せよ．各チェックポイントの結果をスクリーンショット等で保存しておくこと．
発展課題に関しては必須ではないが，すべてをこなすとかなりのロボットレベルが上がるため，これからロボットをやっていきたいというロボット経験者も未経験者もぜひチャレンジしてほしい．
```

### 環境構築とソフトウェア更新

```{important}
**初めて環境構築を行う場合**は，[環境構築](environment-setup.md)のページを参照して，ワークスペースの初期セットアップを行うこと．

**演習開始時に既に環境構築済みの場合**は，以下の更新手順を実行して最新バージョンのソフトウェアを取得すること．
```

#### 演習開始時のソフトウェア更新手順

演習を開始する前に，必ず最新バージョンのソフトウェアを取得すること．
vcsファイル（`.repos.one`や`.repos.jazzy`）が更新されている場合や，既存のリポジトリに変更がある場合は，以下の手順で更新する．

**ROS 2 ワークスペースの更新**

```{code-block} console
$ cd ~/ros2_ws/src
$ source /opt/ros/jazzy/setup.bash
# 最新のvcsファイルを取得して新しいリポジトリをインポート
$ wget https://raw.githubusercontent.com/jsk-enshu/robot-programming/refs/heads/master/.repos.jazzy -O- | vcs import
# 既存のリポジトリを最新版に更新
$ vcs pull
$ cd ~/ros2_ws/
# 依存関係を更新
$ rosdep update
$ rosdep install --from-paths src --ignore-src -y -r
# パッケージをビルド
$ colcon build --symlink-install --packages-select jedy_bringup jedy_description
```

**ROS 1 ワークスペースの更新**

```{code-block} console
$ cd ~/ros_ws/src
$ source /opt/ros/one/setup.bash
# 最新のvcsファイルを取得して新しいリポジトリをインポート
$ wget https://raw.githubusercontent.com/jsk-enshu/robot-programming/refs/heads/master/.repos.one -O- | vcs import
# 既存のリポジトリを最新版に更新
$ vcs pull
$ cd ~/ros_ws
# 依存関係を更新
$ rosdep update
$ rosdep install --from-paths src --ignore-src -y -r
# パッケージをビルド
$ catkin build jedy_ros1_bridge jedyeus
```

```{tip}
`vcs pull`コマンドは，ワークスペース内の全てのGitリポジトリに対して`git pull`を実行する．
新しいリポジトリが追加された場合は`vcs import`で取得され，既存のリポジトリは`vcs pull`で更新される．
```

### ROSコマンドなどのおさらい

#### ROS 1コマンド

```{code-block} console
$ rostopic list
# topic一覧がでる
$ rostopic hz /imu
# 周期がわかる
$ rostopic type /imu
# topicの型が何かわかる
$ rostopic echo /imu
# topicの内容を表示する
$ rosnode list
# rosnode一覧
$ rosmsg show sensor_msgs/Image
# topicのmsgの型の中身が分かる
$ rqt_graph
# ノード一覧がグラフとして表示される
```

#### ROS 2コマンド

```{code-block} console
$ ros2 topic list
# topic一覧がでる
$ ros2 topic hz /imu
# 周期がわかる
$ ros2 topic info /imu
# topicの型とpublisher/subscriber数が分かる
$ ros2 topic echo /imu
# topicの内容を表示する
$ ros2 node list
# rosnode一覧
$ ros2 interface show sensor_msgs/msg/Image
# topicのmsgの型の中身が分かる
$ rqt_graph
# ノード一覧がグラフとして表示される
```

詳細なコマンドの使い方については[ROS 1コマンド](tips/ros1-commands.md)と[ROS 2コマンド](tips/ros2-commands.md)のTipsページを参照すること．

## 双腕移動台車ロボットのハードウェアとセットアップ

本章では双腕移動台車ロボットの全身動作を行っていく．

### ハードウェア構成

双腕移動台車ロボットの片腕は8自由度あり
7自由度がアーム，1自由度がグリッパハンドである．
各関節はKondo科学社の`KRS`シリーズモータを採用している．

```{figure} fig/servo-numbering.png
---
name: fig:arm-servo-numbering
---
Jedyのサーボ順番割付
```

`KRS`サーボモータは ギア機構・制御通信基板・モータを一体化した
アクチュエータモジュールである． `KRS`サーボの通信は シリアル通信
(半二重非同期通信)でなされており
各モータ間はデイジーチェーンで接続が可能である．
コンフィギュレータ基板と呼ばれる通信基板を介して`UART`通信を行っている．
`Radxa`の充電基板にとりついている`ZH3`線のコネクタは`/dev/ttyAML1`というデバイスとして認識されている．
動作電圧は6-12Vであり，2セルのリポバッテリーから8Vを作り出すことで駆動している．

### セットアップ

#### USBシリアルアダプタの準備

本演習ではUSBシリアルアダプタ（FTDI）を使用してPCとJedyを接続する．
USBシリアルアダプタを使用する前に，以下のコマンドでudevルールの設定とユーザー権限を追加する必要がある．

```{code-block} console
$ curl -fsSL https://raw.githubusercontent.com/iory/rcb4/main/rcb4/assets/system/99-rcb4-udev.rules | sudo tee /etc/udev/rules.d/99-rcb4-udev.rules
$ sudo udevadm control --reload-rules
$ sudo udevadm trigger
$ sudo usermod -a -G dialout $USER
$ sudo usermod -a -G plugdev $USER
```

```{important}
グループへの追加後は，設定を反映させるために**PCを再起動**するか，一度ログアウトして再ログインする必要がある．
```

#### 接続と電源投入

本演習では安定化電源とUSBシリアルアダプタ（FTDI）を用いてJedyのセットアップを行う．以下の手順に従って接続を行うこと．

:::{danger}
**安定化電源の取り扱いに関する重要な注意事項**

<span style="color:red; font-weight:bold;">電源スイッチが白いボタンになっており，ACアダプタに電源が供給されていてここが緑色になっていると電源ONとなる．設定するときは必ず電源OFFにすること！</span>

<span style="color:red; font-weight:bold;">安定化電源がONになっているときにつまみは決して回さないこと！</span>

つまみを回すとサーボ制御基板やサーボが破損する可能性がある．また嫌なにおいが立ち込めるため細心の注意を払ってほしい．
:::

```{figure} fig/jedy-hardware-setup.png
---
name: fig:jedy-hardware-setup
---
Jedyのハードウェアセットアップ手順
```

##### セットアップ手順

1. **安定化電源の設定**：安定化電源を電圧7.4V，最大電流10Aに設定する（電源をOFFにした状態で設定すること）

2. **安定化電源のVH2ピンをJedyに接続**：安定化電源のVH2ピンコネクタをJedyのサーボ制御基板に接続する

3. **安定化電源の電源をON**：安定化電源の電源をONにして，白いボタンが緑色に点灯することを確認する

4. **FTDIのUSBシリアルアダプタの3線を接続**：写真のようにFTDIのUSBシリアルアダプタの3線（GND，TX，RX）を接続する

5. **USBシリアルアダプタの線をJedyに接続**：USBシリアルアダプタのケーブルをJedyのサーボ制御基板に接続する

6. **USBシリアルアダプタをPCに接続**：USBシリアルアダプタをPCに接続してセットアップ完了

:::{note}
実機をバッテリーで動かしたい場合には，{doc}`tips/robot-operation`を参照して，バッテリーを充電し接続すること．
:::

(sec-id-check)=
#### IDの確認

`Jedy`のサーボ電源を入れ自分のPCにJedyと接続したUSBシリアルを繋いで以下のコマンドを実行し22個のサーボモータすべてが
緑色に"found"となっていることを確認する．
**なお，これはデバイスにアクセスするプログラムなので，`jedy_bridge.launch`などを立ち上げていると実行できないことに注意する．**

```{code-block} console
$ rosrun kxr_controller scan_servo_ids.py $(rospack find jedy_ros1_bridge)/config/jedy_servo_config.yaml
Waiting servo control board connection ...
Attempt 1: Opened /dev/ttyUSB1 at 1000000 baud
ACK confirmed on attempt 1
Servo ID 0 (rarm_joint0) found
Servo ID 1 (larm_joint0) found
Servo ID 2 (rarm_joint1) found
Servo ID 3 (larm_joint1) found
Servo ID 4 (rarm_joint2) found
Servo ID 5 (larm_joint2) found
Servo ID 6 (rarm_joint3) found
Servo ID 7 (larm_joint3) found
Servo ID 8 (rarm_joint4) found
Servo ID 9 (larm_joint4) found
Servo ID 10 (rarm_joint5) found
Servo ID 11 (larm_joint5) found
Servo ID 12 (rarm_joint6) found
Servo ID 13 (larm_joint6) found
Servo ID 14 (rarm_gripper_joint) found
Servo ID 15 (larm_gripper_joint) found
Servo ID 20 (front_right_wheel_joint) found
Servo ID 21 (front_left_wheel_joint) found
Servo ID 22 (rear_right_wheel_joint) found
Servo ID 23 (rear_left_wheel_joint) found
Servo ID 32 (head_joint0) found
Servo ID 34 (head_joint1) found

All servo IDs in the configuration were found.
```

実行して`found`とでない場合は，ケーブルがささっていないことや，ID書き込みエラーやモータの故障が疑われるのでTAを呼んで`KRS`サーボの交換やIDの再書き込みを一緒に行おう．
サーボIDの書き込みには{doc}`robot-programming-3-2025`の発展課題にあった[Ubuntuでics-managerを使用する方法](tips/kondo-servo.html#ubuntuics-manager)を見ながらやってみよう．

```{code-block} console
$ rosrun kxr_controller scan_servo_ids.py $(rospack find jedy_ros1_bridge)/config/jedy_servo_config.yaml
Waiting servo control board connection ...
Attempt 1: Opened /dev/ttyUSB1 at 1000000 baud
ACK confirmed on attempt 1
Servo ID 0 (rarm_joint0) found
Servo ID 1 (larm_joint0) found
Servo ID 3 (larm_joint1) found
Servo ID 5 (larm_joint2) found
Servo ID 7 (larm_joint3) found
Servo ID 9 (larm_joint4) found
Servo ID 11 (larm_joint5) found
Servo ID 13 (larm_joint6) found
Servo ID 15 (larm_gripper_joint) found
Servo ID 20 (front_right_wheel_joint) found
Servo ID 21 (front_left_wheel_joint) found
Servo ID 22 (rear_right_wheel_joint) found
Servo ID 23 (rear_left_wheel_joint) found
Servo ID 32 (head_joint0) found
Servo ID 34 (head_joint1) found

Servo IDs and joint names not found:<br>
Servo ID 2 (rarm_joint1) not found<br>
Servo ID 4 (rarm_joint2) not found<br>
Servo ID 6 (rarm_joint3) not found<br>
Servo ID 8 (rarm_joint4) not found<br>
Servo ID 10 (rarm_joint5) not found<br>
Servo ID 12 (rarm_joint6) not found<br>
Servo ID 14 (rarm_gripper_joint) not found
```

実行してこのように`not found`と出ているサーボはケーブルが抜けていないかどうかを確認すべきである．一度サーボ電源用のスイッチを切り（青いLEDが消灯するのを確認！）サーボが断線していないかなどを確認する．原因がわかったら再度サーボ電源をオンしよう．
<span style="color:red">**ロボットにおいて電源周りやケーブルの接触不良は物理的に動くものであるため起こりうる．大事なことは問題が起きたときにどのように対処できるかというデバッグ能力であり，今のうちから技術を身に着けていってほしい．**</span>

### <span style="color:green">チェックポイント: サーボモータのIDの確認</span>

```{exercise} サーボモータのIDの確認
:label: ex_servo_id_check

[IDの確認](sec-id-check)にしたがってサーボモータのIDを確認せよ．全てのサーボモータが正しく認識されていることを確認すること．
```

## 演習への取り組み方と移動台車ロボットの共有方法

ロボットPCをROSのMasterとして班員のPCを複数台使うことでそれぞれ個別に指令値を送ったりセンサ値を表示したりしながらロボットを動かすことができる．
しかしロボットの動作自体は一人が送っている間は他の人が送ると動作が上書きされてしまうため，みんなで話ながらロボットを動かしていってほしい．

アドバンスドな話題だがネットワーク越しに複数台のPCから１台のロボットを動かすこともできる．接続設定の詳細については{doc}`tips/multi-pc`を参照すること．

また，これ以降は実機とシミュレーション（Gazebo）の両方に対応している．班員で相談して以下のいずれかの方法で取り組むとよい．

1.  班員全員で演習に取り組みながらチェックポイントごとにロボットの動作指令を送る人を交代する．チェックポイントは全員で取り組む．（全員がセットアップをする必要がある．）

2.  各々がシミュレーションで試してから交代で実機を利用する．演習課題は個人で取り組む．（1人が実機でトラブルと全員が取り組めなくなる．）

## `EusLisp`ロボットインターフェース`*ri*`からのセンサ値取得・台車駆動（シミュレーションにも対応）

`EusLisp`のロボットインターフェースで実際のロボットのセンサ値取得や台車駆動が行えることを確認する．

`EusLisp`の起動は通常のターミナルで直接実行するよりも， `emacs`で`M-x shell`とタイプし起動したターミナルで行うことをお勧めする．
一度打ち込んだコマンドは，`M-p`で履歴を遡ることができるので，同じコマンド打つ手間が減る．EusLisp (`roseus`)のより効率的な作業方法を知りたい場合は[roseusでの効率的な作業方法](tips/roseus-workflow.md)が参考になる．

### ロボットの起動方法

#### 実機
```{code-block} console
# USBシリアルアダプタを接続しJedyのサーボ電源が入っていることを確認して下記を実行
$ source ~/ros_ws/devel/setup.bash
$ roslaunch jedy_ros1_bridge jedy_bridge.launch
```

を起動すると`/dev/ttyUSBx`デバイスファイルにアクセスする`ROS`ノードが起動する．**実機を動かす場合はのセットアップが必要になる．**

#### シミュレーション

シミュレーションでは3つのターミナルを使用する．

**ターミナル1: ROS 1側でroscoreの起動**

```{code-block} console
$ source /opt/ros/one/setup.bash
$ roscore
```

**ターミナル2: ROS 2側でGazeboシミュレーションの起動**

```{code-block} console
$ source /opt/ros/jazzy/setup.bash
$ source ~/ros2_ws/install/setup.bash
$ export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
$ ros2 launch jedy_bringup jedy_gazebo.launch.py
```

```{note}
`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`を設定することで，ROS 2のDDS検出範囲をローカルホストに制限し，ネットワーク上の他のROS 2ノードとの干渉を防ぐ．
```

**ターミナル3: ROS 1とROS 2のブリッジ**

```{code-block} console
$ source /opt/ros/one/setup.bash
$ rosparam set /use_sim_time true
$ source /opt/ros/jazzy/setup.bash
$ source ~/ros2/bridge/install/setup.bash
$ export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
$ ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
```

#### 実機・シミュレーション両方
実機もしくはGazeboが立ち上がったら下記を実行する．

```{code-block} console
$ source ~/ros_ws/devel/setup.bash
$ roscd jedyeus/euslisp
$ roseus jedy-interface.l
```

```{code-block} lisp
irteusgl$ (jedy-init) ;; *jedy*と*ri*を作成する

;; （シミュレーションのみ）Virtual Atom s3のボタンの値を読んで表示する
irteusgl$ (ros::rate 10) ;; ループの周期を10Hzに設定する
irteusgl$ (do-until-key ;; Enterキーが押されるまでループする
            (send *ri* :spin-once) ;; subscribeしているトピックの更新が行われる．
            (ros::ros-info (format nil "atom s3 button state ~A" (send *ri* :state :atom-s3-button)))
            ;; 実行中にAtom S3のボタンをクリックすと値がクリック数に応じて変わる．
            (ros::sleep)) ;; ros::rateで設定した周期になるようにsleepする

;; （実機のみ）首の関節のエンコーダ値を読んで表示する
irteusgl$ (ros::rate 10) ;; ループの周期を10Hzに設定する
irteusgl$ (do-until-key ;; Enterキーが押されるまでループする
            (send *ri* :spin-once) ;; subscribeしているトピックの更新が行われる．
            (send *jedy* :angle-vector (send *ri* :state :potentio-vector))
            (ros::ros-info (format nil "Current head pitch joint angle ~A [deg]" (send *jedy* :head_joint1 :joint-angle)))
            (ros::sleep)) ;; ros::rateで設定した周期になるようにsleepする
;; 【ポイント】
;; - `do-until-key`によるループの生成
;; - ros::rateとros:sleepによる周期の制御

;; 台車を移動させる
irteusgl$ (send *ri* :go-velocity 10 0 0) ;; 大きすぎる数字を入れないように注意!
irteusgl$ (send *ri* :go-velocity 0. 0 -10)
;; 引数は，[前後方向速度] [左右方向速度] [旋回速度]
;; 前方に0.0[mm/s]，旋回10.0[deg/s]の速度で少し走って停止する
;; Jedyはロボットの正面がx+, なので初めて動かすときは注意．
;; シミュレーションではy方向に指令を送ると動きが変になる．

;; センサの値を取得する
irteusgl$ (ros::rate 1)
irteusgl$ (do-until-key
            (send *ri* :spin-once)  ;; subscribeしているトピックの更新が行われる．
            (ros::ros-info (format nil "roll pitch yaw ~A"
            (send *ri* :state :roll-pitch-yaw)))  ;; IMUから得られたロボットの姿勢が変わる．ロボットを動かしてみよう．
            (ros::sleep))
```

`(jedy-init)`は双腕移動台車ロボットの初期化関数である．`(jedy-init)`を実行すると`*jedy*`という大域変数にロボットモデルのインスタンスがセットされ，`EusLisp`の3Dビューア(``irtviewer``)に表示される [^1]. [IRTビューワの操作方法](tips/irtviewer.md)を参照のこと．同時に`*ri*`という大域変数に実ロボットインターフェースのインスタンスがセットされる．

単体のサーボモータやセンサ等の簡単な操作対象の場合は簡単な`Topic`の`pub`/`sub`のみで操作してもシステムは複雑になりにくい．一方，ロボットは多くの関節・アクチュエータを持つ上に，カメラやボタンなどに限らず手先・足先の６軸力センサやロボット全身の傾きを計測するセンサなど多くのセンサを搭載しておりそれらのセンサや関節をリアルタイムに同期しながら制御する必要があるため`Topic`の`pub`/`sub`のみでは全体システムが複雑になってしまう．そこでそれらのセンサや関節との通信機能を一つにまとめた**ロボット操作用インターフェース`*ri*`**を用いることで`(send *ri* :＜メソッド名＞)`のようにシンプルな統一的な表現でロボット全体を操作できるようになる．

上記の`:atom-s3-button`は`Atom S3`のボタンのクリック値を`EusLisp`で取得するものでありクリックしながら実行することで値が変わることを確認しよう．
また，`:go-velocity`は台車を駆動する速度指令を与えるメソッドである．

ロボットインターフェース`*ri*`の内部では{doc}`robot-programming-1-2025`で扱ったプログラムと同じように`topic`の`subscribe`/`publish`を行っている．

別ターミナルで

```{code-block} console
$ source ~/ros_ws/devel/setup.bash
$ rostopic echo /mecanum_drive_controller/reference
```

をしながら， `(send *ri* :go-velocity 10 0 0)` を呼んでみよう．明示的に`topic`の`subscribe`/`publish`を書かずともロボットのセンサ・アクチュエータにアクセスできる点がロボットインターフェースの利点のひとつである．

他にも，

```{code-block} lisp
irteusgl$ (send *ri* :state :roll-pitch-yaw)    ;; IMUセンサの値からロボットの姿勢を取得する
irteusgl$ (send *ri* :state :potentio-vector)   ;; 実機のサーボの角度を取得する
irteusgl$ (send *ri* :go-velocity x y theta)    ;; 速度指令で動かす．単位は x,y:[mm/s], theta:[deg/s]
```

などのセンサ取得や指令値送信ができる．実ロボットのインターフェースプログラムは，[jedy/jedyeus/euslisp/jedy-interface.l](https://github.com/jsk-enshu/robot-programming/blob/master/jedy/jedyeus/euslisp/jedy-interface.l)を参照してみよう．

`EusLisp`の条件分岐や繰り返しを用いてセンサ情報に基づいてロボットが行動するプログラムを書いてみよう．

```{code-block} console
$ source ~/ros_ws/devel/setup.bash
$ roscd jedyeus/euslisp
$ roseus jedy-interface.l
```

実機かシミュレーションでそれぞれ次のコードを実行してみよう．

#### 実機用

```{code-block} lisp
irteusgl$ (jedy-init) ;; *jedy*,*ri*を作成する
irteusgl$ (ros::rate 10)
irteusgl$ (send *jedy* :angle-vector (send *ri* :state :potentio-vector))
irteusgl$ (setq pitch-joint (send *jedy* :head_joint1 :joint-angle))
irteusgl$ (ros::ros-info (format nil "current head pitch angle ~A" pitch-joint))
irteusgl$ (while t
            (send *ri* :go-velocity 10 0 0)
            (send *jedy* :angle-vector (send *ri* :state :potentio-vector))
            (ros::ros-info (format nil "current head pitch angle ~A" (send *jedy* :head_joint1 :joint-angle)))
            (when (> (abs (- pitch-joint (send *jedy* :head_joint1 :joint-angle))) 30)
              (send *ri* :go-velocity 0 0 0)
              (return-from nil nil))
            (send *ri* :spin-once)
            (ros::sleep))
```

これは初期の首のピッチ軸の関節から30度以上動くと台車が止まるプログラムである．

#### シミュレーション用
```{code-block} lisp
irteusgl$ (jedy-init) ;; *jedy*,*ri*を作成する
irteusgl$ (ros::rate 10)
irteusgl$ (while t
            (send *ri* :go-velocity 10 0 0)
            (setq button-state (send *ri* :state :atom-s3-button))
            (ros::ros-info (format nil "button state ~A" button-state))
            (when (not (= button-state 0))
              (send *ri* :send-cmd-vel-raw 0 0 0)
              (return-from nil nil))
            (send *ri* :spin-once)
            (ros::sleep))
```

これは，`Virtual Atom S3`のボタンがクリックされるまで前進指令を繰り返し送り続けるプログラムである．プログラムはファイルに保存しておくと以下のように実行できて便利である．

```{code-block} console
$ source ~/ros_ws/devel/setup.bash
$ roscd jedyeus/exercise/
$ roseus checkpoint2-1-go-forward.l
```

で同様の動作が実行される．

#### 実機・シミュレーション両方

また，`IMU`の値を用いるサンプルもある．`IMU`の傾きを読んで傾きを変えると速度指令を送ることをやめるプログラムとなっている．

```{code-block} console
$ source ~/ros_ws/devel/setup.bash
$ roscd jedyeus/exercise/
$ roseus checkpoint2-2-go-forward-imu.l
```

これを参考にして次のチェックポイントに取り組もう．

### <span style="color:green">チェックポイント: センサとロボットインターフェースを使った反応行動</span>

```{exercise} センサとロボットインターフェースを使った反応行動
:label: ex_sensor_ri

`EusLisp`の関数やメソッドおよび実ロボットインターフェース`*ri*`などを駆使してロボットの反応行動を行うプログラムを書いてみよう．使うセンサは何でもよく(ボタン，`IMU`，エンコーダー値，．..etc)，反応行動も何でもよい(前進後退切り替わる，`IMU`の値に応じてぶつかると前進が停止する．..etc)がセンサ値・反応行動はそれぞれ２つ以上を組み合わせること．

**例：**
ボタンを押されると前進しもう一回ボタンを押されるとストップする．首の`yaw`軸 (`head_joint0`)を右に動かすと右に進み左に動かすと左に進むプログラム．
```

## アームロボットのソフトウェア（シミュレーションにも対応）

本小節ではアームロボットを動かすためのソフトウェアについて説明する．

### ロボットアームの起動

**本章のコマンドはアームを動かすのでアームが何かにぶつからないように周囲に注意しながら演習を行うこと．**

#### 実機

```{code-block} console
# USBシリアルアダプタを接続しJedyのサーボ電源が入っていることを確認して下記を実行．すでに行っていて起動している場合はしなくてよい．
$ source ~/ros_ws/devel/setup.bash
$ roslaunch jedy_ros1_bridge jedy_bridge.launch
```

を起動すると`/dev/ttyUSBx`デバイスファイルにアクセスする`ROS`ノードが起動する．**実機を動かす場合はのセットアップが必要になる．**

#### シミュレーション

シミュレーションでは3つのターミナルを使用する．上ですでにGazebo環境を立ち上げている場合はしなくてよい．

**ターミナル1: ROS 1側でroscoreの起動**

```{code-block} console
$ source /opt/ros/one/setup.bash
$ roscore
```

**ターミナル2: ROS 2側でGazeboシミュレーションの起動**

```{code-block} console
$ source /opt/ros/jazzy/setup.bash
$ source ~/ros2_ws/install/setup.bash
$ export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
$ ros2 launch jedy_bringup jedy_gazebo.launch.py
```

```{note}
`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`を設定することで，ROS 2のDDS検出範囲をローカルホストに制限し，ネットワーク上の他のROS 2ノードとの干渉を防ぐ．
```

**ターミナル3: ROS 1とROS 2のブリッジ**

```{code-block} console
$ source /opt/ros/one/setup.bash
$ rosparam set /use_sim_time true
$ source /opt/ros/jazzy/setup.bash
$ source ~/ros2/bridge/install/setup.bash
$ export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
$ ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
```

ros1_bridgeが正常に起動すると，ROS 1のtopicとROS 2のtopicが自動的に変換されるようになる．

### PythonプログラマのためのEusLisp入門

本演習では，ロボット制御にEusLispという言語を使用する．多くの学生にとってPythonの方が馴染みがあると思われるため，ここではPythonとの対比を通じてEusLispの書き方を理解できるようにする．

:::{figure} fig/euslisp-and-python.png
:align: center
:width: 700px
:name: fig:euslisp-python-comparison

PythonとEusLispの構文対比
:::

#### 基本的な対応関係

| 概念 | Python | EusLisp | 説明 |
|------|--------|---------|------|
| **関数呼び出し** | `jedy_init()` | `(jedy-init)` | 関数名を括弧で囲み，括弧の前に配置 |
| **変数代入** | `a = 1` | `(setq a 1)` | `setq`を使用して代入 |
| **メソッド呼び出し** | `ri.servo_off_all()` | `(send *ri* :servo-off-all)` | `send`でオブジェクトにメソッドを送信 |
| **引数付きメソッド** | `ri.go_velocity(0.1, 0, 0)` | `(send *ri* :go-velocity 0.1 0 0)` | 引数をカンマなしで列挙 |
| **リスト/配列** | `[0, 0, 0, 0, 0, 0]` | `#f(0 0 0 0 0 0)` | `#f`は浮動小数点数ベクトル |
| **入れ子の関数** | `jedy.inverse_kinematics(make_coords(pos=[370, 0, 150]))` | `(send *jedy* :inverse-kinematics (make-coords :pos #f(370 0 150)))` | 内側から外側へ評価される |

#### 重要なポイント

1. **前置記法（Prefix Notation）**
   - Python: `a + b` （演算子が中央）
   - EusLisp: `(+ a b)` （演算子が先頭）

2. **括弧の役割**
   - Python: 関数呼び出しや優先順位の制御
   - EusLisp: 関数呼び出しと式の境界を表す（すべての式が括弧で囲まれる）

3. **メソッド呼び出し**
   - Python: `object.method(arg1, arg2)`
   - EusLisp: `(send object :method arg1 arg2)`
   - EusLispでは`:method`のようにコロンで始まる**キーワードシンボル**でメソッド名を指定

4. **変数の命名規則**
   - EusLispでは，グローバル変数は`*variable*`のようにアスタリスクで囲む慣習がある
   - 例: `*ri*`（ロボットインターフェース），`*jedy*`（ロボットモデル）

詳細な説明については，[PythonプログラマのためのEusLisp](tips/python-to-euslisp.md)を参照のこと．

### `EusLisp`のロボットインターフェース`*ri*`からのアームロボットの制御

以下のコマンドにより
`EusLisp`のロボットインターフェースからアームロボットを動かすことができる．

```{code-block} console
$ source ~/ros_ws/devel/setup.bash
$ roscd jedyeus/euslisp
$ roseus jedy-interface.l
```

```{code-block} lisp
irteusgl$ (jedy-init) ;; ロボットの実機インターフェース*ri*とロボットモデル*jedy*を生成．
irteusgl$ (send *jedy* :reset-pose) ;; 関節角度を:reset-poseにセット
irteusgl$ (send *irtviewer* :draw-objects) ;; 描画を更新
irteusgl$ (send *ri* :angle-vector (send *jedy* :angle-vector) 4000) ;; ここではまだサーボonになっていないため実機は動かない
          ;; 第一引数は目標関節角度 [deg]，第二引数は補間時間 [ms],4秒で目標到達角度になる．
```

`(jedy-init)` 関数は`*jedy*`変数と`*ri*`を生成する． `*jedy*`は，
`EusLisp`のビューワ(`irtviewer`)上に描画されたロボットモデルを表す．
`*ri*`は実際のロボット（及びシミュレーション内のロボット）のセンサ値取得・動作指令を行うためのロボットインターフェースである．

ロボットを動かすためには，以下のように，ロボットモデル(`*jedy*`)の関節を所望の角度にしてからこれを実機(`*ri*`)へ送る．

1.  `(send *jedy* :reset-pose)`でロボットモデルをリセット姿勢にして

2.  `(send *jedy* :angle-vector)`でロボットモデルの関節角度を取得して

3.  `(send *ri* :angle-vector (send *jedy* :angle-vector))`でこれを実機へと送る．

1.のロボットモデルの姿勢を決定する部分は，のように関節角度を直接指定したり[EusLispの逆運動学メソッド](tips/euslisp-ik.md)のように逆運動学を用いてもよい．

(sec-krs-servo-onoff)=
#### `KRS`サーボのON/OFF

##### EusLispからのサーボON/OFF

以下のコマンドでサーボのON/OFFが切り替えられる．

```{code-block} console
$ source ~/ros_ws/devel/setup.bash
$ roscd jedyeus/euslisp
$ roseus jedy-interface.l
```

```{code-block} lisp
irteusgl$ (jedy-init) ;; アーム＋台車ロボットの*ri*と*jedy*を生成
irteusgl$ (send *ri* :servo-on) ;; 全軸サーボがONになる
irteusgl$ (send *ri* :servo-off) ;; 全軸サーボがOFFになる．
;; 関節名を指定してONにする
irteusgl$ (send *ri* :servo-on :names (list "head_joint0" "head_joint1")) ;; 頭のみサーボがONになる．
;; 使用できる関節名は以下のように確認できる．
irteusgl$ (send-all (send *jedy* :joint-list) :name)
("rarm_joint0" "rarm_joint1" "rarm_joint2" "rarm_joint3" "rarm_joint4" "rarm_joint5" "rarm_joint6" "rarm_gripper_joint" "larm_joint0" "larm_joint1" "larm_joint2" "larm_joint3" "larm_joint4" "larm_joint5" "larm_joint6" "larm_gripper_joint" "head_joint0" "head_joint1")
;; 頭部の関節名を指定して角度を変える
irteusgl$ (send *jedy* :head_joint0 :joint-angle 20)
irteusgl$ (send *jedy* :head_joint1 :joint-angle 50)
;; 関節指令を実機に反映させる (3000msかけておくる)
irteusgl$ (send *ri* :angle-vector (send *jedy* :angle-vector) 3000)
```

##### rqtプラグインを使ったサーボON/OFF（実機のみ）

```{important}
この方法は**実機のみ**で使用可能である．シミュレーション環境では使用できない．
```

実機の`jedy_bridge.launch`が起動している状態で，以下のコマンドを実行すると，GUIからサーボのON/OFFや台車・関節の制御が行える：

```{code-block} console
$ source ~/ros_ws/devel/setup.bash
$ roslaunch jedy_ros1_bridge rqt.launch
```

:::{figure} fig/servo-on-rqt-plugin.png
:align: center
:name: fig:servo-on-rqt-plugin
:width: 100%

rqtプラグインによるロボット制御画面
:::

起動すると以下の3つのパネルが表示される：

- **左側**: 台車の速度指令を送るパネル
- **真ん中**: 関節の位置指令を送るパネル
- **右側**: サーボのON/OFFを切り替えるパネル

右側のパネルで各関節のON/OFFボタンをクリックすることで，個別にサーボのON/OFFを切り替えることができる．
また，`All Servo ON`ボタンで全軸を一度にONにしたり，`All Servo OFF`ボタンで全軸を一度にOFFにすることもできる．
この方法を使うことで，コマンドラインからプログラムを書かずに直感的にロボットを操作できるので，ぜひ試してみよう．

特に`KRS`サーボモータは，緑，青，赤サーボの３種類があるが，
赤サーボのみモータに過負荷や過電流がかかったり高温になったとき，サーボがきれ．これは，内部の基板で電流値や温度値を監視し制御器へのエラーフラグを立ててサーボを落とすような処理が組み込まれている．
しかし，今回の演習で使用しているものは緑及び青サーボとなる．
ロボットの関節に負荷がかかるような姿勢に長時間しているとサーボが焼ききれるため，危ないときは背面の黒いスイッチを押してサーボへの供給をとめるか，
`(send *ri* :servo-off)`メソッドでサーボをきってほしい．
サーボが焼ききれた場合にはサーボパーツの交換をしたもらうこととなるため，時間がかかるということを念頭に置いてほしい．

他にも関節ゲインを変更するなど細かい設定や センサ値取得も可能である．興味のある人は， [近藤サーボの製品ページ](https://kondo-robot.com/product/03146)を参照してみてほしい．

(sec-direct-joint-angle)=
#### 関節角度の直接指令による動作生成

```{code-block} lisp
irteusgl$ (send *jedy* :angle-vector #f(90.0 -4.0 -30.0 -100.0 -3.0 -88.0 -1.0 0.0 -90.0 4.0 30.0 -100.0 3.0 -88.0 1.0 -1.0 0.0 0.0))
;; 根本から先端までの関節角度 [deg] (グリッパは含まない)
;; #fはnumpyでいうところのnp.arrayのようなものだが，中で計算することはできない．
;; そのため中で計算をさせたいときはfloat-vectorを使う
irteusgl$ (send *jedy* :angle-vector
 (float-vector (- 90.0 10) -4.0 -30.0 -100.0 -3.0 -88.0 -1.0 0.0 -90.0 4.0 30.0 -100.0 3.0 -88.0 1.0 -1.0 0.0 0.0))
```

関節軸数が多くなるとこのような指定の方法は難しくなってくるので，軸を指定して角度を設定する方法もある．
関節角度の直接指定など．

```{code-block} lisp
# jointのリストの取得
irteusgl$ (send *jedy* :joint-list)
(#<rotational-joint #X6771a18 arm_joint1> #<rotational-joint #X4d7d2e8arm_joint2> (略) ...
# ある関節のインスタンスを取得
irteusgl$ (send *jedy* :head_joint1)
#<rotational-joint #X52db768 head_joint1>
# ある関節軸を動かす(順運動学)
irteusgl$ (send (send *jedy* :head_joint1) :joint-angle -90)
```

繰り返し処理を追加すると以下のようにエンターキーを押すまでロボットがうなずき続ける．

```{code-block} lisp
;; 頭部のみ個別にservo-on
(send *ri* :servo-on :names (list "head_joint0" "head_joint1"))

;; 頭部の関節名を指定して角度を変える
(send *jedy* :head_joint0 :joint-angle 20)
(send *jedy* :head_joint1 :joint-angle 50)

(send *irtviewer* :draw-objects)

;; 関節指令を実機に反映させる (3000msかけておくる)
(send *ri* :angle-vector (send *jedy* :angle-vector) 3000)

;; キーが押されるまでうなずき続ける
(do-until-key
  (send *jedy* :head_joint0 :joint-angle 0)
  (send *jedy* :head_joint1 :joint-angle 90)
  (send *ri* :angle-vector (send *jedy* :angle-vector) 1000)
  (send *ri* :wait-interpolation)  ;; 補間がおわるのを待つ
  (send *jedy* :head_joint0 :joint-angle 0)
  (send *jedy* :head_joint1 :joint-angle 0)
  (send *ri* :angle-vector (send *jedy* :angle-vector) 1000)
  (send *ri* :wait-interpolation)  ;; 補間がおわるのを待つ
)
```

(sec-inverse-kinematics)=
#### 逆運動学による動作生成

手先を目標物に伸ばすというように，手先作業空間での目標に基づいてロボットの運動を生成するには，逆運動学(IK)を用いる．
逆運動学は端的には，「目標位置まで手先を動かすための関節角度はいくらか？」を求める演算である．

```{code-block} console
$ source ~/ros_ws/devel/setup.bash
$ roseus
```

```{code-block} lisp
irteusgl$ (load "package://jedyeus/euslisp/keyboard-ik-sample.l")
```

を実行してみてほしい．ビューワのウィンドウがアクティブなときにキーボード入力をするとアームロボットの腕が動いていると思う．これは，逆運動学を解くことにより手先の指令値に応じて各関節角度を計算しビューワで描画しながら実機へ指令しているサンプルである．

このサンプルの中ではキーボード入力に応じて以下の逆運動学メソッドを呼び出しロボットモデルの関節角度を計算している．

また実機もしくは`gazebo`上のロボットに姿勢を送りたい場合には`keyboard-ik-sample.l`のプログラム中の `(jedy-init :without-ri t)`となっている部分を`(jedy-init :without-ri nil)`とすると実機もしくは`gazebo`上のロボットに姿勢が反映される．

:::{figure} fig/keyboard_ik_sample.jpg
:align: center
:name: fig:keyboard_ik_sample

keyboard-ikによるロボットの姿勢の反映
:::

```{code-block} lisp
irteusgl$ (send *jedy* :reset-pose) ;; 初期姿勢によってはIKが解けないことがある
irteusgl$ (send *jedy* :rarm :inverse-kinematics
            (make-coords :pos (float-vector 120 -90 -30) :rpy (list (deg2rad 30) (deg2rad 0) (deg2rad 0)))
            :debug-view :no-message)
```

逆運動学メソッドの詳細な使い方は[EusLispの逆運動学メソッド](tips/euslisp-ik.md)，を参照のこと．

#### <span style="color:green">チェックポイント: アームロボットの制御</span>

```{exercise} アームロボットの制御
:label: ex_arm_control

1.  [`KRS`サーボのON/OFF](sec-krs-servo-onoff)および[関節角度の直接指令による動作生成](sec-direct-joint-angle)にしたがって頭部以外のモータを動かしてみよ．また，`irtviewer`のロボットモデルと実際のロボットの姿勢が一致していることを確認せよ[^2]．操作方法は[IRTビューワ(irtviewer)の操作方法](tips/irtviewer.md)が役に立つ．

2.  [逆運動学による動作生成](sec-inverse-kinematics)にしたがって逆運動学でロボットの姿勢を生成し実機に送ってみよ．
```

なお，実際に`Jedy`の定義されてるファイルは，[jedy/jedyeus/euslisp/jedy.l](https://github.com/jsk-enshu/robot-programming/tree/master/jedy/jedyeus/euslisp/jedy.l)にある．

## 本日の発展課題

### 課題1(発展)

{doc}`robot-programming-1-2025`でカメラ画像を使って認識を行った．物体の姿勢を推定し，その推定結果からロボット実機の姿勢を決定することは重要である．
ここでは，カメラのRGB画像からマーカーボックスの位置を推定し，ロボットの逆運動学計算を行いピックアップする方法を紹介するので，希望者はTAか先生から`AprilTag`Boxを貸りて，ぜひ挑戦してみてほしい．
また，サンプルで用意しているプログラムはマーカーが地面から垂直にある場合に掴みにいくものとなっているため，横からつかみにいったほうが逆運動学が解け，掴みやすいことや，
双腕を使って掴みやすい位置にアシストしなが持っていくなど発展させることが可能であるので，こちらもぜひ挑戦してみてほしい．

今回使用するマーカーは[AprilTag](https://april.eecs.umich.edu/software/apriltag)と呼ばれるものである．比較的小さいマーカーでも認識しやすい．

#### Intel RealSense D405のセットアップ

本課題ではIntel RealSense Depth Camera D405を使用する．D405は近距離（7cm〜50cm）での深度測定に優れたステレオビジョン方式の深度カメラである．詳細は[Intel RealSense D405のセットアップと活用](tips/realsense-d405.md)を参照のこと．

**librealsense2のインストール**

まず，Intel公式のAPTリポジトリからlibrealsense2をインストールする：

```{code-block} console
# GPGキー用のディレクトリを作成
$ sudo mkdir -p /etc/apt/keyrings

# Intel RealSenseのGPGキーをダウンロード
$ curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null

# APTリポジトリを追加
$ echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | sudo tee /etc/apt/sources.list.d/librealsense.list

# パッケージリストを更新
$ sudo apt-get update

# librealsense2をインストール
$ sudo apt-get install librealsense2-utils librealsense2-dev
```

**ROS 1用RealSenseパッケージのセットアップ**

次に，ROS 1用のRealSenseパッケージをビルドする：

```{code-block} console
# ROS 1環境をセットアップ
$ source /opt/ros/one/setup.bash

# ROS 1ワークスペースに移動
$ cd ~/ros_ws/src

# RealSense ROS 1パッケージをクローン（ros1-legacyブランチ）
$ git clone -b ros1-legacy https://github.com/IntelRealSense/realsense-ros

# 依存パッケージをインストール
$ cd ~/ros_ws
$ rosdep install --from-paths src --ignore-src -y -r

# realsense2_cameraパッケージをビルド
$ catkin build realsense2_camera
```

#### カメラの接続と起動

**D405カメラをUSB 3.0ケーブルでPCに接続する．**

```{important}
USB 3.0ポート（青色のポート）に接続すること．USB 2.0では十分な帯域が確保できず，画像転送が遅くなる可能性がある．
```

**カメラノードの起動**

以下のコマンドでカメラノードを起動する：

```{code-block} console
$ source ~/ros_ws/devel/setup.bash
$ roslaunch jedy_ros1_bridge d405.launch
```

**AprilTag認識ノードの起動**

別のターミナルで以下のコマンドを実行し，AprilTag認識ノードを起動する：

```{code-block} console
$ source ~/ros_ws/devel/setup.bash
$ roslaunch jedy_ros1_bridge apriltag_detection.launch
```

**認識結果の確認**

さらに別のターミナルで以下のコマンドを実行し，認識結果を可視化する：

```{code-block} console
$ source ~/ros_ws/devel/setup.bash
$ rqt_image_view
```

`rqt_image_view`が起動したら，トピックのドロップダウンメニューから`tag_detection`という名前のついたトピック（例：`/camera/color/tag_detections_image/compressed`）を選択する．
この状態でAprilTagBoxをカメラに映すと，認識した位置にマーカーのIDが表示される．

:::{figure} fig/recognition_apriltag.jpg
:align: center
:name: fig:recognition_apriltag

`AprilTag`の認識結果
:::

#### EusLispサンプルプログラムの実行

認識が正常に動作していることを確認したら，EusLispのサンプルプログラムを実行してみよう．
このプログラムは`AprilTag`を認識した位置に対して逆運動学を解き，ロボットアームを伸ばすものである．

```{code-block} console
$ source ~/ros_ws/devel/setup.bash
$ roscd jedyeus/euslisp
$ roseus sample-pickup.l
```

逆運動学が解けた場合，腕を伸ばす様子が`irtviewer`に表示される．

::::{grid} 2
:gutter: 2

:::{grid-item}
:::{figure} fig/recognition_apriltag_with_ik.jpg
:align: center
:name: fig:recognition-apriltag-with-ik

認識した位置にモデル上で手を伸ばしている様子
:::

:::{grid-item}
:::{figure} fig/recognition_apriltag_with_ik_readl_robot.jpg
:align: center
:name: fig:recognition-apriltag-with-ik-real-robot

実機で手を伸ばしている様子
:::

::::

実機に実際に姿勢を送りたい場合は，`sample-pickup.l`の`(setq *send-ri-flag* nil)`と書かれているところを`(setq *send-ri-flag* t)`とする．

ここまで実行してマーカーが滑る場合には，摩擦を上げるためのゴムシートを貸し出すのでロボットの手先に取り付けてみてほしい．
このような工夫で成功率が大きく変わることを体験してみてほしい．


### 課題2.1 (発展)

Jedyの`follow joint trajectory`コントローラーを別々に動かすプログラムを書いて`head_controller`のみを動かすプログラム，`rarm_controller`のみを動かすプログラムで分けて実行することでコントローラーごとに独立して操作できることを確認してみよう．

#### コントローラーの仕組み

EusLispのロボットインターフェースでは，複数のコントローラーを独立して制御する仕組みが実装されている．
[jedy-interface.l](https://github.com/jsk-enshu/robot-programming/blob/master/jedy/jedyeus/euslisp/jedy-interface.l)と[robot-interface.l](https://github.com/jsk-ros-pkg/jsk_pr2eus/blob/master/pr2eus/robot-interface.l)を参考にして，その仕組みを理解しよう．

##### コントローラーの定義

各コントローラーは以下のようにメソッドとして定義されている（[jedy-interface.l 128-163行目](https://github.com/jsk-enshu/robot-programming/blob/master/jedy/jedyeus/euslisp/jedy-interface.l#L128-L163)）：

```lisp
(:head-controller
 ()
 (list
  (list
   (cons :controller-action "head_controller/follow_joint_trajectory")
   (cons :controller-state "head_controller/state")
   (cons :action-type control_msgs::FollowJointTrajectoryAction)
   (cons :joint-names (list "head_joint0" "head_joint1")))))

(:larm-controller
 ()
 (list
  (list
   (cons :controller-action "larm_controller/follow_joint_trajectory")
   (cons :controller-state "larm_controller/state")
   (cons :action-type control_msgs::FollowJointTrajectoryAction)
   (cons :joint-names (list "larm_joint0" "larm_joint1" "larm_joint2"
                            "larm_joint3" "larm_joint4" "larm_joint5"
                            "larm_joint6" "larm_gripper_joint")))))

(:rarm-controller
 ()
 (list
  (list
   (cons :controller-action "rarm_controller/follow_joint_trajectory")
   (cons :controller-state "rarm_controller/state")
   (cons :action-type control_msgs::FollowJointTrajectoryAction)
   (cons :joint-names (list "rarm_joint0" "rarm_joint1" "rarm_joint2"
                            "rarm_joint3" "rarm_joint4" "rarm_joint5"
                            "rarm_joint6" "rarm_gripper_joint")))))
```

##### コントローラーテーブルへの登録

初期化時に，各コントローラーが`controller-table`というハッシュテーブルに登録される（[jedy-interface.l 54-64行目](https://github.com/jsk-enshu/robot-programming/blob/master/jedy/jedyeus/euslisp/jedy-interface.l#L54-L64)）：

```lisp
(dolist (l (list
             (cons :larm-controller "larm_controller/follow_joint_trajectory")
             (cons :rarm-controller "rarm_controller/follow_joint_trajectory")
             (cons :head-controller "head_controller/follow_joint_trajectory")))
  (let ((type (car l))
        (name (cdr l))
        action)
    (setq action (find-if #'(lambda (ac) (string= name (send ac :name)))
                          controller-actions))
    (setf (gethash type controller-table) (list action))))
```

##### `:angle-vector`メソッドのシグネチャ

[robot-interface.l 430-445行目](https://github.com/jsk-ros-pkg/jsk_pr2eus/blob/master/pr2eus/robot-interface.l#L430-L445)を見ると，`:angle-vector`メソッドは以下のシグネチャを持つ：

```lisp
(:angle-vector
 (av &optional (tm nil) (ctype controller-type) (start-time 0)
     &key (scale 1) (min-time 1.0) ...)
 "Send joint angle to robot
- av : joint angle vector [deg]
- tm : time to goal in [msec]
- ctype : controller method name
- start-time : time to start moving
...")
```

**重要**: `ctype`（controller type）は第3引数の**位置引数**であり，キーワード引数ではない点に注意する．

#### 実装例

##### 頭部のみを動かす例

```lisp
;; jedy-interfaceの起動
(jedy-init)

;; 頭部のサーボをON
(send *ri* :servo-on :names (list "head_joint0" "head_joint1"))

;; 頭部の角度を設定
(send *jedy* :head_joint0 :joint-angle 30)
(send *jedy* :head_joint1 :joint-angle 45)

;; 頭部コントローラーのみを使って動作指令を送る
;; 引数: (angle-vector 補間時間[ms] controller-type)
(send *ri* :angle-vector (send *jedy* :angle-vector) 2000 :head-controller)
(send *ri* :wait-interpolation)
```

##### 右腕のみを動かす例

```lisp
;; 右腕のサーボをON
(send *ri* :servo-on :names (list "rarm_joint0" "rarm_joint1" "rarm_joint2"
                                   "rarm_joint3" "rarm_joint4" "rarm_joint5"
                                   "rarm_joint6" "rarm_gripper_joint"))

;; 右腕の姿勢を設定（逆運動学を使用）
(send *jedy* :rarm :inverse-kinematics
      (make-coords :pos #f(300 -200 100)))

;; 右腕コントローラーのみを使って動作指令を送る
(send *ri* :angle-vector (send *jedy* :angle-vector) 3000 :rarm-controller)
(send *ri* :wait-interpolation)
```

##### 複数のコントローラーを同時に独立して動かす例

```lisp
;; 頭部と右腕を同時に動かす（それぞれ独立したコントローラー）
(send *jedy* :head_joint1 :joint-angle 60)
(send *jedy* :rarm :inverse-kinematics
      (make-coords :pos #f(350 -150 120)))

;; 頭部コントローラーに指令を送る（補間時間: 2000ms）
(send *ri* :angle-vector (send *jedy* :angle-vector) 2000 :head-controller)

;; 右腕コントローラーに指令を送る（補間時間: 3000ms, 頭部とは独立して動く）
(send *ri* :angle-vector (send *jedy* :angle-vector) 3000 :rarm-controller)

;; 両方の動作が終わるまで待つ
(send *ri* :wait-interpolation :head-controller)
(send *ri* :wait-interpolation :rarm-controller)
```

#### 重要なポイント

1. **controller typeは第3引数の位置引数**として渡す（キーワード引数ではない）
   ```lisp
   ;; 正しい
   (send *ri* :angle-vector av 2000 :head-controller)

   ;; 間違い（これは動かない）
   (send *ri* :angle-vector av 2000 :controller-type :head-controller)
   ```

2. **各コントローラーは独立**しているため，異なる補間時間で同時に動作させることができる

3. **デフォルト**（`:default-controller`）では全てのコントローラーが使用される

4. **`controller-table`**というハッシュテーブルで管理されているため，拡張が容易である

この仕組みにより，ロボットの異なる部位を独立して制御でき，より複雑な動作パターンを実現できる．

### 課題2.2 (発展)

他の班のJedyと協力して片方を動かすと片方のJedyが動くプログラムを作成してみよ．


### ヒューマノイドロボットJAXONのシミュレーション（発展課題）

本章では，Gazeboシミュレーション上でヒューマノイドロボットJAXONを動かす方法を説明する．
本演習ではROS 2でGazeboシミュレーションを起動し，EusLispから制御する際にはros1_bridgeを用いてROS 1とROS 2を橋渡しする．
JAXONのGazeboシミュレーションはROS 2パッケージ`cart_humanoid_ros2`で提供されている．
[JaxonのシミュレーションでリアルタイムIKを動かす](tips/realtime-ik.md)を参照のこと．

### 課題3(発展)

`EusLisp`の`robot`インターフェース`*ri*`は双腕移動台車ロボットだけでなくヒューマノイド`JAXON`にも用意されており
移動台車アームロボットとヒューマノイド`JAXON`は`*ri*`に共通のメソッドを送信することで制御することができる．
また，手先の目標位置を`ik_＜ターゲット名＞_tgt`という`topic`を`ROS`ノード間で通信しそれを目標値としたIKを解いてロボットの姿勢を決定しているためロボット固有の属性（関節の数や関節軸の向き等）にほとんど依存しないシステムの上で動作している．
従って本日の演習の`launch`ファイルと`euslisp`のファイルを少し書き換えるだけで移動台車アームロボットと`JAXON`を同じようなプログラムでIKによる制御を行うことができるということである．
以下の手順でと同様に手先位置で実機またはシミュレーション上のアームロボットの手先をリアルタイムに動かしてみよ．

### 課題4(発展)

以下の手順でと同様にカメラで認識した手先位置で実機またはシミュレーション上のアームロボットの手先をリアルタイムに動かしてみよ．

### 課題5(発展)

機械情報冬学期演習「デジタルファブリケーション」で自分が設計した部品の立体形状を`STL`ファイルで保存していると思うがその`STL`をGazeboシミュレーション内に出現させよ．

:::{figure} fig/my_model.jpg
:align: center
:name: fig:sample_avatar

JaxonのgazeboにSTLファイルを表示させている例
:::

`cart_humanoid_ros2`パッケージの`cart_humanoid_gazebo.launch.py`で起動するGazeboのworldファイルは[robot-programming/cart_humanoid_ros2/worlds/humanoid_workspace.world](https://github.com/jsk-enshu/robot-programming/blob/master/cart_humanoid_ros2/worlds/humanoid_workspace.world)でありその中の以下の部分をコメントインし


```{console-block} xml
<!-- my_model -->
  <model name="my_model" >
    <pose>1.0 -0.4 1 0 0 0</pose>
    <include>
      <uri>model://my_model</uri>
    </include>
  </model>
```

以下の`my_mesh.stl`を置き換え，`model.sdf`内のscaleなどを調整すると良い．

```{code-block} console
$ cd ~/ros2_ws/src/robot-programming/cart_humanoid_ros2/worlds/model/my_model/
$ ls
model.config  model.sdf  my_mesh.stl
```

※STLの用意や読み込みがどうしても出来ない人はhumanoid_workspace.world内のmy primitive modelに関する部分をコメントインしてプリミティブな幾何形状の組み合わせによる自作モデルを作成しよう

### 課題6(発展)

EusLispのrobotインターフェースに実装されている`inverse-kinematics`機能([参考](tips/euslisp-ik.md))を用いて
任意の物体にアームで触れるプログラムを作成せよ．（※可能であればグリッパを駆使して物体をピックアップできるとなお良い）

### 課題7(発展)

二日目の演習でJedyからJaxonを動かす様子をみたと思う．実機のJedyを動かしシミュレーターのJaxonを動かせるようにしてみよ．

### 課題8(発展)

これまでの授業や演習で学んだロボットプログラミング技法を組み合わせ，
自由な発想でロボットのデモプログラムを作成せよ．


[^1]: `irtviewer`は，`EusLisp`で実装された3次元ビジュアライゼーションツールである．ロボットモデルや幾何形状をインタラクティブに表示できる．

[^2]: `irtviewer`のロボットモデルと実際のロボットの姿勢が一致することでシミュレーションと実機の動作が正しく連携していることを確認できる．

