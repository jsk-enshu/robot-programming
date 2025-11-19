# リアルタイムIKによるオンライン操縦

本来，制御工学における「リアルタイム」「リアルタイム性」の厳密な意味は，
「特定の処理が決められた時間内に完了することが保証されている」ことを指すが，
ここでの「リアルタイムIK」は，「逆運動学計算を高速に周期実行し，入力指令に対し即時追従し続ける」という広義な意味合いであることに注意されたい．

本演習ではROS 2でGazeboシミュレーションを起動し，EusLispから制御する際にはros1_bridgeを用いてROS 1とROS 2を橋渡しする．

## 起動手順

以下の4つのターミナルを用いて，それぞれのプログラムを起動する．

### ターミナル1: ROS 1側でroscoreの起動

```{code-block} console
# ターミナル1: roscoreの起動
$ source /opt/ros/one/setup.bash
$ roscore
```

### ターミナル2: ROS 2側でGazeboシミュレーションの起動

```{code-block} console
# ターミナル2: Gazeboシミュレーションの起動
$ source /opt/ros/jazzy/setup.bash
$ source ~/ros2_ws/install/setup.bash
$ export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
$ export ROS_DOMAIN_ID=11
$ ros2 launch cart_humanoid_ros2 cart_humanoid_gazebo.launch.py
```

**注意**: `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`を設定することで，ROS 2のDDS検出範囲をローカルホストに制限し，ネットワーク上の他のROS 2ノードとの干渉を防ぐ．

### ターミナル3: ROS 1とROS 2のブリッジ

```{code-block} console
# ターミナル3: ros1_bridgeの起動
$ source /opt/ros/one/setup.bash
$ source /opt/ros/jazzy/setup.bash
$ source ~/ros2/bridge/install/setup.bash
$ export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
$ export ROS_DOMAIN_ID=11
$ ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
```

ros1_bridgeが正常に起動すると，ROS 1のtopicとROS 2のtopicが自動的に変換されるようになる．

### ターミナル4: EusLispプログラムの起動

```{code-block} console
# ターミナル4: EusLispプログラムの起動
$ source ~/ros_ws/devel/setup.bash
$ rosparam set /use_sim_time true
$ roscd cart_humanoid/euslisp
$ roseus realtime-ik-sample.l
```

これらを起動すると，irtviewerが立ち上がると同時にGazebo内のロボットが準備姿勢になる．
この状態で，以下のようにコマンドラインからtopicをpublishしてやると，
irtviewer内のロボットモデルとGazebo内のロボットが即時追従する．

```{code-block} console
$ source /opt/ros/one/setup.bash
$ rostopic pub /ik_rarm_tgt geometry_msgs/PoseStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
pose:
  position:
    x: 0.2
    y: -0.5
    z: 0.5
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0"
```

コマンド入力時は適宜Tab補完を利用するとよい

```{figure} ../fig/ik_pub.jpg
---
name: fig:ik_pub
---
リアルタイムIKにより，`rostopic pub`した瞬間，目標値に追従しようとする
```

[realtime-ik-sample.l](https://github.com/jsk-enshu/robot-programming/blob/master/cart_humanoid/euslisp/realtime-ik-sample.l)の内部では，

```lisp
(ros::subscribe "ik_larm_tgt" geometry_msgs::PoseStamped #'send self :larm-cb)
(ros::subscribe "ik_rarm_tgt" geometry_msgs::PoseStamped #'send self :rarm-cb)
(ros::subscribe "ik_lleg_tgt" geometry_msgs::PoseStamped #'send self :lleg-cb)
(ros::subscribe "ik_rleg_tgt" geometry_msgs::PoseStamped #'send self :rleg-cb)
(ros::subscribe "ik_head_tgt" geometry_msgs::PoseStamped #'send self :head-cb)
```

のように`larm`(左手)〜`rleg`(右足),
`head`(この実装ではカメラ注視点)に対する指令値を
`geometry_msgs::PoseStamped`型のTopicとしてsubscribeしている．

そして，指令値をlarm-tgt〜head-tgtに格納した後，
以下のようにして周期的に逆運動学を解くと同時に実ロボットに姿勢を送信している．

```lisp
(ros::rate 10) ;; 10Hz
(while (ros::ok)
  (send *cart_humanoid* :inverse-kinematics (list larm-tgt rarm-tgt lleg-tgt rleg-tgt) ;; 四肢の目標座標
  :move-target (list                 ;; 可動部位を指定(今回は全四肢)
    (send *cart_humanoid* :larm :end-coords)
    (send *cart_humanoid* :rarm :end-coords)
    (send *cart_humanoid* :lleg :end-coords)
    (send *cart_humanoid* :rleg :end-coords))
  :translation-axis (list t t t t)   ;; 目標並進位置のうち，どの要素を追従するか(tでXYZ全て有効)
  :rotation-axis (list t t t t)      ;; 目標回転姿勢のうち，どの要素を追従するか(tでrpy全て有効)
  :stop 2                            ;; 一回の呼び出しで最大何回収束計算するか
  :revert-if-fail nil                ;; IK解が完全に収束しなくても直前の姿勢を採用するか
  :debug-view nil                    ;; デバッグ表示
  )
  (send *cart_humanoid* :look-at-target head-tgt) ;; 注視点制御のみ別関数
  (send *ri* :angle-vector (send *cart_humanoid* :angle-vector) 10) ;; 実ロボットに反映
  (send *irtviewer* :draw-objects :flush nil) ;; 以下，描画のための処理
  (dolist (tgt (list larm-tgt rarm-tgt lleg-tgt rleg-tgt head-tgt)) (send tgt :draw-on :flush nil))
  (send *irtviewer* :viewer :viewsurface :redraw)
  (ros::spin-once)
  (ros::sleep)
  )
)
```

このように，煩雑な逆運動学計算をROSノードとしてモジュール化(i.e.
カプセル化，ブラックボックス化)することで，
これ以降は様々なGUIや入力デバイスから`/ik_*_tgt`に`geometry_msgs::PoseStamped`型でデータを入力するだけで，
ヒューマノイドロボットの全身姿勢を容易に操作できるインターフェースとなる．
