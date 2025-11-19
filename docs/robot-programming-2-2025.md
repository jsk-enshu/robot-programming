# ロボットの全身行動プログラミング

```{eval-rst}
.. raw:: html

   <style>.main-content { counter-reset: exercise 0; }</style>
```

## 本日の演習内容

本日の演習では順運動学や逆運動学を通してロボットアームの目標姿勢を制御する技術に触れる．
順運動学や逆運動学については講義や他の演習で学んできたと思う．
本演習ではそれらの理論に基づいて実際のロボットを動かすためのシステムの使い方を学んでいく．

また，本演習でも2日目の演習と同様に`ROS`を用いたシステムでロボットの行動をプログラミングしていく．
2日目は`ROS`のトピックやサービス等を直接的に呼び出していたが本演習ではロボットインターフェース`*ri*`（`Robot Interface`の頭文字を取って`*ri*`）を用いて間接的に`ROS`のコマンドを送る．
口頭だと`コメアールアイ`だとか`アールアイ`と言う．
ロボットインターフェース`*ri*`ではロボットに搭載される複数の関節やセンサとの`ROS`通信を取りまとめて行っており`*ri*`を用いることで異なる種類のロボットの行動制御プログラムを共通することができるようになる．
異なる種類のロボットは異なるハードウェアを持つためハードウェアに依存する部分のプログラムはロボット毎に異なる．
それらのロボット毎に異なるハードウェアに依存する部分をロボットインターフェースよりも下位にまとめることでロボット毎の違いを吸収している．
これにより同一の上位プログラムから異なるロボットを行動を制御できるようになっている．

さらに付録には順・逆運動学を応用することで
`Gazebo`シミュレーション上のヒューマノイドロボット`JAXON`に対して
人間とヒューマノイドロボットをリアルタイムに同期させるアバターのサンプルも用意してある．
それらのサンプルでは演習用PCの搭載カメラを利用した簡易的なモーションキャプチャの構成を経て`ROS`や2D画像処理技術，アバター技術の基礎に触れることができるので，余裕のある人は必須課題が終わった後や演習後に試してみて欲しい．

本日の目標は以下である．

<div class="screen">

- ロボットモデル(`*jedy*`)とロボットインターフェース(`*ri*`)の違いを理解する

  - ロボットインターフェースによってロボットとデータをやり取りする通信の窓口をまとめておくと便利であることを理解する

  - ロボットモデルによってロボットのリンク長や関節角度に基づいた幾何学的な計算や可視化ができることを理解する

- ロボットインターフェースのメソッドを使えるようになる

  `:state`
  センサ情報を取得する

  `:send-cmd-vel-raw`
  目標の移動速度を送る

  `:go-pos`
  目標の到達位置指令を送る

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

## 環境構築とソフトウェア更新

演習を開始する前に，必ず[環境構築](environment-setup.md)のページを参照して，最新バージョンのソフトウェアを取得すること．

## ROSコマンドなどのおさらい

### ROS 1コマンド

```{code-block} console
$ rostopic list
# topic一覧がでる
$ rostopic hz /atom_s3_button_state
# 周期がわかる
$ rostopic type /atom_s3_button_state
# topicの型が何かわかる
$ rostopic echo /atom_s3_button_state
# topicの内容を表示する
$ rosnode list
# rosnode一覧
$ rosmsg show sensor_msgs/Image
# topicのmsgの型の中身が分かる
$ rqt_graph
# ノード一覧がグラフとして表示される
```

### ROS 2コマンド

```{code-block} console
$ ros2 topic list
# topic一覧がでる
$ ros2 topic hz /atom_s3_button_state
# 周期がわかる
$ ros2 topic info /atom_s3_button_state
# topicの型とpublisher/subscriber数が分かる
$ ros2 topic echo /atom_s3_button_state
# topicの内容を表示する
$ ros2 node list
# rosnode一覧
$ ros2 interface show sensor_msgs/msg/Image
# topicのmsgの型の中身が分かる
$ rqt_graph
# ノード一覧がグラフとして表示される
```

詳細なコマンドの使い方については[ROS 1コマンド](tips/ros1-commands.md)と[ROS 2コマンド](tips/ros2-commands.md)のTipsページを参照すること．

# `EusLisp`ロボットインターフェース`*ri*`からのセンサ値取得・台車駆動（シミュレーションにも対応）

`EusLisp`のロボットインターフェースで実際のロボットのセンサ値取得や台車駆動が行えることを確認する．

`EusLisp`の起動は通常のターミナルで直接実行するよりも， `emacs`で`M-x shell`とタイプし起動したターミナルで行うことをお勧めする．
一度打ち込んだコマンドは，`M-p`で履歴を遡ることができるので，同じコマンド打つ手間が減る．

ロボットPCに`ssh`でログインして，

```{code-block} console
$ source ~/ros_ws/devel/setup.bash
$ ros2 launch jedy_bringup jedy_bringup.launch.py

# シミュレーションでは以下
$ source /opt/ros/jazzy/setup.bash
$ source ~/ros2_ws/install/setup.bash
$ ros2 launch jedy_bringup jedy_gazebo.launch.py
```

を立ち上げ，ロボットPCや遠隔PCの別ターミナルで以下を実行する．

<div class="screen">

```{code-block} console
$ rossetip
$ rossetmaster <ロボットPCのIPアドレス>
$ roscd jedyeus/euslisp
$ roseus jedy-interface.l
```

```{code-block} lisp
irteusgl$ (jedy-init) ;; *jedy*と*ri*を作成する

;; （実機のみ）Atom s3のボタンの値を読んで表示する
irteusgl$ (ros::rate 10) ;; ループの周期を10Hzに設定する
irteusgl$ (do-until-key ;; Enterキーが押されるまでループする
            (send *ri* :spin-once) ;; subscribeしているトピックの更新が行われる．
            (ros::ros-info (format nil "atom s3 button state ~A" (send *ri* :state :atom-s3-button)))
            ;; 実行中にAtom S3のボタンをクリックすと値がクリック数に応じて変わる．
            (ros::sleep)) ;; ros::rateで設定した周期になるようにsleepする
;; 【ポイント】
;; - `do-until-key`によるループの生成
;; - ros::rateとros:sleepによる周期の制御

;; 台車を移動させる
irteusgl$ (send *ri* :send-cmd-vel-raw 10 0 0) ;; 大きすぎる数字を入れないように注意!
irteusgl$ (send *ri* :send-cmd-vel-raw 0. 0 -10)
          ;; 引数は，[前後方向速度] [左右方向速度] [旋回速度]
          ;; 前方に0.0[mm/s]，旋回10.0[deg/s]の速度で少し走って停止する
          ;; Jedyはロボットの正面がx+, なので初めて動かすときは注意．
          ;; y方向にはまだ未対応なので，引数を与えても無視される．

;; センサの値を取得する
irteusgl$ (ros::rate 1)
irteusgl$ (`do-until-key`
            (send *ri* `:spin-once`)  ;; subscribeしているトピックの更新が行われる．
            (ros::ros-info (format nil "roll pitch yaw ~A"
            (send *ri* :state :roll-pitch-yaw)))  ;; IMUから得られたロボットの姿勢が変わる．ロボットを動かしてみよう．
            (ros::sleep))
irteusgl$ (send *ri* :publish-atom-s3-string "hello enshu") ;; 文字が表示される
```

</div>

`(jedy-init)`は双腕移動台車ロボットの初期化関数である．
`(jedy-init)`を実行すると`*jedy*`という大域変数にロボットモデルのインスタンスがセットされ，
`EusLisp`の3Dビューア(``irtviewer``)に表示される [^1].
同時に`*ri*`という大域変数に実ロボットインターフェースのインスタンスがセットされる．

単体のサーボモータやセンサ等の簡単な操作対象の場合は簡単な`Topic`の`pub`/`sub`のみで操作してもシステムは複雑になりにくい．
一方，ロボットは多くの関節・アクチュエータを持つ上に，
カメラやボタンなどに限らず手先・足先の６軸力センサやロボット全身の傾きを計測するセンサなど多くのセンサを搭載しており
それらのセンサや関節をリアルタイムに同期しながら制御する必要があるため
`Topic`の`pub`/`sub`のみでは全体システムが複雑になってしまう．
そこでそれらのセンサや関節との通信機能を一つにまとめた**ロボット操作用インターフェース`*ri*`**を用いることで
`(send *ri* :＜メソッド名＞)`のようにシンプルな統一的な表現でロボット全体を操作できるようになる．

上記の`:atom-s3-button`は`Atom S3`のボタンのクリック値を`EusLisp`で取得するものでありクリックしながら実行することで値が変わることを確認しよう．
また，`:send-cmd-vel-raw`は台車を駆動する速度指令を与えるメソッドである．
さらに`:publish-atom-s3-string`では背面の`Atom S3`に文字を表示する指令を`publish`している．

ロボットインターフェース`*ri*`の内部では2日目の演習で扱ったプログラムと同じように`topic`の`subscribe`/`publish`を行っている．
別ターミナルで

```{code-block} console
$ rostopic echo /atom_s3_additional_info
```

をしながら， `(send *ri* :publish-atom-s3-string "hello robot-programming enshu")` を呼んでみよう．
明示的に`topic`の`subscribe`/`publish`を書かずともロボットのセンサ・アクチュエータにアクセスできる点がロボットインターフェースの利点のひとつである．

他にも，

<div class="screen">

```{code-block} lisp
irteusgl$ (send *ri* :state :atom-s3-button)     ;; 背面Atom S3ボタン（実機のみ）
irteusgl$ (send *ri* :state :roll-pitch-yaw)      ;; IMUセンサの値からロボットの姿勢を取得する
irteusgl$ (send *ri* :state :potentio-vector)   ;; 実機のサーボの角度を取得する
irteusgl$ (send *ri* :send-cmd-vel-raw x y theta) ;; 速度指令で動かす．
                                                        ;; x,y:[mm/s], theta:[deg/s]
```

</div>

などのセンサ取得や指令値送信ができる．
実ロボットのインターフェースプログラムは，
``jedy_bringup`/euslisp/jedy-interface.l` を参照してみよう．

`EusLisp`の条件分岐や繰り返しを用いてセンサ情報に基づいてロボットが行動するプログラムを書いてみよう．

<div class="screen">

```{code-block} console
$ roscd jedyeus/euslisp
$ roseus jedy-interface.l
```

```{code-block} lisp
irteusgl$ (jedy-init) ;; *jedy*,*ri*を作成する
;; シミュレーションでは(jedy-init :simulation t)により*jedy*と*ri*を作成する
irteusgl$ (ros::rate 10)
irteusgl$ (while t
            (send *ri* :send-cmd-vel-raw 10 0 0)
            (setq button-state (send *ri* :state :atom-s3-button))
            (ros::ros-info (format nil "button state ~A" button-state))
            (when (not (= button-state 0))
              (send *ri* :send-cmd-vel-raw 0 0 0)
              (return-from nil nil))
            (send *ri* `:spin-once`)
            (ros::sleep))
```

</div>

これは，`Atom S3`のボタンがクリックされるまで前進指令を繰り返し送り続けるプログラムである．
プログラムはファイルに保存しておくと以下のように実行できて便利である．

```{code-block} console
$ roscd jedy_bringup/exercise/
$ roseus checkpoint2-1-go-forward.l
```

で同様の動作が実行される．
また，`IMU`の値を用いるサンプルもある．`IMU`の傾きを読んで傾きを変えると速度指令を送ることをやめるプログラムとなっている．

```{code-block} console
$ roscd jedy_bringup/exercise/
      $ roseus checkpoint2-2-go-forward-imu.l
```

これを参考にして次の課題に取り組もう．

### <span style="color:green">チェックポイント: センサとロボットインターフェースを使った反応行動</span>

```{exercise} センサとロボットインターフェースを使った反応行動
:label: ex_sensor_ri

`EusLisp`の関数やメソッドおよび実ロボットインターフェース`*ri*`などを駆使してロボットの反応行動を行うプログラムを書いてみよう．
使うセンサは何でもよく(ボタン，`IMU`，エンコーダー値，．..etc)，反応行動も何でもよい
(表示される文字の切り替え，前進後退切り替わる，`IMU`の値に応じてぶつかると前進が停止する．..etc)
がセンサ値・反応行動はそれぞれ２つ以上を組み合わせること．

**例：**
ボタンを押されると音をならし前進しもう一回ボタンを押されるとストップする．
```

# 双腕移動台車ロボットのハードウェアとセットアップ

本章では双腕移動台車ロボットの全身動作を行っていく．

## ハードウェア構成

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

## セットアップ

### 接続と電源投入

背面のバックパック部分の白い制御基板(`KondoH7`基板)から出ている黒い3線のケーブルが`Radxa zero`の上面にある充電管理基板の3線とつながっていることを確認しよう(，を参照).
背面の物理ボタンをONにすると`KondoH7`基板のLEDが青く点滅し，ブザー音が鳴ることを確認しよう．

```{figure} fig/jedy_backs.png
---
name: fig:turtlebot_back2
---
Jedyの背面パネル
USB充電端子，サーボ基板へ8Vを出力する黄色いコネクタの`XT30`端子，`UART`通信のための黒い`ZH3`線ケーブル，バッテリー入力をする黄色い`XT30`端子がある．また`Atom S3`にはロボットのIPアドレスやバッテリー残量が表示されている．
```

### IDの確認

`Jedy`の電源を入れ，ロボットPCに`ssh`でログインして，
以下のコマンドを実行し22個のサーボモータすべてが
緑色に"found"となっていることを確認する．
**なお，これはデバイスにアクセスするプログラムなので，`jedy_bringup.launch.py`などを立ち上げていると実行できないことに注意する．**

```{code-block} console
$ rosrun jedy_bringup scan_ids.py
  Skipping reset for non-USB serial port: /dev/ttyAML1 cannot be reset via USB reset.
  Could not find USB device information for port /dev/ttyAML1
  Opened /dev/ttyAML1 at 1000000 baud
  Servo ID 0 (`rarm_joint0`) found
  Servo ID 1 (`larm_joint0`) found
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
  Servo ID 32 (`head_joint0`) found
  Servo ID 34 (`head_joint1`) found
```

実行して“found”とでない場合は，
ケーブルがささっていないことや，ID書き込みエラーやモータの故障が疑われるので，
TAを呼んで `KRS`サーボの交換やIDの再書き込みを行ってもらう．

```{code-block} console
$ rosrun jedy_bringup scan_ids.py
Skipping reset for non-USB serial port: /dev/ttyAML1 cannot be reset via USB reset.
Could not find USB device information for port /dev/ttyAML1
Opened /dev/ttyAML1 at 1000000 baud
Servo ID 0 (rarm_joint0) found
Servo ID 1 (larm_joint0) found
Servo ID 2 (rarm_joint1) found
Servo ID 3 (larm_joint1) found
Servo ID 4 (rarm_joint2) found
Servo ID 6 (rarm_joint3) found
Servo ID 8 (rarm_joint4) found
Servo ID 10 (rarm_joint5) found
Servo ID 12 (rarm_joint6) found
Servo ID 14 (rarm_gripper_joint) found
Servo ID 20 (front_right_wheel_joint) found
Servo ID 21 (front_left_wheel_joint) found
Servo ID 22 (rear_right_wheel_joint) found
Servo ID 23 (rear_left_wheel_joint) found
Servo ID 32 (`head_joint0`) found
Servo ID 34 (`head_joint1`) found


Servo IDs and joint names not found:
Servo ID 5 (larm_joint2) not found
Servo ID 7 (larm_joint3) not found
Servo ID 9 (larm_joint4) not found
Servo ID 11 (larm_joint5) not found
Servo ID 13 (larm_joint6) not found
Servo ID 15 (larm_gripper_joint) not found
```

実行してこのように“not found”と出ているサーボはケーブルが抜けていないかどうかを確認すべきである．
<span style="color:red">**ロボットにおいて電源周りやケーブルの接触不良は物理的に動くものであるため起こりうる．大事なことは問題が起きたときにどのように対処できるかというデバッグ能力であり，今のうちから技術を身に着けていってほしい．**</span>

# 演習への取り組み方と移動台車ロボットの共有方法

ロボットPCをROSのMasterとして班員のPCを複数台使うことでそれぞれ個別に指令値を送ったりセンサ値を表示したりしながらロボットを動かすことができる．
しかしロボットの動作自体は一人が送っている間は他の人が送ると動作が上書きされてしまうため，みんなで話ながらロボットを動かしていってほしい．

また，これ以降は実機とシミュレーション（Gazebo）の両方に対応している．
班員で相談して以下のいずれかの方法で取り組むとよい．

1.  班員全員で演習に取り組みながらチェックポイントごとにロボットの動作指令を送る人を交代する．演習課題は全員で取り組む．（全員がセットアップをする必要がある．）

2.  各々がシミュレーションで試してから交代で実機を利用する．演習課題は個人で取り組む．（1人が実機でトラブルと全員が取り組めなくなる．）

複数台のPC接続設定は2日目の資料に従う．

# アームロボットのソフトウェア（シミュレーションにも対応）

本小節ではアームロボットを動かすためのソフトウェアについて説明する．

## ロボットアームの起動

**本章のコマンドはアームを動かすのでアームが何かにぶつからないように周囲に注意しながら演習を行うこと．**

ロボットPCにsshでログインして

```{code-block} console
$ source ~/ros_ws/devel/setup.bash
$ ros2 launch jedy_bringup jedy_bringup.launch.py

# シミュレーションでは以下．
$ source ~/ros2_ws/install/setup.bash
$ ros2 launch jedy_bringup jedy_gazebo.launch.py
```

を起動すると`/dev/ttyAML1`デバイスファイルにアクセスする`ROS`ノードが起動する．

**実機を動かす場合はのセットアップが必要になる．**

## PythonプログラマのためのEusLisp入門

本演習では，ロボット制御にEusLispという言語を使用する．多くの学生にとってPythonの方が馴染みがあると思われるため，ここではPythonとの対比を通じてEusLispの書き方を理解できるようにする．

:::{figure} fig/euslisp-and-python.png
:align: center
:width: 700px
:name: fig:euslisp-python-comparison

PythonとEusLispの構文対比
:::

### 基本的な対応関係

| 概念 | Python | EusLisp | 説明 |
|------|--------|---------|------|
| **関数呼び出し** | `jedy_init()` | `(jedy-init)` | 関数名を括弧で囲み，括弧の前に配置 |
| **変数代入** | `a = 1` | `(setq a 1)` | `setq`を使用して代入 |
| **メソッド呼び出し** | `ri.servo_off_all()` | `(send *ri* :servo-off-all)` | `send`でオブジェクトにメソッドを送信 |
| **引数付きメソッド** | `ri.go_velocity(0.1, 0, 0)` | `(send *ri* :go-velocity 0.1 0 0)` | 引数をカンマなしで列挙 |
| **リスト/配列** | `[0, 0, 0, 0, 0, 0]` | `#f(0 0 0 0 0 0)` | `#f`は浮動小数点数ベクトル |
| **入れ子の関数** | `jedy.inverse_kinematics(make_coords(pos=[370, 0, 150]))` | `(send *jedy* :inverse-kinematics (make-coords :pos #f(370 0 150)))` | 内側から外側へ評価される |

### 重要なポイント

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

## `EusLisp`のロボットインターフェース`*ri*`からのアームロボットの制御

以下のコマンドにより
`EusLisp`のロボットインターフェースからアームロボットを動かすことができる．

<div class="screen">

```{code-block} console
$ roscd jedyeus/euslisp
$ roseus jedy-interface.l
```

``` lisp
irteusgl$ (jedy-init) ;; ロボットの実機インターフェース*ri*とロボットモデル*jedy*を生成．
irteusgl$ (send *jedy* :reset-pose) ;; 関節角度を:reset-poseにセット
irteusgl$ (send *irtviewer* :draw-objects) ;; 描画を更新
irteusgl$ (send *ri* :angle-vector (send *jedy* :angle-vector) 4000)
          ;; 第一引数は目標関節角度 [deg]，第二引数は補間時間 [ms],4秒で目標到達角度になる．
```

</div>

`(jedy-init)` 関数は`*jedy*`変数と`*ri*`を生成する． `*jedy*`は，
`EusLisp`のビューワ(`irtviewer`)上に描画されたロボットモデルを表す．
`*ri*`は実際のロボット（及びシミュレーション内のロボット）のセンサ値取得・動作指令を行うためのロボットインターフェースである．

ロボットを動かすためには，以下のように，ロボットモデル(`*jedy*`)の関節を所望の角度にしてからこれを実機(`*ri*`)へ送る．

1.  `(send *jedy* :reset-pose)`でロボットモデルをリセット姿勢にして

2.  `(send *jedy* :angle-vector)`でロボットモデルの関節角度を取得して

3.  `(send *ri* :angle-vector (send *jedy* :angle-vector))`でこれを実機へと送る．

1.のロボットモデルの姿勢を決定する部分は，
のように関節角度を直接指定したり[EusLispの逆運動学メソッド](tips/euslisp-ik.md)のように逆運動学を用いてもよい．

## `KRS`サーボのON/OFF

以下のコマンドでサーボのON/OFFが切り替えられる．

<div class="screen">

```{code-block} console
$ roscd jedyeus/euslisp
$ roseus jedy-interface.l
```

```{code-block} lisp
irteusgl$ (jedy-init) ;; アーム＋台車ロボットの*ri*と*jedy*を生成
irteusgl$ (send *ri* :servo-on) ;; 全軸サーボがONになる
irteusgl$ (send *ri* :servo-off) ;; 全軸サーボがOFFになる．
;; 関節名を指定してONにする
irteusgl$ (send *ri* :servo-on :names (list "head_joint0" "head_joint1") ;; 頭のみサーボがONになる．
;; 使用できる関節名は以下のように確認できる．
irteusgl$ send-all (send *jedy* :joint-list) :name
("rarm_joint0"
"rarm_joint1"
"rarm_joint2"
"rarm_joint3"
"rarm_joint4"
"rarm_joint5"
"rarm_joint6"
"rarm_gripper_joint"
"larm_joint0"
"larm_joint1"
"larm_joint2"
"larm_joint3"
"larm_joint4"
"larm_joint5"
"larm_joint6"
"larm_gripper_joint"
"head_joint0"
"head_joint1")
;; 頭部の関節名を指定して角度を変える
(send *jedy* :head_joint0 :joint-angle 20)
(send *jedy* :head_joint1 :joint-angle 50)
;; 関節指令を実機に反映させる (3000msかけておくる)
(send *ri* :angle-vector (send *jedy* :angle-vector) 3000)
```

</div>

特に`KRS`サーボモータは，緑，青，赤サーボの３種類があるが，
赤サーボのみモータに過負荷や過電流がかかったり高温になったとき，サーボがきれ．これは，内部の基板で電流値や温度値を監視し制御器へのエラーフラグを立ててサーボを落とすような処理が組み込まれている．
しかし，今回の演習で使用しているものは緑及び青サーボとなる．
ロボットの関節に負荷がかかるような姿勢に長時間しているとサーボが焼ききれるため，危ないときは背面の黒いスイッチを押してサーボへの供給をとめるか，
`(send *ri* :servo-off)`メソッドでサーボをきってほしい．
サーボが焼ききれた場合にはサーボパーツの交換をしたもらうこととなるため，時間がかかるということを念頭に置いてほしい．

他にも関節ゲインを変更するなど細かい設定や センサ値取得も可能である．
興味のある人は， <https://kondo-robot.com/product/03146>
などの製品情報ページを参照してみてほしい．

## 関節角度の直接指令による動作生成

<div class="screen">

```{code-block} lisp
irteusgl$ (send *jedy* :angle-vector #f(90.0 -4.0 -30.0 -100.0 -3.0 -88.0 -1.0 0.0 -90.0 4.0 30.0 -100.0 3.0 -88.0 1.0 -1.0 0.0 0.0))
;; 根本から先端までの関節角度 [deg] (グリッパは含まない)
;; #fはnumpyでいうところのnp.arrayのようなものだが，中で計算することはできない．
;; そのため中で計算をさせたいときはfloat-vectorを使う
irteusgl$ (send *jedy* :angle-vector
 (float-vector (- 90.0 10) -4.0 -30.0 -100.0 -3.0 -88.0 -1.0 0.0 -90.0 4.0 30.0 -100.0 3.0 -88.0 1.0 -1.0 0.0 0.0))
```

</div>

関節軸数が多くなるとこのような指定の方法は難しくなってくるので，軸を指定して角度を設定する方法もある．
関節角度の直接指定など．

<div class="screen">

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

</div>

繰り返し処理を追加すると以下のようにエンターキーを押すまでロボットがうなずき続ける．

<div class="screen">

``` lisp
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

</div>

## 逆運動学による動作生成

手先を目標物に伸ばすというように，手先作業空間での目標に基づいてロボットの運動を生成するには，逆運動学(IK)を用いる．
逆運動学は端的には，「目標位置まで手先を動かすための関節角度はいくらか？」を求める演算である．

<div class="screen">

```{code-block} console
$ source ~/ros_ws/devel/setup.bash
$ roseus
```

```{code-block} lisp
irteusgl$ (load "package://jedyeus/euslisp/keyboard-ik-sample.l")
```

</div>

を実行してみてほしい．
ビューワのウィンドウがアクティブなときにキーボード入力をするとアームロボットの腕が動いていると思う．
これは，逆運動学を解くことにより
手先の指令値に応じて各関節角度を計算し
ビューワで描画しながら実機へ指令しているサンプルである．

このサンプルの中では
キーボード入力に応じて以下の逆運動学メソッドを呼び出し
ロボットモデルの関節角度を計算している．

また実機もしくは`gazebo`上のロボットに姿勢を送りたい場合には
`keyboard-ik-sample.l`のプログラム中の `(jedy-init :without-ri t)`
となっている部分を`(jedy-init :without-ri nil)`とすると実機もしくは`gazebo`上のロボットに姿勢が反映される．

:::{figure} fig/keyboard_ik_sample.jpg
:align: center
:name: fig:keyboard_ik_sample

keyboard-ikによるロボットの姿勢の反映
:::

<div class="screen">

``` lisp
irteusgl$ (send *jedy* :reset-pose) ;; 初期姿勢によってはIKが解けないことがある
irteusgl$ (send *jedy* :rarm :inverse-kinematics
            (make-coords :pos (float-vector 120 -90 -30) :rpy (list (deg2rad 30) (deg2rad 0) (deg2rad 0)))
            :debug-view :no-message)
```

</div>

逆運動学メソッドの詳細な使い方は，を参照のこと．

### <span style="color:green">チェックポイント: アームロボットの制御</span>

```{exercise} アームロボットの制御
:label: ex_arm_control

1.  にしたがってサーボモータのIDを確認せよ．

2.  にしたがって頭部以外のモータを動かしてみよ．また，`irtviewer`のロボットモデルと実際のロボットの姿勢が一致していることを確認せよ
    [^2].

3.  にしたがって逆運動学でロボットの姿勢を生成し実機に送ってみよ．
```

なお，実際に`Jedy`の定義されてるファイルは，[jedy/jedyeus/euslisp/jedy.l](https://github.com/jsk-enshu/robot-programming/tree/master/jedy/jedyeus/euslisp/jedy.l)にある．

# 本日の発展課題

## 課題1(発展)

2日目の演習でカメラ画像を使って認識を行った．物体の姿勢を推定し，その推定結果からロボット実機の姿勢を決定することは重要である．
ここでは，カメラのRGB画像からマーカーボックスの位置を推定し，ロボットの逆運動学計算を行いピックアップする方法を紹介するので，希望者はTAか先生から`AprilTag`Boxを貸りて，ぜひ挑戦してみてほしい．
また，サンプルで用意しているプログラムはマーカーが地面から垂直にある場合に掴みにいくものとなっているため，横からつかみにいったほうが逆運動学が解け，掴みやすいことや，
双腕を使って掴みやすい位置にアシストしなが持っていくなど発展させることが可能であるので，こちらもぜひ挑戦してみてほしい．

今回使用するマーカーは[AprilTag](https://april.eecs.umich.edu/software/apriltag)と呼ばれるものである．比較的小さいマーカーでも認識しやすい．

2日目の資料を参考にしながら，以下のプログラムを起動する．カメラの起動はデバイスにアクセスするプログラムとなるのでロボットPCで行う．

```{code-block} console
$ ssh jedy@<ロボットPCのIPアドレス>
      $ ros2 launch jedy_bringup jedy_bringup.launch.py  # すでに立ち上げている場合は立ち上げない．
      # 次に別のターミナルで
      $ ssh jedy@<ロボットPCのIPアドレス>
      $ `roslaunch` `jedy_bringup` d405.launch
```

次にロボットPCで`AprilTag`を認識するプログラムを立ち上げる．このプログラムはロボットPC内部で立ち上がるため，通信帯域が狭くても認識したタグの位置情報だけをネットワーク越しに送るため，
演習室環境でも十分に実行できると思われる．

```{code-block} console
$ ssh jedy@<ロボットPCのIPアドレス>
      $ `roslaunch` `jedy_bringup` apriltag_detection.launch
```

プログラムが無事に起動するとマーカー認識が立ち上がるので，Rvizもしくはrqt_image_viewなどで自分のPCからロボットPCへ`rossetmaster`して
/camera/color/tag_detections_image/compressedというトピックを可視化してみると良い．
この状態でApriltagBoxを移してみると認識した位置にマーカーのIDが表示される．

:::{figure} fig/recognition_apriltag.jpg
:align: center
:name: fig:recognition_apriltag

`AprilTag`の認識結果
:::

また`/camera/color/tag_detections`というトピック名で認識位置の姿勢情報が出ているのでこれを`rostopic echo --noarr`などで確認してみると良い．
この状態で下記を実行すると`AprilTag`を認識した位置に対して逆運動学が解けた場合は腕を伸ばす様子が`irtviewer`に表示される．

```{code-block} console
$ rossetip
$ rossetmaster <ロボットPC>
$ roscd jedyeus/euslisp
$ roseus sample-pickup.l
```

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


## 課題2.1 (発展)

Jedyの`follow joint trajectory`コントローラーを別々に動かすプログラムを書いて`head_controller`のみを動かすプログラム，`rarm_controller`のみを動かすプログラムで分けて実行することでコントローラーごとに独立して操作できることを確認してみよう．

## 課題2.2 (発展)

他の班のJedyと協力して片方を動かすと片方のJedyが動くプログラムを作成してみよ．


## ヒューマノイドロボットJAXONのシミュレーション（発展課題）

本章では，Gazeboシミュレーション上でヒューマノイドロボットJAXONを動かす方法を説明する．
本演習ではROS 2でGazeboシミュレーションを起動し，EusLispから制御する際にはros1_bridgeを用いてROS 1とROS 2を橋渡しする．
JAXONのGazeboシミュレーションはROS 2パッケージ`cart_humanoid_ros2`で提供されている．
[JaxonのシミュレーションでリアルタイムIKを動かす](tips/realtime-ik.md)を参照のこと．

## 課題3(発展)

`EusLisp`の`robot`インターフェース`*ri*`は双腕移動台車ロボットだけでなくヒューマノイド`JAXON`にも用意されており
移動台車アームロボットとヒューマノイド`JAXON`は`*ri*`に共通のメソッドを送信することで制御することができる．
また，手先の目標位置を`ik_＜ターゲット名＞_tgt`という`topic`を`ROS`ノード間で通信しそれを目標値としたIKを解いてロボットの姿勢を決定しているためロボット固有の属性（関節の数や関節軸の向き等）にほとんど依存しないシステムの上で動作している．
従って本日の演習の`launch`ファイルと`euslisp`のファイルを少し書き換えるだけで移動台車アームロボットと`JAXON`を同じようなプログラムでIKによる制御を行うことができるということである．
以下の手順でと同様に手先位置で実機またはシミュレーション上のアームロボットの手先をリアルタイムに動かしてみよ．

## 課題4(発展)

以下の手順でと同様にカメラで認識した手先位置で実機またはシミュレーション上のアームロボットの手先をリアルタイムに動かしてみよ．

## 課題5(発展)

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

## 課題6(発展)

EusLispのrobotインターフェースに実装されている`inverse-kinematics`機能([参考](tips/euslisp-ik.md))を用いて
任意の物体にアームで触れるプログラムを作成せよ．（※可能であればグリッパを駆使して物体をピックアップできるとなお良い）

## 課題7(発展)

二日目の演習でJedyからJaxonを動かす様子をみたと思う．実機のJedyを動かしシミュレーターのJaxonを動かせるようにしてみよ．

## 課題8(発展)

これまでの授業や演習で学んだロボットプログラミング技法を組み合わせ，
自由な発想でロボットのデモプログラムを作成せよ．


[^1]: `irtviewer`は，`EusLisp`で実装された3次元ビジュアライゼーションツールである．ロボットモデルや幾何形状をインタラクティブに表示できる．

[^2]: `irtviewer`のロボットモデルと実際のロボットの姿勢が一致することでシミュレーションと実機の動作が正しく連携していることを確認できる．

