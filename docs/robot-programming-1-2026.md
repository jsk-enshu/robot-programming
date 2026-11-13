(day-recognition-manipulation)=
# 双腕移動台車ロボットを用いた認識操作プログラミング

## 本日の演習内容

1・2 日目は双腕移動台車ロボット **Jedy** を Python で動かす．

- **1 日目 (今日)**: Jedy の「道具」をひととおり使えるようになる．台車で走る，腕と首を動かす (逆運動学)，カメラで見る・認識する，声でしゃべる・聞く．
- **2 日目**: 今日の道具を組み合わせて，**自分がロボットにさせたい動き**を作る ({doc}`robot-programming-2-2026`)．

2025 年度までは EusLisp と ROS 1 を使っていたが，2026 年度は **Python のロボットライブラリ [scikit-robot](https://github.com/iory/scikit-robot) と ROS 2 (Jazzy)** だけを使う．
考え方は同じで，次の 2 つを使い分ける．

- **ロボットモデル `robot`**: リンクの長さや関節角度をもとに幾何計算 (順運動学・逆運動学) をする．モデルの関節を動かしても実機は動かない．
- **ロボットインターフェース `ri`**: 実機 (またはシミュレータ) との窓口．姿勢を送る・センサを読む・台車を動かす・認識を聞く・しゃべる．

`robot` はすぐに変わり，机があっても突き抜ける．`ri` に送った動きは時間がかかり，重力や接触で指令どおりにならないこともある．
だから「`robot` で計画し，`ri` で動かし，`ri` で確かめる」を繰り返す．

:::{figure} fig/jedy-robot-vs-ri.png
:align: center
:name: fig:jedy-robot-vs-ri

半透明の青が `robot` (逆運動学で右手を赤い点へ上げた姿勢)，不透明なのが実機・シミュレータ (`ri`)．
`robot` を変えただけでは，まだ `ri.angle_vector(robot.angle_vector(), 時間)` を送っていないので動かない
:::

```{tip}
**ブラウザだけで試せる**: [scikit-robot-web](https://iory.github.io/scikit-robot-web/?lang=ja) では，Python で書いた `robot` と `ri` のプログラムが
ブラウザの中で動き，シミュレータ (MuJoCo) の Jedy を操作できる (何もインストールしなくてよい)．
3D の画面では半透明の青いロボットが `robot`，不透明なロボットが `ri` の動かす Jedy である．
サンプル「robot vs ri」では，机の中に手を入れる計画を立てると，`robot` は机を突き抜け，`ri` の Jedy は机の上で止まる．
```

**同じ Python プログラムが Gazebo (シミュレータ) と実機のどちらでもそのまま動く．** どれが動くかは起動した launch だけで決まり，プログラムは何も変えない．
まずシミュレータで作り，できたら実機で動かすとよい．

本日の目標は以下である．

<div class="screen">

- ロボットモデル (`robot`) とロボットインターフェース (`ri`) の違いを理解する
- 台車を動かす: `ri.go_velocity(vx, vy, vtheta)`
- 腕と首を動かす: `ri.servo_on()`, `ri.angle_vector(av, time)`, `robot.rarm.inverse_kinematics(coords)`
- カメラで見る: `ri.image()`, `ri.depth_image()`
- カメラで認識する: `ri.sees('ペットボトル')`, `ri.what_do_you_see()`, `ri.janken()`
- 声でしゃべる・聞く: `ri.say('こんにちは')`, `ri.listen()`

</div>

```{important}
各チェックポイントを達成しながら進捗報告シートへの記入を進めること．各チェックポイントの結果をスクリーンショット等で保存しておくこと．
```

角度は rad，時間は秒，速度は m/s・rad/s (SI 単位) で書く．度で考えたいときは `np.deg2rad` / `np.rad2deg` を使う．
EusLisp を知っている人は {doc}`tips/euslisp-to-skrobot` に対応表がある．

## 準備

ワークスペースは {doc}`environment-setup` で作っておく (初回のビルドはダウンロードを含めて数十分かかるので，演習の前に済ませておくこと)．
どのターミナルでも最初に次の 2 行が要る．{doc}`environment-setup` のとおりに `~/.bashrc` に書いてあれば，新しく開いたターミナルでは自動で読み込まれる．

```{code-block} console
$ source ~/ros2_ws/install/setup.bash
$ export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
```

まだ書いていなければ，次で `~/.bashrc` に書いておく (1 回だけ)．

```{code-block} console
$ echo 'source ~/ros2_ws/install/setup.bash' >> ~/.bashrc
$ echo 'export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST' >> ~/.bashrc
$ source ~/.bashrc
```

2 行目は ROS 2 の通信を自分の PC の中だけに閉じる設定である (ロボットは自分の PC に USB でつなぐので，これで他の人のロボットと混ざらない)．
意味や，複数の PC で通信するときの設定は {doc}`tips/ros-network` を見よ．

## ロボットの起動

どちらか **1 つだけ** を起動する (同時に起動すると同じ名前のトピックがぶつかる)．

### Gazebo シミュレーション

```{code-block} console
$ ros2 launch jedy_bringup jedy_gazebo.launch.py box:=true
```

`box:=true` でカメラの演習に使う赤い箱をロボットの左前に置く．
`gui:=false` で画面なし，`render_sensors:=false` で GPU の無い PC 向け (カメラなし)．
頭のカメラは PC の負担を減らすため 1 秒に 5 枚 (`camera_rate:=5`，既定) にしてある．速い動きを追いたいときは `camera_rate:=30` (実機の D405 は 30 Hz)．
ロボットは演習室 (13BC) の机の手前に置かれる (`world:=empty` で何も無い床)．

:::{figure} fig/jedy-gazebo-room13bc.png
:align: center
:name: fig:jedy-gazebo-room13bc

Gazebo の演習室の Jedy と赤い箱 (`box:=true`)
:::

#### Gazebo に物を置く

起動するときの引数で，演習で使う物を置ける (複数指定してよい)．

| 引数 | 置かれる物 |
| --- | --- |
| `box:=true` | 赤い箱 (色で見つける演習用) |
| `demo:=true` | AprilTag の付いた台と立方体，椅子，コップ |
| `person:=true` | 人 (人を見つける・追うデモ用) |
| `checkerboard:=true` | チェッカーボード |

起動したあとでも，別のターミナルから物を足せる．位置は起動したときのロボットの位置が原点で，`-x` が前，`-y` が左，`-z` が上 (m)．

```{code-block} console
$ # 緑の箱 (一辺 0.2 m) をロボットの 1 m 前，0.4 m 右に置く
$ ros2 run ros_gz_sim create -name green_box -x 1.0 -y -0.4 -z 0.1 -string \
  "<sdf version='1.9'><model name='m'><static>true</static><link name='l'><collision name='c'><geometry><box><size>0.2 0.2 0.2</size></box></geometry></collision><visual name='v'><geometry><box><size>0.2 0.2 0.2</size></box></geometry><material><diffuse>0 0.8 0 1</diffuse></material></visual></link></model></sdf>"
$ # Gazebo Fuel (https://app.gazebosim.org/fuel/models) のモデルを URL で置く (例: コップ)
$ ros2 run ros_gz_sim create -name my_cup -x 0.8 -y 0.0 -z 0.0 \
  -file https://fuel.gazebosim.org/1.0/GoogleResearch/models/ACE_Coffee_Mug_Kristen_16_oz_cup/1
$ gz model --list                    # 置かれている物の名前
$ gz service -s /world/jedy_world/remove --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean \
  --timeout 3000 --req 'name: "green_box" type: MODEL'   # 名前を指定して消す
```

`-name` は物ごとに違う名前にする．
物体の認識 (YOLO) は本物らしい見た目の物でないと見つけにくい．単色の箱は色で見つけ (`track_box`)，
「コップ」「ペットボトル」などを認識させたいときは Fuel の実物をスキャンしたモデル (GoogleResearch など) を置くとよい．

### 実機 (RCB-4mini)

初回だけ，USB シリアルアダプタ (Kondo DUAL USB ADAPTER HS) に `/dev/actuo_kondo_rcb4mini` という名前を付ける udev ルールを入れる．
入れないと起動が「`/dev/actuo_kondo_rcb4mini` を待っています」のまま進まない．

```{code-block} console
$ sudo $(ros2 pkg prefix actuo_ros)/share/actuo_ros/udev/install.sh
```

```{code-block} console
$ ros2 launch jedy_bringup jedy_fullbody_all.launch.py camera:=true
```

USB シリアルアダプタが挿さるまで待ってから起動する．サーボは **OFF で起動する**．
`camera:=true` で頭のカメラ (USB でつなぐ RealSense D405) も起動する
(`sudo apt install ros-jazzy-realsense2-camera` が 1 回だけ必要)．

### 確認

どれを起動しても次のトピックが出ている．

```{code-block} console
$ ros2 topic list | grep jedy_fullbody
/jedy_fullbody/joint_states
...
$ ros2 topic hz /jedy_fullbody/joint_states        # 関節角度が来る周期
$ ros2 topic echo /camera/color/camera_info --once # カメラの内部パラメータ
$ rqt_graph                                        # ノードとトピックのつながり
```

## ロボットモデルとロボットインターフェース

Python を対話的に使う．`ipython` を使うと履歴 (上キー) や補完 (Tab) が効いて便利である．

```{code-block} console
$ $(ros2 pkg prefix jedy_skrobot)/share/jedy_skrobot/venv/bin/ipython
```

```{code-block} python
from jedy_skrobot import connect
import numpy as np
import time

robot, ri = connect()   # robot: ロボットモデル, ri: ロボットインターフェース

# 首のピッチ軸の角度を読む (10 Hz で 3 秒)
for _ in range(30):
    av = ri.angle_vector()                 # 今の全関節角度 [rad]
    robot.angle_vector(av)                 # モデルに反映
    print(np.rad2deg(robot.head_joint1.joint_angle()))
    time.sleep(0.1)
```

ファイルに書いたプログラムは，jedy_skrobot の仮想環境の python で実行する (ふつうの `python3` では scikit-robot が見つからない)．

```{code-block} console
$ $(ros2 pkg prefix jedy_skrobot)/share/jedy_skrobot/venv/bin/python ~/enshu/my_program.py
```

ロボットインターフェースの内部では topic の publish / subscribe や service・action の呼び出しをしている．
`ri` のメソッドがどの topic を使うかは `jedy_skrobot/interface.py` の先頭に表がある．

### モデルを画面で見る (PyrenderViewer)

`robot` (モデル) の姿勢は scikit-robot のビューワで見られる．`ri` に送る前に，計画した姿勢を確かめるのに使う．
メッシュ (見た目の形) を読むために `connect(load_meshes=True)` とする (無いとビューワに何も表示されない)．

```{code-block} python
from jedy_skrobot import connect
from skrobot.viewers import PyrenderViewer
import numpy as np

robot, ri = connect(load_meshes=True)
viewer = PyrenderViewer()   # ウィンドウが開く (マウスで回転・拡大できる)
viewer.add(robot)
viewer.show()

robot.angle_vector(ri.angle_vector())   # モデルを今のロボットの姿勢に
viewer.redraw()                         # モデルを変えたら redraw() で描き直す
```

`viewer.redraw()` を呼ばないと画面は変わらない．`viewer.save_image('pose.png')` で今の画面を保存できる．

## 台車を動かす

```{code-block} python
ri.tuck_arms()                                # 走る前に腕をたたむ (下げたままだと床をこする)
ri.go_velocity(0.1, 0.0, 0.0, duration=2.0)   # 前後 [m/s], 左右 [m/s], 旋回 [rad/s]: 2 秒進んで止まる
ri.go_velocity(0.0, 0.1, 0.0, duration=2.0)   # 2 秒左へ (メカナムホイールなので横にも動ける)
ri.go_velocity(0.0, 0.0, 1.0, duration=2.0)   # 2 秒その場で旋回
```

`duration` を省くと 1 回だけ指令を送る．台車は 0.5 秒指令が来ないと自動で止まるので，ループの中で毎回呼ぶ．Jedy の正面は x+ である．

:::{figure} fig/jedy-gazebo-base-square.gif
:align: center
:name: fig:jedy-gazebo-base-square

Gazebo で `ri.tuck_arms()` のあと，前・左・後ろ・右に 0.12 m/s で 2.5 秒ずつ走り，その場で旋回する (実時間)
:::

```{warning}
実機の台車には動ける速さの範囲がある．前後・左右は 0.078〜0.156 m/s，その場旋回は 0.83〜1.66 rad/s 程度．
これより遅い指令は**止まる** (車輪のサーボが回らない範囲なので，ゆっくり進むのではなく止める)．
速すぎる指令は上限で止められ，ログにそう出る．
```

```{note}
**時間はロボットの時計で測る．** シミュレータは実時間より遅く進むことがあるので，`time.time()` で「3 秒進む」と書くと
シミュレーションの中では 3 秒より短くしか進まない．`ri.now()` (秒) と `ri.sleep(秒)` はシミュレータでは
シミュレーション時刻，実機では普通の時計を使うので，どちらでも同じ距離だけ進む．
`go_velocity(..., duration=...)` も同じ時計で測っている．
```

キーボードで操縦することもできる (台車は `/cmd_vel` (`geometry_msgs/TwistStamped`) で動く)．

```{code-block} console
$ ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true
```

`stamped:=true` を忘れると時刻なしの `Twist` が送られ，台車は受け取らない．
起動した端末にキーを打つ (端末を選んでおく)．キーを押すたびに 1 回指令が送られ，台車は 0.5 秒指令が来ないと止まるので，動かし続けるにはキーを押し続ける．

| キー | 動き |
| --- | --- |
| `i` / `,` | 前進 / 後退 |
| `j` / `l` | その場で左 / 右に旋回 |
| `u` `o` `m` `.` | 曲がりながら前進・後退 |
| `J` / `L` (Shift) | 左 / 右へ横に動く (メカナムホイール) |
| `k` | 止まる |
| `z` / `q` | 速さ (前後・左右と旋回の両方) を 1 割下げる / 上げる |
| `x` / `w` | 前後・左右の速さだけを 1 割下げる / 上げる |
| Ctrl-C | 終了 |

速さの既定は前後・左右 0.5 m/s，旋回 1.0 rad/s で，前後・左右は実機の上限 (0.156 m/s 程度) を超える．
`z` を押すたびに速さが 1 割ずつ下がる (今の速さは画面の `currently: speed ... turn ...` に出る)．
ただし `z` は旋回も遅くし，実機は 0.83 rad/s 程度より遅い旋回では止まるので，`z` を 2 回以上押すと旋回しなくなる．
前後・左右だけを下げるなら `x` を使うか，最初から `-p speed:=0.12` を付けて起動する．
`ros2 launch jedy_bringup keyboard_teleop.launch.py` は同じものを新しい端末の窓で開く．実機では PS4 のジョイスティックでも操縦できる．

### <span style="color:green">チェックポイント: 台車</span>

```{exercise} 台車を動かす
:label: ex_base_2026

1. `ri.go_velocity()` で，1 辺 30 cm 程度の四角形を描くように台車を動かしてみよう (前・左・後ろ・右，または前進と 90 度旋回の繰り返し)．
2. `teleop_twist_keyboard` で，`z` や `x` で速さを下げてから，`i` (前進)・`J` / `L` (横)・`j` / `l` (旋回) で同じ四角形を描いてみよう．
```

## 腕と首を動かす

### サーボの ON/OFF

```{code-block} python
ri.servo_on()                                       # 全軸 ON (今の姿勢のまま固まる)
ri.servo_off()                                      # 全軸 OFF (腕が脱力するので支えること)
ri.servo_on(joints=['head_joint0', 'head_joint1'])  # 頭だけ ON
print([j.name for j in robot.joint_list])           # 使える関節名
```

`servo_on()` はまず今の姿勢を送ってから ON にするので，ON にした瞬間に腕が飛ぶことはない．
サーボを OFF にした関節は脱力し，重力で垂れる．OFF の間も関節角度は読めるので，手で動かした角度も `ri.angle_vector()` で分かる．

:::{figure} fig/jedy-gazebo-servo-off.gif
:align: center
:name: fig:jedy-gazebo-servo-off

Gazebo で両腕を `ri.servo_off(joints=...)` すると前腕が垂れ，`ri.servo_on(joints=...)` で垂れた姿勢のまま固まり，
`reset_pose` を送ると戻る (実時間)
:::

```{important}
実機は**サーボ OFF で起動する** (シミュレータは ON で起動する)．同じプログラムを両方で動かせるよう，
プログラムの最初で `ri.servo_on()` を呼んでおくとよい．
関節に負荷がかかる姿勢を長く続けるとサーボが焼ける．危ないときは背面のスイッチでサーボの電源を切るか `ri.servo_off()` を呼ぶこと．
```

RViz の **Servo パネル**でも，関節ごとに ON/OFF できる (ALL ON / ALL OFF もある)．実機の launch は RViz を開き，
Gazebo は `rviz:=true` を付けると開く．「away from command」は送った姿勢と今の姿勢の差で，ON にしたとき腕がどれだけ動くかの目安になる．

:::{figure} fig/jedy-rviz-servo-panel.png
:align: center
:name: fig:jedy-rviz-servo-panel

RViz (`ros2 launch jedy_bringup jedy_gazebo.launch.py rviz:=true`)．左が Servo パネルと Diagnostics パネル
:::

ケーブルが抜けかけている，電源が入っていないなどで応答しないサーボがあると，RViz でその関節が赤く点滅し，
`connect()` の時点で「応答していないサーボがあります」というエラーが出る (`ri.missing_servos()` で `{関節名: 理由}` が得られる)．
プログラムの誤りではないので，コードを直す前にケーブルと電源を確認しよう (抜き差しは TA を呼んでから)．

### 関節角度で動かす (順運動学)

```{code-block} python
robot.reset_pose()                              # モデルを初期姿勢に
ri.angle_vector(robot.angle_vector(), 3.0)      # 3 秒かけて実機へ送る
ri.wait_interpolation()                         # 到着を待つ

robot.head_joint0.joint_angle(np.deg2rad(20))   # 関節を 1 つずつ動かす
robot.head_joint1.joint_angle(np.deg2rad(30))
ri.angle_vector(robot.angle_vector(), 3.0)
ri.wait_interpolation()

# うなずきを 3 回
for _ in range(3):
    for tilt in (0, 30):
        robot.head_joint1.joint_angle(np.deg2rad(tilt))
        ri.angle_vector(robot.angle_vector(), 1.0)
        ri.wait_interpolation()

print(np.rad2deg(robot.angle_vector() - ri.angle_vector()))   # 送った姿勢と実際の姿勢の差 [deg]
```

:::{figure} fig/jedy-fk-head.png
:align: center
:name: fig:jedy-fk-head

左: `robot.reset_pose()`．右: さらに `head_joint0` を 20 度，`head_joint1` を 30 度にしたモデル (PyrenderViewer で `viewer.redraw()` すると同じ絵になる)
:::

:::{figure} fig/jedy-gazebo-nod.gif
:align: center
:name: fig:jedy-gazebo-nod

上のプログラムを Gazebo で動かしたところ: 首を回して傾け，うなずきを 3 回 (実時間)
:::

:::{figure} fig/jedy-ki-move.gif
:align: center
:name: fig:jedy-ki-move

学科 PC で「台車を動かす」と「関節角度で動かす」のコードを 1 つのファイルにして実行したところ (左: Gazebo，右: 端末．1.5 倍速)
:::

### 手先の位置で動かす (逆運動学)

手先を目標の位置へ動かすための関節角度を求めるのが逆運動学 (IK) である．

```{code-block} python
robot.reset_pose()                                   # 初期姿勢によっては IK が解けない
target = robot.rarm.end_coords.copy_worldcoords()    # 今の右手の位置・姿勢
target.translate([0.03, 0.0, 0.03], wrt='world')     # 前に 3 cm, 上に 3 cm
robot.rarm.inverse_kinematics(target, rotation_axis=False)  # 位置だけ合わせる
ri.angle_vector(robot.angle_vector(), 3.0)
ri.wait_interpolation()
```

:::{figure} fig/jedy-ik-rarm.png
:align: center
:name: fig:jedy-ik-rarm

右から見たモデル．赤い点が目標 (今の右手の 3 cm 前・3 cm 上)．左: IK の前 (`reset_pose`)，右: `inverse_kinematics` の後 (手先が目標に着く)
:::

:::{figure} fig/jedy-gazebo-ik-rarm.gif
:align: center
:name: fig:jedy-gazebo-ik-rarm

Gazebo で右から見たところ: IK の解を 3 秒で送り，`reset_pose` に戻す (2 回，実時間)
:::

`ros2 run jedy_skrobot sample_ik` は同じことをして，実際に手先がどこに着いたかを表示する．
IK が解けないときは `inverse_kinematics` が `False` を返し，モデルの関節は動かない．
`ri.angle_vector()` で送る前に PyrenderViewer で解の姿勢を見ておくと，思わぬ姿勢 (腕が体に当たる，床に向かう) に気づける．

### キーボードで手先を動かす

```{code-block} console
$ ros2 run jedy_skrobot keyboard_ik
```

キーを押すたびに目標 (ビューワの赤い球) を 2 cm 動かし，逆運動学で手先を目標へ合わせ，解けたときだけその姿勢を 1 秒でロボットへ送る．
キーは起動した端末に打つ (ビューワの窓ではない)．`--model-only` を付けるとロボットにつながず，ビューワのモデルだけが動く．

| キー | 動き |
| --- | --- |
| `f` / `b` | 前 / 後ろ (x) |
| `h` / `l` | 左 / 右 (y) |
| `k` / `j` | 上 / 下 (z) |
| `s` | 動かす腕を切り替える (右腕 ↔ 左腕) |
| `e` | 頭が目標を見るモードに切り替える (もう一度 `e` で腕に戻る) |
| `r` | 初期姿勢 (`reset_pose`) に戻る |
| `q` | 終了 |

IK が解けないキーを押すと `IK failed` と出て，目標もロボットも動かない (別の向きのキーを押す)．

### 頭と腕を別々に動かす

Jedy の関節は 5 つのコントローラ (`head_controller`, `right_arm_controller`, `left_arm_controller`,
`right_gripper_controller`, `left_gripper_controller`) に分かれている．
`controller_type` を付けるとそのコントローラの関節だけを送り，`wait_interpolation(名前)` でそれだけを待つ．

```{code-block} python
# 頭 (2 秒) と右腕 (4 秒) を同時に．どちらも相手を待たず，相手の動きを止めない
ri.angle_vector(robot.angle_vector(), 4.0, controller_type='right_arm_controller')
ri.angle_vector(robot.angle_vector(), 2.0, controller_type='head_controller')
ri.wait_interpolation('head_controller')        # 2 秒後: 腕はまだ動いている
ri.wait_interpolation('right_arm_controller')   # 4 秒後
```

### <span style="color:green">チェックポイント: 腕と首</span>

```{exercise} 腕と首を動かす
:label: ex_arm_2026

1. 関節角度を指定して，首と両腕を動かしてみよう．`ri.angle_vector()` で読んだ姿勢と送った姿勢が一致していることを確かめよ．
2. 逆運動学で右手を目標の位置へ動かしてみよう．
3. `ros2 run jedy_skrobot keyboard_ik` で右手を前・上へ動かし，`s` で左手に切り替えて動かし，`r` で初期姿勢に戻してみよう．
```

## カメラで見る

頭のカメラの画像は `ri.image()` (RGB の numpy 配列)，距離画像は `ri.depth_image()` (メートル) で読める．

```{code-block} python
rgb, stamp = ri.image()        # (480, 640, 3) uint8, RGB
depth, stamp = ri.depth_image()   # (480, 640) float32 [m]．測れない画素は nan
K, D = ri.camera_info()        # カメラ行列 3x3 と歪み係数
print(depth[240, 320])         # 画像の真ん中までの距離 [m]
```

:::{figure} fig/jedy-gazebo-head-camera-box.png
:align: center
:name: fig:jedy-gazebo-head-camera-box

Gazebo で起動した直後の頭のカメラの画像 (左に赤い箱)
:::

画像は numpy の配列なので，matplotlib で表示したり，OpenCV や自分の Python のプログラムにそのまま渡したりできる．

```{code-block} python
import matplotlib.pyplot as plt

rgb, stamp = ri.image()
depth, stamp = ri.depth_image()
fig, (ax1, ax2) = plt.subplots(1, 2)
ax1.imshow(rgb)
ax2.imshow(depth)
plt.show()            # ウィンドウを閉じると次へ進む
```

`ros2 run jedy_skrobot show_camera` は，今のカメラの画像と深度を表示し続けるプログラムである
(`--once --save camera.png` で 1 枚だけファイルに保存)．中の `recognize(rgb, depth)` が画像から物を探して枠を返し，
それが画像に描かれる．今は赤い所を探しているが，**ここを書き換えれば自分の認識器を試せる**
(`jedy/jedy_skrobot/scripts/enshu/show_camera.py` をコピーして使うとよい)．

```{code-block} console
$ mkdir -p ~/enshu
$ cp ~/ros2_ws/src/robot-programming/jedy/jedy_skrobot/scripts/enshu/show_camera.py ~/enshu/my_camera.py
$ $(ros2 pkg prefix jedy_skrobot)/share/jedy_skrobot/venv/bin/python ~/enshu/my_camera.py
```

色で物を切り出すのは OpenCV でできる．

```{code-block} python
import cv2

rgb, stamp = ri.image()
hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
mask = cv2.inRange(hsv, np.array([0, 120, 70]), np.array([10, 255, 255]))   # 赤
ys, xs = np.nonzero(mask)
if len(xs) > 0:
    cx, cy = int(xs.mean()), int(ys.mean())          # 赤い領域の中心 [pixel]
    depth, _ = ri.depth_image()
    print('center x', cx, 'distance [m]', depth[cy, cx])
```

`ros2 run jedy_skrobot track_box` は，これを使って赤い箱の方へ台車を旋回し続けるサンプルである (Gazebo は `box:=true` で起動しておく)．
中身を読んでみよう (`jedy/jedy_skrobot/scripts/enshu/track_box.py`)．

```{note}
D405 (実機の頭のカメラ) の深度は 0.07〜0.5 m くらいしか測れない．それより遠い物までの距離は nan になる．
```

## カメラで認識する

カメラの画像から「何が写っているか」「どの手の形をしているか」を `ri` に聞ける．

- **物体**: 言葉で指定した物を探す ([YOLOE](https://docs.ultralytics.com/models/yoloe/)．「ペットボトル」「コップ」のように，決まった種類に限らず言葉で指定できる)
- **ジェスチャ**: グー・チョキ・パー，指さし，いいね など ([MediaPipe Gesture Recognizer](https://ai.google.dev/edge/mediapipe/solutions/vision/gesture_recognizer))

認識は `jedy_vision` の 2 つのノードが行い，`ri` はそのノードに問い合わせるだけである (重いモデルは自分のプログラムには入らない)．
ロボットを起動したうえで，別のターミナルで起動しておく．

```{code-block} console
$ ros2 launch jedy_vision recognition.launch.py
```

```{code-block} python
# 物体: 言葉で指定して探す
print(ri.sees('ペットボトル'))                     # 写っていれば True
for d in ri.what_do_you_see(['人', 'コップ']):     # 見つかった物 (確からしい順)
    print(d.label, d.score, d.box)                 # 単語, 確からしさ (0〜1), 画像の中の枠 [x1, y1, x2, y2] (pixel)

if ri.wait_until_sees('人', timeout=30.0):         # 写るまで (最大 30 秒) 待つ
    print('人がいた')

# ジェスチャ
print(ri.gesture())    # 'グー' 'チョキ' 'パー' '指さし' 'いいね' 'よくない' 'アイラブユー' か None
print(ri.janken())     # 'グー' 'チョキ' 'パー' か None
hand = ri.wait_until_janken(timeout=10.0)          # じゃんけんの手が出るまで待つ
```

`d.box` は `ri.image()` の画像の座標なので，枠の中心が画像の左右どちらにあるかで首や台車を回したりできる．

**物の 3D の位置**も分かる．認識のノードが距離画像から枠の中の距離を測り，`d.position` (カメラの座標系) に入れている．
`ri.detection_position(d)` でロボットの座標系 (base_link: 前が x，左が y，上が z) の位置 [m] になる．

```{code-block} python
for d in ri.what_do_you_see(['赤いもの']):
    p = ri.detection_position(d)        # [x, y, z] (m)．距離が測れなかったときは None
    print(d.label, d.score, p)
```

距離が測れるのはカメラの測れる範囲だけである (実機の D405 は 0.07〜0.5 m．遠い物は `None` になる)．
首を動かした後に聞くと，その時の首の角度で計算するので，聞いてから首を動かすこと．

:::{figure} fig/jedy-gazebo-detection-red-object.png
:align: center
:name: fig:jedy-gazebo-detection-red-object

Gazebo (`box:=true`) で `ri.what_do_you_see(['赤いもの'])` を呼んだときの `/object_detection/image`．
枠の上に「単語 確からしさ 距離」．このとき `ri.detection_position(d)` は [1.125, 0.475, 0.135] (箱の手前の面は [1.125, 0.5, 0.134])
:::

:::{figure} fig/jedy-ki-see.gif
:align: center
:name: fig:jedy-ki-see

学科 PC で認識のノードを起動し，上のコードで赤いものの位置を聞いたところ．中央は `/object_detection/image` (2 倍速)
:::

使える日本語の単語は `jedy_vision` の `config/vocabulary.yaml` にある (英語に直して検出器に渡している)．
**表に無い日本語の単語はエラーになる**ので，表に足して `colcon build` し直すか，英語で指定する (英語はそのまま使える)．
英語は短い一般的な名詞ほどよく当たる (`bottle` は見つかり，`plastic bottle` は見つからなかった)．

```{note}
**Gazebo の赤い箱は「箱」では見つからない．** 模様の無い単色の CG の箱は，検出器には箱に見えない
(実測の確からしさ: box 0.02, block 0.15, **red object 0.75**)．`ri.sees('赤いもの')` で探す．
```

```{note}
- 認識のノードが起動していないと，`ri.what_do_you_see()` などは「認識のノードを起動してください」というエラーになる (何も見えない，とはならない)．
- ジェスチャは同じ形が数枚続いて初めて答える (手を動かしている途中の形を拾わないため)．手を止めて 0.5 秒ほど見せる．
- 物体を探すのは 1 回 0.6 秒ほど，単語を変えた最初の 1 回は 1 秒ほど (Gazebo と同じノート PC の CPU で計測)．
- 枠を描いた画像が `/object_detection/image` に出る．`rviz:=true` で開く RViz では右側の「detections (image)」に映る (`rqt_image_view` でも見られる)．
  3D の位置は RViz の「detections (3D)」に緑の球と名前で出る (`/object_detection/markers`)．
- ロボットの代わりに **PC の Web カメラ**で試すこともできる:
  `ros2 run usb_cam usb_cam_node_exe` で `/image_raw` に画像を出し，`ros2 launch jedy_vision recognition.launch.py image_topic:=/image_raw`．
```

### AprilTag で位置を知る・近づく

AprilTag は白黒の正方形のマーカで，番号 (id) と，カメラからの位置・向きが 1 枚の画像から分かる．
Gazebo では `tags:=true` で，64 mm のタグ (36h11，id 0〜9) を貼った柱を 10 本，ロボットのまわり 1.2〜1.7 m に置く
(各タグはロボットの方を向いている．`demo:=true` とは一緒に使わない)．

```{code-block} console
$ ros2 launch jedy_bringup jedy_gazebo.launch.py tags:=true
$ ros2 run jedy_vision tag_pose --size 0.064 --tilt 10            # 見えているタグの位置 (base_link) を表示し，RViz にも出す
$ ros2 run jedy_vision approach_tag --id 2 --size 0.064 --tilt 10 --standoff 0.3   # タグ 2 の正面 0.3 m まで走る
```

`--size` はタグの黒い正方形の一辺 [m]，`--tilt` は首を下に向ける角度 [deg]．
`approach_tag` はタグが見えない間はその場で回って探し，見つけたら `ri.go_velocity()` でタグの正面へ走って止まる．
中身は `jedy/jedy_vision/scripts/approach_tag.py` (と `tag_pose.py`) にあるので，コピーして自分のプログラムに使える．

:::{figure} fig/jedy-gazebo-approach-tag.gif
:align: center
:name: fig:jedy-gazebo-approach-tag

`approach_tag --id 2` (Gazebo，実時間．右上は頭のカメラ)．左を向いてタグ 2 を見つけ，正面 0.3 m で止まる
(Gazebo の真値で，止まった位置の誤差は 7 mm)
:::

:::{figure} fig/jedy-ki-tag.gif
:align: center
:name: fig:jedy-ki-tag

学科 PC で上の `approach_tag` を実行したところ (1.5 倍速)
:::

### 人を見つける (体・顔・手)

Gazebo では `person:=true` で人のモデルを置ける (`person_x`, `person_y` で場所 [m])．
`detect_people` は頭のカメラの画像 1 枚から，人の枠 (YOLO) と距離，体の骨格 (MediaPipe Pose，33 点)，
顔の枠と 478 点のメッシュ，手の 21 点を見つけて，描いた画像を保存する．

```{code-block} console
$ ros2 launch jedy_bringup jedy_gazebo.launch.py person:=true person_x:=1.6 person_y:=0.0
$ ros2 run jedy_vision detect_people --pose --face --hands --tilt -25 --save people.png
```

頭のカメラは床から 0.4 m ほどなので，立っている人の顔を見るには首を上に向ける (`--tilt -25`: 負が上)．
顔は画像の中では小さい (1.5〜2.5 m で 30 px ほど) ので，骨格から頭のまわりを切り出して拡大してから探している．
プログラムからは `jedy_vision` の `PersonDetector`, `PoseEstimator`, `FaceEstimator`, `HandEstimator` を使う
(使い方は `jedy/jedy_vision/scripts/detect_people.py`)．

:::{figure} fig/jedy-gazebo-person-face-hands.png
:align: center
:name: fig:jedy-gazebo-person-face-hands

`detect_people --pose --face --hands --tilt -25` の結果 (Gazebo，1.6 m 先の人)．
青: 人の枠と距離，緑: 骨格，黄: 顔の枠とメッシュ，赤: 手 (体の横にある手は見つからないこともある)
:::

:::{figure} fig/jedy-ki-people.gif
:align: center
:name: fig:jedy-ki-people

学科 PC で上の `detect_people` を実行したところ．中央が保存した `people.png` (2 倍速)
:::

### <span style="color:green">チェックポイント: カメラ</span>

```{exercise} カメラを使った反応行動
:label: ex_camera_2026

カメラ (画像・深度・認識) を使ってロボットが反応するプログラムを書いてみよう．
**例:** `ri.wait_until_janken()` で相手の手を読んで首を振る．赤いもの (`ri.what_do_you_see(['赤いもの'])` の枠，または色の切り出し) が画像の真ん中に来るまで台車を旋回する．
```

## 声でしゃべる・聞く

ロボットに声でしゃべらせ ([VOICEVOX](https://voicevox.hiroshiba.jp/))，人の声を文字にする (音声認識)．
どちらも [jsk_3rdparty](https://github.com/jsk-ros-pkg/jsk_3rdparty) のパッケージで，スピーカーとマイクは**自分の PC のもの**を使う．
音声認識の既定のエンジン (Google) はインターネットにつながっている必要がある．

音声合成の VOICEVOX は環境構築ではビルドしていないので，初めて使う前に 1 回だけビルドする．
エンジン (約 1.3 GB) をダウンロードするので，回線によっては 30 分以上かかる．

```{code-block} console
$ cd ~/ros2_ws/
$ colcon build --symlink-install --packages-up-to voicevox
$ source ~/ros2_ws/install/setup.bash
```

ロボットとは別のターミナルで，それぞれ起動しておく．

```{code-block} console
$ ros2 launch voicevox voicevox_texttospeech.launch.xml
```

```{code-block} console
$ ros2 launch ros_speech_recognition speech_recognition.launch.xml language:=ja-JP launch_sound_play:=false \
    energy_threshold:=300.0 dynamic_energy_threshold:=false tts_action_names:="[robotsound_jp]"
```

```{code-block} python
ri.say('こんにちは。ジェディです。')          # しゃべり終わるまで待つ (1 文数秒)
ri.say('しゃべりながら動きます', wait=False)   # しゃべり始めたらすぐ戻る
ri.say('こんにちは', speaker='ずんだもん-ノーマル')   # 声を変える

text = ri.listen(timeout=10.0)                # 次に聞こえた文．10 秒聞こえなければ None
print('聞こえた文:', text)
if text is not None and '進' in text:
    ri.say('前に進みます')
    ri.go_velocity(0.1, 0.0, 0.0, duration=2.0)
```

<video width="100%" controls preload="metadata">
  <source src="fig/jedy-gazebo-voice.mp4" type="video/mp4">
  お使いのブラウザは動画タグをサポートしていません．
</video>

**動画 (音声あり)：** 上の例を 2 回くり返し，「回」が聞こえたらその場で回るようにしたプログラムを Gazebo で動かしたところ．
左が Gazebo の Jedy，右がプログラムのターミナル (`print` した認識結果)．
ロボットの「こんにちは。ジェディです。」のあとに人が「前に進んで」と言うと，「聞こえた文: 前に進んで」と表示され，「前に進みます」と言って前に進む．
次の「その場で回って」では回る．
(動画の人の声は VOICEVOX で作り，マイクの代わりに `/audio` トピックへ流したもの．認識は本物の Google 音声認識．)

音声認識の引数の意味:

- `energy_threshold:=300.0 dynamic_energy_threshold:=false`: これ以上の大きさの音を声とみなす値を 300 に固定する．
  固定しないと起動時の周りの音に合わせて自動で上がり (ノート PC の内蔵マイクでは 746 になった)，普通に話しても拾わなくなることがある．
  周りの雑音を声と間違えるなら上げ，声を拾わないなら下げる．
- `tts_action_names:="[robotsound_jp]"`: ロボットが `ri.say()` でしゃべっている間は認識を止める (自分の声を認識しないため)．

声の名前は `~/.ros/voicevox/speakers.json` にある．**名前を間違えるとエラーにならず，何もしゃべらない**ので注意．
コマンドでは `ros2 topic pub --once /robotsound_jp/text std_msgs/msg/String "{data: 'こんにちは'}"` でしゃべり，
`ros2 topic echo /speech_to_text` で認識結果を見られる．

```{note}
**何も認識されないとき**
- PC のマイクがミュートになっていないか: `wpctl status` の `Sources:` で，使うマイクに `MUTED` が付いていたら
  `wpctl set-mute @DEFAULT_AUDIO_SOURCE@ 0` と `wpctl set-volume @DEFAULT_AUDIO_SOURCE@ 0.8`
- `ros2 topic hz /audio` でマイクの音が来ているか (約 100 Hz)
- インターネットが使えないときは `engine:=Vosk` で PC の中だけで認識できる (精度は落ちる)
```

### <span style="color:green">チェックポイント: 声</span>

```{exercise} 声を使った行動
:label: ex_speech_2026

話しかけた言葉に応じてロボットが動き，何をするかを声で返事するプログラムを書いてみよう．
**例:** 「前」で前に進み，「回って」でその場で旋回し，「手を挙げて」で右腕を上げる．聞き取れなかったら「もう一度言ってください」と返す．
```

## 本日の課題

```{exercise} 道具を組み合わせる
:label: ex_day1_2026

台車・腕と首・カメラ・声のうち **3 つ以上**を使って，ロボットが人とやりとりするプログラムを書け．
**例:** 「じゃんけんしよう」と言われたら「じゃんけんぽん」としゃべって相手の手を読み，勝ち負けを言って，勝ったら両腕を上げる．

2 日目はこれをもとに自分の作品を作るので，**明日作りたいもの**を 1 行で書いておくこと．
```
